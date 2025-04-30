# ───── flightComputer/fc/mavsdk_fc.py ────────────────────────────────
"""
Async MAVSDK implementation that mimics the old DroneKit FlightController
API so the ground-station and handlers keep working unchanged.
"""
from __future__ import annotations
import asyncio, logging, json, time, math, pathlib, tempfile, subprocess
from typing import Optional, Tuple, Union
import threading

import mavsdk                            # pip install "mavsdk>=1.4"
from mavsdk.action import ActionError, ActionResult 
from mavsdk.telemetry import FlightMode   
from mavsdk.offboard import (OffboardError, VelocityNedYaw)
from .sitl import SitlManager 

_log = logging.getLogger("MavsdkFC")
logging.getLogger("mavsdk").setLevel(logging.DEBUG)

_retryable = {
    ActionResult.Result.FAILED,          # EKF/GPS not ready yet
    ActionResult.Result.COMMAND_DENIED,  # mode / state not right
    ActionResult.Result.BUSY,            # autopilot still doing something
}

# ──────────────────────── small helpers ──────────────────────────────
def _deg(rad: float) -> float:
    return (math.degrees(rad) + 360.0) % 360.0

def _ned_distance(a, b) -> float:
    """Euclidean distance in metres between two PositionNed structures."""
    dx = a.north_m - b.north_m
    dy = a.east_m  - b.east_m
    dz = a.down_m  - b.down_m
    return (dx*dx + dy*dy + dz*dz) ** 0.5


# ────────────────────────   main class   ─────────────────────────────
class FlightController:                            # same public name!
    # -------- life-cycle -------------------------------------------------
    def __init__(self,
                 connection_string: str = "udp://:14540",
                 baud_rate: int      = 57600,
                 simulation_mode: bool = False):
        self.log    = logging.getLogger("FlightController")
        self.addr   = connection_string
        self.baud   = baud_rate
        self.sim    = simulation_mode
        self._connected = False
        
        print("SIM flag =", simulation_mode)   # should print True
        
        # ← create one SitlManager for the lifetime of this FC
        self._sitl = SitlManager()

        self._drone      : mavsdk.System | None = None
        self._loop       : asyncio.AbstractEventLoop | None = None
        self._loop_th    : threading.Thread | None = None
        self._tele_th    : threading.Thread | None = None
        self._telem_suspended = asyncio.Event()
        self._telem_suspended.clear()
        
        # self._cache = {"batt": None, "pos": None, "att": None}
        # self._cache_ready = asyncio.Event()         # first full sample
        # self._cache_ready = None
        self._cache = {"batt": None, "pos": None, "att": None}
        self._cache_ready = None                    # will be bound to the worker loop
        self._ready_once   = False  # remember if we've already set it
        
        
        self._connect_lock  = threading.Lock()
        self._connected_evt = threading.Event()   # set after ✅ MAVSDK connected

    # ── internal: bring up asyncio loop + schedule connect ──────────
    def _ensure_loop(self):
        # fast path – already connected
        if self._connected_evt.is_set():
            return

        # only one thread may run the section below
        with self._connect_lock:
            if self._connected_evt.is_set():          # other thread won the race
                return

            if not self._loop:
                self._loop = asyncio.new_event_loop()
                self._loop_th = threading.Thread(
                    target=self._loop.run_forever, daemon=True)
                self._loop_th.start()
                
                # ------ create per-loop Event *inside* that very loop ------
                self._cache_ready = asyncio.Event(loop=self._loop)

            # schedule connect once
            if not hasattr(self, "_connect_task"):
                self._connect_task = asyncio.run_coroutine_threadsafe(
                    self._async_connect(), self._loop)

            # wait (max 30 s) until the async part signals “ready”
            if not self._connected_evt.wait(timeout=30):
                raise RuntimeError(
                    f"MAVSDK connection timeout on {self.addr}")


    async def _async_connect(self):
        # 1) launch SITL when in simulation_mode
        if self.sim:
            try:
                self.addr = self._sitl.start()       # udp://:14540
            except Exception as e:
                self.log.exception("Failed to launch SITL: %s", e)
                return

        # 2) connect MAVSDK
        self._drone = mavsdk.System()
        self.log.info("Connecting MAVSDK to %s …", self.addr)
        await self._drone.connect(system_address=self.addr)

        # 3) wait until MAVSDK has a link
        async for state in self._drone.core.connection_state():
            if not state.is_connected:
                continue

            self.log.info("✅ MAVSDK connected")

            if self.sim:
                try:
                    # a) turn off pre-arm checks (keeps old DroneKit behaviour)
                    # await self._drone.param.set_param_int("ARMING_CHECK", 0)

                    # b) make sure frame parameters exist
                    # await self._drone.param.set_param_int("FRAME_CLASS", 1)
                    # await self._drone.param.set_param_int("FRAME_TYPE",  1)
                    self.log.debug("SIM → FRAME_CLASS=1, FRAME_TYPE=1, ARMING_CHECK=0")
                except Exception as e:
                    self.log.warning("Could not init SIM params: %s", e)

            self._connected = True
            self._connected_evt.set()     # ← tell the other threads
            
            # start background listeners – they run until shutdown
            self._loop.create_task(self._cache_battery())
            self._loop.create_task(self._cache_position())
            self._loop.create_task(self._cache_attitude())
            break
        
    async def _cache_battery(self):
        async for msg in self._drone.telemetry.battery():
            # self._cache["batt"] = msg
            # self._cache_ready.set()
            
            self._cache["batt"] = msg
            self._signal_if_complete()

    async def _cache_position(self):
        async for msg in self._drone.telemetry.position():
            # self._cache["pos"] = msg
            # self._cache_ready.set()
            self.log.debug("POS", msg.latitude_deg, msg.longitude_deg, msg.relative_altitude_m)        #  ←  TEMP
            self._cache["pos"] = msg
            self._signal_if_complete()

    async def _cache_attitude(self):
        async for msg in self._drone.telemetry.attitude_euler():
            # self._cache["att"] = msg
            # self._cache_ready.set()        
            self._cache["att"] = msg
            self._signal_if_complete()
            
            
    def _signal_if_complete(self):
        """
        Fire the `_cache_ready` event **once** after we've received
        at least one message of *each* stream.  Called by the three
        cache-tasks above (they all run on `self._loop`).
        """
        self.log.debug("signal?", self._cache)
        if (not self._ready_once
                and all(self._cache.values())):   # all three non-None?
            self._cache_ready.set()
            self._ready_once = True            
        
    def connect(self, timeout: float = 10.0):
        self._ensure_loop()
        t0 = time.time()
        while not self._connected:
            if time.time() - t0 > timeout:
                raise RuntimeError(f"MAVSDK connection timeout on {self.addr}")
            time.sleep(0.2)

    def shutdown(self):
        if not self._loop:
            return
        # orderly shutdown of offboard
        asyncio.run_coroutine_threadsafe(self._async_shutdown(), self._loop)
        # Shutdown SITL
        self._sitl.stop()
        # tear down loop
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._loop_th.join(timeout=1.0)

    async def _async_shutdown(self):
        try:
            await self._drone.offboard.stop()
        except Exception:
            pass

    # -------- basic actions -------------------------------------------
    def arm_drone(self):            # sync façade
        return self._run(self._arm_async())

    async def _arm_async(self, timeout: float = 20.0):
        try:
            await self._drone.action.arm()
            return "Drone is ARMED.\nEND_RESPONSE"

        except mavsdk.action.ActionError as err:
            # err._result.result is an enum, err._result.result_str is the label
            reason = err._result.result_str or "Unknown"
            return f"ERR ARM failed: {reason}\nEND_RESPONSE"

    def takeoff_drone(self, alt=2.0):
        return self._run(self._takeoff_async(alt))
    
    async def _wait_until_position_ok(self, timeout=90.0):
        t0 = time.time()
        async for h in self._drone.telemetry.health():
            if (h.is_global_position_ok
                    and h.is_local_position_ok
                    and (getattr(h, "is_home_position_ok", True))):
                return
            if time.time() - t0 > timeout:
                raise RuntimeError("EKF never became ready (no position/home)")


    async def _ensure_guided(self, timeout: float = 5.0) -> None:
        """
        Put Copter into GUIDED and wait until the autopilot confirms
        the mode change (or raise after *timeout* seconds).
        """
        await self._drone.action.set_flight_mode(FlightMode.GUIDED)

        t0 = time.time()
        async for mode in self._drone.telemetry.flight_mode():
            if mode == FlightMode.GUIDED:
                return                              # ✓ success
            if time.time() - t0 > timeout:
                raise RuntimeError("Could not enter GUIDED mode")    

    async def _takeoff_async(self, alt: float = 2.0):
        # 0) ask (politely) for the altitude we want; ignore time-outs
        try:
            await self._drone.param.set_param_float("TKOFF_ALT", alt)
        except Exception:
            pass

        # 1) keep sending TAKEOFF until the autopilot accepts it
        while True:
            try:
                await self._drone.action.takeoff()           # returns on SUCCESS
                break                                        # ✓ accepted
            except ActionError as err:
                if err._result.result in _retryable:         # << change here
                    await asyncio.sleep(1.0)                 # wait → retry
                    continue
                raise                                         # anything else

        # 2) wait until we actually climb
        async for pos in self._drone.telemetry.position():
            if pos.relative_altitude_m >= 0.95 * alt:
                break

        return f"Reached {alt:.1f} m\nEND_RESPONSE"


    def land_drone(self):
        return self._run(self._land_async())

    async def _land_async(self):
        await self._drone.action.land()
        async for state in self._drone.telemetry.landed_state():
            if state == mavsdk.telemetry.LandedState.ON_GROUND:
                break
        return "Drone landed.\nEND_RESPONSE"

    # -------- movement -------------------------------------------------
    def move(self,
             direction: Union[str, Tuple[float, float, float]] = "FWD",
             distance: float = 2.0,
             velocity: float = 1.0,
             block: bool = True,
             duration: float | None = None):
        return self._run(
            self._move_async(direction, distance, velocity, duration),
            block=block
        )

    async def _move_async(self, direction, distance, velocity, duration):
        aliases = {
            "FWD":  ( 1, 0, 0), "BACK": (-1, 0, 0),
            "RIGHT":( 0, 1, 0), "LEFT": ( 0,-1, 0),
            "UP":   ( 0, 0,-1), "DOWN": ( 0, 0, 1),
        }
        vec = aliases.get(direction.upper(), direction)
        if not (isinstance(vec, tuple) and len(vec) == 3):
            return f"Bad direction '{direction}'.\nEND_RESPONSE"

        mag = math.sqrt(sum(v*v for v in vec))
        unit = tuple(v/mag for v in vec)
        duration = duration or distance / velocity
        vx, vy, vz = (u*velocity for u in unit)

        await self._start_offboard_once()
        await self._drone.offboard.set_velocity_ned(
            VelocityNedYaw(vx, vy, -vz, 0))
        await asyncio.sleep(duration)
        await self._drone.offboard.set_velocity_ned(
            VelocityNedYaw(0, 0, 0, 0))
        return f"MOVE OK {direction} {distance} m\nEND_RESPONSE"

    # -------- yaw ------------------------------------------------------
    def yaw_drone(self, arg: str, angle=15):
        return self._run(self._yaw_async(arg, angle))

    async def _yaw_async(self, arg, angle):
        if arg.upper() in ("LEFT", "YAWLEFT"):
            yaw_rate = -abs(angle)
        elif arg.upper() in ("RIGHT", "YAWRIGHT"):
            yaw_rate =  abs(angle)
        else:                                    # numeric value
            yaw_rate = float(arg)

        await self._start_offboard_once()
        await self._drone.offboard.set_velocity_ned(
            VelocityNedYaw(0, 0, 0, yaw_rate))
        await asyncio.sleep(2)
        await self._drone.offboard.set_velocity_ned(
            VelocityNedYaw(0, 0, 0, 0))
        return f"YAW Δ≈{yaw_rate}°\nEND_RESPONSE"

    # -------- telemetry ------------------------------------------------
    def start_telemetry(self, radio, period=0.2):
        if self._tele_th:
            return

        def _worker():
            while True:
                if not self._telem_suspended.is_set():
                    blob = "T:" + json.dumps(self.get_state_dict()) + "\n"
                    radio.send(blob)
                time.sleep(period)

        self._tele_th = threading.Thread(target=_worker, daemon=True)
        self._tele_th.start()

    def stop_telemetry(self):
        self._telem_suspended.set()

    # -------- state snap-shot -----------------------------------------
    def get_state_dict(self):
        return self._run(self._state_async())

    async def _state_async(self):
        """
        Return the most recent battery / position / attitude.
        Wait ⩽5 s for the **first** full sample, afterwards never block.
        """
        # Wait until all three keys are non-None (first message of each stream)
        try:
            await asyncio.wait_for(self._cache_ready.wait(), 5.0)
        except asyncio.TimeoutError:
            pass    # continue – you may still have partial data

        batt = self._cache["batt"]
        pos  = self._cache["pos"]
        att  = self._cache["att"]

        return {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "battery":  {"voltage": batt.voltage_v              if batt else 0.0,
                        "remaining": batt.remaining_percent    if batt else 0.0},
            "location": {"lat": pos.latitude_deg                if pos else 0.0,
                        "lon": pos.longitude_deg               if pos else 0.0,
                        "rel_alt": pos.relative_altitude_m     if pos else 0.0},
            "attitude": {"roll": att.roll_deg                   if att else 0.0,
                        "pitch": att.pitch_deg                 if att else 0.0,
                        "yaw": att.yaw_deg                     if att else 0.0},
        }

        
    def get_state(self):
        blob = json.dumps(self.get_state_dict(), indent=2)
        return blob + "\nEND_RESPONSE"

    # -------- internal -------------------------------------------------
    async def _start_offboard_once(self):
        try:
            await self._drone.offboard.start()
        except OffboardError as err:
            if err._result.result != mavsdk.offboard.OffboardResult.Result.BUSY:
                raise

    def _run(self, coro, block: bool = True):
        """Ensure the asyncio loop is up, connect on-demand, then run *coro*."""
        self._ensure_loop()                          # starts loop + schedules _async_connect

        # ─── NEW: if nothing is connected yet, block until we are ────
        if not self._connected:
            # give SITL and MAVSDK a bit longer on the first call
            self.connect(timeout=20.0)
        # ──────────────────────────────────────────────────────────────

        fut = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return fut.result() if block else None

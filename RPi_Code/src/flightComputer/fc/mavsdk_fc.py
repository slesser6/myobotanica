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
from mavsdk.offboard import (OffboardError, VelocityNedYaw)
from .sitl import SitlManager 

_log = logging.getLogger("MavsdkFC")
logging.getLogger("mavsdk").setLevel(logging.DEBUG)

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

    # ── internal: bring up asyncio loop + schedule connect ──────────
    def _ensure_loop(self):
        if self._loop:
            return
        self._loop = asyncio.new_event_loop()
        self._loop_th = threading.Thread(
            target=self._loop.run_forever, daemon=True
        )
        self._loop_th.start()
        asyncio.run_coroutine_threadsafe(self._async_connect(), self._loop)

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
            break
        
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
    
    async def _wait_until_position_ok(self, timeout: float = 60.0) -> None:
        """
        Wait until the EKF reports a *usable* global position.
        In SITL we don't require the home-position flag.
        """
        t0 = time.time()
        async for health in self._drone.telemetry.health():
            if health.is_global_position_ok:            # ← ONLY this flag
                return
            if time.time() - t0 > timeout:
                raise RuntimeError(
                    f"GPS/EKF still not ready after {timeout} s"
                )


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
        # 0) wait until the EKF is happy
        await self._wait_until_position_ok(timeout=60.0)

        # 1) try to write TKOFF_ALT (ignore failures)
        try:
            await self._drone.param.set_param_float("TKOFF_ALT", alt)
        except Exception:
            pass

        # 2) tell ArduCopter to take off (this switches to GUIDED automatically)
        await self._drone.action.takeoff()

        # 3) wait until we have climbed ~95 % of the target altitude
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
        """Return battery/position/attitude; never block >2 s on silent streams."""
        async def first(gen, timeout=2.0):
            try:
                return await asyncio.wait_for(gen.__anext__(), timeout)
            except (asyncio.TimeoutError, StopAsyncIteration):
                return None                                   # ← default = None

        batt, pos, att = await asyncio.gather(
            first(self._drone.telemetry.battery()),
            first(self._drone.telemetry.position()),
            first(self._drone.telemetry.attitude_euler()),
        )

        return {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "battery":  {"voltage": batt.voltage_v        if batt else 0.0,
                         "remaining": batt.remaining_percent if batt else 0.0},
            "location": {"lat": pos.latitude_deg          if pos else 0.0,
                         "lon": pos.longitude_deg         if pos else 0.0,
                         "rel_alt": pos.relative_altitude_m if pos else 0.0},
            "attitude": {"roll": att.roll_deg             if att else 0.0,
                         "pitch": att.pitch_deg           if att else 0.0,
                         "yaw": att.yaw_deg               if att else 0.0},
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

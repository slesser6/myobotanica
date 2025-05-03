# ───── flightComputer/fc/mavsdk_fc.py ────────────────────────────────
"""
Async MAVSDK implementation that mimics the old DroneKit FlightController
API so the ground-station and handlers keep working unchanged.
"""

# ──────────────────────── Imports ────────────────────────────────────
# Standard library
import asyncio
import json
import logging
import math
import pathlib
import subprocess
import tempfile
import threading
import time
from typing import Optional, Tuple, Union
from collections import namedtuple

# Third-party
import grpc
import mavsdk                              # pip install "mavsdk>=1.4"
from mavsdk.action import ActionError, ActionResult
from mavsdk.telemetry import FlightMode
from mavsdk.offboard import OffboardError, VelocityNedYaw, PositionNedYaw      

# ── FlightController helpers ────────────────────────────────────────
import mavsdk.telemetry as tlm


# Local
from .sitl import SitlManager


# from mavsdk.mavlink_passthrough import MavlinkPassthrough
# mp = MavlinkPassthrough(self._drone)

# Relevant ArduPilot custom-mode numbers
_AP_MODE = {
    "LOITER": 5,
    "POSCTL": 16,      # a.k.a. POSHOLD on ArduPilot
    "HOLD":   17,      # “BRAKE” – immediate stop/hold
}

# ──────────────────────── Logger & Constants ─────────────────────────
_log = logging.getLogger("MavsdkFC")
logging.getLogger("mavsdk").setLevel(logging.ERROR)

# Which action errors should be retried automatically
_RETRYABLE = {
    ActionResult.Result.FAILED,          # EKF/GPS not ready yet
    ActionResult.Result.COMMAND_DENIED,  # mode/state not right
    ActionResult.Result.BUSY,            # autopilot still doing something
}

# ──────────────────────── Helper Functions ───────────────────────────
def _deg(rad: float) -> float:
    """Convert radians to [0..360) degrees."""
    return (math.degrees(rad) + 360.0) % 360.0

def _ned_distance(a, b) -> float:
    """
    Euclidean distance (m) between two PositionNed structures.
    """
    dx = a.north_m - b.north_m
    dy = a.east_m  - b.east_m
    dz = a.down_m  - b.down_m
    return math.sqrt(dx*dx + dy*dy + dz*dz)

def wrap180(deg):
    return ((deg + 180) % 360) - 180

# ──────────────────────── FlightController ───────────────────────────
class FlightController:
    """
    High-level façade over MAVSDK System, plus optional SITL launch.
    """

    # ─────────────── Initialization & Teardown ────────────────────────
    def __init__(
        self,
        connection_string: str = "udp://:14540",
        baud_rate: int = 57600,
        simulation_mode: bool = False,
        indoor_mode : bool = False
    ):
        """
        Initialize logging, SITL manager (if sim), and prepare asyncio loop.
        """
        self.log    = logging.getLogger("FlightController")
        self.addr   = connection_string
        self.baud   = baud_rate
        self.sim    = simulation_mode
        self.indoor = indoor_mode 
        print(f"SIM flag = {simulation_mode}, Indoor Mode Flag: {indoor_mode}")

        # SITL launcher (only used if sim=True)
        self._sitl = SitlManager()

        # MAVSDK system and event loop placeholders
        self._drone      : mavsdk.System | None = None
        self._loop       : asyncio.AbstractEventLoop | None = None
        self._loop_th    : threading.Thread | None = None

        # Telemetry thread
        self._tele_th    : threading.Thread | None = None
        self._telem_suspended = asyncio.Event()
        self._telem_suspended.clear()

        # Telemetry cache and readiness event
        self._cache = {"batt": None, "pos": None, "att": None, "vel": None}
        self._cache_ready = None
        self._ready_once  = False
        
        self._params_applied = False
        
        # Background task to hold position 
        self._hold_task: Optional[asyncio.Task] = None

        # Connection synchronization
        self._connected      = False
        self._connect_lock   = threading.Lock()
        self._connected_evt  = threading.Event()

    def connect(self, timeout: float = 10.0):
        """
        Ensure the asyncio loop is running and wait (up to timeout)
        for MAVSDK to be connected.
        """
        self._ensure_loop()
        t0 = time.time()
        while not self._connected:
            if time.time() - t0 > timeout:
                raise RuntimeError(f"MAVSDK connection timeout on {self.addr}")
            time.sleep(0.2)

    def shutdown(self):
        """
        Stop offboard, kill SITL (if any), and tear down the asyncio loop.
        """
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
        """Internal: stop offboard mode gracefully."""
        try:
            await self._drone.offboard.stop()
        except Exception:
            pass

    # ─────────────── Connection Setup ─────────────────────────────────
    def _ensure_loop(self):
        """
        Start the asyncio loop+thread if not already running,
        and schedule _async_connect() exactly once.
        """
        if self._connected_evt.is_set():
            return

        with self._connect_lock:
            if self._connected_evt.is_set():
                return

            if not self._loop:
                self._loop = asyncio.new_event_loop()
                self._loop_th = threading.Thread(
                    target=self._loop.run_forever, daemon=True
                )
                self._loop_th.start()
                self._cache_ready = asyncio.Event(loop=self._loop)

            if not hasattr(self, "_connect_task"):
                self._connect_task = asyncio.run_coroutine_threadsafe(
                    self._async_connect(), self._loop
                )

            if not self._connected_evt.wait(timeout=30):
                raise RuntimeError(f"MAVSDK connection timeout on {self.addr}")

    async def _async_connect(self):
        """
        Coroutine that:
        1) Launches SITL if in sim mode,
        2) Connects MAVSDK System to self.addr,
        3) Marks connected (unblocks .connect()),
        4) Applies only SIM tweaks (if sim),
        5) Subscribes to telemetry streams,
        6) Spawns cache tasks.
        """
        try:
            if self.sim:
                try:
                    self.addr = self._sitl.start()
                except Exception as e:
                    self.log.exception("Failed to launch SITL: %s", e)
                    return

            self._drone = mavsdk.System()
            self.log.info("Connecting MAVSDK to %s …", self.addr)
            await self._drone.connect(system_address=self.addr)

            async for state in self._drone.core.connection_state():
                if not state.is_connected:
                    continue

                self.log.info("✅ MAVSDK connected")
                await asyncio.sleep(1.0)

                try:
                    batt_monitor = await self._drone.param.get_param_int("BATT_MONITOR")
                    arming_check = await self._drone.param.get_param_int("ARMING_CHECK")

                    if self.sim:
                        sim_batt = await self._drone.param.get_param_float("SIM_BATT_VOLTAGE")
                        self.log.info(
                            f"Param check → BATT_MONITOR={batt_monitor}, SIM_BATT_VOLTAGE={sim_batt:.2f}, ARMING_CHECK={arming_check}"
                        )
                    else:
                        self.log.info(
                            f"Param check → BATT_MONITOR={batt_monitor}, ARMING_CHECK={arming_check}"
                        )
                except Exception as e:
                    self.log.warning("Failed to fetch param(s): %s", e)

                try:
                    version = await asyncio.wait_for(self._drone.info.get_version(), timeout=3.0)
                    self.log.info(
                        "🎛️  Flight controller firmware: %d.%d.%d (git %s)",
                        version.flight_sw_major,
                        version.flight_sw_minor,
                        version.flight_sw_patch,
                        version.flight_sw_git_hash,
                    )

                    identification = await self._drone.info.get_identification()
                    self.log.info(
                        "🔧 Flight controller hardware UID: %s",
                        identification.hardware_uid,
                    )
                except Exception as e:
                    self.log.warning("Error fetching firmware/hardware info: %s", e)

                self._connected = True
                self._connected_evt.set()

                if self.sim:
                    try:
                        await self._drone.param.set_param_int("ARMING_CHECK", 0)
                        self.log.debug("SIM params applied")
                    except Exception as e:
                        self.log.warning("SIM param init failed: %s", e)

                if not self.sim and getattr(self, "indoor", False):
                    self.log.info("🌑 Indoor mode: skipping onboard param writes (offline‐configured)")

                await self._drone.telemetry.set_rate_battery(1.0)
                await self._drone.telemetry.set_rate_position(5.0)
                await self._drone.telemetry.set_rate_attitude_euler(10.0)
                try:
                    await self._drone.telemetry.set_rate_position_velocity_ned(5.0)
                except AttributeError:
                    self.log.info("vehicle doesn’t support position_velocity_ned stream")

                self._loop.create_task(self._cache_battery())
                self._loop.create_task(self._cache_position())
                self._loop.create_task(self._cache_attitude())
                self._loop.create_task(self._cache_velocity_ned())

                # try:
                #     await self._drone.telemetry.set_rate_optical_flow(10.0)
                #     self._loop.create_task(self._cache_optical_flow())
                # except Exception:
                #     self.log.debug("Optical-flow telemetry not enabled (unsupported)")
                                        
                if self.sim:  # Only subscribe to optical flow in simulation
                    try:
                        await self._drone.telemetry.set_rate_optical_flow(10.0)
                        self._loop.create_task(self._cache_optical_flow())
                    except Exception:
                        self.log.debug("Optical-flow telemetry not enabled (unsupported)")
                    
                    

                break

        except Exception as e:
            self.log.exception("Uncaught error during async_connect")
            return

    # ─────────────── Telemetry Caching ────────────────────────────────
    async def _cache_battery(self):
        """Continuously cache the latest battery telemetry."""
        while True:
            try:
                async for msg in self._drone.telemetry.battery():
                    self._cache["batt"] = msg
                    self._signal_if_complete()
            except grpc.aio.AioRpcError:
                self.log.warning("BATTERY STREAM dropped - reconnecting …")
                await asyncio.sleep(1.0)
                

    async def _cache_optical_flow(self):
        """Continuously cache the latest optical flow sensor data."""
        while True:
            try:
                async for f in self._drone.telemetry.optical_flow():
                    self._cache["flow"] = f
            except:                
                self.log.warning("OPTICAL FLOW stream dropped - reconnecting …")
                await asyncio.sleep(1.0)
                
    async def _cache_position(self):
        """Continuously cache the latest global position telemetry."""
        while True:
            try:
                async for msg in self._drone.telemetry.position():
                    self._cache["pos"] = msg
                    # self.log.debug("🔄 got GLOBAL POS: %.6f, %.6f, alt=%.2f", 
                    #             msg.latitude_deg, msg.longitude_deg, msg.relative_altitude_m)
                    self._signal_if_complete()
            except grpc.aio.AioRpcError:
                self.log.warning("position stream dropped - reconnecting …")
                await asyncio.sleep(1.0)

    async def _cache_attitude(self):
        """Continuously cache the latest attitude (Euler angles)."""
        while True:
            try:
                async for msg in self._drone.telemetry.attitude_euler():
                    self._cache["att"] = msg
                    self._signal_if_complete()
            except grpc.aio.AioRpcError:
                self.log.warning("ATTITUDE STREAM dropped - reconnecting …")
                await asyncio.sleep(1.0)

    async def _cache_velocity_ned(self):
        """Continuously cache the latest NED position from position_velocity_ned()."""
        while True:
            try:
                async for pv in self._drone.telemetry.position_velocity_ned():
                    self._cache["vel"] = pv.position
            except grpc.aio.AioRpcError:
                self.log.warning("velocity_ned stream dropped; reconnecting …")
                await asyncio.sleep(1.0)

    def _signal_if_complete(self):
        if self._ready_once:
            return

        # Indoor mode skips GPS (position), uses NED velocity instead
        required_keys = {"batt", "att", "vel"}
        if not self.indoor:
            required_keys.add("pos")

        missing = [k for k in required_keys if self._cache.get(k) is None]
        if not missing:
            self.log.info(f"✅ All required telemetry received: {required_keys}")
            self._cache_ready.set()
            self._ready_once = True
        else:
            self.log.debug(f"Waiting for telemetry: {missing}")


    # ─────────────── Action Helpers ────────────────────────────────────
    def arm_drone(self):
        """Synchronously arm the drone (blocks until complete)."""
        return self._run(self._arm_async())

    async def _arm_async(self, timeout: float = 20.0):
        """
        Coroutine to arm the drone, returning success/failure string.
        """
        try:
            await self._drone.action.arm()
            return "Drone is ARMED.\nEND_RESPONSE"
        except ActionError as err:
            reason = err._result.result_str or "Unknown"
            return f"ERR ARM failed: {reason}\nEND_RESPONSE"

    def takeoff_drone(self, alt: float = 1.5):
        """Synchronously take off to a given altitude."""
        return self._run(self._takeoff_async(alt))
    
    # -------------------------------------------------------------
    async def _takeoff_async(self, alt: float = 1.5, timeout: float = 30.0):
        """
        Arms, switches to GUIDED, takes off to `alt` metres AGL (NED frame),
        then starts a background feeder that keeps the hover pose.
        """

        # ---- guard: already airborne? -----------------------------------
        try:
            if await self._is_airborne():
                self.log.warning("Take-off request ignored – already airborne.")
                return "ERR Take-off blocked: already airborne\nEND_RESPONSE"
        except asyncio.TimeoutError:
            self.log.warning("Take-off guard: no landed_state telemetry; proceeding")

        # ---- set desired altitude once ----------------------------------
        try:
            alt = 1.5 # Hardcode to 1.5m for now 
            await self._drone.action.set_takeoff_altitude(alt)
            self.log.info("✅ Take-off altitude set to %.2f m", alt)
        except mavsdk.action.ActionError as err:
            return f"ERR set_takeoff_altitude: {err._result.result_str}\nEND_RESPONSE"

        # ---- try until GUIDED+ARM+TAKEOFF succeed or timeout ------------
        t0 = time.time()
        while True:
            try:
                # await self._drone.action.set_flight_mode(FlightMode.GUIDED)
                await self._drone.action.arm()
                await self._drone.action.takeoff()
                break                                   # success!
            except ActionError as err:
                if (err._result.result in _RETRYABLE
                        and time.time() - t0 < timeout):
                    self.log.debug("Take-off retry (%s)", err._result.result_str)
                    await asyncio.sleep(1.0)
                    continue
                return f"ERR Take-off failed: {err._result.result_str}\nEND_RESPONSE"

        # ---- wait until we reach 95 % of target altitude ----------------
        try:
            async for ned in self._drone.telemetry.position_velocity_ned():
                current_alt = -ned.position.down_m
                self.log.info("Climbing… alt=%.2f m", current_alt)
                if current_alt >= 0.95 * alt:
                    hover_pos = ned.position
                    break
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            return "ERR link lost during climb\nEND_RESPONSE"

        # ---- latest yaw to include in hover set-point -------------------
        if not self._cache["att"]:
            self._cache["att"] = await self._drone.telemetry.attitude_euler().__anext__()
        hover_yaw = self._cache["att"].yaw_deg

        hover_pt = PositionNedYaw(hover_pos.north_m,
                                hover_pos.east_m,
                                hover_pos.down_m,
                                hover_yaw)

        # ---- start / restart background hold feeder ---------------------
        if getattr(self, "_hold_task", None) and not self._hold_task.done():
            self._hold_task.cancel()

        self._hold_task = self._loop.create_task(
            self._feed_position_hold(hover_pt))
        self.log.info("🔒 Position hold feed started after take-off")

        # ---- final log --------------------------------------------------
        self.log.info("🎯 Hover at N=%.2f E=%.2f D=%.2f (alt=%.2f m)",
                    hover_pos.north_m, hover_pos.east_m,
                    hover_pos.down_m, -hover_pos.down_m)

        return f"Reached {alt:.1f} m and holding\nEND_RESPONSE"


    # Old version 
    # async def _takeoff_async(self, alt: float = 1.5, timeout: float = 30.0):
    #     """
    #     Coroutine to arm & take off, retrying on retryable errors,
    #     then wait until reaching ~95% of target altitude.
    #     Uses local NED position (not GPS) for altitude estimation.
    #     """
        
    # # ── early-exit guard 
    #     # try:
    #     #     if await self._is_airborne():
    #     #         self.log.warning("Take-off request ignored – drone already airborne.")
    #     #         return "ERR Take-off blocked: drone already airborne\nEND_RESPONSE"
    #     # except asyncio.TimeoutError:
    #     #     self.log.warning("Take-off guard: no landed_state telemetry; proceeding anyway")

    #     deadline = time.time() + timeout
    #     while time.time() < deadline:
    #         try:
    #             # Set the takeoff altitude 
    #             try:
    #                 alt = 1.5
    #                 await self._drone.action.set_takeoff_altitude(alt)
    #                 self.log.info(f"✅ Takeoff altitude set to {alt:.2f} meters")
    #             except mavsdk.action.ActionError as err:
    #                 self.log.error(f"❌ Failed to set takeoff altitude: {err._result.result_str}")


    #         # try:
    #         #     await self._drone.action.set_flight_mode(FlightMode.GUIDED)
    #         #     self.log.info(f"✅ Switched to Guided mode")
    #         # except mavsdk.action.ActionError as err:
    #         #     self.log.error(f"❌ Failed to switch to guided mode: {err._result.result_str}")

    #             await self._drone.action.arm()
    #             await self._drone.action.takeoff()
    #             break
    #         except ActionError as err:
    #             if err._result.result in _RETRYABLE:
    #                 await asyncio.sleep(1.0)
    #                 continue
    #             return f"ERR Take-off failed: {err._result.result_str}\nEND_RESPONSE"
    #     else:
    #         return "ERR Take-off timeout\nEND_RESPONSE"

    #     # Wait for altitude using NED data
    #     final_ned = None
    #     try:
    #         async for ned in self._drone.telemetry.position_velocity_ned():
    #             current_alt = -ned.position.down_m  # NED: down is positive, so negate
    #             self.log.info("Climbing… current altitude: %.2f m (NED)", current_alt)
    #             if current_alt >= 0.95 * alt:
    #                 final_ned = ned.position
    #                 break
    #             await asyncio.sleep(0.1)
    #     except asyncio.CancelledError:
    #         return "ERR Link lost during climb\nEND_RESPONSE"
        
    #     # Switch to HOLD mode after takeoff (only on real drone)
    #     if not self.sim:
    #         try:
    #             await self._drone.action.hold()
    #             self.log.info("🛑 Requested hold mode switch… waiting for confirmation")

    #             t0 = time.time()
    #             async for mode in self._drone.telemetry.flight_mode():
    #                 if mode == FlightMode.HOLD:
    #                     self.log.info("✅ Flight mode switched to HOLD")
    #                     break
    #                 if time.time() - t0 > 5.0:
    #                     self.log.warning("⚠️ HOLD mode switch not confirmed in time")
    #                     break
    #         except Exception as e:
    #             self.log.error(f"❌ Failed to switch to HOLD: {e}")
    #     else:
    #         self.log.info("ℹ️ Skipping HOLD mode switch in simulation to prevent auto-landing.")        

    #     # Log final NED position after reaching altitude
    #     if final_ned:
    #         self.log.info(
    #             "🎯 Final NED position: N=%.2f, E=%.2f, D=%.2f",
    #             final_ned.north_m,
    #             final_ned.east_m,
    #             final_ned.down_m
    #         )

    #     return f"Reached {alt:.1f} m\nEND_RESPONSE"
    
    # Future version using mavlink passthrough
    # async def _takeoff_async(self, alt: float = 2.0, timeout: float = 30.0):
    #     """
    #     Coroutine to arm & take off, retrying on retryable errors,
    #     then wait until reaching ~95% of target altitude,
    #     then switch to LOITER mode (for position hold).
    #     """
    #     deadline = time.time() + timeout
    #     while time.time() < deadline:
    #         try:
    #             await self._drone.action.arm()
    #             await self._drone.action.takeoff()
    #             break
    #         except ActionError as err:
    #             if err._result.result in _RETRYABLE:
    #                 await asyncio.sleep(1.0)
    #                 continue
    #             return f"ERR Take-off failed: {err._result.result_str}\nEND_RESPONSE"
    #     else:
    #         return "ERR Take-off timeout\nEND_RESPONSE"

    #     # Wait for altitude
    #     try:
    #         async for pos in self._drone.telemetry.position():
    #             current_alt = pos.relative_altitude_m
    #             self.log.info("Climbing… current altitude: %.2f m", current_alt)
    #             if current_alt >= 0.95 * alt:
    #                 break
    #             await asyncio.sleep(0.1)
    #     except asyncio.CancelledError:
    #         return "ERR Link lost during climb\nEND_RESPONSE"

    #     # Request switch to hold mode
    #     hold_success = False
    #     try:
    #         await self._drone.action.hold()
    #         self.log.info("🛑 Requested HOLD mode switch… waiting for confirmation")

    #         t0 = time.time()
    #         async for mode in self._drone.telemetry.flight_mode():
    #             if mode == FlightMode.HOLD:
    #                 self.log.info("✅ Flight mode switched to HOLD")
    #                 hold_success = True
    #                 break
    #             if time.time() - t0 > 5.0:
    #                 self.log.warning("⚠️  HOLD mode switch not confirmed in time")
    #                 break
    #     except Exception as e:
    #         self.log.error(f"❌ Failed to switch to HOLD: {e}")


    #     # Measure drift in position after 5 seconds        
    #     try:
    #         # Get starting NED position
    #         start_ned = await asyncio.wait_for(
    #             self._drone.telemetry.position_velocity_ned().__anext__(), timeout=2.0
    #         )
    #         start = start_ned.position
    #         self.log.info("🔹 Position hold test: N=%.2f E=%.2f", start.north_m, start.east_m)

    #         await asyncio.sleep(5.0)

    #         # Get ending NED position
    #         end_ned = await asyncio.wait_for(
    #             self._drone.telemetry.position_velocity_ned().__anext__(), timeout=2.0
    #         )
    #         end = end_ned.position

    #         delta_n = end.north_m - start.north_m
    #         delta_e = end.east_m - start.east_m
    #         drift = math.sqrt(delta_n**2 + delta_e**2)

    #         self.log.info("🔍 Drift after 5s: %.2f meters", drift)
    #     except Exception as e:
    #         self.log.warning(f"Could not assess drift: {e}")

    #     if hold_success:
    #         return f"Reached {alt:.1f} m and switched to HOLD\nEND_RESPONSE"
    #     else:
    #         return f"Reached {alt:.1f} m but failed to confirm HOLD mode\nEND_RESPONSE"
    

    async def _is_airborne(self, alt_threshold: float = 0.15) -> bool:
        """
        Returns True if the vehicle is already in the air.

        • Primary test: Telemetry.landed_state()
        • Fallback:     relative_altitude_m > alt_threshold (for firmwares
                        that don’t publish landed-state in SITL).

        Raises asyncio.TimeoutError if no telemetry arrives in 1 s.
        """
        try:
            st = await asyncio.wait_for(
                self._drone.telemetry.landed_state().__anext__(), 1.0
            )
            if st != tlm.LandedState.ON_GROUND:
                self.log.debug("airborne check: landed_state=%s", st.name)
                return True
        except asyncio.TimeoutError:
            self.log.debug("airborne check: landed_state timeout")

        pos = self._cache.get("pos")
        if pos and pos.relative_altitude_m > alt_threshold:
            self.log.debug("airborne check: rel_alt=%.2f m", pos.relative_altitude_m)
            return True

        return False


    def land_drone(self):
        """Synchronously land the drone (blocks until on ground)."""
        return self._run(self._land_async())

    async def _land_async(self):
        """
        Coroutine to land and wait until on ground, logging altitude
        on each update.
        """
        # 1) send the land command
        await self._drone.action.land()

        # 2) then loop until we see ON_GROUND, but log altitude each tick
        async for state in self._drone.telemetry.landed_state():
            # pull one position update so we can log current altitude
            try:
                pos = await asyncio.wait_for(
                    self._drone.telemetry.position().__anext__(),
                    timeout=1.0
                )
                current_alt = pos.relative_altitude_m
                self.log.info("Landing… current altitude: %.2f m", current_alt)
            except asyncio.TimeoutError:
                # if no position came in, just skip the log this round
                self.log.warning("Landing… no position update")

            if state == mavsdk.telemetry.LandedState.ON_GROUND:
                self.log.info("Touchdown confirmed")
                break

        return "Drone landed.\nEND_RESPONSE"

    async def _start_offboard_once(self):
        """
        Send a neutral setpoint then start offboard mode once
        so future set_velocity calls succeed.
        """
        try:
            await self._drone.offboard.set_velocity_ned(VelocityNedYaw(0,0,0,0))
            await self._drone.offboard.start()
        except OffboardError as err:
            if err._result.result != mavsdk.offboard.OffboardResult.Result.BUSY:
                raise

    # ─────────────── Movement & Yaw ───────────────────────────────────
    def move(
        self,
        direction: Union[str, Tuple[float,float,float]] = "FWD",
        distance: float = 0.5,
        velocity: float = 0.5,
        block: bool = True,
        duration: Optional[float] = None
    ):
        """
        Entry point for MOVE commands.  Returns end-string immediately if block=False.
        """
        self.log.info(f"Calling _move_async with direction: {direction}, distance: {distance}, velocity: {velocity}, duration: {duration}")
        return self._run(
            self._move_async(direction, distance, velocity, duration),
            block=block
        )

    # ------------------------------------------------------------------
    # Helper to keep the vehicle frozen without changing flight-mode
    # ------------------------------------------------------------------
    async def _feed_position_hold(self, pos: PositionNedYaw, period: float = 0.2):
        """
        Re-send `pos` every `period` s so ArduPilot maintains a tight
        position hold while remaining in GUIDED/OFFBOARD.
        Cancel this asyncio.Task to release the hold.
        """
        try:
            while True:
                await self._drone.offboard.set_position_ned(pos)
                await asyncio.sleep(period)
        except asyncio.CancelledError:
            # one last neutral command so the FC does not drift
            await self._drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
            raise


    # ------------------------------------------------------------------
    # New MOVE implementation – absolute NED target + background hold
    # ------------------------------------------------------------------
    async def _move_async(self, direction, distance, velocity, duration):
        """
        1) Read current NED position.
        2) Build an absolute target point = start + unit·distance.
        3) Stream that point until we arrive (≤ 0.15 m) or 15 s timeout.
        4) Start a background task that keeps streaming the same point
        so the drone keeps holding position in GUIDED/OFFBOARD.
        """

        aliases = {
            "FWD": (1, 0, 0), "BACK": (-1, 0, 0),
            "RIGHT": (0, 1, 0), "LEFT": (0, -1, 0),
            "UP": (0, 0, -1), "DOWN": (0, 0, 1),
        }
        vec = aliases.get(direction.upper(), direction)
        if not (isinstance(vec, tuple) and len(vec) == 3):
            return f"Bad direction '{direction}'.\nEND_RESPONSE"

        # -------------------------------------------------------------- 1
        try:
            start_pv = await asyncio.wait_for(
                self._drone.telemetry.position_velocity_ned().__anext__(), 2.0)
        except asyncio.TimeoutError:
            return "ERR no NED data – cannot move\nEND_RESPONSE"

        start = start_pv.position
        self.log.info("MOVE %s start @ N=%.2f E=%.2f D=%.2f",
                    direction, start.north_m, start.east_m, start.down_m)

        # -------------------------------------------------------------- 2
        distance = 1 # Hardcode travel distance to 1m (for now)
        mag = math.sqrt(sum(v*v for v in vec))
        unit = tuple(v/mag for v in vec)
        tgt_n = start.north_m + unit[0] * distance
        tgt_e = start.east_m  + unit[1] * distance
        tgt_d = start.down_m  + unit[2] * distance          # +D is down in NED
        target = PositionNedYaw(tgt_n, tgt_e, tgt_d, 0)

        self.log.info(f"Target Position Set, North: {tgt_n}, East: {tgt_e}, Down: {tgt_d}")
        
        # -------------------------------------------------------------- cancel old hold
        if getattr(self, "_hold_task", None) and not self._hold_task.done():
            self._hold_task.cancel()
            self.log.debug("Previous hold task cancelled before MOVE")
                
        # -------------------------------------------------------------- 3
        await self._start_offboard_once()
        arrived = False
        t0 = time.time()

        while True:
            await self._drone.offboard.set_position_ned(target)

            pv = await asyncio.wait_for(
                self._drone.telemetry.position_velocity_ned().__anext__(), 1.0)
            now = pv.position
            err = math.sqrt((now.north_m - tgt_n)**2 +
                            (now.east_m - tgt_e)**2 +
                            (now.down_m - tgt_d)**2)

            if err < 0.15:            # 15 cm arrival band
                arrived = True
                break
            if time.time() - t0 > 15: # safety timeout
                self.log.warning("MOVE timeout: err=%.2f m", err)
                break

            await asyncio.sleep(0.2)  # 5 Hz streaming

        # -------------------------------------------------------------- 4
        # cancel previous hold (if any) and start a new one on `target`
        if getattr(self, "_hold_task", None) and not self._hold_task.done():
            self._hold_task.cancel()
        self._hold_task = self._loop.create_task(
            self._feed_position_hold(target))
        self.log.info("🔒 Position hold feed started (GUIDED)")

        # -------------------------------------------------------------- log & return
        end = (await self._drone.telemetry.position_velocity_ned().__anext__()).position
        moved = math.sqrt((end.north_m - start.north_m)**2 +
                        (end.east_m  - start.east_m )**2 +
                        (end.down_m  - start.down_m )**2)
        self.log.info("MOVE %s end   @ N=%.2f E=%.2f D=%.2f   Δ=%.2f m",
                    direction, end.north_m, end.east_m, end.down_m, moved)

        status = "(arrived)" if arrived else "(timeout)"
        return f"MOVE OK {direction} {moved:.2f} m {status}\nEND_RESPONSE"


# Old version 
    # async def _move_async(self, direction, distance, velocity, duration):
    #     """
    #     Coroutine to:
    #      1) Grab a start NED reading (if available),
    #      2) Enter offboard and send velocity,
    #      3) Sleep for duration,
    #      4) Stop, grab end NED reading, and log Δ distance.
    #     """
    #     aliases = {
    #         "FWD":  ( 1, 0, 0), "BACK": (-1, 0, 0),
    #         "RIGHT":( 0, 1, 0), "LEFT": ( 0,-1, 0),
    #         "UP":   ( 0, 0,-1), "DOWN": ( 0, 0, 1),
    #     }
    #     vec = aliases.get(direction.upper(), direction)
    #     if not (isinstance(vec, tuple) and len(vec) == 3):
    #         return f"Bad direction '{direction}'.\nEND_RESPONSE"

    #     mag = math.sqrt(sum(v*v for v in vec))
    #     unit = tuple(v/mag for v in vec)
    #     duration = duration or (distance / velocity)
    #     vx, vy, vz = (u * velocity for u in unit)
    #     # start NED
    #     start_ned = None
    #     try:
    #         pv0 = await asyncio.wait_for(
    #             self._drone.telemetry.position_velocity_ned().__anext__(),
    #             timeout=2.0
    #         )
    #         start_ned = pv0.position
    #         self.log.info(
    #             "MOVE %s start @ N=%.2f, E=%.2f, D=%.2f",
    #             direction,
    #             start_ned.north_m,
    #             start_ned.east_m,
    #             start_ned.down_m
    #         )
    #     except asyncio.TimeoutError:
    #         self.log.warning("MOVE %s start: no NED data received", direction)

    #     # do the move
    #     await self._start_offboard_once()
    #     self.log.info(f"set_velocity_ned called with vx: {vx}, vy: {vy}, vz: {-vz}")
    #     await self._drone.offboard.set_velocity_ned(VelocityNedYaw(vx, vy, -vz, 0))
    #     await asyncio.sleep(duration)
    #     await self._drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))

    #     hold_success = False
    #     if not self.sim:
    #         await self._drone.offboard.stop()
    #         self.log.info("🛑 Offboard stopped. Switching to HOLD to maintain position...")
    #         self.log.info("Switching to HOLD to maintain position...")
    #         try:
    #             await self._drone.action.hold()
    #             self.log.info("🔁 Requested HOLD mode switch… waiting for confirmation")

    #             t0 = time.time()
    #             async for mode in self._drone.telemetry.flight_mode():
    #                 if mode == FlightMode.HOLD:
    #                     self.log.info("✅ Flight mode switched to HOLD")
    #                     hold_success = True
    #                     break
    #                 if time.time() - t0 > 5.0:
    #                     self.log.warning("⚠️ HOLD mode switch not confirmed in time")
    #                     break
    #         except Exception as e:
    #             self.log.warning(f"❌ Failed to switch to HOLD after move: {e}")
    #     else:
    #         self.log.info("ℹ️ Skipping HOLD mode switch in simulation to prevent auto-landing.")
                       
    #     try:
    #         pv1 = await asyncio.wait_for(
    #             self._drone.telemetry.position_velocity_ned().__anext__(),
    #             timeout=2.0
    #         )
    #         end_ned = pv1.position
    #         if start_ned:
    #             moved = _ned_distance(start_ned, end_ned)
    #             self.log.info(
    #                 "MOVE %s end   @ N=%.2f, E=%.2f, D=%.2f   Δ=%.2f m",
    #                 direction,
    #                 end_ned.north_m,
    #                 end_ned.east_m,
    #                 end_ned.down_m,
    #                 moved
    #             )
    #         else:
    #             self.log.info(
    #                 "MOVE %s end   @ N=%.2f, E=%.2f, D=%.2f",
    #                 direction,
    #                 end_ned.north_m,
    #                 end_ned.east_m,
    #                 end_ned.down_m
    #             )
    #     except asyncio.TimeoutError:
    #         self.log.warning("MOVE %s end: no NED data received", direction)

    #     status = "and switched to HOLD" if hold_success else "but HOLD mode not confirmed"
    #     return f"MOVE OK {direction} {distance} m {status}\nEND_RESPONSE"


    # async def _move_async(self, direction, distance, velocity, duration):
    #     """
    #     Move in NED, log start/end/Δ, then switch to stationary mode
    #     (LOITER-only in SIM).
    #     """
    #     aliases = {
    #         "FWD": (1, 0, 0), "BACK": (-1, 0, 0),
    #         "RIGHT": (0, 1, 0), "LEFT": (0, -1, 0),
    #         "UP": (0, 0, -1), "DOWN": (0, 0, 1),
    #     }
    #     vec = aliases.get(direction.upper(), direction)
    #     if not (isinstance(vec, tuple) and len(vec) == 3):
    #         return f"Bad direction '{direction}'.\nEND_RESPONSE"

    #     mag = math.sqrt(sum(v * v for v in vec))
    #     unit = tuple(v / mag for v in vec)
    #     duration = duration or (distance / velocity)
    #     vx, vy, vz = (u * velocity for u in unit)

    #     # ── log start NED ────────────────────────────────────────────────
    #     start_ned = None
    #     try:
    #         start_ned = (await self._drone.telemetry.position_velocity_ned().__anext__()).position
    #         self.log.info("MOVE %s start @ N=%.2f E=%.2f D=%.2f",
    #                     direction, start_ned.north_m, start_ned.east_m, start_ned.down_m)
    #     except Exception:
    #         self.log.warning("MOVE %s start: NED unavailable", direction)

    #     # ── execute movement in offboard ─────────────────────────────────
    #     await self._start_offboard_once()
    #     await self._drone.offboard.set_velocity_ned(VelocityNedYaw(vx, vy, -vz, 0))
    #     await asyncio.sleep(duration)
    #     await self._drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
    #     await self._drone.offboard.stop()
    #     self.log.info("🛑 Offboard stopped after MOVE")

    #     # ── attempt mode change ─────────────────────────────────────────
    #     success, reason = await self._switch_to_stationary_mode(sim_loiter_only=self.sim)

    #     # ── end NED + Δ log ─────────────────────────────────────────────
    #     end_ned = None
    #     try:
    #         end_ned = (await self._drone.telemetry.position_velocity_ned().__anext__()).position
    #         if start_ned:
    #             moved = _ned_distance(start_ned, end_ned)
    #             self.log.info("MOVE %s end   @ N=%.2f E=%.2f D=%.2f   Δ=%.2f m",
    #                         direction, end_ned.north_m, end_ned.east_m,
    #                         end_ned.down_m, moved)
    #         else:
    #             self.log.info("MOVE %s end   @ N=%.2f E=%.2f D=%.2f",
    #                         direction, end_ned.north_m, end_ned.east_m, end_ned.down_m)
    #     except Exception:
    #         self.log.warning("MOVE %s end: NED unavailable", direction)

    #     delta_str = f"Δ≈{_ned_distance(start_ned, end_ned):.2f} m " if (start_ned and end_ned) else ""
    #     status = f"and hovering in {reason}" if success else f"but hover not confirmed ({reason})"
    #     return f"MOVE {direction} {delta_str}{status}\nEND_RESPONSE"


    def yaw_drone(self, arg: str, angle: float = 15.0):
        """
        Entry point for YAW commands.  Rotates at a fixed rate for ~2s.
        """
        return self._run(self._yaw_async(arg, angle))

    async def _yaw_async(self, arg: str, step: float = 15.0):
        """
        Increment heading by ±`step` degrees, then keep sending the
        current NED position + final yaw so the drone holds station in
        GUIDED/OFFBOARD (works on ArduPilot without mode switches).
        """

        # 1) signed delta
        delta = -abs(step) if arg.upper() in ("LEFT", "YAWLEFT") else abs(step)

        # 2) need initial attitude
        if not self._cache["att"]:
            try:
                self._cache["att"] = await asyncio.wait_for(
                    self._drone.telemetry.attitude_euler().__anext__(), 1.0)
            except asyncio.TimeoutError:
                return "ERR no attitude data\nEND_RESPONSE"

        current_yaw = self._cache["att"].yaw_deg
        target_yaw  = (current_yaw + delta + 360.0) % 360.0

        self.log.info("YAW %+.1f° → target %.1f°", delta, wrap180(target_yaw))
        
        # -------------------------------------------------------------- cancel old hold
        if getattr(self, "_hold_task", None) and not self._hold_task.done():
            self._hold_task.cancel()
            self.log.debug("Previous hold task cancelled before MOVE")        

        # 3) send the yaw set-point (zero linear velocity)
        await self._start_offboard_once()
        await self._drone.offboard.set_velocity_ned(
            VelocityNedYaw(0, 0, 0, target_yaw)
        )

        # 4) wait until heading error < 2° or 5 s timeout
        try:
            t0 = time.time()
            async for att in self._drone.telemetry.attitude_euler():
                err = (att.yaw_deg - target_yaw + 540) % 360 - 180
                if abs(err) < 2.0:
                    break
                if time.time() - t0 > 5:
                    self.log.warning("⚠️  Yaw timeout (err=%.1f°)", err)
                    break
                await asyncio.sleep(0.05)
        except asyncio.CancelledError:
            pass

        # 5) fetch latest NED position for the hold set-point
        try:
            pv = await asyncio.wait_for(
                self._drone.telemetry.position_velocity_ned().__anext__(), 2.0)
            pos_ned = pv.position
        except asyncio.TimeoutError:
            self.log.warning("No NED data after yaw; using zeros")
            pos_ned = namedtuple("XYZ", "north_m east_m down_m")(0, 0, 0)

        # 6) start / restart background position-hold task
        if getattr(self, "_hold_task", None) and not self._hold_task.done():
            self._hold_task.cancel()

        hold_point = PositionNedYaw(pos_ned.north_m,
                                    pos_ned.east_m,
                                    pos_ned.down_m,
                                    target_yaw)

        self._hold_task = self._loop.create_task(
            self._feed_position_hold(hold_point))
        self.log.info("🔒 Position hold feed started after yaw")

        # 7) final log & return
        final_yaw = self._cache["att"].yaw_deg if self._cache["att"] else float("nan")
        self.log.info("YAW start %.1f° → target %.1f° → end %.1f°",
                    wrap180(current_yaw), wrap180(target_yaw), wrap180(final_yaw))

        return f"YAW {delta:+.0f}° complete\nEND_RESPONSE"


# old Version 
    # async def _yaw_async(self, arg: str, step: float = 15.0):
    #     # 1) figure out the signed delta we want
    #     delta = -abs(step) if arg.upper() in ("LEFT", "YAWLEFT") else abs(step)

    #     # 2) current attitude must be in the cache (wait max 1 s)
    #     if not self._cache["att"]:
    #         try:
    #             self._cache["att"] = await asyncio.wait_for(
    #                 self._drone.telemetry.attitude_euler().__anext__(), 1.0)
    #         except asyncio.TimeoutError:
    #             return "ERR no attitude data\nEND_RESPONSE"

    #     current = self._cache["att"].yaw_deg
    #     target  = (current + delta + 360.0) % 360.0        # wrap to 0-360

    #     # self.log.info("YAW %+.1f° → target %.1f°", delta, target)
    #     self.log.info("YAW %+.1f° → target %.1f°", delta, wrap180(target))

    #     # 3) send the new absolute heading
    #     await self._start_offboard_once()
    #     await self._drone.offboard.set_velocity_ned(
    #         VelocityNedYaw(0, 0, 0, target)
    #     )

    #     # 4) wait until we’re close (≤2 °) or 5 s timeout
    #     try:
    #         t0 = time.time()
    #         async for att in self._drone.telemetry.attitude_euler():
    #             err = (att.yaw_deg - target + 540) % 360 - 180   # shortest-path error (–180 … 180)
    #             if abs(err) < 2.0:
    #                 break
    #             if time.time() - t0 > 5:
    #                 self.log.warning("⚠️  Yaw timeout (err=%.1f°)", err)
    #                 break
    #             await asyncio.sleep(0.05)
    #     except asyncio.CancelledError:
    #         pass

    #     if not self.sim:
    #         await self._drone.offboard.stop()
    #         self.log.info("🛑 Offboard stopped. Switching to HOLD to maintain position...")
    #         self.log.info("Switching to HOLD to maintain position...")
    #         try:
    #             await self._drone.action.hold()
    #             self.log.info("🔁 Requested HOLD mode switch… waiting for confirmation")

    #             t0 = time.time()
    #             async for mode in self._drone.telemetry.flight_mode():
    #                 if mode == FlightMode.HOLD:
    #                     self.log.info("✅ Flight mode switched to HOLD")
    #                     hold_success = True
    #                     break
    #                 if time.time() - t0 > 5.0:
    #                     self.log.warning("⚠️ HOLD mode switch not confirmed in time")
    #                     break
    #         except Exception as e:
    #             self.log.warning(f"❌ Failed to switch to HOLD after move: {e}")
    #     else:
    #         self.log.info("ℹ️ Skipping HOLD mode switch in simulation to prevent auto-landing.")
            
    #     # 5) final attitude for the pretty log
    #     final = self._cache["att"].yaw_deg if self._cache["att"] else float("nan")
    #     self.log.info("YAW start %.1f° → target %.1f° → end %.1f°", wrap180(current), wrap180(target), wrap180(final))       
            
    #     return f"YAW {delta:+.0f}° complete\nEND_RESPONSE"


    # async def _yaw_async(self, arg, angle):
    #     """
    #     Yaw for ~2 s, log start/end yaw, then switch to stationary mode
    #     (LOITER-only in SIM).
    #     """
    #     yaw_rate = -abs(angle) if arg.upper() in ("LEFT", "YAWLEFT") else \
    #             abs(angle)  if arg.upper() in ("RIGHT", "YAWRIGHT") else \
    #             float(arg)

    #     # ── log initial yaw ─────────────────────────────────────────────
    #     if self._cache["att"]:
    #         self.log.info("YAW start: %.1f °", self._cache["att"].yaw_deg)

    #     # ── perform yaw in offboard ─────────────────────────────────────
    #     await self._start_offboard_once()
    #     await self._drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, yaw_rate))
    #     await asyncio.sleep(2.0)
    #     await self._drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
    #     await self._drone.offboard.stop()
    #     self.log.info("🛑 Offboard stopped after YAW")

    #     # ── attempt mode change ─────────────────────────────────────────
    #     success, reason = await self._switch_to_stationary_mode(sim_loiter_only=self.sim)

    #     # ── log final yaw ───────────────────────────────────────────────
    #     if self._cache["att"]:
    #         self.log.info("YAW end: %.1f °", self._cache["att"].yaw_deg)

    #     status = f"and hovering in {reason}" if success else f"but hover not confirmed ({reason})"
    #     return f"YAW rate≈{yaw_rate} °/s {status}\nEND_RESPONSE"


    # ─────────────── Telemetry Interface ─────────────────────────────
    def start_telemetry(self, radio, period: float = 0.2):
        """
        Start a background thread that periodically sends the JSON
        state snapshot over the provided radio interface.
        """
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
        """Pause the background telemetry thread."""
        self._telem_suspended.set()

    # ─────────────── State Snapshot ───────────────────────────────────
    def get_state_dict(self):
        """
        Return the latest cached {battery, position, attitude} as a dict.
        """
        return self._run(self._state_async())

    async def _state_async(self):
        """
        Coroutine to assemble a timestamped snapshot of battery, position,
        attitude, and both global + NED location (if available).
        Waits up to 5s for the initial full cache.
        """
        try:
            await asyncio.wait_for(self._cache_ready.wait(), 5.0)
        except asyncio.TimeoutError:
            pass

        batt = self._cache["batt"]
        pos  = self._cache["pos"]
        att  = self._cache["att"]
        vel  = self._cache.get("vel")  # NED position

        global_loc = None
        ned_loc = None

        if pos is not None:
            global_loc = {
                "lat":     pos.latitude_deg,
                "lon":     pos.longitude_deg,
                "rel_alt": pos.relative_altitude_m
            }

        if vel is not None:
            ned_loc = {
                "north_m": vel.north_m,
                "east_m":  vel.east_m,
                "alt_m":  -vel.down_m   # NED → Up
            }

        return {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "indoor_mode": self.indoor,
            "battery": {
                "voltage":   batt.voltage_v if batt else 0.0,
                "remaining": batt.remaining_percent if batt else 0.0
            },
            "location": {
                "gps": global_loc or "unavailable",
                "ned": ned_loc or "unavailable",
                "primary": "ned" if (self.indoor or global_loc is None or global_loc["rel_alt"] < 0.2) else "gps"
            },
            "attitude": {
                "roll":  att.roll_deg if att else 0.0,
                "pitch": att.pitch_deg if att else 0.0,
                "yaw":   att.yaw_deg if att else 0.0
            }
        }

    def get_state(self):
        """Return JSON + END_RESPONSE of the current state snapshot."""
        blob = json.dumps(self.get_state_dict(), indent=2)
        return blob + "\nEND_RESPONSE"

    # ─────────────── Internal Runner ──────────────────────────────────
    async def _start_offboard_once(self):
        """
        Send a neutral setpoint then start offboard mode once
        so future set_velocity calls succeed.
        """
        try:
            await self._drone.offboard.set_velocity_ned(VelocityNedYaw(0,0,0,0))
            await self._drone.offboard.start()
        except OffboardError as err:
            if err._result.result != mavsdk.offboard.OffboardResult.Result.BUSY:
                raise

    def _run(self, coro, block: bool = True):
        """
        Ensure loop+connection, then run the given coroutine.
        If block=True, wait for and return its result.
        """
        self._ensure_loop()
        if not self._connected:
            self.connect(timeout=20.0)

        fut = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return fut.result() if block else None
    
    
    async def _apply_param_file(self, filepath: str):
        """
        Apply parameters from a .parm file to the connected flight controller.
        Supports lines like: PARAM_NAME,VALUE   ; optional comment
        """
        with open(filepath, "r") as f:
            lines = f.readlines()

        for line in lines:
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            # Strip out inline comments
            if ";" in line:
                line = line.split(";")[0].strip()

            if "," not in line:
                self.log.warning(f"Skipping line '{line}': missing comma")
                continue

            try:
                name, val = [x.strip() for x in line.split(",", 1)]
                if "." in val:
                    await self._drone.param.set_param_float(name, float(val))
                else:
                    await self._drone.param.set_param_int(name, int(val))
                self.log.debug(f"Set param {name} = {val}")
                await asyncio.sleep(0.05)  # Avoid overloading the link
            except Exception as e:
                self.log.warning(f"Skipping line '{line}': {e}")


    # async def _switch_to_stationary_mode(
    #     self, *, sim_loiter_only: bool
    # ) -> Tuple[bool, str]:
    #     """
    #     • If Action-server exposes a list of modes → use it (PX-4 style).
    #     • Otherwise (ArduPilot) → send MAV_CMD_DO_SET_MODE via mavlink_passthrough.

    #     Returns (True, <mode_name>) on success, else (False, <reason>).
    #     """
    #     wanted = ["LOITER"] if sim_loiter_only else ["LOITER", "HOLD", "POSCTL"]

    #     # ── 1) TRY THE ACTION-SERVER LIST (works on PX-4) ───────────────
    #     try:
    #         allow_obj = await self._drone.action_server.get_allowable_flight_modes()
    #         modes_attr = (
    #             getattr(allow_obj, "modes", None)
    #             or getattr(allow_obj, "flight_modes", None)
    #         )

    #         if modes_attr:                                   # PX-4 branch
    #             mode_list = list(modes_attr)
    #             target = next((m for m in mode_list if m.name in wanted), None)
    #             if target:
    #                 self.log.info(f"🔁 Requesting mode → {target.name} (PX-4)")
    #                 await self._drone.action_server.flight_mode_change(target)

    #                 # confirm ≤5 s
    #                 t0 = time.time()
    #                 async for fm in self._drone.telemetry.flight_mode():
    #                     if fm == target:
    #                         self.log.info(f"✅ Mode confirmed: {fm.name}")
    #                         return True, target.name
    #                     if time.time() - t0 > 5:
    #                         break
    #                 return False, "timeout waiting for PX-4 confirmation"

    #         else:
    #             self.log.debug("Action-server returned no iterable modes; "
    #                             "falling back to MAVLink set-mode")

    #     except Exception as e:
    #         self.log.debug(f"Action-server branch raised {e}; "
    #                     "falling back to MAVLink set-mode")

    #     # ── 2) FALL BACK: SEND MAV_CMD_DO_SET_MODE (ArduPilot) ──────────
    #     try:
    #         from pymavlink.dialects.v20 import common as mavlink2
    #     except ImportError as e:
    #         reason = f"pymavlink not present ({e})"
    #         self.log.error(reason)
    #         return False, reason

    #     # Create (or reuse) the mavlink-passthrough plugin  ───────────────
    #     try:
    #         if not hasattr(self, "_mp"):
    #             from mavsdk.mavlink_passthrough import MavlinkPassthrough
    #             self._mp = MavlinkPassthrough(self._drone)
    #     except Exception as e:
    #         reason = f"mavlink_passthrough unavailable ({e})"
    #         self.log.error(reason)
    #         return False, reason

    #     # pick first preferred mode that ArduPilot implements
    #     for name in wanted:
    #         if name in _AP_MODE:
    #             mode_num = _AP_MODE[name]
    #             break
    #     else:
    #         return False, "no usable ArduPilot mode in preference list"

    #     msg = mavlink2.MAVLink_command_long_message(
    #         target_system=1,
    #         target_component=1,
    #         command=mavlink2.MAV_CMD_DO_SET_MODE,
    #         confirmation=0,
    #         param1=mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # use custom-mode field
    #         param2=float(mode_num),  # ArduPilot custom-mode number
    #         param3=0, param4=0, param5=0, param6=0, param7=0,
    #     )
    #     self.log.info(f"🔁 Sending MAV_CMD_DO_SET_MODE → {name} ({mode_num})")
    #     self._mp.send_message(msg)

    #     # confirm via Telemetry.flight_mode() ≤5 s
    #     t0 = time.time()
    #     async for fm in self._drone.telemetry.flight_mode():
    #         if fm.name == name:
    #             self.log.info(f"✅ Mode confirmed: {fm.name}")
    #             return True, name
    #         if time.time() - t0 > 5:
    #             break

    #     return False, "timeout waiting for ArduPilot confirmation"

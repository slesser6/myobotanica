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
from mavsdk.offboard import OffboardError, VelocityNedYaw      
         
# from pymavlink.dialects.v20.common import MAVLink_set_gps_global_origin_message, MAVLink_set_home_position_message
# from mavsdk import System
# from mavsdk.mavlink_passthrough import MavlinkPassthrough
# from pymavlink.dialects.v20 import common as mavlink2


# Local
from .sitl import SitlManager

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

    def takeoff_drone(self, alt: float = 2.0):
        """Synchronously take off to a given altitude."""
        return self._run(self._takeoff_async(alt))

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

    async def _takeoff_async(self, alt: float = 2.0, timeout: float = 30.0):
        """
        Coroutine to arm & take off, retrying on retryable errors,
        then wait until reaching ~95% of target altitude.
        Uses local NED position (not GPS) for altitude estimation.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                await self._drone.action.arm()
                await self._drone.action.takeoff()
                break
            except ActionError as err:
                if err._result.result in _RETRYABLE:
                    await asyncio.sleep(1.0)
                    continue
                return f"ERR Take-off failed: {err._result.result_str}\nEND_RESPONSE"
        else:
            return "ERR Take-off timeout\nEND_RESPONSE"

        # Wait for altitude using NED data
        final_ned = None
        try:
            async for ned in self._drone.telemetry.position_velocity_ned():
                current_alt = -ned.position.down_m  # NED: down is positive, so negate
                self.log.info("Climbing… current altitude: %.2f m (NED)", current_alt)
                if current_alt >= 0.95 * alt:
                    final_ned = ned.position
                    break
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            return "ERR Link lost during climb\nEND_RESPONSE"

        # Switch to HOLD mode after takeoff
        # try:
        #     await self._drone.action.hold()
        #     self.log.info("🛑 Requested hold mode switch… waiting for confirmation")

        #     t0 = time.time()
        #     async for mode in self._drone.telemetry.flight_mode():
        #         if mode == FlightMode.HOLD:
        #             self.log.info("✅ Flight mode switched to HOLD")
        #             break
        #         if time.time() - t0 > 5.0:
        #             self.log.warning("⚠️ HOLD mode switch not confirmed in time")
        #             break
        # except Exception as e:
        #     self.log.error(f"❌ Failed to switch to HOLD: {e}")
        
        # Switch to HOLD mode after takeoff (only on real drone)
        if not self.sim:
            try:
                await self._drone.action.hold()
                self.log.info("🛑 Requested hold mode switch… waiting for confirmation")

                t0 = time.time()
                async for mode in self._drone.telemetry.flight_mode():
                    if mode == FlightMode.HOLD:
                        self.log.info("✅ Flight mode switched to HOLD")
                        break
                    if time.time() - t0 > 5.0:
                        self.log.warning("⚠️ HOLD mode switch not confirmed in time")
                        break
            except Exception as e:
                self.log.error(f"❌ Failed to switch to HOLD: {e}")
        else:
            self.log.info("ℹ️ Skipping HOLD mode switch in simulation to prevent auto-landing.")        

        # Log final NED position after reaching altitude
        if final_ned:
            self.log.info(
                "🎯 Final NED position: N=%.2f, E=%.2f, D=%.2f",
                final_ned.north_m,
                final_ned.east_m,
                final_ned.down_m
            )

        return f"Reached {alt:.1f} m\nEND_RESPONSE"

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
        distance: float = 1.0,
        velocity: float = 0.5,
        block: bool = True,
        duration: Optional[float] = None
    ):
        """
        Entry point for MOVE commands.  Returns end-string immediately if block=False.
        """
        return self._run(
            self._move_async(direction, distance, velocity, duration),
            block=block
        )

    async def _move_async(self, direction, distance, velocity, duration):
        """
        Coroutine to:
         1) Grab a start NED reading (if available),
         2) Enter offboard and send velocity,
         3) Sleep for duration,
         4) Stop, grab end NED reading, and log Δ distance.
        """
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
        duration = duration or (distance / velocity)
        vx, vy, vz = (u * velocity for u in unit)

        # start NED
        start_ned = None
        try:
            pv0 = await asyncio.wait_for(
                self._drone.telemetry.position_velocity_ned().__anext__(),
                timeout=2.0
            )
            start_ned = pv0.position
            self.log.info(
                "MOVE %s start @ N=%.2f, E=%.2f, D=%.2f",
                direction,
                start_ned.north_m,
                start_ned.east_m,
                start_ned.down_m
            )
        except asyncio.TimeoutError:
            self.log.warning("MOVE %s start: no NED data received", direction)

        # do the move
        await self._start_offboard_once()
        await self._drone.offboard.set_velocity_ned(VelocityNedYaw(vx, vy, -vz, 0))
        await asyncio.sleep(duration)
        await self._drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
           
        # try:
        #     await self._drone.action.hold()
        #     self.log.info("🔁 Requested HOLD mode switch… waiting for confirmation")

        #     t0 = time.time()
        #     async for mode in self._drone.telemetry.flight_mode():
        #         if mode == FlightMode.HOLD:
        #             self.log.info("✅ Flight mode switched to HOLD")
        #             hold_success = True
        #             break
        #         if time.time() - t0 > 5.0:
        #             self.log.warning("⚠️  HOLD mode switch not confirmed in time")
        #             break
        # except Exception as e:
        #     self.log.warning(f"❌ Failed to switch to HOLD after move: {e}")

        hold_success = False
        if not self.sim:
            await self._drone.offboard.stop()
            self.log.info("🛑 Offboard stopped. Switching to HOLD to maintain position...")
            self.log.info("Switching to HOLD to maintain position...")
            try:
                await self._drone.action.hold()
                self.log.info("🔁 Requested HOLD mode switch… waiting for confirmation")

                t0 = time.time()
                async for mode in self._drone.telemetry.flight_mode():
                    if mode == FlightMode.HOLD:
                        self.log.info("✅ Flight mode switched to HOLD")
                        hold_success = True
                        break
                    if time.time() - t0 > 5.0:
                        self.log.warning("⚠️ HOLD mode switch not confirmed in time")
                        break
            except Exception as e:
                self.log.warning(f"❌ Failed to switch to HOLD after move: {e}")
        else:
            self.log.info("ℹ️ Skipping HOLD mode switch in simulation to prevent auto-landing.")
                       
        try:
            pv1 = await asyncio.wait_for(
                self._drone.telemetry.position_velocity_ned().__anext__(),
                timeout=2.0
            )
            end_ned = pv1.position
            if start_ned:
                moved = _ned_distance(start_ned, end_ned)
                self.log.info(
                    "MOVE %s end   @ N=%.2f, E=%.2f, D=%.2f   Δ=%.2f m",
                    direction,
                    end_ned.north_m,
                    end_ned.east_m,
                    end_ned.down_m,
                    moved
                )
            else:
                self.log.info(
                    "MOVE %s end   @ N=%.2f, E=%.2f, D=%.2f",
                    direction,
                    end_ned.north_m,
                    end_ned.east_m,
                    end_ned.down_m
                )
        except asyncio.TimeoutError:
            self.log.warning("MOVE %s end: no NED data received", direction)

        status = "and switched to HOLD" if hold_success else "but HOLD mode not confirmed"
        return f"MOVE OK {direction} {distance} m {status}\nEND_RESPONSE"

    def yaw_drone(self, arg: str, angle: float = 15.0):
        """
        Entry point for YAW commands.  Rotates at a fixed rate for ~2s.
        """
        return self._run(self._yaw_async(arg, angle))

    async def _yaw_async(self, arg, angle):
        """
        Coroutine to:
        1) Log start yaw (in degrees),
        2) Enter offboard and send yaw rate,
        3) Sleep ~2s,
        4) Stop and log end yaw.
        """
        # Determine yaw rate (deg/s)
        if arg.upper() in ("LEFT", "YAWLEFT"):
            yaw_rate = -abs(angle)
        elif arg.upper() in ("RIGHT", "YAWRIGHT"):
            yaw_rate = abs(angle)
        else:
            yaw_rate = float(arg)

        # Log starting yaw angle (deg)
        start_att = self._cache["att"]
        if start_att:
            self.log.info("YAW %s start: %.1f°", arg, start_att.yaw_deg)

        # Perform the yaw
        await self._start_offboard_once()
        await self._drone.offboard.set_velocity_ned(
            VelocityNedYaw(0, 0, 0, yaw_rate)
        )
        await asyncio.sleep(2)
        # await self._drone.offboard.set_velocity_ned(
        #     VelocityNedYaw(0, 0, 0, 0)
        # )
        
        hold_success = False
        if not self.sim:
            await self._drone.offboard.stop()
            self.log.info("🛑 Offboard stopped after yaw.")
            self.log.info("Switching to HOLD mode after yaw (real hardware)...")
            try:
                await self._drone.action.hold()
                self.log.info("🔁 Requested HOLD mode switch… waiting for confirmation")

                t0 = time.time()
                async for mode in self._drone.telemetry.flight_mode():
                    if mode == FlightMode.HOLD:
                        self.log.info("✅ Flight mode switched to HOLD")
                        hold_success = True
                        break
                    if time.time() - t0 > 5.0:
                        self.log.warning("⚠️  HOLD mode switch not confirmed in time")
                        break
            except Exception as e:
                self.log.warning(f"❌ Failed to switch to HOLD mode after yaw: {e}")
        else:
            self.log.info("ℹ️  Skipping HOLD mode switch (simulation mode)")

        # Log ending yaw angle (deg)
        end_att = self._cache["att"]
        if end_att:
            self.log.info("YAW %s end:   %.1f°   Δ≈%.1f°", arg, end_att.yaw_deg, yaw_rate)

        # Final return string
        status = "and switched to HOLD" if hold_success else (
            "but HOLD mode not confirmed" if not self.sim else " (simulated)")
        return f"YAW Δ≈{yaw_rate}° {status}\nEND_RESPONSE"


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


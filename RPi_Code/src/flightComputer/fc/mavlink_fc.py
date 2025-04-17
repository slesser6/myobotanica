"""
flightComputer.fc.mavlink_fc
MAVLink / DroneKit wrapper used by the rest of the system.
"""
from __future__ import annotations
from typing import Union, Tuple, Optional
import logging, time, io, threading

from dronekit import connect, VehicleMode
from pymavlink import mavutil

from .sitl import SitlManager
from flightComputer import config

# ---------------------------------------------------------------------
def condition_yaw(vehicle, heading, relative=True, yaw_speed=0):
    """
    Helper to send MAV_CMD_CONDITION_YAW.
    """
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0, heading, yaw_speed, 1,
        1 if relative else 0, 0, 0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

# ---------------------------------------------------------------------
class FlightController:
    """
    The public API the rest of the program relies on.
    All previous methods (arm_drone, takeoff_drone, move, yaw, …) preserved.
    """

    # ---------- life‑cycle -------------------------------------------
    def __init__(self,
                 connection_string: str,
                 baud_rate: int,
                 simulation_mode: bool = False):
        self.log = logging.getLogger("FlightController")
        self.simulation_mode = simulation_mode
        self.connection_string = connection_string
        self.baud_rate = baud_rate

        # Will be set in .connect()
        self.vehicle = None
        self._sitl: Optional[SitlManager] = None

    def connect(self):
        """
        Start SITL if requested, then open the MAVLink connection.
        """
        if self.simulation_mode:
            self._sitl = SitlManager()
            self.connection_string = self._sitl.start()

        self.log.info("Connecting to FC on %s @ %d …",
                      self.connection_string, self.baud_rate)
        self.vehicle = connect(self.connection_string,
                               wait_ready=True,
                               baud=self.baud_rate)
        self.log.info("Connected. Firmware %s  Armable=%s",
                      self.vehicle.version, self.vehicle.is_armable)

        if self.simulation_mode:
            # override frame params example
            self.vehicle.parameters['FRAME_CLASS'] = 1
            self.vehicle.parameters['FRAME_TYPE']  = 1

    def close(self):
        if self.vehicle:
            self.vehicle.close()
        if self._sitl:
            self._sitl.stop()

    # ---------- public command entry‑point ---------------------------
    def send_command(self, command: str) -> str | None:
        """
        Dispatch by prefix (GETSTATE, ARM, MOVE, …).
        Returns a multi‑line string ending with END_RESPONSE
        or None on unknown command.
        """
        parts = command.strip().split(":")
        cmd = parts[0].upper()
        rest = parts[1:]

        cmd = cmd.upper()          # make sure we compare upper‑case

        if cmd == "GETSTATE":
            return self.get_state()

        elif cmd == "ARM":
            return self.arm_drone()

        elif cmd == "TAKEOFF":
            target_alt = float(rest[0]) if rest else 2.0
            return self.takeoff_drone(target_alt)

        elif cmd == "MOVE":
            return self.move(rest[0] if rest else "FWD")

        elif cmd in ("MOVEFWD", "MOVEBACK"):
            return self.move(cmd)

        elif cmd == "YAW":
            return self.yaw_drone(rest[0] if rest else "")

        elif cmd in ("YAWLEFT", "YAWRIGHT"):
            return self.yaw_drone(cmd)

        elif cmd == "LAND":
            return self.land_drone()

        else:
            self.log.warning("Unknown FC cmd %s", command)
            return None


    # ---------- state ------------------------------------------------
    def get_state(self):
        v = self.vehicle
        text = (f"Drone state:\n"
                f"  Firmware: {v.version}\n"
                f"  Location: {v.location.global_relative_frame}\n"
                f"  Battery:  {v.battery}\n"
                f"  Mode:     {v.mode.name}\n"
                f"  Armed:    {v.armed}\nEND_RESPONSE")
        return text

    # ---------- arm / take‑off / land -------------------------------
    def arm_drone(self, timeout: int = 10):
        v = self.vehicle

        self.log.info("→ Starting arming sequence")
        # 1) enter GUIDED first
        v.mode = VehicleMode("GUIDED")
        self.log.debug(f"Requested mode GUIDED, actual mode: {v.mode.name}")

        # 2) if we’re running inside SITL, disable pre‑arm checks
        if self.simulation_mode:
            self.log.debug("SIM mode → disabling arming checks")
            v.parameters['ARMING_CHECK'] = 0

        # ---------- wait until EKF, GPS, etc. report armable ----------
        start = time.time()
        while not v.is_armable and time.time() - start < (timeout - 10):
            self.log.debug(f"Waiting for armable: is_armable={v.is_armable}")
            time.sleep(1)

        if not v.is_armable:
            self.log.error("Timed out waiting for drone to become armable")
            return "Failed to arm: not armable\nEND_RESPONSE"

        # 3) request arming
        self.log.info("→ Requesting arm")
        v.armed = True

        start = time.time()
        while not v.armed and time.time() - start < timeout:
            self.log.debug(f"Waiting for armed flag: armed={v.armed}")
            time.sleep(1)

        # final check & log
        if v.armed:
            self.log.info("✅ Drone is ARMED")
            return "Drone is ARMED.\nEND_RESPONSE"
        else:
            self.log.error("❌ Failed to arm drone within timeout")
            return "Failed to arm\nEND_RESPONSE"


    def takeoff_drone(self, alt: float, timeout=15):
        if not self.vehicle.armed:
            arm_msg = self.arm_drone()
            if "Failed" in arm_msg:
                return arm_msg
        self.vehicle.simple_takeoff(alt)
        start = time.time()
        while time.time() - start < timeout:
            if self.vehicle.location.global_relative_frame.alt >= alt * 0.95:
                return f"Reached {alt} m\nEND_RESPONSE"
            time.sleep(1)
        return f"Timeout reaching {alt} m\nEND_RESPONSE"

    def land_drone(self):
        self.vehicle.mode = VehicleMode("LAND")
        return "Drone is landing.\nEND_RESPONSE"

    # ---------- move / yaw ------------------------------------------
    def _send_ned_velocity(self, vx, vy, vz, duration):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000_1111_1100_0111, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)
        for _ in range(int(duration * 10)):
            self.vehicle.send_mavlink(msg)
            time.sleep(0.1)

    def move(self,
             direction: Union[str, Tuple[float, float, float]] = "FWD",
             distance: Optional[float] = None,
             velocity: float = 1.0,
             duration: Optional[float] = None,
             block=True) -> str:

        aliases = {"FWD": (1,0,0), "BACK": (-1,0,0),
                   "RIGHT": (0,1,0), "LEFT": (0,-1,0),
                   "UP": (0,0,-1), "DOWN": (0,0,1)}
        vec = aliases.get(direction.upper(), direction) if isinstance(direction, str) else direction
        mag = (vec[0]**2 + vec[1]**2 + vec[2]**2) ** .5
        if mag == 0:
            return "Zero‑length vector\nEND_RESPONSE"
        unit = tuple(v/mag for v in vec)
        if duration is None:
            distance = distance or 2.0
            duration = distance / velocity
        vx, vy, vz = (v*velocity for v in unit)
        t = threading.Thread(target=self._send_ned_velocity,
                             args=(vx, vy, vz, duration), daemon=True)
        t.start()
        if block: t.join()
        return (f"Moving {direction} {velocity} m/s for {duration:.1f}s\n"
                "END_RESPONSE")

    def yaw_drone(self, direction: str):
        angle = 15
        if direction.upper() in {"YAWLEFT", "LEFT"}:
            condition_yaw(self.vehicle, -angle, True)
            return "Yawed left\nEND_RESPONSE"
        if direction.upper() in {"YAWRIGHT", "RIGHT"}:
            condition_yaw(self.vehicle, angle, True)
            return "Yawed right\nEND_RESPONSE"
        return "Unknown yaw dir\nEND_RESPONSE"

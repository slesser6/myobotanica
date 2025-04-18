"""
flightComputer.fc.mavlink_fc
MAVLink / DroneKit wrapper used by the rest of the system.
"""
from __future__ import annotations
from typing import Union, Tuple, Optional
from math import degrees, radians, cos, sin, sqrt, atan2               
import logging, time, io, threading
import atexit, signal, sys, time, json

from datetime import datetime

from dronekit import connect, VehicleMode
from pymavlink import mavutil

from .sitl import SitlManager
from flightComputer import config

# ─────────────────────── Helper Methods ─────────────────────────
# ---------------------------------------------------------------------
def condition_yaw(vehicle, heading_deg, relative=True, yaw_speed=0):
    """
    Send a CONDITION_YAW.  heading_deg may be +/‑.
    Positive = clockwise, negative = counter‑clockwise.
    """
    mag  = abs(heading_deg)
    direction = 1 if heading_deg >= 0 else -1      # ❶ choose CW / CCW

    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        mag,                # param1 – **always positive**
        yaw_speed,          # param2
        direction,          # param3 – CW/CCW       ← ❷
        1 if relative else 0,
        0, 0, 0)

    vehicle.send_mavlink(msg)
    vehicle.flush()

# ---------------------------------------------------------------------
def _ned_distance_m(a, b):
    """Euclidean distance in metres between two LocationGlobalRelative objects
    (same alt ref).  Good enough for ~ short distances."""
    dx = a.north - b.north
    dy = a.east  - b.east
    dz = a.down  - b.down
    return sqrt(dx*dx + dy*dy + dz*dz)    

# ---------------------------------------------------------------------
def meters_ned(origin, current):
    """ very light‑weight haversine for ≤ few km
    """
    R = 6378137.0                                # earth radius
    dlat = radians(current.lat - origin.lat)
    dlon = radians(current.lon - origin.lon)
    lat0 = radians(origin.lat)

    north = dlat * R
    east  = dlon * R * cos(lat0)
    down  = origin.alt - current.alt
    return round(north, 1), round(east, 1), round(down, 1)

# ---------------------------------------------------------------------
def _deg(rad: float) -> float:
    """Convert radians to degrees in the 0‑360 range."""
    from math import degrees
    return (degrees(rad) + 360) % 360

# ---------------------------------------------------------------------
direction_map = {
    "MOVEFWD"  : "FWD",
    "MOVEBACK" : "BACK",
    "MOVELEFT" : "LEFT",
    "MOVERIGHT": "RIGHT",
}

# ─────────────────────── FlightController Class ─────────────────────────
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
        
        # register graceful shutdown once per instance
        atexit.register(self.shutdown)
        signal.signal(signal.SIGTERM, self._sig_exit)
        signal.signal(signal.SIGINT,  self._sig_exit)        
        
        self._telem_suspended = threading.Event()
        self._telem_suspended.clear()   # telemetry runs by default        
        
    # ---------------------------------------------------------------------
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
            
        self.home = self.vehicle.location.global_frame  # lat/lon/alt

    # ---------------------------------------------------------------------
    def close(self):
        if self.vehicle:
            self.vehicle.close()
        if self._sitl:
            self._sitl.stop()

    # ---------- public command entry‑point ---------------------------
    # def send_command(self, command: str) -> str | None:
    #     """
    #     Dispatch by prefix (GETSTATE, ARM, MOVE, …).
    #     Returns a multi‑line string ending with END_RESPONSE
    #     or None on unknown command.
    #     """
    #     parts = command.strip().split(":")
    #     cmd = parts[0].upper()
    #     rest = parts[1:]

    #     cmd = cmd.upper()          # make sure we compare upper‑case
        
    #     # special commands to pause/resume telemetry on demand
    #     if cmd == "STREAMOFF":
    #         self._telem_suspended.set()
    #         return "Stream paused.\nEND_RESPONSE"
    #     if cmd == "STREAMON":
    #         self._telem_suspended.clear()
    #         return "Stream resumed.\nEND_RESPONSE"   
        
    #             # for anything other than GETSTATE, suspend telemetry
    #     do_suspend = (cmd != "GETSTATE")
    #     if do_suspend:
    #         self._telem_suspended.set()

    #     try:     
        
    #         # always let GETSTATE run through without suspending
    #         if cmd != "GETSTATE":
    #             self._telem_suspended.set()   # pause the stream        

    #         if cmd == "GETSTATE":
    #             return self.get_state()

    #         elif cmd == "ARM":
    #             return self.arm_drone()

    #         elif cmd == "TAKEOFF":
    #             target_alt = float(rest[0]) if rest else 2.0
    #             return self.takeoff_drone(target_alt)

    #         elif cmd == "MOVE":
    #             return self.move(rest[0] if rest else "FWD")


    def send_command(self, command: str) -> Optional[str]:
        """
        Dispatch by prefix (GETSTATE, ARM, MOVE, …).
        Returns a multi‑line string ending with END_RESPONSE
        or None on unknown command.
        """        
        
        parts = command.strip().split(":")
        cmd   = parts[0].upper()
        rest  = parts[1:]

        # 1) Handle the two “stream control” pseudo‑commands
        if cmd == "STREAMOFF":
            self._telem_suspended.set()
            return "Stream paused.\nEND_RESPONSE"
        if cmd == "STREAMON":
            self._telem_suspended.clear()
            return "Stream resumed.\nEND_RESPONSE"

        # 2) Any other command _except_ GETSTATE should suspend telemetry
        if cmd != "GETSTATE":
            self._telem_suspended.set()

        # try:
        # 3) GETSTATE always runs, and _then_ we resume telemetry
        if cmd == "GETSTATE":
            resp = self.get_state()
            # resume now that we’ve served the one‑off request
            self._telem_suspended.clear()
            return resp

        # 4) All your other handlers…
        if cmd == "ARM":
            return self.arm_drone()
        elif cmd == "TAKEOFF":
            alt = float(rest[0]) if rest else 2.0
            return self.takeoff_drone(alt)
        elif cmd == "MOVE":
            return self.move(rest[0] if rest else "FWD")
        elif cmd in direction_map:
            return self.move(direction_map[cmd])

        elif cmd == "YAW":
            return self.yaw_drone(rest[0] if rest else "")

        elif cmd in ("YAWLEFT", "YAWRIGHT"):
            return self.yaw_drone(cmd)

        elif cmd == "LAND":
            return self.land_drone()         

        else:
            self.log.warning("Unknown FC cmd %s", command)
            return None
        
        # finally:
        #     # **Crucially**: do _not_ clear telemetry here unconditionally.
        #     # telemetry will stay paused until a GETSTATE or STREAMON arrives.
        #     pass

    # ---------- state ------------------------------------------------
    def get_state_dict(self) -> dict:          # NEW  ←  called by telemetry
        v = self.vehicle
        return {
            "timestamp":   time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "firmware":    str(v.version),
            "mode":        v.mode.name,
            "armed":       bool(v.armed),
            "is_armable":  bool(v.is_armable),
            "battery": {
                "voltage": v.battery.voltage,
                "current": v.battery.current,
                "level":   v.battery.level,
            },
            "gps": {
                "fix_type":     v.gps_0.fix_type,
                "sats_visible": v.gps_0.satellites_visible,
                "hdop":         v.gps_0.eph,
                "vdop":         v.gps_0.epv,
            },
            "ekf_ok":     bool(v.ekf_ok) if hasattr(v, "ekf_ok") else None,
            "location": {
                "lat":  v.location.global_frame.lat,
                "lon":  v.location.global_frame.lon,
                "alt":  v.location.global_frame.alt,
                "relN": v.location.local_frame.north,
                "relE": v.location.local_frame.east,
                "relD": v.location.local_frame.down,
            },
            "velocity": {
                "vx": v.velocity[0], "vy": v.velocity[1], "vz": v.velocity[2],
            },
            "attitude": {
                "roll":  _deg(v.attitude.roll),
                "pitch": _deg(v.attitude.pitch),
                "yaw":   _deg(v.attitude.yaw),
            },
        }
    
    # ---------------------------------------------------------------------       
    def get_state(self) -> str:                # overwrite the old one
        """
        Human readable one‑shot request (still used by FC:GETSTATE).
        """
        blob = json.dumps(self.get_state_dict(), indent=2)
        return blob + "\nEND_RESPONSE"

    # ---------------------------------------------------------------------    

    def start_telemetry(self, radio, period: float = 0.2) -> None:
        if hasattr(self, "_tele_th") and self._tele_th.is_alive():
            return
        stop = threading.Event()

        def _loop():
            while not stop.is_set():
                if not self._telem_suspended.is_set():
                    blob = json.dumps(self.get_state_dict()) + "\n"
                    radio.send(blob)
                stop.wait(period)

        self._tele_th   = threading.Thread(target=_loop, daemon=True)
        self._tele_stop = stop
        self._tele_th.start()

    def stop_telemetry(self):
        if hasattr(self, "_tele_stop"):
            self._tele_stop.set()

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
        while not v.is_armable and time.time() - start < (timeout):
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
        self.log.info("Beginning takeoff procedure")
        if not self.vehicle.armed:
            arm_msg = self.arm_drone()
            if "Failed" in arm_msg:
                return arm_msg
        self.log.info("Drone armed, attempting takeoff")
        self.vehicle.simple_takeoff(alt)
        start = time.time()
        while time.time() - start < timeout:
            self.log.info(f"Altitude: {self.vehicle.location.global_relative_frame.alt}")
            if self.vehicle.location.global_relative_frame.alt >= alt * 0.95:
                return f"Reached {alt} m\nEND_RESPONSE"
            time.sleep(1)
        return f"Timeout reaching {alt} m\nEND_RESPONSE"

    # def land_drone(self):
    #     self.vehicle.mode = VehicleMode("LAND")
    #     return "Drone is landing.\nEND_RESPONSE"
    
    def land_drone(self, timeout: float = 30.0) -> str:
        """
        Command the drone to land, wait until it actually disarms (i.e. touches down),
        and report back the approximate touchdown speed, final altitude, and confirmation.
        """
        v = self.vehicle
        self.log.info("→ Switching to LAND mode")
        v.mode = VehicleMode("LAND")

        start_time    = time.time()
        last_alt      = v.location.global_relative_frame.alt
        descent_rate  = 0.0

        # As long as we're within the timeout...
        while time.time() - start_time < timeout:
            alt = v.location.global_relative_frame.alt
            vz  = v.velocity[2]            # NED frame: positive = downwards
            descent_rate = abs(vz)
            last_alt     = alt

            # log altitude & rate at INFO level so you see it in your RPi journal
            self.log.info("Landing… altitude=%.2f m, descent_rate=%.2f m/s", alt, descent_rate)

            # once disarmed we know we're on the ground
            if not v.armed:
                self.log.info("✅ Vehicle disarmed — touchdown confirmed")
                break

            time.sleep(0.5)
        else:
            # timeout expired
            self.log.warning("❌ land_drone timed out before disarm")

        # build a human‑readable response for the GCS
        if v.armed:
            return (
                f"Landing timed out—vehicle still armed after {timeout:.0f}s\n"
                "END_RESPONSE"
            )
        else:
            return (
                f"Drone landed successfully.\n"
                f"Final altitude: {last_alt:.2f} m\n"
                f"Touchdown speed ≈ {descent_rate:.2f} m/s\n"
                "END_RESPONSE"
            )


    # ────────────────────────── movement ────────────────────────────────
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
            block: bool = True) -> str:

        aliases = {
            "FWD":  ( 1, 0, 0), "BACK": (-1, 0, 0),
            "RIGHT":( 0, 1, 0), "LEFT": ( 0,-1, 0),
            "UP":   ( 0, 0,-1), "DOWN": ( 0, 0, 1),
        }

        # ---------- resolve direction -----------------------------------
        vec = aliases.get(direction.upper(), direction) if isinstance(direction, str) else direction
        if not (isinstance(vec, tuple) and len(vec) == 3):
            return f"Bad direction '{direction}'.\nEND_RESPONSE"

        mag = sqrt(sum(v*v for v in vec))
        if mag == 0:
            return "Zero‑length vector\nEND_RESPONSE"
        unit = tuple(v/mag for v in vec)

        # ---------- timing ----------------------------------------------
        if duration is None:
            distance  = distance or 2.0          # default 2 m
            duration  = distance / velocity

        vx, vy, vz = (v*velocity for v in unit)

        # ---------- snapshot before -------------------------------------
        start_loc  = self.vehicle.location.local_frame  # NED in metres
        start_yaw  = degrees(self.vehicle.attitude.yaw)

        self.log.info("MOVE %s  %s m/s  %.1fs  (vx %.2f vy %.2f vz %.2f)",
                    direction, velocity, duration, vx, vy, vz)

        worker = threading.Thread(target=self._send_ned_velocity,
                                args=(vx, vy, vz, duration),
                                daemon=True)
        worker.start()
        if block:
            worker.join()

        # ---------- snapshot after --------------------------------------
        end_loc    = self.vehicle.location.local_frame
        end_yaw    = degrees(self.vehicle.attitude.yaw)
        travelled  = _ned_distance_m(start_loc, end_loc)

        self.log.info("MOVE complete travelled ≈ %.2f m", travelled)

        return (f"MOVE OK\n"
                f"Travelled ≈ {travelled:.2f} m @ {velocity:.1f} m/s\n"
                f"Start NED: (N {start_loc.north:+.1f}  E {start_loc.east:+.1f}  D {start_loc.down:+.1f})\n"
                f"End   NED: (N {end_loc.north:+.1f}  E {end_loc.east:+.1f}  D {end_loc.down:+.1f})\n"
                "END_RESPONSE")

    # ───────────────────────────── yaw ──────────────────────────────────
    def yaw_drone(self, direction: str, angle: float = 15) -> str:
        dir_up = direction.upper()
        if dir_up not in {"YAWLEFT", "LEFT", "YAWRIGHT", "RIGHT"}:
            return f"Unknown yaw dir '{direction}'\nEND_RESPONSE"

        before = degrees(self.vehicle.attitude.yaw)
        if dir_up in {"YAWLEFT", "LEFT"}:
            condition_yaw(self.vehicle, -angle, True)
            dir_str = "left"
        else:
            condition_yaw(self.vehicle,  angle, True)
            dir_str = "right"

        # give EKF a moment to settle
        time.sleep(2.0)
        after  = degrees(self.vehicle.attitude.yaw)
        delta  = (after - before + 540) % 360 - 180   # shortest signed diff

        self.log.info("YAW %s %d°, actual Δ=%.1f°  new heading %.1f°",
                    dir_str, angle, delta, after)

        return (f"YAW {dir_str.capitalize()} complete\n"
                f"Before: {before:.1f}°  After: {after:.1f}°  Δ≈{delta:.1f}°\n"
                "END_RESPONSE")

    def _sig_exit(self, *a):
        """Handler for SIGINT / SIGTERM sent by Ctrl‑C or systemd."""
        self.shutdown()
        sys.exit(0)

    # ──────────────────────────────────────────────────────────────────
    def shutdown(self):
        """Close MAVLink socket and stop SITL if it’s running."""
        
        self.stop_telemetry()
        
        if getattr(self, "vehicle", None):
            try:
                self.vehicle.close()
            except Exception:
                pass

        if getattr(self, "sitl", None):
            try:
                self.sitl.stop()      # tells simulator to exit
                # wait a moment so the tcp:5760 listener disappears
                time.sleep(1)
            except Exception:
                pass
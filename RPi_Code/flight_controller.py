# flight_controller.py
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import io
import logging

class FlightController:
    def __init__(self, connection_string='/dev/ttyAMA0', baud_rate=115200):
        self.connection_string = connection_string
        self.baud_rate = baud_rate
        self.vehicle = None

    def connect(self):
        print(f"Connecting to flight controller on {self.connection_string} at {self.baud_rate} baud...")
        self.vehicle = connect(self.connection_string, wait_ready=True, baud=self.baud_rate)
        print("Drone connected!")
        print(f" Firmware: {self.vehicle.version}")
        print(f" Location: {self.vehicle.location.global_relative_frame}")

    def send_command(self, command):
        # Split command by colon to identify sub-commands or parameters.
        parts = command.strip().split(":")
        cmd = parts[0].upper()

        if cmd == "GETSTATE":
            return self.get_state()
        elif cmd == "ARM":
            return self.arm_drone(timeout=10)
        elif cmd == "TAKEOFF":
            # Use provided altitude if available; otherwise, use a default (e.g., 5.0 meters)
            if len(parts) > 1 and parts[1]:
                alt = float(parts[1])
            else:
                alt = 5.0  # Default altitude (adjust this value as needed)
            return self.takeoff_drone(alt, timeout=10)
        elif cmd == "MOVE":
            # Allow legacy command format: e.g., "FC:MOVE:FWD"
            if len(parts) > 1:
                direction = parts[1].upper()
                return self.move_drone(direction)
            else:
                return "No move direction provided.\nEND_RESPONSE"
        elif cmd in ["MOVEFWD", "MOVEBACK"]:
            # New commands with no parameters.
            return self.move_drone(cmd)
        elif cmd == "YAW":
            # Allow the old format: e.g., "FC:YAW:LEFT"
            if len(parts) > 1:
                direction = parts[1].upper()
                return self.yaw_drone(direction)
            else:
                return "No yaw direction provided.\nEND_RESPONSE"
        elif cmd in ["YAWLEFT", "YAWRIGHT"]:
            # New yaw commands without needing a parameter.
            return self.yaw_drone(cmd)
        elif cmd == "LAND":
            return self.land_drone()
        elif cmd.startswith("PLAYTUNE"):
            tune = cmd.replace("PLAYTUNE", "").strip()
            return self.play_tune(tune)
        else:
            print(f"Unknown FC command: {command}")
            return None


    def get_state(self):
        print("Querying flight controller for state...")
        state_info = (
            f"Drone state:\n"
            f"  Firmware: {self.vehicle.version}\n"
            f"  Location: {self.vehicle.location.global_relative_frame}\n"
            f"  Battery: {self.vehicle.battery}\n"
            f"  Mode: {self.vehicle.mode.name}\n"
            f"  Armed: {self.vehicle.armed}"
        )
        # Append end-of-response marker
        return state_info + "\nEND_RESPONSE"

    
    def arm_drone(self, timeout=10):
        """
        Attempts to arm the drone in GUIDED mode.
        Captures CRITICAL logging messages during arming.
        Returns a multi-line string with status and any warnings, terminated by END_RESPONSE.
        """
        # Create an in-memory stream and attach a handler to capture CRITICAL logs.
        log_capture = io.StringIO()
        ch = logging.StreamHandler(log_capture)
        ch.setLevel(logging.CRITICAL)
        logger = logging.getLogger()  # Capture from the root logger.
        logger.addHandler(ch)

        messages = []
        messages.append("Arming drone...")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        start_time = time.time()
        while not self.vehicle.armed and (time.time() - start_time) < timeout:
            msg = "Waiting for arming..."
            print(msg)
            messages.append(msg)
            time.sleep(1)

        # Remove the temporary logging handler.
        logger.removeHandler(ch)
        # Retrieve any CRITICAL messages captured during arming.
        captured_logs = log_capture.getvalue()
        if captured_logs:
            messages.append("Captured warnings:")
            messages.append(captured_logs.strip())

        if not self.vehicle.armed:
            messages.append(f"Failed to arm within {timeout} seconds.")
        else:
            messages.append("Drone is ARMED.")

        messages.append("END_RESPONSE")
        return "\n".join(messages)

    
    def takeoff_drone(self, altitude, timeout=10):
        '''
        Function to have drone takeoff to a specified height [m]. Drone will attempt to arm itself if not armed. 
        '''
        
        responses = []
        # If not armed, attempt to arm.
        if not self.vehicle.armed:
            arm_msg = self.arm_drone(timeout=timeout)
            responses.append(arm_msg)
            # If arming failed, return immediately.
            if "failed" in arm_msg.lower():
                responses.append("END_RESPONSE")
                return "\n".join(responses)
        
        responses.append(f"Taking off to {altitude}m...")
        self.vehicle.simple_takeoff(altitude)

        start_time = time.time()
        # Wait until altitude is reached or timeout expires.
        while time.time() - start_time < timeout:
            current_alt = self.vehicle.location.global_relative_frame.alt
            responses.append(f"Altitude: {current_alt:.1f}m")
            if current_alt >= altitude * 0.95:
                responses.append(f"Reached target altitude of {altitude}m.")
                break
            time.sleep(1)
        else:
            responses.append(f"Timeout: did not reach {altitude}m within {timeout} seconds.")

        responses.append("END_RESPONSE")
        return "\n".join(responses)
    
    
    def move_drone(self, direction):
        # Pre-programmed constants for movement.
        DEFAULT_MOVE_VELOCITY = 1    # meters per second
        DEFAULT_MOVE_DURATION = 2    # seconds (move approximately 2 meters)

        # Check that the drone is armed.
        if not self.vehicle.armed:
            return "Drone is not armed.\nEND_RESPONSE"

        # Check if the drone is in GUIDED mode; if not, attempt to change.
        if self.vehicle.mode.name.upper() != "GUIDED":
            print("Drone is not in GUIDED mode. Attempting to change mode to GUIDED...")
            self.vehicle.mode = VehicleMode("GUIDED")
            # Wait up to 3 seconds for mode change.
            wait_time = 3
            start_time = time.time()
            while self.vehicle.mode.name.upper() != "GUIDED" and (time.time() - start_time) < wait_time:
                time.sleep(0.5)
            if self.vehicle.mode.name.upper() != "GUIDED":
                return "Failed to change mode to GUIDED. Cannot move.\nEND_RESPONSE"
            else:
                print("Drone mode changed to GUIDED.")

        # Helper function to send a velocity command in the NED frame.
        def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,      # time_boot_ms (not used)
                0, 0,   # target system, target component
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
                0b0000111111000111,  # type_mask (only velocity enabled)
                0, 0, 0,             # x, y, z positions (not used)
                velocity_x, velocity_y, velocity_z,  # velocity components in m/s
                0, 0, 0,             # acceleration (not used)
                0, 0)                # yaw, yaw_rate (not used)
            for _ in range(duration):
                self.vehicle.send_mavlink(msg)
                time.sleep(1)

        # Determine move command based on the direction input.
        if direction.upper() in ["MOVEFWD", "FWD"]:
            print(f"Moving forward for {DEFAULT_MOVE_DURATION} seconds at {DEFAULT_MOVE_VELOCITY} m/s...")
            send_ned_velocity(DEFAULT_MOVE_VELOCITY, 0, 0, duration=DEFAULT_MOVE_DURATION)
            return "Drone moved forward.\nEND_RESPONSE"
        elif direction.upper() in ["MOVEBACK", "BACK"]:
            print(f"Moving backward for {DEFAULT_MOVE_DURATION} seconds at {DEFAULT_MOVE_VELOCITY} m/s...")
            send_ned_velocity(-DEFAULT_MOVE_VELOCITY, 0, 0, duration=DEFAULT_MOVE_DURATION)
            return "Drone moved backward.\nEND_RESPONSE"
        else:
            return f"Unknown MOVE command: {direction}\nEND_RESPONSE"

                
    def yaw_drone(self, direction):
        # Pre-programmed yaw angle (in degrees)
        DEFAULT_YAW_ANGLE = 15

        # Check that the drone is armed.
        if not self.vehicle.armed:
            return "Drone is not armed.\nEND_RESPONSE"
        
        # Check if the drone is in GUIDED mode; if not, attempt to change.
        if self.vehicle.mode.name.upper() != "GUIDED":
            print("Drone is not in GUIDED mode. Attempting to change mode to GUIDED...")
            self.vehicle.mode = VehicleMode("GUIDED")
            # Wait up to 3 seconds for mode change.
            wait_time = 3
            start_time = time.time()
            while self.vehicle.mode.name.upper() != "GUIDED" and (time.time() - start_time) < wait_time:
                time.sleep(0.5)
            if self.vehicle.mode.name.upper() != "GUIDED":
                return "Failed to change mode to GUIDED. Cannot yaw.\nEND_RESPONSE"
            else:
                print("Drone mode changed to GUIDED.")

        # Determine yaw angle based on direction.
        # Accept both full commands and the simple LEFT/RIGHT tokens.
        if direction.upper() in ["YAWLEFT", "LEFT"]:
            print(f"Yawing left {DEFAULT_YAW_ANGLE} degrees...")
            self.vehicle.condition_yaw(-DEFAULT_YAW_ANGLE, relative=True)
            return "Drone yawed left.\nEND_RESPONSE"
        elif direction.upper() in ["YAWRIGHT", "RIGHT"]:
            print(f"Yawing right {DEFAULT_YAW_ANGLE} degrees...")
            self.vehicle.condition_yaw(DEFAULT_YAW_ANGLE, relative=True)
            return "Drone yawed right.\nEND_RESPONSE"
        else:
            return f"Unknown YAW command: {direction}\nEND_RESPONSE"

    def land_drone(self):
        print("Landing the drone...")
        self.vehicle.mode = VehicleMode("LAND")
        return "Drone is landing.\nEND_RESPONSE"

    def play_tune(self, tune):
        print(f"Playing tune on FC (simulated): {tune}")
        return f"Playing tune: {tune}\nEND_RESPONSE"

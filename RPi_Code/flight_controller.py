# # flight_controller.py
# from dronekit import connect, VehicleMode, LocationGlobalRelative
# from pymavlink import mavutil
# import dronekit_sitl
# from dronekit_sitl import SITL
# import time
# import io
# import logging
# from typing import Union, Tuple, Optional
# import threading           # you use threading later

  
# def condition_yaw(vehicle, heading, relative=True, yaw_speed=0):
#     """
#     Send MAV_CMD_CONDITION_YAW message to set a new heading.
    
#     Args:
#         vehicle: The DroneKit Vehicle instance.
#         heading: Target yaw in degrees.
#         relative: If True, the heading is relative to the current yaw. If False, it’s absolute.
#         yaw_speed: (Optional) Yaw speed in deg/s (0 lets autopilot auto-calculate).
#     """
#     if relative:
#         is_relative = 1  # Yaw relative to current heading
#     else:
#         is_relative = 0  # Absolute angle
    
#     msg = vehicle.message_factory.command_long_encode(
#         0, 0,                                # target system, target component
#         mavutil.mavlink.MAV_CMD_CONDITION_YAW,# command
#         0,                                   # confirmation
#         heading,                             # param 1: target angle in degrees
#         yaw_speed,                           # param 2: speed (deg/s)
#         1,                                   # param 3: direction (-1: counterclockwise, 1: clockwise), 1 is common if heading is relative
#         is_relative,                         # param 4: relative offset (1) or absolute (0)
#         0, 0, 0)                             # param 5-7: not used
    
#     vehicle.send_mavlink(msg)
#     vehicle.flush()     
   

# class FlightController:
#     def __init__(self, connection_string='/dev/ttyAMA0', baud_rate=115200, simulation_mode=False):
#         """
#         Args:
#             connection_string (str): The string for connecting to the flight controller.
#             baud_rate (int): Baud rate for the connection.
#             simulation_mode (bool): If True, launches SITL and connects via its URL.
#         """
#         self.simulation_mode = simulation_mode
#         self.connection_string = connection_string
#         self.baud_rate = baud_rate
#         self.vehicle = None
#         self.sitl = None  # Will hold the SITL instance if simulation_mode is enabled

#     def connect(self):
#         # If simulation mode is enabled, start DroneKit SITL
#         if self.simulation_mode:
#             print("Starting DroneKit SITL simulation...")
#             try:
#                 from dronekit_sitl import start_default
#             except ImportError as e:
#                 print("Error: dronekit_sitl is not installed.")
#                 raise e

#             # Create a SITL instance
#             self.sitl = SITL(path="/home/OctoDronePi/.dronekit/sitl/copter-3.3/apm")
#             # Launch SITL with a home position specified as "lat,lon,alt,yaw"
#             # Replace with your desired values. For example: 37.8719,-122.2585,100,0
#             home_args = ["--home", "37.8719,-122.2585,100,0"]
#             self.sitl.launch(home_args, await_ready=True, restart=True)           
            
            
#             # Update connection_string to the SITL instance's connection URL
#             self.connection_string = self.sitl.connection_string()
#             print("SITL started successfully with connection string:", self.connection_string)
        
#         print(f"Connecting to flight controller on {self.connection_string} at {self.baud_rate} baud...")
#         self.vehicle = connect(self.connection_string, wait_ready=True, baud=self.baud_rate)
#         print("Drone connected!")
#         print(f" Able to arm? {self.vehicle.is_armable}")
#         print(f" Firmware: {self.vehicle.version}")
#         print(f" Location: {self.vehicle.location.global_relative_frame}")
        
#         # print("Frame related parameters:")
#         # for key, value in self.vehicle.parameters.items():
#         #     if "frame" in key.lower() or "mot" in key.lower():
#         #         print(f"{key}: {value}")        
#         if self.simulation_mode:       
#             print("Origional frame parameters:")
#             print("FRAME_CLASS:", self.vehicle.parameters.get('FRAME_CLASS'))
#             print("FRAME_TYPE:", self.vehicle.parameters.get('FRAME_TYPE'))                
                    
#             print("Overriding frame parameters for multicopter setup...")
#             # https://ardupilot.org/copter/docs/parameters.html
#             self.vehicle.parameters['FRAME_CLASS'] = 1  # 1 indicates a quadcopter
#             self.vehicle.parameters['FRAME_TYPE'] = 1   # 1 denotes x-frame
#             time.sleep(2)
#             print("Updated frame parameters:")
#             print("FRAME_CLASS:", self.vehicle.parameters.get('FRAME_CLASS'))
#             print("FRAME_TYPE:", self.vehicle.parameters.get('FRAME_TYPE'))  
                 
#             # After connecting and verifying the initial state:
#             # print("global_relative_frame before update:", self.vehicle.location.global_relative_frame)
#             # set_home_location(self.vehicle, 37.8719, -122.2585, 10)
#             # time.sleep(10)  # Wait a few seconds for the vehicle to update its position estimate

#             print("\nHome location: %s" % self.vehicle.home_location)
#             print("Global Frame", self.vehicle.location.global_frame)
#             print("Current GPS Fix (global_relative_frame):", self.vehicle.location.global_relative_frame)            

#             # Home location must be within 50km of EKF home location (or setting will fail silently)
#             # In this case, just set value to current location with an easily recognisable altitude (222)
#             my_location_alt = self.vehicle.location.global_frame
#             my_location_alt.lat = 37.8719
#             my_location_alt.lon = -122.2585
#             my_location_alt.alt = 100.0
#             self.vehicle.home_location = my_location_alt
#             time.sleep(2)
#             print(" New Home Location (from attribute - altitude should be 100): %s" % self.vehicle.home_location)
            

                                        
#     def send_command(self, command):
#         # Split command by colon to identify sub-commands or parameters.
#         parts = command.strip().split(":")
#         cmd = parts[0].upper()

#         if cmd == "GETSTATE":
#             return self.get_state()
#         elif cmd == "ARM":
#             return self.arm_drone(timeout=20)
#         elif cmd == "TAKEOFF":
#             # Use provided altitude if available; otherwise, use a default (e.g., 5.0 meters)
#             if len(parts) > 1 and parts[1]:
#                 alt = float(parts[1])
#             else:
#                 alt = 2.0  # DEFAULT ALTITUDE [m]
#             return self.takeoff_drone(alt, timeout=10)
#         elif cmd == "MOVE":
#             # Allow legacy command format: e.g., "FC:MOVE:FWD"
#             if len(parts) > 1:
#                 direction = parts[1].upper()
#                 return self.move_drone(direction)
#             else:
#                 return "No move direction provided.\nEND_RESPONSE"
#         elif cmd in ["MOVEFWD", "MOVEBACK"]:
#             # New commands with no parameters.
#             return self.move_drone(cmd)
#         elif cmd == "YAW":
#             # Allow the old format: e.g., "FC:YAW:LEFT"
#             if len(parts) > 1:
#                 direction = parts[1].upper()
#                 return self.yaw_drone(direction)
#             else:
#                 return "No yaw direction provided.\nEND_RESPONSE"
#         elif cmd in ["YAWLEFT", "YAWRIGHT"]:
#             # New yaw commands without needing a parameter.
#             return self.yaw_drone(cmd)
#         elif cmd == "LAND":
#             return self.land_drone()
#         elif cmd.startswith("PLAYTUNE"):
#             tune = cmd.replace("PLAYTUNE", "").strip()
#             return self.play_tune(tune)
#         else:
#             print(f"Unknown FC command: {command}")
#             return None

#     def get_state(self):
#         print("Querying flight controller for state...")
#         state_info = (
#             f"Drone state:\n"
#             f"  Firmware: {self.vehicle.version}\n"
#             f"  Location: {self.vehicle.location.global_relative_frame}\n"
#             f"  Battery: {self.vehicle.battery}\n"
#             f"  Mode: {self.vehicle.mode.name}\n"
#             f"  Armed: {self.vehicle.armed}"
#         )
#         # Append end-of-response marker
#         return state_info + "\nEND_RESPONSE"

#     def arm_drone(self, timeout=10):
#         """
#         Attempts to arm the drone in GUIDED mode.
#         Debugs pre-arm status and optionally disables checks for simulation.
#         """
#         import time

#         # Attach logging to capture critical messages (for debugging purposes).
#         log_capture = io.StringIO()
#         ch = logging.StreamHandler(log_capture)
#         ch.setLevel(logging.CRITICAL)
#         logger = logging.getLogger()
#         logger.addHandler(ch)

#         messages = []
#         messages.append("Arming drone...")

#         # Set the desired mode and allow time for initialization
#         print("Setting mode to GUIDED...")
#         self.vehicle.mode = VehicleMode("GUIDED")
        
#         print("Home Location:", self.vehicle.home_location)
        
#         if self.simulation_mode:
#             # Optionally disable pre-arm checks in simulation (not for real flights)
#             print("Disabling pre-arm checks for simulation (ARMING_CHECK=0)...")
#             self.vehicle.parameters['ARMING_CHECK'] = 0
        
#         time.sleep(2)  # Allow a moment for mode to update
        
#         # Debug logging for sensor and GPS status.
#         print("DEBUG: Initial is_armable:", self.vehicle.is_armable)
#         print("DEBUG: Vehicle mode:", self.vehicle.mode.name)
#         print("DEBUG: Vehicle armed:", self.vehicle.armed)
#         print("DEBUG: GPS Fix:", self.vehicle.gps_0.fix_type)
#         print("DEBUG: GPS Satellites:", self.vehicle.gps_0.satellites_visible)
#         print("FRAME_CLASS:", self.vehicle.parameters.get('FRAME_CLASS'))
#         print("FRAME_TYPE:", self.vehicle.parameters.get('FRAME_TYPE'))
        
#         # Wait until the vehicle becomes armable, with a longer timeout for SITL initialization.
#         start_time = time.time()
#         while not self.vehicle.is_armable and (time.time() - start_time) < timeout:
#             print("DEBUG: Still not armable. is_armable:", self.vehicle.is_armable)
#             time.sleep(1)

#         if not self.vehicle.is_armable:
#             print("DEBUG: Drone not armable after waiting.")
#             logger.removeHandler(ch)
#             return "Drone not armable\nEND_RESPONSE"
        
#         # Try to arm the vehicle.
#         self.vehicle.armed = True
#         arm_start_time = time.time()
#         while not self.vehicle.armed and (time.time() - arm_start_time) < timeout:
#             msg = "Waiting for arming..."
#             print(msg)
#             messages.append(msg)
#             time.sleep(1)

#         logger.removeHandler(ch)
#         captured_logs = log_capture.getvalue()
#         if captured_logs:
#             messages.append("Captured warnings:")
#             messages.append(captured_logs.strip())

#         if not self.vehicle.armed:
#             messages.append(f"Failed to arm within {timeout} seconds.")
#         else:
#             messages.append("Drone is ARMED.")

#         messages.append("END_RESPONSE")
#         return "\n".join(messages)


#     def takeoff_drone(self, altitude, timeout=10):
#         '''
#         Function to have drone takeoff to a specified height [m]. Drone will attempt to arm itself if not armed. 
#         '''
#         responses = []
#         # If not armed, attempt to arm.
#         if not self.vehicle.armed:
#             arm_msg = self.arm_drone(timeout=timeout)
#             responses.append(arm_msg)
#             # If arming failed, return immediately.
#             if "failed" in arm_msg.lower():
#                 responses.append("END_RESPONSE")
#                 return "\n".join(responses)
        
#         responses.append(f"Taking off to {altitude}m...")
#         self.vehicle.simple_takeoff(altitude)

#         start_time = time.time()
#         # Wait until altitude is reached or timeout expires.
#         while time.time() - start_time < timeout:
#             current_alt = self.vehicle.location.global_relative_frame.alt
#             responses.append(f"Altitude: {current_alt:.1f}m")
#             if current_alt >= altitude * 0.95:
#                 responses.append(f"Reached target altitude of {altitude}m.")
#                 break
#             time.sleep(1)
#         else:
#             responses.append(f"Timeout: did not reach {altitude}m within {timeout} seconds.")

#         responses.append("END_RESPONSE")
#         return "\n".join(responses)

#     # def move_drone(self, direction):
#     #     # Pre-programmed constants for movement.
#     #     DEFAULT_MOVE_VELOCITY = 1    # meters per second
#     #     DEFAULT_MOVE_DURATION = 2    # seconds (move approximately 2 meters)

#     #     # Check that the drone is armed.
#     #     if not self.vehicle.armed:
#     #         return "Drone is not armed.\nEND_RESPONSE"

#     #     # Check if the drone is in GUIDED mode; if not, attempt to change.
#     #     if self.vehicle.mode.name.upper() != "GUIDED":
#     #         print("Drone is not in GUIDED mode. Attempting to change mode to GUIDED...")
#     #         self.vehicle.mode = VehicleMode("GUIDED")
#     #         # Wait up to 3 seconds for mode change.
#     #         wait_time = 3
#     #         start_time = time.time()
#     #         while self.vehicle.mode.name.upper() != "GUIDED" and (time.time() - start_time) < wait_time:
#     #             time.sleep(0.5)
#     #         if self.vehicle.mode.name.upper() != "GUIDED":
#     #             return "Failed to change mode to GUIDED. Cannot move.\nEND_RESPONSE"
#     #         else:
#     #             print("Drone mode changed to GUIDED.")

#     #     # Helper function to send a velocity command in the NED frame.
#     #     def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
#     #         msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
#     #             0,      # time_boot_ms (not used)
#     #             0, 0,   # target system, target component
#     #             mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
#     #             0b0000111111000111,  # type_mask (only velocity enabled)
#     #             0, 0, 0,             # x, y, z positions (not used)
#     #             velocity_x, velocity_y, velocity_z,  # velocity components in m/s
#     #             0, 0, 0,             # acceleration (not used)
#     #             0, 0)                # yaw, yaw_rate (not used)
#     #         for _ in range(duration):
#     #             self.vehicle.send_mavlink(msg)
#     #             time.sleep(1)

#     #     # Determine move command based on the direction input.
#     #     if direction.upper() in ["MOVEFWD", "FWD"]:
#     #         print(f"Moving forward for {DEFAULT_MOVE_DURATION} seconds at {DEFAULT_MOVE_VELOCITY} m/s...")
#     #         send_ned_velocity(DEFAULT_MOVE_VELOCITY, 0, 0, duration=DEFAULT_MOVE_DURATION)
#     #         return "Drone moved forward.\nEND_RESPONSE"
#     #     elif direction.upper() in ["MOVEBACK", "BACK"]:
#     #         print(f"Moving backward for {DEFAULT_MOVE_DURATION} seconds at {DEFAULT_MOVE_VELOCITY} m/s...")
#     #         send_ned_velocity(-DEFAULT_MOVE_VELOCITY, 0, 0, duration=DEFAULT_MOVE_DURATION)
#     #         return "Drone moved backward.\nEND_RESPONSE"
#     #     else:
#     #         return f"Unknown MOVE command: {direction}\nEND_RESPONSE"

#     def _send_ned_velocity(self, vx, vy, vz, duration):
#         """
#         Burst a constant‑velocity NED command for *duration* seconds.
#         Positive X = forward, positive Y = right, positive Z = down.
#         """
#         msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
#             0, 0, 0,
#             mavutil.mavlink.MAV_FRAME_LOCAL_NED,
#             0b0000_1111_1100_0111,      # velocity only
#             0, 0, 0,
#             vx, vy, vz,
#             0, 0, 0,
#             0, 0
#         )
#         for _ in range(int(duration * 10)):       # 10 Hz burst
#             self.vehicle.send_mavlink(msg)
#             time.sleep(0.1)

#     def move(
#             self,
#             direction: Union[str, Tuple[float, float, float]] = "FWD",
#             distance: Optional[float] = None,   # metres
#             velocity: float = 1.0,              # m/s
#             duration: Optional[float] = None,   # seconds
#             block: bool = True
#         ) -> str:
#         """
#         Move in the given direction for *distance* OR *duration*.

#         direction – string alias ("FWD", "BACK", "LEFT", "RIGHT", "UP", "DOWN")
#                     or a raw NED tuple (vx, vy, vz) that will be normalised.
#         distance  – metres to travel. If supplied and duration is not,
#                     duration = distance / velocity
#         velocity  – magnitude of velocity vector (m/s)  (ignored for raw vx/vy/vz)
#         duration  – seconds to command. If both distance and duration are given
#                     duration wins.
#         block     – wait until command burst completes before returning.
#         """
#         if not self.vehicle.armed:
#             return "Drone is not armed.\nEND_RESPONSE"

#         # ----- resolve direction into a unit vector -----
#         ALIASES = {
#             "FWD":  ( 1,  0,  0),
#             "BACK": (-1,  0,  0),
#             "RIGHT":( 0,  1,  0),
#             "LEFT": ( 0, -1,  0),
#             "UP":   ( 0,  0, -1),
#             "DOWN": ( 0,  0,  1),
#         }
#         if isinstance(direction, str):
#             try:
#                 vec = ALIASES[direction.upper()]
#             except KeyError:
#                 return f"Unknown direction '{direction}'.\nEND_RESPONSE"
#         else:
#             vec = direction
#             mag = (vec[0]**2 + vec[1]**2 + vec[2]**2) ** .5
#             if mag == 0:
#                 return "Zero‑length direction vector.\nEND_RESPONSE"
#             vec = tuple(v / mag for v in vec)

#         # ----- derive duration if needed -----
#         if duration is None:
#             if distance is None:
#                 distance = 2.0              # built‑in default
#             duration = distance / velocity

#         # scale velocity vector
#         vx, vy, vz = (v * velocity for v in vec)

#         # ensure GUIDED
#         if self.vehicle.mode.name.upper() != "GUIDED":
#             self.vehicle.mode = VehicleMode("GUIDED")
#             time.sleep(0.5)

#         # fire and (optionally) wait
#         worker = threading.Thread(target=self._send_ned_velocity,
#                                 args=(vx, vy, vz, duration), daemon=True)
#         worker.start()
#         if block:
#             worker.join()

#         return (f"Moving {direction} at {velocity:.1f} m/s for {duration:.1f}s "
#                 f"({vx:.1f},{vy:.1f},{vz:.1f}).\nEND_RESPONSE")
        
#     # Sane defaults that CLI / GUI can list
#     def move_fwd(self, metres: float = 2.0):     # keep old behaviour
#         return self.move("FWD", distance=metres)

#     def move_back(self, metres: float = 2.0):
#         return self.move("BACK", distance=metres)

#     def move_left(self, metres: float = 2.0):
#         return self.move("LEFT", distance=metres)

#     def move_right(self, metres: float = 2.0):
#         return self.move("RIGHT", distance=metres)
        

#     def yaw_drone(self, direction):
#         DEFAULT_YAW_ANGLE = 15

#         if not self.vehicle.armed:
#             return "Drone is not armed.\nEND_RESPONSE"

#         if self.vehicle.mode.name.upper() != "GUIDED":
#             print("Drone is not in GUIDED mode. Attempting to change mode to GUIDED...")
#             self.vehicle.mode = VehicleMode("GUIDED")
#             wait_time = 3
#             start_time = time.time()
#             while self.vehicle.mode.name.upper() != "GUIDED" and (time.time() - start_time) < wait_time:
#                 time.sleep(0.5)
#             if self.vehicle.mode.name.upper() != "GUIDED":
#                 return "Failed to change mode to GUIDED. Cannot yaw.\nEND_RESPONSE"
#             else:
#                 print("Drone mode changed to GUIDED.")

#         if direction.upper() in ["YAWLEFT", "LEFT"]:
#             print(f"Yawing left {DEFAULT_YAW_ANGLE} degrees...")
#             condition_yaw(self.vehicle, -DEFAULT_YAW_ANGLE, relative=True)
#             return "Drone yawed left.\nEND_RESPONSE"
#         elif direction.upper() in ["YAWRIGHT", "RIGHT"]:
#             print(f"Yawing right {DEFAULT_YAW_ANGLE} degrees...")
#             condition_yaw(self.vehicle, DEFAULT_YAW_ANGLE, relative=True)
#             return "Drone yawed right.\nEND_RESPONSE"
#         else:
#             return f"Unknown YAW command: {direction}\nEND_RESPONSE"

#     def land_drone(self):
#         print("Landing the drone...")
#         self.vehicle.mode = VehicleMode("LAND")
#         return "Drone is landing.\nEND_RESPONSE"

#     def play_tune(self, tune):
#         print(f"Playing tune on FC (simulated): {tune}")
#         return f"Playing tune: {tune}\nEND_RESPONSE"

#     def close(self):
#         """
#         Optionally call this method to shut down SITL (if running) and clean up the connection.
#         """
#         if self.vehicle:
#             self.vehicle.close()
#         if self.sitl:
#             print("Stopping SITL simulation...")
#             self.sitl.stop()

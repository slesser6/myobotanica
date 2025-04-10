import serial 
import time
class Drone:
    def __init__(self, drone_cfg, motion_cfg):
        self.baud_rate = drone_cfg.baud_rate
        self.port = drone_cfg.port
        self.takeoff_alt = drone_cfg.takeoff_alt
        self.speed = motion_cfg.speed
        self.debug_mode = motion_cfg.debug_mode
        self.ser = None

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=0.5)
            print(f"\n[Drone] Connected to telemetry radio on {self.port} at {self.baud_rate} baud.")
        except Exception as e:
            print(f"\n[Drone] Error opening serial port {self.port}: {e}")
            return
        
        i = 0
        while not self.sendCommand("FC:GETSTATE\n", 3, True, "END_RESPONSE").is_armable and i < 10:
            time.sleep(1)
            i += 1
        if self.sendCommand("FC:GETSTATE\n", 3, True, "END_RESPONSE").is_armable:
            print(self.sendCommand(self.drone.sendCommand("FC:ARM\n", 10, True, "END_RESPONSE")))
        else:
            print("\n[DRONE] Drone is not armable.")

    def sendCommand(self, cmd, wait_time=2, expect_multi=False, end_keyword=None):
        """
        Sends a command to the RPi and prints all responses until either wait_time expires
        or an end_keyword is detected.
        
        Args:
            command (str): The command string to send (with trailing newline).
            wait_time (float): Maximum seconds to wait if no end_keyword is detected.
            expect_multi (bool): If True, continues reading lines until wait_time expires.
            end_keyword (str): If provided, reading stops when a line exactly matches this keyword.
        
        Returns:
            List[str]: A list of response lines received (excluding the end marker).
        """
        if not self.ser:
            print("\n[Drone] Connect to telemetry radio before sending commands") 
        
        # Flush any stale data before sending
        self.ser.reset_input_buffer()
        if self.debug_mode:
            while True:
                answer = input(f"\n[Drone] Would you like to send this command: {cmd.strip()}? y/n: ").strip().lower()
                if answer == 'y':
                    break
                elif answer == 'n':
                    print(f"\n[Drone] Not sending")
                    return
                else:
                    print("[Drone] Invalid input. Please enter 'y' or 'n'.")

        print(f"\n[Drone] Sending command to RPi: {cmd.strip()}")
        self.ser.write(cmd.encode())

        responses = []
        end_time = time.time() + wait_time
        while True:
            if self.ser.in_waiting:
                line = self.ser.readline().decode().strip()
                # Check if we've reached the end marker
                if end_keyword and line == end_keyword:
                    break
                print(line)
                responses.append(line)
                # If not expecting multi-line responses, break after one line.
                if not expect_multi:
                    break
            # If no data is arriving and we've passed the wait_time, break out.
            if time.time() > end_time:
                break
            time.sleep(0.1)
        if not responses:
            print("\n[Drone] No response (timeout).")
        return responses

    def runScriptMode(self, cmds):
        """
        Example script mode: sends a sequence of commands.
        Each command on the RPi side should append an end-of-response marker.
        """
        # print("\n[Drone] Running preconfigured script...")

        for cmd, wait_sec, end_kw in cmds:
            self.sendCommand(cmd, wait_time=wait_sec, expect_multi=True, end_keyword=end_kw)
            time.sleep(2)
        # print("\n[Drone] Script mode complete.")

    
    def disconnect(self):
        if self.ser is not None:
            self.ser.close()
        print("[Drone] Disconnected to telemetry radio")
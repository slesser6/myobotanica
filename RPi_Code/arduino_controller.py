import serial
import time

class ArduinoController:
    def __init__(self, serial_port='/dev/ttyUSB1', baud_rate=9600):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = None
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            print(f"[DEBUG] Connected to Arduino on {self.serial_port} at {self.baud_rate} baud.")
            time.sleep(2)  # Give Arduino time to reset
        except Exception as e:
            print("[DEBUG] Could not connect to Arduino:", e)

    def wait_for_response(self, timeout=2):
        """Wait for a response from Arduino for up to 'timeout' seconds."""
        end_time = time.time() + timeout
        response_lines = []
        print("[DEBUG] Waiting for response from Arduino...")
        while time.time() < end_time:
            available = self.ser.in_waiting
            if available:
                print(f"[DEBUG] {available} byte(s) available in buffer.")
                try:
                    # Read one line; decode and strip it.
                    line = self.ser.readline().decode(errors='replace').strip()
                except Exception as err:
                    print("[DEBUG] Error decoding line:", err)
                    line = ""
                if line:
                    response_lines.append(line)
                    print(f"[DEBUG] Received line: '{line}'")
                    break  # Remove 'break' if you expect multiple lines
            time.sleep(0.1)
        if not response_lines:
            print("[DEBUG] No response received before timeout.")
        return "\n".join(response_lines) if response_lines else ""

    def send_command(self, command):
        """
        Expecting commands like:
          SERVO:PITCH:30      (for servo 1)
          SERVO:PITCH2:45     (for servo 2)
          PUMP:ON
          PUMP:OFF
        """
        if not self.ser:
            print("[DEBUG] Arduino not connected in send_command()")
            return "Arduino not connected."

        # Log the command before sending.
        print(f"[DEBUG] Sending command: '{command.strip()}'")

        parts = command.split(":")
        cmd_type = parts[0].upper()

        if cmd_type == "SERVO" and len(parts) >= 3:
            # Use the provided axis (PITCH or PITCH2)
            axis = parts[1].upper().strip()
            angle = parts[2].strip()
            msg = f"SERVO:{axis}:{angle}\n"
            print(f"[DEBUG] Writing to serial: {repr(msg)}")
            self.ser.write(msg.encode())
            # Immediately flush output to ensure it is sent.
            self.ser.flush()
            
            # Wait for the Arduino to respond with an acknowledgment.
            response = self.wait_for_response(timeout=3)
            if response:
                return f"Arduino responded: {response}"
            else:
                return f"Sent servo command: {axis} to {angle} degrees, but no response received."
        elif cmd_type == "PUMP" and len(parts) >= 2:
            state = parts[1].upper()
            msg = f"PUMP:{state}\n"
            print(f"[DEBUG] Writing to serial: {repr(msg)}")
            self.ser.write(msg.encode())
            self.ser.flush()
            response = self.wait_for_response(timeout=3)
            if response:
                return f"Arduino responded: {response}"
            else:
                return f"Pump command sent: {state}, but no response received."
        else:
            return f"Unknown Arduino command: {command}"

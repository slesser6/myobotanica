# arduino_controller.py
import serial
import time

class ArduinoController:
    def __init__(self, serial_port='/dev/ttyUSB1', baud_rate=9600):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = None
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            print(f"Connected to Arduino on {self.serial_port} at {self.baud_rate} baud.")
        except Exception as e:
            print("Could not connect to Arduino:", e)

    def send_command(self, command):
        """
        Expecting commands like:
          SERVO:PITCH:30
          SERVO:YAW:120
          PUMP:ON
          PUMP:OFF
        """
        if not self.ser:
            return "Arduino not connected."

        parts = command.split(":")
        cmd_type = parts[0].upper()

        if cmd_type == "SERVO" and len(parts) >= 3:
            axis = parts[1].upper()
            angle = parts[2].strip()
            msg = f"SERVO:{axis}:{angle}\n"
            self.ser.write(msg.encode())
            return f"Sent servo command: {axis} to {angle} degrees."
        elif cmd_type == "PUMP" and len(parts) >= 2:
            state = parts[1].upper()
            msg = f"PUMP:{state}\n"
            self.ser.write(msg.encode())
            return f"Pump set to {state}."
        else:
            return f"Unknown Arduino command: {command}"

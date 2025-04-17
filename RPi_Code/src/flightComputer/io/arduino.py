# flightComputer/io/arduino.py
from __future__ import annotations
from .serial_link import SerialLink
import logging, time
from typing import Final

# command constants
_CMD_SERVO:   Final[str] = "SERVO"
_CMD_PUMP:    Final[str] = "PUMP"
_AXES:        Final[set[str]] = {"PITCH", "PITCH2"}

class ArduinoController(SerialLink):
    """
    High‑level helper that speaks our SERVO:/PUMP: protocol over a SerialLink.
    """

    def __init__(self, port: str, baud: int):
        super().__init__(port, baud, timeout=1)
        # replace noisy prints with logging
        self.log = logging.getLogger("Arduino")

    # ---------- private helpers ----------------------------------------
    def _wait_response(self, timeout: float = 2.0) -> str:
        """Block until a non‑empty line is received or timeout expires."""
        end = time.time() + timeout
        while time.time() < end:
            line = self.read_line()
            if line:
                self.log.debug("RX %s", line)
                return line
            time.sleep(0.1)
        return ""

    # ---------- public API ---------------------------------------------
    def send_command(self, command: str) -> str:
        """
        Accepts strings like:
          SERVO:PITCH:30
          SERVO:PITCH2:45
          PUMP:ON
          PUMP:OFF
        Returns Arduino’s response or a diagnostic string.
        """
        if not self.ser:                               # port not open
            return "Arduino not connected."

        self.log.debug("CMD %s", command.strip())
        parts = command.split(":")

        if len(parts) < 2:
            return "Malformed Arduino command."

        cmd = parts[0].upper()

        # ---------- SERVO ---------------------------------------------
        if cmd == _CMD_SERVO and len(parts) >= 3:
            axis, angle = parts[1].upper(), parts[2].strip()
            if axis not in _AXES:
                return f"Unknown servo axis '{axis}'."
            payload = f"{_CMD_SERVO}:{axis}:{angle}"
            self.write_line(payload)
            self.flush()
            resp = self._wait_response(3)
            return resp or f"Sent servo {axis} -> {angle}; no ACK."

        # ---------- PUMP ----------------------------------------------
        if cmd == _CMD_PUMP and len(parts) >= 2:
            state = parts[1].upper()
            if state not in {"ON", "OFF"}:
                return "Pump state must be ON or OFF."
            self.write_line(f"{_CMD_PUMP}:{state}")
            self.flush()
            resp = self._wait_response(3)
            return resp or f"Pump {state}; no ACK."

        return f"Unknown Arduino command: {command}"

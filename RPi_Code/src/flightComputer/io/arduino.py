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
    def _wait_response(self, timeout=1.0, poll=0.01) -> str:
        end = time.time() + timeout
        while time.time() < end:
            line = self.read_line()
            if not line:
                time.sleep(poll)
                continue
            if line.startswith("OK READY"):       #   ← ignore banner
                self.log.debug("skip banner")
                continue
            self.log.debug("RX %s", line)
            return line
        return ""                 # timeout



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
        # if cmd == _CMD_SERVO:
        #     # Expect exactly 4 fields:  SERVO:BOTH:<inner>:<outer>
        #     if len(parts) != 4 or parts[1].upper() != "BOTH":
        #         return (f"Use the form SERVO:BOTH:<inner>:<outer> (got {command})")

        #     inner, outer = parts[2].strip(), parts[3].strip()

        #     # --- timed write / read ---------------------------------------
        #     t0 = time.perf_counter()
        #     self.write_line(f"SERVO:BOTH:{inner}:{outer}")
        #     # self.flush()

        #     reply = self._wait_response(timeout=1.0, poll=0.01)
        #     dt_ms = (time.perf_counter() - t0) * 1000.0

        #     # one concise log entry
        #     self.log.info("SERVO %-3s %-3s  %.1f ms  %s",
        #                 inner, outer, dt_ms,
        #                 reply or "<timeout>")
            
            
        #     # one concise log entry
        #     self.log.info("SERVO %-3s %-3s  %.1f ms  %s",
        #                   inner, outer, dt_ms,
        #                   reply or "<timeout>")

        #     # -------- append ' complete' if we actually got a reply -------
        #     if reply:                           # non-empty → ACK received
        #         # # avoid double-appending if the device already adds it
        #         # if not reply.rstrip().lower().endswith("complete"):
        #         #     reply = reply.rstrip() + " END_RESPONSE"
                    
        #         # reply = "END_RESPONSE"
                
        #         # add the marker *after* the ACK, not in place of it
        #         if not reply.rstrip().endswith("END_RESPONSE"):
        #             reply = reply.rstrip() + " END_RESPONSE"

        #         return reply                    # will look like
        #                                         # "ACK:SERVO:BOTH:0:90 END_RESPONSE"      
        #         # return reply                    # e.g. "SERVO OK complete"
        #     else:
        #         return "No ACK"
            
        #     # return reply or "No ACK"
        
        
        # ---------- SERVO --------------------------------------------------
        if cmd == _CMD_SERVO:
            # Expect exactly 4 fields:  SERVO:BOTH:<inner>:<outer>
            if len(parts) != 4 or parts[1].upper() != "BOTH":
                return f"Use the form SERVO:BOTH:<inner>:<outer> (got {command})"

            inner, outer = parts[2].strip(), parts[3].strip()

            # --- timed write / read ---------------------------------------
            t0 = time.perf_counter()
            self.write_line(f"SERVO:BOTH:{inner}:{outer}")

            reply = self._wait_response(timeout=1.0, poll=0.01)
            dt_ms = (time.perf_counter() - t0) * 1000.0

            self.log.info("SERVO %-3s %-3s  %.1f ms  %s",
                        inner, outer, dt_ms, reply or "<timeout>")

            # ----------------------------------------------------------------
            # Whatever we got (or not), append the marker on a *separate line*
            # so the GCS can stop reading immediately.
            # ----------------------------------------------------------------
            if reply:                       # Arduino sent an ACK
                return f"{reply}\nEND_RESPONSE"
            else:                           # No reply at all
                return "No ACK\nEND_RESPONSE"
        
        
        # ---------- PUMP ----------------------------------------------
        # if cmd == _CMD_PUMP:
        #     if parts[1].upper() == "ON":
        #         duration = 3000  # default 3000 ms
        #         if len(parts) > 2 and parts[2].isdigit():
        #             duration = int(parts[2])
        #         return self.pump_fixed()
        #     elif parts[1].upper() == "OFF":
        #         self.write_line(f"{_CMD_PUMP}:OFF")
        #         self.flush()
        #         return self._wait_response()
        #     else:
        #         return f"Invalid PUMP command: {command}"        


        # return f"Unknown Arduino command: {command}"

        # ---------- PUMP ---------------------------------------------------
        if cmd == _CMD_PUMP:
            action = parts[1].upper() if len(parts) > 1 else ""

            # ---------------------- ON -------------------------------------
            if action == "ON":
                duration = 3000
                if len(parts) > 2 and parts[2].isdigit():
                    duration = int(parts[2])

                reply = self.pump_fixed(duration)         # <- see tweak below

                # always give the marker its own line
                if reply:
                    return f"{reply}\nEND_RESPONSE"
                else:
                    return "No ACK\nEND_RESPONSE"

            # ---------------------- OFF ------------------------------------
            elif action == "OFF":
                self.write_line(f"{_CMD_PUMP}:OFF")
                reply = self._wait_response(timeout=1.0, poll=0.01)

                if reply:
                    return f"{reply}\nEND_RESPONSE"
                else:
                    return "No ACK\nEND_RESPONSE"

            # ---------------------- bad arg --------------------------------
            else:
                return f"Invalid PUMP command: {command}"


    def set_offset(self, axis: str, degrees: int) -> str:
        """OFFSET:PITCH:-10 → shifts zero by -10°"""
        axis = axis.upper()
        if axis not in _AXES:
            return f"Unknown servo axis '{axis}'."
        self.write_line(f"{_CMD_OFFSET}:{axis}:{degrees}")
        self.flush()
        return self._wait_response()

    # ---------- PUMP ----------------------------------------------
    # def pump_fixed(self) -> str:
    #     """Turn pump on for <ms> milliseconds."""
    #     self.write_line(f"{_CMD_PUMP}:ON:3000")
    #     self.flush()
    #     return self._wait_response()
    
    def pump_fixed(self, duration_ms: int = 3000) -> str:
        """
        Turn the pump ON for duration_ms and return the Arduino's ACK line.
        """
        self.write_line(f"{_CMD_PUMP}:ON:{duration_ms}")
        self.flush()
        return self._wait_response(timeout=1.0, poll=0.01) or ""

    
    
    def pump_on_ms(self, ms: int) -> str:
        """Turn pump on for <ms> milliseconds."""
        self.write_line(f"{_CMD_PUMP}:ON:{ms}")
        self.flush()
        return self._wait_response()

    def get_state(self) -> str:
        """Return 'STATE:PITCH:…:PITCH2:…:PUMP:…'."""
        self.write_line(_CMD_STATE)
        self.flush()
        return self._wait_response()
    
    def __del__(self):
        self.log.info("ArduinoController %s GC-d", self)   # should appear once at exit
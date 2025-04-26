import serial 
import time
import logging
from src.utils import get_logger
class Drone:
    def __init__(self, cfg):
        self._logger = get_logger("Drone")
        if cfg.verbose:
            self._logger.setLevel(logging.DEBUG)
        else:
            self._logger.setLevel(logging.INFO)

        self._baud_rate = cfg.baud_rate
        self._port = cfg.port
        self._debug_mode = cfg.debug_mode
        self._enable = cfg.enable
        self._serial_conn = None
        self.takeoff_alt = cfg.takeoff_alt

    def connect(self):
        if not self._enable:
            self._logger.debug("Not enabled")
            return
        
        try:
            self._serial_conn = serial.Serial(self._port, self._baud_rate, timeout=0.5)
            self._logger.info(f"Connected to telemetry radio on {self._port} at {self._baud_rate} baud")
        except Exception as e:
            self._logger.error(f"Error opening serial port {self._port}: {e}")
            return
        
        i = 0
        while not self.sendCommand("FC:GETSTATE\n", 3, True, "END_RESPONSE").is_armable and i < 10:
            time.sleep(1)
            i += 1
        if self.sendCommand("FC:GETSTATE\n", 3, True, "END_RESPONSE").is_armable:
            self._logger.debug(self.sendCommand("FC:ARM\n", 10, True, "END_RESPONSE"))
        else:
            self._logger.warning("Drone is not armable")

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
        if not self._enable:
            self._logger.debug(cmd) 
            return
        if not self._serial_conn:
            self._logger.error("Connect to telemetry radio before sending commands") 
            return
        
        # Flush any stale data before sending
        self._serial_conn.reset_input_buffer()
        if self._debug_mode:
            while True:
                answer = input(f"\nWould you like to send this command: {cmd.strip()}? y/n: ").strip().lower()
                if answer == 'y':
                    break
                elif answer == 'n':
                    self._logger.debug("Not sending")
                    return
                else:
                    self._logger.info("Invalid input. Please enter 'y' or 'n'")

        self._logger.debug(f"Sending command to RPi: {cmd.strip()}")
        self._serial_conn.write(cmd.encode())

        responses = []
        end_time = time.time() + wait_time
        while True:
            if self._serial_conn.in_waiting:
                line = self._serial_conn.readline().decode().strip()
                # Check if we've reached the end marker
                if end_keyword and line == end_keyword:
                    break
                self._logger.info(line)
                responses.append(line)
                # If not expecting multi-line responses, break after one line.
                if not expect_multi:
                    break
            # If no data is arriving and we've passed the wait_time, break out.
            if time.time() > end_time:
                break
            time.sleep(0.1)
        if not responses:
            self._logger.warning("No response (timeout)")
        return responses

    def runScriptMode(self, cmds):
        """
        Example script mode: sends a sequence of commands.
        Each command on the RPi side should append an end-of-response marker.
        """
        if not self._enable:
            return []
        
        fails = []
        for cmd, wait_sec, end_kw in cmds:
            resp = self.sendCommand(cmd, wait_time=wait_sec, expect_multi=True, end_keyword=end_kw)
            if resp is None:
                fails.append(cmd) 
            else:
                self._logger.debug(resp)
            time.sleep(2)
        return fails

    def disconnect(self):
        if not self._enable:
            return
        if self._serial_conn is not None:
            self._serial_conn.close()
        self._logger.info("Disconnected from telemetry radio")
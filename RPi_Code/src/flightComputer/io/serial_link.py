import serial, time, logging

class SerialLink:
    def __init__(self, port: str, baud: int, timeout: float = 1):
        self.log = logging.getLogger(self.__class__.__name__)
        try:
            self.ser = serial.Serial(port, baud, timeout=timeout)
            self.log.info("Opened %s @ %d", port, baud)
            time.sleep(2)           # Arduino reset or FTDI settle
        except Exception as e:
            self.log.error("Could not open %s: %s", port, e)
            self.ser = None

    # helpers
    def write_line(self, text: str):
        if self.ser:
            self.ser.write((text + "\n").encode())

    def read_line(self) -> str:
        if self.ser and self.ser.in_waiting:
            return self.ser.readline().decode(errors="ignore").strip()
        return ""
    
    def flush(self):
        if self.ser:
            self.ser.flush()


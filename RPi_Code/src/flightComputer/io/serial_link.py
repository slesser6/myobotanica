# flightComputer/io/serial_link.py
import serial, time, logging, threading

class SerialLink:
    def __init__(self, port: str, baud: int, timeout: float = 1):
        self.log = logging.getLogger(self.__class__.__name__)
        self._lock = threading.Lock()               # ← add this
        try:
            # self.ser = serial.Serial(port, baud, timeout=timeout)
            # serial_link.py  (where SerialLink creates self.ser)

            self.ser = serial.Serial(
                port, baud, timeout=0.02,
                rtscts=False,  dsrdtr=False                      # <── important
            )
            self.ser.setDTR(False)    # keep RESET high

            self.log.info("Opened %s @ %d", port, baud)
            time.sleep(2)  # Arduino reset or FTDI settle
        except Exception as e:
            self.log.error("Could not open %s: %s", port, e)
            self.ser = None

    def write_line(self, text: str):
        if not self.ser:
            return
        data = (text + "\n").encode()
        with self._lock:
            self.ser.write(data)
            self.ser.flush()

    def read_line(self) -> str:
        """
        Always block until we see a newline, then return the
        decoded, stripped line.  This guarantees we only ever
        return *complete* \n‑terminated strings.
        """
        if not self.ser:
            return ""
        raw = self.ser.readline()     # blocks until “\n”
        return raw.decode(errors="ignore").strip()
    
    def flush(self):
        if self.ser:
            self.ser.flush()


from .serial_link import SerialLink
from queue import Queue
import threading, logging
import time

class RadioReceiver(threading.Thread, SerialLink):
    def __init__(self, port, baud, bus: Queue):
        SerialLink.__init__(self, port, baud, timeout=None)
        threading.Thread.__init__(self, daemon=True)
        self.bus = bus
        self.log = logging.getLogger("Radio")

    def run(self):
        self.log.info("Listening…")
        while True:
            try:
                line = self.read_line()
                if line:
                    self.log.debug("RX %s", line)
                    self.bus.put(line)
            except Exception as e:
                self.log.error("Radio error: %s", e)
                time.sleep(0.5)

    def send(self, text: str):
        self.write_line(text)
        if self.ser:
            self.ser.flush()
        self.log.debug("TX %s", text)
    

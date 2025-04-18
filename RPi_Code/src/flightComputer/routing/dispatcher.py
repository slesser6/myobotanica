import threading, logging, time, queue            # add time for sleep
from flightComputer.io.radio import RadioReceiver
from flightComputer.config import SERIAL_RADIO_PORT, SERIAL_RADIO_BAUD
from .bus import BUS
from .handlers import MAP

class Dispatcher(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.log = logging.getLogger("Dispatcher")
        # self.radio = RadioReceiver(SERIAL_RADIO_PORT,
        #                            SERIAL_RADIO_BAUD,
        #                            BUS)
        self.radio = RadioReceiver(SERIAL_RADIO_PORT, SERIAL_RADIO_BAUD, BUS)
        self.radio.start()

    def run(self):
        while True:
            try:
                cmd = BUS.get(timeout=0.5)        # ❶ unblockable loop
                # ignore telemetry JSON blobs (they start with "{")
                if cmd.lstrip().startswith("{"):
                    continue
            except queue.Empty:
                continue                          # nothing to do, loop
            if cmd == "__QUIT__":                 # ❷ graceful stop token
                self.log.info("Stopping dispatcher.")
                break

            prefix, _, payload = cmd.partition(":")
            func = MAP.get(prefix)
            if func:
                try:
                    resp = func(payload)
                    if resp:
                        self.radio.send(resp) # Send command response
                                        
                        from flightComputer.fc import FC
                        FC._telem_suspended.clear() # Resume telemetry
                        
                except Exception as e:           # ❸ protect handler errors
                    self.log.error("Handler %s blew up: %s", prefix, e)
            else:
                self.log.warning("Unknown prefix %s", prefix)
            time.sleep(0.01)                     # tiny yield to avoid 100 % CPU

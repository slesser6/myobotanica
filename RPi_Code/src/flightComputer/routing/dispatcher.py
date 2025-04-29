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
        self.log.info("Dispatcher started")
        # while True:
        #     try:
        #         # cmd = BUS.get(timeout=0.5)    # grab the next line pushed by RadioReceiver
        #         cmd = BUS.get(timeout=2)
        #     except queue.Empty:
        #         continue

        while True:
            try:
                cmd = BUS.get_nowait()          # returns immediately
            except queue.Empty:
                time.sleep(0.02)                # 20 ms idle nap
                continue


            # ignore telemetry JSON blobs
            if cmd.lstrip().startswith("{"):
                continue

            if cmd == "__QUIT__":
                self.log.info("Stopping dispatcher.")
                break

            prefix, _, payload = cmd.partition(":")
            func = MAP.get(prefix)
            if func:
                try:
                    resp = func(payload)
                    if resp:
                        self.radio.send(resp)
                        # once the one-off reply is sent, resume telemetry
                        from flightComputer.fc import FC
                        FC._telem_suspended.clear()
                except Exception as e:
                    self.log.error("Handler %s blew up: %s", prefix, e)
            else:
                self.log.warning("Unknown prefix %s", prefix)

            # time.sleep(10)
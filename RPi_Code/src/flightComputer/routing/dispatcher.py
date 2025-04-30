# flightComputer/routing/dispatcher.py
import threading, logging, time, queue, asyncio
from flightComputer.io.radio import RadioReceiver
from flightComputer.config   import SERIAL_RADIO_PORT, SERIAL_RADIO_BAUD
from .bus      import BUS
from .handlers import MAP

class Dispatcher(threading.Thread):
    """
    ① Reads lines pushed onto BUS by RadioReceiver
    ② Looks up a handler in MAP
    ③ Sends the handler’s reply back over the radio
    """

    IDLE_SLEEP = 0.02          # 20 ms; tweak if CPU use is too high

    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.log   = logging.getLogger("Dispatcher")
        self.radio = RadioReceiver(SERIAL_RADIO_PORT,
                                   SERIAL_RADIO_BAUD,
                                   BUS)
        self.radio.start()

    # ───────────────────────── helper ──────────────────────────────
    def _maybe_await(self, obj):
        """Run coro in-thread if it looks awaitable, else return obj."""
        if asyncio.iscoroutine(obj):
            return asyncio.run(obj)        # blocks this thread until done
        return obj

    # ───────────────────────── main loop ───────────────────────────
    def run(self):
        self.log.info("Dispatcher started")
        while True:
            try:
                cmd = BUS.get_nowait()           # returns instantly
            except queue.Empty:
                time.sleep(self.IDLE_SLEEP)      # yield CPU
                continue

            # telemetry JSON blobs come in prefixed with "T:"
            if cmd.lstrip().startswith("{") or cmd.startswith("T:"):
                continue

            if cmd == "__QUIT__":
                self.log.info("Stopping dispatcher.")
                break

            prefix, _, payload = cmd.partition(":")
            func = MAP.get(prefix)

            if not func:
                self.log.warning("Unknown prefix %s", prefix)
                continue

            try:
                raw_resp = func(payload)         # may return coro or str
                resp     = self._maybe_await(raw_resp)
                if resp:
                    self.radio.send(resp)

                    # resume telemetry if it was paused for the one-off
                    from flightComputer.fc import FC
                    FC._telem_suspended.clear()
            except Exception as e:
                self.log.error("Handler %s blew up: %s", prefix, e)

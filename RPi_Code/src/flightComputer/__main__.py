# flightComputer/__main__.py
from flightComputer.util.log           import init
from flightComputer.routing.dispatcher import Dispatcher
from flightComputer.fc                 import FC
import time, logging, signal, sys

def main() -> None:
    init()
    disp = Dispatcher()        # creates RadioReceiver
    disp.start()               # start the thread

    try:
        # lazy-connect: will happen automatically on first command
        while disp.is_alive():  # keep the main thread idle
            time.sleep(0.5)

    except KeyboardInterrupt:
        logging.info("CTRL-C – shutting down …")

    finally:
        # 1) stop dispatcher / radio cleanly
        if disp.is_alive():
            disp.radio.send("__QUIT__")   # ask it to exit its loop
            disp.join(timeout=2)

        # 2) close MAVSDK + terminate SITL
        FC.shutdown()

if __name__ == "__main__":
    main()

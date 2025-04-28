# flightComputer/__main__.py
from flightComputer.util.log           import init
from flightComputer.routing.dispatcher import Dispatcher
from flightComputer.fc                 import FC        # ← singleton

def main() -> None:
    init()
    FC.connect()                        # starts MAVLink (and SITL if requested)
    
    disp = Dispatcher()                 # starts RadioReceiver inside
    # FC.start_telemetry(disp.radio)    
    
    try:
        disp.run()                      # never returns until SIGINT/SIGTERM
    except KeyboardInterrupt:
        pass                            # Ctrl‑C in foreground mode
    finally:
        FC.shutdown()                   # <── graceful clean‑up (new method)

if __name__ == "__main__":
    main()

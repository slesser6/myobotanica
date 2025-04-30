import asyncio, threading, logging
from .mavsdk_fc import MavsdkFlightController

_log = logging.getLogger("FCBridge")
_loop = asyncio.new_event_loop()
threading.Thread(target=_loop.run_forever, daemon=True).start()

_fc = MavsdkFlightController()

def run(coro):
    return asyncio.run_coroutine_threadsafe(coro, _loop).result()

# ---- “DroneKit-like” façade ---------------------------------------
def connect(addr, wait_ready=True, baud=None):
    run(_fc.connect())
    return _fc            # return the async object (works for our needs)

Vehicle = MavsdkFlightController   # alias so old “Vehicle” hints still resolve

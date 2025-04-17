"""
flightComputer.routing.bus
Shared in‑memory message bus.

Anything that wants to publish a new command does:

    from flightComputer.routing.bus import BUS
    BUS.put("FC:ARM")

Anything that wants to consume does:

    from flightComputer.routing.bus import BUS
    cmd = BUS.get()

The dispatcher thread is the main consumer.
"""
from queue import Queue
from typing import TYPE_CHECKING

BUS: Queue = Queue()          # runtime annotation (3.8‑safe)

if TYPE_CHECKING:             # evaluated only by type checkers
    from queue import Queue as _Q
    BUS: _Q[str]              # now they know it's Queue[str]

# Optional helpers ---------------------------------------------------
def publish(msg: str) -> None:
    """Drop a message onto the bus."""
    BUS.put(msg)

def shutdown_sentinel() -> None:
    """Put a special token that tells dispatcher to exit cleanly."""
    BUS.put("__QUIT__")

"""
routing/dispatcher.py 

Routes radio messages (topic:payload) to the correct subsystem.
"""

import time
import logging
from flightComputer.fc import FC                    # MAVSDK FlightController
from flightComputer.io import ARDUINO as ARD        # your Arduino wrapper

log = logging.getLogger("Timing")


# ─────────────────────────── FC handler ──────────────────────────────
def handle_fc(payload: str) -> str:
    """
    Parse a legacy ASCII command and forward it to the new FlightController API.
    Returns the same END_RESPONSE-terminated string the old code produced.
    """
    cmd, *args = payload.strip().split(":")

    try:
        if cmd.upper() == "GETSTATE":
            return FC.get_state()

        elif cmd.upper() == "ARM":
            return FC.arm_drone()

        elif cmd.upper() == "TAKEOFF":
            alt = float(args[0]) if args else 2.0
            return FC.takeoff_drone(alt)

        elif cmd.upper() == "LAND":
            return FC.land_drone()

        elif cmd.upper() == "MOVE":
            # MOVE:<DIR>:<DIST>:<VEL>
            direction = args[0] if len(args) > 0 else "FWD"
            dist      = float(args[1]) if len(args) > 1 else 2.0
            vel       = float(args[2]) if len(args) > 2 else 1.0
            return FC.move(direction, dist, vel)

        elif cmd.upper() == "YAW":
            # YAW:<LEFT|RIGHT|rate_dps>:<angle>
            arg  = args[0] if len(args) > 0 else "RIGHT"
            rate = float(args[1]) if len(args) > 1 else 15
            return FC.yaw_drone(arg, rate)

        else:
            return f"ERR Unknown command '{cmd}'\nEND_RESPONSE"

    except Exception as e:
        log.exception("FC command failed: %s", e)
        return f"ERR {e}\nEND_RESPONSE"


# ──────────────────────── Arduino handler ────────────────────────────
def handle_arduino(payload: str) -> str:
    t_in = time.perf_counter()
    log.debug("handle_arduino IN  %.6f  %s", t_in, payload)

    reply = ARD.send_command(payload)      # unchanged

    t_out = time.perf_counter()
    log.debug("handle_arduino OUT %.6f  Δ=%.3f s  %s",
              t_out, t_out - t_in, reply)
    return reply


# ────────────────────────── dispatch map ─────────────────────────────
MAP = {
    "FC": handle_fc,
    "AR": handle_arduino,
}

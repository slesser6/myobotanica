from flightComputer.fc import FC
from flightComputer.io.arduino import ArduinoController
from flightComputer import config
from .bus import BUS
from flightComputer.fc import FC
from flightComputer.io import ARDUINO as ARD  
import time, logging
log = logging.getLogger("Timing")

# ARD = ArduinoController(config.SERIAL_ARDUINO_PORT,
#                         config.SERIAL_ARDUINO_BAUD)


def handle_fc(payload: str):
    return FC.send_command(payload)

# def handle_arduino(payload: str):
#     return ARD.send_command(payload)

def handle_arduino(payload: str):
    t_in = time.perf_counter()
    log.debug("handle_arduino IN  %.6f  %s", t_in, payload)

    reply = ARD.send_command(payload)

    t_out = time.perf_counter()
    log.debug("handle_arduino OUT %.6f  Δ=%.3f s  %s",
              t_out, t_out - t_in, reply)
    return reply

MAP = {
    "FC": handle_fc,
    "AR": handle_arduino,
}

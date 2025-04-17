from flightComputer.fc import FC
from flightComputer.io.arduino import ArduinoController
from flightComputer import config
from .bus import BUS
from flightComputer.fc import FC



ARD = ArduinoController(config.SERIAL_ARDUINO_PORT,
                        config.SERIAL_ARDUINO_BAUD)

def handle_fc(payload: str):
    return FC.send_command(payload)

def handle_arduino(payload: str):
    return ARD.send_command(payload)

MAP = {
    "FC": handle_fc,
    "AR": handle_arduino,
}

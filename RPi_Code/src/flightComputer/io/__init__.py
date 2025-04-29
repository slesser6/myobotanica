# flightComputer/io/__init__.py
from .arduino import ArduinoController          # local module
from .. import config                           # parent package

ARDUINO = ArduinoController(                    # single, shared handle
    port=config.SERIAL_ARDUINO_PORT,
    baud=config.SERIAL_ARDUINO_BAUD,
)

__all__ = ["ARDUINO"]                           # what “from … import *” exposes

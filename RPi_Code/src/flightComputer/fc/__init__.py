"""
flightComputer.fc package façade.

After the MAVSDK migration the real implementation lives in
mavsdk_fc.py, but external code still imports
    flightComputer.fc.FlightController
and sometimes grabs a singleton called FC.  We keep both symbols here.
"""
from .mavsdk_fc import FlightController        # ← alias

# Project-wide configuration
from flightComputer import config

# One shared instance used by the GUI / CLI
FC = FlightController(
        connection_string=config.FC_CONN_STRING,      # server-style URL
        baud_rate=config.FC_BAUD,
        simulation_mode=config.SIMULATION_MODE,
        indoor_mode=config.INDOOR_MODE
)
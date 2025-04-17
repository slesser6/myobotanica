from .mavlink_fc import FlightController
from flightComputer import config

FC = FlightController(
        connection_string=config.FC_CONN_STRING,
        baud_rate=config.FC_BAUD,
        simulation_mode=config.SIMULATION_MODE
     )

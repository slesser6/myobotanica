SERIAL_RADIO_PORT   = "/dev/radio" # "cp210x converter"
SERIAL_RADIO_BAUD   = 57600

SERIAL_ARDUINO_PORT = "/dev/arduino" # "ch341 uart-converter"
SERIAL_ARDUINO_BAUD = 115200

# Settings for connecting to Mini-Pix FC
# FC_CONN_STRING      = "serial:///dev/ttyAMA0:921600"
# FC_BAUD             = 921600
# INDOOR_MODE         = True # Indoor mode does not use the GPS 
# SIMULATION_MODE     = False          

# Settings for connecting to SITL
FC_CONN_STRING = "udp://:14540"
FC_BAUD             = 115200
INDOOR_MODE         = False 
SIMULATION_MODE     = True 
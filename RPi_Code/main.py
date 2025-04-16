# main.py
import queue
import time
from command_receiver import CommandReceiver
from flight_controller import FlightController
from arduino_controller import ArduinoController  

'''
Main orchestrator running on RPi. Initializes the telemetry radio (CommandReceiver) and the flight controller (via DroneKit or pymavlink), and optionally the Arduino controller. Dispatches commands based on prefixes (FC: vs AR:).
'''

def main():
    # Shared queue for commands
    command_queue = queue.Queue()

    # Initialize the CommandReceiver (telemetry radio interface)
    receiver = CommandReceiver(serial_port='/dev/ttyUSB1', baud_rate=57600, command_queue=command_queue)
    receiver.start()

    # Initialize the FlightController (MAVLink / DroneKit interface)
    ### --- COMMENT / UNCOMMENT to run in simulation mode or to connect to FC
    # Connect to FC 
    # fc = FlightController(connection_string='/dev/ttyAMA0', baud_rate=115200, simulation_mode=False) 
    # Run in Simulation mode
    fc = FlightController(connection_string='/dev/ttyAMA0', baud_rate=115200, simulation_mode=True) 
    fc.connect()

    # # Initialize the Arduino controller (for water arm servos & pump)
    arduino = ArduinoController(serial_port='/dev/ttyUSB0', baud_rate=9600)

    print("Drone hub system running...")
    while True:
        try:
            # Check for new commands from the ground station
            if not command_queue.empty():
                command = command_queue.get()
                print("Dispatching command:", command)

                # If it's a flight controller command (FC:)
                if command.startswith("FC:"):
                    payload = command[3:]
                    response = fc.send_command(payload)
                    if response:
                        receiver.send_response(response)

                # If it's an Arduino command (AR:)
                elif command.startswith("AR:"):
                    payload = command[3:]
                    print("Sending arduino command")
                    response = arduino.send_command(payload)
                    if response:
                        receiver.send_response(response)

                else:
                    print("Unknown command type:", command)

            time.sleep(0.1)
        except KeyboardInterrupt:
            fc.close()
            print("Shutting down...")
            break

if __name__ == '__main__':
    main()

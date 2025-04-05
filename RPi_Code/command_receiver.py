# command_receiver.py
import serial
import threading
import queue

'''
Responsibility: Open the serial port for the telemetry radio and continuously listen for incoming commands.
Output: Push received commands to a shared message queue or use callbacks to notify the main controller.
'''

class CommandReceiver(threading.Thread):
    def __init__(self, serial_port='/dev/ttyUSB0', baud_rate=57600, command_queue=None):
        super().__init__(daemon=True)
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.command_queue = command_queue or queue.Queue()
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)

    def run(self):
        print("Command Receiver: Listening on", self.serial_port)
        while True:
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                if line:
                    print("Received command:", line)
                    self.command_queue.put(line)
            except Exception as e:
                print("Command Receiver error:", e)
    
    def send_response(self, response):
        try:
            self.ser.write((response + "\n").encode())
            print("Sent response:", response)
        except Exception as e:
            print("Error sending response:", e)

if __name__ == '__main__':
    # For standalone testing
    q = queue.Queue()
    receiver = CommandReceiver(command_queue=q)
    receiver.start()
    while True:
        cmd = q.get()
        print("Main thread got command:", cmd)

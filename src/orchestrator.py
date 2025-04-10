from src.utils import load_config
from src.drone import Drone
from src.sensors import Myoband
from src.sensors import Kinect
from src.classifier import Classifier
from src.kinematics import solve_ik

class Orchestrator:
    def __init__(self):
        self.configs = load_config()
        self.drone = Drone(self.configs.drone, self.configs.motion)
        self.myoband = Myoband(self.configs.myoband)
        self.kinect = Kinect(self.configs.kinect)
        self.classifier = Classifier()

    def connect(self):
        self.myoband.connect()
        self.kinect.connect()
        self.drone.connect()

    def disconnect(self):
        self.myoband.disconnect()
        self.kinect.disconnect()
        self.drone.disconnect()

    def poll_sensors(self):
        self.myoband.update()
        classification = None
        
        if self.myoband.is_ready(50):
            data_block = self.myoband.get_data(50)
            classification = self.classifier.classify(data_block)
        position = self.kinect.getPosition()
        return classification, position
    
    def get_status(self):
        print(f""+self.drone.sendCommand("FC:GETSTATE\n", 3, True, "END_RESPONSE"))
        # get status from myoband?
        # get status from kinect

    def run_calculations(self, desired_pos):
        return solve_ik(desired_pos)

    def send_output(self, cmds):
        self.drone.runScriptMode(cmds)
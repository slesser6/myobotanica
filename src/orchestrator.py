from src.utils import load_config
from src.drone import Drone
from src.sensors import Myoband
from src.sensors import Kinect
from src.classifier import Classifier
from src.kinematics import solve_ik

class Orchestrator:
    def __init__(self):
        self.configs = load_config()
        self.drone = Drone(self.configs.drone)
        self.myoband = Myoband(self.configs.myoband)
        self.kinect = Kinect(self.configs.kinect)

    def connect(self):
        self.myoband.connect()
        self.kinect.connect()
        self.drone.connect()

    def disconnect(self):
        self.myoband.disconnect()
        self.kinect.disconnect()
        self.drone.disconnect()

    def loop(self):
        self.myoband.loop()
        self.kinect.loop()

        return self.myoband.data_block, (self.kinect.left_pos, self.kinect.right_pos)
    
    def get_status(self):
        print(f""+self.drone.sendCommand("FC:GETSTATE\n", 3, True, "END_RESPONSE"))
        # get status from myoband?
        # get status from kinect

    def run_calculations(self, desired_pos):
        return solve_ik(desired_pos)

    def send_output(self, cmds):
        self.drone.runScriptMode(cmds)
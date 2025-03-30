from src.utils import load_config
from src.drone import Drone
from src.sensors import Myoband
from src.sensors import Kinnect
from src.classifier import Classifier
from src.kinematics import solve_ik

class Orchestrator:
    def __init__(self):
        self.debug
        self.configs = load_config()
        self.drone = Drone(self.configs.drone, self.configs.motion)
        self.myoband = Myoband(self.configs.myoband)
        self.kinnect = Kinnect(self.configs.kinnect)
        self.classifier = Classifier()

    def connect(self):
        self.myoband.connect()
        self.kinnect.connect()
        self.drone.connect()

    def disconnect(self):
        self.myoband.disconnect()
        self.kinnect.disconnect()
        self.drone.disconnect()

    def poll_sensors(self):
        myo_data = self.myoband.getData()
        classification = self.classifier.classify(myo_data)
        position = self.kinnect.getPosition()
        return classification, position

    def run_calculations(self, desired_pos):
        return solve_ik(desired_pos)

    def send_output(self, cmd, joint_positions):
        self.drone.sendCommand(cmd)
        self.drone.moveArm(joint_positions)
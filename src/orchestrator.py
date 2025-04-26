from src.utils import load_config
from src.drone import Drone
from src.drone import Arm
from src.sensors import Myoband
from src.sensors import Kinect
from src.classifier import Classification
from spatialmath import *
import numpy as np

class Orchestrator:
    def __init__(self):
        configs = load_config()
        self.drone = Drone(configs.drone)
        self.arm = Arm(configs.arm)
        self.myoband = Myoband(configs.myoband)
        self.kinect = Kinect(configs.kinect)
        self.verbose = configs.orchestrator.verbose
        self.polling_period = configs.orchestrator.polling_period

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

        return self.myoband.classification, (self.kinect.left_pos, self.kinect.right_pos)
    
    def get_status(self):
        print(f""+self.drone.sendCommand("FC:GETSTATE\n", 3, True, "END_RESPONSE"))
        # TODO: get status from myoband?
        # TODO: get status from kinect

    def run_calculations(self):
        self.prev_pos = self.arm.current_pose
        if(self.myoband.classification == Classification.WRIST_FLEX_TURN_LEFT):
            self.arm.current_pose = SE3.Rz(np.pi/12)*self.arm.current_pose
        elif(self.myoband.classification == Classification.WRIST_EXT_TURN_RIGHT):
            self.arm.current_pose = SE3.Rz(-np.pi/12)*self.arm.current_pose
        elif(self.myoband.classification == Classification.WRIST_ADD_ARM_DOWN):
            self.arm.current_pose = SE3(0, 0, -.25)*self.arm.current_pose
        elif(self.myoband.classification == Classification.WRIST_ABD_ARM_UP):
            self.arm.current_pose = SE3(0, 0, .25)*self.arm.current_pose
        return self.arm.get_ikine()

    def send_output(self, cmds):
        fails = self.drone.runScriptMode(cmds)
        for fail in fails:
            if fail.startswith("FC:YAW:") or fail.startswith("AR:SERVO"):
                self.arm.update_pose(self.prev_pos)
                break
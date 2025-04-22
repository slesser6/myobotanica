from src.utils import load_config
from src.drone import Drone
from src.sensors import Myoband
from src.sensors import Kinect
# TODO: from src.kinematics import solve_ik

class Orchestrator:
    def __init__(self):
        configs = load_config()
        self.drone = Drone(configs.drone)
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

    def run_calculations(self, desired_pos):
        # TODO: return solve_ik(desired_pos)
        return 0

    def send_output(self, cmds):
        self.drone.runScriptMode(cmds)
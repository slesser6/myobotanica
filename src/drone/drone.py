class Drone:
    def __init__(self, drone_cfg, motion_cfg):
        self.ip = drone_cfg.ip
        self.port = drone_cfg.port
        self.speed = motion_cfg.speed
        self.debug_mode = motion_cfg.debug_mode

    def connect():
        return 0
    
    def sendCommand():
        return 0
    
    def moveArm(joint_values):
        return 0
    
    def disconnect():
        return 0
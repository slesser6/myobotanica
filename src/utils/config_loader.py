import yaml
from .dataclasses import DroneConfig, RobotArmConfig, MotionConfig, MyobandConfig, KinnectConfig, Configs

def load_config(path="config/settings.yaml"):
    with open(path, "r") as f:
        raw = yaml.safe_load(f)

    cfgs = Configs()
    cfgs.drone = DroneConfig(**raw["drone"])
    cfgs.robot_arm = RobotArmConfig(**raw["robot_arm"])
    cfgs.motion = MotionConfig(**raw["motion"])
    cfgs.myoband = MyobandConfig(**raw["myoband"])
    cfgs.kinnect = KinnectConfig(**raw["kinnect"])

    return cfgs
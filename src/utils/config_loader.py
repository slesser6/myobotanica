import yaml
from .dataclasses import DroneConfig, RobotArmConfig, MotionConfig, MyobandConfig, KinnectConfig, Configs

def load_config(path=".\config\settings.yml"):
    with open(path, "r") as f:
        raw = yaml.safe_load(f)

    cfgs = Configs(DroneConfig(**raw["drone"]), RobotArmConfig(**raw["robot_arm"]), MotionConfig(**raw["motion"]), MyobandConfig(**raw["myoband"]), KinnectConfig(**raw["kinnect"]))
    return cfgs
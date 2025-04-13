import yaml
from .dataclasses import DroneConfig, RobotArmConfig, MyobandConfig, KinectConfig, Configs

def load_config(path="./config/settings.yml"):
    with open(path, "r") as f:
        raw = yaml.safe_load(f)

    cfgs = Configs(DroneConfig(**raw["drone"]), RobotArmConfig(**raw["robot_arm"]), MyobandConfig(**raw["myoband"]), KinectConfig(**raw["kinect"]))
    return cfgs
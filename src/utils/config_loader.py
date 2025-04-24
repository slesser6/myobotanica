import yaml
from .dataclasses import DroneConfig, ArmConfig, MyobandConfig, KinectConfig, OrchestratorConfig, Configs

def load_config(path="./config/settings.yml"):
    with open(path, "r") as f:
        raw = yaml.safe_load(f)

    cfgs = Configs(DroneConfig(**raw["drone"]), ArmConfig(**raw["arm"]), MyobandConfig(**raw["myoband"]), KinectConfig(**raw["kinect"]), OrchestratorConfig(**raw["orchestrator"]))
    return cfgs
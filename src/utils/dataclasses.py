# src/config/models.py

from dataclasses import dataclass
from typing import List

@dataclass
class DroneConfig:
    ip: str
    port: int

@dataclass
class RobotArmConfig:
    base_height: float
    link_lengths: List[float]

@dataclass
class MotionConfig:
    speed: str
    debug_mode: int

@dataclass
class MyobandConfig:
    ip: str
    port: int

@dataclass
class KinnectConfig:
    ip: str
    port: int

@dataclass
class Configs:
    drone: DroneConfig
    robot_arm: RobotArmConfig
    motion: MotionConfig
    myoband: MyobandConfig
    kinnect: KinnectConfig
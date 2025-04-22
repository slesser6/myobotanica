# src/config/models.py

from dataclasses import dataclass
from typing import List

@dataclass
class DroneConfig:
    port: str
    baud_rate: int
    takeoff_alt: int
    debug_mode: bool
    verbose: bool
    enable: bool

@dataclass
class RobotArmConfig:
    base_height: float
    link_lengths: List[float]
    verbose: bool
    enable: bool

@dataclass
class MyobandConfig:
    host: str
    port: int
    verbose: bool
    enable: bool

@dataclass
class MyoUdpConfig:
    ip: str
    port1: int
    port2: int
    sample_freq: float
    input_freq: float
    verbose: bool
    enable: bool

@dataclass
class KinectConfig:
    verbose: bool
    enable: bool

@dataclass
class OrchestratorConfig:
    verbose: bool
    polling_period: int

@dataclass
class Configs:
    drone: DroneConfig
    robot_arm: RobotArmConfig
    myoband: MyobandConfig
    kinect: KinectConfig
    orchestrator: OrchestratorConfig
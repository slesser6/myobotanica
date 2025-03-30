# myobotanica

To create the venv, run: `python -m venv venv`

To activate the venv run: `source venv/bin/activate` (or `venv\Scripts\activate` on Windows)

To install the requirements in the venv run: `pip install -r requirements.txt`

To remake the diagram run `pyreverse src/ --output mmd` (Available formats are: .dot, .puml, .plantuml, .mmd, .html)

```mermaid
classDiagram
  class Classifier {
    classify()
  }
  class Configs {
    drone
    kinnect
    motion
    myoband
    robot_arm
  }
  class Drone {
    debug_mode
    ip
    port
    speed
    connect()
    disconnect()
    moveArm()
    sendCommand()
  }
  class DroneConfig {
    ip : str
    port : int
  }
  class Kinnect {
    ip
    port
    connect()
    disconnect()
    getPosition()
  }
  class KinnectConfig {
    ip : str
    port : int
  }
  class MotionConfig {
    debug_mode : int
    speed : str
  }
  class Myoband {
    ip
    port
    connect()
    disconnect()
    getData()
  }
  class MyobandConfig {
    ip : str
    port : int
  }
  class Orchestrator {
    classifier
    configs
    drone
    kinnect
    myoband
    connect()
    disconnect()
    poll_sensors()
    run_calculations(desired_pos)
    send_output(cmd, joint_positions)
  }
  class RobotArmConfig {
    base_height : float
    link_lengths : List[float]
  }
  Classifier --* Orchestrator : classifier
  Drone --* Orchestrator : drone
  Kinnect --* Orchestrator : kinnect
  Myoband --* Orchestrator : myoband
  Configs --* Orchestrator : configs
  DroneConfig --* Configs : drone
  KinnectConfig --* Configs : kinnect
  MotionConfig --* Configs : motion
  MyobandConfig --* Configs : myoband
  RobotArmConfig --* Configs : robot_arm
```
```mermaid
classDiagram
  class src {
  }
  class classifier {
  }
  class classifier {
  }
  class drone {
  }
  class drone {
  }
  class kinematics {
  }
  class forward {
  }
  class inverse {
  }
  class orchestrator {
  }
  class sensors {
  }
  class kinnect {
  }
  class myoband {
  }
  class utils {
  }
  class config_loader {
  }
  class dataclasses {
  }
  classifier --> classifier
  drone --> drone
  kinematics --> forward
  kinematics --> inverse
  orchestrator --> classifier
  orchestrator --> drone
  orchestrator --> kinematics
  orchestrator --> sensors
  orchestrator --> utils
  sensors --> kinnect
  sensors --> myoband
  utils --> config_loader
  config_loader --> dataclasses
  dataclasses --> dataclasses
```
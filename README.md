# myobotanica

To create the venv, run: `python -m venv venv`

To activate the venv run: `source venv/bin/activate` (or `venv\Scripts\activate` on Windows)

To install the requirements in the venv run: `pip install -r requirements.txt`

To remake the diagram run `pyreverse src/ --output mmd` (Available formats are: .dot, .puml, .plantuml, .mmd, .html)
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
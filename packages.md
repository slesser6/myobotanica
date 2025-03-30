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
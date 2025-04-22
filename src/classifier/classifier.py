from enum import Enum

class Classification(Enum):
    WRIST_ROT_IN = 1
    WRIST_ROT_OUT = 2
    ELBOW_FLEX = 3
    ELBOW_EXT = 4
    WRIST_FLEX = 5
    WRIST_EXT = 6
    GRASP = 7
    UNKNOWN = 8
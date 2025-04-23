from enum import Enum

class Classification(Enum):
    WRIST_FLEX_TURN_LEFT = 1
    WRIST_EXT_TURN_RIGHT = 2
    WRIST_ADD_ARM_DOWN = 3
    WRIST_ABD_ARM_UP = 4
    GRASP_SPRAY = 5
    UNKNOWN = 6
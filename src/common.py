from enum import Enum

class Behavior(Enum):
    REACTIVE = 1
    PURE_PURSUIT = 2
    FOLLOW = 3
    STOP = 4
    ONLY_REACTIVE = 5
    SAFETY_STOP = 6
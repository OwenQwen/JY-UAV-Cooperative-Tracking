from enum import Enum

class MissionState(Enum):
    WAIT_FOR_PX4 = 0
    PRESTREAM = 1
    ARMING = 2
    TAKEOFF = 3
    HOVER = 4
    TRANSIT = 5
    SEARCH = 6
    TRACK = 7
    RTL = 8
    COMPLETE = 9
    FAILSAFE = 10

class TargetObservation:
    def __init__(self, x=0.0, y=0.0, z=0.0, fresh=False, timestamp=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.timestamp = timestamp

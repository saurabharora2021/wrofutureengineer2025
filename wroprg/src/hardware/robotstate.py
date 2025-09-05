"""State of the Robot"""

from typing import NamedTuple


class RobotState(NamedTuple):
    """Container for the robot state."""
    front: float =0
    left: float = 0
    right: float =0
    camera_front:float = -1
    camera_left:float = -1
    camera_right:float = -1
    yaw: float = 0

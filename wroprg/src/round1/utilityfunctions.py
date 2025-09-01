"""
Utility functions for the Round 1.
"""
import logging
from typing import Optional
from round1.walker_helpers import EquiWalkerHelper
from utils.mat import mat_color
from utils import constants

logger = logging.getLogger(__name__)

def clamp_angle(val:float,max_angle:float)-> float:
    """Clamp the value between -max_angle and max_angle."""
    return float(max(min(val, max_angle), -max_angle))

def check_bottom_color(hardware_interface, colors) -> str | None:
    """Read the bottom color and return it if it matches the specified colors."""
    r, g, b, _ = hardware_interface.get_bottom_color_rgbi()
    color = mat_color(r, g, b)
    # logger.info("Current rgb: R=%d, G=%d, B=%d", r, g, b)
    # logger.info("Waiting for color: %s, current color: %s", colors, color)
    if color in colors:
        logger.info("Detected color: %s", color)
        return color
    else:
        # logger.info("Color not detected, current color: %s", color)
        return None


class WalkParameters:
    """Parameters for robot walking behavior.

    Groups related parameters to improve readability and maintainability.
    """

    def __init__(self,
                 # Required distance parameters
                 min_front: float,
                 def_left: float,
                 def_right: float,

                 # Required control parameter
                 gyro_default: float,

                 # Optional parameters with sensible defaults
                 speed: float = 50,
                 speed_check: bool = True,
                 min_left: float = 10,
                 min_right: float = 10,
                 force_change: bool = False,
                 weak_gyro: bool = False,
                 is_unknown_direction: bool = False,
                 camera_read: bool = False,
                 base_helper: Optional[EquiWalkerHelper] = None):
        """Initialize walk parameters.

        Args:
            min_front: Minimum front distance before stopping
            def_left: Default/target left distance
            def_right: Default/target right distance
            gyro_default: Default gyro/yaw angle to maintain
            speed: Default walking speed
            speed_check: Whether to check and adjust speed during turns
            min_left: Minimum safe left distance
            min_right: Minimum safe right distance
            force_change: Whether to force parameter changes
            weak_gyro: Whether to use reduced gyro influence
            is_unknown_direction: Whether direction is unknown
            camera_read: Whether to read camera data
            base_helper: Optional helper for walk calculations
        """
        # Distance parameters
        self.min_front = min_front
        self.def_left = def_left
        self.def_right = def_right
        self.min_left = min_left
        self.min_right = min_right

        # Control parameters
        self.gyro_default = gyro_default
        self.speed = speed
        self.speed_check = speed_check

        # Behavior parameters
        self.force_change = force_change
        self.weak_gyro = weak_gyro
        self.is_unknown_direction = is_unknown_direction
        self.camera_read = camera_read

        # Helper parameter
        self.base_helper = base_helper

    def is_close_wall(self) -> bool:
        """Check if the robot is close to a wall."""
        if self.min_left < 5 or self.min_right < 5:
            logger.warning("Minimum distances too small, may cause collisions")
            return True
        return False

    def make_equi_helper(self, base_helper: Optional[EquiWalkerHelper]=None) \
                            -> EquiWalkerHelper:

        """Factory for EquiWalkerHelper with consistent tuning."""
        if base_helper is not None:
            return base_helper

        if self.force_change is False:
            if self.weak_gyro is False:

                helper:EquiWalkerHelper = EquiWalkerHelper(
                    def_distance_left=self.def_left,
                    def_distance_right=self.def_right,
                    max_left_distance=constants.LEFT_DISTANCE_MAX,
                    max_right_distance=constants.RIGHT_DISTANCE_MAX,
                    def_turn_angle=self.gyro_default,
                    min_left=self.min_left,
                    min_right=self.min_right
                )
            else:
                #weak Gyro is true. use less of gyro weight
                helper = EquiWalkerHelper(
                    def_distance_left=self.def_left,
                    def_distance_right=self.def_right,
                    max_left_distance=constants.LEFT_DISTANCE_MAX,
                    max_right_distance=constants.RIGHT_DISTANCE_MAX,
                    def_turn_angle=self.gyro_default,
                    fused_gyro_weight=0.4,
                    kgyro=-4.0,
                    fused_distance_weight=0.5,
                    min_left=self.min_left,
                    min_right=self.min_right
                    )
        else:
            helper = EquiWalkerHelper(
                def_distance_left=self.def_left,
                def_distance_right=self.def_right,
                max_left_distance=constants.LEFT_DISTANCE_MAX,
                max_right_distance=constants.RIGHT_DISTANCE_MAX,
                def_turn_angle=self.gyro_default,
                kp=-3,
                fused_distance_weight=0.4,
                min_left=self.min_left,
                min_right=self.min_right
            )

        return helper

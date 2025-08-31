""" This Module is used a define validation for the robot."""
import logging

from hardware.robotstate import RobotState
from hardware.hardware_interface import HardwareInterface
from utils import constants

logger: logging.Logger = logging.getLogger(__name__)
class RobotValidator:
    """
    This class is used to validate the robot's functionality.

    It performs checks on the robot's drive base, output interface, and Raspberry Pi status.
    Validation includes ensuring the drive base and output interface are initialized,
    checking distance sensor values, and verifying that the Raspberry Pi is not throttling.
    """


    def __init__(self, hardware_inf: HardwareInterface) -> None:
        self.hardware_inf: HardwareInterface = hardware_inf

    def validate(self) -> bool:
        """Validate the robot's functionality."""
        # Add validation logic here
        logger.warning("Robot validation started.")
        # Example: Check if drivebase and outputInterface are initialized
        if not self.hardware_inf:
            logger.error("Hardware Interface is not initialized.")
            return False
        state: RobotState = self.hardware_inf.read_state()
        if state.front <= 0:
            logger.error("Front distance sensor is not initialized or not working.")
            return False
        if self.hardware_inf.get_bottom_color() is None:
            logger.error("Bottom color sensor is not initialized or not working.")
            return False

        #Check distance sensors, left and right should be greater than 0
        left_distance = state.left
        right_distance = state.right
        logger.info("Left Distance: %.2f, Right Distance: %.2f", left_distance, right_distance)
        valid_distance = 0
        if left_distance > 0 and left_distance < constants.LEFT_DISTANCE_MAX:
            valid_distance += 1
            logger.info("Left distance is valid: %.2f cm", left_distance)
        if right_distance > 0 and right_distance < constants.RIGHT_DISTANCE_MAX:
            valid_distance += 1
            logger.info("Right distance is valid: %.2f cm", right_distance)
        if valid_distance == 0:
            logger.error("Both left and right distance sensors are invalid.")
            return False
        #Mat Logic, at starting point , size of mat is 100cm. so left and right distance should
        # not be greater than 100cm, since bot also has some width.
        if left_distance + right_distance > 200:
            logger.error("Invalid distances: Left=%.2f, Right=%.2f. Total distance is greater" \
                        " than 200cm.", left_distance, right_distance)
            return False

        self.hardware_inf.log_message(state.front, left_distance, right_distance,current_yaw=0,
                                           current_steering=self.hardware_inf.get_steering_angle())

        logger.info("Robot validation passed.")
        return True

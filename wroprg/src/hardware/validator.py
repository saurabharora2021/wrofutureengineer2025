""" This Module is used a define validation for the robot."""
import subprocess
import logging

from hardware.hardware_interface import HardwareInterface

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
        if self.hardware_inf.get_front_distance() <= 0:
            logger.error("Front distance sensor is not initialized or not working.")
            return False
        if self.hardware_inf.get_bottom_color() is None:
            logger.error("Bottom color sensor is not initialized or not working.")
            return False

        #Check distance sensors, left and right should be greater than 0
        left_distance = self.hardware_inf.get_left_distance()
        right_distance = self.hardware_inf.get_right_distance()
        logger.info("Left Distance: %.2f, Right Distance: %.2f", left_distance, right_distance)
        if (left_distance <= 0 or right_distance <= 0 or
            left_distance >= self.hardware_inf.get_left_distance_max() or
              right_distance >= self.hardware_inf.get_right_distance_max()):
            logger.error("Invalid distances: Left=%.2f, Right=%.2f.",
                              left_distance, right_distance)
            return False
        #Mat Logic, at starting point , size of mat is 100cm. so left and right distance should
        # not be greater than 100cm, since bot also has some width.
        if left_distance + right_distance > 105:
            logger.error("Invalid distances: Left=%.2f, Right=%.2f. Total distance is greater" \
                        " than 105cm.", left_distance, right_distance)
            return False

        # Lets check if Raspberry Pi is not throttling
        # We need to check if the Raspberry Pi is throttling, which can happen due to overheating
        # or power issues.This needs to be done after the logger is set up, so we can log the
        # results.
        logger.info("Checking Raspberry Pi throttling status")
        self.check_throttling()

        logger.info("Robot validation passed.")
        return True

    def check_throttling(self) -> bool:
        """Checks if the Raspberry Pi is throttled using vcgencmd. Returns True if throttled, 
        False otherwise."""
        try:
            result = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True,
                                    text=True,check=False)
            if result.returncode == 0:
                throttled_hex = result.stdout.strip().split('=')[-1]
                throttled = int(throttled_hex, 16)
                if throttled != 0:
                    logger.error("Pi is throttled! get_throttled=%s", throttled_hex)
                    return True
                else:
                    logger.info("Pi is not throttled.")
                    return False
            else:
                logger.error("Failed to run vcgencmd get_throttled")
                return False
        except (subprocess.CalledProcessError, ValueError, OSError) as e:
            logger.error("Error checking throttling: %s", e)
            return False

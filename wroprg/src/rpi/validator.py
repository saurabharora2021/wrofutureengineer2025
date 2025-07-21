""" This Module is used a define validation for the robot."""
import subprocess
import logging

from rpi.rpi_interface import RpiInterface
from hat.legodriver import BuildHatDriveBase
class RobotValidator:
    """
    This class is used to validate the robot's functionality.

    It performs checks on the robot's drive base, output interface, and Raspberry Pi status.
    Validation includes ensuring the drive base and output interface are initialized,
    checking distance sensor values, and verifying that the Raspberry Pi is not throttling.
    """

    logger: logging.Logger = logging.getLogger(__name__)

    def __init__(self, drivebase: BuildHatDriveBase, hardware_inf: RpiInterface) -> None:
        self.drivebase: BuildHatDriveBase = drivebase
        self.hardware_inf: RpiInterface = hardware_inf

    def validate(self) -> bool:
        """Validate the robot's functionality."""
        # Add validation logic here
        self.logger.warning("Robot validation started.")
        # Example: Check if drivebase and outputInterface are initialized
        if not self.drivebase or not self.hardware_inf:
            self.logger.error("Drivebase or Output Interface is not initialized.")
            return False
        if self.drivebase.get_front_distance() <= 0:
            self.logger.error("Front distance sensor is not initialized or not working.")
            return False
        if self.drivebase.get_bottom_color() is None:
            self.logger.error("Bottom color sensor is not initialized or not working.")
            return False

        #Check distance sensors, left and right should be greater than 0
        left_distance = self.hardware_inf.get_left_distance()
        right_distance = self.hardware_inf.get_right_distance()
        self.logger.info("Left Distance: %s, Right Distance: %s", left_distance, right_distance)
        if (left_distance <= 0 or right_distance <= 0 or 
            left_distance >= self.hardware_inf.get_left_distance_max() or
              right_distance >= self.hardware_inf.get_right_distance_max()):
            self.logger.error("Invalid distances: Left=%s, Right=%s.",
                              left_distance, right_distance)
            return False
        # Lets check if Raspberry Pi is not throttling
        # We need to check if the Raspberry Pi is throttling, which can happen due to overheating
        # or power issues.This needs to be done after the logger is set up, so we can log the
        # results.
        self.logger.info("Checking Raspberry Pi throttling status")
        self.check_throttling()

        self.logger.info("Robot validation passed.")
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
                    self.logger.error("Pi is throttled! get_throttled=%s", throttled_hex)
                    return True
                else:
                    self.logger.info("Pi is not throttled.")
                    return False
            else:
                self.logger.error("Failed to run vcgencmd get_throttled")
                return False
        except (subprocess.CalledProcessError, ValueError, OSError) as e:
            self.logger.error("Error checking throttling: %s", e)
            return False

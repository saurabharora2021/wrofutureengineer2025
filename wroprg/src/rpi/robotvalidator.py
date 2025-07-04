from round1.Walker import Walker
from rpi.LoggerSetup import LoggerSetup 
from rpi.RpiInterface import RpiInterface
from rpi.ShutdownInterfaceManager import ShutdownInterfaceManager
from time import sleep
from hat.BuildHatDriveBase import BuildHatDriveBase
import logging

class RobotValidator:

    def check_throttling(self) -> bool:
        """Checks if the Raspberry Pi is throttled using vcgencmd. Returns True if throttled, False otherwise."""
        import subprocess
        try:
            result = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True)
            if result.returncode == 0:
                throttled_hex = result.stdout.strip().split('=')[-1]
                throttled = int(throttled_hex, 16)
                if throttled != 0:
                    self.logger.error(f"Pi is throttled! get_throttled={throttled_hex}")
                    return True
                else:
                    self.logger.info("Pi is not throttled.")
                    return False
            else:
                self.logger.error("Failed to run vcgencmd get_throttled")
                return False
        except Exception as e:
            self.logger.error(f"Error checking throttling: {e}")
            return False

    logger = logging.getLogger(__name__)
    """This class is used to validate the robot's functionality."""
    
    def __init__(self, drivebase:BuildHatDriveBase, outputInterface:RpiInterface):
        self.drivebase = drivebase
        self.outputInterface = outputInterface

    def validate(self)->bool:
        """Validate the robot's functionality."""
        # Add validation logic here
        self.logger.warning("Robot validation started.")
        # Example: Check if drivebase and outputInterface are initialized
        if not self.drivebase or not self.outputInterface:
            self.logger.error("Drivebase or Output Interface is not initialized.")
            return False
        self.logger.info("Robot validation completed successfully.")

        #Check distance sensors, left and right should be greater than 0
        left_distance = self.outputInterface.getLeftDistance()
        right_distance = self.outputInterface.getRightDistance()
        self.logger.info(f"Left Distance: {left_distance}, Right Distance: {right_distance}")
        if left_distance <= 0 or right_distance <= 0:
            self.logger.error(f"Invalid distances: Left={left_distance}, Right={right_distance}.")
            return False
        # Lets check if Raspberry Pi is not throttling
        # We need to check if the Raspberry Pi is throttling, which can happen due to overheating or power issues.
        # This needs to be done after the logger is set up, so we can log the results.
        self.logger.info("Checking Raspberry Pi throttling status")        
        self.check_throttling()

        self.logger.info("Robot validation passed.")
        return True
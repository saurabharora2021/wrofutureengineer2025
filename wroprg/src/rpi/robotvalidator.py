from round1.Walker import Walker
from rpi.LoggerSetup import LoggerSetup 
from rpi.RpiInterface import RpiInterface
from rpi.ShutdownInterfaceManager import ShutdownInterfaceManager
from time import sleep
from hat.BuildHatDriveBase import BuildHatDriveBase
import logging

class robotValidator:

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
        if left_distance <= 0 or right_distance <= 0:
            self.logger.error(f"Invalid distances: Left={left_distance}, Right={right_distance}.")
            return False

        return True
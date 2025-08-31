""" This script is used to test distance sensor using the BuildHatDriveBase class."""
import logging
from hardware.validator import RobotValidator
from hardware.hardware_interface import HardwareInterface
from utils.helpers import HelperFunctions

def main():
    """ Main function to run the Wro - raspberry test distance sensor Application."""

    helper: HelperFunctions = HelperFunctions(stabilize=True)
    logger = logging.getLogger(__name__)
    pi_inf: HardwareInterface = helper.get_pi_interface()

    def runner():

        robot_validator:RobotValidator =  RobotValidator(pi_inf)
        robot_validator.validate()  # Validate the robot's functionality

        state = pi_inf.read_state()  # Log the distances
        logger.warning("F:%.2f, L:%.2f, R:%.2f, Y:%.2f , ",
                        state.front, state.left, state.right, state.yaw)

        pi_inf.force_flush_messages()

    helper.start_application(runner)

if __name__ == "__main__":
    main()

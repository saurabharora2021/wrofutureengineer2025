""" This script is used to test distance sensor using the BuildHatDriveBase class."""
import logging
import argparse
from rpi.rpi_interface import RpiInterface
from rpi.validator import RobotValidator
from hat.legodriver import BuildHatDriveBase
from utils.helpers import HelperFunctions

def main():
    """ Main function to run the Wro - raspberry test distance sensor Application."""

    parser = argparse.ArgumentParser(description="Wro lego - test distance sensor Application")
    parser.add_argument('--logfile', type=str, default='application.log', help='Path to log file')
    # Added debug argument
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    args = parser.parse_args()

    helper: HelperFunctions = HelperFunctions(args.logfile, args.debug)
    logger = logging.getLogger(__name__)

    drive_base: BuildHatDriveBase
    pi_inf: RpiInterface = helper.get_pi_interface()

    try:

        drive_base = helper.buildhat_init()
        logger.info("Drive Base Initialized")

        robot_validator:RobotValidator =  RobotValidator(drive_base, pi_inf)
        robot_validator.validate()  # Validate the robot's functionality

        left_distance = pi_inf.get_left_distance()
        right_distance = pi_inf.get_right_distance()
        logger.warning("Left Distance: %d cm Right Distance: %d cm",
                     left_distance, right_distance)


        pi_inf.force_flush_messages()
        drive_base.get_bottom_color()

    except (ImportError, AttributeError, RuntimeError) as e:
        logger.error("Error Running Program")
        logger.error("Exception: %s",e)
        pi_inf.led1_red()
        pi_inf.buzzer_beep()
    finally:
        helper.shutdownall()

if __name__ == "__main__":
    main()

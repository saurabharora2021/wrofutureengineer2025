""" This script is used to reset the front wheel of a robot using the BuildHatDriveBase class."""
import logging
import argparse
from hardware.rpi_interface import RpiInterface
from hardware.legodriver import BuildHatDriveBase
from utils.helpers import HelperFunctions

def main():
    """ Main function to run the Wro - raspberry reset Front Wheel Application."""

    parser = argparse.ArgumentParser(description="Wro lego - reset Front Wheel Application")
    parser.add_argument('--logfile', type=str, default='application.log', help='Path to log file')
    # Added debug argument
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    args = parser.parse_args()

    helper: HelperFunctions = HelperFunctions(args.logfile, args.debug)
    logger = logging.getLogger(__name__)

    drive_base: BuildHatDriveBase
    pi_inf: RpiInterface= helper.get_pi_interface()

    try:

        drive_base = helper.buildhat_init()

        logger.info("Drive Base Initialized")
        pi_inf.force_flush_messages()
        drive_base.get_bottom_color()
        # ## Generate a random number for front wheel steering angle.
        # import random
        # rand_int = random.randint(-100, 100)
        # logger.warning("Random Steering Angle: %d", rand_int)
        # drive_base.turn_steering(rand_int)


    except (ImportError, AttributeError, RuntimeError) as e:
        logger.error("Error Running Program")
        logger.error("Exception: %s",e)
        pi_inf.led1_red()
        pi_inf.buzzer_beep()
    finally:
        helper.shutdown_all()

if __name__ == "__main__":
    main()

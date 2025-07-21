""" This script is used to reset the back wheel of a robot using the BuildHatDriveBase class."""
import logging
import argparse
from time import sleep
from rpi.rpi_interface import RpiInterface
from hat.legodriver import BuildHatDriveBase
from utils.helpers import HelperFunctions

def main():
    """ Main function to run the Wro - raspberry back wheel Application."""

    parser = argparse.ArgumentParser(description="Wro lego - back wheel Application")
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

        drive_base.runfront(100)

        sleep(10)  # Allow the motor to run for a while

        drive_base.stop()

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

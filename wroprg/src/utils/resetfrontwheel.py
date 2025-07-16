""" This script is used to reset the front wheel of a robot using the BuildHatDriveBase class."""
import logging
import argparse
from rpi.rpi_interface import RpiInterface
from hat.legodriver import BuildHatDriveBase
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
    pi_inf: RpiInterface

    try:

        pi_inf,drive_base = helper.hardware_init()

        logger.info("Drive Base Initialized")
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

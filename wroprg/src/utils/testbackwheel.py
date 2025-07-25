""" This script is used to reset the back wheel of a robot using the BuildHatDriveBase class."""
import logging
import argparse
from time import sleep
from hardware.hardware_interface import HardwareInterface
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

    pi_inf: HardwareInterface = helper.get_pi_interface()

    try:

        logger.info("Drive Base Initialized")

        pi_inf.drive_forward(100)

        sleep(10)  # Allow the motor to run for a while

        pi_inf.drive_stop()

        pi_inf.force_flush_messages()
        pi_inf.get_bottom_color()

    except (ImportError, AttributeError, RuntimeError) as e:
        logger.error("Error Running Program")
        logger.error("Exception: %s",e)
        pi_inf.led1_red()
        pi_inf.buzzer_beep()
    finally:
        helper.shutdown_all()

if __name__ == "__main__":
    main()

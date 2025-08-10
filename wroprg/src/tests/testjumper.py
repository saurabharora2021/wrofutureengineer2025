""" This script is used to test the Buzzer using the BuildHatDriveBase class."""
import logging
import argparse
from time import sleep
from hardware.hardware_interface import HardwareInterface
from utils.helpers import HelperFunctions

def main():
    """ Main function to run the Wro - raspberry test color Application."""

    parser = argparse.ArgumentParser(description="Wro lego - test color Application")
    parser.add_argument('--logfile', type=str, default='application.log', help='Path to log file')
    # Added debug argument
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    args = parser.parse_args()

    helper: HelperFunctions = HelperFunctions(args.logfile, args.debug,stabilize=False)
    logger = logging.getLogger(__name__)

    pi_inf: HardwareInterface = helper.get_pi_interface()
    try:

        logger.warning("Current Jumper State:%s", pi_inf.get_jumper_state())
        sleep(1)  # Wait for a second to ensure the state is read correctly
        pi_inf.force_flush_messages()

    except (ImportError, AttributeError, RuntimeError) as e:
        logger.error("Error Running Program")
        logger.error("Exception: %s",e)
        pi_inf.led1_red()
        pi_inf.buzzer_beep()
    finally:
        helper.shutdown_all()

if __name__ == "__main__":
    main()

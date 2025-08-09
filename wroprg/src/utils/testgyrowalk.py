""" This script is used to test distance sensor using the BuildHatDriveBase class."""
import logging
import argparse
from hardware.validator import RobotValidator
from hardware.hardware_interface import HardwareInterface
from round1.logicround1 import Walker
from round1.matintelligence import MatIntelligence
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

    pi_inf: HardwareInterface = helper.get_pi_interface()

    try:

        pi_inf.force_flush_messages()

        challenge1walker = Walker(pi_inf)
        pi_inf.start_measurement_recording()

        #action button.
        # pi_inf.wait_for_action()

        intel: MatIntelligence = MatIntelligence()
        challenge1walker.gyro_corner_walk(intel,turn_angle=60)

    except (ImportError, AttributeError, RuntimeError) as e:
        logger.error("Error Running Program")
        logger.error("Exception: %s",e)
        pi_inf.led1_red()
        pi_inf.buzzer_beep()
        raise
    finally:
        helper.shutdown_all()

if __name__ == "__main__":
    main()

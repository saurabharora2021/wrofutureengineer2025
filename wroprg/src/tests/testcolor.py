""" This script is used to test the color using the BuildHatDriveBase class."""
import logging
import argparse
import time
from utils.mat import mat_color
from utils.helpers import HelperFunctions
from hardware.hardware_interface import HardwareInterface

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

        logger.info("sleep 1 seconds before starting")
        pi_inf.buzzer_beep()
        time.sleep(1)
        pi_inf.buzzer_beep()
        logger.info("Starting color sensor test...")
        r, g, b, i ,*other = pi_inf.get_bottom_color_rgbi()

        logger.info("Bottom Color RGBI: R=%d, G=%d, B=%d, I=%d", r, g, b, i)
        logger.debug("Remaining data: %s", other)
        color = mat_color(r, g, b)
        logger.warning("Detected color: %s", color)

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

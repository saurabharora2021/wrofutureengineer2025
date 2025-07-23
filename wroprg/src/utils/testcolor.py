""" This script is used to test the color using the BuildHatDriveBase class."""
import logging
import argparse
from rpi.rpi_interface import RpiInterface
from hat.legodriver import BuildHatDriveBase
from utils.helpers import HelperFunctions
from round1.logicround1 import Walker

def main():
    """ Main function to run the Wro - raspberry test color Application."""

    parser = argparse.ArgumentParser(description="Wro lego - test color Application")
    parser.add_argument('--logfile', type=str, default='application.log', help='Path to log file')
    # Added debug argument
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    args = parser.parse_args()

    helper: HelperFunctions = HelperFunctions(args.logfile, args.debug)
    logger = logging.getLogger(__name__)

    drive_base: BuildHatDriveBase
    pi_inf: RpiInterface = helper.get_pi_interface()
    challenge1walker: Walker

    try:

        drive_base = helper.buildhat_init()
        logger.info("Drive Base Initialized")

        challenge1walker = Walker(drive_base, pi_inf)

        # challenge1walker.start_walk(nooflaps=1)

        r, g, b, i ,*other = drive_base.get_bottom_color_rgbi()

        logger.info("Bottom Color RGBI: R=%d, G=%d, B=%d, I=%d", r, g, b, i)
        logger.debug("Remaining data: %s", other)
        color = challenge1walker.mat_color(r, g, b)
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

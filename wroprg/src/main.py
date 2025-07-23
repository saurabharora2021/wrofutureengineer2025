"""Main application for the Wro - Raspberry Pi interface."""
import logging
import argparse
from round1.logicround1 import Walker
from rpi.rpi_interface import RpiInterface
from rpi.validator import RobotValidator
from hat.legodriver import BuildHatDriveBase
from utils.helpers import HelperFunctions


def main():
    """ Main function to run the Wro - raspberry Application."""

    parser = argparse.ArgumentParser(description="Wro lego - raspberry Application")
    parser.add_argument('--logfile', type=str, default='application.log', help='Path to log file')
    # Added debug argument
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    args = parser.parse_args()

    print(f"Log file: {args.logfile}")
    print(f"Debug mode: {args.debug}")  # Optional: print debug status

    helper: HelperFunctions = HelperFunctions(args.logfile, args.debug)
    logger = logging.getLogger(__name__)

    drive_base: BuildHatDriveBase
    pi_inf: RpiInterface = helper.get_pi_interface()

    try:

        drive_base = helper.buildhat_init()

        logger.info("Drive Base Initialized")
        pi_inf.force_flush_messages()

         # Validate the robot's functionality
        robot_validator: RobotValidator = RobotValidator(drive_base, pi_inf)
        if not robot_validator.validate():
            logger.error("Robot validation failed. Exiting.")
            pi_inf.led1_red()
            pi_inf.buzzer_beep()
            raise RuntimeError("Robot validation failed")
        else:
            pi_inf.led1_green()
            pi_inf.buzzer_beep()

        logger.warning("Test Successful")
        pi_inf.force_flush_messages()

        challenge1walker = Walker(drive_base, pi_inf)

        challenge1walker.start_walk(nooflaps=1)

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

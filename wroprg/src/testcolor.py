""" This script is used to reset the front wheel of a robot using the BuildHatDriveBase class."""
import logging
import argparse
from base.shutdown_handling import ShutdownInterfaceManager
from round1.logicround1 import Walker
from rpi.logger_setup import LoggerSetup
from rpi.rpi_interface import RpiInterface
from rpi.validator import RobotValidator
from hat.legodriver import BuildHatDriveBase

def main():
    """ Main function to run the Wro - raspberry reset Front Wheel Application."""

    parser = argparse.ArgumentParser(description="Wro lego - raspberry Application")
    parser.add_argument('--logfile', type=str, default='application.log', help='Path to log file')
    # Added debug argument
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    args = parser.parse_args()

    print(f"Log file: {args.logfile}")
    print(f"Debug mode: {args.debug}")  # Optional: print debug status


    # Initialize all the components
    shutdown_manager = ShutdownInterfaceManager()

    loggersetup = LoggerSetup()
    shutdown_manager.add_interface(loggersetup)
    logger = logging.getLogger(__name__)

    print("Starting Spike Remote Base application")
    print("Initializing Output Interface")

    pi_inf: RpiInterface = RpiInterface()
    shutdown_manager.add_interface(pi_inf)

    if args.debug:
        # Set log level to DEBUG if debug mode is enabled
        loggersetup.setup(inf=pi_inf, log_file=args.logfile, log_level=logging.DEBUG)
    else:
        loggersetup.setup(inf=pi_inf,log_file=args.logfile, log_level=logging.INFO)


    try:
       # Create an instance of BuildHatDriveBase
        drive_base: BuildHatDriveBase = BuildHatDriveBase(front_motor_port='D', back_motor_port='A',
                                                        bottom_color_sensor_port='C',
                                                        front_distance_sensor_port='B')
        shutdown_manager.add_interface(drive_base)

        logger.info("Drive Base Initialized")


        challenge1walker = Walker(drive_base, pi_inf)

        # challenge1walker.start_walk(nooflaps=1)

        r, g, b, i = drive_base.get_bottom_color_rgbi()
        logger.info("Bottom Color RGBI: R=%d, G=%d, B=%d, I=%d", r, g, b, i)
        color = challenge1walker.mat_color(r, g, b)

        logger.info("Detected color: %s", color)
        

        pi_inf.force_flush_messages()

    except Exception as e:
        logger.error("Error Running Program")
        logger.error("Exception: %s",e)
        pi_inf.led1_red()
        pi_inf.buzzer_beep()
        raise
    finally:
            # Finally, shutdown all interfaces
        logger.warning("Shutting down all interfaces")
        shutdown_manager.shutdown_all()

if __name__ == "__main__":
    main()

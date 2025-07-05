""" This script is used to reset the front wheel of a robot using the BuildHatDriveBase class."""
import logging
import argparse
from time import sleep
from base.shutdown_handling import ShutdownInterfaceManager
from rpi.logger_setup import LoggerSetup
from rpi.rpi_interface import RpiInterface
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


        pi_inf.force_flush_messages()

        drive_base.runfront(100)

        sleep(10)  # Allow the motor to run for a while

        drive_base.stop()

        color = drive_base.get_bottom_color()
        logger.warning("Bottom C=%s",color)
        distance = drive_base.get_front_distance()
        logger.warning("Front : %s cm",distance)
        pi_inf.force_flush_messages()

        pi_inf.led1_off()

    except Exception as e:
        logger.error("Error Running Program")
        logger.error("Exception: %s",e)
        pi_inf.led1_red()
        pi_inf.buzzer_beep()
        raise
    finally:
            # Finally, shutdown all interfaces
        logger.warning("Shutting down all interfaces")
        pi_inf.wait_for_action()

        shutdown_manager.shutdown_all()

if __name__ == "__main__":
    main()

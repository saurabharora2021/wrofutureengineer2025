from round1.Walker import Walker
from rpi.LoggerSetup import LoggerSetup 
from rpi.RpiInterface import RpiInterface
from rpi.ShutdownInterfaceManager import ShutdownInterfaceManager
from time import sleep
from hat.BuildHatDriveBase import BuildHatDriveBase
import rpi.RobotValidator as RobotValidator
import logging
import argparse


def main():

    parser = argparse.ArgumentParser(description="Wro lego - raspberry Application")
    parser.add_argument('--logfile', type=str, default='application.log', help='Path to log file')
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')  # <-- Added debug argument
    args = parser.parse_args()

    print(f"Log file: {args.logfile}")
    print(f"Debug mode: {args.debug}")  # Optional: print debug status


    # Initialize all the components
    shutdownManager = ShutdownInterfaceManager()

    loggersetup = LoggerSetup()
    shutdownManager.add_interface(loggersetup)
    logger = logging.getLogger(__name__)

    print("Starting Spike Remote Base application")
    print("Initializing Output Interface")

    piInterface: RpiInterface = RpiInterface()
    shutdownManager.add_interface(piInterface)

    if args.debug:
        loggersetup.setup(inf=piInterface, log_file=args.logfile, log_level=logging.DEBUG)  # Set log level to DEBUG if debug mode is enabled
    else:
        loggersetup.setup(inf=piInterface,log_file=args.logfile, log_level=logging.INFO)


    try:

        # Create an instance of BuildHatDriveBase
        drive_base: BuildHatDriveBase = BuildHatDriveBase(front_motor_port='D', back_motor_port='A', bottom_color_sensor_port='C', front_distance_sensor_port='B')
        shutdownManager.add_interface(drive_base)

        logger.info("Drive Base Initialized")

        # Validate the robot's functionality
        robot_validator = RobotValidator(drive_base, piInterface)
        if not robot_validator.validate():
            logger.error("Robot validation failed. Exiting.")
            piInterface.LED1_red()
            piInterface.buzzer_beep()
            raise Exception("Robot validation failed")
        else:
            piInterface.LED1_green()
            piInterface.buzzer_beep()

        logger.warning("Test Successful")

        piInterface.force_flush_messages()
        piInterface.wait_for_action()

        challenge1walker = Walker(drive_base, piInterface)

        challenge1walker.start_walk(nooflaps=1)

        # logger.warning("Button Pressed")
        # piInterface.buzzer_beep()


        # # Run the program
        # counter = 0
        # while counter < 10:
        #     logger.warning(f"Right : {piInterface.getRightDistance()} cm")            
        #     logger.warning(f"Left : {piInterface.getLeftDistance()} cm")
        #     piInterface.force_flush_messages()
        #     sleep(1)
        #     counter += 1
        

        # drive_base.runfront(100)
        #sleep(4)
        # drive_base.stop()



        color = drive_base.getBottomColor()
        logger.warning(f"Bottom C={color}")
        distance = drive_base.getFrontDistance()
        logger.warning(f"Front : {distance} cm")
        piInterface.force_flush_messages()

        piInterface.LED1_off()

    except Exception as e:
        logger.error("Error Running Program")
        logger.error(f"Exception: {e}")        
        piInterface.LED1_red()
        piInterface.buzzer_beep()        
        raise   
    finally:
            # Finally, shutdown all interfaces
        logger.warning("Shutting down all interfaces")
        piInterface.wait_for_action()

        shutdownManager.shutdown_all()

if __name__ == "__main__":
    main()

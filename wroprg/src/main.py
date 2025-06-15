from rpi.LoggerSetup import LoggerSetup 
from rpi.OutputInterface import OutputInterface
from rpi.ShutdownInterfaceManager import ShutdownInterfaceManager
from time import sleep
from hat.BuildHatDriveBase import BuildHatDriveBase
import logging



def main():

    # Initialize all the components
    shutdownManager = ShutdownInterfaceManager()

    loggersetup = LoggerSetup()
    shutdownManager.add_interface(loggersetup)
    loggersetup.setup()
    logger = logging.getLogger(__name__)

    logger.info("Starting Spike Remote Base application")
    logger.info("Initializing Output Interface")

    outputInterface: OutputInterface = OutputInterface()
    shutdownManager.add_interface(outputInterface)

    try:

        # Create an instance of BuildHatDriveBase
        drive_base: BuildHatDriveBase = BuildHatDriveBase(front_motor_port='D', back_motor_port='A', bottom_color_sensor_port='C', front_distance_sensor_port='B')
        shutdownManager.add_interface(drive_base)

        logger.info("Drive Base Initialized")
        outputInterface.LED1_green()
        outputInterface.buzzer_beep()

        outputInterface.display_message("Test Successful")
        outputInterface.force_flush_messages()
        outputInterface.wait_for_action()
        logger.info("Action button pressed, starting drive base operations")

        outputInterface.display_message("Button Pressed")
        outputInterface.buzzer_beep()


        # Run the program
        counter = 0
        while counter < 10:
            outputInterface.logAndDisplay(f"Right : {outputInterface.getRightDistance()} cm")            
            outputInterface.logAndDisplay(f"Left : {outputInterface.getLeftDistance()} cm")
            outputInterface.force_flush_messages()
            sleep(1)
            counter += 1
        

        # drive_base.runfront(100)
        sleep(4)
        # drive_base.stop()

        color = drive_base.getBottomColor()
        outputInterface.logAndDisplay(f"Bottom C={color}")
        distance = drive_base.getFrontDistance()
        outputInterface.logAndDisplay(f"Front : {distance} cm")
        outputInterface.force_flush_messages()

        outputInterface.LED1_off()

    except Exception as e:
        outputInterface.display_message(f"Exception: {e}",forceflush=True)
        logger.error("Error Running Program")
        outputInterface.LED1_red()
        outputInterface.buzzer_beep()
        logger.error("Shutting down due to error")
        raise   
    finally:
            # Finally, shutdown all interfaces
        print("Shutting down all interfaces")
        outputInterface.display_message("Shutting down",forceflush=True)
        shutdownManager.shutdown_all()
        logger.info("Shutting down all interfaces")

if __name__ == "__main__":
    main()

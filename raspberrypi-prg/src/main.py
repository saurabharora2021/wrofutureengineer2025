from rpi.LoggerSetup import LoggerSetup 
from rpi.OutputInterface import OutputInterface
from rpi.BatteryMonitor import BatteryMonitor
from rpi.ShutdownInterfaceManager import ShutdownInterfaceManager
from time import sleep
from base.DriveBase import DriveBase
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

    outputInterface.LED1_green()

    try:

        # Create an instance of SpikeRemoteBase
        #drive_base: SpikeRemoteBase = SpikeRemoteBase(front_motor_port='F', back_motor_port='E', bottom_color_sensor_port='C', front_distance_sensor_port='C',debug=False)
        drive_base: BuildHatDriveBase = BuildHatDriveBase(front_motor_port='B', back_motor_port='A', bottom_color_sensor_port='C', front_distance_sensor_port='D')
        shutdownManager.add_interface(drive_base)

        # drive_base.runfront(100)
        # sleep(10)
        # drive_base.stop()

        color = drive_base.getBottomColor()
        print(f"Bottom Color Detected: {color}")
        distance = drive_base.getFrontDistance()
        print(f"Front Distance Detected: {distance} cm")


    except:
        logger.error("Error Running Program")
        outputInterface.LED1_red()
        outputInterface.buzzer_on()
        logger.error("Shutting down due to error")
        raise   
    finally:
            # Finally, shutdown all interfaces
        shutdownManager.shutdown_all()
        logger.info("Shutting down all interfaces")



if __name__ == "__main__":
    main()

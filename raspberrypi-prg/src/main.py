from spikeremote.SpikeRemoteBase import SpikeRemoteBase
from rpi.LoggerSetup import LoggerSetup 
from rpi.OutputInterface import OutputInterface
from rpi.BatteryMonitor import BatteryMonitor
from rpi.ShutdownInterfaceManager import ShutdownInterfaceManager
from time import sleep


def main():

    # Initialize all the components
    shutdownManager = ShutdownInterfaceManager()

    logger = LoggerSetup()
    shutdownManager.add_interface(logger)
    logger.setup()
    logger.log("Starting Spike Remote Base application")
    logger.log("Initializing Output Interface")

    outputInterface: OutputInterface = OutputInterface()
    shutdownManager.add_interface(outputInterface)

    outputInterface.LED1_green()

    try:

        # Create an instance of SpikeRemoteBase
        drive_base: SpikeRemoteBase = SpikeRemoteBase(front_motor_port='F', back_motor_port='E', bottom_color_sensor_port='C', front_distance_sensor_port='C',debug=False)
        shutdownManager.add_interface(drive_base)

        drive_base.runfront(100)
        sleep(5)
        drive_base.stop()


    except:
        logger.log("Error Running Program")
        outputInterface.LED1_red()
        outputInterface.buzzer_on()
        logger.log("Shutting down due to error")
        raise   
    finally:
            # Finally, shutdown all interfaces
        shutdownManager.shutdown_all()
        logger.log("Shutting down all interfaces")



if __name__ == "__main__":
    main()

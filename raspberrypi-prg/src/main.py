from spikeserial.SpikeDriveBase import SpikeDriveBase
from base import DriveBase
from rpi import LoggerSetup
from rpi import OutputInterface
from rpi import BatteryMonitor
from rpi.ShutdownInterfaceManager import ShutdownInterfaceManager


def main():

    ## Initialize all the components
    shutdownManager = ShutdownInterfaceManager()

    # Create an instance of BuildHatDriveBase
    drive_base:DriveBase = SpikeDriveBase()

    logger: LoggerSetup = LoggerSetup()
    shutdownManager.add_interface(logger)
    logger.setup_logging()

    #outputInterface: OutputInterface = OutputInterface()
    #shutdownManager.add_interface(outputInterface)

    # Create an instance of BatteryMonitor
    #battery_monitor: BatteryMonitor = BatteryMonitor(drive_base, outputInterface)
    #shutdownManager.add_interface(battery_monitor)
    #battery_monitor.runMonitoring()    

    
    # Example usage: Move forward for 500mm
    #drive_base.straight(500)
    
    # Example usage: Turn 90 degrees
    #drive_base.turn(90)
    drive_base.message("Hello")

    #Finally, shutdown all interfaces
    shutdownManager.shutdown_all()

if __name__ == "__main__":
    main()
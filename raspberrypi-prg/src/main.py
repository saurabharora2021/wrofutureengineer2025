from spikeremote.SpikeRemoteBase import SpikeRemoteBase
from rpi.LoggerSetup import LoggerSetup 
from rpi.OutputInterface import OutputInterface
from rpi.BatteryMonitor import BatteryMonitor
from rpi.ShutdownInterfaceManager import ShutdownInterfaceManager
from time import sleep,time


def main():

    # Initialize all the components
    shutdownManager = ShutdownInterfaceManager()

    logger = LoggerSetup()
    shutdownManager.add_interface(logger)
    logger.setup()

    # Create an instance of SpikeRemoteBase
    drive_base: SpikeRemoteBase = SpikeRemoteBase(front_motor_port='F', back_motor_port='E', bottom_color_sensor_port='C', front_distance_sensor_port='C',debug=False)
    shutdownManager.add_interface(drive_base)


    outputInterface: OutputInterface = OutputInterface()
    shutdownManager.add_interface(outputInterface)

    # Create an instance of BatteryMonitor
    # battery_monitor: BatteryMonitor = BatteryMonitor(drive_base, outputInterface)
    # shutdownManager.add_interface(battery_monitor)
    # battery_monitor.runMonitoring()

    # Example usage: Move forward for 500mm
    # drive_base.straight(500)

    # Example usage: Turn 90 degrees
    # drive_base.turn(90)
    # battery = drive_base.batterylevel()
    # print(f"Battery level: {battery}%")


    outputInterface.buzzer_on()
    outputInterface.LED1_on()

    time.sleep(2)

    drive_base.write_text("Hello")
    drive_base.beep(300, 1000, 100)

    drive_base.runfront(50)
    sleep(2)
    drive_base.stop()

    # Finally, shutdown all interfaces
    shutdownManager.shutdown_all()


if __name__ == "__main__":
    main()

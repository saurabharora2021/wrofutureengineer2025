import threading
from base.DriveBase import DriveBase
import logging
from rpi.OutputInterface import OutputInterface
from base.ShutdownInterface import ShutdownInterface

class BatteryMonitor(ShutdownInterface):

    logger = logging.getLogger(__name__)

    def __init__(self, drive_base: DriveBase, output_interface: OutputInterface):
        """
        Initialize the BatteryMonitor class.
        :param drive_base: Instance of SpikeDriveBase to communicate with the hub.
        :param output_interface: Instance of OutputInterface to control LEDs.
        """
        self.drive_base = drive_base
        self.output_interface = output_interface
        self.shutdown = False


    def monitor_battery(self, threshold: int = 50):
        """
        Monitor the battery level and control LEDs based on the level.
        :param threshold: Battery percentage below which the low battery LED is turned on.
        :param interval: Time interval (in seconds) between battery checks.
        """
        
        # Get the battery level from the hub
        battery_level = self.drive_base.batterylevel()
        self.logger.info(f"Battery Level: {battery_level}%")

        # Control LEDs based on the battery level
        if battery_level < threshold:
            self.output_interface.set_low_battery_led(True)
        else:
            self.output_interface.set_low_battery_led(False)

        #TODO: add logic for RP UPS

    

    def runMonitoring(self, interval: int = 5,threshold: int = 50):
        """
        Run the battery monitoring in a loop.
        :param interval: Time interval (in seconds) between battery checks.
        """
        try:
            self.thread = threading.Thread(target=self.monitor_battery, args=(threshold,))
            self.thread.start()
            
        except Exception as e:
            self.logger.error(f"Error occurred: {e}")
        finally:
            self.shutdown()

    def shutdown(self):
        if (self.shutdown == False):
            self.shutdown = True
            try:
                self.thread.stop()
            except Exception as e:
                self.logger.error(f"Error occurred: {e}")
            
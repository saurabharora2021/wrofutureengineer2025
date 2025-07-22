""" This modules contains the utility functions for the WRO Future Engineer 2025 project."""
import logging
from base.shutdown_handling import ShutdownInterfaceManager
from rpi.logger_setup import LoggerSetup
from rpi.rpi_interface import RpiInterface
from hat.legodriver import BuildHatDriveBase


class HelperFunctions:
    """ A class containing helper functions for the WRO Future Engineer 2025 project."""

    _shutdown_manager: ShutdownInterfaceManager
    _pi_inf: RpiInterface
    _drive_base: BuildHatDriveBase
    _logger: logging.Logger

    def __init__(self,log_file: str, debugflag: str)-> None:
        """ Initialize the HelperFunctions Logger class."""

        # Initialize all the components
        self._shutdown_manager = ShutdownInterfaceManager()
        loggersetup = LoggerSetup()
        self._shutdown_manager.add_interface(loggersetup)

        print("Initializing Output Interface")
        self._pi_inf = RpiInterface()
        self._shutdown_manager.add_interface(self._pi_inf)

        print("Log file: %s",log_file)
        print("Debug mode:%s", debugflag)  # Optional: print debug status

        if debugflag:
            # Set log level to DEBUG if debug mode is enabled
            loggersetup.setup(inf=self._pi_inf, log_file=log_file, log_level=logging.DEBUG)
        else:
            loggersetup.setup(inf=self._pi_inf,log_file=log_file, log_level=logging.INFO)

        self._logger = logging.getLogger(__name__)
        print("Starting Logger successfully")

    def get_pi_interface(self) -> RpiInterface:
        """ Function to get the Raspberry Pi Interface."""
        return self._pi_inf

    def buildhat_init(self) -> BuildHatDriveBase:
        """ Function to initialize the system components."""

        self._logger.info("BuildHatDriveBase Initialization")

        # Create an instance of BuildHatDriveBase
        self._drive_base = BuildHatDriveBase(front_motor_port='D', back_motor_port='A',
                                                        bottom_color_sensor_port='C',
                                                        front_distance_sensor_port='B')
        self._shutdown_manager.add_interface(self._drive_base)

        self._logger.info("Drive Base Initialized")

        return self._drive_base


    def shutdownall(self):
        """ Function to shutdown all interfaces."""
        self._logger.warning("Shutting down all interfaces")
        self._shutdown_manager.shutdown_all()

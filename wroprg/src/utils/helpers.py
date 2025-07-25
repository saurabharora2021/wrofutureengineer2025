"""This module contains the utility functions for the WRO Future Engineer 2025 project."""
import logging
from base.shutdown_handling import ShutdownInterfaceManager
from rpi.logger_setup import LoggerSetup
from hardware.hardware_interface import HardwareInterface


class HelperFunctions:
    """A class containing helper functions for the WRO Future Engineer 2025 project."""

    def __init__(self, log_file: str, debugflag: bool) -> None:
        """Initialize the HelperFunctions Logger class."""

        # Initialize instance attributes
        self._shutdown_manager: ShutdownInterfaceManager
        self._hardware_interface: HardwareInterface
        self._logger: logging.Logger

        # Initialize all the components
        self._shutdown_manager = ShutdownInterfaceManager()
        loggersetup = LoggerSetup()
        self._shutdown_manager.add_interface(loggersetup)

        print("Log file: %s", log_file)
        print("Debug mode: %s", debugflag)  # Optional: print debug status

        if debugflag:
            # Set log level to DEBUG if debug mode is enabled
            loggersetup.setup(log_file=log_file, log_level=logging.DEBUG)
        else:
            loggersetup.setup(log_file=log_file, log_level=logging.INFO)

        self._logger = logging.getLogger(__name__)
        print("Starting Logger successfully")

        self._logger.warning("Initializing Output Interface")
        self._hardware_interface = HardwareInterface()
        self._shutdown_manager.add_interface(self._hardware_interface)

        loggersetup.add_screen_logger(self._hardware_interface)

        self._hardware_interface.full_initialization()

    def get_pi_interface(self) -> HardwareInterface:
        """Function to get the Raspberry Pi Hardware Interface."""
        return self._hardware_interface

    def shutdown_all(self) -> None:
        """Function to shutdown all interfaces."""
        try:
            self._logger.warning("Shutting down all interfaces")
            self._shutdown_manager.shutdown_all()
            self._logger.info("All interfaces shutdown successfully")
        except Exception as e:
            self._logger.error("Error during shutdown: %s", e)
            raise

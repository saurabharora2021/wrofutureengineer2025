"""This module contains the utility functions for the WRO Future Engineer 2025 project."""
import logging
from typing import Optional
from base.shutdown_handling import ShutdownInterfaceManager
from rpi.logger_setup import LoggerSetup
from rpi.rpi_interface import RpiInterface
from hat.legodriver import BuildHatDriveBase


class HelperFunctions:
    """A class containing helper functions for the WRO Future Engineer 2025 project."""

    def __init__(self, log_file: str, debugflag: bool) -> None:
        """Initialize the HelperFunctions Logger class."""

        # Initialize instance attributes
        self._shutdown_manager: ShutdownInterfaceManager
        self._pi_inf: RpiInterface
        self._drive_base: Optional[BuildHatDriveBase] = None
        self._logger: logging.Logger

        # Initialize all the components
        self._shutdown_manager = ShutdownInterfaceManager()
        loggersetup = LoggerSetup()
        self._shutdown_manager.add_interface(loggersetup)

        print("Initializing Output Interface")
        self._pi_inf = RpiInterface()
        self._shutdown_manager.add_interface(self._pi_inf)

        print("Log file: %s", log_file)
        print("Debug mode: %s", debugflag)  # Optional: print debug status

        if debugflag:
            # Set log level to DEBUG if debug mode is enabled
            loggersetup.setup(inf=self._pi_inf, log_file=log_file, log_level=logging.DEBUG)
        else:
            loggersetup.setup(inf=self._pi_inf, log_file=log_file, log_level=logging.INFO)

        self._logger = logging.getLogger(__name__)
        print("Starting Logger successfully")

    def get_pi_interface(self) -> RpiInterface:
        """Function to get the Raspberry Pi Interface."""
        return self._pi_inf

    def buildhat_init(self) -> BuildHatDriveBase:
        """Function to initialize the system components."""
        if self._drive_base is not None:
            self._logger.warning("Drive base already initialized")
            return self._drive_base

        self._logger.info("BuildHatDriveBase Initialization")

        try:
            # Create an instance of BuildHatDriveBase
            self._drive_base = BuildHatDriveBase(front_motor_port='D', back_motor_port='A',
                                               bottom_color_sensor_port='C',
                                               front_distance_sensor_port='B')
            self._shutdown_manager.add_interface(self._drive_base)
            self._logger.info("Drive Base Initialized")
            return self._drive_base
        except Exception as e:
            self._logger.error("Failed to initialize drive base: %s", e)
            raise RuntimeError(f"Drive base initialization failed: {e}") from e

    def get_drive_base(self) -> BuildHatDriveBase:
        """Function to get the drive base instance."""
        if self._drive_base is None:
            raise RuntimeError("Drive base not initialized. Call buildhat_init() first.")
        return self._drive_base

    def shutdown_all(self) -> None:
        """Function to shutdown all interfaces."""
        try:
            self._logger.warning("Shutting down all interfaces")
            self._shutdown_manager.shutdown_all()
            self._logger.info("All interfaces shutdown successfully")
        except Exception as e:
            self._logger.error("Error during shutdown: %s", e)
            raise

    def is_drive_base_initialized(self) -> bool:
        """Check if the drive base has been initialized."""
        return self._drive_base is not None

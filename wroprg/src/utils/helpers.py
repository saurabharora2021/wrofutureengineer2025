"""This module contains the utility functions for the WRO Future Engineer 2025 project."""
import logging
from base.shutdown_handling import ShutdownInterfaceManager
from base.logger_setup import LoggerSetup
from hardware.hardware_interface import HardwareInterface
from utils.pihealth import PiHealth

class HelperFunctions:
    """A class containing helper functions for the WRO Future Engineer 2025 project."""

    def __init__(self, log_file: str, debugflag: bool,stabilize:bool=True,
                                                screen_logger=True) -> None:
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
        self._hardware_interface = HardwareInterface(stabilize)
        self._shutdown_manager.add_interface(self._hardware_interface)

        if screen_logger:
            self.add_screen_logger(self._hardware_interface)
        else:
            self.add_new_logger(self._hardware_interface)

        self._hardware_interface.wait_for_ready()

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

    def add_screen_logger(self, inf: HardwareInterface) -> None:
        """Add a logger to display messages on the OLED screen."""
        logger = logging.getLogger()
        class ScreenOledHandler(logging.Handler):
            """ Inner class to handle logging to oled display."""
            def __init__(self, oled_interface: HardwareInterface):
                super().__init__()
                self.oled_control_interface = oled_interface

            def emit(self, record):
                msg = self.format(record)
                self.oled_control_interface.display_message(msg)
                if record.levelno >= logging.ERROR:
                    self.oled_control_interface.buzzer_beep()  # Beep on error messages

        # Add the custom handler to the logger
        oledscreen_handler = ScreenOledHandler(inf)
        oledscreen_handler.setLevel(logging.WARNING)  # Set the level for the Oled display
        oled_formatter = logging.Formatter("%(message)s")  # Format for the Oled display
        oledscreen_handler.setFormatter(oled_formatter)
        logger.addHandler(oledscreen_handler)

    def add_new_logger(self, inf: HardwareInterface) -> None:
        """Add a new logger for the OLED display."""
        logger = logging.getLogger()
        class ScreenNewOledHandler(logging.Handler):
            """ Inner class to handle logging to oled display."""
            def __init__(self, oled_interface: HardwareInterface):
                super().__init__()
                self.oled_control_interface = oled_interface

            def emit(self, record):
                msg = self.format(record)
                self.oled_control_interface.add_screen_logger_message([msg])
                if record.levelno >= logging.ERROR:
                    self.oled_control_interface.buzzer_beep()  # Beep on error messages

        # Add the custom handler to the logger
        oledscreen_handler = ScreenNewOledHandler(inf)
        oledscreen_handler.setLevel(logging.WARNING)  # Set the level for the Oled display
        oled_formatter = logging.Formatter("%(message)s")  # Format for the Oled display
        oledscreen_handler.setFormatter(oled_formatter)
        logger.addHandler(oledscreen_handler)

    def get_pi_health(self) -> PiHealth:
        """Function to get the Raspberry Pi health monitor."""
        return PiHealth(duration_seconds=5, threshold=75)

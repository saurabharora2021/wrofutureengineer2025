""" This Module is used the initialize the Logging SubSystem"""
import logging

from logging.handlers import RotatingFileHandler
from base.shutdown_handling import ShutdownInterface
from rpi.rpi_interface import RpiInterface

class LoggerSetup(ShutdownInterface):
    """Class defines the logger and Handlers"""

    # Get the logger instance
    logger: logging.Logger = logging.getLogger(__name__)

    def setup(self, inf: RpiInterface, log_file: str, log_level: int = logging.INFO,
              max_bytes: int = 1048576, backup_count: int = 3) -> None:
        """ Intializes the log interfaces """
        # Remove all handlers associated with the root logger object (to avoid duplicate logs)
        for handler in logging.root.handlers[:]:
            logging.root.removeHandler(handler)

        logger = logging.getLogger()
        logger.setLevel(logging.DEBUG)  # Capture all logs, handlers will filter

        # Rotating file handler for log_level and above

        file_handler = RotatingFileHandler(log_file, mode="a", maxBytes=max_bytes,
                                           backupCount=backup_count)
        file_handler.setLevel(logging.DEBUG)
        file_formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
        file_handler.setFormatter(file_formatter)
        logger.addHandler(file_handler)

        # Console handler for WARNING and above
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.WARNING)
        console_formatter = logging.Formatter("%(levelname)s - %(message)s")
        console_handler.setFormatter(console_formatter)
        logger.addHandler(console_handler)

        # define a custom handler using rpiInferface display_message method
        class RpiInterfaceHandler(logging.Handler):
            """ Inner class to handle logging to oled display."""
            def __init__(self, rpi_interface: RpiInterface):
                super().__init__()
                self.rpi_interface = rpi_interface

            def emit(self, record):
                msg = self.format(record)
                self.rpi_interface.display_message(msg)
                if record.levelno >= logging.ERROR:
                    self.rpi_interface.buzzer_beep()  # Beep on error messages

        # Add the custom handler to the logger
        rpi_handler = RpiInterfaceHandler(inf)
        rpi_handler.setLevel(logging.WARNING)  # Set the level for the RpiInterface display
        rpi_formatter = logging.Formatter("%(message)s")  # Format for the RpiInterface display
        rpi_handler.setFormatter(rpi_formatter)
        logger.addHandler(rpi_handler)
        self.logger.info("Logger setup complete with file: %s, level: %s", log_file, log_level)



    def shutdown(self) -> None:
        # Flush and close all handlers
        for handler in logging.getLogger().handlers:
            handler.flush()
            handler.close()

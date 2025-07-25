""" This Module is used the initialize the Logging SubSystem"""
import logging
from logging.handlers import RotatingFileHandler
from base.shutdown_handling import ShutdownInterface

class LoggerSetup(ShutdownInterface):
    """Class defines the logger and Handlers"""

    # Get the logger instance
    logger: logging.Logger = logging.getLogger(__name__)

    def setup(self, log_file: str, log_level: int = logging.INFO,
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
        file_handler.setLevel(logging.INFO)
        file_formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
        file_handler.setFormatter(file_formatter)
        logger.addHandler(file_handler)

        # Console handler for WARNING and above
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_formatter = logging.Formatter("%(levelname)s - %(message)s")
        console_handler.setFormatter(console_formatter)
        logger.addHandler(console_handler)


        self.logger.info("Logger setup complete with file: %s, level: %s", log_file, log_level)

    def shutdown(self) -> None:
        # Flush and close all handlers
        for handler in logging.getLogger().handlers:
            handler.flush()
            handler.close()

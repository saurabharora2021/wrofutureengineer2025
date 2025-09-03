""" This Module is used the initialize the Logging SubSystem"""
import logging
from logging.handlers import RotatingFileHandler
from base.shutdown_handling import ShutdownInterface

class LoggerSetup(ShutdownInterface):
    """Class defines the logger and Handlers"""

    MAX_BYTES = 1048576
    BACKUP_COUNT = 3
    LOG_LEVEL = logging.INFO
    LOG_FILE = "application.log"

    def setup(self) -> None:
        """ Intializes the log interfaces """
        # Remove all handlers associated with the root logger object (to avoid duplicate logs)
        for handler in logging.root.handlers[:]:
            logging.root.removeHandler(handler)

        logger = logging.getLogger()
        logger.setLevel(logging.DEBUG)  # Capture all logs, handlers will filter

        # Rotating file handler for log_level and above

        file_handler = RotatingFileHandler(self.LOG_FILE, mode="a", maxBytes=self.MAX_BYTES,
                           backupCount=self.BACKUP_COUNT)
        # Force rollover if previous log exists and is not empty
        try:
            if file_handler.stream and file_handler.stream.tell() > 0:
                file_handler.doRollover()
        except (OSError, ValueError, AttributeError):
            pass
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


        logger.info("Logger setup complete with file: %s, level: %s", self.LOG_FILE, \
                                                            self.LOG_LEVEL)

    def shutdown(self) -> None:
        # Flush and close all handlers
        for handler in logging.getLogger().handlers:
            handler.flush()
            handler.close()

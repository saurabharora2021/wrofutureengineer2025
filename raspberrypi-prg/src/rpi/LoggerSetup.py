import logging
from base.ShutdownInterface import ShutdownInterface

class LoggerSetup(ShutdownInterface):

    # Get the logger instance
    logger = logging.getLogger(__name__)
   
    def setup(self):
        # Configure the logger
        logging.basicConfig(
            level=logging.DEBUG,  # Set the logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
            format="%(asctime)s - %(levelname)s - %(message)s",  # Define the log message format
            filename="application.log",  # Log messages to a file
            filemode="a"  # Append to the log file (use 'w' to overwrite)
        )

    def shutdown(self):
        # Flush the log file
        for handler in self.logger.handlers:
            handler.flush()

import logging
from base.ShutdownInterface import ShutdownInterface


class LoggerSetup(ShutdownInterface):

    # Get the logger instance
    logger = logging.getLogger(__name__)

    def setup(self, log_file="application.log"):
        # Remove all handlers associated with the root logger object (to avoid duplicate logs)
        for handler in logging.root.handlers[:]:
            logging.root.removeHandler(handler)
        # Configure the logger to write to a file
        logging.basicConfig(
            level=logging.DEBUG,
            format="%(asctime)s - %(levelname)s - %(message)s",
            filename=log_file,  # Log messages to a file
            filemode="a"        # Append to the log file (use 'w' to overwrite)
        )

    def shutdown(self):
        # Flush and close all handlers
        for handler in logging.getLogger().handlers:
            handler.flush()
            handler.close()
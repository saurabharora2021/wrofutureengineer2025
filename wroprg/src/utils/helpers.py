"""This module contains the utility functions for the WRO Future Engineer 2025 project."""
import threading
import time
import logging
from typing import Callable
from base.shutdown_handling import ShutdownInterfaceManager
from base.logger_setup import LoggerSetup
from hardware.hardware_interface import HardwareInterface
from utils.pihealth import PiHealth

class HelperFunctions:
    """A class containing helper functions for the WRO Future Engineer 2025 project."""

    def __init__(self, stabilize:bool=False,screen_logger=True) -> None:
        """Initialize the HelperFunctions Logger class."""

        # Initialize instance attributes
        self._shutdown_manager: ShutdownInterfaceManager
        self._hardware_interface: HardwareInterface
        self._logger: logging.Logger

        # Initialize all the components
        self._shutdown_manager = ShutdownInterfaceManager()
        loggersetup = LoggerSetup()
        self._shutdown_manager.add_interface(loggersetup)

        loggersetup.setup()

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

        self._stop_event = threading.Event()
        self._is_running = False

    def start_application(self,call_func:Callable[[], None],use_button:bool=False,):
        """Function to start the main application logic in a separate thread."""

        if use_button:
            self._hardware_interface.force_flush_messages()
            self._hardware_interface.led1_green()
            self._hardware_interface.wait_for_action()
            self._hardware_interface.led1_off()
            self._hardware_interface.buzzer_beep()
            self._logger.info("Button pressed.")
            self._hardware_interface.force_flush_messages()

        # We record the start time *after* the initial button press.
        start_time = time.time()
        def runner():
            try:
                self._is_running = True
                self._logger.info("===============STARTING BOT===================")
                call_func()

            except (ImportError, AttributeError, RuntimeError) as e:
                self._logger.error("Error Running Program")
                self._logger.error("Exception: %s",e)
                self._hardware_interface.led1_red()
                self._hardware_interface.buzzer_beep()
            finally:
                #shutdown
                self.shutdown_all()
                self._stop_event.set()
                self._is_running = False


        def on_button_pressed(*args, **kwargs):
            """Button callback: set stop flag and trigger shutdown."""
            # Grace period: Ignore any button press within the first 0.5 seconds
            # to prevent the start-up press from causing an immediate shutdown.
            if time.time() - start_time < 0.5:
                self._logger.debug("Ignoring button press during grace period.")
                return

            if not self._is_running:
                return # Avoid multiple shutdown calls

            try:
                self._logger.info("Stop button pressed, initiating shutdown.")
                self._is_running = False
                self._stop_event.set()
                # Directly call shutdown_all to stop all hardware and interfaces.
                self.shutdown_all()
            except Exception as e:
                # Avoid raising from the callback; just log
                self._logger.exception("Exception in on_button_pressed: %s", e)

        run_thread = threading.Thread(target=runner)

        # register the button callback before starting the worker so presses are handled immediately
        self._hardware_interface.register_button_press(on_button_pressed)

        run_thread.start()
        # wait for button to settle down.
        time.sleep(1)

        while self._is_running:
            if self._stop_event.is_set():
                break
            self._stop_event.wait(0.1)  # Add a timeout to allow periodic checking

        end_time = time.time()
        self._logger.info("==========Bot Stopped=============")
        self._logger.info("Total time: %s seconds", end_time - start_time)

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

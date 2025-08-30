"""This module contains threading functions"""
import threading
import time
import logging
from typing import Callable

from base.shutdown_handling import ShutdownInterface

logger = logging.getLogger(__name__)
class ConditionCheckerThread(threading.Thread):
    """Thread to periodically check a condition and execute a callback function.
       Shuts down after callback is called once.
    """
    def __init__(self, value_check_func, callback_func, interval_ms=1):
        """
        :param value_check_func: Function to check the condition
        :param callback_func: Function to call when the condition is met
        :param interval_ms: Interval in milliseconds to check the condition
        """
        super().__init__()
        self.value_check_func = value_check_func
        self.callback_func = callback_func
        self.interval = interval_ms / 1000.0
        self._stop_event = threading.Event()
        self._is_running = False

    def run(self):
        """Run the thread to check the condition and call the callback function."""
        self._is_running = True
        while not self._stop_event.is_set():
            value = self.value_check_func()
            if value is not None:
                self.callback_func(value)
                self.stop()  # Stop the thread after callback
                break
            time.sleep(self.interval)
        self._is_running = False

    def stop(self):
        """Stop the thread."""
        self._stop_event.set()
        self._is_running = False

    def is_running(self):
        """Check if the thread is currently running."""
        return self._is_running

class ConstantUpdateThread(threading.Thread, ShutdownInterface):
    """Thread to periodically check a condition and execute a callback function.
       Shuts down after callback is called once.
    """
    def __init__(self, call_func:Callable[[], None], interval_ms:float=1):
        """
        :param call_func: Function to call periodically
        :param interval_ms: Interval in milliseconds to call the function
        """
        super().__init__()
        self.call_func = call_func
        self.interval = interval_ms / 1000.0
        self._stop_event = threading.Event()
        self._is_running = False

    def run(self):
        """Run the thread to check the condition and call the callback function."""
        self._is_running = True
        while not self._stop_event.is_set():
            try:
                self.call_func()
            except Exception as e:  # pylint: disable=broad-except
                logger.error(f"Error in ConstantUpdateThread: {e}")
            time.sleep(self.interval)
        self._is_running = False

    def stop(self):
        """Stop the thread."""
        self._stop_event.set()
        self._is_running = False

    def is_running(self):
        """Check if the thread is currently running."""
        return self._is_running

    def shutdown(self):
        return self.stop()

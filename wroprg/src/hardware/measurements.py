""" This class defines the class to keep the measures of hardware sensors and store them. """
from collections import deque
import threading
import time
import logging
import os

from hardware.hardware_interface import HardwareInterface
from base.shutdown_handling import ShutdownInterface

logger = logging.getLogger(__name__)

class Measurement:
    """Class to represent a single measurement from the hardware sensors."""
    def __init__(self, left_distance: float, right_distance: float, front_distance: float,
                    steering_angle: float, timestamp: float):
        self._left_distance = left_distance
        self._right_distance = right_distance
        self._front_distance = front_distance
        self._steering_angle = steering_angle
        self._timestamp = timestamp

    @property
    def left_distance(self) -> float:
        """Get the left distance measurement."""
        return self._left_distance

    @property
    def right_distance(self) -> float:
        """Get the right distance measurement."""
        return self._right_distance

    @property
    def front_distance(self) -> float:
        """Get the front distance measurement."""
        return self._front_distance

    @property
    def steering_angle(self) -> float:
        """Get the steering angle measurement."""
        return self._steering_angle

    @property
    def timestamp(self) -> float:
        """Get the timestamp of the measurement."""
        return self._timestamp

    def __repr__(self):
        return (
            f"Measurement("
            f"left_distance={self.left_distance}, "
            f"right_distance={self.right_distance}, "
            f"front_distance={self.front_distance}, "
            f"steering_angle={self.steering_angle}, "
            f"timestamp={self.timestamp})"
        )


class MeasurementsManager(ShutdownInterface):
    """Class to read and store measurements from hardware sensors in a separate thread."""
    def __init__(self, hardware_interface: HardwareInterface):
        self.measurements: deque[Measurement] = deque(maxlen=5)  # Store last 5 measurements
        self._reading_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._hardware_interface = hardware_interface
        self._mlogger = MeasurementsLogger()

    def add_measurement(self, measurement: Measurement) -> None:
        """Add a new measurement to the list."""
        self.measurements.append(measurement)
        self._mlogger.write_measurement(measurement)
        logger.debug("Added measurement: %s", measurement)


    def get_latest_measurement(self) -> Measurement | None:
        """Get the latest measurement."""
        if self.measurements:
            return self.measurements[-1]
        return None

    def _read_hardware_loop(self) -> None:
        """Thread target: read hardware every 0.5 seconds."""
        while not self._stop_event.is_set():
            if self._hardware_interface is not None:
                left = self._hardware_interface.get_left_distance()
                right = self._hardware_interface.get_right_distance()
                front = self._hardware_interface.get_front_distance()
                steering_angle = self._hardware_interface.get_steering_angle()
                timestamp = time.time()
                measurement = Measurement(left, right, front, steering_angle, timestamp)
                self.add_measurement(measurement)
            time.sleep(0.5)

    def start_reading(self) -> None:
        """Start the background thread for reading hardware."""
        if self._reading_thread is None or not self._reading_thread.is_alive():
            self._stop_event.clear()
            self._reading_thread = threading.Thread(target=self._read_hardware_loop, daemon=True)
            self._reading_thread.start()

    def stop_reading(self) -> None:
        """Stop the background thread for reading hardware."""
        self._stop_event.set()
        if self._reading_thread is not None:
            self._reading_thread.join()
            self._reading_thread = None

    def shutdown(self) -> None:
        """Shutdown the reader and stop the reading thread."""
        self.stop_reading()
        self.measurements.clear()
        self._mlogger.close_file()
        logger.info("MeasurementsReader shutdown complete.")


class MeasurementsLogger:
    """Class to manage writing measurements to a csv file."""

    def __init__(self):
        self.filename = "measurements.csv" # Default filename
        self._file = None
        self.open_file()

    def open_file(self) -> None:
        """Open the file for writing measurements, rotating if it already exists."""
        if os.path.exists(self.filename):
            # Add a timestamp to the old file before rotating
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            rotated_name = f"measurements_{timestamp}.csv"
            os.rename(self.filename, rotated_name)
        self._file = open(self.filename, 'w', encoding='utf-8')
        self._file.write("left_distance,right_distance,front_distance,steering_angle,timestamp\n")

    def write_measurement(self, measurement: Measurement) -> None:
        """Write a single measurement to the file."""
        if self._file is not None:
            line = (
                f"{measurement.left_distance},"
                f"{measurement.right_distance},"
                f"{measurement.front_distance},"
                f"{measurement.steering_angle},"
                f"{measurement.timestamp}\n"
            )
            self._file.write(line)

    def close_file(self) -> None:
        """Close the file."""
        if self._file is not None:
            self._file.close()
            self._file = None

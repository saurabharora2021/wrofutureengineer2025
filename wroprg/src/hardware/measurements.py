""" This class defines the class to keep the measures of hardware sensors and store them. """
import time
import logging
import os
import json
from typing import Any, Dict, Optional
import threading
from base.shutdown_handling import ShutdownInterface
from hardware.robotstate import RobotState

logger = logging.getLogger(__name__)

class Measurement:
    """Class to represent a single measurement from the hardware sensors."""
    def __init__(self, left_distance: float, right_distance: float, front_distance: float,
                 steering_angle: float, yaw: float, timestamp: float,
                 extra_metrics: Optional[Dict[str, Any]] = None):
        self._left_distance = left_distance
        self._right_distance = right_distance
        self._front_distance = front_distance
        self._steering_angle = steering_angle
        self._yaw = yaw
        self._timestamp = timestamp
        # Store extra metrics as a dict; serialize to JSON on write
        self._extra_metrics: Dict[str, Any] = extra_metrics or {}

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

    @property
    def yaw(self) -> float:
        """Get the yaw angle measurement."""
        return self._yaw

    @property
    def extra_metrics(self) -> Dict[str, Any]:
        """Get extra metrics as a dictionary."""
        return self._extra_metrics

    def __repr__(self):
        return (
            f"Measurement("\
            f"left_distance={self.left_distance}, "\
            f"right_distance={self.right_distance}, "\
            f"front_distance={self.front_distance}, "\
            f"steering_angle={self.steering_angle}, "\
            f"yaw={self.yaw}, "\
            f"timestamp={self.timestamp}, "\
            f"extra_metrics={self.extra_metrics})"
        )


class MeasurementsLogger:
    """Class to manage writing measurements to a csv file."""

    def __init__(self):
        self.filename = "measurements.csv" # Default filename
        self._file = None

    def open_file(self) -> None:
        """Open the file for writing measurements, rotating if it already exists."""
        if os.path.exists(self.filename):
            # Add a timestamp to the old file before rotating
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            rotated_name = f"measurements_{timestamp}.csv"
            os.rename(self.filename, rotated_name)
        self._file = open(self.filename, 'w', encoding='utf-8')

    def writeheader(self) -> None:
        """Write the header to the measurements file."""
        self.open_file()
        if self._file is not None:
            self._file.write("left_distance,right_distance,front_distance,steering_angle,"\
                             "yaw,timestamp,extra_metrics\n")
            self._file.flush()
    def write_measurement(self, measurement: Measurement) -> None:
        """Write a single measurement to the file, rounding to 2 decimal places."""
        if self._file is not None:
            # Serialize extra metrics to a JSON string and CSV-escape quotes
            json_str = json.dumps(measurement.extra_metrics, separators=(',', ':'))
            json_escaped = json_str.replace('"', '""')
            line = (
                f"{measurement.left_distance:.2f},"
                f"{measurement.right_distance:.2f},"
                f"{measurement.front_distance:.2f},"
                f"{measurement.steering_angle:.2f},"
                f"{measurement.yaw:.2f},"
                f"{measurement.timestamp:.2f},"
                f"\"{json_escaped}\"\n"
            )
            self._file.write(line)
            self._file.flush()

    def write_comment(self, comment: str) -> None:
        """Write a comment to the measurements file."""
        if self._file is not None:
            self._file.write(f"# {comment}\n")
            self._file.flush()

    def close_file(self) -> None:
        """Close the file."""
        if self._file is not None:
            self._file.close()
            self._file = None

class MeasurementFileLog(ShutdownInterface):
    """Class to read and store measurements from hardware sensors in a separate thread."""

    ENABLE_MEASURE_LOG = False

    def __init__(self, hardware_interface: "HardwareInterface"):

        self._reading_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._hardware_interface = hardware_interface
        self._mlogger = MeasurementsLogger()

    def add_measurement(self, measurement: Measurement) -> None:
        """Add a new measurement to the list."""
        self._mlogger.write_measurement(measurement)
        logger.debug("Added measurement: %s", measurement)

    def add_comment(self, comment: str) -> None:
        """Add a comment to the measurements log."""
        self._mlogger.write_comment(comment)

    def _read_hardware_loop(self) -> None:
        """Thread target: read hardware every 0.5 seconds."""
        start_time = time.time()
        while not self._stop_event.is_set():
            if self._hardware_interface is not None:
                metrics: Dict[str, Any] = {}
                state: RobotState = self._hardware_interface.read_state()
                steering_angle = self._hardware_interface.get_steering_angle()
                yaw = self._hardware_interface.get_yaw()
                # Create a new measurement with the current timestamp
                timestamp = time.time()
                counter = int((timestamp - start_time)*1000)

                (_,_,_, metrics) = self._hardware_interface.camera_measurements.get_distance()

                measurement = Measurement(state.left, state.right, state.front,
                            steering_angle,
                            yaw, counter, extra_metrics=metrics)


                if  self.ENABLE_MEASURE_LOG:
                    self.add_measurement(measurement)

                self._hardware_interface.log_message(front=state.front, left=state.left,
                                                     right=state.right, current_yaw=yaw,
                                                     current_steering=steering_angle)


            time.sleep(1)

    def start_reading(self) -> None:
        """Start the background thread for reading hardware."""
        self._mlogger.writeheader()  # Write header to the measurements file
        if self._reading_thread is None or not self._reading_thread.is_alive():
            self._stop_event.clear()
            self._reading_thread = threading.Thread(target=self._read_hardware_loop, daemon=True)
            self._reading_thread.start()
            self._hardware_interface.disable_logger()

    def stop_reading(self) -> None:
        """Stop the background thread for reading hardware."""
        self._stop_event.set()
        if self._reading_thread is not None:
            self._reading_thread.join()
            self._reading_thread = None

    def shutdown(self) -> None:
        """Shutdown the reader and stop the reading thread."""
        self.stop_reading()
        self._mlogger.close_file()
        logger.info("MeasurementsReader shutdown complete.")

""" This class defines the class to keep the measures of hardware sensors and store them. """
import time
import logging
import os


logger = logging.getLogger(__name__)

class Measurement:
    """Class to represent a single measurement from the hardware sensors."""
    def __init__(self, left_distance: float, right_distance: float, front_distance: float,
                 steering_angle: float, roll: float, pitch: float, yaw: float, timestamp: float):
        self._left_distance = left_distance
        self._right_distance = right_distance
        self._front_distance = front_distance
        self._steering_angle = steering_angle
        self._roll = roll
        self._pitch = pitch
        self._yaw = yaw
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

    @property
    def roll(self) -> float:
        """Get the roll angle measurement."""
        return self._roll

    @property
    def pitch(self) -> float:
        """Get the pitch angle measurement."""
        return self._pitch

    @property
    def yaw(self) -> float:
        """Get the yaw angle measurement."""
        return self._yaw

    def __repr__(self):
        return (
            f"Measurement("\
            f"left_distance={self.left_distance}, "\
            f"right_distance={self.right_distance}, "\
            f"front_distance={self.front_distance}, "\
            f"steering_angle={self.steering_angle}, "\
            f"roll={self.roll}, "\
            f"pitch={self.pitch}, "\
            f"yaw={self.yaw}, "\
            f"timestamp={self.timestamp})"
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
                             "roll,pitch,yaw,timestamp\n")
            self._file.flush()
    def write_measurement(self, measurement: Measurement) -> None:
        """Write a single measurement to the file, rounding to 2 decimal places."""
        if self._file is not None:
            line = (
                f"{measurement.left_distance:.2f},"
                f"{measurement.right_distance:.2f},"
                f"{measurement.front_distance:.2f},"
                f"{measurement.steering_angle:.2f},"
                f"{measurement.roll:.2f},"
                f"{measurement.pitch:.2f},"
                f"{measurement.yaw:.2f},"
                f"{measurement.timestamp:.2f}\n"
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

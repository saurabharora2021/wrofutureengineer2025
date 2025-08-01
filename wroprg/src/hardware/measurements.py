""" This class defines the class to keep the measures of hardware sensors and store them. """
import time
import logging
import os


logger = logging.getLogger(__name__)

class Measurement:
    """Class to represent a single measurement from the hardware sensors."""
    def __init__(self, left_distance: float, right_distance: float, front_distance: float,
                    steering_angle: float, accel_x: float, accel_y: float, accel_z: float,
                    gyro_x: float, gyro_y: float, gyro_z: float, timestamp: float):
        self._left_distance = left_distance
        self._right_distance = right_distance
        self._front_distance = front_distance
        self._steering_angle = steering_angle
        self._accel_x = accel_x
        self._accel_y = accel_y
        self._accel_z = accel_z
        self._gyro_x = gyro_x
        self._gyro_y = gyro_y
        self._gyro_z = gyro_z
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
    def accel_x(self) -> float:
        """Get the x-axis acceleration measurement."""
        return self._accel_x
    @property
    def accel_y(self) -> float:
        """Get the y-axis acceleration measurement."""
        return self._accel_y
    @property
    def accel_z(self) -> float:
        """Get the z-axis acceleration measurement."""
        return self._accel_z
    @property
    def gyro_x(self) -> float:
        """Get the x-axis gyroscope measurement."""
        return self._gyro_x
    @property
    def gyro_y(self) -> float:
        """Get the y-axis gyroscope measurement."""
        return self._gyro_y
    @property
    def gyro_z(self) -> float:
        """Get the z-axis gyroscope measurement."""
        return self._gyro_z

    def __repr__(self):
        return (
            f"Measurement("
            f"left_distance={self.left_distance}, "
            f"right_distance={self.right_distance}, "
            f"front_distance={self.front_distance}, "
            f"steering_angle={self.steering_angle}, "
            f"accel_x={self.accel_x}, "
            f"accel_y={self.accel_y}, "
            f"accel_z={self.accel_z}, "
            f"gyro_x={self.gyro_x}, "
            f"gyro_y={self.gyro_y}, "
            f"gyro_z={self.gyro_z}, "
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
            self._file.write("left_distance,right_distance,front_distance,steering_angle,"
                             "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,timestamp\n")
            self._file.flush()
    def write_measurement(self, measurement: Measurement) -> None:
        """Write a single measurement to the file,rounding to 2 decimal places."""
        if self._file is not None:
            line = (
                f"{measurement.left_distance:.2f},"
                f"{measurement.right_distance:.2f},"
                f"{measurement.front_distance:.2f},"
                f"{measurement.steering_angle:.2f},"
                f"{measurement.accel_x:.2f},"
                f"{measurement.accel_y:.2f},"
                f"{measurement.accel_z:.2f},"
                f"{measurement.gyro_x:.2f},"
                f"{measurement.gyro_y:.2f},"
                f"{measurement.gyro_z:.2f},"
                f"{measurement.timestamp:.2f}\n"
            )
            self._file.write(line)
            self._file.flush()

    def close_file(self) -> None:
        """Close the file."""
        if self._file is not None:
            self._file.close()
            self._file = None

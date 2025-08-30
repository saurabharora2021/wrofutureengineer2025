"""Unified Hardware Interface for WRO Future Engineer 2025 project."""

import logging
from typing import List, Optional
from base.shutdown_handling import ShutdownInterface
from hardware.robotstate import RobotState
from hardware.legodriver import BuildHatDriveBase
from hardware.measurements import MeasurementFileLog
from hardware.orientation import OrientationEstimator
from hardware.rpi_interface import RpiInterface
from hardware.camerameasurements import CameraDistanceMeasurements

logger = logging.getLogger(__name__)

class HardwareInterface(ShutdownInterface):
    """
    Provides unified access to all hardware components.
    Includes methods for both LEGO driver and Raspberry Pi interface.
    """

    def __init__(self,stabilize:bool) -> None:

        self._lego_drive_base: Optional[BuildHatDriveBase] = None
        self._full_initialization()
        self._rpi = RpiInterface(stabilize)

        self._measurements_manager: Optional[MeasurementFileLog] = None


        self._orientation_estimator: Optional[OrientationEstimator] = None
        self._measurements_manager = MeasurementFileLog(self)

        self._orientation_estimator = OrientationEstimator(
                get_accel=self.get_acceleration,
                get_gyro=self.get_gyro,
                get_mag=self.get_magnetometer
        )

        self.camera_measurements = CameraDistanceMeasurements(
                                    self._rpi.get_camera())

    def clear_messages(self) -> None:
        """clear display messages"""
        if self._rpi is None:
            raise RuntimeError("Raspberry Pi interface not initialized." \
            " Call full_initialization() first.")
        else:
            return self._rpi.clear_messages()

    def _full_initialization(self) -> None:
        """Initialize all hardware components."""
        self._lego_drive_base = BuildHatDriveBase(front_motor_port='D', back_motor_port='B',
                                               bottom_color_sensor_port='C')

    def wait_for_ready(self):
        """Wait for complete hardware initialization."""
        if self._lego_drive_base is not None:
            self._lego_drive_base.wait_for_setup()


    def get_right_lidar_distance(self) -> float:
        """Get the distance from the right lidar sensor."""
        return self._rpi.get_right_lidar_distance()

    def get_left_lidar_distance(self) -> float:
        """Get the distance from the left lidar sensor"""
        return self._rpi.get_left_lidar_distance()

    def start_measurement_recording(self) -> None:
        """Start the measurements manager thread."""
        if self._measurements_manager is None:
            raise RuntimeError("Measurements manager not initialized. Call" \
                    " full_initialization() first.")

        if self._orientation_estimator is not None:
            self._orientation_estimator.start_readings()
        self._measurements_manager.start_reading()
        self.camera_measurements.start()

    def add_comment(self, comment: str) -> None:
        """Add a comment to the measurements log."""
        if self._measurements_manager is not None:
            self._measurements_manager.add_comment(comment)

    # --- Raspberry Pi Interface Methods ---

    def log_message(self, front: float, left: float, right: float, current_yaw: float,
                                                            current_steering: float) -> None:
        """Log the sensor readings and robot state."""
        if self._rpi is None:
            raise RuntimeError("Raspberry Pi interface not initialized.")
        self._rpi.log_message(front, left, right, current_yaw, current_steering)

    def get_orientation(self):
        """Get the current (roll, pitch, yaw) in degrees."""
        if self._orientation_estimator is None:
            raise RuntimeError("Orientation estimator not initialized.")
        return self._orientation_estimator.get_orientation()

    def buzzer_beep(self, timer: float = 0.5) -> None:
        """Turn on the buzzer."""
        self._rpi.buzzer_beep(timer)

    def led1_green(self) -> None:
        """Turn on the LED1 green."""
        self._rpi.led1_green()

    def led1_red(self) -> None:
        """Turn on the LED1 red."""
        self._rpi.led1_red()

    def led1_blue(self) -> None:
        """Turn on the LED1 blue."""
        self._rpi.led1_blue()

    def led1_white(self) -> None:
        """Turn on the LED1 white."""
        self._rpi.led1_white()

    def led1_off(self) -> None:
        """Turn off the LED1."""
        self._rpi.led1_off()

    def wait_for_action(self) -> None:
        """Wait for the action button to be pressed."""
        self._rpi.wait_for_action()

    def _get_right_distance(self) -> float:
        """Get the distance from the right distance sensor."""
        return self._rpi.get_right_distance()

    def _get_left_distance(self) -> float:
        """Get the distance from the left distance sensor."""
        return self._rpi.get_left_distance()

    def display_message(self, message: str, forceflush: bool = False) -> None:
        """
        Display a message on the OLED screen.

        Only the last 5 messages are shown on the display.
        """
        self._rpi.display_message(message, forceflush)

    def force_flush_messages(self) -> None:
        """Force flush the messages on the OLED screen."""
        self._rpi.force_flush_messages()

    def add_screen_logger_message(self, message: List[str]) -> None:
        """Add a message to the screen logger."""
        self._rpi.add_screen_logger_message(message)

    def get_jumper_state(self) -> bool:
        """Get the state of the jumper pin."""        
        return self._rpi.get_jumper_state()

    def shutdown(self) -> None:
        """Shutdown the hardware interface."""
        if self._lego_drive_base is None:
            logger.warning("LEGO Drive Base not initialized, skipping shutdown.")
        else:
            self._lego_drive_base.shutdown()
        self.camera_measurements.shutdown()

        self._rpi.shutdown()
        if self._orientation_estimator is not None:
            self._orientation_estimator.shutdown()
        if self._measurements_manager is not None:
            self._measurements_manager.shutdown()

    def reset_steering(self) -> None:
        """Reset the steering mechanism to its default position."""
        if self._lego_drive_base is None:
            raise RuntimeError("LEGO Drive Base not initialized. Call full_initialization() first.")
        self._lego_drive_base.reset_front_motor()

    def get_acceleration(self) -> tuple[float, float, float]:
        """Get the acceleration from the MPU6050 sensor."""
        return self._rpi.get_acceleration()

    def get_magnetometer(self) -> tuple[float, float, float]:
        """Get the magnetometer data from the QMC5883L sensor."""
        return self._rpi.get_magnetometer()

    def get_gyro(self) -> tuple[float, float, float]:
        """Get the gyroscope data from the MPU6050 sensor."""
        return self._rpi.get_gyro()

    def reset_gyro(self)-> None:
        """Reset the yaw angle to zero."""
        if self._orientation_estimator is not None:
            self._orientation_estimator.reset()
        else:
            raise RuntimeError("Orientation estimator not initialized.")

    def is_button_pressed(self):
        """Check if the action button is pressed."""
        if self._rpi is None:
            raise RuntimeError("Raspberry Pi interface not initialized.\
                                Call full_initialization() first.")
        else:
            return self._rpi.action_button.is_active

    # --- LEGO Driver Methods ---

    def camera_off(self) -> None:
        """Turn off the camera."""
        if self._lego_drive_base is None:
            raise RuntimeError("LEGO Drive Base not initialized. Call full_initialization() first.")
        self._lego_drive_base.camera_off()

    def camera_on(self) -> None:
        """Turn on the camera."""
        if self._lego_drive_base is None:
            raise RuntimeError("LEGO Drive Base not initialized. Call full_initialization() first.")
        self._lego_drive_base.camera_on()

    def drive_forward(self, speed: float) -> None:
        """Run the drive base forward at the specified speed."""
        if self._lego_drive_base is None:
            raise RuntimeError("LEGO Drive Base not initialized. Call full_initialization() first.")
        self._lego_drive_base.run_front(speed)

    def drive_backward(self, speed: float) -> None:
        """Run the drive base backward at the specified speed."""
        if self._lego_drive_base is None:
            raise RuntimeError("LEGO Drive Base not initialized. Call full_initialization() first.")
        self._lego_drive_base.run_front(-speed)

    def turn_steering(self, degrees: float, steering_speed: float=40) -> None:
        """
        Turn the steering by the specified degrees.
        Positive degrees turn right, negative turn left.
        Limits steering to +/-38 degrees.
        """
        if self._lego_drive_base is None:
            raise RuntimeError("LEGO Drive Base not initialized. Call full_initialization() first.")
        self._lego_drive_base.turn_steering(degrees, steering_speed)

    def drive_stop(self) -> None:
        """Stop the drive base."""
        if self._lego_drive_base is None:
            raise RuntimeError("LEGO Drive Base not initialized. Call full_initialization() first.")
        self._lego_drive_base.stop()

    def get_bottom_color(self) -> str:
        """Get the color detected by the bottom sensor."""
        if self._lego_drive_base is None:
            raise RuntimeError("LEGO Drive Base not initialized. Call full_initialization() first.")
        return self._lego_drive_base.get_bottom_color()

    def get_bottom_color_rgbi(self) -> list[float]:
        """Get the RGB values detected by the bottom sensor."""
        if self._lego_drive_base is None:
            raise RuntimeError("LEGO Drive Base not initialized. Call full_initialization() first.")
        return self._lego_drive_base.get_bottom_color_rgbi()

    def _get_front_distance(self) -> float:
        """Get the distance to the front obstacle in centimeter."""
        return self._rpi.get_front_distance()

    def get_steering_angle(self) -> float:
        """Get the current steering angle in degrees."""
        if self._lego_drive_base is None:
            raise RuntimeError("LEGO Drive Base not initialized. Call full_initialization() first.")
        return self._lego_drive_base.get_steering_angle()

    ## End of LEGO Driver Methods
    def read_state(self) -> RobotState:
        """Read the current state of the robot."""
        front = self._get_front_distance()
        left = self._get_left_distance()
        right = self._get_right_distance()
        yaw = self.get_orientation()[2]

        (camera_front, camera_left, camera_right, _) = self.camera_measurements.get_distance()
        return RobotState(front=front, left=left, right=right, yaw=yaw,
                        camera_front=camera_front,
                        camera_left=camera_left,
                        camera_right=camera_right)
        # return RobotState(front=front, left=left, right=right, yaw=yaw)

    def disable_logger(self) -> None:
        """Disable the logger."""
        self._rpi.disable_logger()

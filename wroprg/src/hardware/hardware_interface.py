"""Unified Hardware Interface for WRO Future Engineer 2025 project."""

from collections import deque
import threading
import logging
import time
from typing import List, NamedTuple, Optional
from typing import Any, Dict
from base.shutdown_handling import ShutdownInterface
from hardware.hardwareconfig import HardwareConfig
from hardware.pin_config import PinConfig
from hardware.legodriver import BuildHatDriveBase
from hardware.measurements import Measurement, MeasurementsLogger, logger
from hardware.orientation import OrientationEstimator
from hardware.rpi_interface import RpiInterface
from hardware.statsfunctions import DumpKalmanFilter
from hardware.camerameasurements import CameraDistanceMeasurements

logger = logging.getLogger(__name__)

class RobotState(NamedTuple):
    """Container for the robot state."""
    front: float =0
    left: float = 0
    right: float =0
    camera_front:float = 0
    camera_left:float = 0
    camera_right:float = 0
    yaw: float = 0
class HardwareInterface(ShutdownInterface):
    """
    Provides unified access to all hardware components.
    Includes methods for both LEGO driver and Raspberry Pi interface.
    """
    _camera_state: RobotState | None = None

    def __init__(self,stabilize:bool) -> None:
        self._rpi = RpiInterface(stabilize)
        self._lego_drive_base: Optional[BuildHatDriveBase] = None
        self._measurements_manager: Optional[MeasurementsManager] = None
        self._front_distance_kf: Optional[DumpKalmanFilter] = None

        self._orientation_estimator = None

    def set_camera_distance(self,state:RobotState):
        """Set the camera distance state."""
        self._camera_state = state

    def clear_messages(self) -> None:
        """clear display messages"""
        if self._rpi is None:
            raise RuntimeError("Raspberry Pi interface not initialized." \
            " Call full_initialization() first.")
        else:
            return self._rpi.clear_messages()

    def full_initialization(self) -> None:
        """Initialize all hardware components."""
        try:
            if HardwareConfig.CHASSIS_VERSION == 1:
                self._lego_drive_base = BuildHatDriveBase(front_motor_port='D', back_motor_port='A',
                                               bottom_color_sensor_port='C',
                                               front_distance_sensor_port='B')
            elif HardwareConfig.CHASSIS_VERSION == 2:
                self._lego_drive_base = BuildHatDriveBase(front_motor_port='D', back_motor_port='B',
                                               bottom_color_sensor_port='C',
                                               front_distance_sensor_port=None)
                self._measurements_manager = MeasurementsManager(self)
                self._front_distance_kf = DumpKalmanFilter(
                    process_variance=1e-2,      # Larger, as distance changes faster
                    measurement_variance=0.5,   # Moderate, as HC-SR04 is noisy
                    estimated_error=1.0,        # Start with high uncertainty
                    initial_value=self._rpi.get_front_distance(),
                    max_value=self._rpi.get_front_distance_max()
                      # Or your expected starting distance
                )
                self._orientation_estimator = OrientationEstimator(
                        get_accel=self.get_acceleration,
                        get_gyro=self.get_gyro,
                        get_mag=self.get_magnetometer
                )

            else:
                raise ValueError("Unsupported chassis version")

        except Exception as e:
            logger.error("Failed to initialize drive base: %s", e)
            raise RuntimeError(f"Drive base initialization failed: {e}") from e

    def start_measurement_recording(self) -> None:
        """Start the measurements manager thread."""
        if self._measurements_manager is None:
            raise RuntimeError("Measurements manager not initialized. Call" \
                    " full_initialization() first.")
        if self._orientation_estimator is not None:
            self._orientation_estimator.start_readings()
        self._measurements_manager.start_reading()

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

    def camera_enabled(self) -> bool:
        """Check if the camera is enabled."""
        return PinConfig.CAMERA_ENABLED

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

    def get_right_distance(self) -> float:
        """Get the distance from the right distance sensor."""
        return self._rpi.get_right_distance()

    def get_right_distance_max(self) -> float:
        """Get the maximum distance from the right distance sensor."""
        return self._rpi.get_right_distance_max()

    def _get_left_distance(self) -> float:
        """Get the distance from the left distance sensor."""
        return self._rpi.get_left_distance()

    def get_left_distance_max(self) -> float:
        """Get the maximum distance from the left distance sensor."""
        return self._rpi.get_left_distance_max()
    def get_front_distance_max(self) -> float:
        """Get the maximum distance from the front distance sensor."""
        if HardwareConfig.CHASSIS_VERSION == 1:
            raise ValueError("Chassis 1 doesnt support this function")
        elif HardwareConfig.CHASSIS_VERSION == 2:
            return self._rpi.get_front_distance_max()
        else:
            raise ValueError("Unsupported chassis version for front distance sensor.")

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
        if HardwareConfig.CHASSIS_VERSION == 1:
            raise ValueError("not Supported mpu6050")
        if HardwareConfig.CHASSIS_VERSION == 2:
            return self._rpi.get_acceleration()
        else:
            raise ValueError("Unsupported chassis version for acceleration sensor.")

    def get_gyro(self) -> tuple[float, float, float]:
        """Get the gyroscope data from the MPU6050 sensor."""
        if HardwareConfig.CHASSIS_VERSION == 1:
            raise ValueError("LEGO Drive Base not initialized. Call full_initialization()" \
                " first.")
        elif HardwareConfig.CHASSIS_VERSION == 2:
            return self._rpi.get_gyro()
        else:
            raise ValueError("Unsupported chassis version for gyroscope sensor.")

    def get_magnetometer(self):
        """Get the magnetometer instance."""
        if HardwareConfig.CHASSIS_VERSION == 1:
            raise ValueError("not Supported qmc5883l")
        elif HardwareConfig.CHASSIS_VERSION == 2:
            return self._rpi.get_magnetometer()
        else:
            raise ValueError("Unsupported chassis version for magnetometer.")

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

    def turn_steering(self, degrees: float, steering_speed: float=20) -> None:
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
        if HardwareConfig.CHASSIS_VERSION == 1:
            if self._lego_drive_base is None:
                raise ValueError("LEGO Drive Base not initialized. Call full_initialization()" \
                " first.")
            return self._lego_drive_base.get_front_distance()
        elif HardwareConfig.CHASSIS_VERSION == 2:
            if self._front_distance_kf is None:
                raise ValueError("Kalman filter for front distance not initialized.")
            return self._front_distance_kf.update(self._rpi.get_front_distance())
        else:
            raise ValueError("Unsupported chassis version for front distance sensor.")

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
        right = self.get_right_distance()
        yaw = self.get_orientation()[2]
        if self._camera_state is not None:
            return RobotState(front=front, left=left, right=right, yaw=yaw,
                              camera_front=self._camera_state.front,
                              camera_left=self._camera_state.left,
                              camera_right=self._camera_state.right)
        return RobotState(front=front, left=left, right=right, yaw=yaw)

    def disable_logger(self) -> None:
        """Disable the logger."""
        self._rpi.disable_logger()

class MeasurementsManager(ShutdownInterface):
    """Class to read and store measurements from hardware sensors in a separate thread."""

    camera_measurements:Optional[CameraDistanceMeasurements] = None
    def __init__(self, hardware_interface: HardwareInterface):
        self.measurements: deque[Measurement] = deque(maxlen=5)  # Store last 5 measurements
        self._reading_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._hardware_interface = hardware_interface
        self._mlogger = MeasurementsLogger()
        self._rpi = hardware_interface._rpi

    def add_measurement(self, measurement: Measurement) -> None:
        """Add a new measurement to the list."""
        self.measurements.append(measurement)
        self._mlogger.write_measurement(measurement)
        logger.debug("Added measurement: %s", measurement)

    def add_comment(self, comment: str) -> None:
        """Add a comment to the measurements log."""
        self._mlogger.write_comment(comment)

    def get_latest_measurement(self) -> Measurement | None:
        """Get the latest measurement."""
        if self.measurements:
            return self.measurements[-1]
        return None

    def _read_hardware_loop(self) -> None:
        """Thread target: read hardware every 0.5 seconds."""
        start_time = time.time()
        while not self._stop_event.is_set():
            if self._hardware_interface is not None:
                metrics: Dict[str, Any] = {}
                state: RobotState = self._hardware_interface.read_state()
                steering_angle = self._hardware_interface.get_steering_angle()
                roll, pitch, yaw = self._hardware_interface.get_orientation()
                # Create a new measurement with the current timestamp
                timestamp = time.time()
                counter = int((timestamp - start_time)*1000)

                if PinConfig.CAMERA_ENABLED:
                    (front,left,right, metrics) = self.camera_measurements.measure_distance(counter)
                    self._hardware_interface.set_camera_distance(RobotState(\
                            front=front,left=left,right=right,yaw=0.0))
                    state = self._hardware_interface.read_state()

                measurement = Measurement(state.left, state.right, state.front,
                            steering_angle,
                            roll, pitch, yaw,counter,extra_metrics=metrics)

                self.add_measurement(measurement)
                self._rpi.log_message(front=state.front, left=state.left,
                                                           right=state.right,current_yaw=yaw,
                                               current_steering=steering_angle)


            time.sleep(0.25)

    def start_reading(self) -> None:
        """Start the background thread for reading hardware."""
        self._mlogger.writeheader()  # Write header to the measurements file
        if self._reading_thread is None or not self._reading_thread.is_alive():
            self._stop_event.clear()
            self._reading_thread = threading.Thread(target=self._read_hardware_loop, daemon=True)
            self._reading_thread.start()
            self._hardware_interface.disable_logger()
            if PinConfig.CAMERA_ENABLED:
                self.camera_measurements = CameraDistanceMeasurements(
                                        self._rpi.get_camera())
                self.camera_measurements.start()


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

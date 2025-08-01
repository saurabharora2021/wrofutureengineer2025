"""Unified Hardware Interface for WRO Future Engineer 2025 project."""

from collections import deque
import threading
import logging
import time
from typing import Optional
from base.shutdown_handling import ShutdownInterface
from hardware.hardwareconfig import HardwareConfig
from hardware.legodriver import BuildHatDriveBase
from hardware.measurements import Measurement, MeasurementsLogger, logger
from hardware.rpi_interface import RpiInterface
from hardware.statsfunctions import DumpKalmanFilter, KalmanFilter

logger = logging.getLogger(__name__)

class HardwareInterface(ShutdownInterface):
    """
    Provides unified access to all hardware components.
    Includes methods for both LEGO driver and Raspberry Pi interface.
    """

    def __init__(self,stabilize:bool) -> None:
        self._rpi = RpiInterface(stabilize)
        self._lego_drive_base: Optional[BuildHatDriveBase] = None
        self._measurements_manager: Optional[MeasurementsManager] = None
        self._front_distance_kf: Optional[KalmanFilter] = None
        self._kf_accel: Optional[list[KalmanFilter]] = None
        self._kf_gyro: Optional[list[KalmanFilter]] = None

        ##Setup Kalman Filters for left and right distance sensors
        self._left_distance_kf = DumpKalmanFilter(
                        process_variance=0.2,      # Increased, so filter adapts faster
                        measurement_variance=0.5,   # Moderate, as HC-SR04 is noisy
                        estimated_error=1.0,        # Start with high uncertainty
                        initial_value=self._rpi.get_left_distance()
                        )
        self._right_distance_kf = DumpKalmanFilter(
                        process_variance=0.2,      # Increased, so filter adapts faster
                        measurement_variance=0.5,   # Moderate, as HC-SR04 is noisy
                        estimated_error=1.0,        # Start with high uncertainty
                        initial_value=self._rpi.get_right_distance()
                        )

    def full_initialization(self) -> None:
        """Initialize all hardware components."""
        try:
            if HardwareConfig.CHASSIS_VERSION == 1:
                self._lego_drive_base = BuildHatDriveBase(front_motor_port='D', back_motor_port='A',
                                               bottom_color_sensor_port='C',
                                               front_distance_sensor_port='B')
            elif HardwareConfig.CHASSIS_VERSION == 2:
                self._lego_drive_base = BuildHatDriveBase(front_motor_port='D', back_motor_port='A',
                                               bottom_color_sensor_port='C',
                                               front_distance_sensor_port=None)
                self._measurements_manager = MeasurementsManager(self)
                self._front_distance_kf = KalmanFilter(
                    process_variance=1e-2,      # Larger, as distance changes faster
                    measurement_variance=0.5,   # Moderate, as HC-SR04 is noisy
                    estimated_error=1.0,        # Start with high uncertainty
                    initial_value=self._rpi.get_front_distance()
                      # Or your expected starting distance
                )
                # Create Kalman filters for each axis
                self._kf_accel = [KalmanFilter() for _ in range(3)]
                self._kf_gyro = [KalmanFilter() for _ in range(3)]
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
        self._measurements_manager.start_reading()

    # --- Raspberry Pi Interface Methods ---
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

    def get_right_distance(self) -> float:
        """Get the distance from the right distance sensor."""
        return self._right_distance_kf.update(self._rpi.get_right_distance())

    def get_right_distance_max(self) -> float:
        """Get the maximum distance from the right distance sensor."""
        return self._rpi.get_right_distance_max()

    def get_left_distance(self) -> float:
        """Get the distance from the left distance sensor."""
        return self._left_distance_kf.update(self._rpi.get_left_distance())

    def get_left_distance_max(self) -> float:
        """Get the maximum distance from the left distance sensor."""
        return self._rpi.get_left_distance_max()

    def display_message(self, message: str, forceflush: bool = False) -> None:
        """
        Display a message on the OLED screen.

        Only the last 5 messages are shown on the display.
        """
        self._rpi.display_message(message, forceflush)

    def force_flush_messages(self) -> None:
        """Force flush the messages on the OLED screen."""
        self._rpi.force_flush_messages()

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
            if self._lego_drive_base is None:
                raise ValueError("LEGO Drive Base not initialized. Call full_initialization()" \
                " first.")
            return self._lego_drive_base.get_front_distance()
        elif HardwareConfig.CHASSIS_VERSION == 2:
            accel = self._rpi.get_acceleration()
            accel_filtered = [
                self._kf_accel[0].update(accel[0]),
                self._kf_accel[1].update(accel[1]),
                self._kf_accel[2].update(accel[2])
            ]
            return tuple(accel_filtered)
        else:
            raise ValueError("Unsupported chassis version for acceleration sensor.")

    def get_gyro(self) -> tuple[float, float, float]:
        """Get the gyroscope data from the MPU6050 sensor."""
        if HardwareConfig.CHASSIS_VERSION == 1:
            if self._lego_drive_base is None:
                raise ValueError("LEGO Drive Base not initialized. Call full_initialization()" \
                " first.")
            return self._lego_drive_base.get_front_distance()
        elif HardwareConfig.CHASSIS_VERSION == 2:
            gyro = self._rpi.get_gyro()
            gyro_filtered = [
                self._kf_gyro[0].update(gyro[0]),
                self._kf_gyro[1].update(gyro[1]),
                self._kf_gyro[2].update(gyro[2])
            ]
            return tuple(gyro_filtered)
        else:
            raise ValueError("Unsupported chassis version for gyroscope sensor.")

    # --- LEGO Driver Methods ---
    def drive_forward(self, speed: float) -> None:
        """Run the drive base forward at the specified speed."""
        if self._lego_drive_base is None:
            raise RuntimeError("LEGO Drive Base not initialized. Call full_initialization() first.")
        self._lego_drive_base.run_front(speed)

    def turn_steering(self, degrees: float,steering_speed:float=10) -> None:
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

    def get_front_distance(self) -> float:
        """Get the distance to the front obstacle in centimeter."""
        if HardwareConfig.CHASSIS_VERSION == 1:
            if self._lego_drive_base is None:
                raise ValueError("LEGO Drive Base not initialized. Call full_initialization()" \
                " first.")
            return self._lego_drive_base.get_front_distance()
        elif HardwareConfig.CHASSIS_VERSION == 2:
            return self._front_distance_kf.update(self._rpi.get_front_distance())
        else:
            raise ValueError("Unsupported chassis version for front distance sensor.")

    def get_steering_angle(self) -> float:
        """Get the current steering angle in degrees."""
        if self._lego_drive_base is None:
            raise RuntimeError("LEGO Drive Base not initialized. Call full_initialization() first.")
        return self._lego_drive_base.get_steering_angle()

    ## End of LEGO Driver Methods

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
                accel_x, accel_y, accel_z = self._hardware_interface.get_acceleration()
                gyro_x, gyro_y, gyro_z = self._hardware_interface.get_gyro()
                # Create a new measurement with the current timestamp
                timestamp = time.time()
                measurement = Measurement(left, right, front, steering_angle,
                                          accel_x, accel_y, accel_z,
                                          gyro_x, gyro_y, gyro_z, timestamp)
                self.add_measurement(measurement)
            time.sleep(0.25)

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

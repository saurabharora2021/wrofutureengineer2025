"""Unified Hardware Interface for WRO Future Engineer 2025 project.

This merges the previous Raspberry Pi interface into a single class so
all hardware access (LEGO + Raspberry Pi peripherals) is exposed here.
"""

import logging
import time
import math
from typing import List, Optional, Tuple
from board import SCL, SDA
import busio
import adafruit_ssd1306
import adafruit_tca9548a
import adafruit_vl53l0x
from gpiozero import Buzzer, RGBLED, DistanceSensor, Button, Device
from gpiozero.pins.pigpio import PiGPIOFactory
from PIL import Image, ImageDraw, ImageFont
from base.shutdown_handling import ShutdownInterface
from hardware.robotstate import RobotState
from hardware.legodriver import BuildHatDriveBase
from hardware.measurements import MeasurementFileLog
from hardware.orientation import OrientationEstimator
from hardware.camerameasurements import CameraDistanceMeasurements
from hardware.screenlogger import ScreenLogger
from hardware.camera import MyCamera
from utils import constants

logger = logging.getLogger(__name__)


class HardwareInterface(ShutdownInterface):
    """
    Provides unified access to all hardware components.
    Includes methods for both LEGO driver and Raspberry Pi peripherals.
    """

    # --- Constants and defaults (from former RpiInterface) ---
    MAX_STABILIZATION_CHECKS = 5
    LINE_HEIGHT = 10  # pixels per line
    FONT_SIZE = 10  # font size for messages
    LEFT_LASER_CHANNEL = 7
    RIGHT_LASER_CHANNEL = 2
    DEVICE_I2C_CHANNEL = 6
    DISTANCE_FUSION = True

    DISTANCE_SENSOR_DISTANCE = 12.5  # cm distance between sensors

    display_loglines = True

    BUZZER_PIN = 13
    LED1_RED_PIN = 12
    LED1_GREEN_PIN = 6
    LED1_BLUE_PIN = 5
    BUTTON_PIN = 19
    RIGHT_SENSOR_TRIG_PIN = 23
    RIGHT_SENSOR_ECHO_PIN = 21
    RIGHT_DISTANCE_MAX_DISTANCE = 2
    LEFT_SENSOR_TRIG_PIN = 20
    LEFT_SENSOR_ECHO_PIN = 24
    LEFT_DISTANCE_MAX_DISTANCE = 2
    FRONT_SENSOR_TRIG_PIN = 27
    FRONT_SENSOR_ECHO_PIN = 22
    FRONT_DISTANCE_MAX_DISTANCE = 2
    JUMPER_PIN = 26

    # Screen settings
    SCREEN_WIDTH = 128
    SCREEN_HEIGHT = 64
    SCREEN_UPDATE_INTERVAL = 0.5  # seconds
    LED_TEST_DELAY = 0.05  # seconds

    def __init__(self, stabilize: bool) -> None:
        # LEGO drive base initialization
        self._lego_drive_base: Optional[BuildHatDriveBase] = None
        self._full_initialization()

        # Raspberry Pi peripherals initialization (merged from RpiInterface)
        logger.info("Initializing Raspberry Pi peripherals...")

        # Setup I2C devices
        i2c = busio.I2C(SCL, SDA)
        tca = adafruit_tca9548a.TCA9548A(i2c)

        for channel in range(8):
            if tca[channel].try_lock():
                logger.info("Channel %s:", channel)
                addresses = tca[channel].scan()
                logger.info([hex(address) for address in addresses if address != 0x70])
                tca[channel].unlock()

        device_channel = tca[self.DEVICE_I2C_CHANNEL]

        # Sensors on I2C

        #first set orientation.
        self._orientation_estimator = OrientationEstimator(device_channel)

        # OLED display
        self.oled = adafruit_ssd1306.SSD1306_I2C(self.SCREEN_WIDTH, self.SCREEN_HEIGHT,\
                                                         device_channel)
        self.oled.fill(0)
        self.oled.show()

        self.font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", HardwareInterface.FONT_SIZE
        )
        self._last_oled_update: float = 0
        self.image: Image.Image = Image.new("1", (self.SCREEN_WIDTH, self.SCREEN_HEIGHT))
        self.draw: ImageDraw.ImageDraw = ImageDraw.Draw(self.image)

        self.pendingmessage: bool = False
        self.messages: List[str] = []

        self._screenlogger = ScreenLogger(width=self.SCREEN_WIDTH, height=self.SCREEN_HEIGHT)

        # Laser distance sensors via TCA channels
        left_channel = tca[self.LEFT_LASER_CHANNEL]
        right_channel = tca[self.RIGHT_LASER_CHANNEL]
        self.left_laser = adafruit_vl53l0x.VL53L0X(left_channel)
        self.left_laser.measurement_timing_budget = 200000
        self.right_laser = adafruit_vl53l0x.VL53L0X(right_channel)
        self.right_laser.measurement_timing_budget = 200000

        self.display_message("Initializing Pi Interface...")

        # GPIO via pigpio if available
        try:
            Device.pin_factory = PiGPIOFactory()
            logger.info("Using PiGPIOFactory for GPIO pin control.")
        except Exception:  # pylint: disable=broad-except
            Device.pin_factory = None
            logger.error("Failed to initialize PiGPIOFactory")

        # Actuators and buttons
        self.buzzer = Buzzer(self.BUZZER_PIN)
        self.led1 = RGBLED(
            red=self.LED1_RED_PIN,
            green=self.LED1_GREEN_PIN,
            blue=self.LED1_BLUE_PIN,
        )

        # LED test
        self.led1.color = (0, 1, 0)
        time.sleep(self.LED_TEST_DELAY)
        self.led1.color = (1, 0, 0)
        time.sleep(self.LED_TEST_DELAY)
        self.led1.color = (0, 0, 1)
        time.sleep(self.LED_TEST_DELAY)
        self.led1.color = (0, 0, 0)

        self.action_button = Button(self.BUTTON_PIN, hold_time=1)

        # Ultrasonic distance sensors
        self.rightdistancesensor = DistanceSensor(
            echo=self.RIGHT_SENSOR_ECHO_PIN,
            trigger=self.RIGHT_SENSOR_TRIG_PIN,
            partial=True,
            max_distance=self.RIGHT_DISTANCE_MAX_DISTANCE,
        )
        self.leftdistancesensor = DistanceSensor(
            echo=self.LEFT_SENSOR_ECHO_PIN,
            trigger=self.LEFT_SENSOR_TRIG_PIN,
            partial=True,
            max_distance=self.LEFT_DISTANCE_MAX_DISTANCE,
        )
        self.front_distance_sensor = DistanceSensor(
            echo=self.FRONT_SENSOR_ECHO_PIN,
            trigger=self.FRONT_SENSOR_TRIG_PIN,
            partial=True,
            max_distance=self.FRONT_DISTANCE_MAX_DISTANCE,
        )
        self.jumper_pin = Button(self.JUMPER_PIN, hold_time=1)

        # Camera
        self.camera = MyCamera()

        logger.info("Raspberry Pi peripherals initialized successfully.")

        if stabilize:
            logger.warning("Stabilize Distance Sensors...")
            time.sleep(0.5)
            counter = 0
            valid_distance = False
            while counter < self.MAX_STABILIZATION_CHECKS and not valid_distance:
                valid_distance = True
                if (self._get_right_distance() < 0.1 or
                        self._get_right_distance() >= constants.RIGHT_DISTANCE_MAX):
                    logger.info("Right distance sensor is not stable, %.2f cm", \
                                                        self._get_right_distance())
                    valid_distance = False
                if (self._get_left_distance() < 0.1 or
                        self._get_left_distance() >= constants.LEFT_DISTANCE_MAX):
                    logger.info("Left distance sensor is not stable, %.2f cm", \
                                                        self._get_left_distance())
                    valid_distance = False
                if not valid_distance:
                    logger.warning("Waiting for distance sensors to stabilize...")
                    time.sleep(1)
                counter += 1

        # Measurements
        self._measurements_manager: Optional[MeasurementFileLog] = MeasurementFileLog(self)

        self.camera_measurements = CameraDistanceMeasurements(self.camera)

    def _full_initialization(self) -> None:
        """Initialize all hardware components."""
        self._lego_drive_base = BuildHatDriveBase(
            front_motor_port='D', back_motor_port='B', bottom_color_sensor_port='C'
        )

    def wait_for_ready(self) -> None:
        """Wait for complete hardware initialization."""
        if self._lego_drive_base is not None:
            self._lego_drive_base.wait_for_setup()

    # --- LiDAR distances ---
    def get_right_lidar_distance(self) -> float:
        """Get the distance from the right lidar sensor."""
        return self.right_laser.range / 10.0

    def get_left_lidar_distance(self) -> float:
        """Get the distance from the left lidar sensor"""
        return self.left_laser.range / 10.0

    # --- Measurements and orientation management ---
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

    # --- Raspberry Pi Peripheral Methods ---
    def log_message(self, front: float, left: float, right: float, current_yaw: float,
                    current_steering: float) -> None:
        """Log the sensor readings and robot state."""
        image: Image.Image = self._screenlogger.log_message(
            front, left, right, current_yaw, current_steering
        )
        self._paint_display(image)

    def get_orientation(self) -> Tuple[float, float, float]:
        """Get the current (roll, pitch, yaw) in degrees."""
        if self._orientation_estimator is None:
            raise RuntimeError("Orientation estimator not initialized.")
        return self._orientation_estimator.get_orientation()

    def buzzer_beep(self, timer: float = 0.5) -> None:
        """Turn on the buzzer."""
        self.buzzer.on()
        time.sleep(timer)
        self.buzzer.off()

    def led1_green(self) -> None:
        """Turn on the LED1 green."""
        self.led1.color = (0, 1, 0)

    def led1_red(self) -> None:
        """Turn on the LED1 red."""
        self.led1.color = (1, 0, 0)

    def led1_blue(self) -> None:
        """Turn on the LED1 blue."""
        self.led1.color = (0, 0, 1)

    def led1_white(self) -> None:
        """Turn on the LED1 white."""
        self.led1.color = (1, 1, 1)

    def led1_off(self) -> None:
        """Turn off the LED1."""
        self.led1.color = (0, 0, 0)

    def wait_for_action(self) -> None:
        """Wait for the action button to be pressed."""
        logger.warning("Waiting for action button press...")
        self.action_button.wait_for_active()
        logger.info("Action button pressed!")

    # --- Distance helpers ---
    def _get_right_distance(self) -> float:
        """Get the distance from the right distance sensor."""
        ultrasonic = self.rightdistancesensor.distance * 100  # cm
        laser = self.right_laser.range / 10.0  # cm
        return self._fuse_sensors(laser, ultrasonic)

    def _get_left_distance(self) -> float:
        """Get the distance from the left distance sensor."""
        ultrasonic = self.leftdistancesensor.distance * 100  # cm
        laser = self.left_laser.range / 10.0  # cm
        return self._fuse_sensors(laser, ultrasonic)

    def _get_front_distance(self) -> float:
        """Get the distance to the front obstacle in centimeter."""
        return self.front_distance_sensor.distance * 100

    # --- Display helpers ---
    def display_message(self, message: str, forceflush: bool = False) -> None:
        """
        Display a message on the OLED screen.

        Only the last 5 messages are shown on the display.
        """
        now = time.time()
        self.messages.append(message)
        self.messages = self.messages[-5:]
        if forceflush or (now - self._last_oled_update >= self.SCREEN_UPDATE_INTERVAL):
            self._flush_pending_messages()
            self.pendingmessage = False

    def force_flush_messages(self) -> None:
        """Force flush the messages on the OLED screen."""
        if self.pendingmessage:
            self._flush_pending_messages()
            self.pendingmessage = False

    def add_screen_logger_message(self, message: List[str]) -> None:
        """Add a message to the screen logger."""
        self._screenlogger.add_message(message)

    def get_jumper_state(self) -> bool:
        """Get the state of the jumper pin."""
        return self.jumper_pin.is_active

    def shutdown(self) -> None:
        """Shutdown the hardware interface."""
        if self._lego_drive_base is None:
            logger.warning("LEGO Drive Base not initialized, skipping shutdown.")
        else:
            self._lego_drive_base.shutdown()
        self.camera_measurements.shutdown()

        # Shutdown Raspberry Pi peripherals
        try:
            self._flush_pending_messages()
        except Exception as e:  # pylint: disable=broad-except
            logger.error("Error flushing OLED messages during shutdown: %s", e)
        try:
            if self.camera is not None:
                self.camera.close()
        except Exception as e:  # pylint: disable=broad-except
            logger.error("Error closing camera during shutdown: %s", e)
        try:
            self.buzzer.off()
        except Exception as e:  # pylint: disable=broad-except
            logger.error("Error turning off buzzer during shutdown: %s", e)
        try:
            self.led1.color = (0, 0, 0)
        except Exception as e:  # pylint: disable=broad-except
            logger.error("Error turning off LED1 during shutdown: %s", e)
        try:
            self.led1.close()
        except Exception as e:  # pylint: disable=broad-except
            logger.error("Error closing LED1 during shutdown: %s", e)
        try:
            self.buzzer.close()
        except Exception as e:  # pylint: disable=broad-except
            logger.error("Error closing buzzer during shutdown: %s", e)
        try:
            self.rightdistancesensor.close()
        except Exception as e:  # pylint: disable=broad-except
            logger.error("Error closing right distance sensor during shutdown: %s", e)
        try:
            self.leftdistancesensor.close()
        except Exception as e:  # pylint: disable=broad-except
            logger.error("Error closing left distance sensor during shutdown: %s", e)
        try:
            self.front_distance_sensor.close()
        except Exception as e:  # pylint: disable=broad-except
            logger.error("Error closing front distance sensor during shutdown: %s", e)
        try:
            self.jumper_pin.close()
        except Exception as e:  # pylint: disable=broad-except
            logger.error("Error closing jumper pin during shutdown: %s", e)
        if self._orientation_estimator is not None:
            self._orientation_estimator.shutdown()
        if self._measurements_manager is not None:
            self._measurements_manager.shutdown()

    def reset_steering(self) -> None:
        """Reset the steering mechanism to its default position."""
        if self._lego_drive_base is None:
            raise RuntimeError("LEGO Drive Base not initialized. Call full_initialization() first.")
        self._lego_drive_base.reset_front_motor()

    def reset_gyro(self) -> None:
        """Reset the yaw angle to zero."""
        if self._orientation_estimator is not None:
            self._orientation_estimator.reset_yaw()
        else:
            raise RuntimeError("Orientation estimator not initialized.")

    def is_button_pressed(self) -> bool:
        """Check if the action button is pressed."""
        return self.action_button.is_active

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

    def turn_steering(self, degrees: float, steering_speed: float = 40) -> None:
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
        return RobotState(
            front=front,
            left=left,
            right=right,
            yaw=yaw,
            camera_front=camera_front,
            camera_left=camera_left,
            camera_right=camera_right,
        )

    def disable_logger(self) -> None:
        """Disable the logger."""
        self.display_loglines = False

    # -------------------- Additional helpers from former RpiInterface --------------------
    def _paint_display(self, img: Image.Image) -> None:
        self.oled.image(img)
        self.oled.show()

    def _flush_pending_messages(self) -> None:
        now = time.time()
        if self.display_loglines:
            self.draw.rectangle((0, 0, self.SCREEN_WIDTH, self.SCREEN_HEIGHT), outline=0, fill=0)
            for i, msg in enumerate(self.messages):
                self.draw.text((0, i * HardwareInterface.LINE_HEIGHT), msg, font=self.font,\
                                                                    fill=255)
            self.oled.image(self.image)
            self.oled.show()
        self._last_oled_update = now

    def _fuse_sensors(
        self,
        lidar_val: float,
        ultrasonic_val: float,
        lidar_weight: float = 0.7,
        ultrasonic_weight: float = 0.3,
        max_diff: float = 20,
    ) -> float:
        if lidar_val > 0 and ultrasonic_val > 0:
            if abs(lidar_val - ultrasonic_val) > max_diff:
                return lidar_val
            fused = (lidar_val * lidar_weight + ultrasonic_val * ultrasonic_weight) / (
                lidar_weight + ultrasonic_weight
            )
            return fused
        return lidar_val if lidar_val > 0 else ultrasonic_val

    def get_left_wangle(self) -> float:
        """Left Wall angle"""
        ultrasonic = self.leftdistancesensor.distance * 100
        laser = self.left_laser.range / 10.0
        return self._wall_angle(front_dist=laser, back_dist=ultrasonic)

    def get_right_wangle(self) -> float:
        """Right wall angle"""
        ultrasonic = self.rightdistancesensor.distance * 100
        laser = self.right_laser.range / 10.0
        return self._wall_angle(front_dist=laser, back_dist=ultrasonic)

    def get_left_pdistance(self) -> float:
        """Left Perpendicular angle"""
        ultrasonic = self.leftdistancesensor.distance * 100
        laser = self.left_laser.range / 10.0
        return self._perpendicular_distance(front_dist=laser, back_dist=ultrasonic)

    def get_right_pdistance(self) -> float:
        """Right Perpendicular angle"""
        ultrasonic = self.rightdistancesensor.distance * 100
        laser = self.right_laser.range / 10.0
        return self._perpendicular_distance(front_dist=laser, back_dist=ultrasonic)

    def _wall_angle(self, front_dist: float, back_dist: float, sensor_gap: float \
                                                = DISTANCE_SENSOR_DISTANCE) -> float:

        #if any reading is bad, donot calculate angle
        if front_dist < 1 or back_dist < 1 or front_dist > 100 or back_dist > 100:
            return -1
        theta_rad = math.atan2(front_dist - back_dist, sensor_gap)
        theta_deg = math.degrees(theta_rad)
        return theta_deg

    def _perpendicular_distance(self, front_dist: float, back_dist: float) -> float:

        #if any reading is bad, donot calculate angle
        if front_dist < 1 or back_dist < 1 or front_dist > 100 or back_dist > 100:
            return -1
        return (front_dist + back_dist) / 2.0

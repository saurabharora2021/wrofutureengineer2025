"""This module is to interact with all hardware connected to RasperryPi."""
import logging
import time
from typing import List,Optional, Tuple

from board import SCL, SDA
import busio
import adafruit_ssd1306
import adafruit_mpu6050
import adafruit_tca9548a
from qmc5883l import QMC5883L
import adafruit_vl53l0x
from gpiozero import Buzzer, RGBLED, DistanceSensor, Button, Device
from gpiozero.pins.pigpio import PiGPIOFactory
from PIL import Image, ImageDraw, ImageFont
from hardware.screenlogger import ScreenLogger
from hardware.pin_config import PinConfig
from hardware.hardwareconfig import HardwareConfig
from hardware.camera import MyCamera
from base.shutdown_handling import ShutdownInterface

logger: logging.Logger = logging.getLogger(__name__)
class RpiInterface(ShutdownInterface):
    """ This interface defines all Interfaces on Raspberry Pi."""

    # Use the centralized pin configuration class

    MAX_STABILIZATION_CHECKS = 5
    LINE_HEIGHT = 10  # pixels per line
    FONT_SIZE = 10 # font size for messages
    LEFT_LASER_CHANNEL = 7
    RIGHT_LASER_CHANNEL = 2
    DEVICE_I2C_CHANNEL=6

    USE_LASER_DISTANCE=True


    front_distance_sensor: Optional[DistanceSensor] = None
    jumper_pin: Optional[Button] = None
    _screenlogger: Optional[ScreenLogger] = None
    display_loglines = True
    camera: Optional[MyCamera] = None


    def __init__(self,stabilize:bool) -> None:
        """
        Initialize the LED control class.
        Sets up the GPIO pins for the LEDs and initializes the buzzer and RGB LED.
        """
        super().__init__()

        logger.info("Initializing RpiInterface...")

        #Setup I2c devices.
        i2c = busio.I2C(SCL,SDA)  # uses board.SCL and board.SDA

        tca = adafruit_tca9548a.TCA9548A(i2c)

        for channel in range(8):
            if tca[channel].try_lock():
                logger.info("Channel {}:".format(channel))
                addresses = tca[channel].scan()
                logger.info([hex(address) for address in addresses if address != 0x70])
                tca[channel].unlock()

        device_channel= tca[self.DEVICE_I2C_CHANNEL]

        # Initialize MPU6050 sensor
        self.mpu = adafruit_mpu6050.MPU6050(device_channel)

        self.compass = QMC5883L(device_channel)

        # Create the SSD1306 OLED class.
        self.oled = adafruit_ssd1306.SSD1306_I2C(PinConfig.SCREEN_WIDTH,
                                                 PinConfig.SCREEN_HEIGHT, device_channel)

        # Clear display.
        self.oled.fill(0)
        self.oled.show()

        self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                                        RpiInterface.FONT_SIZE)
        # Load a default font
        #self.font: ImageFont.FreeTypeFont | ImageFont.ImageFont = ImageFont.load_default()
        self._last_oled_update: float = 0
        self.image: Image.Image = Image.new("1", (PinConfig.SCREEN_WIDTH, PinConfig.SCREEN_HEIGHT))
        self.draw: ImageDraw.ImageDraw = ImageDraw.Draw(self.image)

        self.pendingmessage: bool = False  # Initialize pendingmessage flag
        self.messages: List[str] = []  # Initialize messages list

        #intialize laser distance sensor
        left_channel = tca[self.LEFT_LASER_CHANNEL]
        right_channel = tca[self.RIGHT_LASER_CHANNEL]

        self.left_laser = adafruit_vl53l0x.VL53L0X(left_channel)
        self.right_laser = adafruit_vl53l0x.VL53L0X(right_channel)

        #Logger is not setup yet, so we use print for initialization messages
        self.display_message("Initializing Pi Interface...")

        # Log the pin configuration for debugging purposes
        PinConfig.log_pin_configuration()

        try:
            # Use pigpio factory if available
            Device.pin_factory = PiGPIOFactory()
            logger.info("Using PiGPIOFactory for GPIO pin control.")
        except : # pylint: disable=bare-except
            # Fallback to default GPIO pin factory
            Device.pin_factory = None
            logger.error("Failed to initialize PiGPIOFactory")

        self.buzzer = Buzzer(PinConfig.BUZZER_PIN)
        self.led1 = RGBLED(red=PinConfig.LED1_RED_PIN,
                           green=PinConfig.LED1_GREEN_PIN,
                           blue=PinConfig.LED1_BLUE_PIN)
        """Turn on the LED1 Test."""
        self.led1.color = (0, 1, 0)  # green
        time.sleep(PinConfig.LED_TEST_DELAY)
        self.led1.color = (1, 0, 0)  # red
        time.sleep(PinConfig.LED_TEST_DELAY)
        self.led1.color = (0, 0, 1)  # blue
        time.sleep(PinConfig.LED_TEST_DELAY)
        self.led1.color = (0, 0, 0)  # off

        # Set up the button
        self.action_button = Button(PinConfig.BUTTON_PIN, hold_time=1)

        self.rightdistancesensor = DistanceSensor(echo=PinConfig.RIGHT_SENSOR_ECHO_PIN,
                                                  trigger=PinConfig.RIGHT_SENSOR_TRIG_PIN,
                                                  partial=True,
                                                  max_distance=
                                                  PinConfig.RIGHT_DISTANCE_MAX_DISTANCE)
        self.leftdistancesensor = DistanceSensor(echo=PinConfig.LEFT_SENSOR_ECHO_PIN,
                                                 trigger=PinConfig.LEFT_SENSOR_TRIG_PIN,
                                                 partial=True,
                                                 max_distance=PinConfig.LEFT_DISTANCE_MAX_DISTANCE)

                # Initialize Optional Front Distance Sensor
        if HardwareConfig.CHASSIS_VERSION == 2:
            self.front_distance_sensor = DistanceSensor(echo=PinConfig.FRONT_SENSOR_ECHO_PIN,
                                                        trigger=PinConfig.FRONT_SENSOR_TRIG_PIN,
                                                        partial=True,
                                                        max_distance=
                                                        PinConfig.FRONT_DISTANCE_MAX_DISTANCE)
            self.jumper_pin = Button(PinConfig.JUMPER_PIN, hold_time=1)


        if PinConfig.CAMERA_ENABLED:
            #setup camera library
            self.camera = MyCamera()

        logger.info("RpiInterface initialized successfully.")

        if stabilize:
            logger.warning("Stabilize Distance Sensors...")
            # Stabilize distance sensors
            time.sleep(1)  # Wait for sensors to stabilize
            counter = 0
            valid_distance = False

            while counter< self.MAX_STABILIZATION_CHECKS and valid_distance is False:
                valid_distance = True
                if (self.get_right_distance() < 0.1 or
                        self.get_right_distance() >= self.get_right_distance_max()):
                    logger.info("Right distance sensor is not stable, %.2f cm",
                                    self.get_right_distance())
                    valid_distance = False
                if (self.get_left_distance() < 0.1 or
                        self.get_left_distance() >= self.get_left_distance_max()):
                    logger.info("Left distance sensor is not stable, %.2f cm",
                                    self.get_left_distance())
                    valid_distance = False
                if valid_distance is False:
                    logger.warning("Waiting for distance sensors to stabilize...")
                    time.sleep(1)
                counter += 1

    def get_magnetometer(self) -> Tuple[float, float, float]:
        """Get the current magnetometer reading (uT) as (mx, my, mz)."""
        return self.compass.magnetic

    def get_camera(self) -> MyCamera:
        """Get the camera instance."""
        return self.camera

    def get_screen_logger(self) -> ScreenLogger:
        """Get the logger for the RpiInterface."""
        if self._screenlogger is None:
            self._screenlogger = ScreenLogger()
        return self._screenlogger

    def add_screen_logger_message(self, message: List[str]) -> None:
        """Add a message to the screen logger."""
        self.get_screen_logger().add_message(message)

    def log_message(self, front: float, left: float, right: float, current_yaw: float,
                                                            current_steering: float) -> None:
        """Log a message to the screen."""
        image: Image.Image = self.get_screen_logger().log_message(front, left, right,
                                                                current_yaw, current_steering)
        self.paint_display(image)


    def disable_logger(self) -> None:
        """Disable the logger."""
        self.display_loglines = False

    def enable_logger(self) -> None:
        """Enable the logger."""
        self.display_loglines = True

    def buzzer_beep(self, timer: float = 1) -> None:
        """Turn on the buzzer."""
        self.buzzer.on()
        time.sleep(timer)
        self.buzzer.off()

    def led1_green(self) -> None:
        """Turn on the LED1 green."""
        self.led1.color = (0, 1, 0)  # green

    def led1_red(self) -> None:
        """Turn on the LED1 red."""
        self.led1.color = (1, 0, 0)  # red

    def led1_blue(self) -> None:
        """Turn on the LED1 blue."""
        self.led1.color = (0, 0, 1)  # blue

    def led1_white(self) -> None:
        """Turn on the LED1 white."""
        self.led1.color = (1, 1, 1)

    def led1_off(self) -> None:
        """Turn off the LED1."""
        self.led1.color = (0, 0, 0)  # off

    def wait_for_action(self) -> None:
        """Wait for the action button to be pressed."""
        logger.warning("Waiting for action button press...")
        self.action_button.wait_for_active()
        logger.info("Action button pressed!")


    def get_right_distance(self) -> float:
        """Get the distance from the distance sensor."""
        ultrasonic = self.rightdistancesensor.distance * 100  # Convert to cm
        if not self.USE_LASER_DISTANCE:
            return ultrasonic
        else: 
            laser = self.right_laser.range / 10.0  # Convert mm to cm
            return self._min_distance(ultrasonic,laser)
    
    def _min_distance(self,ultrasonic:float,laser:float)->float:
        if laser < ultrasonic and laser > 0:
            # logger.info("Using Laser for laser:%.2f ultra:%.2f",laser,ultrasonic)
            return laser
        elif ultrasonic > 0:
            return ultrasonic
        return 0


    def get_right_distance_max(self) -> float:
        """Get the maximum distance for the right distance sensor."""
        return PinConfig.RIGHT_DISTANCE_MAX_DISTANCE * 100  # Convert to cm

    def get_left_distance(self) -> float:
        """Get the distance from the distance sensor."""

        ultrasonic = self.leftdistancesensor.distance * 100  # Convert to cm
        if not self.USE_LASER_DISTANCE:
            return ultrasonic
        else: 
            laser = self.left_laser.range / 10.0  # Convert mm to cm
            return self._min_distance(ultrasonic,laser)

    def get_left_distance_max(self) -> float:
        """Get the maximum distance for the left distance sensor."""
        return PinConfig.LEFT_DISTANCE_MAX_DISTANCE * 100  # Convert to cm

    def get_front_distance(self) -> float:
        """Get the distance from the front distance sensor."""
        if self.front_distance_sensor is None:
            raise ValueError("Front distance sensor is not initialized.")

        # Check if the front distance sensor is initialized
        return self.front_distance_sensor.distance * 100  # Convert to cm

    def get_front_distance_max(self) -> float:
        """Get the maximum distance for the front distance sensor."""
        if self.front_distance_sensor is None:
            raise ValueError("Front distance sensor is not initialized.")

        # Check if the front distance sensor is initialized
        return PinConfig.FRONT_DISTANCE_MAX_DISTANCE * 100  # Convert to cm


    def shutdown(self) -> None:
        try:
            self.flush_pending_messages()
        except Exception as e:  # pylint: disable=broad-except
            logger.error("Error flushing OLED messages during shutdown: %s", e)
        if self.camera is not None:
            self.camera.close()
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
        if self.front_distance_sensor is not None:
            try:
                self.front_distance_sensor.close()
            except Exception as e:  # pylint: disable=broad-except
                logger.error("Error closing front distance sensor during shutdown: %s", e)
        if self.jumper_pin is not None:
            try:
                self.jumper_pin.close()
            except Exception as e:  # pylint: disable=broad-except
                logger.error("Error closing jumper pin during shutdown: %s", e)

    def display_message(self, message: str, forceflush: bool = False) -> None:
        """
        Display a message on the OLED screen.

        Only the last 5 messages are shown on the display.
        """
        now = time.time()
        self.messages.append(message)
        self.messages = self.messages[-5:]  # Keep only the last 5 messages

        if forceflush or (now - self._last_oled_update >= PinConfig.SCREEN_UPDATE_INTERVAL):
            # Only update the OLED if enough time has passed since the last update
            self.flush_pending_messages()
            self.pendingmessage = False

    def flush_pending_messages(self) -> None:
        """Flush the pending messages to the OLED display."""
        now = time.time()
        if self.display_loglines:
            self.draw.rectangle((0, 0, PinConfig.SCREEN_WIDTH, PinConfig.SCREEN_HEIGHT),
                                outline=0, fill=0)
            for i, msg in enumerate(self.messages):
                # Use constant for line height spacing
                self.draw.text((0, i * RpiInterface.LINE_HEIGHT), msg, font=self.font, fill=255)
            self.oled.image(self.image)
            self.oled.show()
        self._last_oled_update = now

    def paint_display(self, img: Image.Image) -> None:
        """Paint the display with the given image."""
        self.oled.image(img)
        self.oled.show()

    def clear_messages(self) -> None:
        """Clear all messages from the display."""
        self.messages.clear()
    def force_flush_messages(self) -> None:
        """Force the OLED to update immediately."""
        if self.pendingmessage:
            self.flush_pending_messages()
            self.pendingmessage = False

    def get_jumper_state(self) -> bool:
        """Get the state of the jumper pin."""
        if self.jumper_pin is None:
            raise ValueError("Jumper pin is not initialized.")
        return self.jumper_pin.is_active

    def get_acceleration(self) -> Tuple[float, float, float]:
        """Get the acceleration from the MPU6050 sensor."""
        accel = self.mpu.acceleration
        return accel
    def get_gyro(self) -> Tuple[float, float, float]:
        """Get the gyroscope data from the MPU6050 sensor."""
        gyro = self.mpu.gyro
        return gyro

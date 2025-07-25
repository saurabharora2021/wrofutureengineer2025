"""This module is to interact with all hardware connected to RasperryPi."""
import logging
import time
from typing import List

from board import SCL, SDA
import busio
import adafruit_ssd1306
from gpiozero import Buzzer, RGBLED, DistanceSensor, Button, Device
from gpiozero.pins.pigpio import PiGPIOFactory
from PIL import Image, ImageDraw, ImageFont

from base.shutdown_handling import ShutdownInterface
from hardware.pin_config import PinConfig

class RpiInterface(ShutdownInterface):
    """ This interface defines all Interfaces on Raspberry Pi."""

    logger: logging.Logger = logging.getLogger(__name__)

    # Use the centralized pin configuration class
    # All pin definitions are imported from PinConfig


    def __init__(self) -> None:
        """
        Initialize the LED control class.
        Sets up the GPIO pins for the LEDs and initializes the buzzer and RGB LED.
        """
        super().__init__()

        self.logger.info("Initializing RpiInterface...")
        #Setup Screen First.
        i2c = busio.I2C(SCL,SDA)  # uses board.SCL and board.SDA

        # Create the SSD1306 OLED class.
        self.oled = adafruit_ssd1306.SSD1306_I2C(PinConfig.SCREEN_WIDTH,
                                                 PinConfig.SCREEN_HEIGHT, i2c)

        # Clear display.
        self.oled.fill(0)
        self.oled.show()

        #self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)
        # Load a default font
        self.font: ImageFont.FreeTypeFont | ImageFont.ImageFont = ImageFont.load_default()
        self._last_oled_update: float = 0
        self.image: Image.Image = Image.new("1", (PinConfig.SCREEN_WIDTH, PinConfig.SCREEN_HEIGHT))
        self.draw: ImageDraw.ImageDraw = ImageDraw.Draw(self.image)

        self.pendingmessage: bool = False  # Initialize pendingmessage flag
        self.messages: List[str] = []  # Initialize messages list

        #Logger is not setup yet, so we use print for initialization messages
        self.display_message("Initializing Pi Interface...")

        # Log the pin configuration for debugging purposes
        PinConfig.log_pin_configuration()

        try:
            # Use pigpio factory if available
            Device.pin_factory = PiGPIOFactory()
        except : # pylint: disable=bare-except
            # Fallback to default GPIO pin factory
            Device.pin_factory = None
            self.logger.warning("Failed to initialize PiGPIOFactory")

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

        self.logger.info("RpiInterface initialized successfully.")


    def buzzer_beep(self, timer: int = 1) -> None:
        """Turn on the buzzer."""
        self.buzzer.blink(on_time=timer, off_time=timer, n=1)  # Blink 1 time.

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
        self.logger.warning("Waiting for action button press...")
        self.action_button.wait_for_active()
        self.logger.info("Action button pressed!")


    def get_right_distance(self) -> float:
        """Get the distance from the distance sensor."""
        return self.rightdistancesensor.distance * 100  # Convert to cm

    def get_right_distance_max(self) -> float:
        """Get the maximum distance for the right distance sensor."""
        return PinConfig.RIGHT_DISTANCE_MAX_DISTANCE * 100  # Convert to cm

    def get_left_distance(self) -> float:
        """Get the distance from the distance sensor."""
        return self.leftdistancesensor.distance * 100  # Convert to cm

    def get_left_distance_max(self) -> float:
        """Get the maximum distance for the left distance sensor."""
        return PinConfig.LEFT_DISTANCE_MAX_DISTANCE * 100  # Convert to cm

    def shutdown(self) -> None:
        try:
            self.flush_pending_messages()
        except Exception as e:  # pylint: disable=broad-except
            self.logger.error("Error flushing OLED messages during shutdown: %s", e)
        try:
            self.buzzer.off()
        except Exception as e:  # pylint: disable=broad-except
            self.logger.error("Error turning off buzzer during shutdown: %s", e)
        try:
            self.led1.color = (0, 0, 0)
        except Exception as e:  # pylint: disable=broad-except
            self.logger.error("Error turning off LED1 during shutdown: %s", e)
        try:
            self.led1.close()
        except Exception as e:  # pylint: disable=broad-except
            self.logger.error("Error closing LED1 during shutdown: %s", e)
        try:
            self.buzzer.close()
        except Exception as e:  # pylint: disable=broad-except
            self.logger.error("Error closing buzzer during shutdown: %s", e)
        try:
            self.rightdistancesensor.close()
        except Exception as e:  # pylint: disable=broad-except
            self.logger.error("Error closing right distance sensor during shutdown: %s", e)
        try:
            self.leftdistancesensor.close()
        except Exception as e:  # pylint: disable=broad-except
            self.logger.error("Error closing left distance sensor during shutdown: %s", e)
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
        self.draw.rectangle((0, 0, PinConfig.SCREEN_WIDTH, PinConfig.SCREEN_HEIGHT),
                            outline=0, fill=0)
        for i, msg in enumerate(self.messages):
            # Use constant for line height spacing
            self.draw.text((0, i * PinConfig.LINE_HEIGHT), msg, font=self.font, fill=255)
        self.oled.image(self.image)
        self.oled.show()
        self._last_oled_update = now

    def force_flush_messages(self) -> None:
        """Force the OLED to update immediately."""
        if self.pendingmessage:
            self.flush_pending_messages()
            self.pendingmessage = False

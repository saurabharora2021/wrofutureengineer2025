from gpiozero import Buzzer,RGBLED, DistanceSensor,Button,Device
import subprocess
from signal import pause
import board
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import logging
#try to import pigpio
from gpiozero.pins.pigpio import PiGPIOFactory



import time
from base.ShutdownInterface import ShutdownInterface

class RpiInterface(ShutdownInterface):

    logger = logging.getLogger(__name__)

    """ Pin Definitions:"""
    # Buzzer on GPIO pin 20
    BUZZER_PIN = 20
    # RGB LED on GPIO pins 26 (red), 19 (green), 13 (blue)
    LED1_RED_PIN = 26
    LED1_GREEN_PIN = 19
    LED1_BLUE_PIN = 13

    LED_TEST_DELAY = 0.25  # seconds

    BUTTON_PIN = 21

    RIGHT_SENSOR_TRIG_PIN = 22
    RIGHT_SENSOR_ECHO_PIN = 27

    LEFT_SENSOR_TRIG_PIN = 23
    LEFT_SENSOR_ECHO_PIN = 24

    # OLED display settings
    # These are the dimensions of the SSD1306 OLED display
    SCREEN_WIDTH = 128
    SCREEN_HEIGHT = 64
    #array to store the messages to be displayed on the OLED screen
    messages = []

    SCREEN_UPDATE_INTERVAL = 0.5  # seconds


    def __init__(self):
        """
        Initialize the LED control class.
        Sets up the GPIO pins for the LEDs and initializes the buzzer and RGB LED.
        """
        super().__init__()

        self.logger.info("Initializing RpiInterface...")
        #Setup Screen First.
        i2c = board.I2C()  # uses board.SCL and board.SDA
    
        # Create the SSD1306 OLED class.
        self.oled = adafruit_ssd1306.SSD1306_I2C(self.SCREEN_WIDTH, self.SCREEN_HEIGHT, i2c)

        # Clear display.
        self.oled.fill(0)
        self.oled.show()
    
        self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)
        self._last_oled_update = 0
        self.image = Image.new("1", (self.oled.width, self.oled.height))
        self.draw = ImageDraw.Draw(self.image)

        #Logger is not setup yet, so we use print for initialization messages
        self.display_message("Initializing Pi Interface...")

        try:
            # Use pigpio factory if available
            Device.pin_factory = PiGPIOFactory()
        except :
            # Fallback to default GPIO pin factory
            Device.pin_factory = None
            self.logger.warning("Pigpio failed.")

        self.buzzer = Buzzer(self.BUZZER_PIN)
        self.led1 = RGBLED(red=self.LED1_RED_PIN, green=self.LED1_GREEN_PIN, blue=self.LED1_BLUE_PIN)
        """Turn on the LED1 Test."""
        self.led1.color = (0, 1, 0)  # green
        time.sleep(self.LED_TEST_DELAY)
        self.led1.color = (1, 0, 0)  # red
        time.sleep(self.LED_TEST_DELAY)
        self.led1.color = (0, 0, 1)  # blue
        time.sleep(self.LED_TEST_DELAY)
        self.led1.color = (0, 0, 0)  # off

        # Set up the shutdown button
        self.action_button = Button(self.BUTTON_PIN, hold_time=1)

        self.rightdistancesensor = DistanceSensor(echo=self.RIGHT_SENSOR_ECHO_PIN,trigger=self.RIGHT_SENSOR_TRIG_PIN,partial=True)
        self.leftdistancesensor = DistanceSensor(echo=self.LEFT_SENSOR_ECHO_PIN,trigger=self.LEFT_SENSOR_TRIG_PIN,partial=True)

        self.logger.info("RpiInterface initialized successfully.")
    
    # Checks if the Raspberry Pi is throttled using vcgencmd
    # This command returns a hex value indicating the throttling status
    # If the value is not zero, it means the Raspberry Pi is throttled
    def check_throttling(self):        
        try:
            result = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True)
            if result.returncode == 0:
                throttled_hex = result.stdout.strip().split('=')[-1]
                throttled = int(throttled_hex, 16)
                if throttled != 0:
                    self.logger.error(f"Pi is throttled! get_throttled={throttled_hex}")
                else:
                    self.logger.warning("Pi is not throttled.")
            else:
                self.logger.error("Failed to run vcgencmd get_throttled")
        except Exception as e:
            logging.error(f"Error checking throttling: {e}")


    def buzzer_beep(self,timer=0.5):
        """Turn on the buzzer."""
        self.buzzer.blink(on_time=timer, off_time=timer, n=1)  # Blink 1 time.
        
    def LED1_green(self):
        """Turn on the LED1 green."""
        self.led1.color = (0, 1, 0) # green

    def LED1_red(self):
        """Turn on the LED1 red."""
        self.led1.color = (1, 0, 0) # red

    def LED1_blue(self):
        """Turn on the LED1 blue."""
        self.led1.color = (0, 0, 1) # blue

    def LED1_white(self):
        """Turn on the LED1 white."""
        self.led1.color = (1, 1, 1)
    

    def LED1_off(self):
        """Turn off the LED1."""
        self.led1.color = (0, 0, 0) # off

    

    def wait_for_action(self):
        """Wait for the action button to be pressed."""
        self.logger.warning("Waiting for action button press...")
        self.action_button.wait_for_press()
        self.logger.info("Action button pressed!")


    def getRightDistance(self):
        """Get the distance from the distance sensor."""
        return self.rightdistancesensor.distance * 100 # Convert to cm    
    
    def getLeftDistance(self):
        """Get the distance from the distance sensor."""
        return self.leftdistancesensor.distance * 100 # Convert to cm    

    def shutdown(self):
        self.flush_pending_messages()
        self.buzzer.off()
        self.led1.color = (0, 0, 0)
        self.led1.close()
        self.buzzer.close()
        self.rightdistancesensor.close()
        self.leftdistancesensor.close()
    def display_message(self, message,forceflush=False):
        """
        Display a message on the OLED screen.

        Only the last 5 messages are shown on the display.
        """
        now = time.time()
        self.messages.append(message)
        self.messages = self.messages[-5:]  # Keep only the last 5 messages
        
        self.pendingmessage = True

        # Only update the OLED if enough time has passed since the last update
        if forceflush or (now - getattr(self, "_last_oled_update", 0) >= self.SCREEN_UPDATE_INTERVAL):
            self.flush_pending_messages()
            self.pendingmessage = False

    
    def flush_pending_messages(self):          
            now = time.time()
            self.draw.rectangle((0, 0, self.SCREEN_WIDTH, self.SCREEN_HEIGHT), outline=0, fill=0)
            for i, msg in enumerate(self.messages):
                self.draw.text((0, i*13), msg, font=self.font, fill=255)
            self.oled.image(self.image)
            self.oled.show()
            self._last_oled_update = now

    def force_flush_messages(self):
        """Force the OLED to update immediately."""
        if (self.pendingmessage):
            self.flush_pending_messages()
            self.pendingmessage = False
from gpiozero import Buzzer,RGBLED, DistanceSensor,Button
from subprocess import check_call
from signal import pause
import board
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import logging



import time
from base.ShutdownInterface import ShutdownInterface

class OutputInterface(ShutdownInterface):

    logger = logging.getLogger(__name__)

    """ Pin Definitions:"""
    # Buzzer on GPIO pin 20
    BUZZER_PIN = 20
    # RGB LED on GPIO pins 26 (red), 19 (green), 16 (blue)
    LED1_RED_PIN = 26
    LED1_GREEN_PIN = 19
    LED1_BLUE_PIN = 13

    BUTTON_PIN = 21

    RIGHT_SENSOR_TRIG_PIN = 22
    RIGHT_SENSOR_ECHO_PIN = 27

    LEFT_SENSOR_TRIG_PIN = 23
    LEFT_SENSOR_ECHO_PIN = 24

    #array to store the messages to be displayed on the OLED screen
    messages = []

    def __init__(self):
        """
        Initialize the LED control class.
        Sets up the GPIO pins for the LEDs and initializes the buzzer and RGB LED.
        """
        super().__init__()
        self.buzzer = Buzzer(self.BUZZER_PIN)
        self.led1 = RGBLED(red=self.LED1_RED_PIN, green=self.LED1_GREEN_PIN, blue=self.LED1_BLUE_PIN)
        """Turn on the LED1 Test."""
        self.led1.color = (0, 1, 0)  # green
        time.sleep(0.5)
        self.led1.color = (1, 0, 0)  # red
        time.sleep(0.5)
        self.led1.color = (0, 0, 1)  # blue
        time.sleep(0.5)
        self.led1.color = (0, 0, 0)  # off

        # Set up the shutdown button
        self.action_button = Button(self.BUTTON_PIN, hold_time=2)

        self.rightdistancesensor = DistanceSensor(echo=self.RIGHT_SENSOR_ECHO_PIN,trigger=self.RIGHT_SENSOR_TRIG_PIN)
        self.leftdistancesensor = DistanceSensor(echo=self.LEFT_SENSOR_ECHO_PIN,trigger=self.LEFT_SENSOR_TRIG_PIN)

        # Setting some variables for our reset pin etc.
        RESET_PIN = digitalio.DigitalInOut(board.D4)

        # Very important... This lets py-gaugette 'know' what pins to use in order to reset the display
        i2c = board.I2C()  # uses board.SCL and board.SDA
        # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

        # Create the SSD1306 OLED class.
        # The first two parameters are the pixel width and pixel height.
        # Change these to the right size for your display!
        self.oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)

        # Clear display.
        self.oled.fill(0)
        self.oled.show()
    
        self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)
        self._last_oled_update = 0
        self._oled_update_interval = 0.1  # seconds
        self.display_message("Initializing Pi...")
    

    def buzzer_beep(self,timer=0.5):
        """Turn on the buzzer."""
        self.buzzer.on()
        time.sleep(timer)
        self.buzzer.off()
        
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
        print("Waiting for action button press...")
        self.action_button.wait_for_press()
        print("Action button pressed!")


    def getRightDistance(self):
        """Get the distance from the distance sensor."""
        return self.rightdistancesensor.distance * 100 # Convert to cm    
    
    def getLeftDistance(self):
        """Get the distance from the distance sensor."""
        return self.leftdistancesensor.distance * 100 # Convert to cm    

    def shutdown(self):
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
        if forceflush or (now - getattr(self, "_last_oled_update", 0) >= getattr(self, "_oled_update_interval", 0.1)):
            self.flush_pending_messages()
            self.pendingmessage = False

    
    def flush_pending_messages(self):          
            now = time.time()
            self.image = Image.new("1", (self.oled.width, self.oled.height))
            self.draw = ImageDraw.Draw(self.image)
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


    def logAndDisplay(self,message):
        """Log and display a message."""
        self.logger.info(message)
        self.display_message(message)
        print(message)
    



from gpiozero import Buzzer
from gpiozero import RGBLED



import time
from base.ShutdownInterface import ShutdownInterface


class OutputInterface(ShutdownInterface):

    # Define GPIO pin numbers for the LEDs
    LED_PIN_LOW_BATTERY = 18  # GPIO pin number for the LED
    LED_PIN_LOW_BATTERY_SPARK = 17  # GPIO pin number for the LED

    def __init__(self):
        """
        Initialize the LED control class.
        Sets up the GPIO pins for the LEDs and initializes the buzzer and RGB LED.
        """
        super().__init__()
        self.buzzer = Buzzer(20)
        self.led1 = RGBLED(red=26, green=19, blue=16)



    def buzzer_on(self):
        """Turn on the buzzer."""
        self.buzzer.on()
        time.sleep(0.5)
        self.buzzer.off()
        
    def LED1_on(self):
        """Turn on the LED1."""
        self.led1.color = (0, 1, 0)  # green
        time.sleep(0.5)
        self.led1.color = (1, 0, 0)  # red
        time.sleep(0.5)
        self.led1.color = (0, 0, 1)  # blue
        time.sleep(0.5)
        self.led1.color = (0, 0, 0)  # off
        

    def LED2_on(self):
        """Turn on the LED2."""
        

    def shutdown(self):
        """Clean up the GPIO settings."""
        GPIO.cleanup()

from gpiozero import Buzzer
from gpiozero import RGBLED
from gpiozero import Button
from subprocess import check_call
from signal import pause

import time
from base.ShutdownInterface import ShutdownInterface


class OutputInterface(ShutdownInterface):

    """ Pin Definitions:"""
    # Buzzer on GPIO pin 20
    BUZZER_PIN = 20
    # RGB LED on GPIO pins 26 (red), 19 (green), 16 (blue)
    LED1_RED_PIN = 26
    LED1_GREEN_PIN = 19
    LED1_BLUE_PIN = 16

    SHUTDOWN_BUTTON_PIN = 21

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

        #shutdown_btn = Button(self.SHUTDOWN_BUTTON_PIN, hold_time=2)
        #shutdown_btn.when_held = self.poweroffbuttonaction

    def poweroffbuttonaction():
        check_call(['sudo', 'poweroff'])


    def buzzer_on(self):
        """Turn on the buzzer."""
        self.buzzer.on()
        time.sleep(0.5)
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
        

    def shutdown(self):
        self.buzzer.off()
        self.led1.color = (0, 0, 0)
        self.led1.close()
        self.buzzer.close()

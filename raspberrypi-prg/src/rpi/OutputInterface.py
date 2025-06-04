from gpiozero import Buzzer,RGBLED, DistanceSensor,Button
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

    LED2_RED_PIN = 13
    LED2_BLUE_PIN = 12
    LED2_GREEN_PIN = 6

    BUTTON_PIN = 21

    DISTANCE1_SENSOR_TRIG_PIN = 27
    DISTANCE1_SENSOR_ECHO_PIN = 17


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

        self.led2 = RGBLED(red=self.LED2_RED_PIN, green=self.LED2_GREEN_PIN, blue=self.LED2_BLUE_PIN)
        """Turn on the LED2 Test."""
        self.led2.color = (0, 1, 0)  # green
        time.sleep(0.5)
        self.led2.color = (1, 0, 0)
        time.sleep(0.5)
        self.led2.color = (0, 0, 1)
        time.sleep(0.5)
        self.led2.color = (0, 0, 0)  # off

        # Set up the shutdown button
        self.action_button = Button(self.BUTTON_PIN, hold_time=2)

        self.distancesensor1 = DistanceSensor(trigger=self.DISTANCE1_SENSOR_TRIG_PIN, echo=self.DISTANCE1_SENSOR_ECHO_PIN)


    def poweroffbuttonaction():
        check_call(['sudo', 'poweroff'])


    def buzzer_beep(self):
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

    def LED2_green(self):
        """Turn on the LED2 green."""
        self.led2.color = (0, 1, 0)

    def LED2_red(self):
        """Turn on the LED2 red."""
        self.led2.color = (1, 0, 0)

    def LED2_blue(self):
        """Turn on the LED2 blue."""
        self.led2.color = (0, 0, 1)

    def LED2_off(self):
        """Turn off the LED2."""
        self.led2.color = (0, 0, 0)

    def LED2_white(self):
        """Turn on the LED2 white."""
        self.led2.color = (1, 1, 1)
    

    def wait_for_action(self):
        """Wait for the action button to be pressed."""
        print("Waiting for action button press...")
        self.action_button.wait_for_press()
        print("Action button pressed!")


    def get_distance_right(self):
        """Get the distance from the distance sensor."""
        return self.distancesensor1.distance * 100 # Convert to cm    

    def shutdown(self):
        self.buzzer.off()
        self.led1.color = (0, 0, 0)
        self.led1.close()
        self.buzzer.close()

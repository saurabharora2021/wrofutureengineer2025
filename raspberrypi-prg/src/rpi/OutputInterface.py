import RPi.GPIO as GPIO
import time
from base.ShutdownInterface import ShutdownInterface


class OutputInterface(ShutdownInterface):

    #Define GPIO pin numbers for the LEDs
    LED_PIN_LOW_BATTERY = 18  # GPIO pin number for the LED
    LED_PIN_LOW_BATTERY_SPARK = 17  # GPIO pin number for the LED


    def __init__(self):
        """
        Initialize the LED control class.
        :param pin: GPIO pin number where the LED is connected.
        """
        GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
        GPIO.setup(OutputInterface.LED_PIN_LOW_BATTERY, GPIO.OUT)  # Set the pin as an output
        GPIO.setup(OutputInterface.LED_PIN_LOW_BATTERY_SPARK, GPIO.OUT)  # Set the pin as an output
        
    def __turn_on(self,pin: int):
        """Turn the LED on."""
        GPIO.output(pin, GPIO.HIGH)

    def __turn_off(self,pin: int):
        """Turn the LED off."""
        GPIO.output(pin, GPIO.LOW)

    def set_low_battery_led(self, state: bool):
        """
        Set the state of the low battery LED.
        :param state: True to turn on the LED, False to turn it off.
        """
        if state:
            self.__turn_on(OutputInterface.LED_PIN_LOW_BATTERY)
        else:
            self.__turn_off(OutputInterface.LED_PIN_LOW_BATTERY)
        

    def set_low_battery_spark_led(self, state: bool):
        """
        Set the state of the low battery spark LED.
        :param state: True to turn on the LED, False to turn it off.
        """
        if state:
            self.__turn_on(OutputInterface.LED_PIN_LOW_BATTERY_SPARK)
        else:
            self.__turn_off(OutputInterface.LED_PIN_LOW_BATTERY_SPARK)

    def __blink(self, duration: float = 0.5):
        """
        Blink the LED for a specified duration.
        :param duration: Duration in seconds for which the LED should blink.
        """
        self.__turn_on()
        time.sleep(duration)
        self.__turn_off()

    def shutdown(self):
        """Clean up the GPIO settings."""
        GPIO.cleanup()
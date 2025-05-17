"""Shutdown the Raspberry Pi when a button is pressed on GPIO pin 21."""
# shutdownpi.py
# This script listens for a button press on GPIO pin 21 and initiates a shutdown of the Raspberry Pi.
import RPi.GPIO as GPIO
import subprocess
import time

shutdown_pin = 21

GPIO.setmode(GPIO.BCM)
GPIO.setup(shutdown_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

DEBOUNCE_TIME = 0.5  # seconds

try:
    while True:
        if GPIO.input(shutdown_pin) == GPIO.LOW:
            # Wait for debounce time and check again
            time.sleep(DEBOUNCE_TIME)
            if GPIO.input(shutdown_pin) == GPIO.LOW:
                print("Shutdown initiated...")
                subprocess.call(["sudo", "shutdown", "-h", "now"], shell=False)
                break
        time.sleep(0.1)

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
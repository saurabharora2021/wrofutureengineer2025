import RPi.GPIO as GPIO
import subprocess
import time

# Define GPIO pin for shutdown signal
shutdown_pin = 40  

# Set up GPIO mode and pin
GPIO.setmode(GPIO.BCM)
GPIO.setup(shutdown_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Wait for the GPIO pin to go low (button pressed)
try:
    while True:
        if GPIO.input(shutdown_pin) == GPIO.LOW:
            print("Shutdown initiated...")
            subprocess.call(["sudo", "shutdown", "-h", "now"], shell=False)
            break
        time.sleep(0.1)

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
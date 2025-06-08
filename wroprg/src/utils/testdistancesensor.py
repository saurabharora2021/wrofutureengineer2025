import time

import RPi.GPIO as GPIO

TRIG = 22
ECHO = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    # Ensure trigger is low
    GPIO.output(TRIG, False)
    time.sleep(0.05)

    # Send 10us pulse to trigger
    print("send trigger")
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    # time.sleep(100)
    GPIO.output(TRIG, False)
    print("send trigger close")

    # Wait for echo to go high
    print("wait for echo")
    while GPIO.input(ECHO) == 0:
        print("waiting for echo to go high")
        pulse_start = time.time()

    # Wait for echo to go low
    print("echo is high, waiting for echo to go low")
    while GPIO.input(ECHO) == 1:
        print("waiting for echo to go low")
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound: 34300 cm/s / 2
    distance = round(distance, 2)
    return distance

try:
    while True:
        dist = get_distance()
        print(f"Distance: {dist} cm")
        time.sleep(5)
except KeyboardInterrupt:
    print("Measurement stopped by user")
finally:
    GPIO.cleanup()
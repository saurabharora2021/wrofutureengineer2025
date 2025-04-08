def read_sensor(sensor):
    """Read the value from the specified sensor."""
    return sensor.read()

def move_motor(motor, speed, duration):
    """Move the specified motor at a given speed for a certain duration."""
    motor.run_time(speed, duration)

def stop_motor(motor):
    """Stop the specified motor."""
    motor.stop()
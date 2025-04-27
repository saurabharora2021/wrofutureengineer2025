from buildhat import Motor

# Initialize the motor on port A
motor = Motor('A')

# Turn the motor by 90 degrees
motor.run_to_position(90)

# Stop the motor
motor.stop()
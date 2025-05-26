from base.DriveBase import DriveBase
from buildhat import Motor,ColorSensor, DistanceSensor
from base import ShutdownInterface

class BuildHatDriveBase(DriveBase, ShutdownInterface):
    def __init__(self, front_motor_port, back_motor_port,bottom_color_sensor_port, front_distance_sensor_port):
        """Initialize the drive base with two motors."""
        self.front_motor = Motor(front_motor_port)
        self.back_motor = Motor(back_motor_port)
        self.bottom_color_sensor_port = bottom_color_sensor_port
        self.front_distance_sensor_port = front_distance_sensor_port

    def runfront(self, speed):
        """Run the drive base forward at the specified speed."""
        self.back_motor.start(speed)

    def stop(self):
        """Stop the drive base."""
        self.back_motor.stop()

    def turnright(self, angle):
        """Turn the drive base to the right by the specified angle."""
        # Example: Use a simple time-based approach for turning
        self.front_motor.run_for_degrees(angle, 50)  # Adjust speed as needed

    def turnleft(self, angle):
        """Turn the drive base to the left by the specified angle."""

        self.front_motor.run_for_degrees(
            (-1)*angle, 50)  # Adjust speed as needed
        
    def shutdown(self):
        """Shutdown the drive base."""
        self.front_motor.reset()
        self.back_motor.reset()
        print("Drive base shutdown complete.")

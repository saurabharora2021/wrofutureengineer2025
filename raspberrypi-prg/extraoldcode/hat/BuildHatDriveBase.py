from base.DriveBase import DriveBase
from buildhat import Motor

class BuildHatDriveBase(DriveBase):
    def __init__(self, front_motor_port, back_motor_port):
        """Initialize the drive base with two motors."""
        self.front_motor = Motor(front_motor_port)
        self.back_motor = Motor(back_motor_port)

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

        self.front_motor.run_for_degrees((-1)*angle, 50)  # Adjust speed as needed

    def batterylevel(self)->int:
        """Get the battery level of the drive base."""
        # Assuming the Build Hat has a method to get battery level
        return 100;
    


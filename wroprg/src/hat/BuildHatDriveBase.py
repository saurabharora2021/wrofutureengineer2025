from buildhat import Motor,ColorSensor, DistanceSensor
from base.ShutdownInterface import ShutdownInterface
import logging

class BuildHatDriveBase(ShutdownInterface):

    logger = logging.getLogger(__name__)

    def __init__(self, front_motor_port, back_motor_port,bottom_color_sensor_port, front_distance_sensor_port):
        """Initialize the drive base with two motors."""
        self.front_motor = Motor(front_motor_port)
        self.back_motor = Motor(back_motor_port)
        self.bottom_color_sensor = ColorSensor(bottom_color_sensor_port)
        self.bottom_color_sensor.on()
        self.front_distance_sensor = DistanceSensor(front_distance_sensor_port)
        self.front_distance_sensor.on()

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
        self.logger.info("Drive base shutdown complete.")

    def getBottomColor(self):
        """Get the color detected by the bottom sensor."""
        return self.bottom_color_sensor.get_color()
    
    def getBottomColorRGBI(self):
        """Get the RGB values detected by the bottom sensor."""
        return self.bottom_color_sensor.get_color_rgbi()
    
    def getFrontDistance(self):
        """Get the distance to the front obstacle."""
        return self.front_distance_sensor.get_distance()
        

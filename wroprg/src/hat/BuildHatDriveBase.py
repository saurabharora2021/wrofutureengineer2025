from buildhat import Motor,ColorSensor, DistanceSensor,Hat
from base.ShutdownInterface import ShutdownInterface
import logging

class BuildHatDriveBase(ShutdownInterface):

    logger = logging.getLogger(__name__)

    def __init__(self, front_motor_port, back_motor_port,bottom_color_sensor_port, front_distance_sensor_port):

        self.logger.warning("BuildHat start..")

        #Build Hat has a history of issues to fail first initialization on reboot. So we ensure that
        # we initialize and handle one failure before proceeding.
        try:
            Hat()  # Attempt to initialize the Build Hat
        except Exception as e:
            self.logger.error("First Buildhat Failed retry.")
            
        hat = Hat()  # Initialize the Build Hat
        self.logger.info(hat.get())  # Enumerate connected devices
        #Lets log the voltage to ensure the Build Hat is powered correctly.
        self.logger.warning("BuildHat v: %s", hat.get_vin())

        """Initialize the drive base with two motors."""
        self.front_motor = Motor(front_motor_port)
        self.back_motor = Motor(back_motor_port)
        self.bottom_color_sensor = ColorSensor(bottom_color_sensor_port)
        self.bottom_color_sensor.on()
        self.front_distance_sensor = DistanceSensor(front_distance_sensor_port)
        self.front_distance_sensor.on()
        self.logger.info("BuildHat success")

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
        self.front_motor.stop()
        self.back_motor.stop()
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
        

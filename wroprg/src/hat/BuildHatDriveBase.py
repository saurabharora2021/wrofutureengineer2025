from buildhat import Motor,ColorSensor, DistanceSensor,Hat
from base.ShutdownInterface import ShutdownInterface
import logging
import time
import math

class BuildHatDriveBase(ShutdownInterface):

    logger = logging.getLogger(__name__)

    steering_in_degrees = 0

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
        self.logger.warning("Position front:%s",self.front_motor.get_position())
        self.resetfrontmotor()  # Reset the front motor position to zero        

    def resetfrontmotor(self):
        if self.front_motor.get_position() !=0 :
            self.logger.info("BuildHat Front Motor is not at zero position, resetting it.")
            self.front_motor.run_for_degrees(-self.front_motor.get_position(),speed=50)  # Reset the front motor position to zero
            self.logger.info("After TurnPosition first round front:%s",self.front_motor.get_position())

        counter = 0
        while abs(self.front_motor.get_position()) > 2  and counter < 3:
            "BuildHat Front Motor is not at zero position, resetting it again."
            time.sleep(1)
            self.front_motor.run_for_degrees(-self.front_motor.get_position(), speed=25,blocking=True)  # Reset the front motor position to zero
            counter += 1
        self.steering_in_degrees = self.front_motor.get_position()  # Update the steering position

        self.logger.warning("After TurnPosition front:%s",self.front_motor.get_position())



    def turnsteering(self, degrees):
        """Turn the drive base by the specified degrees."""
        """For positive degrees, turn right; for negative degrees, turn left."""
        """But since it is a two gear system, we need to add a negative sign to degrees'
        and the gear ration is 1:2, so we need to multiply by 2."""

        expected_position = self.steering_in_degrees + -2*degrees;
        self.front_motor.run_for_degrees(-2*degrees, speed=25,blocking=True)
        counter = 0
        while abs(self.front_motor.get_position() - expected_position) > 2 and counter < 3:
            """If the front motor is not at the expected position, reset it."""
            self.logger.warning("Front Motor is not at expected position, resetting it.")
            self.front_motor.run_for_degrees(-1*(self.front_motor.get_position() - expected_position), speed=25,blocking=True)
            counter += 1
        if (self.front_motor.get_position() - expected_position) > 2:
            self.logger.warning("Front Motor is still not at expected position.")
        else:
            self.logger.info("Front Motor is at expected position.")


    def runfront(self, speed):
        """Run the drive base forward at the specified speed."""
        " Due to the gear combination, we need to run the motor in negative direction to move forward."
        self.back_motor.start(-1*speed)

    def stop(self):
        """Stop the drive base."""
        self.back_motor.stop()
        
    def shutdown(self):
        """Shutdown the drive base."""
        # self.front_motor.stop()
        self.resetfrontmotor()
        self.back_motor.stop()
        self.logger.info("Drive base shutdown complete.")

    def getBottomColor(self):
        """Get the color detected by the bottom sensor."""
        return self.bottom_color_sensor.get_color()
    
    def getBottomColorRGBI(self)->list:
        """Get the RGB values detected by the bottom sensor."""
        return self.bottom_color_sensor.get_color_rgbi()
    
    def getFrontDistance(self):
        """Get the distance to the front obstacle."""
        return self.front_distance_sensor.get_distance()/10  # Convert from mm to cm
        

""" This module implements the Drive Base using Build Hat motors and sensors."""
import logging
from typing import Final, Optional
from buildhat import Motor, ColorSensor, DistanceSensor, Hat
from base.shutdown_handling import ShutdownInterface

# Module-level logger that can be used without self
logger = logging.getLogger(__name__)

class BuildHatDriveBase(ShutdownInterface):
    """ This class implements the Drive Base using Build Hat motors and sensors."""

    MAX_STEERING_DEGREE: Final = 38
    # Negative gear ratio indicates that positive steering input results in a negative motor
    # rotation due to the physical gear setup.
    STEERING_GEAR_RATIO: Final = -2
    DELTA_ANGLE: float = 1

    front_distance_sensor: Optional[DistanceSensor] = None

    def __init__(self, front_motor_port: str, back_motor_port: str, bottom_color_sensor_port: str,
                  front_distance_sensor_port: Optional[str]) -> None:
        """Initialize the drive base with two motors."""

        logger.warning("BuildHat start..")

        # Build Hat has a history of issues to fail first initialization on reboot.
        # So we ensure that we initialize and handle one failure before proceeding.
        try:
            Hat()  # Attempt to initialize the Build Hat
        except Exception:  # pylint: disable=broad-except
            logger.error("First Buildhat Failed retry.")
        _hat = Hat()  # Initialize the Build Hat
        logger.info(_hat.get())  # Enumerate connected devices
        # Lets log the voltage to ensure the Build Hat is powered correctly.
        logger.warning("BuildHat v: %s", _hat.get_vin())


        self.front_motor = Motor(front_motor_port)
        self.back_motor = Motor(back_motor_port)
        self.bottom_color_sensor = ColorSensor(bottom_color_sensor_port)
        self.bottom_color_sensor.on()
        if front_distance_sensor_port is None:
            self.front_distance_sensor = None
            logger.info("Front Lego distance sensor is not connected.")
        else:
            # Initialize the front distance sensor if the port is provided
            logger.info("Front distance sensor port: %s", front_distance_sensor_port)
            self.front_distance_sensor = DistanceSensor(front_distance_sensor_port)
            self.front_distance_sensor.on()

        logger.info("BuildHat success")
        logger.warning("Position front wheel:%s", self.front_motor.get_position())
        self.reset_front_motor()  # Reset the front motor position to zero.
        self.current_position = 0.0

    def reset_front_motor(self) -> None:
        """Reset the front motor position to zero."""
        # We can only move in one direction, if the degree is greater than zero , we need
        # move it anticlockwise to zero. and if it is less than zero, we need to move it
        # clockwise to zero.
        # this is due to the steering nature of the front motor.
        if self.front_motor.get_position() != 0:
            logger.info("BuildHat Front Motor is not at zero position, resetting it.")
            self.check_set_steering(0)

        logger.warning("After TurnPosition front wheel:%s", self.front_motor.get_position())



    def turn_steering(self, degrees: float, steering_speed: float = 20,retry:int=3) -> None:
        """
        Turn the steering by the specified degrees.
        Handles outlier motor position readings.
        """
        # Helper to wrap angle to [-180, 180]
        def wrap_angle(angle):
            return ((angle + 180) % 360) - 180

        # Get current position and sanitize
        current_position = self.front_motor.get_position()
        # If the position is out of expected bounds, reset to zero
        if abs(current_position) > 180:
            logger.warning("Front motor position out of bounds: %s. Resetting to 0.",
                           current_position)
            if current_position > 0:
                self.front_motor.run_to_position(0, speed=steering_speed, blocking=True,
                                                 direction="anticlockwise")
            else:
                self.front_motor.run_to_position(0, speed=steering_speed, blocking=True,
                                                 direction="clockwise")
            current_position = 0

        # Calculate new target position, considering gear ratio and direction
        target_position = current_position + self.STEERING_GEAR_RATIO * degrees
        target_position = wrap_angle(target_position)

        # Clamp to allowed range
        target_position = max(min(target_position, self.MAX_STEERING_DEGREE),
                              -self.MAX_STEERING_DEGREE)

        # Calculate how much to move from current position
        move_degrees = target_position - current_position
        logger.info("Turning front motor to %s (move %s degrees)", target_position, move_degrees)
        self.front_motor.run_for_degrees(move_degrees, speed=steering_speed, blocking=True)

        final_position = self.front_motor.get_position()
        if abs(final_position - target_position) >= self.DELTA_ANGLE and retry > 0:
            logger.warning("Front not correct after turn: %s, expected: %s",
                           final_position, target_position)
            self.turn_steering(degrees, retry - 1)

    def check_set_steering(self, expected_position: float = 0,min_error:float = 2,
                           retrycount:int = 3,steering_speed:float=10) -> None:
        """
        Check if the steering is at the specified degrees.
        If not, set it to the specified degrees.
        """
        logger.info("set steering to %s with min_error %s and retrycount %s",
                    expected_position, min_error, retrycount)
        current_position = self.front_motor.get_position()
        counter = 0
        while abs(current_position - expected_position) > min_error and counter < retrycount:
            # If the front motor is not at the expected position, reset it.
            logger.warning("Front Motor is not at expected position, resetting it. %s",
                                current_position)
            difference =  expected_position - current_position
            self.front_motor.run_for_degrees(difference, speed=steering_speed,
                                                      blocking=True)
            counter += 1
            current_position = self.front_motor.get_position()
        if abs(current_position - expected_position) > min_error:
            logger.warning("Front Motor is still not at expected position," \
            " current position: %s, expected: %s",
                                current_position, expected_position )
        else:
            logger.info("Front Motor is at expected position.")


    def run_front(self, speed: float) -> None:
        """Run the drive base forward at the specified speed."""
        # Due to the gear combination, we need to run the motor in negative
        # direction to move forward.
        self.back_motor.start(-1 * speed)

    def stop(self) -> None:
        """Stop the drive base."""
        self.back_motor.stop()

    def shutdown(self) -> None:
        """Shutdown the drive base."""
        #self.reset_front_motor()
        self.back_motor.stop()
        logger.info("Drive base shutdown complete.")

    def get_bottom_color(self) -> str:
        """Get the color detected by the bottom sensor."""
        return self.bottom_color_sensor.get_color()

    def get_bottom_color_rgbi(self) -> list[float]:
        """Get the RGB values detected by the bottom sensor."""
        return self.bottom_color_sensor.get_color_rgbi()

    def get_front_distance(self) -> float:
        """Get the distance to the front obstacle in centimeter."""
        if self.front_distance_sensor is None:
            raise RuntimeError("Front distance sensor not initialized. Check the port connection.")
        return self.front_distance_sensor.get_distance() / 10  # Convert from mm to cm

    def get_steering_angle(self) -> float:
        """Get the current steering angle in degrees."""
        return self.front_motor.get_position() / self.STEERING_GEAR_RATIO

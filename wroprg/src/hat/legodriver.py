""" This module implements the Drive Base using Build Hat motors and sensors."""
import logging
from typing import Final, Any
from buildhat import Motor, ColorSensor, DistanceSensor, Hat
from base.shutdown_handling import ShutdownInterface

class BuildHatDriveBase(ShutdownInterface):
    """ This class implements the Drive Base using Build Hat motors and sensors."""

    logger: logging.Logger = logging.getLogger(__name__)

    MAX_STEERING_DEGREE: Final = 38
    # Negative gear ratio indicates that positive steering input results in a negative motor
    # rotation due to the physical gear setup.
    STEERING_GEAR_RATIO: Final = -2

    def __init__(self, front_motor_port: str, back_motor_port: str, bottom_color_sensor_port: str,
                  front_distance_sensor_port: str) -> None:
        """Initialize the drive base with two motors."""

        self.logger.warning("BuildHat start..")

        # Build Hat has a history of issues to fail first initialization on reboot. So we ensure that
        # we initialize and handle one failure before proceeding.
        try:
            Hat()  # Attempt to initialize the Build Hat
        except Exception:  # pylint: disable=broad-except
            self.logger.error("First Buildhat Failed retry.")
        _hat = Hat()  # Initialize the Build Hat
        self.logger.info(_hat.get())  # Enumerate connected devices
        # Lets log the voltage to ensure the Build Hat is powered correctly.
        self.logger.warning("BuildHat v: %s", _hat.get_vin())


        self.front_motor = Motor(front_motor_port)
        self.back_motor = Motor(back_motor_port)
        self.bottom_color_sensor = ColorSensor(bottom_color_sensor_port)
        self.bottom_color_sensor.on()
        self.front_distance_sensor = DistanceSensor(front_distance_sensor_port)
        self.front_distance_sensor.on()
        self.logger.info("BuildHat success")
        self.logger.warning("Position front:%s", self.front_motor.get_position())
        self.reset_front_motor()  # Reset the front motor position to zero.

    def reset_front_motor(self) -> None:
        """Reset the front motor position to zero."""
        # We can only move in one direction, if the degree is greater than zero , we need
        # move it anticlockwise to zero. and if it is less than zero, we need to move it
        # clockwise to zero.
        # this is due to the steering nature of the front motor.
        if self.front_motor.get_position() != 0:
            self.logger.info("BuildHat Front Motor is not at zero position, resetting it.")
            self.check_set_steering(0)

        self.logger.warning("After TurnPosition front:%s", self.front_motor.get_position())



    def turn_steering(self, degrees: float) -> None:
        """
        Turn the steering by the specified degrees.
        Positive degrees turn right, negative turn left.
        Limits steering to +/-38 degrees.
        """
        # Calculate new target position, considering gear ratio and direction

        current_position = self.front_motor.get_position()
        target_position = current_position + self.STEERING_GEAR_RATIO * degrees
        # Clamp to allowed range
        target_position = max(min(target_position, self.MAX_STEERING_DEGREE),
                              -self.MAX_STEERING_DEGREE)
        # Calculate how much to move from current position
        move_degrees = target_position - current_position
        self.logger.info("Turning front motor to %s (move %s degrees)", target_position,
                         move_degrees)
        self.front_motor.run_for_degrees(move_degrees, speed=25, blocking=True)
        self.check_set_steering(target_position)  # Ensure the steering is at the expected position

    def check_set_steering(self, expected_position: float = 0) -> None:
        """
        Check if the steering is at the specified degrees.
        If not, set it to the specified degrees.
        """
        current_position = self.front_motor.get_position()
        counter = 0
        while abs(current_position - expected_position) > 2 and counter < 3:
            # If the front motor is not at the expected position, reset it.
            self.logger.warning("Front Motor is not at expected position, resetting it.")
            if current_position > 0:
                # Move anticlockwise to expected position
                self.logger.info("Resetting front motor to expected position anticlockwise.")
                self.front_motor.run_to_position(expected_position, speed=25, blocking=True,
                                                  direction='anticlockwise')
            else:
                # Move clockwise to expected position
                self.logger.info("Resetting front motor to expected position clockwise.")
                self.front_motor.run_to_position(expected_position, speed=25, blocking=True,
                                                  direction='clockwise')
            counter += 1
            current_position = self.front_motor.get_position()
        if abs(current_position - expected_position) > 2:
            self.logger.warning("Front Motor is still not at expected position.")
        else:
            self.logger.info("Front Motor is at expected position.")


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
        self.reset_front_motor()
        self.back_motor.stop()
        self.logger.info("Drive base shutdown complete.")

    def get_bottom_color(self) -> Any:
        """Get the color detected by the bottom sensor."""
        return self.bottom_color_sensor.get_color()

    def get_bottom_color_rgbi(self) -> list[float]:
        """Get the RGB values detected by the bottom sensor."""
        return self.bottom_color_sensor.get_color_rgbi()

    def get_front_distance(self) -> float:
        """Get the distance to the front obstacle in centimeter."""
        return self.front_distance_sensor.get_distance() / 10  # Convert from mm to cm

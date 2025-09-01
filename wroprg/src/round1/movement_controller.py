"""Movement controller encapsulating motion commands and distance tracking."""
from __future__ import annotations

import logging
import time
from typing import Optional

from hardware.hardware_interface import HardwareInterface
from round1.utilityfunctions import clamp_angle


logger = logging.getLogger(__name__)


DIST_PER_SPEED_PER_SEC = 27.6 * 5/100 # Calibrate this value
MAX_ANGLE = 20.0
MAX_DELTA_ANGLE = 8.0
MAX_STEERING_ANGLE = 24.4

class MovementController:
    """Controls robot movement and tracks distance.

    This class centralizes start/stop driving and steering adjustments,
    and computes distance traveled.
    """

    # circumference of the wheel is 27.6
    # 1 revolution = 0.05rps
    # error due to acceleration and deceleration is ignored.

    def __init__(
        self,
        output_inf: HardwareInterface,min_speed: float) -> None:

        self.output_inf = output_inf

        self._walking: bool = False
        self._prev_turn_angle: float = -99.0

        self.min_speed = min_speed

        self.current_speed = 0
        self.start_time = 0.0
        self.distance = 0.0

    # Distance helpers
    def reset_distance(self) -> None:
        """
        Resets the distance calculator to its initial state.
        """
        self.start_time = time.monotonic()
        self.distance = 0
        logger.info("Resetting distance...")

    def get_distance(self) -> float:
        """Returns the total distance traveled."""
        # Add any pending distance from the current interval before returning
        if self._walking:
            self._add_speed()
        return self.distance

    # Motion primitives
    def start_walking(self, speed: float) -> None:
        """Start driving forward at a given speed. Handles speed changes."""
        if not self._walking:
            # Transition from stopped to moving
            self._walking = True
            self.output_inf.drive_forward(speed)
            self.current_speed = speed
            self.start_time = time.monotonic()
            logger.info("Started walking at speed: %.2f", speed)
        elif self.current_speed != speed:
            # Speed change while already moving
            self._add_speed() # Finalize distance for the previous speed
            self.output_inf.drive_forward(speed)
            self.current_speed = speed
            self.start_time = time.monotonic() # Reset timer for the new speed
            logger.info("Changed speed to: %.2f", speed)


    def stop_walking(self) -> None:
        """Stop driving and finalize distance accounting."""
        if self._walking:
            self._add_speed() # Finalize distance for the last segment
            self._walking = False
            self.output_inf.drive_stop()
            self.current_speed = 0
            self.start_time = time.monotonic() # Reset time for consistency
            logger.info("Stopping bot. Total distance: %.2f", self.distance)


    def _add_speed(self):
        """
        Calculates and adds the distance traveled during the last time interval.
        This method is called before any change in speed or when stopping.
        """
        if self.current_speed != 0 and self.start_time > 0:
            now = time.monotonic()
            elapsed_time = now - self.start_time
            self.distance += elapsed_time * self.current_speed * DIST_PER_SPEED_PER_SEC
            self.start_time = now

    def turn_steering_with_logging(
        self,
        turn_angle: Optional[float],
        *,
        current_speed: float,
        delta_angle: float = 0.0,
        speedcheck: bool = False,
        max_turn_angle: Optional[float] = None,
    ) -> None:
        """Adjust steering, clamped and rate-limited; optionally adjust speed.

        - turn_angle None: no-op; optionally ensure motion at current_speed if speedcheck.
        - Clamps per max_delta_angle step from current steering and to max_turn_angle.
        """
        if turn_angle is None:
            logger.debug("Turn angle is None")
            if speedcheck:
                self.start_walking(current_speed)
            return

        current_steering_angle = self.output_inf.get_steering_angle()
        max_delta_angle = MAX_DELTA_ANGLE if delta_angle == 0 else delta_angle

        delta = float(turn_angle) - current_steering_angle
        if abs(delta) > max_delta_angle:
            turn_angle = current_steering_angle + (
                max_delta_angle if delta > 0 else -max_delta_angle
            )

        # Clamp to provided or controller max steering angle
        turn_angle = clamp_angle(turn_angle, max_turn_angle or MAX_ANGLE)

        if turn_angle == self._prev_turn_angle:
            logger.info("Turn angle same as previous; skipping turn")
            self._prev_turn_angle = 0.0
            return
        else:
            self._prev_turn_angle = turn_angle

        if delta >= 0:
            logger.info(
                "Turning right from %.2f to %.2f: %.2f",
                current_steering_angle,
                turn_angle,
                delta,
            )
        else:
            logger.info(
                "Turning left from %.2f to %.2f: %.2f",
                current_steering_angle,
                turn_angle,
                delta,
            )

        # Slow down for large corrections if requested
        if speedcheck and abs(delta) >= MAX_DELTA_ANGLE:
            self.start_walking(self.min_speed)

        self.output_inf.turn_steering(turn_angle)

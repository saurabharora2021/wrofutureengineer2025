"""Movement controller encapsulating motion commands and distance tracking."""
import logging
import time
from typing import Optional
import queue
import threading

from hardware.hardware_interface import HardwareInterface
from round1.utilityfunctions import clamp_angle


logger = logging.getLogger(__name__)

# Calibrate this value
DIST_PER_SPEED_PER_SEC = 0.84   #27.6 * 5/100
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

        # Thread-safe queue for turn requests with a buffer of 1
        self._turn_queue = queue.Queue(maxsize=1)
        # Start the worker thread to process turn commands
        self._turn_worker_thread = threading.Thread(target=self._turn_worker, daemon=True)
        self._turn_worker_thread.start()

    def is_walking(self) -> bool:
        """Returns whether the robot is currently walking."""
        return self._walking

    def _turn_worker(self):
        """Worker thread that processes turn commands from the queue."""
        while True:
            # Wait for a turn angle to be available in the queue
            turn_angle = self._turn_queue.get()
            # A None value is a signal to terminate the thread
            if turn_angle is None:
                self._turn_queue.task_done()
                break

            try:
                # Execute the blocking turn command
                self.output_inf.turn_steering(turn_angle)
            finally:
                # Signal that the task is complete
                self._turn_queue.task_done()

    def shutdown(self):
        """Gracefully shuts down the turn worker thread."""
        logger.info("Shutting down movement controller...")
        self._turn_queue.put(None) # Signal the worker to exit
        self._turn_worker_thread.join() # Wait for the thread to finish
        logger.info("Movement controller shut down.")

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
        # Block until any pending turn operation is complete
        self._turn_queue.join()

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

    def start_backward(self,speed:float)->None:
        """Start driving backward at a given speed."""
        # Block until any pending turn operation is complete
        self._turn_queue.join()

        if not self._walking:
            # Transition from stopped to moving
            self._walking = True
            self.output_inf.drive_backward(speed)
            self.current_speed = -speed
            self.start_time = time.monotonic()
            logger.info("Started walking backward at speed: %.2f", speed)
        elif self.current_speed != speed:
            # Speed change while already moving
            self._add_speed() # Finalize distance for the previous speed
            self.output_inf.drive_backward(speed)
            self.current_speed = -speed
            self.start_time = time.monotonic() # Reset timer for the new speed

    def stop_walking(self) -> None:
        """Stop driving and finalize distance accounting."""
        if self._walking:
            self.output_inf.drive_stop()
            self._add_speed() # Finalize distance for the last segment
            self._walking = False
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
        delta_angle: float = MAX_DELTA_ANGLE,
        speedcheck: bool = False,
        max_turn_angle: Optional[float] = None,
        async_turn:bool = False
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
        max_delta_angle = delta_angle

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
        if speedcheck and abs(delta) >= MAX_DELTA_ANGLE and self._walking:
            self.start_walking(self.min_speed)

        if async_turn:
            # Put the turn command in the queue instead of calling it directly.
            # This will block if the queue is full (i.e., another turn is in progress).
            try:
                self._turn_queue.put(turn_angle, block=True)
            except Exception as e:
                logger.error("Failed to queue turn command: %s", e)
        else:
            #if someone is asking for ansync turn, lets clear queue first.
            self._turn_queue.join()
            self.output_inf.turn_steering(turn_angle)

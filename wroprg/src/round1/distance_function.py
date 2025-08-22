"""Distance Calculation Module"""
import time

class DistanceCalculator:
    """
    Calculates the total distance traveled given speed values at different points in time.
    """
    # circumference of the wheel is 27.6
    # 1 revolution = 0.05rps
    # error due to acceleration and deceleration is ignored.
    WHEEL_CONSTANT = 27.6 * 5/100

    def __init__(self):
        self.current_speed = 0
        self.start_time = 0
        self.distance = 0

    def reset(self):
        """
        Resets the distance calculator to its initial state.
        """
        self.current_speed = 0
        self.start_time = 0
        self.distance = 0

    def stop(self):
        """
        Stops the distance calculation.
        """
        self._add_speed()
        self.current_speed = 0

    def _add_speed(self):
        """
        Adds a speed data point.
        This method is called when the speed changes.
        """
        if self.current_speed != 0:
            end_time = time.time()
            elapsed_time = end_time - self.start_time
            self.distance += elapsed_time * self.current_speed * self.WHEEL_CONSTANT
            self.start_time = end_time

    def run_speed(self, speed: float):
        """
        Runs the speed calculation.
        :param speed: Speed at the current time (units per second).
        """
        self._add_speed()
        self.current_speed = speed

    def get_distance(self) -> float:
        """Returns the total distance traveled."""
        return self.distance

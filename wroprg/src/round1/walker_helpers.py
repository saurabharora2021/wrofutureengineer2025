"""Helper functions for Walker logic in WRO2025."""
import logging
from statistics import mean as average
from collections import deque

logger = logging.getLogger(__name__)

class EquiWalkerHelper:
    """Helper class for Round 1 mathetical Function"""

    MAX_ANGLE = 10
    DELTA_DISTANCE_CM = 1.5
    EQUIWALKMAXDELTA=15

    def __init__(self,def_distance_left: float, def_distance_right: float,
                 max_left_distance: float, max_right_distance: float):
        self._queue = deque(maxlen=2)
        self.def_distance_left = def_distance_left
        self.def_distance_right = def_distance_right
        self.max_left_distance = max_left_distance
        self.max_right_distance = max_right_distance

        if def_distance_left < 10:
            logger.warning("Left distance is less than 10 cm, reset at least 10 cm.")
            self.def_distance_left = 10
            self.def_distance_right= def_distance_right - (10- def_distance_left)
        elif def_distance_right < 10:
            logger.warning("Right distance is less than 10 cm, reset at least 10 cm.")
            self.def_distance_right = 10
            self.def_distance_left = def_distance_left - (10 - def_distance_right)


        logger.info("Default Left: %.2f, Right: %.2f",
                     def_distance_left, def_distance_right)


    def equidistance_walk_func(self, left_distance: float,
                               right_distance: float, kp: float=-1) -> float:
        """Calculate the angle for equidistance walk based on the current distances."""
        errorcount = 0
        # If you are at a point, where the left rail is not present or the right rail is
        # not present, then we will not adjust the steering.

        if left_distance >= self.max_left_distance or left_distance <= 0:
            logger.warning("Left distance is not set.")
            left_distance = self.def_distance_left
            errorcount += 1
        if right_distance >= self.max_right_distance or right_distance <= 0:
            logger.warning("Right distance is not set.")
            right_distance = self.def_distance_right
            errorcount += 1

        left_delta = left_distance - self.def_distance_left
        right_delta = right_distance - self.def_distance_right


        if (abs(left_delta) + abs(right_delta) > self.DELTA_DISTANCE_CM and errorcount < 2):

            logger.warning("Left Delta: %.2f, Right Delta: %.2f", left_delta, right_delta)
            # Handle sudden changes in distance
            left_delta = max(min(left_delta, self.EQUIWALKMAXDELTA), -self.EQUIWALKMAXDELTA)
            right_delta = max(min(right_delta, self.EQUIWALKMAXDELTA), -self.EQUIWALKMAXDELTA)
            # Adjust steering based on the difference in distances
            error = left_delta - right_delta
            if errorcount > 0:
                # If there was an error in any distance, we double the error
                error = error * 2
            angle = self.clamp_angle(kp * error)
            logger.warning("angle: %.2f", angle)
            self._queue.append(angle)
            final_angle = average(self._queue)
            logger.warning("Final angle: %.2f", final_angle)
            return final_angle
        else:
            return None

    def clamp_angle(self, val):
        """Clamp the value between -MAX_ANGLE and MAX_ANGLE."""
        return max(min(val, self.MAX_ANGLE), -self.MAX_ANGLE)

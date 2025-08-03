"""Helper functions for Walker logic in WRO2025."""
import logging
from statistics import mean as average
from collections import deque

logger = logging.getLogger(__name__)

class EquiWalkerHelper:
    """Helper class for Round 1 mathetical Function"""

    DELTA_DISTANCE_CM = 1
    EQUIWALKMAXDELTA=13
    MAX_GYRO_DELTA = 0.5 # Maximum gyro delta angle in degrees
    MIN_WALL_DISTANCE = 10  # Minimum distance to consider a wall present
    # Gyro correction factor,
    # this is multiplied with the delta gyro value, which is quite small
    K_GYRO = 5
    K_DISTANCE = -2
    MAX_ANGLE = 15 # Maximum angle in degrees for steering adjustments

    def __init__(self,def_distance_left: float, def_distance_right: float,
                 max_left_distance: float, max_right_distance: float,
                 kp:float) -> None:
        self._queue = deque(maxlen=2)
        self._queue.append(0.0)  # Initialize with a default value
        self.def_distance_left = def_distance_left
        self.def_distance_right = def_distance_right
        self.max_left_distance = max_left_distance
        self.max_right_distance = max_right_distance
        if kp == 0:
            self.kp = self.K_DISTANCE
        else:
            self.kp = kp

        if def_distance_left < self.MIN_WALL_DISTANCE:
            logger.warning("Left distance is less than 10 cm, reset at least 10 cm.")
            self.def_distance_left = self.MIN_WALL_DISTANCE
            self.def_distance_right= def_distance_right - (self.MIN_WALL_DISTANCE
                                                            - def_distance_left)
        elif def_distance_right < self.MIN_WALL_DISTANCE:
            logger.warning("Right distance is less than 10 cm, reset at least 10 cm.")
            self.def_distance_right = self.MIN_WALL_DISTANCE
            self.def_distance_left = def_distance_left - (self.MIN_WALL_DISTANCE
                                                           - def_distance_right)


        logger.info("Default Left: %.2f, Right: %.2f, Kp: %.2f",
                     def_distance_left, def_distance_right,self.kp)

    def clamp_angle(self, val)-> float:
        """Clamp the value between -MAX_ANGLE and MAX_ANGLE."""
        return float(max(min(val, self.MAX_ANGLE), -self.MAX_ANGLE))



    def equidistance_walk_func(self, left_distance: float,right_distance: float,
                            current_angle:float,current_steering_angle:float) -> float:
        """Calculate the angle for equidistance walk based on the current distances."""
        errorcount = 0
        # If you are at a point, where the left rail is not present or the right rail is
        # not present, then we will not adjust the steering.

        delta_angle = current_angle

        logger.info("Gyro angle : %.2f", delta_angle)
        logger.info("current steering angle: %.2f", current_steering_angle)

        left_delta = left_distance - self.def_distance_left
        right_delta = right_distance - self.def_distance_right


        gyro_correction =0

        if abs(delta_angle) >= self.MAX_GYRO_DELTA:
            logger.warning("Delta angle is too high: %.2f, reducing angles now", delta_angle)
            if delta_angle > 0:
                logger.info("Drifting left, adjusting right gyro correction")
                gyro_correction = self.K_GYRO * abs(delta_angle)
            else:
                logger.info("Drifting right, adjusting left gyro correction")
                gyro_correction = -self.K_GYRO * abs(delta_angle)
            # if left_delta > right_delta:
            #     #we are drifting toward right.go left
            #     if left_delta < 15:
            #         logger.info("Drifting right, adjusting left gyro correction")
            #         gyro_correction = -self.K_GYRO*abs(delta_angle)
            #     else:
            #         # check if left delta is not max
            #         if left_distance == self.max_left_distance:
            #             #left wall is not present or tilt too high
            #             # we can only look at right delta.
            #             if right_delta > 0:
            #                 logger.info("Drifting right, adjusting left gyro correction")
            #                 gyro_correction = -self.K_GYRO*abs(delta_angle)
            #             else:
            #                 logger.info("Drifting left, adjusting right gyro correction")
            #                 gyro_correction = self.K_GYRO*abs(delta_angle)
            #         else:
            #             logger.info("Drifting left, adjusting right gyro correction")
            #             gyro_correction = self.K_GYRO*abs(delta_angle)
            #             left_delta = 15
            # elif left_delta < right_delta:
            #     #we are drifting toward left.go right
            #     if right_delta < 15:
            #         logger.info("Drifting left, adjusting right gyro correction")
            #         gyro_correction = self.K_GYRO*abs(delta_angle)
            #     else:
            #         if right_distance == self.max_right_distance:
            #             #right wall is not present or tilt too high
            #             # we can only look at left delta.
            #             if left_delta > 0:
            #                 logger.info("Drifting left, adjusting right gyro correction")
            #                 gyro_correction = self.K_GYRO*abs(delta_angle)
            #             else:
            #                 logger.info("Drifting right, adjusting left gyro correction")
            #                 gyro_correction = -self.K_GYRO*abs(delta_angle)
            #         else:
            #             logger.info("Drifting left, adjusting right gyro correction")
            #             gyro_correction = self.K_GYRO*abs(delta_angle)
            #             left_delta = 15

            #         logger.info("Drifting right, adjusting left gyro correction")
            #         gyro_correction = -self.K_GYRO*abs(delta_angle)
            #         right_delta = 15

        if left_distance >= self.max_left_distance or left_distance <= 0:
            logger.warning("Left distance is not set.")
            left_distance = self.def_distance_left
            errorcount += 1
        if right_distance >= self.max_right_distance or right_distance <= 0:
            logger.warning("Right distance is not set.")
            right_distance = self.def_distance_right
            errorcount += 1

        logger.warning("Left Delta: %.2f, Right Delta: %.2f , gyro correction: %.2f",
                           left_delta, right_delta, gyro_correction)

        if ((abs(left_delta) + abs(right_delta) > self.DELTA_DISTANCE_CM or gyro_correction!=0)
                        and errorcount < 2):

            # Handle sudden changes in distance
            left_delta = max(min(left_delta, self.EQUIWALKMAXDELTA), -self.EQUIWALKMAXDELTA)
            right_delta = max(min(right_delta, self.EQUIWALKMAXDELTA), -self.EQUIWALKMAXDELTA)
            # Adjust steering based on the difference in distances
            error = left_delta - right_delta
            if errorcount > 0:
                # If there was an error in any distance, we double the error
                error = error * 2
            angle = self.clamp_angle(self.kp * error+gyro_correction)
            logger.warning("angle after gyro %.2f", angle)
            self._queue.append(angle)
            final_angle = average(self._queue)
            logger.warning("Final angle: %.2f", final_angle)
            return float(final_angle)
        else:
            return None # type: ignore

class GyroWalkerHelper:
    """Helper class for Gyro Walker logic in WRO2025."""
    MAX_ANGLE = 15 # Maximum angle in degrees for steering adjustments
    MAX_GYRO_DELTA = 0.5 # Maximum gyro delta angle in degrees
    K_GYRO = 5
    def __init__(self, kgyro: float=0) -> None:
        self.kgyro = kgyro if kgyro != 0 else self.K_GYRO
        logger.info("Gyro Walker Helper initialized with Kgyro: %.2f", self.kgyro)

    def walk_func(self, delta_angle:float, current_steering_angle:float) -> float:
        """Calculate the angle for gyro walk based on the current delta angle."""

        logger.info("Gyro angle : %.2f", delta_angle)
        logger.info("current steering angle: %.2f", current_steering_angle)

        gyro_correction =0

        if abs(delta_angle) >= self.MAX_GYRO_DELTA:
            logger.warning("Delta angle is too high: %.2f, reducing angles now", delta_angle)
            if delta_angle > 0:
                logger.info("Drifting left, adjusting right gyro correction")
                gyro_correction = self.K_GYRO * abs(delta_angle)
            else:
                logger.info("Drifting right, adjusting left gyro correction")
                gyro_correction = -self.K_GYRO * abs(delta_angle)

        angle = self.clamp_angle(self.kgyro*gyro_correction)
        if current_steering_angle != angle:
            logger.info("Adjusting steering angle from %.2f to %.2f", current_steering_angle, angle)
            return float(angle)
        else:
            return None  # type: ignore

    def clamp_angle(self, val)-> float:
        """Clamp the value between -MAX_ANGLE and MAX_ANGLE."""
        return float(max(min(val, self.MAX_ANGLE), -self.MAX_ANGLE))

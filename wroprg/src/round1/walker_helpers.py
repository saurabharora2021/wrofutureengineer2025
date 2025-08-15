"""Helper functions for Walker logic in WRO2025."""
import logging
from statistics import mean as average
from collections import deque
from round1.utilityfunctions import clamp_angle

logger = logging.getLogger(__name__)

class EquiWalkerHelper:
    """Helper class for Round 1 mathetical Function"""

    DELTA_DISTANCE_CM = 1
    EQUIWALKMAXDELTA=13
    MAX_GYRO_DELTA = 0.01 # Maximum gyro delta angle in degrees
    MIN_WALL_DISTANCE = 10  # Minimum distance to consider a wall present
    # Gyro correction factor,
    # this is multiplied with the delta gyro value, which is quite small
    K_GYRO = 10
    K_DISTANCE = -4
    MAX_ANGLE = 15 # Maximum angle in degrees for steering adjustments

    def __init__(self,def_distance_left: float, def_distance_right: float,
                 max_left_distance: float, max_right_distance: float,
                 kp:float, def_turn_angle:float=0.0
                 ) -> None:
        self._queue = deque(maxlen=2)
        self._queue.append(0.0)  # Initialize with a default value
        self.def_distance_left = def_distance_left
        self.def_distance_right = def_distance_right
        self.max_left_distance = max_left_distance
        self.max_right_distance = max_right_distance
        self.def_turn_angle = def_turn_angle
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

    def reset_walk_distances(self, def_distance_left: float, def_distance_right: float) -> None:
        """Reset the default distances for equidistance walking."""
        self.def_distance_left = def_distance_left
        self.def_distance_right = def_distance_right
        logger.info("Reset default distances to Left: %.2f, Right: %.2f",
                     def_distance_left, def_distance_right)

    def equidistance_walk_func(self, left_distance: float,right_distance: float,
                            current_angle:float,current_steering_angle:float) -> float:
        """Calculate the angle for equidistance walk based on the current distances."""
        errorcount = 0
        # If you are at a point, where the left rail is not present or the right rail is
        # not present, then we will not adjust the steering.

        delta_angle = current_angle - self.def_turn_angle

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

        if left_distance >= self.max_left_distance or left_distance <= 0:
            logger.warning("Left distance is not set.")
            left_distance = self.def_distance_left
            left_delta = 0
            errorcount += 1
        if right_distance >= self.max_right_distance or right_distance <= 0:
            logger.warning("Right distance is not set.")
            right_distance = self.def_distance_right
            right_delta = 0
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
            angle = clamp_angle(self.kp * error+gyro_correction, self.MAX_ANGLE)
            logger.info("error %.2f , Kp %.2f", error, self.kp)

            logger.warning("angle after gyro %.2f", angle)
            self._queue.append(angle)
            final_angle = average(self._queue)
            logger.warning("Final angle: %.2f", final_angle)
            return float(final_angle)
        else:
            return None # type: ignore

class GyroWalkerHelper:
    """Helper class for Gyro Walker logic in WRO2025."""
    MAX_ANGLE = 13 # Maximum angle in degrees for steering adjustments
    MAX_GYRO_DELTA = 0.5 # Maximum gyro delta angle in degrees
    K_GYRO = 4
    def __init__(self, kgyro: float=0, walk_angle: float=0) -> None:
        self.kgyro = kgyro if kgyro != 0 else self.K_GYRO
        self.walk_angle = walk_angle
        logger.info("Gyro Walker Helper initialized with Kgyro: %.2f", self.kgyro)

    def walk_func(self, current_angle:float, current_steering_angle:float) -> float:
        """Calculate the angle for gyro walk based on the current delta angle."""

        logger.info("Gyro angle : %.2f", current_angle)
        logger.info("current steering angle: %.2f", current_steering_angle)

        delta_angle = current_angle - self.walk_angle
        gyro_correction =0

        if abs(delta_angle) >= self.MAX_GYRO_DELTA:
            logger.warning("Delta angle is too high: %.2f, reducing angles now", delta_angle)
            if delta_angle > 0:
                logger.info("Drifting left, adjusting right gyro correction")
                gyro_correction = self.K_GYRO * abs(delta_angle)
            else:
                logger.info("Drifting right, adjusting left gyro correction")
                gyro_correction = -self.K_GYRO * abs(delta_angle)

        angle = clamp_angle(self.kgyro*gyro_correction, self.MAX_ANGLE)
        logger.info("clamp gyro steering angle: %.2f", angle)
        if current_steering_angle != angle:
            logger.info("Adjusting steering angle from %.2f to %.2f", current_steering_angle, angle)
            return float(angle)
        else:
            return None  # type: ignore

class GyroWalkerwithMinDistanceHelper:
    """Helper class for Gyro Walker logic with distance."""
    MAX_ANGLE = 20 # Maximum angle in degrees for steering adjustments
    MAX_GYRO_DELTA = 0.5 # Maximum gyro delta angle in degrees
    K_GYRO = 20
    K_DISTANCE = -3.5
    def __init__(self, kgyro: float=0,kdistance:float=0, walk_angle: float=0,
                                            min_left:float=-1,min_right:float=-1) -> None:
        self.kgyro = kgyro if kgyro != 0 else self.K_GYRO
        self.kdistance = kdistance if kdistance != 0 else self.K_DISTANCE
        self.walk_angle = walk_angle
        self.min_left = min_left
        self.min_right = min_right
        logger.info("GWMD Kgyro: %.2f , Kdistance: %.2f", self.kgyro, self.kdistance)
        logger.info("GWMD walk_angle: %.2f", self.walk_angle)
        logger.info("GWMD min_left: %.2f", self.min_left)
        logger.info("GWMD min_right: %.2f", self.min_right)

    def walk_func(self, current_angle:float, current_steering_angle:float,
                                                current_left:float,current_right:float) -> float:
        """Calculate the angle for gyro walk based on the current delta angle."""

        logger.info("Gyro angle : %.2f", current_angle)
        logger.info("current steering angle: %.2f", current_steering_angle)

        delta_angle = current_angle - self.walk_angle
        gyro_correction =0
        distance_angle_correction = 0

        if abs(delta_angle) >= self.MAX_GYRO_DELTA:
            logger.warning("Delta angle is too high: %.2f, reducing angles now", delta_angle)
            if delta_angle > 0:
                logger.info("Drifting left, adjusting right gyro correction")
                gyro_correction = self.kgyro * abs(delta_angle)
            else:
                logger.info("Drifting right, adjusting left gyro correction")
                gyro_correction = -self.kgyro * abs(delta_angle)

        if self.min_left != -1 and current_left < self.min_left:
            logger.info("Gyro min distance walk:left is less %f",current_left)
            distance_angle_correction += self.kdistance * (self.min_left - current_left)
        if self.min_right != -1 and current_right < self.min_right:
            logger.info("Gyro min distance walk:right is less %f",current_right)
            distance_angle_correction += self.kdistance * (self.min_right - current_right)

        angle = clamp_angle(gyro_correction + distance_angle_correction, self.MAX_ANGLE)
        logger.info("clamp gyro steering angle: %.2f", angle)
        if current_steering_angle != angle:
            logger.info("Adjusting steering angle from %.2f to %.2f", current_steering_angle, angle)
            return float(angle)
        else:
            return None  # type: ignore

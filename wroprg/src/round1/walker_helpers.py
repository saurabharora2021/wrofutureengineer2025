"""Helper functions for Walker logic in WRO2025."""
import logging
from statistics import mean as average
from collections import deque
from typing import Optional
import time
from round1.utilityfunctions import clamp_angle


logger = logging.getLogger(__name__)

MAX_ANGLE = 30
class EquiWalkerHelper:
    """Helper class for Round 1 mathetical Function"""

    DELTA_DISTANCE_CM = 1
    EQUIWALKMAXDELTA=13
    MAX_GYRO_DELTA = 0.05 # Maximum gyro delta angle in degrees
    MIN_WALL_DISTANCE = 10  # Minimum distance to consider a wall present

    def __init__(self,def_distance_left: float, def_distance_right: float,
                 max_left_distance: float, max_right_distance: float,
                 kp:float=-4.0, ki:float=0.1, kd:float=0.05, kgyro:float=5,
                   def_turn_angle:float=0.0
                 ) -> None:
        self._queue = deque(maxlen=2)
        self._queue.append(0.0)  # Initialize with a default value
        self.def_distance_left = def_distance_left
        self.def_distance_right = def_distance_right
        self.max_left_distance = max_left_distance
        self.max_right_distance = max_right_distance
        self.def_turn_angle = def_turn_angle
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kgyro = kgyro
        self._prev_time = time.perf_counter()
        self._integral = 0
        self._prev_error = 0

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

    def equidistance_walk_func(self, left_distance: float, right_distance: float,
                            current_angle: float) -> Optional[float]:
        """Calculate the steering angle using full PID control."""
        errorcount = 0

        # Calculate time delta
        now = time.perf_counter()
        dt = max(0.001, now - self._prev_time)  # Minimum 1ms to avoid division by zero
        self._prev_time = now

        # Gyro correction
        delta_angle = current_angle - self.def_turn_angle
        gyro_correction = 0.0
        if abs(delta_angle) >= self.MAX_GYRO_DELTA:
            gyro_correction = self.kgyro * delta_angle

        # Distance error calculation
        left_delta = left_distance - self.def_distance_left
        right_delta = right_distance - self.def_distance_right

        # Validate distances
        if left_distance >= self.max_left_distance or left_distance <= 0:
            left_delta = 0.0
            errorcount += 1
        if right_distance >= self.max_right_distance or right_distance <= 0:
            right_delta = 0.0
            errorcount += 1

        # If both sensors are invalid, return None
        if errorcount >= 2:
            return None

        # Control error: positive means steer right
        distance_error = left_delta - right_delta

        # Deadband to avoid small corrections
        if abs(distance_error) < self.DELTA_DISTANCE_CM and \
                                abs(gyro_correction) < self.MAX_GYRO_DELTA:
            return None

        # Sensor fusion: Combine gyro and distance errors
        gyro_weight = 0.6  # Weight for gyro error
        distance_weight = 0.4  # Weight for distance error
        fused_error = (gyro_weight * gyro_correction) + (distance_weight * distance_error)

        # PID calculations
        # Proportional term
        p_term = self.kp * fused_error

        # Integral term with anti-windup
        if self.ki != 0:
            self._integral += fused_error * dt
            self._integral = max(min(self._integral, MAX_ANGLE / self.ki), -MAX_ANGLE / self.ki)
            i_term = self.ki * self._integral
        else:
            i_term = 0.0

        # Derivative term
        if self.kd != 0 and self._prev_error is not None:
            d_term = self.kd * (fused_error - self._prev_error) / dt
        else:
            d_term = 0.0

        self._prev_error = fused_error

        # Combine PID terms and gyro correction
        raw_angle = p_term + i_term + d_term + gyro_correction
        angle = clamp_angle(raw_angle, MAX_ANGLE)

        # Smoothing
        self._queue.append(angle)
        final_angle = average(self._queue)

        logger.debug("PID: P=%.2f I=%.2f D=%.2f Gyro=%.2f -> %.2f",
                     p_term, i_term, d_term, gyro_correction, final_angle)

        return float(final_angle)

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
        gyro_correction = 0
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

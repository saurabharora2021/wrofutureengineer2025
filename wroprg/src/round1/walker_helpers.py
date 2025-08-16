"""Helper functions for Walker logic in WRO2025."""
import logging
from typing import Optional
import time
from round1.utilityfunctions import clamp_angle


logger = logging.getLogger(__name__)


MAX_ANGLE = 30
MAX_GYRO_DELTA = 0.05 # Maximum gyro delta angle in degrees
DELTA_DISTANCE_CM = 1
class PIDController:
    """Simple PID controller."""

    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._prev_error = 0
        self._integral = 0
        self._prev_time = time.perf_counter()

    def calculate(self, error: float) -> float:
        """Calculate the PID output value for the given error."""

        now = time.perf_counter()
        dt = max(0.001, now - self._prev_time)  # Minimum 1ms to avoid division by zero
        self._prev_time = now

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        i_term = 0
        if self.ki != 0:
            self._integral += error * dt
            self._integral = clamp_angle(self._integral, MAX_ANGLE/self.ki)
            i_term = self.ki * self._integral

        # Derivative term
        d_term = 0
        if self.kd != 0:
            d_term = self.kd * (error - self._prev_error) / dt

        # Save error for next calculation
        self._prev_error = error

        # Combine PID terms
        output = p_term + i_term + d_term

        return clamp_angle(output, MAX_ANGLE)

class EquiWalkerHelper:
    """Helper class for equidistance walking with PID control."""

    def __init__(self, def_distance_left: float, def_distance_right: float,
                 max_left_distance: float, max_right_distance: float,
                 kp: float = -4.0, ki: float = 0.0, kd: float = 0.05,
                 kgyro: float = 5.0, def_turn_angle: float = 0.0,
                 fused_distance_weight: float = 0.5, fused_gyro_weight: float = 0.5):
        self.def_distance_left = def_distance_left
        self.def_distance_right = def_distance_right
        self.max_left_distance = max_left_distance
        self.max_right_distance = max_right_distance
        self.def_turn_angle = def_turn_angle
        self.kgyro = kgyro
        self.fused_distance_weight = fused_distance_weight
        self.fused_gyro_weight = fused_gyro_weight
        self.pid = PIDController(kp, ki, kd)

    def walk_func(self, left_distance: float, right_distance: float,
                               current_angle: float) -> Optional[float]:
        """Calculate the steering angle using sensor fusion PID control."""
        # Gyro correction
        delta_angle = current_angle - self.def_turn_angle
        gyro_correction = 0.0
        if abs(delta_angle) >= MAX_GYRO_DELTA:
            gyro_correction = self.kgyro * delta_angle

        # Distance error calculation
        left_delta = left_distance - self.def_distance_left
        right_delta = right_distance - self.def_distance_right

        # Validate distances
        if left_distance >= self.max_left_distance or left_distance <= 0:
            left_delta = 0.0
        if right_distance >= self.max_right_distance or right_distance <= 0:
            right_delta = 0.0

        # Control error: positive means steer right
        distance_error = left_delta - right_delta

        # Deadband to avoid small corrections
        if abs(distance_error) < DELTA_DISTANCE_CM and \
                                abs(gyro_correction) < MAX_GYRO_DELTA:
            return None

        # Sensor fusion: Combine gyro and distance errors
        fused_error = (self.fused_gyro_weight * gyro_correction) + (self.fused_distance_weight * distance_error)

        # Deadband to avoid small corrections
        if abs(fused_error) < 0.1:
            return None

        # Use shared PID logic
        return self.pid.calculate(fused_error)
class GyroWalkerwithMinDistanceHelper:
    """Helper class for Gyro Walker logic with distance."""

    def __init__(self, kp: float = -4.0, ki: float = 0.0, kd: float = 0.05, kgyro: float = 5.0,
                 walk_angle: float = 0.0, min_left: float = -1, min_right: float = -1,
                 fused_distance_weight: float = 0.6, fused_gyro_weight: float = 0.4) -> None:
        self.walk_angle = walk_angle
        self.min_left = min_left
        self.min_right = min_right
        self.fused_distance_weight = fused_distance_weight
        self.fused_gyro_weight = fused_gyro_weight
        self.kgyro = kgyro

        # Initialize the shared PID controller
        self.pid = PIDController(kp, ki, kd)


    def walk_func(self, left_distance: float, right_distance: float,
                               current_angle: float) -> Optional[float]:
        """Calculate the steering angle using sensor fusion PID control."""
        # Gyro correction
        delta_angle = current_angle - self.walk_angle

        gyro_correction = 0.0
        if abs(delta_angle) >= MAX_GYRO_DELTA:
            gyro_correction = self.kgyro * delta_angle

        # Distance correction
        distance_error = 0.0
        if self.min_left != -1 and left_distance < self.min_left:
            distance_error += self.min_left - left_distance
        if self.min_right != -1 and right_distance < self.min_right:
            distance_error += self.min_right - right_distance

        # Sensor fusion: Combine gyro and distance corrections
        fused_error = (self.fused_gyro_weight * gyro_correction) + (self.fused_distance_weight * distance_error)

        # Deadband to avoid small corrections
        if abs(fused_error) < 0.1:
            return None

        # Use the shared PID controller to calculate the steering angle
        return self.pid.calculate(fused_error)

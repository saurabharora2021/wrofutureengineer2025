"""Helper functions for Walker logic in WRO2025."""
import logging
from typing import Optional,List
import time
from abc import ABC
from hardware.hardware_interface import HardwareInterface
from round1.utilityfunctions import clamp_angle

logger = logging.getLogger(__name__)


MAX_ANGLE = 30
MIN_GYRO_DELTA = 0.05 # Minimum gyro delta angle in degrees
DELTA_DISTANCE_CM = 0.1
class PIDController:
    """Simple PID controller."""

    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._prev_error = 0
        self._integral = 0
        self._prev_time = time.perf_counter()

    def reset(self):
        """reset to zero prev values"""
        self._prev_error = 0
        self._integral = 0
        self._prev_time = time.perf_counter()

    def calculate(self, error: float) -> float:
        """Calculate the PID output value for the given error."""

        now = time.perf_counter()
        dt = min(max(0.001, now - self._prev_time),0.1)  # Minimum 1ms to avoid division by zero
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

        logger.info("PID angle: %.2f", output)
        return clamp_angle(output, MAX_ANGLE)

class EquiWalkerHelper(ABC):
    """Helper class for equidistance walking with PID control."""

    def __init__(self, def_distance_left: float, def_distance_right: float,
                 max_left_distance: float, max_right_distance: float,
                 kp: float = -4.0, ki: float = 0.0, kd: float = -0.05,
                 kgyro: float = -5.0, def_turn_angle: float = 0.0,
                 fused_distance_weight: float = 0.5, fused_gyro_weight: float = 0.5,
                 hardware: Optional[HardwareInterface]=None,
                 min_left: float = 10.00, min_right: float = 10.00) -> None:
# Default before tuning
# ki: float = -0.1
#kd: float = -0.05
        self.def_distance_left = def_distance_left
        self.def_distance_right = def_distance_right
        self.max_left_distance = max_left_distance
        self.max_right_distance = max_right_distance
        self.def_turn_angle = def_turn_angle
        self.kgyro = kgyro
        self.fused_distance_weight = fused_distance_weight
        self.fused_gyro_weight = fused_gyro_weight
        self.min_left = min_left
        self.min_right = min_right
        self.pid = PIDController(kp, ki, kd)
        self.hardware: Optional[HardwareInterface] = hardware
        logger.info("EquiWalkerHelper initialized ...")
        logger.info("EquiWalkerHelper distances: left=%.2f, right=%.2f", def_distance_left,
                     def_distance_right)
        logger.info("EquiWalkerHelper parameters: kgyro=%.2f, def_turn_angle=%.2f, \
                    fused_distance_weight=%.2f, fused_gyro_weight=%.2f",
                    kgyro, def_turn_angle, fused_distance_weight, fused_gyro_weight)

    def log_walk_data(self,distance_error:float,gyro_error:float,fused_error:float,
                      turn:float) -> None:
        """Log walking data to the hardware interface."""
        message:List[str] = [
            f"Error D :{distance_error:.2f} G :{gyro_error:.2f}",
            f"Fuse: {fused_error:.2f} Tu:{turn:.2f}",
            ""
        ]
        if self.hardware is not None:
            self.hardware.add_screen_logger_message(message)
            logger.info("Walk data logged: %s", message)
        else:
            logger.error("Hardware interface not set, cannot log walk data.")
            logger.info(message)

    def process_error(self, distance_error: float, gyro_correction: float) -> Optional[float]:
        """Process the errors and return the steering angle."""
        # Sensor fusion: Combine gyro and distance errors
        fused_error = (self.fused_gyro_weight * gyro_correction) \
                            + (self.fused_distance_weight * distance_error)

        # Deadband to avoid small corrections
        if abs(fused_error) < 0.1:
            return None

        logger.info("Fused Error: %.2f", fused_error)

        # Use shared PID logic
        turn = self.pid.calculate(fused_error)
        self.log_walk_data(distance_error, gyro_error=gyro_correction,
                           fused_error=fused_error, turn=turn)
        return turn

    def walk_func(self, left_distance: float, right_distance: float,
                               current_angle: float) -> Optional[float]:
        """Calculate the steering angle using sensor fusion PID control."""
        # Gyro correction
        delta_angle = current_angle - self.def_turn_angle
        gyro_correction = self.kgyro * delta_angle

        # Distance error calculation
        left_delta = left_distance - self.def_distance_left
        right_delta = right_distance - self.def_distance_right

        # Validate distances, if one distance is bad or missing.
        # overcorrect on the other side.
        if left_distance >= self.max_left_distance or left_distance <= 0:
            left_delta = 0.0
            right_delta=right_delta*2
        if right_distance >= self.max_right_distance or right_distance <= 0:
            right_delta = 0.0
            left_delta=left_delta*2

        # Control error: positive means steer right
        distance_error = left_delta - right_delta

        if left_distance <= self.min_left:
            #close to wall, make delta negative
            distance_error -=10
        elif right_distance <= self.min_right:
            #close to right wall, make delta more postive
            distance_error += 10

        # Deadband to avoid small corrections
        if abs(distance_error) < DELTA_DISTANCE_CM and \
                                abs(delta_angle) < MIN_GYRO_DELTA:
            self.pid.reset()
            return None

        return self.process_error(distance_error, gyro_correction)

class GyroWalkerwithMinDistanceHelper(EquiWalkerHelper):
    """Helper class for Gyro Walker logic with distance."""

    def __init__(self,  kp: float = -4.0, ki: float = 0.0, kd: float = -0.05,
                 kgyro: float = -5.0,
                 def_turn_angle: float = 0.0, min_left: float = -1, min_right: float = -1,
                 fused_distance_weight: float = 0.6, fused_gyro_weight: float = 0.4,
                 hardware: Optional[HardwareInterface]=None
                 ) -> None:
        # Call base class __init__ with default values for required parameters
        super().__init__(
            def_distance_left=0.0,
            def_distance_right=0.0,
            max_left_distance=200.0,
            max_right_distance=200.0,
            kp=kp,
            ki=ki,
            kd=kd,
            kgyro=kgyro,
            def_turn_angle=def_turn_angle,
            fused_distance_weight=fused_distance_weight,
            fused_gyro_weight=fused_gyro_weight,
            hardware=hardware,
            min_left=min_left,
            min_right=min_right
        )


    def walk_func(self, left_distance: float, right_distance: float,
                               current_angle: float) -> Optional[float]:
        """Calculate the steering angle using sensor fusion PID control."""
        # Gyro correction
        delta_angle = current_angle - self.def_turn_angle

        gyro_correction = self.kgyro * delta_angle

        # Distance correction
        distance_error = 0.0
        if self.min_left != -1 and left_distance < self.min_left:
            distance_error += self.min_left - left_distance
        if self.min_right != -1 and right_distance < self.min_right:
            distance_error += self.min_right - right_distance

        # Deadband to avoid small corrections
        if abs(distance_error) < DELTA_DISTANCE_CM and \
                                abs(delta_angle) < MIN_GYRO_DELTA:
            self.pid.reset()
            return None

        return self.process_error(distance_error, gyro_correction)

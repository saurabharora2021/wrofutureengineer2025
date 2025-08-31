"""Helper functions for Walker logic in WRO2025."""
import logging
from typing import Optional,List
import time
from abc import ABC
from round1.utilityfunctions import clamp_angle
from utils import constants

logger = logging.getLogger(__name__)


MAX_ANGLE = 30
MIN_GYRO_DELTA = 0.05 # Minimum gyro delta angle in degrees
DELTA_DISTANCE_CM = 0.5
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

        return clamp_angle(output, MAX_ANGLE)

class EquiWalkerHelper(ABC):
    """Helper class for equidistance walking with PID control."""

    #TODO: fix gyro correction since yaw is positive to right.
    def __init__(self, def_distance_left: float, def_distance_right: float,
                 max_left_distance: float= constants.LEFT_DISTANCE_MAX,
                 max_right_distance: float= constants.RIGHT_DISTANCE_MAX,
                 kp: float = -4.0, ki: float = 0.0, kd: float = -0.05,
                 kgyro: float = 5.5, def_turn_angle: float = 0.0,
                 fused_distance_weight: float = 0.5, fused_gyro_weight: float = 0.5,
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
        logger.info("EquiWalkerHelper initialized ...")
        logger.info("EquiWalkerHelper distances: left=%.2f, right=%.2f", def_distance_left,
                     def_distance_right)
        logger.info("EquiWalkerHelper parameters: kgyro=%.2f, def_turn_angle=%.2f,"\
                    +"fused_distance_weight=%.2f, fused_gyro_weight=%.2f",
                    kgyro, def_turn_angle, fused_distance_weight, fused_gyro_weight)
        self._messages:List[str] = []

    def get_log_data(self)->List[str]:
        """Get the log data."""
        return self._messages


    def log_walk_data(self,distance_error:float,gyro_error:float,fused_error:float,
                      turn:float) -> None:
        """Log walking data to the hardware interface."""
        message:List[str] = [
            f"Error D :{distance_error:.2f} G :{gyro_error:.2f}",
            f"Fuse: {fused_error:.2f} Tu:{turn:.2f}",
            ""
        ]
        logger.info("Walker: Error D :%.2f G :%.2f Fuse: %.2f Tu:%.2f",
                    distance_error, gyro_error, fused_error, turn)
        self._messages = message


    def process_error(self, distance_error: float, gyro_correction: float) -> Optional[float]:
        """Process the errors and return the steering angle."""
        # Sensor fusion: Combine gyro and distance errors
        fused_error = (self.fused_gyro_weight * gyro_correction) \
                            + (self.fused_distance_weight * distance_error)

        # Deadband to avoid small corrections
        if abs(fused_error) < 0.1:
            return None

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
        if left_distance >= self.max_left_distance or left_distance <= 0 or \
              self.def_distance_left == self.max_left_distance:
            left_delta = 0.0
            right_delta=right_delta*2
        if right_distance >= self.max_right_distance or right_distance <= 0 or \
              self.def_distance_right == self.max_right_distance:
            right_delta = 0.0
            left_delta=left_delta*2

        # Control error: positive means steer right
        distance_error = left_delta - right_delta

        if left_distance <= self.min_left and left_distance > 0:
            #close to wall, make delta negative
            distance_error -=5
        elif right_distance <= self.min_right and right_distance > 0:
            #close to right wall, make delta more postive
            distance_error += 5

        # Deadband to avoid small corrections
        if abs(distance_error) < DELTA_DISTANCE_CM and \
                                abs(delta_angle) < MIN_GYRO_DELTA:
            self.pid.reset()
            return None

        return self.process_error(distance_error, gyro_correction)

class GyroWalkerwithMinDistanceHelper(EquiWalkerHelper):
    """Helper class for Gyro Walker logic with distance."""

    def __init__(self,
                 max_left_distance: float = constants.LEFT_DISTANCE_MAX
                 , max_right_distance: float = constants.RIGHT_DISTANCE_MAX,
                 kp: float = -4.0, ki: float = 0.0, kd: float = -0.05,
                 kgyro: float = 5.5,
                 def_turn_angle: float = 0.0, min_left: float = -1, min_right: float = -1,
                 fused_distance_weight: float = 0.6, fused_gyro_weight: float = 0.4,
                 ) -> None:
        # Call base class __init__ with default values for required parameters
        logger.info("GyroWalkerwithMinDistanceHelper minleft %.2f , minright %.2f",
                    min_left, min_right)

        super().__init__(
            def_distance_left=0.0,
            def_distance_right=0.0,
            max_left_distance=max_left_distance,
            max_right_distance=max_right_distance,
            kp=kp,
            ki=ki,
            kd=kd,
            kgyro=kgyro,
            def_turn_angle=def_turn_angle,
            fused_distance_weight=fused_distance_weight,
            fused_gyro_weight=fused_gyro_weight,
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
class FixedTurnWalker(GyroWalkerwithMinDistanceHelper):
    """ This follows a fixed turning angle till min distance is violated."""

    def __init__(self,
                 max_left_distance: float, max_right_distance: float,
                 fixed_turn_angle: float,
                 kp: float = -4.0, ki: float = 0.0, kd: float = -0.05,
                 kgyro: float = -6.0,
                 def_turn_angle: float = 0.0, min_left: float = -1, min_right: float = -1,
                 fused_distance_weight: float = 0.6, fused_gyro_weight: float = 0.4,
                 ) -> None:
        # Call base class __init__ with default values for required parameters
        logger.info("FixedTurnWalker minleft %.2f , minright %.2f",
                    min_left, min_right)

        super().__init__(
            max_left_distance=max_left_distance,
            max_right_distance=max_right_distance,
            kp=kp,
            ki=ki,
            kd=kd,
            kgyro=kgyro,
            def_turn_angle=def_turn_angle,
            fused_distance_weight=fused_distance_weight,
            fused_gyro_weight=fused_gyro_weight,
            min_left=min_left,
            min_right=min_right
        )
        self.fixed_turn_angle = fixed_turn_angle

    def walk_func(self, left_distance: float, right_distance: float,
                               current_angle: float) -> Optional[float]:
        """Calculate the steering angle using sensor fusion PID control."""
        # Distance correction
        distance_error = 0.0
        if self.min_left != -1 and left_distance < self.min_left:
            distance_error += self.min_left - left_distance
        if self.min_right != -1 and right_distance < self.min_right:
            distance_error += self.min_right - right_distance

        # Gyro correction
        delta_angle = current_angle - self.def_turn_angle

        #small error we continue to turn as per plan
        if abs(distance_error) < DELTA_DISTANCE_CM:
            self.pid.reset()
            if abs(delta_angle) < 20:
                #lets reduce the steering angle
                return self.fixed_turn_angle/2
            return self.fixed_turn_angle

        gyro_correction = self.kgyro * delta_angle


        # Deadband to avoid small corrections
        if abs(delta_angle) < MIN_GYRO_DELTA:
            self.pid.reset()
            return None

        return self.process_error(distance_error, gyro_correction)


def main():
    """Main function for testing."""

    # Set up console logging
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s %(levelname)s %(name)s: %(message)s')
    console_handler.setFormatter(formatter)
    logging.getLogger().addHandler(console_handler)
    logging.getLogger().setLevel(logging.INFO)

    # Initialize walker helpers with example parameters
    equi_walker = EquiWalkerHelper(
        def_distance_left=40.0,
        def_distance_right=54.0,
        max_left_distance=200.0,
        max_right_distance=200.0,
        def_turn_angle=0.0,
    )
    #F:134.56, L:40.42, R:56.55, Y:5.50

    # Simulate sensor readings
    left_distance = 40.42
    right_distance = 56.55
    current_angle = 5.50

    print("EquiWalkerHelper turn:", equi_walker.walk_func(left_distance,
                                                           right_distance, current_angle))
    print(equi_walker.get_log_data())

if __name__ == "__main__":
    main()

""" This modules implements the Challenge 1 Walker for the WRO2025 Robot."""
from statistics import mean as average
from time import sleep
import logging
from typing import Tuple
from collections import deque
from base.mat import mat_color
from hardware.hardware_interface import HardwareInterface

logger = logging.getLogger(__name__)
class Walker:
    """This class implements the Challenge 1 Walker for the WRO2025 Robot."""
    # Constants for the walker
    CLOCKWISE_DIRECTION=1
    DEFAULT_SPEED=100
    ANTI_CLOCKWISE_DIRECTION=2
    FRONTDISTANCE_FOR_COLOR_CHECK=120
    WALLFRONTDISTANCE=30
    WALLSIDEDISTANCE=15
    EQUIWALKMAXDELTA=15
    UNKNOWN_DIRECTION=-1
    TURNRIGHT_ANGLE=10
    TURNLEFT_ANGLE=-10
    WALLFRONTENDDISTANCE=10
    D_TARGET = 15  # Desired distance from the wall
    KP = 2.0  # Proportional gain for the controller
    MAX_ANGLE = 10
    DELTA_DISTANCE_CM = 1.5

    output_inf: HardwareInterface
    direction=UNKNOWN_DIRECTION
    corner=False

    def __init__(self, output_inf:HardwareInterface):
        self.output_inf = output_inf
        self._queue: deque[float] = deque(maxlen=3)  # Initialize a deque to store angles.

    def wallunknowndirectioninit(self)-> Tuple[float, callable, bool]:
        """This method is used to initialize the walker when the direction is unknown."""
        logger.info("Initializing walker with unknown direction.")
        left_distance = self.output_inf.get_left_distance()
        right_distance = self.output_inf.get_right_distance()
        self.output_inf.logdistances()  # Log the distances
        if left_distance < right_distance:
            distance = left_distance
            walk_function = self.output_inf.get_left_distance
            isleft = True
        else:
            distance = right_distance
            walk_function = self.output_inf.get_right_distance
            isleft = False
        return (distance, walk_function, isleft)

    def equidistance_walk_init(self) -> Tuple[float, float]:
        """Initialize the equidistance walk with default distances."""
        logger.info("Initializing equidistance walk.")
        left_distance = self.output_inf.get_left_distance()
        right_distance = self.output_inf.get_right_distance()
        self.output_inf.logdistances()  # Log the distances
        self._queue.clear()  # Clear the queue for new distances

        return (left_distance, right_distance)

    def equidistance_walk(self, def_distance_left: float,
                           def_distance_right: float,kp:float = -1) -> Tuple[float, float]:
        """Walk in a straight line, maintaining equal distance from both walls."""
        # logger.info("Starting equidistance walk.")
        errorcount = 0
        left_distance = self.output_inf.get_left_distance()
        right_distance = self.output_inf.get_right_distance()
        self.output_inf.logdistances()  # Log the distances

        #If you are at a point, where the left rail is not present or the right rail is not present,
        # then we will not adjust the steering.
        if left_distance >= self.output_inf.get_left_distance_max() or left_distance <= 0:
            logger.warning("Left distance is not set.")
            left_distance = def_distance_left
            errorcount += 1

        if right_distance >= self.output_inf.get_right_distance_max() or right_distance <= 0 :
            logger.warning("Right distance is not set.")
            right_distance = def_distance_right
            errorcount += 1

        #if the left or right distance is less than 10 cm , we need to move to steer away.
        if left_distance < 10:
            logger.warning("Left distance is less than 10 cm, steering away.")
            right_delta = 10 + (10 - left_distance)*2
            left_delta = 0
        elif right_distance < 10:
            logger.warning("Right distance is less than 10 cm, steering away.")
            left_delta = 10 + (10 - right_distance)*2
            right_delta = 0
        else:
            left_delta = abs(left_distance - def_distance_left)
            right_delta = abs(right_distance - def_distance_right)

        if (abs(left_delta) > self.DELTA_DISTANCE_CM
            or abs(right_delta) > self.DELTA_DISTANCE_CM) and errorcount < 2:

            logger.warning("Left Delta: %.2f, Right Delta: %.2f",
                                left_delta, right_delta)
            #Handle sudden changes in distance
            if abs(left_delta) > self.EQUIWALKMAXDELTA:
                logger.warning("Left distance change is too high: %.2f", left_delta)
                left_delta = self.EQUIWALKMAXDELTA* (left_delta / abs(left_delta))
            if abs(right_delta) > self.EQUIWALKMAXDELTA:
                logger.warning("Right distance change is too high: %.2f", right_delta)
                right_delta = self.EQUIWALKMAXDELTA * (right_delta / abs(right_delta))

            # Adjust steering based on the difference in distances
            error = left_delta - right_delta
            angle = self.clamp(kp * error, -1*self.MAX_ANGLE, self.MAX_ANGLE)
            logger.warning("angle: %.2f", angle)
            self._queue.append(angle)

            final_angle = average(self._queue)
            logger.warning("Final angle: %.2f", final_angle)

            self.output_inf.turn_steering(final_angle)

        return (left_distance, right_distance)


    def wall_follow_func(self,distance_func,isleft:bool,target_distance:float,
                         kp:float=1.0,speed=DEFAULT_SPEED/2):
        """Follow the wall based on the current direction."""
        dist = distance_func()
        error = target_distance - dist
        angle = self.clamp(kp * error, -1*self.MAX_ANGLE, self.MAX_ANGLE)

        #TODO: used turnleft or turnright based on direction
        if isleft is False:
            angle = -angle

        if abs(angle) > 4:
            self.output_inf.turn_steering(angle)

        self.output_inf.drive_forward(speed)

        logger.warning("Distance: %.2f, angle: %.2f", dist, angle)
        sleep(0.1)

    def clamp(self,val, min_val, max_val):
        """Clamp the value between min_val and max_val."""
        return max(min(val, max_val), min_val)

    def wall_follow(self):
        """Follow the wall based on the current direction."""
        dist = 0
        if self.direction == self.CLOCKWISE_DIRECTION:
            dist = self.output_inf.get_left_distance()
        elif self.direction == self.ANTI_CLOCKWISE_DIRECTION:
            dist = self.output_inf.get_right_distance()
        error = self.D_TARGET - dist
        angle = self.clamp(self.KP * error, -1*self.MAX_ANGLE, self.MAX_ANGLE)

        #TODO: used turnleft or turnright based on direction
        # steering.set_angle(angle)
        self.output_inf.drive_forward(self.DEFAULT_SPEED)

        logger.warning("Distance: %.2f, angle: %.2f", dist, angle)
        sleep(0.1)


    def start_walk(self,nooflaps:int=4):
        """Start the walk based on the current direction which is unknown and number of laps."""
        logger.info("Starting to walk...")
        logger.warning("Direction:%s", self.directiontostr(self.direction))

        corner_counter = 0
        # Implement the logic for starting a walk
        if self.direction == self.UNKNOWN_DIRECTION:
            logger.info("Direction is unknown, starting the walk with default distances.")
            default_left_distance, default_right_distance = self.equidistance_walk_init()
            logger.info("Default Left: %.2f, Right: %.2f",
                             default_left_distance, default_right_distance)

            #Lets start the walk until we reach the front distance,but at slow speed.
            self.output_inf.drive_forward(self.DEFAULT_SPEED/2)
            #TODO: Revisit if we need to run this loop or start checking color immediately.
            while (
                self.output_inf.get_front_distance() > self.FRONTDISTANCE_FOR_COLOR_CHECK
                or self.output_inf.get_front_distance() < 0
            ):
                current_left, current_right =self.equidistance_walk(default_left_distance,
                                                                     default_right_distance)

                self.output_inf.logdistances()  # Log the distances

                sleep(0.1)

            self.output_inf.drive_stop()
            self.output_inf.buzzer_beep()
            return
            # sleep(2)

            logger.info("Time to check color")
            color = self.wait_for_color(["blue", "orange"])

            default_left_distance, default_right_distance = self.equidistance_walk_init()


            while (color is None and
                   self.output_inf.get_front_distance() > self.WALLFRONTENDDISTANCE):

                current_left, current_right = self.equidistance_walk(default_left_distance,
                                                                     default_right_distance)
                logger.warning("Left Distance: %.2f, Right Distance: %.2f",
                                     current_left, current_right)
                logger.info("Front Distance:%s",self.output_inf.get_front_distance())
                color = self.check_bottom_color(["blue", "orange"])
                sleep(0.1)

            #Lets first stop the base and then check the color.
            self.output_inf.stop()

            if color is None:
                ##FixMe, can we do this in a better way, can we still walk?
                logger.warning("No color detected, stopping the walk.")
                self.output_inf.buzzer_beep()
                self.output_inf.led1_red()
                self.output_inf.force_flush_messages()
                return
            if color == "blue":
                self.direction = self.ANTI_CLOCKWISE_DIRECTION
            elif color == "orange":
                self.direction = self.CLOCKWISE_DIRECTION

            self.corner = True
            corner_counter= 1

            self.output_inf.buzzer_beep()
            logger.warning("color: %s", color)
            logger.warning("Direction: %s", self.directiontostr(self.direction))
            self.output_inf.stop()
            return

        totalcorners = nooflaps * 4  # Each turn is 90 degrees, so 4 turns make a full circle

        # We will walk until we reach the end of the section or complete the required number of
        while corner_counter < totalcorners:
            if self.corner:
                if self.direction == self.CLOCKWISE_DIRECTION:
                    self.handle_corner(self.TURNRIGHT_ANGLE, self.output_inf.get_right_distance)
                else:
                    self.handle_corner(self.TURNLEFT_ANGLE, self.output_inf.get_left_distance)
                corner_counter += 1
                if corner_counter >= totalcorners:
                    logger.info("Reached the end of the walk.")
                    self.output_inf.drive_stop()
                    return
            else:
                self.follow_wall_until(self.WALLFRONTENDDISTANCE)

    def directiontostr(self,direction):
        """Convert direction to string."""
        if direction == self.CLOCKWISE_DIRECTION:
            return "Clockwise"
        elif direction == self.ANTI_CLOCKWISE_DIRECTION:
            return "Anti-clockwise"
        else:
            return "Unknown"

    def handle_corner(self, turn_angle, side_distance_func):
        """Handle the corner turn based on the current direction."""
        self.output_inf.turn_steering(turn_angle)
        self.output_inf.drive_forward(self.DEFAULT_SPEED)
        while (self.output_inf.get_front_distance() > self.WALLFRONTDISTANCE
                or side_distance_func() > self.WALLSIDEDISTANCE):
            sleep(0.1)
        self.output_inf.turn_steering(-turn_angle)
        self.corner = False

    def follow_wall_until(self, distance):
        """Follow the wall until the front distance is less than the specified distance."""
        while self.output_inf.get_front_distance() > distance:
            self.wall_follow()
            sleep(0.1)
        self.corner = True

    def wait_for_color(self, colors):
        """Wait for the robot to detect one of the specified colors."""
        distance, distance_func, isleft = self.wallunknowndirectioninit()
        logger.info("Base distance: %s", distance)

        r, g, b, _ = self.output_inf.get_bottom_color_rgbi()
        color = mat_color(r, g, b)
        logger.info("Waiting for color: %s, current color: %s", colors, color)
        while color not in colors:
            self.output_inf.logdistances()  # Log the distances
            self.wall_follow_func(distance_func,isleft,distance,speed=self.DEFAULT_SPEED/5)
            sleep(0.1)
            r, g, b, _ = self.output_inf.get_bottom_color_rgbi()
            color = mat_color(r, g, b)
            logger.info("Current rgb: R=%d, G=%d, B=%d", r, g, b)
            logger.info("Waiting for color: %s, current color: %s", colors, color)
        logger.info("Detected color: %s", color)
        self.output_inf.drive_stop()
        self.output_inf.buzzer_beep()
        sleep(1)
        self.output_inf.force_flush_messages()
        return color

    def check_bottom_color(self,colors):
        """Read the bottom color and return it if it matches the specified colors."""
        r, g, b, _ = self.output_inf.get_bottom_color_rgbi()
        color = mat_color(r, g, b)
        logger.info("Current rgb: R=%d, G=%d, B=%d", r, g, b)
        logger.info("Waiting for color: %s, current color: %s", colors, color)
        if color in colors:
            logger.info("Detected color: %s", color)
            return color
        else:
            logger.info("Color not detected, current color: %s", color)
            return None

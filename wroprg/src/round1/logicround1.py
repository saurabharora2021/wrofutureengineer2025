""" This modules implements the Challenge 1 Walker for the WRO2025 Robot."""
from time import sleep
import logging
from typing import Tuple
from base.mat import mat_color
from hardware.hardware_interface import HardwareInterface
from round1.walker_helpers import EquiWalkerHelper

logger = logging.getLogger(__name__)
class Walker:
    """This class implements the Challenge 1 Walker for the WRO2025 Robot."""
    # Constants for the walker
    CLOCKWISE_DIRECTION=1
    DEFAULT_SPEED=100/3
    ANTI_CLOCKWISE_DIRECTION=2
    FRONTDISTANCE_FOR_COLOR_CHECK=120
    WALLFRONTDISTANCE=30
    WALLSIDEDISTANCE=15

    UNKNOWN_DIRECTION=-1
    TURNRIGHT_ANGLE=10
    TURNLEFT_ANGLE=-10
    WALLFRONTENDDISTANCE=10
    D_TARGET = 15  # Desired distance from the wall
    KP = 2.0  # Proportional gain for the controller
    MAX_ANGLE = 10

    output_inf: HardwareInterface
    direction=UNKNOWN_DIRECTION
    corner=False

    def __init__(self, output_inf:HardwareInterface):
        self.output_inf = output_inf

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
            left_distance = self.output_inf.get_left_distance()
            right_distance = self.output_inf.get_right_distance()
            left_distance_max = self.output_inf.get_left_distance_max()
            right_distance_max = self.output_inf.get_right_distance_max()
            default_x_angle = self.output_inf.get_default_x_angle()

            helper:EquiWalkerHelper = EquiWalkerHelper(
                def_distance_left=left_distance,
                def_distance_right=right_distance,
                max_left_distance=left_distance_max,
                max_right_distance=right_distance_max,
                current_angle=default_x_angle
            )



            #Lets start the walk until we reach the front distance,but at slow speed.
            self.output_inf.drive_forward(self.DEFAULT_SPEED)
            #TODO: Revisit if we need to run this loop or start checking color immediately.
            while self.output_inf.get_front_distance() > self.FRONTDISTANCE_FOR_COLOR_CHECK:

                left_distance = self.output_inf.get_left_distance()
                right_distance = self.output_inf.get_right_distance()
                self.output_inf.logdistances()  # Log the distances
                gyro = self.output_inf.get_gyro()
                current_angle = gyro[0]  # Assuming gyro[0] gives the current angle

                turn_angle = helper.equidistance_walk_func(
                                    left_distance, right_distance, current_angle)

                if turn_angle is not None:
                    if turn_angle >= 0:
                        logger.info("Turning right to angle: %.2f", turn_angle)
                        if turn_angle > 10:
                            #lets stop and turn first
                            self.output_inf.drive_stop()
                            self.output_inf.turn_steering(turn_angle)
                            self.output_inf.drive_forward(self.DEFAULT_SPEED)
                    else:
                        logger.info("Turning left to angle: %.2f", turn_angle)
                    # Turn the steering based on the calculated angle
                    self.output_inf.turn_steering(turn_angle)

                sleep(0.05)

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

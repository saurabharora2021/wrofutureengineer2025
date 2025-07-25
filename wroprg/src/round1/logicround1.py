""" This modules implements the Challenge 1 Walker for the WRO2025 Robot."""
from time import sleep
import logging
import math
from typing import Tuple
from hardware.rpi_interface  import RpiInterface
from hardware.legodriver import BuildHatDriveBase

class Walker:
    """This class implements the Challenge 1 Walker for the WRO2025 Robot."""

    logger = logging.getLogger(__name__)


    CLOCKWISE_DIRECTION=1
    DEFAULT_SPEED=100
    ANTI_CLOCKWISE_DIRECTION=2
    FRONTDISTANCE_FOR_COLOR_CHECK=100
    WALLFRONTDISTANCE=30
    WALLSIDEDISTANCE=15
    UNKNOWN_DIRECTION=-1
    TURNRIGHT_ANGLE=10
    TURNLEFT_ANGLE=-10
    WALLFRONTENDDISTANCE=10
    D_TARGET = 15  # Desired distance from the wall
    KP = 2.0  # Proportional gain for the controller
    MAX_ANGLE = 10
    DELTA_DISTANCE_CM = 1.5

    drivebase:BuildHatDriveBase
    output_inf: RpiInterface
    direction=UNKNOWN_DIRECTION
    corner=False

    def __init__(self, drivebase:BuildHatDriveBase, output_inf:RpiInterface):
        self.drivebase = drivebase
        self.output_inf = output_inf

    def wallunknowndirectioninit(self)-> Tuple[float, callable, bool]:
        """This method is used to initialize the walker when the direction is unknown."""
        self.logger.info("Initializing walker with unknown direction.")
        left_distance = self.output_inf.get_left_distance()
        right_distance = self.output_inf.get_right_distance()
        self.logger.warning("Left Distance: %.2f, Right Distance: %.2f",
                            left_distance, right_distance)
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
        self.logger.info("Initializing equidistance walk.")
        left_distance = self.output_inf.get_left_distance()
        right_distance = self.output_inf.get_right_distance()
        self.logger.warning("Left Distance: %.2f, Right Distance: %.2f",
                            left_distance, right_distance)
        return (left_distance, right_distance)

    def equidistance_walk(self, def_distance_left: float,
                           def_distance_right: float,kp:float = 1) -> Tuple[float, float]:
        """Walk in a straight line, maintaining equal distance from both walls."""
        self.logger.info("Starting equidistance walk.")

        left_distance = self.output_inf.get_left_distance()
        right_distance = self.output_inf.get_right_distance()
        self.logger.warning("Left Distance: %.2f, Right Distance: %.2f",
                                 left_distance, right_distance)

        #If you are at a point, where the left rail is not present or the right rail is not present,
        # then we will not adjust the steering.
        if left_distance >= self.output_inf.get_left_distance_max():
            self.logger.warning("Left distance sensor is at maximum distance.")
            left_distance = def_distance_left

        if right_distance >= self.output_inf.get_right_distance_max():
            self.logger.warning("Right distance sensor is at maximum distance.")
            right_distance = def_distance_right

        if (abs(left_distance - def_distance_left)> self.DELTA_DISTANCE_CM
            or abs(right_distance < def_distance_right)> self.DELTA_DISTANCE_CM):
            # Adjust steering based on the difference in distances
            error = (left_distance - def_distance_left) - (right_distance - def_distance_right)
            angle = self.clamp(kp * error, -1*self.MAX_ANGLE, self.MAX_ANGLE)
            self.logger.warning("steering angle: %.2f", angle)
            self.drivebase.turn_steering(angle)

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
            self.drivebase.turn_steering(angle)

        self.drivebase.run_front(speed)

        self.logger.warning("Distance: %.2f, angle: %.2f", dist, angle)
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
        self.drivebase.run_front(self.DEFAULT_SPEED)

        self.logger.warning("Distance: %.2f, angle: %.2f", dist, angle)
        sleep(0.1)

    #based on color.py from buildhat
    def mat_color(self, r, g, b)-> str:
        """Return the color name from RGB

        :param r: Red
        :param g: Green
        :param b: Blue
        :return: Name of the color as a string
        :rtype: str
        """
        table = [("black", (0, 0, 0)),
                 ("orange", (145, 85, 82)), #r to 145 , g to 85, b to 82 based on test color.
                 ("blue", (0, 51, 77)),  #changed blue to 77 based on test color.
                 ("white", (255, 255, 255)),
                 ("line", (179, 179, 179))
                 ]
        near = ""
        euc = math.inf
        for itm in table:
            cur = math.sqrt((r - itm[1][0])**2 + (g - itm[1][1])**2 + (b - itm[1][2])**2)
            if cur < euc:
                near = itm[0]
                euc = cur
        return near
    def start_walk(self,nooflaps:int=4):
        """Start the walk based on the current direction which is unknown and number of laps."""
        self.logger.info("Starting to walk...")
        self.logger.warning("Direction:%s", self.directiontostr(self.direction))

        corner_counter = 0
        # Implement the logic for starting a walk
        if self.direction == self.UNKNOWN_DIRECTION:
            self.logger.info("Front Distance:%s",self.drivebase.get_front_distance())

            default_left_distance, default_right_distance = self.equidistance_walk_init()
            self.logger.info("Default Left Distance: %.2f, Default Right Distance: %.2f",
                             default_left_distance, default_right_distance)

            #Lets start the walk until we reach the front distance,but at slow speed.
            self.drivebase.run_front(self.DEFAULT_SPEED/2)
            #TODO: Revisit if we need to run this loop or start checking color immediately.
            while (
                self.drivebase.get_front_distance() > self.FRONTDISTANCE_FOR_COLOR_CHECK
                or self.drivebase.get_front_distance() < 0
            ):
                current_left, current_right =self.equidistance_walk(default_left_distance,
                                                                     default_right_distance)

                self.logger.warning("Left Distance: %.2f, Right Distance: %.2f",
                                     current_left, current_right)
                self.logger.info("Front Distance:%s",self.drivebase.get_front_distance())
                sleep(0.1)

            self.drivebase.stop()
            self.output_inf.buzzer_beep()
            return
            # sleep(2)

            self.logger.info("Time to check color")
            color = self.wait_for_color(["blue", "orange"])

            default_left_distance, default_right_distance = self.equidistance_walk_init()


            while (color is None and
                   self.drivebase.get_front_distance() > self.WALLFRONTENDDISTANCE):

                current_left, current_right = self.equidistance_walk(default_left_distance,
                                                                     default_right_distance)
                self.logger.warning("Left Distance: %.2f, Right Distance: %.2f",
                                     current_left, current_right)
                self.logger.info("Front Distance:%s",self.drivebase.get_front_distance())
                color = self.check_bottom_color(["blue", "orange"])
                sleep(0.1)

            #Lets first stop the base and then check the color.
            self.drivebase.stop()

            if color is None:
                ##FixMe, can we do this in a better way, can we still walk?
                self.logger.warning("No color detected, stopping the walk.")
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
            self.logger.warning("color: %s", color)
            self.logger.warning("Direction: %s", self.directiontostr(self.direction))
            self.drivebase.stop()
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
                    self.logger.info("Reached the end of the walk.")
                    self.drivebase.back_motor.stop()
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
        self.drivebase.turn_steering(turn_angle)
        self.drivebase.run_front(self.DEFAULT_SPEED)
        while (self.drivebase.get_front_distance() > self.WALLFRONTDISTANCE
                or side_distance_func() > self.WALLSIDEDISTANCE):
            sleep(0.1)
        self.drivebase.turn_steering(-turn_angle)
        self.corner = False

    def follow_wall_until(self, distance):
        """Follow the wall until the front distance is less than the specified distance."""
        while self.drivebase.get_front_distance() > distance:
            self.wall_follow()
            sleep(0.1)
        self.corner = True

    def wait_for_color(self, colors):
        """Wait for the robot to detect one of the specified colors."""
        distance, distance_func, isleft = self.wallunknowndirectioninit()
        self.logger.info("Base distance: %s", distance)

        r, g, b, _ = self.drivebase.get_bottom_color_rgbi()
        color = self.mat_color(r, g, b)
        self.logger.info("Waiting for color: %s, current color: %s", colors, color)
        while color not in colors:
            left_distance = self.output_inf.get_left_distance()
            right_distance = self.output_inf.get_right_distance()
            self.logger.warning("Left Distance: %.2f, Right Distance: %.2f",
                                     left_distance, right_distance)
            self.wall_follow_func(distance_func,isleft,distance,speed=self.DEFAULT_SPEED/5)
            self.logger.info("Front Distance:%s",self.drivebase.get_front_distance())
            sleep(0.1)
            r, g, b, _ = self.drivebase.get_bottom_color_rgbi()
            color = self.mat_color(r, g, b)
            self.logger.info("Current rgb: R=%d, G=%d, B=%d", r, g, b)
            self.logger.info("Waiting for color: %s, current color: %s", colors, color)
        self.logger.info("Detected color: %s", color)
        self.drivebase.stop()
        self.output_inf.buzzer_beep()
        sleep(1)
        self.output_inf.force_flush_messages()
        return color

    def check_bottom_color(self,colors):
        """Read the bottom color and return it if it matches the specified colors."""
        r, g, b, _ = self.drivebase.get_bottom_color_rgbi()
        color = self.mat_color(r, g, b)
        self.logger.info("Current rgb: R=%d, G=%d, B=%d", r, g, b)
        self.logger.info("Waiting for color: %s, current color: %s", colors, color)
        if color in colors:
            self.logger.info("Detected color: %s", color)
            return color
        else:
            self.logger.info("Color not detected, current color: %s", color)
            return None

""" This modules implements the Challenge 1 Walker for the WRO2025 Robot."""
from time import sleep
import logging
from collections import Counter
from base.mat import mat_color
from hardware.hardware_interface import HardwareInterface
from round1.walker_helpers import EquiWalkerHelper, GyroWalkerHelper

logger = logging.getLogger(__name__)
class Walker:
    """This class implements the Challenge 1 Walker for the WRO2025 Robot."""
    # Constants for the walker
    CLOCKWISE_DIRECTION=1
    DEFAULT_SPEED=100/2
    WALK_TO_COLOR_SPEED= 10
    ANTI_CLOCKWISE_DIRECTION=2
    FRONTDISTANCE_FOR_COLOR_CHECK=120
    WALLFRONTDISTANCE=30
    WALLSIDEDISTANCE=15

    UNKNOWN_DIRECTION=-1
    TURNRIGHT_ANGLE=10
    TURNLEFT_ANGLE=-10
    WALLFRONTENDDISTANCE=30
    D_TARGET = 15  # Desired distance from the wall
    KP = 2.0  # Proportional gain for the controller
    MAX_ANGLE = 10

    output_inf: HardwareInterface
    direction=UNKNOWN_DIRECTION
    corner=False

    def __init__(self, output_inf:HardwareInterface):
        self.output_inf = output_inf
        self._path = list()  # For storing distances

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

        self.output_inf.drive_forward(self.DEFAULT_SPEED)

        logger.warning("Distance: %.2f, angle: %.2f", dist, angle)
        sleep(0.1)

    def equidistance_walk_start(self,use_mpu:bool,kp:float=0) -> EquiWalkerHelper:
        """Initialize the EquiWalkerHelper with default distances and angles."""
        if use_mpu:
            self.output_inf.reset_yaw()  # Reset yaw to zero

        left_distance = self.output_inf.get_left_distance()
        right_distance = self.output_inf.get_right_distance()
        left_distance_max = self.output_inf.get_left_distance_max()
        right_distance_max = self.output_inf.get_right_distance_max()
        self._path.clear()  # Clear the path for new distances

        helper:EquiWalkerHelper = EquiWalkerHelper(
            def_distance_left=left_distance,
            def_distance_right=right_distance,
            max_left_distance=left_distance_max,
            max_right_distance=right_distance_max,
            kp=kp
        )
        return helper

    def equidistance_walk(self,helper:EquiWalkerHelper,use_mpu:bool,current_steering)->float:
        """Walk using the equidistance method."""

        # Log the distances
        (front_distance,left_distance,right_distance) = self.output_inf.logdistances()

        self._path.append((front_distance,left_distance,right_distance))

        current_angle = 0
        if use_mpu:
            _, _, yaw = self.output_inf.get_orientation()
            current_angle = yaw # Assuming yaw gives the current angle.

        turn_angle = helper.equidistance_walk_func(
                            left_distance, right_distance, current_angle,current_steering)

        if turn_angle is not None:
            if turn_angle >= 0:
                logger.info("Turning right to angle: %.2f", turn_angle)
            else:
                logger.info("Turning left to angle: %.2f", turn_angle)
            # Turn the steering based on the calculated angle
            self.output_inf.turn_steering(turn_angle)
        return turn_angle


    def start_walk(self,nooflaps:int=4):
        """Start the walk based on the current direction which is unknown and number of laps."""
        logger.info("Starting to walk...")
        logger.warning("Direction:%s", self.directiontostr(self.direction))

        corner_counter = 0
        # Implement the logic for starting a walk
        if self.direction == self.UNKNOWN_DIRECTION:
            logger.info("Direction is unknown, starting the walk with default distances.")

            helper:EquiWalkerHelper = self.equidistance_walk_start(use_mpu=True)

            #Lets start the walk until we reach the front distance,but at slow speed.
            self.output_inf.drive_forward(self.DEFAULT_SPEED)

            while self.output_inf.get_front_distance() > self.FRONTDISTANCE_FOR_COLOR_CHECK:

                self.equidistance_walk(helper,use_mpu=True,current_steering=self.output_inf
                                       .get_steering_angle())
                sleep(0.03)

            self.output_inf.drive_stop()
            self.output_inf.buzzer_beep()

            logger.info("Time to check color")
            sleep(0.5)

            # helper:EquiWalkerHelper = self.equidistance_walk_start(use_mpu,kp=-1.5)
            knowncolor = ["blue", "orange"]
            color = self.check_bottom_color(knowncolor)

            gyrohelper:GyroWalkerHelper = GyroWalkerHelper()
            self.output_inf.reset_yaw()  # Reset yaw to zero

            running = False

            while (color is None and
                   self.output_inf.get_front_distance() > self.WALLFRONTENDDISTANCE):

                color = self.check_bottom_color(knowncolor)
                if color is None:
                    _, _, yaw = self.output_inf.get_orientation()
                    turn_angle = gyrohelper.walk_func(yaw,self.output_inf.
                                                                get_steering_angle())

                    if turn_angle is not None:
                        if turn_angle >= 0:
                            logger.info("Turning right to angle: %.2f", turn_angle)
                        else:
                            logger.info("Turning left to angle: %.2f", turn_angle)
                        # Turn the steering based on the calculated angle
                        self.output_inf.turn_steering(turn_angle)

                    logger.info("Front Distance:%s",self.output_inf.get_front_distance())
                    color = self.check_bottom_color(knowncolor)
                if not running:
                    logger.info("color checking walk...")
                    self.output_inf.drive_forward(self.WALK_TO_COLOR_SPEED)
                    running = True
                    # self.output_inf.buzzer_beep(timer=0.1)
                    # sleep(0.001)

            #Lets first stop the base and then check the color.
            self.output_inf.drive_stop()
            self.output_inf.buzzer_beep()
            logger.info("Front Distance:%s",self.output_inf.get_front_distance())

            color2 = self.check_bottom_color(knowncolor)

            #can we check.if one of the sides is not present?, that means we are at a corner.
            # and based on which side we can determine the direction.
            (_,left,right) = self.output_inf.logdistances()

            MAX_DISTANCE = 100
            if (left > right):
                logger.info("Left side is present, right side is not present," \
                "                                            setting direction to clockwise.")
                direction_hints = self.CLOCKWISE_DIRECTION
            elif (right < left):
                logger.info("Right side is present, left side is not present, " \
                "                                       setting direction to anti-clockwise.")
                direction_hints = self.ANTI_CLOCKWISE_DIRECTION
            else:
                direction_hints = self.UNKNOWN_DIRECTION


            if color is None:
                ##FixMe, can we do this in a better way, can we still walk?
                logger.warning("No color detected, stopping the walk.")
                self.output_inf.buzzer_beep()
                self.output_inf.led1_red()
                self.output_inf.force_flush_messages()
                return
            self.direction = self._color_to_direction(color)


            if (self.direction == direction_hints and self.direction== self.UNKNOWN_DIRECTION):
                #we have not been able to determine the direction.
                #Lets walk upto 25cm front, till you seen side distances as max.
                logger.warning("Direction is unknown, walking until side distances are max.")
                self._handle_no_direction_walk(use_mpu=False,helper=helper)
            elif (self.direction != direction_hints and self.direction != self.UNKNOWN_DIRECTION):
                #we have been able to determine the direction.
                # But hints and color don't match, lets look at color2 if it helps.
                tmp_direction = self._color_to_direction(color2)
                #Majority voting of directions.
                direction_list = [self.direction, tmp_direction, direction_hints]
                direction_counter = Counter(direction_list)

                logger.info("Direction Freq: %s", direction_counter)
                if direction_counter[self.direction] >= 2:
                    logger.warning("Direction is %s, setting direction to %s",
                                   self.directiontostr(self.direction), self.direction)
                elif direction_counter[direction_hints] >= 2:
                    logger.warning("Direction is %s, setting direction to %s",
                                   self.directiontostr(direction_hints), direction_hints)
                    self.direction = direction_hints
                else:
                    logger.warning("Direction is unknown, setting direction to %s",
                                   self.directiontostr(self.UNKNOWN_DIRECTION))
                    self.direction = self.UNKNOWN_DIRECTION
                    self._handle_no_direction_walk(use_mpu=False,helper=helper)

            self.corner = True
            corner_counter= 1

            self.output_inf.buzzer_beep()
            logger.warning("color: %s", color)
            self.output_inf.drive_stop()
            self.output_inf.force_flush_messages()

            return

            #TODO: we cannot determin direction, beep and stop.
            if (self.direction == self.UNKNOWN_DIRECTION):
                logger.warning("Direction is unknown, stopping the walk.")
                self.output_inf.buzzer_beep()
                self.output_inf.led1_red()
                self.output_inf.force_flush_messages()
                return

        totalcorners = nooflaps * 4  # Each turn is 90 degrees, so 4 turns make a full circle

        # We will walk until we reach the end of the section or complete the required number of
        while corner_counter < totalcorners:
            if self.corner:
                helper:EquiWalkerHelper = self.handle_corner_start(use_mpu=False,
                                                                    direction=self.direction)

                if self.direction == self.CLOCKWISE_DIRECTION:
                    turn_angle = self.TURNRIGHT_ANGLE
                else:
                    turn_angle = self.TURNLEFT_ANGLE

                self.output_inf.turn_steering(turn_angle)
                self.output_inf.drive_forward(self.DEFAULT_SPEED)

                while (self.output_inf.get_front_distance() > self.WALLFRONTDISTANCE and
                       self.output_inf.get_front_distance()
                            < self.output_inf.get_front_distance_max()):
                    self.handle_corner_walk(use_mpu=False,direction=self.direction,helper=helper)
                    sleep(0.1)
                ##Turned the corner, lets stop the base.
                self.output_inf.drive_stop()
                self.corner= False
                return

                corner_counter += 1
                if corner_counter >= totalcorners:
                    logger.info("Reached the end of the walk.")
                    self.output_inf.drive_stop()
                    return
            else:
                self.follow_wall_until(self.WALLFRONTENDDISTANCE)

    def _handle_no_direction_walk(self,use_mpu:bool,helper:EquiWalkerHelper):
        #TODO: Implement the logic to handle the case when direction is unknown.
        self.output_inf.buzzer_beep()
        self.output_inf.led1_red()
        return

    def _color_to_direction(self,color)-> int:
        if color == "blue":
            return self.ANTI_CLOCKWISE_DIRECTION
        elif color == "orange":
            return self.CLOCKWISE_DIRECTION
        else:
            return self.UNKNOWN_DIRECTION


    def directiontostr(self,direction):
        """Convert direction to string."""
        if direction == self.CLOCKWISE_DIRECTION:
            return "Clockwise"
        elif direction == self.ANTI_CLOCKWISE_DIRECTION:
            return "Anti-clockwise"
        else:
            return "Unknown"

    def handle_corner_walk(self,use_mpu:bool,direction:int,helper:EquiWalkerHelper,) -> float:
        """Walk using the equidistance method for corner handling"""
        left_distance = self.output_inf.get_left_distance()
        right_distance = self.output_inf.get_right_distance()

        if direction == self.CLOCKWISE_DIRECTION:
            logger.info("Clockwise direction, using right wall, set other to max")
            left_distance = self.output_inf.get_left_distance_max()
        elif direction == self.ANTI_CLOCKWISE_DIRECTION:
            logger.info("Anti-clockwise direction, using left wall, set other to max")
            right_distance = self.output_inf.get_right_distance_max()

        self.output_inf.logdistances()  # Log the distances
        current_angle = 0
        if use_mpu:
            gyro = self.output_inf.get_gyro()
            current_angle = gyro[0]  # Assuming gyro[0] gives the current angle

        turn_angle = helper.equidistance_walk_func(
                            left_distance, right_distance, current_angle,
                            current_steering_angle = self.output_inf.get_steering_angle())

        if turn_angle is not None:
            if turn_angle >= 0:
                logger.info("Turning right to angle: %.2f", turn_angle)
            else:
                logger.info("Turning left to angle: %.2f", turn_angle)
            # Turn the steering based on the calculated angle
            self.output_inf.turn_steering(turn_angle)
        return turn_angle


    def handle_corner_start(self,use_mpu:bool,direction:int,kp:float=0) -> EquiWalkerHelper:
        """Initialize the EquiWalkerHelper with default distances and angles."""
        if use_mpu:
            self.output_inf.reset_yaw()  # Reset yaw to zero

        left_distance = 0
        right_distance = 0

        if direction == self.CLOCKWISE_DIRECTION:
            logger.info("Clockwise direction, using right wall, set other to max")
            left_distance = self.output_inf.get_left_distance_max()
            right_distance = self.output_inf.get_right_distance()
        elif direction == self.ANTI_CLOCKWISE_DIRECTION:
            logger.info("Anti-clockwise direction, using left wall, set other to max")
            right_distance = self.output_inf.get_right_distance_max()
            left_distance = self.output_inf.get_left_distance()

        left_distance_max = self.output_inf.get_left_distance_max()
        right_distance_max = self.output_inf.get_right_distance_max()

        helper:EquiWalkerHelper = EquiWalkerHelper(
            def_distance_left=left_distance,
            def_distance_right=right_distance,
            max_left_distance=left_distance_max,
            max_right_distance=right_distance_max,
            kp=kp
        )
        return helper

    def follow_wall_until(self, distance):
        """Follow the wall until the front distance is less than the specified distance."""
        while self.output_inf.get_front_distance() > distance:
            self.wall_follow()
            sleep(0.1)
        self.corner = True

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

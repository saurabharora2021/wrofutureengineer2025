""" This modules implements the Challenge 1 Walker for the WRO2025 Robot."""
from time import sleep
import logging
from base.mat import mat_color
from hardware.hardware_interface import HardwareInterface
from round1.matintelligence import MATDIRECTION, MATGENERICLOCATION, MatIntelligence
from round1.threadingfunctions import ConditionCheckerThread
from round1.walker_helpers import EquiWalkerHelper, GyroWalkerHelper

logger = logging.getLogger(__name__)
class Walker:
    """This class implements the Challenge 1 Walker for the WRO2025 Robot."""
    # Constants for the walker
    DEFAULT_SPEED=100/2
    WALK_TO_CORNER_SPEED = 25
    MAX_SPEED = 80
    WALK_TO_COLOR_SPEED= 10
    WALLFRONTDISTANCE=30
    WALLSIDEDISTANCE=20

    TURNRIGHT_ANGLE=10
    TURNLEFT_ANGLE=-10
    WALLFRONTENDDISTANCE=30
    D_TARGET = 15  # Desired distance from the wall
    KP = 2.0  # Proportional gain for the controller
    MAX_ANGLE = 10

    output_inf: HardwareInterface

    def __init__(self, output_inf:HardwareInterface):
        self.output_inf = output_inf
        self._line_color: str|None = None
        #setting decimals for float datatype.
        self._current_distance = (0.1, 0.1)

    def clamp(self,val, min_val, max_val):
        """Clamp the value between min_val and max_val."""
        return max(min(val, max_val), min_val)

    def equidistance_walk_start(self,use_mpu:bool,kp:float=0) -> EquiWalkerHelper:
        """Initialize the EquiWalkerHelper with default distances and angles."""
        if use_mpu:
            self.output_inf.reset_yaw()  # Reset yaw to zero

        left_distance = self.output_inf.get_left_distance()
        right_distance = self.output_inf.get_right_distance()
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

    def handle_unknowndirection_walk(self, intelligence: MatIntelligence):
        """Handle walking when the direction is unknown."""

        logger.info("Direction is unknown, starting the walk with default distances.")

        # Log the distances
        (front_distance,left_distance,right_distance) = self.output_inf.logdistances()

        #Add First reading to remember the starting point
        intelligence.add_readings(front_distance, left_distance, right_distance)

        (maxfront,_,__) = intelligence.get_learned_distances()

        helper: EquiWalkerHelper = self.equidistance_walk_start(use_mpu=True)

        # Lets start the walk until we reach the front distance, but at slow speed.
        self.output_inf.drive_forward(self.DEFAULT_SPEED)

        while self.output_inf.get_front_distance() > maxfront:

            self.handle_walk(helper,use_mpu=True,intelligence=intelligence,
                             is_unknown_direction=True)
            sleep(0.03)

        # self.output_inf.drive_stop()
        self.output_inf.drive_forward(self.WALK_TO_COLOR_SPEED)

        logger.info("Time to check color")

        knowncolor = ["blue", "orange"]
        color = self.check_bottom_color(knowncolor)

        gyrohelper:GyroWalkerHelper = GyroWalkerHelper()

        running = False

        if color is None:

            def set_line_color(c):
                self._line_color = c
                self.output_inf.drive_stop()

            def value_check_func():
                return self.check_bottom_color(knowncolor)

            colorchecker: ConditionCheckerThread = ConditionCheckerThread(
                value_check_func=value_check_func,
                callback_func=set_line_color,
                interval_ms=100
            )

            colorchecker.start()

            while (self.output_inf.get_front_distance() > self.WALLFRONTENDDISTANCE
                and self._line_color is None):

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


                if not running:
                    logger.info("color checking walk...")
                    self.output_inf.drive_forward(self.WALK_TO_COLOR_SPEED)
                    running = True
                    # self.output_inf.buzzer_beep(timer=0.1)
                    # sleep(0.001)

            if colorchecker.is_running():
                logger.info("Stopping color checker thread, not found color yet.")
                colorchecker.stop()

            #Lets first stop the base and then check the color.
            self.output_inf.drive_stop()
            self.output_inf.buzzer_beep()
            logger.info("Front Distance:%s",self.output_inf.get_front_distance())
            color = self._line_color

            color2 = self.check_bottom_color(knowncolor)

            #can we check.if one of the sides is not present?, that means we are at a corner.
            # and based on which side we can determine the direction.
            (_,left,right) = self.output_inf.logdistances()

            if left > right:
                logger.info("Left side is not present, right side is present," \
                "                                            setting direction to anitclockwise.")
                direction_hints = MATDIRECTION.ANTICLOCKWISE_DIRECTION
            elif right < left:
                logger.info("left side is present, right side is not present, " \
                "                                       setting direction to clockwise.")
                direction_hints = MATDIRECTION.CLOCKWISE_DIRECTION
            else:
                direction_hints = MATDIRECTION.UNKNOWN_DIRECTION

            #Now i should have three direction readings , from color, color2 and direction_hints

            directioncolor = self._color_to_direction(color)
            directioncolor2 = self._color_to_direction(color2)
            direction = intelligence.vote_direction([directioncolor, directioncolor2,
                                                      direction_hints])

            if direction != MATDIRECTION.UNKNOWN_DIRECTION:
                intelligence.report_direction_side1(direction)
                return

            #This means we cannot determine direction with proper voting.

            if directioncolor != direction_hints:
                # we would give presidence to hints if this not unknown
                if direction_hints != MATDIRECTION.UNKNOWN_DIRECTION:
                    intelligence.report_direction_side1(direction_hints)
                    return
                elif directioncolor != MATDIRECTION.UNKNOWN_DIRECTION:
                    intelligence.report_direction_side1(directioncolor)
                else:
                    #TODO: handle case we cannot handle direction.
                    return

            self.output_inf.buzzer_beep()
            logger.warning("running color: %s", color)
            logger.warning("later color: %s", color2)
            logger.warning("Hints direction: %s", self.directiontostr(direction_hints))
            logger.warning("Final direction: %s", self.directiontostr(direction))

        else:
            #Color is not None in starting itself.
            logger.warning("running color: %s", color)
            direction = self._color_to_direction(color)
            intelligence.report_direction_side1(direction)
            logger.warning("Direction:%s", self.directiontostr(direction))


        self.output_inf.drive_stop()
        self.output_inf.force_flush_messages()

    def start_walk(self,nooflaps:int=4):
        """Start the walk based on the current direction which is unknown and number of laps."""
        logger.info("Starting to walk...")

        intelligence:MatIntelligence = MatIntelligence()

        #this should set the direction
        self.handle_unknowndirection_walk(intelligence)

        #TODO: we cannot determine direction, beep and stop.
        if intelligence.get_direction() == MATDIRECTION.UNKNOWN_DIRECTION:
            logger.warning("Direction is unknown, stopping the walk.")
            self.output_inf.buzzer_beep()
            self.output_inf.led1_red()
            self.output_inf.force_flush_messages()
            return

        #We have detected the directions, and now time to walk knowing the directions.
        while intelligence.get_round_number()<= nooflaps:
            if intelligence.get_generic_location() == MATGENERICLOCATION.CORNER:
                # Handle corner.
                self._current_distance = (0, 0)

                def report_distances(left: float, right: float):
                    logger.info("corner Report. Left: %.2f, Right: %.2f", left, right)
                    self._current_distance = (left, right)

                intelligence.register_callback(report_distances)
                (maxfront,left_def,right_def) = intelligence.get_learned_distances()

                helper:EquiWalkerHelper = self.handle_corner_start(intelligence=intelligence,
                                                                   left=left_def,right=right_def)
                self.output_inf.drive_forward(self.WALK_TO_CORNER_SPEED)
                sleep(0.3)

                while self.output_inf.get_front_distance() > maxfront and \
                     self._current_distance == (0, 0):
                    self.handle_walk(helper=helper, intelligence=intelligence)
                    sleep(0.01)

                intelligence.location_complete()


                #TODO: Turned the corner, lets stop the base.
                self.output_inf.drive_stop()
                return
            else:
                #handle SIDE
                # we are planning to straight the robot and then do a gyro walk.
                 # Log the distances
                (front_distance,left_distance,right_distance) = self.output_inf.logdistances()

                intelligence.add_readings(front_distance, left_distance, right_distance)

                self._current_distance = (0, 0)

                def report_distances(left: float, right: float):
                    logger.info("side Report. Left: %.2f, Right: %.2f", left, right)
                    self._current_distance = (left, right)

                intelligence.register_callback(report_distances)
                (maxfront,left_def,right_def) = intelligence.get_learned_distances()

                helper:EquiWalkerHelper = self.handle_corner_start(intelligence=intelligence,
                                                                   left=left_def,right=right_def)
                self.output_inf.drive_forward(self.DEFAULT_SPEED)
                sleep(0.3)

                while self.output_inf.get_front_distance() > maxfront:
                    while self.output_inf.get_front_distance() > maxfront \
                                            and self._current_distance == (0, 0):
                        self.handle_walk(helper=helper, intelligence=intelligence)
                        sleep(0.01)

                    if self._current_distance != (0, 0):
                        #somehow we have found a smaller point.
                        #reset the helper
                        helper = self.handle_corner_start(intelligence=intelligence,left=left_def,
                                                          right=right_def)

                intelligence.location_complete()


                #TODO: Lets stop the base.
                self.output_inf.drive_stop()
                return

    def _color_to_direction(self,color)-> int:
        if color == "blue":
            return MATDIRECTION.ANTICLOCKWISE_DIRECTION
        elif color == "orange":
            return MATDIRECTION.CLOCKWISE_DIRECTION
        else:
            return MATDIRECTION.UNKNOWN_DIRECTION


    def directiontostr(self,direction):
        """Convert direction to string."""
        if direction == MATDIRECTION.CLOCKWISE_DIRECTION:
            return "Clockwise"
        elif direction == MATDIRECTION.ANTICLOCKWISE_DIRECTION:
            return "Anti-clockwise"
        else:
            return "Unknown"

    def handle_walk(self,helper:EquiWalkerHelper,intelligence:MatIntelligence,use_mpu=False,
                    is_unknown_direction:bool=False) -> float:
        """Handle walk using helper""" 

        logger.info("Handling walk with direction: %s",
                        self.directiontostr(intelligence.get_direction()))

        (front,left_distance,right_distance) = self.output_inf.logdistances()

        intelligence.add_readings(front,left_distance,right_distance)

        if not is_unknown_direction:
            (_,left_def,right_def) = intelligence.get_learned_distances()

            if left_def == -1:
                left_distance = self.output_inf.get_left_distance_max()

            if right_def == -1:
                right_distance = self.output_inf.get_right_distance_max()

        current_angle = 0
        if use_mpu:
            _, _, yaw = self.output_inf.get_orientation()
            current_angle = yaw # Assuming yaw gives the current angle.

        turn_angle = helper.equidistance_walk_func(
                            left_distance, right_distance, current_angle,
                            current_steering= self.output_inf.get_steering_angle())

        self._turn_steering_with_logging(turn_angle)
        return turn_angle

    def _turn_steering_with_logging(self,turn_angle):
        if turn_angle is not None:
            turn_angle = self.clamp(turn_angle, -self.MAX_ANGLE, self.MAX_ANGLE)
            if turn_angle >= 0:
                logger.info("Turning right to angle: %.2f", turn_angle)
            else:
                logger.info("Turning left to angle: %.2f", turn_angle)
            # Turn the steering based on the calculated angle
        self.output_inf.turn_steering(turn_angle)

    def handle_corner_start(self,intelligence:MatIntelligence,left:float,
                                            right:float) -> EquiWalkerHelper:
        """Initialize the EquiWalkerHelper with default distances and angles."""
        logger.info("Handling corner start with direction: %s",
                     self.directiontostr(intelligence.get_direction()))

        left_distance_max = self.output_inf.get_left_distance_max()
        right_distance_max = self.output_inf.get_right_distance_max()

        helper:EquiWalkerHelper = EquiWalkerHelper(
            def_distance_left=left,
            def_distance_right=right,
            max_left_distance=left_distance_max,
            max_right_distance=right_distance_max,kp=0
        )
        return helper


    def check_bottom_color(self,colors)->str|None:
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

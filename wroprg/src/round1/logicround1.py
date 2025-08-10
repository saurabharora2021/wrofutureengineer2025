""" This modules implements the Challenge 1 Walker for the WRO2025 Robot."""
from time import sleep
import logging
from base.mat import mat_color
from hardware.hardware_interface import HardwareInterface
from round1.matintelligence import MATDIRECTION, MATGENERICLOCATION, MatIntelligence
from round1.matintelligence import vote_directions, color_to_direction
from round1.threadingfunctions import ConditionCheckerThread
from round1.walker_helpers import EquiWalkerHelper, GyroWalkerHelper
from round1.walker_helpers import GyroWalkerwithMinDistanceHelper
from round1.utilityfunctions import clamp_angle, directiontostr

logger = logging.getLogger(__name__)
class Walker:
    """This class implements the Challenge 1 Walker for the WRO2025 Robot."""
    # Constants for the walker
    DEFAULT_SPEED=40
    WALK_TO_CORNER_SPEED = 30
    WALK_TO_COLOR_SPEED= 15

    WALLFRONTENDDISTANCE=30

    MAX_ANGLE = 15
    DELTA_ANGLE = 8

    output_inf: HardwareInterface

    def __init__(self, output_inf:HardwareInterface,nooflaps:int=1):
        self.output_inf = output_inf
        self._line_color: str|None = None
        #setting decimals for float datatype.
        self._current_distance = (0.1, 0.1)
        self._nooflaps = nooflaps
        self._intelligence: MatIntelligence = MatIntelligence(roundcount=nooflaps)

        self._walking:bool = False

    def handle_unknowndirection_walk(self):
        """Handle walking when the direction is unknown."""

        logger.info("Direction is unknown, starting the walk with default distances.")

        # Log the distances
        (_, start_left_distance, start_right_distance) = self.read_log_distances()

        gyrodefault = 0
        deltadistance = start_right_distance - start_left_distance

        #If Delta is high move towards the center, move by 10cm otherwise too high correction.
        if abs(deltadistance)> 10:
            if start_left_distance < start_right_distance:
                logger.info("Adjusting left distance")
                start_left_distance += 10
                start_right_distance -= 10
                gyrodefault = -2
            else:
                logger.info("Adjusting right distance")
                start_left_distance -= 10
                start_right_distance += 10
                gyrodefault = 2
            logger.info("adjusted left %.2f , right %.2f",start_left_distance,start_right_distance)

        (maxfront,_,__) = self._intelligence.get_learned_distances()

        logger.info("Max front distance: %.2f", maxfront)

        helper: EquiWalkerHelper = self._handle_walk_start(
                                 left_distance=start_left_distance,
                                 right_distance=start_right_distance,
                                 use_mpu=True,
                                 def_turn_angle=gyrodefault
                                 )

        # Lets start the walk until we reach the front distance, but at slow speed.
        # if self.output_inf.get_front_distance() < 150:
        #     _start_walking(self.WALK_TO_COLOR_SPEED)
        # else:
        #     _start_walking(self.DEFAULT_SPEED)
        self._start_walking(self.WALK_TO_COLOR_SPEED)

        while self.output_inf.get_front_distance() > maxfront:

            self.handle_walk(helper,use_mpu=True,is_unknown_direction=True,speedcheck=True)

        self.output_inf.buzzer_beep()
        #self._stop_walking()
        #return
        self._start_walking(self.WALK_TO_COLOR_SPEED)

        logger.info("Time to check color")

        knowncolor = ["blue", "orange"]
        color = self.check_bottom_color(knowncolor)

        gyrohelper:GyroWalkerHelper = GyroWalkerHelper()

        # running = False

        if color is None:

            def set_line_color(c):
                self._line_color = c
                self._stop_walking()

            def value_check_func():
                return self.check_bottom_color(knowncolor)

            colorchecker: ConditionCheckerThread = ConditionCheckerThread(
                value_check_func=value_check_func,
                callback_func=set_line_color,
                interval_ms=50
            )

            colorchecker.start()

            while (self.output_inf.get_front_distance() > self.WALLFRONTENDDISTANCE
                and self._line_color is None):

                _, _, yaw = self.output_inf.get_orientation()
                turn_angle = gyrohelper.walk_func(yaw,self.output_inf.
                                                            get_steering_angle())

                self._turn_steering_with_logging(turn_angle)

                logger.info("Front Distance:%s",self.output_inf.get_front_distance())

            #Lets first stop the base and then check the color.
            self._stop_walking()
            if colorchecker.is_running():
                logger.info("Stopping color checker thread, not found color yet.")
                colorchecker.stop()

            self.output_inf.buzzer_beep()
            logger.info("Front Distance:%s",self.output_inf.get_front_distance())
            color = self._line_color

            color2 = self.check_bottom_color(knowncolor)

            #can we check.if one of the sides is not present?, that means we are at a corner.
            # and based on which side we can determine the direction.
            (front,left,right) = self.read_log_distances()

            if left + right < 150:
                logger.info("Both sides are present , we have not reached the corner")

                helper: EquiWalkerHelper = self._handle_walk_start(
                                 left_distance=left,
                                 right_distance=right,
                                 use_mpu=False)
                self._start_walking(self.WALK_TO_COLOR_SPEED)
                while front > self.WALLFRONTENDDISTANCE \
                                        and (left + right < 150):
                    self.handle_walk(helper=helper,is_unknown_direction=True)
                    (front,left,right) = self.read_log_distances()
                    # sleep(0.01)

                logger.info("Both side present walk, left %s,right %s front %s", left, right, front)
                self._stop_walking()

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

            directioncolor = color_to_direction(color)
            directioncolor2 = color_to_direction(color2)
            direction = vote_directions([directioncolor, directioncolor2,
                                                      direction_hints])

            if direction != MATDIRECTION.UNKNOWN_DIRECTION:
                self._intelligence.report_direction_side1(direction)
            else:

                #This means we cannot determine direction with proper voting.

                if directioncolor != direction_hints:
                    # we would give presidence to hints if this not unknown
                    if direction_hints != MATDIRECTION.UNKNOWN_DIRECTION:
                        self._intelligence.report_direction_side1(direction_hints)
                        return
                    elif directioncolor != MATDIRECTION.UNKNOWN_DIRECTION:
                        self._intelligence.report_direction_side1(directioncolor)
                    else:
                        #TODO: handle case we cannot handle direction.
                        return

            self.output_inf.buzzer_beep()
            logger.warning("running color: %s", color)
            logger.warning("later color: %s", color2)
            logger.warning("Hints direction: %s", directiontostr(direction_hints))
            logger.warning("Final direction: %s", directiontostr(direction))

        else:
            #Color is not None in starting itself.
            logger.warning("running color: %s", color)
            direction = color_to_direction(color)
            self._intelligence.report_direction_side1(direction)
            logger.warning("Direction:%s", directiontostr(direction))

        self.output_inf.force_flush_messages()

    def start_walk(self):
        """Start the walk based on the current direction which is unknown and number of laps."""
        logger.info("Starting to walk...")

        #this should set the direction
        self.handle_unknowndirection_walk()
        #we don't need camera Now
        self.output_inf.camera_off()
        self.output_inf.reset_yaw()

        #TODO: we cannot determine direction, beep and stop.
        if self._intelligence.get_direction() == MATDIRECTION.UNKNOWN_DIRECTION:
            logger.warning("Direction is unknown, stopping the walk.")
            self.output_inf.buzzer_beep()
            self.output_inf.led1_red()
            self.output_inf.force_flush_messages()
            return

        #We have detected the directions, and now time to walk knowing the directions.
        while self._intelligence.get_round_number()<= self._nooflaps:
            logger.info("Starting walk for location: %s , round: %d",
                         self._intelligence.get_location(), self._intelligence.get_round_number())

            if self._intelligence.get_generic_location() == MATGENERICLOCATION.CORNER:
                # Handle corner.
                self._current_distance = (0, 0)

                #TODO: if the inside is still visible we need to walk ahead and handle it first
                # before turning.

                def report_distances_corner(left: float, right: float):
                    logger.info("corner Report. Left: %.2f, Right: %.2f", left, right)
                    self._current_distance = (left, right)
                    self._stop_walking()

                self._intelligence.register_callback(report_distances_corner)
                (maxfront,left_def,right_def) = self._intelligence.get_learned_distances()

                (x,c_left,c_right) = self.read_log_distances()
                self._intelligence.add_readings(x,c_left,c_right)

                logger.info("Corner Walk with front %.2f ,left %.2f,right %.2f", maxfront,
                            left_def,right_def)

                helper:EquiWalkerHelper =  self._handle_walk_start(left_distance=left_def,
                                                                   right_distance=right_def)

                #add some angle in the direction.
                turn_angle = 0
                if self._intelligence.get_direction() ==MATDIRECTION.CLOCKWISE_DIRECTION:
                    #Turn Right a light
                    turn_angle = 5
                    if c_left < 40:
                        #No turn
                        turn_angle = 0
                    elif c_right < 40:
                        turn_angle = 10
                else:
                    #Turn left a light
                    #Based on location determine the turn angle
                    turn_angle = -5
                    if c_left < 40:
                          #No turn
                        turn_angle = 0
                    elif c_right < 40:
                        turn_angle = -10
                self._turn_steering_with_logging(turn_angle=turn_angle)

                self._start_walking(self.WALK_TO_CORNER_SPEED)

                while self.output_inf.get_front_distance() > maxfront and \
                     self._current_distance == (0, 0):
                    self.handle_walk(helper=helper,is_corner=True)
                    # sleep(0.01)

                self._intelligence.location_complete()
                self._intelligence.unregister_callback()


                #TODO: Turned the corner, lets stop the base.
                self._stop_walking()
                self.output_inf.buzzer_beep()
                self.output_inf.reset_steering()
                self.output_inf.reset_yaw()
                return
            else:
                #handle SIDE
                # we are planning to straight the robot and then do a gyro walk.
                # Log the distances
                self.read_log_distances()

                self._current_distance = (0, 0)

                def report_distances_side(left: float, right: float):
                    logger.info("side Report. Left: %.2f, Right: %.2f", left, right)
                    self._current_distance = (left, right)
                    self._stop_walking()

                self._intelligence.register_callback(report_distances_side)
                (maxfront,left_def,right_def) = self._intelligence.get_learned_distances()

                helper:EquiWalkerHelper = self._handle_walk_start(left_distance=left_def,
                                                                   right_distance=right_def)

                self._start_walking(self.WALK_TO_CORNER_SPEED)
                # sleep(0.1)

                while self.output_inf.get_front_distance() > maxfront:
                    while self.output_inf.get_front_distance() > maxfront \
                                            and self._current_distance == (0, 0):
                        self.handle_walk(helper=helper)
                        sleep(0.01)

                    if self._current_distance != (0, 0):
                        #somehow we have found a smaller point.
                        #reset the helper
                        helper = self._handle_walk_start(left_distance=left_def,
                                                         right_distance=right_def)
                        self._start_walking(self.WALK_TO_CORNER_SPEED)


                self._intelligence.location_complete()
                self._intelligence.unregister_callback()


                #TODO: Lets stop the base.
                self._stop_walking()
                return

    def handle_walk(self,helper:EquiWalkerHelper,use_mpu=False,
                    is_unknown_direction:bool=False,is_corner:bool = False,
                    speedcheck=False) -> float:
        """Handle walk using helper""" 

        logger.info("Handling walk with direction: %s",
                        directiontostr(self._intelligence.get_direction()))

        (_,left_distance,right_distance) = self.read_log_distances()

        if not is_unknown_direction:
            (_,left_def,right_def) = self._intelligence.get_learned_distances()

            if left_def == -1:
                left_distance = self.output_inf.get_left_distance_max()

            if right_def == -1:
                right_distance = self.output_inf.get_right_distance_max()

        if is_corner:
            logger.info("Special corner handling")
            if left_distance >= self.output_inf.get_left_distance_max():
                logger.info("At corner, adjusting steering.")
                left_distance -=10
        current_angle = 0
        if use_mpu:
            _, _, yaw = self.output_inf.get_orientation()
            current_angle = yaw # Assuming yaw gives the current angle.

        turn_angle = helper.equidistance_walk_func(
                            left_distance, right_distance, current_angle,
                            current_steering_angle= self.output_inf.get_steering_angle())

        self._turn_steering_with_logging(turn_angle,speedcheck=speedcheck)
        return turn_angle

    def _turn_steering_with_logging(self,turn_angle,delta_angle:float=0,speedcheck:bool=False):
        if turn_angle is not None:
            turn_angle = clamp_angle(turn_angle, self.MAX_ANGLE)
            current_steering_angle = self.output_inf.get_steering_angle()
            #we should restrict the delta angle to DELTA_ANGLE
            logger.info("turn angle before delta: %.2f", turn_angle)
            delta = turn_angle - current_steering_angle
            max_delta_angle = self.DELTA_ANGLE if delta_angle == 0 else delta_angle
            if abs(delta) > max_delta_angle:
                logger.info("Restricting turn angle to DELTA_ANGLE: %.2f", max_delta_angle)
                turn_angle = current_steering_angle + \
                            (max_delta_angle if delta > 0 else -max_delta_angle)
            logger.info("turn angle after delta: %.2f", turn_angle)
            if turn_angle >= 0:
                logger.info("Turning right to angle: %.2f", turn_angle)
            else:
                logger.info("Turning left to angle: %.2f", turn_angle)
            # Turn the steering based on the calculated angle
            if (speedcheck and abs(max_delta_angle) >= self.DELTA_ANGLE):
                self._start_walking(self.WALK_TO_COLOR_SPEED)
            self.output_inf.turn_steering(turn_angle)

    def _handle_walk_start(self,left_distance:float,
                           right_distance:float,
                           use_mpu:bool=False,kp:float=0,
                           def_turn_angle:float=0.0) -> EquiWalkerHelper:

        logger.info("Handling walk start with direction: %s",
                     directiontostr(self._intelligence.get_direction()))
        if use_mpu:
            self.output_inf.reset_yaw()  # Reset yaw to zero

        left_distance_max = self.output_inf.get_left_distance_max()
        right_distance_max = self.output_inf.get_right_distance_max()

        helper:EquiWalkerHelper = EquiWalkerHelper(
            def_distance_left=left_distance,
            def_distance_right=right_distance,
            max_left_distance=left_distance_max,
            max_right_distance=right_distance_max,
            kp=kp,
            def_turn_angle=def_turn_angle
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

    def read_log_distances(self)->tuple[float, float, float]:
        """Read the distances from the log."""
        (front_distance,left_distance,right_distance) = self.output_inf.logdistances()

        self._intelligence.add_readings(front_distance, left_distance, right_distance)
        return (front_distance, left_distance, right_distance)

    def gyro_corner_walk(self,def_turn_angle:float):
        """Handle the gyro corner walking logic."""
        logger.info("Gyro corner walk initiated with turn angle: %.2f", def_turn_angle)

        self._current_distance = (0, 0)
        def report_distances_corner(left: float, right: float):
            logger.info("corner Report. Left: %.2f, Right: %.2f", left, right)
            self._current_distance = (left, right)

        # Implement the gyro corner walking logic here
        self.output_inf.reset_yaw()
        gyrohelper: GyroWalkerwithMinDistanceHelper = GyroWalkerwithMinDistanceHelper(
            walk_angle=def_turn_angle, min_left=15, min_right=15)

        (def_front, _, _) = self._intelligence.get_learned_distances()
        (front, left, right) = self.read_log_distances()

        self._start_walking(self.WALK_TO_COLOR_SPEED)
        turned = False
        while front > def_front and \
                     self._current_distance == (0, 0):

            _, _, yaw = self.output_inf.get_orientation()
            logger.info("Current Yaw: %.2f", yaw)
            turn_angle = gyrohelper.walk_func(yaw, self.output_inf.get_steering_angle(),
                                              left, right)
            if not turned and yaw < def_turn_angle/2:
                logger.info("Turned achieve lets check distance")
                turned = True
                self._intelligence.reset_current_distance()
                self._intelligence.register_callback(report_distances_corner)

            self._turn_steering_with_logging(turn_angle,delta_angle=10)

        self._stop_walking()
        self._intelligence.location_complete()
        self._intelligence.unregister_callback()

    def _start_walking(self, speed: float):
        """Start the walking movement."""
        self._walking = True
        self.output_inf.drive_forward(speed)

    def _stop_walking(self):
        """Stop the walking movement."""
        self._walking = False
        self.output_inf.drive_stop()

    def gyro_color_walk(self,def_turn_angle:float):
        """Handle the gyro corner walking logic."""
        logger.info("Gyro corner walk initiated with turn angle: %.2f", def_turn_angle)

        self._current_distance = (0, 0)
        def report_distances_corner(left: float, right: float):
            logger.info("corner Report. Left: %.2f, Right: %.2f", left, right)
            self._current_distance = (left, right)

        # Implement the gyro corner walking logic here
        self.output_inf.reset_yaw()
        gyrohelper: GyroWalkerwithMinDistanceHelper = GyroWalkerwithMinDistanceHelper(
            walk_angle=def_turn_angle, min_left=15, min_right=15)

        (def_front, _, _) = self._intelligence.get_learned_distances()
        (front, left, right) = self.read_log_distances()

        self._start_walking(self.WALK_TO_COLOR_SPEED)
        turned = False
        while front > def_front and \
                     self._current_distance == (0, 0):

            _, _, yaw = self.output_inf.get_orientation()
            logger.info("Current Yaw: %.2f", yaw)
            turn_angle = gyrohelper.walk_func(yaw, self.output_inf.get_steering_angle(),
                                              left, right)
            if not turned and yaw < def_turn_angle/2:
                logger.info("Turned achieve lets check distance")
                turned = True
                self._intelligence.reset_current_distance()
                self._intelligence.register_callback(report_distances_corner)

            self._turn_steering_with_logging(turn_angle,delta_angle=10)

        self._stop_walking()
        self._intelligence.location_complete()
        self._intelligence.unregister_callback()
    def handle_straight_walk_to_distance(self,min_front:float,min_left:float,min_right:float,
                                         gyrodefault:float)->None:
        """Handle the straight walking logic."""
        logger.info("Straight walk initiated with min distances - Front: %.2f, Left: %.2f,\
                     Right: %.2f",min_front, min_left, min_right)
        helper: EquiWalkerHelper = self._handle_walk_start(
                                 left_distance=min_left,
                                 right_distance=min_right,
                                 use_mpu=True,
                                 def_turn_angle=gyrodefault
                                 )

        # Lets start the walk until we reach the front distance, but at slow speed.
        # if self.output_inf.get_front_distance() < 150:
        #     _start_walking(self.WALK_TO_COLOR_SPEED)
        # else:
        #     _start_walking(self.DEFAULT_SPEED)
        self._start_walking(self.WALK_TO_COLOR_SPEED)

        while self.output_inf.get_front_distance() > min_front:

            self.handle_walk(helper,use_mpu=True,is_unknown_direction=True,speedcheck=True)

        self.output_inf.buzzer_beep()

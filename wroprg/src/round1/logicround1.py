""" This modules implements the Challenge 1 Walker for the WRO2025 Robot."""
import logging
import time
from typing import Callable, Optional, Tuple
from hardware.hardware_interface import HardwareInterface
from round1.matintelligence import MATDIRECTION, MATGENERICLOCATION, MatIntelligence
from round1.matintelligence import vote_directions, color_to_direction
from round1.walker_helpers import EquiWalkerHelper, GyroWalkerwithMinDistanceHelper
from round1.utilityfunctions import clamp_angle, directiontostr, check_bottom_color
from utils.threadingfunctions import ConditionCheckerThread

logger = logging.getLogger(__name__)
class Walker:
    """This class implements the Challenge 1 Walker for the WRO2025 Robot."""
    # Constants for the walker
    DEFAULT_SPEED=40
    WALK_TO_CORNER_SPEED = 30
    MIN_SPEED= 15

    WALLFRONTENDDISTANCE=30

    MAX_ANGLE = 15
    DELTA_ANGLE = 8

    output_inf: HardwareInterface

    def __init__(self, output_inf:HardwareInterface,nooflaps:int=1):
        self.output_inf = output_inf
        self._line_color: str|None = None
        #setting decimals for float datatype.
        self._current_distance = (0.1, 0.1)
        self._start_time=0
        self._current_yaw = 0

        self._nooflaps = nooflaps
        self._intelligence: MatIntelligence = MatIntelligence(roundcount=nooflaps,
                                                              hardware_interface=output_inf)

        self._walking:bool = False

    def _center_bot_correction(self,front:float, left:float,
                                right:float) -> Tuple[bool,float, float]:
        """Correct the left and right distances to center the bot."""

        #If Delta is high move towards the center, move by 10cm otherwise too high correction.

        deltadistance = right - left
        #If Delta is high move towards the center, move by 10cm otherwise too high correction.
        if abs(deltadistance)> 10:
            correction = 10 if front > 130 else 5
            if left < right:
                logger.info("Adjusting left distance")
                left += correction
                right -= correction
            else:
                logger.info("Adjusting right distance")
                left -= correction
                right += correction
            logger.info("adjusted left %.2f , right %.2f",left,right)
            return False,left, right
        else:
            logger.info("Delta distance is low, not changing distances.")
            return True,left,right

    def handle_unknowndirection_walk(self):
        """Handle walking when the direction is unknown."""

        logger.info("Direction is unknown, starting the walk with default distances.")

        # Log the distances
        (start_front_distance, start_left_distance,
                                 start_right_distance) = self.read_log_distances()

        gyrodefault = self.output_inf.get_orientation()[2]
        totalstartdistance = start_left_distance + start_right_distance

        midpoint:bool = True

        midpoint, start_left_distance, start_right_distance = \
            self._center_bot_correction(start_front_distance, start_left_distance,
                                         start_right_distance)

        (maxfront,_,__) = self._intelligence.get_learned_distances()

        logger.info("Max front distance: %.2f", maxfront)

        self.handle_straight_walk_to_distance(maxfront,start_left_distance,start_right_distance,
                                              gyrodefault,self.MIN_SPEED,speedcheck=True,
                                              force_change=not midpoint)

        #self._stop_walking()
        #return
        #Complete walk to corner , now lets find the color for direction.

        (color,color2) = self.walk_read_mat_color(start_distance=totalstartdistance)

        #ensure we have reached corner.
        self.walk_to_corner()

        (_,left,right) = self.read_log_distances()

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

        self.output_inf.force_flush_messages()

    def start_walk(self):
        """Start the walk based on the current direction which is unknown and number of laps."""
        logger.info("Starting to walk...")

        #this should set the direction
        self.handle_unknowndirection_walk()
        #we don't need camera Now
        self.output_inf.camera_off()

        #we cannot determine direction, beep and stop.
        if self._intelligence.get_direction() == MATDIRECTION.UNKNOWN_DIRECTION:
            logger.error("Direction is unknown, stopping the walk.")
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
                self.handle_corner()
            else:
                #handle SIDE
                self.handle_side()

        #print the matintelligence
        self._intelligence.print_mat_intelligence()

    def _side_bot_centering(self,front:float,learned_left:float,learned_right:float,
                                actual_left:float,
                                    actual_right:float)-> Tuple[bool,float,float]:
        total_learned = learned_left + learned_right
        total_actual = actual_left + actual_right
        if abs(total_learned - total_actual) < 2:
            #forget the difference, center actual an return
            return self._center_bot_correction(front, actual_left, actual_right)
        else:
            # we need to proportionally reduce actual
            if total_actual<total_learned:
                return self._center_bot_correction(front, actual_left, actual_right)
            else:
                # total actual > total_learned
                # we need to proportionally reduce actual
                diff = total_actual - total_learned
                if actual_left> actual_right:
                    actual_left -= diff
                else:
                    actual_right -= diff

                return self._center_bot_correction(front, actual_left, actual_right)

    def handle_side(self):
        """Handle side walk"""

        # we are planning to straight the robot and then do a gyro walk.
        # Log the distances
        self.read_log_distances()

        #lets start with zero heading.
        self._stop_walking()
        self.output_inf.reset_gyro()


        max_left = self.output_inf.get_left_distance_max()
        max_right = self.output_inf.get_right_distance_max()

        (minfront,left_def,right_def) = self._intelligence.get_learned_distances()

        def_yaw = self.output_inf.get_orientation()[2]
        current_yaw = def_yaw

        (current_front,current_left,current_right) = self.read_log_distances()

        midpoint:bool = True

        (midpoint, left_def, right_def) = self._side_bot_centering(current_front,
                                                    left_def,right_def,current_left,current_right)

        logger.info("handle side L:%0.2f, R:%0.2f correction %s", left_def, right_def,not midpoint )
        self._current_distance = (0, 0)

        def report_distances_side(left: float, right: float):
            logger.info("side Report. Left: %.2f, Right: %.2f", left, right)
            self._current_distance = (left, right)
            # self._stop_walking()

        prev_distance = current_left + current_right
        prev_left = current_left
        prev_right = current_right
        def condition_met(_front,left,right,_yaw) -> bool:
            # Define the condition for stopping the walk
            # Either you have found the new minimum point or
            # previous distances are less than current distances by a good margin.
            return self._current_distance == (0, 0) and \
                   abs((left + right)- prev_distance)< 5

        self._intelligence.register_callback(report_distances_side)

        (current_front,current_left,current_right) = self.read_log_distances()
        while current_front > minfront:

            logger.info("Starting round handle side...")
            self.handle_straight_walk_to_distance(min_front=minfront,min_left=left_def,
                                                  min_right=right_def,
                                              gyrodefault=current_yaw,
                                              defaultspeed=self.DEFAULT_SPEED,
                                              precondition=condition_met)
            (current_front,current_left,current_right) = self.read_log_distances()
            #Either you have found a new less point, or current left and right are
            #  less the previous time to correct and center
            if self._current_distance != (0, 0):
                _,left_def, right_def = self._side_bot_centering(current_front,
                                                    learned_left=self._current_distance[0],
                                                    learned_right=self._current_distance[1],
                                                    actual_left=current_left,
                                                    actual_right=current_right)
                self._current_distance = (0, 0)
                logger.info("Updated Def distances current distances - L: %.2f, R: %.2f", left_def,
                                                                                     right_def)
                current_yaw = def_yaw
            elif abs((current_left + current_right)- prev_distance) > 5:

                delta_distance = prev_distance - (current_left + current_right)
                logger.info("Significant distance change detected: %.2f", delta_distance)
                if delta_distance > 0:
                    # If the delta is positive, means we are getting straighter
                    _, left_def, right_def = self._side_bot_centering(current_front,
                                                            learned_left=self._current_distance[0],
                                                            learned_right=self._current_distance[1],
                                                            actual_left=current_left,
                                                            actual_right=current_right)
                    prev_distance = current_left + current_right
                    prev_left = current_left
                    prev_right = current_right
                    current_yaw = def_yaw
                    logger.info("Updated Def distances for positive delta - L: %.2f, R: %.2f",
                                 left_def, right_def)
                else:
                    # distance is negative, means we are getting less straight, this is bad news.
                    # can we somehow correct this.
                    if current_left >= max_left or current_right >= max_right:
                        logger.warning("distance increase from prev distance, but one "\
                                       "side is missing cannot reset.")
                    else:
                        if current_left > prev_left and current_right <= prev_right:
                            #we are drifting right , try to gyro corrections to left.
                            current_yaw +=1
                            prev_distance = current_left + current_right
                            prev_left = current_left
                            prev_right = current_right
                            logger.info("Drifing right , we need to reset gyro to handle")
                            logger.info("Updated Def distances for delta - Yaw: %.2f",current_yaw)
                        elif current_right > prev_right and current_left <= prev_left:
                            # we are drifting left, try gyro corrections to right
                            current_yaw -= 1
                            prev_distance = current_left + current_right
                            prev_left = current_left
                            prev_right = current_right
                            logger.info("Drifing left , we need to reset gyro to handle")
                            logger.info("Updated Def distances for delta - Yaw: %.2f",current_yaw)
            else:
                logger.info("Completed round handle side...")

        self._stop_walking()
        self._intelligence.location_complete()
        self._intelligence.unregister_callback()

    def handle_corner(self):
        """Handle corner walk"""
        self._current_distance = (0, 0)

        self.walk_to_corner()
        current_direction = self._intelligence.get_direction()
        (_front,left,right) = self.read_log_distances()

        def_turn_angle = 0
        current_angle = self.output_inf.get_orientation()[2]
        logger.info("handle_corner current yaw %.2f",current_angle)
        if current_direction == MATDIRECTION.ANTICLOCKWISE_DIRECTION:
            # lets assume this is AntiClockwise and side1 is complete,
            # we have reached corner1
            if right > 50:
                logger.info("Based on current right distance, turn 65")
                def_turn_angle=max(65, 65-current_angle)
            else:
                logger.info("Based on current right distance, turn 55")
                def_turn_angle=max(55, 55-current_angle)
        else:
            if left > 50:
                logger.info("Based on current left distance, turn -65")
                def_turn_angle=min(-65, -65-current_angle)
            else:
                logger.info("Based on current left distance, turn -55")
                def_turn_angle=min(-55 , -55-current_angle)
        self.gyro_corner_walk(def_turn_angle=def_turn_angle)
        self.output_inf.buzzer_beep()

        return

    def _handle_walk(self,helper:EquiWalkerHelper,current_speed:float,use_mpu=False,
                    is_unknown_direction:bool=False,is_corner:bool = False,
                    speedcheck=False) -> Optional[float]:
        """Handle walk using helper""" 

        logger.info("Handling walk with direction: %s",
                        directiontostr(self._intelligence.get_direction()))

        (_,left_distance,right_distance) = self.read_log_distances()

        #TODO: Handle somewhere else.
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

        turn_angle = helper.walk_func(
                            left_distance=left_distance,
                             right_distance=right_distance, current_angle=current_angle)

        self._turn_steering_with_logging(turn_angle,speedcheck=speedcheck,
                                         current_speed=current_speed)
        return turn_angle

    def _turn_steering_with_logging(self,turn_angle,current_speed:float,delta_angle:float=0,
                                    speedcheck:bool=False,
                                    max_turn_angle:float=MAX_ANGLE):
        if turn_angle is not None:
            turn_angle = clamp_angle(turn_angle, max_turn_angle)
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
            turn_angle = clamp_angle(turn_angle, max_turn_angle)
            if turn_angle >= 0:
                logger.info("Turning right to angle: %.2f", turn_angle)
            else:
                logger.info("Turning left to angle: %.2f", turn_angle)
            # Turn the steering based on the calculated angle
            if (speedcheck and abs(max_delta_angle) >= self.DELTA_ANGLE):
                self._start_walking(self.MIN_SPEED)
            self.output_inf.turn_steering(turn_angle)
        else:
            logger.info("Turn angle is None")
            if speedcheck:
                self._start_walking(current_speed)

    def read_log_distances(self)->tuple[float, float, float]:
        """Read the distances from the log."""
        (front_distance,left_distance,right_distance) = self.output_inf.logdistances()

        self._intelligence.add_readings(front_distance, left_distance, right_distance)
        return (front_distance, left_distance, right_distance)

    def gyro_corner_walk(self,def_turn_angle:float):
        """Handle the gyro corner walking logic."""

        #TODO: Implement the gyro corner for round 2 onwards.
        logger.info("Gyro corner walk initiated with turn angle: %.2f", def_turn_angle)

        self._current_distance = (0, 0)
        self._start_time=0
        self._current_yaw = 0

        def report_distances_corner(left: float, right: float):
            logger.info("corner Report. Left: %.2f, Right: %.2f", left, right)
            prev_distance = self._current_distance
            self._current_distance = (left, right)
            if abs(prev_distance[0]+prev_distance[1] - (left + right)) > 2:
                logger.warning("Significant distance change detected.")
                self._start_time = time.time()
                #TODO: would this be called in round 2.
                self._stop_walking()
            self._current_yaw = self.output_inf.get_orientation()[2]

        # Implement the gyro corner walking logic here
        self.output_inf.reset_gyro()
        min_left=20
        min_right=20

        if self._intelligence.get_direction() == MATDIRECTION.CLOCKWISE_DIRECTION:
            min_left = 30
        else:
            min_right = 30

        gyrohelper: GyroWalkerwithMinDistanceHelper = GyroWalkerwithMinDistanceHelper(
            def_turn_angle=def_turn_angle, min_left=min_left, min_right=min_right,
              hardware=self.output_inf)

        (def_front, _, _) = self._intelligence.get_learned_distances()
        (front, left, right) = self.read_log_distances()

        #we are going to start turning before walking
        _, _, yaw = self.output_inf.get_orientation()
        turn_angle = gyrohelper.walk_func(current_angle=yaw,
                                              left_distance=left, right_distance=right)
        self._turn_steering_with_logging(turn_angle,delta_angle=15,max_turn_angle=25,
                                             current_speed=self.MIN_SPEED)

        self._start_walking(self.MIN_SPEED)
        turned = False

        turn_max_delta = abs(def_turn_angle/2)
        current_time = self._start_time
        while front > def_front and \
                     (self._start_time == 0):
            # or (current_time - self._start_time) < 0.5):

            _, _, yaw = self.output_inf.get_orientation()
            logger.info("Current Yaw: %.2f", yaw)
            turn_angle = gyrohelper.walk_func(current_angle=yaw,
                                              left_distance=left, right_distance=right)
            if not turned and abs(yaw - def_turn_angle) < turn_max_delta :
                logger.info("Turned achieve lets check distance=====")
                turned = True
                self._intelligence.reset_current_distance()
                self._intelligence.register_callback(report_distances_corner)

            self._turn_steering_with_logging(turn_angle,delta_angle=15,max_turn_angle=20,
                                             current_speed=self.MIN_SPEED)
            time.sleep(0.001)
            (front, left, right) = self.read_log_distances()
            if self._start_time != 0:
                current_time = time.time()

        if front<= def_front:
            logger.error("Too close to wall for front wall.")

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

    def handle_straight_walk_to_distance(self,min_front:float,min_left:float,min_right:float,
                                         gyrodefault:float,defaultspeed:float,speedcheck:bool=True,
                                         precondition:Optional[Callable[[float,float,float,float],
                                                                            bool]]=None,
                                         use_mpu=True,force_change:bool=False,
                                         base_helper: Optional[EquiWalkerHelper] = None) ->  None:
        """Handle the straight walking logic.
           precondition: Callable Function with argument as (front,left,right),yaw
        """
        logger.info("Straight walk initiated with min distances - Front: %.2f, Left: %.2f,"+
                     " Right: %.2f, Gyro: %.2f",min_front, min_left, min_right, gyrodefault)


        if base_helper is None:
            left_distance_max = self.output_inf.get_left_distance_max()
            right_distance_max = self.output_inf.get_right_distance_max()

            if force_change is False:

                helper:EquiWalkerHelper = EquiWalkerHelper(
                    def_distance_left=min_left,
                    def_distance_right=min_right,
                    max_left_distance=left_distance_max,
                    max_right_distance=right_distance_max,
                    def_turn_angle=gyrodefault,
                    hardware=self.output_inf,
                )
            else:
                helper:EquiWalkerHelper = EquiWalkerHelper(
                    def_distance_left=min_left,
                    def_distance_right=min_right,
                    max_left_distance=left_distance_max,
                    max_right_distance=right_distance_max,
                    def_turn_angle=gyrodefault,
                    hardware=self.output_inf,
                    kp=-3,
                    fused_distance_weight=0.3
                )
        else:
            helper = base_helper

        self._start_walking(defaultspeed)

        (front, left, right) = self.read_log_distances()
        yaw = self.output_inf.get_orientation()[2]

        if precondition is None:
            def noopcond(_front: float, _left: float, _right: float, _yaw: float) -> bool:
                return True

            precondition = noopcond

        while front > min_front and \
                        precondition(front,left,right,yaw) is True:

            self._handle_walk(helper,use_mpu=use_mpu,is_unknown_direction=True,
                              speedcheck=speedcheck,current_speed=defaultspeed)
            (front, left, right) = self.read_log_distances()
            yaw = self.output_inf.get_orientation()[2]
            logger.warning("F:%.2f, L:%.2f, R:%.2f, Y:%.2f , ",
                       front, left, right, yaw)
            time.sleep(0.005)

        self.output_inf.buzzer_beep()

    def walk_read_mat_color(self, start_distance: float) -> Tuple[str|None,str|None]:
        """Read the bottom color using the mat_color function."""

        self._start_walking(self.MIN_SPEED)

        logger.info("Time to check color")

        min_left = 40
        min_right = 40

        if start_distance < 70:
            min_left = 20
            min_right = 20

        knowncolor = ["blue", "orange"]
        color = check_bottom_color(self.output_inf, knowncolor)

        gyrohelper:GyroWalkerwithMinDistanceHelper = GyroWalkerwithMinDistanceHelper(
                                                        hardware=self.output_inf,
                                                        min_left=min_left,
                                                        min_right=min_right,
                                                        kgyro=-5.0,
                                                    )

        # running = False

        if color is None:

            def set_line_color(c):
                self._line_color = c
                self._stop_walking()

            def value_check_func():
                return check_bottom_color(self.output_inf, knowncolor)

            colorchecker: ConditionCheckerThread = ConditionCheckerThread(
                value_check_func=value_check_func,
                callback_func=set_line_color,
                interval_ms=50
            )

            (front,left,right) = self.read_log_distances()
            colorchecker.start()

            while (front > self.WALLFRONTENDDISTANCE
                and self._line_color is None):

                _, _, yaw = self.output_inf.get_orientation()

                turn_angle = gyrohelper.walk_func(left_distance=left, right_distance=right,
                                                  current_angle=yaw)

                self._turn_steering_with_logging(turn_angle,current_speed=self.MIN_SPEED)

                (front,left,right) = self.read_log_distances()

                logger.info("Front Distance:%0.2f",front)

            #Lets first stop the base and then check the color.
            self._stop_walking()
            if colorchecker.is_running():
                logger.info("Stopping color checker thread, not found color yet.")
                colorchecker.stop()

            self.output_inf.buzzer_beep()
            color = self._line_color

        color2 = check_bottom_color(self.output_inf, knowncolor)

        return (color,color2)

    def walk_to_corner(self):
        """Walk to the corner based on the current direction.
        This is used if we have completed our calculation but still not reached corner."""

        #can we check.if one of the sides is not present?, that means we are at a corner.
        (front,left,right) = self.read_log_distances()

        if left + right < 150:
            logger.info("Both sides are present , we have not reached the corner")

            #Lets walk start some more time

            def cond1(_front, _left, _right, _yaw):
                return _left + _right < 150

            self.handle_straight_walk_to_distance(min_front=self.WALLFRONTENDDISTANCE,
                                                    min_left=left,
                                                    min_right=right,
                                                    gyrodefault=0,
                                                    defaultspeed=self.MIN_SPEED,
                                                    precondition=cond1,
                                                    use_mpu=False)

            (front,left,right) = self.read_log_distances()
            logger.info("Both side present walk, left %s,right %s front %s", left, right, front)
        else:
            logger.info("Both sides are not present, we have reached the corner.")

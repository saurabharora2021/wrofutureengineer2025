""" This modules implements the Challenge 1 Walker for the WRO2025 Robot."""
import logging
import time
from typing import Callable, Optional, Tuple
from typing import NamedTuple
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

        # Cache sensor max values once
        self._left_max = self.output_inf.get_left_distance_max()
        self._right_max = self.output_inf.get_right_distance_max()

        self._nooflaps = nooflaps
        self._intelligence: MatIntelligence = MatIntelligence(roundcount=nooflaps,
                                                              hardware_interface=output_inf)

        self._walking:bool = False

     # Small state container and one-read helper
    class RobotState(NamedTuple):
        """Container for the robot state."""
        front: float
        left: float
        right: float
        yaw: float

    def _read_state(self, use_mpu: bool=True) -> "Walker.RobotState":
        front, left, right = self.read_log_distances()
        yaw = self.output_inf.get_orientation()[2] if use_mpu else 0.0
        logger.warning("F:%.2f, L:%.2f, R:%.2f, Y:%.2f , ",
                       front, left, right, yaw)
        return Walker.RobotState(front, left, right, yaw)

    def _center_bot_correction(self,front:float, left:float,
                                right:float) -> Tuple[bool,float,float, float]:
        """Correct the left and right distances to center the bot.
            first argument: bool is corrected value.
            second argument: float is the yaw correction.
            third argument: float is the left distance.
            fourth argument: float is the right distance.
        """
        # If Delta is high move towards the center, move by 10cm otherwise too high correction.

        deltadistance = right - left
        delta_yaw = 0
        # If Delta is high move towards the center, move by 10cm otherwise too high correction.
        if abs(deltadistance)> 20 or right <= 20 or left <= 20:
            correction = 10 if front > 130 else 5
            yaw_correction = 1.5 if front > 130 else 2
            if left < right:
                logger.info("Adjusting left distance, moving to right")
                left += correction
                right -= correction
                delta_yaw = -yaw_correction
            else:
                logger.info("Adjusting right distance")
                left -= correction
                right += correction
                delta_yaw = yaw_correction
            logger.info("adjusted left %.2f , right %.2f, yaw %.2f",left,right,delta_yaw)
            return True,delta_yaw,left, right
        else:
            logger.info("Delta distance is low, not changing distances.")
            return False,delta_yaw,left,right

    def handle_unknowndirection_walk(self):
        """Handle walking when the direction is unknown."""

        logger.info("Direction is unknown, starting the walk with default distances.")

        # Log the start distances
        start_state = self._read_state()

        gyrodefault = start_state.yaw
        totalstartdistance = start_state.left + start_state.right

        is_correction, yaw_delta, start_left_distance, start_right_distance = \
            self._center_bot_correction(start_state.front, start_state.left,
                                         start_state.right)

        (maxfront,_,__) = self._intelligence.get_learned_distances()

        logger.info("Max front distance: %.2f", maxfront)

        self.handle_straight_walk_to_distance(maxfront,start_left_distance,start_right_distance,
                                              gyrodefault+yaw_delta,self.MIN_SPEED,speedcheck=True,
                                              force_change=is_correction)

        #self._stop_walking()
        #return
        #Complete walk to corner , now lets find the color for direction.

        (color,color2) = self.walk_read_mat_color(start_distance=totalstartdistance)

        #ensure we have reached corner.
        self.walk_to_corner()

        state = self._read_state()

        direction = self._decide_direction(color, color2, state.left, state.right)

        if direction != MATDIRECTION.UNKNOWN_DIRECTION:
            self._intelligence.report_direction_side1(direction)
        else:
            return  # unable to determine; stop early

        self.output_inf.buzzer_beep()
        logger.warning("running color: %s", color)
        logger.warning("later color: %s", color2)
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
                                    actual_right:float)-> Tuple[bool,float,float,float]:
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

        #lets start with zero heading.
        self._stop_walking()
        self.output_inf.reset_gyro()

        (min_front,left_def,right_def) = self._intelligence.get_learned_distances()

        current_state = self._read_state()
        def_yaw = current_state.yaw
        current_yaw = def_yaw

        (is_correction, yaw_delta, left_def, right_def) = self._side_bot_centering(
            current_state.front,
            left_def, right_def, current_state.left, current_state.right)

        logger.info("handle side L:%0.2f, R:%0.2f correction %s", left_def,
                    right_def, is_correction)
        self._current_distance = (0, 0)

        def report_distances_side(left: float, right: float):
            logger.info("side Report. Left: %.2f, Right: %.2f", left, right)
            self._current_distance = (left, right)
            # self._stop_walking()

        prev_distance = current_state.left + current_state.right
        prev_left = current_state.left
        prev_right = current_state.right
        def condition_met(_front,left,right,_yaw) -> bool:
            # Define the condition for stopping the walk
            # Either you have found the new minimum point or
            # previous distances are less than current distances by a good margin.
            if self._current_distance != (0, 0):
                logger.info("Condition met with current distance: %s", self._current_distance)
                return False
            if left == 200 or right == 200:
                #lets not try to fix this.
                return True
            if prev_distance - (left + right) < 10:
                #normal case why bother
                return True
            return False

        self._intelligence.register_callback(report_distances_side)

        current_state = self._read_state()
        while current_state.front > min_front:

            logger.info("Starting round handle side...")
            self.handle_straight_walk_to_distance(min_front=min_front,min_left=left_def,
                                                  min_right=right_def,
                                              gyrodefault=current_yaw,
                                              defaultspeed=self.DEFAULT_SPEED,
                                              weak_gyro=False,
                                              keep_walking=condition_met)
            current_state = self._read_state()
            #Either you have found a new less point, or current left and right are
            #  less the previous time to correct and center
            if current_state.front > min_front:
                if self._current_distance != (0, 0):
                    _, yaw_delta, left_def, right_def = self._side_bot_centering(
                                                        current_state.front,
                                                        learned_left=self._current_distance[0],
                                                        learned_right=self._current_distance[1],
                                                        actual_left=current_state.left,
                                                        actual_right=current_state.right)
                    self._current_distance = (0, 0)
                    logger.info("Updated Def distances current distances -"\
                                +" L: %.2f, R: %.2f, Yaw: %.2f",left_def,right_def,current_yaw)
                    prev_distance = current_state.left + current_state.right
                    prev_left = current_state.left
                    prev_right = current_state.right
                    current_yaw = def_yaw + yaw_delta
                elif current_state.left == self._left_max or current_state.right == self._right_max:
                    #we need to ignore this changes and moved ahead.
                    prev_distance = current_state.left + current_state.right
                    prev_left = current_state.left
                    prev_right = current_state.right
                elif prev_distance - (current_state.left + current_state.right) > 10:

                    delta_distance = prev_distance - (current_state.left + current_state.right)
                    logger.info("Significant distance change detected: %.2f", delta_distance)

                    # If the delta is positive, means we are getting straighter or
                    # distance is reducing
                    _, yaw_delta, left_def, right_def = self._side_bot_centering(
                        current_state.front,
                        learned_left=current_state.left,
                        learned_right=current_state.right,
                        actual_left=current_state.left,
                        actual_right=current_state.right
                        )
                    prev_distance = current_state.left + current_state.right
                    prev_left = current_state.left
                    prev_right = current_state.right
                    current_yaw = def_yaw + yaw_delta
                    logger.info("Updated Def dist for positive delta - L: %.2f, R: %.2f, Yaw: %.2f",
                                left_def, right_def, current_yaw)
        # else:
        #     # distance is negative, means we are getting less straight, this is bad news.
        #     # can we somehow correct this.
        #     if current_left >= max_left or current_right >= max_right:
        #         logger.warning("distance increase from prev distance, but one "\
        #                     "side is missing cannot reset.")
        #         prev_distance = current_left + current_right
        #         prev_left = current_left
        #         prev_right = current_right
        #         current_yaw = def_yaw
        #         logger.info("Updated Def distances for negative delta - L: %.2f, R: %.2f",
        #                     left_def, right_def)
        #     else:
        #         if current_left > prev_left and current_right <= prev_right:
        #             #we are drifting right , try to gyro corrections to left.
        #             current_yaw +=1
        #             prev_distance = current_left + current_right
        #             prev_left = current_left
        #             prev_right = current_right
        #             logger.info("Drifing right , we need to reset gyro to handle")
        #             logger.info("Updated Def distances for delta - Yaw: %.2f",current_yaw)
        #         elif current_right > prev_right and current_left <= prev_left:
        #             # we are drifting left, try gyro corrections to right
        #             current_yaw -= 1
        #             prev_distance = current_left + current_right
        #             prev_left = current_left
        #             prev_right = current_right
        #             logger.info("Drifing left , we need to reset gyro to handle")
        #             logger.info("Updated Def distances for delta - Yaw: %.2f",current_yaw)
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
        state = self._read_state()

        def_turn_angle = 0
        current_angle = state.yaw
        logger.info("handle_corner current yaw %.2f",current_angle)
        if current_direction == MATDIRECTION.ANTICLOCKWISE_DIRECTION:
            # lets assume this is AntiClockwise and side1 is complete,
            # we have reached corner1
            if state.right > 50:
                logger.info("Based on current right distance, turn 65")
                def_turn_angle=max(70, 65-current_angle)
            else:
                logger.info("Based on current right distance, turn 55")
                def_turn_angle=max(60, 55-current_angle)
        else:
            if state.left > 50:
                logger.info("Based on current left distance, turn -65")
                def_turn_angle=min(-70, -65-current_angle)
            else:
                logger.info("Based on current left distance, turn -55")
                def_turn_angle=min(-60 , -55-current_angle)
        if self._intelligence.get_round_number() == 1:
            # lets handle the corner walk for round 1.
            # we are going to use the gyro corner walk.
            logger.info("Handling corner walk for round 1 with turn angle: %.2f", def_turn_angle)
            self.gyro_corner_walk_round_1(def_turn_angle=def_turn_angle)
        else:
            # lets handle the corner walk for round 2 onwards.
            # we are going to use the gyro corner walk.
            logger.info("Handling corner walk for round 2 with turn angle: %.2f", def_turn_angle)
            self.gyro_corner_walk_round_n(def_turn_angle=def_turn_angle)
        self.output_inf.buzzer_beep()

        return

    def _handle_walk(self,helper:EquiWalkerHelper,current_speed:float,use_mpu=False,
                    is_unknown_direction:bool=False,is_corner:bool = False,
                    speedcheck=False) -> Optional[float]:
        """Handle walk using helper""" 

        logger.info("Handling walk with direction: %s",
                        directiontostr(self._intelligence.get_direction()))

        state = self._read_state()

        #TODO: Handle somewhere else.
        if not is_unknown_direction:
            (_,left_def,right_def) = self._intelligence.get_learned_distances()

            if left_def == -1:
                state.left = self._left_max

            if right_def == -1:
                state.right = self._right_max

        if is_corner:
            logger.info("Special corner handling")
            if state.left >= self._left_max:
                logger.info("At corner, adjusting steering.")
                state.left -= 10
        current_angle = 0
        if use_mpu:
            current_angle = state.yaw

        turn_angle = helper.walk_func(
                            left_distance=state.left,
                             right_distance=state.right, current_angle=current_angle)

        self._turn_steering_with_logging(turn_angle,speedcheck=speedcheck,
                                         current_speed=current_speed)
        return turn_angle

    def _turn_steering_with_logging(self,turn_angle,current_speed:float,delta_angle:float=0,
                                    speedcheck:bool=False,
                                    max_turn_angle:float=MAX_ANGLE):
        if turn_angle is None:
            logger.info("Turn angle is None")
            if speedcheck:
                self._start_walking(current_speed)
            return

        current_steering_angle = self.output_inf.get_steering_angle()
        max_delta_angle = self.DELTA_ANGLE if delta_angle == 0 else delta_angle

        #we should restrict the delta angle to DELTA_ANGLE
        logger.info("turn angle before delta: %.2f , steering %.2f", \
                    turn_angle,current_steering_angle)

        delta = turn_angle - current_steering_angle

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


    def read_log_distances(self)->tuple[float, float, float]:
        """Read the distances from the log."""
        (front_distance,left_distance,right_distance) = self.output_inf.logdistances()

        self._intelligence.add_readings(front_distance, left_distance, right_distance)
        return (front_distance, left_distance, right_distance)

    def gyro_corner_walk_round_1(self,def_turn_angle:float):
        """Round 1 corner walk using generic routine."""
        min_left, min_right = (30, 20) if self._intelligence.get_direction() \
                                    == MATDIRECTION.CLOCKWISE_DIRECTION else (20, 30)
        self._gyro_corner_walk(def_turn_angle, min_left, min_right)

    def gyro_corner_walk_round_n(self,def_turn_angle:float):
        """Round 2+ corner walk using generic routine."""
        location = self._intelligence.next_location()
        _, l_left, l_right = self._intelligence.get_learned_distances(location)

        # the default distance to 5 cm higer than learned.
        self._intelligence.reset_current_distance(left=l_left+5, right=l_right+5)
        if l_left + l_right > 80:
            min_left = min_right = 30
        else:
            min_left, min_right = ((25, 20) if self._intelligence.get_direction() \
                                        == MATDIRECTION.CLOCKWISE_DIRECTION else (20, 25))
        self._gyro_corner_walk(def_turn_angle, min_left, min_right)

    def _gyro_corner_walk(self, def_turn_angle: float, min_left: float, min_right: float) -> None:
        """Handle the gyro corner walking logic."""

        logger.info("Gyro corner walk round n initiated with turn angle: %.2f", def_turn_angle)

        self._current_distance = (0, 0)
        self._start_time=0

        def report_distances_corner(left: float, right: float):
            logger.info("corner Report. Left: %.2f, Right: %.2f", left, right)
            prev_distance = self._current_distance
            self._current_distance = (left, right)
            if abs(prev_distance[0]+prev_distance[1] - (left + right)) > 2:
                logger.warning("Significant distance change detected.")
                self._start_time = time.time()
                self._stop_walking()

        # Implement the gyro corner walking logic here
        self.output_inf.reset_gyro()

        gyrohelper: GyroWalkerwithMinDistanceHelper = GyroWalkerwithMinDistanceHelper(
            def_turn_angle=def_turn_angle, min_left=min_left, min_right=min_right,
              hardware=self.output_inf)

        (def_front, _, _) = self._intelligence.get_learned_distances()
        state = self._read_state()

        #we are going to start turning before walking

        turn_angle = gyrohelper.walk_func(current_angle=state.yaw,
                                              left_distance=state.left, right_distance=state.right)
        self._turn_steering_with_logging(turn_angle,delta_angle=15,max_turn_angle=25,
                                             current_speed=self.MIN_SPEED)

        self._start_walking(self.MIN_SPEED)
        turned = False

        turn_max_delta = abs(def_turn_angle/2)

        while state.front > def_front and (self._start_time == 0):

            state = self._read_state()
            logger.info("Current Yaw: %.2f", state.yaw)
            turn_angle = gyrohelper.walk_func(current_angle=state.yaw,
                                              left_distance=state.left, right_distance=state.right)
            if not turned and abs(state.yaw - def_turn_angle) < turn_max_delta :
                logger.info("Turned achieve lets check distance=====")
                turned = True
                self._intelligence.reset_current_distance()
                self._intelligence.register_callback(report_distances_corner)

            if state.front > def_front and (self._start_time == 0):

                self._turn_steering_with_logging(turn_angle,delta_angle=15,max_turn_angle=20,
                                             current_speed=self.MIN_SPEED)
                time.sleep(0.001)

        if state.front <= def_front:
            logger.error("Too close to wall for front wall.")

        self._stop_walking()
        self._intelligence.location_complete()
        self._intelligence.unregister_callback()

    def _start_walking(self, speed: float):
        """Start the walking movement."""
        if self._walking is False :
            self._walking = True
            self.output_inf.drive_forward(speed)

    def _stop_walking(self):
        """Stop the walking movement."""
        self._walking = False
        self.output_inf.drive_stop()

    def handle_straight_walk_to_distance(self,min_front:float,min_left:float,min_right:float,
                                         gyrodefault:float,defaultspeed:float,speedcheck:bool=True,
                                         keep_walking:Optional[Callable[[float,float,float,float],
                                                                            bool]]=None,
                                         use_mpu=True,force_change:bool=False,
                                         weak_gyro=False,
                                         base_helper: Optional[EquiWalkerHelper] = None) ->  None:
        """Handle the straight walking logic.
           keep_walking: Callable Function with argument as (front,left,right),yaw
        """
        logger.info("Straight walk initiated with min distances - Front: %.2f, Left: %.2f,"+
                     " Right: %.2f, Gyro: %.2f",min_front, min_left, min_right, gyrodefault)


        helper: EquiWalkerHelper = self._make_equi_helper(
            min_left=min_left,
            min_right=min_right,
            gyrodefault=gyrodefault,
            weak_gyro=weak_gyro,
            force_change=force_change,
            base_helper=base_helper
        )

        state = self._read_state(use_mpu=use_mpu)

        if keep_walking is None:
            def noopcond(_front: float, _left: float, _right: float, _yaw: float) -> bool:
                return True

            keep_walking = noopcond

        self._start_walking(defaultspeed)

        while state.front > min_front and \
                        keep_walking(state.front, state.left, state.right, state.yaw) is True:

            self._handle_walk(helper,use_mpu=use_mpu,is_unknown_direction=True,
                              speedcheck=speedcheck,current_speed=defaultspeed)
            state = self._read_state()

            time.sleep(0.005)

        self.output_inf.buzzer_beep()

    def walk_read_mat_color(self, start_distance: float) -> Tuple[str|None,str|None]:
        """Read the bottom color using the mat_color function."""

        self._stop_walking()

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
                                                        kgyro=-4.0,
                                                        kp=-3.0
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

            state = self._read_state()
            colorchecker.start()

            running=False

            while (state.front > self.WALLFRONTENDDISTANCE
                and self._line_color is None):

                turn_angle = gyrohelper.walk_func(left_distance=state.left,
                                                  right_distance=state.right,
                                                  current_angle=state.yaw)

                self._turn_steering_with_logging(turn_angle,current_speed=self.MIN_SPEED)
                if running is False:
                    self._start_walking(self.MIN_SPEED)
                    running = True

                state = self._read_state()

                logger.info("Front Distance:%0.2f",state.front)

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
        state = self._read_state()

        if state.left + state.right < 150:
            logger.info("Both sides are present , we have not reached the corner")

            #Lets walk start some more time

            def cond1(_front, _left, _right, _yaw):
                return _left + _right < 150

            self.handle_straight_walk_to_distance(min_front=self.WALLFRONTENDDISTANCE,
                                                    min_left=state.left,
                                                    min_right=state.right,
                                                    gyrodefault=0,
                                                    defaultspeed=self.MIN_SPEED,
                                                    keep_walking=cond1,
                                                    use_mpu=False)

            state = self._read_state()
            logger.info("Both side present walk")
        else:
            logger.info("Both sides are not present, we have reached the corner.")

    def _make_equi_helper(self, min_left: float, min_right: float, gyrodefault: float,
                        weak_gyro: bool=False, force_change: bool=False,
                        base_helper: Optional[EquiWalkerHelper]=None) -> EquiWalkerHelper:
        """Factory for EquiWalkerHelper with consistent tuning."""
        if base_helper is not None:
            return base_helper

        left_max = self._left_max
        right_max = self._right_max

        kwargs = dict(
            def_distance_left=min_left,
            def_distance_right=min_right,
            max_left_distance=left_max,
            max_right_distance=right_max,
            def_turn_angle=gyrodefault,
            hardware=self.output_inf,
        )

        if force_change:
            kwargs.update(kp=-3.0, fused_distance_weight=0.3)
        elif weak_gyro:
            kwargs.update(fused_gyro_weight=0.4, kgyro=-4.0, fused_distance_weight=0.5)

        return EquiWalkerHelper(**kwargs)

    def _decide_direction(self, color: str|None, color2: str|None,
                          left: float, right: float) -> MATDIRECTION:
        """Decide direction using color votes and distance hints."""
        logger.info("Deciding direction colors: %s, %s and L=%.2f, R=%.2f",
                     color, color2, left, right)
        direction_hints = (
            MATDIRECTION.ANTICLOCKWISE_DIRECTION if left > right
            else MATDIRECTION.CLOCKWISE_DIRECTION if right > left
            else MATDIRECTION.UNKNOWN_DIRECTION
        )

        directioncolor = color_to_direction(color)
        directioncolor2 = color_to_direction(color2)
        voted = vote_directions([directioncolor, directioncolor2, direction_hints])

        if voted != MATDIRECTION.UNKNOWN_DIRECTION:
            return voted

        logger.info("No clear direction using fallback.")
        # Fallback precedence: hints, then first color, else unknown
        if direction_hints != MATDIRECTION.UNKNOWN_DIRECTION:
            logger.info("Using direction hints: %s", direction_hints)
            return direction_hints
        if directioncolor != MATDIRECTION.UNKNOWN_DIRECTION:
            logger.info("Using first color direction: %s", directioncolor)
            return directioncolor
        return MATDIRECTION.UNKNOWN_DIRECTION

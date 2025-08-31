""" This modules implements the Challenge 1 Walker for the WRO2025 Robot."""
import logging
import time
from typing import Callable, Optional, Tuple, List
from hardware.hardware_interface import HardwareInterface
from hardware.robotstate import RobotState
from round1.distance_function import DistanceCalculator
from round1.walker_helpers import EquiWalkerHelper, GyroWalkerwithMinDistanceHelper
from round1.utilityfunctions import clamp_angle, check_bottom_color
from round1.matintelligence import MatIntelligence
from utils.threadingfunctions import ConditionCheckerThread
from utils import constants
from utils.mat import MATDIRECTION, MATGENERICLOCATION, MATLOCATION
from utils.mat import color_to_direction,vote_directions,locationtostr,directiontostr


logger = logging.getLogger(__name__)
class Walker:
    """This class implements the Challenge 1 Walker for the WRO2025 Robot."""
    # Constants for the walker
    DEFAULT_SPEED=45
    WALK_TO_CORNER_SPEED = 30
    MIN_SPEED= 15
    WALLFRONTFORWALL=60.0
    WALLFRONTENDDISTANCE=30.0
    KNOWN_COLORS = ("blue", "orange")
    MAX_ANGLE = 20
    MAX_STEERING_ANGLE = 24.4
    MAX_DELTA_ANGLE = 8
    YAW_CORRECTION = 0.75
    DISTANCE_CORRECTION = 4

    CORNER_YAW_ANGLE = 57.0
    RECOMENDED_CORNER_STEERING = 18.0

    output_inf: HardwareInterface

    def __init__(self, output_inf:HardwareInterface,nooflaps:int=1):
        self.output_inf = output_inf
        self._line_color: str|None = None
        #setting decimals for float datatype.
        self._current_distance = (0.1, 0.1)

        self._nooflaps = nooflaps
        self.intelligence: MatIntelligence = MatIntelligence(roundcount=nooflaps,
                                                              hardware_interface=output_inf)

        self._walking:bool = False
        self._prev_turn_angle = -99999
        self.distance_calculator = DistanceCalculator()

    def read_state_side(self,camera_read:bool=False) -> RobotState:
        """Read the current state of the robot."""

        state:RobotState  = self.output_inf.read_state()
        reset_state=False
        self.intelligence.add_readings(state.front, state.left, state.right)
        if camera_read is True:
            left = state.left
            right = state.right
            front = state.front
            if state.camera_left > 0 and state.camera_left < state.left:
                #set left using camera left
                left = state.camera_left
                reset_state = True
                logger.info("Using camera left distance: %.2f", left)
            if state.camera_right > 0 and state.camera_right < state.right:
                #set right using camera right
                right = state.camera_right
                reset_state = True
                logger.info("Using camera right distance: %.2f", right)

            direction = self.intelligence.get_direction()
            if direction == MATDIRECTION.CLOCKWISE_DIRECTION and state.left > 75 and \
                            state.camera_front > 0 and state.camera_front < front :
                #set front using camera front
                front = state.camera_front
                reset_state = True
                logger.info("Using camera front distance: %.2f", front)
            elif direction == MATDIRECTION.ANTICLOCKWISE_DIRECTION and state.right > 75 and \
                            state.camera_front > 0 and state.camera_front < front :
                #set front using camera front
                front = state.camera_front
                reset_state = True
                logger.info("Using camera front distance: %.2f", front)

            state = RobotState(front=front, left=left, right=right,camera_front=state.camera_front,\
                               camera_left=state.camera_left,camera_right=state.camera_right, \
                                yaw=state.yaw)

        if reset_state:
            logger.warning("reset state F:%.2f, L:%.2f, R:%.2f, Y:%.2f, CF:%.2f, CL:%.2f, CR:%.2f",
                state.front, state.left, state.right, state.yaw,
                state.camera_front, state.camera_left, state.camera_right)
        else:
            logger.warning("F:%.2f, L:%.2f, R:%.2f, Y:%.2f, CF:%.2f, CL:%.2f, CR:%.2f",
                state.front, state.left, state.right, state.yaw,
                state.camera_front, state.camera_left, state.camera_right)
        return state

    def read_state_corner(self,camera_read:bool=True) -> RobotState:
        """Read the current state of the robot."""

        state:RobotState  = self.output_inf.read_state()
        reset_state=False
        self.intelligence.add_readings(state.front, state.left, state.right)
        if camera_read is True:
            left = state.left
            right = state.right
            front = state.front


            direction = self.intelligence.get_direction()
            if direction == MATDIRECTION.CLOCKWISE_DIRECTION and state.camera_right > 0 \
                  and state.camera_right < state.right:
                #set right using camera right
                right = state.camera_right
                reset_state = True
                logger.info("Using camera right distance: %.2f", right)

            elif direction == MATDIRECTION.ANTICLOCKWISE_DIRECTION and state.camera_left > 0 \
                            and state.camera_left < state.left:
                #set left using camera left
                left = state.camera_left
                reset_state = True
                logger.info("Using camera left distance: %.2f", left)

            state = RobotState(front=front, left=left, right=right,camera_front=state.camera_front,\
                               camera_left=state.camera_left,camera_right=state.camera_right, \
                                yaw=state.yaw)

        if reset_state:
            logger.warning("reset state F:%.2f, L:%.2f, R:%.2f, Y:%.2f, CF:%.2f, CL:%.2f, CR:%.2f",
                state.front, state.left, state.right, state.yaw,
                state.camera_front, state.camera_left, state.camera_right)
        else:
            logger.warning("F:%.2f, L:%.2f, R:%.2f, Y:%.2f, CF:%.2f, CL:%.2f, CR:%.2f",
                state.front, state.left, state.right, state.yaw,
                state.camera_front, state.camera_left, state.camera_right)
        return state

    def center_bot_correction(self,front:float, left:float,
                                right:float,prev_yaw:float) -> Tuple[bool,float,float, float]:
        """Correct the left and right distances to center the bot.
            first argument: bool is corrected value.
            second argument: float is the yaw correction.
            third argument: float is the left distance.
            fourth argument: float is the right distance.
        """
        # If Delta is high move towards the center, move by 10cm otherwise too high correction.

        if left == constants.LEFT_DISTANCE_MAX and right < constants.RIGHT_DISTANCE_MAX:
            if self.intelligence.get_direction() == MATDIRECTION.CLOCKWISE_DIRECTION:
                #make sure your are away from the inside wall you can read.
                left = max(25,left)
                return (True, 0, left, right)
            else:
                #Anti clockwise,
                # make sure you are 30 away from outside wall
                right = max(30,right)
                return (True, 0, left, right)

        if right == constants.RIGHT_DISTANCE_MAX and left < constants.LEFT_DISTANCE_MAX:
            if self.intelligence.get_direction() == MATDIRECTION.CLOCKWISE_DIRECTION:
                # make sure you are 30 away from outside wall
                right = max(30,right)
                return (True, 0, left, right)
            else:
                # Anti clockwise,
                # make sure you are away from the inside wall you can read.
                left = max(25,left)
                return (True, 0, left, right)

        delta_yaw = 0
        # If we are very close to either wall turn.
        if right <= 20 or left <= 20:
            correction = self.DISTANCE_CORRECTION #10 if front > 130 else 5
            if left < right:
                logger.info("Adjusting left distance, moving to right")
                left += correction
                right -= correction
                delta_yaw = -self.YAW_CORRECTION
            else:
                logger.info("Adjusting right distance,moving to left")
                left -= correction
                right += correction
                delta_yaw = self.YAW_CORRECTION
            logger.info("adjusted left %.2f , right %.2f, yaw %.2f",left,right,delta_yaw)
            return True,delta_yaw,left, right
        else:
            logger.info("Delta distance is low, not changing distances.")
            return False,prev_yaw,left,right

    def handle_unknowndirection_walk(self):
        """Handle walking when the direction is unknown."""

        logger.info("Direction is unknown, starting the walk with default distances.")

        # Log the start distances
        start_state = self.read_state_side()

        totalstartdistance = start_state.left + start_state.right

        is_correction, yaw_delta, start_left_distance, start_right_distance = \
            self.center_bot_correction(start_state.front, start_state.left,
                                         start_state.right,0)

        (maxfront,_,__) = self.intelligence.get_learned_distances()

        gyrodefault = start_state.yaw + yaw_delta

        self.handle_straight_walk_to_distance(maxfront,start_left_distance,start_right_distance,
                                              gyrodefault,self.MIN_SPEED,speedcheck=True,
                                              force_change=is_correction)

        #Complete walk to corner , now lets find the color for direction.

        (color,color2) = self.walk_read_mat_color(start_distance=totalstartdistance,
                                                  def_turn_angle=gyrodefault)

        #ensure we have reached corner.
        self.walk_to_corner(def_turn_angle=gyrodefault)

        state = self.read_state_side()

        direction = self._decide_direction(color, color2, state.left, state.right)

        if direction != MATDIRECTION.UNKNOWN_DIRECTION:
            self.intelligence.report_direction_side1(direction)
        else:
            return  # unable to determine; stop early

        self.output_inf.buzzer_beep()
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
        if self.intelligence.get_direction() == MATDIRECTION.UNKNOWN_DIRECTION:
            logger.error("Direction is unknown, stopping the walk.")
            self.output_inf.buzzer_beep()
            self.output_inf.led1_red()
            self.output_inf.force_flush_messages()
            return

        #We have detected the directions, and now time to walk knowing the directions.
        gyrodefault = 0
        while self.intelligence.get_round_number()<= self._nooflaps:
            logger.info("Starting walk for location: %s , round: %d",
                         self.intelligence.get_location(), self.intelligence.get_round_number())

            if self.intelligence.get_generic_location() == MATGENERICLOCATION.CORNER:
                # Handle corner.
                self.handle_corner(gyrodefault)
            else:
                #handle SIDE
                gyrodefault = self.handle_side()

        #print the matintelligence
        self.intelligence.print_mat_intelligence()

    def side_bot_centering(self,front:float,learned_left:float,learned_right:float,
                                actual_left:float,actual_right:float,
                                prev_yaw:float,lenient:bool=False)-> Tuple[bool,float,float,float]:
        """ Side bot Centering for handle side function."""
        total_learned = learned_left + learned_right
        total_actual = actual_left + actual_right
        logger.info("Side Bot Centering: Learned L: %.2f, R: %.2f, Actual L: %.2f, R: %.2f",
                    learned_left, learned_right, actual_left, actual_right)
        #if the gap is less , or the learned is still not correct so less than 40.
        #ignore and center the bot
        if lenient:
            if total_learned > 80 and total_learned <105:
                #we can be lenient
                if actual_left >= 30 and actual_right >= 30:
                    return (False, prev_yaw, actual_left, actual_right)
            elif total_learned > 50 and total_learned < 70:
                if actual_left >= 20 and actual_right >= 20:
                    return (False, prev_yaw, actual_left, actual_right)

        if abs(total_learned - total_actual) < 2 or total_learned < 40:
            #forget the difference, center actual an return
            return self.center_bot_correction(front, actual_left, actual_right,prev_yaw)
        else:
            # we need to proportionally reduce actual
            if total_actual<total_learned:
                return self.center_bot_correction(front, actual_left, actual_right,prev_yaw)
            else:

                logger.info("Proportional reduction of actual distances")
                # total actual > total_learned
                # we need to proportionally reduce actual

                if actual_left <= constants.LEFT_DISTANCE_MAX or \
                        actual_right == constants.RIGHT_DISTANCE_MAX:
                    # If we are at the max distance, we need to be careful
                    logger.info("At max distance, adjusting...")
                    if actual_left == constants.LEFT_DISTANCE_MAX and \
                                actual_right != constants.RIGHT_DISTANCE_MAX:
                        #we can adjust the right distance
                        if actual_right > total_learned/2:
                            actual_right -= 5
                            prev_yaw = -self.YAW_CORRECTION
                    elif actual_right == constants.RIGHT_DISTANCE_MAX and \
                                        actual_left != constants.LEFT_DISTANCE_MAX:
                        #we can adjust the left distance
                        if actual_left > total_learned/2:
                            actual_left -= 5
                            prev_yaw = +self.YAW_CORRECTION
                    logger.info("Adjusted distances: L: %.2f, R: %.2f Y: %.2f",
                                actual_left, actual_right, prev_yaw)
                    return (True, prev_yaw, actual_left, actual_right)

                diff = min(total_actual - total_learned,10)
                if actual_left> actual_right:
                    actual_left -= diff
                else:
                    actual_right -= diff

                if actual_left > 10 and actual_right > 10:
                    return self.center_bot_correction(front, actual_left, actual_right,prev_yaw)
                else:
                    logger.warning("Actual distances are too low after correction,"
                                   + " returning without correction.")
                    return (False,prev_yaw,actual_left, actual_right)

    def handle_side(self,gyroreset:bool = True,def_yaw:float = 0)->float:
        """Handle side walk
        returns the final yaw to be used.
        """

        # we are planning to straight the robot and then do a gyro walk.

        if gyroreset:
            #lets start with zero heading.
            self.stop_walking()
            self.output_inf.reset_gyro()

        (min_front,left_def,right_def) = self.intelligence.get_learned_distances()

        # Get the steering if it more than +-5, reduce it .
        steering = self.output_inf.get_steering_angle()

        if abs(steering) > 5:
            steering = 5 if steering > 0 else -5
            self.output_inf.turn_steering(steering)

        current_state = self.read_state_side(camera_read=True)

        if gyroreset:
            def_yaw = current_state.yaw

        (is_correction, yaw_delta, left_def, right_def) = self.side_bot_centering(
            current_state.front,
            left_def, right_def, current_state.left, current_state.right,0)
        logger.info("handle side: correction: %s, Y: %.2f, L def: %.2f, R def: %.2f",
                    is_correction, yaw_delta, left_def, right_def)

        current_yaw = def_yaw + yaw_delta

        self._current_distance = (0, 0)

        def report_distances_side(left: float, right: float):
            logger.info("side Report. Left: %.2f, Right: %.2f", left, right)
            self._current_distance = (left, right)

        prev_distance = current_state.left + current_state.right
        def condition_met(state:RobotState) -> bool:
            # Define the condition for stopping the walk
            # Either you have found the new minimum point or
            # previous distances are less than current distances by a good margin.
            if self._current_distance != (0, 0):
                logger.info("Condition met with current distance: %s", self._current_distance)
                return False
            # if state.left == self._left_max or state.right == self._right_max:
            # lets not try to fix this.
            # return True
            if prev_distance - (state.left + state.right) > 10:
            #   distance is reducing.
                return False
            return True

        self.intelligence.register_callback(report_distances_side)

        current_state = self.read_state_side(camera_read=True)
        while current_state.front > min_front:

            logger.info("Starting round handle side...")
            current_state = self.handle_straight_walk_to_distance(min_front=min_front,\
                                                                  min_left=left_def,
                                                  min_right=right_def,
                                              gyrodefault=current_yaw,
                                              defaultspeed=self.DEFAULT_SPEED,
                                              weak_gyro=False,
                                              keep_walking=condition_met,
                                              camera_read=True)
            #Either you have found a new less point, or current left and right are
            #  less the previous time to correct and center
            if current_state.front > min_front:
                if self._current_distance != (0, 0):
                    delta = (left_def + right_def) - \
                        (self._current_distance[0]+self._current_distance[1])
                    if delta > 4: #delta should be greater than 4cm to consider
                        _, yaw_delta, left_def, right_def = self.side_bot_centering(
                                                        current_state.front,
                                                        learned_left=self._current_distance[0],
                                                        learned_right=self._current_distance[1],
                                                        actual_left=current_state.left,
                                                        actual_right=current_state.right,
                                                        prev_yaw=yaw_delta,
                                                        lenient=True
                                                    )
                    else:
                        logger.info("new current distance is too close to current def ignoring...")
                    self._current_distance = (0, 0)
                    logger.info("Updated Def distances current distances -"\
                                +" L: %.2f, R: %.2f, Yaw: %.2f",left_def,right_def,current_yaw)
                    prev_distance = current_state.left + current_state.right
                    current_yaw = def_yaw + yaw_delta
                elif current_state.left == constants.LEFT_DISTANCE_MAX or \
                            current_state.right == constants.RIGHT_DISTANCE_MAX:
                    #we need to ignore this changes and moved ahead.
                    prev_distance = current_state.left + current_state.right
                elif prev_distance - (current_state.left + current_state.right) > 10:

                    delta_distance = prev_distance - (current_state.left + current_state.right)
                    logger.info("Significant distance change detected: %.2f", delta_distance)

                    # If the delta is positive, means we are getting straighter or
                    # distance is reducing
                    _, yaw_delta, left_def, right_def = self.side_bot_centering(
                                                    current_state.front,
                                                    learned_left=current_state.left,
                                                    learned_right=current_state.right,
                                                    actual_left=current_state.left,
                                                    actual_right=current_state.right,
                                                    prev_yaw=yaw_delta,
                                                    lenient=True
                                                    )
                    prev_distance = current_state.left + current_state.right
                    current_yaw = def_yaw + yaw_delta
                    logger.info("Updated Def dist for positive delta - L: %.2f, R: %.2f, Yaw: %.2f",
                                left_def, right_def, current_yaw)
            else:
                logger.info("Completed round handle side...")

        self.stop_walking()
        self.intelligence.location_complete()
        self.intelligence.unregister_callback()
        return current_yaw

    def handle_corner(self,gyrodefault:float):
        """Handle corner walk"""

        #gyrodefault is needed for walk to corner, later not used.
        self.walk_to_corner(gyrodefault)
        current_direction = self.intelligence.get_direction()
        state = self.read_state_side()

        def_turn_angle = 0
        current_angle = state.yaw
        logger.info("handle_corner current yaw %.2f",current_angle)
        if current_direction == MATDIRECTION.ANTICLOCKWISE_DIRECTION:
            # lets assume this is AntiClockwise and side1 is complete,
            # we have reached corner1
            if state.right > 40:
                logger.info("Based on current right distance, turn -95")
                def_turn_angle=min(-95, 90-current_angle)
            else:
                logger.info("Based on current right distance, turn -90")
                def_turn_angle=max(-90, -90-current_angle)
            if state.front < 70:
                #too close to front wall,add another 5 to turn
                def_turn_angle+=5
        else:
            if state.left < 40:
                logger.info("Based on current left distance, turn 95")
                def_turn_angle=min(95, 90-current_angle)
            else:
                logger.info("Based on current left distance, turn 90")
                def_turn_angle=max(90 , 90-current_angle)
            if state.front < 70:
                #too close to front wall,add another 5 to turn
                def_turn_angle-=5
        if self.intelligence.get_round_number() == 1:
            # lets handle the corner walk for round 1.
            # we are going to use the gyro corner walk.
            logger.info("Handling corner walk for round 1 with turn angle: %.2f", def_turn_angle)
            self.gyro_corner_walk_round_1(def_turn_angle=def_turn_angle)
        else:
            # lets handle the corner walk for round 2 onwards.
            # we are going to use the gyro corner walk.
            logger.info("Handling corner walk for round 2 with turn angle: %.2f", def_turn_angle)
            self.gyro_corner_walk_round_n(def_turn_angle=def_turn_angle,
                                            location=self.intelligence.get_location())
        self.output_inf.buzzer_beep()

        return

    def turn_steering_with_logging(self,turn_angle:float| None,current_speed:float,
                                    delta_angle:float=0,
                                    speedcheck:bool=False,
                                    max_turn_angle:float=MAX_ANGLE):
        """
        Turn the steering with logging.
        """
        if turn_angle is None:
            logger.info("Turn angle is None")
            if speedcheck:
                self.start_walking(current_speed)
            return

        current_steering_angle = self.output_inf.get_steering_angle()
        max_delta_angle = self.MAX_DELTA_ANGLE if delta_angle == 0 else delta_angle

        #we should restrict the delta angle to DELTA_ANGLE
        logger.info("turn angle before delta: %.2f , steering %.2f", \
                    turn_angle,current_steering_angle)

        delta = float(turn_angle) - current_steering_angle

        if abs(delta) > max_delta_angle:
            turn_angle = current_steering_angle + \
                        (max_delta_angle if delta > 0 else -max_delta_angle)
            logger.info("turn angle after delta: %.2f", turn_angle)
        turn_angle = clamp_angle(turn_angle, max_turn_angle)


        if turn_angle==self._prev_turn_angle:
            logger.info("Turn angle is same as previous angle, not turning.")
            self._prev_turn_angle = 0
            return
        else:
            self._prev_turn_angle = turn_angle


        logger.info("Steering from %.2f to %.2f", current_steering_angle, turn_angle)
        if delta >= 0:
            logger.info("Turning right to angle: %.2f", turn_angle)
        else:
            logger.info("Turning left to angle: %.2f", turn_angle)

        # Turn the steering based on the calculated angle
        if (speedcheck and abs(delta) >= self.MAX_DELTA_ANGLE):
            self.start_walking(self.MIN_SPEED)

        self.output_inf.turn_steering(turn_angle)

    def gyro_corner_walk_round_1(self,def_turn_angle:float):
        """Round 1 corner walk using generic routine."""
        logger.info("Gyro corner walk round 1 started..")
        min_left, min_right = (30, 20) if self.intelligence.get_direction() \
                                    == MATDIRECTION.CLOCKWISE_DIRECTION else (20, 30)
        self._gyro_corner_walk(def_turn_angle, min_left, min_right)

    def gyro_corner_walk_round_n(self,def_turn_angle:float,location:MATLOCATION,
                                    gyroreset: bool=True):
        """Round 2+ corner walk using generic routine."""
        logger.info("Gyro corner walk round n started..")

        _, l_left, l_right = self.intelligence.get_learned_distances(location)

        # the default distance to 5 cm higer than learned.
        self.intelligence.reset_current_distance(left=l_left+5, right=l_right+5)

        self._gyro_corner_walk(def_turn_angle, l_left, l_right,gyroreset)

    def _gyro_corner_walk(self, def_turn_angle: float, min_left: float, min_right: float,
                           gyroreset: bool=True) -> None:
        """Handle the gyro corner walking logic."""

        logger.info("Gyro corner walk round n initiated with turn angle: %.2f", def_turn_angle)

        def report_distances_corner(left: float, right: float):
            logger.info("corner Report. Left: %.2f, Right: %.2f", left, right)
            self._current_distance = (left, right)
            #TODO: we should stop for now lets wait.
            self.stop_walking()

        # Implement the gyro corner walking logic here
        if gyroreset:
            self.output_inf.reset_gyro()

        gyrohelper: GyroWalkerwithMinDistanceHelper = GyroWalkerwithMinDistanceHelper(
            max_left_distance=constants.LEFT_DISTANCE_MAX,
            max_right_distance=constants.RIGHT_DISTANCE_MAX,
            def_turn_angle=def_turn_angle, min_left=min_left, min_right=min_right)

        (def_front, _, _) = self.intelligence.get_learned_distances()
        state = self.read_state_corner()

        #we are going to start turning before walking

        turn_angle = gyrohelper.walk_func(current_angle=state.yaw,
                                              left_distance=state.left, right_distance=state.right)
        self.log_data(gyrohelper)
        self.turn_steering_with_logging(turn_angle,delta_angle=35,
                                        max_turn_angle=self.MAX_STEERING_ANGLE,
                                             current_speed=self.MIN_SPEED)

        turned = False

        turn_max_delta = abs(2*def_turn_angle/3)

        self._current_distance = (0, 0)
        logger.info("Starting corner walk... F:%.2f, current distance %s", state.front,
                                                self._current_distance)
        self.distance_calculator.reset()
        self.start_walking(self.MIN_SPEED)

        while state.front > def_front and self._current_distance == (0,0) \
                        and ((gyroreset is True and abs(state.yaw - def_turn_angle) > 1) or \
                             gyroreset is False) and \
                                self.distance_calculator.get_distance() < 200.0:

            state = self.read_state_corner()
            turn_angle = gyrohelper.walk_func(current_angle=state.yaw,
                                              left_distance=state.left, right_distance=state.right)
            self.log_data(gyrohelper)
            if not turned and abs(state.yaw - def_turn_angle) < turn_max_delta :
                logger.info("Turned achieve lets check distance=====")
                turned = True
                self.intelligence.reset_current_distance()
                self.intelligence.register_callback(report_distances_corner)

            if state.front > def_front and self._current_distance == (0,0):

                self.turn_steering_with_logging(turn_angle,delta_angle=15,
                                                 max_turn_angle=self.MAX_STEERING_ANGLE,
                                                 current_speed=self.MIN_SPEED)
                time.sleep(0.002)
        if state.front <= def_front:
            logger.error("Too close to wall for front wall.")

        self.stop_walking()
        logger.info("End corner walk...")

        self.intelligence.location_complete(state)
        self.intelligence.unregister_callback()

    def log_data(self,helper:Optional[EquiWalkerHelper]=None):
        """Log the data from the helper if provided."""
        if helper is not None:
            messages: List[str] = helper.get_log_data()
            messages.append(f"Loc: {locationtostr(self.intelligence.get_location())}, " )
            self.output_inf.add_screen_logger_message(messages)

    def start_walking(self, speed: float):
        """Start the walking movement, public for testing only"""
        if self._walking is False :
            self._walking = True
            self.output_inf.drive_forward(speed)
            self.distance_calculator.run_speed(speed)

    def stop_walking(self):
        """Stop the walking movement."""
        self._walking = False
        self.output_inf.drive_stop()
        self.distance_calculator.stop()

    def handle_straight_walk_to_distance(self,min_front:float,min_left:float,min_right:float,
                                         gyrodefault:float,defaultspeed:float,speedcheck:bool=True,
                                         keep_walking:Optional[Callable[[RobotState], bool]]=None,
                                         force_change:bool=False,
                                         weak_gyro=False,is_unknown_direction=False,
                                         base_helper: Optional[EquiWalkerHelper] = None,
                                         camera_read=False) ->  RobotState:
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

        (_,left_def,right_def) = self.intelligence.get_learned_distances()

        state = self.read_state_side(camera_read)

        if keep_walking is None:
            def noopcond(_state: RobotState) -> bool:
                return True

            keep_walking = noopcond

        turn_angle = self._inner_turn(state, left_def, right_def,
                                           speedcheck, defaultspeed,
                                           is_unknown_direction, helper,lenient=True)

        #we would walk slow if the turn angle exist.
        if turn_angle is None:
            logger.info("No turn angle detected, walking straight faster")
            self.start_walking(defaultspeed)
        else:
            self.start_walking(self.MIN_SPEED)
        state = self.read_state_side(camera_read)

        prev_turn_angle = 0

        while state.front > min_front and \
                        keep_walking(state) is True:

            turn_angle = self._inner_turn(state, left_def, right_def,
                                           speedcheck, defaultspeed,
                                           is_unknown_direction, helper)

            #lets sleep if no angle or same angle as previous.
            if turn_angle is None or prev_turn_angle == turn_angle:
                # time.sleep(0.001)
                prev_turn_angle = 0
            else:
                prev_turn_angle = turn_angle

            state = self.read_state_side(camera_read)
        return state
        # self.output_inf.buzzer_beep()

    def _inner_turn(self,state:RobotState, left_def:float, right_def:float,
                    speedcheck:bool, defaultspeed:float,is_unknown_direction:bool,
                    helper: EquiWalkerHelper,lenient:bool=False):
        current_left = state.left
        current_right = state.right

        if not is_unknown_direction:
            if left_def == -1:
                current_left = constants.LEFT_DISTANCE_MAX

            if right_def == -1:
                current_right = constants.RIGHT_DISTANCE_MAX

        turn_angle = helper.walk_func(
                        left_distance=current_left,
                            right_distance=current_right, current_angle=state.yaw)

        self.log_data(helper)

        if lenient is True and turn_angle is not None:
            turn_angle = turn_angle * 0.5

        self.turn_steering_with_logging(turn_angle,speedcheck=speedcheck,
                                        current_speed=defaultspeed)
        return turn_angle

    def walk_read_mat_color(self, start_distance: float, def_turn_angle: float) -> \
                                                                Tuple[str|None,str|None]:
        """Read the bottom color using the mat_color function."""

        self.stop_walking()

        logger.info("Time to check color")
        #lets reduce the gyro correction
        def_turn_angle = def_turn_angle * 0.5

        min_left = 40
        min_right = 40

        if start_distance < 70:
            min_left = 20
            min_right = 20

        color = check_bottom_color(self.output_inf, list(self.KNOWN_COLORS))

        if color is None:

            gyrohelper:GyroWalkerwithMinDistanceHelper = GyroWalkerwithMinDistanceHelper(
                                                def_turn_angle=def_turn_angle,
                                                min_left=min_left,
                                                min_right=min_right,
                                                kgyro=-4.0,
                                                kp=-3.0,
                                                max_left_distance=constants.LEFT_DISTANCE_MAX,
                                                max_right_distance=constants.RIGHT_DISTANCE_MAX
                                            )

            def set_line_color(c):
                self._line_color = c
                logger.info("Found Color: %s", c)
                self.stop_walking()

            def value_check_func():
                return check_bottom_color(self.output_inf, list(self.KNOWN_COLORS))

            colorchecker: ConditionCheckerThread = ConditionCheckerThread(
                value_check_func=value_check_func,
                callback_func=set_line_color,
                interval_ms=20
            )


            colorchecker.start()

            try:
                state = self.read_state_side()
                self._line_color = None
                logger.info("Start color walk")
                self.start_walking(self.MIN_SPEED)
                while (state.front > self.WALLFRONTENDDISTANCE
                                    and self._line_color is None):

                    turn_angle = gyrohelper.walk_func(left_distance=state.left,
                                                  right_distance=state.right,
                                                  current_angle=state.yaw)
                    self.log_data(gyrohelper)

                    self.turn_steering_with_logging(turn_angle,current_speed=self.MIN_SPEED)
                    time.sleep(0.005)
                    state = self.read_state_side()

            finally:
                #Lets first stop the base and then check the color.
                self.stop_walking()
                if colorchecker.is_running():
                    logger.info("Stopping color checker thread, not found color yet.")
                    colorchecker.stop()

            logger.info("Completed color walk")

            self.output_inf.buzzer_beep()
            color = self._line_color

        color2 = check_bottom_color(self.output_inf, list(self.KNOWN_COLORS))

        return (color,color2)

    def walk_to_corner(self, def_turn_angle: float):
        """Walk to the corner based on the current direction.
        This is used if we have completed our calculation but still not reached corner."""

        #can we check.if one of the sides is not present?, that means we are at a corner.
        state = self.read_state_side()

        lidar_left = self.output_inf.get_left_lidar_distance()
        lidar_right = self.output_inf.get_right_lidar_distance()

        logger.info("Front Lidar L:%.2f,R:%.2f", lidar_left, lidar_right)
        if lidar_left + lidar_right < 150:
            logger.info("Both sides are present , we have not reached the corner, walk...")

            #Lets walk start some more time

            def cond1(state:RobotState):
                return state.left + state.right < 150 or \
                    (self.output_inf.get_left_lidar_distance() + \
                      self.output_inf.get_right_lidar_distance() < 150)

            direction = self.intelligence.get_direction()

            if direction == MATDIRECTION.CLOCKWISE_DIRECTION:
                #only look at left wall
                state = self.handle_straight_walk_to_distance(min_front=self.WALLFRONTFORWALL,
                                                    min_left=state.left,
                                                    min_right=200,
                                                    gyrodefault=def_turn_angle,
                                                    defaultspeed=self.MIN_SPEED,
                                                    keep_walking=cond1)

            elif direction == MATDIRECTION.ANTICLOCKWISE_DIRECTION:
                # only look at right wall
                state = self.handle_straight_walk_to_distance(min_front=self.WALLFRONTFORWALL,
                                                            min_left=200,
                                                            min_right=state.right,
                                                            gyrodefault=def_turn_angle,
                                                            defaultspeed=self.MIN_SPEED,
                                                            keep_walking=cond1)
            else:
                state = self.handle_straight_walk_to_distance(min_front=self.WALLFRONTFORWALL,
                                                    min_left=state.left,
                                                    min_right=state.right,
                                                    gyrodefault=def_turn_angle,
                                                    defaultspeed=self.MIN_SPEED,
                                                    keep_walking=cond1)
        else:
            logger.info("Both sides are not present, we have reached the corner.")

    def _make_equi_helper(self, min_left: float, min_right: float, gyrodefault: float,
                        weak_gyro: bool=False, force_change: bool=False,
                        base_helper: Optional[EquiWalkerHelper]=None) -> EquiWalkerHelper:
        """Factory for EquiWalkerHelper with consistent tuning."""
        if base_helper is not None:
            return base_helper

        if force_change is False:
            if weak_gyro is False:

                helper:EquiWalkerHelper = EquiWalkerHelper(
                    def_distance_left=min_left,
                    def_distance_right=min_right,
                    max_left_distance=constants.LEFT_DISTANCE_MAX,
                    max_right_distance=constants.RIGHT_DISTANCE_MAX,
                    def_turn_angle=gyrodefault,
                )
            else:
                #weak Gyro is true. use less of gyro weight
                helper:EquiWalkerHelper = EquiWalkerHelper(
                    def_distance_left=min_left,
                    def_distance_right=min_right,
                    max_left_distance=constants.LEFT_DISTANCE_MAX,
                    max_right_distance=constants.RIGHT_DISTANCE_MAX,
                    def_turn_angle=gyrodefault,
                    fused_gyro_weight=0.4,
                    kgyro=-4.0,
                    fused_distance_weight=0.5
                )
        else:
            helper:EquiWalkerHelper = EquiWalkerHelper(
                def_distance_left=min_left,
                def_distance_right=min_right,
                max_left_distance=constants.LEFT_DISTANCE_MAX,
                max_right_distance=constants.RIGHT_DISTANCE_MAX,
                def_turn_angle=gyrodefault,
                kp=-3,
                fused_distance_weight=0.4
            )

        return helper

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

""" This modules implements the Challenge 1 Walker for the WRO2025 Robot."""
import logging
from hardware.hardware_interface import HardwareInterface
from hardware.hardware_interface import RobotState
from round1.walker_helpers import FixedTurnWalker,GyroWalkerwithMinDistanceHelper
from round1.logicround1 import Walker
from utils.mat import MATDIRECTION, MATGENERICLOCATION

logger = logging.getLogger(__name__)
class WalkerN(Walker):
    """This class implements the Challenge 1 Walker for the WRO2025 Robot."""
    # Constants for the walker
    DEFAULT_GYRO_SPEED=40
    DEFAULT_FIRST_WALK_SPEED=25
    CORNER_GYRO_SPEED = 30

    CORNER_TURN_ANGLE = 56
    RECOMENDED_CORNER_STEERING = 17

    output_inf: HardwareInterface

    def __init__(self, output_inf:HardwareInterface,nooflaps:int=2):
        super().__init__(output_inf, nooflaps)
        self.output_inf = output_inf
        self._line_color: str|None = None
        #setting decimals for float datatype.
        self._current_distance = (0.1, 0.1)

        # Cache sensor max values once
        self._left_max = self.output_inf.get_left_distance_max()
        self._right_max = self.output_inf.get_right_distance_max()

        self._nooflaps = nooflaps

        self._walking:bool = False

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
            if self.intelligence.get_round_number() == 1:

                if self.intelligence.get_generic_location() == MATGENERICLOCATION.CORNER:
                    # Handle corner.
                    self.handle_corner(gyrodefault)
                else:
                    #handle SIDE
                    gyrodefault = self.handle_side()
            else:
                self.full_gyro_walk()

    def full_gyro_walk(self):
        """Walk the full path using gyro."""
        #We are going to walk the full gyro path

        #First , i would send my center my vehicle on the current side.
        #we would do this at lowest speed.

        logger.info("walk straight for side 1")

        self.handle_side_walk_n(gyroreset=True,
                             speed=self.DEFAULT_FIRST_WALK_SPEED,
                             def_yaw=0)

        corner_turn_angle = self.CORNER_TURN_ANGLE if self.intelligence.get_direction() == \
                                                    MATDIRECTION.ANTICLOCKWISE_DIRECTION \
                                                        else -self.CORNER_TURN_ANGLE

        state = self.read_state()
        logger.info("Full gyro walk current state: %s", state)
        current_angle = state.yaw

        self.stop_walking()

        # for i in range(1, 0):
        #     #lets walk the corner at 90 degrees corner1
        #     current_angle += corner_turn_angle
        #     self.handle_gyro_corner(
        #         recommended_angle=self.RECOMENDED_CORNER_STEERING,
        #         final_yaw=current_angle,
        #         current_speed=self.CORNER_GYRO_SPEED
        #     )

        #     #go to side 1-2-3
        #     self._gyro_only_walk(def_turn_angle=current_angle)

        # #go to corner 4
        # current_angle += corner_turn_angle
        # self._gyro_only_walk(def_turn_angle=current_angle ,
        #                             gyroreset=False)

    def _gyro_only_walk(self, def_turn_angle: float,
                           gyroreset: bool=True,
                            speed: float=DEFAULT_GYRO_SPEED) -> None:
        """Handle the gyro corner walking logic."""

        logger.info("Gyro walk round n initiated with turn angle: %.2f", def_turn_angle)

        # Implement the gyro corner walking logic here
        if gyroreset:
            self.output_inf.reset_gyro()

        (def_front, min_left, min_right) = self.intelligence.get_learned_distances()
        state = self.read_state()

        #we would try to create a band of 14cm in the mid for walking.
        min_left = min_left - 7
        min_right = min_right - 7

        gyrohelper: GyroWalkerwithMinDistanceHelper = GyroWalkerwithMinDistanceHelper(
            max_left_distance=self._left_max,max_right_distance=self._right_max,
            def_turn_angle=def_turn_angle, min_left=min_left, min_right=min_right)

        #we are going to start turning before walking

        turn_angle = gyrohelper.walk_func(current_angle=state.yaw,
                                              left_distance=state.left, right_distance=state.right)
        self.turn_steering_with_logging(turn_angle,delta_angle=5,
                                        max_turn_angle=self.MAX_STEERING_ANGLE,
                                             current_speed=self.DEFAULT_GYRO_SPEED)

        self._current_distance = (0, 0)
        logger.info("Starting corner walk... F:%.2f, current distance %s", state.front,
                                                self._current_distance)
        self._start_walking(speed)

        while  state.front > def_front :

            state = self.read_state()
            logger.info("Current Yaw: %.2f", state.yaw)
            turn_angle = gyrohelper.walk_func(current_angle=state.yaw,
                                              left_distance=state.left, right_distance=state.right)

            state = self.read_state()

            if state.front > def_front:

                #we are going to turn the steering
                self.turn_steering_with_logging(turn_angle,delta_angle=5,
                                                max_turn_angle=self.MAX_STEERING_ANGLE,
                                            current_speed=self.RECOMENDED_CORNER_STEERING,
                                            speedcheck=True)


        if state.front <= def_front:
            logger.error("Too close to wall for front wall.")

        logger.info("End _gyro_only_walk...")

        self.intelligence.location_complete()
        self.intelligence.unregister_callback()

    def handle_gyro_corner(self,recommended_angle:float,final_yaw:float,current_speed:float):
        """Handle the gyro cornering logic."""


        logger.info("Starting gyro corner walk to angle: %.2f, final yaw: %.2f at speed: %.2f",
                    recommended_angle, final_yaw, current_speed)

        fixedturnwalker: FixedTurnWalker = FixedTurnWalker(max_left_distance=200,
                                                               max_right_distance=200,
                                                               fixed_turn_angle=recommended_angle)

        state: RobotState = self.output_inf.read_state()

        self.output_inf.drive_forward(current_speed)

        while abs(state.yaw - final_yaw) > 0.5 and \
                (state.left == 200 or state.right == 200):

            turn_angle = fixedturnwalker.walk_func(
                left_distance=state.left,
                right_distance=state.right,
                current_angle=state.yaw
            )

            self.turn_steering_with_logging(turn_angle,current_speed=current_speed)
            state = self.output_inf.read_state()

        logger.info("Completed gyro corner walk to yaw: %.2f", state.yaw)

        self.intelligence.location_complete()

    def handle_side_walk_n(self,gyroreset:bool,def_yaw:float,
                         speed:float)->float:
        """Handle side walk
        returns the final yaw to be used.
        """
        if gyroreset:
            #lets start with zero heading.
            self.stop_walking()
            self.output_inf.reset_gyro()

        # Get the steering if it more than +-5, reduce it .
        steering = self.output_inf.get_steering_angle()

        if abs(steering) > 5:
            steering = 5 if steering > 0 else -5
            self.output_inf.turn_steering(steering)

        (min_front,left_def,right_def) = self.intelligence.get_learned_distances()

        current_state = self.read_state()

        if gyroreset:
            def_yaw = current_state.yaw

        (is_correction, yaw_delta, left_def, right_def) = self.side_bot_centering(
            current_state.front,
            left_def, right_def, current_state.left, current_state.right,0)
        logger.info("handle_side_walk_n: correction: %s, Y: %.2f, L def: %.2f, R def: %.2f",
                    is_correction, yaw_delta, left_def, right_def)

        current_yaw = def_yaw + yaw_delta

        prev_distance = current_state.left + current_state.right
        def condition_met(state:RobotState) -> bool:
            # Define the condition for stopping the walk
            # Either you have found the new minimum point or
            # previous distances are less than current distances by a good margin.
            if state.left == self._left_max or state.right == self._right_max:
                #lets not try to fix this.
                return True
            if prev_distance - (state.left + state.right) < 10:
                #normal case why bother
                return True
            return False

        current_state = self.read_state()
        while current_state.front > min_front:

            logger.info("Starting round handle side...")
            self.handle_straight_walk_to_distance(min_front=min_front,min_left=left_def,
                                                  min_right=right_def,
                                              gyrodefault=current_yaw,
                                              defaultspeed=speed,
                                              weak_gyro=False,
                                              speedcheck=True,
                                              keep_walking=condition_met)

            current_state = self.read_state()
            #Either you have found a new less point, or current left and right are
            #  less the previous time to correct and center
            if current_state.front > min_front:
                if current_state.left == self._left_max or current_state.right == self._right_max:
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

        self.intelligence.location_complete()
        return current_state.yaw

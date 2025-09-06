""" This modules implements the Challenge 1 Walker for the WRO2025 Robot."""
import logging
from hardware.hardware_interface import HardwareInterface
from hardware.robotstate import RobotState
from round1.walker_helpers import FixedTurnWalker
from round1.logicround1 import Walker
from round1.walker_helpers import WalkParameters
from round1.utilityfunctions import delta_angle_deg
from round1.movement_controller import MAX_STEERING_ANGLE
from utils import constants
import time
from utils.mat import MATDIRECTION, MATGENERICLOCATION ,MATLOCATION

logger = logging.getLogger(__name__)
class WalkerN(Walker):
    """This class implements the Challenge 1 Walker for the WRO2025 Robot."""
    # Constants for the walker
    DEFAULT_GYRO_SPEED=40
    DEFAULT_FIRST_WALK_SPEED=25
    CORNER_GYRO_SPEED = 30


    output_inf: HardwareInterface

    def __init__(self, output_inf:HardwareInterface,nooflaps:int=2):
        super().__init__(output_inf, nooflaps)
        self.output_inf = output_inf
        self._line_color: str|None = None
        #setting decimals for float datatype.
        self._current_distance = (0.1, 0.1)

        self._nooflaps = nooflaps

        self._walking:bool = False
        self._global_yaw = 0.0
        self._cummulative_yaw = 0.0

    def _full_round1_walk(self):

        #this should set the direction
        self.handle_unknowndirection_walk()
        #we don't need camera Now
        self.output_inf.camera_off()

        #we cannot determine direction, beep and stop.
        if self._direction == MATDIRECTION.UNKNOWN_DIRECTION:
            logger.error("Direction is unknown, stopping the walk.")
            self.output_inf.buzzer_beep()
            self.output_inf.led1_red()
            self.output_inf.force_flush_messages()
            return

        corner_yaw_angle = self.CORNER_YAW_ANGLE if self._direction == \
                                                    MATDIRECTION.CLOCKWISE_DIRECTION \
                                                        else -self.CORNER_YAW_ANGLE
        logger.info("Corner yaw angle: %.2f", corner_yaw_angle)

        current_yaw_angle = 0

        #handle first corner without gyroreset.
        try:
            current_state = self.read_state_side()

            if self._direction == MATDIRECTION.CLOCKWISE_DIRECTION:
                #end more distance on right , since right turn
                self.walk_back(state=current_state,minfront=20,minleft=10,minright=30)
            else:
                self.walk_back(state=current_state,minfront=20,minleft=30,minright=10)

            self.output_inf.camera_pause()
            current_yaw_angle =self.handle_corner_round1(current_yaw_angle,False)   
        finally:
            self.output_inf.camera_restart()

        while self.intelligence.get_round_number() == 1:

            generic_location = self.intelligence.get_generic_location()

            if generic_location == MATGENERICLOCATION.CORNER:
                try:
                    self.output_inf.camera_pause()
                    current_yaw_angle =self.handle_corner_round1(current_yaw_angle)
                finally:
                    self.output_inf.camera_restart()
            else:
                current_yaw_angle = self.handle_side(gyroreset=False,def_yaw=current_yaw_angle)

        self.movementcontroller.stop_walking()


    def start_walk(self):
        """Start the walk based on the current direction which is unknown and number of laps."""
        logger.info("Starting to walk...")

        self._full_round1_walk()

        while self.intelligence.get_round_number()<= self._nooflaps:
            logger.info("Starting walk for location: %s , round: %d",
                         self.intelligence.get_location(), self.intelligence.get_round_number())
            self.full_gyro_walk()
            self.movementcontroller.stop_walking()
            return

    def full_gyro_walk(self):
        """Walk the full path using gyro."""
        #We are going to walk the full gyro path

        #First , i would send my center my vehicle on the current side.
        #we would do this at lowest speed.

        logger.info("round n walk straight for side 1")

        state = self.read_state_side()
        current_yaw_angle = state.yaw

        current_yaw_angle = self.handle_side_walk_n(gyroreset=False,
                            def_yaw=current_yaw_angle)


        for i in range(3): # Loop 3 times for the first 3 corners and sides
            #lets walk the corner at 90 degrees corner1
            try:
                self.output_inf.camera_pause()
                current_yaw_angle =self.handle_gyro_corner_round_n(current_yaw_angle)
            finally:
                self.output_inf.camera_restart()

            current_yaw_angle = self.handle_side(gyroreset=False,def_yaw=current_yaw_angle)

        #go to corner 4
        try:
                self.output_inf.camera_pause()
                current_yaw_angle =self.handle_gyro_corner_round_n(current_yaw_angle)
        finally:
                self.output_inf.camera_restart()

        self.movementcontroller.stop_walking()

    def handle_gyro_corner_round_n(self,gyrodefault:float,gyroreset:bool=True):
        """Handle the gyro cornering logic round n."""

        logger.info("handler corner round1: gyrodefault:%.2f, gyroreset: %s",
                    gyrodefault, gyroreset)

        self.walk_to_corner(gyrodefault)
        state = self.read_state_side()

        def_turn_angle = 0.0
        logger.info("handle_corner current yaw %.2f",state.yaw)

        #we are doing -2 since we want to turn slightly less than actual.
        recommended_turn_angle = self.RECOMMENDED_CORNER_STEERING if self._direction == MATDIRECTION.CLOCKWISE_DIRECTION \
                                            else -self.RECOMMENDED_CORNER_STEERING

        self._global_yaw += 90 if self._direction == MATDIRECTION.CLOCKWISE_DIRECTION else -90

        if gyroreset:
            prev_yaw = self.output_inf.reset_gyro()
            self._cummulative_yaw += prev_yaw
        

        #this should be 90 degrees, it takes care in global context        
        corner_yaw_angle = self._global_yaw - self._cummulative_yaw
        logger.info("After gyro reset prev yaw %.2f, global yaw: %.2f, cummulative yaw: %.2f, corner angle: %.2f",
                    prev_yaw,self._global_yaw,self._cummulative_yaw,corner_yaw_angle)

        if self._direction == MATDIRECTION.ANTICLOCKWISE_DIRECTION:
            # lets assume this is AntiClockwise and side1 is complete,
            # we have reached corner1
            def_turn_angle = corner_yaw_angle
            # we should check if we are too close to wall in front or side.
            # turn angle should be increased            
            if state.right < self.CORNER_RIGHT_DIST_THRESHOLD:
                def_turn_angle = corner_yaw_angle - self.CORNER_EXTRA_TURN_ANGLE
                recommended_turn_angle -= self.CORNER_EXTRA_TURN_ANGLE
            if state.front < self.CORNER_FRONT_DIST_THRESHOLD:
                #too close to front wall,add another 5 to turn
                recommended_turn_angle -= self.CORNER_EXTRA_TURN_ANGLE
        else:
            def_turn_angle = corner_yaw_angle
            if state.left < self.CORNER_LEFT_DIST_THRESHOLD:
                def_turn_angle = corner_yaw_angle + self.CORNER_EXTRA_TURN_ANGLE

            if state.front < self.CORNER_FRONT_DIST_THRESHOLD:
                #too close to front wall, increase corner turn angle
                recommended_turn_angle += self.CORNER_EXTRA_TURN_ANGLE
        

        # lets handle the corner walk for round 1.
        # we are going to use the gyro corner walk.
        logger.info("Handling corner walk for round 1 with turn angle: %.2f", def_turn_angle)
        try:
            min_left, min_right = (30, 20) if self._direction \
                                == MATDIRECTION.CLOCKWISE_DIRECTION else (20, 30)
            angle = self._gyro_corner_walk_n(def_turn_angle, min_left, min_right,\
                                          fixed_turn_angle=recommended_turn_angle)
            self.intelligence.location_complete(state)
            self.intelligence.unregister_callback()
            return def_turn_angle
        finally:
            pass

        return

    def _gyro_corner_walk_n(self, def_turn_angle: float, min_left: float, min_right: float,
                          fixed_turn_angle:float,
                            ) -> None:
        """Handle the gyro corner walking logic."""

        logger.info("Gyro corner walk round n initiated with turn angle: %.2f", def_turn_angle)

        def report_distances_corner(left: float, right: float):
            logger.info("corner Report. Left: %.2f, Right: %.2f", left, right)
            self._current_distance = (left, right)
            #TODO: we should stop for now lets wait.
            self.movementcontroller.stop_walking()

        if def_turn_angle > self.CORNER_YAW_ANGLE:
            fixed_turn_angle = MAX_STEERING_ANGLE

        gyrohelper: FixedTurnWalker = FixedTurnWalker(
            max_left_distance=constants.LEFT_DISTANCE_MAX,
            max_right_distance=constants.RIGHT_DISTANCE_MAX,
            fixed_turn_angle=fixed_turn_angle,
            def_turn_angle=def_turn_angle, min_left=min_left, min_right=min_right)

        (def_front, _, _) = self.intelligence.get_learned_distances()
        state = self.read_state_corner()

        #we are going to start turning before walking

        turn_angle = gyrohelper.walk_func(current_angle=state.yaw,
                                              left_distance=state.left, right_distance=state.right)
        self.log_data(gyrohelper)
        self.movementcontroller.turn_steering_with_logging(turn_angle,delta_angle=35,
                                        max_turn_angle=MAX_STEERING_ANGLE,
                                             current_speed=self.MIN_SPEED,
                                            async_turn=True)

        turned = False

        turn_max_delta = abs(2*def_turn_angle/3)

        self._current_distance = (0, 0)
        logger.info("Starting corner walk... F:%.2f, current distance %s", state.front,
                                                self._current_distance)
        self.movementcontroller.reset_distance()
        self.movementcontroller.start_walking(self.MIN_SPEED)

        while state.front > def_front and self._current_distance == (0,0) \
                        and abs(delta_angle_deg(state.yaw, def_turn_angle)) > 1 \
                                and self.movementcontroller.get_distance() < 100.0:

            state = self.read_state_corner()
            turn_angle = gyrohelper.walk_func(current_angle=state.yaw,
                                              left_distance=state.left, right_distance=state.right)

            if not turned and abs(delta_angle_deg(state.yaw,def_turn_angle)) < turn_max_delta:
                logger.info("Turned achieve lets check distance=====")
                turned = True
                self.intelligence.reset_current_distance()
                self.intelligence.register_callback(report_distances_corner)
                self.movementcontroller.start_walking(self.WALK_TO_CORNER_SPEED)

            if state.front > def_front and self._current_distance == (0,0):

                self.movementcontroller.turn_steering_with_logging(turn_angle,delta_angle=15,
                                                 max_turn_angle=MAX_STEERING_ANGLE,
                                                 current_speed=self.MIN_SPEED)
        if state.front <= def_front:
            logger.error("Too close to wall for front wall.")

        self.log_data(gyrohelper)

        #TODO: if second round works, we can remove this stop. so between corner and side 
        # it would not stop
        self.movementcontroller.stop_walking()
        logger.info("End corner : Front:%.2f distance travelled:%.2f", state.front,\
                     self.movementcontroller.get_distance())

        if  state.front <= def_front and abs(delta_angle_deg(state.yaw,def_turn_angle)) < 10:
            #we stopped since we are close to the wall.
            self.walk_back(state,minfront=20,minleft=min_left,minright=min_right)
            #lets try to turn again .
            self._gyro_corner_walk(def_turn_angle,min_left,min_right,fixed_turn_angle+5)


        return def_turn_angle

    def handle_side_walk_n(self,gyroreset:bool = True,def_yaw:float = 0)->float:

        """Handle side walk n
        returns the final yaw to be used.
        """
        logger.info("handle side, Gyro Reset: %s, Default Yaw: %.2f", gyroreset, def_yaw)

        # see read_state_slide, it would use camera if we have travelled atleast 50 cm
        self.movementcontroller.reset_distance()

        current_state = self.read_state_side()

        # Get the steering if it more than +-5, reduce it .
        steering = self.output_inf.get_steering_angle()
        if abs(steering) > 5:
            steering = 5 if steering > 0 else -5
            if current_state.yaw > 85 and current_state.yaw < 95:
                steering = 0
            self.movementcontroller.turn_steering_with_logging(steering,async_turn=True,current_speed=0)

        # we are planning to straight the robot and then do a gyro walk.
        if gyroreset:
            #lets start with zero heading.
            self.movementcontroller.stop_walking()
            self.output_inf.reset_gyro()

        (min_front,left_def,right_def) = self.intelligence.get_learned_distances()

        current_state = self.read_state_side()

        #check if we are too close to wall,
        if self._direction == MATDIRECTION.CLOCKWISE_DIRECTION:
            self.walk_back(current_state,minfront=20,minleft=1,minright=20)
        else:
            self.walk_back(current_state,minfront=20,minleft=20,minright=1)

        current_state = self.read_state_side()

        if gyroreset:
            def_yaw = current_state.yaw

        (is_correction, yaw_delta, left_def, right_def) = self._positioner.side_bot_centering(
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
        last_stop: float = 0.0

        prev_measure_distance = self.movementcontroller.get_distance()

        def condition_met(state:RobotState) -> bool:
            nonlocal last_stop
            nonlocal prev_measure_distance
            if state.left <= 15 or state.right <= 15:
                if (time.time() - last_stop) > 0.5:
                    self.movementcontroller.stop_walking()
                    logger.info("Condition met: very close to wall stopping L: %.2f, R: %.2f",\
                                 state.left, state.right)
                    last_stop = time.time()
                    return False
                else:
                    logger.info("Close to wall but wait for 500ms")

            if abs(state.yaw - def_yaw) > 10 and self.movementcontroller.get_distance() - prev_measure_distance > 20:
                #we have a 10 degree error, lets stop and correct.
                prev_measure_distance = self.movementcontroller.get_distance()
                logger.info("Condition met: very high yaw close to wall stopping Y: %.2f, Def Y: %.2f",\
                             state.yaw, def_yaw)
                return False


            # Define the condition for stopping the walk
            # Either you have found the new minimum point or
            # previous distances are less than current distances by a good margin.
            if self._current_distance != (0, 0):
                delta = (self._current_distance[0] + self._current_distance[1]) - \
                        prev_distance
                if delta > 2: # we should stop if great than 2 cm.
                    logger.info("Condition met with current distance: %s", self._current_distance)
                    return False
                return False
            # if state.left == self._left_max or state.right == self._right_max:
            # lets not try to fix this.
            # return True
            if prev_distance - (state.left + state.right) > 10:
            #   distance is reducing.
                return False
            return True

        self.intelligence.register_callback(report_distances_side)

        counter = 1

        # this outer loop is to ensuer that we walkback and retry till we have 
        # travelled some distance.
        while self.movementcontroller.get_distance() < 100 and counter <= 3:

            counter+=1

            current_state = self.read_state_side()
            while current_state.front > min_front:
                logger.info("Starting side walk iteration...")


                # Create a walk parameters object
                walk_params = WalkParameters(
                    min_front=min_front,
                    def_left=left_def,
                    def_right=right_def,
                    gyro_default=current_yaw,
                    speed=self.DEFAULT_SPEED,
                    weak_gyro=False,
                    min_left=20,
                    min_right=20,
                )

                # Use the new method with parameter object
                current_state = self.handle_straight_walk(
                    params=walk_params,
                    keep_walking=condition_met
                )

                if current_state.front > min_front:
                    is_path_updated, current_yaw, left_def, right_def, prev_distance = \
                        self.update_side_path(current_state, left_def, right_def, yaw_delta,
                                            def_yaw, prev_distance)
                    if is_path_updated:
                        logger.info("Path updated - L: %.2f, R: %.2f, Yaw: %.2f", left_def,
                                    right_def, current_yaw)
                else:
                    logger.info("Completed round handle side...")

            self.movementcontroller.stop_walking()
            self.intelligence.unregister_callback()

            if self._direction == MATDIRECTION.CLOCKWISE_DIRECTION:
                #end more distance on right , since right turn
                self.walk_back(state=current_state,minfront=20,minleft=10,minright=30)
            else:
                self.walk_back(state=current_state,minfront=20,minleft=30,minright=10)

        self.intelligence.location_complete()

        return current_yaw

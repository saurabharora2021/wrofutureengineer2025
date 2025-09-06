""" This modules implements the Challenge 1 Walker for the WRO2025 Robot."""
import logging
import time
from typing import Callable, Optional, Tuple, List
from hardware.hardware_interface import HardwareInterface
from hardware.robotstate import RobotState
from round1.walker_helpers import EquiWalkerHelper, GyroWalkerwithMinDistanceHelper, WalkParameters
from round1.walker_helpers import FixedTurnWalker
from round1.utilityfunctions import check_bottom_color, delta_angle_deg
from round1.matintelligence import MatIntelligence
from round1.botposition import BotPositioner
from round1.movement_controller import MovementController
from round1.movement_controller import MAX_STEERING_ANGLE
from utils.threadingfunctions import ConditionCheckerThread
from utils import constants
from utils.mat import MATDIRECTION,MATGENERICLOCATION
from utils.mat import locationtostr,directiontostr
from utils.mat import decide_direction

logger = logging.getLogger(__name__)

class Walker:
    """This class implements the Challenge 1 Walker for the WRO2025 Robot."""
    # Constants for the walker
    DEFAULT_SPEED=50
    WALK_TO_CORNER_SPEED = 15
    MIN_SPEED= 20
    CORRECTION_SPEED=10
    WALLFRONTFORWALL=60.0
    WALLFRONTENDDISTANCE=60.0
    KNOWN_COLORS = ("blue", "orange")



    YAW_CORRECTION = 0.75
    DISTANCE_CORRECTION = 4.0

    CORNER_YAW_ANGLE = 90.0
    RECOMMENDED_CORNER_STEERING = 20.0

    # New constants for corner handling
    CORNER_RIGHT_DIST_THRESHOLD = 40.0
    CORNER_LEFT_DIST_THRESHOLD = 40.0
    CORNER_FRONT_DIST_THRESHOLD = 70.0
    CORNER_EXTRA_TURN_ANGLE = 4.0

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
        self._prev_turn_angle = -99.0
        self._direction = MATDIRECTION.UNKNOWN_DIRECTION

        self._positioner = BotPositioner(self.intelligence)

        self.movementcontroller = MovementController(output_inf,min_speed = self.MIN_SPEED)

        # Global yaw tracks intended orientation, not affected by gyro resets.
        # This is the intended angle of the robot, 0 is straight, +90 is right, -90 is left.
        self._global_yaw = 0.0
        self._cummulative_yaw = 0.0


    def read_state(self, location_type: MATGENERICLOCATION,camera:bool) -> RobotState:
        """Read the current state of the robot, optionally using camera data."""
        state: RobotState = self.output_inf.read_state()
        self.intelligence.add_readings(state.front, state.left, state.right)

        
        use_camera = False

        if camera is False:
            logger.info("State (Cam:%s): F:%.2f, L:%.2f, R:%.2f, Y:%.2f, CF:%.2f, CL:%.2f, CR:%.2f",
                   use_camera,state.front, state.left, state.right, state.yaw,
                   state.camera_front,state.camera_left,state.camera_right)
            return state

        front, left, right = state.front, state.left, state.right

        if location_type == MATGENERICLOCATION.SIDE:
            if state.camera_left > 0 and state.camera_left < left:
                left = state.camera_left
                use_camera = True
            if state.camera_right > 0 and state.camera_right < right:
                right = state.camera_right
                use_camera = True
            if self._direction == MATDIRECTION.CLOCKWISE_DIRECTION and state.left > 65 \
                                and state.camera_front > 0 and state.camera_front < front:
                front = state.camera_front
                use_camera = True
            elif self._direction == MATDIRECTION.ANTICLOCKWISE_DIRECTION and state.right > 65 \
                    and state.camera_front > 0 and state.camera_front < front:
                front = state.camera_front
                use_camera = True
            else: #Direction unknown
                if state.camera_front > 0 and state.camera_front < front:
                    front = state.camera_front
                    use_camera = True

        elif location_type == MATGENERICLOCATION.CORNER:
            if self._direction == MATDIRECTION.CLOCKWISE_DIRECTION and \
                                            state.camera_right > 0 and state.camera_right < right:
                right = state.camera_right
                use_camera = True
            elif self._direction == MATDIRECTION.ANTICLOCKWISE_DIRECTION \
                                        and state.camera_left > 0 and state.camera_left < left:
                left = state.camera_left
                use_camera = True

        new_state = RobotState(front=front, left=left, right=right, yaw=state.yaw,
                               camera_front=state.camera_front, camera_left=state.camera_left,
                                 camera_right=state.camera_right)

        logger.info("State (Cam:%s): F:%.2f, L:%.2f, R:%.2f, Y:%.2f, CF:%.2f, CL:%.2f, CR:%.2f",
                   use_camera,new_state.front, new_state.left, new_state.right, new_state.yaw,
                   state.camera_front,state.camera_left,state.camera_right)

        return new_state

    def read_state_side(self) -> RobotState:
        """Read the current state of the robot for a side location."""
        # Interesting we would use camera if we have travelled atleast 50cm on a side
        use_camera = True if self.movementcontroller.get_distance() > 50 else False

        return self.read_state(MATGENERICLOCATION.SIDE,use_camera)

    def read_state_corner(self,camera:bool=False) -> RobotState:
        """Read the current state of the robot for a corner location."""
        return self.read_state(MATGENERICLOCATION.CORNER,camera)

    def handle_unknowndirection_walk(self):
        """Handle walking when the direction is unknown."""

        logger.info("Direction is unknown, starting the walk with default distances.")

        # Log the start distances
        start_state = self.read_state_side()

        totalstartdistance = start_state.left + start_state.right

        is_correction, yaw_delta, start_left_distance, start_right_distance = \
            self._positioner.center_bot_correction(start_state.front, start_state.left,
                                         start_state.right,0)

        (maxfront,_,__) = self.intelligence.get_learned_distances()

        gyrodefault = yaw_delta

        # Create parameter object for unknown direction walk
        walk_params = WalkParameters(
            min_front=maxfront,
            def_left=start_left_distance,
            def_right=start_right_distance,
            gyro_default=gyrodefault,
            speed=self.MIN_SPEED,
            speed_check=True,
            force_change=is_correction
        )
        self.handle_straight_walk(params=walk_params)

        #Complete walk to corner , now lets find the color for direction.

        (color,color2) = self.walk_read_mat_color(start_distance=totalstartdistance,
                                                  def_turn_angle=gyrodefault)

        #ensure we have reached corner, but we dampening the angle otherwise too much 
        #movement
        self.walk_to_corner(def_turn_angle=gyrodefault)

        state = self.read_state_side()

        self._direction = decide_direction(color, color2, state.left, state.right)

        if self._direction != MATDIRECTION.UNKNOWN_DIRECTION:
            self.intelligence.report_direction_side1(self._direction)
        else:
            return  # unable to determine; stop early

        self.output_inf.buzzer_beep()
        logger.warning("Final: %s", directiontostr(self._direction))

        self.output_inf.force_flush_messages()

    def handle_side(self,gyroreset:bool = True,def_yaw:float = 0)->float:
        """Handle side walk
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
            self.movementcontroller.turn_steering_with_logging(steering,async_turn=False,current_speed=0)

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
        direction_sign = 1 if self._direction == MATDIRECTION.CLOCKWISE_DIRECTION else -1
        def condition_met(state:RobotState) -> bool:
            nonlocal last_stop
            nonlocal prev_measure_distance
            nonlocal direction_sign
            if state.left <= 15 or state.right <= 15:
                if (time.time() - last_stop) > 0.5:
                    self.movementcontroller.stop_walking()
                    logger.info("Condition met: very close to wall stopping L: %.2f, R: %.2f",\
                                 state.left, state.right)
                    last_stop = time.time()
                    return False
                else:
                    logger.info("Close to wall but wait for 500ms")

            if direction_sign *(state.yaw - def_yaw) > 10 and self.movementcontroller.get_distance() - prev_measure_distance > 20:
                #we have a 10 degree error, lets stop and correct.
                self.movementcontroller.stop_walking()
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

    def update_side_path(self, current_state: RobotState, learned_left: float, \
                        learned_right: float,yaw_delta: float, def_yaw: float, \
                        prev_distance: float) -> Tuple[bool, float, float, float, float]:
        """Helper to recalculate path definitions during a side walk."""
        new_yaw = def_yaw + yaw_delta

        if self._current_distance != (0, 0):
            delta = (learned_left + learned_right) - (self._current_distance[0] \
                                                + self._current_distance[1])

            if delta > 4:  # delta should be greater than 4cm to consider
                _, new_yaw_delta, new_left, new_right = self._positioner.side_bot_centering(
                    current_state.front,
                    learned_left=self._current_distance[0], learned_right=self._current_distance[1],
                    actual_left=current_state.left, actual_right=current_state.right,
                    prev_yaw=yaw_delta, lenient=True
                )
                self._current_distance = (0, 0)
                return True, def_yaw + new_yaw_delta, new_left, new_right, \
                                        current_state.left + current_state.right
            else:
                self._current_distance = (0, 0) # Reset even if not used

        elif prev_distance - (current_state.left + current_state.right) > 10:
            _, new_yaw_delta, new_left, new_right = self._positioner.side_bot_centering(
                current_state.front,
                learned_left=learned_right, learned_right=learned_left,
                actual_left=current_state.left, actual_right=current_state.right,
                prev_yaw=yaw_delta, lenient=True)

            return True, def_yaw + new_yaw_delta, new_left, new_right, \
                                        current_state.left + current_state.right

        return False, new_yaw, learned_left, learned_right, prev_distance

    def walkbackcond(self,state:RobotState,minfront:float=30,minleft:float=0,minright:float=0):
        """Check conditions for walk back"""

        if (state.camera_front <= minfront and state.camera_front > 0) \
            or state.front <= minfront or \
           (state.camera_left <= minleft and state.camera_left > 0 ) \
              or state.left <= minleft or \
           (state.camera_right <= minright and state.camera_right > 0) \
               or state.right <= minright:
            return True
        return False

    def walk_back(self,state:RobotState,minfront:float=30,minleft:float=0,minright:float=0):
        """Handle side walk back"""

        if self.walkbackcond(state,minfront,minleft,minright):

            logger.info("Handling walk back")

            self.movementcontroller.stop_walking()
            startdistance = self.movementcontroller.get_distance()
            currentdistance = startdistance
            #set steering to zero.
            self.output_inf.reset_steering()
            state = self.read_state_side()

            while self.walkbackcond(state,minfront,minleft,minright) and (startdistance - currentdistance  < 15):
                self.movementcontroller.start_backward(self.MIN_SPEED+10)
                time.sleep(0.001)
                state = self.read_state_side()
            self.movementcontroller.stop_walking()

            logger.info("Completed walk back")

    def handle_corner_round1(self,gyrodefault:float,gyroreset:bool=True):
        """Handle corner walk"""

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

        prev_yaw = state.yaw
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
            angle = self._gyro_corner_walk(def_turn_angle, min_left, min_right,\
                                          fixed_turn_angle=recommended_turn_angle)
            self.intelligence.location_complete(state)
            self.intelligence.unregister_callback()
            return def_turn_angle
        finally:
            pass

        return

    def _gyro_corner_walk(self, def_turn_angle: float, min_left: float, min_right: float,
                          fixed_turn_angle:float,
                            ) -> float:
        """Handle the gyro corner walking logic."""

        logger.info("Gyro corner walk round n initiated with turn angle: %.2f", def_turn_angle)

        (def_front, _, _) = self.intelligence.get_learned_distances()
        state = self.read_state_corner()
        #def_front is 15, we want to be 30 cm away from wall.
        if state.front <= def_front*2:
            #walk back a little.
            self.walk_back(state,minfront=def_front*2,minleft=10,minright=10)



        def report_distances_corner(left: float, right: float):
            logger.info("corner Report. Left: %.2f, Right: %.2f", left, right)
            self._current_distance = (left, right)
            #TODO: we should stop for now lets wait.
            self.movementcontroller.stop_walking()

        if def_turn_angle > self.CORNER_YAW_ANGLE:
            fixed_turn_angle = MAX_STEERING_ANGLE


        #since we overshoot lets a little less.
        def_turn_angle += -2 if MATDIRECTION == MATDIRECTION.CLOCKWISE_DIRECTION else 2
        logger.info("Adjusted turn angle: %.2f", def_turn_angle)

        gyrohelper: FixedTurnWalker = FixedTurnWalker(
            max_left_distance=constants.LEFT_DISTANCE_MAX,
            max_right_distance=constants.RIGHT_DISTANCE_MAX,
            fixed_turn_angle=fixed_turn_angle,
            def_turn_angle=def_turn_angle, min_left=min_left, min_right=min_right)

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

        self.movementcontroller.stop_walking()
        logger.info("End corner : Front:%.2f distance travelled:%.2f", state.front,\
                     self.movementcontroller.get_distance())

        if  state.front <= def_front and abs(delta_angle_deg(state.yaw,def_turn_angle)) < 10:
            #we stopped since we are close to the wall.
            self.walk_back(state,minfront=20,minleft=min_left,minright=min_right)
            #lets try to turn again .
            self._gyro_corner_walk(def_turn_angle,min_left,min_right,fixed_turn_angle+5)


        return def_turn_angle

    def log_data(self,helper:Optional[EquiWalkerHelper]=None):
        """Log the data from the helper if provided."""
        if helper is not None:
            messages: List[str] = helper.get_log_data()
            messages.append(f"Loc: {locationtostr(self.intelligence.get_location())}, " )
            self.output_inf.add_screen_logger_message(messages)

    def handle_straight_walk(self,
                            params: WalkParameters,
                            keep_walking: Optional[Callable[[RobotState], bool]] = None)\
                                                 -> RobotState:
        """Handle the straight walking logic with structured parameters.
        
        Args:
            params: Walking parameters
            keep_walking: Optional function to determine when to keep walking
            
        Returns:
            Final robot state after completing the walk
        """
        logger.info("Straight walk initiated with min distances - Front: %.2f, Left: %.2f, "
                    "Right: %.2f, Gyro: %.2f", params.min_front, params.def_left, 
                    params.def_right, params.gyro_default)

        helper: EquiWalkerHelper = params.make_equi_helper()

        (_,left_def,right_def) = self.intelligence.get_learned_distances()

        state = self.read_state_side()

        if keep_walking is None:
            def noopcond(_state: RobotState) -> bool:
                return True

            keep_walking = noopcond

        if state.front <= params.min_front:
            logger.warning("Front distance %.2f is less than minimum required %.2f, "
                           "not starting walk.", state.front, params.min_front)
            return state

        if state.left <= params.min_left or state.right <= params.min_right:
            #very close to wall , turn more, walk slow
            turn_angle = self._inner_turn(state, left_def, right_def,
                                         params.speed_check, self.CORRECTION_SPEED,
                                         params.is_unknown_direction, helper,lenient=False,
                                            async_turn=True)
        else:
            turn_angle = self._inner_turn(state, left_def, right_def,
                                          params.speed_check, params.speed,
                                          params.is_unknown_direction, helper,lenient=True,
                                            async_turn=True)

        #we would walk slow if the turn angle exist.
        if state.front < params.min_front + 10:
            # logger.info("Front distance %.2f is less than minimum required + 10cm %.2f, "
            #             "walking slow.", state.front, params.min_front + 10)
            self.movementcontroller.start_walking(self.MIN_SPEED)
        elif turn_angle is None:
            logger.info("No turn angle detected, walking straight faster")
            self.movementcontroller.start_walking(params.speed)
        else:
            self.movementcontroller.start_walking(self.MIN_SPEED)
        state = self.read_state_side()

        prev_turn_angle = 0

        while state.front > params.min_front and keep_walking(state) is True:
            turn_angle = self._inner_turn(state, left_def, right_def,
                                         params.speed_check, params.speed,
                                         params.is_unknown_direction, helper)

            #lets sleep if no angle or same angle as previous.
            if turn_angle is None or prev_turn_angle == turn_angle:
                # time.sleep(0.001)
                prev_turn_angle = 0
            else:
                prev_turn_angle = turn_angle

            state = self.read_state_side()
            
            if state.front < params.min_front + 10:
            # logger.info("Front distance %.2f is less than minimum required + 10cm %.2f, "
            #             "walking slow.", state.front, params.min_front + 10)
             if self.movementcontroller.is_walking():
                self.movementcontroller.start_walking(self.MIN_SPEED)

        self.log_data(helper)
        return state
        # self.output_inf.buzzer_beep()

    def _inner_turn(self,state:RobotState, left_def:float, right_def:float,
                    speedcheck:bool, defaultspeed:float,is_unknown_direction:bool,
                    helper: EquiWalkerHelper,lenient:bool=False,async_turn:bool = False):

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

        if lenient is True and turn_angle is not None:
            turn_angle = turn_angle * 0.5

        self.movementcontroller.turn_steering_with_logging(turn_angle,speedcheck=speedcheck,
                                        current_speed=defaultspeed,async_turn=async_turn)
        return turn_angle

    def walk_read_mat_color(self, start_distance: float, def_turn_angle: float) -> \
                                                                Tuple[str|None,str|None]:
        """Read the bottom color using the mat_color function."""

        self.movementcontroller.stop_walking()

        logger.info("Time to check color")
        #lets reduce the gyro correction
        def_turn_angle = def_turn_angle * 0.5

        min_left = 20
        min_right = 20

        if start_distance < 70:
            min_left = 15
            min_right = 15

        color = check_bottom_color(self.output_inf, list(self.KNOWN_COLORS))

        if color is None:

            gyrohelper:GyroWalkerwithMinDistanceHelper = GyroWalkerwithMinDistanceHelper(
                                                def_turn_angle=def_turn_angle,
                                                min_left=min_left,
                                                min_right=min_right,
                                                kgyro = 3.5, # Dont turn too much on gyro.
                                                max_left_distance=constants.LEFT_DISTANCE_MAX,
                                                max_right_distance=constants.RIGHT_DISTANCE_MAX
                                            )

            def set_line_color(c):
                self._line_color = c
                logger.info("Found Color: %s", c)
                self.movementcontroller.stop_walking()

            def value_check_func():
                return check_bottom_color(self.output_inf, list(self.KNOWN_COLORS))

            colorchecker: ConditionCheckerThread = ConditionCheckerThread(
                value_check_func=value_check_func,
                callback_func=set_line_color,
                interval_ms=20
            )

            state = self.read_state_side()

            turn_angle = gyrohelper.walk_func(left_distance=state.left,
                                                  right_distance=state.right,
                                                  current_angle=state.yaw)

            self.movementcontroller.\
                            turn_steering_with_logging(turn_angle,current_speed=self.MIN_SPEED,async_turn=True)


            colorchecker.start()

            try:
                state = self.read_state_side()
                self._line_color = None
                logger.info("Start color walk")
                self.movementcontroller.start_walking(self.MIN_SPEED)
                while (state.front > self.WALLFRONTENDDISTANCE
                                    and self._line_color is None):

                    turn_angle = gyrohelper.walk_func(left_distance=state.left,
                                                  right_distance=state.right,
                                                  current_angle=state.yaw)


                    self.movementcontroller.\
                            turn_steering_with_logging(turn_angle,current_speed=self.MIN_SPEED)
                    time.sleep(0.005)
                    state = self.read_state_side()

            finally:
                #Lets first stop the base and then check the color.
                self.movementcontroller.stop_walking()
                if colorchecker.is_running():
                    logger.info("Stopping color checker thread, not found color yet.")
                    colorchecker.stop()

            self.log_data(gyrohelper)

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
        if state.left + state.right < 150:
            logger.info("Both sides are present , we have not reached the corner, walk...")

            #Lets walk start some more time

            def cond1(state:RobotState):
                return state.left + state.right < 150 or \
                    (self.output_inf.get_left_lidar_distance() + \
                      self.output_inf.get_right_lidar_distance() < 150)

            if self._direction == MATDIRECTION.CLOCKWISE_DIRECTION:
                #only look at left wall
                walk_params = WalkParameters(
                    min_front=self.WALLFRONTFORWALL,
                    def_left=state.left,
                    def_right=200,
                    gyro_default=def_turn_angle,
                    speed=self.WALK_TO_CORNER_SPEED,
                    min_left=20
                )
                state = self.handle_straight_walk(params=walk_params, keep_walking=cond1)

            elif self._direction == MATDIRECTION.ANTICLOCKWISE_DIRECTION:
                # only look at right wall
                walk_params = WalkParameters(
                    min_front=self.WALLFRONTFORWALL,
                    def_left=200,
                    def_right=state.right,
                    gyro_default=def_turn_angle,
                    speed=self.WALK_TO_CORNER_SPEED,
                    min_right=20
                )
                state = self.handle_straight_walk(params=walk_params, keep_walking=cond1)
            else:
                #we want to do gyro walk, but about too much turning
                walk_params = WalkParameters(
                    min_front=self.WALLFRONTFORWALL,
                    def_left=state.left,
                    def_right=state.right,
                    gyro_default=state.yaw/2,
                    speed=self.WALK_TO_CORNER_SPEED,
                    min_right=15,
                    min_left=15
                )
                state = self.handle_straight_walk(params=walk_params, keep_walking=cond1)
        else:
            logger.info("Both sides are not present, we have reached the corner.")

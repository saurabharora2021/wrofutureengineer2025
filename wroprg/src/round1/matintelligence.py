"""This class implemenents the mathematical intelligence for the Mat used. """
import logging
from collections import Counter, deque
from typing import Callable, Optional
import threading
import time
from utils.mat import MATDIRECTION, MATLOCATION, MATGENERICLOCATION
from utils.mat import location_to_genericlocation

from base.shutdown_handling import ShutdownInterface
from hardware.hardware_interface import HardwareInterface

logger = logging.getLogger(__name__)
class MatIntelligence(ShutdownInterface):
    """Class to implement the mathematical intelligence for the Mat used."""

    DEFAULT_DISTANCE = (90.0,20.0)
    MAX_WALL2WALL_DISTANCE = 110.0
    MIN_WALL2WALL_DISTANCE = 60.0
    ROBOT_WIDTH = 20.0  # Width of the robot in cm
    DELTA_ERROR = 10.0 # Maximum error in cm for distance measurements
    FRONTDISTANCE_FOR_COLOR_CHECK=120.0
    MAX_DISTANCE_READING = 200.0 # Maximum distance reading in cm
    WALLFRONTDISTANCE=15.0 # while corner walking , maximum distance from the wall in front
    WALLSIDEDISTANCE=20.0 # while corner walking , maximum distance from the wall on the side

    def __init__(self,roundcount:int = 1, hardware_interface: Optional[HardwareInterface]=None
                                                    ) -> None:
        """Initialize the MatIntelligence class."""
        self._deque = deque()
        self._direction = MATDIRECTION.UNKNOWN_DIRECTION
        self._location = MATLOCATION.SIDE_1
        self._roundno = 1
        self._roundcount = roundcount
        self._readings_counter = 0
        self._hardware_interface: Optional[HardwareInterface] = hardware_interface
        #settings log level to warn to avoid overlogging.
        # logger.setLevel(logging.WARNING)

        # Reading for the start location, for starting position
        self._mem_initial_start = (0,0,0)

        self._locationssequence = [
            MATLOCATION.SIDE_1,
            MATLOCATION.CORNER_1,
            MATLOCATION.SIDE_2,
            MATLOCATION.CORNER_2,
            MATLOCATION.SIDE_3,
            MATLOCATION.CORNER_3,
            MATLOCATION.SIDE_4,
            MATLOCATION.CORNER_4,
        ]

        # Default distances for anticlockwise equidistance walking
        # No values for corners.
        self._default_distances_anticlockwise = {
            MATLOCATION.SIDE_1: (self.FRONTDISTANCE_FOR_COLOR_CHECK,-1,-1),
            MATLOCATION.CORNER_1: (self.WALLFRONTDISTANCE,self.WALLSIDEDISTANCE,-1),
            MATLOCATION.SIDE_2: (100,20,-1),
            MATLOCATION.CORNER_2: (self.WALLFRONTDISTANCE,self.WALLSIDEDISTANCE,-1),
            MATLOCATION.SIDE_3: (100,20,-1),
            MATLOCATION.CORNER_3: (self.WALLFRONTDISTANCE,self.WALLSIDEDISTANCE,-1),
            MATLOCATION.SIDE_4: (100,20,-1),
            MATLOCATION.CORNER_4: (self.WALLFRONTDISTANCE,self.WALLSIDEDISTANCE,-1),
        }

        self._default_distances_unknown = {
            MATLOCATION.SIDE_1: (self.FRONTDISTANCE_FOR_COLOR_CHECK,-1,-1),
        }

        # Default distances for clockwise equidistance walking
        # No values for corners.
        self._default_distances_clockwise = {
            # left, right are not known at this point.
            MATLOCATION.SIDE_1: (self.FRONTDISTANCE_FOR_COLOR_CHECK,-1,-1),
            MATLOCATION.CORNER_1: (self.WALLFRONTDISTANCE,-1,self.WALLSIDEDISTANCE),
            MATLOCATION.SIDE_2: (100,-1,20),
            MATLOCATION.CORNER_2: (self.WALLFRONTDISTANCE,-1,self.WALLSIDEDISTANCE),
            MATLOCATION.SIDE_3: (100,-1,20),
            MATLOCATION.CORNER_3: (self.WALLFRONTDISTANCE,-1,self.WALLSIDEDISTANCE),
            MATLOCATION.SIDE_4: (100,-1,20),
            MATLOCATION.CORNER_4: (self.WALLFRONTDISTANCE,-1,self.WALLSIDEDISTANCE),
        }

        self._learned_distances = {}
        self._current_min_distances = self.DEFAULT_DISTANCE  # (left, right)

        self._callback: Callable[[float,float],None] | None = None

        self._reading_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._start_reading_thread()
        logger.info("MatIntelligence initialized")

    def add_readings(self, front_distance: float, left_distance: float,
                            right_distance: float) -> None:
        """Add readings to the deque."""
        self._deque.append((front_distance, left_distance, right_distance))

    def _start_reading_thread(self):
        """Start the background thread to process readings from the deque."""
        if self._reading_thread is None or not self._reading_thread.is_alive():
            self._stop_event.clear()
            self._reading_thread = threading.Thread(target=self._process_readings, daemon=True)
            self._reading_thread.start()
            logger.info("Started reading thread.")

    def _stop_reading_thread(self):
        """Stop the background reading thread."""
        self._stop_event.set()
        if self._reading_thread is not None:
            self._reading_thread.join()
            logger.info("Stopped reading thread.")

    def _process_readings(self):
        """Thread target: process readings from the deque."""
        while not self._stop_event.is_set():
            if len(self._deque) > 0:
                front_distance, left_distance, right_distance = self._deque.popleft()
                self._process_each_readings(front_distance, left_distance, right_distance)
            else:
                time.sleep(0.01)  # Avoid busy waiting

    def print_mat_intelligence(self):
        """Print the current state of MatIntelligence."""
        logger.info("MatIntelligence State:")
        logger.info("%s",self._learned_distances)


    def get_round_number(self) -> int:
        """Get the current round number."""
        return self._roundno

    def get_direction(self) -> MATDIRECTION:
        """Get the current direction of the Mat Walker."""
        return self._direction

    def get_location(self) -> MATLOCATION:
        """Get the current location of the Mat Walker."""
        return self._location

    def get_generic_location(self) -> MATGENERICLOCATION:
        """Get the current generic location of the Mat Walker."""
        return location_to_genericlocation(self._location)

    def report_direction_side1(self, direction: MATDIRECTION) -> None:
        """Report the direction for Side 1, for the first time"""
        if direction not in MATDIRECTION:
            raise ValueError(f"Invalid direction: {direction}")
        if self._location != MATLOCATION.SIDE_1 :
            raise ValueError("Current location is not SIDE_1, cannot report direction.")

        # We are going to block till all directions are processed for 5 secs.
        self._wait_for_readings()

        self._direction = direction
        #change in location to corner 1
        self._location = MATLOCATION.CORNER_1

        if self._current_min_distances is not None:
            self._learned_distances[MATLOCATION.SIDE_1] = (100,self._current_min_distances[0],
                                                          self._current_min_distances[1])

        logger.info("Report side 1, current min distances: %.2f,%.2f",
                    self._current_min_distances[0], self._current_min_distances[1])
        self._current_min_distances = self.DEFAULT_DISTANCE

    def _wait_for_readings(self, timeout: float = 2.0) -> None:
        """Wait for readings to be processed."""
        start_time = time.time()
        while len(self._deque) > 0 and (time.time() - start_time) < timeout:
            time.sleep(0.01)

    def reset_current_distance(self,left:float = 0, right:float = 0):
        """Reset the current distance readings."""
        if left <=0 or right <= 0:
            self._current_min_distances = self.DEFAULT_DISTANCE
        else:
            self._current_min_distances = (left, right)

    def get_initial_readings(self):
        """Get the initial readings stored in memory."""
        return self._mem_initial_start

    def get_learned_distances(self,location:MATLOCATION|None=None) -> tuple[float, float, float]:
        """Get the learned distances."""
        if location is None:
            location = self._location
        learned_distance = self._learned_distances.get(location,None)
        if learned_distance is None:
            if self._roundno > self._roundcount and location == MATLOCATION.SIDE_1:
                # In last round, we need to return to the same square we started.
                logger.info("Reached the last round, returning to initial position.")
                learned_distance = self._mem_initial_start
            else:

                if self._direction == MATDIRECTION.ANTICLOCKWISE_DIRECTION:
                    learned_distance = self._default_distances_anticlockwise[location]
                elif self._direction == MATDIRECTION.CLOCKWISE_DIRECTION:
                    learned_distance = self._default_distances_clockwise[location]
                else:
                    learned_distance = self._default_distances_unknown[location]

        logger.info("Learned distances for location %s: %s", location, learned_distance)
        if learned_distance is not None:
            return learned_distance
        else:
            return (-1,-1,-1)

    def _mid_distance(self) -> float|None:
        """Get the mid distance for the current location."""
        if self._current_min_distances is not None:
            mid= (self._current_min_distances[0] + self._current_min_distances[1]) / 2
            return mid
        return None

    def _set_learned_distance(self,location:MATLOCATION,mid:float)->None:
        current_dist = self.get_learned_distances(location)
        if current_dist is None:
            logger.info("Learning distances for location: %s", location)
            logger.info("Current min distances: %s", self._current_min_distances)

            self._learned_distances[location] = (100,mid,mid)
        else:
            mid_dist = (current_dist[1] + current_dist[2])/2
            if  mid_dist> mid or mid_dist < 20:
                self._learned_distances[location] = (current_dist[0],mid,mid)
                logger.info("Learning distances for location overwrite: %s", location)
                logger.info("Current min distances overwrite: %s", self._current_min_distances)
            else:
                logger.info("Not learning distances for location: %s", location)


    def location_complete(self) -> MATLOCATION:
        """Change the current location of the Mat Walker."""
        logger.info("Location complete: %s, current dis:%s",self._location,
                            self._current_min_distances)
        if self._roundno == 1:
            self._wait_for_readings()
        next_location = self.next_location(self._location)

        if location_to_genericlocation(self._location) == MATGENERICLOCATION.SIDE:
            # We are at a side, so we can learn the distances.
            mid = self._mid_distance()
            logger.info("location complete: side mid:%s", mid)
            if mid is not None:
                if mid >50:
                    mid = 48
                elif mid < 40 and mid > 29:
                    mid = 28
                self._set_learned_distance(self._location, mid)
        else:
            #We are at a corner , so we learned the distance for the next side.
            mid = self._mid_distance()
            logger.info("location complete: corner mid:%s", mid)
            if mid is not None:
                self._set_learned_distance(next_location, mid)

        if self._location == MATLOCATION.CORNER_4:
            self.reprocess_map()
            self.print_mat_intelligence()
            self._roundno += 1

        #reset the location and min distances
        self._location = next_location

        (_,left,right) = self.get_learned_distances()
        self._current_min_distances = (left,right)


        if self._roundno == 1:
            mid = self._mid_distance()
            if mid is not None and mid < 25:
                logger.info("resetting current min for round 1 to default")
                self._current_min_distances = self.DEFAULT_DISTANCE

        if self._hardware_interface is not None:
            logger.info("NEW LOCATION===%s", self._location)
            self._hardware_interface.add_comment(f"New Location : {self._location}")
        else:
            logger.error("Hardware interface is not set, cannot add comment.")

        logger.info("Current readings... Left: %.2f, Right: %.2f",
                    self._current_min_distances[0], self._current_min_distances[1])
        return self._location

    def reprocess_map(self):
        """Reprocess the map based on the current readings."""
        logger.info("Reprocessing map...")
        ### This method can be used to reprocess the map based on the current readings.
        for location in self._locationssequence:
            generic_loc = location_to_genericlocation(location)
            learned_distance = self.get_learned_distances(location)
            if generic_loc == MATGENERICLOCATION.SIDE:

                # If the location is a side, we can learn the distances.
                if learned_distance is not None:
                    logger.info("Learned distances for location %s: %s", location,
                                learned_distance)
                    #next corner
                    next_corner = self.next_location(location)
                    #next side
                    next_side = self.next_location(next_corner)

                    next_distance = self.get_learned_distances(next_side)
                    if next_distance is not None:
                        logger.info("Learned distances for next side %s: %s", next_side,
                                    next_distance)
                        #lets find the size of next size.
                        total = next_distance[1] + next_distance[2]
                        if total > 50 and total < 130:
                            # we have a longer side, lets walk less
                            learned_distance = (100,next_distance[1],next_distance[2])
                        elif total < 65 and total > 35:
                            #we have a shorter side , lets walk more
                            learned_distance = (70,next_distance[1],next_distance[2])

                        self._learned_distances[location] = learned_distance
                        logger.info("Changed Learned distances for size %s: %s", location,
                                learned_distance)
            elif generic_loc == MATGENERICLOCATION.CORNER:
                # If the location is a corner, we can learn the distances.
                if learned_distance is not None:
                    logger.info("Learned distances for location %s: %s", location,
                                learned_distance)
                    #next side
                    next_side = self.next_location(location)

                    next_distance = self.get_learned_distances(next_side)
                    if next_distance is not None:
                        logger.info("Learned distances for next corner %s: %s", next_side,
                                    next_distance)
                        #lets find the size of next size,
                        total = next_distance[1] + next_distance[2]
                        if total > 50 and total < 130:
                            # we have a longer side.
                            if self._direction == MATDIRECTION.CLOCKWISE_DIRECTION:

                                learned_distance = (self.WALLFRONTDISTANCE, 35, 30)
                            else:
                                learned_distance = (self.WALLFRONTDISTANCE, 30, 35)
                        elif total < 65 and total > 35:
                            #we are on the shorter side
                            if self._direction == MATDIRECTION.CLOCKWISE_DIRECTION:

                                learned_distance = (self.WALLFRONTDISTANCE, 25, 20)
                            else:
                                learned_distance = (self.WALLFRONTDISTANCE, 20, 25)

                        self._learned_distances[location] = learned_distance
                        logger.info("Changed Learned distances for size %s: %s", location,
                                    learned_distance)

    def set_default_distances(self, default_distances: dict[MATLOCATION, tuple[float, float, float]]) -> None:
        """Set the default distances for each location, primarily used for testing."""
        self._learned_distances = default_distances

    def next_location(self,location:MATLOCATION) -> MATLOCATION:
        """Get the next location in the sequence."""

        if location == MATLOCATION.SIDE_1:
            return MATLOCATION.CORNER_1
        elif location == MATLOCATION.CORNER_1:
            return MATLOCATION.SIDE_2
        elif location == MATLOCATION.SIDE_2:
            return MATLOCATION.CORNER_2
        elif location == MATLOCATION.CORNER_2:
            return MATLOCATION.SIDE_3
        elif location == MATLOCATION.SIDE_3:
            return MATLOCATION.CORNER_3
        elif location == MATLOCATION.CORNER_3:
            return MATLOCATION.SIDE_4
        elif location == MATLOCATION.SIDE_4:
            return MATLOCATION.CORNER_4
        elif location == MATLOCATION.CORNER_4:
            return MATLOCATION.SIDE_1
        else:
            raise ValueError(f"Unknown current location: {location}")

    def shutdown(self) -> None:
        """Shutdown the MatIntelligence."""
        logger.info("Shutting down MatIntelligence.")
        self._deque.clear()
        self._stop_reading_thread()
        self._direction = MATDIRECTION.UNKNOWN_DIRECTION
        self._location = MATLOCATION.SIDE_1
        logger.info("MatIntelligence shutdown complete.")

    def _process_each_readings(self, front_distance:float, left_distance: float,
                                    right_distance: float) -> None:
        """Process each reading from the deque."""
        logger.info("Processing reading: front=%.2f, left=%.2f, right=%.2f", front_distance,
                                                        left_distance, right_distance)

        if front_distance < 0 or left_distance < 0 or right_distance < 0:
            return  # Ignore negative distances

        self._readings_counter += 1

        if self._readings_counter == 1:
            # This is the first reading, set the starting distances
            total= left_distance + right_distance
            left_distance = total/2
            right_distance = total/2
            self._mem_initial_start = (front_distance, left_distance, right_distance)
            logger.info("Storing First distances: front=%.2f, left=%.2f, right=%.2f",
                        self._mem_initial_start[0], self._mem_initial_start[1],
                          self._mem_initial_start[2])

        # Update the current minimum distances
        total_distance = left_distance + right_distance
        current_total_distance = total_distance

        if self._current_min_distances is not None:
            current_total_distance= self._current_min_distances[0] + self._current_min_distances[1]

        if total_distance < current_total_distance:
            logger.info("Current distance: %.2f, New distance: %.2f",
                     current_total_distance, total_distance)
            left_distance = total_distance / 2
            right_distance = total_distance / 2

            self._current_min_distances = (left_distance,right_distance)

            logger.info("Updated current minimum distances: %s", self._current_min_distances)
            if total_distance < self.MAX_WALL2WALL_DISTANCE:
                # If the total distance is less than the minimum wall-to-wall distance,
                # time to send the distances to the walker helper.
                if self._callback is not None:
                    logger.warning("reset distance left: %.2f, right: %.2f",
                                   left_distance, right_distance)
                    self._callback(left_distance,right_distance)


    def register_callback(self, callback: Callable[[float,float],None]) -> None:
        """Register the callback instance."""
        if not callable(callback):
            raise TypeError("callback must be a callable")
        self._callback = callback
        logger.info("Callback registered successfully.")

    def unregister_callback(self) -> None:
        """Unregister the callback instance."""
        self._callback = None
        logger.info("Callback unregistered successfully.")

if __name__ == "__main__":
    mat_intelligence = MatIntelligence()
    locations = [
    MATLOCATION.SIDE_1,
    MATLOCATION.SIDE_2,
    MATLOCATION.SIDE_1,
    MATLOCATION.CORNER_1,
    MATLOCATION.SIDE_1,
]

    # Count frequency of each enum value
    freq = Counter(locations)

    print(freq[MATLOCATION.SIDE_1])
    print(freq[MATLOCATION.SIDE_2])
    print(freq)

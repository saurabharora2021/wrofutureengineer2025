"""This class implemenents the mathematical intelligence for the Mat used. """
import logging
from collections import Counter, deque
from enum import Enum,auto
from typing import Callable, Optional
import threading
import time

from base.shutdown_handling import ShutdownInterface
from hardware.hardware_interface import HardwareInterface

logger = logging.getLogger(__name__)

class MATDIRECTION(Enum):
    """Enum to represent the direction of the Mat Walker."""
    CLOCKWISE_DIRECTION = auto()
    ANTICLOCKWISE_DIRECTION = auto()
    UNKNOWN_DIRECTION = auto()

class MATLOCATION(Enum):
    """Enum to represent the current location of the Mat Walker."""
    SIDE_1 = auto()
    SIDE_2 = auto()
    SIDE_3 = auto()
    SIDE_4 = auto()
    CORNER_1 = auto()
    CORNER_2 = auto()
    CORNER_3 = auto()
    CORNER_4 = auto()

class MATGENERICLOCATION(Enum):
    """Enum to represent the generic location of the Mat Walker."""
    SIDE = auto()
    CORNER = auto()


def color_to_direction(color)-> MATDIRECTION:
    """Convert Mat line color to direction."""
    if color == "blue":
        return MATDIRECTION.ANTICLOCKWISE_DIRECTION
    elif color == "orange":
        return MATDIRECTION.CLOCKWISE_DIRECTION
    else:
        return MATDIRECTION.UNKNOWN_DIRECTION

def location_to_genericlocation(location: MATLOCATION) -> MATGENERICLOCATION:
    """Convert current location to generic location."""
    if location in (MATLOCATION.SIDE_1, MATLOCATION.SIDE_2,
                    MATLOCATION.SIDE_3, MATLOCATION.SIDE_4):
        return MATGENERICLOCATION.SIDE
    if location in (MATLOCATION.CORNER_1, MATLOCATION.CORNER_2,
                    MATLOCATION.CORNER_3, MATLOCATION.CORNER_4):
        return MATGENERICLOCATION.CORNER
    else:
        raise ValueError(f"Unknown current location: {location}")

def vote_directions(list_of_directions: list[MATDIRECTION]) -> MATDIRECTION:
    """Vote for the most common direction in the list."""
    if not list_of_directions:
        return MATDIRECTION.UNKNOWN_DIRECTION
    direction_counter = Counter(list_of_directions)
    most_common_direction, _ = direction_counter.most_common(1)[0]
    logger.info("Most common direction: %s", most_common_direction)
    return most_common_direction


class MatIntelligence(ShutdownInterface):
    """Class to implement the mathematical intelligence for the Mat used."""

    DEFAULT_DISTANCE = (100,20)
    MAX_WALL2WALL_DISTANCE = 110
    MIN_WALL2WALL_DISTANCE = 60
    ROBOT_WIDTH = 20  # Width of the robot in cm
    DELTA_ERROR = 10 # Maximum error in cm for distance measurements
    FRONTDISTANCE_FOR_COLOR_CHECK=120
    MAX_DISTANCE_READING = 200 # Maximum distance reading in cm
    WALLFRONTDISTANCE=15 # while corner walking , maximum distance from the wall in front
    WALLSIDEDISTANCE=20 # while corner walking , maximum distance from the wall on the side

    def __init__(self,roundcount:int = 1, hardware_interface: Optional[HardwareInterface]=None) -> None:
        """Initialize the MatIntelligence class."""
        self._deque = deque()
        self._direction = MATDIRECTION.UNKNOWN_DIRECTION
        self._location = MATLOCATION.SIDE_1
        self._roundno = 1
        self._roundcount = roundcount
        self._readings_counter = 0
        self._hardware_interface: Optional[HardwareInterface] = hardware_interface

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
        self._current_min_distances = (100,100)
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

        self._current_min_distances = self.DEFAULT_DISTANCE
        logger.info("Report side 1, current min distances: %.2f", self._current_min_distances)

    def _wait_for_readings(self, timeout: float = 5.0) -> None:
        """Wait for readings to be processed."""
        start_time = time.time()
        while len(self._deque) > 0 and (time.time() - start_time) < timeout:
            time.sleep(0.1)

    def reset_current_distance(self):
        """Reset the current distance readings."""
        self._current_min_distances = self.DEFAULT_DISTANCE

    def get_initial_readings(self):
        """Get the initial readings stored in memory."""
        return self._mem_initial_start

    def get_learned_distances(self) -> tuple[float, float, float]:
        """Get the learned distances."""
        learned_distance = self._learned_distances.get(self._location,None)
        if learned_distance is None:
            if self._roundno == 1:
                if self._direction == MATDIRECTION.ANTICLOCKWISE_DIRECTION:
                    learned_distance = self._default_distances_anticlockwise[self._location]
                elif self._direction == MATDIRECTION.CLOCKWISE_DIRECTION:
                    learned_distance = self._default_distances_clockwise[self._location]
                else:
                    learned_distance = self._default_distances_unknown[self._location]
        elif self._roundno > self._roundcount and self._location == MATLOCATION.SIDE_1:
            # In last round, we need to return to the same square we started.
            learned_distance = self._mem_initial_start

        logger.info("Learned distances for location %s: %s", self._location, learned_distance)
        if learned_distance is not None:
            return learned_distance
        else:
            return (-1,-1,-1)

    def location_complete(self) -> MATLOCATION:
        """Change the current location of the Mat Walker."""
        logger.info("Location complete: %s",self._location)
        #TODO: should we reset in first round ? or every round ?
        # if self._roundno == 1:
            #we are learning the distances for the first round.
        self._wait_for_readings()
        if location_to_genericlocation(self._location) == MATGENERICLOCATION.SIDE:
            # We are at a side, so we can learn the distances.
            if self._current_min_distances is not None:
                self._learned_distances[self._location] = (100,
                                                        self._current_min_distances[0],
                                                        self._current_min_distances[1])
        else:
            #We are at a corner , so we learned the distance for the next side.
            next_location = self._next_location()
            if self._current_min_distances is not None:
                logger.info("Learning distances for next location: %s", next_location)
                logger.info("Current min distances: %s", self._current_min_distances)
                mid_distance = (self._current_min_distances[0] + self._current_min_distances[1])/2
                self._learned_distances[next_location] = (100,
                                                        mid_distance,
                                                        mid_distance)

        if self._location == MATLOCATION.CORNER_4:
            self._roundno += 1
        self._location = self._next_location()
        #reset the min locations.
        if self._learned_distances.get(self._location) is not None:
            self._current_min_distances = self._learned_distances.get(self._location)
        else:
            if self._roundno == 1:
                #this can happen
                self._current_min_distances = self.DEFAULT_DISTANCE
            else:
                logger.error("Cannot find current min for location: %s", self._location)
        if self._hardware_interface is not None:
            self._hardware_interface.add_comment(f"New Location : {self._location}")
        return self._location

    def _next_location(self) -> MATLOCATION:
        """Get the next location in the sequence."""

        if self._location == MATLOCATION.SIDE_1:
            return MATLOCATION.CORNER_1
        elif self._location == MATLOCATION.CORNER_1:
            return MATLOCATION.SIDE_2
        elif self._location == MATLOCATION.SIDE_2:
            return MATLOCATION.CORNER_2
        elif self._location == MATLOCATION.CORNER_2:
            return MATLOCATION.SIDE_3
        elif self._location == MATLOCATION.SIDE_3:
            return MATLOCATION.CORNER_3
        elif self._location == MATLOCATION.CORNER_3:
            return MATLOCATION.SIDE_4
        elif self._location == MATLOCATION.SIDE_4:
            return MATLOCATION.CORNER_4
        elif self._location == MATLOCATION.CORNER_4:
            return MATLOCATION.SIDE_1
        else:
            raise ValueError(f"Unknown current location: {self._location}")

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
            #TODO: lets try to middle the bot.
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

        logger.info("Current distance: %.2f, New distance: %.2f",
                     current_total_distance, total_distance)
        if total_distance < current_total_distance:
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

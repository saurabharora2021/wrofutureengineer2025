""" This modules contains WRO Future Engineer 2025 mat functions and constants."""
import math
import logging
from enum import Enum,auto
from collections import Counter


logger = logging.getLogger(__name__)
#based on color.py from buildhat
def mat_color( r, g, b)-> str:
    """Return the color name from RGB

    :param r: Red
    :param g: Green
    :param b: Blue
    :return: Name of the color as a string
    :rtype: str
    """
    table = [("black", (0, 0, 0)),
                ("orange", (121, 60, 60)), #r to 145 , g to 85, b to 82 based on test color.
                ("blue", (50, 50, 120)),  #changed blue to 77 based on test color.
                ("white", (138, 152, 165)),
              #  ("line", (109, 120, 128))
                ]
    near = ""
    euc = math.inf
    for itm in table:
        cur = math.sqrt((r - itm[1][0])**2 + (g - itm[1][1])**2 + (b - itm[1][2])**2)
        if cur < euc:
            near = itm[0]
            euc = cur
    return near

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

"""
Utility functions for the Round 1.
"""
import logging
from utils.mat import mat_color,MATDIRECTION

logger = logging.getLogger(__name__)

def directiontostr(direction):
    """Convert direction to string."""
    if direction == MATDIRECTION.CLOCKWISE_DIRECTION:
        return "Clockwise"
    elif direction == MATDIRECTION.ANTICLOCKWISE_DIRECTION:
        return "Anti-clockwise"
    else:
        return "Unknown"

def clamp_angle(val:float,max_angle:float)-> float:
    """Clamp the value between -max_angle and max_angle."""
    return float(max(min(val, max_angle), -max_angle))

def check_bottom_color(hardware_interface, colors) -> str | None:
    """Read the bottom color and return it if it matches the specified colors."""
    r, g, b, _ = hardware_interface.get_bottom_color_rgbi()
    color = mat_color(r, g, b)
    logger.info("Current rgb: R=%d, G=%d, B=%d", r, g, b)
    logger.info("Waiting for color: %s, current color: %s", colors, color)
    if color in colors:
        logger.info("Detected color: %s", color)
        return color
    else:
        logger.info("Color not detected, current color: %s", color)
        return None

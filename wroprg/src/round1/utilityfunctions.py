"""
Utility functions for the Round 1.
"""
import logging
from utils.mat import mat_color

logger = logging.getLogger(__name__)

def clamp_angle(val:float,max_angle:float)-> float:
    """Clamp the value between -max_angle and max_angle."""
    return float(max(min(val, max_angle), -max_angle))

def check_bottom_color(hardware_interface, colors) -> str | None:
    """Read the bottom color and return it if it matches the specified colors."""
    r, g, b, _ = hardware_interface.get_bottom_color_rgbi()
    color = mat_color(r, g, b)
    # logger.info("Current rgb: R=%d, G=%d, B=%d", r, g, b)
    # logger.info("Waiting for color: %s, current color: %s", colors, color)
    if color in colors:
        logger.info("Detected color: %s", color)
        return color
    else:
        # logger.info("Color not detected, current color: %s", color)
        return None

def delta_angle_deg(from_deg: float, to_deg: float, prefer_positive_180: bool = False) -> float:
    """
    Return the signed smallest rotation (degrees) to go from `from_deg` to `to_deg`.
    Result is in range [-180, 180). If the exact difference is 180, the sign is -180
    unless `prefer_positive_180` is True, in which case 180 is returned.
    """
    d = (to_deg - from_deg + 180.0) % 360.0 - 180.0
    if prefer_positive_180 and d == -180.0:
        return 180.0
    return d

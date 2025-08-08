"""
Utility functions for the Round 1.
"""
from round1.matintelligence import MATDIRECTION

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

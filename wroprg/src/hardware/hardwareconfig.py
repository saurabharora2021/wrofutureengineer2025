"""Hardware Configuration to management multiple robot chassis versions."""
import logging

logger = logging.getLogger(__name__)

class HardwareConfig:
    """Centralized configuration for hardware settings."""    

    ## Chassis version specifies the version of the robot chassis.
    # V1 - Legobot1
    # V2  - Legobot2 with pcb board
    # V3 - 3d printed chassis
    CHASSIS_VERSION = 2

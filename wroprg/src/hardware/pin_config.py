"""GPIO Pin configuration for the Raspberry Pi."""
import logging
from .hardwareconfig import HardwareConfig

logger = logging.getLogger(__name__)

class PinConfig:
    """
    Centralized configuration class for GPIO pins.
    All pin definitions should be stored here rather than in individual classes.
    Pin configuration varies based on chassis version from HardwareConfig.
    """

    # Base pin configurations for different chassis versions
    _CHASSIS_CONFIGS = {
        1: {  # V1 - Legobot1
            'BUZZER_PIN': 20,
            'LED1_RED_PIN': 26,
            'LED1_GREEN_PIN': 19,
            'LED1_BLUE_PIN': 13,
            'BUTTON_PIN': 21,
            'RIGHT_SENSOR_TRIG_PIN': 22,
            'RIGHT_SENSOR_ECHO_PIN': 27,
            'RIGHT_DISTANCE_MAX_DISTANCE': 2,
            'LEFT_SENSOR_TRIG_PIN': 23,
            'LEFT_SENSOR_ECHO_PIN': 24,
            'LEFT_DISTANCE_MAX_DISTANCE': 2,
            'FRONT_SENSOR_TRIG_PIN': 5,
            'FRONT_SENSOR_ECHO_PIN': 6,
            'FRONT_DISTANCE_MAX_DISTANCE': 2,
        },
        2: {  # V2 - Legobot2 with PCB board
            'BUZZER_PIN': 13,
            'LED1_RED_PIN': 12,
            'LED1_GREEN_PIN': 6,
            'LED1_BLUE_PIN': 5,
            'BUTTON_PIN': 19,
            'RIGHT_SENSOR_TRIG_PIN': 23,
            'RIGHT_SENSOR_ECHO_PIN': 21,
            'RIGHT_DISTANCE_MAX_DISTANCE': 2,
            'LEFT_SENSOR_TRIG_PIN': 20,
            'LEFT_SENSOR_ECHO_PIN': 24,
            'LEFT_DISTANCE_MAX_DISTANCE': 2,
            'FRONT_SENSOR_TRIG_PIN': 27,
            'FRONT_SENSOR_ECHO_PIN': 22,
            'FRONT_DISTANCE_MAX_DISTANCE': 2,
            'JUMPER_PIN': 26,
            'CAMERA_ENABLED': True,  # Set to True if camera is connected and used.
        },
        3: {  # V3 - 3D printed chassis
            'BUZZER_PIN': 17,
            'LED1_RED_PIN': 14,
            'LED1_GREEN_PIN': 15,
            'LED1_BLUE_PIN': 4,
            'BUTTON_PIN': 3,
            'RIGHT_SENSOR_TRIG_PIN': 2,
            'RIGHT_SENSOR_ECHO_PIN': 3,
            'RIGHT_DISTANCE_MAX_DISTANCE': 2,
            'LEFT_SENSOR_TRIG_PIN': 22,
            'LEFT_SENSOR_ECHO_PIN': 23,
            'LEFT_DISTANCE_MAX_DISTANCE': 2,
            'FRONT_SENSOR_TRIG_PIN': 24,
            'FRONT_SENSOR_ECHO_PIN': 25,
            'FRONT_DISTANCE_MAX_DISTANCE': 2,
        }
    }

    ### Add default value for all PINS
    BUZZER_PIN = 0
    LED1_RED_PIN =  0
    LED1_GREEN_PIN = 0
    LED1_BLUE_PIN = 0
    BUTTON_PIN = 0
    RIGHT_SENSOR_TRIG_PIN = 0
    RIGHT_SENSOR_ECHO_PIN = 0
    RIGHT_DISTANCE_MAX_DISTANCE = 0
    LEFT_SENSOR_TRIG_PIN = 0
    LEFT_SENSOR_ECHO_PIN = 0
    LEFT_DISTANCE_MAX_DISTANCE = 0
    FRONT_SENSOR_TRIG_PIN = 0
    FRONT_SENSOR_ECHO_PIN = 0
    FRONT_DISTANCE_MAX_DISTANCE = 0
    JUMPER_PIN = 0 # Used for a jumper to change program detection.
    CAMERA_ENABLED:bool= False  # Set to True if camera is connected and used.


    # Common settings for all chassis versions
    SCREEN_WIDTH = 128
    SCREEN_HEIGHT = 64
    SCREEN_UPDATE_INTERVAL = 0.5  # seconds
    LED_TEST_DELAY = 0.25  # seconds

    @classmethod
    def _get_current_config(cls):
        """Get the configuration for the current chassis version."""
        chassis_version = HardwareConfig.CHASSIS_VERSION
        if chassis_version not in cls._CHASSIS_CONFIGS:
            logger.warning("Unknown chassis version %d, defaulting to V1 configuration",
                           chassis_version)
            chassis_version = 1
        return cls._CHASSIS_CONFIGS[chassis_version]

    @classmethod
    def get_pin_value(cls, pin_name):
        """Get a pin value for the current chassis version."""
        config = cls._get_current_config()
        return config.get(pin_name)

    @classmethod
    def log_pin_configuration(cls):
        """Log the current pin configuration for debugging."""
        chassis_version = HardwareConfig.CHASSIS_VERSION

        logger.info("=== GPIO Pin Configuration (Chassis V%d) ===", chassis_version)
        logger.info("Buzzer Pin: %d", cls.BUZZER_PIN)
        logger.info("RGB LED Pins: Red=%d, Green=%d, Blue=%d",
                   cls.LED1_RED_PIN, cls.LED1_GREEN_PIN, cls.LED1_BLUE_PIN)
        logger.info("Button Pin: %d", cls.BUTTON_PIN)
        logger.info("Right Distance Sensor: Trigger=%d, Echo=%d, Max=%0.1fm",
                   cls.RIGHT_SENSOR_TRIG_PIN, cls.RIGHT_SENSOR_ECHO_PIN,
                   cls.RIGHT_DISTANCE_MAX_DISTANCE)
        logger.info("Left Distance Sensor: Trigger=%d, Echo=%d, Max=%0.1fm",
                   cls.LEFT_SENSOR_TRIG_PIN, cls.LEFT_SENSOR_ECHO_PIN,
                   cls.LEFT_DISTANCE_MAX_DISTANCE)
        logger.info("Front Distance Sensor: Trigger=%d, Echo=%d, Max=%0.1fm",
                   cls.FRONT_SENSOR_TRIG_PIN, cls.FRONT_SENSOR_ECHO_PIN,
                   cls.FRONT_DISTANCE_MAX_DISTANCE)
        logger.info("OLED Display: %dx%d, Update Interval=%0.1fs",
                   cls.SCREEN_WIDTH, cls.SCREEN_HEIGHT,
                   cls.SCREEN_UPDATE_INTERVAL)
        logger.info("LED Test Delay: %0.2fs", cls.LED_TEST_DELAY)


# Set class attributes dynamically based on current chassis version
def _initialize_pins():
    """Initialize pin configuration based on chassis version."""
    chassis_version = HardwareConfig.CHASSIS_VERSION
    logger.info("Initializing pin configuration for Chassis V%d", chassis_version)

    config = PinConfig._get_current_config() # pylint: disable=W0212
    for pin_name, pin_value in config.items():
        setattr(PinConfig, pin_name, pin_value)

# Initialize pins when module is imported
_initialize_pins()

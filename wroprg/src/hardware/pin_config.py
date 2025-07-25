"""GPIO Pin configuration for the Raspberry Pi."""
import logging

logger = logging.getLogger(__name__)

class PinConfig:
    """
    Centralized configuration class for GPIO pins.
    All pin definitions should be stored here rather than in individual classes.
    """

    # Buzzer
    BUZZER_PIN = 20

    # RGB LED pins
    LED1_RED_PIN = 26
    LED1_GREEN_PIN = 19
    LED1_BLUE_PIN = 13

    # Button
    BUTTON_PIN = 21

    # Distance sensors
    RIGHT_SENSOR_TRIG_PIN = 22
    RIGHT_SENSOR_ECHO_PIN = 27
    RIGHT_DISTANCE_MAX_DISTANCE = 2 # Maximum distance in meters

    LEFT_SENSOR_TRIG_PIN = 23
    LEFT_SENSOR_ECHO_PIN = 24
    LEFT_DISTANCE_MAX_DISTANCE = 2  # Maximum distance in meters

    # OLED display settings
    SCREEN_WIDTH = 128
    SCREEN_HEIGHT = 64
    LINE_HEIGHT = 13  # pixels per line
    SCREEN_UPDATE_INTERVAL = 0.5  # seconds

    # LED test timing
    LED_TEST_DELAY = 0.25  # seconds

    @classmethod
    def log_pin_configuration(cls):
        """Log the current pin configuration for debugging."""
        logger.info("=== GPIO Pin Configuration ===")
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
        logger.info("OLED Display: %dx%d, Line Height=%dpx, Update Interval=%0.1fs",
                   cls.SCREEN_WIDTH, cls.SCREEN_HEIGHT, cls.LINE_HEIGHT,
                   cls.SCREEN_UPDATE_INTERVAL)
        logger.info("LED Test Delay: %0.2fs", cls.LED_TEST_DELAY)

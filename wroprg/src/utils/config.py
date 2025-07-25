"""Configuration management for different environments."""
import logging
from utils.platform_utils import is_raspberry_pi

logger = logging.getLogger(__name__)

class Config:
    """Configuration class that adapts based on the platform."""

    def __init__(self):
        self.is_rpi = is_raspberry_pi()
        self.load_config()

    def load_config(self):
        """Load configuration based on platform."""
        if self.is_rpi:
            self.camera_enabled = True
            self.gpio_enabled = True
            self.mock_mode = False
            self.camera_resolution = (640, 480)
            self.camera_format = "RGB888"
        else:
            # Development environment
            self.camera_enabled = True  # Use mock camera
            self.gpio_enabled = False   # Use mock GPIO
            self.mock_mode = True
            self.camera_resolution = (640, 480)
            self.camera_format = "RGB888"

        logger.info("Config loaded for %s platform", "Raspberry Pi"
                    if self.is_rpi else "Development")

    def get_camera_config(self):
        """Get camera configuration for the current platform."""
        return {
            "enabled": self.camera_enabled,
            "mock": self.mock_mode,
            "resolution": self.camera_resolution,
            "format": self.camera_format
        }

    def get_gpio_config(self):
        """Get GPIO configuration for the current platform."""
        return {
            "enabled": self.gpio_enabled,
            "mock": self.mock_mode
        }

# Global config instance
config = Config()

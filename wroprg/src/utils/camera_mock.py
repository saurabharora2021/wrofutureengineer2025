"""Mock implementation of picamera2 for development on non-Raspberry Pi platforms."""
import logging

logger = logging.getLogger(__name__)

class MockPicamera2:
    """Mock implementation of Picamera2 for development purposes."""

    def __init__(self):
        self.is_started = False
        logger.info("Mock Picamera2 initialized for development")

    def create_preview_configuration(self, main=None):
        """Mock configuration creation."""
        logger.info("Mock camera config created with: %s", main)
        return {"mock_config": True}

    def configure(self, config):
        """Mock configuration."""
        logger.info("Mock camera configured with: %s", config)

    def start(self):
        """Mock camera start."""
        self.is_started = True
        logger.info("Mock camera started")

    def capture_array(self):
        """Mock frame capture - returns a dummy RGB array."""
        if not self.is_started:
            raise RuntimeError("Camera not started")

        try:
            import numpy as np  # pylint: disable=import-outside-toplevel
            # Return a mock 640x480 RGB frame (green image)
            mock_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            mock_frame[:, :, 1] = 128  # Green channel
            logger.debug("Mock frame captured with numpy")
            return mock_frame
        except ImportError:
            # Fallback to basic list structure if numpy not available
            logger.debug("Mock frame captured without numpy")
            return [[[0, 128, 0] for _ in range(640)] for _ in range(480)]

    def stop(self):
        """Mock camera stop."""
        self.is_started = False
        logger.info("Mock camera stopped")

def get_camera_class():
    """Factory function to return appropriate camera class based on platform."""
    try:
        from picamera2 import Picamera2  # type: ignore # pylint: disable=import-outside-toplevel
        logger.info("Using real Picamera2")
        return Picamera2
    except ImportError:
        logger.warning("Picamera2 not available, using mock implementation")
        return MockPicamera2

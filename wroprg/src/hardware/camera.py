"""Generic Camera which handles picamera and webcam on other platform"""
import logging

from picamera2 import Picamera2  # type: ignore
from libcamera import controls
import numpy as np
from numpy.typing import NDArray

logger: logging.Logger = logging.getLogger(__name__)
class MyCamera:
    """Generic Camera class to handle both PiCamera2 and OpenCV VideoCapture."""
    def __init__(self):

        self._pi_capture = Picamera2()

        # Configure for ~30 FPS and BGR to avoid extra conversions
        video_config = self._pi_capture.create_video_configuration(
            main={"size": (1332, 990), "format": "BGR888"},
            buffer_count=3,
            controls={
                # ~30 FPS => 33,333 microseconds per frame
                "FrameDurationLimits": (33333, 33333),
            },
        )
        self._pi_capture.configure(video_config)

        # Optional autofocus / exposure control
        try:
            self._pi_capture.set_controls({
                "AfMode": controls.AfModeEnum.Continuous,  # ignored on fixed-focus modules
            })
        except Exception as e:
            logger.warning("set_controls(AfMode) failed: %s", e)

        # Re-apply 30 FPS after configure (some platforms require this)
        try:
            self._pi_capture.set_controls({"FrameDurationLimits": (33333, 33333)})
        except Exception as e:
            logger.warning("Warning: could not set FrameDurationLimits: %s", e)

    def start(self):
        """Start the camera."""
        logger.info("Starting Camera ....")

        self._pi_capture.start()
        # Enforce 30 FPS after start
        try:
            self._pi_capture.set_controls({"FrameDurationLimits": (33333, 33333)})
        except Exception as e:
            logger.warning("Warning: could not enforce 30 FPS after start: %s", e)

    def capture(self) -> NDArray[np.uint8]:
        """capture frame"""
        frame_bgr:NDArray[np.uint8] = self._pi_capture.capture_array("main")
        return frame_bgr

    def close(self):
        """Release the camera resources."""
        self._pi_capture.close()

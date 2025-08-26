"""Generic Camera which handles picamera and webcam on other platform"""
import logging
import cv2

try:
    from picamera2 import Picamera2  # type: ignore
    from libcamera import controls
    HAS_PICAMERA2 = True
except ImportError:
    Picamera2 = None  # type: ignore[assignment]
    HAS_PICAMERA2 = False

logger: logging.Logger = logging.getLogger(__name__)
class MyCamera:
    """Generic Camera class to handle both PiCamera2 and OpenCV VideoCapture."""
    def __init__(self, camera_index=0):
        if HAS_PICAMERA2:
            self._pi_capture = Picamera2()

            # Configure for native 1332x990 (binned) mode
            video_config = self._pi_capture.create_video_configuration(
                main={"size": (1332, 990), "format": "RGB888"},
                buffer_count=3,
                controls={
                    # Target 4 FPS => frame duration ~250,000 microseconds
                    "FrameDurationLimits": (250000, 250000),
                },
            )
            self._pi_capture.configure(video_config)

            # Optional autofocus / exposure control
            try:
                # Use string key to avoid ControlId objects that some versions treat as strings
                self._pi_capture.set_controls({
                    "AfMode": controls.AfModeEnum.Continuous,  # Ignore if fixed-focus
                })
            except Exception as e:
                logger.warning("set_controls(AfMode) failed: %s", e)

            # Ensure 4 FPS if not set in config (some platforms require this after configure)
            try:
                self._pi_capture.set_controls({"FrameDurationLimits": (250000, 250000)})
            except Exception as e:
                logger.warning("Warning: could not set FrameDurationLimits: %s", e)
        else:
            self._cv_capture = cv2.VideoCapture(camera_index) # type: ignore[attr-defined]

    def start(self):
        """Start the camera."""
        if HAS_PICAMERA2:
            self._pi_capture.start()
        else:
            self._cv_capture.open()

    def capture(self):
        """capture frame"""
        if HAS_PICAMERA2:
            return self._pi_capture_m()
        return self._cv_capture_m()

    def _cv_capture_m(self):
        ret, frame = self._cv_capture.read()
        if not ret:
            raise RuntimeError("Failed to capture image from camera.")
        return frame

    def _pi_capture_m(self):
        frame_rgb = self._pi_capture.capture_array()         # Capture frame (RGB888)
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        return frame_bgr

    def close(self):
        """Release the camera resources."""
        if HAS_PICAMERA2:
            self._pi_capture.close()
        else:
            self._cv_capture.release()

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
        else:
            self._cv_capture = cv2.VideoCapture(camera_index)  # type: ignore[attr-defined]

    def start(self):
        """Start the camera."""
        if HAS_PICAMERA2:
            self._pi_capture.start()
            # Enforce 30 FPS after start
            try:
                self._pi_capture.set_controls({"FrameDurationLimits": (33333, 33333)})
            except Exception as e:
                logger.warning("Warning: could not enforce 30 FPS after start: %s", e)
        else:
            self._cv_capture.open()
            try:
                self._cv_capture.set(cv2.CAP_PROP_FPS, 30)
            except Exception:
                pass

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
        # Capture directly in BGR (matches configured format) without color conversion
        frame_bgr = self._pi_capture.capture_array("main")
        return frame_bgr

    def close(self):
        """Release the camera resources."""
        if HAS_PICAMERA2:
            self._pi_capture.close()
        else:
            self._cv_capture.release()

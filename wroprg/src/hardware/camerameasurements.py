"""Camera Measurements"""

import os
import logging
import threading
import time
from typing import Callable, Tuple
from typing import Any, Dict
import cv2
import numpy as np
from base.shutdown_handling import ShutdownInterface
from hardware.camera import MyCamera

logger = logging.getLogger(__name__)
# Enable OpenCV optimizations
cv2.setUseOptimized(True)
try:
    cv2.setNumThreads(2)  # tune for your CPU (e.g., 2–4)
except Exception:
    pass
class CameraDistanceMeasurements(ShutdownInterface):
    """Handles distance measurements using the camera."""

    SAVE_CAMERA_IMAGE:bool = False
    SAVE_CAMERA_IMAGE_ON_CORRECTION:bool = True
    CAMERA_OUTPUT_DIR:str= "output"
    SHOW_IMAGE:bool = False
    MIN_FPS: int = 15
    MAX_FPS: int = 30
    ORIENTATION_DEG: int = 180  # 0 or 180. Use 180 if the camera is upside down.
    SHOW_DEBUG=True

    def __init__(self,camera: MyCamera):
        super().__init__()
        self.camera = camera
        self.camera_left = -1
        self.camera_right = -1
        self.camera_front = -1
        self.metrics: Dict[str, Any] = {}

        #init Thread
        self.camera_thread = CameraCheckThread(self.process_camera,self.MIN_FPS)

        # Preallocated buffers for mask computation
        self._mask_initialized = False
        self._roi_rows: slice | None = None
        self._mask_bool: np.ndarray | None = None
        self._tmp_bool: np.ndarray | None = None
        self._section_w: int = 0



    def start(self):
        """Start the camera."""

        if self.SAVE_CAMERA_IMAGE or self.SAVE_CAMERA_IMAGE_ON_CORRECTION:
            outputdir:str = self.CAMERA_OUTPUT_DIR
            os.makedirs(outputdir, exist_ok=True)

        self.camera.start()
        self.camera_thread.start()

    def shutdown(self):
        return self.camera_thread.shutdown()

    def process_camera(self)-> int:
        """Process the camera frame and extract distance measurements."""

        frame = self.camera.capture()

        (center_p,left_p,right_p,self.camera_front,self.camera_left,self.camera_right) = \
                                    self._measure_border(frame)

        # Save frame to output folder
        if self.SAVE_CAMERA_IMAGE or (self.SAVE_CAMERA_IMAGE_ON_CORRECTION and \
                                      (self.camera_front != -1 or self.camera_left != -1 \
                                        or self.camera_right != -1)):
            counter = time.time()
            outputdir:str = self.CAMERA_OUTPUT_DIR
            filename = os.path.join(outputdir, f"frame_{int(counter * 1000):013d}.jpg")
            cv2.imwrite(filename, frame)
            if self.SHOW_DEBUG:
                logger.info("Saved: %s",filename)


        #lets add extra metrics , for now we will just use the timestamp
        self.metrics['c.frontp'] = center_p
        self.metrics['c.rightp'] = right_p
        self.metrics['c.leftp'] =  left_p
        self.metrics['c.frontd'] = self.camera_front
        self.metrics['c.rightd'] = self.camera_right
        self.metrics['c.leftd'] = self.camera_left

        if self.camera_front == -1 and self.camera_left == -1 and self.camera_right == -1:
            return self.MIN_FPS
        else:
            # logger.info("Increasing FPS-----, %.2f,%.2f,%.2f", self.camera_front,\
            #                      self.camera_left, self.camera_right)
            return self.MAX_FPS

    def get_distance(self,)-> Tuple[float,float,float, Dict[str, Any]]:
        """Measure distance using the camera."""
        # Implement distance measurement logic here

        return (self.camera_front,self.camera_left,self.camera_right,self.metrics)

    def _ensure_mask_buffers(self, image: np.ndarray) -> tuple[np.ndarray, bool]:
        """Init/re-init ROI slice and mask buffers if size/orientation changed."""
        h, w = image.shape[:2]
        if self.ORIENTATION_DEG == 180:
            roi_rows = slice(0, h // 2)   # top half
            flip_lr = True
        else:
            roi_rows = slice(h // 2, h)   # bottom half
            flip_lr = False

        need_init = (
            not self._mask_initialized
            or self._roi_rows != roi_rows
            or self._mask_bool is None
            or self._mask_bool.shape != (roi_rows.stop - roi_rows.start, w)
        )

        if need_init:
            roi_h = roi_rows.stop - roi_rows.start
            self._mask_bool = np.empty((roi_h, w), dtype=bool)
            self._tmp_bool  = np.empty((roi_h, w), dtype=bool)
            self._section_w = w // 4
            self._roi_rows = roi_rows
            self._mask_initialized = True

        # Return a view of the ROI (no copy)
        return image[roi_rows, :], flip_lr

    def _measure_border(self,image)-> Tuple[float,float,float,float,float,float]:
        # Note: image is BGR. Reuse preallocated boolean masks; no rotations or grayscale.
        roi, flip_lr = self._ensure_mask_buffers(image)
        assert self._mask_bool is not None and self._tmp_bool is not None

        # In-place threshold: mask = (B<50) & (G<50) & (R<50)
        np.less(roi[:, :, 0], 50, out=self._mask_bool)                 # B < 50
        np.less(roi[:, :, 1], 50, out=self._tmp_bool)                  # G < 50
        np.logical_and(self._mask_bool, self._tmp_bool, out=self._mask_bool)
        np.less(roi[:, :, 2], 50, out=self._tmp_bool)                  # R < 50
        np.logical_and(self._mask_bool, self._tmp_bool, out=self._mask_bool)

        sw = self._section_w
        # Views (no allocation)
        left_view   = self._mask_bool[:, :sw]
        center_view = self._mask_bool[:, sw:3*sw]
        right_view  = self._mask_bool[:, 3*sw:]

        # Percent black per section
        left_black_percentage   = (np.count_nonzero(left_view)   / left_view.size)   * 100.0
        center_black_percentage = (np.count_nonzero(center_view) / center_view.size) * 100.0
        right_black_percentage  = (np.count_nonzero(right_view)  / right_view.size)  * 100.0

        # Swap logically if upside down
        if flip_lr:
            left_black_percentage, right_black_percentage = right_black_percentage, left_black_percentage

        center_distance = -1.0
        left_distance = -1.0
        right_distance = -1.0

        if center_black_percentage > 50.0:
            center_distance = self._colour_to_distance(center_black_percentage)
        else:
            left_distance = self._colour_to_distance(left_black_percentage)
            right_distance = self._colour_to_distance(right_black_percentage)

        if (center_distance != -1 or left_distance != -1 or right_distance != -1) and self.SHOW_DEBUG:
            logger.info(
                "Camera Percentage: Center: %.2f, Left: %.2f, Right: %.2f",
                center_black_percentage, left_black_percentage, right_black_percentage
            )
            logger.info(
                "Camera Distance: Center: %.2f, Left: %.2f, Right: %.2f",
                center_distance, left_distance, right_distance
            )

        return (
            center_black_percentage, left_black_percentage, right_black_percentage,
            center_distance, left_distance, right_distance
        )

    def _colour_to_distance(self,color_percentage:float)-> float:

        if color_percentage < 50.0:
            return -1.0
        elif color_percentage > 95.0:
            return 5.0
        elif color_percentage > 85.0:
            return 10.0
        elif color_percentage > 75.0:
            return 20.0
        elif color_percentage > 50.0:
            return 30.0
        return -1.0


class CameraCheckThread(threading.Thread, ShutdownInterface):
    """Thread to periodically check a condition and execute a callback function.
       Shuts down after callback is called once.
    """
    MAX_FPS:int = 50
    def __init__(self, call_func:Callable[[], int], fps: float):
        """
        :param call_func: Function to call periodically
        :param interval_ms: Interval in milliseconds to call the function
        """
        super().__init__()
        self.call_func = call_func
        self.currentfps = fps
        self.interval = 1.0 / fps
        self._stop_event = threading.Event()
        self._is_running = False

    def run(self):
        """Run the thread to check the condition and call the callback function."""
        self._is_running = True
        while not self._stop_event.is_set():
            counter= 0
            long_start_time = time.time()
            while counter < 100 and not self._stop_event.is_set():
                try:

                    start_time = time.time()
                    newfps =self.call_func()

                    if newfps != self.currentfps and newfps <= self.MAX_FPS:
                        self.currentfps = newfps
                        self.interval = 1.0 / newfps
                        # logger.info("Setting FPS:%0.2f", newfps)
                    elapsed_time = time.time() - start_time
                    #sleep for delta to maintain constant fps
                    # logger.info("Time for camera:%2f",elapsed_time)
                    time.sleep(max(0, self.interval - elapsed_time))
                except Exception as e:  # pylint: disable=broad-except
                    print(f"Error in CameraCheckThread: {e}")
                finally:
                    counter += 1
            #lets measure fps for each iteration
            fps = counter / (time.time() - long_start_time)
            logger.info("Effective FPS: %.2f", fps)

    def stop(self):
        """Stop the thread."""
        self._stop_event.set()
        self._is_running = False

    def is_running(self):
        """Check if the thread is currently running."""
        return self._is_running

    def shutdown(self):
        return self.stop()

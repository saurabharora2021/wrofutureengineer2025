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
    SAVE_CAMERA_IMAGE_ON_CORRECTION:bool = False
    CAMERA_OUTPUT_DIR:str= "output"
    SHOW_IMAGE:bool = False
    MIN_FPS: int = 10
    MAX_FPS: int = 20
    ORIENTATION_DEG: int = 180        # 0 or 180; swap L/R logically when 180
    SHOW_DEBUG=False
    # New: use 2/3 of the image height (tunable)
    ROI_HEIGHT_FRAC: float = 1.0/2.0  # portion of image height to analyze (e.g., 0.67)

    # Tunables for dark-grey detection (no grayscale conversion)
    USE_SECTION_THRESHOLDS: bool = True
    DARK_ABS_LUMA: int = 120        # start lower than 150; tune 110–150
    DARK_REL_FACTOR: float = 0.90   # more permissive relative threshold
    CHROMA_DIFF_MAX: int = 40       # max(B,G,R) - min(B,G,R) to consider “grey/black”

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
        counter = time.time()


        (center_p,left_p,right_p,self.camera_front,self.camera_left,self.camera_right) = \
                                    self._measure_border(frame,counter)


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

    def _save_image(self,image,counter:float,suffix:str="")-> None:

        if self.SAVE_CAMERA_IMAGE or self.SAVE_CAMERA_IMAGE_ON_CORRECTION:
            # img_rgb = image
            img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            #rotate 180
            img_rgb =  cv2.rotate(img_rgb, cv2.ROTATE_180)
            filename = os.path.join(self.CAMERA_OUTPUT_DIR, \
                            f"frame_{int(counter * 1000):013d}_{suffix}.jpg")
            cv2.imwrite(filename, img_rgb)
            if self.SHOW_DEBUG:
                logger.info("Saved: %s",filename)


    def _measure_border(self, image,counter) -> Tuple[float, float, float, float, float, float]:
        # ROI selection without rotation
        H, W = image.shape[:2]
        roi_h = max(1, int(H * self.ROI_HEIGHT_FRAC))
        if self.ORIENTATION_DEG == 180:
            roi = image[: roi_h, :]
            swap_lr = True
        else:
            roi = image[H - roi_h :, :]
            swap_lr = False

        # Fast integer luma (0..255) from BGR
        b = roi[:, :, 0].astype(np.uint16, copy=False)
        g = roi[:, :, 1].astype(np.uint16, copy=False)
        r = roi[:, :, 2].astype(np.uint16, copy=False)
        y_luma = ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint8, copy=False)

        # Low-chroma (grey) metric
        cmax = np.maximum(np.maximum(r, g), b)
        cmin = np.minimum(np.minimum(r, g), b)
        chroma = (cmax - cmin).astype(np.uint8, copy=False)

        # Split once into left / center / right (center is 50%)
        w_roi = roi.shape[1]
        section_w = w_roi // 4
        l_slice = (slice(None), slice(0, section_w))
        c_slice = (slice(None), slice(section_w, 3 * section_w))
        r_slice = (slice(None), slice(3 * section_w, w_roi))

        if self.USE_SECTION_THRESHOLDS:
            # Per-section medians -> per-section thresholds (use max => more inclusive)
            med_l = float(np.median(y_luma[l_slice]))
            med_c = float(np.median(y_luma[c_slice]))
            med_r = float(np.median(y_luma[r_slice]))
            thr_l = max(self.DARK_ABS_LUMA, int(self.DARK_REL_FACTOR * med_l))
            thr_c = max(self.DARK_ABS_LUMA, int(self.DARK_REL_FACTOR * med_c))
            thr_r = max(self.DARK_ABS_LUMA, int(self.DARK_REL_FACTOR * med_r))

            left_mask   = (y_luma[l_slice] < thr_l) & (chroma[l_slice]  <= self.CHROMA_DIFF_MAX)
            center_mask = (y_luma[c_slice] < thr_c) & (chroma[c_slice]  <= self.CHROMA_DIFF_MAX)
            right_mask  = (y_luma[r_slice] < thr_r) & (chroma[r_slice]  <= self.CHROMA_DIFF_MAX)
        else:
            # Global median -> global threshold
            med = float(np.median(y_luma))
            thr = max(self.DARK_ABS_LUMA, int(self.DARK_REL_FACTOR * med))
            mask = (y_luma < thr) & (chroma <= self.CHROMA_DIFF_MAX)
            left_mask   = mask[:, :section_w]
            center_mask = mask[:, section_w : 3 * section_w]
            right_mask  = mask[:, 3 * section_w :]

        # Percent dark per section
        left_black_percentage   = (np.count_nonzero(left_mask)   / left_mask.size)   * 100.0
        center_black_percentage = (np.count_nonzero(center_mask) / center_mask.size) * 100.0
        right_black_percentage  = (np.count_nonzero(right_mask)  / right_mask.size)  * 100.0

        if swap_lr:
            left_black_percentage, right_black_percentage = \
                    right_black_percentage, left_black_percentage

        center_distance = -1.0
        left_distance = -1.0
        right_distance = -1.0

        if center_black_percentage > 50.0 and left_black_percentage > 50.0 and \
                                                    right_black_percentage > 50.0:
            center_distance = self._colour_to_distance_center(center_black_percentage)
        else:
            left_distance = self._colour_to_distance(left_black_percentage)
            right_distance = self._colour_to_distance(right_black_percentage)

        if self.SAVE_CAMERA_IMAGE or (self.SAVE_CAMERA_IMAGE_ON_CORRECTION and \
                                      (self.camera_front != -1 or self.camera_left != -1 \
                                        or self.camera_right != -1)):
            self._save_image(roi,counter,"full")

        # logger.info(
        #     "Camera Percentage: Center: %.2f, Left: %.2f, Right: %.2f",
        #     center_black_percentage, left_black_percentage, right_black_percentage
        # )

        if (center_distance != -1 or left_distance != -1 or right_distance != -1) and \
                        self.SHOW_DEBUG:
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
        elif color_percentage > 90.0:
            return 5.0
        elif color_percentage > 80.0:
            return 10.0
        elif color_percentage > 70.0:
            return 20.0
        elif color_percentage > 50.0:
            return 30.0
        return -1.0

    def _colour_to_distance_center(self,color_percentage:float)-> float:

        if color_percentage < 60.0:
            return -1.0
        elif color_percentage > 95.0:
            return 5.0
        elif color_percentage > 85.0:
            return 10.0
        elif color_percentage > 75.0:
            return 20.0
        elif color_percentage > 60.0:
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

"""Camera Measurements"""

import os
import logging
import threading
import time
from typing import Callable, Tuple
from typing import Any, Dict
import cv2
from base.shutdown_handling import ShutdownInterface
from hardware.camera import MyCamera

logger = logging.getLogger(__name__)
class CameraDistanceMeasurements(ShutdownInterface):
    """Handles distance measurements using the camera."""

    SAVE_CAMERA_IMAGE:bool = False
    SAVE_CAMERA_IMAGE_ON_CORRECTION:bool = True
    CAMERA_OUTPUT_DIR:str= "output"
    SHOW_IMAGE:bool = False
    MIN_FPS: int = 15
    MAX_FPS: int = 30

    def __init__(self,camera: MyCamera):
        super().__init__()
        self.camera = camera
        self.camera_left = -1
        self.camera_right = -1
        self.camera_front = -1
        self.metrics: Dict[str, Any] = {}

        #init Thread
        self.camera_thread = CameraCheckThread(self.process_camera,self.MIN_FPS)



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

        (self.camera_front,self.camera_left,self.camera_right,center_d,left_d,right_d) = \
                                    self._measure_border(frame)

        # Save frame to output folder
        if self.SAVE_CAMERA_IMAGE or (self.SAVE_CAMERA_IMAGE_ON_CORRECTION and \
                                      (center_d != -1 or left_d != -1 or right_d != -1)):
            counter = time.time()
            outputdir:str = self.CAMERA_OUTPUT_DIR
            filename = os.path.join(outputdir,
                            f"frame_{counter:05d}.jpg")
            cv2.imwrite(filename, frame)
            print(f"Saved {filename}")


        #lets add extra metrics , for now we will just use the timestamp
        self.metrics['c.frontp'] = self.camera_front
        self.metrics['c.rightp'] = self.camera_right
        self.metrics['c.leftp'] = self.camera_left
        self.metrics['c.frontd'] = center_d
        self.metrics['c.rightd'] = right_d
        self.metrics['c.leftd'] = left_d

        if self.camera_front == -1 and self.camera_left == -1 and self.camera_right == -1:
            return self.MIN_FPS
        else:
            return self.MAX_FPS

    def get_distance(self,)-> Tuple[float,float,float, Dict[str, Any]]:
        """Measure distance using the camera."""
        # Implement distance measurement logic here

        return (self.camera_front,self.camera_left,self.camera_right,self.metrics)

    def _measure_border(self,image)-> Tuple[float,float,float,float,float,float]:

        img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # Take the top half of the image
        height = img_rgb.shape[0]
        top_half = img_rgb[:height // 2, :]

        # Rotate the top half 180 degrees clockwise
        img_rgb = cv2.rotate(top_half, cv2.ROTATE_180)
        bottom_half = img_rgb
        if self.SHOW_IMAGE:
            cv2.imshow('Half Image', bottom_half)
        img_rgb = bottom_half


        height, width, _ = img_rgb.shape
        section_width = width // 4

        # Calculate distances for each section
        # left section
        left_image = img_rgb[:, :section_width]
        if self.SHOW_IMAGE:
            cv2.imshow('Left Section', left_image)

        # center section
        center_image = img_rgb[:, section_width:3 * section_width]
        if self.SHOW_IMAGE:
            cv2.imshow('Center Section', center_image)

        # right section
        right_image = img_rgb[:, 3 * section_width:]
        if self.SHOW_IMAGE:
            cv2.imshow('Right Section', right_image)

        center_distance = -1
        left_distance = -1
        right_distance = -1

        #in the bottom 50% image find black percentage.
        bottom_image = center_image[height // 2:, :]
        black_pixels = cv2.inRange(bottom_image, (0, 0, 0), (50, 50, 50))
        center_black_percentage = (cv2.countNonZero(black_pixels) /
                                   (bottom_image.shape[0] * bottom_image.shape[1])) * 100

        bottom_half = left_image[height // 2:, :]
        black_pixels = cv2.inRange(bottom_half, (0, 0, 0), (50, 50, 50))
        left_black_percentage = (cv2.countNonZero(black_pixels) /
                                  (bottom_half.shape[0] * bottom_half.shape[1])) * 100

        bottom_half = right_image[height // 2:, :]
        black_pixels = cv2.inRange(bottom_half, (0, 0, 0), (50, 50, 50))
        right_black_percentage = (cv2.countNonZero(black_pixels) /
                                  (bottom_half.shape[0] * bottom_half.shape[1])) * 100

        if center_black_percentage != -1 or left_black_percentage != -1 or\
                         right_black_percentage != -1:
            logger.info("Camera Percentage: Center: %.2f, Left: %.2f, Right: %.2f",
                        center_black_percentage, left_black_percentage, right_black_percentage)

        #Do colour based distance detections.
        if center_black_percentage > 50:
            center_distance = self._colour_to_distance(center_black_percentage)
        else:
            left_distance = self._colour_to_distance(left_black_percentage)
            right_distance = self._colour_to_distance(right_black_percentage)

        logger.info("Camera Distance: Center: %.2f, Left: %.2f, Right: %.2f",
                        center_distance, left_distance, right_distance)

        return (center_black_percentage, left_black_percentage, right_black_percentage, \
                center_distance, left_distance, right_distance)

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
    MAX_FPS:int = 20
    def __init__(self, call_func:Callable[[], int], fps: float):
        """
        :param call_func: Function to call periodically
        :param interval_ms: Interval in milliseconds to call the function
        """
        super().__init__()
        self.call_func = call_func
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

                    if newfps > 1 and newfps < self.MAX_FPS:
                        self.interval = 1.0 / newfps
                    elapsed_time = time.time() - start_time
                    #sleep for delta to maintain constant fps
                    time.sleep(max(0, self.interval - elapsed_time))
                except Exception as e:  # pylint: disable=broad-except
                    print(f"Error in ConstantUpdateThread: {e}")
                finally:
                    counter += 1
            #lets measure fps for each iteration
            fps = 1.0 / (time.time() - long_start_time)
            logger.info("Effective FPS: %s", fps)
            counter = 0

    def stop(self):
        """Stop the thread."""
        self._stop_event.set()
        self._is_running = False

    def is_running(self):
        """Check if the thread is currently running."""
        return self._is_running

    def shutdown(self):
        return self.stop()

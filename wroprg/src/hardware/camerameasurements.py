"""Camera Measurements"""

import os
import logging
from typing import Tuple
from typing import Any, Dict
import cv2
from hardware.camera import MyCamera

logger = logging.getLogger(__name__)
class CameraDistanceMeasurements:
    """Handles distance measurements using the camera."""

    SAVE_CAMERA_IMAGE:bool = False
    SAVE_CAMERA_IMAGE_ON_CORRECTION:bool = True
    CAMERA_OUTPUT_DIR:str= "output"
    SHOW_IMAGE:bool = False

    def __init__(self,camera: MyCamera):
        super().__init__()
        self.camera = camera

    def start(self):
        """Start the camera."""
        self.camera.start()
        if self.SAVE_CAMERA_IMAGE or self.SAVE_CAMERA_IMAGE_ON_CORRECTION:
            outputdir:str = self.CAMERA_OUTPUT_DIR
            os.makedirs(outputdir, exist_ok=True)


    def measure_distance(self,timestamp:float)-> Tuple[float,float,float, Dict[str, Any]]:
        """Measure distance using the camera."""
        # Implement distance measurement logic here

        extra_metrics: Dict[str, Any] = {}

        frame = self.camera.capture()

        (center_p,left_p,right_p,center_d,left_d,right_d) = self._measure_border(frame)

        # Save frame to output folder
        if self.SAVE_CAMERA_IMAGE or (self.SAVE_CAMERA_IMAGE_ON_CORRECTION and \
                                      (center_d != -1 or left_d != -1 or right_d != -1)):
            counter = timestamp
            outputdir:str = self.CAMERA_OUTPUT_DIR
            filename = os.path.join(outputdir,
                            f"frame_{counter:05d}.jpg")
            cv2.imwrite(filename, frame)
            print(f"Saved {filename}")


        #lets add extra metrics , for now we will just use the timestamp
        extra_metrics['c.frontp'] = center_p
        extra_metrics['c.rightp'] = right_p
        extra_metrics['c.leftp'] = left_p
        extra_metrics['c.frontd'] = center_d
        extra_metrics['c.rightd'] = right_d
        extra_metrics['c.leftd'] = left_d

        return (center_d,left_d,right_d,extra_metrics)

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

        if center_black_percentage != -1 or left_black_percentage != -1 or right_black_percentage != -1:
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

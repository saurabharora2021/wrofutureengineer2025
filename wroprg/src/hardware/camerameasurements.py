"""Camera Measurements"""

import os
import cv2
from hardware.camera import MyCamera
from hardware.measurements import Measurement

class CameraDistanceMeasurements:
    """Handles distance measurements using the camera."""

    SAVE_CAMERA_IMAGE:bool = True
    CAMERA_OUTPUT_DIR:str= "output"

    def __init__(self,camera: MyCamera):
        super().__init__()
        self.camera = camera

    def start(self):
        """Start the camera."""
        self.camera.start()
        if self.SAVE_CAMERA_IMAGE:
            outputdir:str = self.CAMERA_OUTPUT_DIR
            os.makedirs(outputdir, exist_ok=True)


    def measure_distance(self,measurement: Measurement):
        """Measure distance using the camera."""
        # Implement distance measurement logic here

        frame = self.camera.capture()

        # Save frame to output folder
        if self.SAVE_CAMERA_IMAGE:
            counter = measurement.timestamp
            outputdir:str = self.CAMERA_OUTPUT_DIR
            filename = os.path.join(outputdir,
                            f"frame_{counter:05d}.jpg")
            cv2.imwrite(filename, frame)
            print(f"Saved {filename}")


        #lets add extra metrics , for now we will just use the timestamp
        measurement.extra_metrics['c.front'] = "0"
        measurement.extra_metrics['c.right'] = "0"
        measurement.extra_metrics['c.left'] = "0"

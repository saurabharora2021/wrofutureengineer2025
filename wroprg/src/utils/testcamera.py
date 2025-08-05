""" This script is used to test the color using the BuildHatDriveBase class."""
import logging
import argparse
import time
import cv2
from utils.camera_mock import get_camera_class

def main():
    """ Main function to run the Wro - raspberry test color Application."""

    parser = argparse.ArgumentParser(description="Wro lego - test color Application")
    parser.add_argument('--logfile', type=str, default='application.log', help='Path to log file')
    # Added debug argument
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')

    logger = logging.getLogger(__name__)

    try:

        # Use the factory function to get the appropriate camera class
        pi_camera2_class = get_camera_class()
        picam2 = pi_camera2_class()
        camera_config = picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (640, 480)})
        picam2.configure(camera_config)

        picam2.start()
        time.sleep(2)  # Allow camera to warm up

        logger.info("Camera Initialized")

        counter = 0
        while counter < 10:  # Capture 10 frames
            frame = picam2.capture_array()

            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"frame_{timestamp}.jpg"

            cv2.imwrite(filename, frame)
            logger.info("Captured frame saved as %s", filename)
            time.sleep(1)
            counter=counter+1

        picam2.stop()
        logger.info("Camera Stopped")

    except (ImportError, AttributeError, RuntimeError) as e:
        logger.error("Error Running Program")
        logger.error("Exception: %s",e)

if __name__ == "__main__":
    main()

""" This script is used to test the color using the BuildHatDriveBase class."""
import logging
import argparse
from rpi.rpi_interface import RpiInterface
from hat.legodriver import BuildHatDriveBase
from utils.helpers import HelperFunctions
from round1.logicround1 import Walker
from picamera2 import Picamera2
import cv2
import time

def main():
    """ Main function to run the Wro - raspberry test color Application."""

    parser = argparse.ArgumentParser(description="Wro lego - test color Application")
    parser.add_argument('--logfile', type=str, default='application.log', help='Path to log file')
    # Added debug argument
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    args = parser.parse_args()

    helper: HelperFunctions = HelperFunctions(args.logfile, args.debug)
    logger = logging.getLogger(__name__)

    drive_base: BuildHatDriveBase
    pi_inf: RpiInterface = helper.get_pi_interface()
    challenge1walker: Walker

    try:

        drive_base = helper.buildhat_init()
        logger.info("Drive Base Initialized")

        picam2 = Picamera2()
        camera_config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
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
            logger.info(f"Captured frame saved as {filename}")
            time.sleep(1)
            counter=counter+1

        picam2.stop()
        logger.info("Camera Stopped")

        pi_inf.force_flush_messages()

    except (ImportError, AttributeError, RuntimeError) as e:
        logger.error("Error Running Program")
        logger.error("Exception: %s",e)
        pi_inf.led1_red()
        pi_inf.buzzer_beep()
    finally:
        helper.shutdown_all()

if __name__ == "__main__":
    main()

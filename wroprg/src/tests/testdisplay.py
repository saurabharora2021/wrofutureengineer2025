""" This script is used to test distance sensor using the BuildHatDriveBase class."""
import logging
import argparse
import threading
from time import sleep
from hardware.hardware_interface import HardwareInterface
from hardware.validator import RobotValidator
from round1.logicround1 import Walker
from utils.helpers import HelperFunctions

def main():
    """ Main function to run the Wro - raspberry test distance sensor Application."""

    parser = argparse.ArgumentParser(description="Wro lego - test distance sensor Application")
    parser.add_argument('--logfile', type=str, default='application.log', help='Path to log file')
    # Added debug argument
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    args = parser.parse_args()

    helper: HelperFunctions = HelperFunctions(args.logfile, args.debug)
    logger = logging.getLogger(__name__)

    pi_inf: HardwareInterface = helper.get_pi_interface()

    try:

        pi_inf.force_flush_messages()
        #action button.
        pi_inf.buzzer_beep()
        # pi_inf.wait_for_action()
        sleep(1)

        pi_inf.start_measurement_recording()

        def run_gyro_walk():

            while True:
                pi_inf.add_screen_logger_message(["Hello Line 1","Hello Line 2","Hello Line 3"])
                sleep(1)

        # Start gyro walk in a separate thread
        gyro_thread = threading.Thread(target=run_gyro_walk)
        gyro_thread.start()

        # In main thread, call wait_for_action()
        while (pi_inf.is_button_pressed() is False and gyro_thread.is_alive()):
            sleep(0.01)

        # Optionally, wait for the gyro walk thread to finish
        # gyro_thread.join()

    except (ImportError, AttributeError, RuntimeError) as e:
        logger.error("Error Running Program")
        logger.error("Exception: %s",e)
        pi_inf.led1_red()
        pi_inf.buzzer_beep()
        raise
    finally:
        helper.shutdown_all()

if __name__ == "__main__":
    main()

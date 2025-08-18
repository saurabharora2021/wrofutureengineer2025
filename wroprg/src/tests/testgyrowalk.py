""" This script is used to test distance sensor using the BuildHatDriveBase class."""
import logging
import argparse
import threading
from hardware.hardware_interface import HardwareInterface
from round1.logicround1 import Walker
from round1.matintelligence import MATDIRECTION, MatIntelligence
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

        challenge1walker = Walker(pi_inf)
        intel: MatIntelligence = MatIntelligence()
        pi_inf.start_measurement_recording()

        #action button.
        pi_inf.wait_for_action()

        def run_gyro_walk():

            #lets assume this is AntiClockwise and side1 is complete, we have reached corner1
            intel.report_direction_side1(MATDIRECTION.ANTICLOCKWISE_DIRECTION)

            challenge1walker.gyro_corner_walk_round_1(def_turn_angle=60)

        # Start gyro walk in a separate thread
        gyro_thread = threading.Thread(target=run_gyro_walk)
        gyro_thread.start()

        # In main thread, call wait_for_action()
        pi_inf.wait_for_action()

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

""" This script is used to test distance sensor using the BuildHatDriveBase class."""
import logging
import argparse
import threading
from hardware.hardware_interface import HardwareInterface
from round1.logicroundn import WalkerN
from utils.helpers import HelperFunctions
from utils.mat import MATDIRECTION, MATLOCATION

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

     # Default distances for clockwise equidistance walking
    default_distances_clockwise = {
        # left, right are not known at this point.
        MATLOCATION.SIDE_1: (100.0,47.0,47.0),
        MATLOCATION.CORNER_1: (15.0,35.0,30.0),
        MATLOCATION.SIDE_2: (100.0,47.0,47.0),
        MATLOCATION.CORNER_2: (15.0,35.0,30.0),
        MATLOCATION.SIDE_3: (100.0,47.0,47.0),
        MATLOCATION.CORNER_3: (15.0,35.0,30.0),
        MATLOCATION.SIDE_4: (100.0,47.0,47.0),
        MATLOCATION.CORNER_4: (15.0,35.0,30.0),
    }

    try:

        pi_inf.force_flush_messages()
        pi_inf.reset_steering()

        roundwalker:WalkerN = WalkerN(output_inf=pi_inf,nooflaps=2)

        roundwalker.intelligence.report_direction_side1(MATDIRECTION.CLOCKWISE_DIRECTION)
        roundwalker.intelligence.set_location(MATLOCATION.CORNER_4)
        roundwalker.intelligence.set_default_distances(default_distances_clockwise)
        roundwalker.intelligence.reprocess_map()
        roundwalker.intelligence.set_roundno(2)
        roundwalker.intelligence.set_location(MATLOCATION.SIDE_1)
        roundwalker.intelligence.print_mat_intelligence()


        def run_gyro_walk():

            pi_inf.start_measurement_recording()

            roundwalker.full_gyro_walk()

            pi_inf.drive_stop()
            pi_inf.reset_steering()


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

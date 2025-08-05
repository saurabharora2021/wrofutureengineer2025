""" This script is used to test the Orientation Functions for mpu6050 ."""
import logging
import argparse
from time import sleep
from hardware.hardware_interface import HardwareInterface
from utils.helpers import HelperFunctions

def main():
    """ Main function."""

    parser = argparse.ArgumentParser(description="Wro lego - test color Application")
    parser.add_argument('--logfile', type=str, default='application.log', help='Path to log file')
    # Added debug argument
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    args = parser.parse_args()

    helper: HelperFunctions = HelperFunctions(args.logfile, args.debug,stabilize=False)
    logger = logging.getLogger(__name__)

    pi_inf: HardwareInterface = helper.get_pi_interface()
    try:

        pi_inf.force_flush_messages()
        pi_inf.start_measurement_recording()
        sleep(2)
        pi_inf.reset_yaw()
        default_yaw = pi_inf.get_orientation()[2]
        logger.warning("Default Yaw: %.2f", default_yaw)
        while True:
            # Read orientation data
            orientation = pi_inf.get_orientation()
            if orientation is not None:
                delta_yaw = orientation[2] - default_yaw
                logger.warning("Yaw delta: %.2f ", delta_yaw)
                print("Orientation: %s", orientation)
            else:
                logger.warning("Orientation data not available")
                print("Orientation data not available")

            sleep(1)

    except (ImportError, AttributeError, RuntimeError) as e:
        logger.error("Error Running Program")
        logger.error("Exception: %s",e)
        pi_inf.led1_red()
        pi_inf.buzzer_beep()
    finally:
        helper.shutdown_all()

if __name__ == "__main__":
    main()

"""Test script for reading data from the MPU6050 sensor."""
import time
import argparse
import logging

from hardware.hardware_interface import HardwareInterface
from utils.helpers import HelperFunctions


def main():
    """ Main function to run the Wro - raspberry test color Application."""

    parser = argparse.ArgumentParser(description="Wro lego - test color Application")
    parser.add_argument('--logfile', type=str, default='application.log', help='Path to log file')
    # Added debug argument
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    args = parser.parse_args()

    helper: HelperFunctions = HelperFunctions(args.logfile, args.debug,stabilize=False)
    logger = logging.getLogger(__name__)

    pi_inf: HardwareInterface = helper.get_pi_interface()
    try:

        while True:
            accel  = pi_inf.get_acceleration()
            gyro   = pi_inf.get_gyro()
            print(f"Accel (m/s^2): x={accel[0]:.2f}, y={accel[1]:.2f}, z={accel[2]:.2f}")
            print(f"Gyro (rad/s):  x={gyro[0]:.2f}, y={gyro[1]:.2f}, z={gyro[2]:.2f}")
            print("-" * 40)
            time.sleep(1)

    except (ImportError, AttributeError, RuntimeError) as e:
        logger.error("Error Running Program")
        logger.error("Exception: %s",e)
        pi_inf.led1_red()
        pi_inf.buzzer_beep()
    finally:
        helper.shutdown_all()

if __name__ == "__main__":
    main()

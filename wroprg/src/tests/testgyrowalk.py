""" This script is used to test distance sensor using the BuildHatDriveBase class."""
import logging
from hardware.hardware_interface import HardwareInterface
from round1.logicround1 import Walker
from utils.helpers import HelperFunctions
from utils.mat import MATDIRECTION

def main():
    """ Main function to run the Wro - raspberry test distance sensor Application."""

    helper: HelperFunctions = HelperFunctions(stabilize=True)
    logger = logging.getLogger(__name__)
    pi_inf: HardwareInterface = helper.get_pi_interface()

    pi_inf.force_flush_messages()

    challenge1walker = Walker(pi_inf)
    pi_inf.start_measurement_recording()
    pi_inf.reset_gyro()


    def runner():
        #lets assume this is AntiClockwise and side1 is complete, we have reached corner1
        challenge1walker.intelligence.\
            report_direction_side1(MATDIRECTION.CLOCKWISE_DIRECTION)

        challenge1walker.handle_corner(gyrodefault=0)

    helper.start_application(runner)

if __name__ == "__main__":
    main()

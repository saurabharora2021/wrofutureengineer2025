""" This script is used to reset the front wheel of a robot using the BuildHatDriveBase class."""
import logging
from hardware.hardware_interface import HardwareInterface
from utils.helpers import HelperFunctions

def main():
    """ Main function to run the Wro - raspberry reset Front Wheel Application."""

    helper: HelperFunctions = HelperFunctions()
    logger = logging.getLogger(__name__)
    pi_inf: HardwareInterface = helper.get_pi_interface()

    def runner():

        pi_inf.force_flush_messages()
        pi_inf.get_bottom_color()
        logger.info("steering angle: %.2f", pi_inf.get_steering_angle())


    helper.start_application(runner)

if __name__ == "__main__":
    main()

""" This script is used to test the Buzzer using the BuildHatDriveBase class."""
import logging
from hardware.hardware_interface import HardwareInterface
from utils.helpers import HelperFunctions

def main():
    """ Main function to run the Wro - raspberry test color Application."""

    helper: HelperFunctions = HelperFunctions()
    logger = logging.getLogger(__name__)
    pi_inf: HardwareInterface = helper.get_pi_interface()

    def runner():

        logger.warning("Hello World Screen is working")
        pi_inf.force_flush_messages()

    helper.start_application(runner)

if __name__ == "__main__":
    main()

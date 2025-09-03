""" This script is used to test distance sensor using the BuildHatDriveBase class."""
import logging
from hardware.hardware_interface import HardwareInterface
from utils.helpers import HelperFunctions

def main():
    """ Main function to run the Wro - raspberry test distance sensor Application."""

    helper: HelperFunctions = HelperFunctions(stabilize=True)
    logger = logging.getLogger(__name__)
    pi_inf: HardwareInterface = helper.get_pi_interface()

    def runner():

        front_distance = pi_inf.read_state().front
        logger.warning("Front Distance: %d cm", front_distance)

        pi_inf.force_flush_messages()


    helper.start_application(runner)

if __name__ == "__main__":
    main()

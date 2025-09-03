""" This script is used to test the Buzzer using the BuildHatDriveBase class."""
import logging
from time import sleep
from hardware.hardware_interface import HardwareInterface
from utils.helpers import HelperFunctions

def main():
    """ Main function to run the Wro - raspberry test color Application."""

    helper: HelperFunctions = HelperFunctions()
    logger = logging.getLogger(__name__)
    pi_inf: HardwareInterface = helper.get_pi_interface()

    def runner():
        logger.warning("Current Jumper State:%s", pi_inf.get_jumper_state())
        sleep(1)  # Wait for a second to ensure the state is read correctly
        pi_inf.force_flush_messages()

    helper.start_application(runner)

if __name__ == "__main__":
    main()

""" This script is used to test distance sensor using the BuildHatDriveBase class."""
import logging
from time import sleep
from hardware.hardware_interface import HardwareInterface
from utils.helpers import HelperFunctions

def main():
    """ Main function to run the Wro - raspberry test distance sensor Application."""


    helper: HelperFunctions = HelperFunctions()
    logger = logging.getLogger(__name__)
    pi_inf: HardwareInterface = helper.get_pi_interface()

    pi_inf.force_flush_messages()
    pi_inf.buzzer_beep()
    sleep(1)
    pi_inf.start_measurement()

    def runner():
        while True:
            pi_inf.add_screen_logger_message(["Hello Line 1","Hello Line 2","Hello Line 3"])
            sleep(1)

    helper.start_application(runner)


if __name__ == "__main__":
    main()

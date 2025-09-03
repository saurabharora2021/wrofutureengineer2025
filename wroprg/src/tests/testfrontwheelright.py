""" This script is used to reset the front wheel of a robot using the BuildHatDriveBase class."""
import logging
from time import sleep
from hardware.hardware_interface import HardwareInterface
from utils.helpers import HelperFunctions

def main():
    """ Main function to run the Wro - raspberry reset Front Wheel Application."""

    helper: HelperFunctions = HelperFunctions()
    logger = logging.getLogger(__name__)
    pi_inf: HardwareInterface = helper.get_pi_interface()

    def runner():

        pi_inf.force_flush_messages()
        #Turn steering to right 15 degrees
        pi_inf.turn_steering(30)
        pi_inf.buzzer_beep()
        logger.error("Current Steering %.2f", pi_inf.get_steering_angle())
        sleep(6)
        pi_inf.reset_steering()

    helper.start_application(runner)


if __name__ == "__main__":
    main()

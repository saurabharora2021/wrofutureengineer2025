""" This script is used to test the color using the BuildHatDriveBase class."""
import logging
import time
from utils.mat import mat_color
from utils.helpers import HelperFunctions
from hardware.hardware_interface import HardwareInterface

def main():
    """ Main function to run the Wro - raspberry test color Application."""

    helper: HelperFunctions = HelperFunctions()
    logger = logging.getLogger(__name__)
    pi_inf: HardwareInterface = helper.get_pi_interface()

    def runner():

        pi_inf.buzzer_beep()
        time.sleep(1)
        logger.info("Starting color sensor test...")
        r, g, b, i ,*other = pi_inf.get_bottom_color_rgbi()

        logger.info("Bottom Color RGBI: R=%d, G=%d, B=%d, I=%d", r, g, b, i)
        logger.debug("Remaining data: %s", other)
        color = mat_color(r, g, b)
        logger.warning("Detected color: %s", color)

        pi_inf.force_flush_messages()

    helper.start_application(runner)

if __name__ == "__main__":
    main()

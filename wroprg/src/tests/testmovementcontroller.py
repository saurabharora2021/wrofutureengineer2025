""" This script is used to test distance calculation using the BuildHatDriveBase class."""
import logging
import time
from hardware.hardware_interface import HardwareInterface
from round1.movement_controller import MovementController
from utils.helpers import HelperFunctions

def main():
    """ Main function to run the Wro - raspberry test distance calculation Application."""

    helper: HelperFunctions = HelperFunctions(stabilize=True)
    logger = logging.getLogger(__name__)
    pi_inf: HardwareInterface = helper.get_pi_interface()

    controller = MovementController(pi_inf, 20)

    def runner():

        controller.reset_distance()
        front_distance = pi_inf.read_state().front
        logger.info("Starting distance: %2f cm", front_distance)
        controller.start_walking(30)
        time.sleep(3)
        controller.stop_walking()
        end_front_distance = pi_inf.read_state().front
        logger.info("Ending distance: %2f cm", end_front_distance)
        logger.info(" ultra Distance: %d cm", end_front_distance - front_distance)
        logger.info(" wheel Distance: %d cm", controller.get_distance())


    helper.start_application(runner)

if __name__ == "__main__":
    main()

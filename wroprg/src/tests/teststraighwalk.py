""" This script is used to test distance sensor using the BuildHatDriveBase class."""
import logging
from time import sleep
from hardware.hardware_interface import HardwareInterface
from hardware.validator import RobotValidator
from round1.logicround1 import Walker
from utils.helpers import HelperFunctions

def main():
    """ Main function to run the Wro - raspberry test distance sensor Application."""

    helper: HelperFunctions = HelperFunctions()
    logger = logging.getLogger(__name__)
    pi_inf: HardwareInterface = helper.get_pi_interface()

    pi_inf.force_flush_messages()
    #action button.
    pi_inf.buzzer_beep()
    # pi_inf.wait_for_action()
    sleep(1)

    # Validate the robot's functionality
    robot_validator: RobotValidator = RobotValidator(pi_inf)
    if not robot_validator.validate():
        logger.error("Robot validation failed. Exiting.")
        pi_inf.led1_red()
        pi_inf.buzzer_beep()
        raise RuntimeError("Robot validation failed")
    else:
        pi_inf.led1_green()
        pi_inf.buzzer_beep()

    challenge1walker = Walker(pi_inf)
    pi_inf.start_measurement_recording()
    # Log the distances
    start_state = pi_inf.read_state()

    gyrodefault = 0

    maxfront = 100

    logger.info("Max front distance: %.2f", maxfront)

    def runner():

        pi_inf.reset_gyro()  # Reset gyro to zero
        challenge1walker.handle_straight_walk_to_distance(maxfront,start_state.left,
                                                            start_state.right,
                                            gyrodefault,Walker.DEFAULT_SPEED,speedcheck=True)
        pi_inf.drive_stop()


    helper.start_application(runner)

if __name__ == "__main__":
    main()

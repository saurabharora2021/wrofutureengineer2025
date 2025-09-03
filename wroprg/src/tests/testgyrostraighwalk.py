""" This script is used to test distance sensor using the BuildHatDriveBase class."""
import logging
from time import sleep
from hardware.hardware_interface import HardwareInterface
from hardware.validator import RobotValidator
from round1.logicround1 import Walker
from round1.walker_helpers import GyroWalkerwithMinDistanceHelper
from utils.helpers import HelperFunctions

def main():
    """ Main function to run the Wro - raspberry test distance sensor Application."""

    helper: HelperFunctions = HelperFunctions(stabilize=True)
    logger = logging.getLogger(__name__)
    pi_inf: HardwareInterface = helper.get_pi_interface()

    pi_inf.force_flush_messages()
    pi_inf.buzzer_beep()
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
    pi_inf.start_measurement()
    # Log the distances
    start_state = pi_inf.read_state()

    gyrodefault = 0

    maxfront = 120

    DEFAULT_SPEED=30

    logger.info("Max front distance: %.2f", maxfront)

    def runner():

        pi_inf.reset_gyro()  # Reset gyro to zero
        gyrohelper: GyroWalkerwithMinDistanceHelper = GyroWalkerwithMinDistanceHelper(
            def_turn_angle=0, min_left=20, min_right=20)

        challenge1walker.handle_straight_walk_to_distance(maxfront,start_state.left,
                                                            start_state.right, gyrodefault,
                                                            DEFAULT_SPEED,speedcheck=True,
                                                            base_helper=gyrohelper)
        pi_inf.drive_stop()

    helper.start_application(runner)

if __name__ == "__main__":
    main()

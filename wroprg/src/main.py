"""Main application for the Wro - Raspberry Pi interface."""
import logging

from hardware.robotstate import RobotState
from hardware.validator import RobotValidator
from hardware.hardware_interface import HardwareInterface
from round1.logicroundn import WalkerN

from utils.helpers import HelperFunctions


def main():
    """ Main function to run the Wro - raspberry Application."""

    helper: HelperFunctions = HelperFunctions(stabilize=True,screen_logger=False)
    logger = logging.getLogger(__name__)
    pi_inf: HardwareInterface = helper.get_pi_interface()

    pi_inf.force_flush_messages()

    # Validate the robot's functionality
    robot_validator: RobotValidator = RobotValidator(pi_inf)
    pi_inf.disable_logger()
    state:RobotState = pi_inf.read_state()
    pi_inf.log_message(state.front, state.left, state.right, current_yaw=0,
                            current_steering=pi_inf.get_steering_angle())
    if not robot_validator.validate():

        pi_inf.led1_red()
        pi_inf.buzzer_beep()
        raise RuntimeError("Robot validation failed")
    else:
        pi_inf.led1_green()
        pi_inf.buzzer_beep()

    logger.warning("Test Successful")
    pi_inf.reset_steering()
    pi_inf.force_flush_messages()

    walker = WalkerN(pi_inf,nooflaps=2)

    #action button.
    state = pi_inf.read_state()
    pi_inf.log_message(state.front, state.left, state.right, current_yaw=0,
                            current_steering=pi_inf.get_steering_angle())


    def runner():

        pi_inf.start_measurement()
        walker.start_walk()

    helper.start_application(runner,use_button=True)

if __name__ == "__main__":
    main()

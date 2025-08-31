"""Main application for the Wro - Raspberry Pi interface."""
import logging
import argparse
import threading
from time import sleep

from hardware.robotstate import RobotState
from hardware.validator import RobotValidator
from hardware.hardware_interface import HardwareInterface
from round1.logicroundn import WalkerN

from utils.helpers import HelperFunctions


def main():
    """ Main function to run the Wro - raspberry Application."""

    parser = argparse.ArgumentParser(description="Wro lego - raspberry Application")
    parser.add_argument('--logfile', type=str, default='application.log', help='Path to log file')
    # Added debug argument
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    args = parser.parse_args()

    print(f"Log file: {args.logfile}")
    print(f"Debug mode: {args.debug}")  # Optional: print debug status

    helper: HelperFunctions = HelperFunctions(args.logfile, args.debug,screen_logger=False)
    logger = logging.getLogger(__name__)

    pi_inf: HardwareInterface = helper.get_pi_interface()

    try:

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
        pi_inf.force_flush_messages()

        challenge1walker = WalkerN(pi_inf,nooflaps=2)

        #action button.
        state = pi_inf.read_state()
        pi_inf.log_message(state.front, state.left, state.right, current_yaw=0,
                               current_steering=pi_inf.get_steering_angle())
        pi_inf.wait_for_action()
        logger.info("===============STARTING BOT===================")
        pi_inf.buzzer_beep()

        def runner():
            
            pi_inf.start_measurement_recording()
            challenge1walker.start_walk()

        # Start gyro walk in a separate thread
        run_thread = threading.Thread(target=runner)
        run_thread.start()
        sleep(2)

        # In main thread, call wait_for_action()
        while (pi_inf.is_button_pressed() is False and run_thread.is_alive()):
            sleep(0.01)

    except (ImportError, AttributeError, RuntimeError) as e:
        logger.error("Exception: %s",e)
        pi_inf.led1_red()
        state = pi_inf.read_state()
        pi_inf.log_message(state.front, state.left, state.right, current_yaw=0,
                               current_steering=pi_inf.get_steering_angle())
        raise
    finally:
        helper.shutdown_all()

if __name__ == "__main__":
    main()

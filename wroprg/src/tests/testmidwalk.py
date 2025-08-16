""" This script is used to test distance sensor using the BuildHatDriveBase class."""
import logging
import argparse
import threading
from time import sleep
from hardware.hardware_interface import HardwareInterface
from hardware.validator import RobotValidator
from round1.logicround1 import Walker
from utils.helpers import HelperFunctions

def main():
    """ Main function to run the Wro - raspberry test distance sensor Application."""

    parser = argparse.ArgumentParser(description="Wro lego - test distance sensor Application")
    parser.add_argument('--logfile', type=str, default='application.log', help='Path to log file')
    # Added debug argument
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    args = parser.parse_args()

    helper: HelperFunctions = HelperFunctions(args.logfile, args.debug)
    logger = logging.getLogger(__name__)

    pi_inf: HardwareInterface = helper.get_pi_interface()

    try:

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
        start_left_distance = pi_inf.get_left_distance()
        start_right_distance = pi_inf.get_right_distance()

        gyrodefault = 0
        deltadistance = start_right_distance - start_left_distance

        #If Delta is high move towards the center, move by 10cm otherwise too high correction.
        if abs(deltadistance)> 10:
            if start_left_distance < start_right_distance:
                logger.info("Adjusting left distance")
                start_left_distance += 10
                start_right_distance -= 10
                # gyrodefault = -1
            else:
                logger.info("Adjusting right distance")
                start_left_distance -= 10
                start_right_distance += 10
                # gyrodefault = 1
            logger.info("adjusted left %.2f , right %.2f",start_left_distance,start_right_distance)



        maxfront = 120

        logger.info("Max front distance: %.2f", maxfront)

        def run_gyro_walk():

            pi_inf.reset_gyro()  # Reset gyro to zero

            challenge1walker.handle_straight_walk_to_distance(maxfront,start_left_distance,
                                                              start_right_distance,
                                              gyrodefault,Walker.MIN_SPEED,speedcheck=True)
            pi_inf.drive_stop()

        # Start gyro walk in a separate thread
        gyro_thread = threading.Thread(target=run_gyro_walk)
        gyro_thread.start()

        # In main thread, call wait_for_action()
        while (pi_inf.is_button_pressed() is False and gyro_thread.is_alive()):
            sleep(0.01)

        # Optionally, wait for the gyro walk thread to finish
        # gyro_thread.join()

    except (ImportError, AttributeError, RuntimeError) as e:
        logger.error("Error Running Program")
        logger.error("Exception: %s",e)
        pi_inf.led1_red()
        pi_inf.buzzer_beep()
        raise
    finally:
        helper.shutdown_all()

if __name__ == "__main__":
    main()

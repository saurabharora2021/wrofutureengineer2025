""" This script is used to test the Orientation Functions for mpu6050 ."""
import logging
import argparse
from time import sleep
from hardware.robotstate import RobotState
from hardware.hardware_interface import HardwareInterface
from utils.helpers import HelperFunctions

def main():
    """ Main function."""

    parser = argparse.ArgumentParser(description="Wro lego - test color Application")
    parser.add_argument('--logfile', type=str, default='application.log', help='Path to log file')
    # Added debug argument
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    args = parser.parse_args()

    helper: HelperFunctions = HelperFunctions(args.logfile, args.debug,stabilize=True)
    logger = logging.getLogger(__name__)

    pi_inf: HardwareInterface = helper.get_pi_interface()
    try:
        pi_inf.buzzer_beep()
        pi_inf.force_flush_messages()
        pi_inf.start_measurement_recording()
        sleep(2)
        pi_inf.reset_gyro()
        default_yaw = pi_inf.get_yaw()
        logger.warning("Default Yaw: %.2f", default_yaw)
        while True:
            # Read orientation data
            yaw  = pi_inf.get_yaw()
            state:RobotState = pi_inf.read_state()
            right_distance = pi_inf.get_right_lidar_distance()
            left_distance = pi_inf.get_left_lidar_distance()
            right_ultra = pi_inf.get_right_ultra_distance()
            left_ultra = pi_inf.get_left_ultra_distance()

            logger.info("yaw:%0.2f", yaw)
            pi_inf.display_message(f"Yaw: {yaw:.2f}",forceflush=True)
            logger.info("State: Front: %.2f, Left: %.2f, Right: %.2f Camera F:%.2f, Camera L:%.2f,"\
                            +"Camera R:%.2f", state.front, state.left, state.right, \
                            state.camera_front,state.camera_left, state.camera_right)
            logger.info("Left Lidar: %.2f , Ultra: %.2f", left_distance,left_ultra)
            logger.info("Right Lidar: %.2f, Ultra: %.2f", right_distance, right_ultra)

            pi_inf.force_flush_messages()
            sleep(1)

    except (ImportError, AttributeError, RuntimeError) as e:
        logger.error("Error Running Program")
        logger.error("Exception: %s",e)
        pi_inf.led1_red()
        pi_inf.buzzer_beep()
    finally:
        helper.shutdown_all()

if __name__ == "__main__":
    main()

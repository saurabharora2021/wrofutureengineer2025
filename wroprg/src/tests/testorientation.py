""" This script is used to test the Orientation Functions for mpu6050 ."""
import logging
from time import sleep
from hardware.robotstate import RobotState
from hardware.hardware_interface import HardwareInterface
from utils.helpers import HelperFunctions

def main():
    """ Main function."""

    helper: HelperFunctions = HelperFunctions(stabilize=True)
    logger = logging.getLogger(__name__)
    pi_inf: HardwareInterface = helper.get_pi_interface()

    def runner():

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


    helper.start_application(runner)

if __name__ == "__main__":
    main()

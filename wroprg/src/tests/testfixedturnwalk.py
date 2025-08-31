""" This script is used to test distance sensor using the BuildHatDriveBase class."""
import logging
from hardware.hardware_interface import HardwareInterface
from round1.logicroundn import WalkerN
from utils.helpers import HelperFunctions
from utils.mat import MATDIRECTION, MATLOCATION

def main():
    """ Main function to run the Wro - raspberry test distance sensor Application."""


    helper: HelperFunctions = HelperFunctions(stabilize=True)
    logger = logging.getLogger(__name__)
    pi_inf: HardwareInterface = helper.get_pi_interface()


    def runner():

        # Default distances for clockwise equidistance walking
        default_distances_clockwise = {
            # left, right are not known at this point.
            MATLOCATION.SIDE_1: (100.0,47.0,47.0),
            MATLOCATION.CORNER_1: (15.0,35.0,30.0),
            MATLOCATION.SIDE_2: (100.0,47.0,47.0),
            MATLOCATION.CORNER_2: (15.0,35.0,30.0),
            MATLOCATION.SIDE_3: (100.0,47.0,47.0),
            MATLOCATION.CORNER_3: (15.0,35.0,30.0),
            MATLOCATION.SIDE_4: (100.0,47.0,47.0),
            MATLOCATION.CORNER_4: (15.0,35.0,30.0),
        }

        pi_inf.force_flush_messages()
        pi_inf.reset_steering()

        roundwalker:WalkerN = WalkerN(output_inf=pi_inf)

        roundwalker.intelligence.report_direction_side1(MATDIRECTION.CLOCKWISE_DIRECTION)
        roundwalker.intelligence.set_location(MATLOCATION.SIDE_1)
        roundwalker.intelligence.set_default_distances(default_distances_clockwise)
        roundwalker.intelligence.location_complete()
        roundwalker.intelligence.print_mat_intelligence()

        logger.info("=========start walking ............")
        roundwalker.handle_gyro_corner(final_yaw=-roundwalker.CORNER_YAW_ANGLE,
                                        current_speed=roundwalker.CORNER_GYRO_SPEED
                                        )

        pi_inf.drive_stop()
        pi_inf.reset_steering()



    helper.start_application(runner)


if __name__ == "__main__":
    main()

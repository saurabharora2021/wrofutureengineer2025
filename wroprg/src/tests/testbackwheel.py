""" This script is used to reset the back wheel of a robot using the BuildHatDriveBase class."""
from time import sleep
from hardware.hardware_interface import HardwareInterface
from utils.helpers import HelperFunctions

def main():
    """ Main function to run the Wro - raspberry back wheel Application."""

    helper: HelperFunctions = HelperFunctions()

    pi_inf: HardwareInterface = helper.get_pi_interface()

    def runner():

        pi_inf.start_measurement_recording()
        pi_inf.drive_forward(100)
        sleep(10)  # Allow the motor to run for a while
        pi_inf.drive_stop()
        pi_inf.force_flush_messages()
        pi_inf.get_bottom_color()

    helper.start_application(runner)



if __name__ == "__main__":
    main()

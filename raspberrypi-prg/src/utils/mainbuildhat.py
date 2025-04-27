from buildhat import Motor
from buildhat.drivebase import DriveBase

def main():
    # Create motor instances for left and right motors
    left_motor = Motor('A')
    right_motor = Motor('B')
    
    # Create an instance of BuildHatDriveBase
    drive_base = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=114)
    
    # Example usage: Move forward for 500mm
    drive_base.straight(500)
    
    # Example usage: Turn 90 degrees
    drive_base.turn(90)

if __name__ == "__main__":
    main()
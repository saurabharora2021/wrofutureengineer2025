from spikeserial import SpikeDriveBase
from base import DriveBase

def main():
    # Create an instance of BuildHatDriveBase
    drive_base:DriveBase = SpikeDriveBase()
    
    # Example usage: Move forward for 500mm
    drive_base.straight(500)
    
    # Example usage: Turn 90 degrees
    drive_base.turn(90)

if __name__ == "__main__":
    main()
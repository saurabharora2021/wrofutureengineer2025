

import time
import adafruit_tca9548a
import busio
import logging
from board import SCL, SDA
import adafruit_bno055


logger = logging.getLogger(__name__)
class ScratchBNo:

    def __init__(self):
                

        # Initialize I2C
        DEVICE_I2C_CHANNEL = 6

        # Setup I2C devices
        i2c = busio.I2C(SCL, SDA)
        tca = adafruit_tca9548a.TCA9548A(i2c)

        for channel in range(8):
            if tca[channel].try_lock(): # pyright: ignore[reportArgumentType]
                print("Channel %s:", channel)
                addresses = tca[channel].scan() # pyright: ignore[reportArgumentType]
                print([hex(address) for address in addresses if address != 0x70])
                tca[channel].unlock() # pyright: ignore[reportArgumentType]


        device_channel = tca[DEVICE_I2C_CHANNEL]

        # Initialize BNO055
        self.sensor = adafruit_bno055.BNO055_I2C(device_channel)
        self.yaw_offset = 0.0  # Offset to apply to yaw readings

    
    def reset_yaw(self):
        euler = self.sensor.euler
        if euler is not None:
            self.yaw_offset = euler[0] or 0.0  # store current yaw

    def get_yaw(self):
        euler = self.sensor.euler
        if euler is not None:
            yaw = (euler[0] or 0.0) - self.yaw_offset
            # Normalize angle to [-180, 180]
            if yaw > 180:
                yaw -= 360
            elif yaw < -180:
                yaw += 360
            return yaw
        return None

        print("BNO055 Yaw Example")


    def main(self):

        while True:
            # Euler angles are returned as (heading, roll, pitch)
            # heading = yaw
            yaw =   self.get_yaw()
            if yaw is not None:
                print("Yaw: {:.2f},".format(yaw))
            else:
                print("No data")
            time.sleep(0.1)


def main():
    """Main method for orientation."""
    logging.basicConfig(level=logging.INFO)
    # Use module-level logger

    bno = ScratchBNo()
    bno.main()

if __name__ == "__main__":
    main()

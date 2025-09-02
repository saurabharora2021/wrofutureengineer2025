

import time
import adafruit_tca9548a
import busio
from board import SCL, SDA
import adafruit_bno055



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
sensor = adafruit_bno055.BNO055_I2C(device_channel)

print("BNO055 Yaw Example")

while True:
    # Euler angles are returned as (heading, roll, pitch)
    # heading = yaw
    euler = sensor.euler
    if euler is not None:
        yaw, roll, pitch = euler  # unpack values
        print("Yaw: {:.2f}, Roll: {:.2f}, Pitch: {:.2f}".format(yaw, roll, pitch))
    else:
        print("No data")
    time.sleep(0.1)

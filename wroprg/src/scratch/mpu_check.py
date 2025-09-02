"""Quick MPU6050 raw vs scaled check helper.
Run on target Raspberry Pi with I2C available.
"""
import time
from board import SCL, SDA
import busio
from hardware.mpu6050 import MPU6050
import adafruit_tca9548a

def main():
    print("MPU raw vs scaled check")
    DEVICE_I2C_CHANNEL = 6

    # Setup I2C devices
    i2c = busio.I2C(SCL, SDA)
    tca = adafruit_tca9548a.TCA9548A(i2c)

    device_channel = tca[DEVICE_I2C_CHANNEL]

    mpu = MPU6050(device_channel)

    for i in range(200):
        mpu.read()
        print(f"raw={mpu.gyro_raw}, scaled={mpu.gyro}")
        time.sleep(0.02)

if __name__ == '__main__':
    main()

""" This script is used to test the Orientation Functions for mpu6050 ."""
import logging
import argparse
from time import sleep
from board import SCL, SDA
import busio
import math
import adafruit_mpu6050
from qmc5883l import QMC5883L
from hardware.orientation import OrientationEstimator

def main():
    """ Main function."""

    i2c = busio.I2C(SCL,SDA)  # uses board.SCL and board.SDA
    mpu = adafruit_mpu6050.MPU6050(i2c)
    qmc  =  QMC5883L(i2c)

    def get_acceleration():
        return mpu.acceleration

    def get_gyro():
        return mpu.gyro

    def get_magnetic():
        return qmc.magnetic

    estimator:OrientationEstimator = OrientationEstimator(get_accel=get_acceleration, get_gyro=get_gyro, get_mag=get_magnetic)# get_mag=qmc.magnetic)

    # estimator.calibrate_imu()
    print("starting reading...")    
    estimator.start_readings()

    while True:
        # Update the orientation estimate
        estimator.update()
        (roll,pitch,yaw)= estimator.get_orientation()
        print("Orientation: ", round(yaw, 2))
        sleep(1)




if __name__ == "__main__":
    main()

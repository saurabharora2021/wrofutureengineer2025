"""
MPU6050 Calibration Script
"""
import time
import board
import busio
import adafruit_mpu6050

def calibrate_mpu6050(samples=1000, delay=0.01):
    """
    Calibrate the MPU6050 sensor.
    """
    # Initialize I2C and MPU6050
    i2c = busio.I2C(board.SCL, board.SDA)
    mpu = adafruit_mpu6050.MPU6050(i2c)

    default_accel_offsets = [8.858546142578142e-05, -0.20067958895263652, 0.8191784922241325]
    default_gyro_offsets = [-0.12944187766481732, 0.01594671365112254, -0.04114686988758967]

    print("Collecting calibration data...")
    accel_offsets = [0,0,0]
    gyro_offsets = [0,0,0]

    for _ in range(samples):
        accel = mpu.acceleration
        gyro = mpu.gyro
        for i in range(3):
            accel_offsets[i] += (accel[i]-default_accel_offsets[i])
            gyro_offsets[i] += (gyro[i]-default_gyro_offsets[i])
        time.sleep(delay)

    # Average the collected data
    accel_offsets = [x / samples for x in accel_offsets]
    gyro_offsets = [x / samples for x in gyro_offsets]

    # For accelerometer, subtract gravity from Z axis
    accel_offsets[2] -= 9.80665

    print("Calibration complete.")
    print(f"Accelerometer offsets: {accel_offsets}")
    print(f"Gyroscope offsets: {gyro_offsets}")

    return accel_offsets, gyro_offsets

if __name__ == "__main__":
    calibrate_mpu6050()

import time
import board
import busio
import adafruit_mpu6050
from hardware.statsfunctions import KalmanFilter

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize MPU6050 sensor
mpu = adafruit_mpu6050.MPU6050(i2c)

# Create Kalman filters for each axis
kf_accel = [KalmanFilter() for _ in range(3)]
kf_gyro = [KalmanFilter() for _ in range(3)]

print("MPU6050 sensor initialized. Reading data with Kalman filter...")

while True:
    # Read accelerometer data (x, y, z)
    accel = mpu.acceleration
    accel_filtered = [
        kf_accel[0].update(accel[0]),
        kf_accel[1].update(accel[1]),
        kf_accel[2].update(accel[2])
    ]

    # Read gyroscope data (x, y, z)
    gyro = mpu.gyro
    gyro_filtered = [
        kf_gyro[0].update(gyro[0]),
        kf_gyro[1].update(gyro[1]),
        kf_gyro[2].update(gyro[2])
    ]

    # Read temperature
    temperature = mpu.temperature

    print(f"Accel (m/s^2): x={accel_filtered[0]:.2f}, y={accel_filtered[1]:.2f}, z={accel_filtered[2]:.2f}")
    print(f"Gyro (rad/s):  x={gyro_filtered[0]:.2f}, y={gyro_filtered[1]:.2f}, z={gyro_filtered[2]:.2f}")
    print(f"Temperature:   {temperature:.2f} C")
    print("-" * 40)
    time.sleep(1)
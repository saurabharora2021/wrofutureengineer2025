import board
import busio
import adafruit_mpu6050
import time

# Use I2C bus 1 (or 0 for older Pi versions)
i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

while True:
    print("Accelerometer (m/s^2): X=%0.3f Y=%0.3f Z=%0.3f"%mpu.acceleration)
    print("Gyroscope (degrees/sec): X=%0.3f Y=%0.3f Z=%0.3f"%mpu.gyro)
    print("Temperature: %0.1f C"%mpu.temperature)
    time.sleep(1)


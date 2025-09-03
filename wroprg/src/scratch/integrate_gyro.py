# save as src/hardware/integrate_gyro.py and run on Pi
import time
from board import SCL, SDA
import busio
from scratch.mpu6050 import MPU6050
import adafruit_tca9548a
DEVICE_I2C_CHANNEL = 6

# Setup I2C devices
i2c = busio.I2C(SCL, SDA)
tca = adafruit_tca9548a.TCA9548A(i2c)

device_channel = tca[DEVICE_I2C_CHANNEL]

mpu = MPU6050(device_channel)
cum = 0.0
t_prev = time.perf_counter()    
for _ in range(600):
    mpu.read()
    gx, gy, gz = mpu.gyro
    t = time.perf_counter()
    dt = t - t_prev
    t_prev = t
    cum += gz * dt
    print(f"dt={dt:.4f}s gz={gz:.2f}°/s cum={cum:.2f}°")
    time.sleep(0.01)
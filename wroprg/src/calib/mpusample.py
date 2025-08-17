import board
import busio
import adafruit_mpu6050
import time
import math

# --- Sensor Fusion Settings ---
tau = 0.98

# --- Sensor Setup ---
i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

# Initial angles from accelerometer for a starting point
accel_x, accel_y, accel_z = mpu.acceleration
pitch_angle = math.degrees(math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)))
roll_angle = math.degrees(math.atan2(accel_y, accel_z))
yaw_angle = 0.0  # Initialize yaw angle

last_time = time.time()

print("Complementary Filter with Yaw - Press Ctrl+C to exit")
try:
    while True:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # Get accelerometer and gyroscope data
        accel_x, accel_y, accel_z = mpu.acceleration
        gyro_x, gyro_y, gyro_z = mpu.gyro

        # --- Accelerometer Angle Calculation ---
        accel_pitch = math.degrees(math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)))
        accel_roll = math.degrees(math.atan2(accel_y, accel_z))

        # --- Fusion for Roll and Pitch ---
        pitch_angle = tau * (pitch_angle + gyro_y * dt) + (1 - tau) * accel_pitch
        roll_angle = tau * (roll_angle + gyro_x * dt) + (1 - tau) * accel_roll
        
        # --- Yaw Calculation (Gyroscope-only) ---
        # The yaw angle is calculated by integrating the gyro's Z-axis data.
        # This will drift over time.
        yaw_angle += gyro_z * dt
        yaw_angle %= 360 # Keep the angle within a 360-degree range

        print("Roll: {:.2f}° | Pitch: {:.2f}° | Yaw: {:.2f}°".format(roll_angle, pitch_angle, yaw_angle))
        
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nExiting program")
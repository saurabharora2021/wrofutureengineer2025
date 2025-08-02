""" Orientation estimation using MPU6050 data with Kalman filters."""
import math
import time

class SimpleKalmanFilter:
    """A simple 1D Kalman filter for demonstration."""
    def __init__(self, q=0.01, r=0.1, p=1.0, initial_value=0.0):
        self.q = q  # process noise
        self.r = r  # measurement noise
        self.p = p  # estimation error
        self.x = initial_value  # value

    def update(self, measurement):
        # Prediction update
        self.p += self.q
        # Measurement update
        k = self.p / (self.p + self.r)
        self.x += k * (measurement - self.x)
        self.p *= (1 - k)
        return self.x

class OrientationEstimator:
    """
    Estimates roll, pitch, and yaw using MPU6050 data and Kalman filters.
    """
    def __init__(self, get_accel, get_gyro, dt=0.01):
        self.get_accel = get_accel
        self.get_gyro = get_gyro
        self.dt = dt
        self.last_time = time.time()
        # State: roll, pitch, yaw
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        # Kalman filters for roll and pitch
        self.kalman_roll = SimpleKalmanFilter()
        self.kalman_pitch = SimpleKalmanFilter()

    def update(self):
        """Update the orientation estimator with latest sensor data."""
        # Get sensor data
        accel_x, accel_y, accel_z = self.get_accel()
        gyro_x, gyro_y, gyro_z = self.get_gyro()

        # Calculate dt
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # Calculate roll and pitch from accelerometer
        accel_roll = math.degrees(math.atan2(accel_y, accel_z))
        accel_pitch = math.degrees(math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)))

        # Integrate gyroscope data
        self.roll += gyro_x * dt
        self.pitch += gyro_y * dt
        self.yaw += gyro_z * dt  # Yaw will drift!

        # Kalman filter fusion
        self.roll = self.kalman_roll.update(accel_roll)
        self.pitch = self.kalman_pitch.update(accel_pitch)

    def get_orientation(self):
        """Returns (roll, pitch, yaw) in degrees."""
        return self.roll, self.pitch, self.yaw

""" Orientation estimation using MPU6050 data with Kalman filters."""
import math
import time
import logging
from base.shutdown_handling import ShutdownInterface
from utils.threadingfunctions import ConstantUpdateThread

logger = logging.getLogger(__name__)

class SimpleKalmanFilter:
    """A simple 1D Kalman filter for demonstration."""
    def __init__(self, q=0.01, r=0.1, p=1.0, initial_value=0.0):
        self.q = q  # process noise
        self.r = r  # measurement noise
        self.p = p  # estimation error
        self.x = initial_value  # value

    def update(self, measurement):
        """Update the Kalman filter with a new measurement."""
        # Prediction update
        self.p += self.q
        # Measurement update
        k = self.p / (self.p + self.r)
        self.x += k * (measurement - self.x)
        self.p *= (1 - k)
        return self.x

class OrientationEstimator(ShutdownInterface):
    """
    Estimates roll, pitch, and yaw using MPU6050 data and a complementary blend
    with simple Kalman smoothing on accelerometer-derived angles.
    """
    def __init__(self, get_accel, get_gyro, dt=0.01):
        self.get_accel = get_accel      # returns (ax, ay, az) in m/s^2
        self.get_gyro = get_gyro        # returns (gx, gy, gz) in rad/s (Adafruit lib)
        self.dt = dt
        self.last_time = time.perf_counter()

        # State kept in DEGREES
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Gyro bias (rad/s) for yaw axis
        self.yaw_bias = 0.0
        self.stationary_threshold = 0.05  # rad/s

        # Low-pass on accel angles before blending
        self.kalman_roll = SimpleKalmanFilter(q=1e-3, r=5e-2, p=1.0, initial_value=0.0)
        self.kalman_pitch = SimpleKalmanFilter(q=1e-3, r=5e-2, p=1.0, initial_value=0.0)

        # Complementary filter blend factor
        self._alpha = 0.98

        # Calibration offsets
        self.roll_offset = 0.0
        self.pitch_offset = 0.0
        self.yaw_offset = 0.0


        #lets start the thread to read sensor data
        self._thread: ConstantUpdateThread = ConstantUpdateThread(self.update, interval_ms=9)

        self._stationary_gz_samples = []
        self._stationary_sample_limit = 50  # Number of samples to average for bias

    def shutdown(self):
        """Shutdown readings"""
        self._thread.stop()

    def start_readings(self):
        """Start Reading"""
        #Lets calibrate the devices.
        self.calibrate_imu()

        self._thread.start()


    @staticmethod
    def _accel_to_angles(ax: float, ay: float, az: float) -> tuple[float, float]:
        """
        Compute roll and pitch from accelerometer (degrees).
        roll about X, pitch about Y, right-hand rule.
        """
        eps = 1e-9
        az = az if abs(az) > eps else math.copysign(eps, az)
        roll_rad = math.atan2(ay, az)
        pitch_rad = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        return math.degrees(roll_rad), math.degrees(pitch_rad)

    def update(self):
        """Fuse accel and gyro to update roll, pitch, yaw (degrees)."""
        # Time step with clamping
        now = time.perf_counter()
        dt = now - self.last_time
        if dt <= 0 or dt > 0.1:
            dt = self.dt  # clamp to nominal to avoid spikes
        self.last_time = now

        # logger.info("Update orientation... %0.2f", dt)

        # Read sensors
        ax, ay, az = self.get_accel()
        gx, gy, gz = self.get_gyro()

        # Subtract offsets for roll and pitch. Yaw bias is handled separately.
        gx -= math.radians(self.roll_offset)
        gy -= math.radians(self.pitch_offset)
        # gz -= math.radians(self.yaw_offset) # <-- REMOVE THIS LINE

        # Prediction: integrate gyro (convert rad/s to deg/s)
        roll_pred = self.roll + math.degrees(gx) * dt
        pitch_pred = self.pitch + math.degrees(gy) * dt

        # Yaw: only gyro with bias correction (no accel correction available)
        # Adapt bias slowly when stationary
        stationary = abs(gx) < self.stationary_threshold and \
                     abs(gy) < self.stationary_threshold and \
                     abs(gz) < self.stationary_threshold

        if stationary:
            self._stationary_gz_samples.append(gz)
            if len(self._stationary_gz_samples) >= self._stationary_sample_limit:
                # Calculate the new stationary average
                stationary_avg_bias = sum(self._stationary_gz_samples) / \
                                      len(self._stationary_gz_samples)

                # Instead of replacing, gently blend the new average with the existing bias.
                # This respects the initial calibration but allows for slow adaptation.
                # (95% old value, 5% new value)
                self.yaw_bias = self.yaw_bias * 0.95 + stationary_avg_bias * 0.05

                logger.info("Refined Stationary yaw bias to: %.4f rad/s", self.yaw_bias)
                self._stationary_gz_samples.clear()
        else:
            self._stationary_gz_samples.clear()

        # Apply the single, unified bias correction
        corrected_gz = gz - self.yaw_bias
        self.yaw += math.degrees(corrected_gz) * dt

        # Measurement: roll/pitch from accelerometer (smoothed)
        accel_roll, accel_pitch = self._accel_to_angles(ax, ay, az)
        accel_roll = self.kalman_roll.update(accel_roll)
        accel_pitch = self.kalman_pitch.update(accel_pitch)

        # Complementary fusion
        a = self._alpha
        self.roll = a * roll_pred + (1.0 - a) * accel_roll
        self.pitch = a * pitch_pred + (1.0 - a) * accel_pitch

    def reset(self, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
                ) -> None:
        """
        Reset internal state and start estimation again.
        - roll/pitch/yaw are in degrees
        - Reinitializes Kalman filters and gyro yaw-bias
        """
        # Reset orientation (degrees)
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

        # Reset yaw bias to the last known good calibrated value.
        # This is more reliable than sampling a single point.
        self.yaw_bias = math.radians(self.yaw_offset)

        # Reinitialize accel-angle Kalman filters with same tuning
        self.kalman_roll = SimpleKalmanFilter(q=self.kalman_roll.q, r=self.kalman_roll.r, p=1.0,
                                              initial_value=roll)
        self.kalman_pitch = SimpleKalmanFilter(q=self.kalman_pitch.q, r=self.kalman_pitch.r, p=1.0,
                                               initial_value=pitch)

        # Reset time to avoid a large dt on next update
        self.last_time = time.perf_counter()

    def get_orientation(self):
        """Returns (roll, pitch, yaw) in degrees."""
        return self.roll, self.pitch, self.yaw

    def calibrate_imu(self, samples=200) -> tuple[float, float, float]:
        """
        Calibrate IMU by averaging gyro readings over a number of samples.
        Returns offsets for roll, pitch, yaw (in degrees).
        """
        print("Calibrating IMU... Place the robot on a perfectly level surface.")
        time.sleep(0.5)
        total_gx = 0.0
        total_gy = 0.0
        total_gz = 0.0
        for _ in range(samples):
            gx, gy, gz = self.get_gyro()
            total_gx += gx
            total_gy += gy
            total_gz += gz
            time.sleep(0.01)
        # Convert rad/s to deg/s for offsets
        offset_roll = math.degrees(total_gx / samples)
        offset_pitch = math.degrees(total_gy / samples)
        offset_yaw = math.degrees(total_gz / samples)
        logger.info("Calibration complete. Gyro Offsets: Roll: %.2f," \
                                    " Pitch: %.2f, Yaw: %.2f degrees",
                                    offset_roll, offset_pitch, offset_yaw)
        self.roll_offset = offset_roll
        self.pitch_offset = offset_pitch
        self.yaw_offset = offset_yaw

        # Set the primary yaw_bias (in rad/s) from the good calibration
        self.yaw_bias = math.radians(offset_yaw)

        return offset_roll, offset_pitch, offset_yaw

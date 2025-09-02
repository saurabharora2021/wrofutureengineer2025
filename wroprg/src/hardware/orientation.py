""" Orientation estimation using MPU6050 data with Kalman filters."""
import math
import time
import logging
from typing import Tuple
from board import SCL, SDA
import busio
from qmc5883l import QMC5883L
import adafruit_tca9548a
from hardware.mpu6050 import MPU6050
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

    # Based on 90 degreee turn right and left we can only 88 degree turn.
    # No artificial yaw scaling — report true sensor angle in degrees.
    DEGREE_90_CORRECTION = 1.0

    USE_COMPASS = True
    # When True, completely disable MPU6050 usage and drive yaw only from QMC5883L.
    USE_ONLY_COMPASS = False
    # Set the yaw sign so that a right turn yields positive yaw.
    # If your MPU6050 gz increases for left turns (common), use -1.0.
    # If gz increases for right turns, use +1.0.
    gyro_yaw_sign: float = -1.0
    def get_acceleration(self) -> Tuple[float, float, float]:
        """Get the acceleration from the MPU6050 sensor."""
        accel = self.mpu.acceleration
        return accel

    def get_gyroscope(self) -> Tuple[float, float, float]:
        """Get the gyroscope data from the MPU6050 sensor."""
        if self.mpu is None:
            raise ValueError("Gyroscope not available")
        # If we're using the custom MPU6050 wrapper it already returns deg/s.
        if getattr(self, "_gyro_in_deg", False):
            return self.mpu.gyro
        # Adafruit library returns rad/s — convert to deg/s for internal use
        gx, gy, gz = self.mpu.gyro
        return (math.degrees(gx), math.degrees(gy), math.degrees(gz))

    def get_magnetometer(self) -> Tuple[float, float, float]:
        """Get the magnetometer data from the QMC5883L sensor."""
        if self.compass is not None:
            return self.compass.magnetic
        raise ValueError("Magnetometer not available")

    def __init__(self, device_channel, dt=0.01):

        # Sensors on I2C
        if not self.USE_ONLY_COMPASS:
            self.mpu = MPU6050(device_channel)  # Using custom MPU6050 class
            # Our custom MPU6050 returns gyro in deg/s
            self._gyro_in_deg = True
            self.get_accel = self.get_acceleration  # returns (ax, ay, az) in m/s^2
            self.get_gyro = self.get_gyroscope # returns (gx, gy, gz) in deg/s for custom MPU
        else:
            # Disable MPU entirely
            self.mpu = None
            self.get_accel = None
            self.get_gyro = None

        # Enable compass when requested or when running compass-only mode
        if self.USE_ONLY_COMPASS or self.USE_COMPASS:
            self.compass = QMC5883L(device_channel)
            # Magnetometer callback: returns (mx, my, mz) in microteslas (uT)
            self.get_mag = self.get_magnetometer
        else:
            self.compass = None
            self.get_mag = None

        self.dt = dt
        self.last_time = time.perf_counter()

        # Measured update rate (exponential moving average on dt)
        # _ema_dt stores a smoothed delta-time between update() calls.
        self._ema_dt = self.dt
        # EMA alpha for smoothing the measured dt. Small alpha -> heavier smoothing.
        self._ema_alpha = 0.05
        # Current measured Hz (1 / ema_dt)
        self._measured_hz = 1.0 / max(self._ema_dt, 1e-9)
        # Profiling EMAs (ms)
        self._ema_update_ms = self._ema_dt * 1000.0
        self._ema_accel_gyro_ms = 0.0
        self._ema_mag_ms = 0.0
        # How often to log profiling summary (in updates)
        self._profile_log_period = 200
        self._profile_counter = 0

        # Magnetometer throttling: read mag every N updates
        self._mag_period = 3  # Read mag every 2 updates (~50 Hz if update rate ~100 Hz)
        self._mag_counter = 0
        self._last_mag = (0.0, 0.0, 0.0)

        # State kept in DEGREES
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        # Yaw zero-reference (degrees). Reported yaw = yaw - this offset.
        self._yaw_zero_offset_deg = 0.0
        # Defer zeroing until a valid fused yaw exists
        self._yaw_zero_pending = False

        # Gyro bias (deg/s) for yaw axis (custom MPU returns deg/s)
        self.yaw_bias = 0.0
        # Stationary threshold in deg/s (approx 0.05 rad/s -> ~2.86 deg/s)
        self.stationary_threshold = 3.0  # deg/s

    # Low-pass on accel angles before blending
        self.kalman_roll = SimpleKalmanFilter(q=1e-3, r=5e-2, p=1.0, initial_value=0.0)
        self.kalman_pitch = SimpleKalmanFilter(q=1e-3, r=5e-2, p=1.0, initial_value=0.0)

        # Complementary filter blend factor
        self._alpha = 0.98
        #TODO: tune for indoor, expected to be lower. ooutside 0.97, inside 0.99
        self._alpha_yaw = 0.90  # Increased to trust the gyro more and reduce noise

        # Magnetometer fusion tuning
        # Low-pass on mag heading (0..1). Higher to reduce lag on fast turns.
        self._mag_yaw_lpf_alpha = 0.5  # Decreased for more smoothing of mag data
        self._mag_yaw_lpf = None       # internal state
        # If your yaw correction runs away, flip this to -1.0
        self.mag_heading_sign = -1.0
        # Magnetometer sensitivity adjustment (increase if turns are underestimated)
        self._mag_sensitivity_factor = 1.0  # 1.0 is sufficient with faster mag LPF

        # Calibration offsets
        self.roll_offset = 0.0
        self.pitch_offset = 0.0
        self.yaw_offset = 0.0

        # Magnetometer calibration (hard-iron bias) and declination
        self.mag_bias_x = 0.0
        self.mag_bias_y = 0.0
        self.mag_bias_z = 0.0
        # Optional soft-iron scaling could be added later
        self.mag_declination_deg = 0.0  # set your local declination if needed


        #lets start the thread to read sensor data
        self._thread: ConstantUpdateThread = ConstantUpdateThread(self.update, interval_ms=4.9)

        self._stationary_gz_samples = []
        self._stationary_sample_limit = 50  # Number of samples to average for bias

        self.counter = 0

    def shutdown(self):
        """Shutdown readings"""
        self._thread.stop()

    def start_readings(self):
        """Start Reading"""
        # Always calibrate IMU on startup when MPU is enabled
        if not self.USE_ONLY_COMPASS:
            logger.info("Calibrating IMU (startup)...")
            self.calibrate_imu()

        # Defer yaw zeroing to the first valid fused update
        self.reset_yaw(defer_to_next_update=True)
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
        # Update exponential moving average of dt to measure actual update Hz
        # Use EMA to avoid single-frame jitter affecting measured rate too much.
        self._ema_dt = (1.0 - self._ema_alpha) * self._ema_dt + self._ema_alpha * dt
        # Protect against zero
        self._measured_hz = 1.0 / max(self._ema_dt, 1e-9)
        # also track update time in ms (EMA)
        self._ema_update_ms = self._ema_dt * 1000.0

        self.counter +=1

        # Profiling counters
        self._profile_counter += 1
        #every N updates, print the actual hz and per-sensor times.
        if self._profile_counter >= self._profile_log_period:
            logger.info("Orientation: update rate: %.2f Hz, update_ms: %.2f, accel_gyro_ms: %.2f, mag_ms: %.2f",
                        self._measured_hz, self._ema_update_ms, self._ema_accel_gyro_ms, self._ema_mag_ms)
            self._profile_counter = 0

        # Compass-only mode: compute yaw directly from magnetometer (no tilt compensation)
        if self.USE_ONLY_COMPASS:
            if callable(self.get_mag):
                try:
                    mx, my, mz = self.get_mag()
                    mx, my, mz = self._remap_qmc_to_imu(mx, my, mz)
                    # Apply hard-iron bias compensation
                    mx -= self.mag_bias_x
                    my -= self.mag_bias_y
                    mz -= self.mag_bias_z
                    # 2D heading (assumes near-level). Use mag_heading_sign to match yaw sense.
                    #yaw_mag = math.degrees(math.atan2(self.mag_heading_sign * my, mx))
                    # FIX #1: Correct atan2 for X-backward, Y-right coordinate system.
                    yaw_mag = math.degrees(math.atan2(self.mag_heading_sign * my, -mx))
                    yaw_mag = self._wrap_angle_deg(yaw_mag + self.mag_declination_deg)
                    # Smooth heading
                    if self._mag_yaw_lpf is None:
                        self._mag_yaw_lpf = yaw_mag
                    else:
                        err = self._angle_diff_deg(yaw_mag, self._mag_yaw_lpf)
                        self._mag_yaw_lpf = self._wrap_angle_deg(
                            self._mag_yaw_lpf + self._mag_yaw_lpf_alpha * err
                        )
                    self.yaw = self._mag_yaw_lpf
                except (OSError, RuntimeError, ValueError) as e:
                    # Keep previous yaw if read fails
                    logger.debug("Compass-only read failed: %s", e)
            # Handle deferred zeroing once we have a yaw
            if self._yaw_zero_pending:
                self._yaw_zero_offset_deg = self.yaw
                self._yaw_zero_pending = False
                logger.info("Yaw zeroed at first update (compass-only): %.2f deg", \
                                                            self._yaw_zero_offset_deg)
            return

        # logger.info("Update orientation... %0.2f", dt)

        # Read sensors (measure time taken)
        t0 = time.perf_counter()
        self.mpu.read()  # Fast burst read to update internal state 
        ax, ay, az = self.get_accel()
        gx, gy, gz = self.get_gyro()
        t1 = time.perf_counter()
        accel_gyro_ms = (t1 - t0) * 1000.0
        # Update EMA for accel/gyro read time
        alpha = 0.1
        self._ema_accel_gyro_ms = (1.0 - alpha) * self._ema_accel_gyro_ms + alpha * accel_gyro_ms

        # Subtract gyro offsets (offsets are in deg/s). Yaw bias is handled separately.
        gx -= self.roll_offset
        gy -= self.pitch_offset
        # --- Roll & Pitch prediction (deg) ---
        # gx/gy are in deg/s so integration is direct: deg = deg/s * s
        roll_pred = self.roll + gx * dt
        pitch_pred = self.pitch + gy * dt
        # --- Stationary detection for yaw bias refinement ---
        stationary = abs(gx) < self.stationary_threshold and \
                     abs(gy) < self.stationary_threshold and \
                     abs(gz) < self.stationary_threshold

        if stationary:
            self._stationary_gz_samples.append(gz)
            if len(self._stationary_gz_samples) >= self._stationary_sample_limit:
                stationary_avg_bias = sum(self._stationary_gz_samples) \
                                                 / len(self._stationary_gz_samples)
                # stationary_avg_bias is in the same units as get_gyroscope() (deg/s if using custom MPU)
                # keep yaw_bias in deg/s consistently
                self.yaw_bias = self.yaw_bias * 0.95 + stationary_avg_bias * 0.05
                logger.info("Refined Stationary yaw bias to: %.4f deg/s", self.yaw_bias)
                self._stationary_gz_samples.clear()
        else:
            self._stationary_gz_samples.clear()

        # Apply the single, unified bias correction (gz in deg/s)
        corrected_gz = gz - self.yaw_bias
        # Integrate yaw with configured sign so that right turns are positive.
        # corrected_gz * dt yields degrees
        yaw_pred = self.yaw + self.gyro_yaw_sign * corrected_gz * dt

        # --- Accel measurement for roll/pitch ---
        accel_roll, accel_pitch = self._accel_to_angles(ax, ay, az)
        accel_roll = self.kalman_roll.update(accel_roll)
        accel_pitch = self.kalman_pitch.update(accel_pitch)

        # Complementary fusion for roll/pitch
        a = self._alpha
        self.roll = a * roll_pred + (1.0 - a) * accel_roll
        self.pitch = a * pitch_pred + (1.0 - a) * accel_pitch

        # Magnetometer fusion for yaw (tilt-compensated heading)
        self._mag_counter += 1
        if callable(self.get_mag):
            try:
                if self._mag_counter % self._mag_period == 0:
                    t_m0 = time.perf_counter()
                    mx, my, mz = self.get_mag()
                    t_m1 = time.perf_counter()
                    mag_ms = (t_m1 - t_m0) * 1000.0
                    self._ema_mag_ms = (1.0 - alpha) * self._ema_mag_ms + alpha * mag_ms
                    self._last_mag = (mx, my, mz)
                else:
                    mx, my, mz = self._last_mag
                # Axes are wired to match: IMU X=QMC X, IMU Y=QMC Y (Z assumed aligned)
                mx, my, mz = self._remap_qmc_to_imu(mx, my, mz)
                # Apply hard-iron bias compensation
                mx -= self.mag_bias_x
                my -= self.mag_bias_y
                mz -= self.mag_bias_z
                # Compute tilt-compensated heading (deg), add declination
                yaw_mag = self._tilt_compensated_heading(ax, ay, az, mx, my, mz)
                yaw_mag = self._wrap_angle_deg(yaw_mag + self.mag_declination_deg)
                # Smooth magnetometer heading (circular low-pass)
                if self._mag_yaw_lpf is None:
                    self._mag_yaw_lpf = yaw_mag
                else:
                    err = self._angle_diff_deg(yaw_mag, self._mag_yaw_lpf)
                    self._mag_yaw_lpf = self._wrap_angle_deg(self._mag_yaw_lpf + self._mag_yaw_lpf_alpha * err)
                # Trust mag a bit more when stationary
                alpha_yaw = 0.80 if not stationary else 0.60
                self.yaw = self._fuse_angles_deg(yaw_pred, self._mag_yaw_lpf, alpha_yaw, sensitivity=self._mag_sensitivity_factor)
            except (OSError, RuntimeError, ValueError) as e:
                logger.debug("Compass read failed: %s; using gyro-only yaw", e)
                self.yaw = self._wrap_angle_deg(yaw_pred)
        else:
            self.yaw = self._wrap_angle_deg(yaw_pred)

        # If zeroing was requested before a valid yaw existed,
        # set the zero offset now using the current fused yaw.
        if self._yaw_zero_pending:
            self._yaw_zero_offset_deg = self.yaw
            self._yaw_zero_pending = False
            logger.info("Yaw zeroed at first update: %.2f deg", self._yaw_zero_offset_deg)

    @staticmethod
    def _wrap_angle_deg(angle: float) -> float:
        """Wrap angle to [-180, 180)."""
        a = (angle + 180.0) % 360.0 - 180.0
        # Map -180 to 180 if needed to keep continuity (optional)
        return a

    @staticmethod
    def _angle_diff_deg(a: float, b: float) -> float:
        """Compute smallest signed difference a-b in degrees within [-180,180)."""
        d = (a - b + 180.0) % 360.0 - 180.0
        return d

    def _fuse_angles_deg(self, pred: float, meas: float, alpha: float,\
                                                sensitivity: float = 1.0) -> float:
        """Complementary fuse two angles in degrees considering wrap-around."""
        err = self._angle_diff_deg(meas, pred)
        # Apply sensitivity scaling to make magnetometer corrections stronger
        fused = pred + (1.0 - alpha) * err * sensitivity
        return self._wrap_angle_deg(fused)

    def _tilt_compensated_heading(self, ax: float, ay: float, az: float,
                                  mx: float, my: float, mz: float) -> float:
        """Compute tilt-compensated magnetic heading in degrees [-180,180)."""
        # Compute roll/pitch in radians from accel
        roll_deg, pitch_deg = self._accel_to_angles(ax, ay, az)
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)

        # Tilt compensation (AN4248 style)
        mx2 = mx * math.cos(pitch) + mz * math.sin(pitch)
        my2 = mx * math.sin(roll) * math.sin(pitch) + my * math.cos(roll) - \
              mz * math.sin(roll) * math.cos(pitch)
        # Flip sign via mag_heading_sign if rotation sense disagrees with gyro
        #heading = math.degrees(math.atan2(self.mag_heading_sign * my2, mx2))
        # FIX #2: Correct atan2 for X-backward, Y-right coordinate system.
        heading = math.degrees(math.atan2(self.mag_heading_sign * my2, -mx2))
        return self._wrap_angle_deg(heading)

    @staticmethod
    def _remap_qmc_to_imu(mx: float, my: float, mz: float) -> tuple[float, float, float]:
        """Axes are already aligned between QMC5883L and MPU6050 on this robot."""
        return (mx, my, mz)

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
        # yaw_offset is stored in deg/s (gyro units); keep yaw_bias in deg/s
        self.yaw_bias = self.yaw_offset

        # Reinitialize accel-angle Kalman filters with same tuning
        self.kalman_roll = SimpleKalmanFilter(q=self.kalman_roll.q, r=self.kalman_roll.r, p=1.0,
                                              initial_value=roll)
        self.kalman_pitch = SimpleKalmanFilter(q=self.kalman_pitch.q, r=self.kalman_pitch.r, p=1.0,
                                               initial_value=pitch)

        # Reset time to avoid a large dt on next update
        self.last_time = time.perf_counter()

    def reset_yaw(self, to_current: bool = True, offset_deg: float | None = None,
                  defer_to_next_update: bool = False) -> None:
        """
         Set yaw zero-reference for relative yaw reporting.
         - to_current=True: zero to current absolute yaw so subsequent yaw reads start at 0.
         - offset_deg: explicitly set the zero-reference angle in degrees (absolute frame).
         - defer_to_next_update=True: zero at the next fused update (recommended at startup).
         If both provided, to_current takes precedence unless defer_to_next_update is True.
        """
        if defer_to_next_update:
            self._yaw_zero_pending = True
            return
        if to_current:
            self._yaw_zero_offset_deg = self.yaw
        elif offset_deg is not None:
            self._yaw_zero_offset_deg = self._wrap_angle_deg(offset_deg)
        else:
            self._yaw_zero_offset_deg = 0.0

        # logger.info("reset yaw: %.2f", self._yaw_zero_offset_deg)

    def get_yaw(self) -> float:
        """Return yaw in degrees relative to the zero-reference set by reset_yaw()."""
        return self._wrap_angle_deg(self.yaw - self._yaw_zero_offset_deg)*self.DEGREE_90_CORRECTION

    def get_orientation(self):
        """Returns (roll, pitch, yaw) in degrees. Yaw is relative to reset_yaw()."""
        return self.roll, self.pitch, self._wrap_angle_deg(self.yaw - self._yaw_zero_offset_deg)\
                                                        * self.DEGREE_90_CORRECTION

    def calibrate_imu(self, samples=200) -> tuple[float, float, float]:
        """
        Calibrate IMU by averaging gyro readings over a number of samples.
        Returns offsets for roll, pitch, yaw (in degrees).
        """
        print("Calibrating IMU... Place the robot on a perfectly level surface.")
        total_gx = 0.0
        total_gy = 0.0
        total_gz = 0.0
        for _ in range(samples):
            self.mpu.read()  # Fast burst read to update internal state
            gx, gy, gz = self.get_gyro()
            total_gx += gx
            total_gy += gy
            total_gz += gz
            time.sleep(0.01)
        # Compute average gyro offsets in deg/s. If get_gyro() returns rad/s
        # (Adafruit MPU), convert to deg/s using math.degrees(). If it
        # already returns deg/s (custom MPU), keep as-is.
        avg_gx = total_gx / samples
        avg_gy = total_gy / samples
        avg_gz = total_gz / samples
        if getattr(self, "_gyro_in_deg", False):
            offset_roll = avg_gx
            offset_pitch = avg_gy
            offset_yaw = avg_gz
        else:
            offset_roll = math.degrees(avg_gx)
            offset_pitch = math.degrees(avg_gy)
            offset_yaw = math.degrees(avg_gz)
        logger.info("Calibration complete. Gyro Offsets: Roll: %.2f," \
                                    " Pitch: %.2f, Yaw: %.2f degrees",
                                    offset_roll, offset_pitch, offset_yaw)
        self.roll_offset = offset_roll
        self.pitch_offset = offset_pitch
        self.yaw_offset = offset_yaw

        # Set the primary yaw_bias (in deg/s) from the good calibration
        self.yaw_bias = offset_yaw

        return offset_roll, offset_pitch, offset_yaw



def main():
    """Main method for orientation."""
    logging.basicConfig(level=logging.INFO)
    # Use module-level logger

    DEVICE_I2C_CHANNEL = 6

    # Setup I2C devices
    i2c = busio.I2C(SCL, SDA)
    tca = adafruit_tca9548a.TCA9548A(i2c)

    device_channel = tca[DEVICE_I2C_CHANNEL]

    # Create an OrientationEstimator instance
    orientation_estimator = OrientationEstimator(device_channel)

    orientation_estimator.start_readings()

    while True:
        (roll, pitch, yaw) = orientation_estimator.get_orientation()
        logger.info("Orientation: Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw)
        time.sleep(0.5) # main method sleep

if __name__ == "__main__":
    main()

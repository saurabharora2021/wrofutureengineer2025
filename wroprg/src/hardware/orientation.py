""" Orientation estimation using MPU6050 data with Kalman filters."""
import math
import time
import logging
import json
import os
from pathlib import Path
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
    def __init__(self, get_accel, get_gyro, get_mag=None, dt=0.01):
        self.get_accel = get_accel      # returns (ax, ay, az) in m/s^2
        self.get_gyro = get_gyro        # returns (gx, gy, gz) in rad/s (Adafruit lib)
        # self.get_mag = get_mag          # returns (mx, my, mz) in uT; optional
        # Magnetometer callback: returns (mx, my, mz) in microteslas (uT)
        self.get_mag = get_mag
        self.dt = dt
        self.last_time = time.perf_counter()

        # State kept in DEGREES
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        # Yaw zero-reference (degrees). Reported yaw = yaw - this offset.
        self._yaw_zero_offset_deg = 0.0
        # Defer zeroing until a valid fused yaw exists
        self._yaw_zero_pending = False

        # Gyro bias (rad/s) for yaw axis
        self.yaw_bias = 0.0
        self.stationary_threshold = 0.05  # rad/s

        # Low-pass on accel angles before blending
        self.kalman_roll = SimpleKalmanFilter(q=1e-3, r=5e-2, p=1.0, initial_value=0.0)
        self.kalman_pitch = SimpleKalmanFilter(q=1e-3, r=5e-2, p=1.0, initial_value=0.0)

        # Complementary filter blend factor
        self._alpha = 0.98
        #TODO: tune for indoor, expected to be lower. ooutside 0.97, inside 0.99
        self._alpha_yaw = 0.70  # Lower value to trust mag heading more (was 0.90)

        # Magnetometer fusion tuning
        self._mag_yaw_lpf_alpha = 0.2  # low-pass on mag heading (0..1)
        self._mag_yaw_lpf = None       # internal state
        # If your yaw correction runs away, flip this to -1.0
        self.mag_heading_sign = 1.0
        # Magnetometer sensitivity adjustment (increase if turns are underestimated)
        self._mag_sensitivity_factor = 4  # Scale heading error to get full motion range

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
        self._thread: ConstantUpdateThread = ConstantUpdateThread(self.update, interval_ms=8)

        self._stationary_gz_samples = []
        self._stationary_sample_limit = 50  # Number of samples to average for bias

        # Persistent calibration settings
        self._calibration_path: Path = Path.home() / ".wro_orientation_cal.json"
        self._recal_interval_sec: float = 2 * 60 * 60  # 2 hours
        self._cal_loaded: bool = False

    def shutdown(self):
        """Shutdown readings"""
        self._thread.stop()

    def start_readings(self):
        """Start Reading"""
        # Try to load calibration; recalibrate if missing or older than 2 hours
        need_recal = True
        try:
            if self._calibration_path.exists():
                # Prefer timestamp inside file; fall back to mtime
                with open(self._calibration_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                ts = float(data.get("timestamp", 0.0))
                if ts <= 0.0:
                    ts = self._calibration_path.stat().st_mtime
                age = time.time() - ts
                if age < self._recal_interval_sec:
                    # Apply loaded calibration
                    self.roll_offset = float(data.get("roll_offset_deg_per_s", 0.0))
                    self.pitch_offset = float(data.get("pitch_offset_deg_per_s", 0.0))
                    self.yaw_offset = float(data.get("yaw_offset_deg_per_s", 0.0))
                    # Set primary yaw bias (rad/s) from yaw_offset
                    self.yaw_bias = math.radians(self.yaw_offset)
                    # Optional magnetometer calibration if present
                    self.mag_bias_x = float(data.get("mag_bias_x", 0.0))
                    self.mag_bias_y = float(data.get("mag_bias_y", 0.0))
                    self.mag_bias_z = float(data.get("mag_bias_z", 0.0))
                    self.mag_declination_deg = float(data.get("mag_declination_deg", 0.0))
                    need_recal = False
                    self._cal_loaded = True
                    logger.info("Loaded IMU calibration from %s (age: %.0fs)",
                                self._calibration_path, age)
        except Exception as e:
            logger.warning("Failed to load calibration: %s", e)

        if need_recal:
            logger.info("Calibrating IMU (no/old calibration)...")
            self.calibrate_imu()
            try:
                self._save_calibration()
            except Exception as e:
                logger.warning("Failed to save calibration: %s", e)

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

        # logger.info("Update orientation... %0.2f", dt)

        # Read sensors
        ax, ay, az = self.get_accel()
        gx, gy, gz = self.get_gyro()

        # Subtract offsets for roll and pitch. Yaw bias is handled separately.
        gx -= math.radians(self.roll_offset)
        gy -= math.radians(self.pitch_offset)
        # --- Roll & Pitch prediction (deg) ---
        roll_pred = self.roll + math.degrees(gx) * dt
        pitch_pred = self.pitch + math.degrees(gy) * dt
        # --- Stationary detection for yaw bias refinement ---
        stationary = abs(gx) < self.stationary_threshold and \
                     abs(gy) < self.stationary_threshold and \
                     abs(gz) < self.stationary_threshold

        if stationary:
            self._stationary_gz_samples.append(gz)
            if len(self._stationary_gz_samples) >= self._stationary_sample_limit:
                stationary_avg_bias = sum(self._stationary_gz_samples) / len(self._stationary_gz_samples)
                self.yaw_bias = self.yaw_bias * 0.95 + stationary_avg_bias * 0.05
                logger.info("Refined Stationary yaw bias to: %.4f rad/s", self.yaw_bias)
                self._stationary_gz_samples.clear()
        else:
            self._stationary_gz_samples.clear()

        # Apply the single, unified bias correction
        corrected_gz = gz - self.yaw_bias
        yaw_pred = self.yaw + math.degrees(corrected_gz) * dt

        # --- Accel measurement for roll/pitch ---
        accel_roll, accel_pitch = self._accel_to_angles(ax, ay, az)
        accel_roll = self.kalman_roll.update(accel_roll)
        accel_pitch = self.kalman_pitch.update(accel_pitch)

        # Complementary fusion for roll/pitch
        a = self._alpha
        self.roll = a * roll_pred + (1.0 - a) * accel_roll
        self.pitch = a * pitch_pred + (1.0 - a) * accel_pitch

        # Magnetometer fusion for yaw (tilt-compensated heading)
        if callable(self.get_mag):
            try:
                mx, my, mz = self.get_mag()
                # Axes are wired to match: IMU X=QMC X, IMU Y=QMC Y (Z assumed aligned)
                mx, my, mz = self._remap_qmc_to_imu(mx, my, mz)
                # Apply hard-iron bias compensation
                mx -= self.mag_bias_x
                my -= self.mag_bias_y
                mz -= self.mag_bias_z
                # Compute tilt-compensated heading (deg), add declination
                yaw_mag = self._tilt_compensated_heading(ax, ay, az, mx, my, mz)
                # Add magnetic declination
                yaw_mag = self._wrap_angle_deg(yaw_mag + self.mag_declination_deg)
                # Smooth magnetometer heading (circular low-pass)
                if self._mag_yaw_lpf is None:
                    self._mag_yaw_lpf = yaw_mag
                else:
                    err = self._angle_diff_deg(yaw_mag, self._mag_yaw_lpf)
                    self._mag_yaw_lpf = self._wrap_angle_deg(self._mag_yaw_lpf + self._mag_yaw_lpf_alpha * err)
                # Trust mag a bit more when stationary
                alpha_yaw = 0.80 if not stationary else 0.60  # Trust mag more (was 0.85/0.70)
                self.yaw = self._fuse_angles_deg(yaw_pred, self._mag_yaw_lpf, alpha_yaw, 
                                                 sensitivity=self._mag_sensitivity_factor)
            except Exception:  # pylint: disable=broad-except
                # Fallback to gyro-only if magnetometer read fails
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

    def _fuse_angles_deg(self, pred: float, meas: float, alpha: float, sensitivity: float = 1.0) -> float:
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
        heading = math.degrees(math.atan2(self.mag_heading_sign * my2, mx2))
        return self._wrap_angle_deg(heading)

    @staticmethod
    def _remap_qmc_to_imu(mx: float, my: float, mz: float) -> tuple[float, float, float]:
        """Flip axes to match gyro convention"""
        return (-mx, -my, mz)  # Flip X and Y

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

        logger.info("reset yaw: %.2f", self._yaw_zero_offset_deg)

    def get_yaw(self) -> float:
        """Return yaw in degrees relative to the zero-reference set by reset_yaw()."""
        return self._wrap_angle_deg(self.yaw - self._yaw_zero_offset_deg)

    def get_orientation(self):
        """Returns (roll, pitch, yaw) in degrees. Yaw is relative to reset_yaw()."""
        return self.roll, self.pitch, self._wrap_angle_deg(self.yaw - self._yaw_zero_offset_deg)

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

    # --- Persistence helpers ---
    def _save_calibration(self) -> None:
        """Persist gyro (and optional mag) calibration to JSON file."""
        data = {
            "roll_offset_deg_per_s": float(self.roll_offset),
            "pitch_offset_deg_per_s": float(self.pitch_offset),
            "yaw_offset_deg_per_s": float(self.yaw_offset),
            "mag_bias_x": float(self.mag_bias_x),
            "mag_bias_y": float(self.mag_bias_y),
            "mag_bias_z": float(self.mag_bias_z),
            "mag_declination_deg": float(self.mag_declination_deg),
            "timestamp": float(time.time()),
            "recal_interval_sec": float(self._recal_interval_sec),
        }
        try:
            self._calibration_path.parent.mkdir(parents=True, exist_ok=True)
        except Exception:
            pass
        with open(self._calibration_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        logger.info("Saved IMU calibration to %s", self._calibration_path)

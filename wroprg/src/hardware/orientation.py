""" Orientation estimation using MPU6050 data with Kalman filters."""
import math
import time
import logging
from board import SCL, SDA
import busio
import adafruit_bno055
import adafruit_tca9548a
# Using BNO055 for full orientation (fusion on-chip)
from base.shutdown_handling import ShutdownInterface
from utils.threadingfunctions import ConstantUpdateThread
logger = logging.getLogger(__name__)

class OrientationEstimator(ShutdownInterface):
    """
    Estimates orientation using BNO055
    """

    def __init__(self, device_channel):

        # Sensors on I2C: use BNO055 for fused orientation
        # device_channel is expected to be a TCA channel (i2c proxy) or an I2C object
        # keep reference to channel so we can re-create the driver if needed
        self._device_channel = device_channel
        try:
            self.bno = adafruit_bno055.BNO055_I2C(self._device_channel)
        except Exception:
            logger.exception('Failed to initialize BNO055')
            self.bno = None

        # State kept in DEGREES
        self.yaw = 0.0
        # Yaw zero-reference (degrees). Reported yaw = yaw - this offset.
        self._yaw_zero_offset_deg = 0.0
        # If True, the first valid reading will set the yaw zero offset.
        self._zero_on_first_valid = True

        # start update thread around 100Hz (10 ms interval)
        self._thread = ConstantUpdateThread(self.update, interval_ms=4)

        # BNO sanitizer for spike rejection / smoothing
        # Simplified: sanitizer no longer attempts device reinitialization.
        self._bno_sanitizer = self._BNOSanitizer()
        # Update-rate measurement
        self._updates_since_rate = 0
        self._rate_log_interval = 1000
        self._rate_last_time = time.monotonic()


    def reset_yaw(self):
        """Reset Yaw"""
        # Use the current fused reading as the zero reference. If no reading
        # is available, mark to zero on first valid update.
        try:
            euler = None
            if self.bno is not None:
                euler = self.bno.euler
        except Exception:
            euler = None

        if euler is None or euler[0] is None:
            self._zero_on_first_valid = True
            return

        # store current heading as offset (degrees)
        self._yaw_zero_offset_deg = (euler[0] or 0.0)

    def get_yaw(self) -> float:
        """Returns the current yaw"""
        # Apply zero-offset and normalize to [-180, 180)
        raw = (self.yaw - getattr(self, '_yaw_zero_offset_deg', 0.0) + 180.0) % 360.0 - 180.0
        return raw

    def get_anomaly_count(self) -> int:
        """Return the current anomaly counter from the BNO sanitizer."""
        return int(getattr(self._bno_sanitizer, 'anomaly_count', 0))

    def reset_device(self):
        """Attempt to reset/recreate the BNO device and sanitizer state.

        This is a manual operation the caller can invoke when they want to
        re-establish the driver connection. It does not run automatically.
        """
        logger.info('Resetting BNO device and sanitizer')
        try:
            if hasattr(self, '_device_channel') and self._device_channel is not None:
                try:
                    self.bno = adafruit_bno055.BNO055_I2C(self._device_channel)
                    logger.info('BNO055 recreated')
                except Exception:
                    logger.exception('BNO recreate failed')
            # Reset sanitizer internal state
            if hasattr(self._bno_sanitizer, 'reset'):
                self._bno_sanitizer.reset()
        except Exception:
            logger.exception('reset_device encountered an exception')

    def shutdown(self):
        """Shutdown readings"""
        self._thread.stop()

    def start_readings(self):
        """Start Reading"""
        # Defer yaw zeroing to the first valid fused update if no valid
        # reading currently available. reset_yaw will set the flag.
        self.reset_yaw()
        self._thread.start()

    def update(self):

        # Read BNO Euler angles (heading, roll, pitch) and apply sanitization
        euler = None
        try:
            if self.bno is not None:
                # adafruit_bno055.BNO055_I2C.euler returns (heading, roll, pitch)
                euler = self.bno.euler
        except Exception as e:
            logger.debug("BNO read failed: %s", e)

        # If euler data not available, keep previous values
        if euler is None or euler[0] is None:
            # no new measurement, keep fused state
            return

        # euler is (heading, roll, pitch) in degrees on Adafruit driver
        heading, _, _ = euler

        # sanitize and smooth using BNOSanitizer
        yaw = self._bno_sanitizer.sanitize(heading)

        # Ensure yaw wraps consistently
        self.yaw = self._wrap_angle_deg(yaw)

        # Apply one-time yaw-zeroing if requested
        if getattr(self, '_zero_on_first_valid', False):
            self._yaw_zero_offset_deg = self.yaw
            self._zero_on_first_valid = False
        # Track update count and measure rate
        self._updates_since_rate += 1
        if self._updates_since_rate >= self._rate_log_interval:
            now = time.monotonic()
            elapsed = now - getattr(self, '_rate_last_time', now)
            if elapsed <= 0:
                hz = float('inf')
            else:
                hz = self._updates_since_rate / elapsed
            logger.info('Orientation update rate: %.2f Hz over %d updates', hz,\
                                 self._updates_since_rate)
            # reset counters
            self._updates_since_rate = 0
            self._rate_last_time = now

    @staticmethod
    def _wrap_angle_deg(angle: float) -> float:
        """Wrap angle to [-180, 180)."""
        a = (angle + 180.0) % 360.0 - 180.0
        # Map -180 to 180 if needed to keep continuity (optional)
        return a


    # --- BNO spike filter / sanitizer ---
    class _BNOSanitizer:
        def __init__(self):
            self.last_valid = None
            self.smoothed = None
            self.anomaly_count = 0
            self.ANGLE_ABS_LIMIT = 1000.0
            self.MAX_STEP_DEG = 60.0
            self.EMA_ALPHA = 0.15
            self.ANOMALY_RESET_COUNT = 10

        def sanitize(self, raw_angle_deg):
            """Sanitize raw angle readings."""
            if raw_angle_deg is None or (isinstance(raw_angle_deg, float) \
                                         and math.isnan(raw_angle_deg)):
                self._count_anomaly('nan')
                return self._fallback()
            if abs(raw_angle_deg) > self.ANGLE_ABS_LIMIT:
                self._count_anomaly('abs_limit')
                return self._fallback()
            raw = (raw_angle_deg + 180.0) % 360.0 - 180.0
            if self.last_valid is None:
                self.last_valid = raw
                self.smoothed = raw
                self.anomaly_count = 0
                return raw
            d = (raw - self.last_valid + 180.0) % 360.0 - 180.0
            if abs(d) > self.MAX_STEP_DEG:
                self._count_anomaly(f'jump {d:.1f}')
                # Limit the step size to avoid large spikes, but update
                # last_valid so we don't repeatedly correct the same jump.
                corrected = (self.last_valid + math.copysign(self.MAX_STEP_DEG, d) + 180.0) % \
                                                                        360.0 - 180.0
                self.last_valid = corrected
                return self._apply_smoothing(corrected)
            else:
                self.last_valid = raw
                self.anomaly_count = 0
                return self._apply_smoothing(raw)

        def _apply_smoothing(self, angle):
            if self.smoothed is None:
                self.smoothed = angle
            else:
                diff = (angle - self.smoothed + 180.0) % 360.0 - 180.0
                self.smoothed = ((self.smoothed + self.EMA_ALPHA * diff + 180.0) % 360.0) - 180.0
            return self.smoothed

        def _count_anomaly(self, reason):
            self.anomaly_count += 1
            # Use debug level for intermittent anomalies to avoid log spam.
            logger.debug('BNO anomaly #%d: %s', self.anomaly_count, reason)
            # If too many anomalies occur, simply reset the counter and leave
            # recovery to caller via reset_device(). We intentionally do not
            # attempt to reinitialize hardware here.
            if self.anomaly_count >= self.ANOMALY_RESET_COUNT:
                logger.error('BNO repeated anomalies; manual reset recommended')
                self.anomaly_count = 0

        def _fallback(self):
            return self.smoothed if self.smoothed is not None else 0.0

        def reset(self):
            """Reset internal sanitizer state."""
            self.last_valid = None
            self.smoothed = None
            self.anomaly_count = 0

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
        yaw = orientation_estimator.get_yaw()
        logger.info("Orientation: Yaw: %.2f",  yaw)
        time.sleep(0.5) # main method sleep

if __name__ == "__main__":
    main()

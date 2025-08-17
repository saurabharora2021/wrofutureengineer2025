"""
Mock implementation of Adafruit MPU6050 for development on non-RPi platforms.

This provides the same interface as adafruit_mpu6050 but works on any platform.
"""
import logging
import random
import math

logger = logging.getLogger(__name__)

class MPU6050:
    """Mock implementation of Adafruit MPU6050 accelerometer/gyroscope sensor."""

    def __init__(self, i2c, address=0x68):
        """Initialize with I2C bus and optional address."""
        self.i2c = i2c
        self.address = address
        self._simulated_angle = 0  # degrees
        self._simulated_motion = 0  # amount of motion 0-1
        logger.info("Mock MPU6050 initialized on address 0x%02X", address)

    @property
    def acceleration(self):
        """
        Acceleration measured by the sensor in m/s².
        Returns a 3-tuple of X, Y, Z axis values.
        """
        # Generate realistic mock values that change slightly between readings
        # Default orientation: Z is affected by gravity (~9.8 m/s²)
        angle_rad = math.radians(self._simulated_angle)

        # Basic values representing "at rest" with some noise
        accel_x = math.sin(angle_rad) * 9.8 + (random.random() - 0.5) * 0.2
        accel_y = math.cos(angle_rad) * 9.8 + (random.random() - 0.5) * 0.2
        accel_z = 9.8 + (random.random() - 0.5) * 0.2

        # Add some motion if simulated
        if self._simulated_motion > 0:
            motion_factor = self._simulated_motion * 2  # Scale for more noticeable effect
            accel_x += (random.random() - 0.5) * motion_factor
            accel_y += (random.random() - 0.5) * motion_factor
            accel_z += (random.random() - 0.5) * motion_factor

        logger.debug("Mock acceleration: X=%0.2f, Y=%0.2f, Z=%0.2f", accel_x, accel_y, accel_z)
        return (accel_x, accel_y, accel_z)

    @property
    def gyro(self):
        """
        Gyroscope measure in rad/s.
        Returns a 3-tuple of X, Y, Z axis values.
        """
        # Generate mock gyroscope values with some randomness
        # At rest, values should be close to 0
        base_gyro_x = (random.random() - 0.5) * 0.1
        base_gyro_y = (random.random() - 0.5) * 0.1
        base_gyro_z = (random.random() - 0.5) * 0.1

        # Add motion if simulated
        if self._simulated_motion > 0:
            motion_factor = self._simulated_motion * 0.5
            base_gyro_x += (random.random() - 0.5) * motion_factor
            base_gyro_y += (random.random() - 0.5) * motion_factor
            base_gyro_z += (random.random() - 0.5) * motion_factor

        logger.debug("Mock gyro: X=%0.2f, Y=%0.2f, Z=%0.2f",
                   base_gyro_x, base_gyro_y, base_gyro_z)
        return (base_gyro_x, base_gyro_y, base_gyro_z)

    @property
    def temperature(self):
        """Temperature in Celsius."""
        # Room temperature with small variations
        temp = 22 + (random.random() - 0.5)
        logger.debug("Mock temperature: %0.1f°C", temp)
        return temp

    def simulate_tilt(self, angle_degrees):
        """
        Simulate the sensor being tilted at the given angle.
        
        Args:
            angle_degrees: Tilt angle in degrees (0-360)
        """
        self._simulated_angle = angle_degrees % 360
        logger.info("Mock MPU6050 simulating tilt at %0.1f degrees", self._simulated_angle)

    def simulate_motion(self, amount):
        """
        Simulate motion/vibration amount.
        
        Args:
            amount: Motion amount from 0 (still) to 1 (high motion)
        """
        self._simulated_motion = max(0, min(1, amount))
        logger.info("Mock MPU6050 simulating motion level: %0.1f", self._simulated_motion)

"""Example of using the MPU6050 accelerometer in a cross-platform way."""

import time
import logging
import argparse
import board
import busio

# Conditional import for MPU6050
try:
    import adafruit_mpu6050  # type: ignore
    MPU6050_AVAILABLE = True
except ImportError:
    from utils.mock_mpu6050 import MPU6050
    MPU6050_AVAILABLE = False

def main():
    """Main function for accelerometer test."""
    parser = argparse.ArgumentParser(description="MPU6050 Accelerometer Test")
    parser.add_argument('--logfile', type=str, default='accelerometer.log', help='Path to log file')
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    args = parser.parse_args()

    # Setup logging
    logging.basicConfig(
        filename=args.logfile,
        level=logging.DEBUG if args.debug else logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    logger = logging.getLogger(__name__)

    try:
        # Initialize I2C bus
        i2c = busio.I2C(board.SCL, board.SDA)

        # Initialize MPU6050 sensor
        if MPU6050_AVAILABLE:
            mpu = adafruit_mpu6050.MPU6050(i2c)
            logger.info("Using real MPU6050 accelerometer")
            print("Using real MPU6050 accelerometer")
        else:
            mpu = MPU6050(i2c)
            logger.info("Using mock MPU6050 accelerometer")
            print("Using mock MPU6050 accelerometer (simulation mode)")
            # If using mock, simulate some tilting
            mpu.simulate_tilt(15)  # 15 degrees tilt
            mpu.simulate_motion(0.2)  # Low motion level

        # Read sensor data in a loop
        print("Reading accelerometer data... Press Ctrl+C to stop")
        while True:
            # Read sensor values
            accel_x, accel_y, accel_z = mpu.acceleration
            gyro_x, gyro_y, gyro_z = mpu.gyro
            temperature = mpu.temperature

            # Print values
            print(f"Acceleration: X={accel_x:.2f}, Y={accel_y:.2f}, Z={accel_z:.2f} m/s^2")
            print(f"Gyro: X={gyro_x:.2f}, Y={gyro_y:.2f}, Z={gyro_z:.2f} rad/s")
            print(f"Temperature: {temperature:.1f}°C")
            print("-" * 40)

            time.sleep(0.5)

    except KeyboardInterrupt:
        logger.info("Test terminated by user")
        print("\nTest terminated by user")
    except (IOError, ValueError, RuntimeError) as e:
        logger.error("Error: %s", e)
        print(f"Error: {e}")

if __name__ == "__main__":
    main()

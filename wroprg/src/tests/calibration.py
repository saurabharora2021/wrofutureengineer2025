"""Calibration for yaw"""
import time
import logging
import busio
from board import SCL, SDA
import adafruit_tca9548a
from hardware.orientation import OrientationEstimator
from hardware.hardware_interface import HardwareInterface

logger = logging.getLogger(__name__)
class Calibration:
    """
    Calibration for yaw
    """
    def run_calibration(self) -> None:
        """Run Calibration"""

        # Setup I2C devices
        i2c = busio.I2C(SCL, SDA)
        tca = adafruit_tca9548a.TCA9548A(i2c)

        for channel in range(8):
            if tca[channel].try_lock():
                logger.info("Channel %s:", channel)
                addresses = tca[channel].scan()
                logger.info([hex(address) for address in addresses if address != 0x70])
                tca[channel].unlock()

        device_channel = tca[HardwareInterface.DEVICE_I2C_CHANNEL]

        orientation_estimator = OrientationEstimator(device_channel)
        # Simple run to verify readings are wired; no long calibration here.
        orientation_estimator.start_readings()
        time.sleep(0.1)
        orientation_estimator.shutdown()


def main() -> None:
    """Main method to run calibration"""
    Calibration().run_calibration()


if __name__ == "__main__":
    main()

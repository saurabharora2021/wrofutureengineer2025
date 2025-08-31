import time
from hardware.orientation import OrientationEstimator
from hardware.hardware_interface import HardwareInterface


class Calibration:
    def __init__(self) -> None:
        self.hw = HardwareInterface(stabilize=False)

    def get_acceleration(self):
        """Get the current acceleration from the RPi interface."""
        return self.hw.get_acceleration()

    def get_gyro(self):
        """Get the current gyroscope data from the RPi interface."""
        return self.hw.get_gyro()

    def get_magnetometer(self):
        """Get the current magnetometer data from the RPi interface."""
        return self.hw.get_magnetometer()

    def run_calibration(self) -> None:
        orientation_estimator = OrientationEstimator(
            get_accel=self.get_acceleration,
            get_gyro=self.get_gyro,
            get_mag=self.get_magnetometer,
        )
        # Simple run to verify readings are wired; no long calibration here.
        orientation_estimator.start_readings()
        time.sleep(0.1)
        orientation_estimator.shutdown()


def main() -> None:
    Calibration().run_calibration()


if __name__ == "__main__":
    main()

from hardware.orientation import OrientationEstimator
from hardware.rpi_interface import RpiInterface


class Calibration:


    RpiInterface = RpiInterface(stabilize=False)

    def get_acceleration(self):
        """Get the current acceleration from the RPi interface."""
        return self.RpiInterface.get_acceleration()

    def get_gyro(self):
        """Get the current gyroscope data from the RPi interface."""
        return self.RpiInterface.get_gyro()

    def get_magnetometer(self):
        """Get the current magnetometer data from the RPi interface."""
        return self.RpiInterface.get_magnetometer()

    def run_calibration(self):

        orientation_estimator = OrientationEstimator(
                get_accel=self.get_acceleration,
                get_gyro=self.get_gyro,
                get_mag=self.get_magnetometer
        )

        orientation_estimator.calibrate_sensors()


def main():
    calibration = Calibration()
    calibration.run_calibration()


if __name__ == "__main__":
    main()
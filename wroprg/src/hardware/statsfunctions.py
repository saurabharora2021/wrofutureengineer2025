"""This module contains implementations of various statistical functions and filters
 used in hardware sensor data processing."""
class KalmanFilter:
    """ Implementation of a Kalman filter for sensor data smoothing. """
    def __init__(self, process_variance, measurement_variance, estimated_error,
                 initial_value:float,max_value:float):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimated_error = estimated_error
        self.posteri_estimate = initial_value
        self.max_value = max_value  # Maximum value for the filter output

    def update(self, measurement):
        """ Update the Kalman filter with a new measurement. """
        # Prediction update
        priori_estimate = self.posteri_estimate
        priori_error = self.estimated_error + self.process_variance

        # Measurement update
        kalman_gain = priori_error / (priori_error + self.measurement_variance)
        self.posteri_estimate = priori_estimate + kalman_gain * (measurement - priori_estimate)
        self.estimated_error = (1 - kalman_gain) * priori_error

        if self.max_value == measurement:
            return self.max_value
        else:
            return self.posteri_estimate

class DumpKalmanFilter:
    """ A Kalman filter that does not perform any filtering, used for debugging. """
    def __init__(self, process_variance, measurement_variance, estimated_error,
                 initial_value,max_value:float):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimated_error = estimated_error
        self.posteri_estimate = initial_value
        self.max_value = max_value

    def update(self, measurement):
        """ Returns the measurement without any filtering. """
        return measurement

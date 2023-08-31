import numpy as np

class KalmanFilter:
    def __init__(self, initial_state, initial_estimate_error, process_variance, measurement_variance):
        self.state_estimate = initial_state
        self.estimate_error = initial_estimate_error
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance

    def update(self, measurement):
        # Predict
        predicted_state = self.state_estimate
        predicted_error = self.estimate_error + self.process_variance

        # Update
        kalman_gain = predicted_error / (predicted_error + self.measurement_variance)
        self.state_estimate = predicted_state + kalman_gain * (measurement - predicted_state)
        self.estimate_error = (1 - kalman_gain) * predicted_error

        return self.state_estimate




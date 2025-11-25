import numpy as np
import math

def wrap_angle(a):
    return (a + np.pi) % (2*np.pi) - np.pi


class EKF:
    def __init__(self):
        # State vector (x, y, theta)
        self.x = np.zeros(3)

        # Covariance matrix
        self.P = np.eye(3) * 0.01

        # Process noise
        self.Q = np.diag([
            0.01,   # x noise
            0.01,   # y noise
            0.005    # theta noise
        ])

        # GPS measurement noise
        self.R = np.diag([
            0.0001,     # GPS x noise (m^2)
            0.0001,     # GPS y noise (m^2)
            0.0005      # GPS heading noise (rad^2)
        ])

        # Measurement matrix (identity)
        self.H = np.eye(3)


    # ---------------------------------------------------------
    # PREDICTION STEP (called every odometry update)
    # ---------------------------------------------------------
    def predict(self, dx, dy, dtheta):
        # Update state using motion increments
        self.x[0] += dx
        self.x[1] += dy
        self.x[2] += dtheta
        self.x[2] = wrap_angle(self.x[2])

        # Jacobian F (approx for small increments)
        F = np.array([
            [1, 0, -dy],
            [0, 1,  dx],
            [0, 0, 1]
        ])

        # Update covariance
        self.P = F @ self.P @ F.T + self.Q


    # ---------------------------------------------------------
    # UPDATE STEP (called only when GPS packet arrives)
    # ---------------------------------------------------------
    def update(self, gps_x, gps_y, gps_theta):
        z = np.array([gps_x, gps_y, wrap_angle(gps_theta)])

        # Innovation
        y = z - self.x
        y[2] = wrap_angle(y[2])  # angle normalization

        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Update state
        self.x = self.x + K @ y
        self.x[2] = wrap_angle(self.x[2])

        # Update covariance
        I = np.eye(3)
        self.P = (I - K @ self.H) @ self.P

        return self.x.copy()
    
    def set_pos(self, x, y, theta):
        self.x[0] = x
        self.x[1] = y
        self.x[2] = theta
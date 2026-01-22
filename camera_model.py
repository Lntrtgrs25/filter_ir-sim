# camera_model.py
import numpy as np
import math

class CameraModel:
    def __init__(self,
                 fov_deg=120,
                 max_range=5.0,
                 noise_std=0.05,
                 drop_prob=0.1):
        self.fov = math.radians(fov_deg)
        self.max_range = max_range
        self.noise_std = noise_std
        self.drop_prob = drop_prob

    def observe(self, robot_state, ball_state):
        rx, ry, rtheta = robot_state
        bx, by = ball_state

        # relative position
        dx = bx - rx
        dy = by - ry

        # rotate to robot frame
        x_rel = math.cos(-rtheta)*dx - math.sin(-rtheta)*dy
        y_rel = math.sin(-rtheta)*dx + math.cos(-rtheta)*dy

        distance = math.hypot(x_rel, y_rel)
        angle = math.atan2(y_rel, x_rel)

        # out of view
        if distance > self.max_range:
            return None
        if abs(angle) > self.fov / 2:
            return None

        # random drop
        if np.random.rand() < self.drop_prob:
            return None

        # noisy measurement
        z_x = x_rel + np.random.normal(0, self.noise_std)
        z_y = y_rel + np.random.normal(0, self.noise_std)

        return np.array([z_x, z_y])

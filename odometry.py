import math, time

class DifferentialDriveOdometry:
    def __init__(self, wheel_radius, wheel_base, ticks_per_rev):
        self.r = wheel_radius
        self.L = wheel_base
        self.ticks_per_rev = ticks_per_rev

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # radians

        self.prev_ticks_left = 0
        self.prev_ticks_right = 0

        self.history = []
        self.max_history_size = 5000


    def _save_history(self):
        now = time.time()
        self.history.append((now, self.x, self.y, self.theta))

        if len(self.history) > self.max_history_size:
            self.history = self.history[-self.max_history_size:]

    def get_pose_at_time(self, timestamp):
        if not self.history:
            return None

        # Find the entry with timestamp closest to requested time
        closest = min(self.history, key=lambda h: abs(h[0] - timestamp))
        return closest

    def update(self, ticks_left, ticks_right):
        # Delta ticks
        d_ticks_left  = ticks_left - self.prev_ticks_left
        d_ticks_right = ticks_right - self.prev_ticks_right
        self.prev_ticks_left = ticks_left
        self.prev_ticks_right = ticks_right

        # Convert ticks to distance
        d_left  = 2 * math.pi * self.r * d_ticks_left / self.ticks_per_rev
        d_right = 2 * math.pi * self.r * d_ticks_right / self.ticks_per_rev

        # Compute pose increment
        d_center = (d_left + d_right) / 2.0
        d_theta  = (d_right - d_left) / self.L

        # Save old pose
        old_x = self.x
        old_y = self.y
        old_theta = self.theta

        # Update pose
        self.x += d_center * math.cos(self.theta + d_theta / 2)
        self.y += d_center * math.sin(self.theta + d_theta / 2)
        self.theta += d_theta

        # Normalize theta
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Compute increments
        dx = self.x - old_x
        dy = self.y - old_y
        dtheta = self.theta - old_theta

        self._save_history()

        return self.x, self.y, self.theta, dx, dy, dtheta

    def setPos(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = (theta + math.pi) % (2 * math.pi) - math.pi

        self.history.clear()
        self.history.append((time.time(), self.x, self.y, self.theta))

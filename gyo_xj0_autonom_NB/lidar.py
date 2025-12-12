# gyo_xj0_autonom_NB/lidar.py

import math


class SimpleLidar:
    def __init__(
        self,
        fov_deg=120,
        num_rays=31,
        max_range=2.0,
        step=0.02,
    ):
        self.fov = math.radians(fov_deg)
        self.num_rays = num_rays
        self.max_range = max_range
        self.step = step

        self.angles = [
            -self.fov / 2 + i * self.fov / (self.num_rays - 1)
            for i in range(self.num_rays)
        ]

    def scan(self, robot_x, robot_y, robot_yaw, obstacles):
        """
        Returns:
            list of tuples: (angle [rad], distance [m or None])
        """
        results = []

        for angle in self.angles:
            hit_distance = None
            ray_angle = robot_yaw + angle

            dx = math.cos(ray_angle)
            dy = math.sin(ray_angle)

            dist = 0.0
            while dist <= self.max_range:
                px = robot_x + dx * dist
                py = robot_y + dy * dist

                if self._hit_obstacle(px, py, obstacles):
                    hit_distance = dist
                    break

                dist += self.step

            results.append((angle, hit_distance))

        return results

    @staticmethod
    def _hit_obstacle(px, py, obstacles):
        for obs in obstacles:
            ox, oy, r = obs["x"], obs["y"], obs["r"]
            if math.hypot(px - ox, py - oy) <= r:
                return True
        return False

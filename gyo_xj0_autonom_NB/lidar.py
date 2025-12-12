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
        Pure geometric LIDAR scan.

        Args:
            robot_x, robot_y: robot position [m]
            robot_yaw: robot heading [rad]
            obstacles: list of dicts {x, y, r}

        Returns:
            list of (angle [rad], distance [m or None])
        """
        scan_data = []

        for angle in self.angles:
            ray_angle = robot_yaw + angle
            dx = math.cos(ray_angle)
            dy = math.sin(ray_angle)

            distance = None
            d = 0.0

            while d <= self.max_range:
                px = robot_x + dx * d
                py = robot_y + dy * d

                if self._hit(px, py, obstacles):
                    distance = d
                    break

                d += self.step

            scan_data.append((angle, distance))

        return scan_data

    @staticmethod
    def _hit(px, py, obstacles):
        for obs in obstacles:
            ox, oy, r = obs["x"], obs["y"], obs["r"]
            if math.hypot(px - ox, py - oy) <= r:
                return True
        return False

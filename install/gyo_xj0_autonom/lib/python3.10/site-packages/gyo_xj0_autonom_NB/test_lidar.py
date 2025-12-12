from lidar import SimpleLidar
import math

lidar = SimpleLidar()

robot_x = 0.0
robot_y = 0.0
robot_yaw = 0.0

obstacles = [
    {"x": 0.6, "y": 0.0, "r": 0.15},
]

scan = lidar.scan(robot_x, robot_y, robot_yaw, obstacles)

for angle, dist in scan:
    if dist is not None:
        print(f"angle={math.degrees(angle):.1f}°, dist={dist:.2f} m")

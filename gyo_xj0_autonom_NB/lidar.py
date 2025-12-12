import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

class Lidar(Node):
    def __init__(self):
        super().__init__('lidar')
        self.pub = self.create_publisher(Marker, '/lidar', 10)
        self.timer = self.create_timer(0.1, self.publish)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.obstacles = [(1.2, 0.2), (1.8, -0.4), (1.0, 0.8)]
        self.max_range = 2.0
        self.hit_dist = 0.3

    def publish(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lidar"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.03
        marker.color.a = 1.0

        angles = [i * math.pi / 12 for i in range(-6, 7)]

        for a in angles:
            angle = self.robot_yaw + a
            dx = math.cos(angle)
            dy = math.sin(angle)

            hit = False
            for ox, oy in self.obstacles:
                t = (ox - self.robot_x) * dx + (oy - self.robot_y) * dy
                if t > 0:
                    cx = self.robot_x + dx * t
                    cy = self.robot_y + dy * t
                    d = math.hypot(ox - cx, oy - cy)
                    if d < self.hit_dist and t < self.max_range:
                        hit = True

            p1 = Point(x=self.robot_x, y=self.robot_y, z=0.0)
            p2 = Point(
                x=self.robot_x + dx * self.max_range,
                y=self.robot_y + dy * self.max_range,
                z=0.0
            )

            marker.points.append(p1)
            marker.points.append(p2)

            if hit and abs(a) < math.pi / 4:
                marker.colors.append(self.color(1, 0, 0))
                marker.colors.append(self.color(1, 0, 0))
            else:
                marker.colors.append(self.color(0, 1, 0))
                marker.colors.append(self.color(0, 1, 0))

        self.pub.publish(marker)

    def color(self, r, g, b):
        from std_msgs.msg import ColorRGBA
        return ColorRGBA(r=r, g=g, b=b, a=1.0)

def main():
    rclpy.init()
    node = Lidar()
    rclpy.spin(node)
    rclpy.shutdown()

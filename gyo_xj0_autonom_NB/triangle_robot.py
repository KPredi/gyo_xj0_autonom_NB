import math
import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class TriangleRobot(Node):
    def __init__(self):
        super().__init__('triangle_robot')

        self.pub = self.create_publisher(Marker, '/robot_marker', 10)
        self.timer = self.create_timer(0.1, self.update)

        # Robot állapot
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Útvonal (checkpointok)
        self.path = [
            (15.0, 0.0),
            (15.0, -5.0),
            (5.0, -5.0),
            (5.0, -10.0)
        ]
        self.target_index = 0

        self.speed = 0.3       # m/s
        self.turn_speed = 1.5  # rad/s

        self.get_logger().info('TriangleRobot moving')

    def update(self):
        if self.target_index >= len(self.path):
            self.publish_marker()
            return

        tx, ty = self.path[self.target_index]

        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.yaw)

        # Először fordul
        if abs(angle_error) > 0.1:
            self.yaw += math.copysign(self.turn_speed * 0.1, angle_error)
        else:
            # Ha jó irányba néz, előre megy
            self.x += self.speed * math.cos(self.yaw) * 0.1
            self.y += self.speed * math.sin(self.yaw) * 0.1

        # Cél elérve?
        if dist < 0.2:
            self.target_index += 1

        self.publish_marker()

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'robot'
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        size = 0.6

        # Irányított háromszög
        p1 = Point(
            x=self.x + size * math.cos(self.yaw),
            y=self.y + size * math.sin(self.yaw),
            z=0.0
        )
        p2 = Point(
            x=self.x + size * math.cos(self.yaw + 2.5),
            y=self.y + size * math.sin(self.yaw + 2.5),
            z=0.0
        )
        p3 = Point(
            x=self.x + size * math.cos(self.yaw - 2.5),
            y=self.y + size * math.sin(self.yaw - 2.5),
            z=0.0
        )

        marker.points = [p1, p2, p3]

        self.pub.publish(marker)

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def main():
    rclpy.init()
    node = TriangleRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

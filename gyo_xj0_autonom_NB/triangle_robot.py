import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker


class TriangleRobot(Node):

    def __init__(self):
        super().__init__('triangle_robot')

        self.publisher = self.create_publisher(
            Marker,
            '/robot_marker',
            10
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.timer = self.create_timer(0.05, self.update)

        # Robot állapot
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.v = 0.0
        self.w = 0.0

    def cmd_callback(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update(self):
        dt = 0.05

        # Differenciál kinematika
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt

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
        marker.color.b = 0.0
        marker.color.a = 1.0

        size = 0.3
        local_points = [
            ( size,  0.0),
            (-size,  size),
            (-size, -size)
        ]

        for px, py in local_points:
            x = self.x + px * math.cos(self.yaw) - py * math.sin(self.yaw)
            y = self.y + px * math.sin(self.yaw) + py * math.cos(self.yaw)

            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        self.publisher.publish(marker)


def main():
    rclpy.init()
    node = TriangleRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

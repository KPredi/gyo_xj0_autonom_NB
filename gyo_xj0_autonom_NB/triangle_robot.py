import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

class TriangleRobot(Node):
    def __init__(self):
        super().__init__('triangle_robot')
        self.pub = self.create_publisher(Marker, '/robot_marker', 10)
        self.timer = self.create_timer(0.1, self.publish)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def publish(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot"
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

        size = 0.4
        pts = [
            ( size, 0),
            (-size/2,  size/2),
            (-size/2, -size/2)
        ]

        for px, py in pts:
            p = Point()
            p.x = self.x + px * math.cos(self.yaw) - py * math.sin(self.yaw)
            p.y = self.y + px * math.sin(self.yaw) + py * math.cos(self.yaw)
            p.z = 0.0
            marker.points.append(p)

        self.pub.publish(marker)

def main():
    rclpy.init()
    node = TriangleRobot()
    rclpy.spin(node)
    rclpy.shutdown()

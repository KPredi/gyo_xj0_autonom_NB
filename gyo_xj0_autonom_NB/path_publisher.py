import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        self.publisher = self.create_publisher(
            Marker,
            '/path_marker',
            10
        )

        self.timer = self.create_timer(0.5, self.publish_path)

        self.get_logger().info('PathPublisher node started')

    def publish_path(self):
        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.1  # vonal vastagság

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # 90°-os törésekkel rendelkező útvonal
        path_points = [
            (0.0, 0.0),
            (15.0, 0.0),
            (15.0, -5.0),
            (5.0, -5.0),
            (5.0, -10.0)
        ]

        marker.points.clear()

        for x, y in path_points:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        self.publisher.publish(marker)


def main():
    rclpy.init()
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

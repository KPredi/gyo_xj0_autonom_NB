import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')

        self.publisher = self.create_publisher(
            Marker,
            '/path_marker',
            10
        )

        self.timer = self.create_timer(0.5, self.publish_path)

        # FIX, KANYARGÓS ÚTVONAL (jobbra–balra)
        self.path_points = [
            (0.0, 0.0),
            (1.0, 0.5),
            (2.0, -0.5),
            (3.0, 0.7),
            (4.0, -0.3),
            (5.0, 0.0),
        ]

    def publish_path(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.05  # vonal vastagság

        marker.color = ColorRGBA(
            r=0.0,
            g=0.0,
            b=0.0,
            a=1.0
        )

        for x, y in self.path_points:
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

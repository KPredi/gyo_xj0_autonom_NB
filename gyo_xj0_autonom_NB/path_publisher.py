#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.pub = self.create_publisher(Marker, '/path_marker', 10)

        # Ugyanaz a path, mint a robotban (ha módosítod, módosítsd mindkettőben)
        self.path = [
            (0.0, 0.0),
            (15.0, 0.0),
            (15.0, -5.0),
            (5.0, -5.0),
            (5.0, 5.0),
        ]

        self.timer = self.create_timer(0.2, self.publish)
        self.get_logger().info('PathPublisher publishing /path_marker')

    def publish(self):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'path'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.12
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0
        m.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in self.path]
        self.pub.publish(m)


def main():
    rclpy.init()
    node = PathPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
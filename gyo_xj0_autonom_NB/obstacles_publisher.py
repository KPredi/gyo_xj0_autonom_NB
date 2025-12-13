import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class ObstaclesPublisher(Node):
    def __init__(self):
        super().__init__('obstacles_publisher')

        self.pub = self.create_publisher(Marker, '/obstacles_marker', 10)
        self.timer = self.create_timer(0.5, self.publish_obstacles)

        self.get_logger().info('ObstaclesPublisher started')

    def publish_obstacles(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'obstacles'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD

        # téglalap "pixelek" mérete
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points.clear()

        # ---------- 1. akadály (hosszú, keskeny) ----------
        for x in [7.0 + i * 0.4 for i in range(8)]:
            p = Point(x=x, y=0.0, z=0.0)
            marker.points.append(p)

        # ---------- 2. akadály (kicsi) ----------
        for dx in [0.0, 0.4]:
            for dy in [0.0, 0.4]:
                p = Point(x=15.0 + dx, y=-2.5 + dy, z=0.0)
                marker.points.append(p)

        # ---------- 3. akadály (közepes) ----------
        for dx in [0.0, 0.4, 0.8]:
            for dy in [0.0, 0.4]:
                p = Point(x=8.0 + dx, y=-5.0 + dy, z=0.0)
                marker.points.append(p)

        self.pub.publish(marker)


def main():
    rclpy.init()
    node = ObstaclesPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

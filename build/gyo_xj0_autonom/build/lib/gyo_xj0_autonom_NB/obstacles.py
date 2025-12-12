import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class Obstacles(Node):
    def __init__(self):
        super().__init__('obstacles')
        self.pub = self.create_publisher(Marker, '/obstacles', 10)
        self.timer = self.create_timer(0.5, self.publish)

    def publish(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obstacles"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for x, y in [(1.2, 0.2), (1.8, -0.4), (1.0, 0.8)]:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        self.pub.publish(marker)

def main():
    rclpy.init()
    node = Obstacles()
    rclpy.spin(node)
    rclpy.shutdown()

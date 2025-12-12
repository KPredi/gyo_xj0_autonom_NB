import math
import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray, Twist

from gyo_xj0_autonom_NB.lidar import SimpleLidar


class TriangleRobot(Node):
    def __init__(self):
        super().__init__('triangle_robot')

        self.marker_pub = self.create_publisher(
            Marker,
            'robot_marker',
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.create_subscription(
            PoseArray,
            'obstacles',
            self.obstacle_callback,
            10
        )

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.v = 0.2
        self.obstacles = []

        self.lidar = SimpleLidar()

        self.timer = self.create_timer(0.1, self.update)

    def obstacle_callback(self, msg):
        self.obstacles = []
        for p in msg.poses:
            self.obstacles.append({
                "x": p.position.x,
                "y": p.position.y,
                "r": p.position.z
            })

    def update(self):
        scan = self.lidar.scan(
            self.x,
            self.y,
            self.yaw,
            self.obstacles
        )

        stop = False
        for angle, dist in scan:
            if abs(math.degrees(angle)) < 15:
                if dist is not None and dist < 0.5:
                    stop = True
                    break

        if stop:
            self.v = 0.0
            self.get_logger().info("OBSTACLE AHEAD -> STOP")
        else:
            self.v = 0.2

        self.x += math.cos(self.yaw) * self.v * 0.1
        self.y += math.sin(self.yaw) * self.v * 0.1

        self.publish_robot()
        self.publish_cmd()

    def publish_robot(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD

        size = 0.2
        p1 = self.point(self.x + size, self.y)
        p2 = self.point(self.x - size, self.y + size / 2)
        p3 = self.point(self.x - size, self.y - size / 2)

        marker.points = [p1, p2, p3]

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.v
        self.cmd_pub.publish(msg)

    @staticmethod
    def point(x, y):
        from geometry_msgs.msg import Point
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.0
        return p


def main():
    rclpy.init()
    node = TriangleRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

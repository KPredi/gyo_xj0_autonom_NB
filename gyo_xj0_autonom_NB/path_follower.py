import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import math


class PathFollower(Node):

    def __init__(self):
        super().__init__('path_follower')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Point, '/target_point', self.cb, 10)

        self.target = None
        self.timer = self.create_timer(0.1, self.update)

        self.Kp_ang = 1.5
        self.Kp_lin = 0.5

    def cb(self, msg):
        self.target = msg

    def update(self):
        if self.target is None:
            return

        x = self.target.x
        y = self.target.y

        angle = math.atan2(y, x)
        dist = math.sqrt(x*x + y*y)

        cmd = Twist()
        cmd.angular.z = self.Kp_ang * angle
        cmd.linear.x = min(self.Kp_lin * dist, 0.3)

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Pose2D
from visualization_msgs.msg import Marker


class TriangleRobot(Node):
    """
    Rácsos (90°) pályakövetés: mindig csak X vagy csak Y irányban mozog.
    /lidar_stop = True -> azonnal STOP (nem lép tovább).
    Publikál:
      - /robot_marker (Marker)
      - /robot_pose (Pose2D)  <-- lidar ehhez igazodik
    Feliratkozik:
      - /lidar_stop (Bool)
    """

    def __init__(self):
        super().__init__('triangle_robot')

        # Robot állapot
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.speed = 0.35  # m/s
        self.dt = 0.05     # 20 Hz
        self.tol = 0.15    # waypoint tolerancia (ne vágja le túl korán)

        # Stop jel
        self.lidar_stop = False
        self.create_subscription(Bool, '/lidar_stop', self._lidar_cb, 10)

        # Publikálók
        self.robot_pub = self.create_publisher(Marker, '/robot_marker', 10)
        self.pose_pub = self.create_publisher(Pose2D, '/robot_pose', 10)

        # Útvonal (90°-os szegmensek, checkpointok)
        # Itt állítsd, ha más pályát akarsz.
        self.path = [
            (0.0, 0.0),
            (15.0, 0.0),
            (15.0, -5.0),
            (5.0, -5.0),
            (5.0, 5.0),
        ]
        self.target_index = 1  # 0 az induló pont

        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info('TriangleRobot running. Sub: /lidar_stop Pub: /robot_marker /robot_pose')

    def _lidar_cb(self, msg: Bool):
        self.lidar_stop = bool(msg.data)

    def update(self):
        # Ha STOP -> nem mozgunk, de publikálunk (hogy RViz-ben látszódjon)
        if self.lidar_stop:
            self.publish_pose()
            self.publish_robot_marker()
            return

        # Ha vége a pályának -> állj meg stabilan (nincs rángatás)
        if self.target_index >= len(self.path):
            self.publish_pose()
            self.publish_robot_marker()
            return

        tx, ty = self.path[self.target_index]
        dx = tx - self.x
        dy = ty - self.y

        # Rácsos követés: előbb X-t nullázzuk, aztán Y-t (vagy fordítva – itt X az első)
        if abs(dx) > self.tol:
            step = math.copysign(self.speed * self.dt, dx)
            self.x += step
            self.yaw = 0.0 if step >= 0.0 else math.pi
        elif abs(dy) > self.tol:
            step = math.copysign(self.speed * self.dt, dy)
            self.y += step
            self.yaw = math.pi / 2.0 if step >= 0.0 else -math.pi / 2.0
        else:
            # Waypoint elérve -> következő
            self.target_index += 1

        self.publish_pose()
        self.publish_robot_marker()

    def publish_pose(self):
        msg = Pose2D()
        msg.x = float(self.x)
        msg.y = float(self.y)
        msg.theta = float(self.yaw)
        self.pose_pub.publish(msg)

    def publish_robot_marker(self):
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

        # Robot szín: ha stop -> piros, ha megy -> zöld
        if self.lidar_stop:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        marker.color.a = 1.0

        size = 0.6
        # Háromszög lokális pontjai (előre +x)
        local_pts = [( size, 0.0), (-size,  size/2), (-size, -size/2)]

        marker.points = []
        for lx, ly in local_pts:
            rx = math.cos(self.yaw) * lx - math.sin(self.yaw) * ly + self.x
            ry = math.sin(self.yaw) * lx + math.cos(self.yaw) * ly + self.y
            p = Point(x=float(rx), y=float(ry), z=0.0)
            marker.points.append(p)

        self.robot_pub.publish(marker)


def main():
    rclpy.init()
    node = TriangleRobot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
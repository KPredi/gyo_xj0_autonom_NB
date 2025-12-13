#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Pose2D
from visualization_msgs.msg import Marker, MarkerArray


def ray_aabb_intersection(rx, ry, rdx, rdy, minx, maxx, miny, maxy):
    """
    Ray-AABB metszés 2D-ben.
    Visszaadja a legkisebb pozitív t paramétert (távolságot), vagy None-t.
    Ray: (rx,ry) + t*(rdx,rdy), t>=0
    """
    # X slab
    if abs(rdx) < 1e-9:
        if rx < minx or rx > maxx:
            return None
        tx_min, tx_max = -math.inf, math.inf
    else:
        tx1 = (minx - rx) / rdx
        tx2 = (maxx - rx) / rdx
        tx_min, tx_max = (tx1, tx2) if tx1 <= tx2 else (tx2, tx1)

    # Y slab
    if abs(rdy) < 1e-9:
        if ry < miny or ry > maxy:
            return None
        ty_min, ty_max = -math.inf, math.inf
    else:
        ty1 = (miny - ry) / rdy
        ty2 = (maxy - ry) / rdy
        ty_min, ty_max = (ty1, ty2) if ty1 <= ty2 else (ty2, ty1)

    tmin = max(tx_min, ty_min)
    tmax = min(tx_max, ty_max)

    if tmax < 0 or tmin > tmax:
        return None

    # belépési pont tmin; ha tmin<0, akkor a ray a dobozban indul
    t = tmin if tmin >= 0 else tmax
    return t if t >= 0 else None


class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')

        # LIDAR paraméterek
        self.max_range = 4.0          # m
        self.front_fov_deg = 40.0     # -20..+20
        self.ray_step_deg = 10.0      # sugarak 10 fokonként
        self.stop_distance = 0.9      # m (ha ennél közelebb -> STOP)

        self.robot_pose = Pose2D()
        self.have_pose = False
        self.obstacles = []  # list of (minx,maxx,miny,maxy)

        self.create_subscription(Pose2D, '/robot_pose', self.pose_cb, 10)
        self.create_subscription(MarkerArray, '/obstacles', self.obs_cb, 10)

        self.stop_pub = self.create_publisher(Bool, '/lidar_stop', 10)
        self.marker_pub = self.create_publisher(Marker, '/lidar_marker', 10)

        self.timer = self.create_timer(0.1, self.update)
        self.get_logger().info('LidarNode started. Pub: /lidar_stop /lidar_marker Sub: /robot_pose /obstacles')

    def pose_cb(self, msg: Pose2D):
        self.robot_pose = msg
        self.have_pose = True

    def obs_cb(self, msg: MarkerArray):
        rects = []
        for m in msg.markers:
            cx = m.pose.position.x
            cy = m.pose.position.y
            sx = m.scale.x
            sy = m.scale.y
            minx = cx - sx / 2.0
            maxx = cx + sx / 2.0
            miny = cy - sy / 2.0
            maxy = cy + sy / 2.0
            rects.append((minx, maxx, miny, maxy))
        self.obstacles = rects

    def update(self):
        if not self.have_pose:
            return

        rx = self.robot_pose.x
        ry = self.robot_pose.y
        yaw = self.robot_pose.theta

        # Lidar rays
        half = self.front_fov_deg / 2.0
        angles_deg = [a for a in self._frange(-half, half, self.ray_step_deg)]

        blocked = False
        min_front_dist = None

        # RViz marker: LINE_LIST sugarak
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lidar'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.03

        # alapból kék sugarak
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.points = []

        for a_deg in angles_deg:
            a = math.radians(a_deg)
            ang = yaw + a
            rdx = math.cos(ang)
            rdy = math.sin(ang)

            # legközelebbi metszés
            best_t = None
            for (minx, maxx, miny, maxy) in self.obstacles:
                t = ray_aabb_intersection(rx, ry, rdx, rdy, minx, maxx, miny, maxy)
                if t is None:
                    continue
                if t > self.max_range:
                    continue
                if best_t is None or t < best_t:
                    best_t = t

            dist = best_t if best_t is not None else self.max_range

            # STOP döntés csak előre
            if dist < self.stop_distance:
                blocked = True
                if min_front_dist is None or dist < min_front_dist:
                    min_front_dist = dist

            # Marker szakasz
            p0 = Point(x=float(rx), y=float(ry), z=0.05)
            p1 = Point(x=float(rx + rdx * dist), y=float(ry + rdy * dist), z=0.05)
            marker.points.append(p0)
            marker.points.append(p1)

        # ha blocked -> piros sugarak
        if blocked:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

        self.marker_pub.publish(marker)
        self.stop_pub.publish(Bool(data=blocked))

    def _frange(self, start, stop, step):
        x = start
        # hogy pontosan kijöjjön a stop is
        while x <= stop + 1e-9:
            yield round(x, 6)
            x += step


def main():
    rclpy.init()
    node = LidarNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
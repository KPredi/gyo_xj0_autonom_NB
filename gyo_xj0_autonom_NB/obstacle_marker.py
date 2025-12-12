import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray, Pose


class ObstacleMarker(Node):
    def __init__(self):
        super().__init__('obstacle_marker')

        self.marker_pub = self.create_publisher(
            Marker,
            'obstacle_marker',
            10
        )

        self.data_pub = self.create_publisher(
            PoseArray,
            'obstacles',
            10
        )

        # ---- WORLD MODEL (EGY HELYEN DEFINIÁLVA) ----
        self.obstacle = {
            "x": 0.6,
            "y": 0.0,
            "r": 0.15
        }

        self.timer = self.create_timer(0.1, self.publish)

    def publish(self):
        # ---------- RViz MARKER ----------
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obstacles"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = self.obstacle["x"]
        marker.pose.position.y = self.obstacle["y"]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.obstacle["r"] * 2
        marker.scale.y = self.obstacle["r"] * 2
        marker.scale.z = 0.1

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

        # ---------- OBSTACLE DATA ----------
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = marker.header.stamp

        pose = Pose()
        pose.position.x = self.obstacle["x"]
        pose.position.y = self.obstacle["y"]
        pose.position.z = self.obstacle["r"]  # radius Z-ben
        pose.orientation.w = 1.0

        pose_array.poses.append(pose)
        self.data_pub.publish(pose_array)


def main():
    rclpy.init()
    node = ObstacleMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

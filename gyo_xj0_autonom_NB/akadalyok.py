import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray


class Akadalyok(Node):
    """
    3 db téglalap akadály (CUBE marker). Közzéteszi MarkerArray-ként: /obstacles
    A LIDAR node ebből számol (raycast).
    """

    def __init__(self):
        super().__init__('akadalyok')
        self.kotetevo = self.create_publisher(MarkerArray, '/obstacles', 10)

        # 1) hosszú, keskeny az első hosszú egyenes közepénél
        # 2) kisebb a következő szakaszon
        # 3) átlagos a harmadik szakaszon
        self.akadalyok = [
            # (cx, cy, sx, sy)
            (7.5, 0.0, 0.8, 3.5),     # hosszú (Y irányban nyúlik), keskeny X
            (15.0, -2.5, 1.0, 1.0),   # kicsi
            (9.0, -10.0, 2.0, 1.2),    # átlagos
        ]

        self.idozito = self.create_timer(0.2, self.kozzetesz)
        self.get_logger().info('Akadályok közzététele /obstacles (MarkerArray)')

    def kozzetesz(self):
        tomb = MarkerArray()
        for i, (cx, cy, sx, sy) in enumerate(self.akadalyok):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'obstacles'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD

            m.pose.position.x = float(cx)
            m.pose.position.y = float(cy)
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0

            m.scale.x = float(sx)
            m.scale.y = float(sy)
            m.scale.z = 0.4

            m.color.r = 0.3
            m.color.g = 0.3
            m.color.b = 0.3
            m.color.a = 1.0

            tomb.markers.append(m)

        self.kotetevo.publish(tomb)


def main():
    rclpy.init()
    node = Akadalyok()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
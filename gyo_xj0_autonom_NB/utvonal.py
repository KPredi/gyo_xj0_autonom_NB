import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Polygon, Point32


class UtvonalKiado(Node):
    def __init__(self):
        super().__init__('utvonal_kiado')
        self.marker_pub = self.create_publisher(Marker, '/path_marker', 10)
        self.utvonal_pub = self.create_publisher(Polygon, '/robot_path', 10)

        # Csak itt van megadva az útvonal
        self.utvonal = [
            (0.0, 0.0),
            (15.0, 0.0),
            (15.0, -10.0),
            (5.0, -10.0),
            (5.0, -20.0),
        ]

        self.idozito = self.create_timer(0.2, self.kozzetesz)
        self.get_logger().info('UtvonalKiado elindult - közzéteszi /path_marker és /robot_path topicokat')
        
        # Azonnal közzétesszük az útvonalat (nem csak az idozitoban)
        self.kozzetesz_utvonalat_egyszer()

    def kozzetesz_utvonalat_egyszer(self):  # JAVÍTVA: 'kozzetek' helyett 'kozzetesz'
        """Közzéteszi az útvonalat Polygon üzenet formájában"""
        msg = Polygon()
        for x, y in self.utvonal:
            p = Point32()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            msg.points.append(p)
        self.utvonal_pub.publish(msg)
        self.get_logger().info(f'Utvonal közzétéve {len(self.utvonal)} ponttal')

    def kozzetesz(self):
        # Közzétesszük a vizualizációt
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
        m.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in self.utvonal]
        self.marker_pub.publish(m)
        
        # Minden 5. alkalommal (1 másodpercenként) közzétesszük újra az útvonalat is
        self.kozzetesz_szamlalo = getattr(self, 'kozzetesz_szamlalo', 0) + 1
        if self.kozzetesz_szamlalo % 5 == 0:
            self.kozzetesz_utvonalat_egyszer()  # JAVÍTVA: Itt is 'kozzetesz' kell


def main():
    rclpy.init()
    node = UtvonalKiado()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
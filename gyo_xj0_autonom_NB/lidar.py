import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Pose2D
from visualization_msgs.msg import Marker, MarkerArray


def sugarkocka_metszes(rx, ry, rdx, rdy, minx, maxx, miny, maxy):
    """
    Sugár-AABB metszés 2D-ben.
    Visszaadja a legkisebb pozitív t paramétert (távolságot), vagy None-t.
    Sugár: (rx,ry) + t*(rdx,rdy), t>=0
    """
    # X sík
    if abs(rdx) < 1e-9:
        if rx < minx or rx > maxx:
            return None
        tx_min, tx_max = -math.inf, math.inf
    else:
        tx1 = (minx - rx) / rdx
        tx2 = (maxx - rx) / rdx
        tx_min, tx_max = (tx1, tx2) if tx1 <= tx2 else (tx2, tx1)

    # Y sík
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

    t = tmin if tmin >= 0 else tmax
    return t if t >= 0 else None


class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')

        # LIDAR paraméterek
        self.max_tavolsag = 4.0          # m
        self.elso_fov_fok = 40.0         # -20..+20 (megállás érzékelés)
        self.oldal_kezdo_fok = 30.0      # +/-30°-nál kezdődik az oldalsó érzékelés
        self.oldal_veg_fok = 110.0       # +/-110°-ig tart
        self.sugar_lepes_fok = 10.0      # sugarak 10 fokonként
        self.megallas_tavolsag = 1       # m (ha ennél közelebb -> MEGÁLL)
        self.oldal_erzekeles_tav = 1     # m (oldalsó érzékelés távolsága)
        
        # Biztonságos zóna paraméterek
        self.oldal_biztonsagos_zona = 1  # m (biztonsági zóna - ha ennél távolabb, már biztonságos)

        self.robot_pozicio = Pose2D()
        self.van_pozicio = False
        self.akadalyok = []  # lista (minx,maxx,miny,maxy) alakban

        self.create_subscription(Pose2D, '/robot_pose', self.pozicio_cb, 10)
        self.create_subscription(MarkerArray, '/obstacles', self.akadaly_cb, 10)

        # Közzétevők
        self.megallas_pub = self.create_publisher(Bool, '/lidar_stop', 10)
        self.oldal_bal_pub = self.create_publisher(Bool, '/lidar_side_left', 10)
        self.oldal_jobb_pub = self.create_publisher(Bool, '/lidar_side_right', 10)
        # Új: Biztonságos zóna állapot
        self.oldal_bal_biztonsagos_pub = self.create_publisher(Bool, '/lidar_side_left_safe', 10)
        self.oldal_jobb_biztonsagos_pub = self.create_publisher(Bool, '/lidar_side_right_safe', 10)
        # Két marker: egy a sugaraknak, egy a biztonságos zónának
        self.marker_pub = self.create_publisher(Marker, '/lidar_marker', 10)
        self.biztonsagos_zona_marker_pub = self.create_publisher(Marker, '/lidar_safezone_marker', 10)

        self.idozito = self.create_timer(0.1, self.frissites)
        self.get_logger().info('LidarNode elindult. Elülső látómező: -20..+20°, Oldalsó látómező: -110..-30° és 30..110°, Biztonságos zóna: 1m')

    def pozicio_cb(self, msg: Pose2D):
        self.robot_pozicio = msg
        self.van_pozicio = True

    def akadaly_cb(self, msg: MarkerArray):
        teglalapok = []
        for m in msg.markers:
            cx = m.pose.position.x
            cy = m.pose.position.y
            sx = m.scale.x
            sy = m.scale.y
            minx = cx - sx / 2.0
            maxx = cx + sx / 2.0
            miny = cy - sy / 2.0
            maxy = cy + sy / 2.0
            teglalapok.append((minx, maxx, miny, maxy))
        self.akadalyok = teglalapok

    def frissites(self):
        if not self.van_pozicio:
            return

        rx = self.robot_pozicio.x
        ry = self.robot_pozicio.y
        irany = self.robot_pozicio.theta

        # 1. ELÜLSŐ LIDAR (-20°..+20°) - MEGÁLLÁS érzékelés
        elul_blokkolt = False
        elul_min_tav = None
        fel_elul = self.elso_fov_fok / 2.0
        elul_szogek = [a for a in self._f_tartomany(-fel_elul, fel_elul, self.sugar_lepes_fok)]

        for a_fok in elul_szogek:
            tav = self._sugar_tavolsag(rx, ry, irany, a_fok)
            if tav is not None and tav < self.megallas_tavolsag:
                elul_blokkolt = True
                if elul_min_tav is None or tav < elul_min_tav:
                    elul_min_tav = tav

        # 2. OLDALSÓ LIDAR (-110°..-30° és +30°..+110°)
        oldal_bal_blokkolt = False
        oldal_jobb_blokkolt = False
        oldal_bal_biztonsagos = True  # Alapból biztonságos (nincs közel akadály)
        oldal_jobb_biztonsagos = True  # Alapból biztonságos (nincs közel akadály)
        
        # Bal oldali szektor (-110°..-30°)
        bal_szogek = [a for a in self._f_tartomany(-self.oldal_veg_fok, -self.oldal_kezdo_fok, self.sugar_lepes_fok)]
        
        for a_fok in bal_szogek:
            tav = self._sugar_tavolsag(rx, ry, irany, a_fok)
            if tav is not None:
                # Blokkolt-e? 
                if tav < self.oldal_erzekeles_tav:
                    oldal_bal_blokkolt = True
                
                # Biztonságos zónán belül van-e? 
                if tav < self.oldal_biztonsagos_zona:
                    oldal_bal_biztonsagos = False  # VESZÉLYES: van akadály a biztonságos zónán belül
        
        # Jobb oldali szektor (+30°..+110°)
        jobb_szogek = [a for a in self._f_tartomany(self.oldal_kezdo_fok, self.oldal_veg_fok, self.sugar_lepes_fok)]
        
        for a_fok in jobb_szogek:
            tav = self._sugar_tavolsag(rx, ry, irany, a_fok)
            if tav is not None:
                # Blokkolt-e? 
                if tav < self.oldal_erzekeles_tav:
                    oldal_jobb_blokkolt = True
                
                # Biztonságos zónán belül van-e? 
                if tav < self.oldal_biztonsagos_zona:
                    oldal_jobb_biztonsagos = False  # VESZÉLYES: van akadály a biztonságos zónán belül

        # Közzététel
        self.megallas_pub.publish(Bool(data=elul_blokkolt))
        self.oldal_bal_pub.publish(Bool(data=oldal_bal_blokkolt))
        self.oldal_jobb_pub.publish(Bool(data=oldal_jobb_blokkolt))
        # Biztonságos zóna állapot (fordított logika: True = biztonságos, False = veszélyes)
        self.oldal_bal_biztonsagos_pub.publish(Bool(data=oldal_bal_biztonsagos))
        self.oldal_jobb_biztonsagos_pub.publish(Bool(data=oldal_jobb_biztonsagos))

        # Vizuális markerek
        self._publikald_lidar_markerek(rx, ry, irany, elul_szogek, bal_szogek, jobb_szogek, 
                                    elul_blokkolt, oldal_bal_blokkolt, oldal_jobb_blokkolt,
                                    oldal_bal_biztonsagos, oldal_jobb_biztonsagos)
        
        # Új: Biztonságos zóna marker (lila kör)
        self._publikald_biztonsagos_zona_marker(rx, ry, irany)

    def _sugar_tavolsag(self, rx, ry, irany, szog_fok):
        """Egy sugár távolságának számítása"""
        a = math.radians(szog_fok)
        teljes_szog = irany + a
        rdx = math.cos(teljes_szog)
        rdy = math.sin(teljes_szog)

        legjobb_t = None
        for (minx, maxx, miny, maxy) in self.akadalyok:
            t = sugarkocka_metszes(rx, ry, rdx, rdy, minx, maxx, miny, maxy)
            if t is None:
                continue
            if legjobb_t is None or t < legjobb_t:
                legjobb_t = t

        # Ha nincs találat, adjunk vissza egy nagy számot (max_tavolsag + 1)
        return legjobb_t if legjobb_t is not None else (self.max_tavolsag + 1.0)

    def _publikald_lidar_markerek(self, rx, ry, irany, elul_szogek, bal_szogek, jobb_szogek,
                              elul_blokkolt, bal_blokkolt, jobb_blokkolt,
                              bal_biztonsagos, jobb_biztonsagos):
        """LIDAR sugarak megjelenítése RViz-ben"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lidar_rays'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.points = []

        # Összes szög egy listában
        osszes_szog = elul_szogek + bal_szogek + jobb_szogek
        
        for a_fok in osszes_szog:
            tav = self._sugar_tavolsag(rx, ry, irany, a_fok)
            a = math.radians(a_fok)
            teljes_szog = irany + a
            rdx = math.cos(teljes_szog)
            rdy = math.sin(teljes_szog)
            
            p0 = Point(x=float(rx), y=float(ry), z=0.05)
            
            # JAVÍTÁS: Mindig max_tavolsag-ig rajzoljuk a sugarakat
            tav = min(tav, self.max_tavolsag)
            
            p1 = Point(x=float(rx + rdx * tav), y=float(ry + rdy * tav), z=0.05)
            marker.points.append(p0)
            marker.points.append(p1)
        
        # Színek az érzékelési állapotok alapján
        if elul_blokkolt:
            marker.color.r = 1.0  # Piros, ha előtte akadály
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif bal_blokkolt or jobb_blokkolt:
            marker.color.r = 1.0  # Sárga, ha oldalt akadály
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif not bal_biztonsagos or not jobb_biztonsagos:
            # Ha nincs biztonságos zónán belül akadály, de blokkolt sincs
            marker.color.r = 0.0  # Kék, ha semmi
            marker.color.g = 0.0
            marker.color.b = 1.0
        else:
            marker.color.r = 0.0  # Kék, ha semmi
            marker.color.g = 0.0
            marker.color.b = 1.0
            
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def _publikald_biztonsagos_zona_marker(self, rx, ry, irany):
        """Biztonságos zóna megjelenítése RViz-ben (lila kör)"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lidar_safezone'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Vonalvastagság
        
        # LILA szín a biztonságos zónának
        marker.color.r = 0.8  # Lila: piros + kék
        marker.color.g = 0.0
        marker.color.b = 0.8
        marker.color.a = 0.7  # Átlátszóbb
        
        marker.points = []
        
        # Kör rajzolása a biztonságos zóna távolsággal 
        pontok_szama = 36  # 10°-onként
        for i in range(pontok_szama + 1):
            szog = 2.0 * math.pi * i / pontok_szama
            px = rx + self.oldal_biztonsagos_zona * math.cos(szog)
            py = ry + self.oldal_biztonsagos_zona * math.sin(szog)
            marker.points.append(Point(x=float(px), y=float(py), z=0.02))
        
        self.biztonsagos_zona_marker_pub.publish(marker)
        
        # Extra: nyíl a robot irányába a biztonságos zóna körön belül
        nyil_marker = Marker()
        nyil_marker.header.frame_id = 'map'
        nyil_marker.header.stamp = self.get_clock().now().to_msg()
        nyil_marker.ns = 'lidar_safezone'
        nyil_marker.id = 1
        nyil_marker.type = Marker.ARROW
        nyil_marker.action = Marker.ADD
        nyil_marker.scale.x = 0.1
        nyil_marker.scale.y = 0.2
        nyil_marker.scale.z = 0.1
        
        nyil_marker.color.r = 0.8  # Lila
        nyil_marker.color.g = 0.0
        nyil_marker.color.b = 0.8
        nyil_marker.color.a = 1.0
        
        # Nyíl pozíciója és iránya
        nyil_hossz = self.oldal_biztonsagos_zona * 0.8  # A kör belsejében
        nyil_marker.points = []
        nyil_marker.points.append(Point(x=float(rx), y=float(ry), z=0.02))
        nyil_marker.points.append(Point(
            x=float(rx + nyil_hossz * math.cos(irany)),
            y=float(ry + nyil_hossz * math.sin(irany)),
            z=0.02
        ))
        
        self.biztonsagos_zona_marker_pub.publish(nyil_marker)

    def _f_tartomany(self, start, stop, step):
        x = start
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
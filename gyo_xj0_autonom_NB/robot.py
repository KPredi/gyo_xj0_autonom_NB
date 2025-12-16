import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Pose2D, Polygon, Point32
from visualization_msgs.msg import Marker


class TriangleRobot(Node):
    """
    Robot állapotgéppel a teljes kerüléshez:
    - FOLLOW_PATH: Normál útvonalkövetés
    - TURN_RIGHT: 90° jobbra fordulás (kerülés kezdete)
    - GO_SIDEWAYS: Oldalra haladás (akadály mellett)
    - WAIT_AFTER_OBSTACLE: Várás/rövid továbbhaladás az akadály elhagyása után
    - TURN_LEFT_1: 90° balra fordulás (vissza eredeti irányba)
    - GO_FORWARD_AFTER_TURN: Fix távolság megtétele a fordulás után, majd LIDAR ellenőrzés
    - TURN_LEFT_2: 90° balra fordulás (az útvonalra merőlegesen)
    - GO_TO_PATH: Haladás az eredeti útvonal felé
    - TURN_RIGHT_2: 90° jobbra fordulás (vissza az eredeti irányba)
    """

    def __init__(self):
        super().__init__('triangle_robot')

        # Robot állapot
        self.x = 0.0
        self.y = 0.0
        self.elfordulas = 0.0

        # Sebesség paraméterek
        self.normal_sebesseg = 1  # m/s (útvonal követés)
        self.fordulas_sebesseg = 1.0     # rad/s (fordulási sebesség)
        self.dt = 0.05            # 20 Hz
        self.tolerancia = 0.07           # waypoint tolerancia
        self.vonal_tolerancia = 0.07       # útvonal vonalának toleranciája

        # LIDAR jelek
        self.lidar_megallas = False
        self.oldal_bal_blokkolt = False
        self.oldal_jobb_blokkolt = False
        self.oldal_bal_biztonsagos = True  # True = biztonságos (nincs akadály a biztonságos zónán belül)
        self.oldal_jobb_biztonsagos = True # True = biztonságos (nincs akadály a biztonságos zónán belül)
        
        # Feliratkozások
        self.create_subscription(Bool, '/lidar_stop', self._lidar_megallas_cb, 10)
        self.create_subscription(Bool, '/lidar_side_left', self._oldal_bal_cb, 10)
        self.create_subscription(Bool, '/lidar_side_right', self._oldal_jobb_cb, 10)
        self.create_subscription(Bool, '/lidar_side_left_safe', self._oldal_bal_biztonsagos_cb, 10)
        self.create_subscription(Bool, '/lidar_side_right_safe', self._oldal_jobb_biztonsagos_cb, 10)
        self.create_subscription(Polygon, '/robot_path', self._utvonal_cb, 10)

        # Közzétevők
        self.robot_pub = self.create_publisher(Marker, '/robot_marker', 10)
        self.pozicio_pub = self.create_publisher(Pose2D, '/robot_pose', 10)

        # Útvonal (üres kezdetben, majd a path_cb fogja betölteni)
        self.utvonal = []
        self.cel_index = 1  # 0 az induló pont
        self.van_utvonal = False  # Jelzi, hogy már megkaptuk-e az útvonalat
        self.varakozas_utvonalra_szamlalo = 0  # Várakozási számláló

        # Állapotgép változók
        self.allapot = "INIT"  # Kezdeti állapot, várjuk az útvonalat
        self.kerules_kezdet_pozicio = None  # (x, y, elfordulas)
        self.kerules_tavolsag = 0.0
        self.fordulas_celfordulatszam = 0.0
        self.eredeti_utvonal_index = 1  # Hol hagytuk abba az útvonalat
        self.eredeti_irany = ""  # "KELE", "NYUGAT", "ÉSZAK", "DÉL"
        self.megallas_pozicio = None  # Hol állt meg először (x, y)
        self.akadaly_tavolsag = 0.0  # Mennyit mentünk az akadály elhagyása után
        self.akadaly_kuszob = 0.2  # MÉG x métert kell menni az akadály elhagyása után
        
        # Javított paraméterek a GO_FORWARD_AFTER_TURN állapothoz
        self.fordulas_utan_elore_tavolsag = 1  # Fix távolság a balra fordulás után (méter)
        self.fordulas_utan_elore_megtett = 0.0  # Mennyit mentünk már ebben az állapotban
        self.varja_biztonsagos_zonat = False  # Jelzi, hogy már megtette a fix távolságot és a biztonságos zónára vár
        self.biztonsagos_utan_extra_tavolsag = 0.4  # Extra távolság miután a biztonságos zóna biztonságos
        self.extra_tavolsag_megtett = 0.0  # Mennyit mentünk az extra távolságból
        
        # DEBUG: Naplózzuk a LIDAR értékeket
        self.log_szamlalo = 0
        self.log_intervallum = 10  # Minden 10. frissítésnél naplózzunk
        
        # Állapotváltás naplózás
        self.utolso_allapot = ""
        
        self.idozito = self.create_timer(self.dt, self.frissites)
        self.get_logger().info('TriangleRobot elindult - várja az útvonalat a /robot_path topicról')

    def _lidar_megallas_cb(self, msg: Bool):
        self.lidar_megallas = bool(msg.data)

    def _oldal_bal_cb(self, msg: Bool):
        self.oldal_bal_blokkolt = bool(msg.data)

    def _oldal_jobb_cb(self, msg: Bool):
        self.oldal_jobb_blokkolt = bool(msg.data)

    def _oldal_bal_biztonsagos_cb(self, msg: Bool):
        self.oldal_bal_biztonsagos = bool(msg.data)

    def _oldal_jobb_biztonsagos_cb(self, msg: Bool):
        self.oldal_jobb_biztonsagos = bool(msg.data)

    def _utvonal_cb(self, msg: Polygon):
        """Útvonal betöltése a path_publisher-től (Polygon üzenet)"""
        if not self.van_utvonal and len(msg.points) > 0:
            self.utvonal = [(p.x, p.y) for p in msg.points]
            self.van_utvonal = True
            self.get_logger().info(f'Útvonal megkaptva: {len(self.utvonal)} ponttal')
            self.get_logger().info(f'Útvonal pontjai: {self.utvonal}')
            
            # Ha még INIT állapotban vagyunk, váltsunk FOLLOW_PATH-ra
            if self.allapot == "INIT":
                self.allapot = "FOLLOW_PATH"
                self.get_logger().info('Útvonal megérkezett, útvonal követés elkezdve')
        elif not self.van_utvonal:
            self.get_logger().warn('Üres útvonal érkezett, várunk egy érvényes útvonalra...')

    def frissites(self):
        """Fő frissítési függvény"""
        
        # Állapotváltás naplózás
        if self.allapot != self.utolso_allapot:
            self.get_logger().info(f"Állapotváltás: {self.utolso_allapot} -> {self.allapot}")
            self.utolso_allapot = self.allapot
        
        # Ha nincs még útvonal, csak közzétesszük a pozíciót és markert
        if not self.van_utvonal:
            self.varakozas_utvonalra_szamlalo += 1
            
            # Ha túl sokáig várakozunk, kérjük újra az útvonalat
            if self.varakozas_utvonalra_szamlalo % 200 == 0:  # 10 másodperc (200 * 0.05)
                self.get_logger().warn('Még mindig várakozunk az útvonalra. Győződj meg róla, hogy a path_publisher fut.')
            
            self.publikald_poziciot()
            self.publikald_robot_markert()
            self.log_szamlalo += 1
            return
        
        # Állapotgép - csak ha van útvonal
        if self.allapot == "FOLLOW_PATH":
            self._allapot_kovet_utvonalat()
        elif self.allapot == "TURN_RIGHT":
            self._allapot_fordul_jobbra()
        elif self.allapot == "GO_SIDEWAYS":
            self._allapot_oldalra_megy()
        elif self.allapot == "WAIT_AFTER_OBSTACLE":
            self._allapot_var_akadaly_utan()
        elif self.allapot == "TURN_LEFT_1":
            self._allapot_fordul_balra_1()
        elif self.allapot == "GO_FORWARD_AFTER_TURN":
            self._allapot_elore_megy_fordulas_utan()
        elif self.allapot == "TURN_LEFT_2":
            self._allapot_fordul_balra_2()
        elif self.allapot == "GO_TO_PATH":
            self._allapot_megy_utvonalhoz()
        elif self.allapot == "TURN_RIGHT_2":
            self._allapot_fordul_jobbra_2()
        
        # Közzététel mindig (akkor is, ha INIT állapotban vagyunk)
        self.publikald_poziciot()
        self.publikald_robot_markert()
        
        # Növeljük a log számlálót
        self.log_szamlalo += 1

    def _allapot_kovet_utvonalat(self):
        """Normál útvonalkövetés állapot"""
        # Ha MEGÁLLÁS jel érkezett, kezdjük el a kerülést
        if self.lidar_megallas:
            self.allapot = "TURN_RIGHT"
            self.kerules_kezdet_pozicio = (self.x, self.y, self.elfordulas)
            self.megallas_pozicio = (self.x, self.y)  # MEGJEGYZÜK A MEGÁLLÁS POZÍCIÓT
            self.eredeti_utvonal_index = self.cel_index
            
            # Meghatározzuk az eredeti irányt
            self._meghatarozza_eredeti_iranyt()
            
            self.fordulas_celfordulatszam = self.elfordulas - math.pi/2  # 90° jobbra
            self.get_logger().warn("Akadály érzékelve! Kerülési manőver kezdése.")
            self.get_logger().info(f"Megállt pozícióban: ({self.megallas_pozicio[0]:.2f}, {self.megallas_pozicio[1]:.2f})")
            return

        # Ha vége a pályának -> állj meg
        if self.cel_index >= len(self.utvonal):
            return

        # Normál útvonalkövetés
        tx, ty = self.utvonal[self.cel_index]
        dx = tx - self.x
        dy = ty - self.y

        if abs(dx) > self.tolerancia:
            lepes = math.copysign(self.normal_sebesseg * self.dt, dx)
            self.x += lepes
            self.elfordulas = 0.0 if lepes >= 0.0 else math.pi
        elif abs(dy) > self.tolerancia:
            lepes = math.copysign(self.normal_sebesseg * self.dt, dy)
            self.y += lepes
            self.elfordulas = math.pi / 2.0 if lepes >= 0.0 else -math.pi / 2.0
        else:
            # Waypoint elérve -> következő
            self.cel_index += 1

    def _meghatarozza_eredeti_iranyt(self):
        """Meghatározza az eredeti mozgási irányt"""
        aktualis_idx = self.cel_index - 1
        kovetkezo_idx = self.cel_index
        
        if kovetkezo_idx < len(self.utvonal):
            x1, y1 = self.utvonal[aktualis_idx]
            x2, y2 = self.utvonal[kovetkezo_idx]
            
            if abs(x2 - x1) > abs(y2 - y1):  # Vízszintes mozgás
                if x2 > x1:
                    self.eredeti_irany = "KELE"
                else:
                    self.eredeti_irany = "NYUGAT"
            else:  # Függőleges mozgás
                if y2 > y1:
                    self.eredeti_irany = "ÉSZAK"
                else:
                    self.eredeti_irany = "DÉL"
            
            self.get_logger().info(f"Eredeti irány: {self.eredeti_irany}")

    def _allapot_fordul_jobbra(self):
        """90° jobbra fordulás állapot"""
        szog_hiba = self._normalizalt_szog(self.fordulas_celfordulatszam - self.elfordulas)
        
        if abs(szog_hiba) < 0.05:  # ~3°
            self.allapot = "GO_SIDEWAYS"
            self.kerules_tavolsag = 0.0
            self.get_logger().info("Jobbra fordulás befejezve. Oldalra haladás.")
        else:
            # Fordulás a kívánt irányba
            self.elfordulas += self.fordulas_sebesseg * self.dt * (1.0 if szog_hiba > 0 else -1.0)

    def _allapot_oldalra_megy(self):
        """Oldalra haladás (akadály mellett)"""
        # Haladás az aktuális irányba (most már oldalra)
        self.x += math.cos(self.elfordulas) * self.normal_sebesseg * self.dt
        self.y += math.sin(self.elfordulas) * self.normal_sebesseg * self.dt
        self.kerules_tavolsag += self.normal_sebesseg * self.dt

        # Ellenőrizd az oldalsó LIDAR-t (jobb oldal, ami most az "előre" irány)
        if not self.oldal_jobb_blokkolt:
            self.allapot = "WAIT_AFTER_OBSTACLE"
            self.akadaly_tavolsag = 0.0
            self.get_logger().info(f"Akadály elhagyva. További távolság várakozás a fordulás előtt.")

    def _allapot_var_akadaly_utan(self):
        """Várás/rövid továbbhaladás az akadály elhagyása után"""
        # Haladás tovább az aktuális irányba (oldalra)
        self.x += math.cos(self.elfordulas) * self.normal_sebesseg * self.dt
        self.y += math.sin(self.elfordulas) * self.normal_sebesseg * self.dt
        self.akadaly_tavolsag += self.normal_sebesseg * self.dt
        
        # Ha elértük a kívánt további távolságot, akkor forduljunk balra
        if self.akadaly_tavolsag >= self.akadaly_kuszob:
            self.allapot = "TURN_LEFT_1"
            # Vissza az eredeti irányba
            if self.eredeti_irany == "KELE":
                self.fordulas_celfordulatszam = 0.0
            elif self.eredeti_irany == "NYUGAT":
                self.fordulas_celfordulatszam = math.pi
            elif self.eredeti_irany == "ÉSZAK":
                self.fordulas_celfordulatszam = math.pi / 2.0
            elif self.eredeti_irany == "DÉL":
                self.fordulas_celfordulatszam = -math.pi / 2.0
                
            self.get_logger().info(f"{self.akadaly_tavolsag:.2f}m várakozás az akadály után. Balra fordulás az eredeti irányba.")

    def _allapot_fordul_balra_1(self):
        """90° balra fordulás (vissza eredeti irányba)"""
        szog_hiba = self._normalizalt_szog(self.fordulas_celfordulatszam - self.elfordulas)
        
        if abs(szog_hiba) < 0.05:
            self.allapot = "GO_FORWARD_AFTER_TURN"
            self.fordulas_utan_elore_megtett = 0.0  # Reseteljük
            self.varja_biztonsagos_zonat = False  # Reseteljük a biztonságos zóna várakozást
            self.extra_tavolsag_megtett = 0.0  # Reseteljük az extra távolságot
            self.get_logger().info("Első balra fordulás befejezve. Előre haladás fix távolság, majd LIDAR ellenőrzés.")
        else:
            self.elfordulas += self.fordulas_sebesseg * self.dt * (1.0 if szog_hiba > 0 else -1.0)

    def _allapot_elore_megy_fordulas_utan(self):
        """JAVÍTOTT: Fix távolság megtétele, majd LIDAR ellenőrzés (jobb oldali biztonságos zóna), majd extra 0.8m"""
        
        # Mindig haladjunk előre (az eredeti irányban)
        self.x += math.cos(self.elfordulas) * self.normal_sebesseg * self.dt
        self.y += math.sin(self.elfordulas) * self.normal_sebesseg * self.dt
        
        # 1. FÁZIS: Fix távolság megtétele 
        if not self.varja_biztonsagos_zonat:
            self.fordulas_utan_elore_megtett += self.normal_sebesseg * self.dt
            
            # DEBUG: Naplózzuk a haladást
            if self.log_szamlalo % self.log_intervallum == 0:
                self.get_logger().debug(f"GO_FORWARD_AFTER_TURN - 1. fázis (fix távolság): {self.fordulas_utan_elore_megtett:.2f}m / {self.fordulas_utan_elore_tavolsag:.2f}m")
            
            # Ha elértük a fix távolságot, váltsunk a biztonságos zóna ellenőrzésre
            if self.fordulas_utan_elore_megtett >= self.fordulas_utan_elore_tavolsag:
                self.varja_biztonsagos_zonat = True
                self.get_logger().info(f"1. fázis kész: Fix távolság ({self.fordulas_utan_elore_tavolsag:.2f}m) megtéve. Most ellenőrizzük a jobb oldali biztonságos zónát...")
        
        # 2. FÁZIS: Biztonságos zóna ellenőrzés (várakozás amíg biztonságos lesz)
        elif self.varja_biztonsagos_zonat and self.extra_tavolsag_megtett == 0.0:
            # Ellenőrizzük, hogy a jobb oldali biztonságos zóna biztonságos-e (nincs akadály)
            if self.oldal_jobb_biztonsagos:
                self.get_logger().info(f"2. fázis kész: Jobb oldali biztonságos zóna tiszta! Most haladunk további {self.biztonsagos_utan_extra_tavolsag}m...")
                self.extra_tavolsag_megtett = 0.001  # Kis kezdeti érték, hogy belépjünk a 3. fázisba
            else:
                # Még várnunk kell, tovább haladunk és újra ellenőrzünk a következő ciklusban
                if self.log_szamlalo % (self.log_intervallum * 2) == 0:  # Ritkábban naplózzuk
                    self.get_logger().debug(f"Várakozás, hogy a jobb oldali biztonságos zóna tisztuljon... (oldal_jobb_biztonsagos={self.oldal_jobb_biztonsagos})")
        
        # 3. FÁZIS: Extra távolság megtétele miután a biztonságos zóna biztonságos
        elif self.extra_tavolsag_megtett > 0.0:
            self.extra_tavolsag_megtett += self.normal_sebesseg * self.dt
            
            if self.log_szamlalo % self.log_intervallum == 0:
                self.get_logger().debug(f"GO_FORWARD_AFTER_TURN - 3. fázis (extra távolság): {self.extra_tavolsag_megtett:.2f}m / {self.biztonsagos_utan_extra_tavolsag:.2f}m")
            
            if self.extra_tavolsag_megtett >= self.biztonsagos_utan_extra_tavolsag:
                self.allapot = "TURN_LEFT_2"
                # Forduljunk balra (merőlegesen az útvonalra)
                if self.eredeti_irany == "KELE":
                    self.fordulas_celfordulatszam = math.pi / 2.0  # Észak
                elif self.eredeti_irany == "NYUGAT":
                    self.fordulas_celfordulatszam = -math.pi / 2.0  # Dél
                elif self.eredeti_irany == "ÉSZAK":
                    self.fordulas_celfordulatszam = math.pi  # Nyugat
                elif self.eredeti_irany == "DÉL":
                    self.fordulas_celfordulatszam = 0.0  # Kelet
                
                self.get_logger().info(f"3. fázis kész: Extra távolság ({self.biztonsagos_utan_extra_tavolsag:.2f}m) megtéve. Balra fordulás az útvonalhoz való visszatéréshez.")

    def _allapot_fordul_balra_2(self):
        """90° balra fordulás (merőlegesen az útvonalra)"""
        szog_hiba = self._normalizalt_szog(self.fordulas_celfordulatszam - self.elfordulas)
        
        if abs(szog_hiba) < 0.05:
            self.allapot = "GO_TO_PATH"
            self.kerules_tavolsag = 0.0
            self.get_logger().info("Második balra fordulás befejezve. Merőleges haladás az útvonal eléréséhez.")
        else:
            self.elfordulas += self.fordulas_sebesseg * self.dt * (1.0 if szog_hiba > 0 else -1.0)

    def _allapot_megy_utvonalhoz(self):
        """Haladás az eredeti útvonal felé (merőlegesen)"""
        # Haladás a jelenlegi irányba (ami merőleges az útvonalra)
        self.x += math.cos(self.elfordulas) * self.normal_sebesseg * self.dt
        self.y += math.sin(self.elfordulas) * self.normal_sebesseg * self.dt
        self.kerules_tavolsag += self.normal_sebesseg * self.dt
        
        # Ellenőrizzük, hogy elértük-e az eredeti útvonalat
        # Ehhez megnézzük, hogy visszaértünk-e az eredeti megállás pozíció koordinátájához
        utvonal_elerve = False
        
        if self.megallas_pozicio:
            if self.eredeti_irany in ["KELE", "NYUGAT"]:
                # Vízszintes útvonal: y koordináta kell megegyezzen
                tavolsag_vonalhoz = abs(self.y - self.megallas_pozicio[1])
                if tavolsag_vonalhoz < self.vonal_tolerancia:
                    utvonal_elerve = True
                    # Állítsuk be pontosan
                    self.y = self.megallas_pozicio[1]
            else:
                # Függőleges útvonal: x koordináta kell megegyezzen
                tavolsag_vonalhoz = abs(self.x - self.megallas_pozicio[0])
                if tavolsag_vonalhoz < self.vonal_tolerancia:
                    utvonal_elerve = True
                    # Állítsuk be pontosan
                    self.x = self.megallas_pozicio[0]
        
        if utvonal_elerve:
            self.allapot = "TURN_RIGHT_2"
            # Vissza az eredeti irányba
            if self.eredeti_irany == "KELE":
                self.fordulas_celfordulatszam = 0.0
            elif self.eredeti_irany == "NYUGAT":
                self.fordulas_celfordulatszam = math.pi
            elif self.eredeti_irany == "ÉSZAK":
                self.fordulas_celfordulatszam = math.pi / 2.0
            elif self.eredeti_irany == "DÉL":
                self.fordulas_celfordulatszam = -math.pi / 2.0
                
            self.get_logger().info(f"Eredeti útvonal elérve {self.kerules_tavolsag:.2f}m után. Jobbra fordulás az eredeti irányba.")

    def _allapot_fordul_jobbra_2(self):
        """90° jobbra fordulás (vissza az eredeti irányba)"""
        szog_hiba = self._normalizalt_szog(self.fordulas_celfordulatszam - self.elfordulas)
        
        if abs(szog_hiba) < 0.05:
            self.allapot = "FOLLOW_PATH"
            # Folytatjuk az útvonal követést ahol abbahagytuk
            self.cel_index = self.eredeti_utvonal_index
            self.get_logger().info("Végső jobbra fordulás befejezve. Útvonal követés folytatása.")
        else:
            self.elfordulas += self.fordulas_sebesseg * self.dt * (1.0 if szog_hiba > 0 else -1.0)

    def _normalizalt_szog(self, szog):
        """Szög normalizálása -pi és pi között"""
        while szog > math.pi:
            szog -= 2 * math.pi
        while szog < -math.pi:
            szog += 2 * math.pi
        return szog

    def publikald_poziciot(self):
        msg = Pose2D()
        msg.x = float(self.x)
        msg.y = float(self.y)
        msg.theta = float(self.elfordulas)
        self.pozicio_pub.publish(msg)

    def publikald_robot_markert(self):
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

        # Szín az állapot alapján
        szin_tabla = {
            "INIT": (1.0, 1.0, 1.0),  # Fehér (vár az útvonalra)
            "FOLLOW_PATH": (0.0, 1.0, 0.0) if not self.lidar_megallas else (1.0, 0.0, 0.0),  # Zöld/Piros
            "TURN_RIGHT": (1.0, 1.0, 0.0),    # Sárga
            "GO_SIDEWAYS": (0.0, 0.0, 1.0),   # Kék
            "WAIT_AFTER_OBSTACLE": (0.5, 0.5, 0.0),  # Sötétsárga
            "TURN_LEFT_1": (1.0, 0.5, 0.0),   # Narancs
            "GO_FORWARD_AFTER_TURN": (0.8, 0.0, 0.8) if not self.varja_biztonsagos_zonat else (0.4, 0.0, 0.9),  # LILA (fix távolság) / Sötétlila (biztonságos zóna várakozás)
            "TURN_LEFT_2": (0.0, 1.0, 1.0),   # Világoskék
            "GO_TO_PATH": (1.0, 0.0, 1.0),    # Rózsaszín
            "TURN_RIGHT_2": (0.5, 0.5, 0.5),  # Szürke
        }
        
        if self.allapot in szin_tabla:
            r, g, b = szin_tabla[self.allapot]
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
        else:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0  # Fehér ha ismeretlen állapot
            
        marker.color.a = 1.0

        meret = 0.6
        helyi_pontok = [(meret, 0.0), (-meret, meret/2), (-meret, -meret/2)]

        marker.points = []
        for lx, ly in helyi_pontok:
            rx = math.cos(self.elfordulas) * lx - math.sin(self.elfordulas) * ly + self.x
            ry = math.sin(self.elfordulas) * lx + math.cos(self.elfordulas) * ly + self.y
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
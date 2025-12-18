# `gyo_xj0_autonom_NB`

## Nagy beadandó

### Feladat leírása
A projekt célja egy egyszerű autonóm mobilrobot-szimuláció megvalósítása ROS 2 környezetben. A szimulált robot feladata egy előre meghatározott útvonal követése, miközben a LIDAR szenzor adatai alapján képes felismerni és kikerülni az útjába kerülő akadályokat.

### Elkészítésének folyamata
A megvalósítás elején a mobil robotika alapjain használt szimulált autót szerettem volna alapul venni, mivel ez el van látva LIDAR szenzorral és kamerával, viszont úgy láttam, hogy jobb lenne nulláról elkezdeni a feladatot. Így mindent saját módszerrel készült el, az autó egy háromszög lett az egyszerűség kedvéért, az útvonalat egy fekete vonal jelzi a szimulációban, az akadályok szürke kockák lettek és a LIDAR szenzort a kék vonalak helyettesítik. Az elkészítés során több kérdés is szembejött, ilyen volt például a robot mozgása. Végeredményben az egyszerű csak 4 irányú mozgás maradt, mivel ez jelentősen megkönnyítette az elkészítést, kevesebb volt a hibalehetőség a programban és a matematikai számításokban, illetve úgy gondoltam, hogy ez kevésbé fontos része a projektnek. Az akadályok alakja is fontos kérdés, így mivel a téglatest könnyen kezelhető és kiszámítható így arra esett a választás.

## A 4 node tartalma

###Útvonal Kiadó (utvonal.py)
Fő feladata:
  Definiálja az útvonalat
  Közzéteszi az útvonalat két formában:
    /path_marker - Vizualizáció RViz-ben 
    /robot_path - Adatok a robot számára 
    
Fontos részletek:
  Időzítő: Minden 0.2 másodpercben (5Hz) küldi a vizualizációt
  Automatikus küldés: Induláskor azonnal küldi az útvonalat a robotnak
  Ismétlés: Minden 5. alkalommal (1 másodpercenként) újraküldi az adatokat, hogy biztosan megkapja a robot (ez egy hiba elkerülése miatt készült így)
  Koordinátarendszer: Minden pont (x, y) formában, ahol:
    x: vízszintes koordináta
    y: függőleges koordináta

###Akadályok (akadalyok.py)
Fő feladata:
  Létrehoz 3 téglalap alakú akadályt
  Közzéteszi őket RViz-ben megjelenítésre
  
Az akadályok:
  Első akadály
    Középpont: (7.5, 0.0)
    Méret: 0.8m széles × 3.5m magas
  Második akadály
    Középpont: (15.0, -2.5)
    Méret: 1.0m × 1.0m 
  Harmadik akadály 
    Középpont: (9.0, -10.0)
    Méret: 2.0m × 1.2m
    
Működés:
  Minden akadályt CUBE típusú markerrel jelenít meg
  Szürke színűek, 0.4m magasak
  Minden 0.2 másodpercben frissíti a pozíciójukat (bár statikusak)
  A LIDAR ezeket az akadályokat használja a sugár-metszések számításához

###LIDAR Érzékelő (lidar.py)
Fő feladata:
  Szimulálja a robot LIDAR szenzorát
  Érzékeli az akadályokat különböző irányokban
  Küld jeleket a robotnak az akadályokról
  
Érzékelési zónák:
  Elülső zóna (-20°...+20°):
    STOP detektálás
    Ha Xm-nél közelebb van akadály → /lidar_stop = True
  Bal oldali zóna (-110°...-30°):
    Blokkoltság érzékelés: /lidar_side_left
    Biztonságos zóna: /lidar_side_left_safe
  Jobb oldali zóna (30°...110°):
    Blokkoltság érzékelés: /lidar_side_right
    Biztonságos zóna: /lidar_side_right_safe
    
Vizualizáció:
  LIDAR sugarak: Kék vonalak (piros ha előtte akadály, sárga ha oldalt)
  Biztonságos zóna: Lila kör a robot körül

###Robot Irányító (robot.py)
Fő feladata:
  Követi az útvonalat
  Észleli és megkerüli az akadályokat
  Állapotgéppel irányítja a mozgást
  
Állapotgép - Teljes akadálykerülés folyamata:
  1. FOLLOW_PATH - Normál útvonalkövetés
    Követi az útvonal pontjait
    Ha /lidar_stop = True → TURN_RIGHT állapotba vált
  2. TURN_RIGHT - 90° jobbra fordulás
    Fordul 90° jobbra (π/2 radián)
    Cél: oldalra nézzen, hogy az akadály mellett haladhasson
  3. GO_SIDEWAYS - Oldalra haladás
    Halad oldalra (most az akadály mellett)
    Figyeli a jobb oldali LIDAR-t
    Ha nincs többé akadály jobb oldalon → WAIT_AFTER_OBSTACLE
  4. WAIT_AFTER_OBSTACLE - Rövid várakozás
    Még X métert megy tovább, hogy biztosan elhagyja az akadály végét
  5. TURN_LEFT_1 - 90° balra fordulás
    Visszafordul az eredeti irányba
  6. GO_FORWARD_AFTER_TURN - Előre haladás ellenőrzéssel
    Fázis 1: Fix távolság 
      Megtesz X métert az eredeti irányba
      Cél: biztosan túlhaladja az akadályt hosszában
    Fázis 2: Biztonságos zóna várakozás
      Várja, hogy a jobb oldali biztonságos zóna tiszta legyen
      (/lidar_side_right_safe = True)
      Cél: biztosan elhagyta az akadályt oldalirányban is
    Fázis 3: Extra távolság 
      Miután a zóna tiszta, még X métert megy előre
      Extra biztonság, hogy ne legyen túl közel az akadályhoz visszatérve
  7. TURN_LEFT_2 - 90° balra fordulás
    Fordul balra, hogy merőlegesen közeledjen az eredeti útvonalhoz
  8. GO_TO_PATH - Visszatérés az útvonalhoz
    Halad merőlegesen az eredeti útvonal felé
    Amikor eléri az eredeti megállási vonalat → TURN_RIGHT_2
  9. TURN_RIGHT_2 - 90° jobbra fordulás
    Visszafordul az eredeti irányba
  10. Vissza FOLLOW_PATH-ba
    Folytatja az útvonal követést ott

Koordináta-rendszer kezelése:
  Meghatározza, hogy éppen merre halad:
    "KELE" - jobbra (x növekszik)
    "NYUGAT" - balra (x csökken)
    "ÉSZAK" - felfelé (y növekszik)
    "DÉL" - lefelé (y csökken)
  Útvonal visszaállítás:
    Megjegyzi hol hagyta abba az útvonalat (self.eredeti_utvonal_index)
    Megjegyzi a megállási pozíciót (self.megallas_pozicio)
    Visszatérve pontosan ugyanoda tér vissza

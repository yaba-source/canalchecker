# CanalChecker - Roboter-Verfolgungssystem

Ein ROS2-basiertes autonomes Navigationssystem für Roboter, das ArUco-Marker zur Zielerkennung und automatischen Verfolgung nutzt.

## Übersicht

Das CanalChecker-System ist ein vollständig automatisiertes Kontrollsystem für mobile Roboter, das drei spezialisierte Missionstypen durchführt:

1. **Align**: Roboter-Ausrichtung zum Marker
2. **Drive**: Fahrt zu einem definierten Zielabstand
3. **Follow**: Kontinuierliche Verfolgung eines Zielmarkers

Das System verwendet ArUco-Markererkennung für präzise Positions- und Orientierungsverfolgung und implementiert P-Regler für sanfte, kontrollierte Bewegungen.

## Systemvorraussetzungen

- ROS2 (Humble)
- Python 3.10+
- OpenCV mit ArUco-Support
- NumPy
- Roboter mit Odometrie und Geschwindigkeitskontrolle

## Installation

### 1. Workspace klonen
```bash
cd ~
git clone https://github.com/yaba-source/canalchecker.git
cd canalchecker
```

### 2. Dependencies installieren
```bash
sudo apt-get install ros-humble-cv-bridge ros-humble-sensor-msgs
pip install opencv-contrib-python numpy
```

### 3. Workspace bauen
```bash
cd ws
colcon build 
source install/setup.bash
```

## Startup und Launch
### Turtlebot via Launch-File
Der Turtlebot muss als erstes gestartet werden:
```bash 
ros2 launch turtlebot3_bringup robot.launch.py
```

### Hauptstart via Launch-File

Der Haupteinstiegspunkt ist das Launch-File `canalchecker.launch.py`, das alle notwendigen Komponenten startet:

```bash
ros2 launch canalchecker_bringup canalchecker.launch.py
```

Das Launch-File orchestriert folgende Komponenten:

**CameraNode**: Startet die Kamera und führt ArUco-Markererkennung durch. Publiziert erkannte Marker-Informationen auf dem Topic `/aruco_detections`.

**ActionServerHandler**: Zentrale Orchestrierungskomponente, die alle Missionen koordiniert. Überwacht Markererkennung und triggert automatische Modus-Übergänge.

**AlignActionServer**: Action Server für Roboter-Ausrichtung. Dreht den Roboter bis zur Alignment mit dem erkannten Marker.

**DriveActionServer**: Action Server für gezielte Fahrt. Fährt den Roboter zum definierten Zielabstand.

**FollowActionServer**: Action Server für das Folgen eines anderen Roboters. Folgt dem Zielmarker mit Abstands- und Winkelregelung.

**ActionServerHandler**: Client für die Koordination der verschieden Server.



## Projektstruktur

```
canalchecker/
├── ws/                          # ROS2 Workspace
│   ├── src/
│   │   ├── canalchecker_bringup/
│   │   │   └── launch/
│   │   │       └── canalchecker.launch.py    # Hauptstart-Datei
│   │   ├── canalchecker_dev/
│   │   │   ├── ActionServer/
│   │   │   │   ├── AlignActionServer.py
│   │   │   │   ├── DriveActionServer.py
│   │   │   │   └── FollowActionServer.py
│   │   │   ├── logik/
│   │   │   │   ├── AlignLogic.py
│   │   │   │   ├── DriveLogic.py
│   │   │   │   └── FollowLogic.py
│   │   │   ├── MachineVision/
│   │   │   │   ├── ArucoMarkerDetector.py
│   │   │   │   ├── CameraNode.py
│   │   │   │   └── CameraCalibration.py
│   │   │   └── ActionServerHandler.py
│   │   └── canalchecker_interface/
│   │       ├── action/
│   │       │   ├── Align.action
│   │       │   ├── Drive.action
│   │       │   └── Follow.action
│   │       └── msg/
│   │           └── ArucoDetection.msg
│   └── install/
└── Doku/
    └── Dokumentation.md         # Detaillierte Technische Dokumentation
```

## Komponenten

### ActionServerHandler (Zentrale Koordination)

Die Kernkomponente des Systems, die alle Operationen orchestriert:

Startet automatisch mit der Align-Mission. Überwacht kontinuierlich das `/aruco_detections` Topic auf erkannte Marker. Wenn Marker 69 erkannt wird, bricht das System automatisch die laufende Mission ab und startet die Follow-Mission. Nach erfolgreicher Verfolgung (Marker 0 erkannt) kehrt das System zu Align zurück oder wartet auf neue Missionen.

Implementiert Thread-sichere Datenstruktionen für asynchrone Callback-Verarbeitung.

### Action Server

#### AlignActionServer
Richtet den Roboter zum erkannten Marker aus. Nutzt P-Regler für sanfte Drehbewegungen. Timeout nach 60 Sekunden. Feedback zeigt aktuelle Winkelabweichung.

#### DriveActionServer
Fährt den Roboter zu einem definierten Zielabstand. Nutzt P-Regler für kontrollierte Annäherung. Liest Zielabstand dynamisch aus Topic. Feedback zeigt aktuelle Entfernung zum Ziel.

#### FollowActionServer
Kontinuierliche Verfolgung des Zielmarkers. Kombiniert Winkel- und Abstandsregelung. Anpassungsfähig an dynamische Parameter aus Topics. Beendet bei Erkennung von Marker 0 (Zielmarker).

### State Machines

Jeder Action Server nutzt eine dedizierte State Machine für komplexe Verhaltensabläufe:

**AlignLogic**: 2-State Maschine (Suche → Align)
**DriveLogic**: 2-State Maschine (Suche → Fahrt)
**FollowLogic**: 3-State Maschine (Suche → Folge → Fertig)

Alle nutzen P-Regler-basierte Regelungen für präzise Kontrolle.

### Machine Vision

**ArucoMarkerDetector**: Erkennt ArUco-Marker im Videobild. Schätzt 3D-Pose mittels PnP-Algorithmus. Unterstützt mehrere Markergröße basierend auf Marker-ID. Publiziert Abstand und Winkel zu erkannten Markern.

**CameraNode**: Startet Kamera-Interface. Ruft Markererkennung auf. Publiziert ArucoDetection Nachrichten mit 30 Hz.

**CameraCalibration**: Werkzeug zur Kamera-Kalibrierung mit Schachbrett-Muster.

## Topics

### Publizierte Topics

**`/aruco_detections`** (ArucoDetection)
- Erkannte Marker-ID (-1 wenn keine)
- Abstand zum Marker in Metern
- Winkel zum Marker in Grad
- Publiziert von: CameraNode
- Frequenz: 30 Hz

### Abonnierte Topics

**`/max_speed`** (Float32)
- Maximale Fahrtgeschwindigkeit in m/s
- Wird von Action Servern gelesen
- Dynamisch anpassbar während Ausführung

**`/target_distance`** (Float32)
- Zielabstand in Zentimetern für Follow-Mission
- Wird von FollowActionServer gelesen
- Standard: 50 cm

### Publikations-Topics

**`/cmd_vel`** (Twist)
- Roboter-Steuerbefehle
- linear.x: Vorwärts-/Rückwärtsgeschwindigkeit
- angular.z: Drehgeschwindigkeit
- Publiziert von: Action Servern

## Actions

### /align

Ausrichtungs-Action ohne Parameter. Richtet Roboter zum erkannten Marker aus. Feedback: Aktuelle Winkelabweichung. Result: True wenn erfolgreich ausgerichtet.

### /drive

Fahrt-Action ohne Parameter. Fährt zum konfigurierten Zielabstand. Feedback: Aktuelle Entfernung. Result: True wenn Zielabstand erreicht.

### /follow

Folge-Action mit Zielabstand-Parameter. Folgt Marker kontinuierlich. Feedback: Aktuelle Entfernung. Result: True wenn Marker 0 erkannt.

## Konfiguration

Alle Regelparameter sind in den State Machine Dateien konfigurierbar:

### FollowLogic (ws/src/canalchecker_dev/canalchecker_dev/logik/FollowLogic.py)

- `id_to_follow = 69`: Zielmarker-ID
- `kp_angular = 1.1`: Proportional-Koeffizient für Winkelregelung
- `kp_linear = 0.2`: Proportional-Koeffizient für Abstandsregelung
- `max_linear_speed = 0.2`: Maximale Fahrtgeschwindigkeit (m/s)
- `align_angular_speed = 0.05`: Maximale Drehgeschwindigkeit (rad/s)

### AlignLogic (ws/src/canalchecker_dev/canalchecker_dev/logik/AlignLogic.py)

- `kp_angular = 2.0`: Proportional-Koeffizient
- `max_speed = 0.2`: Maximale Drehgeschwindigkeit

### DriveLogic (ws/src/canalchecker_dev/canalchecker_dev/logik/DriveLogic.py)

- `kp = 0.5`: Proportional-Koeffizient
- `max_speed = 0.2`: Maximale Fahrtgeschwindigkeit

### Marker-Größen (ArucoMarkerDetector)

- Marker ID 0: 175 mm
- Marker ID 69: 75 mm

Weitere Marker-Größen können einfach hinzugefügt werden.

## Typical Workflow

### Automatischer Workflow

1. System startet mit `ros2 launch canalchecker_bringup canalchecker.launch.py`
2. CameraNode beginnt Markererkennung
3. ActionServerHandler startet Align-Mission
4. Roboter richtet sich aus
5. System wartet auf Marker 69
6. Wenn Marker 69 erkannt → Automatischer Wechsel zu Follow-Mission
7. Roboter folgt Marker mit Abstands- und Winkelregelung
8. Bei Erkennung von Marker 0 → Follow beendet
9. System kehrt zu Align zurück

### Manuelle Parameter-Anpassung während Laufzeit

```bash
# Maximale Geschwindigkeit ändern (0.0 - 0.2 m/s)
ros2 topic pub /max_speed std_msgs/Float32 "data: 0.1"

# Zielabstand für Follow ändern (in cm)
ros2 topic pub /target_distance std_msgs/Float32 "data: 50.0"

# Topics überwachen
ros2 topic echo /aruco_detections
ros2 topic echo /cmd_vel
```

## Debugging und Überwachung

### Topic-Überwachung

Aruco-Detektionsergebnisse anschauen:
```bash
ros2 topic echo /aruco_detections
```

Roboter-Steuerbefehle überprüfen:
```bash
ros2 topic echo /cmd_vel
```

### Action-Status

Verfügbare Actions auflisten:
```bash
ros2 action list
```

Spezifische Action Information:
```bash
ros2 action info /follow
```

### Häufige Probleme

**Marker werden nicht erkannt**
- Kamera-Setup überprüfen
- Marker unter ausreichend Licht platzieren
- Kamerakalibrierung durchführen

**Roboter dreht sich unkontrolliert**
- kp_angular in FollowLogic reduzieren (z.B. auf 0.8)
- Winkelberechnung in ArucoMarkerDetector überprüfen

**Abstand wird nicht eingehalten**
- kp_linear erhöhen
- /target_distance Topic überprüfen
- Maximale Geschwindigkeit reduzieren

**Action wird nicht abgebrochen**
- ROS2 Action System überprüfen
- Goal Handle Verwaltung debuggen

## Testing

Unit Tests für State Machines:

```bash
cd ws
colcon test --packages-select canalchecker_dev
```

## Detaillierte Dokumentation

Für technische Details siehe `Doku/Dokumentation.md`:
- Vollständige Architektur-Übersicht
- State Machine Beschreibungen
- P-Regler Implementierungen
- Machine Vision Details
- Thread Safety Konzepte

## Lizenz

MIT License - siehe LICENSE Datei

## Kontakt und Support

Bei Fragen oder Problemen:
1. Überprüfe `Doku/Dokumentation.md` für technische Details
2. Überprüfe ROS2 Topics und Actions mit `ros2 topic list` und `ros2 action list`
3. Aktiviere Logging in den Action Servern (Logger-Aufrufe auskommentieren)

## Versionshistorie

- v1.0: Initiale Release mit Align, Drive, Follow Actions
- Support für multiple Marker-Größen
- P-Regler basierte Regelung
- Dynamische Parameter-Anpassung über Topics

---

**Letztes Update**: Januar 2026
**Status**: Produktiv
**Wartung**: Aktiv

# CanalChecker - Gesamtdokumentation

## Inhaltsverzeichnis
1. [Architektur](#architektur)
2. [ActionServerHandler](#actionserverhandler)
3. [Action Server (Align, Drive, Follow)](#action-server)
4. [State Machines & Logik](#state-machines--logik)
5. [Machine Vision (Aruco Detection)](#machine-vision)
6. [Nachrichtenschnittstellen](#nachrichtenschnittstellen)
7. [Workflow & Ausführung](#workflow--ausführung)

---

## Architektur

### Überblick
Das CanalChecker-System folgt einer **hierarchischen ROS2-Architektur** mit einer zentralen Koordinierung.
Die Architektur ist in der Datei Software_arch.drawio zu finden .
Dies kann mittels der ['Draw.io Integration' Extension](https://marketplace.visualstudio.com/items?itemName=hediet.vscode-drawio) innerhalb von VSCode betrachtet werden. Es gibt zwei Versionen der Architektur für den Plan/-Ist vergleich

### Projektplan
Der Projektplan ist unter https://github.com/users/yaba-source/projects/1/views/4

Es gab folgende Änderungen:
- Die Einarbeitung in den Machine Vision Part wurde um eine Woche gekürzt um die ROS Infrasruktur zu erstellen. 
- Der Algin und Drive Action Server wurde gleichzeitig entwickelt da keine Überscheidung der Schnittstellen stattgefunden hat. Diese wurden dann nacheinander Integriert.
- Der Follow wurde vor den Weihnachtsferien entwickelt und nach während der Ferien integriert. Hier waren wir vor dem Plan da Kapazitäten vorhanden waren. 

### Code 
Ist im Git zufinden oder im zugesendeten Ordner.
[GitHub Link](https://github.com/yaba-source/canalchecker)

### Komponenten
- **ActionServerHandler**: Zentrale Koordination, Aruco-Trigger-System
- **Action Server**: DriveActionServer, AlignActionServer, FollowActionServer
- **State Machines**: DriveLogic, AlignLogic, FollowLogic
- **Machine Vision**: ArucoMarkerDetector (Markererkennung)
- **Pub/Sub System**: Topics für Aruco-Daten, Geschwindigkeit, Zielabstand

### Randbedingungen für das System
- Gute Beleuchtung und Sichtbarkeit der Marker 
- Die Marker Größe entspricht der in der Detector funktion angegeben
- WLAN Verbindung zum Roboter
- Aufbau entspticht dem Testaufbau
- Marker in Sichtweite für Kamera
- Oberfläche für den Roboter befahrbar
---

## ActionServerHandler

### Zweck
Der **ActionServerHandler** ist die zentrale Koordinationslayer, die:
- Alle Action Clients verwaltet (Align, Drive, Follow)
- Aruco-Markererkennung überwacht
- Automatische Modusübergänge steuert
- Prioritäten zwischen Actions verwaltet

### Architektur

Der ActionServerHandler verwaltet folgende Attribute:

- _aruco_id: aktuelle erkannte Marker-ID
- _target_aruco_id: 69 (Zielmarker für Follow-Trigger)
- _follow_triggered: Flag zur Vermeidung mehrfacher Trigger
- actionserver_align: Client für Align Action
- actionserver_drive: Client für Drive Action
- actionserver_follow: Client für Follow Action
- current_action: aktuelle Running-Action
- _goal_handle: Handle der aktiven Action

### Funktionen

#### 1. Initialisierung

Der ActionServerHandler abonniert zunächst das Topic 'ArucoDetection', um Marker-Informationen zu empfangen. Danach werden die Action Clients für 'Align', 'Drive' und 'Follow' erstellt. Nach der Initialisierung wird die Align-Action gestartet, gefolgt von Drive.

#### 2. Aruco-Trigger-System

Der `aruco_callback` wird kontinuierlich aufgerufen, wenn eine neue ArucoDetection-Nachricht empfangen wird. Die erkannte Marker-ID wird thread-safe gespeichert. Der Callback läuft parallel zu den laufenden Action Servern (Align oder Drive). Wenn Marker 69 erkannt wird und das `_follow_triggered`-Flag noch nicht gesetzt ist, wird das Flag auf `True` gesetzt und der Follow-Modus ausgelöst.

Wichtig: Das `_follow_triggered`-Flag verhindert, dass Follow mehrfach ausgelöst wird, wenn Marker 69 kontinuierlich erkannt wird. Es wird erst zurückgesetzt, wenn eine neue Mission startet.

#### 3. Follow-Modus Aktivierung

Wenn der Follow-Modus ausgelöst wird und eine andere Action bereits aktiv ist, wird diese abgebrochen. Die _cancel_current_action Methode sendet einen Cancel-Request an die aktive Action. Nach erfolgreichem Cancel wird ein Callback aufgerufen, der die Follow-Action startet. Falls keine Action aktiv ist, wird Follow direkt gestartet.

#### 4. Cancel & Übergangsverwaltung

Die _cancel_current_action Methode prüft, ob ein Goal Handle vorhanden ist und sendet asynchron einen Cancel-Request. Der _on_cancel_done Callback wird aufgerufen, wenn der Cancel-Request verarbeitet wurde. Nach einer kurzen Wartezeit von einer Sekunde wird die Follow-Action gestartet, um sicherzustellen, dass die abgebrochene Action vollständig beendet ist.

#### 5. Goal-Callback-System

Die send_goal Methode wird aufgerufen mit einem Action-Type und einer Goal-Nachricht. Sie wartet auf den Action Server und sendet dann das Goal asynchron mit einem Feedback-Callback. Der goal_response_callback wird aufgerufen, wenn der Server das Goal akzeptiert oder ablehnt. Bei Akzeptanz wird das Goal-Handle gespeichert und auf das Result-Feedback gewartet. Der get_result_callback wird aufgerufen, wenn die Action beendet ist. Er liest die Result-Response aus und prüft den Status, um dann die nächste Action zu bestimmen.

### Thread Safety
- **Aruco-ID Zugriff**: Geschützt durch `_aruco_lock`
- **Follow-Trigger**: Geschützt durch `_follow_triggered` Flag
- **Current Action**: Atomare Zuweisung (Strings sind thread-safe in Python)

---

## Action Server

Drei spezialisierte Action Server, die jeweils eine Phase des Missions-Ablaufs durchführen:

### DriveActionServer

**Zweck**: Fahrt zu einem Zielabstand

```python
class DriveActionServer(Node):
    Topics:
    - Input: /aruco_detections (Marker-Info)
    - Input: /max_speed (dynamische Geschwindigkeitsvorgabe)
    - Output: /cmd_vel (Roboter-Steuerbefehle)
    
    State Machine: DriveLogic

    Eigenschaften:
    - Winkelregelung
    - Feedback: Distanz zu Aruco-ID 0
```

**Ablauf**:
1. Warte auf Aruco-Detektion
2. Verwende Abstandsregelung um zu Zielposition zu fahren
3. Bei Erreichen: Action beendet

**Geschwindigkeit**: Thread-safe aus Topic `/max_speed` gelesen

### AlignActionServer

**Zweck**: Roboter ausrichten

```python
class AlignActionServer(Node):
    Topics:
    - Input: /aruco_detections (Winkel-Info)
    - Input: /max_speed
    - Output: /cmd_vel
    
    State Machine: AlignLogic
    
    Eigenschaften:
    - Timeout: 60 Sekunden
    - Feedback: aktuelle Winkelabweichung
```

**Ablauf**:
1. Erkenne Marker-Winkel aus Aruco-Detektion
2. Verwende P-Regler um Roboter auszurichten
3. Bei Ausrichtung abgeschlossen: Action beendet

### FollowActionServer

**Zweck**: Folge einem Zielmarker

```python
class FollowActionServer(Node):
    Topics:
    - Input: /aruco_detections (Marker-Info)
    - Input: /target_distance (Zielabstand aus Goal)
    - Input: /max_speed
    - Output: /cmd_vel
    
    State Machine: FollowLogic
    
    Eigenschaften:
    - Kontinuierliche Verfolgung
    - Abstandsregelung
    - Winkelausrichtung
```

**Ablauf**:
1. Suche Marker 69
2. Wenn gefunden: Folge mit Abstands- & Winkelregelung
3. Bei Marker 0: Follow beendet (erfolgreiche Zielreichung)

---

## State Machines & Logik

### FollowLogic (3-State Maschine)

```
State 10: SUCHE          State 20: FOLGE            State 30: FERTIG
┌─────────────┐         ┌────────────────┐        ┌──────────────┐
│ Warte auf   │ ─────►  │ Folge Marker   │ ─────► │ Follow Done  │
│ Marker 69   │ ID==69  │ mit P-Regler   │ ID==0  │ = True       │
└─────────────┘         └────────────────┘        └──────────────┘
                              ▲                          │
                              │                          │
                              └──────────────────────────┘
                                  Marker verloren
```

### P-Regler Implementierung

#### Winkelregler

Der Winkelregler implementiert eine proportionale Regelung des Winkel-Fehlers. Der Fehler wird mit dem Proportional-Koeffizient KP_ANGULAR = 2 multipliziert. Das Ergebnis wird auf die maximale Drehgeschwindigkeit von 0.05 rad/s begrenzt. Dies führt zu einer sanften, kontrollierten Rotation des Roboters. Das negative Vorzeichen sorgt für korrekte Drehrichtung.

Beispiel: Wenn ein Marker 30 Grad nach rechts erkannt wird, entspricht dies etwa 0.52 Radianten. Die Regelung berechnet 2 * 0.52 = 1.04, was auf 0.05 rad/s begrenzt wird. Der Roboter dreht sich dann langsam nach rechts.

#### Abstandsregler

Der Abstandsregler implementiert eine proportionale Regelung des Abstands-Fehlers. Der Fehler wird als Differenz zwischen aktuellem Abstand und Zielabstand berechnet und mit dem Koeffizient kp_linear = 0.2 multipliziert. Das Ergebnis wird auf die maximale Fahrtgeschwindigkeit von 0.2 m/s begrenzt. Dies führt zu einer sanften, kontrollierten Annäherung oder Rückwärtsfahrt.

Beispiel: Bei Zielabstand 0.5m und aktuellem Abstand 0.7m ergibt sich ein Fehler von 0.2m. Die Regelung berechnet dann 0.2 * 0.5 = 0.04 m/s Vorwärtsfahrt.

### DriveLogic & AlignLogic

Beide folgen ähnlichem Muster mit State Machines, verwenden aber unterschiedliche Ziele:

- **DriveLogic**: Fahrt zu Zielabstand
- **AlignLogic**: Ausrichtung auf 0° Winkel

---

## Machine Vision

### ArUco-Marker Erkennung - Tiefgreifende Erklärung

#### Grundkonzept

ArUco (Augmented Reality University of Cordoba) ist ein Open-Source Bibliothek zur Erkennung von fiduziellen Markern. Ein ArUco-Marker ist ein quadratisches Muster mit einem einzigartigen binären Code, der eine ID enthält. Das System nutzt diese Marker für:

- **Positionsbestimmung**: Genaue 3D-Pose des Markers relativ zur Kamera
- **Orientierungserkennung**: Winkel und Ausrichtung des Markers
- **Zielidentifizierung**: Unterschiedliche Marker-IDs für verschiedene Aufgaben

#### Marker-Typen im System

Das System verwendet zwei verschiedene Marker mit unterschiedlichen Größen und Bedeutungen:

Marker ID 0 (Großer Marker):
- Größe: 175 mm x 175 mm
- Bedeutung: Zielpunkt - markiert das Endziel der Navigation
- Anwendung: Wird erkannt um zu signalisieren, dass die Follow-Mission abgeschlossen ist
- Erkennungsbereich: Weiter entfernt erkennbar wegen größerer Fläche

Marker ID 69 (Kleiner Marker):
- Größe: 75 mm x 75 mm da an Roboter angebracht und so das Nahe auffahren möglich ist
- Bedeutung: Trigger-Marker - startet den Follow-Modus
- Anwendung: Wird erkannt um automatisch von Align zu Follow zu wechseln
- Erkennungsbereich: Näher bei der Kamera, präzisere lokale Verfolgung

Die unterschiedlichen Größen ermöglichen es dem System, verschiedene Aufgaben zu unterscheiden. Der große Marker 0 dient als Endziel, während der kleine Marker 69 als aktives Navigationsziel dient.


### Kameramodell

Das System verwendet eine kalibrierte Kamera mit intrinsischen und extrinsischen Parametern.

#### Intrinsische Kamera-Parameter (Kameramatrix)

Die Kameramatrix beschreibt wie 3D-Punkte in der Welt in 2D-Bildpunkte projiziert werden. Sie enthält folgende kritische Parameter:

- **Brennweite fx, fy**: Gemessen in Pixeln, beschreibt die Vergrößerung. Typische Werte liegen zwischen 300-1000 je nach Kamera-Auflösung und Linse
- **Hauptpunkt (cx, cy)**: Die Bildkoordinate des Kamera-Zentrums. Normalerweise in der Bildmitte, z.B. (320, 240) für ein 640x480 Bild
- Diese Parameter werden einmalig durch Kamerakalibrierung bestimmt und sind spezifisch für das verwendete Kamera-Setup

#### Distortionskoeffizienten

Reale Kameralinsen erzeugen optische Verzerrungen, die korrigiert werden müssen:

- **Radiale Distortion**: Die Linse verzerrt das Bild am Rand (Verzeichnung). Dies wird durch radiale Distortionskoeffizienten (k1, k2, k3) modelliert
- **Tangentiale Distortion**: Unverhältnisse bei der Linsenmontage verursachen zusätzliche Verzerrungen, modelliert durch (p1, p2)
- Diese Koeffizienten werden durch Kalibrierung bestimmt und speichern, wie stark die Linse verzerrt

#### Kalibrierungsprozess

Die Kalibrierung ist ein einmaliger Prozess:

1. Mehrere Bilder eines Schachbrett-Musters aus verschiedenen Positionen werden aufgenommen
2. Das System erkennt die Ecken des Schachbrettes automatisch
3. Basierend auf diesen bekannten Positionen berechnet ein Algorithmus die Kameramatrix und Distortionskoeffizienten
4. Die Ergebnisse werden in einer Kalibrierungsdatei gespeichert (z.B. picam_calib.npz)

#### Anwendung bei Pose-Estimation

Bei der PnP-Berechnung werden die Kalibrierungsparameter verwendet um:

- Die Linsenverzerrung zu korrigieren, bevor die Marker-Eckpunkte verarbeitet werden
- Korrekte Abstands- und Winkelberechnungen durchzuführen
- Genauigkeit auf wenige Millimeter zu reduzieren

Ohne korrekte Kalibrierung würden Pose-Schätzungen um Zentimeter oder mehr abweichen.
#### Fehlerbehandlung und Robustheit

Das System behandelt mehrere Fehlerszenarien:

- **Keine Marker erkannt**: detect_markers gibt False zurück, estimate_pose wird nicht aufgerufen
- **Zu wenige Marker-Eckpunkte**: solvePnP benötigt mindestens 4 Punkte, normalerweise vorhanden
- **Ambiguöse Lösungen**: solvePnP kann mehrere Lösungen geben, wird durch Use_extrinsicGuess gelöst
- **Numerische Instabilität**: Bei sehr großen oder sehr kleinen Abständen können Fehler auftreten

### Performance und Optimierungen

#### Timing

- **Bilderfassungsrate**: 30 Hz (30 Bilder Pro Sekunde für genau Regelung)
- **Marker-Erkennungslatenz**: ~10-20 ms pro Bild (OpenCV ArUco ist optimiert)
- **Pose-Estimations-Latenz**: ~5-10 ms pro Marker (abhängig von Anzahl Marker)
- **Gesamtlatenz**: Typisch ~30-40 ms vom physischen Marker bis zur verfügbaren Pose-Information


## Nachrichtenschnittstellen

### Custom Message: ArucoDetection

Die ArucoDetection Nachricht enthält folgende Felder:

- aruco_id: Die erkannte Marker-ID (oder -1 wenn kein Marker erkannt wurde)
- aruco_distance: Der Abstand zum erkannten Marker in Metern
- aruco_angle: Der Winkel zum erkannten Marker in Grad

Diese Nachricht wird vom CameraNode (ROS2 Node in MachineVision) publiziert und von allen Action Servern sowie dem ActionServerHandler abonniert.

### Topics

| Topic | Msg Type | Direction | Quelle | Beschreibung |
|-------|----------|-----------|--------|-------------|
| `/aruco_detections` | ArucoDetection | Out → In | CameraNode | Erkannte Marker-Info |
| `/max_speed` | Float32 | Out → In | Extern | Maximale Geschwindigkeit [m/s] |
| `/target_distance` | Float32 | Out → In | FollowActionServer | Zielabstand [cm] |
| `/cmd_vel` | Twist | In ← Out | Action Server | Roboter-Steuerbefehle |

---

## Workflow & Ausführung

### Gesamtablauf

```
START
  │
  ├─► CameraNode (kontinuierlich)
  │     └─► ArucoMarkerDetector.detect_markers()
  │         └─► Publish ArucoDetection Topic
  │
  ├─► ActionServerHandler Initialisierung
  │     └─► send_goal('align', Align.Goal())
  │
  ├─► AlignActionServer.execute_callback() [Phase 1]
  │     │
  │     ├─► State Machine: Richte Roboter aus
  │     │
  │     │   [Parallel dazu:]
  │     │   ActionServerHandler.aruco_callback() [kontinuierlich]
  │     │     ├─► Speichere Marker-ID
  │     │     ├─► Ist Marker 69 erkannt?
  │     │     │   ├─► NEIN: Warte auf nächsten Callback
  │     │     │   └─► JA (und noch nicht getriggert):
  │     │     │       └─► _trigger_follow_mode()
  │     │     │           ├─► Abbruch aktuelle Align-Action
  │     │     │           └─► send_goal('follow', Follow.Goal())
  │     │
  │     └─► [Wenn Marker 69 nicht auftaucht:]
  │         └─► Return Align.Result() ────► Align fertig
  │
  ├─► send_goal('drive', Drive.Goal()) ────► [Phase 2 startet, falls Marker 69 nicht erkannt]
  │
  ├─► DriveActionServer.execute_callback() [Phase 2]
  │     │
  │     ├─► State Machine: Fahre zu Zielabstand
  │     │
  │     │   [Parallel dazu:]
  │     │   ActionServerHandler.aruco_callback() [kontinuierlich]
  │     │     ├─► Speichere Marker-ID
  │     │     ├─► Ist Marker 69 erkannt?
  │     │     │   ├─► NEIN: Warte auf nächsten Callback
  │     │     │   └─► JA (und noch nicht getriggert):
  │     │     │       └─► _trigger_follow_mode()
  │     │     │           ├─► Abbruch aktuelle Drive-Action
  │     │     │           └─► send_goal('follow', Follow.Goal())
  │     │
  │     └─► [Wenn Marker 69 nicht auftaucht:]
  │         └─► Return Drive.Result() ────► Drive fertig
  │
  ├─► FollowActionServer.execute_callback() [Phase 3 - nur wenn Marker 69 erkannt]
  │     ├─► Initialisiere FollowStateMachine
  │     ├─► Loop (30 Hz):
  │     │   ├─► Lese Aruco-Daten
  │     │   ├─► Lese Max Speed aus Topic
  │     │   ├─► State Machine ausführen
  │     │   │   ├─► State 10: Suche Marker 69
  │     │   │   ├─► State 20: Folge Marker 69
  │     │   │   │   ├─► Winkelregler (P-Regler)
  │     │   │   │   └─► Abstandsregler (P-Regler)
  │     │   │   └─► State 30: Abgeschlossen (bei Marker 0 erkannt)
  │     │   └─► Publish Twist auf /cmd_vel
  │     └─► Return Follow.Result()
  │
  └─► Mission beendet
```

**Zusammenfassung des Workflows:**
- **Normal-Ablauf**: Align → Drive → [fertig]
- **Mit Trigger während Align**: Align [wird unterbrochen] → Follow
- **Mit Trigger während Drive**: Align → Drive [wird unterbrochen] → Follow





### Konfigurierbare Parameter

FollowLogic Parameter:

- id_to_follow: Zielmarker für die Verfolgung (Standard: 69)
- kp_angular: Proportional-Koeffizient für Winkelregelung (Standard: 1.1)
- kp_linear: Proportional-Koeffizient für Abstandsregelung (Standard: 0.2)
- max_linear_speed: Maximale Fahrtgeschwindigkeit in m/s (Standard: 0.2)
- align_angular_speed: Maximale Drehgeschwindigkeit in rad/s (Standard: 0.05)
- distance_tolerance: Toleranz beim Zielabstand in Metern (Standard: 0.1)
- angle_tolerance: Toleranz beim Zielwinkel in Grad (Standard: 3)

DriveLogic Parameter:

- kp: Proportional-Koeffizient (Standard: 0.5)
- max_speed: Maximale Fahrtgeschwindigkeit (Standard: 0.2)

AlignLogic Parameter:

- kp_angular: Proportional-Koeffizient (Standard: 2.0)
- max_speed: Maximale Drehgeschwindigkeit (Standard: 0.2)
- angle_tolerance: Zieltoleranz in Grad (Standard: 2)

---


## Gründe für Designentscheidungen

**Reglerauswahl**

Überall wo ein Regler benötigt wird, wurde ein P-Regler implementiert. Dieser verwendet den Regelfehler (z.B. den Unterschied zwischen dem Sollwinkel zu ArUco-ID 0 und dem Istwinkel) und verstärkt ihn mit einer konstanten in gegenrichtung. Die einfache implementierung und parametriesierung, typisch für den Reglertyp, war hier ausschlaggebend. Die Reglerkonstante Kp wurde hier mittels praktischen versuchen bestimmt. 

**Architekturauslegung**

Als Grundarchitektur wurde ein Master-Slave konzept verwendet. Hierbei bildet 'ActionServerHandler.py' den Master und ruft die jeweiligen ActionServer auf. Diese variante wurde gewählt, da auf diesem weg immer nur eine Action gleichzeitig ablaufen kann und so konflikte vermieden werden können. Außerdem wird so das **SOLID** Programierprinzip verwirklicht.



**Warum Multithreaded Executer**

Es hat sich heraus gestellt das die rclpy spin once auf dem Raspberry Pie nicht möglich.   

**Warum cancel_current_action**

Um in den Follow Modus zukommen müssen die andere Goals gecancelt werden.

**Warum nicht OpenCV-ArUco-Winkelberechnungsfunktionen**

Die Winkelrückgabe der OpenCV-Funktion hat sich als unzuverlässig und undurchsichtig herausgestellt.


**ReentrantCallbackGroup vs. MutuallyExclusiveCallbackGroup**

**ReentrantCallbackGroup:**
- Mehrere Callbacks können gleichzeitig ausgeführt werden
- Keine Synchronisation zwischen Callbacks
- Alle Callbacks laufen parallel/asynchron
- Risiko: Race Conditions, wenn Callbacks auf die gleiche Variable zugreifen
- Vorteil: Höhere Performance, schnellere Verarbeitung


**MutuallyExclusiveCallbackGroup:**
- Nur ein Callback läuft zur gleichen Zeit
- Andere Callbacks warten, bis der aktuelle fertig ist (Mutex = gegenseitiger Ausschluss)
- Vorteil: Thread-safe, keine Race Conditions
- Nachteil: Langsamer, da Callbacks sich "blockieren"

Da wir nur einen Callback pro Variable haben ist unser Entscheidung auf Reeantrant Callback Group gefallen. 

**Follow-Action-Server in Extra ActionServer Node**

Der Follow Action Server wurde ausgelagert, um die SOLID-Prinzipien einzuhalten. Die Aufgabe würde sich im Drive Action Server auch gut implementieren lassen, ist jedoch aus Sicht der Code-Erweiterbarkeit zum Handling von weiteren Robotern sinnvoller.

**GoalCallback**

Da wir keine Goals haben, welche wir ablehnen, akzeptieren wir alle Goals und geben jedem Goal eine Bestätigung zurück. 

**Getter nur für bestimmte Variablen**

Die Getter wurden nur bei Variablen verwendet, bei denen Locking relevant ist. Dies dient der Vereinfachung bei der  Implementierung.

**State Machine mit match-case statt If-Else**

Durch State Machine mit match-case sind Strukturen und aktuelle Aktionen klar. Auch für das Debugging und die Wartung des Codes bringen sie enorme Vorteile. 

## Auswertung des Gesamtsystems

Das CanalChecker-System ist eine **hierarchische ROS2-Architektur** mit:

1. **Zentraler Koordination** (ActionServerHandler)
2. **Spezialisierte Action Server** für verschiedene Missionsphasen
3. **State Machines** für komplexe Verhaltenablläufe
4. **P-Regelung** für präzise Bewegungskontrolle
5. **Computer Vision** mit ArUco-Markererkennung
6. **Asynchrones Callback-System** für nicht-blockierende Operationen

Der Workflow folgt automatisch: Align → Drive → Follow, wobei der Übergang zu Follow durch die Erkennung von Marker 69 automatisch ausgelöst wird.

Das System funktioniert unter den gegeben Randbedingungen ohne weitere Probleme und hat keine bekannten Bugs.  





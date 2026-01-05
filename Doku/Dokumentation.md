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
Das CanalChecker-System folgt einer **hierarchischen ROS2-Architektur** mit einer zentrale Koordinierung.
Die Architektur ist in der Datei Software_arch.drawio zu finden 



### Komponenten
- **ActionServerHandler**: Zentrale Koordination, Aruco-Trigger-System
- **Action Server**: DriveActionServer, AlignActionServer, FollowActionServer
- **State Machines**: DriveLogic, AlignLogic, FollowLogic
- **Machine Vision**: ArucoMarkerDetector (Markererkennung)
- **Pub/Sub System**: Topics für Aruco-Daten, Geschwindigkeit, Zielabstand

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

### Hauptablauf

#### 1. Initialisierung

Der ActionServerHandler abonniert zunächst das ArucoDetection Topic, um Marker-Informationen zu empfangen. Danach werden die Action Clients für Align, Drive und Follow erstellt. Nach der Initialisierung wird automatisch die Align-Action gestartet.

#### 2. Aruco-Trigger-System

Der aruco_callback wird aufgerufen, wenn eine neue ArucoDetection Nachricht empfangen wird. Die erkannte Marker-ID wird thread-safe gespeichert. Wenn Marker 69 erkannt wird und das _follow_triggered Flag noch nicht gesetzt ist, wird das Flag auf True gesetzt und der Follow-Modus ausgelöst.

Wichtig: Das _follow_triggered-Flag verhindert, dass Follow mehrfach ausgelöst wird, wenn Marker 69 kontinuierlich erkannt wird.

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
│ Marker 69   │ ID==69  │ mit P-Regler │ ID==0  │ = True       │
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

Beispiel: Bei Zielabstand 0.5m und aktuellem Abstand 0.7m ergibt sich ein Fehler von 0.2m. Die Regelung berechnet dann 0.2 * 0.2 = 0.04 m/s Vorwärtsfahrt.

### DriveLogic & AlignLogic

Beide folgen ähnlichem Muster mit State Machines, verwenden aber unterschiedliche Ziele:

- **DriveLogic**: Fahrt zu Zielabstand
- **AlignLogic**: Ausrichtung auf 0° Winkel

---

## Machine Vision

### ArucoMarkerDetector

Zweck: Aruco-Marker erkennen und Pose schätzen

Der ArucoMarkerDetector verwaltet verschiedene Marker-Größen basierend auf der Marker-ID. Der Marker mit ID 0 hat eine Größe von 175mm (großer Marker), während der Marker mit ID 69 eine Größe von 75mm (kleiner Marker) aufweist.

Die detect_markers Methode führt die OpenCV ArUco Detection durch und gibt zurück, ob Marker erkannt wurden. Die estimate_pose Methode führt folgende Schritte durch:

1. Für jeden erkannten Marker wird die korrekte Größe basierend auf der ID ermittelt
2. Es werden 3D-Objektpunkte basierend auf der Markergröße erstellt
3. Die 2D-Bildpunkte des Markers werden extrahiert
4. Das PnP-Problem (Perspective-n-Point) wird gelöst um die Pose zu schätzen
5. Der Abstand und Winkel zum Marker werden berechnet

Output: Die Methode liefert Listen mit Abständen und Winkeln zu den erkannten Markern.

### Kameramodell

Das System verwendet eine kalibrierte Kamera mit folgenden Parametern:

Die Kameramatrix beschreibt die intrinsischen Kamera-Eigenschaften. Sie enthält die Brennweite in x und y Richtung sowie den Hauptpunkt (optische Achse).

Die Distortionskoeffizienten kompensieren optische Verzerrungen durch die Kameralinse, insbesondere Verzeichnungen und tangentiale Verzerrungen. Diese Parameter wurden durch Kamerakalibrierung bestimmt und sind spezifisch für das verwendete Kamera-Setup.

---

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
  ├─► CameraNode
  │     └─► ArucoMarkerDetector.detect_markers()
  │         └─► Publish ArucoDetection Topic
  │
  ├─► ActionServerHandler.aruco_callback()
  │     ├─► Speichere Marker-ID
  │     ├─► Erkannt Marker 69? ──NO──► Warte auf nächsten Callback
  │     └─► YES
  │         └─► _trigger_follow_mode()
  │             ├─► Cancel aktuelle Action (falls aktiv)
  │             └─► send_goal('follow', Follow.Goal())
  │
  ├─► FollowActionServer.execute_callback_fnc()
  │     ├─► Initialisiere FollowStateMachine
  │     ├─► Loop (30 Hz):
  │     │   ├─► Lese Aruco-Daten
  │     │   ├─► Lese Max Speed aus Topic
  │     │   ├─► State Machine ausführen
  │     │   │   ├─► State 10: Suche Marker 69
  │     │   │   ├─► State 20: Folge Marker 69
  │     │   │   │   ├─► Winkelregler (P-Regler)
  │     │   │   │   └─► Abstandsregler (P-Regler)
  │     │   │   └─► State 30: Abgeschlossen
  │     │   └─► Publish Twist auf /cmd_vel
  │     └─► Return Follow.Result()
  │
  └─► Follow beendet → Nächste Mission
```

### Beispielsequenz: Follow Marker 69

**Zeitpunkt 0s**: Marker 69 erkannt (z.B. bei 1.5m Abstand, 30° Winkel)
```
aruco_callback() aufgerufen
  → _aruco_id = 69
  → _follow_triggered = True
  → _trigger_follow_mode()
    → current_action = "align" (wird abgebrochen)
    → cancel_goal_async()
    → (warte auf Cancel-Callback)
    → _start_follow()
      → send_goal('follow', Follow.Goal())
```

**Zeitpunkt 0.5s**: FollowActionServer gestartet
```
FollowStateMachine initialisiert (State 10)
Loop startet (30 Hz = 33ms pro Iteration)
```

**Iteration 1 (t=0.5s)**:
```
Aruco-Daten: id=69, distance=1.5m, angle=30°
State 10: Marker erkannt → Übergang zu State 20
```

**Iteration 2-N (t=0.53s+)**:
```
State 20 (Follow):
  angle_rad = 30° → 0.524 rad
  angular_speed = pcontroller_angular(0.524)
                = -KP * 0.524
                = -2 * 0.524 = -1.048
                → begrenzt auf -0.05 (dreht nach links)
  
  distance_error = 1.5 - 0.5 = 1.0m
  linear_speed = pcontroller_linear(1.0)
               = 0.2 * 1.0 = 0.2 m/s
  
  Publish: Twist(linear.x=0.2, angular.z=-0.05)
```

**Iteration N+1**: Marker auf 15°
```
angular_speed = -0.05 * (15° / 30°) = -0.025
```

**Iteration N+2**: Marker auf 0° (ausgerichtet)
```
angular_speed = -0.0 m/s (korrekt ausgerichtet)
distance = 0.65m (näher gekommen)
linear_speed = 0.2 * (0.65 - 0.5) = 0.03 m/s
```

**Iteration N+3**: Marker auf -5° (leicht zu weit gedreht)
```
angular_speed = -(-0.05 * (5°/30°)) = 0.0083 (korrigiert)
```

**Nach ~30 Iterationen** (1 Sekunde): 
```
Abstand ≈ 0.5m (Zielabstand erreicht)
Winkel ≈ 0° (ausgerichtet)
follow_done bleibt False bis Marker 0 erkannt
```

---

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

## Debugging & Fehlersuche

### Häufige Probleme

1. **Follow wird nicht ausgelöst**
   - Marker 69 wird nicht erkannt → Camera-Setup prüfen
   - Marker 69 wird erkannt aber Follow nicht gestartet → Handler Logs überprüfen

2. **Roboter dreht sich im Kreis**
   - KP_ANGULAR zu hoch → reduzieren auf 1.0
   - Winkelberechnung falsch → ArucoMarkerDetector prüfen

3. **Abstand wird nicht eingehalten**
   - kp_linear zu klein → erhöhen
   - Target_distance wird nicht richtig gelesen → Topic prüfen

4. **Action wird nicht abgebrochen**
   - Cancel-Callback fehlerhaft → ROS2 Action System prüfen
   - Goal Handle ungültig → Timing-Issue?
5. **Debugging Logger**
   - Auslieferungszustand auskommentiert
   - Können einkommentiert werden

### Monitoring

Zur Überwachung des Systems können folgende ROS2-Befehle verwendet werden:

Topic-Überwachung zur Überprüfung der Aruco-Detektionsergebnisse und der Steuerbefehle.

Action-Status Abfrage um die verfügbaren Actions und ihren Zustand zu überprüfen.

Logs anschauen um Debug-Informationen der einzelnen Server zu sehen.

Diese Tools ermöglichen es, die Systemfunktion in Echtzeit zu überwachen und Probleme zu diagnostizieren.

---

## Zusammenfassung

Das CanalChecker-System ist eine **hierarchische ROS2-Architektur** mit:

1. **Zentraler Koordination** (ActionServerHandler)
2. **Spezialisierte Action Server** für verschiedene Missionsphases
3. **State Machines** für komplexe Verhaltensabläufe
4. **PID-Regelung** für präzise Bewegungskontrolle
5. **Computer Vision** mit ArUco-Markererkennung
6. **Asynchrones Callback-System** für non-blocking Operationen

Der Workflow folgt automatisch: Align → Drive → Follow, wobei der Übergang zu Follow durch Erkennung von Marker 69 automatisch ausgelöst wird.

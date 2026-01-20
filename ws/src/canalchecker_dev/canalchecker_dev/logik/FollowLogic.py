# Erstellt von Yannick B. und Marcel K.
import math

KP_ANGULAR = 2

class FollowStateMachine:
    """
    State Machine für die autonome Verfolgung eines Roboters mittels ArUco-Marker-Erkennung.
    
    
    Die Follow-Logik durchläuft folgende Zustände:
    1. **State 10 (Suche)**: Sucht nach dem Zielmarker (ID 69)
    2. **State 20 (Verfolgung)**: Verfolgt den Roboter aktiv mit Abstandsregelung
    3. **State 30 (Abschluss)**: Beendet die Verfolgung
    
    Eingaben (werden von außen gesetzt):
    - `id`: ArUco-Marker ID der erkannten Marker (-1 wenn keine erkannt)
    - `distance`: Distanz zum erkannten Marker in Metern
    - `angle`: Winkelabweichung zum erkannten Marker in Grad
    - `max_speed`: Maximale Geschwindigkeit (Sicherheitsparameter)
    - `target_distance`: Wunschabstand zum Roboter in cm
    
    Ausgaben (werden vom ActionServer gelesen):
    - `linear_speed`: Vorwärtsgeschwindigkeit für den Roboter in m/s
    - `angular_speed`: Drehgeschwindigkeit für den Roboter in rad/s
    - `follow_done`: Flag, ob die Verfolgung erfolgreich abgeschlossen wurde
    
    Attributes:
        state (int): Aktueller Zustand der State Machine
        logger (logging.Logger): Optional Logger für Debugging
        follow_done (bool): Verfolgung erfolgreich abgeschlossen
        kp_angular (float): Proportional-Koeffizient für Drehgeschwindigkeit
        kp_linear (float): Proportional-Koeffizient für Vorwärtsgeschwindigkeit
    """
    def __init__(self, logger=None):
        """
        Initialisiert die FollowStateMachine für die Verfolgung eines Roboters mit ArUco-Markern.
        
        Diese State Machine implementiert einen Zustandsautomaten mit mehreren Zuständen:
        - State 10: Suche nach dem Ziel-Roboter (Marker ID 69)
        - State 20: Verfolgung des gefundenen Roboters mit Abstandsregelung
        - State 30: Abschluss der Verfolgung
        
        Args:
            logger (logging.Logger, optional): Logger-Objekt für Debugging und Informationsausgabe.
                                               Defaults to None.
        
        
        """
        self.state = 10
        self.logger = logger
        self.follow_done = False
        self.error_rad = None
        self.distance = 0.0
        self.id = -1
        self.angle = 0.0  
        self.id_to_follow = 69
        self.kp_angular = 1.1
        self.align_angular_speed = 0.05
        self.angle_tolerance = 3
        self.linear_speed = 0.0   
        self.angular_speed = 0.0
        self.marker_lost_counter = 0
       
        self.distance_to_robot = 0.5  
        self.target_distance = 50.0   
        self.max_speed = 0.2          
        
        self.distance_tolerance = 0.1
        self.kp_linear = 0.2
        self.max_linear_speed = 0.2
        self.robot_found = True

    
    def pcontroller_angular(self, error_rad):
        """
        P-Regler (Proportional Controller) für die Drehgeschwindigkeit.
        
        Berechnet die erforderliche Drehgeschwindigkeit basierend auf dem Winkelabweichung
        zum Zielmarker. Der Regler ist proportional zum Fehler und wird durch die maximale
        zulässige Drehgeschwindigkeit begrenzt.
        
        Funktionsweise:
        - Bei Abweichung nach links (negativer Fehler): Linksdrehung (positive Ausgabe)
        - Bei Abweichung nach rechts (positiver Fehler): Rechtsdrehung (negative Ausgabe)
        - Die Ausgabe wird auf den Bereich [-max_speed, +max_speed] begrenzt
        
        Args:
            error_rad (float): Winkelfehler in Radiant (negativ = links, positiv = rechts).
        
        Returns:
            float: Berechnete Drehgeschwindigkeit in rad/s, begrenzt auf [-align_angular_speed, +align_angular_speed].
        """
        control = KP_ANGULAR * error_rad 
        max_speed = self.align_angular_speed
        if control > max_speed:
            control = max_speed
        elif control < -max_speed:
            control = -max_speed

        return -control
    
    def pcontroller_linear(self, distance_error):
        """
        P-Regler (Proportional Controller) für die Vorwärtsgeschwindigkeit.
        
        Berechnet die erforderliche Vorwärtsgeschwindigkeit basierend auf dem Abstandsfehler
        zum Zielmarker. Der Regler versucht, einen optimalen Abstand zum Roboter zu halten.
        
        Funktionsweise:
        - Bei zu großem Abstand (positiver Fehler): Schneller fahren (positive Ausgabe)
        - Bei zu kleinem Abstand (negativer Fehler): Bremsen (negative Ausgabe)
        - Die Ausgabe wird auf den Bereich [-max_linear_speed, +max_linear_speed] begrenzt
        
        Args:
            distance_error (float): Abstandsfehler in Metern 
                                    (positiv = zu weit weg, negativ = zu nah dran).
        
        Returns:
            float: Berechnete Vorwärtsgeschwindigkeit in m/s, begrenzt auf 
                   [-max_linear_speed, +max_linear_speed].
        """
        control = self.kp_linear * distance_error
        if control > self.max_linear_speed:
            control = self.max_linear_speed
        elif control < -self.max_linear_speed:
            control = -self.max_linear_speed
        return control
    
    def setSpeedPara(self, linear, angular):
        """
        Setzt die aktuellen Geschwindigkeitsparameter für Vorwärts- und Drehbewegung.
        
        Diese Methode aktualisiert die Geschwindigkeitswerte, die an den Roboter
        gesendet werden sollen. Sie wird typischerweise vom ActionServer aufgerufen,
        um die berechneten Geschwindigkeiten vom Regler zu setzen.
        
        Args:
            linear (float): Vorwärtsgeschwindigkeit in m/s (positiv = vorwärts, negativ = rückwärts).
            angular (float): Drehgeschwindigkeit in rad/s (positiv = links, negativ = rechts).
        
        Returns:
            None
        """
        self.linear_speed = linear
        self.angular_speed = angular
    
    def execute(self):
        """
        Führt einen Zyklus der Follow-State-Machine aus.
        
        Diese Methode implementiert den Zustandsautomaten für die Roboter-Verfolgung.
        Sie wird regelmäßig (z.B. 30 Hz) vom FollowActionServer aufgerufen, um die
        Bewegung des Roboters zu steuern.
        
        Zustände:
        
        **State 10 - Suche nach Roboter:**
            - Sucht nach dem ArUco-Marker mit der ID 69 (Zielroboter)
            - Bleibt stehen während der Suche (linear_speed = 0)
            - Übergang zu State 20 wenn Marker 69 gefunden wird
            - Loggt den Fund des Markers
            
        **State 20 - Verfolgung des Roboters:**
            - Verfolgt Marker 69 aktiv
            - Nutzt P-Regler für Drehrichtung (Winkelausrichtung)
            - Nutzt P-Regler für Vorwärtsgeschwindigkeit (Abstandsregelung)
            - Zielabstand ist target_distance (in cm, wird zu Metern konvertiert)
            - Setzt linear_speed auf 0 wenn Roboter zu nah ist (< distance_to_robot)
            - Stoppt bei Marker-Verlust (nicht Marker 69) - versucht erneut zu suchen
            
        **State 30 - Verfolgung beendet:**
            - Wird aktiviert wenn Marker 0 erkannt wird (Ziel erreicht)
            - Setzt follow_done = True
            - Stoppt alle Bewegungen (linear_speed = 0, angular_speed = 0)
        
        Sicherheitsfeatures:
            - Wenn max_speed = 0: Robot stoppt sofort
            - Abstandsbegrenzung: Robot wird gestoppt wenn zu nah am Ziel
            - Konvertiert target_distance von Centimetern zu Metern
            - Loggt jeden Zustandsübergang wenn Logger verfügbar
        
        Side Effects:
            - Aktualisiert self.linear_speed und self.angular_speed
            - Kann self.state ändern
            - Kann self.follow_done auf True setzen
            - Loggt Informationen/Debugnachrichten bei jedem Durchlauf
        
        Returns:
            None
        """
        if self.max_speed == 0.0:
            self.linear_speed = 0.0
            self.angular_speed = 0.0
            return
        
        self.distance_to_robot = self.target_distance / 100.0
        
        if self.distance < self.distance_to_robot:
            self.linear_speed = 0.0
            return
        match self.state:
            case 10:  
                #if self.logger:
                    # self.logger.info("State 10: Suche nach Roboter")
                if self.id == self.id_to_follow:
                        self.state = 20
                    #if self.logger:
                    #  self.logger.info(f"Marker ID {self.id_to_follow} gefunden bei {self.distance:.2f}m, starte Alignment")
                else:
                    self.robot_found = False
                    
            case 20:
                #if self.logger:
                    # self.logger.info("State 20: Folge Roboter")
                if self.id == self.id_to_follow:
                    self.robot_found = True
                    self.marker_lost_counter = 0

                    angle_rad = math.radians(self.angle)
                    self.angular_speed = self.pcontroller_angular(angle_rad)
                    
         
                    distance_error = self.distance - self.distance_to_robot
                    self.linear_speed = self.pcontroller_linear(distance_error)

                else:
                   # if self.logger:
                    #    self.logger.info("Roboter verloren, suche erneut.")
                    self.linear_speed = 0.0
            case 30:
                #if self.logger:
                   # self.logger.info("State 30: Kein Roboter vorhanden, fahre mit Align fort")
                self.follow_done = True
                self.linear_speed = 0.0
                self.angular_speed = 0.0
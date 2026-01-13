"""Follow Action Server für autonome Roboter-Verfolgung mittels ArUco-Markern.

Dieses Modul implementiert einen ROS2 Action Server, der einen anderen Roboter
basierend auf ArUco-Marker-Erkennung verfolgt. Es koordiniert Sensoreingaben,
Steuerladelogik und Roboterbefehle in einer Multi-threaded Umgebung.

Hauptkomponenten:
    - FollowActionServer: ROS2 Action Server mit ArUco-Verfolgungslogik
    - follow: Action für Verfolgungsbefehle
    - Threads: Multi-threaded Ausführung für gleichzeitige Sensor-Callbacks

Dependencies:
    - rclpy: ROS2 Python-Bibliothek
    - canalchecker_interface: Benutzerdefinierte ROS2 Nachrichten und Actions
    - canalchecker_dev.logik.FollowLogic: Follow-State-Machine Logik
"""
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
import time
from rclpy.duration import Duration
from canalchecker_interface.action import Follow
from canalchecker_interface.msg import ArucoDetection
from canalchecker_dev.logik.FollowLogic import FollowStateMachine
from std_msgs.msg import Float32
import threading
from geometry_msgs.msg import Twist


class FollowActionServer(Node):
    """
    ROS2 Action Server für die Verfolgung eines Roboters mit ArUco-Markern.
    
    Diese Klasse implementiert einen ROS2 Action Server, der einen anderen Roboter
    basierend auf ArUco-Marker-Erkennung verfolgt. Sie fungiert als Schnittstelle
    zwischen den Sensoreinputs (ArUco-Detektion, Zielabstand, Geschwindigkeit)
    und der FollowStateMachine-Logik.
    
    Inputs (ROS2 Topics):
        - `/aruco_detections` (ArucoDetection): ArUco-Marker Erkennungsdaten
        - `/target_distance` (Float32): Zielabstand zum Roboter in cm
        - `/max_speed` (Float32): Maximale Robotergeschwindigkeit in m/s
    
    Outputs (ROS2 Topics):
        - `/cmd_vel` (Twist): Bewegungsbefehle für den Roboter
    
    ROS2 Actions:
        - `follow`: Action Server für Verfolgungsbefehle
    
    Thread-Sicherheit:
        - Alle gemeinsamen Daten sind durch Locks geschützt
        - Separate Locks für ArUco-Daten, Zielabstand und Geschwindigkeit
    """
    def __init__(self):
        """
        Initialisiert den FollowActionServer mit allen notwendigen ROS2 Komponenten.
        
        Erstellt:
        - ROS2 Node mit Namen 'follow_action_server'
        - Action Server für Follow-Aktionen
        - Publisher für Bewegungsbefehle (/cmd_vel)
        - Subscriber für Aruco-Detektionen (/aruco_detections)
        - Subscriber für Zielabstand (/target_distance)
        - Subscriber für maximale Geschwindigkeit (/max_speed)
        
        Initialisiert Thread-Locks für sichere Multi-Threading Zugriffe:
        - _aruco_lock: Schützt ArUco-Erkennungsdaten
        - _target_distance_lock: Schützt Zielabstand
        - _max_speed_lock: Schützt Geschwindigkeitsparameter
        - _goal_lock: Schützt Goal-Handle
        
        Standardwerte:
        - target_distance: 50cm
        - max_speed: 0.15 m/s
        """
        super().__init__('follow_action_server')


        self._goal_lock = threading.Lock()
        self._goal_handle = None
        
        # Aruco Detection Daten
        self._aruco_id = -1
        self._aruco_distance = 0.0
        self._aruco_angle = 0.0
        self._aruco_lock = threading.Lock()
        
        # Ziel-Distanz: Default 50cm, wird überschrieben sobald Topic empfängt
        self._target_distance = 50.0  # Default: 50cm = 0.5m
        self._target_distance_lock = threading.Lock()
        
        # Max-Geschwindigkeit
        self._max_speed = 0.15
        self._max_speed_lock = threading.Lock()
        
        # Publisher für Roboter-Bewegung
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Aruco Detection Subscription
        self.sub_aruco = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.aruco_callback_fnc,
            10)
        
        # Target Distance Subscription
        self.sub_target_distance = self.create_subscription(
            Float32,
            '/target_distance',
            self.target_distance_callback,
            10)
        
        # Max Speed Subscription
        self.sub_max_speed = self.create_subscription(
            Float32,
            '/max_speed',
            self.max_speed_callback,
            10)
        
        # Action Server
        self._action_server = ActionServer(
            self,
            Follow,
            'follow',
            execute_callback=self.execute_callback_fnc,
            goal_callback=self.goal_callback_fnc,
            cancel_callback=self.cancel_callback_fnc,
            callback_group=ReentrantCallbackGroup()
        )
        
        # self.get_logger().info('Follow Action Server initialized')


    def goal_callback_fnc(self, goal_request):
        """
        Callback-Funktion für eingehende Follow-Action Goals.
        
        Diese Funktion wird vom ROS2 Action Server aufgerufen, wenn ein neues
        Follow-Ziel empfangen wird. Sie akzeptiert alle eingehenden Ziele.
        
        Args:
            goal_request: Das eingehende Ziel mit Parametern wie target_distance und max_speed
        
        Returns:
            GoalResponse.ACCEPT: Akzeptiert das Ziel immer
        """
        # self.get_logger().info(
        #     f'Follow goal EMPFANGEN: target_distance={goal_request.target_distance}cm, '
        #     f'max_speed={goal_request.max_speed}m/s'
        # )
        
        return GoalResponse.ACCEPT


    def cancel_callback_fnc(self, goal_handle):
        """
        Callback-Funktion für Stornierungsanfragen von Follow-Aktionen.
        
        Diese Funktion wird vom ROS2 Action Server aufgerufen, wenn ein laufendes
        Follow-Ziel storniert werden soll. Sie akzeptiert alle Stornierungsanfragen.
        
        Args:
            goal_handle: Das Handle des zu stornierenden Ziels
        
        Returns:
            CancelResponse.ACCEPT: Akzeptiert die Stornierung immer
        """
        return CancelResponse.ACCEPT
    
    def aruco_callback_fnc(self, msg: ArucoDetection):
        """
        Callback für ArUco-Marker Erkennungsdaten.
        
        Diese Funktion wird aufgerufen, wenn neue ArUco-Daten vom Erkennungsmodul
        auf dem Topic `/aruco_detections` ankommen. Sie speichert die Daten
        thread-sicher in den entsprechenden Instanzvariablen.
        
        Args:
            msg (ArucoDetection): Nachricht mit ArUco-Erkennungsdaten
                - aruco_id: ID des erkannten Markers (-1 wenn nicht erkannt)
                - aruco_distance: Distanz zum Marker in Metern
                - aruco_angle: Winkelabweichung zum Marker in Grad
        
        Returns:
            None
        """
        with self._aruco_lock:
            self._aruco_id = msg.aruco_id
            self._aruco_distance = msg.aruco_distance
            self._aruco_angle = msg.aruco_angle


    def target_distance_callback(self, msg: Float32):
        """
        Callback für dynamische Änderung des Zielabstands.
        
        Diese Funktion wird aufgerufen, wenn auf dem Topic `/target_distance` eine neue
        Zielabstand-Nachricht ankommt. Dies ermöglicht die Änderung des gewünschten
        Abstands zum Roboter zur Laufzeit.
        
        Args:
            msg (Float32): Neue Zielabstand in Zentimetern
        
        Returns:
            None
        """
        with self._target_distance_lock:
            self._target_distance = msg.data
        # self.get_logger().info(f"Neue Ziel-Distanz: {msg.data:.1f} cm")


    def max_speed_callback(self, msg: Float32):
        """
        Callback für dynamische Änderung der maximalen Geschwindigkeit.
        
        Diese Funktion wird aufgerufen, wenn auf dem Topic `/max_speed` eine neue
        Geschwindigkeitsbegrenzung ankommt. Sie validiert die Eingabe und begrenzt
        sie auf den Bereich [0.0, 0.2] m/s aus Sicherheitsgründen.
        
        Sicherheitsgrenzen:
        - Minimum: 0.0 m/s (Robot stoppt)
        - Maximum: 0.2 m/s (Maximale sichere Geschwindigkeit)
        
        Args:
            msg (Float32): Neue maximale Geschwindigkeit in m/s
        
        Returns:
            None
        """
        speed = msg.data
        if speed < 0.0:
            speed = 0.0
        elif speed > 0.2:
            speed = 0.2
        
        with self._max_speed_lock:
            self._max_speed = speed
        # self.get_logger().info(f"Neue Max-Geschwindigkeit: {speed:.3f} m/s")


    def get_target_distance(self):
        """
        Gibt den aktuellen Zielabstand thread-sicher zurück.
        
        Verwendet einen Lock, um sicherzustellen, dass der Zielabstand nicht während
        eines Zugriffs durch einen anderen Thread verändert wird.
        
        Returns:
            float: Aktueller Zielabstand in Zentimetern
        """
        with self._target_distance_lock:
            return self._target_distance


    def get_max_speed(self):
        """
        Gibt die aktuelle maximale Geschwindigkeit thread-sicher zurück.
        
        Verwendet einen Lock, um sicherzustellen, dass die Geschwindigkeit nicht während
        eines Zugriffs durch einen anderen Thread verändert wird.
        
        Returns:
            float: Aktuelle maximale Geschwindigkeit in m/s (im Bereich [0.0, 0.2])
        """
        with self._max_speed_lock:
            return self._max_speed
    
    def execute_callback_fnc(self, goal_handle):
        """
        Hauptausführungs-Callback für Follow-Aktionen.
        
        Diese Funktion ist die Hauptschleife, die vom ROS2 Action Server aufgerufen wird
        wenn eine Follow-Aktion gestartet wird. Sie koordiniert die Sensoreingaben
        (ArUco-Detektion, Zielabstand, Geschwindigkeit) mit der FollowStateMachine
        und sendet Bewegungsbefehle an den Roboter.
        
        Funktionsweise:
        1. Erstellt eine neue FollowStateMachine Instanz
        2. Lädt Parameter (Zielabstand, max_speed)
        3. Wartet in einer Schleife (30 Hz) auf neue Daten
        4. Kopiert aktuelle Sensordaten in die State Machine
        5. Führt einen Zyklus der State Machine aus
        6. Sendet berechnete Bewegungsbefehle an `/cmd_vel`
        7. Veröffentlicht Feedback mit aktuellem Abstand
        8. Behandelt spezielle Marker-IDs:
           - ID 69: Normaler Verfolgungsmodus
           - ID 0: Ziel erreicht → Aktion erfolgreich
           - ID -1: Kein Marker für > 20s → Aktion abgebrochen
        
        Sicherheitsfeatures:
        - Prüfung auf Cancel-Anfragen bei jedem Durchlauf
        - Timeout von 20 Sekunden bei Marker-Verlust
        - Stoppt Robot vor Beendigung
        - Thread-sichere Datenzugriffe mit Locks
        
        Args:
            goal_handle: ROS2 Action Goal Handle für Feedback und Ergebnisse
        
        Returns:
            Follow.Result: Ergebnisobjekt mit reached=(True/False)
        """
        # self.get_logger().info('★★★ EXECUTE_CALLBACK gestartet!')


        state_machine = FollowStateMachine(logger=self.get_logger())
        
        state_machine.target_distance = self.get_target_distance()
        state_machine.max_speed = self.get_max_speed()
        
        rate = self.create_rate(30)
        aruco_last_seen = time.time()
        no_marker_start = None
        
        while rclpy.ok() and not state_machine.follow_done:
            
            if goal_handle.is_cancel_requested:
                # self.get_logger().info('Cancel empfangen - beende Aktion')
                self._stop_robot()
                goal_handle.canceled()
                return Follow.Result(reached=False)
    
            
            with self._aruco_lock:
                state_machine.id = self._aruco_id
                state_machine.distance = self._aruco_distance
                state_machine.angle = self._aruco_angle
            
            if state_machine.id == 69:
                aruco_last_seen = time.time()
                no_marker_start = None
                
                state_machine.target_distance = self.get_target_distance()
                state_machine.max_speed = self.get_max_speed()
                state_machine.execute()
                
                cmd = Twist()
                cmd.linear.x = state_machine.linear_speed
                cmd.angular.z = state_machine.angular_speed
                self.publisher.publish(cmd)
                
            elif state_machine.id == 0:
                # self.get_logger().info("Marker 0 erkannt → Follow beendet")
                state_machine.follow_done = True
                break
                
            elif state_machine.id == -1:
                if no_marker_start is None:
                    no_marker_start = time.time()
                
                time_elapsed = time.time() - no_marker_start
                self._stop_robot()
                
                if time_elapsed > 20.0:
                    # self.get_logger().error("20 Sekunden ohne Marker → Abbruch")
                    goal_handle.abort()
                    return Follow.Result(reached=False)
            
            else:
                aruco_last_seen = time.time()
                no_marker_start = None
            
            feedback = Follow.Feedback()
            feedback.current_distance = self._aruco_distance
            goal_handle.publish_feedback(feedback)
            
            rate.sleep()
        
        self._stop_robot()


        if state_machine.follow_done:
            goal_handle.succeed()
            # self.get_logger().info('Follow action succeeded')
            return Follow.Result(reached=True)
        else:
            return Follow.Result(reached=False)


    def _stop_robot(self):
        """
        Sendet einen Stopp-Befehl an den Roboter.
        
        Setzt beide Geschwindigkeitskomponenten auf Null und sendet einen Twist-Befehl
        über den `/cmd_vel` Publisher. Dies führt zu sofortiger Stillstand des Roboters.
        
        Bewegungskomponenten:
        - linear.x: 0.0 m/s (keine Vorwärtsbewegung)
        - angular.z: 0.0 rad/s (keine Drehbewegung)
        
        Returns:
            None
        """
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.publisher.publish(stop_cmd)



def main():
    """
    Einstiegspunkt für den Follow Action Server.
    
    Diese Funktion initialisiert ROS2, erstellt eine FollowActionServer Instanz,
    und startet die Event-Loop mit einem MultiThreadedExecutor für gleichzeitige
    Callback-Ausführung.
    
    Der MultiThreadedExecutor ermöglicht:
    - Gleichzeitige Verarbeitung von Sensor-Callbacks
    - Nicht-blockierende Action-Ausführung
    - Optimierte CPU-Auslastung
    
    Returns:
        None
    """
    rclpy.init()
    try:
        follow_action_server = FollowActionServer()
        multithread_executor = MultiThreadedExecutor()
        rclpy.spin(follow_action_server, executor=multithread_executor)
    finally:
        rclpy.shutdown()



if __name__ == '__main__':
    main()

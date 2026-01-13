"""Drive Action Server für Roboter-Navigation mittels ArUco-Markern.

Dieser ROS2 Action Server steuert einen Roboter zur Navigation an ArUco-Marken.
Er verwaltet Sensoreingaben, Bewegungslogik und Fahrbefehle.
"""
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from canalchecker_interface.action import Drive
from canalchecker_interface.msg import ArucoDetection
from canalchecker_dev.logik.DriveLogic import DriveStateMachine
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import threading

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

NO_ROBOT_TIMEOUT = 10

class DriveActionServer(Node):
    """
    ROS2 Action Server für Roboter-Navigation mit ArUco-Marker-Erkennung.
    
    Verwaltet Sensoreingaben (ArUco, Odometrie, Geschwindigkeit) und steuert
    den Roboter über die DriveStateMachine. Timeout nach 10s Marker-Verlust.
    
    Topics:
        - Subscribe: /aruco_detections, /max_speed, /odom
        - Publish: /cmd_vel
    """
    def __init__(self):
        """
        Initialisiert den Drive Action Server mit ROS2-Komponenten und Locks.
        
        Standardwerte: max_speed=0.1 m/s, kein Marker erkannt (id=-1)
        """
        
        self.action_server = ActionServer(
            self,
            Drive,
            'drive',
            execute_callback=self.execute_callback_fnc,
            goal_callback=self.goal_callback_fnc,
            cancel_callback=self.cancel_callback_fnc,
            callback_group=ReentrantCallbackGroup()
        )
        
        self.goal_handle = None

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback_fnc,
            10
        )

        self.sub_max_speed = self.create_subscription(
            Float32,
            '/max_speed',
            self.max_speed_callback_fnc,
            10
        )

        self.sub_aruco = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.aruco_callback_fnc,
            10
        )
        self.linear_speed = 0.0  
        self.angular_speed = 0.0  
        self.max_speed = 0.1
        self.max_speed_lock = threading.Lock()

        self.aruco_dist = 0.0
        self.aruco_angle = 0.0
        self.aruco_id = -1
        self.aruco_lock = threading.Lock()

        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0

        self.no_aruco_start_time = None
        
        # self.get_logger().info('Drive Action Server initialized')

    def goal_callback_fnc(self, goal_request):
        """
        Akzeptiert ein neues Drive-Ziel und extrahiert max_speed.
        
        Returns:
            GoalResponse.ACCEPT
        """
        with self.max_speed_lock:
            self.max_speed = goal_request.max_speed
        return GoalResponse.ACCEPT

    def cancel_callback_fnc(self, goal_handle):
        """
        Akzeptiert Stornierungsanfragen.
        
        Returns:
            CancelResponse.ACCEPT
        """
        return CancelResponse.ACCEPT

    def aruco_callback_fnc(self, msg: ArucoDetection):
        """
        Speichert ArUco-Erkennungsdaten (ID, Abstand, Winkel) thread-sicher.
        """
        self.aruco_dist = msg.aruco_distance
        self.aruco_angle = msg.aruco_angle
        self.aruco_id = msg.aruco_id

    def listener_callback_fnc(self, msg: Odometry):
        """
        Placeholder für Odometrie-Daten. Wird derzeit nicht verwendet.
        """

    def max_speed_callback_fnc(self, msg: Float32):
        """
        Aktualisiert max_speed mit Validierung auf Bereich [0.0, 0.2] m/s.
        """

        if speed < 0.0:
            speed = 0.0
            # self.get_logger().warn("Geschwindigkeit < 0! Setze auf 0.0 m/s")
        elif speed > 0.2:
            speed = 0.2
            # self.get_logger().warn("Geschwindigkeit > 0.2! Begrenze auf 0.2 m/s")
        
        with self.max_speed_lock:
            self.max_speed = speed

    def execute_callback_fnc(self, goal_handle):
        """
        Hauptausführungs-Callback für Drive-Aktionen.
        
        - Erzeugt DriveStateMachine und lädt Parameter
        - Loop (30 Hz): Kopiert Sensordaten, führt State-Machine aus, sendet Befehle
        - Behandelt Marker-Verlust mit 10s Timeout
        - Veröffentlicht Feedback mit Abstand
        - Stoppt Robot bei Abschluss oder Abbruch
        
        Returns:
            Drive.Result mit reached=True/False
        """
        self.goal_handle = goal_handle
        
        # OHNE logger Parameter
        state = DriveStateMachine()
        
        # Max Speed setzen
        with self.max_speed_lock:
            state.max_linear_speed = self.max_speed
        
        rate = self.create_rate(30)
        
        while rclpy.ok() and not state.drive_complete:
            # ERSTE Prüfung: Cancel
            if goal_handle.is_cancel_requested:
                # self.get_logger().info('Cancel empfangen - beende Drive')
                self.stop_robot()
                goal_handle.canceled()
                return Drive.Result(reached=False)
            
            # Aruco Daten übertragen
            with self.aruco_lock:
                state.id = self.aruco_id
                state.angle = self.aruco_angle
                state.distance = self.aruco_dist

            # Max Speed aktualisieren
            with self.max_speed_lock:
                state.max_linear_speed = self.max_speed
            
            # Prüfung ob Aruco vorhanden
            if self.aruco_id == -1:
                # self.get_logger().warn('Kein Aruco gefunden - stoppe')
                self.stop_robot()
                if self.no_aruco_start_time is None:
                    self.no_aruco_start_time = self.get_clock().now()
                else:
                    elapsed = (self.get_clock().now() - self.no_aruco_start_time).nanoseconds / 1e9
                    if elapsed > NO_ROBOT_TIMEOUT:
                        self.get_logger().error("ROBOTER IM WEG!!")
            else:
                # State Machine ausführen
                state.execute()
                self.no_aruco_start_time = None
                
                # Bewegungskommando senden
                self.cmd.linear.x = state.linear_speed
                self.cmd.angular.z = state.angular_speed
                self.publisher.publish(self.cmd)
            
            # Feedback senden
            feedback = Drive.Feedback()
            feedback.dist_to_goal = state.distance
            goal_handle.publish_feedback(feedback)
            
            # Rate sleep
            try:
                rate.sleep()
            except:
                pass
        
        # Roboter stoppen
        self.stop_robot()

        # Ergebnis zurückgeben
        if state.drive_complete:
            goal_handle.succeed()
            # self.get_logger().info('Drive ERFOLGREICH abgeschlossen')
            return Drive.Result(reached=True)
        else:
            # self.get_logger().error('Drive FEHLGESCHLAGEN')
            return Drive.Result(reached=False)

    
    def stop_robot(self):
        """
        Setzt alle Geschwindigkeiten auf 0 und stoppt den Roboter sofort.
        """
        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher.publish(self.cmd)


def main():
    """
    Einstiegspunkt: Initialisiert ROS2 und startet Drive Action Server mit MultiThreadedExecutor.
    """
    rclpy.init()
    try:
        drive_action_server = DriveActionServer()
        multithread_executor = MultiThreadedExecutor()
        rclpy.spin(drive_action_server, multithread_executor)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

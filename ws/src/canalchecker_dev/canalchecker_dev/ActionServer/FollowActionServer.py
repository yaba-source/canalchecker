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

ARUC69_CORRECTION_FACTOR=0.479  # Korrekturfaktor, da der Marker 69 um 54% kleiner als marker 0 ist

class FollowActionServer(Node):
    def __init__(self):
        super().__init__('follow_action_server')

        self._goal_lock = threading.Lock()
        self._goal_handle = None
        
        # Aruco Detection Daten
        self._aruco_id = -1
        self._aruco_distance = 0.0
        self._aruco_angle = 0.0
        self._aruco_lock = threading.Lock()
        
        # Ziel-Distanz und Max-Geschwindigkeit aus Topics
        self._target_distance = 50.0  # Standard in cm
        self._max_speed = 0  # Standard in m/s
        self._target_distance_lock = threading.Lock()
        self._max_speed_lock = threading.Lock()
        
        # Publisher für Roboter-Bewegung
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
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
        
        self.get_logger().info('Follow Action Server initialized')

    def goal_callback_fnc(self, goal_request):
        """Akzeptiert Goal und übernimmt Parameter"""
        self.get_logger().info(
            f'Follow goal received: target_distance={goal_request.target_distance}cm, '
            f'max_speed={goal_request.max_speed}m/s'
        )
        
        
        with self._target_distance_lock:
            self._target_distance = goal_request.target_distance
        
        with self._max_speed_lock:
            self._max_speed = goal_request.max_speed
        
        return GoalResponse.ACCEPT

    def cancel_callback_fnc(self, goal_handle):
        self.get_logger().info('Follow goal cancel requested')
        return CancelResponse.ACCEPT
    
    def aruco_callback_fnc(self, msg: ArucoDetection):
        """Callback für Aruco Detection Daten"""
        with self._aruco_lock:
            self._aruco_id = msg.aruco_id
            self._aruco_distance = (msg.aruco_distance)# Aruco detection factor rausgenommen und im Detector angepasst
            self._aruco_angle = msg.aruco_angle

    def target_distance_callback(self, msg: Float32):
        """Callback für dynamische Ziel-Distanz Änderung"""
        with self._target_distance_lock:
            self._target_distance = msg.data
        self.get_logger().info(f"Neue Ziel-Distanz: {msg.data:.1f} cm")

    def max_speed_callback(self, msg: Float32):
        """Callback für dynamische Geschwindigkeits-Änderung"""
        speed = msg.data
        if speed < 0.0:
            speed = 0.0
        elif speed > 0.2:
            speed = 0.2
        
        with self._max_speed_lock:
            self._max_speed = speed
        self.get_logger().info(f"Neue Max-Geschwindigkeit: {speed:.3f} m/s")

    def get_target_distance(self):
        """Thread-safe Zugriff auf Ziel-Distanz"""
        with self._target_distance_lock:
            return self._target_distance

    def get_max_speed(self):
        """Thread-safe Zugriff auf Max-Geschwindigkeit"""
        with self._max_speed_lock:
            return self._max_speed

    def execute_callback_fnc(self, goal_handle):
        """Hauptschleife für Follow Action"""
        self.get_logger().info('Executing follow action')

        
        state_machine = FollowStateMachine(logger=self.get_logger())
        
        # Ziel-Distanz und Max-Geschwindigkeit in State Machine setzen
        state_machine.target_distance = self.get_target_distance()
        state_machine.max_speed = self.get_max_speed()
        
        rate = self.create_rate(30)
        aruco_last_seen = time.time()
        
        while rclpy.ok() and not state_machine.follow_done:
            if state_machine.id == -1:
                time_now = time.time()
                self.get_logger().info("Keine Marker erkannt. Warte 20sek.")
                self._stop_robot()
                if (time_now - aruco_last_seen) > 20:
                    self.get_logger().error("\n69 oder 0 nicht gefunden.\nAnnahme: Sehe anderen Robo von vorn\nBreche ab!")
                    goal_handle.abort()
                    result = Follow.Result()
                    result.reached = False
                    return result
            elif state_machine.id == 0:
                time_now = time.time()
                self.get_logger().info("Erkenne Marker 0. Warte 20sek.")
                self._stop_robot()
                if (time_now - aruco_last_seen) > 20:
                    state_machine.follow_done = True
                    self.get_logger().info("Keinen Robo erkannt. Beende Follow erfolgreich.")
                    continue
            else:
                aruco_last_seen = time.time()
          
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._stop_robot()
                self.get_logger().info('Goal cancelled')
                return Follow.Result(reached=False)
            
            # Aktuelle Aruco Daten in State Machine übertragen mit thread-lock
            with self._aruco_lock:
                state_machine.id = self._aruco_id
                state_machine.distance = self._aruco_distance
                state_machine.angle = self._aruco_angle
            
           
            state_machine.target_distance = self.get_target_distance()
            state_machine.max_speed = self.get_max_speed()
            
            # State Machine ausführen
            state_machine.execute()
            
            # Bewegungskommando senden
            cmd = Twist()
            cmd.linear.x = state_machine.linear_speed
            cmd.angular.z = state_machine.angular_speed
            self.publisher.publish(cmd)

            # Feedback senden
            feedback = Follow.Feedback()
            feedback.current_distance = self._aruco_distance
            goal_handle.publish_feedback(feedback)
            
            rate.sleep()
        
        # Follow abgeschlossen
        self._stop_robot()

        if state_machine.follow_done:
            goal_handle.succeed()
            result = Follow.Result()
            result.reached = True
            self.get_logger().info('Follow action succeeded')
            return result
        else:
            result = Follow.Result()
            result.reached = False
            self.get_logger().info('Follow action failed')
            return result

    def _stop_robot(self):
        """Stoppt den Roboter"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.publisher.publish(stop_cmd)

        
def main():
    rclpy.init()
    try:
        follow_action_server = FollowActionServer()
        multithread_executor = MultiThreadedExecutor()
        rclpy.spin(follow_action_server, executor=multithread_executor)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
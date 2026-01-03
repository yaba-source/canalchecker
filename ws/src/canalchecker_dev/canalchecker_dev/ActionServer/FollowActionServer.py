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
        self._target_distance = 50.0
        self._max_speed = 0
        self._target_distance_lock = threading.Lock()
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
        """Akzeptiert Goal und übernimmt Parameter"""
        # self.get_logger().info(
        #     f'★★★ Follow goal EMPFANGEN: target_distance={goal_request.target_distance}cm, '
        #     f'max_speed={goal_request.max_speed}m/s'
        # )
        
        with self._target_distance_lock:
            self._target_distance = goal_request.target_distance
        
        with self._max_speed_lock:
            self._max_speed = goal_request.max_speed
        
        return GoalResponse.ACCEPT

    def cancel_callback_fnc(self, goal_handle):
        # self.get_logger().info('Follow goal cancel requested')
        return CancelResponse.ACCEPT
    
    def aruco_callback_fnc(self, msg: ArucoDetection):
        """Callback für Aruco Detection Daten"""
        with self._aruco_lock:
            self._aruco_id = msg.aruco_id
            self._aruco_distance = msg.aruco_distance
            self._aruco_angle = msg.aruco_angle

    def target_distance_callback(self, msg: Float32):
        """Callback für dynamische Ziel-Distanz Änderung"""
        with self._target_distance_lock:
            self._target_distance = msg.data
        # self.get_logger().info(f"Neue Ziel-Distanz: {msg.data:.1f} cm")

    def max_speed_callback(self, msg: Float32):
        """Callback für dynamische Geschwindigkeits-Änderung"""
        speed = msg.data
        if speed < 0.0:
            speed = 0.0
        elif speed > 0.2:
            speed = 0.2
        
        with self._max_speed_lock:
            self._max_speed = speed
        # self.get_logger().info(f"Neue Max-Geschwindigkeit: {speed:.3f} m/s")

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
                return ActionType.Result(reached=False)  # Sauberer Return!
    
            
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

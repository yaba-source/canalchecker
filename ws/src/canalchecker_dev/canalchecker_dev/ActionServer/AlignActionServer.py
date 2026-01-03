import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from canalchecker_interface.action import Align
from canalchecker_dev.logik.AlignLogic import AlignStateMachine
from canalchecker_interface.msg import ArucoDetection
from std_msgs.msg import Float32
import math
import time
import threading
from geometry_msgs.msg import Twist



class AlignActionServer(Node):
    def __init__(self):
        super().__init__('align_action_server')
        
        self._goal_lock = threading.Lock()
        self._goal_handle = None
        self._last_yaw = None
        self._aruco_id = -1
        self._aruco_distance = 0.0
        self._aruco_angle = 0.0
        self._aruco_lock = threading.Lock()
        self._max_speed = 0.1
        self._max_speed_lock = threading.Lock()
        
        # Publisher für cmd_vel
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
       
        # Subscriber für Aruco Detection
        self.sub_aruco = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.aruco_callback_fnc,
            10)
        
        # Subscriber für Max Speed
        self.sub_max_speed = self.create_subscription(
            Float32,
            '/max_speed',
            self.max_speed_callback,
            10)
        
        # Action Server
        self.action_server = ActionServer(
            self,
            Align,
            'align',
            execute_callback=self.execute_callback_fnc,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # self.get_logger().info('Align Action Server initialized')

    def goal_callback(self, goal_request):
        """Callback wenn Goal ankommt"""
        # self.get_logger().info('Goal received')
        # self.get_logger().info(f'Goal attributes: {dir(goal_request)}')
        return GoalResponse.ACCEPT
    
    def aruco_callback_fnc(self, msg: ArucoDetection):
        """Callback für Aruco Detection Daten"""
        # self.get_logger().info(f"ArUco empfangen: ID={msg.aruco_id}, dist={msg.aruco_distance}, angle={msg.aruco_angle}")
        with self._aruco_lock:
            self._aruco_id = msg.aruco_id
            self._aruco_distance = msg.aruco_distance
            self._aruco_angle = msg.aruco_angle
     
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
    
    def get_max_speed(self):
        """Thread-safe Zugriff auf Max-Geschwindigkeit"""
        with self._max_speed_lock:
            return self._max_speed

    def cancel_callback(self, cancel_request):
        """Akzeptiert Cancel-Requests vom Client"""
        # self.get_logger().info('Cancel request received - accepting')
        return CancelResponse.ACCEPT  # Wichtig: Explizit akzeptieren
    
    def execute_callback_fnc(self, goal_handle):
        """Hauptschleife für Align Action"""
        # self.get_logger().info('Executing alignment')
        
        state_machine = AlignStateMachine(logger=self.get_logger())
        state_machine.max_speed = self.get_max_speed()
        
        rate = self.create_rate(30)
        timeout = 60.0
        start_time = time.time()

        while rclpy.ok() and not state_machine.align_done:
            # WICHTIG: Cancel ZUERST prüfen
            if goal_handle.is_cancel_requested:
                # self.get_logger().info('Goal wurde gecancelt - beende sauber')
                self._stop_robot()
                goal_handle.canceled()
                return Align.Result(reached=False)
            
            # Timeout
            if time.time() - start_time > timeout:
                # self.get_logger().error('Alignment timeout')
                self._stop_robot()
                goal_handle.abort()
                return Align.Result(reached=False)
            
            # Aruco Daten
            with self._aruco_lock:
                state_machine.id = self._aruco_id
                state_machine.distance = self._aruco_distance
                state_machine.angle = self._aruco_angle
            
            state_machine.max_speed = self.get_max_speed()
            state_machine.execute()
            
            # Bewegung
            cmd = Twist()
            cmd.linear.x = state_machine.linear_speed
            cmd.angular.z = state_machine.angular_speed
            self.publisher.publish(cmd)
            
            # Feedback
            feedback = Align.Feedback()
            feedback.angle_to_goal = state_machine.angle
            goal_handle.publish_feedback(feedback)
            
            # Rate sleep mit Try-Catch
            try:
                rate.sleep()
            except Exception as e:
                # self.get_logger().debug(f'Rate sleep error (ignoriert): {e}')
        
            # Roboter stoppen
                self._stop_robot()
            
        # Ergebnis
        if state_machine.align_done:
            goal_handle.succeed()
            # self.get_logger().info('Alignment succeeded')
            return Align.Result(reached=True)
        else:
            # self.get_logger().warn('Alignment nicht erreicht')
            return Align.Result(reached=False)

          
        
    def _stop_robot(self):
        """Stoppt den Roboter"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.publisher.publish(stop_cmd)


def main():
    rclpy.init()
    try:
        align_action_server = AlignActionServer()
        multithread_executor = MultiThreadedExecutor()
        rclpy.spin(align_action_server, executor=multithread_executor)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
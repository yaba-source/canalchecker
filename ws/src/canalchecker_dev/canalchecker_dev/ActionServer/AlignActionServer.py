import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from canalchecker_interface.action import Align
from canalchecker_dev.logik.AlignLogic import AlignStateMachine
from canalchecker_interface.msg import ArucoDetection
import math
import time
import threading

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def quaternion_to_yaw(q):
    """

    def 
    q: ein Objekt mit x, y, z, w 
    Rückgabe: yaw in Radiant
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


class AlignActionServer(Node):
    def __init__(self):
        super().__init__('align_action_server')
        
        self._goal_lock = threading.Lock()
        self._goal_handle = None
        self._last_yaw = None
        self._aruco_id = None
        self._aruco_distance = 0.0
        self._aruco_angle = 0.0
        self._aruco_lock = threading.Lock()
        
        # Publisher für cmd_vel
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Subscriber für Odometrie
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback_fnc,
            10
        )
        # Subscriber für Aruco Detection
        self.sub_aruco = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.aruco_callback_fnc,
            10)
        # Action Server
        self.action_server = ActionServer(
            self,
            Align,
            'align',
            execute_callback=self.execute_callback_fnc,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback
        )
        
        self.get_logger().info('Align Action Server initialized')

    def listener_callback_fnc(self, msg: Odometry):
        self._last_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        

    def goal_callback(self, goal_request):
        """Callback wenn Goal ankommt - OHNE auf Attribute zuzugreifen"""
        self.get_logger().info('Goal received')
        
        self.get_logger().info(f'Goal attributes: {dir(goal_request)}')
        return GoalResponse.ACCEPT
    def aruco_callback_fnc(self, msg: ArucoDetection):
        """ CustomMessage mit Aruco Detection Daten """
        with self._aruco_lock:
            self._aruco_id = msg.id
            self._aruco_distance = msg.distance
            self._aruco_angle = msg.angle
     

    def execute_callback_fnc(self, goal_handle):
        self.get_logger().info('Executing alignment')
        
        
        if self._last_yaw is None:
            self.get_logger().warning('No odometry data received yet. Cannot align.')
            goal_handle.abort()
            return Align.Result(reached=False)

            
        state_machine = AlignStateMachine(logger=self.get_logger())
        state_machine.id_to_turn = goal_handle.request.marker_id 
        state_machine.set_p_gains(kp_angular=1.5, max_angular_speed=0.5)

        rate = self.create_rate(20)  # 20 Hz
        timeout = 30.0  # 30 Sekunden Timeout
        start_time = time.time()

        while rclpy.ok() and not state_machine.align_done:
            if time.time() - start_time > timeout:
                self.get_logger().error('Alignment timeout')
                goal_handle.abort()
                self._stop_robot()
                return Align.Result(reached=False)
            
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._stop_robot()
                self.get_logger().info('Goal cancelled')
                return Align.Result(reached=False)
            
            with self._aruco_lock:
                state_machine.id = self._aruco_id
                state_machine.distance = self._aruco_distance
                state_machine.angle = self._aruco_angle

            state_machine.execute()
            cmd = Twist()
            cmd.linear.x = state_machine.linear_speed
            cmd.angular.z = state_machine.angular_speed
            self.publisher.publish(cmd)

            feedback = Align.Feedback()
            feedback.angle_to_goal = state_machine.angle
            goal_handle.publish_feedback(feedback)

            rate.sleep()

        self._stop_robot()

        if state_machine.align_done:
            goal_handle.succeed()
            result = Align.Result()
            result.reached = True
            self.get_logger().info('Alignment succeeded')
            return result
        else:
            result = Align.Result()
            result.reached = False
            return result
          
        

    def _stop_robot(self):
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
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from canalchecker_interface.action import Align
import math
import time
import threading

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def quaternion_to_yaw(q):
    """
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

        
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback_fnc,
            10
        )

        
        self.action_server = ActionServer(
            self,
            Align,
            'align',
            execute_callback=self.execute_callback_fnc,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback
        )

    def listener_callback_fnc(self, msg: Odometry):

        self._last_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.get_logger().info(f"Angle: {self._last_yaw}")

    def goal_callback(self, goal_request):

        self.get_logger().info(f'Goal received: target_angle={goal_request.target_angle}')
        return GoalResponse.ACCEPT

    def execute_callback_fnc(self, goal_handle):
     
        self.get_logger().info('Executing alignment to target angle')
        
     
        if self._last_yaw is None:
            self.get_logger().warn('No odometry data received yet. Cannot align.')
            goal_handle.abort()
            return Align.Result(reached=False)

        # Hier kommt deine Align-Logik hin
        # Beispiel: Einfache Schleife die 10x Feedback sendet
        for i in range(10):
            # Prüfen ob Goal noch aktiv ist
            if not goal_handle.is_active:
                break
            
            
            feedback = Align.Feedback()
            feedback.angle_to_goal = float(10 - i)  # Beispielwert
            goal_handle.publish_feedback(feedback)
            
         
            cmd = Twist()
            cmd.angular.z = 0.1  # Beispielwert
            self.publisher.publish(cmd)
            
            time.sleep(0.5)
        
       
        stop_cmd = Twist()
        stop_cmd.angular.z = 0.0
        self.publisher.publish(stop_cmd)
        
        
        if goal_handle.is_active:
            goal_handle.succeed()
            result = Align.Result()
            result.reached = True
            self.get_logger().info('Alignment succeeded')
            return result
        else:
            result = Align.Result()
            result.reached = False
            return result


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
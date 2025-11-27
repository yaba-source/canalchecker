import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer
from rclpy.node import Node
from canalchecker_interface.action import Drive
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, GoalResponse
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

#from .logic import Logic

class DriveActionServer(Node):
    def __init__(self):
        super().__init__('drive_action_server')
        self.action_server = ActionServer(
            self,
            Drive,
            'drive',
            self.execute_callback_fnc #Zuweisung von Phython verwenden  #default verhalten checken
        )
        self.goal_handle = None

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
        Drive,
        'drive',
        execute_callback=self.execute_callback_fnc,
        goal_callback=self._goal_callback_fnc,
        handle_accepted_callback=self._handle_accepted_callback_fnc,
        callback_group=ReentrantCallbackGroup()
        )

   
    def _goal_callback_fnc(self, goal_request):
        return GoalResponse.ACCEPT

    def _handle_accepted_callback_fnc(self, goal_handle):
        self.goal_handle = goal_handle
        goal_handle.execute()


    def listener_callback_fnc(self, msg: Odometry):
       
        
        self.get_logger().info(f"Pos X: {msg.pose.pose.position.x:.3f} " f"Pos Y: {msg.pose.pose.position.y:.3f}")


    def execute_callback_fnc(self, goal_handle): 
        self.get_logger().info('Goal Received! Driving.')
        self.goal_handle = goal_handle
        self.goal_finished = False
        self.goal_result = None
        for i in range(10):
            if not goal_handle.is_active:
                break
            result = Drive.Result()
        result.success = True 
        return result   
      


def main():
    rclpy.init()
    try:
        drive_action_server = DriveActionServer()
        multithread_executer = MultiThreadedExecutor()
        rclpy.spin(drive_action_server,multithread_executer)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
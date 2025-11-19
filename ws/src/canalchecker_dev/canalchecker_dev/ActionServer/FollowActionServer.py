import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from canalchecker_interface.action import Follow
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

#from .logic import Logic

class FollowActionServer(Node):
    def __init__(self):
        super().__init__('follow_action_server')
        self.action_server = ActionServer(
            self,
            Follow,
            'follow',
            self.execute_callback_fnc
        )
        self.goal_handler = None

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

        self.timer = self.create_timer(0.1, self.timer_callback_fnc)


    def timer_callback_fnc(self):
        if self.goal_handler is not None:
            # For-loop später entfernen und mit logik / logikcalls ersetzen
            for i in range(10):
                self.get_logger().info(str(i))
                feedback = Follow.Feedback()
                feedback.dist_to_robot = float(i)
                self.goal_handler.publish_feedback(feedback)
                time.sleep(0.5)
            result = Follow.Result()
            result.success = True
            self.goal_handler.succeed()
            self.goal_finished = True
            self.goal_handler = None
            self.timer.cancel()


    def listener_callback_fnc(self, msg: Odometry):
        """
        Damit der Subscriber ein callback hat der callable() ist.
        Für den FollowActionServer nicht notwendig.
        """
        pass


    def execute_callback_fnc(self, goal_handle):
        self.get_logger().info('Goal Received! Following Robot.')
        self.goal_handler = goal_handle
        self.goal_finished = False
        self.goal_result = None

        while not self.goal_finished:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return self.goal_result


def main():
    rclpy.init()
    try:
        follow_action_server = FollowActionServer()
        rclpy.spin(follow_action_server)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
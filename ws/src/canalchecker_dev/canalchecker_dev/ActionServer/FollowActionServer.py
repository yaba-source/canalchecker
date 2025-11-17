import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from canalchecker_interface.action import Follow

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

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
        pass

    def listener_callback_fnc(self):
        pass

    def execute_callback_fnc(self):
        pass


def main():
    rclpy.init()
    try:
        follow_action_server = FollowActionServer()
        rclpy.spin(follow_action_server)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
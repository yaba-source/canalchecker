import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from canalchecker_interface.action import Drive
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
                self.get_logger().info("Current: ", i)
                feedback = Drive.Feedback()
                feedback.dist_to_goal = float(i)
                self.goal_handler.publish_feedback(feedback)
                time.sleep(0.5)
            result = Drive.Result()
            result.reached
            self.goal_handler.succeed()
            self.goal_finished = True
            self.goal_handler = None
        else:
            self.destroy_timer()


    def listener_callback_fnc(self, msg: Odometry):
        """
        Gibt die Koordinaten X und Y der Odometrie auf Stdout aus, wenn diese sich ändern.
        """
        
        self.get_logger().info(f"Pos X: {msg.pose.pose.position.x:.3f} " f"Pos Y: {msg.pose.pose.position.y:.3f}")


    def execute_callback_fnc(self, goal_handle):
        self.get_logger().info('Goal Received! Driving.')
        self.goal_handler = goal_handle
        self.goal_finished = False
        self.goal_result = None

        while not self.goal_finished:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return self.goal_result


def main():
    rclpy.init()
    try:
        drive_action_server = DriveActionServer()
        rclpy.spin(drive_action_server)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
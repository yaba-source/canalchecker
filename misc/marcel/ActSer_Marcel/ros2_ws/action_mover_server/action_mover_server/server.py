"""

DO NOT USE FOR ROBOT CONTROL.
BASED ON 'RO36_Aufgabe1-3_sol.md' SOLUTION PROVIDED BY PROFESSOR.
USE ONLY AS REFERENCE.

"""


import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion

from action_server_interface.action import GoTo
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import time

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class ActionMoverServer(Node):
    def __init__(self):
        super().__init__('action_mover_server')

        self.last_pose_x = None
        self.last_pose_y = None
        self.last_pose_theta = None

        self.goal_handle = None
        self.action_server = ActionServer(
            self,
            GoTo,
            'go_to',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callbac,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        def goal_callbac(self, goal_request): # = default behavior + message (could be omitted)
            self.get_logger().info('Received goal request with target pose ' + pose_as_string(goal_request.pose))
            return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        if self._goal_handle is not None and self._goal_handle.is_active:
            self.get_logger().info('Replacing active goal with new goal.')
            self._goal_handle.abort()
        self._goal_handle = goal_handle
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancelling move to pose ' +
                            pose_as_string(goal_handle.request.pose))
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing move to pose ' + pose_as_string(goal_handle.request.pose))
        i = 10
        while (i > 0 and goal_handle.is_active and not goal_handle.is_cancel_requested):
            self.do_action_step(self, goal_handle, i)
            i = i-1

        return self.determine_action_result(self, goal_handle, i)

    def do_action_step(self, goal_handle, i):
        feedback_msg = GoTo.Feedback()
        feedback_msg.dist_to_goal = float(i)
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(1)

    def determine_action_result(self, goal_handle, i):
        result = GoTo.Result()
        if goal_handle.is_active and i == 0:
            self.get_logger().info('Move to pose ' + pose_as_string(goal_handle.request.pose) + ' succeeded.')
            goal_handle.succeed()
            result.reached = True
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Move to pose ' + pose_as_string(goal_handle.request.pose) + ' was cancelled.')
        else:
            if goal_handle.is_active():
                goal_handle.abort()
            self.get_logger().info('Move to pose ' + pose_as_string(goal_handle.request.pose) + ' was aborted.')
        return result
    
    def odom_callback(self, msg):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        self.last_pose_theta = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]
        )

def pose_as_string(pose):
    return 'x={}, y={}, theta={}'.format(pose.x, pose.y, pose.theta)

def main():
    rclpy.init()
    try:
        action_mover_server = ActionMoverServer()
        rclpy.spin(action_mover_server)
        action_mover_server.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

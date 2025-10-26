#------------------------------#
#   EXTERNAL LIBRARY IMPORT    #
#------------------------------#

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from geometry_msgs.msg import Twist
from nav_msgs import Odometry
from tf_transformations import euler_from_quaternion

from action_server_interface.action import GoTo

class ActionMoverServer(Node):
    def __init__(self):
        super().__init__('action_mover_server')

        # Last pose in form [x, y, theta]
        self.last_pose = [None, None, None]
        self.goal_handle = None

        self.action_server = ActionServer(
            self,
            GoTo,
            'go_to',
            execute_callback=self.execute_callback_fnc,
            goal_callback=self.goal_callback_fnc,
            handle_accepted_callback=self.handle_accepted_callback_fnc,
            cancel_callback=self.cancel_callback_fnc
        )
        self.msg_stdout("Action Server has been Initialized.", "info")

        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback_fnc,
            10
        )
        self.msg_stdout("Server now Subscribing to topic 'Odom'.", "info")

        self.command_publish = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        self.msg_stdout("Server now Publishing to topic 'cmd_vel'.", "info")
        self.msg_stdout("Server Init Complete. No Problems Detected.", "info")
        self.msg_stdout("Awaiting Client input!", "info")

    
    def execute_callback_fnc(self, goal_handle):
        """
        If a Goal has been accepted and is to be executed, this function is called.
        Implement Robot control functionality here.
        """
        
        self.msg_stdout("Executing move to new Coordinates...", "info")
        i = 10
        # TODO: CONTINUE HERE


    def goal_callback_fnc(self) -> GoalResponse:
        """
        Checks wheter goal is accepted or not. For now, every Goal is accepted.
        """
        
        self.msg_stdout("Received new Coordinates. Goal Accepted", "info")
        return GoalResponse.ACCEPT
    

    def handle_accepted_callback_fnc(self, goal_handle) -> None:
        """
        If a new Goal is accepted, this function is called.
        """
        
        if self.goal_handle is not None and self.goal_handle.is_active:
            self.msg_stdout("\nReplacing active goal with new goal.\n", "info")
            self.goal_handle.abort()
        self.goal_handle = goal_handle
        goal_handle.execute()


    def cancel_callback_fnc(self, goal_handle) -> CancelResponse:
        """
        Movement is cancelled when this function is called.
        """

        self.msg_stdout("\nCancelling Movement!\n", "info")
        return CancelResponse.ACCEPT


    def odom_callback_fnc(self, msg):
        theta = None
        theta = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]
        )
        self.last_pose[0] = msg.pose.pose.position.x
        self.last_pose[1] = msg.pose.pose.position.y
        self.last_pose[2] = theta


    def msg_stdout(self, string: str, severity: str) -> None:
        """
        Uses ROS2's 'get_logger().info()' to print a message to Standard output (terminal).
        
        :param string: The Message that should be print to stdout.
        :param severity: Criticality of the information. Supported: 'warn', 'fatal', 'info', 'err'.
        """

        # Check for correct 'severity argument'
        # If incorrect / unsupported argument it crashes the program
        match severity:
            case 'info':
                self.get_logger().info(string)
            case 'err':
                self.get_logger().error(string)
            case 'warn':
                self.get_logger().warning(string)
            case 'fatal':
                self.get_logger().fatal(string)
            case _:
                self.get_logger().fatal("\nFunction 'msg_stdout' received incorrect / unsupported / malformed data. Ending Program.\n")
                exit()


def main():
    rclpy.init()
    try:
        ams = ActionMoverServer()
        rclpy.spin(ams)
        ams.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
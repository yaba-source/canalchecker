import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_server_interface.action import GoTo

class ActionMoverClient(Node):
    def __init__(self):
        super().__init__('action_mover_client')
        self.action_client = ActionClient(
            self,
            GoTo,
            'go_to'
        )
    
    def send_goal(self, x: float, y: float, theta: float) -> None:
        """
        Sends Goal to Action Server

        :param x: X-Coordinate for Robot to move to
        :param y: Y-Coordinate for Robot to move to
        :param theta: Angle Robot should rotate
        """

        goal_msg = GoTo.Goal()
        goal_msg.pose.x = float(x)
        goal_msg.pose.y = float(y)
        goal_msg.pose.theta = float(theta)

        self.action_client.wait_for_server()

        self.action_client.\
            send_goal_async(goal_msg, feedback_callback=self.feedback_callback_fnc).\
            add_done_callback(self.goal_response_callback_fnc)
    

    def feedback_callback_fnc(self, msg):
        """
        Handles the Feedback. Prints it to stdout.
        """

        feedback = msg.feedback
        self.get_logger().info('Feedback Received: {0}'.format(feedback))


    def goal_response_callback_fnc(self, promise):
        """
        Wheter goal is rejected or accepted.
        """

        goal_handler = promise.result()
        if not goal_handler.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')

        self._get_result_promise = goal_handler.get_result_async()
        self._get_result_promise.add_done_callback(self.get_result_callback_fnc)

    
    def get_result_callback_fnc(self, promise):
        """
        When a mission is finished, prints a message to stdout.
        """

        result = promise.result().result
        self.get_logger().info('Finished!')


def main(args=None):
    rclpy.init(args=args)

    x = float(input("X: "))
    y = float(input("Y: "))
    theta = float(input("theta: "))

    action_client = ActionMoverClient()
    action_client.send_goal(x, y, theta)

    rclpy.spin(action_client)
    rclpy.shutdown()



if __name__ == '__main__':
    main()
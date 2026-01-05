import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_interface.action import GoTo

class GoToActionClient(Node):
    def __init__(self):
        super().__init__('goto_action_client')  
        self._action_client = ActionClient(self, GoTo, 'goto')  
    
    def send_target(self, x, y, th,linear_speed=0.4, angular_speed=0.5):  
        target_msg = GoTo.Goal()
        target_msg.target_pose.x = float(x)
        target_msg.target_pose.y = float(y)
        target_msg.target_pose.theta = float(th)
        target_msg.max_linear_speed = float(linear_speed)     
        target_msg.max_angular_speed = float(angular_speed)
        
        self._action_client.wait_for_server()  
        self._send_goal_promise = self._action_client.send_goal_async(
            target_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_promise.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, promise):
        goal_handler = promise.result()
        if not goal_handler.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_promise = goal_handler.get_result_async()
        self._get_result_promise.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, promise):
        result = promise.result().result
        self.get_logger().info('done: ' + str(result.success))
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))

def main(args=None):
    rclpy.init(args=args)
    try:
        action_client = GoToActionClient()
        
        x = float(input("Ziel X eingeben: "))
        y = float(input("Ziel Y eingeben: "))
        theta = float(input("Ziel Theta eingeben: "))
        linear_speed = input("Linear Geschwindigkeit (Enter für 0.4): ")
        linear_speed = float(linear_speed) if linear_speed else 0.4       
        angular_speed = input("Winkel Geschwindigkeit (Enter für 0.5): ")
        angular_speed = float(angular_speed) if angular_speed else 0.5

        action_client.send_target(x, y, theta,linear_speed,angular_speed)
        rclpy.spin(action_client)
    except ValueError:
        print(" Ungültige Eingabe!")
    
    finally:
        action_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
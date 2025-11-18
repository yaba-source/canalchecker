import  rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from canalchecker_interface import Align
from canalchecker_interface import Follow
from canalchecker_interface import Drive
from .HandlerLogic import StateMachine

class ActionServerHandler (Node):
    def __init__(self):
        super().init_('actionserverhandler')
        
        self.state_machine=StateMachine(logger=self.get_logger())
        
        
        self.actionserveralign=self.create_client(self,Align, 'align'),
        self.actionserverdrive=self.create_client(self,Drive, 'drive'),
        self.actionserverfollow=self.create_client(self,Follow, 'follow')
      
        self.get_logger().info('Handler started')
        
    def run(self):
        """ Starts the State Machine """
        self.get_logger().info('Starting State Machine execution')
        
        try:
            self.state_machine.execute()
        except Exception as e:
            self.get_logger().error(f'Error in State Machine: {str(e)}')


        
    def send_goal(self, action_type, goal_msg):
        """Sendet Goal an Action Server"""
        if action_type == 'align':
            client = self.actionserver_align
        elif action_type == 'drive':
            client = self.actionserver_drive
        elif action_type == 'follow':
            client = self.actionserver_follow
        else:
            self.get_logger().error(f'Unknown action type: {action_type}')
            return
        client.wait_for_server()
        self._send_goal_promise = client.send_goal_async(
        goal_msg,
        feedback_callback=self.feedback_callback
        )
        self._send_goal_promise.add_done_callback(self.goal_response_callback)

    
    
    def goal_response_callback(self, promise):
        """Callback if acepted"""
        goal_handler = promise.result()
        if not goal_handler.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_promise = goal_handler.get_result_async()
        self._get_result_promise.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, promise):
        """"""
        result = promise.result().result
        self.get_logger().info('Done: ' + str(result.success))
    
    def feedback_callback(self, feedback_msg):
        """ Rü"""
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))

    def main(args=None):
        rclpy.init(args=args)
        try:
        
            handler = ActionServerHandler()
            handler.get_logger().info('Drücke Ctrl+C zum Beenden')

            handler.run()

            rclpy.spin(handler)
        
        except KeyboardInterrupt:
            handler.get_logger().info('Handler beendet')
        finally:
            handler.destroy_node()
            rclpy.shutdown()

    if __name__ == '__main__':
        main()


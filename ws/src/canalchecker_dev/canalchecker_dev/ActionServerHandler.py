import  rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.node import Node
from canalchecker_interface import Align
from canalchecker_interface import Follow
from canalchecker_interface import Drive
from logik.HandlerLogic import StateMachine

class ActionServerHandler (Node):
    def __init__(self):
        super().init_('actionserverhandler')
        self.state_machine=StateMachine(logger=self.get_logger())
        self.actionserveralign=self.create_client(self,Align, 'align'),
        self.actionserverdrive=self.create_client(self,Drive, 'drive'),
        self.actionserverfollow=self.create_client(self,Follow, 'follow')

        self.timer = self.create_timer(0.1, self.state_machine_loop)
        self.current_state = None
        self.get_logger().info('Handler started')
        
    def run(self):
        """ Starts the State Machine """
        if self.current_action is None:
            self.state_machine.execute()
            self.send_goal_for_state()
    
    def send_goal_for_state(self):
        """Sendet Goal basierend auf aktuellem State"""
        if self.state_machine.state == 10:
            self.get_logger().info("Starte Align Server")
            goal_msg = Align.Goal()
            self.send_goal('align', goal_msg)
        
        elif self.state_machine.state == 20:
            self.get_logger().info("Sende Drive Goal")
            goal_msg = Drive.Goal()
            self.send_goal('drive', goal_msg)
        
        elif self.state_machine.state == 30:
            self.get_logger().info("Sende Follow Goal")
            goal_msg = Follow.Goal()
            self.send_goal('follow', goal_msg)


        
    def send_goal(self, action_type, goal_msg):
        """Sendet Goal an Action Server
        Sendet das Goal asynchron an den Action Server
        Registriert Feedback-Callback während der Ausführung
        Registriert Callback wenn der Server antwortet (akzeptiert/ablehnt)
        """
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
        """Callback if acepted/ declined
        """
        goal_handler = promise.result()
        if not goal_handler.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_promise = goal_handler.get_result_async()
        self._get_result_promise.add_done_callback(self.get_result_callback)

    
    def get_result_callback(self, promise):
        """Answer from Server if done"""
        if self.current_action == 'align':
            self.state_machine.set_align_done()
            self.get_logger().info('Align Server: fertig')
        
        elif self.current_action == 'drive':
            self.state_machine.set_drive_done()
            self.get_logger().info('Drive Server: fertig')
        
        elif self.current_action == 'follow':
            self.state_machine.set_follow_done()
            self.get_logger().info('Follow Server: fertig')
        
        self.current_action = None
    
    def feedback_callback(self, feedback_msg):
        """"""
        feedback = feedback_msg.feedback
       

        if self.current_action == 'align':
            self.get_logger().info(f'Align Feedback: {feedback}')
        
        elif self.current_action == 'drive':
            self.get_logger().info(f'Drive Feedback: {feedback}')
        
        elif self.current_action == 'follow':
            self.get_logger().info(f'Follow Feedback: {feedback}')

    def main(args=None):
        rclpy.init(args=args)
        try:
        
            handler = ActionServerHandler()
            handler.get_logger().info('Drücke Ctrl+C zum Beenden')


            rclpy.spin(handler)
        
        except KeyboardInterrupt:
            handler.get_logger().info('Handler beendet')
        finally:
            handler.destroy_node()
            rclpy.shutdown()

    if __name__ == '__main__':
        main()


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from canalchecker_interface.action import Align, Follow, Drive
from .logik.HandlerLogic import StateMachine

class ActionServerHandler(Node):
    def __init__(self):
        super().__init__('actionserverhandler')
        self.state_machine = StateMachine(logger=self.get_logger())
        
        self.actionserver_align = ActionClient(self, Align, 'align')
        self.actionserver_drive = ActionClient(self, Drive, 'drive')
        self.actionserver_follow = ActionClient(self, Follow, 'follow')
        
        self.timer = self.create_timer(0.1, self.state_machine_loop)
        self.current_action = None 
        self.get_logger().info('Handler started')

    def state_machine_loop(self):
        """Timer Callback - führt State Machine aus"""
        if self.current_action is None:
            self.state_machine.execute()
            self.send_goal_for_state()

    def send_goal_for_state(self):
        """Sendet Goal basierend auf aktuellem State"""
        match self.state_machine.state:
            case 10:
                self.get_logger().info("Starte Align Server")
                goal_msg = Align.Goal()
                self.send_goal('align', goal_msg)
            
            case 20:
                self.get_logger().info("Sende Drive Goal")
                goal_msg = Drive.Goal()
                self.send_goal('drive', goal_msg)
            
            case 30:
                self.get_logger().info("Sende Follow Goal")
                goal_msg = Follow.Goal()
                self.send_goal('follow', goal_msg)

    def send_goal(self, action_type, goal_msg):
        """Sendet Goal an Action Server"""
        clients = {
            'align': self.actionserver_align,
            'drive': self.actionserver_drive,
            'follow': self.actionserver_follow
        }
        
        client = clients.get(action_type)
        if not client:
            self.get_logger().error(f'Unknown action type: {action_type}')
            return
        
        self.current_action = action_type
        client.wait_for_server()
        
        self._send_goal_promise = client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_promise.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, promise):
        """Callback wenn Goal akzeptiert/abgelehnt"""
        goal_handler = promise.result()
        if not goal_handler.accepted:
            self.get_logger().info('Goal rejected :(')
            self.current_action = None
            return
        
        self.get_logger().info('Goal accepted :)')
        self._get_result_promise = goal_handler.get_result_async()
        self._get_result_promise.add_done_callback(self.get_result_callback)

    def get_result_callback(self, promise):
        """Callback wenn Server fertig ist - prüft Status und Result"""
        result_response = promise.result()
        result = result_response.result
        status = result_response.status
        
        # Status Codes:
        # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        if status != 4:  # 4 = SUCCEEDED
            self.get_logger().error(
                f'{self.current_action} Server FEHLGESCHLAGEN (status={status})'
            )
            self.current_action = None
            return
        
        # Action war erfolgreich - prüfe Result
        match self.current_action:
            case 'align':
                if hasattr(result, 'reached') and result.reached:
                    self.state_machine.set_align_done()
                    self.get_logger().info('✓ Align Server ERFOLGREICH')
                else:
                    self.get_logger().warn(
                        'Align Server beendet aber reached=False'
                    )
            
            case 'drive':
                if hasattr(result, 'reached') and result.reached:
                    self.state_machine.set_drive_done()
                    self.get_logger().info('✓ Drive Server ERFOLGREICH')
                else:
                    self.get_logger().warn(
                        'Drive Server beendet aber reached=False'
                    )
            
            case 'follow':
                if hasattr(result, 'reached') and result.reached:
                    self.state_machine.set_follow_done()
                    self.get_logger().info('✓ Follow Server ERFOLGREICH')
                else:
                    self.get_logger().warn(
                        'Follow Server beendet aber reached=False'
                    )
        
        self.current_action = None

    def feedback_callback(self, feedback_msg):
        """Callback für Feedback während der Ausführung - NUR zur Anzeige"""
        feedback = feedback_msg.feedback
        
        match self.current_action:
            case 'align':
                if hasattr(feedback, 'angle_to_goal'):
                    self.get_logger().info(
                        f'Align Feedback: angle={feedback.angle_to_goal:.2f}°'
                    )
                else:
                    self.get_logger().info(f'Align Feedback: {feedback}')
            
            case 'drive':
                self.get_logger().info(f'Drive Feedback: {feedback}')
            
            case 'follow':
                self.get_logger().info(f'Follow Feedback: {feedback}')

def main(args=None):
    rclpy.init(args=args)
    try:
        handler = ActionServerHandler()
        handler.get_logger().info('Handler started - press Ctrl+C to exit')
        rclpy.spin(handler)
    
    except KeyboardInterrupt:
        handler.get_logger().info('Handler stopped')
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from canalchecker_interface.action import Follow
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class FollowActionServer(Node):
    def __init__(self):
        super().__init__('follow_action_server')
        
       
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

        
        self._action_server = ActionServer(
            self,
            Follow,
            'follow',
            execute_callback=self.execute_callback_fnc,
            goal_callback=self.goal_callback_fnc,
            cancel_callback=self.cancel_callback_fnc,
            callback_group=ReentrantCallbackGroup()
        )
        
        
        self._last_odom = None
        
        self.get_logger().info('Follow Action Server initialized')

    def listener_callback_fnc(self, msg: Odometry):
        self._last_odom = msg
        

    def goal_callback_fnc(self, goal_request):
        
        self.get_logger().info(f'Follow goal received: target_id={goal_request.target_id}')
        return GoalResponse.ACCEPT

    def cancel_callback_fnc(self, goal_handle):
        
        self.get_logger().info('Follow goal cancel requested')
        return CancelResponse.ACCEPT

    def execute_callback_fnc(self, goal_handle):
        
        self.get_logger().info('Executing follow action')
        
        

        
        for i in range(10):
          
            if not goal_handle.is_active:
                self.get_logger().info('Goal is no longer active')
                break
            
        
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal was canceled')
                result = Follow.Result()
                result.success = False
                return result
            
        
            feedback = Follow.Feedback()
            feedback.distance_to_target = float(10 - i) 
            goal_handle.publish_feedback(feedback)
            
            # Hier würde deine Follow-Logik kommen:
            # - Berechne Abstand zum Ziel
            # - Berechne benötigte Geschwindigkeit
            # - Veröffentliche cmd_vel
            
            cmd = Twist()
            cmd.linear.x = 0.1   
            cmd.angular.z = 0.0  
            self.publisher.publish(cmd)
            
            time.sleep(0.5)
        
    
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.publisher.publish(stop_cmd)
        
    
        if goal_handle.is_active:
            goal_handle.succeed()
            result = Follow.Result()
            result.success = True
            self.get_logger().info('Follow action succeeded')
            return result
        else:
            result = Follow.Result()
            result.success = False
            return result


def main():
    rclpy.init()
    try:
        follow_action_server = FollowActionServer()
        multithread_executor = MultiThreadedExecutor()
        rclpy.spin(follow_action_server, executor=multithread_executor)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
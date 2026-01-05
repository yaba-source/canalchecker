import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from action_interface.action import GoTo
from .logik import navigate_to_goal
from .logik_drive import drive_straight

def quaternion_to_yaw(q):
    """
    q: ein Objekt mit x, y, z, w 
    Rückgabe: yaw in Radiant
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

class RobotTargetNavigation(Node): 
    def __init__(self):
        super().__init__('goto_Action_server')
        self.navigator=navigate_to_goal()
        
        
        self.odomsub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmdpub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Action Server erstellen
        self._action_server = ActionServer(
            self,
            GoTo,
            'goto',
            self.execute_callback
        )
        
        # Timer für zyklisches Aufrufen
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.goal_handler = None
        self._goal_finished = False
        self._goal_result = None
    
    def timer_callback(self):
        if self.goal_handler is not None:
            # logik aufrufen
            self.navigator.navigate_to_target()  
            
        
            twist = Twist()
            twist.linear.x = self.navigator.linear_x

            twist.linear.y = self.navigator.linear_y
            twist.angular.z = self.navigator.angular_z
            self.cmdpub.publish(twist)
            
            # Feedback senden
            feedback_msg = GoTo.Feedback()
            feedback_msg.current_pose.x = self.navigator.current_x
            feedback_msg.current_pose.y = self.navigator.current_y
            feedback_msg.current_pose.theta = self.navigator.current_theta
            self.goal_handler.publish_feedback(feedback_msg)
            
            if self.navigator.finish:
                result = GoTo.Result()
                result.success = self.navigator.success
                if self.navigator.success:
                    self.goal_handler.succeed()
                else: 
                    self.goal_handler.abort()
                self.navigator.resetFinish()
                self._goal_finished = True
                self._goal_result = result
                self.goal_handler = None
    
    def odom_callback(self, msg: Odometry):
        self.get_logger().info("Odom callback")

        self.get_logger().info(
            f"Pos X: {msg.pose.pose.position.x:.3f} "
            f"Pos Y: {msg.pose.pose.position.y:.3f} "
            f"Angle: {quaternion_to_yaw(msg.pose.pose.orientation):.3f}"
        )
        self.navigator.setPosPara(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quaternion_to_yaw(msg.pose.pose.orientation)
        )
        self.navigator.setSpeedPara(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z
        )
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Target received')
        self.goal_handler = goal_handle
        self.navigator.linear_speed = goal_handle.request.max_linear_speed
        self.navigator.angular_speed = goal_handle.request.max_angular_speed
        self.navigator.setTargetPara(
            goal_handle.request.target_pose.x,
            goal_handle.request.target_pose.y,
            goal_handle.request.target_pose.theta
        )
        self._goal_finished = False
        self._goal_result = None
        
        while not self._goal_finished:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return self._goal_result

def main(args=None):
    rclpy.init(args=args)
    try:
        goto_Action_server = RobotTargetNavigation()
        rclpy.spin(goto_Action_server) 
    except KeyboardInterrupt:
        pass
    finally:
        if 'goto_Action_server' in locals():
            goto_Action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from canalchecker_interface.action import Align
import math, time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

#from .logic import Logic

def quaternion_to_yaw(q):
    """
    q: ein Objekt mit x, y, z, w 
    Rückgabe: yaw in Radiant
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

class AlignActionServer(Node):
    def __init__(self):
        super().__init__('align_action_server')
        self.action_server = ActionServer(
            self,
            Align,
            'align',
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
                self.get_logger().info(str(i))
                feedback = Align.Feedback()
                feedback.angle_to_goal = float(i)
                self.goal_handler.publish_feedback(feedback)
                time.sleep(0.5)
            result = Align.Result()
            result.success = True
            self.goal_handler.succeed()
            self.goal_finished = True
            self.goal_handler = None
            self.timer.cancel()


    def listener_callback_fnc(self, msg: Odometry):
        """
        Gibt den Winkel in rad auf stdout aus, wenn dieser sich ändern.
        """
        
        self.get_logger().info(f"Angle: {quaternion_to_yaw(msg.pose.pose.orientation):}")


    def execute_callback_fnc(self, goal_handle):
        self.get_logger().info('Goal Received! Aligning to Target.')
        self.goal_handler = goal_handle
        self.goal_finished = False
        self.goal_result = None

        while not self.goal_finished:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return self.goal_result


def main():
    rclpy.init()
    try:
        align_action_server = AlignActionServer()
        rclpy.spin(align_action_server)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
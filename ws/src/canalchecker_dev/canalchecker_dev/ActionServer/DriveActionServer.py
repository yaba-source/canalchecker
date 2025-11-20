import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from canalchecker_interface.action import Drive
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

#from .logic import Logic

class DriveActionServer(Node):
    def __init__(self):
        super().__init__('drive_action_server')
        self.action_server = ActionServer(
            self,
            Drive,
            'drive',
            self.execute_callback_fnc #Zuweisung von Phython verwenden  #default verhalten checken
        )
        self.goal_handle = None

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
        if self.goal_handle is not None:
            # For-loop später entfernen und mit logik / logikcalls ersetzen Timer verschrotten
            for i in range(10):
                self.get_logger().info(str(i))
                feedback = Drive.Feedback()
                feedback.dist_to_goal = float(i)
                self.goal_handle.publish_feedback(feedback) #Odometrie werte mit ausgeben 
                time.sleep(0.5)
            result = Drive.Result()
            result.reached = True
            self.goal_handle.succeed()
            self.goal_finished = True
            self.goal_result = result
            self.goal_handle = None


    def listener_callback_fnc(self, msg: Odometry):
        """
        Gibt die Koordinaten X und Y der Odometrie auf Stdout aus, wenn diese sich ändern. 
        """
        #Logik hier implemtieren 
        self.get_logger().info(f"Pos X: {msg.pose.pose.position.x:.3f} " f"Pos Y: {msg.pose.pose.position.y:.3f}")


    def execute_callback_fnc(self, goal_handle): #Multithreaded Executer
        self.get_logger().info('Goal Received! Driving.')
        self.goal_handle = goal_handle
        self.goal_finished = False
        self.goal_result = None

        while (self.goal_finished==False):#Ändern auf Multithread
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return self.goal_result


def main():
    rclpy.init()
    try:
        drive_action_server = DriveActionServer()
        rclpy.spin(drive_action_server)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
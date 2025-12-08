import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer
from rclpy.node import Node
from canalchecker_interface.action import Drive
from canalchecker_interface.msg import ArucoDetection
from canalchecker_dev.logik.DriveLogic import DriveStateMachine
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, GoalResponse
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
            execute_callback=self.execute_callback_fnc,
            goal_callback=self.goal_callback_fnc,
            callback_group=ReentrantCallbackGroup()
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

        self.sub_aruco = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.aruco_callback_fnc,
            10
        )

        self.aruco_dist = 0
        self.aruco_angle = 0
        self.aruco_id = -1

   
    def goal_callback_fnc(self, goal_request):
        self.get_logger().info('Goal received')
        return GoalResponse.ACCEPT

    def aruco_callback_fnc(self, msg: ArucoDetection):
        self.aruco_dist = msg.aruco_distance
        self.aruco_angle = msg.aruco_angle
        self.aruco_id = msg.aruco_id

    def listener_callback_fnc(self, msg: Odometry):
       # self.get_logger().info(f"Pos X: {msg.pose.pose.position.x:.3f} " f"Pos Y: {msg.pose.pose.position.y:.3f}")     
       pass   

    def execute_callback_fnc(self, goal_handle): 
        self.get_logger().info('Goal Received! Driving.')
        cmd = Twist()
        cmd.linear.x = 0
        cmd.angular.z = 0
        self.goal_handle = goal_handle
        self.goal_finished = False
        self.goal_result = None
        
        state = DriveStateMachine()
        while rclpy.ok() and not state.drive_complete:
            if self.aruco_id == -1:
                # Wenn kein Aruco Marker erkannt wird, fahre nicht
                self.get_logger().warning('No Aruco found. Stopping drive.')
                self.stop_robot()
                state.state=3
            else:
                self.get_logger().info(f'\nAruco Marker found {self.aruco_id}.\nDistance: {self.aruco_dist}\nAngle: {self.aruco_angle}')
                state.id = self.aruco_id
                state.angle = self.aruco_angle
                state.distance = self.aruco_dist
                cmd.linear.x = state.max_linear_speed
                state.state=1

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.stop_robot()
                self.get_logger().info('Goal cancelled. Robot Stopped.')
                return Drive.Result(reached=False)

            state.execute()
            if self.aruco_id == -1:
                cmd.angular.z = 0.0
                self.stop_robot()
            else:
                cmd.angular.z = float(state.angular_cmd)
                
            self.publisher.publish(cmd)

            feedback = Drive.Feedback()
            feedback.dist_to_goal = state.distance
            goal_handle.publish_feedback(feedback)
        
        self.stop_robot()

        if state.drive_complete:
            goal_handle.succeed()
            result = Drive.Result()
            result.reached = True
            self.get_logger().info('Drive Complete')
            return result
        else:
            result = Drive.Result()
            result.reached = False
            return result 
      
    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publisher.publish(cmd)


def main():
    rclpy.init()
    try:
        drive_action_server = DriveActionServer()
        multithread_executer = MultiThreadedExecutor()
        rclpy.spin(drive_action_server,multithread_executer)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
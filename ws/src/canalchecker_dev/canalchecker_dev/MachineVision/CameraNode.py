import random # Für finalen code unnötig (entfernen)
import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt32, Bool

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.publisher_dist = self.create_publisher(
            UInt32,
            '/aruco_distance',
            10
        )

        self.publisher_detected = self.create_publisher(
            Bool,
            '/aruco_detected',
            10
        )

        self.publisher_angle = self.create_publisher(
            UInt32,
            '/aruco_angle',
            10
        )

        self.publisher_id = self.create_publisher(
            UInt32,
            '/aruco_id',
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback_fnc)

    def timer_callback_fnc(self):
        aruco_dist = UInt32()
        aruco_detected = Bool()
        aruco_angle = UInt32()
        aruco_id = UInt32()

        # 'random' durch funktionscalls ersetzen
        aruco_dist.data = random.randint(0, 10)
        aruco_detected.data = bool(random.randint(0, 1))
        aruco_angle = random.randint(0, 360)
        aruco_id = random.randint(0, 999)
        
        self.publisher_dist.publish(aruco_dist)
        self.publisher_detected.publish(aruco_detected)
        self.publisher_angle.publish(aruco_angle)
        self.publisher_id.publish(aruco_id)


def main():
    rclpy.init()
    try:
        camera_node = CameraNode()
        rclpy.spin(camera_node)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
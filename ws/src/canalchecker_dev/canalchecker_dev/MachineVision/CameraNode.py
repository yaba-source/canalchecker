import random # Für finalen code unnötig (entfernen)
import rclpy
from rclpy.node import Node
from .PictureProcessing import PictureProcessing

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

        image_processed = PictureProcessing.process_frame()
        list(image_processed)

        # 'random' durch funktionscalls ersetzen
        aruco_dist.data = image_processed[2]
        aruco_detected.data = image_processed[0]
        aruco_angle = image_processed[3]
        aruco_id = image_processed[1]
        
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
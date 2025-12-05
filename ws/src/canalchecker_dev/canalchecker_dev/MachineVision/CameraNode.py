
import rclpy
from rclpy.node import Node
from .PictureProcessing import PictureProcessing
from canalchecker_interface.msg import ArucoDetection

from std_msgs.msg import UInt32, Bool, Int32

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.publisher_dist = self.create_publisher(
            ArucoDetection,
            '/aruco_detections',
            100
        )
        self.picture_processor = PictureProcessing()

        self.timer = self.create_timer(1/60, self.timer_callback_fnc)

    def timer_callback_fnc(self):
        aruco_data = ArucoDetection()
        image_processed = self.picture_processor.process_frame()  
        aruco_data.aruco_distance = float(image_processed[1])
        aruco_data.aruco_angle = float(image_processed[2])
        aruco_data.aruco_id = int(image_processed[0])
        
        self.publisher_dist.publish(aruco_data)


def main():
    rclpy.init()
    try:
        camera_node = CameraNode()
        rclpy.spin(camera_node)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
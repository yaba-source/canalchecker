# Erstellt von Marcel K.
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import numpy as np
from .PictureProcessing import PictureProcessing
from canalchecker_interface.msg import ArucoDetection

from std_msgs.msg import UInt32, Bool, Int32

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.publisher_dist = self.create_publisher(
            ArucoDetection,
            '/aruco_detections',
            10
        )

        self.timer = self.create_timer(1/30, self.timer_callback_fnc)

    def timer_callback_fnc(self):
        aruco_data = ArucoDetection()

        functioncall = PictureProcessing()
        image_processed = functioncall.process_frame()

        ids_array, distances_array, angles_array = image_processed
        
        if ids_array is not None and ids_array.size > 0:
            for i in range(ids_array.size):
                aruco_data = ArucoDetection()
                
                aruco_data.aruco_id = int(ids_array[i])
                aruco_data.aruco_distance = float(distances_array[i])
                aruco_data.aruco_angle = float(angles_array[i])
                
                print(f"Published ID {ids_array[i]}: dist={distances_array[i]:.2f}, angle={angles_array[i]:.1f}")
        
        self.publisher_dist.publish(aruco_data)
        


def main():
    rclpy.init()
    try:
        camera_node = CameraNode()
        multithread_executor = MultiThreadedExecutor()
        rclpy.spin(camera_node, executor=multithread_executor)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
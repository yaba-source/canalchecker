import cv2 as cv
import cv2.bridge as bridge
import numpy as np
import math
#from .MachineVision.CameraNode import RosCameraNode

class PictureProcessing:
    def __init__(self):
        """
        Initialize PictureProcessing.
        """
        self.bridge = bridge.CvBridge()

    def convert_ros_to_cv(self, ros_image):
        """
        Convert ROS image message to OpenCV image.
        
        Args:
            ros_image: ROS image message
            
        Returns:
            cv_image: OpenCV image
        """
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
        print("Converted ROS image to OpenCV format.")
        return cv_image
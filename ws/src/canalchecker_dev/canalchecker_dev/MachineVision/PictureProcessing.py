import cv2 as cv
import numpy as np
import math
from .ArucoMarkerDetector import ArucoMarkerDetector


class PictureProcessing:
    def __init__(self, camera_calib=None):
        
        self.aruco_detector = ArucoMarkerDetector(camera_calib=camera_calib)
        self.cap = cv.VideoCapture(0)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return None

        if self.aruco_detector.detect_markers(frame):
            self.aruco_detector.estimate_pose()
            return (True, 
                    self.aruco_detector.ids,
                    self.aruco_detector.distances,
                    self.aruco_detector.angles)
        
        return False, 1000, 0, 0



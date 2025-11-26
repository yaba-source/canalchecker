import cv2 as cv
from .ArucoMarkerDetector import ArucoMarkerDetector

class PictureProcessing:
    def __init__(self):
        self.aruco_detector = ArucoMarkerDetector()
        self.cap = cv.VideoCapture(0)
        
        # Public Variablen
        self.has_marker = False
        self.marker_id = -1
        self.distance = 0.0
        self.angle = 0.0
    
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.has_marker = False
            return False
        
        if self.aruco_detector.detect_markers(frame):
            self.aruco_detector.estimate_pose()
            self.has_marker = True
            self.marker_id = int(self.aruco_detector.ids[0][0])
            self.distance = float(self.aruco_detector.distances[0])
            self.angle = float(self.aruco_detector.angles[0])
        else:
            self.has_marker = False
            self.marker_id = -1
            self.distance = 0.0
            self.angle = 0.0
        
        return True



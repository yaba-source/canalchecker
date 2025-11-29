import cv2 as cv
from .ArucoMarkerDetector import ArucoMarkerDetector

class PictureProcessing():
    def __init__(self):
        self.aruco_detector = ArucoMarkerDetector()
        self.cap = cv.VideoCapture(0)
        
        # Public Variablen
        #self.has_marker = False
        #self.marker_id = -1
        #self.distance = 0.0
        #self.angle = 0.0
    
    def process_frame(self):
        ret, frame = self.cap.read()
        has_marker = False
        marker_id = -1
        distance = 0.0
        angle = 0.0
        
        if not ret:
            has_marker = False
          
            return_val = [marker_id, distance, angle]
            return return_val
        
        if self.aruco_detector.detect_markers(frame):
            self.aruco_detector.estimate_pose()
            has_marker = True
            marker_id = self.aruco_detector.ids[0][0]
            distance = self.aruco_detector.distances[0]
            angle = self.aruco_detector.angles[0]
            print(self.aruco_detector.ids[0][0])
            print(self.aruco_detector.distances[0])
            print(self.aruco_detector.angles[0])
        else:
            has_marker = False
            marker_id = -1
            distance = 0.0
            angle = 0.0

   
        return_val = [marker_id, distance, angle]
        
        return return_val



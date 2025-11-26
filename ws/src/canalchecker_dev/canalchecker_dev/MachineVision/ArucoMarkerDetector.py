import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import math

class ArucoMarkerDetector:
    def __init__(self, camera_calib=None):
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        self.aruco_params = cv.aruco.DetectorParameters()
        self.marker_size = 0.175
        
        self.camera_matrix = np.array([
        [666.08925042,   0.0,         309.4006048 ],
        [0.0,            665.95103659,189.14685266],
        [0.0,              0.0,         1.0       ]
        ], dtype=np.float32)

        self.dist_coeffs = np.array([
            2.56899643e-01,
            -1.30223934e+00,
            -1.26857755e-02,
            -1.44284369e-03,
            2.21341484e+00
            ], dtype=np.float32)
            
     

        self.frame = None
        self.gray = None
        self.corners = None
        self.ids = None
        self.rejected = None
        self.rvecs = None
        self.tvecs = None
        self.distances = None
        self.angles = None


    def detect_markers(self, frame):
        
        self.frame = frame
        self.gray = cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY)
        detector = cv.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.corners, self.ids, self.rejected = detector.detectMarkers(self.gray)
        
        return self.ids is not None and len(self.corners) > 0


    def estimate_pose(self):
        
        if self.ids is None or len(self.corners) == 0:
            return None, None
        
        self.rvecs, self.tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
            self.corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
        
        self.distances = []
        self.angles = []
        
        for i, marker_id in enumerate(self.ids):
            mid = marker_id[0]
            distance = self.tvecs[i][0][2]
            x_offset = self.tvecs[i][0][0]
            y_offset = self.tvecs[i][0][1]
            angle_to_marker = math.degrees(math.atan2(y_offset, x_offset))
            
            self.distances.append(distance)
            self.angles.append(angle_to_marker)
            
            print(f"Marker ID: {mid}")
            print(f"Distance to Marker: {distance} mm")
            print(f"Angle to Marker: {angle_to_marker} degrees\n")
        
        return self.distances, self.angles
    

    


         

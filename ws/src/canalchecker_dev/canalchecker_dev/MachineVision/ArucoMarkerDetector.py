import cv2
import numpy as np
import math

class ArucoMarkerDetector:
    # Marker sizes in meters
    MARKER_SIZE_ID_0 = 0.175   # 175 mm for ID 0
    MARKER_SIZE_ID_69 = 0.075  # 75 mm for ID 69
    DEFAULT_MARKER_SIZE = 0.175  # Default size for other IDs
    
    def __init__(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.camera_matrix = np.array([
            [595.57106144,   0.0,         320.0],
            [0.0,            591.75843245,240.0],
            [0.0,              0.0,         1.0       ]
        ], dtype=np.float32)
        self.dist_coeffs = np.array([
            0.28105746,
            -1.87429319,
            -0.00611864,
            0.00719842,
            3.41067428
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
        self.gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.corners, self.ids, self.rejected = detector.detectMarkers(self.gray)
        return self.ids is not None and len(self.corners) > 0
    
    def estimate_pose(self):
        """
        SolvePnP to estimate the pose of detected Aruco markers.
        :param self: Description
        :return angle to marker in degrees and distance to marker in meters
        """
        if self.ids is None or len(self.corners) == 0:
            return None, None
        
        self.rvecs = []
        self.tvecs = []
        self.distances = []
        self.angles = []
        
        for i, marker_id in enumerate(self.ids):
            mid = marker_id[0]
            
            # Select marker size based on ID
            if mid == 0:
                marker_size = self.MARKER_SIZE_ID_0
            elif mid == 69:
                marker_size = self.MARKER_SIZE_ID_69
            else:
                marker_size = self.DEFAULT_MARKER_SIZE
            
            half_size = marker_size / 2.0
            
            obj_points = np.array([
                [-half_size,  half_size, 0],  
                [ half_size,  half_size, 0], 
                [ half_size, -half_size, 0],  
                [-half_size, -half_size, 0]   
            ], dtype=np.float32)
            
            image_points = self.corners[i][0].astype(np.float32)
            
            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            
            if success:
                self.rvecs.append(rvec)
                self.tvecs.append(tvec)
                
                distance = tvec[2][0]  
                x_offset = tvec[0][0]  
                y_offset = tvec[1][0]
                angle_to_marker = math.degrees(math.atan2(x_offset, distance))
                
                self.distances.append(distance)
                self.angles.append(angle_to_marker)
                
                print(f"Marker ID: {mid}")
                print(f"Distance to Marker: {distance:.3f} m")
                print(f"X Offset: {x_offset:.3f} m")
                print(f"Y Offset: {y_offset:.3f} m")
                print(f"Angle to Marker: {angle_to_marker:.2f} degrees\n")
        
        return self.distances, self.angles
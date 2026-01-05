import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import math
#from .MachineVision.CameraCalibration import CameraCalibration
#from .MachineVision.PictureProcessing import PictureProcessing



class ArucoMarkerDetector:
    def __init__(self, camera_calib=None):
        """
        Initialize ArucoMarkerDetector.
        """
        self.aruco_dict= aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        self.aruco_params = cv.aruco.DetectorParameters()
        self.marker_size_mm = 175  
        
        if camera_calib is not None:
            self.camera_matrix = camera_calib.camera_matrix
            self.dist_coeffs = camera_calib.dist_coeffs
        else:
            
            self.camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32)
            self.dist_coeffs = np.zeros((4, 1))
        

        self.frame = None
        self.gray = None
        self.corners = None
        self.ids = None
        self.rejected = None
        self.rvecs = None
        self.tvecs = None


    def detect_markers(self, frame):
        """
        Detect ArUco markers in the given frame.
        
        Args:
            frame: Input image (BGR or RGB)
            
        Returns:
            bool: True if markers detected, False otherwise
        """
        self.frame = frame
        self.gray = cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY)
        detector = cv.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.corners, self.ids, self.rejected = detector.detectMarkers(self.gray)
        
        return self.ids is not None and len(self.corners) > 0
        




    def estimate_pose(self):
        """
        Estimate pose (position and orientation) of detected markers.
        
        Returns:
            tuple: (rvecs, tvecs) or (None, None) if no markers detected
                - rvecs: Rotation vectors
                - tvecs: Translation vectors (in mm)
        """
        if self.ids is None or len(self.corners) == 0:
            return None, None
        
        self.rvecs, self.tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
            self.corners, self.marker_size_mm, self.camera_matrix, self.dist_coeffs)
        
        marker_poses = {}
        for i, marker_id in enumerate(self.ids):
            mid = marker_id[0]
            distance = self.tvecs[i][0][2]
        
            x_offset = self.tvecs[i][0][0]
            y_offset = self.tvecs[i][0][1]
            angle_to_marker = math.degrees(math.atan2(y_offset, x_offset))
            print(f"Marker ID: {mid}")
            print(f"Distance to Marker: {distance} mm")
            print(f"Angle to Marker: {angle_to_marker} degrees\n")
        
        return angle_to_marker, distance
    




def main():    
    
    detector = ArucoMarkerDetector()
    frame = cv.imread('/home/yb/Test/picture5.png')
    if detector.detect_markers(frame):
        angle, distance = detector.estimate_pose()
        print(f"Angle to Marker: {angle} degrees")
        print(f"Distance to Marker: {distance} mm")

if __name__ == "__main__":
    main()  
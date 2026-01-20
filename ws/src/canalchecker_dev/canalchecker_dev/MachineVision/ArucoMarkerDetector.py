# Erstellt von Yannick B.
import cv2
import numpy as np
import math

class ArucoMarkerDetector:
    """
    Detektor für ArUco-Marker zur Positionsbestimmung und Pose-Estimation.
    
    Diese Klasse nutzt OpenCV's ArUco-Bibliothek um quadratische Marker zu erkennen
    und ihre genaue 3D-Position und Orientierung relativ zur Kamera zu berechnen.
    Das System unterstützt mehrere Marker-Typen mit unterschiedlichen Größen.
    
    Attributes:
        marker_sizes (dict): Dictionary das Marker-IDs auf ihre physische Größe in Metern abbildet.
            - ID 0: 175mm Marker (Zielpunkt)
            - ID 69: 75mm Marker (Trigger-Marker)
        aruco_dict: OpenCV ArUco-Wörterbuch für die Markererkennung (DICT_5X5_100)
        aruco_params: Erkennungsparameter für den ArUco-Detektor
        camera_matrix: Intrinsische Kameramatrix (Brennweite, Hauptpunkt)
        dist_coeffs: Distortionskoeffizienten zur Linsenverzerrungskorrektur
    """
    # Marker sizes in meters
    marker_sizes = {
        0: 0.175,   
        69: 0.075   
    }
    def __init__(self):
        """
        Initialisiert den ArUco-Marker Detektor mit Kalibrierungsparametern.
        
        Lädt das ArUco-Wörterbuch und die intrinsischen Kameraparameter aus der
        durchgeführten Kamerakalibrierung. Dies ist notwendig für korrekte Pose-Estimation.
        """
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
        """
        Erkennt ArUco-Marker im eingegangenen Videobild.
        
        Diese Methode scannt das gesamte Eingangsbild nach bekannten ArUco-Marker-Patterns.
        Sie konvertiert das Bild zu Grayscale um Rechenleistung zu sparen und nutzt dann
        den OpenCV ArUco-Detektor um Marker-Eckpunkte und IDs zu extrahieren.
        
        Args:
            frame: NumPy Array des Eingabebildes im BGR-Format von der Kamera
        
        Returns:
            bool: True wenn mindestens ein Marker erkannt wurde, sonst False
            
        Note:
            Die erkannten Daten werden in den Klassenvariablen self.corners,
            self.ids und self.rejected gespeichert für nachfolgende Verarbeitung.
        """
        self.gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.corners, self.ids, self.rejected = detector.detectMarkers(self.gray)
        
        return self.ids is not None and len(self.corners) > 0
    
    def estimate_pose(self):
        """
        Berechnet die 3D-Pose aller erkannten ArUco-Marker mittels PnP-Estimation.
        
        Diese Methode löst das Perspective-n-Point (PnP) Problem um aus den erkannten
        2D-Bildkoordinaten des Markers und seiner bekannten 3D-Größe die genaue
        3D-Position und Orientierung relativ zur Kamera zu berechnen.
        
        Prozess:
        1. Für jeden erkannten Marker wird die korrekte physische Größe aus marker_sizes geholt
        2. Die vier 3D-Eckpunkte des Markers im Marker-Koordinatensystem werden definiert
        3. solvePnP wird aufgerufen mit den 3D-Punkten und 2D-Bildkoordinaten
        4. Aus der Rotationsmatrix und dem Translationsvektor werden Abstand und Winkel berechnet
        
        Returns:
            tuple: (distances, angles)
                - distances (list): Abstände in Metern von der Kamera zu jedem Marker [m]
                - angles (list): Winkel in Grad zu jedem Marker relativ zur Kamera-Blickrichtung [°]
            
            Returns (None, None) wenn keine Marker erkannt wurden.
            
        Note:
            Der Winkel wird durch atan2 des X-Offsets und des Z-Abstands berechnet.
            Positive Winkel bedeuten Marker rechts von der Kamera-Blickrichtung.
            Negative Winkel bedeuten Marker links von der Kamera-Blickrichtung.
            
            Die Berechnung nutzt die vorher festgestellten Kalibrierungsparameter
            (camera_matrix und dist_coeffs) um genaue Ergebnisse zu erzielen.
        """

        if self.ids is None or len(self.corners) == 0:
            return None, None
        
        self.rvecs = []
        self.tvecs = []
        self.distances = []
        self.angles = []
        
        for i, marker_id in enumerate(self.ids):
            mid = marker_id[0]
            
           
            marker_size = self.marker_sizes.get(mid, 0.175)  
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
                flags=cv2.SOLVEPNP_IPPE_SQUARE ) # because markers are planar
            
            if success:
                self.rvecs.append(rvec)
                self.tvecs.append(tvec)
                
               
                distance = tvec[2][0]  
                x_offset = tvec[0][0]  
                y_offset = tvec[1][0]
                
                angle_to_marker = math.degrees(math.atan2(x_offset, distance))
                
                self.distances.append(distance)
                self.angles.append(angle_to_marker)
                
                #print(f"Marker ID: {mid}")
                #print(f"Distance to Marker: {distance:.3f} m")
                #print(f"X Offset: {x_offset:.3f} m")
                #print(f"Y Offset: {y_offset:.3f} m")
                #print(f"Angle to Marker: {angle_to_marker:.2f} degrees\n")
        
        return self.distances, self.angles

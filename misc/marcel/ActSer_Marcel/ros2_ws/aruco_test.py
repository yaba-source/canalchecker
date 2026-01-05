import cv2
import numpy as np

# Kamera-Parameter aus deiner Kalibrierung
camera_matrix = np.array([
    [666.08925042,   0.0,         309.4006048 ],
    [0.0,            665.95103659,189.14685266],
    [0.0,              0.0,         1.0       ]
], dtype=np.float32)

dist_coeffs = np.array([
    2.56899643e-01,
   -1.30223934e+00,
   -1.26857755e-02,
   -1.44284369e-03,
    2.21341484e+00
], dtype=np.float32)


# Reale Marker-Kantenlänge in Metern (oder cm)
MARKER_LENGTH = 0.175  # z.B. 5 cm = 0.05 m

def main():
    # Bei TurtleBot ggf. /dev/videoX oder GStreamer/ROS-Bridge verwenden
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Kamera konnte nicht geöffnet werden.")
        return

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()

    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Kein Kamerabild empfangen.")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Marker erkennen
            corners, ids, rejected = detector.detectMarkers(gray)

            if ids is not None and len(corners) > 0:
                # Pose des Markers schätzen
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, MARKER_LENGTH, camera_matrix, dist_coeffs
                )

                # Für jeden erkannten Marker Distanz ausgeben
                for i in range(len(ids)):
                    tvec = tvecs[i][0]
                    distance = np.linalg.norm(tvec)  # gleiche Einheit wie MARKER_LENGTH
                    marker_id = int(ids[i])

                    print(f"Aruco erkannt: ID={marker_id}, Distanz={distance:.3f}")
            else:
                # Optional: Nur gelegentlich loggen, um die Konsole nicht zu fluten
                print("Kein ArUco-Marker erkannt.")

    except KeyboardInterrupt:
        print("Beendet durch Benutzer (Strg+C).")

    cap.release()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import cv2
import numpy as np
import os

# ==== Anpassen ====
CHECKERBOARD = (7, 9)   # (innere Ecken pro Zeile, Spalte)
SQUARE_SIZE  = 0.020    # Kantenlänge eines Quadrats in m (20 mm)
IMAGE_DIR    = "images"
OUT_FILE     = "picam_calib.npz"
# ===================

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

images = []
for name in os.listdir(IMAGE_DIR):
    if name.lower().endswith((".png", ".jpg", ".jpeg")):
        images.append(os.path.join(IMAGE_DIR, name))
images.sort()

if not images:
    print("Keine Kalibrierbilder im Verzeichnis gefunden.")
    exit(1)

objpoints = []  # 3D-Punkte
imgpoints = []  # 2D-Punkte
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

img_size = None

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    vis = img.copy()
    if found:
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners_refined)
        objpoints.append(objp)

        cv2.drawChessboardCorners(vis, CHECKERBOARD, corners_refined, found)
        img_size = gray.shape[::-1]
        status = "OK"
    else:
        status = "FAIL"

    cv2.putText(vis, status, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                (0, 255, 0) if found else (0, 0, 255), 2)
    cv2.imshow("Checkerboard Detection", vis)
    key = cv2.waitKey(500)  # 0 für Schritt-für-Schritt, 500 ms für Schnelllauf
    if key == 27:           # ESC zum Abbrechen
        break

cv2.destroyAllWindows()

if len(objpoints) < 1:
    print("Zu wenige gültige Bilder mit erkanntem Schachbrett:", len(objpoints))
    exit(1)

ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, img_size, None, None
)

print("Erkannte Bilder: ", len(objpoints))
print("RMS-Fehler:", ret)
print("Kamera-Matrix:\n", cameraMatrix)
print("Verzerrungskoeffizienten:\n", distCoeffs.ravel())

np.savez(OUT_FILE,
         cameraMatrix=cameraMatrix,
         distCoeffs=distCoeffs,
         img_size=img_size)

print(f"Kalibrierung in '{OUT_FILE}' gespeichert.")

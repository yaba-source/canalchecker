import cv2
import time

# Maximale Bildanzahl
ANZAHL_BILDER = 100

cap = cv2.VideoCapture(0)
if not cap.isOpened():
	print("Kamera Besetzt")
	exit()

for i in range(1, ANZAHL_BILDER + 1):
	ret, frame = cap.read()
	if not ret:
		print("Kein Bild machbar")
		break

	bildname = f"Bild{i}.png"

	cv2.imwrite(bildname, frame)
	print(f"Bild {i} wurde als {bildname} gespeichert")

	if i < ANZAHL_BILDER:
		time.sleep(2)

cap.release()
print(f"Alle {ANZAHL_BILDER} Bilder aufgenommen")

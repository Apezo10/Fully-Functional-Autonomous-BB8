from picamera2 import Picamera2
import cv2

picam2  = Picamera2()

camera_config = picam2.create_preview_configuration(
	main={
		"format": "RGB888",
		"size": (640, 480),
	}
)
picam2.configure(camera_config)
picam2.start()

print("Camera started. Press Q in the camera window to quit.")

try:
	while True:
		#Capture the  current camera frame as an array.
		frame = picam2.capture_array()

		height, width = frame.shape[:2]
		center_x = width // 2
		center_y = height // 2
		
		#Draw a crosshair using OpenCV
		cv2.line(
			frame,
			(center_x - 25, center_y),
			(center_x + 25, center_y),
			(255,255,255),
			2,
		)

		cv2.line(
			frame,
			(center_x, center_y -25),
			(center_x, center_y + 25),
			(255,255,255),
			2,
		)
		
		cv2.putText(
			frame,
			"BB8 OpenCV Camera",
			(10,30),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.7,
			(255,255,255),
			2,
		)

		cv2.imshow("BB8 Camera", frame)

		#Press Q while the amera window is selected
		if cv2.waitKey(1) & 0xFF == ord("q"):
			break

finally:
	picam2.stop()
	picam2.close()
	cv2.destroyAllWindows()

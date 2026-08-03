from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import time

import cv2
from picamera2 import Picamera2


PORT = 8000
WIDTH = 640
HEIGHT = 480
FPS = 10


picam2 = Picamera2()

camera_config = picam2.create_video_configuration(
    main={
        "format": "RGB888",
        "size": (WIDTH, HEIGHT),
    },
    controls={
        "FrameRate": FPS,
    },
)

picam2.configure(camera_config)
picam2.start()

time.sleep(1)


class CameraHandler(BaseHTTPRequestHandler):

    def do_GET(self):

        # Main webpage
        if self.path == "/":
            page = """
            <!DOCTYPE html>
            <html>
                <head>
                    <title>BB-8 Camera</title>
                </head>

                <body style="
                    background-color: #111;
                    color: white;
                    text-align: center;
                    font-family: Arial;
                ">
                    <h1>BB-8 Live Camera</h1>

                    <img
                        src="/stream.mjpg"
                        style="max-width: 95%; border: 2px solid white;"
                    >

                    <p>OpenCV processing is running on the Raspberry Pi.</p>
                </body>
            </html>
            """

            page_bytes = page.encode("utf-8")

            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", len(page_bytes))
            self.end_headers()

            self.wfile.write(page_bytes)

        # Live MJPEG video
        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame",
            )
            self.end_headers()

            try:
                while True:
                    frame = picam2.capture_array()

                    height, width = frame.shape[:2]
                    center_x = width // 2
                    center_y = height // 2

                    # Draw a crosshair with OpenCV
                    cv2.line(
                        frame,
                        (center_x - 25, center_y),
                        (center_x + 25, center_y),
                        (255, 255, 255),
                        2,
                    )

                    cv2.line(
                        frame,
                        (center_x, center_y - 25),
                        (center_x, center_y + 25),
                        (255, 255, 255),
                        2,
                    )

                    cv2.putText(
                        frame,
                        "BB-8 OpenCV Stream",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                    )

                    success, encoded_image = cv2.imencode(
                        ".jpg",
                        frame,
                        [cv2.IMWRITE_JPEG_QUALITY, 70],
                    )

                    if not success:
                        continue

                    jpeg_bytes = encoded_image.tobytes()

                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        f"Content-Length: {len(jpeg_bytes)}\r\n\r\n".encode()
                    )
                    self.wfile.write(jpeg_bytes)
                    self.wfile.write(b"\r\n")

                    time.sleep(1 / FPS)

            except (BrokenPipeError, ConnectionResetError):
                print("Browser disconnected")

        else:
            self.send_error(404)


server = ThreadingHTTPServer(("0.0.0.0", PORT), CameraHandler)

print(f"BB-8 camera server started on port {PORT}")
print("Press Ctrl+C to stop")

try:
    server.serve_forever()

except KeyboardInterrupt:
    print("\nStopping server")

finally:
    server.server_close()
    picam2.stop()
    picam2.close()
from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import time

import cv2
from picamera2 import Picamera2

from config import TrackingConfig
from human_tracking import HumanTracker
from udp_controller import UdpController


PORT = 8000

# This server owns the Pi camera and reuses the same HumanTracker object for
# both command generation and the MJPEG overlay.
config = TrackingConfig(debug_window=False)
cv2.setNumThreads(config.opencv_threads)

picam2 = Picamera2()
camera_config = picam2.create_video_configuration(
    main={
        "format": "RGB888",
        "size": (config.frame_width, config.frame_height),
    },
    controls={
        "FrameRate": config.camera_fps,
    },
)
picam2.configure(camera_config)
picam2.start()
time.sleep(1)

tracker = HumanTracker(config)
controller = UdpController(config)


class CameraHandler(BaseHTTPRequestHandler):
    """HTTP handler for the tracker status page and MJPEG stream."""

    def do_GET(self) -> None:
        if self.path == "/":
            page = """
            <!DOCTYPE html>
            <html>
                <head>
                    <title>BB-8 Human Tracker</title>
                </head>

                <body style="
                    background-color: #111;
                    color: white;
                    text-align: center;
                    font-family: Arial;
                ">
                    <h1>BB-8 Human Tracker</h1>

                    <img
                        src="/stream.mjpg"
                        style="max-width: 95%; border: 2px solid white;"
                    >
                </body>
            </html>
            """
            page_bytes = page.encode("utf-8")

            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", len(page_bytes))
            self.end_headers()
            self.wfile.write(page_bytes)
            return

        if self.path != "/stream.mjpg":
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=frame",
        )
        self.end_headers()

        try:
            while True:
                frame = picam2.capture_array()
                if frame is None:
                    # Drop motor commands immediately if capture fails.
                    controller.stop(force=True)
                    continue

                result = tracker.process_frame(frame)
                # Lost-target zero commands bypass the normal UDP rate limit.
                controller.send(result.forward, result.turn, force=not result.valid)
                frame = tracker.draw_debug(frame, result)

                display_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                success, encoded_image = cv2.imencode(
                    ".jpg",
                    display_frame,
                    [cv2.IMWRITE_JPEG_QUALITY, 70],
                )
                if not success:
                    continue

                jpeg_bytes = encoded_image.tobytes()
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg_bytes)}\r\n\r\n".encode())
                self.wfile.write(jpeg_bytes)
                self.wfile.write(b"\r\n")

        except (BrokenPipeError, ConnectionResetError):
            print("Browser disconnected")


server = ThreadingHTTPServer(("0.0.0.0", PORT), CameraHandler)

print(f"BB-8 human tracking server started on port {PORT}")
print("Open this from another device on the same network:")
print(f"http://<raspberry-pi-ip>:{PORT}")
print("Press Ctrl+C to stop")

try:
    server.serve_forever()
except KeyboardInterrupt:
    print("\nStopping server")
except Exception:
    # Unexpected stream failures should leave the ESP32 with zero commands.
    controller.stop_repeated()
    raise
finally:
    controller.stop_repeated()
    controller.close()
    server.server_close()
    picam2.stop()
    picam2.close()

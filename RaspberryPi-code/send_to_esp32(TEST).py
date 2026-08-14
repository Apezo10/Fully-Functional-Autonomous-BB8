import socket
import time

ESP32_IP = "192.168.86.24"
ESP32_PORT = 4210
SEND_INTERVAL_SECONDS = 0.05

sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM,
)

try:
    while True:
        # Test values between -1.0 and 1.0.
        forward = 0.50
        turn = 0.00

        message = f"{forward:.3f},{turn:.3f}"

        sock.sendto(
            message.encode("utf-8"),
            (ESP32_IP, ESP32_PORT),
        )

        print("Sent:", message)

        time.sleep(SEND_INTERVAL_SECONDS)

except KeyboardInterrupt:
    print("\nSender stopped.")

finally:
    # Send a final stop command.
    sock.sendto(
        b"0.000,0.000",
        (ESP32_IP, ESP32_PORT),
    )

    sock.close()

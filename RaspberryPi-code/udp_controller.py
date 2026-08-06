from __future__ import annotations

import socket
import time

from config import TrackingConfig, clamp


class UdpController:
    """Rate-limited UDP command sender for the ESP32 motor controller."""

    def __init__(self, config: TrackingConfig) -> None:
        self._config = config
        self._address = (config.esp32_ip, config.esp32_port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.settimeout(config.udp_socket_timeout_seconds)
        self._min_send_interval = 1.0 / config.udp_send_hz
        self._last_send_time = 0.0

    def send(self, forward: float, turn: float, force: bool = False) -> bool:
        """Send one ``forward,turn`` command if the rate limit allows it."""

        now = time.monotonic()
        if not force and now - self._last_send_time < self._min_send_interval:
            return False

        # The ESP32 parser accepts -1.0 to 1.0, so clamp before formatting.
        forward = clamp(forward, -1.0, 1.0)
        turn = clamp(turn, -1.0, 1.0)
        message = f"{forward:.3f},{turn:.3f}"
        self._socket.sendto(message.encode("utf-8"), self._address)
        self._last_send_time = now
        return True

    def stop(self, force: bool = True) -> bool:
        """Send an immediate zero command by default."""

        return self.send(0.0, 0.0, force=force)

    def stop_repeated(self) -> None:
        """Send multiple stop packets because UDP delivery is not guaranteed."""

        for _ in range(self._config.shutdown_stop_packets):
            self.stop(force=True)
            time.sleep(self._config.shutdown_stop_delay_seconds)

    def close(self) -> None:
        """Release the UDP socket."""

        self._socket.close()

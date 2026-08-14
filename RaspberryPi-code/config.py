from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parent


@dataclass
class TrackingConfig:
    """Runtime settings for Pi-side detection, tracking, control, and UDP output."""

    # Keep the default frame small enough for a Raspberry Pi 3B+.
    frame_width: int = 320
    frame_height: int = 240
    camera_fps: int = 20
    opencv_threads: int = 2

    # Model files are expected beside the Raspberry Pi scripts unless overridden.
    prototxt_path: Path = BASE_DIR / "MobileNetSSD_deploy.prototxt"
    model_path: Path = BASE_DIR / "MobileNetSSD_deploy.caffemodel"
    confidence_threshold: float = 0.55
    detector_interval_frames: int = 8

    # ESP32 motor controller endpoint. The ESP32 still owns motor safety timeouts.
    esp32_ip: str = "192.168.86.39"
    esp32_port: int = 4210
    udp_send_hz: float = 20.0
    udp_socket_timeout_seconds: float = 0.2
    shutdown_stop_packets: int = 5
    shutdown_stop_delay_seconds: float = 0.05

    # Steering is proportional to normalized horizontal image error.
    turn_kp: float = 0.65
    turn_sign: float = 1.0
    horizontal_dead_zone_ratio: float = 0.08
    max_turn: float = 0.45

    # Tune this on the real robot. It depends on camera mounting and person size.
    desired_area_ratio: float = 0.18
    area_dead_band_ratio: float = 0.15
    max_forward: float = 0.45
    max_reverse: float = -0.30
    forward_kp: float = 1.8
    turn_slowdown_at_max_turn: float = 0.35

    # Tracking and command safety thresholds.
    smoothing_alpha: float = 0.75
    max_seconds_without_frame: float = 0.4
    min_box_area_ratio: float = 0.01
    max_box_area_ratio: float = 0.90
    min_box_size_px: int = 8
    max_outside_ratio: float = 0.35

    debug_window: bool = False
    fps_print_interval_seconds: float = 5.0


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Constrain a numeric value to a closed interval."""

    return max(minimum, min(maximum, value))


def validate_model_files(config: TrackingConfig) -> None:
    """Fail early with a clear message if the local DNN model files are missing."""

    missing = [
        str(path)
        for path in (config.prototxt_path, config.model_path)
        if not path.exists()
    ]
    if missing:
        raise FileNotFoundError(
            "MobileNet-SSD model files are missing:\n"
            + "\n".join(f"  - {path}" for path in missing)
            + "\nPlace the files in RaspberryPi-code or pass --prototxt and --model."
        )

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from picamera2 import Picamera2

from config import TrackingConfig, clamp
from person_detector import Detection, PersonDetector, create_kcf_tracker
from udp_controller import UdpController


Box = tuple[float, float, float, float]


@dataclass
class TrackingResult:
    """Tracking and control output for one processed video frame."""

    valid: bool
    box: Box | None
    source: str
    confidence: float | None
    forward: float
    turn: float
    area_ratio: float
    status: str
    fps: float


class HumanTracker:
    """Single-person MobileNet-SSD detector with KCF tracking between detections."""

    def __init__(self, config: TrackingConfig) -> None:
        self.config = config
        self.detector = PersonDetector(config)
        self.tracker = None
        self.frame_index = 0
        self.last_box: Box | None = None
        self.last_confidence: float | None = None
        self.target_valid = False
        self.smoothed_forward = 0.0
        self.smoothed_turn = 0.0
        self.fps = 0.0
        self._fps_frames = 0
        self._fps_window_start = time.monotonic()

    def process_frame(self, frame: np.ndarray) -> TrackingResult:
        """Process one camera frame and return safe drive commands."""

        self.frame_index += 1
        frame_height, frame_width = frame.shape[:2]
        run_detector = self._should_run_detector(frame_width, frame_height)
        source = "KCF"
        confidence = self.last_confidence
        status = "tracking"

        if run_detector:
            # Periodic DNN correction prevents KCF drift from accumulating.
            detection = self.detector.detect(frame, self.last_box)
            if detection is not None:
                self._reset_tracker(frame, detection)
                source = "MobileNet-SSD"
                confidence = detection.confidence
                status = "detected"
            else:
                self._clear_target()
                return self._lost_result("no person", frame_width, frame_height)
        else:
            ok, box = self.tracker.update(frame) if self.tracker is not None else (False, None)
            if not ok or box is None or not self._is_box_valid(box, frame_width, frame_height):
                # Re-run detection on the same frame so reacquisition is immediate.
                detection = self.detector.detect(frame, self.last_box)
                if detection is None:
                    self._clear_target()
                    return self._lost_result("tracker lost", frame_width, frame_height)

                self._reset_tracker(frame, detection)
                source = "MobileNet-SSD"
                confidence = detection.confidence
                status = "reacquired"
            else:
                self.last_box = tuple(float(value) for value in box)
                self.target_valid = True

        if self.last_box is None or not self._is_box_valid(
            self.last_box,
            frame_width,
            frame_height,
        ):
            self._clear_target()
            return self._lost_result("invalid box", frame_width, frame_height)

        forward, turn, area_ratio = self._calculate_commands(
            self.last_box,
            frame_width,
            frame_height,
        )

        # Smooth normal commands only. Lost-target paths return zero immediately.
        self.smoothed_forward = (
            self.config.smoothing_alpha * self.smoothed_forward
            + (1.0 - self.config.smoothing_alpha) * forward
        )
        self.smoothed_turn = (
            self.config.smoothing_alpha * self.smoothed_turn
            + (1.0 - self.config.smoothing_alpha) * turn
        )
        self._update_fps()

        return TrackingResult(
            valid=True,
            box=self.last_box,
            source=source,
            confidence=confidence,
            forward=self.smoothed_forward,
            turn=self.smoothed_turn,
            area_ratio=area_ratio,
            status=status,
            fps=self.fps,
        )

    def draw_debug(self, frame: np.ndarray, result: TrackingResult) -> np.ndarray:
        """Render tracking diagnostics onto a frame for local display or MJPEG."""

        height, width = frame.shape[:2]
        center_x = width // 2
        cv2.line(frame, (center_x, 0), (center_x, height), (255, 255, 255), 1)

        if result.valid and result.box is not None:
            x, y, box_width, box_height = (int(value) for value in result.box)
            target_center = (x + box_width // 2, y + box_height // 2)
            cv2.rectangle(
                frame,
                (x, y),
                (x + box_width, y + box_height),
                (0, 255, 0),
                2,
            )
            cv2.circle(frame, target_center, 4, (0, 0, 255), -1)

        confidence = "n/a" if result.confidence is None else f"{result.confidence:.2f}"
        lines = [
            f"{result.status} via {result.source}",
            f"conf {confidence} area {result.area_ratio:.3f}",
            f"forward {result.forward:.3f} turn {result.turn:.3f}",
            f"fps {result.fps:.1f}",
        ]
        for row, line in enumerate(lines):
            cv2.putText(
                frame,
                line,
                (8, 22 + row * 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )
        return frame

    def _should_run_detector(self, frame_width: int, frame_height: int) -> bool:
        """Decide whether the current frame needs an expensive DNN pass."""

        if not self.target_valid or self.tracker is None or self.last_box is None:
            return True
        if self.frame_index % self.config.detector_interval_frames == 0:
            return True
        return self._box_leaves_most_of_frame(self.last_box, frame_width, frame_height)

    def _reset_tracker(self, frame: np.ndarray, detection: Detection) -> None:
        """Initialize KCF from a fresh MobileNet-SSD detection."""

        self.tracker = create_kcf_tracker()
        x, y, width, height = detection.box
        tracker_box = (int(x), int(y), int(width), int(height))
        self.tracker.init(frame, tracker_box)
        self.last_box = detection.box
        self.last_confidence = detection.confidence
        self.target_valid = True

    def _clear_target(self) -> None:
        """Drop target state and reset command smoothing."""

        self.tracker = None
        self.last_box = None
        self.last_confidence = None
        self.target_valid = False
        self.smoothed_forward = 0.0
        self.smoothed_turn = 0.0

    def _lost_result(
        self,
        status: str,
        frame_width: int,
        frame_height: int,
    ) -> TrackingResult:
        """Return an immediate zero-command result for any lost-target condition."""

        self._update_fps()
        return TrackingResult(
            valid=False,
            box=None,
            source="none",
            confidence=None,
            forward=0.0,
            turn=0.0,
            area_ratio=0.0,
            status=status,
            fps=self.fps,
        )

    def _is_box_valid(self, box: Box, frame_width: int, frame_height: int) -> bool:
        """Reject malformed, tiny, huge, or fully off-screen tracker boxes."""

        x, y, width, height = box
        values = (x, y, width, height)
        if any(math.isnan(value) or math.isinf(value) for value in values):
            return False
        if width < self.config.min_box_size_px or height < self.config.min_box_size_px:
            return False
        if x + width <= 0 or y + height <= 0 or x >= frame_width or y >= frame_height:
            return False

        area_ratio = (width * height) / float(frame_width * frame_height)
        if area_ratio < self.config.min_box_area_ratio:
            return False
        if area_ratio > self.config.max_box_area_ratio:
            return False
        return True

    def _box_leaves_most_of_frame(
        self,
        box: Box,
        frame_width: int,
        frame_height: int,
    ) -> bool:
        """Return true when too much of the target is outside the frame."""

        x, y, width, height = box
        visible_x1 = clamp(x, 0.0, float(frame_width))
        visible_y1 = clamp(y, 0.0, float(frame_height))
        visible_x2 = clamp(x + width, 0.0, float(frame_width))
        visible_y2 = clamp(y + height, 0.0, float(frame_height))
        visible_area = max(0.0, visible_x2 - visible_x1) * max(0.0, visible_y2 - visible_y1)
        total_area = max(1.0, width * height)
        outside_ratio = 1.0 - visible_area / total_area
        return outside_ratio > self.config.max_outside_ratio

    def _calculate_commands(
        self,
        box: Box,
        frame_width: int,
        frame_height: int,
    ) -> tuple[float, float, float]:
        """Convert a valid tracking box into conservative forward/turn commands."""

        x, _, width, height = box
        target_center_x = x + width / 2.0
        frame_center_x = frame_width / 2.0
        horizontal_error = (target_center_x - frame_center_x) / frame_center_x

        # Negative error is left of center; turn_sign handles motor convention.
        if abs(horizontal_error) < self.config.horizontal_dead_zone_ratio:
            turn = 0.0
        else:
            turn = self.config.turn_sign * self.config.turn_kp * horizontal_error
        turn = clamp(turn, -self.config.max_turn, self.config.max_turn)

        area_ratio = (width * height) / float(frame_width * frame_height)
        low_band = self.config.desired_area_ratio * (1.0 - self.config.area_dead_band_ratio)
        high_band = self.config.desired_area_ratio * (1.0 + self.config.area_dead_band_ratio)

        # Box area is a cheap distance estimate suitable for first-pass testing.
        if area_ratio < low_band:
            forward = self.config.forward_kp * (low_band - area_ratio)
            forward = clamp(forward, 0.0, self.config.max_forward)
        elif area_ratio > high_band:
            reverse = -self.config.forward_kp * (area_ratio - high_band)
            forward = clamp(reverse, self.config.max_reverse, 0.0)
        else:
            forward = 0.0

        # Reduce forward motion during sharper turns to keep first tests cautious.
        turn_fraction = min(1.0, abs(turn) / max(self.config.max_turn, 0.001))
        forward *= 1.0 - self.config.turn_slowdown_at_max_turn * turn_fraction
        return forward, turn, area_ratio

    def _update_fps(self) -> None:
        """Update a rolling FPS estimate without doing per-frame printing."""

        self._fps_frames += 1
        now = time.monotonic()
        elapsed = now - self._fps_window_start
        if elapsed >= 1.0:
            self.fps = self._fps_frames / elapsed
            self._fps_frames = 0
            self._fps_window_start = now


def build_arg_parser() -> argparse.ArgumentParser:
    """Create CLI overrides for deployment-specific settings."""

    parser = argparse.ArgumentParser(description="BB-8 Raspberry Pi human tracker")
    parser.add_argument("--esp32-ip", default=None)
    parser.add_argument("--esp32-port", type=int, default=None)
    parser.add_argument("--prototxt", type=Path, default=None)
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--detector-interval", type=int, default=None)
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--turn-sign", type=float, choices=(-1.0, 1.0), default=None)
    return parser


def config_from_args(args: argparse.Namespace) -> TrackingConfig:
    """Apply command-line overrides to the default configuration."""

    config = TrackingConfig()
    if args.esp32_ip is not None:
        config.esp32_ip = args.esp32_ip
    if args.esp32_port is not None:
        config.esp32_port = args.esp32_port
    if args.prototxt is not None:
        config.prototxt_path = args.prototxt
    if args.model is not None:
        config.model_path = args.model
    if args.width is not None:
        config.frame_width = args.width
    if args.height is not None:
        config.frame_height = args.height
    if args.detector_interval is not None:
        config.detector_interval_frames = args.detector_interval
    if args.debug:
        config.debug_window = True
    if args.turn_sign is not None:
        config.turn_sign = args.turn_sign
    return config


def create_camera(config: TrackingConfig) -> Picamera2:
    """Open the Pi camera at the configured low-latency tracking resolution."""

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
    time.sleep(1.0)
    return picam2


def main() -> int:
    """Run the headless/default robot tracking loop."""

    args = build_arg_parser().parse_args()
    config = config_from_args(args)
    cv2.setNumThreads(config.opencv_threads)

    tracker = HumanTracker(config)
    controller = UdpController(config)
    camera: Picamera2 | None = None
    last_frame_time = time.monotonic()
    last_fps_print = time.monotonic()

    print("Starting with conservative motor commands. Test with robot lifted first.")
    print(f"Sending UDP commands to {config.esp32_ip}:{config.esp32_port}")

    try:
        camera = create_camera(config)
        while True:
            frame = camera.capture_array()
            if frame is None:
                # Camera failure is treated as a safety stop.
                controller.stop(force=True)
                continue

            last_frame_time = time.monotonic()
            result = tracker.process_frame(frame)
            controller.send(result.forward, result.turn, force=not result.valid)

            now = time.monotonic()
            if now - last_frame_time > config.max_seconds_without_frame:
                # Never keep driving from a stale frame.
                controller.stop(force=True)

            if now - last_fps_print >= config.fps_print_interval_seconds:
                print(
                    f"FPS {result.fps:.1f} | {result.status} | "
                    f"forward {result.forward:.3f} turn {result.turn:.3f}"
                )
                last_fps_print = now

            if config.debug_window:
                debug_frame = tracker.draw_debug(frame, result)
                cv2.imshow("BB-8 Human Tracker", cv2.cvtColor(debug_frame, cv2.COLOR_RGB2BGR))
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break

    except KeyboardInterrupt:
        print("\nStopping tracker")
    except Exception:
        # Send stop packets before surfacing unexpected failures.
        controller.stop_repeated()
        raise
    finally:
        controller.stop_repeated()
        controller.close()
        if camera is not None:
            camera.stop()
            camera.close()
        if config.debug_window:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

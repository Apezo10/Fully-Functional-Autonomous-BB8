from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Iterable

import cv2
import numpy as np

from config import TrackingConfig, validate_model_files


PERSON_CLASS_ID = 15


@dataclass(frozen=True)
class Detection:
    """Single person detection in OpenCV tracker box format."""

    box: tuple[float, float, float, float]
    confidence: float


def box_center(box: tuple[float, float, float, float]) -> tuple[float, float]:
    """Return the center point of an ``(x, y, width, height)`` box."""

    x, y, width, height = box
    return x + width / 2.0, y + height / 2.0


def select_best_detection(
    detections: Iterable[Detection],
    previous_box: tuple[float, float, float, float] | None,
) -> Detection | None:
    """Choose the best single target from all person detections.

    When a previous target exists, nearest-center matching avoids jumping to
    another person. On first acquisition, largest area is a stable default.
    """

    candidates = list(detections)
    if not candidates:
        return None

    if previous_box is None:
        return max(candidates, key=lambda item: item.box[2] * item.box[3])

    previous_center = box_center(previous_box)
    return min(
        candidates,
        key=lambda item: hypot(
            box_center(item.box)[0] - previous_center[0],
            box_center(item.box)[1] - previous_center[1],
        ),
    )


class PersonDetector:
    """MobileNet-SSD person detector backed by OpenCV DNN."""

    def __init__(self, config: TrackingConfig) -> None:
        validate_model_files(config)
        self._config = config
        self._net = cv2.dnn.readNetFromCaffe(
            str(config.prototxt_path),
            str(config.model_path),
        )

    def detect(
        self,
        frame: np.ndarray,
        previous_box: tuple[float, float, float, float] | None = None,
    ) -> Detection | None:
        frame_height, frame_width = frame.shape[:2]

        # MobileNet-SSD Caffe expects 300x300 input with this scale and mean.
        # PiCamera frames are RGB888, so swapRB=True converts them for OpenCV DNN.
        blob = cv2.dnn.blobFromImage(
            frame,
            scalefactor=0.007843,
            size=(300, 300),
            mean=(127.5, 127.5, 127.5),
            swapRB=True,
            crop=False,
        )

        self._net.setInput(blob)
        output = self._net.forward()
        detections: list[Detection] = []

        for index in range(output.shape[2]):
            confidence = float(output[0, 0, index, 2])
            class_id = int(output[0, 0, index, 1])

            # VOC class 15 is "person"; all other classes are ignored.
            if class_id != PERSON_CLASS_ID:
                continue
            if confidence < self._config.confidence_threshold:
                continue

            # Convert normalized detector coordinates back into pixel boxes.
            start_x, start_y, end_x, end_y = (
                output[0, 0, index, 3:7]
                * np.array([frame_width, frame_height, frame_width, frame_height])
            )
            x = max(0.0, float(start_x))
            y = max(0.0, float(start_y))
            width = min(float(end_x), float(frame_width - 1)) - x
            height = min(float(end_y), float(frame_height - 1)) - y
            if width <= 0.0 or height <= 0.0:
                continue

            detections.append(Detection((x, y, width, height), confidence))

        return select_best_detection(detections, previous_box)


def create_kcf_tracker():
    """Create a KCF tracker across OpenCV API variants."""

    if hasattr(cv2, "TrackerKCF_create"):
        return cv2.TrackerKCF_create()

    legacy = getattr(cv2, "legacy", None)
    if legacy is not None and hasattr(legacy, "TrackerKCF_create"):
        return legacy.TrackerKCF_create()

    raise RuntimeError(
        "OpenCV KCF tracker support is missing. Install an OpenCV build with "
        "opencv-contrib-python or the distro package that includes legacy trackers."
    )

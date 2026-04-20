"""Frame grabber from the ugv_raspberry camera topic.

The ugv_raspberry ``camera_streamer_focus`` node owns focus control and
publishes JPEGs on a CompressedImage topic.  This module subscribes to
that topic from within the mission process so CV work stays local (no
disk round-trip, no re-encode).

The ``focus`` attribute of the ugv node is altitude-driven via /data;
we do NOT duplicate that control here - we only consume frames.
"""

from __future__ import annotations

import logging
import threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None  # checked by caller via healthcheck

logger = logging.getLogger(__name__)


class FrameGrabber(Node):
    """Keeps the latest decoded BGR frame in a thread-safe slot."""

    def __init__(self, topic: str) -> None:
        super().__init__("mission_frame_grabber")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub = self.create_subscription(
            CompressedImage, topic, self._on_frame, qos)
        self._lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_stamp: float = 0.0
        self._frame_count: int = 0

    def _on_frame(self, msg: CompressedImage) -> None:
        if cv2 is None:
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        with self._lock:
            self._latest_frame = frame
            self._latest_stamp = (msg.header.stamp.sec
                                  + msg.header.stamp.nanosec * 1e-9)
            self._frame_count += 1

    def latest(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._latest_frame is None else self._latest_frame.copy()

    def frame_age_s(self, now_s: float) -> float:
        with self._lock:
            if self._latest_stamp <= 0:
                return float("inf")
            return now_s - self._latest_stamp

    @property
    def frame_count(self) -> int:
        with self._lock:
            return self._frame_count

    def is_healthy(self) -> bool:
        return self.frame_count > 0

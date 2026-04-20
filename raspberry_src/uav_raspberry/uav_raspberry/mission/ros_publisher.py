"""All ROS 2 publishers used by the mission.

Publishes, per confirmed object:
  * ``uav_msgs/DetectionReport`` – full internal record (GPS + QR + photo + map)
  * ``dualtech_msgs/Detection``  – organizer's minimal message
  * ``sensor_msgs/CompressedImage`` – annotated photo overlay
  * ``sensor_msgs/NavSatFix``       – object GPS
  * ``sensor_msgs/CompressedImage`` – mission map (2D top-down)

Mission map is rendered with matplotlib's Agg backend.  It is rebuilt and
re-published every time a new object is confirmed.
"""

from __future__ import annotations

import io
import logging
import threading
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image, NavSatFix

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from uav_msgs.msg import DetectionReport  # type: ignore
from dualtech_msgs.msg import Detection    # type: ignore

from .gps_utils import GpsPoint

logger = logging.getLogger(__name__)


@dataclass
class ConfirmedTarget:
    index: int
    gps: GpsPoint
    ned_north_m: float
    ned_east_m: float
    qr_data: str = ""
    object_type: str = ""
    annotated_jpeg: Optional[bytes] = None


@dataclass
class MissionContext:
    field_length_m: float
    field_width_m: float
    home_gps: GpsPoint
    flight_path_ned: List[Tuple[float, float]] = field(default_factory=list)
    targets: List[ConfirmedTarget] = field(default_factory=list)


class MissionPublisher(Node):
    """ROS 2 node that owns all mission publishers and renders the map."""

    def __init__(
        self,
        topic_detection_report: str,
        topic_detection_organizer: str,
        topic_annotated_image: str,
        topic_object_gps: str,
        topic_mission_map: str,
    ) -> None:
        super().__init__("mission_publisher")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub_report = self.create_publisher(
            DetectionReport, topic_detection_report, qos)
        self._pub_organizer = self.create_publisher(
            Detection, topic_detection_organizer, qos)
        self._pub_annotated = self.create_publisher(
            CompressedImage, topic_annotated_image, qos)
        self._pub_gps = self.create_publisher(
            NavSatFix, topic_object_gps, qos)
        self._pub_map = self.create_publisher(
            CompressedImage, topic_mission_map, qos)

        self._lock = threading.Lock()
        self._detection_counter = 0

    # ── public API ───────────────────────────────────────────────────────────

    def publish_detection(
        self,
        target: ConfirmedTarget,
        ctx: MissionContext,
        annotated_bgr: Optional[np.ndarray],
    ) -> None:
        with self._lock:
            self._detection_counter += 1
            det_id = self._detection_counter

        stamp = self.get_clock().now().to_msg()
        annotated_jpeg = self._encode_jpeg(annotated_bgr) if annotated_bgr is not None else b""
        target.annotated_jpeg = annotated_jpeg

        # ── DetectionReport ──────────────────────────────────────────────────
        report = DetectionReport()
        report.header.stamp = stamp
        report.header.frame_id = "map"
        report.qr_data = target.qr_data
        report.object_type = target.object_type
        report.latitude = target.gps.latitude
        report.longitude = target.gps.longitude
        report.altitude_m = target.gps.altitude_m
        report.photo = self._as_compressed(annotated_jpeg, stamp)

        map_jpeg = self._render_mission_map(ctx)
        report.map_image = self._as_compressed(map_jpeg, stamp)
        self._pub_report.publish(report)

        # ── dualtech_msgs/Detection (organizer) ──────────────────────────────
        org = Detection()
        org.object_id = det_id
        org.object_type = target.object_type
        org.object_image = self._jpeg_to_image(annotated_bgr, stamp)
        self._pub_organizer.publish(org)

        # ── stand-alone annotated image + GPS fix ────────────────────────────
        if annotated_jpeg:
            self._pub_annotated.publish(self._as_compressed(annotated_jpeg, stamp))

        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = "map"
        fix.latitude = target.gps.latitude
        fix.longitude = target.gps.longitude
        fix.altitude = target.gps.altitude_m
        self._pub_gps.publish(fix)

        # ── mission map ──────────────────────────────────────────────────────
        self._pub_map.publish(self._as_compressed(map_jpeg, stamp))

    # ── helpers ──────────────────────────────────────────────────────────────

    def _encode_jpeg(self, frame_bgr: np.ndarray, quality: int = 75) -> bytes:
        if cv2 is None or frame_bgr is None:
            return b""
        ok, buf = cv2.imencode(
            ".jpg", frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        return buf.tobytes() if ok else b""

    def _as_compressed(self, jpeg_bytes: bytes, stamp) -> CompressedImage:
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.format = "jpeg"
        msg.data = list(jpeg_bytes)
        return msg

    def _jpeg_to_image(
        self, frame_bgr: Optional[np.ndarray], stamp
    ) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_frame"
        if frame_bgr is None or cv2 is None:
            msg.width = 0
            msg.height = 0
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = 0
            msg.data = []
            return msg
        h, w = frame_bgr.shape[:2]
        msg.height = h
        msg.width = w
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = frame_bgr.tobytes()
        return msg

    def _render_mission_map(self, ctx: MissionContext) -> bytes:
        fig, ax = plt.subplots(figsize=(6, 6), dpi=110)

        ax.add_patch(plt.Rectangle(
            (0, 0), ctx.field_width_m, ctx.field_length_m,
            linewidth=1.5, edgecolor="black", facecolor="none",
            label="field"))

        if ctx.flight_path_ned:
            ys = [p[0] for p in ctx.flight_path_ned]  # north
            xs = [p[1] for p in ctx.flight_path_ned]  # east
            ax.plot(xs, ys, "-", color="tab:blue",
                    linewidth=1.0, alpha=0.6, label="flight path")

        for t in ctx.targets:
            ax.plot(t.ned_east_m, t.ned_north_m, "o",
                    color="tab:red", markersize=10)
            label = t.qr_data or t.object_type or f"#{t.index}"
            ax.annotate(label,
                        xy=(t.ned_east_m, t.ned_north_m),
                        xytext=(6, 6),
                        textcoords="offset points",
                        fontsize=8)

        ax.set_xlim(-2, ctx.field_width_m + 2)
        ax.set_ylim(-2, ctx.field_length_m + 2)
        ax.set_aspect("equal")
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.set_title(f"Mission map - {len(ctx.targets)} target(s)")
        ax.grid(True, linestyle=":", alpha=0.5)
        ax.legend(loc="upper right", fontsize=8)

        buf = io.BytesIO()
        fig.savefig(buf, format="jpg", bbox_inches="tight")
        plt.close(fig)
        return buf.getvalue()

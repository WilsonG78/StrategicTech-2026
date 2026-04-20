"""Standalone vision bench test - no MAVLink, no ROS required.

Runs the full CV pipeline (HSV grey-crate + pyzbar QR + YOLOv8n) against
live frames from the RPi Camera Module 3 via the same C++-style GStreamer
capture (libcamerasrc + videoconvert + appsink) used by ugv_raspberry,
but wrapped in Python through ``cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)``.

Dual-stream behaviour (per CLAUDE.md):
  * main stream  : 1920x1080 -> fed to every detector (never downscaled)
  * preview      : 640x360 via cv2.resize of the main frame for on-screen
                   preview only.  Skipped silently if $DISPLAY is unset.
  * annotated JPEGs saved at full 1920x1080.

Focus control uses V4L2 ioctl (same subdev as camera_streamer_focus) and
exposes ``set_focus_for_altitude(alt_m)`` using the linear altitude->focus
map from the same mission_params.yaml.

Everything (HSV params, YOLO path, capture size, etc.) comes from the
config yaml - nothing is hardcoded.
"""

from __future__ import annotations

import argparse
import contextlib
import datetime as dt
import fcntl
import json
import logging
import math
import os
import queue
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple
import collections

import numpy as np
import yaml

try:
    import cv2  # type: ignore
except ImportError:
    sys.stderr.write("opencv-python not importable - aborting\n")
    raise

try:
    from pyzbar.pyzbar import decode as pyzbar_decode  # type: ignore
except ImportError:
    pyzbar_decode = None

try:
    from ultralytics import YOLO  # type: ignore
except ImportError:
    YOLO = None

from rich.align import Align
from rich.console import Console, Group
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

logger = logging.getLogger("test_vision")


# ─── V4L2 focus helper (no external deps) ─────────────────────────────────────

_VIDIOC_S_CTRL = 0xC008561C  # _IOWR('V', 28, struct v4l2_control)
_V4L2_CID_FOCUS_AUTO     = 0x009A090C
_V4L2_CID_FOCUS_ABSOLUTE = 0x009A090A


def v4l2_set_focus(subdev: str, focus_value: int) -> bool:
    """Set V4L2 focus_absolute on the given subdev path (0..1000)."""
    try:
        fd = os.open(subdev, os.O_RDWR | os.O_NONBLOCK)
    except OSError as exc:
        logger.debug("open(%s) failed: %s", subdev, exc)
        return False
    try:
        # Disable continuous AF
        buf = struct.pack("Ii", _V4L2_CID_FOCUS_AUTO, 0)
        try:
            fcntl.ioctl(fd, _VIDIOC_S_CTRL, buf)
        except OSError:
            pass
        buf = struct.pack("Ii", _V4L2_CID_FOCUS_ABSOLUTE,
                          max(0, min(1000, int(focus_value))))
        try:
            fcntl.ioctl(fd, _VIDIOC_S_CTRL, buf)
            return True
        except OSError as exc:
            logger.debug("ioctl S_CTRL focus failed: %s", exc)
            return False
    finally:
        os.close(fd)


def altitude_to_focus(
    alt_m: float, alt_min: float, alt_max: float,
    focus_at_min: int, focus_at_max: int,
) -> int:
    t = 0.0 if alt_max == alt_min else (
        max(0.0, min(1.0, (alt_m - alt_min) / (alt_max - alt_min))))
    return int(round(focus_at_min + t * (focus_at_max - focus_at_min)))


# ─── Config ──────────────────────────────────────────────────────────────────

@dataclass
class TestConfig:
    capture_width: int
    capture_height: int
    preview_width: int
    preview_height: int
    preview_every_n_frames: int
    fps: int
    v4l2_subdev: str
    jpeg_save_quality: int
    output_dir: str
    altitude_default_m: float

    # HSV / crate (from run_mission.ros__parameters)
    hsv_lower: Tuple[int, int, int]
    hsv_upper: Tuple[int, int, int]
    min_contour_area: float
    max_contour_area: float
    aspect_min: float
    aspect_max: float
    consecutive_frames: int

    # Focus linear map (shared with camera_streamer_focus)
    focus_alt_min_m: float
    focus_alt_max_m: float
    focus_at_alt_min: int
    focus_at_alt_max: int

    # YOLO
    yolo_model_path: str
    yolo_confidence_threshold: float
    yolo_imgsz: int
    yolo_every_n_frames: int

    # ROS topics (from run_mission / camera_streamer_focus)
    ros_camera_topic: str
    ros_annotated_topic: str
    ros_raw_publish_hz: float


def _resolve_config_path(path: str) -> str:
    """Resolve ``--config`` against the installed package share dir.

    Relative paths like ``config/mission_params.yaml`` stop working as
    soon as test_vision is launched from a directory other than the
    source tree (e.g. by ``ros2 run``).  Try, in order:
      1. the literal path (relative or absolute)
      2. the installed share dir via ament_index
      3. ${AMENT_PREFIX_PATH}/share/uav_raspberry/config/...
    """
    if os.path.isfile(path):
        return path
    # 2. ament_index (only available when rclpy / ROS env is sourced)
    try:
        from ament_index_python.packages import (  # type: ignore
            get_package_share_directory)
        share = get_package_share_directory("uav_raspberry")
        candidate = os.path.join(share, "config", "mission_params.yaml")
        if os.path.isfile(candidate):
            return candidate
    except Exception:  # noqa: BLE001
        pass
    # 3. walk AMENT_PREFIX_PATH manually (no rclpy dep needed)
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if not prefix:
            continue
        candidate = os.path.join(prefix, "share", "uav_raspberry",
                                 "config", "mission_params.yaml")
        if os.path.isfile(candidate):
            return candidate
    raise FileNotFoundError(
        f"mission_params.yaml not found. Tried:\n"
        f"  1. {path}\n"
        f"  2. ament share dir for uav_raspberry\n"
        f"  3. $AMENT_PREFIX_PATH/share/uav_raspberry/config/...\n"
        f"Pass an explicit --config path or `source install/setup.bash` "
        f"and rebuild the package.")


def load_config(path: str) -> TestConfig:
    path = _resolve_config_path(path)
    logger.info("loading config from %s", path)
    with open(path, "r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    run = raw["run_mission"]["ros__parameters"]
    tv = raw.get("test_vision", {}).get("ros__parameters", {})
    csf = raw["camera_streamer_focus"]["ros__parameters"]

    return TestConfig(
        capture_width=int(tv.get("capture_width", run["image_width_px"])),
        capture_height=int(tv.get("capture_height", run["image_height_px"])),
        preview_width=int(tv.get("preview_width", 480)),
        preview_height=int(tv.get("preview_height", 270)),
        preview_every_n_frames=int(tv.get("preview_every_n_frames", 2)),
        fps=int(tv.get("fps", 30)),
        v4l2_subdev=str(tv.get("v4l2_subdev", csf.get("v4l2_subdev",
                                                      "/dev/v4l-subdev0"))),
        jpeg_save_quality=int(tv.get("jpeg_save_quality", 80)),
        output_dir=str(tv.get("output_dir", "tools/test_output")),
        altitude_default_m=float(tv.get("altitude_default_m", 3.5)),
        hsv_lower=(int(run["hsv_h_min"]), int(run["hsv_s_min"]),
                   int(run["hsv_v_min"])),
        hsv_upper=(int(run["hsv_h_max"]), int(run["hsv_s_max"]),
                   int(run["hsv_v_max"])),
        min_contour_area=float(run["min_contour_area_px"]),
        max_contour_area=float(run["max_contour_area_px"]),
        aspect_min=float(run["aspect_ratio_min"]),
        aspect_max=float(run["aspect_ratio_max"]),
        consecutive_frames=int(run["detection_consecutive_frames"]),
        focus_alt_min_m=float(csf["alt_min_m"]),
        focus_alt_max_m=float(csf["alt_max_m"]),
        focus_at_alt_min=int(csf["focus_at_alt_min"]),
        focus_at_alt_max=int(csf["focus_at_alt_max"]),
        yolo_model_path=str(run["yolo_model_path"]),
        yolo_confidence_threshold=float(run["yolo_confidence_threshold"]),
        yolo_imgsz=int(run["yolo_imgsz"]),
        yolo_every_n_frames=int(tv.get("yolo_every_n_frames", 3)),
        ros_camera_topic=str(tv.get("ros_camera_topic",
                                    csf.get("topic",
                                            "/ugv_camera/image_raw/compressed"))),
        ros_annotated_topic=str(tv.get("ros_annotated_topic",
                                       run.get("topic_annotated_image",
                                               "/mission/annotated/compressed"))),
        ros_raw_publish_hz=float(tv.get("ros_raw_publish_hz", 5.0)),
    )


# ─── GStreamer capture (OpenCV backend) ───────────────────────────────────────

def make_gst_pipeline(width: int, height: int, fps: int,
                      variant: str = "nv12") -> str:
    """Build a libcamerasrc->appsink pipeline string.

    Three variants are tried in order (see ``GstCapture``) because
    libcamerasrc's caps negotiation differs across RPi OS releases:
      * ``nv12``    : explicit NV12 format (C++ streamer uses this)
      * ``generic`` : no format filter, let libcamerasrc pick
      * ``jpeg``    : in-pipeline jpegenc then jpegdec (maximum compat)
    """
    # Putting the BGR caps directly on appsink is the OpenCV-friendly
    # form - some OpenCV releases choke on a standalone
    # ``video/x-raw,format=BGR`` capsfilter immediately before appsink
    # (gst_parse_launch returns NULL, then OpenCV prints the misleading
    # 'cannot find appsink in manual pipeline' warning).
    bgr_sink = ("appsink name=appsink0 caps=video/x-raw,format=BGR "
                "sync=false drop=true max-buffers=1 emit-signals=true")
    if variant == "nv12":
        return (
            "libcamerasrc ! "
            f"video/x-raw,width={width},height={height},"
            f"framerate={fps}/1,format=NV12 ! "
            "videoconvert ! " + bgr_sink
        )
    if variant == "generic":
        return (
            "libcamerasrc ! "
            f"video/x-raw,width={width},height={height},"
            f"framerate={fps}/1 ! "
            "videoconvert ! " + bgr_sink
        )
    if variant == "jpeg":
        return (
            "libcamerasrc ! "
            f"video/x-raw,width={width},height={height},"
            f"framerate={fps}/1 ! "
            "videoconvert ! jpegenc ! jpegdec ! " + bgr_sink
        )
    raise ValueError(f"unknown pipeline variant: {variant}")


class _CaptureBackend:
    """Common interface so the main loop stays backend-agnostic."""

    def read(self) -> Optional[np.ndarray]:  # pragma: no cover - interface
        raise NotImplementedError

    def release(self) -> None:  # pragma: no cover - interface
        raise NotImplementedError


class GstCapture(_CaptureBackend):
    def __init__(self, width: int, height: int, fps: int) -> None:
        # Try variants in order of preference.  libcamerasrc caps
        # negotiation varies by RPi OS release: explicit NV12 works on
        # Bookworm mainline, but older images need a generic filter or
        # the jpegenc+jpegdec fallback to drive OpenCV.
        variants = ("nv12", "generic", "jpeg")
        last_pipeline = ""
        for v in variants:
            pipeline = make_gst_pipeline(width, height, fps, variant=v)
            logger.info("trying pipeline variant=%s: %s", v, pipeline)
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                # Grab one frame to confirm the pipeline actually runs
                ok, _ = cap.read()
                if ok:
                    logger.info("GStreamer pipeline OK (variant=%s)", v)
                    self._cap = cap
                    return
                cap.release()
                logger.warning("variant=%s opened but read() failed", v)
            last_pipeline = pipeline

        raise RuntimeError(
            "GStreamer capture failed on every variant.\n"
            f"  last pipeline: {last_pipeline}\n"
            "Diagnostics (run on the Pi, in this order):\n"
            "  1. gst-inspect-1.0 libcamerasrc  "
            "(install gstreamer1.0-libcamera if missing)\n"
            "  2. GST_DEBUG=2 gst-launch-1.0 libcamerasrc ! "
            "'video/x-raw,width=1920,height=1080,framerate=30/1,format=NV12' ! "
            "videoconvert ! fakesink -v  2>&1 | tail -20\n"
            "  3. python3 -c \"import cv2; "
            "print(cv2.getBuildInformation())\" | grep -i -A1 gstreamer\n"
            "  4. stop any process that owns the camera "
            "(ros2 node list | grep camera).")

    def read(self) -> Optional[np.ndarray]:
        ok, frame = self._cap.read()
        return frame if ok else None

    def release(self) -> None:
        with contextlib.suppress(Exception):
            self._cap.release()


class PiCamera2Capture(_CaptureBackend):
    """picamera2 fallback - used when GStreamer negotiation fails.

    Picamera2 talks to libcamera directly (no gst_parse_launch caps
    dance) so it works on every RPi OS release that has the camera
    working at all. Output is converted BGR888 -> OpenCV BGR so the
    rest of the pipeline is unchanged.
    """

    def __init__(self, width: int, height: int, fps: int) -> None:
        try:
            from picamera2 import Picamera2  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "picamera2 not importable. Install with:\n"
                "  sudo apt install -y python3-picamera2\n"
                f"(underlying error: {exc})") from exc

        self._cam = Picamera2()
        cfg = self._cam.create_video_configuration(
            main={"size": (width, height), "format": "BGR888"},
            controls={"FrameDurationLimits": (int(1e6 / fps),
                                               int(1e6 / fps))},
        )
        self._cam.configure(cfg)
        self._cam.start()
        logger.info("picamera2 backend OK  %dx%d @ %d fps", width, height, fps)

    def read(self) -> Optional[np.ndarray]:
        try:
            frame = self._cam.capture_array("main")
        except Exception as exc:  # noqa: BLE001
            logger.debug("picamera2 capture_array failed: %s", exc)
            return None
        # picamera2 BGR888 comes back as (H, W, 3) uint8 - already BGR
        # compatible with OpenCV; nothing to convert.
        return frame

    def release(self) -> None:
        with contextlib.suppress(Exception):
            self._cam.stop()
        with contextlib.suppress(Exception):
            self._cam.close()


class LatestFrameCapture(_CaptureBackend):
    """Background reader that always keeps only the most recent frame.

    Without this wrapper the main loop reads frame N from the queue,
    spends ~60 ms on YOLO/QR/HSV, then reads frame N+1 from the queue -
    the queue backs up and the preview shows frames a full second old.
    The reader thread here drops everything except the latest frame, so
    the preview is real-time regardless of detector cost.
    """

    def __init__(self, inner: _CaptureBackend) -> None:
        self._inner = inner
        self._latest: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._new = threading.Event()
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="tv-capture", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            frame = self._inner.read()
            if frame is None:
                time.sleep(0.001)
                continue
            with self._lock:
                self._latest = frame
                self._new.set()

    def read(self) -> Optional[np.ndarray]:
        # Wait briefly for a frame, but never longer than one capture
        # interval - keeps the main loop responsive to keystrokes.
        if not self._new.wait(timeout=0.1):
            return None
        with self._lock:
            frame = self._latest
            self._new.clear()
        return frame

    def release(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._inner.release()


def open_capture(width: int, height: int, fps: int,
                 backend: str) -> _CaptureBackend:
    """Dispatch to the requested backend, with graceful fallback.

    backend values:
      * ``auto``      - try GStreamer, fall back to picamera2
      * ``gstreamer`` - GStreamer only, raise on failure
      * ``picamera2`` - picamera2 only, raise on failure
    """
    backend = backend.lower()
    if backend == "gstreamer":
        inner: _CaptureBackend = GstCapture(width, height, fps)
    elif backend == "picamera2":
        inner = PiCamera2Capture(width, height, fps)
    elif backend == "auto":
        try:
            inner = GstCapture(width, height, fps)
        except Exception as exc:  # noqa: BLE001
            logger.warning("GStreamer backend unavailable (%s) - "
                           "falling back to picamera2", exc)
            inner = PiCamera2Capture(width, height, fps)
    else:
        raise ValueError(f"unknown backend: {backend}")
    # Wrap with the latest-frame drop thread so the preview is real-time
    # even when YOLO/QR/HSV run slower than the capture rate.
    return LatestFrameCapture(inner)

    def read(self) -> Optional[np.ndarray]:
        ok, frame = self._cap.read()
        return frame if ok else None

    def release(self) -> None:
        with contextlib.suppress(Exception):
            self._cap.release()


# ─── Session state ───────────────────────────────────────────────────────────

@dataclass
class Stats:
    started_at: float = field(default_factory=time.monotonic)
    frames: int = 0
    avg_fps: float = 0.0
    last_fps_mark: float = field(default_factory=time.monotonic)
    last_fps_frames: int = 0

    # YOLO
    yolo_loaded: bool = False
    yolo_last_ms: float = 0.0
    yolo_top_class: str = ""
    yolo_top_conf: float = 0.0
    yolo_detections: int = 0
    yolo_per_class: Dict[str, int] = field(default_factory=dict)

    # QR
    qr_last: str = ""
    qr_total: int = 0
    qr_codes: List[str] = field(default_factory=list)

    # HSV
    hsv_candidates: int = 0
    hsv_confirmed: int = 0

    # Camera
    focus_value: int = 0
    altitude_m: float = 0.0


@dataclass
class GuiState:
    events: Deque[tuple] = field(
        default_factory=lambda: collections.deque(maxlen=60))
    last_key: str = ""
    yolo_on: bool = True
    qr_on: bool = True
    hsv_overlay: bool = False
    frozen: bool = False


# ─── Parallel detectors ──────────────────────────────────────────────────────

class _CrateDetector:
    def __init__(self, cfg: TestConfig) -> None:
        self.cfg = cfg
        self._history: Dict[Tuple[int, int], int] = {}
        self._match_radius = max(cfg.capture_width, cfg.capture_height) * 0.08

    def run(self, frame_bgr: np.ndarray
            ) -> Tuple[List[Tuple[int, int, int, int]],
                       List[Tuple[int, int, int, int]], np.ndarray]:
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array(self.cfg.hsv_lower, dtype=np.uint8),
                           np.array(self.cfg.hsv_upper, dtype=np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

        candidates: List[Tuple[int, int, int, int]] = []
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.cfg.min_contour_area or area > self.cfg.max_contour_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if h == 0:
                continue
            aspect = w / float(h)
            if aspect < self.cfg.aspect_min or aspect > self.cfg.aspect_max:
                continue
            candidates.append((x, y, w, h))

        # Simple 3-frame streak tracker (quantised centres)
        confirmed: List[Tuple[int, int, int, int]] = []
        new_history: Dict[Tuple[int, int], int] = {}
        for (x, y, w, h) in candidates:
            cx = (x + w // 2) // 20 * 20
            cy = (y + h // 2) // 20 * 20
            streak = self._history.get((cx, cy), 0) + 1
            new_history[(cx, cy)] = streak
            if streak >= self.cfg.consecutive_frames:
                confirmed.append((x, y, w, h))
        self._history = new_history
        return candidates, confirmed, mask


def _decode_qr(frame_bgr: np.ndarray
               ) -> List[Tuple[str, Tuple[int, int, int, int]]]:
    if pyzbar_decode is None:
        return []
    out: List[Tuple[str, Tuple[int, int, int, int]]] = []
    try:
        for r in pyzbar_decode(frame_bgr):
            data = r.data.decode("utf-8", errors="replace").strip()
            if not data:
                continue
            x, y, w, h = r.rect.left, r.rect.top, r.rect.width, r.rect.height
            out.append((data, (x, y, w, h)))
    except Exception as exc:  # noqa: BLE001
        logger.debug("pyzbar error: %s", exc)
    return out


class _YoloRunner:
    def __init__(self, cfg: TestConfig) -> None:
        self.cfg = cfg
        self.model = None
        if YOLO is None:
            return
        path = os.path.expanduser(cfg.yolo_model_path)
        if not os.path.exists(path):
            logger.warning("YOLO model not found: %s", path)
            return
        try:
            self.model = YOLO(path)
        except Exception as exc:  # noqa: BLE001
            logger.error("YOLO load failed: %s", exc)

    def loaded(self) -> bool:
        return self.model is not None

    def run(self, frame_bgr: np.ndarray
            ) -> List[Tuple[str, float, Tuple[int, int, int, int]]]:
        if self.model is None:
            return []
        try:
            results = self.model.predict(
                source=frame_bgr,
                imgsz=self.cfg.yolo_imgsz,
                conf=self.cfg.yolo_confidence_threshold,
                verbose=False,
            )
        except Exception as exc:  # noqa: BLE001
            logger.debug("YOLO predict failed: %s", exc)
            return []
        if not results or results[0].boxes is None:
            return []
        r0 = results[0]
        out: List[Tuple[str, float, Tuple[int, int, int, int]]] = []
        for i in range(len(r0.boxes)):
            conf = float(r0.boxes.conf[i].item())
            cls_id = int(r0.boxes.cls[i].item())
            name = r0.names.get(cls_id, str(cls_id))
            x1, y1, x2, y2 = r0.boxes.xyxy[i].tolist()
            out.append((name, conf, (int(x1), int(y1),
                                     int(x2 - x1), int(y2 - y1))))
        return out


# ─── Optional ROS 2 bridge ────────────────────────────────────────────────────

class _RosBridge:
    """Optional rclpy publisher: raw camera frames + annotated frames.

    Imports rclpy lazily so test_vision.py still runs on a dev box
    without ROS installed. ``loaded()`` reports whether publishers are
    actually active.
    """

    def __init__(self, camera_topic: str, annotated_topic: str,
                 raw_publish_hz: float, jpeg_quality: int = 80) -> None:
        self._camera_topic = camera_topic
        self._annotated_topic = annotated_topic
        self._min_interval_s = (1.0 / raw_publish_hz
                                if raw_publish_hz > 0.0 else 0.0)
        self._jpeg_quality = jpeg_quality
        self._last_raw_pub = 0.0
        self._node = None
        self._pub_camera = None
        self._pub_annotated = None
        self._CompressedImage = None
        self._rclpy = None

        try:
            import rclpy  # type: ignore
            from rclpy.qos import (  # type: ignore
                QoSProfile, ReliabilityPolicy, HistoryPolicy)
            from sensor_msgs.msg import CompressedImage  # type: ignore
        except Exception as exc:  # noqa: BLE001
            logger.info("rclpy not available - ROS publishing disabled (%s)",
                        exc)
            return

        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            node = rclpy.create_node("test_vision")
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
            self._pub_camera = node.create_publisher(
                CompressedImage, camera_topic, qos)
            self._pub_annotated = node.create_publisher(
                CompressedImage, annotated_topic, qos)
            self._node = node
            self._rclpy = rclpy
            self._CompressedImage = CompressedImage
            logger.info("ROS bridge active: %s  +  %s",
                        camera_topic, annotated_topic)
        except Exception as exc:  # noqa: BLE001
            logger.warning("ROS bridge init failed: %s", exc)
            self._pub_camera = None
            self._pub_annotated = None

    def loaded(self) -> bool:
        return self._pub_camera is not None

    def _encode(self, frame: np.ndarray) -> Optional[bytes]:
        ok, buf = cv2.imencode(
            ".jpg", frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
        return buf.tobytes() if ok else None

    def _msg(self, frame: np.ndarray):
        if self._node is None or self._CompressedImage is None:
            return None
        payload = self._encode(frame)
        if payload is None:
            return None
        msg = self._CompressedImage()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.format = "jpeg"
        msg.data = payload
        return msg

    def publish_frame(self, frame: np.ndarray) -> None:
        if self._pub_camera is None:
            return
        now = time.monotonic()
        if (self._min_interval_s > 0.0
                and now - self._last_raw_pub < self._min_interval_s):
            return
        msg = self._msg(frame)
        if msg is None:
            return
        self._pub_camera.publish(msg)
        self._last_raw_pub = now

    def publish_annotated(self, frame: np.ndarray) -> None:
        if self._pub_annotated is None:
            return
        msg = self._msg(frame)
        if msg is not None:
            self._pub_annotated.publish(msg)

    def spin_once(self) -> None:
        if self._node is not None and self._rclpy is not None:
            try:
                self._rclpy.spin_once(self._node, timeout_sec=0.0)
            except Exception:  # noqa: BLE001
                pass

    def shutdown(self) -> None:
        if self._node is not None:
            with contextlib.suppress(Exception):
                self._node.destroy_node()
        if self._rclpy is not None:
            with contextlib.suppress(Exception):
                if self._rclpy.ok():
                    self._rclpy.shutdown()


# ─── Saved-output helpers ─────────────────────────────────────────────────────

class OutputSaver:
    def __init__(self, output_dir: str, quality: int) -> None:
        os.makedirs(output_dir, exist_ok=True)
        self.output_dir = output_dir
        self.quality = quality
        self._session = dt.datetime.now().strftime("%Y%m%d_%H%M%S")

    def save(self, frame: np.ndarray, label: str) -> str:
        name = (f"{self._session}__{int(time.time()*1000)}__"
                f"{label}.jpg")
        path = os.path.join(self.output_dir, name)
        cv2.imwrite(path, frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        return path

    def save_summary(self, payload: dict) -> str:
        name = f"{self._session}__session_summary.json"
        path = os.path.join(self.output_dir, name)
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, indent=2)
        return path


# ─── Keyboard (non-blocking) ─────────────────────────────────────────────────

def _start_key_thread(out_q: "queue.Queue[str]") -> threading.Thread:
    def reader() -> None:
        try:
            import termios
            import tty
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setcbreak(fd)
                while True:
                    ch = os.read(fd, 1).decode("utf-8", errors="ignore")
                    if ch:
                        out_q.put(ch.lower())
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
        except Exception:  # noqa: BLE001
            # Windows / non-tty: line-buffered fallback
            while True:
                line = sys.stdin.readline()
                if not line:
                    return
                ch = line.strip()[:1].lower()
                if ch:
                    out_q.put(ch)

    t = threading.Thread(target=reader, name="tv-keys", daemon=True)
    t.start()
    return t


# ─── GUI rendering ────────────────────────────────────────────────────────────

def render_layout(stats: Stats, gui: GuiState, cfg: TestConfig,
                  headless: bool) -> Layout:
    layout = Layout()
    layout.split(
        Layout(name="header", size=3),
        Layout(name="body"),
        Layout(name="footer", size=3),
    )
    layout["body"].split_row(
        Layout(name="stats", ratio=1),
        Layout(name="events", ratio=1),
    )

    elapsed = time.monotonic() - stats.started_at
    mm, ss = divmod(int(elapsed), 60)
    mode = "HEADLESS" if headless else "PREVIEW"
    title = Text.assemble(
        (f"  test_vision  ", "bold cyan"),
        (f"| elapsed {mm:02d}:{ss:02d} ", "white"),
        (f"| mode {mode} ", "magenta"),
        (f"| alt {stats.altitude_m:.2f} m ", "yellow"),
        (f"| focus {stats.focus_value} ", "green"),
    )
    layout["header"].update(Panel(Align.center(title), border_style="cyan"))

    # Stats panel
    tbl = Table.grid(padding=(0, 1))
    tbl.add_row("Camera",
                f"{cfg.capture_width}x{cfg.capture_height}  "
                f"{stats.avg_fps:.1f} fps")
    tbl.add_row("Focus", f"LensPos (V4L2 absolute) = {stats.focus_value}")
    yolo_state = (f"loaded  yes  last {stats.yolo_last_ms:.1f} ms"
                  if stats.yolo_loaded else "not loaded")
    tbl.add_row("YOLO", yolo_state + ("" if gui.yolo_on else "  [OFF]"))
    top = (f"{stats.yolo_top_class} {stats.yolo_top_conf*100:.0f}%"
           if stats.yolo_top_class else "-")
    tbl.add_row("YOLO top", top)
    tbl.add_row("YOLO detections", str(stats.yolo_detections))
    qr_state = f"count {stats.qr_total}" + ("" if gui.qr_on else "  [OFF]")
    tbl.add_row("QR", qr_state)
    tbl.add_row("QR last",
                (stats.qr_last[:40] + "...") if len(stats.qr_last) > 40
                else (stats.qr_last or "-"))
    tbl.add_row("Crate candidates", str(stats.hsv_candidates))
    tbl.add_row("Crate confirmed", str(stats.hsv_confirmed))
    layout["stats"].update(Panel(tbl, title="Live stats", border_style="blue"))

    # Event log
    ev_tbl = Table.grid(padding=(0, 1))
    for stamp, kind, msg in list(gui.events)[-20:]:
        colour = {"QR": "yellow", "YOLO": "green",
                  "CRATE": "red", "FOCUS": "cyan"}.get(kind, "white")
        ev_tbl.add_row(Text(stamp, style="grey50"),
                       Text(kind, style=colour),
                       Text(msg))
    layout["events"].update(Panel(ev_tbl, title="Events",
                                  border_style="green"))

    hsv_text = "HSV mask overlay" + (" ON" if gui.hsv_overlay else " OFF")
    foot = Text.assemble(
        (" [Q] quit ", "bold white"),
        (" [S] save frame ", "bold green"),
        (" [+/-] focus ", "bold cyan"),
        (" [F] freeze ", "magenta"),
        (" [R] reset ", "yellow"),
        (" [M] toggle YOLO ", "green"),
        (f" [H] {hsv_text} ", "blue"),
        (f"    last: {gui.last_key}", "grey50"),
    )
    layout["footer"].update(Panel(Align.center(foot), border_style="white"))
    return layout


# ─── Preview window ──────────────────────────────────────────────────────────

def try_show_preview(frame: np.ndarray, cfg: TestConfig) -> bool:
    """Show frame in the preview window.

    Caller is expected to have already resized ``frame`` to
    (cfg.preview_width, cfg.preview_height); we skip a redundant resize
    on the hot path.
    """
    if os.environ.get("DISPLAY", "") == "" and sys.platform.startswith("linux"):
        return False
    try:
        cv2.imshow("test_vision", frame)
        cv2.waitKey(1)
        return True
    except cv2.error:
        return False


# ─── Main loop ────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--config", default="config/mission_params.yaml")
    p.add_argument("--altitude", type=float, default=None)
    p.add_argument("--model", default=None)
    p.add_argument("--res", default=None, help="capture resolution WxH")
    p.add_argument("--preview", default=None,
                   help="preview window resolution WxH (default 480x270)")
    p.add_argument("--preview-every", type=int, default=None,
                   help="update preview every N captured frames (default 2)")
    p.add_argument("--fps", type=int, default=None)
    p.add_argument("--no-yolo", action="store_true")
    p.add_argument("--yolo-every", type=int, default=None,
                   help="run YOLO every N captured frames (default 3)")
    p.add_argument("--no-qr", action="store_true")
    p.add_argument("--backend", choices=("auto", "gstreamer", "picamera2"),
                   default="auto",
                   help="Capture backend (default: auto = GStreamer then "
                        "picamera2 fallback)")
    p.add_argument("--no-ros", action="store_true",
                   help="Disable ROS 2 publishing (default: auto-detect rclpy)")
    p.add_argument("--ros-camera-topic", default=None,
                   help="Override the raw camera topic")
    p.add_argument("--ros-annotated-topic", default=None,
                   help="Override the annotated frame topic")
    p.add_argument("--ros-raw-hz", type=float, default=None,
                   help="Throttle rate for raw camera publishing (Hz)")
    p.add_argument("--output", default=None)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s")

    cfg = load_config(args.config)
    if args.res:
        w, h = args.res.lower().split("x")
        cfg.capture_width, cfg.capture_height = int(w), int(h)
    if args.preview:
        pw, ph = args.preview.lower().split("x")
        cfg.preview_width, cfg.preview_height = int(pw), int(ph)
    if args.preview_every is not None:
        cfg.preview_every_n_frames = max(1, int(args.preview_every))
    if args.yolo_every is not None:
        cfg.yolo_every_n_frames = max(1, int(args.yolo_every))
    if args.fps:
        cfg.fps = args.fps
    if args.output:
        cfg.output_dir = args.output
    if args.model:
        cfg.yolo_model_path = args.model
    if args.ros_camera_topic:
        cfg.ros_camera_topic = args.ros_camera_topic
    if args.ros_annotated_topic:
        cfg.ros_annotated_topic = args.ros_annotated_topic
    if args.ros_raw_hz is not None:
        cfg.ros_raw_publish_hz = args.ros_raw_hz
    altitude = args.altitude if args.altitude is not None else cfg.altitude_default_m

    saver = OutputSaver(cfg.output_dir, cfg.jpeg_save_quality)
    stats = Stats()
    gui = GuiState()
    gui.yolo_on = not args.no_yolo
    gui.qr_on = not args.no_qr
    stats.altitude_m = altitude

    # Focus once at startup
    focus_val = altitude_to_focus(
        altitude, cfg.focus_alt_min_m, cfg.focus_alt_max_m,
        cfg.focus_at_alt_min, cfg.focus_at_alt_max)
    stats.focus_value = focus_val
    if v4l2_set_focus(cfg.v4l2_subdev, focus_val):
        gui.events.append((time.strftime("%H:%M:%S"), "FOCUS",
                           f"alt={altitude:.2f} m -> focus={focus_val}"))
    else:
        gui.events.append((time.strftime("%H:%M:%S"), "FOCUS",
                           f"v4l2 set failed on {cfg.v4l2_subdev}"))

    # Capture
    try:
        capture = open_capture(
            cfg.capture_width, cfg.capture_height, cfg.fps,
            backend=args.backend)
    except Exception as exc:
        logger.error("capture failed: %s", exc)
        return 2

    crate = _CrateDetector(cfg)
    yolo = _YoloRunner(cfg) if gui.yolo_on else None
    if yolo is not None:
        stats.yolo_loaded = yolo.loaded()

    ros: Optional[_RosBridge] = None
    if not args.no_ros:
        ros = _RosBridge(cfg.ros_camera_topic, cfg.ros_annotated_topic,
                         cfg.ros_raw_publish_hz, cfg.jpeg_save_quality)
        if ros.loaded():
            gui.events.append((time.strftime("%H:%M:%S"), "FOCUS",
                               f"ROS pub -> {cfg.ros_camera_topic} + "
                               f"{cfg.ros_annotated_topic}"))
        else:
            ros = None

    keys: "queue.Queue[str]" = queue.Queue()
    _start_key_thread(keys)

    headless = (os.environ.get("DISPLAY", "") == ""
                and sys.platform.startswith("linux"))
    if headless:
        logger.info("headless mode, preview disabled")

    console = Console()
    try:
        with Live(render_layout(stats, gui, cfg, headless),
                  console=console, refresh_per_second=5, screen=False) as live:
            while True:
                # --- key handling ---
                try:
                    while True:
                        ch = keys.get_nowait()
                        gui.last_key = ch
                        if ch == "q":
                            raise KeyboardInterrupt
                        if ch == "s":
                            frame = capture.read()
                            if frame is not None:
                                path = saver.save(frame, "manual")
                                gui.events.append((time.strftime("%H:%M:%S"),
                                                   "FOCUS",
                                                   f"saved -> {path}"))
                        if ch == "+":
                            focus_val = min(1000, stats.focus_value + 50)
                            stats.focus_value = focus_val
                            v4l2_set_focus(cfg.v4l2_subdev, focus_val)
                        if ch == "-":
                            focus_val = max(0, stats.focus_value - 50)
                            stats.focus_value = focus_val
                            v4l2_set_focus(cfg.v4l2_subdev, focus_val)
                        if ch == "f":
                            gui.frozen = not gui.frozen
                        if ch == "r":
                            stats.frames = 0
                            stats.yolo_detections = 0
                            stats.yolo_per_class.clear()
                            stats.qr_total = 0
                            stats.qr_codes.clear()
                            stats.hsv_confirmed = 0
                        if ch == "m":
                            gui.yolo_on = not gui.yolo_on
                        if ch == "h":
                            gui.hsv_overlay = not gui.hsv_overlay
                except queue.Empty:
                    pass

                if gui.frozen:
                    time.sleep(0.05)
                    live.update(render_layout(stats, gui, cfg, headless))
                    continue

                frame = capture.read()
                if frame is None:
                    time.sleep(0.01)
                    continue
                stats.frames += 1

                if ros is not None:
                    ros.publish_frame(frame)
                    ros.spin_once()

                # fps rolling avg
                now = time.monotonic()
                if now - stats.last_fps_mark >= 1.0:
                    stats.avg_fps = ((stats.frames - stats.last_fps_frames)
                                     / (now - stats.last_fps_mark))
                    stats.last_fps_mark = now
                    stats.last_fps_frames = stats.frames

                # HSV
                candidates, confirmed, mask = crate.run(frame)
                stats.hsv_candidates = len(candidates)
                if confirmed:
                    stats.hsv_confirmed += len(confirmed)
                    annotated = frame.copy()
                    for (x, y, w, h) in confirmed:
                        cv2.rectangle(annotated, (x, y),
                                      (x + w, y + h), (0, 0, 255), 3)
                    path = saver.save(annotated, "crate")
                    if ros is not None:
                        ros.publish_annotated(annotated)
                    for (x, y, w, h) in confirmed:
                        gui.events.append((time.strftime("%H:%M:%S"),
                                           "CRATE",
                                           f"bbox=({x},{y},{w},{h}) -> {path}"))

                # QR
                if gui.qr_on:
                    qrs = _decode_qr(frame)
                    for (data, (x, y, w, h)) in qrs:
                        stats.qr_total += 1
                        stats.qr_last = data
                        stats.qr_codes.append(data)
                        annotated = frame.copy()
                        cv2.rectangle(annotated, (x, y),
                                      (x + w, y + h), (0, 255, 255), 3)
                        cv2.putText(annotated, data[:60], (x, max(0, y - 8)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    (0, 255, 255), 2)
                        path = saver.save(annotated, "qr")
                        if ros is not None:
                            ros.publish_annotated(annotated)
                        gui.events.append((time.strftime("%H:%M:%S"), "QR",
                                           f"{data[:40]}  "
                                           f"pos=({x+w//2},{y+h//2})"))

                # YOLO - run only every N-th frame to keep capture snappy
                run_yolo = (gui.yolo_on and yolo is not None and yolo.loaded()
                            and (stats.frames % cfg.yolo_every_n_frames == 0))
                if run_yolo:
                    t0 = time.monotonic()
                    dets = yolo.run(frame)
                    stats.yolo_last_ms = (time.monotonic() - t0) * 1000.0
                    best: Optional[Tuple[str, float, Tuple[int, int, int, int]]]
                    best = max(dets, key=lambda d: d[1], default=None)
                    if best is not None:
                        stats.yolo_top_class, stats.yolo_top_conf, _ = best
                    for (name, conf, (x, y, w, h)) in dets:
                        stats.yolo_detections += 1
                        stats.yolo_per_class[name] = (
                            stats.yolo_per_class.get(name, 0) + 1)
                        if conf > 0.5:
                            annotated = frame.copy()
                            cv2.rectangle(annotated, (x, y),
                                          (x + w, y + h), (0, 255, 0), 3)
                            cv2.putText(annotated,
                                        f"{name} {conf*100:.0f}%",
                                        (x, max(0, y - 8)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                        (0, 255, 0), 2)
                            path = saver.save(annotated, f"yolo_{name}")
                            if ros is not None:
                                ros.publish_annotated(annotated)
                            gui.events.append((time.strftime("%H:%M:%S"),
                                               "YOLO",
                                               f"{name} {conf*100:.0f}%  "
                                               f"-> {path}"))

                # Preview - throttled to every N-th frame, downscaled early
                # so cv2.imshow does NOT run on a 1920x1080 buffer every tick
                # (that was the source of the perceived latency).
                if (not headless
                        and stats.frames % cfg.preview_every_n_frames == 0):
                    small = cv2.resize(
                        frame, (cfg.preview_width, cfg.preview_height),
                        interpolation=cv2.INTER_NEAREST)
                    if gui.hsv_overlay:
                        mask_small = cv2.resize(
                            mask, (cfg.preview_width, cfg.preview_height),
                            interpolation=cv2.INTER_NEAREST)
                        colour_mask = cv2.cvtColor(
                            mask_small, cv2.COLOR_GRAY2BGR)
                        small = cv2.addWeighted(
                            small, 0.6, colour_mask, 0.4, 0)
                    try_show_preview(small, cfg)

                live.update(render_layout(stats, gui, cfg, headless))
    except KeyboardInterrupt:
        pass
    finally:
        capture.release()
        if ros is not None:
            ros.shutdown()
        if not headless:
            with contextlib.suppress(Exception):
                cv2.destroyAllWindows()

        summary = {
            "runtime_s": time.monotonic() - stats.started_at,
            "frames": stats.frames,
            "avg_fps": stats.avg_fps,
            "yolo_loaded": stats.yolo_loaded,
            "yolo_detections_total": stats.yolo_detections,
            "yolo_per_class": stats.yolo_per_class,
            "qr_codes": stats.qr_codes,
            "qr_total": stats.qr_total,
            "crate_confirmed": stats.hsv_confirmed,
            "focus_final": stats.focus_value,
            "altitude_m": stats.altitude_m,
        }
        path = saver.save_summary(summary)
        logger.info("session summary -> %s", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())

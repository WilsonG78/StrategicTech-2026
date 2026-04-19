"""math_utils.py — pure geometry helpers (no ROS, no MAVSDK, fully testable)."""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import cv2
import numpy as np


# ── Types ─────────────────────────────────────────────────────────────────────

NedPoint = Tuple[float, float]   # (north_m, east_m)

# Default ArUco dictionary used across the system (4×4 grid, up to 50 unique IDs)
DEFAULT_ARUCO_DICT = cv2.aruco.DICT_4X4_50


# ── Lawnmower route ───────────────────────────────────────────────────────────

def generate_lawnmower(
    length_m: float,
    width_m: float,
    spacing_m: float,
) -> List[NedPoint]:
    """Return (north_m, east_m) waypoints for a boustrophedon grid.

    Sweeps North axis, steps East by *spacing_m* on each return leg.
    Guarantees at least two points even for degenerate inputs.
    """
    if spacing_m <= 0:
        raise ValueError("spacing_m must be positive")
    waypoints: List[NedPoint] = []
    east = 0.0
    go_north = True
    while east <= width_m + 1e-6:
        if go_north:
            waypoints.append((0.0, east))
            waypoints.append((length_m, east))
        else:
            waypoints.append((length_m, east))
            waypoints.append((0.0, east))
        east += spacing_m
        go_north = not go_north
    return waypoints


# ── Pixel ↔ NED conversion ────────────────────────────────────────────────────

def pixel_to_ned_offset(
    px_x: float,
    px_y: float,
    img_w: int,
    img_h: int,
    altitude_m: float,
    fov_h_deg: float,
) -> NedPoint:
    """Convert pixel coordinates of a detected object to NED offset from drone.

    Args:
        px_x, px_y : pixel position of the object in the frame
        img_w, img_h: image dimensions in pixels
        altitude_m  : drone altitude above ground (AGL)
        fov_h_deg   : horizontal field-of-view in degrees

    Returns:
        (north_m, east_m) offset of the object from the drone's nadir.
        Positive north = object is ahead; positive east = object is to the right.
    """
    if altitude_m <= 0:
        return 0.0, 0.0
    fov_rad = math.radians(fov_h_deg)
    scale = 2.0 * altitude_m * math.tan(fov_rad / 2.0) / img_w
    east_m  =  (px_x - img_w / 2.0) * scale
    north_m = -(px_y - img_h / 2.0) * scale   # image Y grows down → south
    return north_m, east_m


def ned_to_global(
    drone_lat: float,
    drone_lon: float,
    north_m: float,
    east_m: float,
) -> Tuple[float, float]:
    """Offset NED metres from a reference point to lat/lon (WGS-84 approximation).

    Accurate to <0.1 m for offsets < 1 km.
    """
    R_EARTH = 6_371_000.0
    d_lat = math.degrees(north_m / R_EARTH)
    d_lon = math.degrees(east_m / (R_EARTH * math.cos(math.radians(drone_lat))))
    return drone_lat + d_lat, drone_lon + d_lon


# ── ArUco marker detection ────────────────────────────────────────────────────

def detect_aruco_markers(
    frame: np.ndarray,
    aruco_dict_id: int = DEFAULT_ARUCO_DICT,
) -> List[Tuple[float, float, int, np.ndarray]]:
    """Detect ArUco markers in a BGR frame.

    Args:
        frame        : BGR image from the downward-facing camera.
        aruco_dict_id: OpenCV ArUco dictionary constant (default DICT_4X4_50).

    Returns:
        List of (cx_px, cy_px, marker_id, corners) where:
          cx_px, cy_px  – pixel centre of the marker
          marker_id     – integer ID encoded in the marker
          corners       – np.ndarray shape (4, 2), corner pixel positions
    """
    aruco_dict   = cv2.aruco.Dictionary_get(aruco_dict_id)
    aruco_params = cv2.aruco.DetectorParameters_create()

    corners_list, ids, _ = cv2.aruco.detectMarkers(
        frame, aruco_dict, parameters=aruco_params
    )
    if ids is None or len(ids) == 0:
        return []

    results = []
    for corners, marker_id in zip(corners_list, ids):
        pts  = corners[0]             # shape (4, 2)
        cx   = float(np.mean(pts[:, 0]))
        cy   = float(np.mean(pts[:, 1]))
        results.append((cx, cy, int(marker_id[0]), pts))
    return results


# ── White-blob detection (Phase 1 coarse scan fallback) ───────────────────────

def detect_white_blobs(
    frame: np.ndarray,
    min_area_px: float = 800.0,
) -> List[Tuple[float, float, float]]:
    """Detect large white rectangles (ArUco marker paper) in a BGR frame.

    Used as a fast coarse finder in Phase 1 when the marker is too small to
    decode reliably from altitude.  Phase 2 then uses detect_aruco_markers()
    for precise centering and ID reading.

    Returns list of (cx_px, cy_px, area_px²) tuples for each blob.
    Filters by area and aspect ratio to reject small noise and thin lines.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # White: low saturation, high value
    lower = np.array([0,   0, 180], dtype=np.uint8)
    upper = np.array([180, 50, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)

    # Morphology to fill holes and remove salt-and-pepper noise
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    blobs = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area_px:
            continue
        _, _, w, h = cv2.boundingRect(c)
        aspect = max(w, h) / max(min(w, h), 1)
        if aspect > 5.0:          # reject very thin lines
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        blobs.append((cx, cy, area))
    return blobs


# ── Nearest-neighbour path optimisation ───────────────────────────────────────

def nearest_neighbor_sort(
    targets: List[NedPoint],
    start: NedPoint = (0.0, 0.0),
) -> List[NedPoint]:
    """Greedy nearest-neighbour ordering of inspection targets.

    Not optimal TSP, but O(n²) and good enough for n < 20 targets.
    """
    remaining = list(targets)
    path: List[NedPoint] = []
    current = start
    while remaining:
        idx = min(
            range(len(remaining)),
            key=lambda i: math.hypot(
                remaining[i][0] - current[0],
                remaining[i][1] - current[1],
            ),
        )
        path.append(remaining.pop(idx))
        current = path[-1]
    return path


# ── 2-D mission map generator ─────────────────────────────────────────────────

def generate_map_image(
    waypoints: List[NedPoint],
    detected: List[NedPoint],
    current: NedPoint,
    img_size: int = 400,
) -> np.ndarray:
    """Render a top-down 2D mission map as a BGR numpy array.

    - Dark background
    - Blue lines  : lawnmower path
    - Green filled circles : detected objects
    - Yellow filled circle : current drone position
    """
    img = np.full((img_size, img_size, 3), 20, dtype=np.uint8)
    margin = 30

    all_n = [p[0] for p in waypoints] + [p[0] for p in detected] + [current[0]]
    all_e = [p[1] for p in waypoints] + [p[1] for p in detected] + [current[1]]

    min_n, max_n = min(all_n), max(all_n)
    min_e, max_e = min(all_e), max(all_e)
    span_n = max(max_n - min_n, 1.0)
    span_e = max(max_e - min_e, 1.0)
    draw_w = img_size - 2 * margin
    draw_h = img_size - 2 * margin

    def to_px(n: float, e: float) -> Tuple[int, int]:
        px = int((e - min_e) / span_e * draw_w) + margin
        py = int((max_n - n) / span_n * draw_h) + margin
        return px, py

    # Lawnmower path
    for i in range(len(waypoints) - 1):
        cv2.line(img, to_px(*waypoints[i]), to_px(*waypoints[i + 1]),
                 (180, 80, 30), 1, cv2.LINE_AA)

    # Waypoint dots
    for wp in waypoints:
        cv2.circle(img, to_px(*wp), 3, (100, 100, 200), -1)

    # Detected objects
    for d in detected:
        px = to_px(*d)
        cv2.circle(img, px, 8, (0, 220, 0), -1)
        cv2.circle(img, px, 9, (255, 255, 255), 1)

    # Drone position
    cv2.circle(img, to_px(*current), 6, (0, 230, 230), -1)

    # Legend
    cv2.putText(img, "UAV", to_px(*current), cv2.FONT_HERSHEY_SIMPLEX,
                0.4, (0, 230, 230), 1, cv2.LINE_AA)

    return img


# ── Annotated detection photo ─────────────────────────────────────────────────

def annotate_detection_photo(
    frame: np.ndarray,
    qr_bbox: Optional[np.ndarray],
    qr_data: str,
    object_type: str,
    lat: float,
    lon: float,
) -> np.ndarray:
    """Draw detection overlay on a camera frame.

    Overlays bounding box, QR data (box ID), object type, and GPS coordinates.
    """
    img = frame.copy()
    h, w = img.shape[:2]

    if qr_bbox is not None:
        pts = np.array(qr_bbox, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=2)

    lines = [
        f"ID: {qr_data}",
        f"Type: {object_type}",
        f"Lat: {lat:.6f}",
        f"Lon: {lon:.6f}",
    ]
    y0 = 30
    for line in lines:
        cv2.putText(img, line, (10, y0),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)
        y0 += 28

    return img

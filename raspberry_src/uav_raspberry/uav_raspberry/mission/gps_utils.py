"""Geospatial utilities for the UAV reconnaissance mission.

Provides:
  * GSD calculator from camera optics and altitude.
  * Pixel-offset -> ground-offset conversion at current altitude.
  * Ground-offset -> GPS translation (flat-Earth approximation, sufficient
    for the ~50 m competition field).
  * Haversine distance for target de-duplication.
  * Lawnmower waypoint generator from field geometry + FOV.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

EARTH_RADIUS_M = 6_371_000.0


@dataclass(frozen=True)
class GpsPoint:
    latitude: float
    longitude: float
    altitude_m: float = 0.0


def ground_sample_distance_m_per_px(
    altitude_m: float,
    sensor_width_mm: float,
    focal_length_mm: float,
    image_width_px: int,
) -> float:
    """Ground distance covered by one pixel at ``altitude_m`` (m/px)."""
    if focal_length_mm <= 0 or image_width_px <= 0:
        raise ValueError("focal_length_mm and image_width_px must be positive")
    return (altitude_m * sensor_width_mm) / (focal_length_mm * image_width_px)


def swath_width_m(altitude_m: float, fov_horizontal_deg: float) -> float:
    """Ground width captured by the camera frame at ``altitude_m``."""
    return 2.0 * altitude_m * math.tan(math.radians(fov_horizontal_deg) / 2.0)


def pixel_offset_to_ned_m(
    dx_px: float,
    dy_px: float,
    altitude_m: float,
    gsd_m_per_px: float,
    heading_deg: float = 0.0,
) -> Tuple[float, float]:
    """Convert pixel offset (from image centre) to a NED ground offset.

    ``dx_px`` is +right in the image, ``dy_px`` is +down.  Result is
    ``(north_m, east_m)`` after rotating by ``heading_deg``.
    """
    del altitude_m  # GSD already accounts for altitude
    east_body_m = dx_px * gsd_m_per_px
    north_body_m = -dy_px * gsd_m_per_px  # image-down = south in body frame
    hdg = math.radians(heading_deg)
    north = north_body_m * math.cos(hdg) - east_body_m * math.sin(hdg)
    east = north_body_m * math.sin(hdg) + east_body_m * math.cos(hdg)
    return north, east


def ned_offset_to_gps(
    origin: GpsPoint, north_m: float, east_m: float
) -> GpsPoint:
    """Translate a local NED offset to a GPS point (flat-Earth approximation)."""
    lat_rad = math.radians(origin.latitude)
    dlat = north_m / EARTH_RADIUS_M
    dlon = east_m / (EARTH_RADIUS_M * math.cos(lat_rad))
    return GpsPoint(
        latitude=origin.latitude + math.degrees(dlat),
        longitude=origin.longitude + math.degrees(dlon),
        altitude_m=origin.altitude_m,
    )


def haversine_distance_m(a: GpsPoint, b: GpsPoint) -> float:
    """Great-circle distance between two GPS points in metres."""
    lat1, lat2 = math.radians(a.latitude), math.radians(b.latitude)
    dlat = lat2 - lat1
    dlon = math.radians(b.longitude - a.longitude)
    h = (math.sin(dlat / 2.0) ** 2
         + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2)
    return 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(h))


def apply_drift_correction(
    point: GpsPoint, lat_offset_deg: float, lon_offset_deg: float
) -> GpsPoint:
    """Return a GPS point shifted by the given lat/lon offsets."""
    return GpsPoint(
        latitude=point.latitude + lat_offset_deg,
        longitude=point.longitude + lon_offset_deg,
        altitude_m=point.altitude_m,
    )


def lawnmower_waypoints_ned(
    field_length_m: float,
    field_width_m: float,
    altitude_m: float,
    fov_horizontal_deg: float,
    overlap_pct: float,
) -> List[Tuple[float, float, float]]:
    """Pre-compute a lawnmower pattern as a list of (north, east, down) points.

    Legs run along the North axis, stepping East between legs.  Swath width
    is derived from camera FOV at ``altitude_m`` and reduced by ``overlap_pct``.
    The returned list always starts and ends at ``(0, 0, -altitude_m)``.
    """
    if field_length_m <= 0 or field_width_m <= 0:
        raise ValueError("field dimensions must be positive")

    swath = swath_width_m(altitude_m, fov_horizontal_deg)
    step = max(0.1, swath * (1.0 - overlap_pct / 100.0))

    down = -altitude_m
    waypoints: List[Tuple[float, float, float]] = [(0.0, 0.0, down)]
    heading_north = True
    east = 0.0
    while east <= field_width_m + 1e-6:
        if heading_north:
            waypoints.append((field_length_m, east, down))
        else:
            waypoints.append((0.0, east, down))
        east_next = min(field_width_m, east + step)
        if east_next <= east:
            break
        waypoints.append((waypoints[-1][0], east_next, down))
        east = east_next
        heading_north = not heading_north
        if east >= field_width_m - 1e-6:
            if heading_north:
                waypoints.append((field_length_m, east, down))
            else:
                waypoints.append((0.0, east, down))
            break

    waypoints.append((0.0, 0.0, down))
    return waypoints

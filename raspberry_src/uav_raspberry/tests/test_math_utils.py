"""Tests for math_utils.py — pure geometry helpers."""

import math
import numpy as np
import pytest
import cv2

from uav_raspberry.math_utils import (
    DEFAULT_ARUCO_DICT,
    detect_aruco_markers,
    generate_lawnmower,
    nearest_neighbor_sort,
    ned_to_global,
    pixel_to_ned_offset,
)


# ── generate_lawnmower ────────────────────────────────────────────────────────

class TestGenerateLawnmower:
    def test_basic_grid_has_two_points_per_leg(self):
        wps = generate_lawnmower(10.0, 6.0, 2.0)
        # legs at east=0,2,4,6 → 4 legs × 2 points = 8
        assert len(wps) == 8

    def test_first_leg_goes_north(self):
        wps = generate_lawnmower(10.0, 6.0, 2.0)
        assert wps[0] == (0.0, 0.0)
        assert wps[1] == (10.0, 0.0)

    def test_second_leg_goes_south(self):
        wps = generate_lawnmower(10.0, 6.0, 2.0)
        assert wps[2] == (10.0, 2.0)
        assert wps[3] == (0.0, 2.0)

    def test_north_extent_correct(self):
        wps = generate_lawnmower(5.0, 0.0, 1.0)
        north_values = {p[0] for p in wps}
        assert 0.0 in north_values
        assert 5.0 in north_values

    def test_single_leg_when_width_zero(self):
        wps = generate_lawnmower(5.0, 0.0, 2.0)
        assert len(wps) == 2

    def test_negative_spacing_raises(self):
        with pytest.raises(ValueError):
            generate_lawnmower(10.0, 10.0, -1.0)

    def test_zero_spacing_raises(self):
        with pytest.raises(ValueError):
            generate_lawnmower(10.0, 10.0, 0.0)

    def test_all_east_values_non_negative(self):
        wps = generate_lawnmower(8.0, 8.0, 2.0)
        assert all(e >= 0.0 for _, e in wps)


# ── pixel_to_ned_offset ───────────────────────────────────────────────────────

class TestPixelToNedOffset:
    def _call(self, px_x, px_y, img_w=640, img_h=480, alt=3.0, fov=70.0):
        return pixel_to_ned_offset(px_x, px_y, img_w, img_h, alt, fov)

    def test_centre_pixel_returns_zero_offset(self):
        n, e = self._call(320, 240)
        assert abs(n) < 1e-9
        assert abs(e) < 1e-9

    def test_right_of_centre_gives_positive_east(self):
        _, e = self._call(400, 240)
        assert e > 0

    def test_left_of_centre_gives_negative_east(self):
        _, e = self._call(200, 240)
        assert e < 0

    def test_above_centre_gives_positive_north(self):
        # pixel Y increases downward, so px_y < img_h/2 → north > 0
        n, _ = self._call(320, 100)
        assert n > 0

    def test_below_centre_gives_negative_north(self):
        n, _ = self._call(320, 400)
        assert n < 0

    def test_zero_altitude_returns_zero(self):
        n, e = self._call(400, 300, alt=0.0)
        assert n == 0.0
        assert e == 0.0

    def test_negative_altitude_returns_zero(self):
        n, e = self._call(400, 300, alt=-1.0)
        assert n == 0.0
        assert e == 0.0

    def test_higher_altitude_gives_larger_offset(self):
        _, e_low  = self._call(480, 240, alt=2.0)
        _, e_high = self._call(480, 240, alt=5.0)
        assert e_high > e_low

    def test_fov_scale(self):
        # wider FOV → larger ground coverage per pixel
        _, e_narrow = self._call(480, 240, fov=50.0)
        _, e_wide   = self._call(480, 240, fov=90.0)
        assert e_wide > e_narrow

    def test_symmetry_east_west(self):
        _, e_right = self._call(480, 240)
        _, e_left  = self._call(160, 240)
        assert abs(e_right + e_left) < 1e-9

    def test_symmetry_north_south(self):
        n_up, _   = self._call(320, 100)
        n_down, _ = self._call(320, 380)
        assert abs(n_up + n_down) < 1e-9

    def test_corner_pixel_formula(self):
        img_w, img_h, alt, fov = 640, 480, 3.0, 70.0
        scale = 2.0 * alt * math.tan(math.radians(fov) / 2.0) / img_w
        expected_e = (640 - 320) * scale
        _, e = pixel_to_ned_offset(640, 240, img_w, img_h, alt, fov)
        assert abs(e - expected_e) < 1e-9


# ── ned_to_global ─────────────────────────────────────────────────────────────

class TestNedToGlobal:
    def test_zero_offset_returns_origin(self):
        lat, lon = ned_to_global(52.0, 21.0, 0.0, 0.0)
        assert abs(lat - 52.0) < 1e-9
        assert abs(lon - 21.0) < 1e-9

    def test_north_offset_increases_latitude(self):
        lat, _ = ned_to_global(52.0, 21.0, 100.0, 0.0)
        assert lat > 52.0

    def test_south_offset_decreases_latitude(self):
        lat, _ = ned_to_global(52.0, 21.0, -100.0, 0.0)
        assert lat < 52.0

    def test_east_offset_increases_longitude(self):
        _, lon = ned_to_global(52.0, 21.0, 0.0, 100.0)
        assert lon > 21.0

    def test_100m_north_roughly_0_001_deg(self):
        lat, _ = ned_to_global(52.0, 21.0, 100.0, 0.0)
        # 1 degree latitude ≈ 111 km → 100 m ≈ 0.000900°
        assert abs(lat - 52.0 - 100.0 / 6_371_000 * (180 / math.pi)) < 1e-9


# ── nearest_neighbor_sort ─────────────────────────────────────────────────────

class TestNearestNeighborSort:
    def test_empty_returns_empty(self):
        assert nearest_neighbor_sort([]) == []

    def test_single_element(self):
        pts = [(1.0, 2.0)]
        assert nearest_neighbor_sort(pts) == pts

    def test_visits_all_points(self):
        pts = [(0.0, 5.0), (3.0, 3.0), (6.0, 0.0), (1.0, 1.0)]
        result = nearest_neighbor_sort(pts)
        assert sorted(result) == sorted(pts)

    def test_closest_first_from_origin(self):
        pts = [(10.0, 0.0), (1.0, 0.0), (5.0, 0.0)]
        result = nearest_neighbor_sort(pts, start=(0.0, 0.0))
        assert result[0] == (1.0, 0.0)

    def test_custom_start(self):
        pts = [(0.0, 0.0), (10.0, 0.0)]
        result = nearest_neighbor_sort(pts, start=(9.0, 0.0))
        assert result[0] == (10.0, 0.0)

    def test_collinear_points_sorted_by_distance(self):
        pts = [(4.0, 0.0), (2.0, 0.0), (6.0, 0.0)]
        result = nearest_neighbor_sort(pts, start=(0.0, 0.0))
        assert result == [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0)]

    def test_does_not_mutate_input(self):
        pts = [(1.0, 2.0), (3.0, 4.0)]
        original = list(pts)
        nearest_neighbor_sort(pts)
        assert pts == original


# ── altitude_to_focus (inline — mirrors the C++ formula) ─────────────────────

class TestAltitudeToFocus:
    """Validates the V4L2 focus mapping used in camera_streamer_focus.cpp."""

    FOCUS_GAIN = 600.0

    def _focus(self, alt_m: float) -> int:
        val = int(self.FOCUS_GAIN / max(alt_m, 0.5))
        return max(0, min(1000, val))

    def test_at_3m_gives_200(self):
        assert self._focus(3.0) == 200

    def test_at_half_metre_gives_1000(self):
        # 600/0.5 = 1200, clamped to 1000
        assert self._focus(0.5) == 1000

    def test_zero_alt_uses_minimum_denominator(self):
        # alt=0 → max(0, 0.5)=0.5 → same as 0.5 m
        assert self._focus(0.0) == 1000

    def test_very_high_alt_approaches_zero(self):
        assert self._focus(10000.0) == 0

    def test_focus_decreases_with_altitude(self):
        values = [self._focus(a) for a in [1.0, 2.0, 3.0, 5.0, 10.0]]
        assert values == sorted(values, reverse=True)


# ── detect_aruco_markers ──────────────────────────────────────────────────────

def _make_aruco_frame(marker_id: int, img_size: int = 400) -> np.ndarray:
    """Render a single ArUco marker centred on a white background."""
    marker_size_px = img_size // 2
    aruco_dict = cv2.aruco.getPredefinedDictionary(DEFAULT_ARUCO_DICT)
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px)

    # Place on white background with margin
    frame = np.full((img_size, img_size), 255, dtype=np.uint8)
    offset = (img_size - marker_size_px) // 2
    frame[offset:offset + marker_size_px, offset:offset + marker_size_px] = marker_img
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)


class TestDetectArucoMarkers:
    def test_detects_single_marker(self):
        frame = _make_aruco_frame(7)
        results = detect_aruco_markers(frame)
        assert len(results) == 1

    def test_returns_correct_id(self):
        frame = _make_aruco_frame(3)
        results = detect_aruco_markers(frame)
        assert results[0][2] == 3

    def test_centre_near_image_centre(self):
        img_size = 400
        frame = _make_aruco_frame(0, img_size=img_size)
        results = detect_aruco_markers(frame)
        assert len(results) == 1
        cx, cy, _, _ = results[0]
        # marker is centred ± 10% of image size
        assert abs(cx - img_size / 2) < img_size * 0.10
        assert abs(cy - img_size / 2) < img_size * 0.10

    def test_corners_have_four_points(self):
        frame = _make_aruco_frame(5)
        results = detect_aruco_markers(frame)
        _, _, _, corners = results[0]
        assert corners.shape == (4, 2)

    def test_empty_frame_returns_empty_list(self):
        blank = np.zeros((480, 640, 3), dtype=np.uint8)
        assert detect_aruco_markers(blank) == []

    def test_white_frame_returns_empty_list(self):
        white = np.full((480, 640, 3), 255, dtype=np.uint8)
        assert detect_aruco_markers(white) == []

    def test_custom_dict_id_accepted(self):
        # DICT_4X4_100 = 1; marker should still be found with matching dict
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, 10, 200)
        frame = np.full((400, 400), 255, dtype=np.uint8)
        frame[100:300, 100:300] = marker_img
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        results = detect_aruco_markers(frame, aruco_dict_id=cv2.aruco.DICT_4X4_100)
        assert len(results) == 1
        assert results[0][2] == 10

    def test_wrong_dict_misses_marker(self):
        # Marker encoded with DICT_4X4_100 should NOT match DICT_4X4_50
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, 49, 200)
        frame = np.full((400, 400), 255, dtype=np.uint8)
        frame[100:300, 100:300] = marker_img
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        results = detect_aruco_markers(frame, aruco_dict_id=cv2.aruco.DICT_4X4_50)
        assert results == []

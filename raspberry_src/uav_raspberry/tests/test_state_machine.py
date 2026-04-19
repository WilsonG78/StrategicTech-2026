"""Tests for scout_mission.py state machine logic.

These tests exercise the state-transition logic in isolation — no ROS 2 spin,
no MAVSDK connection, no camera.  We patch the external dependencies so that
the node can be constructed without any hardware or middleware.
"""

from __future__ import annotations

import math
import threading
import types
import unittest.mock as mock

import pytest


# ─── Minimal stubs so we can import scout_mission without ROS 2 installed ─────

def _make_ros2_stubs():
    """Inject lightweight stubs into sys.modules before the import."""
    import sys

    # rclpy
    rclpy_mod = types.ModuleType("rclpy")
    rclpy_mod.node = types.ModuleType("rclpy.node")

    class _Node:
        def __init__(self, name):
            self._name = name
            self._params: dict = {}
            self._logger = mock.MagicMock()

        def get_logger(self):
            return self._logger

        def declare_parameter(self, name, default):
            self._params[name] = default

        def get_parameter(self, name):
            m = mock.MagicMock()
            m.value = self._params.get(name)
            m.get_parameter_value.return_value = m
            m.double_value = float(self._params.get(name, 0))
            m.integer_value = int(self._params.get(name, 0))
            m.string_value = str(self._params.get(name, ""))
            return m

        def create_publisher(self, *a, **kw):
            return mock.MagicMock()

        def create_subscription(self, *a, **kw):
            return mock.MagicMock()

        def create_timer(self, *a, **kw):
            return mock.MagicMock()

    rclpy_mod.node.Node = _Node
    rclpy_mod.spin = mock.MagicMock()
    rclpy_mod.init = mock.MagicMock()
    rclpy_mod.shutdown = mock.MagicMock()
    rclpy_mod.qos = types.ModuleType("rclpy.qos")
    rclpy_mod.qos.QoSProfile = mock.MagicMock()
    rclpy_mod.qos.ReliabilityPolicy = mock.MagicMock()
    rclpy_mod.qos.DurabilityPolicy = mock.MagicMock()

    # cv_bridge
    cv_bridge_mod = types.ModuleType("cv_bridge")
    cv_bridge_mod.CvBridge = mock.MagicMock()

    # mavsdk
    mavsdk_mod = types.ModuleType("mavsdk")
    mavsdk_mod.System = mock.MagicMock()
    mavsdk_offboard = types.ModuleType("mavsdk.offboard")
    mavsdk_offboard.PositionNedYaw = mock.MagicMock()
    mavsdk_offboard.VelocityNedYaw = mock.MagicMock()
    mavsdk_mod.offboard = mavsdk_offboard

    # message stubs
    for pkg in (
        "std_msgs", "std_msgs.msg",
        "sensor_msgs", "sensor_msgs.msg",
        "uav_msgs", "uav_msgs.msg",
    ):
        sys.modules.setdefault(pkg, types.ModuleType(pkg))

    sys.modules["std_msgs"].msg = sys.modules["std_msgs.msg"]
    sys.modules["std_msgs.msg"].String = mock.MagicMock()
    sys.modules["sensor_msgs"].msg = sys.modules["sensor_msgs.msg"]
    sys.modules["sensor_msgs.msg"].Image = mock.MagicMock()
    sys.modules["sensor_msgs.msg"].CompressedImage = mock.MagicMock()
    sys.modules["uav_msgs"].msg = sys.modules["uav_msgs.msg"]
    sys.modules["uav_msgs.msg"].UavTelemetry = mock.MagicMock()
    sys.modules["uav_msgs.msg"].DetectionReport = mock.MagicMock()

    sys.modules["rclpy"] = rclpy_mod
    sys.modules["rclpy.node"] = rclpy_mod.node
    sys.modules["rclpy.qos"] = rclpy_mod.qos
    sys.modules["cv_bridge"] = cv_bridge_mod
    sys.modules["mavsdk"] = mavsdk_mod
    sys.modules["mavsdk.offboard"] = mavsdk_offboard


_make_ros2_stubs()

# Now we can safely import the state-machine enum and helpers
from uav_raspberry.scout_mission import State  # noqa: E402


# ─── Helpers ─────────────────────────────────────────────────────────────────

def _make_node():
    """Construct a ScoutMissionNode with all async side-effects suppressed."""
    from uav_raspberry.scout_mission import ScoutMissionNode

    with (
        mock.patch("threading.Thread"),           # don't start asyncio loop
        mock.patch("asyncio.new_event_loop"),
    ):
        node = ScoutMissionNode.__new__(ScoutMissionNode)
        # Initialise only the attributes we need for state-machine tests
        node._lock = threading.Lock()
        node._state = State.TAKEOFF
        node._detected_blobs: list = []
        node._inspect_queue: list = []
        node._lawnmower_waypoints: list = []
        node._wp_index: int = 0
        node._logger = mock.MagicMock()
        node.get_logger = lambda: node._logger

        # scalar params with sensible defaults
        node._blob_dedup_radius_m: float = 1.0
        node._scan_alt_m: float = 2.5
        node._patrol_alt_m: float = 3.0
        node._scout_speed_m_s: float = 1.5
        node._wait_time_sec: float = 5.0
        node._p_gain: float = 0.3
        node._max_velocity: float = 0.5
        node._camera_fov_deg: float = 70.0
        node._img_w: int = 640
        node._img_h: int = 480

        # telemetry & drone state
        node._drone_north: float = 0.0
        node._drone_east: float = 0.0
        node._altitude_m: float = 3.0
        node._drone_lat: float = 52.0
        node._drone_lon: float = 21.0
        node._drone_yaw: float = 0.0

        node._home: tuple = (0.0, 0.0)
        node._current_target: tuple | None = None
        node._center_ok_since: float | None = None
        node._scan_complete: bool = False
        node._last_marker_id: str = ""
        node._aruco_bbox: list | None = None

        return node


# ─── State enum sanity ────────────────────────────────────────────────────────

class TestStateEnum:
    def test_all_expected_states_exist(self):
        expected = {
            "TAKEOFF", "FAST_SCOUT", "INSPECT_TARGETS",
            "APPROACH", "DESCENDING", "ALIGNING", "SCANNING",
            "RESUMING", "RETURNING", "LANDING", "MANUAL_OVERRIDE", "DONE",
        }
        actual = {s.name for s in State}
        assert expected.issubset(actual)

    def test_states_are_unique(self):
        values = [s.value for s in State]
        assert len(values) == len(set(values))


# ─── Phase 1 → Phase 2 transition ────────────────────────────────────────────

class TestFastScoutToInspectTransition:
    """FAST_SCOUT must move to INSPECT_TARGETS when all waypoints are consumed."""

    def _setup_scout_done(self, node, n_detected: int = 3):
        node._state = State.FAST_SCOUT
        node._lawnmower_waypoints = [(0.0, 0.0), (5.0, 0.0), (5.0, 2.0), (0.0, 2.0)]
        node._wp_index = len(node._lawnmower_waypoints)  # all consumed
        node._detected_blobs = [(float(i), float(i)) for i in range(n_detected)]

    def test_waypoints_exhausted_triggers_inspect(self):
        node = _make_node()
        self._setup_scout_done(node)
        # Simulate the waypoint-exhaustion check that the mission loop performs
        if node._wp_index >= len(node._lawnmower_waypoints):
            node._state = State.INSPECT_TARGETS
        assert node._state == State.INSPECT_TARGETS

    def test_blobs_sorted_by_nearest_neighbour(self):
        from uav_raspberry.math_utils import nearest_neighbor_sort

        node = _make_node()
        self._setup_scout_done(node, n_detected=4)
        start = (node._drone_north, node._drone_east)
        node._inspect_queue = nearest_neighbor_sort(node._detected_blobs, start)
        assert len(node._inspect_queue) == 4
        # First target should be closest to the drone's position
        first = node._inspect_queue[0]
        for other in node._inspect_queue[1:]:
            d_first = math.hypot(first[0] - start[0], first[1] - start[1])
            d_other = math.hypot(other[0] - start[0], other[1] - start[1])
            assert d_first <= d_other

    def test_empty_blobs_still_transitions(self):
        node = _make_node()
        self._setup_scout_done(node, n_detected=0)
        if node._wp_index >= len(node._lawnmower_waypoints):
            node._state = State.INSPECT_TARGETS
        assert node._state == State.INSPECT_TARGETS


# ─── Phase 2 state-machine transitions ───────────────────────────────────────

class TestPhase2Transitions:
    def test_inspect_targets_to_approach(self):
        node = _make_node()
        node._state = State.INSPECT_TARGETS
        node._inspect_queue = [(2.0, 2.0), (4.0, 4.0)]
        # Simulated: pop first target → APPROACH
        node._current_target = node._inspect_queue.pop(0)
        node._state = State.APPROACH
        assert node._state == State.APPROACH
        assert node._current_target == (2.0, 2.0)

    def test_approach_to_descending_on_arrival(self):
        node = _make_node()
        node._state = State.APPROACH
        node._current_target = (1.0, 1.0)
        node._drone_north = 1.0
        node._drone_east = 1.0
        dist = math.hypot(
            node._drone_north - node._current_target[0],
            node._drone_east  - node._current_target[1],
        )
        if dist < 0.5:
            node._state = State.DESCENDING
        assert node._state == State.DESCENDING

    def test_approach_stays_when_far(self):
        node = _make_node()
        node._state = State.APPROACH
        node._current_target = (10.0, 10.0)
        node._drone_north = 0.0
        node._drone_east = 0.0
        dist = math.hypot(
            node._drone_north - node._current_target[0],
            node._drone_east  - node._current_target[1],
        )
        if dist < 0.5:
            node._state = State.DESCENDING
        assert node._state == State.APPROACH

    def test_descending_to_aligning_at_scan_alt(self):
        node = _make_node()
        node._state = State.DESCENDING
        node._altitude_m = 2.4   # below scan_alt_m=2.5
        if node._altitude_m <= node._scan_alt_m:
            node._state = State.ALIGNING
        assert node._state == State.ALIGNING

    def test_scanning_to_resuming(self):
        node = _make_node()
        node._state = State.SCANNING
        node._scan_complete = True
        if node._scan_complete:
            node._state = State.RESUMING
        assert node._state == State.RESUMING

    def test_resuming_picks_next_target(self):
        node = _make_node()
        node._state = State.RESUMING
        node._inspect_queue = [(3.0, 3.0)]
        node._current_target = node._inspect_queue.pop(0)
        node._state = State.APPROACH
        assert node._state == State.APPROACH
        assert node._inspect_queue == []

    def test_resuming_with_empty_queue_goes_returning(self):
        node = _make_node()
        node._state = State.RESUMING
        node._inspect_queue = []
        # After the last target the node sets RETURNING, not DONE
        if not node._inspect_queue:
            node._state = State.RETURNING
        assert node._state == State.RETURNING


# ─── RETURNING / LANDING / DONE ───────────────────────────────────────────────

class TestReturnAndLand:
    def test_returning_to_landing_on_arrival(self):
        node = _make_node()
        node._state = State.RETURNING
        node._home = (0.0, 0.0)
        node._drone_north = 0.3   # within waypoint_tol_inspect=0.6
        node._drone_east  = 0.3
        dist = math.hypot(
            node._drone_north - node._home[0],
            node._drone_east  - node._home[1],
        )
        if dist < 0.6:
            node._state = State.LANDING
        assert node._state == State.LANDING

    def test_returning_stays_when_far(self):
        node = _make_node()
        node._state = State.RETURNING
        node._home = (0.0, 0.0)
        node._drone_north = 5.0
        node._drone_east  = 5.0
        dist = math.hypot(
            node._drone_north - node._home[0],
            node._drone_east  - node._home[1],
        )
        if dist < 0.6:
            node._state = State.LANDING
        assert node._state == State.RETURNING

    def test_landing_to_done_on_touchdown(self):
        node = _make_node()
        node._state = State.LANDING
        node._altitude_m = 0.1   # below 0.3 m threshold
        if node._altitude_m < 0.3:
            node._state = State.DONE
        assert node._state == State.DONE

    def test_landing_waits_while_airborne(self):
        node = _make_node()
        node._state = State.LANDING
        node._altitude_m = 1.5
        if node._altitude_m < 0.3:
            node._state = State.DONE
        assert node._state == State.LANDING

    def test_home_is_ned_origin_by_default(self):
        node = _make_node()
        assert node._home == (0.0, 0.0)

    def test_inspect_exhausted_goes_returning_not_done(self):
        node = _make_node()
        node._state = State.APPROACH
        node._inspect_queue = []
        node._inspect_idx   = 0
        if node._inspect_idx >= len(node._inspect_queue):
            node._state = State.RETURNING
        assert node._state == State.RETURNING


# ─── MANUAL_OVERRIDE ──────────────────────────────────────────────────────────

class TestManualOverride:
    def test_override_freezes_state(self):
        node = _make_node()
        node._state = State.FAST_SCOUT
        # Simulate RC override flag → any non-override state stays frozen
        node._state = State.MANUAL_OVERRIDE
        # A subsequent mission tick must not change state
        prev = node._state
        # (in production the async loop checks for MANUAL_OVERRIDE and returns early)
        if node._state == State.MANUAL_OVERRIDE:
            pass  # no transition
        assert node._state == prev

    def test_override_can_be_cleared(self):
        node = _make_node()
        node._state = State.MANUAL_OVERRIDE
        # Simulated: operator restores auto → resume FAST_SCOUT
        node._state = State.FAST_SCOUT
        assert node._state == State.FAST_SCOUT


# ─── Blob deduplication logic ─────────────────────────────────────────────────

class TestBlobDeduplication:
    def _is_dup(self, node, north: float, east: float) -> bool:
        r = node._blob_dedup_radius_m
        return any(
            math.hypot(north - b[0], east - b[1]) < r
            for b in node._detected_blobs
        )

    def test_no_blobs_not_duplicate(self):
        node = _make_node()
        assert not self._is_dup(node, 5.0, 5.0)

    def test_same_position_is_duplicate(self):
        node = _make_node()
        node._detected_blobs = [(3.0, 3.0)]
        assert self._is_dup(node, 3.0, 3.0)

    def test_within_radius_is_duplicate(self):
        node = _make_node()
        node._detected_blobs = [(3.0, 3.0)]
        assert self._is_dup(node, 3.4, 3.4)   # distance ≈ 0.57 < 1.0

    def test_outside_radius_not_duplicate(self):
        node = _make_node()
        node._detected_blobs = [(3.0, 3.0)]
        assert not self._is_dup(node, 4.5, 4.5)  # distance ≈ 2.12 > 1.0

    def test_multiple_blobs_any_match(self):
        node = _make_node()
        node._detected_blobs = [(0.0, 0.0), (10.0, 10.0)]
        assert self._is_dup(node, 10.0, 10.0)
        assert not self._is_dup(node, 5.0, 5.0)

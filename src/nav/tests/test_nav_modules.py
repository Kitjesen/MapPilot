"""Tests for nav/ modules — WaypointTracker, GlobalPlannerService,
OccupancyGridModule, ElevationMapModule, ESDFModule, SafetyRingModule.

All tests are pure-Python, no ROS2 / hardware / MuJoCo required.
"""

from __future__ import annotations

import json
import math
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path as FilePath
from unittest.mock import patch

import numpy as np
import pytest

pytestmark = [pytest.mark.sim]

_scipy_available = True
try:
    import scipy.ndimage  # noqa: F401 -- availability check for skipUnless
except ImportError:
    _scipy_available = False

from core.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from core.msgs.nav import OccupancyGrid, Odometry, Path
from core.msgs.sensor import PointCloud2
from core.runtime_interface import TOPICS, topic_default_frame_id

# ---------------------------------------------------------------------------
# 1. WaypointTracker
# ---------------------------------------------------------------------------
from nav.waypoint_tracker import (
    EV_PATH_COMPLETE,
    EV_STUCK,
    EV_STUCK_WARN,
    EV_WAYPOINT_REACHED,
    WaypointTracker,
)


class TestWaypointTracker(unittest.TestCase):
    """Tests for WaypointTracker (pure Python, not a Module)."""

    def test_path_complete(self):
        """Walk through 2 waypoints and reach the end."""
        tracker = WaypointTracker(threshold=1.0)
        wp0 = np.array([5.0, 0.0, 0.0])
        wp1 = np.array([10.0, 0.0, 0.0])
        tracker.reset([wp0, wp1], np.array([0.0, 0.0, 0.0]))

        # Move near first waypoint
        status = tracker.update(np.array([4.5, 0.0, 0.0]))
        self.assertEqual(status.event, EV_WAYPOINT_REACHED)
        self.assertEqual(status.wp_index, 1)

        # Move near second waypoint
        status = tracker.update(np.array([9.8, 0.0, 0.0]))
        self.assertEqual(status.event, EV_PATH_COMPLETE)
        self.assertEqual(status.wp_index, 2)

    def test_final_threshold_can_be_tighter_than_intermediate_threshold(self):
        tracker = WaypointTracker(threshold=1.0, final_threshold=0.2)
        wp0 = np.array([5.0, 0.0, 0.0])
        wp1 = np.array([10.0, 0.0, 0.0])
        tracker.reset([wp0, wp1], np.array([0.0, 0.0, 0.0]))

        status = tracker.update(np.array([4.5, 0.0, 0.0]))
        self.assertEqual(status.event, EV_WAYPOINT_REACHED)

        status = tracker.update(np.array([9.7, 0.0, 0.0]))
        self.assertIsNone(status.event)

        status = tracker.update(np.array([9.9, 0.0, 0.0]))
        self.assertEqual(status.event, EV_PATH_COMPLETE)

    def test_stuck_warn_at_half_timeout(self):
        """Not moving for >50% of timeout fires EV_STUCK_WARN once."""
        tracker = WaypointTracker(threshold=1.0, stuck_timeout=0.1, stuck_dist=0.15)
        wp = np.array([10.0, 0.0, 0.0])
        pos = np.array([0.0, 0.0, 0.0])
        tracker.reset([wp], pos)

        time.sleep(0.06)  # >50% of 0.1s
        status = tracker.update(pos)
        self.assertEqual(status.event, EV_STUCK_WARN)

        # Second call at same position should NOT re-fire warn
        status = tracker.update(pos)
        self.assertIsNone(status.event)

    def test_stuck_at_full_timeout(self):
        """Not moving for full timeout fires EV_STUCK once."""
        tracker = WaypointTracker(threshold=1.0, stuck_timeout=0.1, stuck_dist=0.15)
        wp = np.array([10.0, 0.0, 0.0])
        pos = np.array([0.0, 0.0, 0.0])
        tracker.reset([wp], pos)

        time.sleep(0.06)
        tracker.update(pos)  # warn fires here

        time.sleep(0.06)  # total >0.1s
        status = tracker.update(pos)
        self.assertEqual(status.event, EV_STUCK)

        # Should not re-fire
        status = tracker.update(pos)
        self.assertIsNone(status.event)

    def test_clear_discards_path(self):
        tracker = WaypointTracker()
        tracker.reset([np.array([1, 0, 0])], np.array([0, 0, 0]))
        self.assertTrue(tracker.has_path)
        tracker.clear()
        self.assertFalse(tracker.has_path)

    def test_pause_resets_stuck_timer(self):
        """Pause resets stuck timer so no stuck event fires immediately after."""
        tracker = WaypointTracker(threshold=1.0, stuck_timeout=0.1, stuck_dist=0.15)
        wp = np.array([10.0, 0.0, 0.0])
        pos = np.array([0.0, 0.0, 0.0])
        tracker.reset([wp], pos)

        time.sleep(0.08)  # past warn threshold
        tracker.pause()
        # Immediately after pause, no stuck event
        status = tracker.update(pos)
        self.assertIsNone(status.event)

    def test_movement_resets_stuck(self):
        """Moving >= stuck_dist resets stuck timer."""
        tracker = WaypointTracker(threshold=1.0, stuck_timeout=0.1, stuck_dist=0.15)
        wp = np.array([10.0, 0.0, 0.0])
        pos = np.array([0.0, 0.0, 0.0])
        tracker.reset([wp], pos)

        time.sleep(0.06)  # past 50% of timeout
        # Move far enough to reset
        moved = np.array([0.2, 0.0, 0.0])
        status = tracker.update(moved)
        self.assertIsNone(status.event)

        # Wait again — no warn because timer was reset
        time.sleep(0.04)
        status = tracker.update(moved)
        self.assertIsNone(status.event)

    def test_empty_path_no_event(self):
        """Update with no path returns no event."""
        tracker = WaypointTracker()
        status = tracker.update(np.array([0.0, 0.0, 0.0]))
        self.assertIsNone(status.event)
        self.assertEqual(status.wp_total, 0)

    def test_accepts_list_positions_from_runtime_status(self):
        """Runtime status paths can carry plain list positions."""
        tracker = WaypointTracker(threshold=0.1, stuck_dist=0.05)
        tracker.reset([[1.0, 0.0, 0.0]], [0.0, 0.0, 0.0])

        status = tracker.update([0.2, 0.0, 0.0])

        self.assertIsNone(status.event)
        self.assertEqual(status.wp_index, 0)

    def test_z_threshold_blocks_same_xy_different_floor(self):
        tracker = WaypointTracker(threshold=1.0, z_threshold=0.5)
        tracker.reset([np.array([5.0, 0.0, 3.0])], np.array([0.0, 0.0, 0.0]))

        status = tracker.update(np.array([5.0, 0.0, 0.0]))

        self.assertIsNone(status.event)
        self.assertEqual(status.wp_index, 0)

        status = tracker.update(np.array([5.0, 0.0, 2.75]))

        self.assertEqual(status.event, EV_PATH_COMPLETE)


# ---------------------------------------------------------------------------
# 2. GlobalPlannerService
# ---------------------------------------------------------------------------

from nav.global_planner_service import GlobalPlannerService


class _MockBackend:
    """Minimal planner backend for testing _find_safe_goal and _downsample."""

    def __init__(self, grid, resolution=0.2, origin=None):
        self._grid = grid
        self._resolution = resolution
        self._origin = origin if origin is not None else np.array([0.0, 0.0])

    def plan(self, start, goal):
        return [start, goal]

    def update_map(self, grid, **kw):
        self._grid = grid


class TestGlobalPlannerService(unittest.TestCase):
    """Tests for GlobalPlannerService (pure Python, not a Module)."""

    def _make_service(self, grid, resolution=0.2, origin=None):
        svc = GlobalPlannerService(planner_name="mock", downsample_dist=2.0)
        svc._backend = _MockBackend(grid, resolution, origin)
        return svc

    # -- _find_safe_goal --------------------------------------------------------

    def test_find_safe_goal_free_cell(self):
        """Goal on a free cell returns the same position."""
        grid = np.zeros((20, 20), dtype=np.int8)  # all free
        svc = self._make_service(grid, resolution=0.2)
        goal = np.array([1.0, 1.0, 0.0])
        result = svc._find_safe_goal(goal)
        self.assertIsNotNone(result)
        np.testing.assert_array_almost_equal(result[:2], goal[:2])

    def test_find_safe_goal_on_obstacle(self):
        """Goal on obstacle cell — BFS finds nearest free cell."""
        grid = np.zeros((20, 20), dtype=np.int8)
        # Mark goal cell (5,5) as obstacle
        grid[5, 5] = 100
        svc = self._make_service(grid, resolution=0.2)
        goal = np.array([1.0, 1.0, 0.0])  # maps to cell (5, 5)
        result = svc._find_safe_goal(goal)
        self.assertIsNotNone(result)
        # Result should differ from original goal (moved to free cell)
        self.assertFalse(
            np.array_equal(result[:2], goal[:2]),
            "Expected goal to be adjusted away from obstacle",
        )

    def test_find_safe_goal_no_map(self):
        """No backend returns None."""
        svc = GlobalPlannerService(planner_name="mock")
        svc._backend = None
        result = svc._find_safe_goal(np.array([0.0, 0.0, 0.0]))
        self.assertIsNone(result)

    def test_pct_saved_map_artifact_gate_uses_configured_expected_frame(self):
        from core.same_source_map_artifacts import (
            build_saved_map_metadata,
            sha256_file,
        )

        with tempfile.TemporaryDirectory() as temp_dir:
            artifact_dir = FilePath(temp_dir) / "same_source_map"
            artifact_dir.mkdir()
            map_pcd = artifact_dir / "map.pcd"
            tomogram = artifact_dir / "tomogram.pickle"
            map_pcd.write_text("VERSION 0.7\nDATA ascii\n", encoding="ascii")
            tomogram.write_bytes(b"tomogram")
            map_sha = sha256_file(map_pcd)
            metadata = build_saved_map_metadata(
                source_profile="mujoco_fastlio2_live_gate",
                data_source="fastlio2",
                slam_source="fastlio2",
                localization_source="fastlio2",
                mapping_source="/points_raw -> fastlio2 -> /nav/map_cloud",
                frame_id="odom",
                artifacts={
                    "map_pcd": {
                        "path": "map.pcd",
                        "sha256": map_sha,
                        "point_count": 1,
                        "source_profile": "mujoco_fastlio2_live_gate",
                        "data_source": "fastlio2",
                        "slam_source": "fastlio2",
                        "frame_id": "odom",
                    },
                    "tomogram": {
                        "path": "tomogram.pickle",
                        "sha256": sha256_file(tomogram),
                        "source_map_sha256": map_sha,
                        "source_profile": "mujoco_fastlio2_live_gate",
                        "data_source": "fastlio2",
                        "frame_id": "odom",
                        "shape": [1, 1],
                    },
                },
            )
            (artifact_dir / "metadata.json").write_text(
                json.dumps(metadata, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )

            svc = GlobalPlannerService(
                planner_name="pct",
                tomogram=str(tomogram),
                expected_saved_map_frame_id="odom",
            )

            gate = svc._validate_map_artifact_gate()

            self.assertTrue(gate["ok"], gate["blockers"])
            self.assertEqual(gate["expected_frame_id"], "odom")

    def test_navigation_module_can_separate_planning_and_saved_map_frames(self):
        from nav.navigation_module import NavigationModule

        module = NavigationModule(
            planner="pct",
            planning_frame_id="odom",
            expected_saved_map_frame_id="map",
        )

        self.assertEqual(module._planning_frame_id, "odom")
        self.assertEqual(module._planner_svc._expected_saved_map_frame_id, "map")

    # -- _downsample ------------------------------------------------------------

    def test_downsample_preserves_goal(self):
        """Last point of downsampled path is always the goal."""
        svc = GlobalPlannerService(downsample_dist=2.0)
        path = [np.array([0, 0, 0]), np.array([1, 0, 0]),
                np.array([2, 0, 0]), np.array([3, 0, 0]),
                np.array([5, 0, 0]), np.array([7, 0, 0])]
        goal = np.array([7.0, 0.0, 0.0])
        result = svc._downsample(path, goal)
        np.testing.assert_array_almost_equal(result[-1], goal)

    def test_downsample_keeps_precise_goal_near_grid_center(self):
        svc = GlobalPlannerService(downsample_dist=2.0)
        path = [np.array([0.0, 0.0, 0.0]), np.array([0.95, 0.0, 0.0])]
        goal = np.array([1.0, 0.0, 0.0])

        result = svc._downsample(path, goal)

        np.testing.assert_array_almost_equal(result[-1], goal)

    def test_downsample_spacing(self):
        """Consecutive points in downsampled path are >= downsample_dist apart
        (except possibly the last segment to the goal)."""
        svc = GlobalPlannerService(downsample_dist=2.0)
        path = [np.array([float(i), 0, 0]) for i in range(20)]
        goal = np.array([19.0, 0.0, 0.0])
        result = svc._downsample(path, goal)
        # All interior segments should be >= 2.0
        for i in range(1, len(result) - 1):
            dist = np.linalg.norm(result[i] - result[i - 1])
            self.assertGreaterEqual(dist, 2.0 - 1e-6,
                                    f"Segment {i-1}->{i} too short: {dist}")


# ---------------------------------------------------------------------------
# 3. PathFollowerModule
# ---------------------------------------------------------------------------

class _FakeNavCore:
    class Vec3:
        def __init__(self, x, y, z):
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)

    class Cmd:
        def __init__(self, vx, vy, wz):
            self.vx = float(vx)
            self.vy = float(vy)
            self.wz = float(wz)

    class ControlOut:
        def __init__(self, cmd):
            self.cmd = cmd

    class PathFollowerState:
        def __init__(self):
            self.vehicle_speed = 0.0
            self.nav_fwd = 0
            self.pathPointID = 0

    @staticmethod
    def compute_control(*_args):
        return _FakeNavCore.ControlOut(_FakeNavCore.Cmd(1.0, 0.0, 0.4))


class TestPathFollowerModule(unittest.TestCase):

    def test_nav_core_uses_configured_control_gains(self):
        from base_autonomy.modules import path_follower_module

        class FakeNavCoreWithParams(_FakeNavCore):
            class PathFollowerParams:
                pass

        original = path_follower_module.try_import_nav_core
        try:
            path_follower_module.try_import_nav_core = lambda *_args: FakeNavCoreWithParams
            module = path_follower_module.PathFollowerModule(
                backend="nav_core",
                yaw_rate_gain=2.5,
                stop_yaw_rate_gain=2.25,
                dir_diff_thre=0.35,
            )

            module._setup_nav_core()
        finally:
            path_follower_module.try_import_nav_core = original

        self.assertEqual(module._backend, "nav_core")
        self.assertAlmostEqual(module._nc_params.yaw_rate_gain, 2.5)
        self.assertAlmostEqual(module._nc_params.stop_yaw_rate_gain, 2.25)
        self.assertAlmostEqual(module._nc_params.dir_diff_thre, 0.35)
        health = module.health()["path_follower"]
        self.assertAlmostEqual(health["yaw_rate_gain"], 2.5)
        self.assertAlmostEqual(health["stop_yaw_rate_gain"], 2.25)
        self.assertAlmostEqual(health["dir_diff_thre"], 0.35)

    def test_nav_core_keeps_fixed_frame_paths_in_world_reference(self):
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        m = PathFollowerModule(backend="nav_core")
        m._nc = _FakeNavCore

        yaw = math.pi / 2.0
        m._on_odom(
            Odometry(
                pose=Pose(
                    position=Vector3(10.0, -2.0, 0.0),
                    orientation=Quaternion.from_yaw(yaw),
                ),
                frame_id="map",
            )
        )
        self.assertEqual(m._odom_frame_id, "map")

        path = Path(
            poses=[
                PoseStamped(pose=Pose(position=Vector3(10.0, -2.0, 0.5))),
                PoseStamped(pose=Pose(position=Vector3(10.0, 1.0, 1.5))),
            ],
            frame_id="map",
        )
        m._on_path(path)

        self.assertEqual(len(m._nc_path), 2)
        first, last = m._nc_path[0], m._nc_path[-1]
        self.assertAlmostEqual(first.x, 10.0)
        self.assertAlmostEqual(first.y, -2.0)
        self.assertAlmostEqual(first.z, 0.5)
        self.assertAlmostEqual(last.x, 10.0)
        self.assertAlmostEqual(last.y, 1.0)
        self.assertAlmostEqual(last.z, 1.5)
        self.assertEqual(m._x_rec, 0.0)
        self.assertEqual(m._y_rec, 0.0)
        self.assertEqual(m._yaw_rec, 0.0)

    def test_nav_core_normalizes_fixed_frame_path_id(self):
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        m = PathFollowerModule(backend="nav_core")
        m._nc = _FakeNavCore

        m._on_odom(
            Odometry(
                pose=Pose(position=Vector3(10.0, -2.0, 0.0)),
                frame_id="map",
            )
        )

        path = Path(
            poses=[
                PoseStamped(pose=Pose(position=Vector3(10.0, -2.0, 0.5))),
                PoseStamped(pose=Pose(position=Vector3(10.0, 1.0, 1.5))),
            ],
            frame_id="/map",
        )
        m._on_path(path)

        first = m._nc_path[0]
        self.assertAlmostEqual(first.x, 10.0)
        self.assertAlmostEqual(first.y, -2.0)
        self.assertEqual(m._x_rec, 0.0)
        self.assertEqual(m._y_rec, 0.0)
        self.assertEqual(m._yaw_rec, 0.0)

    def test_nav_core_keeps_simulator_world_path_in_fixed_reference(self):
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        m = PathFollowerModule(backend="nav_core")
        m._nc = _FakeNavCore

        m._on_odom(
            Odometry(
                pose=Pose(position=Vector3(10.0, -2.0, 0.0)),
                frame_id="map",
            )
        )

        path = Path(
            poses=[
                PoseStamped(pose=Pose(position=Vector3(10.0, -2.0, 0.5))),
                PoseStamped(pose=Pose(position=Vector3(10.0, 1.0, 1.5))),
            ],
            frame_id="world",
        )
        m._on_path(path)

        first = m._nc_path[0]
        self.assertAlmostEqual(first.x, 10.0)
        self.assertAlmostEqual(first.y, -2.0)
        self.assertEqual(m._x_rec, 0.0)
        self.assertEqual(m._y_rec, 0.0)
        self.assertEqual(m._yaw_rec, 0.0)

    def test_nav_core_records_body_frame_path_in_receipt_reference_frame(self):
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        m = PathFollowerModule(backend="nav_core")
        m._nc = _FakeNavCore

        yaw = math.pi / 2.0
        m._on_odom(
            Odometry(
                pose=Pose(
                    position=Vector3(10.0, -2.0, 0.0),
                    orientation=Quaternion.from_yaw(yaw),
                ),
                frame_id="map",
            )
        )

        path = Path(
            poses=[
                PoseStamped(pose=Pose(position=Vector3(10.0, -2.0, 0.5))),
                PoseStamped(pose=Pose(position=Vector3(10.0, 1.0, 1.5))),
            ],
            frame_id="base_link",
        )
        m._on_path(path)

        self.assertEqual(len(m._nc_path), 2)
        first, last = m._nc_path[0], m._nc_path[-1]
        self.assertAlmostEqual(first.x, 0.0)
        self.assertAlmostEqual(first.y, 0.0)
        self.assertAlmostEqual(first.z, 0.5)
        self.assertAlmostEqual(last.x, 3.0)
        self.assertAlmostEqual(last.y, 0.0)
        self.assertAlmostEqual(last.z, 1.5)
        self.assertEqual(m._x_rec, 10.0)
        self.assertEqual(m._y_rec, -2.0)
        self.assertAlmostEqual(m._yaw_rec, yaw)

    def test_nav_core_publishes_zero_when_path_is_cleared(self):
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        m = PathFollowerModule(backend="nav_core")
        m._nc = _FakeNavCore
        m._nc_params = object()
        m._nc_state = _FakeNavCore.PathFollowerState()
        outputs = []
        m.cmd_vel._add_callback(outputs.append)

        m._on_odom(Odometry(pose=Pose(position=Vector3(0.0, 0.0, 0.0))))
        m._on_path(Path(
            poses=[
                PoseStamped(pose=Pose(position=Vector3(0.0, 0.0, 0.0))),
                PoseStamped(pose=Pose(position=Vector3(2.0, 0.0, 0.0))),
            ],
            frame_id="map",
        ))
        m._on_odom(Odometry(pose=Pose(position=Vector3(0.1, 0.0, 0.0))))

        self.assertGreater(outputs[-1].linear.x, 0.0)
        self.assertGreater(outputs[-1].angular.z, 0.0)

        m._on_path(Path(poses=[], frame_id="map"))

        self.assertEqual(outputs[-1].linear.x, 0.0)
        self.assertEqual(outputs[-1].linear.y, 0.0)
        self.assertEqual(outputs[-1].angular.z, 0.0)
        self.assertEqual(m._smooth_vx, 0.0)
        self.assertEqual(m._smooth_wz, 0.0)
        self.assertEqual(m._nc_path, [])
        self.assertEqual(m._nc_state.vehicle_speed, 0.0)
        self.assertEqual(m._nc_state.nav_fwd, 0)
        self.assertEqual(m._nc_state.pathPointID, 0)

    def test_nav_core_resets_follower_state_when_new_path_arrives(self):
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        m = PathFollowerModule(backend="nav_core")
        m._nc = _FakeNavCore
        m._nc_params = object()
        m._nc_state = _FakeNavCore.PathFollowerState()
        m._nc_state.vehicle_speed = 1.7
        m._nc_state.nav_fwd = 1
        m._nc_state.pathPointID = 5

        m._on_odom(Odometry(pose=Pose(position=Vector3(1.0, 0.0, 0.0)), ts=1.0))
        m._on_path(Path(
            poses=[
                PoseStamped(pose=Pose(position=Vector3(1.0, 0.0, 0.0))),
                PoseStamped(pose=Pose(position=Vector3(2.0, 0.0, 0.0))),
            ],
            frame_id="map",
        ))

        self.assertEqual(m._nc_state.vehicle_speed, 0.0)
        self.assertEqual(m._nc_state.nav_fwd, 0)
        self.assertEqual(m._nc_state.pathPointID, 0)
        self.assertEqual(len(m._nc_path), 2)

    def test_nav_core_applies_local_planner_control_hint(self):
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        class RecordingNavCore(_FakeNavCore):
            last_slow_factor = None
            last_safety_stop = None

            @staticmethod
            def compute_control(*args):
                RecordingNavCore.last_slow_factor = args[5]
                RecordingNavCore.last_safety_stop = args[6]
                return _FakeNavCore.ControlOut(_FakeNavCore.Cmd(1.0, 0.0, 0.4))

        m = PathFollowerModule(backend="nav_core")
        m._nc = RecordingNavCore
        m._nc_params = object()
        m._nc_state = RecordingNavCore.PathFollowerState()
        outputs = []
        m.cmd_vel._add_callback(outputs.append)

        m._on_odom(Odometry(pose=Pose(position=Vector3(0.0, 0.0, 0.0)), ts=1.0))
        m._on_path(Path(
            poses=[
                PoseStamped(pose=Pose(position=Vector3(0.0, 0.0, 0.0))),
                PoseStamped(pose=Pose(position=Vector3(2.0, 0.0, 0.0))),
            ],
            frame_id="map",
        ))
        m._on_control_hint({"slow_down": 2, "ts": time.time(), "reason": "test"})
        m._on_odom(Odometry(pose=Pose(position=Vector3(0.1, 0.0, 0.0)), ts=1.1))

        self.assertEqual(RecordingNavCore.last_slow_factor, 0.40)
        self.assertEqual(RecordingNavCore.last_safety_stop, 0)
        self.assertGreater(outputs[-1].linear.x, 0.0)

        m._on_control_hint({
            "near_field_stop": True,
            "safety_stop": False,
            "ts": time.time(),
            "reason": "near_field_ignored",
        })
        m._on_odom(Odometry(pose=Pose(position=Vector3(0.2, 0.0, 0.0)), ts=1.2))

        self.assertEqual(RecordingNavCore.last_safety_stop, 0)
        self.assertFalse(m.health()["path_follower"]["control_hint"]["safety_stop"])
        self.assertGreater(outputs[-1].linear.x, 0.0)

        m._on_control_hint({
            "near_field_stop": True,
            "safety_stop": True,
            "ts": time.time(),
            "reason": "near_field",
        })

        self.assertEqual(outputs[-1].linear.x, 0.0)
        self.assertEqual(outputs[-1].angular.z, 0.0)
        self.assertTrue(m.health()["path_follower"]["control_hint"]["safety_stop"])

    def test_nav_core_resets_path_reference_on_map_frame_jump(self):
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        m = PathFollowerModule(backend="nav_core")
        m._nc = _FakeNavCore
        m._nc_params = object()
        m._nc_state = _FakeNavCore.PathFollowerState()
        outputs = []
        m.cmd_vel._add_callback(outputs.append)

        m._on_odom(Odometry(pose=Pose(position=Vector3(0.0, 0.0, 0.0))))
        m._on_path(Path(
            poses=[
                PoseStamped(pose=Pose(position=Vector3(0.0, 0.0, 0.0))),
                PoseStamped(pose=Pose(position=Vector3(2.0, 0.0, 0.0))),
            ],
            frame_id="map",
        ))
        m._on_odom(Odometry(pose=Pose(position=Vector3(0.1, 0.0, 0.0))))

        self.assertGreater(outputs[-1].linear.x, 0.0)
        m._nc_state.vehicle_speed = 2.0
        m._nc_state.nav_fwd = 1
        m._nc_state.pathPointID = 4

        m._on_map_frame_jump({"dt_m": 1.0, "dyaw_deg": 20.0})

        self.assertEqual(outputs[-1].linear.x, 0.0)
        self.assertEqual(outputs[-1].linear.y, 0.0)
        self.assertEqual(outputs[-1].angular.z, 0.0)
        self.assertEqual(m._nc_path, [])
        self.assertIsNone(m._path_points)
        self.assertEqual(m._x_rec, m._robot_x)
        self.assertEqual(m._y_rec, m._robot_y)
        self.assertEqual(m._nc_state.vehicle_speed, 0.0)
        self.assertEqual(m._nc_state.nav_fwd, 0)
        self.assertEqual(m._nc_state.pathPointID, 0)

    def test_pid_publishes_zero_when_path_is_cleared(self):
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        m = PathFollowerModule(backend="pid")
        outputs = []
        m.cmd_vel._add_callback(outputs.append)
        m._on_odom(Odometry(pose=Pose(position=Vector3(0.0, 0.0, 0.0))))

        m._on_path(Path(
            poses=[
                PoseStamped(pose=Pose(position=Vector3(0.0, 0.0, 0.0))),
                PoseStamped(pose=Pose(position=Vector3(2.0, 0.0, 0.0))),
            ],
            frame_id="map",
        ))
        self.assertGreater(outputs[-1].linear.x, 0.0)

        m._on_path(Path(poses=[PoseStamped(pose=Pose(position=Vector3(0.0, 0.0, 0.0)))], frame_id="map"))

        self.assertEqual(outputs[-1].linear.x, 0.0)
        self.assertEqual(outputs[-1].linear.y, 0.0)
        self.assertEqual(outputs[-1].angular.z, 0.0)
        self.assertEqual(m._smooth_vx, 0.0)
        self.assertEqual(m._smooth_wz, 0.0)
        self.assertIsNone(m._path_points)


# ---------------------------------------------------------------------------
# 4. OccupancyGridModule
# ---------------------------------------------------------------------------

from nav.occupancy_grid_module import OccupancyGridModule


@unittest.skipUnless(_scipy_available, "scipy not installed in this environment")
class TestOccupancyGridModule(unittest.TestCase):

    def _make_module(self, **kw):
        defaults = dict(
            resolution=0.2, map_radius=5.0, z_min=0.1, z_max=2.0,
            inflation_radius=0.0, robot_clear_radius=0.0, publish_hz=100.0,
        )
        defaults.update(kw)
        m = OccupancyGridModule(**defaults)
        m.setup()
        return m

    def test_default_frame_comes_from_exploration_grid_contract(self):
        m = self._make_module()

        self.assertEqual(m._frame_id, topic_default_frame_id(TOPICS.exploration_grid))

    def test_points_create_occupied_cells(self):
        """Deliver known XY points and verify occupied cells in costmap."""
        m = self._make_module()
        # Set robot position to origin
        odom = Odometry(pose=Pose(position=Vector3(0, 0, 0),
                                   orientation=Quaternion(0, 0, 0, 1)))
        m.odometry._deliver(odom)

        # 3 points at known positions, height within z_min/z_max
        pts = np.array([
            [1.0, 1.0, 0.5],
            [2.0, 2.0, 0.5],
            [3.0, 3.0, 0.5],
        ], dtype=np.float32)
        cloud = PointCloud2.from_numpy(pts, frame_id="map")

        results = []
        m.costmap._add_callback(lambda msg: results.append(msg))
        m.map_cloud._deliver(cloud)

        self.assertEqual(len(results), 1, "Expected one costmap publication")
        costmap = results[0]
        grid = costmap["grid"]
        # Verify at least some cells are occupied (100)
        self.assertTrue(np.any(grid == 100), "Expected occupied cells in costmap")
        self.assertEqual(costmap["frame_id"], topic_default_frame_id(TOPICS.exploration_grid))

    def test_projected_mode_accepts_default_robot_xy_before_odometry(self):
        """Point-cloud updates may arrive before the first odometry sample."""
        m = self._make_module()
        pts = np.array([
            [1.0, 0.0, 0.5],
            [1.5, 0.5, 0.5],
        ], dtype=np.float32)
        cloud = PointCloud2.from_numpy(pts, frame_id="map")

        results = []
        m.costmap._add_callback(lambda msg: results.append(msg))
        m.map_cloud._deliver(cloud)

        self.assertEqual(len(results), 1)
        self.assertTrue(np.any(results[0]["grid"] == 100))

    def test_height_filter(self):
        """Points outside z_min/z_max are ignored."""
        m = self._make_module(z_min=0.5, z_max=1.5)
        odom = Odometry(pose=Pose(position=Vector3(0, 0, 0),
                                   orientation=Quaternion(0, 0, 0, 1)))
        m.odometry._deliver(odom)

        # All points below z_min
        pts = np.array([
            [1.0, 1.0, 0.1],
            [2.0, 2.0, 0.2],
        ], dtype=np.float32)
        cloud = PointCloud2.from_numpy(pts, frame_id="map")

        results = []
        m.costmap._add_callback(lambda msg: results.append(msg))
        m.map_cloud._deliver(cloud)

        # Should not publish because all points are filtered
        self.assertEqual(len(results), 0,
                         "Expected no costmap when all points are height-filtered")

    def test_robot_clear_radius(self):
        """Points within robot_clear_radius of robot are cleared."""
        m = self._make_module(robot_clear_radius=2.0, inflation_radius=0.0)
        odom = Odometry(pose=Pose(position=Vector3(0, 0, 0),
                                   orientation=Quaternion(0, 0, 0, 1)))
        m.odometry._deliver(odom)

        # Points very close to robot — within clear radius
        pts = np.array([
            [0.1, 0.1, 0.5],
            [0.2, 0.2, 0.5],
        ], dtype=np.float32)
        cloud = PointCloud2.from_numpy(pts, frame_id="map")

        results = []
        m.costmap._add_callback(lambda msg: results.append(msg))
        m.map_cloud._deliver(cloud)

        # Points are within robot_clear_radius, so they get filtered
        # (the _on_cloud method filters near-body points before binning)
        self.assertEqual(len(results), 0,
                         "Expected no costmap when all points are within robot clear radius")

    def test_robot_clear_radius_keeps_points_outside_clear_disk(self):
        """Points outside robot_clear_radius remain available for occupancy."""
        m = self._make_module(robot_clear_radius=2.0, inflation_radius=0.0)
        odom = Odometry(pose=Pose(position=Vector3(0, 0, 0),
                                   orientation=Quaternion(0, 0, 0, 1)))
        m.odometry._deliver(odom)

        pts = np.array([[2.2, 0.0, 0.5]], dtype=np.float32)
        cloud = PointCloud2.from_numpy(pts, frame_id="map")

        results = []
        m.costmap._add_callback(lambda msg: results.append(msg))
        m.map_cloud._deliver(cloud)

        self.assertEqual(len(results), 1)
        self.assertTrue(np.any(results[0]["grid"] == 100),
                        "Expected outside-clear-radius point to stay occupied")

    def test_raycast_mode_publishes_exploration_unknown_free_occupied_grid(self):
        """Raycast mode preserves unknown cells for real frontier exploration."""
        m = self._make_module(
            resolution=0.5,
            map_radius=4.0,
            robot_clear_radius=0.0,
            inflation_radius=0.0,
            raycast_free_space=True,
            unknown_as_obstacle_for_costmap=True,
            raycast_max_rays=100,
        )
        odom = Odometry(pose=Pose(position=Vector3(0, 0, 0),
                                   orientation=Quaternion(0, 0, 0, 1)),
                        frame_id="map")
        m.odometry._deliver(odom)

        pts = np.array([
            [2.0, 0.0, 0.6],
            [2.0, 1.0, 0.6],
            [1.5, -1.0, 0.6],
        ], dtype=np.float32)
        cloud = PointCloud2.from_numpy(pts, frame_id="map")

        costmaps = []
        exploration = []
        m.costmap._add_callback(lambda msg: costmaps.append(msg))
        m.exploration_grid._add_callback(lambda msg: exploration.append(msg))
        m.map_cloud._deliver(cloud)

        self.assertEqual(len(costmaps), 1)
        self.assertEqual(len(exploration), 1)
        emap = exploration[0]
        counts = emap["counts"]
        self.assertGreater(counts["unknown"], 0)
        self.assertGreater(counts["free"], 0)
        self.assertGreater(counts["occupied"], 0)
        self.assertTrue(np.any(emap["grid"] < 0), "Exploration grid must keep unknown cells")
        self.assertTrue(np.any(emap["grid"] == 0), "Exploration grid must contain free cells")
        self.assertTrue(np.any(emap["grid"] == 100), "Exploration grid must contain occupied cells")
        self.assertFalse(np.any(costmaps[0]["grid"] < 0), "Navigation costmap must not contain unknown negatives")
        self.assertEqual(emap["frame_id"], topic_default_frame_id(TOPICS.exploration_grid))
        self.assertEqual(emap["accumulation"], "rolling_local_window")
        self.assertEqual(emap["semantic"], "frontier_input_grid")
        self.assertEqual(costmaps[0]["unknown_as_obstacle"], True)

    def test_raycast_mode_accepts_default_robot_xy_before_odometry(self):
        """Raycast mode should also tolerate point clouds before odometry."""
        m = self._make_module(
            resolution=0.5,
            map_radius=4.0,
            robot_clear_radius=0.0,
            inflation_radius=0.0,
            raycast_free_space=True,
            unknown_as_obstacle_for_costmap=True,
            raycast_max_rays=100,
        )
        pts = np.array([
            [2.0, 0.0, 0.6],
            [2.0, 1.0, 0.6],
            [1.5, -1.0, 0.6],
        ], dtype=np.float32)
        cloud = PointCloud2.from_numpy(pts, frame_id="map")

        exploration = []
        m.exploration_grid._add_callback(lambda msg: exploration.append(msg))
        m.map_cloud._deliver(cloud)

        self.assertEqual(len(exploration), 1)
        self.assertGreater(exploration[0]["counts"]["free"], 0)
        self.assertGreater(exploration[0]["counts"]["occupied"], 0)

    def test_raycast_free_inflation_widens_observed_free_corridors(self):
        def free_count(radius: float) -> int:
            m = self._make_module(
                resolution=0.5,
                map_radius=4.0,
                robot_clear_radius=0.0,
                inflation_radius=0.0,
                raycast_free_space=True,
                unknown_as_obstacle_for_costmap=True,
                raycast_max_rays=100,
                raycast_free_inflation_radius=radius,
            )
            odom = Odometry(pose=Pose(position=Vector3(0, 0, 0),
                                       orientation=Quaternion(0, 0, 0, 1)),
                            frame_id="map")
            m.odometry._deliver(odom)
            pts = np.array([
                [2.0, 0.0, 0.6],
                [2.0, 1.0, 0.6],
                [1.5, -1.0, 0.6],
            ], dtype=np.float32)
            cloud = PointCloud2.from_numpy(pts, frame_id="map")
            exploration = []
            m.exploration_grid._add_callback(lambda msg: exploration.append(msg))
            m.map_cloud._deliver(cloud)
            self.assertEqual(len(exploration), 1)
            self.assertGreater(exploration[0]["counts"]["occupied"], 0)
            self.assertGreater(exploration[0]["counts"]["unknown"], 0)
            return int(exploration[0]["counts"]["free"])

        self.assertGreater(free_count(0.5), free_count(0.0))


class _FakeROSHeader:
    def __init__(self):
        self.frame_id = ""
        self.stamp = None


class _FakeROSPoint:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _FakeROSQuaternion:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class _FakeROSPose:
    def __init__(self):
        self.position = _FakeROSPoint()
        self.orientation = _FakeROSQuaternion()


class _FakeROSPoseStamped:
    def __init__(self):
        self.header = _FakeROSHeader()
        self.pose = _FakeROSPose()


class _FakeROSPath:
    def __init__(self):
        self.header = _FakeROSHeader()
        self.poses = []


class _FakeROSMapMetaData:
    def __init__(self):
        self.resolution = 0.0
        self.width = 0
        self.height = 0
        self.origin = _FakeROSPose()


class _FakeROSOccupancyGrid:
    def __init__(self):
        self.header = _FakeROSHeader()
        self.info = _FakeROSMapMetaData()
        self.data = []


@pytest.mark.ros2
class TestROS2PathBridgeModule(unittest.TestCase):
    def test_defaults_come_from_runtime_topic_contract(self):
        from compat.ros2.nav.path_bridge import ROS2PathBridgeModule

        bridge = ROS2PathBridgeModule()

        self.assertEqual(bridge._global_path_topic, TOPICS.global_path)
        self.assertEqual(bridge._local_path_topic, TOPICS.local_path)
        self.assertEqual(
            bridge._global_default_frame_id,
            topic_default_frame_id(TOPICS.global_path),
        )
        self.assertEqual(
            bridge._local_default_frame_id,
            topic_default_frame_id(TOPICS.local_path),
        )

    def test_explicit_default_frame_applies_to_both_path_topics(self):
        from compat.ros2.nav.path_bridge import ROS2PathBridgeModule

        bridge = ROS2PathBridgeModule(default_frame_id="odom")

        self.assertEqual(bridge._global_default_frame_id, "odom")
        self.assertEqual(bridge._local_default_frame_id, "odom")

    def test_global_path_conversion_uses_global_topic_frame_default(self):
        from compat.ros2.nav.path_bridge import ROS2PathBridgeModule

        fake_nav_msgs = types.SimpleNamespace(Path=_FakeROSPath)
        fake_geometry_msgs = types.SimpleNamespace(PoseStamped=_FakeROSPoseStamped)
        fake_modules = {
            "nav_msgs": types.SimpleNamespace(msg=fake_nav_msgs),
            "nav_msgs.msg": fake_nav_msgs,
            "geometry_msgs": types.SimpleNamespace(msg=fake_geometry_msgs),
            "geometry_msgs.msg": fake_geometry_msgs,
        }
        bridge = ROS2PathBridgeModule()

        with patch.dict(sys.modules, fake_modules):
            msg = bridge._to_ros_path(
                [(1.0, 2.0, 0.0)],
                default_frame_id=bridge._global_default_frame_id,
            )

        self.assertEqual(msg.header.frame_id, topic_default_frame_id(TOPICS.global_path))
        self.assertEqual(msg.poses[0].header.frame_id, msg.header.frame_id)
        self.assertEqual(msg.poses[0].pose.position.x, 1.0)
        self.assertEqual(msg.poses[0].pose.position.y, 2.0)


@pytest.mark.ros2
class TestROS2GridBridgeModule(unittest.TestCase):
    def test_default_frame_comes_from_exploration_grid_contract(self):
        from compat.ros2.nav.grid_bridge import ROS2GridBridgeModule

        bridge = ROS2GridBridgeModule()

        self.assertEqual(bridge._exploration_grid_topic, TOPICS.exploration_grid)
        self.assertEqual(
            bridge._default_frame_id,
            topic_default_frame_id(TOPICS.exploration_grid),
        )

    def test_grid_conversion_uses_contract_frame_when_input_lacks_frame(self):
        from compat.ros2.nav.grid_bridge import ROS2GridBridgeModule

        fake_nav_msgs = types.SimpleNamespace(OccupancyGrid=_FakeROSOccupancyGrid)
        fake_modules = {
            "nav_msgs": types.SimpleNamespace(msg=fake_nav_msgs),
            "nav_msgs.msg": fake_nav_msgs,
        }
        bridge = ROS2GridBridgeModule()
        grid = {
            "grid": np.array([[0, 100]], dtype=np.int16),
            "resolution": 0.5,
            "origin": (1.0, 2.0),
        }

        with patch.dict(sys.modules, fake_modules):
            msg = bridge._to_ros_grid(grid)

        self.assertEqual(
            msg.header.frame_id,
            topic_default_frame_id(TOPICS.exploration_grid),
        )
        self.assertEqual(msg.info.width, 2)
        self.assertEqual(msg.info.height, 1)
        self.assertEqual(msg.info.origin.position.x, 1.0)
        self.assertEqual(msg.info.origin.position.y, 2.0)
        self.assertEqual(msg.data, [0, 100])

    def test_grid_conversion_preserves_explicit_input_frame(self):
        from compat.ros2.nav.grid_bridge import ROS2GridBridgeModule

        fake_nav_msgs = types.SimpleNamespace(OccupancyGrid=_FakeROSOccupancyGrid)
        fake_modules = {
            "nav_msgs": types.SimpleNamespace(msg=fake_nav_msgs),
            "nav_msgs.msg": fake_nav_msgs,
        }
        bridge = ROS2GridBridgeModule()
        grid = {
            "grid": np.array([[0]], dtype=np.int16),
            "frame_id": "odom",
        }

        with patch.dict(sys.modules, fake_modules):
            msg = bridge._to_ros_grid(grid)

        self.assertEqual(msg.header.frame_id, "odom")


# ---------------------------------------------------------------------------
# 4. VoxelGridModule
# ---------------------------------------------------------------------------

from nav.voxel_grid_module import VoxelGridModule


class TestVoxelGridModule(unittest.TestCase):

    def _make_module(self, **kw):
        defaults = dict(
            voxel_size=1.0,
            max_range=10.0,
            min_z=-1.0,
            max_z=2.0,
            decay_rate=0.0,
            publish_interval=999.0,
        )
        defaults.update(kw)
        m = VoxelGridModule(**defaults)
        m.setup()
        return m

    def test_cloud_update_accumulates_unique_voxel_counts_and_filters(self):
        m = self._make_module()
        m.odometry._deliver(
            Odometry(
                pose=Pose(
                    position=Vector3(0, 0, 0),
                    orientation=Quaternion(0, 0, 0, 1),
                )
            )
        )
        pts = np.array(
            [
                [0.1, 0.1, 0.1],
                [0.2, 0.2, 0.2],
                [1.2, 0.1, 0.1],
                [20.0, 0.0, 0.0],
                [0.0, 0.0, 3.0],
            ],
            dtype=np.float32,
        )

        m.map_cloud._deliver(PointCloud2.from_numpy(pts, frame_id="map"))

        self.assertEqual(json.loads(m.query_voxel(0.1, 0.1, 0.1))["count"], 2.0)
        self.assertEqual(json.loads(m.query_voxel(1.2, 0.1, 0.1))["count"], 1.0)
        self.assertFalse(json.loads(m.query_voxel(20.0, 0.0, 0.0))["occupied"])
        self.assertFalse(json.loads(m.query_voxel(0.0, 0.0, 3.0))["occupied"])

    def test_duplicate_heavy_cloud_updates_one_voxel_without_python_point_loop(self):
        m = self._make_module()
        pts = np.repeat(
            np.array([[0.25, 0.25, 0.25]], dtype=np.float32),
            repeats=1000,
            axis=0,
        )

        m.map_cloud._deliver(PointCloud2.from_numpy(pts, frame_id="map"))

        stats = json.loads(m.get_voxel_stats())
        self.assertEqual(stats["total_voxels"], 1)
        self.assertEqual(json.loads(m.query_voxel(0.25, 0.25, 0.25))["count"], 1000.0)

    def test_decay_publish_prunes_and_reports_columns(self):
        m = self._make_module(decay_rate=0.5)
        with m._lock:
            m._voxels = {
                (0, 0, 0): 4.0,
                (0, 0, 1): 2.0,
                (1, 1, 0): 1.0,
            }
        stats_payloads = []
        cloud_payloads = []
        m.voxel_map._add_callback(stats_payloads.append)
        m.voxel_cloud._add_callback(cloud_payloads.append)

        m._decay_and_publish()

        self.assertEqual(len(stats_payloads), 1)
        self.assertEqual(len(cloud_payloads), 1)
        self.assertEqual(stats_payloads[0]["total_voxels"], 2)
        self.assertEqual(stats_payloads[0]["column_count"], 1)
        self.assertEqual(stats_payloads[0]["frame_id"], topic_default_frame_id(TOPICS.map_cloud))
        self.assertEqual(cloud_payloads[0].points.shape, (2, 3))
        self.assertEqual(cloud_payloads[0].frame_id, topic_default_frame_id(TOPICS.map_cloud))
        self.assertEqual(json.loads(m.query_voxel(1.1, 1.1, 0.1))["count"], 0.0)

    def test_voxel_publish_uses_normalized_input_cloud_frame(self):
        m = self._make_module(publish_interval=0.0)
        stats_payloads = []
        cloud_payloads = []
        m.voxel_map._add_callback(stats_payloads.append)
        m.voxel_cloud._add_callback(cloud_payloads.append)

        pts = np.array([[0.25, 0.25, 0.25]], dtype=np.float32)
        m.map_cloud._deliver(PointCloud2.from_numpy(pts, frame_id="/odom"))

        self.assertEqual(len(stats_payloads), 1)
        self.assertEqual(len(cloud_payloads), 1)
        self.assertEqual(stats_payloads[0]["frame_id"], "odom")
        self.assertEqual(cloud_payloads[0].frame_id, "odom")


# ---------------------------------------------------------------------------
# 5. ElevationMapModule
# ---------------------------------------------------------------------------

from nav.elevation_map_module import ElevationMapModule


class TestElevationMapModule(unittest.TestCase):

    def _make_module(self, **kw):
        defaults = dict(
            resolution=1.0, map_radius=5.0, z_floor=-10.0, z_ceil=10.0,
            publish_hz=100.0,
        )
        defaults.update(kw)
        m = ElevationMapModule(**defaults)
        m.setup()
        return m

    def test_elevation_min_max(self):
        """Two points at same XY but different Z produce correct min_z/max_z."""
        m = self._make_module()
        odom = Odometry(pose=Pose(position=Vector3(0, 0, 0),
                                   orientation=Quaternion(0, 0, 0, 1)))
        m.odometry._deliver(odom)

        # Two points at x=2, y=2 with z=1.0 and z=3.0
        pts = np.array([
            [2.0, 2.0, 1.0],
            [2.0, 2.0, 3.0],
        ], dtype=np.float32)
        cloud = PointCloud2.from_numpy(pts, frame_id="map")

        results = []
        m.elevation_map._add_callback(lambda msg: results.append(msg))
        m.map_cloud._deliver(cloud)

        self.assertEqual(len(results), 1)
        emap = results[0]
        self.assertEqual(emap["frame_id"], topic_default_frame_id(TOPICS.map_cloud))

        # Find the cell that got hit
        valid = emap["valid"]
        self.assertTrue(np.any(valid), "Expected at least one valid cell")

        # Among valid cells, check min and max
        min_z = emap["min_z"]
        max_z = emap["max_z"]
        valid_min = np.nanmin(min_z[valid])
        valid_max = np.nanmax(max_z[valid])
        self.assertAlmostEqual(valid_min, 1.0, places=1)
        self.assertAlmostEqual(valid_max, 3.0, places=1)

    def test_height_filter_applied(self):
        """Points below z_floor are filtered out."""
        m = self._make_module(z_floor=0.0, z_ceil=10.0)
        odom = Odometry(pose=Pose(position=Vector3(0, 0, 0),
                                   orientation=Quaternion(0, 0, 0, 1)))
        m.odometry._deliver(odom)

        # All points below z_floor
        pts = np.array([
            [1.0, 1.0, -1.0],
            [2.0, 2.0, -2.0],
        ], dtype=np.float32)
        cloud = PointCloud2.from_numpy(pts, frame_id="map")

        results = []
        m.elevation_map._add_callback(lambda msg: results.append(msg))
        m.map_cloud._deliver(cloud)

        # Should not publish because all points are filtered
        self.assertEqual(len(results), 0,
                         "Expected no elevation map when all points are below z_floor")

    def test_elevation_map_uses_normalized_input_cloud_frame(self):
        m = self._make_module()
        m.odometry._deliver(
            Odometry(
                pose=Pose(
                    position=Vector3(0, 0, 0),
                    orientation=Quaternion(0, 0, 0, 1),
                )
            )
        )
        pts = np.array([[1.0, 1.0, 0.5]], dtype=np.float32)
        cloud = PointCloud2.from_numpy(pts, frame_id="/odom")

        results = []
        m.elevation_map._add_callback(results.append)
        m.map_cloud._deliver(cloud)

        self.assertEqual(len(results), 1)
        self.assertEqual(results[0]["frame_id"], "odom")


# ---------------------------------------------------------------------------
# 6. ESDFModule
# ---------------------------------------------------------------------------

from nav.esdf_module import ESDFModule


class TestESDFModule(unittest.TestCase):

    def _make_module(self, **kw):
        defaults = dict(obstacle_threshold=50, publish_hz=100.0)
        defaults.update(kw)
        m = ESDFModule(**defaults)
        m.setup()
        return m

    def test_esdf_positive_in_free_space(self):
        """Free cells have positive distance to obstacle."""
        pytest = __import__("pytest")
        pytest.importorskip("scipy")

        m = self._make_module()

        # 10x10 grid, one obstacle at center
        grid = np.zeros((10, 10), dtype=np.int8)
        grid[5, 5] = 100  # obstacle
        origin = Pose(position=Vector3(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))
        og = OccupancyGrid(grid=grid, resolution=0.2, origin=origin, frame_id="map")

        results = []
        m.esdf._add_callback(lambda msg: results.append(msg))
        m.occupancy_grid._deliver(og)

        self.assertEqual(len(results), 1)
        sdf = results[0]["distance_field"]
        # Free cell far from obstacle should have positive distance
        self.assertGreater(sdf[0, 0], 0.0,
                           "Free cell should have positive ESDF distance")

    def test_esdf_negative_in_obstacle(self):
        """Obstacle cells have negative (or zero) distance."""
        pytest = __import__("pytest")
        pytest.importorskip("scipy")

        m = self._make_module()

        # 10x10 grid, large obstacle block
        grid = np.zeros((10, 10), dtype=np.int8)
        grid[3:7, 3:7] = 100  # 4x4 obstacle
        origin = Pose(position=Vector3(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))
        og = OccupancyGrid(grid=grid, resolution=0.2, origin=origin, frame_id="map")

        results = []
        m.esdf._add_callback(lambda msg: results.append(msg))
        m.occupancy_grid._deliver(og)

        self.assertEqual(len(results), 1)
        sdf = results[0]["distance_field"]
        # Center of obstacle block should have negative distance
        self.assertLess(sdf[5, 5], 0.0,
                        "Interior obstacle cell should have negative ESDF distance")


# ---------------------------------------------------------------------------
# 7. SafetyRingModule
# ---------------------------------------------------------------------------

from nav.safety_ring_module import SafetyRingModule


class TestSafetyRingModule(unittest.TestCase):

    def _make_module(self, **kw):
        defaults = dict(odom_timeout_ms=100.0, cmd_vel_timeout_ms=100.0)
        defaults.update(kw)
        m = SafetyRingModule(**defaults)
        m.setup()
        return m

    def _make_odom(self, x=0.0, y=0.0, vx=0.0, vy=0.0):
        return Odometry(
            pose=Pose(position=Vector3(x, y, 0), orientation=Quaternion(0, 0, 0, 1)),
            twist=Twist(linear=Vector3(vx, vy, 0), angular=Vector3(0, 0, 0)),
        )

    def test_odom_timeout_triggers_stop(self):
        """No odom for longer than timeout triggers stop_cmd == 2."""
        m = self._make_module(odom_timeout_ms=50.0)

        # Deliver one odom to initialize
        m.odometry._deliver(self._make_odom())

        stop_cmds = []
        m.stop_cmd._add_callback(lambda v: stop_cmds.append(v))

        time.sleep(0.06)  # exceed 50ms timeout

        # Deliver another odom — this triggers _publish_safety which
        # checks odom age. But the odom delivery itself updates _last_odom_time.
        # So we need to trigger a check WITHOUT delivering odom.
        # Deliver localization_status to trigger _publish_safety.
        m.localization_status._deliver({"state": "OK"})

        # After timeout without odom, should have published stop_cmd 2
        # But delivering localization_status calls _publish_safety which
        # will check odom timeout. However, _last_odom_time was set by
        # the first odom delivery. Let's wait and trigger via localization.
        time.sleep(0.06)
        m.localization_status._deliver({"state": "OK"})

        self.assertIn(2, stop_cmds,
                      f"Expected stop_cmd=2 for odom timeout, got {stop_cmds}")

    def test_safety_ring_times_out_without_new_callbacks(self):
        """The watchdog must fail closed even when no input callback fires."""
        m = SafetyRingModule(odom_timeout_ms=40, cmd_vel_timeout_ms=40)
        stops = []
        m.stop_cmd._add_callback(stops.append)
        m.setup()
        m.start()
        try:
            m._on_odom(self._make_odom())
            stops.clear()
            time.sleep(0.15)
            self.assertIn(2, stops)
        finally:
            m.stop()

    def test_initial_state_without_odom_is_stop(self):
        """Before the first odom sample, safety must fail closed."""
        m = self._make_module(odom_timeout_ms=500.0)

        stop_cmds = []
        m.stop_cmd._add_callback(stop_cmds.append)

        m._publish_safety()

        self.assertEqual(stop_cmds[-1], 2)

    def test_stop_cmd_suppresses_reentrant_same_level_publish(self):
        """A STOP callback that re-enters safety evaluation must not recurse forever."""
        m = self._make_module()
        m.odometry._deliver(self._make_odom())
        m.localization_status._deliver({"state": "LOST"})

        stop_cmds = []

        def reenter_once(level):
            stop_cmds.append(level)
            if len(stop_cmds) == 1:
                m._publish_safety()

        m.stop_cmd._add_callback(reenter_once)

        m._publish_safety()

        self.assertEqual(stop_cmds, [2])

    def test_stop_cmd_republishes_current_stop_level_after_delivery_returns(self):
        """Repeated non-reentrant STOP checks may resend STOP to restarted downstreams."""
        m = self._make_module()
        m.odometry._deliver(self._make_odom())

        stop_cmds = []
        m.stop_cmd._add_callback(stop_cmds.append)

        m.localization_status._deliver({"state": "LOST"})
        m.localization_status._deliver({"state": "LOST"})

        self.assertGreaterEqual(stop_cmds.count(2), 2)

    def test_localization_lost_triggers_stop(self):
        """Localization state LOST triggers stop_cmd == 2."""
        m = self._make_module()

        # Keep odom alive
        m.odometry._deliver(self._make_odom())

        stop_cmds = []
        m.stop_cmd._add_callback(lambda v: stop_cmds.append(v))

        # Report localization lost
        m.localization_status._deliver({"state": "LOST"})

        self.assertIn(2, stop_cmds,
                      f"Expected stop_cmd=2 for LOST localization, got {stop_cmds}")

    def test_cross_track_error_computation(self):
        """Set a path, deliver odom off-path, and verify CTE in ExecutionEval."""
        m = self._make_module()

        # Set a straight path along X axis from (0,0) to (10,0)
        poses = [
            PoseStamped(pose=Pose(position=Vector3(0, 0, 0))),
            PoseStamped(pose=Pose(position=Vector3(10, 0, 0))),
        ]
        path_msg = Path(poses=poses, frame_id="map")
        m.path._deliver(path_msg)

        evals = []
        m.execution_eval._add_callback(lambda ev: evals.append(ev))

        # Robot is 3m off the path (at y=3)
        m.odometry._deliver(self._make_odom(x=5.0, y=3.0))

        self.assertGreater(len(evals), 0, "Expected at least one ExecutionEval")
        cte = evals[-1].cross_track_error
        self.assertAlmostEqual(cte, 3.0, places=1,
                               msg=f"Expected CTE ~3.0, got {cte}")

    def test_safe_state_when_healthy(self):
        """Regular odom delivery keeps stop_cmd == 0."""
        m = self._make_module(odom_timeout_ms=500.0, cmd_vel_timeout_ms=500.0)

        stop_cmds = []
        m.stop_cmd._add_callback(lambda v: stop_cmds.append(v))

        # Deliver odom and cmd_vel to keep links alive
        m.odometry._deliver(self._make_odom(vx=0.5))
        m.cmd_vel._deliver(Twist(linear=Vector3(0.5, 0, 0), angular=Vector3(0, 0, 0)))
        m.odometry._deliver(self._make_odom(vx=0.5))

        # The only stop_cmd published should be 0 (SAFE)
        if stop_cmds:
            self.assertEqual(stop_cmds[-1], 0,
                             f"Expected stop_cmd=0, got {stop_cmds[-1]}")

    def test_idle_without_cmd_vel_remains_safe(self):
        """Idle robot should not warn only because no command stream is active."""
        m = self._make_module(odom_timeout_ms=500.0, cmd_vel_timeout_ms=50.0)

        safety_states = []
        stop_cmds = []
        m.safety_state._add_callback(lambda s: safety_states.append(s))
        m.stop_cmd._add_callback(lambda v: stop_cmds.append(v))

        time.sleep(0.06)
        m.odometry._deliver(self._make_odom())

        self.assertGreater(len(safety_states), 0)
        self.assertEqual(safety_states[-1].level, 0)
        self.assertNotIn(1, stop_cmds)

    def test_active_path_without_cmd_vel_warns(self):
        """A stale command stream still warns while a path is active."""
        m = self._make_module(odom_timeout_ms=500.0, cmd_vel_timeout_ms=50.0)

        poses = [
            PoseStamped(pose=Pose(position=Vector3(0, 0, 0))),
            PoseStamped(pose=Pose(position=Vector3(1, 0, 0))),
        ]
        m.path._deliver(Path(poses=poses, frame_id="map"))

        safety_states = []
        stop_cmds = []
        m.safety_state._add_callback(lambda s: safety_states.append(s))
        m.stop_cmd._add_callback(lambda v: stop_cmds.append(v))

        time.sleep(0.06)
        m.odometry._deliver(self._make_odom())

        self.assertGreater(len(safety_states), 0)
        self.assertEqual(safety_states[-1].level, 1)
        self.assertIn(1, stop_cmds)


if __name__ == "__main__":
    unittest.main()

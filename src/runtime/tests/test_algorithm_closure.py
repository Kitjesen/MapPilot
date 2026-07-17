from __future__ import annotations

import json
import time

import numpy as np
import pytest

from decision.modules.visual_servo import (
    MODE_FIND,
    VisualServoModule,
)
from decision.vision.bbox import STATE_TRACKING
from maps.modules.traversability import TraversabilityCostModule
from nav.localization_monitor.monitor_module import LocalizationMonitorModule
from nav.navigation import Navigation
from nav.services.plan.global_planner.service import GlobalPlanner
from nav.services.safety.velocity_mux import VelocityMux
from runtime.msgs.geometry import Pose, PoseStamped, Twist, Vector3
from runtime.msgs.nav import Path
from runtime.runtime_interface import TOPICS, topic_default_frame_id

# Detect native map kernel availability for traversability tests
try:
    from maps.adapters.python.kernels import create_map_kernel_backend as _create_kernel

    _HAS_MAP_KERNEL = _create_kernel() is not None
except Exception:
    _HAS_MAP_KERNEL = False

_skip_no_kernel = pytest.mark.skipif(not _HAS_MAP_KERNEL, reason="native map kernel not available")


class _FakePlanner:
    is_ready = True
    has_map = True

    def plan(self, start: np.ndarray, goal: np.ndarray, **kwargs):
        midpoint = np.array([2.5, 0.0, 0.0])
        return [start.copy(), midpoint, goal.copy()], 1.0

    def update_map(self, *args, **kwargs) -> None:
        pass


class _ExplodingPlanner(_FakePlanner):
    def plan(self, start: np.ndarray, goal: np.ndarray, **kwargs):
        raise AssertionError("planner should not run for an already-at-goal preview")


class _SlowPlanner(_FakePlanner):
    def plan(self, start: np.ndarray, goal: np.ndarray, **kwargs):
        time.sleep(0.05)
        return super().plan(start, goal)


class _BrokenPlannerStatus:
    @property
    def is_ready(self) -> bool:
        raise RuntimeError("planner status unavailable")

    @property
    def has_map(self) -> bool:
        return True

    def plan(self, start: np.ndarray, goal: np.ndarray, **kwargs):
        raise AssertionError("planner should not run when status is unavailable")


class _NonfinitePathPlanner(_FakePlanner):
    def plan(self, start: np.ndarray, goal: np.ndarray, **kwargs):
        return [start.copy(), np.array([np.nan, 1.0, 0.0])], 1.0


class _NonfiniteTimingPlanner(_FakePlanner):
    def plan(self, start: np.ndarray, goal: np.ndarray, **kwargs):
        return [start.copy(), goal.copy()], float("nan")


class _OverflowDistancePlanner(_FakePlanner):
    def plan(self, start: np.ndarray, goal: np.ndarray, **kwargs):
        return [start.copy(), np.array([1e308, 0.0, 0.0])], 1.0


class _RuntimeErrorPlanner(_FakePlanner):
    def plan(self, start: np.ndarray, goal: np.ndarray, **kwargs):
        raise RuntimeError("planner returned empty path")


class _ReportingFailurePlanner(_FakePlanner):
    _planner_name = "pct"

    @property
    def last_plan_report(self):
        return {
            "selected_planner": "pct",
            "fallback_reason": "pct path_safety failed",
            "selected_path_safety": {"ok": False, "blocked_sample_count": 2},
            "rejected_plans": [{"planner": "pct", "reason": "unsafe"}],
            "policy": "reject",
        }

    def plan(self, start: np.ndarray, goal: np.ndarray, **kwargs):
        raise RuntimeError("pct path_safety failed")


class _EmptyPathPlanner(_FakePlanner):
    def plan(self, start: np.ndarray, goal: np.ndarray, **kwargs):
        return [], 1.0


class _GridBackend:
    def __init__(self, grid: np.ndarray, resolution: float = 1.0):
        self._grid = grid
        self._resolution = resolution
        self._origin = np.array([0.0, 0.0])

    def plan(self, start: np.ndarray, goal: np.ndarray):
        return [start, goal]


def test_navigation_goal_publishes_global_path_and_first_waypoint():
    nav = Navigation()
    nav._planner_svc = _FakePlanner()

    paths: list[list[np.ndarray]] = []
    waypoints: list[PoseStamped] = []
    states: list[dict] = []
    nav.global_path._add_callback(paths.append)
    nav.waypoint._add_callback(waypoints.append)
    nav.mission_status._add_callback(states.append)

    nav._robot_pos = np.array([0.0, 0.0, 0.0])
    nav._on_goal(PoseStamped(Pose(5.0, 0.0, 0.0)))

    assert len(paths) == 1
    assert isinstance(paths[0], Path)
    assert np.allclose(paths[0][-1], [5.0, 0.0, 0.0])
    assert len(waypoints) == 1
    assert waypoints[0].pose.position.x == 2.5
    assert states[-1]["state"] == "EXECUTING"


@pytest.mark.parametrize(
    ("planner", "reason"),
    [
        (_EmptyPathPlanner(), "planner returned empty path"),
        (_NonfinitePathPlanner(), "planner returned a non-finite path point"),
    ],
)
def test_navigation_rejects_invalid_planner_output_before_local_planning(planner, reason):
    nav = Navigation()
    nav._planner_svc = planner

    paths: list[list[np.ndarray]] = []
    waypoints: list[PoseStamped] = []
    stops: list[Twist] = []
    states: list[dict] = []
    nav.global_path._add_callback(paths.append)
    nav.waypoint._add_callback(waypoints.append)
    nav.recovery_cmd_vel._add_callback(stops.append)
    nav.mission_status._add_callback(states.append)

    nav._robot_pos = np.array([0.0, 0.0, 0.0])
    nav._on_goal(PoseStamped(Pose(5.0, 0.0, 0.0)))

    assert paths == []
    assert waypoints == []
    assert len(stops) == 1
    assert states[-1]["state"] == "FAILED"
    assert reason in states[-1]["failure_reason"]


def test_navigation_publishes_plan_report_when_planner_rejects_path():
    nav = Navigation()
    nav._planner_svc = _ReportingFailurePlanner()

    reports: list[dict] = []
    states: list[dict] = []
    nav.adapter_status._add_callback(reports.append)
    nav.mission_status._add_callback(states.append)

    nav._robot_pos = np.array([0.0, 0.0, 0.0])
    nav._on_goal(PoseStamped(Pose(5.0, 0.0, 0.0)))

    assert reports[-1]["event"] == "global_plan_selection"
    assert reports[-1]["fallback_reason"] == "pct path_safety failed"
    assert reports[-1]["policy"] == "reject"
    assert states[-1]["state"] == "FAILED"


def test_navigation_plan_preview_does_not_mutate_mission_or_publish_ports():
    nav = Navigation()
    nav._planner_svc = _FakePlanner()

    nav._robot_pos = np.array([0.0, 0.0, 0.0])
    initial_state = nav._state

    preview = nav.preview_plan(5.0, 0.0, 0.0)

    assert preview["ok"] is True
    assert preview["feasible"] is True
    assert preview["count"] == 3
    assert preview["path"][-1]["x"] == 5.0
    assert preview["distance_m"] == 5.0
    assert nav._state == initial_state
    assert nav._goal is None
    assert nav.global_path.msg_count == 0
    assert nav.waypoint.msg_count == 0
    assert nav.recovery_cmd_vel.msg_count == 0


def test_navigation_plan_preview_short_circuits_when_already_at_goal():
    nav = Navigation()
    nav._planner_svc = _ExplodingPlanner()
    nav._robot_pos = np.array([1.0, 2.0, 0.0])

    preview = nav.preview_plan(1.0, 2.0, 0.0)

    assert preview["ok"] is True
    assert preview["feasible"] is True
    assert preview["count"] == 1
    assert preview["distance_m"] == 0.0
    assert preview["plan_ms"] == 0.0
    assert preview["reasons"] == ["already_at_goal"]
    assert nav.global_path.msg_count == 0
    assert nav.waypoint.msg_count == 0
    assert nav.recovery_cmd_vel.msg_count == 0


def test_navigation_plan_preview_times_out_without_publishing_ports():
    nav = Navigation(preview_timeout=0.001)
    nav._planner_svc = _SlowPlanner()
    nav._robot_pos = np.array([0.0, 0.0, 0.0])

    preview = nav.preview_plan(1.0, 0.0, 0.0)

    assert preview["ok"] is True
    assert preview["feasible"] is False
    assert preview["reasons"] == ["planning_timeout"]
    assert "planner preview exceeded" in preview["error"]
    assert nav.global_path.msg_count == 0
    assert nav.waypoint.msg_count == 0
    assert nav.recovery_cmd_vel.msg_count == 0


def test_navigation_plan_preview_reports_busy_only_for_concurrent_preview():
    nav = Navigation()
    nav._planner_svc = _FakePlanner()
    nav._robot_pos = np.array([0.0, 0.0, 0.0])
    assert nav._plan_preview._lock.acquire(blocking=False)
    try:
        preview = nav.preview_plan(1.0, 0.0, 0.0)
    finally:
        nav._plan_preview._lock.release()

    assert preview["ok"] is True
    assert preview["feasible"] is False
    assert preview["reasons"] == ["planning_preview_busy"]
    assert "another plan preview" in preview["error"]
    assert nav.global_path.msg_count == 0
    assert nav.waypoint.msg_count == 0
    assert nav.recovery_cmd_vel.msg_count == 0


def test_navigation_plan_preview_reports_planner_runtime_error_as_failure():
    nav = Navigation()
    nav._planner_svc = _RuntimeErrorPlanner()
    nav._robot_pos = np.array([0.0, 0.0, 0.0])

    preview = nav.preview_plan(1.0, 0.0, 0.0)

    assert preview["ok"] is True
    assert preview["feasible"] is False
    assert preview["reasons"] == ["planning_failed"]
    assert "planner returned empty path" in preview["error"]
    assert nav.global_path.msg_count == 0
    assert nav.waypoint.msg_count == 0
    assert nav.recovery_cmd_vel.msg_count == 0


def test_navigation_plan_preview_exposes_plan_safety_failure_report():
    nav = Navigation()
    nav._planner_svc = _ReportingFailurePlanner()
    nav._robot_pos = np.array([0.0, 0.0, 0.0])

    preview = nav.preview_plan(5.0, 0.0, 0.0)

    assert preview["ok"] is True
    assert preview["feasible"] is False
    assert preview["selected_planner"] == "pct"
    assert preview["plan_safety_policy"] == "reject"
    assert preview["fallback_reason"] == "pct path_safety failed"
    assert preview["path_safety"]["ok"] is False
    assert preview["path_safety"]["blocked_sample_count"] == 2
    assert preview["rejected_plans"][0]["planner"] == "pct"
    assert nav.global_path.msg_count == 0
    assert nav.waypoint.msg_count == 0
    assert nav.recovery_cmd_vel.msg_count == 0


def test_navigation_plan_preview_degrades_when_planner_status_fails():
    nav = Navigation()
    nav._planner_svc = _BrokenPlannerStatus()
    nav._robot_pos = np.array([0.0, 0.0, 0.0])

    preview = nav.preview_plan(1.0, 0.0, 0.0)

    assert preview["ok"] is True
    assert preview["feasible"] is False
    assert preview["reasons"] == ["planner_status_unavailable"]
    assert "planner status unavailable" in preview["error"]
    assert nav.global_path.msg_count == 0
    assert nav.waypoint.msg_count == 0


@pytest.mark.parametrize(
    ("planner", "reason"),
    [
        (_NonfinitePathPlanner(), "planner_returned_nonfinite_path"),
        (_NonfiniteTimingPlanner(), "planner_returned_invalid_timing"),
        (_OverflowDistancePlanner(), "planner_returned_invalid_distance"),
    ],
)
def test_navigation_plan_preview_rejects_invalid_planner_output(planner, reason):
    nav = Navigation()
    nav._planner_svc = planner
    nav._robot_pos = np.array([0.0, 0.0, 0.0])

    preview = nav.preview_plan(1.0, 0.0, 0.0)

    json.dumps(preview, allow_nan=False)
    assert preview["ok"] is True
    assert preview["feasible"] is False
    assert preview["reasons"] == [reason]
    assert preview["path"] == []
    assert preview["distance_m"] is None
    assert preview["plan_ms"] is None
    assert nav.global_path.msg_count == 0
    assert nav.waypoint.msg_count == 0


def test_global_planner_safe_goal_bfs_adjusts_obstacle_goal():
    svc = GlobalPlanner(obstacle_thr=49.9)
    grid = np.array(
        [
            [100.0, 100.0, 100.0],
            [100.0, 100.0, 0.0],
            [100.0, 100.0, 100.0],
        ],
        dtype=np.float32,
    )
    svc._backend = _GridBackend(grid)

    safe_goal = svc._find_safe_goal(np.array([1.0, 1.0, 0.0]), tolerance=2.0)

    assert safe_goal is not None
    assert np.allclose(safe_goal, [2.0, 1.0, 0.0])


@_skip_no_kernel
def test_traversability_fusion_preserves_hard_costs_and_relays_esdf():
    module = TraversabilityCostModule(
        safe_distance=2.0,
        max_slope_deg=45.0,
        proximity_cap=50.0,
        publish_hz=1000.0,
    )
    fused: list[dict] = []
    relayed_esdf: list[dict] = []
    module.fused_cost._add_callback(fused.append)
    module.esdf_field._add_callback(relayed_esdf.append)

    esdf = {
        "distance_field": np.array([[0.0, 0.0], [0.0, 2.0]], dtype=np.float32),
        "resolution": 1.0,
        "origin": [0.0, 0.0],
    }
    costmap = {
        "grid": np.array([[0.0, 100.0], [99.0, 0.0]], dtype=np.float32),
        "resolution": 1.0,
        "origin": [0.0, 0.0],
    }

    module._on_esdf(esdf)
    module._on_costmap(costmap)

    assert relayed_esdf == [esdf]
    assert len(fused) == 1
    assert np.allclose(fused[0]["grid"], [[50.0, 100.0], [99.0, 0.0]])
    assert fused[0]["frame_id"] == topic_default_frame_id(TOPICS.exploration_grid)


@_skip_no_kernel
def test_traversability_slope_grid_normalizes_costmap_frame_id():
    module = TraversabilityCostModule(max_slope_deg=45.0, publish_hz=1000.0)
    slopes: list[dict] = []
    fused: list[dict] = []
    module.slope_grid._add_callback(slopes.append)
    module.fused_cost._add_callback(fused.append)

    module._on_elevation(
        {
            "max_z": np.array([[0.0, 0.0], [0.0, 1.0]], dtype=np.float32),
            "valid": np.ones((2, 2), dtype=bool),
            "resolution": 1.0,
            "origin": [0.0, 0.0],
            "frame_id": "map",
        }
    )
    module._on_costmap(
        {
            "grid": np.zeros((2, 2), dtype=np.float32),
            "resolution": 1.0,
            "origin": [0.0, 0.0],
            "frame_id": "/odom",
        }
    )

    assert len(slopes) == 1
    assert len(fused) == 1
    assert slopes[0]["frame_id"] == "odom"
    assert fused[0]["frame_id"] == "odom"


@_skip_no_kernel
def test_traversability_grid_participates_in_fused_cost():
    module = TraversabilityCostModule(max_slope_deg=45.0, publish_hz=1000.0)
    fused: list[dict] = []
    module.fused_cost._add_callback(fused.append)

    module._on_traversability(
        {
            "grid": np.array([[0.0, 70.0], [20.0, 0.0]], dtype=np.float32),
            "resolution": 1.0,
            "origin": [0.0, 0.0],
        }
    )
    module._on_costmap(
        {
            "grid": np.array([[0.0, 0.0], [99.0, 0.0]], dtype=np.float32),
            "resolution": 1.0,
            "origin": [0.0, 0.0],
        }
    )

    assert len(fused) == 1
    assert np.allclose(fused[0]["grid"], [[0.0, 70.0], [99.0, 0.0]])


@_skip_no_kernel
def test_traversability_fusion_refreshes_when_esdf_arrives_after_costmap():
    module = TraversabilityCostModule(safe_distance=2.0, publish_hz=1000.0)
    module._interval = 0.0
    fused: list[dict] = []
    module.fused_cost._add_callback(fused.append)

    module._on_costmap(
        {
            "grid": np.zeros((2, 2), dtype=np.float32),
            "resolution": 1.0,
            "origin": [0.0, 0.0],
        }
    )
    module._on_esdf(
        {
            "distance_field": np.array([[0.0, 2.0], [2.0, 2.0]], dtype=np.float32),
            "resolution": 1.0,
            "origin": [0.0, 0.0],
        }
    )

    assert len(fused) == 2
    assert fused[-1]["grid"][0, 0] > fused[0]["grid"][0, 0]


@_skip_no_kernel
def test_traversability_storage_inputs_keep_latest_without_changing_costmap_clock():
    module = TraversabilityCostModule()
    module.setup()

    assert module.costmap._policy == "all"
    assert module.elevation_map._policy == "latest"
    assert module.esdf._policy == "latest"
    assert module.traversability._policy == "latest"


def test_localization_degeneracy_scales_navigation_speed():
    nav = Navigation()
    monitor = LocalizationMonitorModule()
    monitor.speed_scale._add_callback(nav._on_speed_scale)
    monitor.localization_state._add_callback(nav._on_localization_state)
    monitor.setup()

    monitor.localization_status._deliver({"state": "TRACKING", "degeneracy": "MILD"})
    assert nav._speed_scale == 0.7

    monitor.localization_status._deliver({"state": "TRACKING", "degeneracy": "SEVERE"})
    assert nav._speed_scale == 0.4

    monitor.localization_status._deliver({"state": "FALLBACK_GNSS_ONLY", "degeneracy": "NONE"})
    assert nav._speed_scale == 0.3

    monitor.localization_status._deliver({"state": "TRACKING", "degeneracy": "NONE"})
    assert nav._speed_scale == 1.0


def test_visual_servo_near_target_publishes_cmd_vel_and_nav_stop():
    servo = VisualServoModule(servo_takeover_distance=3.0, servo_takeover_hysteresis=0.3)
    cmd_vel: list[Twist] = []
    goals: list[PoseStamped] = []
    stops: list[int] = []
    servo.cmd_vel._add_callback(cmd_vel.append)
    servo.goal_pose._add_callback(goals.append)
    servo.nav_stop._add_callback(stops.append)
    servo._mode = MODE_FIND
    servo._target_label = "crate"
    servo._latest_depth = np.ones((4, 4), dtype=np.float32)
    servo._intrinsics = (100.0, 100.0, 2.0, 2.0)
    servo._find_target_bbox = lambda: [1, 1, 2, 2]
    servo._bbox_nav.update = lambda **_: {
        "state": STATE_TRACKING,
        "distance": 2.0,
        "target_3d": np.array([2.0, 0.0, 0.0]),
        "linear_x": 0.2,
        "angular_z": -0.1,
    }

    servo._tick_find()

    assert stops == [1]
    assert len(cmd_vel) == 1
    assert cmd_vel[0].linear.x == 0.2
    assert cmd_vel[0].angular.z == -0.1
    assert goals == []


def test_visual_servo_far_target_publishes_goal_pose():
    servo = VisualServoModule(servo_takeover_distance=3.0, servo_takeover_hysteresis=0.3)
    cmd_vel: list[Twist] = []
    goals: list[PoseStamped] = []
    stops: list[int] = []
    servo.cmd_vel._add_callback(cmd_vel.append)
    servo.goal_pose._add_callback(goals.append)
    servo.nav_stop._add_callback(stops.append)
    servo._mode = MODE_FIND
    servo._target_label = "crate"
    servo._latest_depth = np.ones((4, 4), dtype=np.float32)
    servo._intrinsics = (100.0, 100.0, 2.0, 2.0)
    servo._find_target_bbox = lambda: [1, 1, 2, 2]
    servo._bbox_nav.update = lambda **_: {
        "state": STATE_TRACKING,
        "distance": 4.0,
        "target_3d": np.array([4.0, 0.0, 0.0]),
        "linear_x": 0.2,
        "angular_z": -0.1,
    }

    servo._tick_find()

    assert stops == []
    assert cmd_vel == []
    assert len(goals) == 1
    assert goals[0].pose.position.x == 4.0


def test_cmd_vel_mux_selects_highest_priority_active_source():
    mux = VelocityMux(source_timeout=10.0)
    driver_cmds: list[Twist] = []
    active_sources: list[str] = []
    mux.driver_cmd_vel._add_callback(driver_cmds.append)
    mux.active_source._add_callback(active_sources.append)

    mux._on_source("path_follower", Twist(linear=Vector3(x=0.1)))
    mux._on_source("visual_servo", Twist(linear=Vector3(x=0.2)))
    mux._on_source("teleop", Twist(linear=Vector3(x=0.3)))

    assert [cmd.linear.x for cmd in driver_cmds] == [0.1, 0.2, 0.3]
    assert active_sources == ["path_follower", "visual_servo", "teleop"]

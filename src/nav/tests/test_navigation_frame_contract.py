from __future__ import annotations

import json
import math
import time

import numpy as np
import pytest

from nav.mission.navigation import MissionState, Navigation
from nav.mission.tracking.waypoint_tracker import EV_STUCK, TrackerStatus
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import Odometry


class _RecordingPlanner:
    is_ready = True
    has_map = True

    def __init__(self) -> None:
        self.maps: list[dict] = []
        self.plan_calls = 0
        self.plans: list[dict] = []
        self.safe_goal_tolerances: list[float | None] = []

    def plan(self, start: np.ndarray, goal: np.ndarray, **kwargs):
        self.plan_calls += 1
        self.plans.append({
            "start": start.copy(),
            "goal": goal.copy(),
            "kwargs": dict(kwargs),
        })
        self.safe_goal_tolerances.append(kwargs.get("safe_goal_tolerance"))
        return [start.copy(), goal.copy()], 0.0

    def update_map(self, grid, resolution=0.2, origin=None) -> None:
        self.maps.append({
            "grid": grid,
            "resolution": resolution,
            "origin": origin,
        })


class _PublicOnlyPlannerService:
    is_ready = True
    has_map = True

    def __init__(self) -> None:
        self._last_plan_report = {
            "primary_planner": "pct",
            "selected_planner": "astar",
            "policy": "fallback_astar",
            "fallback_reason": "pct path_safety failed",
            "rejected_plans": [{"planner": "pct", "reason": "blocked"}],
            "selected_path_safety": {"ok": False},
            "reached_goal": True,
        }

    def __getattr__(self, name: str):
        if name in {"_planner_name", "_plan_safety_policy", "_fallback_planner_name"}:
            raise AssertionError(f"Navigation touched private planner field: {name}")
        raise AttributeError(name)

    @property
    def last_plan_report(self):
        return dict(self._last_plan_report)

    @property
    def map_artifact_gate(self):
        return {"required": False, "ok": True}

    def backend_status(self):
        return {
            "configured_backend": "pct",
            "backend": "astar",
            "fallback_backend": "astar",
            "degraded": True,
            "degraded_reason": "pct path_safety failed",
        }

    def plan(self, start: np.ndarray, goal: np.ndarray, **kwargs):
        return [start.copy(), goal.copy()], 0.0

    def update_map(self, grid, resolution=0.2, origin=None) -> None:
        return None


class _FlatZPlanner(_RecordingPlanner):
    def plan(self, start: np.ndarray, goal: np.ndarray, **kwargs):
        self.plan_calls += 1
        self.safe_goal_tolerances.append(kwargs.get("safe_goal_tolerance"))
        return [
            np.array([float(start[0]), float(start[1]), 0.0], dtype=float),
            np.array([float(goal[0]), float(goal[1]), 0.0], dtype=float),
        ], 0.0


class _IntermediatePathPlanner(_RecordingPlanner):
    def plan(self, start: np.ndarray, goal: np.ndarray, **kwargs):
        self.plan_calls += 1
        self.safe_goal_tolerances.append(kwargs.get("safe_goal_tolerance"))
        mid = np.array([
            (float(start[0]) + float(goal[0])) * 0.5,
            (float(start[1]) + float(goal[1])) * 0.5,
            float(goal[2]),
        ], dtype=float)
        return [start.copy(), mid, goal.copy()], 0.0


class _DetourPathPlanner(_RecordingPlanner):
    """Path with off-axis waypoints spread beyond the tracker's forward-prune
    search window (5), so the final goal is never visible to
    ``_advance_to_nearest_forward`` from the second waypoint onward. Used to
    exercise goal-proximity completion in isolation from the tracker's own
    forward-pruning + multi-waypoint catch-up (see WaypointTracker.update).
    """
    def plan(self, start: np.ndarray, goal: np.ndarray, **kwargs):
        self.plan_calls += 1
        self.safe_goal_tolerances.append(kwargs.get("safe_goal_tolerance"))
        start = np.asarray(start, dtype=float)
        goal = np.asarray(goal, dtype=float)
        detours = [
            np.array([
                start[0] + (goal[0] - start[0]) * frac,
                start[1] + (goal[1] - start[1]) * frac + 3.0,
                float(goal[2]),
            ], dtype=float)
            for frac in (1 / 6, 2 / 6, 3 / 6, 4 / 6, 5 / 6)
        ]
        return [start.copy(), *detours, goal.copy()], 0.0


class _ProjectedStartPlanner(_RecordingPlanner):
    def plan(self, start: np.ndarray, goal: np.ndarray, **kwargs):
        self.plan_calls += 1
        self.safe_goal_tolerances.append(kwargs.get("safe_goal_tolerance"))
        projected_start = np.array([1.48, 0.0, 0.0], dtype=float)
        return [
            projected_start,
            np.array([1.75, 0.0, 0.0], dtype=float),
            np.array([float(goal[0]), float(goal[1]), 0.0], dtype=float),
        ], 0.0


class _PartialThenFullPlanner:
    is_ready = True
    has_map = True

    def __init__(self) -> None:
        self.calls = 0
        self._last_plan_report = {}

    def plan(self, start: np.ndarray, goal: np.ndarray, **kwargs):
        self.calls += 1
        if self.calls == 1:
            partial = np.array([1.0, 0.0, 0.0], dtype=float)
            self._last_plan_report = {
                "primary_planner": "pct",
                "selected_planner": "pct",
                "selected_path_safety": {"ok": True},
                "fallback_reason": "",
                "rejected_plans": [],
                "policy": "reject",
                "reached_goal": True,
                "primary_replan": {
                    "used": True,
                    "original_goal": goal[:3].tolist(),
                    "repaired_goal": partial.tolist(),
                },
            }
            return [start.copy(), partial], 0.0
        self._last_plan_report = {
            "primary_planner": "pct",
            "selected_planner": "pct",
            "selected_path_safety": {"ok": True},
            "fallback_reason": "",
            "rejected_plans": [],
            "policy": "reject",
            "reached_goal": True,
        }
        return [start.copy(), goal.copy()], 0.0

    @property
    def last_plan_report(self):
        return dict(self._last_plan_report)

    def update_map(self, grid, resolution=0.2, origin=None) -> None:
        return None


class _EmptyThenPathPlanner:
    is_ready = True
    has_map = True

    def __init__(
        self,
        first_error: str = (
            "GlobalPlanner: primary planner returned empty path"
        ),
    ) -> None:
        self.calls = 0
        self.maps: list[dict] = []
        self._last_plan_report = {}
        self._first_error = first_error

    def plan(self, start: np.ndarray, goal: np.ndarray, **kwargs):
        self.calls += 1
        if self.calls == 1:
            self._last_plan_report = {
                "primary_planner": "astar",
                "selected_planner": "astar",
                "fallback_reason": "primary planner returned empty path",
                "rejected_plans": [
                    {
                        "planner": "astar",
                        "reason": "primary planner returned empty path",
                    }
                ],
                "policy": "reject",
                "reached_goal": False,
            }
            raise RuntimeError(self._first_error)
        self._last_plan_report = {
            "primary_planner": "astar",
            "selected_planner": "astar",
            "selected_path_safety": {"ok": True},
            "fallback_reason": "",
            "rejected_plans": [],
            "policy": "reject",
            "reached_goal": True,
        }
        return [start.copy(), goal.copy()], 0.0

    @property
    def last_plan_report(self):
        return dict(self._last_plan_report)

    def update_map(self, grid, resolution=0.2, origin=None) -> None:
        self.maps.append({
            "grid": grid,
            "resolution": resolution,
            "origin": origin,
        })


class _FailuresThenPathPlanner(_EmptyThenPathPlanner):
    def __init__(self, failures: int = 2) -> None:
        super().__init__()
        self._failures = failures

    def plan(self, start: np.ndarray, goal: np.ndarray, **kwargs):
        self.calls += 1
        if self.calls <= self._failures:
            self._last_plan_report = {
                "primary_planner": "astar",
                "selected_planner": "astar",
                "fallback_reason": "primary planner returned empty path",
                "rejected_plans": [
                    {
                        "planner": "astar",
                        "reason": "primary planner returned empty path",
                    }
                ],
                "policy": "reject",
                "reached_goal": False,
            }
            raise RuntimeError(self._first_error)
        self._last_plan_report = {
            "primary_planner": "astar",
            "selected_planner": "astar",
            "selected_path_safety": {"ok": True},
            "fallback_reason": "",
            "rejected_plans": [],
            "policy": "reject",
            "reached_goal": True,
        }
        return [start.copy(), goal.copy()], 0.0


class _StuckTracker:
    current_waypoint = None
    wp_index = 0
    path_length = 1

    def update(self, robot_pos: np.ndarray, robot_yaw: float | None = None):
        return TrackerStatus(0, 1, event=EV_STUCK)


def test_navigation_stop_releases_lifecycle_resources() -> None:
    nav = Navigation(planner="direct")
    nav.setup()
    nav.start()

    assert nav.running is True
    assert nav.goal_pose._callback is not None

    nav.stop()

    assert nav.running is False
    assert nav.goal_pose._callback is None
    assert nav._plan_preview.closed is True

    nav.stop()


def test_navigation_status_uses_public_planner_service_contract() -> None:
    nav = Navigation()
    nav._planner_svc = _PublicOnlyPlannerService()
    nav._odom_frame_id = "map"
    nav._costmap_frame_id = "map"

    try:
        preview = nav.preview_plan(1.0, 0.0)
        status = json.loads(nav.get_navigation_status())
        health = nav.health()
    finally:
        nav.stop()

    assert preview["planner"] == "astar"
    assert preview["selected_planner"] == "astar"
    assert preview["plan_safety_policy"] == "fallback_astar"
    assert preview["fallback_reason"] == "pct path_safety failed"
    assert status["plan_safety_policy"] == "fallback_astar"
    assert status["last_plan_report"]["selected_planner"] == "astar"
    assert health["planner_backend"]["configured_backend"] == "pct"
    assert health["planner_backend"]["backend"] == "astar"
    assert health["planner_backend"]["fallback_backend"] == "astar"
    assert health["navigation"]["last_plan_report"]["policy"] == "fallback_astar"


def test_navigation_reports_frame_mismatch_when_odometry_is_not_in_planning_frame():
    nav = Navigation()
    events: list[dict] = []
    nav.adapter_status._add_callback(events.append)

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(1.0, 2.0, 0.0), orientation=Quaternion()),
        frame_id="odom",
    ))

    assert events[-1]["event"] == "frame_mismatch"
    assert events[-1]["source"] == "odometry"
    assert events[-1]["expected_frame"] == "map"
    assert events[-1]["received_frame"] == "odom"
    assert nav._odom_frame_id == "odom"
    np.testing.assert_array_equal(nav._robot_pos, np.zeros(3))


def test_navigation_accepts_odom_frame_when_map_odom_tf_is_valid():
    nav = Navigation()
    events: list[dict] = []
    nav.adapter_status._add_callback(events.append)
    nav._on_map_odom_tf({
        "valid": True,
        "frame_id": "map",
        "child_frame_id": "odom",
        "tx": 10.0,
        "ty": -2.0,
        "tz": 0.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": math.sin(math.pi / 4.0),
        "qw": math.cos(math.pi / 4.0),
    })

    nav._on_odom(Odometry(
        pose=Pose(
            position=Vector3(1.0, 0.0, 0.0),
            orientation=Quaternion.from_yaw(0.0),
        ),
        frame_id="odom",
    ))

    assert not [event for event in events if event["event"] == "frame_mismatch"]
    assert nav._odom_frame_id == "odom"
    np.testing.assert_allclose(nav._robot_pos, [10.0, -1.0, 0.0], atol=1e-6)
    assert nav._robot_yaw == pytest.approx(math.pi / 2.0)


def test_navigation_reports_frame_mismatch_when_costmap_is_not_in_planning_frame():
    nav = Navigation()
    nav._planner_svc = _RecordingPlanner()
    events: list[dict] = []
    nav.adapter_status._add_callback(events.append)

    nav._on_costmap({
        "grid": np.zeros((2, 2), dtype=np.int8),
        "resolution": 0.5,
        "origin": [0.0, 0.0],
        "frame_id": "odom",
    })

    assert events[-1]["event"] == "frame_mismatch"
    assert events[-1]["source"] == "costmap"
    assert events[-1]["expected_frame"] == "map"
    assert events[-1]["received_frame"] == "odom"
    assert nav._costmap_frame_id == "odom"
    assert nav._planner_svc.maps == []


def test_pct_navigation_does_not_global_replan_from_costmap_by_default():
    nav = Navigation(planner="pct")
    planner = _RecordingPlanner()
    nav._planner_svc = planner
    mission_statuses: list[dict] = []
    global_paths: list[list[np.ndarray]] = []
    nav.mission_status._add_callback(mission_statuses.append)
    nav.global_path._add_callback(global_paths.append)
    nav._set_state("EXECUTING")
    nav._goal = np.array([2.0, 0.0, 0.0], dtype=float)
    nav._last_costmap_replan_time = 0.0
    status_count_before_costmap = len(mission_statuses)

    nav._on_costmap({
        "grid": np.zeros((10, 10), dtype=np.int8),
        "resolution": 0.2,
        "origin": [0.0, 0.0],
        "frame_id": "map",
    })

    assert nav._replan_on_costmap_update is False
    assert mission_statuses[-1]["replan_on_costmap_update"] is False
    assert len(mission_statuses) == status_count_before_costmap
    assert global_paths == []
    assert planner.maps == []
    assert planner.plan_calls == 0


def test_astar_navigation_can_still_replan_from_costmap_update():
    nav = Navigation(planner="astar")
    planner = _RecordingPlanner()
    nav._planner_svc = planner
    nav._state = "EXECUTING"
    nav._goal = np.array([2.0, 0.0, 0.0], dtype=float)
    nav._last_costmap_replan_time = 0.0

    nav._on_costmap({
        "grid": np.zeros((10, 10), dtype=np.int8),
        "resolution": 0.2,
        "origin": [0.0, 0.0],
        "frame_id": "map",
    })

    assert nav._replan_on_costmap_update is True
    assert len(planner.maps) == 1
    assert planner.plan_calls == 1


def test_navigation_anchors_planner_start_height_to_current_odom():
    nav = Navigation(waypoint_threshold=0.35,
        final_waypoint_threshold=0.15,
    )
    nav._planner_svc = _FlatZPlanner()
    waypoints: list[PoseStamped] = []
    paths: list[list[np.ndarray]] = []
    nav.waypoint._add_callback(waypoints.append)
    nav.global_path._add_callback(paths.append)

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(0.0, 0.0, 0.55), orientation=Quaternion()),
        frame_id="map",
    ))
    nav._on_goal(PoseStamped(
        pose=Pose(position=Vector3(1.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    assert nav._state == "EXECUTING"
    assert paths[-1][0][2] == 0.55
    assert paths[-1][1][2] == 0.55
    assert nav._tracker.wp_index == 1
    assert waypoints[-1].pose.position.x == 1.0
    assert waypoints[-1].pose.position.z == 0.55


def test_navigation_inserts_robot_pose_before_projected_planner_start():
    nav = Navigation(waypoint_threshold=0.2,
        final_waypoint_threshold=0.2,
        allow_path_start_insert=True,
    )
    nav._planner_svc = _ProjectedStartPlanner()
    waypoints: list[PoseStamped] = []
    paths: list[list[np.ndarray]] = []
    nav.waypoint._add_callback(waypoints.append)
    nav.global_path._add_callback(paths.append)

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(0.0, 0.0, 0.35), orientation=Quaternion()),
        frame_id="map",
    ))
    nav._on_goal(PoseStamped(
        pose=Pose(position=Vector3(2.0, 0.0, 0.35), orientation=Quaternion()),
        frame_id="map",
    ))

    assert nav._state == "EXECUTING"
    assert len(paths[-1]) == 4
    np.testing.assert_allclose(paths[-1][0], np.array([0.0, 0.0, 0.35]))
    np.testing.assert_allclose(paths[-1][1], np.array([1.48, 0.0, 0.35]))
    assert nav._tracker.wp_index == 1
    assert waypoints[-1].pose.position.x == 1.48
    assert nav._path_start_anchor_status["mode"] == "insert"
    assert nav._path_start_anchor_status["distance_m"] == 1.48


def test_navigation_goal_proximity_completion_is_disabled_by_default():
    nav = Navigation(waypoint_threshold=0.2,
        final_waypoint_threshold=0.2,
    )
    assert nav._complete_path_on_goal_proximity is False
    nav._planner_svc = _DetourPathPlanner()
    events: list[dict] = []
    nav.adapter_status._add_callback(events.append)

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(0.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))
    nav._on_goal(PoseStamped(
        pose=Pose(position=Vector3(2.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))
    assert nav._state == "EXECUTING"
    assert nav._tracker.wp_index == 1

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(1.95, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    assert nav._state == "EXECUTING"
    # Forward-pruning still snaps ahead to the nearest in-window detour
    # waypoint (the final goal sits outside the 5-wide search window from
    # here), but that detour is far from the robot so it must not be
    # marked "reached" -- and with the feature off, proximity to the
    # actual goal must not short-circuit the path either.
    assert nav._tracker.wp_index == 5
    assert not any(
        event.get("event") == "goal_proximity_path_complete"
        for event in events
    )


def test_navigation_goal_proximity_completion_advances_patrol_when_enabled():
    nav = Navigation(waypoint_threshold=0.2,
        final_waypoint_threshold=0.2,
        complete_path_on_goal_proximity=True,
        goal_proximity_completion_threshold=0.25,
    )
    assert nav._complete_path_on_goal_proximity is True
    assert nav._goal_proximity_completion_threshold == 0.25
    planner = _DetourPathPlanner()
    nav._planner_svc = planner
    events: list[dict] = []
    nav.adapter_status._add_callback(events.append)

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(0.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))
    nav._on_patrol_goals([
        {"x": 2.0, "y": 0.0, "z": 0.0},
        {"x": 4.0, "y": 0.0, "z": 0.0},
    ])
    assert nav._state == "EXECUTING"
    assert nav._mission_mode == "PATROL"
    assert nav._patrol_index == 0
    assert nav._tracker.wp_index == 1

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(1.95, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    assert nav._state == "EXECUTING"
    assert nav._mission_mode == "PATROL"
    assert nav._patrol_index == 1
    assert np.allclose(nav._goal, np.array([4.0, 0.0, 0.0]))
    assert planner.plan_calls == 2
    proximity_events = [
        event for event in events
        if event.get("event") == "goal_proximity_path_complete"
    ]
    assert proximity_events
    assert proximity_events[-1]["distance_to_goal_m"] == 0.05
    # Forward-pruning has already snapped the tracker ahead to the nearest
    # in-window detour waypoint (index 5); goal-proximity completion fires
    # independently of that, based on direct distance to the mission goal.
    assert proximity_events[-1]["waypoint_index"] == 5


def test_navigation_accepts_map_frame_odometry_without_frame_mismatch_report():
    nav = Navigation()
    events: list[dict] = []
    nav.adapter_status._add_callback(events.append)

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(1.0, 2.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    assert events == []
    assert nav._odom_frame_id == "map"


def test_navigation_defers_empty_path_until_costmap_retry_succeeds():
    nav = Navigation(defer_empty_path_planning_failure=True,
        empty_path_retry_interval_s=0.1,
        empty_path_retry_timeout_s=5.0,
    )
    planner = _EmptyThenPathPlanner()
    nav._planner_svc = planner
    mission_statuses: list[dict] = []
    adapter_events: list[dict] = []
    waypoints: list[PoseStamped] = []
    nav.mission_status._add_callback(mission_statuses.append)
    nav.adapter_status._add_callback(adapter_events.append)
    nav.waypoint._add_callback(waypoints.append)

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(0.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))
    nav._on_goal(PoseStamped(
        pose=Pose(position=Vector3(2.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    assert nav._state == "PLANNING"
    assert waypoints == []
    assert planner.calls == 1
    assert any(
        event.get("event") == "planning_deferred_empty_path"
        for event in adapter_events
    )
    assert mission_statuses[-1]["last_plan_report"]["fallback_reason"] == ""
    assert mission_statuses[-1]["last_plan_report"]["deferred_planning"]["active"] is True
    assert all(status["state"] != "FAILED" for status in mission_statuses)

    nav._last_costmap_replan_time = 0.0
    nav._on_costmap({
        "grid": np.zeros((10, 10), dtype=np.int8),
        "resolution": 0.2,
        "origin": [0.0, 0.0],
        "frame_id": "map",
    })

    assert planner.calls == 2
    assert nav._state == "EXECUTING"
    assert len(waypoints) == 1
    assert nav._deferred_empty_path_first_ts == 0.0
    assert all(status["state"] != "FAILED" for status in mission_statuses)


def test_navigation_patrol_keeps_patrolling_after_deferred_empty_path_retry():
    nav = Navigation(defer_empty_path_planning_failure=True,
        empty_path_retry_interval_s=0.1,
        empty_path_retry_timeout_s=5.0,
        waypoint_threshold=0.15,
        final_waypoint_threshold=0.15,
    )
    planner = _EmptyThenPathPlanner()
    nav._planner_svc = planner
    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(0.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    nav._on_patrol_goals([
        {"x": 1.0, "y": 0.0, "z": 0.0},
        {"x": 2.0, "y": 0.0, "z": 0.0},
    ])

    assert nav._state == "PLANNING"
    assert planner.calls == 1

    nav._last_costmap_replan_time = 0.0
    nav._on_costmap({
        "grid": np.zeros((10, 10), dtype=np.int8),
        "resolution": 0.2,
        "origin": [0.0, 0.0],
        "frame_id": "map",
    })

    assert planner.calls == 2
    assert nav._state == "EXECUTING"
    assert nav._mission_mode == "PATROL"
    assert nav._patrol_index == 0

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(1.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    assert nav._state == "EXECUTING"
    assert nav._mission_mode == "PATROL"
    assert nav._patrol_index == 1
    assert np.allclose(nav._goal, np.array([2.0, 0.0, 0.0]))


def test_navigation_patrol_recovery_from_failed_empty_path_keeps_patrolling():
    nav = Navigation(defer_empty_path_planning_failure=True,
        empty_path_retry_interval_s=0.1,
        empty_path_retry_timeout_s=0.2,
        waypoint_threshold=0.15,
        final_waypoint_threshold=0.15,
    )
    planner = _FailuresThenPathPlanner(failures=2)
    nav._planner_svc = planner
    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(0.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    nav._on_patrol_goals([
        {"x": 1.0, "y": 0.0, "z": 0.0},
        {"x": 2.0, "y": 0.0, "z": 0.0},
    ])

    assert nav._state == "PLANNING"
    assert planner.calls == 1

    nav._deferred_empty_path_first_ts = time.time() - 1.0
    nav._last_costmap_replan_time = 0.0
    nav._on_costmap({
        "grid": np.zeros((10, 10), dtype=np.int8),
        "resolution": 0.2,
        "origin": [0.0, 0.0],
        "frame_id": "map",
    })

    assert planner.calls == 2
    assert nav._state == "FAILED"
    assert nav._patrol_index == 0

    # After Bug 3 fix: FAILED state does NOT auto-recover on costmap update.
    # Patrol must be explicitly re-started with a new goal.
    nav._last_costmap_replan_time = 0.0
    nav._on_costmap({
        "grid": np.zeros((10, 10), dtype=np.int8),
        "resolution": 0.2,
        "origin": [0.0, 0.0],
        "frame_id": "map",
    })

    assert planner.calls == 2  # unchanged - no auto-recovery from FAILED
    assert nav._state == "FAILED"
    assert nav._patrol_index == 0


def test_navigation_defers_unreachable_safe_goal_until_costmap_retry_succeeds():
    nav = Navigation(defer_empty_path_planning_failure=True,
        empty_path_retry_interval_s=0.1,
        empty_path_retry_timeout_s=5.0,
    )
    planner = _EmptyThenPathPlanner(
        "GlobalPlanner: goal has no reachable free cell within 6.0m"
    )
    nav._planner_svc = planner
    mission_statuses: list[dict] = []
    nav.mission_status._add_callback(mission_statuses.append)

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(0.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))
    nav._on_goal(PoseStamped(
        pose=Pose(position=Vector3(4.0, 1.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    assert nav._state == "PLANNING"
    assert planner.calls == 1
    assert mission_statuses[-1]["last_plan_report"]["fallback_reason"] == ""
    assert mission_statuses[-1]["last_plan_report"]["deferred_planning"]["active"] is True

    nav._last_costmap_replan_time = 0.0
    nav._on_costmap({
        "grid": np.zeros((10, 10), dtype=np.int8),
        "resolution": 0.2,
        "origin": [0.0, 0.0],
        "frame_id": "map",
    })

    assert planner.calls == 2
    assert nav._state == "EXECUTING"
    assert all(status["state"] != "FAILED" for status in mission_statuses)


def test_navigation_blocks_goal_when_odometry_is_not_in_planning_frame():
    nav = Navigation(allow_direct_goal_fallback=True)
    nav._planner_svc = _RecordingPlanner()
    events: list[dict] = []
    waypoints: list[PoseStamped] = []
    nav.adapter_status._add_callback(events.append)
    nav.waypoint._add_callback(waypoints.append)

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(1.0, 2.0, 0.0), orientation=Quaternion()),
        frame_id="odom",
    ))
    nav._on_goal(PoseStamped(
        pose=Pose(position=Vector3(3.0, 4.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    assert nav._state == "FAILED"
    assert waypoints == []
    assert nav._failure_reason == "unsupported odometry frame 'odom'; expected 'map'"
    assert events[-1]["event"] == "navigation_blocked"
    assert events[-1]["source"] == "odometry"


def test_navigation_clears_motion_when_active_odometry_frame_mismatches():
    nav = Navigation()
    events: list[dict] = []
    clears: list[bool] = []
    zeros = []
    nav.adapter_status._add_callback(events.append)
    nav.clear_path._add_callback(clears.append)
    nav.recovery_cmd_vel._add_callback(zeros.append)

    nav._state = "EXECUTING"
    nav._goal = np.array([5.0, 0.0, 0.0])
    nav._tracker.reset([np.array([5.0, 0.0, 0.0])], nav._robot_pos)

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(2.0, 3.0, 0.0), orientation=Quaternion()),
        frame_id="odom",
    ))

    assert nav._state == "FAILED"
    assert nav._failure_reason == "unsupported odometry frame 'odom'; expected 'map'"
    assert nav._tracker.path_length == 0
    np.testing.assert_array_equal(nav._robot_pos, np.zeros(3))
    assert clears == [True]
    assert zeros[-1].linear.x == 0.0
    assert zeros[-1].angular.z == 0.0
    assert events[-1]["event"] == "navigation_blocked"
    assert events[-1]["source"] == "odometry"


def test_navigation_clears_motion_when_active_costmap_frame_mismatches():
    nav = Navigation()
    nav._planner_svc = _RecordingPlanner()
    events: list[dict] = []
    clears: list[bool] = []
    zeros = []
    nav.adapter_status._add_callback(events.append)
    nav.clear_path._add_callback(clears.append)
    nav.recovery_cmd_vel._add_callback(zeros.append)

    nav._state = "EXECUTING"
    nav._goal = np.array([5.0, 0.0, 0.0])
    nav._tracker.reset([np.array([5.0, 0.0, 0.0])], nav._robot_pos)

    nav._on_costmap({
        "grid": np.zeros((2, 2), dtype=np.int8),
        "resolution": 0.5,
        "origin": [0.0, 0.0],
        "frame_id": "odom",
    })

    assert nav._state == "FAILED"
    assert nav._failure_reason == "unsupported costmap frame 'odom'; expected 'map'"
    assert nav._tracker.path_length == 0
    assert nav._planner_svc.maps == []
    assert clears == [True]
    assert zeros[-1].linear.x == 0.0
    assert zeros[-1].angular.z == 0.0
    assert events[-1]["event"] == "navigation_blocked"
    assert events[-1]["source"] == "costmap"


def test_navigation_clears_motion_before_replanning_after_map_frame_jump():
    nav = Navigation()
    nav._planner_svc = _RecordingPlanner()
    clears: list[bool] = []
    zeros = []
    paths: list[list[np.ndarray]] = []
    waypoints: list[PoseStamped] = []
    nav.clear_path._add_callback(clears.append)
    nav.recovery_cmd_vel._add_callback(zeros.append)
    nav.global_path._add_callback(paths.append)
    nav.waypoint._add_callback(waypoints.append)
    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(0.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))
    nav._state = "EXECUTING"
    nav._goal = np.array([2.0, 0.0, 0.0])
    nav._tracker.reset([np.array([2.0, 0.0, 0.0])], nav._robot_pos)

    nav._on_map_frame_jump({"dt_m": 1.0, "dyaw_deg": 15.0})

    assert clears == [True]
    assert zeros[-1].linear.x == 0.0
    assert zeros[-1].angular.z == 0.0
    assert nav._state == "EXECUTING"
    assert len(paths) == 1
    assert len(paths[-1]) == 2
    assert waypoints[-1].frame_id == "map"


def test_navigation_ignores_plain_map_odom_tf_on_jump_event_port():
    nav = Navigation()
    nav._planner_svc = _RecordingPlanner()
    clears: list[bool] = []
    zeros = []
    paths: list[list[np.ndarray]] = []
    nav.clear_path._add_callback(clears.append)
    nav.recovery_cmd_vel._add_callback(zeros.append)
    nav.global_path._add_callback(paths.append)
    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(0.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))
    nav._state = "EXECUTING"
    nav._goal = np.array([2.0, 0.0, 0.0])
    nav._tracker.reset([np.array([2.0, 0.0, 0.0])], nav._robot_pos)

    nav._on_map_frame_jump({
        "valid": True,
        "frame_id": "map",
        "child_frame_id": "odom",
        "tx": 0.0,
        "ty": 0.0,
        "tz": 0.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0,
        "qw": 1.0,
    })

    assert clears == []
    assert zeros == []
    assert paths == []
    assert nav._state == "EXECUTING"

    nav._on_map_frame_jump({
        "type": "map_frame_jump",
        "dt_m": 0.0,
        "dyaw_deg": 0.0,
        "reason": "relocalized_same_pose",
    })

    assert clears == []
    assert zeros == []
    assert paths == []
    assert nav._state == "EXECUTING"


def test_navigation_passes_safe_goal_tolerance_to_planner_service():
    nav = Navigation(safe_goal_tolerance=0.0)
    planner = _RecordingPlanner()
    nav._planner_svc = planner
    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(0.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    nav._on_goal(PoseStamped(
        pose=Pose(position=Vector3(1.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    assert planner.safe_goal_tolerances == [0.0]


def test_navigation_patrol_mode_survives_planning_and_advances_goals():
    nav = Navigation(waypoint_threshold=0.15,
        final_waypoint_threshold=0.15,
    )
    planner = _RecordingPlanner()
    nav._planner_svc = planner
    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(0.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    nav._on_patrol_goals([
        {"x": 1.0, "y": 0.0, "z": 0.0},
        {"x": 2.0, "y": 0.0, "z": 0.0},
    ])

    assert nav._state == "EXECUTING"
    assert nav._mission_mode == "PATROL"
    assert nav._patrol_index == 0
    assert np.allclose(nav._goal, np.array([1.0, 0.0, 0.0]))

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(1.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    assert nav._state == "EXECUTING"
    assert nav._mission_mode == "PATROL"
    assert nav._patrol_index == 1
    assert np.allclose(nav._goal, np.array([2.0, 0.0, 0.0]))
    assert len(planner.safe_goal_tolerances) == 2


def test_navigation_clears_motion_when_path_completes():
    nav = Navigation(waypoint_threshold=0.35)
    clears: list[bool] = []
    zeros = []
    nav.clear_path._add_callback(clears.append)
    nav.recovery_cmd_vel._add_callback(zeros.append)
    nav._state = "EXECUTING"
    nav._goal = np.array([0.1, 0.0, 0.0])
    nav._tracker.reset([np.array([0.1, 0.0, 0.0])], np.zeros(3))

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(0.1, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    assert nav._state == "SUCCESS"
    assert clears == [True]
    assert zeros[-1].linear.x == 0.0
    assert zeros[-1].angular.z == 0.0


def test_navigation_replans_after_reaching_pct_repaired_partial_goal():
    nav = Navigation(waypoint_threshold=0.15,
        final_waypoint_threshold=0.15,
    )
    planner = _PartialThenFullPlanner()
    nav._planner_svc = planner
    events: list[dict] = []
    nav.adapter_status._add_callback(events.append)

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(0.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))
    nav._on_goal(PoseStamped(
        pose=Pose(position=Vector3(2.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    assert planner.calls == 1
    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(1.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    assert planner.calls == 2
    assert nav._state == "EXECUTING"
    assert any(event["event"] == "partial_path_complete_replan" for event in events)

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(2.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    assert nav._state == "SUCCESS"


def test_navigation_can_accept_repaired_partial_goal_as_exploration_progress():
    nav = Navigation(waypoint_threshold=0.15,
        final_waypoint_threshold=0.15,
        accept_partial_goal_progress=True,
    )
    planner = _PartialThenFullPlanner()
    nav._planner_svc = planner
    events: list[dict] = []
    nav.adapter_status._add_callback(events.append)

    goal = PoseStamped(
        pose=Pose(position=Vector3(2.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    )
    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(0.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))
    nav._on_goal(goal)

    assert planner.calls == 1
    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(1.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    assert planner.calls == 1
    assert nav._state == "SUCCESS"
    assert any(
        event["event"] == "partial_goal_progress_complete" for event in events
    )
    assert not any(
        event["event"] == "partial_path_complete_replan" for event in events
    )

    nav._on_goal(goal)

    assert planner.calls == 1
    assert nav._state == "SUCCESS"
    assert any(
        event["event"] == "partial_goal_repeat_ignored" for event in events
    )


def test_navigation_partial_goal_repeat_ignore_expires_for_rolling_exploration():
    nav = Navigation(waypoint_threshold=0.15,
        final_waypoint_threshold=0.15,
        accept_partial_goal_progress=True,
        partial_goal_repeat_ignore_window_s=0.01,
    )
    planner = _PartialThenFullPlanner()
    nav._planner_svc = planner

    goal = PoseStamped(
        pose=Pose(position=Vector3(2.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    )
    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(0.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))
    nav._on_goal(goal)
    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(1.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    assert nav._state == "SUCCESS"
    assert planner.calls == 1

    nav._partial_progress_completed_ts = 0.0
    nav._on_goal(goal)

    assert planner.calls == 2
    assert nav._state == "EXECUTING"


def test_navigation_soft_stop_holds_mission_without_clearing_path():
    nav = Navigation()
    events: list[dict] = []
    clears: list[bool] = []
    zeros = []
    nav.adapter_status._add_callback(events.append)
    nav.clear_path._add_callback(clears.append)
    nav.recovery_cmd_vel._add_callback(zeros.append)
    nav._state = "EXECUTING"
    nav._goal = np.array([5.0, 0.0, 0.0])
    nav._tracker.reset([np.array([2.0, 0.0, 0.0]), np.array([5.0, 0.0, 0.0])], nav._robot_pos)

    nav._on_stop(1)

    assert nav._state == "EXECUTING"
    assert nav._goal is not None
    assert nav._tracker.path_length == 2
    assert clears == []
    assert zeros == []
    assert events[-1]["event"] == "safety_soft_stop"
    assert events[-1]["action"] == "mission_held"


def test_navigation_hard_stop_clears_active_mission_path():
    nav = Navigation()
    events: list[dict] = []
    clears: list[bool] = []
    zeros = []
    nav.adapter_status._add_callback(events.append)
    nav.clear_path._add_callback(clears.append)
    nav.recovery_cmd_vel._add_callback(zeros.append)
    nav._state = "EXECUTING"
    nav._goal = np.array([5.0, 0.0, 0.0])
    nav._tracker.reset([np.array([5.0, 0.0, 0.0])], nav._robot_pos)

    nav._on_stop(2)

    assert nav._state == "IDLE"
    assert nav._goal is None
    assert nav._goal_frame_id is None
    assert nav._tracker.path_length == 0
    assert clears == [True]
    assert zeros[-1].linear.x == 0.0
    assert zeros[-1].angular.z == 0.0
    assert events[-1]["event"] == "safety_stop"
    assert events[-1]["action"] == "mission_cleared"


def test_navigation_timeout_stops_motion_and_clears_path():
    nav = Navigation()
    clears: list[bool] = []
    zeros = []
    nav.clear_path._add_callback(clears.append)
    nav.recovery_cmd_vel._add_callback(zeros.append)
    nav._state = MissionState.EXECUTING
    nav._mission_start_time = time.time() - 10.0
    nav._mission_timeout = 1.0
    nav._goal = np.array([5.0, 0.0, 0.0])
    nav._tracker.reset([np.array([2.0, 0.0, 0.0]), np.array([5.0, 0.0, 0.0])], nav._robot_pos)

    nav._check_mission_timeout()

    assert nav._state == MissionState.FAILED
    assert nav._tracker.path_length == 0
    assert clears == [True]
    assert zeros[-1].linear.x == 0.0
    assert zeros[-1].angular.z == 0.0


def test_navigation_cancel_is_idempotent_motion_cleanup_while_idle():
    nav = Navigation()
    clears: list[bool] = []
    zeros = []
    nav.clear_path._add_callback(clears.append)
    nav.recovery_cmd_vel._add_callback(zeros.append)
    nav._goal = np.array([5.0, 0.0, 0.0])
    nav._tracker.reset([np.array([5.0, 0.0, 0.0])], nav._robot_pos)

    nav._on_cancel("operator_cancel")

    assert nav._state == "IDLE"
    assert nav._goal is None
    assert nav._goal_frame_id is None
    assert nav._tracker.path_length == 0
    assert clears == [True]
    assert zeros[-1].linear.x == 0.0
    assert zeros[-1].angular.z == 0.0


def test_teleop_release_does_not_auto_resume_without_explicit_resume(monkeypatch):
    nav = Navigation(auto_resume_after_teleop=False)
    nav._state = MissionState.EXECUTING
    nav._goal = np.array([1.0, 2.0, 0.0], dtype=float)
    planned: list[bool] = []
    events: list[dict] = []
    nav.adapter_status._add_callback(events.append)
    monkeypatch.setattr(nav, "_plan", lambda: planned.append(True))

    nav._on_teleop_active(True)
    nav._on_teleop_active(False)

    assert planned == []
    assert nav._state == MissionState.PAUSED
    assert nav._pre_teleop_goal is None
    assert events[-1]["event"] == "teleop_release_resume_required"


def test_external_strategy_stuck_recovery_defers_post_action_until_thread_done(monkeypatch):
    nav = Navigation()
    nav._state = MissionState.EXECUTING
    nav._using_external_strategy_path = True
    nav._tracker = _StuckTracker()
    calls: list[tuple[str, str]] = []
    monkeypatch.setattr(
        nav,
        "_execute_recovery_motion",
        lambda post_action="replan": calls.append(("execute", post_action)) or True,
    )
    monkeypatch.setattr(
        nav,
        "_finish_recovery_motion",
        lambda post_action, reason: calls.append(("finish", post_action)),
    )

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(0.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    assert calls == [("execute", "external_strategy")]


def test_navigation_preview_rejects_non_map_odometry_frame():
    nav = Navigation()
    nav._planner_svc = _RecordingPlanner()
    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(1.0, 2.0, 0.0), orientation=Quaternion()),
        frame_id="odom",
    ))

    preview = nav.preview_plan(3.0, 4.0)

    assert preview["feasible"] is False
    assert preview["reasons"] == ["frame_mismatch"]
    assert preview["error"] == "unsupported odometry frame 'odom'; expected 'map'"


def test_external_strategy_path_reanchors_near_current_odometry():
    # Tight waypoint thresholds keep this test focused on re-anchoring
    # geometry: with the default 1.5m threshold, WaypointTracker's
    # multi-waypoint catch-up (see WaypointTracker.update) would consume
    # this whole ~1m-spaced fixture path in a single update() call before
    # the mission ever reaches EXECUTING.
    nav = Navigation(external_strategy_path_control=True,
        external_strategy_start_tolerance_m=1.0,
        waypoint_threshold=0.05,
        final_waypoint_threshold=0.05,
    )
    global_paths: list[list[np.ndarray]] = []
    waypoints: list[PoseStamped] = []
    nav.global_path._add_callback(global_paths.append)
    nav.waypoint._add_callback(waypoints.append)
    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(6.82, 1.54, 0.75), orientation=Quaternion()),
        frame_id="map",
    ))

    nav._on_patrol_goals([
        {"x": 2.2, "y": -0.2, "z": 0.75, "frame_id": "map"},
        {"x": 7.0, "y": 1.0, "z": 0.75, "frame_id": "map"},
        {"x": 8.0, "y": 1.0, "z": 0.75, "frame_id": "map"},
    ])

    assert nav._state == "EXECUTING"
    assert len(global_paths) == 1
    np.testing.assert_allclose(global_paths[-1][0], np.array([6.82, 1.54, 0.75]))
    assert not np.allclose(global_paths[-1][1][:2], np.array([2.2, -0.2]))
    assert waypoints[-1].pose.position.x >= 6.82


def test_external_strategy_path_rejects_unanchored_paths():
    nav = Navigation(external_strategy_path_control=True,
        external_strategy_start_tolerance_m=0.5,
    )
    global_paths: list[list[np.ndarray]] = []
    waypoints: list[PoseStamped] = []
    nav.global_path._add_callback(global_paths.append)
    nav.waypoint._add_callback(waypoints.append)
    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(0.0, 0.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    nav._on_patrol_goals([
        {"x": 10.0, "y": 0.0, "z": 0.0, "frame_id": "map"},
        {"x": 11.0, "y": 0.0, "z": 0.0, "frame_id": "map"},
    ])

    assert nav._state == "FAILED"
    assert "path_not_anchored_near_current_odom" in (nav._failure_reason or "")
    assert global_paths == []
    assert waypoints == []

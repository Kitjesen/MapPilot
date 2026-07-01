from __future__ import annotations

import pickle
from pathlib import Path

import numpy as np
import pytest

from nav.services.plan.global_planner.service import GlobalPlanner
from nav.mission.navigation import Navigation
from nav.services.safety.plan_safety import (
    PlanSafetyGrid,
    evaluate_backend_path_safety,
    evaluate_plan_safety,
    grid_from_tomogram,
)


def test_plan_safety_supports_raw_tomogram_xy_order() -> None:
    grid = np.zeros((4, 3), dtype=np.float32)
    grid[1, 0] = 100.0
    tomo = {
        "data": grid.reshape(1, 1, 4, 3),
        "resolution": 1.0,
        "origin": [0.0, 0.0],
    }

    report = evaluate_plan_safety(
        [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        grid_from_tomogram(tomo),
        obstacle_thr=49.9,
        max_step_m=0.5,
    )

    assert report["ok"] is False
    assert report["blocked_sample_count"] >= 1
    assert report["index_order"] == "xy"


def test_plan_safety_supports_cmu_flat_tomogram_yx_order() -> None:
    grid = np.zeros((3, 4), dtype=np.float32)
    grid[0, 1] = 100.0
    tomo = {
        "data": grid.reshape(1, 1, 3, 4),
        "resolution": 1.0,
        "origin": [0.0, 0.0],
        "grid_info": {"axis_order": "row_y_col_x"},
    }

    report = evaluate_plan_safety(
        [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        grid_from_tomogram(tomo),
        obstacle_thr=49.9,
        max_step_m=0.5,
    )

    assert report["ok"] is False
    assert report["blocked_sample_count"] >= 1
    assert report["index_order"] == "yx"


def test_plan_safety_supports_backend_yx_order() -> None:
    class Backend:
        _grid = np.zeros((3, 4), dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, 0.0], dtype=float)

    Backend._grid[0, 1] = 100.0

    report = evaluate_backend_path_safety(
        [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        Backend(),
        obstacle_thr=49.9,
    )

    assert report is not None
    assert report["ok"] is False
    assert report["index_order"] == "yx"


def test_plan_safety_uses_backend_3d_tomogram_slice() -> None:
    class Backend:
        _grid = np.zeros((3, 3), dtype=np.float32)
        _trav_3d = np.zeros((2, 3, 3), dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, 0.0], dtype=float)
        _slice_h0 = 0.0
        _slice_dh = 1.0
        _grid_is_projection = True

    Backend._trav_3d[0, 1, 1] = 100.0

    blocked_ground = evaluate_backend_path_safety(
        [[1.0, 1.0, 0.0]],
        Backend(),
        obstacle_thr=49.9,
    )
    clear_upper = evaluate_backend_path_safety(
        [[1.0, 1.0, 1.0]],
        Backend(),
        obstacle_thr=49.9,
    )

    assert blocked_ground is not None
    assert clear_upper is not None
    assert blocked_ground["ok"] is False
    assert clear_upper["ok"] is True


def test_plan_safety_uses_backend_elevation_layers_for_pct_height_samples() -> None:
    class Backend:
        _grid = np.zeros((3, 3), dtype=np.float32)
        _trav_3d = np.zeros((2, 3, 3), dtype=np.float32)
        _elev_3d = np.zeros((2, 3, 3), dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, 0.0], dtype=float)
        _slice_h0 = 0.0
        _slice_dh = 0.25
        _grid_is_projection = True

    Backend._elev_3d[1, :, :] = 2.5

    safe_elevated = evaluate_backend_path_safety(
        [[1.0, 1.0, 2.5]],
        Backend(),
        obstacle_thr=49.9,
    )
    far_above = evaluate_backend_path_safety(
        [[1.0, 1.0, 5.0]],
        Backend(),
        obstacle_thr=49.9,
    )

    assert safe_elevated is not None
    assert far_above is not None
    assert safe_elevated["ok"] is True
    assert far_above["ok"] is False
    assert far_above["blocked_samples"][0]["reason"] == "z_out_of_bounds"


def test_plan_safety_clamps_near_ground_height_to_first_tomogram_slice() -> None:
    grid = PlanSafetyGrid(
        grid=np.zeros((3, 3), dtype=np.float32),
        resolution=1.0,
        origin=(0.0, 0.0),
        trav_3d=np.zeros((2, 3, 3), dtype=np.float32),
        slice_h0=0.5,
        slice_dh=0.5,
        use_grid_overlay=True,
    )

    near_ground = evaluate_plan_safety(
        [[1.0, 1.0, 0.06]],
        grid,
        obstacle_thr=49.9,
    )
    far_below = evaluate_plan_safety(
        [[1.0, 1.0, -1.0]],
        grid,
        obstacle_thr=49.9,
    )

    assert near_ground["ok"] is True
    assert far_below["ok"] is False


def test_plan_safety_reports_z_out_of_bounds_samples() -> None:
    grid = PlanSafetyGrid(
        grid=np.zeros((3, 3), dtype=np.float32),
        resolution=1.0,
        origin=(0.0, 0.0),
        trav_3d=np.zeros((2, 3, 3), dtype=np.float32),
        slice_h0=0.28586,
        slice_dh=0.3,
    )

    report = evaluate_plan_safety(
        [[1.0, 1.0, 1.1859]],
        grid,
        obstacle_thr=49.9,
    )

    assert report["ok"] is False
    assert report["blocked_sample_count"] == 1
    blocked = report["blocked_samples"][0]
    assert blocked["reason"] == "z_out_of_bounds"
    assert blocked["layer"] == 3
    assert blocked["row"] == 1
    assert blocked["col"] == 1


def test_pct_wrapper_preserves_optimizer_height_when_converting_xy() -> None:
    root = Path(__file__).resolve().parents[3]
    wrapper = root / "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/scripts/planner_wrapper.py"
    text = wrapper.read_text(encoding="utf-8")

    assert "world[:, 2] = heights" in text
    assert "heights / self.resolution" not in text
    assert "transTrajGrid2Map(self.map_dim, self.center, self.resolution, traj_3d)" not in text


def test_pct_backend_rejects_goal_height_that_only_matches_other_floor() -> None:
    from nav.services.plan.global_planner.algorithm.pct.planner import PCTPlanner

    class FakePlanner:
        def __init__(self) -> None:
            self.layers_t = np.zeros((2, 3, 3), dtype=np.float32)
            self.layers_t[0, 1, 1] = 100.0
            self.layers_g = np.zeros_like(self.layers_t)
            self.layers_g[1, :, :] = 3.0
            self.calls: list[tuple[float, float]] = []

        def pos2idx(self, pos):
            return np.asarray([round(float(pos[0])), round(float(pos[1]))], dtype=float)

        def pos2slice(self, z):
            return float(z) / 3.0

        def get_surface_height(self, _pos):
            return 3.0

        def plan(self, _start_pos, _goal_pos, start_h, goal_h):
            self.calls.append((float(start_h), float(goal_h)))
            return np.asarray([[0.0, 0.0, start_h], [1.0, 1.0, goal_h]], dtype=float)

    planner = FakePlanner()
    backend = PCTPlanner.__new__(PCTPlanner)
    backend._planner = planner
    backend._load_error = None
    backend._obstacle_thr = 49.9
    backend._resolution = 1.0
    backend._origin = np.asarray([0.0, 0.0], dtype=float)
    backend._slice_h0 = 0.0
    backend._slice_dh = 3.0
    backend._trav_3d = planner.layers_t

    path = backend.plan(
        np.asarray([0.0, 0.0, 0.0], dtype=float),
        np.asarray([1.0, 1.0, 0.0], dtype=float),
    )

    assert path == []
    assert planner.calls == []


def test_pct_backend_height_snap_limit_matches_25cm_step_limit() -> None:
    from nav.services.plan.global_planner.algorithm.pct.planner import PCTPlanner

    class FakePlanner:
        def __init__(self, height: float) -> None:
            self.height = height
            self.layers_t = np.zeros((2, 3, 3), dtype=np.float32)
            self.layers_t[0, 1, 1] = 100.0
            self.layers_g = np.zeros_like(self.layers_t)
            self.layers_g[1, :, :] = height
            self.calls: list[tuple[float, float]] = []

        def pos2idx(self, pos):
            return np.asarray([round(float(pos[0])), round(float(pos[1]))], dtype=float)

        def pos2slice(self, z):
            return float(z) / self.height

        def get_surface_height(self, _pos):
            return self.height

        def plan(self, _start_pos, _goal_pos, start_h, goal_h):
            self.calls.append((float(start_h), float(goal_h)))
            return np.asarray([[0.0, 0.0, start_h], [1.0, 1.0, goal_h]], dtype=float)

    def make_backend(height: float):
        planner = FakePlanner(height)
        backend = PCTPlanner.__new__(PCTPlanner)
        backend._planner = planner
        backend._load_error = None
        backend._obstacle_thr = 49.9
        backend._resolution = 1.0
        backend._origin = np.asarray([0.0, 0.0], dtype=float)
        backend._slice_h0 = 0.0
        backend._slice_dh = height
        backend._trav_3d = planner.layers_t
        return backend, planner

    near_backend, near_planner = make_backend(0.24)
    near_path = near_backend.plan(
        np.asarray([0.0, 0.0, 0.0], dtype=float),
        np.asarray([1.0, 1.0, 0.0], dtype=float),
    )

    far_backend, far_planner = make_backend(0.30)
    far_path = far_backend.plan(
        np.asarray([0.0, 0.0, 0.0], dtype=float),
        np.asarray([1.0, 1.0, 0.0], dtype=float),
    )

    assert near_path
    assert near_planner.calls
    assert abs(near_path[-1][2] - 0.24) < 1e-6
    assert far_path == []
    assert far_planner.calls == []


def test_global_planner_does_not_append_unreachable_goal_to_partial_path() -> None:
    class PartialBackend:
        _grid = np.zeros((1, 10), dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, 0.0], dtype=float)
        _last_plan_reached_goal = False

        def plan(self, start, goal):
            return [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]]

    svc = GlobalPlanner(
        planner_name="astar",
        obstacle_thr=49.9,
        downsample_dist=0.1,
        plan_safety_policy="fallback_astar",
    )
    svc._backend = PartialBackend()

    path, _plan_ms = svc.plan(
        np.asarray([0.0, 0.0, 0.0], dtype=float),
        np.asarray([9.0, 0.0, 0.0], dtype=float),
        safe_goal_tolerance=0.0,
    )

    assert [float(v) for v in path[-1][:3]] == [2.0, 0.0, 0.0]
    assert svc.last_plan_report["reached_goal"] is False


def test_global_planner_can_fallback_after_unsafe_plan(monkeypatch: pytest.MonkeyPatch) -> None:
    class UnsafeBackend:
        available = True
        _grid = np.zeros((3, 3), dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, 0.0], dtype=float)

        def plan(self, start, goal):
            return [[0.0, 1.0, 0.0], [2.0, 1.0, 0.0]]

    UnsafeBackend._grid[1, 1] = 100.0

    class SafeBackend:
        available = True
        _grid = UnsafeBackend._grid
        _resolution = 1.0
        _origin = UnsafeBackend._origin

        def plan(self, start, goal):
            return [[0.0, 1.0, 0.0], [0.0, 2.0, 0.0], [2.0, 2.0, 0.0], [2.0, 1.0, 0.0]]

    svc = GlobalPlanner(
        planner_name="pct",
        obstacle_thr=49.9,
        downsample_dist=0.1,
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
    )
    svc._backend = UnsafeBackend()
    monkeypatch.setattr(svc, "_create_backend", lambda name=None: SafeBackend())

    path, _plan_ms = svc.plan(
        np.asarray([0.0, 1.0, 0.0], dtype=float),
        np.asarray([2.0, 1.0, 0.0], dtype=float),
        safe_goal_tolerance=0.0,
    )

    assert [[float(p[0]), float(p[1])] for p in path] == [[0.0, 1.0], [0.0, 2.0], [2.0, 2.0], [2.0, 1.0]]
    report = svc.last_plan_report
    assert report["selected_planner"] == "astar"
    assert report["rejected_plans"][0]["planner"] == "pct"
    assert report["selected_path_safety"]["ok"] is True


def test_global_planner_retries_primary_nearby_goal_before_fallback() -> None:
    class CandidateAwareBackend:
        available = True
        _grid = np.zeros((3, 6), dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, 0.0], dtype=float)
        _last_plan_reached_goal = True

        def plan(self, start, goal):
            gy = float(goal[1])
            if abs(gy) < 0.5:
                return [[0.0, 0.0, 0.0], [4.0, 0.0, 0.0]]
            return [[0.0, 0.0, 0.0], [4.0, gy, 0.0]]

    CandidateAwareBackend._grid[0, 2] = 100.0

    svc = GlobalPlanner(
        planner_name="pct",
        obstacle_thr=49.9,
        downsample_dist=0.1,
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
    )
    svc._backend = CandidateAwareBackend()

    path, _plan_ms = svc.plan(
        np.asarray([0.0, 0.0, 0.0], dtype=float),
        np.asarray([4.0, 0.0, 0.0], dtype=float),
        safe_goal_tolerance=2.0,
    )

    report = svc.last_plan_report
    assert report["selected_planner"] == "pct"
    assert report["fallback_reason"] == ""
    assert report["rejected_plans"] == []
    assert report["selected_path_safety"]["ok"] is True
    assert report["primary_replan"]["used"] is True
    assert abs(float(path[-1][1])) >= 1.0


def test_global_planner_uses_safe_primary_prefix_when_full_pct_path_is_blocked() -> None:
    class PrefixOnlyBackend:
        available = True
        _grid = np.zeros((3, 8), dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, -1.0], dtype=float)
        _last_plan_reached_goal = True

        def plan(self, start, goal):
            return [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
                [3.0, 0.0, 0.0],
                [4.0, 0.0, 0.0],
                [7.0, 0.0, 0.0],
            ]

    PrefixOnlyBackend._grid[1, 4] = 100.0

    svc = GlobalPlanner(
        planner_name="pct",
        obstacle_thr=49.9,
        downsample_dist=0.1,
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
    )
    svc._backend = PrefixOnlyBackend()

    path, _plan_ms = svc.plan(
        np.asarray([0.0, 0.0, 0.0], dtype=float),
        np.asarray([7.0, 0.0, 0.0], dtype=float),
        safe_goal_tolerance=0.0,
    )

    report = svc.last_plan_report
    assert report["selected_planner"] == "pct"
    assert report["fallback_reason"] == ""
    assert report["reached_goal"] is False
    assert report["selected_path_safety"]["ok"] is True
    assert report["primary_replan"]["candidate_source"] == "safe_prefix"
    assert [float(v) for v in path[-1][:2]] == [3.0, 0.0]


def test_global_planner_reject_policy_does_not_use_safe_prefix_repair() -> None:
    class PrefixOnlyBackend:
        available = True
        _grid = np.zeros((3, 8), dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, -1.0], dtype=float)
        _last_plan_reached_goal = True

        def plan(self, start, goal):
            return [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
                [3.0, 0.0, 0.0],
                [4.0, 0.0, 0.0],
                [7.0, 0.0, 0.0],
            ]

    PrefixOnlyBackend._grid[1, 4] = 100.0

    svc = GlobalPlanner(
        planner_name="pct",
        obstacle_thr=49.9,
        downsample_dist=0.1,
        plan_safety_policy="reject",
        fallback_planner_name="astar",
    )
    svc._backend = PrefixOnlyBackend()

    with pytest.raises(RuntimeError, match="path_safety failed"):
        svc.plan(
            np.asarray([0.0, 0.0, 0.0], dtype=float),
            np.asarray([7.0, 0.0, 0.0], dtype=float),
            safe_goal_tolerance=0.0,
        )

    report = svc.last_plan_report
    assert report["selected_planner"] == "pct"
    assert report["fallback_reason"].startswith("pct path_safety failed")
    assert "primary_replan" not in report
    assert report["rejected_plans"]


def test_global_planner_retries_primary_reachable_goal_after_empty_path() -> None:
    class EmptyUntilCloserBackend:
        available = True
        _grid = np.zeros((11, 11), dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, -5.0], dtype=float)
        _last_plan_reached_goal = True

        def plan(self, start, goal):
            if float(goal[0]) > 3.0:
                return []
            return [
                [float(start[0]), float(start[1]), float(start[2])],
                [float(goal[0]), float(goal[1]), float(goal[2])],
            ]

    svc = GlobalPlanner(
        planner_name="pct",
        obstacle_thr=49.9,
        downsample_dist=0.1,
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
    )
    svc._backend = EmptyUntilCloserBackend()

    path, _plan_ms = svc.plan(
        np.asarray([0.0, 0.0, 0.0], dtype=float),
        np.asarray([6.0, 0.0, 0.0], dtype=float),
        safe_goal_tolerance=6.0,
    )

    report = svc.last_plan_report
    assert report["selected_planner"] == "pct"
    assert report["fallback_reason"] == ""
    assert report["rejected_plans"] == []
    assert report["primary_replan"]["used"] is True
    assert report["primary_replan"]["reason"] == "initial_primary_empty_path"
    assert float(path[-1][0]) <= 3.0


def test_global_planner_projects_empty_path_to_reachable_component() -> None:
    class ComponentAwareBackend:
        available = True
        _grid = np.zeros((5, 7), dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, 0.0], dtype=float)
        _last_plan_reached_goal = True

        def plan(self, start, goal):
            gx = int(round(float(goal[0])))
            gy = int(round(float(goal[1])))
            if gx > 2 or gy == 1:
                return []
            return [
                [float(start[0]), float(start[1]), float(start[2])],
                [0.0, 2.0, 0.0],
                [float(goal[0]), float(goal[1]), float(goal[2])],
            ]

    ComponentAwareBackend._grid[:, 3] = 100.0
    ComponentAwareBackend._grid[1, 1] = 100.0
    ComponentAwareBackend._grid[1, 2] = 100.0

    svc = GlobalPlanner(
        planner_name="pct",
        obstacle_thr=49.9,
        downsample_dist=0.1,
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
    )
    svc._backend = ComponentAwareBackend()

    path, _plan_ms = svc.plan(
        np.asarray([0.0, 1.0, 0.0], dtype=float),
        np.asarray([6.0, 1.0, 0.0], dtype=float),
        safe_goal_tolerance=8.0,
    )

    report = svc.last_plan_report
    assert report["selected_planner"] == "pct"
    assert report["fallback_reason"] == ""
    assert report["rejected_plans"] == []
    assert report["primary_replan"]["used"] is True
    assert report["primary_replan"]["candidate_source"] in {
        "nearby_ring",
        "reachable_component",
    }
    assert float(path[-1][0]) <= 2.0
    assert int(round(float(path[-1][1]))) != 1


def test_global_planner_reports_when_fallback_is_unsafe(monkeypatch: pytest.MonkeyPatch) -> None:
    class UnsafeBackend:
        _grid = np.zeros((3, 3), dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, 0.0], dtype=float)

        def plan(self, start, goal):
            return [[0.0, 1.0, 0.0], [2.0, 1.0, 0.0]]

    UnsafeBackend._grid[1, 1] = 100.0

    svc = GlobalPlanner(
        planner_name="pct",
        obstacle_thr=49.9,
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
    )
    svc._backend = UnsafeBackend()
    monkeypatch.setattr(svc, "_create_backend", lambda name=None: UnsafeBackend())

    with pytest.raises(RuntimeError, match="fallback planner did not produce a safe path"):
        svc.plan(
            np.asarray([0.0, 1.0, 0.0], dtype=float),
            np.asarray([2.0, 1.0, 0.0], dtype=float),
            safe_goal_tolerance=0.0,
        )

    report = svc.last_plan_report
    assert report["policy"] == "fallback_astar"
    assert [entry["planner"] for entry in report["rejected_plans"]] == ["pct", "astar"]


def test_global_planner_applies_live_map_to_late_fallback_backend(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    live_grid = np.zeros((3, 3), dtype=np.float32)
    live_grid[1, 1] = 100.0

    class UnsafePrimaryBackend:
        _grid = live_grid
        _resolution = 1.0
        _origin = np.asarray([0.0, 0.0], dtype=float)

        def plan(self, start, goal):
            return [[0.0, 1.0, 0.0], [2.0, 1.0, 0.0]]

    class LateFallbackBackend:
        def __init__(self):
            self.update_calls = 0
            self._grid = None
            self._resolution = 1.0
            self._origin = np.asarray([0.0, 0.0], dtype=float)

        def update_map(self, grid, resolution=0.2, origin=None):
            self.update_calls += 1
            self._grid = np.asarray(grid, dtype=np.float32)
            self._resolution = float(resolution)
            self._origin = np.asarray(origin[:2], dtype=float) if origin is not None else np.zeros(2)

        def plan(self, start, goal):
            assert self.update_calls == 1
            assert self._grid is not None
            return [[0.0, 1.0, 0.0], [0.0, 2.0, 0.0], [2.0, 2.0, 0.0], [2.0, 1.0, 0.0]]

    svc = GlobalPlanner(
        planner_name="pct",
        obstacle_thr=49.9,
        downsample_dist=0.1,
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
    )
    svc._backend = UnsafePrimaryBackend()
    svc.update_map(live_grid, resolution=1.0, origin=np.asarray([0.0, 0.0], dtype=float))

    monkeypatch.setattr(svc, "_create_backend", lambda name=None: LateFallbackBackend())

    path, _plan_ms = svc.plan(
        np.asarray([0.0, 1.0, 0.0], dtype=float),
        np.asarray([2.0, 1.0, 0.0], dtype=float),
        safe_goal_tolerance=0.0,
    )

    assert svc._fallback_backend.update_calls == 1
    assert [[float(p[0]), float(p[1])] for p in path] == [[0.0, 1.0], [0.0, 2.0], [2.0, 2.0], [2.0, 1.0]]
    assert svc.last_plan_report["primary_planner"] == "pct"
    assert svc.last_plan_report["selected_planner"] == "astar"


def test_global_planner_can_reject_unsafe_plan() -> None:
    class UnsafeBackend:
        _grid = np.zeros((3, 3), dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, 0.0], dtype=float)

        def plan(self, start, goal):
            return [[0.0, 1.0, 0.0], [2.0, 1.0, 0.0]]

    UnsafeBackend._grid[1, 1] = 100.0

    svc = GlobalPlanner(plan_safety_policy="reject", obstacle_thr=49.9)
    svc._backend = UnsafeBackend()

    with pytest.raises(RuntimeError, match="path_safety failed"):
        svc.plan(
            np.asarray([0.0, 1.0, 0.0], dtype=float),
            np.asarray([2.0, 1.0, 0.0], dtype=float),
            safe_goal_tolerance=0.0,
        )


def test_global_planner_fallback_policy_rejects_unsafe_astar_without_distinct_fallback() -> None:
    class UnsafeAstarBackend:
        _grid = np.zeros((3, 3), dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, 0.0], dtype=float)

        def plan(self, start, goal):
            return [[0.0, 1.0, 0.0], [2.0, 1.0, 0.0]]

    UnsafeAstarBackend._grid[1, 1] = 100.0

    svc = GlobalPlanner(
        planner_name="astar",
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
        obstacle_thr=49.9,
    )
    svc._backend = UnsafeAstarBackend()

    with pytest.raises(RuntimeError, match="no distinct fallback planner"):
        svc.plan(
            np.asarray([0.0, 1.0, 0.0], dtype=float),
            np.asarray([2.0, 1.0, 0.0], dtype=float),
            safe_goal_tolerance=0.0,
        )
    assert svc.last_plan_report["selected_path_safety"]["ok"] is False
    assert svc.last_plan_report["primary_planner"] == "astar"


def test_global_planner_rejects_unreachable_goal_when_grid_exists() -> None:
    class BlockedBackend:
        _grid = np.full((5, 5), 100.0, dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, 0.0], dtype=float)

        def plan(self, start, goal):
            raise AssertionError("planner should not run when goal has no reachable free cell")

    svc = GlobalPlanner(
        planner_name="astar",
        plan_safety_policy="fallback_astar",
        obstacle_thr=49.9,
    )
    svc._backend = BlockedBackend()

    with pytest.raises(RuntimeError, match="no reachable free cell"):
        svc.plan(
            np.asarray([0.0, 0.0, 0.0], dtype=float),
            np.asarray([2.0, 2.0, 0.0], dtype=float),
            safe_goal_tolerance=1.0,
        )

    report = svc.last_plan_report
    assert report["fallback_reason"].startswith("goal has no reachable free cell")
    assert report["reached_goal"] is False
    assert report["rejected_plans"][0]["planner"] == "astar"


def test_global_planner_reject_policy_does_not_repair_empty_path() -> None:
    class EmptyBackend:
        _grid = np.zeros((10, 10), dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, 0.0], dtype=float)

        def __init__(self) -> None:
            self.calls = 0

        def plan(self, start, goal):
            self.calls += 1
            return []

    backend = EmptyBackend()
    svc = GlobalPlanner(
        planner_name="astar",
        plan_safety_policy="reject",
        obstacle_thr=49.9,
    )
    svc._backend = backend

    with pytest.raises(RuntimeError, match="planner returned empty path"):
        svc.plan(
            np.asarray([0.0, 0.0, 0.0], dtype=float),
            np.asarray([4.0, 4.0, 0.0], dtype=float),
            safe_goal_tolerance=2.0,
        )

    assert backend.calls == 1
    assert "primary_replan" not in svc.last_plan_report
    assert svc.last_plan_report["policy"] == "reject"


def test_pct_global_planner_requires_same_source_tomogram_gate(tmp_path: Path) -> None:
    map_dir = tmp_path / "bad_map"
    map_dir.mkdir()
    (map_dir / "map.pcd").write_text("VERSION 0.7\nPOINTS 0\nDATA ascii\n")
    tomogram = map_dir / "tomogram.pickle"
    tomogram.write_bytes(b"not-a-real-tomogram")

    class Backend:
        _grid = np.zeros((2, 2), dtype=np.uint8)
        _resolution = 1.0
        _origin = np.zeros(2)
        _last_plan_reached_goal = True

        def plan(self, start, goal):
            return [np.asarray(start, dtype=float), np.asarray(goal, dtype=float)]

    svc = GlobalPlanner(
        planner_name="pct",
        tomogram=str(tomogram),
        plan_safety_policy="reject",
    )
    svc._create_backend = lambda name=None: Backend()
    svc.setup()

    gate = svc.map_artifact_gate
    assert gate["required"] is True
    assert gate["ok"] is False
    assert "metadata.json missing" in gate["blockers"]

    with pytest.raises(RuntimeError, match="saved map artifact gate failed"):
        svc.plan(
            np.asarray([0.0, 0.0, 0.0], dtype=float),
            np.asarray([1.0, 1.0, 0.0], dtype=float),
        )

    report = svc.last_plan_report
    assert report["selected_planner"] == "pct"
    assert "metadata.json missing" in report["fallback_reason"]


def test_navigation_exposes_plan_safety_policy() -> None:
    nav = Navigation(
        planner="pct",
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
    )

    summary = nav.health()["navigation"]

    assert summary["planner"] == "pct"
    assert summary["plan_safety_policy"] == "fallback_astar"


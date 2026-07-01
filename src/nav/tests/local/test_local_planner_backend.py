from __future__ import annotations

import os
from types import SimpleNamespace
from typing import Any

import nav.services.plan.local_planner.cmu_py as cmu_py
from nav.services.plan.local_planner.backend import (
    CmuPyLocalPlannerRequest,
    create_cmu_py_backend,
    create_nanobind_backend,
    local_planner_paths_dir,
    plan_cmu_py_local_path,
    read_local_planner_frame_params,
    read_local_planner_grid_config,
    read_local_planner_python_params,
    score_cmu_py_paths,
)
from runtime.msgs.numpy_compat import np


class FakeLocalPlannerParams:
    def __init__(self) -> None:
        self.vehicle_length = 0.6
        self.vehicle_width = 0.6
        self.sensor_offset_x = 0.0
        self.sensor_offset_y = 0.0
        self.obstacle_height_thre = 0.2
        self.ground_height_thre = 0.1
        self.max_speed = 1.0
        self.autonomy_speed = 1.0
        self.two_way_drive = True
        self.adjacent_range = 3.5
        self.check_obstacle = True
        self.check_rot_obstacle = False
        self.use_cost = False
        self.use_terrain_analysis = True
        self.point_per_path_thre = 2
        self.dir_weight = 0.02
        self.dir_thre = 90.0
        self.path_scale = 1.0
        self.min_path_scale = 0.75
        self.path_scale_step = 0.25
        self.path_scale_by_speed = True
        self.min_path_range = 1.0
        self.path_range_step = 0.5
        self.path_range_by_speed = True
        self.path_crop_by_goal = True
        self.slope_weight = 0.0
        self.goal_clear_range = 0.5
        self.near_field_stop_dis = 0.5
        self.recovery_blocked_thre = 2.0
        self.recovery_rotate_time = 2.5
        self.recovery_backup_time = 1.5
        self.recovery_max_cycles = 3
        self.cost_height_thre_1 = 0.15
        self.cost_height_thre_2 = 0.1
        self.dir_to_vehicle = False
        self.goal_behind_range = 0.8
        self.freeze_ang = 90.0
        self.freeze_time = 2.0
        self.omni_dir_goal_thre = 1.0
        self.slow_path_num_thre = 5
        self.slow_group_num_thre = 1
        self.use_traversability_cost = True
        self.traversability_hard_cost = 90.0
        self.traversability_soft_cost = 40.0
        self.traversability_weight = 0.01
        self.min_rel_z = -0.5
        self.max_rel_z = 0.25


class FakeLocalPlanner:
    def __init__(self, params: FakeLocalPlannerParams) -> None:
        self.params = params
        self.loaded_paths_dir: str | None = None

    def load_paths(self, paths_dir: str) -> bool:
        self.loaded_paths_dir = paths_dir
        return True


class FakeNavKernel:
    LocalPlannerParams = FakeLocalPlannerParams
    LocalPlanner = FakeLocalPlanner


def test_nanobind_adapter_assembles_fake_local_planner_params() -> None:
    cfg = SimpleNamespace(
        raw={
            "local_planner": {
                "adjacent_range": 4.25,
                "point_per_path_thre": 7,
                "min_rel_z": -0.35,
                "use_cost": True,
                "traversability_hard_cost": 88.0,
            },
            "terrain": {"max_rel_z": 0.42},
        },
        geometry=SimpleNamespace(vehicle_length=0.72, vehicle_width=0.38),
        safety=SimpleNamespace(obstacle_height_thre=0.31),
        speed=SimpleNamespace(max_speed=1.7, autonomy_speed=1.2),
    )
    paths_dir = os.path.join("tmp", "local_planner", "paths")

    result = create_nanobind_backend(
        nav_kernel_importer=lambda _symbols: FakeNavKernel,
        build_hint_provider=lambda: "build hint",
        config_getter=lambda: cfg,
        paths_dir=paths_dir,
    )

    assert result.core is not None
    assert result.unavailable_reason is None
    assert result.missing_params == []
    assert result.effective_params["vehicle_length"] == 0.72
    assert result.effective_params["vehicle_width"] == 0.38
    assert result.effective_params["obstacle_height_thre"] == 0.31
    assert result.effective_params["max_speed"] == 1.7
    assert result.effective_params["autonomy_speed"] == 1.2
    assert result.effective_params["adjacent_range"] == 4.25
    assert result.effective_params["point_per_path_thre"] == 7
    assert result.effective_params["min_rel_z"] == -0.35
    assert result.effective_params["use_cost"] is True
    assert result.effective_params["traversability_hard_cost"] == 88.0
    assert result.effective_params["use_traversability_cost"] is True
    assert result.effective_params["max_rel_z"] == 0.42
    assert result.core.params.adjacent_range == 4.25
    assert result.core.params.point_per_path_thre == 7
    assert result.core.params.use_cost is True
    assert result.core.params.traversability_hard_cost == 88.0
    assert result.core.loaded_paths_dir == os.path.normpath(paths_dir)


def test_local_planner_paths_dir_resolves_shared_cmu_paths_directory() -> None:
    paths_dir = local_planner_paths_dir()

    assert paths_dir.endswith(
        os.path.normpath("src/nav/services/plan/local_planner/paths")
    )
    assert os.path.isdir(paths_dir)
    assert os.path.exists(os.path.join(paths_dir, "paths.ply"))


def test_cmu_py_adapter_loads_default_paths_dir_with_optional_nav_kernel() -> None:
    seen: dict[str, Any] = {}

    def fake_loader(paths_dir: str) -> dict[str, Any]:
        seen["paths_dir"] = paths_dir
        return {"paths": [], "correspondences": {}}

    result = create_cmu_py_backend(
        nav_kernel_importer=lambda _symbols: None,
        path_loader=fake_loader,
    )

    assert result.nav_kernel is None
    assert result.path_data == {"paths": [], "correspondences": {}}
    assert result.paths_dir == local_planner_paths_dir()
    assert seen["paths_dir"] == local_planner_paths_dir()


def _run_cmu_py_adapter_plan(obstacle_half_width: float):
    path_data = create_cmu_py_backend(nav_kernel_importer=lambda _symbols: None).path_data
    obstacle_points = []
    for x in np.linspace(1.0, 1.8, 5):
        samples = max(3, int(obstacle_half_width * 20) + 1)
        for y in np.linspace(-obstacle_half_width, obstacle_half_width, samples):
            obstacle_points.append([x, y, 0.0, 200.0])
    obstacles = np.asarray(obstacle_points, dtype=np.float32)

    decision = plan_cmu_py_local_path(
        CmuPyLocalPlannerRequest(
            path_data=path_data,
            robot_pos=np.asarray([0.0, 0.0, 0.0], dtype=float),
            robot_yaw=0.0,
            goal=np.asarray([4.0, 0.0, 0.0], dtype=float),
            obstacle_points_world=obstacles,
        )
    )
    return decision, obstacles[:, :2].astype(float)


def test_cmu_py_runtime_adapter_routes_around_feasible_added_obstacle() -> None:
    decision, obstacle_xy = _run_cmu_py_adapter_plan(obstacle_half_width=0.15)

    assert decision is not None
    assert decision.path_found is True
    assert decision.safety_stop is False
    assert decision.reason == "cmu_py_path"
    path_xy = np.asarray([(x, y) for x, y, _z in decision.world_path], dtype=float)
    assert len(path_xy) > 2
    assert float(np.max(np.abs(path_xy[:, 1]))) > 0.25
    min_clearance = float(
        np.min(np.linalg.norm(path_xy[:, None, :] - obstacle_xy[None, :, :], axis=2))
    )
    assert min_clearance > 0.30


def test_cmu_py_runtime_adapter_reports_safety_stop_when_blocked() -> None:
    decision, _obstacle_xy = _run_cmu_py_adapter_plan(obstacle_half_width=0.30)

    assert decision is not None
    assert decision.path_found is False
    assert decision.safety_stop is True
    assert decision.reason == "no_feasible_local_path"
    assert decision.world_path == []


def test_nanobind_adapter_returns_degraded_result_when_nav_kernel_missing() -> None:
    result = create_nanobind_backend(
        nav_kernel_importer=lambda _symbols: None,
        build_hint_provider=lambda: "run build",
    )

    assert result.core is None
    assert result.unavailable_reason == "compatible LingTu native navigation kernel missing"
    assert result.build_hint == "run build"
    assert result.effective_params == {}


def test_local_planner_grid_config_reads_adapter_owned_config_section() -> None:
    cfg = SimpleNamespace(
        raw={
            "local_planner_grid": {
                "voxel_size": "0.05",
                "x_offset": "2.8",
                "y_offset": "4.6",
                "search_radius": "0.7",
            }
        }
    )

    grid = read_local_planner_grid_config(config_getter=lambda: cfg)

    assert grid.loaded_from_config is True
    assert grid.voxel_size == 0.05
    assert grid.x_offset == 2.8
    assert grid.y_offset == 4.6
    assert grid.search_radius == 0.7


def test_local_planner_grid_config_keeps_defaults_without_section() -> None:
    grid = read_local_planner_grid_config(
        config_getter=lambda: SimpleNamespace(raw={})
    )

    assert grid.loaded_from_config is False
    assert grid.voxel_size == 0.02
    assert grid.x_offset == 3.2
    assert grid.y_offset == 5.25
    assert grid.search_radius == 0.45


def test_local_planner_frame_params_read_sensor_offset_from_geometry() -> None:
    cfg = SimpleNamespace(
        geometry=SimpleNamespace(sensor_offset_x=0.31, sensor_offset_y=-0.02)
    )

    params = read_local_planner_frame_params(config_getter=lambda: cfg)

    assert params == {"sensor_offset_x": 0.31, "sensor_offset_y": -0.02}


def test_local_planner_python_params_read_cmu_py_runtime_config() -> None:
    cfg = SimpleNamespace(
        raw={
            "local_planner": {
                "use_cost": True,
                "check_rot_obstacle": True,
                "point_per_path_thre": "4",
                "slope_weight": "3.5",
            }
        },
        geometry=SimpleNamespace(vehicle_length=0.8, vehicle_width=0.4),
        safety=SimpleNamespace(obstacle_height_thre=0.25, ground_height_thre=0.08),
    )

    params = read_local_planner_python_params(config_getter=lambda: cfg)

    assert params["vehicle_length"] == 0.8
    assert params["vehicle_width"] == 0.4
    assert params["obstacle_height_thre"] == 0.25
    assert params["ground_height_thre"] == 0.08
    assert params["point_per_path_thre"] == 4
    assert params["slope_weight"] == 3.5
    assert params["use_cost"] is True
    assert params["check_rot_obstacle"] is True


def test_cmu_py_score_respects_use_cost_gate() -> None:
    group_of_path = np.zeros(343, dtype=np.int32)
    voxel_id = 160 * 531 + 263
    obstacle = np.asarray([[0.0, 0.0, 0.0, 0.15]], dtype=np.float32)

    no_cost = score_cmu_py_paths(
        obstacle,
        4.0,
        0.0,
        4.0,
        0.0,
        {voxel_id: [0]},
        group_of_path,
        161,
        531,
        use_cost=False,
        slope_weight=10.0,
    )
    with_cost = score_cmu_py_paths(
        obstacle,
        4.0,
        0.0,
        4.0,
        0.0,
        {voxel_id: [0]},
        group_of_path,
        161,
        531,
        use_cost=True,
        slope_weight=10.0,
    )

    assert float(np.sum(no_cost)) > 0.0
    assert float(np.sum(with_cost)) < float(np.sum(no_cost))


def test_cmu_py_plan_respects_check_rot_obstacle(monkeypatch) -> None:
    fake_scores = np.zeros(36 * 7, dtype=np.float64)
    fake_scores[27 * 7 + 3] = 100.0
    fake_scores[18 * 7 + 3] = 10.0
    monkeypatch.setattr(cmu_py, "score_cmu_py_paths", lambda *args, **kwargs: fake_scores)
    path_data = {
        "correspondences": {},
        "group_of_path": np.zeros(343, dtype=np.int32),
        "start_paths": [
            np.zeros((0, 3), dtype=np.float32),
            np.zeros((0, 3), dtype=np.float32),
            np.zeros((0, 3), dtype=np.float32),
            np.asarray([[0.5, 0.0, 0.0]], dtype=np.float32),
            np.zeros((0, 3), dtype=np.float32),
            np.zeros((0, 3), dtype=np.float32),
            np.zeros((0, 3), dtype=np.float32),
        ],
        "grid_voxel_num_x": 161,
        "grid_voxel_num_y": 531,
    }

    decision = plan_cmu_py_local_path(
        CmuPyLocalPlannerRequest(
            path_data=path_data,
            robot_pos=np.asarray([0.0, 0.0, 0.0], dtype=float),
            robot_yaw=0.0,
            goal=np.asarray([4.0, 0.0, 0.0], dtype=float),
            obstacle_points_world=np.asarray([[0.0, 0.4, 0.0, 1.0]], dtype=np.float32),
            check_rot_obstacle=True,
            two_way_drive=False,
        )
    )

    assert decision is not None
    assert decision.selected_rot_dir == 18

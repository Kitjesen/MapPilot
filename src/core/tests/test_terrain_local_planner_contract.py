from __future__ import annotations

from pathlib import Path as FilePath
from types import SimpleNamespace

import numpy as np

from base_autonomy.modules.local_planner_module import LocalPlannerModule
from base_autonomy.modules.terrain_module import TerrainModule
from core.config import load_config
from core.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from core.msgs.nav import Odometry, Path
from core.msgs.sensor import PointCloud2
from core.runtime_interface import TOPICS, topic_default_frame_id


_REPO_ROOT = FilePath(__file__).resolve().parents[3]


class _FakeTerrainResult:
    n_points = 2
    terrain_points = [
        1.0,
        0.0,
        0.35,
        0.35,
        1.5,
        0.2,
        0.60,
        0.60,
    ]
    map_width = 2
    map_resolution = 0.1
    elevation_map = [0.0, 0.1, 0.2, 0.3]


class _FakeTerrainCore:
    def process(self, _flat, _ts):
        return _FakeTerrainResult()


class _FakeLocalPathVertex:
    def __init__(self, x: float, y: float, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z


class _FakeLocalPlanResult:
    path = [
        _FakeLocalPathVertex(0.0, 0.0),
        _FakeLocalPathVertex(1.0, 0.0),
    ]
    slow_down = 2
    near_field_stop = True
    path_found = True
    recovery_state = 0


class _FakeLocalPlannerCore:
    captured_params = None

    def __init__(self, params=None):
        type(self).captured_params = params

    def load_paths(self, _paths_dir):
        return True

    def paths_loaded(self):
        return True

    def set_vehicle(self, *_args):
        pass

    def set_goal(self, *_args):
        pass

    def plan(self, *_args):
        return _FakeLocalPlanResult()


class _FakeLocalPlannerParams:
    def __init__(self):
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
        self.use_terrain_analysis = True
        self.point_per_path_thre = 2
        self.min_rel_z = -0.5
        self.max_rel_z = 0.25
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


class _FakeNavCore:
    LocalPlannerParams = _FakeLocalPlannerParams
    LocalPlannerCore = _FakeLocalPlannerCore


def test_terrain_nanobind_preserves_height_intensity_for_local_planner():
    module = TerrainModule(backend="nanobind")
    module._core = _FakeTerrainCore()
    published: list[PointCloud2] = []
    module.terrain_map._add_callback(published.append)

    module._process_nanobind(
        PointCloud2(
            points=np.array([[0.0, 0.0, 0.0]], dtype=np.float32),
            frame_id="body",
            ts=12.0,
        )
    )

    assert len(published) == 1
    terrain = published[0]
    assert terrain.frame_id == topic_default_frame_id(TOPICS.terrain_map)
    assert terrain.points.shape == (2, 4)
    np.testing.assert_allclose(terrain.points[:, 3], np.array([0.35, 0.60], dtype=np.float32))
    assert [field.name for field in terrain.fields] == ["x", "y", "z", "intensity"]


def test_nanobind_local_planner_applies_cmu_parity_config(monkeypatch):
    from base_autonomy.modules import local_planner_module as mod
    import core.config as config_mod

    cfg = SimpleNamespace(
        geometry=SimpleNamespace(
            vehicle_length=1.2,
            vehicle_width=0.7,
            sensor_offset_x=0.11,
            sensor_offset_y=-0.04,
        ),
        safety=SimpleNamespace(
            obstacle_height_thre=0.31,
            ground_height_thre=0.12,
        ),
        speed=SimpleNamespace(
            max_speed=1.6,
            autonomy_speed=0.9,
        ),
        raw={
            "terrain": {
                "min_rel_z": -1.1,
                "max_rel_z": 0.42,
            },
            "local_planner": {
                "two_way_drive": False,
                "adjacent_range": 4.25,
                "check_obstacle": False,
                "check_rot_obstacle": True,
                "use_terrain_analysis": False,
                "point_per_path_thre": 6,
                "dir_weight": 0.045,
                "dir_thre": 72.0,
                "path_scale": 0.95,
                "min_path_scale": 0.65,
                "path_scale_step": 0.15,
                "path_scale_by_speed": False,
                "min_path_range": 1.6,
                "path_range_step": 0.35,
                "path_range_by_speed": False,
                "path_crop_by_goal": False,
                "slope_weight": 2.75,
                "goal_clear_range": 0.85,
                "near_field_stop_dis": 0.95,
                "recovery_blocked_thre": 3.25,
                "recovery_rotate_time": 1.25,
                "recovery_backup_time": 0.75,
                "recovery_max_cycles": 5,
                "cost_height_thre_1": 0.22,
                "cost_height_thre_2": 0.14,
                "dir_to_vehicle": True,
                "goal_behind_range": 1.05,
                "freeze_ang": 105.0,
                "freeze_time": 3.5,
                "omni_dir_goal_thre": 1.8,
                "slow_path_num_thre": 8,
                "slow_group_num_thre": 3,
            },
        },
    )

    monkeypatch.setattr(mod, "try_import_nav_core", lambda *_args, **_kw: _FakeNavCore)
    monkeypatch.setattr(config_mod, "get_config", lambda: cfg)
    _FakeLocalPlannerCore.captured_params = None
    module = LocalPlannerModule(backend="nanobind")
    module._setup_nanobind()

    params = _FakeLocalPlannerCore.captured_params
    assert params is not None
    assert params.vehicle_length == 1.2
    assert params.two_way_drive is False
    assert params.adjacent_range == 4.25
    assert params.check_obstacle is False
    assert params.check_rot_obstacle is True
    assert params.use_terrain_analysis is False
    assert params.point_per_path_thre == 6
    assert params.min_rel_z == -1.1
    assert params.max_rel_z == 0.42
    assert params.dir_weight == 0.045
    assert params.dir_thre == 72.0
    assert params.path_scale == 0.95
    assert params.min_path_scale == 0.65
    assert params.path_scale_step == 0.15
    assert params.path_scale_by_speed is False
    assert params.min_path_range == 1.6
    assert params.path_range_step == 0.35
    assert params.path_range_by_speed is False
    assert params.path_crop_by_goal is False
    assert params.slope_weight == 2.75
    assert params.goal_clear_range == 0.85
    assert params.near_field_stop_dis == 0.95
    assert params.recovery_blocked_thre == 3.25
    assert params.recovery_rotate_time == 1.25
    assert params.recovery_backup_time == 0.75
    assert params.recovery_max_cycles == 5
    assert params.cost_height_thre_1 == 0.22
    assert params.cost_height_thre_2 == 0.14
    assert params.dir_to_vehicle is True
    assert params.goal_behind_range == 1.05
    assert params.freeze_ang == 105.0
    assert params.freeze_time == 3.5
    assert params.omni_dir_goal_thre == 1.8
    assert params.slow_path_num_thre == 8
    assert params.slow_group_num_thre == 3
    assert module.health()["local_planner"]["effective_params"]["adjacent_range"] == 4.25


def test_native_local_planner_uses_adjacent_range_not_path_range_step():
    from base_autonomy.native_factories import local_planner

    cfg = SimpleNamespace(
        geometry=SimpleNamespace(
            vehicle_length=1.0,
            vehicle_width=0.6,
            sensor_offset_x=0.0,
            sensor_offset_y=0.0,
        ),
        speed=SimpleNamespace(max_speed=1.0, autonomy_speed=0.8),
        safety=SimpleNamespace(obstacle_height_thre=0.2, ground_height_thre=0.1),
        raw={
            "nav_install": str(_REPO_ROOT / "install"),
            "terrain": {},
            "autonomy": {},
            "local_planner": {
                "adjacent_range": 4.0,
                "path_range_step": 0.25,
                "path_scale": 0.9,
                "min_path_scale": 0.6,
                "path_scale_step": 0.1,
                "slope_weight": 3.5,
                "omni_dir_goal_thre": 1.7,
                "min_rel_z": -0.9,
                "max_rel_z": 0.35,
            },
        },
    )

    module = local_planner(cfg)

    assert module._native_config.parameters["adjacentRange"] == 4.0
    assert module._native_config.parameters["pathRangeStep"] == 0.25
    assert module._native_config.parameters["pathScale"] == 0.9
    assert module._native_config.parameters["minPathScale"] == 0.6
    assert module._native_config.parameters["pathScaleStep"] == 0.1
    assert module._native_config.parameters["slopeWeight"] == 3.5
    assert module._native_config.parameters["omniDirGoalThre"] == 1.7
    assert module._native_config.parameters["minRelZ"] == -0.9
    assert module._native_config.parameters["maxRelZ"] == 0.35


def test_local_planner_params_binding_exposes_cmu_parity_fields():
    binding = _REPO_ROOT / "src" / "nav" / "core" / "bindings" / "py_nav_core.cpp"
    text = binding.read_text(encoding="utf-8")

    for field in (
        "cost_height_thre_1",
        "cost_height_thre_2",
        "dir_to_vehicle",
        "goal_behind_range",
        "freeze_ang",
        "freeze_time",
        "omni_dir_goal_thre",
        "slow_path_num_thre",
        "slow_group_num_thre",
    ):
        assert f'.def_rw("{field}"' in text


def test_robot_config_declares_cmu_local_planner_parity_defaults():
    cfg = load_config(path=str(_REPO_ROOT / "config" / "robot_config.yaml"))
    lp = cfg.raw["local_planner"]

    assert lp["adjacent_range"] == 3.5
    assert lp["path_scale"] == 1.0
    assert lp["min_path_scale"] == 0.75
    assert lp["path_scale_step"] == 0.25
    assert lp["near_field_stop_dis"] == 0.5
    assert lp["slope_weight"] == 0.0
    assert lp["recovery_blocked_thre"] == 2.0
    assert lp["recovery_rotate_time"] == 2.5
    assert lp["recovery_backup_time"] == 1.5
    assert lp["recovery_max_cycles"] == 3


def test_local_planner_default_path_frame_uses_runtime_topic_contract():
    module = LocalPlannerModule(backend="simple")

    assert module._path_frame_id == topic_default_frame_id(TOPICS.local_path)


def test_local_planner_corridor_goal_prefers_current_floor_at_same_xy():
    module = LocalPlannerModule(backend="simple", corridor_lookahead_m=1.0)
    module._robot_pos = np.asarray([0.0, 0.0, 0.0], dtype=float)
    module._global_path_points = np.asarray(
        [
            [0.0, 0.0, 3.0],
            [1.0, 0.0, 3.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
        ],
        dtype=float,
    )

    goal = module._select_corridor_goal(np.asarray([4.0, 0.0, 0.0], dtype=float))

    np.testing.assert_allclose(goal, np.asarray([1.0, 0.0, 0.0], dtype=float))


def _run_cmu_py_local_plan(obstacle_half_width: float) -> tuple[np.ndarray, np.ndarray]:
    module = LocalPlannerModule(backend="cmu_py")
    module.setup()
    published: list[Path] = []
    module.local_path._add_callback(published.append)
    module._on_odom(
        Odometry(
            pose=Pose(
                position=Vector3(0.0, 0.0, 0.0),
                orientation=Quaternion.from_yaw(0.0),
            ),
            frame_id="map",
            child_frame_id="base_link",
            ts=1.0,
        )
    )

    obstacle_points = []
    for x in np.linspace(1.0, 1.8, 5):
        samples = max(3, int(obstacle_half_width * 20) + 1)
        for y in np.linspace(-obstacle_half_width, obstacle_half_width, samples):
            obstacle_points.append([x, y, 0.0, 200.0])
    obstacles = np.asarray(obstacle_points, dtype=np.float32)
    module._on_added_obstacles(PointCloud2(points=obstacles, frame_id="map", ts=1.0))
    module._latest_waypoint = PoseStamped(
        pose=Pose(
            position=Vector3(4.0, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        frame_id="map",
        ts=1.0,
    )

    module._run_cmu_py()
    assert published, "local planner must publish a path decision"
    path = published[-1]
    path_xy = np.asarray(
        [[pose.pose.position.x, pose.pose.position.y] for pose in path.poses],
        dtype=float,
    )
    return path_xy, obstacles[:, :2].astype(float)


def test_cmu_py_local_planner_routes_around_feasible_added_obstacle():
    path_xy, obstacle_xy = _run_cmu_py_local_plan(obstacle_half_width=0.15)

    assert len(path_xy) > 2
    assert float(np.max(np.abs(path_xy[:, 1]))) > 0.25
    min_clearance = float(
        np.min(np.linalg.norm(path_xy[:, None, :] - obstacle_xy[None, :, :], axis=2))
    )
    assert min_clearance > 0.30


def test_cmu_py_local_planner_does_not_fallback_to_collision_line_when_blocked():
    path_xy, _obstacle_xy = _run_cmu_py_local_plan(obstacle_half_width=0.30)

    assert path_xy.shape == (0,)


def test_nanobind_local_planner_publishes_control_hint_for_near_field_stop():
    module = LocalPlannerModule(backend="nanobind")
    module._core = _FakeLocalPlannerCore()
    hints: list[dict] = []
    module.control_hint._add_callback(hints.append)
    module._latest_waypoint = PoseStamped(
        pose=Pose(position=Vector3(4.0, 0.0, 0.0)),
        frame_id="map",
        ts=1.0,
    )

    module._on_odom(
        Odometry(
            pose=Pose(
                position=Vector3(0.0, 0.0, 0.0),
                orientation=Quaternion.from_yaw(0.0),
            ),
            frame_id="map",
            child_frame_id="base_link",
            ts=1.0,
        )
    )

    assert hints
    assert hints[-1]["slow_down"] == 2
    assert hints[-1]["near_field_stop"] is True
    assert hints[-1]["safety_stop"] is True
    assert hints[-1]["reason"] == "nanobind"

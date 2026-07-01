import pytest

pytestmark = [pytest.mark.sim]

import builtins
import sys
import types
from pathlib import Path

from sim.validation import full_system as full_system_module
from runtime.msgs.numpy_compat import np
from sim.validation.full_system import (
    BLOCKED,
    FAIL,
    PASS,
    ValidationCheck,
    run_validation,
    _timed,
    validate_navigation_blueprint,
    validate_policy_smoke_runtime,
    validate_scene_catalog,
)


def test_scene_catalog_identifies_multifloor_building_contract():
    repo_root = Path(__file__).resolve().parents[2]
    checks = validate_scene_catalog(repo_root)
    by_name = {check.name: check for check in checks}

    assert by_name["required_worlds_exist"].status == PASS
    multifloor = by_name["building_scene_multifloor_contract"]
    assert multifloor.status == PASS
    assert multifloor.evidence["step_count"] >= 10
    assert multifloor.evidence["second_floor_geom_count"] >= 2
    assert multifloor.evidence["max_geom_z"] >= 3.5


def test_static_full_system_validation_covers_required_capabilities():
    report = run_validation(run_mujoco=False)
    categories = {check.category for check in report.checks}

    assert {
        "scene",
        "lidar",
        "slam_localization",
        "navigation",
        "exploration",
        "tracking",
    } <= categories
    assert not [check for check in report.checks if check.status == "fail"]
    nav = next(check for check in report.checks if check.name == "sim_nav_planning_wiring")
    assert report.passed is (nav.status == PASS)
    lidar = [check for check in report.checks if check.category == "lidar"]
    assert lidar and lidar[0].status == BLOCKED


def test_navigation_blueprint_validation_uses_static_profile_graph(monkeypatch):
    real_import = builtins.__import__

    def guarded_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "runtime.blueprints.full_stack":
            raise AssertionError(f"runtime blueprint import attempted: {name}")
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(full_system_module, "_NUMPY_IMPORT_SAFE", False)
    monkeypatch.setattr(builtins, "__import__", guarded_import)

    check = validate_navigation_blueprint(Path(__file__).resolve().parents[2])

    assert check.status == BLOCKED
    assert check.evidence["profile_graph_mode"] == "static"
    assert check.evidence["missing_connections"] == []
    assert check.evidence["runtime_parity"]["status"] == BLOCKED


def test_navigation_blueprint_validation_passes_runtime_parity_when_available(
    monkeypatch,
):
    from runtime.blueprints.profile_graph import WireEdge

    required_edges = (
        WireEdge("MujocoDriverModule", "odometry", "nav.mission", "odometry"),
        WireEdge("MujocoDriverModule", "map_cloud", "OccupancyGridModule", "map_cloud"),
        WireEdge("OccupancyGridModule", "costmap", "TraversabilityCostModule", "costmap"),
        WireEdge("TraversabilityCostModule", "fused_cost", "nav.mission", "costmap"),
        WireEdge("nav.mission", "waypoint", "nav.local_planner", "waypoint"),
        WireEdge("nav.local_planner", "local_path", "nav.path_follower", "local_path"),
        WireEdge("nav.path_follower", "cmd_vel", "nav.velocity_mux", "path_follower_cmd_vel"),
        WireEdge("nav.velocity_mux", "driver_cmd_vel", "MujocoDriverModule", "cmd_vel"),
    )
    graph = types.SimpleNamespace(
        modules=(
            "MujocoDriverModule",
            "nav.mission",
            "OccupancyGridModule",
            "TraversabilityCostModule",
            "nav.local_planner",
            "nav.path_follower",
            "nav.velocity_mux",
        ),
        explicit_wires=required_edges,
        as_snapshot=lambda: {
            "modules": sorted(
                (
                    "MujocoDriverModule",
                    "nav.mission",
                    "OccupancyGridModule",
                    "TraversabilityCostModule",
                    "nav.local_planner",
                    "nav.path_follower",
                    "nav.velocity_mux",
                )
            ),
            "explicit_wires": sorted(edge.as_snapshot() for edge in required_edges),
        },
    )

    def fake_graph_for_profile(_profile, **_kwargs):
        return graph

    monkeypatch.setattr(full_system_module, "_NUMPY_IMPORT_SAFE", True)
    monkeypatch.setattr(
        "runtime.blueprints.profile_graph.graph_for_profile",
        fake_graph_for_profile,
    )

    check = validate_navigation_blueprint(Path(__file__).resolve().parents[2])

    assert check.status == PASS
    assert check.evidence["runtime_parity"]["checked"] is True
    assert check.evidence["runtime_parity"]["status"] == PASS


def test_navigation_blueprint_runtime_internal_import_error_fails(monkeypatch):
    from runtime.blueprints.profile_graph import WireEdge

    required_edges = (
        WireEdge("MujocoDriverModule", "odometry", "nav.mission", "odometry"),
        WireEdge("MujocoDriverModule", "map_cloud", "OccupancyGridModule", "map_cloud"),
        WireEdge("OccupancyGridModule", "costmap", "TraversabilityCostModule", "costmap"),
        WireEdge("TraversabilityCostModule", "fused_cost", "nav.mission", "costmap"),
        WireEdge("nav.mission", "waypoint", "nav.local_planner", "waypoint"),
        WireEdge("nav.local_planner", "local_path", "nav.path_follower", "local_path"),
        WireEdge("nav.path_follower", "cmd_vel", "nav.velocity_mux", "path_follower_cmd_vel"),
        WireEdge("nav.velocity_mux", "driver_cmd_vel", "MujocoDriverModule", "cmd_vel"),
    )
    graph = types.SimpleNamespace(
        modules=(
            "MujocoDriverModule",
            "nav.mission",
            "OccupancyGridModule",
            "TraversabilityCostModule",
            "nav.local_planner",
            "nav.path_follower",
            "nav.velocity_mux",
        ),
        explicit_wires=required_edges,
        as_snapshot=lambda: {
            "modules": sorted(
                (
                    "MujocoDriverModule",
                    "nav.mission",
                    "OccupancyGridModule",
                    "TraversabilityCostModule",
                    "nav.local_planner",
                    "nav.path_follower",
                    "nav.velocity_mux",
                )
            ),
            "explicit_wires": sorted(edge.as_snapshot() for edge in required_edges),
        },
    )

    def fake_graph_for_profile(_profile, **kwargs):
        if kwargs.get("mode") == "runtime":
            raise ModuleNotFoundError(
                "No module named 'runtime.internal_typo'",
                name="runtime.internal_typo",
            )
        return graph

    monkeypatch.setattr(full_system_module, "_NUMPY_IMPORT_SAFE", True)
    monkeypatch.setattr(
        "runtime.blueprints.profile_graph.graph_for_profile",
        fake_graph_for_profile,
    )

    check = validate_navigation_blueprint(Path(__file__).resolve().parents[2])

    assert check.status == FAIL
    assert check.evidence["runtime_parity"]["status"] == FAIL
    assert check.evidence["runtime_parity"]["missing_module"] == "core"


def test_default_validation_requires_navigation_runtime_parity(monkeypatch):
    def passing_check(name: str, category: str) -> ValidationCheck:
        return ValidationCheck(name=name, category=category, status=PASS, summary="ok")

    monkeypatch.setattr(
        full_system_module,
        "validate_scene_catalog",
        lambda _root: [passing_check("required_worlds_exist", "scene")],
    )
    monkeypatch.setattr(
        full_system_module,
        "validate_slam_localization_contract",
        lambda _root: passing_check("slam_localization_metric_gate", "slam_localization"),
    )
    monkeypatch.setattr(
        full_system_module,
        "validate_navigation_blueprint",
        lambda _root: ValidationCheck(
            name="sim_nav_planning_wiring",
            category="navigation",
            status=BLOCKED,
            summary="runtime parity blocked",
            evidence={"runtime_parity": {"status": BLOCKED}},
        ),
    )
    monkeypatch.setattr(
        full_system_module,
        "validate_frontier_exploration_runtime",
        lambda: passing_check("frontier_exploration_to_navigation", "exploration"),
    )
    monkeypatch.setattr(
        full_system_module,
        "validate_person_tracking_runtime",
        lambda: passing_check("person_tracking_behavior_loop", "tracking"),
    )

    report = run_validation(run_mujoco=False)

    assert report.passed is False
    assert any(check.name == "mujoco_lidar_runtime" and check.status == BLOCKED for check in report.checks)


def test_static_full_system_validation_can_treat_blocked_checks_as_failure():
    report = run_validation(run_mujoco=False, require_all=True)

    assert report.passed is False
    assert any(check.status == BLOCKED for check in report.checks)


def _pretend_mujoco_is_installed(monkeypatch, *, extra: tuple[str, ...] = ()):
    real_find_spec = full_system_module.importlib.util.find_spec
    installed = {"mujoco", *extra}

    def fake_find_spec(name, *args, **kwargs):
        if name in installed:
            return object()
        return real_find_spec(name, *args, **kwargs)

    monkeypatch.setattr(full_system_module.importlib.util, "find_spec", fake_find_spec)


def test_mujoco_short_helper_names_keep_compat_aliases():
    assert full_system_module.validate_mujoco_lidar_runtime is full_system_module.check_lidar
    assert full_system_module.validate_mujoco_sensor_runtime is full_system_module.check_sensors
    assert (
        full_system_module.validate_mujoco_kinematic_nav_runtime
        is full_system_module.check_nav_motion
    )


def test_sensor_check_reports_discrete_height_ray_evidence(monkeypatch):
    _pretend_mujoco_is_installed(monkeypatch)

    class FakeEngine:
        def step(self):
            return types.SimpleNamespace(
                imu_gyro=[0.0, 0.0, 0.0],
                imu_projected_gravity=[0.0, 0.0, -1.0],
            )

        def get_lidar_points(self):
            return np.ones((3, 4), dtype=np.float32)

        def get_camera_data(self, _name):
            return types.SimpleNamespace(
                rgb=np.zeros((480, 640, 3), dtype=np.uint8),
                depth=np.ones((480, 640), dtype=np.float32),
                intrinsics=(415.0, 415.0, 320.0, 240.0),
            )

        def get_discrete_rays(self):
            return types.SimpleNamespace(
                heights=np.array([0.5, float("nan")], dtype=np.float32),
                points_body=np.zeros((2, 3), dtype=np.float32),
                points_world=np.ones((2, 3), dtype=np.float32),
                valid_mask=np.array([True, False], dtype=bool),
                pattern="grid",
                metadata={"sample_count": 2, "valid_count": 1},
            )

        def close(self):
            pass

    class FakeDriver:
        def __init__(self, **_kwargs):
            self._engine = FakeEngine()

        def setup(self):
            pass

    monkeypatch.setitem(
        sys.modules,
        "drivers.sim.mujoco.driver",
        types.SimpleNamespace(MujocoDriverModule=FakeDriver),
    )

    check = full_system_module.check_sensors()

    assert check.status == PASS
    assert check.evidence["height_rays"] == {
        "ok": True,
        "pattern": "grid",
        "sample_count": 2,
        "valid_count": 1,
        "heights_shape": [2],
        "points_body_shape": [2, 3],
        "points_world_shape": [2, 3],
        "height_min_m": pytest.approx(0.5),
        "height_max_m": pytest.approx(0.5),
        "metadata": {"sample_count": 2, "valid_count": 1},
    }


def test_mujoco_driver_publishes_height_ray_payload():
    from drivers.sim.mujoco.driver import (
        MUJOCO_MODULE_BODY_FRAME_ID,
        MujocoDriverModule,
    )

    driver = MujocoDriverModule(
        world="open_field",
        render=False,
        enable_camera=False,
        drive_mode="kinematic",
        lidar_publish_every=100000,
        height_ray_publish_every=1,
    )
    published = []
    driver.height_rays._add_callback(published.append)

    class FakeEngine:
        def step(self, _cmd):
            driver._running = False
            return types.SimpleNamespace(
                position=np.array([0.0, 0.0, 0.55], dtype=np.float64),
                orientation=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
                linear_velocity=np.zeros(3, dtype=np.float64),
                angular_velocity=np.zeros(3, dtype=np.float64),
            )

        def get_lidar_points(self):
            return np.zeros((0, 4), dtype=np.float32)

        def get_discrete_rays(self):
            return types.SimpleNamespace(
                pattern="grid",
                heights=np.array([0.55], dtype=np.float32),
                points_body=np.array([[0.0, 0.0, -0.55]], dtype=np.float32),
                points_world=np.array([[0.0, 0.0, 0.0]], dtype=np.float32),
                valid_mask=np.array([True], dtype=bool),
                metadata={"sample_count": 1, "valid_count": 1},
            )

        def get_camera_data(self, _name):
            return None

    driver._engine = FakeEngine()
    driver._running = True
    driver._sim_loop()

    assert len(published) == 1
    payload = published[0]
    assert payload["pattern"] == "grid"
    assert payload["frame_id"] == MUJOCO_MODULE_BODY_FRAME_ID
    np.testing.assert_allclose(payload["heights"], np.array([0.55], dtype=np.float32))
    assert payload["valid_mask"].tolist() == [True]
    assert payload["metadata"] == {"sample_count": 1, "valid_count": 1}


def test_nav_motion_gate_accepts_startup_stop_when_final_safe(monkeypatch):
    _pretend_mujoco_is_installed(monkeypatch)
    monkeypatch.setattr(full_system_module.time, "sleep", lambda _seconds: None)

    class Port:
        def __init__(self, on_deliver=None):
            self.callbacks = []
            self._on_deliver = on_deliver

        def _add_callback(self, callback):
            self.callbacks.append(callback)

        def _deliver(self, msg):
            if self._on_deliver is not None:
                self._on_deliver(msg)
            for callback in list(self.callbacks):
                callback(msg)

    def odom_msg(x):
        return types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=x, y=0.0, z=0.55)
            )
        )

    def twist_msg(vx):
        return types.SimpleNamespace(
            linear=types.SimpleNamespace(x=vx, y=0.0),
            angular=types.SimpleNamespace(z=0.0),
        )

    def cloud_msg(frame_id="odom"):
        return types.SimpleNamespace(
            points=np.asarray(
                [[0.0, 0.0, 0.0], [0.5, 0.0, 0.1], [0.5, 0.5, 0.2]],
                dtype=np.float32,
            ),
            frame_id=frame_id,
        )

    def grid_msg():
        return {
            "grid": np.asarray([[0, 0, -1], [0, 100, 0], [-1, 0, 0]], dtype=np.float32),
            "resolution": 0.2,
            "origin": np.asarray([-0.3, -0.3], dtype=np.float32),
        }

    driver = types.SimpleNamespace(
        odometry=Port(),
        lidar_cloud=Port(),
        map_cloud=Port(),
        health=lambda: {
            "ports_in": {"cmd_vel": {"msg_count": 5, "connected": True}},
            "mujoco": {"drive_mode": "kinematic"},
        },
    )
    ogm = types.SimpleNamespace(costmap=Port())
    terrain = types.SimpleNamespace(terrain_map=Port())
    traversability = types.SimpleNamespace(fused_cost=Port())
    local_planner = types.SimpleNamespace(local_path=Port())
    path_follower = types.SimpleNamespace(cmd_vel=Port())
    mux = types.SimpleNamespace(
        driver_cmd_vel=Port(),
        health=lambda: {"active_source": "path_follower", "source_timeout_s": 2.0},
    )
    safety = types.SimpleNamespace(
        stop_cmd=Port(),
        health=lambda: {"safety_ring": {"level": "SAFE", "assessment": "ON_TRACK"}},
    )

    def deliver_nav_chain(_goal_msg):
        nav.global_path._deliver(object())
        nav.waypoint._deliver(object())
        local_planner.local_path._deliver(object())
        for _ in range(5):
            path_follower.cmd_vel._deliver(twist_msg(0.15))
            mux.driver_cmd_vel._deliver(twist_msg(0.15))
        driver.odometry._deliver(odom_msg(0.20))
        safety.stop_cmd._deliver(0)

    nav = types.SimpleNamespace(
        goal_pose=Port(on_deliver=deliver_nav_chain),
        global_path=Port(),
        waypoint=Port(),
        adapter_status=Port(),
    )

    class FakeSystem:
        modules = {
            "MujocoDriverModule": driver,
            "OccupancyGridModule": ogm,
            "nav.terrain": terrain,
            "TraversabilityCostModule": traversability,
            "nav.mission": nav,
            "nav.local_planner": local_planner,
            "nav.path_follower": path_follower,
            "nav.velocity_mux": mux,
            "nav.safety": safety,
        }

        def get_module(self, name):
            return self.modules[name]

        def start(self):
            driver.lidar_cloud._deliver(cloud_msg("body"))
            driver.map_cloud._deliver(cloud_msg("odom"))
            terrain.terrain_map._deliver(cloud_msg("body"))
            ogm.costmap._deliver(grid_msg())
            traversability.fused_cost._deliver(grid_msg())
            driver.odometry._deliver(odom_msg(0.0))
            safety.stop_cmd._deliver(2)
            safety.stop_cmd._deliver(0)

        def stop(self):
            pass

    import runtime.blueprints.profile_builder as profile_builder

    monkeypatch.setattr(
        profile_builder,
        "build_system_for_profile",
        lambda *_args, **_kwargs: FakeSystem(),
    )

    check = full_system_module.check_nav_motion(
        duration_s=0.5,
        goal_distance_m=1.0,
        min_motion_m=0.15,
    )

    assert check.status == PASS
    assert check.evidence["max_stop_level"] == 2
    assert check.evidence["final_stop_level"] == 0
    assert check.evidence["safety"]["level"] == "SAFE"
    assert check.evidence["nav_only_motion_gate"] is True
    assert check.evidence["pointcloud_to_nav_flow_ok"] is True
    flow = check.evidence["pointcloud_to_nav_flow"]
    assert flow["mujoco_lidar_cloud_body"]["has_xyz"] is True
    assert flow["mujoco_map_cloud_odom"]["max_point_count"] > 0
    assert flow["terrain_map_to_local_planner"]["max_point_count"] > 0
    assert flow["occupancy_costmap_from_pointcloud"]["non_unknown_cells"] > 0
    assert flow["fused_cost_to_navigation"]["non_unknown_cells"] > 0


def test_policy_smoke_reports_missing_policy_as_blocked(monkeypatch):
    _pretend_mujoco_is_installed(monkeypatch, extra=("onnxruntime",))
    from sim.scripts import policy_nav_smoke

    direct = {"policy_loaded": False, "policy_path": "", "moved_m": 0.0}
    nav = {"policy_loaded": False, "policy_path": "", "moved_m": 0.0}
    monkeypatch.setattr(policy_nav_smoke, "run_direct_policy", lambda **_: direct)
    monkeypatch.setattr(policy_nav_smoke, "run_full_stack_nav", lambda **_: nav)
    monkeypatch.setattr(policy_nav_smoke, "_passes_direct", lambda *_args, **_kw: False)
    monkeypatch.setattr(policy_nav_smoke, "_passes_nav", lambda *_args, **_kw: False)
    monkeypatch.setattr(policy_nav_smoke, "_load_policy_metadata", lambda _: {"exists": False})

    check = validate_policy_smoke_runtime(
        world="open_field",
        direct_duration_s=0.1,
        nav_duration_s=0.1,
        goal_distance_m=0.5,
    )

    assert check.status == BLOCKED
    assert "checkpoint" in check.summary


def test_policy_smoke_can_pass_with_explicit_policy(monkeypatch):
    _pretend_mujoco_is_installed(monkeypatch, extra=("onnxruntime",))
    from sim.scripts import policy_nav_smoke

    direct = {"policy_loaded": True, "policy_path": "policy.onnx", "moved_m": 0.3}
    nav = {"policy_loaded": True, "policy_path": "policy.onnx", "moved_m": 0.3}
    nav_kwargs = {}
    passes_nav_kwargs = {}

    def fake_run_full_stack_nav(**kwargs):
        nav_kwargs.update(kwargs)
        return nav

    def fake_passes_nav(_result, **kwargs):
        passes_nav_kwargs.update(kwargs)
        return True

    monkeypatch.setattr(policy_nav_smoke, "run_direct_policy", lambda **_: direct)
    monkeypatch.setattr(policy_nav_smoke, "run_full_stack_nav", fake_run_full_stack_nav)
    monkeypatch.setattr(policy_nav_smoke, "_passes_direct", lambda *_args, **_kw: True)
    monkeypatch.setattr(policy_nav_smoke, "_passes_nav", fake_passes_nav)
    monkeypatch.setattr(policy_nav_smoke, "_load_policy_metadata", lambda _: {"exists": True})

    check = validate_policy_smoke_runtime(
        world="open_field",
        direct_duration_s=0.1,
        nav_duration_s=0.1,
        goal_distance_m=0.5,
        policy_path="policy.onnx",
    )

    assert check.status == PASS
    assert check.evidence["policy_path_arg"] == "policy.onnx"
    assert nav_kwargs["planner_backend"] == "octoplanner3d"
    assert nav_kwargs["local_planner_backend"] == "nanobind"
    assert nav_kwargs["path_follower_backend"] == "nav_kernel"
    assert passes_nav_kwargs == {"min_motion": 0.20, "max_dist_to_goal": 0.10}


def test_timed_marks_missing_environment_dependency_as_blocked():
    def missing_yaml():
        raise ModuleNotFoundError("No module named 'yaml'", name="yaml")

    check = _timed("yaml_required_check", "environment", missing_yaml)

    assert check.status == BLOCKED
    assert check.summary == "missing Python dependency: yaml"
    assert check.evidence["missing_module"] == "yaml"


def test_timed_keeps_internal_missing_module_as_failure():
    def missing_internal_module():
        raise ModuleNotFoundError(
            "No module named 'runtime.internal_typo'",
            name="runtime.internal_typo",
        )

    check = _timed("internal_import_check", "architecture", missing_internal_module)

    assert check.status == FAIL
    assert check.evidence["missing_module"] == "core"

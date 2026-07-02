"""Contract tests for all LocalPlanner backends.

Verifies that every declared backend (nanobind, cmu_py, simple) can be
instantiated, that port types match the spec, that the simple backend survives
the full lifecycle and produces paths, and that the nanobind backend also
works when lingtu_nav_kernel.so is available.  Follows the pattern established in
test_autonomy_modules.py.
"""

from __future__ import annotations

import ast
from pathlib import Path as FsPath

import pytest

from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import Odometry, Path
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import PointCloud2
from runtime.registry import get as registry_get

# =============================================================================
# Instantiation
# =============================================================================


class TestLocalPlannerBackends:
    """Backend-specific contract checks for the local planner service."""

    @pytest.mark.parametrize("backend", ["nanobind", "simple", "cmu_py"])
    def test_all_backends_instantiate(self, backend: str) -> None:
        """Every registered backend name must succeed in __init__."""
        from nav.services.plan.local_planner.service import (
            _AVAILABLE_LOCAL_PLANNER_BACKENDS,
            LocalPlanner,
        )

        assert backend in _AVAILABLE_LOCAL_PLANNER_BACKENDS, (
            f"{backend} not listed in _AVAILABLE_LOCAL_PLANNER_BACKENDS"
        )
        mod = LocalPlanner(backend=backend)
        assert mod._backend == backend
        assert mod._backend_status.configured == backend

    def test_default_backend_is_nanobind(self) -> None:
        """Default backend string must be 'nanobind'."""
        from nav.services.plan.local_planner.service import LocalPlanner

        mod = LocalPlanner()
        assert mod._backend == "nanobind"

    # ------------------------------------------------------------------ #
    # Port contract
    # ------------------------------------------------------------------ #

    def test_port_types_match_spec(self) -> None:
        """All In/Out ports have the correct msg_type."""
        from nav.services.plan.local_planner.service import LocalPlanner

        mod = LocalPlanner(backend="simple")

        # -- Input ports --
        expected_in = {
            "odometry": Odometry,
            "terrain_map": PointCloud2,
            "terrain_map_ext": PointCloud2,
            "traversability": dict,
            "waypoint": PoseStamped,
            "global_path": Path,
            "clear_path": bool,
            "map_frame_jump_event": dict,
            "boundary": PointCloud2,
            "added_obstacles": PointCloud2,
            "check_obstacle": bool,
            "esdf": dict,
        }
        assert len(mod._ports_in) == len(expected_in), (
            f"expected {len(expected_in)} In ports, got {list(mod._ports_in)}"
        )
        for name, expected_type in expected_in.items():
            assert name in mod._ports_in, f"missing In port: {name}"
            assert mod._ports_in[name].msg_type is expected_type, (
                f"In.{name}: expected {expected_type.__name__}, "
                f"got {mod._ports_in[name].msg_type.__name__}"
            )

        # -- Output ports --
        expected_out = {
            "local_path": Path,
            "control_hint": dict,
            "alive": bool,
        }
        assert len(mod._ports_out) == len(expected_out), (
            f"expected {len(expected_out)} Out ports, got {list(mod._ports_out)}"
        )
        for name, expected_type in expected_out.items():
            assert name in mod._ports_out, f"missing Out port: {name}"
            assert mod._ports_out[name].msg_type is expected_type, (
                f"Out.{name}: expected {expected_type.__name__}, "
                f"got {mod._ports_out[name].msg_type.__name__}"
            )

    # ------------------------------------------------------------------ #
    # Lifecycle (simple backend)
    # ------------------------------------------------------------------ #

    def test_simple_backend_lifecycle(self) -> None:
        """setup() -> start() -> stop() works with backend='simple'."""
        from nav.services.plan.local_planner.service import LocalPlanner

        mod = LocalPlanner(backend="simple")
        assert not mod._running

        mod.setup()
        mod.start()
        assert mod._running

        mod.stop()
        assert not mod._running

        # stop is idempotent
        mod.stop()
        assert not mod._running

    def test_simple_backend_alive_toggles(self) -> None:
        """alive Out[bool] publishes True on start(), False on stop()."""
        from nav.services.plan.local_planner.service import LocalPlanner

        mod = LocalPlanner(backend="simple")
        mod.setup()

        alive_values: list[bool] = []
        mod.alive._add_callback(alive_values.append)

        mod.start()
        assert len(alive_values) >= 1
        assert alive_values[-1] is True

        mod.stop()
        assert len(alive_values) >= 2
        assert alive_values[-1] is False

    # ------------------------------------------------------------------ #
    # Data flow (simple backend)
    # ------------------------------------------------------------------ #

    def test_simple_backend_publishes_path_on_waypoint(self) -> None:
        """Inject odometry + waypoint -> local_path published."""
        from nav.services.plan.local_planner.service import LocalPlanner

        mod = LocalPlanner(backend="simple")
        mod.setup()
        mod.start()

        path_values: list[Path] = []
        mod.local_path._add_callback(path_values.append)

        hint_values: list[dict] = []
        mod.control_hint._add_callback(hint_values.append)

        # Pre-load robot position via odometry
        odom = Odometry(
            pose=Pose(position=Vector3(0.0, 0.0, 0.0)),
            frame_id="map",
        )
        mod._on_odom(odom)
        mod._on_traversability({
            "traversability_class": "caution",
            "risk_max": 42.0,
            "risk_mean": 12.5,
        })

        # Inject a waypoint far enough for a multi-point straight line
        wp = PoseStamped(
            pose=Pose(
                position=Vector3(5.0, 0.0, 0.0),
                orientation=Quaternion(0, 0, 0, 1),
            ),
            frame_id="map",
        )
        mod._on_waypoint(wp)

        assert len(path_values) >= 1, "expected local_path to be published"
        published_path = path_values[-1]
        assert isinstance(published_path, Path)
        assert len(published_path.poses) >= 2, (
            f"expected >=2 path points, got {len(published_path.poses)}"
        )

        assert len(hint_values) >= 1
        assert hint_values[-1].get("reason") == "simple_path"
        assert hint_values[-1]["traversability"]["traversability_class"] == "caution"

        mod.stop()

    def test_simple_backend_rejects_frame_mismatch(self) -> None:
        """LocalPlanner must not mix odom-frame pose with map-frame goals."""
        from nav.services.plan.local_planner.service import LocalPlanner

        mod = LocalPlanner(backend="simple")
        mod.setup()
        mod.start()

        path_values: list[Path] = []
        hint_values: list[dict] = []
        mod.local_path._add_callback(path_values.append)
        mod.control_hint._add_callback(hint_values.append)

        mod._on_odom(
            Odometry(
                pose=Pose(position=Vector3(0.0, 0.0, 0.0)),
                frame_id="odom",
            )
        )
        mod._on_waypoint(
            PoseStamped(
                pose=Pose(position=Vector3(5.0, 0.0, 0.0)),
                frame_id="map",
            )
        )

        assert path_values[-1].poses == []
        assert hint_values[-1]["reason"] == "frame_mismatch"
        assert hint_values[-1]["safety_stop"] is True
        assert hint_values[-1]["frame_status"]["mismatches"] == {"odometry": "odom"}
        assert mod.health()["local_planner"]["frame_status"]["ok"] is False

        mod.stop()

    def test_simple_backend_accepts_configured_odom_planning_frame(self) -> None:
        """Profiles that plan in odom can opt in explicitly."""
        from nav.services.plan.local_planner.service import LocalPlanner

        mod = LocalPlanner(backend="simple", planning_frame_id="odom")
        mod.setup()
        mod.start()

        path_values: list[Path] = []
        mod.local_path._add_callback(path_values.append)

        mod._on_odom(
            Odometry(
                pose=Pose(position=Vector3(0.0, 0.0, 0.0)),
                frame_id="odom",
            )
        )
        mod._on_waypoint(
            PoseStamped(
                pose=Pose(position=Vector3(5.0, 0.0, 0.0)),
                frame_id="odom",
            )
        )

        assert len(path_values[-1].poses) >= 2
        assert path_values[-1].frame_id == "odom"
        assert mod.health()["local_planner"]["frame_status"]["ok"] is True

        mod.stop()

    def test_traversability_grid_becomes_virtual_obstacles(self) -> None:
        """High-risk traversability cells enter the shared obstacle cloud."""
        from nav.services.plan.local_planner.obstacles import (
            OBSTACLE_INTENSITY,
            merge_obstacle_clouds,
        )

        grid = np.asarray(
            [
                [0.0, 0.0, 0.0],
                [0.0, 95.0, 20.0],
                [0.0, 0.0, 0.0],
            ],
            dtype=np.float32,
        )

        merged = merge_obstacle_clouds(
            terrain_points=None,
            terrain_ext_points=None,
            traversability={
                "grid": grid,
                "resolution": 0.5,
                "origin": [0.0, 0.0],
            },
            robot_position=np.asarray([0.0, 0.0, 0.0], dtype=np.float32),
            traversability_range_m=3.5,
            boundary_points=None,
            added_obstacle_points=None,
            check_obstacle_enabled=True,
        )

        assert merged.shape == (1, 4)
        assert merged.dtype == np.float32
        assert np.allclose(merged[0, :2], [0.75, 0.75])
        assert merged[0, 3] == OBSTACLE_INTENSITY

    def test_cmu_py_traversability_grid_penalizes_group_scores(self) -> None:
        """Soft risk lowers candidate scores and hard risk blocks them."""
        from nav.services.plan.local_planner.cmu_py import (
            apply_traversability_cost_to_group_scores,
        )
        from nav.services.plan.local_planner.models import GROUP_NUM

        scores = np.ones(36 * GROUP_NUM, dtype=np.float64) * 10.0
        start_paths = [np.asarray([[0.75, 0.0, 0.0]], dtype=np.float32) for _ in range(GROUP_NUM)]
        origin = np.asarray([0.0, -0.75], dtype=np.float32)

        soft_grid = np.zeros((3, 3), dtype=np.float32)
        soft_grid[1, 1] = 70.0
        softened = apply_traversability_cost_to_group_scores(
            scores,
            start_paths,
            cos_yaw=1.0,
            sin_yaw=0.0,
            planner_pos=np.asarray([0.0, 0.0, 0.0], dtype=np.float32),
            rel_goal_dis=3.5,
            path_range=3.5,
            grid=soft_grid,
            resolution=0.5,
            origin=origin,
            hard_cost=90.0,
            soft_cost=40.0,
            weight=0.01,
        )
        assert softened[18 * GROUP_NUM + 3] == pytest.approx(7.0)

        hard_grid = soft_grid.copy()
        hard_grid[1, 1] = 95.0
        blocked = apply_traversability_cost_to_group_scores(
            scores,
            start_paths,
            cos_yaw=1.0,
            sin_yaw=0.0,
            planner_pos=np.asarray([0.0, 0.0, 0.0], dtype=np.float32),
            rel_goal_dis=3.5,
            path_range=3.5,
            grid=hard_grid,
            resolution=0.5,
            origin=origin,
            hard_cost=90.0,
            soft_cost=40.0,
            weight=0.01,
        )
        assert blocked[18 * GROUP_NUM + 3] == 0.0

    def test_nanobind_core_receives_traversability_grid(self) -> None:
        """Service syncs traversability payload into a capable C++ core."""
        from nav.services.plan.local_planner.service import LocalPlanner

        class FakeCore:
            captured = None

            def set_traversability_grid(self, grid, resolution, origin_x, origin_y):
                self.captured = (grid, resolution, origin_x, origin_y)

            def clear_traversability_grid(self):
                self.captured = "cleared"

        core = FakeCore()
        mod = LocalPlanner(backend="nanobind")
        mod._core = core
        mod._on_traversability(
            {
                "grid": np.asarray([[0.0, 95.0]], dtype=np.float32),
                "resolution": 0.5,
                "origin": [1.0, 2.0],
            }
        )

        mod._sync_core_traversability_grid()

        assert core.captured is not None
        grid, resolution, origin_x, origin_y = core.captured
        assert grid.shape == (1, 2)
        assert grid.dtype == np.float32
        assert grid.flags["C_CONTIGUOUS"]
        assert resolution == 0.5
        assert origin_x == 1.0
        assert origin_y == 2.0
        assert grid.tolist() == [[0.0, 95.0]]
        assert mod._last_traversability_grid_status["native"] is True
        assert mod._last_traversability_grid_status["active"] is True
        assert mod._last_traversability_grid_status["rows"] == 1
        assert mod._last_traversability_grid_status["cols"] == 2

    def test_nanobind_uses_native_traversability_not_virtual_obstacles(self) -> None:
        """Grid-capable C++ core receives traversability separately from obstacles."""
        from nav.services.plan.local_planner.service import LocalPlanner

        class FakeCore:
            traversability_grid = None
            obstacle_arg = None

            def set_vehicle(self, *_args):
                pass

            def set_goal(self, *_args):
                pass

            def set_traversability_grid(self, grid, resolution, origin_x, origin_y):
                self.traversability_grid = (grid, resolution, origin_x, origin_y)

            def clear_traversability_grid(self):
                self.traversability_grid = None

            def plan(self, obstacle_xyzi, _timestamp):
                self.obstacle_arg = obstacle_xyzi

                class Result:
                    path = []
                    path_found = False
                    near_field_stop = False
                    recovery_state = 0
                    slow_down = 0

                return Result()

        fake_core = FakeCore()
        mod = LocalPlanner(backend="nanobind")
        mod._core = fake_core
        mod._latest_waypoint = PoseStamped(
            pose=Pose(
                position=Vector3(2.0, 0.0, 0.0),
                orientation=Quaternion(0, 0, 0, 1),
            ),
            frame_id="map",
            ts=1.0,
        )
        mod._on_traversability(
            {
                "grid": np.asarray([[95.0]], dtype=np.float32),
                "resolution": 0.5,
                "origin": [0.0, 0.0],
            }
        )

        mod._run_nanobind(1.0)

        assert fake_core.traversability_grid is not None
        grid, resolution, origin_x, origin_y = fake_core.traversability_grid
        assert grid.shape == (1, 1)
        assert grid[0, 0] == pytest.approx(95.0)
        assert resolution == 0.5
        assert origin_x == 0.0
        assert origin_y == 0.0

        obstacle_arg = fake_core.obstacle_arg
        assert isinstance(obstacle_arg, np.ndarray)
        assert obstacle_arg.dtype == np.float32
        assert obstacle_arg.flags["C_CONTIGUOUS"]
        assert obstacle_arg.shape == (0,)
        assert mod._last_result_diagnostics["traversability_grid"]["native"] is True
        assert mod.health()["local_planner"]["traversability_grid"]["active"] is True

    def test_nanobind_prefers_single_native_plan_frame_call(self) -> None:
        """Newer C++ core receives one frame-level call instead of setter chatter."""
        from nav.services.plan.local_planner.service import LocalPlanner

        class FakeCore:
            plan_frame_calls = []
            setter_calls = []

            def set_vehicle(self, *_args):
                self.setter_calls.append("set_vehicle")

            def set_goal(self, *_args):
                self.setter_calls.append("set_goal")

            def set_traversability_grid(self, *_args):
                self.setter_calls.append("set_traversability_grid")

            def plan_frame(
                self,
                x,
                y,
                z,
                yaw,
                gx,
                gy,
                grid,
                resolution,
                origin_x,
                origin_y,
                obstacle_xyzi,
                timestamp,
            ):
                self.plan_frame_calls.append(
                    (x, y, z, yaw, gx, gy, grid, resolution, origin_x, origin_y, obstacle_xyzi, timestamp)
                )

                class Result:
                    path = []
                    path_found = False
                    near_field_stop = False
                    recovery_state = 0
                    slow_down = 0

                return Result()

            def plan(self, *_args):
                raise AssertionError("plan() fallback should not be called")

        fake_core = FakeCore()
        mod = LocalPlanner(backend="nanobind")
        mod._core = fake_core
        mod._record_input_frame("odometry", "map")
        mod._robot_pos = [1.0, 2.0, 0.3]
        mod._robot_yaw = 0.25
        mod._latest_waypoint = PoseStamped(
            pose=Pose(
                position=Vector3(4.0, 5.0, 0.0),
                orientation=Quaternion(0, 0, 0, 1),
            ),
            frame_id="map",
            ts=1.0,
        )
        mod._on_traversability(
            {
                "grid": np.asarray([[10.0, 20.0]], dtype=np.float32),
                "resolution": 0.5,
                "origin": [0.0, 0.0],
                "frame_id": "map",
            }
        )

        mod._run_nanobind(1.0)

        assert fake_core.setter_calls == []
        assert len(fake_core.plan_frame_calls) == 1
        call = fake_core.plan_frame_calls[0]
        assert call[0:6] == pytest.approx((1.0, 2.0, 0.3, 0.25, 4.0, 5.0))
        assert call[6].shape == (1, 2)
        assert call[7:10] == pytest.approx((0.5, 0.0, 0.0))
        assert isinstance(call[10], np.ndarray)
        assert call[10].dtype == np.float32

    # ------------------------------------------------------------------ #
    # Nanobind backend 闂?works when lingtu_nav_kernel.so is available
    # ------------------------------------------------------------------ #

    @pytest.mark.native
    def test_nanobind_backend_setup_succeeds(self, require_nav_kernel) -> None:
        """nanobind setup succeeds with local lingtu_nav_kernel build."""
        from nav.services.plan.local_planner.service import LocalPlanner

        require_nav_kernel(
            ("LocalPlannerParams", "LocalPlanner"),
            "nav.local_planner",
        )

        mod = LocalPlanner(backend="nanobind")
        mod.setup()
        assert mod._core is not None, (
            "nanobind C++ core should be loaded when lingtu_nav_kernel.so exists"
        )
        assert mod._backend == "nanobind"
        assert not mod._backend_status.degraded

        mod.start()
        assert mod._running
        mod.stop()
        assert not mod._running

    def test_cmu_py_backend_setup_succeeds(self) -> None:
        """cmu_py backend loads paths successfully when PLY files exist."""
        from nav.services.plan.local_planner.service import LocalPlanner

        mod = LocalPlanner(backend="cmu_py")
        mod.setup()
        assert mod._path_data is not None, (
            "cmu_py should load path data from disk"
        )
        assert mod._backend == "cmu_py"

        mod.start()
        assert mod._running
        mod.stop()
        assert not mod._running

    def test_cmu_py_backend_setup_uses_adapter_runtime_bundle(self, monkeypatch) -> None:
        """cmu_py setup should consume the backend adapter bundle only."""
        from nav.services.plan.local_planner import service as module_under_test
        from nav.services.plan.local_planner import runtime as runtime_under_test
        from nav.services.plan.local_planner.backend import (
            CmuPyLocalPlannerBackend,
        )

        sentinel_nav_kernel = object()
        sentinel_path_data = {"sentinel": object()}
        calls = []

        def fake_create_cmu_py_backend() -> CmuPyLocalPlannerBackend:
            calls.append("create")
            return CmuPyLocalPlannerBackend(
                nav_kernel=sentinel_nav_kernel,
                path_data=sentinel_path_data,
                paths_dir="sentinel-paths",
            )

        monkeypatch.setattr(
            runtime_under_test,
            "create_cmu_py_backend",
            fake_create_cmu_py_backend,
        )

        mod = module_under_test.LocalPlanner(backend="cmu_py")
        mod.setup()

        assert calls == ["create"]
        assert mod._native_kernel is sentinel_nav_kernel
        assert mod._path_data is sentinel_path_data

    def test_nanobind_setup_fails_without_nav_kernel(
        self,
        monkeypatch,
    ) -> None:
        """Default local planner must fail fast when LingTu native navigation kernel is unavailable."""
        from nav.services.plan.local_planner import service as module_under_test
        from nav.services.plan.local_planner import runtime as runtime_under_test
        from nav.services.plan.local_planner.backend import (
            NanobindLocalPlannerBackend,
        )

        def fake_create_nanobind_backend() -> NanobindLocalPlannerBackend:
            return NanobindLocalPlannerBackend(
                core=None,
                unavailable_reason="compatible LingTu native navigation kernel missing",
                build_hint="build nav core",
            )

        monkeypatch.setattr(
            runtime_under_test,
            "create_nanobind_backend",
            fake_create_nanobind_backend,
        )
        monkeypatch.setattr(
            runtime_under_test,
            "create_cmu_py_backend",
            lambda *_args, **_kwargs: pytest.fail(
                "cmu_py must not be auto-created for nanobind"
            ),
        )

        mod = module_under_test.LocalPlanner(backend="nanobind")
        with pytest.raises(RuntimeError, match="compatible LingTu native navigation kernel missing"):
            mod.setup()

        assert mod._backend == "nanobind"
        assert mod._backend_status.configured == "nanobind"
        assert mod._backend_status.effective == "nanobind"
        assert mod._backend_status.degraded is False
        assert mod._backend_status.degraded_reason == ""
        assert mod._path_data is None
        assert mod._core is None

    def test_nanobind_plan_receives_contiguous_float32_numpy_buffer(self) -> None:
        """Local planner hot path should avoid Python list materialisation."""
        from nav.services.plan.local_planner.service import LocalPlanner

        class FakeCore:
            obstacle_arg = None

            def set_vehicle(self, *_args):
                pass

            def set_goal(self, *_args):
                pass

            def plan(self, obstacle_xyzi, _timestamp):
                self.obstacle_arg = obstacle_xyzi

                class Result:
                    path = []
                    path_found = False
                    near_field_stop = False
                    recovery_state = 0
                    slow_down = 0

                return Result()

        fake_core = FakeCore()
        mod = LocalPlanner(backend="nanobind")
        mod._core = fake_core
        mod._latest_waypoint = PoseStamped(
            pose=Pose(
                position=Vector3(2.0, 0.0, 0.0),
                orientation=Quaternion(0, 0, 0, 1),
            ),
            frame_id="map",
            ts=1.0,
        )
        mod._on_added_obstacles(
            PointCloud2(
                points=np.asarray(
                    [[0.5, 0.0, 0.0], [0.8, 0.1, 0.0]],
                    dtype=np.float64,
                ),
                frame_id="map",
                ts=1.0,
            )
        )

        mod._run_nanobind(1.0)

        obstacle_arg = fake_core.obstacle_arg
        assert isinstance(obstacle_arg, np.ndarray)
        assert obstacle_arg.dtype == np.float32
        assert obstacle_arg.flags["C_CONTIGUOUS"]
        assert obstacle_arg.shape == (8,)
        assert not isinstance(obstacle_arg, list)

    # ------------------------------------------------------------------ #
    # Error handling
    # ------------------------------------------------------------------ #

    def test_unknown_backend_raises(self) -> None:
        """A bogus backend name must raise ValueError."""
        from nav.services.plan.local_planner.service import LocalPlanner

        with pytest.raises(
            ValueError, match="Unknown local_planner backend 'bogus'"
        ):
            LocalPlanner(backend="bogus")

    def test_runtime_helper_unknown_backend_raises(self) -> None:
        """Runtime setup must not treat unknown names as simple."""
        from nav.services.plan.local_planner.runtime import (
            setup_local_planner_backend,
        )
        from runtime.backend_status import BackendStatus

        with pytest.raises(ValueError, match="Unknown local_planner backend"):
            setup_local_planner_backend(
                "bogus",
                status=BackendStatus.configured_as("bogus"),
            )

    def test_legacy_cmu_backend_is_rejected(self) -> None:
        """The ROS2 NativeModule local planner is no longer a Module backend."""
        from nav.services.plan.local_planner.service import LocalPlanner

        with pytest.raises(ValueError, match="Unknown local_planner backend 'cmu'"):
            LocalPlanner(backend="cmu")

    # ------------------------------------------------------------------ #
    # Registry
    # ------------------------------------------------------------------ #

    def test_all_backends_registered(self) -> None:
        """All local_planner backends appear in the global Registry."""
        for backend in ("nanobind", "simple", "cmu_py"):
            cls = registry_get("local_planner", backend)
            assert cls is not None, (
                f"local_planner/{backend} not registered"
            )

    def test_stop_clears_core(self) -> None:
        """stop() must clear in-process C++ core references."""
        from nav.services.plan.local_planner.service import LocalPlanner

        mod = LocalPlanner(backend="simple")
        mod.setup()
        mod.start()

        # Simulate that _core was populated
        mod._core = object()

        mod.stop()
        assert mod._core is None
        assert not mod._running

    def test_module_has_no_external_node_lifecycle(self) -> None:
        """LocalPlanner must stay inside process, not manage node start/stop."""
        from nav.services.plan.local_planner.runtime import LocalPlannerRuntime

        module_path = (
            FsPath(__file__).resolve().parents[2]
            / "services"
            / "plan"
            / "local_planner"
            / "service.py"
        )
        source = module_path.read_text(encoding="utf-8")

        assert "node" not in LocalPlannerRuntime.__dataclass_fields__
        assert "self._node" not in source
        assert "runtime.node" not in source
        assert "external node" not in source

    def test_module_delegates_grid_config_loading_to_backend_adapter(self) -> None:
        """LocalPlanner should not read global config directly."""
        module_path = (
            FsPath(__file__).resolve().parents[2]
            / "services"
            / "plan"
            / "local_planner"
            / "service.py"
        )
        tree = ast.parse(module_path.read_text(encoding="utf-8"))
        direct_get_config_imports = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom)
            and node.module == "runtime.config"
            and any(alias.name == "get_config" for alias in node.names)
        ]

        assert direct_get_config_imports == []

    def test_module_delegates_backend_setup_to_runtime_helper(self) -> None:
        """LocalPlanner should not instantiate backend adapters directly."""
        module_path = (
            FsPath(__file__).resolve().parents[2]
            / "services"
            / "plan"
            / "local_planner"
            / "service.py"
        )
        source = module_path.read_text(encoding="utf-8")

        assert "setup_local_planner_backend" in source
        assert "create_nanobind_backend(" not in source
        assert "create_cmu_native_module(" not in source
        assert "create_cmu_py_backend(" not in source

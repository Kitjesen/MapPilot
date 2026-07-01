"""Contract tests for all PathFollower backends.

Verifies that every declared backend (nav_kernel, pid) can be
instantiated, that port types match the spec, that the pid backend survives
the full lifecycle and produces cmd_vel, and that the nav_kernel backend also
works when lingtu_nav_kernel.so is available.  Follows the pattern established in
test_autonomy_modules.py.
"""

from __future__ import annotations

import ast
from pathlib import Path as FsPath

import numpy as np
import pytest

from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from runtime.msgs.nav import Odometry, Path
from runtime.registry import get as registry_get

# =============================================================================
# Instantiation
# =============================================================================


class TestPathFollowerBackends:
    """Backend-specific contract checks for nav.path_follower."""

    @pytest.mark.parametrize("backend", ["nav_kernel", "pid"])
    def test_all_backends_instantiate(self, backend: str) -> None:
        """Every registered backend name must succeed in __init__."""
        from nav.local.path_follower import (
            _AVAILABLE_PATH_FOLLOWER_BACKENDS,
            PathFollower,
        )

        assert backend in _AVAILABLE_PATH_FOLLOWER_BACKENDS, (
            f"{backend} not listed in _AVAILABLE_PATH_FOLLOWER_BACKENDS"
        )
        mod = PathFollower(backend=backend)
        assert mod._backend == backend
        assert mod._backend_status.configured == backend

    def test_default_backend_is_nav_kernel(self) -> None:
        """Default backend string must be 'nav_kernel'."""
        from nav.local.path_follower import PathFollower

        mod = PathFollower()
        assert mod._backend == "nav_kernel"

    # ------------------------------------------------------------------ #
    # Port contract
    # ------------------------------------------------------------------ #

    def test_port_types_match_spec(self) -> None:
        """All In/Out ports have the correct msg_type."""
        from nav.local.path_follower import PathFollower

        mod = PathFollower(backend="pid")

        # -- Input ports --
        expected_in = {
            "odometry": Odometry,
            "local_path": Path,
            "control_hint": dict,
            "map_frame_jump_event": dict,
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
            "cmd_vel": Twist,
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
    # Lifecycle (pid backend)
    # ------------------------------------------------------------------ #

    def test_pid_backend_lifecycle(self) -> None:
        """setup() -> start() -> stop() works with backend='pid'."""
        from nav.local.path_follower import PathFollower

        mod = PathFollower(backend="pid")
        assert not mod._running

        mod.setup()
        mod.start()
        assert mod._running

        mod.stop()
        assert not mod._running

        # stop is idempotent
        mod.stop()
        assert not mod._running

    def test_pid_backend_alive_toggles(self) -> None:
        """alive Out[bool] publishes True on start(), False on stop()."""
        from nav.local.path_follower import PathFollower

        mod = PathFollower(backend="pid")
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
    # Data flow (pid backend)
    # ------------------------------------------------------------------ #

    def test_pid_backend_produces_cmd_vel(self) -> None:
        """Inject local_path + odometry -> cmd_vel published."""
        from nav.local.path_follower import PathFollower

        mod = PathFollower(backend="pid")
        mod.setup()
        mod.start()

        cmd_values: list[Twist] = []
        mod.cmd_vel._add_callback(cmd_values.append)

        # Inject a straight-line local_path
        poses = [
            PoseStamped(
                pose=Pose(
                    position=Vector3(0.0, 0.0, 0.0),
                    orientation=Quaternion(0, 0, 0, 1),
                ),
            ),
            PoseStamped(
                pose=Pose(
                    position=Vector3(1.0, 0.0, 0.0),
                    orientation=Quaternion(0, 0, 0, 1),
                ),
            ),
            PoseStamped(
                pose=Pose(
                    position=Vector3(2.0, 0.0, 0.0),
                    orientation=Quaternion(0, 0, 0, 1),
                ),
            ),
            PoseStamped(
                pose=Pose(
                    position=Vector3(3.0, 0.0, 0.0),
                    orientation=Quaternion(0, 0, 0, 1),
                ),
            ),
        ]
        path = Path(poses=poses, frame_id="map")
        # _on_path stores points and calls _pid_step immediately
        mod._on_path(path)

        # The _on_path handler calls _pid_step which publishes cmd_vel
        assert len(cmd_values) >= 1, (
            "expected cmd_vel to be published after path injection"
        )

        # Inject odometry 閳?should trigger another _pid_step
        odom = Odometry(pose=Pose(position=Vector3(0.0, 0.0, 0.0)))
        mod._on_odom(odom)

        assert len(cmd_values) >= 2, (
            "expected cmd_vel after odometry tick"
        )

        # Verify Twist shape
        last_cmd = cmd_values[-1]
        assert isinstance(last_cmd, Twist)
        assert hasattr(last_cmd, "linear")
        assert hasattr(last_cmd, "angular")

        mod.stop()

    # ------------------------------------------------------------------ #
    # Nav_core backend 閳?works when lingtu_nav_kernel.so is available
    # ------------------------------------------------------------------ #

    @pytest.mark.native
    def test_nav_kernel_backend_setup_succeeds(self, require_nav_kernel) -> None:
        """nav_kernel backend initializes when lingtu_nav_kernel.so is present."""
        from nav.local.path_follower import PathFollower

        require_nav_kernel(
            ("PathFollowerParams", "PathFollowerState", "compute_control"),
            "nav.path_follower",
        )

        mod = PathFollower(backend="nav_kernel")
        mod.setup()
        # When LingTu native navigation kernel is available, backend stays nav_kernel
        assert mod._backend in ("nav_kernel",), (
            f"expected 'nav_kernel', got '{mod._backend}'"
        )
        assert mod._nc is not None or mod._backend != "nav_kernel", (
            "nav_kernel setup should create _nc when available"
        )

        mod.start()
        assert mod._running
        mod.stop()
        assert not mod._running

    def test_nav_kernel_setup_uses_adapter_runtime_bundle(self, monkeypatch) -> None:
        """nav_kernel setup should consume the backend adapter bundle only."""
        from nav.local import path_follower as module_under_test
        from nav.local import path_follower_runtime as runtime_under_test
        from nav.local.path_follower_backend import (
            NavKernelPathFollowerAdapter,
        )

        sentinel_runtime = object()
        sentinel_params = object()
        sentinel_state = object()
        seen_tuning = []

        def fake_create_nav_kernel_path_follower_adapter_from_tuning(**tuning):
            seen_tuning.append(tuning)
            return NavKernelPathFollowerAdapter(
                runtime=sentinel_runtime,
                params=sentinel_params,
                state=sentinel_state,
            )

        monkeypatch.setattr(
            runtime_under_test,
            "create_nav_kernel_path_follower_adapter_from_tuning",
            fake_create_nav_kernel_path_follower_adapter_from_tuning,
        )

        mod = module_under_test.PathFollower(
            backend="nav_kernel",
            max_speed=0.7,
            lookahead=1.2,
            goal_tolerance=0.3,
            max_yaw_rate=0.6,
            two_way_drive=False,
        )
        mod.setup()

        assert seen_tuning == [
            {
                "max_speed": 0.7,
                "lookahead": 1.2,
                "goal_tolerance": 0.3,
                "max_yaw_rate": 0.6,
                "turn_speed_yaw_rate_start": 0.0,
                "turn_speed_min_scale": 1.0,
                "yaw_rate_gain": 7.5,
                "stop_yaw_rate_gain": 7.5,
                "dir_diff_thre": 0.1,
                "two_way_drive": False,
            }
        ]
        assert mod._nc is sentinel_runtime
        assert mod._nc_params is sentinel_params
        assert mod._nc_state is sentinel_state

    def test_nav_kernel_setup_falls_back_to_pid_without_nav_kernel(self, monkeypatch) -> None:
        """Default path follower must degrade to a ROS-free Python runtime."""
        from nav.local import path_follower as module_under_test
        from nav.local import path_follower_runtime as runtime_under_test
        from nav.local.path_follower_backend import (
            NavKernelPathFollowerAdapter,
            PidFallbackParams,
        )

        def fake_create_nav_kernel_path_follower_adapter_from_tuning(**_tuning):
            return NavKernelPathFollowerAdapter(
                runtime=None,
                degraded_reason="compatible LingTu native navigation kernel missing",
                build_hint="build nav core",
            )

        def fake_read_pid_fallback_params(max_speed: float) -> PidFallbackParams:
            return PidFallbackParams(
                k_v=0.8,
                l_min=0.35,
                l_max=2.4,
                a_max=1.2,
                v_max=max_speed,
                loaded_from_config=False,
            )

        monkeypatch.setattr(
            runtime_under_test,
            "create_nav_kernel_path_follower_adapter_from_tuning",
            fake_create_nav_kernel_path_follower_adapter_from_tuning,
        )
        monkeypatch.setattr(
            runtime_under_test,
            "read_pid_fallback_params",
            fake_read_pid_fallback_params,
        )

        mod = module_under_test.PathFollower(
            backend="nav_kernel",
            max_speed=0.75,
        )
        mod.setup()

        assert mod._backend == "pid"
        assert mod._backend_status.configured == "nav_kernel"
        assert mod._backend_status.effective == "pid"
        assert mod._backend_status.degraded is True
        assert mod._backend_status.degraded_reason == "compatible LingTu native navigation kernel missing"
        assert mod._nc is None
        assert mod._pp_k_v == 0.8
        assert mod._pp_v_max == 0.75

    def test_pid_setup_uses_adapter_owned_params(self, monkeypatch) -> None:
        """pid setup should read fallback params through the adapter helper."""
        from nav.local import path_follower as module_under_test
        from nav.local import path_follower_runtime as runtime_under_test
        from nav.local.path_follower_backend import PidFallbackParams

        seen_max_speed = []

        def fake_read_pid_fallback_params(max_speed: float) -> PidFallbackParams:
            seen_max_speed.append(max_speed)
            return PidFallbackParams(
                k_v=0.9,
                l_min=0.4,
                l_max=2.6,
                a_max=1.4,
                v_max=0.8,
                loaded_from_config=True,
            )

        monkeypatch.setattr(
            runtime_under_test,
            "read_pid_fallback_params",
            fake_read_pid_fallback_params,
        )

        mod = module_under_test.PathFollower(backend="pid", max_speed=0.75)
        mod.setup()

        assert seen_max_speed == [0.75]
        assert mod._pp_k_v == 0.9
        assert mod._pp_l_min == 0.4
        assert mod._pp_l_max == 2.6
        assert mod._pp_a_max == 1.4
        assert mod._pp_v_max == 0.8

    def test_module_does_not_construct_nav_kernel_config_directly(self) -> None:
        """PathFollower should leave nav_kernel config assembly to adapter."""
        module_path = (
            FsPath(__file__).resolve().parents[2]
            / "local"
            / "path_follower.py"
        )
        tree = ast.parse(module_path.read_text(encoding="utf-8"))

        imported_names = {
            alias.name
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom)
            for alias in node.names
        }
        called_names = {
            node.func.id
            for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        }

        assert "NavKernelPathFollowerConfig" not in imported_names
        assert "NavKernelPathFollowerConfig" not in called_names

    def test_module_delegates_backend_setup_to_runtime_helper(self) -> None:
        """PathFollower should not instantiate backend adapters directly."""
        module_path = (
            FsPath(__file__).resolve().parents[2]
            / "local"
            / "path_follower.py"
        )
        source = module_path.read_text(encoding="utf-8")

        assert "setup_path_follower_runtime" in source
        assert "create_nav_kernel_path_follower_adapter_from_tuning(" not in source
        assert "create_pure_pursuit_native_adapter(" not in source
        assert "read_pid_fallback_params(" not in source

    def test_legacy_pure_pursuit_backend_is_rejected(self) -> None:
        """The ROS2 NativeModule path follower is no longer a Module backend."""
        from nav.local.path_follower import PathFollower

        with pytest.raises(
            ValueError, match="Unknown path_follower backend 'pure_pursuit'"
        ):
            PathFollower(backend="pure_pursuit")

    # ------------------------------------------------------------------ #
    # Error handling
    # ------------------------------------------------------------------ #

    def test_unknown_backend_raises(self) -> None:
        """A bogus backend name must raise ValueError."""
        from nav.local.path_follower import PathFollower

        with pytest.raises(
            ValueError, match="Unknown path_follower backend 'bogus'"
        ):
            PathFollower(backend="bogus")

    # ------------------------------------------------------------------ #
    # Registry
    # ------------------------------------------------------------------ #

    def test_all_backends_registered(self) -> None:
        """All path_follower backends appear in the global Registry."""
        for backend in ("nav_kernel", "pid"):
            cls = registry_get("path_follower", backend)
            assert cls is not None, (
                f"path_follower/{backend} not registered"
            )

    def test_stop_clears_running_state(self) -> None:
        """stop() clears running state without managing external nodes."""
        from nav.local.path_follower import PathFollower

        mod = PathFollower(backend="pid")
        mod.setup()
        mod.start()

        mod._nc_path = ["dummy"]
        mod._path_points = np.array([[0.0, 0.0]])

        mod.stop()
        assert not mod._running

    def test_module_has_no_external_node_lifecycle(self) -> None:
        """PathFollower must stay inside process, not manage node start/stop."""
        from nav.local.path_follower_runtime import PathFollowerRuntime

        module_path = (
            FsPath(__file__).resolve().parents[2]
            / "local"
            / "path_follower.py"
        )
        source = module_path.read_text(encoding="utf-8")

        assert "node" not in PathFollowerRuntime.__dataclass_fields__
        assert "self._node" not in source
        assert "runtime.node" not in source
        assert "external node" not in source

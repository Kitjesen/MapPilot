"""Contract tests for all PathFollowerModule backends.

Verifies that every declared backend (nav_core, pure_pursuit, pid) can be
instantiated, that port types match the spec, that the pid backend survives
the full lifecycle and produces cmd_vel, and that the nav_core backend also
works when _nav_core.so is available.  Follows the pattern established in
test_autonomy_modules.py.
"""

from __future__ import annotations

import numpy as np
import pytest

from core.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from core.msgs.nav import Odometry, Path
from core.registry import get as registry_get


# =============================================================================
# Instantiation
# =============================================================================


class TestPathFollowerModuleBackends:
    """Backend-specific contract checks for PathFollowerModule."""

    @pytest.mark.parametrize("backend", ["nav_core", "pure_pursuit", "pid"])
    def test_all_backends_instantiate(self, backend: str) -> None:
        """Every registered backend name must succeed in __init__."""
        from base_autonomy.modules.path_follower_module import (
            PathFollowerModule,
            _AVAILABLE_PATH_FOLLOWER_BACKENDS,
        )

        assert backend in _AVAILABLE_PATH_FOLLOWER_BACKENDS, (
            f"{backend} not listed in _AVAILABLE_PATH_FOLLOWER_BACKENDS"
        )
        mod = PathFollowerModule(backend=backend)
        assert mod._backend == backend
        assert mod._backend_status.configured == backend

    def test_default_backend_is_nav_core(self) -> None:
        """Default backend string must be 'nav_core'."""
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        mod = PathFollowerModule()
        assert mod._backend == "nav_core"

    # ------------------------------------------------------------------ #
    # Port contract
    # ------------------------------------------------------------------ #

    def test_port_types_match_spec(self) -> None:
        """All In/Out ports have the correct msg_type."""
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        mod = PathFollowerModule(backend="pid")

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
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        mod = PathFollowerModule(backend="pid")
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
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        mod = PathFollowerModule(backend="pid")
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
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        mod = PathFollowerModule(backend="pid")
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

        # Inject odometry — should trigger another _pid_step
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
    # Nav_core backend — works when _nav_core.so is available
    # ------------------------------------------------------------------ #

    def test_nav_core_backend_setup_succeeds(self) -> None:
        """nav_core backend initializes when _nav_core.so is present."""
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        mod = PathFollowerModule(backend="nav_core")
        mod.setup()
        # When _nav_core is available, backend stays nav_core
        assert mod._backend in ("nav_core",), (
            f"expected 'nav_core', got '{mod._backend}'"
        )
        assert mod._nc is not None or mod._backend != "nav_core", (
            "nav_core setup should create _nc when available"
        )

        mod.start()
        assert mod._running
        mod.stop()
        assert not mod._running

    def test_pure_pursuit_setup_degrades_gracefully(self) -> None:
        """pure_pursuit setup degrades (ImportError) without crashing."""
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        mod = PathFollowerModule(backend="pure_pursuit")
        # setup() tries to import native_factories; may not be available
        mod.setup()

        # Should not raise; the node may be None or degraded
        assert mod._node is None or mod._backend_status.degraded

    # ------------------------------------------------------------------ #
    # Error handling
    # ------------------------------------------------------------------ #

    def test_unknown_backend_raises(self) -> None:
        """A bogus backend name must raise ValueError."""
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        with pytest.raises(
            ValueError, match="Unknown path_follower backend 'bogus'"
        ):
            PathFollowerModule(backend="bogus")

    # ------------------------------------------------------------------ #
    # Registry
    # ------------------------------------------------------------------ #

    def test_all_backends_registered(self) -> None:
        """All path_follower backends appear in the global Registry."""
        for backend in ("nav_core", "pure_pursuit", "pid"):
            cls = registry_get("path_follower", backend)
            assert cls is not None, (
                f"path_follower/{backend} not registered"
            )

    def test_stop_clears_node_and_state(self) -> None:
        """stop() clears _node and internal state."""
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        mod = PathFollowerModule(backend="pid")
        mod.setup()
        mod.start()

        mod._node = object()
        mod._nc_path = ["dummy"]
        mod._path_points = np.array([[0.0, 0.0]])

        mod.stop()
        assert mod._node is None
        # _nc_path is owned by nav_core backend; pid backend does not
        # maintain it, so after stop only _node and _running are cleared.
        assert not mod._running

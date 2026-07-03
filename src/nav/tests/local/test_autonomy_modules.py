"""Contract tests for base_autonomy modules.

Verifies module instantiation, port registration, lifecycle (setup/start/stop),
unknown backend rejection, and alive port toggling for:
  - Terrain
  - LocalPlanner
  - PathFollower

These are CONTRACT tests; they verify the module interface contract, not
internal implementation details or algorithmic correctness.
"""
from __future__ import annotations

import numpy as np
import pytest

# =============================================================================
# Terrain
# =============================================================================

class TestTerrain:
    """Contract tests for Terrain (layer=2, terrain analysis)."""

    @pytest.mark.parametrize("backend", ["nanobind", "simple"])
    def test_instantiation_with_backend(self, backend: str):
        """Creating a Terrain with a valid backend should succeed."""
        from nav.local.terrain import Terrain

        mod = Terrain(backend=backend)
        assert mod._backend == backend
        assert mod._backend_status.configured == backend
        assert mod._backend_status.effective == backend
        assert not mod._backend_status.degraded

    def test_ports(self):
        """All In/Out ports declared on the class must be registered."""
        from nav.local.terrain import Terrain
        from runtime.msgs.map import MapCloudFrame
        from runtime.msgs.nav import Odometry
        from runtime.msgs.sensor import PointCloud2

        mod = Terrain(backend="simple")

        # -- Input ports --
        assert "odometry" in mod._ports_in
        assert mod._ports_in["odometry"].msg_type is Odometry

        assert "map_cloud" in mod._ports_in
        assert mod._ports_in["map_cloud"].msg_type is PointCloud2

        assert "map_cloud_frame" in mod._ports_in
        assert mod._ports_in["map_cloud_frame"].msg_type is MapCloudFrame

        assert len(mod._ports_in) == 3, f"expected 3 In ports, got {list(mod._ports_in)}"

        # -- Output ports --
        assert "terrain_map" in mod._ports_out
        assert mod._ports_out["terrain_map"].msg_type is PointCloud2

        assert "terrain_map_ext" in mod._ports_out
        assert mod._ports_out["terrain_map_ext"].msg_type is PointCloud2

        assert "traversability" in mod._ports_out
        assert mod._ports_out["traversability"].msg_type is dict

        assert "elevation_map" in mod._ports_out
        assert mod._ports_out["elevation_map"].msg_type is np.ndarray

        assert "alive" in mod._ports_out
        assert mod._ports_out["alive"].msg_type is bool

        assert len(mod._ports_out) == 5, f"expected 5 Out ports, got {list(mod._ports_out)}"

    def test_lifecycle(self):
        """setup() -> start() -> stop() transitions without error (simple backend)."""
        from nav.local.terrain import Terrain

        mod = Terrain(backend="simple")
        assert not mod._running

        mod.setup()
        mod.start()
        assert mod._running

        mod.stop()
        assert not mod._running

        # stop is idempotent
        mod.stop()
        assert not mod._running

    def test_unknown_backend_raises(self):
        """Passing a bogus backend name must raise ValueError."""
        from nav.local.terrain import Terrain

        with pytest.raises(ValueError, match="Unknown terrain backend 'bogus'"):
            Terrain(backend="bogus")

    def test_alive_toggles_on_start_stop(self):
        """alive Out[bool] must publish True on start(), False on stop()."""
        from nav.local.terrain import Terrain

        mod = Terrain(backend="simple")
        mod.setup()

        # Spy on Published values via internal callback (Out uses __slots__,
        # so mock.patch.object on instance publish is not supported).
        alive_values: list[bool] = []
        mod.alive._add_callback(alive_values.append)

        mod.start()
        assert len(alive_values) >= 1, "expected at least one publish on start"
        assert alive_values[-1] is True

        mod.stop()
        assert len(alive_values) >= 2, "expected a publish on stop"
        assert alive_values[-1] is False


# =============================================================================
# LocalPlanner
# =============================================================================

class TestLocalPlanner:
    """Contract tests for LocalPlanner (layer=2, local path planning)."""

    @pytest.mark.parametrize("backend", ["simple"])
    def test_instantiation_with_backend(self, backend: str):
        """Creating a LocalPlanner with a valid backend should succeed."""
        from nav.services.plan.local_planner.service import LocalPlanner

        mod = LocalPlanner(backend=backend)
        assert mod._backend == backend

    def test_ports(self):
        """All In/Out ports declared on the class must be registered."""
        from nav.services.plan.local_planner.service import LocalPlanner
        from runtime.msgs.geometry import PoseStamped
        from runtime.msgs.nav import Odometry, Path
        from runtime.msgs.sensor import PointCloud2

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
            "map_odom_tf": dict,
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
        assert "local_path" in mod._ports_out
        assert mod._ports_out["local_path"].msg_type is Path

        assert "control_hint" in mod._ports_out
        assert mod._ports_out["control_hint"].msg_type is dict

        assert "alive" in mod._ports_out
        assert mod._ports_out["alive"].msg_type is bool

        assert len(mod._ports_out) == 3, (
            f"expected 3 Out ports, got {list(mod._ports_out)}"
        )

    def test_lifecycle(self):
        """setup() -> start() -> stop() transitions without error (simple backend)."""
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

    def test_unknown_backend_raises(self):
        """Passing a bogus backend name must raise ValueError."""
        from nav.services.plan.local_planner.service import LocalPlanner

        with pytest.raises(ValueError, match="Unknown local_planner backend 'bogus'"):
            LocalPlanner(backend="bogus")

    def test_alive_toggles_on_start_stop(self):
        """alive Out[bool] must publish True on start(), False on stop()."""
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


# =============================================================================
# PathFollower
# =============================================================================

class TestPathFollower:
    """Contract tests for PathFollower (layer=2, path tracking)."""

    @pytest.mark.parametrize("backend", ["nav_kernel", "pid"])
    def test_instantiation_with_backend(self, backend: str):
        """Creating a PathFollower with a valid backend should succeed."""
        from nav.local.path_follower import PathFollower

        mod = PathFollower(backend=backend)
        assert mod._backend == backend

    def test_ports(self):
        """All In/Out ports declared on the class must be registered."""
        from nav.local.path_follower import PathFollower
        from runtime.msgs.geometry import Twist
        from runtime.msgs.nav import Odometry, Path

        mod = PathFollower(backend="pid")

        # -- Input ports --
        expected_in = {
            "odometry": Odometry,
            "local_path": Path,
            "control_hint": dict,
            "map_odom_tf": dict,
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

    def test_lifecycle(self):
        """setup() -> start() -> stop() transitions without error (pid backend)."""
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

    def test_unknown_backend_raises(self):
        """Passing a bogus backend name must raise ValueError."""
        from nav.local.path_follower import PathFollower

        with pytest.raises(ValueError, match="Unknown path_follower backend 'bogus'"):
            PathFollower(backend="bogus")

    def test_alive_toggles_on_start_stop(self):
        """alive Out[bool] must publish True on start(), False on stop()."""
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


# =============================================================================
# Cross-module contract checks
# =============================================================================

def test_all_three_modules_have_alive_port():
    """Every autonomy module must expose an alive Out[bool] port."""
    from nav.services.plan.local_planner.service import LocalPlanner
    from nav.local.path_follower import PathFollower
    from nav.local.terrain import Terrain

    for cls in (Terrain, LocalPlanner, PathFollower):
        backend = "simple" if cls is not PathFollower else "pid"
        mod = cls(backend=backend)
        assert "alive" in mod._ports_out
        assert mod._ports_out["alive"].msg_type is bool


def test_add_autonomy_stack_defaults_to_ros_free_backends():
    """Convenience helper must not default to ROS2 NativeModule backends."""
    from nav.local import add_autonomy_stack
    from runtime.blueprint import Blueprint

    bp = Blueprint()
    add_autonomy_stack(bp)

    configs = {entry.name: entry.config for entry in bp._entries}
    assert configs["nav.terrain"]["backend"] == "nanobind"
    assert configs["nav.local_planner"]["backend"] == "nanobind"
    assert configs["nav.path_follower"]["backend"] == "nav_kernel"


def test_add_autonomy_stack_does_not_send_local_only_backend_to_terrain():
    """Local-planner fallback aliases must not become terrain backend names."""
    from nav.local import add_autonomy_stack
    from runtime.blueprint import Blueprint

    bp = Blueprint()
    add_autonomy_stack(bp, backend="cmu_py", path_follower_backend="pid")

    configs = {entry.name: entry.config for entry in bp._entries}
    assert configs["nav.terrain"]["backend"] == "nanobind"
    assert configs["nav.local_planner"]["backend"] == "cmu_py"
    assert configs["nav.path_follower"]["backend"] == "pid"


# =============================================================================
# Integration: all three modules together in a Blueprint
# =============================================================================

def test_three_module_blueprint_wiring():
    """All three base_autonomy modules coexist in a single Blueprint.

    Verifies that Terrain, LocalPlanner, and PathFollower
    can be added to a Blueprint and built without wiring conflicts.
    """
    from nav.services.plan.local_planner.service import LocalPlanner
    from nav.local.path_follower import PathFollower
    from nav.local.terrain import Terrain
    from runtime.blueprint import Blueprint

    bp = Blueprint()
    bp.add(Terrain, backend="simple")
    bp.add(LocalPlanner, backend="simple")
    bp.add(PathFollower, backend="pid")

    system = bp.build()
    system.start()

    assert "nav.terrain" in system.modules
    assert "nav.local_planner" in system.modules
    assert "nav.path_follower" in system.modules

    # All three modules must be in running state
    assert system.modules["nav.terrain"]._running
    assert system.modules["nav.local_planner"]._running
    assert system.modules["nav.path_follower"]._running

    system.stop()

"""Contract tests for base_autonomy modules.

Verifies module instantiation, port registration, lifecycle (setup/start/stop),
unknown backend rejection, and alive port toggling for:
  - TerrainModule
  - LocalPlannerModule
  - PathFollowerModule

These are CONTRACT tests — they verify the module interface contract, not
internal implementation details or algorithmic correctness.
"""
from __future__ import annotations

import numpy as np
import pytest


# =============================================================================
# TerrainModule
# =============================================================================

class TestTerrainModule:
    """Contract tests for TerrainModule (layer=2, terrain analysis)."""

    @pytest.mark.parametrize("backend", ["nanobind", "simple"])
    def test_instantiation_with_backend(self, backend: str):
        """Creating a TerrainModule with a valid backend should succeed."""
        from base_autonomy.modules.terrain_module import TerrainModule

        mod = TerrainModule(backend=backend)
        assert mod._backend == backend
        assert mod._backend_status.configured == backend
        assert mod._backend_status.effective == backend
        assert not mod._backend_status.degraded

    def test_ports(self):
        """All In/Out ports declared on the class must be registered."""
        from base_autonomy.modules.terrain_module import TerrainModule
        from core.msgs.nav import Odometry
        from core.msgs.sensor import PointCloud2

        mod = TerrainModule(backend="simple")

        # -- Input ports --
        assert "odometry" in mod._ports_in
        assert mod._ports_in["odometry"].msg_type is Odometry

        assert "map_cloud" in mod._ports_in
        assert mod._ports_in["map_cloud"].msg_type is PointCloud2

        assert len(mod._ports_in) == 2, f"expected 2 In ports, got {list(mod._ports_in)}"

        # -- Output ports --
        assert "terrain_map" in mod._ports_out
        assert mod._ports_out["terrain_map"].msg_type is PointCloud2

        assert "traversability" in mod._ports_out
        assert mod._ports_out["traversability"].msg_type is dict

        assert "elevation_map" in mod._ports_out
        assert mod._ports_out["elevation_map"].msg_type is np.ndarray

        assert "alive" in mod._ports_out
        assert mod._ports_out["alive"].msg_type is bool

        assert len(mod._ports_out) == 4, f"expected 4 Out ports, got {list(mod._ports_out)}"

    def test_lifecycle(self):
        """setup() -> start() -> stop() transitions without error (simple backend)."""
        from base_autonomy.modules.terrain_module import TerrainModule

        mod = TerrainModule(backend="simple")
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
        from base_autonomy.modules.terrain_module import TerrainModule

        with pytest.raises(ValueError, match="Unknown terrain backend 'bogus'"):
            TerrainModule(backend="bogus")

    def test_alive_toggles_on_start_stop(self):
        """alive Out[bool] must publish True on start(), False on stop()."""
        from base_autonomy.modules.terrain_module import TerrainModule

        mod = TerrainModule(backend="simple")
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
# LocalPlannerModule
# =============================================================================

class TestLocalPlannerModule:
    """Contract tests for LocalPlannerModule (layer=2, local path planning)."""

    @pytest.mark.parametrize("backend", ["simple"])
    def test_instantiation_with_backend(self, backend: str):
        """Creating a LocalPlannerModule with a valid backend should succeed."""
        from base_autonomy.modules.local_planner_module import LocalPlannerModule

        mod = LocalPlannerModule(backend=backend)
        assert mod._backend == backend

    def test_ports(self):
        """All In/Out ports declared on the class must be registered."""
        from base_autonomy.modules.local_planner_module import LocalPlannerModule
        from core.msgs.geometry import PoseStamped
        from core.msgs.nav import Odometry, Path
        from core.msgs.sensor import PointCloud2

        mod = LocalPlannerModule(backend="simple")

        # -- Input ports --
        expected_in = {
            "odometry": Odometry,
            "terrain_map": PointCloud2,
            "waypoint": PoseStamped,
            "global_path": list,
            "clear_path": bool,
            "map_frame_jump_event": dict,
            "boundary": PointCloud2,
            "added_obstacles": PointCloud2,
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
        from base_autonomy.modules.local_planner_module import LocalPlannerModule

        mod = LocalPlannerModule(backend="simple")
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
        from base_autonomy.modules.local_planner_module import LocalPlannerModule

        with pytest.raises(ValueError, match="Unknown local_planner backend 'bogus'"):
            LocalPlannerModule(backend="bogus")

    def test_alive_toggles_on_start_stop(self):
        """alive Out[bool] must publish True on start(), False on stop()."""
        from base_autonomy.modules.local_planner_module import LocalPlannerModule

        mod = LocalPlannerModule(backend="simple")
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
# PathFollowerModule
# =============================================================================

class TestPathFollowerModule:
    """Contract tests for PathFollowerModule (layer=2, path tracking)."""

    @pytest.mark.parametrize("backend", ["pure_pursuit", "pid"])
    def test_instantiation_with_backend(self, backend: str):
        """Creating a PathFollowerModule with a valid backend should succeed."""
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        mod = PathFollowerModule(backend=backend)
        assert mod._backend == backend

    def test_ports(self):
        """All In/Out ports declared on the class must be registered."""
        from base_autonomy.modules.path_follower_module import PathFollowerModule
        from core.msgs.geometry import Twist
        from core.msgs.nav import Odometry, Path

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

    def test_lifecycle(self):
        """setup() -> start() -> stop() transitions without error (pid backend)."""
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

    def test_unknown_backend_raises(self):
        """Passing a bogus backend name must raise ValueError."""
        from base_autonomy.modules.path_follower_module import PathFollowerModule

        with pytest.raises(ValueError, match="Unknown path_follower backend 'bogus'"):
            PathFollowerModule(backend="bogus")

    def test_alive_toggles_on_start_stop(self):
        """alive Out[bool] must publish True on start(), False on stop()."""
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


# =============================================================================
# Cross-module contract checks
# =============================================================================

def test_all_three_modules_have_alive_port():
    """Every autonomy module must expose an alive Out[bool] port."""
    from base_autonomy.modules.terrain_module import TerrainModule
    from base_autonomy.modules.local_planner_module import LocalPlannerModule
    from base_autonomy.modules.path_follower_module import PathFollowerModule

    for cls in (TerrainModule, LocalPlannerModule, PathFollowerModule):
        backend = "simple" if cls is not PathFollowerModule else "pid"
        mod = cls(backend=backend)
        assert "alive" in mod._ports_out
        assert mod._ports_out["alive"].msg_type is bool


# =============================================================================
# Integration: all three modules together in a Blueprint
# =============================================================================

def test_three_module_blueprint_wiring():
    """All three base_autonomy modules coexist in a single Blueprint.

    Verifies that TerrainModule, LocalPlannerModule, and PathFollowerModule
    can be added to a Blueprint and built without wiring conflicts.
    """
    from base_autonomy.modules.terrain_module import TerrainModule
    from base_autonomy.modules.local_planner_module import LocalPlannerModule
    from base_autonomy.modules.path_follower_module import PathFollowerModule
    from core.blueprint import Blueprint

    bp = Blueprint()
    bp.add(TerrainModule, backend="simple")
    bp.add(LocalPlannerModule, backend="simple")
    bp.add(PathFollowerModule, backend="pid")

    system = bp.build()
    system.start()

    assert "TerrainModule" in system.modules
    assert "LocalPlannerModule" in system.modules
    assert "PathFollowerModule" in system.modules

    # All three modules must be in running state
    assert system.modules["TerrainModule"]._running
    assert system.modules["LocalPlannerModule"]._running
    assert system.modules["PathFollowerModule"]._running

    system.stop()

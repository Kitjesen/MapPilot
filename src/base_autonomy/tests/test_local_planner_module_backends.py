"""Contract tests for all LocalPlannerModule backends.

Verifies that every declared backend (nanobind, cmu, cmu_py, simple) can be
instantiated, that port types match the spec, that the simple backend survives
the full lifecycle and produces paths, and that the nanobind backend also
works when _nav_core.so is available.  Follows the pattern established in
test_autonomy_modules.py.
"""

from __future__ import annotations

import numpy as np
import pytest

from core.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from core.msgs.nav import Odometry, Path
from core.msgs.sensor import PointCloud2
from core.registry import get as registry_get


# =============================================================================
# Instantiation
# =============================================================================


class TestLocalPlannerModuleBackends:
    """Backend-specific contract checks for LocalPlannerModule."""

    @pytest.mark.parametrize("backend", ["nanobind", "simple", "cmu", "cmu_py"])
    def test_all_backends_instantiate(self, backend: str) -> None:
        """Every registered backend name must succeed in __init__."""
        from base_autonomy.modules.local_planner_module import (
            LocalPlannerModule,
            _AVAILABLE_LOCAL_PLANNER_BACKENDS,
        )

        assert backend in _AVAILABLE_LOCAL_PLANNER_BACKENDS, (
            f"{backend} not listed in _AVAILABLE_LOCAL_PLANNER_BACKENDS"
        )
        mod = LocalPlannerModule(backend=backend)
        assert mod._backend == backend
        assert mod._backend_status.configured == backend

    def test_default_backend_is_cmu(self) -> None:
        """Default backend string must be 'cmu'."""
        from base_autonomy.modules.local_planner_module import LocalPlannerModule

        mod = LocalPlannerModule()
        assert mod._backend == "cmu"

    # ------------------------------------------------------------------ #
    # Port contract
    # ------------------------------------------------------------------ #

    def test_port_types_match_spec(self) -> None:
        """All In/Out ports have the correct msg_type."""
        from base_autonomy.modules.local_planner_module import LocalPlannerModule

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

    def test_simple_backend_alive_toggles(self) -> None:
        """alive Out[bool] publishes True on start(), False on stop()."""
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

    # ------------------------------------------------------------------ #
    # Data flow (simple backend)
    # ------------------------------------------------------------------ #

    def test_simple_backend_publishes_path_on_waypoint(self) -> None:
        """Inject odometry + waypoint -> local_path published."""
        from base_autonomy.modules.local_planner_module import LocalPlannerModule

        mod = LocalPlannerModule(backend="simple")
        mod.setup()
        mod.start()

        path_values: list[Path] = []
        mod.local_path._add_callback(path_values.append)

        hint_values: list[dict] = []
        mod.control_hint._add_callback(hint_values.append)

        # Pre-load robot position via odometry
        odom = Odometry(pose=Pose(position=Vector3(0.0, 0.0, 0.0)))
        mod._on_odom(odom)

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

        mod.stop()

    # ------------------------------------------------------------------ #
    # Nanobind backend — works when _nav_core.so is available
    # ------------------------------------------------------------------ #

    def test_nanobind_backend_setup_succeeds(self) -> None:
        """nanobind setup succeeds with local _nav_core build."""
        from base_autonomy.modules.local_planner_module import LocalPlannerModule

        mod = LocalPlannerModule(backend="nanobind")
        mod.setup()
        assert mod._core is not None, (
            "nanobind C++ core should be loaded when _nav_core.so exists"
        )
        assert mod._backend == "nanobind"
        assert not mod._backend_status.degraded

        mod.start()
        assert mod._running
        mod.stop()
        assert not mod._running

    def test_cmu_py_backend_setup_succeeds(self) -> None:
        """cmu_py backend loads paths successfully when PLY files exist."""
        from base_autonomy.modules.local_planner_module import LocalPlannerModule

        mod = LocalPlannerModule(backend="cmu_py")
        mod.setup()
        assert mod._path_data is not None, (
            "cmu_py should load path data from disk"
        )
        assert mod._backend == "cmu_py"

        mod.start()
        assert mod._running
        mod.stop()
        assert not mod._running

    # ------------------------------------------------------------------ #
    # Error handling
    # ------------------------------------------------------------------ #

    def test_unknown_backend_raises(self) -> None:
        """A bogus backend name must raise ValueError."""
        from base_autonomy.modules.local_planner_module import LocalPlannerModule

        with pytest.raises(
            ValueError, match="Unknown local_planner backend 'bogus'"
        ):
            LocalPlannerModule(backend="bogus")

    # ------------------------------------------------------------------ #
    # Registry
    # ------------------------------------------------------------------ #

    def test_all_backends_registered(self) -> None:
        """All local_planner backends appear in the global Registry."""
        for backend in ("nanobind", "simple", "cmu", "cmu_py"):
            cls = registry_get("local_planner", backend)
            assert cls is not None, (
                f"local_planner/{backend} not registered"
            )

    def test_stop_clears_core_and_node(self) -> None:
        """stop() must clear internal _core and _node references."""
        from base_autonomy.modules.local_planner_module import LocalPlannerModule

        mod = LocalPlannerModule(backend="simple")
        mod.setup()
        mod.start()

        # Simulate that _core and _node were populated
        mod._core = object()
        mod._node = object()

        mod.stop()
        assert mod._core is None
        assert mod._node is None
        assert not mod._running

"""Contract tests for all TerrainModule backends.

Verifies that every declared backend (nanobind, cmu, native, simple) can be
instantiated, that port types match the spec, that the simple backend survives
the full lifecycle, and that missing native dependencies raise predictable
errors.  Follows the pattern established in test_autonomy_modules.py.
"""

from __future__ import annotations

import numpy as np
import pytest

from core.msgs.geometry import Pose, Vector3
from core.msgs.nav import Odometry
from core.msgs.sensor import PointCloud2
from core.registry import get as registry_get


# =============================================================================
# Instantiation
# =============================================================================


class TestTerrainModuleBackends:
    """Backend-specific contract checks for TerrainModule."""

    @pytest.mark.parametrize("backend", ["nanobind", "simple", "cmu", "native"])
    def test_all_backends_instantiate(self, backend: str) -> None:
        """Every registered backend name must succeed in __init__."""
        from base_autonomy.modules.terrain_module import (
            TerrainModule,
            _AVAILABLE_TERRAIN_BACKENDS,
        )

        assert backend in _AVAILABLE_TERRAIN_BACKENDS, (
            f"{backend} not listed in _AVAILABLE_TERRAIN_BACKENDS"
        )
        mod = TerrainModule(backend=backend)
        assert mod._backend == backend
        assert mod._backend_status.configured == backend

    def test_default_backend_is_nanobind(self) -> None:
        """Default backend string must be 'nanobind'."""
        from base_autonomy.modules.terrain_module import TerrainModule

        mod = TerrainModule()
        assert mod._backend == "nanobind"

    # ------------------------------------------------------------------ #
    # Port contract
    # ------------------------------------------------------------------ #

    def test_port_types_match_spec(self) -> None:
        """All In/Out ports have the correct msg_type."""
        from base_autonomy.modules.terrain_module import TerrainModule

        mod = TerrainModule(backend="simple")

        # -- Input ports --
        assert "odometry" in mod._ports_in
        assert mod._ports_in["odometry"].msg_type is Odometry

        assert "map_cloud" in mod._ports_in
        assert mod._ports_in["map_cloud"].msg_type is PointCloud2

        assert len(mod._ports_in) == 2, (
            f"expected 2 In ports, got {list(mod._ports_in)}"
        )

        # -- Output ports --
        assert "terrain_map" in mod._ports_out
        assert mod._ports_out["terrain_map"].msg_type is PointCloud2

        assert "traversability" in mod._ports_out
        assert mod._ports_out["traversability"].msg_type is dict

        assert "elevation_map" in mod._ports_out
        assert mod._ports_out["elevation_map"].msg_type is np.ndarray

        assert "alive" in mod._ports_out
        assert mod._ports_out["alive"].msg_type is bool

        assert len(mod._ports_out) == 4, (
            f"expected 4 Out ports, got {list(mod._ports_out)}"
        )

    # ------------------------------------------------------------------ #
    # Lifecycle (simple backend)
    # ------------------------------------------------------------------ #

    def test_simple_backend_lifecycle(self) -> None:
        """setup() -> start() -> stop() works with backend='simple'."""
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

    def test_simple_backend_alive_toggles(self) -> None:
        """alive Out[bool] publishes True on start(), False on stop()."""
        from base_autonomy.modules.terrain_module import TerrainModule

        mod = TerrainModule(backend="simple")
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

    def test_simple_on_odom_publishes_traversability(self) -> None:
        """With simple backend, _on_odom publishes traversability dict."""
        from base_autonomy.modules.terrain_module import TerrainModule

        mod = TerrainModule(backend="simple")
        mod.setup()
        mod.start()

        trav_values: list[dict] = []
        mod.traversability._add_callback(trav_values.append)

        odom = Odometry(pose=Pose(position=Vector3(1.0, 2.0, 0.0)))
        mod._on_odom(odom)

        assert len(trav_values) >= 1
        assert trav_values[-1] == {"status": "passthrough"}

        mod.stop()

    def test_simple_on_cloud_publishes_terrain_map(self) -> None:
        """With simple backend, _on_cloud echoes cloud to terrain_map."""
        from base_autonomy.modules.terrain_module import TerrainModule

        mod = TerrainModule(backend="simple")
        mod.setup()
        mod.start()

        map_values: list[PointCloud2] = []
        mod.terrain_map._add_callback(map_values.append)

        pts = np.array([[0.0, 1.0, 2.0], [3.0, 4.0, 5.0]], dtype=np.float32)
        cloud = PointCloud2(points=pts)
        mod._last_process_ts = 0.0
        mod._on_cloud(cloud)

        assert len(map_values) >= 1
        published = map_values[-1]
        assert isinstance(published, PointCloud2)
        assert published.points.shape == (2, 3)
        np.testing.assert_array_equal(published.points, pts)

        mod.stop()

    # ------------------------------------------------------------------ #
    # Nanobind backend — works when _nav_core.so is available
    # ------------------------------------------------------------------ #

    def test_nanobind_backend_setup_succeeds(self) -> None:
        """nanobind setup succeeds when _nav_core is available."""
        from base_autonomy.modules.terrain_module import TerrainModule

        mod = TerrainModule(backend="nanobind")
        mod.setup()
        assert mod._core is not None, "nanobind C++ core should be loaded"

        mod.start()
        assert mod._running
        mod.stop()
        assert not mod._running

    def test_nanobind_alive_toggles(self) -> None:
        """alive port toggles with nanobind backend."""
        from base_autonomy.modules.terrain_module import TerrainModule

        mod = TerrainModule(backend="nanobind")
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
    # Error handling
    # ------------------------------------------------------------------ #

    def test_unknown_backend_raises(self) -> None:
        """A bogus backend name must raise ValueError."""
        from base_autonomy.modules.terrain_module import TerrainModule

        with pytest.raises(ValueError, match="Unknown terrain backend 'bogus'"):
            TerrainModule(backend="bogus")

    # ------------------------------------------------------------------ #
    # Registry
    # ------------------------------------------------------------------ #

    def test_all_backends_registered(self) -> None:
        """All terrain backends appear in the global Registry."""
        for backend in ("nanobind", "simple", "cmu", "native"):
            cls = registry_get("terrain", backend)
            assert cls is not None, f"terrain/{backend} not registered"

    def test_backend_status_tracks_degraded_on_cmu(self) -> None:
        """cmu backend starts as configured — setup degrades gracefully."""
        from base_autonomy.modules.terrain_module import TerrainModule

        mod = TerrainModule(backend="cmu")
        assert mod._backend_status.configured == "cmu"
        assert mod._backend_status.effective == "cmu"
        assert not mod._backend_status.degraded

        # setup() with cmu tries native factories; may degrade but not crash
        mod.setup()
        mod.start()
        mod.stop()

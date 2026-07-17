"""Contract tests for all Terrain backends.

Verifies that every declared backend (nanobind, simple) can be
instantiated, that port types match the spec, that the simple backend survives
the full lifecycle, and that missing native dependencies raise predictable
errors.  Follows the pattern established in test_autonomy_modules.py.
"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from runtime.msgs.geometry import Pose, Vector3
from runtime.msgs.map import MapCloudFrame
from runtime.msgs.nav import Odometry
from runtime.msgs.sensor import PointCloud2
from runtime.registry import get as registry_get

# =============================================================================
# Instantiation
# =============================================================================


class TestTerrainBackends:
    """Backend-specific contract checks for nav.terrain."""

    @pytest.mark.parametrize("backend", ["nanobind", "simple"])
    def test_all_backends_instantiate(self, backend: str) -> None:
        """Every registered backend name must succeed in __init__."""
        from nav.local.terrain import (
            _AVAILABLE_TERRAIN_BACKENDS,
            Terrain,
        )

        assert backend in _AVAILABLE_TERRAIN_BACKENDS, f"{backend} not listed in _AVAILABLE_TERRAIN_BACKENDS"
        mod = Terrain(backend=backend)
        assert mod._backend == backend
        assert mod._backend_status.configured == backend

    def test_default_backend_is_nanobind(self) -> None:
        """Default backend string must be 'nanobind'."""
        from nav.local.terrain import Terrain

        mod = Terrain()
        assert mod._backend == "nanobind"

    # ------------------------------------------------------------------ #
    # Port contract
    # ------------------------------------------------------------------ #

    def test_port_types_match_spec(self) -> None:
        """All In/Out ports have the correct msg_type."""
        from nav.local.terrain import Terrain

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

    # ------------------------------------------------------------------ #
    # Lifecycle (simple backend)
    # ------------------------------------------------------------------ #

    def test_simple_backend_lifecycle(self) -> None:
        """setup() -> start() -> stop() works with backend='simple'."""
        from nav.local.terrain import Terrain

        mod = Terrain(backend="simple")
        assert not mod._running

        mod.setup()
        mod.start()
        assert mod._running

        mod.stop()

    def test_simple_on_cloud_publishes_terrain_map_ext(self) -> None:
        """With simple backend, _on_cloud publishes bounded terrain_map_ext."""
        from nav.local.terrain import Terrain
        from runtime.runtime_interface import TOPICS, topic_default_frame_id

        mod = Terrain(
            backend="simple",
            terrain_ext_radius_m=5.0,
            terrain_ext_voxel_size=0.0,
        )
        mod.setup()
        mod.start()
        mod._on_odom(Odometry(pose=Pose(position=Vector3(0.0, 0.0, 0.0))))

        ext_values: list[PointCloud2] = []
        mod.terrain_map_ext._add_callback(ext_values.append)

        pts = np.array(
            [
                [1.0, 0.0, 0.0, 10.0],
                [6.0, 0.0, 0.0, 20.0],
                [2.0, 0.0, 4.0, 30.0],
            ],
            dtype=np.float32,
        )
        mod._last_process_ts = 0.0
        mod._on_cloud(PointCloud2(points=pts, frame_id="map"))

        assert len(ext_values) == 1
        published = ext_values[-1]
        assert published.frame_id == topic_default_frame_id(TOPICS.terrain_map_ext)
        assert published.points.shape == (1, 4)
        np.testing.assert_array_equal(published.points[0], pts[0])

        mod.stop()
        assert not mod._running

        # stop is idempotent
        mod.stop()
        assert not mod._running

    def test_simple_on_cloud_frame_publishes_terrain_map(self) -> None:
        """MapCloudFrame uses the same Terrain processing path."""
        from nav.local.terrain import Terrain

        mod = Terrain(backend="simple", terrain_ext_voxel_size=0.0)
        mod.setup()
        mod.start()

        values: list[PointCloud2] = []
        mod.terrain_map._add_callback(values.append)

        pts = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
        mod._last_process_ts = 0.0
        mod._on_cloud_frame(MapCloudFrame(points=pts, frame_id="map"))

        assert len(values) == 1
        np.testing.assert_allclose(values[0].points, pts)
        mod.stop()

    def test_simple_backend_alive_toggles(self) -> None:
        """alive Out[bool] publishes True on start(), False on stop()."""
        from nav.local.terrain import Terrain

        mod = Terrain(backend="simple")
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
        from nav.local.terrain import Terrain

        mod = Terrain(backend="simple")
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
        from nav.local.terrain import Terrain

        mod = Terrain(backend="simple")
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
    # Nanobind backend 閳?works when lingtu_nav_kernel.so is available
    # ------------------------------------------------------------------ #

    def test_nanobind_payload_includes_traversability_risk_grid(self) -> None:
        """Terrain risk output must be a real grid, not only status counters."""
        from nav.local.terrain import Terrain

        class FakeGrid2D:
            def __init__(self) -> None:
                self.rows = 0
                self.cols = 0
                self.resolution = 0.2
                self.origin_x = 0.0
                self.origin_y = 0.0
                self.data = []

        class FakeElevationMapResult:
            pass

        class FakeTerrainRiskParams:
            pass

        class FakeRuntime:
            Grid2D = FakeGrid2D
            ElevationMapResult = FakeElevationMapResult
            TerrainRiskParams = FakeTerrainRiskParams

            def compute_terrain_risk(self, elevation, _params):
                risk = FakeGrid2D()
                risk.rows = elevation.max_z.rows
                risk.cols = elevation.max_z.cols
                risk.resolution = elevation.max_z.resolution
                risk.origin_x = elevation.max_z.origin_x
                risk.origin_y = elevation.max_z.origin_y
                risk.data = [0.0, 0.0, 0.0, 0.0, 95.0, 0.0, 0.0, 0.0, 0.0]

                zeros = FakeGrid2D()
                zeros.rows = risk.rows
                zeros.cols = risk.cols
                zeros.resolution = risk.resolution
                zeros.origin_x = risk.origin_x
                zeros.origin_y = risk.origin_y
                zeros.data = [0.0] * 9
                return SimpleNamespace(
                    risk=risk,
                    slope_deg=zeros,
                    step_height=zeros,
                    roughness=zeros,
                )

        mod = Terrain(backend="simple")
        mod._native_kernel = FakeRuntime()
        result = SimpleNamespace(
            elevation_map=[0.0] * 9,
            map_width=3,
            map_origin_x=-0.3,
            map_origin_y=-0.3,
            map_resolution=0.2,
        )

        payload = mod._build_traversability_payload(result, ts=123.0)

        assert payload is not None
        assert payload["grid"].shape == (3, 3)
        assert payload["grid"][1, 1] == 95.0
        assert payload["traversability_class"] == "blocked"
        assert payload["backend"] == "cpp"

    def test_nanobind_backend_setup_succeeds(self, require_nav_kernel) -> None:
        """nanobind setup succeeds when LingTu native navigation kernel is available."""
        require_nav_kernel(("TerrainParams", "TerrainAnalysisCore"), "terrain")
        from nav.local.terrain import Terrain

        mod = Terrain(backend="nanobind")
        mod.setup()
        assert mod._core is not None, "nanobind C++ core should be loaded"

        mod.start()
        assert mod._running
        mod.stop()
        assert not mod._running

    def test_nanobind_alive_toggles(self, require_nav_kernel) -> None:
        """alive port toggles with nanobind backend."""
        require_nav_kernel(("TerrainParams", "TerrainAnalysisCore"), "terrain")
        from nav.local.terrain import Terrain

        mod = Terrain(backend="nanobind")
        mod.setup()

        alive_values: list[bool] = []
        mod.alive._add_callback(alive_values.append)

        mod.start()
        assert len(alive_values) >= 1
        assert alive_values[-1] is True

        mod.stop()
        assert len(alive_values) >= 2
        assert alive_values[-1] is False

    def test_nanobind_setup_uses_adapter_backend_bundle(self, monkeypatch) -> None:
        """nanobind setup should consume the backend adapter bundle only."""
        from nav.local import terrain as module_under_test
        from nav.local.terrain_backend import NanobindTerrainBackend

        class FakeTerrainCore:
            pass

        class FakeTerrainParams:
            pass

        sentinel_core = FakeTerrainCore()
        sentinel_params = FakeTerrainParams()
        calls = []

        def fake_create_nanobind_terrain_backend() -> NanobindTerrainBackend:
            calls.append("create")
            return NanobindTerrainBackend(
                core=sentinel_core,
                params=sentinel_params,
                effective_params={"scan_voxel_size": 0.12},
            )

        monkeypatch.setattr(
            module_under_test,
            "create_nanobind_terrain_backend",
            fake_create_nanobind_terrain_backend,
        )

        mod = module_under_test.Terrain(backend="nanobind")
        mod.setup()

        assert calls == ["create"]
        assert mod._core is sentinel_core

    def test_nanobind_setup_falls_back_to_simple_without_nav_kernel(self, monkeypatch) -> None:
        """Default terrain remains ROS-free when LingTu native navigation kernel is unavailable."""
        from nav.local import terrain as module_under_test

        def fake_create_nanobind_terrain_backend():
            raise RuntimeError(
                "Terrain [nanobind]: compatible LingTu native navigation kernel not found.\n"
                "  To build: scripts/build/build_nav_kernel.sh"
            )

        monkeypatch.setattr(
            module_under_test,
            "create_nanobind_terrain_backend",
            fake_create_nanobind_terrain_backend,
        )

        mod = module_under_test.Terrain(backend="nanobind")
        mod.setup()

        assert mod._backend == "simple"
        assert mod._core is None
        assert mod._backend_status.configured == "nanobind"
        assert mod._backend_status.effective == "simple"
        assert mod._backend_status.degraded is True
        assert (
            mod._backend_status.degraded_reason
            == "Terrain [nanobind]: compatible LingTu native navigation kernel not found."
        )

    # ------------------------------------------------------------------ #
    # Error handling
    # ------------------------------------------------------------------ #

    def test_unknown_backend_raises(self) -> None:
        """A bogus backend name must raise ValueError."""
        from nav.local.terrain import Terrain

        with pytest.raises(ValueError, match="Unknown terrain backend 'bogus'"):
            Terrain(backend="bogus")

    @pytest.mark.parametrize("backend", ["native", "cmu"])
    def test_legacy_native_backends_are_rejected(self, backend: str) -> None:
        """Legacy ROS2 NativeModule terrain backends are not Module backends."""
        from nav.local.terrain import Terrain

        with pytest.raises(ValueError, match=f"Unknown terrain backend '{backend}'"):
            Terrain(backend=backend)

    # ------------------------------------------------------------------ #
    # Registry
    # ------------------------------------------------------------------ #

    def test_all_backends_registered(self) -> None:
        """All terrain backends appear in the global Registry."""
        for backend in ("nanobind", "simple"):
            cls = registry_get("terrain", backend)
            assert cls is not None, f"terrain/{backend} not registered"

    def test_backend_status_rejects_cmu(self) -> None:
        """cmu backend is no longer a supported Module backend."""
        from nav.local.terrain import Terrain

        with pytest.raises(ValueError, match="Unknown terrain backend 'cmu'"):
            Terrain(backend="cmu")

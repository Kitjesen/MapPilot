"""Terrain - traversability analysis as a pluggable Module.

Analyzes point cloud + odometry to produce terrain maps for local planning.

Backends:
  "nanobind" - C++ terrain_core via nanobind (zero-copy, GIL-released, preferred)
  "simple"   - Python passthrough (testing, no analysis)

Usage::

    bp.add(Terrain, backend="nanobind")
    # In: odometry, map_cloud_frame/map_cloud -> Out: terrain_map, terrain_map_ext,
    # traversability, elevation_map
"""

from __future__ import annotations

import logging
import math
import time
from typing import Any

from nav.local.contracts import (
    TERRAIN_BACKENDS,
    require_terrain_backend,
)
from nav.local.terrain_backend import (
    create_nanobind_terrain_backend,
)
from runtime.backend_status import BackendStatus
from runtime.module import Module
from runtime.msgs.map import MapCloudFrame
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import PointCloud2
from runtime.registry import register
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.stream import In, Out

logger = logging.getLogger(__name__)


_AVAILABLE_TERRAIN_BACKENDS = TERRAIN_BACKENDS


@register("terrain", "nanobind", description="C++ terrain analysis via nanobind (zero-copy)")
@register("terrain", "simple", description="Passthrough for testing")
class Terrain(Module, layer=2):
    """Traversability analysis - ground estimation + obstacle detection.

    "nanobind": C++ TerrainAnalysisCore called directly from Python.
      Data flows through In/Out ports. Zero-copy numpy. GIL released during C++.
    "simple": Passthrough for testing.
    """

    runtime_id = "nav.terrain"

    # -- Inputs --
    odometry: In[Odometry]
    map_cloud_frame: In[MapCloudFrame]
    map_cloud: In[PointCloud2]

    # -- Outputs --
    terrain_map: Out[PointCloud2]
    terrain_map_ext: Out[PointCloud2]
    traversability: Out[dict]
    elevation_map: Out[np.ndarray]
    alive: Out[bool]

    def __init__(
        self,
        backend: str = "nanobind",
        terrain_ext_enabled: bool = True,
        terrain_ext_radius_m: float = 42.0,
        terrain_ext_voxel_size: float = 0.2,
        terrain_ext_max_points: int = 60000,
        **kw,
    ):
        super().__init__(**kw)
        require_terrain_backend(backend)
        self._backend_status = BackendStatus.configured_as(backend)
        self._backend = backend
        self._terrain_ext_enabled = bool(terrain_ext_enabled)
        self._terrain_ext_radius_m = max(0.0, float(terrain_ext_radius_m))
        self._terrain_ext_voxel_size = max(0.0, float(terrain_ext_voxel_size))
        self._terrain_ext_max_points = max(0, int(terrain_ext_max_points))
        self._core = None       # nanobind: TerrainAnalysisCore
        self._native_kernel = None
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_z = 0.0
        self._odom_yaw = 0.0

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.map_cloud.subscribe(self._on_cloud)
        self.map_cloud_frame.subscribe(self._on_cloud_frame)
        self.map_cloud.set_policy("latest")
        self.map_cloud_frame.set_policy("latest")

        if self._backend == "nanobind":
            self._setup_nanobind()
        else:
            logger.info("Terrain: simple backend (passthrough)")

    def _setup_nanobind(self):
        """Setup C++ terrain_core via nanobind binding."""
        try:
            backend = create_nanobind_terrain_backend()
        except RuntimeError as exc:
            reason = str(exc).splitlines()[0] or "nanobind terrain backend unavailable"
            logger.warning(
                "Terrain [nanobind]: %s; falling back to simple passthrough",
                exc,
            )
            self._backend_status.use("simple", reason=reason)
            self._backend = "simple"
            self._core = None
            return
        self._core = backend.core
        self._native_kernel = backend.runtime
        logger.info("Terrain [nanobind]: C++ terrain_core loaded")

    def start(self):
        super().start()
        self.alive.publish(True)
        logger.info("Terrain [%s]: started", self._backend)

    def stop(self):
        if self._core:
            self._core.clear()
            self._core = None
        self._native_kernel = None
        self.alive.publish(False)
        super().stop()

    def _on_odom(self, odom: Odometry):
        self._odom_x = odom.x
        self._odom_y = odom.y
        self._odom_z = getattr(odom, 'z', 0.0)
        self._odom_yaw = odom.yaw

        if self._core:
            # Feed odometry to C++ core (roll/pitch = 0 for ground robot)
            self._core.update_vehicle(
                odom.x, odom.y, self._odom_z,
                0.0, 0.0, odom.yaw)

        if self._backend == "simple":
            self.traversability.publish({"status": "passthrough"})

    def _on_cloud(self, cloud: PointCloud2):
        # GIL hot-path. Three layers of protection:
        # 1. Bypass entirely if env LINGTU_DISABLE_TERRAIN=1
        # 2. Throttle to 0.5 Hz (2 s interval)
        # 3. Cap point count in _process_nanobind below
        import os
        if os.environ.get("LINGTU_DISABLE_TERRAIN", "0") == "1":
            return
        now = time.time()
        if now - getattr(self, "_last_process_ts", 0.0) < 2.0:
            return
        self._last_process_ts = now

        if self._backend == "simple":
            self.terrain_map.publish(cloud)
            self._publish_terrain_ext(cloud)
            return

        if self._backend == "nanobind" and self._core is not None:
            self._publish_terrain_ext(cloud)
            self._process_nanobind(cloud)

    def _on_cloud_frame(self, frame: MapCloudFrame) -> None:
        self._on_cloud(frame.to_pointcloud2())

    def _publish_terrain_ext(self, cloud: PointCloud2) -> None:
        if not self._terrain_ext_enabled:
            return
        pts = getattr(cloud, "points", None)
        if pts is None:
            return
        pts = np.asarray(pts)
        if pts.ndim != 2 or pts.shape[0] == 0 or pts.shape[1] < 3:
            return

        xyz = pts[:, :3].astype(np.float32, copy=False)
        finite = np.isfinite(xyz).all(axis=1)
        dx = xyz[:, 0] - float(self._odom_x)
        dy = xyz[:, 1] - float(self._odom_y)
        dist = np.hypot(dx, dy)
        rel_z = xyz[:, 2] - float(self._odom_z)
        keep = (
            finite
            & (dist <= self._terrain_ext_radius_m)
            & (rel_z > (-2.5 - 0.1 * dist))
            & (rel_z < (1.0 + 0.1 * dist))
        )
        if not np.any(keep):
            return

        cols = min(pts.shape[1], 4)
        ext = np.ascontiguousarray(pts[keep, :cols], dtype=np.float32)
        ext_cloud = PointCloud2(
            points=ext,
            frame_id=topic_default_frame_id(TOPICS.terrain_map_ext),
            ts=getattr(cloud, "ts", time.time()),
        )
        # Keep this stateless; add a time-decay cache only if ext-map flicker hurts planning.
        if self._terrain_ext_voxel_size > 0.0:
            ext_cloud = ext_cloud.voxel_downsample(self._terrain_ext_voxel_size)
        if (
            self._terrain_ext_max_points > 0
            and ext_cloud.points.shape[0] > self._terrain_ext_max_points
        ):
            step = int(np.ceil(ext_cloud.points.shape[0] / self._terrain_ext_max_points))
            ext_cloud = PointCloud2(
                points=ext_cloud.points[::step][: self._terrain_ext_max_points].copy(),
                frame_id=ext_cloud.frame_id,
                ts=ext_cloud.ts,
            )
        self.terrain_map_ext.publish(ext_cloud)

    def _process_nanobind(self, cloud: PointCloud2):
        """Process point cloud through C++ terrain_runtime. GIL released during C++."""
        pts = cloud.points  # numpy Nx3 or Nx4
        if pts is None or len(pts) == 0:
            return

        # Ensure Nx4 float32 (x, y, z, intensity)
        if pts.ndim == 2 and pts.shape[1] == 3:
            pts4 = np.zeros((len(pts), 4), dtype=np.float32)
            pts4[:, :3] = pts
        elif pts.ndim == 2 and pts.shape[1] >= 4:
            pts4 = pts[:, :4].astype(np.float32, copy=False)
        else:
            return

        # W2-5: pass numpy array directly - no 30K truncation, no .tolist()
        # conversion. nanobind reads the buffer protocol, so this is close to
        # zero-copy without requiring the full nb::ndarray<float, ndim<1>>
        # binding refactor (follow-up C++ change).
        flat = np.ascontiguousarray(pts4, dtype=np.float32).ravel()
        ts = getattr(cloud, 'ts', time.time())

        result = self._core.process(flat, ts)

        traversability_payload: dict[str, Any] | None = None
        if result.map_width > 0:
            traversability_payload = self._build_traversability_payload(result, ts)

        # Publish terrain cloud
        if result.n_points > 0:
            arr = np.array(result.terrain_points, dtype=np.float32).reshape(-1, 4)
            self.terrain_map.publish(PointCloud2(
                points=arr,
                frame_id=topic_default_frame_id(TOPICS.terrain_map),
                ts=ts,
            ))
            if traversability_payload is None:
                traversability_payload = {}
            traversability_payload.update({
                "n_obstacles": result.n_points,
                "map_width": result.map_width,
                "map_resolution": result.map_resolution,
            })

        # Publish elevation map
        if result.map_width > 0:
            elev = np.array(result.elevation_map, dtype=np.float32).reshape(
                result.map_width, result.map_width)
            self.elevation_map.publish(elev)
            if traversability_payload is not None:
                self.traversability.publish(traversability_payload)

    def _build_traversability_payload(self, result: Any, ts: float) -> dict[str, Any] | None:
        """Build a 0..100 terrain risk grid from the C++ elevation output."""
        if self._native_kernel is None or result.map_width <= 0:
            return None
        from nav.services.map_layers.cpp_backend import (
            grid_to_array,
            make_grid2d,
            terrain_risk_result_to_payload,
        )

        elev = np.array(result.elevation_map, dtype=np.float32).reshape(
            result.map_width,
            result.map_width,
        )
        valid = np.isfinite(elev)
        if not np.any(valid):
            return None

        runtime = self._native_kernel
        elevation = runtime.ElevationMapResult()
        origin = [float(result.map_origin_x), float(result.map_origin_y)]
        resolution = float(result.map_resolution)
        elevation.min_z = make_grid2d(runtime, elev, resolution=resolution, origin=origin)
        elevation.max_z = make_grid2d(runtime, elev, resolution=resolution, origin=origin)
        elevation.clearance = make_grid2d(
            runtime,
            np.zeros_like(elev, dtype=np.float32),
            resolution=resolution,
            origin=origin,
        )
        elevation.valid = valid.astype(np.uint8).ravel().tolist()

        risk_params = runtime.TerrainRiskParams()
        risk = runtime.compute_terrain_risk(elevation, risk_params)
        payload = terrain_risk_result_to_payload(
            risk,
            ts=ts,
            frame_id=topic_default_frame_id(TOPICS.terrain_map),
        )
        grid = grid_to_array(risk.risk)
        max_risk = float(np.nanmax(grid)) if grid.size else 0.0
        mean_risk = float(np.nanmean(grid)) if grid.size else 0.0
        payload.update({
            "traversability_class": self._classify_risk(max_risk, mean_risk),
            "risk_max": max_risk,
            "risk_mean": mean_risk,
        })
        return payload

    @staticmethod
    def _classify_risk(max_risk: float, mean_risk: float) -> str:
        if not (math.isfinite(max_risk) and math.isfinite(mean_risk)):
            return "unknown"
        if max_risk >= 90.0:
            return "blocked"
        if mean_risk >= 45.0:
            return "rough"
        if mean_risk >= 20.0:
            return "caution"
        return "clear"

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["terrain"] = {
            **self._backend_status.as_health_fields(),
            "has_core": self._core is not None,
            "nodes": {},
        }
        return info

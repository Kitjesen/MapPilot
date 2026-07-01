"""ElevationMapModule - real-time elevation map from LiDAR.

Subscribes to the canonical SLAM map cloud stream.
Bins 3D points by XY cell and computes per-cell height statistics:

  min_z     - lowest point per cell (ground surface estimate)
  max_z     - highest point per cell (obstacle or ceiling top)
  clearance - max_z - min_z (free vertical space)
  valid     - bool mask (True when at least one point hit this cell)

Ports:
  In:  map_cloud_frame (MapCloudFrame), map_cloud (PointCloud2), odometry (Odometry)
  Out: elevation_map (dict)
"""

from __future__ import annotations

import logging
import time
from typing import Any

from nav.services.map_layers.cpp_backend import (
    create_map_kernel_backend,
    elevation_result_to_payload,
)
from runtime.module import Module
from runtime.msgs.map import MapCloudFrame
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import PointCloud2
from runtime.registry import register
from runtime.runtime_interface import TOPICS, normalize_frame_id, topic_default_frame_id
from runtime.stream import In, Out

logger = logging.getLogger(__name__)


@register("map", "elevation", description="Real-time elevation map from LiDAR point cloud")
class ElevationMapModule(Module, layer=2):
    """Per-cell min/max height from LiDAR in a robot-centric sliding window."""

    map_cloud_frame: In[MapCloudFrame]
    map_cloud: In[PointCloud2]
    odometry: In[Odometry]

    elevation_map: Out[dict]

    def __init__(
        self,
        resolution: float = 0.2,
        map_radius: float = 15.0,
        z_floor: float = -0.50,
        z_ceil: float = 3.00,
        publish_hz: float = 2.0,
        frame_id: str | None = None,
        **kw,
    ):
        super().__init__(**kw)
        self._res = resolution
        self._radius = map_radius
        self._z_floor = z_floor
        self._z_ceil = z_ceil
        self._interval = 1.0 / publish_hz
        self._robot_xy = [0.0, 0.0]
        self._gs = int(2 * map_radius / resolution)
        self._default_frame_id = (
            normalize_frame_id(frame_id) or topic_default_frame_id(TOPICS.map_cloud)
        )
        self._map_kernel = None

    def setup(self) -> None:
        self._map_kernel = create_map_kernel_backend()
        self.map_cloud.subscribe(self._on_cloud)
        self.map_cloud_frame.subscribe(self._on_cloud_frame)
        self.odometry.subscribe(self._on_odom)
        self.map_cloud.set_policy("throttle", interval=self._interval)
        self.map_cloud_frame.set_policy("throttle", interval=self._interval)

    def _on_odom(self, odom: Odometry) -> None:
        self._robot_xy[0] = odom.x
        self._robot_xy[1] = odom.y

    def _on_cloud(self, cloud: PointCloud2) -> None:
        if cloud.is_empty:
            return
        pts = cloud.points[:, :3]

        mask = (pts[:, 2] > self._z_floor) & (pts[:, 2] < self._z_ceil)
        pts = pts[mask]
        if pts.shape[0] == 0:
            return
        frame_id = (
            normalize_frame_id(getattr(cloud, "frame_id", None))
            or self._default_frame_id
        )
        now = time.time()

        if self._map_kernel is not None:
            result = self._map_kernel.runtime.build_elevation_map(
                np.ascontiguousarray(pts[:, :3], dtype=np.float32).ravel().tolist(),
                float(self._robot_xy[0]),
                float(self._robot_xy[1]),
                float(self._res),
                float(self._radius),
                float(self._z_floor),
                float(self._z_ceil),
            )
            self.elevation_map.publish(
                elevation_result_to_payload(result, ts=now, frame_id=frame_id)
            )
            return

        origin_xy = np.asarray(self._robot_xy, dtype=np.float64) - self._radius
        gs = self._gs

        ix = np.floor((pts[:, 0] - origin_xy[0]) / self._res).astype(np.int32)
        iy = np.floor((pts[:, 1] - origin_xy[1]) / self._res).astype(np.int32)
        valid = (ix >= 0) & (ix < gs) & (iy >= 0) & (iy < gs)
        ix, iy, z = ix[valid], iy[valid], pts[valid, 2].astype(np.float32)

        n_cells = gs * gs
        flat = iy * gs + ix

        min_flat = np.full(n_cells, np.inf, dtype=np.float32)
        max_flat = np.full(n_cells, -np.inf, dtype=np.float32)
        np.minimum.at(min_flat, flat, z)
        np.maximum.at(max_flat, flat, z)

        hit = min_flat != np.inf
        min_flat = np.where(hit, min_flat, np.nan)
        max_flat = np.where(hit, max_flat, np.nan)

        min_z = min_flat.reshape(gs, gs)
        max_z = max_flat.reshape(gs, gs)
        valid_mask = hit.reshape(gs, gs)
        clearance = np.where(valid_mask, max_z - min_z, np.nan)

        self.elevation_map.publish(
            {
                "min_z": min_z,
                "max_z": max_z,
                "clearance": clearance,
                "valid": valid_mask,
                "resolution": self._res,
                "origin": origin_xy.tolist(),
                "ts": now,
                "frame_id": frame_id,
                "backend": "numpy",
            }
        )

    def _on_cloud_frame(self, frame: MapCloudFrame) -> None:
        self._on_cloud(frame.to_pointcloud2())

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["elevation_map"] = {
            "resolution": self._res,
            "map_radius": self._radius,
            "grid_size": self._gs,
            "z_range": [self._z_floor, self._z_ceil],
            "default_frame_id": self._default_frame_id,
            "backend": "nav_kernel" if self._map_kernel is not None else "numpy",
        }
        return info

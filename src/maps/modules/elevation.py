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

from maps.adapters.python.kernels import (
    create_map_kernel_backend,
    elevation_result_to_payload,
)
from runtime.module import Module
from runtime.msgs.map import MapObservationFrame
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

    map_observation: In[MapObservationFrame]
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
        self._default_frame_id = normalize_frame_id(frame_id) or topic_default_frame_id(TOPICS.map_cloud)
        self._map_kernel = None
        self._last_observation_epoch = 0
        self._last_observation_sequence = 0

    def setup(self) -> None:
        self._map_kernel = create_map_kernel_backend()
        if self._map_kernel is None:
            raise RuntimeError("ElevationMapModule requires the native map kernel")
        self.map_cloud.subscribe(self._on_cloud)
        self.map_observation.subscribe(self._on_observation)
        self.odometry.subscribe(self._on_odom)
        self.map_cloud.set_policy("throttle", interval=self._interval)
        self.map_observation.set_policy("throttle", interval=self._interval)

    def _on_odom(self, odom: Odometry) -> None:
        self._robot_xy[0] = odom.x
        self._robot_xy[1] = odom.y

    def _on_cloud(self, cloud: PointCloud2) -> None:
        if cloud.is_empty:
            return
        frame_id = normalize_frame_id(getattr(cloud, "frame_id", None)) or self._default_frame_id
        self._build_from_points(
            cloud.points,
            robot_x=float(self._robot_xy[0]),
            robot_y=float(self._robot_xy[1]),
            frame_id=frame_id,
            ts=float(getattr(cloud, "ts", 0.0) or time.time()),
        )

    def _on_observation(self, frame: MapObservationFrame) -> None:
        epoch = int(frame.reset_epoch)
        sequence = int(frame.sequence)
        if epoch < self._last_observation_epoch or (
            epoch == self._last_observation_epoch
            and sequence <= self._last_observation_sequence
        ):
            return
        self._last_observation_epoch = epoch
        self._last_observation_sequence = sequence
        self._build_from_points(
            frame.map_points(),
            robot_x=float(frame.sensor_origin.x),
            robot_y=float(frame.sensor_origin.y),
            frame_id=frame.frame_id,
            ts=float(frame.ts),
        )

    def _build_from_points(
        self,
        points: Any,
        *,
        robot_x: float,
        robot_y: float,
        frame_id: str,
        ts: float,
    ) -> None:
        pts = points[:, :3]
        mask = (pts[:, 2] > self._z_floor) & (pts[:, 2] < self._z_ceil)
        pts = pts[mask]
        if pts.shape[0] == 0:
            return

        result = self._map_kernel.runtime.build_elevation_map(
            np.ascontiguousarray(pts[:, :3], dtype=np.float32).ravel().tolist(),
            robot_x,
            robot_y,
            float(self._res),
            float(self._radius),
            float(self._z_floor),
            float(self._z_ceil),
        )
        self.elevation_map.publish(
            elevation_result_to_payload(result, ts=ts, frame_id=frame_id)
        )

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["elevation_map"] = {
            "resolution": self._res,
            "map_radius": self._radius,
            "grid_size": self._gs,
            "z_range": [self._z_floor, self._z_ceil],
            "default_frame_id": self._default_frame_id,
            "backend": "cpp",
        }
        return info

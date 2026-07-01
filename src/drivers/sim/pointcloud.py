"""SimPointCloudProvider generates PointCloud2 from MuJoCo XML scene geometry.

Reads box geoms from a MuJoCo XML file, samples 3-D points on obstacle
surfaces within the LiDAR height band, and publishes a static PointCloud2
at a configurable rate.  This feeds OccupancyGridModule identically to
real SLAM output, exercising the full navigation pipeline in pure Python.

Ports:
  In:  odometry (Odometry) - robot position for periodic re-publish trigger
  Out: map_cloud (PointCloud2) - static obstacle point cloud
"""

from __future__ import annotations

import logging
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from runtime.module import Module
from runtime.msgs.numpy_compat import np
from runtime.msgs.nav import Odometry
from runtime.msgs.sensor import PointCloud2
from runtime.registry import register
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.stream import In, Out
from drivers.sim.mujoco.scene import (
    build_parent_map,
    geom_name_chain,
    geom_world_pose,
)

logger = logging.getLogger(__name__)

SIM_POINTCLOUD_MAP_FRAME_ID = topic_default_frame_id(TOPICS.map_cloud)


@register("sim_lidar", "pointcloud", description="MuJoCo XML static point-cloud provider")
class SimPointCloudProvider(Module, layer=1):
    """Parse MuJoCo XML scene and publish obstacle geometry as PointCloud2."""

    odometry: In[Odometry]
    map_cloud: Out[PointCloud2]

    def __init__(
        self,
        scene_xml: str = "",
        z_min: float = 0.05,
        z_max: float = 2.50,
        sample_spacing: float = 0.10,
        publish_hz: float = 2.0,
        exclude_names: list[str] | None = None,
        **kw,
    ):
        super().__init__(**kw)
        self._scene_xml = scene_xml
        self._z_min = z_min
        self._z_max = z_max
        self._spacing = sample_spacing
        self._interval = 1.0 / publish_hz
        self._exclude = set(exclude_names or ["floor", "robot"])
        self._cloud: PointCloud2 | None = None
        self._last_pub = 0.0

    def setup(self) -> None:
        if not self._scene_xml:
            logger.warning("No scene_xml configured; SimPointCloudProvider idle")
            return
        path = Path(self._scene_xml)
        if not path.exists():
            logger.error("Scene XML not found: %s", path)
            return
        points = self._parse_scene(path)
        if len(points) == 0:
            logger.warning("No obstacle points extracted from %s", path)
            return
        self._cloud = PointCloud2(points=points, frame_id=SIM_POINTCLOUD_MAP_FRAME_ID)
        logger.info(
            "SimPointCloudProvider: %d points from %d geoms in %s",
            len(points), self._geom_count, path.name,
        )
        self.odometry.subscribe(self._on_odom)

    def start(self) -> None:
        super().start()
        # Bootstrap: publish initial cloud so OccupancyGridModule can build
        # a costmap before any cmd_vel to odometry loop starts.
        if self._cloud is not None:
            self._cloud.ts = time.time()
            self.map_cloud.publish(self._cloud)
            self._last_pub = time.time()
            logger.info("SimPointCloudProvider: initial cloud published")

    def _on_odom(self, odom: Odometry) -> None:
        now = time.time()
        if self._cloud is not None and (now - self._last_pub) >= self._interval:
            self._cloud.ts = now
            self.map_cloud.publish(self._cloud)
            self._last_pub = now

    # ------------------------------------------------------------------
    # XML parsing + point sampling
    # ------------------------------------------------------------------

    def _parse_scene(self, xml_path: Path) -> np.ndarray:
        tree = ET.parse(xml_path)
        root = tree.getroot()
        parent_map = build_parent_map(root)
        all_points: list[np.ndarray] = []
        self._geom_count = 0

        for geom in root.iter("geom"):
            gtype = geom.get("type", "sphere")
            if gtype != "box":
                continue
            names = geom_name_chain(geom, parent_map)
            lowered_names = [name.lower() for name in names]
            if any(
                exc in name
                for name in lowered_names
                for exc in self._exclude
            ):
                continue
            size_str = geom.get("size")
            if not size_str:
                continue

            size = np.array([float(x) for x in size_str.split()], dtype=np.float64)
            if len(size) != 3:
                continue

            pos_raw, rotation_raw = geom_world_pose(geom, parent_map)
            pos = np.array(pos_raw, dtype=np.float64)
            rotation = np.array(rotation_raw, dtype=np.float64)

            corners = self._box_corners(pos, size, rotation)
            z_lo = float(corners[:, 2].min())
            z_hi = float(corners[:, 2].max())

            # skip if entirely outside the height band
            if z_hi < self._z_min or z_lo > self._z_max:
                continue

            pts = self._sample_box_perimeter(pos, size, rotation)
            if len(pts) > 0:
                all_points.append(pts)
                self._geom_count += 1

        if not all_points:
            return np.zeros((0, 3), dtype=np.float32)
        return np.vstack(all_points).astype(np.float32)

    def _box_corners(
        self,
        center: np.ndarray,
        half: np.ndarray,
        rotation: np.ndarray,
    ) -> np.ndarray:
        local = np.array(
            [
                [sx * half[0], sy * half[1], sz * half[2]]
                for sx in (-1.0, 1.0)
                for sy in (-1.0, 1.0)
                for sz in (-1.0, 1.0)
            ],
            dtype=np.float64,
        )
        return center + local @ rotation.T

    def _sample_box_perimeter(
        self,
        center: np.ndarray,
        half: np.ndarray,
        rotation: np.ndarray,
    ) -> np.ndarray:
        """Sample 3-D points on the 4 vertical faces of a box."""
        hx, hy, hz = half
        spacing = self._spacing

        z_bounds = self._box_corners(center, half, rotation)[:, 2]
        z_lo = max(float(z_bounds.min()), self._z_min)
        z_hi = min(float(z_bounds.max()), self._z_max)
        z_val = (z_lo + z_hi) / 2.0

        # sample along the 4 edges of the XY rectangle
        points = []

        # bottom/top edges (along X)
        xs = np.arange(-hx, hx + spacing * 0.5, spacing)
        for y_edge in [-hy, hy]:
            local = np.column_stack(
                [
                    xs,
                    np.full(len(xs), y_edge),
                    np.full(len(xs), z_val - center[2]),
                ]
            )
            pts = center + local @ rotation.T
            points.append(pts)

        # left/right edges (along Y)
        ys = np.arange(-hy, hy + spacing * 0.5, spacing)
        for x_edge in [-hx, hx]:
            local = np.column_stack(
                [
                    np.full(len(ys), x_edge),
                    ys,
                    np.full(len(ys), z_val - center[2]),
                ]
            )
            pts = center + local @ rotation.T
            points.append(pts)

        if not points:
            return np.zeros((0, 3), dtype=np.float32)
        sampled = np.vstack(points)
        return sampled[
            (sampled[:, 2] >= self._z_min)
            & (sampled[:, 2] <= self._z_max)
        ]

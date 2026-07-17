"""OccupancyGridModule: native 2D occupancy and cost grids from LiDAR.

Python owns runtime ports and message packing only. Projection, raycast,
inflation, robot footprint clearing, and count generation run inside
``lingtu_maps``.
"""

from __future__ import annotations

import logging
import math
import time
from typing import Any

from maps.adapters.python.kernels import create_map_kernel_backend
from runtime.module import Module
from runtime.msgs.geometry import Pose, Quaternion, Vector3
from runtime.msgs.map import MapObservationFrame
from runtime.msgs.nav import OccupancyGrid, Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import PointCloud2
from runtime.registry import register
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.stream import In, Out

logger = logging.getLogger(__name__)

UNKNOWN = -1
FREE = 0
OCCUPIED = 100


@register("map", "occupancy_grid", description="Native 2D occupancy grid from LiDAR point cloud")
class OccupancyGridModule(Module, layer=2):
    """Build robot-centric local occupancy grids through ``lingtu_maps``."""

    map_observation: In[MapObservationFrame]
    map_cloud: In[PointCloud2]
    odometry: In[Odometry]

    occupancy_grid: Out[OccupancyGrid]
    costmap: Out[dict]
    exploration_grid: Out[dict]

    def __init__(
        self,
        resolution: float = 0.2,
        map_radius: float = 30.0,
        z_min: float = 0.30,
        z_max: float = 2.00,
        inflation_radius: float = 0.25,
        robot_clear_radius: float = 0.60,
        robot_clear_forward: float = 0.0,
        robot_clear_backward: float = 0.0,
        robot_clear_lateral: float = 0.0,
        publish_hz: float = 2.0,
        frame_id: str | None = None,
        raycast_free_space: bool = False,
        unknown_as_obstacle_for_costmap: bool = False,
        raycast_max_rays: int = 1800,
        raycast_free_inflation_radius: float = 0.0,
        **kw: Any,
    ):
        super().__init__(**kw)
        self._res = float(resolution)
        self._radius = float(map_radius)
        self._z_min = float(z_min)
        self._z_max = float(z_max)
        self._inf_radius = float(inflation_radius)
        self._robot_clear_radius = float(robot_clear_radius)
        self._robot_clear_forward = max(0.0, float(robot_clear_forward))
        self._robot_clear_backward = max(0.0, float(robot_clear_backward))
        self._robot_clear_lateral = max(0.0, float(robot_clear_lateral))
        self._interval = 1.0 / max(float(publish_hz), 1e-6)
        self._frame_id = str(frame_id or topic_default_frame_id(TOPICS.exploration_grid))
        self._raycast_free_space = bool(raycast_free_space)
        self._unknown_as_obstacle_for_costmap = bool(unknown_as_obstacle_for_costmap)
        self._raycast_max_rays = max(1, int(raycast_max_rays))
        self._raycast_free_inflation_radius = max(0.0, float(raycast_free_inflation_radius))
        self._robot_xy = [0.0, 0.0]
        self._robot_yaw = 0.0
        self._gs = int(2 * self._radius / self._res)
        self._map_kernel = None

    def setup(self) -> None:
        self._map_kernel = create_map_kernel_backend()
        if self._map_kernel is None:
            raise RuntimeError("OccupancyGridModule requires the native lingtu_maps library")
        self.map_cloud.subscribe(self._on_cloud)
        self.map_observation.subscribe(self._on_observation)
        self.odometry.subscribe(self._on_odom)
        self.map_cloud.set_policy("throttle", interval=self._interval)
        self.map_observation.set_policy("throttle", interval=self._interval)

    def _on_odom(self, odom: Odometry) -> None:
        self._robot_xy[0] = odom.x
        self._robot_xy[1] = odom.y
        if math.isfinite(float(odom.yaw)):
            self._robot_yaw = float(odom.yaw)

    def _on_cloud(self, cloud: PointCloud2) -> None:
        if cloud.is_empty:
            return
        pts = np.asarray(cloud.points[:, :3], dtype=np.float32)
        valid = np.isfinite(pts).all(axis=1)
        pts = np.ascontiguousarray(pts[valid], dtype=np.float32)
        if pts.shape[0] == 0:
            return
        if self._map_kernel is None:
            raise RuntimeError("OccupancyGridModule native backend is not initialized")

        result = self._map_kernel.runtime.build_occupancy_grid(
            pts,
            robot_x=float(self._robot_xy[0]),
            robot_y=float(self._robot_xy[1]),
            robot_yaw=float(self._robot_yaw),
            resolution=float(self._res),
            radius=float(self._radius),
            z_min=float(self._z_min),
            z_max=float(self._z_max),
            inflation_radius=float(self._inf_radius),
            robot_clear_radius=float(self._robot_clear_radius),
            robot_clear_forward=float(self._robot_clear_forward),
            robot_clear_backward=float(self._robot_clear_backward),
            robot_clear_lateral=float(self._robot_clear_lateral),
            raycast_free_space=bool(self._raycast_free_space),
            unknown_as_obstacle_for_costmap=bool(self._unknown_as_obstacle_for_costmap),
            raycast_max_rays=int(self._raycast_max_rays),
            raycast_free_inflation_radius=float(self._raycast_free_inflation_radius),
        )
        counts = dict(result["counts"])
        if not self._raycast_free_space and int(counts.get("occupied", 0)) <= 0:
            return
        self._publish_grids(
            occupancy_grid=np.asarray(result["occupancy"], dtype=np.int8),
            nav_cost=np.asarray(result["cost"], dtype=np.float32),
            origin_xy=np.asarray(result["origin"], dtype=np.float64),
            source="raycast_lidar" if self._raycast_free_space else "projected_lidar",
            counts=counts,
            raycast=self._raycast_free_space,
        )

    def _on_observation(self, frame: MapObservationFrame) -> None:
        self._on_cloud(frame.to_map_pointcloud2())

    def _publish_grids(
        self,
        *,
        occupancy_grid: np.ndarray,
        nav_cost: np.ndarray,
        origin_xy: np.ndarray,
        source: str,
        counts: dict[str, int],
        raycast: bool = False,
    ) -> None:
        origin_pose = Pose(
            position=Vector3(float(origin_xy[0]), float(origin_xy[1]), 0.0),
            orientation=Quaternion(0, 0, 0, 1),
        )
        now = time.time()
        og = OccupancyGrid(
            grid=occupancy_grid,
            resolution=self._res,
            origin=origin_pose,
            ts=now,
            frame_id=self._frame_id,
        )
        common = {
            "resolution": self._res,
            "origin": origin_xy.tolist(),
            "origin_x": float(origin_xy[0]),
            "origin_y": float(origin_xy[1]),
            "height": int(occupancy_grid.shape[0]),
            "width": int(occupancy_grid.shape[1]),
            "ts": now,
            "frame_id": self._frame_id,
            "raycast": bool(raycast),
            "counts": counts,
            "accumulation": "rolling_local_window",
            "semantic": "local_planning_grid",
            "backend": "cpp",
        }
        self.occupancy_grid.publish(og)
        self.costmap.publish(
            {
                **common,
                "grid": nav_cost,
                "source": f"{source}_navigation_costmap",
                "unknown_as_obstacle": self._unknown_as_obstacle_for_costmap,
            }
        )
        self.exploration_grid.publish(
            {
                **common,
                "grid": occupancy_grid.astype(np.int16),
                "source": f"{source}_exploration_grid",
                "semantic": "frontier_input_grid",
            }
        )

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["occupancy_grid"] = {
            "resolution": self._res,
            "map_radius": self._radius,
            "grid_size": self._gs,
            "z_range": [self._z_min, self._z_max],
            "inflation_m": self._inf_radius,
            "robot_clear_m": self._robot_clear_radius,
            "robot_clear_forward_m": self._robot_clear_forward,
            "robot_clear_backward_m": self._robot_clear_backward,
            "robot_clear_lateral_m": self._robot_clear_lateral,
            "frame_id": self._frame_id,
            "raycast_free_space": self._raycast_free_space,
            "unknown_as_obstacle_for_costmap": self._unknown_as_obstacle_for_costmap,
            "backend": "cpp",
        }
        return info

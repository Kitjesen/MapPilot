"""OccupancyGridModule: real-time 2D occupancy and cost grids from LiDAR.

The module has two operating modes:

* Projection mode, the default, preserves the historical behavior: project
  obstacle-height points into a robot-centric costmap.
* Raycast mode marks unknown, free, and occupied cells from scan endpoints.
  This is the mode required by frontier exploration because frontiers only
  exist at the boundary between known free space and unknown space.

Ports:
  In:  map_cloud (PointCloud2), odometry (Odometry)
  Out: occupancy_grid (OccupancyGrid), costmap (dict), exploration_grid (dict)
"""

from __future__ import annotations

import logging
import time
from typing import Any

from core.module import Module
from core.msgs.geometry import Pose, Quaternion, Vector3
from core.msgs.nav import OccupancyGrid, Odometry
from core.msgs.numpy_compat import np
from core.msgs.sensor import PointCloud2
from core.registry import register
from core.runtime_interface import TOPICS, topic_default_frame_id
from core.stream import In, Out

logger = logging.getLogger(__name__)

UNKNOWN = -1
FREE = 0
OCCUPIED = 100
_OCCUPIED_COST = 100.0  # nav_cost value for occupied cells (matches OCCUPIED=100, float for arithmetic)


@register("map", "occupancy_grid", description="2D occupancy grid from LiDAR point cloud")
class OccupancyGridModule(Module, layer=2):
    """Project or raycast LiDAR clouds into local 2D planning grids."""

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
        self._interval = 1.0 / max(float(publish_hz), 1e-6)
        self._frame_id = str(frame_id or topic_default_frame_id(TOPICS.exploration_grid))
        self._raycast_free_space = bool(raycast_free_space)
        self._unknown_as_obstacle_for_costmap = bool(unknown_as_obstacle_for_costmap)
        self._raycast_max_rays = max(1, int(raycast_max_rays))
        self._raycast_free_inflation_radius = max(
            0.0,
            float(raycast_free_inflation_radius),
        )
        self._robot_xy = [0.0, 0.0]
        self._gs = int(2 * self._radius / self._res)
        self._kernel: np.ndarray | None = None
        self._free_kernel: np.ndarray | None = None
        self._robot_clear_radius_sq = self._robot_clear_radius * self._robot_clear_radius
        self._clear_mask_cache_key: tuple[int, int, int, int] | None = None
        self._clear_mask_cache: np.ndarray | None = None

    def setup(self) -> None:
        self.map_cloud.subscribe(self._on_cloud)
        self.odometry.subscribe(self._on_odom)
        self.map_cloud.set_policy("throttle", interval=self._interval)

    def _ensure_kernels(self) -> None:
        if self._kernel is None:
            self._kernel = self._make_circle_kernel(self._inf_radius, self._res)
        if (
            self._raycast_free_inflation_radius > 0.0
            and self._free_kernel is None
        ):
            self._free_kernel = self._make_circle_kernel(
                self._raycast_free_inflation_radius,
                self._res,
            )

    def _on_odom(self, odom: Odometry) -> None:
        self._robot_xy[0] = odom.x
        self._robot_xy[1] = odom.y

    def _on_cloud(self, cloud: PointCloud2) -> None:
        if cloud.is_empty:
            return
        pts = np.asarray(cloud.points[:, :3], dtype=np.float32)
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]
        if pts.shape[0] == 0:
            return

        if self._raycast_free_space:
            self._publish_raycast_grid(pts)
        else:
            self._publish_projected_grid(pts)

    def _publish_projected_grid(self, pts: np.ndarray) -> None:
        mask = (pts[:, 2] > self._z_min) & (pts[:, 2] < self._z_max)
        pts2d = pts[mask, :2]
        if pts2d.shape[0] == 0:
            return

        origin_xy = self._robot_xy - self._radius
        gs = self._gs
        pts2d = self._filter_robot_footprint(pts2d)
        if pts2d.shape[0] == 0:
            return

        ix = np.floor((pts2d[:, 0] - origin_xy[0]) / self._res).astype(np.int32)
        iy = np.floor((pts2d[:, 1] - origin_xy[1]) / self._res).astype(np.int32)
        valid = (ix >= 0) & (ix < gs) & (iy >= 0) & (iy < gs)
        ix, iy = ix[valid], iy[valid]
        if ix.size == 0:
            return

        binary = np.zeros((gs, gs), dtype=np.bool_)
        binary[iy, ix] = True
        clear_mask = self._robot_clear_mask(origin_xy)
        binary[clear_mask] = False

        inflated = self._inflate(binary)
        cost = (inflated.astype(np.float32) * _OCCUPIED_COST).clip(0, _OCCUPIED_COST)
        cost[binary] = _OCCUPIED_COST
        cost[clear_mask] = 0.0
        grid_int8 = cost.astype(np.int8)
        self._publish_grids(
            occupancy_grid=grid_int8,
            nav_cost=cost,
            origin_xy=origin_xy,
            source="projected_lidar",
            counts={
                "unknown": 0,
                "free": int((grid_int8 == 0).sum()),
                "occupied": int((grid_int8 >= OCCUPIED).sum()),
            },
        )

    def _publish_raycast_grid(self, pts: np.ndarray) -> None:
        origin_xy = self._robot_xy - self._radius
        gs = self._gs
        robot_col = int(np.floor((self._robot_xy[0] - origin_xy[0]) / self._res))
        robot_row = int(np.floor((self._robot_xy[1] - origin_xy[1]) / self._res))
        if not (0 <= robot_row < gs and 0 <= robot_col < gs):
            return

        grid = np.full((gs, gs), UNKNOWN, dtype=np.int16)
        clear_mask = self._robot_clear_mask(origin_xy)
        grid[clear_mask] = FREE

        ray_pts = self._downsample_points(pts, self._raycast_max_rays)
        cols = np.floor((ray_pts[:, 0] - origin_xy[0]) / self._res).astype(np.int32)
        rows = np.floor((ray_pts[:, 1] - origin_xy[1]) / self._res).astype(np.int32)
        in_bounds = (cols >= 0) & (cols < gs) & (rows >= 0) & (rows < gs)
        for col, row in zip(cols[in_bounds].tolist(), rows[in_bounds].tolist()):
            cells = self._bresenham(robot_col, robot_row, int(col), int(row))
            for c, r in cells[:-1]:
                if 0 <= r < gs and 0 <= c < gs:
                    grid[r, c] = FREE

        self._ensure_kernels()
        if self._free_kernel is not None:
            widened_free = self._dilate(grid == FREE, self._free_kernel).astype(bool)
            grid[(grid == UNKNOWN) & widened_free] = FREE

        occ_mask = (pts[:, 2] > self._z_min) & (pts[:, 2] < self._z_max)
        occ_pts = pts[occ_mask]
        if occ_pts.shape[0] > 0:
            occ_pts = self._filter_robot_footprint(occ_pts[:, :2], z_source=occ_pts)
        occ_pts = self._downsample_points(occ_pts, self._raycast_max_rays)
        if occ_pts.shape[0] > 0:
            occ_cols = np.floor((occ_pts[:, 0] - origin_xy[0]) / self._res).astype(np.int32)
            occ_rows = np.floor((occ_pts[:, 1] - origin_xy[1]) / self._res).astype(np.int32)
            occ_in_bounds = (
                (occ_cols >= 0) & (occ_cols < gs) & (occ_rows >= 0) & (occ_rows < gs)
            )
            grid[occ_rows[occ_in_bounds], occ_cols[occ_in_bounds]] = OCCUPIED

        occupied = grid == OCCUPIED
        if np.any(occupied):
            inflated = self._inflate(occupied).astype(bool)
            grid[inflated] = OCCUPIED
        grid[clear_mask] = FREE

        nav_cost = grid.astype(np.float32)
        if self._unknown_as_obstacle_for_costmap:
            nav_cost[nav_cost < 0] = _OCCUPIED_COST
        else:
            nav_cost[nav_cost < 0] = 0.0
        nav_cost = nav_cost.clip(0, _OCCUPIED_COST)

        self._publish_grids(
            occupancy_grid=grid.astype(np.int8),
            nav_cost=nav_cost,
            origin_xy=origin_xy,
            source="raycast_lidar",
            counts={
                "unknown": int((grid < 0).sum()),
                "free": int((grid == FREE).sum()),
                "occupied": int((grid >= OCCUPIED).sum()),
            },
            raycast=True,
        )

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

    def _filter_robot_footprint(
        self,
        pts2d: np.ndarray,
        *,
        z_source: np.ndarray | None = None,
    ) -> np.ndarray:
        dx = pts2d[:, 0] - self._robot_xy[0]
        dy = pts2d[:, 1] - self._robot_xy[1]
        far = (dx * dx + dy * dy) >= self._robot_clear_radius_sq
        if z_source is not None:
            return z_source[far]
        return pts2d[far]

    @staticmethod
    def _downsample_points(points: np.ndarray, max_points: int) -> np.ndarray:
        if points.shape[0] <= max_points:
            return points
        step = max(1, int(np.ceil(points.shape[0] / max_points)))
        return points[::step]

    def _inflate(self, binary: np.ndarray) -> np.ndarray:
        """Binary dilation with a pre-built circular kernel."""
        self._ensure_kernels()
        return self._dilate(binary, self._kernel)

    @staticmethod
    def _dilate(binary: np.ndarray, kernel: np.ndarray | None) -> np.ndarray:
        if kernel is None:
            return binary.astype(np.float32)
        try:
            from scipy.ndimage import binary_dilation

            return binary_dilation(binary, structure=kernel).astype(np.float32)
        except ImportError:
            from numpy.lib.stride_tricks import sliding_window_view

            k = int(kernel.shape[0])
            pad = k // 2
            padded = np.pad(binary.astype(np.float32), pad, mode="constant")
            windows = sliding_window_view(padded, (k, k))
            return windows.max(axis=(-2, -1))

    def _robot_clear_mask(self, origin_xy: np.ndarray) -> np.ndarray:
        gs = self._gs
        rx = int(np.floor((self._robot_xy[0] - origin_xy[0]) / self._res))
        ry = int(np.floor((self._robot_xy[1] - origin_xy[1]) / self._res))
        r = max(1, int(np.ceil(self._robot_clear_radius / self._res)))
        key = (gs, rx, ry, r)
        if self._clear_mask_cache_key == key and self._clear_mask_cache is not None:
            return self._clear_mask_cache

        yy, xx = np.ogrid[:gs, :gs]
        self._clear_mask_cache = (xx - rx) ** 2 + (yy - ry) ** 2 <= r**2
        self._clear_mask_cache_key = key
        return self._clear_mask_cache

    @staticmethod
    def _make_circle_kernel(radius_m: float, res: float) -> np.ndarray:
        r = max(1, int(np.ceil(radius_m / res)))
        y, x = np.ogrid[-r : r + 1, -r : r + 1]
        return (x**2 + y**2) <= r**2

    @staticmethod
    def _bresenham(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
        cells: list[tuple[int, int]] = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
        return cells

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["occupancy_grid"] = {
            "resolution": self._res,
            "map_radius": self._radius,
            "grid_size": self._gs,
            "z_range": [self._z_min, self._z_max],
            "inflation_m": self._inf_radius,
            "robot_clear_m": self._robot_clear_radius,
            "frame_id": self._frame_id,
            "raycast_free_space": self._raycast_free_space,
            "unknown_as_obstacle_for_costmap": self._unknown_as_obstacle_for_costmap,
        }
        return info

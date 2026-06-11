#!/usr/bin/env python3
"""ROS 2 smoke test for Gazebo-backed frontier exploration.

The script starts LingTu's in-process WavefrontFrontierExplorer, builds a
2D occupancy grid by raytracing Gazebo LiDAR points from /nav/map_cloud,
forwards produced frontier goals to /nav/goal_pose, and verifies the ROS
navigation stack reacts:

    WavefrontFrontierExplorer -> /nav/goal_pose -> /nav/global_path
    -> /nav/local_path -> /nav/cmd_vel -> Gazebo odometry movement

It never publishes cmd_vel and is intended for server-side Gazebo gates only.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
for item in (ROOT, SRC):
    if str(item) not in sys.path:
        sys.path.insert(0, str(item))


@dataclass
class GazeboFrontierExplorationResult:
    ok: bool = False
    simulation_only: bool = True
    real_robot_motion: bool = False
    cmd_vel_sent_to_hardware: bool = False
    frontier_started: bool = False
    frontier_goal_seen: bool = False
    frontier_goal_published: bool = False
    odometry_seen: bool = False
    map_cloud_seen: bool = False
    terrain_map_seen: bool = False
    terrain_map_ext_seen: bool = False
    cumulative_map_cloud_seen: bool = False
    global_path_seen: bool = False
    local_path_seen: bool = False
    cmd_vel_seen: bool = False
    cmd_vel_nonzero: bool = False
    odom_start_xy: tuple[float, float] | None = None
    odom_last_xy: tuple[float, float] | None = None
    odom_delta_m: float = 0.0
    odom_delta_x_m: float = 0.0
    odom_path_length_m: float = 0.0
    costmap_source: str = "gazebo_lidar_derived"
    mapping_source: str = "gazebo_lidar_raytrace"
    lidar_map_updates: int = 0
    raytrace_updates: int = 0
    known_cells_initial: int = 0
    known_cells_final: int = 0
    known_cells_delta: int = 0
    free_cells_final: int = 0
    occupied_cells_final: int = 0
    explored_area_initial_m2: float = 0.0
    explored_area_final_m2: float = 0.0
    explored_area_delta_m2: float = 0.0
    sensor_map_area_delta_m2: float = 0.0
    frontier_goal: tuple[float, float, float] | None = None
    frontier_goal_count: int = 0
    frontier_goal_in_room: bool = False
    frontier_count_max: int = 0
    global_path_z_range: tuple[float, float] | None = None
    local_path_z_range: tuple[float, float] | None = None
    trajectory_quality: dict = field(default_factory=dict)
    topic_sync: dict = field(default_factory=dict)
    frontier_no_gain_stall: dict = field(default_factory=dict)
    cumulative_map_cloud: dict = field(default_factory=dict)
    registered_cloud: dict = field(default_factory=dict)
    static_obstacles: dict = field(default_factory=dict)
    map_artifacts: dict = field(default_factory=dict)
    samples: dict[str, int] = field(default_factory=dict)
    errors: list[str] = field(default_factory=list)

    def as_dict(self) -> dict:
        return {
            "schema_version": "lingtu.gazebo_frontier_exploration.v1",
            "ok": self.ok,
            "simulation_only": self.simulation_only,
            "real_robot_motion": self.real_robot_motion,
            "cmd_vel_sent_to_hardware": self.cmd_vel_sent_to_hardware,
            "frontier_started": self.frontier_started,
            "frontier_goal_seen": self.frontier_goal_seen,
            "frontier_goal_published": self.frontier_goal_published,
            "odometry_seen": self.odometry_seen,
            "map_cloud_seen": self.map_cloud_seen,
            "terrain_map_seen": self.terrain_map_seen,
            "terrain_map_ext_seen": self.terrain_map_ext_seen,
            "cumulative_map_cloud_seen": self.cumulative_map_cloud_seen,
            "global_path_seen": self.global_path_seen,
            "local_path_seen": self.local_path_seen,
            "cmd_vel_seen": self.cmd_vel_seen,
            "cmd_vel_nonzero": self.cmd_vel_nonzero,
            "odom_start_xy": self.odom_start_xy,
            "odom_last_xy": self.odom_last_xy,
            "odom_delta_m": self.odom_delta_m,
            "odom_delta_x_m": self.odom_delta_x_m,
            "odom_path_length_m": self.odom_path_length_m,
            "costmap_source": self.costmap_source,
            "mapping_source": self.mapping_source,
            "lidar_map_updates": self.lidar_map_updates,
            "raytrace_updates": self.raytrace_updates,
            "known_cells_initial": self.known_cells_initial,
            "known_cells_final": self.known_cells_final,
            "known_cells_delta": self.known_cells_delta,
            "free_cells_final": self.free_cells_final,
            "occupied_cells_final": self.occupied_cells_final,
            "explored_area_initial_m2": self.explored_area_initial_m2,
            "explored_area_final_m2": self.explored_area_final_m2,
            "explored_area_delta_m2": self.explored_area_delta_m2,
            "sensor_map_area_delta_m2": self.sensor_map_area_delta_m2,
            "frontier_goal": self.frontier_goal,
            "frontier_goal_count": self.frontier_goal_count,
            "frontier_goal_in_room": self.frontier_goal_in_room,
            "frontier_count_max": self.frontier_count_max,
            "global_path_z_range": self.global_path_z_range,
            "local_path_z_range": self.local_path_z_range,
            "trajectory_quality": self.trajectory_quality,
            "topic_sync": self.topic_sync,
            "frontier_no_gain_stall": self.frontier_no_gain_stall,
            "cumulative_map_cloud": self.cumulative_map_cloud,
            "registered_cloud": self.registered_cloud,
            "static_obstacles": self.static_obstacles,
            "map_artifacts": self.map_artifacts,
            "samples": self.samples,
            "errors": self.errors,
        }


def _count(samples: dict[str, int], topic: str) -> None:
    samples[topic] = samples.get(topic, 0) + 1


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _percentile(values: list[float], p: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(float(v) for v in values)
    idx = min(len(ordered) - 1, max(0, int(math.ceil((p / 100.0) * len(ordered)) - 1)))
    return ordered[idx]


def _nearest_skew_ms(stamp_sec: float, reference_stamps: list[float]) -> float | None:
    if stamp_sec <= 0.0 or not reference_stamps:
        return None
    return min(abs(stamp_sec - item) for item in reference_stamps) * 1000.0


def _point_to_polyline_distance(
    point: tuple[float, float],
    polyline: list[tuple[float, float]],
) -> float | None:
    if not polyline:
        return None
    px, py = point
    if len(polyline) == 1:
        return math.hypot(px - polyline[0][0], py - polyline[0][1])
    best = float("inf")
    for a, b in zip(polyline, polyline[1:]):
        ax, ay = a
        bx, by = b
        dx = bx - ax
        dy = by - ay
        denom = dx * dx + dy * dy
        if denom <= 1e-12:
            dist = math.hypot(px - ax, py - ay)
        else:
            t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
            dist = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
        best = min(best, dist)
    return best if math.isfinite(best) else None


def _path_is_robot_frame(frame_id: str) -> bool:
    normalized = str(frame_id or "").strip().lstrip("/")
    return normalized in {
        "body",
        "base",
        "base_link",
        "base_footprint",
        "robot",
        "vehicle",
    }


def _robot_path_to_world(
    points: list[tuple[float, float]],
    pose: tuple[float, float, float],
) -> list[tuple[float, float]]:
    x, y, yaw = pose
    c = math.cos(yaw)
    s = math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in points]


def _path_points_in_world(
    points: list[tuple[float, float]],
    *,
    frame_id: str,
    pose: tuple[float, float, float] | None,
) -> list[tuple[float, float]]:
    if not points:
        return []
    if _path_is_robot_frame(frame_id):
        if pose is None:
            return []
        return _robot_path_to_world(points, pose)
    return points


STATIC_OBSTACLE_ROI_PRESETS = {
    "demo_room": {
        "block": ((5.0, 0.0, 0.55), (0.9, 0.9, 1.2)),
        "column": ((1.6, -0.35, 0.6), (0.7, 0.7, 1.4)),
        "wall_left": ((3.0, 1.4, 0.45), (2.2, 0.45, 1.2)),
        "wall_right": ((3.0, -1.4, 0.45), (2.2, 0.45, 1.2)),
    },
    "industrial_park": {
        "loading_dock_wall": ((4.2, 3.1, 0.55), (4.6, 0.7, 1.4)),
        "pipe_rack_left": ((6.6, -3.25, 0.65), (3.8, 0.8, 1.6)),
        "pipe_rack_right": ((6.6, 3.25, 0.65), (3.8, 0.8, 1.6)),
        "pallet_cluster": ((3.5, -2.35, 0.38), (1.7, 1.4, 1.0)),
        "forklift_silhouette": ((12.2, 1.95, 0.45), (1.9, 1.2, 1.2)),
    },
    "none": {},
}


def _static_obstacle_rois(preset: str) -> dict[str, tuple[tuple[float, float, float], tuple[float, float, float]]]:
    return dict(STATIC_OBSTACLE_ROI_PRESETS.get(preset, STATIC_OBSTACLE_ROI_PRESETS["demo_room"]))


def _write_ascii_pcd(path: Path, points: list[tuple[float, float, float]]) -> int:
    path.parent.mkdir(parents=True, exist_ok=True)
    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS x y z",
            "SIZE 4 4 4",
            "TYPE F F F",
            "COUNT 1 1 1",
            f"WIDTH {len(points)}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {len(points)}",
            "DATA ascii",
        ]
    )
    with path.open("w", encoding="ascii") as f:
        f.write(header)
        f.write("\n")
        for x, y, z in points:
            f.write(f"{float(x):.5f} {float(y):.5f} {float(z):.5f}\n")
    return len(points)


def _point_bounds(points: list[tuple[float, float, float]]) -> dict[str, list[float]] | None:
    if not points:
        return None
    return {
        "min": [min(float(p[i]) for p in points) for i in range(3)],
        "max": [max(float(p[i]) for p in points) for i in range(3)],
    }


def _build_tomogram_from_pcd(
    *,
    pcd_path: Path,
    tomogram_path: Path,
    args: argparse.Namespace,
) -> dict[str, Any]:
    import numpy as np

    from global_planning.pct_planner.tomography.scripts.build_tomogram import (
        build_tomogram_from_pcd,
    )

    data = build_tomogram_from_pcd(
        str(pcd_path),
        str(tomogram_path),
        resolution=args.tomogram_resolution,
        slice_dh=args.tomogram_slice_dh,
        ground_h=args.tomogram_ground_h,
        kernel_size=args.tomogram_kernel_size,
        interval_min=args.tomogram_interval_min,
        interval_free=args.tomogram_interval_free,
        slope_max=args.tomogram_slope_max,
        step_max=args.tomogram_step_max,
        standable_ratio=args.tomogram_standable_ratio,
        cost_barrier=args.tomogram_cost_barrier,
        safe_margin=args.tomogram_safe_margin,
        inflation=args.tomogram_inflation,
    )
    arr = np.asarray(data.get("data"))
    return {
        "path": str(tomogram_path),
        "exists": tomogram_path.exists(),
        "shape": [int(v) for v in arr.shape],
        "resolution": float(data.get("resolution", args.tomogram_resolution)),
        "center": np.asarray(data.get("center", [0.0, 0.0]), dtype=float).tolist(),
        "slice_h0": float(data.get("slice_h0", 0.0)),
        "slice_dh": float(data.get("slice_dh", args.tomogram_slice_dh)),
    }


class LidarOccupancyGrid:
    """Small server-side occupancy grid built only from Gazebo LiDAR hits.

    The room bounds are a simulation geofence. Inside those bounds, unknown/free
    state comes from raytraced /nav/map_cloud points, not from odometry reveal.
    """

    def __init__(
        self,
        *,
        size_m: float,
        resolution_m: float,
        robot_free_radius_m: float,
        room_min_x: float,
        room_max_x: float,
        room_min_y: float,
        room_max_y: float,
    ) -> None:
        self.resolution = float(resolution_m)
        self.robot_free_radius = float(robot_free_radius_m)
        self.width = int(size_m / self.resolution)
        self.height = self.width
        self.origin_x = -size_m / 2.0
        self.origin_y = -size_m / 2.0
        self.room_min_x = float(room_min_x)
        self.room_max_x = float(room_max_x)
        self.room_min_y = float(room_min_y)
        self.room_max_y = float(room_max_y)
        self.grid: list[list[int]] | None = None
        self._inside_mask = None
        self._robot_free_cells: set[tuple[int, int]] = set()
        import numpy as np

        self.grid = np.full((self.height, self.width), -1, dtype=np.int16)
        self._inside_mask = np.zeros((self.height, self.width), dtype=bool)
        self._seed_room_geofence()

    def _cell(self, x: float, y: float) -> tuple[int, int] | None:
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        if 0 <= row < self.height and 0 <= col < self.width:
            return row, col
        return None

    def _seed_room_geofence(self) -> None:
        assert self.grid is not None
        assert self._inside_mask is not None
        for row in range(self.height):
            y = self.origin_y + (row + 0.5) * self.resolution
            for col in range(self.width):
                x = self.origin_x + (col + 0.5) * self.resolution
                inside = self._inside_room(x, y)
                self._inside_mask[row, col] = inside
                if not inside:
                    self.grid[row, col] = 100

    def _inside_room(self, x: float, y: float) -> bool:
        return (
            self.room_min_x <= x <= self.room_max_x
            and self.room_min_y <= y <= self.room_max_y
        )

    def out_of_room_distance(self, x: float, y: float) -> float:
        dx = max(self.room_min_x - x, 0.0, x - self.room_max_x)
        dy = max(self.room_min_y - y, 0.0, y - self.room_max_y)
        return math.hypot(dx, dy)

    def cell_value(self, x: float, y: float) -> int | None:
        assert self.grid is not None
        cell = self._cell(x, y)
        if cell is None:
            return None
        row, col = cell
        return int(self.grid[row, col])

    def min_distance_to_occupied(self, x: float, y: float) -> float | None:
        assert self.grid is not None
        assert self._inside_mask is not None
        import numpy as np

        rows, cols = np.where((np.asarray(self.grid) >= 100) & self._inside_mask)
        if len(rows) == 0:
            return None
        best = float("inf")
        for row, col in zip(rows, cols):
            wx = self.origin_x + (int(col) + 0.5) * self.resolution
            wy = self.origin_y + (int(row) + 0.5) * self.resolution
            best = min(best, math.hypot(wx - x, wy - y))
        return best if math.isfinite(best) else None

    def path_cell_report(self, points: list[tuple[float, float]]) -> dict:
        outside = 0
        occupied = 0
        unknown = 0
        free = 0
        min_clearance: float | None = None
        for x, y in points:
            if not self._inside_room(x, y):
                outside += 1
                continue
            value = self.cell_value(x, y)
            if value is None:
                outside += 1
            elif value >= 100:
                occupied += 1
            elif value < 0:
                unknown += 1
            else:
                free += 1
            clearance = self.min_distance_to_occupied(x, y)
            if clearance is not None:
                min_clearance = clearance if min_clearance is None else min(min_clearance, clearance)
        total = len(points)
        return {
            "samples": total,
            "outside_count": outside,
            "occupied_count": occupied,
            "unknown_count": unknown,
            "free_count": free,
            "unknown_ratio": round(float(unknown) / float(total), 4) if total else 0.0,
            "min_clearance_m": round(min_clearance, 4) if min_clearance is not None else None,
        }

    def _bresenham(self, start: tuple[int, int], end: tuple[int, int]) -> list[tuple[int, int]]:
        r0, c0 = start
        r1, c1 = end
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dc - dr
        row, col = r0, c0
        cells: list[tuple[int, int]] = []
        while True:
            cells.append((row, col))
            if row == r1 and col == c1:
                break
            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                col += sc
            if e2 < dc:
                err += dc
                row += sr
        return cells

    def mark_robot_free(self, x: float, y: float) -> int:
        assert self.grid is not None
        center = self._cell(x, y)
        if center is None:
            return 0
        row0, col0 = center
        radius = max(1, int(self.robot_free_radius / self.resolution))
        changed = 0
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if math.hypot(dr, dc) * self.resolution > self.robot_free_radius:
                    continue
                row = row0 + dr
                col = col0 + dc
                wx = self.origin_x + (col + 0.5) * self.resolution
                wy = self.origin_y + (row + 0.5) * self.resolution
                if (
                    0 <= row < self.height
                    and 0 <= col < self.width
                    and self._inside_room(wx, wy)
                    and self._inside_mask[row, col]
                ):
                    self._robot_free_cells.add((row, col))
                    if self.grid[row, col] != 0:
                        self.grid[row, col] = 0
                        changed += 1
        return changed

    def restore_robot_free_cells(self) -> int:
        assert self.grid is not None
        changed = 0
        for row, col in self._robot_free_cells:
            if (
                0 <= row < self.height
                and 0 <= col < self.width
                and self.grid[row, col] != 0
            ):
                self.grid[row, col] = 0
                changed += 1
        return changed

    def update_from_cloud(
        self,
        *,
        pose: tuple[float, float, float],
        points: list[tuple[float, float, float]],
        min_z: float,
        max_z: float,
        min_range_m: float,
        max_range_m: float,
        max_rays: int,
    ) -> int:
        assert self.grid is not None
        rx, ry, _ = pose
        changed = self.mark_robot_free(rx, ry)
        start = self._cell(rx, ry)
        if start is None or not points:
            return changed
        step = max(1, int(math.ceil(len(points) / max(1, max_rays))))
        for px, py, pz in points[::step]:
            if not (min_z <= pz <= max_z):
                continue
            distance = math.hypot(px - rx, py - ry)
            if distance < min_range_m or distance > max_range_m:
                continue
            if distance <= self.robot_free_radius:
                continue
            if not self._inside_room(px, py):
                continue
            end = self._cell(px, py)
            if end is None:
                continue
            cells = self._bresenham(start, end)
            for row, col in cells[:-1]:
                if not (0 <= row < self.height and 0 <= col < self.width):
                    continue
                if not self._inside_mask[row, col]:
                    break
                if self.grid[row, col] != 0:
                    self.grid[row, col] = 0
                    changed += 1
            erow, ecol = cells[-1]
            if (
                0 <= erow < self.height
                and 0 <= ecol < self.width
                and self._inside_mask[erow, ecol]
                and (erow, ecol) not in self._robot_free_cells
                and self.grid[erow, ecol] != 100
            ):
                self.grid[erow, ecol] = 100
                changed += 1
        changed += self.restore_robot_free_cells()
        return changed

    def known_cells(self) -> int:
        assert self.grid is not None
        assert self._inside_mask is not None
        import numpy as np

        grid = np.asarray(self.grid)
        return int(np.count_nonzero((grid >= 0) & self._inside_mask))

    def free_cells(self) -> int:
        assert self.grid is not None
        assert self._inside_mask is not None
        import numpy as np

        grid = np.asarray(self.grid)
        return int(np.count_nonzero((grid >= 0) & (grid <= 50) & self._inside_mask))

    def occupied_cells(self) -> int:
        assert self.grid is not None
        assert self._inside_mask is not None
        import numpy as np

        grid = np.asarray(self.grid)
        return int(np.count_nonzero((grid >= 100) & self._inside_mask))

    def known_points(self, *, max_points: int = 1200) -> list[tuple[float, float]]:
        assert self.grid is not None
        import numpy as np

        assert self._inside_mask is not None
        rows, cols = np.where((np.asarray(self.grid) >= 0) & self._inside_mask)
        if len(rows) == 0:
            return []
        step = max(1, int(math.ceil(len(rows) / max_points)))
        return [
            (
                float(self.origin_x + (int(col) + 0.5) * self.resolution),
                float(self.origin_y + (int(row) + 0.5) * self.resolution),
            )
            for row, col in zip(rows[::step], cols[::step])
        ]

    def navigation_artifact_points(
        self,
        *,
        free_z_m: float,
        obstacle_heights_m: tuple[float, ...],
        stride: int,
    ) -> list[tuple[float, float, float]]:
        """Return a cleaned map PCD derived from the occupancy grid.

        Raw cumulative LiDAR clouds preserve scan rings and vertical return
        layers. This artifact is a navigation map: free cells become floor
        samples and occupied cells become sparse obstacle columns.
        """

        assert self.grid is not None
        assert self._inside_mask is not None
        points: list[tuple[float, float, float]] = []
        step = max(1, int(stride))
        for row in range(0, self.height, step):
            for col in range(0, self.width, step):
                if not self._inside_mask[row, col]:
                    continue
                value = int(self.grid[row, col])
                if value < 0:
                    continue
                x = float(self.origin_x + (col + 0.5) * self.resolution)
                y = float(self.origin_y + (row + 0.5) * self.resolution)
                if value >= 100:
                    for z in obstacle_heights_m:
                        points.append((x, y, float(z)))
                else:
                    points.append((x, y, float(free_z_m)))
        return points

    def as_costmap(self) -> dict:
        assert self.grid is not None
        return {
            "grid": self.grid.copy(),
            "resolution": self.resolution,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "width": self.width,
            "height": self.height,
            "frame_id": "map",
        }

    def to_ros_grid(self, ros_grid_type, stamp):
        assert self.grid is not None
        msg = ros_grid_type()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.info.resolution = float(self.resolution)
        msg.info.width = int(self.width)
        msg.info.height = int(self.height)
        msg.info.origin.position.x = float(self.origin_x)
        msg.info.origin.position.y = float(self.origin_y)
        msg.info.origin.orientation.w = 1.0
        msg.data = [int(v) for v in self.grid.reshape(-1)]
        return msg


def run_smoke(args: argparse.Namespace) -> GazeboFrontierExplorationResult:
    import rclpy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from nav_msgs.msg import OccupancyGrid as ROSOccupancyGrid
    from nav_msgs.msg import Odometry, Path as ROSPath
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    from core.msgs.geometry import Pose as CorePose
    from core.msgs.nav import Odometry as CoreOdometry
    from nav.frontier_explorer_module import WavefrontFrontierExplorer

    rclpy.init(args=None)
    node = Node("lingtu_gazebo_frontier_exploration_smoke")
    result = GazeboFrontierExplorationResult()
    lock = threading.Lock()
    occupancy = LidarOccupancyGrid(
        size_m=args.coverage_size_m,
        resolution_m=args.coverage_resolution_m,
        robot_free_radius_m=args.robot_free_radius_m,
        room_min_x=args.room_min_x,
        room_max_x=args.room_max_x,
        room_min_y=args.room_min_y,
        room_max_y=args.room_max_y,
    )
    goal_pub = node.create_publisher(PoseStamped, "/nav/goal_pose", 10)
    occupancy_pub = node.create_publisher(ROSOccupancyGrid, "/nav/exploration_grid", 2)

    explorer = WavefrontFrontierExplorer(
        min_frontier_size=args.min_frontier_size,
        safe_distance=args.frontier_safe_distance_m,
        lookahead_distance=6.0,
        max_explored_distance=8.0,
        info_gain_threshold=0.0,
        goal_timeout=args.frontier_goal_timeout_sec,
        explore_rate=args.frontier_rate_hz,
        costmap_max_age=10.0,
    )
    current_pose: tuple[float, float, float] | None = None
    current_goal: tuple[float, float, float] | None = None
    current_frontiers: list[dict[str, float]] = []
    current_global_path: list[tuple[float, float]] = []
    current_local_path: list[tuple[float, float]] = []
    current_local_path_frame = ""
    current_cmd_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
    last_goal_publish_at = 0.0
    trace: list[dict] = []
    odom_stamps: list[float] = []
    odom_xy_history: list[tuple[float, float]] = []
    map_cloud_stamp_secs: list[float] = []
    registered_cloud_stamp_secs: list[float] = []
    terrain_map_stamp_secs: list[float] = []
    terrain_map_ext_stamp_secs: list[float] = []
    map_cloud_frames: set[str] = set()
    registered_cloud_frames: set[str] = set()
    terrain_map_frames: set[str] = set()
    terrain_map_ext_frames: set[str] = set()
    global_path_frames: set[str] = set()
    local_path_frames: set[str] = set()
    local_path_start_distances: list[float] = []
    local_path_tracking_errors: list[float] = []
    local_path_goal_alignments: list[float] = []
    local_path_reports: list[dict] = []
    trajectory_state = {
        "last_xy": None,
        "path_length_m": 0.0,
        "room_violation_count": 0,
        "max_out_of_room_m": 0.0,
        "max_abs_y_m": 0.0,
    }
    run_started_at = time.monotonic()
    cumulative_point_counts: list[int] = []
    cumulative_unique_counts: list[int] = []
    cumulative_frames: set[str] = set()
    cumulative_retention_ratios: list[float] = []
    cumulative_prev_voxels: set[tuple[int, int, int]] | None = None
    cumulative_growth_steps = 0
    cumulative_step_count = 0
    registered_unique_counts: list[int] = []
    static_obstacle_rois = _static_obstacle_rois(args.static_roi_preset)
    static_centroids: dict[str, list[tuple[float, float, float]]] = {
        name: [] for name in static_obstacle_rois
    }
    map_artifact_points: dict[tuple[int, int, int], tuple[float, float, float]] = {}
    initial_sensor_map_recorded = False

    def read_xyz_points(msg: PointCloud2, *, max_points: int) -> list[tuple[float, float, float]]:
        fields = {field.name for field in msg.fields}
        if not {"x", "y", "z"} <= fields:
            return []
        points: list[tuple[float, float, float]] = []
        for raw in point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        ):
            x, y, z = float(raw[0]), float(raw[1]), float(raw[2])
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                points.append((x, y, z))
            if len(points) >= max_points:
                break
        return points

    def voxelize(points: list[tuple[float, float, float]]) -> set[tuple[int, int, int]]:
        size = args.cumulative_voxel_size_m
        return {
            (
                math.floor(x / size),
                math.floor(y / size),
                math.floor(z / size),
            )
            for x, y, z in points
        }

    def accumulate_map_artifact(points: list[tuple[float, float, float]]) -> None:
        if not (args.pcd_out or args.tomogram_out or args.build_tomogram):
            return
        if len(map_artifact_points) >= args.map_artifact_max_points:
            return
        size = max(1e-6, float(args.map_artifact_voxel_size_m))
        inv = 1.0 / size
        for x, y, z in points:
            if z < args.map_artifact_min_z_m or z > args.map_artifact_max_z_m:
                continue
            cell = (math.floor(x * inv), math.floor(y * inv), math.floor(z * inv))
            if cell not in map_artifact_points:
                map_artifact_points[cell] = (x, y, z)
                if len(map_artifact_points) >= args.map_artifact_max_points:
                    break

    def roi_centroids(points: list[tuple[float, float, float]]) -> dict[str, tuple[float, float, float]]:
        out: dict[str, tuple[float, float, float]] = {}
        for name, (center, span) in static_obstacle_rois.items():
            cx, cy, cz = center
            sx, sy, sz = span
            selected = [
                (x, y, z)
                for x, y, z in points
                if abs(x - cx) <= sx / 2.0
                and abs(y - cy) <= sy / 2.0
                and abs(z - cz) <= sz / 2.0
            ]
            if len(selected) < args.min_static_roi_points:
                continue
            n = float(len(selected))
            out[name] = (
                sum(x for x, _, _ in selected) / n,
                sum(y for _, y, _ in selected) / n,
                sum(z for _, _, z in selected) / n,
            )
        return out

    def max_centroid_drift(centroids: list[tuple[float, float, float]]) -> float:
        if len(centroids) < 2:
            return 0.0
        window = centroids[-max(2, int(args.static_drift_window_samples)) :]
        first = window[0]
        return max(
            math.sqrt(
                (item[0] - first[0]) ** 2
                + (item[1] - first[1]) ** 2
                + (item[2] - first[2]) ** 2
            )
            for item in window[1:]
        )

    def update_cumulative_report() -> None:
        nonlocal cumulative_growth_steps, cumulative_step_count
        initial_points = cumulative_point_counts[0] if cumulative_point_counts else 0
        final_points = cumulative_point_counts[-1] if cumulative_point_counts else 0
        initial_voxels = cumulative_unique_counts[0] if cumulative_unique_counts else 0
        final_voxels = cumulative_unique_counts[-1] if cumulative_unique_counts else 0
        point_growth_ratio = (
            float(final_points) / float(initial_points)
            if initial_points > 0
            else 0.0
        )
        voxel_growth_ratio = (
            float(final_voxels) / float(initial_voxels)
            if initial_voxels > 0
            else 0.0
        )
        growth_step_ratio = (
            float(cumulative_growth_steps) / float(cumulative_step_count)
            if cumulative_step_count > 0
            else 0.0
        )
        retention_min = min(cumulative_retention_ratios) if cumulative_retention_ratios else 0.0
        static_report = {
            name: {
                "samples": len(values),
                "centroid_drift_max_m": round(max_centroid_drift(values), 4),
            }
            for name, values in static_centroids.items()
            if values
        }
        result.cumulative_map_cloud = {
            "samples": len(cumulative_point_counts),
            "frame_ids": sorted(cumulative_frames),
            "point_count_initial": initial_points,
            "point_count_final": final_points,
            "point_count_delta": final_points - initial_points,
            "point_growth_ratio": round(point_growth_ratio, 4),
            "unique_voxels_initial": initial_voxels,
            "unique_voxels_final": final_voxels,
            "unique_voxels_delta": final_voxels - initial_voxels,
            "unique_voxel_growth_ratio": round(voxel_growth_ratio, 4),
            "growth_step_ratio": round(growth_step_ratio, 4),
            "retention_min": round(retention_min, 4),
        }
        result.static_obstacles = static_report
        result.static_obstacles["roi_preset"] = args.static_roi_preset
        stable_static = [
            name
            for name, item in static_report.items()
            if isinstance(item, dict)
            and int(item.get("samples") or 0) >= 2
            and float(item.get("centroid_drift_max_m") or 0.0)
            <= args.max_static_centroid_drift_m
        ]
        median_registered = 0
        if registered_unique_counts:
            ordered = sorted(registered_unique_counts)
            median_registered = int(ordered[len(ordered) // 2])
        result.registered_cloud = {
            "samples": len(registered_unique_counts),
            "median_unique_voxels": median_registered,
            "map_vs_registered_voxel_ratio": round(
                float(final_voxels) / float(median_registered),
                4,
            )
            if median_registered > 0
            else 0.0,
        }
        result.registered_cloud["stable_static_obstacles"] = stable_static

    def cumulative_gate_ready() -> bool:
        update_cumulative_report()
        stats = result.cumulative_map_cloud
        registered = result.registered_cloud
        static = result.static_obstacles
        if int(stats.get("samples") or 0) < args.min_cumulative_samples:
            return False
        point_delta = int(stats.get("point_count_delta") or 0)
        point_ratio = float(stats.get("point_growth_ratio") or 0.0)
        voxel_delta = int(stats.get("unique_voxels_delta") or 0)
        voxel_ratio = float(stats.get("unique_voxel_growth_ratio") or 0.0)
        if point_delta < args.min_cumulative_point_delta and point_ratio < args.min_cumulative_point_growth_ratio:
            return False
        if voxel_delta < args.min_cumulative_voxel_delta and voxel_ratio < args.min_cumulative_voxel_growth_ratio:
            return False
        if float(stats.get("growth_step_ratio") or 0.0) < args.min_cumulative_growth_step_ratio:
            return False
        if float(stats.get("retention_min") or 0.0) < args.min_cumulative_retention_ratio:
            return False
        if (
            float(registered.get("map_vs_registered_voxel_ratio") or 0.0)
            < args.min_map_vs_registered_voxel_ratio
        ):
            return False
        return sum(
            isinstance(item, dict)
            and int(item.get("samples") or 0) >= 2
            and float(item.get("centroid_drift_max_m") or 0.0) <= args.max_static_centroid_drift_m
            for item in static.values()
        ) >= args.min_stable_static_rois

    def _skew_stats(values: list[float]) -> dict:
        if not values:
            return {
                "samples": 0,
                "max_ms": None,
                "p95_ms": None,
            }
        return {
            "samples": len(values),
            "max_ms": round(max(values), 3),
            "p95_ms": round(_percentile(values, 95.0), 3),
        }

    def _skews_for(stamps: list[float]) -> list[float]:
        values: list[float] = []
        for stamp in stamps:
            skew = _nearest_skew_ms(stamp, odom_stamps)
            if skew is not None:
                values.append(skew)
        ignore = min(
            max(0, int(args.topic_sync_ignore_initial_samples)),
            max(0, len(values) - int(args.min_topic_sync_samples)),
        )
        if ignore > 0:
            return values[ignore:]
        return values

    def update_topic_sync_report() -> None:
        map_cloud_skews_ms = _skews_for(map_cloud_stamp_secs)
        registered_cloud_skews_ms = _skews_for(registered_cloud_stamp_secs)
        terrain_map_skews_ms = _skews_for(terrain_map_stamp_secs)
        terrain_map_ext_skews_ms = _skews_for(terrain_map_ext_stamp_secs)
        max_skew_ms = 0.0
        for values in (
            map_cloud_skews_ms,
            registered_cloud_skews_ms,
            terrain_map_skews_ms,
            terrain_map_ext_skews_ms,
        ):
            if values:
                max_skew_ms = max(max_skew_ms, max(values))
        result.topic_sync = {
            "ok": (
                len(map_cloud_skews_ms) >= args.min_topic_sync_samples
                and len(registered_cloud_skews_ms) >= args.min_topic_sync_samples
                and max_skew_ms <= args.max_cloud_odom_skew_ms
            ),
            "max_cloud_odom_skew_ms": round(max_skew_ms, 3),
            "ignored_initial_samples": int(args.topic_sync_ignore_initial_samples),
            "map_cloud": _skew_stats(map_cloud_skews_ms),
            "registered_cloud": _skew_stats(registered_cloud_skews_ms),
            "terrain_map": _skew_stats(terrain_map_skews_ms),
            "terrain_map_ext": _skew_stats(terrain_map_ext_skews_ms),
            "frames": {
                "map_cloud": sorted(map_cloud_frames),
                "registered_cloud": sorted(registered_cloud_frames),
                "terrain_map": sorted(terrain_map_frames),
                "terrain_map_ext": sorted(terrain_map_ext_frames),
                "global_path": sorted(global_path_frames),
                "local_path": sorted(local_path_frames),
            },
        }

    def write_map_artifacts() -> None:
        requested = bool(args.pcd_out or args.tomogram_out or args.build_tomogram)
        if not requested:
            return
        if args.map_artifact_source == "occupancy_grid":
            obstacle_heights = tuple(
                float(item)
                for item in str(args.map_artifact_obstacle_heights_m).split(",")
                if item.strip()
            )
            points = occupancy.navigation_artifact_points(
                free_z_m=args.map_artifact_free_z_m,
                obstacle_heights_m=obstacle_heights or (0.35, 0.85, 1.25),
                stride=args.map_artifact_grid_stride,
            )
            source_topic = "/nav/exploration_grid"
        else:
            points = list(map_artifact_points.values())
            source_topic = "/nav/cumulative_map_cloud"
        artifacts: dict[str, Any] = {
            "source": args.map_artifact_source,
            "source_topic": source_topic,
            "point_count": len(points),
            "bounds": _point_bounds(points),
            "voxel_size_m": float(args.map_artifact_voxel_size_m),
            "z_filter_m": [
                float(args.map_artifact_min_z_m),
                float(args.map_artifact_max_z_m),
            ],
            "occupancy_cells": {
                "known": result.known_cells_final,
                "free": result.free_cells_final,
                "occupied": result.occupied_cells_final,
                "resolution_m": occupancy.resolution,
            },
        }
        if len(points) < args.min_map_artifact_points:
            result.map_artifacts = artifacts
            result.errors.append(
                "explored map artifact point count "
                f"{len(points)} < {args.min_map_artifact_points}"
            )
            return
        pcd_path = Path(args.pcd_out) if args.pcd_out else None
        tomogram_path = Path(args.tomogram_out) if args.tomogram_out else None
        if pcd_path is not None:
            written = _write_ascii_pcd(pcd_path, points)
            artifacts["pcd"] = {
                "path": str(pcd_path),
                "exists": pcd_path.exists(),
                "point_count": written,
            }
        if args.build_tomogram:
            if pcd_path is None:
                result.errors.append("--build-tomogram requires --pcd-out")
            elif tomogram_path is None:
                result.errors.append("--build-tomogram requires --tomogram-out")
            else:
                try:
                    artifacts["tomogram"] = _build_tomogram_from_pcd(
                        pcd_path=pcd_path,
                        tomogram_path=tomogram_path,
                        args=args,
                    )
                except Exception as exc:
                    result.errors.append(
                        "explored map tomogram build failed: "
                        f"{type(exc).__name__}: {exc}"
                    )
        result.map_artifacts = artifacts

    def terrain_topics_ready() -> bool:
        return (
            result.terrain_map_seen
            and result.terrain_map_ext_seen
            and int(result.samples.get("/nav/terrain_map") or 0) >= args.min_terrain_map_samples
            and int(result.samples.get("/nav/terrain_map_ext") or 0) >= args.min_terrain_map_samples
        )

    def update_trajectory_quality_report() -> None:
        path_length = float(trajectory_state["path_length_m"])
        net = float(result.odom_delta_m)
        ratio = path_length / max(net, 1e-6) if path_length > 0.0 and net > 0.0 else 0.0
        local_path_latest = local_path_reports[-1] if local_path_reports else {}
        local_path_worst_occupied = max(
            (int(item.get("occupied_count") or 0) for item in local_path_reports),
            default=0,
        )
        local_path_worst_outside = max(
            (int(item.get("outside_count") or 0) for item in local_path_reports),
            default=0,
        )
        local_path_worst_unknown_ratio = max(
            (float(item.get("unknown_ratio") or 0.0) for item in local_path_reports),
            default=0.0,
        )
        clearance_candidates = [
            float(clearance)
            for x, y in odom_xy_history
            if occupancy._inside_room(x, y)
            for clearance in [occupancy.min_distance_to_occupied(x, y)]
            if clearance is not None
        ]
        for item in local_path_reports:
            value = item.get("min_clearance_m")
            if value is not None:
                clearance_candidates.append(float(value))
        min_clearance = min(clearance_candidates) if clearance_candidates else None
        start_distance_latest = (
            local_path_start_distances[-1] if local_path_start_distances else None
        )
        tracking_p95 = _percentile(local_path_tracking_errors, 95.0)
        alignment_min = min(local_path_goal_alignments) if local_path_goal_alignments else 0.0
        alignment_median = _percentile(local_path_goal_alignments, 50.0)
        alignment_latest = local_path_goal_alignments[-1] if local_path_goal_alignments else 0.0
        alignment_sustained = max(alignment_median, alignment_latest)
        checks = {
            "odom_samples": len(odom_stamps) >= args.min_trajectory_odom_samples,
            "room_bounds": (
                int(trajectory_state["room_violation_count"]) <= args.max_room_violation_count
                and float(trajectory_state["max_out_of_room_m"]) <= args.max_trajectory_out_of_room_m
            ),
            "path_length_ratio": ratio <= args.max_odom_path_length_ratio if ratio > 0.0 else False,
            "lateral_bounds": float(trajectory_state["max_abs_y_m"]) <= args.max_trajectory_abs_y_m,
            "obstacle_clearance": (
                min_clearance is not None
                and min_clearance >= args.min_obstacle_clearance_m
            ),
            "local_path_start": (
                start_distance_latest is not None
                and start_distance_latest <= args.max_local_path_start_distance_m
            ),
            "local_path_tracking": (
                bool(local_path_tracking_errors)
                and tracking_p95 <= args.max_local_path_tracking_p95_m
            ),
            "local_path_cells": (
                local_path_worst_occupied <= args.max_local_path_occupied_overlap_count
                and local_path_worst_outside <= args.max_local_path_out_of_room_count
                and local_path_worst_unknown_ratio <= args.max_local_path_unknown_ratio
            ),
            "local_path_goal_alignment": (
                bool(local_path_goal_alignments)
                and alignment_sustained >= args.min_local_path_goal_alignment
            ),
        }
        result.odom_path_length_m = round(path_length, 4)
        result.trajectory_quality = {
            "ok": all(checks.values()),
            "checks": checks,
            "odom_samples": len(odom_stamps),
            "odom_path_length_m": result.odom_path_length_m,
            "odom_net_displacement_m": round(net, 4),
            "odom_path_length_ratio": round(ratio, 4),
            "room_violation_count": int(trajectory_state["room_violation_count"]),
            "max_out_of_room_m": round(float(trajectory_state["max_out_of_room_m"]), 4),
            "max_abs_y_m": round(float(trajectory_state["max_abs_y_m"]), 4),
            "min_obstacle_clearance_m": round(min_clearance, 4) if min_clearance is not None else None,
            "local_path_start_distance_latest_m": round(start_distance_latest, 4)
            if start_distance_latest is not None
            else None,
            "local_path_tracking_p95_m": round(tracking_p95, 4),
            "local_path_tracking_samples": len(local_path_tracking_errors),
            "local_path_goal_alignment_min": round(alignment_min, 4),
            "local_path_goal_alignment_median": round(alignment_median, 4),
            "local_path_goal_alignment_latest": round(alignment_latest, 4),
            "local_path_latest_cell_report": local_path_latest,
            "local_path_occupied_overlap_count": local_path_worst_occupied,
            "local_path_out_of_room_count": local_path_worst_outside,
            "local_path_unknown_ratio_max": round(local_path_worst_unknown_ratio, 4),
            "local_path_frame_ids": sorted(local_path_frames),
        }

    def trajectory_gate_ready() -> bool:
        update_trajectory_quality_report()
        update_topic_sync_report()
        return bool(result.trajectory_quality.get("ok")) and bool(result.topic_sync.get("ok"))

    def path_points(msg: ROSPath, *, limit: int = 160) -> tuple[list[tuple[float, float]], tuple[float, float] | None]:
        poses = list(msg.poses or [])
        if not poses:
            return [], None
        step = max(1, int(math.ceil(len(poses) / limit)))
        zs = [float(p.pose.position.z) for p in poses]
        points = [
            (float(p.pose.position.x), float(p.pose.position.y))
            for p in poses[::step]
        ]
        return points, (min(zs), max(zs))

    def room_forward_gate_ready() -> bool:
        if result.frontier_goal is None:
            return False
        if not result.frontier_goal_in_room:
            return False
        if result.frontier_goal[0] < args.min_frontier_goal_x:
            return False
        if result.odom_last_xy is None:
            return False
        x, y = result.odom_last_xy
        return (
            occupancy._inside_room(x, y)
            and result.odom_delta_x_m >= args.min_frontier_odom_delta_x_m
        )

    def publish_occupancy_grid() -> None:
        occupancy_pub.publish(
            occupancy.to_ros_grid(
                ROSOccupancyGrid,
                node.get_clock().now().to_msg(),
            )
        )

    def deliver_frontier_inputs() -> None:
        if current_pose is None:
            return
        x, y, yaw = current_pose
        explorer.odometry._deliver(
            CoreOdometry(pose=CorePose(x, y, 0.0, yaw=yaw), frame_id="map")
        )
        explorer.costmap._deliver(occupancy.as_costmap())
        publish_occupancy_grid()

    def update_sensor_map_metrics() -> None:
        nonlocal initial_sensor_map_recorded
        known = occupancy.known_cells()
        area = round(known * occupancy.resolution * occupancy.resolution, 4)
        result.known_cells_final = known
        result.free_cells_final = occupancy.free_cells()
        result.occupied_cells_final = occupancy.occupied_cells()
        result.explored_area_final_m2 = area
        if not initial_sensor_map_recorded and known > 0:
            result.known_cells_initial = known
            result.explored_area_initial_m2 = area
            initial_sensor_map_recorded = True
        result.known_cells_delta = known - result.known_cells_initial
        result.explored_area_delta_m2 = round(area - result.explored_area_initial_m2, 4)
        result.sensor_map_area_delta_m2 = result.explored_area_delta_m2

    def capture_trace_snapshot() -> None:
        if not args.trace_out or current_pose is None:
            return
        known = occupancy.known_cells()
        trace.append(
            {
                "t": round(time.monotonic() - run_started_at, 3),
                "pose": [round(float(v), 4) for v in current_pose],
                "goal": [round(float(v), 4) for v in current_goal] if current_goal else None,
                "mapping_source": result.mapping_source,
                "known_cells": int(known),
                "explored_area_m2": round(known * occupancy.resolution * occupancy.resolution, 4),
                "frontiers": current_frontiers[:40],
                "global_path": current_global_path,
                "local_path": current_local_path,
                "local_path_frame": current_local_path_frame,
                "cmd_vel": [round(float(v), 4) for v in current_cmd_vel],
                "trajectory_quality": result.trajectory_quality,
                "topic_sync": result.topic_sync,
                "known_points": occupancy.known_points(max_points=args.trace_max_known_points),
            }
        )

    def publish_goal_pose(
        goal_xyz: tuple[float, float, float],
        *,
        frame_id: str = "map",
        count_frontier_goal: bool = False,
    ) -> None:
        nonlocal current_goal, last_goal_publish_at
        ros_goal = PoseStamped()
        ros_goal.header.frame_id = frame_id or "map"
        ros_goal.header.stamp = node.get_clock().now().to_msg()
        ros_goal.pose.position.x = float(goal_xyz[0])
        ros_goal.pose.position.y = float(goal_xyz[1])
        ros_goal.pose.position.z = float(goal_xyz[2])
        ros_goal.pose.orientation.w = 1.0
        with lock:
            current_goal = (
                float(ros_goal.pose.position.x),
                float(ros_goal.pose.position.y),
                float(ros_goal.pose.position.z),
            )
            if count_frontier_goal:
                result.frontier_goal = current_goal
                result.frontier_goal_count += 1
                result.frontier_goal_in_room = occupancy._inside_room(
                    current_goal[0],
                    current_goal[1],
                )
                result.frontier_goal_seen = True
                _count(result.samples, "frontier_goal")
            result.frontier_goal_published = True
            _count(result.samples, "/nav/goal_pose")
        goal_pub.publish(ros_goal)
        last_goal_publish_at = time.monotonic()

    def publish_frontier_goal(goal) -> None:
        publish_goal_pose(
            (
                float(goal.pose.x),
                float(goal.pose.y),
                float(goal.pose.z),
            ),
            frame_id=getattr(goal, "frame_id", "map") or "map",
            count_frontier_goal=True,
        )

    def on_frontiers(frontiers: list) -> None:
        nonlocal current_frontiers
        current_frontiers = [
            {
                "cx": round(float(frontier.get("cx", 0.0)), 4),
                "cy": round(float(frontier.get("cy", 0.0)), 4),
                "size": int(frontier.get("size", 0)),
                "score": round(float(frontier.get("score", 0.0)), 4),
            }
            for frontier in list(frontiers or [])[:40]
            if isinstance(frontier, dict)
        ]
        with lock:
            _count(result.samples, "frontiers")
            result.frontier_count_max = max(result.frontier_count_max, len(frontiers or []))

    explorer.exploration_goal.subscribe(publish_frontier_goal)
    explorer.frontiers.subscribe(on_frontiers)
    explorer.setup()
    explorer.start()

    def on_odom(msg: Odometry) -> None:
        nonlocal current_pose
        _count(result.samples, "/nav/odometry")
        odom_stamps.append(_stamp_sec(msg.header.stamp))
        pose = msg.pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = _yaw_from_quat(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        current_pose = (x, y, yaw)
        odom_xy_history.append((x, y))
        if result.lidar_map_updates > 0:
            occupancy.mark_robot_free(x, y)
        previous_xy = trajectory_state["last_xy"]
        if previous_xy is not None:
            px, py = previous_xy
            trajectory_state["path_length_m"] = float(trajectory_state["path_length_m"]) + math.hypot(x - px, y - py)
        trajectory_state["last_xy"] = (x, y)
        out_distance = occupancy.out_of_room_distance(x, y)
        trajectory_state["max_out_of_room_m"] = max(
            float(trajectory_state["max_out_of_room_m"]),
            out_distance,
        )
        trajectory_state["max_abs_y_m"] = max(float(trajectory_state["max_abs_y_m"]), abs(y))
        if out_distance > args.max_trajectory_out_of_room_m:
            trajectory_state["room_violation_count"] = int(trajectory_state["room_violation_count"]) + 1
        if len(current_local_path) >= 2:
            tracking_error = _point_to_polyline_distance((x, y), current_local_path)
            if tracking_error is not None:
                local_path_tracking_errors.append(tracking_error)
        with lock:
            if not result.odometry_seen:
                result.odom_start_xy = (x, y)
            result.odometry_seen = True
            result.odom_last_xy = (x, y)
            if result.odom_start_xy is not None:
                result.odom_delta_m = math.hypot(
                    x - result.odom_start_xy[0],
                    y - result.odom_start_xy[1],
                )
                result.odom_delta_x_m = x - result.odom_start_xy[0]
            result.odom_path_length_m = round(float(trajectory_state["path_length_m"]), 4)
        if result.lidar_map_updates > 0:
            deliver_frontier_inputs()

        goal = current_goal
        if goal is not None and math.hypot(x - goal[0], y - goal[1]) <= args.goal_reached_radius_m:
            explorer.goal_reached._deliver(True)

    def on_global_path(msg: ROSPath) -> None:
        nonlocal current_global_path
        global_path_frames.add(str(msg.header.frame_id))
        current_global_path, z_range = path_points(msg)
        if z_range is not None:
            result.global_path_z_range = z_range
        _count(result.samples, "/nav/global_path")
        result.global_path_seen = result.global_path_seen or bool(msg.poses)

    def on_local_path(msg: ROSPath) -> None:
        nonlocal current_local_path, current_local_path_frame
        frame_id = str(msg.header.frame_id)
        local_path_frames.add(frame_id)
        raw_points, z_range = path_points(msg)
        current_local_path_frame = frame_id
        current_local_path = (
            _path_points_in_world(raw_points, frame_id=frame_id, pose=current_pose)
            if len(raw_points) >= 2
            else []
        )
        if z_range is not None:
            result.local_path_z_range = z_range
        if len(current_local_path) >= 2:
            report = occupancy.path_cell_report(current_local_path)
            report["frame_id"] = frame_id
            report["robot_frame"] = _path_is_robot_frame(frame_id)
            local_path_reports.append(report)
            if current_pose is not None:
                px, py, _ = current_pose
                local_path_start_distances.append(
                    math.hypot(current_local_path[0][0] - px, current_local_path[0][1] - py)
                )
                tracking_error = _point_to_polyline_distance((px, py), current_local_path)
                if tracking_error is not None:
                    local_path_tracking_errors.append(tracking_error)
                if current_goal is not None:
                    gx = current_goal[0] - px
                    gy = current_goal[1] - py
                    ex = current_local_path[-1][0] - px
                    ey = current_local_path[-1][1] - py
                    denom = math.hypot(gx, gy) * math.hypot(ex, ey)
                    if denom > 1e-9:
                        local_path_goal_alignments.append((gx * ex + gy * ey) / denom)
        _count(result.samples, "/nav/local_path")
        result.local_path_seen = result.local_path_seen or bool(msg.poses)

    def on_cmd_vel(msg: TwistStamped) -> None:
        nonlocal current_cmd_vel
        _count(result.samples, "/nav/cmd_vel")
        result.cmd_vel_seen = True
        tw = msg.twist
        current_cmd_vel = (
            float(tw.linear.x),
            float(tw.linear.y),
            float(tw.angular.z),
        )
        result.cmd_vel_nonzero = result.cmd_vel_nonzero or (
            abs(float(tw.linear.x)) > args.min_cmd_vel
            or abs(float(tw.linear.y)) > args.min_cmd_vel
            or abs(float(tw.angular.z)) > args.min_cmd_vel
        )

    def on_map_cloud(msg: PointCloud2) -> None:
        _count(result.samples, "/nav/map_cloud")
        result.map_cloud_seen = True
        map_cloud_frames.add(str(msg.header.frame_id))
        stamp = _stamp_sec(msg.header.stamp)
        if stamp > 0.0:
            map_cloud_stamp_secs.append(stamp)
        pose = current_pose
        if pose is None:
            return
        points = read_xyz_points(msg, max_points=args.max_occupancy_cloud_points)
        changed = occupancy.update_from_cloud(
            pose=pose,
            points=points,
            min_z=args.occupancy_min_z_m,
            max_z=args.occupancy_max_z_m,
            min_range_m=args.occupancy_min_range_m,
            max_range_m=args.occupancy_max_range_m,
            max_rays=args.max_occupancy_rays,
        )
        with lock:
            result.lidar_map_updates += 1
            result.raytrace_updates += max(0, int(changed))
        update_sensor_map_metrics()
        deliver_frontier_inputs()

    def on_registered_cloud(msg: PointCloud2) -> None:
        _count(result.samples, "/nav/registered_cloud")
        registered_cloud_frames.add(str(msg.header.frame_id))
        stamp = _stamp_sec(msg.header.stamp)
        if stamp > 0.0:
            registered_cloud_stamp_secs.append(stamp)
        points = read_xyz_points(msg, max_points=args.max_cloud_points)
        if points:
            registered_unique_counts.append(len(voxelize(points)))
            update_cumulative_report()

    def on_terrain_map(msg: PointCloud2) -> None:
        _count(result.samples, "/nav/terrain_map")
        result.terrain_map_seen = True
        terrain_map_frames.add(str(msg.header.frame_id))
        stamp = _stamp_sec(msg.header.stamp)
        if stamp > 0.0:
            terrain_map_stamp_secs.append(stamp)

    def on_terrain_map_ext(msg: PointCloud2) -> None:
        _count(result.samples, "/nav/terrain_map_ext")
        result.terrain_map_ext_seen = True
        terrain_map_ext_frames.add(str(msg.header.frame_id))
        stamp = _stamp_sec(msg.header.stamp)
        if stamp > 0.0:
            terrain_map_ext_stamp_secs.append(stamp)

    def on_cumulative_map_cloud(msg: PointCloud2) -> None:
        nonlocal cumulative_prev_voxels, cumulative_growth_steps, cumulative_step_count
        _count(result.samples, "/nav/cumulative_map_cloud")
        result.cumulative_map_cloud_seen = True
        cumulative_frames.add(str(msg.header.frame_id))
        cumulative_point_counts.append(int(msg.width) * int(msg.height))
        points = read_xyz_points(msg, max_points=args.max_cloud_points)
        accumulate_map_artifact(points)
        voxels = voxelize(points)
        cumulative_unique_counts.append(len(voxels))
        if cumulative_prev_voxels is not None:
            cumulative_step_count += 1
            if len(voxels) >= len(cumulative_prev_voxels):
                cumulative_growth_steps += 1
            if cumulative_prev_voxels:
                cumulative_retention_ratios.append(
                    len(cumulative_prev_voxels.intersection(voxels))
                    / float(len(cumulative_prev_voxels))
                )
        cumulative_prev_voxels = voxels
        for name, centroid in roi_centroids(points).items():
            static_centroids[name].append(centroid)
        update_cumulative_report()

    node.create_subscription(Odometry, "/nav/odometry", on_odom, 10)
    node.create_subscription(ROSPath, "/nav/global_path", on_global_path, 10)
    node.create_subscription(ROSPath, "/nav/local_path", on_local_path, 10)
    node.create_subscription(TwistStamped, "/nav/cmd_vel", on_cmd_vel, 10)
    node.create_subscription(PointCloud2, "/nav/map_cloud", on_map_cloud, 10)
    node.create_subscription(PointCloud2, "/nav/registered_cloud", on_registered_cloud, 10)
    node.create_subscription(PointCloud2, "/nav/terrain_map", on_terrain_map, 10)
    node.create_subscription(PointCloud2, "/nav/terrain_map_ext", on_terrain_map_ext, 10)
    node.create_subscription(
        PointCloud2,
        "/nav/cumulative_map_cloud",
        on_cumulative_map_cloud,
        10,
    )

    deadline = time.monotonic() + args.timeout_sec
    started = False
    next_costmap_tick = time.monotonic()
    next_trace_tick = time.monotonic()
    pass_seen_at: float | None = None
    pass_seen_known_cells = 0
    pass_seen_explored_area_m2 = 0.0
    pass_seen_frontier_count_max = 0
    stop_reason = "timeout"
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            for _ in range(args.drain_callbacks_per_tick):
                rclpy.spin_once(node, timeout_sec=0.0)
            now = time.monotonic()
            if current_pose is not None and result.lidar_map_updates > 0 and not started:
                status = explorer.begin_exploration()
                result.frontier_started = status in {"started", "already_running"}
                started = result.frontier_started
            if current_pose is not None and result.lidar_map_updates > 0 and now >= next_costmap_tick:
                deliver_frontier_inputs()
                next_costmap_tick = now + args.coverage_update_sec
            if (
                args.goal_republish_sec > 0.0
                and current_goal is not None
                and not result.global_path_seen
                and now - last_goal_publish_at >= args.goal_republish_sec
            ):
                publish_goal_pose(current_goal)
            update_sensor_map_metrics()
            if now >= next_trace_tick:
                capture_trace_snapshot()
                next_trace_tick = now + args.trace_interval_sec
            if (
                result.frontier_goal_published
                and result.frontier_goal_count >= args.min_frontier_goal_count
                and result.global_path_seen
                and result.local_path_seen
                and result.cmd_vel_nonzero
                and result.odom_delta_m >= args.min_odom_delta_m
                and result.lidar_map_updates >= args.min_lidar_map_updates
                and result.occupied_cells_final >= args.min_occupied_cells
                and result.free_cells_final >= args.min_free_cells
                and result.explored_area_delta_m2 >= args.min_explored_area_delta_m2
                and (not args.require_room_forward_exploration or room_forward_gate_ready())
                and (not args.require_cumulative_map or cumulative_gate_ready())
                and (not args.require_terrain_map_topics or terrain_topics_ready())
                and (not args.require_trajectory_quality or trajectory_gate_ready())
            ):
                if args.continue_after_pass_sec <= 0:
                    stop_reason = "pass_condition_met"
                    break
                if pass_seen_at is None:
                    pass_seen_at = now
                    pass_seen_known_cells = result.known_cells_final
                    pass_seen_explored_area_m2 = result.explored_area_final_m2
                    pass_seen_frontier_count_max = result.frontier_count_max
                elif now - pass_seen_at >= args.continue_after_pass_sec:
                    stop_reason = "post_pass_observation_elapsed"
                    break
    finally:
        explorer.end_exploration()
        explorer.stop()
        node.destroy_node()
        rclpy.shutdown()

    observed_after_pass_s = 0.0
    if pass_seen_at is not None:
        observed_after_pass_s = max(0.0, min(time.monotonic(), deadline) - pass_seen_at)
    result.frontier_no_gain_stall = {
        "checked": args.continue_after_pass_sec > 0.0,
        "ok": (
            args.continue_after_pass_sec > 0.0
            and pass_seen_at is not None
            and observed_after_pass_s >= args.continue_after_pass_sec
            and stop_reason == "post_pass_observation_elapsed"
        ),
        "mode": "post_pass_observation",
        "stop_reason": stop_reason,
        "required_observation_s": round(float(args.continue_after_pass_sec), 3),
        "observed_s": round(observed_after_pass_s, 3),
        "known_cells_at_pass": int(pass_seen_known_cells),
        "known_cells_final": int(result.known_cells_final),
        "known_cells_delta_after_pass": int(result.known_cells_final - pass_seen_known_cells),
        "explored_area_at_pass_m2": round(float(pass_seen_explored_area_m2), 4),
        "explored_area_final_m2": round(float(result.explored_area_final_m2), 4),
        "explored_area_delta_after_pass_m2": round(
            float(result.explored_area_final_m2 - pass_seen_explored_area_m2),
            4,
        ),
        "frontier_count_max_at_pass": int(pass_seen_frontier_count_max),
        "frontier_count_max_final": int(result.frontier_count_max),
    }

    if not result.frontier_started:
        result.errors.append("WavefrontFrontierExplorer did not start")
    if not result.frontier_goal_seen:
        result.errors.append("frontier goal was not produced")
    if not result.frontier_goal_published:
        result.errors.append("/nav/goal_pose was not published from frontier")
    if not result.odometry_seen:
        result.errors.append("/nav/odometry was not observed")
    if not result.map_cloud_seen:
        result.errors.append("/nav/map_cloud was not observed")
    if result.costmap_source != "gazebo_lidar_derived":
        result.errors.append(f"frontier costmap source is not Gazebo LiDAR: {result.costmap_source}")
    if result.lidar_map_updates < args.min_lidar_map_updates:
        result.errors.append(
            f"LiDAR occupancy updates {result.lidar_map_updates} < {args.min_lidar_map_updates}"
        )
    if result.raytrace_updates <= 0:
        result.errors.append("Gazebo LiDAR raytracing did not update the occupancy grid")
    update_cumulative_report()
    update_trajectory_quality_report()
    update_topic_sync_report()
    if args.require_terrain_map_topics:
        if not result.terrain_map_seen:
            result.errors.append("/nav/terrain_map was not observed")
        if not result.terrain_map_ext_seen:
            result.errors.append("/nav/terrain_map_ext was not observed")
        for topic in ("/nav/terrain_map", "/nav/terrain_map_ext"):
            samples = int(result.samples.get(topic) or 0)
            if samples < args.min_terrain_map_samples:
                result.errors.append(
                    f"{topic} samples {samples} < {args.min_terrain_map_samples}"
                )
        frames = result.topic_sync.get("frames") or {}
        if "odom" not in set(frames.get("terrain_map") or []):
            result.errors.append("/nav/terrain_map frame_id did not include odom")
        if "odom" not in set(frames.get("terrain_map_ext") or []):
            result.errors.append("/nav/terrain_map_ext frame_id did not include odom")
    if args.require_trajectory_quality:
        if result.trajectory_quality.get("ok") is not True:
            result.errors.append(
                "trajectory quality gate failed: "
                + json.dumps(result.trajectory_quality, ensure_ascii=False, sort_keys=True)
            )
        if result.topic_sync.get("ok") is not True:
            result.errors.append(
                "cloud/odom topic sync gate failed: "
                + json.dumps(result.topic_sync, ensure_ascii=False, sort_keys=True)
            )
    if args.require_cumulative_map:
        stats = result.cumulative_map_cloud
        registered = result.registered_cloud
        static = result.static_obstacles
        if not result.cumulative_map_cloud_seen:
            result.errors.append("/nav/cumulative_map_cloud was not observed")
        if int(stats.get("samples") or 0) < args.min_cumulative_samples:
            result.errors.append(
                "/nav/cumulative_map_cloud samples "
                f"{int(stats.get('samples') or 0)} < {args.min_cumulative_samples}"
            )
        if "odom" not in set(stats.get("frame_ids") or []):
            result.errors.append("/nav/cumulative_map_cloud frame_id did not include odom")
        point_delta = int(stats.get("point_count_delta") or 0)
        point_ratio = float(stats.get("point_growth_ratio") or 0.0)
        if (
            point_delta < args.min_cumulative_point_delta
            and point_ratio < args.min_cumulative_point_growth_ratio
        ):
            result.errors.append(
                "cumulative map point growth too small: "
                f"delta={point_delta}, ratio={point_ratio:.3f}"
            )
        voxel_delta = int(stats.get("unique_voxels_delta") or 0)
        voxel_ratio = float(stats.get("unique_voxel_growth_ratio") or 0.0)
        if (
            voxel_delta < args.min_cumulative_voxel_delta
            and voxel_ratio < args.min_cumulative_voxel_growth_ratio
        ):
            result.errors.append(
                "cumulative map voxel growth too small: "
                f"delta={voxel_delta}, ratio={voxel_ratio:.3f}"
            )
        if float(stats.get("growth_step_ratio") or 0.0) < args.min_cumulative_growth_step_ratio:
            result.errors.append(
                "cumulative map growth step ratio "
                f"{float(stats.get('growth_step_ratio') or 0.0):.3f} "
                f"< {args.min_cumulative_growth_step_ratio:.3f}"
            )
        if float(stats.get("retention_min") or 0.0) < args.min_cumulative_retention_ratio:
            result.errors.append(
                "cumulative map retention_min "
                f"{float(stats.get('retention_min') or 0.0):.3f} "
                f"< {args.min_cumulative_retention_ratio:.3f}"
            )
        if int(registered.get("samples") or 0) <= 0:
            result.errors.append("/nav/registered_cloud was not observed for negative control")
        elif (
            float(registered.get("map_vs_registered_voxel_ratio") or 0.0)
            < args.min_map_vs_registered_voxel_ratio
        ):
            result.errors.append(
                "cumulative map did not exceed registered single-frame cloud: "
                f"ratio={float(registered.get('map_vs_registered_voxel_ratio') or 0.0):.3f}"
            )
        stable_static = [
            name
            for name, item in static.items()
            if isinstance(item, dict)
            and int(item.get("samples") or 0) >= 2
            and float(item.get("centroid_drift_max_m") or 0.0)
            <= args.max_static_centroid_drift_m
        ]
        if len(stable_static) < args.min_stable_static_rois:
            result.errors.append(
                "stable cumulative-map static obstacle ROIs "
                f"{len(stable_static)} < {args.min_stable_static_rois}"
            )
    if not result.global_path_seen:
        result.errors.append("/nav/global_path with poses was not observed")
    if not result.local_path_seen:
        result.errors.append("/nav/local_path with poses was not observed")
    if not result.cmd_vel_seen:
        result.errors.append("/nav/cmd_vel was not observed")
    elif not result.cmd_vel_nonzero:
        result.errors.append("/nav/cmd_vel never became non-zero")
    if result.odom_delta_m < args.min_odom_delta_m:
        result.errors.append(
            f"/nav/odometry moved {result.odom_delta_m:.3f} m, "
            f"expected >= {args.min_odom_delta_m:.3f} m"
        )
    if result.explored_area_delta_m2 < args.min_explored_area_delta_m2:
        result.errors.append(
            f"explored area grew {result.explored_area_delta_m2:.3f} m^2, "
            f"expected >= {args.min_explored_area_delta_m2:.3f} m^2"
        )
    if result.free_cells_final < args.min_free_cells:
        result.errors.append(
            f"free occupancy cells {result.free_cells_final} < {args.min_free_cells}"
        )
    if result.occupied_cells_final < args.min_occupied_cells:
        result.errors.append(
            f"occupied occupancy cells {result.occupied_cells_final} < {args.min_occupied_cells}"
        )
    if result.frontier_count_max <= 0:
        result.errors.append("frontier candidates were not observed")
    if result.frontier_goal_count < args.min_frontier_goal_count:
        result.errors.append(
            f"frontier goals {result.frontier_goal_count} < {args.min_frontier_goal_count}"
        )
    if args.require_room_forward_exploration:
        if result.frontier_goal is None:
            result.errors.append("frontier goal missing for room-forward gate")
        else:
            if not result.frontier_goal_in_room:
                result.errors.append(
                    f"frontier goal {result.frontier_goal} is outside the Gazebo room bounds"
                )
            if result.frontier_goal[0] < args.min_frontier_goal_x:
                result.errors.append(
                    "frontier goal did not target the forward room: "
                    f"x={result.frontier_goal[0]:.3f}, expected >= {args.min_frontier_goal_x:.3f}"
                )
        if result.odom_last_xy is not None:
            x, y = result.odom_last_xy
            if not occupancy._inside_room(x, y):
                result.errors.append(
                    f"frontier odom ({x:.3f}, {y:.3f}) left Gazebo room bounds"
                )
        if result.odom_delta_x_m < args.min_frontier_odom_delta_x_m:
            result.errors.append(
                "frontier odom did not make forward x progress: "
                f"dx={result.odom_delta_x_m:.3f}, "
                f"expected >= {args.min_frontier_odom_delta_x_m:.3f}"
            )
    for label, z_range in (
        ("global_path", result.global_path_z_range),
        ("local_path", result.local_path_z_range),
    ):
        if z_range is None:
            continue
        z_abs = max(abs(float(z_range[0])), abs(float(z_range[1])))
        if z_abs > args.max_path_abs_z_m:
            result.errors.append(
                f"{label} z range {z_range} exceeds {args.max_path_abs_z_m:.3f} m"
            )
    write_map_artifacts()
    result.ok = not result.errors
    if args.trace_out:
        capture_trace_snapshot()
        trace_payload = {
            "schema_version": "lingtu.gazebo_frontier_trace.v1",
            "result": result.as_dict(),
            "occupancy_grid": {
                "source": result.mapping_source,
                "resolution_m": occupancy.resolution,
                "origin_x": occupancy.origin_x,
                "origin_y": occupancy.origin_y,
                "width": occupancy.width,
                "height": occupancy.height,
                "room_bounds": {
                    "min_x": occupancy.room_min_x,
                    "max_x": occupancy.room_max_x,
                    "min_y": occupancy.room_min_y,
                    "max_y": occupancy.room_max_y,
                },
            },
            "samples": trace,
        }
        path = Path(args.trace_out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(trace_payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout-sec", type=float, default=45.0)
    parser.add_argument("--coverage-size-m", type=float, default=12.0)
    parser.add_argument("--coverage-resolution-m", type=float, default=0.1)
    parser.add_argument("--robot-free-radius-m", type=float, default=0.35)
    parser.add_argument("--room-min-x", type=float, default=-0.95)
    parser.add_argument("--room-max-x", type=float, default=6.65)
    parser.add_argument("--room-min-y", type=float, default=-2.15)
    parser.add_argument("--room-max-y", type=float, default=2.15)
    parser.add_argument(
        "--reveal-radius-m",
        dest="robot_free_radius_m",
        type=float,
        default=argparse.SUPPRESS,
    )
    parser.add_argument("--coverage-update-sec", type=float, default=0.25)
    parser.add_argument("--frontier-goal-timeout-sec", type=float, default=8.0)
    parser.add_argument("--goal-republish-sec", type=float, default=0.5)
    parser.add_argument("--frontier-rate-hz", type=float, default=2.0)
    parser.add_argument("--min-frontier-size", type=int, default=3)
    parser.add_argument("--frontier-safe-distance-m", type=float, default=0.45)
    parser.add_argument("--min-frontier-goal-count", type=int, default=1)
    parser.add_argument("--goal-reached-radius-m", type=float, default=0.45)
    parser.add_argument("--min-cmd-vel", type=float, default=0.01)
    parser.add_argument("--min-odom-delta-m", type=float, default=0.05)
    parser.add_argument("--min-explored-area-delta-m2", type=float, default=0.15)
    parser.add_argument("--min-lidar-map-updates", type=int, default=3)
    parser.add_argument("--min-free-cells", type=int, default=20)
    parser.add_argument("--min-occupied-cells", type=int, default=8)
    parser.add_argument("--occupancy-min-z-m", type=float, default=0.05)
    parser.add_argument("--occupancy-max-z-m", type=float, default=1.45)
    parser.add_argument("--occupancy-min-range-m", type=float, default=0.25)
    parser.add_argument("--occupancy-max-range-m", type=float, default=6.5)
    parser.add_argument("--max-occupancy-cloud-points", type=int, default=50000)
    parser.add_argument("--max-occupancy-rays", type=int, default=2500)
    parser.add_argument("--trace-out", default="")
    parser.add_argument("--trace-interval-sec", type=float, default=0.35)
    parser.add_argument("--trace-max-known-points", type=int, default=1200)
    parser.add_argument("--continue-after-pass-sec", type=float, default=0.0)
    parser.add_argument("--require-room-forward-exploration", action="store_true")
    parser.add_argument("--min-frontier-goal-x", type=float, default=0.75)
    parser.add_argument("--min-frontier-odom-delta-x-m", type=float, default=0.25)
    parser.add_argument("--max-path-abs-z-m", type=float, default=0.15)
    parser.add_argument("--require-trajectory-quality", action="store_true")
    parser.add_argument("--min-trajectory-odom-samples", type=int, default=8)
    parser.add_argument("--max-trajectory-out-of-room-m", type=float, default=0.05)
    parser.add_argument("--max-room-violation-count", type=int, default=0)
    parser.add_argument("--max-odom-path-length-ratio", type=float, default=4.0)
    parser.add_argument("--max-trajectory-abs-y-m", type=float, default=2.05)
    parser.add_argument("--min-obstacle-clearance-m", type=float, default=0.18)
    parser.add_argument("--max-local-path-start-distance-m", type=float, default=1.2)
    parser.add_argument("--max-local-path-tracking-p95-m", type=float, default=1.25)
    parser.add_argument("--max-local-path-occupied-overlap-count", type=int, default=0)
    parser.add_argument("--max-local-path-out-of-room-count", type=int, default=0)
    parser.add_argument("--max-local-path-unknown-ratio", type=float, default=0.85)
    parser.add_argument(
        "--min-local-path-goal-alignment",
        type=float,
        default=-0.05,
        help=(
            "Minimum sustained local-path/frontier alignment. The gate checks "
            "the better of median and latest alignment so short avoidance or "
            "goal-switch transients remain diagnostic instead of failing the run."
        ),
    )
    parser.add_argument("--max-cloud-odom-skew-ms", type=float, default=250.0)
    parser.add_argument("--topic-sync-ignore-initial-samples", type=int, default=2)
    parser.add_argument("--min-topic-sync-samples", type=int, default=3)
    parser.add_argument("--drain-callbacks-per-tick", type=int, default=20)
    parser.add_argument("--require-terrain-map-topics", action="store_true")
    parser.add_argument("--min-terrain-map-samples", type=int, default=2)
    parser.add_argument("--require-cumulative-map", action="store_true")
    parser.add_argument("--max-cloud-points", type=int, default=200000)
    parser.add_argument("--min-cumulative-samples", type=int, default=8)
    parser.add_argument("--cumulative-voxel-size-m", type=float, default=0.2)
    parser.add_argument("--min-cumulative-point-growth-ratio", type=float, default=1.25)
    parser.add_argument("--min-cumulative-point-delta", type=int, default=1000)
    parser.add_argument("--min-cumulative-voxel-growth-ratio", type=float, default=1.2)
    parser.add_argument("--min-cumulative-voxel-delta", type=int, default=80)
    parser.add_argument("--min-cumulative-growth-step-ratio", type=float, default=0.6)
    parser.add_argument("--min-cumulative-retention-ratio", type=float, default=0.6)
    parser.add_argument("--min-map-vs-registered-voxel-ratio", type=float, default=1.5)
    parser.add_argument("--max-static-centroid-drift-m", type=float, default=0.35)
    parser.add_argument("--static-drift-window-samples", type=int, default=12)
    parser.add_argument("--min-static-roi-points", type=int, default=30)
    parser.add_argument("--min-stable-static-rois", type=int, default=3)
    parser.add_argument(
        "--static-roi-preset",
        choices=sorted(STATIC_OBSTACLE_ROI_PRESETS),
        default="demo_room",
    )
    parser.add_argument("--pcd-out", default="")
    parser.add_argument("--tomogram-out", default="")
    parser.add_argument("--build-tomogram", action="store_true")
    parser.add_argument("--min-map-artifact-points", type=int, default=500)
    parser.add_argument(
        "--map-artifact-source",
        choices=("occupancy_grid", "cumulative_cloud"),
        default="occupancy_grid",
    )
    parser.add_argument("--map-artifact-max-points", type=int, default=250000)
    parser.add_argument("--map-artifact-voxel-size-m", type=float, default=0.10)
    parser.add_argument("--map-artifact-min-z-m", type=float, default=-1.0)
    parser.add_argument("--map-artifact-max-z-m", type=float, default=3.0)
    parser.add_argument("--map-artifact-free-z-m", type=float, default=0.05)
    parser.add_argument("--map-artifact-obstacle-heights-m", default="0.35,0.85,1.25")
    parser.add_argument("--map-artifact-grid-stride", type=int, default=1)
    parser.add_argument("--tomogram-resolution", type=float, default=0.2)
    parser.add_argument("--tomogram-slice-dh", type=float, default=0.5)
    parser.add_argument("--tomogram-ground-h", type=float, default=0.0)
    parser.add_argument("--tomogram-kernel-size", type=int, default=7)
    parser.add_argument("--tomogram-interval-min", type=float, default=0.50)
    parser.add_argument("--tomogram-interval-free", type=float, default=0.65)
    parser.add_argument("--tomogram-slope-max", type=float, default=0.40)
    parser.add_argument("--tomogram-step-max", type=float, default=0.17)
    parser.add_argument("--tomogram-standable-ratio", type=float, default=0.20)
    parser.add_argument("--tomogram-cost-barrier", type=float, default=50.0)
    parser.add_argument("--tomogram-safe-margin", type=float, default=0.4)
    parser.add_argument("--tomogram-inflation", type=float, default=0.2)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--json-out", default="")
    args = parser.parse_args()

    try:
        result = run_smoke(args)
    except ImportError as exc:
        result = GazeboFrontierExplorationResult(errors=[f"dependencies unavailable: {exc}"])
    payload = json.dumps(result.as_dict(), ensure_ascii=False, indent=2)
    if args.json:
        print(payload)
    else:
        print(("PASSED" if result.ok else "FAILED") + ": LingTu Gazebo frontier exploration smoke")
        print(payload)
    if args.json_out:
        path = Path(args.json_out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(payload + "\n", encoding="utf-8")
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

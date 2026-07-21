"""Shared local planner backend data models."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from runtime.msgs.numpy_compat import np

PATH_NUM = 343
GROUP_NUM = 7
OBSTACLE_HEIGHT_THRE = 0.2
GROUND_HEIGHT_THRE = 0.1
POINT_PER_PATH_THRE = 2
PATH_RANGE = 3.5
DIR_THRE = 90.0
DIR_WEIGHT = 0.02


@dataclass
class NanobindLocalPlannerBackend:
    core: Any | None
    effective_params: dict[str, Any] = field(default_factory=dict)
    missing_params: list[str] = field(default_factory=list)
    paths_dir: str = ""
    unavailable_reason: str | None = None
    build_hint: str = ""


@dataclass
class CmuPyLocalPlannerBackend:
    nav_kernel: Any | None
    path_data: dict[str, Any]
    paths_dir: str


@dataclass(frozen=True)
class CmuPyLocalPlannerRequest:
    path_data: dict[str, Any]
    robot_pos: np.ndarray
    robot_yaw: float
    goal: np.ndarray
    obstacle_points_world: np.ndarray
    sensor_offset_x: float = 0.0
    sensor_offset_y: float = 0.0
    grid_voxel_size: float = 0.02
    grid_voxel_offset_x: float = 3.2
    grid_voxel_offset_y: float = 5.25
    grid_search_radius: float = 0.45
    vehicle_length: float = 0.6
    vehicle_width: float = 0.6
    obstacle_height_thre: float = OBSTACLE_HEIGHT_THRE
    ground_height_thre: float = GROUND_HEIGHT_THRE
    point_per_path_thre: int = POINT_PER_PATH_THRE
    dir_weight: float = DIR_WEIGHT
    dir_thre: float = DIR_THRE
    path_range: float = PATH_RANGE
    slope_weight: float = 0.0
    use_cost: bool = False
    check_rot_obstacle: bool = False
    two_way_drive: bool = True
    traversability_grid: np.ndarray | None = None
    traversability_resolution: float = 0.0
    traversability_origin: np.ndarray | None = None
    use_traversability_cost: bool = True
    traversability_hard_cost: float = 90.0
    traversability_soft_cost: float = 40.0
    traversability_weight: float = 0.01


@dataclass(frozen=True)
class CmuPyLocalPlannerDecision:
    world_path: list[tuple[float, float, float]]
    path_found: bool
    safety_stop: bool
    reason: str
    selected_group_id: int = -1
    selected_rot_dir: int = -1
    selected_path_group: int = -1
    score: float = 0.0
    obstacle_point_count: int = 0


@dataclass(frozen=True)
class LocalPlannerGridConfig:
    voxel_size: float = 0.02
    x_offset: float = 3.2
    y_offset: float = 5.25
    search_radius: float = 0.45
    loaded_from_config: bool = False

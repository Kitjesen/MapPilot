"""Payload and metadata contract for OctoPlanner3D."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from runtime.msgs.numpy_compat import np


SUPPORTED_MAP_FORMATS = (
    {
        "extension": ".bt",
        "format": "OctoMap binary",
        "source_kind": "octomap_file",
        "requires": [],
        "description": "OctoMap occupancy tree consumed directly by OctoPlanner3D",
    },
    {
        "extension": ".ot",
        "format": "OctoMap full tree",
        "source_kind": "octomap_file",
        "requires": [],
        "description": "Generic OctoMap tree loaded through octomap::AbstractOcTree",
    },
    {
        "extension": ".octomap",
        "format": "OctoMap tree",
        "source_kind": "octomap_file",
        "requires": [],
        "description": "Generic OctoMap tree loaded through octomap::AbstractOcTree",
    },
    {
        "extension": ".pcd",
        "format": "Point cloud",
        "source_kind": "point_cloud_file",
        "requires": ["PCL"],
        "description": "PCL point cloud converted to OctoMap by the headless adapter",
    },
)
SUPPORTED_MAP_EXTENSIONS = tuple(item["extension"] for item in SUPPORTED_MAP_FORMATS)
DEFAULT_PLANNER_CONSTRAINTS: dict[str, Any] = {
    "robot_radius": 0.25,
    "max_iterations": 800000,
    "snap_search_radius_cells": 12,
    "require_ground_support": True,
    "strict_direct_ground_support": False,
    "ground_support_xy_radius_cells": 1,
    "ground_support_depth_cells": 1,
    "enable_preblocked_costmap": True,
    "preblocked_costmap_radius_cells": 3,
    "preblocked_costmap_weight": 2.5,
    "lowest_traversable_only": False,
}
FLOAT_CONSTRAINT_KEYS = {"robot_radius", "preblocked_costmap_weight"}
INT_CONSTRAINT_KEYS = {
    "max_iterations",
    "snap_search_radius_cells",
    "ground_support_xy_radius_cells",
    "ground_support_depth_cells",
    "preblocked_costmap_radius_cells",
}
BOOL_CONSTRAINT_KEYS = {
    "require_ground_support",
    "strict_direct_ground_support",
    "enable_preblocked_costmap",
    "lowest_traversable_only",
}
PLANNER_INPUT_SCHEMA = {
    "map_source": {
        "type": "object",
        "required": True,
        "description": "Generic map source. Current native runtime consumes file sources and converts/loads them into OctoMap.",
        "fields": {
            "kind": {
                "type": "string",
                "allowed": ["octomap_file", "point_cloud_file"],
                "description": "Semantic source type, not tied to the host OS path syntax.",
            },
            "path": {
                "type": "string",
                "description": "Runtime-visible path. Windows paths are converted to WSL paths when the native executable runs in WSL.",
            },
            "format": {
                "type": "string",
                "allowed": ["bt", "ot", "octomap", "pcd", "auto"],
                "description": "Explicit map format or auto by extension.",
            },
            "frame": {
                "type": "string",
                "default": "map",
                "description": "Frame shared by map/start/goal/path.",
            },
        },
    },
    "map_path": {
        "type": "string",
        "required": True,
        "compatibility": "legacy alias for map_source.path",
        "accepted_extensions": list(SUPPORTED_MAP_EXTENSIONS),
        "description": "Runtime-visible map file path. Kept for backwards compatibility with existing LingTu planner service calls.",
    },
    "map_format": {
        "type": "string",
        "required": False,
        "allowed": ["bt", "ot", "octomap", "pcd", "auto"],
        "description": "Format hint; defaults to extension-based auto detection.",
    },
    "start": {
        "type": "array[3]float",
        "required": True,
        "frame": "LingTu planning/map frame",
        "description": "Start position [x, y, z] in meters",
    },
    "goal": {
        "type": "array[3]float",
        "required": True,
        "frame": "LingTu planning/map frame",
        "description": "Goal position [x, y, z] in meters",
    },
    "obstacle_thr": {
        "type": "float",
        "required": False,
        "compatibility": "legacy LingTu cost threshold",
        "description": "Carried for backend compatibility; native OctoMap occupancy, ground support, and collision checks drive planning.",
    },
    "options": {
        "type": "object",
        "required": False,
        "description": "Planner/converter options, including quadruped body envelope and terrain-support constraints.",
        "fields": {
            "planner_family": "octoplanner3d_constrained_global_planner",
            "search_algorithm": "octomap_3d_astar",
            "robot_radius": "Bounding-cylinder radius in meters used by OctoPlanner3D collision checks.",
            "max_iterations": "A* expansion budget.",
            "snap_search_radius_cells": "Grid-cell radius for snapping start/goal to traversable cells.",
            "require_ground_support": "Require occupied/support cells below traversable states.",
            "strict_direct_ground_support": "Require direct support below the body center.",
            "ground_support_xy_radius_cells": "Horizontal support search radius in cells.",
            "ground_support_depth_cells": "Vertical support search depth in cells.",
            "enable_preblocked_costmap": "Penalize cells above or near blocked support.",
            "preblocked_costmap_radius_cells": "Preblocked risk dilation radius in cells.",
            "preblocked_costmap_weight": "Additional cost weight for preblocked-risk cells.",
            "lowest_traversable_only": "Restrict candidate cells to the lowest traversable layer.",
        },
    },
}
PLANNER_OUTPUT_SCHEMA = {
    "planner": {"type": "string", "description": "Planner id, expected to be octoplanner3d"},
    "protocol_version": {"type": "integer", "description": "JSON protocol version emitted by the native wrapper/backend"},
    "ok": {"type": "bool", "description": "True only when the native planner returned a non-empty path"},
    "path": {
        "type": "array[array[3]float]",
        "frame": "LingTu planning/map frame",
        "description": "Global path waypoints [x, y, z] in meters",
    },
    "reached_goal": {"type": "bool", "description": "Whether final path point is within native goal tolerance"},
    "diagnostics": {
        "type": "object",
        "description": "Native planner details such as path point count, goal error, elapsed time, map source, build capabilities, and failure reason",
    },
}


def as_xyz(value: Sequence[float]) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size < 2:
        raise ValueError("point must contain at least x and y")
    z = float(arr[2]) if arr.size >= 3 else 0.0
    xyz = np.asarray([float(arr[0]), float(arr[1]), z], dtype=float)
    if not np.all(np.isfinite(xyz)):
        raise ValueError("point contains non-finite values")
    return xyz


def jsonable_point(point: np.ndarray) -> list[float]:
    return [float(point[0]), float(point[1]), float(point[2])]


def normalize_constraints(options: Mapping[str, Any] | None) -> dict[str, Any]:
    if not options:
        return {}

    normalized: dict[str, Any] = {}
    for key, value in options.items():
        if value is None:
            continue
        if key not in DEFAULT_PLANNER_CONSTRAINTS:
            raise ValueError(f"unknown OctoPlanner3D constraint: {key}")
        if key in BOOL_CONSTRAINT_KEYS:
            normalized[key] = _bool_constraint(value, key)
        elif key in INT_CONSTRAINT_KEYS:
            int_value = int(value)
            if key == "max_iterations" and int_value <= 0:
                raise ValueError("max_iterations must be positive")
            if key != "max_iterations" and int_value < 0:
                raise ValueError(f"{key} must be non-negative")
            normalized[key] = int_value
        elif key in FLOAT_CONSTRAINT_KEYS:
            float_value = float(value)
            if not np.isfinite(float_value):
                raise ValueError(f"{key} must be finite")
            if key == "robot_radius" and float_value <= 0.0:
                raise ValueError("robot_radius must be positive")
            if key == "preblocked_costmap_weight" and float_value < 0.0:
                raise ValueError("preblocked_costmap_weight must be non-negative")
            normalized[key] = float_value
    return normalized


def build_payload(
    *,
    runtime_map_path: str,
    start_xyz: np.ndarray,
    goal_xyz: np.ndarray,
    obstacle_thr: float,
    constraints: Mapping[str, Any],
    grid: Any | None = None,
    resolution: float = 0.2,
    origin: np.ndarray | None = None,
) -> dict[str, Any]:
    fmt = map_format(runtime_map_path)
    payload: dict[str, Any] = {
        "planner": "octoplanner3d",
        "protocol_version": 1,
        "map_path": runtime_map_path,
        "map_source": map_source(runtime_map_path, fmt),
        "map_format": fmt,
        "start": jsonable_point(start_xyz),
        "goal": jsonable_point(goal_xyz),
        "obstacle_thr": obstacle_thr,
        "options": {
            "obstacle_thr": obstacle_thr,
            "map_format": fmt,
            "planner_family": "octoplanner3d_constrained_global_planner",
            "search_algorithm": "octomap_3d_astar",
            "constraint_model": "quadruped_bounding_cylinder_ground_support",
            **dict(constraints),
        },
    }
    if grid is not None:
        grid_arr = np.asarray(grid)
        map_origin = np.asarray(origin if origin is not None else [0.0, 0.0], dtype=float)
        payload["map"] = {
            "resolution": float(resolution),
            "origin": [float(map_origin[0]), float(map_origin[1])],
            "shape": list(grid_arr.shape),
        }
    return payload


def map_source(runtime_map_path: str, fmt: str) -> dict[str, Any]:
    source_kind = "point_cloud_file" if fmt == "pcd" else "octomap_file"
    return {
        "kind": source_kind,
        "path": runtime_map_path,
        "format": fmt,
        "frame": "map",
    }


def map_format(path: str) -> str:
    suffix = Path(str(path or "")).suffix.lower().lstrip(".")
    if suffix in {"bt", "ot", "octomap", "pcd"}:
        return suffix
    return "auto"


def result_diagnostics(result: Mapping[str, Any]) -> dict[str, Any]:
    diagnostics = result.get("diagnostics")
    if isinstance(diagnostics, dict):
        return dict(diagnostics)
    return {}


def parse_path(raw_path: Any) -> list[np.ndarray]:
    if not isinstance(raw_path, list):
        return []
    path: list[np.ndarray] = []
    for item in raw_path:
        try:
            point = as_xyz(item)
        except (TypeError, ValueError):
            return []
        path.append(point)
    return path


def _bool_constraint(value: Any, key: str) -> bool:
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off"}:
            return False
        raise ValueError(f"{key} must be boolean")
    return bool(value)

"""Compile map service settings into its existing native launch environment."""

from __future__ import annotations

import math
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from runtime.yaml_helpers import load_yaml

from ..graph.settings import SYSTEM_SETTINGS_DIR

_FIELDS = {
    "publish": {
        "state_hz": "LINGTU_MAPD_STATE_HZ",
        "cloud_hz": "LINGTU_MAPD_CLOUD_HZ",
        "map_hz": "LINGTU_MAPD_MAP_HZ",
        "scene_hz": "LINGTU_MAPD_SCENE_HZ",
    },
    "input": {
        "max_points": "LINGTU_MAPD_MAX_POINTS",
        "min_range_m": "LINGTU_MAPD_MIN_RANGE_M",
        "max_range_m": "LINGTU_MAPD_MAX_RANGE_M",
        "stale_ms": "LINGTU_MAPD_STALE_MS",
    },
    "occupancy": {
        "resolution_m": "LINGTU_MAPD_OCCUPANCY_RESOLUTION_M",
        "size_x": "LINGTU_MAPD_OCCUPANCY_SIZE_X",
        "size_y": "LINGTU_MAPD_OCCUPANCY_SIZE_Y",
        "size_z": "LINGTU_MAPD_OCCUPANCY_SIZE_Z",
        "slide_m": "LINGTU_MAPD_OCCUPANCY_SLIDE_M",
        "ray_m": "LINGTU_MAPD_OCCUPANCY_RAY_M",
        "p_hit": "LINGTU_MAPD_OCCUPANCY_P_HIT",
        "p_miss": "LINGTU_MAPD_OCCUPANCY_P_MISS",
        "p_min": "LINGTU_MAPD_OCCUPANCY_P_MIN",
        "p_max": "LINGTU_MAPD_OCCUPANCY_P_MAX",
        "p_occ": "LINGTU_MAPD_OCCUPANCY_P_OCC",
        "decay_after_s": "LINGTU_MAPD_OCCUPANCY_DECAY_AFTER_S",
        "decay_ms": "LINGTU_MAPD_DECAY_MS",
    },
    "inflation": {
        "radius_m": "LINGTU_MAPD_INFLATION_RADIUS_M",
        "z_up_m": "LINGTU_MAPD_INFLATION_Z_UP_M",
        "z_down_m": "LINGTU_MAPD_INFLATION_Z_DOWN_M",
    },
    "surface": {
        "residual_m": "LINGTU_MAPD_SURFACE_RESIDUAL_M",
        "min_columns": "LINGTU_MAPD_SURFACE_MIN_COLUMNS",
    },
    "capacity": {
        "cloud_bytes": "LINGTU_MAPD_MAX_CLOUD_BYTES",
        "point_fields": "LINGTU_MAPD_MAX_POINT_FIELDS",
        "point_step": "LINGTU_MAPD_MAX_POINT_STEP",
        "string_bytes": "LINGTU_MAPD_MAX_STRING_BYTES",
        "scene_bytes": "LINGTU_MAPD_MAX_SCENE_BYTES",
        "voxel_snapshot_points": "LINGTU_MAPD_MAX_VOXEL_SNAPSHOT_POINTS",
        "voxel_snapshot_radius_m": "LINGTU_MAPD_VOXEL_SNAPSHOT_RADIUS_M",
        "voxels": "LINGTU_MAPD_MAX_VOXELS",
        "accumulated_cells": "LINGTU_MAPD_MAX_ACCUMULATED_CELLS",
        "accumulated_blocks": "LINGTU_MAPD_MAX_ACCUMULATED_BLOCKS",
        "query_json_bytes": "LINGTU_MAPD_QUERY_MAX_JSON_BYTES",
    },
    "save": {
        "patches": "LINGTU_MAP_SAVE_PATCHES",
        "build_octomap": "LINGTU_MAP_SAVE_BUILD_OCTOMAP",
        "octomap_resolution_m": "LINGTU_MAP_SAVE_OCTOMAP_RESOLUTION",
        "dynamic_filter": "LINGTU_SAVE_DYNAMIC_FILTER",
        "dynamic_filter_required": "LINGTU_SAVE_DYNAMIC_FILTER_REQUIRED",
        "dynamic_filter_command": "LINGTU_SAVE_DYNAMIC_FILTER_COMMAND",
        "dynamic_filter_timeout_s": "LINGTU_MAP_SAVE_DYNAMIC_FILTER_TIMEOUT_SEC",
        "profile": "LINGTU_PROFILE",
        "data_source": "LINGTU_DATA_SOURCE",
    },
}
_BOOLEAN = {"patches", "build_octomap", "dynamic_filter", "dynamic_filter_required"}
_TEXT = {"dynamic_filter_command", "profile", "data_source"}
_INTEGER = {
    "max_points", "stale_ms", "size_x", "size_y", "size_z", "decay_ms", "min_columns",
    "cloud_bytes", "point_fields", "point_step", "string_bytes", "scene_bytes",
    "voxel_snapshot_points", "voxels", "accumulated_cells", "accumulated_blocks", "query_json_bytes",
}
_NONNEGATIVE = {"min_range_m", "decay_after_s", "radius_m", "z_up_m", "z_down_m"}


def default_map_settings(directory: Path = SYSTEM_SETTINGS_DIR) -> Mapping[str, Any]:
    return load_yaml(directory / "maps.yaml")["maps"]


def occupancy_resolution(settings: Mapping[str, Any]) -> float:
    value = settings["occupancy"]["resolution_m"]
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value) or value <= 0:
        raise ValueError("maps.occupancy.resolution_m must be a finite positive number")
    return float(value)


def mapd_environment(settings: Mapping[str, Any], native_environment: Mapping[str, str]) -> dict[str, str]:
    """Validate the source settings once and bind the selected collision geometry."""
    if set(settings) != _FIELDS.keys():
        raise ValueError("maps must contain publish, input, occupancy, inflation, surface, capacity and save")
    environment: dict[str, str] = {}
    for group, fields in _FIELDS.items():
        values = settings[group]
        if not isinstance(values, Mapping) or set(values) != fields.keys():
            raise ValueError(f"maps.{group} has missing or unknown settings")
        for field, env_key in fields.items():
            value = values[field]
            name = f"maps.{group}.{field}"
            if field in _BOOLEAN:
                if not isinstance(value, bool):
                    raise ValueError(f"{name} must be a boolean")
                environment[env_key] = "1" if value else "0"
            elif field in _TEXT:
                if not isinstance(value, str) or (not value and field != "dynamic_filter_command"):
                    raise ValueError(f"{name} must be a string")
                environment[env_key] = value
            else:
                if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value):
                    raise ValueError(f"{name} must be a finite number")
                if field in _INTEGER and not isinstance(value, int):
                    raise ValueError(f"{name} must be an integer")
                if value < 0 or (value == 0 and field not in _NONNEGATIVE):
                    raise ValueError(f"{name} must be positive")
                if field.startswith("p_") and not 0 < value < 1:
                    raise ValueError(f"{name} must be in (0, 1)")
                environment[env_key] = format(value, ".15g")
    if settings["surface"]["min_columns"] < 3:
        raise ValueError("maps.surface.min_columns must be at least 3")
    if settings["input"]["min_range_m"] >= settings["input"]["max_range_m"]:
        raise ValueError("maps.input.min_range_m must be less than max_range_m")

    if native_environment.get("LINGTU_NAV_LOCAL_PLANNER_BACKEND") == "scan":
        radius = native_environment.get("LINGTU_NAV_COLLISION_CYLINDER_RADIUS_M")
        if radius is None:
            raise ValueError("SCAN requires LINGTU_NAV_COLLISION_CYLINDER_RADIUS_M")
        environment.update({
            "LINGTU_MAPD_INFLATION_RADIUS_M": radius,
            # Expanding obstacles into body-centre positions reverses the vertical extents.
            "LINGTU_MAPD_INFLATION_Z_UP_M": native_environment["LINGTU_NAV_COLLISION_CLEARANCE_BELOW_M"],
            "LINGTU_MAPD_INFLATION_Z_DOWN_M": native_environment["LINGTU_NAV_COLLISION_CLEARANCE_ABOVE_M"],
        })
    return environment

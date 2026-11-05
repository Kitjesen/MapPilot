"""Load shared source settings into the existing native configuration."""

from __future__ import annotations

import re
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import yaml

SYSTEM_SETTINGS_DIR = Path(__file__).resolve().parents[4] / "config/system/settings"

_GROUPS = {
    "cmu": {
        key: f"cmu_{key}"
        for key in (
            "ground_height_m",
            "cost_height_high_m",
            "cost_height_low_m",
            "points_per_path",
            "direction_weight",
            "direction_threshold_deg",
            "direction_to_vehicle",
            "path_scale",
            "min_path_scale",
            "path_scale_step",
            "scale_by_speed",
            "min_path_range_m",
            "path_range_step_m",
            "range_by_speed",
            "crop_by_goal",
            "goal_clear_range_m",
            "slow_path_count",
            "slow_group_count",
            "follower_lookahead_m",
            "follower_yaw_gain",
            "follower_nominal_dt_s",
            "follower_switch_time_s",
            "follower_heading_align_rad",
            "follower_omni_goal_m",
            "follower_omni_heading_rad",
            "follower_stop_distance_m",
            "follower_slowdown_distance_m",
        )
    },
    "teleop": {
        "takeover": "allow_teleop_takeover",
        "planner": "teleop_local_planner",
        "max_speed_mps": "teleop_max_speed_mps",
        "max_yaw_rate_rad_s": "teleop_max_yaw_rate_rad_s",
        "horizon_m": "teleop_planner_horizon_m",
        "max_deviation_deg": "teleop_planner_max_deviation_deg",
        **{
            key: f"teleop_{key}"
            for key in (
                "cmd_max_age_s", "min_motion_speed_mps",
                "traversability_hard_cost", "traversability_soft_cost",
            )
        },
    },
    "follower": {
        **{
            key: f"path_follower_{key}"
            for key in (
                "max_speed_mps", "max_accel_mps2", "max_yaw_accel_rad_s2",
                "max_yaw_rate_rad_s", "min_speed_mps", "goal_tolerance_m", "lookahead_m",
            )
        },
        "align_enter_rad": "path_follower_heading_align_enter_rad",
        "align_exit_rad": "path_follower_heading_align_exit_rad",
    },
    "global": {
        **{
            key: f"octoplanner3d_{key}"
            for key in (
                "max_iterations", "snap_search_radius_cells", "ground_support",
                "direct_support_only", "support_radius_cells", "support_depth_cells",
                "search_resolution_m", "surface_closing_radius_m", "support_height_m",
                "support_height_tolerance_m", "lowest_traversable_only", "floor_change_penalty",
                "max_slope", "same_floor_preference", "obstacle_clearance_radius_cells",
                "obstacle_clearance_weight", "support_patch_radius_cells", "support_patch_min_samples",
            )
        },
        "planner": "global_planner",
        "robot_radius_m": "octoplanner3d_robot_radius",
        "max_step_height_m": "octoplanner3d_max_step_height",
        "same_floor_z_tolerance_m": "octoplanner3d_same_floor_z_tolerance",
        "preblocked": "octoplanner3d_enable_preblocked_costmap",
        "preblocked_radius_cells": "octoplanner3d_preblocked_costmap_radius_cells",
        "preblocked_weight": "octoplanner3d_preblocked_costmap_weight",
        "terminal_tolerance_m": "octoplanner3d_terminal_goal_tolerance_m",
        "terminal_xy_tolerance_m": "octoplanner3d_terminal_goal_xy_tolerance_m",
        "terminal_z_tolerance_m": "octoplanner3d_terminal_goal_z_tolerance_m",
    },
}


class _Loader(yaml.SafeLoader):
    def construct_mapping(self, node: yaml.MappingNode, deep: bool = False) -> dict[Any, Any]:
        mapping = super().construct_mapping(node, deep=deep)
        if len(mapping) != len(node.value):
            raise ValueError("duplicate key in settings")
        return mapping


def _flatten(nav: Mapping[str, Any]) -> dict[str, Any]:
    result = dict(nav)
    for group, fields in _GROUPS.items():
        if group not in result:
            continue
        values = result.pop(group)
        if not isinstance(values, Mapping) or set(values) - fields.keys():
            raise ValueError(f"native_nav.{group} contains unknown settings or is not a mapping")
        for key, value in values.items():
            field = fields[key]
            if group == "teleop" and key in {"takeover", "planner"} and not isinstance(value, bool):
                raise ValueError(f"native_nav.{group}.{key} must be a boolean")
            if field in result:
                raise ValueError(f"native_nav.{group}.{key} duplicates {field}")
            result[field] = value
    return result


def _parameters(values: Mapping[str, Any]) -> dict[str, Any]:
    result = dict(values)
    for group in ("scan_planner", "scan_follower"):
        if group not in result:
            continue
        fields = result.pop(group)
        if not isinstance(fields, Mapping):
            raise ValueError(f"parameters.{group} must be a mapping")
        for key, value in fields.items():
            field = f"{group}.{key}"
            if field in result:
                raise ValueError(f"parameters.{field} is declared twice")
            result[field] = value
    return result


def load_settings(
    spec: dict[str, Any], directory: Path, *, standalone: bool = False
) -> None:
    """Expand shared files; standalone includes the defaults below Product overrides."""
    names = ()
    if "includes" in spec:
        names = spec.pop("includes")
        if not isinstance(names, list | tuple):
            raise ValueError("includes must be a list")
        if (
            not names
            or any(not isinstance(name, str) or not name or name != name.strip() for name in names)
            or len(set(names)) != len(names)
        ):
            raise ValueError("includes contains invalid or duplicate names")

    merged: dict[str, dict[str, Any]] = {}
    list_fields = {
        "topics", "capabilities", "topics_add", "capabilities_add",
        "processes_add", "optional_processes_add",
    }
    list_layers: dict[str, list[Any]] = {}
    for name in names:
        if re.fullmatch(r"[a-z][a-z0-9_]*", name) is None:
            raise ValueError(f"invalid includes name: {name!r}")
        path = directory / f"{name}.yaml"
        try:
            data = yaml.load(path.read_text(encoding="utf-8"), Loader=_Loader)
        except (OSError, ValueError, yaml.YAMLError) as exc:
            raise ValueError(f"cannot load settings {path}: {exc}") from exc
        if not isinstance(data, dict) or not data or set(data) - {"native_nav", "parameters", "maps", "camera", "lidar", "slam", "explore", "standalone", *list_fields}:
            raise ValueError(f"settings {path} must contain only native_nav or parameters, component settings or topic/capability/process lists")
        overrides = data.pop("standalone", {})
        if not isinstance(overrides, dict) or set(overrides) - {"native_nav", "parameters"}:
            raise ValueError(f"settings {path} standalone must contain only native_nav or parameters")
        for field in list_fields & data.keys():
            list_layers.setdefault(field, []).append(data.pop(field))
        for section, values in data.items():
            if not isinstance(values, dict):
                raise ValueError(f"settings {path} {section} must be a mapping")
            values = _flatten(values) if section == "native_nav" else _parameters(values) if section == "parameters" else values
            merged.setdefault(section, {}).update(values)
        if standalone:
            for section, values in overrides.items():
                if not isinstance(values, dict):
                    raise ValueError(f"settings {path} standalone.{section} must be a mapping")
                merged.setdefault(section, {}).update(
                    _flatten(values) if section == "native_nav" else _parameters(values)
                )

    if isinstance(spec.get("native_nav"), Mapping):
        spec["native_nav"] = _flatten(spec["native_nav"])
    if isinstance(spec.get("parameters"), Mapping):
        spec["parameters"] = _parameters(spec["parameters"])
    for section, values in merged.items():
        inline = spec.get(section, {})
        if not isinstance(inline, Mapping):
            raise ValueError(f"Product {section} must be a mapping")
        spec[section] = {**values, **inline}
    for field, layers in list_layers.items():
        layers.append(spec.get(field, []))
        if any(
            not isinstance(layer, list)
            or any(not isinstance(topic, str) or not topic or topic != topic.strip() for topic in layer)
            for layer in layers
        ):
            raise ValueError(f"settings {field} must be a list of nonempty names")
        spec[field] = list(dict.fromkeys(topic for layer in layers for topic in layer))

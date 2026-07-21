"""Local planner runtime parameter assembly."""

from __future__ import annotations

import logging
from typing import Any, Callable

from nav.local.models import (
    DIR_THRE,
    DIR_WEIGHT,
    GROUND_HEIGHT_THRE,
    LocalPlannerGridConfig,
    OBSTACLE_HEIGHT_THRE,
    PATH_RANGE,
    POINT_PER_PATH_THRE,
)

logger = logging.getLogger(__name__)


def read_local_planner_grid_config(
    *,
    config_getter: Callable[[], Any] | None = None,
) -> LocalPlannerGridConfig:
    """Read cmu_py grid parameters from runtime config, preserving CMU defaults."""
    defaults = LocalPlannerGridConfig()
    try:
        if config_getter is None:
            from runtime.config import get_config

            config_getter = get_config
        cfg = config_getter()
        raw = getattr(cfg, "raw", {}) or {}
        grid = raw.get("local_planner_grid", {}) or {}
        if not grid:
            return defaults
        return LocalPlannerGridConfig(
            voxel_size=float(grid.get("voxel_size", defaults.voxel_size)),
            x_offset=float(grid.get("x_offset", defaults.x_offset)),
            y_offset=float(grid.get("y_offset", defaults.y_offset)),
            search_radius=float(grid.get("search_radius", defaults.search_radius)),
            loaded_from_config=True,
        )
    except (ImportError, AttributeError, KeyError):
        return defaults


def read_local_planner_frame_params(
    *,
    config_getter: Callable[[], Any] | None = None,
) -> dict[str, float]:
    """Read the planner-frame offset used by the legacy ROS localPlanner."""
    try:
        if config_getter is None:
            from runtime.config import get_config

            config_getter = get_config
        cfg = config_getter()
        geom = getattr(cfg, "geometry", None)
        return {
            "sensor_offset_x": float(getattr(geom, "sensor_offset_x", 0.0)),
            "sensor_offset_y": float(getattr(geom, "sensor_offset_y", 0.0)),
        }
    except (ImportError, AttributeError, TypeError, ValueError):
        return {"sensor_offset_x": 0.0, "sensor_offset_y": 0.0}


def read_local_planner_python_params(
    *,
    config_getter: Callable[[], Any] | None = None,
) -> dict[str, Any]:
    """Read the CMU-python planner parameters that do not require nav_runtime."""
    defaults: dict[str, Any] = {
        "vehicle_length": 0.6,
        "vehicle_width": 0.6,
        "obstacle_height_thre": OBSTACLE_HEIGHT_THRE,
        "ground_height_thre": GROUND_HEIGHT_THRE,
        "point_per_path_thre": POINT_PER_PATH_THRE,
        "dir_weight": DIR_WEIGHT,
        "dir_thre": DIR_THRE,
        "path_range": PATH_RANGE,
        "slope_weight": 0.0,
        "use_cost": False,
        "check_rot_obstacle": False,
        "two_way_drive": True,
        "use_traversability_cost": True,
        "traversability_hard_cost": 90.0,
        "traversability_soft_cost": 40.0,
        "traversability_weight": 0.01,
    }

    try:
        if config_getter is None:
            from runtime.config import get_config

            config_getter = get_config
        cfg = config_getter()
        raw = getattr(cfg, "raw", {}) or {}
        lp = raw.get("local_planner", {}) or {}
        geom = getattr(cfg, "geometry", None)
        safety = getattr(cfg, "safety", None)
        params = dict(defaults)
        params["vehicle_length"] = float(
            getattr(geom, "vehicle_length", params["vehicle_length"])
        )
        params["vehicle_width"] = float(
            getattr(geom, "vehicle_width", params["vehicle_width"])
        )
        params["obstacle_height_thre"] = float(
            getattr(safety, "obstacle_height_thre", params["obstacle_height_thre"])
        )
        params["ground_height_thre"] = float(
            getattr(safety, "ground_height_thre", params["ground_height_thre"])
        )
        for name in (
            "point_per_path_thre",
            "dir_weight",
            "dir_thre",
            "path_range",
            "slope_weight",
            "use_cost",
            "check_rot_obstacle",
            "two_way_drive",
            "use_traversability_cost",
            "traversability_hard_cost",
            "traversability_soft_cost",
            "traversability_weight",
        ):
            if name in lp:
                params[name] = lp[name]
        params["point_per_path_thre"] = int(params["point_per_path_thre"])
        for name in (
            "dir_weight",
            "dir_thre",
            "path_range",
            "slope_weight",
            "traversability_hard_cost",
            "traversability_soft_cost",
            "traversability_weight",
        ):
            params[name] = float(params[name])
        for name in (
            "use_cost",
            "check_rot_obstacle",
            "two_way_drive",
            "use_traversability_cost",
        ):
            params[name] = bool(params[name])
        return params
    except (ImportError, AttributeError, TypeError, ValueError):
        return defaults


def build_local_planner_params(
    nav_kernel: Any,
    *,
    config_getter: Callable[[], Any] | None = None,
) -> tuple[Any, dict[str, Any], list[str]]:
    """Build and populate nav_kernel LocalPlannerParams."""
    params = nav_kernel.LocalPlannerParams()
    summary: dict[str, Any] = {}
    missing: list[str] = []

    def set_param(name: str, value: Any) -> None:
        if not hasattr(params, name):
            missing.append(name)
            return
        setattr(params, name, value)
        summary[name] = getattr(params, name)

    def current(name: str, fallback: Any) -> Any:
        return getattr(params, name, fallback)

    try:
        if config_getter is None:
            from runtime.config import get_config

            config_getter = get_config
        cfg = config_getter()
        raw = getattr(cfg, "raw", {}) or {}
        lp = raw.get("local_planner", {}) or {}
        ta = raw.get("terrain", {}) or {}
        geom = getattr(cfg, "geometry", None)
        safety = getattr(cfg, "safety", None)
        speed = getattr(cfg, "speed", None)

        set_param("vehicle_length", getattr(geom, "vehicle_length", current("vehicle_length", 0.6)))
        set_param("vehicle_width", getattr(geom, "vehicle_width", current("vehicle_width", 0.6)))
        set_param("sensor_offset_x", getattr(geom, "sensor_offset_x", current("sensor_offset_x", 0.0)))
        set_param("sensor_offset_y", getattr(geom, "sensor_offset_y", current("sensor_offset_y", 0.0)))
        set_param(
            "obstacle_height_thre",
            getattr(safety, "obstacle_height_thre", current("obstacle_height_thre", 0.2)),
        )
        set_param(
            "ground_height_thre",
            getattr(safety, "ground_height_thre", current("ground_height_thre", 0.1)),
        )
        set_param("max_speed", getattr(speed, "max_speed", current("max_speed", 1.0)))
        set_param("autonomy_speed", getattr(speed, "autonomy_speed", current("autonomy_speed", 1.0)))

        for name, default in (
            ("two_way_drive", True),
            ("adjacent_range", 3.5),
            ("check_obstacle", True),
            ("check_rot_obstacle", False),
            ("use_cost", False),
            ("use_terrain_analysis", True),
            ("point_per_path_thre", 2),
            ("dir_weight", 0.02),
            ("dir_thre", 90.0),
            ("path_scale", 1.0),
            ("min_path_scale", 0.75),
            ("path_scale_step", 0.25),
            ("path_scale_by_speed", True),
            ("min_path_range", 1.0),
            ("path_range_step", 0.5),
            ("path_range_by_speed", True),
            ("path_crop_by_goal", True),
            ("slope_weight", 0.0),
            ("goal_clear_range", 0.5),
            ("near_field_stop_dis", 0.5),
            ("recovery_blocked_thre", 2.0),
            ("recovery_rotate_time", 2.5),
            ("recovery_backup_time", 1.5),
            ("recovery_max_cycles", 3),
            ("cost_height_thre_1", 0.15),
            ("cost_height_thre_2", 0.1),
            ("dir_to_vehicle", False),
            ("goal_behind_range", 0.8),
            ("freeze_ang", 90.0),
            ("freeze_time", 2.0),
            ("omni_dir_goal_thre", 1.0),
            ("slow_path_num_thre", 5),
            ("slow_group_num_thre", 1),
            ("use_traversability_cost", True),
            ("traversability_hard_cost", 90.0),
            ("traversability_soft_cost", 40.0),
            ("traversability_weight", 0.01),
        ):
            set_param(name, lp.get(name, current(name, default)))

        set_param("min_rel_z", lp.get("min_rel_z", ta.get("min_rel_z", current("min_rel_z", -0.5))))
        set_param("max_rel_z", lp.get("max_rel_z", ta.get("max_rel_z", current("max_rel_z", 0.25))))
    except ImportError:
        logger.info("LocalPlanner [nanobind]: core config unavailable; using C++ defaults")

    return params, summary, missing

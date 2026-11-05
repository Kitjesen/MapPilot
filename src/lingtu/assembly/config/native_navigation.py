"""Compile explicit Product values for the native navigation endpoint."""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Mapping

from lingtu.assembly.graph.settings import SYSTEM_SETTINGS_DIR, load_settings

from .maps import default_map_settings, occupancy_resolution

_SCHEMA_VERSION = "lingtu.native_nav_config.v1"
_CONTROL_MODES = frozenset({"autonomy", "teleop", "teleop_avoid"})
_LEGACY_NAV_PARAMETER_KEYS = frozenset(
    {
        "final_waypoint_threshold",
        "waypoint_threshold",
        "path_follower_goal_tolerance",
        "path_follower_lookahead",
        "path_follower_max_speed",
        "path_follower_min_speed",
        "path_follower_max_accel",
    }
)
_GLOBAL_PLANNERS = frozenset({"octoplanner3d", "far"})
_LOCAL_PLANNERS = frozenset({"cmu", "scan"})
_RECOVERY_ACTIONS = frozenset({"translate", "rotate"})
NATIVE_NAV_ENVIRONMENT = {
    "control_mode": "LINGTU_NAV_CONTROL_MODE",
    "global_planner": "NAV_GLOBAL_PLANNER",
    "local_planner": "LINGTU_NAV_LOCAL_PLANNER_BACKEND",
    "publish_cmd_vel": "LINGTU_NAV_PUBLISH_CMD_VEL",
    "check_obstacle": "LINGTU_NAV_CHECK_OBSTACLE",
    "use_traversability_cost": "LINGTU_NAV_USE_TRAVERSABILITY_COST",
    "allow_teleop_takeover": "LINGTU_NAV_ALLOW_TELEOP_TAKEOVER",
    "teleop_local_planner": "LINGTU_TELEOP_LOCAL_PLANNER",
}
NATIVE_NAV_TEXT_FIELDS = frozenset({"control_mode", "global_planner", "local_planner"})
_TUNING_ENVIRONMENT = {
    "status_s": "LINGTU_NAV_STATUS_S",
    "stop_confirmation_timeout_s": "LINGTU_NAV_STOP_CONFIRMATION_TIMEOUT_S",
    "control_loop_deadline_miss_ratio_limit": "LINGTU_NAV_CONTROL_LOOP_DEADLINE_MISS_RATIO_LIMIT",
    "control_loop_p95_utilization_limit": "LINGTU_NAV_CONTROL_LOOP_P95_UTILIZATION_LIMIT",
    "localization_health_max_age_s": "LINGTU_NAV_LOCALIZATION_HEALTH_MAX_AGE_S",
    "driver_control_max_age_s": "LINGTU_DRIVER_CONTROL_MAX_AGE_S",
    "odom_max_age_s": "LINGTU_NAV_ODOM_MAX_AGE_S",
    "tf_max_age_s": "LINGTU_NAV_TF_MAX_AGE_S",
    "cloud_max_age_s": "LINGTU_NAV_CLOUD_MAX_AGE_S",
    "cloud_pose_max_gap_s": "LINGTU_NAV_CLOUD_POSE_MAX_GAP_S",
    "input_future_tolerance_s": "LINGTU_NAV_INPUT_FUTURE_TOLERANCE_S",
    "smoother_target_timeout_s": "LINGTU_NAV_SMOOTHER_TARGET_TIMEOUT_S",
    "smoother_feedback_timeout_s": "LINGTU_NAV_SMOOTHER_FEEDBACK_TIMEOUT_S",
    "smoother_max_step_s": "LINGTU_NAV_SMOOTHER_MAX_STEP_S",
    "smoother_future_tolerance_s": "LINGTU_NAV_SMOOTHER_FUTURE_TOLERANCE_S",
    "smoother_scale_velocities": "LINGTU_NAV_SMOOTHER_SCALE_VELOCITIES",
    "smoother_x_deadband_mps": "LINGTU_NAV_SMOOTHER_X_DEADBAND_MPS",
    "smoother_y_deadband_mps": "LINGTU_NAV_SMOOTHER_Y_DEADBAND_MPS",
    "smoother_yaw_deadband_rad_s": "LINGTU_NAV_SMOOTHER_YAW_DEADBAND_RADPS",
    "traversability_max_age_s": "LINGTU_NAV_TRAVERSABILITY_MAX_AGE_S",
    "terrain_map_max_age_s": "LINGTU_NAV_TERRAIN_MAP_MAX_AGE_S",
    "traversability_hard_cost": "LINGTU_NAV_TRAVERSABILITY_HARD_COST",
    "traversability_soft_cost": "LINGTU_NAV_TRAVERSABILITY_SOFT_COST",
    "traversability_weight": "LINGTU_NAV_TRAVERSABILITY_WEIGHT",
    "max_obstacle_points": "LINGTU_NAV_DDS_MAX_OBSTACLE_POINTS",
    "local_planner_threads": "LINGTU_NAV_LOCAL_PLANNER_THREADS",
    "local_planner_debug_candidates": "LINGTU_NAV_LOCAL_PLANNER_DEBUG_CANDIDATES",
    "local_map_debug_points": "LINGTU_NAV_LOCAL_MAP_DEBUG_POINTS",
    "live_obstacle_decay_s": "LINGTU_NAV_LIVE_OBSTACLE_DECAY_S",
    "live_obstacle_ray_clearing": "LINGTU_NAV_LIVE_OBSTACLE_RAY_CLEARING",
    "live_obstacle_ray_clear_max_range_m": "LINGTU_NAV_LIVE_OBSTACLE_RAY_CLEAR_MAX_RANGE_M",
    "live_obstacle_ray_clearing_interval_s": "LINGTU_NAV_LIVE_OBSTACLE_RAY_CLEARING_INTERVAL_S",
    "live_obstacle_max_clearing_rays": "LINGTU_NAV_LIVE_OBSTACLE_MAX_CLEARING_RAYS",
    "live_obstacle_min_hits": "LINGTU_NAV_LIVE_OBSTACLE_MIN_HITS",
    "teleop_cmd_max_age_s": "LINGTU_TELEOP_CMD_MAX_AGE_S",
    "teleop_min_motion_speed_mps": "LINGTU_TELEOP_MIN_MOTION_SPEED_MPS",
    "teleop_traversability_hard_cost": "LINGTU_TELEOP_TRAVERSABILITY_HARD_COST",
    "teleop_traversability_soft_cost": "LINGTU_TELEOP_TRAVERSABILITY_SOFT_COST",
}
_CMU_ENVIRONMENT = {
    field: f"LINGTU_NAV_{field.upper()}"
    for field in (
        "cmu_ground_height_m",
        "cmu_cost_height_high_m",
        "cmu_cost_height_low_m",
        "cmu_points_per_path",
        "cmu_direction_weight",
        "cmu_direction_threshold_deg",
        "cmu_direction_to_vehicle",
        "cmu_path_scale",
        "cmu_min_path_scale",
        "cmu_path_scale_step",
        "cmu_scale_by_speed",
        "cmu_min_path_range_m",
        "cmu_path_range_step_m",
        "cmu_range_by_speed",
        "cmu_crop_by_goal",
        "cmu_goal_clear_range_m",
        "cmu_slow_path_count",
        "cmu_slow_group_count",
        "cmu_follower_lookahead_m",
        "cmu_follower_yaw_gain",
        "cmu_follower_nominal_dt_s",
        "cmu_follower_switch_time_s",
        "cmu_follower_heading_align_rad",
        "cmu_follower_omni_goal_m",
        "cmu_follower_omni_heading_rad",
        "cmu_follower_stop_distance_m",
        "cmu_follower_slowdown_distance_m",
    )
}
_FAR_ENVIRONMENT = {
    field: f"LINGTU_NAV_{field.upper()}"
    for field in (
        "far_robot_radius_m", "far_obstacle_clearance_m",
        "far_max_visibility_distance_m", "far_unknown_cost_multiplier",
        "far_corner_separation_cells", "far_snap_search_radius_cells",
        "far_max_graph_nodes", "far_max_visibility_pairs",
        "far_max_search_expansions", "far_simplify_path",
    )
}
_TUNING_INTEGER_FIELDS = frozenset({
    "cmu_points_per_path",
    "cmu_slow_path_count",
    "cmu_slow_group_count",
    "max_obstacle_points", "local_planner_threads",
    "local_planner_debug_candidates", "local_map_debug_points",
    "live_obstacle_max_clearing_rays", "live_obstacle_min_hits",
    "far_corner_separation_cells", "far_snap_search_radius_cells",
    "far_max_graph_nodes", "far_max_visibility_pairs", "far_max_search_expansions",
})
_TUNING_BOOLEAN_FIELDS = frozenset({
    "cmu_direction_to_vehicle",
    "cmu_scale_by_speed",
    "cmu_range_by_speed",
    "cmu_crop_by_goal",
    "smoother_scale_velocities", "live_obstacle_ray_clearing", "far_simplify_path",
})
_OCTOPLANNER_ENVIRONMENT = {
    "octoplanner3d_robot_radius": "LINGTU_NAV_OCTO_ROBOT_RADIUS_M",
    "octoplanner3d_max_iterations": "LINGTU_NAV_OCTO_MAX_ITERATIONS",
    "octoplanner3d_snap_search_radius_cells": "LINGTU_NAV_OCTO_SNAP_RADIUS_CELLS",
    "octoplanner3d_ground_support": "LINGTU_NAV_OCTO_GROUND_SUPPORT",
    "octoplanner3d_direct_support_only": "LINGTU_NAV_OCTO_DIRECT_SUPPORT_ONLY",
    "octoplanner3d_support_radius_cells": "LINGTU_NAV_OCTO_SUPPORT_RADIUS_CELLS",
    "octoplanner3d_support_depth_cells": "LINGTU_NAV_OCTO_SUPPORT_DEPTH_CELLS",
    "octoplanner3d_support_patch_radius_cells": "LINGTU_NAV_OCTO_SUPPORT_PATCH_RADIUS_CELLS",
    "octoplanner3d_support_patch_min_samples": "LINGTU_NAV_OCTO_SUPPORT_PATCH_MIN_SAMPLES",
    "octoplanner3d_search_resolution_m": "LINGTU_NAV_OCTO_SEARCH_RESOLUTION_M",
    "octoplanner3d_surface_closing_radius_m": "LINGTU_NAV_OCTO_SURFACE_CLOSING_RADIUS_M",
    "octoplanner3d_support_height_m": "LINGTU_NAV_OCTO_SUPPORT_HEIGHT_M",
    "octoplanner3d_support_height_tolerance_m": "LINGTU_NAV_OCTO_SUPPORT_HEIGHT_TOLERANCE_M",
    "octoplanner3d_enable_preblocked_costmap": "LINGTU_NAV_OCTO_ENABLE_PREBLOCKED_COSTMAP",
    "octoplanner3d_preblocked_costmap_radius_cells": "LINGTU_NAV_OCTO_PREBLOCKED_RADIUS_CELLS",
    "octoplanner3d_preblocked_costmap_weight": "LINGTU_NAV_OCTO_PREBLOCKED_WEIGHT",
    "octoplanner3d_lowest_traversable_only": "LINGTU_NAV_OCTO_LOWEST_TRAVERSABLE_ONLY",
    "octoplanner3d_floor_change_penalty": "LINGTU_NAV_OCTO_FLOOR_CHANGE_PENALTY",
    "octoplanner3d_max_step_height": "LINGTU_NAV_OCTO_MAX_STEP_HEIGHT_M",
    "octoplanner3d_max_slope": "LINGTU_NAV_OCTO_MAX_SLOPE",
    "octoplanner3d_same_floor_preference": "LINGTU_NAV_OCTO_SAME_FLOOR_PREFERENCE",
    "octoplanner3d_same_floor_z_tolerance": "LINGTU_NAV_OCTO_SAME_FLOOR_Z_TOLERANCE_M",
    "octoplanner3d_obstacle_clearance_radius_cells": "LINGTU_NAV_OCTO_OBSTACLE_CLEARANCE_RADIUS_CELLS",
    "octoplanner3d_obstacle_clearance_weight": "LINGTU_NAV_OCTO_OBSTACLE_CLEARANCE_WEIGHT",
    "octoplanner3d_terminal_goal_tolerance_m": "LINGTU_NAV_OCTO_TERMINAL_GOAL_TOLERANCE_M",
    "octoplanner3d_terminal_goal_xy_tolerance_m": "LINGTU_NAV_OCTO_TERMINAL_GOAL_XY_TOLERANCE_M",
    "octoplanner3d_terminal_goal_z_tolerance_m": "LINGTU_NAV_OCTO_TERMINAL_GOAL_Z_TOLERANCE_M",
}


def local_planner_name(value: Any) -> str:
    """Return one supported local-planner backend name."""

    name = str(value or "").strip().lower()
    if name not in _LOCAL_PLANNERS:
        raise ValueError("local_planner must be cmu or scan")
    return name


def render_native_nav_environment(native_nav: Mapping[str, Any]) -> dict[str, str]:
    """Render the small structured navigation contract for process launch."""

    return {
        environment_key: (
            str(native_nav[field])
            if field in NATIVE_NAV_TEXT_FIELDS
            else _env_bool(bool(native_nav[field]))
        )
        for field, environment_key in NATIVE_NAV_ENVIRONMENT.items()
    }


def _finite_number(
    config: Mapping[str, Any],
    key: str,
    default: float | None = None,
    allow_negative: bool = False,
) -> float:
    value = config.get(key, default)
    if isinstance(value, bool):
        raise ValueError(f"{key} must be a finite number")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{key} must be a finite number") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"{key} must be a finite number")
    if not allow_negative and parsed < 0.0:
        raise ValueError(f"{key} must be non-negative")
    return parsed


def _env_number(value: float) -> str:
    return format(value, ".15g")


def _positive_integer(config: Mapping[str, Any], key: str) -> int:
    value = config.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError(f"{key} must be a positive integer")
    return value


def _bool(config: Mapping[str, Any], key: str, default: bool = False) -> bool:
    value = config.get(key, default)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in {"1", "true", "yes", "on"}:
            return True
        if text in {"0", "false", "no", "off"}:
            return False
        raise ValueError(f"{key} must be a boolean")
    return bool(value)


def _env_bool(value: bool) -> str:
    return "1" if value else "0"


def _octoplanner_environment(config: Mapping[str, Any]) -> dict[str, str]:
    return {
        env_name: _env_bool(value) if isinstance(value, bool) else str(value)
        for config_key, env_name in _OCTOPLANNER_ENVIRONMENT.items()
        for value in (config[config_key],)
    }


def _native_nav_mapping(config: Mapping[str, Any]) -> Mapping[str, Any]:
    native_nav = config.get("native_nav") or {}
    if not isinstance(native_nav, Mapping):
        raise ValueError("native_nav must be a mapping")
    return native_nav


def _recovery_order(value: Any) -> tuple[str, ...]:
    if isinstance(value, str):
        actions = tuple(part.strip().lower() for part in value.split(","))
    elif isinstance(value, (list, tuple)):
        actions = tuple(str(part).strip().lower() for part in value)
    else:
        raise ValueError("native_nav.recovery.behavior_order must be a sequence")
    if (
        not actions
        or any(action not in _RECOVERY_ACTIONS for action in actions)
        or len(set(actions)) != len(actions)
    ):
        raise ValueError(
            "native_nav.recovery.behavior_order must be a unique subset of translate,rotate"
        )
    return actions


def _recovery_config(
    native_nav_config: Mapping[str, Any], defaults: Mapping[str, Any], disabled_attempts: int
) -> dict[str, Any]:
    raw = native_nav_config.get("recovery") or {}
    if not isinstance(raw, Mapping):
        raise ValueError("native_nav.recovery must be a mapping")
    attempts = raw.get("max_attempts", disabled_attempts)
    if isinstance(attempts, bool) or not isinstance(attempts, int) or attempts < 0:
        raise ValueError("native_nav.recovery.max_attempts must be a non-negative integer")
    behavior_order = raw.get("behavior_order")
    raw = {**defaults, **raw}
    recovery = {
        "behavior_order": _recovery_order(
            defaults["behavior_order"] if behavior_order is None else behavior_order
        ),
        "blocked_interval_s": _finite_number(raw, "blocked_interval_s"),
        "rotation_timeout_s": _finite_number(raw, "rotation_timeout_s"),
        "translation_timeout_s": _finite_number(raw, "translation_timeout_s"),
        "max_attempts": attempts,
        # Recovery uses the same responsive product motion contract. A Product
        # may still provide a lower explicit value for a specialized mode.
        "translation_speed_mps": _finite_number(raw, "translation_speed_mps"),
        "rotation_rate_rad_s": _finite_number(raw, "rotation_rate_rad_s"),
        "min_rotation_rad": _finite_number(raw, "min_rotation_rad"),
        "max_rotation_rad": _finite_number(raw, "max_rotation_rad"),
        "rotation_candidate_step_rad": _finite_number(raw, "rotation_candidate_step_rad"),
        "rotation_sample_step_rad": _finite_number(raw, "rotation_sample_step_rad"),
    }
    if recovery["rotation_timeout_s"] <= 0.0:
        raise ValueError("native_nav.recovery.rotation_timeout_s must be positive")
    if recovery["translation_timeout_s"] <= 0.0:
        raise ValueError("native_nav.recovery.translation_timeout_s must be positive")
    if recovery["rotation_rate_rad_s"] <= 0.0:
        raise ValueError("native_nav.recovery.rotation_rate_rad_s must be positive")
    if recovery["min_rotation_rad"] <= 0.0:
        raise ValueError("native_nav.recovery.min_rotation_rad must be positive")
    if not recovery["min_rotation_rad"] <= recovery["max_rotation_rad"] <= math.pi:
        raise ValueError(
            "native_nav.recovery rotation range must satisfy 0 < min <= max <= pi"
        )
    if recovery["rotation_candidate_step_rad"] <= 0.0:
        raise ValueError(
            "native_nav.recovery.rotation_candidate_step_rad must be positive"
        )
    if not 0.0 < recovery["rotation_sample_step_rad"] <= recovery["min_rotation_rad"]:
        raise ValueError(
            "native_nav.recovery.rotation_sample_step_rad must be in (0, min_rotation_rad]"
        )
    return recovery


@dataclass(frozen=True)
class NativeNavConfig:
    """Validated native endpoint configuration."""

    product: str
    parameters: Mapping[str, float]
    native_nav: Mapping[str, Any]
    map_resolution_m: float | None = None
    tuning_environment: Mapping[str, str] = field(default_factory=dict)

    def limit_motion(self, max_linear_mps: float, max_angular_rps: float) -> NativeNavConfig:
        """Fit the Product budget inside the selected robot's physical limits."""
        parameters = dict(self.parameters)
        max_planar_mps = math.hypot(max_linear_mps, max_linear_mps)
        for name, limit in (
            ("teleop_max_speed_mps", max_linear_mps),
            ("teleop_max_yaw_rate_rad_s", max_angular_rps),
            ("path_follower_max_speed_mps", max_planar_mps),
            ("path_follower_max_yaw_rate_rad_s", max_angular_rps),
        ):
            parameters[name] = min(parameters[name], limit)
        parameters["path_follower_min_speed_mps"] = min(
            parameters["path_follower_min_speed_mps"], parameters["path_follower_max_speed_mps"]
        )
        recovery = dict(self.native_nav["recovery"])
        recovery["translation_speed_mps"] = min(recovery["translation_speed_mps"], max_planar_mps)
        recovery["rotation_rate_rad_s"] = min(recovery["rotation_rate_rad_s"], max_angular_rps)
        return replace(self, parameters=parameters, native_nav={**self.native_nav, "recovery": recovery})

    @property
    def environment(self) -> dict[str, str]:
        """Return the exact environment consumed by the native endpoint."""

        parameters = self.parameters
        native_nav = self.native_nav
        recovery = native_nav["recovery"]
        yaw_rate = parameters[
            "path_follower_max_yaw_rate_rad_s"
            if native_nav["control_mode"] == "autonomy"
            else "teleop_max_yaw_rate_rad_s"
        ]
        env = {
            **render_native_nav_environment(native_nav),
            **self.tuning_environment,
            "LINGTU_NAV_DDS_TICK_HZ": _env_number(parameters["tick_hz"]),
            "LINGTU_NAV_CORRIDOR_LOOKAHEAD_M": _env_number(parameters["corridor_lookahead_m"]),
            "LINGTU_NAV_GOAL_REACHED_M": _env_number(parameters["goal_reached_m"]),
            "LINGTU_NAV_GOAL_HEIGHT_TOLERANCE_M": _env_number(
                parameters["goal_height_tolerance_m"]
            ),
            "LINGTU_NAV_DYNAMIC_MIN_CELLS": str(parameters["dynamic_min_cells"]),
            "LINGTU_NAV_DYNAMIC_MIN_SPEED_MPS": _env_number(parameters["dynamic_min_speed_mps"]),
            "LINGTU_NAV_DYNAMIC_CONFIRM_FRAMES": str(parameters["dynamic_confirm_frames"]),
            "LINGTU_NAV_SMOOTHER_ENABLED": _env_bool(bool(native_nav["smoothing"])),
            "LINGTU_NAV_SMOOTHER_X_ACCEL_MPS2": _env_number(parameters["path_follower_max_accel_mps2"]),
            "LINGTU_NAV_SMOOTHER_Y_ACCEL_MPS2": _env_number(parameters["path_follower_max_accel_mps2"]),
            "LINGTU_NAV_SMOOTHER_X_DECEL_MPS2": _env_number(parameters["braking_deceleration_mps2"]),
            "LINGTU_NAV_SMOOTHER_Y_DECEL_MPS2": _env_number(parameters["braking_deceleration_mps2"]),
            # The smoother has independent body-axis limits. Keep both axes
            # at the product's 1 m/s operator budget instead of its legacy
            # lateral default of 0.5 m/s.
            "LINGTU_NAV_SMOOTHER_X_MIN_MPS": _env_number(-parameters["teleop_max_speed_mps"]),
            "LINGTU_NAV_SMOOTHER_X_MAX_MPS": _env_number(parameters["teleop_max_speed_mps"]),
            "LINGTU_NAV_SMOOTHER_Y_MIN_MPS": _env_number(-parameters["teleop_max_speed_mps"]),
            "LINGTU_NAV_SMOOTHER_Y_MAX_MPS": _env_number(parameters["teleop_max_speed_mps"]),
            "LINGTU_NAV_SMOOTHER_YAW_MIN_RADPS": _env_number(-yaw_rate),
            "LINGTU_NAV_SMOOTHER_YAW_MAX_RADPS": _env_number(yaw_rate),
            "LINGTU_NAV_SMOOTHER_YAW_ACCEL_RADPS2": _env_number(parameters["path_follower_max_yaw_accel_rad_s2"]),
            "LINGTU_NAV_SMOOTHER_YAW_DECEL_RADPS2": _env_number(parameters["path_follower_max_yaw_accel_rad_s2"]),
            "LINGTU_NAV_PATH_FOLLOWER_GOAL_TOLERANCE_M": _env_number(parameters["path_follower_goal_tolerance_m"]),
            "LINGTU_NAV_PATH_FOLLOWER_LOOKAHEAD_M": _env_number(parameters["path_follower_lookahead_m"]),
            "LINGTU_NAV_PATH_FOLLOWER_MAX_ACCEL_MPS2": _env_number(parameters["path_follower_max_accel_mps2"]),
            "LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS": _env_number(parameters["path_follower_max_speed_mps"]),
            "LINGTU_NAV_PATH_FOLLOWER_MIN_SPEED_MPS": _env_number(parameters["path_follower_min_speed_mps"]),
            "LINGTU_NAV_PATH_FOLLOWER_MAX_YAW_RATE_RAD_S": _env_number(parameters["path_follower_max_yaw_rate_rad_s"]),
            "LINGTU_NAV_PATH_FOLLOWER_MAX_YAW_ACCEL_RAD_S2": _env_number(
                parameters["path_follower_max_yaw_accel_rad_s2"]
            ),
            "LINGTU_NAV_PATH_FOLLOWER_HEADING_ALIGN_ENTER_RAD": _env_number(
                parameters["path_follower_heading_align_enter_rad"]
            ),
            "LINGTU_NAV_PATH_FOLLOWER_HEADING_ALIGN_EXIT_RAD": _env_number(
                parameters["path_follower_heading_align_exit_rad"]
            ),
            "LINGTU_NAV_RECOVERY_ORDER": ",".join(recovery["behavior_order"]),
            "LINGTU_NAV_RECOVERY_BLOCKED_INTERVAL_S": _env_number(recovery["blocked_interval_s"]),
            "LINGTU_NAV_RECOVERY_ROTATION_TIMEOUT_S": _env_number(recovery["rotation_timeout_s"]),
            "LINGTU_NAV_RECOVERY_TRANSLATION_TIMEOUT_S": _env_number(recovery["translation_timeout_s"]),
            "LINGTU_NAV_RECOVERY_MAX_ATTEMPTS": str(recovery["max_attempts"]),
            "LINGTU_NAV_RECOVERY_TRANSLATION_SPEED_MPS": _env_number(recovery["translation_speed_mps"]),
            "LINGTU_NAV_RECOVERY_ROTATION_RATE_RAD_S": _env_number(recovery["rotation_rate_rad_s"]),
            "LINGTU_NAV_RECOVERY_MIN_ROTATION_RAD": _env_number(recovery["min_rotation_rad"]),
            "LINGTU_NAV_RECOVERY_MAX_ROTATION_RAD": _env_number(recovery["max_rotation_rad"]),
            "LINGTU_NAV_RECOVERY_ROTATION_CANDIDATE_STEP_RAD": _env_number(recovery["rotation_candidate_step_rad"]),
            "LINGTU_NAV_RECOVERY_ROTATION_SAMPLE_STEP_RAD": _env_number(recovery["rotation_sample_step_rad"]),
            # Native endpoint ABI; the value is the active Product name.
            "LINGTU_PRODUCT": self.product,
            "LINGTU_TELEOP_PLANNER_HORIZON_M": _env_number(parameters["teleop_planner_horizon_m"]),
            "LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG": _env_number(parameters["teleop_planner_max_deviation_deg"]),
            "LINGTU_TELEOP_MAX_SPEED_MPS": _env_number(parameters["teleop_max_speed_mps"]),
            "LINGTU_TELEOP_MAX_YAW_RATE": _env_number(parameters["teleop_max_yaw_rate_rad_s"]),
            "LINGTU_TELEOP_OBSTACLE_MARGIN_M": _env_number(parameters["collision_hard_margin_m"]),
            "LINGTU_NAV_VEHICLE_LENGTH_M": _env_number(parameters["vehicle_length_m"]),
            "LINGTU_NAV_VEHICLE_WIDTH_M": _env_number(parameters["vehicle_width_m"]),
            "LINGTU_NAV_COLLISION_CYLINDER_RADIUS_M": _env_number(parameters["collision_cylinder_radius_m"]),
            "LINGTU_NAV_COLLISION_CYLINDER_OFFSET_M": _env_number(parameters["collision_cylinder_offset_m"]),
            "LINGTU_NAV_COLLISION_CLEARANCE_BELOW_M": _env_number(parameters["collision_clearance_below_m"]),
            "LINGTU_NAV_COLLISION_CLEARANCE_ABOVE_M": _env_number(parameters["collision_clearance_above_m"]),
            "LINGTU_NAV_SENSOR_OFFSET_X_M": _env_number(parameters["sensor_offset_x_m"]),
            "LINGTU_NAV_SENSOR_OFFSET_Y_M": _env_number(parameters["sensor_offset_y_m"]),
            "LINGTU_NAV_SENSOR_OFFSET_Z_M": _env_number(parameters["sensor_offset_z_m"]),
            "LINGTU_NAV_WAYPOINT_REACHED_M": _env_number(parameters["waypoint_reached_m"]),
        }
        env.update(_octoplanner_environment(self.native_nav["octoplanner3d"]))
        # Native route vertices are body poses in both envs. Ground-support
        # calibration must not disable the physical collision envelope.
        env["LINGTU_NAV_OCTO_BODY_CLEARANCE_BELOW_M"] = env["LINGTU_NAV_COLLISION_CLEARANCE_BELOW_M"]
        env["LINGTU_NAV_OCTO_BODY_CLEARANCE_ABOVE_M"] = env["LINGTU_NAV_COLLISION_CLEARANCE_ABOVE_M"]
        if self.map_resolution_m is not None:
            # Mapd rounds vertical inflation to whole cells. Global body
            # clearance must cover that same forbidden body-center volume.
            resolution = self.map_resolution_m
            env["LINGTU_NAV_SCAN_VOXEL_RESOLUTION_M"] = _env_number(resolution)
            for direction in ("BELOW", "ABOVE"):
                clearance = float(env[f"LINGTU_NAV_COLLISION_CLEARANCE_{direction}_M"])
                env[f"LINGTU_NAV_OCTO_BODY_CLEARANCE_{direction}_M"] = _env_number(
                    math.ceil(clearance / resolution) * resolution
                )
        return env

    def as_dict(self) -> dict[str, Any]:
        """Return deterministic JSON-ready configuration data."""

        return {
            "schema_version": _SCHEMA_VERSION,
            "product": self.product,
            "parameters": dict(self.parameters),
            "native_nav": dict(self.native_nav),
            "environment": self.environment,
        }


def compile_native_nav_config(
    product: str,
    config: Mapping[str, Any],
    *,
    settings_directory: Path = SYSTEM_SETTINGS_DIR,
) -> NativeNavConfig:
    """Compile native navigation parameters from one explicit Product contract."""

    product_name = str(product).strip()
    if not product_name:
        raise ValueError("native navigation Product must not be empty")
    explicit = _native_nav_mapping(config)
    settings = {"includes": ["control", "global", "local", "follower", "limits", "teleop", "recovery"]}
    load_settings(settings, settings_directory, standalone=True)
    defaults = settings["native_nav"]
    recovery_defaults = defaults.pop("recovery")
    disabled_attempts = defaults.pop("recovery_max_attempts")
    # Bare configurations keep these values tied to the selected mode or goal.
    del defaults["teleop_local_planner"], defaults["goal_height_tolerance_m"]
    native_nav_config = {**defaults, **explicit}
    if "scan_follower" in native_nav_config or "scan_planner" in native_nav_config:
        raise ValueError("SCAN tuning belongs in Product parameters: scan_follower.* and scan_planner.*")
    legacy_keys = sorted(
        (set(config) | set(native_nav_config)) & _LEGACY_NAV_PARAMETER_KEYS
    )
    if legacy_keys:
        raise ValueError(
            "legacy navigation parameters are unsupported: " + ", ".join(legacy_keys)
        )
    raw_control_mode = str(config.get("native_control_mode") or "").strip().lower()
    if not raw_control_mode:
        raise ValueError("native_control_mode must be declared by the compiled Product")
    if raw_control_mode not in _CONTROL_MODES:
        raise ValueError("native_control_mode must be autonomy, teleop, or teleop_avoid")
    global_planner = str(native_nav_config.get("global_planner") or defaults["global_planner"]).strip().lower()
    if global_planner not in _GLOBAL_PLANNERS:
        raise ValueError("native_nav.global_planner must be octoplanner3d or far")
    try:
        local_planner = local_planner_name(native_nav_config.get("local_planner") or defaults["local_planner"])
    except ValueError as exc:
        raise ValueError("native_nav.local_planner must be cmu or scan") from exc
    native_nav = {
        "control_mode": raw_control_mode,
        "global_planner": global_planner,
        "local_planner": local_planner,
        "publish_cmd_vel": _bool(native_nav_config, "publish_cmd_vel"),
        "check_obstacle": _bool(
            native_nav_config,
            "check_obstacle",
            raw_control_mode != "teleop",
        ),
        "use_traversability_cost": _bool(
            native_nav_config,
            "use_traversability_cost",
            False,
        ),
        "allow_teleop_takeover": _bool(
            native_nav_config,
            "allow_teleop_takeover",
            False,
        ),
        "teleop_local_planner": _bool(
            native_nav_config,
            "teleop_local_planner",
            raw_control_mode == "teleop_avoid",
        ),
        "smoothing": _bool(native_nav_config, "smoothing"),
        "recovery": _recovery_config(native_nav_config, recovery_defaults, disabled_attempts),
    }
    if raw_control_mode == "teleop" and (native_nav["check_obstacle"] or native_nav["use_traversability_cost"]):
        raise ValueError("native_nav teleop mode must not enable obstacle or traversability checks")
    if raw_control_mode == "teleop_avoid" and (
        not native_nav["check_obstacle"]
        or not native_nav["teleop_local_planner"]
    ):
        raise ValueError(
            "native_nav teleop_avoid must enable obstacle checks and teleop local planner"
        )
    # SCAN consumes Mapd's 3D collision volume, not the CMU terrain-cost grid.
    if local_planner == "scan":
        native_nav["use_traversability_cost"] = False
    vehicle_length_m = _finite_number(config, "vehicle_length_m", 1.0)
    vehicle_width_m = _finite_number(config, "vehicle_width_m", 0.6)
    vehicle_height_m = _finite_number(config, "vehicle_height_m", 0.5)
    defaults["octoplanner3d_robot_radius"] = 0.5 * vehicle_width_m
    native_nav["octoplanner3d"] = {
        key: explicit.get(key, config.get(key, defaults[key]))
        for key in _OCTOPLANNER_ENVIRONMENT
    }
    default_cylinder_radius_m = math.hypot(0.25 * vehicle_length_m, 0.5 * vehicle_width_m)
    path_follower_goal_tolerance_m = _finite_number(native_nav_config, "path_follower_goal_tolerance_m")
    goal_reached_m = _finite_number(native_nav_config, "goal_reached_m")
    goal_height_tolerance_m = _finite_number(
        native_nav_config,
        "goal_height_tolerance_m",
        goal_reached_m,
    )
    if goal_height_tolerance_m <= 0.0:
        raise ValueError("native_nav.goal_height_tolerance_m must be positive")
    teleop_max_speed_mps = _finite_number(native_nav_config, "teleop_max_speed_mps")
    # Assisted input limits each body axis; planning limits the velocity norm.
    default_path_speed_mps = defaults["path_follower_max_speed_mps"]
    if raw_control_mode == "teleop_avoid":
        default_path_speed_mps = min(
            default_path_speed_mps, math.hypot(teleop_max_speed_mps, teleop_max_speed_mps)
        )
    parameters = {
        "tick_hz": _finite_number(native_nav_config, "tick_hz"),
        "corridor_lookahead_m": _finite_number(native_nav_config, "corridor_lookahead_m"),
        "dynamic_min_cells": _positive_integer(native_nav_config, "dynamic_min_cells"),
        "dynamic_min_speed_mps": _finite_number(native_nav_config, "dynamic_min_speed_mps"),
        "dynamic_confirm_frames": _positive_integer(native_nav_config, "dynamic_confirm_frames"),
        "goal_reached_m": goal_reached_m,
        "goal_height_tolerance_m": goal_height_tolerance_m,
        "path_follower_goal_tolerance_m": path_follower_goal_tolerance_m,
        "path_follower_lookahead_m": _finite_number(native_nav_config, "path_follower_lookahead_m"),
        "path_follower_max_accel_mps2": _finite_number(native_nav_config, "path_follower_max_accel_mps2"),
        "braking_deceleration_mps2": _finite_number(native_nav_config, "braking_deceleration_mps2"),
        "path_follower_max_speed_mps": _finite_number(
            explicit, "path_follower_max_speed_mps", default_path_speed_mps
        ),
        "path_follower_min_speed_mps": _finite_number(native_nav_config, "path_follower_min_speed_mps"),
        "path_follower_max_yaw_rate_rad_s": _finite_number(native_nav_config, "path_follower_max_yaw_rate_rad_s"),
        "path_follower_max_yaw_accel_rad_s2": _finite_number(native_nav_config, "path_follower_max_yaw_accel_rad_s2"),
        "path_follower_heading_align_enter_rad": _finite_number(native_nav_config, "path_follower_heading_align_enter_rad"),
        "path_follower_heading_align_exit_rad": _finite_number(native_nav_config, "path_follower_heading_align_exit_rad"),
        "waypoint_reached_m": _finite_number(native_nav_config, "waypoint_reached_m"),
        "teleop_planner_horizon_m": _finite_number(native_nav_config, "teleop_planner_horizon_m"),
        "teleop_planner_max_deviation_deg": _finite_number(native_nav_config, "teleop_planner_max_deviation_deg"),
        "teleop_max_speed_mps": teleop_max_speed_mps,
        "teleop_max_yaw_rate_rad_s": _finite_number(native_nav_config, "teleop_max_yaw_rate_rad_s"),
        "vehicle_length_m": vehicle_length_m,
        "vehicle_width_m": vehicle_width_m,
        "collision_hard_margin_m": _finite_number(
            config, "collision_hard_margin_m", 0.08
        ),
        "collision_cylinder_radius_m": _finite_number(
            config, "collision_cylinder_radius_m", default_cylinder_radius_m
        ),
        "collision_cylinder_offset_m": _finite_number(
            config, "collision_cylinder_offset_m", 0.25 * vehicle_length_m
        ),
        "collision_clearance_below_m": _finite_number(
            config, "collision_clearance_below_m", 0.5 * vehicle_height_m
        ),
        "collision_clearance_above_m": _finite_number(
            config, "collision_clearance_above_m", 0.5 * vehicle_height_m
        ),
        "sensor_offset_x_m": _finite_number(config, "sensor_offset_x_m", 0.0, allow_negative=True),
        "sensor_offset_y_m": _finite_number(config, "sensor_offset_y_m", 0.0, allow_negative=True),
        "sensor_offset_z_m": _finite_number(config, "sensor_offset_z_m", 0.0, allow_negative=True),
    }
    feedback_mode = native_nav_config["smoother_feedback_mode"]
    if feedback_mode not in ("open", "closed"):
        raise ValueError("native_nav.smoother_feedback_mode must be open or closed")
    obstacle_height_min = _finite_number(native_nav_config, "local_planner_obstacle_height_min_m")
    obstacle_height_max = _finite_number(native_nav_config, "local_planner_obstacle_height_max_m")
    if obstacle_height_max < obstacle_height_min:
        raise ValueError("local planner obstacle height maximum must be at least the minimum")
    tuning_environment = {
        "LINGTU_NAV_SMOOTHER_FEEDBACK_MODE": feedback_mode,
        "LINGTU_NAV_LOCAL_PLANNER_OBSTACLE_HEIGHT_MIN_M": _env_number(obstacle_height_min),
        "LINGTU_NAV_LOCAL_PLANNER_OBSTACLE_HEIGHT_MAX_M": _env_number(obstacle_height_max),
    }
    if local_planner == "cmu":
        tuning_environment["LINGTU_NAV_OBSTACLE_VOXEL_SIZE_M"] = _env_number(
            _finite_number(native_nav_config, "obstacle_voxel_size_m")
        )
    else:
        tuning_environment["LINGTU_NAV_SCAN_MOTION_VOXEL_SIZE_M"] = _env_number(
            _finite_number(native_nav_config, "scan_motion_voxel_size_m")
        )
    tuning_fields = dict(_TUNING_ENVIRONMENT)
    if global_planner == "far":
        tuning_fields.update(_FAR_ENVIRONMENT)
    if local_planner == "cmu":
        tuning_fields.update(_CMU_ENVIRONMENT)
    for key, env_key in tuning_fields.items():
        if key in _TUNING_BOOLEAN_FIELDS:
            value = native_nav_config[key]
            if not isinstance(value, bool):
                raise ValueError(f"{key} must be a boolean")
            tuning_environment[env_key] = _env_bool(value)
        elif key in _TUNING_INTEGER_FIELDS:
            value = native_nav_config[key]
            if type(value) is not int or value < 0:
                raise ValueError(f"{key} must be a non-negative integer")
            tuning_environment[env_key] = str(value)
        else:
            tuning_environment[env_key] = _env_number(_finite_number(native_nav_config, key))
    if local_planner == "scan":
        # Global XYZ search finds a corridor with the local body's width.
        # SCAN checks body length, heading and executable motion with both
        # cylinders; adding their offset here would require room to turn
        # everywhere along a straight corridor.
        native_nav["octoplanner3d"]["octoplanner3d_robot_radius"] = max(
            native_nav["octoplanner3d"]["octoplanner3d_robot_radius"],
            parameters["collision_cylinder_radius_m"],
        )
    if parameters["teleop_planner_horizon_m"] < 0.5:
        raise ValueError("teleop_planner_horizon_m must be at least 0.5")
    if parameters["tick_hz"] <= 0.0:
        raise ValueError("native_nav.tick_hz must be positive")
    if local_planner == "scan" and not math.isclose(
        parameters["tick_hz"], 100.0, rel_tol=0.0, abs_tol=1e-9
    ):
        raise ValueError("native_nav.tick_hz must be 100 when local_planner is scan")
    if parameters["dynamic_confirm_frames"] < 2:
        raise ValueError("dynamic_confirm_frames must be at least 2")
    if parameters["teleop_planner_max_deviation_deg"] > 90.0:
        raise ValueError("teleop_planner_max_deviation_deg must not exceed 90")
    if parameters["teleop_max_speed_mps"] <= 0.0:
        raise ValueError("teleop_max_speed_mps must be positive")
    if parameters["braking_deceleration_mps2"] <= 0.0:
        raise ValueError("braking_deceleration_mps2 must be positive")
    if parameters["teleop_max_yaw_rate_rad_s"] <= 0.0:
        raise ValueError("teleop_max_yaw_rate_rad_s must be positive")
    if parameters["vehicle_length_m"] <= 0.0:
        raise ValueError("vehicle_length_m must be positive")
    if parameters["vehicle_width_m"] <= 0.0:
        raise ValueError("vehicle_width_m must be positive")
    if parameters["collision_cylinder_radius_m"] <= 0.0:
        raise ValueError("collision_cylinder_radius_m must be positive")
    if parameters["collision_hard_margin_m"] < 0.0:
        raise ValueError("collision_hard_margin_m must be non-negative")
    if parameters["path_follower_min_speed_mps"] > parameters["path_follower_max_speed_mps"]:
        raise ValueError("path_follower_min_speed must not exceed path_follower_max_speed")
    if parameters["path_follower_goal_tolerance_m"] > parameters["goal_reached_m"]:
        raise ValueError("path_follower_goal_tolerance must not exceed final_waypoint_threshold")
    if not 0.0 < parameters["path_follower_heading_align_enter_rad"] <= math.pi:
        raise ValueError(
            "path_follower_heading_align_enter_rad must be in (0, pi]"
        )
    if not 0.0 <= parameters["path_follower_heading_align_exit_rad"] < parameters[
        "path_follower_heading_align_enter_rad"
    ]:
        raise ValueError(
            "path_follower_heading_align_exit_rad must be below the enter angle"
        )
    resolution = None
    if native_nav["local_planner"] == "scan":
        maps = config.get("maps")
        resolution = occupancy_resolution(default_map_settings(settings_directory) if maps is None else maps)
    return NativeNavConfig(product_name, parameters, native_nav, resolution, tuning_environment)


__all__ = [
    "NativeNavConfig",
    "compile_native_nav_config",
    "local_planner_name",
]

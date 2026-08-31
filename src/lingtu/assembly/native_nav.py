"""Compile explicit Product values for the native navigation endpoint."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping

_SCHEMA_VERSION = "lingtu.native_nav_config.v1"
_CONTROL_MODES = frozenset({"autonomy", "teleop", "teleop_avoid"})
_GLOBAL_PLANNERS = frozenset({"octoplanner3d", "far"})
_LOCAL_PLANNERS = frozenset({"cmu", "scan"})
_RECOVERY_ACTIONS = frozenset({"translate", "rotate"})
_RECOVERY_DEFAULT_ORDER = ("translate", "rotate")
_OCTOPLANNER_CONFIG = {
    "octoplanner3d_robot_radius": ("LINGTU_NAV_OCTO_ROBOT_RADIUS_M", 0.25),
    "octoplanner3d_max_iterations": ("LINGTU_NAV_OCTO_MAX_ITERATIONS", 500000),
    "octoplanner3d_snap_search_radius_cells": ("LINGTU_NAV_OCTO_SNAP_RADIUS_CELLS", 24),
    "octoplanner3d_require_ground_support": ("LINGTU_NAV_OCTO_REQUIRE_GROUND_SUPPORT", True),
    "octoplanner3d_strict_direct_ground_support": ("LINGTU_NAV_OCTO_STRICT_GROUND_SUPPORT", True),
    "octoplanner3d_ground_support_xy_radius_cells": ("LINGTU_NAV_OCTO_GROUND_SUPPORT_XY_RADIUS_CELLS", 0),
    "octoplanner3d_ground_support_depth_cells": ("LINGTU_NAV_OCTO_GROUND_SUPPORT_DEPTH_CELLS", 2),
    "octoplanner3d_enable_preblocked_costmap": ("LINGTU_NAV_OCTO_ENABLE_PREBLOCKED_COSTMAP", True),
    "octoplanner3d_preblocked_costmap_radius_cells": ("LINGTU_NAV_OCTO_PREBLOCKED_RADIUS_CELLS", 3),
    "octoplanner3d_preblocked_costmap_weight": ("LINGTU_NAV_OCTO_PREBLOCKED_WEIGHT", 2.5),
    "octoplanner3d_lowest_traversable_only": ("LINGTU_NAV_OCTO_LOWEST_TRAVERSABLE_ONLY", False),
    "octoplanner3d_floor_change_penalty": ("LINGTU_NAV_OCTO_FLOOR_CHANGE_PENALTY", 6.0),
    "octoplanner3d_max_step_height": ("LINGTU_NAV_OCTO_MAX_STEP_HEIGHT_M", 0.45),
    "octoplanner3d_max_slope": ("LINGTU_NAV_OCTO_MAX_SLOPE", 0.0),
    "octoplanner3d_same_floor_preference": ("LINGTU_NAV_OCTO_SAME_FLOOR_PREFERENCE", True),
    "octoplanner3d_same_floor_z_tolerance": ("LINGTU_NAV_OCTO_SAME_FLOOR_Z_TOLERANCE_M", 0.75),
    "octoplanner3d_max_same_floor_z_excursion": ("LINGTU_NAV_OCTO_MAX_SAME_FLOOR_Z_EXCURSION_M", 2.0),
    "octoplanner3d_obstacle_clearance_radius_cells": ("LINGTU_NAV_OCTO_OBSTACLE_CLEARANCE_RADIUS_CELLS", 2),
    "octoplanner3d_obstacle_clearance_weight": ("LINGTU_NAV_OCTO_OBSTACLE_CLEARANCE_WEIGHT", 1.5),
}

def local_planner_name(value: Any) -> str:
    """Return one supported local-planner backend name."""

    name = str(value or "").strip().lower()
    if name not in _LOCAL_PLANNERS:
        raise ValueError("local_planner must be cmu or scan")
    return name


def mapd_environment(local_planner: Any) -> dict[str, str]:
    """Return the Mapd collision profile required by one local planner."""

    if str(local_planner or "").strip().lower() != "scan":
        return {}
    return {
        "LINGTU_MAPD_OCCUPANCY_RESOLUTION_M": "0.05",
        "LINGTU_MAPD_OCCUPANCY_SIZE_X": "200",
        "LINGTU_MAPD_OCCUPANCY_SIZE_Y": "200",
        "LINGTU_MAPD_OCCUPANCY_SIZE_Z": "100",
        "LINGTU_MAPD_OCCUPANCY_SLIDE_M": "0.2",
        "LINGTU_MAPD_OCCUPANCY_RAY_M": "5.0",
        "LINGTU_MAPD_OCCUPANCY_P_HIT": "0.85",
        "LINGTU_MAPD_OCCUPANCY_P_MISS": "0.30",
        "LINGTU_MAPD_OCCUPANCY_P_MIN": "0.12",
        "LINGTU_MAPD_OCCUPANCY_P_MAX": "0.98",
        "LINGTU_MAPD_OCCUPANCY_P_OCC": "0.80",
        "LINGTU_MAPD_MAX_COLLISION_SNAPSHOT_POINTS": "4000000",
        "LINGTU_MAPD_MAX_CLOUD_BYTES": "67108864",
    }


def _finite_number(
    config: Mapping[str, Any],
    key: str,
    default: float,
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


def _positive_integer(config: Mapping[str, Any], key: str, default: int) -> int:
    value = config.get(key, default)
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError(f"{key} must be a positive integer")
    return value


def _bool(config: Mapping[str, Any], key: str, default: bool) -> bool:
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
        for config_key, (env_name, default) in _OCTOPLANNER_CONFIG.items()
        for value in (config.get(config_key, default),)
    }


def _native_nav_mapping(config: Mapping[str, Any]) -> Mapping[str, Any]:
    native_nav = config.get("native_nav") or {}
    if not isinstance(native_nav, Mapping):
        raise ValueError("native_nav must be a mapping")
    return native_nav


def _recovery_order(value: Any) -> tuple[str, ...]:
    if value is None:
        return _RECOVERY_DEFAULT_ORDER
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


def _recovery_config(native_nav_config: Mapping[str, Any]) -> dict[str, Any]:
    raw = native_nav_config.get("recovery") or {}
    if not isinstance(raw, Mapping):
        raise ValueError("native_nav.recovery must be a mapping")
    attempts = raw.get("max_attempts", 0)
    if isinstance(attempts, bool) or not isinstance(attempts, int) or attempts < 0:
        raise ValueError("native_nav.recovery.max_attempts must be a non-negative integer")
    recovery = {
        "behavior_order": _recovery_order(raw.get("behavior_order")),
        "blocked_interval_s": _finite_number(raw, "blocked_interval_s", 2.0),
        "rotation_timeout_s": _finite_number(raw, "rotation_timeout_s", 2.5),
        "translation_timeout_s": _finite_number(raw, "translation_timeout_s", 1.5),
        "max_attempts": attempts,
        "translation_speed_mps": _finite_number(raw, "translation_speed_mps", 0.15),
        "rotation_rate_rad_s": _finite_number(raw, "rotation_rate_rad_s", 0.25),
        "min_rotation_rad": _finite_number(raw, "min_rotation_rad", 0.20),
        "max_rotation_rad": _finite_number(raw, "max_rotation_rad", 1.20),
        "rotation_candidate_step_rad": _finite_number(
            raw, "rotation_candidate_step_rad", 0.20
        ),
        "rotation_sample_step_rad": _finite_number(
            raw, "rotation_sample_step_rad", 0.05
        ),
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


def _scan_follower_config(native_nav_config: Mapping[str, Any]) -> dict[str, float]:
    raw = native_nav_config.get("scan_follower") or {}
    if not isinstance(raw, Mapping):
        raise ValueError("native_nav.scan_follower must be a mapping")
    follower = {
        "time_forward_s": _finite_number(raw, "time_forward_s", 0.55),
        "heading_error_rad": _finite_number(raw, "heading_error_rad", 0.8),
        "position_gain": _finite_number(raw, "position_gain", 0.9),
        "yaw_gain": _finite_number(raw, "yaw_gain", 1.2),
        "max_vx_mps": _finite_number(raw, "max_vx_mps", 0.5),
        "max_vy_mps": _finite_number(raw, "max_vy_mps", 0.25),
        "max_yaw_rate_rad_s": _finite_number(raw, "max_yaw_rate_rad_s", 1.0),
    }
    if follower["heading_error_rad"] > math.pi:
        raise ValueError("native_nav.scan_follower.heading_error_rad must not exceed pi")
    if follower["max_vx_mps"] <= 0.0 or follower["max_vy_mps"] <= 0.0:
        raise ValueError("native_nav.scan_follower axis speed limits must be positive")
    if follower["max_yaw_rate_rad_s"] > 1.0:
        raise ValueError("native_nav.scan_follower.max_yaw_rate_rad_s must not exceed 1.0")
    return follower


@dataclass(frozen=True)
class NativeNavConfig:
    """Validated native endpoint configuration."""

    product: str
    parameters: Mapping[str, float]
    native_nav: Mapping[str, Any]

    @property
    def environment(self) -> dict[str, str]:
        """Return the exact environment consumed by the native endpoint."""

        parameters = self.parameters
        native_nav = self.native_nav
        recovery = native_nav["recovery"]
        env = {
            "LINGTU_NAV_CORRIDOR_LOOKAHEAD_M": _env_number(parameters["corridor_lookahead_m"]),
            "LINGTU_NAV_GOAL_REACHED_M": _env_number(parameters["goal_reached_m"]),
            "LINGTU_NAV_CONTROL_MODE": str(native_nav["control_mode"]),
            "NAV_GLOBAL_PLANNER": str(native_nav["global_planner"]),
            "LINGTU_NAV_LOCAL_PLANNER_BACKEND": str(native_nav["local_planner"]),
            "LINGTU_NAV_PUBLISH_CMD_VEL": _env_bool(bool(native_nav["publish_cmd_vel"])),
            "LINGTU_NAV_CHECK_OBSTACLE": _env_bool(bool(native_nav["check_obstacle"])),
            "LINGTU_NAV_DYNAMIC_MIN_CELLS": str(parameters["dynamic_min_cells"]),
            "LINGTU_NAV_DYNAMIC_MIN_SPEED_MPS": _env_number(parameters["dynamic_min_speed_mps"]),
            "LINGTU_NAV_DYNAMIC_CONFIRM_FRAMES": str(parameters["dynamic_confirm_frames"]),
            "LINGTU_NAV_SMOOTHER_ENABLED": _env_bool(bool(native_nav["smoothing"])),
            "LINGTU_NAV_PATH_FOLLOWER_GOAL_TOLERANCE_M": _env_number(parameters["path_follower_goal_tolerance_m"]),
            "LINGTU_NAV_PATH_FOLLOWER_LOOKAHEAD_M": _env_number(parameters["path_follower_lookahead_m"]),
            "LINGTU_NAV_PATH_FOLLOWER_MAX_ACCEL_MPS2": _env_number(parameters["path_follower_max_accel_mps2"]),
            "LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS": _env_number(parameters["path_follower_max_speed_mps"]),
            "LINGTU_NAV_PATH_FOLLOWER_MIN_SPEED_MPS": _env_number(parameters["path_follower_min_speed_mps"]),
            "LINGTU_NAV_PATH_FOLLOWER_MAX_YAW_RATE_RAD_S": _env_number(parameters["path_follower_max_yaw_rate_rad_s"]),
            "LINGTU_NAV_PATH_FOLLOWER_MAX_YAW_ACCEL_RAD_S2": _env_number(parameters["path_follower_max_yaw_accel_rad_s2"]),
            "LINGTU_NAV_PATH_FOLLOWER_HEADING_ALIGN_ENTER_RAD": _env_number(parameters["path_follower_heading_align_enter_rad"]),
            "LINGTU_NAV_PATH_FOLLOWER_HEADING_ALIGN_EXIT_RAD": _env_number(parameters["path_follower_heading_align_exit_rad"]),
            "LINGTU_NAV_SCAN_TIME_FORWARD_S": _env_number(parameters["scan_time_forward_s"]),
            "LINGTU_NAV_SCAN_HEADING_ERROR_RAD": _env_number(parameters["scan_heading_error_rad"]),
            "LINGTU_NAV_SCAN_POSITION_GAIN": _env_number(parameters["scan_position_gain"]),
            "LINGTU_NAV_SCAN_YAW_GAIN": _env_number(parameters["scan_yaw_gain"]),
            "LINGTU_NAV_SCAN_MAX_VX_MPS": _env_number(parameters["scan_max_vx_mps"]),
            "LINGTU_NAV_SCAN_MAX_VY_MPS": _env_number(parameters["scan_max_vy_mps"]),
            "LINGTU_NAV_SCAN_MAX_YAW_RATE_RAD_S": _env_number(parameters["scan_max_yaw_rate_rad_s"]),
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
            "LINGTU_NAV_USE_TRAVERSABILITY_COST": _env_bool(bool(native_nav["use_traversability_cost"])),
            "LINGTU_NAV_ALLOW_TELEOP_TAKEOVER": _env_bool(bool(native_nav["allow_teleop_takeover"])),
            "LINGTU_TELEOP_PLANNER_HORIZON_M": _env_number(parameters["teleop_planner_horizon_m"]),
            "LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG": _env_number(parameters["teleop_planner_max_deviation_deg"]),
            "LINGTU_TELEOP_MAX_SPEED_MPS": _env_number(parameters["teleop_max_speed_mps"]),
            "LINGTU_TELEOP_MAX_YAW_RATE": _env_number(parameters["teleop_max_yaw_rate_rad_s"]),
            "LINGTU_TELEOP_LOCAL_PLANNER": _env_bool(bool(native_nav["teleop_local_planner"])),
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
) -> NativeNavConfig:
    """Compile native navigation parameters from one explicit Product contract."""

    product_name = str(product).strip()
    if not product_name:
        raise ValueError("native navigation Product must not be empty")
    native_nav_config = _native_nav_mapping(config)
    scan_follower = _scan_follower_config(native_nav_config)
    raw_control_mode = str(config.get("native_control_mode") or "").strip().lower()
    if not raw_control_mode:
        raise ValueError("native_control_mode must be declared by the compiled Product")
    if raw_control_mode not in _CONTROL_MODES:
        raise ValueError("native_control_mode must be autonomy, teleop, or teleop_avoid")
    global_planner = str(native_nav_config.get("global_planner") or "octoplanner3d").strip().lower()
    if global_planner not in _GLOBAL_PLANNERS:
        raise ValueError("native_nav.global_planner must be octoplanner3d or far")
    try:
        local_planner = local_planner_name(native_nav_config.get("local_planner") or "cmu")
    except ValueError as exc:
        raise ValueError("native_nav.local_planner must be cmu or scan") from exc
    native_nav = {
        "control_mode": raw_control_mode,
        "global_planner": global_planner,
        "local_planner": local_planner,
        "publish_cmd_vel": _bool(native_nav_config, "publish_cmd_vel", True),
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
        "smoothing": _bool(native_nav_config, "smoothing", False),
        "recovery": _recovery_config(native_nav_config),
    }
    native_nav["octoplanner3d"] = {
        key: native_nav_config.get(key, config.get(key, default))
        for key, (_, default) in _OCTOPLANNER_CONFIG.items()
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
    vehicle_length_m = _finite_number(config, "vehicle_length_m", 1.0)
    vehicle_width_m = _finite_number(config, "vehicle_width_m", 0.6)
    vehicle_height_m = _finite_number(config, "vehicle_height_m", 0.5)
    default_cylinder_radius_m = math.hypot(0.25 * vehicle_length_m, 0.5 * vehicle_width_m)
    path_follower_goal_tolerance_m = _finite_number(
        config, "path_follower_goal_tolerance", 0.2
    )
    parameters = {
        "corridor_lookahead_m": _finite_number(native_nav_config, "corridor_lookahead_m", 3.0),
        "dynamic_min_cells": _positive_integer(native_nav_config, "dynamic_min_cells", 8),
        "dynamic_min_speed_mps": _finite_number(
            native_nav_config, "dynamic_min_speed_mps", 0.25
        ),
        "dynamic_confirm_frames": _positive_integer(
            native_nav_config, "dynamic_confirm_frames", 4
        ),
        "goal_reached_m": _finite_number(config, "final_waypoint_threshold", 0.35),
        "path_follower_goal_tolerance_m": path_follower_goal_tolerance_m,
        "path_follower_lookahead_m": _finite_number(config, "path_follower_lookahead", 0.3),
        "path_follower_max_accel_mps2": _finite_number(
            native_nav_config,
            "path_follower_max_accel_mps2",
            _finite_number(config, "path_follower_max_accel", 1.0),
        ),
        "path_follower_max_speed_mps": _finite_number(config, "path_follower_max_speed", 0.5),
        "path_follower_min_speed_mps": _finite_number(config, "path_follower_min_speed", 0.0),
        "path_follower_max_yaw_rate_rad_s": _finite_number(
            native_nav_config, "path_follower_max_yaw_rate_rad_s", 0.8
        ),
        "path_follower_max_yaw_accel_rad_s2": _finite_number(
            native_nav_config, "path_follower_max_yaw_accel_rad_s2", 2.0
        ),
        "path_follower_heading_align_enter_rad": _finite_number(
            native_nav_config, "path_follower_heading_align_enter_rad", math.pi / 4.0
        ),
        "path_follower_heading_align_exit_rad": _finite_number(
            native_nav_config, "path_follower_heading_align_exit_rad", 0.35
        ),
        "scan_time_forward_s": scan_follower["time_forward_s"],
        "scan_heading_error_rad": scan_follower["heading_error_rad"],
        "scan_position_gain": scan_follower["position_gain"],
        "scan_yaw_gain": scan_follower["yaw_gain"],
        "scan_max_vx_mps": scan_follower["max_vx_mps"],
        "scan_max_vy_mps": scan_follower["max_vy_mps"],
        "scan_max_yaw_rate_rad_s": scan_follower["max_yaw_rate_rad_s"],
        # CMU and SCAN stop tracking the local path with one shared tolerance.
        "scan_finish_distance_m": path_follower_goal_tolerance_m,
        "waypoint_reached_m": _finite_number(config, "waypoint_threshold", 0.6),
        "teleop_planner_horizon_m": _finite_number(native_nav_config, "teleop_planner_horizon_m", 3.5),
        "teleop_planner_max_deviation_deg": _finite_number(native_nav_config, "teleop_planner_max_deviation_deg", 55.0),
        "teleop_max_speed_mps": _finite_number(config, "teleop_max_speed_mps", 0.5),
        "teleop_max_yaw_rate_rad_s": _finite_number(config, "teleop_max_yaw_rate_rad_s", 1.0),
        "vehicle_length_m": vehicle_length_m,
        "vehicle_width_m": vehicle_width_m,
        "collision_hard_margin_m": _finite_number(
            config, "collision_hard_margin_m", 0.15
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
    if (
        "octoplanner3d_robot_radius" not in native_nav_config
        and "octoplanner3d_robot_radius" not in config
    ):
        # The global route must clear the same hard XY envelope that the local
        # planner will execute.  A heading-aligned double cylinder reaches
        # radius + offset from the body origin; one Product-wide constant made
        # Thunder routes too tight and Go2 routes unnecessarily wide.
        native_nav["octoplanner3d"]["octoplanner3d_robot_radius"] = (
            parameters["collision_cylinder_radius_m"]
            + parameters["collision_cylinder_offset_m"]
        )
    if parameters["teleop_planner_horizon_m"] < 0.5:
        raise ValueError("teleop_planner_horizon_m must be at least 0.5")
    if parameters["dynamic_confirm_frames"] < 2:
        raise ValueError("dynamic_confirm_frames must be at least 2")
    if parameters["teleop_planner_max_deviation_deg"] > 90.0:
        raise ValueError("teleop_planner_max_deviation_deg must not exceed 90")
    if parameters["teleop_max_speed_mps"] <= 0.0:
        raise ValueError("teleop_max_speed_mps must be positive")
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
    return NativeNavConfig(product_name, parameters, native_nav)


__all__ = [
    "NativeNavConfig",
    "compile_native_nav_config",
    "local_planner_name",
    "mapd_environment",
]

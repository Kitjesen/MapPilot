"""Compile explicit Product values for the native navigation endpoint."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from typing import Any, Mapping

_SCHEMA_VERSION = "lingtu.native_nav_config.v1"
_CONTROL_MODES = frozenset({"autonomy", "teleop", "teleop_avoid"})


def _finite_number(config: Mapping[str, Any], key: str, default: float) -> float:
    value = config.get(key, default)
    if isinstance(value, bool):
        raise ValueError(f"{key} must be a finite number")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{key} must be a finite number") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"{key} must be a finite number")
    if parsed < 0.0:
        raise ValueError(f"{key} must be non-negative")
    return parsed


def _env_number(value: float) -> str:
    return format(value, ".15g")


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


def _native_nav_mapping(config: Mapping[str, Any]) -> Mapping[str, Any]:
    native_nav = config.get("native_nav") or {}
    if not isinstance(native_nav, Mapping):
        raise ValueError("native_nav must be a mapping")
    return native_nav


def _native_value(
    config: Mapping[str, Any],
    native_nav: Mapping[str, Any],
    key: str,
    default: Any,
) -> Any:
    if key in native_nav:
        return native_nav[key]
    prefixed = f"native_{key}"
    if prefixed in config:
        return config[prefixed]
    return default


@dataclass(frozen=True)
class NativeNavConfig:
    """Validated, fingerprinted native endpoint configuration."""

    product: str
    parameters: Mapping[str, float]
    native_nav: Mapping[str, Any]
    fingerprint: str

    @property
    def environment(self) -> dict[str, str]:
        """Return the exact environment consumed by the native endpoint."""

        parameters = self.parameters
        native_nav = self.native_nav
        return {
            "LINGTU_NAV_CONFIG_FINGERPRINT": self.fingerprint,
            "LINGTU_NAV_CORRIDOR_LOOKAHEAD_M": _env_number(parameters["corridor_lookahead_m"]),
            "LINGTU_NAV_GOAL_REACHED_M": _env_number(parameters["goal_reached_m"]),
            "LINGTU_NAV_CONTROL_MODE": str(native_nav["control_mode"]),
            "LINGTU_NAV_PUBLISH_CMD_VEL": _env_bool(bool(native_nav["publish_cmd_vel"])),
            "LINGTU_NAV_CHECK_OBSTACLE": _env_bool(bool(native_nav["check_obstacle"])),
            "LINGTU_NAV_PATH_FOLLOWER_GOAL_TOLERANCE_M": _env_number(parameters["path_follower_goal_tolerance_m"]),
            "LINGTU_NAV_PATH_FOLLOWER_LOOKAHEAD_M": _env_number(parameters["path_follower_lookahead_m"]),
            "LINGTU_NAV_PATH_FOLLOWER_MAX_ACCEL_MPS2": _env_number(parameters["path_follower_max_accel_mps2"]),
            "LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS": _env_number(parameters["path_follower_max_speed_mps"]),
            "LINGTU_NAV_PATH_FOLLOWER_MIN_SPEED_MPS": _env_number(parameters["path_follower_min_speed_mps"]),
            # Native endpoint ABI; the value is the active Product name.
            "LINGTU_PRODUCT": self.product,
            "LINGTU_NAV_USE_TRAVERSABILITY_COST": _env_bool(bool(native_nav["use_traversability_cost"])),
            "LINGTU_NAV_ALLOW_TELEOP_TAKEOVER": _env_bool(bool(native_nav["allow_teleop_takeover"])),
            "LINGTU_TELEOP_PLANNER_HORIZON_M": _env_number(parameters["teleop_planner_horizon_m"]),
            "LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG": _env_number(parameters["teleop_planner_max_deviation_deg"]),
            "LINGTU_TELEOP_LOCAL_PLANNER": _env_bool(bool(native_nav["teleop_local_planner"])),
            "LINGTU_NAV_WAYPOINT_REACHED_M": _env_number(parameters["waypoint_reached_m"]),
        }

    def as_dict(self) -> dict[str, Any]:
        """Return deterministic JSON-ready configuration data."""

        return {
            "schema_version": _SCHEMA_VERSION,
            "product": self.product,
            "fingerprint": self.fingerprint,
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
    declared_control_mode = str(config.get("native_control_mode") or "").strip().lower()
    native_control_mode = str(native_nav_config.get("control_mode") or "").strip().lower()
    if declared_control_mode and native_control_mode and declared_control_mode != native_control_mode:
        raise ValueError("native_nav.control_mode must match native_control_mode")
    raw_control_mode = native_control_mode or declared_control_mode
    if not raw_control_mode:
        raise ValueError(
            "native_nav.control_mode must be declared by the compiled Product"
        )
    if raw_control_mode not in _CONTROL_MODES:
        raise ValueError("native_nav.control_mode must be autonomy, teleop, or teleop_avoid")
    native_nav = {
        "control_mode": raw_control_mode,
        "publish_cmd_vel": _bool(
            {"publish_cmd_vel": _native_value(config, native_nav_config, "publish_cmd_vel", True)},
            "publish_cmd_vel",
            True,
        ),
        "check_obstacle": _bool(
            {"check_obstacle": _native_value(config, native_nav_config, "check_obstacle", raw_control_mode != "teleop")},
            "check_obstacle",
            raw_control_mode != "teleop",
        ),
        "use_traversability_cost": _bool(
            {
                "use_traversability_cost": _native_value(
                    config,
                    native_nav_config,
                    "use_traversability_cost",
                    raw_control_mode not in {"teleop"},
                )
            },
            "use_traversability_cost",
            raw_control_mode != "teleop",
        ),
        "allow_teleop_takeover": _bool(
            {
                "allow_teleop_takeover": _native_value(
                    config,
                    native_nav_config,
                    "allow_teleop_takeover",
                    False,
                )
            },
            "allow_teleop_takeover",
            False,
        ),
        "teleop_local_planner": _bool(
            {
                "teleop_local_planner": _native_value(
                    config,
                    native_nav_config,
                    "teleop_local_planner",
                    raw_control_mode == "teleop_avoid",
                )
            },
            "teleop_local_planner",
            raw_control_mode == "teleop_avoid",
        ),
    }
    if raw_control_mode == "teleop" and (
        native_nav["check_obstacle"] or native_nav["use_traversability_cost"]
    ):
        raise ValueError("native_nav teleop mode must not enable obstacle or traversability checks")
    if raw_control_mode == "teleop_avoid" and (
        not native_nav["check_obstacle"]
        or not native_nav["use_traversability_cost"]
        or not native_nav["teleop_local_planner"]
    ):
        raise ValueError(
            "native_nav teleop_avoid must enable obstacle checks, traversability, and teleop local planner"
        )
    parameters = {
        "corridor_lookahead_m": _finite_number(config, "native_corridor_lookahead_m", 3.0),
        "goal_reached_m": _finite_number(config, "final_waypoint_threshold", 0.35),
        "path_follower_goal_tolerance_m": _finite_number(config, "path_follower_goal_tolerance", 0.2),
        "path_follower_lookahead_m": _finite_number(config, "path_follower_lookahead", 0.3),
        "path_follower_max_accel_mps2": _finite_number(config, "path_follower_max_accel", 1.0),
        "path_follower_max_speed_mps": _finite_number(config, "path_follower_max_speed", 0.4),
        "path_follower_min_speed_mps": _finite_number(config, "path_follower_min_speed", 0.0),
        "waypoint_reached_m": _finite_number(config, "waypoint_threshold", 0.6),
        "teleop_planner_horizon_m": _finite_number(config, "teleop_planner_horizon_m", 2.0),
        "teleop_planner_max_deviation_deg": _finite_number(config, "teleop_planner_max_deviation_deg", 55.0),
    }
    if parameters["teleop_planner_horizon_m"] < 0.5:
        raise ValueError("teleop_planner_horizon_m must be at least 0.5")
    if parameters["teleop_planner_max_deviation_deg"] > 90.0:
        raise ValueError("teleop_planner_max_deviation_deg must not exceed 90")
    if parameters["path_follower_min_speed_mps"] > parameters["path_follower_max_speed_mps"]:
        raise ValueError("path_follower_min_speed must not exceed path_follower_max_speed")
    if parameters["path_follower_goal_tolerance_m"] > parameters["goal_reached_m"]:
        raise ValueError("path_follower_goal_tolerance must not exceed final_waypoint_threshold")
    fingerprint_source = {
        "schema_version": _SCHEMA_VERSION,
        "product": product_name,
        "native_nav": native_nav,
        "parameters": parameters,
    }
    canonical_json = json.dumps(
        fingerprint_source,
        ensure_ascii=True,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    fingerprint = hashlib.sha256(canonical_json).hexdigest()
    return NativeNavConfig(product_name, parameters, native_nav, fingerprint)


__all__ = ["NativeNavConfig", "compile_native_nav_config"]

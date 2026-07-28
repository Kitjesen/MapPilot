"""Compile resolved Product profile values for the native navigation endpoint."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from typing import Any, Mapping

_SCHEMA_VERSION = "lingtu.native_nav_profile.v2"


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


@dataclass(frozen=True)
class NativeNavProfileConfig:
    """Validated, fingerprinted native endpoint configuration."""

    profile: str
    parameters: Mapping[str, float]
    fingerprint: str

    @property
    def environment(self) -> dict[str, str]:
        """Return the exact environment consumed by the native endpoint."""

        parameters = self.parameters
        return {
            "LINGTU_NAV_CONFIG_FINGERPRINT": self.fingerprint,
            "LINGTU_NAV_CORRIDOR_LOOKAHEAD_M": _env_number(parameters["corridor_lookahead_m"]),
            "LINGTU_NAV_GOAL_REACHED_M": _env_number(parameters["goal_reached_m"]),
            "LINGTU_NAV_PATH_FOLLOWER_GOAL_TOLERANCE_M": _env_number(parameters["path_follower_goal_tolerance_m"]),
            "LINGTU_NAV_PATH_FOLLOWER_LOOKAHEAD_M": _env_number(parameters["path_follower_lookahead_m"]),
            "LINGTU_NAV_PATH_FOLLOWER_MAX_ACCEL_MPS2": _env_number(parameters["path_follower_max_accel_mps2"]),
            "LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS": _env_number(parameters["path_follower_max_speed_mps"]),
            "LINGTU_NAV_PATH_FOLLOWER_MIN_SPEED_MPS": _env_number(parameters["path_follower_min_speed_mps"]),
            "LINGTU_NAV_PROFILE": self.profile,
            "LINGTU_TELEOP_PLANNER_HORIZON_M": _env_number(parameters["teleop_planner_horizon_m"]),
            "LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG": _env_number(parameters["teleop_planner_max_deviation_deg"]),
            "LINGTU_NAV_WAYPOINT_REACHED_M": _env_number(parameters["waypoint_reached_m"]),
        }

    def as_dict(self) -> dict[str, Any]:
        """Return deterministic JSON-ready configuration data."""

        return {
            "schema_version": _SCHEMA_VERSION,
            "profile": self.profile,
            "fingerprint": self.fingerprint,
            "parameters": dict(self.parameters),
            "environment": self.environment,
        }


def native_nav_profile_config(
    profile: str,
    config: Mapping[str, Any],
) -> NativeNavProfileConfig:
    """Compile native navigation parameters from one resolved profile."""

    canonical_profile = str(profile).strip()
    if not canonical_profile:
        raise ValueError("native navigation profile must not be empty")
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
        "profile": canonical_profile,
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
    return NativeNavProfileConfig(canonical_profile, parameters, fingerprint)


__all__ = ["NativeNavProfileConfig", "native_nav_profile_config"]

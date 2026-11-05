"""Native parameters grouped by function and resolved for one Product run."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping

from lingtu.assembly.graph.settings import SYSTEM_SETTINGS_DIR, load_settings

from .map_input import COLLISION_SPECS, MAP_INPUT_SPECS
from .planning import PLANNING_SPECS
from .schema import ParameterSpec, ResolvedParameter, RuntimeParameterSet
from .segment import RISK_SPECS, SEGMENT_SPECS
from .tracking import TRACKING_SPECS

PARAMETER_SPECS: tuple[ParameterSpec, ...] = (
    *PLANNING_SPECS,
    *TRACKING_SPECS,
    *COLLISION_SPECS,
    *SEGMENT_SPECS,
    *MAP_INPUT_SPECS,
    *RISK_SPECS,
)

_SPECS_BY_NAME = {spec.name: spec for spec in PARAMETER_SPECS}


def resolve_parameters(
    *,
    product_parameters: Mapping[str, Any] | None = None,
    env_overrides: Mapping[str, Any] | None = None,
    session_overrides: Mapping[str, Any] | None = None,
    map_publish_hz: float | None = None,
    settings_directory: Path = SYSTEM_SETTINGS_DIR,
) -> RuntimeParameterSet:
    """Resolve defaults, Env, Product, then session overrides in that order."""

    resolved: dict[str, ResolvedParameter] = {}
    if not (session_overrides and _SPECS_BY_NAME.keys() <= session_overrides.keys()):
        settings = {"includes": ["local", "follower", "limits", "explore"]}
        load_settings(settings, settings_directory, standalone=True)
        defaults = settings["parameters"]
        # SCAN inherits the follower motion limits before Product overrides.
        limits = settings["native_nav"]
        defaults["scan_planner.max_velocity_mps"] = limits["path_follower_max_speed_mps"]
        defaults["scan_planner.max_acceleration_mps2"] = limits["path_follower_max_accel_mps2"]
        defaults["scan_follower.max_yaw_rate_rad_s"] = limits["path_follower_max_yaw_rate_rad_s"]
        if defaults.keys() != _SPECS_BY_NAME.keys():
            raise ValueError("navigation settings must define every runtime parameter default")
        _apply_overrides(resolved, defaults)
    _apply_overrides(resolved, env_overrides)
    _apply_overrides(resolved, product_parameters)
    _apply_overrides(resolved, session_overrides)

    if resolved["scan_planner.replan_distance_m"].value < resolved["scan_planner.no_replan_distance_m"].value:
        raise ValueError("scan_planner.replan_distance_m must be at least no_replan_distance_m")

    stop = float(resolved["risk.stop_threshold"].value)
    resume = float(resolved["risk.resume_threshold"].value)
    if resume > stop:
        raise ValueError(
            "risk.resume_threshold must be less than or equal to risk.stop_threshold"
        )

    publish_hz = _positive_optional(map_publish_hz, "map_publish_hz")
    if publish_hz is not None:
        minimum_age = 2.0 / publish_hz
        max_age = float(resolved["map_input.max_age_s"].value)
        if max_age + 1e-12 < minimum_age:
            raise ValueError(
                "map_input.max_age_s must cover at least two configured map "
                f"publication periods ({minimum_age:.6g}s at {publish_hz:.6g}Hz)"
            )

    return RuntimeParameterSet(values=resolved)


def _apply_overrides(
    resolved: dict[str, ResolvedParameter],
    values: Mapping[str, Any] | None,
) -> None:
    if not values:
        return
    unknown = sorted(str(name) for name in set(values) - set(_SPECS_BY_NAME))
    if unknown:
        raise ValueError("unknown runtime parameters: " + ", ".join(unknown))
    for name, raw_value in values.items():
        spec = _SPECS_BY_NAME[str(name)]
        resolved[spec.name] = ResolvedParameter(
            value=spec.coerce(raw_value),
            env_key=spec.env_key,
        )


def _positive_optional(value: Any, name: str) -> float | None:
    if value is None:
        return None
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise ValueError(f"{name} must be finite and strictly positive")
    return parsed


__all__ = [
    "PARAMETER_SPECS",
    "ResolvedParameter",
    "RuntimeParameterSet",
    "resolve_parameters",
]

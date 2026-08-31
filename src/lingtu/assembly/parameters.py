"""Resolve validated native runtime parameters."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping

from runtime.yaml_helpers import load_yaml

PARAMETER_PROFILE_PATH = (
    Path(__file__).resolve().parents[3]
    / "config"
    / "runtime_graph"
    / "parameter_profiles.yaml"
)


@dataclass(frozen=True)
class ParameterSpec:
    """One bounded parameter accepted by the native rolling navigation seam."""

    name: str
    env_key: str
    default: int | float
    parse: Callable[[Any], int | float]
    minimum: int | float
    maximum: int | float

    def coerce(self, value: Any) -> int | float:
        try:
            parsed = self.parse(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"parameter {self.name} has invalid value {value!r}") from exc
        if isinstance(parsed, float) and not math.isfinite(parsed):
            raise ValueError(f"parameter {self.name} must be finite")
        if parsed < self.minimum or parsed > self.maximum:
            raise ValueError(
                f"parameter {self.name}={parsed!r} is outside "
                f"[{self.minimum}, {self.maximum}]"
            )
        return parsed


@dataclass(frozen=True)
class ResolvedParameter:
    """One final native environment value."""

    value: int | float
    env_key: str


@dataclass(frozen=True)
class RuntimeParameterSet:
    """Validated native parameters."""

    values: Mapping[str, ResolvedParameter]

    def environment(self) -> dict[str, str]:
        return {
            resolved.env_key: _environment_value(resolved.value)
            for resolved in self.values.values()
        }


def _integer(value: Any) -> int:
    if isinstance(value, bool):
        raise ValueError("boolean is not an integer parameter")
    if isinstance(value, float) and not value.is_integer():
        raise ValueError("integer parameter must not contain a fraction")
    return int(value)


PARAMETER_SPECS: tuple[ParameterSpec, ...] = (
    ParameterSpec(
        "segment.max_distance_m",
        "LINGTU_NAV_SEGMENT_MAX_DISTANCE_M",
        5.0,
        float,
        0.25,
        50.0,
    ),
    ParameterSpec(
        "segment.max_waypoints",
        "LINGTU_NAV_SEGMENT_MAX_WAYPOINTS",
        32,
        _integer,
        2,
        4096,
    ),
    ParameterSpec(
        "map_input.max_cells",
        "LINGTU_NAV_SEGMENT_MAX_GRID_CELLS",
        262_144,
        _integer,
        64,
        4_194_304,
    ),
    ParameterSpec(
        "map_input.max_age_s",
        "LINGTU_NAV_SEGMENT_MAP_MAX_AGE_S",
        0.35,
        float,
        0.02,
        10.0,
    ),
    ParameterSpec(
        "map_input.future_tolerance_s",
        "LINGTU_NAV_SEGMENT_FUTURE_TOLERANCE_S",
        0.25,
        float,
        0.0,
        2.0,
    ),
    ParameterSpec(
        "risk.stop_threshold",
        "LINGTU_NAV_SEGMENT_RISK_STOP",
        50.0,
        float,
        0.0,
        100.0,
    ),
    ParameterSpec(
        "risk.resume_threshold",
        "LINGTU_NAV_SEGMENT_RISK_RESUME",
        40.0,
        float,
        0.0,
        100.0,
    ),
)

_SPECS_BY_NAME = {spec.name: spec for spec in PARAMETER_SPECS}


def load_parameter_profiles(path: str | Path | None = None) -> dict[str, dict[str, Any]]:
    """Load named parameter profiles without treating the file as defaults."""

    profile_path = Path(path) if path is not None else PARAMETER_PROFILE_PATH
    payload = load_yaml(profile_path, default={})
    if not isinstance(payload, Mapping):
        raise ValueError(f"parameter profile catalog must be an object: {profile_path}")
    profiles = payload.get("profiles", {})
    if not isinstance(profiles, Mapping):
        raise ValueError("parameter profile catalog profiles must be an object")
    result: dict[str, dict[str, Any]] = {}
    for raw_name, raw_values in profiles.items():
        name = str(raw_name).strip()
        if not name or not isinstance(raw_values, Mapping):
            raise ValueError("parameter profile entries require a name and object value")
        result[name] = dict(raw_values)
    return result


def resolve_parameters(
    *,
    parameter_profile: str | None,
    env_overrides: Mapping[str, Any] | None = None,
    session_overrides: Mapping[str, Any] | None = None,
    profiles: Mapping[str, Mapping[str, Any]] | None = None,
    map_publish_hz: float | None = None,
) -> RuntimeParameterSet:
    """Resolve defaults, Env, Product, then session overrides in that order."""

    selected_profile = str(parameter_profile or "").strip() or None
    profile_catalog = dict(profiles or load_parameter_profiles())
    if selected_profile is not None and selected_profile not in profile_catalog:
        raise ValueError(f"unknown runtime parameter profile: {selected_profile}")

    resolved = {
        spec.name: ResolvedParameter(
            value=spec.coerce(spec.default),
            env_key=spec.env_key,
        )
        for spec in PARAMETER_SPECS
    }
    _apply_parameter_layer(resolved, env_overrides)
    if selected_profile is not None:
        _apply_parameter_layer(
            resolved,
            profile_catalog[selected_profile],
        )
    _apply_parameter_layer(resolved, session_overrides)

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


def _apply_parameter_layer(
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


def _environment_value(value: int | float) -> str:
    return str(value) if isinstance(value, int) else repr(value)


__all__ = [
    "PARAMETER_PROFILE_PATH",
    "PARAMETER_SPECS",
    "ResolvedParameter",
    "RuntimeParameterSet",
    "load_parameter_profiles",
    "resolve_parameters",
]

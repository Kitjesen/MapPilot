"""Planner backend naming and runtime profile contract helpers."""

from __future__ import annotations

from typing import Any, Mapping

_PLANNER_NAME_ALIASES = {
    "octplanner": "octoplanner3d",
    "octo": "octoplanner3d",
    "octomap": "octoplanner3d",
}

DEFAULT_GLOBAL_PLANNER = "octoplanner3d"

_DEFAULT_LATENCY_BUDGET_MS = {
    "lite": 50,
    "map": 250,
    "nav": 800,
    "explore": 800,
    "tare_explore": 1000,
    "super_lio": 800,
    "super_lio_relocation": 800,
    "sim": 250,
    "portable_mujoco": 250,
    "sim_nav": 150,
    "sim_mujoco_live": 250,
    "sim_mujoco_octo_live": 800,
    "sim_mujoco_pct_live": 800,
    "sim_gazebo": 250,
    "sim_industrial": 250,
    "sim_cmu_tare": 1000,
    "dev": 250,
    "stub": 150,
}


def normalize_planner_name(name: str | None) -> str:
    """Return the canonical registry name for a planner backend."""

    normalized = str(name or "").strip().lower()
    return _PLANNER_NAME_ALIASES.get(normalized, normalized)


def planner_fallback_chain(value: Any) -> list[str]:
    """Normalize a configured fallback planner or planner list."""

    if value is None:
        return []
    if isinstance(value, str):
        raw_names = [value]
    elif isinstance(value, (list, tuple)):
        raw_names = [str(item) for item in value]
    else:
        raw_names = [str(value)]

    normalized: list[str] = []
    for raw_name in raw_names:
        name = normalize_planner_name(raw_name)
        if name and name not in normalized:
            normalized.append(name)
    return normalized


def resolve_planner_runtime_profile(
    profile: str,
    config: Mapping[str, Any],
) -> dict[str, Any]:
    """Build the planner contract for a resolved runtime profile."""

    primary = normalize_planner_name(
        str(config.get("planner") or config.get("planner_backend") or DEFAULT_GLOBAL_PLANNER)
    )
    fallback_value = config.get("fallback_planners")
    if fallback_value is None:
        fallback_value = config.get("fallback_planner_name")
    fallbacks = [
        name for name in planner_fallback_chain(fallback_value) if name != primary
    ]
    latency_budget = int(
        config.get("planner_latency_budget_ms")
        or _DEFAULT_LATENCY_BUDGET_MS.get(profile, 500)
    )
    return {
        "schema_version": "lingtu.planner_runtime_profile.v1",
        "profile": profile,
        "primary": primary,
        "fallback_planners": fallbacks,
        "plan_safety_policy": str(config.get("plan_safety_policy") or "observe"),
        "latency_budget_ms": max(1, latency_budget),
    }

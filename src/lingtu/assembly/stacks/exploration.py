"""Exploration stack: LingTu module bridges for exploration goal sources.

TARE has exactly one runtime owner:

- `owner="native"` mounts no Python goal producer; Product owns the native
  exploration endpoint.
- `owner="module"`, `backend="tare"` runs the in-process policy for development
  and simulation.
- `owner="module"`, `backend="tare_external"` consumes external CMU/TARE data.

No path controls the robot directly. Exploration submits goals to the navigation
owner, which retains planning, tracking, safety, and command authority.
"""

from __future__ import annotations

import logging

from runtime.blueprint import Blueprint
from runtime.plugin_resolution import stack_module

logger = logging.getLogger(__name__)


def exploration(backend: str = "tare", *, owner: str = "module", **kw) -> Blueprint:
    """Build the module side of the exploration runtime."""
    bp = Blueprint()
    backend = backend or "none"
    owner = owner or "module"

    if owner not in {"module", "native"}:
        raise ValueError(f"Unknown exploration owner {owner!r}. Options: 'module', 'native'.")
    if backend == "none":
        return bp
    if owner == "native":
        if backend != "tare":
            raise ValueError("Native exploration ownership only supports the 'tare' backend.")
        logger.info("TARE exploration is owned by the native endpoint; Python producer omitted")
        return bp

    transport = {"tare": "in_process", "tare_external": "dds"}.get(backend)
    if transport is not None:
        _add_tare_bridge(
            bp,
            **{
                "configured_backend": backend,
                "transport_mode": transport,
                **kw,
            },
        )
        return bp

    raise ValueError(
        f"Unknown exploration backend {backend!r}. "
        "Options: 'tare', 'tare_external', 'none'."
    )


def _add_tare_bridge(bp: Blueprint, **kw) -> None:
    """Add TARE bridge modules without launching a local TARE process."""
    TAREExplorerModule = stack_module(
        "exploration",
        "tare",
        seed_group="exploration",
        fallback="explore.tare.module.TAREExplorerModule",
    )
    ExplorationSupervisorModule = stack_module(
        "exploration",
        "supervisor",
        seed_group="exploration",
        fallback="explore.tare.supervisor.ExplorationSupervisorModule",
    )

    kw = {"prefer_path_strategy": False, **kw}
    bp.add(TAREExplorerModule, alias="TAREExplorerModule", **_tare_kwargs(kw))
    bp.add(
        ExplorationSupervisorModule,
        alias="ExplorationSupervisorModule",
        **_supervisor_kwargs(kw),
    )
    logger.info(
        "TARE exploration bridge enabled (backend=%s, native process off)",
        kw.get("configured_backend", "tare"),
    )


def _tare_kwargs(kw: dict) -> dict:
    """Keep only TAREExplorerModule-relevant kwargs."""
    allowed = {
        "way_point_topic",
        "path_topic",
        "runtime_topic",
        "finish_topic",
        "start_topic",
        "configured_backend",
        "goal_frame_id",
        "way_point_timeout_s",
        "auto_start",
        "hold_active_goal_until_terminal",
        "max_waypoint_distance_m",
        "waypoint_odometry_timeout_s",
        "prefer_path_strategy",
        "path_goal_min_distance_m",
        "path_goal_spacing_m",
        "path_start_tolerance_m",
        "path_max_goal_count",
        "path_strategy_timeout_s",
        "path_strategy_fallback_to_waypoint",
        "navigation_goal_match_tolerance_m",
        "transport_mode",
        "policy_rate_hz",
    }
    return {k: v for k, v in kw.items() if k in allowed}


def _supervisor_kwargs(kw: dict) -> dict:
    """Keep only ExplorationSupervisorModule-relevant kwargs."""
    mapping = {
        "tare_warn_timeout_s": "warn_timeout_s",
        "tare_fallback_timeout_s": "fallback_timeout_s",
        "tare_supervisor_hz": "poll_hz",
    }
    return {dst: kw[src] for src, dst in mapping.items() if src in kw}

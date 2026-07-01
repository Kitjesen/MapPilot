"""Exploration stack: LingTu module bridges for exploration goal sources.

Wavefront frontier lives in the navigation stack. This stack only owns TARE
integration modules. TARE is treated as an external/endpoint-owned runtime:
LingTu consumes its DDS outputs and publishes start/stop hints, but no longer
starts a ROS 2 NativeModule process from the Module graph.
"""

from __future__ import annotations

import logging

from runtime.blueprint import Blueprint
from runtime.blueprints.stacks._registry import stack_module

logger = logging.getLogger(__name__)


def exploration(backend: str = "tare", **kw) -> Blueprint:
    """Build an exploration stack Blueprint.

    Args:
        backend: "tare" | "tare_external" | "none". "tare" is the canonical
            TARE bridge selection; "tare_external" is kept as an explicit
            compatibility alias for endpoint-owned TARE runtimes.
        **kw: forwarded to the module constructors.
    """
    bp = Blueprint()
    backend = backend or "none"

    if backend == "none":
        return bp

    if backend == "tare":
        kw.setdefault("configured_backend", "tare")
        kw.setdefault("transport_mode", "in_process")
        _add_tare_bridge(bp, **kw)
        return bp

    if backend == "tare_external":
        kw.setdefault("configured_backend", "tare_external")
        kw.setdefault("transport_mode", "dds")
        _add_tare_bridge(bp, **kw)
        return bp

    raise ValueError(
        f"Unknown exploration backend {backend!r}. "
        "Options: 'tare', 'tare_external', 'none'. "
        "'wavefront' lives in nav.exploration.frontier_explorer_module."
    )


def _add_tare_bridge(bp: Blueprint, **kw) -> None:
    """Add TARE bridge modules without launching a local TARE process."""
    TAREExplorerModule = stack_module(
        "exploration",
        "tare",
        seed_group="exploration",
        fallback="nav.exploration.tare.module.TAREExplorerModule",
    )
    ExplorationSupervisorModule = stack_module(
        "exploration",
        "supervisor",
        seed_group="exploration",
        fallback="nav.exploration.tare.supervisor.ExplorationSupervisorModule",
    )

    kw.setdefault("prefer_path_strategy", False)
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

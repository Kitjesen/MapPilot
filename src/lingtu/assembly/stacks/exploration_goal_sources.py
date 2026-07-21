"""Exploration goal-source composition for navigation."""

from __future__ import annotations

import logging

from runtime.blueprint import Blueprint
from runtime.plugin_resolution import stack_module

logger = logging.getLogger(__name__)


def frontier_module_config(**config) -> dict:
    """Return WavefrontFrontierExplorer constructor kwargs."""

    return {
        "min_frontier_size": config.get("frontier_min_size", 5),
        "safe_distance": config.get("frontier_safe_distance", 1.0),
        "lookahead_distance": config.get("frontier_lookahead", 5.0),
        "max_explored_distance": config.get("frontier_max_dist", 15.0),
        "info_gain_threshold": config.get("frontier_info_gain", 0.03),
        "goal_timeout": config.get("frontier_goal_timeout", 30.0),
        "explore_rate": config.get("frontier_rate", 2.0),
        "blocked_goal_radius": config.get("frontier_blocked_goal_radius", 1.0),
        "blocked_goal_ttl": config.get("frontier_blocked_goal_ttl", 120.0),
        "approach_standoff_m": config.get("frontier_approach_standoff_m", 0.8),
        "approach_max_target_distance_m": config.get(
            "frontier_approach_max_target_distance_m",
            1.5,
        ),
        "approach_goal_max_distance_m": config.get(
            "frontier_approach_goal_max_distance_m",
            3.0,
        ),
        "reachable_goal_radius": config.get("frontier_reachable_goal_radius", 0.8),
        "navigation_failure_grace_s": config.get(
            "frontier_navigation_failure_grace_s",
            2.0,
        ),
        "cost_obstacle_threshold": config.get(
            "frontier_cost_obstacle_threshold",
            49.9,
        ),
    }


def add_exploration_goal_sources(bp: Blueprint, **config) -> Blueprint:
    """Add optional exploration Modules that feed Navigation goals."""

    if config.get("enable_frontier", False):
        try:
            WavefrontFrontierExplorer = stack_module(
                "exploration",
                "wavefront_frontier",
                seed_group="exploration",
                fallback="explore.frontier.WavefrontFrontierExplorer",
            )
            bp.add(
                WavefrontFrontierExplorer,
                alias="WavefrontFrontierExplorer",
                **frontier_module_config(**config),
            )
            bp.wire(
                "WavefrontFrontierExplorer",
                "exploration_goal",
                "nav.mission",
                "goal_pose",
            )
            bp.wire(
                "nav.mission",
                "mission_status",
                "WavefrontFrontierExplorer",
                "navigation_status",
            )
        except ImportError as e:
            logger.warning("FrontierExplorer not available: %s", e)

    if config.get("enable_traversable_frontier", False):
        try:
            TraversableFrontierModule = stack_module(
                "navigation",
                "traversable_frontier",
                seed_group="navigation",
                fallback="explore.traversable_frontier.TraversableFrontierModule",
            )

            bp.add(
                TraversableFrontierModule,
                alias="TraversableFrontierModule",
                min_frontier_size=config.get("traversable_frontier_min_size", 5),
                safe_distance=config.get("traversable_frontier_safe_distance", 1.0),
                lookahead_distance=config.get("traversable_frontier_lookahead", 5.0),
                max_explored_distance=config.get("traversable_frontier_max_dist", 15.0),
                info_gain_threshold=config.get("traversable_frontier_info_gain", 0.03),
                goal_timeout=config.get("traversable_frontier_goal_timeout", 30.0),
                explore_rate=config.get("traversable_frontier_rate", 2.0),
                blocked_goal_radius=config.get(
                    "traversable_frontier_blocked_goal_radius",
                    1.0,
                ),
                blocked_goal_ttl=config.get(
                    "traversable_frontier_blocked_goal_ttl",
                    120.0,
                ),
                max_slope_deg=config.get("traversable_frontier_max_slope_deg", 35.0),
                max_frontier_cost=config.get("traversable_frontier_max_cost", 80.0),
                semantic_prior_weight=config.get(
                    "traversable_frontier_semantic_prior_weight",
                    0.0,
                ),
            )
        except ImportError as e:
            logger.warning("TraversableFrontierModule not available: %s", e)

    return bp

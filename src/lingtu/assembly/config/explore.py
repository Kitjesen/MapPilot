"""Bind exploration settings to the native endpoint environment."""

from __future__ import annotations

import math
from collections.abc import Mapping

_NUMBER_FIELDS = (
    "tick_hz",
    "command_retry_s",
    "arrival_tolerance_m",
    "goal_timeout_s",
    "status_period_s",
    "control_max_age_s",
    "directed_target_max_ttl_s",
    "directed_progress_weight",
    "odom_max_age_s",
    "snapshot_max_age_s",
    "tf_max_age_s",
    "future_tolerance_s",
    "sensor_range_m",
    "sensor_horizontal_fov_rad",
    "sensor_yaw_offset_rad",
    "candidate_radius_m",
    "min_goal_distance_m",
    "novelty_radius_m",
    "coverage_resolution_m",
    "local_route_radius_m",
    "return_home_distance_m",
    "keypose_min_distance_m",
    "keypose_connect_distance_m",
    "gain_weight",
    "travel_weight",
    "momentum_weight",
    "revisit_weight",
    "max_plan_time_ms",
)
_INTEGER_FIELDS = (
    "command_timeout_ms",
    "min_frontier_size",
    "max_candidates",
    "route_2opt_iterations",
    "max_grid_cells",
    "max_frontier_cells",
    "max_frontier_clusters",
    "max_coverage_cells",
    "max_known_map_candidate_samples",
    "max_known_map_gain_checks_per_candidate",
    "max_known_map_visibility_checks",
    "max_keyposes",
    "max_keypose_edges",
    "max_keypose_neighbor_links",
    "max_route_targets",
)
_BOOLEAN_FIELDS = ("known_map_require_line_of_sight", "return_home")


def explore_environment(settings: Mapping[str, object]) -> dict[str, str]:
    if settings.keys() != {*_NUMBER_FIELDS, *_INTEGER_FIELDS, *_BOOLEAN_FIELDS}:
        raise ValueError("explore settings contain missing or unknown fields")
    environment: dict[str, str] = {}
    for key in _NUMBER_FIELDS:
        value = settings[key]
        if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value):
            raise ValueError(f"explore.{key} must be a finite number")
        environment[f"LINGTU_EXPLORE_{key.upper()}"] = str(value)
    for key in _INTEGER_FIELDS:
        value = settings[key]
        if type(value) is not int or value < 0:
            raise ValueError(f"explore.{key} must be a non-negative integer")
        environment[f"LINGTU_EXPLORE_{key.upper()}"] = str(value)
    for key in _BOOLEAN_FIELDS:
        value = settings[key]
        if not isinstance(value, bool):
            raise ValueError(f"explore.{key} must be a boolean")
        environment[f"LINGTU_EXPLORE_{key.upper()}"] = "1" if value else "0"
    return environment

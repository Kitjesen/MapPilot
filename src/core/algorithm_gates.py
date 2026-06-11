"""Shared algorithm gate definitions.

Keep product diagnostics and server-side closure presets on the same required
gate sequence so readiness claims cannot drift between surfaces.
"""

from __future__ import annotations

INSPECTION_MVP_REQUIRED_GATES: tuple[str, ...] = (
    "gateway_runtime_acceptance",
    "routecheck_preflight",
    "large_terrain",
    "fastlio2_dynamic_inspection",
    "dynamic_obstacle_local_planner",
    "moving_obstacle_sweep",
)

DIMOS_BENCHMARK_REQUIRED_GATES: tuple[str, ...] = (
    "gateway_runtime_acceptance",
    "routecheck_preflight",
    "blocked_route_replan_preflight",
    "navigation_replay_deviation",
    "large_terrain",
    "native_pct_mujoco",
    "dynamic_obstacle_local_planner",
    "fastlio2_dynamic_inspection",
    "moving_obstacle_sweep",
    "large_loop_closure",
    "gazebo_runtime",
    "saved_map_relocalize",
    "pct_saved_map_navigation",
)

G4_SERVER_FULL_SIM_REQUIRED_GATES: tuple[str, ...] = (
    "gateway_runtime_acceptance",
    "routecheck_preflight",
    "blocked_route_replan_preflight",
    "navigation_replay_deviation",
    "multifloor_exploration",
    "large_terrain",
    "native_pct_mujoco",
    "dynamic_obstacle_local_planner",
    "fastlio2_dynamic_inspection",
    "moving_obstacle_sweep",
    "large_loop_closure",
    "gazebo_runtime",
    "saved_map_relocalize",
    "pct_saved_map_navigation",
)

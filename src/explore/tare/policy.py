"""Portable TARE-style exploration policy (C++-delegated).

This is the LingTu-owned exploration entrypoint. The exploration algorithm
(BFS reachability, frontier detection, clustering, viewpoint scoring) lives
solely in C++ (``src/explore/cpp/tare_policy.cpp``) and is exposed to Python
through the ``lingtu_explore_kernel`` nanobind extension. This module keeps only
the stable Python-facing data contract (``TAREPolicyConfig``, ``TAREDecision``)
and delegates ``select()`` to the native backend. Do not reintroduce the Python
algorithm here.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class TAREPolicyConfig:
    min_frontier_size: int = 3
    sensor_range_m: float = 5.0
    candidate_radius_m: float = 2.5
    min_goal_distance_m: float = 0.8
    novelty_radius_m: float = 1.0
    max_candidates: int = 16
    coverage_resolution_m: float = 0.5
    local_route_radius_m: float = 12.0
    return_home_distance_m: float = 0.75
    keypose_min_distance_m: float = 0.75
    keypose_connect_distance_m: float = 6.0
    gain_weight: float = 1.0
    travel_weight: float = 0.20
    momentum_weight: float = 2.0
    revisit_weight: float = 3.0
    max_plan_time_ms: float = 100.0
    route_2opt_iterations: int = 4
    max_grid_cells: int = 1_000_000
    max_frontier_cells: int = 200_000
    max_frontier_clusters: int = 512
    max_coverage_cells: int = 500_000
    max_keyposes: int = 4096
    max_keypose_edges: int = 16384
    max_keypose_neighbor_links: int = 6
    max_route_targets: int = 32
    return_home_when_done: bool = True


@dataclass(frozen=True)
class TAREDecision:
    goal: tuple[float, float, float] | None
    path: list[tuple[float, float, float]] = field(default_factory=list)
    reason: str = ""
    candidates: list[dict[str, Any]] = field(default_factory=list)
    done: bool = False
    diagnostics: dict[str, Any] = field(default_factory=dict)


class PortableTAREPolicy:
    """Select the next exploration viewpoint via the native C++ TARE policy.

    The native backend is created lazily on first ``select()`` so that merely
    constructing the policy (e.g. inside ``TAREExplorerModule``) does not require
    the compiled ``lingtu_explore_kernel`` extension.
    """

    def __init__(self, config: TAREPolicyConfig | None = None) -> None:
        self.config = config or TAREPolicyConfig()
        self._backend: Any | None = None

    def _get_backend(self) -> Any:
        if self._backend is None:
            from explore.tare.backend import create_nanobind_explore_backend

            self._backend = create_nanobind_explore_backend(self.config)
        return self._backend

    def select(
        self,
        *,
        grid_payload: dict[str, Any],
        robot_xy: tuple[float, float],
        robot_yaw: float = 0.0,
        visited_goals: list[tuple[float, float]] | None = None,
    ) -> TAREDecision:
        backend = self._get_backend()
        decision = backend.plan(
            grid_payload=grid_payload,
            robot_xy=robot_xy,
            robot_yaw=robot_yaw,
            visited_goals=visited_goals,
        )
        return _to_tare_decision(decision, robot_xy)


def _to_tare_decision(
    decision: Any,
    robot_xy: tuple[float, float],
) -> TAREDecision:
    """Convert a native ExploreDecision to the Python TAREDecision."""
    native_diagnostics = getattr(decision, "diagnostics", None)
    diagnostics = {
        name: getattr(native_diagnostics, name)
        for name in (
            "phase",
            "frame_id",
            "session_id",
            "map_id",
            "map_content_epoch",
            "reset_epoch",
            "generation",
            "accepted_generation",
            "reachable_free_cells",
            "frontier_cells",
            "frontier_clusters",
            "keypose_nodes",
            "keypose_edges",
            "covered_cells",
            "route_targets",
            "reset_count",
            "route_length_m",
            "planning_time_ms",
            "state_committed",
        )
        if native_diagnostics is not None and hasattr(native_diagnostics, name)
    }
    if not getattr(decision, "has_goal", False):
        return TAREDecision(
            None,
            reason=str(getattr(decision, "reason", "")),
            done=bool(getattr(decision, "done", False)),
            diagnostics=diagnostics,
        )

    goal = (
        float(decision.goal_x),
        float(decision.goal_y),
        float(decision.goal_z),
    )
    route = [
        (float(pose.x), float(pose.y), 0.0)
        for pose in getattr(decision, "route", ())
    ]
    if not route or route[-1][:2] != goal[:2]:
        route.append(goal)
    path = [(float(robot_xy[0]), float(robot_xy[1]), 0.0), *route]
    candidates = [
        {
            "x": float(candidate.x),
            "y": float(candidate.y),
            "score": float(candidate.score),
            "frontier_size": int(candidate.frontier_size),
            "covered_frontier_cells": int(candidate.covered_frontier_cells),
            "distance_m": float(candidate.distance_m),
            "route_cost_m": float(candidate.route_cost_m),
            "revisit_penalty": float(candidate.revisit_penalty),
            "cluster_id": int(candidate.cluster_id),
        }
        for candidate in decision.candidates
    ]
    return TAREDecision(
        goal,
        path=path,
        reason=str(decision.reason),
        candidates=candidates,
        done=bool(getattr(decision, "done", False)),
        diagnostics=diagnostics,
    )

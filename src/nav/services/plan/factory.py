"""Planner service factory for nav.navigation."""

from __future__ import annotations

from typing import Any

from nav.services.plan.contracts import PlannerService
from runtime.profiles.planner_backends import normalize_planner_name


def create_planner_service(
    *,
    planner_name: str = "octoplanner3d",
    map_path: str = "",
    obstacle_thr: float = 49.9,
    downsample_dist: float = 2.0,
    plan_safety_policy: str = "observe",
    fallback_planner_name: str = "",
    expected_saved_map_frame_id: str | None = None,
    map_artifact_gate_required: bool | None = None,
    octoplanner3d_constraints: dict[str, Any] | None = None,
    octoplanner3d_timeout_s: float | None = None,
) -> PlannerService:
    """Return the planner service for a runtime profile."""

    canonical_planner_name = normalize_planner_name(planner_name)
    canonical_fallback_planner_name = normalize_planner_name(fallback_planner_name)

    if canonical_planner_name == "direct":
        from nav.services.plan.compat.direct import MaplessDirectPlannerService

        return MaplessDirectPlannerService(
            planner_name=canonical_planner_name,
            map_path=map_path,
            obstacle_thr=obstacle_thr,
            downsample_dist=downsample_dist,
            plan_safety_policy=plan_safety_policy,
            fallback_planner_name="",
            expected_saved_map_frame_id=expected_saved_map_frame_id,
        )

    from nav.services.plan.global_planner.service import GlobalPlanner

    return GlobalPlanner(
        planner_name=canonical_planner_name,
        map_path=map_path,
        obstacle_thr=obstacle_thr,
        downsample_dist=downsample_dist,
        plan_safety_policy=plan_safety_policy,
        fallback_planner_name=canonical_fallback_planner_name,
        expected_saved_map_frame_id=expected_saved_map_frame_id,
        map_artifact_gate_required=map_artifact_gate_required,
        octoplanner3d_constraints=octoplanner3d_constraints,
        octoplanner3d_timeout_s=octoplanner3d_timeout_s,
    )

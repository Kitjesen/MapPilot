"""Mission status payload published by navigation."""

from __future__ import annotations

import time
from typing import Any, TypedDict

from nav.mission.model.geometry import distance_xy, point_summary
from nav.mission.model.state import (
    MissionMode,
    MissionModeName,
    MissionStateInput,
    MissionStateName,
    MissionTransitionRejected,
    mission_state_name,
)
from nav.services.plan.preview import (
    planner_map_artifact_gate,
    planner_plan_safety_policy,
    planner_service_name,
)


class MissionStatus(TypedDict, total=False):
    state: MissionStateName
    mission_mode: MissionModeName
    phase_reason: str
    last_rejected_transition: MissionTransitionRejected | None
    replan_count: int
    wp_index: int
    wp_total: int
    remaining_waypoints: int
    planning_frame_id: str
    odom_frame_id: str
    costmap_frame_id: str | None
    goal_frame_id: str | None
    goal: list[float] | None
    current_waypoint: list[float] | None
    distance_to_goal_m: float | None
    active_waypoint_distance_m: float | None
    speed_scale: float
    speed_policy: dict[str, Any]
    plan_safety_policy: str
    replan_on_costmap_update: bool
    last_plan_report: dict[str, Any]
    global_plan: dict[str, Any] | None
    map_artifact_gate: dict[str, Any]
    direct_goal_fallback: dict[str, Any] | None
    path_start_anchor: dict[str, Any]
    allow_path_start_insert: bool
    external_strategy_path_control: bool
    using_external_strategy_path: bool
    accept_partial_goal_progress: bool
    degeneracy: str
    failure_reason: str
    localization_paused: bool
    localization_recovery_motion_hold: bool
    ts: float


def nonnegative_int(value: Any) -> int:
    try:
        return max(0, int(value))
    except (TypeError, ValueError):
        return 0


def build_speed_policy(speed_scale: float, reason: str) -> dict[str, Any]:
    mode = "restricted" if speed_scale < 0.5 else "cautious" if speed_scale < 1.0 else "normal"
    return {
        "scale": speed_scale,
        "mode": mode,
        "reason": reason,
        "source": "localization_degeneracy",
        "applied": speed_scale < 1.0,
    }


def build_mission_status(
    *,
    state: MissionStateInput,
    mission_mode: MissionMode,
    phase_reason: str,
    last_rejected_transition: MissionTransitionRejected | None,
    replan_count: int,
    tracker: Any,
    planning_frame_id: str,
    odom_frame_id: str,
    costmap_frame_id: str | None,
    goal_frame_id: str | None,
    goal: Any,
    robot_pos: Any,
    speed_scale: float,
    speed_policy_reason: str,
    planner_svc: Any,
    replan_on_costmap_update: bool,
    last_plan_report: dict[str, Any],
    global_plan: dict[str, Any] | None,
    direct_goal_fallback: dict[str, Any] | None,
    path_start_anchor: dict[str, Any],
    allow_path_start_insert: bool,
    external_strategy_path_control: bool,
    using_external_strategy_path: bool,
    accept_partial_goal_progress: bool,
    degeneracy: str,
    failure_reason: str,
    localization_paused: bool,
    localization_recovery_motion_hold: bool,
) -> MissionStatus:
    current_waypoint = getattr(tracker, "current_waypoint", None)
    wp_index = nonnegative_int(getattr(tracker, "wp_index", 0))
    wp_total = nonnegative_int(getattr(tracker, "path_length", 0))
    return {
        "state": mission_state_name(state),
        "mission_mode": mission_mode.value,
        "phase_reason": phase_reason,
        "last_rejected_transition": last_rejected_transition,
        "replan_count": replan_count,
        "wp_index": wp_index,
        "wp_total": wp_total,
        "remaining_waypoints": max(0, wp_total - wp_index),
        "planning_frame_id": planning_frame_id,
        "odom_frame_id": odom_frame_id,
        "costmap_frame_id": costmap_frame_id,
        "goal_frame_id": goal_frame_id,
        "goal": point_summary(goal),
        "current_waypoint": point_summary(current_waypoint),
        "distance_to_goal_m": distance_xy(robot_pos, goal),
        "active_waypoint_distance_m": distance_xy(robot_pos, current_waypoint),
        "speed_scale": speed_scale,
        "speed_policy": build_speed_policy(speed_scale, speed_policy_reason),
        "plan_safety_policy": planner_plan_safety_policy(planner_svc, "observe"),
        "replan_on_costmap_update": replan_on_costmap_update,
        "last_plan_report": last_plan_report,
        "global_plan": global_plan,
        "map_artifact_gate": planner_map_artifact_gate(planner_svc),
        "direct_goal_fallback": direct_goal_fallback,
        "path_start_anchor": dict(path_start_anchor),
        "allow_path_start_insert": allow_path_start_insert,
        "external_strategy_path_control": external_strategy_path_control,
        "using_external_strategy_path": using_external_strategy_path,
        "accept_partial_goal_progress": accept_partial_goal_progress,
        "degeneracy": degeneracy,
        "failure_reason": failure_reason,
        "localization_paused": localization_paused,
        "localization_recovery_motion_hold": localization_recovery_motion_hold,
        "ts": time.time(),
    }


def build_navigation_status_payload(nav: Any) -> dict[str, Any]:
    goal = nav._get_goal()
    return {
        "state": mission_state_name(nav._get_state()),
        "mission_mode": nav._mission_mode.value,
        "frame_id": nav._planning_frame_id,
        "planning_frame_id": nav._planning_frame_id,
        "odom_frame_id": nav._odom_frame_id,
        "costmap_frame_id": nav._costmap_frame_id,
        "position": {
            "x": round(float(nav._robot_pos[0]), 3),
            "y": round(float(nav._robot_pos[1]), 3),
            "z": round(float(nav._robot_pos[2]), 3),
            "yaw": round(float(nav._robot_yaw), 3),
        },
        "goal": (
            {
                "x": round(float(goal[0]), 3),
                "y": round(float(goal[1]), 3),
                "z": round(float(goal[2]), 3) if len(goal) > 2 else 0.0,
            }
            if goal is not None
            else None
        ),
        "path_length": getattr(nav._tracker, "path_length", 0),
        "waypoint_index": getattr(nav._tracker, "wp_index", 0),
        "complete_path_on_goal_proximity": nav._complete_path_on_goal_proximity,
        "goal_proximity_completion_threshold": nav._goal_proximity_completion_threshold,
        "replan_count": nav._get_replan_count(),
        "failure_reason": nav._get_failure_reason(),
        "plan_safety_policy": planner_plan_safety_policy(nav._planner_svc, "observe"),
        "replan_on_costmap_update": nav._replan_on_costmap_update,
        "last_plan_report": nav._current_plan_report(),
        "global_plan": nav._last_global_plan,
        "map_artifact_gate": planner_map_artifact_gate(nav._planner_svc),
        "direct_goal_fallback": nav._direct_goal_fallback_status,
        "path_start_anchor": dict(nav._path_start_anchor_status),
        "allow_path_start_insert": nav._allow_path_start_insert,
    }


def build_navigation_health(nav: Any, port_summary: dict[str, Any]) -> dict[str, Any]:
    info = dict(port_summary)
    info["planner_backend"] = nav.planner_backend_status()
    info["navigation"] = {
        "planner": planner_service_name(nav._planner_svc),
        "plan_safety_policy": planner_plan_safety_policy(nav._planner_svc, "observe"),
        "last_plan_report": nav._current_plan_report(),
        "global_plan": nav._last_global_plan,
        "map_artifact_gate": planner_map_artifact_gate(nav._planner_svc),
        "state": mission_state_name(nav._get_state()),
        "wp_index": getattr(nav._tracker, "wp_index", 0),
        "wp_total": getattr(nav._tracker, "path_length", 0),
        "complete_path_on_goal_proximity": nav._complete_path_on_goal_proximity,
        "goal_proximity_completion_threshold": nav._goal_proximity_completion_threshold,
        "replan_count": nav._get_replan_count(),
        "failure_reason": nav._get_failure_reason(),
        "replan_on_costmap_update": nav._replan_on_costmap_update,
        "direct_goal_fallback": nav._direct_goal_fallback_status,
        "path_start_anchor": dict(nav._path_start_anchor_status),
        "allow_path_start_insert": nav._allow_path_start_insert,
        "patrol_index": nav._patrol_index if nav._patrol_goals else -1,
        "patrol_total": len(nav._patrol_goals),
    }
    return info

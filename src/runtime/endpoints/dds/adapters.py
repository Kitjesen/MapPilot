"""DDS payload adapters for Python Module messages."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from runtime.msgs import (
    ExplorationRunEvent,
    InspectionTaskEvent,
    JointState,
    NavigationGoalStatus,
    NavigationState,
)


def joint_state_from_dds(payload: Mapping[str, Any]) -> JointState:
    return JointState(
        ts=float(payload["timestamp_s"]),
        robot_model=payload["robot_model"],
        names=list(payload["names"]),
        position=[float(value) for value in payload["position"]],
        velocity=[float(value) for value in payload["velocity"]],
        effort=[float(value) for value in payload["effort"]],
    )


def navigation_state_from_dds(payload: Mapping[str, Any]) -> NavigationState:
    return NavigationState(
        ts=float(payload["timestamp_s"]),
        frame_id=str(payload["frame_id"]),
        boot_id=str(payload["boot_id"]),
        sequence=int(payload["sequence"]),
        control_mode=int(payload["control_mode"]),
        lifecycle_state=int(payload["lifecycle_state"]),
        active_task_id=str(payload["active_task_id"]),
        active_request_id=str(payload["active_request_id"]),
        goal_epoch=int(payload["goal_epoch"]),
        map_id=str(payload["map_id"]),
        map_content_epoch=int(payload["map_content_epoch"]),
        planning_state=int(payload["planning_state"]),
        execution_state=int(payload["execution_state"]),
        recovery_state=int(payload["recovery_state"]),
        progress=float(payload["progress"]),
        authority=str(payload["authority"]),
        hold_reason=str(payload["hold_reason"]),
        failure_code=str(payload["failure_code"]),
    )


def navigation_goal_status_from_dds(
    payload: Mapping[str, Any],
) -> NavigationGoalStatus:
    return NavigationGoalStatus(
        ts=float(payload["timestamp_s"]),
        frame_id=str(payload["frame_id"]),
        boot_id=str(payload["boot_id"]),
        sequence=int(payload["sequence"]),
        task_id=str(payload["task_id"]),
        request_id=str(payload["request_id"]),
        state=int(payload["state"]),
        goal_epoch=int(payload["goal_epoch"]),
        reason=str(payload["reason"]),
    )


def inspection_task_event_from_dds(
    payload: Mapping[str, Any],
) -> InspectionTaskEvent:
    return InspectionTaskEvent(
        ts=float(payload["timestamp_s"]),
        frame_id=str(payload["frame_id"]),
        boot_id=str(payload["boot_id"]),
        event_sequence=int(payload["event_sequence"]),
        kind=int(payload["kind"]),
        task_id=str(payload["task_id"]),
        request_id=str(payload["request_id"]),
        command_request_id=str(payload["command_request_id"]),
        state=int(payload["state"]),
        map_id=str(payload["map_id"]),
        map_content_epoch=int(payload["map_content_epoch"]),
        route_id=str(payload["route_id"]),
        route_revision=int(payload["route_revision"]),
        point_index=int(payload["point_index"]),
        point_count=int(payload["point_count"]),
        loop_index=int(payload["loop_index"]),
        retry_count=int(payload["retry_count"]),
        point_id=str(payload["point_id"]),
        action=str(payload["action"]),
        action_request_id=str(payload["action_request_id"]),
        evidence_id=str(payload["evidence_id"]),
        reason=str(payload["reason"]),
    )


def exploration_run_event_from_dds(
    payload: Mapping[str, Any],
) -> ExplorationRunEvent:
    return ExplorationRunEvent(
        ts=float(payload["timestamp_s"]),
        frame_id=str(payload["frame_id"]),
        boot_id=str(payload["boot_id"]),
        event_sequence=int(payload["event_sequence"]),
        kind=int(payload["kind"]),
        exploration_run_id=str(payload["exploration_run_id"]),
        start_request_id=str(payload["start_request_id"]),
        command_request_id=str(payload["command_request_id"]),
        product_session_id=str(payload["product_session_id"]),
        state=int(payload["state"]),
        route=str(payload["route"]),
        map_id=str(payload["map_id"]),
        map_content_epoch=int(payload["map_content_epoch"]),
        reason=str(payload["reason"]),
        motion_stop_confirmed=payload["motion_stop_confirmed"],
        motion_stop_reason=str(payload["motion_stop_reason"]),
    )


__all__ = [
    "exploration_run_event_from_dds",
    "inspection_task_event_from_dds",
    "joint_state_from_dds",
    "navigation_goal_status_from_dds",
    "navigation_state_from_dds",
]

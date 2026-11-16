"""Classify navigation health facts by their effect on robot motion."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from enum import IntEnum
from typing import Any


class NavigationErrorCode(IntEnum):
    """Stable numeric codes for navigation health facts."""

    MOTION_UPDATE_SLOW = 21001
    MOTION_PROCESSING_FAILED = 21002
    CONTROL_CONSECUTIVE_DEADLINES = 21101
    CONTROL_DEADLINE_RATIO = 21102
    CONTROL_P95_UTILIZATION = 21103
    CONTROL_UNKNOWN = 21104
    INPUT_GATE_NOT_READY = 21201
    RESUME_REQUIRED = 21301
    ESTOP_LATCHED = 21401


@dataclass(frozen=True)
class ErrorDefinition:
    code: NavigationErrorCode
    name: str
    category: str
    impact: str
    severity: str
    description: str
    operator_action: str


ERROR_DEFINITIONS = {
    item.code: item
    for item in (
        ErrorDefinition(
            NavigationErrorCode.MOTION_UPDATE_SLOW,
            "motion_update_over_period",
            "navigation.motion_worker",
            "diagnostic_only",
            "info",
            "The last asynchronous obstacle-worker update exceeded the control period.",
            "Inspect control-loop overrun and input freshness before changing motion policy.",
        ),
        ErrorDefinition(
            NavigationErrorCode.MOTION_PROCESSING_FAILED,
            "motion_processing_failed",
            "navigation.motion_worker",
            "motion_degrading",
            "warning",
            "An obstacle-worker frame failed and did not refresh the current snapshot.",
            "Check the worker detail and wait for a fresh collision snapshot.",
        ),
        ErrorDefinition(
            NavigationErrorCode.CONTROL_CONSECUTIVE_DEADLINES,
            "consecutive_deadline_misses",
            "navigation.control_loop",
            "motion_degrading",
            "warning",
            "The native control loop missed consecutive cycle deadlines.",
            "Inspect the retained timing history before supervised motion.",
        ),
        ErrorDefinition(
            NavigationErrorCode.CONTROL_DEADLINE_RATIO,
            "deadline_miss_ratio_high",
            "navigation.control_loop",
            "motion_degrading",
            "warning",
            "The rolling control-loop deadline-miss ratio exceeded its limit.",
            "Review P99 overrun and workload before motion acceptance.",
        ),
        ErrorDefinition(
            NavigationErrorCode.CONTROL_P95_UTILIZATION,
            "p95_utilization_high",
            "navigation.control_loop",
            "motion_degrading",
            "warning",
            "The rolling P95 control-loop work utilization exceeded its limit.",
            "Reduce control-thread work or inspect CPU contention.",
        ),
        ErrorDefinition(
            NavigationErrorCode.CONTROL_UNKNOWN,
            "control_loop_unhealthy",
            "navigation.control_loop",
            "motion_degrading",
            "warning",
            "The native control loop reports an unhealthy state without a known reason.",
            "Read the native timing history and status reason.",
        ),
        ErrorDefinition(
            NavigationErrorCode.INPUT_GATE_NOT_READY,
            "input_gate_not_ready",
            "navigation.input_gate",
            "motion_hold",
            "warning",
            "Required navigation input is missing, stale, future-dated, or invalid.",
            "Restore fresh odometry, TF, collision, localization, or terrain input.",
        ),
        ErrorDefinition(
            NavigationErrorCode.RESUME_REQUIRED,
            "resume_required",
            "navigation.control_authority",
            "motion_hold",
            "warning",
            "Operator takeover or resume confirmation is still active.",
            "Complete the existing resume transaction.",
        ),
        ErrorDefinition(
            NavigationErrorCode.ESTOP_LATCHED,
            "estop_latched",
            "navigation.control_authority",
            "motion_blocking",
            "error",
            "The software emergency stop is latched.",
            "Complete the estop release transaction before sending motion.",
        ),
    )
}


def _number(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _issue(
    definition: ErrorDefinition,
    source: str,
    reason: str,
    impact: str,
    severity: str,
    message: str,
    **details: Any,
) -> dict[str, Any]:
    return {
        "code": int(definition.code),
        "name": definition.name,
        "source": source or definition.category,
        "reason": reason,
        "impact": impact or definition.impact,
        "severity": severity or definition.severity,
        "message": message or definition.description,
        "operator_action": definition.operator_action,
        "active": True,
        **details,
    }


def navigation_health_issues(snapshot: Mapping[str, Any] | None) -> list[dict[str, Any]]:
    """Return stable issue codes without changing the native motion owner."""

    if not isinstance(snapshot, Mapping):
        return []
    issues: list[dict[str, Any]] = []
    loop = snapshot.get("control_loop_health")
    loop = loop if isinstance(loop, Mapping) else {}
    if loop.get("ready") is True and loop.get("healthy") is False:
        reason = str(loop.get("reason") or "unhealthy")
        definition = {
            "consecutive_deadline_misses": ERROR_DEFINITIONS[NavigationErrorCode.CONTROL_CONSECUTIVE_DEADLINES],
            "deadline_miss_ratio_high": ERROR_DEFINITIONS[NavigationErrorCode.CONTROL_DEADLINE_RATIO],
            "p95_utilization_high": ERROR_DEFINITIONS[NavigationErrorCode.CONTROL_P95_UTILIZATION],
        }.get(reason, ERROR_DEFINITIONS[NavigationErrorCode.CONTROL_UNKNOWN])
        issues.append(_issue(
            definition,
            "",
            reason,
            "",
            "",
            "控制循环出现超时或工作量过高，当前状态只产生诊断告警；输入门禁仍单独决定是否保持零速度。",
            period_ms=_number(loop.get("period_ms")),
            deadline_miss_ratio=_number(loop.get("deadline_miss_ratio")),
            p95_utilization=_number(loop.get("p95_utilization")),
        ))

    frame_gate = snapshot.get("frame_gate")
    frame_gate = frame_gate if isinstance(frame_gate, Mapping) else {}
    frame_error = str(frame_gate.get("motion_worker_error") or "")
    if frame_error:
        issues.append(_issue(
            ERROR_DEFINITIONS[NavigationErrorCode.MOTION_PROCESSING_FAILED],
            "",
            "motion_processing_failed",
            "",
            "",
            "障碍处理线程处理一帧失败，当前快照不会被刷新；若输入继续过期，输入门禁会保持零速度。",
            detail=frame_error,
        ))

    input_gate = snapshot.get("input_gate")
    input_gate = input_gate if isinstance(input_gate, Mapping) else {}
    if input_gate.get("ready") is False:
        reason = str(input_gate.get("reason") or "input_gate_not_ready")
        issues.append(_issue(
            ERROR_DEFINITIONS[NavigationErrorCode.INPUT_GATE_NOT_READY],
            "",
            reason,
            "",
            "",
            "导航输入门禁未就绪，最终运动输出会保持零速度，等待新鲜且有效的输入。",
        ))

    authority = snapshot.get("control_authority")
    authority = authority if isinstance(authority, Mapping) else {}
    if authority.get("estop_latched") is True:
        issues.append(_issue(
            ERROR_DEFINITIONS[NavigationErrorCode.ESTOP_LATCHED],
            "",
            "estop_latched",
            "",
            "",
            "软件急停已锁定，运动命令被阻止。",
        ))
    elif authority.get("resume_required") is True:
        issues.append(_issue(
            ERROR_DEFINITIONS[NavigationErrorCode.RESUME_REQUIRED],
            "",
            "resume_required",
            "",
            "",
            "当前控制权要求恢复确认，机器人保持停止，完成恢复后才会继续。",
        ))

    return issues


def navigation_health_advisories(snapshot: Mapping[str, Any] | None) -> list[dict[str, Any]]:
    """Return metrics that need attention without changing motion status."""

    if not isinstance(snapshot, Mapping):
        return []
    loop = snapshot.get("control_loop_health")
    loop = loop if isinstance(loop, Mapping) else {}
    timing = snapshot.get("timing_ms")
    timing = timing if isinstance(timing, Mapping) else {}
    period_ms = _number(loop.get("period_ms"))
    motion_update_ms = _number(timing.get("motion_update_last"))
    if period_ms is None or motion_update_ms is None or motion_update_ms <= period_ms:
        return []
    return [_issue(
        ERROR_DEFINITIONS[NavigationErrorCode.MOTION_UPDATE_SLOW],
        "",
        "motion_update_over_period",
        "",
        "",
        "异步障碍处理最近一次耗时超过控制周期，仅作为诊断提示；需要同时查看控制循环 overrun 和输入新鲜度。",
        duration_ms=motion_update_ms,
        period_ms=period_ms,
        worker_result="last_completed_snapshot",
    )]


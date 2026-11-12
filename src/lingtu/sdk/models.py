"""Public response values and their Gateway payload parsers."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any


@dataclass
class Position:
    """Robot position in the map frame."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class Pose2D:
    """Planar pose used as a localization seed."""

    x: float
    y: float
    yaw: float


@dataclass
class HealthStatus:
    """System health overview."""

    modules_ok: int = 0
    modules_fail: int = 0
    modules_total: int = 0
    slam_rate: float = 0.0
    mode: str = ""
    session: str = ""
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class NavigationTask:
    """Current navigation task lifecycle."""

    state: str = "UNKNOWN"
    task_id: str = ""
    reason: str = ""


@dataclass
class NavigationGoalAdmission:
    """Whether the Gateway can accept or replace a navigation goal."""

    state: str = "UNKNOWN"
    reason: str = ""


@dataclass
class NavigationControlState:
    """Current navigation control authority."""

    authority: str = "UNKNOWN"
    resume_required: bool = False
    reason: str = ""


@dataclass
class NavigationMotionState:
    """Motion permission, observation, and stop-confirmation evidence."""

    permission: str = "UNKNOWN"
    observation: str = "UNKNOWN"
    stop_confirmation: str = "UNKNOWN"
    reason: str = ""


@dataclass
class NavigationStatus:
    """The four public navigation state axes."""

    task: NavigationTask = field(default_factory=NavigationTask)
    goal_admission: NavigationGoalAdmission = field(default_factory=NavigationGoalAdmission)
    control: NavigationControlState = field(default_factory=NavigationControlState)
    motion: NavigationMotionState = field(default_factory=NavigationMotionState)
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class MapInfo:
    """Summary of a single saved map."""

    name: str = ""
    has_pcd: bool = False
    is_active: bool = False
    size_bytes: int = 0
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class MapList:
    """Response from the maps endpoint."""

    maps: list[MapInfo] = field(default_factory=list)
    active_map: str | None = None
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class SessionInfo:
    """Current robot session state."""

    mode: str = "idle"
    active_map: str | None = None
    slam_profile: str | None = None
    raw: dict[str, Any] = field(default_factory=dict)
    env: str | None = None
    product: str | None = None
    product_session_id: str | None = None


@dataclass
class CommandResult:
    """Command admission receipt (not mission-completion evidence)."""

    ok: bool = False
    message: str = ""
    raw: dict[str, Any] = field(default_factory=dict)
    operation_id: str | None = None
    accepted: bool = False
    request_id: str | None = None
    stage: str = ""
    task_id: str | None = None
    follow_task_id: str | None = None
    product_session_id: str | None = None
    run_id: str | None = None


@dataclass
class RobotState:
    """Full robot state snapshot."""

    mode: str = ""
    odometry: Position | None = None
    mission: NavigationStatus = field(default_factory=NavigationStatus)
    safety_level: str = ""
    raw: dict[str, Any] = field(default_factory=dict)


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _parse_navigation_status(raw: dict[str, Any]) -> NavigationStatus:
    task = _mapping(raw.get("task"))
    admission = _mapping(raw.get("goal_admission"))
    control = _mapping(raw.get("control"))
    motion = _mapping(raw.get("motion"))
    return NavigationStatus(
        task=NavigationTask(
            state=str(task.get("state") or "UNKNOWN"),
            task_id=str(task.get("task_id") or ""),
            reason=str(task.get("reason") or ""),
        ),
        goal_admission=NavigationGoalAdmission(
            state=str(admission.get("state") or "UNKNOWN"),
            reason=str(admission.get("reason") or ""),
        ),
        control=NavigationControlState(
            authority=str(control.get("authority") or "UNKNOWN"),
            resume_required=control.get("resume_required") is True,
            reason=str(control.get("reason") or ""),
        ),
        motion=NavigationMotionState(
            permission=str(motion.get("permission") or "UNKNOWN"),
            observation=str(motion.get("observation") or "UNKNOWN"),
            stop_confirmation=str(motion.get("stop_confirmation") or "UNKNOWN"),
            reason=str(motion.get("reason") or ""),
        ),
        raw=dict(raw),
    )


def _parse_command_result(raw: dict[str, Any]) -> CommandResult:
    command = _mapping(raw.get("command"))
    accepted_signal = command.get("accepted", raw.get("accepted"))
    if accepted_signal is None:
        accepted = raw.get("ok") is True or raw.get("success") is True
    else:
        accepted = accepted_signal is True

    if "ok" in raw:
        response_ok = raw.get("ok") is True
    elif "success" in raw:
        response_ok = raw.get("success") is True
    else:
        response_ok = accepted

    request_id = str(command.get("request_id") or raw.get("request_id") or "").strip() or None
    task_id = str(command.get("task_id") or raw.get("task_id") or "").strip() or None
    follow_task_id = str(command.get("follow_task_id") or raw.get("follow_task_id") or "").strip() or None
    product_session_id = str(command.get("product_session_id") or raw.get("product_session_id") or "").strip() or None
    stage = str(raw.get("stage") or raw.get("status") or command.get("stage") or "")
    operation_id = str(command.get("operation_id") or raw.get("operation_id") or "").strip() or None
    run_id = str(raw.get("run_id") or "").strip() or None
    return CommandResult(
        ok=response_ok and accepted,
        accepted=accepted,
        message=str(raw.get("message", raw.get("reason", raw.get("error", ""))) or ""),
        request_id=request_id,
        task_id=task_id,
        stage=stage,
        raw=raw,
        operation_id=operation_id,
        follow_task_id=follow_task_id,
        product_session_id=product_session_id,
        run_id=run_id,
    )

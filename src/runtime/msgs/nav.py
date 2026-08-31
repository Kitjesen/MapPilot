"""lingtu.runtime.msgs.nav — navigation message types (Odometry, Path, OccupancyGrid).

Follows dimos nav_msgs design, aligned with LingTu /nav/* ROS2 topic contracts.
"""

from __future__ import annotations

import math
import re
import struct
import time
from collections.abc import Iterator
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, ClassVar

from runtime.runtime_interface import body_frame_id, map_frame_id, odom_frame_id

from .geometry import Pose, PoseStamped, Twist
from .numpy_compat import is_numpy_array, np, numpy_import_is_safe

NAV_MAP_FRAME_ID = map_frame_id()
NAV_ODOM_FRAME_ID = odom_frame_id()
NAV_BODY_FRAME_ID = body_frame_id()


class NavigationControlMode(IntEnum):
    UNKNOWN = 0
    AUTONOMY = 1
    TELEOP = 2
    TELEOP_AVOID = 3


class NavigationLifecycle(IntEnum):
    IDLE = 0
    PLANNING = 1
    EXECUTING = 2
    PAUSED = 3
    RECOVERING = 4
    SUCCESS = 5
    FAILED = 6
    CANCELLED = 7


class NavigationPlanningState(IntEnum):
    IDLE = 0
    PLANNING = 1
    READY = 2
    FAILED = 3


class NavigationExecutionState(IntEnum):
    IDLE = 0
    FOLLOWING = 1
    REACHED = 2
    BLOCKED = 3


class NavigationRecoveryState(IntEnum):
    IDLE = 0
    ACTIVE = 1
    SUCCEEDED = 2
    FAILED = 3


class NavigationGoalState(IntEnum):
    PLANNING = 1
    PATH_ACTIVE = 2
    FAILED = 3
    REACHED = 4
    CANCELLED = 5
    PAUSED = 6


class InspectionTaskEventKind(IntEnum):
    TASK_ACCEPTED = 1
    STATE_CHANGED = 2
    MILESTONE = 3
    STOP_CONFIRMATION_FAILED = 4
    EVIDENCE_RECORDED = 5


class InspectionTaskState(IntEnum):
    IDLE = 0
    VALIDATING = 1
    PLANNING = 2
    NAVIGATING = 3
    DWELLING = 4
    PAUSED = 5
    RECOVERING = 6
    SUCCEEDED = 7
    FAILED = 8
    CANCELLED = 9
    SETTLING = 10
    ACTION_PENDING = 11
    PAUSING = 12
    CANCELLING = 13


class ExplorationRunEventKind(IntEnum):
    """Kind of immutable lifecycle fact emitted by the native Explore endpoint."""

    ADMITTED = 1
    STATE_CHANGED = 2
    STOP_CONFIRMATION_FAILED = 3


class ExplorationRunState(IntEnum):
    """Authoritative state of one finite native Explore run."""

    ADMITTED = 1
    RUNNING = 2
    PAUSING = 3
    PAUSED = 4
    CANCELLING = 5
    COMPLETED = 6
    CANCELLED = 7
    FAILED = 8


class NavigationCommandKind(IntEnum):
    GOAL = 1
    TASK_CANCEL = 2
    STOP = 4
    ESTOP = 5
    CLEAR_ESTOP = 6
    RESUME_AUTONOMY = 7
    PAUSE_TASK = 8
    RESUME_TASK = 9


class OperatorMotionAction(IntEnum):
    CLAIM = 1
    RELEASE = 2
    HOLD = 3


def _require_int(value: int, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{field_name} must be an integer")
    return int(value)


def _enum_name(enum_type: type[IntEnum], value: int) -> str:
    try:
        return enum_type(int(value)).name
    except ValueError:
        return "UNKNOWN"


@dataclass(frozen=True)
class NavigationCommandReceipt:
    """Native business ACK preserving logical task and delivery-attempt identity."""

    msg_name: ClassVar[str] = "lingtu.runtime.NavigationCommandReceipt"

    accepted: bool
    kind: int
    task_id: str
    request_id: str
    endpoint_timestamp_s: float
    reason: str

    def __post_init__(self) -> None:
        NavigationCommandKind(
            _require_int(self.kind, "NavigationCommandReceipt.kind")
        )
        if not isinstance(self.accepted, bool):
            raise ValueError("NavigationCommandReceipt.accepted must be a boolean")
        if not isinstance(self.task_id, str) or not self.task_id.strip():
            raise ValueError("NavigationCommandReceipt.task_id is required")
        if not isinstance(self.request_id, str) or not self.request_id.strip():
            raise ValueError("NavigationCommandReceipt.request_id is required")
        if self.task_id == self.request_id:
            raise ValueError(
                "NavigationCommandReceipt.task_id and request_id must be distinct"
            )
        timestamp = float(self.endpoint_timestamp_s)
        if not math.isfinite(timestamp) or timestamp < 0.0:
            raise ValueError(
                "NavigationCommandReceipt.endpoint_timestamp_s must be finite and non-negative"
            )
        if not isinstance(self.reason, str):
            raise ValueError("NavigationCommandReceipt.reason must be a string")

    def to_dict(self) -> dict[str, Any]:
        return {
            "accepted": self.accepted,
            "kind": int(self.kind),
            "kind_name": _enum_name(NavigationCommandKind, self.kind),
            "task_id": self.task_id,
            "request_id": self.request_id,
            "endpoint_timestamp_s": float(self.endpoint_timestamp_s),
            "reason": self.reason,
        }


@dataclass(frozen=True)
class OperatorMotionReceipt:
    """Authoritative receipt for operator motion authority requests."""

    msg_name: ClassVar[str] = "lingtu.runtime.OperatorMotionReceipt"

    accepted: bool
    action: int
    request_id: str
    source_id: str
    source_epoch: int
    source_sequence: int
    accepted_sequence: int
    final_output_sequence: int
    endpoint_timestamp_s: float
    reason: str

    def __post_init__(self) -> None:
        OperatorMotionAction(_require_int(self.action, "OperatorMotionReceipt.action"))
        if not isinstance(self.accepted, bool):
            raise ValueError("OperatorMotionReceipt.accepted must be a boolean")
        if not isinstance(self.request_id, str) or not self.request_id.strip():
            raise ValueError("OperatorMotionReceipt.request_id is required")
        if not isinstance(self.source_id, str) or not self.source_id.strip():
            raise ValueError("OperatorMotionReceipt.source_id is required")

        source_epoch = _require_int(
            self.source_epoch,
            "OperatorMotionReceipt.source_epoch",
        )
        source_sequence = _require_int(
            self.source_sequence,
            "OperatorMotionReceipt.source_sequence",
        )
        accepted_sequence = _require_int(
            self.accepted_sequence,
            "OperatorMotionReceipt.accepted_sequence",
        )
        final_output_sequence = _require_int(
            self.final_output_sequence,
            "OperatorMotionReceipt.final_output_sequence",
        )
        if source_epoch <= 0 or source_sequence <= 0:
            raise ValueError(
                "OperatorMotionReceipt source_epoch and source_sequence must be positive"
            )
        if accepted_sequence < 0 or final_output_sequence < 0:
            raise ValueError("OperatorMotionReceipt sequences cannot be negative")
        if self.accepted and accepted_sequence != source_sequence:
            raise ValueError(
                "OperatorMotionReceipt.accepted_sequence must equal source_sequence "
                "when accepted"
            )
        if not self.accepted and accepted_sequence != 0:
            raise ValueError(
                "OperatorMotionReceipt.accepted_sequence must be 0 when rejected"
            )
        if not self.accepted and final_output_sequence != 0:
            raise ValueError(
                "OperatorMotionReceipt.final_output_sequence must be 0 when rejected"
            )
        timestamp = float(self.endpoint_timestamp_s)
        if not math.isfinite(timestamp) or timestamp <= 0.0:
            raise ValueError(
                "OperatorMotionReceipt.endpoint_timestamp_s must be a positive finite "
                "timestamp"
            )

    @property
    def source_accepted(self) -> bool:
        return bool(self.accepted) and int(self.accepted_sequence) == int(self.source_sequence)

    @property
    def final_output_published(self) -> bool:
        return (
            bool(self.accepted)
            and int(self.action)
            in {int(OperatorMotionAction.HOLD), int(OperatorMotionAction.RELEASE)}
            and int(self.final_output_sequence) > 0
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "accepted": bool(self.accepted),
            "action": int(self.action),
            "action_name": _enum_name(OperatorMotionAction, self.action),
            "request_id": self.request_id,
            "source_id": self.source_id,
            "source_epoch": int(self.source_epoch),
            "source_sequence": int(self.source_sequence),
            "accepted_sequence": int(self.accepted_sequence),
            "final_output_sequence": int(self.final_output_sequence),
            "endpoint_timestamp_s": float(self.endpoint_timestamp_s),
            "reason": self.reason,
            "source_accepted": self.source_accepted,
            "final_output_published": self.final_output_published,
        }


@dataclass(frozen=True)
class NavigationGoalStatus:
    """Task lifecycle event with its originating command-attempt identity."""

    msg_name: ClassVar[str] = "lingtu.dds.NavigationGoalStatus"

    ts: float = field(default_factory=time.time)
    frame_id: str = NAV_MAP_FRAME_ID
    boot_id: str = ""
    sequence: int = 0
    task_id: str = ""
    request_id: str = ""
    state: int = int(NavigationGoalState.PLANNING)
    goal_epoch: int = 0
    reason: str = ""

    def __post_init__(self) -> None:
        if not math.isfinite(float(self.ts)) or self.ts <= 0.0:
            raise ValueError("NavigationGoalStatus.ts must be a positive finite timestamp")
        if not self.frame_id or not self.boot_id or not self.task_id or not self.request_id:
            raise ValueError(
                "NavigationGoalStatus frame_id, boot_id, task_id, and request_id are required"
            )
        if int(self.sequence) <= 0:
            raise ValueError("NavigationGoalStatus.sequence must be positive")
        if int(self.goal_epoch) < 0:
            raise ValueError("NavigationGoalStatus.goal_epoch cannot be negative")
        NavigationGoalState(int(self.state))

    @property
    def terminal(self) -> bool:
        return int(self.state) in {
            int(NavigationGoalState.FAILED),
            int(NavigationGoalState.REACHED),
            int(NavigationGoalState.CANCELLED),
        }

    def to_dict(self) -> dict[str, Any]:
        return {
            "ts": float(self.ts),
            "frame_id": self.frame_id,
            "boot_id": self.boot_id,
            "sequence": int(self.sequence),
            "task_id": self.task_id,
            "request_id": self.request_id,
            "state": int(self.state),
            "state_name": _enum_name(NavigationGoalState, self.state),
            "goal_epoch": int(self.goal_epoch),
            "reason": self.reason,
            "terminal": self.terminal,
        }


@dataclass(frozen=True)
class InspectionTaskEvent:
    """Immutable native fact for one inspection task lifecycle transition."""

    msg_name: ClassVar[str] = "lingtu.dds.InspectionTaskEvent"

    ts: float = field(default_factory=time.time)
    frame_id: str = NAV_MAP_FRAME_ID
    boot_id: str = ""
    event_sequence: int = 0
    kind: int = int(InspectionTaskEventKind.STATE_CHANGED)
    task_id: str = ""
    request_id: str = ""
    command_request_id: str = ""
    state: int = int(InspectionTaskState.IDLE)
    map_id: str = ""
    map_content_epoch: int = 0
    route_id: str = ""
    route_revision: int = 0
    point_index: int = 0
    point_count: int = 0
    loop_index: int = 0
    retry_count: int = 0
    point_id: str = ""
    action: str = ""
    action_request_id: str = ""
    evidence_id: str = ""
    reason: str = ""

    def __post_init__(self) -> None:
        timestamp = float(self.ts)
        if not math.isfinite(timestamp) or timestamp < 0.0:
            raise ValueError("InspectionTaskEvent.ts must be finite and non-negative")
        required = {
            "frame_id": self.frame_id,
            "boot_id": self.boot_id,
            "task_id": self.task_id,
            "request_id": self.request_id,
            "command_request_id": self.command_request_id,
            "map_id": self.map_id,
            "route_id": self.route_id,
        }
        if any(not isinstance(value, str) or not value.strip() for value in required.values()):
            raise ValueError("InspectionTaskEvent identity and map fields are required")
        if _require_int(self.event_sequence, "InspectionTaskEvent.event_sequence") <= 0:
            raise ValueError("InspectionTaskEvent.event_sequence must be positive")
        InspectionTaskEventKind(_require_int(self.kind, "InspectionTaskEvent.kind"))
        InspectionTaskState(_require_int(self.state, "InspectionTaskEvent.state"))
        for field_name in (
            "map_content_epoch",
            "route_revision",
            "point_index",
            "point_count",
            "loop_index",
            "retry_count",
        ):
            if _require_int(getattr(self, field_name), f"InspectionTaskEvent.{field_name}") < 0:
                raise ValueError(f"InspectionTaskEvent.{field_name} cannot be negative")
        if int(self.route_revision) <= 0:
            raise ValueError("InspectionTaskEvent.route_revision must be positive")

    @property
    def terminal(self) -> bool:
        return int(self.state) in {
            int(InspectionTaskState.SUCCEEDED),
            int(InspectionTaskState.FAILED),
            int(InspectionTaskState.CANCELLED),
        }

    @property
    def state_name(self) -> str:
        return _enum_name(InspectionTaskState, self.state)

    @property
    def kind_name(self) -> str:
        return _enum_name(InspectionTaskEventKind, self.kind)

    def to_dict(self) -> dict[str, Any]:
        return {
            "ts": float(self.ts),
            "frame_id": self.frame_id,
            "boot_id": self.boot_id,
            "event_sequence": int(self.event_sequence),
            "kind": int(self.kind),
            "kind_name": self.kind_name,
            "task_id": self.task_id,
            "request_id": self.request_id,
            "command_request_id": self.command_request_id,
            "state": int(self.state),
            "state_name": self.state_name,
            "terminal": self.terminal,
            "map_id": self.map_id,
            "map_content_epoch": int(self.map_content_epoch),
            "route_id": self.route_id,
            "route_revision": int(self.route_revision),
            "point_index": int(self.point_index),
            "point_count": int(self.point_count),
            "loop_index": int(self.loop_index),
            "retry_count": int(self.retry_count),
            "point_id": self.point_id,
            "action": self.action,
            "action_request_id": self.action_request_id,
            "evidence_id": self.evidence_id,
            "reason": self.reason,
        }


_ULID_PATTERN = re.compile(r"^[0-7][0-9A-HJKMNP-TV-Z]{25}$")


@dataclass(frozen=True)
class ExplorationRunEvent:
    """One ordered native fact for a finite autonomous Explore run."""

    msg_name: ClassVar[str] = "lingtu.dds.ExplorationRunEvent"

    ts: float = field(default_factory=time.time)
    frame_id: str = NAV_MAP_FRAME_ID
    boot_id: str = ""
    event_sequence: int = 0
    kind: int = int(ExplorationRunEventKind.STATE_CHANGED)
    exploration_run_id: str = ""
    start_request_id: str = ""
    command_request_id: str = ""
    product_session_id: str = ""
    state: int = int(ExplorationRunState.ADMITTED)
    route: str = ""
    map_id: str = ""
    map_content_epoch: int = 0
    reason: str = ""
    motion_stop_confirmed: bool = False
    motion_stop_reason: str = ""

    def __post_init__(self) -> None:
        timestamp = float(self.ts)
        if not math.isfinite(timestamp) or timestamp <= 0.0:
            raise ValueError("ExplorationRunEvent.ts must be a positive finite timestamp")
        if self.frame_id != NAV_MAP_FRAME_ID:
            raise ValueError(
                f"ExplorationRunEvent.frame_id must be {NAV_MAP_FRAME_ID!r}"
            )
        required = {
            "boot_id": self.boot_id,
            "exploration_run_id": self.exploration_run_id,
            "start_request_id": self.start_request_id,
            "command_request_id": self.command_request_id,
            "product_session_id": self.product_session_id,
        }
        if any(not isinstance(value, str) or not value.strip() for value in required.values()):
            raise ValueError("ExplorationRunEvent lifecycle identity fields are required")
        if not _ULID_PATTERN.fullmatch(self.exploration_run_id):
            raise ValueError(
                "ExplorationRunEvent.exploration_run_id must be a canonical 26-character ULID"
            )
        if self.exploration_run_id in {
            self.start_request_id,
            self.command_request_id,
        }:
            raise ValueError(
                "ExplorationRunEvent run and request identities must be distinct"
            )
        if _require_int(self.event_sequence, "ExplorationRunEvent.event_sequence") <= 0:
            raise ValueError("ExplorationRunEvent.event_sequence must be positive")
        event_kind = ExplorationRunEventKind(
            _require_int(self.kind, "ExplorationRunEvent.kind")
        )
        state = ExplorationRunState(_require_int(self.state, "ExplorationRunEvent.state"))
        map_content_epoch = _require_int(self.map_content_epoch, "ExplorationRunEvent.map_content_epoch")
        if not isinstance(self.route, str) or self.route not in {"live", "map"}:
            raise ValueError("ExplorationRunEvent.route must be 'live' or 'map'")
        if not isinstance(self.map_id, str):
            raise ValueError("ExplorationRunEvent.map_id must be a string")
        if self.route == "map":
            if (
                not self.map_id.strip()
                or map_content_epoch <= 0
            ):
                raise ValueError(
                    "ExplorationRunEvent map identity requires map_id, positive "
                    "map_content_epoch"
                )
        elif self.map_id or map_content_epoch != 0:
            raise ValueError(
                "ExplorationRunEvent live route cannot claim a saved-map identity"
            )
        if (
            not isinstance(self.reason, str)
            or not self.reason.strip()
            or not isinstance(self.motion_stop_reason, str)
        ):
            raise ValueError(
                "ExplorationRunEvent.reason must be non-empty and "
                "motion_stop_reason must be a string"
            )
        if not isinstance(self.motion_stop_confirmed, bool):
            raise ValueError("ExplorationRunEvent.motion_stop_confirmed must be a boolean")
        if (
            not self.motion_stop_confirmed
            and self.motion_stop_reason.strip()
            and event_kind != ExplorationRunEventKind.STOP_CONFIRMATION_FAILED
        ):
            raise ValueError(
                "ExplorationRunEvent unconfirmed motion stop cannot carry a "
                "motion_stop_reason"
            )
        stopped_state = state in {
            ExplorationRunState.PAUSED,
            ExplorationRunState.COMPLETED,
            ExplorationRunState.CANCELLED,
            ExplorationRunState.FAILED,
        }
        if stopped_state and (
            not self.motion_stop_confirmed or not self.motion_stop_reason.strip()
        ):
            raise ValueError(
                "ExplorationRunEvent stopped states require motion_stop_confirmed "
                "and motion_stop_reason"
            )
        if (event_kind == ExplorationRunEventKind.ADMITTED) != (
            state == ExplorationRunState.ADMITTED
        ):
            raise ValueError(
                "ExplorationRunEvent ADMITTED kind and state must appear together"
            )
        if state in {
            ExplorationRunState.PAUSING,
            ExplorationRunState.CANCELLING,
        } and self.motion_stop_confirmed:
            raise ValueError(
                "ExplorationRunEvent stopping transitions cannot claim confirmed parking"
            )
        if event_kind == ExplorationRunEventKind.STOP_CONFIRMATION_FAILED:
            if self.motion_stop_confirmed or not self.motion_stop_reason.strip():
                raise ValueError(
                    "ExplorationRunEvent STOP_CONFIRMATION_FAILED requires an "
                    "unconfirmed stop and a motion_stop_reason"
                )
            if state not in {
                ExplorationRunState.PAUSING,
                ExplorationRunState.CANCELLING,
            }:
                raise ValueError(
                    "ExplorationRunEvent STOP_CONFIRMATION_FAILED must remain nonterminal"
                )

    @property
    def terminal(self) -> bool:
        return int(self.state) in {
            int(ExplorationRunState.COMPLETED),
            int(ExplorationRunState.CANCELLED),
            int(ExplorationRunState.FAILED),
        }

    @property
    def state_name(self) -> str:
        return _enum_name(ExplorationRunState, self.state)

    @property
    def kind_name(self) -> str:
        return _enum_name(ExplorationRunEventKind, self.kind)

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp_s": float(self.ts),
            "frame_id": self.frame_id,
            "boot_id": self.boot_id,
            "event_sequence": int(self.event_sequence),
            "kind": int(self.kind),
            "kind_name": self.kind_name,
            "exploration_run_id": self.exploration_run_id,
            "start_request_id": self.start_request_id,
            "command_request_id": self.command_request_id,
            "product_session_id": self.product_session_id,
            "state": int(self.state),
            "state_name": self.state_name,
            "terminal": self.terminal,
            "route": self.route,
            "map_id": self.map_id,
            "map_content_epoch": int(self.map_content_epoch),
            "reason": self.reason,
            "motion_stop_confirmed": self.motion_stop_confirmed,
            "motion_stop_reason": self.motion_stop_reason,
        }


@dataclass(frozen=True)
class NavigationState:
    """Compact authoritative lifecycle state published by the native nav endpoint."""

    msg_name: ClassVar[str] = "lingtu.dds.NavigationState"

    ts: float = field(default_factory=time.time)
    frame_id: str = NAV_MAP_FRAME_ID
    boot_id: str = ""
    sequence: int = 0
    control_mode: int = int(NavigationControlMode.UNKNOWN)
    lifecycle_state: int = int(NavigationLifecycle.IDLE)
    active_task_id: str = ""
    active_request_id: str = ""
    goal_epoch: int = 0
    map_id: str = ""
    map_content_epoch: int = 0
    planning_state: int = int(NavigationPlanningState.IDLE)
    execution_state: int = int(NavigationExecutionState.IDLE)
    recovery_state: int = int(NavigationRecoveryState.IDLE)
    progress: float = -1.0
    authority: str = "none"
    hold_reason: str = ""
    failure_code: str = ""

    def __post_init__(self) -> None:
        if not math.isfinite(float(self.ts)) or self.ts <= 0.0:
            raise ValueError("NavigationState.ts must be a positive finite timestamp")
        if not self.frame_id:
            raise ValueError("NavigationState.frame_id is required")
        if not self.boot_id:
            raise ValueError("NavigationState.boot_id is required")
        if int(self.sequence) <= 0:
            raise ValueError("NavigationState.sequence must be positive")
        if int(self.goal_epoch) < 0 or int(self.map_content_epoch) < 0:
            raise ValueError("NavigationState epochs and versions cannot be negative")
        if bool(self.active_task_id.strip()) != bool(self.active_request_id.strip()):
            raise ValueError(
                "NavigationState active_task_id and active_request_id must be present together"
            )
        progress = float(self.progress)
        if not math.isfinite(progress) or (progress != -1.0 and not 0.0 <= progress <= 1.0):
            raise ValueError("NavigationState.progress must be -1 or within [0, 1]")

    def to_dict(self) -> dict[str, Any]:
        return {
            "ts": float(self.ts),
            "frame_id": self.frame_id,
            "boot_id": self.boot_id,
            "sequence": int(self.sequence),
            "control_mode": int(self.control_mode),
            "control_mode_name": _enum_name(NavigationControlMode, self.control_mode),
            "lifecycle_state": int(self.lifecycle_state),
            "lifecycle_state_name": _enum_name(NavigationLifecycle, self.lifecycle_state),
            "active_task_id": self.active_task_id,
            "active_request_id": self.active_request_id,
            "goal_epoch": int(self.goal_epoch),
            "map_id": self.map_id,
            "map_content_epoch": int(self.map_content_epoch),
            "planning_state": int(self.planning_state),
            "planning_state_name": _enum_name(NavigationPlanningState, self.planning_state),
            "execution_state": int(self.execution_state),
            "execution_state_name": _enum_name(NavigationExecutionState, self.execution_state),
            "recovery_state": int(self.recovery_state),
            "recovery_state_name": _enum_name(NavigationRecoveryState, self.recovery_state),
            "progress": float(self.progress),
            "authority": self.authority,
            "hold_reason": self.hold_reason,
            "failure_code": self.failure_code,
        }


def _header_from_stamp(ts: float, frame_id: str) -> Any:
    from .sensor import Header

    return Header.from_stamp(ts, frame_id)


def _zero_covariance36() -> list[float]:
    return [0.0] * 36


# ---------------------------------------------------------------------------
# Odometry
# ---------------------------------------------------------------------------

@dataclass
class Odometry:
    """Odometry message — pose + twist + frame metadata.

    Maps to ROS2 nav_msgs/Odometry and /slam/odometry topic.
    """

    msg_name: ClassVar[str] = "nav_msgs.Odometry"
    pose: Pose = field(default_factory=Pose)
    twist: Twist = field(default_factory=Twist)
    ts: float = 0.0
    frame_id: str = NAV_ODOM_FRAME_ID
    child_frame_id: str = NAV_BODY_FRAME_ID

    def __post_init__(self) -> None:
        if self.ts == 0.0:
            self.ts = time.time()

    # -- position convenience properties ------------------------------------

    @property
    def x(self) -> float:
        return self.pose.x

    @property
    def y(self) -> float:
        return self.pose.y

    @property
    def z(self) -> float:
        return self.pose.z

    @property
    def yaw(self) -> float:
        return self.pose.yaw

    # -- velocity convenience properties ------------------------------------

    @property
    def vx(self) -> float:
        return self.twist.linear.x

    @property
    def vy(self) -> float:
        return self.twist.linear.y

    @property
    def wz(self) -> float:
        return self.twist.angular.z

    # -- serialisation -------------------------------------------------------

    @property
    def header(self) -> Any:
        return _header_from_stamp(self.ts, self.frame_id)

    def to_dict(self) -> dict[str, Any]:
        return {
            "pose": self.pose.to_dict(),
            "twist": self.twist.to_dict(),
            "ts": self.ts,
            "frame_id": self.frame_id,
            "child_frame_id": self.child_frame_id,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> Odometry:
        return cls(
            pose=Pose.from_dict(d.get("pose", {})),
            twist=Twist.from_dict(d.get("twist", {})),
            ts=float(d.get("ts", 0)),
            frame_id=str(d.get("frame_id", NAV_ODOM_FRAME_ID)),
            child_frame_id=str(d.get("child_frame_id", NAV_BODY_FRAME_ID)),
        )

    def to_ros_dict(self) -> dict[str, Any]:
        return {
            "header": self.header.to_dict(),
            "child_frame_id": self.child_frame_id,
            "pose": {
                "pose": self.pose.to_ros_dict(),
                "covariance": _zero_covariance36(),
            },
            "twist": {
                "twist": self.twist.to_ros_dict(),
                "covariance": _zero_covariance36(),
            },
        }

    def encode(self) -> bytes:
        """Binary encode: ts(8) + frame_ids + pose(56) + twist(48)."""
        f1 = self.frame_id.encode("utf-8")
        f2 = self.child_frame_id.encode("utf-8")
        header = struct.pack("<dHH", self.ts, len(f1), len(f2)) + f1 + f2
        return header + self.pose.encode() + self.twist.encode()

    @classmethod
    def decode(cls, data: bytes) -> Odometry:
        ts, l1, l2 = struct.unpack("<dHH", data[:12])
        off = 12
        frame_id = data[off: off + l1].decode("utf-8")
        off += l1
        child_frame_id = data[off: off + l2].decode("utf-8")
        off += l2
        pose = Pose.decode(data[off: off + 56])
        off += 56
        twist = Twist.decode(data[off: off + 48])
        return cls(pose=pose, twist=twist, ts=ts,
                   frame_id=frame_id, child_frame_id=child_frame_id)

    def __repr__(self) -> str:
        return (
            f"Odometry(pos=[{self.x:.3f}, {self.y:.3f}, {self.z:.3f}], "
            f"yaw={self.yaw:.3f}, vx={self.vx:.3f}, wz={self.wz:.3f}, "
            f"frame='{self.frame_id}' -> '{self.child_frame_id}')"
        )


# ---------------------------------------------------------------------------
# Path
# ---------------------------------------------------------------------------

@dataclass
class Path:
    """Path message — sequence of PoseStamped.

    Maps to ROS2 nav_msgs/Path and /nav/global_path, /nav/local_path topics.
    """

    msg_name: ClassVar[str] = "nav_msgs.Path"
    poses: list[PoseStamped] = field(default_factory=list)
    ts: float = 0.0
    frame_id: str = NAV_MAP_FRAME_ID

    def __post_init__(self) -> None:
        if self.ts == 0.0:
            self.ts = time.time()

    # -- list-like interface -------------------------------------------------

    def __len__(self) -> int:
        return len(self.poses)

    def __bool__(self) -> bool:
        return len(self.poses) > 0

    def __getitem__(self, index: int | slice) -> PoseStamped | list[PoseStamped]:
        return self.poses[index]

    def __iter__(self) -> Iterator[PoseStamped]:
        return iter(self.poses)

    # -- queries -------------------------------------------------------------

    def head(self) -> PoseStamped | None:
        """First waypoint, or None if path is empty."""
        return self.poses[0] if self.poses else None

    def last(self) -> PoseStamped | None:
        """Last waypoint, or None if path is empty."""
        return self.poses[-1] if self.poses else None

    def reverse(self) -> Path:
        """Return a new path with poses reversed (immutable)."""
        return Path(poses=list(reversed(self.poses)), ts=self.ts, frame_id=self.frame_id)

    def total_length(self) -> float:
        """Total path length — sum of Euclidean distances between consecutive poses."""
        if len(self.poses) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(self.poses)):
            a, b = self.poses[i - 1], self.poses[i]
            dx = b.x - a.x
            dy = b.y - a.y
            dz = b.z - a.z
            total += math.sqrt(dx * dx + dy * dy + dz * dz)
        return total

    # -- serialisation -------------------------------------------------------

    @property
    def header(self) -> Any:
        return _header_from_stamp(self.ts, self.frame_id)

    def to_dict(self) -> dict[str, Any]:
        return {
            "poses": [p.to_dict() for p in self.poses],
            "ts": self.ts,
            "frame_id": self.frame_id,
        }

    def to_ros_dict(self) -> dict[str, Any]:
        return {
            "header": self.header.to_dict(),
            "poses": [p.to_ros_dict() for p in self.poses],
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> Path:
        return cls(
            poses=[PoseStamped.from_dict(p) for p in d.get("poses", [])],
            ts=float(d.get("ts", 0)),
            frame_id=str(d.get("frame_id", NAV_MAP_FRAME_ID)),
        )

    def encode(self) -> bytes:
        """Binary encode: header + N × pose_stamped."""
        fb = self.frame_id.encode("utf-8")
        parts = [struct.pack("<dIH", self.ts, len(self.poses), len(fb)), fb]
        for ps in self.poses:
            parts.append(ps.encode())
        return b"".join(parts)

    @classmethod
    def decode(cls, data: bytes) -> Path:
        ts, n, flen = struct.unpack("<dIH", data[:14])
        off = 14
        frame_id = data[off: off + flen].decode("utf-8")
        off += flen
        poses: list[PoseStamped] = []
        for _ in range(n):
            ps = PoseStamped.decode(data[off:])
            poses.append(ps)
            # PoseStamped.encode: _META(12) + frame_bytes + Pose(56)
            meta_size = PoseStamped._META.size
            _, inner_flen = PoseStamped._META.unpack(data[off: off + meta_size])
            off += meta_size + inner_flen + Pose._FMT.size
        return cls(poses=poses, ts=ts, frame_id=frame_id)

    def __repr__(self) -> str:
        length = f"{self.total_length():.2f}m" if self.poses else "empty"
        return f"Path(n={len(self.poses)}, length={length}, frame='{self.frame_id}')"


# ---------------------------------------------------------------------------
# OccupancyGrid
# ---------------------------------------------------------------------------

@dataclass
class OccupancyGrid:
    """Occupancy grid map.

    Maps to ROS2 nav_msgs/OccupancyGrid.
    Cell values: FREE=0, OCCUPIED=100, UNKNOWN=-1.
    """

    msg_name: ClassVar[str] = "nav_msgs.OccupancyGrid"
    FREE: int = 0
    OCCUPIED: int = 100
    UNKNOWN: int = -1

    grid: Any = field(default_factory=list)
    resolution: float = 0.05
    origin: Pose = field(default_factory=Pose)
    ts: float = 0.0
    frame_id: str = NAV_MAP_FRAME_ID

    def __post_init__(self) -> None:
        if self.ts == 0.0:
            self.ts = time.time()
        if not is_numpy_array(self.grid):
            if not self.grid and not numpy_import_is_safe():
                return
            self.grid = (
                np.array(self.grid, dtype=np.int8)
                if self.grid
                else np.zeros((0, 0), dtype=np.int8)
            )
        elif self.grid.dtype != np.int8:
            self.grid = self.grid.astype(np.int8)

    # -- dimension properties ------------------------------------------------

    @property
    def height(self) -> int:
        if is_numpy_array(self.grid):
            return int(self.grid.shape[0]) if self.grid.ndim == 2 else 0
        return len(self.grid) if isinstance(self.grid, list) else 0

    @property
    def width(self) -> int:
        if is_numpy_array(self.grid):
            return int(self.grid.shape[1]) if self.grid.ndim == 2 else 0
        if isinstance(self.grid, list) and self.grid:
            first = self.grid[0]
            return len(first) if isinstance(first, list) else 0
        return 0

    # -- coordinate transforms -----------------------------------------------

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """World coordinates → grid indices (row, col)."""
        col = int((x - self.origin.x) / self.resolution)
        row = int((y - self.origin.y) / self.resolution)
        return (row, col)

    def grid_to_world(self, row: int, col: int) -> tuple[float, float]:
        """Grid indices → world coordinates (x, y)."""
        x = self.origin.x + col * self.resolution
        y = self.origin.y + row * self.resolution
        return (x, y)

    def cell_value(self, x: float, y: float) -> int:
        """Cell value at world (x, y); out of bounds returns UNKNOWN (-1)."""
        row, col = self.world_to_grid(x, y)
        if 0 <= row < self.height and 0 <= col < self.width:
            if is_numpy_array(self.grid):
                return int(self.grid[row, col])
            return int(self.grid[row][col])
        return self.UNKNOWN

    # -- serialisation -------------------------------------------------------

    @property
    def header(self) -> Any:
        return _header_from_stamp(self.ts, self.frame_id)

    def to_dict(self) -> dict[str, Any]:
        return {
            "grid": self.grid.tolist() if hasattr(self.grid, "tolist") else self.grid,
            "resolution": self.resolution,
            "origin": self.origin.to_dict(),
            "ts": self.ts,
            "frame_id": self.frame_id,
            "width": self.width,
            "height": self.height,
        }

    def to_ros_dict(self) -> dict[str, Any]:
        header = self.header
        grid_data = (
            self.grid.reshape(-1).tolist()
            if is_numpy_array(self.grid)
            else np.array(self.grid, dtype=np.int8).reshape(-1).tolist()
            if self.grid
            else []
        )
        return {
            "header": header.to_dict(),
            "info": {
                "map_load_time": header.stamp.to_dict(),
                "resolution": float(self.resolution),
                "width": int(self.width),
                "height": int(self.height),
                "origin": self.origin.to_ros_dict(),
            },
            "data": [int(v) for v in grid_data],
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> OccupancyGrid:
        grid_data = d.get("grid", [])
        grid = np.array(grid_data, dtype=np.int8) if grid_data else []
        return cls(
            grid=grid,
            resolution=float(d.get("resolution", 0.05)),
            origin=Pose.from_dict(d.get("origin", {})),
            ts=float(d.get("ts", 0)),
            frame_id=str(d.get("frame_id", NAV_MAP_FRAME_ID)),
        )

    def encode(self) -> bytes:
        """Binary encode: header + origin(56) + grid data."""
        fb = self.frame_id.encode("utf-8")
        header = struct.pack("<dffIIH", self.ts, self.resolution,
                             0.0,  # reserved
                             self.height, self.width, len(fb)) + fb
        origin_bytes = self.origin.encode()
        if is_numpy_array(self.grid):
            grid_bytes = self.grid.tobytes()
        elif not self.grid:
            grid_bytes = b""
        else:
            grid_bytes = np.array(self.grid, dtype=np.int8).tobytes()
        return header + origin_bytes + grid_bytes

    @classmethod
    def decode(cls, data: bytes) -> OccupancyGrid:
        ts, res, _, h, w, flen = struct.unpack("<dffIIH", data[:26])
        off = 26
        frame_id = data[off: off + flen].decode("utf-8")
        off += flen
        origin = Pose.decode(data[off: off + 56])
        off += 56
        nbytes = h * w
        if nbytes == 0:
            return cls(grid=[], resolution=res, origin=origin, ts=ts, frame_id=frame_id)
        grid = np.frombuffer(data[off: off + nbytes], dtype=np.int8).reshape((h, w)).copy()
        return cls(grid=grid, resolution=res, origin=origin, ts=ts, frame_id=frame_id)

    def __repr__(self) -> str:
        return (
            f"OccupancyGrid({self.width}x{self.height}, "
            f"res={self.resolution}m, frame='{self.frame_id}')"
        )

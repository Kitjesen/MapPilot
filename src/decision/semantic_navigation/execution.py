"""Semantic ownership of a goal admitted through native navigation."""

from dataclasses import dataclass

from runtime.msgs.geometry import PoseStamped

from .observation import ObjectTarget
from .verification import TargetVerification


@dataclass
class GoalExecution:
    task_id: str
    request_id: str
    instruction_epoch: int
    instruction: str
    purpose: str
    pose: PoseStamped
    state: str = "dispatching"
    reason: str = ""
    terminal: bool = False
    boot_id: str = ""
    sequence: int = 0
    cancel_request_id: str = ""
    target: ObjectTarget | None = None
    verification: TargetVerification | None = None

    def to_dict(self) -> dict:
        return {
            "task_id": self.task_id,
            "request_id": self.request_id,
            "purpose": self.purpose,
            "state": self.state,
            "reason": self.reason,
            "terminal": self.terminal,
            "cancel_pending": bool(self.cancel_request_id) and not self.terminal,
            "verification": self.verification.to_dict() if self.verification is not None else None,
            "object_target": {
                "id": self.target.object_id,
                "label": self.target.label,
                "position": list(self.target.position),
            } if self.target is not None else None,
        }

"""TaskDecomposerModule -- task decomposition as independent Module.

Wraps TaskDecomposer into lingtu.runtime.Module with In/Out ports.
Uses decompose_with_rules() for offline rule-based decomposition.

Ports:
  In:  instruction (str)
  Out: task_plan (dict)
"""

from __future__ import annotations

import logging
from typing import Any

from runtime.module import Module
from runtime.stream import In, Out

logger = logging.getLogger(__name__)


class TaskDecomposerModule(Module, layer=4):
    """Task decomposition module (Layer 4 Planning)."""

    instruction: In[str]
    task_plan: Out[dict]

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)
        self._decomposer: Any | None = None

    def setup(self) -> None:
        self.instruction.subscribe(self._on_instruction)
        self._ensure_decomposer()

    def _ensure_decomposer(self) -> None:
        if self._decomposer is not None:
            return
        try:
            from decision.tasking.task_decomposer import TaskDecomposer
            self._decomposer = TaskDecomposer()
        except Exception as e:
            logger.warning("TaskDecomposer init failed: %s", e)
            self._decomposer = None

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["last_decompose_time"] = getattr(self, "_last_decompose_time", None)
        info["subtask_count"] = getattr(self, "_subtask_count", 0)
        return info

    def _on_instruction(self, instruction: str) -> None:
        if not instruction or not instruction.strip():
            return
        plan = self._decompose(instruction.strip())
        if plan is not None:
            self.task_plan.publish(plan)

    def _decompose(self, instruction: str) -> dict | None:
        if self._decomposer is None:
            return self._offline_decompose(instruction)
        try:
            plan = self._decomposer.decompose_with_rules(instruction)
            if plan is not None:
                return plan.to_dict()
        except Exception as e:
            logger.warning("Rule decomposition error: %s", e)
        return self._offline_decompose(instruction)

    @staticmethod
    def _offline_decompose(instruction: str) -> dict:
        return {
            "instruction": instruction,
            "total_steps": 2,
            "current_step": 0,
            "is_complete": False,
            "subgoals": [
                {"step_id": 0, "action": "navigate", "target": instruction,
                 "status": "pending", "retry_count": 0},
                {"step_id": 1, "action": "verify", "target": instruction,
                 "status": "pending", "retry_count": 0},
            ],
        }

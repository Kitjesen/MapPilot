"""Task decomposition into semantic subgoals."""

import json
import logging
import re
from dataclasses import dataclass, field
from enum import Enum

logger = logging.getLogger(__name__)

# TaskRulesMixin is imported after local enums/dataclasses so rule helpers can
# use the canonical SubGoalAction symbols defined in this module.
from .rules import TaskRulesMixin


class SubGoalAction(Enum):
    """Sub Goal Action."""

    NAVIGATE = "navigate"
    FIND = "find"
    APPROACH = "approach"
    VERIFY = "verify"
    LOOK_AROUND = "look_around"
    EXPLORE = "explore"
    BACKTRACK = "backtrack"
    WAIT = "wait"
    FOLLOW = "follow"
    STOP = "stop"
    PICK = "pick"
    PLACE = "place"
    STATUS = "status"
    PATROL = "patrol"
    SAVE_MAP = "save_map"
    SAVE_POI = "save_poi"
    SET_SPEED = "set_speed"
    SET_GEOFENCE = "set_geofence"
    RETURN_HOME = "return_home"
    PAUSE = "pause"
    RESUME = "resume"


class SubGoalStatus(Enum):
    """Sub Goal Status."""

    PENDING = "pending"
    ACTIVE = "active"
    COMPLETED = "completed"
    FAILED = "failed"
    SKIPPED = "skipped"


@dataclass
class SubGoal:
    """Sub Goal."""

    step_id: int
    action: SubGoalAction
    target: str
    parameters: dict = field(default_factory=dict)
    status: SubGoalStatus = SubGoalStatus.PENDING
    result: dict | None = None
    retry_count: int = 0
    max_retries: int = 2

    def to_dict(self) -> dict:
        return {
            "step_id": self.step_id,
            "action": self.action.value,
            "target": self.target,
            "status": self.status.value,
            "retry_count": self.retry_count,
        }


@dataclass
class TaskPlan:
    """Task Plan."""

    instruction: str
    subgoals: list[SubGoal] = field(default_factory=list)
    current_step: int = 0

    @property
    def is_complete(self) -> bool:
        return bool(self.subgoals) and all(
            sg.status in (SubGoalStatus.COMPLETED, SubGoalStatus.SKIPPED) for sg in self.subgoals
        )

    @property
    def is_failed(self) -> bool:
        return any(sg.status == SubGoalStatus.FAILED and sg.retry_count >= sg.max_retries for sg in self.subgoals)

    @property
    def active_subgoal(self) -> SubGoal | None:
        for sg in self.subgoals:
            if sg.status in (SubGoalStatus.PENDING, SubGoalStatus.ACTIVE):
                return sg
        return None

    def advance(self):
        """Advance."""
        active = self.active_subgoal
        if active:
            active.status = SubGoalStatus.COMPLETED
            self.current_step += 1

    def fail_current(self):
        """Fail current."""
        active = self.active_subgoal
        if active:
            active.retry_count += 1
            if active.retry_count >= active.max_retries:
                active.status = SubGoalStatus.FAILED
            else:
                active.status = SubGoalStatus.PENDING

    def to_dict(self) -> dict:
        return {
            "instruction": self.instruction,
            "total_steps": len(self.subgoals),
            "current_step": self.current_step,
            "is_complete": self.is_complete,
            "subgoals": [sg.to_dict() for sg in self.subgoals],
        }


class TaskDecomposer(TaskRulesMixin):
    """Task Decomposer."""

    _knowledge_graph = None  # type: Any | None

    @classmethod
    def set_knowledge_graph(cls, kg) -> None:
        """Set knowledge graph."""
        cls._knowledge_graph = kg

    def build_decomposition_prompt(
        self,
        instruction: str,
        scene_summary: str = "",
        language: str = "zh",
    ) -> list[dict[str, str]]:
        """Build decomposition prompt."""
        available_actions = ", ".join(a.value for a in SubGoalAction)

        system = f"""You are a robot task planner. Decompose the instruction into subgoals.

Available actions: {available_actions}

Rules:
1. Each subgoal must be atomic (single action)
2. navigate: go to a region or position
3. find: search for a matching object in the scene graph
4. approach: move to within 0.5m of the target
5. verify: close-range target identity confirmation
6. look_around: 360-degree scan in place
7. explore: go to an unexplored area
8. backtrack: return to a previous position
9. wait: wait for a condition

Output format (strict JSON):
{{
  "subgoals": [
    {{"action": "navigate", "target": "...", "parameters": {{}}}},
    ...
  ]
}}"""
        user_content = f"## Instruction\n{instruction}"
        if scene_summary:
            user_content += f"\n\n## Current Scene\n{scene_summary}"

        return [
            {"role": "system", "content": system},
            {"role": "user", "content": user_content},
        ]

    def parse_decomposition_response(self, instruction: str, response_text) -> TaskPlan:
        """Parse decomposition response."""

        if isinstance(response_text, dict):
            data = response_text
            subgoals = []
            for i, sg_data in enumerate(data.get("subgoals", [])):
                action_str = sg_data.get("action", "navigate")
                try:
                    action = SubGoalAction(action_str)
                except ValueError:
                    action = SubGoalAction.NAVIGATE
                subgoals.append(
                    SubGoal(
                        step_id=i,
                        action=action,
                        target=sg_data.get("target", instruction),
                        parameters=sg_data.get("parameters", {}),
                    )
                )
            if not subgoals:
                subgoals = [
                    SubGoal(step_id=0, action=SubGoalAction.NAVIGATE, target=instruction),
                    SubGoal(step_id=1, action=SubGoalAction.VERIFY, target=instruction),
                ]
            return TaskPlan(instruction=instruction, subgoals=subgoals)

        match = re.search(r"```(?:json)?\s*([\s\S]*?)```", response_text)
        if match:
            raw = match.group(1).strip()
        else:
            start = response_text.find("{")
            end = response_text.rfind("}")
            if start == -1 or end == -1:
                logger.error("No JSON in decomposition response: %s", response_text[:200])

                return TaskPlan(
                    instruction=instruction,
                    subgoals=[
                        SubGoal(step_id=0, action=SubGoalAction.NAVIGATE, target=instruction),
                        SubGoal(step_id=1, action=SubGoalAction.VERIFY, target=instruction),
                    ],
                )
            raw = response_text[start : end + 1]

        try:
            data = json.loads(raw)
        except json.JSONDecodeError as e:
            logger.error("JSON parse error in decomposition: %s", e)
            return TaskPlan(
                instruction=instruction,
                subgoals=[
                    SubGoal(step_id=0, action=SubGoalAction.NAVIGATE, target=instruction),
                    SubGoal(step_id=1, action=SubGoalAction.VERIFY, target=instruction),
                ],
            )

        subgoals = []
        for i, sg_data in enumerate(data.get("subgoals", [])):
            action_str = sg_data.get("action", "navigate")
            try:
                action = SubGoalAction(action_str)
            except ValueError:
                action = SubGoalAction.NAVIGATE

            subgoals.append(
                SubGoal(
                    step_id=i,
                    action=action,
                    target=sg_data.get("target", instruction),
                    parameters=sg_data.get("parameters", {}),
                )
            )

        if not subgoals:
            subgoals = [
                SubGoal(step_id=0, action=SubGoalAction.NAVIGATE, target=instruction),
                SubGoal(step_id=1, action=SubGoalAction.VERIFY, target=instruction),
            ]

        return TaskPlan(instruction=instruction, subgoals=subgoals)

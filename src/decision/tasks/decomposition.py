"""Task decomposition into semantic subgoals."""

import json
import logging
import re
from dataclasses import dataclass, field
from enum import Enum

logger = logging.getLogger(__name__)

# Decomposition emits symbolic targets. Geometry is resolved by the scene
# query and native planner layers, never by this parser.
_GEOMETRIC_PARAMETER_KEYS = frozenset(
    {
        "x", "y", "z", "roll", "pitch", "yaw", "position", "pose",
        "coordinates", "world_pose", "map_pose", "frame_id",
    }
)

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

    def validate(self) -> tuple[str, ...]:
        """Return schema violations without resolving or executing the goal."""
        errors: list[str] = []
        if not isinstance(self.step_id, int) or self.step_id < 0:
            errors.append("step_id_must_be_non_negative_integer")
        if not isinstance(self.action, SubGoalAction):
            errors.append("action_must_be_subgoal_action")
        if not isinstance(self.target, str) or not self.target.strip():
            errors.append("target_must_be_non_empty_string")
        if not isinstance(self.parameters, dict):
            errors.append("parameters_must_be_object")
        else:
            geometric = sorted(set(self.parameters) & _GEOMETRIC_PARAMETER_KEYS)
            if geometric:
                errors.append(f"geometric_parameters_not_allowed:{','.join(geometric)}")
        return tuple(errors)

    def to_dict(self) -> dict:
        return {
            "step_id": self.step_id,
            "action": self.action.value,
            "target": self.target,
            "parameters": dict(self.parameters),
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

    def validate(self) -> tuple[str, ...]:
        """Validate the symbolic plan contract without consulting a scene."""
        errors: list[str] = []
        if not isinstance(self.instruction, str) or not self.instruction.strip():
            errors.append("instruction_must_be_non_empty_string")
        if not isinstance(self.subgoals, list):
            return tuple([*errors, "subgoals_must_be_list"])
        if [goal.step_id for goal in self.subgoals] != list(range(len(self.subgoals))):
            errors.append("step_ids_must_be_zero_based_sequential")
        for goal in self.subgoals:
            errors.extend(f"subgoal_{goal.step_id}:{error}" for error in goal.validate())
        return tuple(errors)

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
            "schema_version": "task_plan.v1",
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

    def decompose(self, instruction: str) -> TaskPlan | None:
        """Parse the task structure without resolving semantic targets.

        Rule parsing is deliberately bounded to commands that can be classified
        without a scene graph.  A ``None`` result means that a caller should
        obtain an LLM decomposition response and pass it to
        :meth:`parse_decomposition_response`.
        """
        text = str(instruction or "").strip()
        if not text:
            return None
        return self.decompose_with_rules(text)

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
2. navigate: name a semantic destination or region; never emit coordinates or poses
3. find: name a matching object or relation symbolically; do not query the scene graph yet
4. approach: move to within 0.5m of the target
5. verify: close-range target identity confirmation
6. look_around: 360-degree scan in place
7. explore: go to an unexplored area
8. backtrack: return to a previous position
9. wait: wait for a condition
10. This stage only creates a symbolic task plan. Do not resolve room/object IDs,
    map frames, positions, paths, or motion commands.
11. follow: continuously follow a described person until cancellation or timeout.
    Keep identifying clothing and appearance attributes in the target description.
    The follow skill owns person selection, identity confirmation, continuous tracking,
    and rediscovery after losing the person. For a pure follow request, emit exactly one
    follow subgoal; do not add find, verify, or approach around it. Preserve find,
    verify, or approach only when the user explicitly requests those as independent steps.
12. stop: cancel the ongoing motion task.

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

        def fallback() -> TaskPlan:
            return TaskPlan(
                instruction=instruction,
                subgoals=[
                    SubGoal(
                        step_id=0,
                        action=SubGoalAction.STATUS,
                        target="task_parse_failed",
                        parameters={"instruction": instruction},
                    ),
                ],
            )

        def from_data(data) -> TaskPlan:
            if not isinstance(data, dict) or not isinstance(data.get("subgoals", []), list):
                return fallback()
            subgoals = []
            for raw_goal in data.get("subgoals", []):
                if not isinstance(raw_goal, dict):
                    logger.warning("Ignoring non-object subgoal")
                    continue
                try:
                    action = SubGoalAction(raw_goal.get("action", SubGoalAction.NAVIGATE.value))
                except (TypeError, ValueError):
                    action = SubGoalAction.NAVIGATE
                target = raw_goal.get("target", instruction)
                if not isinstance(target, str):
                    target = str(target)
                raw_parameters = raw_goal.get("parameters", {})
                if not isinstance(raw_parameters, dict):
                    raw_parameters = {}
                parameters = {
                    key: value for key, value in raw_parameters.items() if key not in _GEOMETRIC_PARAMETER_KEYS
                }
                removed = set(raw_parameters) - set(parameters)
                if removed:
                    logger.warning("Ignoring geometric task parameters: %s", sorted(removed))
                subgoals.append(
                    SubGoal(
                        step_id=len(subgoals),
                        action=action,
                        target=target,
                        parameters=parameters,
                    )
                )
            return TaskPlan(instruction=instruction, subgoals=subgoals) if subgoals else fallback()

        if isinstance(response_text, dict):
            return from_data(response_text)

        match = re.search(r"```(?:json)?\s*([\s\S]*?)```", response_text)
        if match:
            raw = match.group(1).strip()
        else:
            start = response_text.find("{")
            end = response_text.rfind("}")
            if start == -1 or end == -1:
                logger.error("No JSON in decomposition response: %s", response_text[:200])

                return fallback()
            raw = response_text[start : end + 1]

        try:
            data = json.loads(raw)
        except json.JSONDecodeError as e:
            logger.error("JSON parse error in decomposition: %s", e)
            return fallback()

        return from_data(data)

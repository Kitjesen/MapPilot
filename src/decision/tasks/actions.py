"""Action command shaping for semantic subgoals.

ActionExecutor maps decomposed subgoals to command payloads used by the
SemanticPlannerModule. It does not drive hardware directly; motion still flows
through Module outputs, Navigation, and velocity arbitration.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any

from runtime.msgs.numpy_compat import np
from runtime.runtime_interface import map_frame_id
from runtime.utils.sanitize import safe_json_loads

logger = logging.getLogger(__name__)
ACTION_COMMAND_MAP_FRAME_ID = map_frame_id()


class ActionStatus(Enum):
    """Action Status."""

    IDLE = "idle"
    EXECUTING = "executing"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    TIMEOUT = "timeout"


@dataclass
class ActionCommand:
    """Action Command."""

    command_type: str  # "goal" | "velocity" | "cancel"

    target_x: float = 0.0
    target_y: float = 0.0
    target_z: float = 0.0
    target_yaw: float = 0.0

    linear_x: float = 0.0
    angular_z: float = 0.0

    timeout_sec: float = 30.0
    approach_speed: float = 0.3
    frame_id: str = ACTION_COMMAND_MAP_FRAME_ID


class ActionExecutor:
    """Action Executor."""

    def __init__(
        self,
        approach_distance: float = 0.5,
        verify_distance: float = 0.8,
        look_around_speed: float = 0.5,
        look_around_duration: float = 12.0,
        nav_timeout: float = 60.0,
    ) -> None:
        self.approach_distance = approach_distance
        self.verify_distance = verify_distance
        self.look_around_speed = look_around_speed
        self.look_around_duration = look_around_duration
        self.nav_timeout = nav_timeout

        self._status = ActionStatus.IDLE
        self._start_time = 0.0
        self._lock = threading.Lock()

    def _set_executing(self) -> None:
        """Set executing."""
        with self._lock:
            self._status = ActionStatus.EXECUTING
            self._start_time = time.time()

    @property
    def status(self) -> ActionStatus:
        return self._status

    def generate_navigate_command(
        self,
        target_position: dict[str, float],
        robot_position: dict[str, float] | None = None,
    ) -> ActionCommand:
        """Generate navigate command."""

        yaw = 0.0
        if robot_position:
            dx = target_position["x"] - robot_position["x"]
            dy = target_position["y"] - robot_position["y"]
            yaw = math.atan2(dy, dx)

        self._set_executing()

        return ActionCommand(
            command_type="goal",
            target_x=target_position["x"],
            target_y=target_position["y"],
            target_z=target_position.get("z", 0.0),
            target_yaw=yaw,
            timeout_sec=self.nav_timeout,
        )

    def generate_approach_command(
        self,
        target_position: dict[str, float],
        robot_position: dict[str, float],
    ) -> ActionCommand:
        """Generate approach command."""
        dx = target_position["x"] - robot_position["x"]
        dy = target_position["y"] - robot_position["y"]
        dist = math.sqrt(dx * dx + dy * dy)
        yaw = math.atan2(dy, dx)

        if dist > self.approach_distance:
            scale = (dist - self.approach_distance) / dist
            goal_x = robot_position["x"] + dx * scale
            goal_y = robot_position["y"] + dy * scale
        else:
            goal_x = robot_position["x"]
            goal_y = robot_position["y"]

        self._set_executing()

        return ActionCommand(
            command_type="goal",
            target_x=goal_x,
            target_y=goal_y,
            target_z=target_position.get("z", 0.0),
            target_yaw=yaw,
            approach_speed=0.15,
            timeout_sec=20.0,
        )

    def generate_look_around_command(self) -> ActionCommand:
        """Generate look around command."""
        self._set_executing()

        return ActionCommand(
            command_type="velocity",
            angular_z=self.look_around_speed,
            timeout_sec=self.look_around_duration,
        )

    def generate_verify_command(
        self,
        target_position: dict[str, float],
        robot_position: dict[str, float],
    ) -> ActionCommand:
        """Generate verify command."""
        dx = target_position["x"] - robot_position["x"]
        dy = target_position["y"] - robot_position["y"]
        yaw = math.atan2(dy, dx)

        self._set_executing()

        return ActionCommand(
            command_type="goal",
            target_x=robot_position["x"],
            target_y=robot_position["y"],
            target_yaw=yaw,
            timeout_sec=10.0,
        )

    def generate_backtrack_command(
        self,
        backtrack_position: np.ndarray,
    ) -> ActionCommand:
        """Generate backtrack command."""
        self._set_executing()

        return ActionCommand(
            command_type="goal",
            target_x=float(backtrack_position[0]),
            target_y=float(backtrack_position[1]),
            target_z=float(backtrack_position[2]) if len(backtrack_position) > 2 else 0.0,
            timeout_sec=self.nav_timeout,
        )

    def check_timeout(self) -> bool:
        """Check timeout."""
        with self._lock:
            if self._status != ActionStatus.EXECUTING:
                return False
            return (time.time() - self._start_time) > self.nav_timeout

    def mark_succeeded(self) -> None:
        with self._lock:
            self._status = ActionStatus.SUCCEEDED

    def mark_failed(self) -> None:
        with self._lock:
            self._status = ActionStatus.FAILED

    def reset(self) -> None:
        with self._lock:
            self._status = ActionStatus.IDLE
            self._start_time = 0.0

    def lera_recover(
        self,
        failed_action: str,
        current_labels: list[str],
        original_goal: str,
        failure_count: int = 1,
        llm_client: Any | None = None,
        event_loop: Any | None = None,
    ) -> str:
        """Lera recover."""

        label_str = ", ".join(current_labels[:8]) if current_labels else "no visible objects"
        scene_desc = f"Visible nearby objects: {label_str}"

        prompt = (
            "A robot navigation action failed. Analyze the likely cause and choose a recovery.\n\n"
            f"Failed action: {failed_action}\n"
            f"{scene_desc}\n"
            f"Original goal: {original_goal}\n"
            f"Failure count: {failure_count}\n\n"
            "Output strict JSON only, with no extra text:\n"
            '{"reason": "one sentence failure cause", '
            '"action": "retry_different_path|expand_search|requery_goal|abort", '
            '"params": {}}\n\n'
            "Selection rules:\n"
            "- retry_different_path: path is blocked but the target is likely correct\n"
            "- expand_search: target was not found at the expected place\n"
            "- requery_goal: original goal may be ambiguous or incorrectly grounded\n"
            "- abort: repeated failures and no reasonable recovery remains"
        )

        valid_actions = ("retry_different_path", "expand_search", "requery_goal", "abort")

        if llm_client is not None:
            try:
                import asyncio
                import re

                messages = [{"role": "user", "content": prompt}]

                loop = event_loop
                if loop is not None and loop.is_running():
                    future = asyncio.run_coroutine_threadsafe(llm_client.chat(messages, temperature=0.3), loop)
                    response = future.result(timeout=15.0)
                else:
                    tmp_loop = asyncio.new_event_loop()
                    try:
                        response = tmp_loop.run_until_complete(llm_client.chat(messages, temperature=0.3))
                    finally:
                        tmp_loop.close()

                if not response:
                    logger.warning("[LERa] LLM returned empty response")
                else:
                    match = re.search(r"\{[^{}]*(?:\{[^{}]*\}[^{}]*)*\}", response, re.DOTALL)
                    if match:
                        data = safe_json_loads(match.group(), default={})
                        action = data.get("action", "")
                        if action in valid_actions:
                            logger.info(
                                "[LERa] LLM recovery: reason=%s action=%s",
                                data.get("reason", "."),
                                action,
                            )
                            return action
            except Exception as e:
                logger.warning("[LERa] LLM call failed, using fallback: %s", e)

        if failure_count >= 3:
            return "abort"
        elif failure_count >= 2:
            return "expand_search"
        return "retry_different_path"

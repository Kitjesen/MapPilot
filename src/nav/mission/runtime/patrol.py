"""Patrol runtime helpers for Navigation."""

from __future__ import annotations

import logging

from nav.mission.model.state import MissionEvent
from runtime.msgs.numpy_compat import np

logger = logging.getLogger(__name__)


class NavigationPatrolMixin:
    def _start_patrol_route(
        self,
        goals: list[np.ndarray],
        *,
        loop: bool,
    ) -> None:
        if self._same_patrol_goals(goals, loop=loop):
            return
        self._patrol_goals = goals
        self._patrol_loop = loop
        self._patrol_index = 0
        if self._external_strategy_path_control:
            self._start_external_strategy_path(goals)
            return
        self._start_goal(
            self._patrol_goals[0],
            frame_id=self._planning_frame_id,
            event=MissionEvent.PATROL_ACCEPTED,
            reason="patrol accepted",
        )
        logger.info("Patrol started: %d goals, loop=%s", len(self._patrol_goals), self._patrol_loop)

    def _on_patrol_goals(self, goals: list) -> None:
        if not goals:
            return
        parsed_goals: list[np.ndarray] = []
        patrol_loop = False
        for item in goals:
            goal = self._parse_patrol_goal(item)
            if goal is None:
                self._reject_invalid_goal(source="patrol_goals")
                return
            parsed_goals.append(goal)
            if isinstance(item, dict) and item.get("loop"):
                patrol_loop = True
        self._start_patrol_route(parsed_goals, loop=patrol_loop)

    def _parse_patrol_goal(self, item) -> np.ndarray | None:
        if isinstance(item, dict):
            frame_id = str(item.get("frame_id") or self._planning_frame_id)
            if frame_id != self._planning_frame_id:
                self._reject_goal_frame(frame_id, source="patrol_goals")
                return None
            raw = [item.get("x"), item.get("y"), item.get("z", self._resolve_goal_z(None))]
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            raw = [item[0], item[1], item[2] if len(item) > 2 else self._resolve_goal_z(None)]
        else:
            return None
        try:
            goal = np.array(raw, dtype=float)
        except (TypeError, ValueError):
            return None
        if goal.shape != (3,) or not np.all(np.isfinite(goal)):
            return None
        return goal

    def _same_patrol_goals(self, goals: list[np.ndarray], *, loop: bool) -> bool:
        if not self._is_planning_or_path_state():
            return False
        if loop != self._patrol_loop or len(goals) != len(self._patrol_goals):
            return False
        if not goals:
            return False
        return all(
            np.linalg.norm(
                np.asarray(a[:3], dtype=float) - np.asarray(b[:3], dtype=float)
            ) <= 0.05
            for a, b in zip(goals, self._patrol_goals)
        )

    def _advance_patrol(self) -> bool:
        self._patrol_index += 1
        if self._patrol_index >= len(self._patrol_goals):
            if self._patrol_loop:
                self._patrol_index = 0
            else:
                return False
        self._goal = self._patrol_goals[self._patrol_index]
        self._set_replan_count(0)
        logger.info(
            "Patrol advancing to goal %d/%d",
            self._patrol_index + 1,
            len(self._patrol_goals),
        )
        self._plan()
        return True

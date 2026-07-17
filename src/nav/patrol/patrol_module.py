"""Standalone patrol route execution module.

Extracted from the former NavigationPatrolMixin.  This module owns:
  1. Parsing and validating incoming patrol goal lists
  2. Sequential waypoint dispatch to Navigation via goal_pose Out port
  3. Patrol lifecycle tracking (index, loop, completion)

Communication with Navigation is exclusively through ports:
  - Receives patrol_goals (list) from external callers (skills, gateway, REPL)
  - Receives nav_status (dict) from Navigation to detect waypoint completion
  - Publishes goal_pose (PoseStamped) to drive Navigation to each waypoint
  - Publishes patrol_status (dict) for monitoring / health surfaces
"""

from __future__ import annotations

import logging
import time
from enum import Enum
from typing import Any

from nav.model.state import MissionState
from nav.services.goals import build_goal_pose
from runtime.module import Module
from runtime.msgs.geometry import PoseStamped
from runtime.msgs.numpy_compat import np
from runtime.registry import register
from runtime.stream import In, Out

logger = logging.getLogger(__name__)

# Default planning frame when nav_status has not yet arrived.
_DEFAULT_PLANNING_FRAME_ID = "map"


class PatrolState(str, Enum):
    """Internal patrol lifecycle states."""

    IDLE = "IDLE"
    PATROLLING = "PATROLLING"
    COMPLETE = "COMPLETE"


@register("patrol", "default", description="Patrol route execution module")
class PatrolModule(Module, layer=5):
    """Standalone patrol route execution module.

    Receives a list of waypoints, dispatches them one-by-one to Navigation
    through the goal_pose port, and advances on waypoint completion detected
    from the nav_status port.
    """

    runtime_id = "nav.patrol"

    # -- Inputs --
    patrol_goals: In[list]
    nav_status: In[dict]

    # -- Outputs --
    goal_pose: Out[PoseStamped]
    patrol_status: Out[dict]

    def __init__(
        self,
        planning_frame_id: str = _DEFAULT_PLANNING_FRAME_ID,
        default_loop: bool = False,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._planning_frame_id = planning_frame_id
        self._default_loop = default_loop

        # Patrol FSM state
        self._state: PatrolState = PatrolState.IDLE
        self._goals: list[np.ndarray] = []
        self._index: int = 0
        self._loop: bool = False
        self._loop_count: int = 0  # how many full cycles completed

        # Cached nav_status for completion detection
        self._last_nav_state: str = ""
        self._waiting_for_nav: bool = False  # True after sending a goal

    def setup(self) -> None:
        self.patrol_goals.subscribe(self._on_patrol_goals)
        self.nav_status.subscribe(self._on_nav_status)
        self._publish_status()

    # ------------------------------------------------------------------ #
    # Input handlers
    # ------------------------------------------------------------------ #

    def _on_patrol_goals(self, goals: list) -> None:
        """Parse incoming patrol goals and start the route."""
        if not goals:
            return

        parsed_goals: list[np.ndarray] = []
        patrol_loop = self._default_loop

        for item in goals:
            goal = self._parse_patrol_goal(item)
            if goal is None:
                logger.warning("Patrol rejected invalid goal: %s", item)
                self._publish_status(error="invalid_goal")
                return
            parsed_goals.append(goal)
            if isinstance(item, dict) and item.get("loop"):
                patrol_loop = True

        # Skip if same route is already active
        if self._same_patrol_goals(parsed_goals, loop=patrol_loop):
            logger.debug("Patrol route unchanged, ignoring duplicate")
            return

        self._start_route(parsed_goals, loop=patrol_loop)

    def _on_nav_status(self, status: dict) -> None:
        """Watch nav_status for waypoint completion to advance patrol."""
        nav_state = str(status.get("state", ""))
        mission_mode = str(status.get("mission_mode", ""))

        # Update planning frame from Navigation if available
        frame = status.get("planning_frame_id")
        if frame:
            self._planning_frame_id = frame

        if self._state != PatrolState.PATROLLING:
            self._last_nav_state = nav_state
            return

        # Detect that Navigation finished the current waypoint
        if not self._waiting_for_nav:
            self._last_nav_state = nav_state
            return

        # SUCCESS or IDLE after we dispatched a goal means waypoint reached
        reached = nav_state == MissionState.SUCCESS.value or (
            nav_state == MissionState.IDLE.value and self._last_nav_state == MissionState.SUCCESS.value
        )
        # Also handle FAILED: skip the current waypoint and try next
        failed = nav_state == MissionState.FAILED.value

        if reached or failed:
            if failed:
                logger.warning(
                    "Patrol waypoint %d/%d failed, skipping",
                    self._index + 1,
                    len(self._goals),
                )
            self._waiting_for_nav = False
            if not self._advance():
                # Patrol finished (no loop or no more waypoints)
                self._state = PatrolState.COMPLETE
                logger.info(
                    "Patrol complete: %d waypoints, %d loops",
                    len(self._goals),
                    self._loop_count,
                )
            self._publish_status()

        self._last_nav_state = nav_state

    # ------------------------------------------------------------------ #
    # Patrol lifecycle
    # ------------------------------------------------------------------ #

    def _start_route(self, goals: list[np.ndarray], *, loop: bool) -> None:
        """Initialize patrol state and dispatch the first waypoint."""
        self._goals = goals
        self._loop = loop
        self._index = 0
        self._loop_count = 0
        self._state = PatrolState.PATROLLING
        self._dispatch_current()
        logger.info(
            "Patrol started: %d goals, loop=%s",
            len(self._goals),
            self._loop,
        )
        self._publish_status()

    def _advance(self) -> bool:
        """Advance to the next patrol waypoint.

        Returns True if a new waypoint was dispatched, False if patrol done.
        """
        self._index += 1
        if self._index >= len(self._goals):
            if self._loop:
                self._index = 0
                self._loop_count += 1
            else:
                return False

        self._dispatch_current()
        logger.info(
            "Patrol advancing to goal %d/%d (loop %d)",
            self._index + 1,
            len(self._goals),
            self._loop_count,
        )
        return True

    def _dispatch_current(self) -> None:
        """Publish the current patrol waypoint as a goal_pose."""
        goal = self._goals[self._index]
        pose = build_goal_pose(
            x=float(goal[0]),
            y=float(goal[1]),
            z=float(goal[2]),
            yaw=0.0,
            planning_frame_id=self._planning_frame_id,
        )
        self.goal_pose.publish(pose)
        self._waiting_for_nav = True

    # ------------------------------------------------------------------ #
    # Goal parsing
    # ------------------------------------------------------------------ #

    def _parse_patrol_goal(self, item: Any) -> np.ndarray | None:
        """Parse a single patrol goal into a 3-element numpy array."""
        if isinstance(item, dict):
            frame_id = str(item.get("frame_id") or self._planning_frame_id)
            if frame_id != self._planning_frame_id:
                logger.warning(
                    "Patrol goal frame %r != planning frame %r, rejecting",
                    frame_id,
                    self._planning_frame_id,
                )
                return None
            raw = [
                item.get("x"),
                item.get("y"),
                item.get("z", 0.0),
            ]
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            raw = [item[0], item[1], item[2] if len(item) > 2 else 0.0]
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
        """Check if new goals are identical to the current active route."""
        if self._state != PatrolState.PATROLLING:
            return False
        if loop != self._loop or len(goals) != len(self._goals):
            return False
        if not goals:
            return False
        return all(
            np.linalg.norm(np.asarray(a[:3], dtype=float) - np.asarray(b[:3], dtype=float)) <= 0.05
            for a, b in zip(goals, self._goals)
        )

    # ------------------------------------------------------------------ #
    # Status publishing
    # ------------------------------------------------------------------ #

    def _publish_status(self, *, error: str = "") -> None:
        """Publish current patrol progress."""
        self.patrol_status.publish(
            {
                "state": self._state.value,
                "patrol_index": self._index,
                "patrol_total": len(self._goals),
                "loop": self._loop,
                "loop_count": self._loop_count,
                "planning_frame_id": self._planning_frame_id,
                "current_goal": (
                    self._goals[self._index].tolist() if self._goals and self._index < len(self._goals) else None
                ),
                "error": error,
                "ts": time.time(),
            }
        )

    # ------------------------------------------------------------------ #
    # Health / diagnostics
    # ------------------------------------------------------------------ #

    def health(self) -> dict[str, Any]:
        return {
            "patrol_state": self._state.value,
            "patrol_index": self._index,
            "patrol_total": len(self._goals),
            "loop": self._loop,
            "loop_count": self._loop_count,
            "waiting_for_nav": self._waiting_for_nav,
        }

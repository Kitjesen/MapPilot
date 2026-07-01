"""ActionExecutorModule -- action execution as independent Module.

Wraps ActionExecutor into lingtu.runtime.Module with In/Out ports.
Converts GoalResult into PoseStamped (goal_pose) or Twist (cmd_vel).

Ports:
  In:  resolved_goal (GoalResult), odometry (Odometry), scene_graph (SceneGraph)
  Out: goal_pose (PoseStamped), cmd_vel (Twist), execution_status (str JSON)
"""

from __future__ import annotations

import json
import logging
import math
import threading
from typing import Any

from runtime.module import Module
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.semantic import GoalResult as MsgGoalResult
from runtime.msgs.semantic import SceneGraph
from runtime.runtime_interface import map_frame_id
from runtime.stream import In, Out

logger = logging.getLogger(__name__)
ACTION_EXECUTOR_MODULE_MAP_FRAME_ID = map_frame_id()


class ActionExecutorModule(Module, layer=4):
    """Action execution module (Layer 4 Planning).

    Config kwargs:
        approach_distance: float (default 0.5m)
        nav_timeout: float (default 60.0s)
        max_lera_retries: int (default 3)
    """

    resolved_goal: In[MsgGoalResult]
    odometry: In[Odometry]
    scene_graph: In[SceneGraph]

    goal_pose: Out[PoseStamped]
    cmd_vel: Out[Twist]
    execution_status: Out[str]

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)
        self._approach_distance: float = config.get("approach_distance", 0.5)
        self._nav_timeout: float = config.get("nav_timeout", 60.0)
        self._max_lera_retries: int = config.get("max_lera_retries", 3)
        self._executor: Any | None = None
        self._latest_odom: Odometry | None = None
        self._latest_sg: SceneGraph | None = None
        self._failure_count: int = 0
        self._lock = threading.Lock()

    def setup(self) -> None:
        self.resolved_goal.subscribe(self._on_resolved_goal)
        self.odometry.subscribe(self._on_odometry)
        self.scene_graph.subscribe(self._on_scene_graph)
        self._ensure_executor()

    def _ensure_executor(self) -> None:
        if self._executor is not None:
            return
        try:
            from decision.tasking.action_executor import ActionExecutor
            self._executor = ActionExecutor(
                approach_distance=self._approach_distance,
                nav_timeout=self._nav_timeout,
            )
        except Exception as e:
            logger.warning("ActionExecutor init failed: %s", e)
            self._executor = None

    def _on_odometry(self, odom: Odometry) -> None:
        with self._lock:
            self._latest_odom = odom

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        with self._lock:
            self._latest_sg = sg

    def _on_resolved_goal(self, goal: MsgGoalResult) -> None:
        if not goal.is_valid:
            self.execution_status.publish(json.dumps({
                "state": "rejected", "reason": "invalid_goal",
                "target_label": goal.target_label}))
            return
        self._failure_count = 0
        self._execute_goal(goal)

    def _execute_goal(self, goal: MsgGoalResult) -> None:
        with self._lock:
            odom = self._latest_odom
        yaw = 0.0
        if odom is not None:
            dx = goal.target_x - odom.x
            dy = goal.target_y - odom.y
            if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                yaw = math.atan2(dy, dx)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        pose = PoseStamped(
            pose=Pose(
                position=Vector3(goal.target_x, goal.target_y, goal.target_z),
                orientation=Quaternion(x=0.0, y=0.0, z=qz, w=qw),
            ),
            frame_id=ACTION_EXECUTOR_MODULE_MAP_FRAME_ID,
        )
        self.goal_pose.publish(pose)
        self.execution_status.publish(json.dumps({
            "state": "executing",
            "action": goal.action,
            "target_label": goal.target_label,
            "confidence": goal.confidence,
            "path": goal.path,
            "target_x": goal.target_x,
            "target_y": goal.target_y,
        }))

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["executing"] = self._executor is not None and getattr(self, "_current", None) is not None
        info["step_count"] = getattr(self, "_step_count", 0)
        info["error_count"] = self._failure_count
        return info

    def lera_recover(self, failed_action: str, original_goal: str) -> str:
        self._failure_count += 1
        if self._failure_count >= self._max_lera_retries:
            return "abort"
        elif self._failure_count >= 2:
            return "expand_search"
        return "retry_different_path"

    def reset_failure_count(self) -> None:
        self._failure_count = 0

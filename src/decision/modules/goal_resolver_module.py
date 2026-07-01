"""GoalResolverModule -- Fast-Slow goal resolution as independent Module.

Wraps GoalResolver (mixin-based) into lingtu.runtime.Module with In/Out ports.
Does NOT rewrite GoalResolver logic -- imports and delegates to goal_resolver.py.

Ports:
  In:  scene_graph (SceneGraph), instruction (str), odometry (Odometry)
  Out: resolved_goal (GoalResult), planner_status (str JSON)
"""

from __future__ import annotations

import asyncio
import json
import logging
import threading
from typing import Any

from runtime.module import Module
from runtime.msgs.nav import Odometry
from runtime.msgs.semantic import GoalResult as MsgGoalResult
from runtime.msgs.semantic import SceneGraph
from runtime.stream import In, Out

logger = logging.getLogger(__name__)


class GoalResolverModule(Module, layer=4):
    """Fast-Slow goal resolution module (Layer 4 Planning).

    Config kwargs:
        fast_path_threshold: float (default 0.75)
        confidence_threshold: float (default 0.6)
        entropy_threshold: float (default 1.5)
        use_slow_path: bool (default True)
    """

    # -- Port declarations --
    scene_graph: In[SceneGraph]
    instruction: In[str]
    odometry: In[Odometry]

    resolved_goal: Out[MsgGoalResult]
    planner_status: Out[str]

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)
        self._fast_path_threshold: float = config.get("fast_path_threshold", 0.75)
        self._confidence_threshold: float = config.get("confidence_threshold", 0.6)
        self._entropy_threshold: float = config.get("entropy_threshold", 1.5)
        self._use_slow_path: bool = config.get("use_slow_path", True)
        self._resolver: Any | None = None
        self._latest_sg: SceneGraph | None = None
        self._latest_odom: Odometry | None = None
        self._lock = threading.Lock()

    def setup(self) -> None:
        """Register port callbacks and lazily build GoalResolver."""
        self.scene_graph.subscribe(self._on_scene_graph)
        self.instruction.subscribe(self._on_instruction)
        self.odometry.subscribe(self._on_odometry)
        self._ensure_resolver()

    def _ensure_resolver(self) -> None:
        """Lazily build GoalResolver -- avoids crash when no LLM key."""
        if self._resolver is not None:
            return
        try:
            from decision.goal_resolution.goal_resolver import GoalResolver
            from decision.llm.llm_client import LLMConfig
            primary = LLMConfig(backend="mock", model="mock")
            self._resolver = GoalResolver(
                primary_config=primary,
                fast_path_threshold=self._fast_path_threshold,
                confidence_threshold=self._confidence_threshold,
            )
        except Exception as e:
            logger.warning("GoalResolver init failed, offline mode: %s", e)
            self._resolver = None

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        with self._lock:
            self._latest_sg = sg

    def _on_odometry(self, odom: Odometry) -> None:
        with self._lock:
            self._latest_odom = odom

    def _on_instruction(self, instruction: str) -> None:
        if not instruction or not instruction.strip():
            return
        self._resolve(instruction.strip())

    def _resolve(self, instruction: str) -> None:
        """Run Fast Path, optionally fall back to Slow Path."""
        self.planner_status.publish(json.dumps({
            "state": "resolving", "instruction": instruction}))
        with self._lock:
            sg = self._latest_sg
            odom = self._latest_odom
        sg_json = sg.to_json() if sg else '{"objects":[],"relations":[],"regions":[]}'
        robot_pos = None
        if odom is not None:
            robot_pos = {"x": odom.x, "y": odom.y, "z": odom.z}
        result = self._try_fast_path(instruction, sg_json, robot_pos)
        if result is not None:
            self._publish_result(result, instruction)
            return
        if self._use_slow_path and self._resolver is not None:
            slow_result = self._try_slow_path(instruction, sg_json, robot_pos)
            if slow_result is not None:
                self._publish_result(slow_result, instruction)
                return
        self.planner_status.publish(json.dumps({
            "state": "failed", "instruction": instruction,
            "reason": "no_match"}))

    def _try_fast_path(self, instruction, sg_json, robot_pos):
        if self._resolver is None:
            return self._offline_fast_path(instruction, sg_json)
        try:
            raw = self._resolver.fast_resolve(instruction, sg_json, robot_pos)
            if raw is not None and raw.is_valid:
                if (raw.score_entropy > self._entropy_threshold
                        and raw.confidence < 0.85):
                    return None
                return self._convert_result(raw)
        except Exception as e:
            logger.warning("Fast path error: %s", e)
        return None

    def _try_slow_path(self, instruction, sg_json, robot_pos):
        if self._resolver is None:
            return None
        try:
            loop = asyncio.new_event_loop()
            try:
                raw = loop.run_until_complete(
                    self._resolver.resolve(instruction, sg_json, robot_pos))
            finally:
                loop.close()
            if raw is not None and raw.is_valid:
                return self._convert_result(raw)
        except Exception as e:
            logger.warning("Slow path error: %s", e)
        return None

    def _offline_fast_path(self, instruction, sg_json):
        """Offline Fast Path -- pure keyword matching."""
        try:
            sg_dict = json.loads(sg_json)
        except (ValueError, TypeError):
            return None
        objects = sg_dict.get("objects", [])
        if not objects:
            return None
        inst_lower = instruction.lower()
        best_obj, best_score = None, 0.0
        for obj in objects:
            label = str(obj.get("label", "")).lower()
            if not label:
                continue
            if label in inst_lower or inst_lower in label:
                score = obj.get("confidence", 0.5)
                if score > best_score:
                    best_score = score
                    best_obj = obj
        if best_obj is None or best_score < 0.1:
            return None
        pos = best_obj.get("position", [0, 0, 0])
        if isinstance(pos, dict):
            x, y, z = pos.get("x", 0), pos.get("y", 0), pos.get("z", 0)
        else:
            x, y = float(pos[0]), float(pos[1])
            z = float(pos[2]) if len(pos) > 2 else 0.0
        return MsgGoalResult(
            action="navigate",
            target_label=str(best_obj.get("label", "")),
            target_x=float(x), target_y=float(y), target_z=float(z),
            confidence=float(best_score),
            reasoning="offline keyword match: " + str(best_obj.get("label", "")),
            is_valid=True, path="fast")

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["fast_hits"] = getattr(self, "_fast_hits", 0)
        info["slow_calls"] = getattr(self, "_slow_calls", 0)
        return info

    @staticmethod
    def _convert_result(raw: Any) -> MsgGoalResult:
        return MsgGoalResult(
            action=getattr(raw, "action", "navigate"),
            target_label=getattr(raw, "target_label", ""),
            target_x=float(getattr(raw, "target_x", 0)),
            target_y=float(getattr(raw, "target_y", 0)),
            target_z=float(getattr(raw, "target_z", 0)),
            confidence=float(getattr(raw, "confidence", 0)),
            reasoning=getattr(raw, "reasoning", ""),
            is_valid=bool(getattr(raw, "is_valid", False)),
            path=getattr(raw, "path", ""),
            hint_room=getattr(raw, "hint_room", ""),
            score_entropy=float(getattr(raw, "score_entropy", 0)))

    def _publish_result(self, result: MsgGoalResult, instruction: str) -> None:
        self.resolved_goal.publish(result)
        self.planner_status.publish(json.dumps({
            "state": "resolved", "instruction": instruction,
            "confidence": result.confidence, "path": result.path,
            "target_label": result.target_label}))

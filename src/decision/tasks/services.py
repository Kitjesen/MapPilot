"""Small decision-domain service wrappers.

These classes are not runtime Modules. They keep strategy objects easy to test
without pulling in Blueprint, ROS, or Gateway code.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


class GoalResolutionService:
    """Fast/slow goal resolution wrapper around a GoalResolver-like object."""

    def __init__(self, resolver: Any) -> None:
        self._resolver = resolver

    def resolve_fast(
        self,
        instruction: str,
        scene_graph_json: str,
        robot_pos: dict | None = None,
    ) -> Any:
        try:
            return self._resolver.fast_resolve(instruction, scene_graph_json, robot_position=robot_pos)
        except Exception:
            logger.exception("Fast path resolution failed")
            return None

    async def resolve_slow(
        self,
        instruction: str,
        scene_graph_json: str,
        robot_pos: dict | None = None,
    ) -> Any:
        try:
            return await self._resolver.resolve(instruction, scene_graph_json, robot_position=robot_pos)
        except Exception:
            logger.exception("Slow path resolution failed")
            return None

    def resolve(
        self,
        instruction: str,
        scene_graph_json: str,
        robot_pos: dict | None = None,
    ) -> Any:
        return self.resolve_fast(instruction, scene_graph_json, robot_pos)

    def resolve_by_tag(self, instruction: str) -> Any:
        if hasattr(self._resolver, "_resolve_by_tag"):
            return self._resolver._resolve_by_tag(instruction)
        return None

    @property
    def resolver(self) -> Any:
        return self._resolver


class FrontierExplorationService:
    """Frontier extraction and scoring wrapper."""

    def __init__(
        self,
        scorer: Any,
        clip_encoder: Any = None,
        semantic_prior: Any = None,
    ) -> None:
        self._scorer = scorer
        if clip_encoder and hasattr(scorer, "set_clip_encoder"):
            scorer.set_clip_encoder(clip_encoder)
        if semantic_prior and hasattr(scorer, "set_semantic_prior_engine"):
            scorer.set_semantic_prior_engine(semantic_prior)

    def evaluate(
        self,
        costmap: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
        robot_pos: np.ndarray,
        instruction: str = "",
        scene_graph: dict | None = None,
    ) -> Any | None:
        try:
            self._scorer.update_costmap(costmap, resolution, origin_x, origin_y)
            frontiers = self._scorer.extract_frontiers(robot_pos)
            if not frontiers:
                return None
            scene = scene_graph or {}
            self._scorer.score_frontiers(
                instruction=instruction,
                robot_position=robot_pos,
                scene_objects=scene.get("objects"),
                scene_relations=scene.get("relations"),
                scene_rooms=scene.get("rooms"),
            )
            return self._scorer.get_best_frontier()
        except Exception:
            logger.exception("Frontier evaluation failed")
            return None

    def record_failure(self, frontier_pos: np.ndarray) -> None:
        self._scorer.record_frontier_failure(frontier_pos)

    @property
    def scorer(self) -> Any:
        return self._scorer


class ActionExecutionService:
    """Action command generation and LERa recovery wrapper."""

    def __init__(self, executor: Any, llm_client: Any = None) -> None:
        self._executor = executor
        self._llm = llm_client

    def navigate(self, target_pos: np.ndarray, robot_pos: np.ndarray) -> Any:
        return self._executor.generate_navigate_command(
            self._position(target_pos), self._position(robot_pos),
        )

    @staticmethod
    def _position(position: np.ndarray) -> dict[str, float]:
        return dict(zip(("x", "y", "z"), map(float, position)))

    def approach(
        self,
        target_pos: np.ndarray,
        robot_pos: np.ndarray,
        stop_distance: float = 1.0,
    ) -> Any:
        return self._executor.generate_approach_command(
            self._position(target_pos),
            self._position(robot_pos),
            stop_distance=stop_distance,
        )

    def look_around(self, robot_pos: np.ndarray) -> Any:
        return self._executor.generate_look_around_command()

    async def recover(
        self,
        failed_action: str,
        scene_state: dict,
        event_loop: Any = None,
    ) -> str:
        """Recover from current_labels, original_goal and failure_count in scene_state."""
        try:
            return await asyncio.to_thread(
                self._executor.lera_recover,
                failed_action=failed_action,
                current_labels=scene_state.get("current_labels", []),
                original_goal=scene_state.get("original_goal", failed_action),
                failure_count=scene_state.get("failure_count", 1),
                llm_client=self._llm,
                event_loop=event_loop,
            )
        except Exception:
            logger.exception("LERa recovery failed")
            return "retry_different_path"

    @property
    def executor(self) -> Any:
        return self._executor


__all__ = [
    "ActionExecutionService",
    "FrontierExplorationService",
    "GoalResolutionService",
]

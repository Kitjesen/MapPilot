"""FrontierModule — Frontier 探索评分模块 (Module 模式封装)。

将 FrontierScorer + exploration_strategy 的纯算法逻辑封装为
Module In/Out 端口接口，供 Blueprint autoconnect 使用。

层级: L4 (Planning)
输入: scene_graph, odometry, instruction
输出: frontier_goal (最佳探索目标), frontier_scores (调试信息)
"""

from __future__ import annotations

import logging
import math
from typing import Any

from runtime import In, Module, Out
from runtime.msgs import Odometry, PoseStamped, SceneGraph
from runtime.msgs.geometry import Pose, Quaternion, Vector3
from runtime.msgs.numpy_compat import np
from runtime.runtime_interface import map_frame_id

from decision.exploration.exploration_strategy import extract_frontier_scene_data
from decision.exploration.frontier_scorer import FrontierScorer

logger = logging.getLogger(__name__)
FRONTIER_MODULE_MAP_FRAME_ID = map_frame_id()


class FrontierModule(Module, layer=4):
    """Frontier 探索评分模块。

    接收场景图、里程计和指令，评分 frontiers 后发布最佳探索目标。

    Config:
        min_frontier_size: 最小 frontier 大小 (default 5)
        max_frontiers: 最大 frontier 数 (default 10)
        score_threshold: 最低发布阈值 (default 0.2)
        **kwargs: 传递给 FrontierScorer 的其他参数
    """

    # -- 端口声明 --
    scene_graph: In[SceneGraph]
    odometry: In[Odometry]
    instruction: In[str]

    frontier_goal: Out[PoseStamped]
    frontier_scores: Out[dict]

    def __init__(self, **config: Any) -> None:
        self._score_threshold = config.pop("score_threshold", 0.2)

        scorer_kwargs = {}
        for key in (
            "min_frontier_size", "max_frontiers", "distance_weight",
            "novelty_weight", "language_weight", "grounding_weight",
            "tsp_reorder", "nearby_object_radius",
        ):
            if key in config:
                scorer_kwargs[key] = config.pop(key)

        super().__init__(**config)
        self._scorer = FrontierScorer(**scorer_kwargs)
        self._last_odom: Odometry | None = None
        self._last_instruction: str = ""
        self._last_sg: SceneGraph | None = None
        self._visited: list[np.ndarray] = []

    # -- 生命周期 --

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.instruction.subscribe(self._on_instruction)
        self.scene_graph.subscribe(self._on_scene_graph)

    def _on_odom(self, odom: Odometry) -> None:
        if not (math.isfinite(odom.x) and math.isfinite(odom.y)):
            return
        self._last_odom = odom

    def _on_instruction(self, instr: str) -> None:
        self._last_instruction = instr
        self._scorer.clear_failure_memory()

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        self._last_sg = sg
        self._evaluate()

    # -- 核心逻辑 --

    def _evaluate(self) -> None:
        """评分 frontiers 并发布最佳目标。"""
        if self._last_odom is None or self._last_sg is None:
            return
        if not self._last_instruction:
            return

        odom = self._last_odom
        robot_pos = np.array([odom.x, odom.y], dtype=np.float64)

        # FrontierScorer 需要 costmap grid，没有则无法提取 frontiers
        if self._scorer._grid is None:
            return

        frontiers = self._scorer.extract_frontiers(robot_pos)
        if not frontiers:
            return

        # 从场景图提取评分所需数据
        sg_json = self._last_sg.to_json()
        objects, relations, rooms = extract_frontier_scene_data(sg_json)

        self._scorer.score_frontiers(
            instruction=self._last_instruction,
            robot_position=robot_pos,
            visited_positions=self._visited if self._visited else None,
            scene_objects=objects,
            scene_relations=relations,
            scene_rooms=rooms,
        )

        best = self._scorer.get_best_frontier()
        if best is None or best.score < self._score_threshold:
            return

        # 记录已访问位置
        self._visited.append(robot_pos.copy())

        # 发布最佳 frontier 为 PoseStamped
        goal = PoseStamped(
            pose=Pose(
                position=Vector3(
                    float(best.center_world[0]),
                    float(best.center_world[1]),
                    float(odom.z),
                ),
                orientation=Quaternion(),
            ),
            frame_id=FRONTIER_MODULE_MAP_FRAME_ID,
        )
        self.frontier_goal.publish(goal)

        # 发布评分详情
        scores_dict = {
            "frontier_count": len(frontiers),
            "best_score": float(best.score),
            "best_direction": best.direction_label,
            "best_nearby_labels": best.nearby_labels[:5],
            "best_description": getattr(best, "description", ""),
        }
        self.frontier_scores.publish(scores_dict)

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["frontier_count"] = len(getattr(self._scorer, "_frontiers", []))
        info["exploration_active"] = self._last_instruction != ""
        return info

    # -- 外部 API --

    def update_costmap(
        self,
        grid_data: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
    ) -> None:
        """更新 costmap 数据 (由外部调用)。"""
        self._scorer.update_costmap(grid_data, resolution, origin_x, origin_y)

    def record_failure(self, position: np.ndarray) -> None:
        """记录探索失败位置。"""
        self._scorer.record_frontier_failure(position)

    @property
    def scorer(self) -> FrontierScorer:
        """访问内部 FrontierScorer (测试用)。"""
        return self._scorer

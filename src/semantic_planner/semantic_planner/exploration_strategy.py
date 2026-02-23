"""
探索策略辅助模块。

将 Frontier 相关的纯算法逻辑从 ROS2 节点中剥离出来，便于单测。
"""

import json
from typing import Dict, List, Tuple, Optional

import numpy as np

from .frontier_scorer import FrontierScorer
from .goal_resolver import GoalResult


def extract_frontier_scene_data(
    scene_graph_json: str,
) -> Tuple[List[Dict], List[Dict], List[Dict]]:
    """从场景图中抽取 Frontier 评分需要的 objects/relations/rooms 子集。"""
    try:
        sg = json.loads(scene_graph_json)
    except (json.JSONDecodeError, TypeError):
        return [], [], []

    raw_objects = sg.get("objects", [])
    raw_relations = sg.get("relations", [])
    raw_rooms = sg.get("rooms", sg.get("regions", []))
    if not isinstance(raw_objects, list):
        raw_objects = []
    if not isinstance(raw_relations, list):
        raw_relations = []
    if not isinstance(raw_rooms, list):
        raw_rooms = []

    objects: List[Dict] = []
    for obj in raw_objects:
        if not isinstance(obj, dict):
            continue
        pos = obj.get("position", {})

        try:
            # 支持 dict {"x": ..., "y": ...} 和 list/tuple [x, y, z] 两种格式
            if isinstance(pos, (list, tuple)):
                if len(pos) < 2:
                    continue
                x = float(pos[0])
                y = float(pos[1])
            elif isinstance(pos, dict):
                if "x" not in pos or "y" not in pos:
                    continue
                x = float(pos.get("x", 0.0))
                y = float(pos.get("y", 0.0))
            else:
                continue

            if not np.isfinite(x) or not np.isfinite(y):
                continue

            objects.append({
                "id": obj.get("id", -1),
                "label": str(obj.get("label", "")),
                "position": {
                    "x": x,
                    "y": y,
                },
            })
        except (TypeError, ValueError):
            continue

    relations = [r for r in raw_relations if isinstance(r, dict)]
    rooms = [r for r in raw_rooms if isinstance(r, dict)]
    return objects, relations, rooms


def generate_frontier_goal(
    frontier_scorer: FrontierScorer,
    instruction: str,
    robot_position: Dict[str, float],
    visited_positions: Optional[List[np.ndarray]],
    scene_graph_json: str,
    score_threshold: float = 0.2,
) -> Optional[GoalResult]:
    """基于 Frontier 评分器生成探索目标。"""
    robot_xy = np.array([robot_position["x"], robot_position["y"]], dtype=np.float64)

    frontiers = frontier_scorer.extract_frontiers(robot_xy)
    if not frontiers:
        return None

    scene_objects, scene_relations, scene_rooms = extract_frontier_scene_data(scene_graph_json)
    frontier_scorer.score_frontiers(
        instruction=instruction,
        robot_position=robot_xy,
        visited_positions=visited_positions,
        scene_objects=scene_objects,
        scene_relations=scene_relations,
        scene_rooms=scene_rooms,
    )
    best = frontier_scorer.get_best_frontier()
    if best is None or best.score < score_threshold:
        return None

    nearby_text = ", ".join(best.nearby_labels[:3]) if best.nearby_labels else "none"
    reasoning = (
        f"Frontier score={best.score:.2f}, dir={best.direction_label}, "
        f"size={best.size}, nearby={nearby_text}"
    )
    return GoalResult(
        action="explore",
        target_x=float(best.center_world[0]),
        target_y=float(best.center_world[1]),
        target_z=float(robot_position.get("z", 0.0)),
        target_label=f"frontier:{best.direction_label}",
        confidence=float(best.score),
        reasoning=reasoning,
        is_valid=True,
        path="frontier",
    )

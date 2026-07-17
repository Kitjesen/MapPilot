"""Pure frontier strategy helpers used by semantic planning."""

from __future__ import annotations

import json
import math
from typing import TYPE_CHECKING

from runtime.msgs.numpy_compat import np

from .scorer import FrontierScorer

if TYPE_CHECKING:
    from decision.goals.resolver import GoalResult


def extract_frontier_scene_data(
    scene_graph_json: str,
) -> tuple[list[dict], list[dict], list[dict]]:
    """Extract frontier scene data."""
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

    objects: list[dict] = []
    for obj in raw_objects:
        if not isinstance(obj, dict):
            continue
        pos = obj.get("position", {})

        try:
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

            if not math.isfinite(x) or not math.isfinite(y):
                continue

            objects.append(
                {
                    "id": obj.get("id", -1),
                    "label": str(obj.get("label", "")),
                    "position": {
                        "x": x,
                        "y": y,
                    },
                }
            )
        except (TypeError, ValueError):
            continue

    relations = [r for r in raw_relations if isinstance(r, dict)]
    rooms = [r for r in raw_rooms if isinstance(r, dict)]
    return objects, relations, rooms


def generate_frontier_goal(
    frontier_scorer: FrontierScorer,
    instruction: str,
    robot_position: dict[str, float],
    visited_positions: list[np.ndarray] | None,
    scene_graph_json: str,
    score_threshold: float = 0.2,
) -> GoalResult | None:
    """Generate frontier goal."""
    try:
        rx = float(robot_position.get("x", 0.0))
        ry = float(robot_position.get("y", 0.0))
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(rx) and math.isfinite(ry)):
        return None
    robot_xy = np.array([rx, ry], dtype=np.float64)

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
    reasoning = f"Frontier score={best.score:.2f}, dir={best.direction_label}, size={best.size}, nearby={nearby_text}"
    from decision.goals.resolver import GoalResult

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

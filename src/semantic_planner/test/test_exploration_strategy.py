"""
test_exploration_strategy.py — Frontier 探索策略纯逻辑测试
"""

import json
import unittest

import numpy as np

from semantic_planner.frontier_scorer import FrontierScorer, FREE_CELL, UNKNOWN_CELL
from semantic_planner.exploration_strategy import (
    extract_frontier_scene_data,
    generate_frontier_goal,
)


class TestExtractFrontierSceneData(unittest.TestCase):
    """场景图提取测试。"""

    def test_extract_filters_invalid_objects(self):
        sg = json.dumps({
            "objects": [
                {"id": 1, "label": "door", "position": {"x": 1.2, "y": 2.3}},
                {"id": 2, "label": "bad-no-pos"},
                {"id": 3, "label": "bad-pos", "position": {"x": "nan", "y": 1}},
                "not-a-dict",
            ],
            "relations": [
                {"subject_id": 1, "relation": "near", "object_id": 2},
                "invalid-relation",
            ],
        })

        objects, relations, rooms = extract_frontier_scene_data(sg)

        self.assertEqual(len(objects), 1)
        self.assertEqual(objects[0]["label"], "door")
        self.assertEqual(len(relations), 1)
        self.assertEqual(len(rooms), 0)

    def test_extract_bad_json_returns_empty(self):
        objects, relations, rooms = extract_frontier_scene_data("not-json")
        self.assertEqual(objects, [])
        self.assertEqual(relations, [])
        self.assertEqual(rooms, [])


    def test_extract_rooms_from_scene_graph(self):
        sg = json.dumps({
            "objects": [
                {"id": 1, "label": "desk", "position": {"x": 1.0, "y": 2.0}},
            ],
            "relations": [],
            "rooms": [
                {"room_id": 0, "name": "office", "center": {"x": 1.0, "y": 2.0}},
            ],
        })

        objects, relations, rooms = extract_frontier_scene_data(sg)
        self.assertEqual(len(objects), 1)
        self.assertEqual(len(rooms), 1)
        self.assertEqual(rooms[0]["name"], "office")


class TestGenerateFrontierGoal(unittest.TestCase):
    """Frontier 目标生成测试。"""

    @staticmethod
    def _make_scorer_with_costmap() -> FrontierScorer:
        scorer = FrontierScorer(min_frontier_size=3, max_frontiers=6)
        grid = np.full((20, 20), UNKNOWN_CELL, dtype=np.int8)
        grid[:10, :] = FREE_CELL
        scorer.update_costmap(grid, resolution=0.1, origin_x=0.0, origin_y=0.0)
        return scorer

    def test_generate_frontier_goal_success(self):
        scorer = self._make_scorer_with_costmap()
        scene_graph = json.dumps({
            "objects": [
                {
                    "id": 1,
                    "label": "door",
                    "position": {"x": 1.0, "y": 1.0, "z": 0.0},
                }
            ],
            "relations": [],
        })

        result = generate_frontier_goal(
            frontier_scorer=scorer,
            instruction="find door",
            robot_position={"x": 0.5, "y": 0.5, "z": 0.0},
            visited_positions=[],
            scene_graph_json=scene_graph,
            score_threshold=0.0,
        )

        self.assertIsNotNone(result)
        self.assertTrue(result.is_valid)
        self.assertEqual(result.action, "explore")
        self.assertEqual(result.path, "frontier")
        self.assertTrue(result.target_label.startswith("frontier:"))

    def test_generate_frontier_goal_without_costmap_returns_none(self):
        scorer = FrontierScorer(min_frontier_size=3)
        scene_graph = json.dumps({"objects": [], "relations": []})

        result = generate_frontier_goal(
            frontier_scorer=scorer,
            instruction="find door",
            robot_position={"x": 0.5, "y": 0.5, "z": 0.0},
            visited_positions=[],
            scene_graph_json=scene_graph,
            score_threshold=0.0,
        )

        self.assertIsNone(result)

    def test_generate_frontier_goal_respects_threshold(self):
        scorer = self._make_scorer_with_costmap()
        scene_graph = json.dumps({"objects": [], "relations": []})

        result = generate_frontier_goal(
            frontier_scorer=scorer,
            instruction="find door",
            robot_position={"x": 0.5, "y": 0.5, "z": 0.0},
            visited_positions=[],
            scene_graph_json=scene_graph,
            score_threshold=1.1,
        )

        self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()

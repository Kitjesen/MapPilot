"""Decision module."""

import asyncio
import json
import unittest

import numpy as np

from decision.frontiers.scorer import Frontier
from decision.goals.sgnav import SGNavReasoner


class TestSGNavReasoner(unittest.TestCase):
    """Test S G Nav Reasoner."""

    def setUp(self):
        self.loop = asyncio.new_event_loop()

    def tearDown(self):
        self.loop.close()

    def test_reason_subgraphs_prefers_relevant_room(self):
        reasoner = SGNavReasoner(use_llm_reasoning=False, max_subgraphs=4)
        scene_graph = json.dumps(
            {
                "subgraphs": [
                    {
                        "subgraph_id": "room_0",
                        "level": "room",
                        "center": {"x": 1.0, "y": 1.0},
                        "object_ids": [1, 2],
                        "object_labels": ["door", "sign"],
                        "relation_count": 2,
                    },
                    {
                        "subgraph_id": "room_1",
                        "level": "room",
                        "center": {"x": 8.0, "y": 8.0},
                        "object_ids": [3],
                        "object_labels": ["chair"],
                        "relation_count": 0,
                    },
                ]
            }
        )

        scores = self.loop.run_until_complete(
            reasoner.reason_subgraphs(
                instruction="find the door",
                scene_graph_json=scene_graph,
                robot_position={"x": 0.0, "y": 0.0, "z": 0.0},
                language="en",
                llm_chat=None,
            )
        )

        self.assertGreater(len(scores), 0)
        self.assertEqual(scores[0].subgraph_id, "room_0")

    def test_select_frontier_with_subgraph_interpolation(self):
        reasoner = SGNavReasoner(
            use_llm_reasoning=False,
            frontier_base_weight=0.3,
            max_subgraphs=4,
        )

        scene_graph = json.dumps(
            {
                "subgraphs": [
                    {
                        "subgraph_id": "room_0",
                        "level": "room",
                        "center": {"x": 1.0, "y": 1.0},
                        "object_ids": [1],
                        "object_labels": ["fire extinguisher", "door"],
                        "relation_count": 2,
                    },
                    {
                        "subgraph_id": "room_1",
                        "level": "room",
                        "center": {"x": 8.0, "y": 8.0},
                        "object_ids": [2],
                        "object_labels": ["desk"],
                        "relation_count": 0,
                    },
                ]
            }
        )

        f_near = Frontier(
            frontier_id=0,
            center=np.array([10.0, 10.0]),
            center_world=np.array([1.2, 1.1]),
            size=20,
            score=0.35,
            distance=1.0,
            direction_label="north",
        )
        f_far = Frontier(
            frontier_id=1,
            center=np.array([80.0, 80.0]),
            center_world=np.array([8.0, 8.0]),
            size=24,
            score=0.8,
            distance=8.0,
            direction_label="east",
        )

        selection = self.loop.run_until_complete(
            reasoner.select_frontier(
                instruction="find fire extinguisher",
                scene_graph_json=scene_graph,
                robot_position={"x": 0.0, "y": 0.0, "z": 0.0},
                frontiers=[f_near, f_far],
                language="en",
                llm_chat=None,
            )
        )

        self.assertIsNotNone(selection)
        self.assertEqual(selection.frontier.frontier_id, 0)

    def test_reperception_rejects_low_credibility_target(self):
        reasoner = SGNavReasoner(
            use_llm_reasoning=False,
            reject_threshold=0.35,
            false_positive_penalty=0.25,
        )
        scene_graph = json.dumps(
            {
                "objects": [
                    {"id": 1, "label": "chair", "score": 0.8},
                    {"id": 2, "label": "desk", "score": 0.7},
                ]
            }
        )

        reject, cred, _ = reasoner.evaluate_target_credibility(
            target_label="fire extinguisher",
            scene_graph_json=scene_graph,
            path_confidence=0.2,
            confirmed_visible=False,
        )

        self.assertTrue(reject)
        self.assertLess(cred, 0.35)

    def test_reperception_accepts_confirmed_target(self):
        reasoner = SGNavReasoner(use_llm_reasoning=False)
        scene_graph = json.dumps(
            {
                "objects": [
                    {"id": 1, "label": "fire extinguisher", "score": 0.9},
                ]
            }
        )

        reject, cred, _ = reasoner.evaluate_target_credibility(
            target_label="fire extinguisher",
            scene_graph_json=scene_graph,
            path_confidence=0.4,
            confirmed_visible=True,
        )

        self.assertFalse(reject)
        self.assertGreaterEqual(cred, 0.5)

    def test_room_gate_boosts_group_inside_top_room(self):
        reasoner = SGNavReasoner(
            use_llm_reasoning=False,
            room_gate_weight=0.4,
            max_subgraphs=6,
        )
        scene_graph = json.dumps(
            {
                "subgraphs": [
                    {
                        "subgraph_id": "room_0",
                        "level": "room",
                        "room_id": 0,
                        "center": {"x": 1.0, "y": 1.0},
                        "object_ids": [1],
                        "object_labels": ["door"],
                        "relation_count": 1,
                    },
                    {
                        "subgraph_id": "room_1",
                        "level": "room",
                        "room_id": 1,
                        "center": {"x": 8.0, "y": 8.0},
                        "object_ids": [2],
                        "object_labels": ["desk"],
                        "relation_count": 0,
                    },
                    {
                        "subgraph_id": "group_0",
                        "level": "group",
                        "room_id": 0,
                        "center": {"x": 1.2, "y": 1.1},
                        "object_ids": [1],
                        "object_labels": ["door"],
                        "relation_count": 0,
                    },
                    {
                        "subgraph_id": "group_1",
                        "level": "group",
                        "room_id": 1,
                        "center": {"x": 8.1, "y": 8.0},
                        "object_ids": [2],
                        "object_labels": ["desk"],
                        "relation_count": 0,
                    },
                ]
            }
        )

        scores = self.loop.run_until_complete(
            reasoner.reason_subgraphs(
                instruction="find the door",
                scene_graph_json=scene_graph,
                robot_position={"x": 0.0, "y": 0.0, "z": 0.0},
                language="en",
                llm_chat=None,
            )
        )

        score_map = {s.subgraph_id: s.score for s in scores}
        self.assertGreater(score_map.get("group_0", 0.0), score_map.get("group_1", 0.0))


if __name__ == "__main__":
    unittest.main()

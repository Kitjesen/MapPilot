"""Decision module."""

import json
import logging
import unittest
from unittest.mock import patch

from decision.goals.resolver import GoalResolver
from decision.llm.client import LLMConfig


def _make_scene_graph(objects, relations=None, regions=None):
    """Make scene graph."""
    return json.dumps(
        {
            "timestamp": 0,
            "object_count": len(objects),
            "objects": objects,
            "relations": relations or [],
            "regions": regions or [],
            "summary": "test scene",
        }
    )


class TestFastResolve(unittest.TestCase):
    """Test Fast Resolve."""

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="test")
        self.resolver = GoalResolver(self.config, fast_path_threshold=0.5)

    def test_exact_label_match(self):
        """Test exact label match."""
        sg = _make_scene_graph(
            [
                {
                    "id": 0,
                    "label": "chair",
                    "position": {"x": 3.0, "y": 2.0, "z": 0.0},
                    "score": 0.9,
                    "detection_count": 5,
                },
            ]
        )
        result = self.resolver.fast_resolve("go to the chair", sg)
        self.assertIsNotNone(result)
        self.assertEqual(result.action, "navigate")
        self.assertEqual(result.target_label, "chair")
        self.assertAlmostEqual(result.target_x, 3.0)
        self.assertEqual(result.path, "fast")

    def test_no_match_returns_none(self):
        """Test no match returns none."""
        sg = _make_scene_graph(
            [
                {"id": 0, "label": "chair", "position": {"x": 1, "y": 1, "z": 0}, "score": 0.9, "detection_count": 3},
            ]
        )
        result = self.resolver.fast_resolve("find the elephant", sg)
        self.assertIsNone(result)

    def test_low_confidence_defers(self):
        """Test low confidence defers."""
        resolver = GoalResolver(self.config, fast_path_threshold=0.95)
        sg = _make_scene_graph(
            [
                {"id": 0, "label": "chair", "position": {"x": 1, "y": 1, "z": 0}, "score": 0.3, "detection_count": 1},
            ]
        )
        result = resolver.fast_resolve("go to chair", sg)
        self.assertIsNone(result)

    def test_partial_keyword_match(self):
        """Test partial keyword match."""
        sg = _make_scene_graph(
            [
                {
                    "id": 0,
                    "label": "fire extinguisher",
                    "position": {"x": 5, "y": 5, "z": 0},
                    "score": 0.85,
                    "detection_count": 4,
                },
            ]
        )
        result = self.resolver.fast_resolve("找灭火器 fire", sg)

        self.assertIsNotNone(result)

    def test_spatial_relation_boosts_score(self):
        """Test spatial relation boosts score."""
        sg = _make_scene_graph(
            objects=[
                {"id": 0, "label": "chair", "position": {"x": 3, "y": 2, "z": 0}, "score": 0.8, "detection_count": 3},
                {"id": 1, "label": "door", "position": {"x": 3, "y": 3, "z": 0}, "score": 0.9, "detection_count": 5},
            ],
            relations=[
                {"subject_id": 0, "relation": "near", "object_id": 1, "distance": 1.0},
            ],
        )

        result = self.resolver.fast_resolve("chair near the door", sg)
        self.assertIsNotNone(result)

        self.assertIn(result.target_label, ["chair", "door"])

    def test_empty_scene_graph(self):
        result = self.resolver.fast_resolve("go to chair", "{}")
        self.assertIsNone(result)

    def test_invalid_json(self):
        result = self.resolver.fast_resolve("go to chair", "not json")
        self.assertIsNone(result)

    def test_distance_preference(self):
        """Test distance preference."""
        sg = _make_scene_graph(
            [
                {
                    "id": 0,
                    "label": "chair",
                    "position": {"x": 100, "y": 100, "z": 0},
                    "score": 0.9,
                    "detection_count": 5,
                },
                {"id": 1, "label": "chair", "position": {"x": 2, "y": 2, "z": 0}, "score": 0.88, "detection_count": 5},
            ]
        )
        result = self.resolver.fast_resolve(
            "go to the chair",
            sg,
            robot_position={"x": 0, "y": 0, "z": 0},
        )
        self.assertIsNotNone(result)

        self.assertAlmostEqual(result.target_x, 2.0)

    def test_chinese_instruction(self):
        """Test chinese instruction."""
        sg = _make_scene_graph(
            [
                {"id": 0, "label": "灭火器", "position": {"x": 4, "y": 3, "z": 0}, "score": 0.9, "detection_count": 3},
            ]
        )
        result = self.resolver.fast_resolve("去灭火器那里", sg)
        self.assertIsNotNone(result)
        self.assertEqual(result.target_label, "灭火器")


class TestSelectiveGrounding(unittest.TestCase):
    """Test Selective Grounding."""

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="test")
        self.resolver = GoalResolver(self.config)

    def test_small_scene_not_filtered(self):
        """Test small scene not filtered."""
        sg = _make_scene_graph(
            [
                {"id": i, "label": f"obj_{i}", "position": {"x": i, "y": 0, "z": 0}, "score": 0.5, "detection_count": 1}
                for i in range(5)
            ]
        )
        result = self.resolver._selective_grounding("find obj_1", sg, max_objects=15)
        result_data = json.loads(result)
        self.assertEqual(result_data["object_count"], 5)

    def test_large_scene_filtered(self):
        """Test large scene filtered."""
        objects = [
            {
                "id": i,
                "label": f"random_obj_{i}",
                "position": {"x": i, "y": 0, "z": 0},
                "score": 0.5,
                "detection_count": 1,
            }
            for i in range(30)
        ]

        objects.append(
            {
                "id": 99,
                "label": "red door",
                "position": {"x": 50, "y": 50, "z": 0},
                "score": 0.9,
                "detection_count": 5,
            }
        )
        sg = _make_scene_graph(objects)
        result = self.resolver._selective_grounding("find the red door", sg, max_objects=10)
        result_data = json.loads(result)
        self.assertLessEqual(result_data["object_count"], 10)

        labels = [o["label"] for o in result_data["objects"]]
        self.assertIn("red door", labels)

    def test_relation_hop_expansion(self):
        """Test relation hop expansion."""
        objects = [
            {"id": i, "label": f"noise_{i}", "position": {"x": i, "y": 0, "z": 0}, "score": 0.3, "detection_count": 1}
            for i in range(25)
        ]

        objects.append(
            {"id": 100, "label": "door", "position": {"x": 10, "y": 10, "z": 0}, "score": 0.9, "detection_count": 5}
        )
        objects.append(
            {"id": 101, "label": "sign", "position": {"x": 11, "y": 10, "z": 0}, "score": 0.8, "detection_count": 3}
        )
        sg = _make_scene_graph(
            objects,
            relations=[{"subject_id": 100, "relation": "near", "object_id": 101, "distance": 1.0}],
        )
        result = self.resolver._selective_grounding("find the door", sg, max_objects=10)
        result_data = json.loads(result)
        filtered_ids = {o["id"] for o in result_data["objects"]}
        self.assertIn(100, filtered_ids)
        self.assertIn(101, filtered_ids)

    def test_no_match_fallback_to_top_score(self):
        """Test no match fallback to top score."""
        objects = [
            {
                "id": i,
                "label": f"item_{i}",
                "position": {"x": i, "y": 0, "z": 0},
                "score": float(i) / 25,
                "detection_count": i + 1,
            }
            for i in range(25)
        ]
        sg = _make_scene_graph(objects)
        result = self.resolver._selective_grounding("find the unicorn", sg, max_objects=5)
        result_data = json.loads(result)
        self.assertLessEqual(result_data["object_count"], 5)

    def test_invalid_json_returns_original(self):
        result = self.resolver._selective_grounding("test", "not json")
        self.assertEqual(result, "not json")


class TestKeywordExtraction(unittest.TestCase):
    """Test Keyword Extraction."""

    def test_english_keywords(self):
        kws = GoalResolver._extract_keywords("go to the red chair near the door")

        self.assertIn("red", kws)
        self.assertIn("chair", kws)
        self.assertIn("door", kws)
        self.assertNotIn("the", kws)
        self.assertNotIn("to", kws)

    def test_chinese_keywords(self):
        kws = GoalResolver._extract_keywords("去红色灭火器旁边")

        all_text = " ".join(kws)
        self.assertIn("灭火器", all_text)

    def test_mixed_keywords(self):
        kws = GoalResolver._extract_keywords("找fire extinguisher")
        self.assertIn("fire", kws)
        self.assertIn("extinguisher", kws)


class TestRoomFallback(unittest.TestCase):
    """Test Room Fallback."""

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="test")
        self.resolver = GoalResolver(self.config, fast_path_threshold=0.75)

    def _make_sg_with_rooms(self, objects=None, rooms=None):
        """Make sg with rooms."""
        return json.dumps(
            {
                "timestamp": 0,
                "object_count": len(objects or []),
                "objects": objects or [],
                "relations": [],
                "rooms": rooms or [],
            }
        )

    def test_room_keyword_match_chinese(self):
        """Test room keyword match chinese."""
        sg = self._make_sg_with_rooms(
            objects=[
                {"id": 0, "label": "table", "position": [1, 1, 0], "score": 0.3, "detection_count": 1},
            ],
            rooms=[
                {"name": "厨房", "center": [5.0, 3.0, 0.0], "object_ids": [0]},
                {"name": "客厅", "center": [10.0, 8.0, 0.0], "object_ids": []},
            ],
        )
        result = self.resolver.fast_resolve("去厨房", sg)
        self.assertIsNotNone(result)
        self.assertEqual(result.path, "fast")
        self.assertEqual(result.target_label, "厨房")
        self.assertAlmostEqual(result.target_x, 5.0)
        self.assertAlmostEqual(result.target_y, 3.0)
        self.assertEqual(result.hint_room, "厨房")
        self.assertIsNotNone(result.hint_room_center)
        # confidence = match_score * 0.6, match_score >= 0.7
        self.assertGreater(result.confidence, 0.0)
        self.assertLessEqual(result.confidence, 1.0)

    def test_room_keyword_match_english(self):
        """Test room keyword match english."""
        sg = self._make_sg_with_rooms(
            rooms=[
                {"name": "kitchen", "center": [4.0, 2.0, 0.0], "object_ids": []},
                {"name": "bedroom", "center": [12.0, 6.0, 0.0], "object_ids": []},
            ],
        )
        result = self.resolver.fast_resolve("find the kitchen", sg)
        self.assertIsNotNone(result)
        self.assertEqual(result.target_label, "kitchen")
        self.assertAlmostEqual(result.target_x, 4.0)

    def test_room_fallback_not_triggered_when_object_matches(self):
        """Test room fallback not triggered when object matches."""
        sg = self._make_sg_with_rooms(
            objects=[
                {"id": 0, "label": "chair", "position": [3.0, 2.0, 0.0], "score": 0.9, "detection_count": 5},
            ],
            rooms=[
                {"name": "chair_room", "center": [10.0, 10.0, 0.0], "object_ids": [0]},
            ],
        )
        result = self.resolver.fast_resolve("go to the chair", sg)
        self.assertIsNotNone(result)
        self.assertEqual(result.target_label, "chair")

        self.assertAlmostEqual(result.target_x, 3.0)

    def test_room_no_match_returns_none(self):
        """Test room no match returns none."""
        sg = self._make_sg_with_rooms(
            rooms=[
                {"name": "kitchen", "center": [4.0, 2.0, 0.0], "object_ids": []},
            ],
        )
        result = self.resolver.fast_resolve("find the elephant", sg)
        self.assertIsNone(result)

    def test_room_center_dict_format(self):
        """Test room center dict format."""
        sg = self._make_sg_with_rooms(
            rooms=[
                {"name": "走廊", "center": {"x": 7.0, "y": 4.0, "z": 0.0}, "object_ids": []},
            ],
        )
        result = self.resolver.fast_resolve("到走廊", sg)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.target_x, 7.0)
        self.assertAlmostEqual(result.target_y, 4.0)

    def test_room_no_center_skipped(self):
        """Test room no center skipped."""
        sg = self._make_sg_with_rooms(
            rooms=[
                {"name": "厨房", "object_ids": []},  # no center
            ],
        )
        result = self.resolver.fast_resolve("去厨房", sg)
        self.assertIsNone(result)

    def test_room_empty_rooms_returns_none(self):
        """Test room empty rooms returns none."""
        sg = self._make_sg_with_rooms(rooms=[])
        result = self.resolver.fast_resolve("去厨房", sg)
        self.assertIsNone(result)


class TestFastPathScoringWeights(unittest.TestCase):
    def test_loads_weights_from_yaml(self):
        import decision.goals.fast as fast

        fast._fast_path_weights_loaded = False
        config = {"fast_path_fusion": {"label": 0.4, "clip": 0.3, "detector": 0.2, "spatial": 0.1}}
        with patch.object(fast, "_load_semantic_scoring_yaml", return_value=config):
            fast._load_fast_path_weights()

        self.assertAlmostEqual(fast.WEIGHT_LABEL_MATCH, 0.4)
        self.assertAlmostEqual(fast.WEIGHT_CLIP_SIM, 0.3)
        self.assertAlmostEqual(fast.WEIGHT_DETECTOR_SCORE, 0.2)
        self.assertAlmostEqual(fast.WEIGHT_SPATIAL_HINT, 0.1)

    def test_missing_section_uses_defaults_and_logs(self):
        import decision.goals.fast as fast

        fast._fast_path_weights_loaded = False
        fast.WEIGHT_LABEL_MATCH = fast._DEFAULTS_FAST_PATH["label"]
        fast.WEIGHT_CLIP_SIM = fast._DEFAULTS_FAST_PATH["clip"]
        fast.WEIGHT_DETECTOR_SCORE = fast._DEFAULTS_FAST_PATH["detector"]
        fast.WEIGHT_SPATIAL_HINT = fast._DEFAULTS_FAST_PATH["spatial"]

        with self.assertLogs(level=logging.INFO) as logs:
            with patch.object(fast, "_load_semantic_scoring_yaml", return_value={}):
                fast._load_fast_path_weights()

        self.assertAlmostEqual(fast.WEIGHT_LABEL_MATCH, 0.35)
        self.assertIn("default weights", "\n".join(logs.output))


if __name__ == "__main__":
    unittest.main()

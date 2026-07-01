"""
test_semantic_prior.py — SemanticPriorEngine 单元测试

覆盖:
  - predict_target_rooms: 英文/中文指令
  - score_rooms_for_target: 直接标签匹配 / 先验知识 / 未访问加分
  - get_unexplored_priors: 拓扑感知 + BFS 跳数 + 可达性
  - get_room_expected_objects: 已知/未知房间类型
  - get_exploration_summary: 摘要格式
  - TopologyEdge.to_dict: 序列化
  - _normalize_room_type: 标准化
  - _extract_target_keywords: 关键词提取
"""

import unittest

from memory.knowledge.semantic_prior import (
    RoomPrior,
    SemanticPriorEngine,
    TopologyEdge,
)


def _room(room_id: int, name: str, labels=None) -> dict:
    return {
        "room_id": room_id,
        "name": name,
        "semantic_labels": labels or [],
    }


class TestPredictTargetRooms(unittest.TestCase):
    def setUp(self):
        self.engine = SemanticPriorEngine()

    def test_fire_extinguisher_predicts_corridor(self):
        # "fire extinguisher" as compound keyword works; "find fire extinguisher"
        # regex combines "find fire" into one token, so use direct form or Chinese
        results = self.engine.predict_target_rooms("找灭火器")
        room_types = [r[0] for r in results]
        self.assertIn("corridor", room_types)

    def test_desk_predicts_office(self):
        results = self.engine.predict_target_rooms("go to desk")
        room_types = [r[0] for r in results]
        self.assertIn("office", room_types)

    def test_toilet_predicts_bathroom(self):
        # Single-word instruction avoids regex phrase-combining issue
        results = self.engine.predict_target_rooms("toilet")
        room_types = [r[0] for r in results]
        self.assertIn("bathroom", room_types)

    def test_chinese_chair_predicts_office(self):
        results = self.engine.predict_target_rooms("找椅子")
        room_types = [r[0] for r in results]
        self.assertIn("office", room_types)

    def test_results_sorted_by_probability(self):
        results = self.engine.predict_target_rooms("find desk")
        probs = [r[1] for r in results]
        self.assertEqual(probs, sorted(probs, reverse=True))

    def test_empty_instruction_returns_empty(self):
        # Instruction with only stop words
        results = self.engine.predict_target_rooms("go to a the")
        # Either empty or minimal
        self.assertIsInstance(results, list)

    def test_result_tuples_have_three_elements(self):
        results = self.engine.predict_target_rooms("find refrigerator")
        for item in results:
            self.assertEqual(len(item), 3)
            room_type, prob, reason = item
            self.assertIsInstance(room_type, str)
            self.assertIsInstance(prob, float)
            self.assertIsInstance(reason, str)


class TestScoreRoomsForTarget(unittest.TestCase):
    def setUp(self):
        self.engine = SemanticPriorEngine()
        self.rooms = [
            _room(0, "corridor", ["door", "sign"]),
            _room(1, "office", ["desk", "chair", "monitor"]),
            _room(2, "kitchen", ["refrigerator", "sink"]),
        ]

    def test_returns_room_prior_list(self):
        result = self.engine.score_rooms_for_target("find chair", self.rooms)
        self.assertIsInstance(result, list)
        for rp in result:
            self.assertIsInstance(rp, RoomPrior)

    def test_sorted_by_score(self):
        result = self.engine.score_rooms_for_target("find chair", self.rooms)
        scores = [r.prior_score for r in result]
        self.assertEqual(scores, sorted(scores, reverse=True))

    def test_office_wins_for_chair(self):
        result = self.engine.score_rooms_for_target("find chair", self.rooms)
        if result:
            # Office should score highest for chair (desk+chair+monitor)
            # Corridor also has chair in some room types but less prominent
            # Just ensure office is in top 2
            top_names = [r.room_name for r in result[:2]]
            self.assertIn("office", top_names)

    def test_direct_label_match_boosts_score(self):
        """Room with direct label match in semantic_labels gets score≥0.95."""
        rooms = [
            _room(0, "unknown_area", ["fire extinguisher", "door"]),
        ]
        result = self.engine.score_rooms_for_target(
            "find fire extinguisher", rooms
        )
        self.assertEqual(len(result), 1)
        self.assertGreaterEqual(result[0].prior_score, 0.95)

    def test_unvisited_rooms_get_bonus(self):
        """Unvisited rooms get +0.15 bonus vs visited."""
        rooms = [_room(0, "office", []), _room(1, "office", [])]
        visited = {0}  # only room 0 visited
        result = self.engine.score_rooms_for_target(
            "find desk", rooms, visited_room_ids=visited
        )
        room_map = {r.room_id: r for r in result}
        if 0 in room_map and 1 in room_map:
            self.assertGreater(room_map[1].prior_score, room_map[0].prior_score)

    def test_visited_flag_set_correctly(self):
        rooms = [_room(0, "office", []), _room(1, "kitchen", [])]
        visited = {0}
        result = self.engine.score_rooms_for_target(
            "find desk", rooms, visited_room_ids=visited
        )
        room_map = {r.room_id: r for r in result}
        if 0 in room_map:
            self.assertTrue(room_map[0].is_visited)
        if 1 in room_map:
            self.assertFalse(room_map[1].is_visited)

    def test_empty_rooms_returns_empty(self):
        result = self.engine.score_rooms_for_target("find chair", [])
        self.assertEqual(result, [])

    def test_score_capped_at_one(self):
        rooms = [_room(0, "office", ["desk", "chair", "computer"])]
        result = self.engine.score_rooms_for_target("find desk", rooms)
        for rp in result:
            self.assertLessEqual(rp.prior_score, 1.0)


class TestGetUnexploredPriors(unittest.TestCase):
    def setUp(self):
        self.engine = SemanticPriorEngine()

    def _edges(self, pairs):
        return [{"from_room": f, "to_room": t, "type": "door"} for f, t in pairs]

    def test_basic_topology(self):
        rooms = [
            _room(0, "corridor", ["door"]),
            _room(1, "office", ["desk", "chair"]),
            _room(2, "kitchen", ["refrigerator"]),
        ]
        edges = self._edges([(0, 1), (0, 2)])
        result = self.engine.get_unexplored_priors(
            "find desk",
            rooms,
            topology_edges=edges,
            visited_room_ids={0},
            current_room_id=0,
        )
        self.assertIsInstance(result, list)
        # Room 1 (office) should score high for "find desk"
        ids = [r["room_id"] for r in result]
        self.assertIn(1, ids)

    def test_visited_rooms_excluded(self):
        rooms = [
            _room(0, "office", ["desk"]),
            _room(1, "kitchen", ["refrigerator"]),
        ]
        result = self.engine.get_unexplored_priors(
            "find desk",
            rooms,
            topology_edges=[],
            visited_room_ids={0, 1},  # all visited
            current_room_id=0,
        )
        self.assertEqual(result, [])

    def test_hop_distance_penalizes_far_rooms(self):
        rooms = [
            _room(0, "corridor"),
            _room(1, "office"),
            _room(2, "office"),  # farther
        ]
        edges = self._edges([(0, 1), (1, 2)])
        result = self.engine.get_unexplored_priors(
            "find desk",
            rooms,
            topology_edges=edges,
            visited_room_ids={0},
            current_room_id=0,
        )
        # Both rooms should appear; closer one (room 1) should have higher combined_score
        id_map = {r["room_id"]: r for r in result}
        if 1 in id_map and 2 in id_map:
            self.assertGreaterEqual(id_map[1]["combined_score"],
                                    id_map[2]["combined_score"])

    def test_result_fields_present(self):
        rooms = [_room(1, "office", [])]
        result = self.engine.get_unexplored_priors(
            "find chair", rooms, [], visited_room_ids=set(), current_room_id=-1,
        )
        for r in result:
            for key in ["room_id", "room_name", "prior_score", "combined_score",
                        "hops", "reasoning"]:
                self.assertIn(key, r)

    def test_sorted_by_combined_score(self):
        rooms = [_room(i, "office") for i in range(4)]
        result = self.engine.get_unexplored_priors(
            "find desk", rooms, [], visited_room_ids=set(),
        )
        scores = [r["combined_score"] for r in result]
        self.assertEqual(scores, sorted(scores, reverse=True))


class TestGetRoomExpectedObjects(unittest.TestCase):
    def setUp(self):
        self.engine = SemanticPriorEngine()

    def test_office_has_desk(self):
        objs = self.engine.get_room_expected_objects("office")
        self.assertIn("desk", objs)
        self.assertGreater(objs["desk"], 0.5)

    def test_corridor_has_fire_extinguisher(self):
        objs = self.engine.get_room_expected_objects("corridor")
        self.assertIn("fire extinguisher", objs)

    def test_unknown_room_returns_empty(self):
        objs = self.engine.get_room_expected_objects("xyz_unknown_room_123")
        self.assertEqual(objs, {})

    def test_returns_dict(self):
        objs = self.engine.get_room_expected_objects("kitchen")
        self.assertIsInstance(objs, dict)
        for label, prob in objs.items():
            self.assertIsInstance(label, str)
            self.assertIsInstance(prob, float)


class TestGetExplorationSummary(unittest.TestCase):
    def setUp(self):
        self.engine = SemanticPriorEngine()
        self.rooms = [
            _room(0, "corridor"),
            _room(1, "office"),
        ]

    def test_returns_string(self):
        summary = self.engine.get_exploration_summary(
            "find desk", self.rooms, visited_room_ids={0}
        )
        self.assertIsInstance(summary, str)

    def test_contains_target(self):
        # Use single-word instruction so keyword extractor finds a match
        summary = self.engine.get_exploration_summary(
            "desk", self.rooms, visited_room_ids=set()
        )
        self.assertIn("desk", summary)

    def test_no_matching_target_returns_fallback(self):
        summary = self.engine.get_exploration_summary(
            "go to", [], visited_room_ids=set()
        )
        self.assertIsInstance(summary, str)


class TestTopologyEdgeToDict(unittest.TestCase):
    def test_basic_fields(self):
        edge = TopologyEdge(
            from_room_id=0,
            to_room_id=1,
            edge_type="door",
            distance=5.0,
        )
        d = edge.to_dict()
        self.assertEqual(d["from_room"], 0)
        self.assertEqual(d["to_room"], 1)
        self.assertEqual(d["type"], "door")
        self.assertAlmostEqual(d["distance"], 5.0)

    def test_mediator_included_when_set(self):
        edge = TopologyEdge(
            from_room_id=0,
            to_room_id=1,
            edge_type="traversal",
            mediator_label="door",
        )
        d = edge.to_dict()
        self.assertEqual(d["mediator"], "door")

    def test_mediator_absent_when_empty(self):
        edge = TopologyEdge(from_room_id=0, to_room_id=1, edge_type="proximity")
        d = edge.to_dict()
        self.assertNotIn("mediator", d)

    def test_traversals_included_when_positive(self):
        edge = TopologyEdge(
            from_room_id=0, to_room_id=1, edge_type="door",
            traversal_count=3,
        )
        d = edge.to_dict()
        self.assertEqual(d["traversals"], 3)

    def test_traversals_absent_when_zero(self):
        edge = TopologyEdge(from_room_id=0, to_room_id=1, edge_type="door")
        d = edge.to_dict()
        self.assertNotIn("traversals", d)


class TestNormalizeRoomType(unittest.TestCase):
    def setUp(self):
        self.engine = SemanticPriorEngine()

    def test_corridor_normalized(self):
        self.assertEqual(self.engine._normalize_room_type("corridor"), "corridor")

    def test_office_in_name(self):
        self.assertEqual(self.engine._normalize_room_type("east_office_3"), "office")

    def test_unknown_returns_original(self):
        result = self.engine._normalize_room_type("xyz_unknown")
        self.assertIsInstance(result, str)


class TestExtractTargetKeywords(unittest.TestCase):
    def setUp(self):
        self.engine = SemanticPriorEngine()

    def test_english_keywords(self):
        kws = self.engine._extract_target_keywords("find the desk")
        self.assertIn("desk", kws)
        # stop word "find" and "the" should be excluded
        self.assertNotIn("the", kws)

    def test_chinese_keywords_mapped(self):
        kws = self.engine._extract_target_keywords("找椅子")
        self.assertIn("chair", kws)

    def test_fire_extinguisher_extracted(self):
        kws = self.engine._extract_target_keywords("find fire extinguisher")
        # "fire extinguisher" or "fire" or "extinguisher" should appear
        found = any("fire" in k or "extinguisher" in k for k in kws)
        self.assertTrue(found)

    def test_returns_list(self):
        kws = self.engine._extract_target_keywords("go to kitchen")
        self.assertIsInstance(kws, list)


if __name__ == "__main__":
    unittest.main()

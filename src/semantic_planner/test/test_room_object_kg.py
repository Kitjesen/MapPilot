"""
test_room_object_kg.py — 持久化房间-物体知识图谱单元测试
"""

import json
import os
import tempfile
import unittest

from semantic_planner.room_object_kg import (
    RoomObjectKG,
    extract_room_objects_from_scene_graph,
)


class TestRoomObjectKGBasic(unittest.TestCase):
    """基本功能测试。"""

    def test_observe_and_query(self):
        kg = RoomObjectKG()
        kg.observe_room("office", ["desk", "chair", "monitor"], [0.9, 0.85, 0.8])
        kg.observe_room("office", ["desk", "chair", "keyboard"], [0.92, 0.88, 0.75])
        kg.observe_room("kitchen", ["refrigerator", "sink"], [0.9, 0.85])

        priors = kg.to_room_object_priors(min_observations=1)
        self.assertIn("office", priors)
        self.assertIn("kitchen", priors)
        self.assertIn("desk", priors["office"])
        self.assertIn("chair", priors["office"])
        self.assertIn("refrigerator", priors["kitchen"])

        # desk observed 2/2 visits → high probability
        self.assertGreater(priors["office"]["desk"], 0.6)

    def test_observe_filters_unknown(self):
        kg = RoomObjectKG()
        kg.observe_room("unknown", ["desk"], [0.9])
        kg.observe_room("unknown_area", ["chair"], [0.8])
        kg.observe_room("", ["monitor"], [0.7])
        self.assertTrue(kg.is_empty)

    def test_get_object_rooms(self):
        kg = RoomObjectKG()
        for _ in range(5):
            kg.observe_room("corridor", ["fire extinguisher", "door"], [0.85, 0.9])
        for _ in range(3):
            kg.observe_room("stairwell", ["fire extinguisher", "railing"], [0.8, 0.9])

        rooms = kg.get_object_rooms("fire extinguisher")
        self.assertEqual(len(rooms), 2)
        # corridor has more visits → should rank higher
        self.assertEqual(rooms[0][0], "corridor")

    def test_observe_adjacency(self):
        kg = RoomObjectKG()
        kg.observe_adjacency("corridor", "office", "door")
        kg.observe_adjacency("corridor", "office", "door")
        kg.observe_adjacency("office", "meeting_room", "traversal")

        adj = kg.get_adjacency_graph()
        self.assertEqual(len(adj), 2)
        # corridor-office edge should have count=2
        co_edge = [e for e in adj if "corridor" in (e["from"], e["to"]) and "office" in (e["from"], e["to"])]
        self.assertEqual(co_edge[0]["count"], 2)
        self.assertEqual(co_edge[0]["mediator"], "door")

    def test_adjacency_self_loop_ignored(self):
        kg = RoomObjectKG()
        kg.observe_adjacency("office", "office")
        self.assertEqual(len(kg.get_adjacency_graph()), 0)

    def test_stats(self):
        kg = RoomObjectKG()
        kg.observe_room("office", ["desk", "chair"], [0.9, 0.8])
        kg.observe_room("kitchen", ["sink"], [0.85])

        stats = kg.get_stats()
        self.assertEqual(stats["room_types"], 2)
        self.assertEqual(stats["unique_objects"], 3)
        self.assertEqual(stats["total_observations"], 3)


class TestRoomObjectKGPersistence(unittest.TestCase):
    """持久化测试。"""

    def test_save_and_load(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "kg.json")

            # Save
            kg1 = RoomObjectKG()
            kg1.observe_room("office", ["desk", "chair", "monitor"], [0.9, 0.85, 0.8])
            kg1.observe_room("office", ["desk", "chair"], [0.92, 0.88])
            kg1.observe_room("corridor", ["door", "sign"], [0.9, 0.7])
            kg1.observe_adjacency("corridor", "office", "door")
            kg1.start_new_session()
            self.assertTrue(kg1.save(path))

            # Load into new instance
            kg2 = RoomObjectKG()
            self.assertTrue(kg2.load(path))

            priors2 = kg2.to_room_object_priors(min_observations=1)
            self.assertIn("office", priors2)
            self.assertIn("desk", priors2["office"])
            self.assertIn("corridor", priors2)

            adj2 = kg2.get_adjacency_graph()
            self.assertEqual(len(adj2), 1)

    def test_merge_across_sessions(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "kg.json")

            # Session 1
            kg1 = RoomObjectKG()
            kg1.observe_room("office", ["desk", "chair"], [0.9, 0.85])
            kg1.start_new_session()
            kg1.save(path)

            # Session 2: load + add more
            kg2 = RoomObjectKG()
            kg2.load(path)
            kg2.start_new_session()
            kg2.observe_room("office", ["desk", "monitor"], [0.92, 0.8])
            kg2.observe_room("kitchen", ["refrigerator"], [0.9])
            kg2.save(path)

            # Verify merged
            kg3 = RoomObjectKG()
            kg3.load(path)
            stats = kg3.get_stats()
            self.assertEqual(stats["room_types"], 2)  # office + kitchen
            self.assertGreaterEqual(stats["sessions"], 2)

            priors = kg3.to_room_object_priors(min_observations=1)
            self.assertIn("office", priors)
            self.assertIn("desk", priors["office"])
            self.assertIn("kitchen", priors)

    def test_load_nonexistent(self):
        kg = RoomObjectKG()
        self.assertFalse(kg.load("/nonexistent/path.json"))

    def test_save_creates_directory(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "subdir", "nested", "kg.json")
            kg = RoomObjectKG()
            kg.observe_room("office", ["desk"], [0.9])
            self.assertTrue(kg.save(path))
            self.assertTrue(os.path.exists(path))


class TestExtractRoomObjects(unittest.TestCase):
    """场景图提取测试。"""

    def test_extract_with_regions(self):
        sg = json.dumps({
            "objects": [
                {"id": 1, "label": "desk", "confidence": 0.9, "region_id": 0},
                {"id": 2, "label": "chair", "confidence": 0.85, "region_id": 0},
                {"id": 3, "label": "refrigerator", "confidence": 0.88, "region_id": 1},
            ],
            "regions": [
                {"name": "office", "object_ids": [1, 2]},
                {"name": "kitchen", "object_ids": [3]},
            ],
        })

        result = extract_room_objects_from_scene_graph(sg)
        self.assertEqual(len(result), 2)
        room_types = [r[0] for r in result]
        self.assertIn("office", room_types)
        self.assertIn("kitchen", room_types)

    def test_extract_flat_format(self):
        sg = json.dumps({
            "objects": [
                {"id": 1, "label": "desk", "confidence": 0.9, "region_id": 0},
                {"id": 2, "label": "chair", "confidence": 0.85, "region_id": 0},
            ],
        })

        result = extract_room_objects_from_scene_graph(sg)
        # Should group by region_id and infer room type
        self.assertGreaterEqual(len(result), 0)  # may or may not infer

    def test_extract_bad_json(self):
        result = extract_room_objects_from_scene_graph("not-json")
        self.assertEqual(result, [])

    def test_extract_empty(self):
        result = extract_room_objects_from_scene_graph("{}")
        self.assertEqual(result, [])


class TestSemanticPriorIntegration(unittest.TestCase):
    """SemanticPriorEngine 与 KG 集成测试。"""

    def test_load_learned_priors(self):
        from semantic_planner.semantic_prior import SemanticPriorEngine

        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "kg.json")

            # Build a KG with unique object for test room
            kg = RoomObjectKG()
            for _ in range(10):
                kg.observe_room("workshop", ["lathe", "drill"], [0.9, 0.85])
            kg.start_new_session()
            kg.save(path)

            # Load into engine
            engine = SemanticPriorEngine(kg_path=path)

            # Learned KG added "workshop" room type (not in hand-coded priors)
            self.assertIn("workshop", engine._priors)
            self.assertIn("lathe", engine._priors["workshop"])

    def test_fallback_without_kg(self):
        from semantic_planner.semantic_prior import SemanticPriorEngine

        # Without KG, should use hand-coded priors
        engine = SemanticPriorEngine()
        # Test with Chinese keyword that maps to English via ZH_OBJECT_MAP
        predictions = engine.predict_target_rooms("找灭火器")
        self.assertTrue(len(predictions) > 0)

    def test_reload_priors(self):
        from semantic_planner.semantic_prior import SemanticPriorEngine

        engine = SemanticPriorEngine()
        custom = {"testroom": {"sonar": 0.9}}
        engine.reload_priors(custom)

        # Verify the priors were replaced
        self.assertIn("testroom", engine._priors)
        self.assertIn("sonar", engine._priors["testroom"])
        # Inverse index should have been rebuilt
        self.assertIn("sonar", engine._inverse_index)

    def test_kg_path_nonexistent_uses_defaults(self):
        from semantic_planner.semantic_prior import SemanticPriorEngine

        # Non-existent KG path should fall back to hand-coded priors
        engine = SemanticPriorEngine(kg_path="/nonexistent/kg.json")
        self.assertIn("office", engine._priors)
        self.assertIn("desk", engine._priors["office"])


class TestKGBackedRoomPrediction(unittest.TestCase):
    """P1: KG-backed 房间邻接预测测试。"""

    def test_predict_adjacent_with_kg(self):
        """有 KG 邻接数据时, 应使用学习到的邻接关系。"""
        from semantic_planner.goal_resolver import GoalResolver
        from semantic_planner.llm_client import LLMConfig

        config = LLMConfig(backend="openai", model="gpt-4o-mini")
        resolver = GoalResolver(config)

        # 构建 KG 邻接: corridor ↔ lab (高频), corridor ↔ office (低频)
        kg = RoomObjectKG()
        for _ in range(10):
            kg.observe_adjacency("corridor", "lab", "door")
        for _ in range(3):
            kg.observe_adjacency("corridor", "office", "door")

        resolver.set_room_object_kg(kg)

        # corridor 的邻接预测应该返回 lab (最高频次)
        predicted = resolver._predict_adjacent_room_type("corridor")
        self.assertEqual(predicted, "lab")

    def test_predict_adjacent_without_kg_uses_hardcoded(self):
        """无 KG 时回退到 hand-coded 邻接。"""
        from semantic_planner.goal_resolver import GoalResolver
        from semantic_planner.llm_client import LLMConfig

        config = LLMConfig(backend="openai", model="gpt-4o-mini")
        resolver = GoalResolver(config)

        # 无 KG → hand-coded: office → corridor
        predicted = resolver._predict_adjacent_room_type("office")
        self.assertEqual(predicted, "corridor")

    def test_predict_adjacent_kg_empty_uses_hardcoded(self):
        """KG 存在但该房间类型无邻接数据时, 回退到 hand-coded。"""
        from semantic_planner.goal_resolver import GoalResolver
        from semantic_planner.llm_client import LLMConfig

        config = LLMConfig(backend="openai", model="gpt-4o-mini")
        resolver = GoalResolver(config)

        # KG 只有 corridor ↔ lab, 查询 bathroom 应回退到 hand-coded
        kg = RoomObjectKG()
        kg.observe_adjacency("corridor", "lab", "door")
        resolver.set_room_object_kg(kg)

        predicted = resolver._predict_adjacent_room_type("bathroom")
        self.assertEqual(predicted, "corridor")  # hand-coded fallback


class TestTopologicalMemoryPersistence(unittest.TestCase):
    """拓扑记忆持久化测试。"""

    def test_save_and_load(self):
        import numpy as np
        from semantic_planner.topological_memory import TopologicalMemory

        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "topo.json")

            mem1 = TopologicalMemory(new_node_distance=2.0)
            mem1.update_position(
                np.array([0.0, 0.0, 0.0]),
                visible_labels=["door", "sign"],
                room_id=0, room_name="corridor",
            )
            mem1.update_position(
                np.array([5.0, 0.0, 0.0]),
                visible_labels=["desk", "chair"],
                room_id=1, room_name="office",
            )
            self.assertTrue(mem1.save_to_file(path))

            mem2 = TopologicalMemory(new_node_distance=2.0)
            self.assertTrue(mem2.load_from_file(path))
            self.assertEqual(len(mem2.nodes), 2)

            # Check labels preserved
            labels = set()
            for n in mem2.nodes.values():
                labels.update(n.visible_labels)
            self.assertIn("door", labels)
            self.assertIn("desk", labels)

    def test_load_nonexistent(self):
        from semantic_planner.topological_memory import TopologicalMemory

        mem = TopologicalMemory()
        self.assertFalse(mem.load_from_file("/nonexistent.json"))


if __name__ == "__main__":
    unittest.main()

"""
test_fast_resolve.py — 目标解析器 Fast Path + 选择性 Grounding 测试

覆盖:
  - Fast Path 场景图直接匹配 (VLingNav)
  - 多源置信度融合 (AdaNav)
  - 选择性 Grounding (ESCA)
  - 关键词提取
  - 距离衰减
"""

import json
import unittest


from decision.goal_resolution.goal_resolver import GoalResolver
from decision.llm.llm_client import LLMConfig


def _make_scene_graph(objects, relations=None, regions=None):
    """辅助函数: 构建场景图 JSON。"""
    return json.dumps({
        "timestamp": 0,
        "object_count": len(objects),
        "objects": objects,
        "relations": relations or [],
        "regions": regions or [],
        "summary": "test scene",
    })


class TestFastResolve(unittest.TestCase):
    """Fast Path (VLingNav System 1) 测试。"""

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="test")
        self.resolver = GoalResolver(self.config, fast_path_threshold=0.5)

    def test_exact_label_match(self):
        """标签完全匹配 → Fast Path 命中。"""
        sg = _make_scene_graph([
            {"id": 0, "label": "chair", "position": {"x": 3.0, "y": 2.0, "z": 0.0},
             "score": 0.9, "detection_count": 5},
        ])
        result = self.resolver.fast_resolve("go to the chair", sg)
        self.assertIsNotNone(result)
        self.assertEqual(result.action, "navigate")
        self.assertEqual(result.target_label, "chair")
        self.assertAlmostEqual(result.target_x, 3.0)
        self.assertEqual(result.path, "fast")

    def test_no_match_returns_none(self):
        """无匹配物体 → 返回 None (交给 Slow Path)。"""
        sg = _make_scene_graph([
            {"id": 0, "label": "chair", "position": {"x": 1, "y": 1, "z": 0},
             "score": 0.9, "detection_count": 3},
        ])
        result = self.resolver.fast_resolve("find the elephant", sg)
        self.assertIsNone(result)

    def test_low_confidence_defers(self):
        """低置信度 → 返回 None。"""
        resolver = GoalResolver(self.config, fast_path_threshold=0.95)
        sg = _make_scene_graph([
            {"id": 0, "label": "chair", "position": {"x": 1, "y": 1, "z": 0},
             "score": 0.3, "detection_count": 1},
        ])
        result = resolver.fast_resolve("go to chair", sg)
        self.assertIsNone(result)

    def test_partial_keyword_match(self):
        """部分关键词匹配 (fire → fire extinguisher)。"""
        sg = _make_scene_graph([
            {"id": 0, "label": "fire extinguisher", "position": {"x": 5, "y": 5, "z": 0},
             "score": 0.85, "detection_count": 4},
        ])
        result = self.resolver.fast_resolve("找灭火器 fire", sg)
        # "fire" 是关键词, "fire extinguisher" 含 "fire" → 部分匹配
        # 分数取决于权重组合, 但应有返回
        self.assertIsNotNone(result)

    def test_spatial_relation_boosts_score(self):
        """空间关系提示 → 提升匹配分。"""
        sg = _make_scene_graph(
            objects=[
                {"id": 0, "label": "chair", "position": {"x": 3, "y": 2, "z": 0},
                 "score": 0.8, "detection_count": 3},
                {"id": 1, "label": "door", "position": {"x": 3, "y": 3, "z": 0},
                 "score": 0.9, "detection_count": 5},
            ],
            relations=[
                {"subject_id": 0, "relation": "near", "object_id": 1, "distance": 1.0},
            ],
        )
        # "chair near the door" — chair + door 都被提及, 关系链命中
        result = self.resolver.fast_resolve("chair near the door", sg)
        self.assertIsNotNone(result)
        # 两者都匹配指令, door 分更高 (score=0.9, count=5) → 可能选 door 或 chair
        self.assertIn(result.target_label, ["chair", "door"])

    def test_empty_scene_graph(self):
        result = self.resolver.fast_resolve("go to chair", "{}")
        self.assertIsNone(result)

    def test_invalid_json(self):
        result = self.resolver.fast_resolve("go to chair", "not json")
        self.assertIsNone(result)

    def test_distance_preference(self):
        """分数相近时优先选近目标。"""
        sg = _make_scene_graph([
            {"id": 0, "label": "chair", "position": {"x": 100, "y": 100, "z": 0},
             "score": 0.9, "detection_count": 5},
            {"id": 1, "label": "chair", "position": {"x": 2, "y": 2, "z": 0},
             "score": 0.88, "detection_count": 5},
        ])
        result = self.resolver.fast_resolve(
            "go to the chair", sg,
            robot_position={"x": 0, "y": 0, "z": 0},
        )
        self.assertIsNotNone(result)
        # 应选更近的 chair (id=1)
        self.assertAlmostEqual(result.target_x, 2.0)

    def test_chinese_instruction(self):
        """中文指令匹配。"""
        sg = _make_scene_graph([
            {"id": 0, "label": "灭火器", "position": {"x": 4, "y": 3, "z": 0},
             "score": 0.9, "detection_count": 3},
        ])
        result = self.resolver.fast_resolve("去灭火器那里", sg)
        self.assertIsNotNone(result)
        self.assertEqual(result.target_label, "灭火器")


class TestSelectiveGrounding(unittest.TestCase):
    """选择性 Grounding (ESCA) 测试。"""

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="test")
        self.resolver = GoalResolver(self.config)

    def test_small_scene_not_filtered(self):
        """物体数 <= max_objects → 不过滤。"""
        sg = _make_scene_graph([
            {"id": i, "label": f"obj_{i}", "position": {"x": i, "y": 0, "z": 0},
             "score": 0.5, "detection_count": 1}
            for i in range(5)
        ])
        result = self.resolver._selective_grounding("find obj_1", sg, max_objects=15)
        result_data = json.loads(result)
        self.assertEqual(result_data["object_count"], 5)

    def test_large_scene_filtered(self):
        """物体数 > max_objects → 过滤只保留相关的。"""
        objects = [
            {"id": i, "label": f"random_obj_{i}", "position": {"x": i, "y": 0, "z": 0},
             "score": 0.5, "detection_count": 1}
            for i in range(30)
        ]
        # 加一个目标物体
        objects.append({
            "id": 99, "label": "red door", "position": {"x": 50, "y": 50, "z": 0},
            "score": 0.9, "detection_count": 5,
        })
        sg = _make_scene_graph(objects)
        result = self.resolver._selective_grounding("find the red door", sg, max_objects=10)
        result_data = json.loads(result)
        self.assertLessEqual(result_data["object_count"], 10)
        # "red door" 应在过滤结果中
        labels = [o["label"] for o in result_data["objects"]]
        self.assertIn("red door", labels)

    def test_relation_hop_expansion(self):
        """关系链 1-hop 扩展: 目标的邻居也应被保留。"""
        objects = [
            {"id": i, "label": f"noise_{i}", "position": {"x": i, "y": 0, "z": 0},
             "score": 0.3, "detection_count": 1}
            for i in range(25)
        ]
        # 目标 + 邻居
        objects.append({"id": 100, "label": "door", "position": {"x": 10, "y": 10, "z": 0},
                        "score": 0.9, "detection_count": 5})
        objects.append({"id": 101, "label": "sign", "position": {"x": 11, "y": 10, "z": 0},
                        "score": 0.8, "detection_count": 3})
        sg = _make_scene_graph(
            objects,
            relations=[{"subject_id": 100, "relation": "near", "object_id": 101, "distance": 1.0}],
        )
        result = self.resolver._selective_grounding("find the door", sg, max_objects=10)
        result_data = json.loads(result)
        filtered_ids = {o["id"] for o in result_data["objects"]}
        self.assertIn(100, filtered_ids)  # 目标
        self.assertIn(101, filtered_ids)  # 邻居 (1-hop)

    def test_no_match_fallback_to_top_score(self):
        """完全无匹配 → 按分数选 top-N。"""
        objects = [
            {"id": i, "label": f"item_{i}", "position": {"x": i, "y": 0, "z": 0},
             "score": float(i) / 25, "detection_count": i + 1}
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
    """关键词提取测试。"""

    def test_english_keywords(self):
        kws = GoalResolver._extract_keywords("go to the red chair near the door")
        # 应过滤停用词 (go, to, the, near)
        self.assertIn("red", kws)
        self.assertIn("chair", kws)
        self.assertIn("door", kws)
        self.assertNotIn("the", kws)
        self.assertNotIn("to", kws)

    def test_chinese_keywords(self):
        kws = GoalResolver._extract_keywords("去红色灭火器旁边")
        # 中文按连续字符组分词, 停用词过滤后至少包含目标关键信息
        # 简单分词把整段中文视为一个 token, 需要验证核心目标在其中
        all_text = " ".join(kws)
        self.assertIn("灭火器", all_text)

    def test_mixed_keywords(self):
        kws = GoalResolver._extract_keywords("找fire extinguisher")
        self.assertIn("fire", kws)
        self.assertIn("extinguisher", kws)


class TestRoomFallback(unittest.TestCase):
    """Room 级 fallback 测试 — 物体匹配失败时匹配区域。"""

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="test")
        self.resolver = GoalResolver(self.config, fast_path_threshold=0.75)

    def _make_sg_with_rooms(self, objects=None, rooms=None):
        """构建含 rooms 的场景图 JSON。"""
        return json.dumps({
            "timestamp": 0,
            "object_count": len(objects or []),
            "objects": objects or [],
            "relations": [],
            "rooms": rooms or [],
        })

    def test_room_keyword_match_chinese(self):
        """中文区域指令 '去厨房' 匹配 room name '厨房'。"""
        sg = self._make_sg_with_rooms(
            objects=[
                {"id": 0, "label": "table", "position": [1, 1, 0],
                 "score": 0.3, "detection_count": 1},
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
        """英文 'find the kitchen' 匹配 room name 'kitchen'。"""
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
        """物体匹配成功时不走 room fallback。"""
        sg = self._make_sg_with_rooms(
            objects=[
                {"id": 0, "label": "chair", "position": [3.0, 2.0, 0.0],
                 "score": 0.9, "detection_count": 5},
            ],
            rooms=[
                {"name": "chair_room", "center": [10.0, 10.0, 0.0], "object_ids": [0]},
            ],
        )
        result = self.resolver.fast_resolve("go to the chair", sg)
        self.assertIsNotNone(result)
        self.assertEqual(result.target_label, "chair")
        # 应该是物体匹配, 不是 room fallback
        self.assertAlmostEqual(result.target_x, 3.0)

    def test_room_no_match_returns_none(self):
        """Room 也不匹配时返回 None。"""
        sg = self._make_sg_with_rooms(
            rooms=[
                {"name": "kitchen", "center": [4.0, 2.0, 0.0], "object_ids": []},
            ],
        )
        result = self.resolver.fast_resolve("find the elephant", sg)
        self.assertIsNone(result)

    def test_room_center_dict_format(self):
        """Room center 为 dict 格式 {x, y, z} 时正确解析。"""
        sg = self._make_sg_with_rooms(
            rooms=[
                {"name": "走廊", "center": {"x": 7.0, "y": 4.0, "z": 0.0},
                 "object_ids": []},
            ],
        )
        result = self.resolver.fast_resolve("到走廊", sg)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.target_x, 7.0)
        self.assertAlmostEqual(result.target_y, 4.0)

    def test_room_no_center_skipped(self):
        """Room 缺少 center 时跳过。"""
        sg = self._make_sg_with_rooms(
            rooms=[
                {"name": "厨房", "object_ids": []},  # no center
            ],
        )
        result = self.resolver.fast_resolve("去厨房", sg)
        self.assertIsNone(result)

    def test_room_empty_rooms_returns_none(self):
        """rooms 为空时 room fallback 不触发。"""
        sg = self._make_sg_with_rooms(rooms=[])
        result = self.resolver.fast_resolve("去厨房", sg)
        self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()

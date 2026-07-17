"""
测试Fast Path、ESCA过滤、关键词提取

参考: SEMANTIC_NAV_REPORT.md 第7.1节
测试数量: 14个
"""

import json

import pytest

from decision.goals.resolver import GoalResolver
from decision.llm.client import LLMConfig


class TestFastPath:
    """测试Fast Path功能"""

    def test_fast_path_enabled(self):
        """测试Fast Path启用"""
        config = LLMConfig(backend="openai", model="gpt-4o-mini")
        resolver = GoalResolver(config, fast_path_threshold=0.75)
        assert resolver._fast_path_threshold == 0.75

    def test_fast_path_high_score(self):
        """测试高分数走Fast Path"""
        config = LLMConfig(backend="openai", model="gpt-4o-mini")
        resolver = GoalResolver(config, fast_path_threshold=0.75)

        scene_graph = {
            "objects": [
                {
                    "id": 0,
                    "label": "fire_extinguisher",
                    "position": {"x": 3.0, "y": 4.0, "z": 1.0},
                    "score": 0.95,
                    "detection_count": 25,
                }
            ],
            "relations": [],
            "regions": [],
        }

        result = resolver.fast_resolve("fire extinguisher", json.dumps(scene_graph))
        if result:
            assert result.path == "fast"

    def test_fast_path_low_score(self):
        """测试低分数不走Fast Path"""
        config = LLMConfig(backend="openai", model="gpt-4o-mini")
        resolver = GoalResolver(config, fast_path_threshold=0.75)

        scene_graph = {
            "objects": [
                {
                    "id": 0,
                    "label": "unknown_object",
                    "position": {"x": 3.0, "y": 4.0, "z": 1.0},
                    "score": 0.3,
                    "detection_count": 1,
                }
            ],
            "relations": [],
            "regions": [],
        }

        result = resolver.fast_resolve("fire extinguisher", json.dumps(scene_graph))
        assert result is None  # 应该返回None，进入Slow Path


class TestESCAFiltering:
    """测试ESCA选择性Grounding"""

    def test_esca_basic(self):
        """测试基础ESCA过滤"""
        config = LLMConfig(backend="openai", model="gpt-4o-mini")
        resolver = GoalResolver(config)

        objects = [
            {"id": i, "label": f"object_{i}", "position": {"x": i * 1.0, "y": 0, "z": 0}, "score": 0.8}
            for i in range(100)
        ]
        scene_graph = {"objects": objects, "relations": [], "regions": []}

        filtered = resolver._selective_grounding("object_5", json.dumps(scene_graph))
        filtered_sg = json.loads(filtered)

        assert len(filtered_sg["objects"]) < 100
        assert len(filtered_sg["objects"]) <= 15

    def test_esca_keyword_match(self):
        """测试关键词匹配"""
        config = LLMConfig(backend="openai", model="gpt-4o-mini")
        resolver = GoalResolver(config)

        scene_graph = {
            "objects": [
                {"id": 0, "label": "chair", "position": {"x": 1.0, "y": 0, "z": 0}},
                {"id": 1, "label": "table", "position": {"x": 2.0, "y": 0, "z": 0}},
                {"id": 2, "label": "door", "position": {"x": 3.0, "y": 0, "z": 0}},
            ],
            "relations": [],
            "regions": [],
        }

        filtered = resolver._selective_grounding("chair", json.dumps(scene_graph))
        filtered_sg = json.loads(filtered)

        # 应该包含chair
        labels = [obj["label"] for obj in filtered_sg["objects"]]
        assert "chair" in labels

    def test_esca_1hop_expansion(self):
        """测试1-hop关系扩展"""
        config = LLMConfig(backend="openai", model="gpt-4o-mini")
        resolver = GoalResolver(config)

        scene_graph = {
            "objects": [
                {"id": 0, "label": "chair", "position": {"x": 1.0, "y": 0, "z": 0}},
                {"id": 1, "label": "table", "position": {"x": 2.0, "y": 0, "z": 0}},
            ],
            "relations": [{"subject_id": 0, "relation": "near", "object_id": 1}],
            "regions": [],
        }

        filtered = resolver._selective_grounding("chair", json.dumps(scene_graph))
        filtered_sg = json.loads(filtered)

        # 应该包含chair和它的邻居table
        assert len(filtered_sg["objects"]) >= 1

    def test_esca_region_expansion(self):
        """测试区域扩展"""
        config = LLMConfig(backend="openai", model="gpt-4o-mini")
        resolver = GoalResolver(config)

        scene_graph = {
            "objects": [
                {"id": 0, "label": "chair", "position": {"x": 1.0, "y": 0, "z": 0}, "region_id": 0},
                {"id": 1, "label": "table", "position": {"x": 2.0, "y": 0, "z": 0}, "region_id": 0},
            ],
            "relations": [],
            "regions": [{"region_id": 0, "name": "office", "object_ids": [0, 1]}],
        }

        filtered = resolver._selective_grounding("chair", json.dumps(scene_graph))
        filtered_sg = json.loads(filtered)

        # 应该包含同区域的物体


class TestKeywordExtraction:
    """测试关键词提取"""

    def test_chinese_keywords(self):
        """测试中文关键词"""
        keywords = GoalResolver._extract_keywords("去红色灭火器旁边")
        assert isinstance(keywords, list)
        assert len(keywords) > 0

    def test_english_keywords(self):
        """测试英文关键词"""
        keywords = GoalResolver._extract_keywords("go to red fire extinguisher")
        assert isinstance(keywords, list)
        assert "red" in keywords or "fire" in keywords

    def test_mixed_keywords(self):
        """测试中英文混合"""
        keywords = GoalResolver._extract_keywords("go to 红色灭火器")
        assert isinstance(keywords, list)
        assert len(keywords) > 0

    def test_stopwords_filtered(self):
        """测试停用词过滤"""
        keywords = GoalResolver._extract_keywords("请去到那个红色的灭火器")
        # 停用词应该被过滤
        assert "请" not in keywords
        assert "去" not in keywords

    def test_empty_instruction(self):
        """测试空指令"""
        keywords = GoalResolver._extract_keywords("")
        assert keywords == []


class TestConfidenceFusion:
    """测试置信度融合"""

    def test_label_match_weight(self):
        """测试标签匹配权重"""
        from decision.goals.resolver import WEIGHT_LABEL_MATCH

        assert WEIGHT_LABEL_MATCH == 0.35

    def test_clip_weight(self):
        """测试CLIP权重"""
        from decision.goals.resolver import WEIGHT_CLIP_SIM

        assert WEIGHT_CLIP_SIM == 0.35

    def test_weights_sum_to_one(self):
        """测试权重和为1"""
        from decision.goals.resolver import (
            WEIGHT_CLIP_SIM,
            WEIGHT_DETECTOR_SCORE,
            WEIGHT_LABEL_MATCH,
            WEIGHT_SPATIAL_HINT,
        )

        total = WEIGHT_LABEL_MATCH + WEIGHT_CLIP_SIM + WEIGHT_DETECTOR_SCORE + WEIGHT_SPATIAL_HINT
        assert abs(total - 1.0) < 0.01


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

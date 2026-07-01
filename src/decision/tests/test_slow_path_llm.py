"""
test_slow_path_llm.py — Slow Path LLM推理测试

目标:
  1. 测试完整的Fast-Slow流程
  2. 验证Slow Path的LLM推理
  3. 测试ESCA过滤 + LLM的组合效果

注意:
  - 需要配置LLM API key
  - 可以使用mock LLM进行测试
"""

import asyncio
import json
import unittest
from unittest.mock import AsyncMock, MagicMock


from decision.goal_resolution.goal_resolver import GoalResolver
from decision.llm.llm_client import LLMConfig


def make_scene_graph(objects, relations=None):
    """构建场景图JSON"""
    return json.dumps({
        "timestamp": 0,
        "object_count": len(objects),
        "objects": objects,
        "relations": relations or [],
        "regions": [],
        "summary": "test scene",
    })


class TestSlowPathLLM(unittest.TestCase):
    """Slow Path LLM推理测试"""

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="gpt-4")
        self.resolver = GoalResolver(
            self.config,
            fast_path_threshold=0.95,  # 设置高阈值，强制走Slow Path
        )
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

    def tearDown(self):
        self.loop.close()

    def test_slow_path_with_mock_llm(self):
        """
        测试Slow Path流程（使用mock LLM）

        场景: Fast Path置信度不够，进入Slow Path
        """
        # 构建复杂场景：有多个相似物体
        objects = [
            {
                "id": 0,
                "label": "fire extinguisher",
                "position": {"x": 5, "y": 3, "z": 0.8},
                "score": 0.85,
                "detection_count": 5,
            },
            {
                "id": 1,
                "label": "red box",
                "position": {"x": 6, "y": 4, "z": 0.5},
                "score": 0.88,
                "detection_count": 6,
            },
            {
                "id": 2,
                "label": "red chair",
                "position": {"x": 3, "y": 2, "z": 0},
                "score": 0.82,
                "detection_count": 4,
            },
        ]

        # 添加大量噪音物体（触发ESCA过滤）
        for i in range(3, 50):
            objects.append({
                "id": i,
                "label": f"random_object_{i}",
                "position": {"x": i, "y": i % 10, "z": 0},
                "score": 0.5,
                "detection_count": 2,
            })

        sg = make_scene_graph(objects)
        instruction = "find red fire extinguisher"

        # Mock LLM响应
        mock_llm_response = json.dumps({
            "action": "navigate",
            "target": {"x": 5.0, "y": 3.0, "z": 0.8},
            "target_label": "fire extinguisher",
            "confidence": 0.92,
            "reasoning": "Found fire extinguisher matching 'red fire extinguisher' in filtered scene graph",
        })

        # 设置mock
        self.resolver._primary = MagicMock()
        self.resolver._primary.is_available.return_value = True
        self.resolver._primary.chat = AsyncMock(return_value=mock_llm_response)

        # 运行测试
        result = self.loop.run_until_complete(
            self.resolver.resolve(
                instruction=instruction,
                scene_graph_json=sg,
                robot_position={"x": 0, "y": 0, "z": 0},
                language="en",
            )
        )

        # 验证结果
        self.assertTrue(result.is_valid)
        self.assertEqual(result.action, "navigate")
        self.assertEqual(result.target_label, "fire extinguisher")
        self.assertEqual(result.path, "slow")  # 确认走了Slow Path
        self.assertAlmostEqual(result.target_x, 5.0)
        self.assertAlmostEqual(result.confidence, 0.92)

        # 验证LLM被调用了
        self.resolver._primary.chat.assert_called_once()

        # 获取发送给LLM的消息
        call_args = self.resolver._primary.chat.call_args
        call_args[0][0]

        print("\n=== Slow Path LLM Test ===")
        print(f"Instruction: {instruction}")
        print(f"Original objects: {len(objects)}")
        print(f"Result: {result.target_label} at ({result.target_x}, {result.target_y})")
        print(f"Path: {result.path}")
        print(f"Confidence: {result.confidence}")

    def test_esca_filtering_before_llm(self):
        """
        测试ESCA过滤是否在LLM调用前生效

        验证: 发送给LLM的场景图应该是过滤后的
        """
        # 构建大场景图（200个物体）
        objects = []
        for i in range(200):
            objects.append({
                "id": i,
                "label": f"object_{i}",
                "position": {"x": i, "y": i % 20, "z": 0},
                "score": 0.5,
                "detection_count": 2,
            })

        # 添加目标物体
        objects.append({
            "id": 999,
            "label": "target_door",
            "position": {"x": 50, "y": 50, "z": 0},
            "score": 0.9,
            "detection_count": 5,
        })

        sg = make_scene_graph(objects)
        instruction = "find the target door"

        # Mock LLM响应
        mock_llm_response = json.dumps({
            "action": "navigate",
            "target": {"x": 50.0, "y": 50.0, "z": 0.0},
            "target_label": "target_door",
            "confidence": 0.95,
            "reasoning": "Found target_door in filtered scene graph",
        })

        self.resolver._primary = MagicMock()
        self.resolver._primary.is_available.return_value = True
        self.resolver._primary.chat = AsyncMock(return_value=mock_llm_response)

        # 运行测试
        result = self.loop.run_until_complete(
            self.resolver.resolve(
                instruction=instruction,
                scene_graph_json=sg,
                language="en",
            )
        )

        # 验证LLM被调用
        self.resolver._primary.chat.assert_called_once()

        # 获取发送给LLM的消息
        call_args = self.resolver._primary.chat.call_args
        messages = call_args[0][0]

        # 提取场景图内容
        user_message = messages[1]["content"]

        # 验证场景图被过滤了
        # 原始201个物体，应该被过滤到15个左右
        self.assertIn("target_door", user_message)

        # 粗略验证：过滤后的场景图应该比原始的短很多
        # 原始场景图 ~2000 tokens，过滤后 ~300 tokens
        self.assertLess(len(user_message), 5000)  # 应该远小于完整场景图

        print("\n=== ESCA Filtering Test ===")
        print(f"Original objects: {len(objects)}")
        print(f"User message length: {len(user_message)} chars")
        print(f"Result: {result.target_label}")
        print(f"Path: {result.path}")

    def test_fast_to_slow_fallback(self):
        """
        测试Fast Path失败后自动切换到Slow Path

        场景:
        1. Fast Path因阈值设为 0.99 而无法满足（强制失败）
        2. 自动进入Slow Path
        3. LLM成功解析
        """
        objects = [
            {
                "id": 0,
                "label": "workstation",  # 不直接匹配指令关键词
                "position": {"x": 3, "y": 2, "z": 0},
                "score": 0.7,
                "detection_count": 3,
            },
            {
                "id": 1,
                "label": "monitor",
                "position": {"x": 3, "y": 3, "z": 0},
                "score": 0.8,
                "detection_count": 4,
            },
        ]

        sg = make_scene_graph(objects)
        instruction = "go to the place where I can code"

        # Mock LLM响应
        mock_llm_response = json.dumps({
            "action": "navigate",
            "target": {"x": 3.0, "y": 2.0, "z": 0.0},
            "target_label": "workstation",
            "confidence": 0.88,
            "reasoning": "A workstation with a monitor is where one can code",
        })

        self.resolver._primary = MagicMock()
        self.resolver._primary.is_available.return_value = True
        self.resolver._primary.chat = AsyncMock(return_value=mock_llm_response)

        # 设置极高阈值，强制 Fast Path 失败
        self.resolver._fast_path_threshold = 0.99

        # 运行测试
        result = self.loop.run_until_complete(
            self.resolver.resolve(
                instruction=instruction,
                scene_graph_json=sg,
                language="en",
            )
        )

        # 验证结果
        self.assertTrue(result.is_valid)
        self.assertEqual(result.path, "slow")  # 应该走了Slow Path
        self.assertEqual(result.target_label, "workstation")

        print("\n=== Fast-to-Slow Fallback Test ===")
        print(f"Instruction: {instruction}")
        print("Fast Path: Failed (confidence < 0.75)")
        print("Slow Path: Success")
        print(f"Result: {result.target_label}")
        print(f"Confidence: {result.confidence}")


class TestSlowPathPrompt(unittest.TestCase):
    """测试Slow Path的Prompt构建"""

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="gpt-4")
        self.resolver = GoalResolver(self.config)

    def test_prompt_contains_filtered_scene_graph(self):
        """验证Prompt包含过滤后的场景图"""
        # 构建大场景图
        objects = [{"id": i, "label": f"obj_{i}", "position": {"x": i, "y": 0, "z": 0},
                    "score": 0.5, "detection_count": 1} for i in range(100)]
        objects.append({"id": 999, "label": "target", "position": {"x": 50, "y": 50, "z": 0},
                       "score": 0.9, "detection_count": 5})

        sg = make_scene_graph(objects)
        instruction = "find target"

        # 测试ESCA过滤
        filtered_sg = self.resolver._selective_grounding(instruction, sg, max_objects=15)
        filtered_data = json.loads(filtered_sg)

        # 验证过滤效果
        self.assertLessEqual(filtered_data["object_count"], 15)
        self.assertIn("target", [o["label"] for o in filtered_data["objects"]])

        print("\n=== Prompt Construction Test ===")
        print(f"Original objects: {len(objects)}")
        print(f"Filtered objects: {filtered_data['object_count']}")
        print(f"Target preserved: {'target' in [o['label'] for o in filtered_data['objects']]}")


if __name__ == "__main__":
    # 运行测试
    unittest.main(verbosity=2)

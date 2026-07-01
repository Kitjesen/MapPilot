"""
test_slow_path_real_llm.py — Slow Path真实LLM测试

目标:
  1. 使用真实的LLM API测试Slow Path
  2. 验证完整的Fast-Slow流程
  3. 测试实际的推理效果

使用方法:
  1. 设置环境变量: export OPENAI_API_KEY=your_key
  2. 运行: pytest test_slow_path_real_llm.py -v -s

注意:
  - 需要真实的API key
  - 会产生API费用
  - 可以跳过（使用 @unittest.skipIf）
"""

import asyncio
import json
import os
import unittest


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


@unittest.skipIf(
    not os.getenv("OPENAI_API_KEY"),
    "Skipping real LLM tests (no API key)"
)
class TestSlowPathRealLLM(unittest.TestCase):
    """Slow Path真实LLM测试"""

    def setUp(self):
        self.config = LLMConfig(
            backend="openai",
            model="gpt-4o-mini",  # 使用便宜的模型
            api_key=os.getenv("OPENAI_API_KEY"),
        )
        self.resolver = GoalResolver(
            self.config,
            fast_path_threshold=0.95,  # 高阈值，强制走Slow Path
        )
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

    def tearDown(self):
        self.loop.close()

    def test_real_llm_simple_navigation(self):
        """
        测试真实LLM：简单导航任务

        场景: "go to the red chair"
        """
        objects = [
            {
                "id": 0,
                "label": "red chair",
                "position": {"x": 3.0, "y": 2.0, "z": 0.0},
                "score": 0.85,
                "detection_count": 5,
            },
            {
                "id": 1,
                "label": "blue chair",
                "position": {"x": 5.0, "y": 3.0, "z": 0.0},
                "score": 0.82,
                "detection_count": 4,
            },
            {
                "id": 2,
                "label": "table",
                "position": {"x": 4.0, "y": 2.5, "z": 0.0},
                "score": 0.88,
                "detection_count": 6,
            },
        ]

        sg = make_scene_graph(objects)
        instruction = "go to the red chair"

        print(f"\n{'='*60}")
        print("Test: Simple Navigation")
        print(f"{'='*60}")
        print(f"Instruction: {instruction}")
        print(f"Objects: {len(objects)}")

        # 运行真实LLM推理
        result = self.loop.run_until_complete(
            self.resolver.resolve(
                instruction=instruction,
                scene_graph_json=sg,
                robot_position={"x": 0, "y": 0, "z": 0},
                language="en",
            )
        )

        print("\nResult:")
        print(f"  Path: {result.path}")
        print(f"  Action: {result.action}")
        print(f"  Target: {result.target_label}")
        print(f"  Position: ({result.target_x:.1f}, {result.target_y:.1f}, {result.target_z:.1f})")
        print(f"  Confidence: {result.confidence:.2f}")
        print(f"  Reasoning: {result.reasoning}")

        # 验证结果
        self.assertTrue(result.is_valid)
        self.assertEqual(result.action, "navigate")
        self.assertEqual(result.target_label, "red chair")
        self.assertAlmostEqual(result.target_x, 3.0, places=1)

    def test_real_llm_spatial_reasoning(self):
        """
        测试真实LLM：空间关系推理

        场景: "find the chair near the door"
        """
        objects = [
            {
                "id": 0,
                "label": "chair",
                "position": {"x": 3.0, "y": 2.0, "z": 0.0},
                "score": 0.85,
                "detection_count": 5,
            },
            {
                "id": 1,
                "label": "door",
                "position": {"x": 3.0, "y": 3.0, "z": 0.0},
                "score": 0.9,
                "detection_count": 6,
            },
            {
                "id": 2,
                "label": "chair",
                "position": {"x": 10.0, "y": 10.0, "z": 0.0},
                "score": 0.83,
                "detection_count": 4,
            },
        ]

        relations = [
            {"subject_id": 0, "relation": "near", "object_id": 1, "distance": 1.0},
        ]

        sg = make_scene_graph(objects, relations)
        instruction = "find the chair near the door"

        print(f"\n{'='*60}")
        print("Test: Spatial Reasoning")
        print(f"{'='*60}")
        print(f"Instruction: {instruction}")
        print(f"Objects: {len(objects)}")
        print(f"Relations: {len(relations)}")

        # 运行真实LLM推理
        result = self.loop.run_until_complete(
            self.resolver.resolve(
                instruction=instruction,
                scene_graph_json=sg,
                language="en",
            )
        )

        print("\nResult:")
        print(f"  Path: {result.path}")
        print(f"  Target: {result.target_label}")
        print(f"  Position: ({result.target_x:.1f}, {result.target_y:.1f})")
        print(f"  Confidence: {result.confidence:.2f}")
        print(f"  Reasoning: {result.reasoning}")

        # 验证结果：应该选择靠近门的椅子（id=0）
        self.assertTrue(result.is_valid)
        self.assertAlmostEqual(result.target_x, 3.0, places=1)
        self.assertAlmostEqual(result.target_y, 2.0, places=1)

    def test_real_llm_with_esca_filtering(self):
        """
        测试真实LLM + ESCA过滤

        场景: 大场景图（100个物体），测试ESCA过滤效果
        """
        # 构建大场景图
        objects = []
        for i in range(100):
            objects.append({
                "id": i,
                "label": f"random_object_{i}",
                "position": {"x": float(i), "y": float(i % 10), "z": 0.0},
                "score": 0.5,
                "detection_count": 2,
            })

        # 添加目标物体
        objects.append({
            "id": 999,
            "label": "fire extinguisher",
            "position": {"x": 50.0, "y": 50.0, "z": 0.8},
            "score": 0.92,
            "detection_count": 7,
        })

        sg = make_scene_graph(objects)
        instruction = "find the fire extinguisher"

        print(f"\n{'='*60}")
        print("Test: ESCA Filtering + LLM")
        print(f"{'='*60}")
        print(f"Instruction: {instruction}")
        print(f"Original objects: {len(objects)}")

        # 测试ESCA过滤
        filtered_sg = self.resolver._selective_grounding(instruction, sg, max_objects=15)
        filtered_data = json.loads(filtered_sg)

        print(f"Filtered objects: {filtered_data['object_count']}")
        print(f"Token reduction: {(1 - filtered_data['object_count']/len(objects))*100:.1f}%")

        # 运行真实LLM推理
        result = self.loop.run_until_complete(
            self.resolver.resolve(
                instruction=instruction,
                scene_graph_json=sg,
                language="en",
            )
        )

        print("\nResult:")
        print(f"  Path: {result.path}")
        print(f"  Target: {result.target_label}")
        print(f"  Position: ({result.target_x:.1f}, {result.target_y:.1f})")
        print(f"  Confidence: {result.confidence:.2f}")

        # 验证结果
        self.assertTrue(result.is_valid)
        self.assertEqual(result.target_label, "fire extinguisher")
        self.assertAlmostEqual(result.target_x, 50.0, places=1)

    def test_real_llm_chinese_instruction(self):
        """
        测试真实LLM：中文指令

        场景: "去红色灭火器"
        """
        objects = [
            {
                "id": 0,
                "label": "灭火器",
                "position": {"x": 5.0, "y": 3.0, "z": 0.8},
                "score": 0.88,
                "detection_count": 6,
            },
            {
                "id": 1,
                "label": "红色盒子",
                "position": {"x": 6.0, "y": 4.0, "z": 0.5},
                "score": 0.85,
                "detection_count": 5,
            },
        ]

        sg = make_scene_graph(objects)
        instruction = "去红色灭火器"

        print(f"\n{'='*60}")
        print("Test: Chinese Instruction")
        print(f"{'='*60}")
        print(f"Instruction: {instruction}")
        print(f"Objects: {len(objects)}")

        # 运行真实LLM推理
        result = self.loop.run_until_complete(
            self.resolver.resolve(
                instruction=instruction,
                scene_graph_json=sg,
                language="zh",
            )
        )

        print("\nResult:")
        print(f"  Path: {result.path}")
        print(f"  Target: {result.target_label}")
        print(f"  Position: ({result.target_x:.1f}, {result.target_y:.1f})")
        print(f"  Confidence: {result.confidence:.2f}")
        print(f"  Reasoning: {result.reasoning}")

        # 验证结果
        self.assertTrue(result.is_valid)
        self.assertEqual(result.target_label, "灭火器")


if __name__ == "__main__":
    # 检查API key
    if not os.getenv("OPENAI_API_KEY"):
        print("\n" + "="*60)
        print("WARNING: No OPENAI_API_KEY found")
        print("="*60)
        print("\nTo run real LLM tests:")
        print("  1. Set API key: export OPENAI_API_KEY=your_key")
        print("  2. Run: pytest test_slow_path_real_llm.py -v -s")
        print("\nSkipping real LLM tests...")
        print("="*60 + "\n")

    # 运行测试
    unittest.main(verbosity=2)

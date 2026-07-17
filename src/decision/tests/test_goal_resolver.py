"""Decision module."""

import asyncio
import json
import unittest
from unittest.mock import AsyncMock, MagicMock

from decision.goals.resolver import GoalResolver
from decision.llm.client import (
    ClaudeClient,
    LLMConfig,
    LLMError,
    OpenAIClient,
    QwenClient,
    create_llm_client,
)
from decision.llm.prompts import (
    build_exploration_prompt,
    build_goal_resolution_prompt,
)


class TestLLMClientFactory(unittest.TestCase):
    """Test L L M Client Factory."""

    def test_create_openai_client(self):
        config = LLMConfig(backend="openai")
        client = create_llm_client(config)
        self.assertIsInstance(client, OpenAIClient)

    def test_create_claude_client(self):
        config = LLMConfig(backend="claude")
        client = create_llm_client(config)
        self.assertIsInstance(client, ClaudeClient)

    def test_create_qwen_client(self):
        config = LLMConfig(backend="qwen")
        client = create_llm_client(config)
        self.assertIsInstance(client, QwenClient)

    def test_create_unknown_raises(self):
        config = LLMConfig(backend="unknown")
        with self.assertRaises(ValueError):
            create_llm_client(config)


class TestPromptTemplates(unittest.TestCase):
    """Test Prompt Templates."""

    def test_goal_resolution_prompt_zh(self):
        messages = build_goal_resolution_prompt(
            instruction="去红色灭火器旁边",
            scene_graph_json='{"objects": []}',
            robot_position={"x": 1.0, "y": 2.0, "z": 0.0},
            language="zh",
        )
        self.assertEqual(len(messages), 2)
        self.assertEqual(messages[0]["role"], "system")
        self.assertEqual(messages[1]["role"], "user")
        self.assertIn("红色灭火器", messages[1]["content"])
        self.assertIn("当前位置", messages[1]["content"])

    def test_goal_resolution_prompt_en(self):
        messages = build_goal_resolution_prompt(
            instruction="go to the red door",
            scene_graph_json='{"objects": []}',
            language="en",
        )
        self.assertEqual(len(messages), 2)
        self.assertIn("quadruped robot", messages[0]["content"])
        self.assertIn("red door", messages[1]["content"])

    def test_exploration_prompt(self):
        messages = build_exploration_prompt(
            instruction="找灭火器",
            explored_directions=[{"x": 3.0, "y": 4.0}],
            robot_position={"x": 1.0, "y": 2.0},
            language="zh",
        )
        self.assertEqual(len(messages), 2)
        self.assertIn("探索", messages[0]["content"])


class TestGoalResolver(unittest.TestCase):
    """Test Goal Resolver."""

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="test")
        self.loop = asyncio.new_event_loop()

    def tearDown(self):
        self.loop.close()

    def test_parse_navigate_response(self):
        """Test parse navigate response."""
        resolver = GoalResolver(self.config)
        response_text = json.dumps(
            {
                "action": "navigate",
                "target": {"x": 3.2, "y": 1.5, "z": 0.0},
                "target_label": "红色灭火器",
                "confidence": 0.85,
                "reasoning": "场景图中有一个红色灭火器",
            }
        )
        result = resolver._parse_llm_response(response_text)
        self.assertTrue(result.is_valid)
        self.assertEqual(result.action, "navigate")
        self.assertAlmostEqual(result.target_x, 3.2)
        self.assertAlmostEqual(result.target_y, 1.5)
        self.assertEqual(result.target_label, "红色灭火器")
        self.assertAlmostEqual(result.confidence, 0.85)

    def test_parse_explore_response(self):
        """Test parse explore response."""
        resolver = GoalResolver(self.config)
        response_text = json.dumps(
            {
                "action": "explore",
                "target": {"x": 5.0, "y": 0.0, "z": 0.0},
                "target_label": "",
                "confidence": 0.3,
                "reasoning": "目标不在场景图中",
            }
        )
        result = resolver._parse_llm_response(response_text)
        self.assertTrue(result.is_valid)
        self.assertEqual(result.action, "explore")
        self.assertAlmostEqual(result.confidence, 0.3)

    def test_parse_markdown_json(self):
        """Test parse markdown json."""
        resolver = GoalResolver(self.config)
        response_text = """Here's the plan:
```json
{
  "action": "navigate",
  "target": {"x": 1.0, "y": 2.0, "z": 0.0},
  "target_label": "door",
  "confidence": 0.9,
  "reasoning": "Found it"
}
```"""
        result = resolver._parse_llm_response(response_text)
        self.assertTrue(result.is_valid)
        self.assertEqual(result.action, "navigate")

    def test_parse_invalid_response(self):
        """Test parse invalid response."""
        resolver = GoalResolver(self.config)
        result = resolver._parse_llm_response("this is not json at all")
        self.assertFalse(result.is_valid)
        self.assertEqual(result.action, "error")

    def test_reset_exploration(self):
        """Test reset exploration."""
        resolver = GoalResolver(self.config)
        resolver._explored_directions.append({"x": 1.0, "y": 2.0})
        resolver._explore_step_count = 5
        resolver.reset_exploration()
        self.assertEqual(len(resolver._explored_directions), 0)
        self.assertEqual(resolver._explore_step_count, 0)

    def test_resolve_with_mock_llm(self):
        """Test resolve with mock llm."""
        resolver = GoalResolver(self.config)
        mock_response = json.dumps(
            {
                "action": "navigate",
                "target": {"x": 5.0, "y": 3.0, "z": 0.0},
                "target_label": "chair",
                "confidence": 0.92,
                "reasoning": "Found a chair in scene graph",
            }
        )
        resolver._primary = MagicMock()
        resolver._primary.is_available.return_value = True
        resolver._primary.chat = AsyncMock(return_value=mock_response)

        result = self.loop.run_until_complete(
            resolver.resolve(
                instruction="go to the chair",
                scene_graph_json='{"objects": [{"label": "chair", "position": {"x": 5, "y": 3, "z": 0}}]}',
                robot_position={"x": 0, "y": 0, "z": 0},
                language="en",
            )
        )
        self.assertTrue(result.is_valid)
        self.assertEqual(result.action, "navigate")
        self.assertAlmostEqual(result.target_x, 5.0)

    def test_fallback_on_primary_failure(self):
        """Test fallback on primary failure."""
        fallback_config = LLMConfig(backend="qwen", model="test")
        resolver = GoalResolver(self.config, fallback_config=fallback_config)

        mock_response = json.dumps(
            {
                "action": "navigate",
                "target": {"x": 1.0, "y": 1.0, "z": 0.0},
                "target_label": "door",
                "confidence": 0.8,
                "reasoning": "Fallback found it",
            }
        )

        resolver._primary = MagicMock()
        resolver._primary.is_available.return_value = True
        resolver._primary.chat = AsyncMock(side_effect=LLMError("API Error"))

        resolver._fallback = MagicMock()
        resolver._fallback.is_available.return_value = True
        resolver._fallback.chat = AsyncMock(return_value=mock_response)

        result = self.loop.run_until_complete(
            resolver.resolve(
                instruction="go to the door",
                scene_graph_json="{}",
                language="en",
            )
        )
        self.assertTrue(result.is_valid)
        self.assertEqual(result.target_label, "door")


if __name__ == "__main__":
    unittest.main()

"""Decision module."""

import json
import unittest

from decision.tasks.decomposition import (
    SubGoal,
    SubGoalAction,
    SubGoalStatus,
    TaskDecomposer,
    TaskPlan,
)


class TestRuleDecomposition(unittest.TestCase):
    """Test Rule Decomposition."""

    def setUp(self):
        self.decomposer = TaskDecomposer()

    def test_simple_nav_zh(self):
        plan = self.decomposer.decompose_with_rules("去门那里")
        self.assertIsNotNone(plan)
        actions = [sg.action for sg in plan.subgoals]
        self.assertIn(SubGoalAction.NAVIGATE, actions)
        self.assertIn(SubGoalAction.APPROACH, actions)
        self.assertIn(SubGoalAction.VERIFY, actions)

        self.assertNotIn(SubGoalAction.FIND, actions)

    def test_simple_nav_zh_variants(self):
        for cmd in ["到椅子旁边", "走到灭火器", "前往出口", "导航到大门"]:
            plan = self.decomposer.decompose_with_rules(cmd)
            self.assertIsNotNone(plan, f"Should decompose: '{cmd}'")
            self.assertTrue(len(plan.subgoals) >= 3)

    def test_simple_find_zh(self):
        plan = self.decomposer.decompose_with_rules("找红色灭火器")
        self.assertIsNotNone(plan)
        actions = [sg.action for sg in plan.subgoals]
        self.assertIn(SubGoalAction.FIND, actions)
        self.assertIn(SubGoalAction.LOOK_AROUND, actions)
        self.assertIn(SubGoalAction.NAVIGATE, actions)
        self.assertIn(SubGoalAction.VERIFY, actions)

    def test_simple_nav_en(self):
        plan = self.decomposer.decompose_with_rules("go to the kitchen")
        self.assertIsNotNone(plan)
        self.assertTrue(any(sg.action == SubGoalAction.NAVIGATE for sg in plan.subgoals))

    def test_simple_find_en(self):
        plan = self.decomposer.decompose_with_rules("find the red chair")
        self.assertIsNotNone(plan)
        self.assertTrue(any(sg.action == SubGoalAction.FIND for sg in plan.subgoals))

    def test_complex_instruction_returns_none(self):
        plan = self.decomposer.decompose_with_rules("先去厨房拿杯子然后回来")
        self.assertIsNone(plan)

    def test_unrecognized_returns_none(self):
        plan = self.decomposer.decompose_with_rules("打开空调")
        self.assertIsNone(plan)


class TestLLMDecompositionParsing(unittest.TestCase):
    """Test L L M Decomposition Parsing."""

    def setUp(self):
        self.decomposer = TaskDecomposer()

    def test_parse_valid_json(self):
        response = json.dumps(
            {
                "subgoals": [
                    {"action": "navigate", "target": "kitchen"},
                    {"action": "find", "target": "red cup"},
                    {"action": "approach", "target": "red cup"},
                    {"action": "verify", "target": "red cup"},
                ]
            }
        )
        plan = self.decomposer.parse_decomposition_response("去厨房拿红杯子", response)
        self.assertEqual(len(plan.subgoals), 4)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.NAVIGATE)
        self.assertEqual(plan.subgoals[1].action, SubGoalAction.FIND)
        self.assertEqual(plan.subgoals[1].target, "red cup")

    def test_parse_markdown_json(self):
        response = """Here's the plan:
```json
{
  "subgoals": [
    {"action": "look_around", "target": "area"},
    {"action": "navigate", "target": "door"}
  ]
}
```"""
        plan = self.decomposer.parse_decomposition_response("test", response)
        self.assertEqual(len(plan.subgoals), 2)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.LOOK_AROUND)

    def test_parse_invalid_returns_fallback(self):
        plan = self.decomposer.parse_decomposition_response("去门", "this is garbage")
        self.assertIsNotNone(plan)
        self.assertTrue(len(plan.subgoals) >= 1)

    def test_parse_unknown_action_defaults_navigate(self):
        response = json.dumps(
            {
                "subgoals": [
                    {"action": "fly", "target": "moon"},
                ]
            }
        )
        plan = self.decomposer.parse_decomposition_response("fly", response)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.NAVIGATE)

    def test_build_decomposition_prompt(self):
        messages = self.decomposer.build_decomposition_prompt(
            "去厨房拿红杯子", scene_summary="3 objects in scene", language="zh"
        )
        self.assertEqual(len(messages), 2)
        self.assertEqual(messages[0]["role"], "system")
        self.assertIn("navigate", messages[0]["content"])
        self.assertIn("红杯子", messages[1]["content"])

    def test_build_decomposition_prompt_en(self):
        messages = self.decomposer.build_decomposition_prompt("go to the kitchen", language="en")
        self.assertEqual(len(messages), 2)
        self.assertIn("robot task planner", messages[0]["content"])


class TestTaskPlanStateMachine(unittest.TestCase):
    """Test Task Plan State Machine."""

    def _make_plan(self, n_steps=3):
        subgoals = [SubGoal(step_id=i, action=SubGoalAction.NAVIGATE, target=f"target_{i}") for i in range(n_steps)]
        return TaskPlan(instruction="test", subgoals=subgoals)

    def test_initial_state(self):
        plan = self._make_plan()
        self.assertFalse(plan.is_complete)
        self.assertFalse(plan.is_failed)
        active = plan.active_subgoal
        self.assertIsNotNone(active)
        self.assertEqual(active.step_id, 0)

    def test_advance(self):
        plan = self._make_plan(3)
        plan.advance()  # complete step 0
        self.assertEqual(plan.current_step, 1)
        self.assertEqual(plan.subgoals[0].status, SubGoalStatus.COMPLETED)
        active = plan.active_subgoal
        self.assertIsNotNone(active)
        self.assertEqual(active.step_id, 1)

    def test_complete_all(self):
        plan = self._make_plan(2)
        plan.advance()
        plan.advance()
        self.assertTrue(plan.is_complete)
        self.assertIsNone(plan.active_subgoal)

    def test_fail_with_retry(self):
        plan = self._make_plan(2)
        plan.fail_current()  # retry 1 (max=2)
        self.assertFalse(plan.is_failed)
        self.assertEqual(plan.subgoals[0].status, SubGoalStatus.PENDING)
        self.assertEqual(plan.subgoals[0].retry_count, 1)

        plan.fail_current()
        self.assertTrue(plan.is_failed)
        self.assertEqual(plan.subgoals[0].status, SubGoalStatus.FAILED)

    def test_to_dict(self):
        plan = self._make_plan(2)
        plan.advance()
        d = plan.to_dict()
        self.assertEqual(d["total_steps"], 2)
        self.assertEqual(d["current_step"], 1)
        self.assertFalse(d["is_complete"])
        self.assertEqual(len(d["subgoals"]), 2)
        self.assertEqual(d["subgoals"][0]["status"], "completed")


if __name__ == "__main__":
    unittest.main()

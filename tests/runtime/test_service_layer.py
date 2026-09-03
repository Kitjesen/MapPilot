"""Tests for the retained planner service layer.

Pure algorithm testing with mocks. No ROS2, no GPU, no API keys.
"""

import unittest
from unittest.mock import MagicMock

import numpy as np

# ---------------------------------------------------------------------------
# Mock components
# ---------------------------------------------------------------------------


class MockGoalResolver:
    def __init__(self):
        self._fast_path_threshold = 0.75

    def fast_resolve(self, instruction, scene_graph_json):
        result = MagicMock()
        result.confidence = 0.9
        result.label = "chair"
        result.position = [1.0, 2.0, 0.0]
        return result

    def _resolve_by_tag(self, instruction):
        if "office" in instruction:
            r = MagicMock()
            r.label = "office"
            r.confidence = 1.0
            return r
        return None


class MockFrontierScorer:
    def __init__(self):
        self._best = None

    def update_costmap(self, costmap, resolution, ox, oy):
        pass

    def extract_frontiers(self, robot_pos):
        f = MagicMock()
        f.position = np.array([5.0, 3.0])
        f.score = 0.8
        self._best = f
        return [f]

    def score_frontiers(self, **kwargs):
        pass

    def get_best_frontier(self):
        return self._best

    def record_failure(self, pos):
        pass


class MockActionExecutor:
    def generate_navigate_command(self, target, robot_pos):
        cmd = MagicMock()
        cmd.action_type = "navigate"
        cmd.target = target
        return cmd

    def generate_approach_command(self, target, robot_pos, stop_distance=1.0):
        cmd = MagicMock()
        cmd.action_type = "approach"
        return cmd

    def generate_look_around_command(self, robot_pos):
        cmd = MagicMock()
        cmd.action_type = "look_around"
        return cmd


# ---------------------------------------------------------------------------
# GoalResolutionService tests
# ---------------------------------------------------------------------------


class TestGoalResolutionService(unittest.TestCase):
    def test_resolve_fast(self):
        from decision.tasks.services import GoalResolutionService

        svc = GoalResolutionService(resolver=MockGoalResolver())
        result = svc.resolve_fast("find the chair", '{"objects":[]}')
        self.assertIsNotNone(result)
        self.assertEqual(result.confidence, 0.9)

    def test_resolve_auto(self):
        from decision.tasks.services import GoalResolutionService

        svc = GoalResolutionService(resolver=MockGoalResolver())
        result = svc.resolve("find the chair", '{"objects":[]}')
        self.assertIsNotNone(result)

    def test_resolve_by_tag(self):
        from decision.tasks.services import GoalResolutionService

        svc = GoalResolutionService(resolver=MockGoalResolver())
        result = svc.resolve_by_tag("go to the office")
        self.assertIsNotNone(result)
        self.assertEqual(result.label, "office")

    def test_resolve_by_tag_miss(self):
        from decision.tasks.services import GoalResolutionService

        svc = GoalResolutionService(resolver=MockGoalResolver())
        result = svc.resolve_by_tag("find a dog")
        self.assertIsNone(result)


# ---------------------------------------------------------------------------
# FrontierExplorationService tests
# ---------------------------------------------------------------------------


class TestFrontierExplorationService(unittest.TestCase):
    def test_evaluate(self):
        from decision.tasks.services import FrontierExplorationService

        svc = FrontierExplorationService(scorer=MockFrontierScorer())
        costmap = np.zeros((100, 100), dtype=np.int8)
        best = svc.evaluate(costmap, 0.05, 0.0, 0.0, np.array([0.0, 0.0]), "explore")
        self.assertIsNotNone(best)
        self.assertAlmostEqual(best.score, 0.8)

    def test_record_failure(self):
        from decision.tasks.services import FrontierExplorationService

        svc = FrontierExplorationService(scorer=MockFrontierScorer())
        svc.record_failure(np.array([1.0, 2.0]))  # should not crash


# ---------------------------------------------------------------------------
# ActionExecutionService tests
# ---------------------------------------------------------------------------


class TestActionExecutionService(unittest.TestCase):
    def test_navigate(self):
        from decision.tasks.services import ActionExecutionService

        svc = ActionExecutionService(executor=MockActionExecutor())
        cmd = svc.navigate(np.array([5, 3, 0]), np.array([0, 0, 0]))
        self.assertEqual(cmd.action_type, "navigate")

    def test_approach(self):
        from decision.tasks.services import ActionExecutionService

        svc = ActionExecutionService(executor=MockActionExecutor())
        cmd = svc.approach(np.array([5, 3, 0]), np.array([0, 0, 0]))
        self.assertEqual(cmd.action_type, "approach")

    def test_look_around(self):
        from decision.tasks.services import ActionExecutionService

        svc = ActionExecutionService(executor=MockActionExecutor())
        cmd = svc.look_around(np.array([0, 0, 0]))
        self.assertEqual(cmd.action_type, "look_around")


if __name__ == "__main__":
    unittest.main(verbosity=2)

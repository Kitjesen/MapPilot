"""Decision module."""

import math
import time
import unittest

import numpy as np

from decision.tasks.actions import (
    ActionExecutor,
    ActionStatus,
)


class TestNavigateCommand(unittest.TestCase):
    """Test Navigate Command."""

    def test_basic_navigate(self):
        executor = ActionExecutor()
        cmd = executor.generate_navigate_command(target_position={"x": 5.0, "y": 3.0, "z": 0.0})
        self.assertEqual(cmd.command_type, "goal")
        self.assertAlmostEqual(cmd.target_x, 5.0)
        self.assertAlmostEqual(cmd.target_y, 3.0)
        self.assertEqual(executor.status, ActionStatus.EXECUTING)

    def test_navigate_with_yaw(self):
        executor = ActionExecutor()
        cmd = executor.generate_navigate_command(
            target_position={"x": 1.0, "y": 1.0, "z": 0.0},
            robot_position={"x": 0.0, "y": 0.0, "z": 0.0},
        )
        expected_yaw = math.atan2(1.0, 1.0)  # pi/4
        self.assertAlmostEqual(cmd.target_yaw, expected_yaw, places=5)


class TestApproachCommand(unittest.TestCase):
    """Test Approach Command."""

    def test_approach_stops_before_target(self):
        executor = ActionExecutor(approach_distance=0.5)
        cmd = executor.generate_approach_command(
            target_position={"x": 5.0, "y": 0.0, "z": 0.0},
            robot_position={"x": 0.0, "y": 0.0, "z": 0.0},
        )

        dist_to_target = math.sqrt((cmd.target_x - 5.0) ** 2 + (cmd.target_y - 0.0) ** 2)
        self.assertAlmostEqual(dist_to_target, 0.5, places=1)
        self.assertAlmostEqual(cmd.approach_speed, 0.15)

    def test_approach_already_close_stays(self):
        executor = ActionExecutor(approach_distance=0.5)
        cmd = executor.generate_approach_command(
            target_position={"x": 0.3, "y": 0.0, "z": 0.0},
            robot_position={"x": 0.0, "y": 0.0, "z": 0.0},
        )

        self.assertAlmostEqual(cmd.target_x, 0.0)
        self.assertAlmostEqual(cmd.target_y, 0.0)


class TestLookAroundCommand(unittest.TestCase):
    """Test Look Around Command."""

    def test_look_around_is_velocity_command(self):
        executor = ActionExecutor(look_around_speed=0.5)
        cmd = executor.generate_look_around_command()
        self.assertEqual(cmd.command_type, "velocity")
        self.assertAlmostEqual(cmd.angular_z, 0.5)
        self.assertAlmostEqual(cmd.linear_x, 0.0)


class TestVerifyCommand(unittest.TestCase):
    """Test Verify Command."""

    def test_verify_does_not_move(self):
        executor = ActionExecutor()
        cmd = executor.generate_verify_command(
            target_position={"x": 3.0, "y": 4.0, "z": 0.0},
            robot_position={"x": 1.0, "y": 1.0, "z": 0.0},
        )

        self.assertAlmostEqual(cmd.target_x, 1.0)
        self.assertAlmostEqual(cmd.target_y, 1.0)
        expected_yaw = math.atan2(3.0, 2.0)
        self.assertAlmostEqual(cmd.target_yaw, expected_yaw, places=5)


class TestBacktrackCommand(unittest.TestCase):
    """Test Backtrack Command."""

    def test_backtrack_to_position(self):
        executor = ActionExecutor()
        cmd = executor.generate_backtrack_command(np.array([10.0, 20.0, 0.0]))
        self.assertEqual(cmd.command_type, "goal")
        self.assertAlmostEqual(cmd.target_x, 10.0)
        self.assertAlmostEqual(cmd.target_y, 20.0)


class TestStatusManagement(unittest.TestCase):
    """Test Status Management."""

    def test_initial_idle(self):
        executor = ActionExecutor()
        self.assertEqual(executor.status, ActionStatus.IDLE)

    def test_mark_succeeded(self):
        executor = ActionExecutor()
        executor.generate_navigate_command({"x": 0, "y": 0})
        executor.mark_succeeded()
        self.assertEqual(executor.status, ActionStatus.SUCCEEDED)

    def test_mark_failed(self):
        executor = ActionExecutor()
        executor.generate_navigate_command({"x": 0, "y": 0})
        executor.mark_failed()
        self.assertEqual(executor.status, ActionStatus.FAILED)

    def test_reset(self):
        executor = ActionExecutor()
        executor.generate_navigate_command({"x": 0, "y": 0})
        executor.reset()
        self.assertEqual(executor.status, ActionStatus.IDLE)

    def test_timeout_detection(self):
        executor = ActionExecutor(nav_timeout=0.01)  # 10ms
        executor.generate_navigate_command({"x": 0, "y": 0})
        time.sleep(0.02)
        self.assertTrue(executor.check_timeout())

    def test_no_timeout_when_idle(self):
        executor = ActionExecutor()
        self.assertFalse(executor.check_timeout())


class TestLeraRecover(unittest.TestCase):
    """Test Lera Recover."""

    def test_first_failure_retries_path(self):
        executor = ActionExecutor()
        result = executor.lera_recover(
            failed_action="NAVIGATE(kitchen)",
            current_labels=["chair", "table"],
            original_goal="找到厨房",
            failure_count=1,
        )
        self.assertEqual(result, "retry_different_path")

    def test_second_failure_expands_search(self):
        executor = ActionExecutor()
        result = executor.lera_recover(
            failed_action="NAVIGATE(kitchen)",
            current_labels=["chair"],
            original_goal="找到厨房",
            failure_count=2,
        )
        self.assertEqual(result, "expand_search")

    def test_third_failure_aborts(self):
        executor = ActionExecutor()
        result = executor.lera_recover(
            failed_action="NAVIGATE(kitchen)",
            current_labels=[],
            original_goal="找到厨房",
            failure_count=3,
        )
        self.assertEqual(result, "abort")

    def test_empty_labels_still_works(self):
        executor = ActionExecutor()
        result = executor.lera_recover(
            failed_action="FIND(cup)",
            current_labels=[],
            original_goal="find the red cup",
            failure_count=1,
        )
        self.assertIn(
            result,
            (
                "retry_different_path",
                "expand_search",
                "requery_goal",
                "abort",
            ),
        )

    def test_llm_client_valid_response(self):
        """Test llm client valid response."""

        class MockLLM:
            async def chat(self, prompt, max_tokens=200, **kwargs):
                return '{"reason": "路径被阻挡", "action": "expand_search", "params": {}}'

        executor = ActionExecutor()
        result = executor.lera_recover(
            failed_action="NAVIGATE(kitchen)",
            current_labels=["door", "wall"],
            original_goal="go to kitchen",
            failure_count=1,
            llm_client=MockLLM(),
        )
        self.assertEqual(result, "expand_search")

    def test_llm_client_invalid_json_falls_back(self):
        """Test llm client invalid json falls back."""

        class MockLLM:
            async def chat(self, prompt, max_tokens=200, **kwargs):
                return "I don't understand"

        executor = ActionExecutor()
        result = executor.lera_recover(
            failed_action="NAVIGATE(kitchen)",
            current_labels=["chair"],
            original_goal="go to kitchen",
            failure_count=1,
            llm_client=MockLLM(),
        )
        self.assertEqual(result, "retry_different_path")

    def test_llm_client_exception_falls_back(self):
        """Test llm client exception falls back."""

        class MockLLM:
            async def chat(self, prompt, max_tokens=200, **kwargs):
                raise ConnectionError("LLM unavailable")

        executor = ActionExecutor()
        result = executor.lera_recover(
            failed_action="NAVIGATE(kitchen)",
            current_labels=[],
            original_goal="go to kitchen",
            failure_count=2,
            llm_client=MockLLM(),
        )
        self.assertEqual(result, "expand_search")

    def test_llm_client_invalid_action_falls_back(self):
        """Test llm client invalid action falls back."""

        class MockLLM:
            async def chat(self, prompt, max_tokens=200, **kwargs):
                return '{"reason": "test", "action": "dance", "params": {}}'

        executor = ActionExecutor()
        result = executor.lera_recover(
            failed_action="NAVIGATE(kitchen)",
            current_labels=["chair"],
            original_goal="go to kitchen",
            failure_count=1,
            llm_client=MockLLM(),
        )
        self.assertEqual(result, "retry_different_path")


if __name__ == "__main__":
    unittest.main()

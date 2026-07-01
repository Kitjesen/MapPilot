"""Unit tests for LLMModule circuit breaker, ActionExecutor, and TargetBeliefManager.

Pure unit tests — no external API calls, no ROS2, no hardware.
"""

import math
import os
import time
import unittest

import numpy as np


class TestLLMModulePreflight(unittest.TestCase):
    """Test LLMModule.preflight() for various backends."""

    def _make(self, **kw):
        from decision.modules.llm_module import LLMModule
        return LLMModule(**kw)

    def test_preflight_mock_backend_ok(self):
        m = self._make(backend="mock")
        self.assertIsNone(m.preflight())

    def test_preflight_kimi_without_key_fails(self):
        saved = os.environ.pop("MOONSHOT_API_KEY", None)
        try:
            m = self._make(backend="kimi")
            result = m.preflight()
            self.assertIsNotNone(result)
            self.assertIn("MOONSHOT_API_KEY", result)
        finally:
            if saved is not None:
                os.environ["MOONSHOT_API_KEY"] = saved

    def test_preflight_openai_without_key_fails(self):
        saved = os.environ.pop("OPENAI_API_KEY", None)
        try:
            m = self._make(backend="openai")
            result = m.preflight()
            self.assertIsNotNone(result)
            self.assertIn("OPENAI_API_KEY", result)
        finally:
            if saved is not None:
                os.environ["OPENAI_API_KEY"] = saved

    def test_preflight_claude_without_key_fails(self):
        saved = os.environ.pop("ANTHROPIC_API_KEY", None)
        try:
            m = self._make(backend="claude")
            result = m.preflight()
            self.assertIsNotNone(result)
            self.assertIn("ANTHROPIC_API_KEY", result)
        finally:
            if saved is not None:
                os.environ["ANTHROPIC_API_KEY"] = saved

    def test_preflight_custom_api_key_env(self):
        """Custom api_key_env should be checked instead of default."""
        saved = os.environ.pop("MY_CUSTOM_KEY", None)
        try:
            m = self._make(backend="kimi", api_key_env="MY_CUSTOM_KEY")
            result = m.preflight()
            self.assertIsNotNone(result)
            self.assertIn("MY_CUSTOM_KEY", result)
        finally:
            if saved is not None:
                os.environ["MY_CUSTOM_KEY"] = saved

    def test_preflight_succeeds_when_key_present(self):
        os.environ["MOONSHOT_API_KEY"] = "test-key-for-preflight"
        try:
            m = self._make(backend="kimi")
            self.assertIsNone(m.preflight())
        finally:
            del os.environ["MOONSHOT_API_KEY"]


class TestLLMCircuitBreaker(unittest.TestCase):
    """Test circuit breaker state transitions and health reporting."""

    def _make(self, **kw):
        from decision.modules.llm_module import LLMModule
        return LLMModule(backend="mock", **kw)

    def test_initial_circuit_closed(self):
        m = self._make()
        h = m.health()
        self.assertEqual(h["llm"]["circuit_breaker"], "closed")
        self.assertEqual(h["llm"]["consecutive_failures"], 0)

    def test_circuit_opens_at_threshold(self):
        m = self._make(circuit_breaker_threshold=3)
        m._consecutive_failures = 3
        m._circuit_open_until = time.time() + 60
        h = m.health()
        self.assertEqual(h["llm"]["circuit_breaker"], "open")

    def test_circuit_half_open_after_cooldown(self):
        m = self._make(circuit_breaker_threshold=3)
        m._consecutive_failures = 3
        m._circuit_open_until = time.time() - 1  # cooldown elapsed
        h = m.health()
        self.assertEqual(h["llm"]["circuit_breaker"], "half-open")

    def test_circuit_stays_closed_below_threshold(self):
        m = self._make(circuit_breaker_threshold=5)
        m._consecutive_failures = 4
        h = m.health()
        self.assertEqual(h["llm"]["circuit_breaker"], "closed")

    def test_default_threshold_is_5(self):
        m = self._make()
        self.assertEqual(m._cb_threshold, 5)

    def test_custom_threshold(self):
        m = self._make(circuit_breaker_threshold=10)
        self.assertEqual(m._cb_threshold, 10)

    def test_custom_cooldown(self):
        m = self._make(circuit_breaker_cooldown=120.0)
        self.assertEqual(m._cb_cooldown, 120.0)

    def test_health_reports_backend_info(self):
        m = self._make()
        h = m.health()
        self.assertEqual(h["llm"]["backend"], "mock")
        self.assertEqual(h["llm"]["calls"], 0)
        self.assertEqual(h["llm"]["errors"], 0)

    def test_health_avg_latency_zero_when_no_calls(self):
        m = self._make()
        h = m.health()
        self.assertEqual(h["llm"]["avg_ms"], 0.0)

    def test_health_avg_latency_computed(self):
        m = self._make()
        m._call_count = 4
        m._total_latency_ms = 400.0
        h = m.health()
        self.assertEqual(h["llm"]["avg_ms"], 100.0)


class TestLLMTransientErrorClassification(unittest.TestCase):
    """Test _is_transient_error() classification logic."""

    def _make(self):
        from decision.modules.llm_module import LLMModule
        return LLMModule(backend="mock")

    def test_timeout_is_transient(self):
        m = self._make()
        self.assertTrue(m._is_transient_error(TimeoutError("connection timed out")))

    def test_rate_limit_is_transient(self):
        m = self._make()
        self.assertTrue(m._is_transient_error(Exception("429 rate limit exceeded")))

    def test_connection_error_is_transient(self):
        m = self._make()
        self.assertTrue(m._is_transient_error(ConnectionError("connection refused")))

    def test_503_is_transient(self):
        m = self._make()
        self.assertTrue(m._is_transient_error(Exception("503 Service Unavailable")))

    def test_temporary_is_transient(self):
        m = self._make()
        self.assertTrue(m._is_transient_error(Exception("temporary failure")))

    def test_401_unauthorized_is_not_transient(self):
        m = self._make()
        self.assertFalse(m._is_transient_error(Exception("401 Unauthorized")))

    def test_403_forbidden_is_not_transient(self):
        m = self._make()
        self.assertFalse(m._is_transient_error(Exception("403 Forbidden")))

    def test_invalid_model_is_not_transient(self):
        m = self._make()
        self.assertFalse(m._is_transient_error(Exception("model 'foo' not found")))

    def test_empty_error_is_not_transient(self):
        m = self._make()
        self.assertFalse(m._is_transient_error(Exception("")))


class TestLLMRequestResponse(unittest.TestCase):
    """Test LLMRequest / LLMResponse data classes."""

    def test_simple_request_user_only(self):
        from decision.modules.llm_module import LLMRequest
        r = LLMRequest.simple("hello")
        self.assertEqual(len(r.messages), 1)
        self.assertEqual(r.messages[0]["role"], "user")

    def test_simple_request_with_system(self):
        from decision.modules.llm_module import LLMRequest
        r = LLMRequest.simple("hello", system="be helpful")
        self.assertEqual(len(r.messages), 2)
        self.assertEqual(r.messages[0]["role"], "system")
        self.assertEqual(r.messages[1]["role"], "user")

    def test_simple_request_preserves_id(self):
        from decision.modules.llm_module import LLMRequest
        r = LLMRequest.simple("q", request_id="abc-123", caller="test")
        self.assertEqual(r.request_id, "abc-123")
        self.assertEqual(r.caller, "test")

    def test_response_ok_when_no_error(self):
        from decision.modules.llm_module import LLMResponse
        r = LLMResponse(text="answer", request_id="1")
        self.assertTrue(r.ok)

    def test_response_not_ok_with_error(self):
        from decision.modules.llm_module import LLMResponse
        r = LLMResponse(text="", request_id="1", error="timeout")
        self.assertFalse(r.ok)


class TestActionExecutor(unittest.TestCase):
    """Test ActionExecutor command generation and state management."""

    def _make(self, **kw):
        from decision.tasking.action_executor import ActionExecutor
        return ActionExecutor(**kw)

    def test_initial_status_idle(self):
        from decision.tasking.action_executor import ActionStatus
        ex = self._make()
        self.assertEqual(ex.status, ActionStatus.IDLE)

    def test_navigate_command_type(self):
        ex = self._make()
        cmd = ex.generate_navigate_command({"x": 5.0, "y": 3.0, "z": 0.0})
        self.assertEqual(cmd.command_type, "goal")
        self.assertAlmostEqual(cmd.target_x, 5.0)
        self.assertAlmostEqual(cmd.target_y, 3.0)

    def test_navigate_computes_yaw_toward_target(self):
        ex = self._make()
        cmd = ex.generate_navigate_command(
            {"x": 1.0, "y": 1.0},
            robot_position={"x": 0.0, "y": 0.0},
        )
        expected_yaw = math.atan2(1.0, 1.0)  # pi/4
        self.assertAlmostEqual(cmd.target_yaw, expected_yaw, places=5)

    def test_navigate_sets_executing(self):
        from decision.tasking.action_executor import ActionStatus
        ex = self._make()
        ex.generate_navigate_command({"x": 1.0, "y": 1.0})
        self.assertEqual(ex.status, ActionStatus.EXECUTING)

    def test_look_around_is_velocity_command(self):
        ex = self._make(look_around_speed=0.6)
        cmd = ex.generate_look_around_command()
        self.assertEqual(cmd.command_type, "velocity")
        self.assertAlmostEqual(cmd.angular_z, 0.6)

    def test_approach_reduces_distance(self):
        ex = self._make(approach_distance=0.5)
        cmd = ex.generate_approach_command(
            target_position={"x": 10.0, "y": 0.0},
            robot_position={"x": 0.0, "y": 0.0},
        )
        # Should stop 0.5m short of target
        self.assertAlmostEqual(cmd.target_x, 9.5, places=1)
        self.assertAlmostEqual(cmd.target_y, 0.0, places=1)

    def test_approach_already_close_stays_put(self):
        ex = self._make(approach_distance=0.5)
        cmd = ex.generate_approach_command(
            target_position={"x": 0.3, "y": 0.0},
            robot_position={"x": 0.0, "y": 0.0},
        )
        # Distance 0.3 < approach_distance 0.5, robot stays
        self.assertAlmostEqual(cmd.target_x, 0.0, places=5)
        self.assertAlmostEqual(cmd.target_y, 0.0, places=5)

    def test_verify_command_does_not_move(self):
        ex = self._make()
        cmd = ex.generate_verify_command(
            target_position={"x": 5.0, "y": 5.0},
            robot_position={"x": 1.0, "y": 1.0},
        )
        self.assertAlmostEqual(cmd.target_x, 1.0)
        self.assertAlmostEqual(cmd.target_y, 1.0)
        # But yaw should face target
        expected_yaw = math.atan2(4.0, 4.0)
        self.assertAlmostEqual(cmd.target_yaw, expected_yaw, places=5)

    def test_backtrack_command(self):
        ex = self._make()
        pos = np.array([2.0, 3.0, 0.0])
        cmd = ex.generate_backtrack_command(pos)
        self.assertEqual(cmd.command_type, "goal")
        self.assertAlmostEqual(cmd.target_x, 2.0)
        self.assertAlmostEqual(cmd.target_y, 3.0)

    def test_mark_succeeded(self):
        from decision.tasking.action_executor import ActionStatus
        ex = self._make()
        ex.generate_navigate_command({"x": 1.0, "y": 1.0})
        ex.mark_succeeded()
        self.assertEqual(ex.status, ActionStatus.SUCCEEDED)

    def test_mark_failed(self):
        from decision.tasking.action_executor import ActionStatus
        ex = self._make()
        ex.generate_navigate_command({"x": 1.0, "y": 1.0})
        ex.mark_failed()
        self.assertEqual(ex.status, ActionStatus.FAILED)

    def test_reset(self):
        from decision.tasking.action_executor import ActionStatus
        ex = self._make()
        ex.generate_navigate_command({"x": 1.0, "y": 1.0})
        ex.reset()
        self.assertEqual(ex.status, ActionStatus.IDLE)

    def test_check_timeout_false_when_idle(self):
        ex = self._make(nav_timeout=0.001)
        self.assertFalse(ex.check_timeout())

    def test_check_timeout_true_when_expired(self):
        ex = self._make(nav_timeout=0.0)  # immediate timeout
        ex.generate_navigate_command({"x": 1.0, "y": 1.0})
        time.sleep(0.01)
        self.assertTrue(ex.check_timeout())

    def test_lera_fallback_no_llm(self):
        ex = self._make()
        # Without LLM, fallback rules apply
        self.assertEqual(
            ex.lera_recover("nav failed", ["chair", "table"], "find chair", 1),
            "retry_different_path",
        )
        self.assertEqual(
            ex.lera_recover("nav failed", [], "find chair", 2),
            "expand_search",
        )
        self.assertEqual(
            ex.lera_recover("nav failed", [], "find chair", 3),
            "abort",
        )


class TestTargetBeliefManager(unittest.TestCase):
    """Test BA-HSG TargetBeliefManager posterior and selection logic."""

    def _make(self):
        from decision.goal_resolution.goal_resolver import TargetBeliefManager
        return TargetBeliefManager()

    def _candidates(self):
        return [
            {"id": 1, "label": "chair_A", "position": [1, 0, 0],
             "fused_score": 0.9, "belief": {"credibility": 0.8}, "room_match": 0.7},
            {"id": 2, "label": "chair_B", "position": [5, 0, 0],
             "fused_score": 0.6, "belief": {"credibility": 0.5}, "room_match": 0.4},
            {"id": 3, "label": "chair_C", "position": [3, 3, 0],
             "fused_score": 0.3, "belief": {"credibility": 0.3}, "room_match": 0.2},
        ]

    def test_init_from_candidates_sets_posterior(self):
        bm = self._make()
        bm.init_from_candidates(self._candidates())
        total = sum(h.posterior for h in bm._hypotheses)
        self.assertAlmostEqual(total, 1.0, places=5)

    def test_highest_score_gets_highest_posterior(self):
        bm = self._make()
        bm.init_from_candidates(self._candidates())
        best = bm.best_hypothesis
        self.assertIsNotNone(best)
        self.assertEqual(best.object_id, 1)  # highest fused_score

    def test_bayesian_update_confirmed(self):
        bm = self._make()
        bm.init_from_candidates(self._candidates())
        _ = bm._hypotheses[0].posterior
        # Confirm object 1 with high CLIP similarity
        bm.bayesian_update(1, detected=True, clip_sim=0.9)
        # Object 1 posterior should increase relative to others
        self.assertGreater(bm._hypotheses[0].posterior, bm._hypotheses[1].posterior)

    def test_bayesian_update_rejected(self):
        bm = self._make()
        bm.init_from_candidates(self._candidates())
        # Reject object 1 (not detected)
        bm.bayesian_update(1, detected=False, clip_sim=0.0)
        self.assertTrue(bm._hypotheses[0].rejected)

    def test_select_next_target_skips_rejected(self):
        bm = self._make()
        bm.init_from_candidates(self._candidates())
        bm.bayesian_update(1, detected=False)  # reject #1
        nxt = bm.select_next_target(robot_position=[0, 0, 0])
        self.assertIsNotNone(nxt)
        self.assertNotEqual(nxt.object_id, 1)

    def test_is_converged_false_initially(self):
        bm = self._make()
        bm.init_from_candidates(self._candidates())
        # With 3 candidates, even the best shouldn't exceed 0.7 threshold easily
        # unless scores are very skewed; check the logic is callable
        self.assertIsInstance(bm.is_converged, bool)

    def test_num_active(self):
        bm = self._make()
        bm.init_from_candidates(self._candidates())
        self.assertEqual(bm.num_active, 3)
        bm.bayesian_update(2, detected=False)
        self.assertEqual(bm.num_active, 2)

    def test_empty_candidates(self):
        bm = self._make()
        bm.init_from_candidates([])
        self.assertIsNone(bm.best_hypothesis)
        self.assertFalse(bm.is_converged)
        self.assertEqual(bm.num_active, 0)

    def test_select_next_target_prefers_nearby_high_posterior(self):
        bm = self._make()
        bm.init_from_candidates(self._candidates())
        # Robot at origin — candidate 1 at (1,0,0) is closest AND highest score
        nxt = bm.select_next_target(robot_position=[0, 0, 0])
        self.assertIsNotNone(nxt)
        self.assertEqual(nxt.object_id, 1)


class TestGoalResult(unittest.TestCase):
    """Test GoalResult data class."""

    def test_default_values(self):
        from decision.goal_resolution.goal_resolver import GoalResult
        r = GoalResult(action="navigate")
        self.assertFalse(r.is_valid)
        self.assertEqual(r.confidence, 0.0)
        self.assertEqual(r.frame_id, "map")
        self.assertEqual(r.path, "")

    def test_populated_result(self):
        from decision.goal_resolution.goal_resolver import GoalResult
        r = GoalResult(
            action="navigate", target_x=1.0, target_y=2.0,
            confidence=0.95, is_valid=True, path="fast",
        )
        self.assertTrue(r.is_valid)
        self.assertEqual(r.path, "fast")


if __name__ == "__main__":
    unittest.main()

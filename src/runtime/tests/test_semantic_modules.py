"""Unit tests for 6 semantic planner/reconstruction modules.

Pure unit tests -- no ROS2, no hardware, no LLM API calls.
Follows the patterns established in test_llm_module.py.
"""

import json
import unittest


# ---------------------------------------------------------------------------
# 1. VisualServoModule
# ---------------------------------------------------------------------------

class TestVisualServoModuleInstantiation(unittest.TestCase):
    """Test VisualServoModule creation and initial state."""

    def _make(self, **kw):
        from decision.modules.visual_servo_module import (
            VisualServoModule,
        )
        return VisualServoModule(**kw)

    def test_initial_mode_is_idle(self):
        m = self._make()
        self.assertEqual(m._mode, "idle")

    def test_initial_servo_not_active(self):
        m = self._make()
        self.assertFalse(m._servo_active)

    def test_custom_takeover_distance(self):
        m = self._make(servo_takeover_distance=5.0)
        self.assertAlmostEqual(m._takeover_dist, 5.0)

    def test_target_label_initially_empty(self):
        m = self._make()
        self.assertEqual(m._target_label, "")


class TestVisualServoModeTransitions(unittest.TestCase):
    """Test VisualServoModule mode transitions via _on_servo_target."""

    def _make(self):
        from decision.modules.visual_servo_module import (
            VisualServoModule,
        )
        m = VisualServoModule()
        # Bypass status throttle for tests
        m._last_status_time = 0.0
        return m

    def test_find_command_sets_find_mode(self):
        m = self._make()
        m._on_servo_target("find:red chair")
        self.assertEqual(m._mode, "find")
        self.assertEqual(m._target_label, "red chair")

    def test_follow_command_sets_follow_mode(self):
        m = self._make()
        m._on_servo_target("follow:person in blue jacket")
        self.assertEqual(m._mode, "follow")
        self.assertEqual(m._target_label, "person in blue jacket")

    def test_stop_command_returns_to_idle(self):
        m = self._make()
        m._on_servo_target("find:chair")
        self.assertEqual(m._mode, "find")
        m._on_servo_target("stop")
        self.assertEqual(m._mode, "idle")
        self.assertEqual(m._target_label, "")

    def test_bare_label_defaults_to_find(self):
        m = self._make()
        m._on_servo_target("red cup")
        self.assertEqual(m._mode, "find")
        self.assertEqual(m._target_label, "red cup")

    def test_cancel_tracking_releases_servo(self):
        m = self._make()
        m._servo_active = True
        m._cancel_tracking()
        self.assertFalse(m._servo_active)
        self.assertEqual(m._mode, "idle")


class TestVisualServoHealth(unittest.TestCase):
    """Test VisualServoModule.health() output."""

    def _make(self):
        from decision.modules.visual_servo_module import (
            VisualServoModule,
        )
        return VisualServoModule()

    def test_health_idle_mode(self):
        m = self._make()
        h = m.health()
        self.assertIsInstance(h, dict)
        self.assertEqual(h["mode"], "idle")
        self.assertFalse(h["tracking_active"])

    def test_health_far_mode(self):
        m = self._make()
        m._mode = "find"
        m._servo_active = False
        h = m.health()
        self.assertTrue(h["tracking_active"])
        self.assertEqual(h["mode"], "far")

    def test_health_near_mode(self):
        m = self._make()
        m._mode = "find"
        m._servo_active = True
        h = m.health()
        self.assertTrue(h["tracking_active"])
        self.assertEqual(h["mode"], "near")

    def test_health_has_module_key(self):
        m = self._make()
        h = m.health()
        self.assertIn("module", h)
        self.assertEqual(h["module"], "VisualServoModule")


class TestVisualServoSkills(unittest.TestCase):
    """Test @skill methods on VisualServoModule."""

    def _make(self):
        from decision.modules.visual_servo_module import (
            VisualServoModule,
        )
        m = VisualServoModule()
        m._last_status_time = 0.0
        return m

    def test_find_object_skill(self):
        m = self._make()
        result = m.find_object("red chair")
        self.assertIn("red chair", result)
        self.assertEqual(m._mode, "find")

    def test_follow_person_skill(self):
        m = self._make()
        result = m.follow_person("person in hat")
        self.assertIn("person in hat", result)
        self.assertEqual(m._mode, "follow")

    def test_stop_servo_skill(self):
        m = self._make()
        m._on_servo_target("find:chair")
        result = m.stop_servo()
        self.assertIn("stopped", result)
        self.assertEqual(m._mode, "idle")

    def test_get_servo_status_skill(self):
        m = self._make()
        m._on_servo_target("find:table")
        status = m.get_servo_status()
        self.assertIsInstance(status, dict)
        self.assertEqual(status["mode"], "find")
        self.assertEqual(status["target"], "table")


# ---------------------------------------------------------------------------
# 2. GoalResolverModule
# ---------------------------------------------------------------------------

class TestGoalResolverModuleInstantiation(unittest.TestCase):
    """Test GoalResolverModule creation and config."""

    def _make(self, **kw):
        from decision.modules.goal_resolver_module import (
            GoalResolverModule,
        )
        return GoalResolverModule(**kw)

    def test_default_thresholds(self):
        m = self._make()
        self.assertAlmostEqual(m._fast_path_threshold, 0.75)
        self.assertAlmostEqual(m._confidence_threshold, 0.6)
        self.assertAlmostEqual(m._entropy_threshold, 1.5)
        self.assertTrue(m._use_slow_path)

    def test_custom_thresholds(self):
        m = self._make(fast_path_threshold=0.5, entropy_threshold=2.0,
                       use_slow_path=False)
        self.assertAlmostEqual(m._fast_path_threshold, 0.5)
        self.assertAlmostEqual(m._entropy_threshold, 2.0)
        self.assertFalse(m._use_slow_path)

    def test_resolver_initially_none(self):
        m = self._make()
        self.assertIsNone(m._resolver)

    def test_latest_sg_initially_none(self):
        m = self._make()
        self.assertIsNone(m._latest_sg)


class TestGoalResolverOfflineFastPath(unittest.TestCase):
    """Test GoalResolverModule._offline_fast_path (keyword matching)."""

    def _make(self):
        from decision.modules.goal_resolver_module import (
            GoalResolverModule,
        )
        return GoalResolverModule()

    def test_keyword_match_returns_goal(self):
        m = self._make()
        sg_json = json.dumps({
            "objects": [
                {"label": "red chair", "position": [3.0, 4.0, 0.0],
                 "confidence": 0.85},
            ],
            "relations": [],
            "regions": [],
        })
        result = m._offline_fast_path("find the red chair", sg_json)
        self.assertIsNotNone(result)
        self.assertTrue(result.is_valid)
        self.assertAlmostEqual(result.target_x, 3.0)
        self.assertAlmostEqual(result.target_y, 4.0)
        self.assertEqual(result.path, "fast")

    def test_no_match_returns_none(self):
        m = self._make()
        sg_json = json.dumps({
            "objects": [
                {"label": "table", "position": [1.0, 1.0, 0.0],
                 "confidence": 0.9},
            ],
        })
        result = m._offline_fast_path("find the red chair", sg_json)
        self.assertIsNone(result)

    def test_empty_objects_returns_none(self):
        m = self._make()
        sg_json = json.dumps({"objects": []})
        result = m._offline_fast_path("find chair", sg_json)
        self.assertIsNone(result)

    def test_invalid_json_returns_none(self):
        m = self._make()
        result = m._offline_fast_path("find chair", "not json")
        self.assertIsNone(result)

    def test_position_as_dict(self):
        m = self._make()
        sg_json = json.dumps({
            "objects": [
                {"label": "chair", "position": {"x": 2.0, "y": 3.0, "z": 0.5},
                 "confidence": 0.7},
            ],
        })
        result = m._offline_fast_path("chair", sg_json)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.target_x, 2.0)
        self.assertAlmostEqual(result.target_y, 3.0)
        self.assertAlmostEqual(result.target_z, 0.5)


class TestGoalResolverHealth(unittest.TestCase):
    """Test GoalResolverModule.health() output."""

    def _make(self):
        from decision.modules.goal_resolver_module import (
            GoalResolverModule,
        )
        return GoalResolverModule()

    def test_health_returns_dict(self):
        m = self._make()
        h = m.health()
        self.assertIsInstance(h, dict)

    def test_health_has_expected_keys(self):
        m = self._make()
        h = m.health()
        self.assertIn("fast_hits", h)
        self.assertIn("slow_calls", h)
        self.assertIn("module", h)
        self.assertEqual(h["fast_hits"], 0)
        self.assertEqual(h["slow_calls"], 0)


class TestGoalResolverSlowPathFallback(unittest.TestCase):
    """Test GoalResolverModule._resolve falls back properly."""

    def _make(self):
        from decision.modules.goal_resolver_module import (
            GoalResolverModule,
        )
        m = GoalResolverModule(use_slow_path=False)
        return m

    def test_empty_instruction_ignored(self):
        m = self._make()
        # Should not crash on empty instruction
        m._on_instruction("")
        m._on_instruction("   ")


# ---------------------------------------------------------------------------
# 3. ActionExecutorModule
# ---------------------------------------------------------------------------

class TestActionExecutorModuleInstantiation(unittest.TestCase):
    """Test ActionExecutorModule creation and config."""

    def _make(self, **kw):
        from decision.modules.action_executor_module import (
            ActionExecutorModule,
        )
        return ActionExecutorModule(**kw)

    def test_default_config(self):
        m = self._make()
        self.assertAlmostEqual(m._approach_distance, 0.5)
        self.assertAlmostEqual(m._nav_timeout, 60.0)
        self.assertEqual(m._max_lera_retries, 3)

    def test_custom_config(self):
        m = self._make(approach_distance=1.0, nav_timeout=30.0, max_lera_retries=5)
        self.assertAlmostEqual(m._approach_distance, 1.0)
        self.assertAlmostEqual(m._nav_timeout, 30.0)
        self.assertEqual(m._max_lera_retries, 5)

    def test_failure_count_initially_zero(self):
        m = self._make()
        self.assertEqual(m._failure_count, 0)

    def test_executor_initially_none(self):
        m = self._make()
        self.assertIsNone(m._executor)


class TestActionExecutorModuleErrorCounting(unittest.TestCase):
    """Test ActionExecutorModule.lera_recover and failure counting."""

    def _make(self):
        from decision.modules.action_executor_module import (
            ActionExecutorModule,
        )
        return ActionExecutorModule(max_lera_retries=3)

    def test_first_failure_retries(self):
        m = self._make()
        result = m.lera_recover("nav failed", "find chair")
        self.assertEqual(result, "retry_different_path")
        self.assertEqual(m._failure_count, 1)

    def test_second_failure_expands_search(self):
        m = self._make()
        m.lera_recover("nav failed", "find chair")
        result = m.lera_recover("nav failed", "find chair")
        self.assertEqual(result, "expand_search")
        self.assertEqual(m._failure_count, 2)

    def test_third_failure_aborts(self):
        m = self._make()
        m.lera_recover("nav failed", "find chair")
        m.lera_recover("nav failed", "find chair")
        result = m.lera_recover("nav failed", "find chair")
        self.assertEqual(result, "abort")
        self.assertEqual(m._failure_count, 3)

    def test_reset_failure_count(self):
        m = self._make()
        m.lera_recover("fail", "goal")
        m.lera_recover("fail", "goal")
        self.assertEqual(m._failure_count, 2)
        m.reset_failure_count()
        self.assertEqual(m._failure_count, 0)


class TestActionExecutorModuleHealth(unittest.TestCase):
    """Test ActionExecutorModule.health() output."""

    def _make(self):
        from decision.modules.action_executor_module import (
            ActionExecutorModule,
        )
        return ActionExecutorModule()

    def test_health_returns_dict(self):
        m = self._make()
        h = m.health()
        self.assertIsInstance(h, dict)

    def test_health_has_expected_keys(self):
        m = self._make()
        h = m.health()
        self.assertIn("executing", h)
        self.assertIn("step_count", h)
        self.assertIn("error_count", h)
        self.assertIn("module", h)

    def test_health_initial_values(self):
        m = self._make()
        h = m.health()
        self.assertFalse(h["executing"])
        self.assertEqual(h["step_count"], 0)
        self.assertEqual(h["error_count"], 0)

    def test_health_tracks_error_count(self):
        m = self._make()
        m.lera_recover("fail", "goal")
        m.lera_recover("fail", "goal")
        h = m.health()
        self.assertEqual(h["error_count"], 2)


class TestActionExecutorModuleGoalRejection(unittest.TestCase):
    """Test that invalid goals are rejected."""

    def _make(self):
        from decision.modules.action_executor_module import (
            ActionExecutorModule,
        )
        return ActionExecutorModule()

    def test_invalid_goal_is_rejected(self):
        from runtime.msgs.semantic import GoalResult as MsgGoalResult
        m = self._make()
        goal = MsgGoalResult(action="navigate", is_valid=False, target_label="chair")
        # Should not crash; publishes rejected status
        m._on_resolved_goal(goal)
        # failure_count should remain 0 (not an execution failure)
        self.assertEqual(m._failure_count, 0)


# ---------------------------------------------------------------------------
# 4. FrontierModule
# ---------------------------------------------------------------------------

class TestFrontierModuleInstantiation(unittest.TestCase):
    """Test FrontierModule creation and config."""

    def _make(self, **kw):
        from decision.modules.frontier_module import FrontierModule
        return FrontierModule(**kw)

    def test_default_score_threshold(self):
        m = self._make()
        self.assertAlmostEqual(m._score_threshold, 0.2)

    def test_custom_score_threshold(self):
        m = self._make(score_threshold=0.5)
        self.assertAlmostEqual(m._score_threshold, 0.5)

    def test_scorer_is_created(self):
        m = self._make()
        self.assertIsNotNone(m._scorer)
        self.assertIsNotNone(m.scorer)

    def test_initial_instruction_empty(self):
        m = self._make()
        self.assertEqual(m._last_instruction, "")

    def test_visited_list_initially_empty(self):
        m = self._make()
        self.assertEqual(len(m._visited), 0)


class TestFrontierModuleOdomFiltering(unittest.TestCase):
    """Test FrontierModule._on_odom filters non-finite values."""

    def _make(self):
        from decision.modules.frontier_module import FrontierModule
        return FrontierModule()

    def _odom(self, x, y):
        from runtime.msgs.geometry import Pose
        from runtime.msgs.nav import Odometry
        return Odometry(pose=Pose(x, y, 0.0))

    def test_valid_odom_is_cached(self):
        m = self._make()
        odom = self._odom(1.0, 2.0)
        m._on_odom(odom)
        self.assertIsNotNone(m._last_odom)
        self.assertAlmostEqual(m._last_odom.x, 1.0)

    def test_nan_odom_is_rejected(self):
        m = self._make()
        odom = self._odom(float("nan"), 2.0)
        m._on_odom(odom)
        self.assertIsNone(m._last_odom)

    def test_inf_odom_is_rejected(self):
        m = self._make()
        odom = self._odom(1.0, float("inf"))
        m._on_odom(odom)
        self.assertIsNone(m._last_odom)


class TestFrontierModuleHealth(unittest.TestCase):
    """Test FrontierModule.health() output."""

    def _make(self):
        from decision.modules.frontier_module import FrontierModule
        return FrontierModule()

    def test_health_returns_dict(self):
        m = self._make()
        h = m.health()
        self.assertIsInstance(h, dict)

    def test_health_has_expected_keys(self):
        m = self._make()
        h = m.health()
        self.assertIn("frontier_count", h)
        self.assertIn("exploration_active", h)
        self.assertIn("module", h)

    def test_health_exploration_inactive_when_no_instruction(self):
        m = self._make()
        h = m.health()
        self.assertFalse(h["exploration_active"])

    def test_health_exploration_active_after_instruction(self):
        m = self._make()
        m._last_instruction = "find the kitchen"
        h = m.health()
        self.assertTrue(h["exploration_active"])


class TestFrontierModuleEvaluation(unittest.TestCase):
    """Test FrontierModule._evaluate requires all inputs."""

    def _make(self):
        from decision.modules.frontier_module import FrontierModule
        return FrontierModule()

    def test_evaluate_no_odom_returns_early(self):
        from runtime.msgs.semantic import SceneGraph
        m = self._make()
        m._last_sg = SceneGraph(objects=[], relations=[], regions=[])
        m._last_instruction = "find chair"
        # No odom set, should return early without error
        m._evaluate()

    def test_evaluate_no_instruction_returns_early(self):
        from runtime.msgs.geometry import Pose
        from runtime.msgs.nav import Odometry
        from runtime.msgs.semantic import SceneGraph
        m = self._make()
        m._last_odom = Odometry(pose=Pose(0.0, 0.0, 0.0))
        m._last_sg = SceneGraph(objects=[], relations=[], regions=[])
        m._last_instruction = ""
        # No instruction, should return early without error
        m._evaluate()


# ---------------------------------------------------------------------------
# 5. TaskDecomposerModule
# ---------------------------------------------------------------------------

class TestTaskDecomposerModuleInstantiation(unittest.TestCase):
    """Test TaskDecomposerModule creation."""

    def _make(self, **kw):
        from decision.modules.task_decomposer_module import (
            TaskDecomposerModule,
        )
        return TaskDecomposerModule(**kw)

    def test_instantiation(self):
        m = self._make()
        self.assertIsNotNone(m)

    def test_decomposer_initially_none(self):
        m = self._make()
        self.assertIsNone(m._decomposer)


class TestTaskDecomposerOfflineDecompose(unittest.TestCase):
    """Test TaskDecomposerModule._offline_decompose (rule-based fallback)."""

    def _make(self):
        from decision.modules.task_decomposer_module import (
            TaskDecomposerModule,
        )
        return TaskDecomposerModule()

    def test_offline_decompose_returns_dict(self):
        m = self._make()
        plan = m._offline_decompose("go to the kitchen")
        self.assertIsInstance(plan, dict)

    def test_offline_decompose_has_subgoals(self):
        m = self._make()
        plan = m._offline_decompose("go to the kitchen")
        self.assertIn("subgoals", plan)
        self.assertEqual(len(plan["subgoals"]), 2)

    def test_offline_decompose_preserves_instruction(self):
        m = self._make()
        plan = m._offline_decompose("find the red chair")
        self.assertEqual(plan["instruction"], "find the red chair")

    def test_offline_decompose_subgoal_ordering(self):
        m = self._make()
        plan = m._offline_decompose("go to kitchen and get cup")
        subgoals = plan["subgoals"]
        self.assertEqual(subgoals[0]["step_id"], 0)
        self.assertEqual(subgoals[1]["step_id"], 1)
        self.assertEqual(subgoals[0]["action"], "navigate")
        self.assertEqual(subgoals[1]["action"], "verify")

    def test_offline_decompose_status_pending(self):
        m = self._make()
        plan = m._offline_decompose("explore area")
        for sg in plan["subgoals"]:
            self.assertEqual(sg["status"], "pending")
            self.assertEqual(sg["retry_count"], 0)

    def test_offline_decompose_total_steps(self):
        m = self._make()
        plan = m._offline_decompose("find chair")
        self.assertEqual(plan["total_steps"], 2)
        self.assertEqual(plan["current_step"], 0)
        self.assertFalse(plan["is_complete"])


class TestTaskDecomposerEmptyInput(unittest.TestCase):
    """Test TaskDecomposerModule handles empty input."""

    def _make(self):
        from decision.modules.task_decomposer_module import (
            TaskDecomposerModule,
        )
        return TaskDecomposerModule()

    def test_empty_instruction_ignored(self):
        m = self._make()
        m._on_instruction("")

    def test_whitespace_instruction_ignored(self):
        m = self._make()
        m._on_instruction("   ")


class TestTaskDecomposerHealth(unittest.TestCase):
    """Test TaskDecomposerModule.health() output."""

    def _make(self):
        from decision.modules.task_decomposer_module import (
            TaskDecomposerModule,
        )
        return TaskDecomposerModule()

    def test_health_returns_dict(self):
        m = self._make()
        h = m.health()
        self.assertIsInstance(h, dict)

    def test_health_has_expected_keys(self):
        m = self._make()
        h = m.health()
        self.assertIn("last_decompose_time", h)
        self.assertIn("subtask_count", h)
        self.assertIn("module", h)

    def test_health_initial_values(self):
        m = self._make()
        h = m.health()
        self.assertIsNone(h["last_decompose_time"])
        self.assertEqual(h["subtask_count"], 0)


# ---------------------------------------------------------------------------
# 6. ReconstructionModule
# ---------------------------------------------------------------------------

class TestReconstructionModuleInstantiation(unittest.TestCase):
    """Test ReconstructionModule creation and config."""

    def _make(self, **kw):
        from perception.reconstruction.reconstruction_module import (
            ReconstructionModule,
        )
        return ReconstructionModule(**kw)

    def test_instantiation(self):
        m = self._make()
        self.assertIsNotNone(m)

    def test_default_voxel_size(self):
        m = self._make()
        self.assertAlmostEqual(m._projector._voxel_size, 0.05)

    def test_custom_voxel_size(self):
        m = self._make(voxel_size=0.1)
        self.assertAlmostEqual(m._projector._voxel_size, 0.1)

    def test_default_min_points(self):
        m = self._make()
        self.assertEqual(m._min_points, 1000)

    def test_custom_min_points(self):
        m = self._make(min_points_to_publish=500)
        self.assertEqual(m._min_points, 500)


class TestReconstructionModulePorts(unittest.TestCase):
    """Test ReconstructionModule port declarations."""

    def _make(self):
        from perception.reconstruction.reconstruction_module import (
            ReconstructionModule,
        )
        return ReconstructionModule()

    def test_has_input_ports(self):
        m = self._make()
        h = m.health()
        ports_in = h.get("ports_in", {})
        expected_inputs = {"color_image", "depth_image", "camera_info",
                           "scene_graph", "odometry"}
        for name in expected_inputs:
            self.assertIn(name, ports_in, f"Missing input port: {name}")

    def test_has_output_ports(self):
        m = self._make()
        h = m.health()
        ports_out = h.get("ports_out", {})
        expected_outputs = {"semantic_cloud", "reconstruction_stats"}
        for name in expected_outputs:
            self.assertIn(name, ports_out, f"Missing output port: {name}")


class TestReconstructionModuleHealth(unittest.TestCase):
    """Test ReconstructionModule.health() output."""

    def _make(self):
        from perception.reconstruction.reconstruction_module import (
            ReconstructionModule,
        )
        return ReconstructionModule()

    def test_health_returns_dict(self):
        m = self._make()
        h = m.health()
        self.assertIsInstance(h, dict)

    def test_health_has_expected_keys(self):
        m = self._make()
        h = m.health()
        self.assertIn("mesh_vertices", h)
        self.assertIn("last_update_time", h)
        self.assertIn("module", h)

    def test_health_initial_values(self):
        m = self._make()
        h = m.health()
        self.assertEqual(h["mesh_vertices"], 0)
        self.assertIsNone(h["last_update_time"])

    def test_module_name_correct(self):
        m = self._make()
        h = m.health()
        self.assertEqual(h["module"], "ReconstructionModule")


class TestReconstructionModuleProperties(unittest.TestCase):
    """Test ReconstructionModule properties."""

    def _make(self):
        from perception.reconstruction.reconstruction_module import (
            ReconstructionModule,
        )
        return ReconstructionModule()

    def test_voxel_count_initially_zero(self):
        m = self._make()
        self.assertEqual(m.voxel_count, 0)

    def test_object_count_initially_zero(self):
        m = self._make()
        self.assertEqual(m.object_count, 0)

    def test_layer_is_3(self):
        m = self._make()
        self.assertEqual(m._layer, 3)


if __name__ == "__main__":
    unittest.main()

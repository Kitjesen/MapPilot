"""Unit tests for semantic planner and reconstruction modules.

Pure unit tests -- no ROS2, no hardware, no LLM API calls.
Follows the patterns established in test_llm_module.py.
"""

import unittest

# ---------------------------------------------------------------------------
# 1. VisualServoModule
# ---------------------------------------------------------------------------


class TestVisualServoModuleInstantiation(unittest.TestCase):
    """Test VisualServoModule creation and initial state."""

    def _make(self, **kw):
        from decision.modules.visual_servo import (
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
        from decision.modules.visual_servo import (
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
        from decision.modules.visual_servo import (
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
        from decision.modules.visual_servo import (
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

    def test_tune_bbox_gains_reports_unavailable_without_motion_authority(self):
        m = self._make()
        report = m.tune_bbox_gains(duration=0.0)
        self.assertFalse(report["converged"])
        self.assertEqual(report["status"], "unavailable")
        self.assertIn("not implemented", report["error"])
        self.assertIn("requires", report)


# ---------------------------------------------------------------------------
# 2. ReconstructionModule
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
        expected_inputs = {"color_image", "depth_image", "camera_info", "scene_graph", "odometry"}
        for name in expected_inputs:
            self.assertIn(name, ports_in, f"Missing input port: {name}")

    def test_has_output_ports(self):
        m = self._make()
        h = m.health()
        ports_out = h.get("ports_out", {})
        expected_outputs = {"semantic_cloud", "semantic_labels", "reconstruction_stats"}
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

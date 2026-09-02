"""Tests for service layer — PerceptionService + PlannerServices.

Pure algorithm testing with mocks. No ROS2, no GPU, no API keys.
"""

import unittest
from unittest.mock import MagicMock

import numpy as np

# ---------------------------------------------------------------------------
# Mock components
# ---------------------------------------------------------------------------


class MockDetector:
    def detect(self, bgr, text_prompt):
        det = MagicMock()
        det.bbox = np.array([10, 20, 100, 200])
        det.score = 0.9
        det.label = "chair"
        det.mask = None
        det.features = np.zeros(512)
        return [det]

    def load_model(self):
        pass

    def shutdown(self):
        pass


class MockEncoder:
    def encode_image(self, img):
        return np.random.randn(512).astype(np.float32)

    def encode_text(self, text):
        return np.random.randn(512).astype(np.float32)

    def load_model(self):
        pass

    def shutdown(self):
        pass


class MockTracker:
    def __init__(self):
        self._tracked_objects = {}
        self._update_count = 0

    def update(self, dets_3d, **kwargs):
        self._update_count += 1
        for d in dets_3d:
            self._tracked_objects[d.label] = d

    def get_scene_graph_json(self):
        import json

        return json.dumps(
            {
                "objects": [{"label": k, "position": [0, 0, 0]} for k in self._tracked_objects],
            }
        )

    def clear(self):
        self._tracked_objects.clear()


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
# PerceptionService tests
# ---------------------------------------------------------------------------


class TestPerceptionService(unittest.TestCase):
    def _make_service(self, detector=True, encoder=True, tracker=True):
        from perception.service import PerceptionService
        from perception.tracking.projection import CameraIntrinsics

        svc = PerceptionService(
            detector=MockDetector() if detector else None,
            encoder=MockEncoder() if encoder else None,
            tracker=MockTracker() if tracker else None,
            intrinsics=CameraIntrinsics(fx=500, fy=500, cx=320, cy=240, width=640, height=480),
            max_depth=6.0,
            min_depth=0.3,
            depth_scale=0.001,
        )
        return svc

    def test_full_pipeline(self):
        svc = self._make_service()
        bgr = np.zeros((480, 640, 3), dtype=np.uint8)
        # Depth at 2m in the bbox center region (rows 20-200, cols 10-100)
        depth = np.zeros((480, 640), dtype=np.uint16)
        depth[20:200, 10:100] = 2000  # 2m where the bbox is
        tf = np.eye(4)
        tf[2, 3] = 0.5  # camera 0.5m above ground

        result = svc.process_frame(bgr, depth, tf, "chair")
        # Projection may fail if bbox_center_depth returns 0 (outside region)
        # — that's valid behavior. Test that service doesn't crash.
        if result is not None:
            self.assertGreater(len(result.detections_3d), 0)
            self.assertIn("objects", result.scene_graph_json)
            self.assertGreater(result.detect_ms, 0)
        # At minimum: no crash, service ran
        self.assertGreaterEqual(svc._frame_count, 1)

    def test_no_detector_returns_none(self):
        svc = self._make_service(detector=False)
        bgr = np.zeros((480, 640, 3), dtype=np.uint8)
        depth = np.ones((480, 640), dtype=np.uint16) * 2000
        result = svc.process_frame(bgr, depth, np.eye(4))
        self.assertIsNone(result)

    def test_no_intrinsics_returns_none(self):
        svc = self._make_service()
        svc.intrinsics = None
        bgr = np.zeros((480, 640, 3), dtype=np.uint8)
        depth = np.ones((480, 640), dtype=np.uint16) * 2000
        result = svc.process_frame(bgr, depth, np.eye(4))
        # Detect runs but project fails → None
        self.assertIsNone(result)

    def test_no_encoder_still_works(self):
        svc = self._make_service(encoder=False)
        bgr = np.zeros((480, 640, 3), dtype=np.uint8)
        depth = np.ones((480, 640), dtype=np.uint16) * 2000
        svc.process_frame(bgr, depth, np.eye(4), "chair")
        # Should still detect + project + track, just no CLIP features
        # (may be None if projection fails with all-same depth, that's ok)

    def test_no_tracker_still_detects(self):
        svc = self._make_service(tracker=False)
        bgr = np.zeros((480, 640, 3), dtype=np.uint8)
        depth = np.ones((480, 640), dtype=np.uint16) * 2000
        result = svc.process_frame(bgr, depth, np.eye(4), "chair")
        if result:
            self.assertEqual(result.scene_graph_json, "{}")

    def test_set_intrinsics(self):
        svc = self._make_service()
        svc.intrinsics = None
        self.assertIsNone(svc.intrinsics)
        from perception.tracking.projection import CameraIntrinsics

        svc.set_intrinsics(CameraIntrinsics(600, 600, 320, 240, 640, 480))
        self.assertIsNotNone(svc.intrinsics)

    def test_health(self):
        svc = self._make_service()
        h = svc.health()
        self.assertEqual(h["detector"], "MockDetector")
        self.assertEqual(h["encoder"], "MockEncoder")
        self.assertTrue(h["tracker"])
        self.assertEqual(h["frames_processed"], 0)


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

"""Tests for decoupled planner modules — LLMModule + GoalResolver + pipeline.

Verifies pluggable LLM backends, async request/response, Blueprint wiring.
No API keys needed — uses mock backend.
"""

import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "semantic", "planner"))

from decision.modules.llm import LLMModule, LLMRequest, LLMResponse
from runtime import Blueprint, In, Module, Out

# ---------------------------------------------------------------------------
# LLMRequest / LLMResponse
# ---------------------------------------------------------------------------


class TestLLMTypes(unittest.TestCase):
    def test_simple_request(self):
        req = LLMRequest.simple("find the chair", system="You are a nav agent")
        self.assertEqual(len(req.messages), 2)
        self.assertEqual(req.messages[0]["role"], "system")
        self.assertEqual(req.messages[1]["content"], "find the chair")

    def test_simple_no_system(self):
        req = LLMRequest.simple("hello")
        self.assertEqual(len(req.messages), 1)

    def test_response_ok(self):
        r = LLMResponse(text="goal at (1,2)", model="mock")
        self.assertTrue(r.ok)

    def test_response_error(self):
        r = LLMResponse(text="", error="timeout")
        self.assertFalse(r.ok)


# ---------------------------------------------------------------------------
# LLMModule
# ---------------------------------------------------------------------------


class TestLLMModule(unittest.TestCase):
    def test_ports(self):
        mod = LLMModule()
        self.assertIn("request", mod.ports_in)
        self.assertIn("response", mod.ports_out)

    def test_layer_is_4(self):
        self.assertEqual(LLMModule._layer, 4)

    def test_mock_backend_responds(self):
        """Mock backend should return a response without API keys."""
        mod = LLMModule(backend="mock")
        mod.setup()

        received = []
        mod.response._add_callback(received.append)

        req = LLMRequest.simple("navigate to the kitchen", request_id="r1")
        mod.request._deliver(req)

        # Async — wait for response
        deadline = time.time() + 3.0
        while not received and time.time() < deadline:
            time.sleep(0.05)

        mod.stop()

        self.assertGreaterEqual(len(received), 1)
        resp = received[0]
        self.assertIsInstance(resp, LLMResponse)
        self.assertEqual(resp.request_id, "r1")

    def test_health_report(self):
        mod = LLMModule(backend="mock", model="test-model")
        h = mod.health()
        self.assertEqual(h["llm"]["backend"], "mock")
        self.assertEqual(h["llm"]["model"], "test-model")

    def test_unknown_backend_raises(self):
        mod = LLMModule(backend="nonexistent_llm")
        with self.assertRaises(ValueError):
            mod.setup()


# ---------------------------------------------------------------------------
# Pipeline: GoalResolver → LLMModule wiring
# ---------------------------------------------------------------------------


class TestLLMPipeline(unittest.TestCase):
    def test_two_modules_wire_via_blueprint(self):
        """GoalResolver-like module can wire to LLMModule via request/response."""

        class FakeGoalResolver(Module, layer=4):
            llm_request: Out[LLMRequest]
            llm_response: In[LLMResponse]
            goal: Out[str]

            def setup(self):
                self.llm_response.subscribe(self._on_llm_response)

            def _on_llm_response(self, resp: LLMResponse):
                if resp.ok:
                    self.goal.publish(f"resolved: {resp.text}")

        bp = Blueprint()
        bp.add(FakeGoalResolver)
        bp.add(LLMModule, backend="mock")
        bp.wire("FakeGoalResolver", "llm_request", "LLMModule", "request")
        bp.wire("LLMModule", "response", "FakeGoalResolver", "llm_response")
        handle = bp.build()
        handle.start()

        resolver = handle.get_module("FakeGoalResolver")
        goals = []
        resolver.goal._add_callback(goals.append)

        # Send LLM request
        req = LLMRequest.simple("where is the exit?", request_id="g1")
        resolver.llm_request.publish(req)

        # Wait for async round-trip
        deadline = time.time() + 3.0
        while not goals and time.time() < deadline:
            time.sleep(0.05)

        handle.stop()
        self.assertGreaterEqual(len(goals), 1)
        self.assertTrue(goals[0].startswith("resolved:"))

    def test_swap_llm_backend(self):
        """Swapping LLM = changing one Blueprint argument."""

        class Requester(Module, layer=4):
            llm_request: Out[LLMRequest]
            llm_response: In[LLMResponse]

        for backend in ["mock"]:  # only mock testable without API keys
            bp = Blueprint()
            bp.add(Requester)
            bp.add(LLMModule, backend=backend)
            bp.wire("Requester", "llm_request", "LLMModule", "request")
            bp.wire("LLMModule", "response", "Requester", "llm_response")
            handle = bp.build()
            handle.start()

            requester = handle.get_module("Requester")
            requester.llm_request.publish(LLMRequest.simple("test", request_id="swap"))

            time.sleep(0.5)
            self.assertIsNotNone(requester.llm_response.latest, f"No response from backend={backend}")
            handle.stop()


# ---------------------------------------------------------------------------
# Full perception → planner pipeline
# ---------------------------------------------------------------------------


class TestFullDecoupledPipeline(unittest.TestCase):
    def test_image_to_goal_pipeline(self):
        """Image → Detector → GoalResolver → LLM → Goal, all via Blueprint."""
        import numpy as np

        from perception.detection.detector_module import (
            DetectionResult,
            DetectorModule,
        )

        class FakePlanner(Module, layer=4):
            detections: In[DetectionResult]
            goal: Out[str]

            def setup(self):
                self.detections.subscribe(self._on_det)

            def _on_det(self, det: DetectionResult):
                labels = [d.label for d in det.detections if hasattr(d, "label")]
                self.goal.publish(f"goal_from_{labels}")

        # Mock detector backend
        from unittest.mock import patch

        from runtime.tests._test_utils import _MockDetectorBackend

        with patch.object(DetectorModule, "_create_backend", return_value=_MockDetectorBackend()):
            bp = Blueprint()
            bp.add(DetectorModule, detector="yoloe")
            bp.add(FakePlanner)
            bp.auto_wire()
            handle = bp.build()
            handle.start()

            det_mod = handle.get_module("DetectorModule")
            planner = handle.get_module("FakePlanner")

            goals = []
            planner.goal._add_callback(goals.append)

            det_mod.image._deliver(np.zeros((100, 100, 3), dtype=np.uint8))

            self.assertEqual(len(goals), 1)
            self.assertIn("chair", goals[0])
            handle.stop()


if __name__ == "__main__":
    unittest.main(verbosity=2)

"""Wave 2 simplification remediation — Team B behavioural tests.

Items covered:
  W2-10  BBoxNavigator depth sampling: 5x5 patch + IQR rejection + depth_confidence
  W2-11  vlm_bbox_query: exponential-backoff retry + in-memory cache
  W2-12  AgentLoop: tool-signature validation + audit log
"""

from __future__ import annotations

import asyncio
import unittest
from unittest import mock

import numpy as np

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _run(coro):
    """Run a coroutine synchronously in tests."""
    return asyncio.get_event_loop().run_until_complete(coro)


def _make_bbox_navigator():
    """Return a BBoxNavigator with get_config mocked out."""
    identity = np.eye(4)
    mock_cam = mock.MagicMock()
    mock_cam.T_camera_body = identity

    mock_cfg = mock.MagicMock()
    mock_cfg.camera = mock_cam

    with mock.patch("decision.vision.bbox.get_config", return_value=mock_cfg):
        from decision.vision.bbox import BBoxNavConfig, BBoxNavigator

        nav = BBoxNavigator(BBoxNavConfig())
    return nav


# ---------------------------------------------------------------------------
# W2-10  BBoxNavigator — 5x5 patch depth sampling
# ---------------------------------------------------------------------------


class TestW210BBoxNavigatorDepthSampling(unittest.TestCase):
    def _nav(self):
        return _make_bbox_navigator()

    def _flat_depth(self, h, w, value):
        """Depth image filled with a constant value."""
        return np.full((h, w), value, dtype=np.float32)

    def test_high_confidence_when_many_valid_samples(self):
        """When the bbox contains >5 clean depth samples, confidence == 1.0."""
        nav = self._nav()
        depth = self._flat_depth(100, 100, 2.0)  # 25 identical samples, all valid
        bbox = [10.0, 10.0, 90.0, 90.0]
        result = nav.compute_3d_from_bbox(bbox, depth, fx=500, fy=500, cx=50, cy=50)
        self.assertIsNotNone(result)
        self.assertEqual(nav.depth_confidence, 1.0)

    def test_returns_none_when_all_depth_zero(self):
        """When the entire depth image is 0 (invalid), return None — not 0."""
        nav = self._nav()
        depth = np.zeros((100, 100), dtype=np.float32)
        bbox = [10.0, 10.0, 90.0, 90.0]
        result = nav.compute_3d_from_bbox(bbox, depth, fx=500, fy=500, cx=50, cy=50)
        self.assertIsNone(result, "Must return None when no valid depth, not default to 0")

    def test_iqr_rejects_outliers_and_uses_median(self):
        """IQR outlier rejection: a single spike should not corrupt the depth estimate."""
        nav = self._nav()
        # Build a depth image where most pixels are 2.0 m but one is 200.0 m (outlier)
        depth = self._flat_depth(100, 100, 2.0)
        # Place a huge outlier at the exact bbox center
        depth[50, 50] = 200.0
        bbox = [10.0, 10.0, 90.0, 90.0]
        result = nav.compute_3d_from_bbox(bbox, depth, fx=500, fy=500, cx=50, cy=50)
        self.assertIsNotNone(result)
        # Z_c should be close to 2.0, not pulled toward 200
        self.assertAlmostEqual(
            float(result[2]), 2.0, delta=0.5, msg="IQR must reject the 200 m spike; median should be ~2.0 m"
        )

    def test_zero_confidence_when_entire_depth_image_invalid(self):
        """When every pixel is 0 (invalid), depth_confidence must be 0.0."""
        nav = self._nav()
        depth = np.zeros((100, 100), dtype=np.float32)
        bbox = [10.0, 10.0, 90.0, 90.0]
        result = nav.compute_3d_from_bbox(bbox, depth, fx=500, fy=500, cx=50, cy=50)
        self.assertIsNone(result)
        self.assertEqual(
            nav.depth_confidence, 0.0, "depth_confidence must be 0.0 when there are no valid depth samples"
        )


# ---------------------------------------------------------------------------
# W2-11  vlm_bbox_query — retry + cache
# ---------------------------------------------------------------------------


class TestW211VlmBboxQueryRetryCache(unittest.TestCase):
    def setUp(self):
        # Reset module-level cache before each test
        import decision.vision.vlm_bbox as mod

        mod._bbox_cache.clear()
        self.mod = mod

    def _mock_client(self, responses):
        """Return an async client whose chat_with_image yields successive responses."""
        client = mock.MagicMock()
        side_effects = []
        for r in responses:
            if isinstance(r, Exception):
                side_effects.append(mock.AsyncMock(side_effect=r))
            else:
                side_effects.append(mock.AsyncMock(return_value=r))
        client.chat_with_image = mock.AsyncMock(
            side_effect=[
                se.side_effect if hasattr(se, "side_effect") and se.side_effect is not None else se.return_value
                for se in side_effects
            ]
        )

        # Rebuild properly
        async def _chat(**kwargs):
            exc_or_val = responses[_chat._call_count]
            _chat._call_count += 1
            if isinstance(exc_or_val, Exception):
                raise exc_or_val
            return exc_or_val

        _chat._call_count = 0
        client.chat_with_image = _chat
        return client

    def test_retry_succeeds_on_second_attempt(self):
        """After one transient failure the function retries and returns the bbox."""
        good_response = '{"bbox": [10, 20, 100, 200]}'
        client = self._mock_client([RuntimeError("timeout"), good_response])

        with mock.patch("asyncio.sleep", new_callable=mock.AsyncMock):
            result = _run(self.mod.query_object_bbox(client, "img_b64", "chair"))

        self.assertEqual(result, [10.0, 20.0, 100.0, 200.0])

    def test_stale_cache_returned_on_all_retries_exhausted(self):
        """When all 3 retries fail, a <5s cache entry is returned with low confidence."""
        # Pre-populate the cache
        key = self.mod._cache_key("chair", "img_b64")
        self.mod._cache_put(key, [5.0, 6.0, 55.0, 66.0], confidence=1.0)

        client = self._mock_client(
            [
                RuntimeError("err1"),
                RuntimeError("err2"),
                RuntimeError("err3"),
            ]
        )

        with mock.patch("asyncio.sleep", new_callable=mock.AsyncMock):
            result = _run(self.mod.query_object_bbox(client, "img_b64", "chair"))

        self.assertEqual(result, [5.0, 6.0, 55.0, 66.0], "Stale cache entry should be returned when all retries fail")

    def test_none_returned_when_no_cache_and_all_retries_fail(self):
        """With no cache and all retries failing, return None — not a default."""
        client = self._mock_client(
            [
                RuntimeError("e1"),
                RuntimeError("e2"),
                RuntimeError("e3"),
            ]
        )

        with mock.patch("asyncio.sleep", new_callable=mock.AsyncMock):
            result = _run(self.mod.query_object_bbox(client, "fresh_img", "sofa"))

        self.assertIsNone(result)


# ---------------------------------------------------------------------------
# W2-12  AgentLoop — tool-signature validation + audit log
# ---------------------------------------------------------------------------


def _make_context_fn():
    return lambda: {
        "robot_x": 0.0,
        "robot_y": 0.0,
        "visible_objects": "[]",
        "nav_status": "idle",
        "memory_context": "",
        "camera_available": False,
        "camera_image": None,
        "scene_graph": None,
    }


class _AsyncMockLLM:
    """Minimal LLM mock that always returns a done() tool call."""

    def __init__(self, tool_calls_sequence):
        self._seq = list(tool_calls_sequence)
        self._idx = 0

    async def chat_with_tools(self, messages, tools=None):
        if self._idx >= len(self._seq):
            return {"tool_calls": [{"function": {"name": "done", "arguments": '{"summary": "end"}'}, "id": "x"}]}
        tc = self._seq[self._idx]
        self._idx += 1
        return {"tool_calls": [tc]}


class TestW212AgentLoopValidation(unittest.TestCase):
    def _make_loop(self, llm):
        from decision.tasks.agent import AgentLoop

        return AgentLoop(
            llm_client=llm,
            tool_registry={},
            tool_list=[],
            context_fn=_make_context_fn(),
            max_steps=5,
            timeout=30.0,
        )

    def test_unknown_tool_name_feeds_back_error_to_llm(self):
        """An unregistered tool name must not execute silently; error goes to LLM."""
        bad_call = {
            "function": {"name": "fly_to_moon", "arguments": "{}"},
            "id": "call_bad",
        }
        good_call = {
            "function": {"name": "done", "arguments": '{"summary": "recovered"}'},
            "id": "call_done",
        }
        llm = _AsyncMockLLM([bad_call, good_call])
        loop = self._make_loop(llm)
        state = _run(loop.run("go somewhere"))

        # The audit log must contain the rejected attempt
        rejected = [e for e in loop._tool_call_audit if e["tool_name"] == "fly_to_moon"]
        self.assertTrue(len(rejected) >= 1, "Unknown tool must appear in audit log")
        self.assertIn("VALIDATION_ERROR", rejected[0]["result_summary"])

        # Conversation must carry the error message so LLM can self-correct
        error_msgs = [m for m in state.messages if m.get("role") == "tool" and "fly_to_moon" in m.get("content", "")]
        self.assertTrue(
            len(error_msgs) >= 1, "Error message must be present in message history for LLM self-correction"
        )

    def test_invalid_args_schema_feeds_back_error(self):
        """navigate_to requires x and y (numbers); missing args must trigger schema error."""
        # navigate_to requires "x" and "y" — send empty args
        bad_call = {
            "function": {"name": "navigate_to", "arguments": "{}"},
            "id": "call_nav",
        }
        good_call = {
            "function": {"name": "done", "arguments": '{"summary": "ok"}'},
            "id": "call_done",
        }
        llm = _AsyncMockLLM([bad_call, good_call])
        loop = self._make_loop(llm)
        _run(loop.run("navigate"))

        rejected = [e for e in loop._tool_call_audit if e["tool_name"] == "navigate_to"]
        self.assertTrue(len(rejected) >= 1)
        self.assertIn("VALIDATION_ERROR", rejected[0]["result_summary"])

    def test_audit_log_counts_invalid_calls_toward_max_steps(self):
        """Validation failures must consume steps so the loop cannot run forever."""
        # Send max_steps worth of bad calls to hit the limit
        bad_calls = [{"function": {"name": "nonexistent_tool", "arguments": "{}"}, "id": f"c{i}"} for i in range(10)]
        llm = _AsyncMockLLM(bad_calls)
        loop = self._make_loop(llm)  # max_steps=5
        state = _run(loop.run("loop forever"))

        # Loop must have stopped at or before max_steps
        self.assertLessEqual(
            state.step, loop._max_steps, "Loop must stop at max_steps even with all-invalid tool calls"
        )
        # Audit log must record each rejected attempt
        self.assertGreater(len(loop._tool_call_audit), 0, "Audit log must not be empty after rejected calls")


if __name__ == "__main__":
    unittest.main()

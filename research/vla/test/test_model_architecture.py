"""
Unit tests for VLA model architecture components.

Tests the core model modules WITHOUT loading the actual Qwen2.5-VL-3B model
(which requires GPU and ~7GB). Instead, uses mock/small tensors to verify
shapes, logic, and interfaces.
"""

import math
import unittest
from unittest.mock import Mock, patch

import numpy as np
import torch

from vla_nav.model.backbone import (
    DynamicFPSSampler,
    FrameEntry,
    GridPooler,
)
from vla_nav.model.adacot import (
    AdaCoTModule,
    AdaCoTTriggerHead,
    CoTGenerator,
    THINK_TOKEN,
    NO_THINK_TOKEN,
)
from vla_nav.model.vlingmem import (
    VLingMemModule,
    MemoryEntry,
    SummaryEncoder,
    VisualKeyEncoder,
)
from vla_nav.model.action_head import ActionHead, ResidualBlock


# ===========================================================================
# Dynamic FPS Sampler
# ===========================================================================
class TestDynamicFPSSampler(unittest.TestCase):
    def setUp(self):
        self.sampler = DynamicFPSSampler(fs_max=10.0, stability=5.0, max_frames=8)

    def test_empty_buffer(self):
        result = self.sampler.sample([], current_time=10.0)
        self.assertEqual(len(result), 0)

    def test_single_frame(self):
        frame = FrameEntry(image=np.zeros((2, 2, 3), dtype=np.uint8), timestamp=9.0)
        result = self.sampler.sample([frame], current_time=10.0)
        self.assertEqual(len(result), 1)

    def test_recent_frames_preferred(self):
        frames = [
            FrameEntry(image=np.zeros((2, 2, 3), dtype=np.uint8), timestamp=float(t))
            for t in range(0, 20)
        ]
        result = self.sampler.sample(frames, current_time=20.0)
        self.assertLessEqual(len(result), 8)
        # Most recent frame must be included
        timestamps = [f.timestamp for f in result]
        self.assertIn(19.0, timestamps)

    def test_output_sorted_by_time(self):
        frames = [
            FrameEntry(image=np.zeros((2, 2, 3), dtype=np.uint8), timestamp=float(t))
            for t in range(10)
        ]
        result = self.sampler.sample(frames, current_time=10.0)
        timestamps = [f.timestamp for f in result]
        self.assertEqual(timestamps, sorted(timestamps))

    def test_max_frames_respected(self):
        frames = [
            FrameEntry(image=np.zeros((2, 2, 3), dtype=np.uint8), timestamp=float(t))
            for t in range(100)
        ]
        result = self.sampler.sample(frames, current_time=100.0)
        self.assertLessEqual(len(result), 8)


# ===========================================================================
# Grid Pooler
# ===========================================================================
class TestGridPooler(unittest.TestCase):
    def test_stride_1_no_op(self):
        pooler = GridPooler()
        features = torch.randn(1, 49, 256)
        result = GridPooler.pool(features, stride=1)
        self.assertEqual(result.shape, features.shape)

    def test_stride_2_reduces(self):
        features = torch.randn(1, 49, 256)  # 7x7 spatial
        result = GridPooler.pool(features, stride=2)
        # 7/2 = 3 → 3*3 = 9
        self.assertLess(result.shape[1], 49)

    def test_compute_stride_recent(self):
        pooler = GridPooler(g_param=3.0, base_stride=2)
        stride = pooler.compute_stride(dt=0.1)  # Very recent
        self.assertEqual(stride, max(1, int(2 / math.exp(-0.1 / 3.0))))

    def test_compute_stride_old(self):
        pooler = GridPooler(g_param=3.0, base_stride=2)
        stride = pooler.compute_stride(dt=30.0)  # Old frame
        self.assertGreater(stride, 2)


# ===========================================================================
# AdaCoT Trigger Head
# ===========================================================================
class TestAdaCoTTriggerHead(unittest.TestCase):
    def test_output_shape(self):
        head = AdaCoTTriggerHead(hidden_dim=256)
        hidden = torch.randn(4, 256)
        logits = head(hidden)
        self.assertEqual(logits.shape, (4, 2))

    def test_predict_prob_range(self):
        head = AdaCoTTriggerHead(hidden_dim=256)
        hidden = torch.randn(1, 256)
        prob = head.predict_prob(hidden)
        self.assertGreaterEqual(prob, 0.0)
        self.assertLessEqual(prob, 1.0)


class TestAdaCoTModule(unittest.TestCase):
    def test_trigger_loss(self):
        module = AdaCoTModule(hidden_dim=128)
        hidden = torch.randn(8, 128)
        labels = torch.randint(0, 2, (8,))
        loss = module.compute_trigger_loss(hidden, labels)
        self.assertGreater(loss.item(), 0.0)

    def test_statistics_tracking(self):
        module = AdaCoTModule(hidden_dim=128, trigger_threshold=0.0)
        # Force THINK by setting threshold to 0
        self.assertEqual(module.think_ratio, 0.0)
        module._total_steps = 10
        module._think_steps = 3
        self.assertAlmostEqual(module.think_ratio, 0.3)

    def test_reset_statistics(self):
        module = AdaCoTModule(hidden_dim=128)
        module._total_steps = 100
        module._think_steps = 30
        module.reset_statistics()
        self.assertEqual(module._total_steps, 0)
        self.assertEqual(module._think_steps, 0)


# ===========================================================================
# VLingMem
# ===========================================================================
class TestSummaryEncoder(unittest.TestCase):
    def test_output_shape(self):
        encoder = SummaryEncoder(hidden_dim=256, summary_dim=64)
        hidden_seq = torch.randn(1, 10, 256)
        summary = encoder(hidden_seq)
        self.assertEqual(summary.shape, (1, 64))

    def test_normalized(self):
        encoder = SummaryEncoder(hidden_dim=256, summary_dim=64)
        hidden_seq = torch.randn(1, 10, 256)
        summary = encoder(hidden_seq)
        norm = torch.norm(summary, dim=-1)
        self.assertAlmostEqual(norm.item(), 1.0, places=4)


class TestVisualKeyEncoder(unittest.TestCase):
    def test_output_shape(self):
        encoder = VisualKeyEncoder(visual_dim=256, key_dim=32)
        visual = torch.randn(1, 256)
        key = encoder(visual)
        self.assertEqual(key.shape, (1, 32))


class TestVLingMemModule(unittest.TestCase):
    def setUp(self):
        self.mem = VLingMemModule(
            hidden_dim=128, summary_dim=32, visual_key_dim=16,
            max_entries=5, top_k=2,
        )

    def test_store_and_size(self):
        self.assertEqual(self.mem.size, 0)
        self.mem.store(
            cot_text="Saw a door ahead",
            cot_hidden_states=torch.randn(1, 5, 128),
            visual_feature=torch.randn(1, 128),
            position=np.array([1.0, 2.0, 0.0]),
            heading=0.5,
        )
        self.assertEqual(self.mem.size, 1)

    def test_eviction(self):
        for i in range(10):
            self.mem.store(
                cot_text=f"Memory {i}",
                cot_hidden_states=None,
                visual_feature=torch.randn(1, 128),
                position=np.array([float(i), 0.0, 0.0]),
                heading=0.0,
            )
        self.assertEqual(self.mem.size, 5)  # max_entries=5

    def test_retrieve_empty(self):
        query = torch.randn(1, 128)
        results = self.mem.retrieve(query)
        self.assertEqual(len(results), 0)

    def test_retrieve_returns_top_k(self):
        for i in range(5):
            self.mem.store(
                cot_text=f"Memory {i}",
                cot_hidden_states=None,
                visual_feature=torch.randn(1, 128),
                position=np.array([float(i), 0.0, 0.0]),
                heading=0.0,
            )
        query = torch.randn(1, 128)
        results = self.mem.retrieve(query)
        self.assertLessEqual(len(results), 2)  # top_k=2

    def test_format_memory_text(self):
        self.mem.store(
            cot_text="Found kitchen entrance",
            cot_hidden_states=None,
            visual_feature=torch.randn(1, 128),
            position=np.array([3.0, 4.0, 0.0]),
            heading=1.57,
        )
        text = self.mem.format_memory_text(
            self.mem._bank,
            current_position=np.array([0.0, 0.0, 0.0]),
        )
        self.assertIn("Mem 1", text)
        self.assertIn("3.0", text)
        self.assertIn("kitchen", text)

    def test_clear(self):
        self.mem.store(
            cot_text="test",
            cot_hidden_states=None,
            visual_feature=torch.randn(1, 128),
            position=np.zeros(3),
            heading=0.0,
        )
        self.mem.clear()
        self.assertEqual(self.mem.size, 0)


# ===========================================================================
# Action Head
# ===========================================================================
class TestResidualBlock(unittest.TestCase):
    def test_output_shape(self):
        block = ResidualBlock(dim=64)
        x = torch.randn(2, 64)
        y = block(x)
        self.assertEqual(y.shape, (2, 64))


class TestActionHead(unittest.TestCase):
    def setUp(self):
        self.head = ActionHead(
            hidden_dim=128, action_dim=3, horizon=5,
            max_linear_step=0.5, max_angular_step=0.785,
        )

    def test_output_shape(self):
        hidden = torch.randn(2, 128)
        waypoints = self.head(hidden)
        self.assertEqual(waypoints.shape, (2, 5, 3))

    def test_bounded_output(self):
        hidden = torch.randn(100, 128)
        waypoints = self.head(hidden)
        self.assertTrue((waypoints[..., :2].abs() <= 0.5 + 1e-6).all())
        self.assertTrue((waypoints[..., 2].abs() <= 0.785 + 1e-6).all())

    def test_loss_computation(self):
        hidden = torch.randn(4, 128)
        predicted = self.head(hidden)
        target = torch.randn(4, 5, 3) * 0.3
        loss = self.head.compute_loss(predicted, target)
        self.assertGreater(loss.item(), 0.0)

    def test_loss_with_mask(self):
        predicted = torch.randn(4, 5, 3)
        target = torch.randn(4, 5, 3)
        mask = torch.ones(4, 5)
        mask[0, 3:] = 0  # Mask out last 2 steps of first sample
        loss = self.head.compute_loss(predicted, target, mask)
        self.assertGreater(loss.item(), 0.0)

    def test_first_waypoint(self):
        hidden = torch.randn(2, 128)
        waypoints = self.head(hidden)
        first = self.head.get_first_waypoint(waypoints)
        self.assertEqual(first.shape, (2, 3))

    def test_world_coordinate_conversion(self):
        waypoints = torch.tensor([[[0.1, 0.0, 0.0],
                                   [0.1, 0.0, 0.0],
                                   [0.1, 0.0, 0.0],
                                   [0.1, 0.0, 0.0],
                                   [0.1, 0.0, 0.0]]])
        world = self.head.waypoints_to_world(
            waypoints, robot_x=0.0, robot_y=0.0, robot_theta=0.0
        )
        self.assertEqual(len(world), 5)
        # Moving forward 0.1m at heading 0 → x increases
        self.assertAlmostEqual(world[0][0], 0.1, places=4)
        self.assertAlmostEqual(world[0][1], 0.0, places=4)


# ===========================================================================
# CoT Generator
# ===========================================================================
class TestCoTGenerator(unittest.TestCase):
    def test_build_prompt(self):
        gen = CoTGenerator()
        prompt = gen.build_prompt(
            instruction="Go to the kitchen",
            observation_summary="At (2.0, 3.0), step 5",
            memory_summary="Previously saw a corridor",
        )
        self.assertIn(THINK_TOKEN, prompt)
        self.assertIn("kitchen", prompt)
        self.assertIn("corridor", prompt)


if __name__ == "__main__":
    unittest.main()

"""Focused tests split from the former monolithic offline semantic pipeline."""

import json

import numpy as np
import pytest

from decision.goals.resolver import TargetBeliefManager
from perception.tracking.instance_tracker import InstanceTracker, TrackedObject
from perception.tracking.projection import Detection3D


class TestBeliefSystemEndToEnd:
    """模拟完整导航 episode, 验证信念系统行为。"""

    def test_belief_update_through_navigation(self):
        """模拟 episode: 检测→追踪→信念更新→场景图输出。"""
        tracker = InstanceTracker(max_objects=50)

        # 固定 CLIP 特征 (每个物体的特征一致, 确保跨帧匹配)
        rng = np.random.RandomState(42)
        chair_feat = rng.randn(512).astype(np.float32)
        chair_feat /= np.linalg.norm(chair_feat)
        desk_feat = rng.randn(512).astype(np.float32)
        desk_feat /= np.linalg.norm(desk_feat)

        # 模拟 10 帧检测, 逐渐建立场景图
        for _ in range(10):
            detections = [
                Detection3D(
                    label="chair", score=0.85 + rng.randn() * 0.02,
                    position=np.array([3.0 + rng.randn() * 0.05,
                                       4.0 + rng.randn() * 0.05, 0.4]),
                    features=chair_feat + rng.randn(512).astype(np.float32) * 0.01,
                    bbox_2d=np.array([100, 100, 200, 200]),
                    depth=3.0,
                ),
                Detection3D(
                    label="desk", score=0.90 + rng.randn() * 0.02,
                    position=np.array([4.0 + rng.randn() * 0.05,
                                       3.5 + rng.randn() * 0.05, 0.7]),
                    features=desk_feat + rng.randn(512).astype(np.float32) * 0.01,
                    bbox_2d=np.array([200, 100, 350, 250]),
                    depth=4.0,
                ),
            ]
            tracker.update(detections)

        sg_json = tracker.get_scene_graph_json()
        sg = json.loads(sg_json)

        assert len(sg["objects"]) >= 2, "Should have at least 2 tracked objects"

        for obj in sg["objects"]:
            assert "belief" in obj
            assert obj["belief"]["P_exist"] > 0.6, \
                f"Object {obj['label']} P_exist={obj['belief']['P_exist']:.2f} too low after 10 frames"
            assert obj["belief"]["sigma_pos"] <= 1.0, \
                f"Object {obj['label']} sigma_pos={obj['belief']['sigma_pos']:.2f} should not increase"

    def test_miss_streak_reduces_belief(self):
        """模拟物体消失: 先检测到, 后连续未检测 → 信念下降。"""
        obj = TrackedObject(
            object_id=0, label="bottle",
            position=np.array([5.0, 3.0, 0.8]),
            best_score=0.8,
        )

        det = Detection3D(
            label="bottle", score=0.85,
            position=np.array([5.0, 3.0, 0.8]),
            features=np.array([]),
            bbox_2d=np.array([100, 100, 200, 200]),
            depth=5.0,
        )
        for _ in range(5):
            obj.update(det)

        p_after_detections = obj.existence_prob

        for _ in range(10):
            obj.record_miss()

        p_after_misses = obj.existence_prob
        assert p_after_misses < p_after_detections, \
            f"Belief should decrease: {p_after_detections:.3f} → {p_after_misses:.3f}"

    def test_graph_diffusion_boosts_new_object(self):
        """模拟图扩散: 高可信度房间中新检测的物体应获得加成。"""
        tracker = InstanceTracker(max_objects=50)

        established_detections = [
            Detection3D(
                label="desk", score=0.92,
                position=np.array([4.0, 3.5, 0.7]),
                features=np.random.randn(512).astype(np.float32),
                bbox_2d=np.array([100, 100, 250, 200]),
                depth=4.0,
            ),
            Detection3D(
                label="chair", score=0.88,
                position=np.array([4.5, 3.0, 0.4]),
                features=np.random.randn(512).astype(np.float32),
                bbox_2d=np.array([200, 150, 300, 300]),
                depth=4.5,
            ),
            Detection3D(
                label="monitor", score=0.85,
                position=np.array([4.2, 3.2, 0.8]),
                features=np.random.randn(512).astype(np.float32),
                bbox_2d=np.array([150, 50, 250, 150]),
                depth=4.2,
            ),
        ]

        for _ in range(15):
            tracker.update(established_detections)

        new_det = Detection3D(
            label="keyboard", score=0.72,
            position=np.array([4.0, 3.3, 0.75]),
            features=np.random.randn(512).astype(np.float32),
            bbox_2d=np.array([160, 100, 240, 140]),
            depth=4.0,
        )
        tracker.update([new_det, *established_detections])

        sg = json.loads(tracker.get_scene_graph_json())
        keyboard = next((o for o in sg["objects"] if o["label"] == "keyboard"), None)
        assert keyboard is not None, "Keyboard should be tracked"

        # The keyboard's credibility should benefit from being near established objects
        assert keyboard["belief"]["P_exist"] > 0.5

class TestMultiHypothesisFullScenario:
    """模拟完整的多假设导航场景。"""

    def test_disambiguation_scenario(self):
        """场景: "find the fire extinguisher" — 走廊有 3 个灭火器。"""
        candidates = [
            {"id": 1, "label": "fire extinguisher", "position": [4.0, 1.0, 0.8],
             "fused_score": 0.82, "belief": {"credibility": 0.82}, "room_match": 0.8},
            {"id": 30, "label": "fire extinguisher", "position": [8.0, 0.5, 0.8],
             "fused_score": 0.78, "belief": {"credibility": 0.72}, "room_match": 0.8},
            {"id": 31, "label": "fire extinguisher", "position": [12.0, -0.5, 0.8],
             "fused_score": 0.72, "belief": {"credibility": 0.65}, "room_match": 0.8},
        ]

        mgr = TargetBeliefManager()
        mgr.init_from_candidates(candidates)

        assert mgr.num_active == 3
        assert not mgr.is_converged

        first = mgr.select_next_target(robot_position=[0.0, 0.0])
        assert first is not None
        first_id = first.object_id

        mgr.bayesian_update(object_id=first_id, detected=False, clip_sim=0.1)
        assert mgr.num_active == 2

        second = mgr.select_next_target(robot_position=[4.0, 1.0])
        assert second is not None
        assert second.object_id != first_id

        mgr.bayesian_update(object_id=second.object_id, detected=True, clip_sim=0.9)

        assert mgr.best_hypothesis.object_id == second.object_id
        assert mgr.best_hypothesis.posterior > 0.5

    def test_all_rejected_returns_none(self):
        """所有候选都被拒绝 → 应触发探索。"""
        candidates = [
            {"id": 1, "label": "chair", "position": [3.0, 4.0, 0.4],
             "fused_score": 0.6, "belief": {"credibility": 0.5}, "room_match": 0.5},
            {"id": 2, "label": "chair", "position": [5.0, 2.0, 0.4],
             "fused_score": 0.55, "belief": {"credibility": 0.45}, "room_match": 0.5},
        ]

        mgr = TargetBeliefManager()
        mgr.init_from_candidates(candidates)

        mgr.bayesian_update(object_id=1, detected=False, clip_sim=0.1)
        mgr.bayesian_update(object_id=2, detected=False, clip_sim=0.1)

        result = mgr.select_next_target()
        assert result is None, "All rejected → should return None (trigger explore)"

class TestModelIntegration:
    """GCN model + InstanceTracker integration tests (requires belief_network)."""

    def setup_method(self):
        pytest.importorskip(
            "memory.knowledge.belief.network",
            reason="belief_network module not available",
        )
        from memory.knowledge.belief.network import HAS_TORCH
        from memory.knowledge.knowledge_graph import IndustrialKnowledgeGraph
        from perception.tracking.instance_tracker import InstanceTracker
        from perception.tracking.projection import Detection3D
        self.InstanceTracker = InstanceTracker
        self.Detection3D = Detection3D
        self.KG = IndustrialKnowledgeGraph
        self.torch_ok = HAS_TORCH

    def test_train_belief_model(self):
        """InstanceTracker should be able to train belief model."""
        if not self.torch_ok:
            pytest.skip("PyTorch not available")
        kg = self.KG()
        tracker = self.InstanceTracker(max_objects=100, knowledge_graph=kg)
        # train_belief_model returns bool (True on success)
        success = tracker.train_belief_model(num_scenes=100, epochs=3)
        assert success is True
        assert tracker._belief_model is not None

    def test_gcn_mode_produces_valid_beliefs(self):
        """With trained model, BP should still produce valid belief states."""
        if not self.torch_ok:
            pytest.skip("PyTorch not available")
        kg = self.KG()
        tracker = self.InstanceTracker(max_objects=100, knowledge_graph=kg)
        tracker.train_belief_model(num_scenes=100, epochs=3)

        # Add objects
        for label, pos in [("desk", [2.0, 3.0, 0.7]),
                           ("chair", [2.5, 3.0, 0.5]),
                           ("computer", [2.0, 2.5, 0.6])]:
            tracker.update([self.Detection3D(
                label=label, score=0.9, position=np.array(pos),
                features=np.array([]),
                bbox_2d=np.array([0, 0, 100, 100]), depth=2.0,
            )])
        for obj in tracker.objects.values():
            assert 0.0 < obj.existence_prob < 1.0
            bd = obj.to_belief_dict()
            assert "P_exist" in bd

    def test_fallback_without_model(self):
        """Without trained model, should fall back to KG lookup."""
        kg = self.KG()
        tracker = self.InstanceTracker(max_objects=100, knowledge_graph=kg)
        assert tracker._belief_model is None
        for label, pos in [("desk", [2.0, 3.0, 0.7]),
                           ("chair", [2.5, 3.0, 0.5])]:
            tracker.update([self.Detection3D(
                label=label, score=0.9, position=np.array(pos),
                features=np.array([]),
                bbox_2d=np.array([0, 0, 100, 100]), depth=2.0,
            )])
        # Should still work (KG fallback)
        for obj in tracker.objects.values():
            assert obj.existence_prob > 0.0

    def test_scene_graph_json_with_model(self):
        """Scene graph JSON should be valid with GCN model active."""
        if not self.torch_ok:
            pytest.skip("PyTorch not available")
        kg = self.KG()
        tracker = self.InstanceTracker(max_objects=100, knowledge_graph=kg)
        tracker.train_belief_model(num_scenes=50, epochs=2)
        for label, pos in [("desk", [2.0, 3.0, 0.7]),
                           ("chair", [2.5, 3.0, 0.5])]:
            tracker.update([self.Detection3D(
                label=label, score=0.9, position=np.array(pos),
                features=np.array([]),
                bbox_2d=np.array([0, 0, 100, 100]), depth=2.0,
            )])
        sg = json.loads(tracker.get_scene_graph_json())
        assert sg["graph_version"] == "3.0"
        assert "objects" in sg
        assert "phantom_nodes" in sg

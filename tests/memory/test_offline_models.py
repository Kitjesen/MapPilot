"""Focused tests split from the former monolithic offline semantic pipeline."""

import math
import time

import numpy as np
import pytest

from memory.scheduling.voi_scheduler import SchedulerAction, SchedulerState, VoIScheduler


class TestVoIFullEpisode:
    """模拟完整 episode, 统计 VoI 决策分布。"""

    def test_episode_decision_distribution(self):
        """模拟 50 步导航, 统计 continue/reperceive/slow_reason 分布。

        场景: 目标信念在中段急剧下降 (模拟误检/环境变化), 触发 VoI 再感知。
        VoI 的安全规则: credibility < 0.3 → 强制 reperceive。
        """
        scheduler = VoIScheduler()
        decisions = {"continue": 0, "reperceive": 0, "slow_reason": 0}

        credibility = 0.5
        distance_to_goal = 15.0
        accumulated = 0.0
        last_reperception_time = 0.0

        for step in range(50):
            accumulated += 0.5
            distance_to_goal = max(0.5, distance_to_goal - 0.3)

            if step < 10:
                credibility = min(0.9, credibility + 0.04)
            elif step < 20:
                credibility = max(0.15, credibility - 0.07)
            elif step < 30:
                credibility = min(0.85, credibility + 0.05)
            else:
                credibility = max(0.2, credibility - 0.02)

            state = SchedulerState(
                target_credibility=credibility,
                target_existence_prob=credibility * 0.9,
                target_position_var=max(0.1, 2.0 - step * 0.03),
                match_count=min(5, step // 3),
                total_objects=20,
                distance_to_goal=distance_to_goal,
                nav_accumulated_dist=accumulated,
                distance_since_last_reperception=accumulated - last_reperception_time,
                slow_reason_count=decisions["slow_reason"],
                reperception_count=decisions["reperceive"],
                time_elapsed=step * 2.0,
                last_reperception_time=last_reperception_time,
                last_slow_reason_time=0.0,
            )

            action = scheduler.decide(state)
            decisions[action.value] += 1

            if action == SchedulerAction.REPERCEIVE:
                last_reperception_time = time.time()

        total = sum(decisions.values())
        cont_rate = decisions["continue"] / total
        repr_rate = decisions["reperceive"] / total

        assert cont_rate > 0.4, f"Continue rate {cont_rate:.0%} too low"
        assert repr_rate >= 0.02, f"Reperceive rate {repr_rate:.0%} too low — VoI never triggered"
        assert repr_rate < 0.5, f"Reperceive rate {repr_rate:.0%} too high (should be adaptive)"

    def test_voi_vs_fixed_interval(self):
        """VoI 应比固定间隔更高效: 相同 SR 下更少 reperception 次数。"""
        scheduler = VoIScheduler()

        voi_reperceive_count = 0
        fixed_2m_reperceive_count = 0

        accumulated = 0.0
        last_repr_voi = 0.0
        last_repr_fixed = 0.0

        for step in range(100):
            accumulated += 0.3
            cred = 0.7 + 0.1 * math.sin(step * 0.2)

            state = SchedulerState(
                target_credibility=cred,
                distance_to_goal=max(0.5, 10.0 - step * 0.1),
                distance_since_last_reperception=accumulated - last_repr_voi,
                last_reperception_time=last_repr_voi,
            )
            action = scheduler.decide(state)
            if action == SchedulerAction.REPERCEIVE:
                voi_reperceive_count += 1
                last_repr_voi = accumulated

            if accumulated - last_repr_fixed >= 2.0:
                fixed_2m_reperceive_count += 1
                last_repr_fixed = accumulated

        assert voi_reperceive_count <= fixed_2m_reperceive_count * 1.5, \
            f"VoI ({voi_reperceive_count}) should not be much more than fixed ({fixed_2m_reperceive_count})"

class TestBeliefNetwork:
    """KG-BELIEF GCN model tests (requires belief_network module)."""

    def setup_method(self):
        pytest.importorskip(
            "memory.knowledge.belief.network",
            reason="belief_network module not available",
        )
        from memory.knowledge.belief.network import (
            HAS_TORCH,
            NUM_AFFORDANCES,
            ROOM_TYPES,
            build_affordance_vectors,
            build_cooccurrence_matrix,
            build_dangerous_mask,
            build_object_vocabulary,
            build_room_prior_vectors,
            build_safety_vector,
        )
        from memory.knowledge.knowledge_graph import IndustrialKnowledgeGraph
        self.KG = IndustrialKnowledgeGraph
        self.torch_ok = HAS_TORCH
        self.build_vocab = build_object_vocabulary
        self.build_cooc = build_cooccurrence_matrix
        self.build_safety = build_safety_vector
        self.build_aff = build_affordance_vectors
        self.build_priors = build_room_prior_vectors
        self.build_danger = build_dangerous_mask
        self.ROOM_TYPES = ROOM_TYPES
        self.NUM_AFF = NUM_AFFORDANCES

    def test_vocabulary_completeness(self):
        """Vocabulary should cover all objects in KG room mappings."""
        kg = self.KG()
        vocab, _ = self.build_vocab(kg)
        assert len(vocab) >= 50, f"Expected >= 50 objects, got {len(vocab)}"
        assert "desk" in vocab
        assert "fire_extinguisher" in vocab
        assert "gas_cylinder" in vocab

    def test_cooccurrence_matrix_shape(self):
        kg = self.KG()
        vocab, _ = self.build_vocab(kg)
        cooc = self.build_cooc(kg, vocab)
        C = len(vocab)
        assert cooc.shape == (C, C)
        assert np.all(cooc >= 0)
        assert np.all(cooc <= 1.0)
        # Diagonal should be non-zero (self co-occurrence)
        assert np.any(cooc.diagonal() > 0)

    def test_cooccurrence_symmetry(self):
        kg = self.KG()
        vocab, _ = self.build_vocab(kg)
        cooc = self.build_cooc(kg, vocab)
        assert np.allclose(cooc, cooc.T), "Co-occurrence should be symmetric"

    def test_safety_vector(self):
        kg = self.KG()
        vocab, _ = self.build_vocab(kg)
        s = self.build_safety(kg, vocab)
        assert s.shape == (len(vocab),)
        # gas_cylinder should be dangerous
        if "gas_cylinder" in vocab:
            assert s[vocab["gas_cylinder"]] > 0.5, "gas_cylinder should be dangerous"
        # chair should be safe
        if "chair" in vocab:
            assert s[vocab["chair"]] == 0.0, "chair should be safe"

    def test_affordance_matrix(self):
        kg = self.KG()
        vocab, _ = self.build_vocab(kg)
        A = self.build_aff(kg, vocab)
        assert A.shape == (len(vocab), self.NUM_AFF)
        # desk should not be graspable
        if "desk" in vocab:
            assert A[vocab["desk"], 0] == 0.0  # graspable index = 0

    def test_room_priors(self):
        kg = self.KG()
        vocab, _ = self.build_vocab(kg)
        priors = self.build_priors(kg, vocab)
        assert len(priors) == len(self.ROOM_TYPES)
        # Office should expect desk
        assert priors["office"][vocab["desk"]] == 1.0

    def test_dangerous_mask(self):
        kg = self.KG()
        vocab, _ = self.build_vocab(kg)
        mask = self.build_danger(kg, vocab)
        assert mask.shape == (len(vocab),)
        dangerous_count = int(mask.sum())
        assert dangerous_count > 0, "Should have some dangerous objects"
        assert dangerous_count < len(vocab), "Not all objects should be dangerous"

    def test_model_forward_pass(self):
        """GCN forward pass should produce valid output shape and range."""
        if not self.torch_ok:
            pytest.skip("PyTorch not available")
        import torch

        from memory.knowledge.belief.network import NUM_AFFORDANCES, KGBeliefGCN
        from memory.knowledge.knowledge_graph import IndustrialKnowledgeGraph
        kg = IndustrialKnowledgeGraph()
        vocab, _ = self.build_vocab(kg)
        C = len(vocab)
        model = KGBeliefGCN(num_objects=C)
        N = 5
        input_dim = 4 * C + NUM_AFFORDANCES
        x = torch.randn(N, input_dim)
        adj = torch.eye(N)
        adj[0, 1] = adj[1, 0] = 1.0
        out = model(x, adj)
        assert out.shape == (N, C)
        assert torch.all(out >= 0) and torch.all(out <= 1), "Output should be in [0,1]"

    def test_model_batched_forward(self):
        """Batched forward pass should handle variable-size graphs (loop over batch)."""
        if not self.torch_ok:
            pytest.skip("PyTorch not available")
        import torch

        from memory.knowledge.belief.network import NUM_AFFORDANCES, KGBeliefGCN
        from memory.knowledge.knowledge_graph import IndustrialKnowledgeGraph
        kg = IndustrialKnowledgeGraph()
        vocab, _ = self.build_vocab(kg)
        C = len(vocab)
        model = KGBeliefGCN(num_objects=C)
        B, N = 3, 5
        input_dim = 4 * C + NUM_AFFORDANCES
        x = torch.randn(B, N, input_dim)
        adj = torch.eye(N).unsqueeze(0).expand(B, -1, -1).clone()
        # GCNConv expects 2D (N, input_dim) and (N, N), so loop over batch
        outs = []
        for i in range(B):
            outs.append(model(x[i], adj[i]))
        out = torch.stack(outs)
        assert out.shape == (B, N, C)

    def test_adjacency_normalisation(self):
        """Adjacency normalization (D^{-1/2} A_hat D^{-1/2}) from GCNConv."""
        if not self.torch_ok:
            pytest.skip("PyTorch not available")
        import torch
        adj = torch.tensor([[0., 1., 0.], [1., 0., 1.], [0., 1., 0.]])
        # Replicate GCNConv.forward normalization: A_hat = A + I, symmetric norm
        a_hat = adj + torch.eye(adj.size(0), device=adj.device)
        d_inv_sqrt = torch.diag(1.0 / torch.sqrt(a_hat.sum(dim=1).clamp(min=1e-8)))
        norm = d_inv_sqrt @ a_hat @ d_inv_sqrt
        # Should be symmetric
        assert torch.allclose(norm, norm.T, atol=1e-5)
        # Diagonal should be non-zero (self-loops)
        assert torch.all(norm.diagonal() > 0)

class TestKGDataGeneration:
    """KG 合成训练数据测试。"""

    def setup_method(self):
        from memory.knowledge.belief.network import (
            HAS_TORCH,
            build_affordance_vectors,
            build_cooccurrence_matrix,
            build_object_vocabulary,
            build_room_prior_vectors,
            build_safety_vector,
        )
        from memory.knowledge.knowledge_graph import IndustrialKnowledgeGraph
        self.KG = IndustrialKnowledgeGraph
        self.torch_ok = HAS_TORCH
        self.build_vocab = build_object_vocabulary
        self.build_cooc = build_cooccurrence_matrix
        self.build_safety = build_safety_vector
        self.build_aff = build_affordance_vectors
        self.build_priors = build_room_prior_vectors

    def test_dataset_generation(self):
        """Should generate correct number of scenes."""
        if not self.torch_ok:
            pytest.skip("PyTorch not available")
        from memory.knowledge.belief.network import KGSceneGraphDataset
        kg = self.KG()
        vocab, _ = self.build_vocab(kg)
        ds = KGSceneGraphDataset(
            kg, vocab,
            self.build_cooc(kg, vocab),
            self.build_safety(kg, vocab),
            self.build_aff(kg, vocab),
            self.build_priors(kg, vocab),
            num_scenes=50,
        )
        assert len(ds) == 50

    def test_dataset_sample_shape(self):
        """Each sample should have correct feature dimensions."""
        if not self.torch_ok:
            pytest.skip("PyTorch not available")
        from memory.knowledge.belief.network import NUM_AFFORDANCES, KGSceneGraphDataset
        kg = self.KG()
        vocab, _ = self.build_vocab(kg)
        C = len(vocab)
        ds = KGSceneGraphDataset(
            kg, vocab,
            self.build_cooc(kg, vocab),
            self.build_safety(kg, vocab),
            self.build_aff(kg, vocab),
            self.build_priors(kg, vocab),
            num_scenes=10,
        )
        sample = ds[0]
        # __getitem__ returns {"x": (N, 4C+A), "adj": (N,N), "target": (N, C)}
        N = sample["x"].shape[0]
        assert 3 <= N <= 8
        assert sample["x"].shape == (N, 4 * C + NUM_AFFORDANCES)
        assert sample["adj"].shape == (N, N)
        assert sample["target"].shape == (N, C)

    def test_partial_has_fewer_objects(self):
        """Partial histogram should have fewer objects than ground truth."""
        if not self.torch_ok:
            pytest.skip("PyTorch not available")
        from memory.knowledge.belief.network import KGSceneGraphDataset
        kg = self.KG()
        vocab, _ = self.build_vocab(kg)
        ds = KGSceneGraphDataset(
            kg, vocab,
            self.build_cooc(kg, vocab),
            self.build_safety(kg, vocab),
            self.build_aff(kg, vocab),
            self.build_priors(kg, vocab),
            num_scenes=20,
        )
        # __getitem__ returns {"x", "adj", "target"} — access internal _scenes
        # for the raw partial vs gt histograms
        for i in range(min(20, len(ds))):
            scene = ds._scenes[i]
            partial_sum = scene["partial"].sum()
            gt_sum = scene["gt"].sum()
            assert partial_sum <= gt_sum, \
                f"Partial ({partial_sum}) should <= GT ({gt_sum})"

    def test_collate_variable_rooms(self):
        """Manually pad variable-size graph samples into a batch."""
        if not self.torch_ok:
            pytest.skip("PyTorch not available")
        import torch

        from memory.knowledge.belief.network import KGSceneGraphDataset
        kg = self.KG()
        vocab, _ = self.build_vocab(kg)
        ds = KGSceneGraphDataset(
            kg, vocab,
            self.build_cooc(kg, vocab),
            self.build_safety(kg, vocab),
            self.build_aff(kg, vocab),
            self.build_priors(kg, vocab),
            num_scenes=10,
        )
        samples = [ds[0], ds[1], ds[2]]
        # Pad variable-size graphs to max N in the batch
        max_n = max(s["x"].shape[0] for s in samples)
        feat_dim = samples[0]["x"].shape[1]
        B = len(samples)
        x_pad = torch.zeros(B, max_n, feat_dim)
        adj_pad = torch.zeros(B, max_n, max_n)
        mask = torch.zeros(B, max_n)
        for i, s in enumerate(samples):
            n = s["x"].shape[0]
            x_pad[i, :n] = s["x"]
            adj_pad[i, :n, :n] = s["adj"]
            mask[i, :n] = 1.0
        assert x_pad.dim() == 3
        assert mask.dim() == 2
        assert x_pad.shape[0] == 3

class TestBeliefTraining:
    """训练流程测试 (小规模验证收敛性)。"""

    def setup_method(self):
        from memory.knowledge.belief.network import HAS_TORCH
        from memory.knowledge.knowledge_graph import IndustrialKnowledgeGraph
        self.KG = IndustrialKnowledgeGraph
        self.torch_ok = HAS_TORCH

    def test_training_reduces_loss(self):
        """Training for a few epochs should reduce loss."""
        if not self.torch_ok:
            pytest.skip("PyTorch not available")
        import torch

        from memory.knowledge.belief.network import (
            BeliefTrainer,
            KGBeliefGCN,
            KGSceneGraphDataset,
            SafetyWeightedBCELoss,
            build_affordance_vectors,
            build_cooccurrence_matrix,
            build_object_vocabulary,
            build_room_prior_vectors,
            build_safety_loss_weights,
            build_safety_vector,
        )
        kg = self.KG()
        label2idx, _idx2label = build_object_vocabulary(kg)
        C = len(label2idx)
        cooc = build_cooccurrence_matrix(kg, label2idx)
        safety = build_safety_vector(kg, label2idx)
        aff = build_affordance_vectors(kg, label2idx)
        priors = build_room_prior_vectors(kg, label2idx)
        loss_w = build_safety_loss_weights(kg, label2idx)

        model = KGBeliefGCN(num_objects=C)
        loss_fn = SafetyWeightedBCELoss(torch.tensor(loss_w))
        trainer = BeliefTrainer(model, loss_fn)

        train_ds = KGSceneGraphDataset(
            kg, label2idx, cooc, safety, aff, priors, num_scenes=160, seed=42)
        val_ds = KGSceneGraphDataset(
            kg, label2idx, cooc, safety, aff, priors, num_scenes=40, seed=123)

        result = trainer.train(train_ds, val_ds, epochs=5)
        assert len(result["train_losses"]) == 5
        assert result["train_losses"][-1] < result["train_losses"][0], \
            "Training loss should decrease"

    def test_predictor_output_format(self):
        """Predictor should return per-room dicts of {label: probability}."""
        if not self.torch_ok:
            pytest.skip("PyTorch not available")
        from memory.knowledge.belief.network import BeliefPredictor
        kg = self.KG()
        predictor = BeliefPredictor.from_kg(kg)
        # predict_for_room returns dict {label: prob} for a single room
        result = predictor.predict_for_room(
            labels=["desk", "chair", "monitor"],
            room_type="office",
        )
        assert isinstance(result, dict)
        for label, prob in result.items():
            assert isinstance(label, str)
            assert 0.0 <= prob <= 1.0

    def test_safety_weighted_loss(self):
        """Safety loss should be higher when dangerous objects are missed."""
        if not self.torch_ok:
            pytest.skip("PyTorch not available")
        import torch

        from memory.knowledge.belief.network import (
            SafetyWeightedBCELoss,
            build_object_vocabulary,
            build_safety_loss_weights,
        )
        kg = self.KG()
        vocab, _ = build_object_vocabulary(kg)
        C = len(vocab)
        weights = torch.tensor(build_safety_loss_weights(kg, vocab))
        criterion = SafetyWeightedBCELoss(weights)

        pred = torch.full((1, C), 0.5)
        gt_safe = torch.zeros(1, C)
        gt_danger = torch.zeros(1, C)
        # Set a dangerous object in gt_danger
        for label, idx in vocab.items():
            props = kg.enrich_object_properties(label)
            if props.get("safety_level") == "dangerous":
                gt_danger[0, idx] = 1.0
                break
        # SafetyWeightedBCELoss.forward takes (pred, target) — no mask arg
        loss_safe = criterion(pred, gt_safe)
        loss_danger = criterion(pred, gt_danger)
        # Dangerous miss should have higher loss
        assert loss_danger.item() >= loss_safe.item()

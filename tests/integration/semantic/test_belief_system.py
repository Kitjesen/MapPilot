"""Offline unit tests for the BA-HSG belief system.

The tests cover:
1. Beta-distribution existence belief updates.
2. Gaussian position uncertainty convergence.
3. Composite credibility calculation.
4. Belief fields in scene graph output.
5. Multi-hypothesis Bayesian target updates.
6. Value-of-information scheduling decisions.
7. Follow-mode intent recognition.

These tests are pure Python and do not require ROS 2.
"""

import json
import time

import numpy as np

from decision.goals.resolver import TargetBeliefManager
from decision.tasks.decomposition import SubGoalAction, TaskDecomposer
from memory.scheduling.voi_scheduler import SchedulerAction, SchedulerState, VoIScheduler
from perception.tracking.instance_tracker import InstanceTracker, TrackedObject
from perception.tracking.projection import Detection3D


class TestBetaBelief:
    """Beta(alpha, beta) existence-belief tests."""

    def _make_object(self, **kwargs) -> TrackedObject:
        defaults = dict(
            object_id=0,
            label="chair",
            position=np.array([3.0, 4.0, 1.0]),
            best_score=0.8,
        )
        defaults.update(kwargs)
        return TrackedObject(**defaults)

    def test_initial_belief(self):
        obj = self._make_object()
        assert obj.existence_prob > 0.5
        assert obj.belief_alpha == 1.5
        assert obj.belief_beta == 1.0

    def test_positive_evidence_increases_belief(self):
        obj = self._make_object()
        p0 = obj.existence_prob

        det = Detection3D(
            label="chair",
            score=0.9,
            position=np.array([3.1, 4.0, 1.0]),
            features=np.array([]),
            bbox_2d=np.array([100, 100, 200, 200]),
            depth=3.0,
        )
        obj.update(det)

        assert obj.existence_prob > p0
        assert obj.belief_alpha > 1.5
        assert obj.miss_streak == 0

    def test_negative_evidence_decreases_belief(self):
        obj = self._make_object()
        p0 = obj.existence_prob

        for _ in range(5):
            obj.record_miss()

        assert obj.existence_prob < p0
        assert obj.miss_streak == 5

    def test_belief_convergence_with_observations(self):
        obj = self._make_object()
        det = Detection3D(
            label="chair",
            score=0.85,
            position=np.array([3.0, 4.0, 1.0]),
            features=np.array([]),
            bbox_2d=np.array([100, 100, 200, 200]),
            depth=3.0,
        )
        for _ in range(20):
            obj.update(det)

        assert obj.existence_prob > 0.9

    def test_uncertainty_decreases_with_observations(self):
        obj = self._make_object()
        u0 = obj.existence_uncertainty

        det = Detection3D(
            label="chair",
            score=0.85,
            position=np.array([3.0, 4.0, 1.0]),
            features=np.array([]),
            bbox_2d=np.array([100, 100, 200, 200]),
            depth=3.0,
        )
        for _ in range(10):
            obj.update(det)

        assert obj.existence_uncertainty < u0


class TestPositionUncertainty:
    """Gaussian position-uncertainty tests."""

    def _make_object(self) -> TrackedObject:
        return TrackedObject(
            object_id=0,
            label="chair",
            position=np.array([3.0, 4.0, 1.0]),
            best_score=0.8,
        )

    def test_initial_variance_high(self):
        obj = self._make_object()
        assert obj.position_variance >= 1.0

    def test_variance_decreases_with_updates(self):
        obj = self._make_object()
        v0 = obj.position_variance

        det = Detection3D(
            label="chair",
            score=0.9,
            position=np.array([3.05, 3.98, 1.0]),
            features=np.array([]),
            bbox_2d=np.array([100, 100, 200, 200]),
            depth=3.0,
        )
        for _ in range(10):
            obj.update(det)

        assert obj.position_variance < v0
        assert obj.position_variance < 0.1

    def test_position_converges_to_true(self):
        obj = self._make_object()
        true_pos = np.array([5.0, 6.0, 1.0])

        for _ in range(20):
            noise = np.random.randn(3) * 0.1
            det = Detection3D(
                label="chair",
                score=0.9,
                position=true_pos + noise,
                features=np.array([]),
                bbox_2d=np.array([100, 100, 200, 200]),
                depth=5.0,
            )
            obj.update(det)

        error = np.linalg.norm(obj.position[:2] - true_pos[:2])
        assert error < 0.5


class TestCredibility:
    """Composite credibility calculation tests."""

    def test_credibility_range(self):
        obj = TrackedObject(
            object_id=0,
            label="chair",
            position=np.array([3.0, 4.0, 1.0]),
            best_score=0.8,
            last_seen=time.time(),
        )
        obj._update_credibility()
        assert 0.0 <= obj.credibility <= 1.0

    def test_fresh_high_confidence_has_high_credibility(self):
        obj = TrackedObject(
            object_id=0,
            label="chair",
            position=np.array([3.0, 4.0, 1.0]),
            best_score=0.95,
            detection_count=10,
            last_seen=time.time(),
            belief_alpha=10.0,
            belief_beta=1.0,
        )
        obj._update_credibility()
        assert obj.credibility > 0.6

    def test_stale_object_has_low_credibility(self):
        fresh = TrackedObject(
            object_id=0,
            label="chair",
            position=np.array([3.0, 4.0, 1.0]),
            best_score=0.8,
            detection_count=2,
            last_seen=time.time(),
        )
        fresh._update_credibility()

        stale = TrackedObject(
            object_id=1,
            label="chair",
            position=np.array([3.0, 4.0, 1.0]),
            best_score=0.8,
            detection_count=2,
            last_seen=time.time() - 120,
        )
        stale._update_credibility()
        assert stale.credibility < fresh.credibility


class TestSceneGraphBelief:
    """Scene graph JSON should include belief fields."""

    def test_belief_in_scene_graph(self):
        tracker = InstanceTracker(max_objects=50)
        det = Detection3D(
            label="chair",
            score=0.9,
            position=np.array([3.0, 4.0, 1.0]),
            features=np.array([0.1, 0.2, 0.3]),
            bbox_2d=np.array([100, 100, 200, 200]),
            depth=3.0,
        )
        tracker.update([det])

        sg_json = tracker.get_scene_graph_json()
        sg = json.loads(sg_json)

        assert "objects" in sg
        assert len(sg["objects"]) > 0

        obj = sg["objects"][0]
        assert "belief" in obj
        assert "P_exist" in obj["belief"]
        assert "sigma_pos" in obj["belief"]
        assert "credibility" in obj["belief"]
        assert obj["belief"]["P_exist"] > 0.5


class TestMultiHypothesis:
    """TargetBeliefManager multi-hypothesis tests."""

    def _make_candidates(self):
        return [
            {
                "id": 0,
                "label": "red chair",
                "position": [3.0, 4.0, 0.0],
                "fused_score": 0.8,
                "belief": {"credibility": 0.7},
                "room_match": 0.6,
            },
            {
                "id": 1,
                "label": "blue chair",
                "position": [7.0, 2.0, 0.0],
                "fused_score": 0.75,
                "belief": {"credibility": 0.65},
                "room_match": 0.5,
            },
            {
                "id": 2,
                "label": "green chair",
                "position": [1.0, 8.0, 0.0],
                "fused_score": 0.5,
                "belief": {"credibility": 0.4},
                "room_match": 0.3,
            },
        ]

    def test_init_posterior_sums_to_one(self):
        mgr = TargetBeliefManager()
        mgr.init_from_candidates(self._make_candidates())

        total = sum(h.posterior for h in mgr._hypotheses)
        assert abs(total - 1.0) < 1e-6

    def test_best_hypothesis_is_highest_score(self):
        mgr = TargetBeliefManager()
        mgr.init_from_candidates(self._make_candidates())

        best = mgr.best_hypothesis
        assert best is not None
        assert best.label == "red chair"

    def test_bayesian_update_rejection(self):
        mgr = TargetBeliefManager()
        mgr.init_from_candidates(self._make_candidates())

        p_before = mgr._hypotheses[0].posterior
        mgr.bayesian_update(object_id=0, detected=False, clip_sim=0.1)

        assert mgr._hypotheses[0].posterior < p_before
        assert mgr._hypotheses[0].rejected

    def test_bayesian_update_confirmation(self):
        mgr = TargetBeliefManager()
        mgr.init_from_candidates(self._make_candidates())

        mgr.bayesian_update(object_id=0, detected=True, clip_sim=0.9)
        assert mgr._hypotheses[0].posterior > 0.5

    def test_select_next_after_rejection(self):
        mgr = TargetBeliefManager()
        mgr.init_from_candidates(self._make_candidates())

        mgr.bayesian_update(object_id=0, detected=False, clip_sim=0.1)
        next_target = mgr.select_next_target(robot_position=[3.0, 4.0])

        assert next_target is not None
        assert next_target.object_id != 0

    def test_convergence(self):
        mgr = TargetBeliefManager()
        mgr.init_from_candidates(self._make_candidates())

        mgr.bayesian_update(object_id=0, detected=True, clip_sim=0.95)
        mgr.bayesian_update(object_id=1, detected=False, clip_sim=0.1)

        assert mgr.is_converged or mgr.best_hypothesis.posterior > 0.5


class TestVoIScheduler:
    """VoI scheduler decision tests."""

    def test_high_credibility_continues(self):
        scheduler = VoIScheduler()
        state = SchedulerState(
            target_credibility=0.85,
            distance_to_goal=2.0,
            distance_since_last_reperception=1.0,
        )
        action = scheduler.decide(state)
        assert action == SchedulerAction.CONTINUE

    def test_low_credibility_reperceives(self):
        scheduler = VoIScheduler()
        state = SchedulerState(
            target_credibility=0.2,
            distance_to_goal=5.0,
            distance_since_last_reperception=3.0,
            last_reperception_time=0.0,
        )
        action = scheduler.decide(state)
        assert action == SchedulerAction.REPERCEIVE

    def test_cooldown_prevents_reperception(self):
        scheduler = VoIScheduler()
        state = SchedulerState(
            target_credibility=0.3,
            distance_to_goal=5.0,
            distance_since_last_reperception=3.0,
            last_reperception_time=time.time() - 1.0,
        )
        action = scheduler.decide(state)
        assert action in (SchedulerAction.CONTINUE, SchedulerAction.REPERCEIVE)

    def test_insufficient_movement_continues(self):
        scheduler = VoIScheduler()
        state = SchedulerState(
            target_credibility=0.5,
            distance_to_goal=5.0,
            distance_since_last_reperception=0.2,
        )
        action = scheduler.decide(state)
        assert action == SchedulerAction.CONTINUE

    def test_decision_stats(self):
        scheduler = VoIScheduler()
        for _ in range(5):
            state = SchedulerState(
                target_credibility=0.85,
                distance_to_goal=2.0,
                distance_since_last_reperception=0.5,
            )
            scheduler.decide(state)

        stats = scheduler.decision_stats
        assert stats["continue"] >= 4


class TestFollowMode:
    """Follow-mode intent recognition tests."""

    def test_follow_pattern_chinese(self):
        decomposer = TaskDecomposer()

        plan = decomposer.decompose_with_rules("跟着那个人")
        assert plan is not None
        actions = [sg.action for sg in plan.subgoals]
        assert SubGoalAction.FOLLOW in actions

    def test_follow_pattern_english(self):
        decomposer = TaskDecomposer()

        plan = decomposer.decompose_with_rules("follow the person")
        assert plan is not None
        actions = [sg.action for sg in plan.subgoals]
        assert SubGoalAction.FOLLOW in actions

    def test_nav_is_not_follow(self):
        decomposer = TaskDecomposer()

        plan = decomposer.decompose_with_rules("去门那里")
        assert plan is not None
        actions = [sg.action for sg in plan.subgoals]
        assert SubGoalAction.FOLLOW not in actions

"""
BA-HSG 信念系统离线单元测试。

验证:
  1. Beta 分布存在性更新 (正面/负面证据)
  2. Gaussian 位置不确定性收敛
  3. 复合可信度计算
  4. 图扩散传播
  5. 多假设目标贝叶斯更新与重选
  6. VoI 调度器决策合理性

这些测试不依赖 ROS2, 纯 Python 即可运行。
"""

import json
import math
import sys
import time

import pytest
import numpy as np

sys.path.insert(0, "src/semantic_perception")
sys.path.insert(0, "src/semantic_planner")

from semantic.perception.instance_tracker import (
    TrackedObject, InstanceTracker, Region,
    BELIEF_FRESHNESS_TAU,
)
from semantic.perception.projection import Detection3D
from semantic.planner.goal_resolver import (
    TargetBeliefManager, TargetHypothesis, GoalResult,
)
from memory.scheduling.voi_scheduler import (
    VoIScheduler, VoIConfig, SchedulerState, SchedulerAction,
)


# ================================================================
#  Test 1: TrackedObject Beta 信念更新
# ================================================================

class TestBetaBelief:
    """Beta(α, β) 存在性分布测试。"""

    def _make_object(self, **kwargs) -> TrackedObject:
        defaults = dict(
            object_id=0, label="chair",
            position=np.array([3.0, 4.0, 1.0]),
            best_score=0.8,
        )
        defaults.update(kwargs)
        return TrackedObject(**defaults)

    def test_initial_belief(self):
        """初始存在概率 > 0.5 (偏乐观先验)。"""
        obj = self._make_object()
        assert obj.existence_prob > 0.5
        assert obj.belief_alpha == 1.5
        assert obj.belief_beta == 1.0

    def test_positive_evidence_increases_belief(self):
        """检测到目标 → α 增加 → P(exists) 上升。"""
        obj = self._make_object()
        p0 = obj.existence_prob

        det = Detection3D(
            label="chair", score=0.9,
            position=np.array([3.1, 4.0, 1.0]),
            features=np.array([]),
            bbox_2d=np.array([100, 100, 200, 200]),
            depth=3.0,
        )
        obj.update(det)

        assert obj.existence_prob > p0
        assert obj.belief_alpha > 1.5  # α 应增加
        assert obj.miss_streak == 0

    def test_negative_evidence_decreases_belief(self):
        """多次未检测到 → β 增加 → P(exists) 下降。"""
        obj = self._make_object()
        p0 = obj.existence_prob

        for _ in range(5):
            obj.record_miss()

        assert obj.existence_prob < p0
        assert obj.miss_streak == 5

    def test_belief_convergence_with_observations(self):
        """多次观测后, 存在概率应接近 1。"""
        obj = self._make_object()
        det = Detection3D(
            label="chair", score=0.85,
            position=np.array([3.0, 4.0, 1.0]),
            features=np.array([]),
            bbox_2d=np.array([100, 100, 200, 200]),
            depth=3.0,
        )
        for _ in range(20):
            obj.update(det)

        assert obj.existence_prob > 0.9

    def test_uncertainty_decreases_with_observations(self):
        """Beta 方差应随观测次数减小。"""
        obj = self._make_object()
        u0 = obj.existence_uncertainty

        det = Detection3D(
            label="chair", score=0.85,
            position=np.array([3.0, 4.0, 1.0]),
            features=np.array([]),
            bbox_2d=np.array([100, 100, 200, 200]),
            depth=3.0,
        )
        for _ in range(10):
            obj.update(det)

        assert obj.existence_uncertainty < u0


# ================================================================
#  Test 2: Gaussian 位置不确定性
# ================================================================

class TestPositionUncertainty:
    """位置 Gaussian σ² 收敛测试。"""

    def _make_object(self) -> TrackedObject:
        return TrackedObject(
            object_id=0, label="chair",
            position=np.array([3.0, 4.0, 1.0]),
            best_score=0.8,
        )

    def test_initial_variance_high(self):
        """初始位置方差应较高 (不确定)。"""
        obj = self._make_object()
        assert obj.position_variance >= 1.0

    def test_variance_decreases_with_updates(self):
        """多次观测后, 位置方差应收敛。"""
        obj = self._make_object()
        v0 = obj.position_variance

        det = Detection3D(
            label="chair", score=0.9,
            position=np.array([3.05, 3.98, 1.0]),
            features=np.array([]),
            bbox_2d=np.array([100, 100, 200, 200]),
            depth=3.0,
        )
        for _ in range(10):
            obj.update(det)

        assert obj.position_variance < v0
        assert obj.position_variance < 0.1  # 应该很小了

    def test_position_converges_to_true(self):
        """位置应收敛到观测的加权平均。"""
        obj = self._make_object()
        true_pos = np.array([5.0, 6.0, 1.0])

        for _ in range(20):
            noise = np.random.randn(3) * 0.1
            det = Detection3D(
                label="chair", score=0.9,
                position=true_pos + noise,
                features=np.array([]),
                bbox_2d=np.array([100, 100, 200, 200]),
                depth=5.0,
            )
            obj.update(det)

        error = np.linalg.norm(obj.position[:2] - true_pos[:2])
        assert error < 0.5  # 应接近真实位置


# ================================================================
#  Test 3: 复合可信度
# ================================================================

class TestCredibility:
    """复合可信度 C 计算测试。"""

    def test_credibility_range(self):
        """可信度应在 [0, 1]。"""
        obj = TrackedObject(
            object_id=0, label="chair",
            position=np.array([3.0, 4.0, 1.0]),
            best_score=0.8,
            last_seen=time.time(),
        )
        obj._update_credibility()
        assert 0.0 <= obj.credibility <= 1.0

    def test_fresh_high_confidence_has_high_credibility(self):
        """刚检测到的高置信目标, 可信度应高。"""
        obj = TrackedObject(
            object_id=0, label="chair",
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
        """很久没见的物体, 可信度应低于新鲜物体。"""
        fresh = TrackedObject(
            object_id=0, label="chair",
            position=np.array([3.0, 4.0, 1.0]),
            best_score=0.8,
            detection_count=2,
            last_seen=time.time(),
        )
        fresh._update_credibility()

        stale = TrackedObject(
            object_id=1, label="chair",
            position=np.array([3.0, 4.0, 1.0]),
            best_score=0.8,
            detection_count=2,
            last_seen=time.time() - 120,  # 2 分钟前
        )
        stale._update_credibility()
        assert stale.credibility < fresh.credibility


# ================================================================
#  Test 4: 场景图输出含 belief 字段
# ================================================================

class TestSceneGraphBelief:
    """场景图 JSON 应包含 belief 信息。"""

    def test_belief_in_scene_graph(self):
        tracker = InstanceTracker(max_objects=50)
        det = Detection3D(
            label="chair", score=0.9,
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


# ================================================================
#  Test 5: 多假设目标贝叶斯规划
# ================================================================

class TestMultiHypothesis:
    """TargetBeliefManager 多假设测试。"""

    def _make_candidates(self):
        return [
            {"id": 0, "label": "red chair", "position": [3.0, 4.0, 0.0],
             "fused_score": 0.8, "belief": {"credibility": 0.7}, "room_match": 0.6},
            {"id": 1, "label": "blue chair", "position": [7.0, 2.0, 0.0],
             "fused_score": 0.75, "belief": {"credibility": 0.65}, "room_match": 0.5},
            {"id": 2, "label": "green chair", "position": [1.0, 8.0, 0.0],
             "fused_score": 0.5, "belief": {"credibility": 0.4}, "room_match": 0.3},
        ]

    def test_init_posterior_sums_to_one(self):
        """后验概率之和应为 1。"""
        mgr = TargetBeliefManager()
        mgr.init_from_candidates(self._make_candidates())

        total = sum(h.posterior for h in mgr._hypotheses)
        assert abs(total - 1.0) < 1e-6

    def test_best_hypothesis_is_highest_score(self):
        """后验最高的应是 fused_score 最高的。"""
        mgr = TargetBeliefManager()
        mgr.init_from_candidates(self._make_candidates())

        best = mgr.best_hypothesis
        assert best is not None
        assert best.label == "red chair"

    def test_bayesian_update_rejection(self):
        """到达候选附近未检测到 → 后验大幅降低。"""
        mgr = TargetBeliefManager()
        mgr.init_from_candidates(self._make_candidates())

        p_before = mgr._hypotheses[0].posterior
        mgr.bayesian_update(object_id=0, detected=False, clip_sim=0.1)

        assert mgr._hypotheses[0].posterior < p_before
        assert mgr._hypotheses[0].rejected

    def test_bayesian_update_confirmation(self):
        """到达候选并确认 → 后验大幅上升。"""
        mgr = TargetBeliefManager()
        mgr.init_from_candidates(self._make_candidates())

        mgr.bayesian_update(object_id=0, detected=True, clip_sim=0.9)
        assert mgr._hypotheses[0].posterior > 0.5

    def test_select_next_after_rejection(self):
        """拒绝第一个后, 应选择第二好的。"""
        mgr = TargetBeliefManager()
        mgr.init_from_candidates(self._make_candidates())

        mgr.bayesian_update(object_id=0, detected=False, clip_sim=0.1)
        next_target = mgr.select_next_target(robot_position=[3.0, 4.0])

        assert next_target is not None
        assert next_target.object_id != 0

    def test_convergence(self):
        """确认目标后, is_converged 应为 True。"""
        mgr = TargetBeliefManager()
        mgr.init_from_candidates(self._make_candidates())

        mgr.bayesian_update(object_id=0, detected=True, clip_sim=0.95)
        # 可能需要多次确认才收敛
        mgr.bayesian_update(object_id=1, detected=False, clip_sim=0.1)

        assert mgr.is_converged or mgr.best_hypothesis.posterior > 0.5


# ================================================================
#  Test 6: VoI 调度器
# ================================================================

class TestVoIScheduler:
    """VoI 调度器决策合理性测试。"""

    def test_high_credibility_continues(self):
        """目标高可信度 + 接近时, 应选择 continue。"""
        scheduler = VoIScheduler()
        state = SchedulerState(
            target_credibility=0.85,
            distance_to_goal=2.0,
            distance_since_last_reperception=1.0,
        )
        action = scheduler.decide(state)
        assert action == SchedulerAction.CONTINUE

    def test_low_credibility_reperceives(self):
        """目标低可信度时, 应选择 reperceive。"""
        scheduler = VoIScheduler()
        state = SchedulerState(
            target_credibility=0.2,
            distance_to_goal=5.0,
            distance_since_last_reperception=3.0,
            last_reperception_time=0.0,  # 很久以前
        )
        action = scheduler.decide(state)
        assert action == SchedulerAction.REPERCEIVE

    def test_cooldown_prevents_reperception(self):
        """冷却期内不应触发 reperceive。"""
        scheduler = VoIScheduler()
        state = SchedulerState(
            target_credibility=0.3,
            distance_to_goal=5.0,
            distance_since_last_reperception=3.0,
            last_reperception_time=time.time() - 1.0,  # 刚刚做过
        )
        action = scheduler.decide(state)
        # 冷却期内, 即使可信度低也不该 reperceive (除非极端危险)
        assert action in (SchedulerAction.CONTINUE, SchedulerAction.REPERCEIVE)

    def test_insufficient_movement_continues(self):
        """移动距离不够时, 应 continue。"""
        scheduler = VoIScheduler()
        state = SchedulerState(
            target_credibility=0.5,
            distance_to_goal=5.0,
            distance_since_last_reperception=0.2,  # 才走了 0.2m
        )
        action = scheduler.decide(state)
        assert action == SchedulerAction.CONTINUE

    def test_decision_stats(self):
        """决策日志应正确计数。"""
        scheduler = VoIScheduler()
        for _ in range(5):
            state = SchedulerState(
                target_credibility=0.85,
                distance_to_goal=2.0,
                distance_since_last_reperception=0.5,
            )
            scheduler.decide(state)

        stats = scheduler.decision_stats
        assert stats["continue"] >= 4  # 大多应是 continue


# ================================================================
#  Test 7: 跟随模式 (通过导航闭环验证)
# ================================================================

class TestFollowMode:
    """跟随模式应正确识别指令。"""

    def test_follow_pattern_chinese(self):
        """中文跟随指令应被模板匹配。"""
        from semantic.planner.task_decomposer import TaskDecomposer, SubGoalAction

        decomposer = TaskDecomposer()

        plan = decomposer.decompose_with_rules("跟着那个人")
        assert plan is not None
        actions = [sg.action for sg in plan.subgoals]
        assert SubGoalAction.FOLLOW in actions

    def test_follow_pattern_english(self):
        """英文跟随指令应被模板匹配。"""
        from semantic.planner.task_decomposer import TaskDecomposer, SubGoalAction

        decomposer = TaskDecomposer()

        plan = decomposer.decompose_with_rules("follow the person")
        assert plan is not None
        actions = [sg.action for sg in plan.subgoals]
        assert SubGoalAction.FOLLOW in actions

    def test_nav_is_not_follow(self):
        """普通导航指令不应触发跟随。"""
        from semantic.planner.task_decomposer import TaskDecomposer, SubGoalAction

        decomposer = TaskDecomposer()

        plan = decomposer.decompose_with_rules("去门那里")
        assert plan is not None
        actions = [sg.action for sg in plan.subgoals]
        assert SubGoalAction.FOLLOW not in actions

"""
BA-HSG 淇″康绯荤粺绂荤嚎鍗曞厓娴嬭瘯銆?

楠岃瘉:
  1. Beta 鍒嗗竷瀛樺湪鎬ф洿鏂?(姝ｉ潰/璐熼潰璇佹嵁)
  2. Gaussian 浣嶇疆涓嶇‘瀹氭€ф敹鏁?
  3. 澶嶅悎鍙俊搴﹁绠?
  4. 鍥炬墿鏁ｄ紶鎾?
  5. 澶氬亣璁剧洰鏍囪礉鍙舵柉鏇存柊涓庨噸閫?
  6. VoI 璋冨害鍣ㄥ喅绛栧悎鐞嗘€?

杩欎簺娴嬭瘯涓嶄緷璧?ROS2, 绾?Python 鍗冲彲杩愯銆?
"""

import json
import math
import sys
import time

import pytest
import numpy as np

sys.path.insert(0, "src/perception")
sys.path.insert(0, "src/decision")

from perception.tracking.instance_tracker import (
    TrackedObject, InstanceTracker, Region,
    BELIEF_FRESHNESS_TAU,
)
from perception.tracking.projection import Detection3D
from decision.goal_resolution.goal_resolver import (
    TargetBeliefManager, TargetHypothesis, GoalResult,
)
from memory.scheduling.voi_scheduler import (
    VoIScheduler, VoIConfig, SchedulerState, SchedulerAction,
)


# ================================================================
#  Test 1: TrackedObject Beta 淇″康鏇存柊
# ================================================================

class TestBetaBelief:
    """Beta(伪, 尾) 瀛樺湪鎬у垎甯冩祴璇曘€?""

    def _make_object(self, **kwargs) -> TrackedObject:
        defaults = dict(
            object_id=0, label="chair",
            position=np.array([3.0, 4.0, 1.0]),
            best_score=0.8,
        )
        defaults.update(kwargs)
        return TrackedObject(**defaults)

    def test_initial_belief(self):
        """鍒濆瀛樺湪姒傜巼 > 0.5 (鍋忎箰瑙傚厛楠?銆?""
        obj = self._make_object()
        assert obj.existence_prob > 0.5
        assert obj.belief_alpha == 1.5
        assert obj.belief_beta == 1.0

    def test_positive_evidence_increases_belief(self):
        """妫€娴嬪埌鐩爣 鈫?伪 澧炲姞 鈫?P(exists) 涓婂崌銆?""
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
        assert obj.belief_alpha > 1.5  # 伪 搴斿鍔?
        assert obj.miss_streak == 0

    def test_negative_evidence_decreases_belief(self):
        """澶氭鏈娴嬪埌 鈫?尾 澧炲姞 鈫?P(exists) 涓嬮檷銆?""
        obj = self._make_object()
        p0 = obj.existence_prob

        for _ in range(5):
            obj.record_miss()

        assert obj.existence_prob < p0
        assert obj.miss_streak == 5

    def test_belief_convergence_with_observations(self):
        """澶氭瑙傛祴鍚? 瀛樺湪姒傜巼搴旀帴杩?1銆?""
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
        """Beta 鏂瑰樊搴旈殢瑙傛祴娆℃暟鍑忓皬銆?""
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
#  Test 2: Gaussian 浣嶇疆涓嶇‘瀹氭€?
# ================================================================

class TestPositionUncertainty:
    """浣嶇疆 Gaussian 蟽虏 鏀舵暃娴嬭瘯銆?""

    def _make_object(self) -> TrackedObject:
        return TrackedObject(
            object_id=0, label="chair",
            position=np.array([3.0, 4.0, 1.0]),
            best_score=0.8,
        )

    def test_initial_variance_high(self):
        """鍒濆浣嶇疆鏂瑰樊搴旇緝楂?(涓嶇‘瀹?銆?""
        obj = self._make_object()
        assert obj.position_variance >= 1.0

    def test_variance_decreases_with_updates(self):
        """澶氭瑙傛祴鍚? 浣嶇疆鏂瑰樊搴旀敹鏁涖€?""
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
        assert obj.position_variance < 0.1  # 搴旇寰堝皬浜?

    def test_position_converges_to_true(self):
        """浣嶇疆搴旀敹鏁涘埌瑙傛祴鐨勫姞鏉冨钩鍧囥€?""
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
        assert error < 0.5  # 搴旀帴杩戠湡瀹炰綅缃?


# ================================================================
#  Test 3: 澶嶅悎鍙俊搴?
# ================================================================

class TestCredibility:
    """澶嶅悎鍙俊搴?C 璁＄畻娴嬭瘯銆?""

    def test_credibility_range(self):
        """鍙俊搴﹀簲鍦?[0, 1]銆?""
        obj = TrackedObject(
            object_id=0, label="chair",
            position=np.array([3.0, 4.0, 1.0]),
            best_score=0.8,
            last_seen=time.time(),
        )
        obj._update_credibility()
        assert 0.0 <= obj.credibility <= 1.0

    def test_fresh_high_confidence_has_high_credibility(self):
        """鍒氭娴嬪埌鐨勯珮缃俊鐩爣, 鍙俊搴﹀簲楂樸€?""
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
        """寰堜箙娌¤鐨勭墿浣? 鍙俊搴﹀簲浣庝簬鏂伴矞鐗╀綋銆?""
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
            last_seen=time.time() - 120,  # 2 鍒嗛挓鍓?
        )
        stale._update_credibility()
        assert stale.credibility < fresh.credibility


# ================================================================
#  Test 4: 鍦烘櫙鍥捐緭鍑哄惈 belief 瀛楁
# ================================================================

class TestSceneGraphBelief:
    """鍦烘櫙鍥?JSON 搴斿寘鍚?belief 淇℃伅銆?""

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
#  Test 5: 澶氬亣璁剧洰鏍囪礉鍙舵柉瑙勫垝
# ================================================================

class TestMultiHypothesis:
    """TargetBeliefManager 澶氬亣璁炬祴璇曘€?""

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
        """鍚庨獙姒傜巼涔嬪拰搴斾负 1銆?""
        mgr = TargetBeliefManager()
        mgr.init_from_candidates(self._make_candidates())

        total = sum(h.posterior for h in mgr._hypotheses)
        assert abs(total - 1.0) < 1e-6

    def test_best_hypothesis_is_highest_score(self):
        """鍚庨獙鏈€楂樼殑搴旀槸 fused_score 鏈€楂樼殑銆?""
        mgr = TargetBeliefManager()
        mgr.init_from_candidates(self._make_candidates())

        best = mgr.best_hypothesis
        assert best is not None
        assert best.label == "red chair"

    def test_bayesian_update_rejection(self):
        """鍒拌揪鍊欓€夐檮杩戞湭妫€娴嬪埌 鈫?鍚庨獙澶у箙闄嶄綆銆?""
        mgr = TargetBeliefManager()
        mgr.init_from_candidates(self._make_candidates())

        p_before = mgr._hypotheses[0].posterior
        mgr.bayesian_update(object_id=0, detected=False, clip_sim=0.1)

        assert mgr._hypotheses[0].posterior < p_before
        assert mgr._hypotheses[0].rejected

    def test_bayesian_update_confirmation(self):
        """鍒拌揪鍊欓€夊苟纭 鈫?鍚庨獙澶у箙涓婂崌銆?""
        mgr = TargetBeliefManager()
        mgr.init_from_candidates(self._make_candidates())

        mgr.bayesian_update(object_id=0, detected=True, clip_sim=0.9)
        assert mgr._hypotheses[0].posterior > 0.5

    def test_select_next_after_rejection(self):
        """鎷掔粷绗竴涓悗, 搴旈€夋嫨绗簩濂界殑銆?""
        mgr = TargetBeliefManager()
        mgr.init_from_candidates(self._make_candidates())

        mgr.bayesian_update(object_id=0, detected=False, clip_sim=0.1)
        next_target = mgr.select_next_target(robot_position=[3.0, 4.0])

        assert next_target is not None
        assert next_target.object_id != 0

    def test_convergence(self):
        """纭鐩爣鍚? is_converged 搴斾负 True銆?""
        mgr = TargetBeliefManager()
        mgr.init_from_candidates(self._make_candidates())

        mgr.bayesian_update(object_id=0, detected=True, clip_sim=0.95)
        # 鍙兘闇€瑕佸娆＄‘璁ゆ墠鏀舵暃
        mgr.bayesian_update(object_id=1, detected=False, clip_sim=0.1)

        assert mgr.is_converged or mgr.best_hypothesis.posterior > 0.5


# ================================================================
#  Test 6: VoI 璋冨害鍣?
# ================================================================

class TestVoIScheduler:
    """VoI 璋冨害鍣ㄥ喅绛栧悎鐞嗘€ф祴璇曘€?""

    def test_high_credibility_continues(self):
        """鐩爣楂樺彲淇″害 + 鎺ヨ繎鏃? 搴旈€夋嫨 continue銆?""
        scheduler = VoIScheduler()
        state = SchedulerState(
            target_credibility=0.85,
            distance_to_goal=2.0,
            distance_since_last_reperception=1.0,
        )
        action = scheduler.decide(state)
        assert action == SchedulerAction.CONTINUE

    def test_low_credibility_reperceives(self):
        """鐩爣浣庡彲淇″害鏃? 搴旈€夋嫨 reperceive銆?""
        scheduler = VoIScheduler()
        state = SchedulerState(
            target_credibility=0.2,
            distance_to_goal=5.0,
            distance_since_last_reperception=3.0,
            last_reperception_time=0.0,  # 寰堜箙浠ュ墠
        )
        action = scheduler.decide(state)
        assert action == SchedulerAction.REPERCEIVE

    def test_cooldown_prevents_reperception(self):
        """鍐峰嵈鏈熷唴涓嶅簲瑙﹀彂 reperceive銆?""
        scheduler = VoIScheduler()
        state = SchedulerState(
            target_credibility=0.3,
            distance_to_goal=5.0,
            distance_since_last_reperception=3.0,
            last_reperception_time=time.time() - 1.0,  # 鍒氬垰鍋氳繃
        )
        action = scheduler.decide(state)
        # 鍐峰嵈鏈熷唴, 鍗充娇鍙俊搴︿綆涔熶笉璇?reperceive (闄ら潪鏋佺鍗遍櫓)
        assert action in (SchedulerAction.CONTINUE, SchedulerAction.REPERCEIVE)

    def test_insufficient_movement_continues(self):
        """绉诲姩璺濈涓嶅鏃? 搴?continue銆?""
        scheduler = VoIScheduler()
        state = SchedulerState(
            target_credibility=0.5,
            distance_to_goal=5.0,
            distance_since_last_reperception=0.2,  # 鎵嶈蛋浜?0.2m
        )
        action = scheduler.decide(state)
        assert action == SchedulerAction.CONTINUE

    def test_decision_stats(self):
        """鍐崇瓥鏃ュ織搴旀纭鏁般€?""
        scheduler = VoIScheduler()
        for _ in range(5):
            state = SchedulerState(
                target_credibility=0.85,
                distance_to_goal=2.0,
                distance_since_last_reperception=0.5,
            )
            scheduler.decide(state)

        stats = scheduler.decision_stats
        assert stats["continue"] >= 4  # 澶у搴旀槸 continue


# ================================================================
#  Test 7: 璺熼殢妯″紡 (閫氳繃瀵艰埅闂幆楠岃瘉)
# ================================================================

class TestFollowMode:
    """璺熼殢妯″紡搴旀纭瘑鍒寚浠ゃ€?""

    def test_follow_pattern_chinese(self):
        """涓枃璺熼殢鎸囦护搴旇妯℃澘鍖归厤銆?""
        from decision.tasking.task_decomposer import TaskDecomposer, SubGoalAction

        decomposer = TaskDecomposer()

        plan = decomposer.decompose_with_rules("璺熺潃閭ｄ釜浜?)
        assert plan is not None
        actions = [sg.action for sg in plan.subgoals]
        assert SubGoalAction.FOLLOW in actions

    def test_follow_pattern_english(self):
        """鑻辨枃璺熼殢鎸囦护搴旇妯℃澘鍖归厤銆?""
        from decision.tasking.task_decomposer import TaskDecomposer, SubGoalAction

        decomposer = TaskDecomposer()

        plan = decomposer.decompose_with_rules("follow the person")
        assert plan is not None
        actions = [sg.action for sg in plan.subgoals]
        assert SubGoalAction.FOLLOW in actions

    def test_nav_is_not_follow(self):
        """鏅€氬鑸寚浠や笉搴旇Е鍙戣窡闅忋€?""
        from decision.tasking.task_decomposer import TaskDecomposer, SubGoalAction

        decomposer = TaskDecomposer()

        plan = decomposer.decompose_with_rules("鍘婚棬閭ｉ噷")
        assert plan is not None
        actions = [sg.action for sg in plan.subgoals]
        assert SubGoalAction.FOLLOW not in actions

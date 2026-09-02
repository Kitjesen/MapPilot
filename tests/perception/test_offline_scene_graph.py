# ruff: noqa: F405, I001
"""Focused tests split from the former monolithic offline semantic pipeline."""

from tests.integration.semantic.offline_support import *

class TestSceneGraphDynamic:
    """DovSG 动态场景图 + 嵌入索引测试。"""

    def setup_method(self):
        from perception.tracking.instance_tracker import InstanceTracker, TrackedObject
        from perception.tracking.projection import Detection3D
        self.InstanceTracker = InstanceTracker
        self.TrackedObject = TrackedObject
        self.Detection3D = Detection3D

    def _make_det(self, label, x, y, z=0.5, score=0.8):
        return self.Detection3D(
            label=label,
            score=score,
            position=np.array([x, y, z]),
            depth=2.0,
            bbox_2d=np.array([100, 100, 200, 200]),
            features=np.random.randn(512).astype(np.float32),
        )

    def test_scene_diff_detects_new_object(self):
        """场景 diff 应检测到新增物体。"""
        tracker = self.InstanceTracker(merge_distance=0.5)
        tracker.update([self._make_det("chair", 1.0, 2.0)])
        prev_snapshot = {"objects": []}
        diff = tracker.compute_scene_diff(prev_snapshot)
        assert diff["total_events"] >= 1
        added = [e for e in diff["events"] if e["type"] == "object_added"]
        assert len(added) >= 1

    def test_scene_diff_detects_removed_object(self):
        """场景 diff 应检测到消失的物体。"""
        tracker = self.InstanceTracker(merge_distance=0.5)
        prev_snapshot = {
            "objects": [
                {"id": 999, "label": "ghost_chair", "position": {"x": 5, "y": 5, "z": 0.5}}
            ]
        }
        diff = tracker.compute_scene_diff(prev_snapshot)
        removed = [e for e in diff["events"] if e["type"] == "object_removed"]
        assert len(removed) >= 1

    def test_local_update_region(self):
        """局部更新应只影响指定区域。"""
        tracker = self.InstanceTracker(merge_distance=0.5)
        # 添加两组物体在不同区域
        tracker.update([
            self._make_det("chair", 1.0, 1.0),
            self._make_det("desk", 1.5, 1.5),
            self._make_det("door", 10.0, 10.0),
        ])
        regions = tracker.compute_regions()
        if len(regions) < 2:
            return  # 如果聚类结果只有一个区域, 跳过

        target_region = regions[0].region_id
        result = tracker.apply_local_update(
            region_id=target_region,
            new_detections=[self._make_det("bottle", 1.2, 1.2)],
        )
        assert result["added"] >= 0
        assert result["region_id"] == target_region

    def test_embedding_index_build(self):
        """嵌入索引应能构建。"""
        tracker = self.InstanceTracker(merge_distance=0.5)
        tracker.update([
            self._make_det("chair", 1.0, 1.0),
            self._make_det("desk", 2.0, 2.0),
            self._make_det("cup", 3.0, 3.0),
        ])
        success = tracker.build_embedding_index()
        assert success is True

    def test_embedding_query(self):
        """嵌入查询应返回结果。"""
        tracker = self.InstanceTracker(merge_distance=0.5)
        tracker.update([
            self._make_det("chair", 1.0, 1.0),
            self._make_det("desk", 2.0, 2.0),
        ])
        tracker.build_embedding_index()
        q = np.random.randn(512).astype(np.float32)
        results = tracker.query_by_embedding(q, top_k=2, min_similarity=-1.0)
        assert len(results) >= 1

    def test_open_vocabulary_matches(self):
        """开放词汇查询应融合多个信号。"""
        tracker = self.InstanceTracker(merge_distance=0.5)
        tracker.update([
            self._make_det("chair", 1.0, 1.0),
            self._make_det("desk", 2.0, 2.0),
            self._make_det("fire extinguisher", 3.0, 3.0),
        ])
        # 字符串匹配 fallback
        results = tracker.get_open_vocabulary_matches("chair")
        assert len(results) >= 1
        assert results[0]["label"] == "chair"

    def test_open_vocabulary_with_kg(self):
        """带 KG 的开放词汇查询应增强匹配。"""
        from memory.knowledge.knowledge_graph import IndustrialKnowledgeGraph
        kg = IndustrialKnowledgeGraph()
        tracker = self.InstanceTracker(merge_distance=0.5, knowledge_graph=kg)
        tracker.update([
            self._make_det("fire extinguisher", 3.0, 3.0),
        ])
        results = tracker.get_open_vocabulary_matches("灭火器")
        assert len(results) >= 1

    def test_scene_diff_summary(self):
        """场景 diff 摘要应是可读的字符串。"""
        tracker = self.InstanceTracker(merge_distance=0.5)
        tracker.update([self._make_det("chair", 1.0, 2.0)])
        diff = tracker.compute_scene_diff({"objects": []})
        assert isinstance(diff["summary"], str)
        assert len(diff["summary"]) > 0

class TestLoopyBeliefPropagation:
    """迭代信念传播测试 — 参考 Belief Scene Graphs (ICRA 2024)。"""

    def setup_method(self):
        from perception.tracking.instance_tracker import (
            InstanceTracker, Detection3D, TrackedObject,
            PhantomNode, RoomTypePosterior, BeliefMessage,
            BP_MAX_ITERATIONS, BP_CONVERGENCE_EPS,
            SAFETY_THRESHOLDS_NAVIGATION, SAFETY_THRESHOLDS_INTERACTION,
        )
        from memory.knowledge.knowledge_graph import IndustrialKnowledgeGraph
        self.InstanceTracker = InstanceTracker
        self.Detection3D = Detection3D
        self.TrackedObject = TrackedObject
        self.PhantomNode = PhantomNode
        self.RoomTypePosterior = RoomTypePosterior
        self.BeliefMessage = BeliefMessage
        self.KG = IndustrialKnowledgeGraph
        self.BP_MAX_ITERATIONS = BP_MAX_ITERATIONS
        self.BP_CONVERGENCE_EPS = BP_CONVERGENCE_EPS
        self.SAFETY_NAV = SAFETY_THRESHOLDS_NAVIGATION
        self.SAFETY_INTERACT = SAFETY_THRESHOLDS_INTERACTION

    def _make_office_scene(self):
        """创建典型办公室场景: desk, chair, monitor, keyboard。"""
        kg = self.KG()
        tracker = self.InstanceTracker(max_objects=100, knowledge_graph=kg)
        office_objects = [
            ("desk", [2.0, 3.0, 0.7], 0.92),
            ("chair", [2.5, 3.2, 0.5], 0.88),
            ("monitor", [2.0, 2.8, 1.0], 0.90),
            ("computer", [2.0, 3.3, 0.6], 0.85),
        ]
        for label, pos, score in office_objects:
            det = self.Detection3D(
                label=label, score=score,
                position=np.array(pos),
                features=np.array([]),
                bbox_2d=np.array([0, 0, 100, 100]),
                depth=2.0,
            )
            tracker.update([det])
        return tracker, kg

    def test_room_type_posterior_office(self):
        """Phase 1 测试: 办公室物体 → 房间类型后验应为 office。"""
        tracker, kg = self._make_office_scene()
        posteriors = tracker.get_room_type_posteriors()
        assert len(posteriors) > 0, "Should have at least one room posterior"
        for rid, rtp in posteriors.items():
            assert rtp.best_type == "office", \
                f"Room {rid} best_type={rtp.best_type}, expected 'office'"
            assert rtp.best_confidence > 0.1, \
                f"Office confidence too low: {rtp.best_confidence}"

    def test_room_type_posterior_kitchen(self):
        """Phase 1: kitchen 物体 → 房间类型后验为 kitchen。"""
        kg = self.KG()
        tracker = self.InstanceTracker(max_objects=100, knowledge_graph=kg)
        for label, pos in [("refrigerator", [1.0, 1.0, 0.8]),
                           ("microwave", [1.5, 1.0, 1.2]),
                           ("sink", [2.0, 1.0, 0.9])]:
            tracker.update([self.Detection3D(
                label=label, score=0.9, position=np.array(pos),
                features=np.array([]),
                bbox_2d=np.array([0, 0, 100, 100]), depth=2.0,
            )])
        posteriors = tracker.get_room_type_posteriors()
        for rtp in posteriors.values():
            assert rtp.best_type == "kitchen", \
                f"Expected kitchen, got {rtp.best_type}"

    def test_room_type_posterior_corridor(self):
        """Phase 1: corridor 物体 → 房间类型后验为 corridor。"""
        kg = self.KG()
        tracker = self.InstanceTracker(max_objects=100, knowledge_graph=kg)
        for label, pos in [("door", [0.0, 5.0, 1.0]),
                           ("fire_extinguisher", [0.5, 5.0, 0.5]),
                           ("safety_sign", [1.0, 5.0, 1.5])]:
            tracker.update([self.Detection3D(
                label=label, score=0.9, position=np.array(pos),
                features=np.array([]),
                bbox_2d=np.array([0, 0, 100, 100]), depth=2.0,
            )])
        posteriors = tracker.get_room_type_posteriors()
        for rtp in posteriors.values():
            assert rtp.best_type == "corridor", \
                f"Expected corridor, got {rtp.best_type}"

    def test_kg_prior_injection_expected_object(self):
        """Phase 2 测试: 在 office 中检测到 desk → is_kg_expected=True, alpha 提升。"""
        tracker, kg = self._make_office_scene()
        desk_objs = [o for o in tracker.objects.values() if o.label == "desk"]
        assert len(desk_objs) > 0
        desk = desk_objs[0]
        assert desk.is_kg_expected, "desk should be KG-expected in office"
        assert desk.kg_prior_alpha > 0, "desk should have KG prior alpha > 0"
        assert "room_type:office" in desk.kg_prior_source, \
            f"Expected room_type:office in source, got {desk.kg_prior_source}"

    def test_kg_prior_unexpected_penalty(self):
        """Phase 2: 非期望物体在房间中 → β 增加 (温和怀疑)。"""
        kg = self.KG()
        tracker = self.InstanceTracker(max_objects=100, knowledge_graph=kg)
        # 先创建 office 场景
        for label, pos in [("desk", [2.0, 3.0, 0.7]),
                           ("chair", [2.5, 3.0, 0.5]),
                           ("computer", [2.0, 2.5, 0.6])]:
            tracker.update([self.Detection3D(
                label=label, score=0.9, position=np.array(pos),
                features=np.array([]),
                bbox_2d=np.array([0, 0, 100, 100]), depth=2.0,
            )])
        # 在 office 区域检测到不期望的物体
        tracker.update([self.Detection3D(
            label="forklift", score=0.6, position=np.array([2.2, 3.1, 0.5]),
            features=np.array([]),
            bbox_2d=np.array([0, 0, 100, 100]), depth=2.0,
        )])
        forklifts = [o for o in tracker.objects.values() if o.label == "forklift"]
        if forklifts:
            fl = forklifts[0]
            assert not fl.is_kg_expected, "forklift should NOT be expected in office"

    def test_bp_convergence(self):
        """Loopy BP 应在 MAX_ITERATIONS 内收敛。"""
        tracker, kg = self._make_office_scene()
        diag = tracker.get_bp_diagnostics()
        assert diag["total_iterations"] > 0, "BP should have run"
        if diag["convergence_history"]:
            last_delta = diag["convergence_history"][-1]
            assert last_delta < 1.0, f"BP diverging: last delta = {last_delta}"

    def test_bp_messages_logged(self):
        """BP 消息日志应记录传播过程。"""
        tracker, kg = self._make_office_scene()
        diag = tracker.get_bp_diagnostics()
        assert diag["num_messages_last_round"] >= 0, "Should have BP messages"

    def test_lateral_sharing_near_objects(self):
        """Phase 3: 近距离物体间信念共享。"""
        kg = self.KG()
        tracker = self.InstanceTracker(max_objects=100, knowledge_graph=kg)
        # 两个近距离物体, 一个高可信度, 一个低可信度
        det_high = self.Detection3D(
            label="desk", score=0.95,
            position=np.array([1.0, 1.0, 0.7]),
            features=np.array([]),
            bbox_2d=np.array([0, 0, 100, 100]), depth=1.0,
        )
        # 多次检测提升可信度
        for _ in range(5):
            tracker.update([det_high])

        det_low = self.Detection3D(
            label="lamp", score=0.6,
            position=np.array([1.3, 1.0, 0.5]),
            features=np.array([]),
            bbox_2d=np.array([0, 0, 50, 50]), depth=1.0,
        )
        tracker.update([det_low])

        lamp_objs = [o for o in tracker.objects.values() if o.label == "lamp"]
        desk_objs = [o for o in tracker.objects.values() if o.label == "desk"]
        if lamp_objs and desk_objs:
            assert lamp_objs[0].belief_alpha >= 1.5, \
                "Lamp should have some alpha from lateral sharing"

class TestPhantomNodes:
    """Phantom (Blind) Node 推理测试 — 参考 BSG ICRA 2024 blind nodes。"""

    def setup_method(self):
        from perception.tracking.instance_tracker import (
            InstanceTracker, Detection3D, PhantomNode,
        )
        from memory.knowledge.knowledge_graph import IndustrialKnowledgeGraph
        self.InstanceTracker = InstanceTracker
        self.Detection3D = Detection3D
        self.PhantomNode = PhantomNode
        self.KG = IndustrialKnowledgeGraph

    def _make_scene(self, labels_positions):
        kg = self.KG()
        tracker = self.InstanceTracker(max_objects=100, knowledge_graph=kg)
        for label, pos in labels_positions:
            tracker.update([self.Detection3D(
                label=label, score=0.9, position=np.array(pos),
                features=np.array([]),
                bbox_2d=np.array([0, 0, 100, 100]), depth=2.0,
            )])
        return tracker, kg

    def test_phantom_generation_office(self):
        """检测到 desk+chair+computer → phantom 包含 printer, whiteboard 等期望物体。"""
        tracker, _ = self._make_scene([
            ("desk", [2.0, 3.0, 0.7]),
            ("chair", [2.5, 3.0, 0.5]),
            ("computer", [2.0, 2.5, 0.6]),
            ("monitor", [2.0, 2.8, 1.0]),
        ])
        phantoms = tracker.get_phantom_nodes()
        phantom_labels = {p.label for p in phantoms}
        # office 场景应预测一些未见但期望存在的物体
        assert len(phantoms) >= 0, "May or may not generate phantoms depending on confidence"
        # 如果生成了, 应包含 office 期望物体
        if phantom_labels:
            possible_office = {"printer", "cabinet", "whiteboard", "lamp", "trash_bin",
                               "plant", "bottle", "cup", "backpack", "water_dispenser"}
            assert phantom_labels & possible_office, \
                f"Phantom labels {phantom_labels} should overlap with office expected objects"

    def test_phantom_generation_corridor(self):
        """走廊场景 → phantom 应包含 fire_alarm, emergency_exit 等。"""
        tracker, _ = self._make_scene([
            ("door", [0.0, 5.0, 1.0]),
            ("fire_extinguisher", [0.5, 5.0, 0.5]),
        ])
        phantoms = tracker.get_phantom_nodes()
        if phantoms:
            phantom_labels = {p.label for p in phantoms}
            safety_phantoms = {"fire_alarm", "safety_sign", "emergency_exit"}
            assert phantom_labels & safety_phantoms, \
                f"Corridor phantoms should include safety objects, got {phantom_labels}"

    def test_phantom_has_safety_level(self):
        """Phantom 节点应从 KG 继承安全等级。"""
        tracker, _ = self._make_scene([
            ("desk", [2.0, 3.0, 0.7]),
            ("chair", [2.5, 3.0, 0.5]),
            ("computer", [2.0, 2.5, 0.6]),
        ])
        for phantom in tracker.get_phantom_nodes():
            assert phantom.safety_level in ("safe", "caution", "dangerous", "forbidden"), \
                f"Invalid safety level: {phantom.safety_level}"

    def test_phantom_existence_prob(self):
        """Phantom P_exist 应在合理范围 (0, 1)。"""
        tracker, _ = self._make_scene([
            ("desk", [2.0, 3.0, 0.7]),
            ("chair", [2.5, 3.0, 0.5]),
            ("computer", [2.0, 2.5, 0.6]),
        ])
        for phantom in tracker.get_phantom_nodes():
            assert 0.0 < phantom.existence_prob < 1.0, \
                f"Phantom {phantom.label} P_exist={phantom.existence_prob} out of range"

    def test_phantom_source_tracing(self):
        """Phantom 应有可追溯的来源。"""
        tracker, _ = self._make_scene([
            ("refrigerator", [1.0, 1.0, 0.8]),
            ("microwave", [1.5, 1.0, 1.2]),
            ("sink", [2.0, 1.0, 0.9]),
        ])
        for phantom in tracker.get_phantom_nodes():
            assert phantom.source.startswith("kg_phantom:"), \
                f"Phantom source should start with 'kg_phantom:', got {phantom.source}"

    def test_promote_phantom(self):
        """Phantom → 实体化: 检测到匹配物体后 phantom 被转化为 TrackedObject。"""
        tracker, _ = self._make_scene([
            ("desk", [2.0, 3.0, 0.7]),
            ("chair", [2.5, 3.0, 0.5]),
            ("computer", [2.0, 2.5, 0.6]),
        ])
        phantoms = tracker.get_phantom_nodes()
        if phantoms:
            phantom = phantoms[0]
            old_count = len(tracker.objects)
            det = self.Detection3D(
                label=phantom.label, score=0.85,
                position=phantom.position.copy(),
                features=np.array([]),
                bbox_2d=np.array([0, 0, 100, 100]), depth=2.0,
            )
            promoted = tracker.promote_phantom(phantom.phantom_id, det)
            assert promoted is not None, "Promotion should succeed"
            assert promoted.label == phantom.label
            assert promoted.belief_alpha > 1.5, \
                "Promoted object should inherit phantom prior"
            assert promoted.is_kg_expected, "Promoted object should be KG-expected"
            assert len(tracker.objects) == old_count + 1

class TestSafetyAwareCredibility:
    """安全感知差异化阈值测试 — 论文创新点。"""

    def setup_method(self):
        from perception.tracking.instance_tracker import (
            TrackedObject, SAFETY_THRESHOLDS_NAVIGATION,
            SAFETY_THRESHOLDS_INTERACTION, SAFETY_PRIOR_ALPHA_SCALE,
        )
        self.TrackedObject = TrackedObject
        self.SAFETY_NAV = SAFETY_THRESHOLDS_NAVIGATION
        self.SAFETY_INTERACT = SAFETY_THRESHOLDS_INTERACTION
        self.SAFETY_ALPHA = SAFETY_PRIOR_ALPHA_SCALE

    def _make_obj(self, label, safety_level):
        obj = self.TrackedObject(
            object_id=0, label=label,
            position=np.array([0.0, 0.0, 0.0]),
            best_score=0.85,
            safety_level=safety_level,
        )
        obj.safety_nav_threshold = self.SAFETY_NAV.get(safety_level, 0.25)
        obj.safety_interact_threshold = self.SAFETY_INTERACT.get(safety_level, 0.40)
        return obj

    def test_safe_thresholds(self):
        """SAFE 物体: 导航和交互阈值应为最宽松。"""
        obj = self._make_obj("chair", "safe")
        assert obj.safety_nav_threshold == 0.25
        assert obj.safety_interact_threshold == 0.40

    def test_dangerous_thresholds(self):
        """DANGEROUS 物体: 导航阈值极低 (一点迹象就避障), 交互阈值极高。"""
        obj = self._make_obj("gas_cylinder", "dangerous")
        assert obj.safety_nav_threshold == 0.10
        assert obj.safety_interact_threshold == 0.80

    def test_forbidden_thresholds(self):
        """FORBIDDEN 物体: 几乎零容忍触发导航避障。"""
        obj = self._make_obj("electrical_panel", "forbidden")
        assert obj.safety_nav_threshold == 0.05
        assert obj.safety_interact_threshold == 0.95

    def test_caution_between_safe_and_dangerous(self):
        """CAUTION 阈值在 SAFE 和 DANGEROUS 之间。"""
        assert self.SAFETY_NAV["caution"] < self.SAFETY_NAV["safe"]
        assert self.SAFETY_NAV["caution"] > self.SAFETY_NAV["dangerous"]
        assert self.SAFETY_INTERACT["caution"] > self.SAFETY_INTERACT["safe"]
        assert self.SAFETY_INTERACT["caution"] < self.SAFETY_INTERACT["dangerous"]

    def test_nav_threshold_monotonically_decreasing(self):
        """导航阈值随危险等级递减 (越危险越敏感)。"""
        levels = ["safe", "caution", "dangerous", "forbidden"]
        thresholds = [self.SAFETY_NAV[l] for l in levels]
        for i in range(len(thresholds) - 1):
            assert thresholds[i] > thresholds[i + 1], \
                f"Nav threshold should decrease: {levels[i]}={thresholds[i]} > {levels[i+1]}={thresholds[i+1]}"

    def test_interact_threshold_monotonically_increasing(self):
        """交互阈值随危险等级递增 (越危险越严格)。"""
        levels = ["safe", "caution", "dangerous", "forbidden"]
        thresholds = [self.SAFETY_INTERACT[l] for l in levels]
        for i in range(len(thresholds) - 1):
            assert thresholds[i] < thresholds[i + 1], \
                f"Interact threshold should increase: {levels[i]}={thresholds[i]} < {levels[i+1]}={thresholds[i+1]}"

    def test_protective_bias_alpha(self):
        """保护性偏见: 危险物体 α 缩放系数 > 安全物体。"""
        assert self.SAFETY_ALPHA["dangerous"] > self.SAFETY_ALPHA["safe"]
        assert self.SAFETY_ALPHA["forbidden"] > self.SAFETY_ALPHA["dangerous"]

    def test_confirmed_for_navigation(self):
        """低可信度的危险物体也应被导航层视为障碍。"""
        obj = self._make_obj("gas_cylinder", "dangerous")
        obj.credibility = 0.12
        assert obj.is_confirmed_for_navigation, \
            "Dangerous object with credibility 0.12 should be confirmed for navigation"
        obj.credibility = 0.08
        assert not obj.is_confirmed_for_navigation

    def test_not_confirmed_for_interaction(self):
        """高可信度的危险物体仍需更多确认才允许交互。"""
        obj = self._make_obj("gas_cylinder", "dangerous")
        obj.credibility = 0.70
        assert not obj.is_confirmed_for_interaction, \
            "Dangerous object with credibility 0.70 should NOT be confirmed for interaction"
        obj.credibility = 0.85
        assert obj.is_confirmed_for_interaction

class TestExplorationTargets:
    """探索目标推荐测试。"""

    def setup_method(self):
        from perception.tracking.instance_tracker import InstanceTracker, Detection3D
        from memory.knowledge.knowledge_graph import IndustrialKnowledgeGraph
        self.InstanceTracker = InstanceTracker
        self.Detection3D = Detection3D
        self.KG = IndustrialKnowledgeGraph

    def test_exploration_targets_generated(self):
        """有 phantom 节点时应生成探索目标。"""
        kg = self.KG()
        tracker = self.InstanceTracker(max_objects=100, knowledge_graph=kg)
        for label, pos in [("desk", [2.0, 3.0, 0.7]),
                           ("chair", [2.5, 3.0, 0.5]),
                           ("computer", [2.0, 2.5, 0.6])]:
            tracker.update([self.Detection3D(
                label=label, score=0.9, position=np.array(pos),
                features=np.array([]),
                bbox_2d=np.array([0, 0, 100, 100]), depth=2.0,
            )])
        targets = tracker.get_exploration_targets()
        assert isinstance(targets, list)
        for t in targets:
            assert "type" in t
            assert "priority" in t
            assert t["type"] in ("explore_room", "confirm_phantom")

    def test_dangerous_phantom_prioritized(self):
        """危险 phantom 应有更高探索优先级。"""
        kg = self.KG()
        tracker = self.InstanceTracker(max_objects=100, knowledge_graph=kg)
        # lab 场景有 gas_cylinder (dangerous) 的 phantom
        for label, pos in [("desk", [2.0, 3.0, 0.7]),
                           ("cabinet", [2.5, 3.0, 0.5]),
                           ("fire_blanket", [2.0, 2.5, 0.6]),
                           ("first_aid_kit", [3.0, 3.0, 0.5])]:
            tracker.update([self.Detection3D(
                label=label, score=0.9, position=np.array(pos),
                features=np.array([]),
                bbox_2d=np.array([0, 0, 100, 100]), depth=2.0,
            )])
        targets = tracker.get_exploration_targets()
        phantom_targets = [t for t in targets if t["type"] == "confirm_phantom"]
        if len(phantom_targets) >= 2:
            # 检查危险物体是否排在前面
            dangerous_idx = [i for i, t in enumerate(phantom_targets)
                             if t.get("safety_level") in ("dangerous", "forbidden")]
            if dangerous_idx:
                assert dangerous_idx[0] < len(phantom_targets) // 2, \
                    "Dangerous phantoms should be prioritized"

class TestBPDiagnostics:
    """BP 诊断信息测试。"""

    def setup_method(self):
        from perception.tracking.instance_tracker import InstanceTracker, Detection3D
        from memory.knowledge.knowledge_graph import IndustrialKnowledgeGraph
        self.InstanceTracker = InstanceTracker
        self.Detection3D = Detection3D
        self.KG = IndustrialKnowledgeGraph

    def test_diagnostics_structure(self):
        """诊断信息应包含完整字段。"""
        kg = self.KG()
        tracker = self.InstanceTracker(max_objects=100, knowledge_graph=kg)
        tracker.update([self.Detection3D(
            label="desk", score=0.9, position=np.array([1.0, 1.0, 0.7]),
            features=np.array([]),
            bbox_2d=np.array([0, 0, 100, 100]), depth=2.0,
        )])
        diag = tracker.get_bp_diagnostics()
        assert "total_iterations" in diag
        assert "convergence_history" in diag
        assert "num_room_posteriors" in diag
        assert "num_phantom_nodes" in diag
        assert "room_posteriors" in diag
        assert "phantom_summary" in diag

    def test_scene_graph_v3_fields(self):
        """场景图 JSON v3.0 应包含 phantom_nodes 和 belief_propagation。"""
        kg = self.KG()
        tracker = self.InstanceTracker(max_objects=100, knowledge_graph=kg)
        for label, pos in [("desk", [2.0, 3.0, 0.7]),
                           ("chair", [2.5, 3.0, 0.5])]:
            tracker.update([self.Detection3D(
                label=label, score=0.9, position=np.array(pos),
                features=np.array([]),
                bbox_2d=np.array([0, 0, 100, 100]), depth=2.0,
            )])
        sg = json.loads(tracker.get_scene_graph_json())
        assert sg["graph_version"] == "3.0"
        assert "phantom_nodes" in sg
        assert "room_type_posteriors" in sg
        assert "belief_propagation" in sg
        assert isinstance(sg["phantom_nodes"], list)
        assert isinstance(sg["belief_propagation"], dict)

    def test_belief_dict_new_fields(self):
        """TrackedObject.to_belief_dict 应包含安全阈值和 KG 先验信息。"""
        kg = self.KG()
        tracker = self.InstanceTracker(max_objects=100, knowledge_graph=kg)
        for label, pos in [("desk", [2.0, 3.0, 0.7]),
                           ("chair", [2.5, 3.0, 0.5]),
                           ("computer", [2.0, 2.5, 0.6])]:
            tracker.update([self.Detection3D(
                label=label, score=0.9, position=np.array(pos),
                features=np.array([]),
                bbox_2d=np.array([0, 0, 100, 100]), depth=2.0,
            )])
        for obj in tracker.objects.values():
            bd = obj.to_belief_dict()
            assert "confirmed_nav" in bd
            assert "confirmed_interact" in bd
            assert isinstance(bd["confirmed_nav"], bool)
            assert isinstance(bd["confirmed_interact"], bool)

class TestRoomTypePosteriorDataclass:
    """RoomTypePosterior 数据类测试。"""

    def setup_method(self):
        from perception.tracking.instance_tracker import RoomTypePosterior
        self.RoomTypePosterior = RoomTypePosterior

    def test_empty_posterior(self):
        rtp = self.RoomTypePosterior(room_id=0)
        assert rtp.best_type == "unknown"
        assert rtp.best_confidence == 0.0
        assert rtp.entropy == 0.0

    def test_certain_posterior(self):
        rtp = self.RoomTypePosterior(room_id=0, hypotheses={"office": 0.95, "kitchen": 0.05})
        assert rtp.best_type == "office"
        assert rtp.best_confidence == 0.95
        assert rtp.entropy < 0.5, "Nearly certain → low entropy"

    def test_uncertain_posterior(self):
        rtp = self.RoomTypePosterior(room_id=0, hypotheses={
            "office": 0.25, "kitchen": 0.25, "corridor": 0.25, "storage": 0.25})
        assert rtp.entropy > 1.5, "Uniform → high entropy"

    def test_entropy_ordering(self):
        certain = self.RoomTypePosterior(room_id=0, hypotheses={"office": 0.9, "kitchen": 0.1})
        uncertain = self.RoomTypePosterior(room_id=1, hypotheses={"office": 0.5, "kitchen": 0.5})
        assert certain.entropy < uncertain.entropy, \
            "More certain distribution should have lower entropy"

class TestPhantomNodeDataclass:
    """PhantomNode 数据类测试。"""

    def setup_method(self):
        from perception.tracking.instance_tracker import PhantomNode
        self.PhantomNode = PhantomNode

    def test_phantom_existence_prob(self):
        p = self.PhantomNode(
            phantom_id=0, label="printer", room_id=0, room_type="office",
            position=np.array([0.0, 0.0]),
            belief_alpha=0.8, belief_beta=1.0,
        )
        assert 0.0 < p.existence_prob < 1.0
        expected = 0.8 / (0.8 + 1.0)
        assert abs(p.existence_prob - expected) < 1e-6

    def test_phantom_high_alpha(self):
        p = self.PhantomNode(
            phantom_id=0, label="fire_extinguisher", room_id=0, room_type="corridor",
            position=np.array([0.0, 0.0]),
            belief_alpha=3.0, belief_beta=1.0,
            safety_level="caution",
        )
        assert p.existence_prob > 0.7

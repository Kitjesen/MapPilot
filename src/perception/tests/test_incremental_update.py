"""测试 InstanceTracker 增量更新功能 (DovSG 局部更新)。

覆盖:
  - update_local(): 只处理 robot_pos 附近 update_radius 范围内的物体
  - remove_stale_objects(): 正确移除超时 + 低置信度物体
  - get_scene_graph_diff_json(): diff 输出格式正确
"""

import json
import os
import sys
import time

import numpy as np

# 把 src 路径加入 sys.path, 使 import 不依赖 ROS2 安装
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_SRC_DIR = os.path.join(_REPO_ROOT, "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from perception.tracking.instance_tracker import InstanceTracker
from perception.tracking.projection import Detection3D

# ─────────────────────────────────────────────
#  Helpers
# ─────────────────────────────────────────────

def _make_det(
    label: str,
    x: float,
    y: float,
    z: float = 0.0,
    score: float = 0.8,
    features: np.ndarray = None,
) -> Detection3D:
    """构造一个最小化 Detection3D (不需要 ROS2 环境)。"""
    det = Detection3D.__new__(Detection3D)
    det.label = label
    det.position = np.array([x, y, z], dtype=np.float64)
    det.score = score
    det.features = features if features is not None else np.array([])
    det.bbox_2d = np.array([0.0, 0.0, 0.0, 0.0])
    det.depth = float(np.linalg.norm(det.position))
    det.points = np.empty((0, 3))
    return det


def _make_tracker() -> InstanceTracker:
    return InstanceTracker(
        merge_distance=0.5,
        clip_threshold=0.75,
        max_objects=200,
        stale_timeout=300.0,
    )


# ─────────────────────────────────────────────
#  update_local() 测试
# ─────────────────────────────────────────────

class TestUpdateLocal:
    def test_returns_required_keys(self):
        """update_local 返回 dict 包含 added / updated / decayed 三个键。"""
        tracker = _make_tracker()
        robot_pos = np.array([0.0, 0.0, 0.0])
        dets = [_make_det("chair", 1.0, 0.0)]
        result = tracker.update_local(dets, robot_pos, update_radius=5.0)
        assert "added" in result
        assert "updated" in result
        assert "decayed" in result

    def test_new_object_within_radius_is_added(self):
        """半径内的新检测结果应被加入 _objects 并出现在 added 列表。"""
        tracker = _make_tracker()
        robot_pos = np.array([0.0, 0.0, 0.0])
        dets = [_make_det("chair", 1.0, 0.0)]
        result = tracker.update_local(dets, robot_pos, update_radius=5.0)
        assert len(result["added"]) == 1
        assert len(tracker.objects) == 1

    def test_objects_outside_radius_are_decayed_not_matched(self):
        """半径外的已有物体不做 CLIP 匹配, 但出现在 decayed 列表。"""
        tracker = _make_tracker()
        # 先通过 update() 在远处添加物体
        far_det = _make_det("table", 20.0, 0.0)
        tracker.update([far_det])
        assert len(tracker.objects) == 1

        # 在原点附近执行局部更新, 不提交任何检测
        robot_pos = np.array([0.0, 0.0, 0.0])
        result = tracker.update_local([], robot_pos, update_radius=5.0)

        # 远处物体应在 decayed, 不在 updated 或 added
        assert 20 > 5.0  # 确保它确实在半径外 (距离约 20m)
        assert len(result["decayed"]) == 1
        assert len(result["added"]) == 0
        assert len(result["updated"]) == 0
        # 物体仍然存在 (只是衰减, 不删除)
        assert len(tracker.objects) == 1

    def test_existing_object_within_radius_is_updated(self):
        """半径内已有物体被再次检测到时, 应出现在 updated 列表。"""
        tracker = _make_tracker()
        # 添加一个近处物体
        near_det = _make_det("door", 1.0, 0.5)
        tracker.update([near_det])
        obj_id = next(iter(tracker.objects.keys()))
        initial_count = tracker.objects[obj_id].detection_count

        # 再次在附近检测到
        robot_pos = np.array([0.0, 0.0, 0.0])
        result = tracker.update_local(
            [_make_det("door", 1.1, 0.5)],
            robot_pos,
            update_radius=5.0,
        )
        assert obj_id in result["updated"]
        assert tracker.objects[obj_id].detection_count > initial_count

    def test_objects_both_inside_and_outside_radius(self):
        """同时存在半径内外物体时, 分类正确。"""
        tracker = _make_tracker()
        # 一个近处 (2m), 一个远处 (15m)
        tracker.update([
            _make_det("chair", 2.0, 0.0),
            _make_det("table", 15.0, 0.0),
        ])
        assert len(tracker.objects) == 2

        robot_pos = np.array([0.0, 0.0, 0.0])
        result = tracker.update_local([], robot_pos, update_radius=5.0)

        # 近处物体: 在 update_radius 内, 未被检测到 (但不一定被 decayed)
        # 远处物体: 在 update_radius 外, 应在 decayed
        assert len(result["decayed"]) == 1
        assert len(result["added"]) == 0

    def test_multiple_new_detections_added(self):
        """多个新物体都在半径内时, 全部被 added。"""
        tracker = _make_tracker()
        robot_pos = np.array([0.0, 0.0, 0.0])
        dets = [
            _make_det("chair", 1.0, 0.0),
            _make_det("table", 2.0, 1.0),
            _make_det("door", 3.0, 0.0),
        ]
        result = tracker.update_local(dets, robot_pos, update_radius=10.0)
        assert len(result["added"]) == 3
        assert len(tracker.objects) == 3

    def test_zero_radius_all_objects_decayed(self):
        """update_radius=0 时, 所有现有物体都在范围外, 全部 decayed。"""
        tracker = _make_tracker()
        tracker.update([_make_det("chair", 5.0, 0.0)])
        robot_pos = np.array([0.0, 0.0, 0.0])
        result = tracker.update_local([], robot_pos, update_radius=0.0)
        assert len(result["decayed"]) == 1
        assert len(result["added"]) == 0

    def test_empty_tracker_no_crash(self):
        """空 tracker 调用 update_local 不崩溃。"""
        tracker = _make_tracker()
        robot_pos = np.array([0.0, 0.0, 0.0])
        result = tracker.update_local([], robot_pos, update_radius=5.0)
        assert result["added"] == []
        assert result["updated"] == []
        assert result["decayed"] == []


# ─────────────────────────────────────────────
#  remove_stale_objects() 测试
# ─────────────────────────────────────────────

class TestRemoveStaleObjects:
    def test_returns_list_of_strings(self):
        """返回类型应为 List[str]。"""
        tracker = _make_tracker()
        result = tracker.remove_stale_objects()
        assert isinstance(result, list)
        for item in result:
            assert isinstance(item, str)

    def test_empty_tracker_returns_empty(self):
        """空 tracker 调用不崩溃, 返回空列表。"""
        tracker = _make_tracker()
        assert tracker.remove_stale_objects() == []

    def test_fresh_high_confidence_object_not_removed(self):
        """新鲜且高置信度物体不被移除。"""
        tracker = _make_tracker()
        tracker.update([_make_det("chair", 1.0, 0.0)])
        removed = tracker.remove_stale_objects(stale_timeout_sec=30.0, min_confidence=0.1)
        assert len(removed) == 0
        assert len(tracker.objects) == 1

    def test_stale_low_confidence_object_is_removed(self):
        """过期 + 低置信度的物体应被移除。"""
        tracker = _make_tracker()
        tracker.update([_make_det("ghost_object", 1.0, 0.0)])
        obj_id = next(iter(tracker.objects.keys()))
        obj = tracker.objects[obj_id]

        # 手动设置成"很久以前见过"且"低置信度"
        obj.last_seen = time.time() - 100.0  # 100 秒前
        obj.credibility = 0.05               # 低于阈值

        removed = tracker.remove_stale_objects(stale_timeout_sec=30.0, min_confidence=0.1)
        assert str(obj_id) in removed
        assert obj_id not in tracker.objects

    def test_stale_but_high_confidence_not_removed(self):
        """过期但置信度高的物体不被移除 (双条件: 超时 AND 低置信度)。"""
        tracker = _make_tracker()
        tracker.update([_make_det("important_sign", 1.0, 0.0)])
        obj_id = next(iter(tracker.objects.keys()))
        obj = tracker.objects[obj_id]

        obj.last_seen = time.time() - 100.0  # 过期
        obj.credibility = 0.8                 # 但置信度高

        removed = tracker.remove_stale_objects(stale_timeout_sec=30.0, min_confidence=0.5)
        assert str(obj_id) not in removed
        assert obj_id in tracker.objects

    def test_fresh_but_low_confidence_not_removed(self):
        """新鲜但置信度低的物体不被移除 (需同时满足两个条件)。"""
        tracker = _make_tracker()
        tracker.update([_make_det("uncertain_obj", 1.0, 0.0)])
        obj_id = next(iter(tracker.objects.keys()))
        obj = tracker.objects[obj_id]

        # 时间戳是当前 → 刚见过, 未超时
        obj.last_seen = time.time()
        obj.credibility = 0.02  # 很低

        removed = tracker.remove_stale_objects(stale_timeout_sec=30.0, min_confidence=0.1)
        assert str(obj_id) not in removed
        assert obj_id in tracker.objects

    def test_multiple_stale_objects_all_removed(self):
        """多个过期 + 低置信度物体全部被移除。"""
        tracker = _make_tracker()
        dets = [
            _make_det("obj_a", 1.0, 0.0),
            _make_det("obj_b", 2.0, 0.0),
            _make_det("obj_c", 3.0, 0.0),
        ]
        tracker.update(dets)
        # 全部设成过期低置信度
        for obj in tracker.objects.values():
            obj.last_seen = time.time() - 200.0
            obj.credibility = 0.01

        removed = tracker.remove_stale_objects(stale_timeout_sec=30.0, min_confidence=0.1)
        assert len(removed) == 3
        assert len(tracker.objects) == 0

    def test_mixed_stale_and_fresh_only_stale_removed(self):
        """混合场景: 只有过期+低置信度的物体被移除, 其余保留。"""
        tracker = _make_tracker()
        dets = [
            _make_det("stale_obj", 1.0, 0.0),
            _make_det("fresh_obj", 2.0, 0.0),
        ]
        tracker.update(dets)
        ids = list(tracker.objects.keys())

        # obj_ids[0] → 过期低置信
        tracker.objects[ids[0]].last_seen = time.time() - 200.0
        tracker.objects[ids[0]].credibility = 0.01
        # obj_ids[1] → 新鲜
        tracker.objects[ids[1]].last_seen = time.time()
        tracker.objects[ids[1]].credibility = 0.9

        removed = tracker.remove_stale_objects(stale_timeout_sec=30.0, min_confidence=0.1)
        assert len(removed) == 1
        assert str(ids[0]) in removed
        assert len(tracker.objects) == 1


# ─────────────────────────────────────────────
#  get_scene_graph_diff_json() 测试
# ─────────────────────────────────────────────

class TestGetSceneGraphDiffJson:
    def _empty_snapshot(self) -> dict:
        return {"objects": [], "relations": [], "regions": []}

    def test_returns_valid_json_string(self):
        """返回值应为合法 JSON 字符串。"""
        tracker = _make_tracker()
        diff_str = tracker.get_scene_graph_diff_json(self._empty_snapshot())
        parsed = json.loads(diff_str)
        assert isinstance(parsed, dict)

    def test_required_keys_present(self):
        """JSON 必须包含 added / updated / removed / timestamp / summary。"""
        tracker = _make_tracker()
        diff_str = tracker.get_scene_graph_diff_json(self._empty_snapshot())
        parsed = json.loads(diff_str)
        for key in ("added", "updated", "removed", "timestamp", "summary"):
            assert key in parsed, f"Missing key: {key}"

    def test_added_objects_detected(self):
        """与空快照比较, 所有当前物体应在 added 列表中。"""
        tracker = _make_tracker()
        tracker.update([
            _make_det("chair", 1.0, 0.0),
            _make_det("table", 2.0, 0.0),
        ])
        diff_str = tracker.get_scene_graph_diff_json(self._empty_snapshot())
        parsed = json.loads(diff_str)
        assert len(parsed["added"]) == 2

    def test_removed_objects_detected(self):
        """物体消失时, 应出现在 removed 列表。"""
        tracker = _make_tracker()
        # 快照中有两个物体
        prev_snapshot = {
            "objects": [
                {"id": 99, "label": "ghost", "position": {"x": 1.0, "y": 0.0, "z": 0.0},
                 "belief": {"credibility": 0.5}},
            ]
        }
        # 当前 tracker 为空
        diff_str = tracker.get_scene_graph_diff_json(prev_snapshot)
        parsed = json.loads(diff_str)
        assert len(parsed["removed"]) == 1
        assert parsed["removed"][0]["id"] == 99

    def test_no_changes_empty_lists(self):
        """当前状态与快照一致时, added/updated/removed 均为空。"""
        tracker = _make_tracker()
        tracker.update([_make_det("chair", 1.0, 0.0, score=0.8)])
        # 从 get_scene_graph_json() 取快照 (含 objects 列表)
        sg_str = tracker.get_scene_graph_json()
        snapshot = json.loads(sg_str)

        diff_str = tracker.get_scene_graph_diff_json(snapshot)
        parsed = json.loads(diff_str)

        # 位置和置信度均未变化, 应无 added / removed
        assert len(parsed["added"]) == 0
        assert len(parsed["removed"]) == 0

    def test_updated_objects_detected_on_position_change(self):
        """物体位置显著移动时, 应出现在 updated 列表。"""
        tracker = _make_tracker()
        tracker.update([_make_det("chair", 1.0, 0.0)])
        obj_id = next(iter(tracker.objects.keys()))

        # 构造快照: 物体在旧位置
        prev_snapshot = {
            "objects": [
                {
                    "id": obj_id,
                    "label": "chair",
                    "position": {"x": 1.0, "y": 0.0, "z": 0.0},
                    "belief": {"credibility": 0.5},
                }
            ]
        }

        # 将物体移动到新位置
        tracker.objects[obj_id].position = np.array([5.0, 0.0, 0.0])

        diff_str = tracker.get_scene_graph_diff_json(prev_snapshot)
        parsed = json.loads(diff_str)
        updated_ids = [e["id"] for e in parsed["updated"]]
        assert obj_id in updated_ids

    def test_added_entry_has_required_fields(self):
        """added 列表中每个条目应包含 id / label / position / credibility。"""
        tracker = _make_tracker()
        tracker.update([_make_det("monitor", 1.0, 1.0)])
        diff_str = tracker.get_scene_graph_diff_json(self._empty_snapshot())
        parsed = json.loads(diff_str)
        assert len(parsed["added"]) == 1
        entry = parsed["added"][0]
        for field in ("id", "label", "position", "credibility"):
            assert field in entry, f"Missing field '{field}' in added entry"
        assert isinstance(entry["position"], dict)
        for axis in ("x", "y", "z"):
            assert axis in entry["position"]

    def test_timestamp_is_recent(self):
        """timestamp 字段应接近当前时间 (±5 秒)。"""
        tracker = _make_tracker()
        diff_str = tracker.get_scene_graph_diff_json(self._empty_snapshot())
        parsed = json.loads(diff_str)
        assert abs(parsed["timestamp"] - time.time()) < 5.0

    def test_summary_is_string(self):
        """summary 字段应为字符串。"""
        tracker = _make_tracker()
        diff_str = tracker.get_scene_graph_diff_json(self._empty_snapshot())
        parsed = json.loads(diff_str)
        assert isinstance(parsed["summary"], str)

    def test_summary_contains_added_info_when_objects_added(self):
        """有 added 物体时, summary 应包含 'added' 关键词。"""
        tracker = _make_tracker()
        tracker.update([_make_det("chair", 1.0, 0.0)])
        diff_str = tracker.get_scene_graph_diff_json(self._empty_snapshot())
        parsed = json.loads(diff_str)
        assert "added" in parsed["summary"].lower()


# ─────────────────────────────────────────────
#  集成测试: 组合使用
# ─────────────────────────────────────────────

class TestIncrementalUpdateIntegration:
    def test_update_local_then_diff(self):
        """update_local 之后通过 diff 能观察到新增物体。"""
        tracker = _make_tracker()
        prev_snapshot: dict = {"objects": []}

        robot_pos = np.array([0.0, 0.0, 0.0])
        dets = [_make_det("chair", 1.0, 0.0), _make_det("table", 2.0, 0.0)]
        tracker.update_local(dets, robot_pos, update_radius=5.0)

        diff_str = tracker.get_scene_graph_diff_json(prev_snapshot)
        parsed = json.loads(diff_str)
        assert len(parsed["added"]) == 2

    def test_remove_stale_then_diff_shows_removed(self):
        """remove_stale_objects 移除物体后, diff 能检测到 removed。"""
        tracker = _make_tracker()
        tracker.update([_make_det("old_chair", 1.0, 0.0)])

        # 取初始快照
        sg_str = tracker.get_scene_graph_json()
        snapshot = json.loads(sg_str)

        # 强制物体过期 + 低置信度
        for obj in tracker.objects.values():
            obj.last_seen = time.time() - 200.0
            obj.credibility = 0.01

        removed = tracker.remove_stale_objects(stale_timeout_sec=30.0, min_confidence=0.1)
        assert len(removed) == 1

        diff_str = tracker.get_scene_graph_diff_json(snapshot)
        parsed = json.loads(diff_str)
        assert len(parsed["removed"]) == 1

    def test_update_local_does_not_process_remote_with_clip(self):
        """局部更新时, 远处物体不被 CLIP 匹配处理 (detection_count 不变)。"""
        tracker = _make_tracker()
        far_feat = np.random.randn(512)
        far_feat = far_feat / np.linalg.norm(far_feat)
        far_det = _make_det("chair", 50.0, 0.0, features=far_feat)
        tracker.update([far_det])
        remote_obj = next(iter(tracker.objects.values()))
        initial_count = remote_obj.detection_count

        # 在原点附近发一个相同 label 的检测 (理论上可以匹配), 但用局部更新
        near_det = _make_det("chair", 1.0, 0.0, features=far_feat)
        robot_pos = np.array([0.0, 0.0, 0.0])
        result = tracker.update_local([near_det], robot_pos, update_radius=5.0)

        # 远处物体不应被更新 (detection_count 不变)
        assert remote_obj.detection_count == initial_count
        # 近处的 "chair" 应被当作新物体 added
        assert len(result["added"]) == 1

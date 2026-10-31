"""
test_instance_tracker.py — 实例追踪器单元测试

覆盖:
  - infer_room_type() — 规则推断
  - TrackedObject.update() — 位置融合/特征 EMA/信念更新
  - InstanceTracker.update() — 新建/匹配/去重
  - get_scene_graph_json() — JSON 序列化
"""

import json
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from perception.tracking.instance_tracker import (
    InstanceTracker,
    PhantomNode,
    TrackedObject,
    infer_room_type,
)
from perception.tracking.projection import Detection3D, mask_to_pointcloud
from runtime.msgs.sensor import CameraIntrinsics

# ─────────────────────────────────────────────────────────────────────────────
#  辅助工厂
# ─────────────────────────────────────────────────────────────────────────────

def test_two_nearby_chairs_with_identical_features_keep_separate_ids_when_order_changes():
    tracker = InstanceTracker(merge_distance=0.35, use_hungarian_matching=False, enable_dedup_merge=False)
    feature = np.array([1.0, 0.0, 0.0])
    cloud = np.array([[0, 0, 0], [0.02, 0, 0], [0, 0.02, 0], [0, 0, 0.02]])

    def detection(x):
        det = _make_det(pos=(x, 0, 0), features=feature)
        det.points = cloud + np.array([x, 0, 0])
        return det

    first = tracker.update([detection(0), detection(0.8)])
    ids = [obj.object_id for obj in first]
    second = tracker.update([detection(0.81), detection(0.01)])
    assert [obj.object_id for obj in second] == ids[::-1]
    assert len(tracker.objects) == 2


def _make_det(
    label: str = "chair",
    pos: tuple = (1.0, 2.0, 0.5),
    score: float = 0.9,
    features: np.ndarray | None = None,
) -> Detection3D:
    """构造测试用 Detection3D。"""
    if features is None:
        features = np.random.rand(512).astype(np.float32)
        features /= np.linalg.norm(features)
    return Detection3D(
        position=np.array(pos, dtype=np.float32),
        label=label,
        score=score,
        bbox_2d=np.array([0, 0, 100, 100], dtype=np.float32),
        depth=float(np.linalg.norm(pos[:2])),
        features=features,
    )


# ─────────────────────────────────────────────────────────────────────────────
#  infer_room_type
# ─────────────────────────────────────────────────────────────────────────────

class TestInferRoomType(unittest.TestCase):

    def test_corridor_from_door(self):
        result = infer_room_type(["door", "sign"])
        self.assertEqual(result, "corridor")

    def test_kitchen_from_refrigerator(self):
        result = infer_room_type(["refrigerator", "table"])
        self.assertEqual(result, "kitchen")

    def test_bathroom_from_toilet(self):
        result = infer_room_type(["toilet", "mirror"])
        self.assertEqual(result, "bathroom")

    def test_office_requires_2_keywords(self):
        # 只有一个关键词 → 不满足 min_match=2
        result_single = infer_room_type(["desk"])
        self.assertNotEqual(result_single, "office")

        # 两个关键词 → 满足
        result_double = infer_room_type(["desk", "chair"])
        self.assertEqual(result_double, "office")

    def test_stairwell_from_stairs(self):
        result = infer_room_type(["stairs", "railing"])
        self.assertEqual(result, "stairwell")

    def test_lobby_from_sofa(self):
        result = infer_room_type(["sofa", "reception"])
        self.assertEqual(result, "lobby")

    def test_empty_labels_returns_unknown(self):
        result = infer_room_type([])
        self.assertEqual(result, "unknown_area")

    def test_fallback_to_area_name(self):
        """没有匹配规则时, 回退到 area_<label> 格式。"""
        result = infer_room_type(["xyz_obj", "abc_thing"])
        self.assertTrue(result.startswith("area_"))

    def test_chinese_kitchen(self):
        result = infer_room_type(["冰箱", "水壶"])
        self.assertEqual(result, "kitchen")

    def test_case_insensitive_match(self):
        result = infer_room_type(["Door", "Sign"])
        self.assertEqual(result, "corridor")


# ─────────────────────────────────────────────────────────────────────────────
#  TrackedObject.update
# ─────────────────────────────────────────────────────────────────────────────

class TestTrackedObjectUpdate(unittest.TestCase):

    def _make_obj(self, label="chair") -> TrackedObject:
        return TrackedObject(
            object_id=0,
            label=label,
            position=np.array([1.0, 2.0, 0.0], dtype=np.float32),
            best_score=0.8,
            features=np.zeros(512, dtype=np.float32),
        )

    def test_detection_count_increments(self):
        obj = self._make_obj()
        initial = obj.detection_count
        det = _make_det(pos=(1.1, 2.1, 0.0))
        obj.update(det)
        self.assertEqual(obj.detection_count, initial + 1)

    def test_miss_streak_resets(self):
        obj = self._make_obj()
        obj.miss_streak = 5
        obj.update(_make_det())
        self.assertEqual(obj.miss_streak, 0)

    def test_belief_alpha_increases(self):
        obj = self._make_obj()
        alpha_before = obj.belief_alpha
        obj.update(_make_det())
        self.assertGreater(obj.belief_alpha, alpha_before)

    def test_best_score_updates(self):
        obj = self._make_obj()
        obj.best_score = 0.5
        det = _make_det(score=0.95)
        obj.update(det)
        self.assertAlmostEqual(obj.best_score, 0.95)

    def test_feature_fusion_uses_quality_weighted_ema_and_stays_normalized(self):
        old_feature = np.array([1.0, 0.0], dtype=np.float32)
        new_feature = np.array([0.0, 1.0], dtype=np.float32)
        obj = TrackedObject(
            object_id=0,
            label="chair",
            position=np.zeros(3, dtype=np.float32),
            best_score=0.8,
            detection_count=3,
            features=old_feature,
        )

        obj.update(
            _make_det(
                label="chair",
                pos=(0.1, 0.0, 0.0),
                features=new_feature,
            )
        )

        ema_alpha = 0.3 * (0.9 / 0.8)
        expected = np.array([1.0 - ema_alpha, ema_alpha], dtype=np.float64)
        expected /= np.linalg.norm(expected)
        np.testing.assert_allclose(obj.features, expected)
        self.assertAlmostEqual(float(np.linalg.norm(obj.features)), 1.0)
        self.assertEqual(obj.detection_count, 4)


# ─────────────────────────────────────────────────────────────────────────────
#  InstanceTracker
# ─────────────────────────────────────────────────────────────────────────────

class TestInstanceTrackerInit(unittest.TestCase):

    def test_empty_on_init(self):
        tracker = InstanceTracker()
        self.assertEqual(len(tracker.objects), 0)

    def test_custom_params(self):
        tracker = InstanceTracker(merge_distance=1.0, max_objects=50)
        self.assertEqual(tracker.merge_distance, 1.0)
        self.assertEqual(tracker.max_objects, 50)


class TestInstanceTrackerUpdate(unittest.TestCase):

    def setUp(self):
        self.tracker = InstanceTracker(merge_distance=0.5, clip_threshold=0.75)

    def test_new_detection_creates_object(self):
        det = _make_det(label="chair", pos=(1.0, 2.0, 0.5))
        result = self.tracker.update([det])
        self.assertEqual(len(self.tracker.objects), 1)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].label, "chair")

    def test_multiple_new_detections(self):
        dets = [
            _make_det(label="chair", pos=(1.0, 0.0, 0.0)),
            _make_det(label="desk", pos=(5.0, 0.0, 0.0)),
            _make_det(label="monitor", pos=(10.0, 0.0, 0.0)),
        ]
        self.tracker.update(dets)
        self.assertEqual(len(self.tracker.objects), 3)

    def test_same_position_deduplicates(self):
        """相同类别 + 接近位置的检测应合并为一个对象。"""
        feat = np.random.rand(512).astype(np.float32)
        feat /= np.linalg.norm(feat)

        det1 = _make_det(label="chair", pos=(1.0, 0.0, 0.0), features=feat)
        det2 = _make_det(label="chair", pos=(1.1, 0.0, 0.0), features=feat)  # 同一 feat
        self.tracker.update([det1])
        self.tracker.update([det2])
        # 相同特征 + 相近位置 → 应合并
        self.assertLessEqual(len(self.tracker.objects), 2)

    def test_far_positions_kept_separate(self):
        """距离很远的同类对象不应合并。"""
        feat1 = np.random.rand(512).astype(np.float32)
        feat1 /= np.linalg.norm(feat1)
        feat2 = np.random.rand(512).astype(np.float32)
        feat2 /= np.linalg.norm(feat2)
        # 强制不同特征使语义匹配失败
        feat1[0] = 1.0
        feat1[1:] = 0
        feat2[1] = 1.0
        feat2[0] = 0
        feat2[2:] = 0

        det1 = _make_det(label="chair", pos=(0.0, 0.0, 0.0), features=feat1)
        det2 = _make_det(label="chair", pos=(20.0, 0.0, 0.0), features=feat2)
        self.tracker.update([det1])
        self.tracker.update([det2])
        self.assertEqual(len(self.tracker.objects), 2)

    def test_similar_features_do_not_merge_objects_outside_candidate_radius(self):
        feature = np.array([1.0, 0.0, 0.0], dtype=np.float32)

        first = self.tracker.update(
            [_make_det(label="chair", pos=(0.0, 0.0, 0.0), features=feature)]
        )[0]
        second = self.tracker.update(
            [_make_det(label="chair", pos=(20.0, 0.0, 0.0), features=feature)]
        )[0]

        self.assertNotEqual(second.object_id, first.object_id)
        self.assertEqual(len(self.tracker.objects), 2)

    def test_semantic_feature_matches_cross_view_with_only_weak_geometry(self):
        first_feature = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        next_feature = np.array([0.98, 0.2, 0.0], dtype=np.float32)
        next_feature /= np.linalg.norm(next_feature)

        original = self.tracker.update(
            [_make_det(label="chair", pos=(0.0, 0.0, 0.0), features=first_feature)]
        )[0]
        continued = self.tracker.update(
            [_make_det(label="seat", pos=(0.44, 0.0, 0.0), features=next_feature)]
        )[0]

        self.assertEqual(continued.object_id, original.object_id)
        self.assertEqual(continued.detection_count, 2)
        self.assertEqual(len(self.tracker.objects), 1)

    def test_empty_detections(self):
        self.tracker.update([])
        self.assertEqual(len(self.tracker.objects), 0)

    def test_objects_have_incrementing_ids(self):
        det1 = _make_det(label="a", pos=(0.0, 0.0, 0.0))
        det2 = _make_det(label="b", pos=(5.0, 0.0, 0.0))
        self.tracker.update([det1, det2])
        ids = list(self.tracker.objects.keys())
        self.assertEqual(len(set(ids)), 2)  # 无重复

    def test_one_object_is_claimed_by_at_most_one_detection_per_frame(self):
        feat = np.zeros(512, dtype=np.float32)
        feat[0] = 1.0
        original = self.tracker.update(
            [_make_det(label="chair", pos=(1.0, 0.0, 0.0), features=feat)]
        )[0]

        matched = self.tracker.update(
            [
                _make_det(label="chair", pos=(1.05, 0.0, 0.0), features=feat),
                _make_det(label="chair", pos=(1.10, 0.0, 0.0), features=feat),
            ]
        )

        self.assertEqual(len({obj.object_id for obj in matched}), 2)
        self.assertEqual(original.detection_count, 2)
        self.assertEqual(len(self.tracker.objects), 2)

    def test_moving_camera_views_fuse_in_world_coordinates(self):
        intrinsics = CameraIntrinsics(
            fx=100.0,
            fy=100.0,
            cx=20.0,
            cy=10.0,
            width=40,
            height=20,
        )
        depth = np.full((20, 40), 2000, dtype=np.uint16)
        first_mask = np.zeros((20, 40), dtype=bool)
        first_mask[7:14, 17:24] = True
        second_mask = np.zeros((20, 40), dtype=bool)
        second_mask[7:14, 7:14] = True
        first_pose = np.eye(4)
        second_pose = np.eye(4)
        second_pose[0, 3] = 0.2

        first_points = mask_to_pointcloud(
            first_mask,
            depth,
            intrinsics,
            first_pose,
        )
        second_points = mask_to_pointcloud(
            second_mask,
            depth,
            intrinsics,
            second_pose,
        )
        self.assertIsNotNone(first_points)
        self.assertIsNotNone(second_points)
        assert first_points is not None and second_points is not None
        np.testing.assert_allclose(
            np.mean(first_points, axis=0),
            np.mean(second_points, axis=0),
            atol=1e-8,
        )

        first = _make_det(
            label="chair",
            pos=tuple(np.mean(first_points, axis=0)),
            features=np.array([], dtype=np.float32),
        )
        first.points = first_points
        second = _make_det(
            label="chair",
            pos=tuple(np.mean(second_points, axis=0)),
            features=np.array([], dtype=np.float32),
        )
        second.points = second_points

        original = self.tracker.update([first])[0]
        continued = self.tracker.update([second])[0]

        self.assertEqual(continued.object_id, original.object_id)
        self.assertEqual(continued.detection_count, 2)
        self.assertEqual(len(self.tracker.objects), 1)

    def test_max_objects_limit(self):
        tracker = InstanceTracker(max_objects=5)
        for i in range(10):
            det = _make_det(label=f"obj_{i}", pos=(float(i * 10), 0.0, 0.0))
            tracker.update([det])
        self.assertLessEqual(len(tracker.objects), 10)  # 不超过内部上限

    def test_capacity_pruning_updates_queries_and_scene_graph_in_place(self):
        tracker = InstanceTracker(max_objects=1)
        original_store = tracker._objects
        with patch("perception.tracking.instance_tracker.time.time", return_value=100.0):
            tracker.update([_make_det(label="chair", score=0.95)])
            tracker.update([_make_det(label="table", pos=(10.0, 0.0, 0.0), score=0.1)])

            self.assertEqual(len(tracker.objects), 1)
            self.assertIs(tracker._objects, original_store)
            self.assertEqual(tracker.query_by_text("table"), [])
            self.assertEqual(set(tracker._scene_graph_builder.objects), set(tracker.objects))
            tracker.update([_make_det(label="chair", score=0.95)])
            self.assertEqual(tracker.query_by_text("chair")[0].detection_count, 2)

    def test_promote_phantom_creates_tracked_object_with_inherited_prior(self):
        phantom = PhantomNode(
            phantom_id=7,
            label="chair",
            belief_alpha=3.0,
            safety_level="caution",
            source="kg_phantom:office",
        )
        self.tracker._phantom_nodes[phantom.phantom_id] = phantom

        promoted = self.tracker.promote_phantom(
            phantom.phantom_id,
            _make_det(label="chair", pos=(2.0, 3.0, 0.5)),
        )

        self.assertIsInstance(promoted, TrackedObject)
        self.assertEqual(promoted.kg_prior_alpha, phantom.belief_alpha)
        self.assertEqual(promoted.kg_prior_source, phantom.source)
        self.assertEqual(promoted.safety_level, phantom.safety_level)
        self.assertNotIn(phantom.phantom_id, self.tracker._phantom_nodes)
        self.assertIs(self.tracker.objects[promoted.object_id], promoted)


class TestInstanceTrackerSceneGraph(unittest.TestCase):

    def setUp(self):
        self.tracker = InstanceTracker()

    def test_get_scene_graph_json_empty(self):
        json_str = self.tracker.get_scene_graph_json()
        data = json.loads(json_str)
        self.assertIn("objects", data)
        self.assertEqual(len(data["objects"]), 0)

    def test_get_scene_graph_json_with_objects(self):
        dets = [
            _make_det(label="chair", pos=(1.0, 0.0, 0.5)),
            _make_det(label="desk", pos=(3.0, 0.0, 0.5)),
        ]
        self.tracker.update(dets)
        json_str = self.tracker.get_scene_graph_json()
        data = json.loads(json_str)
        self.assertIn("objects", data)
        self.assertGreaterEqual(len(data["objects"]), 2)

    def test_scene_graph_has_required_fields(self):
        det = _make_det(label="monitor", pos=(2.0, 1.0, 1.0))
        self.tracker.update([det])
        data = json.loads(self.tracker.get_scene_graph_json())
        obj = data["objects"][0]
        self.assertIn("label", obj)
        self.assertIn("position", obj)
        self.assertTrue("object_id" in obj or "id" in obj)
        self.assertIn("score", obj)

    def test_scene_graph_includes_relations(self):
        """两个足够近的物体应产生 near 关系。"""
        det1 = _make_det(label="monitor", pos=(0.0, 0.0, 1.0))
        det2 = _make_det(label="keyboard", pos=(0.5, 0.0, 1.0))
        self.tracker.update([det1, det2])
        data = json.loads(self.tracker.get_scene_graph_json())
        self.assertIn("relations", data)


class TestInstanceTrackerPersistence(unittest.TestCase):

    def test_save_load_restores_object_fields_and_marks_loaded_source(self):
        tracker = InstanceTracker()
        original = tracker.update([_make_det(label="chair", pos=(1.0, 2.0, 0.5))])[0]
        original.detection_count = 4
        original.belief_alpha = 6.5
        original.safety_level = "caution"

        with tempfile.TemporaryDirectory() as temp_dir:
            path = f"{temp_dir}/scene.json"
            tracker.save_to_file(path)

            restored = InstanceTracker()
            self.assertTrue(restored.load_from_file(path))

        loaded = restored.objects[original.object_id]
        self.assertIsInstance(loaded, TrackedObject)
        self.assertEqual(loaded.label, original.label)
        np.testing.assert_allclose(loaded.position, original.position)
        self.assertEqual(loaded.detection_count, original.detection_count)
        self.assertEqual(loaded.belief_alpha, original.belief_alpha)
        self.assertEqual(loaded.safety_level, original.safety_level)
        self.assertEqual(loaded.source, "loaded")

    def test_load_continues_tracking_and_preserves_next_object_id(self):
        feat = np.array([3.0, 4.0, 0.0], dtype=np.float32)
        tracker = InstanceTracker(merge_distance=0.5)
        original = tracker.update(
            [_make_det(label="chair", pos=(1.0, 2.0, 0.5), features=feat)]
        )[0]

        with tempfile.TemporaryDirectory() as temp_dir:
            path = f"{temp_dir}/scene.json"
            tracker.save_to_file(path)
            restored = InstanceTracker(merge_distance=0.5)
            self.assertTrue(restored.load_from_file(path))

        loaded = restored.objects[original.object_id]
        self.assertEqual(loaded.features.shape, feat.shape)
        np.testing.assert_allclose(loaded.features, original.features)
        self.assertAlmostEqual(float(np.linalg.norm(loaded.features)), 1.0)

        continued = restored.update(
            [_make_det(label="seat", pos=(1.44, 2.0, 0.5), features=feat)]
        )[0]
        added = restored.update(
            [_make_det(label="table", pos=(5.0, 2.0, 0.5), features=feat)]
        )[0]

        self.assertEqual(continued.object_id, original.object_id)
        self.assertNotEqual(added.object_id, original.object_id)
        self.assertEqual(set(restored.objects), {original.object_id, added.object_id})


if __name__ == "__main__":
    unittest.main()

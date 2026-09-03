"""
test_laplacian_filter.py — Laplacian 模糊检测单元测试
"""

import unittest

import numpy as np
import pytest

from runtime.msgs.sensor import CameraIntrinsics


@pytest.fixture(scope="module")
def laplacian_filter():
    pytest.importorskip("cv2", reason="OpenCV 仅在机器人环境可用")
    from perception.detection import laplacian_filter

    return laplacian_filter


class TestLaplacianFilter:
    """Laplacian 方差模糊检测测试。"""

    def test_sharp_image_not_blurry(self, laplacian_filter):
        """清晰图像 (高方差) 不应被判定为模糊。"""
        # 创建棋盘格图案 (高频内容 → 高 Laplacian 方差)
        img = np.zeros((100, 100), dtype=np.uint8)
        img[::2, ::2] = 255
        img[1::2, 1::2] = 255
        variance = laplacian_filter.compute_laplacian_variance(img)
        assert variance > 100.0
        assert not laplacian_filter.is_blurry(img, threshold=100.0)

    def test_blurry_image_is_blurry(self, laplacian_filter):
        """模糊图像 (低方差) 应被判定为模糊。"""
        # 纯灰色图像 → Laplacian 全零
        img = np.ones((100, 100), dtype=np.uint8) * 128
        variance = laplacian_filter.compute_laplacian_variance(img)
        assert variance == pytest.approx(0.0, abs=1e-5)
        assert laplacian_filter.is_blurry(img, threshold=100.0)

    def test_bgr_image_conversion(self, laplacian_filter):
        """BGR 3通道图像应自动转灰度。"""
        bgr = np.zeros((100, 100, 3), dtype=np.uint8)
        bgr[:, :, 1] = 128  # 绿色通道
        result = laplacian_filter.is_blurry(bgr, threshold=100.0)
        # 纯色 → 模糊
        assert result

    def test_gradient_image(self, laplacian_filter):
        """渐变图像应有中等方差。"""
        img = np.tile(np.arange(256, dtype=np.uint8), (100, 1))[:, :100]
        variance = laplacian_filter.compute_laplacian_variance(img)
        assert variance > 0

    def test_custom_threshold(self, laplacian_filter):
        """自定义阈值应生效。"""
        img = np.zeros((100, 100), dtype=np.uint8)
        img[::2, ::2] = 255
        img[1::2, 1::2] = 255
        # 使用极高阈值 → 应判为模糊
        assert laplacian_filter.is_blurry(img, threshold=1e10)
        # 使用极低阈值 → 不应判为模糊
        assert not laplacian_filter.is_blurry(img, threshold=0.0)


class TestProjection(unittest.TestCase):
    """3D 投影基础测试。"""

    def test_project_center_pixel(self):
        """中心像素投影到光心应为 (0, 0, depth)。"""
        from perception.tracking.projection import project_to_3d
        intrinsics = CameraIntrinsics(
            fx=500, fy=500, cx=320, cy=240, width=640, height=480
        )
        p = project_to_3d(320, 240, 2.0, intrinsics)
        np.testing.assert_allclose(p, [0.0, 0.0, 2.0], atol=1e-6)

    def test_project_zero_depth(self):
        """Zero depth remains on the camera plane."""
        from perception.tracking.projection import project_to_3d

        intrinsics = CameraIntrinsics(
            fx=500, fy=500, cx=320, cy=240, width=640, height=480
        )
        p = project_to_3d(150, 150, 0.0, intrinsics)
        self.assertEqual(p[2], 0.0)

    def test_transform_identity(self):
        """单位变换应保持坐标不变。"""
        from perception.tracking.projection import transform_point
        point = np.array([1.0, 2.0, 3.0])
        tf = np.eye(4)
        result = transform_point(point, tf)
        np.testing.assert_allclose(result, point, atol=1e-6)

    def test_transform_translation(self):
        """纯平移变换应正确偏移。"""
        from perception.tracking.projection import transform_point
        point = np.array([1.0, 2.0, 3.0])
        tf = np.eye(4)
        tf[:3, 3] = [10.0, 20.0, 30.0]
        result = transform_point(point, tf)
        np.testing.assert_allclose(result, [11.0, 22.0, 33.0], atol=1e-6)


class TestInstanceTracker(unittest.TestCase):
    """实例追踪器测试。"""

    def test_new_detection_creates_instance(self):
        """新检测应创建新实例。"""
        from perception.tracking.instance_tracker import InstanceTracker
        from perception.tracking.projection import Detection3D
        tracker = InstanceTracker(merge_distance=0.5)
        det = Detection3D(
            position=np.array([1.0, 2.0, 0.0]),
            label="chair",
            score=0.9,
            bbox_2d=np.array([100, 100, 200, 200]),
            depth=2.0,
            features=np.array([]),
        )
        matched = tracker.update([det])
        self.assertEqual(len(matched), 1)
        self.assertEqual(matched[0].label, "chair")
        self.assertEqual(len(tracker.objects), 1)

    def test_nearby_same_label_merges(self):
        """近距离同标签检测应合并。"""
        from perception.tracking.instance_tracker import InstanceTracker
        from perception.tracking.projection import Detection3D
        tracker = InstanceTracker(merge_distance=1.0)
        det1 = Detection3D(
            position=np.array([1.0, 2.0, 0.0]),
            label="chair",
            score=0.9,
            bbox_2d=np.array([100, 100, 200, 200]),
            depth=2.0,
            features=np.array([]),
        )
        det2 = Detection3D(
            position=np.array([1.3, 2.2, 0.0]),
            label="chair",
            score=0.85,
            bbox_2d=np.array([110, 110, 210, 210]),
            depth=2.1,
            features=np.array([]),
        )
        tracker.update([det1])
        tracker.update([det2])
        self.assertEqual(len(tracker.objects), 1)
        obj = next(iter(tracker.objects.values()))
        self.assertEqual(obj.detection_count, 2)

    def test_different_labels_separate(self):
        """不同标签应保持分离。"""
        from perception.tracking.instance_tracker import InstanceTracker
        from perception.tracking.projection import Detection3D
        tracker = InstanceTracker(merge_distance=1.0)
        det1 = Detection3D(
            position=np.array([1.0, 2.0, 0.0]),
            label="chair",
            score=0.9,
            bbox_2d=np.array([100, 100, 200, 200]),
            depth=2.0,
            features=np.array([]),
        )
        det2 = Detection3D(
            position=np.array([1.0, 2.0, 0.0]),
            label="desk",
            score=0.85,
            bbox_2d=np.array([100, 100, 200, 200]),
            depth=2.0,
            features=np.array([]),
        )
        tracker.update([det1])
        tracker.update([det2])
        self.assertEqual(len(tracker.objects), 2)

    def test_scene_graph_json(self):
        """场景图 JSON 应可正常解析。"""
        import json

        from perception.tracking.instance_tracker import InstanceTracker
        from perception.tracking.projection import Detection3D
        tracker = InstanceTracker()
        det = Detection3D(
            position=np.array([1.0, 2.0, 3.0]),
            label="fire extinguisher",
            score=0.95,
            bbox_2d=np.array([50, 50, 150, 150]),
            depth=3.0,
            features=np.array([]),
        )
        tracker.update([det])
        sg_json = tracker.get_scene_graph_json()
        sg = json.loads(sg_json)
        self.assertEqual(sg["object_count"], 1)
        self.assertEqual(sg["objects"][0]["label"], "fire extinguisher")
        # SG-Nav 层次结构字段
        self.assertIn("rooms", sg)
        self.assertIn("groups", sg)
        self.assertIn("subgraphs", sg)
        self.assertIn("hierarchy_edges", sg)
        self.assertIn("views", sg)

    def test_view_nodes_recorded(self):
        """记录视角后 scene_graph 应包含 view 节点。"""
        import json

        from perception.tracking.instance_tracker import InstanceTracker
        from perception.tracking.projection import Detection3D

        tracker = InstanceTracker()
        det = Detection3D(
            position=np.array([1.0, 2.0, 3.0]),
            label="chair",
            score=0.9,
            bbox_2d=np.array([10, 10, 30, 30]),
            depth=2.0,
            features=np.array([]),
        )
        matched = tracker.update([det])
        tracker.record_view(np.array([0.0, 0.0, 1.2]), [matched[0].object_id])

        sg = json.loads(tracker.get_scene_graph_json())
        self.assertGreaterEqual(len(sg.get("views", [])), 1)
        # 至少应有一条 view->object 关系边
        has_view_edge = any(
            e.get("parent_type") == "view" and e.get("child_type") == "object"
            for e in sg.get("hierarchy_edges", [])
        )
        self.assertTrue(has_view_edge)

    def test_feature_fusion_across_observations(self):
        """同一实例多次观测时应融合特征并保持归一化。"""
        from perception.tracking.instance_tracker import InstanceTracker
        from perception.tracking.projection import Detection3D

        tracker = InstanceTracker(merge_distance=1.0)
        det1 = Detection3D(
            position=np.array([0.0, 0.0, 1.0]),
            label="chair",
            score=0.8,
            bbox_2d=np.array([0, 0, 20, 20]),
            depth=1.0,
            features=np.array([1.0, 0.0, 0.0]),
        )
        det2 = Detection3D(
            position=np.array([0.2, 0.1, 1.0]),
            label="chair",
            score=0.85,
            bbox_2d=np.array([1, 1, 21, 21]),
            depth=1.0,
            features=np.array([0.0, 1.0, 0.0]),
        )

        tracker.update([det1])
        tracker.update([det2])
        obj = next(iter(tracker.objects.values()))
        self.assertEqual(obj.features.shape[0], 3)
        self.assertGreater(np.linalg.norm(obj.features), 0.9)

    def test_text_query(self):
        """文本查询应返回匹配物体。"""
        from perception.tracking.instance_tracker import InstanceTracker
        from perception.tracking.projection import Detection3D
        tracker = InstanceTracker()
        for label in ["chair", "fire extinguisher", "desk", "red chair"]:
            tracker.update([Detection3D(
                position=np.random.rand(3),
                label=label,
                score=0.8,
                bbox_2d=np.array([0, 0, 100, 100]),
                depth=2.0,
                features=np.array([]),
            )])
        results = tracker.query_by_text("chair")
        self.assertEqual(len(results), 2)  # "chair" and "red chair"


if __name__ == "__main__":
    unittest.main()

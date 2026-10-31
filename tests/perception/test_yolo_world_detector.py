"""
YOLO-World检测器单元测试 (与实际 yolo_world_detector.py API 对齐)

测试内容:
- 初始化参数
- 类别缓存 (enable_cache + _current_classes)
- detect() 接口
- detect_batch() 接口
- get_statistics() 接口
- 性能监控 (avg_fps)
- 错误处理
"""

import unittest
from threading import Lock
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from perception.detection.yolo_world_detector import YOLOWorldDetector


class _CpuArray:
    def __init__(self, values):
        self._values = np.asarray(values)

    def numpy(self):
        return self._values


class _Boxes:
    def __init__(self, xyxy, conf, cls):
        self.xyxy = _CpuArray(xyxy)
        self.conf = _CpuArray(conf)
        self.cls = _CpuArray(cls)
        self.cpu_calls = 0

    def __len__(self):
        return len(self.xyxy.numpy())

    def cpu(self):
        self.cpu_calls += 1
        return self


class _WarmPredictor:
    def __init__(self, results):
        self.done_warmup = True
        self.args = SimpleNamespace(conf=None, iou=None, verbose=True)
        self._lock = Lock()
        self._results = results
        self.calls = []
        self.dataset = None
        self.batch = None
        self.results = None

    def setup_source(self, source):
        self.calls.append(("setup_source", source))
        images = source if isinstance(source, list) else [source]
        self.dataset = iter([([f"image{i}.jpg" for i in range(len(images))], images, [""] * len(images))])

    def preprocess(self, images):
        self.calls.append(("preprocess", images))
        return "prepared"

    def inference(self, prepared):
        self.calls.append(("inference", prepared))
        return "predictions"

    def postprocess(self, predictions, prepared, images):
        self.calls.append(("postprocess", predictions, prepared, images))
        return self._results


def test_registered_detector_loads_explicit_weights(monkeypatch, tmp_path):
    import sys
    from types import SimpleNamespace

    from perception.backends import DetectorSpec
    from runtime.registry import get

    load = Mock(return_value=Mock())
    monkeypatch.setitem(sys.modules, "ultralytics", SimpleNamespace(YOLO=load))
    weights = str(tmp_path / "yolov8s-worldv2.pt")
    detector = get("detector", "yolo_world").create(DetectorSpec(name="yolo_world", model_path=weights))
    detector.load_model()
    load.assert_called_once_with(weights)


def test_default_detector_requests_pt_checkpoint(monkeypatch):
    import sys
    from types import SimpleNamespace

    load = Mock(return_value=Mock())
    monkeypatch.setitem(sys.modules, "ultralytics", SimpleNamespace(YOLO=load))
    YOLOWorldDetector(model_size="s").load_model()
    load.assert_called_once_with("yolov8s-worldv2.pt")


class TestYOLOWorldDetectorInit(unittest.TestCase):
    """YOLO-World检测器初始化测试"""

    def test_default_initialization(self):
        """默认参数初始化应该成功"""
        detector = YOLOWorldDetector()
        self.assertEqual(detector.model_size, "l")
        self.assertEqual(detector.confidence, 0.3)
        self.assertEqual(detector.iou_threshold, 0.5)
        self.assertFalse(detector.tensorrt)
        self.assertFalse(detector.tensorrt_int8)
        self.assertEqual(detector.batch_size, 1)
        self.assertTrue(detector.enable_cache)
        self.assertIsNone(detector._model)

    def test_custom_parameters(self):
        """自定义参数应该正确设置"""
        detector = YOLOWorldDetector(
            model_size="m",
            confidence=0.5,
            iou_threshold=0.7,
            tensorrt=True,
            tensorrt_int8=True,
            batch_size=4,
            enable_cache=False,
        )
        self.assertEqual(detector.model_size, "m")
        self.assertEqual(detector.confidence, 0.5)
        self.assertEqual(detector.iou_threshold, 0.7)
        self.assertTrue(detector.tensorrt)
        self.assertTrue(detector.tensorrt_int8)
        self.assertEqual(detector.batch_size, 4)
        self.assertFalse(detector.enable_cache)


class TestYOLOWorldDetectorClassCache(unittest.TestCase):
    """类别缓存测试 (通过 detect() 内部逻辑)"""

    def test_initial_classes_none(self):
        """初始类别应为 None"""
        detector = YOLOWorldDetector()
        self.assertIsNone(detector._current_classes)

    def test_detect_sets_classes(self):
        """第一次 detect() 应设置类别"""
        detector = YOLOWorldDetector()
        detector._model = Mock()
        detector._model.predictor = None

        # Mock predict 返回空结果
        mock_result = Mock()
        mock_result.boxes = None
        detector._model.predict = Mock(return_value=[mock_result])

        detector.detect(
            np.zeros((480, 640, 3), dtype=np.uint8),
            "chair . table"
        )

        detector._model.set_classes.assert_called_once()
        self.assertEqual(detector._current_classes, ["chair", "table"])

    def test_same_classes_no_redundant_set(self):
        """相同类别不应重复调用 set_classes"""
        detector = YOLOWorldDetector(enable_cache=True)
        detector._model = Mock()
        detector._model.predictor = None
        mock_result = Mock()
        mock_result.boxes = None
        detector._model.predict = Mock(return_value=[mock_result])

        detector.detect(np.zeros((480, 640, 3), dtype=np.uint8), "chair . table")
        detector.detect(np.zeros((480, 640, 3), dtype=np.uint8), "chair . table")

        # set_classes 只调用一次
        self.assertEqual(detector._model.set_classes.call_count, 1)

    def test_different_classes_updates(self):
        """不同类别应更新"""
        detector = YOLOWorldDetector(enable_cache=True)
        detector._model = Mock()
        detector._model.predictor = None
        mock_result = Mock()
        mock_result.boxes = None
        detector._model.predict = Mock(return_value=[mock_result])

        detector.detect(np.zeros((480, 640, 3), dtype=np.uint8), "chair . table")
        detector.detect(np.zeros((480, 640, 3), dtype=np.uint8), "door . window")

        self.assertEqual(detector._model.set_classes.call_count, 2)
        self.assertEqual(detector._current_classes, ["door", "window"])


class TestYOLOWorldDetectorDetection(unittest.TestCase):
    """检测功能测试"""

    def _make_mock_result(self, xyxys, confs, cls_ids):
        """创建 mock 检测结果"""
        import torch
        mock_result = Mock()
        if len(xyxys) == 0:
            mock_result.boxes = Mock()
            mock_result.boxes.__len__ = Mock(return_value=0)
            return mock_result

        boxes = Mock()
        boxes.__len__ = Mock(return_value=len(xyxys))

        # 创建单个 box mock 列表
        box_mocks = []
        for i in range(len(xyxys)):
            box = Mock()
            box.xyxy = [torch.tensor(xyxys[i:i+1], dtype=torch.float32)]
            box.conf = [torch.tensor([confs[i]])]
            box.cls = [torch.tensor([cls_ids[i]])]
            box_mocks.append(box)

        boxes.__getitem__ = Mock(side_effect=lambda i: box_mocks[i])
        boxes.__iter__ = Mock(return_value=iter(box_mocks))

        mock_result.boxes = boxes
        return mock_result

    def test_detect_with_no_model_raises(self):
        """模型未加载时 detect 应抛出异常"""
        detector = YOLOWorldDetector()
        with self.assertRaises(RuntimeError):
            detector.detect(
                np.zeros((480, 640, 3), dtype=np.uint8),
                "chair"
            )

    def test_detect_empty_classes_returns_empty(self):
        """空类别返回空列表"""
        detector = YOLOWorldDetector()
        detector._model = Mock()
        detector._model.predictor = None
        result = detector.detect(
            np.zeros((480, 640, 3), dtype=np.uint8),
            ""
        )
        self.assertEqual(len(result), 0)

    def test_detect_returns_detection2d_list(self):
        """detect() 应返回 Detection2D 列表"""
        detector = YOLOWorldDetector()
        detector._model = Mock()
        detector._model.predictor = None
        mock_result = Mock()
        mock_result.boxes = _Boxes(
            [[100.0, 100.0, 200.0, 200.0], [20.0, 30.0, 40.0, 50.0]],
            [0.85, 0.625],
            [0, 3],
        )
        detector._model.predict = Mock(return_value=[mock_result])

        detections = detector.detect(
            np.zeros((480, 640, 3), dtype=np.uint8),
            "chair . table"
        )

        self.assertEqual(mock_result.boxes.cpu_calls, 1)
        self.assertEqual([item.label for item in detections], ["chair", "class_3"])
        self.assertTrue(np.array_equal(detections[0].bbox, [100.0, 100.0, 200.0, 200.0]))
        self.assertAlmostEqual(detections[0].score, 0.85, places=6)
        self.assertTrue(np.array_equal(detections[1].bbox, [20.0, 30.0, 40.0, 50.0]))
        self.assertAlmostEqual(detections[1].score, 0.625, places=6)


class TestYOLOWorldDetectorBatchProcessing(unittest.TestCase):
    """批处理测试"""

    def test_batch_detect_no_model_raises(self):
        """模型未加载时 batch detect 应抛出异常"""
        detector = YOLOWorldDetector()
        with self.assertRaises(RuntimeError):
            detector.detect_batch(
                [np.zeros((480, 640, 3), dtype=np.uint8)],
                "chair"
            )

    def test_batch_detect_empty_returns_empty(self):
        """空图像列表返回空"""
        detector = YOLOWorldDetector()
        detector._model = Mock()
        detector._model.predictor = None
        result = detector.detect_batch([], "chair")
        self.assertEqual(len(result), 0)

    def test_batch_transfers_each_result_once_and_preserves_empty_results(self):
        detector = YOLOWorldDetector()
        detector._model = Mock()
        detector._model.predictor = None
        first = _Boxes([[1.0, 2.0, 3.0, 4.0], [5.0, 6.0, 7.0, 8.0]], [0.9, 0.8], [1, 0])
        empty = _Boxes([], [], [])
        detector._model.predict.return_value = [
            Mock(boxes=first),
            Mock(boxes=empty),
        ]

        detections = detector.detect_batch(
            [
                np.zeros((10, 10, 3), dtype=np.uint8),
                np.zeros((10, 10, 3), dtype=np.uint8),
            ],
            "chair . table",
        )

        self.assertEqual(first.cpu_calls, 1)
        self.assertEqual(empty.cpu_calls, 0)
        self.assertEqual([item.label for item in detections[0]], ["table", "chair"])
        self.assertEqual(detections[1], [])


class TestYOLOWorldDetectorPredictorStages(unittest.TestCase):
    def test_first_prediction_uses_official_predict_then_reuses_warmed_predictor(self):
        detector = YOLOWorldDetector(confidence=0.41, iou_threshold=0.62, device="cpu")
        model = Mock()
        model.predictor = None
        initial = Mock(boxes=_Boxes([], [], []))
        warmed = _WarmPredictor([Mock(boxes=_Boxes([], [], []))])

        def initialize(source, **kwargs):
            model.predictor = warmed
            self.assertEqual(source.shape, (12, 13, 3))
            self.assertEqual(
                kwargs,
                {"conf": 0.41, "iou": 0.62, "verbose": False, "device": "cpu"},
            )
            return [initial]

        model.predict.side_effect = initialize
        detector._model = model
        image = np.zeros((12, 13, 3), dtype=np.uint8)

        detector.detect(image, "person")
        detector.detect(image, "person")

        model.predict.assert_called_once()
        self.assertEqual(
            [call[0] for call in warmed.calls],
            ["setup_source", "preprocess", "inference", "postprocess"],
        )

    def test_warmed_predictor_receives_batch_and_current_nms_knobs(self):
        results = [
            Mock(boxes=_Boxes([[1, 2, 3, 4]], [0.7], [0])),
            Mock(boxes=_Boxes([[5, 6, 7, 8]], [0.8], [1])),
        ]
        predictor = _WarmPredictor(results)
        model = Mock(predictor=predictor)
        detector = YOLOWorldDetector(confidence=0.2, iou_threshold=0.3)
        detector._model = model
        detector.confidence = 0.44
        detector.iou_threshold = 0.66
        images = [
            np.zeros((10, 11, 3), dtype=np.uint8),
            np.zeros((12, 13, 3), dtype=np.uint8),
        ]

        detections = detector.detect_batch(images, "chair . table")

        model.predict.assert_not_called()
        self.assertIs(predictor.calls[0][1], images)
        self.assertEqual(predictor.calls[1], ("preprocess", images))
        self.assertEqual(predictor.args.conf, 0.44)
        self.assertEqual(predictor.args.iou, 0.66)
        self.assertFalse(predictor.args.verbose)
        self.assertEqual([item.label for item in detections[0]], ["chair"])
        self.assertEqual([item.label for item in detections[1]], ["table"])

    def test_class_change_updates_model_without_reinitializing_predictor(self):
        predictor = _WarmPredictor([Mock(boxes=_Boxes([], [], []))])
        model = Mock(predictor=predictor)
        detector = YOLOWorldDetector(enable_cache=True)
        detector._model = model
        image = np.zeros((10, 10, 3), dtype=np.uint8)

        detector.detect(image, "chair")
        detector.detect(image, "table")

        self.assertEqual(model.set_classes.call_args_list[0].args[0], ["chair"])
        self.assertEqual(model.set_classes.call_args_list[1].args[0], ["table"])
        model.predict.assert_not_called()


class TestYOLOWorldDetectorStatistics(unittest.TestCase):
    """性能统计测试"""

    def test_get_statistics_returns_expected_keys(self):
        """get_statistics 应返回所有需要的键"""
        detector = YOLOWorldDetector()
        stats = detector.get_statistics()

        expected_keys = [
            "detect_count", "total_time", "avg_time_per_detect",
            "avg_fps", "tensorrt_enabled", "tensorrt_int8",
            "current_classes",
        ]
        for key in expected_keys:
            self.assertIn(key, stats, f"Missing key: {key}")

    def test_initial_avg_fps_zero(self):
        """初始 FPS 为 0"""
        detector = YOLOWorldDetector()
        self.assertEqual(detector.avg_fps, 0.0)

    def test_reset_statistics(self):
        """重置统计"""
        detector = YOLOWorldDetector()
        detector._detect_count = 10
        detector._total_detect_time = 5.0
        detector._fps_history = [10.0, 20.0]

        detector.reset_statistics()

        self.assertEqual(detector._detect_count, 0)
        self.assertEqual(detector._total_detect_time, 0.0)
        self.assertEqual(len(detector._fps_history), 0)


class TestYOLOWorldDetectorShutdown(unittest.TestCase):
    """生命周期测试"""

    def test_shutdown_clears_model(self):
        """shutdown 应清除模型"""
        detector = YOLOWorldDetector()
        detector._model = Mock()
        detector._current_classes = ["chair"]

        detector.shutdown()

        self.assertIsNone(detector._model)
        self.assertIsNone(detector._current_classes)


class TestYOLOWorldDetectorTensorRT(unittest.TestCase):
    """TensorRT 测试"""

    def test_tensorrt_flags_stored(self):
        """TensorRT 标志应正确存储"""
        detector = YOLOWorldDetector(tensorrt=True, tensorrt_int8=True)
        self.assertTrue(detector.tensorrt)
        self.assertTrue(detector.tensorrt_int8)

    def test_tensorrt_disabled_by_default(self):
        """TensorRT 默认禁用"""
        detector = YOLOWorldDetector()
        self.assertFalse(detector.tensorrt)
        self.assertFalse(detector.tensorrt_int8)


if __name__ == '__main__':
    unittest.main()

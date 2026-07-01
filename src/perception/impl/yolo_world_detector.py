"""
YOLO-World检测器 - API适配实现

将现有的YOLOWorldDetector适配到DetectorAPI接口
"""

import logging
import time

import numpy as np

from ..api.detector_api import DetectorAPI
from ..api.exceptions import (
    DetectorError,
    DetectorInferenceError,
    DetectorInitError,
    InvalidImageError,
)
from ..api.types import BBox2D, Detection2D, PerceptionConfig

logger = logging.getLogger(__name__)


class YOLOWorldDetector(DetectorAPI):
    """
    YOLO-World检测器实现

    特性:
    - 开放词汇检测
    - TensorRT优化（可选）
    - 动态类别缓存
    - 性能监控
    """

    def __init__(self, config: PerceptionConfig | None = None):
        """
        初始化YOLO-World检测器

        Args:
            config: 感知配置对象

        Raises:
            DetectorInitError: 初始化失败
        """
        self.config = config or PerceptionConfig()

        # 从config或使用默认值
        self.model_size = getattr(config, 'model_size', 'l') if config else 'l'
        self.confidence = config.confidence_threshold if config else 0.3
        self.iou_threshold = config.iou_threshold if config else 0.5
        self.device = ""  # auto
        self.tensorrt = False
        self.tensorrt_int8 = False
        self.batch_size = 1
        self.enable_cache = True

        self._model = None
        self._current_classes: list[str] | None = None
        self._tensorrt_engine_path: str | None = None

        # 性能统计
        self._detect_count = 0
        self._total_detect_time = 0.0
        self._fps_history = []
        self._max_fps_history = 100

        # 自动加载模型
        try:
            self._load_model()
        except Exception as e:
            raise DetectorInitError(f"Failed to initialize YOLO-World: {e}") from e

    def _load_model(self):
        """加载YOLO-World模型"""
        try:
            from ultralytics import YOLO

            model_name = f"yolov8{self.model_size}-worldv2"
            logger.info(
                f"Loading YOLO-World: {model_name}, device={self.device or 'auto'}"
            )

            self._model = YOLO(model_name)

            # TensorRT优化（如果启用）
            if self.tensorrt:
                self._export_tensorrt(model_name)

            logger.info("YOLO-World model loaded successfully")

        except ImportError:
            raise DetectorInitError(
                "ultralytics not installed. Run: pip install ultralytics"
            ) from None
        except Exception as e:
            raise DetectorInitError(f"Failed to load YOLO-World model: {e}") from e

    def _export_tensorrt(self, model_name: str):
        """导出TensorRT引擎"""
        try:
            from ultralytics import YOLO

            logger.info("Exporting to TensorRT...")

            export_kwargs = {
                "format": "engine",
                "half": not self.tensorrt_int8,
                "device": self.device or 0,
                "workspace": 4,
            }

            if self.tensorrt_int8:
                export_kwargs["int8"] = True
                logger.info("Using INT8 quantization")

            self._model.export(**export_kwargs)

            engine_path = model_name.replace(".pt", ".engine")
            self._tensorrt_engine_path = engine_path
            self._model = YOLO(engine_path)

            logger.info(f"TensorRT engine loaded: {engine_path}")

        except Exception as e:
            logger.warning(f"TensorRT export failed, using PyTorch: {e}")
            self.tensorrt = False

    def detect(self, image: np.ndarray) -> list[Detection2D]:
        """
        检测图像中的物体

        Args:
            image: RGB图像，shape=(H, W, 3)，dtype=uint8

        Returns:
            2D检测结果列表

        Raises:
            DetectorInferenceError: 检测失败
            InvalidImageError: 图像格式无效
        """
        # 验证图像
        if image is None or not isinstance(image, np.ndarray):
            raise InvalidImageError("Image must be a numpy array")

        if image.ndim != 3 or image.shape[2] != 3:
            raise InvalidImageError(
                f"Image must be HxWx3, got shape {image.shape}"
            )

        if self._model is None:
            raise DetectorInferenceError("Model not loaded")

        if self._current_classes is None or len(self._current_classes) == 0:
            logger.warning("No classes set, returning empty detections")
            return []

        start_time = time.time()

        try:
            # 推理
            results = self._model.predict(
                image,
                conf=self.confidence,
                iou=self.iou_threshold,
                verbose=False,
                device=self.device,
            )

            # 解析结果
            detections = []
            if results and len(results) > 0:
                result = results[0]
                boxes = result.boxes

                if boxes is not None and len(boxes) > 0:
                    for i in range(len(boxes)):
                        box = boxes[i]
                        xyxy = box.xyxy[0].cpu().numpy()
                        conf = float(box.conf[0])
                        cls_id = int(box.cls[0])

                        if cls_id < len(self._current_classes):
                            label = self._current_classes[cls_id]
                        else:
                            label = f"class_{cls_id}"

                        detections.append(Detection2D(
                            label=label,
                            confidence=conf,
                            bbox=BBox2D(
                                x1=float(xyxy[0]),
                                y1=float(xyxy[1]),
                                x2=float(xyxy[2]),
                                y2=float(xyxy[3])
                            ),
                            class_id=cls_id
                        ))

            # 性能统计
            elapsed = time.time() - start_time
            self._detect_count += 1
            self._total_detect_time += elapsed

            fps = 1.0 / elapsed if elapsed > 0 else 0.0
            self._fps_history.append(fps)
            if len(self._fps_history) > self._max_fps_history:
                self._fps_history.pop(0)

            if self._detect_count % 100 == 0:
                avg_fps = sum(self._fps_history) / len(self._fps_history)
                logger.info(
                    f"Detection stats: count={self._detect_count}, "
                    f"avg_fps={avg_fps:.1f}, "
                    f"avg_time={self._total_detect_time / self._detect_count:.3f}s"
                )

            return detections

        except Exception as e:
            raise DetectorInferenceError(f"Detection failed: {e}") from e

    def set_classes(self, classes: list[str]):
        """
        设置检测类别

        Args:
            classes: 类别名称列表

        Raises:
            DetectorError: 设置失败
        """
        if not classes:
            raise DetectorError("Classes list cannot be empty")

        try:
            # 动态类别缓存
            if self.enable_cache and classes != self._current_classes:
                self._model.set_classes(classes)
                self._current_classes = classes
                logger.debug(f"Updated classes: {classes}")
            elif not self.enable_cache:
                self._model.set_classes(classes)
                self._current_classes = classes

        except Exception as e:
            raise DetectorError(f"Failed to set classes: {e}") from e

    def get_classes(self) -> list[str]:
        """
        获取当前检测类别

        Returns:
            类别名称列表
        """
        return self._current_classes or []

    def get_model_info(self) -> dict:
        """
        获取模型信息

        Returns:
            模型信息字典
        """
        return {
            "name": "YOLO-World",
            "version": f"v8{self.model_size}-worldv2",
            "input_size": (640, 640),
            "backend": "tensorrt" if self.tensorrt else "pytorch",
            "device": self.device or "auto",
            "tensorrt_int8": self.tensorrt_int8,
            "avg_fps": sum(self._fps_history) / len(self._fps_history) if self._fps_history else 0.0,
            "detect_count": self._detect_count,
        }

    def set_confidence_threshold(self, threshold: float):
        """
        设置置信度阈值

        Args:
            threshold: 置信度阈值，范围 [0, 1]
        """
        if not 0.0 <= threshold <= 1.0:
            raise DetectorError(f"Threshold must be in [0, 1], got {threshold}")

        self.confidence = threshold
        logger.info(f"Confidence threshold set to {threshold}")

    def get_confidence_threshold(self) -> float:
        """
        获取置信度阈值

        Returns:
            当前置信度阈值
        """
        return self.confidence

    def warmup(self, num_iterations: int = 10):
        """
        预热模型

        Args:
            num_iterations: 预热迭代次数
        """
        if self._model is None:
            logger.warning("Model not loaded, skipping warmup")
            return

        logger.info(f"Warming up model with {num_iterations} iterations...")

        # 创建dummy图像
        dummy_image = np.zeros((640, 640, 3), dtype=np.uint8)

        # 设置dummy类别
        if self._current_classes is None:
            self.set_classes(["person", "chair", "table"])

        for i in range(num_iterations):
            try:
                self.detect(dummy_image)
            except Exception as e:
                logger.warning(f"Warmup iteration {i} failed: {e}")

        logger.info("Warmup completed")

    def __del__(self):
        """清理资源"""
        if self._model is not None:
            del self._model
            self._model = None

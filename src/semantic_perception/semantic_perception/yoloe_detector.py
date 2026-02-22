"""
YOLO-E 开放词汇实例分割检测器 (USS-Nav 风格)

USS-Nav pipeline: YOLO-E → instance mask + label → mask + depth → 点云
替代原 YOLO-World bbox 检测, 提供逐像素 mask 用于精确 3D 点云重建。

关键优势 (vs YOLO-World bbox):
  - 输出实例 mask → 可生成精确物体点云 (而非单点投影)
  - LRPC prompt-free 模式: 内置 1200+ 类别, 无需外部语言模型
  - RepRTA text-prompted: 推理零开销的开放词汇
  - 比 YOLO-World v2 快 1.4x, 精度 +3.5 AP (LVIS)

参考:
  - YOLOE: Real-Time Seeing Anything (2025)
  - USS-Nav: Unified Spatio-Semantic Scene Graph (2026)
  - YOLOE-26: YOLO26 + YOLOE (2026, NMS-free)

依赖: pip install ultralytics>=8.3.0
"""

import logging
import time
from typing import Dict, List, Optional

import numpy as np

from .detector_base import DetectorBase, Detection2D

logger = logging.getLogger(__name__)


class YOLOEDetector(DetectorBase):
    """
    YOLO-E 实例分割检测器 (USS-Nav pipeline 核心)。

    输出 Detection2D 包含 instance mask, 供下游 mask→点云 投影使用。
    """

    def __init__(
        self,
        model_size: str = "l",
        confidence: float = 0.3,
        iou_threshold: float = 0.5,
        device: str = "",
        tensorrt: bool = False,
        max_detections: int = 30,
        mask_downsample: int = 1,
    ):
        self.model_size = model_size
        self.confidence = confidence
        self.iou_threshold = iou_threshold
        self.device = device
        self.tensorrt = tensorrt
        self.max_detections = max_detections
        self.mask_downsample = mask_downsample

        self._model = None
        self._current_classes: Optional[List[str]] = None

        self._detect_count = 0
        self._total_detect_time = 0.0
        self._fps_history: List[float] = []

    @property
    def avg_fps(self) -> float:
        if not self._fps_history:
            return 0.0
        return sum(self._fps_history) / len(self._fps_history)

    def load_model(self) -> None:
        """加载 YOLO-E 分割模型。"""
        try:
            from ultralytics import YOLO

            model_name = f"yoloe-11{self.model_size}-seg.pt"
            logger.info(
                "Loading YOLO-E seg: %s, device=%s, tensorrt=%s",
                model_name, self.device or "auto", self.tensorrt,
            )

            self._model = YOLO(model_name)

            if self.tensorrt:
                self._export_tensorrt(model_name)

            logger.info("YOLO-E seg model loaded")

        except Exception as e:
            logger.warning(
                "YOLO-E seg load failed, falling back to yolov8-seg: %s", e
            )
            try:
                from ultralytics import YOLO
                fallback = f"yolov8{self.model_size}-seg.pt"
                logger.info("Fallback: loading %s", fallback)
                self._model = YOLO(fallback)
                logger.info("Fallback seg model loaded")
            except Exception as e2:
                logger.error("Fallback also failed: %s", e2)
                raise

    def _export_tensorrt(self, model_name: str):
        """导出 TensorRT 引擎 (FP16)。"""
        try:
            from ultralytics import YOLO

            logger.info("Exporting YOLO-E to TensorRT FP16...")
            self._model.export(
                format="engine",
                half=True,
                device=self.device or 0,
                workspace=4,
            )
            engine_path = model_name.replace(".pt", ".engine")
            self._model = YOLO(engine_path)
            logger.info("TensorRT engine loaded: %s", engine_path)
        except Exception as e:
            logger.warning("TensorRT export failed, using PyTorch: %s", e)
            self.tensorrt = False

    def detect(self, rgb: np.ndarray, text_prompt: str) -> List[Detection2D]:
        """
        YOLO-E 实例分割检测。

        Returns:
            Detection2D 列表, 每个包含 .mask (HxW bool) 字段
        """
        if self._model is None:
            raise RuntimeError("Model not loaded. Call load_model() first.")

        start_time = time.time()

        classes = [c.strip() for c in text_prompt.split(".") if c.strip()]
        if not classes:
            return []

        if classes != self._current_classes:
            try:
                self._model.set_classes(classes)
            except AttributeError:
                pass
            self._current_classes = classes

        try:
            results = self._model.predict(
                rgb,
                conf=self.confidence,
                iou=self.iou_threshold,
                verbose=False,
                device=self.device,
                retina_masks=True,
                max_det=self.max_detections,
            )
        except Exception as e:
            logger.error("YOLO-E detection failed: %s", e)
            return []

        detections = []
        if results and len(results) > 0:
            result = results[0]
            boxes = result.boxes
            masks = result.masks

            if boxes is not None and len(boxes) > 0:
                h, w = rgb.shape[:2]

                for i in range(len(boxes)):
                    box = boxes[i]
                    xyxy = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])

                    label = classes[cls_id] if cls_id < len(classes) else f"class_{cls_id}"

                    instance_mask = None
                    if masks is not None and i < len(masks):
                        raw_mask = masks[i].data[0].cpu().numpy()
                        if raw_mask.shape != (h, w):
                            import cv2
                            raw_mask = cv2.resize(
                                raw_mask.astype(np.float32), (w, h),
                                interpolation=cv2.INTER_NEAREST,
                            )
                        instance_mask = raw_mask > 0.5

                    detections.append(Detection2D(
                        bbox=xyxy,
                        label=label,
                        score=conf,
                        class_id=cls_id,
                        mask=instance_mask,
                    ))

        elapsed = time.time() - start_time
        self._detect_count += 1
        self._total_detect_time += elapsed
        fps = 1.0 / elapsed if elapsed > 0 else 0.0
        self._fps_history.append(fps)
        if len(self._fps_history) > 100:
            self._fps_history.pop(0)

        if self._detect_count % 100 == 0:
            logger.info(
                "YOLO-E stats: count=%d, avg_fps=%.1f",
                self._detect_count, self.avg_fps,
            )

        return detections

    def get_statistics(self) -> Dict[str, float]:
        avg_time = (self._total_detect_time / self._detect_count
                    if self._detect_count > 0 else 0.0)
        return {
            "detect_count": self._detect_count,
            "avg_time_per_detect": avg_time,
            "avg_fps": self.avg_fps,
            "tensorrt_enabled": self.tensorrt,
        }

    def shutdown(self) -> None:
        self._model = None
        self._current_classes = None
        logger.info("YOLO-E detector shut down")

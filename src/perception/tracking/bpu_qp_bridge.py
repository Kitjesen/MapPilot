"""
BPU → qp_perception 桥接层

将 LingTu 的 BPU YOLO 检测器适配为 qp_perception 的 Detector 接口，
接入 FusionMOT 跟踪 + OSNet Re-ID + WeightedTargetSelector 目标选择。

链路:
  BPU YOLO (26ms, .hbm)
    → BPUDetectorAdapter (实现 qp_perception.Detector 协议)
    → FusionMOT (IoU + OSNet Re-ID + Kalman)
    → WeightedTargetSelector (多线索目标选择)
    → TargetObservation (给 PersonTracker / 导航用)

用法:
    from perception.tracking.bpu_qp_bridge import create_perception_pipeline
    pipeline = create_perception_pipeline()
    target = pipeline.process(frame, timestamp)
"""

import logging
import time

import numpy as np

logger = logging.getLogger(__name__)


def create_perception_pipeline(
    bpu_confidence: float = 0.22,
    bpu_iou_threshold: float = 0.45,
    bpu_max_detections: int = 64,
    bpu_min_box_size_px: int = 12,
    bpu_model_path: str | None = None,
    class_whitelist: list[str] | None = None,
    frame_width: int = 640,
    frame_height: int = 480,
    reid_backbone: str = "osnet_x1_0",
    tracker_type: str = "fusion",
):
    """
    创建完整的 BPU + qp_perception 感知管线。

    Args:
        bpu_confidence: BPU detection confidence threshold.
        bpu_iou_threshold: BPU NMS IoU threshold.
        bpu_max_detections: Max detections kept after NMS.
        bpu_min_box_size_px: Smallest bbox edge kept for tracking.
        bpu_model_path: BPU .hbm 模型路径 (None=自动发现)
        class_whitelist: 只保留这些类别 (None=默认 person, ["person"]=仅人)
        frame_width: 帧宽度 (用于目标选择器)
        frame_height: 帧高度
        reid_backbone: Re-ID 模型 ("osnet_x1_0" 或 "mobilenet")
        tracker_type: "fusion" (FusionMOT) 或 "botsort" (降级)

    Returns:
        PerceptionPipeline 实例
    """
    # BPU detector tuned for crowded person scenes by default.
    if class_whitelist is None:
        class_whitelist = ["person"]

    from ..detection.bpu_detector import BPUDetector
    bpu = BPUDetector(
        confidence=bpu_confidence,
        iou_threshold=bpu_iou_threshold,
        model_path=bpu_model_path,
        max_detections=bpu_max_detections,
        min_box_size_px=bpu_min_box_size_px,
    )
    bpu.load_model()

    detector = BPUDetectorAdapter(
        bpu_detector=bpu,
        class_whitelist=class_whitelist,
    )

    # qp_perception 跟踪器 + 选择器
    try:
        from qp_perception.config import SelectorConfig
        from qp_perception.reid.extractor import ReIDConfig, ReIDExtractor
        from qp_perception.selection.weighted import WeightedTargetSelector
        from qp_perception.tracking.fusion import FusionMOT, FusionMOTConfig  # noqa: F401

        # Re-ID
        reid_cfg = ReIDConfig(backbone=reid_backbone, device="")
        reid = ReIDExtractor(reid_cfg)

        # FusionMOT
        mot_cfg = FusionMOTConfig()
        tracker = FusionMOTWrapper(mot_cfg, reid)

        # 目标选择器
        sel_cfg = SelectorConfig(
            preferred_classes={"person": 0.5},
        )
        selector = WeightedTargetSelector(frame_width, frame_height, sel_cfg)

        logger.info(
            "qp_perception pipeline: BPU + FusionMOT + OSNet Re-ID + WeightedSelector"
        )
        return PerceptionPipeline(detector, tracker, selector)

    except ImportError as e:
        logger.warning("qp_perception not available (%s), falling back to BoT-SORT", e)
        # 降级: 用 ultralytics BoT-SORT
        return BPUTrackerFallback(bpu, class_whitelist)


class FusionMOTWrapper:
    """将 FusionMOT 的 numpy 接口包装为 qp_perception Tracker 协议。"""

    def __init__(self, config, reid_extractor):
        from qp_perception.tracking.fusion import FusionMOT
        self._mot = FusionMOT(config=config, feature_dim=reid_extractor.feature_dim)
        self._reid = reid_extractor

    def update(self, detections, timestamp: float):
        """Tracker 协议: detections → list[Track]。"""

        if not detections:
            # 传空数组更新（维护 lost tracks）
            empty = np.empty((0, 4))
            self._mot.update(empty, np.array([]), None, timestamp)
            return []

        # 转为 numpy: bboxes (N,4) xywh, confidences (N,)
        bboxes = np.array([[d.bbox.x, d.bbox.y, d.bbox.w, d.bbox.h] for d in detections], dtype=np.float32)
        confs = np.array([d.confidence for d in detections], dtype=np.float32)
        [d.class_id for d in detections]

        # Re-ID 特征提取 (需要 frame，但 Detector 协议不传 frame)
        # 这里 features=None，FusionMOT 会退化到 IoU+motion 匹配
        # 完整版需要在 PerceptionPipeline.process() 里提取
        features = None

        results = self._mot.update(bboxes, confs, features, timestamp)
        return self._to_tracks(results, detections, timestamp)

    def update_with_frame(self, detections, frame: np.ndarray, timestamp: float):
        """Selective Re-ID: only extract features for ambiguous detections."""

        if not detections:
            empty = np.empty((0, 4))
            self._mot.update(empty, np.array([]), None, timestamp)
            return []

        bboxes = np.array([[d.bbox.x, d.bbox.y, d.bbox.w, d.bbox.h] for d in detections], dtype=np.float32)
        confs = np.array([d.confidence for d in detections], dtype=np.float32)

        # Selective Re-ID: skip extraction for high-IoU matched detections
        results = self._mot.update_selective(
            bboxes, confs, timestamp,
            reid_extractor=self._reid,
            frame=frame,
        )
        return self._to_tracks(results, detections, timestamp)

    def _to_tracks(self, results, detections, timestamp):
        from qp_perception.types import BoundingBox, Track

        tracks = []
        for track_id, bbox_arr, conf in results:
            x, y, w, h = bbox_arr
            # 匹配 class_id: 找最近的检测框
            best_cls = "unknown"
            best_dist = float("inf")
            for d in detections:
                dx = d.bbox.x + d.bbox.w / 2 - (x + w / 2)
                dy = d.bbox.y + d.bbox.h / 2 - (y + h / 2)
                dist = dx * dx + dy * dy
                if dist < best_dist:
                    best_dist = dist
                    best_cls = d.class_id

            tracks.append(Track(
                track_id=int(track_id),
                bbox=BoundingBox(x=float(x), y=float(y), w=float(w), h=float(h)),
                confidence=float(conf),
                class_id=best_cls,
                first_seen_ts=timestamp,
                last_seen_ts=timestamp,
            ))
        return tracks


class BPUDetectorAdapter:
    """将 BPUDetector 适配为 qp_perception.Detector 协议。"""

    def __init__(
        self,
        bpu_detector,
        class_whitelist: list[str] | None = None,
        text_prompt: str = "",
    ):
        self._bpu = bpu_detector
        self._whitelist = set(c.lower() for c in class_whitelist) if class_whitelist else None
        # 构建 text_prompt (BPU 检测器需要)
        if text_prompt:
            self._prompt = text_prompt
        elif class_whitelist:
            self._prompt = " . ".join(class_whitelist)
        else:
            self._prompt = ""  # 不过滤

    def detect(self, frame: object, timestamp: float):
        """qp_perception Detector 协议: frame → list[Detection]。"""
        from qp_perception.types import BoundingBox, Detection

        if not isinstance(frame, np.ndarray):
            return []

        raw = self._bpu.detect(frame, self._prompt)

        detections = []
        for r in raw:
            label = r.label.lower()
            if self._whitelist and label not in self._whitelist:
                continue

            x1, y1, x2, y2 = r.bbox
            w, h = x2 - x1, y2 - y1
            if w <= 0 or h <= 0:
                continue

            detections.append(Detection(
                bbox=BoundingBox(x=float(x1), y=float(y1), w=float(w), h=float(h)),
                confidence=float(r.score),
                class_id=label,
                timestamp=timestamp,
            ))

        return detections


class PerceptionPipeline:
    """完整的感知管线: BPU 检测 → FusionMOT 跟踪 → 目标选择。"""

    def __init__(self, detector, tracker, selector):
        self._detector = detector
        self._tracker = tracker
        self._selector = selector
        self._frame_count = 0
        self._total_ms = 0.0

    def process(self, frame: np.ndarray, timestamp: float | None = None):
        """
        处理一帧，返回 (tracks, target)。

        Args:
            frame: BGR 图像
            timestamp: 时间戳 (None=自动)

        Returns:
            (tracks: list[Track], target: TargetObservation | None)
        """
        if timestamp is None:
            timestamp = time.time()

        t0 = time.monotonic()

        # 检测
        detections = self._detector.detect(frame, timestamp)

        # 跟踪 (FusionMOTWrapper 带 Re-ID 特征提取)
        if isinstance(self._tracker, FusionMOTWrapper) and detections:
            tracks = self._tracker.update_with_frame(detections, frame, timestamp)
        else:
            tracks = self._tracker.update(detections, timestamp)

        # 目标选择
        target = self._selector.select(tracks, timestamp)

        elapsed = (time.monotonic() - t0) * 1000
        self._frame_count += 1
        self._total_ms += elapsed

        if self._frame_count % 100 == 0:
            avg = self._total_ms / self._frame_count
            logger.info(
                "Pipeline avg: %.1fms/frame (%.1f FPS), %d tracks",
                avg, 1000 / max(avg, 1), len(tracks),
            )

        return tracks, target

    def get_all_tracks(self):
        """获取当前所有活跃 track (用于 VLM 选人时裁剪)。"""
        return self._tracker.get_active_tracks() if hasattr(self._tracker, 'get_active_tracks') else []

    @property
    def stats(self):
        if self._frame_count == 0:
            return {"frames": 0, "avg_ms": 0, "fps": 0}
        avg = self._total_ms / self._frame_count
        return {
            "frames": self._frame_count,
            "avg_ms": round(avg, 1),
            "fps": round(1000 / max(avg, 1), 1),
        }


class BPUTrackerFallback:
    """降级方案: qp_perception 不可用时用 BoT-SORT。"""

    def __init__(self, bpu_detector, class_whitelist=None):
        from .bpu_tracker import BPUTracker
        self._tracker = BPUTracker(bpu_detector, tracker_type="botsort")
        self._prompt = " . ".join(class_whitelist) if class_whitelist else "person"

    def process(self, frame, timestamp=None):
        tracked = self._tracker.track(frame, self._prompt)
        # 转为 (tracks, target) 格式, target=最高置信度的 person
        persons = [t for t in tracked if t.label == "person"]
        target = max(persons, key=lambda t: t.score) if persons else None
        return tracked, target

    @property
    def stats(self):
        return {"frames": 0, "avg_ms": 0, "fps": 0, "mode": "botsort_fallback"}

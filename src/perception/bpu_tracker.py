"""
BPU 检测 + BoT-SORT 跟踪器 — 实时多目标跟踪。

架构:
  BPUDetector.detect() → list[Detection2D]   (~26ms, BPU 硬件)
        ↓
  _DetectionResults wrapper (xyxy→xywh 格式转换)
        ↓
  BOTSORT.update()                            (~3ms, 纯 numpy/scipy)
        ↓
  list[TrackedDetection]                      (带持久 track_id)

跟踪器直接使用 ultralytics BoT-SORT 实现，不依赖 PyTorch 推理。
ReID 默认关闭，仅靠 IoU + Kalman 滤波，适合嵌入式场景。
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from .detector_base import Detection2D


@dataclass
class TrackedDetection:
    """带持久跟踪 ID 的检测结果。"""
    track_id: int             # BoT-SORT 持久 ID (跨帧稳定)
    bbox: np.ndarray          # [x1, y1, x2, y2] in pixels (float32)
    score: float              # 检测置信度 0-1
    label: str                # 检测类别文本
    class_id: int             # COCO class_id
    mask: np.ndarray | None = None  # HxW bool 实例分割 mask (可选)
    det_idx: int = -1         # 对应原始 Detection2D 的索引 (用于回溯 mask)


class _DetectionResults:
    """将 list[Detection2D] 包装为 BYTETracker.update() 期望的接口。

    BYTETracker/BOTSORT 要求 results 对象支持:
      - results.conf    → (N,) float32 置信度
      - results.cls     → (N,) float32 类别 ID
      - results.xywh    → (N, 4) float32 [cx, cy, w, h]
      - results.xyxy    → (N, 4) float32 [x1, y1, x2, y2]  (GMC 用)
      - results[mask]   → 同类型子集 (布尔索引)
      - len(results)    → int
    """

    def __init__(self, dets: list[Detection2D]):
        n = len(dets)
        if n == 0:
            self.conf = np.empty((0,), dtype=np.float32)
            self.cls = np.empty((0,), dtype=np.float32)
            self._xyxy = np.empty((0, 4), dtype=np.float32)
            self._orig_indices: list[int] = []
        else:
            self.conf = np.array([d.score for d in dets], dtype=np.float32)
            self.cls = np.array([d.class_id for d in dets], dtype=np.float32)
            self._xyxy = np.vstack([d.bbox.astype(np.float32) for d in dets])
            self._orig_indices = list(range(n))

    @property
    def xywh(self) -> np.ndarray:
        """xyxy → [cx, cy, w, h]"""
        if len(self._xyxy) == 0:
            return np.empty((0, 4), dtype=np.float32)
        x1, y1, x2, y2 = self._xyxy[:, 0], self._xyxy[:, 1], self._xyxy[:, 2], self._xyxy[:, 3]
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        w = x2 - x1
        h = y2 - y1
        return np.stack([cx, cy, w, h], axis=-1)

    @property
    def xyxy(self) -> np.ndarray:
        return self._xyxy

    def __len__(self) -> int:
        return len(self.conf)

    def __getitem__(self, mask) -> _DetectionResults:
        """支持布尔或整数索引，返回子集。"""
        obj = _DetectionResults.__new__(_DetectionResults)
        obj.conf = self.conf[mask]
        obj.cls = self.cls[mask]
        obj._xyxy = self._xyxy[mask]
        # 处理布尔 mask 和整数索引两种情况
        if isinstance(mask, (np.ndarray,)) and mask.dtype == bool:
            obj._orig_indices = [self._orig_indices[i] for i, m in enumerate(mask) if m]
        else:
            idx = np.arange(len(self.conf))[mask]
            obj._orig_indices = [self._orig_indices[i] for i in idx]
        return obj


def _build_botsort_args(
    track_high_thresh: float = 0.25,
    track_low_thresh: float = 0.1,
    new_track_thresh: float = 0.25,
    track_buffer: int = 30,
    match_thresh: float = 0.8,
    fuse_score: bool = True,
    gmc_method: str = "sparseOptFlow",
    proximity_thresh: float = 0.5,
    appearance_thresh: float = 0.8,
    with_reid: bool = False,
    model: str = "auto",
):
    """构造 BOTSORT 需要的 args 命名空间 (无需解析 yaml)。"""
    from types import SimpleNamespace
    return SimpleNamespace(
        tracker_type="botsort",
        track_high_thresh=track_high_thresh,
        track_low_thresh=track_low_thresh,
        new_track_thresh=new_track_thresh,
        track_buffer=track_buffer,
        match_thresh=match_thresh,
        fuse_score=fuse_score,
        gmc_method=gmc_method,
        proximity_thresh=proximity_thresh,
        appearance_thresh=appearance_thresh,
        with_reid=with_reid,
        model=model,
    )


class BPUTracker:
    """BPU 检测 + BoT-SORT 跟踪 = 实时多目标跟踪。

    用法::

        detector = BPUDetector(confidence=0.25)
        detector.load_model()
        tracker = BPUTracker(detector)

        for frame in video_stream:
            results = tracker.track(frame, "person . chair")
            for r in results:
                print(r.track_id, r.label, r.bbox)

    Args:
        bpu_detector: 已 load_model() 的 BPUDetector 实例。
        tracker_type: "botsort" 或 "bytetrack"。
        frame_rate: 视频帧率，影响 lost track 保留时长。
        track_high_thresh: 第一阶段匹配置信度阈值。
        track_low_thresh: 第二阶段低分检测阈值。
        new_track_thresh: 新建 track 的最低置信度。
        track_buffer: lost track 保留帧数。
        match_thresh: IoU 匹配阈值。
        gmc_method: 全局运动补偿方法 (sparseOptFlow/orb/sift/ecc/none)。
                    摄像头固定时设 "none" 可节省约 1ms。
    """

    def __init__(
        self,
        bpu_detector,
        tracker_type: str = "botsort",
        frame_rate: int = 30,
        track_high_thresh: float = 0.25,
        track_low_thresh: float = 0.1,
        new_track_thresh: float = 0.25,
        track_buffer: int = 30,
        match_thresh: float = 0.8,
        gmc_method: str = "sparseOptFlow",
    ):
        self._detector = bpu_detector
        self._tracker_type = tracker_type.lower()

        args = _build_botsort_args(
            track_high_thresh=track_high_thresh,
            track_low_thresh=track_low_thresh,
            new_track_thresh=new_track_thresh,
            track_buffer=track_buffer,
            match_thresh=match_thresh,
            gmc_method=gmc_method,
            with_reid=False,  # 嵌入式不用 ReID 模型
        )

        if self._tracker_type == "botsort":
            from ultralytics.trackers.bot_sort import BOTSORT
            self._tracker = BOTSORT(args, frame_rate=frame_rate)
        elif self._tracker_type == "bytetrack":
            from ultralytics.trackers.byte_tracker import BYTETracker
            self._tracker = BYTETracker(args, frame_rate=frame_rate)
        else:
            raise ValueError(f"tracker_type must be 'botsort' or 'bytetrack', got '{tracker_type}'")

        # 保存最新一帧的原始检测，用于 track_id → mask 的回溯
        self._last_dets: list[Detection2D] = []

        # 性能统计
        self.last_detect_ms: float = 0.0
        self.last_track_ms: float = 0.0

    def track(self, bgr_frame: np.ndarray, text_prompt: str) -> list[TrackedDetection]:
        """对单帧执行 BPU 检测 + BoT-SORT 跟踪。

        Args:
            bgr_frame: HxWx3 uint8 BGR 图像。
            text_prompt: ". " 分隔的目标标签 (e.g. "person . chair . door")。

        Returns:
            已激活 track 的 TrackedDetection 列表，按 track_id 升序排列。
            空帧返回空列表。
        """
        # 1. BPU 检测
        t0 = time.perf_counter()
        dets: list[Detection2D] = self._detector.detect(bgr_frame, text_prompt)
        self.last_detect_ms = (time.perf_counter() - t0) * 1000.0
        self._last_dets = dets

        # 2. 构造跟踪器输入
        t1 = time.perf_counter()
        results = _DetectionResults(dets)

        # 3. BOTSORT/BYTETracker.update()
        # 返回 (N, 7) float32: [x1, y1, x2, y2, track_id, score, cls, det_idx]
        # STrack.result = [*xyxy, track_id, score, cls, idx]
        raw_tracks = self._tracker.update(results, bgr_frame)
        self.last_track_ms = (time.perf_counter() - t1) * 1000.0

        if len(raw_tracks) == 0:
            return []

        # 4. 转换为 TrackedDetection
        # raw_tracks 列: [x1, y1, x2, y2, track_id, score, cls, det_idx]
        tracked: list[TrackedDetection] = []
        for row in raw_tracks:
            x1, y1, x2, y2 = row[0], row[1], row[2], row[3]
            track_id = int(row[4])
            score = float(row[5])
            class_id = int(row[6])
            det_idx = int(row[7])  # 原始检测列表中的索引 (STrack.idx)

            # 从原始检测中取 label / mask
            label = f"class_{class_id}"
            mask = None
            if 0 <= det_idx < len(dets):
                orig = dets[det_idx]
                label = orig.label
                mask = orig.mask

            tracked.append(TrackedDetection(
                track_id=track_id,
                bbox=np.array([x1, y1, x2, y2], dtype=np.float32),
                score=score,
                label=label,
                class_id=class_id,
                mask=mask,
                det_idx=det_idx,
            ))

        tracked.sort(key=lambda t: t.track_id)
        return tracked

    def reset(self) -> None:
        """重置跟踪状态 (换场景时调用)。"""
        self._tracker.reset()
        self._last_dets = []

    @property
    def active_track_ids(self) -> list[int]:
        """当前活跃 track ID 列表。"""
        return [t.track_id for t in self._tracker.tracked_stracks if t.is_activated]

    def timing_summary(self) -> str:
        """返回最近一帧的耗时摘要字符串。"""
        total = self.last_detect_ms + self.last_track_ms
        return (
            f"detect={self.last_detect_ms:.1f}ms  "
            f"track={self.last_track_ms:.1f}ms  "
            f"total={total:.1f}ms"
        )

"""
实例追踪器 - API适配实现

将现有的InstanceTracker适配到TrackerAPI接口
"""

import logging
import time

import numpy as np

from ..api.exceptions import TrackerError
from ..api.tracker_api import TrackerAPI
from ..api.types import Detection3D, PerceptionConfig, Position3D

logger = logging.getLogger(__name__)


class InstanceTracker(TrackerAPI):
    """
    实例追踪器实现

    特性:
    - 跨帧物体追踪
    - 位置平滑（EMA）
    - 重复检测合并
    - 空间关系计算
    """

    def __init__(self, config: PerceptionConfig | None = None):
        """
        初始化追踪器

        Args:
            config: 感知配置对象
        """
        self.config = config or PerceptionConfig()

        # 追踪参数
        self.merge_distance = config.merge_distance if config else 0.5
        self.iou_threshold = config.iou_threshold if config else 0.5
        self.max_age = float(self.config.metadata.get("max_age_sec", 3.0))
        self.min_hits = 3  # 最小命中次数
        self.ema_alpha = 0.3  # EMA平滑系数

        # 追踪状态
        self._tracks: dict[str, Detection3D] = {}
        self._next_id = 0
        self._frame_count = 0

        logger.info(
            f"InstanceTracker initialized: "
            f"merge_distance={self.merge_distance}, "
            f"max_age={self.max_age}"
        )

    def update(
        self,
        detections: list[Detection3D],
        timestamp: float | None = None
    ) -> list[Detection3D]:
        """
        更新追踪器

        Args:
            detections: 当前帧的3D检测结果
            timestamp: 时间戳（可选）

        Returns:
            追踪后的检测结果（带有持久化ID）

        Raises:
            TrackerError: 追踪失败
        """
        if timestamp is None:
            timestamp = time.time()

        self._frame_count += 1

        try:
            # 1. 匹配现有追踪
            matched_tracks = []
            unmatched_detections = []

            for det in detections:
                matched = False

                # 尝试匹配现有追踪
                for _track_id, track in self._tracks.items():
                    if self._should_merge(det, track):
                        # 更新追踪
                        self._update_track(track, det, timestamp)
                        matched_tracks.append(track)
                        matched = True
                        break

                if not matched:
                    unmatched_detections.append(det)

            # 2. 创建新追踪
            for det in unmatched_detections:
                new_track = self._create_track(det, timestamp)
                matched_tracks.append(new_track)

            # 3. 清理旧追踪
            self._prune_old_tracks(timestamp)

            return matched_tracks

        except Exception as e:
            raise TrackerError(f"Tracking update failed: {e}") from e

    def _should_merge(self, det: Detection3D, track: Detection3D) -> bool:
        """判断是否应该合并检测和追踪"""
        # 标签必须相同
        if det.label.lower() != track.label.lower():
            return False

        # 计算3D距离
        dist = np.linalg.norm(
            np.array([det.position_3d.x, det.position_3d.y, det.position_3d.z]) -
            np.array([track.position_3d.x, track.position_3d.y, track.position_3d.z])
        )

        return dist < self.merge_distance

    def _update_track(self, track: Detection3D, det: Detection3D, timestamp: float):
        """更新追踪（EMA平滑）"""
        # EMA位置平滑
        alpha = self.ema_alpha
        track.position_3d.x = alpha * det.position_3d.x + (1 - alpha) * track.position_3d.x
        track.position_3d.y = alpha * det.position_3d.y + (1 - alpha) * track.position_3d.y
        track.position_3d.z = alpha * det.position_3d.z + (1 - alpha) * track.position_3d.z

        # 更新置信度（取最大值）
        track.confidence = max(track.confidence, det.confidence)

        # 更新统计
        track.detection_count += 1
        track.last_seen = timestamp

        # 更新CLIP特征（如果有）
        if det.clip_feature is not None:
            if track.clip_feature is None:
                track.clip_feature = det.clip_feature
            else:
                # EMA平滑特征
                track.clip_feature = (
                    alpha * det.clip_feature +
                    (1 - alpha) * track.clip_feature
                )

    def _create_track(self, det: Detection3D, timestamp: float) -> Detection3D:
        """创建新追踪"""
        track_id = f"track_{self._next_id}"
        self._next_id += 1

        # 创建新追踪对象
        track = Detection3D(
            id=track_id,
            label=det.label,
            confidence=det.confidence,
            bbox_2d=det.bbox_2d,
            position_3d=Position3D(
                x=det.position_3d.x,
                y=det.position_3d.y,
                z=det.position_3d.z
            ),
            clip_feature=det.clip_feature.copy() if det.clip_feature is not None else None,
            detection_count=1,
            last_seen=timestamp
        )

        # 添加到追踪字典
        self._tracks[track_id] = track

        return track

    def _prune_old_tracks(self, current_time: float):
        """清理旧的追踪"""
        to_remove = []

        for track_id, track in self._tracks.items():
            age = current_time - track.last_seen

            # 删除太久未见的追踪
            if age > self.max_age:
                to_remove.append(track_id)

        for track_id in to_remove:
            del self._tracks[track_id]
            logger.debug(f"Removed old track: {track_id}")

    def get_all_tracks(self) -> list[Detection3D]:
        """
        获取所有追踪的物体

        Returns:
            所有追踪物体列表
        """
        return list(self._tracks.values())

    def get_track_by_id(self, track_id: str) -> Detection3D | None:
        """
        根据ID获取追踪物体

        Args:
            track_id: 追踪ID

        Returns:
            追踪物体，如果不存在返回None
        """
        return self._tracks.get(track_id)

    def remove_track(self, track_id: str):
        """
        移除追踪物体

        Args:
            track_id: 追踪ID
        """
        if track_id in self._tracks:
            del self._tracks[track_id]
            logger.debug(f"Removed track: {track_id}")

    def reset(self):
        """重置追踪器（清空所有追踪）"""
        self._tracks.clear()
        self._next_id = 0
        self._frame_count = 0
        logger.info("Tracker reset")

    def get_track_count(self) -> int:
        """
        获取当前追踪物体数量

        Returns:
            追踪物体数量
        """
        return len(self._tracks)

    def configure(self, config: dict):
        """
        配置追踪器

        Args:
            config: 配置字典
        """
        if "merge_distance" in config:
            self.merge_distance = config["merge_distance"]
            logger.info(f"Updated merge_distance: {self.merge_distance}")

        if "iou_threshold" in config:
            self.iou_threshold = config["iou_threshold"]
            logger.info(f"Updated iou_threshold: {self.iou_threshold}")

        if "max_age_sec" in config:
            self.max_age = float(config["max_age_sec"])
            logger.info(f"Updated max_age_sec: {self.max_age}")
        elif "max_age" in config:
            self.max_age = float(config["max_age"])
            logger.info(f"Updated max_age_sec: {self.max_age}")

        if "min_hits" in config:
            self.min_hits = config["min_hits"]
            logger.info(f"Updated min_hits: {self.min_hits}")

        if "ema_alpha" in config:
            self.ema_alpha = config["ema_alpha"]
            logger.info(f"Updated ema_alpha: {self.ema_alpha}")

    def prune_old_tracks(self, max_age: float):
        """
        清理旧的追踪

        Args:
            max_age: 最大年龄（秒）
        """
        current_time = time.time()
        to_remove = []

        for track_id, track in self._tracks.items():
            age = current_time - track.last_seen
            if age > max_age:
                to_remove.append(track_id)

        for track_id in to_remove:
            del self._tracks[track_id]

        if to_remove:
            logger.info(f"Pruned {len(to_remove)} old tracks")

    def get_statistics(self) -> dict:
        """
        获取追踪统计信息

        Returns:
            统计信息字典
        """
        if not self._tracks:
            return {
                "track_count": 0,
                "frame_count": self._frame_count,
                "avg_detection_count": 0.0,
                "avg_confidence": 0.0,
            }

        detection_counts = [t.detection_count for t in self._tracks.values()]
        confidences = [t.confidence for t in self._tracks.values()]

        return {
            "track_count": len(self._tracks),
            "frame_count": self._frame_count,
            "avg_detection_count": np.mean(detection_counts),
            "avg_confidence": np.mean(confidences),
            "max_detection_count": max(detection_counts),
            "min_detection_count": min(detection_counts),
        }

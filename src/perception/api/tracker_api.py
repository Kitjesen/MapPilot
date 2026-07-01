"""
Semantic Perception API - 追踪器接口

定义实例追踪器的抽象接口
"""

from abc import ABC, abstractmethod


from .types import Detection3D


class TrackerAPI(ABC):
    """
    实例追踪器API接口

    职责：
    - 跨帧追踪物体实例
    - 合并重复检测
    - 维护物体ID一致性

    支持的实现：
    - ConceptGraphs风格追踪器
    - 简单IOU追踪器
    - 自定义追踪器

    使用示例：
        tracker = InstanceTracker(config)
        tracked = tracker.update(detections_3d)
    """

    @abstractmethod
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
        pass

    @abstractmethod
    def get_all_tracks(self) -> list[Detection3D]:
        """
        获取所有追踪的物体

        Returns:
            所有追踪物体列表
        """
        pass

    @abstractmethod
    def get_track_by_id(self, track_id: str) -> Detection3D | None:
        """
        根据ID获取追踪物体

        Args:
            track_id: 追踪ID

        Returns:
            追踪物体，如果不存在返回None
        """
        pass

    @abstractmethod
    def remove_track(self, track_id: str):
        """
        移除追踪物体

        Args:
            track_id: 追踪ID
        """
        pass

    @abstractmethod
    def reset(self):
        """重置追踪器（清空所有追踪）"""
        pass

    @abstractmethod
    def get_track_count(self) -> int:
        """
        获取当前追踪物体数量

        Returns:
            追踪物体数量
        """
        pass

    @abstractmethod
    def configure(self, config: dict):
        """
        配置追踪器

        Args:
            config: 配置字典，可能包含：
                - merge_distance: 合并距离阈值
                - iou_threshold: IOU阈值
                - max_age: 最大丢失帧数
                - min_hits: 最小命中次数
        """
        pass

    def prune_old_tracks(self, max_age: float):
        """
        清理旧的追踪（可选实现）

        Args:
            max_age: 最大年龄（秒）
        """
        pass

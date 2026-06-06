"""
Semantic Perception API - 公共类型定义

定义所有API接口使用的数据类型
"""

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    import numpy as np


@dataclass
class BBox2D:
    """2D边界框"""
    x1: float
    y1: float
    x2: float
    y2: float

    def to_list(self) -> list[float]:
        """转换为列表格式"""
        return [self.x1, self.y1, self.x2, self.y2]

    def area(self) -> float:
        """计算面积"""
        return (self.x2 - self.x1) * (self.y2 - self.y1)

    def center(self) -> tuple:
        """计算中心点"""
        return ((self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2)


@dataclass
class Position3D:
    """3D位置"""
    x: float
    y: float
    z: float

    def to_list(self) -> list[float]:
        """转换为列表格式"""
        return [self.x, self.y, self.z]

    def to_dict(self) -> dict[str, float]:
        """转换为字典格式"""
        return {"x": self.x, "y": self.y, "z": self.z}


@dataclass
class Detection2D:
    """2D检测结果"""
    label: str
    confidence: float
    bbox: BBox2D
    class_id: int | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """转换为字典格式"""
        return {
            "label": self.label,
            "confidence": self.confidence,
            "bbox": self.bbox.to_list(),
            "class_id": self.class_id,
            "metadata": self.metadata
        }


@dataclass
class Detection3D:
    """3D检测结果"""
    id: str
    label: str
    confidence: float
    bbox_2d: BBox2D
    position_3d: Position3D
    clip_feature: "np.ndarray | None" = None
    detection_count: int = 1
    last_seen: float = 0.0
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """转换为字典格式（不包含numpy数组）"""
        return {
            "id": self.id,
            "label": self.label,
            "confidence": self.confidence,
            "bbox_2d": self.bbox_2d.to_list(),
            "position_3d": self.position_3d.to_dict(),
            "detection_count": self.detection_count,
            "last_seen": self.last_seen,
            "has_clip_feature": self.clip_feature is not None,
            "metadata": self.metadata
        }


@dataclass
class Relation:
    """场景图关系"""
    subject_id: str
    predicate: str  # near, on, left_of, right_of, etc.
    object_id: str
    confidence: float = 1.0
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """转换为字典格式"""
        return {
            "subject_id": self.subject_id,
            "predicate": self.predicate,
            "object_id": self.object_id,
            "confidence": self.confidence,
            "metadata": self.metadata
        }


@dataclass
class Region:
    """场景区域"""
    name: str
    object_ids: list[str]
    center: Position3D | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """转换为字典格式"""
        return {
            "name": self.name,
            "object_ids": self.object_ids,
            "center": self.center.to_dict() if self.center else None,
            "metadata": self.metadata
        }


@dataclass
class SceneGraph:
    """场景图"""
    objects: list[Detection3D]
    relations: list[Relation]
    regions: list[Region] = field(default_factory=list)
    timestamp: float = 0.0
    frame_id: str = "map"
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """转换为字典格式"""
        return {
            "objects": [obj.to_dict() for obj in self.objects],
            "relations": [rel.to_dict() for rel in self.relations],
            "regions": [reg.to_dict() for reg in self.regions],
            "timestamp": self.timestamp,
            "frame_id": self.frame_id,
            "metadata": self.metadata
        }

    def get_object_by_id(self, obj_id: str) -> Detection3D | None:
        """根据ID获取物体"""
        for obj in self.objects:
            if obj.id == obj_id:
                return obj
        return None

    def get_objects_by_label(self, label: str) -> list[Detection3D]:
        """根据标签获取物体列表"""
        return [obj for obj in self.objects if obj.label.lower() == label.lower()]


@dataclass
class CameraInfo:
    """相机参数"""
    fx: float  # 焦距x
    fy: float  # 焦距y
    cx: float  # 主点x
    cy: float  # 主点y
    width: int
    height: int
    depth_scale: float = 0.001  # 深度缩放因子

    def to_dict(self) -> dict[str, Any]:
        """转换为字典格式"""
        return {
            "fx": self.fx,
            "fy": self.fy,
            "cx": self.cx,
            "cy": self.cy,
            "width": self.width,
            "height": self.height,
            "depth_scale": self.depth_scale
        }


@dataclass
class PerceptionConfig:
    """感知系统配置"""
    detector_type: str = "yolo_world"
    encoder_type: str = "clip"
    enable_tracking: bool = True
    enable_scene_graph: bool = True
    confidence_threshold: float = 0.3
    iou_threshold: float = 0.5
    merge_distance: float = 0.5
    max_depth: float = 6.0
    min_depth: float = 0.3
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """转换为字典格式"""
        return {
            "detector_type": self.detector_type,
            "encoder_type": self.encoder_type,
            "enable_tracking": self.enable_tracking,
            "enable_scene_graph": self.enable_scene_graph,
            "confidence_threshold": self.confidence_threshold,
            "iou_threshold": self.iou_threshold,
            "merge_distance": self.merge_distance,
            "max_depth": self.max_depth,
            "min_depth": self.min_depth,
            "metadata": self.metadata
        }

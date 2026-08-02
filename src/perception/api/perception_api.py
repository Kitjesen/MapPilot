"""
Semantic Perception API - 感知系统接口

定义语义感知系统的顶层抽象接口
"""

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import numpy as np

from .types import CameraInfo, Detection3D, PerceptionConfig, SceneGraph


class PerceptionAPI(ABC):
    """
    语义感知API接口

    职责：
    - 处理RGB-D图像
    - 输出3D检测结果
    - 维护场景图
    - 提供统一的感知接口

    运行时实现由 ``PerceptionModule`` 提供；本接口只描述组件合同。

        detections = perception.process_frame(
            rgb_image, depth_image, camera_info, transform
        )

        scene_graph = perception.get_scene_graph()
    """

    @abstractmethod
    def process_frame(
        self,
        rgb_image: "np.ndarray",
        depth_image: "np.ndarray",
        camera_info: CameraInfo,
        transform: "np.ndarray | None" = None
    ) -> list[Detection3D]:
        """
        处理单帧RGB-D图像

        Args:
            rgb_image: RGB图像，shape=(H, W, 3)，dtype=uint8
            depth_image: 深度图像，shape=(H, W)，dtype=uint16或float32
            camera_info: 相机内参
            transform: 相机到世界坐标系的变换矩阵 (4x4)，可选

        Returns:
            3D检测结果列表

        Raises:
            PerceptionAPIError: 处理失败
            InvalidImageError: 图像格式无效
            InvalidDepthError: 深度图无效
            InvalidCameraInfoError: 相机参数无效
        """
        pass

    @abstractmethod
    def get_scene_graph(self) -> SceneGraph:
        """
        获取当前场景图

        Returns:
            场景图对象，包含物体、关系、区域

        Raises:
            SceneGraphError: 场景图生成失败
        """
        pass

    @abstractmethod
    def get_detections(self) -> list[Detection3D]:
        """
        获取当前所有检测结果

        Returns:
            3D检测结果列表
        """
        pass

    @abstractmethod
    def query_objects(
        self,
        label: str | None = None,
        min_confidence: float = 0.0,
        position_filter: dict | None = None
    ) -> list[Detection3D]:
        """
        查询物体

        Args:
            label: 物体标签（可选），例如 "chair"
            min_confidence: 最小置信度阈值
            position_filter: 位置过滤器（可选），例如 {"x_min": 0, "x_max": 5}

        Returns:
            符合条件的物体列表
        """
        pass

    @abstractmethod
    def reset(self):
        """
        重置感知系统

        清空所有历史检测和场景图
        """
        pass

    @abstractmethod
    def configure(self, config: PerceptionConfig):
        """
        配置感知系统

        Args:
            config: 感知配置对象

        Raises:
            ConfigurationError: 配置失败
        """
        pass

    @abstractmethod
    def get_config(self) -> PerceptionConfig:
        """
        获取当前配置

        Returns:
            感知配置对象
        """
        pass

    @abstractmethod
    def get_statistics(self) -> dict:
        """
        获取统计信息

        Returns:
            统计信息字典，包含：
            - total_detections: 总检测数
            - unique_objects: 唯一物体数
            - avg_confidence: 平均置信度
            - processing_time_ms: 平均处理时间（毫秒）
            - fps: 处理帧率
        """
        pass

    @abstractmethod
    def set_detection_classes(self, classes: list[str]):
        """
        设置检测类别

        Args:
            classes: 类别名称列表
        """
        pass

    @abstractmethod
    def get_detection_classes(self) -> list[str]:
        """
        获取当前检测类别

        Returns:
            类别名称列表
        """
        pass

    def enable_visualization(self, enable: bool = True):
        """
        启用/禁用可视化（可选实现）

        Args:
            enable: 是否启用
        """
        pass

    def get_visualization(self) -> "np.ndarray | None":
        """
        获取可视化图像（可选实现）

        Returns:
            可视化图像，如果未启用返回None
        """
        pass

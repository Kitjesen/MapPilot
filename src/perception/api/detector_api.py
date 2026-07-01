"""
Semantic Perception API - 检测器接口

定义物体检测器的抽象接口
"""

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import numpy as np

from .types import Detection2D


class DetectorAPI(ABC):
    """
    物体检测器API接口

    支持的实现：
    - YOLO-World (开放词汇检测)
    - Grounding DINO (开放词汇检测)
    - 自定义检测器

    使用示例：
        detector = YOLOWorldDetector(config)
        detections = detector.detect(image)
    """

    @abstractmethod
    def detect(self, image: "np.ndarray") -> list[Detection2D]:
        """
        检测图像中的物体

        Args:
            image: RGB图像，shape=(H, W, 3)，dtype=uint8

        Returns:
            2D检测结果列表

        Raises:
            DetectorError: 检测失败
            InvalidImageError: 图像格式无效
        """
        pass

    @abstractmethod
    def set_classes(self, classes: list[str]):
        """
        设置检测类别（开放词汇检测）

        Args:
            classes: 类别名称列表，例如 ["chair", "table", "person"]

        Raises:
            DetectorError: 设置失败
        """
        pass

    @abstractmethod
    def get_classes(self) -> list[str]:
        """
        获取当前检测类别

        Returns:
            类别名称列表
        """
        pass

    @abstractmethod
    def get_model_info(self) -> dict:
        """
        获取模型信息

        Returns:
            模型信息字典，包含：
            - name: 模型名称
            - version: 版本
            - input_size: 输入尺寸 (width, height)
            - backend: 推理后端 (pytorch | tensorrt | onnx)
            - device: 设备 (cpu | cuda)
        """
        pass

    @abstractmethod
    def set_confidence_threshold(self, threshold: float):
        """
        设置置信度阈值

        Args:
            threshold: 置信度阈值，范围 [0, 1]
        """
        pass

    @abstractmethod
    def get_confidence_threshold(self) -> float:
        """
        获取置信度阈值

        Returns:
            当前置信度阈值
        """
        pass

    def warmup(self, num_iterations: int = 10):
        """
        预热模型（可选实现）

        Args:
            num_iterations: 预热迭代次数
        """
        pass

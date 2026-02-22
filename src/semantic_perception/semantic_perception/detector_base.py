"""
检测器抽象基类 — 支持多种开放词汇检测后端的统一接口。

当前实现:
  - YOLOWorldDetector  (默认, pip install ultralytics, ~50MB, 10+ FPS on Jetson)
  - GroundingDINODetector (可选, 高精度, ~900MB, 需 clone 仓库)

对比 (论文参考):
  - LOVON (2024):      YOLO + CLIP 二阶段
  - SG-Nav (NeurIPS 2024): GroundingDINO
  - OrionNav (2025):   OWL-ViT
  - YOLO-World (CVPR 2024): 本实现的默认方案
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


@dataclass
class Detection2D:
    """2D 检测结果。"""
    bbox: np.ndarray          # [x1, y1, x2, y2] in pixels
    score: float              # 检测置信度 0-1
    label: str                # 检测类别文本
    class_id: int = -1        # 可选类别 ID
    features: np.ndarray = field(default_factory=lambda: np.array([]))  # 语义特征 (可选)
    mask: Optional[np.ndarray] = None  # HxW bool 实例分割 mask (USS-Nav: mask→点云)


class DetectorBase(ABC):
    """检测器抽象基类。"""

    @abstractmethod
    def load_model(self) -> None:
        """加载模型权重, 在构造后调用一次。"""
        ...

    @abstractmethod
    def detect(self, rgb: np.ndarray, text_prompt: str) -> List[Detection2D]:
        """
        对单张 RGB 图片进行开放词汇检测。

        Args:
            rgb: HxWx3 uint8 BGR 图像
            text_prompt: 文本提示, 多类别用 ". " 分隔 (GroundingDINO 格式)

        Returns:
            检测列表
        """
        ...

    @abstractmethod
    def shutdown(self) -> None:
        """释放 GPU 资源。"""
        ...

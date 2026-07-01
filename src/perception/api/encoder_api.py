"""
Semantic Perception API - 编码器接口

定义视觉-语言编码器的抽象接口
"""

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import numpy as np



class EncoderAPI(ABC):
    """
    视觉-语言编码器API接口

    支持的实现：
    - CLIP (ViT-B/32, ViT-L/14)
    - BLIP
    - 自定义编码器

    使用示例：
        encoder = CLIPEncoder(model="ViT-B/32")
        image_feat = encoder.encode_image(image)
        text_feat = encoder.encode_text("a red chair")
        similarity = encoder.compute_similarity(image_feat, text_feat)
    """

    @abstractmethod
    def encode_image(self, image: "np.ndarray") -> "np.ndarray":
        """
        编码图像

        Args:
            image: RGB图像，shape=(H, W, 3)，dtype=uint8

        Returns:
            图像特征向量，shape=(D,)，dtype=float32

        Raises:
            EncoderError: 编码失败
            InvalidImageError: 图像格式无效
        """
        pass

    @abstractmethod
    def encode_text(self, text: str) -> "np.ndarray":
        """
        编码文本

        Args:
            text: 文本字符串

        Returns:
            文本特征向量，shape=(D,)，dtype=float32

        Raises:
            EncoderError: 编码失败
        """
        pass

    @abstractmethod
    def encode_images_batch(self, images: list["np.ndarray"]) -> "np.ndarray":
        """
        批量编码图像

        Args:
            images: 图像列表

        Returns:
            图像特征矩阵，shape=(N, D)，dtype=float32

        Raises:
            EncoderError: 编码失败
        """
        pass

    @abstractmethod
    def encode_texts_batch(self, texts: list[str]) -> "np.ndarray":
        """
        批量编码文本

        Args:
            texts: 文本列表

        Returns:
            文本特征矩阵，shape=(N, D)，dtype=float32

        Raises:
            EncoderError: 编码失败
        """
        pass

    @abstractmethod
    def compute_similarity(
        self,
        image_features: "np.ndarray",
        text_features: "np.ndarray"
    ) -> float:
        """
        计算图像-文本相似度

        Args:
            image_features: 图像特征，shape=(D,)
            text_features: 文本特征，shape=(D,)

        Returns:
            相似度分数，范围 [0, 1]

        Raises:
            EncoderError: 计算失败
        """
        pass

    @abstractmethod
    def compute_similarity_matrix(
        self,
        image_features: "np.ndarray",
        text_features: "np.ndarray"
    ) -> "np.ndarray":
        """
        计算相似度矩阵

        Args:
            image_features: 图像特征矩阵，shape=(N, D)
            text_features: 文本特征矩阵，shape=(M, D)

        Returns:
            相似度矩阵，shape=(N, M)，范围 [0, 1]

        Raises:
            EncoderError: 计算失败
        """
        pass

    @abstractmethod
    def get_feature_dim(self) -> int:
        """
        获取特征维度

        Returns:
            特征向量维度
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
            - feature_dim: 特征维度
            - device: 设备 (cpu | cuda)
        """
        pass

    def warmup(self, num_iterations: int = 10):
        """
        预热模型（可选实现）

        Args:
            num_iterations: 预热迭代次数
        """
        pass

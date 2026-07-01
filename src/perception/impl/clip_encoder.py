"""
CLIP编码器 - API适配实现

将现有的CLIPEncoder适配到EncoderAPI接口
"""

import hashlib
import logging
import time

import numpy as np

from ..api.encoder_api import EncoderAPI
from ..api.exceptions import (
    EncoderInferenceError,
    EncoderInitError,
)
from ..api.types import PerceptionConfig

logger = logging.getLogger(__name__)


class CLIPEncoder(EncoderAPI):
    """
    CLIP编码器实现

    特性:
    - 图像和文本编码
    - 特征缓存机制
    - 批处理优化
    - 多尺度特征（可选）
    """

    def __init__(self, config: PerceptionConfig | None = None):
        """
        初始化CLIP编码器

        Args:
            config: 感知配置对象

        Raises:
            EncoderInitError: 初始化失败
        """
        self.config = config or PerceptionConfig()

        # 配置参数
        self.model_name = "ViT-B/32"
        self.device = "cuda"
        self.enable_cache = True
        self.cache_size = 1000
        self.batch_size = 32
        self.multi_scale = False

        self._model = None
        self._preprocess = None
        self._tokenizer = None
        self._feature_dim: int = 0

        # 特征缓存（LRU）
        self._image_cache: dict[str, np.ndarray] = {}
        self._text_cache: dict[str, np.ndarray] = {}
        self._cache_hits = 0
        self._cache_misses = 0

        # 性能统计
        self._encode_count = 0
        self._total_encode_time = 0.0

        # 自动加载模型
        try:
            self._load_model()
        except Exception as e:
            raise EncoderInitError(f"Failed to initialize CLIP: {e}") from e

    def _load_model(self):
        """加载CLIP模型"""
        try:
            import open_clip
            import torch

            logger.info(f"Loading CLIP model: {self.model_name}")

            # 加载模型
            self._model, _, self._preprocess = open_clip.create_model_and_transforms(
                self.model_name,
                pretrained="openai",
                device=self.device
            )

            # 获取tokenizer
            self._tokenizer = open_clip.get_tokenizer(self.model_name)

            # 设置为评估模式
            self._model.eval()

            # 获取特征维度
            with torch.no_grad():
                dummy_text = self._tokenizer(["test"])
                dummy_features = self._model.encode_text(dummy_text.to(self.device))
                self._feature_dim = dummy_features.shape[-1]

            logger.info(
                f"CLIP model loaded: {self.model_name}, "
                f"feature_dim={self._feature_dim}, device={self.device}"
            )

        except ImportError:
            raise EncoderInitError(
                "open_clip not installed. Run: pip install open-clip-torch"
            ) from None
        except Exception as e:
            raise EncoderInitError(f"Failed to load CLIP model: {e}") from e

    def encode_image(self, image: np.ndarray) -> np.ndarray:
        """
        编码图像

        Args:
            image: RGB图像，shape=(H, W, 3)，dtype=uint8

        Returns:
            图像特征向量 (D,)

        Raises:
            EncoderInferenceError: 编码失败
        """
        if self._model is None:
            raise EncoderInferenceError("Model not loaded")

        # 检查缓存
        if self.enable_cache:
            cache_key = self._compute_image_hash(image)
            if cache_key in self._image_cache:
                self._cache_hits += 1
                return self._image_cache[cache_key]
            self._cache_misses += 1

        start_time = time.time()

        try:
            import torch
            from PIL import Image

            # 转换为PIL Image
            pil_image = Image.fromarray(image)

            # 预处理
            image_tensor = self._preprocess(pil_image).unsqueeze(0).to(self.device)

            # 编码
            with torch.no_grad():
                image_features = self._model.encode_image(image_tensor)
                image_features = image_features / image_features.norm(dim=-1, keepdim=True)

            # 转换为numpy
            features = image_features.cpu().numpy()[0]

            # 缓存
            if self.enable_cache:
                self._update_cache(self._image_cache, cache_key, features)

            # 性能统计
            elapsed = time.time() - start_time
            self._encode_count += 1
            self._total_encode_time += elapsed

            return features

        except Exception as e:
            raise EncoderInferenceError(f"Image encoding failed: {e}") from e

    def encode_text(self, text: str) -> np.ndarray:
        """
        编码文本

        Args:
            text: 文本字符串

        Returns:
            文本特征向量 (D,)

        Raises:
            EncoderInferenceError: 编码失败
        """
        if self._model is None:
            raise EncoderInferenceError("Model not loaded")

        # 检查缓存
        if self.enable_cache:
            cache_key = text
            if cache_key in self._text_cache:
                self._cache_hits += 1
                return self._text_cache[cache_key]
            self._cache_misses += 1

        start_time = time.time()

        try:
            import torch

            # Tokenize
            text_tokens = self._tokenizer([text]).to(self.device)

            # 编码
            with torch.no_grad():
                text_features = self._model.encode_text(text_tokens)
                text_features = text_features / text_features.norm(dim=-1, keepdim=True)

            # 转换为numpy
            features = text_features.cpu().numpy()[0]

            # 缓存
            if self.enable_cache:
                self._update_cache(self._text_cache, cache_key, features)

            # 性能统计
            elapsed = time.time() - start_time
            self._encode_count += 1
            self._total_encode_time += elapsed

            return features

        except Exception as e:
            raise EncoderInferenceError(f"Text encoding failed: {e}") from e

    def encode_images_batch(self, images: list[np.ndarray]) -> np.ndarray:
        """
        批量编码图像

        Args:
            images: 图像列表

        Returns:
            特征矩阵 (N, D)

        Raises:
            EncoderInferenceError: 编码失败
        """
        if not images:
            return np.array([])

        try:
            import torch
            from PIL import Image

            # 预处理所有图像
            image_tensors = []
            for img in images:
                pil_image = Image.fromarray(img)
                tensor = self._preprocess(pil_image)
                image_tensors.append(tensor)

            # 批处理
            batch_tensor = torch.stack(image_tensors).to(self.device)

            # 编码
            with torch.no_grad():
                features = self._model.encode_image(batch_tensor)
                features = features / features.norm(dim=-1, keepdim=True)

            return features.cpu().numpy()

        except Exception as e:
            raise EncoderInferenceError(f"Batch image encoding failed: {e}") from e

    def encode_texts_batch(self, texts: list[str]) -> np.ndarray:
        """
        批量编码文本

        Args:
            texts: 文本列表

        Returns:
            特征矩阵 (N, D)

        Raises:
            EncoderInferenceError: 编码失败
        """
        if not texts:
            return np.array([])

        try:
            import torch

            # Tokenize
            text_tokens = self._tokenizer(texts).to(self.device)

            # 编码
            with torch.no_grad():
                features = self._model.encode_text(text_tokens)
                features = features / features.norm(dim=-1, keepdim=True)

            return features.cpu().numpy()

        except Exception as e:
            raise EncoderInferenceError(f"Batch text encoding failed: {e}") from e

    def compute_similarity(
        self,
        image_features: np.ndarray,
        text_features: np.ndarray
    ) -> float:
        """
        计算图像-文本相似度

        Args:
            image_features: 图像特征 (D,)
            text_features: 文本特征 (D,)

        Returns:
            相似度分数 [0, 1]
        """
        # 余弦相似度
        similarity = np.dot(image_features, text_features)
        # 归一化到[0, 1]
        similarity = (similarity + 1.0) / 2.0
        return float(similarity)

    def text_image_similarity(
        self,
        text: str,
        image_features_list: list[np.ndarray]
    ) -> list[float]:
        """
        计算文本与多个图像的相似度

        Args:
            text: 文本字符串
            image_features_list: 图像特征列表

        Returns:
            相似度列表
        """
        if not image_features_list:
            return []

        # 编码文本
        text_features = self.encode_text(text)

        # 计算相似度
        similarities = []
        for img_feat in image_features_list:
            sim = self.compute_similarity(img_feat, text_features)
            similarities.append(sim)

        return similarities

    def get_feature_dim(self) -> int:
        """
        获取特征维度

        Returns:
            特征向量维度
        """
        return self._feature_dim

    def get_model_info(self) -> dict:
        """
        获取模型信息

        Returns:
            模型信息字典
        """
        return {
            "name": "CLIP",
            "version": self.model_name,
            "feature_dim": self._feature_dim,
            "device": self.device,
            "cache_enabled": self.enable_cache,
            "cache_hit_rate": self.cache_hit_rate,
            "encode_count": self._encode_count,
            "avg_encode_time": (
                self._total_encode_time / self._encode_count
                if self._encode_count > 0 else 0.0
            ),
        }

    @property
    def cache_hit_rate(self) -> float:
        """缓存命中率"""
        total = self._cache_hits + self._cache_misses
        return self._cache_hits / total if total > 0 else 0.0

    def clear_cache(self):
        """清空缓存"""
        self._image_cache.clear()
        self._text_cache.clear()
        self._cache_hits = 0
        self._cache_misses = 0
        logger.info("Cache cleared")

    def _compute_image_hash(self, image: np.ndarray) -> str:
        """计算图像哈希（用于缓存键）"""
        return hashlib.md5(image.tobytes()).hexdigest()

    def _update_cache(self, cache: dict, key: str, value: np.ndarray):
        """更新缓存（LRU策略）"""
        if len(cache) >= self.cache_size:
            # 删除最旧的条目
            oldest_key = next(iter(cache))
            del cache[oldest_key]

        cache[key] = value

    def __del__(self):
        """清理资源"""
        if self._model is not None:
            del self._model
            self._model = None

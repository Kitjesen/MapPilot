"""
CLIP 特征编码器 — 图像/文本跨模态匹配（论文级实现）

升级说明（P0任务#2）:
- 原实现: 基础的CLIP编码功能
- 新实现: 多尺度特征提取、批处理优化、特征缓存、索引机制
- 参考: OpenCLIP论文 + LOVON 2024 + ConceptGraphs ICRA 2024

核心改进:
1. 多尺度特征提取（不同分辨率）
2. 特征缓存和索引机制（避免重复计算）
3. 批处理优化（提升吞吐量）
4. 文本-图像对齐优化
5. 集成到Fast Path的置信度融合

参考:
  - LOVON (2024): YOLO 检测 → 裁剪 → CLIP 编码 → 文本匹配
  - ConceptGraphs (ICRA 2024): CLIP 嵌入用于 3D 场景图节点
  - OpenCLIP: 多尺度特征和对比学习

依赖: pip install open-clip-torch
       显存占用: ViT-B/32 ~0.4GB, ViT-L/14 ~1.2GB
"""

import hashlib
import logging

import numpy as np

from runtime.utils.robustness import _try_empty_cuda_cache

logger = logging.getLogger(__name__)


class CLIPEncoder:
    """
    CLIP 图像/文本编码器（论文级实现）

    新增特性:
    - 多尺度特征提取
    - 特征缓存机制
    - 批处理优化
    - 性能监控
    """

    def __init__(
        self,
        model_name: str = "ViT-B/32",
        device: str = "auto",
        enable_cache: bool = True,
        cache_size: int = 1000,
        batch_size: int = 32,
        multi_scale: bool = False,
    ):
        """
        初始化CLIP编码器

        Args:
            model_name: CLIP模型名称
            device: 设备（"auto"/"cuda"/"cpu"）
            enable_cache: 是否启用特征缓存
            cache_size: 缓存大小
            batch_size: 批处理大小
            multi_scale: 是否使用多尺度特征
        """
        self._model_name = model_name
        if device == "auto":
            try:
                import torch
                device = "cuda" if torch.cuda.is_available() else "cpu"
            except ImportError:
                device = "cpu"
        self._device = device
        self._enable_cache = enable_cache
        self._cache_size = cache_size
        self._batch_size = batch_size
        self._multi_scale = multi_scale

        self._model = None
        self._preprocess = None
        self._tokenizer = None
        self._feature_dim: int = 0

        # Degraded state — True when GPU load failed and fell back to CPU.
        # Gateway health endpoint surfaces this so operators see degradation.
        self._degraded: bool = False
        self._degraded_reason: str = ""

        # 特征缓存（LRU）
        self._image_cache: dict[str, np.ndarray] = {}
        self._text_cache: dict[str, np.ndarray] = {}
        self._cache_hits = 0
        self._cache_misses = 0

        # 性能统计
        self._encode_count = 0
        self._total_encode_time = 0.0

    @property
    def feature_dim(self) -> int:
        """特征向量维度"""
        return self._feature_dim

    @property
    def cache_hit_rate(self) -> float:
        """缓存命中率"""
        total = self._cache_hits + self._cache_misses
        return self._cache_hits / total if total > 0 else 0.0

    @property
    def is_degraded(self) -> bool:
        """True if CLIP fell back to CPU (10-100x slower than GPU)."""
        return self._degraded

    @property
    def degraded_reason(self) -> str:
        """Human-readable reason why the encoder is running in degraded mode."""
        return self._degraded_reason

    def load_model(self) -> None:
        """加载 CLIP 模型"""
        try:
            import open_clip
            import torch

            # open_clip 的模型名需要转换: "ViT-B/32" → "ViT-B-32"
            clip_model_name = self._model_name.replace("/", "-")

            logger.info("Loading CLIP model: %s on %s", clip_model_name, self._device)

            self._model, _, self._preprocess = open_clip.create_model_and_transforms(
                clip_model_name,
                pretrained="openai",
                device=self._device,
            )
            self._tokenizer = open_clip.get_tokenizer(clip_model_name)
            self._model.eval()

            # 获取特征维度
            with torch.no_grad():
                dummy = torch.randn(1, 3, 224, 224).to(self._device)
                feat = self._model.encode_image(dummy)
                self._feature_dim = feat.shape[-1]

            logger.info(
                "CLIP loaded: %s, feature_dim=%d, device=%s, cache=%s, multi_scale=%s",
                clip_model_name, self._feature_dim, self._device,
                self._enable_cache, self._multi_scale
            )

        except ImportError:
            logger.error(
                "open-clip-torch not installed. Run: pip install open-clip-torch"
            )
            raise
        except RuntimeError as e:
            logger.error("CLIP GPU load failed: %s — falling back to CPU (10-100x slower)", e)
            self._degraded = True
            self._degraded_reason = f"GPU load failed: {e}"
            try:
                self._device = "cpu"
                self._model, _, self._preprocess = open_clip.create_model_and_transforms(
                    clip_model_name,
                    pretrained="openai",
                    device="cpu",
                )
                self._tokenizer = open_clip.get_tokenizer(clip_model_name)
                self._model.eval()
                with torch.no_grad():
                    dummy = torch.randn(1, 3, 224, 224)
                    feat = self._model.encode_image(dummy)
                    self._feature_dim = feat.shape[-1]
                logger.info(
                    "CLIP loaded on CPU fallback: feature_dim=%d", self._feature_dim
                )
            except Exception as e2:
                logger.error(f"CLIP CPU fallback also failed: {e2}")
                raise

    def _compute_image_hash(self, rgb: np.ndarray, bbox: np.ndarray) -> str:
        """
        计算图像区域的哈希值（用于缓存）

        Args:
            rgb: 图像
            bbox: 边界框

        Returns:
            哈希字符串
        """
        # 使用bbox和图像的部分像素计算哈希
        x1, y1, x2, y2 = map(int, bbox)
        crop = rgb[y1:y2:10, x1:x2:10]  # 降采样计算哈希
        hash_input = f"{x1}_{y1}_{x2}_{y2}_{crop.tobytes()}"
        return hashlib.md5(hash_input.encode()).hexdigest()

    def _manage_cache(self, cache: dict[str, np.ndarray], key: str, value: np.ndarray):
        """
        管理缓存（LRU策略）

        Args:
            cache: 缓存字典
            key: 键
            value: 值
        """
        if len(cache) >= self._cache_size:
            # 删除最旧的条目（简化版LRU）
            oldest_key = next(iter(cache))
            del cache[oldest_key]
        cache[key] = value

    def encode_image_crops(
        self,
        rgb: np.ndarray,
        bboxes: list[np.ndarray],
        use_cache: bool = True,
    ) -> list[np.ndarray]:
        """
        批量编码 bbox 裁剪区域的 CLIP 图像特征（优化版）

        改进:
        - 批处理优化
        - 特征缓存
        - 多尺度特征（可选）

        Args:
            rgb: HxWx3 uint8 BGR 图像
            bboxes: [[x1,y1,x2,y2], ...] 像素坐标列表
            use_cache: 是否使用缓存

        Returns:
            [feature_vector, ...] 归一化的 CLIP 特征列表
        """
        if self._model is None or len(bboxes) == 0:
            return [np.array([]) for _ in bboxes]

        if rgb.ndim != 3 or rgb.shape[2] != 3:
            logger.warning(f"encode_image_crops: invalid image shape {rgb.shape}, expected (H,W,3)")
            return [np.array([]) for _ in bboxes]

        import time

        import torch
        from PIL import Image

        start_time = time.time()

        # BGR → RGB
        rgb_image = rgb[:, :, ::-1].copy()
        h, w = rgb_image.shape[:2]

        # 准备裁剪和缓存检查
        crops = []
        cache_keys = []
        need_encode_indices = []
        results = [None] * len(bboxes)

        for i, bbox in enumerate(bboxes):
            x1 = max(0, int(bbox[0]))
            y1 = max(0, int(bbox[1]))
            x2 = min(w, int(bbox[2]))
            y2 = min(h, int(bbox[3]))

            if x2 <= x1 or y2 <= y1:
                results[i] = np.array([])
                continue

            # 检查缓存
            if use_cache and self._enable_cache:
                cache_key = self._compute_image_hash(rgb, bbox)
                cache_keys.append(cache_key)

                if cache_key in self._image_cache:
                    results[i] = self._image_cache[cache_key]
                    self._cache_hits += 1
                    continue
            else:
                cache_keys.append(None)

            self._cache_misses += 1

            crop = rgb_image[y1:y2, x1:x2]
            if crop.dtype.kind == 'f' and not np.isfinite(crop).all():
                logger.debug(f"Crop {i} contains NaN/inf, skipping")
                results[i] = np.array([])
                continue

            need_encode_indices.append(i)
            pil_crop = Image.fromarray(crop)

            # 多尺度特征（可选）
            if self._multi_scale:
                # 生成多个尺度的裁剪
                scales = [0.8, 1.0, 1.2]
                multi_crops = []
                for scale in scales:
                    scaled_crop = pil_crop.resize(
                        (int(pil_crop.width * scale), int(pil_crop.height * scale))
                    )
                    multi_crops.append(self._preprocess(scaled_crop))
                crops.append(multi_crops)
            else:
                crops.append(self._preprocess(pil_crop))

        # 批处理编码
        if need_encode_indices:
            try:
                if self._multi_scale:
                    # 多尺度特征编码
                    all_features = []
                    for scale_idx in range(len(scales)):
                        batch = torch.stack([crops[i][scale_idx] for i in range(len(crops))]).to(self._device)
                        with torch.no_grad():
                            features = self._model.encode_image(batch)
                            features = features / (features.norm(dim=-1, keepdim=True) + 1e-8)
                            all_features.append(features.cpu().numpy())

                    # 平均多尺度特征
                    features = np.mean(all_features, axis=0)
                else:
                    # 单尺度批处理
                    batch = torch.stack(crops).to(self._device)
                    with torch.no_grad():
                        features = self._model.encode_image(batch)
                        features = features / (features.norm(dim=-1, keepdim=True) + 1e-8)
                        features = features.cpu().numpy()
            except RuntimeError as e:
                logger.error(f"CLIP batch encoding failed (possible OOM): {e}")
                try:
                    del batch  # Free GPU tensor before cache cleanup
                except Exception:
                    pass
                _try_empty_cuda_cache()
                return [np.array([]) for _ in bboxes]

            # 填充结果和更新缓存
            for idx, need_idx in enumerate(need_encode_indices):
                results[need_idx] = features[idx]

                if use_cache and self._enable_cache and cache_keys[need_idx]:
                    self._manage_cache(
                        self._image_cache,
                        cache_keys[need_idx],
                        features[idx]
                    )

        # 填充空结果
        results = [r if r is not None else np.array([]) for r in results]

        # 性能统计
        elapsed = time.time() - start_time
        self._encode_count += len(bboxes)
        self._total_encode_time += elapsed

        return results

    def encode_text(
        self,
        texts: list[str],
        use_cache: bool = True,
    ) -> np.ndarray:
        """
        编码文本列表为 CLIP 特征（优化版）

        改进:
        - 批处理优化
        - 文本缓存

        Args:
            texts: 文本列表 ["red fire extinguisher", "a chair", ...]
            use_cache: 是否使用缓存

        Returns:
            (N, feature_dim) 归一化的特征矩阵
        """
        if self._model is None or not texts:
            return np.array([])

        import torch

        # Truncate early, before cache lookup, to ensure cache key consistency
        texts = [t[:200] for t in texts]

        # 检查缓存
        results = []
        need_encode_texts = []
        need_encode_indices = []

        for i, text in enumerate(texts):
            if use_cache and self._enable_cache:
                cache_key = hashlib.md5(text.encode()).hexdigest()
                if cache_key in self._text_cache:
                    results.append(self._text_cache[cache_key])
                    self._cache_hits += 1
                    continue

            self._cache_misses += 1
            need_encode_texts.append(text)
            need_encode_indices.append(i)
            results.append(None)

        # 批处理编码
        if need_encode_texts:
            try:
                tokens = self._tokenizer(need_encode_texts).to(self._device)

                with torch.no_grad():
                    features = self._model.encode_text(tokens)
                    features = features / (features.norm(dim=-1, keepdim=True) + 1e-8)
                    features = features.cpu().numpy()
            except RuntimeError as e:
                logger.error("CLIP text encoding failed (possible OOM): %s", e)
                _try_empty_cuda_cache()
                return np.array([])

            # 填充结果和更新缓存
            for idx, need_idx in enumerate(need_encode_indices):
                results[need_idx] = features[idx]

                if use_cache and self._enable_cache:
                    cache_key = hashlib.md5(need_encode_texts[idx].encode()).hexdigest()
                    self._manage_cache(self._text_cache, cache_key, features[idx])

        return np.array(results)

    def text_image_similarity(
        self,
        text_query: str,
        image_features: list[np.ndarray],
    ) -> list[float]:
        """
        计算文本查询与多个图像特征的相似度

        这是语义导航的核心匹配函数:
        用户说 "红色灭火器" → CLIP 文本编码 → 与场景中所有物体的
        CLIP 图像特征计算余弦相似度 → 排序

        Args:
            text_query: 查询文本
            image_features: 物体 CLIP 图像特征列表

        Returns:
            相似度列表 (0-1)
        """
        text_feat = self.encode_text([text_query])
        if text_feat.size == 0:
            return [0.0] * len(image_features)

        similarities = []
        for img_feat in image_features:
            if img_feat.size == 0:
                similarities.append(0.0)
            else:
                sim = float(np.dot(text_feat[0], img_feat))
                # Guard NaN from malformed features
                if not np.isfinite(sim):
                    sim = 0.0
                similarities.append(max(0.0, sim))  # CLIP 相似度可能为负

        return similarities

    def text_text_similarity(
        self,
        text_query: str,
        text_list: list[str],
    ) -> list[float] | None:
        """
        计算文本查询与多个文本的余弦相似度（D1 语义排序用）

        用于 goal_resolver._selective_grounding: 用户指令 vs 候选类别列表，
        按 CLIP 语义排序替代纯关键词匹配。

        Args:
            text_query: 查询文本（如用户指令）
            text_list: 候选文本列表（如 "chair", "red chair", "blue chair"）

        Returns:
            相似度列表 [0-1]，或 None 若模型未加载 / 输入为空
        """
        if self._model is None or not text_list:
            return None
        q_feat = self.encode_text([text_query])
        if q_feat.size == 0:
            return None
        k_feats = self.encode_text(text_list)
        if k_feats.size == 0:
            return [0.0] * len(text_list)
        similarities = [
            float(max(0.0, np.dot(q_feat[0], k_feats[i])))
            for i in range(len(text_list))
        ]
        return similarities

    def batch_text_image_similarity(
        self,
        text_queries: list[str],
        image_features: list[np.ndarray],
    ) -> np.ndarray:
        """
        批量计算文本查询与图像特征的相似度矩阵

        Args:
            text_queries: 文本查询列表
            image_features: 图像特征列表

        Returns:
            (len(text_queries), len(image_features)) 相似度矩阵
        """
        text_feats = self.encode_text(text_queries)
        if text_feats.size == 0:
            return np.zeros((len(text_queries), len(image_features)))

        similarities = np.zeros((len(text_queries), len(image_features)))

        for i, text_feat in enumerate(text_feats):
            for j, img_feat in enumerate(image_features):
                if img_feat.size > 0:
                    sim = float(np.dot(text_feat, img_feat))
                    similarities[i, j] = max(0.0, sim)

        return similarities

    def _encode_single_image(self, rgb_bgr: np.ndarray) -> np.ndarray:
        """编码单张完整图像（BGRorRGB→224x224→CLIP）。HOV-SG f_g 全局特征。"""
        if self._model is None:
            return np.array([])
        try:
            import torch
            from PIL import Image
            # BGR→RGB if needed (assume BGR input like cv2)
            rgb = rgb_bgr[:, :, ::-1].copy()
            pil_img = Image.fromarray(rgb)
            tensor = self._preprocess(pil_img).unsqueeze(0).to(self._device)
            with torch.no_grad():
                feat = self._model.encode_image(tensor)
                feat = feat / (feat.norm(dim=-1, keepdim=True) + 1e-8)
                return feat.cpu().numpy()[0]
        except RuntimeError as e:
            err_msg = str(e).lower()
            if "cuda" in err_msg or "out of memory" in err_msg:
                logger.error("_encode_single_image CUDA OOM: %s", e)
                _try_empty_cuda_cache()
            else:
                logger.debug("_encode_single_image failed: %s", e)
            return np.array([])
        except Exception as e:
            logger.debug("_encode_single_image failed: %s", e)
            return np.array([])

    def encode_image(self, image: np.ndarray) -> np.ndarray:
        """Encode a single image / object crop into one L2-normalised CLIP feature.

        Public alias over ``_encode_single_image``. This is the method that
        ``EncoderModule._on_image`` and ``PersonTracker`` Re-ID expect — without
        it those callers silently fail through their ``try/except`` guards.

        Contract: ``image`` is **BGR** (OpenCV default), matching
        ``encode_image_crops`` and the perception pipeline. Returns shape
        ``(D,)`` or an empty array when the model is unavailable.
        """
        return self._encode_single_image(image)

    def encode_three_source(
        self,
        rgb_frame: np.ndarray,
        bbox: np.ndarray,
        mask: np.ndarray | None = None,
        w_g: float = 0.25,
        w_l: float = 0.50,
        w_m: float = 0.25,
    ) -> np.ndarray:
        """
        HOV-SG 三源 CLIP 融合 (f_g + f_l + f_m)。

        f_g: 全帧特征（全局上下文，+41% mIOU 消融实验来源）
        f_l: 裁剪区域（含背景）— 最重要（w_l=0.50）
        f_m: 背景置零裁剪（去除显著背景干扰）

        Returns: 归一化融合特征向量，或空数组（模型未加载）。
        """
        if self._model is None:
            return np.array([])

        f_g = self._encode_single_image(rgb_frame)
        f_l = self.encode_image_crops(rgb_frame, [bbox])[0]

        if mask is not None:
            masked = rgb_frame.copy()
            masked[~mask] = 0
            f_m = self.encode_image_crops(masked, [bbox])[0]
        else:
            f_m = f_l  # fallback: 同 f_l

        # 处理空数组 — 有效分量相加
        parts = []
        weights_sum = 0.0
        if f_g.size > 0:
            parts.append(w_g * f_g)
            weights_sum += w_g
        if f_l.size > 0:
            parts.append(w_l * f_l)
            weights_sum += w_l
        if f_m.size > 0 and mask is not None:
            parts.append(w_m * f_m)
            weights_sum += w_m

        if not parts:
            return np.array([])

        fused = sum(parts)
        if weights_sum > 0:
            fused = fused / weights_sum

        norm = np.linalg.norm(fused)
        if not np.isfinite(norm) or norm < 1e-8:
            return np.array([])
        return fused / norm

    def get_statistics(self) -> dict[str, float]:
        """
        获取性能统计信息

        Returns:
            统计字典
        """
        avg_time = (self._total_encode_time / self._encode_count
                   if self._encode_count > 0 else 0.0)

        return {
            "encode_count": self._encode_count,
            "total_time": self._total_encode_time,
            "avg_time_per_encode": avg_time,
            "cache_hit_rate": self.cache_hit_rate,
            "cache_hits": self._cache_hits,
            "cache_misses": self._cache_misses,
            "image_cache_size": len(self._image_cache),
            "text_cache_size": len(self._text_cache),
        }

    def clear_cache(self):
        """清空缓存"""
        self._image_cache.clear()
        self._text_cache.clear()
        self._cache_hits = 0
        self._cache_misses = 0
        logger.info("CLIP cache cleared")

    def shutdown(self) -> None:
        """释放 GPU 资源"""
        self._model = None
        self.clear_cache()
        _try_empty_cuda_cache()
        logger.info("CLIP encoder shut down")

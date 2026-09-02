"""
CLIP编码器单元测试 (与实际 clip_encoder.py API 对齐)

测试内容:
- 初始化参数
- 缓存机制
- text_image_similarity 接口
- encode_image_crops 接口
- encode_text 接口
- 性能统计
- shutdown
"""

import unittest
from unittest.mock import Mock

import numpy as np
import pytest

from perception.encoding.clip_encoder import CLIPEncoder


class TestCLIPEncoderInit(unittest.TestCase):
    """CLIP编码器初始化测试"""

    def test_default_initialization(self):
        """默认参数初始化应该成功"""
        encoder = CLIPEncoder()
        self.assertEqual(encoder._model_name, "ViT-B/32")
        self.assertIn(encoder._device, ("cuda", "cpu"))  # cuda when GPU available, cpu otherwise
        self.assertTrue(encoder._enable_cache)
        self.assertEqual(encoder._cache_size, 1000)
        self.assertEqual(encoder._batch_size, 32)
        self.assertFalse(encoder._multi_scale)
        self.assertIsNone(encoder._model)

    def test_custom_parameters(self):
        """自定义参数应该正确设置"""
        encoder = CLIPEncoder(
            model_name="ViT-L/14",
            device="cpu",
            enable_cache=False,
            cache_size=500,
            batch_size=16,
            multi_scale=True,
        )
        self.assertEqual(encoder._model_name, "ViT-L/14")
        self.assertEqual(encoder._device, "cpu")
        self.assertFalse(encoder._enable_cache)
        self.assertEqual(encoder._cache_size, 500)
        self.assertEqual(encoder._batch_size, 16)
        self.assertTrue(encoder._multi_scale)

    def test_feature_dim_initially_zero(self):
        """特征维度初始化为0"""
        encoder = CLIPEncoder()
        self.assertEqual(encoder.feature_dim, 0)

    def test_cache_hit_rate_initially_zero(self):
        """缓存命中率初始为0"""
        encoder = CLIPEncoder()
        self.assertEqual(encoder.cache_hit_rate, 0.0)


class TestCLIPEncoderCache(unittest.TestCase):
    """特征缓存测试"""

    def test_manage_cache_eviction(self):
        """LRU缓存应该在满时淘汰旧条目"""
        encoder = CLIPEncoder(cache_size=2)
        cache = {}
        encoder._manage_cache(cache, "k1", np.array([1.0]))
        encoder._manage_cache(cache, "k2", np.array([2.0]))
        encoder._manage_cache(cache, "k3", np.array([3.0]))
        # 缓存大小不应超过 cache_size
        self.assertLessEqual(len(cache), 2)
        # 最新的 k3 应在缓存中
        self.assertIn("k3", cache)

    def test_clear_cache(self):
        """清空缓存"""
        encoder = CLIPEncoder(cache_size=10)
        encoder._image_cache["test"] = np.array([1.0])
        encoder._text_cache["test"] = np.array([1.0])
        encoder._cache_hits = 5
        encoder._cache_misses = 3

        encoder.clear_cache()

        self.assertEqual(len(encoder._image_cache), 0)
        self.assertEqual(len(encoder._text_cache), 0)
        self.assertEqual(encoder._cache_hits, 0)
        self.assertEqual(encoder._cache_misses, 0)


class TestCLIPEncoderTextImageSimilarity(unittest.TestCase):
    """text_image_similarity 接口测试"""

    def test_no_model_returns_zeros(self):
        """模型未加载时返回全零"""
        encoder = CLIPEncoder()
        feats = [np.random.randn(512).astype(np.float32) for _ in range(3)]
        result = encoder.text_image_similarity("hello", feats)
        self.assertEqual(len(result), 3)
        self.assertTrue(all(s == 0.0 for s in result))

    def test_empty_features_return_zero(self):
        """空特征向量返回0"""
        encoder = CLIPEncoder()
        result = encoder.text_image_similarity("hello", [np.array([])])
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0], 0.0)

    def test_similarity_with_mock_model(self):
        """使用 mock 模型验证相似度计算"""
        torch = pytest.importorskip("torch", reason="torch not installed")
        encoder = CLIPEncoder()
        # 模拟已加载的模型
        text_feat = np.random.randn(512).astype(np.float32)
        text_feat = text_feat / np.linalg.norm(text_feat)

        # Mock encode_text 返回归一化的特征
        encoder._model = Mock()
        encoder._tokenizer = Mock()

        import torch
        mock_tensor = torch.tensor(text_feat[np.newaxis, :])
        mock_norm = mock_tensor / mock_tensor.norm(dim=-1, keepdim=True)
        encoder._model.encode_text = Mock(return_value=mock_norm)

        img_feat = text_feat.copy()  # 相同特征 → 高相似度
        result = encoder.text_image_similarity("test", [img_feat])

        self.assertEqual(len(result), 1)
        self.assertGreater(result[0], 0.9)


class TestCLIPEncoderEncodeImageCrops(unittest.TestCase):
    """encode_image_crops 接口测试"""

    def test_no_model_returns_empty_arrays(self):
        """模型未加载时返回空数组"""
        encoder = CLIPEncoder()
        rgb = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        bboxes = [np.array([100, 100, 200, 200])]
        result = encoder.encode_image_crops(rgb, bboxes)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].size, 0)

    def test_empty_bboxes_returns_empty_list(self):
        """空 bbox 列表返回空"""
        encoder = CLIPEncoder()
        rgb = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        result = encoder.encode_image_crops(rgb, [])
        self.assertEqual(len(result), 0)

    def test_invalid_bbox_returns_empty_array(self):
        """无效 bbox (x2<=x1) 返回空数组"""
        pytest.importorskip("torch", reason="torch not installed")
        encoder = CLIPEncoder()
        encoder._model = Mock()  # 需要模型存在
        rgb = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        bboxes = [np.array([200, 200, 100, 100])]  # 无效: x2 < x1
        result = encoder.encode_image_crops(rgb, bboxes)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].size, 0)


class TestCLIPEncoderEncodeText(unittest.TestCase):
    """encode_text 接口测试"""

    def test_no_model_returns_empty(self):
        """模型未加载时返回空数组"""
        encoder = CLIPEncoder()
        result = encoder.encode_text(["hello"])
        self.assertEqual(result.size, 0)

    def test_empty_texts_returns_empty(self):
        """空文本列表返回空数组"""
        encoder = CLIPEncoder()
        result = encoder.encode_text([])
        self.assertEqual(result.size, 0)


class TestCLIPEncoderStatistics(unittest.TestCase):
    """性能统计测试"""

    def test_get_statistics_returns_all_keys(self):
        """统计接口应返回所有需要的键"""
        encoder = CLIPEncoder()
        stats = encoder.get_statistics()

        expected_keys = [
            "encode_count", "total_time", "avg_time_per_encode",
            "cache_hit_rate", "cache_hits", "cache_misses",
            "image_cache_size", "text_cache_size",
        ]
        for key in expected_keys:
            self.assertIn(key, stats, f"Missing key: {key}")

    def test_statistics_after_cache_ops(self):
        """缓存操作后统计应正确更新"""
        encoder = CLIPEncoder()
        encoder._cache_hits = 10
        encoder._cache_misses = 5
        stats = encoder.get_statistics()
        self.assertEqual(stats["cache_hits"], 10)
        self.assertEqual(stats["cache_misses"], 5)
        self.assertAlmostEqual(stats["cache_hit_rate"], 10 / 15)


class TestCLIPEncoderShutdown(unittest.TestCase):
    """生命周期测试"""

    def test_shutdown_clears_model(self):
        """shutdown 应清除模型和缓存"""
        encoder = CLIPEncoder()
        encoder._model = Mock()
        encoder._image_cache["test"] = np.array([1.0])

        encoder.shutdown()

        self.assertIsNone(encoder._model)
        self.assertEqual(len(encoder._image_cache), 0)


class TestCLIPEncoderBatchSimilarity(unittest.TestCase):
    """batch_text_image_similarity 测试"""

    def test_no_model_returns_zeros(self):
        """模型未加载时返回零矩阵"""
        encoder = CLIPEncoder()
        result = encoder.batch_text_image_similarity(
            ["hello", "world"],
            [np.random.randn(512).astype(np.float32)]
        )
        self.assertEqual(result.shape, (2, 1))
        self.assertTrue(np.allclose(result, 0.0))


if __name__ == '__main__':
    unittest.main()

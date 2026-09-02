"""Focused tests for VectorMemoryModule — CLIP embedding + vector search.

Pure unit tests -- no ROS2, no hardware, no external services.
Follows patterns from test_memory_modules.py.
"""

from __future__ import annotations

import json
import sys
import types
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from runtime.msgs.geometry import Vector3
from runtime.msgs.nav import Odometry, Pose
from runtime.msgs.semantic import Detection3D, SceneGraph

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _odom(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Odometry:
    return Odometry(pose=Pose(x, y, z))


def _scene_graph(labels: list[str]) -> SceneGraph:
    objects = [
        Detection3D(id=f"obj_{i}", label=lbl, confidence=0.9,
                     position=Vector3(float(i), 0.0, 0.0))
        for i, lbl in enumerate(labels)
    ]
    return SceneGraph(objects=objects, relations=[], regions=[])


# ===========================================================================
# 1. Instantiation & Ports
# ===========================================================================

class TestVectorMemoryInstantiation(unittest.TestCase):
    """Test VectorMemoryModule creation and configuration."""

    def _make(self, **kw):
        from memory.modules.vector_memory_module import VectorMemoryModule
        return VectorMemoryModule(**kw)

    def test_default_persist_dir(self):
        m = self._make()
        self.assertTrue(m._persist_dir.endswith("vector_memory"))

    def test_custom_persist_dir(self):
        m = self._make(persist_dir="/tmp/vec_test")
        self.assertEqual(m._persist_dir, "/tmp/vec_test")

    def test_custom_collection_name(self):
        m = self._make(collection_name="my_memory")
        self.assertEqual(m._collection_name, "my_memory")

    def test_default_store_interval(self):
        m = self._make()
        self.assertAlmostEqual(m._store_interval, 5.0)

    def test_custom_store_interval(self):
        m = self._make(store_interval=2.0)
        self.assertAlmostEqual(m._store_interval, 2.0)

    def test_default_max_results(self):
        m = self._make()
        self.assertEqual(m._max_results, 5)

    def test_custom_max_results(self):
        m = self._make(max_results=10)
        self.assertEqual(m._max_results, 10)

    def test_initial_store_count_zero(self):
        m = self._make()
        self.assertEqual(m._store_count, 0)

    def test_initial_use_chromadb_false(self):
        m = self._make()
        self.assertFalse(m._use_chromadb)

    def test_initial_encoder_type_none(self):
        m = self._make()
        self.assertEqual(m._encoder_type, "none")

    def test_custom_max_np_entries(self):
        m = self._make(max_np_entries=100)
        self.assertEqual(m._max_np_entries, 100)


class TestVectorMemoryPorts(unittest.TestCase):
    """Test that In/Out ports exist with correct types."""

    def _make(self, **kw):
        from memory.modules.vector_memory_module import VectorMemoryModule
        return VectorMemoryModule(**kw)

    def test_input_ports_exist(self):
        from runtime.stream import In
        m = self._make()
        self.assertIsInstance(m.scene_graph, In)
        self.assertIsInstance(m.odometry, In)
        self.assertIsInstance(m.image, In)

    def test_output_ports_exist(self):
        from runtime.stream import Out
        m = self._make()
        self.assertIsInstance(m.query_result, Out)

    def test_port_names(self):
        m = self._make()
        self.assertEqual(m.scene_graph.name, "scene_graph")
        self.assertEqual(m.odometry.name, "odometry")
        self.assertEqual(m.image.name, "image")
        self.assertEqual(m.query_result.name, "query_result")


# ===========================================================================
# 2. State & Helpers
# ===========================================================================

class TestVectorMemoryState(unittest.TestCase):
    """Test state tracking and helper methods."""

    def _make(self, **kw):
        from memory.modules.vector_memory_module import VectorMemoryModule
        return VectorMemoryModule(**kw)

    def test_on_odom_updates_position(self):
        m = self._make()
        m._on_odom(_odom(7.0, 8.0, 1.25))
        self.assertEqual(m._robot_xy, (7.0, 8.0))
        self.assertEqual(m._robot_z, 1.25)

    def test_on_image_stores_latest(self):
        m = self._make()
        img = np.zeros((100, 100, 3), dtype=np.uint8)
        m._on_image(img)
        self.assertIs(m._latest_image, img)

    def test_on_scene_graph_without_labels_skips(self):
        m = self._make()
        sg = SceneGraph(objects=[], relations=[], regions=[])
        m._latest_labels = []
        m._on_scene_graph(sg)
        self.assertEqual(m._latest_labels, [])

    def test_on_scene_graph_stores_labels(self):
        m = self._make()
        sg = _scene_graph(["chair", "table"])
        m._on_scene_graph(sg)
        self.assertEqual(m._latest_labels, ["chair", "table"])

    def test_hash_embedding_deterministic(self):
        from memory.modules.vector_memory_module import VectorMemoryModule
        v1 = VectorMemoryModule._hash_embedding("hello world")
        v2 = VectorMemoryModule._hash_embedding("hello world")
        np.testing.assert_array_equal(v1, v2)

    def test_hash_embedding_normalized(self):
        from memory.modules.vector_memory_module import VectorMemoryModule
        v = VectorMemoryModule._hash_embedding("backpack bench tree")
        norm = float(np.linalg.norm(v))
        self.assertAlmostEqual(norm, 1.0, places=5)

    def test_hash_embedding_stop_words_filtered(self):
        from memory.modules.vector_memory_module import VectorMemoryModule
        v1 = VectorMemoryModule._hash_embedding("find the backpack")
        v2 = VectorMemoryModule._hash_embedding("backpack")
        np.testing.assert_array_equal(v1, v2)

    def test_semantic_encoder_ready_true_modes(self):
        m = self._make()
        for mode in ("mobileclip", "clip", "sentence_transformers"):
            m._encoder_type = mode
            self.assertTrue(m._semantic_encoder_ready())

    def test_semantic_encoder_ready_false_modes(self):
        m = self._make()
        for mode in ("none", "lexical_hash"):
            m._encoder_type = mode
            self.assertFalse(m._semantic_encoder_ready())


class TestVectorMemoryEncoderSelection(unittest.TestCase):
    def _module(self):
        from memory.modules.vector_memory_module import VectorMemoryModule

        return VectorMemoryModule()

    @staticmethod
    def _provider(*, load_error=None, dimensions=512):
        encoder = MagicMock()
        if load_error is not None:
            encoder.load_model.side_effect = load_error
        encoder.encode_text.return_value = np.ones((1, dimensions), dtype=np.float32)
        provider = MagicMock()
        provider.create.return_value = encoder
        return provider, encoder

    def test_mobileclip_is_loaded_before_use(self):
        module = self._module()
        provider, encoder = self._provider()

        with (
            patch("runtime.plugin_seed.seed_registered_plugins"),
            patch("memory.modules.vector_memory_module.get", return_value=provider),
        ):
            module._init_encoder()

        encoder.load_model.assert_called_once()
        self.assertEqual(module._encoder_type, "mobileclip")
        self.assertIs(module._encoder, encoder)

    def test_clip_fallback_loads_when_mobileclip_fails(self):
        module = self._module()
        mobileclip, _ = self._provider(load_error=RuntimeError("unavailable"))
        clip, clip_encoder = self._provider()

        with (
            patch("runtime.plugin_seed.seed_registered_plugins"),
            patch(
                "memory.modules.vector_memory_module.get",
                side_effect=lambda _kind, backend: {"mobileclip": mobileclip, "clip": clip}[backend],
            ),
        ):
            module._init_encoder()

        clip_encoder.load_model.assert_called_once()
        self.assertEqual(module._encoder_type, "clip")

    def test_sentence_transformer_fallback_encodes_unit_vector(self):
        module = self._module()
        sentence_transformers = types.ModuleType("sentence_transformers")

        class SentenceTransformer:
            def __init__(self, _model_name):
                pass

            def encode(self, _text, normalize_embeddings=False):
                vector = np.ones(384, dtype=np.float32)
                return vector / np.linalg.norm(vector)

        sentence_transformers.SentenceTransformer = SentenceTransformer
        with (
            patch("runtime.plugin_seed.seed_registered_plugins"),
            patch("memory.modules.vector_memory_module.get", side_effect=KeyError),
            patch.dict(sys.modules, {"sentence_transformers": sentence_transformers}),
        ):
            module._init_encoder()

        vector = module._encode_text("backpack near the entrance")
        self.assertEqual(module._encoder_type, "sentence_transformers")
        self.assertEqual(vector.shape, (384,))
        self.assertEqual(vector.dtype, np.float32)
        self.assertAlmostEqual(float(np.linalg.norm(vector)), 1.0, places=5)
        self.assertEqual(module.health()["encoder_type"], "sentence_transformers")

    def test_missing_encoders_use_lexical_hash(self):
        module = self._module()
        real_import = __import__

        def block_sentence_transformers(name, *args, **kwargs):
            if name == "sentence_transformers":
                raise ImportError("blocked for test")
            return real_import(name, *args, **kwargs)

        with (
            patch("runtime.plugin_seed.seed_registered_plugins"),
            patch("memory.modules.vector_memory_module.get", side_effect=KeyError),
            patch("builtins.__import__", side_effect=block_sentence_transformers),
        ):
            module._init_encoder()

        self.assertEqual(module._encoder_type, "lexical_hash")


# ===========================================================================
# 3. Numpy Fallback Store and Query
# ===========================================================================

class TestVectorMemoryNumpyStore(unittest.TestCase):
    """Test numpy fallback store and query operations."""

    def _make(self, **kw):
        from memory.modules.vector_memory_module import VectorMemoryModule
        m = VectorMemoryModule(**kw)
        m._use_chromadb = False
        m._encoder = None
        return m

    def test_store_snapshot_creates_entry(self):
        m = self._make()
        m._robot_xy = (1.0, 2.0)
        m._robot_z = 1.5
        m._store_snapshot(["chair", "table"])
        self.assertEqual(len(m._np_embeddings), 1)
        self.assertEqual(m._np_metadata[0]["x"], 1.0)
        self.assertEqual(m._store_count, 1)

    def test_store_snapshot_stores_all_labels(self):
        m = self._make()
        m._robot_xy = (3.0, 4.0)
        m._store_snapshot(["backpack", "bench", "tree"])
        self.assertIn("backpack", m._np_metadata[0]["labels"])
        self.assertIn("bench", m._np_metadata[0]["labels"])

    def test_numpy_query_returns_results(self):
        m = self._make()
        m._robot_xy = (5.0, 10.0)
        m._store_snapshot(["backpack", "bench"])
        results = m._query("backpack")
        self.assertGreater(len(results), 0)
        self.assertAlmostEqual(results[0]["x"], 5.0)

    def test_numpy_query_empty_store(self):
        m = self._make()
        results = m._query("anything")
        self.assertEqual(results, [])

    def test_lru_eviction(self):
        m = self._make(max_np_entries=3)
        for i in range(5):
            m._robot_xy = (float(i), 0.0)
            m._store_snapshot([f"label_{i}"])
        self.assertEqual(len(m._np_embeddings), 3)
        # Oldest (label_0, label_1) should be evicted; first entry should be label_2
        self.assertAlmostEqual(m._np_metadata[0]["x"], 2.0)

    def test_numpy_query_skips_mismatched_dimensions(self):
        m = self._make()
        m._encoder_type = "lexical_hash"
        m._embedding_dim = 512
        m._np_embeddings.append(np.ones(4, dtype=np.float32))
        m._np_metadata.append({
            "x": 1.0, "y": 2.0, "labels": "bad-dim",
            "encoder_type": "lexical_hash", "embedding_dim": 4,
            "semantic_encoder_ready": False, "degraded": True, "navigable": False,
        })
        results = m._query("bad-dim")
        self.assertEqual(results, [])


# ===========================================================================
# 4. Skill Methods
# ===========================================================================

class TestVectorMemorySkills(unittest.TestCase):
    """Test @skill-exposed methods."""

    def _make(self, **kw):
        from memory.modules.vector_memory_module import VectorMemoryModule
        m = VectorMemoryModule(**kw)
        m._use_chromadb = False
        m._encoder = None
        return m

    def test_query_location_not_found(self):
        m = self._make()
        result = json.loads(m.query_location("missing thing"))
        self.assertFalse(result["found"])
        self.assertEqual(result["encoder_type"], "none")

    def test_query_location_found(self):
        m = self._make()
        m._robot_xy = (3.0, 4.0)
        m._robot_z = 1.25
        m._store_snapshot(["backpack", "tree"])
        result = json.loads(m.query_location("backpack"))
        self.assertTrue(result["found"])
        self.assertIn("best", result)
        self.assertEqual(result["best"]["z"], 1.25)

    def test_query_location_returns_results_list(self):
        m = self._make()
        m._robot_xy = (0.0, 0.0)
        m._store_snapshot(["bottle"])
        result = json.loads(m.query_location("bottle"))
        self.assertTrue(result["found"])
        self.assertGreaterEqual(len(result["results"]), 1)

    def test_get_memory_stats_numpy(self):
        m = self._make()
        m._robot_xy = (0.0, 0.0)
        m._store_snapshot(["item"])
        stats = json.loads(m.get_memory_stats())
        self.assertEqual(stats["backend"], "numpy")
        self.assertEqual(stats["entries"], 1)

    def test_get_memory_stats_empty(self):
        m = self._make()
        stats = json.loads(m.get_memory_stats())
        self.assertEqual(stats["entries"], 0)

    def test_health_numpy_backend(self):
        m = self._make()
        m._robot_xy = (0.0, 0.0)
        m._store_snapshot(["item"])
        h = m.health()
        self.assertEqual(h["backend"], "numpy")
        self.assertEqual(h["entry_count"], 1)


if __name__ == "__main__":
    unittest.main()

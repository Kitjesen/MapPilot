"""Wave 3 Team E — simplification remediation tests.

W3-1: VectorMemoryModule encoder fallback → sentence-transformers or lexical hash.
W3-6: SemanticMapperModule Bayesian Dirichlet-Multinomial observation gate.

All tests are self-contained: heavy dependencies (sentence-transformers, CLIP,
ChromaDB, ROS2) are mocked so the suite runs on any dev machine without extras.
"""

from __future__ import annotations

import sys
import types
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

# ---------------------------------------------------------------------------
# Helper: build a minimal fake sentence-transformers package
# ---------------------------------------------------------------------------

def _make_fake_st_class(encode_fn=None):
    """Return a fake SentenceTransformer class that accepts a model name arg."""

    class FakeST:
        def __init__(self, model_name=None):
            self._model_name = model_name

        def encode(self, text, normalize_embeddings=False):
            if encode_fn:
                return encode_fn(text)
            # 384-dim unit vector by default
            vec = np.ones(384, dtype=np.float32)
            vec /= np.linalg.norm(vec)
            return vec

    return FakeST


def _make_st_module(encode_fn=None):
    """Return a fake sentence_transformers module with SentenceTransformer."""
    st_mod = types.ModuleType("sentence_transformers")
    st_mod.SentenceTransformer = _make_fake_st_class(encode_fn)
    return st_mod


# ---------------------------------------------------------------------------
# Import-blocking helpers for _init_encoder tests
# ---------------------------------------------------------------------------

_REAL_IMPORT = __builtins__.__import__ if hasattr(__builtins__, "__import__") else __import__


def _block_clip_import(name, *args, **kwargs):
    """Block the CLIP encoder import path; allow everything else."""
    if name.startswith("perception"):
        raise ImportError(f"blocked for test: {name}")
    return _REAL_IMPORT(name, *args, **kwargs)


def _block_all_encoders_import(name, *args, **kwargs):
    """Block both CLIP and sentence-transformers imports."""
    if name.startswith("perception"):
        raise ImportError(f"blocked for test: {name}")
    if name == "sentence_transformers":
        raise ImportError("blocked for test: sentence_transformers")
    return _REAL_IMPORT(name, *args, **kwargs)


def _make_mobileclip_module(fail_load: bool = False):
    mod = types.ModuleType("perception.mobileclip_encoder")
    mod.instances = []

    class FakeMobileCLIPEncoder:
        def __init__(self, device="auto"):
            self.device = device
            self.loaded = False
            mod.instances.append(self)

        def load_model(self):
            if fail_load:
                raise RuntimeError("mobileclip unavailable")
            self.loaded = True

        def encode_text(self, texts):
            if not self.loaded:
                return np.array([])
            vec = np.ones((len(texts), 512), dtype=np.float32)
            vec /= np.linalg.norm(vec, axis=1, keepdims=True)
            return vec

    mod.MobileCLIPEncoder = FakeMobileCLIPEncoder
    return mod


def _make_clip_module():
    mod = types.ModuleType("perception.clip_encoder")
    mod.instances = []

    class FakeCLIPEncoder:
        def __init__(self, model_name="ViT-B/32", device="auto", **_kw):
            self.model_name = model_name
            self.device = device
            self.loaded = False
            mod.instances.append(self)

        def load_model(self):
            self.loaded = True

        def encode_text(self, texts):
            if not self.loaded:
                return np.array([])
            vec = np.ones((len(texts), 512), dtype=np.float32)
            vec /= np.linalg.norm(vec, axis=1, keepdims=True)
            return vec

    mod.CLIPEncoder = FakeCLIPEncoder
    return mod


class TestVectorMemorySemanticEncoderSelection(unittest.TestCase):
    """VectorMemory should load real encoder weights before smoke testing."""

    def _fresh_vmm_class(self):
        sys.modules.pop("memory.modules.vector_memory_module", None)
        from memory.modules.vector_memory_module import VectorMemoryModule
        return VectorMemoryModule

    def test_mobileclip_selected_after_load_model(self):
        VectorMemoryModule = self._fresh_vmm_class()
        mod = VectorMemoryModule()
        fake_mobileclip = _make_mobileclip_module()

        with patch.dict(
            sys.modules,
            {
                "perception.mobileclip_encoder": fake_mobileclip,
            },
        ):
            mod._init_encoder()

        self.assertEqual(mod._encoder_type, "mobileclip")
        self.assertTrue(fake_mobileclip.instances[-1].loaded)
        self.assertIs(mod._encoder, fake_mobileclip.instances[-1])

    def test_clip_fallback_loads_model_when_mobileclip_fails(self):
        VectorMemoryModule = self._fresh_vmm_class()
        mod = VectorMemoryModule()
        fake_mobileclip = _make_mobileclip_module(fail_load=True)
        fake_clip = _make_clip_module()

        with patch.dict(
            sys.modules,
            {
                "perception.mobileclip_encoder": fake_mobileclip,
                "perception.clip_encoder": fake_clip,
            },
        ):
            mod._init_encoder()

        self.assertEqual(mod._encoder_type, "clip")
        self.assertTrue(fake_clip.instances[-1].loaded)
        self.assertEqual(fake_clip.instances[-1].model_name, "ViT-B/32")


# ---------------------------------------------------------------------------
# W3-1 tests
# ---------------------------------------------------------------------------

try:
    import sentence_transformers as _  # noqa: F401
    _HAVE_ST = True
except Exception:
    _HAVE_ST = False


@unittest.skipUnless(
    _HAVE_ST,
    "sentence_transformers not installed in CI — skip encoder selection tests "
    "that rely on patching the live package.")
class TestVectorMemoryEncoderType(unittest.TestCase):
    """W3-1: _encoder_type attribute and encoder selection logic."""

    def _fresh_vmm_class(self):
        """Import VectorMemoryModule fresh (drop cached copy)."""
        sys.modules.pop("memory.modules.vector_memory_module", None)
        from memory.modules.vector_memory_module import VectorMemoryModule
        return VectorMemoryModule

    def _make_mod_with_st(self, fake_st_class=None):
        """Return a VectorMemoryModule instance whose _init_encoder will find
        sentence-transformers (CLIP absent).  We patch SentenceTransformer on
        the real installed package so no torch internals are exercised."""
        if fake_st_class is None:
            fake_st_class = _make_fake_st_class()
        VectorMemoryModule = self._fresh_vmm_class()
        mod = VectorMemoryModule.__new__(VectorMemoryModule)
        VectorMemoryModule.__init__(mod)
        # Block CLIP, inject our fake ST class.
        with patch("builtins.__import__", side_effect=_block_clip_import):
            with patch("sentence_transformers.SentenceTransformer", fake_st_class):
                mod._init_encoder()
        return mod

    def test_encoder_type_is_none_before_init(self):
        """_encoder_type defaults to 'none' before _init_encoder is called."""
        VectorMemoryModule = self._fresh_vmm_class()
        mod = VectorMemoryModule.__new__(VectorMemoryModule)
        VectorMemoryModule.__init__(mod)
        self.assertEqual(mod._encoder_type, "none")

    def test_st_encoder_selected_when_clip_absent(self):
        """When CLIP is unavailable but sentence-transformers is present, encoder_type = 'sentence_transformers'."""
        mod = self._make_mod_with_st()
        self.assertEqual(mod._encoder_type, "sentence_transformers")
        self.assertIsNotNone(mod._st_model)

    def test_graceful_disable_when_both_encoders_missing(self):
        """When neither CLIP nor sentence-transformers is importable,
        _init_encoder marks the module as 'lexical_hash' (no hard exception)
        so the rest of the system can still start and exact-label queries
        remain available while semantic search is reported as degraded."""
        VectorMemoryModule = self._fresh_vmm_class()
        mod = VectorMemoryModule.__new__(VectorMemoryModule)
        VectorMemoryModule.__init__(mod)

        # Temporarily evict sentence_transformers from sys.modules so the
        # 'from sentence_transformers import ...' inside _init_encoder actually
        # triggers the ImportError path rather than hitting the cached module.
        saved_st = sys.modules.pop("sentence_transformers", None)
        try:
            with patch("builtins.__import__", side_effect=_block_all_encoders_import):
                # Should NOT raise — graceful disable instead.
                mod._init_encoder()
            self.assertEqual(mod._encoder_type, "lexical_hash")
        finally:
            if saved_st is not None:
                sys.modules["sentence_transformers"] = saved_st

    def test_hash_embedding_not_used_by_live_encoder_path(self):
        """_hash_embedding is NOT called when a real encoder is active.

        The live _encode_text path (CLIP or sentence-transformers) returns a
        semantically meaningful vector without touching _hash_embedding.
        _hash_embedding is reached only by the explicit degraded fallback or
        when setup() is bypassed in unit tests.
        """
        mod = self._make_mod_with_st()
        # _encoder_type must be 'sentence_transformers', not 'none'
        self.assertNotEqual(mod._encoder_type, "none",
                            "_encoder_type must not be 'none' after _init_encoder")
        # Verify encode goes through ST path, not hash path
        vec = mod._encode_text("backpack")
        self.assertIsNotNone(vec)
        # ST produces 384-dim; hash produces 512-dim — confirm we got ST output
        self.assertEqual(vec.shape[0], 384,
                         "Expected 384-dim ST output, got hash-based 512-dim output")

    def test_encode_text_returns_unit_vector(self):
        """_encode_text returns a float32 ndarray normalized to unit L2."""
        mod = self._make_mod_with_st()
        vec = mod._encode_text("backpack near the entrance")
        self.assertIsNotNone(vec)
        self.assertEqual(vec.dtype, np.float32)
        norm = float(np.linalg.norm(vec))
        self.assertAlmostEqual(norm, 1.0, places=5)

    def test_health_exposes_encoder_type(self):
        """health() dict must include 'encoder_type' key."""
        mod = self._make_mod_with_st()
        # health() calls super().port_summary() — stub it.
        mod.port_summary = lambda: {}
        h = mod.health()
        self.assertIn("encoder_type", h)
        self.assertEqual(h["encoder_type"], "sentence_transformers")


# ---------------------------------------------------------------------------
# W3-6 tests
# ---------------------------------------------------------------------------

class TestSemanticMapperObservationGate(unittest.TestCase):
    """W3-6: Bayesian Dirichlet-Multinomial gate in SemanticMapperModule."""

    def _make_mapper(self, min_obs: int = 3):
        """Instantiate SemanticMapperModule without Blueprint or ROS2."""
        sys.modules.pop("memory.modules.semantic_mapper_module", None)

        kg_mock = MagicMock()
        kg_mock.is_empty = False

        with patch.dict(sys.modules, {
            "memory.knowledge.room_object_kg": MagicMock(RoomObjectKG=MagicMock(return_value=kg_mock)),
            "memory.spatial.topology_graph": MagicMock(TopologySemGraph=MagicMock(return_value=MagicMock())),
        }):
            from memory.modules.semantic_mapper_module import SemanticMapperModule

        mapper = SemanticMapperModule.__new__(SemanticMapperModule)
        SemanticMapperModule.__init__(mapper, min_observations_for_commit=min_obs)
        mapper._kg = kg_mock
        mapper._tsg = None
        return mapper, kg_mock

    def _make_sg(self, room: str, labels: list[str]):
        """Build a minimal fake SceneGraph with one region."""
        obj_list = []
        for i, lbl in enumerate(labels):
            obj = MagicMock()
            obj.label = lbl
            obj.confidence = 0.9
            obj.id = str(i)
            obj_list.append(obj)

        region = MagicMock()
        region.name = room
        region.object_ids = [str(i) for i in range(len(labels))]
        region.center = None

        sg = MagicMock()
        sg.regions = [region]
        sg.objects = obj_list
        sg.get_object_by_id = lambda oid: next(
            (o for o in obj_list if o.id == oid), None
        )
        return sg

    def test_single_observation_does_not_commit(self):
        """A single observation must NOT call kg.observe_room (below threshold)."""
        mapper, kg_mock = self._make_mapper(min_obs=3)
        sg = self._make_sg("kitchen", ["person", "counter"])
        mapper._update_kg(sg)
        kg_mock.observe_room.assert_not_called()

    def test_commits_after_reaching_threshold(self):
        """After exactly min_obs observations, kg.observe_room is called."""
        mapper, kg_mock = self._make_mapper(min_obs=3)
        sg = self._make_sg("kitchen", ["microwave"])
        for _ in range(3):
            mapper._update_kg(sg)
        # observe_room should have been called at least once (on the 3rd update)
        kg_mock.observe_room.assert_called()

    def test_below_threshold_stays_silent(self):
        """Two observations (< min_obs=3) produce zero KG writes."""
        mapper, kg_mock = self._make_mapper(min_obs=3)
        sg = self._make_sg("office", ["desk", "chair"])
        mapper._update_kg(sg)
        mapper._update_kg(sg)
        kg_mock.observe_room.assert_not_called()

    def test_get_posterior_laplace_smoothing(self):
        """get_posterior returns Dirichlet-Multinomial posterior with alpha=1.0."""
        mapper, _ = self._make_mapper(min_obs=3)
        # Manually inject DM counts: kitchen has {microwave:4, sink:2}
        mapper._dm_counts["kitchen"] = {"microwave": 4, "sink": 2}
        # P(microwave|kitchen) = (4+1)/(6+1*2) = 5/8 = 0.625
        p = mapper.get_posterior("kitchen", "microwave")
        self.assertAlmostEqual(p, 5.0 / 8.0, places=5)

    def test_get_posterior_unseen_room(self):
        """get_posterior returns 0.0 for a room with no observations."""
        mapper, _ = self._make_mapper()
        self.assertEqual(mapper.get_posterior("unknown_room", "chair"), 0.0)

    def test_obs_counts_accumulated_correctly(self):
        """_obs_counts increments on every update, not just after threshold."""
        mapper, _ = self._make_mapper(min_obs=5)
        sg = self._make_sg("lab", ["robot_arm"])
        for _ in range(4):
            mapper._update_kg(sg)
        self.assertEqual(mapper._obs_counts.get(("lab", "robot_arm"), 0), 4)

    def test_dm_counts_always_updated(self):
        """DM counts accumulate unconditionally (before and after gate threshold)."""
        mapper, _ = self._make_mapper(min_obs=10)
        sg = self._make_sg("corridor", ["sign", "door"])
        mapper._update_kg(sg)
        mapper._update_kg(sg)
        self.assertEqual(mapper._dm_counts.get("corridor", {}).get("sign", 0), 2)


if __name__ == "__main__":
    unittest.main()

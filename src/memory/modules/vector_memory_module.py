"""VectorMemoryModule — CLIP embedding + ChromaDB vector search for fuzzy spatial queries.

Stores scene snapshots as CLIP embeddings associated with robot position.
Supports queries like "去上次放背包的地方" → vector search → return position.

Pipeline:
  scene_graph + odometry + image → CLIP encode scene labels → ChromaDB upsert
  query text → CLIP encode → ChromaDB search → top-k (position, score, labels)

ChromaDB runs in-process (persistent mode, no server needed).
Falls back to brute-force numpy search if chromadb not installed.

Ports:
  In:  scene_graph (SceneGraph), odometry (Odometry), image (np.ndarray)
  Out: query_result (dict)  — triggered by query_location skill
"""

from __future__ import annotations

import hashlib
import logging
import os
import time
from typing import Any

import numpy as np

from core.backend_status import BackendStatus
from core.encoder_protocol import EncoderProtocol
from core.module import Module, skill
from core.msgs.nav import Odometry
from core.msgs.semantic import SceneGraph
from core.registry import register
from core.stream import In, Out

logger = logging.getLogger(__name__)

_DEFAULT_PERSIST_DIR = os.path.join(os.path.expanduser("~"), ".nova", "vector_memory")


@register("vector_memory", "default", description="CLIP + ChromaDB spatial vector search")
class VectorMemoryModule(Module, layer=3):
    _run_in_worker = True
    _worker_group = "semantic"
    """Vector-based spatial memory for fuzzy location queries.

    Stores CLIP embeddings of scene snapshots indexed by robot position.
    Queries return the closest matching location from past experience.
    """

    scene_graph: In[SceneGraph]
    odometry:    In[Odometry]
    image:       In[np.ndarray]

    query_result: Out[dict]

    def __init__(
        self,
        persist_dir: str = "",
        collection_name: str = "spatial_memory",
        store_interval: float = 5.0,
        max_results: int = 5,
        **kw,
    ):
        super().__init__(**kw)
        self._encoder_status = BackendStatus.configured_as(kw.get("encoder_backend", "auto"))
        self._persist_dir = persist_dir or _DEFAULT_PERSIST_DIR
        self._collection_name = collection_name
        self._store_interval = store_interval
        self._max_results = max_results

        self._collection = None      # ChromaDB collection
        self._encoder: EncoderProtocol | None = None
        self._st_model = None        # sentence-transformers model (fallback to CLIP)
        self._encoder_type = "none"  # "mobileclip" | "clip" | "sentence_transformers" | "lexical_hash" | "none"
        self._embedding_dim: int | None = None
        self._use_chromadb = False

        self._max_np_entries: int = kw.get("max_np_entries", 10000)

        # Fallback numpy store — deque(maxlen) gives O(1) FIFO eviction vs O(n) list.pop(0)
        from collections import deque
        self._np_embeddings: deque = deque(maxlen=self._max_np_entries)
        self._np_metadata: deque = deque(maxlen=self._max_np_entries)

        self._robot_xy = (0.0, 0.0)
        self._robot_z = 0.0
        self._latest_labels: list[str] = []
        self._latest_image: np.ndarray | None = None
        self._last_store_time = 0.0
        self._store_count = 0

    def setup(self) -> None:
        self.scene_graph.subscribe(self._on_scene_graph)
        self.odometry.subscribe(self._on_odom)
        self.image.subscribe(self._on_image)
        self.scene_graph.set_policy("throttle", interval=self._store_interval)

        self._init_encoder()
        self._init_store()

    def _init_encoder(self) -> None:
        # Attempt 1: MobileCLIP text encoder (preferred for robot text queries).
        # Encoder constructors do not load weights; load_model() is required
        # before encode_text() can produce embeddings.
        try:
            from semantic.perception.mobileclip_encoder import MobileCLIPEncoder

            candidate = MobileCLIPEncoder(device="auto")
            candidate.load_model()
            test_result = candidate.encode_text(["test"])
            if test_result is not None and len(test_result) > 0:
                self._encoder = candidate
                self._encoder_type = "mobileclip"
                self._encoder_status.use("mobileclip", degraded=False)
                self._embedding_dim = int(np.array(test_result[0]).size)
                logger.info("VectorMemoryModule: MobileCLIP text encoder ready")
                return
            logger.info("VectorMemoryModule: MobileCLIP smoke-test returned empty; trying CLIP")
        except ImportError:
            logger.info("VectorMemoryModule: MobileCLIP encoder not available, trying CLIP")
        except Exception as exc:
            logger.warning("VectorMemoryModule: MobileCLIP init failed (%s), trying CLIP", exc)

        # Attempt 1: CLIP encoder (preferred — full image+text support).
        # A smoke-test encode call verifies the model weights are actually usable,
        # not just that the class imports.
        try:
            from semantic.perception.clip_encoder import CLIPEncoder
            candidate = CLIPEncoder(model_name="ViT-B/32", device="auto")
            candidate.load_model()
            test_result = candidate.encode_text(["test"])
            if test_result is not None and len(test_result) > 0:
                self._encoder = candidate
                self._encoder_type = "clip"
                self._encoder_status.use("clip", degraded=False)
                self._embedding_dim = int(np.array(test_result[0]).size)
                logger.info("VectorMemoryModule: CLIP encoder ready")
                return
            logger.info("VectorMemoryModule: CLIP encoder smoke-test returned empty — trying sentence-transformers")
        except ImportError:
            logger.info("VectorMemoryModule: CLIP encoder not available, trying sentence-transformers")
        except Exception as exc:
            logger.warning("VectorMemoryModule: CLIP encoder init failed (%s), trying sentence-transformers", exc)

        # Attempt 2: sentence-transformers (CPU-safe, 384-dim, semantically meaningful).
        try:
            from sentence_transformers import SentenceTransformer
            self._st_model = SentenceTransformer("all-MiniLM-L6-v2")
            self._encoder_type = "sentence_transformers"
            self._encoder_status.use("sentence_transformers", degraded=False)
            self._embedding_dim = 384
            logger.info("VectorMemoryModule: sentence-transformers encoder ready (all-MiniLM-L6-v2, 384-dim)")
            return
        except ImportError:
            pass
        except Exception as exc:
            logger.warning("VectorMemoryModule: sentence-transformers model load failed: %s", exc)

        # No semantic encoder available: keep the module usable with a stable
        # lexical fallback, while reporting that semantic embeddings are degraded.
        self._encoder_type = "lexical_hash"
        self._encoder_status.use(
            "lexical_hash",
            reason="no semantic text encoder available",
            degraded=True,
        )
        self._embedding_dim = 512
        logger.warning(
            "VectorMemoryModule: no semantic text encoder available; using "
            "deterministic lexical fallback. Install sentence-transformers for "
            "semantic vector search."
        )

    def _init_store(self) -> None:
        try:
            import chromadb
            os.makedirs(self._persist_dir, exist_ok=True)
            client = chromadb.PersistentClient(path=self._persist_dir)
            self._collection = client.get_or_create_collection(
                name=self._collection_name,
                metadata={
                    "hnsw:space": "cosine",
                    "lingtu_encoder_type": self._encoder_type,
                    "lingtu_embedding_dim": str(self._embedding_dim or ""),
                },
            )
            self._use_chromadb = True
            count = self._collection.count()
            logger.info("VectorMemoryModule: ChromaDB ready (%d entries)", count)
        except ImportError:
            logger.info("VectorMemoryModule: chromadb not installed, using numpy fallback")
        except Exception as e:
            logger.warning("VectorMemoryModule: ChromaDB init failed (%s), using numpy", e)

    # ── Input handlers ────────────────────────────────────────────────────────

    def _on_odom(self, odom: Odometry) -> None:
        self._robot_xy = (odom.x, odom.y)
        self._robot_z = getattr(odom, "z", 0.0)

    def _on_image(self, img: np.ndarray) -> None:
        self._latest_image = img

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        labels = [obj.label for obj in sg.objects if obj.label]
        if not labels:
            return

        self._latest_labels = labels
        now = time.time()
        if now - self._last_store_time < self._store_interval:
            return

        self._store_snapshot(labels)
        self._last_store_time = now

    # ── Store ─────────────────────────────────────────────────────────────────

    def _snapshot_metadata(self, text: str, embedding: np.ndarray) -> dict[str, Any]:
        semantic_encoder_ready = self._semantic_encoder_ready()
        degraded = self._encoder_type == "lexical_hash"
        embedding_dim = int(np.array(embedding).size)
        return {
            "x": float(self._robot_xy[0]),
            "y": float(self._robot_xy[1]),
            "z": float(self._robot_z),
            "labels": text,
            "ts": time.time(),
            "encoder_type": self._encoder_type,
            "semantic_encoder_ready": semantic_encoder_ready,
            "degraded": degraded,
            "navigable": semantic_encoder_ready and not degraded,
            "embedding_dim": embedding_dim,
        }

    def _store_snapshot(self, labels: list[str]) -> None:
        text = ", ".join(sorted(set(labels)))
        embedding = self._encode_text(text)
        if embedding is None:
            return

        meta = self._snapshot_metadata(text, embedding)
        doc_id = f"snap_{self._store_count}"
        self._store_count += 1

        if self._use_chromadb and self._collection is not None:
            try:
                self._collection.upsert(
                    ids=[doc_id],
                    embeddings=[embedding.tolist()],
                    metadatas=[meta],
                    documents=[text],
                )
                return
            except Exception as exc:
                logger.warning(
                    "VectorMemory: ChromaDB upsert failed (%s), using numpy fallback",
                    exc,
                )
                self._use_chromadb = False

        self._np_embeddings.append(embedding)
        self._np_metadata.append(meta)
        # deque(maxlen) auto-evicts oldest entry automatically.

    def _encode_text(self, text: str) -> np.ndarray | None:
        """Encode text to a unit-L2-normalized embedding vector.

        Priority: MobileCLIP -> CLIP -> sentence-transformers -> deterministic lexical hash.
        The lexical path is intentionally exposed as a degraded runtime fallback
        so fuzzy location queries still work for exact labels without optional
        model dependencies.
        """
        if self._encoder_type in ("mobileclip", "clip") and self._encoder is not None:
            try:
                results = self._encoder.encode_text([text])
                if results is not None and len(results) > 0:
                    vec = np.array(results[0], dtype=np.float32).flatten()
                    norm = float(np.linalg.norm(vec))
                    if norm > 1e-8:
                        vec /= norm
                        return vec
            except Exception as exc:
                logger.debug(
                    "VectorMemory: %s encode_text failed: %s",
                    self._encoder_type,
                    exc,
                )
            return None

        if self._encoder_type == "sentence_transformers" and self._st_model is not None:
            try:
                vec = np.array(
                    self._st_model.encode(text, normalize_embeddings=True),
                    dtype=np.float32,
                ).flatten()
                return vec
            except Exception as exc:
                logger.debug("VectorMemory: sentence-transformers encode failed: %s", exc)
            return None

        # _encoder_type == "lexical_hash": runtime degraded fallback.
        # _encoder_type == "none": setup() was bypassed in lightweight tests.
        return self._hash_embedding(text)

    @staticmethod
    def _hash_embedding(text: str, dim: int = 512) -> np.ndarray:
        """Deterministic hash embedding for text (fallback when no CLIP).

        Tokenizes on spaces, commas, and common stop words to ensure
        'find backpack' matches 'backpack, bench'.
        """
        import re
        _STOP = {
            "go", "to", "the", "find", "where", "is", "a", "an", "my",
            "at", "in", "on", "for", "of", "area", "place", "spot",
        }
        words = re.split(r'[,\s]+', text.lower().strip())
        words = [w for w in words if w and w not in _STOP]
        vec = np.zeros(dim, dtype=np.float32)
        for word in words:
            digest = hashlib.blake2b(word.encode("utf-8"), digest_size=8).digest()
            h = int.from_bytes(digest, byteorder="big", signed=False) % dim
            vec[h] += 1.0
        norm = np.linalg.norm(vec)
        if norm > 0:
            vec /= norm
        return vec

    # ── Query ─────────────────────────────────────────────────────────────────

    def _entry_count(self) -> int:
        if self._use_chromadb and self._collection is not None:
            try:
                return int(self._collection.count())
            except Exception as exc:
                logger.warning(
                    "VectorMemory: ChromaDB count failed (%s), using numpy fallback",
                    exc,
                )
                self._use_chromadb = False
        return len(self._np_embeddings)

    def _hit_matches_active_encoder(self, meta: dict[str, Any] | None) -> bool:
        if not isinstance(meta, dict):
            return False

        meta_encoder = str(meta.get("encoder_type") or "")
        if meta_encoder != self._encoder_type:
            return False

        meta_dim = meta.get("embedding_dim")
        if meta_dim is not None and self._embedding_dim is not None:
            try:
                if int(meta_dim) != int(self._embedding_dim):
                    return False
            except (TypeError, ValueError):
                return False
        return True

    def _query(self, text: str, n: int = 0) -> list[dict]:
        n = n or self._max_results
        embedding = self._encode_text(text)
        if embedding is None:
            return []

        if self._use_chromadb and self._collection is not None:
            try:
                count = int(self._collection.count())
                results = self._collection.query(
                    query_embeddings=[embedding.tolist()],
                    n_results=min(n, max(count, 1)),
                )
                hits = []
                for i in range(len(results["ids"][0])):
                    meta = results["metadatas"][0][i]
                    if not self._hit_matches_active_encoder(meta):
                        continue
                    hits.append({
                        "x": meta["x"],
                        "y": meta["y"],
                        "z": meta.get("z", 0.0),
                        "labels": meta.get("labels", ""),
                        "score": 1.0 - results["distances"][0][i],  # cosine → similarity
                        "ts": meta.get("ts", 0),
                        "encoder_type": meta.get("encoder_type", "unknown"),
                        "semantic_encoder_ready": bool(meta.get("semantic_encoder_ready", False)),
                        "degraded": bool(meta.get("degraded", True)),
                        "navigable": bool(meta.get("navigable", False)),
                        "embedding_dim": meta.get("embedding_dim"),
                    })
                return hits
            except Exception as e:
                logger.warning("VectorMemory: ChromaDB query failed: %s", e)
                return self._numpy_query(embedding, n)
        else:
            return self._numpy_query(embedding, n)

    def _numpy_query(self, query_vec: np.ndarray, n: int) -> list[dict]:
        if not self._np_embeddings:
            return []

        rows = []
        metas = []
        query_vec = np.asarray(query_vec, dtype=np.float32).flatten()
        for vec, meta in zip(list(self._np_embeddings), list(self._np_metadata)):
            arr = np.asarray(vec, dtype=np.float32).flatten()
            if arr.shape != query_vec.shape:
                continue
            if not self._hit_matches_active_encoder(meta):
                continue
            rows.append(arr)
            metas.append(meta)

        if not rows:
            return []

        try:
            mat = np.stack(rows)
            sims = mat @ query_vec
        except Exception as exc:
            logger.warning("VectorMemory: numpy query failed: %s", exc)
            return []

        top_idx = np.argsort(sims)[::-1][:n]
        return [
            {**metas[i], "score": float(sims[i])}
            for i in top_idx if sims[i] > 0.1
        ]

    # ── @skill methods (MCP-exposed) ──────────────────────────────────────────

    @skill
    def query_location(self, text: str) -> str:
        """Fuzzy search for a location by natural language description.

        Example: "去上次放背包的地方" → returns closest matching position.
        """
        import json
        results = self._query(text)
        semantic_encoder_ready = self._semantic_encoder_ready()
        degraded = self._encoder_type == "lexical_hash"
        navigable = semantic_encoder_ready and not degraded
        if not results:
            return json.dumps({
                "query": text,
                "found": False,
                "results": [],
                "encoder_type": self._encoder_type,
                "semantic_encoder_ready": semantic_encoder_ready,
                "degraded": degraded,
                "navigable": navigable,
            }, ensure_ascii=False)
        return json.dumps({
            "query": text,
            "found": True,
            "best": {"x": results[0]["x"], "y": results[0]["y"],
                     "z": results[0].get("z", 0.0),
                     "labels": results[0]["labels"], "score": results[0]["score"]},
            "results": results[:3],
            "encoder_type": self._encoder_type,
            "semantic_encoder_ready": semantic_encoder_ready,
            "degraded": degraded,
            "navigable": navigable,
        }, ensure_ascii=False)

    def _semantic_encoder_ready(self) -> bool:
        return self._encoder_type in (
            "mobileclip",
            "clip",
            "sentence_transformers",
        )

    def _encoder_backend_health(self) -> dict[str, Any]:
        fields = self._encoder_status.as_health_fields()
        if self._encoder_type == "none" or fields["backend"] == self._encoder_type:
            return fields
        degraded = self._encoder_type == "lexical_hash"
        return {
            "configured_backend": fields["configured_backend"],
            "backend": self._encoder_type,
            "degraded": degraded,
            "degraded_reason": (
                "no semantic text encoder available" if degraded else ""
            ),
        }

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        entry_count = self._entry_count()
        info["entry_count"] = entry_count
        info["backend"] = "chromadb" if self._use_chromadb else "numpy"
        info["store_count"] = self._store_count
        info["encoder_ready"] = self._encoder_type != "none"
        info["semantic_encoder_ready"] = self._semantic_encoder_ready()
        info["degraded"] = self._encoder_type == "lexical_hash"
        info["encoder_type"] = self._encoder_type
        info["encoder_backend"] = self._encoder_backend_health()
        info["embedding_dim"] = self._embedding_dim
        info["persist_dir"] = self._persist_dir
        return info

    @skill
    def get_memory_stats(self) -> str:
        """Return vector memory statistics."""
        import json
        count = self._entry_count()
        return json.dumps({
            "backend": "chromadb" if self._use_chromadb else "numpy",
            "entries": count,
            "store_count": self._store_count,
            "persist_dir": self._persist_dir,
            "max_np_entries": self._max_np_entries,
            "encoder_type": self._encoder_type,
            "encoder_backend": self._encoder_backend_health(),
            "semantic_encoder_ready": self._semantic_encoder_ready(),
            "degraded": self._encoder_type == "lexical_hash",
            "embedding_dim": self._embedding_dim,
        })

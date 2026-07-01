"""Comprehensive tests for 6 memory modules.

Pure unit tests -- no ROS2, no hardware, no external services.
Follows the patterns established in test_semantic_modules.py and test_cmd_vel_mux.py.
"""

from __future__ import annotations

import json
import hashlib
import os
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from runtime.msgs.geometry import Vector3
from runtime.msgs.nav import Odometry, Pose
from runtime.msgs.semantic import Detection3D, Region, SceneGraph

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _odom(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Odometry:
    return Odometry(pose=Pose(x, y, z))


def _scene_graph(labels: list[str], region_name: str = "") -> SceneGraph:
    """Build a minimal SceneGraph with the given labels and optional region."""
    objects = [
        Detection3D(id=f"obj_{i}", label=lbl, confidence=0.9,
                     position=Vector3(float(i), 0.0, 0.0))
        for i, lbl in enumerate(labels)
    ]
    regions = []
    if region_name:
        regions = [Region(
            name=region_name,
            object_ids=[o.id for o in objects],
            center=Vector3(0.0, 0.0, 0.0),
        )]
    return SceneGraph(objects=objects, relations=[], regions=regions)


# ===========================================================================
# 1. SemanticMapperModule
# ===========================================================================

class TestSemanticMapperInstantiation(unittest.TestCase):
    """Test SemanticMapperModule creation and initial state."""

    def _make(self, **kw):
        from memory.modules.semantic_mapper_module import SemanticMapperModule
        return SemanticMapperModule(**kw)

    def test_default_save_dir(self):
        m = self._make()
        self.assertTrue(m._save_dir.endswith("semantic"))

    def test_custom_save_dir(self):
        m = self._make(save_dir="/tmp/test_smap")
        self.assertEqual(m._save_dir, "/tmp/test_smap")

    def test_initial_kg_is_none(self):
        m = self._make()
        self.assertIsNone(m._kg)

    def test_initial_tsg_is_none(self):
        m = self._make()
        self.assertIsNone(m._tsg)

    def test_initial_robot_xy(self):
        m = self._make()
        self.assertEqual(m._robot_xy, (0.0, 0.0))

    def test_initial_sg_count_zero(self):
        m = self._make()
        self.assertEqual(m._sg_count, 0)

    def test_custom_save_interval(self):
        m = self._make(save_interval_s=60.0)
        self.assertAlmostEqual(m._save_interval, 60.0)


class TestSemanticMapperSetup(unittest.TestCase):
    """Test SemanticMapperModule setup and subscriptions."""

    def _make(self, **kw):
        from memory.modules.semantic_mapper_module import SemanticMapperModule
        return SemanticMapperModule(**kw)

    @patch("memory.modules.semantic_mapper_module.SemanticMapperModule._init_backends")
    def test_setup_subscribes_ports(self, mock_init):
        m = self._make()
        m.setup()
        # setup ran without error and _init_backends was called
        mock_init.assert_called_once()

    def test_stable_room_id_is_deterministic(self):
        m = self._make()
        id1 = m._stable_room_id("kitchen")
        id2 = m._stable_room_id("kitchen")
        self.assertEqual(id1, id2)

    def test_stable_room_id_increments_for_new_rooms(self):
        m = self._make()
        id_a = m._stable_room_id("kitchen")
        id_b = m._stable_room_id("living_room")
        self.assertNotEqual(id_a, id_b)
        self.assertEqual(id_b, id_a + 1)

    def test_on_odom_updates_robot_xy(self):
        m = self._make()
        m._on_odom(_odom(3.0, 4.0))
        self.assertAlmostEqual(m._robot_xy[0], 3.0)
        self.assertAlmostEqual(m._robot_xy[1], 4.0)

    def test_on_scene_graph_skips_empty_regions(self):
        m = self._make()
        m._kg = None
        m._tsg = None
        sg = _scene_graph(["chair"], region_name="")
        sg.regions = []
        m._on_scene_graph(sg)
        self.assertEqual(m._sg_count, 0)

    def test_extract_objects(self):
        m = self._make()
        sg = _scene_graph(["chair", "table"])
        labels, confs = m._extract_objects(sg, ["obj_0", "obj_1"])
        self.assertEqual(labels, ["chair", "table"])
        self.assertEqual(len(confs), 2)

    def test_extract_objects_missing_id(self):
        m = self._make()
        sg = _scene_graph(["chair"])
        labels, confs = m._extract_objects(sg, ["nonexistent_id"])
        self.assertEqual(labels, [])
        self.assertEqual(confs, [])

    def test_health_returns_dict(self):
        m = self._make()
        h = m.health()
        self.assertIn("semantic_mapper", h)
        self.assertEqual(h["semantic_mapper"]["sg_updates"], 0)

    def test_kg_path(self):
        m = self._make(save_dir="/tmp/test_dir")
        self.assertEqual(m._kg_path(), os.path.join("/tmp/test_dir", "room_object_kg.json"))

    def test_tsg_path(self):
        m = self._make(save_dir="/tmp/test_dir")
        self.assertEqual(m._tsg_path(), os.path.join("/tmp/test_dir", "topology_graph.json"))


# ===========================================================================
# 2. VectorMemoryModule
# ===========================================================================

class TestVectorMemoryInstantiation(unittest.TestCase):
    """Test VectorMemoryModule creation and initial state."""

    def _make(self, **kw):
        from memory.modules.vector_memory_module import VectorMemoryModule
        return VectorMemoryModule(**kw)

    def test_default_persist_dir(self):
        m = self._make()
        self.assertTrue(m._persist_dir.endswith("vector_memory"))

    def test_custom_persist_dir(self):
        m = self._make(persist_dir="/tmp/vec_test")
        self.assertEqual(m._persist_dir, "/tmp/vec_test")

    def test_initial_store_count_zero(self):
        m = self._make()
        self.assertEqual(m._store_count, 0)

    def test_initial_use_chromadb_false(self):
        m = self._make()
        self.assertFalse(m._use_chromadb)

    def test_default_max_results(self):
        m = self._make()
        self.assertEqual(m._max_results, 5)

    def test_custom_max_np_entries(self):
        m = self._make(max_np_entries=100)
        self.assertEqual(m._max_np_entries, 100)


class TestVectorMemoryOperations(unittest.TestCase):
    """Test VectorMemoryModule store and query with numpy fallback."""

    def _make(self, **kw):
        from memory.modules.vector_memory_module import VectorMemoryModule
        m = VectorMemoryModule(**kw)
        # Do not call setup() to avoid importing CLIP / ChromaDB
        m._use_chromadb = False
        m._encoder = None
        return m

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

    def test_hash_embedding_uses_stable_word_hash(self):
        from memory.modules.vector_memory_module import VectorMemoryModule

        v = VectorMemoryModule._hash_embedding("backpack")
        expected = int.from_bytes(
            hashlib.blake2b(b"backpack", digest_size=8).digest(),
            byteorder="big",
            signed=False,
        ) % 512

        self.assertGreater(v[expected], 0.0)
        self.assertEqual(int(np.count_nonzero(v)), 1)

    def test_runtime_lexical_fallback_remains_queryable(self):
        m = self._make()
        m._encoder_type = "lexical_hash"
        m._robot_xy = (2.0, 3.0)
        m._store_snapshot(["backpack", "bench"])

        results = m._query("find backpack")
        response = json.loads(m.query_location("find backpack"))

        self.assertGreater(len(results), 0)
        self.assertAlmostEqual(results[0]["x"], 2.0)
        self.assertTrue(response["found"])
        self.assertFalse(response["semantic_encoder_ready"])
        self.assertTrue(response["degraded"])
        self.assertFalse(response["navigable"])

    def test_runtime_lexical_fallback_reports_degraded_health(self):
        m = self._make()
        m._encoder_type = "lexical_hash"

        health = m.health()
        stats = json.loads(m.get_memory_stats())
        self.assertTrue(health["encoder_ready"])
        self.assertFalse(health["semantic_encoder_ready"])
        self.assertTrue(health["degraded"])
        self.assertEqual(health["encoder_type"], "lexical_hash")
        self.assertEqual(health["encoder_backend"]["configured_backend"], "auto")
        self.assertEqual(health["encoder_backend"]["backend"], "lexical_hash")
        self.assertTrue(health["encoder_backend"]["degraded"])
        self.assertIn("semantic text encoder", health["encoder_backend"]["degraded_reason"])
        self.assertEqual(stats["encoder_type"], "lexical_hash")
        self.assertEqual(stats["encoder_backend"]["backend"], "lexical_hash")
        self.assertFalse(stats["semantic_encoder_ready"])
        self.assertTrue(stats["degraded"])

    def test_store_snapshot_uses_numpy_fallback(self):
        m = self._make()
        m._robot_xy = (1.0, 2.0)
        m._robot_z = 1.5
        m._store_snapshot(["chair", "table"])
        self.assertEqual(len(m._np_embeddings), 1)
        self.assertEqual(m._np_metadata[0]["x"], 1.0)
        self.assertEqual(m._np_metadata[0]["z"], 1.5)
        self.assertEqual(m._np_metadata[0]["encoder_type"], "none")
        self.assertFalse(m._np_metadata[0]["navigable"])
        self.assertEqual(m._store_count, 1)

    def test_chromadb_upsert_failure_falls_back_to_numpy(self):
        class _BrokenCollection:
            def upsert(self, **kwargs):
                raise ValueError("embedding dimension mismatch")

        m = self._make()
        m._encoder_type = "lexical_hash"
        m._use_chromadb = True
        m._collection = _BrokenCollection()
        m._robot_xy = (4.0, 5.0)

        m._store_snapshot(["backpack"])

        self.assertFalse(m._use_chromadb)
        self.assertEqual(len(m._np_embeddings), 1)
        self.assertEqual(m._np_metadata[0]["labels"], "backpack")

    def test_chromadb_legacy_encoder_hits_are_filtered(self):
        class _Collection:
            def count(self):
                return 1

            def query(self, **kwargs):
                return {
                    "ids": [["snap_legacy"]],
                    "metadatas": [[{
                        "x": 1.0,
                        "y": 2.0,
                        "labels": "backpack",
                        "ts": 1.0,
                        "encoder_type": "lexical_hash",
                        "semantic_encoder_ready": False,
                        "degraded": True,
                        "navigable": False,
                        "embedding_dim": 512,
                    }]],
                    "distances": [[0.0]],
                }

        m = self._make()
        m._encoder_type = "clip"
        m._embedding_dim = 512
        m._use_chromadb = True
        m._collection = _Collection()
        m._encode_text = lambda text: np.ones(512, dtype=np.float32)

        self.assertEqual(m._query("find backpack"), [])

    def test_chromadb_count_failure_does_not_break_health_or_stats(self):
        class _BrokenCollection:
            def count(self):
                raise RuntimeError("locked")

        m = self._make()
        m._use_chromadb = True
        m._collection = _BrokenCollection()

        health = m.health()
        stats_raw = m.get_memory_stats()
        import json
        stats = json.loads(stats_raw) if isinstance(stats_raw, str) else stats_raw

        self.assertFalse(m._use_chromadb)
        self.assertEqual(health["backend"], "numpy")
        self.assertEqual(health["entry_count"], 0)
        self.assertEqual(stats["backend"], "numpy")
        self.assertEqual(stats["entries"], 0)

    def test_numpy_query_skips_mismatched_embedding_dimensions(self):
        m = self._make()
        m._encoder_type = "lexical_hash"
        m._embedding_dim = 512
        m._np_embeddings.append(np.ones(4, dtype=np.float32))
        m._np_metadata.append({
            "x": 1.0,
            "y": 2.0,
            "labels": "bad-dim",
            "encoder_type": "lexical_hash",
            "embedding_dim": 4,
            "semantic_encoder_ready": False,
            "degraded": True,
            "navigable": False,
        })

        self.assertEqual(m._query("bad-dim"), [])

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
        # Oldest entries should be evicted
        self.assertAlmostEqual(m._np_metadata[0]["x"], 2.0)

    def test_on_odom_updates_position(self):
        m = self._make()
        m._on_odom(_odom(7.0, 8.0, 1.25))
        self.assertEqual(m._robot_xy, (7.0, 8.0))
        self.assertEqual(m._robot_z, 1.25)

    def test_query_location_skill_not_found(self):
        m = self._make()
        result = json.loads(m.query_location("missing thing"))
        self.assertFalse(result["found"])

    def test_query_location_skill_found(self):
        m = self._make()
        m._robot_xy = (3.0, 4.0)
        m._robot_z = 1.25
        m._store_snapshot(["backpack", "tree"])
        result = json.loads(m.query_location("backpack"))
        self.assertTrue(result["found"])
        self.assertIn("best", result)
        self.assertEqual(result["best"]["z"], 1.25)

    def test_get_memory_stats_numpy(self):
        m = self._make()
        m._robot_xy = (0.0, 0.0)
        m._store_snapshot(["item"])
        stats = json.loads(m.get_memory_stats())
        self.assertEqual(stats["backend"], "numpy")
        self.assertEqual(stats["entries"], 1)


# ===========================================================================
# 3. EpisodicMemoryModule
# ===========================================================================

class TestEpisodicMemoryInstantiation(unittest.TestCase):
    """Test EpisodicMemoryModule creation and initial state."""

    def _make(self, **kw):
        from memory.modules.episodic_module import EpisodicMemoryModule
        return EpisodicMemoryModule(**kw)

    def test_default_creation(self):
        m = self._make()
        self.assertIsNotNone(m._memory)

    def test_custom_max_records(self):
        m = self._make(max_records=100)
        self.assertEqual(m._memory.MAX_RECORDS, 100)

    def test_custom_min_distance(self):
        m = self._make(min_distance_m=5.0)
        self.assertAlmostEqual(m._memory.MIN_DISTANCE_M, 5.0)

    def test_memory_property_accessible(self):
        m = self._make()
        self.assertIs(m.memory, m._memory)

    def test_initial_record_count_zero(self):
        m = self._make()
        self.assertEqual(len(m._memory), 0)


class TestEpisodicMemoryOperations(unittest.TestCase):
    """Test EpisodicMemoryModule record addition and dedup."""

    def _make(self, **kw):
        from memory.modules.episodic_module import EpisodicMemoryModule
        m = EpisodicMemoryModule(**kw)
        m.setup()
        return m

    def test_on_scene_graph_requires_odom(self):
        m = self._make()
        sg = _scene_graph(["chair"])
        m._on_scene_graph(sg)
        # No record added because no odometry received yet
        self.assertEqual(len(m._memory), 0)

    def test_record_added_with_odom(self):
        m = self._make()
        m._on_odom(_odom(1.0, 2.0, 0.0))
        sg = _scene_graph(["chair", "table"])
        m._on_scene_graph(sg)
        self.assertEqual(len(m._memory), 1)

    def test_position_dedup(self):
        m = self._make(min_distance_m=2.0)
        m._on_odom(_odom(1.0, 1.0))
        m._on_scene_graph(_scene_graph(["chair"]))
        # Move less than min_distance
        m._on_odom(_odom(1.5, 1.5))
        m._on_scene_graph(_scene_graph(["table"]))
        # Only one record because distance < 2.0
        self.assertEqual(len(m._memory), 1)

    def test_record_added_after_sufficient_movement(self):
        m = self._make(min_distance_m=1.0)
        m._on_odom(_odom(0.0, 0.0))
        m._on_scene_graph(_scene_graph(["chair"]))
        m._on_odom(_odom(5.0, 5.0))
        m._on_scene_graph(_scene_graph(["table"]))
        self.assertEqual(len(m._memory), 2)

    def test_time_ordering(self):
        m = self._make(min_distance_m=0.1)
        for i in range(3):
            m._on_odom(_odom(float(i * 10), 0.0))
            m._on_scene_graph(_scene_graph([f"obj_{i}"]))
        records = m._memory.recent_n(3)
        for j in range(len(records) - 1):
            self.assertLessEqual(records[j].timestamp, records[j + 1].timestamp)

    def test_health_returns_record_count(self):
        m = self._make()
        m._on_odom(_odom(1.0, 2.0))
        m._on_scene_graph(_scene_graph(["a"]))
        h = m.health()
        self.assertEqual(h["record_count"], 1)
        self.assertIsNotNone(h["oldest_ts"])

    def test_health_empty(self):
        m = self._make()
        h = m.health()
        self.assertEqual(h["record_count"], 0)
        self.assertIsNone(h["oldest_ts"])

    def test_get_recent_observations_skill(self):
        m = self._make(min_distance_m=0.1)
        m._on_odom(_odom(0.0, 0.0))
        m._on_scene_graph(_scene_graph(["chair"]))
        m._on_odom(_odom(10.0, 10.0))
        m._on_scene_graph(_scene_graph(["desk"]))
        result = json.loads(m.get_recent_observations(5))
        self.assertIn("observations", result)
        self.assertEqual(len(result["observations"]), 2)

    def test_memory_context_published(self):
        m = self._make()
        published = []
        m.memory_context.subscribe(lambda s: published.append(s))
        m._on_odom(_odom(1.0, 2.0))
        m._on_scene_graph(_scene_graph(["chair"]))
        self.assertEqual(len(published), 1)
        self.assertIsInstance(published[0], str)


# ===========================================================================
# 4. TaggedLocationsModule
# ===========================================================================

class TestTaggedLocationsInstantiation(unittest.TestCase):
    """Test TaggedLocationsModule creation."""

    def _make(self, **kw):
        from memory.modules.tagged_locations_module import TaggedLocationsModule
        return TaggedLocationsModule(**kw)

    def test_default_creation(self):
        m = self._make()
        self.assertIsNotNone(m._store)

    def test_store_property_accessible(self):
        m = self._make()
        self.assertIs(m.store, m._store)

    def test_initial_no_tags(self):
        m = self._make()
        self.assertEqual(len(m._store.list_all()), 0)


class TestTaggedLocationsOperations(unittest.TestCase):
    """Test tag/untag/goto/list operations."""

    def _make(self, **kw):
        from memory.modules.tagged_locations_module import TaggedLocationsModule
        m = TaggedLocationsModule(**kw)
        m.setup()
        return m

    def test_save_command(self):
        m = self._make()
        status_msgs = []
        m.tag_status.subscribe(lambda s: status_msgs.append(s))
        m._on_odom(_odom(5.0, 10.0, 0.0))
        m._on_command("save:home")
        self.assertEqual(status_msgs[-1], "saved:home")
        entry = m._store.query("home")
        self.assertIsNotNone(entry)
        self.assertAlmostEqual(entry["position"][0], 5.0)

    def test_save_without_odom_reports_error(self):
        m = self._make()
        status_msgs = []
        m.tag_status.subscribe(lambda s: status_msgs.append(s))
        m._on_command("save:place")
        self.assertEqual(status_msgs[-1], "error:no_odometry")

    def test_save_empty_name_reports_error(self):
        m = self._make()
        status_msgs = []
        m.tag_status.subscribe(lambda s: status_msgs.append(s))
        m._on_odom(_odom(1.0, 1.0))
        m._on_command("save:")
        self.assertEqual(status_msgs[-1], "error:empty_name")

    def test_goto_exact_match(self):
        m = self._make()
        m._store.tag("office", x=1.0, y=2.0, z=0.0)
        published = []
        m.saved_location.subscribe(lambda p: published.append(p))
        status_msgs = []
        m.tag_status.subscribe(lambda s: status_msgs.append(s))
        m._on_command("goto:office")
        self.assertEqual(len(published), 1)
        self.assertAlmostEqual(published[0].pose.x, 1.0)
        self.assertEqual(status_msgs[-1], "recalled:office")

    def test_goto_fuzzy_match(self):
        m = self._make()
        m._store.tag("conference room", x=10.0, y=20.0)
        published = []
        m.saved_location.subscribe(lambda p: published.append(p))
        m._on_command("goto:conference")
        self.assertEqual(len(published), 1)

    def test_goto_not_found(self):
        m = self._make()
        status_msgs = []
        m.tag_status.subscribe(lambda s: status_msgs.append(s))
        m._on_command("goto:nonexistent")
        self.assertEqual(status_msgs[-1], "not_found:nonexistent")

    def test_remove_existing(self):
        m = self._make()
        m._store.tag("temp", x=0.0, y=0.0)
        status_msgs = []
        m.tag_status.subscribe(lambda s: status_msgs.append(s))
        m._on_command("remove:temp")
        self.assertEqual(status_msgs[-1], "removed:temp")
        self.assertIsNone(m._store.query("temp"))

    def test_remove_nonexistent(self):
        m = self._make()
        status_msgs = []
        m.tag_status.subscribe(lambda s: status_msgs.append(s))
        m._on_command("remove:ghost")
        self.assertEqual(status_msgs[-1], "not_found:ghost")

    def test_list_command(self):
        m = self._make()
        m._store.tag("a", x=1.0, y=0.0)
        m._store.tag("b", x=2.0, y=0.0)
        status_msgs = []
        m.tag_status.subscribe(lambda s: status_msgs.append(s))
        m._on_command("list")
        self.assertTrue(status_msgs[-1].startswith("locations:"))

    def test_list_command_empty(self):
        m = self._make()
        status_msgs = []
        m.tag_status.subscribe(lambda s: status_msgs.append(s))
        m._on_command("list")
        self.assertEqual(status_msgs[-1], "locations:")

    def test_unknown_command(self):
        m = self._make()
        status_msgs = []
        m.tag_status.subscribe(lambda s: status_msgs.append(s))
        m._on_command("dance")
        self.assertTrue(status_msgs[-1].startswith("unknown_command:"))

    def test_empty_command_ignored(self):
        m = self._make()
        status_msgs = []
        m.tag_status.subscribe(lambda s: status_msgs.append(s))
        m._on_command("")
        self.assertEqual(len(status_msgs), 0)

    def test_json_persistence(self):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
            path = f.name
        try:
            m = self._make(json_path=path)
            m._on_odom(_odom(7.0, 8.0, 0.0))
            m._on_command("save:park")
            m._store.save()

            # Create a new store from the same file
            from memory.spatial.tagged_locations import TaggedLocationStore
            store2 = TaggedLocationStore(json_path=path)
            entry = store2.query("park")
            self.assertIsNotNone(entry)
            self.assertAlmostEqual(entry["position"][0], 7.0)
        finally:
            os.unlink(path)

    def test_health_returns_location_count(self):
        m = self._make()
        m._store.tag("x", x=0.0, y=0.0)
        h = m.health()
        self.assertEqual(h["location_count"], 1)

    def test_list_tags_skill(self):
        m = self._make()
        m._store.tag("cafe", x=1.0, y=2.0)
        result = json.loads(m.list_tags())
        self.assertIn("tags", result)
        self.assertEqual(len(result["tags"]), 1)

    def test_go_to_tag_skill_found(self):
        m = self._make()
        m._store.tag("lab", x=3.0, y=4.0, z=0.0)
        result = json.loads(m.go_to_tag("lab"))
        self.assertTrue(result["success"])
        self.assertEqual(result["name"], "lab")

    def test_go_to_tag_skill_not_found(self):
        m = self._make()
        result = json.loads(m.go_to_tag("nowhere"))
        self.assertFalse(result["success"])


# ===========================================================================
# 5. TemporalMemoryModule
# ===========================================================================

class TestTemporalMemoryInstantiation(unittest.TestCase):
    """Test TemporalMemoryModule creation and initial state."""

    def _make(self, **kw):
        from memory.modules.temporal_memory_module import TemporalMemoryModule
        return TemporalMemoryModule(**kw)

    def test_default_creation(self):
        m = self._make()
        self.assertEqual(m._max_records, 1000)
        self.assertEqual(len(m._buffer), 0)

    def test_custom_max_records(self):
        m = self._make(max_records=50)
        self.assertEqual(m._max_records, 50)
        self.assertEqual(m._buffer.maxlen, 50)

    def test_custom_min_observation_interval(self):
        m = self._make(min_observation_interval=2.0)
        self.assertAlmostEqual(m._min_observation_interval, 2.0)

    def test_initial_buffer_empty(self):
        m = self._make()
        self.assertEqual(len(m._buffer), 0)

    def test_initial_summary_cache_empty(self):
        m = self._make()
        self.assertEqual(m._summary_cache, "")


class TestTemporalMemoryOperations(unittest.TestCase):
    """Test TemporalMemoryModule buffer operations and queries."""

    def _make(self, **kw):
        from memory.modules.temporal_memory_module import TemporalMemoryModule
        kw.setdefault("min_observation_interval", 0.0)
        m = TemporalMemoryModule(**kw)
        m.setup()
        return m

    def test_on_scene_graph_adds_record(self):
        m = self._make()
        m._on_odom(_odom(1.0, 2.0))
        m._on_scene_graph(_scene_graph(["chair"]))
        self.assertEqual(len(m._buffer), 1)

    def test_on_scene_graph_without_odom_uses_origin(self):
        m = self._make()
        m._on_scene_graph(_scene_graph(["chair"]))
        self.assertEqual(len(m._buffer), 1)
        self.assertEqual(m._buffer[0].position, (0.0, 0.0, 0.0))

    def test_min_observation_interval_dedup(self):
        m = self._make(min_observation_interval=10.0)
        m._on_scene_graph(_scene_graph(["chair"]))
        m._on_scene_graph(_scene_graph(["table"]))
        # Second one should be skipped due to interval
        self.assertEqual(len(m._buffer), 1)

    def test_deque_overflow_respects_maxlen(self):
        m = self._make(max_records=3)
        for i in range(5):
            m._last_record_ts = 0.0  # Reset to bypass interval check
            m._on_scene_graph(_scene_graph([f"obj_{i}"]))
        self.assertEqual(len(m._buffer), 3)

    def test_record_contains_objects(self):
        m = self._make()
        m._on_scene_graph(_scene_graph(["chair", "lamp"]))
        rec = m._buffer[0]
        labels = [o["label"] for o in rec.objects]
        self.assertIn("chair", labels)
        self.assertIn("lamp", labels)

    def test_record_position_from_odom(self):
        m = self._make()
        m._on_odom(_odom(3.0, 4.0, 1.0))
        m._on_scene_graph(_scene_graph(["box"]))
        self.assertEqual(m._buffer[0].position, (3.0, 4.0, 1.0))

    def test_query_by_label(self):
        m = self._make()
        m._on_scene_graph(_scene_graph(["chair"]))
        m._last_record_ts = 0.0
        m._on_scene_graph(_scene_graph(["table"]))
        results = m.query_by_label("chair")
        self.assertEqual(len(results), 1)

    def test_query_by_label_case_insensitive(self):
        m = self._make()
        m._on_scene_graph(_scene_graph(["Chair"]))
        results = m.query_by_label("chair")
        self.assertEqual(len(results), 1)

    def test_query_last_seen(self):
        m = self._make()
        m._on_odom(_odom(1.0, 1.0))
        m._on_scene_graph(_scene_graph(["person"]))
        m._last_record_ts = 0.0
        m._on_odom(_odom(5.0, 5.0))
        m._on_scene_graph(_scene_graph(["person", "dog"]))
        rec = m.query_last_seen("person")
        self.assertIsNotNone(rec)
        self.assertEqual(rec.position, (5.0, 5.0, 0.0))

    def test_query_last_seen_not_found(self):
        m = self._make()
        m._on_scene_graph(_scene_graph(["chair"]))
        rec = m.query_last_seen("elephant")
        self.assertIsNone(rec)

    def test_query_by_time(self):
        m = self._make()
        before = time.time()
        m._on_scene_graph(_scene_graph(["a"]))
        after = time.time()
        results = m.query_by_time(before, after)
        self.assertEqual(len(results), 1)

    def test_health_returns_buffer_size(self):
        m = self._make()
        m._on_scene_graph(_scene_graph(["x"]))
        h = m.health()
        self.assertEqual(h["buffer_size"], 1)

    def test_memory_context_published(self):
        m = self._make()
        published = []
        m.memory_context.subscribe(lambda s: published.append(s))
        m._on_scene_graph(_scene_graph(["monitor"]))
        self.assertEqual(len(published), 1)

    def test_thread_safety_concurrent_writes(self):
        m = self._make()
        errors = []

        def writer(offset):
            try:
                for i in range(20):
                    m._last_record_ts = 0.0
                    m._on_scene_graph(_scene_graph([f"obj_{offset}_{i}"]))
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=writer, args=(t,)) for t in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertEqual(len(errors), 0)
        # Buffer should have items (exact count depends on timing)
        self.assertGreater(len(m._buffer), 0)


class TestTemporalRecord(unittest.TestCase):
    """Test TemporalRecord data class."""

    def test_to_dict(self):
        from memory.modules.temporal_memory_module import TemporalRecord
        rec = TemporalRecord(
            timestamp=1000.0,
            position=(1.0, 2.0, 3.0),
            objects=[{"label": "chair", "confidence": 0.9, "x": 0, "y": 0, "z": 0}],
        )
        d = rec.to_dict()
        self.assertEqual(d["timestamp"], 1000.0)
        self.assertEqual(d["position"], [1.0, 2.0, 3.0])

    def test_from_dict_roundtrip(self):
        from memory.modules.temporal_memory_module import TemporalRecord
        original = TemporalRecord(
            timestamp=500.0,
            position=(10.0, 20.0, 0.0),
            objects=[{"label": "box", "confidence": 0.8, "x": 1, "y": 2, "z": 0}],
            room="kitchen",
        )
        d = original.to_dict()
        restored = TemporalRecord.from_dict(d)
        self.assertAlmostEqual(restored.timestamp, 500.0)
        self.assertEqual(restored.room, "kitchen")
        self.assertAlmostEqual(restored.position[0], 10.0)


# ===========================================================================
# 6. TopologicalMemoryModule
# ===========================================================================

class TestTopologicalInstantiation(unittest.TestCase):
    """Test TopologicalMemoryModule creation."""

    def _make(self, **kw):
        from memory.modules.topological_module import TopologicalMemoryModule
        return TopologicalMemoryModule(**kw)

    def test_default_creation(self):
        m = self._make()
        self.assertIsNotNone(m._memory)

    def test_memory_property_accessible(self):
        m = self._make()
        self.assertIs(m.memory, m._memory)

    def test_custom_new_node_distance(self):
        m = self._make(new_node_distance=5.0)
        self.assertAlmostEqual(m._memory.new_node_distance, 5.0)

    def test_custom_max_nodes(self):
        m = self._make(max_nodes=100)
        self.assertEqual(m._memory.max_nodes, 100)

    def test_initial_no_nodes(self):
        m = self._make()
        self.assertEqual(len(m._memory.nodes), 0)


class TestTopologicalOperations(unittest.TestCase):
    """Test TopologicalMemoryModule node creation and graph operations."""

    def _make(self, **kw):
        from memory.modules.topological_module import TopologicalMemoryModule
        kw.setdefault("new_node_distance", 1.0)
        m = TopologicalMemoryModule(**kw)
        m.setup()
        return m

    def test_update_creates_node(self):
        m = self._make()
        m._on_odom(_odom(0.0, 0.0, 0.0))
        m._on_scene_graph(_scene_graph(["chair"]))
        self.assertGreaterEqual(len(m._memory.nodes), 1)

    def test_nearby_positions_reuse_node(self):
        m = self._make(new_node_distance=5.0)
        m._on_odom(_odom(0.0, 0.0))
        m._on_scene_graph(_scene_graph(["a"]))
        m._on_odom(_odom(0.5, 0.5))
        m._on_scene_graph(_scene_graph(["b"]))
        self.assertEqual(len(m._memory.nodes), 1)

    def test_distant_positions_create_new_nodes(self):
        m = self._make(new_node_distance=1.0)
        m._on_odom(_odom(0.0, 0.0))
        m._on_scene_graph(_scene_graph(["a"]))
        m._on_odom(_odom(10.0, 10.0))
        m._on_scene_graph(_scene_graph(["b"]))
        self.assertGreaterEqual(len(m._memory.nodes), 2)

    def test_edge_created_between_nodes(self):
        m = self._make(new_node_distance=1.0)
        m._on_odom(_odom(0.0, 0.0))
        m._on_scene_graph(_scene_graph(["a"]))
        m._on_odom(_odom(10.0, 0.0))
        m._on_scene_graph(_scene_graph(["b"]))
        # Check that the second node has the first as a neighbor
        nodes = m._memory.nodes
        if len(nodes) >= 2:
            has_edge = any(len(n.neighbors) > 0 for n in nodes.values())
            self.assertTrue(has_edge)

    def test_visible_labels_stored(self):
        m = self._make()
        m._on_odom(_odom(0.0, 0.0))
        m._on_scene_graph(_scene_graph(["monitor", "keyboard"]))
        nodes = m._memory.nodes
        self.assertGreater(len(nodes), 0)
        first_node = next(iter(nodes.values()))
        self.assertIn("monitor", first_node.visible_labels)

    def test_topo_summary_published(self):
        m = self._make()
        published = []
        m.topo_summary.subscribe(lambda s: published.append(s))
        m._on_odom(_odom(0.0, 0.0))
        m._on_scene_graph(_scene_graph(["desk"]))
        self.assertGreaterEqual(len(published), 1)

    def test_topo_graph_published(self):
        m = self._make()
        published = []
        m.topo_graph.subscribe(lambda d: published.append(d))
        m._on_odom(_odom(0.0, 0.0))
        m._on_scene_graph(_scene_graph(["lamp"]))
        self.assertGreaterEqual(len(published), 1)
        self.assertIsInstance(published[0], dict)

    def test_health_returns_node_edge_count(self):
        m = self._make()
        m._on_odom(_odom(0.0, 0.0))
        m._on_scene_graph(_scene_graph(["a"]))
        h = m.health()
        self.assertIn("node_count", h)
        self.assertIn("edge_count", h)
        self.assertGreaterEqual(h["node_count"], 1)

    def test_no_update_without_odom(self):
        m = self._make()
        m._on_scene_graph(_scene_graph(["a"]))
        # Without odom, _update returns early
        self.assertEqual(len(m._memory.nodes), 0)

    def test_graph_consistency_multiple_updates(self):
        m = self._make(new_node_distance=1.0)
        positions = [(0, 0), (5, 0), (10, 0), (10, 5), (5, 5)]
        for x, y in positions:
            m._on_odom(_odom(float(x), float(y)))
            m._on_scene_graph(_scene_graph(["obj"]))
        nodes = m._memory.nodes
        # Should have created multiple nodes
        self.assertGreaterEqual(len(nodes), 3)
        # Each node should have valid positions
        for node in nodes.values():
            self.assertTrue(np.isfinite(node.position).all())


if __name__ == "__main__":
    unittest.main()

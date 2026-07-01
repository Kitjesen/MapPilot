"""Focused tests for SemanticMapperModule — KG + TSG update pipeline.

Pure unit tests -- no ROS2, no hardware, no external services.
Follows patterns from test_memory_modules.py.
"""

from __future__ import annotations

import os
import tempfile
import unittest
from unittest.mock import MagicMock, patch

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
# 1. Instantiation & Ports
# ===========================================================================

class TestSemanticMapperInstantiation(unittest.TestCase):
    """Test creation and configuration defaults."""

    def _make(self, **kw):
        from memory.modules.semantic_mapper_module import SemanticMapperModule
        return SemanticMapperModule(**kw)

    def test_default_save_dir(self):
        m = self._make()
        self.assertTrue(m._save_dir.endswith("semantic"))

    def test_custom_save_dir(self):
        m = self._make(save_dir="/tmp/custom_smap")
        self.assertEqual(m._save_dir, "/tmp/custom_smap")

    def test_default_save_interval(self):
        m = self._make()
        self.assertAlmostEqual(m._save_interval, 30.0)

    def test_custom_save_interval(self):
        m = self._make(save_interval_s=60.0)
        self.assertAlmostEqual(m._save_interval, 60.0)

    def test_default_min_obs(self):
        m = self._make()
        self.assertEqual(m._min_obs, 1)

    def test_custom_min_obs(self):
        m = self._make(min_observations_for_commit=3)
        self.assertEqual(m._min_obs, 3)


class TestSemanticMapperPorts(unittest.TestCase):
    """Test that In/Out ports exist with correct types."""

    def _make(self, **kw):
        from memory.modules.semantic_mapper_module import SemanticMapperModule
        return SemanticMapperModule(**kw)

    def test_input_ports_exist(self):
        from runtime.stream import In
        m = self._make()
        self.assertIsInstance(m.scene_graph, In)
        self.assertIsInstance(m.odometry, In)

    def test_output_ports_exist(self):
        from runtime.stream import Out
        m = self._make()
        self.assertIsInstance(m.topo_summary, Out)
        self.assertIsInstance(m.room_graph, Out)

    def test_port_names(self):
        m = self._make()
        self.assertEqual(m.scene_graph.name, "scene_graph")
        self.assertEqual(m.odometry.name, "odometry")
        self.assertEqual(m.topo_summary.name, "topo_summary")
        self.assertEqual(m.room_graph.name, "room_graph")


# ===========================================================================
# 2. Setup & State
# ===========================================================================

class TestSemanticMapperSetup(unittest.TestCase):
    """Test setup and basic state."""

    def _make(self, **kw):
        from memory.modules.semantic_mapper_module import SemanticMapperModule
        return SemanticMapperModule(**kw)

    @patch("memory.modules.semantic_mapper_module.SemanticMapperModule._init_backends")
    def test_setup_subscribes(self, mock_init):
        m = self._make()
        m.setup()
        mock_init.assert_called_once()

    def test_initial_state(self):
        m = self._make()
        self.assertIsNone(m._kg)
        self.assertIsNone(m._tsg)
        self.assertEqual(m._robot_xy, (0.0, 0.0))
        self.assertEqual(m._sg_count, 0)
        self.assertEqual(m._room_name_to_id, {})

    def test_stable_room_id_deterministic(self):
        m = self._make()
        id1 = m._stable_room_id("kitchen")
        id2 = m._stable_room_id("kitchen")
        self.assertEqual(id1, id2)

    def test_stable_room_id_increments(self):
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


# ===========================================================================
# 3. Scene Graph Processing
# ===========================================================================

class TestSemanticMapperProcessing(unittest.TestCase):
    """Test KG/TSG updates from scene graph events."""

    def _make(self, **kw):
        from memory.modules.semantic_mapper_module import SemanticMapperModule
        return SemanticMapperModule(**kw)

    def test_empty_regions_skips(self):
        m = self._make()
        m._kg = None
        m._tsg = None
        sg = _scene_graph(["chair"], region_name="")
        sg.regions = []
        m._on_scene_graph(sg)
        self.assertEqual(m._sg_count, 0)

    def test_on_scene_graph_increments_count(self):
        m = self._make()
        m._kg = MagicMock()
        m._tsg = MagicMock()
        m._tsg.to_prompt_context.return_value = "summary text"
        m._tsg.rooms = []
        m._tsg.to_dict.return_value = {}
        m._tsg.current_room_id = -1
        m._tsg.frontiers = []
        sg = _scene_graph(["chair", "table"], region_name="kitchen")
        m._on_scene_graph(sg)
        self.assertEqual(m._sg_count, 1)

    def test_extract_objects(self):
        m = self._make()
        sg = _scene_graph(["chair", "table"])
        labels, confs = m._extract_objects(sg, ["obj_0", "obj_1"])
        self.assertEqual(labels, ["chair", "table"])
        self.assertEqual(len(confs), 2)

    def test_extract_objects_missing_id(self):
        m = self._make()
        sg = _scene_graph(["chair"])
        labels, confs = m._extract_objects(sg, ["nonexistent"])
        self.assertEqual(labels, [])
        self.assertEqual(confs, [])

    def test_update_kg_skipped_without_kg(self):
        m = self._make()
        m._kg = None
        sg = _scene_graph(["chair"], region_name="kitchen")
        # Should not raise
        m._update_kg(sg)

    def test_update_tsg_skipped_without_tsg(self):
        m = self._make()
        m._tsg = None
        sg = _scene_graph(["chair"], region_name="kitchen")
        # Should not raise
        m._update_tsg(sg)


# ===========================================================================
# 4. Observation Gate & Dirichlet-Multinomial Posterior
# ===========================================================================

class TestSemanticMapperObservationGate(unittest.TestCase):
    """Test _min_obs gating and Dirichlet-Multinomial posterior."""

    def _make(self, **kw):
        from memory.modules.semantic_mapper_module import SemanticMapperModule
        return SemanticMapperModule(**kw)

    def test_min_obs_gate_blocks_below_threshold(self):
        m = self._make(min_observations_for_commit=3)
        m._kg = MagicMock()
        sg = _scene_graph(["chair"], region_name="kitchen")
        # First two observations should not commit
        m._update_kg(sg)
        self.assertEqual(m._obs_counts.get(("kitchen", "chair")), 1)
        m._kg.observe_room.assert_not_called()
        m._update_kg(sg)
        self.assertEqual(m._obs_counts.get(("kitchen", "chair")), 2)
        m._kg.observe_room.assert_not_called()

    def test_min_obs_gate_commits_at_threshold(self):
        m = self._make(min_observations_for_commit=3)
        m._kg = MagicMock()
        sg = _scene_graph(["chair"], region_name="kitchen")
        for _ in range(3):
            m._update_kg(sg)
        m._kg.observe_room.assert_called_once()

    def test_min_obs_1_commits_immediately(self):
        m = self._make(min_observations_for_commit=1)
        m._kg = MagicMock()
        sg = _scene_graph(["chair"], region_name="kitchen")
        m._update_kg(sg)
        m._kg.observe_room.assert_called_once_with("kitchen", ["chair"], [0.9])

    def test_dm_counts_updated_unconditionally(self):
        m = self._make(min_observations_for_commit=5)
        m._kg = MagicMock()
        sg = _scene_graph(["chair"], region_name="kitchen")
        m._update_kg(sg)
        self.assertEqual(m._dm_counts["kitchen"]["chair"], 1)
        m._kg.observe_room.assert_not_called()

    def test_get_posterior_unseen_room(self):
        m = self._make()
        self.assertEqual(m.get_posterior("unknown", "chair"), 0.0)

    def test_get_posterior_unseen_label(self):
        m = self._make()
        m._dm_counts["kitchen"] = {"chair": 5, "table": 3}
        prob = m.get_posterior("kitchen", "oven")
        # alpha=1.0, K=2, total=8
        # P = (0 + 1) / (8 + 1*2) = 1/10 = 0.1
        self.assertAlmostEqual(prob, 0.1)

    def test_get_posterior_exact(self):
        m = self._make()
        m._dm_counts["kitchen"] = {"chair": 5, "table": 3}
        prob = m.get_posterior("kitchen", "chair")
        # P = (5 + 1) / (8 + 1*2) = 6/10 = 0.6
        self.assertAlmostEqual(prob, 0.6)


# ===========================================================================
# 5. Skill Methods
# ===========================================================================

class TestSemanticMapperSkills(unittest.TestCase):
    """Test @skill-exposed methods."""

    def _make(self, **kw):
        from memory.modules.semantic_mapper_module import SemanticMapperModule
        return SemanticMapperModule(**kw)

    def test_get_room_summary_no_tsg(self):
        m = self._make()
        m._tsg = None
        result = m.get_room_summary()
        self.assertEqual(result, "No rooms mapped yet.")

    def test_get_room_summary_no_rooms(self):
        m = self._make()
        m._tsg = MagicMock()
        m._tsg.rooms = []
        result = m.get_room_summary()
        self.assertEqual(result, "No rooms mapped yet.")

    def test_query_room_for_object_no_kg(self):
        m = self._make()
        m._kg = None
        result = m.query_room_for_object("chair")
        self.assertEqual(result["source"], "no_data")

    def test_query_room_for_object_empty_kg(self):
        m = self._make()
        m._kg = MagicMock()
        m._kg.is_empty = True
        result = m.query_room_for_object("chair")
        self.assertEqual(result["source"], "no_data")

    def test_get_exploration_target_no_tsg(self):
        m = self._make()
        m._tsg = None
        result = m.get_exploration_target("go explore")
        self.assertIsNone(result["target"])
        self.assertIn("no topology", result["reason"])

    def test_get_semantic_status_no_kg(self):
        m = self._make()
        m._kg = None
        result = m.get_semantic_status()
        self.assertEqual(result["kg"], {})

    def test_get_semantic_status_with_kg(self):
        m = self._make()
        m._kg = MagicMock()
        m._kg.get_stats.return_value = {"rooms": 2, "objects": 5}
        m._tsg = MagicMock()
        m._tsg.rooms = [MagicMock(), MagicMock()]
        m._tsg.frontiers = [MagicMock()]
        m._tsg.current_room_id = 0
        dummy_room = MagicMock()
        dummy_room.name = "kitchen"
        m._tsg.get_node.return_value = dummy_room
        result = m.get_semantic_status()
        self.assertEqual(result["kg"]["rooms"], 2)
        self.assertEqual(result["tsg"]["rooms"], 2)
        self.assertEqual(result["tsg"]["current_room"], "kitchen")


# ===========================================================================
# 6. Persistence
# ===========================================================================

class TestSemanticMapperPersistence(unittest.TestCase):
    """Test save and load with temporary directory."""

    def _make(self, **kw):
        from memory.modules.semantic_mapper_module import SemanticMapperModule
        return SemanticMapperModule(**kw)

    def test_kg_path(self):
        m = self._make(save_dir="/tmp/test_smap")
        self.assertEqual(m._kg_path(), os.path.join("/tmp/test_smap", "room_object_kg.json"))

    def test_tsg_path(self):
        m = self._make(save_dir="/tmp/test_smap")
        self.assertEqual(m._tsg_path(), os.path.join("/tmp/test_smap", "topology_graph.json"))

    def test_save_now_creates_dir(self):
        with tempfile.TemporaryDirectory() as td:
            sub = os.path.join(td, "nested", "dir")
            m = self._make(save_dir=sub)
            m._kg = MagicMock()
            m._kg.is_empty = False
            tsg_mock = MagicMock()
            tsg_mock.rooms = [MagicMock()]
            m._tsg = tsg_mock
            m._save_now()
            self.assertTrue(os.path.isdir(sub))

    def test_save_now_no_kg(self):
        with tempfile.TemporaryDirectory() as td:
            m = self._make(save_dir=td)
            m._kg = None
            m._tsg = None
            m._save_now()
            # Should not raise, no files written
            self.assertFalse(os.path.exists(m._kg_path()))
            self.assertFalse(os.path.exists(m._tsg_path()))

    def test_load_from_dir_empty_dir_returns_true(self):
        m = self._make()
        with tempfile.TemporaryDirectory() as td:
            result = m.load_from_dir(td)
            # No files exist but load_from_dir returns True (no error, just empty)
            self.assertTrue(result)

    def test_health(self):
        m = self._make()
        h = m.health()
        self.assertIn("semantic_mapper", h)
        self.assertEqual(h["semantic_mapper"]["sg_updates"], 0)


if __name__ == "__main__":
    unittest.main()

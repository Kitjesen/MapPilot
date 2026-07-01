"""Focused tests for EpisodicMemoryModule — spatio-temporal memory recording.

Pure unit tests -- no ROS2, no hardware, no external services.
Follows patterns from test_memory_modules.py.
"""

from __future__ import annotations

import json
import unittest

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

class TestEpisodicMemoryInstantiation(unittest.TestCase):
    """Test EpisodicMemoryModule creation and configuration."""

    def _make(self, **kw):
        from memory.modules.episodic_module import EpisodicMemoryModule
        return EpisodicMemoryModule(**kw)

    def test_default_creation(self):
        m = self._make()
        self.assertIsNotNone(m._memory)
        self.assertEqual(m._memory.MAX_RECORDS, 500)
        self.assertAlmostEqual(m._memory.MIN_DISTANCE_M, 1.0)

    def test_custom_max_records(self):
        m = self._make(max_records=100)
        self.assertEqual(m._memory.MAX_RECORDS, 100)

    def test_custom_min_distance(self):
        m = self._make(min_distance_m=5.0)
        self.assertAlmostEqual(m._memory.MIN_DISTANCE_M, 5.0)

    def test_initial_record_count_zero(self):
        m = self._make()
        self.assertEqual(len(m._memory), 0)


class TestEpisodicMemoryPorts(unittest.TestCase):
    """Test that In/Out ports exist with correct types."""

    def _make(self, **kw):
        from memory.modules.episodic_module import EpisodicMemoryModule
        return EpisodicMemoryModule(**kw)

    def test_input_ports_exist(self):
        from runtime.stream import In
        m = self._make()
        self.assertIsInstance(m.scene_graph, In)
        self.assertIsInstance(m.odometry, In)

    def test_output_ports_exist(self):
        from runtime.stream import Out
        m = self._make()
        self.assertIsInstance(m.memory_context, Out)

    def test_port_names(self):
        m = self._make()
        self.assertEqual(m.scene_graph.name, "scene_graph")
        self.assertEqual(m.odometry.name, "odometry")
        self.assertEqual(m.memory_context.name, "memory_context")


# ===========================================================================
# 2. Setup & Memory Property
# ===========================================================================

class TestEpisodicMemorySetup(unittest.TestCase):
    """Test lifecycle and property access."""

    def _make(self, **kw):
        from memory.modules.episodic_module import EpisodicMemoryModule
        return EpisodicMemoryModule(**kw)

    def test_setup_runs_without_error(self):
        m = self._make()
        # setup subscribes ports; verify it doesn't raise
        m.setup()
        # After setup, the module should be ready to record
        self.assertIsNotNone(m._memory)

    def test_memory_property_accessible(self):
        m = self._make()
        self.assertIs(m.memory, m._memory)

    def test_health_empty(self):
        m = self._make()
        h = m.health()
        self.assertEqual(h["record_count"], 0)
        self.assertIsNone(h["oldest_ts"])


# ===========================================================================
# 3. Scene Graph Recording
# ===========================================================================

class TestEpisodicMemoryRecording(unittest.TestCase):
    """Test recording from scene graph events."""

    def _make(self, **kw):
        from memory.modules.episodic_module import EpisodicMemoryModule
        m = EpisodicMemoryModule(**kw)
        m.setup()
        return m

    def test_on_scene_graph_without_odom_skips(self):
        m = self._make()
        sg = _scene_graph(["chair"])
        m._on_scene_graph(sg)
        self.assertEqual(len(m._memory), 0)

    def test_record_added_with_odom(self):
        m = self._make()
        m._on_odom(_odom(1.0, 2.0, 0.0))
        sg = _scene_graph(["chair", "table"])
        m._on_scene_graph(sg)
        self.assertEqual(len(m._memory), 1)

    def test_multiple_labels_recorded(self):
        m = self._make()
        m._on_odom(_odom(1.0, 2.0, 0.0))
        m._on_scene_graph(_scene_graph(["monitor", "keyboard", "mouse"]))
        self.assertEqual(len(m._memory), 1)
        record = m._memory.recent_n(1)[0]
        self.assertIn("monitor", record.labels)
        self.assertIn("keyboard", record.labels)
        self.assertIn("mouse", record.labels)

    def test_position_dedup(self):
        m = self._make(min_distance_m=2.0)
        m._on_odom(_odom(1.0, 1.0))
        m._on_scene_graph(_scene_graph(["chair"]))
        m._on_odom(_odom(1.5, 1.5))
        m._on_scene_graph(_scene_graph(["table"]))
        self.assertEqual(len(m._memory), 1)

    def test_record_added_after_sufficient_movement(self):
        m = self._make(min_distance_m=1.0)
        m._on_odom(_odom(0.0, 0.0))
        m._on_scene_graph(_scene_graph(["chair"]))
        m._on_odom(_odom(5.0, 5.0))
        m._on_scene_graph(_scene_graph(["table"]))
        self.assertEqual(len(m._memory), 2)

    def test_memory_context_published(self):
        m = self._make()
        published = []
        m.memory_context.subscribe(lambda s: published.append(s))
        m._on_odom(_odom(1.0, 2.0))
        m._on_scene_graph(_scene_graph(["chair"]))
        self.assertEqual(len(published), 1)
        self.assertIsInstance(published[0], str)

    def test_health_with_record(self):
        m = self._make()
        m._on_odom(_odom(1.0, 2.0))
        m._on_scene_graph(_scene_graph(["chair"]))
        h = m.health()
        self.assertEqual(h["record_count"], 1)
        self.assertIsNotNone(h["oldest_ts"])

    def test_time_ordering(self):
        m = self._make(min_distance_m=0.1)
        for i in range(3):
            m._on_odom(_odom(float(i * 10), 0.0))
            m._on_scene_graph(_scene_graph([f"obj_{i}"]))
        records = m._memory.recent_n(3)
        for j in range(len(records) - 1):
            self.assertLessEqual(records[j].timestamp, records[j + 1].timestamp)


# ===========================================================================
# 4. Skill Methods
# ===========================================================================

class TestEpisodicMemorySkills(unittest.TestCase):
    """Test @skill-exposed methods."""

    def _make(self, **kw):
        from memory.modules.episodic_module import EpisodicMemoryModule
        m = EpisodicMemoryModule(**kw)
        m.setup()
        return m

    def test_get_recent_observations_empty(self):
        m = self._make()
        result = json.loads(m.get_recent_observations(5))
        self.assertIn("observations", result)
        self.assertEqual(len(result["observations"]), 0)

    def test_get_recent_observations_with_data(self):
        m = self._make(min_distance_m=0.1)
        m._on_odom(_odom(0.0, 0.0))
        m._on_scene_graph(_scene_graph(["chair"]))
        m._on_odom(_odom(10.0, 10.0))
        m._on_scene_graph(_scene_graph(["desk"]))
        result = json.loads(m.get_recent_observations(5))
        self.assertIn("observations", result)
        self.assertEqual(len(result["observations"]), 2)

    def test_get_recent_observations_returns_labels(self):
        m = self._make(min_distance_m=0.1)
        m._on_odom(_odom(0.0, 0.0))
        m._on_scene_graph(_scene_graph(["laptop"]))
        result = json.loads(m.get_recent_observations(5))
        self.assertEqual(result["observations"][0]["labels"], ["laptop"])

    def test_get_recent_observations_returns_position(self):
        m = self._make(min_distance_m=0.1)
        m._on_odom(_odom(3.0, 4.0, 1.0))
        m._on_scene_graph(_scene_graph(["box"]))
        result = json.loads(m.get_recent_observations(5))
        self.assertAlmostEqual(result["observations"][0]["position"][0], 3.0)
        self.assertAlmostEqual(result["observations"][0]["position"][1], 4.0)


if __name__ == "__main__":
    unittest.main()

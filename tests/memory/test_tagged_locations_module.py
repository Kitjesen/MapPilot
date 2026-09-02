"""Focused tests for TaggedLocationsModule — named location tag CRUD.

Pure unit tests -- no ROS2, no hardware, no external services.
Follows patterns from test_memory_modules.py.
"""

from __future__ import annotations

import json
import os
import tempfile
import unittest

from runtime.msgs.nav import Odometry, Pose

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _odom(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Odometry:
    return Odometry(pose=Pose(x, y, z))


# ===========================================================================
# 1. Instantiation & Ports
# ===========================================================================

class TestTaggedLocationsInstantiation(unittest.TestCase):
    """Test TaggedLocationsModule creation and configuration."""

    def _make(self, **kw):
        from memory.modules.tagged_locations_module import TaggedLocationsModule
        return TaggedLocationsModule(**kw)

    def test_default_creation(self):
        m = self._make()
        self.assertIsNotNone(m._store)

    def test_custom_json_path(self):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
            path = f.name
        try:
            m = self._make(json_path=path)
            self.assertEqual(m._store._path, path)
        finally:
            os.unlink(path)

    def test_store_property_accessible(self):
        m = self._make()
        self.assertIs(m.store, m._store)

    def test_initial_no_tags(self):
        m = self._make()
        self.assertEqual(len(m._store.list_all()), 0)


class TestTaggedLocationsPorts(unittest.TestCase):
    """Test that In/Out ports exist with correct types."""

    def _make(self, **kw):
        from memory.modules.tagged_locations_module import TaggedLocationsModule
        return TaggedLocationsModule(**kw)

    def test_input_ports_exist(self):
        from runtime.stream import In
        m = self._make()
        self.assertIsInstance(m.odometry, In)
        self.assertIsInstance(m.tag_command, In)

    def test_output_ports_exist(self):
        from runtime.stream import Out
        m = self._make()
        self.assertIsInstance(m.saved_location, Out)
        self.assertIsInstance(m.tag_status, Out)

    def test_port_names(self):
        m = self._make()
        self.assertEqual(m.odometry.name, "odometry")
        self.assertEqual(m.tag_command.name, "tag_command")
        self.assertEqual(m.saved_location.name, "saved_location")
        self.assertEqual(m.tag_status.name, "tag_status")


# ===========================================================================
# 2. Setup & Initial State
# ===========================================================================

class TestTaggedLocationsSetup(unittest.TestCase):
    """Test setup and initial state."""

    def _make(self, **kw):
        from memory.modules.tagged_locations_module import TaggedLocationsModule
        return TaggedLocationsModule(**kw)

    def test_setup_runs_without_error(self):
        m = self._make()
        # setup subscribes ports; verify it doesn't raise
        m.setup()
        # After setup, the module should be ready to process commands
        self.assertIsNotNone(m._store)

    def test_health_empty(self):
        m = self._make()
        h = m.health()
        self.assertEqual(h["location_count"], 0)

    def test_health_with_tags(self):
        m = self._make()
        m._store.tag("home", x=0.0, y=0.0)
        h = m.health()
        self.assertEqual(h["location_count"], 1)


# ===========================================================================
# 3. Tag CRUD Operations
# ===========================================================================

class TestTaggedLocationsCRUD(unittest.TestCase):
    """Test save/goto/remove/list operations."""

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


# ===========================================================================
# 4. Skill Methods
# ===========================================================================

class TestTaggedLocationsSkills(unittest.TestCase):
    """Test @skill-exposed methods."""

    def _make(self, **kw):
        from memory.modules.tagged_locations_module import TaggedLocationsModule
        m = TaggedLocationsModule(**kw)
        m.setup()
        return m

    def test_list_tags_skill(self):
        m = self._make()
        m._store.tag("cafe", x=1.0, y=2.0)
        result = json.loads(m.list_tags())
        self.assertIn("tags", result)
        self.assertEqual(len(result["tags"]), 1)

    def test_list_tags_skill_empty(self):
        m = self._make()
        result = json.loads(m.list_tags())
        self.assertIn("tags", result)
        self.assertEqual(len(result["tags"]), 0)

    def test_go_to_tag_skill_found(self):
        m = self._make()
        m._store.tag("lab", x=3.0, y=4.0, z=0.0)
        result = json.loads(m.go_to_tag("lab"))
        self.assertTrue(result["success"])
        self.assertEqual(result["name"], "lab")

    def test_go_to_tag_skill_found_with_position(self):
        m = self._make()
        m._store.tag("dock", x=7.0, y=8.0, z=0.5)
        result = json.loads(m.go_to_tag("dock"))
        self.assertTrue(result["success"])
        self.assertAlmostEqual(result["position"][0], 7.0)
        self.assertAlmostEqual(result["position"][1], 8.0)

    def test_go_to_tag_skill_not_found(self):
        m = self._make()
        result = json.loads(m.go_to_tag("nowhere"))
        self.assertFalse(result["success"])
        self.assertIn("not found", result["error"])

    def test_go_to_tag_skill_empty_name(self):
        m = self._make()
        result = json.loads(m.go_to_tag(""))
        self.assertFalse(result["success"])
        self.assertIn("empty", result["error"])


# ===========================================================================
# 5. Persistence
# ===========================================================================

class TestTaggedLocationsPersistence(unittest.TestCase):
    """Test JSON persistence with temporary files."""

    def test_json_persistence(self):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
            path = f.name
        try:
            from memory.modules.tagged_locations_module import TaggedLocationsModule
            m = TaggedLocationsModule(json_path=path)
            m.setup()
            m._on_odom(_odom(7.0, 8.0, 0.0))
            m._on_command("save:park")
            m._store.save()

            from memory.spatial.tagged_locations import TaggedLocationStore
            store2 = TaggedLocationStore(json_path=path)
            entry = store2.query("park")
            self.assertIsNotNone(entry)
            self.assertAlmostEqual(entry["position"][0], 7.0)
        finally:
            if os.path.exists(path):
                os.unlink(path)

    def test_multiple_tags_persist(self):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
            path = f.name
        try:
            from memory.modules.tagged_locations_module import TaggedLocationsModule
            m = TaggedLocationsModule(json_path=path)
            m.setup()
            m._store.tag("a", x=1.0, y=0.0)
            m._store.tag("b", x=2.0, y=0.0)
            m._store.save()

            from memory.spatial.tagged_locations import TaggedLocationStore
            store2 = TaggedLocationStore(json_path=path)
            self.assertEqual(len(store2.list_all()), 2)
        finally:
            if os.path.exists(path):
                os.unlink(path)


if __name__ == "__main__":
    unittest.main()

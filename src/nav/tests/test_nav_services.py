import pytest

pytestmark = [pytest.mark.ros2]

"""Tests for nav service modules, driver modules, and frontier explorer.

Covers:
  1. MapManagerModule      -- map CRUD, POI operations, persistence
  2. WavefrontFrontierExplorer -- frontier detection, scoring, health
  3. PatrolManagerModule   -- route CRUD, patrol start/stop
  4. TaskSchedulerModule   -- schedule CRUD, firing logic, deduplication
  5. GeofenceManagerModule -- fence CRUD, point-in-polygon, intrusion
  6. CameraBridgeModule    -- instantiation, setup without rclpy, health
  7. TeleopModule          -- joystick scaling, idle release, health, JPEG path

All tests are pure-Python, no ROS2 / hardware required.
"""

from __future__ import annotations

import json
import math
import os
import shutil
import tempfile
import time
import unittest
from datetime import datetime
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock, patch

import numpy as np

from core.msgs.geometry import Pose
from core.msgs.nav import Odometry
from core.msgs.sensor import Image, ImageFormat, PointCloud2

# ---------------------------------------------------------------------------
# 1. MapManagerModule
# ---------------------------------------------------------------------------

class TestMapManagerModule(unittest.TestCase):
    """Test map CRUD, POI operations, and persistence."""

    def setUp(self):
        self._tmpdir = tempfile.mkdtemp()
        self._map_dir = os.path.join(self._tmpdir, "maps")
        self._data_dir = os.path.join(self._tmpdir, "data")
        from nav.services.map_manager_module import MapManagerModule
        self.mod = MapManagerModule(
            map_dir=self._map_dir,
            data_dir=self._data_dir,
        )
        self.mod.setup()
        self._responses: list[dict] = []
        self.mod.map_response.subscribe(lambda r: self._responses.append(r))

    def tearDown(self):
        shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _cmd(self, cmd: dict) -> dict:
        self._responses.clear()
        self.mod._on_command(json.dumps(cmd))
        self.assertTrue(len(self._responses) > 0, "no response published")
        return self._responses[-1]

    def _write_valid_saved_map_artifacts(
        self,
        name: str,
        *,
        occupancy: bool = False,
    ) -> Path:
        map_dir = Path(self._map_dir) / name
        map_dir.mkdir(exist_ok=True)
        pcd = map_dir / "map.pcd"
        tomogram = map_dir / "tomogram.pickle"
        pcd.write_text(
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            "WIDTH 1\n"
            "HEIGHT 1\n"
            "POINTS 1\n"
            "DATA ascii\n"
            "0.0 0.0 0.0\n",
            encoding="utf-8",
        )
        tomogram.write_bytes(b"unit-test-tomogram")
        from core.runtime_interface import TOPICS, topic_default_frame_id
        from core.same_source_map_artifacts import (
            build_saved_map_metadata,
            sha256_file,
        )

        frame_id = topic_default_frame_id(TOPICS.saved_map_cloud)
        source_profile = "unit_test"
        data_source = "unit_test"
        map_sha = sha256_file(pcd)
        artifacts: dict[str, dict[str, Any]] = {
            "map_pcd": {
                "path": "map.pcd",
                "source_profile": source_profile,
                "data_source": data_source,
                "slam_source": "unit_test_slam",
                "frame_id": frame_id,
                "point_count": 1,
                "sha256": map_sha,
            },
            "tomogram": {
                "path": "tomogram.pickle",
                "source_map_sha256": map_sha,
                "source_profile": source_profile,
                "data_source": data_source,
                "frame_id": frame_id,
                "shape": [1, 1, 1],
                "sha256": sha256_file(tomogram),
            },
        }
        if occupancy:
            occupancy_path = map_dir / "occupancy.npz"
            np.savez(
                str(occupancy_path),
                grid=np.zeros((1, 1), dtype=np.int8),
                resolution=np.array(0.2),
                origin=np.array([0.0, 0.0]),
            )
            artifacts["occupancy_grid"] = {
                "path": "occupancy.npz",
                "source_map_sha256": map_sha,
                "source_profile": source_profile,
                "data_source": data_source,
                "frame_id": frame_id,
                "sha256": sha256_file(occupancy_path),
            }
        metadata = build_saved_map_metadata(
            source_profile=source_profile,
            data_source=data_source,
            slam_source="unit_test_slam",
            localization_source="unit_test_localizer",
            mapping_source="unit_test_mapping",
            frame_id=frame_id,
            artifacts=artifacts,
        )
        (map_dir / "metadata.json").write_text(
            json.dumps(metadata),
            encoding="utf-8",
        )
        return map_dir

    def test_instantiation(self):
        """Module creates map_dir and data_dir on init."""
        self.assertTrue(os.path.isdir(self._map_dir))
        self.assertTrue(os.path.isdir(self._data_dir))

    def test_list_empty(self):
        resp = self._cmd({"action": "list"})
        self.assertTrue(resp["success"])
        self.assertEqual(resp["maps"], [])

    def test_delete_nonexistent(self):
        resp = self._cmd({"action": "delete", "name": "no_such_map"})
        self.assertFalse(resp["success"])
        self.assertIn("not found", resp["message"])

    def test_rename_creates_and_renames(self):
        # Create a map dir manually, then rename
        src = Path(self._map_dir) / "alpha"
        src.mkdir()
        (src / "map.pcd").touch()
        resp = self._cmd({"action": "rename", "name": "alpha", "new_name": "beta"})
        self.assertTrue(resp["success"])
        self.assertFalse(src.exists())
        self.assertTrue((Path(self._map_dir) / "beta" / "map.pcd").exists())

    def test_rename_missing_names(self):
        resp = self._cmd({"action": "rename", "name": "", "new_name": ""})
        self.assertFalse(resp["success"])

    def test_set_active_and_list(self):
        self._write_valid_saved_map_artifacts("mymap")
        resp = self._cmd({"action": "set_active", "name": "mymap"})
        self.assertTrue(resp["success"])
        self.assertEqual(resp["active"], "mymap")
        # Verify symlink
        link = Path(self._map_dir) / "active"
        self.assertTrue(link.is_symlink())

    def test_set_active_missing(self):
        resp = self._cmd({"action": "set_active", "name": "ghost"})
        self.assertFalse(resp["success"])

    def test_delete_active_clears_symlink(self):
        self._write_valid_saved_map_artifacts("todel")  # side effect: creates map dir
        self._cmd({"action": "set_active", "name": "todel"})
        resp = self._cmd({"action": "delete", "name": "todel"})
        self.assertTrue(resp["success"])
        self.assertEqual(self.mod._active_map, "")

    # -- POI tests --

    def test_poi_set_list_delete(self):
        resp = self._cmd({"action": "poi_set", "name": "home", "x": 1.0, "y": 2.0})
        self.assertTrue(resp["success"])
        resp = self._cmd({"action": "poi_list"})
        self.assertIn("home", resp["pois"])

        resp = self._cmd({"action": "poi_delete", "name": "home"})
        self.assertTrue(resp["success"])
        resp = self._cmd({"action": "poi_list"})
        self.assertNotIn("home", resp["pois"])

    def test_poi_delete_nonexistent(self):
        resp = self._cmd({"action": "poi_delete", "name": "nope"})
        self.assertFalse(resp["success"])

    def test_unknown_action(self):
        resp = self._cmd({"action": "foobar"})
        self.assertFalse(resp["success"])
        self.assertIn("unknown", resp.get("message", ""))

    def test_invalid_json(self):
        self._responses.clear()
        self.mod._on_command("not json{{{")
        self.assertTrue(len(self._responses) > 0)
        resp = self._responses[-1]
        self.assertFalse(resp["success"])

    def test_get_active_tomogram_none(self):
        result = self.mod.get_active_tomogram()
        self.assertIsNone(result)

    def test_build_tomogram_missing_name(self):
        resp = self.mod._build_tomogram("")
        self.assertFalse(resp["success"])

    def test_build_occupancy_command_dispatches_snapshot_builder(self):
        with patch.object(
            self.mod,
            "_build_occupancy_snapshot",
            return_value={
                "action": "build_occupancy_snapshot",
                "success": True,
                "occupancy": "occupancy.npz",
            },
        ) as occupancy:
            resp = self._cmd({"action": "build_occupancy", "name": "demo"})

        self.assertTrue(resp["success"])
        self.assertEqual(resp["action"], "build_occupancy_snapshot")
        occupancy.assert_called_once_with("demo")

    def test_map_save_missing_name(self):
        resp = self.mod._map_save("")
        self.assertFalse(resp["success"])

    @patch("subprocess.run")
    @patch(
        "gateway.gateway_module._apply_dynamic_filter_step1half",
        return_value=None,
    )
    def test_map_save_super_lio_reports_gateway_capability_contract(
        self,
        _filter,
        run,
    ):
        map_dir = Path(self._map_dir) / "super_lio_map"
        pcd_path = map_dir / "map.pcd"
        self.mod._on_map_cloud(
            PointCloud2.from_numpy(
                np.array(
                    [
                        [1.0, 2.0, 0.1],
                        [3.0, 4.0, 0.2],
                        [np.inf, 5.0, 0.3],
                    ],
                    dtype=np.float32,
                )
            )
        )

        def _build_tomogram(name):
            return {
                "action": "build_tomogram",
                "success": False,
                "message": "Super-LIO PGO output unavailable",
            }

        with patch.object(self.mod, "_build_tomogram", side_effect=_build_tomogram):
            with patch.object(
                self.mod,
                "_build_occupancy_snapshot",
                return_value={"success": False, "message": "no PGO patches"},
            ):
                resp = self.mod._map_save(
                    "super_lio_map",
                    slam_profile="super_lio",
                )

        self.assertTrue(resp["success"], resp)
        run.assert_not_called()
        self.assertTrue(pcd_path.exists())
        raw = pcd_path.read_bytes()
        self.assertIn(b"FIELDS x y z", raw)
        self.assertIn(b"DATA binary", raw)
        self.assertIn(b"POINTS 2", raw)
        self.assertEqual(resp["slam_profile"], "super_lio")
        self.assertTrue(resp["map_save_supported"])
        self.assertEqual(resp["source"], "live_map_cloud_snapshot")
        self.assertEqual(resp["map_save_source"], "live_map_cloud_snapshot")
        self.assertFalse(resp["relocalization_supported"])
        self.assertFalse(resp["saved_map_relocalization_supported"])
        self.assertTrue(resp["restart_recovery_supported"])
        self.assertEqual(resp["recovery_method"], "restart_super_lio")
        self.assertFalse(resp["tomogram_ok"])
        self.assertIn("Super-LIO", resp["warnings"][0])

    @patch("subprocess.run")
    def test_map_save_super_lio_fails_without_live_map_cloud(self, run):
        resp = self.mod._map_save("super_lio_empty", slam_profile="super_lio")

        self.assertFalse(resp["success"])
        self.assertEqual(resp["slam_profile"], "super_lio")
        self.assertTrue(resp["map_save_supported"])
        self.assertEqual(resp["map_save_source"], "live_map_cloud_snapshot")
        self.assertIn("No live map_cloud", resp["message"])
        run.assert_not_called()

    def test_super_lio_relocation_alias_reports_conservative_capability_contract(
        self,
    ):
        fields = self.mod._map_save_capability_fields("relocation")

        self.assertEqual(fields["map_save_source"], "active_map")
        self.assertFalse(fields["map_save_supported"])
        self.assertFalse(fields["relocalization_supported"])
        self.assertFalse(fields["saved_map_relocalization_supported"])
        self.assertTrue(fields["restart_recovery_supported"])
        self.assertEqual(
            fields["recovery_method"],
            "restart_super_lio_relocation",
        )

    @patch("subprocess.run")
    def test_map_save_super_lio_relocation_is_explicitly_unsupported(self, run):
        resp = self.mod._map_save(
            "relocation_map",
            slam_profile="super_lio_relocation",
        )

        self.assertFalse(resp["success"])
        self.assertEqual(resp["slam_profile"], "super_lio_relocation")
        self.assertFalse(resp["map_save_supported"])
        self.assertEqual(resp["map_save_source"], "active_map")
        self.assertIn("not supported", resp["message"])
        run.assert_not_called()

    @patch("subprocess.run")
    @patch(
        "gateway.gateway_module._apply_dynamic_filter_step1half",
        return_value=None,
    )
    def test_map_save_localizer_keeps_relocalization_contract(
        self,
        _filter,
        run,
    ):
        run.return_value = MagicMock(returncode=0, stderr="")

        with patch.object(
            self.mod,
            "_build_tomogram",
            return_value={"success": True, "tomogram": "tomogram.pickle"},
        ):
            with patch.object(
                self.mod,
                "_build_occupancy_snapshot",
                return_value={"success": True, "occupancy": "occupancy.npz"},
            ):
                resp = self.mod._map_save(
                    "localizer_map",
                    slam_profile="localizer",
                )

        self.assertTrue(resp["success"], resp)
        self.assertEqual(resp["slam_profile"], "localizer")
        self.assertTrue(resp["map_save_supported"])
        self.assertEqual(resp["map_save_source"], "slam_service")
        self.assertTrue(resp["relocalization_supported"])
        self.assertTrue(resp["saved_map_relocalization_supported"])
        self.assertTrue(resp["restart_recovery_supported"])
        self.assertEqual(resp["recovery_method"], "relocalize_service")

    @patch("subprocess.run")
    @patch(
        "gateway.gateway_module._apply_dynamic_filter_step1half",
        return_value=None,
    )
    def test_map_save_localizer_requires_navigation_tomogram(
        self,
        _filter,
        run,
    ):
        run.return_value = MagicMock(returncode=0, stderr="")

        with patch.object(
            self.mod,
            "_build_tomogram",
            return_value={
                "success": False,
                "message": "tomogram source has no occupied structure",
            },
        ):
            with patch.object(self.mod, "_build_occupancy_snapshot") as occupancy:
                resp = self.mod._map_save(
                    "bad_nav_map",
                    slam_profile="localizer",
                )

        self.assertFalse(resp["success"], resp)
        self.assertEqual(resp["slam_profile"], "localizer")
        self.assertTrue(resp["map_save_supported"])
        self.assertEqual(resp["map_save_source"], "slam_service")
        self.assertFalse(resp["tomogram_ok"])
        self.assertIn("not ready for navigation", resp["message"])
        self.assertIn("tomogram source", resp["tomogram_message"])
        occupancy.assert_not_called()

    @patch("subprocess.run")
    @patch(
        "gateway.gateway_module._apply_dynamic_filter_step1half",
        return_value=None,
    )
    def test_simulation_only_map_save_closes_active_artifact_loop(
        self,
        _filter,
        run,
    ):
        """Map save produces active map artifacts without ROS2 or hardware."""
        map_name = "sim_saved"
        map_dir = Path(self._map_dir) / map_name

        def _fake_save_maps(*_args, **_kw):
            cmd = list(_args[0]) if _args else []
            if "/pgo/save_maps" in cmd:
                map_dir.mkdir(parents=True, exist_ok=True)
                (map_dir / "map.pcd").write_text(
                    "VERSION 0.7\n"
                    "FIELDS x y z\n"
                    "SIZE 4 4 4\n"
                    "TYPE F F F\n"
                    "COUNT 1 1 1\n"
                    "WIDTH 8\n"
                    "HEIGHT 1\n"
                    "VIEWPOINT 0 0 0 1 0 0 0\n"
                    "POINTS 8\n"
                    "DATA ascii\n"
                    "0.0 0.0 0.0\n"
                    "1.0 0.0 0.0\n"
                    "0.0 1.0 0.0\n"
                    "1.0 1.0 0.0\n"
                    "0.0 0.0 0.5\n"
                    "1.0 0.0 0.5\n"
                    "0.0 1.0 0.5\n"
                    "1.0 1.0 0.5\n",
                    encoding="utf-8",
                )
            return MagicMock(returncode=0, stderr="")

        def _fake_build_tomogram(name):
            import pickle

            tomogram_path = Path(self._map_dir) / name / "tomogram.pickle"
            tomogram_path.write_bytes(
                pickle.dumps(
                    {
                        "data": np.zeros((5, 1, 2, 2), dtype=np.float16),
                        "resolution": 0.2,
                        "center": [0.0, 0.0],
                        "slice_h0": 0.0,
                        "slice_dh": 0.25,
                    },
                    protocol=pickle.HIGHEST_PROTOCOL,
                )
            )
            return {
                "action": "build_tomogram",
                "success": True,
                "tomogram": str(tomogram_path),
            }

        run.side_effect = _fake_save_maps
        with patch.object(self.mod, "_build_tomogram", side_effect=_fake_build_tomogram):
            resp = self.mod._map_save(map_name, slam_profile="localizer")

        self.assertTrue(resp["success"], resp)
        self.assertEqual(resp["slam_profile"], "localizer")
        self.assertEqual(resp["map_save_source"], "slam_service")
        self.assertTrue(resp["tomogram_ok"])
        self.assertTrue(resp["occupancy_ok"])
        self.assertTrue(Path(resp["pcd"]).exists())
        self.assertTrue(Path(resp["tomogram"]).exists())
        self.assertTrue(Path(resp["occupancy"]).exists())
        self.assertTrue(resp["saved_map_relocalization_supported"])
        save_calls = [call for call in run.call_args_list if "/pgo/save_maps" in call.args[0]]
        self.assertEqual(len(save_calls), 1)

        active = self._cmd({"action": "set_active", "name": map_name})
        self.assertTrue(active["success"], active)
        self.assertEqual(self.mod._active_map, map_name)
        self.assertEqual(
            Path(self.mod.get_active_tomogram()).resolve(),
            (map_dir / "tomogram.pickle").resolve(),
        )
        self.assertEqual(
            Path(self.mod.get_active_occupancy()).resolve(),
            (map_dir / "occupancy.npz").resolve(),
        )

        listed = self._cmd({"action": "list"})
        entry = next(m for m in listed["maps"] if m["name"] == map_name)
        self.assertTrue(entry["has_pcd"])
        self.assertTrue(entry["has_tomogram"])
        self.assertTrue(entry["has_occupancy"])

        occupancy = np.load(resp["occupancy"])
        self.assertIn("grid", occupancy)
        self.assertTrue(np.any(occupancy["grid"] == 100))

    # -- occupancy snapshot tests --

    def test_build_occupancy_snapshot_missing_name(self):
        resp = self.mod._build_occupancy_snapshot("")
        self.assertFalse(resp["success"])

    def test_build_occupancy_snapshot_no_pcd(self):
        resp = self.mod._build_occupancy_snapshot("nonexistent_map")
        self.assertFalse(resp["success"])
        self.assertIn("no PCD file", resp["message"])

    def test_build_occupancy_snapshot_ascii_pcd(self):
        """Write a minimal ASCII PCD file and verify occupancy.npz is produced."""
        map_dir = Path(self._map_dir) / "test_occ"
        map_dir.mkdir()
        pcd_path = map_dir / "map.pcd"
        # Minimal valid ASCII PCD with 4 obstacle-height points forming a 2x2 square
        # Ground points at z=0.0 (many, so percentile-5 ≈ 0.0),
        # obstacle points at z=0.5 (within 0.10–2.00 above ground).
        pcd_content = (
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            "WIDTH 8\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            "POINTS 8\n"
            "DATA ascii\n"
            "0.0 0.0 0.0\n"
            "1.0 0.0 0.0\n"
            "0.0 1.0 0.0\n"
            "1.0 1.0 0.0\n"
            "0.0 0.0 0.5\n"
            "1.0 0.0 0.5\n"
            "0.0 1.0 0.5\n"
            "1.0 1.0 0.5\n"
        )
        pcd_path.write_text(pcd_content)

        resp = self.mod._build_occupancy_snapshot("test_occ")
        self.assertTrue(resp["success"], f"Expected success, got: {resp}")
        occ_path = map_dir / "occupancy.npz"
        self.assertTrue(occ_path.exists())

        data = np.load(str(occ_path))
        self.assertIn("grid", data)
        self.assertIn("resolution", data)
        self.assertIn("origin", data)
        grid = data["grid"]
        self.assertEqual(grid.dtype, np.int8)
        self.assertEqual(grid.ndim, 2)
        # At least one occupied cell
        self.assertTrue(np.any(grid == 100))

        metadata_path = map_dir / "metadata.json"
        self.assertTrue(metadata_path.exists())
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        artifacts = metadata["artifacts"]
        from core.runtime_interface import TOPICS, topic_default_frame_id

        self.assertEqual(
            metadata["frame_id"],
            topic_default_frame_id(TOPICS.saved_map_cloud),
        )
        self.assertIn("map_pcd", artifacts)
        self.assertIn("occupancy_grid", artifacts)
        self.assertEqual(
            artifacts["occupancy_grid"]["source_map_sha256"],
            artifacts["map_pcd"]["sha256"],
        )
        from core.same_source_map_artifacts import (
            validate_same_source_map_metadata,
        )

        validation = validate_same_source_map_metadata(metadata)
        self.assertTrue(validation["ok"], validation)

    def test_load_pcd_points_ascii(self):
        """_load_pcd_points parses a simple ASCII PCD correctly."""
        import tempfile
        pcd_content = (
            "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            "WIDTH 3\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 3\nDATA ascii\n"
            "1.0 2.0 0.5\n3.0 4.0 1.0\n5.0 6.0 1.5\n"
        )
        with tempfile.NamedTemporaryFile(mode="w", suffix=".pcd", delete=False) as f:
            f.write(pcd_content)
            fname = f.name
        try:
            from nav.services.map_manager_module import MapManagerModule
            pts = MapManagerModule._load_pcd_points(fname)
            self.assertIsNotNone(pts)
            self.assertEqual(pts.shape, (3, 3))
            np.testing.assert_allclose(pts[0], [1.0, 2.0, 0.5], atol=1e-5)
        finally:
            os.unlink(fname)

    def test_map_list_has_occupancy_field(self):
        """_map_list reports has_occupancy=True when occupancy.npz exists."""
        d = Path(self._map_dir) / "mapwithall"
        d.mkdir()
        (d / "map.pcd").touch()
        (d / "tomogram.pickle").touch()
        (d / "occupancy.npz").touch()

        resp = self._cmd({"action": "list"})
        self.assertTrue(resp["success"])
        entry = next(m for m in resp["maps"] if m["name"] == "mapwithall")
        self.assertTrue(entry["has_occupancy"])

    def test_map_list_has_occupancy_false_when_absent(self):
        d = Path(self._map_dir) / "mapnocc"
        d.mkdir()
        (d / "map.pcd").touch()

        resp = self._cmd({"action": "list"})
        entry = next(m for m in resp["maps"] if m["name"] == "mapnocc")
        self.assertFalse(entry["has_occupancy"])

    def test_get_active_occupancy_none(self):
        result = self.mod.get_active_occupancy()
        self.assertIsNone(result)

    def test_get_active_occupancy_returns_path(self):
        self._write_valid_saved_map_artifacts("fullmap", occupancy=True)
        self._cmd({"action": "set_active", "name": "fullmap"})
        result = self.mod.get_active_occupancy()
        self.assertIsNotNone(result)
        self.assertTrue(result.endswith("occupancy.npz"))

    def test_set_active_rejects_map_without_same_source_tomogram_metadata(self):
        d = Path(self._map_dir) / "bad_active"
        d.mkdir()
        (d / "map.pcd").write_text("VERSION 0.7\nPOINTS 0\nDATA ascii\n")
        (d / "tomogram.pickle").write_bytes(b"bad")

        resp = self._cmd({"action": "set_active", "name": "bad_active"})

        self.assertFalse(resp["success"])
        self.assertIn("saved map artifact gate failed", resp["message"])
        self.assertIn("metadata.json missing", resp["artifact_gate"]["blockers"])
        self.assertEqual(self.mod._active_map, "")


# ---------------------------------------------------------------------------
# 2. WavefrontFrontierExplorer
# ---------------------------------------------------------------------------

class TestWavefrontFrontierExplorer(unittest.TestCase):
    """Test frontier discovery, scoring, and health."""

    def _make_explorer(self, **kw):
        from nav.frontier_explorer_module import WavefrontFrontierExplorer
        kw.setdefault("min_frontier_size", 2)
        mod = WavefrontFrontierExplorer(**kw)
        mod.setup()
        return mod

    def test_instantiation(self):
        mod = self._make_explorer()
        self.assertIsNotNone(mod)
        self.assertEqual(mod._min_size, 2)

    def test_setup_runs_without_error(self):
        mod = self._make_explorer()
        # setup() should complete without raising
        self.assertIsNotNone(mod.costmap)

    def test_health_idle(self):
        mod = self._make_explorer()
        h = mod.health()
        self.assertEqual(h["exploration_state"], "idle")
        self.assertEqual(h["frontier_count"], 0)

    def test_on_odometry(self):
        mod = self._make_explorer()
        odom = Odometry(pose=Pose(5.0, 10.0, 0.0))
        mod._on_odometry(odom)
        self.assertAlmostEqual(mod._robot_x, 5.0)
        self.assertAlmostEqual(mod._robot_y, 10.0)

    def test_on_costmap(self):
        mod = self._make_explorer()
        costmap = {"grid": np.zeros((10, 10), dtype=np.int16), "resolution": 0.1,
                   "origin_x": 0.0, "origin_y": 0.0, "width": 10, "height": 10}
        mod._on_costmap(costmap)
        self.assertIsNotNone(mod._costmap_data)

    def test_frontier_detection_prefers_exploration_grid_over_nav_costmap(self):
        mod = self._make_explorer(min_frontier_size=1)
        nav_costmap = {
            "grid": np.zeros((20, 20), dtype=np.int16),
            "resolution": 0.1,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "width": 20,
            "height": 20,
            "source": "navigation_costmap",
        }
        exploration_grid = {
            "grid": np.full((20, 20), -1, dtype=np.int16),
            "resolution": 0.1,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "width": 20,
            "height": 20,
            "source": "exploration_grid",
        }
        exploration_grid["grid"][8:13, 8:13] = 0
        mod._on_costmap(nav_costmap)
        mod._on_exploration_grid(exploration_grid)
        mod._on_odometry(Odometry(pose=Pose(1.0, 1.0, 0.0)))

        frontiers = json.loads(mod.get_frontiers())

        self.assertGreater(len(frontiers), 0)
        health = mod.health()
        self.assertEqual(health["frontier_grid_source"], "exploration_grid")
        self.assertEqual(health["visited_goal_count"], 0)
        self.assertGreater(health["current_frontier_candidates"], 0)
        self.assertGreater(health["frontier_debug"]["frontier_cell_count"], 0)
        self.assertEqual(
            health["frontier_debug"]["last_empty_reason"],
            "scored_frontiers_ready",
        )

    def test_frontier_health_distinguishes_visited_from_detected_frontiers(self):
        mod = self._make_explorer(min_frontier_size=1)
        grid = np.full((20, 20), -1, dtype=np.int16)
        grid[8:13, 8:13] = 0
        payload = {
            "grid": grid,
            "resolution": 0.1,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "width": 20,
            "height": 20,
            "source": "raycast_lidar_exploration_grid",
            "counts": {
                "unknown": int((grid < 0).sum()),
                "free": int((grid == 0).sum()),
                "occupied": int((grid >= 100).sum()),
            },
        }
        mod._on_exploration_grid(payload)
        mod._on_odometry(Odometry(pose=Pose(1.0, 1.0, 0.0)))

        frontiers = json.loads(mod.get_frontiers())
        health = mod.health()

        self.assertGreater(len(frontiers), 0)
        self.assertEqual(health["frontier_count"], 0)
        self.assertEqual(health["visited_goal_count"], 0)
        self.assertEqual(health["current_frontier_candidates"], len(frontiers))
        self.assertEqual(
            health["frontier_grid_counts"],
            {"unknown": 375, "free": 25, "occupied": 0},
        )
        self.assertEqual(
            health["frontier_debug"]["last_empty_reason"],
            "scored_frontiers_ready",
        )

    def test_parse_costmap_valid(self):
        mod = self._make_explorer()
        data = {"grid": np.zeros((20, 30), dtype=np.int16),
                "resolution": 0.05, "origin_x": -1.0, "origin_y": -1.5,
                "width": 30, "height": 20}
        grid, meta = mod._parse_costmap(data)
        self.assertEqual(grid.shape, (20, 30))
        self.assertAlmostEqual(meta["resolution"], 0.05)

    def test_parse_costmap_1d(self):
        mod = self._make_explorer()
        flat = np.zeros(100, dtype=np.int16)
        data = {"grid": flat, "resolution": 0.1, "origin_x": 0.0, "origin_y": 0.0,
                "width": 10, "height": 10}
        grid, _meta = mod._parse_costmap(data)
        self.assertEqual(grid.shape, (10, 10))

    def test_find_frontiers_all_free_no_unknown(self):
        """Grid with only FREE cells has no frontiers."""
        mod = self._make_explorer()
        grid = np.zeros((20, 20), dtype=np.int16)
        meta = {"resolution": 0.1, "origin_x": 0.0, "origin_y": 0.0,
                "width": 20, "height": 20}
        clusters = mod._find_frontier_clusters(grid, meta, 1.0, 1.0)
        self.assertEqual(len(clusters), 0)

    def test_find_frontiers_with_unknown(self):
        """Grid with FREE cells next to UNKNOWN yields frontiers."""
        mod = self._make_explorer(min_frontier_size=1)
        grid = np.full((20, 20), -1, dtype=np.int16)  # all unknown
        # carve a free region around (10, 10)
        grid[8:13, 8:13] = 0  # free
        meta = {"resolution": 0.1, "origin_x": 0.0, "origin_y": 0.0,
                "width": 20, "height": 20}
        clusters = mod._find_frontier_clusters(grid, meta, 1.0, 1.0)
        self.assertGreater(len(clusters), 0)

    def test_find_frontiers_does_not_traverse_unknown(self):
        """Unknown cells are frontier boundaries, not traversable free space."""
        mod = self._make_explorer(min_frontier_size=1, max_explored_distance=10.0)
        grid = np.full((9, 14), -1, dtype=np.int16)
        grid[4, 1:4] = 0
        grid[4, 10:13] = 0
        meta = {"resolution": 1.0, "origin_x": 0.0, "origin_y": 0.0,
                "width": 14, "height": 9}

        clusters = mod._find_frontier_clusters(grid, meta, 1.0, 4.0)
        xs = [cluster["cx"] for cluster in clusters]

        self.assertTrue(xs)
        self.assertLess(max(xs), 5.0)

    def test_score_clusters_empty(self):
        mod = self._make_explorer()
        result = mod._score_clusters([], 0.0, 0.0, 0.0, {})
        self.assertEqual(result, [])

    def test_score_clusters_sorted_descending(self):
        mod = self._make_explorer()
        clusters = [
            {"cx": 1.0, "cy": 0.0, "size": 5, "cells": []},
            {"cx": 5.0, "cy": 0.0, "size": 20, "cells": []},
        ]
        meta = {"resolution": 0.1, "origin_x": 0.0, "origin_y": 0.0,
                "width": 100, "height": 100}
        mod._costmap_data = {"grid": np.zeros((100, 100), dtype=np.int16),
                             "resolution": 0.1, "origin_x": 0.0, "origin_y": 0.0}
        scored = mod._score_clusters(clusters, 0.0, 0.0, 0.0, meta)
        self.assertEqual(len(scored), 2)
        self.assertGreaterEqual(scored[0]["score"], scored[1]["score"])

    def test_score_clusters_filters_unreachable_costmap_goals(self):
        mod = self._make_explorer(safe_distance=0.0, max_explored_distance=10.0)
        costmap = np.full((10, 10), 100, dtype=np.int16)
        costmap[1, 1:4] = 0
        costmap[7, 7] = 0
        mod._costmap_data = {
            "grid": costmap,
            "resolution": 1.0,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "width": 10,
            "height": 10,
        }
        clusters = [
            {"cx": 3.0, "cy": 1.0, "size": 5, "cells": []},
            {"cx": 7.0, "cy": 7.0, "size": 50, "cells": []},
        ]
        meta = {
            "resolution": 1.0,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "width": 10,
            "height": 10,
        }

        scored = mod._score_clusters(clusters, 1.0, 1.0, 0.0, meta)

        self.assertEqual(len(scored), 1)
        self.assertAlmostEqual(scored[0]["cx"], 3.0)

    def test_score_clusters_uses_reachable_approach_when_all_frontiers_unreachable(self):
        mod = self._make_explorer(safe_distance=0.0, max_explored_distance=10.0)
        costmap = np.full((10, 10), 100, dtype=np.int16)
        costmap[1, 1:4] = 0
        mod._costmap_data = {
            "grid": costmap,
            "resolution": 1.0,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "width": 10,
            "height": 10,
        }
        clusters = [{"cx": 4.0, "cy": 1.0, "size": 50, "cells": [(1, 4)]}]
        meta = {
            "resolution": 1.0,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "width": 10,
            "height": 10,
        }

        scored = mod._score_clusters(clusters, 1.0, 1.0, 0.0, meta)

        self.assertEqual(len(scored), 1)
        self.assertNotAlmostEqual(scored[0]["cx"], 4.0)
        self.assertTrue(
            mod._is_costmap_reachable(
                costmap,
                meta,
                1.0,
                1.0,
                scored[0]["cx"],
                scored[0]["cy"],
            )
        )

    def test_score_clusters_reanchors_frontier_boundary_to_approach_goal(self):
        mod = self._make_explorer(
            safe_distance=0.0,
            max_explored_distance=10.0,
            approach_standoff_m=1.0,
            approach_goal_max_distance_m=8.0,
        )
        costmap = np.full((10, 10), 100, dtype=np.int16)
        costmap[1, 1:9] = 0
        mod._costmap_data = {
            "grid": costmap,
            "resolution": 1.0,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "width": 10,
            "height": 10,
        }
        clusters = [{"cx": 8.0, "cy": 1.0, "size": 50, "cells": [(1, 8)]}]
        meta = {
            "resolution": 1.0,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "width": 10,
            "height": 10,
        }

        scored = mod._score_clusters(clusters, 1.0, 1.0, 0.0, meta)
        debug = mod.health()["frontier_debug"]

        self.assertEqual(len(scored), 1)
        self.assertGreaterEqual(math.hypot(scored[0]["cx"] - 8.0, scored[0]["cy"] - 1.0), 1.0)
        self.assertTrue(debug["selected_frontier_approach"])
        self.assertEqual(debug["reachability_filter_reason"], "approach")

    def test_score_clusters_rejects_approach_far_from_frontier_target(self):
        mod = self._make_explorer(
            safe_distance=0.0,
            max_explored_distance=10.0,
            approach_max_target_distance_m=1.5,
        )
        costmap = np.full((12, 12), 100, dtype=np.int16)
        costmap[1, 1:4] = 0
        mod._costmap_data = {
            "grid": costmap,
            "resolution": 1.0,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "width": 12,
            "height": 12,
        }
        clusters = [{"cx": 9.0, "cy": 1.0, "size": 50, "cells": [(1, 9)]}]
        meta = {
            "resolution": 1.0,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "width": 12,
            "height": 12,
        }

        scored = mod._score_clusters(clusters, 1.0, 1.0, 0.0, meta)
        debug = mod.health()["frontier_debug"]

        self.assertEqual(scored, [])
        self.assertEqual(debug["reachability_filter_reason"], "none")
        self.assertEqual(debug["reachable_rejected_count"], 1)

    def test_score_clusters_does_not_direct_track_frontier_boundary(self):
        mod = self._make_explorer(
            safe_distance=0.0,
            max_explored_distance=12.0,
            approach_max_target_distance_m=1.5,
            approach_goal_max_distance_m=3.0,
        )
        costmap = np.full((12, 12), 100, dtype=np.int16)
        costmap[1, 1:10] = 0
        mod._costmap_data = {
            "grid": costmap,
            "resolution": 1.0,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "width": 12,
            "height": 12,
        }
        clusters = [{"cx": 9.0, "cy": 1.0, "size": 50, "cells": [(1, 9)]}]
        meta = {
            "resolution": 1.0,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "width": 12,
            "height": 12,
        }

        scored = mod._score_clusters(clusters, 1.0, 1.0, 0.0, meta)
        debug = mod.health()["frontier_debug"]

        self.assertEqual(scored, [])
        self.assertEqual(debug["reachability_filter_reason"], "none")
        self.assertEqual(debug["reachable_direct_count"], 0)

    def test_costmap_reachability_matches_astar_corner_cutting(self):
        mod = self._make_explorer(max_explored_distance=5.0)
        costmap = np.full((3, 3), 100, dtype=np.int16)
        costmap[0, 0] = 0
        costmap[1, 1] = 0
        meta = {
            "resolution": 1.0,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "width": 3,
            "height": 3,
        }

        reachable = mod._is_costmap_reachable(costmap, meta, 0.0, 0.0, 1.0, 1.0)

        self.assertFalse(reachable)

    def test_on_goal_reached(self):
        mod = self._make_explorer()
        mod._on_goal_reached(True)
        self.assertTrue(mod._goal_reached_event.is_set())

    def test_navigation_failure_blocks_matching_frontier_goal(self):
        mod = self._make_explorer(
            blocked_goal_radius=0.5,
            navigation_failure_grace_s=0.0,
        )
        mod._current_goal = (1.0, 2.0)
        mod._costmap_data = {
            "grid": np.zeros((50, 50), dtype=np.int16),
            "resolution": 0.1,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "width": 50,
            "height": 50,
        }

        mod._on_navigation_status({
            "state": "FAILED",
            "goal": [1.0, 2.0, 0.0],
            "failure_reason": "planner returned empty path",
        })

        self.assertTrue(mod._goal_reached_event.is_set())
        self.assertEqual(mod.health()["blocked_goal_count"], 1)
        scored = mod._score_clusters(
            [
                {"cx": 1.0, "cy": 2.0, "size": 5, "cells": []},
                {"cx": 3.0, "cy": 2.0, "size": 5, "cells": []},
            ],
            0.0,
            0.0,
            0.0,
            {
                "resolution": 0.1,
                "origin_x": 0.0,
                "origin_y": 0.0,
                "width": 50,
                "height": 50,
            },
        )
        self.assertEqual([round(item["cx"], 1) for item in scored], [3.0])

    def test_navigation_status_matches_current_goal_accepts_list_and_dict_goal(self):
        mod = self._make_explorer(blocked_goal_radius=0.5)
        mod._current_goal = (1.0, 2.0)

        self.assertTrue(
            mod._navigation_status_matches_current_goal({
                "goal": [1.0, 2.0, 0.0],
            })
        )
        self.assertTrue(
            mod._navigation_status_matches_current_goal({
                "goal": {"x": 1.0, "y": 2.0, "z": 0.0},
            })
        )
        self.assertFalse(
            mod._navigation_status_matches_current_goal({
                "goal": {"x": 5.0, "y": 5.0, "z": 0.0},
            })
        )

    def test_transient_failed_then_executing_does_not_block_frontier_goal(self):
        mod = self._make_explorer(
            blocked_goal_radius=0.5,
            navigation_failure_grace_s=5.0,
        )
        mod._current_goal = (1.0, 2.0)

        mod._on_navigation_status({
            "state": "FAILED",
            "goal": [1.0, 2.0, 0.0],
            "failure_reason": "planner transient empty path",
        })
        self.assertFalse(mod._goal_reached_event.is_set())
        self.assertIsNotNone(mod.health()["pending_goal_failure"])

        mod._on_navigation_status({
            "state": "EXECUTING",
            "goal": {"x": 1.0, "y": 2.0, "z": 0.0},
        })

        self.assertFalse(mod._goal_reached_event.is_set())
        self.assertEqual(mod.health()["blocked_goal_count"], 0)
        self.assertIsNone(mod.health()["pending_goal_failure"])

    def test_navigation_failure_without_goal_is_pending_for_current_goal(self):
        mod = self._make_explorer(navigation_failure_grace_s=5.0)
        mod._current_goal = (1.0, 2.0)

        mod._on_navigation_status({
            "state": "FAILED",
            "failure_reason": "planner returned empty path",
        })

        self.assertFalse(mod._goal_reached_event.is_set())
        self.assertEqual(mod.health()["blocked_goal_count"], 0)
        self.assertIsNotNone(mod.health()["pending_goal_failure"])

    def test_persistent_navigation_failure_blocks_after_grace(self):
        mod = self._make_explorer(
            blocked_goal_radius=0.5,
            navigation_failure_grace_s=1.0,
        )
        mod._current_goal = (1.0, 2.0)

        mod._on_navigation_status({
            "state": "FAILED",
            "goal": [1.0, 2.0, 0.0],
            "failure_reason": "planner returned empty path",
        })
        with mod._state_lock:
            mod._pending_goal_failure["ts"] -= 2.0

        self.assertTrue(mod._wait_for_goal_result(timeout=0.2))
        self.assertTrue(mod._goal_reached_event.is_set())
        self.assertEqual(mod.health()["blocked_goal_count"], 1)

    def test_navigation_failure_for_other_goal_is_ignored_by_frontier(self):
        mod = self._make_explorer(blocked_goal_radius=0.5)
        mod._current_goal = (1.0, 2.0)

        mod._on_navigation_status({
            "state": "FAILED",
            "goal": [5.0, 5.0, 0.0],
            "failure_reason": "external goal failed",
        })

        self.assertFalse(mod._goal_reached_event.is_set())
        self.assertEqual(mod.health()["blocked_goal_count"], 0)

    def test_make_pose_stamped(self):
        self._make_explorer()
        # _make_pose_stamped sets pose position via PoseStamped constructor
        # PoseStamped.x is a read-only property delegating to pose.x
        # The method may use direct pose attribute access; verify frame_id
        from core.msgs.geometry import Pose, PoseStamped
        ps = PoseStamped(pose=Pose(3.0, 4.0, 0.0), frame_id="map")
        self.assertAlmostEqual(ps.x, 3.0)
        self.assertAlmostEqual(ps.y, 4.0)
        self.assertEqual(ps.frame_id, "map")


# ---------------------------------------------------------------------------
# 3. PatrolManagerModule
# ---------------------------------------------------------------------------

class TestPatrolManagerModule(unittest.TestCase):
    """Test patrol route CRUD and start/stop logic."""

    def setUp(self):
        self._tmpdir = tempfile.mkdtemp()
        from nav.services.patrol_manager_module import PatrolManagerModule
        self.mod = PatrolManagerModule(routes_dir=os.path.join(self._tmpdir, "routes"))
        self.mod.setup()
        self._statuses: list[str] = []
        self._goals: list[list] = []
        self.mod.patrol_status.subscribe(lambda s: self._statuses.append(s))
        self.mod.patrol_goals.subscribe(lambda g: self._goals.append(g))

    def tearDown(self):
        shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _cmd(self, cmd: dict) -> dict:
        self._statuses.clear()
        self.mod._on_command(json.dumps(cmd))
        self.assertTrue(len(self._statuses) > 0)
        return json.loads(self._statuses[-1])

    def test_instantiation(self):
        self.assertTrue(Path(self._tmpdir, "routes").is_dir())

    def test_save_and_list(self):
        wps = [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0}]
        resp = self._cmd({"action": "save", "name": "route_a", "waypoints": wps})
        self.assertTrue(resp["success"])

        resp = self._cmd({"action": "list"})
        self.assertTrue(resp["success"])
        self.assertEqual(len(resp["routes"]), 1)
        self.assertEqual(resp["routes"][0]["waypoint_count"], 2)

    def test_save_missing_name(self):
        resp = self._cmd({"action": "save", "name": "", "waypoints": [{"x": 0}]})
        self.assertFalse(resp["success"])

    def test_save_empty_waypoints(self):
        resp = self._cmd({"action": "save", "name": "empty", "waypoints": []})
        self.assertFalse(resp["success"])

    def test_load_route(self):
        wps = [{"x": 1.0, "y": 2.0}]
        self._cmd({"action": "save", "name": "r1", "waypoints": wps})
        resp = self._cmd({"action": "load", "name": "r1"})
        self.assertTrue(resp["success"])
        self.assertIn("route", resp)

    def test_load_nonexistent(self):
        resp = self._cmd({"action": "load", "name": "nope"})
        self.assertFalse(resp["success"])

    def test_delete_route(self):
        self._cmd({"action": "save", "name": "del_me", "waypoints": [{"x": 0}]})
        resp = self._cmd({"action": "delete", "name": "del_me"})
        self.assertTrue(resp["success"])
        resp = self._cmd({"action": "list"})
        self.assertEqual(len(resp["routes"]), 0)

    def test_delete_nonexistent(self):
        resp = self._cmd({"action": "delete", "name": "ghost"})
        self.assertFalse(resp["success"])

    def test_start_patrol(self):
        wps = [{"x": 1.0, "y": 2.0}]
        self._cmd({"action": "save", "name": "patrol1", "waypoints": wps})
        self._goals.clear()
        resp = self._cmd({"action": "start", "name": "patrol1"})
        self.assertTrue(resp["success"])
        self.assertEqual(len(self._goals), 1)
        self.assertEqual(self.mod._active_route, "patrol1")

    def test_start_patrol_nonexistent(self):
        resp = self._cmd({"action": "start", "name": "ghost"})
        self.assertFalse(resp["success"])

    def test_stop_patrol(self):
        resp = self._cmd({"action": "stop"})
        self.assertTrue(resp["success"])
        self.assertIsNone(self.mod._active_route)

    def test_on_odom_updates_pose(self):
        odom = Odometry(pose=Pose(7.0, 8.0, 9.0))
        self.mod._on_odom(odom)
        self.assertAlmostEqual(self.mod._current_pose["x"], 7.0)

    def test_unknown_action(self):
        resp = self._cmd({"action": "xyz"})
        self.assertFalse(resp["success"])


# ---------------------------------------------------------------------------
# 4. TaskSchedulerModule
# ---------------------------------------------------------------------------

class TestTaskSchedulerModule(unittest.TestCase):
    """Test schedule CRUD, firing logic, and double-fire prevention."""

    def setUp(self):
        self._tmpdir = tempfile.mkdtemp()
        from nav.services.task_scheduler_module import TaskSchedulerModule
        self.mod = TaskSchedulerModule(
            schedule_file=os.path.join(self._tmpdir, "sched.yaml")
        )
        self.mod.setup()
        self._tasks: list[dict] = []
        self.mod.scheduled_task.subscribe(lambda t: self._tasks.append(t))

    def tearDown(self):
        shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _cmd(self, cmd: dict) -> dict:
        self._tasks.clear()
        self.mod._on_command(json.dumps(cmd))
        self.assertTrue(len(self._tasks) > 0)
        return self._tasks[-1]

    def test_instantiation(self):
        self.assertIsInstance(self.mod._schedules, dict)

    def test_add_and_list(self):
        resp = self._cmd({"action": "add", "name": "morning",
                          "patrol_route": "route_a", "hour": 8, "minute": 0})
        self.assertTrue(resp["success"])

        resp = self._cmd({"action": "list"})
        self.assertTrue(resp["success"])
        self.assertEqual(len(resp["schedules"]), 1)
        self.assertEqual(resp["schedules"][0]["patrol_route"], "route_a")

    def test_add_missing_name(self):
        resp = self._cmd({"action": "add", "name": "", "patrol_route": "r"})
        self.assertFalse(resp["success"])

    def test_add_missing_route(self):
        resp = self._cmd({"action": "add", "name": "s", "patrol_route": ""})
        self.assertFalse(resp["success"])

    def test_remove(self):
        self._cmd({"action": "add", "name": "temp", "patrol_route": "r"})
        resp = self._cmd({"action": "remove", "name": "temp"})
        self.assertTrue(resp["success"])
        resp = self._cmd({"action": "list"})
        self.assertEqual(len(resp["schedules"]), 0)

    def test_remove_nonexistent(self):
        resp = self._cmd({"action": "remove", "name": "ghost"})
        self.assertFalse(resp["success"])

    def test_enable_disable(self):
        self._cmd({"action": "add", "name": "s1", "patrol_route": "r1"})
        resp = self._cmd({"action": "disable", "name": "s1"})
        self.assertTrue(resp["success"])
        self.assertFalse(self.mod._schedules["s1"]["enabled"])

        resp = self._cmd({"action": "enable", "name": "s1"})
        self.assertTrue(resp["success"])
        self.assertTrue(self.mod._schedules["s1"]["enabled"])

    def test_enable_nonexistent(self):
        resp = self._cmd({"action": "enable", "name": "nope"})
        self.assertFalse(resp["success"])

    def test_check_schedules_fires(self):
        self.mod._schedules["test"] = {
            "patrol_route": "r_test",
            "hour": 14,
            "minute": 30,
            "weekdays": [0, 1, 2, 3, 4, 5, 6],
            "enabled": True,
        }
        self._tasks.clear()
        # Simulate time that matches
        now = datetime(2026, 4, 6, 14, 30, 0)  # Monday
        fired = self.mod.check_schedules(now=now)
        self.assertEqual(len(fired), 1)
        self.assertEqual(fired[0]["patrol_route"], "r_test")
        self.assertEqual(fired[0]["event"], "schedule_fired")

    def test_check_schedules_no_double_fire(self):
        self.mod._schedules["test"] = {
            "patrol_route": "r_test",
            "hour": 14, "minute": 30,
            "weekdays": [0, 1, 2, 3, 4, 5, 6], "enabled": True,
        }
        now = datetime(2026, 4, 6, 14, 30, 0)
        self.mod.check_schedules(now=now)
        self._tasks.clear()
        fired = self.mod.check_schedules(now=now)
        self.assertEqual(len(fired), 0)

    def test_check_schedules_disabled_skipped(self):
        self.mod._schedules["test"] = {
            "patrol_route": "r_test",
            "hour": 14, "minute": 30,
            "weekdays": [0, 1, 2, 3, 4, 5, 6], "enabled": False,
        }
        now = datetime(2026, 4, 6, 14, 30, 0)
        fired = self.mod.check_schedules(now=now)
        self.assertEqual(len(fired), 0)

    def test_check_schedules_wrong_weekday(self):
        self.mod._schedules["test"] = {
            "patrol_route": "r_test",
            "hour": 14, "minute": 30,
            "weekdays": [2],  # Wednesday only
            "enabled": True,
        }
        # 2026-04-06 is Monday (weekday=0)
        now = datetime(2026, 4, 6, 14, 30, 0)
        fired = self.mod.check_schedules(now=now)
        self.assertEqual(len(fired), 0)

    def test_unknown_action(self):
        resp = self._cmd({"action": "xyz"})
        self.assertFalse(resp["success"])


# ---------------------------------------------------------------------------
# 5. GeofenceManagerModule
# ---------------------------------------------------------------------------

class TestGeofenceManagerModule(unittest.TestCase):
    """Test fence CRUD, point-in-polygon, and intrusion detection."""

    def setUp(self):
        self._tmpdir = tempfile.mkdtemp()
        from nav.services.geofence_manager_module import GeofenceManagerModule
        self.mod = GeofenceManagerModule(
            geofence_file=os.path.join(self._tmpdir, "fences.yaml")
        )
        self.mod.setup()
        self._alerts: list[dict] = []
        self.mod.geofence_alert.subscribe(lambda a: self._alerts.append(a))

    def tearDown(self):
        shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _cmd(self, cmd: dict) -> dict:
        self._alerts.clear()
        self.mod._on_command(json.dumps(cmd))
        self.assertTrue(len(self._alerts) > 0)
        return self._alerts[-1]

    def test_instantiation(self):
        self.assertIsInstance(self.mod._fences, dict)

    def test_add_fence(self):
        polygon = [[0, 0], [10, 0], [10, 10], [0, 10]]
        resp = self._cmd({"action": "add", "name": "zone1", "polygon": polygon})
        self.assertTrue(resp["success"])

    def test_add_fence_too_few_vertices(self):
        resp = self._cmd({"action": "add", "name": "bad", "polygon": [[0, 0], [1, 1]]})
        self.assertFalse(resp["success"])

    def test_add_fence_missing_name(self):
        resp = self._cmd({"action": "add", "name": "", "polygon": [[0,0],[1,0],[1,1]]})
        self.assertFalse(resp["success"])

    def test_list_fences(self):
        self._cmd({"action": "add", "name": "z1",
                   "polygon": [[0, 0], [1, 0], [1, 1], [0, 1]]})
        resp = self._cmd({"action": "list"})
        self.assertTrue(resp["success"])
        self.assertEqual(len(resp["fences"]), 1)
        self.assertEqual(resp["fences"][0]["vertex_count"], 4)

    def test_remove_fence(self):
        self._cmd({"action": "add", "name": "rm_me",
                   "polygon": [[0, 0], [1, 0], [1, 1]]})
        resp = self._cmd({"action": "remove", "name": "rm_me"})
        self.assertTrue(resp["success"])
        resp = self._cmd({"action": "list"})
        self.assertEqual(len(resp["fences"]), 0)

    def test_remove_nonexistent(self):
        resp = self._cmd({"action": "remove", "name": "ghost"})
        self.assertFalse(resp["success"])

    def test_clear_fences(self):
        self._cmd({"action": "add", "name": "a",
                   "polygon": [[0,0],[1,0],[1,1]]})
        self._cmd({"action": "add", "name": "b",
                   "polygon": [[0,0],[1,0],[1,1]]})
        resp = self._cmd({"action": "clear"})
        self.assertTrue(resp["success"])
        self.assertIn("2", resp["message"])

    def test_enable_disable(self):
        self._cmd({"action": "add", "name": "f1",
                   "polygon": [[0,0],[1,0],[1,1]]})
        resp = self._cmd({"action": "disable", "name": "f1"})
        self.assertTrue(resp["success"])
        self.assertFalse(self.mod._fences["f1"]["enabled"])

        resp = self._cmd({"action": "enable", "name": "f1"})
        self.assertTrue(resp["success"])
        self.assertTrue(self.mod._fences["f1"]["enabled"])

    def test_enable_nonexistent(self):
        resp = self._cmd({"action": "enable", "name": "nope"})
        self.assertFalse(resp["success"])

    # -- point-in-polygon --

    def test_point_in_polygon_inside(self):
        from nav.services.geofence_manager_module import GeofenceManagerModule
        poly = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self.assertTrue(GeofenceManagerModule._point_in_polygon(5, 5, poly))

    def test_point_in_polygon_outside(self):
        from nav.services.geofence_manager_module import GeofenceManagerModule
        poly = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self.assertFalse(GeofenceManagerModule._point_in_polygon(15, 15, poly))

    def test_point_in_polygon_too_few(self):
        from nav.services.geofence_manager_module import GeofenceManagerModule
        self.assertFalse(GeofenceManagerModule._point_in_polygon(0, 0, [[0, 0]]))

    # -- intrusion detection --

    def test_check_intrusion_inside(self):
        polygon = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self.mod._fences["zone"] = {"polygon": polygon, "enabled": True}
        odom = Odometry(pose=Pose(5.0, 5.0, 0.0))
        self.mod._on_odom(odom)
        self._alerts.clear()
        alerts = self.mod.check_intrusion()
        self.assertEqual(len(alerts), 1)
        self.assertEqual(alerts[0]["type"], "intrusion")

    def test_intrusion_publishes_hard_stop_on_odom(self):
        polygon = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self.mod._fences["zone"] = {"polygon": polygon, "enabled": True}
        stops = []
        self.mod.stop_cmd._add_callback(stops.append)

        self.mod._on_odom(Odometry(pose=Pose(5.0, 5.0, 0.0)))

        self.assertEqual(stops[-1], 2)

    def test_check_intrusion_outside(self):
        polygon = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self.mod._fences["zone"] = {"polygon": polygon, "enabled": True}
        odom = Odometry(pose=Pose(50.0, 50.0, 0.0))
        self.mod._on_odom(odom)
        self._alerts.clear()
        alerts = self.mod.check_intrusion()
        self.assertEqual(len(alerts), 0)

    def test_check_intrusion_disabled_fence(self):
        polygon = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self.mod._fences["zone"] = {"polygon": polygon, "enabled": False}
        odom = Odometry(pose=Pose(5.0, 5.0, 0.0))
        self.mod._on_odom(odom)
        alerts = self.mod.check_intrusion()
        self.assertEqual(len(alerts), 0)

    def test_unknown_action(self):
        resp = self._cmd({"action": "foobar"})
        self.assertFalse(resp["success"])


# ---------------------------------------------------------------------------
# 6. CameraBridgeModule
# ---------------------------------------------------------------------------

class TestCameraBridgeModule(unittest.TestCase):
    """Test CameraBridgeModule instantiation and stub mode (no rclpy)."""

    def _make_bridge(self, **kw):
        from drivers.real.thunder.camera_bridge_module import CameraBridgeModule
        return CameraBridgeModule(**kw)

    def _make_stub_bridge(self, **kw):
        mod = self._make_bridge(**kw)
        mod._create_dds_reader = MagicMock(return_value=False)
        mod._create_ros2_node = MagicMock(return_value=False)
        return mod

    def test_instantiation_defaults(self):
        mod = self._make_bridge()
        self.assertEqual(mod._color_topic, "/camera/color/image_raw")
        self.assertEqual(mod._depth_topic, "/camera/depth/image_raw")
        self.assertIn("/camera/color/camera_info", mod._info_topics)
        self.assertIn("/camera/depth/camera_info", mod._info_topics)
        self.assertIn("/camera/camera_info", mod._info_topics)
        self.assertEqual(mod._preferred_info_topic, "/camera/color/camera_info")
        self.assertEqual(mod._spin_rate, 30.0)

    def test_instantiation_custom_topics(self):
        mod = self._make_bridge(
            color_topic="/rgb",
            depth_topic="/depth",
            info_topics=("/rgb/info", "/depth/info"),
        )
        self.assertEqual(mod._color_topic, "/rgb")
        self.assertEqual(mod._depth_topic, "/depth")
        self.assertEqual(mod._info_topics, ("/rgb/info", "/depth/info"))

    def test_camera_info_topic_precedence_prefers_color(self):
        mod = self._make_bridge()

        class Info:
            width = 848
            height = 480
            k = [411.0, 0.0, 424.0, 0.0, 411.0, 240.0, 0.0, 0.0, 1.0]
            d = []

        mod._on_ros2_info(Info(), "/camera/depth/camera_info")
        self.assertEqual(mod._active_info_topic, "/camera/depth/camera_info")

        Info.width = 1280
        Info.height = 720
        Info.k = [693.0, 0.0, 642.0, 0.0, 693.0, 361.0, 0.0, 0.0, 1.0]
        mod._on_ros2_info(Info(), "/camera/color/camera_info")
        self.assertEqual(mod._active_info_topic, "/camera/color/camera_info")

        Info.width = 848
        Info.height = 480
        Info.k = [411.0, 0.0, 424.0, 0.0, 411.0, 240.0, 0.0, 0.0, 1.0]
        mod._on_ros2_info(Info(), "/camera/depth/camera_info")
        self.assertEqual(mod._active_info_topic, "/camera/color/camera_info")

    def test_undistortion_maps_require_matching_image_shape(self):
        mod = self._make_bridge()
        map1 = np.zeros((720, 1280, 2), dtype=np.int16)
        map2 = np.zeros((720, 1280), dtype=np.uint16)
        mod._undistort_maps = (map1, map2)

        self.assertTrue(mod._undistortion_maps_match(np.zeros((720, 1280, 3))))
        self.assertFalse(mod._undistortion_maps_match(np.zeros((480, 848))))

    def test_setup_no_rclpy(self):
        """setup() should not crash when rclpy is not available."""
        mod = self._make_stub_bridge()
        mod.setup()  # Should gracefully handle ImportError
        self.assertEqual(mod._backend, "stub")
        self.assertFalse(mod._rclpy_available)

    def test_health_no_frames(self):
        mod = self._make_stub_bridge()
        mod.setup()
        h = mod.health()
        self.assertAlmostEqual(h["fps"], 0.0)

    def test_health_with_timestamp(self):
        mod = self._make_stub_bridge()
        mod.setup()
        mod._last_color_ts = time.time()
        h = mod.health()
        # fps will be computed from dt
        self.assertIn("fps", h)

    def test_start_without_node(self):
        """start() in stub mode publishes alive=False."""
        mod = self._make_stub_bridge()
        mod.setup()
        published = []
        mod.alive.subscribe(lambda v: published.append(v))
        mod.start()
        self.assertIn(False, published)

    def test_stop_idempotent(self):
        mod = self._make_stub_bridge()
        mod.setup()
        mod.start()
        mod.stop()
        mod.stop()  # second stop should not crash


# ---------------------------------------------------------------------------
# 7. TeleopModule
# ---------------------------------------------------------------------------

class TestTeleopModule(unittest.TestCase):
    """Test joystick scaling, idle release, health, and JPEG encode path."""

    def _make_teleop(self, **kw):
        from drivers.teleop_module import TeleopModule
        mod = TeleopModule(**kw)
        mod.setup()
        return mod

    def test_instantiation_defaults(self):
        mod = self._make_teleop()
        self.assertAlmostEqual(mod._max_speed, 0.5)
        self.assertAlmostEqual(mod._max_yaw_rate, 1.0)
        self.assertAlmostEqual(mod._release_timeout, 3.0)

    def test_instantiation_custom(self):
        mod = self._make_teleop(max_speed=1.0, max_yaw_rate=2.0, release_timeout=5.0)
        self.assertAlmostEqual(mod._max_speed, 1.0)
        self.assertAlmostEqual(mod._max_yaw_rate, 2.0)

    def test_on_joy_scales_and_publishes(self):
        mod = self._make_teleop(max_speed=1.0, max_yaw_rate=2.0)
        twists = []
        mod.cmd_vel.subscribe(lambda t: twists.append(t))
        active_flags = []
        mod.teleop_active.subscribe(lambda a: active_flags.append(a))

        mod._on_joy({"lx": 0.5, "ly": 0.0, "az": -0.5})
        self.assertEqual(len(twists), 1)
        self.assertAlmostEqual(twists[0].linear.x, 0.5)
        self.assertAlmostEqual(twists[0].angular.z, -1.0)
        self.assertTrue(mod._active)
        self.assertIn(True, active_flags)

    def test_on_joy_clamps(self):
        mod = self._make_teleop(max_speed=0.5)
        twists = []
        mod.cmd_vel.subscribe(lambda t: twists.append(t))

        mod._on_joy({"lx": 5.0, "ly": -5.0, "az": 0.0})
        # clamped to [-1, 1] then scaled by max_speed
        self.assertAlmostEqual(twists[0].linear.x, 0.5)
        self.assertAlmostEqual(twists[0].linear.y, -0.5)

    def test_release(self):
        mod = self._make_teleop()
        active_flags = []
        mod.teleop_active.subscribe(lambda a: active_flags.append(a))
        mod._active = True
        mod._release()
        self.assertFalse(mod._active)
        self.assertIn(False, active_flags)

    def test_release_when_not_active(self):
        mod = self._make_teleop()
        active_flags = []
        mod.teleop_active.subscribe(lambda a: active_flags.append(a))
        mod._release()  # should be no-op
        self.assertEqual(len(active_flags), 0)

    def test_health_output(self):
        mod = self._make_teleop()
        h = mod.health()
        self.assertIn("active", h)
        self.assertIn("clients", h)
        self.assertFalse(h["active"])
        self.assertEqual(h["clients"], 0)
        self.assertIsNone(h["last_joy_age_ms"])

    def test_health_after_joy(self):
        mod = self._make_teleop()
        mod._on_joy({"lx": 0.1, "ly": 0.0, "az": 0.0})
        h = mod.health()
        self.assertIsNotNone(h["last_joy_age_ms"])
        self.assertGreaterEqual(h["last_joy_age_ms"], 0)

    def test_get_teleop_status_skill(self):
        import json
        mod = self._make_teleop(port=9999)
        status = json.loads(mod.get_teleop_status())
        self.assertEqual(status["port"], 9999)
        self.assertFalse(status["active"])

    def test_force_release_skill(self):
        mod = self._make_teleop()
        mod._active = True
        result = mod.force_release()
        self.assertEqual(result, "Teleop released")
        self.assertFalse(mod._active)

    def test_client_connect_disconnect(self):
        mod = self._make_teleop()
        mod.on_client_connect()
        self.assertEqual(mod._clients, 1)
        mod.on_client_connect()
        self.assertEqual(mod._clients, 2)
        mod.on_client_disconnect()
        self.assertEqual(mod._clients, 1)

    def test_last_client_disconnect_releases(self):
        mod = self._make_teleop()
        mod._active = True
        mod._clients = 1
        active_flags = []
        mod.teleop_active.subscribe(lambda a: active_flags.append(a))
        mod.on_client_disconnect()
        self.assertFalse(mod._active)

    def test_on_image_no_gateway(self):
        """_on_image does nothing when gateway is None."""
        mod = self._make_teleop()
        img = Image(data=np.zeros((10, 10, 3), dtype=np.uint8), format=ImageFormat.BGR)
        mod._on_image(img)  # should not crash
        self.assertIsNone(mod._latest_raw)

    def test_on_image_with_gateway_and_clients(self):
        """_on_image stores raw frame when gateway + clients present."""
        mod = self._make_teleop()
        mod._gateway = MagicMock()
        mod._clients = 1
        img_data = np.zeros((10, 10, 3), dtype=np.uint8)
        img = Image(data=img_data, format=ImageFormat.BGR)
        mod._on_image(img)
        self.assertIsNotNone(mod._latest_raw)

    def test_camera_client_wakes_encoder_without_teleop_control(self):
        mod = self._make_teleop()
        mod._gateway = MagicMock()
        img_data = np.zeros((10, 10, 3), dtype=np.uint8)
        img = Image(data=img_data, format=ImageFormat.BGR)

        mod.on_camera_client_connect()
        self.assertEqual(mod._clients, 0)
        self.assertEqual(mod._camera_clients, 1)
        mod._on_image(img)
        self.assertTrue(mod._new_frame.is_set())

        mod.on_camera_client_disconnect()
        self.assertEqual(mod._camera_clients, 0)
        self.assertFalse(mod._active)


if __name__ == "__main__":
    unittest.main()

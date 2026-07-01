"""Tests for MapService -- map CRUD, POI operations, command dispatch.

All tests are pure-Python, no ROS2 / hardware required.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import pytest

from runtime.msgs.sensor import PointCloud2
from runtime.msgs.map import MapCloudFrame
from nav.services.maps import MapService
from nav.services.plan.global_planner.artifacts import SavedMapArtifacts


# -- fixtures ------------------------------------------------------------------


@pytest.fixture
def map_manager(tmp_path):
    """MapService with temp directories."""
    map_dir = tmp_path / "maps"
    data_dir = tmp_path / "data"
    mod = MapService(map_dir=str(map_dir), data_dir=str(data_dir))
    mod.setup()
    # Collect published responses
    responses: list[dict] = []
    events: list[dict] = []
    mod.map_response.subscribe(lambda r: responses.append(r))
    mod.map_event.subscribe(lambda e: events.append(e))
    mod._test_responses = responses
    mod._test_events = events
    return mod


def _cmd(mod, cmd: dict) -> dict:
    """Send a JSON command and return the last published response."""
    mod._test_responses.clear()
    mod._test_events.clear()
    mod._on_command(json.dumps(cmd))
    assert len(mod._test_responses) > 0, "no response published"
    return mod._test_responses[-1]


def _write_minimal_pcd(map_dir: Path) -> None:
    """Write a minimal valid PCD file into an existing map dir."""
    (map_dir / "map.pcd").write_text(
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        "COUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0.0 0.0 0.0\n",
        encoding="utf-8",
    )


def _fake_octomap_converter(tmp_path: Path) -> tuple[str, ...]:
    script = tmp_path / "fake_octomap_converter.py"
    script.write_text(
        "from __future__ import annotations\n"
        "import argparse\n"
        "from pathlib import Path\n"
        "parser = argparse.ArgumentParser()\n"
        "parser.add_argument('--input', required=True)\n"
        "parser.add_argument('--output', required=True)\n"
        "parser.add_argument('--resolution', required=True)\n"
        "parser.add_argument('--frame', required=True)\n"
        "args = parser.parse_args()\n"
        "Path(args.output).write_bytes(\n"
        "    f'FAKE_BT {args.resolution} {args.frame}\\n'.encode('utf-8')\n"
        "    + Path(args.input).read_bytes()\n"
        ")\n",
        encoding="utf-8",
    )
    return (
        sys.executable,
        str(script),
        "--input",
        "{input}",
        "--output",
        "{output}",
        "--resolution",
        "{resolution}",
        "--frame",
        "{frame}",
    )


# -- tests ---------------------------------------------------------------------


class TestMapService:
    """MapService instantiation, port types, command dispatch."""

    def test_instantiation(self, map_manager):
        """Module creates map_dir and data_dir on init."""
        assert os.path.isdir(map_manager._map_dir)
        assert os.path.isdir(map_manager._data_dir)

    def test_port_types(self, map_manager):
        """Verify In/Out port registration and types via port_summary."""
        s = map_manager.port_summary()
        assert "map_command" in s["ports_in"]
        assert s["ports_in"]["map_command"]["type"] == "str"
        assert "map_response" in s["ports_out"]
        assert s["ports_out"]["map_response"]["type"] == "dict"
        assert "map_event" in s["ports_out"]
        assert s["ports_out"]["map_event"]["type"] == "dict"
        assert "map_cloud" in s["ports_in"]
        assert s["ports_in"]["map_cloud"]["type"] == "PointCloud2"
        assert "map_cloud_frame" in s["ports_in"]
        assert s["ports_in"]["map_cloud_frame"]["type"] == "MapCloudFrame"

    def test_list_empty(self, map_manager):
        """list returns empty when no maps exist."""
        resp = _cmd(map_manager, {"action": "list"})
        assert resp["success"] is True
        assert resp["maps"] == []
        assert map_manager._test_events == []

    def test_create_publishes_map_event(self, map_manager):
        resp = _cmd(map_manager, {"action": "create", "name": "event_map"})

        assert resp["success"] is True
        assert len(map_manager._test_events) == 1
        event = map_manager._test_events[-1]
        assert event["schema_version"] == "map.event"
        assert event["event"] == "map.created"
        assert event["map_id"] == "event_map"
        assert event["record_version"] == "event_map:v1"

    def test_get_map_types_exposes_stable_catalog(self, map_manager):
        """MapService exposes the map type contract for UI and planners."""
        resp = _cmd(map_manager, {"action": "get_map_types"})

        assert resp["success"] is True
        assert resp["schema_version"] == "map.types"
        assert resp["catalog"]["schema_version"] == "map.types"
        assert resp["record_schema_version"] == "map.record"
        assert resp["catalog"]["bundle_schema_version"] == "map.bundle"
        assert "READY" in resp["states"]
        assert "global_3d_occupancy" in resp["classes"]
        assert resp["artifacts"]["octomap"]["type"] == "OCTOMAP_3D"
        assert resp["artifacts"]["occupancy_grid"]["filename"] == "occupancy.npz"
        assert resp["aliases"]["occupancy"] == "occupancy_grid"
        assert resp["capabilities"]["navigation_safety_3d"] == "OCTOMAP_3D"

    def test_delete_nonexistent(self, map_manager):
        """delete on a non-existent map returns failure."""
        resp = _cmd(map_manager, {"action": "delete", "name": "no_such_map"})
        assert resp["success"] is False
        assert "not found" in resp["message"]

    def test_rename_creates_and_renames(self, map_manager):
        """rename a map directory and verify files move."""
        src = Path(map_manager._map_dir) / "alpha"
        src.mkdir()
        (src / "map.pcd").touch()
        resp = _cmd(map_manager, {"action": "rename", "name": "alpha", "new_name": "beta"})
        assert resp["success"] is True
        assert not src.exists()
        assert (Path(map_manager._map_dir) / "beta" / "map.pcd").exists()

    def test_set_active_validates_artifacts(self, map_manager):
        """set_active rejects a map without metadata artifacts."""
        d = Path(map_manager._map_dir) / "badmap"
        d.mkdir()
        (d / "map.pcd").write_text("VERSION 0.7\nPOINTS 0\nDATA ascii\n")
        (d / "tomogram.pickle").write_bytes(b"bad")
        resp = _cmd(map_manager, {"action": "set_active", "name": "badmap"})
        assert resp["success"] is False
        assert "saved map artifact gate failed" in resp["message"]
        assert map_manager._test_events[-1]["event"] == "map.validation_failed"

    def test_poi_set_list_delete(self, map_manager):
        """POI CRUD: set, list, then delete."""
        resp = _cmd(map_manager, {"action": "poi_set", "name": "home", "x": 1.0, "y": 2.0})
        assert resp["success"] is True

        resp = _cmd(map_manager, {"action": "poi_list"})
        assert "home" in resp["pois"]

        resp = _cmd(map_manager, {"action": "poi_delete", "name": "home"})
        assert resp["success"] is True

        resp = _cmd(map_manager, {"action": "poi_list"})
        assert "home" not in resp["pois"]

    def test_poi_delete_nonexistent(self, map_manager):
        """delete a POI that does not exist."""
        resp = _cmd(map_manager, {"action": "poi_delete", "name": "nope"})
        assert resp["success"] is False

    def test_unknown_action(self, map_manager):
        """unknown action returns error."""
        resp = _cmd(map_manager, {"action": "foobar"})
        assert resp["success"] is False
        assert resp["reason_code"] == "unknown_action"
        assert "unknown" in resp.get("message", "")

    def test_failed_map_commands_include_reason_code(self, map_manager):
        resp = _cmd(map_manager, {"action": "delete", "name": "no_such_map"})

        assert resp["success"] is False
        assert resp["reason_code"] == "map_not_found"

    def test_build_artifact_accepts_capability_and_map_class(
        self,
        map_manager,
        monkeypatch,
    ):
        calls: list[str] = []

        def fake_occupancy(name):
            calls.append(f"occupancy:{name}")
            return {"action": "build_occupancy_snapshot", "success": True}

        def fake_octomap(name):
            calls.append(f"octomap:{name}")
            return {"action": "build_octomap", "success": True}

        monkeypatch.setattr(map_manager.pipeline, "build_occupancy_snapshot", fake_occupancy)
        monkeypatch.setattr(map_manager.pipeline, "build_octomap_artifact", fake_octomap)

        by_capability = _cmd(
            map_manager,
            {
                "action": "build_artifact",
                "name": "demo",
                "capability": "path_planning_2d",
            },
        )
        by_class = _cmd(
            map_manager,
            {
                "action": "build_artifact",
                "name": "demo",
                "map_class": "global_3d_occupancy",
            },
        )

        assert by_capability["success"] is True
        assert by_class["success"] is True
        assert calls == ["occupancy:demo", "octomap:demo"]

    def test_build_artifact_reports_unavailable_modeled_types(self, map_manager):
        resp = _cmd(
            map_manager,
            {
                "action": "build_artifact",
                "name": "demo",
                "capability": "trajectory_optimization",
            },
        )

        assert resp["success"] is False
        assert resp["reason_code"] == "builder_unavailable"
        assert resp["artifact_type"] == "ESDF"

    def test_command_router_uses_facade_save_wrapper(self, map_manager, monkeypatch):
        """map_command save remains compatible with facade wrapper overrides."""
        called = {}

        def fake_save(name, slam_profile=None):
            called["args"] = (name, slam_profile)
            return {"action": "save", "success": True, "map_id": name}

        monkeypatch.setattr(map_manager, "_map_save", fake_save)

        resp = _cmd(
            map_manager,
            {"action": "save", "name": "patched", "slam_profile": "localizer"},
        )

        assert resp["success"] is True
        assert called["args"] == ("patched", "localizer")

    def test_invalid_json(self, map_manager):
        """malformed JSON is handled gracefully."""
        map_manager._test_responses.clear()
        map_manager._on_command("not json{{{")
        assert len(map_manager._test_responses) > 0
        resp = map_manager._test_responses[-1]
        assert resp["success"] is False

    @pytest.mark.parametrize(
        "action,payload",
        [
            ("save", {"name": "../outside"}),
            ("delete", {"name": "../outside"}),
            ("set_active", {"name": "../outside"}),
            ("get_health", {"name": "../outside"}),
            ("get_map_bundle", {"name": "../outside", "capability": "path_planning"}),
            ("rename", {"name": "valid", "new_name": "../outside"}),
            ("save", {"name": ".hidden"}),
            ("save", {"name": "-dash"}),
            ("save", {"name": "bad/name"}),
            ("save", {"name": "bad\\name"}),
        ],
    )
    def test_map_command_rejects_unsafe_map_names(
        self,
        map_manager,
        tmp_path,
        action,
        payload,
    ):
        outside = Path(map_manager._map_dir).parent / "outside"
        resp = _cmd(map_manager, {"action": action, **payload})

        assert resp["success"] is False
        assert "invalid map name" in resp["message"]
        assert not outside.exists()

    def test_map_command_rejects_absolute_map_name(self, map_manager, tmp_path):
        absolute_name = str(tmp_path / "outside")
        resp = _cmd(map_manager, {"action": "save", "name": absolute_name})

        assert resp["success"] is False
        assert "invalid map name" in resp["message"]
        assert not Path(absolute_name).exists()

    def test_get_record_rejects_metadata_artifact_path_escape(self, map_manager):
        from runtime.same_source_map_artifacts import sha256_file

        map_dir = Path(map_manager._map_dir) / "safe_map"
        map_dir.mkdir()
        (map_dir / "map.pcd").write_text("VERSION 0.7\nPOINTS 0\nDATA ascii\n")
        outside = Path(map_manager._map_dir).parent / "outside.npz"
        outside.write_bytes(b"outside grid")
        (map_dir / "metadata.json").write_text(
            json.dumps(
                {
                    "artifacts": {
                        "occupancy_grid": {
                            "path": "../outside.npz",
                            "sha256": sha256_file(outside),
                            "source_map_sha256": "not-relevant",
                        }
                    }
                }
            ),
            encoding="utf-8",
        )
        (map_dir / "map_record.json").write_text(
            json.dumps(
                {
                    "artifacts": [
                        {
                            "type": "OCCUPANCY_2D",
                            "uri": str(outside),
                            "hash": sha256_file(outside),
                            "source_map_id": "safe_map",
                            "generator": "malicious",
                            "build_config": {},
                        }
                    ],
                    "capabilities": ["path_planning_2d"],
                    "health": {"active_allowed": True},
                }
            ),
            encoding="utf-8",
        )

        record_resp = _cmd(map_manager, {"action": "get_record", "name": "safe_map"})
        assert record_resp["success"] is True
        artifact_uris = {
            str(Path(artifact["uri"]).resolve())
            for artifact in record_resp["record"]["artifacts"]
        }
        assert str(outside.resolve()) not in artifact_uris
        for uri in artifact_uris:
            Path(uri).resolve().relative_to(map_dir.resolve())

        bundle_resp = _cmd(
            map_manager,
            {
                "action": "get_map_bundle",
                "name": "safe_map",
                "capability": "path_planning_2d",
            },
        )
        assert bundle_resp["success"] is False

    def test_on_map_cloud_stores_finite_points(self, map_manager):
        """map_cloud is filtered to finite XYZ points."""
        import numpy as np

        cloud = PointCloud2.from_numpy(
            np.array([[1.0, 2.0, 0.1], [3.0, 4.0, 0.2], [np.inf, np.nan, 0.3]], dtype=np.float32)
        )
        map_manager._on_map_cloud(cloud)
        with map_manager._map_cloud_lock:
            stored = map_manager._latest_map_points
        assert stored is not None
        assert stored.shape == (2, 3)

    def test_on_map_cloud_frame_full_replaces_points(self, map_manager):
        """FULL map frames replace the live snapshot."""
        import numpy as np

        map_manager._on_map_cloud_frame(
            MapCloudFrame(
                points=np.array([[1.0, 2.0, 0.1], [3.0, 4.0, 0.2]], dtype=np.float32),
                mode="FULL",
                frame_id="map",
            )
        )
        map_manager._on_map_cloud_frame(
            MapCloudFrame(
                points=np.array([[9.0, 8.0, 0.3]], dtype=np.float32),
                mode="FULL",
                frame_id="map",
            )
        )

        with map_manager._map_cloud_lock:
            stored = map_manager._latest_map_points
            info = dict(map_manager.runtime_bridge.latest_map_frame_info)
        assert stored is not None
        assert stored.shape == (1, 3)
        np.testing.assert_allclose(stored, [[9.0, 8.0, 0.3]])
        assert info["mode"] == "FULL"
        assert info["stored_points"] == 1

    def test_on_map_cloud_frame_keyframe_and_incremental_append(self, map_manager):
        """KEYFRAME and INCREMENTAL frames append to the live map snapshot."""
        import numpy as np

        map_manager._on_map_cloud_frame(
            MapCloudFrame(
                points=np.array([[1.0, 2.0, 0.1], [float("inf"), 0.0, 0.0]], dtype=np.float32),
                mode="KEYFRAME",
                frame_id="map",
            )
        )
        map_manager._on_map_cloud_frame(
            MapCloudFrame(
                points=np.array([[3.0, 4.0, 0.2]], dtype=np.float32),
                mode="INCREMENTAL",
                frame_id="map",
            )
        )

        with map_manager._map_cloud_lock:
            stored = map_manager._latest_map_points
            info = dict(map_manager.runtime_bridge.latest_map_frame_info)
        assert stored is not None
        assert stored.shape == (2, 3)
        assert info["mode"] == "INCREMENTAL"
        assert info["stored_points"] == 2

    def test_on_map_cloud_frame_accepts_wire_dict(self, map_manager):
        """Dict payloads can enter through bridge adapters before full typing."""
        import numpy as np

        payload = MapCloudFrame(
            points=np.array([[1.0, 2.0, 3.0]], dtype=np.float32),
            mode="FULL",
            frame_id="map",
            source="dds_adapter",
        ).to_dict(include_points=True)

        map_manager._on_map_cloud_frame(payload)

        with map_manager._map_cloud_lock:
            stored = map_manager._latest_map_points
            info = dict(map_manager.runtime_bridge.latest_map_frame_info)
        assert stored is not None
        assert stored.shape == (1, 3)
        assert info["source"] == "dds_adapter"

    def test_map_cloud_frame_save_produces_global_planner_artifact(
        self,
        map_manager,
        tmp_path,
        monkeypatch,
    ):
        """Typed SLAM map frames can become the active OctoPlanner3D map."""
        import numpy as np

        monkeypatch.setenv("NAV_MAP_DIR", str(map_manager._map_dir))
        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)
        map_manager._on_map_cloud_frame(
            MapCloudFrame(
                points=np.array(
                    [
                        [0.0, 0.0, 0.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [1.0, 1.0, 0.0],
                        [0.0, 0.0, 0.5],
                        [1.0, 0.0, 0.5],
                        [0.0, 1.0, 0.5],
                        [1.0, 1.0, 0.5],
                    ],
                    dtype=np.float32,
                ),
                mode="FULL",
                frame_id="map",
                source="native_slam:test",
            )
        )

        saved = _cmd(
            map_manager,
            {"action": "save", "name": "frame_map", "slam_profile": "super_lio"},
        )
        assert saved["success"] is True, saved
        assert saved["octomap_ok"] is True, saved
        assert saved["occupancy_ok"] is True, saved
        assert b"DATA binary\n" in Path(saved["pcd"]).read_bytes()

        active = _cmd(map_manager, {"action": "set_active", "name": "frame_map"})
        assert active["success"] is True, active

        artifacts = SavedMapArtifacts.from_runtime()
        bundle = artifacts.planner_map_bundle("octoplanner3d")
        assert bundle["schema_version"] == "map.bundle"
        assert bundle["capability"] == "navigation_safety_3d"
        assert bundle["artifact"]["type"] == "OCTOMAP_3D"
        assert artifacts.planner_map_path("octoplanner3d").endswith("octomap.bt")

    def test_get_active_tomogram_none(self, map_manager):
        """get_active_tomogram returns None when no active map."""
        result = map_manager.get_active_tomogram()
        assert result is None

    def test_map_list_reports_has_occupancy(self, map_manager):
        """_map_list reports has_occupancy when occupancy.npz exists."""
        d = Path(map_manager._map_dir) / "mapwithall"
        d.mkdir()
        (d / "map.pcd").touch()
        (d / "tomogram.pickle").touch()
        (d / "occupancy.npz").touch()
        (d / "octomap.bt").touch()

        resp = _cmd(map_manager, {"action": "list"})
        entry = next(m for m in resp["maps"] if m["name"] == "mapwithall")
        assert entry["has_occupancy"] is True
        assert entry["has_octomap"] is True
        assert "global_3d_occupancy" in entry["map_classes"]
        assert entry["artifacts"]["octomap"]["role"] == "octoplanner3d_global_planning"
        assert entry["artifacts"]["occupancy_grid"]["map_class"] == "static_2d_occupancy"
        assert entry["artifacts"]["occupancy"]["map_class"] == "static_2d_occupancy"

    def test_build_octomap_command_writes_artifact_and_metadata(
        self,
        map_manager,
        tmp_path,
    ):
        """build_octomap creates octomap.bt and metadata without planning-time conversion."""
        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)
        d = Path(map_manager._map_dir) / "octomap_source"
        d.mkdir()
        _write_minimal_pcd(d)

        resp = _cmd(map_manager, {"action": "build_octomap", "name": "octomap_source"})

        assert resp["success"] is True, resp
        assert resp["status"] == "built"
        assert Path(resp["octomap"]).is_file()
        metadata_path = Path(resp["metadata"]["path"])
        assert metadata_path.is_file()
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        assert metadata["build_mode"] == "external_pcl_converter"
        assert metadata["artifacts"]["octomap"]["path"] == "octomap.bt"
        assert metadata["artifacts"]["octomap"]["source_map_sha256"] == metadata["artifacts"]["map_pcd"]["sha256"]
        record_path = Path(resp["metadata"]["map_record"])
        assert record_path.is_file()
        record = json.loads(record_path.read_text(encoding="utf-8"))
        assert record["schema_version"] == "map.record"
        assert record["map_id"] == "octomap_source"
        assert record["version_id"] == "octomap_source:v1"
        assert "navigation_safety_3d" in record["capabilities"]
        octomap = next(item for item in record["artifacts"] if item["type"] == "OCTOMAP_3D")
        assert octomap["name"] == "octomap"
        assert octomap["map_class"] == "global_3d_occupancy"
        assert octomap["capability"] == "navigation_safety_3d"
        assert record["health"]["active_allowed"] is True

    def test_build_octomap_fails_when_post_build_gate_fails(
        self,
        map_manager,
        tmp_path,
        monkeypatch,
    ):
        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)
        d = Path(map_manager._map_dir) / "bad_gate"
        d.mkdir()
        _write_minimal_pcd(d)

        def fake_gate(*_args, **_kwargs):
            return {"ok": False, "blockers": ["octomap path escapes map directory"]}

        monkeypatch.setattr(
            "nav.services.map.pipeline.validate_saved_map_artifact_dir",
            fake_gate,
        )

        resp = _cmd(map_manager, {"action": "build_octomap", "name": "bad_gate"})

        assert resp["success"] is False
        assert resp["metadata"]["ok"] is False
        assert resp["metadata"]["map_record"] is None
        assert "saved map artifact gate failed" in resp["message"]
        assert "octomap path escapes map directory" in resp["message"]

    def test_set_active_accepts_octomap_ready_map_without_tomogram(
        self,
        map_manager,
        tmp_path,
    ):
        """OctoPlanner3D-ready maps can be active even before legacy tomogram exists."""
        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)
        d = Path(map_manager._map_dir) / "octomap_only"
        d.mkdir()
        _write_minimal_pcd(d)
        build = _cmd(map_manager, {"action": "build_octomap", "name": "octomap_only"})
        assert build["success"] is True, build

        resp = _cmd(map_manager, {"action": "set_active", "name": "octomap_only"})

        assert resp["success"] is True, resp
        assert resp["active"] == "octomap_only"
        assert resp["octomap"].endswith("octomap.bt")
        assert map_manager.get_active_octomap().endswith("octomap.bt")
        artifacts = map_manager.get_active_artifacts()
        assert artifacts["octomap"].endswith("octomap.bt")
        assert artifacts["tomogram"] is None
        assert artifacts["artifacts"]["octomap"]["exists"] is True
        assert "global_3d_occupancy" in artifacts["map_classes"]
        assert artifacts["record"]["state"] == "ACTIVE"

    def test_get_map_bundle_returns_artifact_for_capability(
        self,
        map_manager,
        tmp_path,
    ):
        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)
        d = Path(map_manager._map_dir) / "bundle_map"
        d.mkdir()
        _write_minimal_pcd(d)
        build = _cmd(map_manager, {"action": "build_octomap", "name": "bundle_map"})
        assert build["success"] is True, build

        resp = _cmd(
            map_manager,
            {
                "action": "get_map_bundle",
                "name": "bundle_map",
                "capability": "navigation_safety_3d",
            },
        )

        assert resp["success"] is True, resp
        assert resp["schema_version"] == "map.bundle"
        assert resp["map_id"] == "bundle_map"
        assert resp["version_id"] == "bundle_map:v1"
        assert resp["state"] == "READY"
        assert resp["frame_id"]
        assert resp["artifact"]["type"] == "OCTOMAP_3D"
        assert resp["artifact"]["map_class"] == "global_3d_occupancy"
        assert "navigation_safety_3d" in resp["available_capabilities"]
        assert any(item["type"] == "POINTCLOUD" for item in resp["artifacts"])
        assert resp["record"]["schema_version"] == "map.record"
        assert resp["artifact"]["uri"].endswith("octomap.bt")

    def test_get_health_rejects_unknown_map(self, map_manager):
        resp = _cmd(map_manager, {"action": "get_health", "name": "ghost"})

        assert resp["success"] is False
        assert "map not found" in resp["message"]

    def test_retired_map_cannot_be_activated(self, map_manager, tmp_path):
        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)
        d = Path(map_manager._map_dir) / "old_map"
        d.mkdir()
        _write_minimal_pcd(d)
        build = _cmd(map_manager, {"action": "build_octomap", "name": "old_map"})
        assert build["success"] is True, build
        retired = _cmd(map_manager, {"action": "retire", "name": "old_map"})
        assert retired["success"] is True
        assert retired["record"]["state"] == "RETIRED"

        resp = _cmd(map_manager, {"action": "set_active", "name": "old_map"})

        assert resp["success"] is False
        assert "retired" in resp["message"]

    def test_active_rename_preserves_active_record_state(self, map_manager, tmp_path):
        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)
        d = Path(map_manager._map_dir) / "active_old"
        d.mkdir()
        _write_minimal_pcd(d)
        build = _cmd(map_manager, {"action": "build_octomap", "name": "active_old"})
        assert build["success"] is True, build
        active = _cmd(map_manager, {"action": "set_active", "name": "active_old"})
        assert active["success"] is True, active

        renamed = _cmd(
            map_manager,
            {"action": "rename", "name": "active_old", "new_name": "active_new"},
        )

        assert renamed["success"] is True, renamed
        record = _cmd(map_manager, {"action": "get_record", "name": "active_new"})
        assert record["record"]["state"] == "ACTIVE"

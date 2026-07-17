"""Tests for MapsModule -- map CRUD, POI operations, command dispatch.

All tests are pure-Python, no ROS2 / hardware required.
"""

from __future__ import annotations

import json
import os
import struct
import sys
from pathlib import Path

import pytest

from maps.artifacts import sha256_file
from maps.modules.service import MapsModule
from nav.services.plan.global_planner.artifacts import SavedMapArtifacts
from runtime.msgs.map import MapCloudFrame, MapControlRequest, SemanticSaveResult
from runtime.msgs.sensor import PointCloud2

# -- fixtures ------------------------------------------------------------------


@pytest.fixture
def map_manager(tmp_path, monkeypatch):
    """MapsModule with temp directories."""
    # Unit tests do not build the native prune executable. Tests that exercise
    # the dynamic-filter transaction opt in explicitly with a fake cleaner.
    monkeypatch.setenv("LINGTU_SAVE_DYNAMIC_FILTER", "0")
    map_dir = tmp_path / "maps"
    data_dir = tmp_path / "data"
    mod = MapsModule(map_dir=str(map_dir), data_dir=str(data_dir))
    mod.setup()
    # Collect published responses
    responses: list[dict] = []
    events: list[dict] = []
    mod.map_response.subscribe(lambda r: responses.append(r))
    mod.map_event.subscribe(lambda e: events.append(e))
    mod._test_responses = responses
    mod._test_events = events
    yield mod
    mod.stop()


def _cmd(mod, cmd: dict) -> dict:
    """Send a typed control request and return the published response."""
    mod._test_responses.clear()
    mod._test_events.clear()
    mod._on_command(MapControlRequest.from_mapping(cmd))
    assert len(mod._test_responses) > 0, "no response published"
    return mod._test_responses[-1]


def _write_minimal_pcd(map_dir: Path) -> None:
    """Write a minimal valid PCD file into an existing map dir."""
    (map_dir / "map.pcd").write_text(
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        "COUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0.0 0.0 0.0\n",
        encoding="utf-8",
    )


def _write_minimal_semantic_map(path: Path, *, corrupt: bool = False) -> None:
    """Write a minimal semantic_map.bin matching the native persistence contract."""
    body = b"map" + b"lingtu.semantic"
    checksum = 14695981039346656037
    for byte in body:
        checksum ^= byte
        checksum = (checksum * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    if corrupt:
        checksum ^= 1
    header = struct.pack(
        "<8sIIIIIIfQQQQ",
        b"LTSEMAP\0",
        1,
        0x01020304,
        68,
        3,
        len(b"lingtu.semantic"),
        1,
        0.2,
        7,
        0,
        len(body),
        checksum,
    )
    path.write_bytes(header + body)


def _has_save_source_staging(map_dir: Path) -> bool:
    builds = map_dir / ".builds"
    if not builds.is_dir():
        return False
    return any(path.name.startswith("save_source_") for path in builds.iterdir())


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
        "parser.add_argument('--free-layers-above', required=True)\n"
        "parser.add_argument('--free-dilation-cells', required=True)\n"
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
        "--free-layers-above",
        "{free_layers_above}",
        "--free-dilation-cells",
        "{free_dilation_cells}",
        "--frame",
        "{frame}",
    )


def _fake_map_optimizer(tmp_path: Path) -> tuple[str, ...]:
    script = tmp_path / "fake_map_optimizer.py"
    script.write_text(
        "from __future__ import annotations\n"
        "import argparse\n"
        "import json\n"
        "from pathlib import Path\n"
        "parser = argparse.ArgumentParser()\n"
        "parser.add_argument('--map', required=True)\n"
        "parser.add_argument('--out', required=True)\n"
        "args = parser.parse_args()\n"
        "root = Path(args.map)\n"
        "pcd = root / 'map.pcd'\n"
        "pcd.write_text(\n"
        "    'VERSION 0.7\\nFIELDS x y z\\nSIZE 4 4 4\\nTYPE F F F\\n'\n"
        "    'COUNT 1 1 1\\nWIDTH 2\\nHEIGHT 1\\nPOINTS 2\\nDATA ascii\\n'\n"
        "    '0.0 0.0 0.0\\n1.0 0.0 0.0\\n',\n"
        "    encoding='utf-8',\n"
        ")\n"
        "patch_count = len(list((root / 'patches').glob('*.pcd')))\n"
        "print(json.dumps({\n"
        "    'ok': True,\n"
        "    'code': 'optimized',\n"
        "    'message': 'fake optimizer ok',\n"
        "    'patch_count': patch_count,\n"
        "}))\n",
        encoding="utf-8",
    )
    return (
        sys.executable,
        str(script),
        "--map",
        "{map}",
        "--out",
        "{out}",
    )


def _fake_dynamic_filter(tmp_path: Path) -> tuple[str, ...]:
    script = tmp_path / "fake_dynamic_filter.py"
    script.write_text(
        "from __future__ import annotations\n"
        "import argparse\n"
        "import shutil\n"
        "from pathlib import Path\n"
        "parser = argparse.ArgumentParser()\n"
        "parser.add_argument('--map-dir', required=True)\n"
        "parser.add_argument('--overwrite', action='store_true')\n"
        "parser.add_argument('--apply', action='store_true')\n"
        "args = parser.parse_args()\n"
        "root = Path(args.map_dir)\n"
        "pcd = root / 'map.pcd'\n"
        "before = pcd.read_text(encoding='utf-8')\n"
        "if 'POINTS 2\\n' not in before:\n"
        "    raise SystemExit('dynamic filter ran before optimizer')\n"
        "clean = (\n"
        "    'VERSION 0.7\\nFIELDS x y z\\nSIZE 4 4 4\\nTYPE F F F\\n'\n"
        "    'COUNT 1 1 1\\nWIDTH 1\\nHEIGHT 1\\nPOINTS 1\\nDATA ascii\\n'\n"
        "    '1.0 0.0 0.0\\n'\n"
        ")\n"
        "shutil.copy2(pcd, root / 'map.pcd.preclean')\n"
        "(root / 'map.clean.pcd').write_text(clean, encoding='utf-8')\n"
        "pcd.write_text(clean, encoding='utf-8')\n",
        encoding="utf-8",
    )
    return (
        sys.executable,
        str(script),
        "--map-dir",
        "{map}",
    )


def _fake_octomap_editor(tmp_path: Path) -> tuple[str, ...]:
    script = tmp_path / "fake_octomap_editor.py"
    script.write_text(
        "from __future__ import annotations\n"
        "import argparse\n"
        "from pathlib import Path\n"
        "parser = argparse.ArgumentParser()\n"
        "parser.add_argument('--map', required=True)\n"
        "parser.add_argument('--output', required=True)\n"
        "parser.add_argument('--state', required=True)\n"
        "parser.add_argument('--x', required=True)\n"
        "parser.add_argument('--y', required=True)\n"
        "parser.add_argument('--z', required=True)\n"
        "parser.add_argument('--radius', required=True)\n"
        "parser.add_argument('--shape', required=True)\n"
        "args = parser.parse_args()\n"
        "Path(args.output).write_bytes(Path(args.map).read_bytes() + b'\\nedited')\n"
        'print(\'{"ok":true,"edited_voxels":7,"effective_state":"occupied"}\')\n',
        encoding="utf-8",
    )
    return (
        sys.executable,
        str(script),
        "--map",
        "{map}",
        "--output",
        "{output}",
        "--state",
        "{state}",
        "--x",
        "{x}",
        "--y",
        "{y}",
        "--z",
        "{z}",
        "--radius",
        "{radius}",
        "--shape",
        "{shape}",
    )


# -- tests ---------------------------------------------------------------------


class TestMapsModule:
    """MapsModule instantiation, port types, command dispatch."""

    def test_instantiation(self, map_manager):
        """Module creates map_dir and data_dir on init."""
        assert os.path.isdir(map_manager._map_dir)
        assert os.path.isdir(map_manager._data_dir)

    def test_port_types(self, map_manager):
        """Verify In/Out port registration and types via port_summary."""
        s = map_manager.port_summary()
        assert "map_command" in s["ports_in"]
        assert s["ports_in"]["map_command"]["type"] == "MapControlRequest"
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
        """MapsModule exposes the map type contract for UI and planners."""
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
        assert resp["artifacts"]["semantic"]["filename"] == "semantic_map.bin"
        assert resp["aliases"]["occupancy"] == "occupancy_grid"
        assert resp["capabilities"]["navigation_safety_3d"] == "OCTOMAP_3D"

    def test_delete_nonexistent(self, map_manager):
        """delete on a non-existent map returns failure."""
        resp = _cmd(map_manager, {"action": "delete", "name": "no_such_map"})
        assert resp["success"] is False
        assert "not found" in resp["message"]

    def test_delete_does_not_bypass_native_failure(
        self,
        map_manager,
        monkeypatch,
    ):
        map_dir = Path(map_manager._map_dir) / "root_owned"
        patches = map_dir / "patches"
        patches.mkdir(parents=True)
        (patches / "scan_000001.pcd").write_text("pcd", encoding="utf-8")

        monkeypatch.setattr(
            map_manager.storage,
            "delete_map",
            lambda _name: {"success": False, "message": "permission denied"},
        )

        resp = _cmd(map_manager, {"action": "delete", "name": "root_owned"})

        assert resp["success"] is False
        assert resp["message"] == "permission denied"
        assert "delete_method" not in resp
        assert map_dir.exists()

    def test_rename_creates_and_renames(self, map_manager):
        """rename a map directory and verify files move."""
        src = Path(map_manager._map_dir) / "alpha"
        src.mkdir()
        (src / "map.pcd").touch()
        resp = _cmd(map_manager, {"action": "rename", "name": "alpha", "new_name": "beta"})
        assert resp["success"] is True
        assert not src.exists()
        assert (Path(map_manager._map_dir) / "beta" / "map.pcd").exists()

    def test_restore_source_backup_invalidates_derived_artifacts(self, map_manager):
        map_dir = Path(map_manager._map_dir) / "restore_me"
        map_dir.mkdir()
        _write_minimal_pcd(map_dir)
        original = (map_dir / "map.pcd").read_bytes()
        (map_dir / "map.pcd.preclean").write_bytes(original)
        (map_dir / "map.pcd").write_text("cleaned", encoding="utf-8")
        for filename in (
            "occupancy.npz",
            "octomap.ot",
            "metadata.json",
            "voxel_edits.jsonl",
        ):
            (map_dir / filename).write_text("stale", encoding="utf-8")

        resp = _cmd(map_manager, {"action": "restore_source", "name": "restore_me"})

        assert resp["success"] is True
        assert resp["state"] == "STALE"
        assert resp["transactional_visibility"] == "staged_until_commit"
        assert (map_dir / "map.pcd").read_bytes() == original
        assert not list(map_dir.glob("map.pcd.replaced-*"))
        assert not (map_dir / "occupancy.npz").exists()
        assert not (map_dir / "octomap.ot").exists()
        metadata = json.loads((map_dir / "metadata.json").read_text(encoding="utf-8"))
        assert metadata["metadata_state"] == "invalidated"
        assert not (map_dir / "voxel_edits.jsonl").exists()
        record = _cmd(map_manager, {"action": "get_record", "name": "restore_me"})
        assert record["record"]["state"] == "STALE"

    def test_restore_source_backup_deactivates_selected_map(self, map_manager):
        map_dir = Path(map_manager._map_dir) / "active_restore"
        map_dir.mkdir()
        _write_minimal_pcd(map_dir)
        (map_dir / "map.pcd.predufo").write_bytes((map_dir / "map.pcd").read_bytes())
        (map_dir / "map.pcd").write_text("cleaned", encoding="utf-8")
        map_manager.storage.native_store.set_active_map("active_restore", strict=False)
        map_manager.storage.active_map = "active_restore"

        resp = _cmd(
            map_manager,
            {"action": "restore_source_backup", "name": "active_restore"},
        )

        assert resp["success"] is True
        assert resp["deactivated"] is True
        assert map_manager.storage.active_map == ""
        assert map_manager.storage.native_store.active_map_id() == ""

    def test_restore_source_backup_requires_backup(self, map_manager):
        map_dir = Path(map_manager._map_dir) / "no_backup"
        map_dir.mkdir()
        (map_dir / "map.pcd").write_text("current", encoding="utf-8")

        resp = _cmd(map_manager, {"action": "restore_source", "name": "no_backup"})

        assert resp["success"] is False
        assert resp["reason_code"] == "source_backup_missing"

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
        created = _cmd(map_manager, {"action": "create", "name": "poi_map"})
        assert created["success"] is True

        resp = _cmd(
            map_manager,
            {
                "action": "poi_set",
                "map_id": "poi_map",
                "name": "home",
                "x": 1.0,
                "y": 2.0,
                "tags": {"kind": "dock"},
            },
        )
        assert resp["success"] is True

        resp = _cmd(map_manager, {"action": "poi_list", "map_id": "poi_map"})
        assert "home" in resp["pois"]
        assert resp["pois"]["home"]["tags"]["kind"] == "dock"

        resp = _cmd(
            map_manager,
            {"action": "poi_delete", "map_id": "poi_map", "name": "home"},
        )
        assert resp["success"] is True

        resp = _cmd(map_manager, {"action": "poi_list", "map_id": "poi_map"})
        assert "home" not in resp["pois"]

    def test_poi_delete_nonexistent(self, map_manager):
        """delete a POI that does not exist."""
        created = _cmd(map_manager, {"action": "create", "name": "poi_map"})
        assert created["success"] is True
        resp = _cmd(
            map_manager,
            {"action": "poi_delete", "map_id": "poi_map", "name": "nope"},
        )
        assert resp["success"] is False

    def test_map_graph_rollback_and_build_queue_are_native(self, map_manager):
        first = _cmd(map_manager, {"action": "create", "name": "floor_a"})
        second = _cmd(map_manager, {"action": "create", "name": "floor_b"})
        assert first["success"] is True
        assert second["success"] is True

        edge = _cmd(
            map_manager,
            {
                "action": "set_map_edge",
                "from": "floor_a",
                "to": "floor_b",
                "type": "stairs",
            },
        )
        assert edge["success"] is True
        graph = _cmd(map_manager, {"action": "map_graph"})
        assert graph["edges"][0]["from"] == "floor_a"
        assert graph["edges"][0]["to"] == "floor_b"
        assert graph["edges"][0]["type"] == "stairs"

        queued = _cmd(
            map_manager,
            {
                "action": "enqueue_build",
                "name": "floor_a",
                "artifact_type": "OCTOMAP_3D",
            },
        )
        assert queued["success"] is True
        queue = _cmd(map_manager, {"action": "build_queue"})
        assert queue["items"][0]["map_id"] == "floor_a"
        assert queue["items"][0]["state"] in {
            "QUEUED",
            "RUNNING",
            "SUCCEEDED",
            "FAILED",
        }

        # Use native service directly here because these maps are intentionally
        # missing strict planning artifacts; rollback itself is native.
        active_a = map_manager.storage.native_service.set_active_map("floor_a", strict=False)
        active_b = map_manager.storage.native_service.set_active_map("floor_b", strict=False)
        assert active_a["success"] is True
        assert active_b["success"] is True
        rolled = _cmd(map_manager, {"action": "rollback_active"})
        assert rolled["success"] is True
        assert rolled["active"] == "floor_a"

    def test_edit_voxels_uses_native_transaction(self, tmp_path, monkeypatch):
        """OctoMap, metadata, and journal publish as one native transaction."""
        mod = MapsModule(
            map_dir=str(tmp_path / "maps"),
            data_dir=str(tmp_path / "data"),
            map_artifact_converter_command=_fake_octomap_converter(tmp_path),
            octomap_editor_command=_fake_octomap_editor(tmp_path),
        )
        mod.setup()
        responses: list[dict] = []
        events: list[dict] = []
        mod.map_response.subscribe(lambda r: responses.append(r))
        mod.map_event.subscribe(lambda e: events.append(e))
        mod._test_responses = responses
        mod._test_events = events

        source_dir = tmp_path / "source"
        source_dir.mkdir()
        _write_minimal_pcd(source_dir)
        assert _cmd(mod, {"action": "create", "name": "demo"})["success"] is True
        assert (
            _cmd(
                mod,
                {
                    "action": "import_pcd",
                    "name": "demo",
                    "source_path": str(source_dir / "map.pcd"),
                },
            )["success"]
            is True
        )
        assert _cmd(mod, {"action": "build_octomap", "name": "demo"})["success"] is True
        demo = Path(mod._map_dir) / "demo"

        resp = _cmd(
            mod,
            {
                "action": "edit_voxels",
                "name": "demo",
                "state": "preblocked",
                "x": 1.0,
                "y": 2.0,
                "z": 0.5,
                "radius": 0.3,
            },
        )

        assert resp["success"] is True
        assert resp["edit"]["state"] == "preblocked"
        assert resp["edit"]["edited_voxels"] == 7
        assert resp["transactional_visibility"] == "staged_until_commit"
        assert resp["rolled_back"] is False
        assert (demo / "octomap.ot").read_bytes().endswith(b"\nedited")
        assert not list(demo.glob("octomap.ot.preedit-*"))
        journal_entry = json.loads((demo / "voxel_edits.jsonl").read_text(encoding="utf-8").strip())
        assert journal_entry["state"] == "preblocked"
        metadata = json.loads((demo / "metadata.json").read_text(encoding="utf-8"))
        assert metadata["artifacts"]["octomap"]["manual_voxel_edit"] is True
        assert metadata["artifacts"]["octomap"]["sha256"] == sha256_file(demo / "octomap.ot")
        assert metadata["manual_voxel_edits"]["count"] == 1
        assert events[-1]["event"] == "map.edited"

        queried = _cmd(mod, {"action": "get_voxel_edits", "name": "demo"})
        assert queried["success"] is True
        assert queried["map_id"] == "demo"
        assert queried["edits"][-1]["state"] == "preblocked"

        before = {name: (demo / name).read_bytes() for name in ("octomap.ot", "metadata.json", "voxel_edits.jsonl")}
        monkeypatch.setenv("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER", "1")
        failed = _cmd(
            mod,
            {
                "action": "edit_voxels",
                "name": "demo",
                "state": "free",
                "x": 1.0,
                "y": 2.0,
                "z": 0.5,
                "radius": 0.3,
            },
        )
        monkeypatch.delenv("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER")
        assert failed["success"] is False
        assert failed["reason_code"] == "transaction_commit_failed"
        assert failed["rolled_back"] is True
        assert all((demo / name).read_bytes() == value for name, value in before.items())
        assert not any(path.name.endswith("_transaction") for path in (demo / ".builds").iterdir())

    def test_validate_artifacts_is_owned_by_maps_service(self, map_manager, monkeypatch):
        created = _cmd(map_manager, {"action": "create", "name": "demo"})
        assert created["success"] is True
        calls = []

        def fake_validate(map_id, **kwargs):
            calls.append((map_id, kwargs))
            return {
                "action": "validate_artifacts",
                "success": True,
                "map_id": map_id,
                "gate": {"ok": True, "blockers": [], "artifacts": {}},
            }

        monkeypatch.setattr(
            map_manager.storage.native_service,
            "validate_artifacts",
            fake_validate,
        )

        validated = _cmd(
            map_manager,
            {
                "action": "validate_artifacts",
                "name": "demo",
                "require_octomap": True,
                "require_occupancy": False,
                "expected_frame_id": "map",
            },
        )

        assert validated["success"] is True
        assert validated["map_id"] == "demo"
        assert validated["gate"]["ok"] is True
        assert calls == [
            (
                "demo",
                {
                    "require_octomap": True,
                    "require_occupancy": False,
                    "expected_frame_id": "map",
                },
            )
        ]

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
        created = _cmd(map_manager, {"action": "create", "name": "demo"})
        assert created["success"] is True

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
        assert "build_status" not in by_capability
        assert "build_status" not in by_class
        assert calls == ["occupancy:demo", "octomap:demo"]

    def test_build_status_reports_latest_native_pipeline_state(self, map_manager):
        created = _cmd(map_manager, {"action": "create", "name": "buildable"})
        assert created["success"] is True

        before = _cmd(map_manager, {"action": "get_build_status", "name": "buildable"})
        assert before["success"] is True
        assert before["has_build"] is False

        begin = map_manager.storage.native_service.begin_build("buildable", "OCTOMAP_3D")
        assert begin["success"] is True

        running = _cmd(map_manager, {"action": "build_status", "name": "buildable"})
        assert running["running"] is True
        assert running["status"] == "RUNNING"

        finish = map_manager.storage.native_service.finish_build(
            "buildable",
            begin["build_id"],
            success=False,
            message="converter failed",
        )
        assert finish["success"] is True

        after = _cmd(map_manager, {"action": "get_pipeline_status", "name": "buildable"})
        assert after["running"] is False
        assert after["status"] == "FAILED"
        assert after["message"] == "converter failed"

    def test_build_artifact_builds_native_grid_artifacts(self, map_manager):
        missing = _cmd(
            map_manager,
            {
                "action": "build_artifact",
                "name": "demo",
                "capability": "trajectory_optimization",
            },
        )

        assert missing["success"] is False
        assert missing["reason_code"] == "map_not_found"

        created = _cmd(map_manager, {"action": "create", "name": "demo"})
        assert created["success"] is True
        _write_minimal_pcd(Path(map_manager._map_dir) / "demo")
        points = _cmd(
            map_manager,
            {"action": "get_map_points", "name": "demo", "max_points": 1},
        )
        assert points["success"] is True
        assert points["returned"] == 1
        occupancy = _cmd(map_manager, {"action": "build_occupancy", "name": "demo"})
        assert occupancy["success"] is True

        esdf = _cmd(
            map_manager,
            {
                "action": "build_artifact",
                "name": "demo",
                "capability": "trajectory_optimization",
            },
        )
        assert esdf["success"] is True
        assert esdf["action"] == "build_esdf_artifact"
        assert (Path(map_manager._map_dir) / "demo" / "esdf.npz").is_file()

        traversability = _cmd(
            map_manager,
            {
                "action": "build_artifact",
                "name": "demo",
                "capability": "traversability",
            },
        )
        assert traversability["success"] is True
        assert traversability["action"] == "build_traversability_artifact"
        assert (Path(map_manager._map_dir) / "demo" / "traversability.npz").is_file()

        record = _cmd(map_manager, {"action": "get_record", "name": "demo"})
        assert "trajectory_optimization" in record["record"]["capabilities"]
        assert "traversability" in record["record"]["capabilities"]

    def test_build_semantic_artifact_requires_valid_binary_source(self, map_manager):
        created = _cmd(map_manager, {"action": "create", "name": "semantic_demo"})
        assert created["success"] is True

        missing = _cmd(
            map_manager,
            {
                "action": "build_artifact",
                "name": "semantic_demo",
                "capability": "semantic_query",
            },
        )
        assert missing["success"] is False
        assert missing["reason_code"] == "invalid_semantic_source"

        map_dir = Path(map_manager._map_dir) / "semantic_demo"
        _write_minimal_semantic_map(map_dir / "semantic_map.bin", corrupt=True)
        corrupt = _cmd(
            map_manager,
            {
                "action": "build_artifact",
                "name": "semantic_demo",
                "capability": "semantic_query",
            },
        )
        assert corrupt["success"] is False
        assert corrupt["reason_code"] == "invalid_semantic_source"
        assert "checksum" in corrupt["message"]

        _write_minimal_semantic_map(map_dir / "semantic_map.bin")
        semantic = _cmd(
            map_manager,
            {
                "action": "build_artifact",
                "name": "semantic_demo",
                "capability": "semantic_query",
            },
        )
        assert semantic["success"] is True, semantic
        assert semantic["action"] == "build_semantic_artifact"
        assert semantic["navigation_ready"] is False
        assert semantic["generation"] == 7
        assert semantic["voxel_count"] == 0
        assert semantic["frame_id"] == "map"
        assert semantic["taxonomy"] == "lingtu.semantic"
        assert semantic["taxonomy_version"] == 1

        record = _cmd(map_manager, {"action": "get_record", "name": "semantic_demo"})
        assert "semantic_query" in record["record"]["capabilities"]
        assert record["record"]["health"]["active_allowed"] is False

        active = _cmd(map_manager, {"action": "set_active", "name": "semantic_demo"})
        assert active["success"] is False

    def test_command_router_uses_facade_save_wrapper(self, map_manager, monkeypatch):
        """map_command save remains compatible with facade wrapper overrides."""
        called = {}

        def fake_save(name, slam_profile=None, map_opt=None):
            called["args"] = (name, slam_profile, map_opt)
            return {"action": "save", "success": True, "map_id": name}

        monkeypatch.setattr(map_manager, "_map_save", fake_save)

        resp = _cmd(
            map_manager,
            {"action": "save", "name": "patched", "slam_profile": "localizer"},
        )

        assert resp["success"] is True
        assert called["args"] == ("patched", "localizer", None)

    def test_invalid_control_request(self, map_manager):
        """Untyped control payloads are rejected without mutating state."""
        map_manager._test_responses.clear()
        map_manager._on_command("not a request")  # type: ignore[arg-type]
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
        assert resp["reason_code"] == "invalid_map_name"
        assert "invalid map" in resp["message"]
        assert not outside.exists()

    def test_map_command_rejects_absolute_map_name(self, map_manager, tmp_path):
        absolute_name = str(tmp_path / "outside")
        resp = _cmd(map_manager, {"action": "save", "name": absolute_name})

        assert resp["success"] is False
        assert "invalid map name" in resp["message"]
        assert not Path(absolute_name).exists()

    def test_get_record_rejects_metadata_artifact_path_escape(self, map_manager):
        from maps.artifacts import sha256_file

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
        artifact_uris = {str(Path(artifact["uri"]).resolve()) for artifact in record_resp["record"]["artifacts"]}
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
        assert artifacts.planner_map_path("octoplanner3d").endswith("octomap.ot")

    def test_map_save_uses_navigation_package_transaction(
        self,
        map_manager,
        tmp_path,
    ):
        """MapsModule save must use the native SaveMap job transaction."""
        import numpy as np

        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)

        def reject_legacy_package(*_args, **_kwargs):
            raise AssertionError("Python navigation-package orchestration is forbidden")

        map_manager.pipeline.build_navigation_package = reject_legacy_package
        map_manager._on_map_cloud_frame(
            MapCloudFrame(
                points=np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.2]], dtype=np.float32),
                mode="FULL",
                frame_id="map",
                source="native_slam:test",
            )
        )

        saved = _cmd(
            map_manager,
            {"action": "save", "name": "transaction_map", "slam_profile": "super_lio"},
        )

        assert saved["success"] is True, saved
        assert saved["status"] == "ready"
        assert saved["navigation_ready"] is True
        assert saved["octomap_ok"] is True
        assert saved["version"] == 1
        assert Path(saved["manifest"]).is_file()
        assert saved["transactional_visibility"] == ("immutable_version_then_atomic_pointer")
        assert saved["source_map_transaction"]["success"] is True
        assert not _has_save_source_staging(Path(map_manager._map_dir) / "transaction_map")

    def test_map_save_commits_live_semantic_artifact(self, map_manager, tmp_path):
        import numpy as np

        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)
        map_manager._on_map_cloud_frame(
            MapCloudFrame(
                points=np.array(
                    [[0.0, 0.0, 0.0], [1.0, 0.0, 0.2], [0.0, 1.0, 0.1]],
                    dtype=np.float32,
                ),
                mode="FULL",
                frame_id="map",
                source="native_slam:test",
            )
        )

        def respond(request):
            _write_minimal_semantic_map(Path(request.path))
            map_manager._on_semantic_save_result(
                SemanticSaveResult(
                    request_id=request.request_id,
                    map_id=request.map_id,
                    path=request.path,
                    success=True,
                    generation=4,
                    voxel_count=12,
                    message="validated",
                )
            )

        map_manager.semantic_save_request.subscribe(respond)

        saved = map_manager._map_save("semantic_save", slam_profile="super_lio")

        assert saved["success"] is True
        assert saved["semantic_ok"] is True
        assert saved["semantic"]["generation"] == 4
        assert saved["semantic"]["voxel_count"] == 12
        semantic = next(item for item in saved["record"]["artifacts"] if item["type"] == "SEMANTIC")
        assert ".versions" in semantic["uri"]
        assert Path(semantic["uri"]).is_file()

    def test_map_save_runs_optimizer_then_dynamic_filter_before_planner_artifacts(
        self,
        map_manager,
        tmp_path,
        monkeypatch,
    ):
        """PGO output is cleaned before any navigation artifact is built."""

        class PatchMapSaveAdapter:
            def save_slam_map(self, file_path, **_kwargs):
                pcd = Path(file_path)
                pcd.parent.mkdir(parents=True, exist_ok=True)
                _write_minimal_pcd(pcd.parent)
                (pcd.parent / "poses.txt").write_text(
                    "0 0 0 0 0 0 1\n",
                    encoding="utf-8",
                )
                patches = pcd.parent / "patches"
                patches.mkdir(exist_ok=True)
                (patches / "000001.pcd").write_text(
                    "VERSION .7\nPOINTS 1\nDATA ascii\n0 0 0\n",
                    encoding="utf-8",
                )
                return {"success": True, "source": "fake_patch_save"}

        monkeypatch.setenv("LINGTU_SAVE_DYNAMIC_FILTER", "1")
        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)
        map_manager.pipeline.map_opt.strategy = "pgo"
        map_manager.pipeline.map_opt.command = _fake_map_optimizer(tmp_path)
        map_manager.pipeline.map_prune_command = _fake_dynamic_filter(tmp_path)
        map_manager.runtime_bridge.map_save_adapter = PatchMapSaveAdapter()

        saved = _cmd(
            map_manager,
            {"action": "save", "name": "optimized_map", "slam_profile": "native_dds"},
        )

        assert saved["success"] is True, saved
        assert saved["map_optimization_ok"] is True
        assert saved["map_optimization_performed"] is True
        assert saved["map_optimization"]["performed"] is True
        assert saved["map_optimization"]["strategy"] == "pgo"
        assert saved["map_optimization"]["changed"] is True
        assert saved["map_optimization"]["patch_count"] == 1
        assert saved["source_map_transaction"]["dynamic_filter"]["success"] is True
        assert saved["octomap_ok"] is True
        assert saved["transactional_visibility"] == ("immutable_version_then_atomic_pointer")
        assert saved["source_map_transaction"]["success"] is True
        assert not _has_save_source_staging(Path(map_manager._map_dir) / "optimized_map")
        assert b"POINTS 1\n" in Path(saved["pcd"]).read_bytes()
        assert b"POINTS 1\n" in Path(saved["octomap"]).read_bytes()

    @pytest.mark.parametrize("optimizer_required", (False, True))
    def test_map_save_preserves_upstream_slam_refinement_instead_of_rebuilding_from_poses(
        self,
        map_manager,
        tmp_path,
        optimizer_required,
        monkeypatch,
    ):
        """A downstream odometry-only optimizer must not replace Fast-LIO output."""

        class RefinedPatchMapSaveAdapter:
            def save_slam_map(self, file_path, **_kwargs):
                pcd = Path(file_path)
                pcd.parent.mkdir(parents=True, exist_ok=True)
                _write_minimal_pcd(pcd.parent)
                (pcd.parent / "poses.txt").write_text(
                    "000001.pcd 0 0 0 1 0 0 0\n",
                    encoding="utf-8",
                )
                patches = pcd.parent / "patches"
                patches.mkdir(exist_ok=True)
                (patches / "000001.pcd").write_text(
                    "VERSION .7\nPOINTS 1\nDATA ascii\n9 9 9\n",
                    encoding="utf-8",
                )
                (pcd.parent / "map_optimization.json").write_text(
                    json.dumps(
                        {
                            "schema_version": "lingtu.slam.map_optimization.v1",
                            "status": "optimized_refined_no_loop",
                            "refine_applied": True,
                            "loop_closure_applied": False,
                        }
                    ),
                    encoding="utf-8",
                )
                return {"success": True, "source": "fake_refined_patch_save"}

        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)
        monkeypatch.setenv("LINGTU_SAVE_DYNAMIC_FILTER_REQUIRED", "0")
        map_manager.pipeline.map_opt.strategy = "pgo"
        map_manager.pipeline.map_opt.command = _fake_map_optimizer(tmp_path)
        map_manager.pipeline.map_opt.required = optimizer_required
        map_manager.runtime_bridge.map_save_adapter = RefinedPatchMapSaveAdapter()

        saved = _cmd(
            map_manager,
            {
                "action": "save",
                "name": f"refined_map_{int(optimizer_required)}",
                "slam_profile": "native_dds",
            },
        )

        if optimizer_required:
            assert saved["success"] is False, saved
            assert saved["reason_code"] == "map_optimization_failed"
            return

        assert saved["success"] is True, saved
        assert saved["map_optimization_ok"] is False
        assert saved["map_optimization_performed"] is False
        assert saved["map_optimization"]["status"] == "skipped"
        assert saved["map_optimization"]["performed"] is False
        assert saved["map_optimization"]["reason_code"] == (
            "upstream_slam_optimization_preserved"
        )
        assert saved["map_optimization"]["changed"] is False
        assert b"POINTS 1\n" in Path(saved["pcd"]).read_bytes()

    @pytest.mark.parametrize("dynamic_filter_required", (False, True))
    def test_map_save_skips_pose_based_cleaner_when_upstream_loop_poses_are_not_exported(
        self,
        map_manager,
        tmp_path,
        monkeypatch,
        dynamic_filter_required,
    ):
        class LoopCorrectedMapSaveAdapter:
            def save_slam_map(self, file_path, **_kwargs):
                pcd = Path(file_path)
                pcd.parent.mkdir(parents=True, exist_ok=True)
                _write_minimal_pcd(pcd.parent)
                (pcd.parent / "poses.txt").write_text(
                    "000001.pcd 0 0 0 1 0 0 0\n",
                    encoding="utf-8",
                )
                patches = pcd.parent / "patches"
                patches.mkdir(exist_ok=True)
                (patches / "000001.pcd").write_text(
                    "VERSION .7\nPOINTS 1\nDATA ascii\n0 0 0\n",
                    encoding="utf-8",
                )
                (pcd.parent / "map_optimization.json").write_text(
                    json.dumps(
                        {
                            "schema_version": "lingtu.slam.map_optimization.v1",
                            "status": "optimized_loop_closed",
                            "refine_applied": True,
                            "loop_closure_applied": True,
                        }
                    ),
                    encoding="utf-8",
                )
                return {"success": True, "source": "fake_loop_corrected_save"}

        monkeypatch.setenv("LINGTU_SAVE_DYNAMIC_FILTER", "1")
        monkeypatch.setenv(
            "LINGTU_SAVE_DYNAMIC_FILTER_REQUIRED",
            "1" if dynamic_filter_required else "0",
        )
        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)
        map_manager.pipeline.map_prune_command = (
            sys.executable,
            "-c",
            "raise SystemExit(9)",
        )
        map_manager.runtime_bridge.map_save_adapter = LoopCorrectedMapSaveAdapter()

        saved = _cmd(
            map_manager,
            {
                "action": "save",
                "name": f"loop_corrected_map_{int(dynamic_filter_required)}",
                "slam_profile": "native_dds",
            },
        )

        if dynamic_filter_required:
            assert saved["success"] is False, saved
            assert saved["reason_code"] == "dynamic_filter_failed"
            return

        assert saved["success"] is True, saved
        dynamic_filter = saved["source_map_transaction"]["dynamic_filter"]
        assert dynamic_filter["success"] is False
        assert dynamic_filter["status"] == "skipped"
        assert dynamic_filter["performed"] is False
        assert dynamic_filter["reason_code"] == "upstream_pose_map_consistency_unproven"
        assert dynamic_filter["changed"] is False

    def test_map_save_preserves_source_when_upstream_optimization_report_is_incomplete(
        self,
        map_manager,
        tmp_path,
        monkeypatch,
    ):
        class IncompleteReportMapSaveAdapter:
            def save_slam_map(self, file_path, **_kwargs):
                pcd = Path(file_path)
                pcd.parent.mkdir(parents=True, exist_ok=True)
                _write_minimal_pcd(pcd.parent)
                (pcd.parent / "poses.txt").write_text(
                    "000001.pcd 0 0 0 1 0 0 0\n",
                    encoding="utf-8",
                )
                patches = pcd.parent / "patches"
                patches.mkdir(exist_ok=True)
                (patches / "000001.pcd").write_text(
                    "VERSION .7\nPOINTS 1\nDATA ascii\n9 9 9\n",
                    encoding="utf-8",
                )
                (pcd.parent / "map_optimization.json").write_text(
                    json.dumps(
                        {
                            "schema_version": "lingtu.slam.map_optimization.v1",
                            "refine_applied": True,
                        }
                    ),
                    encoding="utf-8",
                )
                return {"success": True, "source": "incomplete_report_save"}

        monkeypatch.setenv("LINGTU_SAVE_DYNAMIC_FILTER", "1")
        monkeypatch.setenv("LINGTU_SAVE_DYNAMIC_FILTER_REQUIRED", "0")
        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)
        map_manager.pipeline.map_opt.strategy = "pgo"
        map_manager.pipeline.map_opt.command = _fake_map_optimizer(tmp_path)
        map_manager.pipeline.map_prune_command = _fake_dynamic_filter(tmp_path)
        map_manager.runtime_bridge.map_save_adapter = IncompleteReportMapSaveAdapter()

        saved = _cmd(
            map_manager,
            {
                "action": "save",
                "name": "bad_opt_report",
                "slam_profile": "native_dds",
            },
        )

        assert saved["success"] is True, saved
        assert saved["map_optimization_ok"] is False
        assert saved["map_optimization_performed"] is False
        assert saved["map_optimization"]["success"] is False
        assert saved["map_optimization"]["reason_code"] == (
            "upstream_optimization_report_invalid"
        )
        dynamic_filter = saved["source_map_transaction"]["dynamic_filter"]
        assert dynamic_filter["success"] is False
        assert dynamic_filter["reason_code"] == "upstream_optimization_report_invalid"
        assert b"POINTS 1\n" in Path(saved["pcd"]).read_bytes()

    def test_map_save_fails_closed_when_required_dynamic_filter_fails(
        self,
        map_manager,
        tmp_path,
        monkeypatch,
    ):
        """A failed cleaner must not publish artifacts built from dirty PGO output."""

        class SnapshotAdapter:
            def save_slam_map(self, file_path, **_kwargs):
                _write_minimal_pcd(Path(file_path).parent)
                return {"success": True, "source": "fake_snapshot"}

        monkeypatch.setenv("LINGTU_SAVE_DYNAMIC_FILTER", "1")
        monkeypatch.delenv("LINGTU_SAVE_DYNAMIC_FILTER_REQUIRED", raising=False)
        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)
        map_manager.pipeline.map_prune_command = (
            sys.executable,
            "-c",
            "raise SystemExit(7)",
        )
        map_manager.runtime_bridge.map_save_adapter = SnapshotAdapter()

        saved = _cmd(
            map_manager,
            {"action": "save", "name": "filter_failed", "slam_profile": "native_dds"},
        )

        assert saved["success"] is False
        assert saved["reason_code"] == "dynamic_filter_failed", saved
        assert not _has_save_source_staging(Path(map_manager._map_dir) / "filter_failed")
        assert not (Path(map_manager._map_dir) / "filter_failed" / "current").exists()

    def test_map_save_without_octomap_fails_without_publishing(
        self,
        map_manager,
    ):
        """A required OctoMap failure cannot publish a partial current version."""
        import numpy as np

        map_manager._on_map_cloud_frame(
            MapCloudFrame(
                points=np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.2]], dtype=np.float32),
                mode="FULL",
                frame_id="map",
                source="native_slam:test",
            )
        )

        saved = _cmd(
            map_manager,
            {"action": "save", "name": "pcd_only", "slam_profile": "super_lio"},
        )

        assert saved["success"] is False, saved
        assert saved["reason_code"] == "octomap_build_failed"
        map_dir = Path(map_manager._map_dir) / "pcd_only"
        assert not (map_dir / "current_version.txt").exists()
        assert not (map_dir / "map.pcd").exists()

    def test_map_save_rejects_explicit_unhealthy_slam(
        self,
        map_manager,
        tmp_path,
    ):
        import numpy as np

        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)
        map_manager.runtime_bridge.on_localization_status({"state": "LOST", "message": "scan matching diverged"})
        map_manager._on_map_cloud_frame(
            MapCloudFrame(
                points=np.array(
                    [[0.0, 0.0, 0.0], [1.0, 0.0, 0.2], [0.0, 1.0, 0.1]],
                    dtype=np.float32,
                ),
                mode="FULL",
                frame_id="map",
                source="native_slam:test",
            )
        )

        saved = _cmd(
            map_manager,
            {"action": "save", "name": "unhealthy", "slam_profile": "super_lio"},
        )

        assert saved["success"] is False
        assert saved["reason_code"] == "slam_unhealthy"
        assert not (Path(map_manager._map_dir) / "unhealthy" / "current_version.txt").exists()

    def test_map_save_request_id_replays_without_recapturing(
        self,
        map_manager,
        tmp_path,
    ):
        import numpy as np

        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)
        map_manager._on_map_cloud_frame(
            MapCloudFrame(
                points=np.array(
                    [[0.0, 0.0, 0.0], [1.0, 0.0, 0.2], [0.0, 1.0, 0.1]],
                    dtype=np.float32,
                ),
                mode="FULL",
                frame_id="map",
                source="native_slam:test",
            )
        )
        command = {
            "action": "save",
            "name": "idempotent",
            "slam_profile": "super_lio",
            "request_id": "stable_request",
        }

        first = _cmd(map_manager, command)
        second = _cmd(map_manager, command)

        assert first["success"] is True, first
        assert second["success"] is True, second
        assert second["replayed"] is True
        assert first["version"] == second["version"] == 1
        jobs = _cmd(map_manager, {"action": "list_save_jobs", "limit": 10})
        assert jobs["count"] == 1

    def test_map_save_versions_are_listed_and_rolled_back(
        self,
        map_manager,
        tmp_path,
    ):
        import numpy as np

        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)
        map_manager._on_map_cloud_frame(
            MapCloudFrame(
                points=np.array(
                    [[0.0, 0.0, 0.0], [1.0, 0.0, 0.2], [0.0, 1.0, 0.1]],
                    dtype=np.float32,
                ),
                mode="FULL",
                frame_id="map",
                source="native_slam:test",
            )
        )
        first = _cmd(
            map_manager,
            {
                "action": "save",
                "name": "versioned",
                "slam_profile": "super_lio",
                "request_id": "versioned_v1",
            },
        )
        second = _cmd(
            map_manager,
            {
                "action": "save",
                "name": "versioned",
                "slam_profile": "super_lio",
                "request_id": "versioned_v2",
            },
        )
        assert first["success"] is True, first
        assert second["success"] is True, second
        assert (first["version"], second["version"]) == (1, 2)

        versions = _cmd(
            map_manager,
            {"action": "list_map_versions", "name": "versioned"},
        )
        assert versions["success"] is True
        assert [item["version"] for item in versions["versions"]] == [2, 1]
        assert versions["versions"][0]["current"] is True

        rolled = _cmd(
            map_manager,
            {
                "action": "rollback_map_version",
                "name": "versioned",
                "version": 1,
            },
        )
        assert rolled["success"] is True, rolled
        assert rolled["version"] == 1
        record = _cmd(map_manager, {"action": "get_record", "name": "versioned"})
        assert record["record"]["version"] == 1

        invalid = _cmd(
            map_manager,
            {
                "action": "rollback_map_version",
                "name": "versioned",
                "version": 0,
            },
        )
        assert invalid["success"] is False
        assert invalid["reason_code"] == "invalid_version"

    def test_map_save_adapter_success_without_pcd_fails_and_cleans_new_dir(
        self,
        map_manager,
    ):
        class SuccessWithoutFileAdapter:
            def save_slam_map(self, file_path, **_kwargs):
                return {
                    "success": True,
                    "source": "fake_success_without_file",
                    "pcd": str(file_path),
                }

        map_manager.runtime_bridge.map_save_adapter = SuccessWithoutFileAdapter()

        saved = _cmd(
            map_manager,
            {"action": "save", "name": "phantom_map", "slam_profile": "native_dds"},
        )

        assert saved["success"] is False, saved
        assert saved["diagnostic_code"] == "map_pcd_missing_after_save"
        assert "did not write map.pcd" in saved["message"]
        assert not (Path(map_manager._map_dir) / "phantom_map").exists()

    def test_import_pcd_invalidates_old_derived_artifacts(self, map_manager, tmp_path):
        src = tmp_path / "source.pcd"
        src.write_text(
            "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
            "COUNT 1 1 1\nWIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA ascii\n"
            "0.0 0.0 0.0\n0.1 0.0 0.0\n2.0 0.0 0.0\n",
            encoding="utf-8",
        )
        d = Path(map_manager._map_dir) / "imported"
        d.mkdir()
        (d / "octomap.ot").write_bytes(b"stale")
        (d / "occupancy.npz").write_bytes(b"stale")
        (d / "tomogram.pickle").write_bytes(b"stale")
        (d / "poses.txt").write_text("stale", encoding="utf-8")
        (d / "patches").mkdir()

        resp = _cmd(
            map_manager,
            {
                "action": "import_pcd",
                "name": "imported",
                "source_path": str(src),
                "voxel_size": 0.5,
            },
        )

        assert resp["success"] is True, resp
        assert resp["status"] == "partial"
        assert resp["transactional_visibility"] == "staged_until_commit"
        assert resp["point_count"] == 2
        assert Path(resp["pcd"]).is_file()
        assert not (d / "octomap.ot").exists()
        assert not (d / "occupancy.npz").exists()
        assert not (d / "tomogram.pickle").exists()
        assert not (d / "poses.txt").exists()
        assert any(path.name.startswith("patches.stale-") for path in d.iterdir())
        metadata = json.loads((d / "metadata.json").read_text(encoding="utf-8"))
        assert set(metadata["artifacts"]) == {"map_pcd"}
        assert resp["navigation_ready"] is False

    def test_crop_map_invalidates_octomap_until_rebuilt(self, map_manager):
        d = Path(map_manager._map_dir) / "cropped"
        d.mkdir()
        (d / "map.pcd").write_text(
            "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
            "COUNT 1 1 1\nWIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA ascii\n"
            "0.0 0.0 0.0\n1.0 0.0 0.0\n5.0 0.0 0.0\n",
            encoding="utf-8",
        )
        (d / "octomap.ot").write_bytes(b"stale")

        resp = _cmd(
            map_manager,
            {
                "action": "crop",
                "name": "cropped",
                "bounds": {"min": [-0.5, -0.5, -0.5], "max": [1.5, 0.5, 0.5]},
            },
        )

        assert resp["success"] is True, resp
        assert resp["transactional_visibility"] == "staged_until_commit"
        assert resp["point_count"] == 2
        assert resp["removed_points"] == 1
        assert not (d / "octomap.ot").exists()
        metadata = json.loads((d / "metadata.json").read_text(encoding="utf-8"))
        assert metadata["metadata_state"] == "invalidated"
        assert metadata["navigation_ready"] is False
        metadata = json.loads((d / "metadata.json").read_text(encoding="utf-8"))
        assert set(metadata["artifacts"]) == {"map_pcd"}

    def test_get_active_octomap_none(self, map_manager):
        """get_active_octomap returns None when no active map."""
        result = map_manager.get_active_octomap()
        assert result is None

    def test_map_list_reports_has_occupancy(self, map_manager):
        """_map_list reports has_occupancy when occupancy.npz exists."""
        d = Path(map_manager._map_dir) / "mapwithall"
        d.mkdir()
        (d / "map.pcd").touch()
        (d / "tomogram.pickle").touch()
        (d / "occupancy.npz").touch()
        (d / "octomap.ot").touch()

        resp = _cmd(map_manager, {"action": "list"})
        entry = next(m for m in resp["maps"] if m["name"] == "mapwithall")
        assert entry["has_occupancy"] is True
        assert entry["has_octomap"] is True
        artifacts = entry["record"]["artifacts"]
        assert any(artifact["map_class"] == "global_3d_occupancy" for artifact in artifacts)
        assert any(
            artifact["name"] == "occupancy_grid" and artifact["map_class"] == "static_2d_occupancy"
            for artifact in artifacts
        )

    def test_build_octomap_command_writes_artifact_and_metadata(
        self,
        map_manager,
        tmp_path,
    ):
        """build_octomap creates octomap.ot and metadata without planning-time conversion."""
        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)
        d = Path(map_manager._map_dir) / "octomap_source"
        d.mkdir()
        _write_minimal_pcd(d)

        resp = _cmd(map_manager, {"action": "build_octomap", "name": "octomap_source"})

        assert resp["success"] is True, resp
        assert resp["status"] == "built"
        assert Path(resp["octomap"]).is_file()
        metadata_path = Path(resp["metadata"])
        assert metadata_path.is_file()
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        assert metadata["build_mode"] == "external_pcl_converter"
        assert metadata["artifacts"]["octomap"]["path"] == "octomap.ot"
        assert metadata["artifacts"]["octomap"]["source_map_sha256"] == metadata["artifacts"]["map_pcd"]["sha256"]
        record_response = map_manager.storage.native_service.get_record("octomap_source")
        assert record_response["success"] is True
        record = record_response["record"]
        assert record["schema_version"] == "map.record"
        assert record["map_id"] == "octomap_source"
        assert record["version_id"] == "octomap_source:v1"
        assert "navigation_safety_3d" in record["capabilities"]
        octomap = next(item for item in record["artifacts"] if item["type"] == "OCTOMAP_3D")
        assert octomap["name"] == "octomap"
        assert octomap["map_class"] == "global_3d_occupancy"
        assert octomap["capability"] == "navigation_safety_3d"
        assert record["health"]["active_allowed"] is True

    def test_build_octomap_rolls_back_when_native_publish_fails(
        self,
        map_manager,
        tmp_path,
        monkeypatch,
    ):
        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)
        d = Path(map_manager._map_dir) / "bad_gate"
        d.mkdir()
        _write_minimal_pcd(d)

        monkeypatch.setenv("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER", "1")

        resp = _cmd(map_manager, {"action": "build_octomap", "name": "bad_gate"})

        assert resp["success"] is False
        assert resp["reason_code"] == "transaction_commit_failed"
        assert resp["rolled_back"] is True
        assert not (d / "octomap.ot").exists()
        assert not (d / "metadata.json").exists()

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
        assert resp["octomap"].endswith("octomap.ot")
        assert map_manager.get_active_octomap().endswith("octomap.ot")
        artifacts = map_manager.get_active_artifacts()
        assert artifacts["octomap"].endswith("octomap.ot")
        assert "tomogram" not in artifacts
        assert artifacts["artifacts"]["octomap"]["exists"] is True
        assert "global_3d_occupancy" in artifacts["map_classes"]
        assert artifacts["record"]["state"] == "ACTIVE"

    def test_active_artifacts_resolve_from_native_active_state_without_symlink(
        self,
        map_manager,
        tmp_path,
    ):
        """Active map recovery should not require the legacy active symlink."""
        map_manager._map_artifact_converter_command = _fake_octomap_converter(tmp_path)
        d = Path(map_manager._map_dir) / "native_active"
        d.mkdir()
        _write_minimal_pcd(d)
        build = _cmd(map_manager, {"action": "build_octomap", "name": "native_active"})
        assert build["success"] is True, build
        active = _cmd(map_manager, {"action": "set_active", "name": "native_active"})
        assert active["success"] is True, active

        active_link = Path(map_manager._map_dir) / "active"
        if active_link.exists() or active_link.is_symlink():
            active_link.unlink()
        map_manager.stop()
        restarted = MapsModule(
            map_dir=str(map_manager._map_dir),
            data_dir=str(tmp_path / "fresh_data_dir"),
        )
        assert restarted._active_map == "native_active"
        assert restarted.get_active_octomap().endswith("octomap.ot")
        artifacts = restarted.get_active_artifacts()
        assert artifacts["record"]["state"] == "ACTIVE"
        assert artifacts["octomap"].endswith("octomap.ot")
        restarted.stop()

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
        assert resp["artifact"]["uri"].endswith("octomap.ot")

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

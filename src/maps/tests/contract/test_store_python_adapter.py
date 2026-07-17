"""Contract tests for the Python-owned native MapStore handle."""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys

import pytest


def _require_native_maps_lib() -> None:
    if not os.environ.get("LINGTU_MAPS_LIB"):
        pytest.skip("LINGTU_MAPS_LIB is required for native MapStore adapter tests")


def _write_valid_occupancy_metadata(map_dir, *, frame_id: str = "map") -> None:
    map_sha = hashlib.sha256((map_dir / "map.pcd").read_bytes()).hexdigest()
    occupancy_sha = hashlib.sha256((map_dir / "occupancy.npz").read_bytes()).hexdigest()
    metadata = {
        "frame_id": frame_id,
        "artifacts": {
            "map_pcd": {
                "path": "map.pcd",
                "sha256": map_sha,
                "frame_id": frame_id,
            },
            "occupancy_grid": {
                "path": "occupancy.npz",
                "sha256": occupancy_sha,
                "source_map_sha256": map_sha,
                "frame_id": frame_id,
            },
        },
    }
    (map_dir / "metadata.json").write_text(
        json.dumps(metadata, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def test_native_map_store_lifecycle(tmp_path) -> None:
    _require_native_maps_lib()

    from maps.adapters.python.store import NativeMapStore

    store = NativeMapStore(tmp_path)
    assert NativeMapStore.validate_map_id("floor_1")
    assert not NativeMapStore.validate_map_id("../bad")

    store.create_map("floor_1")
    assert store.list_map_ids() == ["floor_1"]

    with pytest.raises(RuntimeError):
        store.set_active_map("floor_1", strict=True)

    map_dir = tmp_path / "floor_1"
    (map_dir / "map.pcd").write_text("VERSION .7\n", encoding="utf-8")
    (map_dir / "occupancy.npz").write_text("grid", encoding="utf-8")
    _write_valid_occupancy_metadata(map_dir)
    assert store.artifact_count("floor_1") == 2
    validation = store.validate_artifacts(
        "floor_1",
        require_occupancy=True,
        expected_frame_id="map",
    )
    assert validation["success"] is True
    assert validation["map_id"] == "floor_1"
    assert validation["gate"]["ok"] is True
    record = store.record("floor_1")
    assert record is not None
    assert record["map_id"] == "floor_1"
    assert record["health"]["active_allowed"] is True
    assert record["health"]["localization_stability"] is None
    assert record["health"]["planning_success_rate"] is None
    assert record["health"]["collision_rate"] is None
    assert record["health"]["overall_score"] is None
    assert record["health"]["status"] == "unknown"
    assert record["health"]["reason_code"] == "not_enough_data"
    assert "path_planning_2d" in record["capabilities"]
    assert store.list_records()[0]["map_id"] == "floor_1"
    bundle = store.bundle("floor_1", "path_planning_2d")
    assert bundle["success"] is True
    assert bundle["schema_version"] == "map.bundle"
    assert bundle["artifact"]["type"] == "OCCUPANCY_2D"
    assert store.bundle("floor_1", "trajectory_optimization")["success"] is False

    store.set_active_map("floor_1", strict=True)
    assert store.active_map_id() == "floor_1"
    active_record = store.active_record()
    assert active_record is not None
    assert active_record["state"] == "ACTIVE"
    assert not (tmp_path / "active").exists()
    assert (tmp_path / "active_map.txt").read_text(encoding="utf-8") == "floor_1\n"

    store.rename_map("floor_1", "floor_2")
    assert store.list_map_ids() == ["floor_2"]
    assert store.active_map_id() == "floor_2"

    store.clear_active_map()
    assert store.active_map_id() == ""
    assert store.active_record() is None
    store.set_active_map("floor_2", strict=True)

    store.delete_map("floor_2")
    assert store.list_map_ids() == []
    assert store.active_map_id() == ""
    store.close()


def test_native_map_store_validation_coexists_with_full_service(tmp_path) -> None:
    _require_native_maps_lib()

    from maps.adapters.python.service import NativeMapsService
    from maps.adapters.python.store import NativeMapStore

    service = NativeMapsService(tmp_path)
    store = None
    try:
        assert service.create_map("shared_map")["success"] is True
        map_dir = tmp_path / "shared_map"
        (map_dir / "map.pcd").write_text("VERSION .7\n", encoding="utf-8")
        (map_dir / "occupancy.npz").write_text("grid", encoding="utf-8")
        _write_valid_occupancy_metadata(map_dir)

        store = NativeMapStore(tmp_path)
        validation = store.validate_artifacts(
            "shared_map",
            require_occupancy=True,
            expected_frame_id="map",
        )
        assert validation["success"] is True
        assert validation["map_id"] == "shared_map"
        assert validation["gate"]["ok"] is True
        store.set_active_map("shared_map", strict=True)

        from maps.services.active_assets import ActiveMapAssets

        gate = ActiveMapAssets(map_roots=(tmp_path,)).validate_artifact_path(
            str(map_dir / "occupancy.npz"),
            require_occupancy=True,
            expected_frame_id="map",
        )
        assert gate["ok"] is True, gate
    finally:
        if store is not None:
            store.close()
        service.close()


def test_native_maps_service_lifecycle(tmp_path) -> None:
    _require_native_maps_lib()

    from maps.adapters.python.service import NativeMapsService

    service = NativeMapsService(tmp_path)
    map_types = service.get_map_types()
    assert map_types["schema_version"] == "map.types"
    assert map_types["capabilities"]["path_planning_2d"] == "OCCUPANCY_2D"
    assert map_types["capabilities"]["traversability"] == "TRAVERSABILITY"

    created = service.create_map("floor_1")
    assert created["success"] is True
    assert created["record"]["map_id"] == "floor_1"

    begin = service.begin_build("floor_1", "OCCUPANCY_2D")
    assert begin["success"] is True
    assert begin["status"] == "RUNNING"
    assert begin["artifact_type"] == "OCCUPANCY_2D"
    blocked = service.begin_build("floor_1", "OCTOMAP_3D")
    assert blocked["success"] is False
    assert blocked["reason_code"] == "build_in_progress"
    running = service.get_build_status("floor_1")
    assert running["success"] is True
    assert running["running"] is True
    finished = service.finish_build(
        "floor_1",
        begin["build_id"],
        success=True,
        message="ok",
    )
    assert finished["success"] is True
    assert finished["status"] == "SUCCEEDED"
    status = service.get_build_status("floor_1")
    assert status["running"] is False
    assert status["status"] == "SUCCEEDED"

    missing_active = service.get_active_map()
    assert missing_active["success"] is False

    failed_active = service.set_active_map("floor_1", strict=True)
    assert failed_active["success"] is False
    assert failed_active["reason_code"] == "artifact_gate_failed"

    source_pcd = tmp_path / "source.pcd"
    source_pcd.write_text(
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        "COUNT 1 1 1\nWIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA ascii\n"
        "0.0 0.0 0.0\n0.1 0.0 0.0\n2.0 0.0 0.0\n",
        encoding="utf-8",
    )
    imported = service.import_pcd("floor_1", source_pcd, voxel_size=0.5)
    assert imported["success"] is True
    assert imported["transactional_visibility"] == "staged_until_commit"
    assert imported["point_count"] == 2
    assert (tmp_path / "floor_1" / "map.pcd").is_file()
    points = service.get_map_points("floor_1", max_points=1)
    assert points["success"] is True
    assert points["returned"] == 1
    assert len(points["points"]) == 1

    saved_source = service.create_map("saved_source")
    assert saved_source["success"] is True
    source_stage = tmp_path.parent / f"{tmp_path.name}_source_stage"
    source_stage.mkdir()
    (source_stage / "map.pcd").write_text(source_pcd.read_text(encoding="utf-8"), encoding="utf-8")
    (source_stage / "poses.txt").write_text("000001.pcd 0 0 0 1 0 0 0\n", encoding="utf-8")
    patches = source_stage / "patches"
    patches.mkdir()
    (patches / "000001.pcd").write_text(source_pcd.read_text(encoding="utf-8"), encoding="utf-8")
    optimizer_script = tmp_path / "fake_source_optimizer.py"
    optimizer_script.write_text(
        "from __future__ import annotations\n"
        "import argparse\n"
        "from pathlib import Path\n"
        "parser = argparse.ArgumentParser()\n"
        "parser.add_argument('--map', required=True)\n"
        "parser.add_argument('--out', required=True)\n"
        "args = parser.parse_args()\n"
        "pcd = Path(args.map) / 'map.pcd'\n"
        "pcd.write_text(\n"
        "    'VERSION 0.7\\nFIELDS x y z\\nSIZE 4 4 4\\nTYPE F F F\\n'\n"
        "    'COUNT 1 1 1\\nWIDTH 2\\nHEIGHT 1\\nPOINTS 2\\nDATA ascii\\n'\n"
        "    '0 0 0\\n1 0 0\\n',\n"
        "    encoding='utf-8',\n"
        ")\n",
        encoding="utf-8",
    )
    committed = service.commit_saved_source(
        "saved_source",
        source_stage,
        dynamic_filter_enabled=False,
        optimizer_command=subprocess.list2cmdline(
            [sys.executable, str(optimizer_script), "--map", "{map}", "--out", "{out}"]
        ),
    )
    assert committed["success"] is True
    assert committed["mode"] == "native_saved_source_transaction"
    assert committed["map_optimization_ok"] is True
    assert committed["map_optimization_performed"] is True
    assert committed["map_optimization"]["performed"] is True
    assert committed["map_optimization"]["patch_count"] == 1
    assert (tmp_path / "saved_source" / "poses.txt").is_file()
    assert (tmp_path / "saved_source" / "patches" / "000001.pcd").is_file()

    cropped = service.crop_pcd(
        "floor_1",
        {"min": [-0.5, -0.5, -0.5], "max": [0.5, 0.5, 0.5]},
    )
    assert cropped["success"] is True
    assert cropped["transactional_visibility"] == "staged_until_commit"
    assert cropped["point_count"] == 1
    assert cropped["removed_points"] == 1

    occupancy = service.build_occupancy_snapshot("floor_1")
    assert occupancy["success"] is True
    assert occupancy["mode"] == "projection_native"
    assert (tmp_path / "floor_1" / "occupancy.npz").is_file()
    np = pytest.importorskip("numpy")
    with np.load(tmp_path / "floor_1" / "occupancy.npz") as data:
        assert data["grid"].ndim == 2
        assert data["resolution"].shape == ()
        assert data["origin"].shape == (2,)

    esdf = service.build_esdf_artifact("floor_1")
    assert esdf["success"] is True
    assert esdf["mode"] == "native_grid_artifact"
    with np.load(tmp_path / "floor_1" / "esdf.npz") as data:
        assert data["distance"].ndim == 2
        assert data["grad_x"].shape == data["distance"].shape
        assert data["grad_y"].shape == data["distance"].shape

    traversability = service.build_traversability_artifact("floor_1")
    assert traversability["success"] is True
    assert traversability["mode"] == "native_grid_artifact"
    with np.load(tmp_path / "floor_1" / "traversability.npz") as data:
        assert data["cost"].ndim == 2

    _write_valid_occupancy_metadata(tmp_path / "floor_1")

    active = service.set_active_map("floor_1", strict=True)
    assert active["success"] is True
    assert active["active"] == "floor_1"
    validation = service.validate_artifacts(
        "floor_1",
        require_octomap=False,
        require_occupancy=True,
        expected_frame_id="map",
    )
    assert validation["success"] is True
    assert validation["gate"]["ok"] is True

    poi = service.set_poi(
        "floor_1",
        "dock",
        x=1.0,
        y=2.0,
        z=0.3,
        tags={"kind": "charger"},
    )
    assert poi["success"] is True
    pois = service.list_poi("floor_1")
    assert pois["pois"]["dock"]["tags"]["kind"] == "charger"
    assert service.delete_poi("floor_1", "dock")["success"] is True

    floor_b = service.create_map("floor_b")
    assert floor_b["success"] is True
    edge = service.set_map_edge("floor_1", "floor_b", edge_type="door")
    assert edge["success"] is True
    graph = service.list_map_graph()
    assert graph["edges"][0]["from"] == "floor_1"
    assert graph["edges"][0]["to"] == "floor_b"

    queued = service.enqueue_build("floor_1", "OCTOMAP_3D")
    assert queued["success"] is True
    queue = service.get_build_queue()
    assert queue["items"][0]["artifact_type"] == "OCTOMAP_3D"
    popped = service.pop_build_queue()
    assert popped["success"] is False
    assert popped["reason_code"] == "worker_owned_queue"
    job = service.get_artifact_job(queued["job"]["request_id"])
    assert job["success"] is True
    assert job["job"]["map_id"] == "floor_1"

    assert service.set_active_map("floor_b", strict=False)["success"] is True
    rolled_back = service.rollback_active_map()
    assert rolled_back["success"] is True
    assert rolled_back["active"] == "floor_1"

    listed = service.list_maps()
    assert listed["maps"][0]["name"] == "floor_1"
    assert listed["maps"][0]["has_occupancy"] is True
    assert listed["maps"][0]["has_esdf"] is True
    assert listed["maps"][0]["has_traversability"] is True

    record = service.get_record("floor_1")
    assert record["success"] is True
    assert "path_planning_2d" in record["record"]["capabilities"]
    assert "trajectory_optimization" in record["record"]["capabilities"]
    assert "traversability" in record["record"]["capabilities"]
    bundle = service.get_bundle("floor_1", "path_planning_2d")
    assert bundle["success"] is True
    assert bundle["schema_version"] == "map.bundle"
    assert bundle["artifact"]["type"] == "OCCUPANCY_2D"
    esdf_bundle = service.get_bundle("floor_1", "trajectory_optimization")
    assert esdf_bundle["success"] is True
    assert esdf_bundle["artifact"]["type"] == "ESDF"
    trav_bundle = service.get_bundle("floor_1", "traversability")
    assert trav_bundle["success"] is True
    assert trav_bundle["artifact"]["type"] == "TRAVERSABILITY"
    health = service.get_health("floor_1")
    assert health["health"]["active_allowed"] is True

    renamed = service.rename_map("floor_1", "floor_2")
    assert renamed["success"] is True
    assert renamed["map_id"] == "floor_2"
    assert service.clear_active_map()["active"] == ""

    deleted = service.delete_map("floor_2")
    assert deleted["success"] is True
    assert service.delete_map("floor_b")["success"] is True
    assert service.delete_map("saved_source")["success"] is True
    assert service.list_maps()["maps"] == []
    service.close()

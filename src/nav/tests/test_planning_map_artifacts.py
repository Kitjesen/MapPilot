from __future__ import annotations

import json
from pathlib import Path

from nav.services.map.records import build_map_record, write_map_record
from nav.services.plan.global_planner.artifacts import SavedMapArtifacts, default_map_dirs


def test_saved_map_artifacts_resolve_planner_specific_active_files(
    tmp_path: Path,
    monkeypatch,
) -> None:
    maps_dir = tmp_path / "maps"
    active = maps_dir / "active"
    active.mkdir(parents=True)
    octomap = active / "octomap.ot"
    tomogram = active / "tomogram.pickle"
    octomap.write_bytes(b"octomap")
    tomogram.write_bytes(b"tomogram")
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    artifacts = SavedMapArtifacts.from_runtime()

    assert artifacts.planner_map_path("octoplanner3d") == str(octomap)
    assert artifacts.planner_map_path("pct") == str(tomogram)
    assert artifacts.planner_map_path("astar") == str(tomogram)


def test_saved_map_artifacts_explicit_existing_path_wins(
    tmp_path: Path,
    monkeypatch,
) -> None:
    maps_dir = tmp_path / "maps"
    active = maps_dir / "active"
    active.mkdir(parents=True)
    (active / "octomap.ot").write_bytes(b"active")
    explicit = tmp_path / "explicit.bt"
    explicit.write_bytes(b"explicit")
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    artifacts = SavedMapArtifacts.from_runtime(str(explicit))

    assert artifacts.planner_map_path("octoplanner3d") == str(explicit)


def test_octoplanner3d_active_octomap_wins_over_explicit_legacy_pcd(
    tmp_path: Path,
    monkeypatch,
) -> None:
    maps_dir = tmp_path / "maps"
    active = maps_dir / "active"
    active.mkdir(parents=True)
    explicit_pcd = active / "map.pcd"
    octomap = active / "octomap.bt"
    explicit_pcd.write_bytes(b"pcd")
    octomap.write_bytes(b"octomap")
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    artifacts = SavedMapArtifacts.from_runtime(str(explicit_pcd))

    assert artifacts.planner_map_path("octoplanner3d") == str(octomap)


def test_saved_map_artifacts_resolve_active_map_record_bundle(
    tmp_path: Path,
    monkeypatch,
) -> None:
    maps_dir = tmp_path / "maps"
    active = maps_dir / "active"
    active.mkdir(parents=True)
    octomap = active / "octomap.ot"
    octomap.write_bytes(b"octomap")
    write_map_record(active, build_map_record(active, map_id="active_map"))
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    artifacts = SavedMapArtifacts.from_runtime()
    bundle = artifacts.planner_map_bundle("octoplanner3d")

    assert bundle["schema_version"] == "map.bundle"
    assert bundle["capability"] == "navigation_safety_3d"
    assert bundle["artifact"]["uri"] == str(octomap)
    assert artifacts.planner_map_path("octoplanner3d") == str(octomap)


def test_octoplanner3d_prefers_active_octomap_over_stale_record_uri(
    tmp_path: Path,
    monkeypatch,
) -> None:
    maps_dir = tmp_path / "maps"
    active = maps_dir / "active"
    active.mkdir(parents=True)
    pcd = active / "map.pcd"
    octomap = active / "octomap.bt"
    pcd.write_bytes(b"pcd")
    octomap.write_bytes(b"octomap")
    (active / "map_record.json").write_text(
        json.dumps(
            {
                "schema_version": "map.record",
                "map_id": "active_map",
                "state": "ACTIVE",
                "scope": {"frame_id": "map"},
                "artifacts": [
                    {
                        "type": "OCTOMAP_3D",
                        "uri": "map.pcd",
                        "hash": "",
                        "source_map_id": "active_map",
                        "generator": "legacy_record",
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    artifacts = SavedMapArtifacts.from_runtime()

    assert artifacts.planner_map_path("octoplanner3d") == str(octomap)


def test_saved_map_artifacts_bundle_ignores_missing_artifact_uri(
    tmp_path: Path,
    monkeypatch,
) -> None:
    maps_dir = tmp_path / "maps"
    active = maps_dir / "active"
    active.mkdir(parents=True)
    octomap = active / "octomap.ot"
    octomap.write_bytes(b"octomap")
    record = build_map_record(active, map_id="active_map").to_dict()
    octomap.unlink()
    (active / "map_record.json").write_text(
        json.dumps(record),
        encoding="utf-8",
    )
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    artifacts = SavedMapArtifacts.from_runtime()

    assert artifacts.planner_map_bundle("octoplanner3d") == {}
    assert artifacts.planner_map_path("octoplanner3d") == ""


def test_saved_map_artifacts_resolve_static_occupancy_next_to_map_path(
    tmp_path: Path,
    monkeypatch,
) -> None:
    maps_dir = tmp_path / "maps"
    active = maps_dir / "active"
    active.mkdir(parents=True)
    octomap = active / "octomap.ot"
    occupancy = active / "occupancy.npz"
    octomap.write_bytes(b"octomap")
    occupancy.write_bytes(b"occupancy")
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    artifacts = SavedMapArtifacts.from_runtime()

    assert artifacts.static_occupancy_path(str(octomap)) == str(occupancy)


def test_astar_prefers_static_occupancy_over_tomogram(
    tmp_path: Path,
    monkeypatch,
) -> None:
    maps_dir = tmp_path / "maps"
    active = maps_dir / "active"
    active.mkdir(parents=True)
    occupancy = active / "occupancy.npz"
    tomogram = active / "tomogram.pickle"
    occupancy.write_bytes(b"occupancy")
    tomogram.write_bytes(b"tomogram")
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    artifacts = SavedMapArtifacts.from_runtime()

    assert artifacts.planner_map_path("astar") == str(occupancy)


def test_default_map_dirs_put_env_map_dir_first(tmp_path: Path, monkeypatch) -> None:
    maps_dir = tmp_path / "maps"
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    assert default_map_dirs()[0] == maps_dir

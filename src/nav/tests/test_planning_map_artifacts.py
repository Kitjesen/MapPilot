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
    octomap = active / "octomap.bt"
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
    (active / "octomap.bt").write_bytes(b"active")
    explicit = tmp_path / "explicit.bt"
    explicit.write_bytes(b"explicit")
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    artifacts = SavedMapArtifacts.from_runtime(str(explicit))

    assert artifacts.planner_map_path("octoplanner3d") == str(explicit)


def test_saved_map_artifacts_resolve_active_map_record_bundle(
    tmp_path: Path,
    monkeypatch,
) -> None:
    maps_dir = tmp_path / "maps"
    active = maps_dir / "active"
    active.mkdir(parents=True)
    octomap = active / "octomap.bt"
    octomap.write_bytes(b"octomap")
    write_map_record(active, build_map_record(active, map_id="active_map"))
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    artifacts = SavedMapArtifacts.from_runtime()
    bundle = artifacts.planner_map_bundle("octoplanner3d")

    assert bundle["schema_version"] == "map.bundle"
    assert bundle["capability"] == "navigation_safety_3d"
    assert bundle["artifact"]["uri"] == str(octomap)
    assert artifacts.planner_map_path("octoplanner3d") == str(octomap)


def test_saved_map_artifacts_bundle_ignores_missing_artifact_uri(
    tmp_path: Path,
    monkeypatch,
) -> None:
    maps_dir = tmp_path / "maps"
    active = maps_dir / "active"
    active.mkdir(parents=True)
    octomap = active / "octomap.bt"
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
    octomap = active / "octomap.bt"
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

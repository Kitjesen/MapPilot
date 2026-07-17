from __future__ import annotations

from pathlib import Path

from nav.services.plan.global_planner.artifacts import SavedMapArtifacts, default_map_dirs


def _create_active_map(maps_dir: Path, name: str = "active_map") -> Path:
    active = maps_dir / name
    active.mkdir(parents=True)
    (maps_dir / "active_map.txt").write_text(f"{name}\n", encoding="utf-8")
    return active


def test_saved_map_artifacts_resolve_planner_specific_active_files(
    tmp_path: Path,
    monkeypatch,
) -> None:
    maps_dir = tmp_path / "maps"
    active = _create_active_map(maps_dir)
    octomap = active / "octomap.ot"
    occupancy = active / "occupancy.npz"
    octomap.write_bytes(b"octomap")
    occupancy.write_bytes(b"occupancy")
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    artifacts = SavedMapArtifacts.from_runtime()

    assert artifacts.planner_map_path("octoplanner3d") == str(octomap)
    assert artifacts.planner_map_path("pct") == ""
    assert artifacts.planner_map_path("astar") == str(occupancy)


def test_saved_map_artifacts_explicit_existing_path_wins(
    tmp_path: Path,
    monkeypatch,
) -> None:
    maps_dir = tmp_path / "maps"
    active = _create_active_map(maps_dir)
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
    active = _create_active_map(maps_dir)
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
    active = _create_active_map(maps_dir)
    octomap = active / "octomap.ot"
    octomap.write_bytes(b"octomap")
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    artifacts = SavedMapArtifacts.from_runtime()
    bundle = artifacts.planner_map_bundle("octoplanner3d")

    assert bundle["schema_version"] == "map.bundle"
    assert bundle["capability"] == "navigation_safety_3d"
    assert bundle["artifact"]["uri"] == str(octomap)
    assert artifacts.planner_map_path("octoplanner3d") == str(octomap)


def test_octoplanner3d_ignores_legacy_map_record_uri(
    tmp_path: Path,
    monkeypatch,
) -> None:
    maps_dir = tmp_path / "maps"
    active = _create_active_map(maps_dir)
    pcd = active / "map.pcd"
    octomap = active / "octomap.bt"
    pcd.write_bytes(b"pcd")
    octomap.write_bytes(b"octomap")
    (active / "map_record.json").write_text("not a native record\n", encoding="utf-8")
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    artifacts = SavedMapArtifacts.from_runtime()

    assert artifacts.planner_map_path("octoplanner3d") == str(octomap)


def test_saved_map_artifacts_bundle_ignores_missing_artifact_uri(
    tmp_path: Path,
    monkeypatch,
) -> None:
    maps_dir = tmp_path / "maps"
    active = _create_active_map(maps_dir)
    octomap = active / "octomap.ot"
    octomap.write_bytes(b"octomap")
    octomap.unlink()
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    artifacts = SavedMapArtifacts.from_runtime()

    assert artifacts.planner_map_bundle("octoplanner3d") == {}
    assert artifacts.planner_map_path("octoplanner3d") == ""


def test_saved_map_artifacts_resolve_static_occupancy_from_native_bundle(
    tmp_path: Path,
    monkeypatch,
) -> None:
    maps_dir = tmp_path / "maps"
    active = _create_active_map(maps_dir)
    octomap = active / "octomap.ot"
    occupancy = active / "occupancy.npz"
    octomap.write_bytes(b"octomap")
    occupancy.write_bytes(b"occupancy")
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    artifacts = SavedMapArtifacts.from_runtime()

    assert artifacts.static_occupancy_path(str(octomap)) == str(occupancy)


def test_astar_uses_static_occupancy_only(
    tmp_path: Path,
    monkeypatch,
) -> None:
    maps_dir = tmp_path / "maps"
    active = _create_active_map(maps_dir)
    occupancy = active / "occupancy.npz"
    occupancy.write_bytes(b"occupancy")
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    artifacts = SavedMapArtifacts.from_runtime()

    assert artifacts.planner_map_path("astar") == str(occupancy)


def test_default_map_dirs_put_env_map_dir_first(tmp_path: Path, monkeypatch) -> None:
    maps_dir = tmp_path / "maps"
    monkeypatch.setenv("NAV_MAP_DIR", str(maps_dir))

    assert default_map_dirs()[0] == maps_dir

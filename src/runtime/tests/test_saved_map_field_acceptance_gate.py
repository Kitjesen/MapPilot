from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

pytestmark = [pytest.mark.sim]

REPO_ROOT = Path(__file__).resolve().parents[3]


def _write_pcd(path: Path) -> None:
    path.write_text(
        "\n".join(
            [
                "# .PCD v0.7 - Point Cloud Data file format",
                "VERSION 0.7",
                "FIELDS x y z",
                "SIZE 4 4 4",
                "TYPE F F F",
                "COUNT 1 1 1",
                "WIDTH 3",
                "HEIGHT 1",
                "VIEWPOINT 0 0 0 1 0 0 0",
                "POINTS 3",
                "DATA ascii",
                "0 0 0",
                "1 0 0",
                "0 1 0",
                "",
            ]
        ),
        encoding="ascii",
    )


def _write_metadata(map_dir: Path) -> None:
    octomap = map_dir / "octomap.ot"
    octomap.write_bytes(
        b"# Octomap OcTree binary file\nid OcTree\nsize 1\nres 0.1\ndata\n\x00"
    )
    metadata = {
        "schema_version": "lingtu.saved_map_artifacts.v1",
        "source_profile": "field_acceptance_fixture",
        "data_source": "native_dds",
        "slam_source": "native_dds",
        "localization_source": "native_dds",
        "mapping_source": "livox_dds -> slam_dds -> mapd",
        "frame_id": "map",
        "created_at": "2026-01-01T00:00:00+00:00",
        "artifacts": {
            "map_pcd": {
                "path": "map.pcd",
                "source_profile": "field_acceptance_fixture",
                "data_source": "native_dds",
                "slam_source": "native_dds",
                "frame_id": "map",
                "point_count": 3,
            },
            "octomap": {
                "path": "octomap.ot",
                "source_profile": "field_acceptance_fixture",
                "data_source": "native_dds",
                "frame_id": "map",
            },
        },
    }
    (map_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _write_map(
    tmp_path: Path,
    *,
    trajectory_points: list[tuple[float, float]],
    pose_points: list[tuple[float, float]],
) -> Path:
    map_dir = tmp_path / "map"
    map_dir.mkdir()
    _write_pcd(map_dir / "map.pcd")
    _write_metadata(map_dir)
    patches = map_dir / "patches"
    patches.mkdir()
    for idx in range(len(pose_points)):
        _write_pcd(patches / f"scan_{idx:06d}.pcd")
    (map_dir / "trajectory.txt").write_text(
        "".join(f"{idx} {x} {y} 0 0 0 0 1\n" for idx, (x, y) in enumerate(trajectory_points)),
        encoding="utf-8",
    )
    (map_dir / "poses.txt").write_text(
        "".join(f"scan_{idx:06d}.pcd {x} {y} 0 1 0 0 0\n" for idx, (x, y) in enumerate(pose_points)),
        encoding="utf-8",
    )
    (map_dir / "poses_optimized.txt").write_text(
        (map_dir / "poses.txt").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    return map_dir


def _run_gate(map_dir: Path) -> subprocess.CompletedProcess[str]:
    script = REPO_ROOT / "scripts" / "gates" / "saved_map_field_acceptance.py"
    return subprocess.run(
        [sys.executable, str(script), str(map_dir), "--json"],
        cwd=str(REPO_ROOT),
        text=True,
        capture_output=True,
        check=False,
    )


def test_saved_map_field_acceptance_rejects_degenerate_patch_coverage(
    tmp_path: Path,
) -> None:
    map_dir = _write_map(
        tmp_path,
        trajectory_points=[(0.0, 0.0), (6.0, -5.0)],
        pose_points=[(5.7400 + idx * 0.0003, -5.0640) for idx in range(20)],
    )

    proc = _run_gate(map_dir)

    assert proc.returncode == 2, proc.stdout + proc.stderr
    payload = json.loads(proc.stdout)
    assert payload["ok"] is False
    assert payload["map_id"] == "map"
    assert "map_dir" not in payload
    assert payload["artifacts"]["octomap"]["exists"] is True
    assert payload["trajectory"]["trajectory_xy_length_m"] > 7.0
    assert payload["trajectory"]["pose_coverage_ratio"] < 0.01
    assert any("saved patch poses cover too little" in item for item in payload["blockers"])


def test_saved_map_field_acceptance_accepts_covered_keyframes(tmp_path: Path) -> None:
    points = [(idx * 0.5, 0.0) for idx in range(20)]
    map_dir = _write_map(
        tmp_path,
        trajectory_points=points,
        pose_points=points,
    )

    proc = _run_gate(map_dir)

    assert proc.returncode == 0, proc.stdout + proc.stderr
    payload = json.loads(proc.stdout)
    assert payload["ok"] is True
    assert payload["map_id"] == "map"
    assert "map_dir" not in payload
    assert payload["artifacts"]["octomap"]["exists"] is True
    assert payload["trajectory"]["pose_coverage_ratio"] == 1.0
    assert payload["trajectory"]["patch_count"] == 20
    assert payload["blockers"] == []


def test_saved_map_field_acceptance_rejects_missing_patch_file(tmp_path: Path) -> None:
    points = [(idx * 0.5, 0.0) for idx in range(20)]
    map_dir = _write_map(
        tmp_path,
        trajectory_points=points,
        pose_points=points,
    )
    (map_dir / "patches" / "scan_000019.pcd").unlink()

    proc = _run_gate(map_dir)

    assert proc.returncode == 2, proc.stdout + proc.stderr
    payload = json.loads(proc.stdout)
    assert payload["ok"] is False
    assert "patch file/pose count mismatch: 19 != 20" in payload["blockers"]


def test_saved_map_field_acceptance_requires_local_octomap(tmp_path: Path) -> None:
    points = [(idx * 0.5, 0.0) for idx in range(20)]
    map_dir = _write_map(
        tmp_path,
        trajectory_points=points,
        pose_points=points,
    )
    (map_dir / "octomap.ot").unlink()

    proc = _run_gate(map_dir)

    assert proc.returncode == 2, proc.stdout + proc.stderr
    payload = json.loads(proc.stdout)
    assert "octomap.ot missing" in payload["blockers"]

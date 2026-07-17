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
    from maps.artifacts import build_saved_map_metadata, sha256_file

    map_pcd = map_dir / "map.pcd"
    octomap = map_dir / "octomap.ot"
    tomogram = map_dir / "tomogram.pickle"
    octomap.write_bytes(b"octomap")
    tomogram.write_bytes(b"tomogram")
    map_sha = sha256_file(map_pcd)
    metadata = build_saved_map_metadata(
        source_profile="field_acceptance_fixture",
        data_source="native_dds",
        slam_source="native_dds",
        localization_source="native_dds",
        mapping_source="livox_dds -> slam_dds -> gateway_save",
        frame_id="map",
        artifacts={
            "map_pcd": {
                "path": "map.pcd",
                "sha256": map_sha,
                "source_profile": "field_acceptance_fixture",
                "data_source": "native_dds",
                "slam_source": "native_dds",
                "frame_id": "map",
                "point_count": 3,
            },
            "octomap": {
                "path": "octomap.ot",
                "sha256": sha256_file(octomap),
                "source_map_sha256": map_sha,
                "source_profile": "field_acceptance_fixture",
                "data_source": "native_dds",
                "frame_id": "map",
            },
            "tomogram": {
                "path": "tomogram.pickle",
                "sha256": sha256_file(tomogram),
                "source_map_sha256": map_sha,
                "source_profile": "field_acceptance_fixture",
                "data_source": "native_dds",
                "frame_id": "map",
                "shape": [2, 2, 2],
            },
        },
    )
    (map_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _write_optimization(map_dir: Path, *, status: str = "ok") -> None:
    (map_dir / "map_optimization.json").write_text(
        json.dumps(
            {
                "success": status == "ok",
                "status": status,
                "strategy": "pgo",
                "pose_count": 20,
                "patch_count": 20,
                "iterations": 0,
                "initial_cost": 0.0,
                "final_cost": 0.0,
            },
            indent=2,
        )
        + "\n",
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
    _write_optimization(map_dir)
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


def test_saved_map_field_acceptance_rejects_degenerate_optimizer_input(
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
    assert payload["artifact_gate"]["ok"] is True
    assert payload["trajectory"]["trajectory_xy_length_m"] > 7.0
    assert payload["trajectory"]["pose_coverage_ratio"] < 0.01
    assert any("optimizer keyframes cover too little" in item for item in payload["blockers"])


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
    assert payload["artifact_gate"]["ok"] is True
    assert payload["trajectory"]["pose_coverage_ratio"] == 1.0
    assert payload["blockers"] == []

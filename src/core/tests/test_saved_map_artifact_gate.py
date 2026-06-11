from __future__ import annotations

import pytest

pytestmark = [pytest.mark.sim]

import json
import subprocess
import sys
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[3]


def _write_same_source_map(tmp_path: Path, *, frame_id: str = "odom") -> Path:
    from core.same_source_map_artifacts import (
        write_same_source_map_artifacts,
    )

    report = write_same_source_map_artifacts(
        artifact_dir=tmp_path / "same_source_map",
        points=np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.2],
                [0.0, 1.0, 0.3],
            ],
            dtype=np.float32,
        ),
        frame_id=frame_id,
        world=tmp_path / "scene.xml",
        source_topics=("/nav/map_cloud",),
        mapping_input_path="/points_raw + /imu_raw -> fastlio2 -> /nav/map_cloud",
        build_tomogram=False,
        tomogram_resolution=0.2,
        tomogram_slice_dh=0.25,
        tomogram_ground_h=0.0,
        map_artifact_max_span_m=50.0,
        tomogram_max_cells=1_000_000,
        source="mujoco_fastlio2_live_gate",
    )
    assert report["ok"] is True, report
    return Path(report["artifact_dir"])


def test_saved_map_artifact_dir_gate_accepts_same_source_metadata(tmp_path: Path):
    from core.same_source_map_artifacts import (
        validate_saved_map_artifact_dir,
    )

    artifact_dir = _write_same_source_map(tmp_path)

    result = validate_saved_map_artifact_dir(artifact_dir)

    assert result["schema_version"] == "lingtu.saved_map_artifacts.gate.v1"
    assert result["ok"] is True
    assert result["blockers"] == []
    assert result["artifacts"]["map_pcd"]["sha256_ok"] is True
    assert set(result["checked_artifact_formats"]) == {
        "map_pcd",
        "tomogram",
        "occupancy_grid",
        "metadata",
    }
    assert result["checked_required_artifacts"] == ["map_pcd"]
    assert result["checked_allowed_frame_ids"] == ["map", "odom"]
    assert result["checked_frame_id"] == "odom"
    assert result["checked_expected"] == {
        "data_source": None,
        "source_profile": None,
        "frame_id": None,
    }
    assert result["checked_source_fields"] == [
        "source_profile",
        "data_source",
        "slam_source",
        "localization_source",
        "mapping_source",
    ]


def test_tomogram_builder_loader_falls_back_to_legacy_pct_script_dir(
    tmp_path: Path,
    monkeypatch,
):
    import builtins

    import core.same_source_map_artifacts as artifacts

    script_dir = tmp_path / "PCT_planner" / "tomography" / "scripts"
    script_dir.mkdir(parents=True)
    (script_dir / "build_tomogram.py").write_text(
        "def build_tomogram_from_pcd(*args, **kwargs):\n"
        "    return {'data': [1], 'source': 'legacy'}\n",
        encoding="utf-8",
    )
    monkeypatch.setattr(artifacts, "_tomography_script_dirs", lambda: [script_dir])

    real_import = builtins.__import__

    def fake_import(name, *args, **kwargs):
        if name == "global_planning.pct_planner.tomography.scripts.build_tomogram":
            raise ModuleNotFoundError(name)
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fake_import)
    sys.modules.pop("build_tomogram", None)
    try:
        builder = artifacts._load_build_tomogram_from_pcd()
        assert builder("map.pcd", "tomogram.pickle") == {
            "data": [1],
            "source": "legacy",
        }
    finally:
        sys.modules.pop("build_tomogram", None)
        while str(script_dir) in sys.path:
            sys.path.remove(str(script_dir))


def test_saved_map_artifact_dir_gate_reports_expected_source_contract(
    tmp_path: Path,
):
    from core.same_source_map_artifacts import (
        validate_saved_map_artifact_dir,
    )

    artifact_dir = _write_same_source_map(tmp_path, frame_id="/odom")

    result = validate_saved_map_artifact_dir(
        artifact_dir,
        expected_data_source="mujoco_fastlio2_live_gate",
        expected_source_profile="mujoco_fastlio2_live_gate",
        expected_frame_id="/odom",
    )

    assert result["ok"] is True
    assert result["blockers"] == []
    assert result["checked_expected"] == {
        "data_source": "mujoco_fastlio2_live_gate",
        "source_profile": "mujoco_fastlio2_live_gate",
        "frame_id": "odom",
    }
    assert result["checked_frame_id"] == "odom"


def test_saved_map_artifact_dir_gate_rejects_missing_metadata(tmp_path: Path):
    from core.same_source_map_artifacts import (
        validate_saved_map_artifact_dir,
        write_ascii_pcd,
    )

    artifact_dir = tmp_path / "map_without_metadata"
    write_ascii_pcd(
        artifact_dir / "map.pcd",
        np.array([[0.0, 0.0, 0.0]], dtype=np.float32),
    )

    result = validate_saved_map_artifact_dir(artifact_dir)

    assert result["ok"] is False
    assert "metadata.json missing" in result["blockers"]


def test_saved_map_artifact_dir_gate_rejects_file_sha_drift(tmp_path: Path):
    from core.same_source_map_artifacts import (
        validate_saved_map_artifact_dir,
    )

    artifact_dir = _write_same_source_map(tmp_path)
    with (artifact_dir / "map.pcd").open("a", encoding="ascii") as fh:
        fh.write("2.0 2.0 2.0\n")

    result = validate_saved_map_artifact_dir(artifact_dir)

    assert result["ok"] is False
    assert "map_pcd sha256 does not match file" in result["blockers"]


def test_saved_map_artifact_dir_gate_rejects_expected_frame_mismatch(
    tmp_path: Path,
):
    from core.same_source_map_artifacts import (
        validate_saved_map_artifact_dir,
    )

    artifact_dir = _write_same_source_map(tmp_path)

    result = validate_saved_map_artifact_dir(
        artifact_dir,
        expected_frame_id="map",
    )

    assert result["ok"] is False
    assert "metadata.frame_id does not match expected frame_id" in result["blockers"]


def test_saved_map_artifact_dir_gate_normalizes_expected_frame_ids(
    tmp_path: Path,
):
    from core.same_source_map_artifacts import (
        validate_saved_map_artifact_dir,
    )

    artifact_dir = _write_same_source_map(tmp_path, frame_id="/odom")
    metadata = json.loads((artifact_dir / "metadata.json").read_text(encoding="utf-8"))

    result = validate_saved_map_artifact_dir(
        artifact_dir,
        expected_frame_id="/odom",
    )

    assert result["ok"] is True
    assert result["blockers"] == []
    assert metadata["frame_id"] == "odom"
    assert metadata["artifacts"]["map_pcd"]["frame_id"] == "odom"


def test_saved_map_artifact_dir_gate_accepts_repo_relative_artifact_paths(
    tmp_path: Path,
    monkeypatch,
):
    from core.same_source_map_artifacts import (
        build_saved_map_metadata,
        sha256_file,
        validate_saved_map_artifact_dir,
    )

    artifact_dir = (
        tmp_path
        / "artifacts/server_sim_closure/runtime/run/same_source_map"
    )
    artifact_dir.mkdir(parents=True)
    map_pcd = artifact_dir / "map.pcd"
    tomogram = artifact_dir / "tomogram.pickle"
    map_pcd.write_text("VERSION 0.7\nDATA ascii\n", encoding="ascii")
    tomogram.write_bytes(b"tomogram")
    map_rel = map_pcd.relative_to(tmp_path).as_posix()
    tomogram_rel = tomogram.relative_to(tmp_path).as_posix()
    metadata = build_saved_map_metadata(
        source_profile="mujoco_fastlio2_live_gate",
        data_source="fastlio2",
        slam_source="fastlio2",
        localization_source="fastlio2",
        mapping_source="/points_raw -> fastlio2 -> /nav/map_cloud",
        frame_id="odom",
        artifacts={
            "map_pcd": {
                "path": map_rel,
                "sha256": sha256_file(map_pcd),
                "point_count": 1,
                "source_profile": "mujoco_fastlio2_live_gate",
                "data_source": "fastlio2",
                "slam_source": "fastlio2",
                "frame_id": "odom",
            },
            "tomogram": {
                "path": tomogram_rel,
                "sha256": sha256_file(tomogram),
                "source_map_sha256": sha256_file(map_pcd),
                "source_profile": "mujoco_fastlio2_live_gate",
                "data_source": "fastlio2",
                "frame_id": "odom",
                "shape": [1, 1],
            },
        },
    )
    (artifact_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    monkeypatch.chdir(tmp_path)

    result = validate_saved_map_artifact_dir(
        artifact_dir,
        require_tomogram=True,
        expected_frame_id="odom",
    )

    assert result["ok"] is True
    assert result["blockers"] == []
    assert result["artifacts"]["map_pcd"]["path"] == str(map_pcd)
    assert result["artifacts"]["tomogram"]["path"] == str(tomogram)


def test_saved_map_artifact_gate_script_rejects_missing_required_tomogram(
    tmp_path: Path,
):
    artifact_dir = _write_same_source_map(tmp_path)
    script = REPO_ROOT / "scripts" / "saved_map_artifact_gate.py"

    proc = subprocess.run(
        [
            sys.executable,
            str(script),
            str(artifact_dir),
            "--require-tomogram",
            "--json",
        ],
        cwd=str(REPO_ROOT),
        text=True,
        capture_output=True,
        check=False,
    )

    assert proc.returncode == 2
    payload = json.loads(proc.stdout)
    assert payload["ok"] is False
    assert payload["validation_gate"]["acceptance_step"] == 2
    assert payload["validation_gate"]["requires_prior_gates"] == ["runtime_audit"]
    assert payload["validation_gate"]["operator_summary_sections"] == [
        "Expected",
        "Required artifacts",
        "Metadata",
        "Artifacts",
        "Blockers",
    ]
    assert (
        "artifact_gate_payload_declares_checked_artifacts_frames_sources"
        in payload["validation_gate"]["validates"]
    )
    assert "tomogram required but missing" in payload["blockers"]
    assert payload["checked_required_artifacts"] == ["map_pcd", "tomogram"]
    assert payload["checked_frame_id"] == "odom"


def test_saved_map_artifact_gate_script_prints_operator_summary_by_default(
    tmp_path: Path,
):
    artifact_dir = _write_same_source_map(tmp_path)
    script = REPO_ROOT / "scripts" / "saved_map_artifact_gate.py"

    proc = subprocess.run(
        [sys.executable, str(script), str(artifact_dir), "--require-tomogram"],
        cwd=str(REPO_ROOT),
        text=True,
        capture_output=True,
        check=False,
    )

    assert proc.returncode == 2
    assert "Saved map artifact gate: FAIL" in proc.stdout
    assert f"Map dir: {artifact_dir}" in proc.stdout
    assert "Validation gate:" in proc.stdout
    assert (
        "  step=2 "
        "required_when=saved_map_tomogram_occupancy_or_pct_artifact_is_used "
        "prior=runtime_audit"
    ) in proc.stdout
    assert (
        "proves=saved_map_metadata_exists,"
        "map_pcd_checksum_matches_metadata,"
        "tomogram_and_occupancy_derive_from_same_map_pcd,"
        "artifact_frame_and_source_metadata_are_consistent"
    ) in proc.stdout
    assert "Frame: observed=odom allowed=map,odom" in proc.stdout
    assert "Required artifacts:" in proc.stdout
    assert "  map_pcd" in proc.stdout
    assert "  tomogram" in proc.stdout
    assert "Artifacts:" in proc.stdout
    assert "  map_pcd exists=true sha256_ok=true" in proc.stdout
    assert "Blockers:" in proc.stdout
    assert "  tomogram required but missing" in proc.stdout
    assert '"schema_version":' not in proc.stdout

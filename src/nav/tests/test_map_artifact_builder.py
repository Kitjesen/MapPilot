"""Tests for the saved-map MapArtifactBuilder."""

from __future__ import annotations

import json
import sys
from pathlib import Path

from nav.services.map_layers.map_artifact_builder import (
    SUPPORTED_BUILD_MODES,
    MapArtifactBuilder,
    MapArtifactBuilderConfig,
    build_for_saved_map,
)
from runtime.same_source_map_artifacts import (
    sha256_file,
    validate_same_source_map_metadata,
)


def _write_minimal_pcd(map_dir: Path) -> Path:
    map_dir.mkdir(parents=True, exist_ok=True)
    pcd_path = map_dir / "map.pcd"
    pcd_path.write_text(
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
    return pcd_path


def _fake_converter_command(tmp_path: Path) -> tuple[str, ...]:
    script = tmp_path / "fake_pcd_to_octomap.py"
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
        "source = Path(args.input)\n"
        "payload = (\n"
        "    f'FAKE_BT resolution={args.resolution} frame={args.frame}\\n'\n"
        "    .encode('utf-8')\n"
        ")\n"
        "Path(args.output).write_bytes(payload + source.read_bytes())\n",
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


def test_missing_map_pcd_reports_clear_failure(tmp_path: Path) -> None:
    map_dir = tmp_path / "saved_map"
    map_dir.mkdir()

    report = build_for_saved_map(
        map_dir,
        converter_command=None,
        use_env_converter=False,
    )

    assert report.ok is False
    assert report.status == "missing_map_pcd"
    assert any("map.pcd" in blocker for blocker in report.blockers)
    assert not (map_dir / "octomap.ot").exists()
    assert not (map_dir / "metadata.json").exists()


def test_existing_map_pcd_without_converter_fails_without_fake_octomap(
    tmp_path: Path,
) -> None:
    map_dir = tmp_path / "source_only_map"
    _write_minimal_pcd(map_dir)

    report = build_for_saved_map(
        map_dir,
        converter_command=None,
        use_env_converter=False,
    )

    assert report.ok is False
    assert report.status == "missing_converter"
    assert any("converter" in blocker for blocker in report.blockers)
    assert not (map_dir / "octomap.ot").exists()
    assert not (map_dir / "metadata.json").exists()


def test_fake_converter_success_builds_octomap_and_metadata(tmp_path: Path) -> None:
    map_dir = tmp_path / "saved_map"
    _write_minimal_pcd(map_dir)

    builder = MapArtifactBuilder(
        MapArtifactBuilderConfig(
            converter_command=_fake_converter_command(tmp_path),
            use_env_converter=False,
            resolution=0.15,
            frame_id="map",
        )
    )
    report = builder.build_for_saved_map(map_dir)

    assert report.ok is True, report.to_dict()
    assert report.status == "built"
    assert report.reused is False
    assert report.converter["returncode"] == 0
    assert (map_dir / "octomap.ot").read_bytes().startswith(b"FAKE_BT")
    assert (map_dir / "metadata.json").is_file()


def test_octoplanner3d_converter_env_alias_is_supported(monkeypatch) -> None:
    monkeypatch.delenv("LINGTU_MAP_ARTIFACT_CONVERTER", raising=False)
    monkeypatch.delenv("LINGTU_OCTOMAP_CONVERTER", raising=False)
    monkeypatch.setenv(
        "LINGTU_OCTOPLANNER3D_PCD_CONVERTER",
        "/opt/lingtu/bin/octoplanner3d_pcd_to_octomap",
    )

    config = MapArtifactBuilderConfig(converter_command=None, use_env_converter=True)

    assert (
        config.resolved_converter_command()
        == "/opt/lingtu/bin/octoplanner3d_pcd_to_octomap"
    )


def test_metadata_schema_records_sources_builder_and_octomap(tmp_path: Path) -> None:
    map_dir = tmp_path / "schema_map"
    pcd_path = _write_minimal_pcd(map_dir)

    report = build_for_saved_map(
        map_dir,
        converter_command=_fake_converter_command(tmp_path),
        use_env_converter=False,
        resolution=0.25,
        frame_id="map",
    )
    assert report.ok is True, report.to_dict()

    metadata = json.loads((map_dir / "metadata.json").read_text(encoding="utf-8"))
    pcd_sha = sha256_file(pcd_path)
    octomap_sha = sha256_file(map_dir / "octomap.ot")

    assert metadata["schema_version"] == "lingtu.saved_map_artifacts.v1"
    assert metadata["build_mode"] == "external_pcl_converter"
    assert metadata["resolution"] == 0.25
    assert metadata["free_layers_above"] == 3
    assert metadata["free_dilation_cells"] == 1
    assert metadata["frame"] == "map"
    assert metadata["frame_id"] == "map"
    assert metadata["builder"]["name"] == "LingTu MapArtifactBuilder"
    assert metadata["builder"]["version"] == metadata["builder_version"]
    assert set(SUPPORTED_BUILD_MODES).issubset(set(metadata["supported_build_modes"]))
    assert metadata["source_hashes"]["map_pcd"] == pcd_sha

    artifacts = metadata["artifacts"]
    assert artifacts["map_pcd"]["sha256"] == pcd_sha
    assert artifacts["map_pcd"]["point_count"] == 1
    assert artifacts["octomap"]["path"] == "octomap.ot"
    assert artifacts["octomap"]["sha256"] == octomap_sha
    assert artifacts["octomap"]["source_map_sha256"] == pcd_sha
    assert artifacts["octomap"]["build_mode"] == "external_pcl_converter"
    assert artifacts["octomap"]["resolution"] == 0.25
    assert artifacts["octomap"]["free_layers_above"] == 3
    assert artifacts["octomap"]["free_dilation_cells"] == 1

    validation = validate_same_source_map_metadata(metadata)
    assert validation["ok"], validation


def test_reuse_when_hash_matches_metadata_without_converter(tmp_path: Path) -> None:
    map_dir = tmp_path / "reuse_map"
    _write_minimal_pcd(map_dir)

    first = build_for_saved_map(
        map_dir,
        converter_command=_fake_converter_command(tmp_path),
        use_env_converter=False,
        resolution=0.2,
        frame_id="map",
    )
    assert first.ok is True, first.to_dict()
    octomap_sha = sha256_file(map_dir / "octomap.ot")
    metadata_before = json.loads((map_dir / "metadata.json").read_text(encoding="utf-8"))

    second = build_for_saved_map(
        map_dir,
        converter_command=None,
        use_env_converter=False,
        resolution=0.2,
        frame_id="map",
    )

    assert second.ok is True, second.to_dict()
    assert second.status == "reused"
    assert second.reused is True
    assert second.converter == {}
    assert sha256_file(map_dir / "octomap.ot") == octomap_sha
    metadata_after = json.loads((map_dir / "metadata.json").read_text(encoding="utf-8"))
    assert metadata_after == metadata_before


def test_preserved_artifacts_reject_path_escape(tmp_path: Path) -> None:
    map_dir = tmp_path / "escape_map"
    pcd_path = _write_minimal_pcd(map_dir)
    outside = tmp_path / "outside.npz"
    outside.write_bytes(b"outside occupancy")
    pcd_sha = sha256_file(pcd_path)
    (map_dir / "metadata.json").write_text(
        json.dumps(
            {
                "source_profile": "map_artifact_builder",
                "data_source": "map_artifact_builder",
                "slam_source": "unknown",
                "localization_source": "unknown",
                "mapping_source": "old",
                "frame_id": "map",
                "created_at": "2026-01-01T00:00:00+00:00",
                "artifacts": {
                    "occupancy_grid": {
                        "path": "../outside.npz",
                        "sha256": sha256_file(outside),
                        "source_map_sha256": pcd_sha,
                        "source_profile": "map_artifact_builder",
                        "data_source": "map_artifact_builder",
                        "frame_id": "map",
                        "shape": [1, 1],
                    }
                },
            }
        ),
        encoding="utf-8",
    )

    report = build_for_saved_map(
        map_dir,
        converter_command=_fake_converter_command(tmp_path),
        use_env_converter=False,
        frame_id="map",
    )

    assert report.ok is True, report.to_dict()
    metadata = json.loads((map_dir / "metadata.json").read_text(encoding="utf-8"))
    assert "occupancy_grid" not in metadata["artifacts"]

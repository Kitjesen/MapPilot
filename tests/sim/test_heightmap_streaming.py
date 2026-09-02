# ruff: noqa: S101
"""Streaming heightmap artifact construction contracts."""

from __future__ import annotations

import struct
from pathlib import Path

import pytest

from sim.catalog.importers.heightmap import (
    build_heightmap_artifacts,
    build_heightmap_artifacts_from_u16_file,
)


def _samples(width: int, height: int) -> tuple[int, ...]:
    return tuple((row * 997 + column * 313) % 65_536 for row in range(height) for column in range(width))


def test_streaming_little_endian_artifacts_match_sequence_builder_byte_for_byte(tmp_path: Path) -> None:
    width = height = 17
    samples = _samples(width, height)
    source = tmp_path / "height-little.r16"
    source.write_bytes(struct.pack(f"<{len(samples)}H", *samples))

    sequence = build_heightmap_artifacts(
        samples=samples,
        width=width,
        height=height,
        extent_m=(80.0, 120.0),
        elevation_min_m=-3.0,
        elevation_max_m=11.0,
        spawn_xy_m=(5.0, -7.5),
        artifact_root=tmp_path / "sequence" / "artifacts",
        mesh_name="equivalent_terrain",
    )
    streamed = build_heightmap_artifacts_from_u16_file(
        source_path=source,
        width=width,
        height=height,
        endian="little",
        extent_m=(80.0, 120.0),
        elevation_min_m=-3.0,
        elevation_max_m=11.0,
        spawn_xy_m=(5.0, -7.5),
        artifact_root=tmp_path / "streamed" / "artifacts",
        mesh_name="equivalent_terrain",
    )

    assert streamed == sequence
    for name in ("heightfield_f32.bin", "heightfield_r16.png", "terrain.obj"):
        assert (tmp_path / "streamed" / "artifacts" / name).read_bytes() == (
            tmp_path / "sequence" / "artifacts" / name
        ).read_bytes()


def test_streaming_big_endian_preserves_png_and_reverses_rows_for_mujoco(tmp_path: Path) -> None:
    width, height = 17, 33
    samples = _samples(width, height)
    source = tmp_path / "height-big.r16"
    source_payload = struct.pack(f">{len(samples)}H", *samples)
    source.write_bytes(source_payload)

    expected = build_heightmap_artifacts(
        samples=samples,
        width=width,
        height=height,
        extent_m=(64.0, 128.0),
        elevation_min_m=2.0,
        elevation_max_m=22.0,
        spawn_xy_m=(0.0, 0.0),
        artifact_root=tmp_path / "expected" / "artifacts",
    )
    actual = build_heightmap_artifacts_from_u16_file(
        source_path=source,
        width=width,
        height=height,
        endian="big",
        extent_m=(64.0, 128.0),
        elevation_min_m=2.0,
        elevation_max_m=22.0,
        spawn_xy_m=(0.0, 0.0),
        artifact_root=tmp_path / "actual" / "artifacts",
    )

    assert actual == expected
    assert source.read_bytes() == source_payload
    assert actual.sample_min_u16 == min(samples)
    assert actual.sample_max_u16 == max(samples)
    assert actual.sampled_elevation_min_m == pytest.approx(2.0 + min(samples) / 65_535 * 20.0)
    assert actual.sampled_elevation_max_m == pytest.approx(2.0 + max(samples) / 65_535 * 20.0)
    assert actual.sampled_elevation_range_m == pytest.approx(
        actual.sampled_elevation_max_m - actual.sampled_elevation_min_m
    )
    assert actual.mujoco_elevation_scale_m == pytest.approx(actual.sampled_elevation_range_m)
    normalization = actual.alignment["physics"]["normalization"]
    assert normalization["geom_origin_z_m"] == pytest.approx(actual.sampled_elevation_min_m)
    assert normalization["hfield_elevation_scale_m"] == pytest.approx(actual.sampled_elevation_range_m)
    assert normalization["flat_field_epsilon_m"] == 0.0
    f32_payload = (tmp_path / "actual" / "artifacts" / "heightfield_f32.bin").read_bytes()
    nrow, ncol = struct.unpack_from("<ii", f32_payload)
    values = struct.unpack_from(f"<{width * height}f", f32_payload, 8)
    assert (nrow, ncol) == (height, width)
    assert values[:width] == pytest.approx([sample / 65_535.0 for sample in samples[-width:]])
    assert values[-width:] == pytest.approx([sample / 65_535.0 for sample in samples[:width]])


@pytest.mark.parametrize("builder_kind", ["sequence", "streaming"])
def test_artifact_builders_can_skip_unreal_obj(tmp_path: Path, builder_kind: str) -> None:
    width, height = 17, 33
    samples = _samples(width, height)
    artifact_root = tmp_path / builder_kind / "artifacts"
    artifact_root.mkdir(parents=True)
    stale_obj = artifact_root / "terrain.obj"
    stale_obj.write_text("stale", encoding="ascii")
    common = {
        "width": width,
        "height": height,
        "extent_m": (32.0, 64.0),
        "elevation_min_m": -1.0,
        "elevation_max_m": 4.0,
        "spawn_xy_m": (0.0, 0.0),
        "artifact_root": artifact_root,
        "emit_unreal_obj": False,
    }
    if builder_kind == "sequence":
        artifacts = build_heightmap_artifacts(samples=samples, **common)
    else:
        source = tmp_path / "height.r16"
        source.write_bytes(struct.pack(f"<{len(samples)}H", *samples))
        artifacts = build_heightmap_artifacts_from_u16_file(source_path=source, **common)

    assert not stale_obj.exists()
    assert [asset["role"] for asset in artifacts.assets] == ["mujoco_hfield", "unreal_height_png"]
    assert artifacts.alignment["visual"]["format"] == "ue_16bit_png"

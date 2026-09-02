# ruff: noqa: S101

"""Contracts for the deterministic OpenField_HF source terrain."""

from __future__ import annotations

import hashlib
import json
import struct
import zlib
from pathlib import Path

import pytest
import yaml

from sim.catalog import CatalogResolver
from sim.tools.worlds.open_field_hf.generate import generate_open_field_hf

REPO_ROOT = Path(__file__).resolve().parents[2]


GENERATED_PATHS = (
    Path("sim/packages/worlds/open_field_hf/world.package.yaml"),
    Path("sim/packages/worlds/open_field_hf/provenance/terrain.provenance.json"),
    Path("sim/packages/worlds/open_field_hf/visual/ue_import.recipe.json"),
    Path("sim/packages/worlds/open_field_hf/physics/open_field_hf.xml"),
    Path("sim/packages/worlds/open_field_hf/generated/heightfield_r16.png"),
    Path("sim/packages/worlds/open_field_hf/generated/heightfield_f32.bin"),
    Path("sim/packages/worlds/open_field_hf/generated/terrain.obj"),
    Path("sim/packages/worlds/open_field_hf/generated/asset-manifest.json"),
)


def _generated_bytes(repo_root: Path) -> dict[Path, bytes]:
    return {path: (repo_root / path).read_bytes() for path in GENERATED_PATHS}


def test_checked_in_assets_match_the_deterministic_generator(
    tmp_path: Path,
) -> None:
    generated_repo = tmp_path / "generated-repo"

    generate_open_field_hf(generated_repo, seed=20260807)

    assert _generated_bytes(REPO_ROOT) == _generated_bytes(generated_repo)


def test_generation_is_byte_for_byte_deterministic(tmp_path: Path) -> None:
    first = tmp_path / "first"
    second = tmp_path / "second"

    generate_open_field_hf(first, seed=20260807)
    generate_open_field_hf(second, seed=20260807)

    assert _generated_bytes(first) == _generated_bytes(second)


def test_world_package_is_catalog_legal_and_keeps_production_metadata_separate(
    tmp_path: Path,
) -> None:
    generate_open_field_hf(tmp_path)
    package_root = tmp_path / "sim/packages/worlds/open_field_hf"

    CatalogResolver(tmp_path, (package_root,))
    manifest = yaml.safe_load((package_root / "world.package.yaml").read_text(encoding="utf-8"))

    assert set(manifest) == {
        "schema",
        "id",
        "version",
        "kind",
        "description",
        "physics",
        "visual",
        "entities",
    }
    assert manifest["physics"] == {
        "mjcf": "physics/open_field_hf.xml",
        "global_policy": {
            "timestep_s": 0.002,
            "integrator": "rk4",
            "solver": "newton",
            "iterations": 100,
            "gravity_mps2": [0.0, 0.0, -9.81],
        },
    }
    assert manifest["visual"] == {
        "binding": "WorldVisual:OpenFieldHF",
        "level": "/Game/RobotSim/Maps/OpenFieldRuntime",
    }


def _read_png_u16(path: Path) -> tuple[int, int, int, int, list[int]]:
    payload = path.read_bytes()
    assert payload.startswith(b"\x89PNG\r\n\x1a\n")
    offset = 8
    width = height = bit_depth = color_type = 0
    compressed = bytearray()
    while offset < len(payload):
        length = struct.unpack_from(">I", payload, offset)[0]
        kind = payload[offset + 4 : offset + 8]
        data = payload[offset + 8 : offset + 8 + length]
        offset += 12 + length
        if kind == b"IHDR":
            width, height, bit_depth, color_type = struct.unpack_from(">IIBB", data)
        elif kind == b"IDAT":
            compressed.extend(data)
        elif kind == b"IEND":
            break

    raw = zlib.decompress(bytes(compressed))
    row_bytes = 1 + width * 2
    assert len(raw) == row_bytes * height
    samples: list[int] = []
    for row in range(height):
        row_start = row * row_bytes
        assert raw[row_start] == 0
        samples.extend(struct.unpack_from(f">{width}H", raw, row_start + 1))
    return width, height, bit_depth, color_type, samples


def test_heightfield_and_obj_use_the_same_samples_and_coordinate_scale(
    tmp_path: Path,
) -> None:
    generate_open_field_hf(tmp_path)
    generated_root = tmp_path / "sim/packages/worlds/open_field_hf/generated"
    asset_manifest = json.loads((generated_root / "asset-manifest.json").read_text(encoding="utf-8"))
    contract = asset_manifest["coordinate_contract"]
    width, height, bit_depth, color_type, samples = _read_png_u16(generated_root / "heightfield_r16.png")
    vertices = [
        [float(value) for value in line.split()[1:]]
        for line in (generated_root / "terrain.obj").read_text(encoding="ascii").splitlines()
        if line.startswith("v ")
    ]

    assert [width, height] == contract["grid_px"] == [253, 253]
    assert bit_depth == 16
    assert color_type == 0
    assert contract["extent_m"] == [160.0, 160.0]
    assert contract["sample_spacing_m"] == pytest.approx([160.0 / 252.0, 160.0 / 252.0])
    assert len(vertices) == len(samples) == width * height

    selected = [0, width - 1, (height // 2) * width + width // 2, width * height - 1]
    spacing_x, spacing_y = contract["sample_spacing_m"]
    elevation_scale_m = contract["height_encoding"]["elevation_scale_m"]
    vertical_origin_m = contract["vertical_origin_m"]
    for index in selected:
        row, column = divmod(index, width)
        assert vertices[index] == pytest.approx(
            [
                (-80.0 + column * spacing_x) * 100.0,
                -(80.0 - row * spacing_y) * 100.0,
                (samples[index] / 65_535 * elevation_scale_m + vertical_origin_m) * 100.0,
            ],
            abs=1e-5,
        )

    center_index = (height // 2) * width + width // 2
    assert vertices[center_index][2] == pytest.approx(0.0, abs=1e-5)
    assert contract["spawn_reference"] == {
        "sample_pixel": [width // 2, height // 2],
        "world_position_m": [0.0, 0.0, 0.0],
        "sample_u16": samples[center_index],
    }

    terrain_asset = next(item for item in asset_manifest["assets"] if item["role"] == "unreal_mesh")
    assert terrain_asset["vertex_count"] == width * height
    assert terrain_asset["triangle_count"] == 2 * (width - 1) * (height - 1)


def _records_digest(records: list[dict[str, object]]) -> str:
    identity = [
        {"path": record["path"], "sha256": record["sha256"]}
        for record in sorted(records, key=lambda item: str(item["path"]))
    ]
    payload = (
        json.dumps(
            identity,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            separators=(",", ": "),
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def test_asset_manifest_and_provenance_digest_exact_generated_bytes(
    tmp_path: Path,
) -> None:
    generate_open_field_hf(tmp_path)
    generated_root = tmp_path / "sim/packages/worlds/open_field_hf/generated"
    package_root = tmp_path / "sim/packages/worlds/open_field_hf"
    asset_manifest = json.loads((generated_root / "asset-manifest.json").read_text(encoding="utf-8"))
    provenance = json.loads((package_root / "provenance/terrain.provenance.json").read_text(encoding="utf-8"))

    for record in asset_manifest["assets"]:
        payload = (tmp_path / str(record["path"])).read_bytes()
        assert record["bytes"] == len(payload)
        assert record["sha256"] == hashlib.sha256(payload).hexdigest()
    assert asset_manifest["asset_set_digest"] == _records_digest(asset_manifest["assets"])

    for record in provenance["outputs"]:
        payload = (tmp_path / str(record["path"])).read_bytes()
        assert record["bytes"] == len(payload)
        assert record["sha256"] == hashlib.sha256(payload).hexdigest()
    assert provenance["asset_set_digest"] == _records_digest(provenance["outputs"])
    assert provenance["source"] == {
        "type": "procedural",
        "owner": "LingTu project",
        "license": "LicenseRef-LingTu-Project-Owned",
        "third_party_assets": [],
    }


def test_mujoco_310_compiles_generated_world_when_available(
    tmp_path: Path,
) -> None:
    mujoco = pytest.importorskip("mujoco")
    generate_open_field_hf(tmp_path)
    model_path = tmp_path / "sim/packages/worlds/open_field_hf/physics/open_field_hf.xml"

    model = mujoco.MjModel.from_xml_path(str(model_path))
    hfield_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_HFIELD,
        "open_field_hf_terrain",
    )
    terrain_geom_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        "terrain",
    )

    assert hfield_id >= 0
    assert terrain_geom_id >= 0
    assert model.hfield_nrow[hfield_id] == 253
    assert model.hfield_ncol[hfield_id] == 253
    assert model.hfield_size[hfield_id].tolist() == pytest.approx([80.0, 80.0, 8.0, 1.0])
    assert model.geom_type[terrain_geom_id] == mujoco.mjtGeom.mjGEOM_HFIELD
    assert model.geom_dataid[terrain_geom_id] == hfield_id


def test_mujoco_binary_hfield_preserves_the_authoritative_u16_samples(
    tmp_path: Path,
) -> None:
    mujoco = pytest.importorskip("mujoco")
    generate_open_field_hf(tmp_path)
    world_root = tmp_path / "sim/packages/worlds/open_field_hf"
    width, height, _, _, samples = _read_png_u16(world_root / "generated/heightfield_r16.png")
    binary_payload = (world_root / "generated/heightfield_f32.bin").read_bytes()
    nrow, ncol = struct.unpack_from("<ii", binary_payload)
    binary_samples = struct.unpack_from(f"<{nrow * ncol}f", binary_payload, 8)
    expected_internal = [
        samples[row * width + column] / 65_535 for row in reversed(range(height)) for column in range(width)
    ]

    assert [nrow, ncol] == [height, width]
    assert len(binary_payload) == 8 + 4 * width * height
    assert min(samples) == 0
    assert max(samples) == 65_535
    assert binary_samples == pytest.approx(expected_internal, abs=1e-7)

    model = mujoco.MjModel.from_xml_path(str(world_root / "physics" / "open_field_hf.xml"))
    hfield_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_HFIELD,
        "open_field_hf_terrain",
    )
    data_address = int(model.hfield_adr[hfield_id])
    loaded = model.hfield_data[data_address : data_address + width * height].tolist()
    assert loaded == pytest.approx(expected_internal, abs=1e-7)
    assert 'file="../generated/heightfield_f32.bin"' in (
        world_root / "physics" / "open_field_hf.xml"
    ).read_text(encoding="utf-8")

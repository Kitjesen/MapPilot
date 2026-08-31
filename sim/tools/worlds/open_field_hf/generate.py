"""Generate deterministic OpenField_HF physics and visual source assets.

Only Python standard-library facilities are used. One unsigned 16-bit sample
grid is the authority for the MuJoCo heightfield, the Unreal import heightmap,
and every vertex in the OBJ terrain mesh.
"""

from __future__ import annotations

import argparse
import binascii
import hashlib
import json
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

DEFAULT_SEED = 20260807
_U16_MAX = 65_535
_Q16 = 1 << 16
_U64_MASK = (1 << 64) - 1


@dataclass(frozen=True)
class TerrainSpec:
    """Immutable physical and sampling contract for OpenField_HF."""

    width_px: int = 253
    height_px: int = 253
    size_x_m: float = 160.0
    size_y_m: float = 160.0
    elevation_scale_m: float = 8.0
    base_depth_m: float = 1.0

    @property
    def spacing_x_m(self) -> float:
        """Return horizontal sample spacing in metres."""

        return self.size_x_m / (self.width_px - 1)

    @property
    def spacing_y_m(self) -> float:
        """Return vertical sample spacing in metres."""

        return self.size_y_m / (self.height_px - 1)


@dataclass(frozen=True)
class GeneratedWorld:
    """Paths and content identity emitted by one generation."""

    repo_root: Path
    package_root: Path
    world_root: Path
    asset_set_digest: str


def _canonical_json(value: object) -> bytes:
    return (
        json.dumps(
            value,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            separators=(",", ": "),
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _splitmix64(value: int) -> int:
    value = (value + 0x9E3779B97F4A7C15) & _U64_MASK
    value = ((value ^ (value >> 30)) * 0xBF58476D1CE4E5B9) & _U64_MASK
    value = ((value ^ (value >> 27)) * 0x94D049BB133111EB) & _U64_MASK
    return value ^ (value >> 31)


def _hash_u16(seed: int, x: int, y: int) -> int:
    mixed = (seed & _U64_MASK) ^ ((x * 0xD6E8FEB86659FD93) & _U64_MASK) ^ ((y * 0xA5A3564E27F8862B) & _U64_MASK)
    return (_splitmix64(mixed) >> 48) & _U16_MAX


def _smooth_q16(value: int) -> int:
    squared = value * value // _Q16
    return squared * (3 * _Q16 - 2 * value) // _Q16


def _lerp_int(start: int, end: int, amount_q16: int) -> int:
    return start + (end - start) * amount_q16 // _Q16


def _value_noise(seed: int, x: int, y: int, cell_size: int) -> int:
    lattice_x = x // cell_size
    lattice_y = y // cell_size
    fraction_x = (x % cell_size) * _Q16 // cell_size
    fraction_y = (y % cell_size) * _Q16 // cell_size
    blend_x = _smooth_q16(fraction_x)
    blend_y = _smooth_q16(fraction_y)
    north = _lerp_int(
        _hash_u16(seed, lattice_x, lattice_y),
        _hash_u16(seed, lattice_x + 1, lattice_y),
        blend_x,
    )
    south = _lerp_int(
        _hash_u16(seed, lattice_x, lattice_y + 1),
        _hash_u16(seed, lattice_x + 1, lattice_y + 1),
        blend_x,
    )
    return _lerp_int(north, south, blend_y)


def _height_samples(spec: TerrainSpec, seed: int) -> tuple[int, ...]:
    if spec.width_px < 2 or spec.height_px < 2:
        raise ValueError("heightfield must contain at least 2x2 samples")
    if spec.size_x_m <= 0 or spec.size_y_m <= 0:
        raise ValueError("terrain extent must be positive")
    if spec.elevation_scale_m <= 0 or spec.base_depth_m <= 0:
        raise ValueError("height scales must be positive")

    center_x = (spec.width_px - 1) // 2
    center_y = (spec.height_px - 1) // 2
    spawn_height = 18_000
    inner_radius_px = 10
    outer_radius_px = 24
    inner_squared = inner_radius_px * inner_radius_px
    outer_squared = outer_radius_px * outer_radius_px
    samples: list[int] = []

    for row in range(spec.height_px):
        for column in range(spec.width_px):
            octave_1 = _value_noise(seed ^ 0x13579BDF, column, row, 64)
            octave_2 = _value_noise(seed ^ 0x2468ACE0, column, row, 32)
            octave_3 = _value_noise(seed ^ 0x10203040, column, row, 16)
            height = (
                18_000
                + (octave_1 - 32_768) * 11_000 // 32_768
                + (octave_2 - 32_768) * 5_500 // 32_768
                + (octave_3 - 32_768) * 2_750 // 32_768
            )

            # The diagonal drainage line makes orientation observable and
            # supplies a non-symmetric navigation feature.
            drainage_center = center_y + (column - center_x) // 5
            drainage_distance = abs(row - drainage_center)
            if drainage_distance < 14:
                height -= (14 - drainage_distance) * 220

            dx = column - center_x
            dy = row - center_y
            radius_squared = dx * dx + dy * dy
            if radius_squared <= inner_squared:
                height = spawn_height
            elif radius_squared < outer_squared:
                blend = (radius_squared - inner_squared) * _Q16 // (outer_squared - inner_squared)
                height = _lerp_int(spawn_height, height, _smooth_q16(blend))

            samples.append(max(1_024, min(58_000, height)))

    minimum = min(samples)
    maximum = max(samples)
    if minimum == maximum:
        raise ValueError("heightfield synthesis produced no elevation range")
    source_range = maximum - minimum
    return tuple((sample - minimum) * _U16_MAX // source_range for sample in samples)


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    checksum = binascii.crc32(kind)
    checksum = binascii.crc32(payload, checksum) & 0xFFFFFFFF
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", checksum)


def _stored_zlib(payload: bytes) -> bytes:
    blocks = bytearray(b"\x78\x01")
    offset = 0
    while offset < len(payload):
        chunk = payload[offset : offset + 65_535]
        offset += len(chunk)
        blocks.append(1 if offset == len(payload) else 0)
        blocks.extend(struct.pack("<H", len(chunk)))
        blocks.extend(struct.pack("<H", 0xFFFF ^ len(chunk)))
        blocks.extend(chunk)
    blocks.extend(struct.pack(">I", zlib.adler32(payload) & 0xFFFFFFFF))
    return bytes(blocks)


def _height_png(spec: TerrainSpec, samples: Sequence[int]) -> bytes:
    rows = bytearray()
    for row in range(spec.height_px):
        rows.append(0)
        start = row * spec.width_px
        for sample in samples[start : start + spec.width_px]:
            rows.extend(struct.pack(">H", sample))
    header = struct.pack(
        ">IIBBBBB",
        spec.width_px,
        spec.height_px,
        16,
        0,
        0,
        0,
        0,
    )
    return (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", header)
        + _png_chunk(b"IDAT", _stored_zlib(bytes(rows)))
        + _png_chunk(b"IEND", b"")
    )


def _mujoco_hfield(spec: TerrainSpec, samples: Sequence[int]) -> bytes:
    payload = bytearray(struct.pack("<ii", spec.height_px, spec.width_px))
    for row in reversed(range(spec.height_px)):
        start = row * spec.width_px
        normalized = (sample / _U16_MAX for sample in samples[start : start + spec.width_px])
        payload.extend(struct.pack(f"<{spec.width_px}f", *normalized))
    return bytes(payload)


def _height_m(
    sample: int,
    spec: TerrainSpec,
    vertical_origin_m: float = 0.0,
) -> float:
    return sample * spec.elevation_scale_m / _U16_MAX + vertical_origin_m


def _terrain_obj(
    spec: TerrainSpec,
    samples: Sequence[int],
    seed: int,
    vertical_origin_m: float,
) -> bytes:
    lines = [
        "# LingTu OpenField_HF deterministic terrain mesh",
        f"# seed={seed} source_grid={spec.width_px}x{spec.height_px}",
        "# coordinates=unreal_lh_z_up_cm; one vertex per heightfield sample",
        "o OpenField_HF_Terrain",
    ]
    for row in range(spec.height_px):
        source_y_m = spec.size_y_m / 2.0 - row * spec.spacing_y_m
        ue_y_cm = -source_y_m * 100.0
        for column in range(spec.width_px):
            source_x_m = -spec.size_x_m / 2.0 + column * spec.spacing_x_m
            sample = samples[row * spec.width_px + column]
            lines.append(
                f"v {source_x_m * 100.0:.6f} {ue_y_cm:.6f} {_height_m(sample, spec, vertical_origin_m) * 100.0:.6f}"
            )

    for row in range(spec.height_px - 1):
        for column in range(spec.width_px - 1):
            north_west = row * spec.width_px + column + 1
            north_east = north_west + 1
            south_west = north_west + spec.width_px
            south_east = south_west + 1
            lines.append(f"f {north_west} {north_east} {south_east}")
            lines.append(f"f {north_west} {south_east} {south_west}")
    return ("\n".join(lines) + "\n").encode("ascii")


def _coordinate_contract(
    spec: TerrainSpec,
    samples: Sequence[int],
) -> dict[str, object]:
    center_column = spec.width_px // 2
    center_row = spec.height_px // 2
    center_sample = samples[center_row * spec.width_px + center_column]
    vertical_origin_m = -_height_m(center_sample, spec)
    return {
        "source_frame": "mujoco_rh_z_up_m",
        "unreal_frame": "unreal_lh_z_up_cm",
        "axis_mapping": ["x", "-y", "z"],
        "origin": "terrain_center",
        "vertical_origin_m": vertical_origin_m,
        "spawn_reference": {
            "sample_pixel": [center_column, center_row],
            "sample_u16": center_sample,
            "world_position_m": [0.0, 0.0, 0.0],
        },
        "extent_m": [spec.size_x_m, spec.size_y_m],
        "grid_px": [spec.width_px, spec.height_px],
        "sample_spacing_m": [spec.spacing_x_m, spec.spacing_y_m],
        "height_encoding": {
            "type": "uint16_normalized",
            "bit_depth": 16,
            "range": [0, _U16_MAX],
            "elevation_scale_m": spec.elevation_scale_m,
            "formula": ("world_height_m = sample_u16 / 65535 * elevation_scale_m + vertical_origin_m"),
        },
        "image_mapping": {
            "column_0_x_m": -spec.size_x_m / 2.0,
            "column_last_x_m": spec.size_x_m / 2.0,
            "row_0_y_m": spec.size_y_m / 2.0,
            "row_last_y_m": -spec.size_y_m / 2.0,
        },
        "mujoco_hfield": {
            "size": [
                spec.size_x_m / 2.0,
                spec.size_y_m / 2.0,
                spec.elevation_scale_m,
                spec.base_depth_m,
            ],
            "geom_position_m": [0.0, 0.0, vertical_origin_m],
        },
        "unreal_landscape": {
            "location_cm": [
                0.0,
                0.0,
                spec.elevation_scale_m * 50.0 + vertical_origin_m * 100.0,
            ],
            "scale_xyz": [
                spec.spacing_x_m * 100.0,
                spec.spacing_y_m * 100.0,
                spec.elevation_scale_m * 100.0 / 512.0,
            ],
        },
    }


def _manifest() -> bytes:
    return (
        b"schema: lingtu.sim.world-package.v1\n"
        b"id: open_field_hf\n"
        b"version: 1.0.0\n"
        b"kind: world\n"
        b"description: Deterministic same-source outdoor terrain foundation for MuJoCo and Unreal.\n"
        b"physics:\n"
        b"  mjcf: ../../../worlds/open_field_hf/open_field_hf.xml\n"
        b"visual:\n"
        b"  binding: WorldVisual:OpenFieldHF\n"
        b"  level: /Game/RobotSim/Maps/OpenFieldRuntime\n"
        b"entities: []\n"
    )


def _mjcf(spec: TerrainSpec, vertical_origin_m: float) -> bytes:
    return (
        "<!-- Generated OpenField_HF world; see package provenance. -->\n"
        '<mujoco model="open_field_hf">\n'
        '  <compiler angle="radian" autolimits="true"/>\n'
        '  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4" solver="Newton" iterations="100"/>\n'
        "  <asset>\n"
        '    <hfield name="open_field_hf_terrain" '
        'file="generated/heightfield_f32.bin" '
        'content_type="image/vnd.mujoco.hfield" '
        f'size="{spec.size_x_m / 2.0:g} {spec.size_y_m / 2.0:g} '
        f'{spec.elevation_scale_m:g} {spec.base_depth_m:g}"/>\n'
        '    <texture name="terrain_grid" type="2d" builtin="checker" '
        'rgb1=".18 .24 .12" rgb2=".28 .20 .10" width="512" height="512"/>\n'
        '    <material name="terrain_material" texture="terrain_grid" '
        'texrepeat="32 32" reflectance="0.02" roughness="0.9"/>\n'
        "  </asset>\n"
        "  <worldbody>\n"
        '    <geom name="terrain" type="hfield" hfield="open_field_hf_terrain" '
        f'pos="0 0 {vertical_origin_m:.9g}" '
        'material="terrain_material" conaffinity="1" condim="3" '
        'friction="0.9 0.02 0.001"/>\n'
        '    <light name="sun" pos="20 -30 50" dir="-.3 .4 -1" '
        'directional="true" diffuse=".9 .85 .75" specular=".1 .1 .1"/>\n'
        "  </worldbody>\n"
        "</mujoco>\n"
    ).encode()


def _output_record(relative_path: Path, payload: bytes) -> dict[str, object]:
    return {
        "path": relative_path.as_posix(),
        "bytes": len(payload),
        "sha256": _sha256(payload),
    }


def _asset_set_digest(records: Sequence[dict[str, object]]) -> str:
    identity = [
        {"path": record["path"], "sha256": record["sha256"]}
        for record in sorted(records, key=lambda item: str(item["path"]))
    ]
    return _sha256(_canonical_json(identity))


def generate_open_field_hf(
    repo_root: Path | str,
    *,
    seed: int = DEFAULT_SEED,
    spec: TerrainSpec = TerrainSpec(),
) -> GeneratedWorld:
    """Generate all canonical OpenField_HF assets below repo_root."""

    if isinstance(seed, bool) or not isinstance(seed, int):
        raise TypeError("seed must be an integer")
    repo_root = Path(repo_root).resolve()
    package_root = repo_root / "sim" / "packages" / "worlds" / "open_field_hf"
    world_root = repo_root / "sim" / "worlds" / "open_field_hf"
    generated_root = world_root / "generated"
    (package_root / "provenance").mkdir(parents=True, exist_ok=True)
    (package_root / "visual").mkdir(parents=True, exist_ok=True)
    generated_root.mkdir(parents=True, exist_ok=True)

    samples = _height_samples(spec, seed)
    coordinate_contract = _coordinate_contract(spec, samples)
    vertical_origin_m = float(coordinate_contract["vertical_origin_m"])
    png_path = Path("sim/worlds/open_field_hf/generated/heightfield_r16.png")
    binary_path = Path("sim/worlds/open_field_hf/generated/heightfield_f32.bin")
    obj_path = Path("sim/worlds/open_field_hf/generated/terrain.obj")
    asset_manifest_path = Path("sim/worlds/open_field_hf/generated/asset-manifest.json")
    package_manifest_path = Path("sim/packages/worlds/open_field_hf/world.package.yaml")
    mjcf_path = Path("sim/worlds/open_field_hf/open_field_hf.xml")
    recipe_path = Path("sim/packages/worlds/open_field_hf/visual/ue_import.recipe.json")
    provenance_path = Path("sim/packages/worlds/open_field_hf/provenance/terrain.provenance.json")

    generated: dict[Path, bytes] = {
        package_manifest_path: _manifest(),
        mjcf_path: _mjcf(spec, vertical_origin_m),
        png_path: _height_png(spec, samples),
        binary_path: _mujoco_hfield(spec, samples),
        obj_path: _terrain_obj(spec, samples, seed, vertical_origin_m),
    }
    png_record = _output_record(png_path, generated[png_path])
    binary_record = _output_record(binary_path, generated[binary_path])
    obj_record = _output_record(obj_path, generated[obj_path])
    source_asset_digest = _asset_set_digest([png_record, binary_record, obj_record])
    asset_manifest = {
        "schema": "lingtu.sim.world-asset-manifest.v1",
        "world_package": "open_field_hf@1.0.0",
        "generator": {
            "module": "sim.tools.worlds.open_field_hf.generate",
            "algorithm": "lingtu_integer_value_noise.v2",
            "seed": seed,
        },
        "coordinate_contract": coordinate_contract,
        "assets": [
            {
                "role": "ue_heightfield",
                "format": "16-bit grayscale PNG",
                **png_record,
            },
            {
                "role": "mujoco_heightfield",
                "format": "image/vnd.mujoco.hfield",
                "row_order": "south_to_north_y_ascending",
                **binary_record,
            },
            {
                "role": "unreal_mesh",
                "sample_grid": [spec.width_px, spec.height_px],
                "vertex_count": spec.width_px * spec.height_px,
                "triangle_count": 2 * (spec.width_px - 1) * (spec.height_px - 1),
                **obj_record,
            },
        ],
        "asset_set_digest": source_asset_digest,
    }
    generated[asset_manifest_path] = _canonical_json(asset_manifest)
    asset_manifest_record = _output_record(asset_manifest_path, generated[asset_manifest_path])

    recipe = {
        "schema": "lingtu.sim.unreal-world-import-recipe.v1",
        "status": "source_assets_only",
        "world_package": "open_field_hf@1.0.0",
        "binding": "WorldVisual:OpenFieldHF",
        "target_level": "/Game/RobotSim/Maps/OpenField_HF",
        "coordinate_contract": coordinate_contract,
        "sources": {
            "asset_manifest": asset_manifest_record,
            "heightfield_png": png_record,
            "mujoco_heightfield": binary_record,
            "terrain_obj": obj_record,
        },
        "unreal_landscape_import": {
            "recommended_source": png_path.as_posix(),
            "dimensions_px": [spec.width_px, spec.height_px],
            "format": "16-bit grayscale PNG",
            "location_cm": coordinate_contract["unreal_landscape"]["location_cm"],
            "scale_xyz": coordinate_contract["unreal_landscape"]["scale_xyz"],
            "material_requirement": ("Create project-owned grass, mud, and rock PBR layers."),
        },
        "blender_mesh_import": {
            "source": obj_path.as_posix(),
            "coordinates": "unreal_lh_z_up_cm",
            "import_scale": 1.0,
            "vertex_per_height_sample": True,
        },
        "limitations": [
            ("No Unreal UMAP, Landscape material, foliage, lighting bake, Cook, or packaged build is generated."),
            (
                "This recipe establishes import inputs and scale contracts; "
                "it is not evidence of a finished high-fidelity UE world."
            ),
        ],
    }
    generated[recipe_path] = _canonical_json(recipe)

    records = [
        _output_record(path, payload)
        for path, payload in sorted(generated.items(), key=lambda item: item[0].as_posix())
    ]
    asset_digest = _asset_set_digest(records)
    provenance = {
        "schema": "lingtu.sim.generated-terrain-provenance.v1",
        "asset_id": "open_field_hf.terrain",
        "version": "1.0.0",
        "generator": {
            "module": "sim.tools.worlds.open_field_hf.generate",
            "algorithm": "lingtu_integer_value_noise.v2",
            "seed": seed,
        },
        "source": {
            "type": "procedural",
            "owner": "LingTu project",
            "license": "LicenseRef-LingTu-Project-Owned",
            "third_party_assets": [],
        },
        "coordinate_contract": coordinate_contract,
        "sample_statistics": {
            "minimum_u16": min(samples),
            "maximum_u16": max(samples),
            "minimum_height_m": _height_m(min(samples), spec, vertical_origin_m),
            "maximum_height_m": _height_m(max(samples), spec, vertical_origin_m),
        },
        "outputs": records,
        "asset_set_digest": asset_digest,
        "reproducibility": {
            "deterministic": True,
            "standard_library_only": True,
            "command": (f"python -m sim.tools.worlds.open_field_hf.generate --repo-root . --seed {seed}"),
        },
    }
    generated[provenance_path] = _canonical_json(provenance)

    for relative_path, payload in generated.items():
        target = repo_root / relative_path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(payload)

    return GeneratedWorld(
        repo_root=repo_root,
        package_root=package_root,
        world_root=world_root,
        asset_set_digest=asset_digest,
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate deterministic OpenField_HF source terrain assets.")
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Generate assets and print their deterministic identity."""

    args = _build_parser().parse_args(argv)
    generated = generate_open_field_hf(args.repo_root, seed=args.seed)
    print(
        json.dumps(
            {
                "asset_set_digest": generated.asset_set_digest,
                "package_root": str(generated.package_root),
                "world_root": str(generated.world_root),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

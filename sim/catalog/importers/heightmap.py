"""Deterministic heightmap artifact builders for imported simulation worlds."""

from __future__ import annotations

import math
import os
import struct
import tempfile
import zlib
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any, BinaryIO, Iterator, Sequence, TextIO

from .contracts import ImportCode, ImportFailure

_PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
_MUJOCO_FLAT_HFIELD_ELEVATION_SCALE_M = 1.0e-6


@dataclass(frozen=True)
class HeightmapArtifacts:
    """Generated same-source physics and visual terrain artifacts."""

    width: int
    height: int
    extent_m: tuple[float, float]
    elevation_min_m: float
    elevation_max_m: float
    sample_min_u16: int
    sample_max_u16: int
    sampled_elevation_min_m: float
    sampled_elevation_max_m: float
    sampled_elevation_range_m: float
    mujoco_elevation_scale_m: float
    sample_spacing_m: tuple[float, float]
    spawn_height_m: float
    assets: tuple[dict[str, Any], ...]
    physics_bounds_m: dict[str, list[float]]
    visual_bounds_m: dict[str, list[float]]
    alignment: dict[str, Any]


def read_u16_height_grid(path: Path, *, width: int, height: int, endian: str = "little") -> tuple[int, ...]:
    """Read an exact uint16 height grid without normalization side effects."""

    if width < 2 or height < 2:
        raise ImportFailure("heightmap grid must be at least 2x2", code=ImportCode.MODEL_INVALID, context="heightmap")
    if endian not in {"little", "big"}:
        raise ImportFailure("heightmap endian must be little or big", context="heightmap.endian")
    payload = Path(path).read_bytes()
    expected = width * height * 2
    if len(payload) != expected:
        raise ImportFailure(
            "heightmap u16 payload size does not match width*height*2",
            code=ImportCode.MODEL_INVALID,
            context=str(path),
            details={"expected_bytes": expected, "actual_bytes": len(payload)},
        )
    prefix = "<" if endian == "little" else ">"
    return struct.unpack(f"{prefix}{width * height}H", payload)


def build_heightmap_artifacts(
    *,
    samples: Sequence[int],
    width: int,
    height: int,
    extent_m: Sequence[float],
    elevation_min_m: float,
    elevation_max_m: float,
    spawn_xy_m: Sequence[float],
    artifact_root: Path,
    mesh_name: str = "terrain",
    emit_unreal_obj: bool = True,
) -> HeightmapArtifacts:
    """Build MuJoCo hfield, Unreal PNG/OBJ, and an alignment report from one u16 grid."""

    if len(samples) != width * height:
        raise ImportFailure("heightmap sample count does not match grid dimensions", code=ImportCode.MODEL_INVALID)
    if any(isinstance(value, bool) or not isinstance(value, int) or value < 0 or value > 65_535 for value in samples):
        raise ImportFailure("heightmap samples must be uint16 values", code=ImportCode.MODEL_INVALID)
    extent_x, extent_y = _positive_pair(extent_m, "heightmap.extent_m")
    z_min = _finite_float(elevation_min_m, "heightmap.elevation_min_m")
    z_max = _finite_float(elevation_max_m, "heightmap.elevation_max_m")
    if z_max <= z_min:
        raise ImportFailure(
            "heightmap elevation_max_m must be greater than elevation_min_m", code=ImportCode.MODEL_INVALID
        )
    spawn_x, spawn_y = _finite_pair(spawn_xy_m, "spawn.position_m")
    half_x = extent_x / 2.0
    half_y = extent_y / 2.0
    if not (-half_x <= spawn_x <= half_x and -half_y <= spawn_y <= half_y):
        raise ImportFailure(
            "spawn position is outside the terrain bounds",
            code=ImportCode.WORLD_ALIGNMENT_INVALID,
            context="spawn.position_m",
            details={"spawn_xy_m": [spawn_x, spawn_y], "bounds_xy_m": [[-half_x, -half_y], [half_x, half_y]]},
        )

    artifact_root = Path(artifact_root)
    artifact_root.mkdir(parents=True, exist_ok=True)
    f32_path = artifact_root / "heightfield_f32.bin"
    png_path = artifact_root / "heightfield_r16.png"
    obj_path = artifact_root / "terrain.obj"
    spacing_x = extent_x / float(width - 1)
    spacing_y = extent_y / float(height - 1)

    sample_min_u16, sample_max_u16 = _write_mujoco_hfield(
        f32_path,
        samples=samples,
        width=width,
        height=height,
    )
    _write_png_u16(png_path, samples=samples, width=width, height=height)
    if emit_unreal_obj:
        _write_unreal_obj(
            obj_path,
            samples=samples,
            width=width,
            height=height,
            extent_m=(extent_x, extent_y),
            elevation_min_m=z_min,
            elevation_max_m=z_max,
            mesh_name=mesh_name,
        )
    elif obj_path.exists():
        obj_path.unlink()
    spawn_z = _sample_height(
        samples,
        width=width,
        height=height,
        extent_m=(extent_x, extent_y),
        elevation_min_m=z_min,
        elevation_max_m=z_max,
        x_m=spawn_x,
        y_m=spawn_y,
    )
    sampled_elevation_min_m, sampled_elevation_max_m, sampled_elevation_range_m = _sampled_elevation_range(
        sample_min_u16,
        sample_max_u16,
        z_min,
        z_max,
    )
    mujoco_elevation_scale_m = max(sampled_elevation_range_m, _MUJOCO_FLAT_HFIELD_ELEVATION_SCALE_M)
    bounds = {"min_m": [-half_x, -half_y, z_min], "max_m": [half_x, half_y, z_max]}
    artifact_paths = [
        ("mujoco_hfield", f32_path),
        ("unreal_height_png", png_path),
    ]
    if emit_unreal_obj:
        artifact_paths.append(("unreal_mesh_obj", obj_path))
    assets = tuple(
        {
            "role": role,
            "path": path.relative_to(artifact_root.parent).as_posix(),
            "bytes": path.stat().st_size,
        }
        for role, path in artifact_paths
    )
    alignment = {
        "schema": "lingtu.sim.world-heightmap-alignment.v1",
        "units": {"length": "m", "up_axis": "Z", "handedness": "RH"},
        "grid_px": [width, height],
        "extent_m": [extent_x, extent_y],
        "sample_spacing_m": [spacing_x, spacing_y],
        "height_encoding": {
            "input": "uint16",
            "formula": "height_m = sample_u16 / 65535 * (elevation_max_m - elevation_min_m) + elevation_min_m",
            "elevation_min_m": z_min,
            "elevation_max_m": z_max,
        },
        "physics": {
            "format": "mujoco_hfield_f32",
            "bounds_m": bounds,
            "normalization": _mujoco_normalization_contract(
                sample_min_u16=sample_min_u16,
                sample_max_u16=sample_max_u16,
                sampled_elevation_min_m=sampled_elevation_min_m,
                sampled_elevation_max_m=sampled_elevation_max_m,
                sampled_elevation_range_m=sampled_elevation_range_m,
                mujoco_elevation_scale_m=mujoco_elevation_scale_m,
            ),
        },
        "visual": {
            "format": "ue_16bit_png_plus_obj_cm" if emit_unreal_obj else "ue_16bit_png",
            "bounds_m": bounds,
        },
        "spawn": {"position_m": [spawn_x, spawn_y, spawn_z], "aligned_to_heightmap": True},
    }
    return HeightmapArtifacts(
        width=width,
        height=height,
        extent_m=(extent_x, extent_y),
        elevation_min_m=z_min,
        elevation_max_m=z_max,
        sample_min_u16=sample_min_u16,
        sample_max_u16=sample_max_u16,
        sampled_elevation_min_m=sampled_elevation_min_m,
        sampled_elevation_max_m=sampled_elevation_max_m,
        sampled_elevation_range_m=sampled_elevation_range_m,
        mujoco_elevation_scale_m=mujoco_elevation_scale_m,
        sample_spacing_m=(spacing_x, spacing_y),
        spawn_height_m=spawn_z,
        assets=assets,
        physics_bounds_m=bounds,
        visual_bounds_m=bounds,
        alignment=alignment,
    )


def build_heightmap_artifacts_from_u16_file(
    *,
    source_path: Path,
    width: int,
    height: int,
    extent_m: Sequence[float],
    elevation_min_m: float,
    elevation_max_m: float,
    spawn_xy_m: Sequence[float],
    artifact_root: Path,
    endian: str = "little",
    mesh_name: str = "terrain",
    emit_unreal_obj: bool = True,
) -> HeightmapArtifacts:
    """Stream artifacts from an exact raw uint16 grid without materializing it in memory."""

    source_path = Path(source_path)
    _validate_u16_source(source_path, width=width, height=height, endian=endian)
    extent_x, extent_y = _positive_pair(extent_m, "heightmap.extent_m")
    z_min = _finite_float(elevation_min_m, "heightmap.elevation_min_m")
    z_max = _finite_float(elevation_max_m, "heightmap.elevation_max_m")
    if z_max <= z_min:
        raise ImportFailure(
            "heightmap elevation_max_m must be greater than elevation_min_m", code=ImportCode.MODEL_INVALID
        )
    spawn_x, spawn_y = _finite_pair(spawn_xy_m, "spawn.position_m")
    half_x = extent_x / 2.0
    half_y = extent_y / 2.0
    if not (-half_x <= spawn_x <= half_x and -half_y <= spawn_y <= half_y):
        raise ImportFailure(
            "spawn position is outside the terrain bounds",
            code=ImportCode.WORLD_ALIGNMENT_INVALID,
            context="spawn.position_m",
            details={"spawn_xy_m": [spawn_x, spawn_y], "bounds_xy_m": [[-half_x, -half_y], [half_x, half_y]]},
        )

    artifact_root = Path(artifact_root)
    artifact_root.mkdir(parents=True, exist_ok=True)
    f32_path = artifact_root / "heightfield_f32.bin"
    png_path = artifact_root / "heightfield_r16.png"
    obj_path = artifact_root / "terrain.obj"
    sample_min_u16, sample_max_u16 = _stream_mujoco_hfield(
        f32_path,
        source_path=source_path,
        width=width,
        height=height,
        endian=endian,
    )
    _stream_png_u16(png_path, source_path=source_path, width=width, height=height, endian=endian)
    if emit_unreal_obj:
        _stream_unreal_obj(
            obj_path,
            source_path=source_path,
            width=width,
            height=height,
            endian=endian,
            extent_m=(extent_x, extent_y),
            elevation_min_m=z_min,
            elevation_max_m=z_max,
            mesh_name=mesh_name,
        )
    elif obj_path.exists():
        obj_path.unlink()

    spawn_z = _sample_height_from_u16_file(
        source_path,
        width=width,
        height=height,
        endian=endian,
        extent_m=(extent_x, extent_y),
        elevation_min_m=z_min,
        elevation_max_m=z_max,
        x_m=spawn_x,
        y_m=spawn_y,
    )
    sampled_elevation_min_m, sampled_elevation_max_m, sampled_elevation_range_m = _sampled_elevation_range(
        sample_min_u16,
        sample_max_u16,
        z_min,
        z_max,
    )
    mujoco_elevation_scale_m = max(sampled_elevation_range_m, _MUJOCO_FLAT_HFIELD_ELEVATION_SCALE_M)
    spacing_x = extent_x / float(width - 1)
    spacing_y = extent_y / float(height - 1)
    bounds = {"min_m": [-half_x, -half_y, z_min], "max_m": [half_x, half_y, z_max]}
    artifact_paths = [
        ("mujoco_hfield", f32_path),
        ("unreal_height_png", png_path),
    ]
    if emit_unreal_obj:
        artifact_paths.append(("unreal_mesh_obj", obj_path))
    assets = tuple(
        {
            "role": role,
            "path": path.relative_to(artifact_root.parent).as_posix(),
            "bytes": path.stat().st_size,
        }
        for role, path in artifact_paths
    )
    alignment = {
        "schema": "lingtu.sim.world-heightmap-alignment.v1",
        "units": {"length": "m", "up_axis": "Z", "handedness": "RH"},
        "grid_px": [width, height],
        "extent_m": [extent_x, extent_y],
        "sample_spacing_m": [spacing_x, spacing_y],
        "height_encoding": {
            "input": "uint16",
            "formula": "height_m = sample_u16 / 65535 * (elevation_max_m - elevation_min_m) + elevation_min_m",
            "elevation_min_m": z_min,
            "elevation_max_m": z_max,
        },
        "physics": {
            "format": "mujoco_hfield_f32",
            "bounds_m": bounds,
            "normalization": _mujoco_normalization_contract(
                sample_min_u16=sample_min_u16,
                sample_max_u16=sample_max_u16,
                sampled_elevation_min_m=sampled_elevation_min_m,
                sampled_elevation_max_m=sampled_elevation_max_m,
                sampled_elevation_range_m=sampled_elevation_range_m,
                mujoco_elevation_scale_m=mujoco_elevation_scale_m,
            ),
        },
        "visual": {
            "format": "ue_16bit_png_plus_obj_cm" if emit_unreal_obj else "ue_16bit_png",
            "bounds_m": bounds,
        },
        "spawn": {"position_m": [spawn_x, spawn_y, spawn_z], "aligned_to_heightmap": True},
    }
    return HeightmapArtifacts(
        width=width,
        height=height,
        extent_m=(extent_x, extent_y),
        elevation_min_m=z_min,
        elevation_max_m=z_max,
        sample_min_u16=sample_min_u16,
        sample_max_u16=sample_max_u16,
        sampled_elevation_min_m=sampled_elevation_min_m,
        sampled_elevation_max_m=sampled_elevation_max_m,
        sampled_elevation_range_m=sampled_elevation_range_m,
        mujoco_elevation_scale_m=mujoco_elevation_scale_m,
        sample_spacing_m=(spacing_x, spacing_y),
        spawn_height_m=spawn_z,
        assets=assets,
        physics_bounds_m=bounds,
        visual_bounds_m=bounds,
        alignment=alignment,
    )


def _validate_u16_source(path: Path, *, width: int, height: int, endian: str) -> None:
    if width < 2 or height < 2:
        raise ImportFailure("heightmap grid must be at least 2x2", code=ImportCode.MODEL_INVALID, context="heightmap")
    if endian not in {"little", "big"}:
        raise ImportFailure("heightmap endian must be little or big", context="heightmap.endian")
    expected = width * height * 2
    try:
        actual = path.stat().st_size
    except OSError as exc:
        raise ImportFailure("heightmap u16 source is not readable", context=str(path)) from exc
    if actual != expected:
        raise ImportFailure(
            "heightmap u16 payload size does not match width*height*2",
            code=ImportCode.MODEL_INVALID,
            context=str(path),
            details={"expected_bytes": expected, "actual_bytes": actual},
        )


@contextmanager
def _atomic_binary_target(path: Path) -> Iterator[BinaryIO]:
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            yield stream
        temporary.replace(path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


@contextmanager
def _atomic_text_target(path: Path) -> Iterator[TextIO]:
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="ascii") as stream:
            yield stream
        temporary.replace(path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def _unpack_u16_row(payload: bytes, *, width: int, endian: str) -> tuple[int, ...]:
    prefix = "<" if endian == "little" else ">"
    return struct.unpack(f"{prefix}{width}H", payload)


def _stream_mujoco_hfield(
    path: Path,
    *,
    source_path: Path,
    width: int,
    height: int,
    endian: str,
) -> tuple[int, int]:
    row_bytes = width * 2
    sample_min_u16 = 65_535
    sample_max_u16 = 0
    with source_path.open("rb") as source, _atomic_binary_target(path) as target:
        target.write(struct.pack("<ii", height, width))
        for row in reversed(range(height)):
            source.seek(row * row_bytes)
            samples = _unpack_u16_row(source.read(row_bytes), width=width, endian=endian)
            sample_min_u16 = min(sample_min_u16, min(samples))
            sample_max_u16 = max(sample_max_u16, max(samples))
            target.write(struct.pack(f"<{width}f", *(sample / 65_535.0 for sample in samples)))
    return sample_min_u16, sample_max_u16


def _stream_png_u16(path: Path, *, source_path: Path, width: int, height: int, endian: str) -> None:
    descriptor, compressed_name = tempfile.mkstemp(prefix=f".{path.name}.idat.", suffix=".tmp", dir=path.parent)
    compressed_path = Path(compressed_name)
    compressor = zlib.compressobj(level=9)
    try:
        with os.fdopen(descriptor, "wb") as compressed, source_path.open("rb") as source:
            for _row in range(height):
                samples = _unpack_u16_row(source.read(width * 2), width=width, endian=endian)
                encoded = bytearray((0,))
                for sample in samples:
                    encoded.extend(struct.pack(">H", sample))
                compressed.write(compressor.compress(bytes(encoded)))
            compressed.write(compressor.flush())
        ihdr = struct.pack(">IIBBBBB", width, height, 16, 0, 0, 0, 0)
        with _atomic_binary_target(path) as target:
            target.write(_PNG_SIGNATURE)
            target.write(_png_chunk(b"IHDR", ihdr))
            target.write(struct.pack(">I", compressed_path.stat().st_size))
            target.write(b"IDAT")
            crc = zlib.crc32(b"IDAT")
            with compressed_path.open("rb") as compressed:
                while payload := compressed.read(1024 * 1024):
                    target.write(payload)
                    crc = zlib.crc32(payload, crc)
            target.write(struct.pack(">I", crc & 0xFFFFFFFF))
            target.write(_png_chunk(b"IEND", b""))
    finally:
        compressed_path.unlink(missing_ok=True)


def _stream_unreal_obj(
    path: Path,
    *,
    source_path: Path,
    width: int,
    height: int,
    endian: str,
    extent_m: tuple[float, float],
    elevation_min_m: float,
    elevation_max_m: float,
    mesh_name: str,
) -> None:
    spacing_x = extent_m[0] / float(width - 1)
    spacing_y = extent_m[1] / float(height - 1)
    with source_path.open("rb") as source, _atomic_text_target(path) as target:
        target.write(f"o {mesh_name}\n")
        for row in range(height):
            y_m = extent_m[1] / 2.0 - row * spacing_y
            samples = _unpack_u16_row(source.read(width * 2), width=width, endian=endian)
            for column, sample in enumerate(samples):
                x_m = -extent_m[0] / 2.0 + column * spacing_x
                z_m = _height_from_sample(sample, elevation_min_m, elevation_max_m)
                target.write(f"v {x_m * 100.0:.9f} {-y_m * 100.0:.9f} {z_m * 100.0:.9f}\n")
        for row in range(height):
            v = 1.0 - row / float(height - 1)
            for column in range(width):
                u = column / float(width - 1)
                target.write(f"vt {u:.9f} {v:.9f}\n")
        for row in range(height - 1):
            for column in range(width - 1):
                v00 = row * width + column + 1
                v10 = row * width + column + 2
                v01 = (row + 1) * width + column + 1
                v11 = (row + 1) * width + column + 2
                target.write(f"f {v00}/{v00} {v01}/{v01} {v10}/{v10}\n")
                target.write(f"f {v10}/{v10} {v01}/{v01} {v11}/{v11}\n")


def _sample_height_from_u16_file(
    source_path: Path,
    *,
    width: int,
    height: int,
    endian: str,
    extent_m: tuple[float, float],
    elevation_min_m: float,
    elevation_max_m: float,
    x_m: float,
    y_m: float,
) -> float:
    column = round((x_m + extent_m[0] / 2.0) / (extent_m[0] / float(width - 1)))
    row = round((extent_m[1] / 2.0 - y_m) / (extent_m[1] / float(height - 1)))
    column = max(0, min(width - 1, int(column)))
    row = max(0, min(height - 1, int(row)))
    prefix = "<" if endian == "little" else ">"
    with source_path.open("rb") as source:
        source.seek((row * width + column) * 2)
        sample = struct.unpack(f"{prefix}H", source.read(2))[0]
    return _height_from_sample(sample, elevation_min_m, elevation_max_m)


def _finite_float(value: Any, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ImportFailure(f"{context} must be finite numeric data", context=context)
    return float(value)


def _finite_pair(value: Sequence[float], context: str) -> tuple[float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) < 2:
        raise ImportFailure(f"{context} must contain at least two numeric values", context=context)
    return (_finite_float(value[0], f"{context}[0]"), _finite_float(value[1], f"{context}[1]"))


def _positive_pair(value: Sequence[float], context: str) -> tuple[float, float]:
    x, y = _finite_pair(value, context)
    if x <= 0.0 or y <= 0.0:
        raise ImportFailure(f"{context} must contain positive values", context=context)
    return x, y


def _height_from_sample(sample: int, z_min: float, z_max: float) -> float:
    return float(sample) / 65_535.0 * (z_max - z_min) + z_min


def _sampled_elevation_range(
    sample_min_u16: int,
    sample_max_u16: int,
    elevation_min_m: float,
    elevation_max_m: float,
) -> tuple[float, float, float]:
    sampled_min_m = _height_from_sample(sample_min_u16, elevation_min_m, elevation_max_m)
    sampled_max_m = _height_from_sample(sample_max_u16, elevation_min_m, elevation_max_m)
    return sampled_min_m, sampled_max_m, sampled_max_m - sampled_min_m


def _mujoco_normalization_contract(
    *,
    sample_min_u16: int,
    sample_max_u16: int,
    sampled_elevation_min_m: float,
    sampled_elevation_max_m: float,
    sampled_elevation_range_m: float,
    mujoco_elevation_scale_m: float,
) -> dict[str, Any]:
    flat_field_epsilon_m = (
        _MUJOCO_FLAT_HFIELD_ELEVATION_SCALE_M if sample_min_u16 == sample_max_u16 else 0.0
    )
    return {
        "compiler_behavior": "affine_sample_min_max_to_unit_interval",
        "sample_min_u16": sample_min_u16,
        "sample_max_u16": sample_max_u16,
        "sampled_elevation_min_m": sampled_elevation_min_m,
        "sampled_elevation_max_m": sampled_elevation_max_m,
        "sampled_elevation_range_m": sampled_elevation_range_m,
        "geom_origin_z_m": sampled_elevation_min_m,
        "hfield_elevation_scale_m": mujoco_elevation_scale_m,
        "flat_field_epsilon_m": flat_field_epsilon_m,
    }


def _sample_height(
    samples: Sequence[int],
    *,
    width: int,
    height: int,
    extent_m: tuple[float, float],
    elevation_min_m: float,
    elevation_max_m: float,
    x_m: float,
    y_m: float,
) -> float:
    column = round((x_m + extent_m[0] / 2.0) / (extent_m[0] / float(width - 1)))
    row = round((extent_m[1] / 2.0 - y_m) / (extent_m[1] / float(height - 1)))
    column = max(0, min(width - 1, int(column)))
    row = max(0, min(height - 1, int(row)))
    return _height_from_sample(samples[row * width + column], elevation_min_m, elevation_max_m)


def _write_mujoco_hfield(
    path: Path,
    *,
    samples: Sequence[int],
    width: int,
    height: int,
) -> tuple[int, int]:
    values = [samples[row * width + column] / 65_535.0 for row in reversed(range(height)) for column in range(width)]
    path.write_bytes(struct.pack(f"<ii{len(values)}f", height, width, *values))
    return min(samples), max(samples)


def _png_chunk(kind: bytes, data: bytes) -> bytes:
    return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)


def _write_png_u16(path: Path, *, samples: Sequence[int], width: int, height: int) -> None:
    raw = bytearray()
    for row in range(height):
        raw.append(0)
        for column in range(width):
            raw.extend(struct.pack(">H", samples[row * width + column]))
    ihdr = struct.pack(">IIBBBBB", width, height, 16, 0, 0, 0, 0)
    path.write_bytes(
        b"".join(
            (
                _PNG_SIGNATURE,
                _png_chunk(b"IHDR", ihdr),
                _png_chunk(b"IDAT", zlib.compress(bytes(raw), level=9)),
                _png_chunk(b"IEND", b""),
            )
        )
    )


def _write_unreal_obj(
    path: Path,
    *,
    samples: Sequence[int],
    width: int,
    height: int,
    extent_m: tuple[float, float],
    elevation_min_m: float,
    elevation_max_m: float,
    mesh_name: str,
) -> None:
    spacing_x = extent_m[0] / float(width - 1)
    spacing_y = extent_m[1] / float(height - 1)
    lines = [f"o {mesh_name}"]
    for row in range(height):
        y_m = extent_m[1] / 2.0 - row * spacing_y
        for column in range(width):
            x_m = -extent_m[0] / 2.0 + column * spacing_x
            z_m = _height_from_sample(samples[row * width + column], elevation_min_m, elevation_max_m)
            lines.append(f"v {x_m * 100.0:.9f} {-y_m * 100.0:.9f} {z_m * 100.0:.9f}")
    for row in range(height):
        v = 1.0 - row / float(height - 1)
        for column in range(width):
            u = column / float(width - 1)
            lines.append(f"vt {u:.9f} {v:.9f}")
    for row in range(height - 1):
        for column in range(width - 1):
            v00 = row * width + column + 1
            v10 = row * width + column + 2
            v01 = (row + 1) * width + column + 1
            v11 = (row + 1) * width + column + 2
            lines.append(f"f {v00}/{v00} {v01}/{v01} {v10}/{v10}")
            lines.append(f"f {v10}/{v10} {v01}/{v01} {v11}/{v11}")
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


__all__ = [
    "HeightmapArtifacts",
    "build_heightmap_artifacts",
    "build_heightmap_artifacts_from_u16_file",
    "read_u16_height_grid",
]

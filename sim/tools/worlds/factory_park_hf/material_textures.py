"""Author deterministic, dependency-free surface detail textures."""

from __future__ import annotations

import math
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path

_ASSET_STEMS = {
    "asphalt_worn": {
        "detail": "T_FP_AsphaltWorn_Detail",
        "normal": "T_FP_AsphaltWorn_Normal",
    },
    "concrete_weathered": {
        "detail": "T_FP_ConcreteWeathered_Detail",
        "normal": "T_FP_ConcreteWeathered_Normal",
    },
}


@dataclass(frozen=True)
class SurfaceTextureSet:
    """In-memory RGB texture pair and its intended world-space repeat size."""

    detail_rgb: bytes
    normal_rgb: bytes
    size: int
    tile_size_cm: float


def _periodic_height(profile: str, x: int, y: int, size: int) -> float:
    u = 2.0 * math.pi * x / (size - 1)
    v = 2.0 * math.pi * y / (size - 1)
    if profile == "asphalt_worn":
        return (
            0.48 * math.sin(7.0 * u + 3.0 * v)
            + 0.31 * math.cos(13.0 * u - 9.0 * v)
            + 0.21 * math.sin(23.0 * u + 19.0 * v)
        )
    return (
        0.55 * math.sin(2.0 * u)
        + 0.27 * math.cos(5.0 * v + u)
        + 0.18 * math.sin(11.0 * u - 7.0 * v)
    )


def _encode_channel(value: float) -> int:
    return max(0, min(255, int(round(value))))


def _make_texture_set(profile: str, size: int, tile_size_cm: float) -> SurfaceTextureSet:
    heights = [
        [_periodic_height(profile, x, y, size) for x in range(size)]
        for y in range(size)
    ]
    detail = bytearray()
    normal = bytearray()
    strength = 1.7 if profile == "asphalt_worn" else 1.1
    roughness_center = 209.0 if profile == "asphalt_worn" else 201.0
    roughness_variation = 18.0 if profile == "asphalt_worn" else 16.0
    for y in range(size):
        for x in range(size):
            shade = _encode_channel(roughness_center + heights[y][x] * roughness_variation)
            detail.extend((shade, shade, shade))
            left = heights[y][(x - 1) % (size - 1)]
            right = heights[y][(x + 1) % (size - 1)]
            down = heights[(y - 1) % (size - 1)][x]
            up = heights[(y + 1) % (size - 1)][x]
            nx = -(right - left) * strength
            ny = -(up - down) * strength
            nz = 2.0
            length = math.sqrt(nx * nx + ny * ny + nz * nz)
            normal.extend(
                (
                    _encode_channel(127.5 * (nx / length + 1.0)),
                    _encode_channel(127.5 * (ny / length + 1.0)),
                    _encode_channel(127.5 * (nz / length + 1.0)),
                )
            )
    return SurfaceTextureSet(bytes(detail), bytes(normal), size, tile_size_cm)


def generate_surface_texture_set(size: int = 256) -> dict[str, SurfaceTextureSet]:
    """Generate the two bounded texture pairs consumed by the UE material builder."""

    if size < 8:
        raise ValueError("texture size must be at least 8 pixels")
    return {
        "asphalt_worn": _make_texture_set("asphalt_worn", size, 200.0),
        "concrete_weathered": _make_texture_set("concrete_weathered", size, 160.0),
    }


def _png_chunk(chunk_type: bytes, payload: bytes) -> bytes:
    checksum = zlib.crc32(chunk_type)
    checksum = zlib.crc32(payload, checksum) & 0xFFFFFFFF
    return struct.pack(">I", len(payload)) + chunk_type + payload + struct.pack(">I", checksum)


def _encode_rgb_png(rgb: bytes, size: int) -> bytes:
    stride = 3 * size
    scanlines = b"".join(
        b"\x00" + rgb[offset : offset + stride]
        for offset in range(0, len(rgb), stride)
    )
    return b"".join(
        (
            b"\x89PNG\r\n\x1a\n",
            _png_chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 2, 0, 0, 0)),
            _png_chunk(b"IDAT", zlib.compress(scanlines, level=9)),
            _png_chunk(b"IEND", b""),
        )
    )


def author_surface_textures(
    output_dir: Path,
    size: int = 256,
) -> dict[str, dict[str, Path]]:
    """Write deterministic RGB PNGs and return the files consumed by Unreal."""

    output_dir.mkdir(parents=True, exist_ok=True)
    authored: dict[str, dict[str, Path]] = {}
    for profile, texture_set in generate_surface_texture_set(size).items():
        profile_files: dict[str, Path] = {}
        for channel, rgb in (
            ("detail", texture_set.detail_rgb),
            ("normal", texture_set.normal_rgb),
        ):
            path = output_dir / f"{_ASSET_STEMS[profile][channel]}.png"
            path.write_bytes(_encode_rgb_png(rgb, texture_set.size))
            profile_files[channel] = path
        authored[profile] = profile_files
    return authored

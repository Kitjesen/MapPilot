"""Binary wire codecs for high-rate gateway streams.

The default JSON-over-SSE path is ~1.4 MB / 60k-point cloud and forces the
browser to allocate one PyFloat-equivalent per coordinate.  Quantized-int16
packing collapses a frame to ~360 KB on the wire (≤180 KB after the WS
permessage-deflate filter) and decodes in the browser as a single
zero-copy Int16Array view.

Frame format (little-endian)::

    offset  size  field
    ------  ----  --------------------------------
    0       4     magic         b"PCLD"
    4       1     version       u8 = 1
    5       1     flags         u8  bit0=has_color
    6       2     reserved      u16 = 0
    8       4     count         u32  point count
    12      4     scale         f32  metres per int16 unit
    16      4     origin_x      f32  metres
    20      4     origin_y      f32
    24      4     origin_z      f32
    28      —     positions     int16[count*3]  (xyz interleaved)
    …       —     colors        u8[count*3]     (rgb, only if flags bit0)
"""

from __future__ import annotations

import struct
from typing import Tuple

import numpy as np

MAGIC = b"PCLD"
VERSION = 1
HEADER_SIZE = 28
HEADER_FMT = "<4sBBHIffff"  # magic, version, flags, _, count, scale, ox, oy, oz

FLAG_HAS_COLOR = 0x01


def encode_pointcloud(
    pts: np.ndarray,
    *,
    scale: float = 0.005,
    colors: np.ndarray | None = None,
) -> bytes:
    """Pack an (N,3) float array into the binary frame above.

    ``scale`` is the wire resolution in metres (5 mm by default — well below
    LiDAR noise).  ``origin`` is auto-computed as ``pts.min(axis=0)`` so the
    int16 range covers ±32767 * scale ≈ ±163 m around it.

    All work stays in numpy / struct, no per-point Python objects.
    """
    if pts.size == 0:
        return _empty_header()

    pts = np.ascontiguousarray(pts[:, :3], dtype=np.float32)
    origin = pts.min(axis=0)
    quant = np.rint((pts - origin) / scale).clip(-32768, 32767).astype(np.int16)

    flags = 0
    payload = quant.tobytes()
    if colors is not None and len(colors) == len(pts):
        flags |= FLAG_HAS_COLOR
        rgb = np.ascontiguousarray(colors[:, :3], dtype=np.uint8)
        payload += rgb.tobytes()

    header = struct.pack(
        HEADER_FMT,
        MAGIC, VERSION, flags, 0,
        len(pts), float(scale),
        float(origin[0]), float(origin[1]), float(origin[2]),
    )
    return header + payload


def decode_pointcloud(buf: bytes) -> Tuple[np.ndarray, np.ndarray | None]:
    """Inverse of :func:`encode_pointcloud` — used by tests, not in hot path."""
    if len(buf) < HEADER_SIZE:
        raise ValueError("buffer too small for header")
    magic, version, flags, _, count, scale, ox, oy, oz = struct.unpack_from(
        HEADER_FMT, buf, 0,
    )
    if magic != MAGIC:
        raise ValueError(f"bad magic: {magic!r}")
    if version != VERSION:
        raise ValueError(f"unsupported version: {version}")

    pos_bytes = count * 3 * 2
    pos = np.frombuffer(buf, dtype=np.int16, count=count * 3, offset=HEADER_SIZE)
    xyz = pos.reshape(-1, 3).astype(np.float32) * scale + np.array(
        [ox, oy, oz], dtype=np.float32,
    )

    colors: np.ndarray | None = None
    if flags & FLAG_HAS_COLOR:
        col_off = HEADER_SIZE + pos_bytes
        colors = np.frombuffer(
            buf, dtype=np.uint8, count=count * 3, offset=col_off,
        ).reshape(-1, 3).copy()
    return xyz, colors


def _empty_header() -> bytes:
    return struct.pack(HEADER_FMT, MAGIC, VERSION, 0, 0, 0, 1.0, 0.0, 0.0, 0.0)

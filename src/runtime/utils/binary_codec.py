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

Version 2 uses the u16 at offset 6 as a variable header size and adds scene
epoch, sequence, timestamp, stream kind, and UTF-8 frame id before the aligned
point payload. Decoding remains backward-compatible with version 1.
"""

from __future__ import annotations

import ctypes
import ctypes.util
import logging
import math
import os
import struct
import sys
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)

MAGIC = b"PCLD"
VERSION = 1
VERSION_V2 = 2
HEADER_SIZE = 28
HEADER_FMT = "<4sBBHIffff"  # magic, version, flags, _, count, scale, ox, oy, oz
V2_HEADER_FMT = "<4sBBHIffffIIdBBH"
V2_HEADER_BASE_SIZE = struct.calcsize(V2_HEADER_FMT)

FLAG_HAS_COLOR = 0x01

_STREAM_KIND_TO_CODE = {"cloud": 0, "map": 1, "scan": 2, "reset": 3}
_STREAM_CODE_TO_KIND = {value: key for key, value in _STREAM_KIND_TO_CODE.items()}


@dataclass(frozen=True)
class DecodedPointCloudFrame:
    """Decoded point cloud plus viewer alignment metadata."""

    points: np.ndarray
    colors: np.ndarray | None
    frame_id: str
    epoch: int
    stamp_s: float
    sequence: int
    stream_kind: str


class _NativeCodec:
    def __init__(self, lib: ctypes.CDLL) -> None:
        self._lib = lib
        self._encode = lib.lingtu_encode_pointcloud_v1
        self._encode.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint32,
            ctypes.c_float,
            ctypes.c_void_p,
            ctypes.c_uint8,
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.POINTER(ctypes.c_size_t),
        ]
        self._encode.restype = ctypes.c_int
        self._free = lib.lingtu_free_buffer
        self._free.argtypes = [ctypes.c_void_p]
        self._free.restype = None

    def encode(
        self,
        pts: np.ndarray,
        *,
        scale: float,
        colors: np.ndarray | None,
    ) -> bytes:
        pts = np.ascontiguousarray(pts[:, :3], dtype=np.float32)
        count = int(pts.shape[0])
        rgb: np.ndarray | None = None
        if colors is not None and len(colors) == count:
            rgb = np.ascontiguousarray(colors[:, :3], dtype=np.uint8)

        out = ctypes.c_void_p()
        out_len = ctypes.c_size_t()
        rc = self._encode(
            ctypes.c_void_p(pts.ctypes.data),
            ctypes.c_uint32(count),
            ctypes.c_float(float(scale)),
            ctypes.c_void_p(0 if rgb is None else rgb.ctypes.data),
            ctypes.c_uint8(1 if rgb is not None else 0),
            ctypes.byref(out),
            ctypes.byref(out_len),
        )
        if rc != 0:
            raise RuntimeError(f"native pointcloud codec failed: {rc}")
        try:
            return ctypes.string_at(out, out_len.value)
        finally:
            if out:
                self._free(out)


def _library_names() -> tuple[str, ...]:
    if sys.platform.startswith("win"):
        return ("lingtu_pointcloud_codec.dll",)
    if sys.platform == "darwin":
        return ("liblingtu_pointcloud_codec.dylib", "lingtu_pointcloud_codec.dylib")
    return ("liblingtu_pointcloud_codec.so", "lingtu_pointcloud_codec.so")


def _native_library_candidates() -> list[Path]:
    env = os.environ.get("LINGTU_POINTCLOUD_CODEC_LIB")
    candidates: list[Path] = [Path(env)] if env else []

    repo = Path(__file__).resolve().parents[3]
    kernel = repo / "src" / "kernels" / "gateway" / "pointcloud_codec"
    roots = (
        kernel / "build",
        kernel / "build" / "Release",
        kernel / "build" / "Debug",
        repo / "build" / "pointcloud_codec",
        repo / "build" / "pointcloud_codec" / "Release",
        repo / "build" / "pointcloud_codec" / "Debug",
    )
    for root in roots:
        for name in _library_names():
            candidates.append(root / name)
    return candidates


@lru_cache(maxsize=1)
def _load_native_codec() -> _NativeCodec | None:
    for candidate in _native_library_candidates():
        if candidate.is_file():
            try:
                return _NativeCodec(ctypes.CDLL(str(candidate)))
            except OSError:
                continue
    found = ctypes.util.find_library("lingtu_pointcloud_codec")
    if found:
        try:
            return _NativeCodec(ctypes.CDLL(found))
        except OSError:
            return None
    return None


def encoder_backend() -> str:
    """Return the active point-cloud encoder backend name."""

    return "cpp" if _load_native_codec() is not None else "numpy"


def encode_pointcloud(
    pts: np.ndarray,
    *,
    scale: float = 0.005,
    colors: np.ndarray | None = None,
    frame_id: str | None = None,
    epoch: int = 0,
    stamp_s: float = 0.0,
    sequence: int = 0,
    stream_kind: str = "cloud",
) -> bytes:
    """Pack an (N,3) float array into the binary frame above.

    ``scale`` is the minimum wire resolution in metres (5 mm by default). The
    origin is the cloud bounding-box center; larger outdoor extents increase
    the effective scale just enough to avoid int16 saturation.

    All work stays in numpy / struct, no per-point Python objects.
    """
    if not math.isfinite(float(scale)) or scale <= 0:
        raise ValueError("scale must be finite and greater than zero")

    use_v2 = frame_id is not None
    native = None if use_v2 else _load_native_codec()
    if native is not None and pts.size:
        native_pts = np.asarray(pts[:, :3], dtype=np.float32)
        native_span = np.ptp(native_pts, axis=0)
        if not np.isfinite(native_pts).all() or float(np.max(native_span)) > 32766.0 * float(scale):
            native = None
    if native is not None:
        try:
            return native.encode(pts, scale=scale, colors=colors)
        except Exception as exc:
            logger.debug("native pointcloud codec failed, falling back to numpy: %s", exc)

    if pts.size == 0:
        if use_v2:
            return _empty_header_v2(
                frame_id=frame_id or "",
                epoch=epoch,
                stamp_s=stamp_s,
                sequence=sequence,
                stream_kind=stream_kind,
            )
        return _empty_header()

    pts = np.ascontiguousarray(pts[:, :3], dtype=np.float32)
    if not np.isfinite(pts).all():
        raise ValueError("point cloud contains non-finite coordinates")
    bounds_min = pts.min(axis=0).astype(np.float64)
    bounds_max = pts.max(axis=0).astype(np.float64)
    origin = bounds_min + (bounds_max - bounds_min) * 0.5
    half_extent = float(np.max((bounds_max - bounds_min) * 0.5))
    effective_scale = max(float(scale), half_extent / 32766.0)
    quantized = np.rint((pts - origin) / effective_scale)
    if np.any(quantized < -32768) or np.any(quantized > 32767):
        raise ValueError("point cloud extent exceeds int16 quantization range")
    quant = quantized.astype(np.int16)

    flags = 0
    payload = quant.tobytes()
    if colors is not None and len(colors) == len(pts):
        flags |= FLAG_HAS_COLOR
        rgb = np.ascontiguousarray(colors[:, :3], dtype=np.uint8)
        payload += rgb.tobytes()

    if use_v2:
        header = _pack_v2_header(
            flags=flags,
            count=len(pts),
            scale=effective_scale,
            origin=origin,
            frame_id=frame_id or "",
            epoch=epoch,
            stamp_s=stamp_s,
            sequence=sequence,
            stream_kind=stream_kind,
        )
    else:
        header = struct.pack(
            HEADER_FMT,
            MAGIC,
            VERSION,
            flags,
            0,
            len(pts),
            effective_scale,
            float(origin[0]),
            float(origin[1]),
            float(origin[2]),
        )
    return header + payload


def decode_pointcloud(buf: bytes) -> tuple[np.ndarray, np.ndarray | None]:
    """Decode a v1 or v2 cloud while preserving the legacy return shape."""

    frame = decode_pointcloud_frame(buf)
    return frame.points, frame.colors


def decode_pointcloud_frame(buf: bytes) -> DecodedPointCloudFrame:
    """Decode a cloud frame, including v2 alignment metadata."""
    if len(buf) < HEADER_SIZE:
        raise ValueError("buffer too small for header")
    magic, version, flags, header_field, count, scale, ox, oy, oz = struct.unpack_from(
        HEADER_FMT,
        buf,
        0,
    )
    if magic != MAGIC:
        raise ValueError(f"bad magic: {magic!r}")
    if version not in (VERSION, VERSION_V2):
        raise ValueError(f"unsupported version: {version}")
    if flags & ~FLAG_HAS_COLOR:
        raise ValueError(f"unsupported flags: {flags:#x}")
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError(f"invalid scale: {scale}")
    if not all(math.isfinite(value) for value in (ox, oy, oz)):
        raise ValueError("origin must be finite")

    frame_id = ""
    epoch = 0
    stamp_s = 0.0
    sequence = 0
    stream_kind = "cloud"
    payload_offset = HEADER_SIZE
    if version == VERSION_V2:
        if len(buf) < V2_HEADER_BASE_SIZE:
            raise ValueError("buffer too small for v2 header")
        (
            _,
            _,
            _,
            header_size,
            _,
            _,
            _,
            _,
            _,
            epoch,
            sequence,
            stamp_s,
            stream_code,
            frame_len,
            _,
        ) = struct.unpack_from(V2_HEADER_FMT, buf, 0)
        if header_size < V2_HEADER_BASE_SIZE or header_size > len(buf):
            raise ValueError(f"invalid v2 header size: {header_size}")
        frame_end = V2_HEADER_BASE_SIZE + frame_len
        if frame_end > header_size:
            raise ValueError("frame_id extends beyond v2 header")
        try:
            frame_id = buf[V2_HEADER_BASE_SIZE:frame_end].decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ValueError("frame_id is not valid UTF-8") from exc
        if stream_code not in _STREAM_CODE_TO_KIND:
            raise ValueError(f"unsupported stream kind code: {stream_code}")
        if not math.isfinite(stamp_s):
            raise ValueError(f"invalid stamp: {stamp_s}")
        stream_kind = _STREAM_CODE_TO_KIND[stream_code]
        payload_offset = header_size
    elif header_field != 0:
        raise ValueError(f"invalid v1 reserved field: {header_field}")

    pos_bytes = count * 3 * 2
    color_bytes = count * 3 if flags & FLAG_HAS_COLOR else 0
    required_bytes = payload_offset + pos_bytes + color_bytes
    if required_bytes != len(buf):
        raise ValueError(
            f"invalid payload size: expected {required_bytes} bytes, got {len(buf)}",
        )

    pos = np.frombuffer(buf, dtype=np.int16, count=count * 3, offset=payload_offset)
    xyz = pos.reshape(-1, 3).astype(np.float32) * scale + np.array(
        [ox, oy, oz],
        dtype=np.float32,
    )

    colors: np.ndarray | None = None
    if flags & FLAG_HAS_COLOR:
        col_off = payload_offset + pos_bytes
        colors = (
            np.frombuffer(
                buf,
                dtype=np.uint8,
                count=count * 3,
                offset=col_off,
            )
            .reshape(-1, 3)
            .copy()
        )
    return DecodedPointCloudFrame(
        points=xyz,
        colors=colors,
        frame_id=frame_id,
        epoch=int(epoch),
        stamp_s=float(stamp_s),
        sequence=int(sequence),
        stream_kind=stream_kind,
    )


def _empty_header() -> bytes:
    return struct.pack(HEADER_FMT, MAGIC, VERSION, 0, 0, 0, 1.0, 0.0, 0.0, 0.0)


def _pack_v2_header(
    *,
    flags: int,
    count: int,
    scale: float,
    origin: np.ndarray,
    frame_id: str,
    epoch: int,
    stamp_s: float,
    sequence: int,
    stream_kind: str,
) -> bytes:
    frame_bytes = frame_id.encode("utf-8")
    if len(frame_bytes) > 255:
        raise ValueError("frame_id must be at most 255 UTF-8 bytes")
    if stream_kind not in _STREAM_KIND_TO_CODE:
        raise ValueError(f"unsupported stream kind: {stream_kind!r}")
    if not 0 <= int(epoch) <= 0xFFFFFFFF:
        raise ValueError("epoch must fit in uint32")
    if not 0 <= int(sequence) <= 0xFFFFFFFF:
        raise ValueError("sequence must fit in uint32")
    if not math.isfinite(float(stamp_s)):
        raise ValueError("stamp_s must be finite")

    padding = b"\x00" if len(frame_bytes) % 2 else b""
    header_size = V2_HEADER_BASE_SIZE + len(frame_bytes) + len(padding)
    base = struct.pack(
        V2_HEADER_FMT,
        MAGIC,
        VERSION_V2,
        flags,
        header_size,
        count,
        scale,
        float(origin[0]),
        float(origin[1]),
        float(origin[2]),
        int(epoch),
        int(sequence),
        float(stamp_s),
        _STREAM_KIND_TO_CODE[stream_kind],
        len(frame_bytes),
        0,
    )
    return base + frame_bytes + padding


def _empty_header_v2(
    *,
    frame_id: str,
    epoch: int,
    stamp_s: float,
    sequence: int,
    stream_kind: str,
) -> bytes:
    return _pack_v2_header(
        flags=0,
        count=0,
        scale=1.0,
        origin=np.zeros(3, dtype=np.float32),
        frame_id=frame_id,
        epoch=epoch,
        stamp_s=stamp_s,
        sequence=sequence,
        stream_kind=stream_kind,
    )

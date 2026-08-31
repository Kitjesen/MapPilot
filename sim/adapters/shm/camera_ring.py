"""Platform-neutral implementation of ``lingtu.camera.shm_frame.v1``.

The byte layout in this module is intentionally the same as
``src/drivers/real/camera/shm.py`` and ``native/shm_frame_ring.hpp``.  The
in-memory backend makes the contract testable on every platform; the Windows
backend maps the same bytes with a named file mapping.  Neither backend
chooses a global stream name.
"""

from __future__ import annotations

import mmap
import os
import struct
import time
import zlib
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Callable, MutableSequence, cast

SHM_MAGIC = b"LTCSHM01"
SHM_SCHEMA_VERSION = 1
SHM_SCHEMA = "lingtu.camera.shm_frame.v1"
SUPERBLOCK = struct.Struct("<8sHHHHIIIIQQQQ")
SLOT_HEADER = struct.Struct("<QQQIIIIIIIIHHHH16s64s10d24sQ")


class StreamKind(IntEnum):
    """Stream discriminator encoded in each canonical camera slot."""

    INFO = 1
    COLOR = 2
    DEPTH = 3


class CameraShmError(RuntimeError):
    """Base class for rejected camera shared-memory operations."""


class CameraShmNotReady(CameraShmError):
    """The ring has not committed a frame yet."""


class CameraShmChanged(CameraShmError):
    """A slot or the superblock changed during a read."""


class CameraShmCorrupt(CameraShmError):
    """Mapped bytes do not satisfy the v1 ABI."""


class CameraShmStale(CameraShmError):
    """A frame timestamp falls outside the configured live window."""


@dataclass(frozen=True, slots=True)
class CameraShmAllocation:
    """One explicitly allocated ring mapping and its immutable geometry."""

    name: str
    slot_count: int = 2
    slot_capacity: int = 8 * 1024 * 1024

    def __post_init__(self) -> None:
        if not self.name or "\x00" in self.name:
            raise ValueError("camera SHM mapping name must be non-empty and NUL-free")
        if self.slot_count < 2 or self.slot_count > 0xFFFF:
            raise ValueError("camera SHM slot_count must be in [2, 65535]")
        if self.slot_capacity <= 0 or self.slot_capacity > 0xFFFFFFFF:
            raise ValueError("camera SHM slot_capacity must be positive and uint32-sized")

    @property
    def mapping_size(self) -> int:
        """Return the exact bytes required by this ring allocation."""

        return SUPERBLOCK.size + self.slot_count * (SLOT_HEADER.size + self.slot_capacity)


@dataclass(frozen=True, slots=True)
class FrameMetadata:
    """Metadata written into one committed camera slot."""

    stream_kind: StreamKind
    timestamp_ns: int
    width: int
    height: int
    stride: int
    encoding: str
    frame_id: str
    fx: float = 0.0
    fy: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    depth_scale: float = 0.001
    dist_k1: float = 0.0
    dist_k2: float = 0.0
    dist_p1: float = 0.0
    dist_p2: float = 0.0
    dist_k3: float = 0.0


@dataclass(frozen=True, slots=True)
class FrameSnapshot:
    """Immutable copy of one validated committed camera frame."""

    schema_version: int
    sequence: int
    timestamp_ns: int
    stream_kind: StreamKind
    width: int
    height: int
    stride: int
    encoding: str
    frame_id: str
    payload: bytes
    fx: float = 0.0
    fy: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    depth_scale: float = 0.001
    dist_k1: float = 0.0
    dist_k2: float = 0.0
    dist_p1: float = 0.0
    dist_p2: float = 0.0
    dist_k3: float = 0.0


class InMemoryMappingBackend:
    """Explicit named mapping registry for same-process contract tests."""

    def __init__(self) -> None:
        self._mappings: dict[str, bytearray] = {}

    def create(self, allocation: CameraShmAllocation) -> bytearray:
        """Create or reopen an in-memory mapping with the supplied name."""

        existing = self._mappings.get(allocation.name)
        if existing is not None:
            if len(existing) != allocation.mapping_size:
                raise CameraShmCorrupt("existing camera SHM mapping has a different size")
            return existing
        mapping = bytearray(allocation.mapping_size)
        self._mappings[allocation.name] = mapping
        return mapping

    def open(self, allocation: CameraShmAllocation) -> bytearray:
        """Open a previously created in-memory mapping."""

        try:
            return self._mappings[allocation.name]
        except KeyError as exc:
            raise CameraShmNotReady(f"camera SHM mapping is unavailable: {allocation.name}") from exc

    def destroy(self, name: str) -> None:
        """Release a named in-memory mapping from this backend."""

        self._mappings.pop(name, None)


class WindowsNamedMappingBackend:
    """Windows named-file-mapping backend for the same ring bytes."""

    def create(self, allocation: CameraShmAllocation) -> mmap.mmap:
        """Create or open a writable Windows named mapping."""

        self._require_windows()
        return mmap.mmap(
            -1,
            allocation.mapping_size,
            tagname=allocation.name,
            access=mmap.ACCESS_WRITE,
        )

    def open(self, allocation: CameraShmAllocation) -> mmap.mmap:
        """Open an existing Windows named mapping for read access."""

        self._require_windows()
        return mmap.mmap(
            -1,
            allocation.mapping_size,
            tagname=allocation.name,
            access=mmap.ACCESS_READ,
        )

    @staticmethod
    def _require_windows() -> None:
        if os.name != "nt":
            raise OSError("Windows named camera mappings require Windows")


class CameraShmWriter:
    """Write committed frames using the canonical two-slot seqlock protocol."""

    def __init__(
        self,
        mapping: MutableSequence[int] | mmap.mmap,
        allocation: CameraShmAllocation,
        *,
        now_ns: Callable[[], int] = time.time_ns,
    ) -> None:
        if len(mapping) != allocation.mapping_size:
            raise CameraShmCorrupt("camera SHM mapping size does not match allocation")
        self._mapping = mapping
        self.allocation = allocation
        self._now_ns = now_ns
        self._last_sequence: int = 0
        self._initialize_or_validate()

    @property
    def last_sequence(self) -> int:
        """Return the most recently published sequence."""

        return self._last_sequence

    def publish(self, metadata: FrameMetadata, payload: bytes | bytearray | memoryview) -> int:
        """Commit one validated frame and return its monotonically increasing sequence."""

        payload_bytes = bytes(payload)
        self._validate_frame(metadata, len(payload_bytes))
        if self._last_sequence >= (0xFFFFFFFFFFFFFFFF - 1) // 2:
            raise CameraShmError("camera SHM sequence exhausted")

        sequence = self._last_sequence + 1
        slot_index = (sequence - 1) % self.allocation.slot_count
        slot_offset = self._slot_offset(slot_index)
        dirty_token = sequence * 2 + 1
        committed_token = sequence * 2
        timestamp_ns = metadata.timestamp_ns or int(self._now_ns())
        encoding = metadata.encoding.encode("ascii")
        frame_id = metadata.frame_id.encode("utf-8")
        calibration = (
            metadata.fx,
            metadata.fy,
            metadata.cx,
            metadata.cy,
            metadata.depth_scale,
            metadata.dist_k1,
            metadata.dist_k2,
            metadata.dist_p1,
            metadata.dist_p2,
            metadata.dist_k3,
        )

        writable_mapping = cast(Any, self._mapping)
        SLOT_HEADER.pack_into(
            writable_mapping,
            slot_offset,
            dirty_token,
            sequence,
            timestamp_ns,
            metadata.width,
            metadata.height,
            metadata.stride,
            len(payload_bytes),
            self.allocation.slot_capacity,
            zlib.crc32(payload_bytes) & 0xFFFFFFFF if payload_bytes else 0,
            0,
            0,
            int(metadata.stream_kind),
            len(encoding),
            SHM_SCHEMA_VERSION,
            SLOT_HEADER.size,
            encoding.ljust(16, b"\0"),
            frame_id.ljust(64, b"\0"),
            *calibration,
            bytes(24),
            0,
        )
        payload_offset = slot_offset + SLOT_HEADER.size
        self._mapping[payload_offset : payload_offset + len(payload_bytes)] = payload_bytes
        struct.pack_into("<Q", writable_mapping, slot_offset + 248, committed_token)
        struct.pack_into("<Q", writable_mapping, slot_offset, committed_token)
        struct.pack_into("<Q", writable_mapping, 48, int(self._now_ns()))
        struct.pack_into("<I", writable_mapping, 20, slot_index)
        struct.pack_into("<Q", writable_mapping, 32, sequence)
        self._last_sequence = sequence
        return sequence

    def _initialize_or_validate(self) -> None:
        raw = bytes(self._mapping[: SUPERBLOCK.size])
        if raw == bytes(SUPERBLOCK.size):
            created_ns = int(self._now_ns())
            SUPERBLOCK.pack_into(
                cast(Any, self._mapping),
                0,
                SHM_MAGIC,
                SHM_SCHEMA_VERSION,
                SUPERBLOCK.size,
                SLOT_HEADER.size,
                self.allocation.slot_count,
                self.allocation.slot_capacity,
                0,
                0,
                0,
                0,
                created_ns,
                created_ns,
                0,
            )
        header = _parse_superblock(cast(Any, self._mapping), self.allocation)
        self._last_sequence = header.published_sequence

    def _slot_offset(self, index: int) -> int:
        return SUPERBLOCK.size + index * (SLOT_HEADER.size + self.allocation.slot_capacity)

    def _validate_frame(self, metadata: FrameMetadata, payload_size: int) -> None:
        try:
            stream_kind = StreamKind(metadata.stream_kind)
        except ValueError as exc:
            raise ValueError("unknown camera SHM stream kind") from exc
        try:
            encoding_size = len(metadata.encoding.encode("ascii"))
        except UnicodeEncodeError as exc:
            raise ValueError("camera SHM encoding must be ASCII") from exc
        if encoding_size > 15 or len(metadata.frame_id.encode("utf-8")) > 63:
            raise ValueError("camera SHM encoding or frame_id is too long")
        if payload_size > self.allocation.slot_capacity or payload_size > 0xFFFFFFFF:
            raise ValueError("camera SHM payload exceeds slot capacity")
        if stream_kind is StreamKind.INFO:
            if metadata.width <= 0 or metadata.height <= 0 or metadata.stride != 0 or payload_size:
                raise ValueError("camera info SHM frame geometry is invalid")
            return
        if metadata.width <= 0 or metadata.height <= 0 or metadata.stride <= 0:
            raise ValueError("camera image SHM dimensions are invalid")
        if payload_size != metadata.stride * metadata.height:
            raise ValueError("camera image SHM payload must equal stride * height")
        bpp = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "mono8": 1, "8UC1": 1, "16UC1": 2, "32FC1": 4}.get(metadata.encoding)
        if bpp is None or metadata.stride < metadata.width * bpp:
            raise ValueError("camera image SHM encoding or stride is invalid")


class CameraShmReader:
    """Read and validate the newest committed frame from a v1 ring."""

    def __init__(
        self,
        mapping: bytes | bytearray | memoryview | mmap.mmap,
        *,
        max_age_s: float | None = 1.0,
        max_future_skew_s: float = 2.0,
        consistency_attempts: int = 4,
    ) -> None:
        self._mapping = mapping
        self.max_age_s = max_age_s
        self.max_future_skew_s = max_future_skew_s
        self.consistency_attempts = max(1, int(consistency_attempts))
        self._last_sequence = 0
        self._last_rejected_sequence = 0

    @property
    def last_sequence(self) -> int:
        """Return the most recently accepted sequence."""

        return self._last_sequence

    def read_latest(self, *, now_ns: int | None = None) -> FrameSnapshot | None:
        """Return a new committed frame, or ``None`` when sequence is unchanged."""

        header = _parse_superblock(self._mapping)
        sequence_hint = header.published_sequence
        if sequence_hint == 0:
            raise CameraShmNotReady("camera SHM has no committed frame")
        if sequence_hint in {self._last_sequence, self._last_rejected_sequence}:
            return None
        last_changed: CameraShmChanged | None = None
        for _ in range(self.consistency_attempts):
            try:
                frame = self._read_once(now_ns=now_ns)
            except CameraShmChanged as exc:
                last_changed = exc
                continue
            except (CameraShmCorrupt, CameraShmStale):
                self._last_rejected_sequence = sequence_hint
                raise
            if frame.sequence == self._last_sequence:
                return None
            self._last_sequence = frame.sequence
            self._last_rejected_sequence = 0
            return frame
        raise last_changed or CameraShmChanged("camera SHM slot remained unstable")

    def _read_once(self, *, now_ns: int | None) -> FrameSnapshot:
        first_superblock = bytes(self._mapping[: SUPERBLOCK.size])
        header = _parse_superblock(first_superblock)
        if header.published_sequence == 0:
            raise CameraShmNotReady("camera SHM has no committed frame")
        if header.active_slot >= header.slot_count:
            raise CameraShmCorrupt("camera SHM active slot is outside the ring")
        expected_size = SUPERBLOCK.size + header.slot_count * (SLOT_HEADER.size + header.slot_capacity)
        if len(self._mapping) != expected_size:
            raise CameraShmCorrupt("camera SHM size does not match its superblock")

        slot_offset = SUPERBLOCK.size + header.active_slot * (SLOT_HEADER.size + header.slot_capacity)
        first_slot = bytes(self._mapping[slot_offset : slot_offset + SLOT_HEADER.size])
        slot = SLOT_HEADER.unpack(first_slot)
        guard_begin, sequence, timestamp_ns = int(slot[0]), int(slot[1]), int(slot[2])
        if guard_begin == 0 or guard_begin & 1 or guard_begin != int(slot[-1]):
            raise CameraShmChanged("camera SHM slot is being written")
        if sequence != header.published_sequence or guard_begin != sequence * 2:
            raise CameraShmChanged("camera SHM slot and superblock sequences disagree")
        payload_size, payload_capacity = int(slot[6]), int(slot[7])
        if payload_capacity != header.slot_capacity or payload_size > payload_capacity:
            raise CameraShmCorrupt("camera SHM payload capacity is invalid")
        payload_offset = slot_offset + SLOT_HEADER.size
        payload = bytes(self._mapping[payload_offset : payload_offset + payload_size])
        if first_slot != bytes(
            self._mapping[slot_offset : slot_offset + SLOT_HEADER.size]
        ) or first_superblock != bytes(self._mapping[: SUPERBLOCK.size]):
            raise CameraShmChanged("camera SHM changed while copying the frame")

        if int(slot[13]) != SHM_SCHEMA_VERSION or int(slot[14]) != SLOT_HEADER.size:
            raise CameraShmCorrupt("camera SHM slot schema does not match superblock")
        encoding_size = int(slot[12])
        if encoding_size > 15:
            raise CameraShmCorrupt("camera SHM encoding exceeds the fixed field")
        try:
            stream_kind = StreamKind(int(slot[11]))
            encoding = bytes(slot[15][:encoding_size]).decode("ascii", errors="strict")
            frame_id = bytes(slot[16]).split(b"\0", 1)[0].decode("utf-8", errors="strict")
        except (ValueError, UnicodeError) as exc:
            raise CameraShmCorrupt("camera SHM slot metadata is invalid") from exc
        width, height, stride = int(slot[3]), int(slot[4]), int(slot[5])
        _validate_payload(stream_kind, encoding, width, height, stride, payload_size)
        if int(slot[8]) != (zlib.crc32(payload) & 0xFFFFFFFF if payload else 0):
            raise CameraShmCorrupt("camera SHM payload CRC mismatch")
        _validate_timestamp(timestamp_ns, now_ns, self.max_age_s, self.max_future_skew_s)
        calibration = tuple(float(value) for value in slot[17:27])
        return FrameSnapshot(
            schema_version=int(slot[13]),
            sequence=sequence,
            timestamp_ns=timestamp_ns,
            stream_kind=stream_kind,
            width=width,
            height=height,
            stride=stride,
            encoding=encoding,
            frame_id=frame_id,
            payload=payload,
            fx=calibration[0],
            fy=calibration[1],
            cx=calibration[2],
            cy=calibration[3],
            depth_scale=calibration[4],
            dist_k1=calibration[5],
            dist_k2=calibration[6],
            dist_p1=calibration[7],
            dist_p2=calibration[8],
            dist_k3=calibration[9],
        )


@dataclass(frozen=True, slots=True)
class _Superblock:
    slot_count: int
    slot_capacity: int
    active_slot: int
    published_sequence: int


def _parse_superblock(
    mapping: bytes | bytearray | memoryview | mmap.mmap,
    allocation: CameraShmAllocation | None = None,
) -> _Superblock:
    if len(mapping) < SUPERBLOCK.size:
        raise CameraShmCorrupt("camera SHM is shorter than the superblock")
    values = SUPERBLOCK.unpack(bytes(mapping[: SUPERBLOCK.size]))
    if values[0] != SHM_MAGIC:
        raise CameraShmCorrupt("camera SHM magic does not match LTCSHM01")
    if values[1] != SHM_SCHEMA_VERSION:
        raise CameraShmCorrupt("unsupported camera SHM schema version")
    if values[2] != SUPERBLOCK.size or values[3] != SLOT_HEADER.size:
        raise CameraShmCorrupt("camera SHM header sizes do not match schema v1")
    slot_count, slot_capacity = int(values[4]), int(values[5])
    if slot_count < 2 or slot_capacity <= 0:
        raise CameraShmCorrupt("camera SHM ring geometry is invalid")
    if allocation is not None and (slot_count != allocation.slot_count or slot_capacity != allocation.slot_capacity):
        raise CameraShmCorrupt("camera SHM mapping geometry does not match allocation")
    if int(values[10]) <= 0:
        raise CameraShmCorrupt("camera SHM generation timestamp is not positive")
    return _Superblock(slot_count, slot_capacity, int(values[6]), int(values[9]))


def _validate_payload(
    stream_kind: StreamKind, encoding: str, width: int, height: int, stride: int, payload_size: int
) -> None:
    if stream_kind is StreamKind.INFO:
        if payload_size != 0 or stride != 0 or width <= 0 or height <= 0:
            raise CameraShmCorrupt("camera info SHM frame geometry is invalid")
        return
    if width <= 0 or height <= 0 or stride <= 0 or payload_size != stride * height:
        raise CameraShmCorrupt("camera SHM payload is not exactly stride * height")
    bpp = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "mono8": 1, "8UC1": 1, "16UC1": 2, "32FC1": 4}.get(encoding)
    if bpp is None or stride < width * bpp:
        raise CameraShmCorrupt(f"unsupported or invalid camera SHM encoding: {encoding!r}")


def _validate_timestamp(
    timestamp_ns: int, now_ns: int | None, max_age_s: float | None, max_future_skew_s: float
) -> None:
    if timestamp_ns <= 0:
        raise CameraShmCorrupt("camera SHM timestamp is not positive")
    current_ns = time.time_ns() if now_ns is None else int(now_ns)
    age_ns = current_ns - timestamp_ns
    if age_ns < -int(max_future_skew_s * 1e9):
        raise CameraShmStale("camera SHM timestamp is too far in the future")
    if max_age_s is not None and age_ns > int(max_age_s * 1e9):
        raise CameraShmStale("camera SHM frame exceeded the stale threshold")

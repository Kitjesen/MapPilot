"""Versioned shared-memory camera frame reader.

The C++ camera service owns the POSIX SHM writer.  Python only maps the
corresponding ``/dev/shm`` file and copies a frame after the slot seqlock,
layout, timestamp, payload length, and CRC have all been validated.
"""

from __future__ import annotations

import mmap
import os
import struct
import time
import zlib
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path

SHM_MAGIC = b"LTCSHM01"
SHM_SCHEMA_VERSION = 1
SHM_SCHEMA = "lingtu.camera.shm_frame.v1"
SUPERBLOCK = struct.Struct("<8sHHHHIIIIQQQQ")
SLOT_HEADER = struct.Struct("<QQQIIIIIIIIHHHH16s64s10d24sQ")
POSIX_SHM_DIRECTORY = Path("/dev/shm")  # noqa: S108 - required POSIX SHM mount


class StreamKind(IntEnum):
    """Camera stream identifiers encoded in each SHM slot."""

    INFO = 1
    COLOR = 2
    DEPTH = 3


class ShmFrameError(RuntimeError):
    """Base class for rejected shared-memory camera frames."""


class ShmUnavailable(ShmFrameError):
    """The writer's shared-memory object is not available."""


class FrameNotReady(ShmFrameError):
    """The writer has not committed its first frame."""


class FrameChanged(ShmFrameError):
    """The writer changed a slot while it was being copied."""


class FrameCorrupt(ShmFrameError):
    """The mapped frame violates the schema or payload contract."""


class FrameStale(ShmFrameError):
    """The frame timestamp is outside the accepted live window."""


@dataclass(frozen=True, slots=True)
class FrameSnapshot:
    """Immutable, fully validated copy of one committed camera frame."""

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

    @property
    def timestamp_s(self) -> float:
        """Return the Unix timestamp in seconds."""

        return self.timestamp_ns * 1e-9


class ShmFrameWriter:
    """Publish camera frames to a portable file-backed LTCSHM01 ring."""

    def __init__(
        self,
        path: str | os.PathLike[str],
        *,
        stream_kind: StreamKind,
        slot_capacity: int,
        slot_count: int = 2,
    ) -> None:
        self.path = Path(path)
        self.stream_kind = StreamKind(stream_kind)
        self.slot_capacity = int(slot_capacity)
        self.slot_count = int(slot_count)
        if self.slot_count < 2 or self.slot_count > 0xFFFF:
            raise ValueError("camera SHM slot_count must be in [2, 65535]")
        if self.slot_capacity <= 0 or self.slot_capacity > 0xFFFFFFFF:
            raise ValueError("camera SHM slot_capacity must be in [1, 2^32 - 1]")
        if not self.path.parent.is_dir():
            raise FileNotFoundError(f"camera SHM parent directory is missing: {self.path.parent}")
        self._fd: int | None = None
        self._mapping: mmap.mmap | None = None
        self._last_sequence = 0
        self._generation_ns = 0
        self._open()

    @property
    def last_sequence(self) -> int:
        """Return the newest sequence committed by this writer."""

        return self._last_sequence

    @property
    def generation_ns(self) -> int:
        """Return the generation identifier written to the superblock."""

        return self._generation_ns

    def __enter__(self) -> ShmFrameWriter:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    def close(self) -> None:
        """Flush and close the mapped ring while leaving its file available."""

        mapping, fd = self._mapping, self._fd
        self._mapping = None
        self._fd = None
        if mapping is not None:
            mapping.flush()
            mapping.close()
        if fd is not None:
            os.close(fd)

    def publish(
        self,
        *,
        timestamp_ns: int,
        width: int,
        height: int,
        stride: int,
        encoding: str,
        frame_id: str,
        payload: bytes | bytearray | memoryview,
        fx: float = 0.0,
        fy: float = 0.0,
        cx: float = 0.0,
        cy: float = 0.0,
        depth_scale: float = 0.001,
        dist_k1: float = 0.0,
        dist_k2: float = 0.0,
        dist_p1: float = 0.0,
        dist_p2: float = 0.0,
        dist_k3: float = 0.0,
    ) -> int:
        """Validate and atomically commit one complete frame."""

        mapping = self._mapping
        if mapping is None:
            raise RuntimeError("camera SHM writer is closed")
        payload_bytes = bytes(payload)
        timestamp = int(timestamp_ns)
        width_value = int(width)
        height_value = int(height)
        stride_value = int(stride)
        self._validate_frame(
            timestamp_ns=timestamp,
            width=width_value,
            height=height_value,
            stride=stride_value,
            encoding=encoding,
            frame_id=frame_id,
            payload_size=len(payload_bytes),
        )
        if self._last_sequence >= (0xFFFFFFFFFFFFFFFF - 1) // 2:
            raise RuntimeError("camera SHM sequence exhausted")

        sequence = self._last_sequence + 1
        slot_index = (sequence - 1) % self.slot_count
        slot_offset = SUPERBLOCK.size + slot_index * (SLOT_HEADER.size + self.slot_capacity)
        dirty_token = sequence * 2 + 1
        committed_token = sequence * 2
        struct.pack_into("<Q", mapping, slot_offset, dirty_token)
        struct.pack_into("<Q", mapping, slot_offset + SLOT_HEADER.size - 8, 0)
        encoding_bytes = encoding.encode("ascii")
        frame_id_bytes = frame_id.encode("utf-8")
        SLOT_HEADER.pack_into(
            mapping,
            slot_offset,
            dirty_token,
            sequence,
            timestamp,
            width_value,
            height_value,
            stride_value,
            len(payload_bytes),
            self.slot_capacity,
            zlib.crc32(payload_bytes) & 0xFFFFFFFF,
            0,
            0,
            int(self.stream_kind),
            len(encoding_bytes),
            SHM_SCHEMA_VERSION,
            SLOT_HEADER.size,
            encoding_bytes.ljust(16, b"\0"),
            frame_id_bytes.ljust(64, b"\0"),
            float(fx),
            float(fy),
            float(cx),
            float(cy),
            float(depth_scale),
            float(dist_k1),
            float(dist_k2),
            float(dist_p1),
            float(dist_p2),
            float(dist_k3),
            bytes(24),
            0,
        )
        payload_offset = slot_offset + SLOT_HEADER.size
        mapping[payload_offset : payload_offset + len(payload_bytes)] = payload_bytes
        struct.pack_into("<Q", mapping, slot_offset + SLOT_HEADER.size - 8, committed_token)
        struct.pack_into("<Q", mapping, slot_offset, committed_token)
        heartbeat_ns = time.time_ns()
        SUPERBLOCK.pack_into(
            mapping,
            0,
            SHM_MAGIC,
            SHM_SCHEMA_VERSION,
            SUPERBLOCK.size,
            SLOT_HEADER.size,
            self.slot_count,
            self.slot_capacity,
            slot_index,
            0,
            0,
            sequence,
            self._generation_ns,
            heartbeat_ns,
            0,
        )
        mapping.flush()
        self._last_sequence = sequence
        return sequence

    def _open(self) -> None:
        previous_generation = 0
        try:
            with self.path.open("rb") as existing:
                header = existing.read(SUPERBLOCK.size)
            if len(header) == SUPERBLOCK.size:
                values = SUPERBLOCK.unpack(header)
                if values[0] == SHM_MAGIC:
                    previous_generation = int(values[10])
        except FileNotFoundError:
            pass
        expected_size = SUPERBLOCK.size + self.slot_count * (SLOT_HEADER.size + self.slot_capacity)
        fd = os.open(self.path, os.O_CREAT | os.O_RDWR, 0o600)
        try:
            os.chmod(self.path, 0o600)
            os.ftruncate(fd, expected_size)
            mapping = mmap.mmap(fd, expected_size, access=mmap.ACCESS_WRITE)
        except Exception:
            os.close(fd)
            raise
        self._fd = fd
        self._mapping = mapping
        self._generation_ns = max(time.time_ns(), previous_generation + 1)
        mapping[:] = bytes(expected_size)
        SUPERBLOCK.pack_into(
            mapping,
            0,
            SHM_MAGIC,
            SHM_SCHEMA_VERSION,
            SUPERBLOCK.size,
            SLOT_HEADER.size,
            self.slot_count,
            self.slot_capacity,
            0,
            0,
            0,
            0,
            self._generation_ns,
            self._generation_ns,
            0,
        )
        mapping.flush()

    def _validate_frame(
        self,
        *,
        timestamp_ns: int,
        width: int,
        height: int,
        stride: int,
        encoding: str,
        frame_id: str,
        payload_size: int,
    ) -> None:
        if timestamp_ns <= 0:
            raise ValueError("camera SHM timestamp must be positive")
        if not 0 < width <= 0xFFFFFFFF or not 0 < height <= 0xFFFFFFFF:
            raise ValueError("camera SHM dimensions must be positive uint32 values")
        if not 0 <= stride <= 0xFFFFFFFF:
            raise ValueError("camera SHM stride must be a uint32 value")
        try:
            encoding_bytes = encoding.encode("ascii")
            frame_id_bytes = frame_id.encode("utf-8")
        except UnicodeError as exc:
            raise ValueError("camera SHM encoding and frame_id must be encodable") from exc
        if not encoding_bytes or len(encoding_bytes) > 15 or len(frame_id_bytes) > 63:
            raise ValueError("camera SHM encoding or frame_id exceeds its fixed field")
        if payload_size > self.slot_capacity:
            raise ValueError("camera SHM payload exceeds slot capacity")
        if self.stream_kind is StreamKind.INFO:
            if stride != 0 or payload_size != 0:
                raise ValueError("camera info SHM frame must not carry image bytes")
            return
        if stride <= 0 or payload_size != stride * height:
            raise ValueError("camera image SHM payload must equal stride * height")
        bytes_per_pixel = {
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "mono8": 1,
            "8UC1": 1,
            "16UC1": 2,
            "32FC1": 4,
        }.get(encoding)
        if bytes_per_pixel is None or stride < width * bytes_per_pixel:
            raise ValueError("camera image SHM encoding or stride is invalid")


def posix_shm_path(name: str) -> Path:
    """Return the Linux filesystem path for a POSIX SHM object name."""

    value = name.strip()
    if not value:
        raise ValueError("POSIX SHM name must not be empty")
    if value.startswith(f"{POSIX_SHM_DIRECTORY}/"):
        return Path(value)
    if value.startswith("/"):
        value = value[1:]
    if not value or "/" in value:
        raise ValueError("POSIX SHM name must contain only one leading slash")
    return POSIX_SHM_DIRECTORY / value


class ShmFrameReader:
    """Read the newest committed frame from a versioned SHM ring."""

    def __init__(
        self,
        path: str | os.PathLike[str],
        *,
        max_age_s: float | None = 1.0,
        max_future_skew_s: float = 2.0,
        consistency_attempts: int = 4,
    ) -> None:
        self.path = Path(path)
        self.max_age_s = max_age_s
        self.max_future_skew_s = max_future_skew_s
        self.consistency_attempts = max(1, int(consistency_attempts))
        self._fd: int | None = None
        self._mapping: mmap.mmap | None = None
        self._identity: tuple[int, int, int] | None = None
        self._last_sequence = 0
        self._last_rejected_sequence = 0
        self._generation_ns = 0

    @property
    def last_sequence(self) -> int:
        """Return the newest sequence accepted by this reader."""

        return self._last_sequence

    def close(self) -> None:
        """Close the current file descriptor and mmap view."""

        mapping, fd = self._mapping, self._fd
        self._mapping = None
        self._fd = None
        self._identity = None
        if mapping is not None:
            mapping.close()
        if fd is not None:
            os.close(fd)

    def __enter__(self) -> ShmFrameReader:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    def read_latest(self, *, now_ns: int | None = None) -> FrameSnapshot | None:
        """Return a new committed frame, or ``None`` when sequence is unchanged."""

        self._ensure_open()
        sequence_hint, generation_ns = self._published_sequence_hint()
        if self._generation_ns != generation_ns:
            self._generation_ns = generation_ns
            self._last_sequence = 0
            self._last_rejected_sequence = 0
        if sequence_hint == 0:
            raise FrameNotReady("camera SHM has no committed frame")
        if sequence_hint in {self._last_sequence, self._last_rejected_sequence}:
            return None
        last_changed: FrameChanged | None = None
        for _ in range(self.consistency_attempts):
            try:
                frame = self._read_once(now_ns=now_ns)
            except FrameChanged as exc:
                last_changed = exc
                continue
            except (FrameCorrupt, FrameStale):
                self._last_rejected_sequence = sequence_hint
                raise
            if frame.sequence == self._last_sequence:
                return None
            self._last_sequence = frame.sequence
            self._last_rejected_sequence = 0
            return frame
        raise last_changed or FrameChanged("camera slot remained unstable")

    def _published_sequence_hint(self) -> tuple[int, int]:
        mapping = self._mapping
        if mapping is None:
            raise ShmUnavailable("camera SHM is not mapped")
        values = SUPERBLOCK.unpack(bytes(mapping[: SUPERBLOCK.size]))
        magic, schema_version, superblock_size, slot_header_size = values[:4]
        if magic != SHM_MAGIC:
            raise FrameCorrupt("camera SHM magic does not match LTCSHM01")
        if schema_version != SHM_SCHEMA_VERSION:
            raise FrameCorrupt(f"unsupported camera SHM schema version: {schema_version}")
        if superblock_size != SUPERBLOCK.size or slot_header_size != SLOT_HEADER.size:
            raise FrameCorrupt("camera SHM header sizes do not match schema v1")
        generation_ns = int(values[10])
        if generation_ns <= 0:
            raise FrameCorrupt("camera SHM generation timestamp is not positive")
        return int(values[9]), generation_ns

    def _ensure_open(self) -> None:
        try:
            stat = self.path.stat()
        except FileNotFoundError as exc:
            self.close()
            raise ShmUnavailable(f"camera SHM is missing: {self.path}") from exc
        identity = (int(stat.st_dev), int(stat.st_ino), int(stat.st_size))
        if self._mapping is not None and identity == self._identity:
            return
        self.close()
        if stat.st_size < SUPERBLOCK.size:
            raise FrameCorrupt(f"camera SHM is shorter than the {SUPERBLOCK.size}-byte superblock")
        try:
            fd = os.open(self.path, os.O_RDONLY)
            mapping = mmap.mmap(fd, stat.st_size, access=mmap.ACCESS_READ)
        except OSError as exc:
            if "fd" in locals():
                os.close(fd)
            raise ShmUnavailable(f"cannot map camera SHM {self.path}: {exc}") from exc
        self._fd = fd
        self._mapping = mapping
        self._identity = identity

    def _read_once(self, *, now_ns: int | None) -> FrameSnapshot:
        mapping = self._mapping
        if mapping is None:
            raise ShmUnavailable("camera SHM is not mapped")
        first_superblock = bytes(mapping[: SUPERBLOCK.size])
        values = SUPERBLOCK.unpack(first_superblock)
        (
            magic,
            schema_version,
            superblock_size,
            slot_header_size,
            slot_count,
            slot_capacity,
            active_slot,
            _flags,
            _reserved,
            published_sequence,
            _created_ns,
            _heartbeat_ns,
            _reserved_ns,
        ) = values
        if magic != SHM_MAGIC:
            raise FrameCorrupt("camera SHM magic does not match LTCSHM01")
        if schema_version != SHM_SCHEMA_VERSION:
            raise FrameCorrupt(f"unsupported camera SHM schema version: {schema_version}")
        if superblock_size != SUPERBLOCK.size or slot_header_size != SLOT_HEADER.size:
            raise FrameCorrupt("camera SHM header sizes do not match schema v1")
        if slot_count < 2 or slot_capacity <= 0:
            raise FrameCorrupt("camera SHM ring geometry is invalid")
        if active_slot >= slot_count:
            raise FrameCorrupt("camera SHM active slot is outside the ring")
        expected_size = superblock_size + slot_count * (slot_header_size + slot_capacity)
        if len(mapping) != expected_size:
            raise FrameCorrupt(f"camera SHM size mismatch: expected {expected_size}, got {len(mapping)}")
        if published_sequence == 0:
            raise FrameNotReady("camera SHM has no committed frame")

        slot_offset = superblock_size + active_slot * (slot_header_size + slot_capacity)
        first_header = bytes(mapping[slot_offset : slot_offset + slot_header_size])
        slot = SLOT_HEADER.unpack(first_header)
        guard_begin = int(slot[0])
        guard_end = int(slot[-1])
        sequence = int(slot[1])
        if guard_begin == 0 or guard_begin & 1 or guard_begin != guard_end:
            raise FrameChanged("camera SHM slot is being written")
        if sequence != published_sequence or guard_begin != sequence * 2:
            raise FrameChanged("camera SHM slot and superblock sequences disagree")

        payload_size = int(slot[6])
        payload_capacity = int(slot[7])
        if payload_capacity != slot_capacity or payload_size > payload_capacity:
            raise FrameCorrupt("camera SHM payload capacity is invalid")
        payload_offset = slot_offset + slot_header_size
        payload = bytes(mapping[payload_offset : payload_offset + payload_size])
        second_header = bytes(mapping[slot_offset : slot_offset + slot_header_size])
        second_superblock = bytes(mapping[: SUPERBLOCK.size])
        if first_header != second_header or first_superblock != second_superblock:
            raise FrameChanged("camera SHM changed while copying the frame")

        stream_kind = self._stream_kind(slot[11])
        encoding_size = int(slot[12])
        slot_schema = int(slot[13])
        encoded_header_size = int(slot[14])
        if slot_schema != SHM_SCHEMA_VERSION or encoded_header_size != SLOT_HEADER.size:
            raise FrameCorrupt("camera SHM slot schema does not match superblock")
        if encoding_size > 15:
            raise FrameCorrupt("camera SHM encoding exceeds the fixed field")
        encoding = bytes(slot[15][:encoding_size]).decode("ascii", errors="strict")
        frame_id = bytes(slot[16]).split(b"\0", 1)[0].decode("utf-8", errors="strict")
        width = int(slot[3])
        height = int(slot[4])
        stride = int(slot[5])
        self._validate_payload(stream_kind, encoding, width, height, stride, payload_size)
        expected_crc = int(slot[8])
        actual_crc = zlib.crc32(payload) & 0xFFFFFFFF
        if expected_crc != actual_crc:
            raise FrameCorrupt("camera SHM payload CRC mismatch")

        timestamp_ns = int(slot[2])
        self._validate_timestamp(timestamp_ns, now_ns=now_ns)
        calibration = tuple(float(value) for value in slot[17:27])
        return FrameSnapshot(
            schema_version=slot_schema,
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

    @staticmethod
    def _stream_kind(value: int) -> StreamKind:
        try:
            return StreamKind(int(value))
        except ValueError as exc:
            raise FrameCorrupt(f"unknown camera SHM stream kind: {value}") from exc

    @staticmethod
    def _validate_payload(
        stream_kind: StreamKind,
        encoding: str,
        width: int,
        height: int,
        stride: int,
        payload_size: int,
    ) -> None:
        if stream_kind is StreamKind.INFO:
            if payload_size != 0 or stride != 0:
                raise FrameCorrupt("camera info SHM slot must not carry image bytes")
            if width <= 0 or height <= 0:
                raise FrameCorrupt("camera info SHM dimensions are invalid")
            return
        if width <= 0 or height <= 0 or stride <= 0:
            raise FrameCorrupt("camera SHM image dimensions are invalid")
        if payload_size != stride * height:
            raise FrameCorrupt("camera SHM payload is not exactly stride * height")
        bytes_per_pixel = {
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "mono8": 1,
            "8UC1": 1,
            "16UC1": 2,
            "32FC1": 4,
        }.get(encoding)
        if bytes_per_pixel is None:
            raise FrameCorrupt(f"unsupported camera SHM encoding: {encoding!r}")
        if stride < width * bytes_per_pixel:
            raise FrameCorrupt("camera SHM stride is shorter than one image row")

    def _validate_timestamp(self, timestamp_ns: int, *, now_ns: int | None) -> None:
        if timestamp_ns <= 0:
            raise FrameCorrupt("camera SHM timestamp is not positive")
        current_ns = time.time_ns() if now_ns is None else int(now_ns)
        age_ns = current_ns - timestamp_ns
        if age_ns < -int(self.max_future_skew_s * 1e9):
            raise FrameStale("camera SHM timestamp is too far in the future")
        if self.max_age_s is not None and age_ns > int(self.max_age_s * 1e9):
            raise FrameStale("camera SHM frame exceeded the stale threshold")

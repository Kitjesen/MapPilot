#!/usr/bin/env python3
"""Convert normalized LiDAR/IMU JSONL records into LingTu LTU1 replay data.

Dataset-specific ROS 1/ROS 2 readers belong in an offline compatibility
environment. This converter is the stable, dependency-free boundary consumed
after those readers have normalized timestamps, units, and per-point timing.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import tempfile
from pathlib import Path
from typing import Any, BinaryIO, NamedTuple

MAGIC = b"LTU1"
RECORD_CLOUD = 1
RECORD_IMU = 2
MAX_PAYLOAD_BYTES = 256 * 1024 * 1024
UINT8_MAX = (1 << 8) - 1
UINT16_MAX = (1 << 16) - 1
UINT32_MAX = (1 << 32) - 1
UINT64_MAX = (1 << 64) - 1
FLOAT32_MAX = float.fromhex("0x1.fffffep+127")

HEADER = struct.Struct("<4sB3xQIII")
POINT = struct.Struct("<ffffIBBH")
IMU = struct.Struct("<ffffff")


class ConversionError(ValueError):
    """Raised when normalized input cannot be represented safely as LTU1."""


class ConversionStats(NamedTuple):
    records: int
    cloud_records: int
    imu_records: int
    points: int
    undeskewed_points: int


def _mapping(value: Any, *, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ConversionError(f"{label} must be a JSON object")
    return value


def _uint(value: Any, *, label: str, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConversionError(f"{label} must be an integer")
    if value < 0 or value > maximum:
        raise ConversionError(f"{label} must be in [0, {maximum}]")
    return value


def _float32(value: Any, *, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConversionError(f"{label} must be numeric")
    number = float(value)
    if not math.isfinite(number):
        raise ConversionError(f"{label} must be finite")
    if abs(number) > FLOAT32_MAX:
        raise ConversionError(f"{label} exceeds float32 range")
    return number


def _vector3(value: Any, *, label: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ConversionError(f"{label} must contain exactly three values")
    return (
        _float32(value[0], label=f"{label}[0]"),
        _float32(value[1], label=f"{label}[1]"),
        _float32(value[2], label=f"{label}[2]"),
    )


def _pack_imu(record: dict[str, Any], *, line_number: int) -> bytes:
    gyro = _vector3(record.get("gyro"), label=f"line {line_number} gyro")
    acc = _vector3(record.get("acc"), label=f"line {line_number} acc")
    return IMU.pack(*gyro, *acc)


def _pack_cloud(
    record: dict[str, Any],
    *,
    line_number: int,
    allow_undeskewed: bool,
) -> tuple[int, bytes, int]:
    raw_points = record.get("points")
    if not isinstance(raw_points, list):
        raise ConversionError(f"line {line_number} points must be a JSON array")
    transport_only = record.get("transport_only", False)
    if not isinstance(transport_only, bool):
        raise ConversionError(f"line {line_number} transport_only must be boolean")
    if transport_only and not allow_undeskewed:
        raise ConversionError(f"line {line_number} is transport_only; use --allow-undeskewed explicitly")
    count = _uint(
        len(raw_points),
        label=f"line {line_number} point count",
        maximum=UINT32_MAX,
    )
    payload_bytes = count * POINT.size
    if payload_bytes > MAX_PAYLOAD_BYTES:
        raise ConversionError(f"line {line_number} cloud payload exceeds {MAX_PAYLOAD_BYTES} bytes")

    payload = bytearray(payload_bytes)
    previous_offset = -1
    undeskewed_points = 0
    for index, raw_point in enumerate(raw_points):
        label = f"line {line_number} point {index}"
        point = _mapping(raw_point, label=label)
        if point.get("offset_time_ns") is None and allow_undeskewed:
            offset_time_ns = 0
            undeskewed_points += 1
        else:
            offset_time_ns = _uint(
                point.get("offset_time_ns"),
                label=f"{label} offset_time_ns",
                maximum=UINT32_MAX,
            )
            if transport_only:
                undeskewed_points += 1
        if offset_time_ns < previous_offset:
            raise ConversionError(f"{label} offset_time_ns must be monotonic within the cloud")
        previous_offset = offset_time_ns
        POINT.pack_into(
            payload,
            index * POINT.size,
            _float32(point.get("x"), label=f"{label} x"),
            _float32(point.get("y"), label=f"{label} y"),
            _float32(point.get("z"), label=f"{label} z"),
            _float32(point.get("intensity", 0.0), label=f"{label} intensity"),
            offset_time_ns,
            _uint(point.get("tag", 0), label=f"{label} tag", maximum=UINT8_MAX),
            _uint(point.get("line", 0), label=f"{label} line", maximum=UINT8_MAX),
            _uint(
                point.get("flags", 0),
                label=f"{label} flags",
                maximum=UINT16_MAX,
            ),
        )
    return count, bytes(payload), undeskewed_points


def _write_record(
    stream: BinaryIO,
    *,
    record_type: int,
    timestamp_ns: int,
    sequence: int,
    count: int,
    payload: bytes,
) -> None:
    stream.write(
        HEADER.pack(
            MAGIC,
            record_type,
            timestamp_ns,
            sequence,
            count,
            len(payload),
        )
    )
    stream.write(payload)


def convert_jsonl(
    source_path: str | Path,
    output_path: str | Path,
    *,
    allow_undeskewed: bool = False,
) -> ConversionStats:
    """Convert normalized JSONL while replacing the output only on success."""

    source = Path(source_path)
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)

    records = 0
    cloud_records = 0
    imu_records = 0
    points = 0
    undeskewed_points = 0
    previous_timestamp_ns: int | None = None
    temp_path: Path | None = None

    try:
        with tempfile.NamedTemporaryFile(
            mode="wb",
            dir=output.parent,
            prefix=f".{output.name}.",
            suffix=".tmp",
            delete=False,
        ) as target:
            temp_path = Path(target.name)
            with source.open("r", encoding="utf-8") as handle:
                for line_number, raw_line in enumerate(handle, start=1):
                    if not raw_line.strip():
                        continue
                    try:
                        decoded = json.loads(raw_line)
                    except json.JSONDecodeError as exc:
                        raise ConversionError(f"line {line_number} is invalid JSON: {exc.msg}") from exc
                    record = _mapping(decoded, label=f"line {line_number}")
                    timestamp_ns = _uint(
                        record.get("timestamp_ns"),
                        label=f"line {line_number} timestamp_ns",
                        maximum=UINT64_MAX,
                    )
                    if previous_timestamp_ns is not None and timestamp_ns < previous_timestamp_ns:
                        raise ConversionError(f"line {line_number} timestamp_ns must be monotonic")
                    previous_timestamp_ns = timestamp_ns
                    sequence = _uint(
                        record.get("sequence", records),
                        label=f"line {line_number} sequence",
                        maximum=UINT32_MAX,
                    )

                    record_type = record.get("type")
                    if record_type == "imu":
                        payload = _pack_imu(record, line_number=line_number)
                        _write_record(
                            target,
                            record_type=RECORD_IMU,
                            timestamp_ns=timestamp_ns,
                            sequence=sequence,
                            count=1,
                            payload=payload,
                        )
                        imu_records += 1
                    elif record_type == "cloud":
                        count, payload, record_undeskewed_points = _pack_cloud(
                            record,
                            line_number=line_number,
                            allow_undeskewed=allow_undeskewed,
                        )
                        _write_record(
                            target,
                            record_type=RECORD_CLOUD,
                            timestamp_ns=timestamp_ns,
                            sequence=sequence,
                            count=count,
                            payload=payload,
                        )
                        cloud_records += 1
                        points += count
                        undeskewed_points += record_undeskewed_points
                    else:
                        raise ConversionError(f"line {line_number} type must be 'imu' or 'cloud'")
                    records += 1
            if imu_records == 0:
                raise ConversionError("replay must contain at least one IMU record")
            if cloud_records == 0:
                raise ConversionError("replay must contain at least one cloud record")
            if points == 0:
                raise ConversionError("replay must contain at least one point")
            target.flush()
            os.fsync(target.fileno())

        os.replace(temp_path, output)
        temp_path = None
    except (OSError, UnicodeError) as exc:
        raise ConversionError(str(exc)) from exc
    finally:
        if temp_path is not None:
            try:
                temp_path.unlink()
            except FileNotFoundError:
                pass

    return ConversionStats(
        records=records,
        cloud_records=cloud_records,
        imu_records=imu_records,
        points=points,
        undeskewed_points=undeskewed_points,
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Convert normalized LiDAR/IMU JSONL into LTU1 replay data.")
    parser.add_argument("source", type=Path, help="normalized JSONL input")
    parser.add_argument("output", type=Path, help="LTU1 output file")
    parser.add_argument(
        "--allow-undeskewed",
        action="store_true",
        help=(
            "fill missing per-point offsets with zero for transport/runtime "
            "smoke tests; output is not valid frontend accuracy evidence"
        ),
    )
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    try:
        stats = convert_jsonl(
            args.source,
            args.output,
            allow_undeskewed=args.allow_undeskewed,
        )
    except ConversionError as exc:
        raise SystemExit(f"conversion failed: {exc}") from exc
    print(
        json.dumps(
            {
                "output": str(args.output),
                "records": stats.records,
                "cloud_records": stats.cloud_records,
                "imu_records": stats.imu_records,
                "points": stats.points,
                "undeskewed_points": stats.undeskewed_points,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

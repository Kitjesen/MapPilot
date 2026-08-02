#!/usr/bin/env python3
"""Convert ROS 2 LiDAR/IMU bags into LingTu normalized replay JSONL.

This module intentionally has no import-time ROS dependency. The ROS 2 bag
reader is loaded lazily by ``convert_manifest`` so the parsing and validation
logic can be tested in normal development environments.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import tempfile
from collections.abc import Iterable, Iterator, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, NamedTuple

REPLAY_SCHEMA = "lingtu.slam.replay.v1"
EXPECTED_ADAPTER = "ros2_bag_to_normalized_jsonl"
SUPPORTED_IMU_TYPES = {"sensor_msgs/msg/Imu", "sensor_msgs/Imu"}
SUPPORTED_POINTCLOUD2_TYPES = {"sensor_msgs/msg/PointCloud2", "sensor_msgs/PointCloud2"}
SUPPORTED_LIVOX_TYPES = {
    "livox_ros_driver2/msg/CustomMsg",
    "livox_ros_driver2/CustomMsg",
}

POINT_TIME_UNITS = {"ns", "us", "s"}
UINT32_MAX = (1 << 32) - 1


class ConversionError(ValueError):
    """Raised when a bag cannot be normalized safely."""


class ConversionStats(NamedTuple):
    records: int
    cloud_records: int
    imu_records: int
    points: int
    undeskewed_cloud_records: int
    undeskewed_points: int


@dataclass(frozen=True)
class StreamSpec:
    topic: str
    message_type: str
    point_time_field: str | None = None
    point_time_unit: str | None = None


@dataclass(frozen=True)
class ManifestSpec:
    source_path: Path
    lidar: StreamSpec
    imu: StreamSpec


@dataclass(frozen=True)
class BagRecord:
    topic: str
    message_type: str
    payload: Any
    timestamp_ns: int


_POINT_FIELD_FORMATS: dict[int, tuple[str, int]] = {
    1: ("b", 1),  # INT8
    2: ("B", 1),  # UINT8
    3: ("h", 2),  # INT16
    4: ("H", 2),  # UINT16
    5: ("i", 4),  # INT32
    6: ("I", 4),  # UINT32
    7: ("f", 4),  # FLOAT32
    8: ("d", 8),  # FLOAT64
}


def _mapping(value: Any, *, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ConversionError(f"{label} must be a JSON object")
    return value


def _clean_message_type(message_type: Any) -> str:
    return str(message_type or "").strip()


def _header_timestamp_ns(message: Any, fallback_ns: int) -> int:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return fallback_ns
    return int(sec) * 1_000_000_000 + int(nanosec)


def _livox_timebase_ns(message: Any, fallback_ns: int) -> int:
    timebase = getattr(message, "timebase", None)
    if timebase is None:
        return _header_timestamp_ns(message, fallback_ns)
    if isinstance(timebase, bool) or not isinstance(timebase, int):
        raise ConversionError("Livox CustomMsg timebase must be an integer nanosecond timestamp")
    if timebase < 0:
        raise ConversionError("Livox CustomMsg timebase must be non-negative")
    return timebase


def _finite_float(value: Any, *, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConversionError(f"{label} must be numeric")
    number = float(value)
    if not math.isfinite(number):
        raise ConversionError(f"{label} must be finite")
    return number


def _vector3(value: Any, *, label: str) -> list[float]:
    return [
        _finite_float(getattr(value, "x", None), label=f"{label}.x"),
        _finite_float(getattr(value, "y", None), label=f"{label}.y"),
        _finite_float(getattr(value, "z", None), label=f"{label}.z"),
    ]


def _stream_from_manifest(data: Mapping[str, Any], *, name: str) -> StreamSpec:
    streams = _mapping(data.get("streams"), label="streams")
    raw_stream = _mapping(streams.get(name), label=f"streams.{name}")
    point_time = raw_stream.get("point_time") or {}
    if point_time is not None and not isinstance(point_time, Mapping):
        raise ConversionError(f"streams.{name}.point_time must be a JSON object")
    return StreamSpec(
        topic=str(raw_stream.get("source_topic", "")),
        message_type=_clean_message_type(raw_stream.get("source_message_type")),
        point_time_field=(
            str(point_time["field"])
            if isinstance(point_time, Mapping) and point_time.get("field") is not None
            else None
        ),
        point_time_unit=(
            str(point_time["unit"]) if isinstance(point_time, Mapping) and point_time.get("unit") is not None else None
        ),
    )


def load_manifest(path: str | Path) -> ManifestSpec:
    manifest_path = Path(path)
    try:
        with manifest_path.open("r", encoding="utf-8") as handle:
            data = _mapping(json.load(handle), label="manifest")
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ConversionError(f"failed to read manifest: {exc}") from exc

    schema_version = str(data.get("schema_version", ""))
    if schema_version != REPLAY_SCHEMA:
        raise ConversionError(f"schema_version must be {REPLAY_SCHEMA}, got {schema_version or '<empty>'}")

    source = _mapping(data.get("source"), label="source")
    source_format = str(source.get("format", "")).strip()
    if source_format != "ros2_bag":
        raise ConversionError(f"source.format must be 'ros2_bag', got {source_format or '<empty>'}")
    adapter = str(source.get("conversion_adapter", "")).strip()
    if adapter != EXPECTED_ADAPTER:
        raise ConversionError(f"source.conversion_adapter must be {EXPECTED_ADAPTER!r}, got {adapter or '<empty>'!r}")
    raw_source_path = str(source.get("path", "")).strip()
    if not raw_source_path:
        raise ConversionError("source.path is required")
    source_path = Path(raw_source_path)

    lidar = _stream_from_manifest(data, name="lidar")
    imu = _stream_from_manifest(data, name="imu")
    if not lidar.topic:
        raise ConversionError("streams.lidar.source_topic is required")
    if not imu.topic:
        raise ConversionError("streams.imu.source_topic is required")
    if lidar.message_type not in SUPPORTED_POINTCLOUD2_TYPES | SUPPORTED_LIVOX_TYPES:
        raise ConversionError(f"unsupported lidar message type: {lidar.message_type or '<empty>'}")
    if imu.message_type not in SUPPORTED_IMU_TYPES:
        raise ConversionError(f"unsupported imu message type: {imu.message_type or '<empty>'}")
    if lidar.message_type in SUPPORTED_POINTCLOUD2_TYPES:
        if lidar.point_time_field and lidar.point_time_unit not in POINT_TIME_UNITS:
            raise ConversionError("streams.lidar.point_time.unit must be one of ns, us, s")
    if lidar.message_type in SUPPORTED_LIVOX_TYPES:
        if lidar.point_time_field != "offset_time" or lidar.point_time_unit not in POINT_TIME_UNITS:
            raise ConversionError(
                "livox_ros_driver2/msg/CustomMsg requires streams.lidar.point_time "
                "field=offset_time and unit one of ns, us, s"
            )

    return ManifestSpec(source_path=source_path, lidar=lidar, imu=imu)


def _field_attr(field: Any, name: str, default: Any = None) -> Any:
    if isinstance(field, Mapping):
        return field.get(name, default)
    return getattr(field, name, default)


def _point_field_map(fields: Iterable[Any]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for field in fields:
        name = str(_field_attr(field, "name", ""))
        if name:
            result[name] = field
    return result


def _validate_point_fields(fields: Mapping[str, Any], *, point_step: int) -> None:
    for name, field in fields.items():
        datatype = int(_field_attr(field, "datatype"))
        count = int(_field_attr(field, "count", 1))
        offset = int(_field_attr(field, "offset"))
        if count <= 0:
            raise ConversionError(f"PointCloud2 field {name} count must be positive")
        if datatype not in _POINT_FIELD_FORMATS:
            raise ConversionError(f"PointCloud2 field {name} uses unsupported datatype {datatype}")
        _fmt, size = _POINT_FIELD_FORMATS[datatype]
        if offset < 0:
            raise ConversionError(f"PointCloud2 field {name} offset must be non-negative")
        if offset + size * count > point_step:
            raise ConversionError(f"PointCloud2 field {name} exceeds point_step")


def _field_value(
    data: bytes | bytearray | memoryview,
    base_offset: int,
    field: Any,
    *,
    endian: str,
    label: str,
) -> Any:
    datatype = int(_field_attr(field, "datatype"))
    count = int(_field_attr(field, "count", 1))
    offset = int(_field_attr(field, "offset"))
    if datatype not in _POINT_FIELD_FORMATS:
        raise ConversionError(f"{label} uses unsupported PointField datatype {datatype}")
    fmt, _size = _POINT_FIELD_FORMATS[datatype]
    absolute = base_offset + offset
    try:
        if count == 1:
            return struct.unpack_from(endian + fmt, data, absolute)[0]
        return struct.unpack_from(endian + (fmt * count), data, absolute)
    except struct.error as exc:
        raise ConversionError(f"{label} failed to unpack PointCloud2 field: {exc}") from exc


def _point_time_to_ns(value: Any, *, unit: str, label: str) -> int:
    number = _finite_float(value, label=label)
    if unit == "ns":
        offset = int(number)
    elif unit == "us":
        offset = round(number * 1_000)
    elif unit == "s":
        offset = round(number * 1_000_000_000)
    else:
        raise ConversionError(f"{label} unit must be one of ns, us, s")
    if offset < 0 or offset > UINT32_MAX:
        raise ConversionError(f"{label} must fit uint32 nanoseconds")
    return offset


def normalize_imu(message: Any, *, timestamp_ns: int, sequence: int) -> dict[str, Any]:
    return {
        "type": "imu",
        "timestamp_ns": _header_timestamp_ns(message, timestamp_ns),
        "sequence": sequence,
        "gyro": _vector3(getattr(message, "angular_velocity", None), label="imu.angular_velocity"),
        "acc": _vector3(getattr(message, "linear_acceleration", None), label="imu.linear_acceleration"),
    }


def normalize_pointcloud2(
    message: Any,
    *,
    timestamp_ns: int,
    sequence: int,
    point_time_field: str | None,
    point_time_unit: str | None,
    allow_undeskewed: bool,
) -> tuple[dict[str, Any], int, bool]:
    fields = _point_field_map(getattr(message, "fields", ()))
    for required in ("x", "y", "z"):
        if required not in fields:
            raise ConversionError(f"PointCloud2 missing required field {required}")

    if point_time_field and point_time_field not in fields:
        raise ConversionError(f"PointCloud2 missing point time field {point_time_field}")
    if not point_time_field and not allow_undeskewed:
        raise ConversionError(
            "PointCloud2 point time field is required; use --allow-undeskewed for transport-only replay"
        )

    point_step = int(getattr(message, "point_step", 0))
    row_step = int(getattr(message, "row_step", 0))
    width = int(getattr(message, "width", 0))
    height = int(getattr(message, "height", 1) or 1)
    if point_step <= 0:
        raise ConversionError("PointCloud2 point_step must be positive")
    if row_step < point_step * width:
        raise ConversionError("PointCloud2 row_step is smaller than point_step * width")
    _validate_point_fields(fields, point_step=point_step)

    data = bytes(getattr(message, "data", b""))
    required_bytes = row_step * height
    if len(data) < required_bytes:
        raise ConversionError("PointCloud2 data is shorter than row_step * height")

    endian = ">" if bool(getattr(message, "is_bigendian", False)) else "<"
    intensity_field = fields.get("intensity") or fields.get("reflectivity")
    tag_field = fields.get("tag")
    line_field = fields.get("line") or fields.get("ring")
    flags_field = fields.get("flags")

    points: list[dict[str, Any]] = []
    undeskewed = False
    previous_offset = -1
    for row in range(height):
        row_base = row * row_step
        for col in range(width):
            point_base = row_base + col * point_step
            point: dict[str, Any] = {
                "x": _finite_float(
                    _field_value(data, point_base, fields["x"], endian=endian, label="PointCloud2.x"),
                    label="x",
                ),
                "y": _finite_float(
                    _field_value(data, point_base, fields["y"], endian=endian, label="PointCloud2.y"),
                    label="y",
                ),
                "z": _finite_float(
                    _field_value(data, point_base, fields["z"], endian=endian, label="PointCloud2.z"),
                    label="z",
                ),
            }
            if intensity_field is not None:
                point["intensity"] = _finite_float(
                    _field_value(data, point_base, intensity_field, endian=endian, label="PointCloud2.intensity"),
                    label="intensity",
                )
            if point_time_field:
                offset_ns = _point_time_to_ns(
                    _field_value(data, point_base, fields[point_time_field], endian=endian, label=point_time_field),
                    unit=point_time_unit or "ns",
                    label=f"PointCloud2.{point_time_field}",
                )
            else:
                offset_ns = 0
                undeskewed = True
            if offset_ns < previous_offset:
                raise ConversionError("PointCloud2 point offsets must be monotonic within the cloud")
            previous_offset = offset_ns
            point["offset_time_ns"] = offset_ns

            if tag_field is not None:
                point["tag"] = int(_field_value(data, point_base, tag_field, endian=endian, label="PointCloud2.tag"))
            if line_field is not None:
                point["line"] = int(_field_value(data, point_base, line_field, endian=endian, label="PointCloud2.line"))
            if flags_field is not None:
                point["flags"] = int(
                    _field_value(data, point_base, flags_field, endian=endian, label="PointCloud2.flags")
                )
            points.append(point)

    return (
        {
            "type": "cloud",
            "timestamp_ns": _header_timestamp_ns(message, timestamp_ns),
            "sequence": sequence,
            "points": points,
            "transport_only": undeskewed,
        },
        len(points),
        undeskewed,
    )


def normalize_livox_custom_msg(
    message: Any,
    *,
    timestamp_ns: int,
    sequence: int,
    point_time_field: str | None,
    point_time_unit: str | None,
    allow_undeskewed: bool,
) -> tuple[dict[str, Any], int, bool]:
    if point_time_field != "offset_time" or point_time_unit not in POINT_TIME_UNITS:
        raise ConversionError("Livox CustomMsg requires point_time_field=offset_time and point_time_unit in ns, us, s")
    points: list[dict[str, Any]] = []
    previous_offset = -1
    undeskewed = False
    for index, raw_point in enumerate(getattr(message, "points", ())):
        offset_value = getattr(raw_point, point_time_field, None)
        if offset_value is None:
            if not allow_undeskewed:
                raise ConversionError(f"Livox CustomMsg point {point_time_field} is required")
            offset_ns = 0
            undeskewed = True
        else:
            offset_ns = _point_time_to_ns(
                offset_value,
                unit=point_time_unit,
                label=f"Livox point {index} {point_time_field}",
            )
        if offset_ns < previous_offset:
            raise ConversionError("Livox CustomMsg point offsets must be monotonic within the cloud")
        previous_offset = offset_ns
        points.append(
            {
                "x": _finite_float(getattr(raw_point, "x", None), label=f"Livox point {index}.x"),
                "y": _finite_float(getattr(raw_point, "y", None), label=f"Livox point {index}.y"),
                "z": _finite_float(getattr(raw_point, "z", None), label=f"Livox point {index}.z"),
                "intensity": _finite_float(
                    getattr(raw_point, "reflectivity", getattr(raw_point, "intensity", 0.0)),
                    label=f"Livox point {index}.reflectivity",
                ),
                "offset_time_ns": offset_ns,
                "tag": int(getattr(raw_point, "tag", 0)),
                "line": int(getattr(raw_point, "line", 0)),
            }
        )
    return (
        {
            "type": "cloud",
            "timestamp_ns": _livox_timebase_ns(message, timestamp_ns),
            "sequence": sequence,
            "points": points,
            "transport_only": undeskewed,
        },
        len(points),
        undeskewed,
    )


def _normalize_record(
    record: BagRecord,
    manifest: ManifestSpec,
    *,
    sequence: int,
    allow_undeskewed: bool,
) -> tuple[dict[str, Any], int, bool] | None:
    if record.topic == manifest.imu.topic:
        message_type = _clean_message_type(record.message_type) or manifest.imu.message_type
        if message_type not in SUPPORTED_IMU_TYPES:
            raise ConversionError(f"unexpected IMU message type on {record.topic}: {message_type}")
        return normalize_imu(record.payload, timestamp_ns=record.timestamp_ns, sequence=sequence), 0, False

    if record.topic == manifest.lidar.topic:
        message_type = _clean_message_type(record.message_type) or manifest.lidar.message_type
        manifest_is_pointcloud2 = manifest.lidar.message_type in SUPPORTED_POINTCLOUD2_TYPES
        actual_is_pointcloud2 = message_type in SUPPORTED_POINTCLOUD2_TYPES
        manifest_is_livox = manifest.lidar.message_type in SUPPORTED_LIVOX_TYPES
        actual_is_livox = message_type in SUPPORTED_LIVOX_TYPES
        if (manifest_is_pointcloud2 and not actual_is_pointcloud2) or (manifest_is_livox and not actual_is_livox):
            raise ConversionError(
                f"actual LiDAR message type {message_type} does not match manifest {manifest.lidar.message_type}"
            )
        if message_type in SUPPORTED_POINTCLOUD2_TYPES:
            return normalize_pointcloud2(
                record.payload,
                timestamp_ns=record.timestamp_ns,
                sequence=sequence,
                point_time_field=manifest.lidar.point_time_field,
                point_time_unit=manifest.lidar.point_time_unit,
                allow_undeskewed=allow_undeskewed,
            )
        if message_type in SUPPORTED_LIVOX_TYPES:
            return normalize_livox_custom_msg(
                record.payload,
                timestamp_ns=record.timestamp_ns,
                sequence=sequence,
                point_time_field=manifest.lidar.point_time_field,
                point_time_unit=manifest.lidar.point_time_unit,
                allow_undeskewed=allow_undeskewed,
            )
        raise ConversionError(f"unexpected LiDAR message type on {record.topic}: {message_type}")
    return None


def _write_jsonl_atomically(
    records: Iterable[BagRecord],
    output_path: str | Path,
    manifest: ManifestSpec,
    *,
    allow_undeskewed: bool = False,
) -> ConversionStats:
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)

    records_seen = 0
    cloud_records = 0
    imu_records = 0
    points = 0
    undeskewed_cloud_records = 0
    undeskewed_points = 0
    previous_timestamp_ns: int | None = None
    temp_path: Path | None = None

    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=output.parent,
            prefix=f".{output.name}.",
            suffix=".tmp",
            delete=False,
        ) as target:
            temp_path = Path(target.name)
            for record in records:
                normalized = _normalize_record(
                    record,
                    manifest,
                    sequence=records_seen,
                    allow_undeskewed=allow_undeskewed,
                )
                if normalized is None:
                    continue
                payload, point_count, undeskewed = normalized
                timestamp_ns = int(payload["timestamp_ns"])
                if previous_timestamp_ns is not None and timestamp_ns < previous_timestamp_ns:
                    raise ConversionError("normalized record timestamps must be monotonic")
                previous_timestamp_ns = timestamp_ns

                if payload["type"] == "imu":
                    imu_records += 1
                elif payload["type"] == "cloud":
                    cloud_records += 1
                    points += point_count
                    if undeskewed:
                        undeskewed_cloud_records += 1
                        undeskewed_points += point_count
                else:
                    raise ConversionError(f"unsupported normalized record type: {payload['type']}")

                target.write(json.dumps(payload, sort_keys=True, separators=(",", ":")))
                target.write("\n")
                records_seen += 1

            if cloud_records == 0:
                raise ConversionError("replay must contain at least one cloud record")
            if imu_records == 0:
                raise ConversionError("replay must contain at least one IMU record")
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
        records=records_seen,
        cloud_records=cloud_records,
        imu_records=imu_records,
        points=points,
        undeskewed_cloud_records=undeskewed_cloud_records,
        undeskewed_points=undeskewed_points,
    )


def _load_ros2_bag_dependencies() -> tuple[Any, Any, Any]:
    try:
        import rosbag2_py  # type: ignore[import-not-found]
        from rclpy.serialization import deserialize_message  # type: ignore[import-not-found]
        from rosidl_runtime_py.utilities import get_message  # type: ignore[import-not-found]
    except ImportError as exc:
        raise ConversionError(
            "ROS 2 bag conversion requires rosbag2_py, rclpy, and rosidl_runtime_py. "
            "Run this adapter inside a ROS 2 environment or pre-normalize the bag offline."
        ) from exc
    return rosbag2_py, deserialize_message, get_message


def iter_ros2_bag_records(
    source_path: str | Path,
    topics: set[str],
    *,
    storage_id: str = "sqlite3",
) -> Iterator[BagRecord]:
    rosbag2_py, deserialize_message, get_message = _load_ros2_bag_dependencies()
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(source_path), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)

    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_classes: dict[str, Any] = {}
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic not in topics:
            continue
        message_type = topic_types.get(topic, "")
        if topic not in message_classes:
            message_classes[topic] = get_message(message_type)
        yield BagRecord(
            topic=topic,
            message_type=message_type,
            payload=deserialize_message(serialized, message_classes[topic]),
            timestamp_ns=int(timestamp_ns),
        )


def convert_manifest(
    manifest_path: str | Path,
    output_path: str | Path,
    *,
    allow_undeskewed: bool = False,
    storage_id: str = "sqlite3",
) -> ConversionStats:
    manifest = load_manifest(manifest_path)
    if not manifest.source_path.exists():
        raise ConversionError(f"source path does not exist: {manifest.source_path}")
    topics = {manifest.lidar.topic, manifest.imu.topic}
    return _write_jsonl_atomically(
        iter_ros2_bag_records(manifest.source_path, topics, storage_id=storage_id),
        output_path,
        manifest,
        allow_undeskewed=allow_undeskewed,
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Convert a ROS 2 LiDAR/IMU bag manifest into normalized JSONL.")
    parser.add_argument("manifest", type=Path, help="lingtu.slam.replay.v1 manifest")
    parser.add_argument("output", type=Path, help="normalized JSONL output")
    parser.add_argument("--storage-id", default="sqlite3", help="rosbag2 storage id, for example sqlite3 or mcap")
    parser.add_argument(
        "--allow-undeskewed",
        action="store_true",
        help=(
            "allow clouds without per-point timing by writing zero offsets; "
            "output is transport-only and not frontend accuracy evidence"
        ),
    )
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    try:
        stats = convert_manifest(
            args.manifest,
            args.output,
            allow_undeskewed=args.allow_undeskewed,
            storage_id=args.storage_id,
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
                "undeskewed_cloud_records": stats.undeskewed_cloud_records,
                "undeskewed_points": stats.undeskewed_points,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

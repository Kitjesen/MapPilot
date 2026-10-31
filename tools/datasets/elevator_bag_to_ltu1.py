"""Convert official Elevator-LIO ROS1 bags to native LTU1 without ROS.

Run as ``python -m tools.datasets.elevator_bag_to_ltu1 BAG OUTPUT_DIR``.
The standard library reads ROS1 bag V2 records and none/bz2 chunks; lz4 uses an
already installed system liblz4 through ctypes. NumPy copies
documented Livox point fields without filtering or constructing per-point JSON.
"""

from __future__ import annotations

import argparse
import bz2
import ctypes
import ctypes.util
import io
import json
import math
import struct
import sys
from collections import Counter
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any, BinaryIO, Iterator

import numpy as np
from tools.datasets.normalized_lidar_imu_to_ltu1 import HEADER, IMU, MAGIC, POINT

LIDAR_TOPIC = "/livox/lidar"
IMU_TOPIC = "/livox/imu"
EVENT_TOPIC = "/LIO/set_elevator_flag"
DATASET_URL = "https://huggingface.co/datasets/xiaofan0100/Elevator-LIO-Dataset"
DATASET_REVISION = "972a33f0f02a9a04c283bcf664b33bed989ee2cf"
LIVOX_POINT = np.dtype(
    [("offset_time", "<u4"), ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
     ("reflectivity", "u1"), ("tag", "u1"), ("line", "u1")], align=False,
)
LTU_POINT = np.dtype(
    [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"),
     ("offset_time_ns", "<u4"), ("tag", "u1"), ("line", "u1"), ("flags", "<u2")],
    align=False,
)
assert LIVOX_POINT.itemsize == 19 and LTU_POINT.itemsize == POINT.size


@lru_cache(maxsize=1)
def _lz4_library() -> ctypes.CDLL:
    """Load the host library without adding a Python or ROS dependency."""
    name = ctypes.util.find_library("lz4")
    if not name and sys.platform == "win32":
        bundled = Path(sys.prefix) / "Library" / "bin" / "liblz4.dll"
        if bundled.is_file():
            name = str(bundled)
    if not name:
        raise RuntimeError("lz4 ROS1 chunks require an installed system liblz4")
    library = ctypes.CDLL(name)
    library.LZ4F_isError.argtypes = [ctypes.c_size_t]
    library.LZ4F_isError.restype = ctypes.c_uint
    library.LZ4F_getErrorName.argtypes = [ctypes.c_size_t]
    library.LZ4F_getErrorName.restype = ctypes.c_char_p
    library.LZ4F_createDecompressionContext.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_uint]
    library.LZ4F_createDecompressionContext.restype = ctypes.c_size_t
    library.LZ4F_freeDecompressionContext.argtypes = [ctypes.c_void_p]
    library.LZ4F_freeDecompressionContext.restype = ctypes.c_size_t
    library.LZ4F_decompress.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_size_t),
                                      ctypes.c_void_p, ctypes.POINTER(ctypes.c_size_t), ctypes.c_void_p]
    library.LZ4F_decompress.restype = ctypes.c_size_t
    return library


def _decompress_lz4(payload: bytes, expected_size: int) -> bytes:
    library = _lz4_library()
    context = ctypes.c_void_p()
    result = library.LZ4F_createDecompressionContext(ctypes.byref(context), 100)
    if library.LZ4F_isError(result):
        raise ValueError(library.LZ4F_getErrorName(result).decode())
    try:
        output = ctypes.create_string_buffer(expected_size)
        size_out = ctypes.c_size_t(expected_size)
        size_in = ctypes.c_size_t(len(payload))
        result = library.LZ4F_decompress(context, output, ctypes.byref(size_out), payload,
                                         ctypes.byref(size_in), None)
        if library.LZ4F_isError(result):
            raise ValueError("invalid lz4 bag chunk: " + library.LZ4F_getErrorName(result).decode())
        if result != 0 or size_in.value != len(payload) or size_out.value != expected_size:
            raise ValueError("incomplete lz4 bag chunk or uncompressed length mismatch")
        return output.raw
    finally:
        library.LZ4F_freeDecompressionContext(context)


def _exact(stream: BinaryIO, count: int) -> bytes:
    data = stream.read(count)
    if len(data) != count:
        raise ValueError("truncated ROS1 bag record")
    return data


def _fields(raw: bytes) -> dict[str, bytes]:
    fields = {}
    cursor = 0
    while cursor < len(raw):
        length = struct.unpack_from("<I", raw, cursor)[0]
        cursor += 4
        end = cursor + length
        if end > len(raw):
            raise ValueError("truncated ROS1 bag header field")
        key, value = raw[cursor:end].split(b"=", 1)
        fields[key.decode("ascii")] = value
        cursor = end
    return fields


def _record_header(stream: BinaryIO) -> tuple[dict[str, bytes], int] | None:
    size = stream.read(4)
    if not size:
        return None
    if len(size) != 4:
        raise ValueError("truncated ROS1 bag record length")
    header = _fields(_exact(stream, struct.unpack("<I", size)[0]))
    return header, struct.unpack("<I", _exact(stream, 4))[0]


@dataclass
class Connection:
    id: int
    topic: str
    msgtype: str
    definition: str
    msgcount: int = 0


class Ros1Bag:
    """Reader for the bag V2 layout used by the official elevator recordings."""

    def __init__(self, path: Path):
        self.path = path
        self.connections: dict[int, Connection] = {}
        self.compression_counts: Counter[str] = Counter()
        counts: Counter[int] = Counter()
        with path.open("rb") as source:
            self._magic(source)
            while item := _record_header(source):
                header, size = item
                op = header["op"][0]
                if op == 7:
                    self._connection(header, _exact(source, size))
                else:
                    if op == 5:
                        self.compression_counts[header["compression"].decode()] += 1
                    elif op == 4:
                        counts[struct.unpack("<I", header["conn"])[0]] += struct.unpack("<I", header["count"])[0]
                    source.seek(size, 1)
        for key, count in counts.items():
            if key in self.connections:
                self.connections[key].msgcount = count
        unsupported = set(self.compression_counts) - {"none", "bz2", "lz4"}
        if unsupported:
            raise ValueError(f"unsupported bag chunk compression: {sorted(unsupported)}")

    @staticmethod
    def _magic(source: BinaryIO) -> None:
        if source.readline() != b"#ROSBAG V2.0\n":
            raise ValueError("expected ROS1 bag V2.0")

    def _connection(self, header: dict[str, bytes], payload: bytes) -> None:
        key = struct.unpack("<I", header["conn"])[0]
        fields = _fields(payload)
        previous = self.connections.get(key)
        self.connections[key] = Connection(
            key, header["topic"].decode(), fields["type"].decode(),
            fields["message_definition"].decode(), previous.msgcount if previous else 0,
        )

    def _messages(self, source: BinaryIO) -> Iterator[tuple[Connection, int, bytes]]:
        while item := _record_header(source):
            header, size = item
            payload = _exact(source, size)
            op = header["op"][0]
            if op == 7:
                self._connection(header, payload)
            elif op == 2:
                seconds, nanoseconds = struct.unpack("<II", header["time"])
                yield self.connections[struct.unpack("<I", header["conn"])[0]], seconds * 1_000_000_000 + nanoseconds, payload
            elif op == 5:
                compression = header["compression"].decode()
                if compression == "bz2":
                    payload = bz2.decompress(payload)
                elif compression == "lz4":
                    payload = _decompress_lz4(payload, struct.unpack("<I", header["size"])[0])
                if len(payload) != struct.unpack("<I", header["size"])[0]:
                    raise ValueError("bag chunk uncompressed length mismatch")
                yield from self._messages(io.BytesIO(payload))

    def messages(self) -> Iterator[tuple[Connection, int, bytes]]:
        with self.path.open("rb") as source:
            self._magic(source)
            yield from self._messages(source)


def ros1_header(raw: bytes | memoryview) -> tuple[dict[str, Any], int]:
    """Read the ROS1 Header, retaining its exact integer sensor timestamp."""
    seq, seconds, nanoseconds, length = struct.unpack_from("<IIII", raw)
    end = 16 + length
    if end > len(raw) or nanoseconds >= 1_000_000_000:
        raise ValueError("invalid ROS1 sensor Header")
    return {
        "header_sequence": seq,
        "header_timestamp_ns": seconds * 1_000_000_000 + nanoseconds,
        "frame_id": bytes(raw[16:end]).decode("utf-8"),
    }, end


def livox_payload(raw: bytes | memoryview) -> tuple[dict[str, Any], bytes, np.ndarray]:
    header, offset = ros1_header(raw)
    timebase, point_num, lidar_id, r0, r1, r2, array_count = struct.unpack_from(
        "<QIBBBBI", raw, offset,
    )
    offset += 20
    if point_num != array_count or len(raw) != offset + array_count * LIVOX_POINT.itemsize:
        raise ValueError("Livox point_num/array/payload length mismatch")
    points = np.frombuffer(raw, dtype=LIVOX_POINT, count=array_count, offset=offset)
    output = np.zeros(array_count, dtype=LTU_POINT)
    for name in ("x", "y", "z", "tag", "line"):
        output[name] = points[name]
    output["offset_time_ns"] = points["offset_time"]
    output["intensity"] = points["reflectivity"]
    header.update(timebase_ns=timebase, lidar_id=lidar_id, rsvd=[r0, r1, r2], points=point_num)
    # CustomPoint has no separate flags field. Its original tag/line are retained.
    return header, output.tobytes(), points


def imu_payload(raw: bytes | memoryview) -> tuple[dict[str, Any], bytes, tuple[float, ...]]:
    header, offset = ros1_header(raw)
    if len(raw) != offset + 296:
        raise ValueError("sensor_msgs/Imu payload length mismatch")
    gyro = struct.unpack_from("<ddd", raw, offset + 104)
    acceleration = struct.unpack_from("<ddd", raw, offset + 200)
    values = (*gyro, *acceleration)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("nonfinite IMU sample cannot be replayed")
    return header, IMU.pack(*values), values


def convert_bag(source: Path, output_dir: Path) -> dict[str, Any]:
    bag = Ros1Bag(source)
    output_dir.mkdir(parents=True, exist_ok=True)
    stream_path = output_dir / f"{source.stem}.ltu"
    index_path = output_dir / f"{source.stem}.records.jsonl"
    events_path = output_dir / f"{source.stem}.events.tsv"
    report_path = output_dir / f"{source.stem}.manifest.json"
    for path in (stream_path, index_path, events_path, report_path):
        if path.exists():
            raise FileExistsError(f"output already exists: {path}")

    counts: Counter[str] = Counter()
    tag_counts = np.zeros(256, dtype=np.int64)
    line_counts = np.zeros(256, dtype=np.int64)
    acceleration_norms: list[float] = []
    frame_ids: dict[str, set[str]] = {LIDAR_TOPIC: set(), IMU_TOPIC: set()}
    previous_stamps: dict[str, int] = {}
    regressions: Counter[str] = Counter()
    first_stamp: int | None = None
    last_stamp = 0
    offset_min: int | None = None
    offset_max = 0
    timebase_deltas: list[int] = []
    events: list[dict[str, Any]] = []
    max_imu_float32_error = 0.0

    with stream_path.open("wb") as stream, index_path.open("w", encoding="utf-8") as index, events_path.open("w", encoding="utf-8") as event_file:
        for connection, bag_stamp, raw in bag.messages():
            topic = connection.topic
            if topic not in {LIDAR_TOPIC, IMU_TOPIC, EVENT_TOPIC}:
                continue
            if topic == EVENT_TOPIC:
                if len(raw) != 1 or raw[0] not in {0, 1}:
                    raise ValueError("elevator flag is not a ROS1 std_msgs/Bool")
                event = {"timestamp_ns": bag_stamp, "active": bool(raw[0]), "topic": topic}
                events.append(event)
                event_file.write(f"{bag_stamp}\t{int(raw[0])}\n")
                counts[topic] += 1
                continue
            if topic == LIDAR_TOPIC:
                if not connection.msgtype.endswith("/CustomMsg"):
                    raise ValueError(f"unexpected LiDAR type: {connection.msgtype}")
                info, payload, points = livox_payload(raw)
                record_type, count = 1, len(points)
                counts["points"] += count
                tag_counts += np.bincount(points["tag"], minlength=256)
                line_counts += np.bincount(points["line"], minlength=256)
                if count:
                    low, high = int(points["offset_time"].min()), int(points["offset_time"].max())
                    offset_min = low if offset_min is None else min(offset_min, low)
                    offset_max = max(offset_max, high)
                    if np.any(points["offset_time"][1:] < points["offset_time"][:-1]):
                        counts["clouds_with_unsorted_offsets"] += 1
                timebase_deltas.append(info["timebase_ns"] - info["header_timestamp_ns"])
            else:
                if connection.msgtype != "sensor_msgs/Imu":
                    raise ValueError(f"unexpected IMU type: {connection.msgtype}")
                info, payload, values = imu_payload(raw)
                record_type, count = 2, 1
                max_imu_float32_error = max(max_imu_float32_error, max(abs(a-b) for a, b in zip(values, IMU.unpack(payload), strict=True)))
                if len(acceleration_norms) < 200:
                    acceleration_norms.append(math.sqrt(sum(value * value for value in values[3:])))
            stamp = info["header_timestamp_ns"]
            if topic in previous_stamps and stamp < previous_stamps[topic]:
                regressions[topic] += 1
            previous_stamps[topic] = stamp
            frame_ids[topic].add(info["frame_id"])
            first_stamp = stamp if first_stamp is None else min(first_stamp, stamp)
            last_stamp = max(last_stamp, stamp)
            sequence = counts["records"]
            stream.write(HEADER.pack(MAGIC, record_type, stamp, sequence, count, len(payload)))
            stream.write(payload)
            index.write(json.dumps({"sequence": sequence, "topic": topic, "bag_timestamp_ns": bag_stamp, **info}, separators=(",", ":")) + "\n")
            counts["records"] += 1
            counts[topic] += 1
    if not counts[LIDAR_TOPIC] or not counts[IMU_TOPIC]:
        raise ValueError("bag requires /livox/lidar and /livox/imu messages")
    connection_records = [
        {"topic": connection.topic, "type": connection.msgtype, "count": counts[connection.topic],
         "indexed_message_count": connection.msgcount, "definition": connection.definition}
        for connection in bag.connections.values()
    ]
    for connection in bag.connections.values():
        if connection.topic in {LIDAR_TOPIC, IMU_TOPIC, EVENT_TOPIC} and connection.msgcount and counts[connection.topic] != connection.msgcount:
            raise ValueError(f"bag index/message count mismatch on {connection.topic}")
    report = {
        "schema": "lingtu.elevator_dataset_conversion.v1",
        "source": str(source.resolve()), "source_bytes": source.stat().st_size,
        "dataset_url": DATASET_URL, "dataset_revision": DATASET_REVISION,
        "license": "CC BY 4.0", "license_url": "https://creativecommons.org/licenses/by/4.0/",
        "conversion": "raw ROS1 fields -> LTU1; no point filter, rescale, restamp or synthetic events",
        "connections": connection_records, "counts": dict(counts),
        "chunk_compression_counts": dict(bag.compression_counts),
        "frame_ids": {topic: sorted(values) for topic, values in frame_ids.items()},
        "first_sensor_timestamp_ns": first_stamp, "last_sensor_timestamp_ns": last_stamp,
        "duration_s": (last_stamp - first_stamp) * 1e-9 if first_stamp else 0.0,
        "ordering": "bag record order; each LTU1 record retains its sensor header timestamp",
        "per_topic_stamp_regressions": dict(regressions),
        "point_offset_unit": "ns", "point_offset_min": offset_min, "point_offset_max": offset_max,
        "tag_counts": {str(i): int(n) for i, n in enumerate(tag_counts) if n},
        "line_counts": {str(i): int(n) for i, n in enumerate(line_counts) if n},
        "flags": "0: source CustomPoint has no separate flags; tag and line copied unchanged",
        "timebase_minus_header_ns": {"min": min(timebase_deltas), "max": max(timebase_deltas)},
        "imu_units": "as recorded, no rescale; angular velocity rad/s; verify acceleration calibration against source driver",
        "first_200_imu_acc_norm_median": float(np.median(acceleration_norms)),
        "imu_float32_max_absolute_quantization_error": max_imu_float32_error,
        "recorded_manual_events": events,
        "outputs": {"ltu": str(stream_path), "index": str(index_path), "events": str(events_path)},
    }
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    print(json.dumps(convert_bag(args.bag, args.output_dir), indent=2))


if __name__ == "__main__":
    main()

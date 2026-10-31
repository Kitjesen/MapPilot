"""Convert SuperLoc Corridor02 Velodyne/IMU records for offline LTLIO tests.

This is an offline carrier adaptation, not a production Velodyne driver. All
16 rings retain their geometry and point times; the Livox-only intake's line
and tag selectors are neutralized. Images are not estimator inputs.
"""
from __future__ import annotations

import argparse
import json
import struct
from collections import Counter
from pathlib import Path

import numpy as np
from tools.datasets.elevator_bag_to_ltu1 import (
    HEADER,
    LTU_POINT,
    MAGIC,
    Ros1Bag,
    imu_payload,
    ros1_header,
)

LIDAR = "/velodyne_points"
IMU = "/imu/data"


def pointcloud_payload(raw: bytes) -> tuple[dict, bytes]:
    header, cursor = ros1_header(raw)
    height, width, count = struct.unpack_from("<III", raw, cursor)
    cursor += 12
    fields = []
    for _ in range(count):
        length = struct.unpack_from("<I", raw, cursor)[0]
        cursor += 4
        name = raw[cursor:cursor + length].decode()
        cursor += length
        offset, kind, elements = struct.unpack_from("<IBI", raw, cursor)
        cursor += 9
        fields.append((name, offset, kind, elements))
    bigendian, step, row_step, size = struct.unpack_from("<BIII", raw, cursor)
    cursor += 13
    expected = {"x": 7, "y": 7, "z": 7, "intensity": 7, "ring": 4, "time": 7}
    if ({name: kind for name, _, kind, n in fields if n == 1} != expected or
            bigendian or row_step != width * step or size != height * row_step or
            len(raw) != cursor + size + 1):
        raise ValueError("unexpected Corridor02 PointCloud2 layout")
    dtype = np.dtype(dict(names=[f[0] for f in fields],
                          formats=[{4: "<u2", 7: "<f4"}[f[2]] for f in fields],
                          offsets=[f[1] for f in fields], itemsize=step))
    points = np.frombuffer(raw, dtype=dtype, count=height * width, offset=cursor)
    seconds = points["time"].astype(np.float64)
    if not np.isfinite(seconds).all() or np.any(seconds < 0) or np.any(seconds >= 1):
        raise ValueError("Corridor02 point times must be relative seconds within one scan")
    offsets = np.rint(seconds * 1e9).astype(np.uint32)
    result = np.zeros(len(points), dtype=LTU_POINT)
    for name in ("x", "y", "z", "intensity"):
        result[name] = points[name]
    result["offset_time_ns"] = offsets
    # The production Livox intake accepts six lines. Ring identity does not
    # enter its point-to-plane residual; preserve every ring in this carrier.
    order = np.argsort(offsets, kind="stable")
    result = result[order]
    header.update(points=len(points), fields=fields,
                  ring_counts={str(k): int(v) for k, v in zip(*np.unique(points["ring"], return_counts=True))},
                  offset_min_ns=int(offsets.min()) if len(offsets) else None,
                  offset_max_ns=int(offsets.max()) if len(offsets) else None,
                  original_time_ordered=bool(np.all(offsets[1:] >= offsets[:-1])))
    return header, result.tobytes()


def convert(bag_path: Path, output: Path, seconds: float | None = None) -> dict:
    bag = Ros1Bag(bag_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    counts = Counter()
    rings = Counter()
    first = None
    last = None
    imu_norms = []
    first_cloud = None
    frames = {LIDAR: set(), IMU: set()}
    with output.open("xb") as stream:
        for connection, _, raw in bag.messages():
            if connection.topic == LIDAR:
                info, payload = pointcloud_payload(raw)
                first_cloud = first_cloud or info
                record_type, count = 1, info["points"]
            elif connection.topic == IMU:
                info, payload, values = imu_payload(raw)
                record_type, count = 2, 1
            else:
                continue
            stamp = info["header_timestamp_ns"]
            first = stamp if first is None else min(first, stamp)
            if seconds is not None and stamp > first + round(seconds * 1e9):
                break
            last = stamp if last is None else max(last, stamp)
            frames[connection.topic].add(info["frame_id"])
            stream.write(HEADER.pack(MAGIC, record_type, stamp, counts["records"], count, len(payload)))
            stream.write(payload)
            counts["records"] += 1
            counts[connection.topic] += 1
            if record_type == 1:
                counts["points"] += count
                rings.update(info["ring_counts"])
                counts["unsorted_clouds"] += not info["original_time_ordered"]
            else:
                imu_norms.append(float(np.linalg.norm(values[3:])))
    report = dict(source=str(bag_path.resolve()), output=str(output.resolve()), counts=dict(counts),
                  prefix_seconds=seconds, sensor_duration_s=(last-first)*1e-9,
                  frame_ids={k: sorted(v) for k, v in frames.items()}, first_cloud=first_cloud,
                  ring_counts=dict(rings), imu_acceleration_median_mps2=float(np.median(imu_norms)),
                  point_time="Recorded relative float32 seconds rounded to nearest integer ns; stable time sort",
                  carrier="Geometry/intensity retained; original rings recorded above; carrier line=tag=0",
                  limitation="Offline Velodyne carrier, not field support; dataset-specific extrinsics and GT frame required")
    output.with_suffix(".conversion.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seconds", type=float)
    args = parser.parse_args()
    print(json.dumps(convert(args.bag, args.output, args.seconds), indent=2))


if __name__ == "__main__":
    main()

"""Convert Hilti 2023 Robot BPearl/XSens ROS1 bags to an offline LTU1 carrier.

Run with ``python -m tools.datasets.hilti_robot_to_ltu1 BAG OUTPUT --report JSON``.
This reuses the repository's streaming ROS1 reader; no ROS dependency is needed.
"""
from __future__ import annotations

import argparse
import json
import struct
from collections import Counter
from pathlib import Path

import numpy as np
from tools.datasets.elevator_bag_to_ltu1 import LTU_POINT, Ros1Bag, imu_payload, ros1_header
from tools.datasets.normalized_lidar_imu_to_ltu1 import HEADER, MAGIC

LIDAR_TOPIC = "/rslidar_points"
IMU_TOPIC = "/imu/data"
FIELDS = {"x": (0, 7, 1), "y": (4, 7, 1), "z": (8, 7, 1),
          "intensity": (12, 7, 1), "ring": (16, 4, 1), "timestamp": (18, 8, 1)}
SOURCE_POINT = np.dtype({"names": list(FIELDS),
                         "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f8"],
                         "offsets": [0, 4, 8, 12, 16, 18], "itemsize": 26})


def seconds_to_ns(times: np.ndarray) -> np.ndarray:
    """Round represented seconds without multiplying the Unix epoch by 1e9."""
    whole = np.floor(times).astype(np.int64)
    return whole * 1_000_000_000 + np.rint((times - whole) * 1e9).astype(np.int64)


def normalized_cloud(raw: bytes) -> tuple[dict, int, np.ndarray]:
    info, cursor = ros1_header(raw)
    height, width, num_fields = struct.unpack_from("<III", raw, cursor)
    cursor += 12
    fields = {}
    for _ in range(num_fields):
        size = struct.unpack_from("<I", raw, cursor)[0]
        cursor += 4
        name = raw[cursor:cursor + size].decode()
        cursor += size
        fields[name] = struct.unpack_from("<IBI", raw, cursor)
        cursor += 9
    bigendian, step, row_step, size = struct.unpack_from("<?III", raw, cursor)
    cursor += 13
    if (fields != FIELDS or bigendian or step != 26 or row_step < width * step
            or size != row_step * height or len(raw) != cursor + size + 1):
        raise ValueError("unexpected Robot BPearl PointCloud2 schema/row layout")
    points = np.ndarray((height, width), dtype=SOURCE_POINT, buffer=raw, offset=cursor,
                        strides=(row_step, step)).reshape(-1)
    times = points["timestamp"]
    if (not len(points) or not np.isfinite(times).all() or np.any(times < 1e9)
            or np.any(times > 3e9)):
        raise ValueError("Robot points require finite absolute-seconds timestamps")
    absolute = seconds_to_ns(times)
    start, end = int(absolute.min()), int(absolute.max())
    if end - start >= 2**32:
        raise ValueError("Robot scan exceeds LTU1 uint32 nanosecond offsets")
    if np.any(points["ring"] >= 32):
        raise ValueError("Robot BPearl ring outside 0..31")
    valid = np.isfinite(points["x"]) & np.isfinite(points["y"]) & np.isfinite(points["z"])
    if not valid.any():
        raise ValueError("Robot scan has no finite geometry")
    selected = np.flatnonzero(valid)
    # Preserve one original no-return endpoint if geometry filtering would
    # shorten the scan. Its NaN geometry remains unusable, never a fake return.
    marker = int(absolute[selected].max()) < end
    if marker:
        selected = np.append(selected, np.flatnonzero(absolute == end)[-1])
    order = np.argsort(absolute[selected], kind="stable")
    selected = selected[order]
    output = np.zeros(len(selected), dtype=LTU_POINT)
    for field in ("x", "y", "z", "intensity"):
        output[field] = points[field][selected]
    output["offset_time_ns"] = absolute[selected] - start
    # BPearl rings are not Livox lines. Neutralize Livox line/tag selectors;
    # preserve all source ring identities in the bag and index histograms.
    info.update(height=height, width=width, row_step=row_step, source_points=len(points),
                source_first_point_ns=int(absolute[0]), source_last_point_ns=int(absolute[-1]),
                source_min_point_ns=start, source_max_point_ns=end,
                source_header_minus_end_ns=info["header_timestamp_ns"] - end,
                valid_geometry_points=int(valid.sum()), filtered_nonfinite_geometry=int((~valid).sum()),
                timing_only_nan_endpoint=int(marker), output_points=len(output),
                source_time_order_changed=bool(np.any(np.diff(absolute) < 0)),
                source_ring_counts=np.bincount(points["ring"], minlength=32).tolist(),
                offset_min_ns=int(output["offset_time_ns"].min()),
                offset_max_ns=int(output["offset_time_ns"].max()))
    return info, start, output


def convert(source: Path, output: Path, seconds: float | None = None) -> dict:
    if seconds is not None and seconds <= 0:
        raise ValueError("--seconds must be positive")
    bag = Ros1Bag(source)
    output.parent.mkdir(parents=True, exist_ok=True)
    index_path = output.with_suffix(".records.jsonl")
    counts, regressions = Counter(), Counter()
    previous, ranges, first_records = {}, {}, {}
    ring_totals = np.zeros(32, dtype=np.int64)
    acc_norms, imu_times = [], []
    first_stamp = None
    max_span = 0
    header_deltas = []
    with output.open("xb") as stream, index_path.open("x", encoding="utf-8") as index:
        for connection, bag_stamp, raw in bag.messages():
            if connection.topic not in {LIDAR_TOPIC, IMU_TOPIC}:
                continue
            if connection.topic == LIDAR_TOPIC:
                if connection.msgtype != "sensor_msgs/PointCloud2":
                    raise ValueError("unexpected Robot LiDAR message type")
                info, stamp, cloud = normalized_cloud(raw)
                payload, kind, count = cloud.tobytes(), 1, len(cloud)
            else:
                if connection.msgtype != "sensor_msgs/Imu":
                    raise ValueError("unexpected Robot IMU message type")
                info, payload, values = imu_payload(raw)
                stamp, kind, count = info["header_timestamp_ns"], 2, 1
            if first_stamp is None:
                first_stamp = stamp
            if seconds is not None and stamp > first_stamp + round(seconds * 1e9):
                break
            sequence = counts["records"]
            stream.write(HEADER.pack(MAGIC, kind, stamp, sequence, count, len(payload)))
            stream.write(payload)
            record = {"sequence": sequence, "topic": connection.topic, "bag_timestamp_ns": bag_stamp,
                      "ltu_timestamp_ns": stamp, **info}
            index.write(json.dumps(record, separators=(",", ":")) + "\n")
            first_records.setdefault(connection.topic, record)
            counts["records"] += 1
            counts[connection.topic] += 1
            if kind == 1:
                for name in ("source_points", "valid_geometry_points", "filtered_nonfinite_geometry",
                             "timing_only_nan_endpoint", "output_points"):
                    counts[name] += info[name]
                ring_totals += info["source_ring_counts"]
                max_span = max(max_span, info["offset_max_ns"])
                header_deltas.append(info["source_header_minus_end_ns"])
            elif len(acc_norms) < 200:
                acc_norms.append(float(np.linalg.norm(values[3:])))
                imu_times.append(stamp)
            if connection.topic in previous and stamp < previous[connection.topic]:
                regressions[connection.topic] += 1
            previous[connection.topic] = stamp
            interval = ranges.setdefault(connection.topic, [stamp, stamp])
            interval[0], interval[1] = min(interval[0], stamp), max(interval[1], stamp)
            if counts["records"] % 10000 == 0:
                print(json.dumps({"records": counts["records"], "clouds": counts[LIDAR_TOPIC],
                                  "elapsed_s": (stamp - first_stamp) / 1e9}), flush=True)
    expected = {c.topic: c.msgcount for c in bag.connections.values()
                if c.topic in {LIDAR_TOPIC, IMU_TOPIC}}
    if seconds is None and any(counts[key] != value for key, value in expected.items()):
        raise ValueError("complete conversion does not match bag index message counts")
    return {"source": str(source.resolve()), "source_bytes": source.stat().st_size,
            "output": str(output.resolve()), "output_bytes": output.stat().st_size,
            "index": str(index_path.resolve()), "prefix_seconds": seconds,
            "counts": dict(counts), "bag_index_counts": expected,
            "topic_ranges_ns": ranges, "per_topic_time_regressions": dict(regressions),
            "source_ring_counts": ring_totals.tolist(), "first_records": first_records,
            "maximum_scan_span_ns": max_span,
            "header_minus_measured_scan_end_ns_range": [min(header_deltas), max(header_deltas)],
            "initial_imu_median_period_ns": float(np.median(np.diff(imu_times))),
            "initial_acc_norm_median_si": float(np.median(acc_norms)),
            "time_shift_seconds": 0.0, "acc_scale": 1.0,
            "time_rule": "Source absolute f64 seconds split into integer/fraction, round nearest ns; stable point sort; no shift",
            "geometry_rule": "Finite xyz copied unchanged; invalid geometry filtered except one original NaN endpoint when needed to preserve measured scan end",
            "carrier_rule": "Offline carrier only: line=tag=flags=0; original 0..31 rings in source/index; no Livox line interpretation",
            "scope": "Robot BPearl/XSens offline evaluation; not production RoboSense driver qualification"}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seconds", type=float)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    report = convert(args.bag, args.output, args.seconds)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": report["output"], "counts": report["counts"]}), flush=True)


if __name__ == "__main__":
    main()

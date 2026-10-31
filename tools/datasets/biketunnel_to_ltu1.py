"""Convert TRaIL-Odom BikeTunnel2 MID-360 input without ROS or new packages.

Select the internal MID-360 SI-compliant IMU only. PointCloud2 timestamps are
absolute Unix nanoseconds stored as float64, not seconds or relative offsets.
The exported LTU1 keeps raw geometry and retains all point tags and lines.
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

LIDAR = "/gt_box/livox/lidar"
IMU = "/gt_box/livox/imu_si_compliant"
RAW_IMU = "/gt_box/livox/imu"
GT = "/gt_box/ap20/prism_position"


def pointcloud_payload(raw: bytes) -> tuple[dict, bytes, np.ndarray]:
    header, offset = ros1_header(raw)
    height, width, count = struct.unpack_from("<III", raw, offset)
    offset += 12
    fields = []
    for _ in range(count):
        length = struct.unpack_from("<I", raw, offset)[0]
        offset += 4
        name = raw[offset:offset + length].decode()
        offset += length
        position, kind, elements = struct.unpack_from("<IBI", raw, offset)
        offset += 9
        fields.append((name, position, kind, elements))
    bigendian, step, row_step, size = struct.unpack_from("<BIII", raw, offset)
    offset += 13
    if bigendian or row_step != width * step or size != row_step * height or len(raw) != offset + size + 1:
        raise ValueError("unexpected BikeTunnel2 PointCloud2 storage layout")
    expected = {"x": 7, "y": 7, "z": 7, "intensity": 7, "tag": 2, "line": 2, "timestamp": 8}
    if {name: kind for name, _, kind, n in fields if n == 1} != expected:
        raise ValueError(f"unexpected BikeTunnel2 fields: {fields}")
    dtype = np.dtype({"names": [f[0] for f in fields],
                      "formats": [{2: "u1", 7: "<f4", 8: "<f8"}[f[2]] for f in fields],
                      "offsets": [f[1] for f in fields], "itemsize": step})
    points = np.frombuffer(raw, dtype=dtype, count=width * height, offset=offset)
    if not np.isfinite(points["timestamp"]).all():
        raise ValueError("nonfinite absolute point timestamp")
    absolute_ns = np.rint(points["timestamp"]).astype(np.int64)
    original_header_ns = header["header_timestamp_ns"]
    base_ns = min(original_header_ns, int(absolute_ns.min())) if len(absolute_ns) else original_header_ns
    offsets = absolute_ns - base_ns
    if len(offsets) and (base_ns < original_header_ns - 128 or offsets.max() >= 1_000_000_000):
        raise ValueError("point timestamps do not express nanoseconds after this scan header")
    result = np.zeros(len(points), dtype=LTU_POINT)
    for field in ("x", "y", "z", "intensity", "tag", "line"):
        result[field] = points[field]
    result["offset_time_ns"] = offsets
    header.update(header_timestamp_ns=base_ns, original_header_timestamp_ns=original_header_ns,
                  header_shift_ns=base_ns - original_header_ns,
                  fields=fields, point_step=step, points=len(points),
                  offset_min_ns=int(offsets.min()) if len(offsets) else None,
                  offset_max_ns=int(offsets.max()) if len(offsets) else None)
    return header, result.tobytes(), points


def rotation(q):
    x, y, z, w = np.asarray(q) / np.linalg.norm(q)
    return np.array([[1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def calibration(root: Path) -> dict:
    data = json.loads((root / "metadata/bag-inspection.json").read_text(encoding="utf-8"))
    transforms = {(v["parent"], v["child"]): v for v in data["tf_static"]["unique_transforms"]}
    def matrix(parent, child):
        value = transforms[parent, child]
        output = np.eye(4)
        output[:3, :3] = rotation(value["quaternion_xyzw"])
        output[:3, 3] = value["translation_m"]
        return output
    lidar_from_imu = matrix("livox_lidar", "livox_imu")
    imu_from_lidar = np.linalg.inv(lidar_from_imu)
    base_from_imu = matrix("box_base", "livox_lidar") @ lidar_from_imu
    imu_from_prism = np.linalg.inv(base_from_imu) @ matrix("box_base", "prism")
    return {"source": str(root / "calibration/tf_static/tf_static.db3"),
            "decoded_source_record": str(root / "metadata/bag-inspection.json"),
            "imu_from_lidar": imu_from_lidar.tolist(),
            "imu_from_prism": imu_from_prism.tolist(),
            "prism_in_imu_m": imu_from_prism[:3, 3].tolist(),
            "position_evaluation": "p_prism = p_imu + R_world_imu * prism_in_imu; no orientation GT"}


def convert(root: Path, output: Path, seconds: float | None = None) -> dict:
    bag_path = next((root / "raw").glob("*nuc_livox_ready.bag"))
    bag = Ros1Bag(bag_path)
    output.mkdir(parents=True, exist_ok=True)
    stream_path = output / "biketunnel2.ltu"
    if stream_path.exists():
        raise FileExistsError(stream_path)
    counts = Counter()
    first = None
    previous = {}
    spans, norms, raw_norms, offsets = [], [], [], []
    header_shifts = []
    tag_counts = np.zeros(256, dtype=np.int64)
    line_counts = np.zeros(256, dtype=np.int64)
    raw_by_stamp = {}
    scale_errors = []
    first_cloud = None
    with stream_path.open("wb") as stream:
        for connection, bag_stamp, raw in bag.messages():
            topic = connection.topic
            counts["decoded:" + topic] += 1
            if topic == RAW_IMU:
                info, _, values = imu_payload(raw)
                raw_norms.append(float(np.linalg.norm(values[3:])))
                raw_by_stamp[info["header_timestamp_ns"]] = np.asarray(values[3:])
                continue
            if topic == IMU:
                info, payload, values = imu_payload(raw)
                kind, count = 2, 1
                norms.append(float(np.linalg.norm(values[3:])))
                original = raw_by_stamp.pop(info["header_timestamp_ns"], None)
                if original is not None:
                    scale_errors.append(float(np.max(abs(np.asarray(values[3:]) - 9.80665 * original))))
            elif topic == LIDAR:
                info, payload, points = pointcloud_payload(raw)
                first_cloud = first_cloud or info
                kind, count = 1, len(points)
                counts["points"] += count
                tag_counts += np.bincount(points["tag"], minlength=256)
                line_counts += np.bincount(points["line"], minlength=256)
                offsets.extend([info["offset_min_ns"], info["offset_max_ns"]])
                header_shifts.append(info["header_shift_ns"])
                counts["unsorted_clouds"] += int(np.any(np.diff(points["timestamp"]) < 0))
            else:
                continue
            stamp = info["header_timestamp_ns"]
            first = stamp if first is None else min(first, stamp)
            if seconds is not None and stamp > first + round(seconds * 1e9):
                break
            if topic in previous and stamp <= previous[topic]:
                raise ValueError(f"nonincreasing header time: {topic}")
            previous[topic] = stamp
            spans.append(abs(bag_stamp - stamp))
            counts["selected:" + topic] += 1
            stream.write(HEADER.pack(MAGIC, kind, stamp, counts["records"], count, len(payload)))
            stream.write(payload)
            counts["records"] += 1
    if seconds is None:
        for c in bag.connections.values():
            if counts["decoded:" + c.topic] != c.msgcount:
                raise ValueError(f"bag index differs from decoded count: {c.topic}")
    cal = calibration(root)
    (output / "calibration.json").write_text(json.dumps(cal, indent=2) + "\n", encoding="utf-8")
    gt_rows = []
    gt_bag = Ros1Bag(next((root / "ground_truth").glob("*.bag")))
    gt_frames = set()
    for c, _, raw in gt_bag.messages():
        if c.topic != GT or c.msgtype != "geometry_msgs/PointStamped":
            raise ValueError("unexpected GT type")
        h, off = ros1_header(raw)
        if len(raw) != off + 24:
            raise ValueError("unexpected GT PointStamped length")
        gt_frames.add(h["frame_id"])
        gt_rows.append((h["header_timestamp_ns"], *struct.unpack_from("<ddd", raw, off)))
    with (output / "prism-position.txt").open("w", encoding="utf-8") as file:
        file.write("# Sensor timestamp seconds; x y z prism position; no attitude GT\n")
        for stamp, x, y, z in gt_rows:
            file.write(f"{stamp // 1_000_000_000}.{stamp % 1_000_000_000:09d} {x:.17g} {y:.17g} {z:.17g}\n")
    result = {"dataset": "TRaIL-Odom BikeTunnel2", "source": str(bag_path.resolve()),
              "output": str(stream_path.resolve()), "prefix_seconds": seconds,
              "counts": dict(counts), "first_cloud": first_cloud,
              "point_timestamp_unit": "absolute Unix ns as double; nearest integer ns minus scan header",
              "point_time_quantization_at_dataset_epoch_ns": 256,
              "cloud_header_shift_min_max_ns": [min(header_shifts), max(header_shifts)],
              "cloud_header_shift_reason": "A point rounded to float64 ns may precede header by <=128 ns; use earlier base and retain every represented absolute point time exactly",
              "relative_point_offset_min_max_ns": [min(offsets), max(offsets)],
              "bag_minus_sensor_header_abs_max_ns": max(spans),
              "tag_counts": {str(i): int(v) for i, v in enumerate(tag_counts) if v},
              "line_counts": {str(i): int(v) for i, v in enumerate(line_counts) if v},
              "imu_topic": IMU, "acc_scale": 1.0,
              "imu_acceleration_norm_median_mps2": float(np.median(norms)),
              "raw_imu_acceleration_norm_median_g": float(np.median(raw_norms)),
              "matched_raw_SI_accel_samples": len(scale_errors),
              "max_SI_minus_9p80665_raw_accel_mps2": max(scale_errors) if scale_errors else None,
              "GT": {"samples": len(gt_rows), "frames": sorted(gt_frames),
                     "first_ns": gt_rows[0][0], "last_ns": gt_rows[-1][0],
                     "meaning": "Prism position only; orientation error must not be reported"},
              "geometry": "Raw point xyz/intensity/tag/line preserved; no clipping or resampling",
              "calibration": cal}
    (output / "conversion.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seconds", type=float)
    args = parser.parse_args()
    print(json.dumps(convert(args.dataset, args.output, args.seconds), indent=2))


if __name__ == "__main__":
    main()

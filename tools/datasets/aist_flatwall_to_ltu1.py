"""Convert AIST Flatwall Avia bags without ROS; retain raw acceleration in g.

The matching replay configuration must set acc_scale=9.80665 exactly once.
Only /livox/lidar CustomMsg and /livox/imu are selected; /livox/points is a
duplicate representation of the same scans and is deliberately not fused.
"""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

import numpy as np
from tools.datasets.elevator_bag_to_ltu1 import (
    HEADER,
    MAGIC,
    Ros1Bag,
    imu_payload,
    livox_payload,
)


def convert_bag(source: Path, output_dir: Path) -> dict:
    bag = Ros1Bag(source)
    output_dir.mkdir(parents=True, exist_ok=True)
    target = output_dir / f"{source.stem}.ltu"
    manifest = output_dir / f"{source.stem}.manifest.json"
    if target.exists() or manifest.exists():
        raise FileExistsError(target)
    counts: Counter = Counter()
    previous: dict[str, int] = {}
    first, last = None, 0
    norms = []
    with target.open("wb") as stream:
        for connection, _, raw in bag.messages():
            topic = connection.topic
            if topic == "/livox/lidar":
                if not connection.msgtype.endswith("/CustomMsg"):
                    raise ValueError(connection.msgtype)
                info, payload, points = livox_payload(raw)
                # Preserve original point order, timing, tags and all six Avia lines.
                kind, count = 1, len(points)
                counts["points"] += count
                counts["unsorted_clouds"] += int(np.any(np.diff(points["offset_time"].astype(np.int64)) < 0))
            elif topic == "/livox/imu":
                if connection.msgtype != "sensor_msgs/Imu":
                    raise ValueError(connection.msgtype)
                info, payload, values = imu_payload(raw)
                kind, count = 2, 1
                norms.append(float(np.linalg.norm(values[3:])))
            else:
                continue
            stamp = info["header_timestamp_ns"]
            if topic in previous and stamp <= previous[topic]:
                raise ValueError(f"nonincreasing sensor time: {topic}")
            previous[topic] = stamp
            first = stamp if first is None else min(first, stamp)
            last = max(last, stamp)
            stream.write(HEADER.pack(MAGIC, kind, stamp, counts["records"], count, len(payload)))
            stream.write(payload)
            counts[topic] += 1
            counts["records"] += 1
    for connection in bag.connections.values():
        if connection.topic in previous and connection.msgcount and counts[connection.topic] != connection.msgcount:
            raise ValueError(f"index mismatch: {connection.topic}")
    if not all(counts[topic] for topic in ("/livox/lidar", "/livox/imu")):
        raise ValueError("missing Avia LiDAR or IMU")
    result = dict(
        source=str(source.resolve()), source_bytes=source.stat().st_size,
        dataset_url="https://zenodo.org/records/7641866", license="CC BY 4.0",
        output=str(target.resolve()), counts=dict(counts),
        first_sensor_ns=first, last_sensor_ns=last, duration_s=(last-first)*1e-9,
        imu_raw_acceleration_norm_median=float(np.median(norms)),
        acceleration="Raw g, unchanged; replay acc_scale=9.80665",
        timestamps="Sensor header stamps, no fitted time offset; per-point ns retained",
        geometry="Raw LiDAR coordinates; no clipping, point filtering or coordinate transform",
    )
    manifest.write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    print(json.dumps(convert_bag(args.bag, args.output_dir), indent=2))


if __name__ == "__main__":
    main()

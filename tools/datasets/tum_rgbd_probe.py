#!/usr/bin/env python3
"""Validate a TUM RGB-D sequence and render one depth frame in map coordinates."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterable

import numpy as np
from PIL import Image


INTRINSICS = {"fx": 517.3, "fy": 516.5, "cx": 318.6, "cy": 255.3, "depth_scale": 5000.0}


def _indexed(path: Path) -> list[tuple[float, Path]]:
    entries: list[tuple[float, Path]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        timestamp, relative = line.split(maxsplit=1)
        entries.append((float(timestamp), path.parent / relative))
    return entries


def _associate(left: Iterable[tuple[float, Path]], right: Iterable[tuple[float, Path]], max_delta_s: float) -> list[dict[str, object]]:
    right_items = list(right)
    pairs: list[dict[str, object]] = []
    right_index = 0
    for left_timestamp, left_path in left:
        while right_index + 1 < len(right_items) and abs(right_items[right_index + 1][0] - left_timestamp) <= abs(right_items[right_index][0] - left_timestamp):
            right_index += 1
        if right_index < len(right_items) and abs(right_items[right_index][0] - left_timestamp) <= max_delta_s:
            right_timestamp, right_path = right_items[right_index]
            pairs.append({"rgb_timestamp": left_timestamp, "depth_timestamp": right_timestamp, "delta_s": abs(left_timestamp - right_timestamp), "rgb": left_path.as_posix(), "depth": right_path.as_posix()})
            right_index += 1
    return pairs


def _pose_matrix(record: tuple[float, np.ndarray]) -> tuple[float, np.ndarray]:
    timestamp, values = record
    tx, ty, tz, qx, qy, qz, qw = values
    rotation = np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = [tx, ty, tz]
    return timestamp, transform


def _poses(path: Path) -> list[tuple[float, np.ndarray]]:
    result: list[tuple[float, np.ndarray]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        values = np.fromstring(line, sep=" ")
        if len(values) == 8:
            result.append(_pose_matrix((float(values[0]), values[1:])))
    return result


def _nearest_pose(timestamp: float, poses: list[tuple[float, np.ndarray]], max_delta_s: float) -> tuple[float, np.ndarray] | None:
    if not poses:
        return None
    candidate = min(poses, key=lambda item: abs(item[0] - timestamp))
    return candidate if abs(candidate[0] - timestamp) <= max_delta_s else None


def _project(depth: np.ndarray, transform: np.ndarray, stride: int) -> tuple[np.ndarray, np.ndarray]:
    rows, columns = np.indices(depth.shape)
    rows = rows[::stride, ::stride]
    columns = columns[::stride, ::stride]
    values = depth[::stride, ::stride].astype(np.float64) / INTRINSICS["depth_scale"]
    valid = (values > 0.1) & (values < 8.0)
    z = values[valid]
    x = (columns[valid] - INTRINSICS["cx"]) * z / INTRINSICS["fx"]
    y = (rows[valid] - INTRINSICS["cy"]) * z / INTRINSICS["fy"]
    camera_points = np.column_stack((x, y, z, np.ones_like(z)))
    map_points = (transform @ camera_points.T).T[:, :3]
    return camera_points[:, :3], map_points


def probe(sequence: Path, output_dir: Path, *, frame_index: int = 0, association_delta_s: float = 0.02) -> dict[str, object]:
    rgb_entries = _indexed(sequence / "rgb.txt")
    depth_entries = _indexed(sequence / "depth.txt")
    pairs = _associate(rgb_entries, depth_entries, association_delta_s)
    poses = _poses(sequence / "groundtruth.txt")
    if not pairs:
        raise ValueError("no RGB/depth pairs satisfy the timestamp tolerance")
    selected = pairs[frame_index]
    pose = _nearest_pose(float(selected["rgb_timestamp"]), poses, 0.05)
    if pose is None:
        raise ValueError("selected RGB/depth pair has no nearby ground-truth pose")
    depth = np.asarray(Image.open(str(selected["depth"])))
    camera_points, map_points = _project(depth, pose[1], stride=4)
    output_dir.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output_dir / "frame-0000-points.npz", camera_points=camera_points, map_points=map_points)

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(1, 2, figsize=(12, 5), dpi=160)
    axes[0].scatter(camera_points[:, 0], camera_points[:, 2], s=0.25, c=camera_points[:, 2], cmap="viridis", linewidths=0)
    axes[0].set_title("TUM RGB-D depth → camera points")
    axes[0].set_xlabel("camera x (m)"); axes[0].set_ylabel("camera z (m)"); axes[0].set_aspect("equal", adjustable="box")
    axes[1].scatter(map_points[:, 0], map_points[:, 1], s=0.25, c=map_points[:, 2], cmap="viridis", linewidths=0)
    axes[1].set_title("same points → ground-truth map frame")
    axes[1].set_xlabel("map x (m)"); axes[1].set_ylabel("map y (m)"); axes[1].set_aspect("equal", adjustable="box")
    figure.tight_layout()
    figure.savefig(output_dir / "frame-0000-projection.png")
    plt.close(figure)

    report = {
        "schema_version": "lingtu.tum_rgbd_probe.v1",
        "sequence": sequence.as_posix(),
        "rgb_frames": len(rgb_entries),
        "depth_frames": len(depth_entries),
        "rgb_depth_pairs": len(pairs),
        "max_pair_delta_s": max(float(pair["delta_s"]) for pair in pairs),
        "groundtruth_poses": len(poses),
        "selected_pair": selected,
        "selected_pose_delta_s": abs(float(pose[0]) - float(selected["rgb_timestamp"])),
        "valid_depth_points": int(len(map_points)),
        "camera_intrinsics": INTRINSICS,
        "output": {
            "projection_png": (output_dir / "frame-0000-projection.png").as_posix(),
            "points_npz": (output_dir / "frame-0000-points.npz").as_posix(),
        },
        "limitations": [
            "TUM RGB-D has no LiDAR stream or object semantic labels.",
            "This proves RGB-D projection and pose usage, not LiDAR-camera fusion or persistent object identity.",
        ],
    }
    (output_dir / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sequence", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(probe(args.sequence, args.output_dir), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

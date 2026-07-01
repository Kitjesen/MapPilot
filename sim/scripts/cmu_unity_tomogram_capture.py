#!/usr/bin/env python3
"""Capture CMU Unity point clouds and build a same-source PCT tomogram."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from pathlib import Path
from typing import Any

import numpy as np


DEFAULT_TOPICS = ("/slam/map_cloud", "/nav/terrain_map_ext")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _finite_point(x: float, y: float, z: float) -> bool:
    return math.isfinite(x) and math.isfinite(y) and math.isfinite(z)


def _write_ascii_pcd(path: Path, points: np.ndarray) -> int:
    path.parent.mkdir(parents=True, exist_ok=True)
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError(f"expected Nx3 points, got shape={pts.shape}")

    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS x y z",
            "SIZE 4 4 4",
            "TYPE F F F",
            "COUNT 1 1 1",
            f"WIDTH {len(pts)}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {len(pts)}",
            "DATA ascii",
        ]
    )
    with path.open("w", encoding="ascii") as f:
        f.write(header)
        f.write("\n")
        for x, y, z in pts:
            f.write(f"{float(x):.5f} {float(y):.5f} {float(z):.5f}\n")
    return int(len(pts))


def _read_ascii_pcd_xyz(path: Path) -> np.ndarray:
    points: list[tuple[float, float, float]] = []
    in_data = False
    with path.open("r", encoding="ascii") as f:
        for line in f:
            text = line.strip()
            if not text:
                continue
            if not in_data:
                if text.lower() == "data ascii":
                    in_data = True
                continue
            parts = text.split()
            if len(parts) < 3:
                continue
            x, y, z = (float(parts[0]), float(parts[1]), float(parts[2]))
            if _finite_point(x, y, z):
                points.append((x, y, z))
    if not points:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray(points, dtype=np.float32)


def _xyz_from_read_point(raw: Any) -> tuple[float, float, float]:
    names = getattr(getattr(raw, "dtype", None), "names", None)
    if names and {"x", "y", "z"}.issubset(set(names)):
        return float(raw["x"]), float(raw["y"]), float(raw["z"])
    x, y, z = raw[:3]
    return float(x), float(y), float(z)


class CloudAccumulator:
    def __init__(
        self,
        *,
        voxel_size: float,
        z_min: float,
        z_max: float,
        max_points: int,
    ) -> None:
        self.voxel_size = float(voxel_size)
        self.z_min = float(z_min)
        self.z_max = float(z_max)
        self.max_points = int(max_points)
        self._points_by_cell: dict[tuple[int, int, int], tuple[float, float, float]] = {}
        self.topic_samples: dict[str, int] = {}
        self.topic_points: dict[str, int] = {}
        self.frames: dict[str, list[str]] = {}

    def add_cloud(self, topic: str, msg: Any) -> None:
        from sensor_msgs_py import point_cloud2

        self.topic_samples[topic] = self.topic_samples.get(topic, 0) + 1
        frame = str(getattr(getattr(msg, "header", None), "frame_id", "") or "")
        if frame:
            frames = self.frames.setdefault(topic, [])
            if frame not in frames:
                frames.append(frame)

        added = 0
        inv = 1.0 / self.voxel_size if self.voxel_size > 0 else 1.0
        for raw in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = _xyz_from_read_point(raw)
            if not _finite_point(x, y, z):
                continue
            if z < self.z_min or z > self.z_max:
                continue
            cell = (math.floor(x * inv), math.floor(y * inv), math.floor(z * inv))
            if cell not in self._points_by_cell:
                self._points_by_cell[cell] = (x, y, z)
                added += 1
                if len(self._points_by_cell) >= self.max_points:
                    break
        self.topic_points[topic] = self.topic_points.get(topic, 0) + added

    def points(self) -> np.ndarray:
        if not self._points_by_cell:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray(list(self._points_by_cell.values()), dtype=np.float32)

    def summary(self) -> dict[str, Any]:
        pts = self.points()
        if len(pts) == 0:
            bounds = None
        else:
            bounds = {
                "min": pts.min(axis=0).astype(float).tolist(),
                "max": pts.max(axis=0).astype(float).tolist(),
            }
        return {
            "samples": self.topic_samples,
            "new_points_by_topic": self.topic_points,
            "frames": self.frames,
            "voxel_size": self.voxel_size,
            "point_count": int(len(pts)),
            "bounds": bounds,
        }


def _build_flat_traversability_tomogram(
    pcd_path: Path,
    tomogram_path: Path,
    args: argparse.Namespace,
) -> dict[str, Any]:
    points = _read_ascii_pcd_xyz(pcd_path)
    if len(points) == 0:
        raise ValueError(f"PCD has no finite XYZ points: {pcd_path}")

    resolution = float(args.resolution)
    if resolution <= 0:
        raise ValueError("resolution must be positive")
    pad = float(args.flat_pad_m)
    xy_min = points[:, :2].min(axis=0) - pad
    xy_max = points[:, :2].max(axis=0) + pad
    shape_xy = np.ceil((xy_max - xy_min) / resolution).astype(int) + 1
    if np.any(shape_xy <= 0):
        raise ValueError(f"invalid flat tomogram shape: {shape_xy.tolist()}")

    # PCT and LingTu path safety consume grids as row=y, col=x. Earlier
    # CMU flat maps were written as x,y, which made native PCT plan in a
    # transposed obstacle field while the safety gate checked a corrected grid.
    trav = np.full(
        (int(shape_xy[1]), int(shape_xy[0])),
        float(args.flat_obstacle_cost),
        dtype=np.float32,
    )
    cols = np.clip(
        np.rint((points[:, 0] - xy_min[0]) / resolution).astype(int),
        0,
        trav.shape[1] - 1,
    )
    rows = np.clip(
        np.rint((points[:, 1] - xy_min[1]) / resolution).astype(int),
        0,
        trav.shape[0] - 1,
    )
    free = np.full_like(trav, bool(args.flat_default_free), dtype=bool)
    floor_like = points[:, 2] <= float(args.ground_h) + float(args.flat_floor_z_max)
    if bool(args.flat_default_free):
        free[rows[floor_like], cols[floor_like]] = True
    else:
        free[rows, cols] = True

    for _ in range(max(0, int(args.flat_free_dilation_cells))):
        grown = free.copy()
        grown[1:, :] |= free[:-1, :]
        grown[:-1, :] |= free[1:, :]
        grown[:, 1:] |= free[:, :-1]
        grown[:, :-1] |= free[:, 1:]
        grown[1:, 1:] |= free[:-1, :-1]
        grown[:-1, :-1] |= free[1:, 1:]
        grown[1:, :-1] |= free[:-1, 1:]
        grown[:-1, 1:] |= free[1:, :-1]
        free = grown

    obstacle = np.zeros_like(trav, dtype=bool)
    obstacle_like = points[:, 2] >= float(args.ground_h) + float(args.flat_obstacle_z_min)
    obstacle[rows[obstacle_like], cols[obstacle_like]] = True
    for _ in range(max(0, int(args.flat_obstacle_inflation_cells))):
        grown = obstacle.copy()
        grown[1:, :] |= obstacle[:-1, :]
        grown[:-1, :] |= obstacle[1:, :]
        grown[:, 1:] |= obstacle[:, :-1]
        grown[:, :-1] |= obstacle[:, 1:]
        grown[1:, 1:] |= obstacle[:-1, :-1]
        grown[:-1, :-1] |= obstacle[1:, 1:]
        grown[1:, :-1] |= obstacle[:-1, 1:]
        grown[:-1, 1:] |= obstacle[1:, :-1]
        obstacle = grown
    free &= ~obstacle

    border = max(0, int(args.flat_border_cells))
    if border:
        free[:border, :] = False
        free[-border:, :] = False
        free[:, :border] = False
        free[:, -border:] = False
    trav[free] = float(args.flat_free_cost)

    data = np.zeros((5, 1, trav.shape[0], trav.shape[1]), dtype=np.float32)
    data[0, 0] = trav
    data[3, 0] = float(args.ground_h)
    data[4, 0] = float(args.flat_ceiling_h)
    center = ((xy_min + xy_max) / 2.0).astype(float)
    tomogram = {
        "data": data,
        "resolution": resolution,
        "center": center.tolist(),
        "origin": xy_min.astype(float).tolist(),
        "slice_h0": float(args.ground_h),
        "slice_dh": float(args.flat_slice_dh),
        "grid_info": {
            "axis_order": "row_y_col_x",
            "origin": xy_min.astype(float).tolist(),
            "resolution": resolution,
            "shape_yx": [int(trav.shape[0]), int(trav.shape[1])],
        },
        "meta": {
            "source": "cmu_unity_flat_traversability",
            "input_pcd": str(pcd_path),
            "input_pcd_sha256": _sha256_file(pcd_path),
            "free_cells": int(np.count_nonzero(free)),
            "obstacle_cells": int(np.count_nonzero(obstacle)),
            "total_cells": int(free.size),
            "default_free": bool(args.flat_default_free),
            "free_dilation_cells": int(args.flat_free_dilation_cells),
            "obstacle_inflation_cells": int(args.flat_obstacle_inflation_cells),
            "border_cells": border,
        },
    }

    tomogram_path.parent.mkdir(parents=True, exist_ok=True)
    import pickle

    with tomogram_path.open("wb") as f:
        pickle.dump(tomogram, f, protocol=pickle.HIGHEST_PROTOCOL)
    return tomogram


def _build_tomogram(pcd_path: Path, tomogram_path: Path, args: argparse.Namespace) -> dict[str, Any]:
    if args.tomogram_mode == "flat_traversability":
        data = _build_flat_traversability_tomogram(pcd_path, tomogram_path, args)
        arr = np.asarray(data.get("data"))
        return {
            "path": str(tomogram_path),
            "exists": tomogram_path.exists(),
            "sha256": _sha256_file(tomogram_path) if tomogram_path.exists() else "",
            "input_pcd": str(pcd_path),
            "input_pcd_sha256": _sha256_file(pcd_path),
            "same_source_input": True,
            "shape": list(arr.shape),
            "resolution": float(data.get("resolution", args.resolution)),
            "center": np.asarray(data.get("center", [0.0, 0.0]), dtype=float).tolist(),
            "origin": np.asarray(data.get("origin", [0.0, 0.0]), dtype=float).tolist(),
            "slice_h0": float(data.get("slice_h0", 0.0)),
            "slice_dh": float(data.get("slice_dh", args.flat_slice_dh)),
            "mode": args.tomogram_mode,
            "free_cells": int(data.get("meta", {}).get("free_cells", 0)),
            "obstacle_cells": int(data.get("meta", {}).get("obstacle_cells", 0)),
            "total_cells": int(data.get("meta", {}).get("total_cells", 0)),
        }

    from nav.services.plan.global_planner.algorithm.pct.vendor.pct_planner.tomography.scripts.build_tomogram import (
        build_tomogram_from_pcd,
    )

    data = build_tomogram_from_pcd(
        str(pcd_path),
        str(tomogram_path),
        resolution=args.resolution,
        slice_dh=args.slice_dh,
        ground_h=args.ground_h,
        kernel_size=args.kernel_size,
        interval_min=args.interval_min,
        interval_free=args.interval_free,
        slope_max=args.slope_max,
        step_max=args.step_max,
        standable_ratio=args.standable_ratio,
        cost_barrier=args.cost_barrier,
        safe_margin=args.safe_margin,
        inflation=args.inflation,
    )
    arr = np.asarray(data.get("data"))
    return {
        "path": str(tomogram_path),
        "exists": tomogram_path.exists(),
        "sha256": _sha256_file(tomogram_path) if tomogram_path.exists() else "",
        "input_pcd": str(pcd_path),
        "input_pcd_sha256": _sha256_file(pcd_path),
        "same_source_input": True,
        "shape": list(arr.shape),
        "resolution": float(data.get("resolution", args.resolution)),
        "center": np.asarray(data.get("center", [0.0, 0.0]), dtype=float).tolist(),
        "slice_h0": float(data.get("slice_h0", 0.0)),
        "slice_dh": float(data.get("slice_dh", args.slice_dh)),
        "mode": args.tomogram_mode,
    }


def run_capture(args: argparse.Namespace) -> dict[str, Any]:
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2

    rclpy.init(args=None)
    node = rclpy.create_node("cmu_unity_tomogram_capture")
    accumulator = CloudAccumulator(
        voxel_size=args.voxel_size,
        z_min=args.z_min,
        z_max=args.z_max,
        max_points=args.max_points,
    )
    topics = list(dict.fromkeys(args.topic or DEFAULT_TOPICS))
    cloud_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions = [
        node.create_subscription(
            PointCloud2,
            topic,
            lambda msg, topic=topic: accumulator.add_cloud(topic, msg),
            cloud_qos,
        )
        for topic in topics
    ]

    deadline = time.time() + float(args.duration_sec)
    try:
        while time.time() < deadline and len(accumulator.points()) < args.max_points:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        for sub in subscriptions:
            node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()

    points = accumulator.points()
    report: dict[str, Any] = {
        "schema_version": "lingtu.same_source_map_capture.v1",
        "ok": False,
        "topics": topics,
        "pcd": str(args.pcd_out),
        "tomogram": str(args.tomogram_out),
        "capture": accumulator.summary(),
        "source_contract": {
            "map_save_source": "runtime_cloud_capture",
            "required_topics": topics,
            "captured_topics": [],
            "same_source_pcd": False,
            "same_source_tomogram": False,
        },
        "artifacts": {},
        "blockers": [],
    }
    if len(points) < args.min_points:
        report["blockers"].append(f"captured point count below threshold: {len(points)} < {args.min_points}")
        return report

    point_count = _write_ascii_pcd(args.pcd_out, points)
    report["pcd_point_count"] = point_count
    pcd_artifact = {
        "path": str(args.pcd_out),
        "exists": args.pcd_out.exists(),
        "point_count": int(point_count),
        "sha256": _sha256_file(args.pcd_out) if args.pcd_out.exists() else "",
        "source_topics": topics,
    }
    report["artifacts"]["pcd"] = pcd_artifact
    report["source_contract"]["captured_topics"] = [
        topic
        for topic in topics
        if int(accumulator.topic_samples.get(topic) or 0) > 0
    ]
    report["source_contract"]["same_source_pcd"] = bool(
        pcd_artifact["exists"]
        and pcd_artifact["point_count"] == int(len(points))
        and bool(pcd_artifact["sha256"])
    )
    if args.build_tomogram:
        try:
            report["tomogram_build"] = _build_tomogram(args.pcd_out, args.tomogram_out, args)
            report["artifacts"]["tomogram"] = {
                "path": report["tomogram_build"].get("path"),
                "exists": report["tomogram_build"].get("exists"),
                "sha256": report["tomogram_build"].get("sha256", ""),
                "input_pcd": report["tomogram_build"].get("input_pcd"),
                "input_pcd_sha256": report["tomogram_build"].get("input_pcd_sha256", ""),
                "same_source_input": report["tomogram_build"].get("same_source_input"),
                "mode": report["tomogram_build"].get("mode"),
                "shape": report["tomogram_build"].get("shape"),
            }
            report["source_contract"]["same_source_tomogram"] = bool(
                report["artifacts"]["tomogram"]["exists"]
                and report["artifacts"]["tomogram"]["same_source_input"] is True
                and report["artifacts"]["tomogram"]["input_pcd_sha256"]
                == pcd_artifact["sha256"]
                and bool(report["artifacts"]["tomogram"]["sha256"])
            )
        except Exception as exc:
            report["blockers"].append(f"tomogram build failed: {type(exc).__name__}: {exc}")
            return report
    report["ok"] = not report["blockers"]
    return report


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", action="append", default=None)
    parser.add_argument("--duration-sec", type=float, default=90.0)
    parser.add_argument("--min-points", type=int, default=500)
    parser.add_argument("--max-points", type=int, default=250_000)
    parser.add_argument("--voxel-size", type=float, default=0.10)
    parser.add_argument("--z-min", type=float, default=-1.0)
    parser.add_argument("--z-max", type=float, default=3.0)
    parser.add_argument("--pcd-out", type=Path, required=True)
    parser.add_argument("--tomogram-out", type=Path, required=True)
    parser.add_argument("--build-tomogram", action="store_true")
    parser.add_argument(
        "--tomogram-mode",
        choices=("official", "flat_traversability"),
        default="official",
        help=(
            "official uses PCT's point-cloud tomography; flat_traversability "
            "projects observed terrain cells into a single traversability layer "
            "for map-first CMU Unity navigation validation"
        ),
    )
    parser.add_argument("--json-out", type=Path, default=None)
    parser.add_argument("--resolution", type=float, default=0.2)
    parser.add_argument("--slice-dh", type=float, default=0.5)
    parser.add_argument("--ground-h", type=float, default=0.0)
    parser.add_argument("--kernel-size", type=int, default=7)
    parser.add_argument("--interval-min", type=float, default=0.50)
    parser.add_argument("--interval-free", type=float, default=0.65)
    parser.add_argument("--slope-max", type=float, default=0.40)
    parser.add_argument("--step-max", type=float, default=0.17)
    parser.add_argument("--standable-ratio", type=float, default=0.20)
    parser.add_argument("--cost-barrier", type=float, default=50.0)
    parser.add_argument("--safe-margin", type=float, default=0.4)
    parser.add_argument("--inflation", type=float, default=0.2)
    parser.add_argument("--flat-pad-m", type=float, default=2.0)
    parser.add_argument("--flat-free-dilation-cells", type=int, default=3)
    parser.add_argument("--flat-default-free", action="store_true")
    parser.add_argument("--flat-floor-z-max", type=float, default=0.35)
    parser.add_argument("--flat-obstacle-z-min", type=float, default=0.35)
    parser.add_argument("--flat-obstacle-inflation-cells", type=int, default=1)
    parser.add_argument("--flat-border-cells", type=int, default=2)
    parser.add_argument("--flat-slice-dh", type=float, default=1.0)
    parser.add_argument("--flat-ceiling-h", type=float, default=2.2)
    parser.add_argument("--flat-free-cost", type=float, default=1.0)
    parser.add_argument("--flat-obstacle-cost", type=float, default=50.0)
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    report = run_capture(args)
    text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())

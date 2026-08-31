"""Diagnose whether a PCD/OctoPlanner3D route has enough support evidence.

This is a map-quality diagnostic, not a planner. It compares the point cloud
that produced an OctoMap with the global path returned by OctoPlanner3D and
reports whether the path is actually supported by nearby occupied map points.
"""

from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]


def _parse_pcd_header(path: Path) -> tuple[dict[str, str], int]:
    header: dict[str, str] = {}
    offset = 0
    with path.open("rb") as fh:
        while True:
            line = fh.readline()
            if not line:
                raise ValueError(f"PCD header missing DATA line: {path}")
            offset += len(line)
            text = line.decode("ascii", errors="ignore").strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            key = parts[0].upper()
            header[key] = " ".join(parts[1:])
            if key == "DATA":
                return header, offset


def load_pcd_xyz(path: Path, *, limit: int = 0) -> np.ndarray:
    header, offset = _parse_pcd_header(path)
    fields = header.get("FIELDS", "").split()
    if not {"x", "y", "z"}.issubset(set(fields)):
        raise ValueError(f"PCD has no x/y/z fields: {path}")
    count = int(header.get("POINTS") or header.get("WIDTH") or "0")
    sizes = [int(v) for v in header.get("SIZE", "").split()]
    types = header.get("TYPE", "").split()
    counts = [int(v) for v in header.get("COUNT", " ".join(["1"] * len(fields))).split()]
    if not sizes or not types:
        raise ValueError(f"PCD missing SIZE/TYPE metadata: {path}")
    expanded: list[tuple[str, int, str]] = []
    for field, size, typ, repeat in zip(fields, sizes, types, counts):
        for _ in range(repeat):
            expanded.append((field, size, typ))
    point_step = sum(size for _, size, _ in expanded)
    xyz_offsets: dict[str, tuple[int, int, str]] = {}
    cur = 0
    for field, size, typ in expanded:
        if field in {"x", "y", "z"} and field not in xyz_offsets:
            xyz_offsets[field] = (cur, size, typ)
        cur += size
    data_kind = header.get("DATA", "").lower()
    if data_kind == "ascii":
        rows: list[list[float]] = []
        with path.open("r", encoding="ascii", errors="ignore") as fh:
            for line in fh:
                if line.strip().lower().startswith("data"):
                    break
            for line in fh:
                parts = line.split()
                if len(parts) < len(fields):
                    continue
                rows.append([float(parts[fields.index(axis)]) for axis in ("x", "y", "z")])
                if limit > 0 and len(rows) >= limit:
                    break
        return np.asarray(rows, dtype=np.float64)
    if data_kind != "binary":
        raise ValueError(f"unsupported PCD DATA kind {data_kind!r}: {path}")
    take = count if limit <= 0 else min(count, limit)
    out = np.empty((take, 3), dtype=np.float64)
    unpack_cache: dict[tuple[int, str], str] = {}

    def read_value(blob: bytes, base: int, axis: str) -> float:
        rel, size, typ = xyz_offsets[axis]
        key = (size, typ)
        fmt = unpack_cache.get(key)
        if fmt is None:
            if typ == "F" and size == 4:
                fmt = "<f"
            elif typ == "F" and size == 8:
                fmt = "<d"
            elif typ == "I" and size == 4:
                fmt = "<i"
            elif typ == "U" and size == 4:
                fmt = "<I"
            else:
                raise ValueError(f"unsupported PCD field type {typ}{size} for {axis}")
            unpack_cache[key] = fmt
        return float(struct.unpack_from(fmt, blob, base + rel)[0])

    blob = path.read_bytes()
    for i in range(take):
        base = offset + i * point_step
        out[i, 0] = read_value(blob, base, "x")
        out[i, 1] = read_value(blob, base, "y")
        out[i, 2] = read_value(blob, base, "z")
    return out[np.isfinite(out).all(axis=1)]


def load_path_from_json(path: Path, *, route_name: str = "") -> list[list[float]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(data, list):
        if route_name:
            for item in data:
                if isinstance(item, dict) and item.get("name") == route_name:
                    return item.get("path") or []
            raise ValueError(f"route {route_name!r} not found in {path}")
        first = data[0] if data else {}
        return first.get("path") if isinstance(first, dict) else []
    if isinstance(data, dict):
        if isinstance(data.get("plan"), dict) and data["plan"].get("path"):
            return data["plan"]["path"]
        if data.get("path"):
            return data["path"]
    return []


def voxelize(points: np.ndarray, resolution: float) -> np.ndarray:
    return np.floor(points / resolution).astype(np.int64)


def point_summary(points: np.ndarray, resolution: float) -> dict[str, Any]:
    if points.size == 0:
        return {"points": 0}
    bbox_min = points.min(axis=0)
    bbox_max = points.max(axis=0)
    extent = np.maximum(bbox_max - bbox_min, 1e-9)
    vox = voxelize(points, resolution)
    occ = np.unique(vox, axis=0)
    columns = np.unique(vox[:, :2], axis=0)
    return {
        "points": int(points.shape[0]),
        "bbox_min": [round(float(v), 4) for v in bbox_min],
        "bbox_max": [round(float(v), 4) for v in bbox_max],
        "extent_m": [round(float(v), 4) for v in extent],
        "density_points_per_m3": round(float(points.shape[0] / np.prod(extent)), 4),
        "occupied_voxels_at_resolution": int(occ.shape[0]),
        "occupied_xy_columns_at_resolution": int(columns.shape[0]),
        "avg_points_per_occupied_voxel": round(float(points.shape[0] / max(1, occ.shape[0])), 4),
    }


def _support_keys_for_path(
    path: np.ndarray,
    resolution: float,
    *,
    xy_radius_cells: int,
    depth_cells: int,
) -> list[list[tuple[int, int, int]]]:
    out: list[list[tuple[int, int, int]]] = []
    for point in path:
        cell = np.floor(point / resolution).astype(np.int64)
        keys: list[tuple[int, int, int]] = []
        for dz in range(1, max(1, depth_cells) + 1):
            for dx in range(-xy_radius_cells, xy_radius_cells + 1):
                for dy in range(-xy_radius_cells, xy_radius_cells + 1):
                    keys.append((int(cell[0] + dx), int(cell[1] + dy), int(cell[2] - dz)))
        out.append(keys)
    return out


def path_support_summary(
    points: np.ndarray,
    path_rows: list[list[float]],
    resolution: float,
    *,
    xy_radius_cells: int,
    depth_cells: int,
) -> dict[str, Any]:
    path = np.asarray(path_rows, dtype=np.float64)
    if points.size == 0 or path.size == 0:
        return {"path_points": int(path.shape[0]) if path.size else 0, "supported_ratio": 0.0}
    occ_keys = {tuple(row) for row in voxelize(points, resolution).tolist()}
    supported: list[bool] = []
    for keys in _support_keys_for_path(
        path,
        resolution,
        xy_radius_cells=xy_radius_cells,
        depth_cells=depth_cells,
    ):
        supported.append(any(key in occ_keys for key in keys))

    deltas = np.diff(path, axis=0)
    dxy = np.linalg.norm(deltas[:, :2], axis=1) if len(path) > 1 else np.asarray([])
    dz = np.abs(deltas[:, 2]) if len(path) > 1 else np.asarray([])
    slope = dz / np.maximum(dxy, 1e-9) if len(path) > 1 else np.asarray([])
    vertical_like = np.where((dxy < resolution * 1.5) & (dz > resolution * 0.5))[0]
    unsupported_indices = [i for i, ok in enumerate(supported) if not ok]
    return {
        "path_points": int(path.shape[0]),
        "path_bbox_min": [round(float(v), 4) for v in path.min(axis=0)],
        "path_bbox_max": [round(float(v), 4) for v in path.max(axis=0)],
        "supported_points": int(sum(supported)),
        "unsupported_points": int(len(supported) - sum(supported)),
        "supported_ratio": round(float(sum(supported) / max(1, len(supported))), 4),
        "first_unsupported_indices": unsupported_indices[:20],
        "max_segment_dxy_m": round(float(dxy.max()) if dxy.size else 0.0, 4),
        "max_segment_dz_m": round(float(dz.max()) if dz.size else 0.0, 4),
        "max_segment_abs_slope": round(float(slope.max()) if slope.size else 0.0, 4),
        "vertical_like_segment_count": int(vertical_like.size),
        "first_vertical_like_segments": [int(v) for v in vertical_like[:20]],
    }


def _nearest_xy_gap_summary(points: np.ndarray, path_rows: list[list[float]]) -> dict[str, Any]:
    path = np.asarray(path_rows, dtype=np.float64)
    if points.size == 0 or path.size == 0:
        return {}
    # Sample for speed. This is diagnostic only.
    step = max(1, points.shape[0] // 50000)
    sample = points[::step]
    gaps = []
    for point in path:
        dz_mask = np.abs(sample[:, 2] - point[2]) <= 0.35
        candidates = sample[dz_mask] if dz_mask.any() else sample
        dist = np.linalg.norm(candidates[:, :2] - point[:2], axis=1)
        gaps.append(float(dist.min()))
    arr = np.asarray(gaps)
    return {
        "nearest_same_layer_xy_gap_m": {
            "median": round(float(np.median(arr)), 4),
            "p90": round(float(np.percentile(arr, 90)), 4),
            "max": round(float(arr.max()), 4),
        }
    }


def diagnose(args: argparse.Namespace) -> dict[str, Any]:
    points = load_pcd_xyz(Path(args.pcd), limit=int(args.limit))
    path = load_path_from_json(Path(args.path_json), route_name=str(args.route_name or ""))
    report = {
        "schema_version": "lingtu.octoplanner3d_map_quality.v1",
        "pcd": str(Path(args.pcd)),
        "path_json": str(Path(args.path_json)),
        "route_name": str(args.route_name or ""),
        "resolution": float(args.resolution),
        "support_xy_radius_cells": int(args.support_xy_radius_cells),
        "support_depth_cells": int(args.support_depth_cells),
        "point_cloud": point_summary(points, float(args.resolution)),
        "path_support": path_support_summary(
            points,
            path,
            float(args.resolution),
            xy_radius_cells=int(args.support_xy_radius_cells),
            depth_cells=int(args.support_depth_cells),
        ),
    }
    report["path_support"].update(_nearest_xy_gap_summary(points, path))
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", required=True)
    parser.add_argument("--path-json", required=True)
    parser.add_argument("--route-name", default="")
    parser.add_argument("--resolution", type=float, default=0.2)
    parser.add_argument("--support-xy-radius-cells", type=int, default=2)
    parser.add_argument("--support-depth-cells", type=int, default=2)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--out", default="")
    args = parser.parse_args()
    report = diagnose(args)
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

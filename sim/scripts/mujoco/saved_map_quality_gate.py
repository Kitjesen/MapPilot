#!/usr/bin/env python3
"""Validate native MuJoCo saved-map geometry against the simulated scene.

This gate is intentionally stricter than a point-count check. It rejects maps
whose obstacle-height cells drift away from the known MuJoCo obstacle footprint,
which catches wall ghosting and thickening that a motion-only SLAM gate misses.
"""

from __future__ import annotations

import argparse
import io
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections import Counter, deque
from pathlib import Path
from typing import Any, Iterable

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from runtime.msgs.numpy_compat import np  # noqa: E402

DEFAULT_WORLD_XML = ROOT / "sim" / "worlds" / "mujoco" / "industrial_park_scene.xml"
SCHEMA_VERSION = "lingtu.mujoco_saved_map_quality_gate.v1"
_OBSTACLE_EXCLUDED_PREFIXES = (
    "ground",
    "road",
    "roof",
    "beam",
    "pipe",
    "start_disk",
    "goal_disk",
    "chassis",
)


def _parse_vec(text: str | None, expected: int | None = None) -> list[float]:
    if not text:
        return []
    try:
        values = [float(v) for v in text.split()]
    except ValueError:
        return []
    if expected is not None and len(values) != expected:
        return []
    if not all(math.isfinite(v) for v in values):
        return []
    return values


def _pcd_scalar_dtype(type_code: str, size: int) -> Any:
    code = str(type_code).upper()
    if code == "F" and size == 4:
        return "<f4"
    if code == "F" and size == 8:
        return "<f8"
    if code == "I" and size == 1:
        return "<i1"
    if code == "I" and size == 2:
        return "<i2"
    if code == "I" and size == 4:
        return "<i4"
    if code == "I" and size == 8:
        return "<i8"
    if code == "U" and size == 1:
        return "<u1"
    if code == "U" and size == 2:
        return "<u2"
    if code == "U" and size == 4:
        return "<u4"
    if code == "U" and size == 8:
        return "<u8"
    raise ValueError(f"unsupported PCD field type={type_code!r} size={size}")


def read_pcd_xyz(path: Path | str) -> tuple[np.ndarray, dict[str, Any]]:
    """Read XYZ columns from ASCII or binary PCD without requiring PCL."""

    pcd_path = Path(path)
    raw = pcd_path.read_bytes()
    stream = io.BytesIO(raw)
    header_lines: list[str] = []
    data_offset = 0
    data_mode = ""
    while True:
        line = stream.readline()
        if not line:
            raise ValueError(f"PCD DATA line missing: {pcd_path}")
        data_offset = stream.tell()
        text = line.decode("ascii", errors="replace").strip()
        header_lines.append(text)
        if text.upper().startswith("DATA "):
            data_mode = text.split(None, 1)[1].strip().lower()
            break

    header: dict[str, list[str]] = {}
    for line in header_lines:
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) >= 2:
            header[parts[0].upper()] = parts[1:]

    fields = header.get("FIELDS", [])
    sizes = [int(v) for v in header.get("SIZE", [])]
    types = header.get("TYPE", [])
    counts = [int(v) for v in header.get("COUNT", ["1"] * len(fields))]
    points = int(header.get("POINTS", ["0"])[0] or 0)
    if points <= 0:
        width = int(header.get("WIDTH", ["0"])[0] or 0)
        height = int(header.get("HEIGHT", ["1"])[0] or 1)
        points = width * height
    if not {"x", "y", "z"}.issubset(set(fields)):
        raise ValueError(f"PCD must contain x y z fields: {pcd_path}")

    body = raw[data_offset:]
    field_indices = [fields.index(axis) for axis in ("x", "y", "z")]
    if data_mode == "ascii":
        text = body.decode("ascii", errors="replace").strip()
        if not text:
            xyz = np.zeros((0, 3), dtype=np.float32)
        else:
            arr = np.loadtxt(io.StringIO(text), dtype=np.float32, usecols=field_indices)
            xyz = np.asarray(arr, dtype=np.float32)
            if xyz.ndim == 1:
                xyz = xyz.reshape(1, -1)
    elif data_mode == "binary":
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError(f"incomplete PCD binary header: {pcd_path}")
        dtype_fields: list[tuple[str, Any] | tuple[str, Any, tuple[int, ...]]] = []
        for name, size, type_code, count in zip(fields, sizes, types, counts):
            scalar = np.dtype(_pcd_scalar_dtype(type_code, int(size)))
            if int(count) == 1:
                dtype_fields.append((name, scalar))
            else:
                dtype_fields.append((name, scalar, (int(count),)))
        dtype = np.dtype(dtype_fields)
        records = np.frombuffer(body, dtype=dtype, count=points)
        xyz = np.column_stack([records[axis].astype(np.float32, copy=False) for axis in ("x", "y", "z")])
    else:
        raise ValueError(f"unsupported PCD DATA mode={data_mode!r}: {pcd_path}")

    return xyz.astype(np.float32, copy=False), {
        "path": str(pcd_path),
        "fields": fields,
        "points_header": int(points),
        "data": data_mode,
    }


def _find_robot_start_xy(root: ET.Element) -> tuple[float, float]:
    body = root.find(".//body[@name='robot_placeholder']")
    if body is not None:
        pos = _parse_vec(body.attrib.get("pos"), 3)
        if pos:
            return float(pos[0]), float(pos[1])
    return 0.0, 0.0


def _world_xml_path(value: str | Path) -> Path:
    text = str(value or "").strip()
    if not text or text == "industrial_park":
        return DEFAULT_WORLD_XML
    path = Path(text)
    if path.is_file():
        return path
    candidate = ROOT / "sim" / "worlds" / "mujoco" / text
    if candidate.is_file():
        return candidate
    if not text.endswith(".xml"):
        candidate = ROOT / "sim" / "worlds" / "mujoco" / f"{text}_scene.xml"
        if candidate.is_file():
            return candidate
    return path


def _is_expected_obstacle_geom(name: str, geom_type: str, pos: list[float], size: list[float], z_min: float, z_max: float) -> bool:
    if geom_type not in {"box", "cylinder"}:
        return False
    lowered = name.lower()
    if any(lowered.startswith(prefix) for prefix in _OBSTACLE_EXCLUDED_PREFIXES):
        return False
    if geom_type == "box" and len(size) >= 3:
        gz0, gz1 = pos[2] - size[2], pos[2] + size[2]
    elif geom_type == "cylinder" and len(size) >= 2:
        gz0, gz1 = pos[2] - size[1], pos[2] + size[1]
    else:
        return False
    return gz1 >= z_min and gz0 <= z_max


def _iter_expected_obstacle_geoms(world_xml: Path, z_min: float, z_max: float) -> tuple[list[dict[str, Any]], tuple[float, float]]:
    root = ET.parse(world_xml).getroot()
    start_xy = _find_robot_start_xy(root)
    geoms: list[dict[str, Any]] = []
    for geom in root.findall(".//geom"):
        name = geom.attrib.get("name", "")
        geom_type = geom.attrib.get("type", "box").strip().lower()
        size_expected = 2 if geom_type == "cylinder" else 3
        pos = _parse_vec(geom.attrib.get("pos"), 3)
        size = _parse_vec(geom.attrib.get("size"), size_expected)
        if not pos or not size:
            continue
        if not _is_expected_obstacle_geom(name, geom_type, pos, size, z_min, z_max):
            continue
        euler = _parse_vec(geom.attrib.get("euler"), 3)
        yaw = float(euler[2]) if len(euler) == 3 else 0.0
        geoms.append({"name": name, "type": geom_type, "pos": pos, "size": size, "yaw": yaw})
    return geoms, start_xy


def _sample_box_cells(
    *,
    center_x: float,
    center_y: float,
    half_x: float,
    half_y: float,
    yaw: float,
    cell_m: float,
) -> set[tuple[int, int]]:
    margin = cell_m * 0.75
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    radius_x = abs(cos_y) * half_x + abs(sin_y) * half_y + margin
    radius_y = abs(sin_y) * half_x + abs(cos_y) * half_y + margin
    ix0 = math.floor((center_x - radius_x) / cell_m)
    ix1 = math.floor((center_x + radius_x) / cell_m)
    iy0 = math.floor((center_y - radius_y) / cell_m)
    iy1 = math.floor((center_y + radius_y) / cell_m)
    cells: set[tuple[int, int]] = set()
    for ix in range(ix0, ix1 + 1):
        x = (ix + 0.5) * cell_m
        for iy in range(iy0, iy1 + 1):
            y = (iy + 0.5) * cell_m
            dx = x - center_x
            dy = y - center_y
            local_x = cos_y * dx + sin_y * dy
            local_y = -sin_y * dx + cos_y * dy
            if abs(local_x) <= half_x + margin and abs(local_y) <= half_y + margin:
                cells.add((ix, iy))
    return cells


def _sample_cylinder_cells(
    *,
    center_x: float,
    center_y: float,
    radius: float,
    cell_m: float,
) -> set[tuple[int, int]]:
    margin = cell_m * 0.75
    r = radius + margin
    ix0 = math.floor((center_x - r) / cell_m)
    ix1 = math.floor((center_x + r) / cell_m)
    iy0 = math.floor((center_y - r) / cell_m)
    iy1 = math.floor((center_y + r) / cell_m)
    cells: set[tuple[int, int]] = set()
    r2 = r * r
    for ix in range(ix0, ix1 + 1):
        x = (ix + 0.5) * cell_m
        for iy in range(iy0, iy1 + 1):
            y = (iy + 0.5) * cell_m
            if (x - center_x) ** 2 + (y - center_y) ** 2 <= r2:
                cells.add((ix, iy))
    return cells


def expected_obstacle_cells(
    world_xml: Path | str,
    *,
    cell_m: float,
    z_min: float,
    z_max: float,
    start_xy: tuple[float, float] | None = None,
) -> tuple[set[tuple[int, int]], dict[str, Any]]:
    world_path = _world_xml_path(world_xml)
    geoms, detected_start_xy = _iter_expected_obstacle_geoms(world_path, z_min, z_max)
    sx, sy = start_xy if start_xy is not None else detected_start_xy
    cells: set[tuple[int, int]] = set()
    for geom in geoms:
        pos = geom["pos"]
        x = float(pos[0]) - float(sx)
        y = float(pos[1]) - float(sy)
        if geom["type"] == "box":
            size = geom["size"]
            cells.update(
                _sample_box_cells(
                    center_x=x,
                    center_y=y,
                    half_x=float(size[0]),
                    half_y=float(size[1]),
                    yaw=float(geom.get("yaw") or 0.0),
                    cell_m=cell_m,
                )
            )
        elif geom["type"] == "cylinder":
            cells.update(
                _sample_cylinder_cells(
                    center_x=x,
                    center_y=y,
                    radius=float(geom["size"][0]),
                    cell_m=cell_m,
                )
            )
    return cells, {
        "world_xml": str(world_path),
        "parsed_obstacle_geoms": len(geoms),
        "start_xy_scene_m": [float(sx), float(sy)],
        "map_frame_assumption": "scene_xy_minus_robot_placeholder_start_xy",
    }


def _component_filter(cells: set[tuple[int, int]], min_component_cells: int) -> tuple[set[tuple[int, int]], dict[str, int]]:
    if not cells:
        return set(), {"connected_components": 0, "kept_component_count": 0, "largest_component_cells": 0}
    remaining = set(cells)
    kept: set[tuple[int, int]] = set()
    component_count = 0
    kept_component_count = 0
    largest = 0
    neighbors = [
        (-1, -1), (0, -1), (1, -1),
        (-1, 0), (1, 0),
        (-1, 1), (0, 1), (1, 1),
    ]
    while remaining:
        start = remaining.pop()
        q: deque[tuple[int, int]] = deque([start])
        comp = {start}
        while q:
            cx, cy = q.popleft()
            for dx, dy in neighbors:
                nb = (cx + dx, cy + dy)
                if nb in remaining:
                    remaining.remove(nb)
                    comp.add(nb)
                    q.append(nb)
        component_count += 1
        largest = max(largest, len(comp))
        if len(comp) >= max(1, int(min_component_cells)):
            kept_component_count += 1
            kept.update(comp)
    return kept, {
        "connected_components": int(component_count),
        "kept_component_count": int(kept_component_count),
        "largest_component_cells": int(largest),
    }


def candidate_obstacle_cells(
    xyz: np.ndarray,
    *,
    cell_m: float,
    z_min: float,
    z_max: float,
    min_points_per_cell: int,
    min_component_cells: int,
) -> tuple[set[tuple[int, int]], dict[str, Any]]:
    finite = np.isfinite(xyz).all(axis=1)
    pts = xyz[finite]
    if pts.shape[0] == 0:
        return set(), {
            "finite_points": 0,
            "obstacle_band_points": 0,
            "raw_obstacle_cells": 0,
            "dense_obstacle_cells": 0,
            "kept_cells": 0,
            "kept_points": 0,
        }
    band = pts[(pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)]
    if band.shape[0] == 0:
        return set(), {
            "finite_points": int(pts.shape[0]),
            "obstacle_band_points": 0,
            "raw_obstacle_cells": 0,
            "dense_obstacle_cells": 0,
            "kept_cells": 0,
            "kept_points": 0,
        }

    indices = np.floor(band[:, :2] / float(cell_m)).astype(np.int64)
    counts = Counter((int(ix), int(iy)) for ix, iy in indices.tolist())
    dense = {cell for cell, count in counts.items() if count >= max(1, int(min_points_per_cell))}
    kept, comp_report = _component_filter(dense, int(min_component_cells))
    kept_points = sum(counts[cell] for cell in kept)
    return kept, {
        "finite_points": int(pts.shape[0]),
        "obstacle_band_points": int(band.shape[0]),
        "raw_obstacle_cells": int(len(counts)),
        "dense_obstacle_cells": int(len(dense)),
        "kept_cells": int(len(kept)),
        "kept_points": int(kept_points),
        "min_points_per_cell": int(min_points_per_cell),
        "min_connected_component_cells": int(min_component_cells),
        "cell_m": float(cell_m),
        **comp_report,
    }


def _nearest_cell_distances_m(candidates: Iterable[tuple[int, int]], expected: set[tuple[int, int]], cell_m: float) -> list[float]:
    expected_list = list(expected)
    if not expected_list:
        return []
    distances: list[float] = []
    for cx, cy in candidates:
        best_sq = min((cx - ex) * (cx - ex) + (cy - ey) * (cy - ey) for ex, ey in expected_list)
        distances.append(math.sqrt(float(best_sq)) * float(cell_m))
    return distances


def _cells_to_xy(cells: Iterable[tuple[int, int]], cell_m: float) -> np.ndarray:
    values = list(cells)
    if not values:
        return np.zeros((0, 2), dtype=np.float64)
    return np.asarray(
        [[(ix + 0.5) * cell_m, (iy + 0.5) * cell_m] for ix, iy in values],
        dtype=np.float64,
    )


def _distance_metrics_from_distances(
    distances: list[float] | np.ndarray,
    *,
    candidate_count: int,
    expected_count: int,
    near_distance_m: float,
    far_distance_m: float,
) -> dict[str, Any]:
    if candidate_count <= 0:
        near_count = 0
        far_count = 0
        near_ratio = 0.0
        far_ratio = 1.0
        median = None
        p90 = None
    else:
        arr = np.asarray(distances, dtype=np.float64)
        near_count = int((arr <= near_distance_m).sum())
        far_count = int((arr > far_distance_m).sum())
        near_ratio = float(near_count / candidate_count)
        far_ratio = float(far_count / candidate_count)
        median = float(np.median(arr)) if arr.size else None
        p90 = float(np.percentile(arr, 90)) if arr.size else None
    return {
        "candidate_cells": int(candidate_count),
        "expected_obstacle_cells": int(expected_count),
        "candidate_cells_within_near_distance": int(near_count),
        "candidate_cells_within_near_distance_ratio": float(near_ratio),
        "candidate_cells_farther_than_far_distance": int(far_count),
        "candidate_cells_farther_than_far_distance_ratio": float(far_ratio),
        "nearest_distance_median_m": median,
        "nearest_distance_p90_m": p90,
        "near_distance_m": float(near_distance_m),
        "far_distance_m": float(far_distance_m),
    }


def _nearest_distances_xy(points_xy: np.ndarray, expected_xy: np.ndarray) -> np.ndarray:
    if points_xy.size == 0 or expected_xy.size == 0:
        return np.zeros((0,), dtype=np.float64)
    try:
        from scipy.spatial import cKDTree

        distances, _ = cKDTree(expected_xy).query(points_xy, k=1)
        return np.asarray(distances, dtype=np.float64)
    except Exception:
        chunks: list[np.ndarray] = []
        for start in range(0, points_xy.shape[0], 256):
            chunk = points_xy[start : start + 256]
            diff = chunk[:, None, :] - expected_xy[None, :, :]
            chunks.append(np.sqrt(np.min(np.sum(diff * diff, axis=2), axis=1)))
        return np.concatenate(chunks) if chunks else np.zeros((0,), dtype=np.float64)


def _aligned_scene_metrics(
    candidates: set[tuple[int, int]],
    expected: set[tuple[int, int]],
    *,
    cell_m: float,
    near_distance_m: float,
    far_distance_m: float,
    yaw_range_deg: float,
    yaw_step_deg: float,
    translation_range_m: float,
    translation_step_m: float,
) -> dict[str, Any]:
    candidate_xy = _cells_to_xy(candidates, cell_m)
    expected_xy = _cells_to_xy(expected, cell_m)
    if candidate_xy.size == 0 or expected_xy.size == 0:
        return {
            "enabled": True,
            "applied": False,
            "reason": "empty_candidate_or_expected_cells",
            **_distance_metrics_from_distances(
                [],
                candidate_count=int(candidate_xy.shape[0]),
                expected_count=int(expected_xy.shape[0]),
                near_distance_m=near_distance_m,
                far_distance_m=far_distance_m,
            ),
        }

    yaw_range = max(0.0, float(yaw_range_deg))
    yaw_step = max(0.1, float(yaw_step_deg))
    trans_range = max(0.0, float(translation_range_m))
    trans_step = max(0.05, float(translation_step_m))
    yaw_values = np.arange(-yaw_range, yaw_range + yaw_step * 0.5, yaw_step)
    trans_values = np.arange(-trans_range, trans_range + trans_step * 0.5, trans_step)

    best: dict[str, Any] | None = None
    for yaw_deg in yaw_values:
        theta = math.radians(float(yaw_deg))
        rot = np.asarray(
            [
                [math.cos(theta), -math.sin(theta)],
                [math.sin(theta), math.cos(theta)],
            ],
            dtype=np.float64,
        )
        rotated = candidate_xy @ rot.T
        for tx in trans_values:
            for ty in trans_values:
                transformed = rotated + np.asarray([float(tx), float(ty)], dtype=np.float64)
                distances = _nearest_distances_xy(transformed, expected_xy)
                metrics = _distance_metrics_from_distances(
                    distances,
                    candidate_count=int(candidate_xy.shape[0]),
                    expected_count=int(expected_xy.shape[0]),
                    near_distance_m=near_distance_m,
                    far_distance_m=far_distance_m,
                )
                score = (
                    metrics["candidate_cells_farther_than_far_distance_ratio"]
                    - metrics["candidate_cells_within_near_distance_ratio"]
                )
                if best is None or score < best["score"]:
                    best = {
                        "score": float(score),
                        "yaw_deg": float(yaw_deg),
                        "translation_xy_m": [float(tx), float(ty)],
                        **metrics,
                    }
    assert best is not None
    return {
        "enabled": True,
        "applied": True,
        "search": {
            "yaw_range_deg": float(yaw_range),
            "yaw_step_deg": float(yaw_step),
            "translation_range_m": float(trans_range),
            "translation_step_m": float(trans_step),
        },
        **best,
    }


def _layer_counts(xyz: np.ndarray, z_min: float, z_max: float) -> dict[str, int]:
    pts = xyz[np.isfinite(xyz).all(axis=1)]
    return {
        "very_low_below_neg0p35": int((pts[:, 2] < -0.35).sum()) if pts.size else 0,
        "ground_like_neg0p35_to_0p25": int(((pts[:, 2] >= -0.35) & (pts[:, 2] <= 0.25)).sum()) if pts.size else 0,
        f"obstacle_band_{z_min:.2f}_to_{z_max:.2f}": int(((pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)).sum()) if pts.size else 0,
        f"high_above_{z_max:.2f}": int((pts[:, 2] > z_max).sum()) if pts.size else 0,
    }


def _load_map_optimization_metadata(pcd_path: Path | str) -> dict[str, Any]:
    metadata_path = Path(pcd_path).parent / "map_optimization.json"
    if not metadata_path.is_file():
        return {"present": False, "path": str(metadata_path)}
    try:
        payload = json.loads(metadata_path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {
            "present": True,
            "path": str(metadata_path),
            "readable": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
    if not isinstance(payload, dict):
        return {
            "present": True,
            "path": str(metadata_path),
            "readable": False,
            "error": "metadata_not_object",
        }
    return {
        "present": True,
        "readable": True,
        "path": str(metadata_path),
        "schema_version": payload.get("schema_version"),
        "status": payload.get("status"),
        "backend": payload.get("backend"),
        "refine_backend": payload.get("refine_backend"),
        "loop_closure_enabled": bool(payload.get("loop_closure_enabled", False)),
        "loop_closure_applied": bool(payload.get("loop_closure_applied", False)),
        "refine_applied": bool(
            payload.get("refine_applied", payload.get("hba_refine_applied", False))
        ),
        "hba_refine_applied": bool(payload.get("hba_refine_applied", False)),
        "loop_count": int(payload.get("loop_count") or 0),
        "optimized_pose_count": int(payload.get("optimized_pose_count") or 0),
        "raw_map_points": int(payload.get("raw_map_points") or 0),
        "optimized_map_points": int(payload.get("optimized_map_points") or 0),
        "loop_closure_error_m": payload.get("loop_closure_error_m"),
    }


def evaluate_saved_map_quality(
    *,
    pcd_path: Path | str,
    world_xml: Path | str,
    cell_m: float = 0.2,
    z_min: float = 0.30,
    z_max: float = 1.60,
    min_points_per_cell: int = 3,
    min_component_cells: int = 8,
    near_distance_m: float = 0.4,
    far_distance_m: float = 0.6,
    min_near_ratio: float = 0.80,
    max_far_ratio: float = 0.15,
    min_candidate_cells: int = 50,
    start_xy: tuple[float, float] | None = None,
    align_to_scene: bool = True,
    alignment_yaw_range_deg: float = 8.0,
    alignment_yaw_step_deg: float = 0.5,
    alignment_translation_range_m: float = 1.0,
    alignment_translation_step_m: float = 0.1,
) -> tuple[dict[str, Any], set[tuple[int, int]], set[tuple[int, int]]]:
    xyz, pcd = read_pcd_xyz(pcd_path)
    candidates, candidate_report = candidate_obstacle_cells(
        xyz,
        cell_m=cell_m,
        z_min=z_min,
        z_max=z_max,
        min_points_per_cell=min_points_per_cell,
        min_component_cells=min_component_cells,
    )
    expected, expected_report = expected_obstacle_cells(
        world_xml,
        cell_m=cell_m,
        z_min=z_min,
        z_max=z_max,
        start_xy=start_xy,
    )
    candidate_count = len(candidates)
    raw_distances = _nearest_cell_distances_m(candidates, expected, cell_m)
    raw_metrics = _distance_metrics_from_distances(
        raw_distances,
        candidate_count=candidate_count,
        expected_count=len(expected),
        near_distance_m=near_distance_m,
        far_distance_m=far_distance_m,
    )
    if align_to_scene:
        basis_metrics = _aligned_scene_metrics(
            candidates,
            expected,
            cell_m=cell_m,
            near_distance_m=near_distance_m,
            far_distance_m=far_distance_m,
            yaw_range_deg=alignment_yaw_range_deg,
            yaw_step_deg=alignment_yaw_step_deg,
            translation_range_m=alignment_translation_range_m,
            translation_step_m=alignment_translation_step_m,
        )
        quality_basis = "aligned_scene_overlay"
    else:
        basis_metrics = {"enabled": False, **raw_metrics}
        quality_basis = "raw_scene_overlay"
    near_ratio = float(basis_metrics["candidate_cells_within_near_distance_ratio"])
    far_ratio = float(basis_metrics["candidate_cells_farther_than_far_distance_ratio"])
    remaining_gaps: list[str] = []
    if candidate_count < int(min_candidate_cells):
        remaining_gaps.append(f"map_quality_too_few_candidate_cells:{candidate_count}<min={int(min_candidate_cells)}")
    if near_ratio < float(min_near_ratio):
        remaining_gaps.append(
            "map_quality_obstacle_cells_not_near_scene:"
            f"near_ratio={near_ratio:.3f}<min={float(min_near_ratio):.3f}"
        )
    if far_ratio > float(max_far_ratio):
        remaining_gaps.append(
            "map_quality_far_ghost_cells:"
            f"far_ratio={far_ratio:.3f}>max={float(max_far_ratio):.3f}"
        )

    report = {
        "schema_version": SCHEMA_VERSION,
        "ok": not remaining_gaps,
        "pcd": pcd,
        "world": expected_report,
        "thresholds": {
            "cell_m": float(cell_m),
            "z_min": float(z_min),
            "z_max": float(z_max),
            "min_points_per_cell": int(min_points_per_cell),
            "min_connected_component_cells": int(min_component_cells),
            "near_distance_m": float(near_distance_m),
            "far_distance_m": float(far_distance_m),
            "min_near_ratio": float(min_near_ratio),
            "max_far_ratio": float(max_far_ratio),
            "min_candidate_cells": int(min_candidate_cells),
            "align_to_scene": bool(align_to_scene),
        },
        "layers": _layer_counts(xyz, z_min, z_max),
        "candidate_filter": candidate_report,
        "quality_basis": quality_basis,
        "map_optimization": _load_map_optimization_metadata(pcd_path),
        "scene_overlay": basis_metrics,
        "raw_scene_overlay": raw_metrics,
        "remaining_gaps": remaining_gaps,
    }
    return report, candidates, expected


def _cell_centers_xy_m(cells: set[tuple[int, int]], cell_m: float) -> np.ndarray:
    cell_list = np.asarray(list(cells), dtype=np.float32)
    if not cell_list.size:
        return np.empty((0, 2), dtype=np.float32)
    return (cell_list[:, :2] + 0.5) * float(cell_m)


def _apply_overlay_alignment_xy_m(xy_m: np.ndarray, overlay_metrics: dict[str, Any] | None) -> np.ndarray:
    if xy_m.size == 0 or not overlay_metrics or not bool(overlay_metrics.get("applied")):
        return xy_m
    yaw_deg = float(overlay_metrics.get("yaw_deg") or 0.0)
    translation = overlay_metrics.get("translation_xy_m") or [0.0, 0.0]
    theta = math.radians(yaw_deg)
    rot = np.asarray(
        [
            [math.cos(theta), -math.sin(theta)],
            [math.sin(theta), math.cos(theta)],
        ],
        dtype=np.float64,
    )
    trans = np.asarray([float(translation[0]), float(translation[1])], dtype=np.float64)
    return (xy_m.astype(np.float64) @ rot.T + trans).astype(np.float32)


def _write_overlay_plot(
    *,
    out_path: Path,
    candidates: set[tuple[int, int]],
    expected: set[tuple[int, int]],
    cell_m: float,
    far_distance_m: float,
    overlay_metrics: dict[str, Any] | None = None,
) -> None:
    import matplotlib.pyplot as plt

    expected_xy = _cell_centers_xy_m(expected, cell_m)
    candidate_xy_raw = _cell_centers_xy_m(candidates, cell_m)
    candidate_xy = _apply_overlay_alignment_xy_m(candidate_xy_raw, overlay_metrics)
    plot_basis = "aligned" if overlay_metrics and bool(overlay_metrics.get("applied")) else "raw"
    fig, ax = plt.subplots(figsize=(8, 8), dpi=150)
    if expected_xy.size:
        ax.scatter(
            expected_xy[:, 0],
            expected_xy[:, 1],
            s=4,
            c="#d0d0d0",
            marker="s",
            label="expected obstacle footprint",
            alpha=0.45,
        )
    if candidate_xy.size:
        distances = _nearest_distances_xy(candidate_xy.astype(np.float64), expected_xy.astype(np.float64))
        colors = ["#d62728" if d > far_distance_m else "#1f77b4" for d in distances]
        ax.scatter(
            candidate_xy[:, 0],
            candidate_xy[:, 1],
            s=8,
            c=colors,
            marker="s",
            label=f"saved-map obstacle cells ({plot_basis})",
            alpha=0.85,
        )
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x m")
    ax.set_ylabel("y m")
    title = "MuJoCo saved-map quality overlay"
    if plot_basis == "aligned":
        translation = overlay_metrics.get("translation_xy_m") or [0.0, 0.0]
        title += (
            f" (aligned yaw={float(overlay_metrics.get('yaw_deg') or 0.0):.1f} deg, "
            f"t=({float(translation[0]):.2f},{float(translation[1]):.2f}) m)"
        )
    ax.set_title(title)
    ax.grid(True, linewidth=0.4, alpha=0.35)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", required=True, help="Native saved map PCD path.")
    parser.add_argument("--world", default=str(DEFAULT_WORLD_XML), help="MuJoCo world XML path or preset name.")
    parser.add_argument("--cell-m", type=float, default=0.2)
    parser.add_argument("--z-min", type=float, default=0.30)
    parser.add_argument("--z-max", type=float, default=1.60)
    parser.add_argument("--min-points-per-cell", type=int, default=3)
    parser.add_argument("--min-component-cells", type=int, default=8)
    parser.add_argument("--near-distance-m", type=float, default=0.4)
    parser.add_argument("--far-distance-m", type=float, default=0.6)
    parser.add_argument("--min-near-ratio", type=float, default=0.80)
    parser.add_argument("--max-far-ratio", type=float, default=0.15)
    parser.add_argument("--min-candidate-cells", type=int, default=50)
    parser.add_argument(
        "--no-align-to-scene",
        action="store_true",
        help="Disable bounded 2D rigid alignment before scene-footprint scoring.",
    )
    parser.add_argument("--alignment-yaw-range-deg", type=float, default=8.0)
    parser.add_argument("--alignment-yaw-step-deg", type=float, default=0.5)
    parser.add_argument("--alignment-translation-range-m", type=float, default=1.0)
    parser.add_argument("--alignment-translation-step-m", type=float, default=0.1)
    parser.add_argument("--start-xy", default="", help="Override scene start xy as 'x,y'; default reads robot_placeholder.")
    parser.add_argument("--json-out", default="")
    parser.add_argument("--plot-out", default="")
    return parser


def _parse_start_xy(value: str) -> tuple[float, float] | None:
    text = str(value or "").strip()
    if not text:
        return None
    parts = [part.strip() for part in text.split(",")]
    if len(parts) != 2:
        raise ValueError("--start-xy must be empty or 'x,y'")
    x, y = (float(part) for part in parts)
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("--start-xy values must be finite")
    return x, y


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    report, candidates, expected = evaluate_saved_map_quality(
        pcd_path=args.pcd,
        world_xml=args.world,
        cell_m=float(args.cell_m),
        z_min=float(args.z_min),
        z_max=float(args.z_max),
        min_points_per_cell=int(args.min_points_per_cell),
        min_component_cells=int(args.min_component_cells),
        near_distance_m=float(args.near_distance_m),
        far_distance_m=float(args.far_distance_m),
        min_near_ratio=float(args.min_near_ratio),
        max_far_ratio=float(args.max_far_ratio),
        min_candidate_cells=int(args.min_candidate_cells),
        start_xy=_parse_start_xy(args.start_xy),
        align_to_scene=not bool(args.no_align_to_scene),
        alignment_yaw_range_deg=float(args.alignment_yaw_range_deg),
        alignment_yaw_step_deg=float(args.alignment_yaw_step_deg),
        alignment_translation_range_m=float(args.alignment_translation_range_m),
        alignment_translation_step_m=float(args.alignment_translation_step_m),
    )
    if args.plot_out:
        plot_path = Path(args.plot_out)
        plot_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            _write_overlay_plot(
                out_path=plot_path,
                candidates=candidates,
                expected=expected,
                cell_m=float(args.cell_m),
                far_distance_m=float(args.far_distance_m),
                overlay_metrics=report.get("scene_overlay") if report.get("quality_basis") == "aligned_scene_overlay" else None,
            )
            report["plot"] = str(plot_path)
            report["plot_basis"] = (
                "aligned_scene_overlay" if report.get("quality_basis") == "aligned_scene_overlay" else "raw_scene_overlay"
            )
        except Exception as exc:  # pragma: no cover - optional diagnostic path
            report.setdefault("warnings", []).append(f"plot_failed:{type(exc).__name__}:{exc}")
    text = json.dumps(report, indent=2, sort_keys=True)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if report.get("ok") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())

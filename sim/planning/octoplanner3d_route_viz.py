#!/usr/bin/env python3
"""Plot OctoPlanner3D route JSON over the real building2_9 PCD cloud."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PCD = (
    REPO_ROOT
    / "sim/fixtures/octoplanner3d/building2_9.pcd"
)
DEFAULT_ROUTES_JSON = REPO_ROOT / "artifacts/octoplanner3d_building2_route_tests.json"
DEFAULT_OUTPUT = (
    REPO_ROOT / "artifacts/generated_route_cloud/building2_9_pcd_octoplanner3d_routes.png"
)
DEFAULT_ROUTE_NAMES = ("cross_floor_up", "mid_to_top")


def _read_pcd_header(raw: bytes) -> tuple[dict[str, list[str]], int]:
    header: dict[str, list[str]] = {}
    pos = 0
    while pos < len(raw):
        end = raw.find(b"\n", pos)
        if end < 0:
            raise ValueError("PCD header is missing DATA line")
        line = raw[pos:end].decode("ascii", errors="ignore").strip()
        pos = end + 1
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        header[parts[0].upper()] = parts[1:]
        if parts[0].upper() == "DATA":
            return header, pos
    raise ValueError("PCD header is missing DATA line")


def load_pcd_xyz(path: Path) -> np.ndarray:
    """Load x/y/z columns from ASCII or binary PCD."""

    raw = path.read_bytes()
    header, offset = _read_pcd_header(raw)
    fields = header.get("FIELDS", [])
    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError(f"PCD must contain x/y/z fields: {path}")

    points = int(header.get("POINTS", ["0"])[0])
    data_type = header.get("DATA", [""])[0].lower()
    if points <= 0:
        raise ValueError(f"PCD POINTS is not positive: {path}")

    if data_type == "ascii":
        table = np.loadtxt(path, comments="#", skiprows=len(raw[:offset].splitlines()))
        idx = [fields.index(name) for name in ("x", "y", "z")]
        return table[:, idx].astype(np.float32, copy=False)

    if data_type != "binary":
        raise ValueError(f"Unsupported PCD DATA type {data_type!r}: {path}")

    sizes = [int(v) for v in header.get("SIZE", [])]
    types = header.get("TYPE", [])
    counts = [int(v) for v in header.get("COUNT", ["1"] * len(fields))]
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError(f"Malformed PCD header: {path}")

    stride = sum(size * count for size, count in zip(sizes, counts))
    payload = raw[offset : offset + points * stride]
    if len(payload) < points * stride:
        raise ValueError(f"PCD binary payload is shorter than header declares: {path}")

    byte_rows = np.frombuffer(payload, dtype=np.uint8).reshape(points, stride)
    offsets: dict[str, tuple[int, int, str]] = {}
    cursor = 0
    for field, size, type_code, count in zip(fields, sizes, types, counts):
        offsets[field] = (cursor, size * count, type_code)
        cursor += size * count

    columns = []
    for name in ("x", "y", "z"):
        start, width, type_code = offsets[name]
        if width < 4 or type_code != "F":
            raise ValueError(f"PCD field {name!r} must be float32-compatible: {path}")
        field_bytes = byte_rows[:, start : start + 4].copy()
        columns.append(np.frombuffer(field_bytes.tobytes(), dtype="<f4"))
    return np.column_stack(columns).astype(np.float32, copy=False)


def deterministic_sample(points: np.ndarray, max_points: int) -> np.ndarray:
    if max_points <= 0 or len(points) <= max_points:
        return points
    idx = np.linspace(0, len(points) - 1, max_points, dtype=np.int64)
    return points[idx]


def load_route_map(path: Path) -> dict[str, np.ndarray]:
    payload: Any = json.loads(path.read_text(encoding="utf-8"))
    routes: dict[str, np.ndarray] = {}
    if isinstance(payload, list):
        for item in payload:
            if not isinstance(item, dict) or "name" not in item or "path" not in item:
                continue
            arr = np.asarray(item["path"], dtype=np.float32)
            if arr.ndim == 2 and arr.shape[1] >= 3 and len(arr) > 0:
                routes[str(item["name"])] = arr[:, :3]
    elif isinstance(payload, dict):
        for name, value in payload.items():
            arr = np.asarray(value, dtype=np.float32)
            if arr.ndim == 2 and arr.shape[1] >= 3 and len(arr) > 0:
                routes[str(name)] = arr[:, :3]
    if not routes:
        raise ValueError(f"No route paths found in {path}")
    return routes


def route_metrics(route: np.ndarray) -> dict[str, float]:
    """Return geometry metrics that explain whether a route is ground-like."""

    if len(route) < 2:
        z_min = float(route[:, 2].min()) if len(route) else 0.0
        z_max = float(route[:, 2].max()) if len(route) else 0.0
        return {
            "points": float(len(route)),
            "z_min": z_min,
            "z_max": z_max,
            "z_range": z_max - z_min,
            "max_segment_dz": 0.0,
            "max_segment_slope": 0.0,
        }

    delta = np.diff(route[:, :3], axis=0)
    dxy = np.linalg.norm(delta[:, :2], axis=1)
    dz = np.abs(delta[:, 2])
    slopes = np.divide(dz, dxy, out=np.full_like(dz, np.inf), where=dxy > 1e-9)
    return {
        "points": float(len(route)),
        "z_min": float(route[:, 2].min()),
        "z_max": float(route[:, 2].max()),
        "z_range": float(route[:, 2].max() - route[:, 2].min()),
        "max_segment_dz": float(dz.max(initial=0.0)),
        "max_segment_slope": float(slopes.max(initial=0.0)),
    }


def _set_equalish_axes(ax: Any, points: np.ndarray, route: np.ndarray) -> None:
    merged = np.vstack([points[:, :3], route[:, :3]])
    mins = merged.min(axis=0)
    maxs = merged.max(axis=0)
    center = (mins + maxs) / 2.0
    radius = max(float((maxs - mins).max()) / 2.0, 1.0)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


def plot_routes(
    points: np.ndarray,
    routes: dict[str, np.ndarray],
    route_names: list[str],
    output: Path,
) -> None:
    missing = [name for name in route_names if name not in routes]
    if missing:
        raise ValueError(f"Missing route(s) {missing}; available={sorted(routes)}")

    fig = plt.figure(figsize=(17, 8.5), dpi=180)
    fig.suptitle("building2_9 PCD + OctoPlanner3D planned paths", fontsize=20, weight="bold")
    cmap = plt.get_cmap("RdYlGn_r")
    z = points[:, 2]
    scatter_handle = None

    views = [(22, -62), (24, 34)]
    for i, route_name in enumerate(route_names[:2], start=1):
        ax = fig.add_subplot(1, 2, i, projection="3d")
        route = routes[route_name]
        scatter_handle = ax.scatter(
            points[:, 0],
            points[:, 1],
            points[:, 2],
            c=z,
            cmap=cmap,
            s=0.18,
            alpha=0.18,
            linewidths=0,
            rasterized=True,
        )
        ax.plot(
            route[:, 0],
            route[:, 1],
            route[:, 2],
            color="#ff8c00",
            linewidth=3.2,
            solid_capstyle="round",
            label="Planned path",
        )
        ax.scatter(*route[0], c="#238b35", s=34, depthshade=False, label="Start")
        ax.scatter(*route[-1], c="#e31a1c", marker="*", s=58, depthshade=False, label="Goal")
        ax.text(*route[0], "Start", color="#238b35", fontsize=9, weight="bold")
        ax.text(*route[-1], "Goal", color="#e31a1c", fontsize=9, weight="bold")
        ax.set_title(route_name, fontsize=14)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.view_init(elev=views[i - 1][0], azim=views[i - 1][1])
        _set_equalish_axes(ax, points, route)

    if scatter_handle is not None:
        cbar = fig.colorbar(scatter_handle, ax=fig.axes, shrink=0.72, pad=0.02)
        cbar.set_label("PCD point height Z (m)")

    handles = [
        plt.Line2D([0], [0], color="#ff8c00", lw=3, label="Planned path"),
        plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="#238b35", label="Start"),
        plt.Line2D([0], [0], marker="*", color="w", markerfacecolor="#e31a1c", label="Goal"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=3, frameon=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, bbox_inches="tight")
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot OctoPlanner3D routes over the real building2_9 PCD cloud."
    )
    parser.add_argument("--pcd", type=Path, default=DEFAULT_PCD)
    parser.add_argument("--routes-json", type=Path, default=DEFAULT_ROUTES_JSON)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--route-names", default=",".join(DEFAULT_ROUTE_NAMES))
    parser.add_argument("--max-points", type=int, default=220_000)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    points = deterministic_sample(load_pcd_xyz(args.pcd), args.max_points)
    routes = load_route_map(args.routes_json)
    route_names = [name.strip() for name in args.route_names.split(",") if name.strip()]
    if not route_names:
        raise ValueError("--route-names must select at least one route")
    plot_routes(points, routes, route_names, args.out)
    summary = {
        "pcd": str(args.pcd),
        "routes_json": str(args.routes_json),
        "out": str(args.out),
        "points_plotted": int(len(points)),
        "routes": route_names[:2],
        "route_metrics": {
            name: route_metrics(routes[name])
            for name in route_names[:2]
            if name in routes
        },
        "z_min": float(points[:, 2].min()),
        "z_max": float(points[:, 2].max()),
    }
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

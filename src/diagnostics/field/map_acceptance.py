#!/usr/bin/env python3
# ruff: noqa: D103
"""Field acceptance gate for a saved mapping run.

This gate checks the saved directory's required files and trajectory/patch
coverage. Native mapd artifact semantics are validated separately by
``python -m diagnostics.field.map_artifacts``.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from itertools import pairwise
from pathlib import Path
from typing import Any


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate saved-map field readiness beyond artifact structure.",
    )
    parser.add_argument("saved_map_dir", type=Path, help="Saved map directory")
    parser.add_argument("--require-octomap", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--require-occupancy", action="store_true")
    parser.add_argument("--min-trajectory-m", type=float, default=1.0)
    parser.add_argument("--min-pose-coverage-ratio", type=float, default=0.20)
    parser.add_argument("--min-patch-poses", type=int, default=10)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--plot-out", type=Path)
    parser.add_argument("--json", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(list(argv or sys.argv[1:]))
    root = args.saved_map_dir.expanduser().resolve()
    blockers: list[str] = []
    warnings: list[str] = []

    trajectory = _trajectory_report(
        root,
        min_trajectory_m=float(args.min_trajectory_m),
        min_pose_coverage_ratio=float(args.min_pose_coverage_ratio),
        min_patch_poses=int(args.min_patch_poses),
    )
    blockers.extend(str(item) for item in trajectory["blockers"])
    warnings.extend(str(item) for item in trajectory["warnings"])

    pcd = _pcd_report(root / "map.pcd")
    metadata = _file_report(root / "metadata.json")
    octomap = _file_report(root / "octomap.ot")
    occupancy = _file_report(root / "occupancy.npz")
    if not pcd["exists"]:
        blockers.append("map.pcd missing")
    elif pcd["point_count"] == 0:
        blockers.append("map.pcd declares zero points")
    if not metadata["exists"]:
        blockers.append("metadata.json missing")
    if args.require_octomap and not octomap["exists"]:
        blockers.append("octomap.ot missing")
    if args.require_occupancy and not occupancy["exists"]:
        blockers.append("occupancy.npz missing")

    payload: dict[str, Any] = {
        "schema_version": "lingtu.saved_map.field_acceptance.v1",
        "ok": not blockers,
        "map_id": root.name,
        "acceptance": {
            "min_trajectory_m": float(args.min_trajectory_m),
            "min_pose_coverage_ratio": float(args.min_pose_coverage_ratio),
            "min_patch_poses": int(args.min_patch_poses),
            "require_octomap": bool(args.require_octomap),
            "require_occupancy": bool(args.require_occupancy),
        },
        "artifacts": {
            "map_pcd": pcd,
            "metadata": metadata,
            "octomap": octomap,
            "occupancy": occupancy,
        },
        "trajectory": trajectory,
        "plot": {
            "path": str(args.plot_out) if args.plot_out else "",
            "written": False,
        },
        "warnings": warnings,
        "blockers": blockers,
    }
    if args.plot_out:
        plot_report = _write_overlay_plot(root, args.plot_out, payload)
        payload["plot"] = plot_report
        if plot_report.get("warning"):
            payload["warnings"].append(str(plot_report["warning"]))
    text = json.dumps(payload, indent=2, sort_keys=True)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    if args.json:
        print(text)
    else:
        print(_format_text(payload))
    return 0 if payload["ok"] else 2


def _trajectory_report(
    root: Path,
    *,
    min_trajectory_m: float,
    min_pose_coverage_ratio: float,
    min_patch_poses: int,
) -> dict[str, Any]:
    trajectory = _read_xy_path(root / "trajectory.txt", x_index=1, y_index=2)
    poses = _read_xy_path(root / "poses.txt", x_index=1, y_index=2)
    patch_count = sum(1 for path in (root / "patches").glob("*.pcd") if path.is_file())
    trajectory_len = _xy_length(trajectory)
    pose_len = _xy_length(poses)
    coverage = pose_len / trajectory_len if trajectory_len > 1e-9 else 0.0

    blockers: list[str] = []
    warnings: list[str] = []
    if len(trajectory) < 2:
        blockers.append("trajectory.txt has fewer than 2 poses")
    if trajectory_len < min_trajectory_m:
        blockers.append(f"trajectory too short: {trajectory_len:.3f} m < {min_trajectory_m:.3f} m")
    if len(poses) < min_patch_poses:
        blockers.append(f"too few patch poses: {len(poses)} < {min_patch_poses}")
    if patch_count < min_patch_poses:
        blockers.append(f"too few patch files: {patch_count} < {min_patch_poses}")
    if patch_count != len(poses):
        blockers.append(f"patch file/pose count mismatch: {patch_count} != {len(poses)}")
    if trajectory_len >= min_trajectory_m and coverage < min_pose_coverage_ratio:
        blockers.append(
            "saved patch poses cover too little of the run: "
            f"{pose_len:.3f} m / {trajectory_len:.3f} m = {coverage:.3f}"
        )

    return {
        "trajectory_points": len(trajectory),
        "pose_count": len(poses),
        "patch_count": patch_count,
        "trajectory_xy_length_m": round(trajectory_len, 3),
        "pose_xy_length_m": round(pose_len, 3),
        "pose_coverage_ratio": round(coverage, 4),
        "start_xy": _round_xy(trajectory[0]) if trajectory else None,
        "end_xy": _round_xy(trajectory[-1]) if trajectory else None,
        "blockers": blockers,
        "warnings": warnings,
    }


def _pcd_report(path: Path) -> dict[str, Any]:
    point_count: int | None = None
    if path.is_file():
        try:
            with path.open("rb") as fh:
                for raw in fh:
                    line = raw.decode("ascii", errors="ignore").strip()
                    upper = line.upper()
                    if upper.startswith("POINTS "):
                        point_count = _as_int(line.split(maxsplit=1)[1])
                    if upper.startswith("DATA "):
                        break
        except OSError:
            point_count = None
    return {
        "exists": path.is_file(),
        "size_bytes": path.stat().st_size if path.is_file() else 0,
        "point_count": int(point_count or 0),
    }


def _file_report(path: Path) -> dict[str, Any]:
    return {
        "exists": path.is_file(),
        "size_bytes": path.stat().st_size if path.is_file() else 0,
    }


def _write_overlay_plot(root: Path, output: Path, payload: dict[str, Any]) -> dict[str, Any]:
    report: dict[str, Any] = {
        "path": str(output),
        "written": False,
    }
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        report["warning"] = f"matplotlib unavailable: {type(exc).__name__}: {exc}"
        return report

    trajectory = _read_xy_path(root / "trajectory.txt", x_index=1, y_index=2)
    poses = _read_xy_path(root / "poses.txt", x_index=1, y_index=2)
    if not trajectory and not poses:
        report["warning"] = "no trajectory or pose data available for plot"
        return report

    output.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, 6), dpi=160)
    if trajectory:
        xs, ys = zip(*trajectory)
        ax.plot(xs, ys, color="#4f83ff", linewidth=2.0, label="full SLAM trajectory")
        ax.scatter([trajectory[0][0]], [trajectory[0][1]], color="#2ebd67", s=48, label="start", zorder=4)
        ax.scatter([trajectory[-1][0]], [trajectory[-1][1]], color="#ff5757", marker="x", s=64, label="end", zorder=4)
    if poses:
        xs, ys = zip(*poses)
        ax.plot(xs, ys, color="#ff9f1a", linewidth=1.5, alpha=0.9, label="saved patch poses")
        ax.scatter(xs, ys, color="#ff9f1a", s=10, alpha=0.6)

    ok = bool(payload.get("ok"))
    status = "PASS" if ok else "FAIL"
    traj = payload.get("trajectory", {})
    title = (
        f"{status}: trajectory {traj.get('trajectory_xy_length_m')} m, "
        f"keyframes {traj.get('pose_xy_length_m')} m, "
        f"coverage {traj.get('pose_coverage_ratio')}"
    )
    ax.set_title(title, loc="left", fontsize=11, fontweight="bold")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, alpha=0.25)
    ax.axis("equal")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(output)
    plt.close(fig)
    report["written"] = True
    return report


def _read_xy_path(path: Path, *, x_index: int, y_index: int) -> list[tuple[float, float]]:
    if not path.is_file():
        return []
    points: list[tuple[float, float]] = []
    try:
        with path.open(encoding="utf-8") as fh:
            for line in fh:
                parts = line.strip().split()
                if len(parts) <= max(x_index, y_index):
                    continue
                try:
                    points.append((float(parts[x_index]), float(parts[y_index])))
                except ValueError:
                    continue
    except OSError:
        return []
    return points


def _xy_length(points: list[tuple[float, float]]) -> float:
    length = 0.0
    for (x0, y0), (x1, y1) in pairwise(points):
        length += math.hypot(x1 - x0, y1 - y0)
    return length


def _round_xy(point: tuple[float, float]) -> list[float]:
    return [round(float(point[0]), 3), round(float(point[1]), 3)]


def _as_int(value: Any) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


def _format_text(payload: dict[str, Any]) -> str:
    status = "PASS" if payload["ok"] else "FAIL"
    trajectory = payload["trajectory"]
    lines = [
        f"Saved map field acceptance: {status}",
        f"map: {payload['map_id']}",
        (
            "trajectory: "
            f"{trajectory['trajectory_xy_length_m']} m, "
            f"poses: {trajectory['pose_xy_length_m']} m, "
            f"coverage: {trajectory['pose_coverage_ratio']}"
        ),
    ]
    if payload["warnings"]:
        lines.append("warnings:")
        lines.extend(f"  - {item}" for item in payload["warnings"])
    if payload["blockers"]:
        lines.append("blockers:")
        lines.extend(f"  - {item}" for item in payload["blockers"])
    return "\n".join(lines)


if __name__ == "__main__":
    raise SystemExit(main())

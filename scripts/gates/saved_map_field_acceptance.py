#!/usr/bin/env python3
"""Field acceptance gate for a saved mapping run.

This gate is intentionally stricter than the artifact checksum gate. A map can
have valid files and still be unusable if the optimizer only saw stationary or
near-duplicate keyframes. This script checks both artifact integrity and the
trajectory/keyframe coverage needed for a product map.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]


def _ensure_import_path() -> None:
    for candidate in (ROOT / "src", ROOT):
        path = str(candidate)
        if path not in sys.path:
            sys.path.insert(0, path)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate saved-map field readiness beyond file checksums.",
    )
    parser.add_argument("map_dir", type=Path, help="Saved map directory")
    parser.add_argument("--require-octomap", action="store_true", default=True)
    parser.add_argument("--no-require-octomap", dest="require_octomap", action="store_false")
    parser.add_argument("--require-occupancy", action="store_true")
    parser.add_argument("--allow-missing-optimization", action="store_true")
    parser.add_argument("--min-trajectory-m", type=float, default=1.0)
    parser.add_argument("--min-pose-coverage-ratio", type=float, default=0.20)
    parser.add_argument("--min-patch-poses", type=int, default=10)
    parser.add_argument("--expected-data-source")
    parser.add_argument("--expected-source-profile")
    parser.add_argument("--expected-frame-id")
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--plot-out", type=Path)
    parser.add_argument("--json", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(list(argv or sys.argv[1:]))
    _ensure_import_path()

    from maps.artifacts import validate_saved_map_artifact_dir

    root = args.map_dir
    blockers: list[str] = []
    warnings: list[str] = []

    artifact_gate = validate_saved_map_artifact_dir(
        root,
        require_octomap=args.require_octomap,
        require_occupancy=args.require_occupancy,
        expected_data_source=args.expected_data_source,
        expected_source_profile=args.expected_source_profile,
        expected_frame_id=args.expected_frame_id,
    )
    if not artifact_gate.get("ok"):
        blockers.extend(f"artifact: {item}" for item in artifact_gate.get("blockers", []))

    trajectory = _trajectory_report(
        root,
        min_trajectory_m=float(args.min_trajectory_m),
        min_pose_coverage_ratio=float(args.min_pose_coverage_ratio),
        min_patch_poses=int(args.min_patch_poses),
    )
    blockers.extend(str(item) for item in trajectory["blockers"])
    warnings.extend(str(item) for item in trajectory["warnings"])

    optimization = _optimization_report(root)
    if not args.allow_missing_optimization:
        if not optimization["exists"]:
            blockers.append("map_optimization.json missing")
        elif optimization["status"] in {"failed", "degraded"}:
            blockers.append(
                f"map optimization not accepted: {optimization.get('reason_code') or optimization['status']}"
            )
        elif optimization.get("success") is False:
            blockers.append("map optimization reported success=false")
    if optimization.get("reason_code") == "optimizer_input_degenerate":
        blockers.append("map optimization used degenerate keyframe input")
    if optimization.get("no_numeric_progress"):
        warnings.append("map optimization made no numeric progress")

    pcd = _pcd_report(root / "map.pcd")
    if pcd["exists"] and pcd["point_count"] == 0:
        blockers.append("map.pcd declares zero points")

    payload: dict[str, Any] = {
        "schema_version": "lingtu.saved_map.field_acceptance.v1",
        "ok": not blockers,
        "map_dir": str(root),
        "acceptance": {
            "min_trajectory_m": float(args.min_trajectory_m),
            "min_pose_coverage_ratio": float(args.min_pose_coverage_ratio),
            "min_patch_poses": int(args.min_patch_poses),
            "require_octomap": bool(args.require_octomap),
            "require_occupancy": bool(args.require_occupancy),
            "require_optimization": not bool(args.allow_missing_optimization),
        },
        "artifacts": {
            "map_pcd": pcd,
        },
        "artifact_gate": artifact_gate,
        "trajectory": trajectory,
        "optimization": optimization,
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
    optimized = _read_xy_path(root / "poses_optimized.txt", x_index=1, y_index=2)
    trajectory_len = _xy_length(trajectory)
    pose_len = _xy_length(poses)
    optimized_len = _xy_length(optimized)
    coverage = pose_len / trajectory_len if trajectory_len > 1e-9 else 0.0

    blockers: list[str] = []
    warnings: list[str] = []
    if len(trajectory) < 2:
        blockers.append("trajectory.txt has fewer than 2 poses")
    if trajectory_len < min_trajectory_m:
        blockers.append(f"trajectory too short: {trajectory_len:.3f} m < {min_trajectory_m:.3f} m")
    if len(poses) < min_patch_poses:
        blockers.append(f"too few patch poses: {len(poses)} < {min_patch_poses}")
    if trajectory_len >= min_trajectory_m and coverage < min_pose_coverage_ratio:
        blockers.append(
            "optimizer keyframes cover too little of the run: "
            f"{pose_len:.3f} m / {trajectory_len:.3f} m = {coverage:.3f}"
        )
    if optimized and optimized_len < max(0.10, trajectory_len * 0.05):
        warnings.append("optimized poses are nearly stationary; verify optimizer input and output")

    return {
        "trajectory_points": len(trajectory),
        "pose_count": len(poses),
        "optimized_pose_count": len(optimized),
        "trajectory_xy_length_m": round(trajectory_len, 3),
        "pose_xy_length_m": round(pose_len, 3),
        "optimized_pose_xy_length_m": round(optimized_len, 3) if optimized else None,
        "pose_coverage_ratio": round(coverage, 4),
        "start_xy": _round_xy(trajectory[0]) if trajectory else None,
        "end_xy": _round_xy(trajectory[-1]) if trajectory else None,
        "blockers": blockers,
        "warnings": warnings,
    }


def _optimization_report(root: Path) -> dict[str, Any]:
    path = root / "map_optimization.json"
    report: dict[str, Any] = {
        "path": str(path),
        "exists": path.is_file(),
        "status": "missing",
        "success": None,
        "reason_code": "",
        "no_numeric_progress": False,
    }
    if not path.is_file():
        return report
    try:
        loaded = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        report.update(
            {
                "status": "unreadable",
                "reason_code": "map_optimization_unreadable",
                "error": f"{type(exc).__name__}: {exc}",
            }
        )
        return report
    if not isinstance(loaded, dict):
        report.update(
            {
                "status": "invalid",
                "reason_code": "map_optimization_not_object",
            }
        )
        return report

    status = str(loaded.get("status") or ("ok" if loaded.get("success") else "unknown"))
    initial_cost = _as_float(loaded.get("initial_cost"))
    final_cost = _as_float(loaded.get("final_cost"))
    iterations = _as_int(loaded.get("iterations"))
    report.update(
        {
            "status": status,
            "success": loaded.get("success"),
            "reason_code": str(loaded.get("reason_code") or loaded.get("code") or ""),
            "strategy": loaded.get("strategy"),
            "iterations": iterations,
            "initial_cost": initial_cost,
            "final_cost": final_cost,
            "pose_count": _as_int(loaded.get("pose_count")),
            "patch_count": _as_int(loaded.get("patch_count")),
            "diagnostics": loaded.get("diagnostics") if isinstance(loaded.get("diagnostics"), dict) else {},
        }
    )
    report["no_numeric_progress"] = bool(
        iterations == 0
        and initial_cost is not None
        and final_cost is not None
        and math.isclose(initial_cost, final_cost, rel_tol=1e-9, abs_tol=1e-12)
    )
    diagnostics = report.get("diagnostics") or {}
    if diagnostics.get("optimizer_input_degenerate") is True:
        report["reason_code"] = "optimizer_input_degenerate"
        if status == "ok":
            report["status"] = "degraded"
    return report


def _pcd_report(path: Path) -> dict[str, Any]:
    point_count: int | None = None
    if path.is_file():
        try:
            with path.open("rb") as fh:
                for raw in fh:
                    try:
                        line = raw.decode("ascii", errors="ignore").strip()
                    except Exception:
                        continue
                    upper = line.upper()
                    if upper.startswith("POINTS "):
                        point_count = _as_int(line.split(maxsplit=1)[1])
                    if upper.startswith("DATA "):
                        break
        except OSError:
            point_count = None
    return {
        "path": str(path),
        "exists": path.is_file(),
        "size_bytes": path.stat().st_size if path.is_file() else 0,
        "point_count": int(point_count or 0),
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
    optimized = _read_xy_path(root / "poses_optimized.txt", x_index=1, y_index=2)
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
        ax.plot(xs, ys, color="#ff9f1a", linewidth=1.5, alpha=0.9, label="optimizer input poses")
        ax.scatter(xs, ys, color="#ff9f1a", s=10, alpha=0.6)
    if optimized:
        xs, ys = zip(*optimized)
        ax.plot(xs, ys, color="#35c9c6", linewidth=1.5, alpha=0.85, label="optimized poses")

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
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        length += math.hypot(x1 - x0, y1 - y0)
    return length


def _round_xy(point: tuple[float, float]) -> list[float]:
    return [round(float(point[0]), 3), round(float(point[1]), 3)]


def _as_float(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


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
        f"map: {payload['map_dir']}",
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

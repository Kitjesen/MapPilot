#!/usr/bin/env python3
"""Run repeated native-DDS MuJoCo navigation acceptance in long-range distance mode."""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections.abc import Iterable
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_MANIFEST = ROOT / "config" / "runtime_graph" / "acceptance" / "mujoco_native_navigation_acceptance.json"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.scripts.mujoco import native_navigation_acceptance as nav


def _load_manifest(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"manifest is not an object: {path}")
    return value


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=True, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _resolve_video_path(out_dir: Path, video: dict[str, Any]) -> str:
    raw = str(video.get("path") or "").strip()
    if not raw:
        return ""
    candidate = Path(raw)
    if not candidate.is_absolute():
        candidate = out_dir / raw
    if candidate.is_file():
        return str(candidate.resolve())
    return str(candidate)


def _attempt_id(index: int) -> str:
    return f"{index + 1:02d}"


def _build_summary_text(report: dict[str, Any]) -> str:
    lines = [
        f"mode: {report.get('mode')}",
        f"ok: {report.get('ok')}",
        f"attempts: {report.get('attempts')} / {report.get('required_attempts')}",
        f"successes: {report.get('successes')}",
        f"failures: {report.get('failures')}",
        f"strict: {report.get('strict')}",
        "",
    ]
    for item in report.get("attempts_report", []):
        lines.append(
            (
                f"attempt-{item.get('attempt')}: ok={item.get('ok')} "
                f"goal_reached={item.get('goal_reached')} "
                f"dist_reduction={item.get('goal_distance_reduction_m')} "
                f"path={item.get('sim_path_length_xy_m')} "
                f"video={item.get('video_path', '')}"
            )
        )
    return "\n".join(lines)


def _build_summary_html(report: dict[str, Any]) -> str:
    rows = []
    for item in report.get("attempts_report", []):
        video = item.get("video", {})
        video_path = str(item.get("video_path") or "")
        video_link = (
            f'<a href="{video_path}">video</a>' if video_path else "N/A"
        )
        rows.append(
            "<tr>"
            f"<td>{item.get('attempt')}</td>"
            f"<td>{item.get('ok')}</td>"
            f"<td>{item.get('goal')}</td>"
            f"<td>{item.get('goal_reached')}</td>"
            f"<td>{item.get('goal_distance_reduction_m')}</td>"
            f"<td>{item.get('sim_path_length_xy_m')}</td>"
            f"<td>{video.get('ok')}</td>"
            f"<td>{video_link}</td>"
            "</tr>"
        )
    table = "\n".join(rows)
    return (
        "<html><body>"
        "<h3>Long-range native navigation acceptance</h3>"
        f"<p>ok={report.get('ok')} attempts={report.get('attempts')}/{report.get('required_attempts')} "
        f"successes={report.get('successes')} failures={report.get('failures')}</p>"
        "<table border=\"1\" cellpadding=\"4\" cellspacing=\"0\">"
        "<tr><th>attempt</th><th>ok</th><th>goal</th><th>goal_reached</th>"
        "<th>reduction_m</th><th>path_m</th><th>video_ok</th><th>video</th></tr>"
        f"{table}"
        "</table></body></html>"
    )

def _build_long_range_goals(*, count: int, min_distance_m: float, max_distance_m: float) -> list[list[float]]:
    if count <= 0:
        return []
    if not (math.isfinite(min_distance_m) and math.isfinite(max_distance_m) and min_distance_m > 0.0 and max_distance_m > 0.0):
        raise ValueError("goal distance bounds must be positive finite numbers")
    if max_distance_m < min_distance_m:
        raise ValueError("goal max distance must be >= min distance")
    if count == 1:
        return [[(min_distance_m + max_distance_m) * 0.5, 0.0, 0.30, 0.0]]
    step = (max_distance_m - min_distance_m) / (count - 1)
    # Alternate tiny lateral drift to avoid single-axis aliasing while keeping 50–70 m chord lengths.
    goals: list[list[float]] = []
    for index in range(count):
        distance = min_distance_m + step * index
        lateral = 0.15 * ((index % 3) - 1)
        goals.append([distance, lateral, 0.30, 0.0])
    return goals


def _run_single_navigation_goal(
    *,
    attempt_index: int,
    manifest_path: Path,
    out_base: Path,
    goal: Iterable[float],
    domain_id_base: int,
    phase: str,
    record_video: bool,
    run_fn: Callable[[argparse.Namespace], dict[str, Any]] = nav.run,
) -> dict[str, Any]:
    manifest = _load_manifest(manifest_path)
    manifest["goal"] = [float(value) for value in goal]
    run_manifest = out_base / "manifests" / f"attempt_{attempt_index + 1:02d}.json"
    _write_json(run_manifest, manifest)
    args = argparse.Namespace(
        manifest=str(run_manifest),
        mode=phase,
        domain_id=domain_id_base + attempt_index,
        world="",
        map_dir="",
        out_dir=str(out_base / f"attempt_{attempt_index + 1:02d}"),
        build_helper=False,
        prepare_assets=True,
        preflight_only=False,
        phase_duration_s=None,
        record_video=bool(record_video),
        video_width=1920,
        video_height=1080,
        video_fps=24.0,
        video_lidar_points=640,
        diagnostic_imu_acc_mode="",
        diagnostic_scan_time_profile="",
        diagnostic_imu_acc_lowpass_hz=None,
        diagnostic_imu_acc_max_dynamic_mps2=None,
        diagnostic_imu_acc_max_slew_mps3=None,
    )
    run_report = run_fn(args)
    goal_metrics = run_report.get("phases", {}).get("motion", {}).get("goal_metrics", {})
    motion_phase = run_report.get("phases", {}).get("motion", {})
    video_report = motion_phase.get("video", {})
    out_dir = Path(out_base / f"attempt_{attempt_index + 1:02d}")
    video_path = _resolve_video_path(out_dir, video_report) if isinstance(video_report, dict) else ""
    return {
        "attempt": attempt_index + 1,
        "goal": manifest["goal"],
        "manifest": str(run_manifest),
        "out_dir": str(out_dir),
        "ok": bool(run_report.get("ok")),
        "blockers": list(run_report.get("blockers") or []),
        "goal_reached": bool(goal_metrics.get("native_goal_reached", False)),
        "goal_distance_reduction_m": float(goal_metrics.get("distance_reduction_m") or 0.0),
        "sim_path_length_xy_m": float(goal_metrics.get("sim_path_length_xy_m") or 0.0),
        "video": video_report,
        "video_path": video_path,
    }


def run_long_range_acceptance(
    *,
    manifest: Path,
    attempts: int,
    min_distance_m: float,
    max_distance_m: float,
    phase: str,
    artifact_dir: Path,
    domain_id_base: int = 220,
    record_video: bool = False,
    strict: bool = True,
    run_fn: Callable[[argparse.Namespace], dict[str, Any]] = nav.run,
) -> dict[str, Any]:
    goals = _build_long_range_goals(
        count=attempts,
        min_distance_m=min_distance_m,
        max_distance_m=max_distance_m,
    )
    attempt_reports: list[dict[str, Any]] = []
    for index, goal in enumerate(goals):
        attempt_report = _run_single_navigation_goal(
            attempt_index=index,
            manifest_path=manifest,
            out_base=artifact_dir,
            goal=goal,
            domain_id_base=domain_id_base,
            phase=phase,
            record_video=record_video,
            run_fn=run_fn,
        )
        attempt_reports.append(attempt_report)
        if strict and not attempt_report["ok"]:
            break

    succeeded = [item for item in attempt_reports if item["ok"]]
    all_success = len(succeeded) == attempts
    return {
        "schema_version": "lingtu.mujoco.long_range_navigation.acceptance.v1",
        "mode": "native_navigation_long_range",
        "ok": all_success,
        "summary_text": _build_summary_text({
            "mode": "native_navigation_long_range",
            "ok": all_success,
            "attempts": len(attempt_reports),
            "required_attempts": attempts,
            "successes": len(succeeded),
            "failures": len(attempt_reports) - len(succeeded),
            "strict": strict,
            "attempts_report": attempt_reports,
        }),
        "attempts": len(attempt_reports),
        "required_attempts": attempts,
        "successes": len(succeeded),
        "failures": len(attempt_reports) - len(succeeded),
        "all_success": all_success and len(attempt_reports) == attempts,
        "strict": strict,
        "artifact_dir": str(artifact_dir),
        "attempts_report": attempt_reports,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", default=str(DEFAULT_MANIFEST))
    parser.add_argument("--attempts", type=int, default=10)
    parser.add_argument("--min-distance-m", type=float, default=50.0)
    parser.add_argument("--max-distance-m", type=float, default=70.0)
    parser.add_argument("--phase", choices=["motion", "no_motion", "both"], default="motion")
    parser.add_argument("--domain-id-base", type=int, default=220)
    parser.add_argument("--artifact-dir", default=str(ROOT / "artifacts" / "mujoco_long_range_navigation"))
    parser.add_argument("--record-video", action="store_true")
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--json-out", default=None)
    parser.add_argument("--html-summary", default="")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    artifact_dir = Path(args.artifact_dir).expanduser().resolve()
    report = run_long_range_acceptance(
        manifest=Path(args.manifest).expanduser().resolve(),
        attempts=int(args.attempts),
        min_distance_m=float(args.min_distance_m),
        max_distance_m=float(args.max_distance_m),
        phase=args.phase,
        artifact_dir=artifact_dir,
        domain_id_base=int(args.domain_id_base),
        record_video=bool(args.record_video),
        strict=bool(args.strict),
    )
    output = (
        Path(args.json_out).expanduser().resolve()
        if args.json_out is not None
        else artifact_dir / "long_range_acceptance_report.json"
    )
    _write_json(output, report)
    html_summary = Path(args.html_summary).expanduser().resolve() if args.html_summary else None
    if html_summary is not None:
        html_summary.parent.mkdir(parents=True, exist_ok=True)
        html_summary.write_text(_build_summary_html(report), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=True, indent=2, sort_keys=True))
    return 0 if report["all_success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

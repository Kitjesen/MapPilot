#!/usr/bin/env python3
"""Validate PCT saved-map navigation after runtime relocalization.

This gate composes the Phase 3/4 product evidence:

same-source tomogram + saved map -> relocalization LOCKED -> PCT preview ->
native localPlanner/pathFollower -> MuJoCo closed-loop motion.

It is intentionally simulation-only and delegates motion evidence to
``native_pct_mujoco_gate.py`` instead of duplicating the local planner harness.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from drivers.sim.mujoco_scene_metadata import write_scene_obstacle_metadata  # noqa: E402

DEFAULT_SAVED_MAP_GOAL = [4.5, 3.0]
DEFAULT_RELOCALIZE_REPORT_MAX_AGE_S = 86_400.0


def _latest(patterns: tuple[str, ...]) -> Path | None:
    for pattern in patterns:
        existing = [path for path in ROOT.glob(pattern) if path.exists()]
        if existing:
            return max(existing, key=lambda path: path.stat().st_mtime)
    return None


def _workspace_path(path: Path) -> Path:
    path = path.expanduser()
    if path.is_absolute():
        return path
    return (ROOT / path).resolve()


def _expected_tomogram_from_relocalize_report(report: dict[str, Any]) -> Path | None:
    map_value = str(report.get("map_pcd") or "").strip()
    if not map_value:
        return None
    map_path = _workspace_path(Path(map_value))
    return map_path.parent / "tomogram.pickle"


def _resolve_tomogram_from_relocalize_report(report: dict[str, Any]) -> Path | None:
    tomogram = _expected_tomogram_from_relocalize_report(report)
    return tomogram if tomogram is not None and tomogram.exists() else None


def _resolve_tomogram(
    path: Path | None,
    *,
    relocalize_report: dict[str, Any] | None = None,
) -> Path:
    expected = (
        _expected_tomogram_from_relocalize_report(relocalize_report)
        if relocalize_report is not None
        else None
    )
    if path is not None:
        resolved = _workspace_path(path)
        if expected is not None and resolved.resolve() != expected.resolve():
            raise ValueError(
                "explicit tomogram does not match relocalize_report.map_pcd sibling "
                f"tomogram: {resolved} != {expected}"
            )
        if not resolved.is_file():
            raise FileNotFoundError(f"tomogram not found: {resolved}")
        return resolved
    if expected is not None and expected.exists():
        return expected
    if expected is not None:
        raise FileNotFoundError(
            "relocalize_report.map_pcd sibling tomogram missing: "
            f"{expected}"
        )
    latest = _latest(
        (
            "artifacts/server_sim_closure/cli_tare_endpoint_mujoco_live*/**/same_source_map/tomogram.pickle",
            "artifacts/server_sim_closure/mujoco_tare_exploration*/**/same_source_map/tomogram.pickle",
            "artifacts/server_sim_closure/cli_explore_endpoint_mujoco_live*/**/same_source_map/tomogram.pickle",
            "artifacts/server_sim_closure/mujoco_fastlio2_live*/**/same_source_map/tomogram.pickle",
            "artifacts/server_sim_closure/**/same_source_map/tomogram.pickle",
        )
    )
    if latest is None:
        raise FileNotFoundError("no same-source tomogram.pickle found")
    return latest


def _resolve_relocalize_report(path: Path | None) -> Path:
    if path is not None:
        return path
    latest = _latest(
        (
            "artifacts/server_sim_closure/saved_map_relocalize*/report.json",
            "artifacts/saved_map_relocalize*/report.json",
        )
    )
    if latest is None:
        raise FileNotFoundError("no saved-map relocalization report found")
    return latest


def _metadata_for_tomogram(tomogram: Path) -> Path | None:
    candidate = tomogram.parent / "metadata.json"
    return candidate if candidate.exists() else None


def _map_pcd_from_relocalize_report(report: dict[str, Any]) -> Path | None:
    map_value = str(report.get("map_pcd") or "").strip()
    if not map_value:
        return None
    return _workspace_path(Path(map_value))


def _scene_for_tomogram(tomogram: Path, explicit: Path | None) -> Path:
    if explicit is not None:
        return explicit
    metadata = _metadata_for_tomogram(tomogram)
    if metadata is not None:
        try:
            payload = json.loads(metadata.read_text(encoding="utf-8"))
            world = Path(str(payload.get("world") or ""))
            if world.exists():
                return world
        except Exception:
            pass
    return ROOT / "sim/worlds/mujoco/industrial_park_scene.xml"


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _safe_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if result == result else None


def _report_freshness(
    path: Path,
    payload: dict[str, Any],
    *,
    max_age_s: float | None,
) -> dict[str, Any]:
    if max_age_s is not None and max_age_s <= 0:
        max_age_s = None
    checked_at = time.time()
    try:
        mtime = path.stat().st_mtime
    except OSError:
        mtime = 0.0
    generated_at = _safe_float(payload.get("generated_at"))
    reference_time = generated_at if generated_at is not None else mtime
    age_s = max(0.0, checked_at - reference_time)
    blockers: list[str] = []
    if max_age_s is not None and age_s > max_age_s:
        blockers.append(f"report_age_s {age_s:.3f} > max_age_s {max_age_s:.3f}")
    if generated_at is not None and generated_at - checked_at > 300.0:
        blockers.append("generated_at is too far in the future")
    return {
        "checked": True,
        "path": str(path),
        "mtime": mtime,
        "generated_at": generated_at,
        "age_s": round(age_s, 3),
        "max_age_s": max_age_s,
        "fresh": not blockers,
        "stale": bool(blockers),
        "blockers": blockers,
    }


def _relocalize_report_freshness(
    args: argparse.Namespace,
    path: Path,
    payload: dict[str, Any],
) -> dict[str, Any]:
    return _report_freshness(
        path,
        payload,
        max_age_s=getattr(
            args,
            "max_relocalize_report_age_s",
            DEFAULT_RELOCALIZE_REPORT_MAX_AGE_S,
        ),
    )


def _sha256_file(path: Path | None) -> str:
    if path is None or not path.is_file():
        return ""
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _same_source_metadata_for_map(map_pcd: Path | None) -> dict[str, Any]:
    if map_pcd is None:
        return {}
    metadata = map_pcd.parent / "metadata.json"
    if not metadata.is_file():
        return {}
    try:
        payload = json.loads(metadata.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def _metadata_artifact(metadata: dict[str, Any], name: str) -> dict[str, Any]:
    artifacts = metadata.get("artifacts")
    if not isinstance(artifacts, dict):
        return {}
    artifact = artifacts.get(name)
    return dict(artifact) if isinstance(artifact, dict) else {}


def _same_source_hash_identity(
    *,
    map_pcd: Path | None,
    tomogram: Path,
    source_tomogram_sha256: str,
    map_artifacts: dict[str, Any],
) -> tuple[dict[str, Any], list[str]]:
    relocalize_metadata = _same_source_metadata_for_map(map_pcd)
    relocalize_map = _metadata_artifact(relocalize_metadata, "map_pcd")
    relocalize_tomogram = _metadata_artifact(relocalize_metadata, "tomogram")
    source_assets = map_artifacts.get("assets") if isinstance(map_artifacts.get("assets"), dict) else {}
    source_map = (
        dict(source_assets.get("map_pcd"))
        if isinstance(source_assets.get("map_pcd"), dict)
        else {}
    )
    source_tomogram = (
        dict(source_assets.get("tomogram"))
        if isinstance(source_assets.get("tomogram"), dict)
        else {}
    )

    relocalize_map_sha = str(relocalize_map.get("sha256") or "")
    relocalize_tomogram_sha = str(relocalize_tomogram.get("sha256") or "")
    relocalize_tomogram_source_sha = str(
        relocalize_tomogram.get("source_map_sha256") or ""
    )
    source_map_sha = str(source_map.get("sha256") or "")
    source_tomogram_sha = str(source_tomogram.get("sha256") or "")
    source_tomogram_source_sha = str(source_tomogram.get("source_map_sha256") or "")
    actual_map_sha = _sha256_file(map_pcd)
    actual_tomogram_sha = _sha256_file(tomogram)

    checks = {
        "relocalization_metadata_loads": bool(relocalize_metadata),
        "source_map_sha256_matches_relocalization": bool(
            source_map_sha and relocalize_map_sha and source_map_sha == relocalize_map_sha
        ),
        "source_tomogram_sha256_matches_relocalization": bool(
            source_tomogram_sha
            and relocalize_tomogram_sha
            and source_tomogram_sha == relocalize_tomogram_sha
        ),
        "source_tomogram_file_sha256_matches_metadata": bool(
            source_tomogram_sha256
            and source_tomogram_sha
            and source_tomogram_sha256 == source_tomogram_sha
        ),
        "relocalization_map_file_sha256_matches_metadata": bool(
            actual_map_sha and relocalize_map_sha and actual_map_sha == relocalize_map_sha
        ),
        "relocalization_tomogram_file_sha256_matches_metadata": bool(
            actual_tomogram_sha
            and relocalize_tomogram_sha
            and actual_tomogram_sha == relocalize_tomogram_sha
        ),
        "relocalization_tomogram_source_map_sha256_matches_map": bool(
            relocalize_tomogram_source_sha
            and relocalize_map_sha
            and relocalize_tomogram_source_sha == relocalize_map_sha
        ),
        "source_tomogram_source_map_sha256_matches_map": bool(
            source_tomogram_source_sha
            and source_map_sha
            and source_tomogram_source_sha == source_map_sha
        ),
    }
    blockers = [
        f"same-source hash identity failed: {name}"
        for name, ok in checks.items()
        if not ok
    ]
    return {
        "ok": not blockers,
        "checks": checks,
        "metadata": {
            "relocalization": {
                "schema_version": relocalize_metadata.get("schema_version") or "",
                "path": str(map_pcd.parent / "metadata.json") if map_pcd is not None else "",
            },
            "source": map_artifacts.get("metadata") or {},
        },
        "hashes": {
            "relocalization_map_sha256": relocalize_map_sha,
            "relocalization_tomogram_sha256": relocalize_tomogram_sha,
            "relocalization_tomogram_source_map_sha256": relocalize_tomogram_source_sha,
            "source_map_sha256": source_map_sha,
            "source_tomogram_sha256": source_tomogram_sha,
            "source_tomogram_source_map_sha256": source_tomogram_source_sha,
            "actual_map_sha256": actual_map_sha,
            "actual_tomogram_sha256": actual_tomogram_sha,
            "source_report_tomogram_sha256": source_tomogram_sha256,
        },
    }, blockers


def _validate_relocalization(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    service = report.get("service") or {}
    localizer = report.get("localizer") or {}
    if report.get("ok") is not True:
        blockers.append("relocalization report.ok is not true")
    if report.get("runtime_relocalization_validated") is not True:
        blockers.append("runtime_relocalization_validated is not true")
    if service.get("success") is not True:
        blockers.append("/nav/relocalize service did not succeed")
    if str(localizer.get("latest_health_state") or "").upper() != "LOCKED":
        blockers.append("localizer latest_health_state is not LOCKED")
    if int(localizer.get("saved_map_cloud_points_latest") or 0) < 1000:
        blockers.append("saved_map_cloud_points_latest below threshold")
    return not blockers, blockers, {
        "service": service,
        "latest_health_state": localizer.get("latest_health_state"),
        "latest_health": localizer.get("latest_health"),
        "saved_map_cloud_points_latest": localizer.get("saved_map_cloud_points_latest"),
        "map_to_odom_xy_m": localizer.get("map_to_odom_xy_m"),
        "map_to_odom_z_abs_m": localizer.get("map_to_odom_z_abs_m"),
    }


def _relocalization_prerequisite_blockers(
    *,
    relocalize_freshness: dict[str, Any],
    relocalization_ok: bool,
    relocalization_blockers: list[str],
) -> list[str]:
    blockers = list(relocalize_freshness.get("blockers") or [])
    blockers.extend(relocalization_blockers)
    if not relocalization_ok:
        blockers.append("saved-map relocalization prerequisite failed")
    return blockers


def _mark_relocalization_prerequisite_failed(
    report: dict[str, Any],
    *,
    blockers: list[str],
    relocalization_ok: bool,
) -> dict[str, Any]:
    report["plan_preview"] = {
        "ok": False,
        "skipped": True,
        "reason": "saved_map_relocalization_prerequisite_failed",
        "selected_planner": "",
        "fallback_reason": "",
        "path_count": 0,
    }
    report["native_gate"] = {
        "ok": False,
        "skipped": True,
        "reason": "saved_map_relocalization_prerequisite_failed",
    }
    report["contract_checks"] = {
        "relocalization_locked": relocalization_ok,
        "source_report_loads": False,
        "tomogram_matches_relocalization": None,
        "map_pcd_matches_relocalization": None,
        "scene_xml_matches_saved_map": None,
        "pct_no_fallback": None,
        "pct_native_backend_used": None,
        "pct_native_runtime_ok": None,
        "pct_optimizer_disabled": None,
        "pct_native_raw_path": None,
        "same_source_map_artifact": None,
        "same_source_hash_identity": None,
    }
    report["blockers"] = blockers
    report["ok"] = False
    return report


def _run_plan_preview(args: argparse.Namespace, *, tomogram: Path, out_path: Path) -> dict[str, Any]:
    cmd = [
        sys.executable,
        str(ROOT / "scripts/plan_preview.py"),
        "--planner",
        "pct",
        "--tomogram",
        str(tomogram),
        "--map-root",
        str(tomogram.parent),
        "--internal-only",
        "--strict",
        "--compact",
        "--max-endpoint-z-error-m",
        str(args.max_endpoint_z_error_m),
        "--timeout",
        str(args.preview_timeout_s),
    ]
    if args.start:
        cmd.extend(["--start", *[str(v) for v in args.start]])
    if args.goal:
        if not args.start:
            cmd.append("--use-last-pose")
        cmd.extend(["--goal", *[str(v) for v in args.goal]])
    started = time.time()
    proc = subprocess.run(
        cmd,
        cwd=str(ROOT),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=max(5.0, float(args.preview_timeout_s) + 10.0),
        check=False,
    )
    stdout = proc.stdout.strip()
    parsed: dict[str, Any] = {}
    for line in reversed(stdout.splitlines()):
        if line.strip().startswith("{"):
            parsed = json.loads(line)
            break
    parsed.setdefault("command", cmd)
    parsed.setdefault("returncode", proc.returncode)
    parsed.setdefault("stderr_tail", proc.stderr[-2000:])
    parsed.setdefault("wall_s", round(time.time() - started, 3))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(parsed, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return parsed


def _point_xyz(point: dict[str, Any]) -> list[float]:
    return [float(point.get("x", 0.0)), float(point.get("y", 0.0)), float(point.get("z", 0.0))]


def _select_preview_case(preview: dict[str, Any]) -> dict[str, Any]:
    returncode = preview.get("returncode")
    if returncode not in (None, 0):
        stderr = str(preview.get("stderr_tail") or "").strip()
        suffix = f": {stderr[-500:]}" if stderr else ""
        raise RuntimeError(
            f"PCT plan preview command failed with returncode {returncode}{suffix}"
        )
    for case in preview.get("cases") or []:
        body = case.get("preview") or {}
        if case.get("feasible") is True and body.get("selected_planner") == "pct":
            path = body.get("path") or []
            if len(path) >= 2:
                return case
    raise RuntimeError("no feasible PCT preview case with a non-empty path")


def _pct_mode_evidence(
    preview: dict[str, Any],
    case: dict[str, Any],
) -> dict[str, Any]:
    optimizer = case.get("pct_optimizer_enabled")
    if optimizer is None:
        optimizer = preview.get("pct_optimizer_enabled")
    path_mode = str(
        case.get("pct_planner_path_mode")
        or preview.get("pct_planner_path_mode")
        or ""
    )
    return {
        "pct_optimizer_enabled": optimizer,
        "pct_planner_path_mode": path_mode,
    }


def _pct_native_raw_path_blockers(mode: dict[str, Any]) -> list[str]:
    blockers: list[str] = []
    if mode.get("pct_optimizer_enabled") is not False:
        blockers.append("PCT optimizer mode evidence is not disabled")
    if mode.get("pct_planner_path_mode") != "native_astar_raw_path":
        blockers.append("PCT planner path mode is not native_astar_raw_path")
    return blockers


def _build_source_report(
    *,
    preview: dict[str, Any],
    case: dict[str, Any],
    tomogram: Path,
    scene_xml: Path,
    map_pcd: Path | None,
    map_metadata: Path | None,
    obstacle_metadata: Path,
    output: Path,
    route_name: str,
) -> dict[str, Any]:
    body = case.get("preview") or {}
    path = [_point_xyz(point) for point in body.get("path") or []]
    start = path[0]
    goal = path[-1]
    pct_runtime = preview.get("pct_runtime_libs") or {}
    pct_mode = _pct_mode_evidence(preview, case)
    source = {
        "schema_version": "lingtu.pct_saved_map_navigation_source.v1",
        "validation_level": "saved_map_relocalized_pct_plan_preview",
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "route": route_name,
        "cases": [
            {
                "route": route_name,
                "passed": True,
                "assets": {
                    "scene_xml": str(scene_xml),
                    "tomogram": str(tomogram),
                    "map_pcd": str(map_pcd or ""),
                    "metadata": str(obstacle_metadata),
                    "map_metadata": str(map_metadata or ""),
                    "start": start,
                    "goal": goal,
                },
                "selection": {
                    "primary_planner": "pct",
                    "selected_planner": "pct",
                    "fallback_used": False,
                    "selected_route_ok": True,
                    "policy": "saved_map_relocalized_pct_no_fallback",
                },
                "path_safety": body.get("path_safety") or {"ok": True},
                "planning": [
                    {
                        "planner": "pct",
                        "backend_class": case.get("backend_class") or body.get("backend_class") or "_PCTBackend",
                        "native_backend_used": bool(pct_runtime.get("ok", True)),
                        "native_runtime": pct_runtime or {"ok": True},
                        **pct_mode,
                        "plan_ms": body.get("plan_ms"),
                        "route_ok": True,
                        "path_safety": body.get("path_safety") or {"ok": True},
                        "start": start,
                        "goal": goal,
                        "path": path,
                    }
                ],
            }
        ],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(source, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return source


def _run_native_gate(args: argparse.Namespace, *, source_report: Path, out_path: Path) -> dict[str, Any]:
    cmd = [
        sys.executable,
        str(ROOT / "sim/scripts/native_pct_mujoco_gate.py"),
        "--source-report",
        str(source_report),
        "--route",
        str(args.route_name),
        "--planner",
        "pct",
        "--artifact-dir",
        str(out_path.parent),
        "--json-out",
        str(out_path),
        "--ros-domain-id",
        str(args.ros_domain_id),
        "--timeout-s",
        str(args.timeout_s),
        "--sim-vehicle",
        str(args.sim_vehicle),
        "--min-route-progress-ratio",
        str(args.min_route_progress_ratio),
        "--near-field-stop-distance",
        str(args.near_field_stop_distance),
        "--goal-threshold-m",
        str(args.goal_threshold_m),
        "--goal-clear-range",
        str(args.goal_clear_range),
        "--waypoint-threshold-m",
        str(args.waypoint_threshold_m),
        "--waypoint-safety-margin",
        str(args.waypoint_safety_margin),
        "--waypoint-detour-margin",
        str(args.waypoint_detour_margin),
        "--waypoint-collision-sample-step",
        str(args.waypoint_collision_sample_step),
        "--strict",
    ]
    if args.video_out:
        cmd.extend(
            [
                "--video-out",
                str(args.video_out),
                "--video-layout",
                str(args.video_layout),
                "--video-width",
                str(args.video_width),
                "--video-height",
                str(args.video_height),
                "--video-fps",
                str(args.video_fps),
            ]
        )
    proc = subprocess.run(
        cmd,
        cwd=str(ROOT),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=max(float(args.timeout_s) + 90.0, 120.0),
        check=False,
    )
    if out_path.exists():
        report = _load_json(out_path)
    else:
        report = {"ok": False}
    report["command"] = cmd
    report["process_returncode"] = proc.returncode
    report["stdout_tail"] = proc.stdout[-2000:]
    report["stderr_tail"] = proc.stderr[-2000:]
    out_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def _same_resolved_path(left: Path | None, right: Path | None) -> bool:
    if left is None or right is None:
        return False
    try:
        return left.resolve() == right.resolve()
    except Exception:
        return str(left) == str(right)


def _validate_source_identity(
    *,
    source_report: Path,
    route_name: str,
    tomogram: Path,
    map_pcd: Path | None,
    scene_xml: Path,
) -> dict[str, Any]:
    from sim.scripts.native_pct_mujoco_gate import (
        _load_pct_route,
        _planner_contract,
        _source_map_artifacts,
    )

    route = _load_pct_route(source_report, route=route_name, planner="pct")
    source_contract = _planner_contract(route, "pct")
    map_artifacts = _source_map_artifacts(route)
    source_tomogram = (
        _workspace_path(Path(source_contract["tomogram"]))
        if source_contract.get("tomogram")
        else None
    )
    route_map_pcd_raw = (
        route.case.get("assets", {}).get("map_pcd")
        if isinstance(route.case.get("assets"), dict)
        else ""
    )
    route_map_pcd = _workspace_path(Path(str(route_map_pcd_raw))) if route_map_pcd_raw else None

    blockers: list[str] = []
    if not _same_resolved_path(source_tomogram, tomogram):
        blockers.append("source report tomogram does not match relocalized saved-map tomogram")
    if map_pcd is not None and not _same_resolved_path(route_map_pcd, map_pcd):
        blockers.append("source report map_pcd does not match relocalization map_pcd")
    if not _same_resolved_path(route.scene_xml, scene_xml):
        blockers.append("source report scene_xml does not match saved-map scene")
    if not map_artifacts.get("ok"):
        blockers.append("source report lacks same-source map/tomogram artifact proof")
    hash_identity, hash_blockers = _same_source_hash_identity(
        map_pcd=map_pcd,
        tomogram=tomogram,
        source_tomogram_sha256=str(source_contract["tomogram_sha256"] or ""),
        map_artifacts=map_artifacts,
    )
    blockers.extend(hash_blockers)
    blockers.extend(_pct_native_raw_path_blockers(source_contract))

    return {
        "source_planning_contract": source_contract,
        "map_artifacts": map_artifacts,
        "same_source_hash_identity": hash_identity,
        "contract_checks": {
            "source_report_loads": True,
            "tomogram_matches_relocalization": _same_resolved_path(
                source_tomogram, tomogram
            ),
            "map_pcd_matches_relocalization": (
                _same_resolved_path(route_map_pcd, map_pcd)
                if map_pcd is not None
                else None
            ),
            "scene_xml_matches_saved_map": _same_resolved_path(
                route.scene_xml, scene_xml
            ),
            "pct_no_fallback": not source_contract["fallback_used"],
            "pct_native_backend_used": source_contract["native_backend_used"],
            "pct_native_runtime_ok": source_contract["native_runtime_ok"],
            "pct_optimizer_disabled": (
                source_contract.get("pct_optimizer_enabled") is False
            ),
            "pct_native_raw_path": (
                source_contract.get("pct_planner_path_mode")
                == "native_astar_raw_path"
            ),
            "same_source_map_artifact": map_artifacts.get("ok") is True,
            "same_source_hash_identity": hash_identity.get("ok") is True,
        },
        "blockers": blockers,
    }


def _write_report_if_requested(args: argparse.Namespace, report: dict[str, Any]) -> None:
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )


def _contract_only_report(args: argparse.Namespace) -> dict[str, Any]:
    run_dir = args.run_dir
    run_dir.mkdir(parents=True, exist_ok=True)

    report: dict[str, Any] = {
        "schema_version": "lingtu.pct_saved_map_navigation_gate.v1",
        "execution_mode": "contract_only",
        "validation_only": True,
        "validation_level": "saved_map_relocalized_pct_contract",
        "ok": False,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "tomogram": str(args.tomogram or ""),
        "map_pcd": "",
        "scene_xml": str(args.scene_xml or ""),
        "map_metadata": "",
        "relocalize_report": str(args.relocalize_report or ""),
        "source_report": str(args.source_report or ""),
        "run_dir": str(run_dir),
        "claim_boundary": "contract_only_no_pct_preview_or_mujoco_motion",
        "validation_limitations": [
            "Validates saved-map/PCT source-report binding only.",
            "Does not run PCT plan_preview.",
            "Does not launch native_pct_mujoco_gate.py motion execution.",
            "Does not prove localPlanner/pathFollower cmd_vel or MuJoCo goal reaching.",
        ],
        "blockers": [],
    }

    try:
        relocalize_report_path = _resolve_relocalize_report(args.relocalize_report)
        relocalize = _load_json(relocalize_report_path)
        relocalize_freshness = _relocalize_report_freshness(
            args,
            relocalize_report_path,
            relocalize,
        )
        tomogram = _resolve_tomogram(args.tomogram, relocalize_report=relocalize)
        scene_xml = _scene_for_tomogram(tomogram, args.scene_xml)
        map_metadata = _metadata_for_tomogram(tomogram)
        map_pcd = _map_pcd_from_relocalize_report(relocalize)
        report.update(
            {
                "tomogram": str(tomogram),
                "map_pcd": str(map_pcd or ""),
                "scene_xml": str(scene_xml),
                "map_metadata": str(map_metadata or ""),
                "relocalize_report": str(relocalize_report_path),
                "relocalize_report_freshness": relocalize_freshness,
            }
        )
        reloc_ok, reloc_blockers, reloc_summary = _validate_relocalization(relocalize)
        report["relocalization"] = {
            "ok": reloc_ok,
            "blockers": reloc_blockers,
            **reloc_summary,
        }
        blockers = list(relocalize_freshness.get("blockers") or [])
        blockers.extend(reloc_blockers)
        if not reloc_ok:
            blockers.append("saved-map relocalization prerequisite failed")
        if args.source_report is None:
            blockers.append("contract-only mode requires --source-report")
        else:
            from sim.scripts.native_pct_mujoco_gate import (
                _load_pct_route,
                _planner_contract,
                _source_map_artifacts,
            )

            source_report = _workspace_path(args.source_report)
            route = _load_pct_route(source_report, route=args.route_name, planner="pct")
            source_contract = _planner_contract(route, "pct")
            map_artifacts = _source_map_artifacts(route)
            source_tomogram = (
                _workspace_path(Path(source_contract["tomogram"]))
                if source_contract.get("tomogram")
                else None
            )
            route_map_pcd_raw = (
                route.case.get("assets", {}).get("map_pcd")
                if isinstance(route.case.get("assets"), dict)
                else ""
            )
            route_map_pcd = _workspace_path(Path(str(route_map_pcd_raw))) if route_map_pcd_raw else None

            if not _same_resolved_path(source_tomogram, tomogram):
                blockers.append("source report tomogram does not match relocalized saved-map tomogram")
            if map_pcd is not None and not _same_resolved_path(route_map_pcd, map_pcd):
                blockers.append("source report map_pcd does not match relocalization map_pcd")
            if not _same_resolved_path(route.scene_xml, scene_xml):
                blockers.append("source report scene_xml does not match saved-map scene")
            if not map_artifacts.get("ok"):
                blockers.append("source report lacks same-source map/tomogram artifact proof")
            hash_identity, hash_blockers = _same_source_hash_identity(
                map_pcd=map_pcd,
                tomogram=tomogram,
                source_tomogram_sha256=str(source_contract["tomogram_sha256"] or ""),
                map_artifacts=map_artifacts,
            )
            blockers.extend(hash_blockers)

            report["plan_preview"] = {
                "ok": True,
                "validation_only": True,
                "selected_planner": source_contract["selected_planner"],
                "fallback_reason": "" if not source_contract["fallback_used"] else "fallback_used",
                "path_count": len(route.path),
                "source": "source_report_contract_only",
            }
            report["native_gate"] = {
                "ok": True,
                "validation_only": True,
                "execution_mode": "contract_only",
                "selected_planner": source_contract["selected_planner"],
                "fallback_used": source_contract["fallback_used"],
                "pct_native_backend_used": source_contract["native_backend_used"],
                "pct_runtime_ok": source_contract["native_runtime_ok"],
                "reached_goal": False,
                "claim_boundary": "contract_only_no_mujoco_motion",
            }
            report["source_planning_contract"] = source_contract
            report["map_artifacts"] = map_artifacts
            report["same_source_hash_identity"] = hash_identity
            report["contract_checks"] = {
                "relocalization_locked": reloc_ok,
                "source_report_loads": True,
                "tomogram_matches_relocalization": _same_resolved_path(
                    source_tomogram, tomogram
                ),
                "map_pcd_matches_relocalization": (
                    _same_resolved_path(route_map_pcd, map_pcd)
                    if map_pcd is not None
                    else None
                ),
                "scene_xml_matches_saved_map": _same_resolved_path(
                    route.scene_xml, scene_xml
                ),
                "pct_no_fallback": not source_contract["fallback_used"],
                "pct_native_backend_used": source_contract["native_backend_used"],
                "pct_native_runtime_ok": source_contract["native_runtime_ok"],
                "pct_optimizer_disabled": (
                    source_contract.get("pct_optimizer_enabled") is False
                ),
                "pct_native_raw_path": (
                    source_contract.get("pct_planner_path_mode")
                    == "native_astar_raw_path"
                ),
                "same_source_map_artifact": map_artifacts.get("ok") is True,
                "same_source_hash_identity": hash_identity.get("ok") is True,
            }
            blockers.extend(_pct_native_raw_path_blockers(source_contract))
        report["blockers"] = blockers
        report["ok"] = not blockers
    except Exception as exc:
        report["ok"] = False
        report["error"] = str(exc)
        report["blockers"] = [str(exc)]
        report["contract_checks"] = {
            "relocalization_locked": False,
            "source_report_loads": False,
            "tomogram_matches_relocalization": False,
            "map_pcd_matches_relocalization": False,
            "scene_xml_matches_saved_map": False,
            "pct_no_fallback": False,
            "pct_native_backend_used": None,
            "pct_native_runtime_ok": None,
            "pct_optimizer_disabled": None,
            "pct_native_raw_path": None,
            "same_source_map_artifact": False,
            "same_source_hash_identity": False,
        }

    _write_report_if_requested(args, report)
    return report


def run_gate(args: argparse.Namespace) -> dict[str, Any]:
    if getattr(args, "contract_only", False):
        return _contract_only_report(args)

    run_dir = args.run_dir
    run_dir.mkdir(parents=True, exist_ok=True)
    relocalize_report_path = _resolve_relocalize_report(args.relocalize_report)
    relocalize = _load_json(relocalize_report_path)
    relocalize_freshness = _relocalize_report_freshness(
        args,
        relocalize_report_path,
        relocalize,
    )
    map_pcd = _map_pcd_from_relocalize_report(relocalize)

    report: dict[str, Any] = {
        "schema_version": "lingtu.pct_saved_map_navigation_gate.v1",
        "validation_level": "saved_map_relocalized_pct_navigation",
        "ok": False,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "tomogram": str(args.tomogram or ""),
        "map_pcd": str(map_pcd or ""),
        "scene_xml": str(args.scene_xml or ""),
        "map_metadata": "",
        "relocalize_report": str(relocalize_report_path),
        "relocalize_report_freshness": relocalize_freshness,
        "run_dir": str(run_dir),
        "blockers": [],
    }
    try:
        reloc_ok, reloc_blockers, reloc_summary = _validate_relocalization(relocalize)
        report["relocalization"] = {
            "ok": reloc_ok,
            "blockers": reloc_blockers,
            **reloc_summary,
        }
        prerequisite_blockers = _relocalization_prerequisite_blockers(
            relocalize_freshness=relocalize_freshness,
            relocalization_ok=reloc_ok,
            relocalization_blockers=reloc_blockers,
        )
        if prerequisite_blockers:
            _mark_relocalization_prerequisite_failed(
                report,
                blockers=prerequisite_blockers,
                relocalization_ok=reloc_ok,
            )
            _write_report_if_requested(args, report)
            return report

        tomogram = _resolve_tomogram(args.tomogram, relocalize_report=relocalize)
        scene_xml = _scene_for_tomogram(tomogram, args.scene_xml)
        map_metadata = _metadata_for_tomogram(tomogram)
        report.update(
            {
                "tomogram": str(tomogram),
                "scene_xml": str(scene_xml),
                "map_metadata": str(map_metadata or ""),
            }
        )

        obstacle_metadata_path = run_dir / "scene_obstacles.json"
        obstacle_metadata = write_scene_obstacle_metadata(
            scene_xml=scene_xml,
            source_map_metadata=map_metadata,
            output=obstacle_metadata_path,
        )
        report["scene_obstacle_metadata"] = {
            "path": str(obstacle_metadata_path),
            "obstacle_count": obstacle_metadata.get("obstacle_count"),
        }

        preview_path = run_dir / "pct_plan_preview.json"
        preview = _run_plan_preview(args, tomogram=tomogram, out_path=preview_path)
        case = _select_preview_case(preview)
        pct_mode = _pct_mode_evidence(preview, case)
        report["plan_preview"] = {
            "ok": bool(preview.get("ok")),
            "path": str(preview_path),
            "selected_case": case.get("name"),
            "selected_planner": (case.get("preview") or {}).get("selected_planner"),
            "fallback_reason": (case.get("preview") or {}).get("fallback_reason"),
            "path_count": len((case.get("preview") or {}).get("path") or []),
            **pct_mode,
            "pct_runtime_libs": preview.get("pct_runtime_libs"),
        }
        preview_mode_blockers = _pct_native_raw_path_blockers(report["plan_preview"])
        if preview_mode_blockers:
            report["native_gate"] = {
                "ok": False,
                "skipped": True,
                "reason": "pct_plan_preview_mode_contract_failed",
            }
            report["blockers"] = preview_mode_blockers
            report["ok"] = False
            _write_report_if_requested(args, report)
            return report

        source_report_path = run_dir / "pct_saved_map_source_report.json"
        _build_source_report(
            preview=preview,
            case=case,
            tomogram=tomogram,
            scene_xml=scene_xml,
            map_pcd=map_pcd,
            map_metadata=map_metadata,
            obstacle_metadata=obstacle_metadata_path,
            output=source_report_path,
            route_name=args.route_name,
        )
        report["source_report"] = str(source_report_path)
        report["native_gate_parameters"] = {
            "timeout_s": float(args.timeout_s),
            "min_route_progress_ratio": float(args.min_route_progress_ratio),
            "near_field_stop_distance": float(args.near_field_stop_distance),
            "goal_threshold_m": float(args.goal_threshold_m),
            "goal_clear_range": float(args.goal_clear_range),
            "waypoint_threshold_m": float(args.waypoint_threshold_m),
            "waypoint_safety_margin": float(args.waypoint_safety_margin),
            "waypoint_detour_margin": float(args.waypoint_detour_margin),
            "waypoint_collision_sample_step": float(args.waypoint_collision_sample_step),
        }

        source_identity = _validate_source_identity(
            source_report=source_report_path,
            route_name=args.route_name,
            tomogram=tomogram,
            map_pcd=map_pcd,
            scene_xml=scene_xml,
        )
        report["source_planning_contract"] = source_identity["source_planning_contract"]
        report["map_artifacts"] = source_identity["map_artifacts"]
        report["same_source_hash_identity"] = source_identity["same_source_hash_identity"]
        report["contract_checks"] = {
            "relocalization_locked": reloc_ok,
            **source_identity["contract_checks"],
        }

        pre_native_blockers = list(prerequisite_blockers)
        if preview.get("ok") is not True:
            pre_native_blockers.append("PCT plan preview report.ok is not true")
        if report["plan_preview"]["selected_planner"] != "pct":
            pre_native_blockers.append("PCT preview selected planner is not pct")
        if report["plan_preview"]["fallback_reason"]:
            pre_native_blockers.append("PCT preview reported fallback_reason")
        pre_native_blockers.extend(
            _pct_native_raw_path_blockers(report["plan_preview"])
        )
        pre_native_blockers.extend(source_identity["blockers"])
        if pre_native_blockers:
            report["native_gate"] = {
                "ok": False,
                "skipped": True,
                "reason": "pre_native_contract_failed",
            }
            report["blockers"] = pre_native_blockers
            report["ok"] = False
            _write_report_if_requested(args, report)
            return report

        native_report_path = run_dir / "native_pct/report.json"
        native = _run_native_gate(args, source_report=source_report_path, out_path=native_report_path)
        report["native_gate"] = native

        blockers = list(pre_native_blockers)
        if native.get("ok") is not True:
            blockers.append("native PCT MuJoCo closed-loop gate failed")
        if native.get("selected_planner") != "pct":
            blockers.append("native selected_planner is not pct")
        if native.get("fallback_used") is not False:
            blockers.append("native fallback_used is not false")
        if native.get("reached_goal") is not True:
            blockers.append("native gate did not reach goal")
        report["blockers"] = blockers
        report["ok"] = not blockers
    except Exception as exc:
        report["ok"] = False
        report["error"] = str(exc)
        report["blockers"] = [str(exc)]

    _write_report_if_requested(args, report)
    return report


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tomogram", type=Path, default=None)
    parser.add_argument("--relocalize-report", type=Path, default=None)
    parser.add_argument("--scene-xml", type=Path, default=None)
    parser.add_argument("--source-report", type=Path, default=None)
    parser.add_argument(
        "--contract-only",
        action="store_true",
        help=(
            "Validate saved-map relocalization/source-report binding without "
            "running PCT plan_preview or MuJoCo motion."
        ),
    )
    parser.add_argument("--run-dir", type=Path, default=ROOT / "artifacts/server_sim_closure/pct_saved_map_navigation")
    parser.add_argument("--json-out", type=Path, default=ROOT / "artifacts/server_sim_closure/pct_saved_map_navigation/report.json")
    parser.add_argument("--route-name", default="saved_map_internal")
    parser.add_argument("--ros-domain-id", default=os.environ.get("ROS_DOMAIN_ID", "161"))
    parser.add_argument("--preview-timeout-s", type=float, default=45.0)
    parser.add_argument("--max-endpoint-z-error-m", type=float, default=1.0)
    parser.add_argument(
        "--max-relocalize-report-age-s",
        type=float,
        default=DEFAULT_RELOCALIZE_REPORT_MAX_AGE_S,
        help=(
            "Maximum accepted age for the saved-map relocalization prerequisite "
            "report. Use 0 to disable only for controlled diagnostics."
        ),
    )
    parser.add_argument("--timeout-s", type=float, default=120.0)
    parser.add_argument("--min-route-progress-ratio", type=float, default=0.90)
    parser.add_argument("--near-field-stop-distance", type=float, default=0.35)
    parser.add_argument("--goal-threshold-m", type=float, default=0.50)
    parser.add_argument("--goal-clear-range", type=float, default=0.45)
    parser.add_argument("--waypoint-threshold-m", type=float, default=0.25)
    parser.add_argument("--waypoint-safety-margin", type=float, default=0.12)
    parser.add_argument("--waypoint-detour-margin", type=float, default=0.18)
    parser.add_argument("--waypoint-collision-sample-step", type=float, default=0.05)
    parser.add_argument("--sim-vehicle", choices=("quadruped", "omni_cart"), default="omni_cart")
    parser.add_argument("--start", nargs="+", type=float, default=None)
    parser.add_argument("--goal", nargs="+", type=float, default=DEFAULT_SAVED_MAP_GOAL)
    parser.add_argument("--video-out", type=Path, default=None)
    parser.add_argument("--video-layout", choices=("observer", "evidence", "scene_overlay"), default="scene_overlay")
    parser.add_argument("--video-width", type=int, default=1280)
    parser.add_argument("--video-height", type=int, default=720)
    parser.add_argument("--video-fps", type=float, default=20.0)
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    report = run_gate(args)
    print(json.dumps(report, indent=2, sort_keys=True))
    if args.strict and not report.get("ok"):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

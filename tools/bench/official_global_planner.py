#!/usr/bin/env python3
"""Run reproducible OctoPlanner3D regressions on pinned upstream PCD files."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import urllib.request
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MANIFEST = REPO_ROOT / "config" / "acceptance" / "official_global_planner.json"


class RegressionError(RuntimeError):
    """A regression case or its execution environment is invalid."""


def _json_dump(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _distance(lhs: list[float], rhs: list[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(lhs, rhs)))


def evaluate_planner_result(
    *,
    returncode: int,
    result: Any,
    expected: dict[str, Any],
    start: list[float],
    goal: list[float],
    start_snap_tolerance_m: float,
    goal_tolerance_m: float,
) -> list[str]:
    """Return semantic mismatches without confusing process failures with negative plans."""
    errors: list[str] = []
    if returncode not in (0, 2):
        return [f"planner process failed with exit code {returncode}"]
    if not isinstance(result, dict):
        return ["planner stdout was not a JSON object"]

    expect_ok = expected.get("ok")
    actual_ok = result.get("ok")
    if not isinstance(actual_ok, bool):
        errors.append("planner result has no boolean ok field")
        return errors
    if actual_ok != expect_ok:
        errors.append(f"expected ok={expect_ok!r}, got {actual_ok!r}")

    if expect_ok is False:
        if returncode != 2:
            errors.append(f"negative plan must exit 2, got {returncode}")
        expected_error = expected.get("error")
        if result.get("error") != expected_error:
            errors.append(f"expected error={expected_error!r}, got {result.get('error')!r}")
        if result.get("path") != []:
            errors.append("negative plan returned a non-empty or invalid path")
        if result.get("reached_goal") is not False:
            errors.append("negative plan did not report reached_goal=false")
        return errors

    if returncode != 0:
        errors.append(f"successful plan must exit 0, got {returncode}")
    if result.get("reached_goal") is not True:
        errors.append("successful plan did not report reached_goal=true")
    path = result.get("path")
    if not isinstance(path, list) or not path:
        errors.append("successful plan returned an empty path")
        return errors

    points: list[list[float]] = []
    for index, point in enumerate(path):
        if (
            not isinstance(point, list)
            or len(point) != 3
            or any(not isinstance(value, (int, float)) or not math.isfinite(value) for value in point)
        ):
            errors.append(f"path point {index} is not a finite XYZ triple")
            return errors
        points.append([float(value) for value in point])

    start_error = _distance(points[0], start)
    goal_error = _distance(points[-1], goal)
    goal_xy_error = math.hypot(points[-1][0] - goal[0], points[-1][1] - goal[1])
    goal_z_error = abs(points[-1][2] - goal[2])
    tolerance_epsilon = 1e-6
    if start_error > start_snap_tolerance_m + tolerance_epsilon:
        errors.append(
            f"first path point is {start_error:.6f} m from start, limit is {start_snap_tolerance_m:.6f} m"
        )
    if goal_error > goal_tolerance_m + tolerance_epsilon:
        errors.append(f"last path point is {goal_error:.6f} m from goal, limit is {goal_tolerance_m:.6f} m")

    diagnostics = result.get("diagnostics")
    if isinstance(diagnostics, dict):
        reported = {
            "goal_error_m": goal_error,
            "goal_xy_error_m": goal_xy_error,
            "goal_z_error_m": goal_z_error,
        }
        for key, recomputed in reported.items():
            value = diagnostics.get(key)
            if isinstance(value, (int, float)) and math.isfinite(value) and abs(float(value) - recomputed) > 1e-4:
                errors.append(f"reported {key}={value} disagrees with recomputed {recomputed:.6f}")
    return errors


def _run(command: list[str], *, timeout_s: float, stdin: bytes | None = None) -> subprocess.CompletedProcess[bytes]:
    return subprocess.run(command, input=stdin, capture_output=True, check=False, timeout=timeout_s)


def _require_binary(build_dir: Path, name: str) -> Path:
    binary = build_dir / name
    if not binary.is_file():
        raise RegressionError(f"required binary not found: {binary}")
    return binary


def _materialize_dataset(
    *,
    dataset: dict[str, Any],
    relative_path: str,
    data_dir: Path,
    upstream_dir: Path | None,
    download: bool,
    timeout_s: float,
) -> Path:
    commit = dataset["commit"]
    target = data_dir / commit / relative_path
    if target.is_file():
        return target
    target.parent.mkdir(parents=True, exist_ok=True)

    if upstream_dir is not None:
        completed = _run(
            ["git", "-C", str(upstream_dir), "show", f"{commit}:{relative_path}"],
            timeout_s=timeout_s,
        )
        if completed.returncode != 0:
            raise RegressionError(
                f"git show failed for {relative_path}: {completed.stderr.decode('utf-8', errors='replace').strip()}"
            )
        target.write_bytes(completed.stdout)
        return target

    if download:
        raw_url = dataset["raw_url_template"].format(commit=commit, path=relative_path)
        try:
            with urllib.request.urlopen(raw_url, timeout=timeout_s) as response:
                target.write_bytes(response.read())
        except OSError as exc:
            raise RegressionError(f"download failed for {raw_url}: {exc}") from exc
        return target

    raise RegressionError(
        f"dataset is not cached at {target}; pass --upstream-dir or explicitly allow --download"
    )


def _convert_map(
    *,
    converter: Path,
    pcd: Path,
    target: Path,
    profile: dict[str, Any],
    timeout_s: float,
) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    command = [
        str(converter),
        "--input",
        str(pcd),
        "--output",
        str(target),
        "--resolution",
        str(profile["resolution_m"]),
        "--support-dilation-cells",
        str(profile["support_dilation_cells"]),
        "--free-layers-above",
        str(profile["free_layers_above"]),
        "--free-dilation-cells",
        str(profile["free_dilation_cells"]),
    ]
    completed = _run(command, timeout_s=timeout_s)
    target.with_suffix(".converter.stdout.txt").write_bytes(completed.stdout)
    target.with_suffix(".converter.stderr.txt").write_bytes(completed.stderr)
    if completed.returncode != 0 or not target.is_file():
        raise RegressionError(f"PCD conversion failed for {pcd} with exit code {completed.returncode}")


def _edit_map(
    *,
    editor: Path,
    source: Path,
    case_dir: Path,
    edits: list[dict[str, Any]],
    timeout_s: float,
) -> Path:
    current = source
    for index, edit in enumerate(edits):
        target = case_dir / f"map-edit-{index + 1}.ot"
        command = [
            str(editor),
            "--map",
            str(current),
            "--output",
            str(target),
            "--state",
            edit["state"],
            "--shape",
            edit.get("shape", "box"),
            "--x",
            str(edit["x"]),
            "--y",
            str(edit["y"]),
            "--z",
            str(edit["z"]),
            "--radius",
            str(edit["radius_m"]),
        ]
        completed = _run(command, timeout_s=timeout_s)
        (case_dir / f"map-edit-{index + 1}.stdout.txt").write_bytes(completed.stdout)
        (case_dir / f"map-edit-{index + 1}.stderr.txt").write_bytes(completed.stderr)
        if completed.returncode != 0 or not target.is_file():
            raise RegressionError(f"map edit {index + 1} failed with exit code {completed.returncode}")
        current = target
    return current


def _write_xyz(path: Path, points: list[list[float]]) -> None:
    path.write_text("".join(f"{point[0]:.9g} {point[1]:.9g} {point[2]:.9g}\n" for point in points), encoding="utf-8")


def _verify_path(
    *,
    verifier: Path,
    map_path: Path,
    xyz_path: Path,
    planning_anchor: list[float] | None = None,
    options: dict[str, Any],
    output_dir: Path,
    timeout_s: float,
) -> list[str]:
    option_flags = {
        "robot_radius": "--robot-radius",
        "body_clearance_below_m": "--body-clearance-below",
        "body_clearance_above_m": "--body-clearance-above",
        "direct_support_only": "--direct-support-only",
        "support_radius_cells": "--support-radius-cells",
        "support_depth_cells": "--support-depth-cells",
        "support_height_m": "--support-height",
        "support_height_tolerance_m": "--support-height-tolerance",
        "support_patch_radius_cells": "--support-patch-radius-cells",
        "support_patch_min_samples": "--support-patch-min-samples",
        "lowest_traversable_only": "--lowest-traversable-only",
        "max_step_height": "--max-step-height",
        "max_slope": "--max-slope",
    }
    command = [str(verifier), "--map", str(map_path), "--path", str(xyz_path)]
    if planning_anchor is not None:
        command.extend(["--planning-anchor", *(str(float(value)) for value in planning_anchor)])
    for key, flag in option_flags.items():
        if key in options:
            value = options[key]
            if isinstance(value, bool):
                value = str(value).lower()
            command.extend([flag, str(value)])
    completed = _run(
        command,
        timeout_s=timeout_s,
    )
    (output_dir / "verifier.stdout.json").write_bytes(completed.stdout)
    (output_dir / "verifier.stderr.txt").write_bytes(completed.stderr)
    if completed.returncode not in (0, 2):
        return [f"path verifier failed with exit code {completed.returncode}"]
    try:
        result = json.loads(completed.stdout)
    except (UnicodeDecodeError, json.JSONDecodeError):
        return ["path verifier stdout was not valid JSON"]
    if not isinstance(result, dict) or result.get("route_valid") is not True:
        reason = result.get("route_reason") if isinstance(result, dict) else None
        return [f"path verifier did not return route_valid=true (reason={reason!r})"]
    if result.get("sweep_valid") is not True:
        failures = result.get("sweep_failures", [])
        first = failures[0] if failures else None
        return [f"path verifier did not return sweep_valid=true (first_failure={first!r})"]
    if result.get("motion_diagnostic_valid") is not True:
        failures = result.get("motion_failures", [])
        first = failures[0] if failures else None
        return [f"path verifier did not return motion_diagnostic_valid=true (first_failure={first!r})"]
    return []


def _lingtu_source_provenance() -> dict[str, Any]:
    revision = _run(["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"], timeout_s=30.0)
    if revision.returncode != 0:
        raise RegressionError("could not read the LingTu source revision")
    tracked_surfaces = [
        "src/nav/cpp/planning/global/octoplanner",
        "src/maps/include/lingtu/maps/build",
        "src/maps/cpp/build",
        "tools/bench/official_global_planner.py",
        "config/acceptance/official_global_planner.json",
    ]
    status = _run(
        ["git", "-C", str(REPO_ROOT), "status", "--porcelain", "--", *tracked_surfaces],
        timeout_s=30.0,
    )
    if status.returncode != 0:
        raise RegressionError("could not read the LingTu source status")
    status_lines = status.stdout.decode("utf-8", errors="replace").splitlines()
    return {
        "git_revision": revision.stdout.decode("ascii", errors="replace").strip(),
        "dirty": bool(status_lines),
        "dirty_paths": status_lines,
    }


def run_manifest(args: argparse.Namespace) -> dict[str, Any]:
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    build_dir = args.build_dir.resolve()
    output_dir = args.output_dir.resolve()
    data_dir = args.data_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    _json_dump(output_dir / "manifest.json", manifest)

    converter = _require_binary(build_dir, "octoplanner3d_pcd_to_octomap")
    planner = _require_binary(build_dir, "octoplanner3d_headless")
    editor = _require_binary(build_dir, "octoplanner3d_edit_octomap")
    verifier = _require_binary(build_dir, "octoplanner3d_verify_path")
    dataset = manifest["dataset"]
    profiles = manifest["map_profiles"]
    options = manifest["planner_options"]
    cached_maps: dict[tuple[str, str], Path] = {}
    summaries: list[dict[str, Any]] = []

    _materialize_dataset(
        dataset=dataset,
        relative_path=dataset["license_path"],
        data_dir=data_dir,
        upstream_dir=args.upstream_dir.resolve() if args.upstream_dir else None,
        download=args.download,
        timeout_s=args.timeout,
    )

    for case in manifest["cases"]:
        case_id = case["id"]
        case_dir = output_dir / case_id
        case_dir.mkdir(parents=True, exist_ok=True)
        relative_path = case["pcd"]
        profile_name = case["map_profile"]
        cache_key = (relative_path, profile_name)
        if cache_key not in cached_maps:
            pcd = _materialize_dataset(
                dataset=dataset,
                relative_path=relative_path,
                data_dir=data_dir,
                upstream_dir=args.upstream_dir.resolve() if args.upstream_dir else None,
                download=args.download,
                timeout_s=args.timeout,
            )
            map_name = f"{Path(relative_path).stem}-{profile_name}.ot"
            map_path = output_dir / "maps" / map_name
            _convert_map(
                converter=converter,
                pcd=pcd,
                target=map_path,
                profile=profiles[profile_name],
                timeout_s=args.timeout,
            )
            cached_maps[cache_key] = map_path
        map_path = _edit_map(
            editor=editor,
            source=cached_maps[cache_key],
            case_dir=case_dir,
            edits=case.get("map_edits", []),
            timeout_s=args.timeout,
        )

        request = {"map_path": str(map_path), "start": case["start"], "goal": case["goal"], "options": options}
        _json_dump(case_dir / "request.json", request)
        completed = _run(
            [str(planner)],
            stdin=json.dumps(request).encode("utf-8"),
            timeout_s=args.timeout,
        )
        (case_dir / "planner.stdout.json").write_bytes(completed.stdout)
        (case_dir / "planner.stderr.txt").write_bytes(completed.stderr)
        try:
            result = json.loads(completed.stdout)
        except (UnicodeDecodeError, json.JSONDecodeError):
            result = None
        _json_dump(case_dir / "result.json", result)
        xyz_path = case_dir / "path.xyz"
        xyz_path.write_text("", encoding="utf-8")
        if isinstance(result, dict):
            path = result.get("path")
            if isinstance(path, list) and all(
                isinstance(point, list)
                and len(point) == 3
                and all(isinstance(value, (int, float)) and math.isfinite(value) for value in point)
                for point in path
            ):
                _write_xyz(xyz_path, path)
        errors = evaluate_planner_result(
            returncode=completed.returncode,
            result=result,
            expected=case["expected"],
            start=case["start"],
            goal=case["goal"],
            start_snap_tolerance_m=manifest["assertions"]["start_snap_tolerance_m"],
            goal_tolerance_m=manifest["assertions"]["goal_tolerance_m"],
        )
        if not errors and case["expected"]["ok"]:
            errors.extend(
                _verify_path(
                    verifier=verifier,
                    map_path=map_path,
                    xyz_path=xyz_path,
                    planning_anchor=[float(value) for value in case["start"]],
                    options=options,
                    output_dir=case_dir,
                    timeout_s=args.timeout,
                )
            )
        summary = {
            "id": case_id,
            "ok": not errors,
            "expected": case["expected"],
            "planner_returncode": completed.returncode,
            "errors": errors,
            "artifacts": str(case_dir),
        }
        summaries.append(summary)

    summary = {
        "ok": all(case["ok"] for case in summaries),
        "validation_scope": "offline_global_route_geometry",
        "execution_validated": False,
        "manifest": str(args.manifest.resolve()),
        "dataset": dataset,
        "lingtu_source": _lingtu_source_provenance(),
        "planner_options": options,
        "cases": summaries,
    }
    _json_dump(output_dir / "summary.json", summary)
    return summary


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--build-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--data-dir", type=Path, default=REPO_ROOT / "build" / "official-global-planner-data")
    parser.add_argument("--upstream-dir", type=Path)
    parser.add_argument("--download", action="store_true", help="download pinned PCD files from the manifest repository")
    parser.add_argument("--timeout", type=float, default=300.0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        summary = run_manifest(args)
    except (RegressionError, KeyError, OSError, subprocess.TimeoutExpired, json.JSONDecodeError) as exc:
        print(f"official global planner regression failed: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

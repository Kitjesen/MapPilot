#!/usr/bin/env python3
"""Preview geometry-backed semantic approach poses with the native planner."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import tempfile
from pathlib import Path
from typing import Any


def _last_json(stdout: bytes) -> dict[str, Any] | None:
    for line in reversed(stdout.decode("utf-8", errors="replace").splitlines()):
        line = line.strip()
        if not line:
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            return value
    return None


def _admitted(returncode: int, result: dict[str, Any] | None) -> tuple[bool, str]:
    if returncode != 0:
        return False, f"planner_exit_{returncode}"
    if result is None:
        return False, "planner_output_not_json"
    if result.get("ok") is not True:
        return False, str(result.get("error", "planner_rejected"))
    if result.get("reached_goal") is not True:
        return False, "goal_not_reached"
    if not isinstance(result.get("path"), list) or not result["path"]:
        return False, "empty_path"
    return True, "accepted"


def _summarize_results(results: list[dict[str, Any]]) -> tuple[str, int, dict[str, int]]:
    admitted_count = sum(bool(item.get("admitted")) for item in results)
    planner_admitted_count = sum(bool(item.get("planner_admitted")) for item in results)
    if not results:
        status = "no_candidates"
    elif admitted_count == 0:
        # Distinguish a disconnected target from a path that the discrete
        # planner found but the independent continuous gate rejected.
        status = "verifier_blocked" if planner_admitted_count else "target_unreachable"
    else:
        status = "candidate_admitted"
    reason_counts: dict[str, int] = {}
    for item in results:
        reason = str(item.get("reason", "unknown"))
        reason_counts[reason] = reason_counts.get(reason, 0) + 1
    return status, admitted_count, reason_counts


def _verify_path(
    verifier: Path,
    map_path: Path,
    path: list[list[float]],
    start: list[float],
    options: dict[str, Any],
    work_dir: Path,
) -> dict[str, Any]:
    """Run the independent verifier with the planner's request anchor."""
    path_file = tempfile.NamedTemporaryFile(
        mode="w", suffix=".xyz", prefix="semantic-preview-", dir=work_dir,
        delete=False, encoding="utf-8"
    )
    try:
        with path_file:
            path_file.write("\n".join(" ".join(str(float(value)) for value in point) for point in path))
            path_file.write("\n")
        command = [
            str(verifier),
            "--map", str(map_path),
            "--path", path_file.name,
            "--planning-anchor", *(str(float(value)) for value in start),
        ]
        option_args = {
            "robot_radius": "--robot-radius",
            "body_clearance_below_m": "--body-clearance-below",
            "body_clearance_above_m": "--body-clearance-above",
            "direct_support_only": "--direct-support-only",
            "support_depth_cells": "--support-depth-cells",
            "support_height_m": "--support-height",
            "support_height_tolerance_m": "--support-height-tolerance",
            "support_patch_radius_cells": "--support-patch-radius-cells",
            "support_patch_min_samples": "--support-patch-min-samples",
            "max_step_height": "--max-step-height",
            "max_slope": "--max-slope",
        }
        for key, option in option_args.items():
            if key in options:
                value = options[key]
                if isinstance(value, bool):
                    value = "true" if value else "false"
                command.extend((option, str(value)))
        completed = subprocess.run(command, capture_output=True, check=False, timeout=60.0)
        native = _last_json(completed.stdout)
        return {
            "returncode": completed.returncode,
            "result": native,
            "valid": completed.returncode == 0 and bool(native and native.get("valid") is True),
        }
    finally:
        Path(path_file.name).unlink(missing_ok=True)


def preview(
    query_path: Path,
    map_path: Path,
    planner: Path,
    start: list[float],
    options: dict[str, Any],
    verifier: Path | None = None,
    timeout_s: float = 60.0,
) -> dict[str, Any]:
    query = json.loads(query_path.read_text(encoding="utf-8"))
    approach = query.get("approach_goal_candidates", {})
    candidates = approach.get("candidates", [])
    if approach.get("motion_authorization") is not False:
        raise ValueError("semantic query must keep motion_authorization=false")
    results: list[dict[str, Any]] = []
    for index, candidate in enumerate(candidates):
        goal = [float(candidate[axis]) for axis in ("x", "y", "z")]
        if not all(math.isfinite(value) for value in (*start, *goal)):
            raise ValueError("preview pose contains a non-finite coordinate")
        request = {"map_path": str(map_path), "start": start, "goal": goal, "options": options}
        completed = subprocess.run(
            [str(planner)],
            input=(json.dumps(request) + "\n").encode("utf-8"),
            capture_output=True,
            check=False,
            timeout=timeout_s,
        )
        native = _last_json(completed.stdout)
        admitted, reason = _admitted(completed.returncode, native)
        verification = None
        planner_admitted = admitted
        if admitted and verifier is not None and native is not None:
            verification = _verify_path(
                verifier, map_path, native["path"], start, options, query_path.parent
            )
            admitted = bool(verification["valid"])
            if not admitted:
                verification_result = verification.get("result") or {}
                reason = "verification_" + str(
                    verification_result.get("reason", "rejected")
                )
        results.append(
            {
                "candidate_index": index,
                "goal": goal,
                "yaw": float(candidate.get("yaw", 0.0)),
                "admitted": admitted,
                "planner_admitted": planner_admitted,
                "reason": reason,
                "native_result": native,
                "verification": verification,
            }
        )
    status, admitted_count, reason_counts = _summarize_results(results)
    return {
        "schema_version": "lingtu.semantic_goal_preview.v1",
        "query": query_path.as_posix(),
        "map": map_path.as_posix(),
        "planner": planner.as_posix(),
        "candidate_count": len(results),
        "admitted_count": admitted_count,
        "planner_admitted_count": sum(bool(item.get("planner_admitted")) for item in results),
        "verification_passed_count": sum(
            bool((item.get("verification") or {}).get("valid")) for item in results
        ),
        "status": status,
        "failure_reasons": reason_counts,
        "results": results,
        "motion_authorization": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("query", type=Path)
    parser.add_argument("--map", dest="map_path", type=Path, required=True)
    parser.add_argument("--planner", type=Path, required=True)
    parser.add_argument(
        "--verifier", type=Path,
        help="independent path verifier; final admission requires verifier.valid",
    )
    parser.add_argument("--start", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument("--options", type=Path, help="JSON file containing native planner options")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    options = json.loads(args.options.read_text(encoding="utf-8")) if args.options else {}
    report = preview(
        args.query, args.map_path, args.planner, list(args.start), options,
        verifier=args.verifier,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in (
        "candidate_count", "admitted_count", "planner_admitted_count",
        "verification_passed_count", "status", "motion_authorization",
    )}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Run saved-map cases through the production headless planner and geometry verifier."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from itertools import pairwise
from pathlib import Path

VERIFY_OPTIONS = {
    "robot_radius": "robot-radius",
    "body_offset_m": "body-offset",
    "body_clearance_below_m": "body-clearance-below",
    "body_clearance_above_m": "body-clearance-above",
    "direct_support_only": "direct-support-only",
    "support_depth_cells": "support-depth-cells",
    "search_resolution_m": "search-resolution",
    "surface_closing_radius_m": "surface-closing-radius",
    "surface_max_hole_area_m2": "surface-max-hole-area",
    "surface_max_slope": "surface-max-slope",
    "support_height_m": "support-height",
    "support_height_tolerance_m": "support-height-tolerance",
    "support_patch_radius_cells": "support-patch-radius-cells",
    "support_patch_min_samples": "support-patch-min-samples",
    "lowest_traversable_only": "lowest-traversable-only",
    "max_step_height": "max-step-height",
    "max_slope": "max-slope",
}


def assess(plan: dict, geometry: dict) -> str:
    if plan.get("returncode") != 0 or not plan.get("ok") or not plan.get("reached_goal"):
        return "planning_failed"
    if geometry.get("returncode") != 0 or not all(
            geometry.get(key) is True for key in ("route_valid", "sweep_valid", "valid")):
        return "geometry_failed"
    if plan.get("benchmark", {}).get("same_result") is not True:
        return "repeat_changed"
    if "rust_fallback" not in plan["diagnostics"]:
        return "missing_diagnostics"
    if plan["diagnostics"]["rust_fallback"] is not False or plan["diagnostics"].get("used_rust_graph") is not True:
        return "fallback_pass"
    return "rust_pass"


def run(command: list[str], payload: dict | None, directory: Path, name: str,
        timeout: float) -> dict:
    (directory / f"{name}.command.json").write_text(
        json.dumps(command, indent=2), encoding="utf-8")
    result = subprocess.run(command, input=json.dumps(payload) if payload else None,
                            capture_output=True, text=True, timeout=timeout)
    (directory / f"{name}.log").write_text(result.stderr, encoding="utf-8")
    (directory / f"{name}.json").write_text(result.stdout, encoding="utf-8")
    value = json.loads(result.stdout)
    value["returncode"] = result.returncode
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", type=Path, required=True)
    parser.add_argument("--planner", type=Path, required=True)
    parser.add_argument("--verifier", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=120)
    args = parser.parse_args()
    manifest = json.loads(args.cases.read_text(encoding="utf-8"))
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "cases.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    rows = []
    for case in manifest["cases"]:
        directory = args.output / case["id"]
        directory.mkdir()
        request = {"map_path": str(Path(case["map_path"]).resolve()),
                   "start": case["start"], "goal": case["goal"],
                   "options": manifest["options"],
                   "benchmark": {"samples": args.samples,
                                 "cache_directory": str((directory / "cache").resolve())}}
        for key in ("start_yaw", "goal_yaw"):
            if key in case:
                request[key] = case[key]
        (directory / "request.json").write_text(json.dumps(request, indent=2), encoding="utf-8")
        try:
            plan = run([str(args.planner.resolve())], request, directory, "plan", args.timeout)
            geometry = {}
            route = plan.get("path", [])
            if plan.get("ok") and len(route) >= 2:
                route_path = directory / "route.xyz"
                route_path.write_text("\n".join(" ".join(map(str, p)) for p in route), encoding="utf-8")
                command = [str(args.verifier.resolve()), "--map", request["map_path"],
                           "--path", str(route_path.resolve()), "--planning-anchor",
                           *map(str, case["start"])]
                if "start_yaw" in case:
                    command.extend(["--planning-anchor-yaw", str(case["start_yaw"])])
                for key, flag in VERIFY_OPTIONS.items():
                    value = plan["diagnostics"]["constraints"][key]
                    command.extend(["--" + flag, str(value).lower()])
                geometry = run(command, None, directory, "geometry", args.timeout)
            status = assess(plan, geometry)
            rows.append({"id": case["id"], "status": status,
                         "route_length_m": sum(math.dist(a, b) for a, b in pairwise(route)),
                         "route_points": len(route), "returncode": plan["returncode"],
                         "diagnostics": plan.get("diagnostics", {}),
                         "benchmark": plan.get("benchmark", {}), "geometry": geometry})
        except (subprocess.TimeoutExpired, OSError, ValueError, KeyError) as error:
            rows.append({"id": case["id"], "status": "runner_failed", "error": str(error)})
    (args.output / "report.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    print(json.dumps(rows, indent=2))
    return 0 if all(row["status"] == "rust_pass" for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())

"""Freeze dataset requests with resolved Go2 nav parameters for native component probes."""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime
from itertools import pairwise
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

def linux_path(path: Path) -> str:
    text = path.resolve().as_posix()
    return f"/mnt/{text[0].lower()}{text[2:]}" if len(text) > 2 and text[1] == ":" else text


def write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def prepare(out: Path) -> None:
    from lingtu.assembly.compile_plan import compile_run_plan
    from lingtu.assembly.config.native_navigation import _OCTOPLANNER_CONFIG
    input_dir = out / "input"
    if (input_dir / "manifest.json").exists():
        raise ValueError(f"Use a new evidence directory; frozen input already exists: {input_dir}")
    input_dir.mkdir(parents=True, exist_ok=True)
    plan = compile_run_plan("nav", "real", robot="unitree/go2", local_planner="scan")
    plan.write(input_dir / "run-plan.json")
    environment = plan.native_process_environment
    write_json(input_dir / "environment.json", environment)
    options = {}
    for name, (key, default) in _OCTOPLANNER_CONFIG.items():
        raw = environment[key]
        value = raw == "1" if isinstance(default, bool) else type(default)(raw)
        options[name.removeprefix("octoplanner3d_")] = value
    options["body_clearance_below_m"] = float(environment["LINGTU_NAV_OCTO_BODY_CLEARANCE_BELOW_M"])
    options["body_clearance_above_m"] = float(environment["LINGTU_NAV_OCTO_BODY_CLEARANCE_ABOVE_M"])
    cases = [
        ("geode", "build/dataset-mapping-20261021/maps/geode",
         "build/dataset-mapping-20261021/planning/geode/local_geometry_control_native/request.json",
         "build/dataset-planning-fix-20261021/planning/geode/local_geometry_control/summary.json"),
        ("garage", "build/garage-validation-20261021/maps/no_loop",
         "build/garage-validation-20261021/planning/no_loop/local_20_25_native/request.json",
         "build/garage-validation-20261021/planning/no_loop/local_20_25_native/result.json"),
    ]
    records = []
    for name, map_dir, request_path, result_path in cases:
        source = ROOT / map_dir / "octomap.ot"
        copied_map = input_dir / f"{name}.ot"
        shutil.copy2(source, copied_map)
        shutil.copy2(ROOT / map_dir / "metadata.json", input_dir / f"{name}.metadata.json")
        original = json.loads((ROOT / request_path).read_text(encoding="utf-8-sig"))
        write_json(input_dir / f"{name}.original-request.json", original)
        saved_result = json.loads((ROOT / result_path).read_text(encoding="utf-8-sig"))
        effective = saved_result.get("result", saved_result)["diagnostics"]["constraints"]
        write_json(input_dir / f"{name}.prior-result.json", saved_result)
        diagnostic = {**original, "map_path": linux_path(copied_map), "options": effective}
        product = {**diagnostic, "options": options}
        write_json(input_dir / f"{name}.diagnostic-request.json", diagnostic)
        write_json(input_dir / f"{name}.product-request.json", product)
        diagnostic_environment = dict(environment)
        for field, (key, _) in _OCTOPLANNER_CONFIG.items():
            value = effective[field.removeprefix("octoplanner3d_")]
            diagnostic_environment[key] = str(int(value)) if isinstance(value, bool) else str(value)
        diagnostic_environment["LINGTU_NAV_OCTO_BODY_CLEARANCE_BELOW_M"] = str(effective["body_clearance_below_m"])
        diagnostic_environment["LINGTU_NAV_OCTO_BODY_CLEARANCE_ABOVE_M"] = str(effective["body_clearance_above_m"])
        write_json(input_dir / f"{name}.diagnostic-environment.json", diagnostic_environment)
        differences = {key: {"diagnostic": effective.get(key), "product": value}
                       for key, value in options.items() if effective.get(key) != value}
        write_json(input_dir / f"{name}.config-difference.json", differences)
        records.append({"name": name, "map_source": str(source), "map_copy": str(copied_map),
                        "map_bytes": copied_map.stat().st_size, "request_source": request_path,
                        "effective_diagnostic_options_source": result_path,
                        "start": original["start"], "goal": original["goal"],
                        "configuration_difference": differences})
    write_json(input_dir / "manifest.json", {
        "created_at_local": datetime.now().isoformat(), "timezone": "Asia/Shanghai",
        "scope": "native component probe; real Go2 parameter resolution only",
        "product_started": False, "hardware_access": False, "maps_rebuilt": False,
        "physical_simulation": False, "sensor_feedback": False,
        "feedback_model": "optional ideal body vx/vy/wz integration; constant initial Z",
        "truth_from_reference_z": False, "cases": records,
    })
    print(json.dumps({"input": str(input_dir), "cases": [x["name"] for x in records]}, ensure_ascii=False))


def run_case(out: Path, case: str, variant: str, seconds: float, label: str) -> None:
    inputs = out / "input"
    request = json.loads((inputs / f"{case}.{variant}-request.json").read_text())
    env_path = inputs / ("environment.json" if variant == "product" else f"{case}.diagnostic-environment.json")
    environment = json.loads(env_path.read_text())
    destination = out / "runs" / (label or f"{case}-{variant}")
    destination.mkdir(parents=True, exist_ok=False)
    command = [str(out / "bin/benchmark_saved_route"), request["map_path"],
               *map(str, request["start"]), *map(str, request["goal"]), str(destination), str(seconds)]
    started = time.time()
    record = {"command": command, "started": started, "case": case, "variant": variant,
              "input_environment": str(env_path), "physical_simulation": False}
    write_json(destination / "command.json", record)
    with (destination / "stdout.log").open("w") as stdout, (destination / "stderr.log").open("w") as stderr:
        try:
            result = subprocess.run(command, env={**os.environ, **environment}, stdout=stdout, stderr=stderr, timeout=300)
            record["returncode"] = result.returncode
        except subprocess.TimeoutExpired:
            record["returncode"] = None
            record["timeout_s"] = 300
    record["elapsed_s"] = time.time() - started
    write_json(destination / "command.json", record)
    print(json.dumps(record, ensure_ascii=False))


def summarize(out: Path) -> None:
    """Report measured route spacing and active XY tracking, never default zeros."""
    runs = {}
    for folder in sorted((out / "runs").iterdir()):
        command = json.loads((folder / "command.json").read_text())
        if "elapsed_s" not in command:
            raise ValueError(f"Run is still pending: {folder.name}")
        result = json.loads((folder / "summary.json").read_text())
        request = json.loads((out / "input" / (
            f"{command['case']}.{command['variant']}-request.json")).read_text())
        metrics = {"summary": result, "command": command}
        route_file = folder / "global-route.csv"
        if route_file.exists():
            with route_file.open(newline="") as stream:
                route = [[float(row[key]) for key in ("x_m", "y_m", "z_m")]
                         for row in csv.DictReader(stream)]
            lengths = [math.dist(a, b) for a, b in pairwise(route)]
            metrics["route"] = {
                "points": len(route), "length_3d_m": sum(lengths),
                "spacing_min_m": min(lengths) if lengths else None,
                "spacing_mean_m": sum(lengths) / len(lengths) if lengths else None,
                "spacing_max_m": max(lengths) if lengths else None,
                "request_to_route_start_m": math.dist(request["start"], route[0]),
            }
        feedback_file = folder / "feedback.csv"
        if feedback_file.exists():
            with feedback_file.open(newline="") as stream:
                frames = list(csv.DictReader(stream))
            errors = [float(row["position_error_m"]) for row in frames
                      if row["tracking_active"] == "1"]
            metrics["feedback"] = {
                "samples": len(frames), "tracking_active_samples": len(errors),
                "tracking_error_definition": "XY distance to the time-evaluated local spline; ideal velocity integration, not physical accuracy",
                "active_xy_rmse_m": math.sqrt(sum(v*v for v in errors) / len(errors)) if errors else None,
                "active_xy_max_m": max(errors) if errors else None,
                "feedback_elapsed_s": float(frames[-1]["time_s"]) - float(frames[0]["time_s"]) if frames else None,
            }
        if "final_position" in result:
            final, goal = result["final_position"], request["goal"]
            metrics["requested_goal_error"] = {
                "xy_m": math.dist(final[:2], goal[:2]),
                "z_signed_m": final[2] - goal[2],
                "xyz_m": math.dist(final, goal),
            }
        runs[folder.name] = metrics
    write_json(out / "results.json", {
        "scope": "frozen saved-map native component experiment, not Product/physics/field acceptance",
        "z_feedback": "constant initial Z; never copied from reference route",
        "performance": "single-run wall times on development WSL; not realtime qualification",
        "runs": runs,
    })
    print(json.dumps({"output": str(out / "results.json"), "runs": list(runs)}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("prepare", "run", "summarize"))
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--case", choices=("geode", "garage"), default="geode")
    parser.add_argument("--variant", choices=("product", "diagnostic"), default="product")
    parser.add_argument("--seconds", type=float, default=20)
    parser.add_argument("--label", default="")
    args = parser.parse_args()
    if args.action == "prepare":
        prepare(args.out.resolve())
    elif args.action == "summarize":
        summarize(args.out.resolve())
    else:
        run_case(args.out.resolve(), args.case, args.variant, args.seconds, args.label)


if __name__ == "__main__":
    main()

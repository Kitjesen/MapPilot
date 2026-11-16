#!/usr/bin/env python3
"""Build a pinned official DimOS MLS crate and run matched occupied-map queries."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import urllib.request
from pathlib import Path

from mls_regression import VERIFY_OPTIONS


def run(command: list[str], cwd: Path, log: Path | None = None,
        accepted_codes: tuple[int, ...] = (0,)) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(command, cwd=cwd, text=True, encoding="utf-8", errors="replace",
                            capture_output=True, check=False)
    if log is not None:
        log.write_text(result.stdout + result.stderr, encoding="utf-8")
    if result.returncode not in accepted_codes:
        raise RuntimeError(f"command failed ({result.returncode}): {' '.join(command)}\n{result.stderr}")
    return result


def verify_routes(report: dict, output: Path, verifier: Path, options: dict, root: Path) -> None:
    for case in report["cases"]:
        route_file = output / f"{case['id']}.route.xyz"
        body_file = output / f"{case['id']}.body.route.xyz"
        points = [list(map(float, line.split())) for line in route_file.read_text().splitlines() if line.strip()]
        for point in points:
            point[2] += options["support_height_m"]
        body_file.write_text("".join(" ".join(map(str, point)) + "\n" for point in points), encoding="utf-8")
        command = [str(verifier.resolve()), "--map", str((root / case["map_path"]).resolve()),
                   "--path", str(body_file), "--planning-anchor", *map(str, case["start"])]
        for key, flag in VERIFY_OPTIONS.items():
            command.extend(["--" + flag, str(options[key]).lower()])
        checked = run(command, root, output / f"{case['id']}.geometry.log", (0, 2))
        case["geometry_command"] = command
        case["geometry"] = json.loads(checked.stdout.strip().splitlines()[-1])
        case["geometry"]["returncode"] = checked.returncode
        print(case["id"], "geometry", json.dumps(case["geometry"]), flush=True)
        (output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")


def fetch_source(commit: str, directory: Path) -> None:
    prefix = "dimos/navigation/global_planner/mls_planner/rust/"
    url = f"https://api.github.com/repos/dimensionalOS/dimos/git/trees/{commit}?recursive=1"
    with urllib.request.urlopen(url) as response:
        tree = json.load(response)
    for item in tree["tree"]:
        if item["type"] != "blob" or not item["path"].startswith(prefix):
            continue
        relative = item["path"][len(prefix):]
        output = directory / relative
        output.parent.mkdir(parents=True, exist_ok=True)
        source = f"https://raw.githubusercontent.com/dimensionalOS/dimos/{commit}/{item['path']}"
        with urllib.request.urlopen(source) as response:
            output.write_bytes(response.read())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", type=Path, required=True)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dump", type=Path, required=True)
    parser.add_argument("--commit", default="83d807bb91bf6436a1a0e9c2a3260d96478636aa")
    parser.add_argument("--fetch", action="store_true")
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--verify", type=Path)
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    source_root = args.source_root.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    cases = json.loads(args.cases.read_text(encoding="utf-8-sig"))
    options = cases["options"]
    if args.verify_only:
        if args.verify is None:
            parser.error("--verify-only requires --verify")
        report = json.loads((output / "report.json").read_text(encoding="utf-8"))
        verify_routes(report, output, args.verify, options, root)
        return
    if args.fetch:
        fetch_source(args.commit, source_root)
    examples = source_root / "examples"
    examples.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(Path(__file__).with_name("dimos_same_map.rs"), examples / "bench_same_map.rs")
    build = ["cargo", "build", "--locked", "--release", "--manifest-path",
             str(source_root / "Cargo.toml"), "--example", "bench_same_map"]
    if not args.skip_build:
        run(build, root, output / "build.log")
    executable = source_root / "target/release/examples/bench_same_map.exe"
    if not executable.exists():
        executable = executable.with_suffix("")
    config = {
        "robot_height": options["support_height_m"] + options["body_clearance_above_m"],
        "node_spacing_m": options["search_resolution_m"],
        "surface_closing_radius": options["surface_closing_radius_m"],
        "wall_clearance_m": 0.0, "wall_buffer_m": options["search_resolution_m"],
        "wall_buffer_weight": 0.0, "step_threshold_m": options["max_step_height"],
        "step_penalty_weight": 0.0,
        "goal_tolerance": options["terminal_goal_xy_tolerance_m"],
        "goal_z_tolerance": options["terminal_goal_z_tolerance_m"],
        "worker_threads": 1,
    }
    report = {
        "upstream_commit": args.commit,
        "build_command": build,
        "timing_scope": "Official Planner.update_global_map / Planner.plan / Planner.update_region; excludes map read, export, process startup and LingTu GeometryGate",
        "point_export": "resolution-level occupied voxel centers from octoplanner3d_dump_octomap --expand",
        "endpoint_semantics": "start and goal z = body z - support_height_m",
        "platform": sys.platform,
        "dimos_config": config,
        "unsupported_dimos_options": ["max_slope", "known-free evidence", "LingTu body/sweep GeometryGate"],
        "cases": [],
    }
    for case in cases["cases"]:
        occupied = output / f"{case['id']}.occupied.xyz"
        export_command = [str(args.dump.resolve()), "--input", str((root / case["map_path"]).resolve()), "--output", str(occupied), "--expand"]
        if sys.platform == "win32" and args.dump.suffix != ".exe":
            def wsl_path(path: str) -> str:
                resolved = Path(path).resolve()
                return f"/mnt/{resolved.drive[0].lower()}/{resolved.as_posix()[3:]}"
            export_command = ["wsl", "-e", wsl_path(str(args.dump)),
                              "--input", wsl_path(case["map_path"]),
                              "--output", wsl_path(str(occupied)), "--expand"]
        exported = run(export_command, root)
        export = json.loads(exported.stdout.strip().splitlines()[-1])
        start = list(case["start"])
        goal = list(case["goal"])
        start[2] -= options["support_height_m"]
        goal[2] -= options["support_height_m"]
        command = [str(executable), str(occupied), str(export["resolution"]),
                   *map(str, start), *map(str, goal), str(output / case["id"]), str(args.samples),
                   *[str(config[key]) for key in ("robot_height", "node_spacing_m", "surface_closing_radius",
                                                 "step_threshold_m", "goal_tolerance", "goal_z_tolerance", "wall_buffer_m")]]
        measured = run(command, root, output / f"{case['id']}.log")
        metrics = json.loads(measured.stdout.strip().splitlines()[-1])
        report["cases"].append({**case, "ground_start": start, "ground_goal": goal,
                                "export_command": export_command, "export": export,
                                "command": command, "metrics": metrics})
        (output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(case["id"], json.dumps(metrics), flush=True)
    if args.verify is not None:
        verify_routes(report, output, args.verify, options, root)


if __name__ == "__main__":
    main()

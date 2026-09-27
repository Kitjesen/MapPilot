#!/usr/bin/env python3
"""Run a fixed three-way saved-map evidence comparison outside Maps runtime."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


GROUPS = (
    ("old_clean_old_replay", "old_prune", "old_replay"),
    ("new_clean_old_replay", "new_prune", "old_replay"),
    ("new_clean_new_replay", "new_prune", "new_replay"),
)
REQUIRED_SOURCE = ("poses.txt", "scan_origin.txt", "patch_bundle.manifest", "patches")


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def command(path: Path, *arguments: object) -> list[str]:
    prefix = [sys.executable, str(path)] if path.suffix.lower() == ".py" else [str(path)]
    return [*prefix, *(str(argument) for argument in arguments)]


def run(
    argv: list[str], timeout_s: float, *, stdin_json: dict[str, Any] | None = None
) -> dict[str, Any]:
    started = time.perf_counter()
    try:
        completed = subprocess.run(
            argv,
            input=None if stdin_json is None else json.dumps(stdin_json, separators=(",", ":")),
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )
    except subprocess.TimeoutExpired as error:
        return {
            "argv": argv,
            "timed_out": True,
            "returncode": None,
            "elapsed_ms": (time.perf_counter() - started) * 1000.0,
            "stderr": str(error),
        }
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    parsed: Any = None
    if completed.stdout.strip():
        try:
            parsed = json.loads(completed.stdout)
        except json.JSONDecodeError:
            parsed = None
    result = {
        "argv": argv,
        "returncode": completed.returncode,
        "elapsed_ms": elapsed_ms,
        "stdout_json": parsed,
    }
    if completed.stderr.strip():
        result["stderr"] = completed.stderr.strip()
    if parsed is None and completed.stdout.strip():
        result["stdout"] = completed.stdout.strip()
    return result


def source_gaps(source: Path) -> list[str]:
    gaps = [name for name in REQUIRED_SOURCE if not (source / name).exists()]
    patches = source / "patches"
    if patches.is_dir() and not any(patches.glob("*.pcd")):
        gaps.append("patches/*.pcd")
    return gaps


def evaluation_gaps(config: dict[str, Any]) -> list[str]:
    gaps: list[str] = []
    if not isinstance(config.get("planner_options"), dict):
        gaps.append("planner_options")
    roles = {item.get("role") for item in config.get("rois", []) if isinstance(item, dict)}
    for role in ("body_clearance", "ground_support"):
        if role not in roles:
            gaps.append(f"roi:{role}")
    kinds = {item.get("kind") for item in config.get("labels", []) if isinstance(item, dict)}
    for kind in ("wall", "post", "residue"):
        if kind not in kinds:
            gaps.append(f"label:{kind}")
    plans = config.get("plans", [])
    if not isinstance(plans, list) or not plans:
        gaps.append("fixed_plans")
    else:
        for index, plan in enumerate(plans):
            required = ("name", "start", "goal", "expected_ok")
            if not isinstance(plan, dict) or not all(key in plan for key in required):
                gaps.append(f"plan:{index}")
    return gaps


def stage_source(source: Path, destination: Path, input_pcd: Path) -> None:
    shutil.copytree(source, destination)
    staged_input = destination / input_pcd.relative_to(source)
    if staged_input != destination / "map.pcd":
        shutil.copy2(staged_input, destination / "map.pcd")


def group_result(
    name: str,
    candidate: Path,
    prune: Path,
    replay: Path,
    tool: Path,
    planner: Path,
    config_path: Path,
    config: dict[str, Any],
    resolution: float,
    timeout_s: float,
) -> dict[str, Any]:
    prune_result = run(
        command(prune, "--map-dir", candidate, "--apply", "--overwrite"), timeout_s
    )
    output_map = candidate / "comparison.ot"
    replay_result = (
        run(command(replay, "replay", candidate, output_map, resolution), timeout_s)
        if prune_result["returncode"] == 0
        else {"skipped": "prune_failed", "returncode": None}
    )
    inspect_result = (
        run(command(tool, "inspect", output_map, config_path), timeout_s)
        if replay_result["returncode"] == 0
        else {"skipped": "replay_failed"}
    )
    plan_results = []
    if replay_result["returncode"] == 0:
        for plan in config.get("plans", []):
            request = {
                "map_path": str(output_map),
                "start": plan["start"],
                "goal": plan["goal"],
                "options": config.get("planner_options", {}),
            }
            plan_results.append({"name": plan["name"], **run(command(planner), timeout_s, stdin_json=request)})
    successful = (
        prune_result["returncode"] == 0
        and replay_result["returncode"] == 0
        and inspect_result.get("returncode") == 0
        and all(item["returncode"] in (0, 2) for item in plan_results)
    )
    return {
        "name": name,
        "candidate_dir": str(candidate),
        "prune": prune_result,
        "replay": replay_result,
        "inspection": inspect_result,
        "plans": plan_results,
        "completed": successful,
    }


def final_expectations_met(config: dict[str, Any], group: dict[str, Any]) -> bool:
    inspection = group["inspection"].get("stdout_json")
    if not isinstance(inspection, dict):
        return False
    labels = inspection.get("labels")
    if not isinstance(labels, list) or not all(item.get("matches") is True for item in labels):
        return False
    expected_plans = {plan["name"]: plan["expected_ok"] for plan in config["plans"]}
    for result in group["plans"]:
        payload = result.get("stdout_json")
        if not isinstance(payload, dict) or payload.get("ok") is not expected_plans[result["name"]]:
            return False
    return len(group["plans"]) == len(expected_plans)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--input-pcd", type=Path)
    parser.add_argument("--work-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--tool", type=Path, required=True)
    parser.add_argument("--old-prune", type=Path, required=True)
    parser.add_argument("--new-prune", type=Path, required=True)
    parser.add_argument("--old-replay", type=Path, required=True)
    parser.add_argument("--new-replay", type=Path, required=True)
    parser.add_argument("--planner", type=Path, required=True)
    parser.add_argument("--code-sha", required=True)
    parser.add_argument("--content-epoch", required=True)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--timeout", type=float, default=300.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = args.source.resolve()
    input_pcd = (args.input_pcd or (source / "map.pcd.preclean" if (source / "map.pcd.preclean").is_file() else source / "map.pcd")).resolve()
    if source not in input_pcd.parents:
        raise ValueError("input PCD must belong to the source snapshot")
    gaps = source_gaps(source)
    if not input_pcd.is_file():
        gaps.append("input_pcd")
    config = load_json(args.config)
    eval_gaps = evaluation_gaps(config)
    report: dict[str, Any] = {
        "schema": "lingtu.maps.offline_evidence_comparison.v1",
        "validation_level": "offline",
        "field_navigation_accepted": False,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "code_sha": args.code_sha,
        "content_epoch": args.content_epoch,
        "source": str(source),
        "input_pcd": str(input_pcd),
        "resolution_m": args.resolution,
        "timeout_s": args.timeout,
        "config": config,
        "source_gaps": gaps,
        "evaluation_gaps": eval_gaps,
        "phase": "running",
        "groups": [],
    }
    args.work_dir.mkdir(parents=True, exist_ok=True)
    report_path = args.work_dir / "comparison.json"
    if gaps:
        report["acceptance_ready"] = False
        report["reason"] = "source_snapshot_incomplete"
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps({"report": str(report_path), "completed": False, "source_gaps": gaps}))
        return 2
    executables = {
        "old_prune": args.old_prune.resolve(),
        "new_prune": args.new_prune.resolve(),
        "old_replay": args.old_replay.resolve(),
        "new_replay": args.new_replay.resolve(),
    }
    for name, prune_key, replay_key in GROUPS:
        candidate = args.work_dir / name
        if candidate.exists():
            raise FileExistsError(f"candidate already exists: {candidate}")
        stage_source(source, candidate, input_pcd)
        report["groups"].append(
            group_result(
                name, candidate, executables[prune_key], executables[replay_key],
                args.tool.resolve(), args.planner.resolve(), args.config.resolve(), config,
                args.resolution, args.timeout,
            )
        )
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    commands_completed = all(group["completed"] for group in report["groups"])
    expectations_met = (
        not eval_gaps and commands_completed and final_expectations_met(config, report["groups"][-1])
    )
    report["comparison_complete"] = commands_completed
    report["final_expectations_met"] = expectations_met
    report["acceptance_ready"] = expectations_met
    report["phase"] = "complete"
    if eval_gaps:
        report["reason"] = "evaluation_fixture_incomplete"
    elif not commands_completed:
        report["reason"] = "comparison_command_failed"
    elif not expectations_met:
        report["reason"] = "final_expectation_failed"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({"report": str(report_path), "completed": report["acceptance_ready"]}))
    return 0 if report["acceptance_ready"] else 2


if __name__ == "__main__":
    raise SystemExit(main())

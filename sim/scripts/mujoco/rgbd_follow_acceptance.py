"""Run a long RGB-D person-fixture follow in the industrial park."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from sim.scripts.mujoco import native_navigation_acceptance as native

from lingtu.assembly.compiler import compile_run_plan


def assess(phase_dir, minimum_travel=25):
    rows = [json.loads(line) for line in (phase_dir / "motion.jsonl").read_text().splitlines()]
    rows = [r for r in rows if r["driving"] and r.get("mocap_pose")]
    visual = json.loads((phase_dir / "rgbd-follow-report.json").read_text())
    distances = [math.dist([r["x"], r["y"]], r["mocap_pose"]["position_m"][:2]) for r in rows]
    travel = math.dist(rows[0]["mocap_pose"]["position_m"], rows[-1]["mocap_pose"]["position_m"])
    observations = [json.loads(line) for line in (phase_dir / "rgbd-follow.jsonl").read_text().splitlines()]
    errors = []
    times = np.array([r["sim_time_s"] for r in rows])
    for obs in observations:
        if obs["event"] != "observation" or not obs["projected_targets"]:
            continue
        index = int(np.argmin(abs(times - obs["sim_time_s"])))
        if abs(times[index] - obs["sim_time_s"]) < 0.11:
            errors.append(math.dist(obs["projected_targets"][0][:2], rows[index]["mocap_pose"]["position_m"][:2]))
    blockers = []
    if travel < minimum_travel:
        blockers.append("person_travel_incomplete")
    if not distances or min(distances) < 1.0:
        blockers.append("person_clearance_below_1m")
    if distances and max(distances) > 4:
        blockers.append("follow_separation_above_4m")
    if visual["visible_fraction"] < 0.95:
        blockers.append("visual_visibility_below_95pct")
    if not errors or np.percentile(errors, 95) > 0.4:
        blockers.append("rgbd_position_error_above_40cm")
    return {
        "ok": not blockers,
        "blockers": blockers,
        "person_travel_m": travel,
        "distance_min_m": min(distances),
        "distance_median_m": float(np.median(distances)),
        "distance_p95_m": float(np.percentile(distances, 95)),
        "distance_max_m": max(distances),
        "projection_xy_error_p95_m": float(np.percentile(errors, 95)) if errors else None,
        "visual": visual,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--goal-deadband-m", type=float, default=.25)
    args = parser.parse_args()
    out = args.out_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    manifest = native._load_manifest(native.ROOT / "config/acceptance/mujoco/rgbd_follow.json")
    manifest.pop("asset_builder", None)
    manifest["rgbd_follow_goal_deadband_m"] = args.goal_deadband_m
    manifest["map_dir"] = str(Path.home() / "data/lingtu/maps/industrial_park_tracking")
    if args.smoke:
        manifest["dynamic_obstacle"].update(end_xyz=[41, 9.5, 0], duration_s=12)
        manifest["phases"]["motion"]["duration_s"] = 36
        manifest["goal"] = [41, 7.5, 0.55, math.pi / 2]
        manifest["thresholds"].update(min_motion_m=2, min_net_displacement_m=2, min_goal_distance_reduction_m=2)
    path = out / "navigation.json"
    native._write_json(path, manifest)
    plan = compile_run_plan(
        "nav", "sim", robot="doso/thunder_v4", local_planner="scan", env_config={"backend": "mujoco"}
    )
    report = native.run(
        argparse.Namespace(
            manifest=str(path),
            out_dir=str(out / "native"),
            mode="motion",
            build_helper=False,
            prepare_assets=False,
            preflight_only=False,
            domain_id=226,
            validated_run_plan=plan,
            run_plan_verified=False,
            record_video=True,
            video_width=960,
            video_height=540,
            video_fps=4,
            video_lidar_points=400,
        )
    )
    phase = report.get("phases", {}).get("motion", {})
    try:
        following = assess(out / "native/motion", minimum_travel=2.9 if args.smoke else 26.4)
    except (OSError, ValueError, IndexError, KeyError) as exc:
        following = {"ok": False, "blockers": [f"follow_evidence_missing:{exc}"]}
    result = {
        "ok": report["ok"] and following["ok"],
        "following": following,
        "native_blockers": report.get("blockers", []),
        "goal_metrics": phase.get("goal_metrics"),
        "terminal_stop": phase.get("terminal_driver_stop"),
        "contacts": (phase.get("sensor_report") or {}).get("entity_contacts"),
        "scope": manifest["acceptance_scope"],
    }
    native._write_json(out / "summary.json", result)
    print(json.dumps(result, indent=2))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

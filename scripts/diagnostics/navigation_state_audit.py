#!/usr/bin/env python3
"""Audit the local Navigation mission FSM without ROS or MuJoCo."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from nav.navigation import Navigation
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import Odometry


def _parse_xyz(value: str) -> tuple[float, float, float]:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) not in (2, 3):
        raise argparse.ArgumentTypeError("expected X,Y or X,Y,Z")
    try:
        x = float(parts[0])
        y = float(parts[1])
        z = float(parts[2]) if len(parts) == 3 else 0.0
    except ValueError as exc:
        raise argparse.ArgumentTypeError("goal/start must be numeric") from exc
    return x, y, z


def _odom(xyz: tuple[float, float, float]) -> Odometry:
    return Odometry(
        pose=Pose(
            position=Vector3(*xyz),
            orientation=Quaternion(),
        ),
        frame_id="map",
        ts=time.time(),
    )


def _goal(xyz: tuple[float, float, float]) -> PoseStamped:
    return PoseStamped(
        pose=Pose(
            position=Vector3(*xyz),
            orientation=Quaternion(),
        ),
        frame_id="map",
        ts=time.time(),
    )


def run_audit(args: argparse.Namespace) -> dict[str, Any]:
    nav = Navigation(
        planner=args.planner,
        waypoint_threshold=args.waypoint_threshold,
        final_waypoint_threshold=args.final_waypoint_threshold,
        complete_path_on_goal_proximity=True,
        goal_proximity_completion_threshold=args.final_waypoint_threshold,
    )
    states: list[str] = []
    mission_statuses: list[dict[str, Any]] = []
    paths: list[list[Any]] = []
    waypoints: list[PoseStamped] = []
    adapter_events: list[dict[str, Any]] = []

    nav.planner_status._add_callback(states.append)
    nav.mission_status._add_callback(mission_statuses.append)
    nav.global_path._add_callback(paths.append)
    nav.waypoint._add_callback(waypoints.append)
    nav.adapter_status._add_callback(adapter_events.append)

    nav.setup()
    nav._on_odom(_odom(args.start))
    nav._on_goal(_goal(args.goal))
    nav._on_odom(_odom(args.goal))
    final_state = nav._get_state().value

    status_has_reason = all(bool(item.get("phase_reason")) for item in mission_statuses if item.get("state"))
    checks = {
        "entered_planning": "PLANNING" in states,
        "entered_executing": "EXECUTING" in states,
        "published_global_path": bool(paths),
        "published_waypoint": bool(waypoints),
        "completed_after_goal_odom": final_state == "SUCCESS",
        "mission_status_has_phase_reason": status_has_reason,
        "no_illegal_transition": not any(
            event.get("event") == "mission_state_transition_rejected" for event in adapter_events
        ),
    }
    return {
        "ok": all(checks.values()),
        "checks": checks,
        "states": states,
        "final_state": final_state,
        "path_count": len(paths),
        "last_path_len": len(paths[-1]) if paths else 0,
        "waypoint_count": len(waypoints),
        "last_mission_status": mission_statuses[-1] if mission_statuses else {},
        "adapter_events": adapter_events,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run a local navigation FSM audit: goal -> path -> waypoint -> success.",
    )
    parser.add_argument("--planner", default="direct")
    parser.add_argument("--start", type=_parse_xyz, default=(0.0, 0.0, 0.0))
    parser.add_argument("--goal", type=_parse_xyz, default=(1.0, 0.0, 0.0))
    parser.add_argument("--waypoint-threshold", type=float, default=0.2)
    parser.add_argument("--final-waypoint-threshold", type=float, default=0.2)
    parser.add_argument("--json-out", default="")
    args = parser.parse_args()

    report = run_audit(args)
    text = json.dumps(report, indent=2, sort_keys=True)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

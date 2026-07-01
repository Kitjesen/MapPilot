#!/usr/bin/env python3
"""Compatibility wrapper for the legacy ROS simulation launch.

This wrapper keeps `sim/launch/sim.launch.py` usable without reviving the old
deleted script path. It starts the vendored PCT ROS planner only when the run
has an explicit nonzero ROS_DOMAIN_ID, so accidental default-domain launches
fail before any simulation planner node is created.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
LEGACY_PLANNER = (
    REPO_ROOT
    / "src"
    / "nav"
    / "planning"
    / "vendor"
    / "pct_planner"
    / "planner"
    / "scripts"
    / "legacy"
    / "global_planner.py"
)


def _isolated_domain(env: dict[str, str]) -> str:
    domain = (env.get("ROS_DOMAIN_ID") or "").strip()
    if not domain or domain == "0":
        raise SystemExit(
            "Refusing to start legacy global planner without ROS_DOMAIN_ID set "
            "to a nonzero isolated simulation domain."
        )
    return domain


def _planner_args(env: dict[str, str]) -> list[str]:
    if not LEGACY_PLANNER.exists():
        raise SystemExit(f"legacy global planner missing: {LEGACY_PLANNER}")

    args = [
        sys.executable,
        str(LEGACY_PLANNER),
        "--ros-args",
        "-r",
        "/goal_pose:=/nav/goal_pose",
        "-r",
        "/pct_path:=/nav/global_path",
        "-r",
        "/pct_planner/status:=/nav/planner_status",
        "-p",
        "map_frame:=map",
        "-p",
        "robot_frame:=body",
        "-p",
        "flatten_path_z:=true",
    ]

    map_file = (env.get("LINGTU_SIM_GLOBAL_PLANNER_MAP") or "").strip()
    if map_file:
        args.extend(["-p", f"map_file:={map_file}"])

    ground_h = (env.get("LINGTU_SIM_TOMOGRAM_GROUND_H") or "").strip()
    if ground_h:
        args.extend(["-p", f"tomogram_ground_h:={ground_h}"])

    return args


def main() -> int:
    env = dict(os.environ)
    _isolated_domain(env)
    args = _planner_args(env)
    os.execvpe(args[0], args, env)
    return 127


if __name__ == "__main__":
    raise SystemExit(main())

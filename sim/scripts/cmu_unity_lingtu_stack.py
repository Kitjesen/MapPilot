#!/usr/bin/env python3
"""Run LingTu against an external CMU Unity/TARE ROS graph.

This is a simulation-only entry point. It does not start hardware drivers,
LingTu-managed SLAM services, or LingTu's in-tree TARE binary. It expects the
CMU Unity workspace and the LingTu CMU adapter to publish the canonical
`/nav/*` and `/exploration/*` topics in the same ROS_DOMAIN_ID.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TOMOGRAM = (
    ROOT
    / "src"
    / "nav"
    / "planning"
    / "vendor"
    / "pct_planner"
    / "rsc"
    / "tomogram"
    / "building2_9.pickle"
)


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planner", choices=("pct", "astar"), default="pct")
    parser.add_argument(
        "--tomogram",
        default=os.environ.get("LINGTU_CMU_TOMOGRAM"),
        help=(
            "Same-source CMU Unity tomogram for PCT. Required for planner=pct; "
            "the legacy building2_9 map is accepted only when "
            "LINGTU_CMU_ALLOW_STATIC_TOMOGRAM=1."
        ),
    )
    parser.add_argument("--gateway-port", type=int, default=5050)
    parser.add_argument("--allow-default-ros-domain", action="store_true")
    parser.add_argument(
        "--enable-frontier",
        action="store_true",
        help="Use LingTu wavefront frontier exploration on the CMU Unity live map.",
    )
    parser.add_argument("--frontier-min-size", type=int, default=3)
    parser.add_argument("--frontier-safe-distance", type=float, default=0.35)
    parser.add_argument("--frontier-lookahead", type=float, default=4.0)
    parser.add_argument("--frontier-max-dist", type=float, default=18.0)
    parser.add_argument("--frontier-rate", type=float, default=1.0)
    parser.add_argument("--frontier-goal-timeout", type=float, default=20.0)
    parser.add_argument("--frontier-info-gain", type=float, default=0.0)
    parser.add_argument(
        "--waypoint-threshold",
        type=float,
        default=_env_float("LINGTU_CMU_WAYPOINT_THRESHOLD", 0.45),
        help="Navigation waypoint reach radius for CMU Unity exploration.",
    )
    parser.add_argument(
        "--final-waypoint-threshold",
        type=float,
        default=_env_float("LINGTU_CMU_FINAL_WAYPOINT_THRESHOLD", 0.35),
        help="Final waypoint reach radius for CMU Unity exploration.",
    )
    parser.add_argument(
        "--stuck-timeout",
        type=float,
        default=_env_float("LINGTU_CMU_STUCK_TIMEOUT", 25.0),
        help="Seconds without progress before LingTu declares STUCK.",
    )
    parser.add_argument(
        "--stuck-dist-thre",
        type=float,
        default=_env_float("LINGTU_CMU_STUCK_DIST_THRE", 0.08),
        help="Minimum progress distance that resets stuck detection.",
    )
    parser.add_argument(
        "--downsample-dist",
        type=float,
        default=_env_float("LINGTU_CMU_DOWNSAMPLE_DIST", 0.6),
        help="Global path waypoint spacing for the LingTu planner output.",
    )
    parser.add_argument(
        "--path-follower-goal-tolerance",
        type=float,
        default=_env_float("LINGTU_CMU_PATH_FOLLOWER_GOAL_TOLERANCE", 0.35),
    )
    parser.add_argument(
        "--safe-goal-tolerance",
        type=float,
        default=_env_float("LINGTU_CMU_SAFE_GOAL_TOLERANCE", 0.4),
        help=(
            "Maximum distance LingTu may adjust a planner goal to a nearby safe cell. "
            "Keep this small for exploration so a far TARE goal cannot be accepted "
            "as a nearby partial-goal success."
        ),
    )
    parser.add_argument(
        "--plan-safety-policy",
        choices=("off", "observe", "reject", "fallback_astar"),
        default=os.environ.get("LINGTU_CMU_PLAN_SAFETY_POLICY", "fallback_astar"),
        help="Plan safety policy used by LingTu's planner service in CMU Unity simulation.",
    )
    parser.add_argument(
        "--no-exploration-auto-start",
        action="store_true",
        help="Do not auto-start external TARE waypoints; useful for explicit goal demos.",
    )
    parser.add_argument(
        "--disable-direct-goal-fallback",
        action="store_true",
        help=(
            "Fail the navigation mission instead of bypassing the planner with a "
            "direct goal when the selected planner returns no path. Use this for "
            "PCT validation gates."
        ),
    )
    parser.add_argument(
        "--disable-external-strategy-path-control",
        action="store_true",
        default=os.environ.get("LINGTU_CMU_EXTERNAL_STRATEGY_PATH_CONTROL", "0") != "1",
        help=(
            "Do not execute external TARE strategy paths directly through LingTu "
            "tracking. Strict PCT gates can disable this to force every target "
            "through the global planner."
        ),
    )
    parser.add_argument(
        "--disable-tare-path-strategy",
        action="store_true",
        default=not _env_bool("LINGTU_CMU_PREFER_TARE_PATH_STRATEGY", True),
        help=(
            "Use only TARE's rolling /way_point. By default CMU Unity uses "
            "TARE's re-anchored /local_path strategy as a multi-goal source; "
            "set LINGTU_CMU_PREFER_TARE_PATH_STRATEGY=0 for raw waypoint mode."
        ),
    )
    parser.add_argument(
        "--tare-path-start-tolerance",
        type=float,
        default=_env_float("LINGTU_CMU_TARE_PATH_START_TOLERANCE", 1.5),
        help="Maximum distance from current odom to the nearest TARE path point.",
    )
    parser.add_argument(
        "--tare-path-goal-min-distance",
        type=float,
        default=_env_float("LINGTU_CMU_TARE_PATH_GOAL_MIN_DISTANCE", 1.0),
        help="Skip TARE path points closer than this to current odom.",
    )
    parser.add_argument(
        "--tare-path-goal-spacing",
        type=float,
        default=_env_float("LINGTU_CMU_TARE_PATH_GOAL_SPACING", 0.75),
        help="Minimum spacing between TARE strategy subgoals sent to LingTu.",
    )
    parser.add_argument(
        "--disable-partial-goal-progress",
        action="store_true",
        default=not _env_bool("LINGTU_CMU_ACCEPT_PARTIAL_GOAL_PROGRESS", True),
        help=(
            "Keep retrying the raw TARE waypoint after a safe partial path. "
            "The default treats each reachable partial endpoint as exploration "
            "progress so TARE can keep rolling instead of deadlocking on one "
            "far waypoint."
        ),
    )
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if domain in {"", "0"} and not args.allow_default_ros_domain:
        print(
            "Refusing to start CMU Unity LingTu stack on default ROS_DOMAIN_ID. "
            "Set an isolated non-zero domain, for example: export ROS_DOMAIN_ID=73",
            file=sys.stderr,
        )
        return 2
    tomogram = args.tomogram or str(DEFAULT_TOMOGRAM)
    allow_static_tomogram = os.environ.get("LINGTU_CMU_ALLOW_STATIC_TOMOGRAM", "") == "1"
    if args.planner == "pct":
        if not args.tomogram:
            print(
                "Refusing CMU Unity PCT without a same-source tomogram. "
                "Run launch_cmu_unity_lingtu_runtime.sh with "
                "LINGTU_CMU_AUTO_TOMOGRAM=1, or pass --tomogram explicitly.",
                file=sys.stderr,
            )
            return 2
        try:
            uses_default_tomogram = Path(args.tomogram).resolve() == DEFAULT_TOMOGRAM.resolve()
        except OSError:
            uses_default_tomogram = str(args.tomogram) == str(DEFAULT_TOMOGRAM)
        if uses_default_tomogram and not allow_static_tomogram:
            print(
                "Refusing CMU Unity PCT with the legacy building2_9 tomogram. "
                "Use a CMU Unity same-source tomogram, or set "
                "LINGTU_CMU_ALLOW_STATIC_TOMOGRAM=1 for a deliberate legacy test.",
                file=sys.stderr,
            )
            return 2

    sys.path.insert(0, str(ROOT / "src"))
    sys.path.insert(0, str(ROOT))

    print(
        "CMU Unity ROS endpoint runtime was removed: "
        f"ROS_DOMAIN_ID={domain}, planner={args.planner}, tomogram={tomogram}. "
        "Use the native simulation stack."
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())

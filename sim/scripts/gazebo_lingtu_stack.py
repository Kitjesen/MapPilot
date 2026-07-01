#!/usr/bin/env python3
"""Run the LingTu Module stack against the external Gazebo ROS/GZ boundary."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile",
        choices=("sim_gazebo", "sim_industrial", "explore"),
        default=os.environ.get("LINGTU_PROFILE", "sim_industrial"),
    )
    parser.add_argument("--gateway-port", type=int, default=5050)
    parser.add_argument("--planner", choices=("astar", "pct"), default="astar")
    parser.add_argument("--frontier-safe-distance", type=float, default=0.80)
    parser.add_argument("--frontier-max-dist", type=float, default=20.0)
    parser.add_argument("--frontier-rate", type=float, default=2.0)
    parser.add_argument("--disable-semantic", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    sys.path.insert(0, str(ROOT / "src"))
    sys.path.insert(0, str(ROOT))

    domain = os.environ.get("ROS_DOMAIN_ID", "")
    print(
        "Gazebo ROS endpoint runtime was removed: "
        f"profile={args.profile}, ROS_DOMAIN_ID={domain}, planner={args.planner}. "
        "Use the native sim_endpoint/Gazebo adapter path."
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())

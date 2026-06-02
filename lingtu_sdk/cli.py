"""LingTu CLI -- quick robot control from terminal."""

from __future__ import annotations

import argparse
import sys

from lingtu_sdk import LingTuClient


def main() -> None:
    """LingTu CLI entry point -- quick robot control from terminal."""
    p = argparse.ArgumentParser(description="LingTu Robot CLI")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5050)
    sub = p.add_subparsers(dest="cmd")

    # lingtu go 10 5
    go = sub.add_parser("go")
    go.add_argument("x", type=float)
    go.add_argument("y", type=float)
    go.add_argument("--yaw", type=float, default=0)

    # lingtu stop
    sub.add_parser("stop")

    # lingtu state
    sub.add_parser("state")

    # lingtu health
    sub.add_parser("health")

    # lingtu maps
    sub.add_parser("maps")

    # lingtu save-map <name>
    save = sub.add_parser("save-map")
    save.add_argument("name")

    # lingtu nav-status
    sub.add_parser("nav-status")

    # lingtu position
    sub.add_parser("position")

    # lingtu session
    sub.add_parser("session")

    args = p.parse_args()
    robot = LingTuClient(args.host, args.port)

    if args.cmd == "go":
        r = robot.go(args.x, args.y, args.yaw)
        print(r.message or "ok")
    elif args.cmd == "stop":
        r = robot.stop()
        print(r.message or "ok")
    elif args.cmd == "state":
        s = robot.state()
        print(f"Mode: {s.mode}")
        print(f"Pos: ({s.odometry.x:.2f}, {s.odometry.y:.2f}, yaw={s.odometry.yaw:.2f})")
        print(f"Mission: {s.mission.state}")
    elif args.cmd == "health":
        h = robot.health()
        print(f"Modules: {h.modules_ok}/{h.modules_total} ok")
        print(f"SLAM rate: {h.slam_rate:.1f} Hz")
        print(f"Mode: {h.mode}")
    elif args.cmd == "maps":
        ml = robot.maps()
        print(f"Active map: {ml.active_map}")
        for m in ml.maps:
            print(f"  {m.name}  active={m.is_active}  pcd={m.has_pcd}")
    elif args.cmd == "save-map":
        r = robot.save_map(args.name)
        print(r.message or "saved")
    elif args.cmd == "nav-status":
        ns = robot.navigation_status()
        print(f"State: {ns.state}")
        print(f"Dist to goal: {ns.distance_to_goal:.2f}m")
        print(f"Goal: ({ns.goal.x:.2f}, {ns.goal.y:.2f})")
    elif args.cmd == "position":
        p = robot.position()
        print(f"x={p.x:.2f}  y={p.y:.2f}  z={p.z:.2f}  yaw={p.yaw:.2f}")
    elif args.cmd == "session":
        s = robot.session()
        print(f"Mode: {s.mode}")
        print(f"Map: {s.active_map}")
        print(f"SLAM: {s.slam_profile}")
    else:
        p.print_help()

    robot.close()


if __name__ == "__main__":
    main()

"""LingTu CLI -- quick robot control from terminal."""

from __future__ import annotations

import argparse
import json

from lingtu.sdk import LingTuClient


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

    inspection_routes = sub.add_parser("inspection-routes")
    inspection_routes.add_argument("--map-id")

    inspection_start = sub.add_parser("inspection-start")
    inspection_start.add_argument("route_id")
    inspection_start.add_argument("--map-id")
    inspection_start.add_argument("--revision", type=int, default=0)
    inspection_start.add_argument("--request-id")

    inspection_status = sub.add_parser("inspection-status")
    inspection_status.add_argument("task_id")

    for action in ("pause", "resume", "cancel"):
        control = sub.add_parser(f"inspection-{action}")
        control.add_argument("task_id")
        control.add_argument("--reason", default=f"operator_{action}")
        control.add_argument("--request-id")

    inspection_report = sub.add_parser("inspection-report")
    inspection_report.add_argument("task_id")

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
        print(f"Navigation task: {s.mission.task.state}")
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
        print(f"Task: {ns.task.state}")
        print(f"Goal admission: {ns.goal_admission.state}")
        print(f"Control: {ns.control.authority}")
        print(f"Motion: {ns.motion.permission} / {ns.motion.observation}")
    elif args.cmd == "position":
        p = robot.position()
        print(f"x={p.x:.2f}  y={p.y:.2f}  z={p.z:.2f}  yaw={p.yaw:.2f}")
    elif args.cmd == "session":
        s = robot.session()
        print(f"Mode: {s.mode}")
        print(f"Map: {s.active_map}")
        print(f"SLAM: {s.slam_profile}")
    elif args.cmd == "inspection-routes":
        print(json.dumps(robot.inspection_routes(args.map_id), ensure_ascii=False, indent=2))
    elif args.cmd == "inspection-start":
        result = robot.start_inspection(
            args.route_id,
            map_id=args.map_id,
            revision=args.revision,
            request_id=args.request_id,
        )
        print(json.dumps(result.raw, ensure_ascii=False, indent=2))
    elif args.cmd == "inspection-status":
        print(json.dumps(robot.inspection_task(args.task_id), ensure_ascii=False, indent=2))
    elif args.cmd in {"inspection-pause", "inspection-resume", "inspection-cancel"}:
        action = args.cmd.removeprefix("inspection-")
        result = getattr(robot, f"{action}_inspection")(
            args.task_id,
            reason=args.reason,
            request_id=args.request_id,
        )
        print(json.dumps(result.raw, ensure_ascii=False, indent=2))
    elif args.cmd == "inspection-report":
        print(json.dumps(robot.inspection_report(args.task_id), ensure_ascii=False, indent=2))
    else:
        p.print_help()

    robot.close()


if __name__ == "__main__":
    main()

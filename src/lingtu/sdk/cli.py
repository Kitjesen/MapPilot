"""LingTu CLI -- quick robot control from terminal."""

from __future__ import annotations

import argparse
import json
import os

from lingtu.sdk import LingTuClient


def main() -> None:
    """LingTu CLI entry point -- quick robot control from terminal."""
    p = argparse.ArgumentParser(description="LingTu Robot CLI")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5050)
    p.add_argument("--base-url", help="HTTP(S) Gateway origin, overrides --host and --port")
    p.add_argument("--api-key", default=os.environ.get("LINGTU_API_KEY"), help="Gateway API key, defaults to LINGTU_API_KEY")
    sub = p.add_subparsers(dest="cmd")

    # lingtu go 10 5
    go = sub.add_parser("go")
    go.add_argument("x", type=float)
    go.add_argument("y", type=float)
    go.add_argument("--yaw", type=float)
    go.add_argument("--z", type=float, default=0.0, help="Map height in metres")
    go.add_argument("--max-speed-mps", type=float)
    go.add_argument("--tolerance-m", type=float)

    # lingtu stop
    sub.add_parser("stop")

    for action in ("pause", "resume", "cancel"):
        control = sub.add_parser(f"nav-{action}")
        control.add_argument("task_id")
        control.add_argument("--reason", default=f"operator_{action}")
        control.add_argument("--request-id")
    speed = sub.add_parser("nav-speed")
    speed.add_argument("task_id")
    speed.add_argument("max_speed_mps", type=float)
    speed.add_argument("--request-id")

    # lingtu state
    sub.add_parser("state")

    # lingtu health
    sub.add_parser("health")

    # lingtu maps
    sub.add_parser("maps")

    # lingtu save-map <name>
    save = sub.add_parser("save-map")
    save.add_argument("name")
    save.add_argument("--request-id")
    save.add_argument("--no-auto-process", action="store_true", help="Archive the source for manual processing")
    save.add_argument("--wait", action="store_true")
    save.add_argument("--wait-for-source", action="store_true")
    save.add_argument("--timeout", type=float, default=300.0)

    operations = sub.add_parser("map-operations")
    operations.add_argument("--limit", type=int, default=100)
    for action in ("status", "process", "retry", "cancel", "reoptimize"):
        operation = sub.add_parser(f"map-{action}")
        operation.add_argument("operation_id")
        if action == "reoptimize":
            operation.add_argument("--request-id")
    for artifact in ("source", "pcd", "octomap"):
        download = sub.add_parser(f"map-download-{artifact}")
        download.add_argument("operation_id" if artifact == "source" else "name")
        download.add_argument("target")

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
    robot = LingTuClient(args.host, args.port, api_key=args.api_key, base_url=args.base_url)

    if args.cmd == "go":
        r = robot.navigation.go(args.x, args.y, args.yaw, z=args.z,
                                max_speed_mps=args.max_speed_mps, acceptance_radius_m=args.tolerance_m)
        print(r.message or "ok")
    elif args.cmd == "stop":
        r = robot.system.emergency_stop()
        print(r.message or "ok")
    elif args.cmd in {"nav-pause", "nav-resume", "nav-cancel"}:
        action = args.cmd.removeprefix("nav-")
        if action == "cancel":
            result = robot.navigation.cancel(args.reason, task_id=args.task_id, request_id=args.request_id)
        else:
            result = getattr(robot.navigation, action)(args.task_id, reason=args.reason, request_id=args.request_id)
        print(json.dumps(result.raw, ensure_ascii=False, indent=2))
    elif args.cmd == "nav-speed":
        result = robot.navigation.set_speed_limit(args.task_id, args.max_speed_mps, request_id=args.request_id)
        print(json.dumps(result.raw, ensure_ascii=False, indent=2))
    elif args.cmd == "state":
        s = robot.system.state()
        print(f"Mode: {s.mode}")
        if s.odometry is None:
            print("Pos: unavailable")
        else:
            print(
                f"Pos: ({s.odometry.x:.2f}, {s.odometry.y:.2f}, "
                f"yaw={s.odometry.yaw:.2f})"
            )
        print(f"Navigation task: {s.mission.task.state}")
    elif args.cmd == "health":
        h = robot.system.health()
        print(f"Modules: {h.modules_ok}/{h.modules_total} ok")
        print(f"SLAM rate: {h.slam_rate:.1f} Hz")
        print(f"Mode: {h.mode}")
    elif args.cmd == "maps":
        ml = robot.map.list()
        print(f"Active map: {ml.active_map}")
        for m in ml.maps:
            print(f"  {m.name}  active={m.is_active}  pcd={m.has_pcd}")
    elif args.cmd == "save-map":
        if args.wait or args.wait_for_source:
            payload = robot.map.save_and_wait(
                args.name, request_id=args.request_id, auto_process=not args.no_auto_process,
                timeout=args.timeout, wait_for_source=args.wait_for_source,
            )
        else:
            payload = robot.map.save(
                args.name, request_id=args.request_id, auto_process=not args.no_auto_process,
            ).raw
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    elif args.cmd == "map-operations":
        print(json.dumps(robot.map.operations(args.limit), ensure_ascii=False, indent=2))
    elif args.cmd == "map-status":
        print(json.dumps(robot.map.operation(args.operation_id), ensure_ascii=False, indent=2))
    elif args.cmd in {"map-process", "map-retry", "map-cancel", "map-reoptimize"}:
        action = args.cmd.removeprefix("map-")
        method = getattr(robot.map, f"{action}_operation")
        result = method(args.operation_id, request_id=args.request_id) if action == "reoptimize" else method(args.operation_id)
        print(json.dumps(result.raw, ensure_ascii=False, indent=2))
    elif args.cmd in {"map-download-source", "map-download-pcd", "map-download-octomap"}:
        artifact = args.cmd.removeprefix("map-download-")
        identity = args.operation_id if artifact == "source" else args.name
        print(getattr(robot.map, f"download_{artifact}")(identity, args.target))
    elif args.cmd == "nav-status":
        ns = robot.navigation.status()
        print(f"Task: {ns.task.state}")
        print(f"Goal admission: {ns.goal_admission.state}")
        print(f"Control: {ns.control.authority}")
        print(f"Motion: {ns.motion.permission} / {ns.motion.observation}")
    elif args.cmd == "position":
        position = robot.navigation.position()
        print(f"x={position.x:.2f}  y={position.y:.2f}  z={position.z:.2f}  yaw={position.yaw:.2f}")
    elif args.cmd == "session":
        session = robot.system.session()
        print(f"Mode: {session.mode}")
        print(f"Map: {session.active_map}")
        print(f"SLAM: {session.slam_profile}")
    elif args.cmd == "inspection-routes":
        print(json.dumps(robot.tasks.inspection_routes(args.map_id), ensure_ascii=False, indent=2))
    elif args.cmd == "inspection-start":
        result = robot.tasks.start_inspection(
            args.route_id,
            map_id=args.map_id,
            revision=args.revision,
            request_id=args.request_id,
        )
        print(json.dumps(result.raw, ensure_ascii=False, indent=2))
    elif args.cmd == "inspection-status":
        print(json.dumps(robot.tasks.inspection_task(args.task_id), ensure_ascii=False, indent=2))
    elif args.cmd in {"inspection-pause", "inspection-resume", "inspection-cancel"}:
        action = args.cmd.removeprefix("inspection-")
        result = getattr(robot.tasks, f"{action}_inspection")(
            args.task_id,
            reason=args.reason,
            request_id=args.request_id,
        )
        print(json.dumps(result.raw, ensure_ascii=False, indent=2))
    elif args.cmd == "inspection-report":
        print(json.dumps(robot.tasks.inspection_report(args.task_id), ensure_ascii=False, indent=2))
    else:
        p.print_help()

    robot.close()


if __name__ == "__main__":
    main()

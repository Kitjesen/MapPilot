"""LingTu CLI — quick robot control from terminal."""
import argparse, sys
from lingtu_sdk import LingTuClient

def main():
    p = argparse.ArgumentParser(description="LingTu Robot CLI")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5050)
    sub = p.add_subparsers(dest="cmd")

    # lingtu go 10 5
    go = sub.add_parser("go")
    go.add_argument("x", type=float); go.add_argument("y", type=float)
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

    args = p.parse_args()
    robot = LingTuClient(args.host, args.port)

    if args.cmd == "go":       print(robot.go(args.x, args.y, args.yaw))
    elif args.cmd == "stop":   print(robot.stop())
    elif args.cmd == "state":  print(robot.state())
    elif args.cmd == "health": print(robot.health())
    elif args.cmd == "maps":   print(robot.maps())
    elif args.cmd == "save-map": print(robot.save_map(args.name))

    robot.close()

if __name__ == "__main__":
    main()

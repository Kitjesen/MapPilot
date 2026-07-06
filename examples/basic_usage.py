"""LingTu SDK Quickstart.

Demonstrates the core ``LingTuClient`` API for navigating, inspecting state,
managing maps, and controlling the robot.

Requirements:
    pip install lingtu-sdk

Usage:
    python examples/basic_usage.py              # connect to localhost
    python examples/basic_usage.py 192.168.66.190  # connect to robot
"""

from __future__ import annotations

import sys

from lingtu.sdk import LingTuClient


def main(host: str = "127.0.0.1") -> None:
    """Run the quickstart demonstration."""
    robot = LingTuClient(host, port=5050)

    # -- Navigate ---------------------------------------------------------
    # Absolute coordinates in the map frame
    robot.go(10.0, 5.0)                  # go to (10, 5)
    robot.go(5.0, 3.0, yaw=1.57)        # go to (5, 3), facing east

    # Semantic target -- sends a natural-language instruction to the planner
    robot.go_to("会议室")                 # "go to the meeting room"

    # Direct velocity command (bypasses planner)
    robot.drive(vx=0.5, wz=0.0)          # drive forward at 0.5 m/s

    # Emergency stop
    robot.stop()

    # -- State ------------------------------------------------------------
    pos = robot.position()
    print(f"Position: x={pos.x:.2f}, y={pos.y:.2f}, yaw={pos.yaw:.2f}")

    full = robot.state()
    print(f"Mode: {full.mode}, Safety: {full.safety_level}")

    health = robot.health()
    print(f"Modules: {health.modules_ok}/{health.modules_total} ok")

    ns = robot.navigation_status()
    print(f"Mission: {ns.state}, dist to goal: {ns.distance_to_goal:.2f}m")

    # -- Maps -------------------------------------------------------------
    ml = robot.maps()
    print(f"Active map: {ml.active_map}")
    for m in ml.maps:
        print(f"  Map: {m.name}  PCD={m.has_pcd}  Active={m.is_active}")

    robot.save_map("lab_0602")             # save current SLAM map
    robot.use_map("lab_0602")              # activate for navigation

    # -- Mode -------------------------------------------------------------
    robot.set_mode("autonomous")           # engage autonomous navigation
    robot.set_mode("manual")               # release to manual control

    # -- Hot-swap driver --------------------------------------------------
    robot.swap("sim_mujoco")               # switch to simulated driver

    # -- Tagged locations -------------------------------------------------
    robot.tag_location("充电站", use_current_pose=True)
    locs = robot.locations()
    for loc in locs.get("locations", []):
        print(f"  Tag: {loc.get('name')}  "
              f"({loc.get('x', 0):.1f}, {loc.get('y', 0):.1f})")

    # -- Batch waypoint navigation ---------------------------------------
    # robot.batch_go([(10, 5), (15, 8), (20, 10)])

    # -- Wait for arrival -------------------------------------------------
    # robot.go(10.0, 5.0)
    # status = robot.wait_until_arrived(timeout=60.0)
    # print(f"Arrived! Dist: {status.distance_to_goal:.2f}m")

    # -- SLAM operations --------------------------------------------------
    st = robot.slam_status()
    print(f"SLAM: {st.get('state', '?')}")

    # -- Cleanup ----------------------------------------------------------
    robot.close()


if __name__ == "__main__":
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    main(host)

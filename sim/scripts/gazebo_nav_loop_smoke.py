#!/usr/bin/env python3
"""ROS 2 smoke test for Gazebo-backed navigation closure.

Publishes a map-frame goal and verifies the navigation chain reacts while
Gazebo remains the odometry source:

    /nav/goal_pose -> /nav/global_path -> /nav/local_path -> /nav/cmd_vel
    /nav/cmd_vel -> Gazebo diff drive -> /slam/odometry movement

The script publishes only /nav/goal_pose. It never publishes cmd_vel and is
intended for simulation gates only.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass, field


NAV_TOPIC_PUBLISHER_ALLOWLIST: dict[str, set[str]] = {
    "/slam/odometry": {
        "lingtu_ros2_driver",
        "lingtu_gazebo_runtime_adapter",
        "sim_robot_node",
    },
    "/nav/global_path": {
        "global_planner",
        "pct_path_adapter",
        "lingtu_gazebo_line_global_planner",
        "nav.mission",
        "navigation",
    },
    "/nav/local_path": {
        "localPlanner",
        "local_planner",
        "nav.local_planner",
    },
    "/nav/cmd_vel": {
        "pathFollower",
        "path_follower",
        "nav.path_follower",
        "path_follower",
        "lingtu_ros2_driver",
    },
}


@dataclass
class GazeboNavLoopResult:
    ok: bool = False
    simulation_only: bool = True
    real_robot_motion: bool = False
    cmd_vel_sent_to_hardware: bool = False
    goal_published: bool = False
    odometry_seen: bool = False
    global_path_seen: bool = False
    local_path_seen: bool = False
    cmd_vel_seen: bool = False
    cmd_vel_nonzero: bool = False
    odom_start_xy: tuple[float, float] | None = None
    odom_last_xy: tuple[float, float] | None = None
    odom_delta_m: float = 0.0
    odom_delta_x_m: float = 0.0
    cmd_vel_linear_x_max: float = 0.0
    cmd_vel_linear_x_min: float = 0.0
    cmd_vel_angular_z_abs_max: float = 0.0
    stop_last: int | None = None
    stop_max: int | None = None
    samples: dict[str, int] = field(default_factory=dict)
    publisher_contract: dict[str, object] = field(default_factory=dict)
    errors: list[str] = field(default_factory=list)

    def as_dict(self) -> dict:
        return {
            "schema_version": "lingtu.gazebo_nav_loop.v1",
            "ok": self.ok,
            "simulation_only": self.simulation_only,
            "real_robot_motion": self.real_robot_motion,
            "cmd_vel_sent_to_hardware": self.cmd_vel_sent_to_hardware,
            "goal_published": self.goal_published,
            "odometry_seen": self.odometry_seen,
            "global_path_seen": self.global_path_seen,
            "local_path_seen": self.local_path_seen,
            "cmd_vel_seen": self.cmd_vel_seen,
            "cmd_vel_nonzero": self.cmd_vel_nonzero,
            "odom_start_xy": self.odom_start_xy,
            "odom_last_xy": self.odom_last_xy,
            "odom_delta_m": self.odom_delta_m,
            "odom_delta_x_m": self.odom_delta_x_m,
            "cmd_vel_linear_x_max": self.cmd_vel_linear_x_max,
            "cmd_vel_linear_x_min": self.cmd_vel_linear_x_min,
            "cmd_vel_angular_z_abs_max": self.cmd_vel_angular_z_abs_max,
            "stop_last": self.stop_last,
            "stop_max": self.stop_max,
            "samples": self.samples,
            "publisher_contract": self.publisher_contract,
            "errors": self.errors,
        }


def _count(samples: dict[str, int], topic: str) -> None:
    samples[topic] = samples.get(topic, 0) + 1


def _publisher_contract(node) -> dict[str, object]:
    topics: dict[str, dict[str, object]] = {}
    errors: list[str] = []
    for topic, allowed in NAV_TOPIC_PUBLISHER_ALLOWLIST.items():
        infos = node.get_publishers_info_by_topic(topic)
        publishers = sorted(
            f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")
            for info in infos
        )
        names = {item.rsplit("/", maxsplit=1)[-1] for item in publishers}
        disallowed = sorted(name for name in names if name not in allowed)
        ok = bool(publishers) and not disallowed
        if not publishers:
            errors.append(f"{topic} has no publishers")
        if disallowed:
            errors.append(
                f"{topic} has disallowed publishers: {', '.join(disallowed)}"
            )
        topics[topic] = {
            "ok": ok,
            "publishers": publishers,
            "allowed_node_names": sorted(allowed),
            "disallowed_node_names": disallowed,
        }
    return {
        "ok": not errors,
        "topics": topics,
        "errors": errors,
    }


def run_smoke(args: argparse.Namespace) -> GazeboNavLoopResult:
    import rclpy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from nav_msgs.msg import Odometry, Path
    from rclpy.node import Node
    from std_msgs.msg import Int8

    rclpy.init(args=None)
    node = Node("lingtu_gazebo_nav_loop_smoke")
    result = GazeboNavLoopResult()
    goal_pub = node.create_publisher(PoseStamped, "/nav/goal_pose", 10)

    def on_odom(msg: Odometry) -> None:
        _count(result.samples, "/slam/odometry")
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if not result.odometry_seen:
            result.odom_start_xy = (x, y)
        result.odometry_seen = True
        result.odom_last_xy = (x, y)
        if result.odom_start_xy is not None:
            result.odom_delta_x_m = x - result.odom_start_xy[0]
            result.odom_delta_m = math.hypot(x - result.odom_start_xy[0], y - result.odom_start_xy[1])

    def on_global_path(msg: Path) -> None:
        _count(result.samples, "/nav/global_path")
        result.global_path_seen = result.global_path_seen or bool(msg.poses)

    def on_local_path(msg: Path) -> None:
        _count(result.samples, "/nav/local_path")
        result.local_path_seen = result.local_path_seen or bool(msg.poses)

    def on_cmd_vel(msg: TwistStamped) -> None:
        _count(result.samples, "/nav/cmd_vel")
        result.cmd_vel_seen = True
        tw = msg.twist
        vx = float(tw.linear.x)
        wz = float(tw.angular.z)
        result.cmd_vel_linear_x_max = max(result.cmd_vel_linear_x_max, vx)
        result.cmd_vel_linear_x_min = min(result.cmd_vel_linear_x_min, vx)
        result.cmd_vel_angular_z_abs_max = max(result.cmd_vel_angular_z_abs_max, abs(wz))
        result.cmd_vel_nonzero = result.cmd_vel_nonzero or (
            abs(vx) > args.min_cmd_vel
            or abs(float(tw.linear.y)) > args.min_cmd_vel
            or abs(wz) > args.min_cmd_vel
        )

    def on_stop(msg: Int8) -> None:
        _count(result.samples, "/nav/stop")
        value = int(msg.data)
        result.stop_last = value
        result.stop_max = value if result.stop_max is None else max(result.stop_max, value)

    node.create_subscription(Odometry, "/slam/odometry", on_odom, 10)
    node.create_subscription(Path, "/nav/global_path", on_global_path, 10)
    node.create_subscription(Path, "/nav/local_path", on_local_path, 10)
    node.create_subscription(TwistStamped, "/nav/cmd_vel", on_cmd_vel, 10)
    node.create_subscription(Int8, "/nav/stop", on_stop, 10)

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = float(args.goal_x)
    goal.pose.position.y = float(args.goal_y)
    goal.pose.position.z = float(args.goal_z)
    goal.pose.orientation.w = 1.0

    deadline = time.monotonic() + args.timeout_sec
    next_goal_publish = time.monotonic() + args.goal_delay_sec
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_goal_publish and now <= deadline - 1.0:
                goal.header.stamp = node.get_clock().now().to_msg()
                goal_pub.publish(goal)
                result.goal_published = True
                next_goal_publish = now + args.goal_republish_sec
            rclpy.spin_once(node, timeout_sec=0.1)
            forward_ready = (
                not args.require_forward_progress
                or (
                    result.cmd_vel_linear_x_max > args.min_cmd_vel
                    and result.odom_delta_x_m >= args.min_forward_odom_x_m
                )
            )
            if (
                result.global_path_seen
                and result.local_path_seen
                and result.cmd_vel_nonzero
                and result.odom_delta_m >= args.min_odom_delta_m
                and forward_ready
            ):
                break
    finally:
        result.publisher_contract = _publisher_contract(node)
        node.destroy_node()
        rclpy.shutdown()

    if not result.goal_published:
        result.errors.append("/nav/goal_pose was not published")
    if not result.odometry_seen:
        result.errors.append("/slam/odometry was not observed")
    if not result.global_path_seen:
        result.errors.append("/nav/global_path with poses was not observed")
    if not result.local_path_seen:
        result.errors.append("/nav/local_path with poses was not observed")
    if not result.cmd_vel_seen:
        result.errors.append("/nav/cmd_vel was not observed")
    elif not result.cmd_vel_nonzero:
        result.errors.append("/nav/cmd_vel never became non-zero")
    if result.odom_delta_m < args.min_odom_delta_m:
        result.errors.append(
            f"/slam/odometry moved {result.odom_delta_m:.3f} m, "
            f"expected >= {args.min_odom_delta_m:.3f} m"
        )
    if args.require_forward_progress:
        if result.cmd_vel_linear_x_max <= args.min_cmd_vel:
            result.errors.append(
                "forward goal did not produce positive cmd_vel.linear.x: "
                f"max={result.cmd_vel_linear_x_max:.3f}"
            )
        if result.odom_delta_x_m < args.min_forward_odom_x_m:
            result.errors.append(
                "forward goal did not produce positive odom x progress: "
                f"dx={result.odom_delta_x_m:.3f}, expected >= {args.min_forward_odom_x_m:.3f}"
            )
    publisher_errors = list((result.publisher_contract or {}).get("errors") or [])
    result.errors.extend(publisher_errors)
    result.ok = not result.errors
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    parser.add_argument("--goal-delay-sec", type=float, default=2.0)
    parser.add_argument("--goal-republish-sec", type=float, default=1.0)
    parser.add_argument("--goal-x", type=float, default=2.0)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--goal-z", type=float, default=0.0)
    parser.add_argument("--min-cmd-vel", type=float, default=0.01)
    parser.add_argument("--min-odom-delta-m", type=float, default=0.05)
    parser.add_argument("--require-forward-progress", action="store_true")
    parser.add_argument("--min-forward-odom-x-m", type=float, default=0.03)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--json-out", default="")
    args = parser.parse_args()

    try:
        result = run_smoke(args)
    except ImportError as exc:
        result = GazeboNavLoopResult(errors=[f"ROS 2 Python dependencies unavailable: {exc}"])
    payload = json.dumps(result.as_dict(), ensure_ascii=False, indent=2)
    if args.json:
        print(payload)
    else:
        print(("PASSED" if result.ok else "FAILED") + ": LingTu Gazebo nav loop smoke")
        print(payload)
    if args.json_out:
        from pathlib import Path

        path = Path(args.json_out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(payload + "\n", encoding="utf-8")
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

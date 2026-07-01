#!/usr/bin/env python3
"""lingtu doctor - check field services, hardware, and data flow."""

from __future__ import annotations

import json
import os
import shlex
import socket
import subprocess
import sys
import time
from pathlib import Path


def find_repo_root(start: Path) -> Path:
    for path in (start, *start.parents):
        if (path / "pyproject.toml").is_file() and (path / "AGENTS.md").is_file():
            return path
    raise RuntimeError(f"Could not find repository root from {start}")


REPO_ROOT = find_repo_root(Path(__file__).resolve().parent)


def check(name: str, ok: bool, detail: str = "") -> bool:
    mark = "\033[32mOK\033[0m" if ok else "\033[31mFAIL\033[0m"
    print("  [%s] %s%s" % (mark, name, ("  " + detail) if detail else ""))
    return ok


def service_active(name: str) -> bool:
    try:
        result = subprocess.run(["systemctl", "is-active", "--quiet", name], timeout=3)
        return result.returncode == 0
    except Exception:
        return False


def port_open(host: str, port: int) -> bool:
    sock = socket.socket()
    sock.settimeout(1)
    try:
        sock.connect((host, port))
        sock.close()
        return True
    except Exception:
        return False


def run(cmd: list[str], timeout: int = 5) -> str:
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout,
            check=False,
        )
        return result.stdout.strip()
    except Exception:
        return ""


def ros2_topic_list() -> str:
    setup_files = [
        os.environ.get("LINGTU_ROS_SETUP", "/opt/ros/humble/setup.bash"),
        os.environ.get("LINGTU_ROS_OVERLAY_SETUP", "/opt/nova/lingtu/v1.8.0/install/setup.bash"),
    ]
    source_cmds = " && ".join(
        f"[ -f {shlex.quote(path)} ] && source {shlex.quote(path)} || true"
        for path in setup_files
    )
    return run(["bash", "-lc", f"{source_cmds} && ros2 topic list 2>/dev/null"], timeout=5)


def read_run_state() -> dict[str, object]:
    path = REPO_ROOT / ".lingtu" / "run.json"
    if not path.is_file():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


print("\n  \033[1mLingTu Doctor\033[0m\n")

print("  --- Services ---")
for svc in ["brainstem", "lidar", "camera"]:
    check(svc, service_active(svc))
for svc in ["slam", "slam_pgo", "localizer"]:
    active = service_active(svc)
    check(svc, True, "active" if active else "inactive (on-demand)")

print("\n  --- Hardware ---")
lsusb = run(["lsusb"])
livox_ping = run(["ping", "-c1", "-W1", "192.168.1.115"])
check("LiDAR (Livox)", "192.168.1.115" in livox_ping, "192.168.1.115")
check("Camera (Orbbec)", "orbbec" in lsusb.lower() or "2bc5" in lsusb.lower())
check("IMU (CP210x)", "cp210x" in lsusb.lower() or "10c4" in lsusb.lower())
check("Xbox Controller", "8bitdo" in lsusb.lower() or "xbox" in lsusb.lower())
check("CAN Bus", os.path.exists("/sys/class/net/can0"))

print("\n  --- Ports ---")
check("brainstem gRPC :13145", port_open("127.0.0.1", 13145))
for name, port in [("Gateway :5050", 5050), ("MCP :8090", 8090)]:
    if port_open("127.0.0.1", port):
        check(name, True)
    else:
        print("  [\033[33m--\033[0m] %s  (lingtu not running)" % name)

print("\n  --- Runtime State ---")
state = read_run_state()
if state:
    check("run.json", True, "profile=%s pid=%s" % (state.get("profile", "?"), state.get("pid", "?")))
else:
    check("run.json", False, "LingTu daemon state not found")

print("\n  --- ROS2 Topics ---")
topics = ros2_topic_list()
for topic in [
    "/nav/odometry",
    "/nav/map_cloud",
    "/nav/registered_cloud",
    "/camera/color/image_raw",
    "/camera/depth/image_raw",
    "/nav/lidar_scan",
]:
    check(topic, topic in topics)

slam_was_off = not service_active("slam")
auto_started_slam = False
if slam_was_off and os.environ.get("LINGTU_DOCTOR_AUTOSTART_SLAM", "0") == "1":
    print("\n  [..] Starting slam for data flow test...")
    subprocess.run(["sudo", "systemctl", "start", "slam"], capture_output=True, timeout=5, check=False)
    time.sleep(8)
    auto_started_slam = True
elif slam_was_off:
    print("\n  [--] slam is inactive; set LINGTU_DOCTOR_AUTOSTART_SLAM=1 to start it for data flow checks.")

print("\n  --- Data Flow (3s) ---")
sys.path.insert(0, str(REPO_ROOT / "src"))
try:
    from compat.ros2.context import ensure_rclpy, get_shared_executor, shutdown_shared_executor

    ensure_rclpy()
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image, PointCloud2

    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=5)
    counts = {"odom": 0, "cloud": 0, "color": 0, "depth": 0}
    node = Node("doctor")
    node.create_subscription(Odometry, "/nav/odometry", lambda _msg: counts.__setitem__("odom", counts["odom"] + 1), qos)
    node.create_subscription(PointCloud2, "/nav/map_cloud", lambda _msg: counts.__setitem__("cloud", counts["cloud"] + 1), qos)
    node.create_subscription(Image, "/camera/color/image_raw", lambda _msg: counts.__setitem__("color", counts["color"] + 1), qos)
    node.create_subscription(Image, "/camera/depth/image_raw", lambda _msg: counts.__setitem__("depth", counts["depth"] + 1), qos)
    executor = get_shared_executor()
    executor.add_node(node)
    time.sleep(3)
    check("SLAM odometry", counts["odom"] > 0, "%d msgs (%.0f Hz)" % (counts["odom"], counts["odom"] / 3))
    check("SLAM cloud", counts["cloud"] > 0, "%d msgs" % counts["cloud"])
    check("Camera color", counts["color"] > 0, "%d msgs" % counts["color"])
    check("Camera depth", counts["depth"] > 0, "%d msgs" % counts["depth"])
    node.destroy_node()
    shutdown_shared_executor()
except Exception as exc:
    print("  [SKIP] Data flow test: %s" % exc)

if auto_started_slam:
    subprocess.run(["sudo", "systemctl", "stop", "slam"], capture_output=True, timeout=5, check=False)

print("\n  --- Maps ---")
map_dir = os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/nova/maps"))
if os.path.isdir(map_dir):
    maps = [
        name
        for name in os.listdir(map_dir)
        if os.path.isdir(os.path.join(map_dir, name)) and name != "active"
    ]
    active_path = os.path.join(map_dir, "active")
    active = os.path.basename(os.readlink(active_path)) if os.path.islink(active_path) else "none"
    check("Maps directory", True, "%d maps, active=%s" % (len(maps), active))
else:
    check("Maps directory", False, map_dir)

print("\n  --- Python ---")
try:
    import lingtu  # noqa: F401

    check("lingtu importable", True)
except Exception as exc:
    check("lingtu importable", False, str(exc))

try:
    from core.blueprints.profile_builder import blueprint_for_resolved_profile
    from core.runtime.resolver import resolve_profile_config

    cfg = resolve_profile_config("thunder-nav", overrides={"run_startup_checks": False})
    blueprint_for_resolved_profile("thunder-nav", cfg)
    check("Thunder profile builder", True)
except Exception as exc:
    check("Thunder profile builder", False, str(exc))

print()

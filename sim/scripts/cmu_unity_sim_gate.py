#!/usr/bin/env python3
"""Preflight gate for the isolated CMU Unity/TARE benchmark workspace.

This gate is intentionally conservative. It does not launch Unity or publish
any ROS command. It verifies that the external CMU workspace, Unity assets,
colcon build output, and LingTu TARE remapping contract exist before anyone can
claim that the CMU/Unity simulation line is ready for runtime validation.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
for candidate in (ROOT / "src", ROOT):
    path = str(candidate)
    if path not in sys.path:
        sys.path.insert(0, path)

from cli.profiles_data import PROFILES
from nav.exploration.tare.topics import TARE_REMAPS
from runtime.runtime_interface import TOPICS
from sim.engine.bridge.cmu_unity_lingtu_adapter import required_relay_contract

SCHEMA_VERSION = "lingtu.cmu_unity_sim_gate.v1"
CMU_REPO_URL = "https://github.com/jizhang-cmu/autonomy_stack_mecanum_wheel_platform.git"
DEFAULT_CMU_WORKSPACE = ROOT.parent / "autonomy_stack_mecanum_wheel_platform"
EXPECTED_BRANCH = "humble"
ROS_HUMBLE_SETUP = Path("/opt/ros/humble/setup.bash")
ROS2_FALLBACK = Path("/opt/ros/humble/bin/ros2")

REQUIRED_CMU_PATHS: dict[str, str] = {
    "system_simulation": "system_simulation.sh",
    "system_simulation_with_exploration": "system_simulation_with_exploration_planner.sh",
    "vehicle_system_launch": "src/base_autonomy/vehicle_simulator/launch/system_simulation.launch",
    "vehicle_exploration_launch": (
        "src/base_autonomy/vehicle_simulator/launch/"
        "system_simulation_with_exploration_planner.launch"
    ),
    "vehicle_simulator_rviz": "src/base_autonomy/vehicle_simulator/rviz/vehicle_simulator.rviz",
    "tare_explore_world_launch": "src/exploration_planner/tare_planner/launch/explore_world.launch",
    "tare_explore_launch": "src/exploration_planner/tare_planner/launch/explore.launch",
    "tare_indoor_small_config": "src/exploration_planner/tare_planner/config/indoor_small.yaml",
    "vehicle_simulator_source": "src/base_autonomy/vehicle_simulator/src/vehicleSimulator.cpp",
    "local_planner_source": "src/base_autonomy/local_planner/src/localPlanner.cpp",
    "path_follower_source": "src/base_autonomy/local_planner/src/pathFollower.cpp",
}

UNITY_ENVIRONMENT_PATHS: dict[str, str] = {
    "model_binary": "src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64",
    "unity_player": "src/base_autonomy/vehicle_simulator/mesh/unity/environment/UnityPlayer.so",
    "model_data": "src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model_Data",
}

BUILD_PATHS: dict[str, str] = {
    "setup_bash": "install/setup.bash",
}

CMU_TOPIC_CONTRACT: dict[str, str] = {
    "/registered_scan": "vehicle/local planner scan input",
    "/state_estimation": "simulated odometry/state estimate",
    "/state_estimation_at_scan": "scan-synchronized odometry",
    "/terrain_map": "local terrain cloud",
    "/terrain_map_ext": "extended terrain cloud",
    "/way_point": "TARE/FAR waypoint output",
    "/path": "local planner path output",
    "/cmd_vel": "simulated velocity command",
    "/navigation_boundary": "FAR/TARE exploration boundary input",
}


def _topic_contract_token_present(topic: str, source: str) -> bool:
    if topic in source:
        return True
    for name, value in vars(TOPICS).items():
        if value == topic and f"TOPICS.{name}" in source:
            return True
    return False

OPTIONAL_CMU_TOPIC_CONTRACT: dict[str, str] = {
    "/global_path_full": "TARE full global exploration strategy path",
    "/global_path": "TARE trimmed global exploration strategy path",
    "/local_path": "TARE local exploration strategy path",
}

LINGTU_ADAPTER_SAFETY_TOKENS: tuple[str, ...] = (
    "--relay-cmd-vel-to-sim",
    "--allow-default-ros-domain",
    "Refusing to relay /nav/cmd_vel -> /cmd_vel",
    "Simulation-only",
)

LINGTU_CMU_STACK_TOKENS: tuple[str, ...] = (
    "robot=\"sim_endpoint\"",
    "slam_profile=\"none\"",
    "CMUUnityEndpointRuntime",
    "enable_endpoint_waypoint_bridge=False",
    "enable_nav_out=False",
    "enable_frontier=args.enable_frontier",
    "allow_direct_goal_fallback=not args.disable_direct_goal_fallback",
    "direct_goal_fallback_on_planner_failure=not args.disable_direct_goal_fallback",
    "exploration_backend=\"none\" if args.enable_frontier else \"tare_external\"",
    "manage_external_services=False",
    "run_startup_checks=False",
    "latch_stop_signal=False",
    "ROS_DOMAIN_ID",
)


def _add_check(
    checks: list[dict[str, Any]],
    name: str,
    ok: bool,
    *,
    required: bool = True,
    detail: Any = None,
) -> None:
    item: dict[str, Any] = {
        "name": name,
        "ok": bool(ok),
        "required": bool(required),
    }
    if detail is not None:
        item["detail"] = detail
    checks.append(item)


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except FileNotFoundError:
        return ""


def _run_git(workspace: Path, args: list[str]) -> str:
    try:
        result = subprocess.run(
            ["git", "-C", str(workspace), *args],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except Exception:
        return ""
    if result.returncode != 0:
        return ""
    return result.stdout.strip()


def _command_ok(command: list[str]) -> bool:
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except Exception:
        return False
    return result.returncode == 0


def _ros2_help_ok() -> bool:
    if not ROS_HUMBLE_SETUP.exists():
        return False
    return _command_ok([
        "bash",
        "-lc",
        f"source {ROS_HUMBLE_SETUP} && ros2 --help",
    ])


def _path_checks(workspace: Path, paths: dict[str, str]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for name, relative in paths.items():
        path = workspace / relative
        result[name] = {
            "path": str(path),
            "exists": path.exists(),
            "is_file": path.is_file(),
            "is_dir": path.is_dir(),
        }
    return result


def _all_existing(path_result: dict[str, dict[str, Any]]) -> bool:
    return all(bool(item.get("exists")) for item in path_result.values())


def _check_cmu_workspace(
    workspace: Path,
    *,
    require_git: bool,
    require_humble_branch: bool,
    require_unity_model: bool,
    require_build: bool,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    checks: list[dict[str, Any]] = []

    exists = workspace.exists() and workspace.is_dir()
    _add_check(checks, "cmu_workspace_exists", exists, detail=str(workspace))
    if not exists:
        return (
            {
                "path": str(workspace),
                "exists": False,
                "repo_url": CMU_REPO_URL,
            },
            checks,
        )

    git_dir_exists = (workspace / ".git").exists()
    branch = _run_git(workspace, ["branch", "--show-current"]) if git_dir_exists else ""
    head = _run_git(workspace, ["rev-parse", "--short", "HEAD"]) if git_dir_exists else ""
    remote = _run_git(workspace, ["config", "--get", "remote.origin.url"]) if git_dir_exists else ""
    _add_check(
        checks,
        "cmu_git_workspace",
        git_dir_exists,
        required=require_git,
        detail={"branch": branch, "head": head, "remote": remote},
    )
    if require_humble_branch:
        _add_check(
            checks,
            "cmu_humble_branch",
            branch == EXPECTED_BRANCH,
            detail={"expected": EXPECTED_BRANCH, "actual": branch},
        )

    required_paths = _path_checks(workspace, REQUIRED_CMU_PATHS)
    _add_check(checks, "cmu_required_source_paths", _all_existing(required_paths), detail=required_paths)

    unity_paths = _path_checks(workspace, UNITY_ENVIRONMENT_PATHS)
    _add_check(
        checks,
        "cmu_unity_environment_assets",
        _all_existing(unity_paths),
        required=require_unity_model,
        detail=unity_paths,
    )

    build_paths = _path_checks(workspace, BUILD_PATHS)
    _add_check(
        checks,
        "cmu_colcon_build_output",
        _all_existing(build_paths),
        required=require_build,
        detail=build_paths,
    )

    combined_text = "\n".join(
        _read_text(workspace / relative)
        for relative in (
            REQUIRED_CMU_PATHS["vehicle_system_launch"],
            REQUIRED_CMU_PATHS["vehicle_exploration_launch"],
            REQUIRED_CMU_PATHS["tare_explore_launch"],
            REQUIRED_CMU_PATHS["tare_indoor_small_config"],
            REQUIRED_CMU_PATHS["vehicle_simulator_source"],
            REQUIRED_CMU_PATHS["local_planner_source"],
            REQUIRED_CMU_PATHS["path_follower_source"],
        )
    )
    topic_presence = {
        topic: (topic in combined_text)
        for topic in CMU_TOPIC_CONTRACT
    }
    optional_topic_presence = {
        topic: (topic in combined_text)
        for topic in OPTIONAL_CMU_TOPIC_CONTRACT
    }
    _add_check(
        checks,
        "cmu_topic_contract",
        all(topic_presence.values()),
        detail={"topics": topic_presence, "optional_topics": optional_topic_presence},
    )

    return (
        {
            "path": str(workspace),
            "exists": True,
            "repo_url": CMU_REPO_URL,
            "branch": branch,
            "head": head,
            "remote": remote,
            "required_paths": required_paths,
            "unity_environment": unity_paths,
            "build": build_paths,
            "topic_contract": topic_presence,
            "optional_topic_contract": optional_topic_presence,
        },
        checks,
    )


def _check_lingtu_contract() -> tuple[dict[str, Any], list[dict[str, Any]]]:
    checks: list[dict[str, Any]] = []
    tare_topics = ROOT / "src/nav/exploration/tare/topics.py"
    profiles = ROOT / "cli/profiles_data.py"
    adapter_path = ROOT / "sim/engine/bridge/cmu_unity_lingtu_adapter.py"
    stack_script = ROOT / "sim/scripts/cmu_unity_lingtu_stack.py"
    topic_text = _read_text(tare_topics)
    tare_profile = PROFILES.get("tare_explore", {})
    cmu_profile = PROFILES.get("sim_cmu_tare", {})
    adapter_text = _read_text(adapter_path)
    stack_text = _read_text(stack_script)

    uses_adapter_alias_contract = 'adapter_remappings("tare")' in topic_text
    remap_presence = {
        f"{src}->{dst}": (
            TARE_REMAPS.get(src) == dst
            and (
                uses_adapter_alias_contract
                or (src in topic_text and _topic_contract_token_present(dst, topic_text))
            )
        )
        for src, dst in TARE_REMAPS.items()
    }
    _add_check(
        checks,
        "lingtu_tare_remap_contract",
        all(remap_presence.values()),
        detail={
            "file": str(tare_topics),
            "uses_adapter_alias_contract": uses_adapter_alias_contract,
            "remaps": remap_presence,
        },
    )

    profile_required_tokens = {
        "tare_explore": "tare_explore" in PROFILES,
        "wavefront_frontier_disabled": tare_profile.get("enable_frontier") is False,
        "traversable_frontier_disabled": (
            tare_profile.get("enable_traversable_frontier") is False
        ),
        "exploration_backend_tare": (
            tare_profile.get("exploration_backend") == "tare"
        ),
        "planner_octoplanner3d": tare_profile.get("planner") == "octoplanner3d",
    }
    _add_check(
        checks,
        "lingtu_tare_explore_profile",
        all(profile_required_tokens.values()),
        detail={"file": str(profiles), "tokens": profile_required_tokens},
    )
    cmu_profile_required_tokens = {
        "sim_cmu_tare": "sim_cmu_tare" in PROFILES,
        "runtime_contract": cmu_profile.get("_runtime_contract") == "cmu_unity_external",
        "external_launcher": (
            cmu_profile.get("_external_launcher")
            == "sim/scripts/launch_cmu_unity_lingtu_runtime.sh"
        ),
        "robot_sim_endpoint": cmu_profile.get("_default_robot") == "sim_endpoint",
        "slam_none": cmu_profile.get("slam_profile") == "none",
        "exploration_backend_tare_external": (
            cmu_profile.get("exploration_backend") == "tare_external"
        ),
        "endpoint_waypoint_bridge_disabled": (
            cmu_profile.get("enable_endpoint_waypoint_bridge") is False
        ),
        "nav_out_disabled": (
            cmu_profile.get("enable_nav_out") is False
        ),
    }
    _add_check(
        checks,
        "lingtu_cmu_tare_profile",
        all(cmu_profile_required_tokens.values()),
        detail={"file": str(profiles), "tokens": cmu_profile_required_tokens},
    )
    _add_check(
        checks,
        "lingtu_cmu_adapter_exists",
        adapter_path.exists(),
        detail=str(adapter_path),
    )
    _add_check(
        checks,
        "lingtu_cmu_stack_script_exists",
        stack_script.exists(),
        detail=str(stack_script),
    )

    relay_presence: dict[str, bool] = {}
    relay_errors: list[str] = []
    try:
        adapter_required_relays = required_relay_contract(relay_cmd_vel_to_sim=True)
        relay_presence = {relay: bool(msg_type) for relay, msg_type in adapter_required_relays.items()}
    except Exception as exc:
        adapter_required_relays = {}
        relay_presence = {}
        relay_errors.append(str(exc))

    _add_check(
        checks,
        "lingtu_cmu_adapter_relay_contract",
        bool(relay_presence) and all(relay_presence.values()),
        detail={
            "file": str(adapter_path),
            "relays": relay_presence,
            "errors": relay_errors,
        },
    )

    safety_presence = {
        token: (token in adapter_text)
        for token in LINGTU_ADAPTER_SAFETY_TOKENS
    }
    _add_check(
        checks,
        "lingtu_cmu_adapter_safety_contract",
        all(safety_presence.values()),
        detail={
            "adapter": str(adapter_path),
            "tokens": safety_presence,
        },
    )

    stack_presence = {token: token in stack_text for token in LINGTU_CMU_STACK_TOKENS}
    _add_check(
        checks,
        "lingtu_cmu_stack_simulation_contract",
        stack_script.exists() and all(stack_presence.values()),
        detail={
            "script": str(stack_script),
            "tokens": stack_presence,
        },
    )

    return (
        {
            "tare_topics": str(tare_topics),
            "profiles": str(profiles),
            "adapter": str(adapter_path),
            "stack_script": str(stack_script),
            "remaps": TARE_REMAPS,
            "profile_tokens": profile_required_tokens,
            "cmu_profile_tokens": cmu_profile_required_tokens,
            "adapter_required_relays": adapter_required_relays,
        },
        checks,
    )


def _check_host_tooling(require_ros: bool) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    checks: list[dict[str, Any]] = []
    ros2_path = shutil.which("ros2")
    if ros2_path is None and ROS2_FALLBACK.exists():
        ros2_path = str(ROS2_FALLBACK)
    colcon_path = shutil.which("colcon")

    _add_check(
        checks,
        "host_ros_humble_setup",
        ROS_HUMBLE_SETUP.exists(),
        required=require_ros,
        detail=str(ROS_HUMBLE_SETUP),
    )
    _add_check(
        checks,
        "host_ros2_cli",
        ros2_path is not None,
        required=require_ros,
        detail=ros2_path or "",
    )
    _add_check(
        checks,
        "host_ros2_cli_functional",
        bool(ros2_path) and _ros2_help_ok(),
        required=require_ros,
        detail={
            "ros2": ros2_path or "",
            "setup": str(ROS_HUMBLE_SETUP),
            "command": f"source {ROS_HUMBLE_SETUP} && ros2 --help",
        },
    )
    _add_check(
        checks,
        "host_colcon_cli",
        colcon_path is not None,
        required=require_ros,
        detail=colcon_path or "",
    )
    _add_check(
        checks,
        "host_colcon_cli_functional",
        bool(colcon_path) and _command_ok([str(colcon_path), "--help"]),
        required=require_ros,
        detail=colcon_path or "",
    )
    return (
        {
            "ros_humble_setup": str(ROS_HUMBLE_SETUP),
            "ros2": ros2_path,
            "colcon": colcon_path,
        },
        checks,
    )


def build_report(args: argparse.Namespace) -> dict[str, Any]:
    workspace = Path(args.cmu_workspace).expanduser().resolve()
    checks: list[dict[str, Any]] = []

    host, host_checks = _check_host_tooling(require_ros=args.require_ros)
    checks.extend(host_checks)

    cmu, cmu_checks = _check_cmu_workspace(
        workspace,
        require_git=args.require_git,
        require_humble_branch=args.require_humble_branch,
        require_unity_model=args.require_unity_model,
        require_build=args.require_build,
    )
    checks.extend(cmu_checks)

    lingtu, lingtu_checks = _check_lingtu_contract()
    checks.extend(lingtu_checks)

    blockers = [
        check["name"]
        for check in checks
        if check.get("required") is True and check.get("ok") is not True
    ]
    ok = not blockers
    return {
        "schema_version": SCHEMA_VERSION,
        "ok": ok,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "runtime_executed": False,
        "generated_at": time.time(),
        "host_tooling": host,
        "cmu_workspace": cmu,
        "lingtu_contract": lingtu,
        "checks": checks,
        "blockers": blockers,
        "next_commands": [
            (
                "git clone --branch humble --depth 1 "
                f"{CMU_REPO_URL} <cmu_workspace>"
            ),
            "download the Unity environment into src/base_autonomy/vehicle_simulator/mesh/unity/environment",
            "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release",
            (
                "launch CMU Unity + LingTu through the ordered wrapper: "
                "ROS_DOMAIN_ID=75 DISPLAY=:1 "
                "bash sim/scripts/launch_cmu_unity_lingtu_runtime.sh start --gate --rviz"
            ),
            "rerun sim/scripts/cmu_unity_sim_gate.py --strict",
        ],
    }


def _build_parser() -> argparse.ArgumentParser:
    default_workspace = os.environ.get("LINGTU_CMU_AUTONOMY_WS") or str(DEFAULT_CMU_WORKSPACE)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cmu-workspace", default=default_workspace)
    parser.add_argument("--json-out", type=Path, default=ROOT / "artifacts/server_sim_closure/cmu_unity_sim/report.json")
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--require-git", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--require-humble-branch", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--require-unity-model", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--require-build", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--require-ros", action=argparse.BooleanOptionalAction, default=True)
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    report = build_report(args)
    text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    args.json_out.parent.mkdir(parents=True, exist_ok=True)
    args.json_out.write_text(text + "\n", encoding="utf-8")
    print(text)
    if args.strict and not report["ok"]:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Summarize LingTu server-side simulation closure reports."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import platform
import shlex
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
for import_path in (ROOT, SRC):
    import_path_text = str(import_path)
    if import_path_text not in sys.path:
        sys.path.insert(0, import_path_text)

from runtime.algorithm_gates import DIMOS_BENCHMARK_REQUIRED_GATES
from runtime.algorithm_gates import G4_SERVER_FULL_SIM_REQUIRED_GATES
from runtime.algorithm_gates import INSPECTION_MVP_REQUIRED_GATES

MID360_PATTERN_REL = "sim/assets/livox/mid360.npy"
MID360_PATTERN_SHA256 = "448821576a658673e8f7929992c8c0d687eb052657d7b584d038729a83da1bfb"
MUJOCO_WORLD_ASSET_REL = "sim/worlds/mujoco/industrial_park_scene.xml"
GAZEBO_NAV_SOURCE_RELS = (
    "sim/planning/sim_navigation.launch.py",
    "sim/planning/gazebo_line_global_planner.py",
    "sim/planning/terrain_passthrough_node.py",
    "sim/scripts/gazebo_nav_loop_smoke.py",
    "sim/scripts/gazebo_frontier_exploration_smoke.py",
    "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/scripts/global_planner.py",
)
LIVE_NAV_MAP_TOPIC = "/slam/map_cloud"
DEFAULT_REQUIRED_MAX_REPORT_AGE_S = 24.0 * 60.0 * 60.0
DEFAULT_FRESHNESS_REQUIRED_GATES = frozenset(G4_SERVER_FULL_SIM_REQUIRED_GATES)
NAVIGATION_REPLAY_DEVIATION_REPORT_REL = "artifacts/server_sim_closure/navigation_replay_deviation/report.json"
NAVIGATION_REPLAY_DEVIATION_TRACE_REL = "artifacts/server_sim_closure/navigation_replay_deviation/trace.json"
MOVING_OBSTACLE_SWEEP_REPORT_REL = "artifacts/server_sim_closure/moving_obstacle_sweep/report.json"
LARGE_LOOP_INSPECTION_GOALS = "4.8,0.0;4.8,5.7;0.0,5.7;0.0,0.0"

LOCAL_NON_MOTION_HOST_REQUIREMENTS = (
    "local Python runtime only",
    "must preserve simulation_only=true, real_robot_motion=false, cmd_vel_sent_to_hardware=false",
)
LOCAL_NUMERIC_NAV_HOST_REQUIREMENTS = (
    *LOCAL_NON_MOTION_HOST_REQUIREMENTS,
    "stable NumPy/local planner runtime; Windows MINGW NumPy builds are not accepted for this gate",
)
LOCAL_NUMERIC_SIM_HOST_REQUIREMENTS = (
    *LOCAL_NON_MOTION_HOST_REQUIREMENTS,
    "stable NumPy/local simulation runtime; Windows MINGW NumPy builds are not accepted for this gate",
)
PRODUCT_LOCAL_PLANNER_BACKEND = "nanobind"
PRODUCT_PATH_FOLLOWER_BACKEND = "nav_kernel"
PRODUCT_GLOBAL_PLANNER_BACKEND = "octoplanner3d"
PCT_NATIVE_HOST_REQUIREMENTS = (
    "Linux host or S100P/aarch64 runtime with PCT native extension modules matching the host architecture",
    "Python ABI matching the built PCT modules; the current checked-good bundled host ABI is CPython 3.10",
    "PCT shared libraries available next to the extension modules or through LINGTU_PCT_LIB_DIR",
)
ROS2_MUJOCO_PCT_HOST_REQUIREMENTS = (
    *PCT_NATIVE_HOST_REQUIREMENTS,
    "ROS 2 Humble environment sourced",
    "MuJoCo EGL/headless rendering available",
    f"official MID-360 scan pattern asset available ({MID360_PATTERN_REL})",
    f"product MuJoCo world asset available ({MUJOCO_WORLD_ASSET_REL})",
    "isolated simulation domain with no physical robot drivers or hardware command publishers",
)
ROS2_LOCAL_PLANNER_HOST_REQUIREMENT = (
    "ROS 2 local_planner package available "
    "(ros2 run local_planner localPlanner/pathFollower)"
)
ROS2_PCT_ADAPTERS_HOST_REQUIREMENT = (
    "ROS 2 pct_adapters package available "
    "(ros2 run pct_adapters pct_path_adapter)"
)
ROS2_FASTLIO2_HOST_REQUIREMENT = (
    "ROS 2 fastlio2 package available (ros2 run fastlio2 lio_node)"
)
ROS2_MUJOCO_FASTLIO2_HOST_REQUIREMENTS = (
    *ROS2_MUJOCO_PCT_HOST_REQUIREMENTS,
    ROS2_FASTLIO2_HOST_REQUIREMENT,
)
ROS2_MUJOCO_PCT_LOCAL_PLANNER_HOST_REQUIREMENTS = (
    *ROS2_MUJOCO_PCT_HOST_REQUIREMENTS,
    ROS2_LOCAL_PLANNER_HOST_REQUIREMENT,
)
ROS2_MUJOCO_LOCALIZATION_HOST_REQUIREMENTS = (
    "ROS 2 Humble environment sourced",
    "MuJoCo/Fast-LIO live feed available in an isolated simulation domain",
    f"official MID-360 scan pattern asset available ({MID360_PATTERN_REL})",
    f"product MuJoCo world asset available ({MUJOCO_WORLD_ASSET_REL})",
    "localizer runtime available (ros2 run localizer localizer_node)",
    "generated saved-map assets available",
    "no physical robot drivers or hardware command publishers",
)
ROS2_GAZEBO_HOST_REQUIREMENTS = (
    "ROS 2 Humble environment sourced",
    "Gazebo/Ignition runtime available",
    "isolated ROS_DOMAIN_ID and Gazebo partition",
    "no physical robot drivers or hardware command publishers",
)
ROS2_GAZEBO_NAV_HOST_REQUIREMENTS = (
    *ROS2_GAZEBO_HOST_REQUIREMENTS,
    "Gazebo navigation source scripts available",
    ROS2_LOCAL_PLANNER_HOST_REQUIREMENT,
    ROS2_PCT_ADAPTERS_HOST_REQUIREMENT,
)

HARDWARE_COMMAND_TOPICS = (
    "/cmd_vel",
    "/nav/cmd_vel",
    "/s100p/cmd_vel",
    "/dog/cmd_vel",
)
HARDWARE_SUBSCRIBER_KEYWORDS = (
    "driver",
    "hardware",
    "s100p",
    "thunder",
    "nova",
    "dog",
)
SETUP_SERVER_ROS_PCT_COMMAND = "bash scripts/deploy/setup_server_ros_pct.sh"
SERVER_DIMOS_HOST_PREFLIGHT_COMMAND = (
    "PYTHONPATH=src:. python3 sim/scripts/server_sim_closure.py "
    "--preset dimos_benchmark --required-only --host-preflight "
    "--json-out artifacts/server_sim_closure/host_preflight_dimos_benchmark.json"
)
PCT_RUNTIME_PREFLIGHT_COMMAND = (
    "PYTHONPATH=src:. python3 sim/scripts/pct_runtime_preflight.py "
    "--json-out artifacts/server_sim_closure/pct_runtime_preflight/report.json"
)
PCT_RUNTIME_STRICT_PREFLIGHT_COMMAND = (
    "PYTHONPATH=src:. python3 sim/scripts/pct_runtime_preflight.py "
    "--strict --json-out artifacts/server_sim_closure/pct_runtime_preflight/report.json"
)
SAVED_MAP_RELOCALIZE_PREFLIGHT_COMMAND = (
    "PYTHONPATH=src:. python3 sim/scripts/saved_map_relocalize_runtime_gate.py "
    "--preflight-only "
    "--json-out artifacts/server_sim_closure/saved_map_relocalize_runtime_preflight/report.json"
)
HOST_CHECK_ACTIONS = {
    "local_non_motion": "local non-motion checks are already host-safe",
    "local_numeric_nav": (
        "rerun local numeric navigation gates on a Linux Python host with NumPy "
        "and the local planner runtime available"
    ),
    "pct_native": (
        "build/source the PCT native runtime on Linux with CPython 3.10, then "
        "rerun the DimOS host preflight"
    ),
    "ros2_humble": (
        "install/source ROS 2 Humble on the isolated simulation host, then "
        "rerun host preflight"
    ),
    "mujoco_headless": (
        "install the MuJoCo Python runtime and use EGL or OSMesa headless "
        "rendering before running MuJoCo gates"
    ),
    "mid360_pattern": (
        "sync the official MID-360 scan pattern asset before running MuJoCo "
        "LiDAR gates"
    ),
    "mujoco_world_asset": (
        "sync the product MuJoCo world asset before running MuJoCo simulation "
        "gates"
    ),
    "gazebo_runtime": (
        "install a Gazebo/Ignition runtime with gz or ign on PATH before "
        "running Gazebo gates"
    ),
    "gazebo_navigation_sources": (
        "sync the Gazebo navigation source scripts before running Gazebo "
        "navigation gates"
    ),
    "isolated_ros_domain": (
        "set ROS_DOMAIN_ID to a nonzero isolated simulation domain before "
        "launching ROS-backed simulation gates"
    ),
    "hardware_subscribers": (
        "prove hardware command topics have no physical robot subscribers on "
        "the isolated simulation domain"
    ),
    "ros2_local_planner": (
        "build/source the ROS 2 local_planner package so localPlanner and "
        "pathFollower are visible before running closed-loop MuJoCo gates"
    ),
    "ros2_pct_adapters": (
        "build/source the ROS 2 pct_adapters package so pct_path_adapter is "
        "visible before running Gazebo navigation gates"
    ),
    "ros2_fastlio2": (
        "build/source the Fast-LIO2 ROS 2 package so fastlio2/lio_node is "
        "visible before running live Fast-LIO MuJoCo gates"
    ),
    "localizer_runtime": (
        "build/source the localizer runtime on the ROS 2 simulation host before "
        "saved-map relocalization gates"
    ),
}
HOST_CHECK_PRIORITY = (
    "local_numeric_nav",
    "pct_native",
    "ros2_humble",
    "mujoco_headless",
    "mid360_pattern",
    "mujoco_world_asset",
    "gazebo_runtime",
    "gazebo_navigation_sources",
    "isolated_ros_domain",
    "hardware_subscribers",
    "ros2_fastlio2",
    "ros2_local_planner",
    "ros2_pct_adapters",
    "localizer_runtime",
)
HOST_CHECK_DIAGNOSTIC_COMMANDS = {
    "local_numeric_nav": (
        'python -c "import platform, numpy; print(platform.system(), numpy.__version__)"',
    ),
    "pct_native": (
        PCT_RUNTIME_PREFLIGHT_COMMAND,
        SETUP_SERVER_ROS_PCT_COMMAND,
        PCT_RUNTIME_STRICT_PREFLIGHT_COMMAND,
        SERVER_DIMOS_HOST_PREFLIGHT_COMMAND,
    ),
    "ros2_humble": (
        SETUP_SERVER_ROS_PCT_COMMAND,
        "source /opt/ros/humble/setup.bash",
        SERVER_DIMOS_HOST_PREFLIGHT_COMMAND,
    ),
    "mujoco_headless": (
        'python3 -c "import mujoco; print(mujoco.__version__)"',
        "export MUJOCO_GL=${MUJOCO_GL:-egl}",
        SERVER_DIMOS_HOST_PREFLIGHT_COMMAND,
    ),
    "mid360_pattern": (
        "ls -l sim/assets/livox/mid360.npy",
        (
            'python3 -c "from pathlib import Path; import hashlib; '
            "p=Path('sim/assets/livox/mid360.npy'); "
            "print(p.exists(), hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else '')\""
        ),
        SETUP_SERVER_ROS_PCT_COMMAND,
        SERVER_DIMOS_HOST_PREFLIGHT_COMMAND,
    ),
    "mujoco_world_asset": (
        "ls -l sim/worlds/mujoco/industrial_park_scene.xml",
        (
            'python3 -c "from pathlib import Path; '
            "p=Path('sim/worlds/mujoco/industrial_park_scene.xml'); "
            "print(p.exists(), p.stat().st_size if p.exists() else 0)\""
        ),
        SETUP_SERVER_ROS_PCT_COMMAND,
        SERVER_DIMOS_HOST_PREFLIGHT_COMMAND,
    ),
    "gazebo_runtime": (
        "gz sim --versions || ign gazebo --versions",
        SERVER_DIMOS_HOST_PREFLIGHT_COMMAND,
    ),
    "gazebo_navigation_sources": (
        "ls -l sim/planning/sim_navigation.launch.py sim/planning/gazebo_line_global_planner.py sim/planning/terrain_passthrough_node.py sim/scripts/gazebo_nav_loop_smoke.py sim/scripts/gazebo_frontier_exploration_smoke.py src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/scripts/global_planner.py",
        "python3 -m py_compile sim/planning/sim_navigation.launch.py sim/planning/gazebo_line_global_planner.py sim/planning/terrain_passthrough_node.py sim/scripts/gazebo_nav_loop_smoke.py sim/scripts/gazebo_frontier_exploration_smoke.py src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/scripts/global_planner.py",
        SERVER_DIMOS_HOST_PREFLIGHT_COMMAND,
    ),
    "isolated_ros_domain": (
        "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-75}",
        SERVER_DIMOS_HOST_PREFLIGHT_COMMAND,
    ),
    "hardware_subscribers": (
        'for topic in /cmd_vel /nav/cmd_vel /s100p/cmd_vel /dog/cmd_vel; do ros2 topic info "$topic" || true; done',
        SERVER_DIMOS_HOST_PREFLIGHT_COMMAND,
    ),
    "ros2_local_planner": (
        "source /opt/ros/humble/setup.bash && source install/setup.bash 2>/dev/null || true",
        "ros2 pkg executables local_planner",
        "ros2 run local_planner localPlanner --ros-args --help",
        "ros2 run local_planner pathFollower --ros-args --help",
        SERVER_DIMOS_HOST_PREFLIGHT_COMMAND,
    ),
    "ros2_pct_adapters": (
        "source /opt/ros/humble/setup.bash && source install/setup.bash 2>/dev/null || true",
        "ros2 pkg executables pct_adapters",
        "ros2 run pct_adapters pct_path_adapter --ros-args --help",
        SERVER_DIMOS_HOST_PREFLIGHT_COMMAND,
    ),
    "ros2_fastlio2": (
        "source /opt/ros/humble/setup.bash && source install/setup.bash 2>/dev/null || true",
        "ros2 pkg prefix fastlio2",
        "ros2 pkg executables fastlio2",
        SERVER_DIMOS_HOST_PREFLIGHT_COMMAND,
    ),
    "localizer_runtime": (
        SAVED_MAP_RELOCALIZE_PREFLIGHT_COMMAND,
        "source /opt/ros/humble/setup.bash && source install/setup.bash 2>/dev/null || true",
        "ros2 pkg prefix localizer",
        "ros2 pkg executables localizer",
        "ros2 run localizer localizer_node --ros-args --help",
        SETUP_SERVER_ROS_PCT_COMMAND,
        SAVED_MAP_RELOCALIZE_PREFLIGHT_COMMAND,
        SERVER_DIMOS_HOST_PREFLIGHT_COMMAND,
    ),
}


@dataclass(frozen=True)
class GateSpec:
    name: str
    description: str
    default_patterns: tuple[str, ...]
    command: str
    evaluator: Callable[[dict[str, Any]], tuple[bool, list[str], dict[str, Any]]]
    host_requirements: tuple[str, ...] = ()
    expected_report_path: str = ""


def _load_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(payload, dict):
        payload.setdefault("_report_path", str(path))
        return payload
    return {"_report_path": str(path), "value": payload}


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _bool_false(report: dict[str, Any], key: str) -> bool:
    return report.get(key) is False


def _safe_float(value: Any, default: float = 999.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return default


def _ensure_core_import_path() -> None:
    import sys

    for candidate in (ROOT / "src", ROOT):
        path = str(candidate)
        if path not in sys.path:
            sys.path.insert(0, path)


def _extend_unique(blockers: list[str], additions: list[str] | tuple[str, ...]) -> None:
    for item in additions:
        if item not in blockers:
            blockers.append(item)


def _shared_runtime_evidence(
    report: dict[str, Any],
    *,
    expected_contract: str,
    require_contract: bool = True,
    require_paths: bool = True,
    require_command: bool = True,
    require_frame_links: bool = False,
    require_data_flow: bool = False,
) -> tuple[list[str], dict[str, Any]]:
    has_contract = isinstance(report.get("runtime_contract"), dict)
    validation_required = (
        require_contract
        or require_paths
        or require_command
        or require_frame_links
        or require_data_flow
    )
    if not has_contract and not validation_required:
        return [], {
            "checked": False,
            "expected_contract": expected_contract,
            "required": False,
            "contract_required": require_contract,
            "frame_links_required": require_frame_links,
            "data_flow_required": require_data_flow,
            "reason": "runtime_contract not declared",
            "ok": True,
            "blockers": [],
        }

    try:
        _ensure_core_import_path()
        from runtime.diagnostics.runtime_evidence import validate_runtime_evidence

        result = validate_runtime_evidence(
            report,
            expected_contract,
            require_paths=require_paths,
            require_command=require_command,
            require_frame_links=require_frame_links,
            require_data_flow=require_data_flow,
        )
    except Exception as exc:
        blocker = f"runtime_evidence validation unavailable: {type(exc).__name__}: {exc}"
        return [blocker], {
            "checked": False,
            "expected_contract": expected_contract,
            "required": validation_required,
            "contract_required": require_contract,
            "frame_links_required": require_frame_links,
            "data_flow_required": require_data_flow,
            "ok": False,
            "blockers": [blocker],
        }

    blockers = list(result.blockers)
    return blockers, {
        "checked": True,
        "expected_contract": expected_contract,
        "required": validation_required,
        "contract_required": require_contract,
        "frame_links_required": require_frame_links,
        "data_flow_required": require_data_flow,
        "ok": result.ok,
        "blockers": blockers,
    }


def _require_live_nav_map_growth(map_growth: dict[str, Any], blockers: list[str]) -> None:
    source = str(map_growth.get("accepted_cumulative_growth_source") or "")
    if source != LIVE_NAV_MAP_TOPIC:
        blockers.append(f"map_growth.accepted_cumulative_growth_source is not {LIVE_NAV_MAP_TOPIC}")

    min_nav = _safe_float(map_growth.get("min_map_area_growth_m2"), default=0.0)
    if min_nav <= 0.0:
        blockers.append("map_growth.min_map_area_growth_m2 is not positive")

    if _safe_int(map_growth.get("nav_map_cloud_area_samples")) <= 0:
        blockers.append("map_growth.nav_map_cloud_area_samples missing")

    nav_area = map_growth.get("nav_map_cloud_xy_area") or {}
    if _safe_float(nav_area.get("growth_m2"), default=0.0) < min_nav:
        blockers.append(f"{LIVE_NAV_MAP_TOPIC} area growth below threshold")


def _require_exploration_grid_signal(
    explorer: dict[str, Any],
    blockers: list[str],
    *,
    prefix: str,
) -> None:
    if _safe_int(explorer.get("exploration_grid_samples")) <= 0:
        blockers.append(f"{prefix}.exploration_grid_samples missing")

    metadata = (
        explorer.get("exploration_grid_metadata_last")
        if isinstance(explorer.get("exploration_grid_metadata_last"), dict)
        else {}
    )
    if str(metadata.get("source") or "") != "raycast_lidar_exploration_grid":
        blockers.append(f"{prefix}.exploration_grid source is not raycast_lidar_exploration_grid")

    for grid_name in ("exploration_grid_first", "exploration_grid_last"):
        counts = explorer.get(grid_name) if isinstance(explorer.get(grid_name), dict) else {}
        for cell_name in ("free", "unknown"):
            if _safe_int(counts.get(cell_name)) <= 0:
                blockers.append(f"{prefix}.{grid_name}.{cell_name} missing")


def _require_exploration_progress(map_growth: dict[str, Any], blockers: list[str]) -> None:
    if _safe_int(map_growth.get("exploration_area_samples")) <= 0:
        blockers.append("map_growth.exploration_area_samples missing")

    min_explored = _safe_float(map_growth.get("min_explored_area_growth_m2"), default=0.0)
    if min_explored <= 0.0:
        blockers.append("map_growth.min_explored_area_growth_m2 is not positive")

    min_coverage = _safe_float(
        map_growth.get("min_exploration_coverage_growth_ratio"),
        default=0.0,
    )
    if min_coverage <= 0.0:
        blockers.append("map_growth.min_exploration_coverage_growth_ratio is not positive")

    explored_area = map_growth.get("exploration_known_area") or {}
    if _safe_float(explored_area.get("growth_m2"), default=0.0) < min_explored:
        blockers.append("exploration known area growth below threshold")

    coverage = map_growth.get("exploration_coverage") or {}
    if _safe_float(coverage.get("growth_ratio"), default=0.0) < min_coverage:
        blockers.append("exploration coverage growth below threshold")


def _moving_obstacle_trail_margin(trail: dict[str, Any]) -> Any:
    margin = trail.get("min_clearance_minus_robot_radius_m")
    if margin is None:
        margin = trail.get("min_margin_m")
    return margin


def _moving_obstacles_enabled(moving_obstacles: dict[str, Any]) -> bool:
    mode = str(moving_obstacles.get("mode") or "none")
    return bool(moving_obstacles.get("enabled")) or mode != "none"


def _require_moving_obstacle_evidence(
    moving_obstacles: dict[str, Any],
    blockers: list[str],
    *,
    prefix: str = "moving_obstacles",
) -> dict[str, Any]:
    trail = (
        moving_obstacles.get("trail_clearance")
        if isinstance(moving_obstacles.get("trail_clearance"), dict)
        else {}
    )
    margin = _moving_obstacle_trail_margin(trail)
    if moving_obstacles.get("ok") is not True:
        blockers.append(f"{prefix}.ok is not true")
    if _safe_int(moving_obstacles.get("published_update_count")) <= 0:
        blockers.append(f"{prefix}.published_update_count missing")
    if _safe_int(moving_obstacles.get("published_point_count_max")) <= 0:
        blockers.append(f"{prefix}.published_point_count_max missing")
    if trail.get("checked") is not True:
        blockers.append(f"{prefix}.trail_clearance.checked is not true")
    if trail.get("collision") is True:
        blockers.append(f"{prefix}.trail_clearance.collision is true")
    if _safe_float(margin, default=-999.0) < 0.0:
        blockers.append(f"{prefix}.trail_clearance margin is negative")
    return {
        "enabled": moving_obstacles.get("enabled"),
        "mode": str(moving_obstacles.get("mode") or "none"),
        "count": moving_obstacles.get("count"),
        "period_s": moving_obstacles.get("period_s"),
        "ok": moving_obstacles.get("ok"),
        "published_update_count": moving_obstacles.get("published_update_count"),
        "published_point_count_max": moving_obstacles.get("published_point_count_max"),
        "trail_collision": trail.get("collision"),
        "trail_min_margin_m": margin,
    }


def _fastlio2_live_video_evidence(report: dict[str, Any]) -> dict[str, Any]:
    video = report.get("video") if isinstance(report.get("video"), dict) else {}
    path = str(report.get("video_path") or video.get("path") or "")
    frame_count = _safe_int(report.get("video_frame_count", video.get("frames")))
    sample_count = _safe_int(report.get("video_sample_count", video.get("samples")))
    return {
        "required": bool(path),
        "path": path,
        "frame_count": frame_count,
        "sample_count": sample_count,
    }


def _require_fastlio2_live_video_evidence(
    report: dict[str, Any],
    blockers: list[str],
    *,
    prefix: str = "fastlio2_live",
) -> dict[str, Any]:
    evidence = _fastlio2_live_video_evidence(report)
    if evidence["required"]:
        if _safe_int(evidence.get("frame_count")) <= 0:
            blockers.append(f"{prefix} video_frame_count missing")
        if _safe_int(evidence.get("sample_count")) <= 0:
            blockers.append(f"{prefix} video_sample_count missing")
    return evidence


def _require_same_source_map_artifacts(
    payload: dict[str, Any],
    blockers: list[str],
    *,
    prefix: str,
    require_tomogram: bool = True,
) -> dict[str, Any]:
    map_artifacts = (
        payload.get("map_artifacts")
        if isinstance(payload.get("map_artifacts"), dict)
        else {}
    )
    source_contract = (
        map_artifacts.get("source_contract")
        if isinstance(map_artifacts.get("source_contract"), dict)
        else {}
    )
    assets = (
        map_artifacts.get("assets")
        if isinstance(map_artifacts.get("assets"), dict)
        else {}
    )
    map_pcd = assets.get("map_pcd") if isinstance(assets.get("map_pcd"), dict) else {}
    tomogram = assets.get("tomogram") if isinstance(assets.get("tomogram"), dict) else {}
    map_sha = str(map_pcd.get("sha256") or "")
    map_point_count = _safe_int(map_pcd.get("point_count"))
    tomogram_sha = str(tomogram.get("sha256") or "")
    tomogram_source_sha = str(
        tomogram.get("source_map_sha256")
        or tomogram.get("input_pcd_sha256")
        or ""
    )
    tomogram_expected = bool(
        require_tomogram
        or tomogram
        or source_contract.get("same_source_tomogram") is not None
    )

    local_blockers: list[str] = []
    if map_artifacts.get("ok") is not True:
        local_blockers.append(f"{prefix}.map_artifacts.ok is not true")
    if source_contract.get("same_source_pcd") is not True:
        local_blockers.append(f"{prefix}.same_source_pcd is not true")
    if not map_sha:
        local_blockers.append(f"{prefix}.map_pcd.sha256 missing")
    if map_point_count <= 0:
        local_blockers.append(f"{prefix}.map_pcd.point_count missing")
    if tomogram_expected:
        if source_contract.get("same_source_tomogram") is not True:
            local_blockers.append(f"{prefix}.same_source_tomogram is not true")
        if not tomogram_sha:
            local_blockers.append(f"{prefix}.tomogram.sha256 missing")
        if not tomogram_source_sha:
            local_blockers.append(f"{prefix}.tomogram.source_map_sha256 missing")
        if map_sha and tomogram_source_sha and tomogram_source_sha != map_sha:
            local_blockers.append(
                f"{prefix}.tomogram.source_map_sha256 does not match map_pcd.sha256"
            )
    blockers.extend(local_blockers)
    return {
        "ok": not local_blockers,
        "map_artifacts_ok": map_artifacts.get("ok"),
        "same_source_pcd": source_contract.get("same_source_pcd"),
        "same_source_tomogram": source_contract.get("same_source_tomogram"),
        "map_pcd_sha256": map_sha,
        "map_pcd_point_count": map_point_count,
        "tomogram_sha256": tomogram_sha,
        "tomogram_source_map_sha256": tomogram_source_sha,
        "blockers": local_blockers,
    }


def _require_same_source_hash_identity(
    payload: dict[str, Any],
    blockers: list[str],
    *,
    prefix: str,
) -> dict[str, Any]:
    identity = (
        payload.get("same_source_hash_identity")
        if isinstance(payload.get("same_source_hash_identity"), dict)
        else {}
    )
    checks = identity.get("checks") if isinstance(identity.get("checks"), dict) else {}
    local_blockers: list[str] = []
    if identity.get("ok") is not True:
        local_blockers.append(f"{prefix}.same_source_hash_identity.ok is not true")
    if not checks:
        local_blockers.append(f"{prefix}.same_source_hash_identity.checks missing")
    for name, ok in checks.items():
        if ok is not True:
            local_blockers.append(
                f"{prefix}.same_source_hash_identity.{name} is not true"
            )
    blockers.extend(local_blockers)
    return {
        "ok": not local_blockers,
        "checks": checks,
        "hashes": identity.get("hashes") if isinstance(identity.get("hashes"), dict) else {},
        "blockers": local_blockers,
    }


def _resolve_report_relative_path(report: dict[str, Any], path_value: Any) -> Path:
    path = Path(str(path_value or ""))
    if path.is_absolute():
        return path
    report_path = report.get("_report_path")
    if report_path:
        report_dir = Path(str(report_path)).resolve().parent
        candidate = (report_dir / path).resolve()
        if candidate.exists():
            return candidate
    return (ROOT / path).resolve()


@lru_cache(maxsize=16)
def _module_import_is_safe(module_name: str) -> bool:
    if importlib.util.find_spec(module_name) is None:
        return False
    try:
        probe = subprocess.run(
            [sys.executable, "-c", f"import {module_name}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=10,
        )
    except Exception:
        return False
    return probe.returncode == 0


def _verify_video_artifact(
    report: dict[str, Any],
    video: dict[str, Any],
    blockers: list[str],
    *,
    prefix: str,
) -> dict[str, Any]:
    path_text = str(video.get("path") or "")
    evidence = {
        "path": path_text,
        "frames": _safe_int(video.get("frames", video.get("frame_count"))),
        "samples": _safe_int(video.get("samples", video.get("sample_count"))),
        "exists": False,
        "decode_checked": False,
        "decode_ok": False,
        "nonblack_checked": False,
        "nonblack_ok": False,
    }
    if not path_text:
        return evidence

    path = _resolve_report_relative_path(report, path_text)
    evidence["resolved_path"] = str(path)
    if not path.exists():
        blockers.append(f"{prefix} video file missing")
        return evidence
    evidence["exists"] = True
    evidence["size_bytes"] = path.stat().st_size
    if path.stat().st_size <= 0:
        blockers.append(f"{prefix} video file is empty")
        return evidence

    if not _module_import_is_safe("cv2"):
        evidence["decode_error"] = "cv2 import unavailable or unsafe"
        blockers.append(f"{prefix} video decoder unavailable")
        return evidence

    try:
        import cv2  # type: ignore
    except Exception as exc:
        evidence["decode_error"] = f"{type(exc).__name__}: {exc}"
        blockers.append(f"{prefix} video decoder unavailable")
        return evidence

    capture = cv2.VideoCapture(str(path))
    try:
        evidence["decode_checked"] = True
        ok, frame = capture.read()
        if not ok or frame is None:
            blockers.append(f"{prefix} video first frame is not decodable")
            return evidence
        evidence["decode_ok"] = True
        evidence["first_frame_shape"] = list(frame.shape)
        if not _module_import_is_safe("numpy"):
            evidence["nonblack_error"] = "numpy import unavailable or unsafe"
            blockers.append(f"{prefix} video nonblack validation unavailable")
            return evidence
        try:
            import numpy as np

            mean_intensity = float(np.mean(frame))
            evidence["first_frame_mean_intensity"] = round(mean_intensity, 3)
            evidence["nonblack_checked"] = True
            if mean_intensity <= 1.0:
                blockers.append(f"{prefix} video first frame is black")
            else:
                evidence["nonblack_ok"] = True
        except Exception as exc:
            evidence["nonblack_error"] = f"{type(exc).__name__}: {exc}"
            blockers.append(f"{prefix} video nonblack validation unavailable")
    finally:
        capture.release()
    return evidence


def _verify_file_artifact(
    report: dict[str, Any],
    path_value: Any,
    blockers: list[str],
    *,
    prefix: str,
) -> dict[str, Any]:
    path_text = str(path_value or "").strip()
    evidence: dict[str, Any] = {
        "path": path_text,
        "exists": False,
        "is_file": False,
        "size_bytes": 0,
    }
    if not path_text:
        blockers.append(f"{prefix} path missing")
        return evidence

    path = _resolve_report_relative_path(report, path_text)
    evidence["resolved_path"] = str(path)
    if not path.exists():
        blockers.append(f"{prefix} file missing")
        return evidence
    evidence["exists"] = True
    if not path.is_file():
        blockers.append(f"{prefix} is not a file")
        return evidence
    evidence["is_file"] = True
    size_bytes = path.stat().st_size
    evidence["size_bytes"] = size_bytes
    if size_bytes <= 0:
        blockers.append(f"{prefix} file is empty")
    return evidence


def _canonical_path_key(path: Path) -> str:
    return os.path.normcase(str(path.resolve()))


def _same_source_saved_map_binding(
    report: dict[str, Any],
    blockers: list[str],
) -> dict[str, Any]:
    evidence: dict[str, Any] = {
        "ok": False,
        "relocalize_report": str(report.get("relocalize_report") or ""),
        "tomogram": str(report.get("tomogram") or ""),
    }
    relocalize_text = str(report.get("relocalize_report") or "").strip()
    tomogram_text = str(report.get("tomogram") or "").strip()
    if not relocalize_text or not tomogram_text:
        return evidence

    relocalize_path = _resolve_report_relative_path(report, relocalize_text)
    tomogram_path = _resolve_report_relative_path(report, tomogram_text)
    evidence["relocalize_report_resolved_path"] = str(relocalize_path)
    evidence["tomogram_resolved_path"] = str(tomogram_path)
    if not relocalize_path.is_file():
        return evidence

    try:
        relocalize_payload = json.loads(relocalize_path.read_text(encoding="utf-8"))
    except Exception as exc:
        blockers.append(
            f"pct_saved_map_navigation relocalize_report JSON unreadable: {type(exc).__name__}"
        )
        return evidence
    if not isinstance(relocalize_payload, dict):
        blockers.append("pct_saved_map_navigation relocalize_report is not a JSON object")
        return evidence

    relocalize_for_resolution = dict(relocalize_payload)
    relocalize_for_resolution["_report_path"] = str(relocalize_path)
    map_pcd_text = str(relocalize_payload.get("map_pcd") or "").strip()
    map_pcd_artifact = _verify_file_artifact(
        relocalize_for_resolution,
        map_pcd_text,
        blockers,
        prefix="pct_saved_map_navigation relocalize_report.map_pcd",
    )
    evidence["map_pcd"] = map_pcd_artifact
    if not map_pcd_text:
        return evidence

    map_pcd_path = _resolve_report_relative_path(relocalize_for_resolution, map_pcd_text)
    expected_tomogram = map_pcd_path.parent / "tomogram.pickle"
    evidence["expected_tomogram"] = str(expected_tomogram)
    if _canonical_path_key(tomogram_path) != _canonical_path_key(expected_tomogram):
        blockers.append(
            "pct_saved_map_navigation tomogram is not sibling of "
            "relocalize_report.map_pcd"
        )
        return evidence
    evidence["ok"] = True
    return evidence


def _runtime_contract_blockers(
    report: dict[str, Any],
    *,
    expected_name: str,
) -> tuple[list[str], dict[str, Any]]:
    contract = report.get("runtime_contract")
    if not isinstance(contract, dict):
        return ["runtime_contract missing"], {}
    blockers: list[str] = []
    if contract.get("name") != expected_name:
        blockers.append(f"runtime_contract.name is not {expected_name}")
    if contract.get("ok") is not True:
        blockers.append("runtime_contract.ok is not true")
    definition = contract.get("definition") if isinstance(contract.get("definition"), dict) else {}

    canonical: dict[str, Any] = {}
    try:
        _ensure_core_import_path()
        from runtime.blueprints.simulation_contract import simulation_runtime_contract

        canonical = simulation_runtime_contract(expected_name).as_report()
    except Exception as exc:
        blockers.append(f"runtime_contract canonical definition unavailable: {type(exc).__name__}: {exc}")

    if definition.get("name") != expected_name:
        blockers.append(f"runtime_contract.definition.name is not {expected_name}")
    if definition.get("simulation_only") is not True:
        blockers.append("runtime_contract.definition.simulation_only is not true")
    if definition.get("isolated_domain_required") is not True:
        blockers.append("runtime_contract.definition.isolated_domain_required is not true")
    for key in (
        "provider",
        "profile",
        "world",
        "launch_script",
        "rviz_config",
        "adapter_script",
        "data_source_contract",
        "command_topic",
        "simulation_only",
        "isolated_domain_required",
        "contract_role",
        "runtime_stage",
        "map_dependency",
        "world_sensor_owner",
        "slam_source",
        "localization_source",
        "mapping_source",
        "slam_validated",
        "requires_live_slam",
        "requires_saved_map",
        "requires_tomogram",
    ):
        if canonical and definition.get(key) != canonical.get(key):
            blockers.append(f"runtime_contract.definition.{key} does not match canonical contract")
    for key in (
        "canonical_topics",
        "native_topics",
        "lingtu_owns",
        "simulator_owns",
        "required_runtime_topics",
        "required_path_topics",
        "required_map_growth_topics",
        "required_scan_topics",
        "required_slam_topics",
    ):
        if canonical and list(definition.get(key) or []) != list(canonical.get(key) or []):
            blockers.append(f"runtime_contract.definition.{key} does not match canonical contract")

    topic_evidence = contract.get("topic_evidence") if isinstance(contract.get("topic_evidence"), dict) else {}
    for topic in list((canonical or definition).get("required_runtime_topics") or []):
        if (topic_evidence.get(topic) or {}).get("ok") is not True:
            blockers.append(f"runtime_contract topic evidence missing or failed for {topic}")

    for topic in list((canonical or definition).get("required_path_topics") or []):
        requirement = (report.get("path_requirements") or {}).get(topic)
        evidence = topic_evidence.get(topic) or {}
        if not (isinstance(requirement, dict) and requirement.get("ok") is True) and evidence.get("ok") is not True:
            blockers.append(f"runtime_contract path evidence missing or failed for {topic}")
    for topic in list((canonical or definition).get("required_map_growth_topics") or []):
        requirement = (report.get("map_requirements") or {}).get(topic)
        evidence = topic_evidence.get(topic) or {}
        if not (isinstance(requirement, dict) and requirement.get("ok") is True) and evidence.get("ok") is not True:
            blockers.append(f"runtime_contract map-growth evidence missing or failed for {topic}")
    for topic in list((canonical or definition).get("required_scan_topics") or []):
        requirement = (report.get("scan_requirements") or {}).get(topic)
        evidence = topic_evidence.get(topic) or {}
        if not (isinstance(requirement, dict) and requirement.get("ok") is True) and evidence.get("ok") is not True:
            blockers.append(f"runtime_contract scan evidence missing or failed for {topic}")
    for topic in list((canonical or definition).get("required_slam_topics") or []):
        evidence = topic_evidence.get(topic) or {}
        if evidence.get("ok") is not True:
            blockers.append(f"runtime_contract slam evidence missing or failed for {topic}")

    publisher_identity = (
        contract.get("publisher_identity")
        if isinstance(contract.get("publisher_identity"), dict)
        else {}
    )
    if publisher_identity.get("blocked_hardware_nodes"):
        blockers.append("runtime_contract publisher_identity has hardware-looking endpoints")
    if publisher_identity.get("unexpected_command_publishers"):
        blockers.append("runtime_contract publisher_identity has unexpected command publishers")
    for error in contract.get("errors") or []:
        blockers.append(f"runtime_contract: {error}")
    return blockers, contract


def _frame(report: dict[str, Any], key: str) -> str:
    frames = report.get("frames") or {}
    return str(frames.get(key) or "")


def _lidar_source(report: dict[str, Any]) -> dict[str, Any]:
    source = report.get("lidar_source")
    if isinstance(source, dict):
        return source
    video = report.get("video")
    if isinstance(video, dict) and isinstance(video.get("lidar_source"), dict):
        return video["lidar_source"]
    return {}


def _require_mid360_pattern(report: dict[str, Any], blockers: list[str], prefix: str) -> dict[str, Any]:
    source = _lidar_source(report)
    if source.get("forced_pattern") is not True:
        blockers.append(f"{prefix}.forced_pattern is not true")
    if str(source.get("pattern_sha256") or "").lower() != MID360_PATTERN_SHA256:
        blockers.append(f"{prefix}.pattern_sha256 is not official MID-360 asset")
    pattern_path = str(source.get("pattern_path") or "").replace("\\", "/")
    if not pattern_path.endswith(MID360_PATTERN_REL):
        blockers.append(f"{prefix}.pattern_path is not {MID360_PATTERN_REL}")
    if int(source.get("samples_per_frame") or 0) <= 0:
        blockers.append(f"{prefix}.samples_per_frame missing")
    return source


def _extract_algorithm_backends(report: dict[str, Any]) -> dict[str, Any]:
    backends = report.get("algorithm_backends")
    if isinstance(backends, dict) and backends:
        return backends

    for key in ("command_flow", "tracking_replay", "native_gate", "native_pct_gate"):
        nested = report.get(key)
        if isinstance(nested, dict):
            nested_backends = _extract_algorithm_backends(nested)
            if nested_backends:
                return nested_backends

    combined: dict[str, Any] = {}
    for case in report.get("cases") or []:
        if not isinstance(case, dict):
            continue
        case_backends = _extract_algorithm_backends(case)
        for surface, evidence in case_backends.items():
            combined.setdefault(str(surface), evidence)
    return combined


def _algorithm_backends_from_gates(gates: dict[str, Any]) -> dict[str, Any]:
    summary: dict[str, Any] = {}
    for name in _ordered_gate_names(set(gates)):
        gate = gates.get(name) or {}
        evidence = gate.get("evidence") or {}
        if not isinstance(evidence, dict):
            continue
        backends = evidence.get("algorithm_backends")
        if isinstance(backends, dict) and backends:
            summary[name] = backends
    return summary


def _eval_large_terrain(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("ok") is not True:
        blockers.append("report.ok is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")

    cases = list(report.get("cases") or [])
    pct_plans = [
        plan
        for case in cases
        for plan in case.get("planning", [])
        if str(plan.get("planner")).lower() == "pct"
    ]
    native_pct = any(
        bool(_first_present(plan.get("native_runtime_used"), plan.get("native_backend_used")))
        for plan in pct_plans
    )
    path_safe = all(bool((case.get("path_safety") or {}).get("ok", True)) for case in cases)
    if not pct_plans:
        blockers.append("no PCT planning case found")
    if not native_pct:
        blockers.append("no native PCT case found")
    if not path_safe:
        blockers.append("path safety failed")
    for blocker in report.get("environment_blockers") or []:
        if str(blocker) not in blockers:
            blockers.append(str(blocker))

    return not blockers, blockers, {
        "case_count": len(cases),
        "execution_mode": str(report.get("execution_mode") or ""),
        "native_pct": native_pct,
        "native_runtime": report.get("native_runtime") or {},
        "environment": report.get("environment") or {},
        "environment_blockers": report.get("environment_blockers") or [],
        "errors": report.get("errors") or [],
        "path_safe": path_safe,
        "routes": [case.get("route") for case in cases],
        "algorithm_backends": _extract_algorithm_backends(report),
    }


def _first_present(*values: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return None


def _pct_optimizer_evidence(
    report: dict[str, Any],
    source_contract: dict[str, Any],
) -> dict[str, Any]:
    return {
        "enabled": _first_present(
            report.get("pct_optimizer_enabled"),
            source_contract.get("pct_optimizer_enabled"),
        ),
        "attempted": _first_present(
            report.get("pct_optimizer_attempted"),
            source_contract.get("pct_optimizer_attempted"),
        ),
        "accepted": _first_present(
            report.get("pct_optimizer_accepted"),
            source_contract.get("pct_optimizer_accepted"),
        ),
        "reject_reason": str(
            _first_present(
                report.get("pct_optimizer_reject_reason"),
                source_contract.get("pct_optimizer_reject_reason"),
            )
            or ""
        ),
        "blocked_sample_count": _safe_int(
            _first_present(
                report.get("pct_optimizer_blocked_sample_count"),
                source_contract.get("pct_optimizer_blocked_sample_count"),
            )
        ),
        "path_mode": str(
            _first_present(
                report.get("pct_planner_path_mode"),
                source_contract.get("pct_planner_path_mode"),
            )
            or ""
        ),
    }


def _require_pct_optimizer_contract(
    report: dict[str, Any],
    source_contract: dict[str, Any],
    blockers: list[str],
) -> dict[str, Any]:
    evidence = _pct_optimizer_evidence(report, source_contract)
    enabled = evidence["enabled"]
    attempted = evidence["attempted"]
    accepted = evidence["accepted"]
    reject_reason = evidence["reject_reason"]
    path_mode = evidence["path_mode"]
    rejection_recorded = (
        attempted is True
        and accepted is False
        and bool(reject_reason)
    )

    if enabled not in (True, False):
        blockers.append("pct_optimizer_enabled is not recorded")
    if path_mode not in {"native_astar_raw_path", "optimized_trajectory"}:
        blockers.append("pct_planner_path_mode is not supported")
    elif enabled is False and path_mode != "native_astar_raw_path":
        blockers.append("pct optimizer disabled but path mode is not native_astar_raw_path")
    elif enabled is True and path_mode == "optimized_trajectory" and accepted is False:
        blockers.append("optimized trajectory path mode is marked rejected")
    elif enabled is True and path_mode == "native_astar_raw_path" and not rejection_recorded:
        blockers.append(
            "native raw path mode lacks recorded optimizer rejection"
        )

    evidence["optimizer_rejection_recorded"] = rejection_recorded
    return evidence


def _eval_native_pct_mujoco(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    claim_boundary = str(report.get("claim_boundary") or "")
    waypoint_filter = report.get("waypoint_safety_filter") or {}
    waypoint_goal_blocked = (
        report.get("native_gate_skipped") is True
        and claim_boundary == "waypoint_safety_filter_truncated_pct_path"
    )
    ros2_runtime_blocked = (
        report.get("native_gate_skipped") is True
        and claim_boundary == "ros2_runtime_unavailable"
    )
    if report.get("ok") is not True and not waypoint_goal_blocked and not ros2_runtime_blocked:
        blockers.append("report.ok is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")
    if (
        report.get("reached_goal") is not True
        and not waypoint_goal_blocked
        and not ros2_runtime_blocked
    ):
        blockers.append("reached_goal is not true")
    if str(report.get("planner") or "").lower() != "pct":
        blockers.append("planner is not pct")
    pct_native_runtime_used = _first_present(
        report.get("pct_native_runtime_used"),
        report.get("pct_native_backend_used"),
    )
    if pct_native_runtime_used is not True:
        blockers.append("pct_native_runtime_used is not true")
    if report.get("pct_runtime_ok") is not True:
        blockers.append("pct_runtime_ok is not true")
    if _safe_int(report.get("pct_path_count")) < 2:
        blockers.append("pct_path_count < 2")
    if str(report.get("primary_planner") or "").lower() != "pct":
        blockers.append("primary_planner is not pct")
    if str(report.get("selected_planner") or "").lower() != "pct":
        blockers.append("selected_planner is not pct")
    if report.get("fallback_used") is not False:
        blockers.append("fallback_used is not false")
    if str(report.get("global_planner_source") or "") != "source_report/native_pct_tomogram":
        blockers.append("global_planner_source is not source_report/native_pct_tomogram")
    chain = report.get("planning_chain") or {}
    if chain.get("fallback_allowed") is not False:
        blockers.append("planning_chain.fallback_allowed is not false")
    if str(chain.get("local_planner") or "") != "cmu_ros2_native/localPlanner":
        blockers.append("planning_chain.local_planner is not cmu_ros2_native/localPlanner")
    if str(chain.get("path_follower") or "") != "cmu_ros2_native/pathFollower":
        blockers.append("planning_chain.path_follower is not cmu_ros2_native/pathFollower")
    source_contract = report.get("source_planning_contract") or {}
    if str(source_contract.get("primary_planner") or "").lower() != "pct":
        blockers.append("source_planning_contract.primary_planner is not pct")
    if str(source_contract.get("selected_planner") or "").lower() != "pct":
        blockers.append("source_planning_contract.selected_planner is not pct")
    if source_contract.get("fallback_used") is not False:
        blockers.append("source_planning_contract.fallback_used is not false")
    if source_contract.get("path_safety_ok") is not True:
        blockers.append("source_planning_contract.path_safety_ok is not true")
    source_native_runtime_used = _first_present(
        source_contract.get("native_runtime_used"),
        source_contract.get("native_backend_used"),
    )
    if source_native_runtime_used is not True:
        blockers.append("source_planning_contract.native_runtime_used is not true")
    if source_contract.get("tomogram_exists") is not True:
        blockers.append("source_planning_contract.tomogram_exists is not true")
    if not str(source_contract.get("tomogram_sha256") or ""):
        blockers.append("source_planning_contract.tomogram_sha256 is missing")
    pct_optimizer = _require_pct_optimizer_contract(report, source_contract, blockers)
    same_source_artifacts = _require_same_source_map_artifacts(
        report,
        blockers,
        prefix="native_pct_mujoco",
    )
    if ros2_runtime_blocked:
        for blocker in report.get("blockers") or []:
            text = str(blocker)
            if text not in blockers:
                blockers.append(text)
        return False, blockers, {
            "planner": report.get("planner"),
            "primary_planner": report.get("primary_planner"),
            "selected_planner": report.get("selected_planner"),
            "fallback_used": report.get("fallback_used"),
            "global_planner_source": report.get("global_planner_source"),
            "source_planning_contract": source_contract,
            "pct_native_runtime_used": pct_native_runtime_used,
            "pct_runtime_ok": report.get("pct_runtime_ok"),
            "pct_path_count": report.get("pct_path_count"),
            "pct_optimizer": pct_optimizer,
            "same_source_artifacts": same_source_artifacts,
            "claim_boundary": claim_boundary,
            "native_gate_skipped": report.get("native_gate_skipped"),
            "environment": report.get("environment") or {},
            "command_generation": report.get("command_generation") or {},
            "native_node_commands": report.get("native_node_commands") or {},
            "waypoint_safety_filter": {
                "enabled": waypoint_filter.get("enabled"),
                "goal_preserved": waypoint_filter.get("goal_preserved"),
                "source_path_goal_xy": waypoint_filter.get("source_path_goal_xy"),
                "follow_path_goal_xy": waypoint_filter.get("follow_path_goal_xy"),
                "goal_shift_m": waypoint_filter.get("goal_shift_m"),
            },
            "requested_goal_xy": report.get("requested_goal_xy"),
            "pct_path_goal_xy": report.get("pct_path_goal_xy"),
            "tracking_goal_xy": report.get("tracking_goal_xy"),
            "frames": report.get("frames") or {},
        }
    if waypoint_goal_blocked:
        if waypoint_filter.get("enabled") is not True:
            blockers.append("waypoint_safety_filter.enabled is not true")
        if waypoint_filter.get("goal_preserved") is not False:
            blockers.append("waypoint_safety_filter.goal_preserved is not false")
        else:
            blockers.append("waypoint_safety_filter.goal_preserved is not true")
        return False, blockers, {
            "planner": report.get("planner"),
            "primary_planner": report.get("primary_planner"),
            "selected_planner": report.get("selected_planner"),
            "fallback_used": report.get("fallback_used"),
            "global_planner_source": report.get("global_planner_source"),
            "source_planning_contract": source_contract,
            "pct_native_runtime_used": pct_native_runtime_used,
            "same_source_artifacts": same_source_artifacts,
            "claim_boundary": claim_boundary,
            "native_gate_skipped": report.get("native_gate_skipped"),
            "waypoint_safety_filter": {
                "enabled": waypoint_filter.get("enabled"),
                "goal_preserved": waypoint_filter.get("goal_preserved"),
                "source_path_goal_xy": waypoint_filter.get("source_path_goal_xy"),
                "follow_path_goal_xy": waypoint_filter.get("follow_path_goal_xy"),
                "goal_shift_m": waypoint_filter.get("goal_shift_m"),
                "original_count": waypoint_filter.get("original_count"),
                "path_count": waypoint_filter.get("path_count"),
                "skipped_unsafe_waypoint_count": waypoint_filter.get("skipped_unsafe_waypoint_count"),
            },
            "requested_goal_xy": report.get("requested_goal_xy"),
            "pct_path_goal_xy": report.get("pct_path_goal_xy"),
            "tracking_goal_xy": report.get("tracking_goal_xy"),
            "obstacle_aware": report.get("obstacle_aware") or {},
            "frames": report.get("frames") or {},
        }
    obstacle_aware = report.get("obstacle_aware") or {}
    if obstacle_aware.get("enabled") is not True:
        blockers.append("obstacle_aware.enabled is not true")
    if int(obstacle_aware.get("metadata_points") or 0) <= 0:
        blockers.append("obstacle_aware.metadata_points missing")
    if waypoint_filter.get("enabled") is True and waypoint_filter.get("goal_preserved") is not True:
        blockers.append("waypoint_safety_filter.goal_preserved is not true")
    clearance = report.get("obstacle_clearance") or {}
    if clearance.get("checked") is not True:
        blockers.append("obstacle clearance was not checked")
    if clearance.get("collision") is True:
        blockers.append("collision is true")
    local_path_samples = report.get("local_path_samples") or []
    if not local_path_samples:
        blockers.append("local_path_samples missing")
    trajectory_quality = report.get("trajectory_quality") or {}
    if trajectory_quality.get("ok") is not True:
        blockers.append("trajectory_quality.ok is not true")
    moving_obstacles = report.get("moving_obstacles") or {}
    moving_mode = str(moving_obstacles.get("mode") or "none")
    moving_enabled = bool(moving_obstacles.get("enabled")) or moving_mode != "none"
    moving_trail = moving_obstacles.get("trail_clearance") or {}
    moving_trail_margin = moving_trail.get("min_clearance_minus_robot_radius_m")
    if moving_trail_margin is None:
        moving_trail_margin = moving_trail.get("min_margin_m")
    if moving_enabled:
        if moving_obstacles.get("ok") is not True:
            blockers.append("moving_obstacles.ok is not true")
        if int(moving_obstacles.get("published_update_count") or 0) <= 0:
            blockers.append("moving_obstacles.published_update_count missing")
        if int(moving_obstacles.get("published_point_count_max") or 0) <= 0:
            blockers.append("moving_obstacles.published_point_count_max missing")
        if moving_trail.get("checked") is not True:
            blockers.append("moving_obstacles.trail_clearance.checked is not true")
        if moving_trail.get("collision") is True:
            blockers.append("moving_obstacles.trail_clearance.collision is true")
        if _safe_float(moving_trail_margin, default=-999.0) < 0.0:
            blockers.append("moving_obstacles.trail_clearance margin is negative")
    video = report.get("video") or {}
    video_path = str(video.get("path") or "")
    video_required = bool(video_path)
    video_artifact: dict[str, Any] = {}
    if video_required:
        if video.get("exists") is not True:
            blockers.append("native_pct video.exists is not true")
        if int(video.get("frames") or 0) <= 0:
            blockers.append("native_pct video.frames missing")
        video_artifact = _verify_video_artifact(
            report,
            video,
            blockers,
            prefix="native_pct",
        )
    if _safe_float(report.get("final_distance_m")) > 0.8:
        blockers.append("final_distance_m > 0.8")
    if _frame(report, "goal") and _frame(report, "goal") != "map":
        blockers.append("goal frame is not map")
    if _frame(report, "cmd_vel") and _frame(report, "cmd_vel") != "base_link":
        blockers.append("cmd_vel frame is not base_link")
    lidar_source = _require_mid360_pattern(report, blockers, "lidar_source")

    return not blockers, blockers, {
        "planner": report.get("planner"),
        "primary_planner": report.get("primary_planner"),
        "selected_planner": report.get("selected_planner"),
        "fallback_used": report.get("fallback_used"),
        "global_planner_source": report.get("global_planner_source"),
        "source_planning_contract": source_contract,
        "pct_native_runtime_used": pct_native_runtime_used,
        "pct_optimizer": pct_optimizer,
        "same_source_artifacts": same_source_artifacts,
        "final_distance_m": report.get("final_distance_m"),
        "moved_m": report.get("moved_m"),
        "obstacle_aware": {
            "enabled": obstacle_aware.get("enabled"),
            "metadata_points": obstacle_aware.get("metadata_points"),
        },
        "waypoint_safety_filter": {
            "enabled": waypoint_filter.get("enabled"),
            "goal_preserved": waypoint_filter.get("goal_preserved"),
            "source_path_goal_xy": waypoint_filter.get("source_path_goal_xy"),
            "follow_path_goal_xy": waypoint_filter.get("follow_path_goal_xy"),
            "goal_shift_m": waypoint_filter.get("goal_shift_m"),
        },
        "requested_goal_xy": report.get("requested_goal_xy"),
        "pct_path_goal_xy": report.get("pct_path_goal_xy"),
        "tracking_goal_xy": report.get("tracking_goal_xy"),
        "local_path_sample_count": len(local_path_samples),
        "min_clearance_m": clearance.get("min_clearance_m"),
        "clearance_checked": clearance.get("checked"),
        "collision": clearance.get("collision"),
        "trajectory_quality": {
            "ok": trajectory_quality.get("ok"),
            "p95_lateral_error_m": trajectory_quality.get("p95_lateral_error_m"),
            "final_progress_ratio": trajectory_quality.get("final_progress_ratio"),
        },
        "moving_obstacles": {
            "enabled": moving_obstacles.get("enabled"),
            "mode": moving_mode,
            "count": moving_obstacles.get("count"),
            "period_s": moving_obstacles.get("period_s"),
            "ok": moving_obstacles.get("ok"),
            "published_update_count": moving_obstacles.get("published_update_count"),
            "published_point_count_max": moving_obstacles.get("published_point_count_max"),
            "trail_collision": moving_trail.get("collision"),
            "trail_min_margin_m": moving_trail_margin,
        },
        "video": {
            "required": video_required,
            "path": video_path,
            "exists": video.get("exists"),
            "frames": video.get("frames"),
            "layout": video.get("layout"),
            "artifact": video_artifact,
        },
        "lidar_source": lidar_source,
        "frames": report.get("frames") or {},
    }


def _eval_pct_saved_map_navigation(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("ok") is not True:
        blockers.append("report.ok is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")
    if report.get("validation_level") != "saved_map_relocalized_pct_navigation":
        blockers.append("validation_level is not saved_map_relocalized_pct_navigation")

    relocalization = report.get("relocalization") or {}
    if relocalization.get("ok") is not True:
        blockers.append("relocalization.ok is not true")
    if str(relocalization.get("latest_health_state") or "").upper() != "LOCKED":
        blockers.append("relocalization latest_health_state is not LOCKED")

    preview = report.get("plan_preview") or {}
    native = report.get("native_gate") or {}
    prerequisite_failed = (
        preview.get("skipped") is True
        and preview.get("reason") == "saved_map_relocalization_prerequisite_failed"
    ) or (
        native.get("skipped") is True
        and native.get("reason") == "saved_map_relocalization_prerequisite_failed"
    )
    if prerequisite_failed:
        _extend_unique(blockers, [str(item) for item in report.get("blockers") or []])
        relocalize_report_artifact = _verify_file_artifact(
            report,
            report.get("relocalize_report"),
            blockers,
            prefix="pct_saved_map_navigation relocalize_report",
        )
        return False, blockers, {
            "tomogram": report.get("tomogram"),
            "relocalize_report": report.get("relocalize_report"),
            "relocalization": {
                "latest_health_state": relocalization.get("latest_health_state"),
                "saved_map_cloud_points_latest": relocalization.get(
                    "saved_map_cloud_points_latest"
                ),
                "map_to_odom_xy_m": relocalization.get("map_to_odom_xy_m"),
            },
            "plan_preview": {
                "skipped": preview.get("skipped"),
                "reason": preview.get("reason"),
                "path_count": preview.get("path_count"),
            },
            "native_gate": {
                "skipped": native.get("skipped"),
                "reason": native.get("reason"),
            },
            "artifacts": {
                "relocalize_report": relocalize_report_artifact,
            },
            "contract_checks": report.get("contract_checks") or {},
        }

    if preview.get("ok") is not True:
        blockers.append("plan_preview.ok is not true")
    if preview.get("selected_planner") != "pct":
        blockers.append("plan_preview.selected_planner is not pct")
    if preview.get("fallback_reason"):
        blockers.append("plan_preview fallback_reason is non-empty")
    if int(preview.get("path_count") or 0) < 2:
        blockers.append("plan_preview path_count < 2")

    tomogram_artifact = _verify_file_artifact(
        report,
        report.get("tomogram"),
        blockers,
        prefix="pct_saved_map_navigation tomogram",
    )
    relocalize_report_artifact = _verify_file_artifact(
        report,
        report.get("relocalize_report"),
        blockers,
        prefix="pct_saved_map_navigation relocalize_report",
    )
    same_source_binding = _same_source_saved_map_binding(report, blockers)
    same_source_artifacts = _require_same_source_map_artifacts(
        report,
        blockers,
        prefix="pct_saved_map_navigation",
    )
    same_source_hash_identity = _require_same_source_hash_identity(
        report,
        blockers,
        prefix="pct_saved_map_navigation",
    )

    native_ok, native_blockers, native_evidence = _eval_native_pct_mujoco(native)
    if not native_ok:
        blockers.extend(f"native_gate.{item}" for item in native_blockers)

    scene_meta = report.get("scene_obstacle_metadata") or {}
    if int(scene_meta.get("obstacle_count") or 0) <= 0:
        blockers.append("scene_obstacle_metadata.obstacle_count missing")

    return not blockers, blockers, {
        "tomogram": report.get("tomogram"),
        "relocalize_report": report.get("relocalize_report"),
        "relocalization": {
            "latest_health_state": relocalization.get("latest_health_state"),
            "saved_map_cloud_points_latest": relocalization.get("saved_map_cloud_points_latest"),
            "map_to_odom_xy_m": relocalization.get("map_to_odom_xy_m"),
        },
        "plan_preview": {
            "selected_case": preview.get("selected_case"),
            "selected_planner": preview.get("selected_planner"),
            "path_count": preview.get("path_count"),
        },
        "native_gate": native_evidence,
        "scene_obstacle_metadata": scene_meta,
        "artifacts": {
            "tomogram": tomogram_artifact,
            "relocalize_report": relocalize_report_artifact,
            "same_source_binding": same_source_binding,
            "same_source_artifacts": same_source_artifacts,
            "same_source_hash_identity": same_source_hash_identity,
        },
    }


def _eval_dynamic_obstacle_local_planner(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    report_errors = [str(error) for error in report.get("errors") or []]
    if report.get("ok") is not True:
        blockers.append("report.ok is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")
    _extend_unique(blockers, report_errors)

    phases = list(report.get("phases") or [])
    if report_errors and not phases:
        return False, blockers, {
            "backend_actual": report.get("backend_actual"),
            "execution_mode": report.get("execution_mode") or "",
            "dynamic_replan_verified": report.get("dynamic_replan_verified"),
            "obstacle_response_verified": report.get("obstacle_response_verified"),
            "clear_path_recovery_verified": report.get("clear_path_recovery_verified"),
            "min_clearance_m": report.get("min_clearance_m"),
            "phase_count": 0,
            "avoidance_sequence": [],
            "frames": report.get("frames") or {},
            "algorithm_backends": _extract_algorithm_backends(report),
            "environment": report.get("environment") or {},
            "errors": report_errors,
        }

    if report.get("backend_actual") != "nanobind":
        blockers.append("backend_actual is not nanobind")
    if report.get("dynamic_replan_verified") is not True:
        blockers.append("dynamic_replan_verified is not true")
    if report.get("obstacle_response_verified") is not True:
        blockers.append("obstacle_response_verified is not true")
    if report.get("clear_path_recovery_verified") is not True:
        blockers.append("clear_path_recovery_verified is not true")
    if _safe_float(report.get("min_clearance_m"), default=0.0) < 0.25:
        blockers.append("min_clearance_m < 0.25")

    by_name = {str(phase.get("name") or ""): phase for phase in phases}
    for name in ("clear_initial", "obstacle_left", "obstacle_right", "obstacle_center", "clear_recovered"):
        if name not in by_name:
            blockers.append(f"{name} phase missing")
    if by_name.get("obstacle_left", {}).get("avoidance_side") != "right":
        blockers.append("left obstacle did not produce right detour")
    if by_name.get("obstacle_right", {}).get("avoidance_side") != "left":
        blockers.append("right obstacle did not produce left detour")
    if by_name.get("clear_recovered", {}).get("avoidance_side") != "straight":
        blockers.append("clear_recovered did not return straight")
    for phase in phases:
        if int(phase.get("path_count") or 0) < 20:
            blockers.append(f"{phase.get('name') or 'unknown'} path_count < 20")
        if str(phase.get("path_frame_id") or "") != "map":
            blockers.append(f"{phase.get('name') or 'unknown'} path frame is not map")

    return not blockers, blockers, {
        "backend_actual": report.get("backend_actual"),
        "execution_mode": report.get("execution_mode") or "",
        "dynamic_replan_verified": report.get("dynamic_replan_verified"),
        "obstacle_response_verified": report.get("obstacle_response_verified"),
        "clear_path_recovery_verified": report.get("clear_path_recovery_verified"),
        "min_clearance_m": report.get("min_clearance_m"),
        "phase_count": len(phases),
        "avoidance_sequence": [
            by_name.get("clear_initial", {}).get("avoidance_side"),
            by_name.get("obstacle_left", {}).get("avoidance_side"),
            by_name.get("obstacle_right", {}).get("avoidance_side"),
            by_name.get("obstacle_center", {}).get("avoidance_side"),
            by_name.get("clear_recovered", {}).get("avoidance_side"),
        ],
        "frames": report.get("frames") or {},
        "algorithm_backends": _extract_algorithm_backends(report),
        "environment": report.get("environment") or {},
        "errors": report_errors,
    }


def _eval_fastlio2_live(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    runtime_evidence_blockers, runtime_evidence = _shared_runtime_evidence(
        report,
        expected_contract="mujoco_fastlio2_live",
        require_contract=True,
        require_paths=False,
        require_command=False,
        require_frame_links=True,
        require_data_flow=True,
    )
    _extend_unique(blockers, runtime_evidence_blockers)
    for key in (
        "ok",
        "simulation_only",
        "live_mujoco_lidar_verified",
        "live_mujoco_imu_verified",
        "slam_algorithm_output_verified",
        "canonical_nav_outputs_verified",
        "bridge_verified",
    ):
        if report.get(key) is not True:
            blockers.append(f"{key} is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if report.get("cmd_vel_sent_to_hardware") is not False:
        blockers.append("cmd_vel_sent_to_hardware is not false")
    published_lidar_frame = _frame(report, "published_lidar")
    if published_lidar_frame and published_lidar_frame not in {"body", "lidar_link"}:
        blockers.append("published_lidar frame is not body or lidar_link")
    outputs = report.get("outputs") or {}
    for key in ("nav_odometry", "nav_registered_cloud", "nav_map_cloud"):
        if int(outputs.get(key) or 0) <= 0:
            blockers.append(f"outputs.{key} missing")
    lidar_source = _require_mid360_pattern(report, blockers, "lidar_source")
    frontier = report.get("lingtu_frontier") or {}
    tare = report.get("lingtu_tare") or {}
    inspection = report.get("lingtu_inspection") or {}
    navigation_chain = report.get("navigation_chain") or {}
    map_growth = report.get("map_growth") or {}
    if (
        frontier.get("enabled") is True
        or tare.get("enabled") is True
        or inspection.get("enabled") is True
    ):
        _require_live_nav_map_growth(map_growth, blockers)
    if frontier.get("enabled") is True:
        if frontier.get("verified") is not True:
            blockers.append("lingtu_frontier.verified is not true")
        if frontier.get("started_after_slam_ready") is not True:
            blockers.append("lingtu_frontier.started_after_slam_ready is not true")
        if int(frontier.get("goal_count") or 0) <= 0:
            blockers.append("lingtu_frontier.goal_count missing")
        min_required_goals = int(frontier.get("min_required_goals") or 1)
        if int(frontier.get("successful_navigation_goal_count") or 0) < min_required_goals:
            blockers.append("lingtu_frontier.successful_navigation_goal_count below required")
        if int(frontier.get("failed_navigation_goal_count") or 0) > 0:
            blockers.append("lingtu_frontier.failed_navigation_goal_count is nonzero")
        frontier_health = frontier.get("frontier_health")
        if (
            isinstance(frontier_health, dict)
            and int(frontier_health.get("blocked_goal_count") or 0) > 0
        ):
            blockers.append("lingtu_frontier.frontier_health.blocked_goal_count is nonzero")
        if int(outputs.get("nav_cmd_vel_nonzero") or 0) <= 0:
            blockers.append("outputs.nav_cmd_vel_nonzero missing")
        if navigation_chain.get("planner_fallback_used") is True:
            blockers.append("navigation_chain.planner_fallback_used is true")
        if navigation_chain.get("planner_repair_used") is True:
            blockers.append("navigation_chain.planner_repair_used is true")
        direct_goal_fallback = navigation_chain.get("direct_goal_fallback")
        if isinstance(direct_goal_fallback, dict) and direct_goal_fallback.get("used") is True:
            blockers.append("navigation_chain.direct_goal_fallback.used is true")
        health = navigation_chain.get("health") if isinstance(navigation_chain.get("health"), dict) else {}
        if health and str(health.get("plan_safety_policy") or "") != "reject":
            blockers.append("navigation_chain.health.plan_safety_policy is not reject")
        explored_area = map_growth.get("exploration_known_area") or {}
        coverage = map_growth.get("exploration_coverage") or {}
        grid_growth_is_acceptance_metric = (
            map_growth.get("exploration_grid_growth_is_acceptance_metric") is not False
        )
        min_explored = _safe_float(map_growth.get("min_explored_area_growth_m2"), default=0.0)
        min_coverage = _safe_float(
            map_growth.get("min_exploration_coverage_growth_ratio"),
            default=0.0,
        )
        if grid_growth_is_acceptance_metric:
            if _safe_float(explored_area.get("growth_m2"), default=0.0) < min_explored:
                blockers.append("exploration known area growth below threshold")
            if _safe_float(coverage.get("growth_ratio"), default=0.0) < min_coverage:
                blockers.append("exploration coverage growth below threshold")
    if tare.get("enabled") is True:
        if tare.get("verified") is not True:
            blockers.append("lingtu_tare.verified is not true")
        if tare.get("started_after_slam_ready") is not True:
            blockers.append("lingtu_tare.started_after_slam_ready is not true")
        _require_exploration_grid_signal(tare, blockers, prefix="lingtu_tare")
        _require_exploration_progress(map_growth, blockers)
        if int(tare.get("goal_count") or 0) <= 0:
            blockers.append("lingtu_tare.goal_count missing")
        min_required_goals = int(tare.get("min_required_goals") or 1)
        if int(tare.get("successful_navigation_goal_count") or 0) < min_required_goals:
            blockers.append("lingtu_tare.successful_navigation_goal_count below required")
        if int(tare.get("failed_navigation_goal_count") or 0) > 0:
            blockers.append("lingtu_tare.failed_navigation_goal_count is nonzero")
        if int(outputs.get("nav_cmd_vel_nonzero") or 0) <= 0:
            blockers.append("outputs.nav_cmd_vel_nonzero missing")
        if navigation_chain.get("planner_fallback_used") is True:
            blockers.append("navigation_chain.planner_fallback_used is true")
        if navigation_chain.get("planner_repair_used") is True:
            blockers.append("navigation_chain.planner_repair_used is true")
        direct_goal_fallback = navigation_chain.get("direct_goal_fallback")
        if isinstance(direct_goal_fallback, dict) and direct_goal_fallback.get("used") is True:
            blockers.append("navigation_chain.direct_goal_fallback.used is true")
        health = navigation_chain.get("health") if isinstance(navigation_chain.get("health"), dict) else {}
        if health and str(health.get("plan_safety_policy") or "") != "reject":
            blockers.append("navigation_chain.health.plan_safety_policy is not reject")
        if int(tare.get("global_path_points_max") or 0) < 2:
            blockers.append("lingtu_tare.global_path_points_max below required")
        if int(tare.get("local_path_points_max") or 0) < 2:
            blockers.append("lingtu_tare.local_path_points_max below required")

    if inspection.get("enabled") is True:
        inspection_success_count = int(inspection.get("successful_navigation_goal_count") or 0)
        if inspection.get("verified") is not True:
            blockers.append("lingtu_inspection.verified is not true")
        if inspection.get("started_after_slam_ready") is not True:
            blockers.append("lingtu_inspection.started_after_slam_ready is not true")
        if int(inspection.get("goal_count") or 0) <= 0:
            blockers.append("lingtu_inspection.goal_count missing")
        min_required_checkpoints = int(inspection.get("min_required_checkpoints") or 1)
        inspection_success_requirement_met = inspection_success_count >= min_required_checkpoints
        if not inspection_success_requirement_met:
            blockers.append("lingtu_inspection.successful_navigation_goal_count below required")
        if (
            int(inspection.get("failed_navigation_goal_count") or 0) > 0
            and not inspection_success_requirement_met
        ):
            blockers.append("lingtu_inspection.failed_navigation_goal_count is nonzero")
        if int(outputs.get("nav_cmd_vel_nonzero") or 0) <= 0:
            blockers.append("outputs.nav_cmd_vel_nonzero missing")
        if navigation_chain.get("planner_fallback_used") is True:
            blockers.append("navigation_chain.planner_fallback_used is true")
        if navigation_chain.get("planner_repair_used") is True:
            blockers.append("navigation_chain.planner_repair_used is true")
        direct_goal_fallback = navigation_chain.get("direct_goal_fallback")
        if isinstance(direct_goal_fallback, dict) and direct_goal_fallback.get("used") is True:
            blockers.append("navigation_chain.direct_goal_fallback.used is true")
        health = navigation_chain.get("health") if isinstance(navigation_chain.get("health"), dict) else {}
        if health and str(health.get("plan_safety_policy") or "") != "reject":
            blockers.append("navigation_chain.health.plan_safety_policy is not reject")
        if int(inspection.get("global_path_points_max") or 0) < 2:
            blockers.append("lingtu_inspection.global_path_points_max below required")
        if int(inspection.get("local_path_points_max") or 0) < 2:
            blockers.append("lingtu_inspection.local_path_points_max below required")

    moving_obstacles = (
        report.get("moving_obstacles")
        if isinstance(report.get("moving_obstacles"), dict)
        else {}
    )
    if _moving_obstacles_enabled(moving_obstacles):
        moving_obstacle_evidence = _require_moving_obstacle_evidence(
            moving_obstacles,
            blockers,
        )
    else:
        moving_obstacle_evidence = {
            "enabled": moving_obstacles.get("enabled"),
            "mode": str(moving_obstacles.get("mode") or "none"),
            "count": moving_obstacles.get("count"),
            "period_s": moving_obstacles.get("period_s"),
            "ok": moving_obstacles.get("ok"),
            "published_update_count": moving_obstacles.get("published_update_count"),
            "published_point_count_max": moving_obstacles.get("published_point_count_max"),
            "trail_collision": None,
            "trail_min_margin_m": None,
        }
    video_evidence = _require_fastlio2_live_video_evidence(report, blockers)

    return not blockers, blockers, {
        "outputs": outputs,
        "states_seen": report.get("states_seen") or [],
        "point_count": report.get("point_count") or {},
        "lidar_source": lidar_source,
        "map_growth": map_growth,
        "lingtu_frontier": frontier,
        "lingtu_tare": tare,
        "lingtu_inspection": inspection,
        "navigation_chain": navigation_chain,
        "moving_obstacles": moving_obstacle_evidence,
        "video": video_evidence,
        "frames": report.get("frames") or {},
        "runtime_evidence": runtime_evidence,
    }


def _subcheck_ok(report: dict[str, Any], key: str, blockers: list[str]) -> dict[str, Any]:
    value = report.get(key) if isinstance(report.get(key), dict) else {}
    if value.get("checked") is not True:
        blockers.append(f"{key}.checked is not true")
    if value.get("ok") is not True:
        blockers.append(f"{key}.ok is not true")
    return value


def _eval_fastlio2_dynamic_inspection(
    report: dict[str, Any],
) -> tuple[bool, list[str], dict[str, Any]]:
    base_ok, blockers, evidence = _eval_fastlio2_live(report)
    blockers = list(blockers)
    inspection = report.get("lingtu_inspection") if isinstance(report.get("lingtu_inspection"), dict) else {}
    navigation_chain = report.get("navigation_chain") if isinstance(report.get("navigation_chain"), dict) else {}
    moving_obstacles = (
        report.get("moving_obstacles")
        if isinstance(report.get("moving_obstacles"), dict)
        else {}
    )
    video = _fastlio2_live_video_evidence(report)

    if inspection.get("enabled") is not True:
        blockers.append("lingtu_inspection.enabled is not true")
    if str(inspection.get("global_planner") or "").lower() != "pct":
        blockers.append("lingtu_inspection.global_planner is not pct")
    if inspection.get("replan_on_costmap_update") is not False:
        blockers.append("lingtu_inspection.replan_on_costmap_update is not false")
    if str(inspection.get("patrol_state") or "").upper() != "SUCCESS":
        blockers.append("lingtu_inspection.patrol_state is not SUCCESS")
    min_checkpoints = max(3, _safe_int(inspection.get("min_required_checkpoints"), default=3))
    if _safe_int(inspection.get("successful_navigation_goal_count")) < min_checkpoints:
        blockers.append("lingtu_inspection successful checkpoints below core algorithm threshold")

    if navigation_chain.get("planner_fallback_used") is not False:
        blockers.append("navigation_chain.planner_fallback_used is not false")
    if navigation_chain.get("planner_repair_used") is not False:
        blockers.append("navigation_chain.planner_repair_used is not false")

    if not _moving_obstacles_enabled(moving_obstacles):
        blockers.append("moving_obstacles are not enabled")
    if _safe_int(moving_obstacles.get("count")) < 3:
        blockers.append("moving_obstacles.count < 3")
    speed = (
        moving_obstacles.get("speed_bounds")
        if isinstance(moving_obstacles.get("speed_bounds"), dict)
        else {}
    )
    if _safe_float(speed.get("peak_planar_speed_bound_mps"), default=0.0) < 0.75:
        blockers.append("moving_obstacles peak_planar_speed_bound_mps < 0.75")
    trail = (
        moving_obstacles.get("trail_clearance")
        if isinstance(moving_obstacles.get("trail_clearance"), dict)
        else {}
    )
    trail_margin = _moving_obstacle_trail_margin(trail)
    if _safe_float(trail_margin, default=-999.0) < 0.25:
        blockers.append("moving_obstacles trail clearance margin < 0.25")

    if not video.get("path"):
        blockers.append("fastlio2_dynamic_inspection video_path missing")
    if _safe_int(video.get("frame_count")) <= 0:
        blockers.append("fastlio2_dynamic_inspection video_frame_count missing")
    if _safe_int(video.get("sample_count")) <= 0:
        blockers.append("fastlio2_dynamic_inspection video_sample_count missing")
    if report.get("video_mujoco_render_error"):
        blockers.append("video_mujoco_render_error is present")
    video_artifact = _verify_video_artifact(
        report,
        video,
        blockers,
        prefix="fastlio2_dynamic_inspection",
    )

    true_mapping_path = str(report.get("true_mapping_input_path") or "")
    for token in ("/lidar/raw_frame", "/imu/raw", "fastlio2", "/slam/odometry", "/slam/map_cloud"):
        if token not in true_mapping_path:
            blockers.append(f"true_mapping_input_path missing {token}")

    z_consistency = _subcheck_ok(report, "fastlio2_z_consistency", blockers)
    yaw_consistency = _subcheck_ok(report, "fastlio2_yaw_consistency", blockers)
    motion_consistency = _subcheck_ok(report, "fastlio2_motion_consistency", blockers)

    contract = report.get("deliverable_contract") if isinstance(report.get("deliverable_contract"), dict) else {}
    checks = contract.get("checks") if isinstance(contract.get("checks"), dict) else {}
    for key in (
        "raw_mujoco_lidar",
        "raw_mujoco_imu",
        "fastlio2_odometry_and_map",
        "inspection_patrol",
        "moving_obstacle_evidence",
        "nav_cmd_vel_nonzero",
        "same_source_map_artifact",
    ):
        if checks.get(key) is not True:
            blockers.append(f"deliverable_contract.checks.{key} is not true")
    same_source_artifacts = _require_same_source_map_artifacts(
        report,
        blockers,
        prefix="fastlio2_dynamic_inspection",
        require_tomogram=True,
    )

    dynamic_evidence = {
        "inspection": {
            "global_planner": inspection.get("global_planner"),
            "replan_on_costmap_update": inspection.get("replan_on_costmap_update"),
            "successful_navigation_goal_count": inspection.get("successful_navigation_goal_count"),
            "min_required_checkpoints": inspection.get("min_required_checkpoints"),
            "patrol_state": inspection.get("patrol_state"),
            "local_path_count": inspection.get("local_path_count"),
        },
        "moving_obstacles": {
            "count": moving_obstacles.get("count"),
            "period_s": moving_obstacles.get("period_s"),
            "speed_bounds": speed,
            "trail_min_margin_m": trail_margin,
            "published_update_count": moving_obstacles.get("published_update_count"),
            "published_point_count_max": moving_obstacles.get("published_point_count_max"),
        },
        "video": video,
        "video_artifact": video_artifact,
        "fastlio2_consistency": {
            "z_delta_error_m": z_consistency.get("z_delta_error_m"),
            "yaw_delta_error_rad": yaw_consistency.get("yaw_delta_error_rad"),
            "fastlio2_moved_m": motion_consistency.get("fastlio2_moved_m"),
            "sim_moved_m": motion_consistency.get("sim_moved_m"),
        },
        "true_mapping_input_path": true_mapping_path,
        "same_source_artifacts": same_source_artifacts,
    }
    evidence["core_algorithm"] = dynamic_evidence
    return bool(base_ok) and not blockers, blockers, evidence


def _eval_moving_obstacle_sweep(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("ok") is not True:
        blockers.append("moving_obstacle_sweep report.ok is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")

    required_speed_bins = list(report.get("required_speed_bins") or [])
    required_density_bins = list(report.get("required_density_bins") or [])
    required_pairs = list(report.get("required_pairs") or [])
    covered_pairs = list(report.get("covered_pairs") or [])
    missing_pairs = list(report.get("missing_pairs") or [])
    required_live_nav_chain = report.get("required_live_nav_chain")
    required_scan_time_profile = str(report.get("required_scan_time_profile") or "")
    require_video_file = report.get("require_video_file") is True
    cases = list(report.get("cases") or [])
    video_artifacts: dict[str, Any] = {}
    same_source_artifacts: dict[str, Any] = {}
    for speed_bin in ("slow", "fast"):
        if speed_bin not in required_speed_bins:
            blockers.append(f"required_speed_bins missing {speed_bin}")
    for density_bin in ("sparse", "dense"):
        if density_bin not in required_density_bins:
            blockers.append(f"required_density_bins missing {density_bin}")
    if len(required_pairs) < 4:
        blockers.append("required speed/density pair count < 4")
    if _safe_int(report.get("case_count")) < len(required_pairs):
        blockers.append("case_count below required pair count")
    if _safe_int(report.get("passed_pair_count")) < len(required_pairs):
        blockers.append("passed_pair_count below required pair count")
    for pair in required_pairs:
        if pair not in covered_pairs:
            blockers.append(f"required speed/density pair missing: {pair}")
    for pair in missing_pairs:
        blockers.append(f"missing required speed/density pair: {pair}")
    if required_live_nav_chain is not True:
        blockers.append("required_live_nav_chain is not true")
    if required_scan_time_profile != "physical_rolling":
        blockers.append("required_scan_time_profile is not physical_rolling")
    for case in cases:
        if not isinstance(case, dict):
            continue
        pair = str(case.get("pair") or case.get("path") or "unknown")
        if str(case.get("scan_time_profile") or "") != "physical_rolling":
            blockers.append(f"moving_obstacle_sweep case {pair} scan_time_profile is not physical_rolling")
        live_nav_chain = (
            case.get("live_nav_chain")
            if isinstance(case.get("live_nav_chain"), dict)
            else {}
        )
        if live_nav_chain.get("ok") is not True:
            blockers.append(f"moving_obstacle_sweep case {pair} live nav chain is not ok")
            for child_blocker in live_nav_chain.get("blockers") or []:
                blockers.append(
                    f"moving_obstacle_sweep case {pair}: live nav chain: {child_blocker}"
                )
        if required_live_nav_chain is True or live_nav_chain:
            same_source_artifacts[pair] = _require_same_source_map_artifacts(
                live_nav_chain,
                blockers,
                prefix=f"moving_obstacle_sweep case {pair}.live_nav_chain",
                require_tomogram=True,
            )
        if require_video_file:
            video = case.get("video") if isinstance(case.get("video"), dict) else {}
            video_path = str(case.get("video_path") or video.get("path") or "")
            video_frames = _safe_int(
                case.get("video_frame_count", video.get("frame_count", video.get("frames")))
            )
            video_samples = _safe_int(
                case.get("video_sample_count", video.get("sample_count", video.get("samples")))
            )
            if not video_path:
                blockers.append(f"moving_obstacle_sweep case {pair} video_path missing")
            if video_frames <= 0:
                blockers.append(f"moving_obstacle_sweep case {pair} video_frame_count missing")
            if video_samples <= 0:
                blockers.append(f"moving_obstacle_sweep case {pair} video_sample_count missing")
            video_artifacts[pair] = _verify_video_artifact(
                {"_report_path": case.get("path") or report.get("_report_path", "")},
                {
                    "path": video_path,
                    "frames": video_frames,
                    "samples": video_samples,
                },
                blockers,
                prefix=f"moving_obstacle_sweep case {pair}",
            )
    for blocker in report.get("blockers") or []:
        blockers.append(str(blocker))
    for blocker in report.get("environment_blockers") or []:
        if str(blocker) not in blockers:
            blockers.append(str(blocker))
    for blocker in report.get("artifact_blockers") or []:
        if str(blocker) not in blockers:
            blockers.append(str(blocker))

    return not blockers, blockers, {
        "case_count": report.get("case_count"),
        "passed_case_count": report.get("passed_case_count"),
        "failed_case_count": report.get("failed_case_count"),
        "passed_pair_count": report.get("passed_pair_count"),
        "minimal_red_defect": report.get("minimal_red_defect") or {},
        "blocking_subsystems": report.get("blocking_subsystems") or [],
        "environment_blockers": report.get("environment_blockers") or [],
        "artifact_blockers": report.get("artifact_blockers") or [],
        "preflight": report.get("preflight") or {},
        "next_action_hint": report.get("next_action_hint") or "",
        "required_speed_bins": required_speed_bins,
        "required_density_bins": required_density_bins,
        "required_pairs": required_pairs,
        "covered_speed_bins": report.get("covered_speed_bins") or [],
        "covered_density_bins": report.get("covered_density_bins") or [],
        "covered_pairs": covered_pairs,
        "missing_pairs": missing_pairs,
        "required_live_nav_chain": required_live_nav_chain,
        "required_scan_time_profile": required_scan_time_profile,
        "min_clearance_m": report.get("min_clearance_m"),
        "require_video": report.get("require_video"),
        "require_video_file": report.get("require_video_file"),
        "video_artifacts": video_artifacts,
        "same_source_artifacts": same_source_artifacts,
        "cases": cases,
    }


def _eval_large_loop_closure(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("ok") is not True:
        blockers.append("large_loop_closure report.ok is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")

    best_case = report.get("best_case") if isinstance(report.get("best_case"), dict) else {}
    thresholds = report.get("thresholds") if isinstance(report.get("thresholds"), dict) else {}
    min_path_length = _safe_float(thresholds.get("min_path_length_m"), default=20.0)
    max_loop_error = _safe_float(thresholds.get("max_loop_closure_error_m"), default=0.75)
    max_fastlio_loop_error = _safe_float(
        thresholds.get("max_fastlio_loop_closure_error_m"),
        default=1.0,
    )
    max_loop_yaw_error = _safe_float(thresholds.get("max_loop_yaw_error_rad"), default=0.5)
    required_scan_time_profile = str(thresholds.get("required_scan_time_profile") or "")
    require_video_file = thresholds.get("require_video_file") is True
    passed_case_count = _safe_int(report.get("passed_case_count"))
    best_case_video_artifact: dict[str, Any] = {}
    best_case_same_source_artifacts: dict[str, Any] = {}
    if required_scan_time_profile != "physical_rolling":
        blockers.append("large_loop_closure required_scan_time_profile is not physical_rolling")
    if passed_case_count <= 0:
        blockers.append("passed_case_count is 0")
    else:
        if str(best_case.get("scan_time_profile") or "") != "physical_rolling":
            blockers.append("best_case.scan_time_profile is not physical_rolling")
        if str(best_case.get("global_planner") or "").lower() != "pct":
            blockers.append("best_case.global_planner is not pct")
        if _safe_float(best_case.get("sim_path_length_m"), default=0.0) < min_path_length:
            blockers.append("best_case.sim_path_length_m below threshold")
        if _safe_float(best_case.get("fastlio2_path_length_m"), default=0.0) < min_path_length * 0.8:
            blockers.append("best_case.fastlio2_path_length_m below threshold")
        if _safe_float(best_case.get("sim_loop_closure_error_m"), default=999.0) > max_loop_error:
            blockers.append("best_case.sim_loop_closure_error_m above threshold")
        if (
            _safe_float(best_case.get("fastlio2_loop_closure_error_m"), default=999.0)
            > max_fastlio_loop_error
        ):
            blockers.append("best_case.fastlio2_loop_closure_error_m above threshold")
        if _safe_float(best_case.get("sim_loop_yaw_error_rad"), default=999.0) > max_loop_yaw_error:
            blockers.append("best_case.sim_loop_yaw_error_rad above threshold")
        if (
            _safe_float(best_case.get("fastlio2_loop_yaw_error_rad"), default=999.0)
            > max_loop_yaw_error
        ):
            blockers.append("best_case.fastlio2_loop_yaw_error_rad above threshold")
        if _safe_int(best_case.get("nav_cmd_vel_nonzero")) <= 0:
            blockers.append("best_case.nav_cmd_vel_nonzero missing")
        if _safe_int(best_case.get("local_path_count")) <= 0:
            blockers.append("best_case.local_path_count missing")
        if _safe_int(best_case.get("video_frame_count")) <= 0:
            blockers.append("best_case.video_frame_count missing")
        best_case_same_source_artifacts = _require_same_source_map_artifacts(
            best_case,
            blockers,
            prefix="large_loop_closure best_case",
            require_tomogram=True,
        )
        if require_video_file:
            best_video = best_case.get("video") if isinstance(best_case.get("video"), dict) else {}
            video_path = str(best_case.get("video_path") or best_video.get("path") or "")
            video_frames = _safe_int(
                best_case.get("video_frame_count", best_video.get("frame_count", best_video.get("frames")))
            )
            video_samples = _safe_int(
                best_case.get("video_sample_count", best_video.get("sample_count", best_video.get("samples")))
            )
            if not video_path:
                blockers.append("best_case.video_path missing")
            if video_samples <= 0:
                blockers.append("best_case.video_sample_count missing")
            best_case_video_artifact = _verify_video_artifact(
                {"_report_path": best_case.get("path") or report.get("_report_path", "")},
                {
                    "path": video_path,
                    "frames": video_frames,
                    "samples": video_samples,
                },
                blockers,
                prefix="large_loop_closure best_case",
            )
    for blocker in report.get("blockers") or []:
        blockers.append(str(blocker))
    for index, case in enumerate(report.get("cases") or []):
        if not isinstance(case, dict) or case.get("ok") is True:
            continue
        for blocker in case.get("blockers") or []:
            blockers.append(f"case[{index}]: {blocker}")

    return not blockers, blockers, {
        "case_count": report.get("case_count"),
        "passed_case_count": report.get("passed_case_count"),
        "failed_case_count": report.get("failed_case_count"),
        "best_case": best_case,
        "minimal_red_defect": report.get("minimal_red_defect") or {},
        "blocking_subsystems": report.get("blocking_subsystems") or [],
        "thresholds": thresholds,
        "best_case_video_artifact": best_case_video_artifact,
        "best_case_same_source_artifacts": best_case_same_source_artifacts,
        "cases": report.get("cases") or [],
    }


def _eval_policy_nav(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("passed") is not True:
        blockers.append("report.passed is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")

    checks = list(report.get("checks") or [])
    policy_loaded = any(bool(check.get("policy_loaded")) for check in checks)
    nav_checks = [check for check in checks if check.get("mode") == "full_stack_policy_nav"]
    nav_passed = any(bool(check.get("passed")) for check in nav_checks)
    if not policy_loaded:
        blockers.append("no ONNX policy loaded")
    if not nav_passed:
        blockers.append("full_stack_policy_nav did not pass")
    for nav_check in nav_checks:
        seen = nav_check.get("seen") or {}
        planner_status = nav_check.get("global_planner_backend_status") or {}
        planner_backend = str(planner_status.get("configured_backend") or "")
        local_backend = str(nav_check.get("local_planner_backend_actual") or "")
        path_backend = str(nav_check.get("path_follower_backend_actual") or "")
        if planner_backend != PRODUCT_GLOBAL_PLANNER_BACKEND:
            blockers.append(
                "full_stack_policy_nav global_planner_backend_status.configured_backend is not octoplanner3d"
            )
        if local_backend != PRODUCT_LOCAL_PLANNER_BACKEND:
            blockers.append(
                "full_stack_policy_nav local_planner_backend_actual is not nanobind"
            )
        if path_backend != PRODUCT_PATH_FOLLOWER_BACKEND:
            blockers.append(
                "full_stack_policy_nav path_follower_backend_actual is not nav_kernel"
            )
        if int(seen.get("direct_fallback", 0)) != 0:
            blockers.append("full_stack_policy_nav used direct_goal_fallback")
        for key in ("local_path", "path_follower_cmd", "mux_cmd", "waypoints"):
            if int(seen.get(key, 0)) <= 0:
                blockers.append(f"full_stack_policy_nav missing {key}")
    for check in checks:
        if check.get("cmd_vel_sent_to_hardware") is not False:
            blockers.append(f"{check.get('mode')} did not report hardware-safe cmd_vel")
        contacts = check.get("contacts") if isinstance(check.get("contacts"), dict) else {}
        if int(contacts.get("foot_contact_sample_count") or 0) <= 0:
            blockers.append(f"{check.get('mode')} missing foot-ground contact samples")
        if int(contacts.get("unique_feet_count") or 0) < 2:
            blockers.append(f"{check.get('mode')} saw fewer than two contacting feet")
        if int(contacts.get("non_foot_ground_contacts") or 0) != 0:
            blockers.append(f"{check.get('mode')} had non-foot ground contacts")

    last_nav = nav_checks[-1] if nav_checks else {}
    return not blockers, blockers, {
        "check_count": len(checks),
        "policy_loaded": policy_loaded,
        "nav_passed": nav_passed,
        "policy_paths": [check.get("policy_path") for check in checks if check.get("policy_path")],
        "global_planner_backend_status": last_nav.get("global_planner_backend_status"),
        "local_planner_backend_actual": last_nav.get("local_planner_backend_actual"),
        "path_follower_backend_actual": last_nav.get("path_follower_backend_actual"),
        "nav_seen": last_nav.get("seen") or {},
        "nav_state": last_nav.get("nav_state"),
        "nav_dist_to_goal_m": last_nav.get("dist_to_goal_m"),
        "contacts": [check.get("contacts") for check in checks if check.get("contacts")],
        "frames": [check.get("frames") for check in checks if check.get("frames")],
    }


def _eval_gateway_dry_run(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("ok") is not True:
        blockers.append("report.ok is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")
    if report.get("gateway_used") is not True:
        blockers.append("gateway_used is not true")
    if not _bool_false(report, "driver_used"):
        blockers.append("driver_used is not false")
    published = report.get("published") or {}
    expected = {"goal_pose": 1, "cmd_vel": 0, "stop_cmd": 0}
    for name, count in expected.items():
        if name not in published:
            blockers.append(f"published.{name} is missing")
        elif int(published.get(name, 0)) != count:
            blockers.append(f"published.{name} is not {count}")

    return not blockers, blockers, {
        "published": published,
        "frames": report.get("frames") or {},
        "target": report.get("target"),
    }


def _eval_routecheck_preflight(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("mode") != "routecheck_non_motion":
        blockers.append("mode is not routecheck_non_motion")
    if report.get("outcome") != "pass":
        blockers.append("outcome is not pass")
    if int(report.get("exit_status") or 0) != 0:
        blockers.append("exit_status is not 0")
    if report.get("non_motion") is not True:
        blockers.append("non_motion is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")
    if report.get("gateway_used") is not True:
        blockers.append("gateway_used is not true")
    if not _bool_false(report, "driver_used"):
        blockers.append("driver_used is not false")
    published = report.get("published") or {}
    for name in ("goal_pose", "cmd_vel", "stop_cmd"):
        if name not in published:
            blockers.append(f"published.{name} is missing")
        elif int(published.get(name, 0)) != 0:
            blockers.append(f"published.{name} is not 0")
    if not report.get("map"):
        blockers.append("map is missing")
    goal = report.get("goal") or {}
    for key in ("x", "y", "yaw"):
        if key not in goal:
            blockers.append(f"goal.{key} is missing")

    phases = report.get("phases") or {}
    phase_evidence: dict[str, Any] = {}
    for name in ("baseline", "candidate"):
        phase = phases.get(name) or {}
        phase_evidence[name] = {
            "feasible": phase.get("feasible"),
            "count": phase.get("count"),
            "selected_planner": phase.get("selected_planner") or phase.get("planner"),
            "plan_safety_policy": phase.get("plan_safety_policy"),
            "path_safety_ok": phase.get("path_safety_ok"),
            "active_cmd_source_before": phase.get("active_cmd_source_before"),
            "rejected_plan_count": phase.get("rejected_plan_count"),
        }
        if phase.get("non_motion") is not True:
            blockers.append(f"{name}.non_motion is not true")
        if phase.get("can_accept_goal") is not True:
            blockers.append(f"{name}.can_accept_goal is not true")
        if str(phase.get("active_cmd_source_before") or "none").lower() not in {"none", "null", "-", ""}:
            blockers.append(f"{name}.active_cmd_source_before is not none")
        if phase.get("feasible") is not True:
            blockers.append(f"{name}.feasible is not true")
        if int(phase.get("count") or 0) < 2:
            blockers.append(f"{name}.count < 2")
        if not (phase.get("selected_planner") or phase.get("planner")):
            blockers.append(f"{name}.selected_planner is missing")
        if phase.get("path_safety_ok") is not True:
            blockers.append(f"{name}.path_safety_ok is not true")

    return not blockers, blockers, {
        "map": report.get("map"),
        "goal": goal,
        "outcome": report.get("outcome"),
        "phases": phase_evidence,
        "published": published,
        "artifacts": report.get("artifacts") or {},
    }


def _eval_blocked_route_replan_preflight(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("schema_version") != "lingtu.blocked_route_replan_gate.v1":
        blockers.append("schema_version is not lingtu.blocked_route_replan_gate.v1")
    if report.get("ok") is not True:
        blockers.append("report.ok is not true")
    if report.get("mode") != "blocked_route_replan_non_motion":
        blockers.append("mode is not blocked_route_replan_non_motion")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")
    if report.get("gateway_used") is not True:
        blockers.append("gateway_used is not true")
    if not _bool_false(report, "driver_used"):
        blockers.append("driver_used is not false")

    published = report.get("published") or {}
    for name in ("goal_pose", "cmd_vel", "stop_cmd"):
        if name not in published:
            blockers.append(f"published.{name} is missing")
        elif int(published.get(name, 0)) != 0:
            blockers.append(f"published.{name} is not 0")

    if report.get("baseline_path_intersects_block") is not True:
        blockers.append("baseline_path_intersects_block is not true")
    if report.get("candidate_path_avoids_block") is not True:
        blockers.append("candidate_path_avoids_block is not true")
    if report.get("candidate_route_changed") is not True:
        blockers.append("candidate_route_changed is not true")
    if report.get("blocked_route_replanned") is not True:
        blockers.append("blocked_route_replanned is not true")

    phases = report.get("phases") if isinstance(report.get("phases"), dict) else {}
    phase_evidence: dict[str, Any] = {}
    for name in ("baseline", "blocked_candidate"):
        phase = phases.get(name) if isinstance(phases.get(name), dict) else {}
        phase_evidence[name] = {
            "feasible": phase.get("feasible"),
            "count": phase.get("count"),
            "selected_planner": phase.get("selected_planner"),
            "active_cmd_source_before": phase.get("active_cmd_source_before"),
            "intersects_block": phase.get("intersects_block"),
            "min_block_clearance_m": phase.get("min_block_clearance_m"),
            "max_lateral_offset_m": phase.get("max_lateral_offset_m"),
            "blocked_route_replanned": phase.get("blocked_route_replanned"),
            "reasons": phase.get("reasons") or [],
        }
        if phase.get("non_motion") is not True:
            blockers.append(f"{name}.non_motion is not true")
        if phase.get("can_accept_goal") is not True:
            blockers.append(f"{name}.can_accept_goal is not true")
        if str(phase.get("active_cmd_source_before") or "none").lower() not in {"none", "null", "-", ""}:
            blockers.append(f"{name}.active_cmd_source_before is not none")
        if phase.get("feasible") is not True:
            blockers.append(f"{name}.feasible is not true")
        if int(phase.get("count") or 0) < 2:
            blockers.append(f"{name}.count < 2")
        if not phase.get("selected_planner"):
            blockers.append(f"{name}.selected_planner is missing")

    candidate = phases.get("blocked_candidate") if isinstance(phases.get("blocked_candidate"), dict) else {}
    thresholds = report.get("thresholds") if isinstance(report.get("thresholds"), dict) else {}
    min_clearance = _safe_float(thresholds.get("min_block_clearance_m"), 0.25)
    if _safe_float(candidate.get("min_block_clearance_m"), 0.0) < min_clearance:
        blockers.append("blocked_candidate.min_block_clearance_m below threshold")
    if "blocked_route_replan" not in (candidate.get("reasons") or []):
        blockers.append("blocked_candidate missing blocked_route_replan reason")

    _extend_unique(blockers, [str(error) for error in report.get("errors") or []])
    return not blockers, blockers, {
        "synthetic_block": report.get("synthetic_block") or {},
        "thresholds": thresholds,
        "baseline_path_intersects_block": report.get("baseline_path_intersects_block"),
        "candidate_path_avoids_block": report.get("candidate_path_avoids_block"),
        "candidate_route_changed": report.get("candidate_route_changed"),
        "blocked_route_replanned": report.get("blocked_route_replanned"),
        "published": published,
        "phases": phase_evidence,
        "artifacts": report.get("artifacts") or {},
    }


def _eval_navigation_replay_deviation(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("schema_version") != "lingtu.navigation_replay_deviation_gate.v1":
        blockers.append("schema_version is not lingtu.navigation_replay_deviation_gate.v1")
    if report.get("ok") is not True:
        blockers.append("report.ok is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")

    thresholds = report.get("thresholds") if isinstance(report.get("thresholds"), dict) else {}
    min_sample_count = _safe_float(thresholds.get("min_sample_count"), 5.0)
    min_cmd_count = _safe_float(thresholds.get("min_cmd_vel_count"), 3.0)
    min_cmd_ratio = _safe_float(thresholds.get("min_cmd_vel_nonzero_ratio"), 0.2)
    min_odom_count = _safe_float(thresholds.get("min_odometry_count"), 5.0)
    min_odom_motion = _safe_float(thresholds.get("min_odom_motion_m"), 0.25)
    max_final_distance = _safe_float(thresholds.get("max_final_distance_m"), 0.8)
    max_tracking_p95 = _safe_float(thresholds.get("max_tracking_error_p95_m"), 0.6)
    max_tracking_final = _safe_float(thresholds.get("max_tracking_error_final_m"), 0.8)

    sample_count = _safe_float(report.get("sample_count"), 0.0)
    global_path_count = _safe_float(report.get("global_path_count"), 0.0)
    local_path_count = _safe_float(report.get("local_path_count"), 0.0)
    cmd_vel_count = _safe_float(report.get("cmd_vel_count"), 0.0)
    cmd_vel_nonzero = _safe_float(report.get("cmd_vel_nonzero"), 0.0)
    cmd_vel_nonzero_ratio = _safe_float(report.get("cmd_vel_nonzero_ratio"), 0.0)
    odometry_count = _safe_float(report.get("odometry_count"), 0.0)
    odom_motion_m = _safe_float(report.get("odom_motion_m"), 0.0)
    final_distance_m = _safe_float(report.get("final_distance_m"), 999.0)
    tracking_error_p95_m = _safe_float(report.get("tracking_error_p95_m"), 999.0)
    tracking_error_final_m = _safe_float(report.get("tracking_error_final_m"), 999.0)

    if sample_count < min_sample_count:
        blockers.append("sample_count below replay threshold")
    if global_path_count <= 0:
        blockers.append("global_path_count is missing")
    if local_path_count <= 0:
        blockers.append("local_path_count is missing")
    if cmd_vel_count < min_cmd_count:
        blockers.append("cmd_vel_count below replay threshold")
    if cmd_vel_nonzero <= 0:
        blockers.append("cmd_vel_nonzero is missing")
    if cmd_vel_nonzero_ratio < min_cmd_ratio:
        blockers.append("cmd_vel_nonzero_ratio below threshold")
    if odometry_count < min_odom_count:
        blockers.append("odometry_count below replay threshold")
    if odom_motion_m < min_odom_motion:
        blockers.append("odom_motion_m below threshold")
    if final_distance_m > max_final_distance:
        blockers.append("final_distance_m above threshold")
    if tracking_error_p95_m > max_tracking_p95:
        blockers.append("tracking_error_p95_m above threshold")
    if tracking_error_final_m > max_tracking_final:
        blockers.append("tracking_error_final_m above threshold")

    checks = report.get("deviation_checks") if isinstance(report.get("deviation_checks"), dict) else {}
    for name in (
        "data_presence",
        "command_replay",
        "odometry_replay",
        "goal_deviation",
        "tracking_deviation",
        "command_safety",
    ):
        check = checks.get(name) if isinstance(checks.get(name), dict) else {}
        if not check:
            blockers.append(f"deviation_checks.{name} is missing")
        elif check.get("ok") is not True:
            blocker = str(check.get("blocker") or "not ok")
            blockers.append(f"deviation_checks.{name}: {blocker}")

    for blocker in report.get("remaining_gaps") or []:
        if str(blocker) not in blockers:
            blockers.append(str(blocker))

    return not blockers, blockers, {
        "trace_source": report.get("trace_source"),
        "trace_kind": report.get("trace_kind"),
        "sample_count": report.get("sample_count"),
        "global_path_count": report.get("global_path_count"),
        "local_path_count": report.get("local_path_count"),
        "cmd_vel_count": report.get("cmd_vel_count"),
        "cmd_vel_nonzero": report.get("cmd_vel_nonzero"),
        "cmd_vel_nonzero_ratio": report.get("cmd_vel_nonzero_ratio"),
        "odometry_count": report.get("odometry_count"),
        "odom_motion_m": report.get("odom_motion_m"),
        "odom_displacement_m": report.get("odom_displacement_m"),
        "final_distance_m": report.get("final_distance_m"),
        "tracking_error_p95_m": report.get("tracking_error_p95_m"),
        "tracking_error_final_m": report.get("tracking_error_final_m"),
        "deviation_checks": checks,
    }


def _eval_gateway_runtime_acceptance(
    report: dict[str, Any],
) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    checks = report.get("checks") if isinstance(report.get("checks"), dict) else {}
    gateway_contract = (
        checks.get("gateway_contract")
        if isinstance(checks.get("gateway_contract"), dict)
        else {}
    )
    dataflow = (
        checks.get("module_first_dataflow")
        if isinstance(checks.get("module_first_dataflow"), dict)
        else {}
    )
    stage_evidence = (
        checks.get("stage_evidence")
        if isinstance(checks.get("stage_evidence"), dict)
        else {}
    )

    if report.get("schema_version") != "lingtu.gateway_runtime_acceptance.v1":
        blockers.append("schema_version is not lingtu.gateway_runtime_acceptance.v1")
    if report.get("ok") is not True:
        blockers.append("report.ok is not true")
    if report.get("mode") != "non_motion":
        blockers.append("mode is not non_motion")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")
    if report.get("ros2_topic_required") is not False:
        blockers.append("ros2_topic_required is not false")
    _extend_unique(blockers, [str(blocker) for blocker in report.get("blockers") or []])

    if gateway_contract.get("ok") is not True:
        blockers.append("gateway_contract.ok is not true")
    missing_links = [str(item) for item in gateway_contract.get("missing_links") or []]
    if missing_links:
        blockers.append("gateway contract missing links: " + ", ".join(missing_links))

    if dataflow.get("ok") is not True:
        blockers.append("module_first_dataflow.ok is not true")
    if dataflow.get("ros2_topic_required") is True:
        blockers.append("Gateway acceptance must not require ros2 topic")
    if dataflow.get("module_port_bus_primary") is not True:
        blockers.append("module_port_bus_primary is not true")
    if dataflow.get("ros2_adapter_primary") is True:
        blockers.append("ros2_adapter_primary is true")
    if dataflow.get("arbitrary_publish_supported") is not False:
        blockers.append("arbitrary_publish_supported is not false")
    if _safe_int(dataflow.get("command_interface_count")) <= 0:
        blockers.append("command_interface_count is missing")

    missing_command_interfaces = [
        str(item) for item in dataflow.get("missing_command_interfaces") or []
    ]
    unexpected_command_interfaces = [
        str(item) for item in dataflow.get("unexpected_command_interfaces") or []
    ]
    missing_topics = [str(item) for item in dataflow.get("missing_topics") or []]
    non_observable_topics = [
        str(item) for item in dataflow.get("non_observable_topics") or []
    ]
    missing_stream_interfaces = [
        str(item) for item in dataflow.get("missing_stream_interfaces") or []
    ]
    missing_live_topics = [
        str(item) for item in dataflow.get("missing_live_topics") or []
    ]
    if missing_command_interfaces:
        blockers.append(
            "missing Gateway command interfaces: "
            + ", ".join(missing_command_interfaces)
        )
    if unexpected_command_interfaces:
        blockers.append(
            "unexpected Gateway command interfaces: "
            + ", ".join(unexpected_command_interfaces)
        )
    if missing_topics:
        blockers.append("missing product runtime topics: " + ", ".join(missing_topics))
    if non_observable_topics:
        blockers.append(
            "non-observable product runtime topics: "
            + ", ".join(non_observable_topics)
        )
    if missing_stream_interfaces:
        blockers.append(
            "missing Gateway SSE stream interfaces: "
            + ", ".join(missing_stream_interfaces)
        )
    if missing_live_topics and report.get("mode") != "non_motion":
        blockers.append("missing live Module samples: " + ", ".join(missing_live_topics))

    if stage_evidence.get("ok") is not True:
        blockers.append("stage_evidence.ok is not true")
    if _safe_int(stage_evidence.get("stage_count")) <= 0:
        blockers.append("stage_evidence.stage_count is missing")
    missing_stages = [str(item) for item in stage_evidence.get("missing_stages") or []]
    not_live_stages = [str(item) for item in stage_evidence.get("not_live_stages") or []]
    missing_tokens = (
        stage_evidence.get("missing_tokens")
        if isinstance(stage_evidence.get("missing_tokens"), dict)
        else {}
    )
    if missing_stages:
        blockers.append("missing runtime stages: " + ", ".join(missing_stages))
    if not_live_stages and report.get("mode") != "non_motion":
        blockers.append("non-live runtime stages: " + ", ".join(not_live_stages))
    if missing_tokens and report.get("mode") != "non_motion":
        blockers.append(
            "runtime stages missing required inputs: "
            + ", ".join(sorted(str(key) for key in missing_tokens))
        )

    observable_topics = [str(item) for item in dataflow.get("observable_topics") or []]
    streamable_topics = [str(item) for item in dataflow.get("streamable_topics") or []]
    return not blockers, blockers, {
        "mode": report.get("mode"),
        "runtime_contract": report.get("runtime_contract")
        or dataflow.get("runtime_contract"),
        "ros2_topic_required": report.get("ros2_topic_required"),
        "module_port_bus_primary": dataflow.get("module_port_bus_primary"),
        "ros2_adapter_primary": dataflow.get("ros2_adapter_primary"),
        "arbitrary_publish_supported": dataflow.get("arbitrary_publish_supported"),
        "command_interface_count": dataflow.get("command_interface_count"),
        "observable_topic_count": len(observable_topics),
        "streamable_topic_count": len(streamable_topics),
        "stage_count": stage_evidence.get("stage_count"),
        "missing_links": missing_links,
        "missing_topics": missing_topics,
        "non_observable_topics": non_observable_topics,
        "missing_stream_interfaces": missing_stream_interfaces,
        "missing_command_interfaces": missing_command_interfaces,
        "unexpected_command_interfaces": unexpected_command_interfaces,
        "missing_stages": missing_stages,
        "missing_stage_tokens": missing_tokens,
        "not_live_stages": not_live_stages,
    }


def _eval_gazebo_runtime(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("ok") is not True:
        blockers.append("report.ok is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")
    if int(report.get("samples") or 0) <= 0:
        blockers.append("tf samples missing")
    if report.get("odometry_frame_id") != "odom":
        blockers.append("odometry_frame_id is not odom")
    if report.get("odometry_child_frame_id") != "body":
        blockers.append("odometry_child_frame_id is not body")

    frames = report.get("topic_frames") or {}
    samples = report.get("topic_samples") or {}
    point_counts = report.get("point_counts") or {}
    expected = {
        "/slam/map_cloud": "odom",
        "/slam/registered_cloud": "body",
    }
    for topic, frame in expected.items():
        if int(samples.get(topic) or 0) <= 0:
            blockers.append(f"{topic} samples missing")
        if frames.get(topic) != frame:
            blockers.append(f"{topic} frame is not {frame}")
    for topic in ("/slam/map_cloud", "/slam/registered_cloud"):
        if int(point_counts.get(topic) or 0) <= 0:
            blockers.append(f"{topic} points missing")

    nav_loop = report.get("nav_loop") or {}
    if nav_loop.get("ok") is not True:
        blockers.append("nav_loop.ok is not true")
    if nav_loop.get("simulation_only") is not True:
        blockers.append("nav_loop.simulation_only is not true")
    if not _bool_false(nav_loop, "real_robot_motion"):
        blockers.append("nav_loop.real_robot_motion is not false")
    if not _bool_false(nav_loop, "cmd_vel_sent_to_hardware"):
        blockers.append("nav_loop.cmd_vel_sent_to_hardware is not false")
    for key in (
        "goal_published",
        "odometry_seen",
        "global_path_seen",
        "local_path_seen",
        "cmd_vel_seen",
        "cmd_vel_nonzero",
    ):
        if nav_loop.get(key) is not True:
            blockers.append(f"nav_loop.{key} is not true")
    odom_delta_m = _safe_float(nav_loop.get("odom_delta_m"), default=0.0)
    if odom_delta_m < 0.05:
        blockers.append("nav_loop.odom_delta_m < 0.05")
    nav_samples = nav_loop.get("samples") or nav_loop.get("topic_samples") or {}
    for topic in ("/nav/global_path", "/nav/local_path", "/nav/cmd_vel", "/slam/odometry"):
        if int(nav_samples.get(topic) or 0) <= 0:
            blockers.append(f"nav_loop {topic} samples missing")
    publisher_contract = nav_loop.get("publisher_contract") or {}
    if publisher_contract.get("ok") is not True:
        blockers.append("nav_loop.publisher_contract.ok is not true")
    for error in publisher_contract.get("errors") or []:
        blockers.append(f"nav_loop publisher contract: {error}")

    frontier = report.get("frontier_exploration") or {}
    if frontier.get("ok") is not True:
        blockers.append("frontier_exploration.ok is not true")
    if frontier.get("simulation_only") is not True:
        blockers.append("frontier_exploration.simulation_only is not true")
    if not _bool_false(frontier, "real_robot_motion"):
        blockers.append("frontier_exploration.real_robot_motion is not false")
    if not _bool_false(frontier, "cmd_vel_sent_to_hardware"):
        blockers.append("frontier_exploration.cmd_vel_sent_to_hardware is not false")
    for key in (
        "frontier_started",
        "frontier_goal_seen",
        "frontier_goal_published",
        "odometry_seen",
        "map_cloud_seen",
        "global_path_seen",
        "local_path_seen",
        "cmd_vel_seen",
        "cmd_vel_nonzero",
    ):
        if frontier.get(key) is not True:
            blockers.append(f"frontier_exploration.{key} is not true")
    frontier_odom_delta = _safe_float(frontier.get("odom_delta_m"), default=0.0)
    if frontier_odom_delta < 0.05:
        blockers.append("frontier_exploration.odom_delta_m < 0.05")
    frontier_odom_delta_x = _safe_float(frontier.get("odom_delta_x_m"), default=0.0)
    if frontier_odom_delta_x < 0.05:
        blockers.append("frontier_exploration.odom_delta_x_m < 0.05")
    frontier_area_delta = _safe_float(frontier.get("explored_area_delta_m2"), default=0.0)
    if frontier_area_delta <= 0.0:
        blockers.append("frontier_exploration.explored_area_delta_m2 <= 0")
    if int(frontier.get("known_cells_delta") or 0) <= 0:
        blockers.append("frontier_exploration.known_cells_delta <= 0")
    if int(frontier.get("frontier_count_max") or 0) <= 0:
        blockers.append("frontier_exploration.frontier_count_max <= 0")
    frontier_goal = frontier.get("frontier_goal") or []
    if not isinstance(frontier_goal, list) or len(frontier_goal) < 2:
        blockers.append("frontier_exploration.frontier_goal missing")
    elif _safe_float(frontier_goal[0], default=0.0) < 0.75:
        blockers.append("frontier_exploration.frontier_goal.x < 0.75")
    frontier_samples = frontier.get("samples") or {}
    for topic in (
        "/nav/goal_pose",
        "/nav/global_path",
        "/nav/local_path",
        "/nav/cmd_vel",
        "/slam/odometry",
        "/nav/terrain_map",
        "/nav/terrain_map_ext",
    ):
        if int(frontier_samples.get(topic) or 0) <= 0:
            blockers.append(f"frontier_exploration {topic} samples missing")
    if frontier.get("terrain_map_seen") is not True:
        blockers.append("frontier_exploration.terrain_map_seen is not true")
    if frontier.get("terrain_map_ext_seen") is not True:
        blockers.append("frontier_exploration.terrain_map_ext_seen is not true")
    trajectory_quality = frontier.get("trajectory_quality") or {}
    if trajectory_quality.get("ok") is not True:
        blockers.append("frontier_exploration.trajectory_quality.ok is not true")
    if int(trajectory_quality.get("room_violation_count") or 0) > 0:
        blockers.append("frontier_exploration trajectory left room")
    if float(trajectory_quality.get("max_out_of_room_m") or 0.0) > 0.05:
        blockers.append("frontier_exploration max_out_of_room_m > 0.05")
    if int(trajectory_quality.get("local_path_occupied_overlap_count") or 0) > 0:
        blockers.append("frontier_exploration local_path occupied overlap")
    clearance = trajectory_quality.get("min_obstacle_clearance_m")
    if clearance is None or float(clearance) < 0.18:
        blockers.append("frontier_exploration min_obstacle_clearance_m < 0.18")
    topic_sync = frontier.get("topic_sync") or {}
    if topic_sync.get("ok") is not True:
        blockers.append("frontier_exploration.topic_sync.ok is not true")
    if _safe_float(topic_sync.get("max_cloud_odom_skew_ms"), default=999999.0) > 250.0:
        blockers.append("frontier_exploration cloud/odom skew > 250 ms")
    frontier_stall = frontier.get("frontier_no_gain_stall")
    if not isinstance(frontier_stall, dict):
        frontier_stall = {}
    if frontier_stall.get("checked") is not True:
        blockers.append("frontier_exploration.frontier_no_gain_stall.checked is not true")
    if frontier_stall.get("ok") is not True:
        blockers.append("frontier_exploration.frontier_no_gain_stall.ok is not true")
    if str(frontier_stall.get("stop_reason") or "") != "post_pass_observation_elapsed":
        blockers.append(
            "frontier_exploration.frontier_no_gain_stall.stop_reason is not post_pass_observation_elapsed"
        )
    required_observation_s = _safe_float(frontier_stall.get("required_observation_s"), default=0.0)
    observed_s = _safe_float(frontier_stall.get("observed_s"), default=0.0)
    if required_observation_s <= 0.0:
        blockers.append("frontier_exploration.frontier_no_gain_stall.required_observation_s <= 0")
    if observed_s + 1e-6 < required_observation_s:
        blockers.append("frontier_exploration.frontier_no_gain_stall.observed_s below required")
    cumulative = frontier.get("cumulative_map_cloud") or {}
    if frontier.get("cumulative_map_cloud_seen") is not True:
        blockers.append("frontier_exploration.cumulative_map_cloud_seen is not true")
    if "odom" not in set(cumulative.get("frame_ids") or []):
        blockers.append("frontier_exploration cumulative map frame_id missing odom")
    if int(cumulative.get("samples") or 0) < 8:
        blockers.append("frontier_exploration cumulative map samples < 8")
    if int(cumulative.get("unique_voxels_delta") or 0) <= 0:
        blockers.append("frontier_exploration cumulative unique_voxels_delta <= 0")
    if float(cumulative.get("retention_min") or 0.0) < 0.6:
        blockers.append("frontier_exploration cumulative retention_min < 0.6")
    registered = frontier.get("registered_cloud") or {}
    if float(registered.get("map_vs_registered_voxel_ratio") or 0.0) < 1.5:
        blockers.append("frontier_exploration cumulative map_vs_registered_voxel_ratio < 1.5")
    static = frontier.get("static_obstacles") or {}
    if not any(
        int(item.get("samples") or 0) >= 2
        and _safe_float(item.get("centroid_drift_max_m"), default=999.0) <= 0.25
        for item in static.values()
    ):
        blockers.append("frontier_exploration stable static obstacle ROI missing")

    tare = report.get("tare_exploration") or {}
    if tare.get("ok") is not True:
        blockers.append("tare_exploration.ok is not true")
    if tare.get("simulation_only") is not True:
        blockers.append("tare_exploration.simulation_only is not true")
    if not _bool_false(tare, "real_robot_motion"):
        blockers.append("tare_exploration.real_robot_motion is not false")
    if not _bool_false(tare, "cmd_vel_sent_to_hardware"):
        blockers.append("tare_exploration.cmd_vel_sent_to_hardware is not false")
    if tare.get("source_contract_ok") is not True:
        blockers.append("tare_exploration.source_contract_ok is not true")
    if tare.get("backend") != "tare":
        blockers.append("tare_exploration.backend is not tare")
    if tare.get("runtime_required") is True and tare.get("runtime_available") is not True:
        blockers.append("tare_exploration runtime required but unavailable")

    return not blockers, blockers, {
        "samples": report.get("samples"),
        "topic_samples": samples,
        "topic_frames": frames,
        "point_counts": point_counts,
        "nav_loop": {
            "ok": nav_loop.get("ok"),
            "goal_published": nav_loop.get("goal_published"),
            "odometry_seen": nav_loop.get("odometry_seen"),
            "global_path_seen": nav_loop.get("global_path_seen"),
            "local_path_seen": nav_loop.get("local_path_seen"),
            "cmd_vel_seen": nav_loop.get("cmd_vel_seen"),
            "cmd_vel_nonzero": nav_loop.get("cmd_vel_nonzero"),
            "odom_delta_m": nav_loop.get("odom_delta_m"),
            "odom_start_xy": nav_loop.get("odom_start_xy"),
            "odom_last_xy": nav_loop.get("odom_last_xy"),
            "topic_samples": nav_samples,
            "publisher_contract": publisher_contract,
        },
        "frontier_exploration": {
            "ok": frontier.get("ok"),
            "frontier_started": frontier.get("frontier_started"),
            "frontier_goal": frontier.get("frontier_goal"),
            "frontier_goal_published": frontier.get("frontier_goal_published"),
            "frontier_count_max": frontier.get("frontier_count_max"),
            "odom_delta_m": frontier.get("odom_delta_m"),
            "known_cells_delta": frontier.get("known_cells_delta"),
            "explored_area_delta_m2": frontier.get("explored_area_delta_m2"),
            "cumulative_map_cloud": cumulative,
            "registered_cloud": registered,
            "static_obstacles": static,
            "trajectory_quality": trajectory_quality,
            "topic_sync": topic_sync,
            "frontier_no_gain_stall": frontier_stall,
            "topic_samples": frontier_samples,
        },
        "tare_exploration": {
            "ok": tare.get("ok"),
            "backend": tare.get("backend"),
            "source_contract_ok": tare.get("source_contract_ok"),
            "runtime_required": tare.get("runtime_required"),
            "runtime_available": tare.get("runtime_available"),
            "gazebo_runtime_verified": tare.get("gazebo_runtime_verified"),
        },
    }


def _eval_cmu_unity_sim(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("schema_version") != "lingtu.cmu_unity_sim_gate.v1":
        blockers.append("schema_version is not lingtu.cmu_unity_sim_gate.v1")
    if report.get("ok") is not True:
        blockers.append("report.ok is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")
    if report.get("runtime_executed") is not False:
        blockers.append("runtime_executed is not false")

    checks = list(report.get("checks") or [])
    check_by_name = {str(item.get("name")): item for item in checks}
    required_checks = (
        "host_ros_humble_setup",
        "host_ros2_cli",
        "host_ros2_cli_functional",
        "host_colcon_cli",
        "host_colcon_cli_functional",
        "cmu_workspace_exists",
        "cmu_git_workspace",
        "cmu_humble_branch",
        "cmu_required_source_paths",
        "cmu_unity_environment_assets",
        "cmu_colcon_build_output",
        "cmu_topic_contract",
        "lingtu_tare_remap_contract",
        "lingtu_tare_explore_profile",
        "lingtu_cmu_tare_profile",
        "lingtu_cmu_adapter_exists",
        "lingtu_cmu_adapter_relay_contract",
        "lingtu_cmu_adapter_safety_contract",
    )
    for name in required_checks:
        if (check_by_name.get(name) or {}).get("ok") is not True:
            blockers.append(f"{name} is not true")

    cmu = report.get("cmu_workspace") or {}
    topic_contract = cmu.get("topic_contract") or {}
    for topic in (
        "/registered_scan",
        "/state_estimation",
        "/terrain_map",
        "/terrain_map_ext",
        "/way_point",
        "/cmd_vel",
    ):
        if topic_contract.get(topic) is not True:
            blockers.append(f"cmu topic {topic} missing")

    lingtu = report.get("lingtu_contract") or {}
    remaps = lingtu.get("remaps") or {}
    if remaps.get("/registered_scan") != "/slam/map_cloud":
        blockers.append("LingTu /registered_scan remap is not /slam/map_cloud")
    if remaps.get("/state_estimation") != "/slam/odometry":
        blockers.append("LingTu /state_estimation remap is not /slam/odometry")
    if remaps.get("/state_estimation_at_scan") != "/slam/odometry":
        blockers.append("LingTu /state_estimation_at_scan remap is not /slam/odometry")
    if remaps.get("/way_point") != "/exploration/way_point":
        blockers.append("LingTu /way_point remap is not /exploration/way_point")

    adapter_required_relays = lingtu.get("adapter_required_relays") or {}
    for relay in (
        "/state_estimation->/slam/odometry",
        "/state_estimation_at_scan->/slam/state_at_scan",
        "/registered_scan->/slam/map_cloud",
        "/terrain_map->/nav/terrain_map",
        "/terrain_map_ext->/nav/terrain_map_ext",
        "/way_point->/exploration/way_point",
        "/exploration/start->/start_exploration",
        "/nav/cmd_vel->/cmd_vel",
    ):
        if relay not in adapter_required_relays:
            blockers.append(f"LingTu adapter relay {relay} missing")

    return not blockers, blockers, {
        "workspace": cmu.get("path"),
        "branch": cmu.get("branch"),
        "head": cmu.get("head"),
        "remote": cmu.get("remote"),
        "checks": {str(item.get("name")): bool(item.get("ok")) for item in checks},
        "blockers": report.get("blockers") or [],
    }


def _eval_cmu_unity_runtime(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("schema_version") != "lingtu.cmu_unity_runtime_gate.v1":
        blockers.append("schema_version is not lingtu.cmu_unity_runtime_gate.v1")
    if report.get("ok") is not True:
        blockers.append("report.ok is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")
    if report.get("runtime_executed") is not True:
        blockers.append("runtime_executed is not true")
    runtime_evidence_blockers, shared_runtime_evidence = _shared_runtime_evidence(
        report,
        expected_contract="cmu_unity_external",
        require_frame_links=True,
        require_data_flow=True,
    )
    _extend_unique(blockers, runtime_evidence_blockers)
    if str(report.get("ros_domain_id") or "") in {"", "0"}:
        blockers.append("ROS_DOMAIN_ID is empty or default")
    contract_blockers, runtime_contract = _runtime_contract_blockers(
        report,
        expected_name="cmu_unity_external",
    )
    _extend_unique(blockers, contract_blockers)

    thresholds = report.get("thresholds") or {}
    waypoints = report.get("waypoints") or {}
    min_waypoint_samples = int(thresholds.get("min_waypoint_samples") or 1)
    if int((waypoints.get("/way_point") or {}).get("samples") or 0) < min_waypoint_samples:
        blockers.append("/way_point samples below threshold")
    waypoint_unique_requirements = report.get("waypoint_unique_requirements") or {}
    for topic, requirement in waypoint_unique_requirements.items():
        if requirement.get("ok") is not True:
            blockers.append(f"{topic} unique waypoint requirement failed")

    cmd = report.get("cmd_vel") or {}
    if int(cmd.get("nonzero_samples") or 0) < int(thresholds.get("min_cmd_vel_samples") or 3):
        blockers.append("/nav/cmd_vel nonzero_samples below threshold")
    if _safe_float(cmd.get("max_norm"), default=0.0) < _safe_float(thresholds.get("min_cmd_vel"), default=0.01):
        blockers.append("/nav/cmd_vel max_norm below threshold")

    odometry = report.get("odometry") or {}
    odom_delta = max(
        (_safe_float((item or {}).get("delta_m"), default=0.0) for item in odometry.values()),
        default=0.0,
    )
    if odom_delta < _safe_float(thresholds.get("min_odom_delta_m"), default=0.10):
        blockers.append("odom delta below threshold")

    cloud = report.get("cloud_coverage") or {}
    if _safe_float(cloud.get("best_area_delta_m2"), default=0.0) < _safe_float(
        thresholds.get("min_map_area_delta_m2"),
        default=0.5,
    ):
        blockers.append("map/exploration area delta below threshold")

    hardware = report.get("hardware_safety") or {}
    if hardware.get("blocked_hardware_nodes"):
        blockers.append("hardware-looking command subscriber present")

    tare_navigation = report.get("tare_navigation") or {}
    if int(tare_navigation.get("success_count") or 0) < 1:
        blockers.append("TARE navigation success_count below threshold")
    if int(tare_navigation.get("failure_count") or 0) > 0:
        blockers.append("TARE navigation failure_count is nonzero")

    tare_strategy_quality = report.get("tare_strategy_quality")
    if not isinstance(tare_strategy_quality, dict):
        tare_strategy_quality = {}
        blockers.append("tare_strategy_quality missing")
    if tare_strategy_quality.get("checked") is not True:
        blockers.append("tare_strategy_quality.checked is not true")
    if tare_strategy_quality.get("ok") is not True:
        blockers.append("tare_strategy_quality.ok is not true")
    tare_strategy_thresholds = tare_strategy_quality.get("thresholds") or {}
    min_tare_waypoints = int(tare_strategy_thresholds.get("min_tare_waypoints") or 1)
    min_tare_paths = int(tare_strategy_thresholds.get("min_tare_paths") or 1)
    min_tare_strategy_paths = int(
        tare_strategy_thresholds.get("min_tare_strategy_paths") or 0
    )
    min_tare_successes = int(
        tare_strategy_thresholds.get("min_tare_navigation_successes") or 1
    )
    if int(tare_strategy_quality.get("waypoint_count") or 0) < min_tare_waypoints:
        blockers.append("tare_strategy_quality.waypoint_count below threshold")
    if int(tare_strategy_quality.get("path_count") or 0) < min_tare_paths:
        blockers.append("tare_strategy_quality.path_count below threshold")
    if int(tare_strategy_quality.get("strategy_path_count") or 0) < min_tare_strategy_paths:
        blockers.append("tare_strategy_quality.strategy_path_count below threshold")
    if int(tare_strategy_quality.get("navigation_success_count") or 0) < min_tare_successes:
        blockers.append("tare_strategy_quality.navigation_success_count below threshold")
    if int(tare_strategy_quality.get("navigation_failure_count") or 0) > 0:
        blockers.append("tare_strategy_quality.navigation_failure_count is nonzero")
    for blocker in tare_strategy_quality.get("blockers") or []:
        blockers.append(f"tare_strategy_quality: {blocker}")

    frontier_stall = report.get("frontier_no_gain_stall")
    if not isinstance(frontier_stall, dict):
        frontier_stall = {}
        blockers.append("frontier_no_gain_stall missing")
    if frontier_stall.get("checked") is not True:
        blockers.append("frontier_no_gain_stall.checked is not true")
    if frontier_stall.get("ok") is not True:
        blockers.append("frontier_no_gain_stall.ok is not true")
    if frontier_stall.get("mode") != "late_activity_observation":
        blockers.append("frontier_no_gain_stall.mode is not late_activity_observation")
    if frontier_stall.get("stop_reason") != "late_activity_window_verified":
        blockers.append("frontier_no_gain_stall.stop_reason is not late_activity_window_verified")
    required_observation_s = _safe_float(
        frontier_stall.get("required_observation_s"),
        default=0.0,
    )
    observed_s = _safe_float(frontier_stall.get("observed_s"), default=0.0)
    if required_observation_s <= 0.0:
        blockers.append("frontier_no_gain_stall.required_observation_s <= 0")
    if observed_s < required_observation_s:
        blockers.append("frontier_no_gain_stall.observed_s below required")
    late_activity_stall = frontier_stall.get("late_activity") or {}
    for name in ("odometry_ok", "cmd_vel_ok", "paths_ok"):
        if late_activity_stall.get(name) is not True:
            blockers.append(f"frontier_no_gain_stall.late_activity.{name} is not true")
    if not (
        late_activity_stall.get("map_growth_ok") is True
        or late_activity_stall.get("map_growth_accepted_flat_after_total_growth") is True
    ):
        blockers.append("frontier_no_gain_stall.late_activity.map_growth is not accepted")

    return not blockers, blockers, {
        "ros_domain_id": report.get("ros_domain_id"),
        "waypoints": waypoints,
        "waypoint_unique_requirements": waypoint_unique_requirements,
        "cmd_vel": cmd,
        "odometry_delta_m": odom_delta,
        "motion_progress": report.get("motion_progress") or {},
        "progress_requirements": report.get("progress_requirements") or {},
        "cloud_coverage": cloud,
        "hardware_safety": hardware,
        "runtime_contract": runtime_contract,
        "runtime_evidence": shared_runtime_evidence,
        "tare_navigation": tare_navigation,
        "tare_strategy_quality": tare_strategy_quality,
        "frontier_no_gain_stall": frontier_stall,
        "blockers": report.get("blockers") or [],
    }


def _eval_cmu_unity_pct_strict(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    _, blockers, runtime_evidence = _eval_cmu_unity_runtime(report)
    blockers = list(blockers)

    if report.get("cmd_vel_exclusive_to_lingtu") is not True:
        blockers.append("cmd_vel_exclusive_to_lingtu is not true")

    planner = report.get("planner_diagnostics") or {}
    if planner.get("available") is not True:
        blockers.append("planner diagnostics unavailable")
    if planner.get("primary_planner") != "pct":
        blockers.append("primary_planner is not pct")
    if planner.get("selected_planner") != "pct":
        blockers.append("selected_planner is not pct")
    if planner.get("fallback_used") is not False:
        blockers.append("planner fallback was used")
    if int(planner.get("rejected_plan_count") or 0) != 0:
        blockers.append("planner rejected_plan_count is not 0")
    if planner.get("reached_goal") is not True:
        blockers.append("PCT did not report reached_goal")

    navigation_failure = report.get("navigation_failure") or {}
    if navigation_failure.get("failed") is True:
        blockers.append("navigation mission failed")
    if str(navigation_failure.get("state") or "").upper() in {"FAILED", "STUCK"}:
        blockers.append("navigation state is failed/stuck")

    direct_goal_fallback = report.get("direct_goal_fallback") or {}
    if direct_goal_fallback.get("used") is True:
        blockers.append("direct goal fallback was used")

    path_requirements = report.get("path_requirements") or {}
    paths = report.get("paths") or {}
    for topic in ("/nav/global_path", "/nav/local_path"):
        req = path_requirements.get(topic) or {}
        item = paths.get(topic) or {}
        observed_nonempty = int(req.get("observed_nonempty_samples") or item.get("nonempty_samples") or 0)
        observed_poses = int(req.get("observed_max_poses") or item.get("max_poses") or 0)
        if req.get("ok") is not True or observed_nonempty <= 0 or observed_poses < 2:
            blockers.append(f"{topic} strict path requirement failed")

    scan_requirements = report.get("scan_requirements") or {}
    registered_scan = scan_requirements.get("/slam/registered_cloud") or {}
    if registered_scan.get("ok") is not True or int(registered_scan.get("observed_samples") or 0) <= 0:
        blockers.append("/slam/registered_cloud strict scan requirement failed")

    map_requirements = report.get("map_requirements") or {}
    map_cloud = map_requirements.get("/slam/map_cloud") or {}
    if map_cloud.get("ok") is not True or _safe_float(map_cloud.get("observed_area_delta_m2"), default=0.0) <= 0.0:
        blockers.append("/slam/map_cloud strict map-growth requirement failed")
    terrain_map = map_requirements.get("/nav/terrain_map_ext") or {}
    if terrain_map.get("ok") is not True or _safe_float(terrain_map.get("observed_area_delta_m2"), default=0.0) <= 0.0:
        blockers.append("/nav/terrain_map_ext strict map-growth requirement failed")

    hardware = report.get("hardware_safety") or {}
    if hardware.get("unexpected_command_publishers"):
        blockers.append("unexpected /cmd_vel publisher present")

    return not blockers, blockers, {
        **runtime_evidence,
        "planner_diagnostics": {
            "available": planner.get("available"),
            "primary_planner": planner.get("primary_planner"),
            "selected_planner": planner.get("selected_planner"),
            "policy": planner.get("policy"),
            "fallback_used": planner.get("fallback_used"),
            "fallback_reason": planner.get("fallback_reason"),
            "rejected_plan_count": planner.get("rejected_plan_count"),
            "reached_goal": planner.get("reached_goal"),
        },
        "cmd_vel_exclusive_to_lingtu": report.get("cmd_vel_exclusive_to_lingtu"),
        "path_requirements": path_requirements,
        "scan_requirements": scan_requirements,
        "map_requirements": map_requirements,
        "navigation_failure": navigation_failure,
        "direct_goal_fallback": direct_goal_fallback,
    }


def _eval_saved_map_relocalize(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("ok") is not True:
        blockers.append("report.ok is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")
    if report.get("validation_level") != "runtime_relocalization":
        blockers.append("validation_level is not runtime_relocalization")
    if report.get("runtime_stage") != "saved_map_relocalization":
        blockers.append("runtime_stage is not saved_map_relocalization")
    if report.get("map_dependency") != "saved_map_required":
        blockers.append("map_dependency is not saved_map_required")
    if report.get("requires_saved_map") is not True:
        blockers.append("requires_saved_map is not true")
    if report.get("requires_live_slam") is not True:
        blockers.append("requires_live_slam is not true")
    if report.get("runtime_relocalization_executed") is not True:
        blockers.append("runtime_relocalization_executed is not true")
    if report.get("runtime_relocalization_validated") is not True:
        blockers.append("runtime_relocalization_validated is not true")

    map_pcd_artifact = _verify_file_artifact(
        report,
        report.get("map_pcd"),
        blockers,
        prefix="saved_map_relocalize map_pcd",
    )
    map_metadata_contract = (
        report.get("map_metadata_contract")
        if isinstance(report.get("map_metadata_contract"), dict)
        else {}
    )
    map_metadata_checks = (
        map_metadata_contract.get("checks")
        if isinstance(map_metadata_contract.get("checks"), dict)
        else {}
    )
    if map_metadata_contract.get("ok") is not True:
        blockers.append("map_metadata_contract.ok is not true")
    for name, ok in map_metadata_checks.items():
        if ok is not True:
            blockers.append(f"map_metadata_contract.{name} is not true")

    service = report.get("service") if isinstance(report.get("service"), dict) else {}
    global_reloc = report.get("global_relocalization_requested") is True
    if service.get("available") is not True:
        blockers.append(
            "/slam/global_relocalize service was not available"
            if global_reloc else "/slam/relocalize service was not available"
        )
    if service.get("success") is not True:
        message = str(service.get("message") or "").lower()
        if not (global_reloc and "already running" in message):
            blockers.append(
                "/slam/global_relocalize service did not succeed"
                if global_reloc else "/slam/relocalize service did not succeed"
            )

    live_feed = report.get("live_feed") if isinstance(report.get("live_feed"), dict) else {}
    if live_feed.get("ok") is not True:
        blockers.append("live_feed.ok is not true")
    if (live_feed.get("fastlio2_z_consistency") or {}).get("ok") is not True:
        blockers.append("live_feed.fastlio2_z_consistency.ok is not true")
    outputs = live_feed.get("outputs") if isinstance(live_feed.get("outputs"), dict) else {}
    for key in ("fastlio2_odometry", "fastlio2_cloud_registered", "fastlio2_cloud_map"):
        if int(outputs.get(key) or 0) <= 0:
            blockers.append(f"live_feed.outputs.{key} missing")

    localizer = report.get("localizer") if isinstance(report.get("localizer"), dict) else {}
    thresholds = report.get("thresholds") if isinstance(report.get("thresholds"), dict) else {}
    min_points = int(thresholds.get("min_saved_map_points") or 1000)
    min_tracking = int(thresholds.get("min_tracking_health_samples") or 1)
    max_xy = _safe_float(thresholds.get("max_map_odom_xy_m"), default=5.0)
    max_z = _safe_float(thresholds.get("max_map_odom_z_abs_m"), default=2.0)
    if int(localizer.get("saved_map_cloud_samples") or 0) <= 0:
        blockers.append("/slam/saved_map_cloud samples missing")
    if int(localizer.get("saved_map_cloud_points_latest") or 0) < min_points:
        blockers.append("/slam/saved_map_cloud point count below threshold")
    if int(localizer.get("tracking_health_samples") or 0) < min_tracking:
        blockers.append("localizer tracking health samples below threshold")
    if str(localizer.get("latest_health_state") or "") not in {"LOCKED", "RECOVERED"}:
        blockers.append("localizer latest health is not LOCKED/RECOVERED")
    if int(localizer.get("map_to_odom_tf_samples") or 0) <= 0:
        blockers.append("map->odom TF samples missing")
    map_xy = _safe_float(localizer.get("map_to_odom_xy_m"), default=999.0)
    if map_xy > max_xy:
        blockers.append("map->odom XY correction exceeds threshold")
    map_z = _safe_float(localizer.get("map_to_odom_z_abs_m"), default=999.0)
    if map_z > max_z:
        blockers.append("map->odom Z correction exceeds threshold")
    if global_reloc:
        if report.get("global_relocalization_validated") is not True:
            blockers.append("global_relocalization_validated is not true")
        if localizer.get("bbs3d_success_observed") is not True:
            blockers.append("BBS3D success was not observed")
        if localizer.get("bbs3d_disabled_observed") is True:
            blockers.append("BBS3D disabled state was observed")
        if int(localizer.get("lost_health_samples") or 0) <= 0:
            blockers.append("kidnapped localizer did not report LOST before recovery")
        min_global_xy = _safe_float(thresholds.get("min_global_map_odom_xy_m"), default=0.0)
        if min_global_xy > 0.0 and map_xy < min_global_xy:
            blockers.append("kidnapped map->odom XY correction below threshold")

    return not blockers, blockers, {
        "validation_level": report.get("validation_level"),
        "runtime_stage": report.get("runtime_stage"),
        "map_dependency": report.get("map_dependency"),
        "runtime_relocalization_executed": report.get("runtime_relocalization_executed"),
        "runtime_relocalization_validated": report.get("runtime_relocalization_validated"),
        "global_relocalization_requested": global_reloc,
        "global_relocalization_validated": report.get("global_relocalization_validated"),
        "service": service,
        "live_feed": live_feed,
        "localizer": localizer,
        "thresholds": thresholds,
        "map_pcd": report.get("map_pcd"),
        "artifacts": {
            "map_pcd": map_pcd_artifact,
            "map_metadata_contract": map_metadata_contract,
        },
    }


def _eval_multifloor_exploration(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("passed") is not True:
        blockers.append("report.passed is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if report.get("cmd_vel_sent_to_hardware") is not False:
        blockers.append("cmd_vel_sent_to_hardware is not false")
    if report.get("suite") != "multifloor_route_matrix":
        blockers.append("suite is not multifloor_route_matrix")
    if report.get("validation_level") != "kinematic_module_ports":
        blockers.append("validation_level is not kinematic_module_ports")
    if report.get("local_planner_backend_verified") not in {"nanobind", "cmu", "cmu_py"}:
        blockers.append("production local planner backend was not verified")
    if report.get("production_local_planner_required") is not True:
        blockers.append("production_local_planner_required is not true")
    if report.get("production_local_planner_verified") is not True:
        blockers.append("production_local_planner_verified is not true")
    if report.get("frontier_loop_enabled") is not True:
        blockers.append("frontier_loop_enabled is not true")

    required_routes = {"same_floor", "lower_approach", "upper_floor", "cross_floor"}
    routes = set(report.get("routes") or [])
    missing_routes = sorted(required_routes - routes)
    if missing_routes:
        blockers.append(f"missing routes: {', '.join(missing_routes)}")

    exploration = report.get("exploration") or {}
    if exploration.get("ok") is not True:
        blockers.append("exploration.ok is not true")
    if exploration.get("closed_loop") is not True:
        blockers.append("exploration.closed_loop is not true")
    if exploration.get("probe_mode") != "frontier_navigation_closed_loop":
        blockers.append("exploration probe is not frontier_navigation_closed_loop")
    rounds = list(exploration.get("rounds") or [])
    if not rounds:
        blockers.append("frontier closed-loop rounds missing")
    for idx, item in enumerate(rounds):
        if not (item.get("goal") or {}).get("frontier"):
            blockers.append(f"frontier round {idx} missing frontier goal")
        if (item.get("command_flow") or {}).get("ok") is not True:
            blockers.append(f"frontier round {idx} command_flow failed")
        if (item.get("tracking_replay") or {}).get("ok") is not True:
            blockers.append(f"frontier round {idx} tracking_replay failed")

    cases = list(report.get("cases") or [])
    if len(cases) < len(required_routes):
        blockers.append("case_count is lower than required route count")
    native_pct_runtime_blocked = int(report.get("native_pct_blocked_count") or 0) > 0
    for case in cases:
        route = str(case.get("route") or "")
        if case.get("passed") is not True:
            blockers.append(f"{route or 'unknown'} case did not pass")
        if (case.get("lidar_localization") or {}).get("ok") is not True:
            blockers.append(f"{route or 'unknown'} lidar_localization failed")
        native_gate = case.get("native_pct_gate") or {}
        if native_gate.get("ok") is not True:
            blockers.append(f"{route or 'unknown'} native_pct_gate failed")
        runtime = native_gate.get("runtime") or native_gate.get("native_runtime") or {}
        runtime_error = str(runtime.get("error") or native_gate.get("reason") or "").lower()
        if (
            native_gate.get("status") == "blocked"
            and (
                runtime.get("ok") is False
                or "no runnable pct native modules" in runtime_error
                or "pct planner unavailable" in runtime_error
            )
        ):
            native_pct_runtime_blocked = True
        if (case.get("command_flow") or {}).get("ok") is not True:
            blockers.append(f"{route or 'unknown'} command_flow failed")
        if (case.get("tracking_replay") or {}).get("ok") is not True:
            blockers.append(f"{route or 'unknown'} tracking_replay failed")
        if (case.get("command_flow") or {}).get("cmd_vel_sent_to_hardware") is not False:
            blockers.append(f"{route or 'unknown'} command_flow did not report hardware-safe cmd_vel")
        if (case.get("tracking_replay") or {}).get("cmd_vel_sent_to_hardware") is not False:
            blockers.append(f"{route or 'unknown'} tracking_replay did not report hardware-safe cmd_vel")
    if "cross_floor" in routes:
        cross_floor = next((case for case in cases if case.get("route") == "cross_floor"), {})
        gate = cross_floor.get("native_pct_gate") or {}
        if gate.get("floor_graph_composition_verified") is not True:
            blockers.append("cross_floor floor-graph composition not verified")
        if int(gate.get("native_pct_feasible_segments") or 0) < 2:
            blockers.append("cross_floor does not have two feasible native PCT segments")
    if native_pct_runtime_blocked and "PCT native runtime unavailable" not in blockers:
        blockers.append("PCT native runtime unavailable")

    return not blockers, blockers, {
        "case_count": len(cases),
        "routes": sorted(routes),
        "local_planner_backend_verified": report.get("local_planner_backend_verified"),
        "production_local_planner_verified": report.get("production_local_planner_verified"),
        "frontier_loop_enabled": report.get("frontier_loop_enabled"),
        "frontier_rounds": len(rounds),
        "native_pct_gate_passed_count": report.get("native_pct_gate_passed_count"),
        "native_pct_blocked_count": report.get("native_pct_blocked_count"),
        "native_pct_runtime_blocked": native_pct_runtime_blocked,
        "validation_level": report.get("validation_level"),
        "algorithm_backends": _extract_algorithm_backends(report),
    }


GATES: tuple[GateSpec, ...] = (
    GateSpec(
        "gateway_runtime_acceptance",
        "Gateway Runtime Data Plane acceptance through ModulePort streams, stage evidence, and command whitelist",
        (
            "artifacts/server_sim_closure/gateway_runtime_acceptance/report.json",
            "artifacts/gateway_runtime_acceptance*/report.json",
        ),
        "PYTHONPATH=src:. python3 lingtu.py gateway-runtime-acceptance "
        "--acceptance-mode non_motion "
        "--json-out artifacts/server_sim_closure/gateway_runtime_acceptance/report.json",
        _eval_gateway_runtime_acceptance,
        host_requirements=LOCAL_NON_MOTION_HOST_REQUIREMENTS,
    ),
    GateSpec(
        "multifloor_exploration",
        "Multi-floor route matrix with LiDAR localization contract, local planning, tracking, and frontier closed-loop",
        (
            "artifacts/server_sim_closure/multifloor_exploration/report.json",
            "artifacts/multifloor_nav_validation*/report.json",
        ),
        "PYTHONPATH=src:. python3 sim/scripts/multifloor_nav_validation.py "
        "--route matrix --planners pct,astar --skip-mujoco --frontier-loop "
        "--local-planner-backend nanobind --require-production-local-planner "
        "--output-dir artifacts/server_sim_closure/multifloor_exploration "
        "--json-out artifacts/server_sim_closure/multifloor_exploration/report.json --strict",
        _eval_multifloor_exploration,
        host_requirements=(*LOCAL_NON_MOTION_HOST_REQUIREMENTS, *PCT_NATIVE_HOST_REQUIREMENTS),
    ),
    GateSpec(
        "large_terrain",
        "PCT/native path generation over large static terrain assets",
        (
            "artifacts/server_sim_closure/large_terrain/report.json",
            "artifacts/large_terrain_nav_validation*/report.json",
        ),
        "LINGTU_PCT_OPTIMIZE_TRAJECTORY=1 PYTHONPATH=src:. "
        "python3 sim/scripts/large_terrain_nav_validation.py "
        "--output-dir artifacts/server_sim_closure/large_terrain "
        "--planners pct,astar "
        "--json-out artifacts/server_sim_closure/large_terrain/report.json",
        _eval_large_terrain,
        host_requirements=(*LOCAL_NON_MOTION_HOST_REQUIREMENTS, *PCT_NATIVE_HOST_REQUIREMENTS),
    ),
    GateSpec(
        "native_pct_mujoco",
        "Native PCT route through ROS2 localPlanner/pathFollower into MuJoCo kinematic motion",
        (
            "artifacts/server_sim_closure/native_pct_mujoco/report.json",
            "artifacts/server_sim_closure/native_pct_mujoco/report.*.server.json",
            "artifacts/server_sim_closure/mujoco_fastlio2_live/pct-moving-obstacle-*/report.json",
            "artifacts/native_pct_mujoco_gate*/report.json",
            "artifacts/native_pct_large_terrain*/report.json",
            "artifacts/native_pct_effect/report.json",
            "artifacts/native_pct_effect/*pct*overlay*_report.json",
        ),
        "bash -lc 'source /opt/ros/humble/setup.bash && "
        "source install/setup.bash 2>/dev/null || true; "
        "export MUJOCO_GL=${MUJOCO_GL:-egl}; "
        "export PYOPENGL_PLATFORM=${PYOPENGL_PLATFORM:-egl}; "
        "PYTHONPATH=src:.:/opt/ros/humble/local/lib/python3.10/dist-packages:"
        "/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH "
        "python3 sim/scripts/mujoco/native_pct_gate.py "
        "--source-report artifacts/server_sim_closure/large_terrain/report.json "
        "--route terrain_short --planner pct "
        "--artifact-dir artifacts/server_sim_closure/native_pct_mujoco "
        "--json-out artifacts/server_sim_closure/native_pct_mujoco/report.json "
        "--timeout-s 80 --near-field-stop-distance 0.35 --strict'",
        _eval_native_pct_mujoco,
        host_requirements=ROS2_MUJOCO_PCT_LOCAL_PLANNER_HOST_REQUIREMENTS,
    ),
    GateSpec(
        "dynamic_obstacle_local_planner",
        "Nanobind LocalPlanner replans around changing added_obstacles without hardware cmd_vel",
        (
            "artifacts/server_sim_closure/dynamic_obstacle_local_planner/report.json",
            "artifacts/dynamic_obstacle_local_planner*/report.json",
        ),
        "PYTHONPATH=src:. python3 sim/scripts/dynamic_obstacle_local_planner_gate.py "
        "--backend nanobind "
        "--json-out artifacts/server_sim_closure/dynamic_obstacle_local_planner/report.json",
        _eval_dynamic_obstacle_local_planner,
        host_requirements=LOCAL_NUMERIC_NAV_HOST_REQUIREMENTS,
    ),
    GateSpec(
        "fastlio2_live",
        "Fast-LIO2 on live MuJoCo LiDAR/IMU into SlamBridgeModule",
        (
            "artifacts/server_sim_closure/fastlio2_live/report.json",
            "artifacts/server_sim_closure/mujoco_fastlio2_live/*/report.json",
            "artifacts/server_sim_closure/mujoco_fastlio2_live*/*/report.json",
            "artifacts/mujoco_fastlio2_live*/report.json",
            "artifacts/mujoco_fastlio2_live*/*/report.json",
            "artifacts/fastlio2*/report.json",
        ),
        "LINGTU_MUJOCO_LIVE_RUN_DIR=artifacts/server_sim_closure/mujoco_fastlio2_live "
        "bash sim/scripts/mujoco/launch_fastlio2_live.sh gate",
        _eval_fastlio2_live,
        host_requirements=ROS2_MUJOCO_FASTLIO2_HOST_REQUIREMENTS,
    ),
    GateSpec(
        "fastlio2_dynamic_inspection",
        "Core end-to-end algorithm gate: live MID-360/IMU -> Fast-LIO2 -> PCT -> local planner with moving obstacles and MP4 evidence",
        (
            "artifacts/server_sim_closure/mujoco_fastlio2_live*/inspection*/report.json",
            "artifacts/server_sim_closure/mujoco_fastlio2_live/inspection*/report.json",
            "artifacts/server_sim_closure/mujoco_fastlio2_live*/inspection*/*/report.json",
            "artifacts/server_sim_closure/mujoco_fastlio2_live/inspection*/*/report.json",
            "artifacts/mujoco_fastlio2_live*/inspection*/report.json",
            "artifacts/mujoco_fastlio2_live*/inspection*/*/report.json",
        ),
        "bash -lc 'source /opt/ros/humble/setup.bash && "
        "source install/setup.bash 2>/dev/null || true; "
        "export MUJOCO_GL=${MUJOCO_GL:-egl}; "
        "export PYOPENGL_PLATFORM=${PYOPENGL_PLATFORM:-egl}; "
        "export LINGTU_MUJOCO_LIVE_FASTLIO_LIDAR_INPUT=${LINGTU_MUJOCO_LIVE_FASTLIO_LIDAR_INPUT:-timed_pointcloud2}; "
        "export LINGTU_PCT_OPTIMIZE_TRAJECTORY=${LINGTU_PCT_OPTIMIZE_TRAJECTORY:-1}; "
        "export LINGTU_MUJOCO_LIVE_RUN_DIR=${LINGTU_MUJOCO_LIVE_RUN_DIR:-artifacts/server_sim_closure/mujoco_fastlio2_live}; "
        "export LINGTU_MUJOCO_LIVE_WORLD=${LINGTU_MUJOCO_LIVE_WORLD:-artifacts/server_sim_closure/large_terrain/large_terrain_scene.xml}; "
        "export LINGTU_MUJOCO_LIVE_INSPECTION_PLANNER=${LINGTU_MUJOCO_LIVE_INSPECTION_PLANNER:-pct}; "
        "export LINGTU_MUJOCO_LIVE_INSPECTION_TOMOGRAM=${LINGTU_MUJOCO_LIVE_INSPECTION_TOMOGRAM:-artifacts/server_sim_closure/large_terrain/tomogram.pickle}; "
        "export LINGTU_MUJOCO_LIVE_INSPECTION_GOALS=${LINGTU_MUJOCO_LIVE_INSPECTION_GOALS:-0.5,0.05;1.0,0.1;1.5,0.15}; "
        "export LINGTU_MUJOCO_LIVE_BUILD_TOMOGRAM=${LINGTU_MUJOCO_LIVE_BUILD_TOMOGRAM:-1}; "
        "export LINGTU_MUJOCO_LIVE_FASTLIO_IESKF_MAX_ITER=${LINGTU_MUJOCO_LIVE_FASTLIO_IESKF_MAX_ITER:-10}; "
        "export LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_COUNT=${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_COUNT:-3}; "
        "export LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_START_S=${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_START_S:-2}; "
        "export LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_PERIOD_S=${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_PERIOD_S:-6}; "
        "export LINGTU_MUJOCO_LIVE_CMD_VEL_MUX_SOURCE_TIMEOUT=${LINGTU_MUJOCO_LIVE_CMD_VEL_MUX_SOURCE_TIMEOUT:-5.0}; "
        "export LINGTU_MUJOCO_LIVE_CMD_VEL_ANGULAR_LIMIT=${LINGTU_MUJOCO_LIVE_CMD_VEL_ANGULAR_LIMIT:-0.25}; "
        "export LINGTU_MUJOCO_LIVE_NAV_MAX_ANGULAR_Z=${LINGTU_MUJOCO_LIVE_NAV_MAX_ANGULAR_Z:-0.20}; "
        "export LINGTU_MUJOCO_LIVE_RUNTIME_FAULT_CONFIRM_SAMPLES=${LINGTU_MUJOCO_LIVE_RUNTIME_FAULT_CONFIRM_SAMPLES:-6}; "
        "export LINGTU_MUJOCO_LIVE_RUNTIME_MOTION_FAULT_MIN_SIM_M=${LINGTU_MUJOCO_LIVE_RUNTIME_MOTION_FAULT_MIN_SIM_M:-1.0}; "
        "PYTHONPATH=src:.:/opt/ros/humble/local/lib/python3.10/dist-packages:"
        "/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH "
        "bash sim/scripts/mujoco/launch_fastlio2_live.sh inspection-moving-obstacle-video'",
        _eval_fastlio2_dynamic_inspection,
        host_requirements=ROS2_MUJOCO_FASTLIO2_HOST_REQUIREMENTS,
    ),
    GateSpec(
        "moving_obstacle_sweep",
        "Aggregate live Fast-LIO2/PCT moving-obstacle inspection reports across speed and density bins",
        (
            "artifacts/server_sim_closure/moving_obstacle_sweep/report.json",
            "artifacts/server_sim_closure/moving_obstacle_sweep/report*.json",
            "artifacts/moving_obstacle_sweep*/report.json",
        ),
        "bash -lc 'source /opt/ros/humble/setup.bash && "
        "source install/setup.bash 2>/dev/null || true; "
        "export MUJOCO_GL=${MUJOCO_GL:-egl}; "
        "export PYOPENGL_PLATFORM=${PYOPENGL_PLATFORM:-egl}; "
        "export LINGTU_MUJOCO_LIVE_FASTLIO_LIDAR_INPUT=${LINGTU_MUJOCO_LIVE_FASTLIO_LIDAR_INPUT:-timed_pointcloud2}; "
        "PYTHONPATH=src:.:/opt/ros/humble/local/lib/python3.10/dist-packages:"
        "/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH "
        "python3 sim/scripts/moving_obstacle_sweep_gate.py --run-matrix "
        "--child-run-root artifacts/server_sim_closure/moving_obstacle_sweep/children "
        "--world ${LINGTU_MUJOCO_LIVE_WORLD:-artifacts/server_sim_closure/large_terrain/large_terrain_scene.xml} "
        "--inspection-tomogram ${LINGTU_MUJOCO_LIVE_INSPECTION_TOMOGRAM:-artifacts/server_sim_closure/large_terrain/tomogram.pickle} "
        "--required-speed-bins slow,fast --required-density-bins sparse,dense "
        "--required-scan-time-profile physical_rolling --require-video-file "
        "--json-out artifacts/server_sim_closure/moving_obstacle_sweep/report.json --strict'",
        _eval_moving_obstacle_sweep,
        host_requirements=ROS2_MUJOCO_FASTLIO2_HOST_REQUIREMENTS,
    ),
    GateSpec(
        "large_loop_closure",
        "Large-range loop route: live Fast-LIO2 mapping/localization -> PCT -> local planner/path follower -> closed-loop return",
        (
            "artifacts/server_sim_closure/large_loop_closure/report.json",
            "artifacts/server_sim_closure/large_loop*/large_loop_closure_report.json",
            "artifacts/server_sim_closure/large_loop*/report*.json",
            "artifacts/large_loop_closure*/report.json",
        ),
        "bash -lc 'source /opt/ros/humble/setup.bash && "
        "source install/setup.bash 2>/dev/null || true; "
        "export MUJOCO_GL=${MUJOCO_GL:-egl}; "
        "export PYOPENGL_PLATFORM=${PYOPENGL_PLATFORM:-egl}; "
        "export LINGTU_MUJOCO_LIVE_FASTLIO_LIDAR_INPUT=${LINGTU_MUJOCO_LIVE_FASTLIO_LIDAR_INPUT:-timed_pointcloud2}; "
        "export LINGTU_PCT_OPTIMIZE_TRAJECTORY=${LINGTU_PCT_OPTIMIZE_TRAJECTORY:-1}; "
        "export LINGTU_MUJOCO_LIVE_RUN_DIR=${LINGTU_MUJOCO_LIVE_RUN_DIR:-artifacts/server_sim_closure/mujoco_fastlio2_live}; "
        "export LINGTU_MUJOCO_LIVE_WORLD=${LINGTU_MUJOCO_LIVE_WORLD:-artifacts/server_sim_closure/large_terrain/large_terrain_scene.xml}; "
        "export LINGTU_MUJOCO_LIVE_DURATION_INSPECTION=${LINGTU_MUJOCO_LIVE_DURATION_INSPECTION:-240}; "
        "export LINGTU_MUJOCO_LIVE_MAX_WALL_TIME_S=${LINGTU_MUJOCO_LIVE_MAX_WALL_TIME_S:-7200}; "
        "export LINGTU_MUJOCO_LIVE_NAV_MAX_LINEAR_SPEED=${LINGTU_MUJOCO_LIVE_NAV_MAX_LINEAR_SPEED:-0.45}; "
        "export LINGTU_MUJOCO_LIVE_CMD_VEL_LINEAR_LIMIT=${LINGTU_MUJOCO_LIVE_CMD_VEL_LINEAR_LIMIT:-0.45}; "
        "export LINGTU_MUJOCO_LIVE_CMD_VEL_LINEAR_ACCEL_LIMIT=${LINGTU_MUJOCO_LIVE_CMD_VEL_LINEAR_ACCEL_LIMIT:-0.8}; "
        "export LINGTU_MUJOCO_LIVE_CMD_VEL_ANGULAR_LIMIT=${LINGTU_MUJOCO_LIVE_CMD_VEL_ANGULAR_LIMIT:-0.25}; "
        "export LINGTU_MUJOCO_LIVE_NAV_MAX_ANGULAR_Z=${LINGTU_MUJOCO_LIVE_NAV_MAX_ANGULAR_Z:-0.25}; "
        "export LINGTU_MUJOCO_LIVE_BUILD_TOMOGRAM=${LINGTU_MUJOCO_LIVE_BUILD_TOMOGRAM:-1}; "
        "export LINGTU_MUJOCO_LIVE_FASTLIO_IESKF_MAX_ITER=${LINGTU_MUJOCO_LIVE_FASTLIO_IESKF_MAX_ITER:-10}; "
        "export LINGTU_MUJOCO_LIVE_RUNTIME_FAULT_CONFIRM_SAMPLES=${LINGTU_MUJOCO_LIVE_RUNTIME_FAULT_CONFIRM_SAMPLES:-6}; "
        "export LINGTU_MUJOCO_LIVE_RUNTIME_MOTION_FAULT_MIN_SIM_M=${LINGTU_MUJOCO_LIVE_RUNTIME_MOTION_FAULT_MIN_SIM_M:-1.0}; "
        "export LINGTU_MUJOCO_LIVE_NAV_TURN_SPEED_YAW_RATE_START=${LINGTU_MUJOCO_LIVE_NAV_TURN_SPEED_YAW_RATE_START:-0.0}; "
        "export LINGTU_MUJOCO_LIVE_NAV_TURN_SPEED_MIN_SCALE=${LINGTU_MUJOCO_LIVE_NAV_TURN_SPEED_MIN_SCALE:-1.0}; "
        "export LINGTU_MUJOCO_LIVE_INSPECTION_PLANNER=${LINGTU_MUJOCO_LIVE_INSPECTION_PLANNER:-pct}; "
        "export LINGTU_MUJOCO_LIVE_INSPECTION_TOMOGRAM=${LINGTU_MUJOCO_LIVE_INSPECTION_TOMOGRAM:-artifacts/server_sim_closure/large_terrain/tomogram.pickle}; "
        "export LINGTU_MUJOCO_LIVE_INSPECTION_GOALS=${LINGTU_MUJOCO_LIVE_INSPECTION_GOALS:-"
        f"{LARGE_LOOP_INSPECTION_GOALS}"
        "}; "
        "export LINGTU_MUJOCO_LIVE_INSPECTION_MIN_CHECKPOINTS=${LINGTU_MUJOCO_LIVE_INSPECTION_MIN_CHECKPOINTS:-4}; "
        "export LINGTU_MUJOCO_LIVE_INSPECTION_DOWNSAMPLE_DIST=${LINGTU_MUJOCO_LIVE_INSPECTION_DOWNSAMPLE_DIST:-1.0}; "
        "export LINGTU_MUJOCO_LIVE_INSPECTION_WAYPOINT_THRESHOLD=${LINGTU_MUJOCO_LIVE_INSPECTION_WAYPOINT_THRESHOLD:-0.65}; "
        "export LINGTU_MUJOCO_LIVE_INSPECTION_FINAL_WAYPOINT_THRESHOLD=${LINGTU_MUJOCO_LIVE_INSPECTION_FINAL_WAYPOINT_THRESHOLD:-0.65}; "
        "export LINGTU_MUJOCO_LIVE_INSPECTION_COMPLETE_PATH_ON_GOAL_PROXIMITY=${LINGTU_MUJOCO_LIVE_INSPECTION_COMPLETE_PATH_ON_GOAL_PROXIMITY:-1}; "
        "export LINGTU_MUJOCO_LIVE_INSPECTION_GOAL_PROXIMITY_COMPLETION_THRESHOLD=${LINGTU_MUJOCO_LIVE_INSPECTION_GOAL_PROXIMITY_COMPLETION_THRESHOLD:-0.65}; "
        "export LINGTU_MUJOCO_LIVE_INSPECTION_PATH_GOAL_TOLERANCE=${LINGTU_MUJOCO_LIVE_INSPECTION_PATH_GOAL_TOLERANCE:-0.12}; "
        "export LINGTU_MUJOCO_LIVE_INSPECTION_PATH_LOOKAHEAD=${LINGTU_MUJOCO_LIVE_INSPECTION_PATH_LOOKAHEAD:-2.0}; "
        "export LINGTU_MUJOCO_LIVE_INSPECTION_PATH_MIN_SPEED=${LINGTU_MUJOCO_LIVE_INSPECTION_PATH_MIN_SPEED:-0.15}; "
        "export LINGTU_MUJOCO_LIVE_INSPECTION_PATH_YAW_RATE_GAIN=${LINGTU_MUJOCO_LIVE_INSPECTION_PATH_YAW_RATE_GAIN:-2.5}; "
        "export LINGTU_MUJOCO_LIVE_INSPECTION_PATH_STOP_YAW_RATE_GAIN=${LINGTU_MUJOCO_LIVE_INSPECTION_PATH_STOP_YAW_RATE_GAIN:-2.5}; "
        "export LINGTU_MUJOCO_LIVE_INSPECTION_PATH_DIR_DIFF_THRE=${LINGTU_MUJOCO_LIVE_INSPECTION_PATH_DIR_DIFF_THRE:-1.8}; "
        "export LINGTU_MUJOCO_LIVE_SCAN_TIME_PROFILE=${LINGTU_MUJOCO_LIVE_SCAN_TIME_PROFILE:-physical_rolling}; "
        "PYTHONPATH=src:.:/opt/ros/humble/local/lib/python3.10/dist-packages:"
        "/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH "
        "run_start=$(date +%s); "
        "bash sim/scripts/mujoco/launch_fastlio2_live.sh inspection-loop-video; "
        "latest=$(sed -n \"s/^latest_run_dir=//p\" \"$LINGTU_MUJOCO_LIVE_RUN_DIR/latest.txt\" | tail -n 1); "
        "test -n \"$latest\" && test -f \"$latest/report.json\" && "
        "test \"$(stat -c %Y \"$latest/report.json\")\" -ge \"$run_start\" && "
        "python3 sim/scripts/large_loop_closure_gate.py --report \"$latest/report.json\" "
        "--required-scan-time-profile physical_rolling --require-video-file "
        "--json-out artifacts/server_sim_closure/large_loop_closure/report.json --strict'",
        _eval_large_loop_closure,
        host_requirements=ROS2_MUJOCO_FASTLIO2_HOST_REQUIREMENTS,
    ),
    GateSpec(
        "mujoco_tare_exploration",
        "Native TARE on MuJoCo MID-360/Fast-LIO with LingTu navigation and cmd_vel closure",
        (
            "artifacts/server_sim_closure/mujoco_tare_exploration/report.json",
            "artifacts/server_sim_closure/mujoco_tare_exploration*/tare-*/report.json",
            "artifacts/server_sim_closure/mujoco_fastlio2_live*/tare-*/report.json",
            "artifacts/mujoco_tare_exploration*/tare-*/report.json",
        ),
        "bash -lc 'source /opt/ros/humble/setup.bash && "
        "source install/setup.bash 2>/dev/null || true; "
        "export MUJOCO_GL=${MUJOCO_GL:-egl}; "
        "export LINGTU_MUJOCO_LIVE_RUN_DIR=${LINGTU_MUJOCO_LIVE_RUN_DIR:-artifacts/server_sim_closure/mujoco_tare_exploration}; "
        "export LINGTU_MUJOCO_LIVE_WORLD=${LINGTU_MUJOCO_LIVE_WORLD:-industrial_park}; "
        "export LINGTU_MUJOCO_LIVE_DURATION_TARE=${LINGTU_MUJOCO_LIVE_DURATION_TARE:-180}; "
        "export LINGTU_MUJOCO_LIVE_BUILD_TOMOGRAM=${LINGTU_MUJOCO_LIVE_BUILD_TOMOGRAM:-1}; "
        "PYTHONPATH=src:.:/opt/ros/humble/local/lib/python3.10/dist-packages:"
        "/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH "
        "bash sim/scripts/mujoco/launch_fastlio2_live.sh tare'",
        _eval_fastlio2_live,
        host_requirements=ROS2_MUJOCO_FASTLIO2_HOST_REQUIREMENTS,
    ),
    GateSpec(
        "policy_nav",
        "ONNX gait policy with full-stack simulated navigation",
        (
            "artifacts/server_sim_closure/policy_nav/report.json",
            "artifacts/policy_nav*/report.json",
            "artifacts/*policy*/*.json",
            "artifacts/policy_direct_verify.json",
        ),
        "PYTHONPATH=src:. MUJOCO_GL=egl python3 sim/scripts/policy_nav_smoke.py "
        "--direct-duration 4 --goal-distance 0.8 --nav-duration 18 "
        "--nav-planner-backend octoplanner3d "
        "--nav-local-planner-backend nanobind --nav-path-follower-backend nav_kernel "
        "--nav-costmap-wait 3 --nav-path-min-speed 0.25 --nav-path-max-speed 0.6 "
        "--nav-waypoint-threshold 0.30 --nav-final-waypoint-threshold 0.25 "
        "--nav-path-goal-tolerance 0.25 --max-nav-dist-to-goal 0.35 "
        "--min-nav-motion 0.35 --nav-max-angular-z 0.15 "
        "--json-out artifacts/server_sim_closure/policy_nav/report.json",
        _eval_policy_nav,
        host_requirements=LOCAL_NUMERIC_SIM_HOST_REQUIREMENTS,
    ),
    GateSpec(
        "gateway_dry_run",
        "Gateway goal/preview command flow without driver or hardware cmd_vel",
        ("artifacts/gateway_goal_dry_run*/report.json", "artifacts/server_sim_closure/gateway_dry_run/report.json"),
        "PYTHONPATH=src:. python3 sim/scripts/gateway_goal_dry_run_gate.py --json-out artifacts/server_sim_closure/gateway_dry_run/report.json --strict",
        _eval_gateway_dry_run,
        host_requirements=LOCAL_NON_MOTION_HOST_REQUIREMENTS,
    ),
    GateSpec(
        "routecheck_preflight",
        "Gateway non-motion route preflight comparing baseline and candidate planning",
        (
            "artifacts/server_sim_closure/routecheck/summary.json",
            "artifacts/super_lio_route_preflight_*/summary.json",
            "artifacts/routecheck*/summary.json",
        ),
        "PYTHONPATH=src:. python3 sim/scripts/routecheck_preflight_gate.py "
        "--map server_sim_demo --goal-x 1.0 --goal-y 0.0 --goal-yaw 0.0 "
        "--json-out artifacts/server_sim_closure/routecheck/summary.json --strict",
        _eval_routecheck_preflight,
        host_requirements=LOCAL_NON_MOTION_HOST_REQUIREMENTS,
    ),
    GateSpec(
        "blocked_route_replan_preflight",
        "Gateway non-motion blocked-route replanning preflight with synthetic route obstruction",
        (
            "artifacts/server_sim_closure/blocked_route_replan/report.json",
            "artifacts/blocked_route_replan*/report.json",
        ),
        "PYTHONPATH=src:. python3 sim/scripts/blocked_route_replan_gate.py "
        "--map server_sim_demo --goal-x 2.0 --goal-y 0.0 "
        "--json-out artifacts/server_sim_closure/blocked_route_replan/report.json --strict",
        _eval_blocked_route_replan_preflight,
        host_requirements=LOCAL_NON_MOTION_HOST_REQUIREMENTS,
    ),
    GateSpec(
        "navigation_replay_deviation",
        "Offline navigation replay/deviation over routecheck-derived or recorded global_path, local_path, cmd_vel, and odometry",
        (
            "artifacts/server_sim_closure/navigation_replay_deviation/report.json",
            "artifacts/navigation_replay_deviation*/report.json",
        ),
        "PYTHONPATH=src:. python3 sim/scripts/navigation_replay_deviation_gate.py "
        "--routecheck-report artifacts/server_sim_closure/routecheck/summary.json "
        "--write-trace artifacts/server_sim_closure/navigation_replay_deviation/trace.json "
        "--json-out artifacts/server_sim_closure/navigation_replay_deviation/report.json --strict",
        _eval_navigation_replay_deviation,
        host_requirements=LOCAL_NON_MOTION_HOST_REQUIREMENTS,
    ),
    GateSpec(
        "gazebo_runtime",
        "ROS-native Gazebo TF, point-cloud, navigation loop, cumulative map growth, wavefront frontier exploration, and TARE contract",
        (
            "artifacts/server_sim_closure/gazebo_runtime_explore/report_grid_astar_odomfoot.json",
            "artifacts/server_sim_closure/gazebo_runtime_explore/report.json",
            "artifacts/server_sim_closure/gazebo_runtime_nav/report.json",
            "artifacts/server_sim_closure/gazebo_runtime/report.json",
        ),
        "bash -lc 'source /opt/ros/humble/setup.bash && "
        "source install/setup.bash 2>/dev/null || true; "
        "DOMAIN=${ROS_DOMAIN_ID:-30}; "
        "PART=lingtu_grid_astar_odomfoot_$(date +%s); "
        "PYTHONPATH=src:.:/opt/ros/humble/local/lib/python3.10/dist-packages:"
        "/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH "
        "python3 sim/scripts/gazebo_runtime_gate.py --check-nav-loop "
        "--check-frontier-exploration --check-cumulative-map --check-tare-contract "
        "--warmup-sec 20 --smoke-timeout-sec 20 --nav-timeout-sec 35 "
        "--nav-gazebo-warmup-sec 10 --nav-warmup-sec 0.0 "
        "--nav-goal-delay-sec 0.0 --nav-goal-republish-sec 0.5 "
        "--frontier-timeout-sec 60 --frontier-continue-after-pass-sec 5 "
        "--spawn-x 0.0 --spawn-y 0.0 --nav-goal-x 3.0 --nav-goal-y 1.0 --nav-goal-z 0.0 "
        "--ros-domain-id $DOMAIN --gz-partition $PART "
        "--frontier-trace-out artifacts/server_sim_closure/gazebo_runtime_explore/frontier_trace_grid_astar_odomfoot.json "
        "--json-out artifacts/server_sim_closure/gazebo_runtime_explore/report_grid_astar_odomfoot.json "
        "--launch-log artifacts/server_sim_closure/gazebo_runtime_explore/launch_grid_astar_odomfoot.log'",
        _eval_gazebo_runtime,
        host_requirements=ROS2_GAZEBO_NAV_HOST_REQUIREMENTS,
    ),
    GateSpec(
        "cmu_unity_sim",
        "Isolated CMU Humble Unity/TARE benchmark preflight with LingTu adapter/remap contract and no hardware output",
        (
            "artifacts/server_sim_closure/cmu_unity_sim/report.json",
            "artifacts/cmu_unity_sim*/report.json",
        ),
        "PYTHONPATH=src:. python3 sim/scripts/cmu_unity_sim_gate.py "
        "--json-out artifacts/server_sim_closure/cmu_unity_sim/report.json --strict",
        _eval_cmu_unity_sim,
        host_requirements=LOCAL_NUMERIC_SIM_HOST_REQUIREMENTS,
    ),
    GateSpec(
        "cmu_unity_runtime",
        "Runtime CMU Unity + LingTu adapter/TARE/nav closed loop with real waypoints, cmd_vel, odom, and map-area growth",
        (
            "artifacts/server_sim_closure/cmu_unity_runtime/report.json",
            "artifacts/server_sim_closure/cmu_unity_pct_strict/report_unique_waypoints.json",
            "artifacts/server_sim_closure/cmu_unity_pct_strict/report.json",
            "artifacts/cmu_unity_runtime*/report.json",
        ),
        "bash -lc 'source /opt/ros/humble/setup.bash && "
        "source install/setup.bash 2>/dev/null || true; "
        "PYTHONPATH=src:.:/opt/ros/humble/local/lib/python3.10/dist-packages:"
        "/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH "
        "python3 sim/scripts/cmu_unity_runtime_gate.py "
        "--json-out artifacts/server_sim_closure/cmu_unity_runtime/report.json --strict'",
        _eval_cmu_unity_runtime,
        host_requirements=ROS2_GAZEBO_HOST_REQUIREMENTS,
    ),
    GateSpec(
        "cmu_unity_pct_strict",
        "Runtime CMU Unity + TARE + LingTu PCT with same-source tomogram, no planner fallback, path topics, odom motion, and map growth",
        (
            "artifacts/server_sim_closure/cmu_unity_pct_strict/report_unique_waypoints.json",
            "artifacts/server_sim_closure/cmu_unity_pct_strict/report.json",
            "artifacts/server_sim_closure/cmu_unity_pct_strict_*/report.json",
        ),
        "bash -lc 'source /opt/ros/humble/setup.bash && "
        "source install/setup.bash 2>/dev/null || true; "
        "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-75}; "
        "export DISPLAY=${DISPLAY:-:1}; "
        "export FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}; "
        "export LINGTU_CMU_RUN_DIR=${LINGTU_CMU_RUN_DIR:-artifacts/server_sim_closure/cmu_unity_pct_strict}; "
        "export LINGTU_CMU_PLANNER=pct; "
        "export LINGTU_CMU_START_CMU_TARE=1; "
        "export LINGTU_CMU_TARE_SCENARIO=${LINGTU_CMU_TARE_SCENARIO:-indoor_large}; "
        "export LINGTU_CMU_TARE_AUTOSTART=0; "
        "export LINGTU_CMU_ENABLE_FRONTIER=0; "
        "export LINGTU_CMU_EXTERNAL_STRATEGY_PATH_CONTROL=0; "
        "export LINGTU_CMU_AUTO_SESSION=1; "
        "export LINGTU_CMU_EXPLORATION_AUTO_START=0; "
        "export LINGTU_CMU_ALLOW_DIRECT_GOAL_FALLBACK=0; "
        "export LINGTU_CMU_AUTO_TOMOGRAM=${LINGTU_CMU_AUTO_TOMOGRAM:-1}; "
        "export LINGTU_CMU_TOMOGRAM_TOPICS=${LINGTU_CMU_TOMOGRAM_TOPICS:-/slam/map_cloud,/nav/terrain_map_ext}; "
        "export LINGTU_CMU_TOMOGRAM_MODE=${LINGTU_CMU_TOMOGRAM_MODE:-official}; "
        "export LINGTU_CMU_TOMOGRAM_DURATION_SEC=${LINGTU_CMU_TOMOGRAM_DURATION_SEC:-20}; "
        "export LINGTU_CMU_GATE_TIMEOUT_SEC=${LINGTU_CMU_GATE_TIMEOUT_SEC:-240}; "
        "export LINGTU_CMU_GATE_MIN_UNIQUE_WAYPOINTS=${LINGTU_CMU_GATE_MIN_UNIQUE_WAYPOINTS:-3}; "
        "export LINGTU_CMU_GATE_REQUIRED_MAP_TOPICS=${LINGTU_CMU_GATE_REQUIRED_MAP_TOPICS:-/slam/map_cloud,/nav/terrain_map_ext}; "
        "export LINGTU_CMU_GATE_REQUIRE_PCT=1; "
        "PYTHONPATH=src:.:/opt/ros/humble/local/lib/python3.10/dist-packages:"
        "/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH "
        "sim/scripts/launch_cmu_unity_lingtu_runtime.sh start --gate --rviz'",
        _eval_cmu_unity_pct_strict,
        host_requirements=ROS2_GAZEBO_HOST_REQUIREMENTS,
    ),
    GateSpec(
        "saved_map_relocalize",
        "Runtime saved-map relocalization with live MuJoCo/Fast-LIO and localizer",
        (
            "artifacts/server_sim_closure/saved_map_relocalize_runtime/report.json",
            "artifacts/server_sim_closure/saved_map_relocalize_runtime*/report.json",
            "artifacts/saved_map_relocalize_runtime*/report.json",
        ),
        "bash -lc 'source /opt/ros/humble/setup.bash && "
        "source install/setup.bash 2>/dev/null || true; "
        "export MUJOCO_GL=${MUJOCO_GL:-egl}; "
        "export PYOPENGL_PLATFORM=${PYOPENGL_PLATFORM:-egl}; "
        "PYTHONPATH=src:.:/opt/ros/humble/local/lib/python3.10/dist-packages:"
        "/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH "
        "python3 sim/scripts/saved_map_relocalize_runtime_gate.py "
        "--map-pcd latest "
        "--duration 12 "
        "--scan-time-profile map_metadata "
        "--live-drive-source fixed "
        "--mid360-samples-per-frame 15000 "
        "--fastlio-lidar-input timed_pointcloud2 "
        "--fastlio-ieskf-max-iter 10 "
        "--runtime-fault-confirm-samples 6 "
        "--runtime-motion-fault-min-sim-m 1.0 "
        "--localizer-rough-score-thresh 0.35 "
        "--localizer-refine-score-thresh 0.35 "
        "--json-out artifacts/server_sim_closure/saved_map_relocalize_runtime/report.json --strict'",
        _eval_saved_map_relocalize,
        host_requirements=ROS2_MUJOCO_LOCALIZATION_HOST_REQUIREMENTS,
    ),
    GateSpec(
        "bbs3d_kidnapped_relocalize",
        "BBS3D kidnapped-robot global relocalization with live MuJoCo/Fast-LIO and localizer",
        (
            "artifacts/server_sim_closure/bbs3d_kidnapped_relocalize/report.json",
            "artifacts/server_sim_closure/bbs3d_kidnapped_relocalize*/report.json",
        ),
        "bash -lc 'source /opt/ros/humble/setup.bash && "
        "source install/setup.bash 2>/dev/null || true; "
        "export MUJOCO_GL=${MUJOCO_GL:-egl}; "
        "export PYOPENGL_PLATFORM=${PYOPENGL_PLATFORM:-egl}; "
        "PYTHONPATH=src:.:/opt/ros/humble/local/lib/python3.10/dist-packages:"
        "/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH "
        "python3 sim/scripts/saved_map_relocalize_runtime_gate.py "
        "--map-pcd latest --check-global-relocalize "
        "--duration 12 "
        "--scan-time-profile map_metadata "
        "--live-drive-source fixed "
        "--mid360-samples-per-frame 15000 "
        "--fastlio-lidar-input timed_pointcloud2 "
        "--fastlio-ieskf-max-iter 10 "
        "--runtime-fault-confirm-samples 6 "
        "--runtime-motion-fault-min-sim-m 1.0 "
        "--localizer-rough-score-thresh 0.35 "
        "--localizer-refine-score-thresh 0.35 "
        "--kidnap-initial-x 3.0 --kidnap-initial-y 2.0 --kidnap-initial-yaw 1.2 "
        "--max-map-odom-xy-m 10.0 --monitor-after-service-s 30 "
        "--run-dir artifacts/server_sim_closure/bbs3d_kidnapped_relocalize "
        "--json-out artifacts/server_sim_closure/bbs3d_kidnapped_relocalize/report.json --strict'",
        _eval_saved_map_relocalize,
        host_requirements=ROS2_MUJOCO_LOCALIZATION_HOST_REQUIREMENTS,
    ),
    GateSpec(
        "pct_saved_map_navigation",
        "Saved-map PCT navigation after relocalization: tomogram -> PCT -> localPlanner/pathFollower -> MuJoCo motion",
        (
            "artifacts/server_sim_closure/pct_saved_map_navigation/report.json",
            "artifacts/server_sim_closure/pct_saved_map_navigation*/report.json",
            "artifacts/pct_saved_map_navigation*/report.json",
        ),
        "bash -lc 'source /opt/ros/humble/setup.bash && "
        "source install/setup.bash 2>/dev/null || true; "
        "export MUJOCO_GL=${MUJOCO_GL:-egl}; "
        "export PYOPENGL_PLATFORM=${PYOPENGL_PLATFORM:-egl}; "
        "PYTHONPATH=src:.:/opt/ros/humble/local/lib/python3.10/dist-packages:"
        "/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH "
        "python3 sim/scripts/pct_saved_map_navigation_gate.py "
        "--relocalize-report artifacts/server_sim_closure/saved_map_relocalize_runtime/report.json "
        "--max-relocalize-report-age-s 86400 "
        "--goal 4.5 3.0 --timeout-s 80 --near-field-stop-distance 0.35 "
        "--json-out artifacts/server_sim_closure/pct_saved_map_navigation/report.json --strict'",
        _eval_pct_saved_map_navigation,
        host_requirements=ROS2_MUJOCO_PCT_LOCAL_PLANNER_HOST_REQUIREMENTS,
    ),
)

ALGORITHM_PRESETS: dict[str, tuple[str, ...]] = {
    "core_algorithm": (
        "large_terrain",
        "dynamic_obstacle_local_planner",
        "fastlio2_dynamic_inspection",
        "moving_obstacle_sweep",
        "large_loop_closure",
    ),
    "inspection_mvp": (
        *INSPECTION_MVP_REQUIRED_GATES,
    ),
    "dimos_benchmark": (*DIMOS_BENCHMARK_REQUIRED_GATES,),
    "g4_server_full_sim": (*G4_SERVER_FULL_SIM_REQUIRED_GATES,),
    "full_algorithm": tuple(spec.name for spec in GATES),
}


ALGORITHM_VALIDATION_FLOW: tuple[dict[str, Any], ...] = (
    {
        "id": "runtime_data_plane",
        "title": "Gateway Runtime Data Plane",
        "gates": ("gateway_runtime_acceptance",),
        "proves": "Gateway/ModulePort runtime streams, stage evidence, and command whitelist are product-visible through the runtime data plane",
    },
    {
        "id": "map_asset",
        "title": "Map/tomogram asset",
        "gates": ("large_terrain",),
        "proves": "large same-source map/tomogram artifacts exist for route and PCT checks",
    },
    {
        "id": "offline_navigation_replay",
        "title": "Offline navigation replay/deviation",
        "gates": ("navigation_replay_deviation",),
        "proves": "recorded global path, local path, cmd_vel, and odometry replay within endpoint and tracking deviation bounds",
    },
    {
        "id": "blocked_route_replanning",
        "title": "Blocked-route replanning",
        "gates": ("blocked_route_replan_preflight",),
        "proves": "a baseline route intersects a synthetic block while the candidate preview replans around it without motion commands",
    },
    {
        "id": "static_global_planning",
        "title": "Static global planning",
        "gates": ("native_pct_mujoco", "pct_saved_map_navigation"),
        "proves": "PCT global planning runs on saved static map/tomogram artifacts",
    },
    {
        "id": "local_dynamic_avoidance",
        "title": "Local dynamic avoidance",
        "gates": ("dynamic_obstacle_local_planner", "moving_obstacle_sweep"),
        "proves": "local planner and cmd_vel chain react to moving obstacles without planner fallback",
    },
    {
        "id": "realtime_mapping_localization",
        "title": "Realtime mapping/localization",
        "gates": ("fastlio2_dynamic_inspection", "large_loop_closure"),
        "proves": "raw LiDAR/IMU feed Fast-LIO mapping/localization during navigation",
    },
    {
        "id": "long_range_loop_closure",
        "title": "Long-range loop closure",
        "gates": ("large_loop_closure",),
        "proves": "large same-source route keeps bounded drift while PCT/local path/cmd_vel remain active",
    },
    {
        "id": "saved_map_lifecycle",
        "title": "Saved-map lifecycle",
        "gates": ("saved_map_relocalize", "pct_saved_map_navigation"),
        "proves": "saved map relocalization and same-source saved-map PCT navigation both work",
    },
    {
        "id": "command_safety",
        "title": "Command safety",
        "gates": ("routecheck_preflight",),
        "proves": "route preview and validation do not publish hardware motion commands",
    },
    {
        "id": "ros_integration",
        "title": "ROS integration smoke",
        "gates": ("gazebo_runtime",),
        "proves": "ROS topics, publisher identity, map growth, and navigation loop are coherent",
    },
)


def _ordered_gate_names(names: set[str]) -> list[str]:
    if names == set(DIMOS_BENCHMARK_REQUIRED_GATES):
        return list(DIMOS_BENCHMARK_REQUIRED_GATES)
    if names == set(G4_SERVER_FULL_SIM_REQUIRED_GATES):
        return list(G4_SERVER_FULL_SIM_REQUIRED_GATES)
    if names == set(INSPECTION_MVP_REQUIRED_GATES):
        return list(INSPECTION_MVP_REQUIRED_GATES)
    ordered = [spec.name for spec in GATES if spec.name in names]
    ordered_set = set(ordered)
    ordered.extend(sorted(names - ordered_set))
    return ordered


GATE_RUN_DEPENDENCIES: dict[str, tuple[str, ...]] = {
    "native_pct_mujoco": ("large_terrain",),
    "fastlio2_dynamic_inspection": ("large_terrain",),
    "moving_obstacle_sweep": ("fastlio2_dynamic_inspection",),
    "large_loop_closure": ("fastlio2_dynamic_inspection",),
    "pct_saved_map_navigation": ("saved_map_relocalize",),
}


def _expected_report_path(spec: GateSpec) -> str:
    if spec.expected_report_path:
        return spec.expected_report_path
    return spec.default_patterns[0] if spec.default_patterns else ""


def _normalized_gate_run_invocation(
    command: str,
    *,
    platform_system: str | None = None,
) -> tuple[str | list[str], dict[str, Any]]:
    """Return a subprocess invocation for a gate command on the current host."""
    resolved_platform = platform_system or platform.system()
    if resolved_platform.lower() != "windows":
        return command, {"shell": True}

    try:
        import shlex

        tokens = shlex.split(command, posix=True)
    except Exception:
        return command, {"shell": True}
    if not tokens:
        return command, {"shell": True}

    env_updates: dict[str, str] = {}
    command_index = 0
    for token in tokens:
        key, separator, value = token.partition("=")
        if not separator:
            break
        if not key or not (key[0].isalpha() or key[0] == "_"):
            break
        if not all(char.isalnum() or char == "_" for char in key):
            break
        if "$" in value or "`" in value:
            return command, {"shell": True}
        env_updates[key] = value
        command_index += 1

    if not env_updates or command_index >= len(tokens):
        return command, {"shell": True}

    interpreter = Path(tokens[command_index]).name.lower()
    if interpreter not in {"python", "python.exe", "python3", "python3.exe"}:
        return command, {"shell": True}

    args = tokens[command_index + 1 :]
    if not args:
        return command, {"shell": True}

    env = os.environ.copy()
    for key, value in env_updates.items():
        if key == "PYTHONPATH":
            pythonpath_entries: list[str] = []
            for entry in value.split(":"):
                if entry == "src":
                    pythonpath_entries.append(str(SRC))
                elif entry == ".":
                    pythonpath_entries.append(str(ROOT))
                elif entry:
                    pythonpath_entries.append(entry)
            existing = env.get("PYTHONPATH")
            if existing:
                pythonpath_entries.append(existing)
            env[key] = os.pathsep.join(pythonpath_entries)
        else:
            env[key] = value
    return [sys.executable, *args], {"shell": False, "env": env}


def _host_requirements_for_gates(names: set[str]) -> dict[str, list[str]]:
    specs = {spec.name: spec for spec in GATES}
    return {
        name: list(specs[name].host_requirements)
        for name in _ordered_gate_names(names)
        if name in specs
    }


def _missing_required_commands(
    gates: dict[str, Any],
    missing_or_failed: list[str],
) -> list[dict[str, Any]]:
    specs = {spec.name: spec for spec in GATES}
    commands: list[dict[str, Any]] = []
    for name in _ordered_gate_names(set(missing_or_failed)):
        spec = specs.get(name)
        if spec is None:
            continue
        gate = gates.get(name) or {}
        commands.append(
            {
                "name": name,
                "description": spec.description,
                "status": gate.get("status", "missing"),
                "path": gate.get("path", ""),
                "expected_report_path": _expected_report_path(spec),
                "accepted_patterns": list(spec.default_patterns),
                "command": spec.command,
                "host_requirements": list(spec.host_requirements),
            }
        )
    return commands


_BLOCKER_CATEGORY_KEYWORDS: tuple[tuple[str, tuple[str, ...]], ...] = (
    (
        "product_data_plane",
        (
            "gateway",
            "runtime dataflow",
            "runtime data plane",
            "module_port",
            "moduleport",
            "sse stream",
        ),
    ),
    (
        "slam_localization",
        (
            "fast-lio",
            "fastlio",
            "slam",
            "localization",
            "relocalize",
            "tracking_health",
            "z_consistency",
            "yaw_consistency",
            "motion_consistency",
            "z drift",
            "yaw drift",
            "odometry diverged",
        ),
    ),
    (
        "environment_runtime",
        (
            "native runtime",
            "abi",
            "python_tag",
            "shared library",
            "no runnable pct native modules",
            "runtime unavailable",
            "launch_script",
            "launch script",
            "mingw numpy",
            "numpy",
            "local planner runtime",
        ),
    ),
    (
        "planning_tracking",
        (
            "pct",
            "planner",
            "global_path",
            "local_path",
            "path_length",
            "cmd_vel_nonzero",
            "checkpoint",
            "patrol",
            "goal",
            "navigation",
            "replay",
            "replan",
            "blocked_route",
            "synthetic block",
            "deviation",
            "tracking_error",
            "path follower",
        ),
    ),
    (
        "dynamic_obstacle",
        (
            "moving_obstacle",
            "moving obstacle",
            "speed/density",
            "trail_clearance",
            "collision",
            "clearance",
        ),
    ),
    (
        "artifact_contract",
        (
            "report missing",
            "report_age_s",
            "same-source",
            "tomogram",
            "inspection_tomogram",
            "world file",
            "world/tomogram",
            "metadata",
            "video",
            "file missing",
            "stale",
        ),
    ),
    (
        "command_safety",
        (
            "cmd_vel_sent_to_hardware",
            "real_robot_motion",
            "route preview",
            "preflight",
        ),
    ),
    (
        "simulation_integration",
        (
            "ros topics",
            "gazebo",
            "cmu",
            "unity",
            "publisher",
            "bridge",
        ),
    ),
)


def _blocker_categories(blockers: list[str]) -> set[str]:
    text = "\n".join(str(blocker).lower() for blocker in blockers)
    categories: set[str] = set()
    for category, keywords in _BLOCKER_CATEGORY_KEYWORDS:
        if any(keyword in text for keyword in keywords):
            categories.add(category)
    if not categories and blockers:
        categories.add("unclassified")
    return categories


def _gate_failure_categories(gate: dict[str, Any]) -> set[str]:
    categories = _blocker_categories(
        [str(blocker) for blocker in gate.get("blockers") or []]
    )
    evidence = gate.get("evidence") if isinstance(gate.get("evidence"), dict) else {}
    for system in evidence.get("blocking_subsystems") or []:
        if str(system):
            categories.add(str(system))
    minimal_red_defect = (
        evidence.get("minimal_red_defect")
        if isinstance(evidence.get("minimal_red_defect"), dict)
        else {}
    )
    subsystem = str(minimal_red_defect.get("blocking_subsystem") or "")
    if subsystem:
        categories.add(subsystem)
    return categories


def _validation_flow_summary(
    gates: dict[str, Any],
    required_names: set[str],
) -> list[dict[str, Any]]:
    flow: list[dict[str, Any]] = []
    for stage in ALGORITHM_VALIDATION_FLOW:
        gate_names = tuple(str(name) for name in stage["gates"])
        required_gates = [name for name in gate_names if name in required_names]
        passed_gates = [
            name for name in required_gates if bool((gates.get(name) or {}).get("ok"))
        ]
        blocking_gates = [name for name in required_gates if name not in passed_gates]
        if not required_gates:
            status = "not_required"
        elif blocking_gates:
            status = "failed"
        else:
            status = "passed"
        flow.append(
            {
                "id": stage["id"],
                "title": stage["title"],
                "status": status,
                "required_gates": _ordered_gate_names(set(required_gates)),
                "passed_gates": _ordered_gate_names(set(passed_gates)),
                "blocking_gates": _ordered_gate_names(set(blocking_gates)),
                "all_gates": list(gate_names),
                "proves": stage["proves"],
            }
        )
    return flow


def _algorithm_validation_summary(
    gates: dict[str, Any],
    required_names: set[str],
    missing_or_failed: list[str],
) -> dict[str, Any]:
    blocking_categories: dict[str, list[str]] = {}
    gate_categories: dict[str, list[str]] = {}
    for name in _ordered_gate_names(set(missing_or_failed)):
        gate = gates.get(name) or {}
        categories = sorted(_gate_failure_categories(gate) or {"not verified"})
        gate_categories[name] = categories
        for category in categories:
            blocking_categories.setdefault(category, []).append(name)

    claim_allowed = not missing_or_failed
    stop_condition = (
        "all required gates passed; simulation-only algorithm-health claim is allowed"
        if claim_allowed
        else (
            "do not claim algorithm health until these required gates pass: "
            + ", ".join(_ordered_gate_names(set(missing_or_failed)))
        )
    )
    next_actions = _next_actions(gates, missing_or_failed, gate_categories)
    return {
        "claim": "simulation_algorithm_health",
        "claim_allowed": claim_allowed,
        "flow_ok": claim_allowed,
        "highest_priority_blocker": next(
            iter(_ordered_gate_names(set(missing_or_failed))),
            "",
        ),
        "required_gate_sequence": _ordered_gate_names(required_names),
        "validation_flow": _validation_flow_summary(gates, required_names),
        "claim_boundary": {
            "global_planning_source": "static_saved_map_tomogram",
            "realtime_mapping_source": "fastlio2_lidar_imu",
            "local_dynamic_inputs": ["live_costmap", "moving_obstacles"],
            "live_costmap_role": "local_planning_and_safety_only",
            "simulation_only": True,
            "field_readiness": False,
        },
        "blocking_gate_count": len(missing_or_failed),
        "blocking_categories": blocking_categories,
        "gate_categories": gate_categories,
        "next_actions": next_actions,
        "stop_condition": stop_condition,
        "interpretation": (
            "simulation/replay evidence only; not physical S100P field readiness"
        ),
    }


_NEXT_ACTION_CATEGORY_PRIORITY = (
    "product_data_plane",
    "environment_runtime",
    "artifact_contract",
    "slam_localization",
    "dynamic_obstacle",
    "planning_tracking",
    "command_safety",
    "simulation_integration",
    "unclassified",
)


_NEXT_ACTION_TYPE_BY_CATEGORY = {
    "product_data_plane": "fix_gateway_runtime_acceptance_then_rerun",
    "environment_runtime": "fix_runtime_then_rerun",
    "artifact_contract": "generate_missing_report",
    "slam_localization": "debug_localization_then_rerun",
    "dynamic_obstacle": "rerun_dynamic_obstacle_matrix",
    "planning_tracking": "debug_planning_tracking_then_rerun",
    "command_safety": "fix_command_boundary_then_rerun",
    "simulation_integration": "fix_simulation_bridge_then_rerun",
    "unclassified": "inspect_gate_report_then_rerun",
}


def _primary_next_action_category(categories: list[str]) -> str:
    category_set = set(categories)
    for category in _NEXT_ACTION_CATEGORY_PRIORITY:
        if category in category_set:
            return category
    return categories[0] if categories else "unclassified"


def _next_actions(
    gates: dict[str, Any],
    missing_or_failed: list[str],
    gate_categories: dict[str, list[str]],
) -> list[dict[str, Any]]:
    actions: list[dict[str, Any]] = []
    for name in _ordered_gate_names(set(missing_or_failed)):
        gate = gates.get(name) or {}
        categories = list(gate_categories.get(name) or ["unclassified"])
        category = _primary_next_action_category(categories)
        actions.append(
            {
                "gate": name,
                "status": gate.get("status", "missing"),
                "category": category,
                "categories": categories,
                "action_type": _NEXT_ACTION_TYPE_BY_CATEGORY.get(
                    category,
                    "inspect_gate_report_then_rerun",
                ),
                "blockers": list(gate.get("blockers") or ["not verified"]),
                "command": gate.get("command", ""),
                "report_path": gate.get("path", ""),
                "expected_report_path": gate.get("expected_report_path", ""),
                "accepted_patterns": list(gate.get("accepted_patterns") or []),
                "host_requirements": list(gate.get("host_requirements") or []),
            }
        )
    return actions


def _python_tag() -> str:
    return f"py{sys.version_info.major}{sys.version_info.minor}"


def _default_module_available(name: str) -> bool:
    try:
        return importlib.util.find_spec(name) is not None
    except Exception:
        return False


def _default_path_exists(path: str | Path) -> bool:
    return Path(path).exists()


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _inspect_pct_runtime_for_host(machine: str | None) -> dict[str, Any]:
    try:
        from nav.services.plan.global_planner.algorithm.pct.runtime.api import inspect_pct_runtime

        return dict(inspect_pct_runtime(ROOT, machine=machine))
    except Exception as exc:
        return {
            "ok": False,
            "host_platform_supported": platform.system().lower() == "linux",
            "python_abi_matches_known_good": _python_tag() == "py310",
            "error": f"{type(exc).__name__}: {exc}",
        }


def _ros_hardware_subscribers(env: dict[str, str]) -> list[str] | None:
    if shutil.which("ros2") is None:
        return None
    subscribers: list[str] = []
    query_env = os.environ.copy()
    query_env.update(env)
    for topic in HARDWARE_COMMAND_TOPICS:
        try:
            result = subprocess.run(
                ["ros2", "topic", "info", topic, "--verbose"],
                cwd=ROOT,
                env=query_env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=2.0,
                check=False,
            )
        except Exception:
            continue
        if result.returncode != 0:
            continue
        for line in result.stdout.splitlines():
            lowered = line.lower()
            if any(keyword in lowered for keyword in HARDWARE_SUBSCRIBER_KEYWORDS):
                subscribers.append(f"{topic}: {line.strip()}")
    return subscribers


def _parse_ros2_package_executables_result(
    result: subprocess.CompletedProcess[str],
    *,
    package: str,
) -> list[str] | None:
    if result.returncode == 0:
        return [line.strip() for line in result.stdout.splitlines() if line.strip()]
    package_missing = (
        f"Package '{package}' not found" in (result.stderr or "")
        or f"Package '{package}' not found" in (result.stdout or "")
    )
    if package_missing:
        return []
    return None


def _ros2_package_executables(package: str, env: dict[str, str]) -> list[str] | None:
    query_env = os.environ.copy()
    query_env.update(env)

    ros_distro = str(query_env.get("ROS_DISTRO") or "humble")
    setup_bash = Path(f"/opt/ros/{ros_distro}/setup.bash")
    workspace_setup = ROOT / "install" / "setup.bash"
    if os.name == "posix" and shutil.which("bash") and (
        setup_bash.exists() or workspace_setup.exists()
    ):
        source_commands: list[str] = ["set +u"]
        if setup_bash.exists():
            source_commands.append(f"source {shlex.quote(str(setup_bash))}")
        if workspace_setup.exists():
            source_commands.append(f"source {shlex.quote(str(workspace_setup))}")
        source_commands.extend(
            [
                "set -u",
                "ros2 pkg executables " + shlex.quote(str(package)),
            ]
        )
        try:
            result = subprocess.run(
                ["bash", "-lc", "; ".join(source_commands)],
                cwd=ROOT,
                env=query_env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=5.0,
                check=False,
            )
        except Exception:
            result = None
        if result is not None:
            parsed = _parse_ros2_package_executables_result(
                result,
                package=package,
            )
            if parsed is not None:
                return parsed

    if shutil.which("ros2", path=query_env.get("PATH")) is None:
        return None
    try:
        result = subprocess.run(
            ["ros2", "pkg", "executables", package],
            cwd=ROOT,
            env=query_env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=5.0,
            check=False,
        )
    except Exception:
        return None
    parsed = _parse_ros2_package_executables_result(result, package=package)
    if parsed is not None:
        return parsed
    return []


def _add_check(
    checks: dict[str, dict[str, Any]],
    blockers: list[str],
    name: str,
    *,
    ok: bool,
    blocker: str = "",
    evidence: dict[str, Any] | None = None,
) -> None:
    payload = {
        "ok": bool(ok),
        "blocker": "" if ok else blocker,
        "evidence": evidence or {},
    }
    if not ok:
        payload["recommended_action"] = HOST_CHECK_ACTIONS.get(
            name,
            "inspect the failed host check and fix the simulation host",
        )
        payload["diagnostic_commands"] = list(
            HOST_CHECK_DIAGNOSTIC_COMMANDS.get(name, ())
        )
    checks[name] = payload
    if not ok and blocker:
        blockers.append(blocker)


def _failed_host_check_names(checks: dict[str, dict[str, Any]]) -> list[str]:
    return [
        name
        for name, check in checks.items()
        if isinstance(check, dict) and check.get("ok") is False
    ]


def _dedupe_commands(commands: list[str]) -> list[str]:
    seen: set[str] = set()
    deduped: list[str] = []
    for command in commands:
        if command and command not in seen:
            seen.add(command)
            deduped.append(command)
    return deduped


def _host_check_diagnostic_commands(check_names: list[str]) -> list[str]:
    commands: list[str] = []
    for name in check_names:
        commands.extend(HOST_CHECK_DIAGNOSTIC_COMMANDS.get(name, ()))
    return _dedupe_commands(commands)


def _host_check_recommended_action(check_names: list[str]) -> str:
    actions = [
        HOST_CHECK_ACTIONS.get(name, "inspect the failed host check and fix the simulation host")
        for name in check_names
        if name in HOST_CHECK_ACTIONS
    ]
    if not actions:
        return ""
    return "; ".join(dict.fromkeys(actions))


def _host_check_rank(check_name: str) -> int:
    try:
        return HOST_CHECK_PRIORITY.index(check_name)
    except ValueError:
        return len(HOST_CHECK_PRIORITY)


def _host_setup_plan(
    *,
    gates: dict[str, Any],
    runnable_gates: list[str],
    blocked_gates: list[str],
    current_host: dict[str, Any],
) -> dict[str, Any]:
    checks_by_name: dict[str, dict[str, Any]] = {}
    for gate_name, raw_gate in gates.items():
        gate = raw_gate if isinstance(raw_gate, dict) else {}
        checks = gate.get("checks") if isinstance(gate, dict) else {}
        if not isinstance(checks, dict):
            continue
        for check_name, raw_check in checks.items():
            check = raw_check if isinstance(raw_check, dict) else {}
            if check.get("ok") is not False:
                continue
            name = str(check_name)
            entry = checks_by_name.setdefault(
                name,
                {
                    "check": name,
                    "gates": [],
                    "blockers": [],
                    "evidence_samples": [],
                    "recommended_action": (
                        str(check.get("recommended_action") or "")
                        or HOST_CHECK_ACTIONS.get(
                            name,
                            "inspect the failed host check and fix the simulation host",
                        )
                    ),
                    "diagnostic_commands": list(
                        check.get("diagnostic_commands")
                        or HOST_CHECK_DIAGNOSTIC_COMMANDS.get(name, ())
                    ),
                },
            )
            entry["gates"].append(str(gate_name))
            blocker = str(check.get("blocker") or "")
            if blocker:
                entry["blockers"].append(blocker)
            evidence = check.get("evidence")
            if isinstance(evidence, dict) and evidence and len(entry["evidence_samples"]) < 3:
                entry["evidence_samples"].append(dict(evidence))

    failed_checks = sorted(
        checks_by_name.values(),
        key=lambda item: (_host_check_rank(str(item["check"])), str(item["check"])),
    )
    for item in failed_checks:
        item["gates"] = list(dict.fromkeys(item["gates"]))
        item["blockers"] = list(dict.fromkeys(item["blockers"]))
        item["diagnostic_commands"] = _dedupe_commands(
            [str(command) for command in item.get("diagnostic_commands", [])]
        )

    return {
        "checked": True,
        "source": "host_preflight_gates",
        "ok": not failed_checks,
        "current_host": dict(current_host),
        "ready_to_run_gates": list(runnable_gates),
        "blocked_gates": list(blocked_gates),
        "failed_check_count": len(failed_checks),
        "failed_checks": failed_checks,
        "stop_condition": (
            "host can run the selected DimOS gates"
            if not failed_checks
            else (
                "fix host checks before running blocked DimOS gates: "
                + ", ".join(str(item["check"]) for item in failed_checks)
            )
        ),
    }


def host_preflight(
    *,
    required: set[str],
    platform_system: str | None = None,
    machine: str | None = None,
    python_tag: str | None = None,
    env: dict[str, str] | None = None,
    executable_exists: Callable[[str], bool] | None = None,
    module_available: Callable[[str], bool] | None = None,
    path_exists: Callable[[str | Path], bool] | None = None,
    pct_runtime_report: dict[str, Any] | None = None,
    hardware_subscribers: Callable[[], list[str] | None] | None = None,
    ros2_package_executables: Callable[[str], list[str] | None] | None = None,
) -> dict[str, Any]:
    """Read-only host preflight for server-sim gate execution.

    This does not launch gate commands. It only checks whether the current host
    satisfies the static requirements attached to selected gate specs.
    """
    required_names = required or {spec.name for spec in GATES}
    specs = {spec.name: spec for spec in GATES}
    unknown = sorted(required_names - set(specs))
    if unknown:
        raise ValueError(f"unknown required gate(s): {', '.join(unknown)}")

    resolved_env = dict(os.environ if env is None else env)
    resolved_platform = platform_system or platform.system()
    resolved_machine = machine or platform.machine()
    resolved_python_tag = python_tag or _python_tag()
    exists = executable_exists or (lambda name: shutil.which(name) is not None)
    has_module = module_available or _default_module_available
    has_path = path_exists or _default_path_exists
    pct_report = (
        dict(pct_runtime_report)
        if pct_runtime_report is not None
        else _inspect_pct_runtime_for_host(resolved_machine)
    )
    subscriber_probe = hardware_subscribers or (
        lambda: _ros_hardware_subscribers(resolved_env)
    )
    ros2_executable_probe = ros2_package_executables or (
        lambda package: _ros2_package_executables(package, resolved_env)
    )

    ros_domain_id = str(resolved_env.get("ROS_DOMAIN_ID") or "")
    ros_distro = str(resolved_env.get("ROS_DISTRO") or "")
    mujoco_gl = str(resolved_env.get("MUJOCO_GL") or "")
    generated_at = time.time()

    gates: dict[str, Any] = {}
    for name in _ordered_gate_names(required_names):
        spec = specs[name]
        requirements = tuple(spec.host_requirements)
        requirement_text = "\n".join(requirements)
        checks: dict[str, dict[str, Any]] = {}
        blockers: list[str] = []

        if "local Python runtime only" in requirement_text:
            _add_check(
                checks,
                blockers,
                "local_non_motion",
                ok=True,
                evidence={"current_python_tag": resolved_python_tag},
            )

        numeric_runtime_label = ""
        if "stable NumPy/local planner runtime" in requirement_text:
            numeric_runtime_label = "local-planner"
        elif "stable NumPy/local simulation runtime" in requirement_text:
            numeric_runtime_label = "local simulation"

        if numeric_runtime_label:
            numpy_available = bool(has_module("numpy"))
            platform_ok = resolved_platform.lower() != "windows"
            local_numeric_ok = numpy_available and platform_ok
            numeric_blockers: list[str] = []
            if not numpy_available:
                numeric_blockers.append(
                    f"NumPy module unavailable for {numeric_runtime_label} gate"
                )
            if not platform_ok:
                numeric_blockers.append(
                    f"Windows/MINGW NumPy {numeric_runtime_label} runtime is not accepted; run this gate on Linux"
                )
            _add_check(
                checks,
                blockers,
                "local_numeric_nav",
                ok=local_numeric_ok,
                blocker="; ".join(numeric_blockers),
                evidence={
                    "platform_system": resolved_platform,
                    "machine": resolved_machine,
                    "numpy_module_available": numpy_available,
                },
            )

        if "PCT native extension modules" in requirement_text:
            platform_ok = resolved_platform.lower() == "linux"
            python_ok = resolved_python_tag == "py310"
            runtime_ok = bool(pct_report.get("ok"))
            pct_ok = platform_ok and python_ok and runtime_ok
            pct_blockers: list[str] = []
            if not platform_ok:
                pct_blockers.append(
                    f"PCT native runtime unavailable on {resolved_platform}; Linux/S100P required"
                )
            if not python_ok:
                pct_blockers.append(
                    f"PCT native runtime requires CPython 3.10 ABI; current {resolved_python_tag}"
                )
            if not runtime_ok:
                detail = str(pct_report.get("error") or "runtime inspection did not pass")
                pct_blockers.append(f"PCT native runtime unavailable: {detail}")
            _add_check(
                checks,
                blockers,
                "pct_native",
                ok=pct_ok,
                blocker="; ".join(pct_blockers),
                evidence=pct_report,
            )

        if "ROS 2 Humble" in requirement_text:
            setup_path = Path("/opt/ros/humble/setup.bash")
            ros_ok = ros_distro == "humble" or bool(has_path(setup_path))
            _add_check(
                checks,
                blockers,
                "ros2_humble",
                ok=ros_ok,
                blocker=(
                    "ROS 2 Humble environment unavailable; source "
                    "/opt/ros/humble/setup.bash on the isolated simulation host"
                ),
                evidence={
                    "ROS_DISTRO": ros_distro,
                    "setup_bash": str(setup_path),
                    "setup_bash_exists": bool(has_path(setup_path)),
                    "ros2_executable": bool(exists("ros2")),
                },
            )

        if "MuJoCo" in requirement_text:
            mujoco_ok = bool(has_module("mujoco")) and mujoco_gl.lower() in {
                "",
                "egl",
                "osmesa",
            }
            _add_check(
                checks,
                blockers,
                "mujoco_headless",
                ok=mujoco_ok,
                blocker="MuJoCo headless runtime unavailable or MUJOCO_GL is not egl/osmesa",
                evidence={
                    "module_available": bool(has_module("mujoco")),
                    "MUJOCO_GL": mujoco_gl,
                },
            )

        if "MID-360 scan pattern asset" in requirement_text:
            pattern_path = ROOT / MID360_PATTERN_REL
            pattern_exists = bool(has_path(pattern_path))
            pattern_sha = ""
            sha_ok: bool | None = None
            if pattern_exists and path_exists is None:
                try:
                    pattern_sha = _file_sha256(pattern_path)
                    sha_ok = pattern_sha == MID360_PATTERN_SHA256
                except Exception:
                    sha_ok = False
            mid360_ok = pattern_exists and (sha_ok is not False)
            if not pattern_exists:
                mid360_blocker = (
                    f"official MID-360 scan pattern asset missing: {MID360_PATTERN_REL}"
                )
            elif sha_ok is False:
                mid360_blocker = (
                    "official MID-360 scan pattern asset SHA256 mismatch: "
                    f"{pattern_sha or 'unreadable'}"
                )
            else:
                mid360_blocker = ""
            _add_check(
                checks,
                blockers,
                "mid360_pattern",
                ok=mid360_ok,
                blocker=mid360_blocker,
                evidence={
                    "path": str(pattern_path),
                    "relative_path": MID360_PATTERN_REL,
                    "exists": pattern_exists,
                    "expected_sha256": MID360_PATTERN_SHA256,
                    "sha256": pattern_sha,
                    "sha256_checked": sha_ok is not None,
                },
            )

        if "product MuJoCo world asset" in requirement_text:
            world_asset_path = ROOT / MUJOCO_WORLD_ASSET_REL
            world_asset_exists = bool(has_path(world_asset_path))
            _add_check(
                checks,
                blockers,
                "mujoco_world_asset",
                ok=world_asset_exists,
                blocker=(
                    f"product MuJoCo world asset missing: {MUJOCO_WORLD_ASSET_REL}"
                ),
                evidence={
                    "path": str(world_asset_path),
                    "relative_path": MUJOCO_WORLD_ASSET_REL,
                    "exists": world_asset_exists,
                },
            )

        if "Gazebo" in requirement_text:
            gazebo_ok = bool(exists("gz") or exists("ign"))
            _add_check(
                checks,
                blockers,
                "gazebo_runtime",
                ok=gazebo_ok,
                blocker="Gazebo/Ignition executable unavailable on this host",
                evidence={"gz": bool(exists("gz")), "ign": bool(exists("ign"))},
            )

        if "Gazebo navigation source scripts" in requirement_text:
            source_paths = [ROOT / rel for rel in GAZEBO_NAV_SOURCE_RELS]
            missing_sources = [
                rel
                for rel, path in zip(GAZEBO_NAV_SOURCE_RELS, source_paths)
                if not bool(has_path(path))
            ]
            _add_check(
                checks,
                blockers,
                "gazebo_navigation_sources",
                ok=not missing_sources,
                blocker=(
                    "Gazebo navigation source script(s) missing: "
                    + ", ".join(missing_sources)
                )
                if missing_sources
                else "",
                evidence={
                    "required_relative_paths": list(GAZEBO_NAV_SOURCE_RELS),
                    "missing_relative_paths": missing_sources,
                    "paths": [str(path) for path in source_paths],
                },
            )

        needs_isolation = (
            "isolated" in requirement_text.lower()
            or "no physical robot drivers" in requirement_text.lower()
            or "no hardware command" in requirement_text.lower()
        )
        if needs_isolation:
            domain_ok = bool(ros_domain_id) and ros_domain_id != "0"
            _add_check(
                checks,
                blockers,
                "isolated_ros_domain",
                ok=domain_ok,
                blocker=(
                    "ROS_DOMAIN_ID must be set to a nonzero isolated simulation domain "
                    "before running this gate"
                ),
                evidence={"ROS_DOMAIN_ID": ros_domain_id},
            )

            subscribers = subscriber_probe()
            subscriber_ok = subscribers == []
            if subscribers is None:
                subscriber_blocker = (
                    "hardware subscriber audit unavailable; run with ros2 available "
                    "or provide a zero-subscriber audit before gate execution"
                )
            elif subscribers:
                subscriber_blocker = (
                    "hardware command subscribers present: " + ", ".join(subscribers)
                )
            else:
                subscriber_blocker = ""
            _add_check(
                checks,
                blockers,
                "hardware_subscribers",
                ok=subscriber_ok,
                blocker=subscriber_blocker,
                evidence={
                    "topics": list(HARDWARE_COMMAND_TOPICS),
                    "subscribers": [] if subscribers is None else list(subscribers),
                    "audit_available": subscribers is not None,
                },
            )

        if "fastlio2 package" in requirement_text:
            fastlio2_executables = ros2_executable_probe("fastlio2")
            executable_names = {
                line.split()[-1]
                for line in (fastlio2_executables or [])
                if str(line).strip()
            }
            required_executables = {"lio_node"}
            missing_executables = sorted(required_executables - executable_names)
            fastlio2_ok = (
                fastlio2_executables is not None
                and not missing_executables
            )
            if fastlio2_executables is None:
                fastlio2_blocker = (
                    "ROS 2 fastlio2 package audit unavailable; source ROS 2 "
                    "and the LingTu/Fast-LIO2 install space before running this gate"
                )
            elif missing_executables:
                fastlio2_blocker = (
                    "ROS 2 fastlio2 package missing required executable(s): "
                    + ", ".join(missing_executables)
                )
            else:
                fastlio2_blocker = ""
            _add_check(
                checks,
                blockers,
                "ros2_fastlio2",
                ok=fastlio2_ok,
                blocker=fastlio2_blocker,
                evidence={
                    "package": "fastlio2",
                    "audit_available": fastlio2_executables is not None,
                    "executables": list(fastlio2_executables or []),
                    "required_executables": sorted(required_executables),
                    "missing_executables": missing_executables,
                },
            )

        if "local_planner package" in requirement_text:
            local_planner_executables = ros2_executable_probe("local_planner")
            executable_names = {
                line.split()[-1]
                for line in (local_planner_executables or [])
                if str(line).strip()
            }
            required_executables = {"localPlanner", "pathFollower"}
            missing_executables = sorted(required_executables - executable_names)
            local_planner_ok = (
                local_planner_executables is not None
                and not missing_executables
            )
            if local_planner_executables is None:
                local_planner_blocker = (
                    "ROS 2 local_planner package audit unavailable; source ROS 2 "
                    "and the LingTu install space before running this gate"
                )
            elif missing_executables:
                local_planner_blocker = (
                    "ROS 2 local_planner package missing required executable(s): "
                    + ", ".join(missing_executables)
                )
            else:
                local_planner_blocker = ""
            _add_check(
                checks,
                blockers,
                "ros2_local_planner",
                ok=local_planner_ok,
                blocker=local_planner_blocker,
                evidence={
                    "package": "local_planner",
                    "audit_available": local_planner_executables is not None,
                    "executables": list(local_planner_executables or []),
                    "required_executables": sorted(required_executables),
                    "missing_executables": missing_executables,
                },
            )

        if "pct_adapters package" in requirement_text:
            pct_adapter_executables = ros2_executable_probe("pct_adapters")
            executable_names = {
                line.split()[-1]
                for line in (pct_adapter_executables or [])
                if str(line).strip()
            }
            required_executables = {"pct_path_adapter"}
            missing_executables = sorted(required_executables - executable_names)
            pct_adapter_ok = (
                pct_adapter_executables is not None
                and not missing_executables
            )
            if pct_adapter_executables is None:
                pct_adapter_blocker = (
                    "ROS 2 pct_adapters package audit unavailable; source ROS 2 "
                    "and the LingTu install space before running this gate"
                )
            elif missing_executables:
                pct_adapter_blocker = (
                    "ROS 2 pct_adapters package missing required executable(s): "
                    + ", ".join(missing_executables)
                )
            else:
                pct_adapter_blocker = ""
            _add_check(
                checks,
                blockers,
                "ros2_pct_adapters",
                ok=pct_adapter_ok,
                blocker=pct_adapter_blocker,
                evidence={
                    "package": "pct_adapters",
                    "audit_available": pct_adapter_executables is not None,
                    "executables": list(pct_adapter_executables or []),
                    "required_executables": sorted(required_executables),
                    "missing_executables": missing_executables,
                },
            )

        if "localizer runtime" in requirement_text:
            localizer_executables = ros2_executable_probe("localizer")
            executable_names = {
                line.split()[-1]
                for line in (localizer_executables or [])
                if str(line).strip()
            }
            required_executables = {"localizer_node"}
            missing_executables = sorted(required_executables - executable_names)
            localizer_ok = (
                localizer_executables is not None
                and not missing_executables
            )
            if localizer_executables is None:
                localizer_blocker = (
                    "ROS 2 localizer package audit unavailable; source ROS 2 "
                    "and the LingTu/localizer install space before running this gate"
                )
            elif missing_executables:
                localizer_blocker = (
                    "ROS 2 localizer package missing required executable(s): "
                    + ", ".join(missing_executables)
                )
            else:
                localizer_blocker = ""
            _add_check(
                checks,
                blockers,
                "localizer_runtime",
                ok=localizer_ok,
                blocker=localizer_blocker,
                evidence={
                    "package": "localizer",
                    "audit_available": localizer_executables is not None,
                    "executables": list(localizer_executables or []),
                    "required_executables": sorted(required_executables),
                    "missing_executables": missing_executables,
                },
            )

        ok = not blockers
        failed_checks = _failed_host_check_names(checks)
        gates[name] = {
            "ok": ok,
            "status": "runnable" if ok else "blocked",
            "description": spec.description,
            "host_requirements": list(requirements),
            "checks": checks,
            "failed_checks": failed_checks,
            "blockers": blockers,
            "recommended_action": _host_check_recommended_action(failed_checks),
            "diagnostic_commands": _host_check_diagnostic_commands(failed_checks),
            "command": spec.command,
            "expected_report_path": _expected_report_path(spec),
            "accepted_patterns": list(spec.default_patterns),
        }

    blocked_gates = [
        name for name in _ordered_gate_names(required_names) if not gates[name]["ok"]
    ]
    runnable_gates = [
        name for name in _ordered_gate_names(required_names) if gates[name]["ok"]
    ]
    current_host = {
        "platform_system": resolved_platform,
        "machine": resolved_machine,
        "python_tag": resolved_python_tag,
        "ROS_DOMAIN_ID": ros_domain_id,
        "ROS_DISTRO": ros_distro,
        "MUJOCO_GL": mujoco_gl,
    }
    return {
        "schema_version": "lingtu.server_sim_host_preflight.v1",
        "execution_mode": "host_preflight_only",
        "report_contract_checked": True,
        "report_freshness": {
            "checked": True,
            "source": "live_host_preflight",
            "generated_at": generated_at,
            "fresh": True,
            "stale": False,
            "blockers": [],
        },
        "ok": not blocked_gates,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "run_missing": False,
        "gate_runs": [],
        "generated_at": generated_at,
        "required_gate_sequence": _ordered_gate_names(required_names),
        "current_host": current_host,
        "runnable_gates": runnable_gates,
        "blocked_gates": blocked_gates,
        "gates": gates,
        "host_setup_plan": _host_setup_plan(
            gates=gates,
            runnable_gates=runnable_gates,
            blocked_gates=blocked_gates,
            current_host=current_host,
        ),
        "next_actions": [
            {
                "gate": name,
                "action_type": "fix_host_preflight_then_run_gate",
                "blockers": list(gates[name]["blockers"]),
                "failed_checks": list(gates[name]["failed_checks"]),
                "recommended_action": gates[name]["recommended_action"],
                "diagnostic_commands": list(gates[name]["diagnostic_commands"]),
                "command": gates[name]["command"],
                "expected_report_path": gates[name]["expected_report_path"],
                "host_requirements": list(gates[name]["host_requirements"]),
            }
            for name in blocked_gates
        ],
    }


def _candidate_matches(patterns: tuple[str, ...]) -> list[Path]:
    seen: set[Path] = set()
    candidates: list[Path] = []
    for pattern in patterns:
        for path in ROOT.glob(pattern):
            if path.is_file() and path not in seen:
                seen.add(path)
                candidates.append(path)
    return sorted(candidates, key=lambda path: path.stat().st_mtime, reverse=True)


def _best_match(
    spec: GateSpec,
    *,
    max_report_age_s: float | None = None,
    generated_at: float | None = None,
) -> Path | None:
    candidates = _candidate_matches(spec.default_patterns)
    if not candidates:
        return None
    if max_report_age_s is not None:
        reference_time = time.time() if generated_at is None else generated_at
        fresh_candidates = [
            path
            for path in candidates
            if max(0.0, reference_time - path.stat().st_mtime) <= max_report_age_s
        ]
        if fresh_candidates:
            candidates = fresh_candidates
    for path in candidates:
        try:
            ok, _, _ = spec.evaluator(_load_json(path))
        except Exception:
            ok = False
        if ok:
            return path
    return candidates[0]


def summarize(
    *,
    report_overrides: dict[str, Path],
    required: set[str],
    max_report_age_s: float | None = None,
    include_optional: bool = True,
) -> dict[str, Any]:
    gates: dict[str, Any] = {}
    generated_at = time.time()
    required_names = required or {spec.name for spec in GATES}
    selected_specs = (
        GATES
        if include_optional
        else tuple(spec for spec in GATES if spec.name in required_names)
    )
    for spec in selected_specs:
        spec_max_report_age_s = max_report_age_s
        if (
            spec_max_report_age_s is None
            and spec.name in required_names
            and spec.name in DEFAULT_FRESHNESS_REQUIRED_GATES
        ):
            spec_max_report_age_s = DEFAULT_REQUIRED_MAX_REPORT_AGE_S
        path = report_overrides.get(spec.name) or _best_match(
            spec,
            max_report_age_s=spec_max_report_age_s,
            generated_at=generated_at,
        )
        if path is None:
            gates[spec.name] = {
                "description": spec.description,
                "exists": False,
                "ok": False,
                "status": "missing",
                "blockers": ["report missing"],
                "path": "",
                "expected_report_path": _expected_report_path(spec),
                "accepted_patterns": list(spec.default_patterns),
                "command": spec.command,
                "host_requirements": list(spec.host_requirements),
                "is_fresh": False,
            }
            continue
        if not path.exists():
            gates[spec.name] = {
                "description": spec.description,
                "exists": False,
                "ok": False,
                "status": "missing",
                "blockers": [f"report missing: {path}"],
                "path": str(path),
                "expected_report_path": _expected_report_path(spec),
                "accepted_patterns": list(spec.default_patterns),
                "command": spec.command,
                "host_requirements": list(spec.host_requirements),
                "is_fresh": False,
            }
            continue
        report_mtime = path.stat().st_mtime
        report_age_s = max(0.0, generated_at - report_mtime)
        freshness_blockers: list[str] = []
        if spec_max_report_age_s is not None and report_age_s > spec_max_report_age_s:
            freshness_blockers.append(
                f"report_age_s {report_age_s:.3f} > max_report_age_s {spec_max_report_age_s:.3f}"
            )
        try:
            report = _load_json(path)
            report_timestamp = report.get("generated_at") or report.get("timestamp") or report_mtime
            ok, blockers, evidence = spec.evaluator(report)
            blockers = [*blockers, *freshness_blockers]
            ok = bool(ok) and not freshness_blockers
            gates[spec.name] = {
                "description": spec.description,
                "exists": True,
                "ok": bool(ok),
                "status": "passed" if ok else "failed",
                "blockers": blockers,
                "path": str(path),
                "report_mtime": report_mtime,
                "report_age_s": round(report_age_s, 3),
                "report_timestamp": report_timestamp,
                "max_report_age_s": spec_max_report_age_s,
                "is_fresh": not freshness_blockers,
                "expected_report_path": _expected_report_path(spec),
                "accepted_patterns": list(spec.default_patterns),
                "command": spec.command,
                "host_requirements": list(spec.host_requirements),
                "evidence": evidence,
            }
        except Exception as exc:
            gates[spec.name] = {
                "description": spec.description,
                "exists": True,
                "ok": False,
                "status": "invalid",
                "blockers": [str(exc), *freshness_blockers],
                "path": str(path),
                "report_mtime": report_mtime,
                "report_age_s": round(report_age_s, 3),
                "max_report_age_s": spec_max_report_age_s,
                "is_fresh": not freshness_blockers,
                "expected_report_path": _expected_report_path(spec),
                "accepted_patterns": list(spec.default_patterns),
                "command": spec.command,
                "host_requirements": list(spec.host_requirements),
            }

    missing_or_failed = [
        name
        for name in _ordered_gate_names(required_names)
        if not bool((gates.get(name) or {}).get("ok"))
    ]
    optional_missing_or_failed = [
        name
        for name in _ordered_gate_names(set(gates) - required_names)
        if not bool((gates.get(name) or {}).get("ok"))
    ]
    verified = {name: bool(item.get("ok")) for name, item in gates.items()}
    algorithm_backends = _algorithm_backends_from_gates(gates)
    algorithm_validation = _algorithm_validation_summary(
        gates,
        required_names,
        missing_or_failed,
    )
    return {
        "schema_version": "lingtu.server_sim_closure.v1",
        "execution_mode": "summary_only",
        "run_missing": False,
        "initial_missing_or_failed": [],
        "gate_runs": [],
        "ok": not missing_or_failed,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "generated_at": generated_at,
        "include_optional": include_optional,
        "max_report_age_s": max_report_age_s,
        "required": sorted(required_names),
        "required_gate_sequence": _ordered_gate_names(required_names),
        "host_requirements": _host_requirements_for_gates(required_names),
        "verified": verified,
        "missing_or_failed": missing_or_failed,
        "missing_required_commands": _missing_required_commands(gates, missing_or_failed),
        "optional_missing_or_failed": optional_missing_or_failed,
        "algorithm_backends": algorithm_backends,
        "algorithm_validation": algorithm_validation,
        "next_actions": algorithm_validation["next_actions"],
        "gates": gates,
        "remaining_gaps": [
            f"{name}: {', '.join((gates.get(name) or {}).get('blockers') or ['not verified'])}"
            for name in missing_or_failed
        ],
        "optional_gaps": [
            f"{name}: {', '.join((gates.get(name) or {}).get('blockers') or ['not verified'])}"
            for name in optional_missing_or_failed
        ],
    }


def _dedupe_blockers(blockers: list[str]) -> list[str]:
    seen: set[str] = set()
    deduped: list[str] = []
    for blocker in blockers:
        blocker_text = str(blocker).strip()
        if not blocker_text or blocker_text in seen:
            continue
        seen.add(blocker_text)
        deduped.append(blocker_text)
    return deduped


def _run_failure_status(status: str) -> str:
    return {
        "failed": "execution_failed",
        "timeout": "execution_timeout",
        "error": "execution_error",
        "report_failed": "report_failed_after_run",
        "host_blocked": "host_blocked",
        "dependency_blocked": "dependency_blocked",
    }.get(status, status or "execution_failed")


def _gate_report_written_during_run(
    gate: dict[str, Any],
    run: dict[str, Any],
) -> bool:
    if not bool(gate.get("exists")):
        return False
    try:
        report_mtime = float(gate.get("report_mtime"))
        started_at = float(run.get("started_at"))
    except (TypeError, ValueError):
        return False
    return report_mtime >= started_at - 1.0


def _apply_run_missing_gate_overrides(
    summary: dict[str, Any],
    run_results: list[dict[str, Any]],
) -> dict[str, Any]:
    """Keep the final summary tied to this run instead of older matched reports."""

    gates = summary.get("gates") if isinstance(summary.get("gates"), dict) else {}
    specs = {spec.name: spec for spec in GATES}
    for run in run_results:
        name = str(run.get("name") or "")
        status = str(run.get("status") or "")
        if not name or status == "passed":
            continue
        spec = specs.get(name)
        if spec is None:
            continue
        current_gate = gates.get(name) if isinstance(gates.get(name), dict) else {}
        report_from_this_run = _gate_report_written_during_run(current_gate, run)
        effective_status = status
        if (
            status == "failed"
            and report_from_this_run
            and not bool(current_gate.get("ok"))
        ):
            effective_status = "report_failed"
        blockers: list[str] = []
        error = str(run.get("error") or "").strip()
        if error:
            blockers.append(error)
        if effective_status == "failed":
            returncode = run.get("returncode")
            if returncode is None:
                blockers.append("gate command failed")
            else:
                blockers.append(f"gate command failed with returncode {returncode}")
        elif effective_status == "timeout":
            blockers.append("gate command timed out")
        elif effective_status == "error":
            blockers.append("gate command raised before producing a valid report")
        elif effective_status == "report_failed":
            blockers.extend(str(item) for item in run.get("report_blockers") or [])
            blockers.extend(str(item) for item in current_gate.get("blockers") or [])
            if status == "failed" and run.get("returncode") is not None:
                blockers.append(
                    f"gate command exited nonzero after writing failure report: "
                    f"returncode {run.get('returncode')}"
                )
            if not blockers:
                blockers.append("gate report did not pass after command")
        elif effective_status == "host_blocked":
            blockers.append("host preflight blocked this gate")
        elif effective_status == "dependency_blocked":
            dependencies = list(run.get("dependency_blockers") or [])
            blockers.append(
                "blocked by unmet prerequisite gate(s): " + ", ".join(dependencies)
                if dependencies
                else "blocked by unmet prerequisite gate(s)"
            )
        if bool(current_gate.get("ok")):
            blockers.append(
                "current run did not pass; ignoring any matching prior report for this gate"
            )

        gates[name] = {
            **current_gate,
            "description": current_gate.get("description") or spec.description,
            "exists": bool(current_gate.get("exists")),
            "ok": False,
            "status": _run_failure_status(effective_status),
            "blockers": _dedupe_blockers(blockers or ["gate did not pass in this run"]),
            "path": current_gate.get("path") or str(run.get("report_path") or ""),
            "expected_report_path": _expected_report_path(spec),
            "accepted_patterns": list(spec.default_patterns),
            "command": spec.command,
            "host_requirements": list(spec.host_requirements),
            "run_status": status,
            "effective_run_status": effective_status,
            "report_from_this_run": report_from_this_run,
            "returncode": run.get("returncode"),
            "started_at": run.get("started_at"),
            "finished_at": run.get("finished_at"),
            "elapsed_s": run.get("elapsed_s"),
            "executed_command": run.get("executed_command"),
            "shell": run.get("shell"),
            "report_status": run.get("report_status"),
            "report_path": run.get("report_path"),
            "report_blockers": list(run.get("report_blockers") or []),
        }

    required_names = set(str(name) for name in summary.get("required") or [])
    missing_or_failed = [
        name
        for name in _ordered_gate_names(required_names)
        if not bool((gates.get(name) or {}).get("ok"))
    ]
    optional_missing_or_failed = [
        name
        for name in _ordered_gate_names(set(gates) - required_names)
        if not bool((gates.get(name) or {}).get("ok"))
    ]
    summary["gates"] = gates
    summary["verified"] = {name: bool(item.get("ok")) for name, item in gates.items()}
    summary["missing_or_failed"] = missing_or_failed
    summary["missing_required_commands"] = _missing_required_commands(
        gates,
        missing_or_failed,
    )
    summary["optional_missing_or_failed"] = optional_missing_or_failed
    summary["algorithm_backends"] = _algorithm_backends_from_gates(gates)
    algorithm_validation = _algorithm_validation_summary(
        gates,
        required_names,
        missing_or_failed,
    )
    summary["algorithm_validation"] = algorithm_validation
    summary["next_actions"] = algorithm_validation["next_actions"]
    summary["ok"] = not missing_or_failed
    summary["remaining_gaps"] = [
        f"{name}: {', '.join((gates.get(name) or {}).get('blockers') or ['not verified'])}"
        for name in missing_or_failed
    ]
    summary["optional_gaps"] = [
        f"{name}: {', '.join((gates.get(name) or {}).get('blockers') or ['not verified'])}"
        for name in optional_missing_or_failed
    ]
    return summary


def run_missing_required_gates(
    *,
    report_overrides: dict[str, Path],
    required: set[str],
    max_report_age_s: float | None = None,
    include_optional: bool = True,
    gate_timeout_s: float | None = None,
    stop_on_run_failure: bool = False,
    skip_host_blocked: bool = False,
    runner: Callable[..., subprocess.CompletedProcess[Any]] = subprocess.run,
) -> dict[str, Any]:
    initial_summary = summarize(
        report_overrides=report_overrides,
        required=required,
        max_report_age_s=max_report_age_s,
        include_optional=include_optional,
    )
    specs = {spec.name: spec for spec in GATES}
    run_results: list[dict[str, Any]] = []
    initial_missing = set(initial_summary["missing_or_failed"])
    dependency_names: set[str] = set()
    pending_dependency_scan = set(initial_missing) | set(required)
    while pending_dependency_scan:
        gate_name = pending_dependency_scan.pop()
        for dependency in GATE_RUN_DEPENDENCIES.get(gate_name, ()):
            if dependency in dependency_names:
                continue
            dependency_names.add(dependency)
            pending_dependency_scan.add(dependency)
    dependency_summary = summarize(
        report_overrides=report_overrides,
        required=dependency_names,
        max_report_age_s=max_report_age_s,
        include_optional=False,
    ) if dependency_names else {"gates": {}}
    completed_ok = {
        name
        for name, gate in {
            **(dependency_summary.get("gates") or {}),
            **(initial_summary.get("gates") or {}),
        }.items()
        if bool((gate or {}).get("ok"))
    }
    run_missing_preflight: dict[str, Any] | None = None
    host_blocked: set[str] = set()
    if skip_host_blocked and initial_missing:
        run_missing_preflight = host_preflight(required=initial_missing)
        host_blocked = set(run_missing_preflight.get("blocked_gates") or [])

    for name in _ordered_gate_names(initial_missing):
        spec = specs.get(name)
        if spec is None:
            continue
        if name in host_blocked:
            gate_preflight = (run_missing_preflight or {}).get("gates", {}).get(name, {})
            blockers = list(gate_preflight.get("blockers") or ["host preflight blocked"])
            preflight_checks = gate_preflight.get("checks") if isinstance(gate_preflight.get("checks"), dict) else {}
            failed_checks = list(
                gate_preflight.get("failed_checks")
                or _failed_host_check_names(preflight_checks)
            )
            diagnostic_commands = list(
                gate_preflight.get("diagnostic_commands")
                or _host_check_diagnostic_commands(failed_checks)
            )
            recommended_action = str(
                gate_preflight.get("recommended_action")
                or _host_check_recommended_action(failed_checks)
            )
            run_results.append(
                {
                    "name": name,
                    "status": "host_blocked",
                    "returncode": None,
                    "elapsed_s": 0.0,
                    "command": spec.command,
                    "executed_command": None,
                    "shell": None,
                    "error": "; ".join(blockers),
                    "failed_checks": failed_checks,
                    "recommended_action": recommended_action,
                    "diagnostic_commands": diagnostic_commands,
                    "host_preflight": gate_preflight,
                }
            )
            continue
        unmet_dependencies = [
            dependency
            for dependency in GATE_RUN_DEPENDENCIES.get(name, ())
            if dependency not in completed_ok
        ]
        if unmet_dependencies:
            run_results.append(
                {
                    "name": name,
                    "status": "dependency_blocked",
                    "returncode": None,
                    "elapsed_s": 0.0,
                    "command": spec.command,
                    "executed_command": None,
                    "shell": None,
                    "error": "blocked by unmet prerequisite gate(s): "
                    + ", ".join(unmet_dependencies),
                    "dependency_blockers": unmet_dependencies,
                }
            )
            continue
        started = time.time()
        returncode: int | None = None
        error = ""
        status = "failed"
        report_gate: dict[str, Any] = {}
        run_command, invocation = _normalized_gate_run_invocation(spec.command)
        try:
            result = runner(
                run_command,
                cwd=ROOT,
                **invocation,
                timeout=None if gate_timeout_s is None else float(gate_timeout_s),
                check=False,
            )
            returncode = int(result.returncode)
            status = "passed" if returncode == 0 else "failed"
            if returncode == 0:
                gate_summary = summarize(
                    report_overrides=report_overrides,
                    required={name},
                    max_report_age_s=max_report_age_s,
                    include_optional=False,
                )
                report_gate = gate_summary.get("gates", {}).get(name, {})
                if bool(report_gate.get("ok")):
                    completed_ok.add(name)
                else:
                    status = "report_failed"
                    error = "; ".join(
                        str(item)
                        for item in (
                            report_gate.get("blockers")
                            or ["gate report did not pass after command"]
                        )
                    )
        except subprocess.TimeoutExpired as exc:
            status = "timeout"
            error = f"timeout after {gate_timeout_s:g}s: {exc}"
        except Exception as exc:  # pragma: no cover - CLI diagnostics path.
            status = "error"
            error = f"{type(exc).__name__}: {exc}"

        finished = time.time()
        run_results.append(
            {
                "name": name,
                "status": status,
                "returncode": returncode,
                "started_at": started,
                "finished_at": finished,
                "elapsed_s": round(finished - started, 3),
                "command": spec.command,
                "executed_command": run_command,
                "shell": bool(invocation.get("shell")),
                "error": error,
                "report_status": report_gate.get("status"),
                "report_path": report_gate.get("path"),
                "report_blockers": list(report_gate.get("blockers") or []),
            }
        )
        if stop_on_run_failure and status != "passed":
            break

    final_summary = summarize(
        report_overrides=report_overrides,
        required=required,
        max_report_age_s=max_report_age_s,
        include_optional=include_optional,
    )
    final_summary["run_missing"] = True
    final_summary["execution_mode"] = "run_missing"
    final_summary["initial_missing_or_failed"] = list(initial_summary["missing_or_failed"])
    final_summary["gate_runs"] = run_results
    final_summary["skip_host_blocked"] = bool(skip_host_blocked)
    final_summary["skipped_host_blocked_gates"] = [
        item["name"] for item in run_results if item.get("status") == "host_blocked"
    ]
    final_summary["skipped_dependency_blocked_gates"] = [
        item["name"] for item in run_results if item.get("status") == "dependency_blocked"
    ]
    if run_missing_preflight is not None:
        final_summary["run_missing_host_preflight"] = run_missing_preflight
    return _apply_run_missing_gate_overrides(final_summary, run_results)


def materialize_navigation_replay_deviation_topic_jsonl(
    topic_jsonl_path: Path,
    *,
    report_path: Path | None = None,
    trace_path: Path | None = None,
) -> tuple[Path, dict[str, Any]]:
    """Build a navigation replay/deviation report from a recorded topic JSONL."""
    from sim.scripts.navigation_replay_deviation_gate import build_report

    report_path = report_path or ROOT / NAVIGATION_REPLAY_DEVIATION_REPORT_REL
    trace_path = trace_path or ROOT / NAVIGATION_REPLAY_DEVIATION_TRACE_REL
    report = build_report(topic_jsonl_path=topic_jsonl_path, write_trace=trace_path)
    _write_json(report_path, report)
    return report_path, {
        "gate": "navigation_replay_deviation",
        "source": "recorded_topic_jsonl",
        "source_path": str(topic_jsonl_path),
        "report_path": str(report_path),
        "trace_path": str(trace_path) if report.get("trace_artifact") else "",
        "ok": report.get("ok") is True,
        "trace_kind": report.get("trace_kind") or "",
        "remaining_gaps": list(report.get("remaining_gaps") or []),
    }


def materialize_moving_obstacle_sweep_report_only(
    *,
    report_globs: list[str],
    report_path: Path | None = None,
    allow_missing_video_file: bool = False,
) -> tuple[Path, dict[str, Any]]:
    """Build a moving-obstacle sweep aggregate from existing child reports."""
    from sim.scripts import moving_obstacle_sweep_gate

    report_path = report_path or ROOT / MOVING_OBSTACLE_SWEEP_REPORT_REL
    child_reports = moving_obstacle_sweep_gate._discover_reports((), report_globs)
    report = moving_obstacle_sweep_gate.evaluate_reports(
        child_reports,
        required_speed_bins=("slow", "fast"),
        required_density_bins=("sparse", "dense"),
        min_clearance_m=0.25,
        require_video=True,
        require_video_file=not allow_missing_video_file,
        require_live_nav_chain=True,
        required_scan_time_profile="physical_rolling",
    )
    report["execution_mode"] = "report_only_aggregate"
    report["run_matrix"] = {
        "enabled": False,
        "reason": "server_sim_closure materialized aggregate from existing child reports",
    }
    report["claim_boundary"] = (
        "report_only_aggregate_requires_child_live_fastlio_pct_cmd_vel_evidence"
    )
    report["source_report_globs"] = list(report_globs)
    report["source_child_reports"] = [str(path) for path in child_reports]
    _write_json(report_path, report)
    return report_path, {
        "gate": "moving_obstacle_sweep",
        "source": "existing_child_reports",
        "source_report_globs": list(report_globs),
        "source_child_reports": [str(path) for path in child_reports],
        "report_path": str(report_path),
        "ok": report.get("ok") is True,
        "case_count": report.get("case_count"),
        "passed_pair_count": report.get("passed_pair_count"),
        "missing_pairs": list(report.get("missing_pairs") or []),
        "blockers": list(report.get("blockers") or []),
        "claim_boundary": report.get("claim_boundary") or "",
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gateway-runtime-acceptance-report", type=Path, default=None)
    parser.add_argument("--multifloor-exploration-report", type=Path, default=None)
    parser.add_argument("--large-terrain-report", type=Path, default=None)
    parser.add_argument("--native-pct-mujoco-report", type=Path, default=None)
    parser.add_argument("--dynamic-obstacle-local-planner-report", type=Path, default=None)
    parser.add_argument("--fastlio2-live-report", type=Path, default=None)
    parser.add_argument("--fastlio2-dynamic-inspection-report", type=Path, default=None)
    parser.add_argument("--moving-obstacle-sweep-report", type=Path, default=None)
    parser.add_argument(
        "--moving-obstacle-sweep-report-glob",
        action="append",
        default=[],
        help=(
            "Materialize moving_obstacle_sweep from existing child reports "
            "matching this glob before summarizing. This reads reports only and "
            "does not launch the ROS2/MuJoCo matrix."
        ),
    )
    parser.add_argument(
        "--moving-obstacle-sweep-report-only-allow-missing-video-file",
        action="store_true",
        help=(
            "With --moving-obstacle-sweep-report-glob, accept video counters "
            "without requiring the referenced MP4 files to exist. Intended for "
            "local fixture tests only; DimOS evidence should omit this flag."
        ),
    )
    parser.add_argument("--large-loop-closure-report", type=Path, default=None)
    parser.add_argument("--mujoco-tare-exploration-report", type=Path, default=None)
    parser.add_argument("--policy-nav-report", type=Path, default=None)
    parser.add_argument("--gateway-dry-run-report", type=Path, default=None)
    parser.add_argument("--routecheck-preflight-report", type=Path, default=None)
    parser.add_argument("--blocked-route-replan-preflight-report", type=Path, default=None)
    parser.add_argument("--navigation-replay-deviation-report", type=Path, default=None)
    parser.add_argument(
        "--navigation-replay-deviation-topic-jsonl",
        type=Path,
        default=None,
        help=(
            "Build navigation_replay_deviation evidence directly from recorded "
            "topic JSONL before summarizing."
        ),
    )
    parser.add_argument("--gazebo-runtime-report", type=Path, default=None)
    parser.add_argument("--cmu-unity-sim-report", type=Path, default=None)
    parser.add_argument("--cmu-unity-runtime-report", type=Path, default=None)
    parser.add_argument("--cmu-unity-pct-strict-report", type=Path, default=None)
    parser.add_argument("--saved-map-relocalize-report", type=Path, default=None)
    parser.add_argument("--pct-saved-map-navigation-report", type=Path, default=None)
    parser.add_argument(
        "--required",
        default=None,
        help="Comma-separated gate names required for ok=true",
    )
    parser.add_argument(
        "--preset",
        choices=sorted(ALGORITHM_PRESETS),
        default=None,
        help=(
            "Named required-gate preset; inspection_mvp is product-facing "
            "patrol readiness, dimos_benchmark is the strict reference suite, "
            "and g4_server_full_sim is the server-side full simulation matrix."
        ),
    )
    parser.add_argument(
        "--max-report-age-s",
        type=float,
        default=None,
        help="Optional maximum age in seconds for accepted report artifacts.",
    )
    parser.add_argument(
        "--run-missing",
        action="store_true",
        help="Run missing or stale required gates in preset order, then summarize again.",
    )
    parser.add_argument(
        "--skip-host-blocked",
        action="store_true",
        help=(
            "With --run-missing, do not launch gates blocked by read-only host "
            "preflight; record them as host_blocked gate_runs instead."
        ),
    )
    parser.add_argument(
        "--host-preflight",
        action="store_true",
        help=(
            "Check whether this host can safely run the selected required gates. "
            "This is read-only and does not launch gate commands."
        ),
    )
    parser.add_argument(
        "--gate-timeout-s",
        type=float,
        default=None,
        help="Optional per-gate timeout used with --run-missing.",
    )
    parser.add_argument(
        "--stop-on-run-failure",
        action="store_true",
        help="Stop --run-missing after the first gate command that exits nonzero or times out.",
    )
    parser.add_argument(
        "--required-only",
        action="store_true",
        help="Evaluate only gates listed by --required; useful for setup-safe subset summaries.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=ROOT / "artifacts/server_sim_closure_summary.json",
        help="Write the JSON report to this path; use '-' for stdout only.",
    )
    parser.add_argument("--strict", action="store_true")
    return parser


def _required_from_args(args: argparse.Namespace) -> set[str]:
    if args.preset:
        return set(ALGORITHM_PRESETS[args.preset])
    if args.required:
        return {item.strip() for item in args.required.split(",") if item.strip()}
    return {spec.name for spec in GATES}


def main() -> int:
    args = _build_parser().parse_args()
    overrides = {
        "gateway_runtime_acceptance": args.gateway_runtime_acceptance_report,
        "multifloor_exploration": args.multifloor_exploration_report,
        "large_terrain": args.large_terrain_report,
        "native_pct_mujoco": args.native_pct_mujoco_report,
        "dynamic_obstacle_local_planner": args.dynamic_obstacle_local_planner_report,
        "fastlio2_live": args.fastlio2_live_report,
        "fastlio2_dynamic_inspection": args.fastlio2_dynamic_inspection_report,
        "moving_obstacle_sweep": args.moving_obstacle_sweep_report,
        "large_loop_closure": args.large_loop_closure_report,
        "mujoco_tare_exploration": args.mujoco_tare_exploration_report,
        "policy_nav": args.policy_nav_report,
        "gateway_dry_run": args.gateway_dry_run_report,
        "routecheck_preflight": args.routecheck_preflight_report,
        "blocked_route_replan_preflight": args.blocked_route_replan_preflight_report,
        "navigation_replay_deviation": args.navigation_replay_deviation_report,
        "gazebo_runtime": args.gazebo_runtime_report,
        "cmu_unity_sim": args.cmu_unity_sim_report,
        "cmu_unity_runtime": args.cmu_unity_runtime_report,
        "cmu_unity_pct_strict": args.cmu_unity_pct_strict_report,
        "saved_map_relocalize": args.saved_map_relocalize_report,
        "pct_saved_map_navigation": args.pct_saved_map_navigation_report,
    }
    required = _required_from_args(args)
    valid_names = {spec.name for spec in GATES}
    unknown = sorted(required - valid_names)
    if unknown:
        raise SystemExit(f"unknown required gate(s): {', '.join(unknown)}")
    if args.host_preflight and args.run_missing:
        raise SystemExit("--host-preflight cannot be combined with --run-missing")
    if args.skip_host_blocked and not args.run_missing:
        raise SystemExit("--skip-host-blocked requires --run-missing")
    if args.navigation_replay_deviation_report and args.navigation_replay_deviation_topic_jsonl:
        raise SystemExit(
            "--navigation-replay-deviation-report cannot be combined with "
            "--navigation-replay-deviation-topic-jsonl"
        )
    if args.moving_obstacle_sweep_report and args.moving_obstacle_sweep_report_glob:
        raise SystemExit(
            "--moving-obstacle-sweep-report cannot be combined with "
            "--moving-obstacle-sweep-report-glob"
        )
    if args.host_preflight and args.navigation_replay_deviation_topic_jsonl:
        raise SystemExit(
            "--navigation-replay-deviation-topic-jsonl cannot be combined with "
            "--host-preflight"
        )
    if args.host_preflight and args.moving_obstacle_sweep_report_glob:
        raise SystemExit(
            "--moving-obstacle-sweep-report-glob cannot be combined with "
            "--host-preflight"
        )

    materialized_inputs: list[dict[str, Any]] = []
    if args.navigation_replay_deviation_topic_jsonl is not None:
        report_path, materialized = materialize_navigation_replay_deviation_topic_jsonl(
            args.navigation_replay_deviation_topic_jsonl
        )
        overrides["navigation_replay_deviation"] = report_path
        materialized_inputs.append(materialized)
    if args.moving_obstacle_sweep_report_glob:
        report_path, materialized = materialize_moving_obstacle_sweep_report_only(
            report_globs=list(args.moving_obstacle_sweep_report_glob),
            allow_missing_video_file=bool(
                args.moving_obstacle_sweep_report_only_allow_missing_video_file
            ),
        )
        overrides["moving_obstacle_sweep"] = report_path
        materialized_inputs.append(materialized)

    summary_kwargs = {
        "report_overrides": {key: value for key, value in overrides.items() if value is not None},
        "required": required,
        "max_report_age_s": args.max_report_age_s,
        "include_optional": not args.required_only,
    }
    if args.host_preflight:
        summary = host_preflight(required=required)
    elif args.run_missing:
        summary = run_missing_required_gates(
            **summary_kwargs,
            gate_timeout_s=args.gate_timeout_s,
            stop_on_run_failure=args.stop_on_run_failure,
            skip_host_blocked=args.skip_host_blocked,
        )
    else:
        summary = summarize(**summary_kwargs)
    if materialized_inputs:
        summary["materialized_inputs"] = materialized_inputs
    text = json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True)
    print(text)
    if args.json_out and str(args.json_out) != "-":
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    return 0 if summary.get("ok") or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())

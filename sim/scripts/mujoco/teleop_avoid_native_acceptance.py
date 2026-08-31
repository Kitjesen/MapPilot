#!/usr/bin/env python3
"""Run and evaluate the native ``teleop_avoid`` MuJoCo product chain."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

EXTERNAL_ARM_SCHEMA = "lingtu.mujoco.external_arm.v1"
EXTERNAL_ARM_STATUS_SCHEMA = "lingtu.mujoco.external_arm_status.v1"
_REDACTED_COMMAND_OPTIONS = frozenset({"--external-arm-token"})
_WINDOWS_PROCESS_CONTROL = os.name == "nt"


SCHEMA_VERSION = "lingtu.mujoco.teleop_avoid_native_acceptance.v1"
ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


DEFAULT_MANIFEST = ROOT / "config" / "runtime_graph" / "acceptance" / "mujoco_teleop_avoid_native_acceptance.json"
DEFAULT_COMMAND_DURATION_S = 25.0
DEFAULT_SCENARIOS = (
    "free",
    "obstacle_detour_left",
    "obstacle_detour_right",
    "obstacle_slow",
    "obstacle_stop",
    "terrain_soft",
    "terrain_hard",
    "traversability_dropout_recovery",
    "moving_person_clear",
    "slam_inputs_dropout_recovery",
)
OPTIONAL_SCENARIOS = (
    "terrain_soft_injected",
    "terrain_hard_injected",
)
TERRAIN_PRODUCER_CONTRACT: dict[str, Any] = {
    "source": "height_plus_slope_step_roughness",
    "robot_radius_m": 0.45,
    "soft_height_m": 0.08,
    "hard_height_m": 0.20,
    "soft_slope_deg": 12.0,
    "hard_slope_deg": 28.0,
    "soft_cost": 40.0,
    "hard_cost": 100.0,
    "terrain_cache_max_points": 20000,
}
TERRAIN_SCENE_CONTRACT: dict[str, dict[str, Any]] = {
    "terrain_soft": {
        "geometry": "continuous_intersecting_plane",
        "nominal_slope_deg": 12.5,
        "slope_transition_x_m": 3.80,
        "forward_probe_component": "surface_risk_cost",
        "forward_probe_x_min_m": 3.35,
        "forward_probe_x_max_m": 6.50,
        "forward_probe_abs_y_max_m": 0.25,
        "forward_probe_max_occupancy_cost_exclusive": 40.0,
        "forward_probe_min_cost": 40.0,
        "forward_probe_max_cost_exclusive": 80.0,
    },
    "terrain_hard": {
        "geometry": "continuous_intersecting_plane",
        "nominal_slope_deg": 28.0,
        "slope_transition_x_m": 3.80,
        "forward_probe_component": "surface_risk_cost",
        "forward_probe_x_min_m": 3.35,
        "forward_probe_x_max_m": 6.50,
        "forward_probe_abs_y_max_m": 0.25,
        "forward_probe_max_occupancy_cost_exclusive": 40.0,
        "forward_probe_min_cost": 80.0,
        "forward_probe_max_cost_exclusive": None,
    },
}
FIELD_TELEOP_AVOID_PROFILE: dict[str, float | int] = {
    "traversability_publish_hz": 10,
    "traversability_slow_hz": 5,
    "traversability_tick_hz": 50,
    "traversability_cloud_pose_max_gap_s": 0.10,
    "observed_free_ttl_s": 0.60,
    "observed_free_inflation_radius_m": 0.75,
    "traversability_max_age_s": 1.5,
    "odom_max_age_s": 0.25,
    "tf_max_age_s": 0.25,
    "cloud_max_age_s": 0.35,
    "cloud_pose_max_gap_s": 0.10,
    "localization_health_max_age_s": 0.5,
    "teleop_cmd_max_age_s": 0.35,
    "input_recovery_frames": 3,
    "stop_confirmation_timeout_s": 4.0,
    "status_period_s": 0.2,
    "sensor_offset_x_m": -0.011,
    "sensor_offset_y_m": -0.02329,
    "sensor_offset_z_m": 0.04412,
}
# STOP is acknowledged only after navd's parking confirmation gate finishes.
# Keep discovery/transport headroom outside that safety window instead of
# letting a short-lived CLI time out before navd can report the true result.
NATIVE_STOP_CLIENT_TIMEOUT_MS = int(float(FIELD_TELEOP_AVOID_PROFILE["stop_confirmation_timeout_s"]) * 1000) + 3000
NATIVE_STOP_PROCESS_TIMEOUT_S = NATIVE_STOP_CLIENT_TIMEOUT_MS / 1000.0 + 2.0
NATIVE_STOP_MARGIN_SIM_S = float(FIELD_TELEOP_AVOID_PROFILE["stop_confirmation_timeout_s"]) + 4.0
MAPD_STATUS_SCHEMA = "lingtu.maps.runtime.v1"
MAPD_STATUS_MAX_AGE_S = 2.0
MAPD_RUNTIME_PROFILE: dict[str, float | int] = {
    "state_hz": 2,
    "cloud_hz": 10,
    "map_hz": 2,
    "scene_hz": 2,
    "max_points": 300000,
    "max_cloud_bytes": 16777216,
    "max_fields": 16,
    "max_point_step": 64,
    "max_string_bytes": 4096,
    "max_scene_bytes": 33554432,
    "max_voxel_snapshot_points": 200000,
    "voxel_snapshot_radius_m": 30,
    "max_voxels": 500000,
    "max_accumulated_cells": 2000000,
    "max_accumulated_blocks": 4096,
    "carve_min_z_m": -0.7,
    "carve_max_z_m": 1.8,
    "min_range_m": 0.25,
    "max_range_m": 30.0,
    "decay_ms": 250,
    "stale_ms": 1000,
}
MAPD_DATA_CONTRACT: dict[str, str] = {
    "input": "/slam/map_observation",
    "scene": "/maps/scene",
    "navigation_traversability": "/nav/traversability",
    "navigation_traversability_role": "standalone_safety_authority",
}
DYNAMIC_PERSON_CONTRACT: dict[str, Any] = {
    "body_name": "acceptance_moving_person",
    "start_xyz": [1.60, -1.20, 0.0],
    "end_xyz": [1.60, 1.20, 0.0],
    "motion_start_s": 1.0,
    "motion_duration_s": 3.0,
    "roi_center": [1.60, 0.0, 0.90],
    "roi_half_extent": [0.35, 0.35, 0.90],
    "scene_poll_hz": 8.0,
    "minimum_peak_excess_points": 3,
    "clear_grace_s": 1.75,
    "maximum_residual_points": 2,
    "maximum_residual_fraction": 0.25,
}
DYNAMIC_PERSON_CONTROL_REASONS = frozenset(
    {
        "obstacle_slow",
        "obstacle_terrain_slow",
        "obstacle_stop",
        "terrain_stop",
    }
)
DYNAMIC_PERSON_CONTROL_MIN_REDUCTION_FRACTION = 0.10
DYNAMIC_PERSON_CONTROL_MAX_STATUS_GAP_S = float(FIELD_TELEOP_AVOID_PROFILE["status_period_s"]) * 3.0
SIMULATION_POSTURE_GATE: dict[str, float | int] = {
    # ThunderV4 stands near z=0.48 m. This leaves margin for gait compression
    # while rejecting a body resting on the floor.
    "min_base_z_m": 0.30,
    "max_abs_roll_rad": math.radians(45.0),
    "max_abs_pitch_rad": math.radians(45.0),
    # Motion evidence is sampled near 20 Hz; three samples reject a transient
    # contact impulse while detecting a sustained fall in about 150 ms.
    "min_consecutive_invalid_samples": 3,
}
PRODUCT_POSTURE_REQUIRED_CASES = frozenset(
    {
        "obstacle_detour_left",
        "obstacle_detour_right",
        "obstacle_slow",
        "obstacle_stop",
        "terrain_soft",
        "terrain_hard",
    }
)
DETOUR_CASES = frozenset({"obstacle_detour_left", "obstacle_detour_right"})
BASE_FRAME_COMMANDS: dict[str, tuple[float, float, float]] = {
    "forward": (1.0, 0.0, 0.0),
    "back": (-1.0, 0.0, 0.0),
    "left": (0.0, 1.0, 0.0),
    "right": (0.0, -1.0, 0.0),
    "turn-left": (0.0, 0.0, 1.0),
    "turn-right": (0.0, 0.0, -1.0),
}
BASE_FRAME_YAWS_DEG = (0, 90, 180, -90)
_CARDINAL_WORLD_COMMANDS: dict[int, dict[str, tuple[float, float]]] = {
    0: {"forward": (1.0, 0.0), "back": (-1.0, 0.0), "left": (0.0, 1.0), "right": (0.0, -1.0)},
    90: {"forward": (0.0, 1.0), "back": (0.0, -1.0), "left": (-1.0, 0.0), "right": (1.0, 0.0)},
    180: {"forward": (-1.0, 0.0), "back": (1.0, 0.0), "left": (0.0, -1.0), "right": (0.0, 1.0)},
    -90: {"forward": (0.0, -1.0), "back": (0.0, 1.0), "left": (1.0, 0.0), "right": (-1.0, 0.0)},
}
OBSTACLE_DISTANCE_BANDS: dict[str, dict[str, float | bool]] = {
    "obstacle_slow": {
        "minimum_m": 0.55,
        "minimum_inclusive": False,
        "maximum_m": 1.20,
        "maximum_inclusive": True,
    },
    "obstacle_stop": {
        "minimum_m": 0.0,
        "minimum_inclusive": True,
        "maximum_m": 0.55,
        "maximum_inclusive": True,
    },
}
OBSTACLE_LIDAR_EVIDENCE_GATE: dict[str, float | int] = {
    "minimum_observations": 3,
    "minimum_in_band_ratio": 0.80,
    "geometry_tolerance_m": 0.06,
    "minimum_point_height_m": 0.10,
}
POST_STOP_STATIONARY_GATE: dict[str, float | int] = {
    "minimum_samples": 3,
    "minimum_span_s": 0.20,
    "maximum_excursion_m": 0.03,
}
TRAVERSABILITY_MAX_AGE_S = float(FIELD_TELEOP_AVOID_PROFILE["traversability_max_age_s"])
MIN_PRODUCT_SENSOR_RATE_RATIO = 0.90


_SCENE_GEOMS: dict[str, dict[str, str]] = {
    "obstacle_detour_left": {
        "name": "acceptance_obstacle_detour_left",
        "type": "box",
        "pos": "2.00 -0.55 0.45",
        "size": "0.30 0.85 0.45",
        "contype": "1",
        "conaffinity": "15",
        "group": "0",
        "rgba": "0.90 0.35 0.08 1",
    },
    "obstacle_detour_right": {
        "name": "acceptance_obstacle_detour_right",
        "type": "box",
        "pos": "2.00 0.55 0.45",
        "size": "0.30 0.85 0.45",
        "contype": "1",
        "conaffinity": "15",
        "group": "0",
        "rgba": "0.90 0.35 0.08 1",
    },
    "obstacle_slow": {
        "name": "acceptance_obstacle_slow",
        "type": "box",
        # Keep the obstacle inside the swept vehicle envelope while leaving
        # the centre ray corridor visible to the observed-free backstop.
        # This isolates live-obstacle slowdown from unknown terrain shadow.
        "pos": "0.94 0.35 0.40",
        "size": "0.04 0.08 0.40",
        "contype": "0",
        "conaffinity": "0",
        "group": "0",
        "rgba": "0.95 0.25 0.10 1",
    },
    "obstacle_stop": {
        "name": "acceptance_obstacle_stop",
        "type": "box",
        "pos": "0.49 0 0.40",
        "size": "0.04 0.25 0.40",
        "contype": "0",
        "conaffinity": "0",
        "group": "0",
        "rgba": "0.95 0.10 0.10 1",
    },
    "obstacle_stop_demo": {
        # Presentation-only approach lane. Keep at least 1.10 m beyond the
        # conservative 0.50 m ThunderV4 forward envelope at the start pose.
        "name": "acceptance_obstacle_stop_demo",
        "type": "box",
        "pos": "1.70 0 0.40",
        "size": "0.04 0.25 0.40",
        "contype": "0",
        "conaffinity": "0",
        "group": "0",
        "rgba": "0.95 0.10 0.10 1",
    },
    "terrain_soft": {
        "name": "acceptance_terrain_soft",
        # The plane intersects the floor at x=3.80 m and rises continuously
        # in the travel direction. It avoids a terminal step being classified
        # as a hard terrain feature instead of the intended slope.
        "type": "plane",
        "pos": "3.80 0 0",
        "size": "10 10 0.10",
        "euler": "0 -12.5 0",
        "contype": "1",
        "conaffinity": "15",
        "group": "0",
        "rgba": "0.80 0.55 0.10 1",
    },
    "terrain_hard": {
        "name": "acceptance_terrain_hard",
        "type": "plane",
        "pos": "3.80 0 0",
        "size": "10 10 0.10",
        "euler": "0 -28 0",
        # This uses the same continuous construction as terrain_soft so the
        # classification is caused by the slope, not a leading or trailing lip.
        "contype": "1",
        "conaffinity": "15",
        "group": "0",
        "rgba": "0.75 0.10 0.75 1",
    },
}

_OBSERVED_FREE_BACKSTOP = {
    "name": "acceptance_observed_free_backstop",
    "type": "box",
    "pos": "3.95 0 0.60",
    "size": "0.04 0.84 0.60",
    "contype": "0",
    "conaffinity": "0",
    "group": "0",
    "rgba": "0.35 0.35 0.35 1",
}


def _redact_command_args(command: Sequence[str]) -> list[str]:
    """Hide the external-arm token from diagnostic command output."""

    redacted = list(command)
    for index, value in enumerate(redacted[:-1]):
        if value == "--external-arm-token":
            redacted[index + 1] = "<redacted>"
    return redacted


def _with_native_env(
    command: Sequence[str],
    **values: str,
) -> tuple[list[str], Mapping[str, str]]:
    """Inject process-local environment without changing the parent Host."""

    assignments = {name: str(value) for name, value in values.items()}
    if os.name == "nt" and len(command) >= 3 and command[1] == "-e":
        env_args = [f"{name}={value}" for name, value in assignments.items()]
        return [*command[:2], "env", *env_args, *command[2:]], {}
    if os.name != "nt":
        env_args = [f"{name}={value}" for name, value in assignments.items()]
        return ["env", *env_args, *command], {}
    return list(command), assignments


_DRIVER_BRIDGE_DEFAULTS: dict[str, float | int] = {
    "max_linear_mps": 1.0,
    "max_angular_rps": 1.0,
    "command_timeout_ms": 200,
    "heartbeat_timeout_ms": 500,
    "apply_timeout_ms": 500,
}
DRIVER_SAMPLE_SEMANTICS = "successful_mujoco_step_then_LT_DRIVER_APPLIED_V2"


def _validated_driver_bridge_clock_platform(
    binaries: Mapping[str, Path],
) -> str:
    from sim.scripts.mujoco import native_navigation_acceptance as native

    return native._validated_native_clock_platform(
        Path(binaries["navigation"]),
        Path(binaries["driver_bridge"]),
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--run-plan", type=Path)
    parser.add_argument(
        "--state-provider",
        choices=("fastlio2", "mujoco_navigation_fixture"),
        default=None,
        help="Override the manifest state provider for isolated local-navigation acceptance.",
    )
    parser.add_argument(
        "--scenario",
        action="append",
        choices=("all", *DEFAULT_SCENARIOS, *OPTIONAL_SCENARIOS),
        default=None,
        help="Repeat for multiple scenarios; all runs the producer-E2E default matrix.",
    )
    parser.add_argument(
        "--artifact-dir",
        type=Path,
        default=ROOT / "artifacts" / "mujoco_teleop_avoid_native_acceptance",
    )
    parser.add_argument("--json-out", type=Path, default=None)
    parser.add_argument("--domain-base", type=int, default=None)
    parser.add_argument(
        "--duration-s",
        type=float,
        default=None,
        help="Override teleop_command.duration_s from the acceptance manifest.",
    )
    parser.add_argument("--warmup-s", type=float, default=15.0)
    parser.add_argument("--startup-timeout-s", type=float, default=60.0)
    parser.add_argument("--driving-start-timeout-s", type=float, default=45.0)
    parser.add_argument("--command-vx", type=float, default=0.5)
    parser.add_argument("--command-vy", type=float, default=0.0)
    parser.add_argument("--command-wz", type=float, default=0.0)
    parser.add_argument("--start-yaw-deg", type=float, default=None)
    parser.add_argument(
        "--frame-matrix",
        action="store_true",
        help=(
            "Run the 24 physical body-command/start-yaw cases. "
            "A verified Product acceptance runs this matrix automatically."
        ),
    )
    parser.add_argument(
        "--odom-prior-diagnostic",
        action="store_true",
        help=("Simulation-only downstream isolation using MuJoCo pose prior; never eligible for the product gate."),
    )
    parser.add_argument(
        "--realtime-factor",
        type=float,
        default=None,
        help="Diagnostic MuJoCo hardware-clock factor; default uses the product manifest.",
    )
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--strict", action="store_true")
    return parser


def _binary_source_provenance(
    binaries: Mapping[str, Path],
) -> tuple[dict[str, dict[str, Any]], list[str]]:
    """Record binary identity and reject core artifacts older than their sources."""

    nav_cpp = ROOT / "src" / "nav" / "cpp"
    endpoint_cpp = nav_cpp / "endpoint"
    message_cpp = ROOT / "src" / "message" / "cpp"
    snapshot_file = ROOT / "src" / "native" / "snapshot_file.hpp"
    maps_core_sources = [
        ROOT / "src" / "maps" / "CMakeLists.txt",
        ROOT / "src" / "maps" / "cpp",
        ROOT / "src" / "maps" / "include" / "lingtu" / "maps",
    ]
    inspection_core_sources = [
        ROOT / "src" / "nav" / "inspection" / "CMakeLists.txt",
        ROOT / "src" / "nav" / "inspection" / "inspection.cpp",
        ROOT / "src" / "nav" / "inspection" / "inspection.hpp",
        ROOT / "src" / "nav" / "inspection" / "store.cpp",
        ROOT / "src" / "nav" / "inspection" / "store.hpp",
    ]
    vendored_small_gicp = ROOT / "third_party" / "research_localization" / "small_gicp" / "include" / "small_gicp"
    optional_small_gicp_sources = [vendored_small_gicp] if vendored_small_gicp.is_dir() else []
    common_sources = [
        ROOT / "src" / "message" / "idl" / "messages.idl",
        message_cpp / "CMakeLists.txt",
        message_cpp / "topics.hpp",
        message_cpp / "qos.hpp",
    ]
    adapter_dds = ROOT / "sim" / "adapters" / "dds"
    driver_native = ROOT / "src" / "drivers" / "real" / "motion"
    source_exclusions = {
        "navigation": (
            endpoint_cpp / "tools" / "navctl.cpp",
            endpoint_cpp / "tools" / "mock.cpp",
            endpoint_cpp / "traversability",
            endpoint_cpp / "explore",
        ),
    }
    source_specs = {
        "navigation": [
            endpoint_cpp,
            nav_cpp / "planning",
            nav_cpp / "execution",
            nav_cpp / "navigation",
            nav_cpp / "platform",
            nav_cpp / "tracking",
            nav_cpp / "trajectory",
            nav_cpp / "include",
            nav_cpp / "CMakeLists.txt",
            nav_cpp / "cmake" / "NavCoreTargets.cmake",
            endpoint_cpp / "CMakeLists.txt",
            nav_cpp / "planning" / "global" / "octoplanner" / "CMakeLists.txt",
            ROOT / "src" / "explore" / "cpp" / "explore_contract.hpp",
            message_cpp / "inspection_command.hpp",
            message_cpp / "navigation_command.hpp",
            message_cpp / "operator_motion.hpp",
            snapshot_file,
            *inspection_core_sources,
            *maps_core_sources,
            *common_sources,
        ],
        "navigation_control": [
            nav_cpp / "client",
            endpoint_cpp / "tools" / "navctl.cpp",
            nav_cpp / "CMakeLists.txt",
            endpoint_cpp / "CMakeLists.txt",
            message_cpp / "exploration_command.hpp",
            message_cpp / "inspection_command.hpp",
            message_cpp / "navigation_command.hpp",
            message_cpp / "operator_motion.hpp",
            *common_sources,
        ],
        "explore": [
            endpoint_cpp / "explore",
            ROOT / "src" / "explore" / "cpp",
            nav_cpp / "CMakeLists.txt",
            endpoint_cpp / "CMakeLists.txt",
            message_cpp / "exploration_command.hpp",
            *common_sources,
        ],
        "traversability": [
            endpoint_cpp / "traversability",
            endpoint_cpp / "nav" / "dds" / "frame.hpp",
            endpoint_cpp / "nav" / "dds" / "drain.hpp",
            endpoint_cpp / "nav" / "input" / "gate.hpp",
            nav_cpp / "include" / "nav_kernel" / "dynamic_clear_core.hpp",
            nav_cpp / "include" / "nav_kernel" / "terrain_core.hpp",
            nav_cpp / "include" / "nav_kernel" / "types.hpp",
            nav_cpp / "CMakeLists.txt",
            nav_cpp / "cmake" / "NavCoreTargets.cmake",
            endpoint_cpp / "CMakeLists.txt",
            snapshot_file,
            *maps_core_sources,
            *common_sources,
        ],
        "slam": [
            ROOT / "src" / "localization" / "slam" / "cpp",
            ROOT / "src" / "localization" / "slam" / "cpp" / "CMakeLists.txt",
            ROOT / "src" / "localization" / "fastlio2" / "src",
            ROOT / "src" / "localization" / "localizer" / "src" / "localizers",
            ROOT / "src" / "maps" / "cpp" / "semantic_map_persistence.cpp",
            ROOT / "src" / "maps" / "include" / "lingtu" / "maps" / "semantic_map_persistence.hpp",
            snapshot_file,
            *optional_small_gicp_sources,
            *common_sources,
        ],
        "sensor_publisher": [
            ROOT / "src" / "drivers" / "real" / "lidar" / "sdk2_stream",
            ROOT / "src" / "drivers" / "real" / "lidar" / "sdk2_stream" / "CMakeLists.txt",
            ROOT / "src" / "drivers" / "real" / "lidar" / "native",
            *common_sources,
        ],
        "sensor_publisher_windows": [
            adapter_dds / "sensor_publisher.cpp",
            adapter_dds / "CMakeLists.txt",
            *common_sources,
        ],
        "driver_bridge": [
            adapter_dds / "mujoco_driver_bridge.cpp",
            adapter_dds / "mujoco_driver_bridge_core.cpp",
            adapter_dds / "mujoco_driver_bridge_core.hpp",
            adapter_dds / "mujoco_driver_bridge_protocol.cpp",
            adapter_dds / "mujoco_driver_bridge_protocol.hpp",
            adapter_dds / "CMakeLists.txt",
            driver_native / "core.cpp",
            driver_native / "core.hpp",
            driver_native / "cmd_vel_writer_gate.hpp",
            driver_native / "command_freshness_gate.hpp",
            driver_native / "output_ack.hpp",
            *common_sources,
        ],
        "mapd": [
            *maps_core_sources,
            *common_sources,
        ],
    }
    runtime_dependency_source_specs = {
        "navigation_control": {
            "lingtu_nav_client": [
                nav_cpp / "client",
                nav_cpp / "CMakeLists.txt",
                endpoint_cpp / "CMakeLists.txt",
                message_cpp / "exploration_command.hpp",
                message_cpp / "inspection_command.hpp",
                message_cpp / "navigation_command.hpp",
                message_cpp / "operator_motion.hpp",
                *common_sources,
            ],
        },
    }
    provenance: dict[str, dict[str, Any]] = {}
    blockers: list[str] = []
    for name, binary_source in binaries.items():
        binary = Path(binary_source)
        if not binary.is_file():
            continue
        stat = binary.stat()
        item: dict[str, Any] = {
            "path": str(binary),
            "size_bytes": int(stat.st_size),
            "mtime_ns": int(stat.st_mtime_ns),
        }
        if name == "navigation_control":
            client_library = binary.with_name(
                "lingtu_nav_client.dll" if binary.suffix.lower() == ".exe" else "liblingtu_nav_client.so"
            )
            if client_library.is_file():
                library_stat = client_library.stat()
                item["runtime_dependencies"] = {
                    "lingtu_nav_client": {
                        "path": str(client_library),
                        "size_bytes": int(library_stat.st_size),
                        "mtime_ns": int(library_stat.st_mtime_ns),
                    }
                }
            else:
                blockers.append("native_runtime_dependency_missing:navigation_control")
        source_key = (
            "sensor_publisher_windows" if name == "sensor_publisher" and binary.suffix.lower() == ".exe" else name
        )
        specs = source_specs.get(source_key) or []
        if specs:
            item["source_specs"] = [str(source) for source in specs]
            missing_specs = [str(source) for source in specs if not source.is_file() and not source.is_dir()]
            if missing_specs:
                item["missing_source_specs"] = missing_specs
                blockers.extend(f"native_source_spec_missing:{name}:{source}" for source in missing_specs)
                provenance[name] = item
                continue
            source_files: list[Path] = []
            excluded_sources = tuple(source_exclusions.get(name) or ())
            if excluded_sources:
                item["excluded_source_specs"] = [str(source) for source in excluded_sources]
            for source in specs:
                if source.is_file():
                    if source not in excluded_sources:
                        source_files.append(source)
                    continue
                source_files.extend(
                    path
                    for suffix in (
                        "*.c",
                        "*.cc",
                        "*.cxx",
                        "*.cpp",
                        "*.h",
                        "*.hh",
                        "*.hpp",
                        "*.idl",
                    )
                    for path in source.rglob(suffix)
                    if path.is_file()
                    and not path.name.startswith("test_")
                    and "tests" not in path.parts
                    and not any(path == excluded or excluded in path.parents for excluded in excluded_sources)
                )
            latest_source_mtime_ns = max(
                (path.stat().st_mtime_ns for path in source_files),
                default=0,
            )
            item["source_latest_mtime_ns"] = int(latest_source_mtime_ns)
            item["newer_than_sources"] = stat.st_mtime_ns >= latest_source_mtime_ns
            if not item["newer_than_sources"]:
                blockers.append(f"native_binary_stale:{name}")
            if name == "navigation_control" and item.get("runtime_dependencies"):
                dependency = item["runtime_dependencies"]["lingtu_nav_client"]
                dependency_specs = runtime_dependency_source_specs[name]["lingtu_nav_client"]
                dependency["source_specs"] = [str(source) for source in dependency_specs]
                missing_dependency_specs = [
                    str(source) for source in dependency_specs if not source.is_file() and not source.is_dir()
                ]
                if missing_dependency_specs:
                    dependency["missing_source_specs"] = missing_dependency_specs
                    blockers.extend(
                        f"native_runtime_dependency_source_spec_missing:navigation_control:{source}"
                        for source in missing_dependency_specs
                    )
                else:
                    dependency_source_files: list[Path] = []
                    for source in dependency_specs:
                        if source.is_file():
                            dependency_source_files.append(source)
                            continue
                        dependency_source_files.extend(
                            path
                            for suffix in (
                                "*.c",
                                "*.cc",
                                "*.cxx",
                                "*.cpp",
                                "*.h",
                                "*.hh",
                                "*.hpp",
                                "*.idl",
                            )
                            for path in source.rglob(suffix)
                            if path.is_file() and not path.name.startswith("test_") and "tests" not in path.parts
                        )
                    dependency_latest_mtime_ns = max(
                        (path.stat().st_mtime_ns for path in dependency_source_files),
                        default=0,
                    )
                    dependency["source_latest_mtime_ns"] = int(dependency_latest_mtime_ns)
                    dependency["newer_than_sources"] = int(dependency["mtime_ns"]) >= dependency_latest_mtime_ns
                if dependency.get("newer_than_sources") is False:
                    blockers.append("native_runtime_dependency_stale:navigation_control")
        provenance[name] = item
    return provenance, blockers


def _teleop_product_contract_evidence(
    manifest: Mapping[str, Any],
) -> dict[str, Any]:
    """Bind this harness to the canonical teleop_avoid Product declaration."""

    from lingtu.products import product_lifecycle

    lifecycle = product_lifecycle("teleop_avoid")
    canonical = {
        "product": lifecycle.product,
        "source": "config/runtime_graph/products/teleop_avoid.yaml",
        "native_control_mode": lifecycle.native_control_mode,
        "slam_mode": lifecycle.slam_mode,
        "requires_map": lifecycle.requires_map,
    }
    raw_declared = manifest.get("product_contract")
    declared = dict(raw_declared) if isinstance(raw_declared, Mapping) else {}
    blockers: list[str] = []
    if not declared:
        blockers.append("teleop_product_contract_missing")
    for field, expected in canonical.items():
        actual = declared.get(field)
        if actual != expected:
            blockers.append(f"teleop_product_contract_mismatch:{field}:expected={expected}:actual={actual}")

    slam_runtime = manifest.get("slam_runtime")
    slam_runtime = dict(slam_runtime) if isinstance(slam_runtime, Mapping) else {}
    if str(slam_runtime.get("mode") or "").strip().lower() != "mapping":
        blockers.append("teleop_slam_runtime_mode_not_mapping")
    asset_builder = manifest.get("asset_builder")
    asset_builder = dict(asset_builder) if isinstance(asset_builder, Mapping) else {}
    if str(asset_builder.get("kind") or "") != "scene_only":
        blockers.append("teleop_asset_builder_not_scene_only")
    if str(manifest.get("extends") or ""):
        blockers.append("teleop_manifest_must_not_inherit_navigation_acceptance")
    if str(manifest.get("map_dir") or "") or manifest.get("map_files"):
        blockers.append("teleop_saved_map_contract_forbidden")
    if manifest.get("goal"):
        blockers.append("teleop_autonomous_goal_contract_forbidden")
    return {
        "ok": not blockers,
        "canonical": canonical,
        "declared": declared,
        "slam_runtime": slam_runtime,
        "asset_builder": asset_builder,
        "blockers": blockers,
    }


def _teleop_scene_evidence(scene_xml: Path) -> dict[str, Any]:
    blockers: list[str] = []
    scene_size = 0
    if not scene_xml.is_file():
        blockers.append(f"teleop_asset_scene_missing:{scene_xml}")
    else:
        try:
            scene_size = int(scene_xml.stat().st_size)
        except OSError:
            scene_size = 0
        if scene_size <= 0:
            blockers.append(f"teleop_asset_scene_empty:{scene_xml}")
        else:
            try:
                root = ET.parse(scene_xml).getroot()
            except (ET.ParseError, OSError) as exc:
                blockers.append(f"teleop_asset_scene_invalid:{scene_xml}:{type(exc).__name__}")
            else:
                if root.tag != "mujoco" or root.find("worldbody") is None:
                    blockers.append(f"teleop_asset_scene_worldbody_missing:{scene_xml}")
    return {
        "ok": not blockers,
        "scene_xml": str(scene_xml),
        "scene_size_bytes": scene_size,
        "blockers": blockers,
    }


def _prepare_teleop_scene_asset(
    manifest: dict[str, Any],
    artifact_dir: Path,
) -> dict[str, Any]:
    """Prepare only MuJoCo geometry; teleop_avoid does not own a saved map."""

    from sim.scripts.mujoco import native_navigation_acceptance as native
    from sim.scripts.mujoco import saved_map_plan_gate

    world_value = str(manifest.get("world") or "")
    if world_value:
        configured_scene = native._repo_path(world_value)
        evidence = _teleop_scene_evidence(configured_scene)
        if evidence.get("ok") is True:
            manifest["world"] = str(configured_scene)
            return {
                "attempted": False,
                "ok": True,
                "reason": "configured_scene_ready",
                **evidence,
            }
        return {
            "attempted": False,
            "ok": False,
            "reason": "configured_scene_invalid",
            **evidence,
        }

    raw_spec = manifest.get("asset_builder")
    spec = dict(raw_spec) if isinstance(raw_spec, Mapping) else {}
    if str(spec.get("kind") or "") != "scene_only":
        return {
            "attempted": False,
            "ok": False,
            "reason": "teleop_scene_builder_not_configured",
            "blockers": ["teleop_asset_builder_not_scene_only"],
        }
    scene_xml = artifact_dir / "prepared_assets" / "scene.xml"
    scene_xml.parent.mkdir(parents=True, exist_ok=True)
    try:
        saved_map_plan_gate.generate_scene_xml(
            scene_xml,
            length=float(spec.get("length") or 3.0),
            width=float(spec.get("width") or 1.8),
            scene_preset=str(spec.get("scene_preset") or "corridor"),
        )
    except (OSError, TypeError, ValueError) as exc:
        return {
            "attempted": True,
            "ok": False,
            "reason": f"teleop_scene_builder_failed:{type(exc).__name__}:{exc}",
            "scene_xml": str(scene_xml),
            "blockers": ["teleop_scene_builder_failed"],
        }
    evidence = _teleop_scene_evidence(scene_xml.resolve())
    if evidence.get("ok") is True:
        manifest["world"] = str(scene_xml.resolve())
    return {
        "attempted": True,
        "ok": evidence.get("ok") is True,
        "reason": ("teleop_scene_prepared" if evidence.get("ok") is True else "teleop_scene_invalid"),
        "builder": spec,
        **evidence,
    }


def _policy_runtime_evidence(*, required: bool) -> dict[str, Any]:
    """Check the Python modules required by the MuJoCo policy runner."""

    modules = ("mujoco", "onnxruntime")
    missing = [name for name in modules if importlib.util.find_spec(name) is None]
    blockers = [f"python_runtime_dependency_missing:{name}" for name in missing] if required else []
    return {
        "required": required,
        "python": sys.executable,
        "modules": {name: name not in missing for name in modules},
        "blockers": blockers,
        "ok": not blockers,
    }


def prepare_runtime(args: argparse.Namespace) -> dict[str, Any]:
    """Prepare saved-map-free teleop_avoid and resolve native dependencies."""

    from sim.scripts.mujoco import native_navigation_acceptance as native

    artifact_dir = Path(args.artifact_dir).expanduser().resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = Path(args.manifest).expanduser().resolve()
    manifest = native._load_manifest(manifest_path)
    if getattr(args, "state_provider", None):
        slam_runtime = dict(manifest.get("slam_runtime") or {})
        slam_runtime["provider"] = str(args.state_provider)
        manifest["slam_runtime"] = slam_runtime

    product_contract = _teleop_product_contract_evidence(manifest)
    asset_preparation = _prepare_teleop_scene_asset(manifest, artifact_dir)
    binaries, paths, blockers, provenance = native._preflight_map_free(manifest)
    policy_path = Path(paths.get("policy") or "")
    policy_runtime = _policy_runtime_evidence(required=policy_path.is_file())
    blockers.extend(policy_runtime["blockers"])
    state_provider = str(((manifest.get("slam_runtime") or {}).get("provider") or "fastlio2")).strip().lower()
    required_binaries = {
        "sensor_publisher",
        "mapd",
        "navigation",
        "navigation_control",
        "driver_bridge",
    }
    if state_provider != "mujoco_navigation_fixture":
        required_binaries.add("slam")

    binary_provenance, stale_blockers = _binary_source_provenance(binaries)
    blockers.extend(stale_blockers)
    out_of_scope: list[str] = []
    in_scope: list[str] = []
    for blocker in blockers:
        binary_name = ""
        if blocker.startswith(
            (
                "native_binary_missing:",
                "native_binary_stale:",
                "native_source_spec_missing:",
                "native_runtime_dependency_stale:",
            )
        ):
            binary_name = blocker.split(":", 2)[1]
        if binary_name and binary_name not in required_binaries:
            out_of_scope.append(blocker)
        else:
            in_scope.append(blocker)
    blockers = [
        *[str(value) for value in product_contract.get("blockers") or ()],
        *in_scope,
    ]
    for name in sorted(required_binaries):
        if name not in binaries:
            blockers.append(f"native_binary_missing:{name}")
    native_clock_platform = ""
    if {"navigation", "driver_bridge"} <= set(binaries):
        try:
            native_clock_platform = _validated_driver_bridge_clock_platform(binaries)
        except ValueError:
            blockers.append("native_driver_clock_platform_mismatch")
    if os.name != "nt":
        blockers.append("host_contract_requires_windows_wsl2")
    if "sensor_publisher_dds_unavailable" in blockers and "sensor_publisher" in binaries:
        for _ in range(2):
            publisher_ok, publisher_probe = native._probe_sensor_publisher(binaries["sensor_publisher"])
            provenance["sensor_publisher_probe_retry"] = publisher_probe
            if publisher_ok:
                blockers = [value for value in blockers if value != "sensor_publisher_dds_unavailable"]
                break
            time.sleep(0.5)
    if asset_preparation.get("ok") is not True:
        blockers.append(str(asset_preparation.get("reason") or "asset_preparation_failed"))
        blockers.extend(str(value) for value in asset_preparation.get("blockers") or ())
    blockers = list(dict.fromkeys(str(value) for value in blockers))
    return {
        "ok": not blockers,
        "blockers": blockers,
        "manifest": manifest,
        "binaries": binaries,
        "paths": paths,
        "details": {
            "host_contract": "windows_wsl2_x86_64",
            "manifest": str(manifest_path),
            "product_contract": product_contract,
            "asset_preparation": asset_preparation,
            "runtime_provenance": provenance,
            "policy_runtime": policy_runtime,
            "binary_provenance": binary_provenance,
            "native_clock_platform": native_clock_platform,
            "out_of_scope_preflight_findings": out_of_scope,
            "binaries": {name: str(path) for name, path in binaries.items()},
            "paths": {name: str(path) for name, path in paths.items()},
        },
    }


def _run_control(
    binary: Path,
    arguments: Sequence[str],
    *,
    domain_id: int,
    env: Mapping[str, str] | None = None,
    timeout_s: float = 10.0,
) -> dict[str, Any]:
    from sim.scripts.mujoco import native_navigation_acceptance as native

    command = native._native_command(
        binary,
        *(str(value) for value in arguments),
        "--domain-id",
        str(domain_id),
    )
    command_env: Mapping[str, str] = {}
    if env:
        command, command_env = _with_native_env(command, **dict(env))
    try:
        popen_options: dict[str, Any] = {}
        if command_env:
            inherited = dict(os.environ)
            inherited.update(command_env)
            popen_options["env"] = inherited
        completed = subprocess.run(
            command,
            cwd=ROOT,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=max(1.0, float(timeout_s)),
            check=False,
            **popen_options,
        )
        return {
            "command": command,
            "returncode": int(completed.returncode),
            "stdout": completed.stdout or "",
            "stderr": completed.stderr or "",
        }
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {
            "command": command,
            "returncode": None,
            "stdout": "",
            "stderr": f"{type(exc).__name__}:{exc}",
        }


_OPERATOR_MOTION_EVENT_RE = re.compile(
    r"LT_OPERATOR_MOTION_EVENT_V1 "
    r"action=(claim|sample|hold|release) accepted=(true|false) "
    r"source_id=(\S+) source_epoch=(\d+) source_sequence=(\d+) sample_count=(\d+)$"
)


def parse_operator_motion_events(output: str) -> list[dict[str, Any]]:
    """Parse only the machine-readable lifecycle emitted after typed DDS success."""

    events: list[dict[str, Any]] = []
    for line in str(output or "").splitlines():
        match = _OPERATOR_MOTION_EVENT_RE.search(line.strip())
        if match is None:
            continue
        action, accepted, source_id, source_epoch, source_sequence, sample_count = match.groups()
        events.append(
            {
                "action": action,
                "accepted": accepted == "true",
                "source_id": source_id,
                "source_epoch": int(source_epoch),
                "source_sequence": int(source_sequence),
                "sample_count": int(sample_count),
            }
        )
    return events


def _twist_is_zero(value: Any) -> bool:
    if not isinstance(value, Mapping):
        return False
    for axis in ("vx", "vy", "wz"):
        if axis not in value or isinstance(value[axis], bool):
            return False
        try:
            number = float(value[axis])
        except (TypeError, ValueError):
            return False
        if not math.isfinite(number) or abs(number) > 1e-6:
            return False
    return True


POST_STOP_REQUIRED_STATUS_SAMPLES = 3
POST_STOP_EVIDENCE_TIMEOUT_S = max(
    3.0,
    float(FIELD_TELEOP_AVOID_PROFILE["status_period_s"]) * (POST_STOP_REQUIRED_STATUS_SAMPLES + 5),
)
FREE_COMMAND_ACCEPTED_REASONS = frozenset({"accepted", "teleop_assist_control_ready"})


def _native_stop_arguments(reason: str) -> list[str]:
    return ["stop", str(reason), "--timeout-ms", str(NATIVE_STOP_CLIENT_TIMEOUT_MS)]


def _final_cmd_vel_from_nav_status(nav: Mapping[str, Any]) -> Mapping[str, Any] | None:
    operator_motion = nav.get("operator_motion")
    operator_motion = operator_motion if isinstance(operator_motion, Mapping) else {}
    status = operator_motion.get("status")
    status = status if isinstance(status, Mapping) else {}
    final_cmd = status.get("final_cmd_vel")
    if isinstance(final_cmd, Mapping):
        return final_cmd
    final_cmd = nav.get("final_cmd_vel")
    return final_cmd if isinstance(final_cmd, Mapping) else None


def _finite_number(value: Any) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(float(value))


def _nav_status_stamp_s(nav: Mapping[str, Any]) -> float | None:
    value = nav.get("stamp_s")
    if _finite_number(value):
        return float(value)
    return None


def _native_stop_accepted(stdout: str) -> bool:
    return re.search(r"(?m)^accepted stop:", str(stdout or "")) is not None


def _native_cleanup_stop_blockers(
    *,
    product_gate_eligible: bool,
    stop_result: Mapping[str, Any],
    post_stop_zero_output: Mapping[str, Any],
) -> list[str]:
    blockers: list[str] = []
    if stop_result and stop_result.get("returncode") != 0:
        blockers.append("native_cleanup_stop_failed")
    if product_gate_eligible and stop_result.get("acked") is not True:
        blockers.append("native_cleanup_stop_ack_missing")
    if product_gate_eligible and post_stop_zero_output.get("zero_output_observed") is not True:
        blockers.append("native_cleanup_post_stop_zero_unproven")
    return blockers


def _ready_path(plan: Any, process: str, root: Path) -> Path:
    selected = tuple(item for item in getattr(plan, "processes", ()) if item.name == process)
    if len(selected) != 1 or selected[0].command is None:
        raise ValueError(f"attach-only teleop_avoid requires one {process}")
    readiness = selected[0].command.readiness
    if readiness is None or readiness.kind != "file" or not readiness.target:
        raise ValueError(f"attach-only {process} requires file readiness")
    return root / readiness.target


def _status(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


def _samples(path: Path, worker: threading.Thread, timeout_s: float) -> list[dict[str, Any]]:
    deadline = time.monotonic() + max(0.1, timeout_s)
    samples: list[dict[str, Any]] = []
    last_stamp = -math.inf
    while time.monotonic() < deadline and (worker.is_alive() or len(samples) < 3):
        nav = _status(path)
        stamp = _nav_status_stamp_s(nav)
        if stamp is not None and stamp > last_stamp:
            samples.append({"wall_s": time.time(), "nav": nav})
            last_stamp = stamp
        time.sleep(0.05)
    return samples


def _fresh_zero(path: Path, after: float, timeout_s: float) -> list[dict[str, Any]]:
    deadline = time.monotonic() + max(0.1, timeout_s)
    samples: list[dict[str, Any]] = []
    last_stamp = after
    while time.monotonic() < deadline and len(samples) < POST_STOP_REQUIRED_STATUS_SAMPLES:
        nav = _status(path)
        stamp = _nav_status_stamp_s(nav)
        if stamp is not None and stamp > last_stamp:
            samples.append(nav)
            last_stamp = stamp
        time.sleep(0.05)
    return samples


def _path_lateral_offset_m(value: Any, command_heading_rad: float) -> float:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        return 0.0
    points: list[tuple[float, float]] = []
    for point in value:
        if not isinstance(point, Sequence) or isinstance(point, (str, bytes)) or len(point) < 2:
            return 0.0
        if not _finite_number(point[0]) or not _finite_number(point[1]):
            return 0.0
        points.append((float(point[0]), float(point[1])))
    if len(points) < 3:
        return 0.0
    start_x, start_y = points[0]
    c = math.cos(command_heading_rad)
    s = math.sin(command_heading_rad)
    return max(
        abs(-(x - start_x) * s + (y - start_y) * c)
        for x, y in points[1:]
    )


def _initial_command_heading_rad(plan: Any, vx: float, vy: float) -> float:
    command_angle = math.atan2(vy, vx)
    simulation = getattr(plan, "simulation", None)
    if not isinstance(simulation, Mapping):
        return command_angle
    physics = simulation.get("physics_plan")
    if not isinstance(physics, Mapping):
        return command_angle
    robots = physics.get("robots")
    if not isinstance(robots, Sequence) or isinstance(robots, (str, bytes)) or len(robots) != 1:
        return command_angle
    robot = robots[0]
    if not isinstance(robot, Mapping):
        return command_angle
    spawn = robot.get("spawn")
    if not isinstance(spawn, Mapping):
        return command_angle
    quaternion = spawn.get("quaternion_wxyz")
    if (
        not isinstance(quaternion, Sequence)
        or isinstance(quaternion, (str, bytes))
        or len(quaternion) != 4
        or not all(_finite_number(value) for value in quaternion)
    ):
        return command_angle
    w, x, y, z = (float(value) for value in quaternion)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return yaw + command_angle


def _twist_is_nonzero(value: Any) -> bool:
    if not isinstance(value, Mapping):
        return False
    values: list[float] = []
    for axis in ("vx", "vy", "wz"):
        if not _finite_number(value.get(axis)):
            return False
        values.append(float(value[axis]))
    return any(abs(number) > 1e-4 for number in values)


def _attached_metrics(
    samples: Sequence[Mapping[str, Any]],
    motion: Mapping[str, Any],
    post_stop: Sequence[Mapping[str, Any]],
    *,
    command_heading_rad: float,
) -> dict[str, Any]:
    nav = [item.get("nav") for item in samples if isinstance(item.get("nav"), Mapping)]
    ready = [
        item
        for item in nav
        if item.get("control_mode") == "teleop_avoid"
        and isinstance(item.get("input_gate"), Mapping)
        and item["input_gate"].get("ready") is True
    ]
    teleop = [item.get("teleop") for item in ready if isinstance(item.get("teleop"), Mapping)]
    local = [item.get("last_local") for item in ready if isinstance(item.get("last_local"), Mapping)]
    planner = [
        item.get("local_planner_debug")
        for item in ready
        if isinstance(item.get("local_planner_debug"), Mapping)
    ]
    events = parse_operator_motion_events(f"{motion.get('stdout') or ''}\n{motion.get('stderr') or ''}")

    def counts(values: Sequence[str]) -> dict[str, int]:
        result: dict[str, int] = {}
        for value in values:
            result[value] = result.get(value, 0) + 1
        return result

    offsets = [
        _path_lateral_offset_m(item.get("local_path"), command_heading_rad)
        for item in ready
    ]
    return {
        "ready_samples": len(ready),
        "teleop_reasons": counts([str(item.get("reason") or "") for item in teleop]),
        "local_reasons": counts([str(item.get("reason") or "") for item in local]),
        "planner_reasons": counts([str(item.get("search_reason") or "") for item in planner]),
        "continuity_reused_samples": sum(
            1 for item in planner if item.get("continuity_reused") is True
        ),
        "max_local_path_points": max(
            (max(int(item.get("local_path_points") or 0), len(item.get("local_path") or ())) for item in ready),
            default=0,
        ),
        "max_lateral_detour_m": max(offsets, default=0.0),
        "nonzero_output_samples": sum(
            1 for item in teleop if _twist_is_nonzero(item.get("output"))
        ),
        "resume_required_samples": sum(
            1
            for item in ready
            if isinstance(item.get("control_authority"), Mapping)
            and item["control_authority"].get("resume_required") is True
        ),
        "operator_actions": sorted(
            {str(item.get("action") or "") for item in events if item.get("accepted") is True}
        ),
        "post_stop_samples": len(post_stop),
        "post_stop_zero_samples": sum(
            1 for item in post_stop if _twist_is_zero(_final_cmd_vel_from_nav_status(item))
        ),
    }


def _attached_blockers(
    samples: Sequence[Mapping[str, Any]],
    motion: Mapping[str, Any],
    stop: Mapping[str, Any],
    post_stop: Sequence[Mapping[str, Any]],
    *,
    minimum_lateral_detour_m: float = 0.5,
    command_heading_rad: float,
) -> list[str]:
    metrics = _attached_metrics(
        samples,
        motion,
        post_stop,
        command_heading_rad=command_heading_rad,
    )
    actions = set(metrics["operator_actions"])
    blockers: list[str] = []
    if metrics["ready_samples"] <= 0:
        blockers.append("teleop_avoid_ready_missing")
    if metrics["max_local_path_points"] < 2:
        blockers.append("local_avoid_path_missing")
    if metrics["max_lateral_detour_m"] + 1e-9 < minimum_lateral_detour_m:
        blockers.append("local_detour_geometry_missing")
    if metrics["nonzero_output_samples"] <= 0:
        blockers.append("nonzero_control_missing")
    if metrics["resume_required_samples"] > 0:
        blockers.append("resume_required_during_detour")
    if motion.get("returncode") != 0 or not {"claim", "sample", "hold", "release"} <= actions:
        blockers.append("operator_motion_lifecycle_incomplete")
    if stop.get("returncode") != 0 or not _native_stop_accepted(str(stop.get("stdout") or "")):
        blockers.append("native_stop_ack_missing")
    if len(post_stop) < POST_STOP_REQUIRED_STATUS_SAMPLES or not all(
        _twist_is_zero(_final_cmd_vel_from_nav_status(item)) for item in post_stop
    ):
        blockers.append("post_stop_zero_barrier_failed")
    return blockers


def run_attached(
    *,
    plan: Any,
    run_plan_path: Path,
    product_session_id: str,
    prepared: Mapping[str, Any],
    args: argparse.Namespace,
) -> dict[str, Any]:
    """Exercise the active Product; never create a Product process."""

    if getattr(plan, "product", None) != "teleop_avoid":
        raise ValueError("attach-only acceptance requires teleop_avoid")
    names = {item.name for item in getattr(plan, "processes", ())}
    sensors = tuple(name for name in ("lidar_publisher", "imu_publisher", "camera_publisher") if name in names)
    if not {"lidar_publisher", "imu_publisher"} <= set(sensors) or "sensor_publisher" in names:
        raise ValueError("attach-only teleop_avoid requires split lidar and IMU processes")
    control = Path((prepared.get("binaries") or {})["navigation_control"])
    domain_id = int(args.domain_base)
    status_path = _ready_path(plan, "nav_runtime", run_plan_path.parent)
    environment = dict(getattr(plan, "native_process_environment", {}) or {})
    environment["LINGTU_PRODUCT_SESSION_ID"] = product_session_id
    manifest = prepared.get("manifest")
    manifest = manifest if isinstance(manifest, Mapping) else {}
    teleop_command = manifest.get("teleop_command")
    teleop_command = teleop_command if isinstance(teleop_command, Mapping) else {}
    configured_duration_s = teleop_command.get("duration_s", DEFAULT_COMMAND_DURATION_S)
    duration_s = float(args.duration_s if args.duration_s is not None else configured_duration_s)
    if duration_s <= 0.0:
        raise ValueError("teleop_avoid command duration must be positive")
    result: dict[str, Any] = {}

    def drive() -> None:
        result.update(
            _run_control(
                control,
                (
                    "operator-motion",
                    str(float(args.command_vx)),
                    str(float(getattr(args, "command_vy", 0.0))),
                    str(float(getattr(args, "command_wz", 0.0))),
                    "--duration-s",
                    str(duration_s),
                    "--rate-hz",
                    "10",
                    "--source-id",
                    f"mujoco-exact-teleop-avoid-{domain_id}",
                    "--lease-ttl-ms",
                    "2000",
                    "--freshness-budget-ms",
                    "350",
                    "--cleanup-settle-ms",
                    "300",
                    "--timeout-ms",
                    "3000",
                ),
                domain_id=domain_id,
                env=environment,
                timeout_s=max(12.0, duration_s + 6.0),
            )
        )

    worker = threading.Thread(target=drive, name="teleop-avoid-acceptance")
    worker.start()
    timeline = _samples(status_path, worker, duration_s + 8.0)
    worker.join(timeout=1.0)
    if worker.is_alive():
        raise RuntimeError("operator motion did not finish")
    before_stop = _nav_status_stamp_s(_status(status_path)) or time.time()
    stop = _run_control(
        control,
        _native_stop_arguments("teleop_avoid_exact_acceptance_stop"),
        domain_id=domain_id,
        env=environment,
        timeout_s=NATIVE_STOP_PROCESS_TIMEOUT_S,
    )
    post_stop = _fresh_zero(
        status_path,
        before_stop,
        POST_STOP_EVIDENCE_TIMEOUT_S,
    )
    detour_acceptance = manifest.get("detour_acceptance")
    detour_acceptance = detour_acceptance if isinstance(detour_acceptance, Mapping) else {}
    robot_geometry = manifest.get("robot_geometry")
    robot_geometry = robot_geometry if isinstance(robot_geometry, Mapping) else {}
    minimum_lateral_detour_m = float(detour_acceptance.get("minimum_lateral_detour_m") or 0.5)
    command_vx = float(args.command_vx)
    command_vy = float(getattr(args, "command_vy", 0.0))
    command_heading_rad = _initial_command_heading_rad(plan, command_vx, command_vy)
    metrics = _attached_metrics(
        timeline,
        result,
        post_stop,
        command_heading_rad=command_heading_rad,
    )
    blockers = _attached_blockers(
        timeline,
        result,
        stop,
        post_stop,
        minimum_lateral_detour_m=minimum_lateral_detour_m,
        command_heading_rad=command_heading_rad,
    )
    return {
        "ok": not blockers,
        "mode": "attach_only",
        "run_plan": str(run_plan_path.expanduser().resolve()),
        "product_session_id": product_session_id,
        "sensor_processes": list(sensors),
        "timeline_samples": len(timeline),
        "operator_motion": result,
        "stop": stop,
        "post_stop_samples": post_stop,
        "metrics": metrics,
        "command": {
            "vx": command_vx,
            "vy": command_vy,
            "wz": float(getattr(args, "command_wz", 0.0)),
            "duration_s": duration_s,
        },
        "physical_acceptance": dict(detour_acceptance),
        "robot_geometry": dict(robot_geometry),
        "blockers": blockers,
    }

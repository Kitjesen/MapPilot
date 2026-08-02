#!/usr/bin/env python3
"""Run and evaluate the native ``teleop_avoid`` MuJoCo product chain."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
import re
import secrets
import shutil
import signal
import statistics
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


def _redact_command_args(command: Sequence[str]) -> list[str]:
    """Return a report-safe copy of a subprocess command."""

    redacted = list(command)
    index = 0
    while index < len(redacted):
        if redacted[index] not in _REDACTED_COMMAND_OPTIONS:
            index += 1
            continue
        if index + 1 < len(redacted):
            redacted[index + 1] = "<redacted>"
        index += 2
    return redacted


SCHEMA_VERSION = "lingtu.mujoco.teleop_avoid_native_acceptance.v1"
ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
DEFAULT_MANIFEST = (
    ROOT
    / "config"
    / "runtime_graph"
    / "acceptance"
    / "mujoco_teleop_avoid_native_acceptance.json"
)
DEFAULT_SCENARIOS = (
    "free",
    "obstacle_slow",
    "obstacle_stop",
    "terrain_soft",
    "terrain_hard",
    "traversability_dropout_recovery",
)
OPTIONAL_SCENARIOS = (
    "moving_person_clear",
    "terrain_soft_injected",
    "terrain_hard_injected",
    "slam_inputs_dropout_recovery",
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
    "traversability_max_age_s": 1.5,
    "odom_max_age_s": 0.25,
    "tf_max_age_s": 0.25,
    "cloud_max_age_s": 0.35,
    "cloud_pose_max_gap_s": 0.10,
    "localization_health_max_age_s": 0.5,
    "input_recovery_frames": 3,
    "stop_confirmation_timeout_s": 4.0,
    "sensor_offset_x_m": -0.011,
    "sensor_offset_y_m": -0.02329,
    "sensor_offset_z_m": 0.04412,
}
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
TRAVERSABILITY_MAX_AGE_S = float(FIELD_TELEOP_AVOID_PROFILE["traversability_max_age_s"])

_SCENE_GEOMS: dict[str, dict[str, str]] = {
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


def _append_moving_person(worldbody: ET.Element) -> None:
    body = ET.SubElement(
        worldbody,
        "body",
        {
            "name": str(DYNAMIC_PERSON_CONTRACT["body_name"]),
            "pos": " ".join(
                str(value) for value in DYNAMIC_PERSON_CONTRACT["start_xyz"]
            ),
            "mocap": "true",
        },
    )
    common = {
        "contype": "0",
        "conaffinity": "0",
        "group": "0",
    }
    ET.SubElement(
        body,
        "geom",
        {
            **common,
            "name": "acceptance_moving_person_torso",
            "type": "capsule",
            "size": "0.20 0.42",
            "pos": "0 0 1.00",
            "rgba": "0.85 0.18 0.18 1",
        },
    )
    ET.SubElement(
        body,
        "geom",
        {
            **common,
            "name": "acceptance_moving_person_head",
            "type": "sphere",
            "size": "0.13",
            "pos": "0 0 1.58",
            "rgba": "0.90 0.68 0.52 1",
        },
    )
    for suffix, y in (("left", "0.11"), ("right", "-0.11")):
        ET.SubElement(
            body,
            "geom",
            {
                **common,
                "name": f"acceptance_moving_person_leg_{suffix}",
                "type": "capsule",
                "size": "0.08 0.40",
                "pos": f"0 {y} 0.42",
                "rgba": "0.18 0.22 0.62 1",
            },
        )


def build_scene_variant(base_scene: Path, output: Path, scenario: str) -> Path:
    """Copy one prepared scene and add only the requested runtime hazard."""

    tree = ET.parse(base_scene)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError(f"MuJoCo scene has no worldbody: {base_scene}")
    for geom in list(worldbody.findall("geom")):
        if str(geom.attrib.get("name") or "").startswith("acceptance_"):
            worldbody.remove(geom)
    for body in list(worldbody.findall("body")):
        if str(body.attrib.get("name") or "").startswith("acceptance_"):
            worldbody.remove(body)
    backstop = dict(_OBSERVED_FREE_BACKSTOP)
    terrain_scene = scenario in TERRAIN_SCENE_CONTRACT
    if terrain_scene:
        # The product MID-360 pattern only observes a low rise several
        # metres ahead. Extend the generated corridor so the producer can
        # observe and cache it before it enters the native teleop probe.
        for geom in worldbody.findall("geom"):
            if geom.attrib.get("type") != "box":
                continue
            if geom.attrib.get("name") in {"floor", "left_rail", "right_rail"}:
                pos = str(geom.attrib.get("pos") or "0 0 0").split()
                size = str(geom.attrib.get("size") or "0 0 0").split()
                if len(pos) == 3 and len(size) == 3:
                    pos[0] = "3.500"
                    size[0] = "4.500"
                    geom.set("pos", " ".join(pos))
                    geom.set("size", " ".join(size))
    if not terrain_scene:
        ET.SubElement(worldbody, "geom", backstop)
    spec = _SCENE_GEOMS.get(scenario)
    if spec is not None:
        ET.SubElement(worldbody, "geom", dict(spec))
    if scenario == "moving_person_clear":
        _append_moving_person(worldbody)
    output.parent.mkdir(parents=True, exist_ok=True)
    if hasattr(ET, "indent"):
        ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return output.resolve()


def build_odom_prior_diagnostic_config(base: Path, output: Path) -> Path:
    """Derive a simulation-only SLAM config without mutating product defaults."""

    text = base.read_text(encoding="utf-8")
    marker = "odom_prior_enabled: false"
    if marker not in text:
        raise ValueError(f"SLAM config does not expose disabled odom prior: {base}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        text.replace(marker, "odom_prior_enabled: true", 1),
        encoding="utf-8",
    )
    return output.resolve()


def _sensor_publisher_write_args(tolerances: Mapping[str, Any]) -> list[str]:
    if "sensor_publisher_write_mode" not in tolerances:
        return []
    mode = str(tolerances["sensor_publisher_write_mode"])
    if mode not in {"sync", "async_fifo"}:
        raise ValueError(f"unsupported sensor publisher write mode: {mode}")
    args = ["--publisher-write-mode", mode]
    option_specs = (
        ("sensor_publisher_async_max_bytes", "--async-publisher-max-bytes", int),
        ("sensor_publisher_async_max_records", "--async-publisher-max-records", int),
        ("sensor_publisher_async_max_batches", "--async-publisher-max-batches", int),
        ("sensor_publisher_async_oldest_s", "--async-publisher-oldest-s", float),
        ("sensor_publisher_async_shutdown_s", "--async-publisher-shutdown-s", float),
    )
    for key, option, converter in option_specs:
        if key in tolerances:
            args.extend([option, str(converter(tolerances[key]))])
    return args


def _map_scene_monitor_command(
    *,
    navigation_control: Path,
    output: Path,
    domain_id: int,
) -> list[str]:
    """Build the read-only native MapScene ROI observer command."""

    from sim.scripts.mujoco import native_navigation_acceptance as native

    script = ROOT / "sim" / "scripts" / "mujoco" / "map_scene_roi_monitor.py"
    client_library = Path(navigation_control).with_name("liblingtu_nav_client.so")
    center = [float(value) for value in DYNAMIC_PERSON_CONTRACT["roi_center"]]
    half = [float(value) for value in DYNAMIC_PERSON_CONTRACT["roi_half_extent"]]
    args = [
        "--library",
        native._linux_arg(client_library),
        "--domain-id",
        str(int(domain_id)),
        "--output",
        native._linux_arg(output),
        "--center-x",
        str(center[0]),
        "--center-y",
        str(center[1]),
        "--center-z",
        str(center[2]),
        "--half-x",
        str(half[0]),
        "--half-y",
        str(half[1]),
        "--half-z",
        str(half[2]),
        "--poll-hz",
        str(float(DYNAMIC_PERSON_CONTRACT["scene_poll_hz"])),
    ]
    if os.name != "nt":
        return [sys.executable, str(script), *args]
    wsl = shutil.which("wsl.exe") or shutil.which("wsl")
    if not wsl:
        raise FileNotFoundError("wsl.exe is required for the native MapScene monitor")
    return [
        wsl,
        "-e",
        "env",
        f"PYTHONPATH={native._linux_arg(SRC)}",
        "python3",
        native._linux_arg(script),
        *args,
    ]


def build_execution_plan(
    *,
    scenario: str,
    domain_id: int,
    binaries: Mapping[str, Path],
    paths: Mapping[str, Path],
    case_dir: Path,
    duration_s: float,
    warmup_s: float,
    command_vx: float,
    manifest: Mapping[str, Any],
    external_arm_timeout_s: float = 120.0,
) -> dict[str, Any]:
    """Describe the exact real processes and artifacts for one scenario."""

    from sim.scripts.mujoco import native_navigation_acceptance as native

    case_dir = case_dir.resolve()
    case_dir.mkdir(parents=True, exist_ok=True)
    external_arm_token = secrets.token_hex(16)
    external_arm_timeout = max(1.0, float(external_arm_timeout_s))
    artifacts = {
        "scene": str(case_dir / "scene.xml"),
        "slam_status": str(case_dir / "slam_status.json"),
        "slam_cloud_dir": str(case_dir / "slam_clouds"),
        "mapd_status": str(case_dir / "mapd_status.json"),
        "traversability_status": str(case_dir / "traversability_status.json"),
        "nav_status": str(case_dir / "nav_status.json"),
        "sensor_report": str(case_dir / "sensor_report.json"),
        "parent_sensor_diagnostics": str(case_dir / "parent_sensor_diagnostics.json"),
        "sensor_arm": str(case_dir / "sensor_arm.json"),
        "sensor_arm_status": str(case_dir / "sensor_arm_status.json"),
        "motion_log": str(case_dir / "motion.jsonl"),
        "nav_timeline": str(case_dir / "nav_timeline.jsonl"),
        "sensor_publisher_pid": str(case_dir / "sensor_publisher.pid"),
        "cmd_vel_tap_pid": str(case_dir / "cmd_vel_tap.pid"),
        "teleop_log": str(case_dir / "teleop_command.log"),
        "teleop_current_log": str(case_dir / "teleop_command_current.log"),
        "estop_latch": str(case_dir / "estop_latch.json"),
        "map_scene_roi": str(case_dir / "map_scene_roi.jsonl"),
    }
    Path(artifacts["slam_cloud_dir"]).mkdir(parents=True, exist_ok=True)
    slam_runtime = dict(manifest.get("slam_runtime") or {})
    state_provider = str(slam_runtime.get("provider") or "fastlio2").strip().lower()
    slam_mode = str(slam_runtime.get("mode") or "mapping").strip().lower()
    if slam_mode != "mapping":
        raise ValueError(
            "teleop_avoid product acceptance requires slam_runtime.mode=mapping"
        )
    start = [float(value) for value in manifest.get("start") or [0.0, 0.0, 0.48, 0.0]]
    tolerances = dict(manifest.get("runtime_tolerances") or {})
    scene_variant = scenario.removesuffix("_injected")
    if scenario.endswith("_injected") or scene_variant in {
        "traversability_dropout_recovery",
        "slam_inputs_dropout_recovery",
    }:
        scene_variant = "free"
    obstacle_case = scenario in {"obstacle_slow", "obstacle_stop"}
    terrain_surface_case = scene_variant in TERRAIN_SCENE_CONTRACT
    obstacle_filter_excluded = obstacle_case or terrain_surface_case
    traversability_parameters = {
        "obstacle_min_z_m": 2.0 if obstacle_filter_excluded else 0.10,
        "obstacle_max_z_m": 3.0 if obstacle_filter_excluded else 1.20,
        "terrain_soft_height_m": 2.0 if obstacle_case else 0.08,
        "terrain_hard_height_m": 3.0 if obstacle_case else 0.20,
        "terrain_soft_slope_deg": 100.0 if obstacle_case else 12.0,
        "terrain_hard_slope_deg": 120.0 if obstacle_case else 28.0,
    }

    if state_provider != "mujoco_navigation_fixture":
        slam_command = native._native_command(
            Path(binaries["slam"]),
            "--backend",
            "fastlio2",
            "--mode",
            slam_mode,
            "--config",
            native._linux_arg(Path(paths["slam_config"])),
            "--domain-id",
            str(domain_id),
            "--tick-hz",
            "50",
            "--status-json",
            native._linux_arg(Path(artifacts["slam_status"])),
            "--status-json-hz",
            "10",
            "--cloud-snapshot-dir",
            native._linux_arg(Path(artifacts["slam_cloud_dir"])),
            "--cloud-snapshot-hz",
            "2",
        )
    else:
        slam_command = None
    if state_provider != "mujoco_navigation_fixture":
        mapd_command = native._native_command(
            Path(binaries["mapd"]),
            "--domain-id",
            str(domain_id),
            "--status-file",
            native._linux_arg(Path(artifacts["mapd_status"])),
            "--state-hz",
            str(MAPD_RUNTIME_PROFILE["state_hz"]),
            "--cloud-hz",
            str(MAPD_RUNTIME_PROFILE["cloud_hz"]),
            "--map-hz",
            str(MAPD_RUNTIME_PROFILE["map_hz"]),
            "--scene-hz",
            str(MAPD_RUNTIME_PROFILE["scene_hz"]),
            "--max-points",
            str(MAPD_RUNTIME_PROFILE["max_points"]),
            "--max-cloud-bytes",
            str(MAPD_RUNTIME_PROFILE["max_cloud_bytes"]),
            "--max-fields",
            str(MAPD_RUNTIME_PROFILE["max_fields"]),
            "--max-point-step",
            str(MAPD_RUNTIME_PROFILE["max_point_step"]),
            "--max-string-bytes",
            str(MAPD_RUNTIME_PROFILE["max_string_bytes"]),
            "--max-scene-bytes",
            str(MAPD_RUNTIME_PROFILE["max_scene_bytes"]),
            "--max-voxel-snapshot-points",
            str(MAPD_RUNTIME_PROFILE["max_voxel_snapshot_points"]),
            "--voxel-snapshot-radius",
            str(MAPD_RUNTIME_PROFILE["voxel_snapshot_radius_m"]),
            "--max-voxels",
            str(MAPD_RUNTIME_PROFILE["max_voxels"]),
            "--max-accumulated-cells",
            str(MAPD_RUNTIME_PROFILE["max_accumulated_cells"]),
            "--max-accumulated-blocks",
            str(MAPD_RUNTIME_PROFILE["max_accumulated_blocks"]),
            "--carve-min-z",
            str(MAPD_RUNTIME_PROFILE["carve_min_z_m"]),
            "--carve-max-z",
            str(MAPD_RUNTIME_PROFILE["carve_max_z_m"]),
            "--min-range",
            str(MAPD_RUNTIME_PROFILE["min_range_m"]),
            "--max-range",
            str(MAPD_RUNTIME_PROFILE["max_range_m"]),
            "--decay-ms",
            str(MAPD_RUNTIME_PROFILE["decay_ms"]),
            "--stale-ms",
            str(MAPD_RUNTIME_PROFILE["stale_ms"]),
        )
    else:
        mapd_command = None
    traversability_command = native._native_command(
        Path(binaries["traversability"]),
        "--domain-id",
        str(domain_id),
        "--publish-hz",
        str(FIELD_TELEOP_AVOID_PROFILE["traversability_publish_hz"]),
        "--slow-hz",
        str(FIELD_TELEOP_AVOID_PROFILE["traversability_slow_hz"]),
        "--tick-hz",
        str(FIELD_TELEOP_AVOID_PROFILE["traversability_tick_hz"]),
        "--cloud-pose-max-gap-s",
        str(FIELD_TELEOP_AVOID_PROFILE["traversability_cloud_pose_max_gap_s"]),
        "--resolution",
        "0.2",
        "--radius",
        "6",
        "--z-max",
        f"{traversability_parameters['obstacle_max_z_m']:.2f}",
        "--obstacle-min-z",
        f"{traversability_parameters['obstacle_min_z_m']:.2f}",
        "--robot-radius",
        "0.45",
        "--observed-free-ttl-s",
        str(FIELD_TELEOP_AVOID_PROFILE["observed_free_ttl_s"]),
        "--terrain-soft-height-m",
        f"{traversability_parameters['terrain_soft_height_m']:.2f}",
        "--terrain-hard-height-m",
        f"{traversability_parameters['terrain_hard_height_m']:.2f}",
        "--terrain-soft-slope-deg",
        f"{traversability_parameters['terrain_soft_slope_deg']:.1f}",
        "--terrain-hard-slope-deg",
        f"{traversability_parameters['terrain_hard_slope_deg']:.1f}",
        "--terrain-cache-max-points",
        "20000",
        "--max-points",
        "5000",
        "--sensor-offset-x-m",
        str(FIELD_TELEOP_AVOID_PROFILE["sensor_offset_x_m"]),
        "--sensor-offset-y-m",
        str(FIELD_TELEOP_AVOID_PROFILE["sensor_offset_y_m"]),
        "--sensor-offset-z-m",
        str(FIELD_TELEOP_AVOID_PROFILE["sensor_offset_z_m"]),
        "--status-file",
        native._linux_arg(Path(artifacts["traversability_status"])),
    )
    navigation_command = native._native_command(
        Path(binaries["navigation"]),
        "--control-mode",
        "teleop_avoid",
        "--domain-id",
        str(domain_id),
        "--tick-hz",
        "20",
        "--max-obstacle-points",
        "5000",
        "--odom-max-age-s",
        str(FIELD_TELEOP_AVOID_PROFILE["odom_max_age_s"]),
        "--tf-max-age-s",
        str(FIELD_TELEOP_AVOID_PROFILE["tf_max_age_s"]),
        "--cloud-max-age-s",
        str(FIELD_TELEOP_AVOID_PROFILE["cloud_max_age_s"]),
        "--cloud-pose-max-gap-s",
        str(FIELD_TELEOP_AVOID_PROFILE["cloud_pose_max_gap_s"]),
        "--traversability-max-age-s",
        str(TRAVERSABILITY_MAX_AGE_S),
        "--localization-health-max-age-s",
        str(FIELD_TELEOP_AVOID_PROFILE["localization_health_max_age_s"]),
        "--input-future-tolerance-s",
        str(float(tolerances.get("input_future_tolerance_s") or 0.05)),
        "--input-recovery-frames",
        str(FIELD_TELEOP_AVOID_PROFILE["input_recovery_frames"]),
        "--stop-confirmation-timeout-s",
        str(FIELD_TELEOP_AVOID_PROFILE["stop_confirmation_timeout_s"]),
        "--publish-cmd-vel",
        "true",
        "--teleop-local-planner",
        "true",
        "--path-library",
        native._linux_arg(Path(paths["path_library"])),
        "--check-obstacle",
        "true",
        "--use-traversability-cost",
        "true",
        "--allow-legacy-motion-inputs",
        "false",
        "--status-file",
        native._linux_arg(Path(artifacts["nav_status"])),
        "--status-s",
        "0.05",
        "--estop-latch-file",
        native._linux_arg(Path(artifacts["estop_latch"])),
    )
    sensor_command = [
        sys.executable,
        str(Path(paths["sensor_runner"])),
        "--world",
        artifacts["scene"],
        "--start",
        ",".join(str(value) for value in start[:3]),
        "--start-anchor",
        "warmup",
        "--duration",
        str(max(1.0, float(duration_s))),
        "--settle-s",
        "1.0",
        "--warmup-s",
        str(max(0.0, float(warmup_s))),
        "--drive-ramp-s",
        "0",
        "--drive-mode",
        "policy",
        "--policy-path",
        str(Path(paths["policy"])),
        "--command-source",
        "dds",
        "--cmd-vel-tap-bin",
        str(Path(binaries["cmd_vel_tap"])),
        "--cmd-vel-pid-file",
        artifacts["cmd_vel_tap_pid"],
        "--cmd-vel-timeout-s",
        "0.25",
        "--sim-hardware-realtime-factor",
        str(float(tolerances.get("sim_hardware_realtime_factor") or 1.0)),
        "--publisher-bin",
        str(Path(binaries["sensor_publisher"])),
        "--publisher-pid-file",
        artifacts["sensor_publisher_pid"],
        "--domain-id",
        str(domain_id),
        "--external-arm-file",
        artifacts["sensor_arm"],
        "--external-arm-token",
        external_arm_token,
        "--external-arm-timeout-s",
        str(external_arm_timeout),
        "--external-arm-status-json",
        artifacts["sensor_arm_status"],
        "--external-arm-scenario",
        scenario,
        "--slam-status-json",
        artifacts["slam_status"],
        "--require-slam-output",
        "--motion-log",
        artifacts["motion_log"],
        "--motion-log-hz",
        "20",
        "--motion-log-lidar-points",
        "0",
        "--nav-status-json",
        artifacts["nav_status"],
        "--json-out",
        artifacts["sensor_report"],
        *native._parent_sensor_diagnostics_args(
            Path(artifacts["parent_sensor_diagnostics"]),
            tolerances,
        ),
    ]
    if "sim_hardware_catch_up_yield_steps" in tolerances:
        sensor_command.extend(
            [
                "--sim-hardware-catch-up-yield-steps",
                str(int(tolerances["sim_hardware_catch_up_yield_steps"])),
            ]
        )
    sensor_command.extend(_sensor_publisher_write_args(tolerances))
    sensor_command.extend(native._sensor_runtime_args(dict(manifest)))
    if scenario == "moving_person_clear":
        sensor_command.extend(
            [
                "--mocap-motion-body",
                str(DYNAMIC_PERSON_CONTRACT["body_name"]),
                "--mocap-motion-start",
                ",".join(
                    str(value)
                    for value in DYNAMIC_PERSON_CONTRACT["start_xyz"]
                ),
                "--mocap-motion-end",
                ",".join(
                    str(value)
                    for value in DYNAMIC_PERSON_CONTRACT["end_xyz"]
                ),
                "--mocap-motion-start-s",
                str(float(DYNAMIC_PERSON_CONTRACT["motion_start_s"])),
                "--mocap-motion-duration-s",
                str(float(DYNAMIC_PERSON_CONTRACT["motion_duration_s"])),
            ]
        )
    if state_provider == "mujoco_navigation_fixture":
        if "--slam-status-json" in sensor_command:
            index = sensor_command.index("--slam-status-json")
            del sensor_command[index : index + 2]
        sensor_command = [value for value in sensor_command if value != "--require-slam-output"]
        if "--navigation-fixture" not in sensor_command:
            sensor_command.append("--navigation-fixture")
        if "--publish-odom-prior" not in sensor_command:
            sensor_command.append("--publish-odom-prior")
        if "--scan-time-profile" in sensor_command:
            sensor_command[sensor_command.index("--scan-time-profile") + 1] = "instantaneous"
        else:
            sensor_command.extend(["--scan-time-profile", "instantaneous"])
        if "--navigation-fixture-cloud-points" not in sensor_command:
            sensor_command.extend(["--navigation-fixture-cloud-points", "15000"])
    if bool(manifest.get("_odom_prior_diagnostic")):
        sensor_command.append("--allow-kinematic-fastlio-acceptance")
    teleop_command = native._native_command(
        Path(binaries["navigation_control"]),
        "operator-motion",
        str(float(command_vx)),
        "0",
        "0",
        "--duration-s",
        str(max(60.0, 3.0 * (float(warmup_s) + float(duration_s)))),
        "--rate-hz",
        "10",
        "--domain-id",
        str(domain_id),
        "--source-id",
        f"mujoco-teleop-avoid-{domain_id}",
        "--lease-ttl-ms",
        "2000",
        "--freshness-budget-ms",
        "350",
        "--cleanup-settle-ms",
        "300",
        "--timeout-ms",
        "3000",
    )
    processes = []
    if slam_command is not None:
        processes.append({"name": "slam", "command": slam_command, "log": str(case_dir / "slam.log")})
    if mapd_command is not None:
        processes.append({"name": "mapd", "command": mapd_command, "log": str(case_dir / "mapd.log")})
    if scenario == "moving_person_clear" and mapd_command is not None:
        processes.append(
            {
                "name": "map_scene_monitor",
                "command": _map_scene_monitor_command(
                    navigation_control=Path(binaries["navigation_control"]),
                    output=Path(artifacts["map_scene_roi"]),
                    domain_id=domain_id,
                ),
                "log": str(case_dir / "map_scene_monitor.log"),
            }
        )
    processes.extend([
        {"name": "traversability", "command": traversability_command, "log": str(case_dir / "traversability.log")},
        {"name": "navigation", "command": navigation_command, "log": str(case_dir / "navigation.log")},
        {"name": "sensor", "command": sensor_command, "log": str(case_dir / "sensor.log")},
    ])
    return {
        "scenario": scenario,
        "scene_variant": scene_variant,
        "domain_id": int(domain_id),
        "product_contract": {
            "product": "teleop_avoid",
            "native_control_mode": "teleop_avoid",
            "slam_mode": slam_mode,
            "requires_map": False,
        },
        "processes": processes,
        "teleop_command": teleop_command,
        "artifacts": artifacts,
        "external_arm": {
            "required": True,
            "schema": EXTERNAL_ARM_SCHEMA,
            "status_schema": EXTERNAL_ARM_STATUS_SCHEMA,
            "token": external_arm_token,
            "token_sha256_12": hashlib.sha256(external_arm_token.encode()).hexdigest()[:12],
            "timeout_s": external_arm_timeout,
        },
        "functional_scope": {
            "live_obstacle_layer": not terrain_surface_case,
            "terrain_surface_isolation": terrain_surface_case,
            "mapd_process": mapd_command is not None,
            "dynamic_obstacle_residual_gate": scenario == "moving_person_clear",
            "mapd_data_contract": dict(MAPD_DATA_CONTRACT),
            "traversability_process": True,
            "traversability_cost_in_decision": True,
            "traversability_parameters": traversability_parameters,
            "isolation": (
                "live_obstacle_decision_with_free_cost_producer_thresholds"
                if obstacle_case
                else "terrain_surface_decision_with_obstacle_layer_excluded"
                if terrain_surface_case
                else "full_teleop_avoid_inputs"
            ),
        },
        "terrain_producer_contract": {
            **TERRAIN_PRODUCER_CONTRACT,
            "scenario_geometry": dict(TERRAIN_SCENE_CONTRACT.get(scene_variant) or {}),
        },
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
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
    parser.add_argument("--domain-base", type=int, default=226)
    parser.add_argument("--duration-s", type=float, default=25.0)
    parser.add_argument("--warmup-s", type=float, default=15.0)
    parser.add_argument("--startup-timeout-s", type=float, default=60.0)
    parser.add_argument("--driving-start-timeout-s", type=float, default=45.0)
    parser.add_argument("--command-vx", type=float, default=0.18)
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


def _requested_scenarios(values: Sequence[str] | None) -> list[str]:
    selected = list(values or ["all"])
    if "all" in selected:
        return list(DEFAULT_SCENARIOS)
    return list(dict.fromkeys(selected))


def _write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

def _external_arm_token_digest(token: str) -> str:
    return hashlib.sha256(token.encode("utf-8")).hexdigest()[:12]


def trigger_external_arm(
    path: Path,
    *,
    token: str,
    domain_id: int,
    scenario: str,
) -> dict[str, Any]:
    """Atomically arm one sensor run with a case-scoped identity."""

    normalized_token = str(token).strip()
    normalized_scenario = str(scenario).strip()
    if not normalized_token:
        raise ValueError("external arm token is empty")
    if not normalized_scenario:
        raise ValueError("external arm scenario is empty")
    payload = {
        "schema": EXTERNAL_ARM_SCHEMA,
        "arm": True,
        "token": normalized_token,
        "domain_id": int(domain_id),
        "scenario": normalized_scenario,
    }
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(f".{target.name}.tmp")
    not_before_ns = time.time_ns()
    temporary.write_text(
        json.dumps(payload, ensure_ascii=True, sort_keys=True, separators=(",", ":")),
        encoding="utf-8",
    )
    os.replace(temporary, target)
    return {
        "triggered": True,
        "path": str(target),
        "not_before_ns": not_before_ns,
        "trigger_wall_s": time.time(),
        "schema": EXTERNAL_ARM_SCHEMA,
        "domain_id": int(domain_id),
        "scenario": normalized_scenario,
        "token_sha256_12": _external_arm_token_digest(normalized_token),
    }


def external_arm_status_evidence(
    path: Path,
    *,
    token: str,
    domain_id: int,
    scenario: str,
    not_before_ns: int = 0,
) -> dict[str, Any]:
    """Validate one sensor arm acknowledgement without exposing its token."""

    source = Path(path)
    try:
        stat = source.stat()
        value = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        stat = None
        value = {}
    status = value if isinstance(value, Mapping) else {}
    bounded_status = {
        "schema": str(status.get("schema") or ""),
        "enabled": status.get("enabled") is True,
        "state": str(status.get("state") or ""),
        "acknowledged": status.get("acknowledged") is True,
        "domain_id": status.get("domain_id"),
        "scenario": str(status.get("scenario") or ""),
        "token_sha256_12": str(status.get("token_sha256_12") or ""),
        "arm_observed_sim_time_s": status.get("arm_observed_sim_time_s"),
        "wait_elapsed_wall_s": status.get("wait_elapsed_wall_s"),
        "last_error": str(status.get("last_error") or ""),
    }
    blockers: list[str] = []
    if stat is None or not status:
        blockers.append("external_arm_status_missing")
    else:
        if int(stat.st_mtime_ns) < max(0, int(not_before_ns)):
            blockers.append("external_arm_status_stale")
        if bounded_status["schema"] != EXTERNAL_ARM_STATUS_SCHEMA:
            blockers.append("external_arm_status_schema_mismatch")
        if bounded_status["enabled"] is not True:
            blockers.append("external_arm_status_not_enabled")
        if bounded_status["state"] != "armed":
            blockers.append("external_arm_status_not_armed")
        if bounded_status["acknowledged"] is not True:
            blockers.append("external_arm_ack_missing")
        arm_observed_sim_time_s = bounded_status["arm_observed_sim_time_s"]
        if (
            not isinstance(arm_observed_sim_time_s, (int, float))
            or not math.isfinite(float(arm_observed_sim_time_s))
        ):
            blockers.append("external_arm_observed_time_invalid")
        if bounded_status["domain_id"] != int(domain_id):
            blockers.append("external_arm_domain_mismatch")
        if bounded_status["scenario"] != str(scenario):
            blockers.append("external_arm_scenario_mismatch")
        if bounded_status["token_sha256_12"] != _external_arm_token_digest(str(token)):
            blockers.append("external_arm_token_mismatch")
    return {
        "acknowledged": not blockers,
        "path": str(source),
        "mtime_ns": int(stat.st_mtime_ns) if stat is not None else 0,
        "not_before_ns": max(0, int(not_before_ns)),
        "status": bounded_status,
        "blockers": blockers,
    }


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _binary_source_provenance(
    binaries: Mapping[str, Path],
) -> tuple[dict[str, dict[str, Any]], list[str]]:
    """Record binary identity and reject core artifacts older than their sources."""

    nav_cpp = ROOT / "src" / "nav" / "cpp"
    endpoint_cpp = nav_cpp / "endpoint"
    message_cpp = ROOT / "src" / "message" / "cpp"
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
    vendored_small_gicp = (
        ROOT
        / "third_party"
        / "research_localization"
        / "small_gicp"
        / "include"
        / "small_gicp"
    )
    optional_small_gicp_sources = (
        [vendored_small_gicp] if vendored_small_gicp.is_dir() else []
    )
    common_sources = [
        ROOT / "src" / "message" / "idl" / "lingtu_slam.idl",
        message_cpp / "CMakeLists.txt",
        message_cpp / "dds_topics.hpp",
        message_cpp / "dds_qos_profiles.hpp",
    ]
    source_specs = {
        "navigation": [
            endpoint_cpp,
            nav_cpp / "planning",
            nav_cpp / "control",
            nav_cpp / "engine",
            nav_cpp / "include",
            nav_cpp / "CMakeLists.txt",
            nav_cpp / "cmake" / "NavCoreTargets.cmake",
            endpoint_cpp / "CMakeLists.txt",
            nav_cpp / "planning" / "global" / "octoplanner" / "CMakeLists.txt",
            ROOT / "src" / "explore" / "cpp" / "explore_contract.hpp",
            message_cpp / "inspection_command.hpp",
            message_cpp / "navigation_command.hpp",
            message_cpp / "operator_motion.hpp",
            message_cpp / "snapshot_file.hpp",
            *inspection_core_sources,
            *maps_core_sources,
            *common_sources,
        ],
        "navigation_control": [
            nav_cpp / "client",
            endpoint_cpp / "motion" / "nav_control.cpp",
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
            endpoint_cpp / "frame_transform.hpp",
            endpoint_cpp / "plan" / "dds_drain_policy.hpp",
            endpoint_cpp / "plan" / "input_gate.hpp",
            nav_cpp / "include" / "nav_kernel" / "dynamic_clear_core.hpp",
            nav_cpp / "include" / "nav_kernel" / "terrain_core.hpp",
            nav_cpp / "include" / "nav_kernel" / "types.hpp",
            nav_cpp / "CMakeLists.txt",
            nav_cpp / "cmake" / "NavCoreTargets.cmake",
            endpoint_cpp / "CMakeLists.txt",
            message_cpp / "snapshot_file.hpp",
            *maps_core_sources,
            *common_sources,
        ],
        "slam": [
            ROOT / "src" / "localization" / "slam" / "cpp",
            ROOT / "src" / "localization" / "slam" / "cpp" / "CMakeLists.txt",
            ROOT / "src" / "localization" / "fastlio2" / "src",
            ROOT / "src" / "localization" / "localizer" / "src" / "localizers",
            ROOT
            / "src"
            / "maps"
            / "include"
            / "lingtu"
            / "maps"
            / "c_api"
            / "semantic_occupancy.h",
            message_cpp / "snapshot_file.hpp",
            *optional_small_gicp_sources,
            *common_sources,
        ],
        "sensor_publisher": [
            ROOT / "src" / "drivers" / "real" / "lidar" / "sdk2_stream",
            ROOT / "src" / "drivers" / "real" / "lidar" / "sdk2_stream" / "CMakeLists.txt",
            ROOT / "src" / "drivers" / "real" / "lidar" / "native",
            *common_sources,
        ],
        "cmd_vel_tap": [
            ROOT / "sim" / "native_dds" / "cmd_vel_tap.cpp",
            ROOT / "sim" / "native_dds" / "CMakeLists.txt",
            *common_sources,
        ],
        "mapd": [
            *maps_core_sources,
            *common_sources,
        ],
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
            "sha256": _sha256_file(binary),
        }
        if name == "navigation_control":
            client_library = binary.with_name("liblingtu_nav_client.so")
            if client_library.is_file():
                library_stat = client_library.stat()
                item["runtime_dependencies"] = {
                    "lingtu_nav_client": {
                        "path": str(client_library),
                        "size_bytes": int(library_stat.st_size),
                        "mtime_ns": int(library_stat.st_mtime_ns),
                        "sha256": _sha256_file(client_library),
                    }
                }
            else:
                blockers.append("native_runtime_dependency_missing:navigation_control")
        specs = source_specs.get(name) or []
        if specs:
            item["source_specs"] = [str(source) for source in specs]
            missing_specs = [
                str(source)
                for source in specs
                if not source.is_file() and not source.is_dir()
            ]
            if missing_specs:
                item["missing_source_specs"] = missing_specs
                blockers.extend(
                    f"native_source_spec_missing:{name}:{source}"
                    for source in missing_specs
                )
                provenance[name] = item
                continue
            source_files: list[Path] = []
            for source in specs:
                if source.is_file():
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
                    if path.is_file() and not path.name.startswith("test_") and "tests" not in path.parts
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
                library_mtime_ns = int(item["runtime_dependencies"]["lingtu_nav_client"]["mtime_ns"])
                if library_mtime_ns < latest_source_mtime_ns:
                    blockers.append("native_runtime_dependency_stale:navigation_control")
        provenance[name] = item
    return provenance, blockers


def _teleop_product_contract_evidence(
    manifest: Mapping[str, Any],
) -> dict[str, Any]:
    """Bind this harness to the canonical teleop_avoid Product declaration."""

    from runtime.profiles.product_lifecycle import product_lifecycle

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
            blockers.append(
                f"teleop_product_contract_mismatch:{field}:expected={expected}:actual={actual}"
            )

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
                blockers.append(
                    f"teleop_asset_scene_invalid:{scene_xml}:{type(exc).__name__}"
                )
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
        "reason": (
            "teleop_scene_prepared"
            if evidence.get("ok") is True
            else "teleop_scene_invalid"
        ),
        "builder": spec,
        **evidence,
    }


def _policy_runtime_evidence(*, required: bool) -> dict[str, Any]:
    """Check the Python modules required by the MuJoCo policy runner."""

    modules = ("mujoco", "onnxruntime")
    missing = [name for name in modules if importlib.util.find_spec(name) is None]
    blockers = (
        [f"python_runtime_dependency_missing:{name}" for name in missing]
        if required
        else []
    )
    return {
        "required": required,
        "python": sys.executable,
        "modules": {name: name not in missing for name in modules},
        "blockers": blockers,
        "ok": not blockers,
    }


def prepare_runtime(args: argparse.Namespace) -> dict[str, Any]:
    """Prepare the map-free teleop_avoid Product and resolve native dependencies."""

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
    state_provider = str(
        ((manifest.get("slam_runtime") or {}).get("provider") or "fastlio2")
    ).strip().lower()
    if state_provider == "mujoco_navigation_fixture":
        binaries.pop("mapd", None)
    required_binaries = {
        "sensor_publisher",
        "traversability",
        "navigation",
        "navigation_control",
        "cmd_vel_tap",
    }
    if state_provider != "mujoco_navigation_fixture":
        required_binaries.update({"slam", "mapd"})

    out_of_scope: list[str] = []
    in_scope: list[str] = []
    for blocker in blockers:
        missing_binary = (
            blocker.split(":", 1)[1]
            if blocker.startswith("native_binary_missing:")
            else ""
        )
        if missing_binary and missing_binary not in required_binaries:
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
    binary_provenance, stale_blockers = _binary_source_provenance(binaries)
    blockers.extend(stale_blockers)
    if os.name != "nt":
        blockers.append("host_contract_requires_windows_wsl2")
    if "sensor_publisher_dds_unavailable" in blockers and "sensor_publisher" in binaries:
        for _ in range(2):
            publisher_ok, publisher_probe = native._probe_sensor_publisher(
                binaries["sensor_publisher"]
            )
            provenance["sensor_publisher_probe_retry"] = publisher_probe
            if publisher_ok:
                blockers = [
                    value
                    for value in blockers
                    if value != "sensor_publisher_dds_unavailable"
                ]
                break
            time.sleep(0.5)
    if asset_preparation.get("ok") is not True:
        blockers.append(
            str(asset_preparation.get("reason") or "asset_preparation_failed")
        )
        blockers.extend(
            str(value) for value in asset_preparation.get("blockers") or ()
        )
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
            "out_of_scope_preflight_findings": out_of_scope,
            "binaries": {name: str(path) for name, path in binaries.items()},
            "paths": {name: str(path) for name, path in paths.items()},
        },
    }


def mapd_status_evidence(
    path: Path,
    *,
    required: bool,
    not_before_ns: int = 0,
    now_ns: int | None = None,
    evidence_scope: str = "product_e2e",
    product_gate_eligible: bool = True,
    omission_reason: str = "",
) -> dict[str, Any]:
    """Return one bounded, freshness-checked native mapd evidence snapshot."""

    from sim.scripts.mujoco import native_navigation_acceptance as native

    source = Path(path)
    base: dict[str, Any] = {
        "required": bool(required),
        "status_file": str(source),
        "source_topic": MAPD_DATA_CONTRACT["input"],
        "scene_topic": MAPD_DATA_CONTRACT["scene"],
        "navigation_traversability_topic": MAPD_DATA_CONTRACT["navigation_traversability"],
        "navigation_traversability_role": MAPD_DATA_CONTRACT["navigation_traversability_role"],
        "evidence_scope": evidence_scope,
        "product_evidence": False,
    }
    if not required:
        return {
            **base,
            "evaluated": False,
            "available": False,
            "fresh": False,
            "ok": False,
            "coverage": "not_covered",
            "omission_allowed": bool(omission_reason),
            "omission_reason": omission_reason,
            "status": {},
            "blockers": [],
        }

    try:
        status_stat = source.stat()
    except OSError:
        status_stat = None
    status = native._load_json(source) if status_stat is not None else {}
    observed_now_ns = time.time_ns() if now_ns is None else int(now_ns)
    status_mtime_ns = int(status_stat.st_mtime_ns) if status_stat is not None else 0
    age_s = (
        max(0.0, (observed_now_ns - status_mtime_ns) / 1_000_000_000.0)
        if status_stat is not None
        else None
    )
    fresh = bool(
        status_stat is not None
        and status_mtime_ns >= max(0, int(not_before_ns))
        and age_s is not None
        and age_s <= MAPD_STATUS_MAX_AGE_S
    )
    numeric_fields = (
        "accepted_observations",
        "processed_observations",
        "generation",
        "dds_received",
        "dds_decoded",
        "dds_rejected",
        "dds_write_attempts",
        "dds_write_failures",
        "dds_serialization_rejections",
        "dds_scene_oversize_rejections",
        "dds_unhealthy_writers",
        "state_published_generation",
        "realtime_clouds_published_generation",
        "map_layers_published_generation",
        "scene_published_generation",
        "voxel_capacity_rejections",
        "accumulated_capacity_rejections",
    )
    numeric: dict[str, int | None] = {}
    for field in numeric_fields:
        value = status.get(field)
        numeric[field] = value if isinstance(value, int) and not isinstance(value, bool) else None
    bounded_status = {
        "schema_version": str(status.get("schema_version") or ""),
        "process": str(status.get("process") or ""),
        "status": str(status.get("status") or ""),
        "ready": status.get("ready") is True,
        "running": status.get("running") is True,
        "live": status.get("live") is True,
        "required_publications_ready": status.get("required_publications_ready") is True,
        "current_generation_published": status.get("current_generation_published") is True,
        "capacity_limited": (
            status.get("capacity_limited")
            if isinstance(status.get("capacity_limited"), bool)
            else None
        ),
        **numeric,
    }
    blockers: list[str] = []
    if status_stat is None or not status:
        blockers.append("mapd_status_missing")
    else:
        if not fresh:
            blockers.append("mapd_status_stale")
        if bounded_status["schema_version"] != MAPD_STATUS_SCHEMA:
            blockers.append("mapd_status_schema_invalid")
        if bounded_status["process"] != "mapd":
            blockers.append("mapd_status_process_invalid")
        if bounded_status["status"] != "ready":
            blockers.append("mapd_status_status_not_ready")
        for field in (
            "ready",
            "running",
            "live",
            "required_publications_ready",
            "current_generation_published",
        ):
            if bounded_status[field] is not True:
                blockers.append(f"mapd_status_{field}_false")
        for field in (
            "accepted_observations",
            "processed_observations",
            "generation",
            "dds_received",
            "dds_decoded",
            "dds_write_attempts",
        ):
            value = numeric[field]
            if value is None or value <= 0:
                blockers.append(f"mapd_status_{field}_not_positive")
        for field in (
            "dds_rejected",
            "dds_write_failures",
            "dds_serialization_rejections",
            "dds_scene_oversize_rejections",
            "dds_unhealthy_writers",
            "voxel_capacity_rejections",
            "accumulated_capacity_rejections",
        ):
            if numeric[field] != 0:
                blockers.append(f"mapd_status_{field}_nonzero")
        generation = numeric["generation"]
        for field in (
            "state_published_generation",
            "realtime_clouds_published_generation",
            "map_layers_published_generation",
            "scene_published_generation",
        ):
            if generation is None or numeric[field] != generation:
                blockers.append(f"mapd_status_{field}_mismatch")
        if bounded_status["capacity_limited"] is not False:
            blockers.append("mapd_status_capacity_limited_not_false")
    ok = not blockers
    coverage = "product_e2e_mapd" if product_gate_eligible else "non_product_mapd_diagnostic"
    return {
        **base,
        "evaluated": True,
        "available": status_stat is not None and bool(status),
        "fresh": fresh,
        "status_mtime_ns": status_mtime_ns,
        "not_before_ns": max(0, int(not_before_ns)),
        "age_s": age_s,
        "max_age_s": MAPD_STATUS_MAX_AGE_S,
        "ok": ok,
        "coverage": coverage,
        "omission_allowed": False,
        "omission_reason": "",
        "status": bounded_status,
        "product_evidence": bool(ok and product_gate_eligible),
        "blockers": blockers,
    }


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    samples: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            samples.append(value)
    return samples


def evaluate_dynamic_obstacle_residual(
    scene_samples: Sequence[Mapping[str, Any]],
    motion: Mapping[str, Any],
    *,
    contract: Mapping[str, Any] = DYNAMIC_PERSON_CONTRACT,
) -> dict[str, Any]:
    """Prove detection and bounded clearing in the current mapd scene product."""

    blockers: list[str] = []
    samples = [
        dict(sample)
        for sample in scene_samples
        if sample.get("type") == "scene"
        and isinstance(sample.get("wall_s"), (int, float))
        and isinstance(sample.get("roi_counts"), Mapping)
    ]
    samples.sort(key=lambda sample: float(sample["wall_s"]))
    motion_start = motion.get("motion_start_wall_s")
    motion_complete = motion.get("motion_complete_wall_s")
    if (
        motion.get("enabled") is not True
        or motion.get("motion_started") is not True
        or motion.get("motion_completed") is not True
        or not isinstance(motion_start, (int, float))
        or not isinstance(motion_complete, (int, float))
        or not math.isfinite(float(motion_start))
        or not math.isfinite(float(motion_complete))
        or float(motion_complete) <= float(motion_start)
    ):
        blockers.append("dynamic_obstacle_motion_evidence_missing")
        return {
            "evaluated": True,
            "ok": False,
            "contract": dict(contract),
            "sample_count": len(samples),
            "layers": {},
            "blockers": blockers,
        }
    clear_grace_s = float(contract["clear_grace_s"])
    clear_not_before = float(motion_complete) + clear_grace_s
    baseline = [
        sample for sample in samples if float(sample["wall_s"]) < float(motion_start)
    ]
    active = [
        sample
        for sample in samples
        if float(motion_start) <= float(sample["wall_s"]) <= float(motion_complete)
    ]
    cleared = [
        sample for sample in samples if float(sample["wall_s"]) >= clear_not_before
    ]
    if len(baseline) < 2:
        blockers.append("dynamic_obstacle_baseline_scene_samples_missing")
    if len(active) < 2:
        blockers.append("dynamic_obstacle_active_scene_samples_missing")
    if len(cleared) < 2:
        blockers.append("dynamic_obstacle_post_clear_scene_samples_missing")

    identities = {
        (
            str(sample.get("producer_boot_id") or ""),
            int(sample.get("reset_epoch") or 0),
        )
        for sample in samples
    }
    if len(identities) != 1 or next(iter(identities), ("", 0))[0] == "":
        blockers.append("dynamic_obstacle_scene_identity_changed")
    generations = [int(sample.get("generation") or 0) for sample in samples]
    if any(current <= previous for previous, current in zip(generations, generations[1:])):
        blockers.append("dynamic_obstacle_scene_generation_not_monotonic")

    layer_metrics: dict[str, Any] = {}
    minimum_peak = int(contract["minimum_peak_excess_points"])
    maximum_residual = int(contract["maximum_residual_points"])
    maximum_fraction = float(contract["maximum_residual_fraction"])
    for layer in ("live", "voxel", "accumulated"):
        baseline_counts = [
            int((sample["roi_counts"] or {}).get(layer) or 0)
            for sample in baseline
        ]
        active_counts = [
            int((sample["roi_counts"] or {}).get(layer) or 0)
            for sample in active
        ]
        post_counts = [
            int((sample["roi_counts"] or {}).get(layer) or 0)
            for sample in cleared
        ]
        baseline_count = (
            float(statistics.median(baseline_counts[-5:]))
            if baseline_counts
            else 0.0
        )
        peak_count = max(active_counts, default=0)
        peak_excess = max(0.0, float(peak_count) - baseline_count)
        allowed_excess = max(
            maximum_residual,
            int(math.ceil(peak_excess * maximum_fraction)),
        )
        clear_threshold = baseline_count + allowed_excess
        final_count = (
            float(statistics.median(post_counts[-3:]))
            if post_counts
            else None
        )
        clear_sample = next(
            (
                sample
                for sample in cleared
                if int((sample["roi_counts"] or {}).get(layer) or 0)
                <= clear_threshold
            ),
            None,
        )
        layer_metrics[layer] = {
            "baseline_count": baseline_count,
            "peak_count": peak_count,
            "peak_excess": peak_excess,
            "allowed_residual_excess": allowed_excess,
            "clear_threshold": clear_threshold,
            "final_count": final_count,
            "clear_latency_s": (
                float(clear_sample["wall_s"]) - float(motion_complete)
                if clear_sample is not None
                else None
            ),
        }
        if peak_excess < minimum_peak:
            blockers.append(f"dynamic_obstacle_{layer}_visibility_missing")
        if layer in {"voxel", "accumulated"} and (
            final_count is None or final_count > clear_threshold
        ):
            blockers.append(f"dynamic_obstacle_{layer}_residual_excessive")

    return {
        "evaluated": True,
        "ok": not blockers,
        "contract": dict(contract),
        "sample_count": len(samples),
        "phase_samples": {
            "baseline": len(baseline),
            "active": len(active),
            "post_clear": len(cleared),
        },
        "motion_start_wall_s": float(motion_start),
        "motion_complete_wall_s": float(motion_complete),
        "clear_not_before_wall_s": clear_not_before,
        "scene_identity": [
            {"producer_boot_id": boot_id, "reset_epoch": epoch}
            for boot_id, epoch in sorted(identities)
        ],
        "layers": layer_metrics,
        "blockers": list(dict.fromkeys(blockers)),
    }


def _file_size_or_zero(path: Path) -> int:
    try:
        return int(path.stat().st_size)
    except OSError:
        return 0


def read_native_gate_transitions(
    path: Path,
    *,
    start_offset: int = 0,
    end_offset: int | None = None,
) -> list[str]:
    """Read de-duplicated input-gate states from one native log byte window."""

    try:
        with path.open("rb") as stream:
            start = max(0, int(start_offset))
            stream.seek(start)
            length = None if end_offset is None else max(0, int(end_offset) - start)
            payload = stream.read(length)
    except OSError:
        return []
    text = payload.decode("utf-8", errors="replace").replace("\x00", "")
    transitions: list[str] = []
    for match in re.finditer(r"\bgate=([a-z_]+)\b", text):
        value = match.group(1)
        if not transitions or transitions[-1] != value:
            transitions.append(value)
    return transitions


def contains_ordered_transition(values: Sequence[str], expected: Sequence[str]) -> bool:
    cursor = 0
    for value in values:
        if cursor < len(expected) and value == expected[cursor]:
            cursor += 1
    return cursor == len(expected)


def reset_case_artifacts(artifacts: Mapping[str, Path]) -> None:
    """Remove evidence from a prior run while preserving the case directory."""

    for name, source in artifacts.items():
        path = Path(source)
        if name == "slam_cloud_dir":
            if path.exists():
                shutil.rmtree(path)
            path.mkdir(parents=True, exist_ok=True)
        elif path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink(missing_ok=True)


def project_motion_timestamp(
    *,
    last_sensor_timestamp_s: float,
    last_write_wall_s: float,
    event_wall_s: float,
    realtime_factor: float,
) -> float:
    """Project a wall event onto the paced simulated-hardware clock."""

    elapsed_wall_s = max(0.0, float(event_wall_s) - float(last_write_wall_s))
    return float(last_sensor_timestamp_s) + elapsed_wall_s * max(0.0, float(realtime_factor))


def _motion_timestamp_for_event(
    path: Path,
    *,
    event_wall_s: float,
    realtime_factor: float,
) -> float | None:
    samples = _read_jsonl(path)
    if not samples:
        return None
    last_timestamp = samples[-1].get("t")
    if not isinstance(last_timestamp, (int, float)):
        return None
    try:
        last_write_wall_s = path.stat().st_mtime
    except OSError:
        return None
    return project_motion_timestamp(
        last_sensor_timestamp_s=float(last_timestamp),
        last_write_wall_s=last_write_wall_s,
        event_wall_s=float(event_wall_s),
        realtime_factor=float(realtime_factor),
    )


def _write_jsonl(path: Path, samples: Sequence[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(json.dumps(dict(sample), ensure_ascii=False) + "\n" for sample in samples),
        encoding="utf-8",
    )


def terrain_scene_forward_probe_attribution(
    scenario: str,
    status: Mapping[str, Any],
) -> dict[str, Any]:
    """Prove that the target scene, rather than a guard geom, drove terrain risk."""

    contract = TERRAIN_SCENE_CONTRACT.get(scenario)
    if contract is None:
        return {
            "required": False,
            "ok": True,
            "scenario": scenario,
            "candidate_samples": [],
            "matching_samples": [],
        }
    forward_probe = status.get("forward_probe")
    if not isinstance(forward_probe, Mapping):
        forward_probe = {}
    try:
        terrain_generation = int(forward_probe.get("terrain_generation") or 0)
    except (TypeError, ValueError):
        terrain_generation = 0
    component = str(contract["forward_probe_component"])
    x_min = float(contract["forward_probe_x_min_m"])
    x_max = float(contract["forward_probe_x_max_m"])
    cost_min = float(contract["forward_probe_min_cost"])
    maximum = contract["forward_probe_max_cost_exclusive"]
    cost_max = float(maximum) if maximum is not None else None
    abs_y_max = float(contract["forward_probe_abs_y_max_m"])
    occupancy_max = float(contract["forward_probe_max_occupancy_cost_exclusive"])
    fused_component_tolerance = 1e-3
    raw_samples = forward_probe.get("samples")
    samples = (
        raw_samples
        if isinstance(raw_samples, Sequence) and not isinstance(raw_samples, (str, bytes, bytearray))
        else ()
    )
    candidates: list[dict[str, Any]] = []
    matches: list[dict[str, Any]] = []
    for raw_sample in samples:
        if (
            not isinstance(raw_sample, Mapping)
            or raw_sample.get("used_by_teleop") is not True
            or raw_sample.get("in_bounds") is not True
            or raw_sample.get("observed_before_overlays") is not True
        ):
            continue
        try:
            map_x = float(raw_sample.get("map_x"))
            map_y = float(raw_sample.get("map_y"))
            component_cost = float(raw_sample.get(component))
            occupancy_cost = float(raw_sample.get("occupancy_cost"))
            fused_cost = float(raw_sample.get("fused_cost"))
        except (TypeError, ValueError):
            continue
        if not all(math.isfinite(value) for value in (map_x, map_y, component_cost, occupancy_cost, fused_cost)):
            continue
        if not x_min <= map_x <= x_max or abs(map_y) > abs_y_max:
            continue
        sample = {
            "map_x": map_x,
            "map_y": map_y,
            component: component_cost,
            "occupancy_cost": occupancy_cost,
            "fused_cost": fused_cost,
        }
        candidates.append(sample)
        if (
            component_cost >= cost_min
            and (cost_max is None or component_cost < cost_max)
            and occupancy_cost < occupancy_max
            and math.isclose(
                fused_cost,
                component_cost,
                abs_tol=fused_component_tolerance,
            )
        ):
            matches.append(sample)
    return {
        "required": True,
        "ok": terrain_generation > 0 and bool(matches),
        "scenario": scenario,
        "terrain_generation": terrain_generation,
        "component": component,
        "x_range_m": [x_min, x_max],
        "cost_range": [cost_min, cost_max],
        "abs_y_max_m": abs_y_max,
        "occupancy_cost_max_exclusive": occupancy_max,
        "fused_component_abs_tolerance": fused_component_tolerance,
        "candidate_samples": candidates,
        "matching_samples": matches,
    }


def _wait_for_terrain_scene_forward_probe(
    *,
    scenario: str,
    traversability_status: Path,
    timeout_s: float,
) -> dict[str, Any]:
    """Wait for a current forward-probe sample attributable to the terrain scene."""

    from sim.scripts.mujoco import native_navigation_acceptance as native

    deadline = time.monotonic() + max(0.1, float(timeout_s))
    result = terrain_scene_forward_probe_attribution(scenario, {})
    while time.monotonic() < deadline:
        result = terrain_scene_forward_probe_attribution(
            scenario,
            native._load_json(traversability_status),
        )
        if result["ok"]:
            return result
        time.sleep(0.05)
    return result


def _capture_nav_status(
    *,
    path: Path,
    phase: str,
    timeline: list[dict[str, Any]],
    state: dict[str, Any],
) -> dict[str, Any]:
    from sim.scripts.mujoco import native_navigation_acceptance as native

    nav = native._load_json(path)
    if not nav:
        return {}
    stamp = float(nav.get("stamp_s") or 0.0)
    key = (phase, stamp)
    if key != state.get("last_key"):
        timeline.append({"phase": phase, "wall_s": time.time(), "nav": nav})
        state["last_key"] = key
    state["last_nav"] = nav
    return nav


def _collect_for(
    *,
    sensor: Any,
    nav_status: Path,
    phase: str,
    timeline: list[dict[str, Any]],
    state: dict[str, Any],
    duration_s: float,
) -> dict[str, Any]:
    deadline = time.monotonic() + max(0.0, float(duration_s))
    latest: dict[str, Any] = dict(state.get("last_nav") or {})
    while time.monotonic() < deadline and sensor.poll() is None:
        observed = _capture_nav_status(
            path=nav_status,
            phase=phase,
            timeline=timeline,
            state=state,
        )
        if observed:
            latest = observed
        time.sleep(0.05)
    return latest


def _wait_for_policy_driving(
    *,
    sensor: Any,
    teleop: Any,
    motion_log: Path,
    timeout_s: float,
) -> tuple[bool, str]:
    """Wait until MuJoCo reports post-arm policy driving."""

    deadline = time.monotonic() + max(0.1, float(timeout_s))
    while time.monotonic() < deadline:
        if sensor.poll() is not None:
            return False, "sensor_runner_exited_before_driving"
        teleop_returncode = teleop.poll()
        if teleop_returncode is not None:
            return False, f"continuous_teleop_exited_before_driving:{teleop_returncode}"
        samples = _read_jsonl(motion_log)
        if any(sample.get("driving") is True for sample in samples[-40:]):
            return True, "driving"
        time.sleep(0.05)
    return False, "policy_driving_start_timeout"


def _wait_for_continuous_teleop_admission(
    *,
    sensor: Any,
    teleop: Any,
    nav_status: Path,
    timeline: list[dict[str, Any]],
    state: dict[str, Any],
    source_id: str,
    timeout_s: float,
) -> tuple[bool, str, dict[str, Any]]:
    """Wait for native status to bind a continuous typed source to admission."""

    evidence: dict[str, Any] = {}
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    while time.monotonic() < deadline:
        if sensor.poll() is not None:
            return False, "sensor_runner_exited_before_teleop_admission", evidence
        teleop_returncode = teleop.poll()
        if teleop_returncode is not None:
            return (
                False,
                f"continuous_teleop_exited_before_admission:{teleop_returncode}",
                evidence,
            )
        nav = _capture_nav_status(
            path=nav_status,
            phase="pre_arm",
            timeline=timeline,
            state=state,
        )
        operator_motion = nav.get("operator_motion")
        operator_motion = operator_motion if isinstance(operator_motion, Mapping) else {}
        status = operator_motion.get("status")
        status = status if isinstance(status, Mapping) else {}
        evidence = {
            "source_id": str(status.get("active_source_id") or ""),
            "source_epoch": int(status.get("active_source_epoch") or 0),
            "has_active_authority": status.get("has_active_authority") is True,
            "admitted_sequence": int(status.get("admitted_sequence") or 0),
            "final_output_sequence": int(status.get("final_output_sequence") or 0),
        }
        if (
            evidence["source_id"] == str(source_id)
            and evidence["source_epoch"] > 0
            and evidence["has_active_authority"] is True
            and evidence["admitted_sequence"] > 0
        ):
            return True, "continuous_teleop_admitted", evidence
        time.sleep(0.05)
    return False, "continuous_teleop_admission_timeout", evidence


def _wait_for_external_arm_ack(
    *,
    sensor: Any,
    teleop: Any,
    status_path: Path,
    token: str,
    domain_id: int,
    scenario: str,
    not_before_ns: int,
    timeout_s: float,
) -> tuple[bool, str, dict[str, Any]]:
    """Wait for the sensor to acknowledge the exact case-scoped arm token."""

    evidence = external_arm_status_evidence(
        status_path,
        token=token,
        domain_id=domain_id,
        scenario=scenario,
        not_before_ns=not_before_ns,
    )
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    while time.monotonic() < deadline:
        if sensor.poll() is not None:
            return False, "sensor_runner_exited_before_external_arm_ack", evidence
        teleop_returncode = teleop.poll()
        if teleop_returncode is not None:
            return (
                False,
                f"continuous_teleop_exited_before_external_arm_ack:{teleop_returncode}",
                evidence,
            )
        evidence = external_arm_status_evidence(
            status_path,
            token=token,
            domain_id=domain_id,
            scenario=scenario,
            not_before_ns=not_before_ns,
        )
        if evidence.get("acknowledged") is True:
            return True, "external_arm_acknowledged", evidence
        status = evidence.get("status")
        status = status if isinstance(status, Mapping) else {}
        if (
            status.get("domain_id") == int(domain_id)
            and str(status.get("scenario") or "") == str(scenario)
            and str(status.get("state") or "") in {"invalid", "timed_out"}
        ):
            return False, "external_arm_sensor_rejected", evidence
        time.sleep(0.05)
    return False, "external_arm_ack_timeout", evidence


def _wait_for_runtime_ready(
    *,
    sensor: Any,
    nav_status: Path,
    slam_status: Path,
    mapd_status: Path,
    traversability_status: Path,
    timeline: list[dict[str, Any]],
    state: dict[str, Any],
    state_provider: str,
    mapd_required: bool,
    mapd_not_before_ns: int,
    evidence_scope: str,
    product_gate_eligible: bool,
    timeout_s: float,
) -> tuple[bool, str, dict[str, Any]]:
    from sim.scripts.mujoco import native_navigation_acceptance as native

    omission_reason = (
        "state_provider_cannot_publish_slam_map_observation"
        if not mapd_required
        else ""
    )
    mapd_evidence = mapd_status_evidence(
        mapd_status,
        required=mapd_required,
        not_before_ns=mapd_not_before_ns,
        evidence_scope=evidence_scope,
        product_gate_eligible=product_gate_eligible,
        omission_reason=omission_reason,
    )
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    while time.monotonic() < deadline:
        if sensor.poll() is not None:
            return False, "sensor_runner_exited_before_runtime_ready", mapd_evidence
        nav = _capture_nav_status(
            path=nav_status,
            phase="warmup",
            timeline=timeline,
            state=state,
        )
        slam = native._load_json(slam_status)
        traversability = native._load_json(traversability_status)
        mapd_evidence = mapd_status_evidence(
            mapd_status,
            required=mapd_required,
            not_before_ns=mapd_not_before_ns,
            evidence_scope=evidence_scope,
            product_gate_eligible=product_gate_eligible,
            omission_reason=omission_reason,
        )
        state_ready = bool(nav.get("has_odom"))
        if state_provider != "mujoco_navigation_fixture":
            state_ready = str(slam.get("state") or "").upper() == "TRACKING"
        if (
            state_ready
            and str(nav.get("control_mode") or "") == "teleop_avoid"
            and bool((nav.get("input_gate") or {}).get("ready"))
            and int((traversability.get("counters") or {}).get("published") or 0) > 0
            and (not mapd_required or mapd_evidence.get("ok") is True)
        ):
            return True, "ready", mapd_evidence
        time.sleep(0.1)
    reason = (
        "mapd_runtime_startup_gate_failed"
        if mapd_required and mapd_evidence.get("ok") is not True
        else "native_runtime_startup_timeout"
    )
    return False, reason, mapd_evidence


def _wait_for_teleop_reason(
    *,
    expected: set[str],
    sensor: Any,
    nav_status: Path,
    phase: str,
    timeline: list[dict[str, Any]],
    state: dict[str, Any],
    timeout_s: float,
) -> tuple[bool, str]:
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    last_reason = ""
    while time.monotonic() < deadline and sensor.poll() is None:
        nav = _capture_nav_status(
            path=nav_status,
            phase=phase,
            timeline=timeline,
            state=state,
        )
        last_reason = str((nav.get("teleop") or {}).get("reason") or last_reason)
        if last_reason in expected:
            return True, last_reason
        time.sleep(0.05)
    return False, last_reason


def _wait_for_input_gate_reason(
    *,
    expected: set[str],
    sensor: Any,
    nav_status: Path,
    phase: str,
    timeline: list[dict[str, Any]],
    state: dict[str, Any],
    timeout_s: float,
) -> tuple[bool, str]:
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    last_reason = ""
    while time.monotonic() < deadline and sensor.poll() is None:
        nav = _capture_nav_status(
            path=nav_status,
            phase=phase,
            timeline=timeline,
            state=state,
        )
        last_reason = str((nav.get("input_gate") or {}).get("reason") or last_reason)
        if last_reason in expected:
            return True, last_reason
        time.sleep(0.05)
    return False, last_reason


def _run_control(
    binary: Path,
    arguments: Sequence[str],
    *,
    domain_id: int,
    timeout_s: float = 10.0,
) -> dict[str, Any]:
    from sim.scripts.mujoco import native_navigation_acceptance as native

    command = native._native_command(
        binary,
        *(str(value) for value in arguments),
        "--domain-id",
        str(domain_id),
    )
    try:
        completed = subprocess.run(
            command,
            cwd=ROOT,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=max(1.0, float(timeout_s)),
            check=False,
        )
        return {
            "command": command,
            "returncode": int(completed.returncode),
            "stdout": (completed.stdout or "")[-4000:],
            "stderr": (completed.stderr or "")[-4000:],
        }
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {
            "command": command,
            "returncode": None,
            "stdout": "",
            "stderr": f"{type(exc).__name__}:{exc}",
        }


def cleanup_owned_pid_file(
    name: str,
    path: Path,
    *,
    pid_wait_s: float = 2.0,
    term_wait_s: float = 2.0,
    kill_wait_s: float = 1.0,
) -> dict[str, Any]:
    """Stop one explicitly owned child even when its pidfile appears late."""

    from sim.scripts.mujoco import native_navigation_acceptance as native

    def pid_alive(pid_value: int | None) -> bool:
        if os.name == "nt":
            return bool(native._wsl_pid_alive(pid_value))
        if pid_value is None or pid_value <= 0:
            return False
        try:
            os.kill(pid_value, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def send_signal(pid_value: int | None, signal_name: str) -> bool:
        if os.name == "nt":
            return bool(native._signal_wsl_pid(pid_value, signal_name))
        if pid_value is None or pid_value <= 0:
            return False
        signum = getattr(signal, f"SIG{signal_name}", None)
        if signum is None:
            return False
        try:
            os.kill(pid_value, signum)
        except (OSError, ProcessLookupError, ValueError):
            return False
        return True

    def wait_for_exit(pid_value: int | None, timeout_s: float) -> bool:
        if os.name == "nt":
            return bool(native._wait_wsl_pid_exit(pid_value, timeout_s))
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() <= deadline:
            if not pid_alive(pid_value):
                return True
            time.sleep(0.05)
        return not pid_alive(pid_value)

    wait_s = max(0.0, float(pid_wait_s))
    pid = native._read_linux_pid(path, timeout_s=wait_s)
    alive_before = pid_alive(pid)
    term_sent = False
    kill_sent = False
    if alive_before:
        term_sent = send_signal(pid, "TERM")
        if not wait_for_exit(pid, max(0.0, float(term_wait_s))):
            kill_sent = send_signal(pid, "KILL")
            wait_for_exit(pid, max(0.0, float(kill_wait_s)))
    alive_after = pid_alive(pid)
    return {
        "name": name,
        "linux_pid": pid,
        "pid_file": str(path),
        "pid_wait_s": wait_s,
        "alive_before_cleanup": alive_before,
        "term_sent": term_sent,
        "kill_sent": kill_sent,
        "alive_after_cleanup": alive_after,
        "clean": not alive_after,
    }


def _inspect_pid_command(pid: int | None) -> dict[str, Any]:
    """Read one PID command line without treating PID reuse as ownership."""

    from sim.scripts.mujoco import native_navigation_acceptance as native

    if pid is None or pid <= 0:
        return {"alive": False, "readable": True, "command": ""}
    if os.name == "nt":
        wsl = shutil.which("wsl.exe") or shutil.which("wsl")
        if not wsl:
            return {"alive": True, "readable": False, "command": ""}
        try:
            completed = subprocess.run(
                [wsl, "-e", "cat", f"/proc/{pid}/cmdline"],
                capture_output=True,
                timeout=5.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return {
                "alive": bool(native._wsl_pid_alive(pid)),
                "readable": False,
                "command": "",
            }
        if completed.returncode != 0:
            alive = bool(native._wsl_pid_alive(pid))
            return {"alive": alive, "readable": not alive, "command": ""}
        command = completed.stdout.replace(b"\0", b" ").decode("utf-8", errors="replace").strip()
        return {"alive": True, "readable": True, "command": command}
    try:
        command = (Path("/proc") / str(pid) / "cmdline").read_bytes()
    except FileNotFoundError:
        return {"alive": False, "readable": True, "command": ""}
    except OSError:
        return {"alive": True, "readable": False, "command": ""}
    return {
        "alive": True,
        "readable": True,
        "command": command.replace(b"\0", b" ").decode("utf-8", errors="replace").strip(),
    }


def reclaim_prior_case_processes(
    case_dir: Path,
    artifacts: Mapping[str, Path],
    expected_commands: Mapping[str, Sequence[str]],
) -> list[dict[str, Any]]:
    """Reclaim pidfile-owned children before replacing their ownership handles."""

    candidates = {
        "prior_slam": case_dir / "slam.pid",
        "prior_mapd": case_dir / "mapd.pid",
        "prior_traversability": case_dir / "traversability.pid",
        "prior_navigation": case_dir / "navigation.pid",
        "prior_teleop_command": case_dir / "teleop_command.pid",
        "prior_teleop_command_current": case_dir / "teleop_command_current.pid",
        "prior_sensor_publisher": Path(artifacts["sensor_publisher_pid"]),
        "prior_cmd_vel_tap": Path(artifacts["cmd_vel_tap_pid"]),
    }
    results: list[dict[str, Any]] = []
    for name, path in candidates.items():
        if not path.is_file():
            continue
        from sim.scripts.mujoco import native_navigation_acceptance as native

        pid = native._read_linux_pid(path, timeout_s=0.1)
        inspection = _inspect_pid_command(pid)
        expected = [str(value) for value in expected_commands.get(name, ())]
        command = str(inspection.get("command") or "")
        ownership_match = bool(expected) and all(token in command for token in expected)
        if inspection.get("alive") is not True:
            path.unlink(missing_ok=True)
            result = {
                "name": name,
                "linux_pid": pid,
                "pid_file": str(path),
                "clean": True,
                "ownership_match": False,
                "action": "stale_dead_handle_removed",
            }
        elif inspection.get("readable") is not True:
            result = {
                "name": name,
                "linux_pid": pid,
                "pid_file": str(path),
                "clean": False,
                "ownership_match": False,
                "action": "ownership_inspection_failed",
            }
        elif not ownership_match:
            path.unlink(missing_ok=True)
            result = {
                "name": name,
                "linux_pid": pid,
                "pid_file": str(path),
                "clean": True,
                "ownership_match": False,
                "action": "stale_reused_pid_handle_removed",
            }
        else:
            result = cleanup_owned_pid_file(name, path, pid_wait_s=0.1)
            result["ownership_match"] = True
            result["action"] = "matched_owned_process_reclaimed"
        result["observed_command"] = command
        result["expected_command_tokens"] = expected
        result["ownership_handle_reclaimed"] = bool(result.get("clean"))
        results.append(result)
    return results


def signal_managed_process(process: Any, signal_name: str) -> bool:
    """Signal a managed native child on WSL or a direct POSIX runtime."""

    from sim.scripts.mujoco import native_navigation_acceptance as native

    linux_pid = getattr(process, "linux_pid", None)
    pid_path = getattr(process, "pid_path", None)
    if linux_pid is None and pid_path is not None:
        linux_pid = native._read_linux_pid(Path(pid_path), timeout_s=0.5)
        if linux_pid is not None:
            process.linux_pid = linux_pid
    if linux_pid is not None:
        return bool(native._signal_wsl_pid(linux_pid, signal_name))

    child = getattr(process, "process", None)
    if child is None or child.poll() is not None:
        return False
    signum = getattr(signal, f"SIG{str(signal_name).upper()}", None)
    if signum is None:
        return False
    try:
        child.send_signal(signum)
    except (OSError, ProcessLookupError, ValueError):
        return False
    return True


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


def _operator_motion_events_from_log(path: Path) -> list[dict[str, Any]]:
    try:
        output = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return []
    return parse_operator_motion_events(output)


def _dedupe_operator_motion_events(
    events: Sequence[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    deduped: dict[tuple[str, int, int, str], dict[str, Any]] = {}
    for event in events:
        source_id = str(event.get("source_id") or "")
        source_epoch = int(event.get("source_epoch") or 0)
        source_sequence = int(event.get("source_sequence") or 0)
        action = str(event.get("action") or "")
        if not source_id or source_epoch <= 0 or source_sequence <= 0 or not action:
            continue
        deduped[(source_id, source_epoch, source_sequence, action)] = dict(event)
    return sorted(
        deduped.values(),
        key=lambda item: (
            str(item.get("source_id") or ""),
            int(item.get("source_epoch") or 0),
            int(item.get("source_sequence") or 0),
            str(item.get("action") or ""),
        ),
    )


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
FREE_COMMAND_ACCEPTED_REASONS = frozenset(
    {"accepted", "teleop_assist_control_ready"}
)


def _cleanup_stop_trigger_sim_s(
    *,
    arm_observed_sim_time_s: float,
    total_duration_s: float,
    stop_margin_s: float,
) -> float:
    """Schedule cleanup before the absolute simulation horizon expires."""

    return max(
        float(arm_observed_sim_time_s) + 0.5,
        float(total_duration_s) - float(stop_margin_s),
    )


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


def _strict_twist_is_zero(value: Mapping[str, Any]) -> bool:
    return all(_finite_number(value.get(axis)) and abs(float(value[axis])) <= 1e-6 for axis in ("vx", "vy", "wz"))


def _native_stop_accepted(stdout: str) -> bool:
    return re.search(r"(?m)^accepted stop:", str(stdout or "")) is not None


def _annotate_native_stop_ack(stop_result: dict[str, Any], *, ack_wall_s: float) -> None:
    acked = stop_result.get("returncode") == 0 and _native_stop_accepted(str(stop_result.get("stdout") or ""))
    stop_result["accepted"] = bool(acked)
    stop_result["acked"] = bool(acked)
    if acked:
        stop_result["ack_wall_s"] = float(ack_wall_s)


def _post_stop_zero_output_evidence(
    timeline: Sequence[Mapping[str, Any]],
    *,
    stop_ack_wall_s: float,
    pre_stop_status_stamp_s: float | None,
    window_start_wall_s: float,
    window_end_wall_s: float,
    required_status_samples: int = POST_STOP_REQUIRED_STATUS_SAMPLES,
) -> dict[str, Any]:
    status_samples: list[Mapping[str, Any]] = []
    invalid_stamp_samples = 0
    for item in timeline:
        if not isinstance(item, Mapping) or not isinstance(item.get("nav"), Mapping):
            continue
        wall_s = item.get("wall_s")
        if not _finite_number(wall_s):
            continue
        wall = float(wall_s)
        if not (wall > float(stop_ack_wall_s) and float(window_start_wall_s) <= wall <= float(window_end_wall_s)):
            continue
        stamp_s = _nav_status_stamp_s(item["nav"])
        if stamp_s is None or (pre_stop_status_stamp_s is not None and stamp_s <= float(pre_stop_status_stamp_s)):
            invalid_stamp_samples += 1
            continue
        status_samples.append(item)
    final_cmd_samples = 0
    nonzero_final_cmd_samples = 0
    missing_final_cmd_samples = 0
    invalid_final_cmd_samples = 0
    for item in status_samples:
        final_cmd = _final_cmd_vel_from_nav_status(item["nav"])
        if final_cmd is None:
            missing_final_cmd_samples += 1
            continue
        if not all(_finite_number(final_cmd.get(axis)) for axis in ("vx", "vy", "wz")):
            invalid_final_cmd_samples += 1
            continue
        final_cmd_samples += 1
        if not _strict_twist_is_zero(final_cmd):
            nonzero_final_cmd_samples += 1
    zero_output_observed = (
        float(window_end_wall_s) > float(window_start_wall_s)
        and pre_stop_status_stamp_s is not None
        and len(status_samples) >= int(required_status_samples)
        and final_cmd_samples == len(status_samples)
        and missing_final_cmd_samples == 0
        and invalid_final_cmd_samples == 0
        and nonzero_final_cmd_samples == 0
    )
    return {
        "zero_output_observed": zero_output_observed,
        "stop_ack_wall_s": float(stop_ack_wall_s),
        "pre_stop_status_stamp_s": pre_stop_status_stamp_s,
        "window_start_wall_s": float(window_start_wall_s),
        "window_end_wall_s": float(window_end_wall_s),
        "required_status_samples": int(required_status_samples),
        "status_samples": len(status_samples),
        "final_cmd_samples": final_cmd_samples,
        "zero_final_cmd_samples": final_cmd_samples - nonzero_final_cmd_samples,
        "nonzero_final_cmd_samples": nonzero_final_cmd_samples,
        "missing_final_cmd_samples": missing_final_cmd_samples,
        "invalid_final_cmd_samples": invalid_final_cmd_samples,
        "invalid_or_stale_status_samples": invalid_stamp_samples,
    }


def _collect_post_stop_zero_output_evidence(
    *,
    sensor: Any,
    nav_status: Path,
    timeline: list[dict[str, Any]],
    state: dict[str, Any],
    stop_ack_wall_s: float,
    pre_stop_status_stamp_s: float | None,
    duration_s: float = 0.35,
) -> dict[str, Any]:
    window_start_wall_s = time.time()
    _collect_for(
        sensor=sensor,
        nav_status=nav_status,
        phase="post_stop",
        timeline=timeline,
        state=state,
        duration_s=duration_s,
    )
    return _post_stop_zero_output_evidence(
        timeline,
        stop_ack_wall_s=stop_ack_wall_s,
        pre_stop_status_stamp_s=pre_stop_status_stamp_s,
        window_start_wall_s=window_start_wall_s,
        window_end_wall_s=time.time(),
    )


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


def evaluate_operator_motion_lifecycle(
    timeline: Sequence[Mapping[str, Any]],
    events: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Bind one typed source epoch to admission, output, and zero-barrier status."""

    grouped: dict[tuple[str, int], list[dict[str, Any]]] = {}
    for raw in events:
        if raw.get("accepted") is not True:
            continue
        source_id = str(raw.get("source_id") or "")
        source_epoch = int(raw.get("source_epoch") or 0)
        if not source_id or source_epoch <= 0:
            continue
        grouped.setdefault((source_id, source_epoch), []).append(dict(raw))
    complete = [
        (identity, values)
        for identity, values in grouped.items()
        if {str(item.get("action") or "") for item in values}
        >= {"claim", "sample", "hold", "release"}
    ]
    if not complete:
        return {
            "ok": False,
            "blockers": [
                "operator_motion_claim_ack_missing",
                "operator_motion_sample_admission_missing",
                "operator_motion_final_output_mapping_missing",
                "operator_motion_hold_zero_barrier_missing",
                "operator_motion_release_zero_barrier_missing",
            ],
            "source_id": "",
            "source_epoch": 0,
            "events": [],
        }
    (source_id, source_epoch), selected_events = max(
        complete,
        key=lambda item: max(int(event.get("sample_count") or 0) for event in item[1]),
    )
    sample_sequences = {
        int(event.get("source_sequence") or 0)
        for event in selected_events
        if event.get("action") == "sample"
    }
    hold_sequence = max(
        int(event.get("source_sequence") or 0)
        for event in selected_events
        if event.get("action") == "hold"
    )
    release_sequence = max(
        int(event.get("source_sequence") or 0)
        for event in selected_events
        if event.get("action") == "release"
    )

    matching_status: list[dict[str, Any]] = []
    for sample in timeline:
        nav = sample.get("nav") or {}
        motion = nav.get("operator_motion") or {}
        if not isinstance(motion, Mapping):
            continue
        matching_status.append(
            {
                "phase": str(sample.get("phase") or ""),
                "last_ack": dict(motion.get("last_ack") or {}),
                "status": dict(motion.get("status") or {}),
            }
        )

    admitted = [
        item
        for item in matching_status
        if str((item["status"]).get("active_source_id") or "") == source_id
        and int((item["status"]).get("active_source_epoch") or 0) == source_epoch
        and int((item["status"]).get("admitted_sequence") or 0) in sample_sequences
    ]
    mapped = [
        item
        for item in admitted
        if int((item["status"]).get("final_output_sequence") or 0) > 0
    ]

    def zero_barrier(action: int, sequence: int, *, holding: bool) -> list[dict[str, Any]]:
        observed: list[dict[str, Any]] = []
        for item in matching_status:
            ack = item["last_ack"]
            status = item["status"]
            if (
                str(ack.get("source_id") or "") != source_id
                or int(ack.get("source_epoch") or 0) != source_epoch
                or int(ack.get("source_sequence") or 0) != sequence
                or int(ack.get("action") or 0) != action
                or ack.get("accepted") is not True
                or int(ack.get("final_output_sequence") or 0) <= 0
                or not _twist_is_zero(status.get("final_cmd_vel") or {})
            ):
                continue
            if holding:
                if status.get("holding") is not True:
                    continue
            elif status.get("has_active_authority") is not False:
                continue
            observed.append(item)
        return observed

    hold_barriers = zero_barrier(3, hold_sequence, holding=True)
    release_barriers = zero_barrier(2, release_sequence, holding=False)
    blockers: list[str] = []
    if not any(event.get("action") == "claim" for event in selected_events):
        blockers.append("operator_motion_claim_ack_missing")
    if not admitted:
        blockers.append("operator_motion_sample_admission_missing")
    if not mapped:
        blockers.append("operator_motion_final_output_mapping_missing")
    if not hold_barriers:
        blockers.append("operator_motion_hold_zero_barrier_missing")
    if not release_barriers:
        blockers.append("operator_motion_release_zero_barrier_missing")
    return {
        "ok": not blockers,
        "blockers": blockers,
        "source_id": source_id,
        "source_epoch": source_epoch,
        "events": selected_events,
        "admitted_status_samples": len(admitted),
        "mapped_final_output_samples": len(mapped),
        "hold_zero_barrier_samples": len(hold_barriers),
        "release_zero_barrier_samples": len(release_barriers),
        "max_admitted_sequence": max(
            (int(item["status"].get("admitted_sequence") or 0) for item in admitted),
            default=0,
        ),
        "max_mapped_final_output_sequence": max(
            (int(item["status"].get("final_output_sequence") or 0) for item in mapped),
            default=0,
        ),
    }


def continuous_teleop_exit_blocker(returncode: int | None) -> str:
    return "" if returncode is None else f"continuous_teleop_exited_early:{int(returncode)}"


def typed_teleop_delivery_blocker(
    delivery: Mapping[str, Any],
    *,
    product_gate_eligible: bool,
) -> str:
    unstable = int(delivery.get("retry_count") or 0) > 0 or bool(delivery.get("failure_reason_counts"))
    return "typed_teleop_delivery_unstable" if product_gate_eligible and unstable else ""


class ResilientTeleopProcess:
    """Restart the typed teleop CLI after transient request/ACK timeouts."""

    name = "teleop_command"

    def __init__(
        self,
        command: Sequence[str],
        log_path: Path,
        *,
        managed_process_factory: Any = None,
        retry_delay_s: float = 0.05,
    ) -> None:
        self.command = list(command)
        self.log_path = Path(log_path)
        self.current_log_path = self.log_path.with_name(f"{self.log_path.stem}_current{self.log_path.suffix}")
        self._factory = managed_process_factory
        self._retry_delay_s = max(0.0, float(retry_delay_s))
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._current: Any = None
        self._lock = threading.Lock()
        self._final_returncode: int | None = None
        self._error = ""
        self.attempts: list[dict[str, Any]] = []
        self.cleanup: dict[str, Any] = {}

    def start(self) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_path.unlink(missing_ok=True)
        self.current_log_path.unlink(missing_ok=True)
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="teleop-avoid-typed-command",
            daemon=False,
        )
        self._thread.start()

    def _new_process(self) -> Any:
        if self._factory is not None:
            return self._factory(
                f"teleop_command_attempt_{len(self.attempts) + 1}",
                list(self.command),
                self.current_log_path,
            )
        from sim.scripts.mujoco import native_navigation_acceptance as native

        return native.ManagedProcess(
            f"teleop_command_attempt_{len(self.attempts) + 1}",
            list(self.command),
            self.current_log_path,
        )

    def _record_attempt(self, process: Any, returncode: int | None) -> None:
        try:
            output = Path(process.log_path).read_text(encoding="utf-8", errors="replace")
        except (AttributeError, OSError, TypeError):
            output = process.tail()
        failure_reason = ""
        rejection_markers = ("operator motion command rejected:", "navigation command rejected:")
        for rejection_marker in rejection_markers:
            if rejection_marker in output:
                failure_reason = output.split(rejection_marker, 1)[1].splitlines()[0].strip()
                break
        if not failure_reason and "dds_wait_for_acks" in output and "Timeout" in output:
            failure_reason = "dds_ack_timeout"
        elif returncode not in (None, 0):
            failure_reason = "typed_teleop_client_failed"
        attempt = {
            "attempt": len(self.attempts) + 1,
            "returncode": returncode,
            "ack_timeout": "dds_wait_for_acks" in output and "Timeout" in output,
            "failure_reason": failure_reason,
            "cleanup": dict(getattr(process, "cleanup", {}) or {}),
            "operator_motion_events": parse_operator_motion_events(output),
            "log_tail": output[-4000:],
        }
        self.attempts.append(attempt)
        with self.log_path.open("a", encoding="utf-8") as stream:
            stream.write(
                f"=== attempt {attempt['attempt']} returncode={returncode} "
                f"ack_timeout={str(attempt['ack_timeout']).lower()} ===\n"
            )
            if output:
                stream.write(output)
                if not output.endswith("\n"):
                    stream.write("\n")

    def _run(self) -> None:
        try:
            while not self._stop_event.is_set():
                process = self._new_process()
                with self._lock:
                    self._current = process
                process.start()
                while process.poll() is None and not self._stop_event.wait(0.05):
                    pass
                returncode = process.poll()
                process.stop()
                self._record_attempt(process, returncode)
                with self._lock:
                    self._current = None
                if self._stop_event.wait(self._retry_delay_s):
                    break
            self._final_returncode = 0 if self._stop_event.is_set() else 1
        except Exception as exc:
            self._error = f"{type(exc).__name__}:{exc}"
            self._final_returncode = 1

    def poll(self) -> int | None:
        if self._thread is not None and self._thread.is_alive():
            return None
        return self._final_returncode

    def request_stop(self) -> None:
        self._stop_event.set()

    def stop(self) -> None:
        self.request_stop()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=5.0)
        if thread is not None and thread.is_alive():
            with self._lock:
                current = self._current
            if current is not None:
                current.stop()
            thread.join(timeout=3.0)
        alive = bool(thread is not None and thread.is_alive())
        attempts_clean = all(bool((attempt.get("cleanup") or {}).get("clean")) for attempt in self.attempts)
        self.cleanup = {
            "name": self.name,
            "linux_pid": None,
            "pid_file": str(self.current_log_path.with_suffix(".pid")),
            "alive_before_cleanup": True,
            "alive_after_cleanup": alive,
            "clean": not alive and attempts_clean,
            "relay_returncode": self._final_returncode,
            "attempts": len(self.attempts),
            "ack_timeouts": sum(bool(attempt.get("ack_timeout")) for attempt in self.attempts),
            "error": self._error,
        }

    def tail(self, limit: int = 5000) -> str:
        try:
            value = self.log_path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return ""
        return value[-limit:]

    def snapshot(self) -> dict[str, Any]:
        reason_counts: dict[str, int] = {}
        for attempt in self.attempts:
            reason = str(attempt.get("failure_reason") or "")
            if reason:
                reason_counts[reason] = reason_counts.get(reason, 0) + 1
        return {
            "attempts": list(self.attempts),
            "attempt_count": len(self.attempts),
            "ack_timeout_count": sum(bool(attempt.get("ack_timeout")) for attempt in self.attempts),
            "retry_count": max(0, len(self.attempts) - 1),
            "failure_reason_counts": reason_counts,
            "error": self._error,
        }


def execute_case(
    *,
    scenario: str,
    domain_id: int,
    prepared: Mapping[str, Any],
    args: argparse.Namespace,
) -> dict[str, Any]:
    """Execute one isolated native endpoint mode with owned-process cleanup."""

    from sim.scripts.mujoco import native_navigation_acceptance as native

    manifest = dict(prepared.get("manifest") or {})
    state_provider = str(
        ((manifest.get("slam_runtime") or {}).get("provider") or "fastlio2")
    ).strip().lower()
    tolerances = dict(manifest.get("runtime_tolerances") or {})
    if getattr(args, "realtime_factor", None) is not None:
        tolerances["sim_hardware_realtime_factor"] = max(0.05, float(args.realtime_factor))
    manifest["runtime_tolerances"] = tolerances
    binaries = {name: Path(path) for name, path in (prepared.get("binaries") or {}).items()}
    paths = {name: Path(path) for name, path in (prepared.get("paths") or {}).items()}
    case_dir = Path(args.artifact_dir).expanduser().resolve() / "cases" / scenario
    odom_prior_diagnostic = bool(getattr(args, "odom_prior_diagnostic", False))
    simulation_fixture = state_provider == "mujoco_navigation_fixture"
    injected_diagnostic = scenario.endswith("_injected")
    mapd_required = not simulation_fixture
    product_gate_eligible = not (
        odom_prior_diagnostic or simulation_fixture or injected_diagnostic
    )
    evidence_scope = (
        "dds_consumer_contract_injected" if injected_diagnostic
        else "simulation_pose_prior_diagnostic" if odom_prior_diagnostic
        else "local_planner_simulation_fixture" if simulation_fixture
        else "product_e2e"
    )
    evidence_class = "product" if product_gate_eligible else "non_product_diagnostic"
    if odom_prior_diagnostic:
        paths["slam_config"] = build_odom_prior_diagnostic_config(
            paths["slam_config"],
            case_dir / "slam_config_odom_prior_diagnostic.yaml",
        )
        sensor_runtime = dict(manifest.get("sensor_runtime") or {})
        sensor_runtime["publish_odom_prior"] = True
        manifest["sensor_runtime"] = sensor_runtime
        manifest["_odom_prior_diagnostic"] = True
    terrain_physical_case = scenario in {"terrain_soft", "terrain_hard"}
    effective_duration_s = (
        max(35.0, float(args.duration_s))
        if terrain_physical_case
        else float(args.duration_s)
    )
    plan = build_execution_plan(
        scenario=scenario,
        domain_id=domain_id,
        binaries=binaries,
        paths=paths,
        case_dir=case_dir,
        duration_s=effective_duration_s,
        warmup_s=float(args.warmup_s),
        command_vx=float(args.command_vx),
        external_arm_timeout_s=max(
            1.0, float(args.startup_timeout_s) + float(args.driving_start_timeout_s) + 30.0
        ),
        manifest=manifest,
    )
    artifacts = {name: Path(value) for name, value in plan["artifacts"].items()}
    arm_contract = dict(plan.get("external_arm") or {})
    domain_tokens = ["--domain-id", str(domain_id)]
    prior_process_cleanup = reclaim_prior_case_processes(
        case_dir,
        artifacts,
        {
            **(
                {"prior_slam": [Path(binaries["slam"]).name, *domain_tokens]}
                if "slam" in binaries else {}
            ),
            "prior_mapd": [
                Path(binaries.get("mapd", Path("mapd"))).name,
                *domain_tokens,
            ],
            "prior_traversability": [
                Path(binaries["traversability"]).name,
                *domain_tokens,
            ],
            "prior_navigation": [Path(binaries["navigation"]).name, *domain_tokens],
            "prior_teleop_command": [
                Path(binaries["navigation_control"]).name,
                "operator-motion",
                *domain_tokens,
            ],
            "prior_teleop_command_current": [
                Path(binaries["navigation_control"]).name,
                "operator-motion",
                *domain_tokens,
            ],
            "prior_sensor_publisher": [
                Path(binaries["sensor_publisher"]).name,
                *domain_tokens,
            ],
            "prior_cmd_vel_tap": [
                Path(binaries["cmd_vel_tap"]).name,
                *domain_tokens,
            ],
        },
    )
    prior_cleanup_ok = all(bool(item.get("clean")) for item in prior_process_cleanup)
    if prior_cleanup_ok:
        reset_case_artifacts(artifacts)
        build_scene_variant(paths["world"], artifacts["scene"], str(plan["scene_variant"]))
    mapd_not_before_ns = time.time_ns()

    processes = [
        native.ManagedProcess(
            str(item["name"]),
            list(item["command"]),
            Path(str(item["log"])),
        )
        for item in plan["processes"]
    ]
    by_name = {process.name: process for process in processes}
    sensor = by_name["sensor"]
    slam = by_name.get("slam")
    traversability = by_name["traversability"]
    teleop: Any = None
    teleop_cleanup: dict[str, Any] | None = None
    teleop_precleanup_returncode: int | None = None
    timeline: list[dict[str, Any]] = []
    timeline_state: dict[str, Any] = {}
    events: list[dict[str, Any]] = []
    started_commands: list[dict[str, Any]] = []
    process_cleanup: list[dict[str, Any]] = []
    unexpected_core_exits: list[dict[str, Any]] = []
    startup_ok = False
    startup_reason = "not_started"
    phase_error = ""
    paused_fault_process: Any = None
    paused_fault_name = ""
    signal_events: list[dict[str, Any]] = []
    fault_log_start_offset: int | None = None
    fault_log_end_offset: int | None = None
    injection_result: dict[str, Any] = {}
    teleop_probe_result: dict[str, Any] = {}
    stop_result: dict[str, Any] = {}
    post_stop_zero_output: dict[str, Any] = {}
    cleanup_stop_sent = False
    observed_reason = ""
    terrain_forward_probe_attribution = terrain_scene_forward_probe_attribution(scenario, {})
    current_phase = "warmup"
    driving_ready = False
    driving_ready_reason = "not_waited"
    teleop_admission_ready = False
    teleop_admission_reason = "not_waited"
    teleop_admission_evidence: dict[str, Any] = {}
    external_arm_trigger: dict[str, Any] = {}
    external_arm_ack_ready = False
    external_arm_ack_reason = "not_waited"
    external_arm_ack_evidence: dict[str, Any] = {}
    mapd_evidence = mapd_status_evidence(
        artifacts["mapd_status"],
        required=mapd_required,
        not_before_ns=mapd_not_before_ns,
        evidence_scope=evidence_scope,
        product_gate_eligible=product_gate_eligible,
        omission_reason=(
            "state_provider_cannot_publish_slam_map_observation"
            if not mapd_required
            else ""
        ),
    )
    realtime_factor = float((manifest.get("runtime_tolerances") or {}).get("sim_hardware_realtime_factor") or 1.0)

    def mark(phase: str) -> None:
        nonlocal current_phase
        current_phase = phase
        wall_s = time.time()
        events.append(
            {
                "phase": phase,
                "wall_s": wall_s,
                "motion_s": _motion_timestamp_for_event(
                    artifacts["motion_log"],
                    event_wall_s=wall_s,
                    realtime_factor=realtime_factor,
                ),
            }
        )

    try:
        if not prior_cleanup_ok:
            raise RuntimeError("prior_owned_process_cleanup_failed")
        for process in processes:
            process.start()
            started_commands.append(
                {
                    "name": process.name,
                    "command": _redact_command_args(process.command),
                }
            )
        startup_ok, startup_reason, mapd_evidence = _wait_for_runtime_ready(
            sensor=sensor,
            nav_status=artifacts["nav_status"],
            slam_status=artifacts["slam_status"],
            mapd_status=artifacts["mapd_status"],
            traversability_status=artifacts["traversability_status"],
            timeline=timeline,
            state=timeline_state,
            state_provider=state_provider,
            mapd_required=mapd_required,
            mapd_not_before_ns=mapd_not_before_ns,
            evidence_scope=evidence_scope,
            product_gate_eligible=product_gate_eligible,
            timeout_s=float(args.startup_timeout_s),
        )
        if startup_ok:
            teleop_probe_result = _run_control(
                binaries["navigation_control"],
                [
                    "operator-motion",
                    str(float(args.command_vx)),
                    "0",
                    "0",
                    "--lease-ttl-ms",
                    "2000",
                    "--cleanup-settle-ms",
                    "300",
                ],
                domain_id=domain_id,
                timeout_s=8.0,
            )
            immediate_probe_events = parse_operator_motion_events(
                str(teleop_probe_result.get("stdout") or "")
            )
            immediate_probe_actions = {
                str(event.get("action") or "") for event in immediate_probe_events
            }
            if (
                teleop_probe_result.get("returncode") != 0
                or immediate_probe_actions < {"claim", "sample", "hold", "release"}
            ):
                raise RuntimeError("typed_operator_motion_probe_lifecycle_failed")
            teleop = ResilientTeleopProcess(
                list(plan["teleop_command"]),
                artifacts["teleop_log"],
            )
            teleop.start()
            started_commands.append(
                {
                    "name": teleop.name,
                    "command": _redact_command_args(teleop.command),
                }
            )
            arm_phase_timeout_s = max(
                2.0, float(args.driving_start_timeout_s) / 3.0
            )
            (
                teleop_admission_ready,
                teleop_admission_reason,
                teleop_admission_evidence,
            ) = _wait_for_continuous_teleop_admission(
                sensor=sensor,
                teleop=teleop,
                nav_status=artifacts["nav_status"],
                timeline=timeline,
                state=timeline_state,
                source_id=f"mujoco-teleop-avoid-{domain_id}",
                timeout_s=arm_phase_timeout_s,
            )
            if not teleop_admission_ready:
                raise RuntimeError(teleop_admission_reason)
            external_arm_trigger = trigger_external_arm(
                artifacts["sensor_arm"],
                token=str(arm_contract.get("token") or ""),
                domain_id=domain_id,
                scenario=scenario,
            )
            mark("arm")
            external_arm_trigger["trigger_motion_s"] = events[-1].get("motion_s")
            (
                external_arm_ack_ready,
                external_arm_ack_reason,
                external_arm_ack_evidence,
            ) = _wait_for_external_arm_ack(
                sensor=sensor,
                teleop=teleop,
                status_path=artifacts["sensor_arm_status"],
                token=str(arm_contract.get("token") or ""),
                domain_id=domain_id,
                scenario=scenario,
                not_before_ns=int(external_arm_trigger["not_before_ns"]),
                timeout_s=arm_phase_timeout_s,
            )
            if not external_arm_ack_ready:
                raise RuntimeError(external_arm_ack_reason)
            driving_ready, driving_ready_reason = _wait_for_policy_driving(
                sensor=sensor,
                teleop=teleop,
                motion_log=artifacts["motion_log"],
                timeout_s=arm_phase_timeout_s,
            )
            if not driving_ready:
                raise RuntimeError(driving_ready_reason)

            if scenario in {
                "traversability_dropout_recovery",
                "slam_inputs_dropout_recovery",
            }:
                fault_name = "traversability" if scenario == "traversability_dropout_recovery" else "slam"
                if fault_name == "slam" and slam is None:
                    raise RuntimeError("slam_inputs_dropout_requires_fastlio2")
                fault_process = traversability if fault_name == "traversability" else slam
                expected_stale = {"traversability_stale"} if fault_name == "traversability" else {"odom_stale"}
                stale_timeout_s = (
                    TRAVERSABILITY_MAX_AGE_S + 1.0
                    if fault_name == "traversability"
                    else float(FIELD_TELEOP_AVOID_PROFILE["localization_health_max_age_s"]) + 1.0
                )
                mark("baseline")
                _collect_for(
                    sensor=sensor,
                    nav_status=artifacts["nav_status"],
                    phase="baseline",
                    timeline=timeline,
                    state=timeline_state,
                    duration_s=1.0,
                )
                if scenario == "traversability_dropout_recovery":
                    fault_log_start_offset = _file_size_or_zero(case_dir / "navigation.log")
                pause_ok = signal_managed_process(fault_process, "STOP")
                if pause_ok:
                    paused_fault_process = fault_process
                    paused_fault_name = fault_name
                signal_events.append(
                    {
                        "process": fault_name,
                        "signal": "STOP",
                        "ok": pause_ok,
                        "wall_s": time.time(),
                    }
                )
                mark("dropout_grace")
                stale_found, observed_reason = _wait_for_input_gate_reason(
                    expected=expected_stale,
                    sensor=sensor,
                    nav_status=artifacts["nav_status"],
                    phase="dropout_grace",
                    timeline=timeline,
                    state=timeline_state,
                    timeout_s=stale_timeout_s,
                )
                mark("dropout")
                _collect_for(
                    sensor=sensor,
                    nav_status=artifacts["nav_status"],
                    phase="dropout",
                    timeline=timeline,
                    state=timeline_state,
                    duration_s=1.0 if stale_found else 0.5,
                )
                resume_ok = signal_managed_process(fault_process, "CONT")
                if resume_ok:
                    paused_fault_process = None
                    paused_fault_name = ""
                signal_events.append(
                    {
                        "process": fault_name,
                        "signal": "CONT",
                        "ok": resume_ok,
                        "wall_s": time.time(),
                    }
                )
                mark("recovery")
                _collect_for(
                    sensor=sensor,
                    nav_status=artifacts["nav_status"],
                    phase="recovery",
                    timeline=timeline,
                    state=timeline_state,
                    duration_s=2.0,
                )
                if scenario == "traversability_dropout_recovery":
                    fault_log_end_offset = _file_size_or_zero(case_dir / "navigation.log")
            elif scenario.endswith("_injected"):
                mark("approach")
                _collect_for(
                    sensor=sensor,
                    nav_status=artifacts["nav_status"],
                    phase="approach",
                    timeline=timeline,
                    state=timeline_state,
                    duration_s=0.5,
                )
                pause_ok = signal_managed_process(traversability, "STOP")
                if pause_ok:
                    paused_fault_process = traversability
                    paused_fault_name = "traversability"
                signal_events.append(
                    {
                        "process": "traversability",
                        "signal": "STOP",
                        "ok": pause_ok,
                        "wall_s": time.time(),
                    }
                )
                cost = "50" if scenario == "terrain_soft_injected" else "100"
                injection_result = _run_control(
                    binaries["navigation_control"],
                    ["trav", cost],
                    domain_id=domain_id,
                    timeout_s=12.0,
                )
                expected = {"terrain_slow", "obstacle_terrain_slow"} if cost == "50" else {"terrain_stop"}
                found, observed_reason = _wait_for_teleop_reason(
                    expected=expected,
                    sensor=sensor,
                    nav_status=artifacts["nav_status"],
                    phase="approach",
                    timeline=timeline,
                    state=timeline_state,
                    timeout_s=0.8,
                )
                if found:
                    mark("steady")
                    _collect_for(
                        sensor=sensor,
                        nav_status=artifacts["nav_status"],
                        phase="steady",
                        timeline=timeline,
                        state=timeline_state,
                        duration_s=0.6,
                    )
                mark("post_case")
                resume_ok = signal_managed_process(traversability, "CONT")
                if resume_ok:
                    paused_fault_process = None
                    paused_fault_name = ""
                signal_events.append(
                    {
                        "process": "traversability",
                        "signal": "CONT",
                        "ok": resume_ok,
                        "wall_s": time.time(),
                    }
                )
            elif scenario == "moving_person_clear":
                mark("steady")
                dynamic_horizon_s = (
                    float(DYNAMIC_PERSON_CONTRACT["motion_start_s"])
                    + float(DYNAMIC_PERSON_CONTRACT["motion_duration_s"])
                    + float(DYNAMIC_PERSON_CONTRACT["clear_grace_s"])
                    + 1.0
                )
                _collect_for(
                    sensor=sensor,
                    nav_status=artifacts["nav_status"],
                    phase="steady",
                    timeline=timeline,
                    state=timeline_state,
                    duration_s=dynamic_horizon_s,
                )
                mark("post_case")
            elif scenario == "free":
                mark("steady")
                _collect_for(
                    sensor=sensor,
                    nav_status=artifacts["nav_status"],
                    phase="steady",
                    timeline=timeline,
                    state=timeline_state,
                    duration_s=3.0,
                )
                mark("post_case")
            else:
                mark("approach")
                expected_by_case = {
                    "obstacle_slow": {"obstacle_slow", "obstacle_terrain_slow"},
                    "obstacle_stop": {"obstacle_stop"},
                    "terrain_soft": {"terrain_slow"},
                    "terrain_hard": {"terrain_stop"},
                }
                found, observed_reason = _wait_for_teleop_reason(
                    expected=expected_by_case.get(scenario, set()),
                    sensor=sensor,
                    nav_status=artifacts["nav_status"],
                    phase="approach",
                    timeline=timeline,
                    state=timeline_state,
                    timeout_s=max(
                        2.0,
                        (4.0 if simulation_fixture and terrain_physical_case else 1.0)
                        * (effective_duration_s - 6.0),
                    ),
                )
                if found:
                    mark("steady")
                    if terrain_physical_case:
                        terrain_forward_probe_attribution = _wait_for_terrain_scene_forward_probe(
                            scenario=scenario,
                            traversability_status=artifacts["traversability_status"],
                            timeout_s=2.0,
                        )
                    _collect_for(
                        sensor=sensor,
                        nav_status=artifacts["nav_status"],
                        phase="steady",
                        timeline=timeline,
                        state=timeline_state,
                        duration_s=2.0 if scenario == "obstacle_slow" else 1.0,
                    )
                    mark("post_case")

            # The fixture can run below real time on CPU-constrained hosts.
            # Scale the deadline from the declared real-time factor. The
            # simulation fixture also keeps a conservative 4x host allowance;
            # product runs do not inherit that slow-host floor.
            simulated_horizon_s = effective_duration_s
            declared_factor = max(0.05, realtime_factor)
            wall_time_scale = max(
                4.0 if simulation_fixture else 1.0,
                1.0 / declared_factor,
            )
            case_timeout_s = max(45.0, wall_time_scale * (simulated_horizon_s + 5.0) + 15.0)
            deadline = time.monotonic() + case_timeout_s
            while sensor.poll() is None and time.monotonic() < deadline:
                _capture_nav_status(
                    path=artifacts["nav_status"],
                    phase=current_phase,
                    timeline=timeline,
                    state=timeline_state,
                )
                motion_samples = _read_jsonl(artifacts["motion_log"])
                if motion_samples and not cleanup_stop_sent:
                    arm_status = external_arm_ack_evidence.get("status")
                    arm_status = arm_status if isinstance(arm_status, Mapping) else {}
                    arm_observed_sim_time_s = float(
                        arm_status.get("arm_observed_sim_time_s") or 0.0
                    )
                    latest_sim_s = float(motion_samples[-1].get("sim_time_s") or 0.0)
                    stop_margin_sim_s = max(3.0, 5.0 * realtime_factor)
                    stop_trigger_sim_s = _cleanup_stop_trigger_sim_s(
                        arm_observed_sim_time_s=arm_observed_sim_time_s,
                        total_duration_s=effective_duration_s,
                        stop_margin_s=stop_margin_sim_s,
                    )
                    if (
                        startup_ok
                        and driving_ready
                        and latest_sim_s >= stop_trigger_sim_s
                        and by_name["navigation"].poll() is None
                    ):
                        mark("cleanup")
                        if teleop is not None and teleop_cleanup is None:
                            teleop_precleanup_returncode = teleop.poll()
                            teleop.request_stop()
                            _collect_for(
                                sensor=sensor,
                                nav_status=artifacts["nav_status"],
                                phase="cleanup",
                                timeline=timeline,
                                state=timeline_state,
                                duration_s=1.2,
                            )
                            teleop.stop()
                            teleop_cleanup = dict(teleop.cleanup)
                            process_cleanup.append(teleop_cleanup)
                            _capture_nav_status(
                                path=artifacts["nav_status"],
                                phase="cleanup",
                                timeline=timeline,
                                state=timeline_state,
                            )
                        pre_stop_status_stamp_s = _nav_status_stamp_s(
                            timeline_state.get("last_nav") or {}
                        )
                        stop_result = _run_control(
                            binaries["navigation_control"],
                            ["stop", "teleop_avoid_acceptance_cleanup"],
                            domain_id=domain_id,
                            timeout_s=8.0,
                        )
                        stop_completed_wall_s = time.time()
                        _annotate_native_stop_ack(
                            stop_result,
                            ack_wall_s=stop_completed_wall_s,
                        )
                        if stop_result.get("acked") is True:
                            post_stop_zero_output = _collect_post_stop_zero_output_evidence(
                                sensor=sensor,
                                nav_status=artifacts["nav_status"],
                                timeline=timeline,
                                state=timeline_state,
                                stop_ack_wall_s=stop_completed_wall_s,
                                pre_stop_status_stamp_s=pre_stop_status_stamp_s,
                            )
                        cleanup_stop_sent = True
                time.sleep(0.05)
            if sensor.poll() is None:
                raise TimeoutError("MuJoCo sensor/policy runner exceeded case deadline")
    except Exception as exc:
        phase_error = f"{type(exc).__name__}:{exc}"
    finally:
        started_names = {item["name"] for item in started_commands}
        for name in ("slam", "mapd", "traversability", "navigation"):
            if name not in started_names:
                continue
            returncode = by_name[name].poll()
            if returncode is not None:
                unexpected_core_exits.append({"name": name, "returncode": int(returncode)})
        if paused_fault_process is not None:
            resumed = signal_managed_process(paused_fault_process, "CONT")
            signal_events.append(
                {
                    "process": paused_fault_name,
                    "signal": "CONT_CLEANUP",
                    "ok": resumed,
                    "wall_s": time.time(),
                }
            )
            paused_fault_process = None
            paused_fault_name = ""
        if teleop is not None and teleop_cleanup is None:
            teleop_precleanup_returncode = teleop.poll()
            teleop.request_stop()
            _collect_for(
                sensor=sensor,
                nav_status=artifacts["nav_status"],
                phase="cleanup",
                timeline=timeline,
                state=timeline_state,
                duration_s=1.2,
            )
            teleop.stop()
            teleop_cleanup = dict(teleop.cleanup)
            process_cleanup.append(teleop_cleanup)
            _capture_nav_status(
                path=artifacts["nav_status"],
                phase="cleanup",
                timeline=timeline,
                state=timeline_state,
            )
        if not stop_result and by_name["navigation"].poll() is None:
            pre_stop_status_stamp_s = _nav_status_stamp_s(
                timeline_state.get("last_nav") or {}
            )
            stop_result = _run_control(
                binaries["navigation_control"],
                ["stop", "teleop_avoid_acceptance_cleanup"],
                domain_id=domain_id,
                timeout_s=8.0,
            )
            stop_completed_wall_s = time.time()
            _annotate_native_stop_ack(
                stop_result,
                ack_wall_s=stop_completed_wall_s,
            )
            if stop_result.get("acked") is True:
                post_stop_zero_output = _collect_post_stop_zero_output_evidence(
                    sensor=sensor,
                    nav_status=artifacts["nav_status"],
                    timeline=timeline,
                    state=timeline_state,
                    stop_ack_wall_s=stop_completed_wall_s,
                    pre_stop_status_stamp_s=pre_stop_status_stamp_s,
                )
        for process in reversed(processes):
            returncode = process.poll()
            if (
                process.name in {"slam", "mapd", "traversability", "navigation"}
                and returncode is not None
                and not any(item["name"] == process.name for item in unexpected_core_exits)
            ):
                unexpected_core_exits.append({"name": process.name, "returncode": int(returncode)})
            process.stop()
            process_cleanup.append(dict(process.cleanup))
        process_cleanup.append(
            cleanup_owned_pid_file(
                "sensor_publisher",
                artifacts["sensor_publisher_pid"],
            )
        )
        process_cleanup.append(
            cleanup_owned_pid_file(
                "cmd_vel_tap",
                artifacts["cmd_vel_tap_pid"],
            )
        )

    _write_jsonl(artifacts["nav_timeline"], timeline)
    sensor_report = native._load_json(artifacts["sensor_report"])
    dynamic_obstacle_residual = (
        evaluate_dynamic_obstacle_residual(
            _read_jsonl(artifacts["map_scene_roi"]),
            dict(sensor_report.get("mocap_motion") or {}),
        )
        if scenario == "moving_person_clear"
        else {
            "evaluated": False,
            "ok": None,
            "reason": "scenario_does_not_request_dynamic_obstacle_residual",
        }
    )
    raw_motion = _read_jsonl(artifacts["motion_log"])
    motion_samples = assign_motion_phases(raw_motion, events)
    evaluation_name = scenario.removesuffix("_injected")
    evaluation = evaluate_case(
        evaluation_name,
        nav_samples=timeline,
        motion_samples=motion_samples,
        command_vx=float(args.command_vx),
        injected=scenario.endswith("_injected"),
    )
    evaluation_scope = str(evaluation.get("evidence_scope") or "")
    simulation_producer_e2e = bool(evaluation.get("producer_e2e"))
    if odom_prior_diagnostic:
        evaluation["evidence_scope"] = evidence_scope
    elif simulation_fixture and evaluation_scope == "terrain_producer_e2e":
        evaluation["evidence_scope"] = "terrain_producer_simulation_fixture"
    elif simulation_fixture and evaluation_scope == "product_e2e":
        evaluation["evidence_scope"] = evidence_scope
    evaluation["product_gate_eligible"] = product_gate_eligible
    if simulation_fixture:
        evaluation["simulation_producer_e2e"] = simulation_producer_e2e
        evaluation["producer_e2e"] = False
        evaluation["driver_ack_evidence_scope"] = "simulated_typed_dds_consumer"
    if scenario == "traversability_dropout_recovery":
        log_window_complete = (
            fault_log_start_offset is not None
            and fault_log_end_offset is not None
            and fault_log_end_offset >= fault_log_start_offset
        )
        gate_transitions = read_native_gate_transitions(
            case_dir / "navigation.log",
            start_offset=fault_log_start_offset or 0,
            end_offset=fault_log_end_offset if log_window_complete else 0,
        )
        native_hysteresis_proven = contains_ordered_transition(
            gate_transitions,
            ("traversability_stale", "recovering", "ready"),
        ) if log_window_complete else False
        metrics = evaluation.setdefault("metrics", {})
        metrics["native_gate_transitions"] = gate_transitions
        metrics["native_gate_log_window"] = {
            "start_offset": fault_log_start_offset,
            "end_offset": fault_log_end_offset,
            "complete": log_window_complete,
        }
        metrics["native_gate_hysteresis_proven"] = native_hysteresis_proven
        if native_hysteresis_proven:
            evaluation["blockers"] = [
                blocker
                for blocker in evaluation.get("blockers") or []
                if blocker != "dropout_recovery_hysteresis_missing"
            ]
            metrics["input_gate_generation_recovery_proven"] = True
            metrics["input_gate_recovery_evidence"] = "fault_scoped_native_tick_log"
            evaluation["ok"] = not evaluation["blockers"]
    last_traversability = native._load_json(artifacts["traversability_status"])
    terrain_risk = dict(last_traversability.get("terrain_risk") or {})
    teleop_delivery = teleop.snapshot() if teleop is not None else {}
    probe_operator_motion_events = parse_operator_motion_events(
        str(teleop_probe_result.get("stdout") or "")
    )
    continuous_operator_motion_events = [
        dict(event)
        for attempt in teleop_delivery.get("attempts") or []
        for event in attempt.get("operator_motion_events") or []
    ]
    continuous_operator_motion_events = _dedupe_operator_motion_events(
        [
            *continuous_operator_motion_events,
            *(
                _operator_motion_events_from_log(teleop.log_path)
                if teleop is not None
                else []
            ),
        ]
    )
    operator_motion_evidence = evaluate_operator_motion_lifecycle(
        timeline,
        continuous_operator_motion_events,
    )
    blockers = list(evaluation.get("blockers") or [])
    if not prior_cleanup_ok:
        blockers.append("prior_owned_process_cleanup_failed")
    if not startup_ok:
        blockers.append(startup_reason)
    if mapd_required and mapd_evidence.get("ok") is not True:
        blockers.append("mapd_runtime_evidence_missing_or_degraded")
    if product_gate_eligible and mapd_evidence.get("product_evidence") is not True:
        blockers.append("mapd_product_evidence_missing_or_degraded")
    if (
        scenario == "moving_person_clear"
        and dynamic_obstacle_residual.get("ok") is not True
    ):
        blockers.extend(
            str(value)
            for value in dynamic_obstacle_residual.get("blockers") or ()
        )
    if phase_error:
        blockers.append("case_runtime_error")
    if (
        startup_ok
        and not teleop_admission_ready
        and teleop_admission_reason != "not_waited"
    ):
        blockers.append(teleop_admission_reason)
    if startup_ok and teleop_admission_ready and not external_arm_trigger:
        blockers.append("external_arm_not_triggered")
    if external_arm_trigger and not external_arm_ack_ready:
        blockers.append(external_arm_ack_reason)
    if external_arm_ack_ready and not driving_ready:
        blockers.append(driving_ready_reason)
    if sensor_report.get("ok") is not True:
        blockers.append("sensor_or_slam_acceptance_failed")
    if sensor_report.get("policy_loaded") is not True:
        blockers.append("thunderv4_policy_not_loaded")
    if sensor_report.get("command_source") != "dds":
        blockers.append("mujoco_command_source_not_dds")
    if int((sensor_report.get("cmd_vel") or {}).get("samples") or 0) <= 0:
        blockers.append("typed_dds_cmd_vel_tap_empty")
    probe_actions = {str(event.get("action") or "") for event in probe_operator_motion_events}
    if startup_ok and (
        teleop_probe_result.get("returncode") != 0
        or probe_actions < {"claim", "sample", "hold", "release"}
    ):
        blockers.append("typed_operator_motion_probe_lifecycle_failed")
    if startup_ok:
        blockers.extend(str(value) for value in operator_motion_evidence.get("blockers") or [])
    continuous_exit = continuous_teleop_exit_blocker(teleop_precleanup_returncode)
    if continuous_exit:
        blockers.append(continuous_exit)
    delivery_blocker = typed_teleop_delivery_blocker(
        teleop_delivery,
        product_gate_eligible=product_gate_eligible,
    )
    if delivery_blocker:
        blockers.append(delivery_blocker)
    if stop_result and stop_result.get("returncode") != 0:
        blockers.append("native_cleanup_stop_failed")
    if product_gate_eligible and stop_result.get("acked") is not True:
        blockers.append("native_cleanup_stop_ack_missing")
    if product_gate_eligible and post_stop_zero_output.get("zero_output_observed") is not True:
        blockers.append("native_cleanup_post_stop_zero_unproven")
    if injection_result and injection_result.get("returncode") != 0:
        blockers.append("traversability_injection_failed")
    if not all(bool(item.get("clean")) for item in process_cleanup):
        blockers.append("acceptance_process_cleanup_failed")
    if unexpected_core_exits:
        blockers.extend(
            f"core_process_exited_early:{item['name']}:{item['returncode']}" for item in unexpected_core_exits
        )
    if any(
        event.get("signal") in {"STOP", "CONT", "CONT_CLEANUP"} and event.get("ok") is not True
        for event in signal_events
    ):
        blockers.append("input_fault_signal_failed")
    if startup_ok and any(not isinstance(event.get("motion_s"), (int, float)) for event in events):
        blockers.append("motion_phase_clock_unresolved")
    if scenario in {"terrain_soft", "terrain_hard"}:
        if (
            str(terrain_risk.get("source") or "") != TERRAIN_PRODUCER_CONTRACT["source"]
            or int(terrain_risk.get("cells") or 0) <= 0
        ):
            blockers.append("terrain_producer_risk_observation_missing")
        if terrain_forward_probe_attribution.get("ok") is not True:
            blockers.append("terrain_scene_forward_probe_attribution_missing")
    blockers = list(dict.fromkeys(str(value) for value in blockers))
    report = {
        "schema_version": "lingtu.mujoco.teleop_avoid_native_case.v1",
        "scenario": scenario,
        "ok": not blockers,
        "evidence_scope": evidence_scope,
        "evidence_class": evidence_class,
        "product_gate_eligible": product_gate_eligible,
        "driver_ack_evidence_scope": "simulated_typed_dds_consumer",
        "mapd_evidence": mapd_evidence,
        "dynamic_obstacle_residual": dynamic_obstacle_residual,
        "odom_prior_diagnostic": {
            "enabled": odom_prior_diagnostic,
            "derived_slam_config": (str(paths["slam_config"]) if odom_prior_diagnostic else ""),
            "publishes_simulation_pose_prior": odom_prior_diagnostic,
            "allow_kinematic_fastlio_acceptance": odom_prior_diagnostic,
        },
        "control_ingress_coverage": {
            "tested": "typed operator-motion claim/sample/hold/release with native status correlation",
            "not_covered": [
                "Gateway WebSocket teleop ingress",
                "Python in-process velocity arbitration (mutually exclusive with native endpoint modes)",
            ],
        },
        "input_fault_coverage": {
            "method": (
                "SIGSTOP/SIGCONT process-level correlated dropout"
                if scenario
                in {
                    "traversability_dropout_recovery",
                    "slam_inputs_dropout_recovery",
                }
                else "not exercised"
            ),
            "paused_process": (
                "traversability"
                if scenario == "traversability_dropout_recovery"
                else "slam"
                if scenario == "slam_inputs_dropout_recovery"
                else ""
            ),
            "covered_inputs": (
                ["rt/nav/traversability"]
                if scenario == "traversability_dropout_recovery"
                else [
                    "rt/slam/odometry",
                    "rt/tf",
                    "rt/slam/registered_cloud",
                    "rt/slam/localization_health",
                ]
                if scenario == "slam_inputs_dropout_recovery"
                else []
            ),
            "independent_per_topic_dropout": False,
            "coverage_boundary": (
                (
                    "The SLAM fault is a correlated producer outage; odometry, TF, "
                    "registered-cloud, and localization-health are not paused one-by-one."
                )
                if scenario == "slam_inputs_dropout_recovery"
                else ("The traversability producer is paused as a whole; no DDS topic is faulted independently.")
                if scenario == "traversability_dropout_recovery"
                else "No input-dropout fault is exercised in this scenario."
            ),
        },
        "domain_id": domain_id,
        "startup": {
            "ok": startup_ok,
            "reason": startup_reason,
            "mapd_gate_required": mapd_required,
            "mapd_gate_ok": (
                mapd_evidence.get("ok") is True if mapd_required else None
            ),
        },
        "external_arm": {
            "required": arm_contract.get("required") is True,
            "contract": {
                key: value
                for key, value in arm_contract.items()
                if key != "token"
            },
            "teleop_admission": {
                "ok": teleop_admission_ready,
                "reason": teleop_admission_reason,
                "evidence": teleop_admission_evidence,
            },
            "trigger": external_arm_trigger,
            "ack": {
                "ok": external_arm_ack_ready,
                "reason": external_arm_ack_reason,
                "evidence": external_arm_ack_evidence,
            },
            "sensor": dict(sensor_report.get("external_arm") or {}),
        },
        "policy_driving_start": {
            "ok": driving_ready,
            "reason": driving_ready_reason,
        },
        "failure": dict(evaluation.get("failure") or {}),
        "simulation_posture": dict(evaluation.get("simulation_posture") or {}),
        "evaluation": evaluation,
        "functional_scope": dict(plan["functional_scope"]),
        "terrain_producer_contract": dict(plan["terrain_producer_contract"]),
        "sim_hardware_realtime_factor": realtime_factor,
        "runtime_profile": {
            "name": "real_teleop_avoid",
            "source": "scripts/deploy/thunder/lingtu-{nav,traversability}-dds.service",
            "parameters": dict(FIELD_TELEOP_AVOID_PROFILE),
        },
        "slam_runtime": {
            "mode": str((manifest.get("slam_runtime") or {}).get("mode") or "mapping"),
            "requires_map": False,
        },
        "terrain_producer_observation": terrain_risk,
        "observed_reason": observed_reason,
        "terrain_scene_forward_probe_attribution": terrain_forward_probe_attribution,
        "events": events,
        "signal_events": signal_events,
        "injection": {
            "used": scenario.endswith("_injected"),
            "scope": "dds_consumer_contract_injected" if scenario.endswith("_injected") else "not_used",
            "result": injection_result,
        },
        "native_stop": stop_result,
        "post_stop_zero_output": post_stop_zero_output,
        "typed_teleop": {
            "probe": teleop_probe_result,
            "probe_lifecycle": probe_operator_motion_events,
            "native_status_correlation": operator_motion_evidence,
            "continuous_precleanup_returncode": teleop_precleanup_returncode,
            "continuous_cleanup": teleop_cleanup,
            "continuous_retries": (teleop_delivery),
            "log": str(teleop.log_path) if teleop is not None else "",
        },
        "sensor_report": sensor_report,
        "nav_samples": len(timeline),
        "motion_samples": len(motion_samples),
        "processes": started_commands,
        "process_cleanup": {
            "zero_leftovers": prior_cleanup_ok and all(bool(item.get("clean")) for item in process_cleanup),
            "processes": process_cleanup,
            "prior_owned_processes": prior_process_cleanup,
        },
        "unexpected_core_exits": unexpected_core_exits,
        "artifacts": {name: str(path) for name, path in artifacts.items()},
        "logs": {
            **{process.name: str(process.log_path) for process in processes},
            **({teleop.name: str(teleop.log_path)} if teleop is not None else {}),
        },
        "log_tails": {
            **{process.name: process.tail() for process in processes},
            **({teleop.name: teleop.tail()} if teleop is not None else {}),
        },
        "blockers": blockers,
        "error": phase_error,
    }
    _write_json(case_dir / "report.json", report)
    return report


def run(
    args: argparse.Namespace,
    *,
    prepare_runtime_fn: Any = None,
    execute_case_fn: Any = None,
) -> dict[str, Any]:
    prepare = prepare_runtime_fn or prepare_runtime
    prepared = prepare(args)
    prepared_manifest = dict(prepared.get("manifest") or {})
    state_provider = str(
        ((prepared_manifest.get("slam_runtime") or {}).get("provider") or "fastlio2")
    ).strip().lower()
    odom_prior_diagnostic = bool(getattr(args, "odom_prior_diagnostic", False))
    simulation_fixture = state_provider == "mujoco_navigation_fixture"
    scenarios = _requested_scenarios(getattr(args, "scenario", None))
    injected_scenarios = [scenario for scenario in scenarios if scenario.endswith("_injected")]
    product_gate_eligible = not (
        odom_prior_diagnostic or simulation_fixture or injected_scenarios
    )
    if simulation_fixture:
        evidence_scope = "local_planner_simulation_fixture"
    elif odom_prior_diagnostic:
        evidence_scope = "simulation_pose_prior_diagnostic"
    elif injected_scenarios and len(injected_scenarios) == len(scenarios):
        evidence_scope = "dds_consumer_contract_injected"
    elif injected_scenarios:
        evidence_scope = "mixed_product_and_non_product_evidence"
    else:
        evidence_scope = "product_e2e"
    evidence_class = "product" if product_gate_eligible else "non_product_diagnostic"
    preflight = dict(prepared.get("details") or {})
    preflight["ok"] = bool(prepared.get("ok"))
    preflight["blockers"] = list(prepared.get("blockers") or [])
    cases: list[dict[str, Any]] = []
    blockers = list(preflight["blockers"])

    if not bool(args.preflight_only) and preflight["ok"]:
        if execute_case_fn is None:
            execute_case_fn = execute_case
        domain_base = int(args.domain_base)
        if domain_base < 0 or domain_base + len(scenarios) - 1 > 232:
            blockers.append("cyclonedds_domain_out_of_port_range")
        else:
            for offset, scenario in enumerate(scenarios):
                case = execute_case_fn(
                    scenario=scenario,
                    domain_id=domain_base + offset,
                    prepared=prepared,
                    args=args,
                )
                cases.append(case)
                blockers.extend(str(value) for value in case.get("blockers") or [])
                if case.get("product_gate_eligible") is True:
                    case_mapd = case.get("mapd_evidence")
                    if (
                        not isinstance(case_mapd, Mapping)
                        or case_mapd.get("product_evidence") is not True
                    ):
                        blockers.append(
                            f"mapd_product_evidence_missing_or_degraded:{scenario}"
                        )

    blockers = list(dict.fromkeys(blockers))
    failure_reason = next(
        (
            str((case.get("failure") or {}).get("reason") or "")
            for case in cases
            if str((case.get("failure") or {}).get("reason") or "")
        ),
        "",
    )
    all_requested_cases_executed = (
        len(cases) == len(scenarios)
        and all(
            str(case.get("scenario") or "") == scenario
            for case, scenario in zip(cases, scenarios, strict=True)
        )
    )
    acceptance_evaluated = (
        not bool(args.preflight_only)
        and preflight["ok"]
        and all_requested_cases_executed
    )
    report_ok = (
        preflight["ok"]
        and not blockers
        and all(case.get("ok") is True for case in cases)
    )
    report = {
        "schema_version": SCHEMA_VERSION,
        "ok": report_ok,
        "mode": "teleop_avoid",
        "evidence_scope": evidence_scope,
        "evidence_class": evidence_class,
        "product_gate_eligible": product_gate_eligible,
        "acceptance_evaluated": acceptance_evaluated,
        "product_acceptance_passed": (
            acceptance_evaluated and product_gate_eligible and report_ok
        ),
        "requested_case_count": len(scenarios),
        "executed_case_count": len(cases),
        "all_requested_cases_executed": all_requested_cases_executed,
        "driver_ack_evidence_scope": "simulated_typed_dds_consumer",
        "control_ingress_coverage": {
            "tested": "native CLI typed DDS request/application-ack",
            "not_covered": "Gateway WebSocket ingress",
        },
        "diagnostic_overrides": {
            "realtime_factor": getattr(args, "realtime_factor", None),
        },
        "terrain_producer_contract": dict(TERRAIN_PRODUCER_CONTRACT),
        "mapd_evidence": {
            "contract": dict(MAPD_DATA_CONTRACT),
            "required_for_executed_cases": not simulation_fixture,
            "coverage": (
                "not_covered_state_provider_cannot_publish_slam_map_observation"
                if simulation_fixture
                else "preflight_only_not_evaluated"
                if bool(args.preflight_only)
                else evidence_class
            ),
            "cases": [
                {
                    "scenario": str(case.get("scenario") or ""),
                    **dict(case.get("mapd_evidence") or {}),
                }
                for case in cases
                if isinstance(case.get("mapd_evidence"), Mapping)
            ],
        },
        "scenarios": scenarios,
        "preflight_only": bool(args.preflight_only),
        "preflight": preflight,
        "cases": cases,
        "failure": ({"reason": failure_reason} if failure_reason else {}),
        "blockers": blockers,
    }
    output = (
        Path(args.json_out).expanduser().resolve()
        if args.json_out
        else Path(args.artifact_dir).expanduser().resolve() / "report.json"
    )
    _write_json(output, report)
    report["report_path"] = str(output)
    return report


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(list(argv) if argv is not None else None)
    report = run(args)
    print(
        json.dumps(
            {
                "ok": report["ok"],
                "acceptance_evaluated": report["acceptance_evaluated"],
                "product_acceptance_passed": report["product_acceptance_passed"],
                "executed_case_count": report["executed_case_count"],
                "requested_case_count": report["requested_case_count"],
                "blockers": report["blockers"],
                "report": report["report_path"],
            },
            indent=2,
            ensure_ascii=False,
        )
    )
    return 1 if bool(args.strict) and report["ok"] is not True else 0


def _twist_norm(value: Mapping[str, Any] | Sequence[Any]) -> float:
    if isinstance(value, Mapping):
        vx = float(value.get("vx") or 0.0)
        vy = float(value.get("vy") or 0.0)
        wz = float(value.get("wz") or 0.0)
    else:
        parts = list(value)
        vx = float(parts[0]) if len(parts) > 0 else 0.0
        vy = float(parts[1]) if len(parts) > 1 else 0.0
        wz = float(parts[2]) if len(parts) > 2 else 0.0
    return math.sqrt(vx * vx + vy * vy + wz * wz)


def _policy_motion_xy(samples: Sequence[Mapping[str, Any]]) -> float:
    driving = [sample for sample in samples if sample.get("driving") is True]
    if len(driving) < 2:
        return 0.0
    first = driving[0]
    last = driving[-1]
    return math.hypot(
        float(last.get("x") or 0.0) - float(first.get("x") or 0.0),
        float(last.get("y") or 0.0) - float(first.get("y") or 0.0),
    )


def _phase_motion_counts(samples: Sequence[Mapping[str, Any]], phase: str) -> tuple[int, int]:
    selected = [sample for sample in samples if sample.get("phase") == phase and sample.get("driving") is True]
    nonzero = sum(_twist_norm(sample.get("cmd") or ()) > 1e-4 for sample in selected)
    return int(nonzero), len(selected) - int(nonzero)


def assign_motion_phases(
    samples: Sequence[Mapping[str, Any]],
    events: Sequence[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    """Correlate motion samples with markers on the same hardware clock."""

    ordered_events = sorted(
        (
            (
                float(
                    event.get("motion_s")
                    if isinstance(event.get("motion_s"), (int, float))
                    else event.get("wall_s") or 0.0
                ),
                str(event.get("phase") or ""),
            )
            for event in events
            if str(event.get("phase") or "")
        ),
        key=lambda item: item[0],
    )
    assigned: list[dict[str, Any]] = []
    for source in samples:
        sample = dict(source)
        stamp = float(sample.get("t") or 0.0)
        phase = "warmup"
        for event_stamp, event_phase in ordered_events:
            if stamp + 1e-9 < event_stamp:
                break
            phase = event_phase
        sample["phase"] = phase
        assigned.append(sample)
    return assigned


def _counter_delta(samples: Sequence[Mapping[str, Any]], counter_name: str) -> int | None:
    values = [
        int(((sample.get("nav") or {}).get("counters") or {})[counter_name])
        for sample in samples
        if counter_name in ((sample.get("nav") or {}).get("counters") or {})
    ]
    return max(values) - min(values) if len(values) >= 2 else None


def _evaluate_dropout_recovery(
    *,
    case_name: str,
    expected_stale_reason: str,
    nav_samples: Sequence[Mapping[str, Any]],
    motion_samples: Sequence[Mapping[str, Any]],
    command_vx: float,
    require_correlated_slam_dropout: bool = False,
) -> dict[str, Any]:
    blockers: list[str] = []
    baseline = [sample for sample in nav_samples if sample.get("phase") == "baseline"]
    dropout = [sample for sample in nav_samples if sample.get("phase") == "dropout"]
    recovery = [sample for sample in nav_samples if sample.get("phase") == "recovery"]
    if not any(bool(((sample.get("nav") or {}).get("input_gate") or {}).get("ready")) for sample in baseline):
        blockers.append("dropout_baseline_not_ready")
    dropout_reasons = [
        str(((sample.get("nav") or {}).get("input_gate") or {}).get("reason") or "") for sample in dropout
    ]
    if expected_stale_reason not in dropout_reasons:
        blockers.append(f"{expected_stale_reason}_not_observed")
    baseline_nonzero, _ = _phase_motion_counts(motion_samples, "baseline")
    dropout_nonzero, dropout_zero = _phase_motion_counts(motion_samples, "dropout")
    recovery_nonzero, _ = _phase_motion_counts(motion_samples, "recovery")
    dropout_final_nonzero = sum(
        _twist_norm((sample.get("nav") or {}).get("final_cmd_vel") or {}) > 1e-4 for sample in dropout
    )
    dropout_final_zero = len(dropout) - dropout_final_nonzero
    if baseline_nonzero <= 0:
        blockers.append("dropout_baseline_policy_command_missing")
    if dropout_zero < 2 or dropout_nonzero > 0:
        blockers.append("dropout_zero_policy_window_missing")
    if dropout_final_zero < 2 or dropout_final_nonzero > 0:
        blockers.append("dropout_zero_final_command_window_missing")

    recovery_sequence: list[str] = []
    recovering_nonzero_final = 0
    for sample in recovery:
        gate = (sample.get("nav") or {}).get("input_gate") or {}
        state = "ready" if gate.get("ready") is True else str(gate.get("reason") or "")
        if state in {"recovering", "ready"} and (not recovery_sequence or recovery_sequence[-1] != state):
            recovery_sequence.append(state)
        if state == "recovering" and _twist_norm((sample.get("nav") or {}).get("final_cmd_vel") or {}) > 1e-4:
            recovering_nonzero_final += 1
    if recovery_sequence[:2] != ["recovering", "ready"]:
        blockers.append("dropout_recovery_hysteresis_missing")
    if recovering_nonzero_final > 0:
        blockers.append("dropout_recovery_nonzero_before_ready")
    if recovery_nonzero <= 0:
        blockers.append("dropout_recovered_policy_command_missing")

    counts = [int(((sample.get("nav") or {}).get("counters") or {}).get("teleop_cmd") or 0) for sample in nav_samples]
    count_delta = max(counts) - min(counts) if len(counts) >= 2 else 0
    dropout_counts = [
        int(((sample.get("nav") or {}).get("counters") or {}).get("teleop_cmd") or 0) for sample in dropout
    ]
    dropout_count_delta = max(dropout_counts) - min(dropout_counts) if len(dropout_counts) >= 2 else 0
    if dropout_count_delta < 2:
        blockers.append("dropout_continuous_teleop_missing")

    correlated_streams_stale: list[str] = []
    correlated_counter_deltas: dict[str, int | None] = {}
    if require_correlated_slam_dropout:
        age_fields = {
            "odom": ("odom_age_s", "odom_max_age_s"),
            "tf": ("tf_age_s", "tf_max_age_s"),
            "registered_cloud": ("cloud_age_s", "cloud_max_age_s"),
            "localization_health": (
                "localization_health_age_s",
                "localization_health_max_age_s",
            ),
        }
        for stream_name, (age_name, limit_name) in age_fields.items():
            if any(
                float(((sample.get("nav") or {}).get("input_gate") or {}).get(age_name) or -1.0)
                > float(((sample.get("nav") or {}).get("input_gate") or {}).get(limit_name) or 0.0)
                > 0.0
                for sample in dropout
            ):
                correlated_streams_stale.append(stream_name)
            else:
                blockers.append(f"slam_dropout_stream_stale_not_observed:{stream_name}")
        for stream_name, counter_name in {
            "odom": "odom",
            "tf": "tf",
            "registered_cloud": "registered_clouds",
        }.items():
            delta = _counter_delta(dropout, counter_name)
            correlated_counter_deltas[stream_name] = delta
            if delta is None:
                blockers.append(f"slam_dropout_counter_missing:{stream_name}")
            elif delta != 0:
                blockers.append(f"slam_dropout_stream_advanced:{stream_name}")

    return {
        "schema_version": SCHEMA_VERSION,
        "case": case_name,
        "ok": not blockers,
        "injected": False,
        "evidence_scope": "product_e2e_fault_injection",
        "producer_e2e": True,
        "blockers": blockers,
        "metrics": {
            "command_vx": float(command_vx),
            "nav_samples": len(nav_samples),
            "motion_samples": len(motion_samples),
            "dropout_reason": (
                expected_stale_reason
                if expected_stale_reason in dropout_reasons
                else (dropout_reasons[-1] if dropout_reasons else "")
            ),
            "dropout_zero_cmd_samples": dropout_zero,
            "dropout_zero_final_cmd_samples": dropout_final_zero,
            "recovery_sequence": recovery_sequence,
            "input_gate_generation_recovery_proven": (recovery_sequence[:2] == ["recovering", "ready"]),
            "teleop_command_count_delta": count_delta,
            "dropout_teleop_command_count_delta": dropout_count_delta,
            "recovering_nonzero_final_cmd_samples": recovering_nonzero_final,
            "baseline_nonzero_cmd_samples": baseline_nonzero,
            "recovery_nonzero_cmd_samples": recovery_nonzero,
            "correlated_slam_streams_stale": correlated_streams_stale,
            "correlated_slam_counter_deltas": correlated_counter_deltas,
        },
    }


def _finite_float(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _roll_pitch_from_wxyz(
    quaternion: Sequence[Any],
) -> tuple[float, float] | None:
    if len(quaternion) < 4:
        return None
    values = [_finite_float(value) for value in quaternion[:4]]
    if any(value is None for value in values):
        return None
    w, x, y, z = (float(value) for value in values)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-9:
        return None
    w, x, y, z = (value / norm for value in (w, x, y, z))
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    return roll, pitch


def _posture_candidates(sample: Mapping[str, Any], *, origin: str) -> list[tuple[str, Mapping[str, Any]]]:
    candidates: list[tuple[str, Mapping[str, Any]]] = [(origin, sample)]
    for key in ("motion", "simulation_posture", "sim_posture"):
        nested = sample.get(key)
        if isinstance(nested, Mapping):
            candidates.append((f"{origin}.{key}", nested))
    nav = sample.get("nav")
    if isinstance(nav, Mapping):
        for key in ("simulation_posture", "sim_posture"):
            nested = nav.get(key)
            if isinstance(nested, Mapping):
                candidates.append((f"{origin}.nav.{key}", nested))
    return candidates


def _extract_simulation_posture(sample: Mapping[str, Any], *, origin: str) -> dict[str, Any] | None:
    for source, candidate in _posture_candidates(sample, origin=origin):
        base_z_m: float | None = None
        roll_rad: float | None = None
        pitch_rad: float | None = None
        source_kind = source

        qpos = candidate.get("qpos")
        if isinstance(qpos, Sequence) and not isinstance(qpos, (str, bytes)) and len(qpos) >= 7:
            base_z_m = _finite_float(qpos[2])
            roll_pitch = _roll_pitch_from_wxyz(qpos[3:7])
            if roll_pitch is not None:
                roll_rad, pitch_rad = roll_pitch
            source_kind = f"{source}.qpos"

        if base_z_m is None:
            for key in ("base_z_m", "base_z", "z"):
                base_z_m = _finite_float(candidate.get(key))
                if base_z_m is not None:
                    break
        if roll_rad is None:
            for key in ("roll_rad", "roll"):
                roll_rad = _finite_float(candidate.get(key))
                if roll_rad is not None:
                    break
        if pitch_rad is None:
            for key in ("pitch_rad", "pitch"):
                pitch_rad = _finite_float(candidate.get(key))
                if pitch_rad is not None:
                    break
        if roll_rad is None or pitch_rad is None:
            orientation = candidate.get("orientation")
            if isinstance(orientation, Mapping):
                roll_pitch = _roll_pitch_from_wxyz(
                    [
                        orientation.get("w"),
                        orientation.get("x"),
                        orientation.get("y"),
                        orientation.get("z"),
                    ]
                )
                if roll_pitch is not None:
                    roll_rad = roll_rad if roll_rad is not None else roll_pitch[0]
                    pitch_rad = pitch_rad if pitch_rad is not None else roll_pitch[1]

        explicit_fall = any(candidate.get(key) is True for key in ("fallen", "fall_detected", "is_fallen"))
        if base_z_m is not None or roll_rad is not None or pitch_rad is not None or explicit_fall:
            return {
                "source": source_kind,
                "phase": str(sample.get("phase") or ""),
                "base_z_m": base_z_m,
                "roll_rad": roll_rad,
                "pitch_rad": pitch_rad,
                "explicit_fall": explicit_fall,
            }
    return None


def evaluate_simulation_posture(
    motion_samples: Sequence[Mapping[str, Any]],
    nav_samples: Sequence[Mapping[str, Any]] = (),
) -> dict[str, Any]:
    """Validate MuJoCo body posture before interpreting safety-policy reasons."""

    driving_motion = [sample for sample in motion_samples if sample.get("driving") is True]
    motion_records = [
        record
        for sample in driving_motion
        if (record := _extract_simulation_posture(sample, origin="motion")) is not None
    ]
    if not motion_records:
        motion_records = [
            record
            for sample in motion_samples
            if (record := _extract_simulation_posture(sample, origin="motion")) is not None
        ]
    records = motion_records
    evidence_origin = "motion" if records else "timeline"
    if not records:
        records = [
            record
            for sample in nav_samples
            if (record := _extract_simulation_posture(sample, origin="timeline")) is not None
        ]

    thresholds = dict(SIMULATION_POSTURE_GATE)
    min_base_z_m = float(thresholds["min_base_z_m"])
    max_abs_roll_rad = float(thresholds["max_abs_roll_rad"])
    max_abs_pitch_rad = float(thresholds["max_abs_pitch_rad"])
    required_run = int(thresholds["min_consecutive_invalid_samples"])
    current_run = 0
    longest_run = 0
    invalid_samples = 0
    explicit_fall_samples = 0
    violation_codes: set[str] = set()
    first_invalid: dict[str, Any] = {}
    base_z_values: list[float] = []
    roll_values: list[float] = []
    pitch_values: list[float] = []

    for index, record in enumerate(records):
        violations: list[str] = []
        base_z_m = record.get("base_z_m")
        roll_rad = record.get("roll_rad")
        pitch_rad = record.get("pitch_rad")
        if isinstance(base_z_m, (int, float)):
            base_z_values.append(float(base_z_m))
            if float(base_z_m) < min_base_z_m:
                violations.append("base_z_below_min")
        if isinstance(roll_rad, (int, float)):
            roll_values.append(float(roll_rad))
            if abs(float(roll_rad)) > max_abs_roll_rad:
                violations.append("roll_exceeds_limit")
        if isinstance(pitch_rad, (int, float)):
            pitch_values.append(float(pitch_rad))
            if abs(float(pitch_rad)) > max_abs_pitch_rad:
                violations.append("pitch_exceeds_limit")
        if record.get("explicit_fall") is True:
            violations.append("explicit_fall")
            explicit_fall_samples += 1

        if violations:
            invalid_samples += 1
            current_run += 1
            longest_run = max(longest_run, current_run)
            violation_codes.update(violations)
            if not first_invalid:
                first_invalid = {
                    "sample_index": index,
                    "source": record.get("source"),
                    "phase": record.get("phase"),
                    "base_z_m": base_z_m,
                    "roll_rad": roll_rad,
                    "pitch_rad": pitch_rad,
                    "violations": violations,
                }
        else:
            current_run = 0

    fall_detected = explicit_fall_samples > 0 or longest_run >= required_run
    failure_reason = "simulation_posture_invalid" if fall_detected else ""
    return {
        "ok": not fall_detected,
        "evaluated": bool(records),
        "reason": failure_reason,
        "evidence_origin": evidence_origin if records else "unavailable",
        "sources": sorted({str(record.get("source") or "") for record in records}),
        "thresholds": thresholds,
        "samples": len(records),
        "invalid_samples": invalid_samples,
        "longest_invalid_run": longest_run,
        "fall_detected": fall_detected,
        "min_base_z_m": min(base_z_values) if base_z_values else None,
        "max_abs_roll_rad": max((abs(value) for value in roll_values), default=None),
        "max_abs_pitch_rad": max((abs(value) for value in pitch_values), default=None),
        "violations": sorted(violation_codes),
        "first_invalid": first_invalid,
    }


def _posture_failure_evaluation(
    name: str,
    *,
    nav_samples: Sequence[Mapping[str, Any]],
    motion_samples: Sequence[Mapping[str, Any]],
    command_vx: float,
    injected: bool,
    posture: Mapping[str, Any],
) -> dict[str, Any]:
    terrain_case = name in {"terrain_soft", "terrain_hard"}
    evidence_scope = (
        "dds_consumer_contract_injected"
        if terrain_case and injected
        else "terrain_producer_e2e"
        if terrain_case
        else "product_e2e"
    )
    return {
        "schema_version": SCHEMA_VERSION,
        "case": name,
        "ok": False,
        "injected": bool(injected),
        "evidence_scope": evidence_scope,
        "producer_e2e": bool(terrain_case and not injected),
        "blockers": ["simulation_posture_invalid"],
        "failure": {"reason": "simulation_posture_invalid"},
        "simulation_posture": dict(posture),
        "policy_attribution": {
            "evaluated": False,
            "reason": "simulation_posture_invalid",
        },
        "metrics": {
            "command_vx": float(command_vx),
            "nav_samples": len(nav_samples),
            "motion_samples": len(motion_samples),
            "ready_samples": 0,
            "steady_nonzero_cmd_samples": 0,
            "steady_zero_cmd_samples": 0,
            "teleop_command_count_delta": 0,
            "median_policy_cmd_vx": 0.0,
            "median_output_scale": 0.0,
            "accepted_ratio": 0.0,
            "nonzero_command_ratio": 0.0,
            "policy_motion_xy_m": 0.0,
            "obstacle_distance_m": -1.0,
            "traversability_cost": -1.0,
            # Do not recast safety reasons observed after a fall as causal
            # obstacle or terrain evidence.
            "teleop_reasons": [],
        },
    }


def evaluate_case(
    name: str,
    *,
    nav_samples: Sequence[Mapping[str, Any]],
    motion_samples: Sequence[Mapping[str, Any]],
    command_vx: float,
    injected: bool = False,
) -> dict[str, Any]:
    """Evaluate one observed scenario through status and policy-motion interfaces."""

    posture = evaluate_simulation_posture(motion_samples, nav_samples)
    if posture.get("ok") is not True:
        return _posture_failure_evaluation(
            name,
            nav_samples=nav_samples,
            motion_samples=motion_samples,
            command_vx=command_vx,
            injected=injected,
            posture=posture,
        )

    if name == "traversability_dropout_recovery":
        result = _evaluate_dropout_recovery(
            case_name=name,
            expected_stale_reason="traversability_stale",
            nav_samples=nav_samples,
            motion_samples=motion_samples,
            command_vx=command_vx,
        )
        result["simulation_posture"] = posture
        result["policy_attribution"] = {"evaluated": True, "reason": ""}
        return result
    if name == "slam_inputs_dropout_recovery":
        result = _evaluate_dropout_recovery(
            case_name=name,
            expected_stale_reason="odom_stale",
            nav_samples=nav_samples,
            motion_samples=motion_samples,
            command_vx=command_vx,
            require_correlated_slam_dropout=True,
        )
        result["simulation_posture"] = posture
        result["policy_attribution"] = {"evaluated": True, "reason": ""}
        return result

    blockers: list[str] = []
    steady_nav = [sample for sample in nav_samples if sample.get("phase") == "steady"]
    steady_motion = [sample for sample in motion_samples if sample.get("phase") == "steady"]
    ready = [sample for sample in steady_nav if bool(((sample.get("nav") or {}).get("input_gate") or {}).get("ready"))]
    if not ready:
        blockers.append("input_gate_ready_missing")
    if not any(str((sample.get("nav") or {}).get("control_mode") or "") == "teleop_avoid" for sample in steady_nav):
        blockers.append("teleop_avoid_mode_missing")

    reasons = [str(((sample.get("nav") or {}).get("teleop") or {}).get("reason") or "") for sample in ready]
    free_accepted_count = sum(
        reason in FREE_COMMAND_ACCEPTED_REASONS for reason in reasons
    )
    if name == "free" and free_accepted_count == 0:
        blockers.append("free_command_not_accepted")
    if name == "obstacle_slow":
        slow_reasons = {"obstacle_slow", "obstacle_terrain_slow"}
        if sum(reason in slow_reasons for reason in reasons) < 3:
            blockers.append("obstacle_slow_decision_missing")
    if name == "obstacle_stop" and reasons.count("obstacle_stop") < 3:
        blockers.append("obstacle_stop_decision_missing")
    if name == "terrain_soft" and sum(reason in {"terrain_slow", "obstacle_terrain_slow"} for reason in reasons) < 3:
        blockers.append("terrain_soft_decision_missing")
    if name == "terrain_hard" and reasons.count("terrain_stop") < 3:
        blockers.append("terrain_hard_decision_missing")

    nonzero = sum(
        1 for sample in steady_motion if sample.get("driving") is True and _twist_norm(sample.get("cmd") or ()) > 1e-4
    )
    zero = sum(
        1 for sample in steady_motion if sample.get("driving") is True and _twist_norm(sample.get("cmd") or ()) <= 1e-4
    )
    teleop_counts = [
        int(((sample.get("nav") or {}).get("counters") or {}).get("teleop_cmd") or 0) for sample in steady_nav
    ]
    teleop_count_delta = max(teleop_counts) - min(teleop_counts) if len(teleop_counts) >= 2 else 0
    if teleop_count_delta < 2:
        blockers.append(f"{name}_continuous_teleop_missing")
    command_values = [
        float((sample.get("cmd") or [0.0])[0])
        for sample in steady_motion
        if sample.get("driving") is True and sample.get("cmd")
    ]
    policy_motion_xy_m = _policy_motion_xy(steady_motion)
    median_policy_cmd_vx = statistics.median(command_values) if command_values else 0.0
    median_output_scale = median_policy_cmd_vx / float(command_vx) if abs(float(command_vx)) > 1e-9 else 0.0
    accepted_ratio = free_accepted_count / len(reasons) if reasons else 0.0
    nonzero_ratio = nonzero / len(command_values) if command_values else 0.0
    obstacle_distances = [
        float(((sample.get("nav") or {}).get("teleop") or {}).get("obstacle_distance_m"))
        for sample in ready
        if isinstance(
            (((sample.get("nav") or {}).get("teleop") or {}).get("obstacle_distance_m")),
            (int, float),
        )
        and float(((sample.get("nav") or {}).get("teleop") or {}).get("obstacle_distance_m")) >= 0.0
    ]
    traversability_costs = [
        float(((sample.get("nav") or {}).get("teleop") or {}).get("traversability_cost"))
        for sample in ready
        if isinstance(
            (((sample.get("nav") or {}).get("teleop") or {}).get("traversability_cost")),
            (int, float),
        )
        and float(((sample.get("nav") or {}).get("teleop") or {}).get("traversability_cost")) >= 0.0
    ]
    if name == "free":
        hazard_reasons = sorted(
            {
                reason
                for reason in reasons
                if reason and ("obstacle" in reason or "terrain" in reason or reason in {"yaw_obstacle", "stopped"})
            }
        )
        if hazard_reasons:
            blockers.append("free_hazard_decision_observed")
        if accepted_ratio < 0.80:
            blockers.append("free_accepted_ratio_too_low")
        if not 0.80 <= median_output_scale <= 1.05:
            blockers.append("free_output_scale_out_of_range")
        if nonzero_ratio < 0.80:
            blockers.append("free_nonzero_command_ratio_too_low")
        if nonzero < 3:
            blockers.append("free_policy_command_missing")
        if policy_motion_xy_m < 0.10:
            blockers.append("free_policy_motion_too_small")
    elif name == "obstacle_slow":
        if nonzero < 3:
            blockers.append("obstacle_slow_policy_command_missing")
        if not 0.25 <= median_output_scale <= 0.45:
            blockers.append("obstacle_slow_scale_out_of_range")
        if not obstacle_distances:
            blockers.append("obstacle_slow_distance_missing")
    elif name == "obstacle_stop":
        if zero < 3:
            blockers.append("obstacle_stop_zero_command_missing")
        if policy_motion_xy_m > 0.03:
            blockers.append("obstacle_stop_policy_motion_excessive")
    elif name == "terrain_soft":
        median_cost = statistics.median(traversability_costs) if traversability_costs else -1.0
        if not 40.0 <= median_cost < 80.0:
            blockers.append("terrain_soft_cost_out_of_range")
        if not 0.25 <= median_output_scale <= 0.45:
            blockers.append("terrain_soft_scale_out_of_range")
        if nonzero < 3:
            blockers.append("terrain_soft_policy_command_missing")
    elif name == "terrain_hard":
        median_cost = statistics.median(traversability_costs) if traversability_costs else -1.0
        if median_cost < 80.0:
            blockers.append("terrain_hard_cost_below_threshold")
        if zero < 3 or nonzero > 0:
            blockers.append("terrain_hard_zero_command_missing")
        if policy_motion_xy_m > 0.03:
            blockers.append("terrain_hard_policy_motion_excessive")

    terrain_case = name in {"terrain_soft", "terrain_hard"}
    evidence_scope = (
        "dds_consumer_contract_injected"
        if terrain_case and injected
        else "terrain_producer_e2e"
        if terrain_case
        else "product_e2e"
    )

    return {
        "schema_version": SCHEMA_VERSION,
        "case": name,
        "ok": not blockers,
        "injected": bool(injected),
        "evidence_scope": evidence_scope,
        "producer_e2e": bool(terrain_case and not injected),
        "blockers": blockers,
        "simulation_posture": posture,
        "policy_attribution": {"evaluated": True, "reason": ""},
        "metrics": {
            "command_vx": float(command_vx),
            "nav_samples": len(nav_samples),
            "motion_samples": len(motion_samples),
            "ready_samples": len(ready),
            "steady_nonzero_cmd_samples": nonzero,
            "steady_zero_cmd_samples": zero,
            "teleop_command_count_delta": teleop_count_delta,
            "median_policy_cmd_vx": median_policy_cmd_vx,
            "median_output_scale": round(median_output_scale, 6),
            "accepted_ratio": round(accepted_ratio, 6),
            "nonzero_command_ratio": round(nonzero_ratio, 6),
            "policy_motion_xy_m": policy_motion_xy_m,
            "obstacle_distance_m": (statistics.median(obstacle_distances) if obstacle_distances else -1.0),
            "traversability_cost": (statistics.median(traversability_costs) if traversability_costs else -1.0),
            "teleop_reasons": sorted(set(reasons)),
        },
    }


if __name__ == "__main__":
    raise SystemExit(main())

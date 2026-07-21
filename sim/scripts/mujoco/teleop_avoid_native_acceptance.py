#!/usr/bin/env python3
"""Run and evaluate the native ``teleop_avoid`` MuJoCo product chain."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
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

SCHEMA_VERSION = "lingtu.mujoco.teleop_avoid_native_acceptance.v1"
ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
DEFAULT_MANIFEST = ROOT / "config" / "runtime_graph" / "endpoints" / "mujoco_native_navigation_acceptance.json"
DEFAULT_SCENARIOS = (
    "free",
    "obstacle_slow",
    "obstacle_stop",
    "terrain_soft",
    "terrain_hard",
    "traversability_dropout_recovery",
)
OPTIONAL_SCENARIOS = (
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
    "track_against_map_period_s": 5.0,
    "sensor_offset_x_m": -0.011,
    "sensor_offset_y_m": -0.02329,
    "sensor_offset_z_m": 0.04412,
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
        "pos": "0.94 0 0.40",
        "size": "0.04 0.25 0.40",
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
        "type": "box",
        "pos": "1.55 0 0.05",
        "size": "0.65 0.72 0.05",
        "group": "0",
        "rgba": "0.80 0.55 0.10 1",
    },
    "terrain_hard": {
        "name": "acceptance_terrain_hard",
        "type": "box",
        "pos": "1.55 0 0.12",
        "size": "0.65 0.72 0.12",
        "group": "0",
        "rgba": "0.75 0.10 0.75 1",
    },
}


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
    spec = _SCENE_GEOMS.get(scenario)
    if spec is not None:
        ET.SubElement(worldbody, "geom", dict(spec))
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
) -> dict[str, Any]:
    """Describe the exact real processes and artifacts for one scenario."""

    from sim.scripts.mujoco import native_navigation_acceptance as native

    case_dir = case_dir.resolve()
    case_dir.mkdir(parents=True, exist_ok=True)
    artifacts = {
        "scene": str(case_dir / "scene.xml"),
        "slam_status": str(case_dir / "slam_status.json"),
        "slam_cloud_dir": str(case_dir / "slam_clouds"),
        "traversability_status": str(case_dir / "traversability_status.json"),
        "nav_status": str(case_dir / "nav_status.json"),
        "sensor_report": str(case_dir / "sensor_report.json"),
        "motion_log": str(case_dir / "motion.jsonl"),
        "nav_timeline": str(case_dir / "nav_timeline.jsonl"),
        "sensor_publisher_pid": str(case_dir / "sensor_publisher.pid"),
        "cmd_vel_tap_pid": str(case_dir / "cmd_vel_tap.pid"),
        "teleop_log": str(case_dir / "teleop_command.log"),
        "teleop_current_log": str(case_dir / "teleop_command_current.log"),
        "estop_latch": str(case_dir / "estop_latch.json"),
    }
    Path(artifacts["slam_cloud_dir"]).mkdir(parents=True, exist_ok=True)
    start = [float(value) for value in manifest.get("start") or [0.0, 0.0, 0.48, 0.0]]
    tolerances = dict(manifest.get("runtime_tolerances") or {})
    scene_variant = scenario.removesuffix("_injected")
    if scenario.endswith("_injected") or scene_variant in {
        "traversability_dropout_recovery",
        "slam_inputs_dropout_recovery",
    }:
        scene_variant = "free"
    obstacle_case = scenario in {"obstacle_slow", "obstacle_stop"}
    traversability_parameters = {
        "obstacle_min_z_m": 2.0 if obstacle_case else 0.10,
        "terrain_soft_height_m": 2.0 if obstacle_case else 0.08,
        "terrain_hard_height_m": 3.0 if obstacle_case else 0.20,
        "terrain_soft_slope_deg": 100.0 if obstacle_case else 12.0,
        "terrain_hard_slope_deg": 120.0 if obstacle_case else 28.0,
    }

    slam_command = native._native_command(
        Path(binaries["slam"]),
        "--backend",
        "fastlio2",
        "--mode",
        "localization",
        "--map",
        native._linux_arg(Path(paths["slam"])),
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
        "--track-against-map-period-s",
        str(
            float(
                tolerances.get("track_against_map_period_s") or FIELD_TELEOP_AVOID_PROFILE["track_against_map_period_s"]
            )
        ),
        "--track-against-map-initial-pose",
        *(str(value) for value in start),
    )
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
        "--publish-cmd-vel",
        "true",
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
    ]
    sensor_command.extend(native._sensor_runtime_args(dict(manifest)))
    if bool(manifest.get("_odom_prior_diagnostic")):
        sensor_command.append("--allow-kinematic-fastlio-acceptance")
    teleop_command = native._native_command(
        Path(binaries["navigation_control"]),
        "teleop",
        str(float(command_vx)),
        "0",
        "0",
        "--duration-s",
        str(max(60.0, 3.0 * (float(warmup_s) + float(duration_s)))),
        "--rate-hz",
        "10",
        "--domain-id",
        str(domain_id),
        "--timeout-ms",
        "3000",
    )
    return {
        "scenario": scenario,
        "scene_variant": scene_variant,
        "domain_id": int(domain_id),
        "processes": [
            {"name": "slam", "command": slam_command, "log": str(case_dir / "slam.log")},
            {"name": "traversability", "command": traversability_command, "log": str(case_dir / "traversability.log")},
            {"name": "navigation", "command": navigation_command, "log": str(case_dir / "navigation.log")},
            {"name": "sensor", "command": sensor_command, "log": str(case_dir / "sensor.log")},
        ],
        "teleop_command": teleop_command,
        "artifacts": artifacts,
        "functional_scope": {
            "live_obstacle_layer": True,
            "traversability_process": True,
            "traversability_cost_in_decision": True,
            "traversability_parameters": traversability_parameters,
            "isolation": (
                "live_obstacle_decision_with_free_cost_producer_thresholds"
                if obstacle_case
                else "full_teleop_avoid_inputs"
            ),
        },
        "terrain_producer_contract": {
            **TERRAIN_PRODUCER_CONTRACT,
            "scenario_step_height_m": (
                0.10 if scene_variant == "terrain_soft" else 0.24 if scene_variant == "terrain_hard" else None
            ),
        },
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
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
    parser.add_argument(
        "--track-against-map-period-s",
        type=float,
        default=None,
        help="Diagnostic native SLAM map-registration period; product manifest is unchanged by default.",
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

    endpoint_cpp = ROOT / "src" / "nav" / "services" / "endpoint" / "cpp"
    common_sources = [
        ROOT / "src" / "message" / "idl" / "lingtu_slam.idl",
        ROOT / "src" / "message" / "cpp" / "dds_topics.hpp",
        ROOT / "src" / "message" / "cpp" / "dds_qos_profiles.hpp",
    ]
    source_specs = {
        "navigation": [
            endpoint_cpp / "active_octomap_gate.cpp",
            endpoint_cpp / "active_octomap_gate.hpp",
            endpoint_cpp / "nav_native_endpoint.cpp",
            endpoint_cpp / "global_plan_task.cpp",
            endpoint_cpp / "global_plan_task.hpp",
            endpoint_cpp / "nav_dds_runtime.cpp",
            endpoint_cpp / "nav_dds_runtime.hpp",
            endpoint_cpp / "nav_endpoint_config.cpp",
            endpoint_cpp / "nav_endpoint_config.hpp",
            endpoint_cpp / "input_gate.cpp",
            endpoint_cpp / "input_gate.hpp",
            endpoint_cpp / "nav_endpoint_messages.cpp",
            endpoint_cpp / "nav_endpoint_messages.hpp",
            endpoint_cpp / "nav_status_writer.cpp",
            endpoint_cpp / "nav_status_writer.hpp",
            endpoint_cpp / "motion_layer.cpp",
            endpoint_cpp / "motion_layer.hpp",
            endpoint_cpp / "live_obstacle_layer.cpp",
            endpoint_cpp / "live_obstacle_layer.hpp",
            endpoint_cpp / "teleop_safety.cpp",
            endpoint_cpp / "teleop_safety.hpp",
            endpoint_cpp / "control_authority.hpp",
            endpoint_cpp / "estop_latch_store.hpp",
            endpoint_cpp / "frame_transform.hpp",
            endpoint_cpp / "grid_inflation.hpp",
            endpoint_cpp / "point_cloud_layout.hpp",
            endpoint_cpp / "pose_buffer.hpp",
            endpoint_cpp / "transform_buffer.hpp",
            ROOT / "src" / "nav" / "services" / "plan" / "cpp",
            ROOT / "src" / "nav" / "kernel" / "src" / "path_follower_core.cpp",
            *common_sources,
        ],
        "navigation_control": [
            ROOT / "src" / "nav" / "commands" / "cpp",
            endpoint_cpp / "nav_control.cpp",
            *common_sources,
        ],
        "traversability": [
            endpoint_cpp / "traversability_dds.cpp",
            endpoint_cpp / "frame_transform.hpp",
            endpoint_cpp / "grid_inflation.hpp",
            endpoint_cpp / "observed_free_cache.hpp",
            endpoint_cpp / "observed_safety_grid.hpp",
            endpoint_cpp / "point_cloud_layout.hpp",
            endpoint_cpp / "safety_grid_probe.hpp",
            endpoint_cpp / "terrain_risk.hpp",
            endpoint_cpp / "transform_buffer.hpp",
            endpoint_cpp / "traversability_geometry.hpp",
            *common_sources,
        ],
        "slam": [
            ROOT / "src" / "localization" / "slam" / "cpp",
            ROOT / "src" / "localization" / "fastlio2" / "src",
            ROOT / "src" / "localization" / "localizer" / "src" / "localizers",
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
        if specs and all(source.is_file() or source.is_dir() for source in specs):
            source_files: list[Path] = []
            for source in specs:
                if source.is_file():
                    source_files.append(source)
                    continue
                source_files.extend(
                    path
                    for suffix in ("*.cpp", "*.hpp")
                    for path in source.rglob(suffix)
                    if path.is_file() and not path.name.startswith("test_") and "tests" not in path.parts
                )
            latest_source_mtime_ns = max(
                (path.stat().st_mtime_ns for path in source_files),
                default=0,
            )
            item["source_specs"] = [str(source) for source in specs]
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


def prepare_runtime(args: argparse.Namespace) -> dict[str, Any]:
    """Prepare same-source assets and resolve every native runtime dependency."""

    from sim.scripts.mujoco import native_navigation_acceptance as native

    artifact_dir = Path(args.artifact_dir).expanduser().resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = Path(args.manifest).expanduser().resolve()
    manifest = native._load_manifest(manifest_path)
    cached_scene = artifact_dir / "prepared_assets" / "scene.xml"
    cached_map_dir = artifact_dir / "prepared_assets" / "same_source_map"
    cached_names = manifest.get("map_files") or {}
    cached_map_ready = (cached_map_dir / str(cached_names.get("slam") or "map.pcd")).is_file()
    if not str(manifest.get("world") or "") and cached_scene.is_file() and cached_map_ready:
        manifest["world"] = str(cached_scene)
        manifest["map_dir"] = str(cached_map_dir)
    world_value = str(manifest.get("world") or "")
    world_path = native._repo_path(world_value) if world_value else None
    map_dir = native._repo_path(str(manifest.get("map_dir") or ""))
    slam_name = str((manifest.get("map_files") or {}).get("slam") or "map.pcd")
    slam_path = map_dir / slam_name
    if world_path is not None and world_path.is_file() and slam_path.is_file():
        asset_preparation = {
            "attempted": False,
            "ok": True,
            "reason": "teleop_avoid_assets_ready",
            "scene_xml": str(world_path),
            "map_dir": str(map_dir),
        }
    else:
        asset_preparation = native._prepare_acceptance_assets(manifest, artifact_dir)
    binaries, paths, blockers, provenance = native._preflight(manifest)
    required_binaries = {
        "sensor_publisher",
        "slam",
        "traversability",
        "navigation",
        "navigation_control",
        "cmd_vel_tap",
    }
    out_of_scope: list[str] = []
    in_scope: list[str] = []
    for blocker in blockers:
        missing_binary = blocker.split(":", 1)[1] if blocker.startswith("native_binary_missing:") else ""
        if (
            blocker.startswith("runtime_path_missing:path_library:")
            or blocker.startswith("map_artifact_missing:planner:")
            or blocker.startswith("map_artifact_missing:metadata:")
            or blocker
            in {
                "octomap_metadata_hash_mismatch",
                "octomap_not_derived_from_selected_map",
            }
            or (missing_binary and missing_binary not in required_binaries)
        ):
            out_of_scope.append(blocker)
        else:
            in_scope.append(blocker)
    blockers = in_scope
    for name in sorted(required_binaries):
        if name not in binaries:
            blockers.append(f"native_binary_missing:{name}")
    binary_provenance, stale_blockers = _binary_source_provenance(binaries)
    blockers.extend(stale_blockers)
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
            "asset_preparation": asset_preparation,
            "map_provenance": provenance,
            "binary_provenance": binary_provenance,
            "out_of_scope_preflight_findings": out_of_scope,
            "binaries": {name: str(path) for name, path in binaries.items()},
            "paths": {name: str(path) for name, path in paths.items()},
        },
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
    """Wait until MuJoCo has left warmup before starting a fault window."""

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


def _wait_for_runtime_ready(
    *,
    sensor: Any,
    nav_status: Path,
    slam_status: Path,
    traversability_status: Path,
    timeline: list[dict[str, Any]],
    state: dict[str, Any],
    timeout_s: float,
) -> tuple[bool, str]:
    from sim.scripts.mujoco import native_navigation_acceptance as native

    deadline = time.monotonic() + max(0.1, float(timeout_s))
    while time.monotonic() < deadline:
        if sensor.poll() is not None:
            return False, "sensor_runner_exited_before_runtime_ready"
        nav = _capture_nav_status(
            path=nav_status,
            phase="warmup",
            timeline=timeline,
            state=state,
        )
        slam = native._load_json(slam_status)
        traversability = native._load_json(traversability_status)
        if (
            str(slam.get("state") or "").upper() == "TRACKING"
            and str(nav.get("control_mode") or "") == "teleop_avoid"
            and bool((nav.get("input_gate") or {}).get("ready"))
            and int((traversability.get("counters") or {}).get("published") or 0) > 0
        ):
            return True, "ready"
        time.sleep(0.1)
    return False, "native_runtime_startup_timeout"


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
        output = process.tail()
        failure_reason = ""
        rejection_marker = "navigation command rejected:"
        if rejection_marker in output:
            failure_reason = output.split(rejection_marker, 1)[1].splitlines()[0].strip()
        elif "dds_wait_for_acks" in output and "Timeout" in output:
            failure_reason = "dds_ack_timeout"
        elif returncode not in (None, 0):
            failure_reason = "typed_teleop_client_failed"
        attempt = {
            "attempt": len(self.attempts) + 1,
            "returncode": returncode,
            "ack_timeout": "dds_wait_for_acks" in output and "Timeout" in output,
            "failure_reason": failure_reason,
            "cleanup": dict(getattr(process, "cleanup", {}) or {}),
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

    def stop(self) -> None:
        self._stop_event.set()
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
    tolerances = dict(manifest.get("runtime_tolerances") or {})
    if getattr(args, "realtime_factor", None) is not None:
        tolerances["sim_hardware_realtime_factor"] = max(0.05, float(args.realtime_factor))
    if getattr(args, "track_against_map_period_s", None) is not None:
        tolerances["track_against_map_period_s"] = max(0.1, float(args.track_against_map_period_s))
    manifest["runtime_tolerances"] = tolerances
    binaries = {name: Path(path) for name, path in (prepared.get("binaries") or {}).items()}
    paths = {name: Path(path) for name, path in (prepared.get("paths") or {}).items()}
    case_dir = Path(args.artifact_dir).expanduser().resolve() / "cases" / scenario
    odom_prior_diagnostic = bool(getattr(args, "odom_prior_diagnostic", False))
    if odom_prior_diagnostic:
        paths["slam_config"] = build_odom_prior_diagnostic_config(
            paths["slam_config"],
            case_dir / "slam_config_odom_prior_diagnostic.yaml",
        )
        sensor_runtime = dict(manifest.get("sensor_runtime") or {})
        sensor_runtime["publish_odom_prior"] = True
        manifest["sensor_runtime"] = sensor_runtime
        manifest["_odom_prior_diagnostic"] = True
    plan = build_execution_plan(
        scenario=scenario,
        domain_id=domain_id,
        binaries=binaries,
        paths=paths,
        case_dir=case_dir,
        duration_s=float(args.duration_s),
        warmup_s=float(args.warmup_s),
        command_vx=float(args.command_vx),
        manifest=manifest,
    )
    artifacts = {name: Path(value) for name, value in plan["artifacts"].items()}
    domain_tokens = ["--domain-id", str(domain_id)]
    prior_process_cleanup = reclaim_prior_case_processes(
        case_dir,
        artifacts,
        {
            "prior_slam": [Path(binaries["slam"]).name, *domain_tokens],
            "prior_traversability": [
                Path(binaries["traversability"]).name,
                *domain_tokens,
            ],
            "prior_navigation": [Path(binaries["navigation"]).name, *domain_tokens],
            "prior_teleop_command": [
                Path(binaries["navigation_control"]).name,
                "teleop",
                *domain_tokens,
            ],
            "prior_teleop_command_current": [
                Path(binaries["navigation_control"]).name,
                "teleop",
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
    slam = by_name["slam"]
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
    injection_result: dict[str, Any] = {}
    teleop_probe_result: dict[str, Any] = {}
    stop_result: dict[str, Any] = {}
    observed_reason = ""
    current_phase = "warmup"
    driving_ready = False
    driving_ready_reason = "not_waited"
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
            started_commands.append({"name": process.name, "command": process.command})
        startup_ok, startup_reason = _wait_for_runtime_ready(
            sensor=sensor,
            nav_status=artifacts["nav_status"],
            slam_status=artifacts["slam_status"],
            traversability_status=artifacts["traversability_status"],
            timeline=timeline,
            state=timeline_state,
            timeout_s=float(args.startup_timeout_s),
        )
        if startup_ok:
            teleop_probe_result = _run_control(
                binaries["navigation_control"],
                ["teleop", str(float(args.command_vx)), "0", "0"],
                domain_id=domain_id,
                timeout_s=8.0,
            )
            teleop = ResilientTeleopProcess(
                list(plan["teleop_command"]),
                artifacts["teleop_log"],
            )
            teleop.start()
            started_commands.append({"name": teleop.name, "command": teleop.command})
            driving_ready, driving_ready_reason = _wait_for_policy_driving(
                sensor=sensor,
                teleop=teleop,
                motion_log=artifacts["motion_log"],
                timeout_s=float(args.driving_start_timeout_s),
            )
            if not driving_ready:
                raise RuntimeError(driving_ready_reason)

            if scenario in {
                "traversability_dropout_recovery",
                "slam_inputs_dropout_recovery",
            }:
                fault_name = "traversability" if scenario == "traversability_dropout_recovery" else "slam"
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
            elif scenario == "free":
                mark("steady")
            else:
                mark("approach")
                expected_by_case = {
                    "obstacle_slow": {"obstacle_slow", "obstacle_terrain_slow"},
                    "obstacle_stop": {"obstacle_stop"},
                    "terrain_soft": {"terrain_slow", "obstacle_terrain_slow"},
                    "terrain_hard": {"terrain_stop"},
                }
                found, observed_reason = _wait_for_teleop_reason(
                    expected=expected_by_case.get(scenario, set()),
                    sensor=sensor,
                    nav_status=artifacts["nav_status"],
                    phase="approach",
                    timeline=timeline,
                    state=timeline_state,
                    timeout_s=max(2.0, float(args.duration_s) - 1.5),
                )
                if found:
                    mark("steady")
                    _collect_for(
                        sensor=sensor,
                        nav_status=artifacts["nav_status"],
                        phase="steady",
                        timeline=timeline,
                        state=timeline_state,
                        duration_s=1.0,
                    )
                    mark("post_case")

            deadline = time.monotonic() + max(20.0, float(args.duration_s) + 45.0)
            while sensor.poll() is None and time.monotonic() < deadline:
                _capture_nav_status(
                    path=artifacts["nav_status"],
                    phase=current_phase,
                    timeline=timeline,
                    state=timeline_state,
                )
                time.sleep(0.05)
            if sensor.poll() is None:
                raise TimeoutError("MuJoCo sensor/policy runner exceeded case deadline")
    except Exception as exc:
        phase_error = f"{type(exc).__name__}:{exc}"
    finally:
        started_names = {item["name"] for item in started_commands}
        for name in ("slam", "traversability", "navigation"):
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
        if teleop is not None:
            teleop_precleanup_returncode = teleop.poll()
            teleop.stop()
            teleop_cleanup = dict(teleop.cleanup)
            process_cleanup.append(teleop_cleanup)
        if by_name["navigation"].poll() is None:
            stop_result = _run_control(
                binaries["navigation_control"],
                ["stop", "teleop_avoid_acceptance_cleanup"],
                domain_id=domain_id,
                timeout_s=8.0,
            )
        for process in reversed(processes):
            returncode = process.poll()
            if (
                process.name in {"slam", "traversability", "navigation"}
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
    last_traversability = native._load_json(artifacts["traversability_status"])
    terrain_risk = dict(last_traversability.get("terrain_risk") or {})
    teleop_delivery = teleop.snapshot() if teleop is not None else {}
    blockers = list(evaluation.get("blockers") or [])
    if not prior_cleanup_ok:
        blockers.append("prior_owned_process_cleanup_failed")
    if not startup_ok:
        blockers.append(startup_reason)
    if phase_error:
        blockers.append("case_runtime_error")
    if startup_ok and not driving_ready:
        blockers.append(driving_ready_reason)
    if sensor_report.get("ok") is not True:
        blockers.append("sensor_or_slam_acceptance_failed")
    if sensor_report.get("policy_loaded") is not True:
        blockers.append("thunderv4_policy_not_loaded")
    if sensor_report.get("command_source") != "dds":
        blockers.append("mujoco_command_source_not_dds")
    if int((sensor_report.get("cmd_vel") or {}).get("samples") or 0) <= 0:
        blockers.append("typed_dds_cmd_vel_tap_empty")
    if startup_ok and (
        teleop_probe_result.get("returncode") != 0
        or "accepted teleop" not in str(teleop_probe_result.get("stdout") or "")
    ):
        blockers.append("typed_teleop_command_ack_failed")
    continuous_exit = continuous_teleop_exit_blocker(teleop_precleanup_returncode)
    if continuous_exit:
        blockers.append(continuous_exit)
    delivery_blocker = typed_teleop_delivery_blocker(
        teleop_delivery,
        product_gate_eligible=not odom_prior_diagnostic,
    )
    if delivery_blocker:
        blockers.append(delivery_blocker)
    if stop_result and stop_result.get("returncode") != 0:
        blockers.append("native_cleanup_stop_failed")
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
    blockers = list(dict.fromkeys(str(value) for value in blockers))
    report = {
        "schema_version": "lingtu.mujoco.teleop_avoid_native_case.v1",
        "scenario": scenario,
        "ok": not blockers,
        "evidence_scope": ("simulation_pose_prior_diagnostic" if odom_prior_diagnostic else "product_e2e"),
        "product_gate_eligible": not odom_prior_diagnostic,
        "odom_prior_diagnostic": {
            "enabled": odom_prior_diagnostic,
            "derived_slam_config": (str(paths["slam_config"]) if odom_prior_diagnostic else ""),
            "publishes_simulation_pose_prior": odom_prior_diagnostic,
            "allow_kinematic_fastlio_acceptance": odom_prior_diagnostic,
        },
        "control_ingress_coverage": {
            "tested": "lingtu_nav_control typed DDS request/application-ack",
            "not_covered": [
                "Gateway WebSocket teleop ingress",
                "Python CmdVelMux (mutually exclusive with native endpoint modes)",
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
        "startup": {"ok": startup_ok, "reason": startup_reason},
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
            "name": "thunder_field_teleop_avoid",
            "source": "scripts/deploy/thunder/lingtu-{nav,traversability}-dds.service",
            "parameters": dict(FIELD_TELEOP_AVOID_PROFILE),
        },
        "slam_track_against_map_period_s": float(
            (manifest.get("runtime_tolerances") or {}).get("track_against_map_period_s")
            or FIELD_TELEOP_AVOID_PROFILE["track_against_map_period_s"]
        ),
        "terrain_producer_observation": terrain_risk,
        "observed_reason": observed_reason,
        "events": events,
        "signal_events": signal_events,
        "injection": {
            "used": scenario.endswith("_injected"),
            "scope": "dds_consumer_contract_injected" if scenario.endswith("_injected") else "not_used",
            "result": injection_result,
        },
        "native_stop": stop_result,
        "typed_teleop": {
            "probe": teleop_probe_result,
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
    scenarios = _requested_scenarios(getattr(args, "scenario", None))
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

    blockers = list(dict.fromkeys(blockers))
    failure_reason = next(
        (
            str((case.get("failure") or {}).get("reason") or "")
            for case in cases
            if str((case.get("failure") or {}).get("reason") or "")
        ),
        "",
    )
    report = {
        "schema_version": SCHEMA_VERSION,
        "ok": preflight["ok"] and not blockers and all(case.get("ok") is True for case in cases),
        "mode": "teleop_avoid",
        "evidence_scope": (
            "simulation_pose_prior_diagnostic" if bool(getattr(args, "odom_prior_diagnostic", False)) else "product_e2e"
        ),
        "product_gate_eligible": not bool(getattr(args, "odom_prior_diagnostic", False)),
        "control_ingress_coverage": {
            "tested": "native CLI typed DDS request/application-ack",
            "not_covered": "Gateway WebSocket ingress",
        },
        "diagnostic_overrides": {
            "realtime_factor": getattr(args, "realtime_factor", None),
            "track_against_map_period_s": getattr(args, "track_against_map_period_s", None),
        },
        "terrain_producer_contract": dict(TERRAIN_PRODUCER_CONTRACT),
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
    if name == "free" and "accepted" not in reasons:
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
    accepted_ratio = reasons.count("accepted") / len(reasons) if reasons else 0.0
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

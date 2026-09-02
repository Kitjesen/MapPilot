#!/usr/bin/env python3
"""Run and summarize LingTu simulation diagnostics."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import platform
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
for import_path in (ROOT, SRC):
    import_path_text = str(import_path)
    if import_path_text not in sys.path:
        sys.path.insert(0, import_path_text)

from lingtu.control import ProductControl  # noqa: E402
from runtime.algorithm_gates import (  # noqa: E402
    DIMOS_BENCHMARK_REQUIRED_GATES,
    G4_SERVER_FULL_SIM_REQUIRED_GATES,
    INSPECTION_MVP_REQUIRED_GATES,
)

MID360_PATTERN_REL = "sim/packages/sensors/livox/mid360/assets/mid360.npy"
MUJOCO_WORLD_ASSET_REL = "sim/packages/worlds/industrial_park/physics/industrial_park_scene.xml"
NATIVE_SLAM_RUNTIME_REL = "build/slam_core/slamd"
NATIVE_SLAM_CONTROL_REL = "build/slam_core/slamctl"
NATIVE_SLAM_CONFIG_REL = "src/localization/fastlio2/config/sim_mid360_slam.yaml"
NATIVE_DDS_SENSOR_PUBLISHER_REL = "build/mujoco_native_dds/lingtu_mujoco_sensor_publisher"
NAVIGATION_REPLAY_DEVIATION_REPORT_REL = "artifacts/sim_diagnostics/navigation_replay_deviation/report.json"
NAVIGATION_REPLAY_DEVIATION_TRACE_REL = "artifacts/sim_diagnostics/navigation_replay_deviation/trace.json"

LOCAL_NON_MOTION_HOST_REQUIREMENTS = (
    "local Python runtime only",
    "must preserve simulation_only=true, real_robot_motion=false, cmd_vel_sent_to_hardware=false",
)
NATIVE_MUJOCO_LOCALIZATION_HOST_REQUIREMENTS = (
    "MuJoCo native DDS sensor feed available in an isolated native CycloneDDS domain",
    f"official MID-360 scan pattern asset available ({MID360_PATTERN_REL})",
    f"product MuJoCo world asset available ({MUJOCO_WORLD_ASSET_REL})",
    f"native localization runtime available ({NATIVE_SLAM_RUNTIME_REL})",
    f"native localization control available ({NATIVE_SLAM_CONTROL_REL})",
    f"native localization config available ({NATIVE_SLAM_CONFIG_REL})",
    f"native DDS sensor publisher available ({NATIVE_DDS_SENSOR_PUBLISHER_REL})",
    "explicit saved-map PCD input available",
    "no physical robot drivers or hardware command publishers",
)
HOST_CHECK_ACTIONS = {
    "local_non_motion": "local non-motion checks are already host-safe",
    "mujoco_headless": (
        "install the MuJoCo Python runtime and use EGL or OSMesa headless rendering before running MuJoCo gates"
    ),
    "mid360_pattern": ("sync the official MID-360 scan pattern asset before running MuJoCo LiDAR gates"),
    "mujoco_world_asset": ("sync the product MuJoCo world asset before running MuJoCo simulation gates"),
    "native_localization_runtime": (
        "build slamd, slamctl, and the native DDS sensor publisher before saved-map relocalization gates"
    ),
    "isolated_dds_domain": (
        "set LINGTU_DDS_DOMAIN_ID to a nonzero isolated simulation domain before launching native localization gates"
    ),
    "saved_map_input": "provide an existing saved-map PCD with --saved-map-pcd PATH",
}
HOST_CHECK_PRIORITY = (
    "mujoco_headless",
    "mid360_pattern",
    "mujoco_world_asset",
    "isolated_dds_domain",
    "saved_map_input",
    "native_localization_runtime",
)


@dataclass(frozen=True)
class GateSpec:
    """Describe one report-backed simulation gate."""

    name: str
    description: str
    default_patterns: tuple[str, ...]
    evaluator: Callable[[dict[str, Any]], tuple[bool, list[str], dict[str, Any]]]
    host_requirements: tuple[str, ...] = ()
    expected_report_path: str = ""
    requires_saved_map: bool = False
    requires_dds_domain: bool = False


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


def _extend_unique(blockers: list[str], additions: list[str] | tuple[str, ...]) -> None:
    for item in additions:
        if item not in blockers:
            blockers.append(item)


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

    return (
        not blockers,
        blockers,
        {
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
        },
    )


def _eval_gateway_runtime_acceptance(
    report: dict[str, Any],
) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    checks = report.get("checks") if isinstance(report.get("checks"), dict) else {}
    gateway_contract = checks.get("gateway_contract") if isinstance(checks.get("gateway_contract"), dict) else {}
    observability = (
        checks.get("gateway_observability")
        if isinstance(checks.get("gateway_observability"), dict)
        else {}
    )
    motion = checks.get("motion") if isinstance(checks.get("motion"), dict) else {}
    stage_evidence = checks.get("stage_evidence") if isinstance(checks.get("stage_evidence"), dict) else {}

    if report.get("schema_version") != "lingtu.gateway_runtime_acceptance.v3":
        blockers.append("schema_version is not lingtu.gateway_runtime_acceptance.v3")
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

    if observability.get("ok") is not True:
        blockers.append("gateway_observability.ok is not true")
    if observability.get("ros2_topic_required") is True:
        blockers.append("Gateway acceptance must not require ros2 topic")
    if observability.get("arbitrary_publish_supported") is not False:
        blockers.append("arbitrary_publish_supported is not false")
    if _safe_int(observability.get("command_interface_count")) <= 0:
        blockers.append("command_interface_count is missing")
    if motion.get("ok") is not True:
        blockers.append("motion.ok is not true")

    missing_command_interfaces = [str(item) for item in observability.get("missing_command_interfaces") or []]
    unexpected_command_interfaces = [str(item) for item in observability.get("unexpected_command_interfaces") or []]
    missing_topics = [str(item) for item in observability.get("missing_topics") or []]
    non_observable_topics = [str(item) for item in observability.get("non_observable_topics") or []]
    missing_stream_interfaces = [str(item) for item in observability.get("missing_stream_interfaces") or []]
    missing_live_topics = [str(item) for item in observability.get("missing_live_topics") or []]
    if missing_command_interfaces:
        blockers.append("missing Gateway command interfaces: " + ", ".join(missing_command_interfaces))
    if unexpected_command_interfaces:
        blockers.append("unexpected Gateway command interfaces: " + ", ".join(unexpected_command_interfaces))
    if missing_topics:
        blockers.append("missing product runtime topics: " + ", ".join(missing_topics))
    if non_observable_topics:
        blockers.append("non-observable product runtime topics: " + ", ".join(non_observable_topics))
    if missing_stream_interfaces:
        blockers.append("missing Gateway SSE stream interfaces: " + ", ".join(missing_stream_interfaces))
    if missing_live_topics and report.get("mode") != "non_motion":
        blockers.append("missing live Module samples: " + ", ".join(missing_live_topics))

    if stage_evidence.get("ok") is not True:
        blockers.append("stage_evidence.ok is not true")
    if _safe_int(stage_evidence.get("stage_count")) <= 0:
        blockers.append("stage_evidence.stage_count is missing")
    missing_stages = [str(item) for item in stage_evidence.get("missing_stages") or []]
    not_live_stages = [str(item) for item in stage_evidence.get("not_live_stages") or []]
    missing_tokens = (
        stage_evidence.get("missing_tokens") if isinstance(stage_evidence.get("missing_tokens"), dict) else {}
    )
    if missing_stages:
        blockers.append("missing runtime stages: " + ", ".join(missing_stages))
    if not_live_stages and report.get("mode") != "non_motion":
        blockers.append("non-live runtime stages: " + ", ".join(not_live_stages))
    if missing_tokens and report.get("mode") != "non_motion":
        blockers.append(
            "runtime stages missing required inputs: " + ", ".join(sorted(str(key) for key in missing_tokens))
        )

    observable_topics = [str(item) for item in observability.get("observable_topics") or []]
    streamable_topics = [str(item) for item in observability.get("streamable_topics") or []]
    return (
        not blockers,
        blockers,
        {
            "mode": report.get("mode"),
            "runtime_contract": report.get("runtime_contract") or observability.get("runtime_contract"),
            "ros2_topic_required": report.get("ros2_topic_required"),
            "motion_kind": motion.get("kind"),
            "motion_owner": motion.get("motion_owner"),
            "arbitrary_publish_supported": observability.get("arbitrary_publish_supported"),
            "command_interface_count": observability.get("command_interface_count"),
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
        },
    )


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
        report.get("map_metadata_contract") if isinstance(report.get("map_metadata_contract"), dict) else {}
    )
    map_metadata_checks = (
        map_metadata_contract.get("checks") if isinstance(map_metadata_contract.get("checks"), dict) else {}
    )
    if map_metadata_contract.get("ok") is not True:
        blockers.append("map_metadata_contract.ok is not true")
    for name, ok in map_metadata_checks.items():
        if ok is not True:
            blockers.append(f"map_metadata_contract.{name} is not true")

    relocalization = (
        report.get("relocalization")
        if isinstance(report.get("relocalization"), dict)
        else {}
    )
    global_reloc = report.get("global_relocalization_requested") is True
    if relocalization.get("available") is not True:
        blockers.append(
            "typed-DDS global relocalization response was not available"
            if global_reloc
            else "typed-DDS seeded relocalization response was not available"
        )
    if relocalization.get("success") is not True:
        message = str(relocalization.get("message") or "").lower()
        if not (global_reloc and "already running" in message):
            blockers.append(
                "typed-DDS global relocalization did not succeed"
                if global_reloc
                else "typed-DDS seeded relocalization did not succeed"
            )

    sensor_feed = report.get("sensor_feed") if isinstance(report.get("sensor_feed"), dict) else {}
    if sensor_feed.get("ok") is not True:
        blockers.append("sensor_feed.ok is not true")
    if not str(sensor_feed.get("report") or ""):
        blockers.append("sensor_feed.report missing")

    localization = report.get("localization") if isinstance(report.get("localization"), dict) else {}
    thresholds = report.get("thresholds") if isinstance(report.get("thresholds"), dict) else {}
    min_points = int(thresholds.get("min_saved_map_points") or 1000)
    min_tracking = int(thresholds.get("min_tracking_health_samples") or 1)
    max_xy = _safe_float(thresholds.get("max_map_odom_xy_m"), default=5.0)
    max_z = _safe_float(thresholds.get("max_map_odom_z_abs_m"), default=2.0)
    if int(localization.get("saved_map_cloud_samples") or 0) <= 0:
        blockers.append("/slam/saved_map_cloud samples missing")
    if int(localization.get("saved_map_cloud_points_latest") or 0) < min_points:
        blockers.append("/slam/saved_map_cloud point count below threshold")
    if int(localization.get("tracking_health_samples") or 0) < min_tracking:
        blockers.append("localization tracking health samples below threshold")
    if str(localization.get("latest_health_state") or "") != "TRACKING":
        blockers.append("localization latest health is not TRACKING")
    if int(localization.get("map_to_odom_tf_samples") or 0) <= 0:
        blockers.append("map->odom TF samples missing")
    map_xy = _safe_float(localization.get("map_to_odom_xy_m"), default=999.0)
    if map_xy > max_xy:
        blockers.append("map->odom XY correction exceeds threshold")
    map_z = _safe_float(localization.get("map_to_odom_z_abs_m"), default=999.0)
    if map_z > max_z:
        blockers.append("map->odom Z correction exceeds threshold")
    if global_reloc:
        if report.get("global_relocalization_validated") is not True:
            blockers.append("global_relocalization_validated is not true")
        if localization.get("bbs3d_success_observed") is not True:
            blockers.append("BBS3D success was not observed")
        if localization.get("bbs3d_disabled_observed") is True:
            blockers.append("BBS3D disabled state was observed")
        min_global_xy = _safe_float(thresholds.get("min_global_map_odom_xy_m"), default=0.0)
        if min_global_xy <= 0.0:
            blockers.append("kidnapped map->odom minimum correction is not positive")
        elif map_xy < min_global_xy:
            blockers.append("kidnapped map->odom XY correction below threshold")
        expected_start = localization.get("kidnap_start_xyz")
        observed_start = sensor_feed.get("start_anchor_xyz")
        try:
            expected_start_xyz = [
                float(expected_start[axis]) for axis in ("x", "y", "z")
            ]
            observed_start_xyz = [float(observed_start[index]) for index in range(3)]
        except (KeyError, IndexError, TypeError, ValueError):
            expected_start_xyz = []
            observed_start_xyz = []
        if (
            not expected_start_xyz
            or not observed_start_xyz
            or math.dist(expected_start_xyz, observed_start_xyz) > 0.05
        ):
            blockers.append("kidnapped sensor start position evidence is missing or inconsistent")

    return (
        not blockers,
        blockers,
        {
            "validation_level": report.get("validation_level"),
            "runtime_stage": report.get("runtime_stage"),
            "map_dependency": report.get("map_dependency"),
            "runtime_relocalization_executed": report.get("runtime_relocalization_executed"),
            "runtime_relocalization_validated": report.get("runtime_relocalization_validated"),
            "global_relocalization_requested": global_reloc,
            "global_relocalization_validated": report.get("global_relocalization_validated"),
            "relocalization": relocalization,
            "sensor_feed": sensor_feed,
            "localization": localization,
            "thresholds": thresholds,
            "map_pcd": report.get("map_pcd"),
            "artifacts": {
                "map_pcd": map_pcd_artifact,
                "map_metadata_contract": map_metadata_contract,
            },
        },
    )


GATES: tuple[GateSpec, ...] = (
    GateSpec(
        name="gateway_runtime_acceptance",
        description="Gateway Runtime Data Plane acceptance through ModulePort streams, stage evidence, and command whitelist",
        default_patterns=(
            "artifacts/sim_diagnostics/gateway_runtime_acceptance/report.json",
            "artifacts/gateway_runtime_acceptance*/report.json",
        ),
        evaluator=_eval_gateway_runtime_acceptance,
        host_requirements=LOCAL_NON_MOTION_HOST_REQUIREMENTS,
    ),
    GateSpec(
        name="navigation_replay_deviation",
        description="Offline navigation replay/deviation over recorded global_path, local_path, cmd_vel, and odometry",
        default_patterns=(
            "artifacts/sim_diagnostics/navigation_replay_deviation/report.json",
            "artifacts/navigation_replay_deviation*/report.json",
        ),
        evaluator=_eval_navigation_replay_deviation,
        host_requirements=LOCAL_NON_MOTION_HOST_REQUIREMENTS,
    ),
    GateSpec(
        name="saved_map_relocalize",
        description="Runtime saved-map relocalization with native MuJoCo DDS sensors and slamd",
        default_patterns=(
            "artifacts/sim_diagnostics/saved_map_relocalize_runtime/report.json",
            "artifacts/sim_diagnostics/saved_map_relocalize_runtime*/report.json",
            "artifacts/saved_map_relocalize_runtime*/report.json",
        ),
        evaluator=_eval_saved_map_relocalize,
        host_requirements=NATIVE_MUJOCO_LOCALIZATION_HOST_REQUIREMENTS,
        requires_saved_map=True,
        requires_dds_domain=True,
    ),
    GateSpec(
        name="bbs3d_kidnapped_relocalize",
        description="BBS3D kidnapped-robot global relocalization with native MuJoCo DDS sensors and slamd",
        default_patterns=(
            "artifacts/sim_diagnostics/bbs3d_kidnapped_relocalize/report.json",
            "artifacts/sim_diagnostics/bbs3d_kidnapped_relocalize*/report.json",
        ),
        evaluator=_eval_saved_map_relocalize,
        host_requirements=NATIVE_MUJOCO_LOCALIZATION_HOST_REQUIREMENTS,
        requires_saved_map=True,
        requires_dds_domain=True,
    ),
)

ALGORITHM_PRESETS: dict[str, tuple[str, ...]] = {
    "inspection_mvp": (*INSPECTION_MVP_REQUIRED_GATES,),
    "dimos_benchmark": (*DIMOS_BENCHMARK_REQUIRED_GATES,),
    "g4_server_full_sim": (*G4_SERVER_FULL_SIM_REQUIRED_GATES,),
}


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


def _expected_report_path(spec: GateSpec) -> str:
    if spec.expected_report_path:
        return spec.expected_report_path
    return spec.default_patterns[0] if spec.default_patterns else ""


def _host_requirements_for_gates(names: set[str]) -> dict[str, list[str]]:
    specs = {spec.name: spec for spec in GATES}
    return {name: list(specs[name].host_requirements) for name in _ordered_gate_names(names) if name in specs}


def _missing_reports(
    gates: dict[str, Any],
    missing_or_failed: list[str],
) -> list[dict[str, Any]]:
    specs = {spec.name: spec for spec in GATES}
    reports: list[dict[str, Any]] = []
    for name in _ordered_gate_names(set(missing_or_failed)):
        spec = specs.get(name)
        if spec is None:
            continue
        gate = gates.get(name) or {}
        reports.append(
            {
                "name": name,
                "description": spec.description,
                "status": gate.get("status", "missing"),
                "path": gate.get("path", ""),
                "expected_report_path": _expected_report_path(spec),
                "accepted_patterns": list(spec.default_patterns),
                "host_requirements": list(spec.host_requirements),
            }
        )
    return reports


def _algorithm_validation_summary(
    gates: dict[str, Any],
    required_names: set[str],
    missing_or_failed: list[str],
) -> dict[str, Any]:
    ordered_required = _ordered_gate_names(required_names)
    ordered_missing = _ordered_gate_names(set(missing_or_failed))
    claim_allowed = not missing_or_failed
    stop_condition = (
        "all required gates passed; simulation-only algorithm-health claim is allowed"
        if claim_allowed
        else (
            "do not claim algorithm health until these required gates pass: "
            + ", ".join(ordered_missing)
        )
    )
    validation_flow = [
        {
            "id": name,
            "gates": [name],
            "status": "passed" if (gates.get(name) or {}).get("ok") is True else "failed",
        }
        for name in ordered_required
    ]
    next_actions = [
        {
            "gate": name,
            "status": (gates.get(name) or {}).get("status", "missing"),
            "action_type": "inspect_or_collect_report",
            "blockers": list((gates.get(name) or {}).get("blockers") or ["not verified"]),
            "report_path": (gates.get(name) or {}).get("path", ""),
            "expected_report_path": (gates.get(name) or {}).get("expected_report_path", ""),
        }
        for name in ordered_missing
    ]
    return {
        "claim": "simulation_algorithm_health",
        "claim_allowed": claim_allowed,
        "flow_ok": claim_allowed,
        "highest_priority_blocker": ordered_missing[0] if ordered_missing else "",
        "required_gate_sequence": ordered_required,
        "validation_flow": validation_flow,
        "claim_boundary": {
            "gateway_runtime": "module_port_non_motion_acceptance",
            "navigation_evidence": "offline_topic_replay",
            "localization_runtime": "native_mujoco_dds_and_slamd",
            "relocalization_modes": ["saved_map_seeded", "bbs3d_global_kidnapped"],
            "simulation_only": True,
            "field_readiness": False,
        },
        "blocking_gate_count": len(missing_or_failed),
        "blocking_categories": {"diagnostics": ordered_missing} if ordered_missing else {},
        "gate_categories": {name: ["diagnostics"] for name in ordered_missing},
        "next_actions": next_actions,
        "stop_condition": stop_condition,
        "interpretation": ("simulation/replay evidence only; not physical S100P field readiness"),
    }


def _python_tag() -> str:
    return f"py{sys.version_info.major}{sys.version_info.minor}"


def _default_module_available(name: str) -> bool:
    try:
        return importlib.util.find_spec(name) is not None
    except Exception:
        return False


def _default_path_exists(path: str | Path) -> bool:
    return Path(path).exists()


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
    checks[name] = payload
    if not ok and blocker:
        blockers.append(blocker)


def _failed_host_check_names(checks: dict[str, dict[str, Any]]) -> list[str]:
    return [name for name, check in checks.items() if isinstance(check, dict) and check.get("ok") is False]


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
            "host can run the selected simulation diagnostics"
            if not failed_checks
            else (
                "fix host checks before running blocked simulation diagnostics: "
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
    dds_domain_id: int | str | None = None,
    saved_map_pcd: Path | None = None,
    executable_exists: Callable[[str], bool] | None = None,
    module_available: Callable[[str], bool] | None = None,
    path_exists: Callable[[str | Path], bool] | None = None,
) -> dict[str, Any]:
    """Check whether the current host can collect the selected diagnostics.

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
    has_module = module_available or _default_module_available
    has_path = path_exists or _default_path_exists
    resolved_dds_domain_id = str(
        dds_domain_id if dds_domain_id is not None else resolved_env.get("LINGTU_DDS_DOMAIN_ID") or ""
    ).strip()
    resolved_saved_map_pcd = saved_map_pcd.resolve() if saved_map_pcd is not None else None
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
            _add_check(
                checks,
                blockers,
                "mid360_pattern",
                ok=pattern_exists,
                blocker=(f"official MID-360 scan pattern asset missing: {MID360_PATTERN_REL}"),
                evidence={
                    "path": str(pattern_path),
                    "relative_path": MID360_PATTERN_REL,
                    "exists": pattern_exists,
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
                blocker=(f"product MuJoCo world asset missing: {MUJOCO_WORLD_ASSET_REL}"),
                evidence={
                    "path": str(world_asset_path),
                    "relative_path": MUJOCO_WORLD_ASSET_REL,
                    "exists": world_asset_exists,
                },
            )

        needs_native_dds_isolation = "native CycloneDDS domain" in requirement_text
        if needs_native_dds_isolation:
            dds_domain_ok = (
                resolved_dds_domain_id.isdigit()
                and 1 <= int(resolved_dds_domain_id) <= 231
            )
            _add_check(
                checks,
                blockers,
                "isolated_dds_domain",
                ok=dds_domain_ok,
                blocker=(
                    "LINGTU_DDS_DOMAIN_ID must be set to a nonzero isolated simulation domain "
                    "before running this gate"
                ),
                evidence={"LINGTU_DDS_DOMAIN_ID": resolved_dds_domain_id},
            )

        needs_saved_map_input = (
            spec.requires_saved_map or "explicit saved-map PCD input" in requirement_text
        )
        if needs_saved_map_input:
            saved_map_exists = bool(
                resolved_saved_map_pcd is not None
                and str(resolved_saved_map_pcd).strip()
                and has_path(resolved_saved_map_pcd)
            )
            _add_check(
                checks,
                blockers,
                "saved_map_input",
                ok=saved_map_exists,
                blocker=(
                    "explicit saved-map PCD input is required and must exist; pass --saved-map-pcd PATH"
                ),
                evidence={
                    "path": str(resolved_saved_map_pcd) if resolved_saved_map_pcd else "",
                    "exists": saved_map_exists,
                    "source": "external_input",
                },
            )

        if "native localization runtime" in requirement_text:
            native_artifacts = {
                "slamd": ROOT / NATIVE_SLAM_RUNTIME_REL,
                "slamctl": ROOT / NATIVE_SLAM_CONTROL_REL,
                "slam_config": ROOT / NATIVE_SLAM_CONFIG_REL,
                "sensor_publisher": ROOT / NATIVE_DDS_SENSOR_PUBLISHER_REL,
            }
            artifact_checks = {
                name: bool(has_path(path)) for name, path in native_artifacts.items()
            }
            missing_artifacts = [name for name, present in artifact_checks.items() if not present]
            native_runtime_ok = not missing_artifacts
            _add_check(
                checks,
                blockers,
                "native_localization_runtime",
                ok=native_runtime_ok,
                blocker=(
                    "native localization artifact(s) missing: " + ", ".join(missing_artifacts)
                    if missing_artifacts
                    else ""
                ),
                evidence={
                    "artifacts": {
                        name: {"path": str(path), "exists": artifact_checks[name]}
                        for name, path in native_artifacts.items()
                    },
                    "missing_artifacts": missing_artifacts,
                },
            )

        ok = not blockers
        failed_checks = _failed_host_check_names(checks)
        input_blocked = checks.get("saved_map_input", {}).get("ok") is False
        gates[name] = {
            "ok": ok,
            "status": "runnable" if ok else ("input_blocked" if input_blocked else "blocked"),
            "description": spec.description,
            "host_requirements": list(requirements),
            "checks": checks,
            "failed_checks": failed_checks,
            "blockers": blockers,
            "recommended_action": _host_check_recommended_action(failed_checks),
            "expected_report_path": _expected_report_path(spec),
            "accepted_patterns": list(spec.default_patterns),
        }

    blocked_gates = [name for name in _ordered_gate_names(required_names) if not gates[name]["ok"]]
    input_blocked_gates = [
        name for name in blocked_gates if gates[name]["status"] == "input_blocked"
    ]
    runnable_gates = [name for name in _ordered_gate_names(required_names) if gates[name]["ok"]]
    current_host = {
        "platform_system": resolved_platform,
        "machine": resolved_machine,
        "python_tag": resolved_python_tag,
        "LINGTU_DDS_DOMAIN_ID": resolved_dds_domain_id,
        "saved_map_pcd": str(resolved_saved_map_pcd) if resolved_saved_map_pcd else "",
        "MUJOCO_GL": mujoco_gl,
    }
    return {
        "schema_version": "lingtu.sim_diagnostics.host_preflight.v1",
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
        "generated_at": generated_at,
        "required_gate_sequence": _ordered_gate_names(required_names),
        "current_host": current_host,
        "runnable_gates": runnable_gates,
        "blocked_gates": blocked_gates,
        "input_blocked_gates": input_blocked_gates,
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
) -> Path | None:
    candidates = _candidate_matches(spec.default_patterns)
    return candidates[0] if candidates else None


def summarize(
    *,
    report_overrides: dict[str, Path],
    required: set[str],
    max_report_age_s: float | None = None,
    include_optional: bool = True,
) -> dict[str, Any]:
    """Summarize the selected gate reports without executing gate commands."""

    gates: dict[str, Any] = {}
    generated_at = time.time()
    required_names = required or {spec.name for spec in GATES}
    selected_specs = GATES if include_optional else tuple(spec for spec in GATES if spec.name in required_names)
    for spec in selected_specs:
        spec_max_report_age_s = max_report_age_s
        path = report_overrides.get(spec.name) or _best_match(spec)
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
                "host_requirements": list(spec.host_requirements),
                "is_fresh": False,
            }
            continue
        report_mtime = path.stat().st_mtime
        report_age_s = max(0.0, generated_at - report_mtime)
        freshness_blockers: list[str] = []
        if spec_max_report_age_s is not None and report_age_s > spec_max_report_age_s:
            freshness_blockers.append(f"report_age_s {report_age_s:.3f} > max_report_age_s {spec_max_report_age_s:.3f}")
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
                "host_requirements": list(spec.host_requirements),
            }

    missing_or_failed = [
        name for name in _ordered_gate_names(required_names) if not bool((gates.get(name) or {}).get("ok"))
    ]
    optional_missing_or_failed = [
        name for name in _ordered_gate_names(set(gates) - required_names) if not bool((gates.get(name) or {}).get("ok"))
    ]
    verified = {name: bool(item.get("ok")) for name, item in gates.items()}
    algorithm_backends = _algorithm_backends_from_gates(gates)
    algorithm_validation = _algorithm_validation_summary(
        gates,
        required_names,
        missing_or_failed,
    )
    return {
        "schema_version": "lingtu.sim_diagnostics.v1",
        "execution_mode": "summary_only",
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
        "missing_reports": _missing_reports(gates, missing_or_failed),
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


def run_product_diagnostics(
    product: str,
    *,
    collect: Callable[[], dict[str, Any]],
    control: Any | None = None,
    robot: str | None = None,
    state_dir: Path | None = None,
) -> dict[str, Any]:
    """Start one sim Product, collect diagnostics, and stop it via ProductControl."""

    lifecycle = control or ProductControl(robot=robot, env="sim")
    product_control: dict[str, Any] = {
        "switch": None,
        "status": None,
        "stop": None,
    }
    try:
        switch_report = lifecycle.switch(product, state_dir=state_dir)
    except Exception as exc:
        return {
            "schema_version": "lingtu.sim_diagnostics.v1",
            "execution_mode": "product_diagnostics",
            "ok": False,
            "simulation_only": True,
            "product": product,
            "error": str(exc),
            "product_control": product_control,
        }

    product_control["switch"] = switch_report
    if switch_report.get("ok") is not True:
        return {
            "schema_version": "lingtu.sim_diagnostics.v1",
            "execution_mode": "product_diagnostics",
            "ok": False,
            "simulation_only": True,
            "product": product,
            "error": str(switch_report.get("error") or "Product switch failed"),
            "product_control": product_control,
        }

    diagnostics: dict[str, Any] = {}
    error = ""
    try:
        status = lifecycle.status(state_dir=state_dir)
        product_control["status"] = status
        if status.get("status") != "active" or status.get("product") != product:
            raise RuntimeError(f"ProductControl did not activate {product!r}")
        diagnostics = collect()
        if not isinstance(diagnostics, dict):
            raise TypeError("diagnostic collector must return a dict")
    except Exception as exc:
        error = str(exc)
    finally:
        try:
            product_control["stop"] = lifecycle.stop(state_dir=state_dir)
        except Exception as exc:
            product_control["stop"] = {
                "ok": False,
                "status": "failed",
                "error": str(exc),
            }

    result = dict(diagnostics)
    diagnostics_ok = result.get("ok") is True and not error
    stop_ok = (product_control["stop"] or {}).get("ok") is True
    result.update(
        {
            "schema_version": "lingtu.sim_diagnostics.v1",
            "execution_mode": "product_diagnostics",
            "ok": diagnostics_ok and stop_ok,
            "simulation_only": True,
            "product": product,
            "diagnostics_ok": diagnostics_ok,
            "error": error or str((product_control["stop"] or {}).get("error") or ""),
            "product_control": product_control,
        }
    )
    return result


def materialize_navigation_replay_deviation_topic_jsonl(
    topic_jsonl_path: Path,
    *,
    report_path: Path | None = None,
    trace_path: Path | None = None,
) -> tuple[Path, dict[str, Any]]:
    """Build a navigation replay/deviation report from a recorded topic JSONL."""
    from sim.evaluation.navigation_replay import build_report

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


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gateway-runtime-acceptance-report", type=Path, default=None)
    parser.add_argument("--navigation-replay-deviation-report", type=Path, default=None)
    parser.add_argument(
        "--navigation-replay-deviation-topic-jsonl",
        type=Path,
        default=None,
        help=("Build navigation_replay_deviation evidence directly from recorded topic JSONL before summarizing."),
    )
    parser.add_argument("--saved-map-relocalize-report", type=Path, default=None)
    parser.add_argument("--bbs3d-kidnapped-relocalize-report", type=Path, default=None)
    parser.add_argument(
        "--dds-domain-id",
        type=int,
        default=None,
        help="Native DDS domain checked by host preflight (1..231).",
    )
    parser.add_argument(
        "--saved-map-pcd",
        type=Path,
        default=None,
        help="Explicit external saved-map PCD consumed by saved-map and BBS3D gates.",
    )
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
            "Named report set. These gates provide bounded non-motion, replay, and "
            "localization evidence; they do not prove patrol completion or full simulation."
        ),
    )
    parser.add_argument(
        "--max-report-age-s",
        type=float,
        default=None,
        help="Optional maximum age in seconds for accepted report artifacts.",
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
        "--required-only",
        action="store_true",
        help="Evaluate only gates listed by --required; useful for setup-safe subset summaries.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=ROOT / "artifacts/sim_diagnostics/summary.json",
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
    """Run the simulation diagnostics CLI."""

    args = _build_parser().parse_args()
    overrides = {
        "gateway_runtime_acceptance": args.gateway_runtime_acceptance_report,
        "navigation_replay_deviation": args.navigation_replay_deviation_report,
        "saved_map_relocalize": args.saved_map_relocalize_report,
        "bbs3d_kidnapped_relocalize": args.bbs3d_kidnapped_relocalize_report,
    }
    required = _required_from_args(args)
    valid_names = {spec.name for spec in GATES}
    unknown = sorted(required - valid_names)
    if unknown:
        raise SystemExit(f"unknown required gate(s): {', '.join(unknown)}")
    if args.navigation_replay_deviation_report and args.navigation_replay_deviation_topic_jsonl:
        raise SystemExit(
            "--navigation-replay-deviation-report cannot be combined with --navigation-replay-deviation-topic-jsonl"
        )
    if args.host_preflight and args.navigation_replay_deviation_topic_jsonl:
        raise SystemExit("--navigation-replay-deviation-topic-jsonl cannot be combined with --host-preflight")

    materialized_inputs: list[dict[str, Any]] = []
    if args.navigation_replay_deviation_topic_jsonl is not None:
        report_path, materialized = materialize_navigation_replay_deviation_topic_jsonl(
            args.navigation_replay_deviation_topic_jsonl
        )
        overrides["navigation_replay_deviation"] = report_path
        materialized_inputs.append(materialized)
    summary_kwargs = {
        "report_overrides": {key: value for key, value in overrides.items() if value is not None},
        "required": required,
        "max_report_age_s": args.max_report_age_s,
        "include_optional": not args.required_only,
    }
    if args.host_preflight:
        summary = host_preflight(
            required=required,
            dds_domain_id=args.dds_domain_id,
            saved_map_pcd=args.saved_map_pcd,
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

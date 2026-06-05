#!/usr/bin/env python3
"""Run Fast-LIO2 on live MuJoCo LiDAR/IMU and verify LingTu bridge outputs.

This gate is simulation-only. It does not connect to robot services and does
not publish hardware commands. A headless MuJoCo engine generates raw
`/points_raw` and `/imu_raw`; Fast-LIO2 consumes those topics and publishes
real algorithm outputs, which are then fed into `SlamBridgeModule`.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import subprocess
import sys
import time
import struct
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Sequence

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core.runtime_evidence import validate_runtime_evidence
from core.runtime_interface import (
    FRAME_LINKS,
    TOPICS,
    adapter_source_for_target,
    lidar_extrinsic,
    resolved_runtime_data_flow,
    simulator_world_frame_id,
    topic_default_frame_id,
)
from drivers.sim.mujoco_live_runtime import (
    DEFAULT_MID360_PATTERN,
    DEFAULT_MID360_SAMPLES_PER_FRAME,
    build_engine as _build_engine,
    parse_start as _parse_start,
    resolve_mid360_pattern as _resolve_mid360_pattern,
    resolve_world as _resolve_world,
    scene_start as _scene_start,
)
from drivers.sim.mujoco_lingtu_stack import (
    build_fastlio2_frontier_stack,
    build_fastlio2_inspection_stack,
    build_fastlio2_tare_stack,
)
from drivers.sim.mujoco_sensor_bridge import (
    angle_delta_rad as _angle_delta_rad,
    make_imu_msg as _make_imu_msg,
    make_livox_custom_msg as _make_livox_custom_msg,
    make_odom_body_tf as _make_odom_body_tf,
    make_pointcloud2 as _make_pointcloud2,
    make_sim_odometry_msg as _make_sim_odometry_msg,
    make_transform_msg as _make_transform_msg,
    sensor_xyzi_to_body_xyzi as _sensor_xyzi_to_body_xyzi,
    specific_force_body as _specific_force_body,
    world_xyzi_to_sensor_xyzi as _world_xyzi_to_sensor_xyzi,
    yaw_from_quat_xyzw as _yaw_from_quat_xyzw,
)
from core.same_source_map_artifacts import (
    add_points_to_voxel_store as _add_points_to_voxel_store,
    write_same_source_map_artifacts as _write_same_source_map_artifacts,
)
from slam.fastlio2_live_bridge import (
    FastLio2Process,
    ensure_ros2_cli,
    tail_file as _tail,
    write_fastlio2_config as _write_fastlio2_config,
)
from slam.fastlio2_nav_bridge import FastLio2NavBridgeRuntime

SIM_WORLD_FRAME_ID = simulator_world_frame_id()
SIM_MAP_FRAME_ID = FRAME_LINKS["map_to_odom"].parent
SIM_ODOM_FRAME_ID = FRAME_LINKS["map_to_odom"].child
SIM_BODY_FRAME_ID = FRAME_LINKS["odom_to_body"].child
SIM_LIDAR_FRAME_ID = FRAME_LINKS["body_to_lidar"].child
SIM_NAV_ODOMETRY_FRAME_ID = topic_default_frame_id(TOPICS.odometry)
SIM_NAV_REGISTERED_CLOUD_FRAME_ID = topic_default_frame_id(TOPICS.registered_cloud)
# Fast-LIO live map clouds remain in local odom until a localizer owns map->odom.
SIM_FASTLIO_LIVE_MAP_FRAME_ID = SIM_NAV_ODOMETRY_FRAME_ID
FASTLIO_REGISTERED_CLOUD_TOPIC = adapter_source_for_target(
    "fastlio2",
    TOPICS.registered_cloud,
)
FASTLIO_MAP_CLOUD_TOPIC = adapter_source_for_target("fastlio2", TOPICS.map_cloud)
FASTLIO_ODOMETRY_TOPIC = adapter_source_for_target("fastlio2", TOPICS.odometry)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _mujoco_fastlio_contract_definition() -> tuple[dict[str, Any], list[str]]:
    try:
        from core.blueprints.simulation_contract import simulation_runtime_contract

        return simulation_runtime_contract("mujoco_fastlio2_live").as_report(), []
    except Exception as exc:
        return {}, [f"{type(exc).__name__}: {exc}"]


def _mujoco_frame_evidence(
    *,
    odom_body_samples: int,
    odom_body_source: str,
    static_tf_published: bool,
) -> dict[str, dict[str, Any]]:
    return {
        "map_to_odom": {
            "ok": bool(static_tf_published),
            "parent": FRAME_LINKS["map_to_odom"].parent,
            "child": FRAME_LINKS["map_to_odom"].child,
            "static": bool(static_tf_published),
            "source": "static_tf_broadcaster" if static_tf_published else "not_published",
        },
        "odom_to_body": {
            "ok": int(odom_body_samples) > 0,
            "parent": FRAME_LINKS["odom_to_body"].parent,
            "child": FRAME_LINKS["odom_to_body"].child,
            "samples": int(odom_body_samples),
            "source": odom_body_source,
        },
        "body_to_lidar": {
            "ok": bool(static_tf_published),
            "parent": FRAME_LINKS["body_to_lidar"].parent,
            "child": FRAME_LINKS["body_to_lidar"].child,
            "static": bool(static_tf_published),
            "source": "static_tf_broadcaster" if static_tf_published else "not_published",
        },
    }


def _mujoco_hardware_safety() -> dict[str, Any]:
    return {
        "blocked_hardware_nodes": [],
        "unexpected_command_publishers": [],
        "topics": {TOPICS.cmd_vel: ["/mujoco_velocity_adapter"]},
    }


def _mujoco_data_flow_evidence(
    *,
    topic_evidence: dict[str, dict[str, Any]],
    navigation_required: bool,
    gate_exception: bool = False,
) -> dict[str, dict[str, Any]]:
    runtime_flow = resolved_runtime_data_flow("mujoco_fastlio2_live")
    stages = {stage.name: stage for stage in runtime_flow}

    def topic_ok(topic: str) -> bool:
        return (topic_evidence.get(topic) or {}).get("ok") is True

    def stage_report(
        name: str,
        *,
        ok: bool,
        required: bool,
        reason: str = "",
    ) -> dict[str, Any]:
        stage = stages[name]
        return {
            "ok": bool(ok),
            "required": bool(required),
            "inputs": list(stage.inputs),
            "outputs": list(stage.outputs),
            "owner": stage.owner,
            "frame_role": stage.frame_role,
            "map_dependency": stage.map_dependency,
            "reason": reason,
        }

    if gate_exception:
        return {
            stage.name: stage_report(
                stage.name,
                ok=False,
                required=True,
                reason="gate_exception",
            )
            for stage in runtime_flow
        }

    raw_ok = topic_ok(TOPICS.raw_lidar_points) and topic_ok(TOPICS.raw_imu)
    slam_ok = raw_ok and topic_ok(TOPICS.odometry) and topic_ok(TOPICS.map_cloud)
    planning_ok = topic_ok(TOPICS.global_path) and topic_ok(TOPICS.local_path)
    command_ok = topic_ok(TOPICS.cmd_vel)
    optional_reason = "" if navigation_required else "not_required_for_basic_slam_gate"
    return {
        "endpoint_adapter": stage_report(
            "endpoint_adapter",
            ok=raw_ok,
            required=True,
        ),
        "slam_or_relayed_localization_map": stage_report(
            "slam_or_relayed_localization_map",
            ok=slam_ok,
            required=True,
        ),
        "map_layers_and_exploration": stage_report(
            "map_layers_and_exploration",
            ok=(topic_ok(TOPICS.map_cloud) if navigation_required else False),
            required=navigation_required,
            reason=optional_reason,
        ),
        "global_planning": stage_report(
            "global_planning",
            ok=(topic_ok(TOPICS.global_path) if navigation_required else False),
            required=navigation_required,
            reason=optional_reason,
        ),
        "local_planning_and_following": stage_report(
            "local_planning_and_following",
            ok=(planning_ok and command_ok if navigation_required else False),
            required=navigation_required,
            reason=optional_reason,
        ),
        "command_boundary": stage_report(
            "command_boundary",
            ok=(command_ok if navigation_required else False),
            required=navigation_required,
            reason=optional_reason,
        ),
    }


def _mujoco_runtime_contract(
    *,
    definition: dict[str, Any],
    definition_errors: list[str],
    topic_evidence: dict[str, dict[str, Any]],
    frame_evidence: dict[str, dict[str, Any]],
    data_flow_evidence: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    required_topics = (
        definition.get("required_runtime_topics")
        if isinstance(definition, dict)
        else ()
    ) or ()
    required_slam_topics = (
        definition.get("required_slam_topics")
        if isinstance(definition, dict)
        else ()
    ) or ()
    return {
        "name": "mujoco_fastlio2_live",
        "ok": (
            not definition_errors
            and all((topic_evidence.get(topic) or {}).get("ok") is True for topic in required_topics)
            and all((topic_evidence.get(topic) or {}).get("ok") is True for topic in required_slam_topics)
            and all((item or {}).get("ok") is True for item in frame_evidence.values())
            and all(
                (item or {}).get("ok") is True
                or (item or {}).get("required") is False
                for item in data_flow_evidence.values()
            )
        ),
        "definition": definition,
        "topic_evidence": topic_evidence,
        "frame_evidence": frame_evidence,
        "data_flow_evidence": data_flow_evidence,
        "publisher_identity": {
            "blocked_hardware_nodes": [],
            "unexpected_command_publishers": [],
        },
        "errors": definition_errors,
    }


def _runtime_evidence_report(result: Any) -> dict[str, Any]:
    return {
        "ok": bool(result.ok),
        "blockers": list(result.blockers),
        "frame_links_required": True,
        "data_flow_required": True,
    }


def _exception_lidar_source(args: argparse.Namespace) -> dict[str, Any]:
    pattern = Path(str(getattr(args, "mid360_pattern", "") or ""))
    pattern_exists = pattern.is_file()
    try:
        pattern_sha256 = _sha256_file(pattern) if pattern_exists else ""
    except Exception:
        pattern_sha256 = ""
    return {
        "kind": (
            "MuJoCo mj_multiRay with official Livox MID-360 scan pattern"
            if pattern_exists
            else "MuJoCo raw LiDAR source did not initialize"
        ),
        "fastlio_lidar_input": getattr(args, "fastlio_lidar_input", ""),
        "scan_time_profile": getattr(args, "scan_time_profile", ""),
        "forced_pattern": pattern_exists,
        "pattern_path": str(pattern) if pattern else "",
        "pattern_sha256": pattern_sha256,
        "samples_per_frame": int(getattr(args, "mid360_samples_per_frame", 0) or 0),
        "fallback_n_rays": int(getattr(args, "n_rays", 0) or 0),
    }


def _gate_exception_report(args: argparse.Namespace, exc: Exception) -> dict[str, Any]:
    definition, definition_errors = _mujoco_fastlio_contract_definition()
    required_topics = list((definition or {}).get("required_runtime_topics") or ())
    required_slam_topics = list((definition or {}).get("required_slam_topics") or ())
    topic_evidence = {
        topic: {"ok": False, "samples": 0, "reason": "gate_exception"}
        for topic in dict.fromkeys([*required_topics, *required_slam_topics])
    }
    frame_evidence = _mujoco_frame_evidence(
        odom_body_samples=0,
        odom_body_source="gate_exception",
        static_tf_published=False,
    )
    data_flow_evidence = _mujoco_data_flow_evidence(
        topic_evidence=topic_evidence,
        navigation_required=True,
        gate_exception=True,
    )
    runtime_contract = _mujoco_runtime_contract(
        definition=definition,
        definition_errors=definition_errors,
        topic_evidence=topic_evidence,
        frame_evidence=frame_evidence,
        data_flow_evidence=data_flow_evidence,
    )
    hardware_safety = _mujoco_hardware_safety()
    runtime_evidence = validate_runtime_evidence(
        {
            "runtime_contract": runtime_contract,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "outputs": {},
            "hardware_safety": hardware_safety,
        },
        "mujoco_fastlio2_live",
        require_paths=False,
        require_command=False,
        require_frame_links=True,
        require_data_flow=True,
    )
    runtime_fault = f"{type(exc).__name__}: {exc}"
    return {
        "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
        "ok": False,
        "remaining_gaps": [
            f"gate_exception: {runtime_fault}",
            *list(runtime_evidence.blockers),
        ],
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "duration_clock": args.duration_clock,
        "scan_time_profile": args.scan_time_profile,
        "runtime_faults": [runtime_fault],
        "runtime_contract": runtime_contract,
        "runtime_evidence": _runtime_evidence_report(runtime_evidence),
        "hardware_safety": hardware_safety,
        "lidar_source": _exception_lidar_source(args),
        "args": vars(args),
    }


_DEGENERACY_RE = re.compile(
    r"DEGENERACY DETECTED:\s*(?P<dof>\d+)\s*/\s*(?P<total>\d+)"
    r".*?cond=(?P<cond>[0-9.eE+-]+).*?eff_ratio=(?P<eff>[0-9.eE+-]+)"
)
_IEKF_RE = re.compile(
    r"IEKF did not converge:.*?cond=(?P<cond>[0-9.eE+-]+)"
)


def _fastlio2_log_diagnostics(path: Path) -> dict[str, Any]:
    """Summarize Fast-LIO observability warnings from the node log."""

    diagnostics: dict[str, Any] = {
        "log_path": str(path),
        "log_exists": bool(path.exists()),
        "degeneracy_warning_count": 0,
        "iekf_nonconverged_count": 0,
        "max_condition_number": 0.0,
        "max_degenerate_dof_count": 0,
        "min_effective_ratio": None,
        "latest_warnings": [],
    }
    if not path.exists():
        return diagnostics
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except Exception as exc:
        diagnostics["read_error"] = f"{type(exc).__name__}: {exc}"
        return diagnostics

    latest: list[str] = []
    min_eff: float | None = None
    max_cond = 0.0
    max_dof = 0
    degen_count = 0
    iekf_count = 0
    for line in lines:
        degen = _DEGENERACY_RE.search(line)
        iekf = _IEKF_RE.search(line)
        if degen is not None:
            degen_count += 1
            cond = float(degen.group("cond"))
            eff = float(degen.group("eff"))
            dof = int(degen.group("dof"))
            max_cond = max(max_cond, cond)
            max_dof = max(max_dof, dof)
            min_eff = eff if min_eff is None else min(min_eff, eff)
            latest.append(line)
        elif iekf is not None:
            iekf_count += 1
            max_cond = max(max_cond, float(iekf.group("cond")))
            latest.append(line)

    diagnostics.update(
        {
            "degeneracy_warning_count": degen_count,
            "iekf_nonconverged_count": iekf_count,
            "max_condition_number": round(max_cond, 4),
            "max_degenerate_dof_count": max_dof,
            "min_effective_ratio": round(min_eff, 4) if min_eff is not None else None,
            "latest_warnings": latest[-8:],
        }
    )
    return diagnostics


def _wall_timeout_status(elapsed_wall_s: float, max_wall_time_s: float) -> dict[str, Any]:
    elapsed = max(0.0, float(elapsed_wall_s))
    limit = max(0.0, float(max_wall_time_s or 0.0))
    triggered = bool(limit > 0.0 and elapsed >= limit)
    return {
        "enabled": bool(limit > 0.0),
        "triggered": triggered,
        "elapsed_wall_s": round(elapsed, 3),
        "max_wall_time_s": float(limit),
        "fault": (
            f"gate wall timeout after {elapsed:.1f}s (limit={limit:.1f}s)"
            if triggered
            else ""
        ),
    }


def _round_float(value: Any, digits: int = 4) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return round(result, digits)


def _round_xyz(value: Any) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return None
    rounded = [_round_float(value[index]) for index in range(3)]
    if any(item is None for item in rounded):
        return None
    return [float(item) for item in rounded]


def _path_xyz(point: Any) -> list[float] | None:
    pose = getattr(point, "pose", point)
    position = getattr(pose, "position", pose)
    if all(hasattr(position, attr) for attr in ("x", "y")):
        return [
            float(getattr(position, "x")),
            float(getattr(position, "y")),
            float(getattr(position, "z", 0.0)),
        ]
    if isinstance(position, dict):
        try:
            return [
                float(position["x"]),
                float(position["y"]),
                float(position.get("z", 0.0)),
            ]
        except (KeyError, TypeError, ValueError):
            return None
    if isinstance(position, (list, tuple)) and len(position) >= 2:
        try:
            z_value = position[2] if len(position) >= 3 else 0.0
            return [float(position[0]), float(position[1]), float(z_value)]
        except (TypeError, ValueError):
            return None
    return None


def _path_summary(path: Any, *, max_points: int = 6) -> dict[str, Any]:
    poses = list(getattr(path, "poses", path) or [])
    points = [point for point in (_path_xyz(item) for item in poses) if point is not None]
    finite_points = [
        point
        for point in points
        if all(math.isfinite(float(value)) for value in point[:3])
    ]
    length_m = 0.0
    for prev, cur in zip(finite_points[:-1], finite_points[1:]):
        length_m += math.dist(prev[:3], cur[:3])
    if not finite_points:
        return {
            "point_count": len(poses),
            "finite_point_count": 0,
            "path_length_m": 0.0,
            "first_xyz": None,
            "last_xyz": None,
            "points_head": [],
            "points_tail": [],
        }
    xs = [point[0] for point in finite_points]
    ys = [point[1] for point in finite_points]
    return {
        "point_count": len(poses),
        "finite_point_count": len(finite_points),
        "path_length_m": round(float(length_m), 4),
        "first_xyz": _round_xyz(finite_points[0]),
        "last_xyz": _round_xyz(finite_points[-1]),
        "bounds_xy": [
            round(float(min(xs)), 4),
            round(float(min(ys)), 4),
            round(float(max(xs)), 4),
            round(float(max(ys)), 4),
        ],
        "points_head": [_round_xyz(point) for point in finite_points[:max_points]],
        "points_tail": [_round_xyz(point) for point in finite_points[-max_points:]],
    }


def _degeneracy_detail_sample(data: Any, *, stamp_s: float | None = None) -> dict[str, Any] | None:
    values = list(data or [])
    if len(values) < 14:
        return None
    dof_mask = [_round_float(values[5 + index]) for index in range(6)]
    if any(item is None for item in dof_mask):
        return None
    sample = {
        "condition_number": _round_float(values[0]),
        "min_eigenvalue": _round_float(values[1]),
        "max_eigenvalue": _round_float(values[2]),
        "effective_ratio": _round_float(values[3]),
        "degenerate_dof_count": int(float(values[4])),
        "dof_mask": [float(item) for item in dof_mask],
        "pos_cov_trace": _round_float(values[11]),
        "ieskf_iter_num": int(float(values[12])),
        "ieskf_converged": bool(float(values[13]) >= 0.5),
    }
    if stamp_s is not None:
        sample["stamp_s"] = _round_float(stamp_s, 3)
    return sample


def _summarize_degeneracy_detail_samples(samples: list[dict[str, Any]]) -> dict[str, Any]:
    valid = [sample for sample in samples if isinstance(sample, dict)]
    max_condition = 0.0
    min_effective: float | None = None
    max_degenerate = 0
    max_pos_cov = 0.0
    max_iter = 0
    nonconverged = 0
    tx_degenerate = 0
    ty_degenerate = 0
    tz_degenerate = 0
    for sample in valid:
        condition = _round_float(sample.get("condition_number")) or 0.0
        effective = _round_float(sample.get("effective_ratio"))
        pos_cov = _round_float(sample.get("pos_cov_trace")) or 0.0
        max_condition = max(max_condition, condition)
        if effective is not None:
            min_effective = effective if min_effective is None else min(min_effective, effective)
        max_degenerate = max(
            max_degenerate,
            int(float(sample.get("degenerate_dof_count") or 0)),
        )
        max_pos_cov = max(max_pos_cov, pos_cov)
        max_iter = max(max_iter, int(float(sample.get("ieskf_iter_num") or 0)))
        nonconverged += int(sample.get("ieskf_converged") is False)
        mask = sample.get("dof_mask")
        if isinstance(mask, (list, tuple)) and len(mask) >= 6:
            tx_degenerate += int(float(mask[3]) < 0.5)
            ty_degenerate += int(float(mask[4]) < 0.5)
            tz_degenerate += int(float(mask[5]) < 0.5)
    return {
        "sample_count": len(valid),
        "max_condition_number": round(max_condition, 4),
        "min_effective_ratio": min_effective,
        "max_degenerate_dof_count": max_degenerate,
        "tx_degenerate_count": tx_degenerate,
        "ty_degenerate_count": ty_degenerate,
        "tz_degenerate_count": tz_degenerate,
        "iekf_nonconverged_count": nonconverged,
        "max_pos_cov_trace": round(max_pos_cov, 4),
        "max_ieskf_iter_num": max_iter,
        "last_sample": valid[-1] if valid else None,
        "samples_tail": valid[-8:],
    }


def _update_runtime_fault_streak(
    streaks: dict[str, int],
    *,
    kind: str,
    confirm_samples: int,
) -> dict[str, Any]:
    required = max(1, int(confirm_samples))
    normalized = str(kind or "")
    if not normalized:
        for key in list(streaks):
            streaks[key] = 0
        return {"kind": "", "streak": 0, "confirmed": False}
    for key in list(streaks):
        if key != normalized:
            streaks[key] = 0
    streaks[normalized] = int(streaks.get(normalized, 0)) + 1
    streak = int(streaks[normalized])
    return {
        "kind": normalized,
        "streak": streak,
        "confirmed": bool(streak >= required),
    }


def _nearest_sim_pose_sample(
    samples: list[tuple[float, float, float, float, float]],
    *,
    target_sim_time_s: float,
    max_dt_s: float = 0.25,
) -> dict[str, Any] | None:
    if not samples:
        return None
    target = float(target_sim_time_s)
    best = min(samples, key=lambda item: abs(float(item[0]) - target))
    dt = abs(float(best[0]) - target)
    if dt > float(max_dt_s):
        return None
    return {
        "sim_time_s": float(best[0]),
        "xyz": [float(best[1]), float(best[2]), float(best[3])],
        "yaw": float(best[4]),
        "dt_s": dt,
    }


def _navigation_diagnostic_sample(
    *,
    sim_time_s: float,
    wall_time_s: float,
    first_sim_xyz: list[float] | None,
    current_sim_xyz: list[float] | None,
    first_sim_yaw: float | None,
    current_sim_yaw: float | None,
    first_odom_xyz: list[float] | None,
    current_odom_xyz: list[float] | None,
    first_odom_yaw: float | None,
    current_odom_yaw: float | None,
    latest_nav_cmd: dict[str, float],
    now_s: float,
    cmd_vel_timeout_s: float,
    command_fresh: bool,
    global_path_counts: list[int],
    local_path_counts: list[int],
    waypoint_count: int,
    navigation_health: dict[str, Any],
    runtime_faults: list[str],
) -> dict[str, Any]:
    sim_z_delta = None
    fastlio_z_delta = None
    fastlio_z_delta_error = None
    if first_sim_xyz is not None and current_sim_xyz is not None:
        sim_z_delta = float(current_sim_xyz[2]) - float(first_sim_xyz[2])
    if first_odom_xyz is not None and current_odom_xyz is not None:
        fastlio_z_delta = float(current_odom_xyz[2]) - float(first_odom_xyz[2])
    if sim_z_delta is not None and fastlio_z_delta is not None:
        fastlio_z_delta_error = abs(fastlio_z_delta - sim_z_delta)

    fastlio_yaw_delta_error = None
    if (
        first_sim_yaw is not None
        and current_sim_yaw is not None
        and first_odom_yaw is not None
        and current_odom_yaw is not None
    ):
        sim_yaw_delta = _angle_delta_rad(current_sim_yaw, first_sim_yaw)
        fastlio_yaw_delta = _angle_delta_rad(current_odom_yaw, first_odom_yaw)
        fastlio_yaw_delta_error = abs(_angle_delta_rad(fastlio_yaw_delta, sim_yaw_delta))

    cmd_stamp = float(latest_nav_cmd.get("stamp") or 0.0)
    cmd_age_s = max(0.0, float(now_s) - cmd_stamp) if cmd_stamp > 0.0 else None
    linear_norm = math.hypot(
        float(latest_nav_cmd.get("vx") or 0.0),
        float(latest_nav_cmd.get("vy") or 0.0),
    )
    last_plan = (
        navigation_health.get("last_plan_report")
        if isinstance(navigation_health.get("last_plan_report"), dict)
        else {}
    )
    return {
        "sim_time_s": round(float(sim_time_s), 3),
        "wall_time_s": round(float(wall_time_s), 3),
        "sim_xyz": _round_xyz(current_sim_xyz),
        "fastlio2_xyz": _round_xyz(current_odom_xyz),
        "fastlio2_z_delta_error_m": _round_float(fastlio_z_delta_error),
        "fastlio2_yaw_delta_error_rad": _round_float(fastlio_yaw_delta_error),
        "nav_cmd": {
            "fresh": bool(command_fresh),
            "age_s": _round_float(cmd_age_s),
            "timeout_s": round(float(cmd_vel_timeout_s), 3),
            "linear_norm": _round_float(linear_norm),
            "angular_z": _round_float(latest_nav_cmd.get("wz")),
        },
        "navigation": {
            "state": str(navigation_health.get("state") or ""),
            "patrol_index": int(navigation_health.get("patrol_index") or 0),
            "patrol_total": int(navigation_health.get("patrol_total") or 0),
            "failure_reason": str(navigation_health.get("failure_reason") or ""),
            "primary_planner": str(last_plan.get("primary_planner") or ""),
            "selected_planner": str(last_plan.get("selected_planner") or ""),
            "fallback_reason": str(last_plan.get("fallback_reason") or ""),
        },
        "paths": {
            "global_path_count": len(global_path_counts),
            "global_path_points_latest": int(global_path_counts[-1]) if global_path_counts else 0,
            "global_path_points_max": max(global_path_counts) if global_path_counts else 0,
            "local_path_count": len(local_path_counts),
            "local_path_points_latest": int(local_path_counts[-1]) if local_path_counts else 0,
            "local_path_points_max": max(local_path_counts) if local_path_counts else 0,
            "waypoint_count": int(waypoint_count),
        },
        "runtime_fault_count": len(runtime_faults),
        "latest_runtime_fault": runtime_faults[-1] if runtime_faults else "",
    }


def _control_quality_report(
    *,
    applied_cmd_stats: dict[str, float | int],
    max_yaw_per_meter: float,
    max_angular_saturation_ratio: float,
) -> dict[str, object]:
    linear_m = float(applied_cmd_stats.get("linear_distance_integral_m") or 0.0)
    angular_rad = float(applied_cmd_stats.get("angular_abs_integral_rad") or 0.0)
    cmd_samples = int(
        applied_cmd_stats.get("cmd_samples")
        or applied_cmd_stats.get("samples")
        or 0
    )
    sat_samples = int(applied_cmd_stats.get("angular_saturation_samples") or 0)
    yaw_per_meter = angular_rad / max(linear_m, 1e-6)
    saturation_ratio = sat_samples / max(cmd_samples, 1)
    blockers: list[str] = []
    if yaw_per_meter > float(max_yaw_per_meter):
        blockers.append("yaw_per_meter too high")
    if saturation_ratio > float(max_angular_saturation_ratio):
        blockers.append("angular saturation ratio too high")
    return {
        "ok": not blockers,
        "blockers": blockers,
        "linear_distance_integral_m": round(linear_m, 4),
        "angular_abs_integral_rad": round(angular_rad, 4),
        "yaw_per_meter": round(float(yaw_per_meter), 4),
        "angular_saturation_ratio": round(float(saturation_ratio), 4),
        "max_yaw_per_meter": float(max_yaw_per_meter),
        "max_angular_saturation_ratio": float(max_angular_saturation_ratio),
    }


def _dynamic_obstacle_sweep_quality(
    *,
    cases: list[dict[str, object]],
    required_densities: tuple[int, ...],
    required_speeds: tuple[float, ...],
) -> dict[str, object]:
    blockers: list[str] = []
    passed = [
        case
        for case in cases
        if case.get("collision") is False and case.get("runtime_evidence_ok") is True
    ]
    covered_densities = {int(case["density"]) for case in passed if "density" in case}
    covered_speeds = {
        round(float(case["speed_mps"]), 3)
        for case in passed
        if "speed_mps" in case
    }
    for density in required_densities:
        if int(density) not in covered_densities:
            blockers.append(f"missing passed density {density}")
    for speed in required_speeds:
        normalized_speed = round(float(speed), 3)
        if normalized_speed not in covered_speeds:
            blockers.append(f"missing passed speed {speed}")
    return {
        "ok": not blockers,
        "blockers": blockers,
        "covered_density_count": len(covered_densities),
        "covered_speed_count": len(covered_speeds),
        "passed_case_count": len(passed),
    }


def _finite_values(items: Sequence[Any]) -> list[float]:
    values: list[float] = []
    for item in items:
        value = _round_float(item, 9)
        if value is not None:
            values.append(float(value))
    return values


def _fastlio_large_loop_diagnostic_report(
    *,
    segment_consistency: Sequence[dict[str, Any]],
    imu_samples: Sequence[dict[str, Any]],
    scan_relative_times_s: Sequence[Any],
    scan_time_profile: str,
    command_samples: Sequence[dict[str, Any]],
) -> dict[str, Any]:
    """Summarize diagnostics that explain, but do not decide, large-loop failures."""

    valid_segments = [item for item in segment_consistency if isinstance(item, dict)]
    z_errors = _finite_values(item.get("z_delta_error_m") for item in valid_segments)
    yaw_errors = _finite_values(item.get("yaw_delta_error_rad") for item in valid_segments)
    worst_segment: dict[str, Any] = {}
    if valid_segments:
        worst_segment = max(
            valid_segments,
            key=lambda item: _round_float(item.get("z_delta_error_m")) or -1.0,
        )

    valid_imu = [item for item in imu_samples if isinstance(item, dict)]
    imu_dt = _finite_values(item.get("dt_s") for item in valid_imu)
    imu_acc = _finite_values(item.get("acc_norm") for item in valid_imu)
    imu_gyro = _finite_values(item.get("gyro_norm") for item in valid_imu)
    imu_gyro_z = _finite_values(item.get("gyro_z_radps") for item in valid_imu)
    imu_gyro_z_integral = 0.0
    for item in valid_imu:
        gyro_z = _round_float(item.get("gyro_z_radps"), 9)
        dt_s = _round_float(item.get("dt_s"), 9)
        if gyro_z is not None and dt_s is not None:
            imu_gyro_z_integral += float(gyro_z) * float(dt_s)

    scan_times = _finite_values(scan_relative_times_s)
    scan_deltas = [
        scan_times[index] - scan_times[index - 1]
        for index in range(1, len(scan_times))
    ]

    valid_commands = [item for item in command_samples if isinstance(item, dict)]
    linear_x = _finite_values(item.get("linear_x") for item in valid_commands)
    linear_y = _finite_values(item.get("linear_y") for item in valid_commands)
    angular_z = _finite_values(item.get("angular_z") for item in valid_commands)
    command_angular_integral = 0.0
    previous_command_time: float | None = None
    previous_angular_z: float | None = None
    for item in valid_commands:
        sim_time = _round_float(item.get("sim_time_s"), 9)
        wz = _round_float(item.get("angular_z"), 9)
        if sim_time is None or wz is None:
            continue
        if previous_command_time is not None and previous_angular_z is not None:
            dt_s = max(0.0, float(sim_time) - float(previous_command_time))
            command_angular_integral += float(previous_angular_z) * dt_s
        previous_command_time = float(sim_time)
        previous_angular_z = float(wz)
    sources = sorted(
        {
            str(item.get("source") or "")
            for item in valid_commands
            if str(item.get("source") or "")
        }
    )
    linear_norms: list[float] = []
    for index, vx in enumerate(linear_x):
        vy = linear_y[index] if index < len(linear_y) else 0.0
        linear_norms.append(float(math.hypot(vx, vy)))
    yaw_input_consistency_checked = (
        abs(float(command_angular_integral)) > 0.05
        and abs(float(imu_gyro_z_integral)) > 0.05
    )
    yaw_input_consistency_ok = bool(
        yaw_input_consistency_checked
        and math.copysign(1.0, float(command_angular_integral))
        == math.copysign(1.0, float(imu_gyro_z_integral))
    )

    return {
        "schema_version": "lingtu.fastlio_large_loop_diagnostics.v1",
        "diagnostic_only": True,
        "segment_consistency": {
            "sample_count": len(valid_segments),
            "max_z_delta_error_m": max(z_errors) if z_errors else None,
            "max_yaw_delta_error_rad": max(yaw_errors) if yaw_errors else None,
            "worst_segment": worst_segment,
            "samples_tail": valid_segments[-8:],
        },
        "imu_statistics": {
            "sample_count": len(valid_imu),
            "mean_dt_s": float(sum(imu_dt) / len(imu_dt)) if imu_dt else None,
            "max_dt_s": max(imu_dt) if imu_dt else None,
            "max_acc_norm": max(imu_acc) if imu_acc else None,
            "mean_acc_norm": float(sum(imu_acc) / len(imu_acc)) if imu_acc else None,
            "max_gyro_norm": max(imu_gyro) if imu_gyro else None,
            "mean_gyro_norm": float(sum(imu_gyro) / len(imu_gyro)) if imu_gyro else None,
            "max_gyro_z_radps": max(imu_gyro_z) if imu_gyro_z else None,
            "min_gyro_z_radps": min(imu_gyro_z) if imu_gyro_z else None,
            "mean_gyro_z_radps": (
                float(sum(imu_gyro_z) / len(imu_gyro_z)) if imu_gyro_z else None
            ),
            "gyro_z_signed_integral_rad": float(imu_gyro_z_integral),
            "positive_gyro_z_samples": sum(1 for value in imu_gyro_z if value > 1e-4),
            "negative_gyro_z_samples": sum(1 for value in imu_gyro_z if value < -1e-4),
        },
        "scan_timing_statistics": {
            "profile": str(scan_time_profile or ""),
            "point_count": len(scan_times),
            "min_time_s": min(scan_times) if scan_times else None,
            "max_time_s": max(scan_times) if scan_times else None,
            "span_s": (max(scan_times) - min(scan_times)) if scan_times else None,
            "mean_delta_s": (
                float(sum(scan_deltas) / len(scan_deltas)) if scan_deltas else None
            ),
            "max_delta_s": max(scan_deltas) if scan_deltas else None,
        },
        "command_trajectory_summary": {
            "sample_count": len(valid_commands),
            "source": sources[0] if len(sources) == 1 else ("mixed" if sources else ""),
            "sources": sources,
            "max_linear_x": max(linear_x) if linear_x else None,
            "min_linear_x": min(linear_x) if linear_x else None,
            "max_linear_norm": max(linear_norms) if linear_norms else None,
            "max_angular_z": max(angular_z) if angular_z else None,
            "max_abs_angular_z": max((abs(value) for value in angular_z), default=None),
            "angular_signed_integral_from_samples_rad": float(command_angular_integral),
            "samples_tail": valid_commands[-12:],
        },
        "yaw_input_consistency": {
            "checked": bool(yaw_input_consistency_checked),
            "ok": yaw_input_consistency_ok,
            "command_angular_signed_integral_rad": float(command_angular_integral),
            "imu_gyro_z_signed_integral_rad": float(imu_gyro_z_integral),
        },
    }


def _load_ros_modules(*, require_livox: bool = True):
    try:
        import rclpy  # type: ignore
        from geometry_msgs.msg import TransformStamped  # type: ignore
        from nav_msgs.msg import Odometry  # type: ignore
        from sensor_msgs.msg import Imu, PointCloud2, PointField  # type: ignore
        from tf2_ros import (  # type: ignore
            StaticTransformBroadcaster,
            TransformBroadcaster,
        )
    except Exception as exc:  # pragma: no cover - depends on ROS2 env
        raise RuntimeError(
            "ROS2 Python modules are unavailable. Source /opt/ros/humble/setup.bash "
            "and the LingTu install/setup.bash before running this gate."
        ) from exc
    CustomMsg = None
    CustomPoint = None
    try:
        from livox_ros_driver2.msg import CustomMsg, CustomPoint  # type: ignore
    except Exception as exc:  # pragma: no cover - depends on ROS2 env
        if require_livox:
            raise RuntimeError(
                "livox_ros_driver2 Python messages are unavailable. Build/source "
                "livox_ros_driver2, or run this gate with "
                "--fastlio-lidar-input timed_pointcloud2 for a ROS environment "
                "without Livox CustomMsg support."
            ) from exc
    return (
        rclpy,
        Imu,
        Odometry,
        PointCloud2,
        PointField,
        CustomMsg,
        CustomPoint,
        TransformStamped,
        TransformBroadcaster,
        StaticTransformBroadcaster,
    )


def _stamp_from_seconds(stamp_s: float, time_cls: Any):
    msg = time_cls()
    sec = math.floor(stamp_s)
    msg.sec = int(sec)
    msg.nanosec = int(round((stamp_s - sec) * 1e9))
    if msg.nanosec >= 1_000_000_000:
        msg.sec += 1
        msg.nanosec -= 1_000_000_000
    return msg


def _relative_times_for_scan(
    point_count: int,
    lidar_period_s: float,
    *,
    scan_time_profile: str,
) -> np.ndarray:
    """Return per-point times matching how this validation scan was produced."""

    count = max(0, int(point_count))
    profile = str(scan_time_profile or "synthetic_rolling").strip().lower()
    if profile == "instantaneous":
        return np.zeros(count, dtype=np.float32)
    if profile == "synthetic_rolling":
        return np.linspace(
            0.0,
            float(lidar_period_s),
            num=count,
            endpoint=False,
            dtype=np.float32,
        )
    raise ValueError(f"unsupported scan_time_profile: {scan_time_profile}")


def _coerce_xyzi_cloud(points: Any, *, default_intensity: float = 100.0) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32)
    if pts.size == 0:
        return np.zeros((0, 4), dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] < 3:
        raise ValueError(f"expected point cloud shape (N, >=3), got {pts.shape}")
    if pts.shape[1] < 4:
        intensity = np.full((len(pts), 1), float(default_intensity), dtype=np.float32)
        return np.hstack([pts[:, :3], intensity]).astype(np.float32, copy=False)
    return pts[:, :4].astype(np.float32, copy=False)


def _physical_rolling_scan_from_samples(
    samples: Sequence[tuple[float, np.ndarray, np.ndarray, int]],
    *,
    scan_start_s: float,
    scan_end_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, int, int]:
    """Build one scan from subscans captured at their actual simulation times."""

    start = float(scan_start_s)
    end = float(scan_end_s)
    eps = 1e-6
    sensor_chunks: list[np.ndarray] = []
    world_chunks: list[np.ndarray] = []
    time_chunks: list[np.ndarray] = []
    moving_point_count = 0
    selected_subscans = 0
    for sim_time_s, cloud_sensor, cloud_world, moving_count in samples:
        t = float(sim_time_s)
        if t < start - eps or t > end + eps:
            continue
        sensor_pts = np.asarray(cloud_sensor, dtype=np.float32)
        world_pts = np.asarray(cloud_world, dtype=np.float32)
        if sensor_pts.size == 0 or sensor_pts.ndim != 2 or sensor_pts.shape[1] < 4:
            continue
        if world_pts.size == 0 or world_pts.ndim != 2 or world_pts.shape[1] < 4:
            world_pts = np.zeros((len(sensor_pts), 4), dtype=np.float32)
        sensor_chunks.append(sensor_pts[:, :4])
        world_chunks.append(world_pts[:, :4])
        relative_t = max(0.0, min(float(end - start), t - start))
        time_chunks.append(np.full(len(sensor_pts), relative_t, dtype=np.float32))
        moving_point_count += max(0, int(moving_count))
        selected_subscans += 1
    if not sensor_chunks:
        return (
            np.zeros((0, 4), dtype=np.float32),
            np.zeros((0, 4), dtype=np.float32),
            np.zeros(0, dtype=np.float32),
            0,
            0,
        )
    return (
        np.vstack(sensor_chunks).astype(np.float32, copy=False),
        np.vstack(world_chunks).astype(np.float32, copy=False),
        np.concatenate(time_chunks).astype(np.float32, copy=False),
        int(moving_point_count),
        int(selected_subscans),
    )


def _odom_xyz(msg: Any) -> list[float] | None:
    try:
        p = msg.pose.pose.position
        return [float(p.x), float(p.y), float(p.z)]
    except Exception:
        return None


def _odom_yaw(msg: Any) -> float | None:
    try:
        q = msg.pose.pose.orientation
        return _yaw_from_quat_xyzw([q.x, q.y, q.z, q.w])
    except Exception:
        return None


def _pointcloud_xy_stats(msg: Any) -> dict[str, Any]:
    """Return count and XY bounding area for a ROS2 PointCloud2 message."""
    try:
        n_pts = int(msg.width) * int(msg.height)
        if n_pts <= 0:
            return {"point_count": 0, "xy_area_m2": 0.0}
        raw = bytes(msg.data)
        step = int(msg.point_step)
        fields = {str(field.name).lower(): int(field.offset) for field in msg.fields}
        ox = fields.get("x")
        oy = fields.get("y")
        if ox is None or oy is None:
            return {"point_count": n_pts, "xy_area_m2": 0.0, "error": "missing_xy_fields"}

        if step == 16 and ox == 0 and oy == 4:
            arr = np.frombuffer(raw, dtype=np.float32, count=n_pts * 4).reshape(n_pts, 4)
            xy = arr[:, :2]
        else:
            xy = np.zeros((n_pts, 2), dtype=np.float32)
            for idx in range(n_pts):
                base = idx * step
                xy[idx, 0] = struct.unpack_from("<f", raw, base + ox)[0]
                xy[idx, 1] = struct.unpack_from("<f", raw, base + oy)[0]

        finite = np.isfinite(xy).all(axis=1)
        xy = xy[finite]
        if xy.shape[0] <= 0:
            return {"point_count": 0, "xy_area_m2": 0.0}
        mins = np.min(xy, axis=0)
        maxs = np.max(xy, axis=0)
        span = np.maximum(maxs - mins, 0.0)
        return {
            "point_count": int(xy.shape[0]),
            "min_x": float(mins[0]),
            "min_y": float(mins[1]),
            "max_x": float(maxs[0]),
            "max_y": float(maxs[1]),
            "xy_area_m2": float(span[0] * span[1]),
        }
    except Exception as exc:
        return {"point_count": 0, "xy_area_m2": 0.0, "error": str(exc)}


def _pointcloud_xyz_sample(msg: Any, max_points: int = 4000) -> np.ndarray:
    """Extract a bounded xyz sample from a PointCloud2 message."""
    try:
        n_pts = int(msg.width) * int(msg.height)
        if n_pts <= 0:
            return np.empty((0, 3), dtype=np.float32)
        raw = bytes(msg.data)
        step = int(msg.point_step)
        fields = {str(field.name).lower(): int(field.offset) for field in msg.fields}
        ox = fields.get("x")
        oy = fields.get("y")
        oz = fields.get("z")
        if ox is None or oy is None or oz is None:
            return np.empty((0, 3), dtype=np.float32)

        if step == 16 and ox == 0 and oy == 4 and oz == 8:
            xyz = np.frombuffer(raw, dtype=np.float32, count=n_pts * 4).reshape(n_pts, 4)[:, :3]
        else:
            xyz = np.zeros((n_pts, 3), dtype=np.float32)
            for idx in range(n_pts):
                base = idx * step
                xyz[idx, 0] = struct.unpack_from("<f", raw, base + ox)[0]
                xyz[idx, 1] = struct.unpack_from("<f", raw, base + oy)[0]
                xyz[idx, 2] = struct.unpack_from("<f", raw, base + oz)[0]
        finite = np.isfinite(xyz).all(axis=1)
        xyz = xyz[finite]
        return _sample_points(xyz, max_points)
    except Exception:
        return np.empty((0, 3), dtype=np.float32)


def _sample_points(points: np.ndarray, max_points: int) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[0] <= 0:
        return np.empty((0, 3), dtype=np.float32)
    if pts.shape[1] < 3:
        return np.empty((0, 3), dtype=np.float32)
    pts = pts[:, :3]
    if pts.shape[0] <= max_points:
        return pts.copy()
    step = max(1, int(np.ceil(pts.shape[0] / max_points)))
    return pts[::step][:max_points].copy()


def _render_mujoco_overview(engine: Any, state: dict[str, Any]) -> np.ndarray | None:
    """Render a third-person MuJoCo frame for validation videos.

    This is visual evidence only; sensor and SLAM validation still comes from
    the raw LiDAR/IMU and Fast-LIO topics.
    """
    if state.get("disabled"):
        return None
    try:
        import mujoco  # type: ignore

        model = getattr(engine, "_model", None)
        data = getattr(engine, "_data", None)
        if model is None or data is None:
            return None
        renderer = state.get("renderer")
        camera = state.get("camera")
        if renderer is None or camera is None:
            renderer = mujoco.Renderer(model, 360, 640)
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.distance = 16.0
            camera.azimuth = -55.0
            camera.elevation = -58.0
            state["renderer"] = renderer
            state["camera"] = camera
        robot_state = engine.get_robot_state()
        robot_pos = np.asarray(robot_state.position[:3], dtype=np.float64)
        target = np.array([robot_pos[0], robot_pos[1], max(robot_pos[2], 0.5) + 0.8], dtype=np.float64)
        prev = state.get("lookat")
        if prev is None:
            lookat = target
        else:
            lookat = np.asarray(prev, dtype=np.float64) * 0.8 + target * 0.2
        state["lookat"] = lookat
        camera.lookat[:] = lookat
        renderer.update_scene(data, camera=camera)
        return renderer.render().copy()
    except Exception as exc:
        state["disabled"] = True
        state["error"] = f"{type(exc).__name__}: {exc}"
        return None


def _write_stage_video(
    samples: list[dict[str, Any]],
    report: dict[str, Any],
    out_path: Path,
    *,
    fps: float = 8.0,
) -> None:
    """Render an offline acceptance video from the same live gate samples."""
    if not samples:
        return
    try:
        import cv2  # type: ignore
    except Exception as exc:
        report["video_error"] = f"opencv unavailable: {exc}"
        return

    out_path.parent.mkdir(parents=True, exist_ok=True)
    width, height = 1280, 720
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(out_path), fourcc, max(float(fps), 1.0), (width, height))
    if not writer.isOpened():
        report["video_error"] = f"failed to open video writer: {out_path}"
        return

    def finite_series(key: str) -> np.ndarray:
        values = np.asarray(
            [sample.get(key, [np.nan, np.nan]) for sample in samples],
            dtype=np.float32,
        )
        if values.ndim != 2 or values.shape[1] < 2:
            return np.empty((0, 2), dtype=np.float32)
        values = values[:, :2]
        return values[np.isfinite(values).all(axis=1)]

    sim_series = finite_series("sim_xy")
    odom_series = finite_series("odom_xy")

    def finite_series_xyz(key: str) -> np.ndarray:
        values = np.asarray(
            [sample.get(key, [np.nan, np.nan, np.nan]) for sample in samples],
            dtype=np.float32,
        )
        if values.ndim != 2 or values.shape[1] < 3:
            return np.empty((0, 3), dtype=np.float32)
        values = values[:, :3]
        return values[np.isfinite(values).all(axis=1)]

    sim_xyz_series = finite_series_xyz("sim_xyz")
    odom_xyz_series = finite_series_xyz("odom_xyz")
    z_offset = 0.0
    if len(sim_xyz_series) > 0 and len(odom_xyz_series) > 0:
        z_offset = float(odom_xyz_series[0, 2] - sim_xyz_series[0, 2])

    def fit_sim_to_odom() -> dict[str, Any]:
        count = min(len(sim_series), len(odom_series))
        if count <= 0:
            return {
                "method": "identity",
                "rotation_rad": 0.0,
                "translation_xy": [0.0, 0.0],
                "rmse_m": None,
                "max_error_m": None,
                "sample_count": 0,
            }
        src = np.asarray(sim_series[:count], dtype=np.float64)
        dst = np.asarray(odom_series[:count], dtype=np.float64)
        finite = np.isfinite(src).all(axis=1) & np.isfinite(dst).all(axis=1)
        src = src[finite]
        dst = dst[finite]
        if len(src) < 3 or float(np.linalg.norm(src[-1] - src[0])) < 0.2:
            translation = dst[0] - src[0]
            transformed = src + translation
            errors = np.linalg.norm(transformed - dst, axis=1)
            return {
                "method": "translation_only",
                "rotation_rad": 0.0,
                "translation_xy": [float(translation[0]), float(translation[1])],
                "rmse_m": float(np.sqrt(np.mean(errors * errors))) if len(errors) else None,
                "max_error_m": float(np.max(errors)) if len(errors) else None,
                "sample_count": int(len(src)),
            }
        src_center = np.mean(src, axis=0)
        dst_center = np.mean(dst, axis=0)
        src0 = src - src_center
        dst0 = dst - dst_center
        h = src0.T @ dst0
        u, _s, vt = np.linalg.svd(h)
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1, :] *= -1.0
            rotation = vt.T @ u.T
        translation = dst_center - rotation @ src_center
        transformed = (rotation @ src.T).T + translation
        errors = np.linalg.norm(transformed - dst, axis=1)
        return {
            "method": "se2_best_fit",
            "rotation_rad": float(math.atan2(rotation[1, 0], rotation[0, 0])),
            "translation_xy": [float(translation[0]), float(translation[1])],
            "rmse_m": float(np.sqrt(np.mean(errors * errors))),
            "max_error_m": float(np.max(errors)),
            "sample_count": int(len(src)),
        }

    alignment = fit_sim_to_odom()
    theta = float(alignment.get("rotation_rad") or 0.0)
    c, s = math.cos(theta), math.sin(theta)
    alignment_rotation = np.asarray([[c, -s], [s, c]], dtype=np.float32)
    alignment_translation = np.asarray(
        alignment.get("translation_xy") or [0.0, 0.0],
        dtype=np.float32,
    )

    def transform_sim_xy(xy: Any) -> np.ndarray:
        point = np.asarray(xy, dtype=np.float32)
        if point.shape != (2,) or not np.isfinite(point).all():
            return np.array([np.nan, np.nan], dtype=np.float32)
        return alignment_rotation @ point + alignment_translation

    def transform_sim_xyz(xyz: Any) -> np.ndarray:
        point = np.asarray(xyz, dtype=np.float32)
        if point.shape != (3,) or not np.isfinite(point).all():
            return np.array([np.nan, np.nan, np.nan], dtype=np.float32)
        xy = alignment_rotation @ point[:2] + alignment_translation
        return np.array([xy[0], xy[1], point[2] + z_offset], dtype=np.float32)

    def transform_sim_points(points: Any) -> np.ndarray:
        pts = np.asarray(points, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[0] <= 0 or pts.shape[1] < 2:
            return np.empty((0, 3), dtype=np.float32)
        out = pts[:, :3].copy()
        finite = np.isfinite(out[:, :2]).all(axis=1)
        out = out[finite]
        if out.shape[0] <= 0:
            return np.empty((0, 3), dtype=np.float32)
        out[:, :2] = out[:, :2] @ alignment_rotation.T + alignment_translation
        if out.shape[1] >= 3:
            out[:, 2] += float(z_offset)
        return out

    report["video_alignment"] = alignment
    report["video_alignment"]["z_offset_m"] = float(z_offset)

    def bounds_for(key: str) -> tuple[float, float, float, float]:
        chunks: list[np.ndarray] = []
        for sample in samples:
            pts = sample.get(key)
            if isinstance(pts, np.ndarray) and pts.size:
                if key == "raw_points":
                    chunks.append(transform_sim_points(pts)[:, :2])
                else:
                    chunks.append(pts[:, :2])
        if key in {"raw_points", "map_points"}:
            aligned_sim = np.asarray(
                [transform_sim_xy(sample.get("sim_xy")) for sample in samples],
                dtype=np.float32,
            )
            aligned_sim = aligned_sim[np.isfinite(aligned_sim).all(axis=1)]
            if aligned_sim.size:
                chunks.append(aligned_sim)
            if odom_series.size:
                chunks.append(odom_series)
        if not chunks:
            return (-5.0, 5.0, -5.0, 5.0)
        xy = np.vstack(chunks)
        finite = np.isfinite(xy).all(axis=1)
        xy = xy[finite]
        if xy.size == 0:
            return (-5.0, 5.0, -5.0, 5.0)
        min_x, min_y = np.min(xy, axis=0)
        max_x, max_y = np.max(xy, axis=0)
        pad = max(1.0, float(max(max_x - min_x, max_y - min_y)) * 0.1)
        return (float(min_x - pad), float(max_x + pad), float(min_y - pad), float(max_y + pad))

    map_bounds = bounds_for("map_points")

    def sim_world_bounds2d() -> tuple[float, float, float, float]:
        chunks: list[np.ndarray] = []
        for sample in samples:
            raw = sample.get("raw_points")
            if isinstance(raw, np.ndarray) and raw.size and raw.shape[1] >= 2:
                chunks.append(raw[:, :2])
        if sim_series.size:
            chunks.append(sim_series)
        if not chunks:
            return (-5.0, 5.0, -5.0, 5.0)
        xy = np.vstack(chunks)
        xy = xy[np.isfinite(xy).all(axis=1)]
        if xy.size == 0:
            return (-5.0, 5.0, -5.0, 5.0)
        min_x, min_y = np.min(xy, axis=0)
        max_x, max_y = np.max(xy, axis=0)
        pad = max(1.0, float(max(max_x - min_x, max_y - min_y)) * 0.15)
        return (float(min_x - pad), float(max_x + pad), float(min_y - pad), float(max_y + pad))

    sim_bounds = sim_world_bounds2d()

    traj_chunks: list[np.ndarray] = []
    aligned_sim_all = np.asarray(
        [transform_sim_xy(sample.get("sim_xy")) for sample in samples],
        dtype=np.float32,
    )
    if aligned_sim_all.ndim == 2:
        aligned_sim_all = aligned_sim_all[np.isfinite(aligned_sim_all).all(axis=1)]
        if aligned_sim_all.size:
            traj_chunks.append(aligned_sim_all)
    if odom_series.size:
        traj_chunks.append(odom_series)
    if traj_chunks:
        traj_xy = np.vstack(traj_chunks)
        min_x, min_y = np.min(traj_xy, axis=0)
        max_x, max_y = np.max(traj_xy, axis=0)
        pad = max(0.5, float(max(max_x - min_x, max_y - min_y)) * 0.25)
        traj_bounds = (float(min_x - pad), float(max_x + pad), float(min_y - pad), float(max_y + pad))
    else:
        traj_bounds = (-2.0, 2.0, -2.0, 2.0)
    panel_bg = (28, 31, 36)
    grid_color = (58, 63, 72)
    text = (235, 238, 245)
    yellow = (40, 210, 245)
    green = (110, 220, 130)
    blue = (230, 160, 80)
    red = (80, 90, 245)

    def world_to_px(
        x: float,
        y: float,
        rect: tuple[int, int, int, int],
        bounds: tuple[float, float, float, float],
    ) -> tuple[int, int]:
        x0, y0, x1, y1 = rect
        min_x, max_x, min_y, max_y = bounds
        sx = (x - min_x) / max(max_x - min_x, 1e-6)
        sy = (y - min_y) / max(max_y - min_y, 1e-6)
        px = int(x0 + sx * (x1 - x0))
        py = int(y1 - sy * (y1 - y0))
        return px, py

    def draw_panel(frame: np.ndarray, rect: tuple[int, int, int, int], title: str) -> None:
        x0, y0, x1, y1 = rect
        cv2.rectangle(frame, (x0, y0), (x1, y1), panel_bg, -1)
        cv2.rectangle(frame, (x0, y0), (x1, y1), (85, 92, 105), 1)
        for i in range(1, 5):
            x = x0 + int((x1 - x0) * i / 5)
            y = y0 + int((y1 - y0) * i / 5)
            cv2.line(frame, (x, y0), (x, y1), grid_color, 1)
            cv2.line(frame, (x0, y), (x1, y), grid_color, 1)
        cv2.putText(frame, title, (x0 + 14, y0 + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, text, 2)

    def draw_points(
        frame: np.ndarray,
        pts: np.ndarray,
        rect: tuple[int, int, int, int],
        bounds: tuple[float, float, float, float],
        color: tuple[int, int, int],
        *,
        max_draw: int = 2500,
        radius: int = 0,
    ) -> None:
        if pts.size == 0:
            return
        pts = _sample_points(pts, max_draw)
        for x, y in pts[:, :2]:
            px, py = world_to_px(float(x), float(y), rect, bounds)
            if rect[0] <= px <= rect[2] and rect[1] <= py <= rect[3]:
                if radius > 0:
                    cv2.circle(frame, (px, py), radius, color, -1)
                else:
                    frame[py, px] = color

    def draw_polyline(
        frame: np.ndarray,
        xy: np.ndarray,
        rect: tuple[int, int, int, int],
        bounds: tuple[float, float, float, float],
        color: tuple[int, int, int],
    ) -> None:
        xy = np.asarray(xy, dtype=np.float32)
        if xy.ndim != 2 or xy.shape[0] < 2 or xy.shape[1] < 2:
            return
        xy = xy[:, :2]
        xy = xy[np.isfinite(xy).all(axis=1)]
        if xy.shape[0] < 2:
            return
        pts = [world_to_px(float(x), float(y), rect, bounds) for x, y in xy]
        for a, b in zip(pts[:-1], pts[1:]):
            cv2.line(frame, a, b, color, 2)

    def bounds3d_for(kind: str) -> tuple[float, float, float, float, float, float]:
        chunks: list[np.ndarray] = []
        if kind == "map":
            for sample in samples:
                map_pts = sample.get("map_points")
                if isinstance(map_pts, np.ndarray) and map_pts.size:
                    chunks.append(np.asarray(map_pts[:, :3], dtype=np.float32))
                raw_pts = sample.get("raw_points")
                raw_aligned = transform_sim_points(raw_pts)
                if raw_aligned.size:
                    chunks.append(raw_aligned[:, :3])
        if kind == "trajectory":
            aligned_sim = np.asarray(
                [transform_sim_xyz(sample.get("sim_xyz")) for sample in samples],
                dtype=np.float32,
            )
            aligned_sim = aligned_sim[np.isfinite(aligned_sim).all(axis=1)]
            if aligned_sim.size:
                chunks.append(aligned_sim)
            if odom_xyz_series.size:
                chunks.append(odom_xyz_series)
        if not chunks:
            return (-5.0, 5.0, -5.0, 5.0, -1.0, 3.0)
        xyz = np.vstack(chunks)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if xyz.size == 0:
            return (-5.0, 5.0, -5.0, 5.0, -1.0, 3.0)
        mins = np.min(xyz, axis=0)
        maxs = np.max(xyz, axis=0)
        spans = np.maximum(maxs - mins, 1e-3)
        pad_xy = max(1.0, float(max(spans[0], spans[1])) * 0.12)
        pad_z = max(0.4, float(spans[2]) * 0.18)
        return (
            float(mins[0] - pad_xy),
            float(maxs[0] + pad_xy),
            float(mins[1] - pad_xy),
            float(maxs[1] + pad_xy),
            float(mins[2] - pad_z),
            float(maxs[2] + pad_z),
        )

    def project_3d(
        xyz: np.ndarray,
        rect: tuple[int, int, int, int],
        bounds: tuple[float, float, float, float, float, float],
    ) -> tuple[np.ndarray, np.ndarray]:
        pts = np.asarray(xyz, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[0] <= 0 or pts.shape[1] < 3:
            return np.empty((0, 2), dtype=np.int32), np.empty((0,), dtype=np.float32)
        pts = pts[:, :3]
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.shape[0] <= 0:
            return np.empty((0, 2), dtype=np.int32), np.empty((0,), dtype=np.float32)
        min_x, max_x, min_y, max_y, min_z, max_z = bounds
        center = np.array(
            [(min_x + max_x) * 0.5, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5],
            dtype=np.float32,
        )
        span = max(max_x - min_x, max_y - min_y, (max_z - min_z) * 2.4, 1e-3)
        p = pts - center
        yaw = math.radians(-42.0)
        pitch = math.radians(58.0)
        rz = np.array(
            [
                [math.cos(yaw), -math.sin(yaw), 0.0],
                [math.sin(yaw), math.cos(yaw), 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        rx = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, math.cos(pitch), -math.sin(pitch)],
                [0.0, math.sin(pitch), math.cos(pitch)],
            ],
            dtype=np.float32,
        )
        q = (p @ rz.T) @ rx.T
        x0, y0, x1, y1 = rect
        scale = min(x1 - x0, y1 - y0) / (span * 1.25)
        px = ((x0 + x1) * 0.5 + q[:, 0] * scale).astype(np.int32)
        py = ((y0 + y1) * 0.55 - q[:, 1] * scale).astype(np.int32)
        return np.stack([px, py], axis=1), q[:, 2].astype(np.float32)

    def draw_points_3d(
        frame: np.ndarray,
        pts: np.ndarray,
        rect: tuple[int, int, int, int],
        bounds: tuple[float, float, float, float, float, float],
        color: tuple[int, int, int],
        *,
        max_draw: int = 3500,
        radius: int = 1,
    ) -> None:
        pts = _sample_points(np.asarray(pts, dtype=np.float32), max_draw)
        if pts.size == 0:
            return
        pix, depth = project_3d(pts, rect, bounds)
        if pix.size == 0:
            return
        order = np.argsort(depth)
        x0, y0, x1, y1 = rect
        for px, py in pix[order]:
            if x0 <= int(px) <= x1 and y0 <= int(py) <= y1:
                if radius > 0:
                    cv2.circle(frame, (int(px), int(py)), radius, color, -1)
                else:
                    frame[int(py), int(px)] = color

    def draw_polyline_3d(
        frame: np.ndarray,
        xyz: np.ndarray,
        rect: tuple[int, int, int, int],
        bounds: tuple[float, float, float, float, float, float],
        color: tuple[int, int, int],
    ) -> None:
        pts = np.asarray(xyz, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[0] < 2 or pts.shape[1] < 3:
            return
        pts = pts[:, :3]
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.shape[0] < 2:
            return
        pix, _depth = project_3d(pts, rect, bounds)
        for a, b in zip(pix[:-1], pix[1:]):
            cv2.line(frame, tuple(int(v) for v in a), tuple(int(v) for v in b), color, 2)

    def draw_axes_3d(
        frame: np.ndarray,
        rect: tuple[int, int, int, int],
        bounds: tuple[float, float, float, float, float, float],
    ) -> None:
        axis = np.asarray(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        pix, _depth = project_3d(axis, rect, bounds)
        if pix.shape[0] != 4:
            return
        origin = tuple(int(v) for v in pix[0])
        axes = [
            (pix[1], (60, 80, 245), "x"),
            (pix[2], (90, 220, 90), "y"),
            (pix[3], (245, 170, 80), "z"),
        ]
        for point, color, label in axes:
            end = tuple(int(v) for v in point)
            cv2.line(frame, origin, end, color, 2)
            cv2.putText(frame, label, end, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    map_bounds3d = bounds3d_for("map")
    traj_bounds3d = bounds3d_for("trajectory")
    sim_traj: list[list[float]] = []
    odom_traj: list[list[float]] = []
    sim_truth_xy: list[list[float]] = []
    scene_rect = (24, 56, 624, 356)
    map_rect = (656, 56, 1256, 356)
    traj_rect = (24, 398, 624, 696)
    info_rect = (656, 398, 1256, 696)

    for idx, sample in enumerate(samples):
        frame = np.full((height, width, 3), (18, 20, 24), dtype=np.uint8)
        t_s = float(sample.get("t_s") or 0.0)
        cv2.putText(
            frame,
            "LingTu MuJoCo + MID360 + Fast-LIO live validation",
            (24, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            text,
            2,
        )
        cv2.putText(
            frame,
            f"t={t_s:05.1f}s  sample={idx + 1}/{len(samples)}  SE2 rmse={alignment.get('rmse_m') or 0.0:.2f}m",
            (900, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            text,
            2,
        )

        draw_panel(frame, scene_rect, "MuJoCo environment render")
        overview = sample.get("overview_rgb")
        if isinstance(overview, np.ndarray) and overview.size:
            x0, y0, x1, y1 = scene_rect
            rgb = np.asarray(overview, dtype=np.uint8)
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if rgb.ndim == 3 else rgb
            bgr = cv2.resize(bgr, (x1 - x0 - 2, y1 - y0 - 2), interpolation=cv2.INTER_AREA)
            frame[y0 + 1 : y1 - 1, x0 + 1 : x1 - 1] = bgr
            cv2.putText(
                frame,
                "chase MuJoCo render, camera follows robot",
                (x0 + 14, y1 - 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                text,
                1,
            )
        else:
            cv2.putText(frame, "MuJoCo renderer unavailable", (48, 206), cv2.FONT_HERSHEY_SIMPLEX, 0.7, text, 2)

        inset = (scene_rect[2] - 232, scene_rect[3] - 132, scene_rect[2] - 12, scene_rect[3] - 12)
        cv2.rectangle(frame, (inset[0], inset[1]), (inset[2], inset[3]), (20, 22, 26), -1)
        cv2.rectangle(frame, (inset[0], inset[1]), (inset[2], inset[3]), (110, 118, 130), 1)
        raw_for_inset = sample.get("raw_points", np.empty((0, 3)))
        draw_points(frame, raw_for_inset, inset, sim_bounds, (80, 85, 95), max_draw=800)
        sim_xy_for_inset = sample.get("sim_xy")
        if sim_xy_for_inset is not None and np.isfinite(sim_xy_for_inset).all():
            sim_truth_xy.append([float(sim_xy_for_inset[0]), float(sim_xy_for_inset[1])])
        draw_polyline(frame, np.asarray(sim_truth_xy, dtype=np.float32), inset, sim_bounds, blue)
        if sim_truth_xy:
            px, py = world_to_px(sim_truth_xy[-1][0], sim_truth_xy[-1][1], inset, sim_bounds)
            cv2.circle(frame, (px, py), 5, (60, 60, 245), -1)
            cv2.putText(frame, "robot", (px + 7, py - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (60, 60, 245), 1)
        cv2.putText(
            frame,
            "MuJoCo truth trail",
            (inset[0] + 8, inset[1] + 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            text,
            1,
        )

        draw_panel(
            frame,
            map_rect,
            f"3D Fast-LIO map cloud ({TOPICS.map_cloud}, {SIM_FASTLIO_LIVE_MAP_FRAME_ID})",
        )
        draw_axes_3d(frame, map_rect, map_bounds3d)
        draw_points_3d(
            frame,
            sample.get("map_points", np.empty((0, 3))),
            map_rect,
            map_bounds3d,
            green,
            max_draw=4500,
            radius=1,
        )
        raw_points = sample.get("raw_points", np.empty((0, 3)))
        raw_points_aligned = transform_sim_points(raw_points)
        draw_points_3d(
            frame,
            raw_points_aligned,
            map_rect,
            map_bounds3d,
            yellow,
            max_draw=1200,
            radius=1,
        )
        moving_points = sample.get("moving_obstacle_points", np.empty((0, 3)))
        moving_points_aligned = transform_sim_points(moving_points)
        draw_points_3d(
            frame,
            moving_points_aligned,
            map_rect,
            map_bounds3d,
            red,
            max_draw=1200,
            radius=2,
        )
        cv2.putText(
            frame,
            "green=Fast-LIO accumulated  yellow=current MID360 scan  red=dynamic obstacle returns",
            (map_rect[0] + 14, map_rect[3] - 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            text,
            1,
        )

        draw_panel(frame, traj_rect, "3D trajectory consistency")
        draw_axes_3d(frame, traj_rect, traj_bounds3d)
        sim_xy = sample.get("sim_xy")
        odom_xy = sample.get("odom_xy")
        sim_xyz = sample.get("sim_xyz")
        odom_xyz = sample.get("odom_xyz")
        if sim_xyz is not None and np.isfinite(sim_xyz).all():
            aligned = transform_sim_xyz(sim_xyz)
            if np.isfinite(aligned).all():
                sim_traj.append([float(aligned[0]), float(aligned[1]), float(aligned[2])])
        elif sim_xy is not None and np.isfinite(sim_xy).all():
            aligned_xy = transform_sim_xy(sim_xy)
            if np.isfinite(aligned_xy).all():
                sim_traj.append([float(aligned_xy[0]), float(aligned_xy[1]), 0.0])
        if odom_xyz is not None and np.isfinite(odom_xyz).all():
            odom_traj.append([float(odom_xyz[0]), float(odom_xyz[1]), float(odom_xyz[2])])
        elif odom_xy is not None and np.isfinite(odom_xy).all():
            odom_traj.append([float(odom_xy[0]), float(odom_xy[1]), 0.0])
        draw_polyline_3d(frame, np.asarray(sim_traj, dtype=np.float32), traj_rect, traj_bounds3d, blue)
        draw_polyline_3d(frame, np.asarray(odom_traj, dtype=np.float32), traj_rect, traj_bounds3d, red)
        cv2.putText(
            frame,
            "blue=MuJoCo truth -> odom display fit  red=Fast-LIO odom",
            (42, 682),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            text,
            1,
        )

        draw_panel(frame, info_rect, "Gate evidence")
        lines = [
            f"SLAM state: {sample.get('slam_state', 'UNKNOWN')}",
            f"raw points/frame: {int(sample.get('raw_point_count') or 0)}",
            f"map cloud samples: {int(sample.get('map_cloud_count') or 0)}",
            f"sim moved: {float(sample.get('sim_moved_m') or 0.0):.2f} m",
            f"Fast-LIO moved: {float(sample.get('odom_moved_m') or 0.0):.2f} m",
            f"map XY area: {float(sample.get('map_area_m2') or 0.0):.1f} m2",
            f"traj SE2 rmse/max: {alignment.get('rmse_m') or 0.0:.2f}/{alignment.get('max_error_m') or 0.0:.2f} m",
            f"nav outputs: odom={int(sample.get('nav_odom_count') or 0)} map={int(sample.get('nav_map_count') or 0)}",
            f"moving obstacles visible: {int(sample.get('moving_obstacle_box_count') or 0)}",
            f"frames: odom={sample.get('nav_odom_frame', 'odom')} map_cloud={sample.get('nav_map_frame', 'odom')}",
            "hardware cmd_vel: disabled",
        ]
        y = 438
        for line in lines:
            cv2.putText(frame, line, (678, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, text, 2)
            y += 30
        writer.write(frame)

    writer.release()
    report["video_path"] = str(out_path)
    report["video_frame_count"] = len(samples)


def _area_growth(samples: list[float]) -> dict[str, float]:
    if not samples:
        return {"first_m2": 0.0, "last_m2": 0.0, "max_m2": 0.0, "growth_m2": 0.0}
    first = float(samples[0])
    last = float(samples[-1])
    max_area = float(max(samples))
    return {
        "first_m2": first,
        "last_m2": last,
        "max_m2": max_area,
        "growth_m2": max(0.0, max_area - first),
    }


def _coverage_growth(samples: list[dict[str, float]]) -> dict[str, float]:
    ratios: list[float] = []
    known_values: list[float] = []
    total_values: list[float] = []
    for sample in samples:
        known = float(sample.get("known_m2") or 0.0)
        unknown = float(sample.get("unknown_m2") or 0.0)
        total = max(0.0, known + unknown)
        ratio = known / total if total > 0.0 else 0.0
        ratios.append(float(ratio))
        known_values.append(float(known))
        total_values.append(float(total))
    if not ratios:
        return {
            "first_ratio": 0.0,
            "last_ratio": 0.0,
            "max_ratio": 0.0,
            "growth_ratio": 0.0,
            "first_known_m2": 0.0,
            "last_known_m2": 0.0,
            "first_total_m2": 0.0,
            "last_total_m2": 0.0,
        }
    first = ratios[0]
    max_ratio = max(ratios)
    return {
        "first_ratio": float(first),
        "last_ratio": float(ratios[-1]),
        "max_ratio": float(max_ratio),
        "growth_ratio": float(max(0.0, max_ratio - first)),
        "first_known_m2": float(known_values[0]),
        "last_known_m2": float(known_values[-1]),
        "first_total_m2": float(total_values[0]),
        "last_total_m2": float(total_values[-1]),
    }


def _box_surface_points(
    box: dict[str, Any],
    *,
    spacing: float,
    intensity: float,
) -> list[tuple[float, float, float, float]]:
    cx, cy, cz = [float(v) for v in box["position"][:3]]
    hx, hy, hz = [float(v) for v in box["half_size"][:3]]
    z = max(0.15, min(cz + hz * 0.35, 0.75))
    step = max(0.02, float(spacing))
    xs = np.arange(cx - hx, cx + hx + step * 0.5, step)
    ys = np.arange(cy - hy, cy + hy + step * 0.5, step)
    points: list[tuple[float, float, float, float]] = []
    thin_x = hx <= step * 0.75
    thin_y = hy <= step * 0.75
    if thin_x or thin_y:
        for x in xs:
            for y in ys:
                points.append((float(x), float(y), float(z), float(intensity)))
        return points
    for x in xs:
        points.append((float(x), float(cy - hy), float(z), float(intensity)))
        points.append((float(x), float(cy + hy), float(z), float(intensity)))
    for y in ys:
        points.append((float(cx - hx), float(y), float(z), float(intensity)))
        points.append((float(cx + hx), float(y), float(z), float(intensity)))
    return points


def _point_box_clearance(point: tuple[float, float], box: dict[str, Any]) -> float:
    cx, cy = [float(v) for v in box["position"][:2]]
    hx, hy = [float(v) for v in box["half_size"][:2]]
    dx = abs(float(point[0]) - cx) - hx
    dy = abs(float(point[1]) - cy) - hy
    outside_x = max(dx, 0.0)
    outside_y = max(dy, 0.0)
    if dx <= 0.0 and dy <= 0.0:
        return -min(-dx, -dy)
    return math.hypot(outside_x, outside_y)


def _live_moving_obstacle_boxes_from_pose(
    *,
    position_xy: tuple[float, float],
    yaw_rad: float,
    elapsed_s: float,
    mode: str,
    count: int,
    start_s: float,
    duration_s: float,
    period_s: float,
    forward_m: float,
    forward_step_m: float,
    lateral_phase_step_rad: float,
    lateral_amplitude_m: float,
    along_amplitude_m: float,
    radius_m: float,
    height_m: float,
) -> list[dict[str, Any]]:
    mode = str(mode or "none")
    if mode == "none":
        return []
    if elapsed_s < float(start_s):
        return []
    if float(duration_s) > 0.0 and elapsed_s > float(start_s) + float(duration_s):
        return []
    if mode != "robot_crossing":
        raise ValueError(f"unsupported moving_obstacle_mode: {mode}")

    obstacle_count = int(max(1, min(32, int(count))))
    t = float(elapsed_s) - float(start_s)
    period = max(0.1, float(period_s))
    base_phase = 2.0 * math.pi * t / period
    fwd = np.asarray([math.cos(yaw_rad), math.sin(yaw_rad)], dtype=np.float64)
    normal = np.asarray([-fwd[1], fwd[0]], dtype=np.float64)
    anchor = np.asarray(position_xy, dtype=np.float64)
    radius = max(0.02, float(radius_m))
    height = max(0.05, float(height_m))

    boxes: list[dict[str, Any]] = []
    for obstacle_idx in range(obstacle_count):
        offset = (float(obstacle_idx) - (float(obstacle_count) - 1.0) * 0.5) * float(forward_step_m)
        phase = base_phase + float(obstacle_idx) * float(lateral_phase_step_rad)
        lateral = float(lateral_amplitude_m) * math.sin(phase)
        along = float(along_amplitude_m) * math.sin(phase * 0.5)
        center = anchor + fwd * (float(forward_m) + offset + along) + normal * lateral
        boxes.append(
            {
                "name": (
                    "live_robot_crossing_obstacle"
                    if obstacle_count == 1
                    else f"live_robot_crossing_obstacle_{obstacle_idx}"
                ),
                "floor_id": 0,
                "position": [float(center[0]), float(center[1]), height * 0.5],
                "half_size": [radius, radius, height * 0.5],
                "elapsed_s": float(elapsed_s),
            }
        )
    return boxes


def _live_moving_obstacle_points(
    boxes: list[dict[str, Any]],
    *,
    spacing: float,
    intensity: float,
) -> list[tuple[float, float, float, float]]:
    points: list[tuple[float, float, float, float]] = []
    for box in boxes:
        points.extend(_box_surface_points(box, spacing=spacing, intensity=intensity))
    return points


def _live_moving_obstacle_speed_bounds(
    *,
    period_s: float,
    lateral_amplitude_m: float,
    along_amplitude_m: float,
) -> dict[str, float]:
    period = max(0.1, float(period_s))
    lateral_peak = abs(float(lateral_amplitude_m)) * 2.0 * math.pi / period
    along_peak = abs(float(along_amplitude_m)) * math.pi / period
    return {
        "peak_lateral_speed_mps": round(float(lateral_peak), 4),
        "peak_along_speed_mps": round(float(along_peak), 4),
        "peak_planar_speed_bound_mps": round(float(math.hypot(lateral_peak, along_peak)), 4),
    }


def _live_moving_obstacle_trail_clearance(
    *,
    timed_trail: list[tuple[float, float, float, float]],
    robot_radius_m: float,
    mode: str,
    count: int,
    start_s: float,
    duration_s: float,
    period_s: float,
    forward_m: float,
    forward_step_m: float,
    lateral_phase_step_rad: float,
    lateral_amplitude_m: float,
    along_amplitude_m: float,
    radius_m: float,
    height_m: float,
) -> dict[str, Any]:
    if str(mode or "none") == "none":
        return {"checked": False, "collision": False}
    best: float | None = None
    best_sample: dict[str, Any] | None = None
    active_count = 0
    for elapsed_s, x, y, yaw in timed_trail:
        boxes = _live_moving_obstacle_boxes_from_pose(
            position_xy=(float(x), float(y)),
            yaw_rad=float(yaw),
            elapsed_s=float(elapsed_s),
            mode=mode,
            count=count,
            start_s=start_s,
            duration_s=duration_s,
            period_s=period_s,
            forward_m=forward_m,
            forward_step_m=forward_step_m,
            lateral_phase_step_rad=lateral_phase_step_rad,
            lateral_amplitude_m=lateral_amplitude_m,
            along_amplitude_m=along_amplitude_m,
            radius_m=radius_m,
            height_m=height_m,
        )
        if not boxes:
            continue
        active_count += 1
        for box in boxes:
            clearance = _point_box_clearance((float(x), float(y)), box)
            if best is None or clearance < best:
                best = clearance
                best_sample = {
                    "elapsed_s": round(float(elapsed_s), 3),
                    "robot_xy": [round(float(x), 4), round(float(y), 4)],
                    "obstacle": box.get("name"),
                    "obstacle_xy": [
                        round(float(box["position"][0]), 4),
                        round(float(box["position"][1]), 4),
                    ],
                }
    margin = None if best is None else float(best) - float(robot_radius_m)
    return {
        "checked": True,
        "active_trail_sample_count": int(active_count),
        "min_clearance_m": None if best is None else round(float(best), 4),
        "min_clearance_minus_robot_radius_m": None if margin is None else round(float(margin), 4),
        "collision": bool(margin is not None and margin < 0.0),
        "robot_radius_m": float(robot_radius_m),
        "min_sample": best_sample,
    }


def _goal_xy(value: Any) -> tuple[float, float] | None:
    if isinstance(value, dict):
        value = value.get("goal") or value.get("current_goal")
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        return None
    try:
        return float(value[0]), float(value[1])
    except (TypeError, ValueError):
        return None


def _goal_xy_matches(
    a: tuple[float, float] | None,
    b: tuple[float, float] | None,
    *,
    tolerance: float = 1.0,
) -> bool:
    if a is None or b is None:
        return False
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1])) <= tolerance


def _parse_inspection_goals(value: str | list[Any] | tuple[Any, ...] | None) -> list[list[float]]:
    if value is None:
        return []
    raw: Any = value
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return []
        if text.startswith("["):
            raw = json.loads(text)
        else:
            raw = [item.strip() for item in text.split(";") if item.strip()]
    goals: list[list[float]] = []
    if not isinstance(raw, (list, tuple)):
        raise ValueError("inspection goals must be a JSON list or ';'-separated x,y[,z] list")
    for item in raw:
        frame_id = ""
        if isinstance(item, dict):
            frame_id = str(item.get("frame_id") or "")
            coords = [item.get("x"), item.get("y"), item.get("z", 0.0)]
        elif isinstance(item, str):
            coords = [part.strip() for part in item.split(",")]
        elif isinstance(item, (list, tuple)):
            coords = list(item)
        else:
            raise ValueError(f"unsupported inspection goal item: {item!r}")
        if frame_id and frame_id != SIM_NAV_ODOMETRY_FRAME_ID:
            raise ValueError(
                "inspection goal frame must be "
                f"{SIM_NAV_ODOMETRY_FRAME_ID}: {frame_id}"
            )
        if len(coords) < 2:
            raise ValueError(f"inspection goal needs x,y: {item!r}")
        try:
            x = float(coords[0])
            y = float(coords[1])
            z = float(coords[2]) if len(coords) > 2 and coords[2] not in (None, "") else 0.0
        except (TypeError, ValueError) as exc:
            raise ValueError(f"non-numeric inspection goal: {item!r}") from exc
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            raise ValueError(f"non-finite inspection goal: {item!r}")
        goals.append([x, y, z])
    return goals


def _map_frame_origin_world_xy_from_tomogram(
    tomogram: Path | str | None,
) -> tuple[float, float] | None:
    if not tomogram:
        return None
    path = Path(str(tomogram))
    candidates = []
    if path.is_dir():
        candidates.append(path / "metadata.json")
    else:
        candidates.append(path.parent / "metadata.json")
    for candidate in candidates:
        try:
            payload = json.loads(candidate.read_text(encoding="utf-8"))
        except Exception:
            continue
        origin = payload.get("map_frame_origin_world_xy")
        if isinstance(origin, (list, tuple)) and len(origin) >= 2:
            try:
                return (float(origin[0]), float(origin[1]))
            except (TypeError, ValueError):
                continue
    return None


def _state_in_map_frame(
    state: Any,
    origin_world_xy: tuple[float, float] | None,
) -> Any:
    position = np.asarray(state.position, dtype=np.float64).copy()
    if origin_world_xy is not None and position.shape[0] >= 2:
        position[0] -= float(origin_world_xy[0])
        position[1] -= float(origin_world_xy[1])
    return SimpleNamespace(
        position=position,
        orientation=np.asarray(state.orientation, dtype=np.float64).copy(),
        linear_velocity=np.asarray(state.linear_velocity, dtype=np.float64).copy(),
        angular_velocity=np.asarray(state.angular_velocity, dtype=np.float64).copy(),
    )


def _limit_command_delta(
    *,
    target: tuple[float, float, float],
    previous: tuple[float, float, float],
    dt_s: float,
    linear_accel_limit: float,
    angular_accel_limit: float,
) -> tuple[float, float, float]:
    dt = max(0.0, float(dt_s))
    linear_step = max(0.0, float(linear_accel_limit)) * dt
    angular_step = max(0.0, float(angular_accel_limit)) * dt

    def _limit_axis(target_value: float, previous_value: float, step: float) -> float:
        delta = float(target_value) - float(previous_value)
        if step <= 0.0:
            return float(target_value)
        return float(previous_value) + float(np.clip(delta, -step, step))

    return (
        _limit_axis(float(target[0]), float(previous[0]), linear_step),
        _limit_axis(float(target[1]), float(previous[1]), linear_step),
        _limit_axis(float(target[2]), float(previous[2]), angular_step),
    )


def _select_nav_cmd_for_step(
    *,
    latest_nav_cmd: dict[str, float],
    now_s: float,
    cmd_vel_timeout_s: float,
) -> dict[str, float | bool | None]:
    cmd_stamp = float(latest_nav_cmd.get("stamp") or 0.0)
    age_s = max(0.0, float(now_s) - cmd_stamp) if cmd_stamp > 0.0 else None
    timeout_s = float(cmd_vel_timeout_s)
    fresh = bool(cmd_stamp > 0.0 and (timeout_s <= 0.0 or float(age_s or 0.0) <= timeout_s))
    return {
        "fresh": fresh,
        "age_s": age_s,
        "vx": float(latest_nav_cmd.get("vx") or 0.0) if fresh else 0.0,
        "vy": float(latest_nav_cmd.get("vy") or 0.0) if fresh else 0.0,
        "wz": float(latest_nav_cmd.get("wz") or 0.0) if fresh else 0.0,
    }


def _motion_consistency_report(
    *,
    fastlio2_moved_m: float | None,
    fastlio2_path_length_m: float | None,
    sim_moved_m: float | None,
    sim_path_length_m: float | None,
) -> dict[str, Any]:
    checked = bool(fastlio2_moved_m is not None and sim_moved_m is not None)
    report: dict[str, Any] = {
        "checked": checked,
        "ok": checked,
        "fastlio2_moved_m": fastlio2_moved_m,
        "sim_moved_m": sim_moved_m,
        "fastlio2_path_length_m": fastlio2_path_length_m,
        "sim_path_length_m": sim_path_length_m,
        "motion_delta_error_m": None,
        "max_allowed_motion_error_m": None,
        "max_allowed_fastlio2_moved_m": None,
        "motion_scale_ratio": None,
    }
    if not checked:
        return report

    fast_m = float(fastlio2_moved_m or 0.0)
    sim_m = float(sim_moved_m or 0.0)
    error_m = abs(fast_m - sim_m)
    # Keep startup tolerance, but do not let short fixed-motion checks hide
    # large odometry scale errors. A 0.4x or 2x estimate is not acceptable even
    # when the absolute distance is still below a meter.
    allowed_error_m = max(0.12, sim_m * 0.35)
    report.update(
        {
            "ok": bool(error_m <= allowed_error_m),
            "motion_delta_error_m": error_m,
            "max_allowed_motion_error_m": allowed_error_m,
            "max_allowed_fastlio2_moved_m": sim_m + allowed_error_m,
            "motion_scale_ratio": (
                fast_m / sim_m if abs(sim_m) > 1e-6 else None
            ),
        }
    )
    return report


def _nav_planner_has_live_map(nav_module: Any) -> bool:
    planner_svc = getattr(nav_module, "_planner_svc", None)
    if planner_svc is None:
        return False
    try:
        return bool(getattr(planner_svc, "has_map", False))
    except Exception:
        return False


def _exploration_area_sample(grid: dict[str, Any]) -> dict[str, float]:
    counts = dict(grid.get("counts") or {})
    resolution = float(grid.get("resolution") or 0.0)
    cell_area = resolution * resolution
    unknown = int(counts.get("unknown") or 0)
    free = int(counts.get("free") or 0)
    occupied = int(counts.get("occupied") or 0)
    known = free + occupied
    return {
        "unknown_cells": float(unknown),
        "free_cells": float(free),
        "occupied_cells": float(occupied),
        "known_cells": float(known),
        "unknown_m2": float(unknown * cell_area),
        "free_m2": float(free * cell_area),
        "occupied_m2": float(occupied * cell_area),
        "known_m2": float(known * cell_area),
    }


def run_gate(
    *,
    world: Path,
    duration: float,
    drive_vx: float,
    drive_vy: float,
    drive_wz: float,
    n_rays: int,
    start: list[float] | None,
    mujoco_memory: str,
    mid360_pattern: Path | str | None,
    mid360_samples_per_frame: int,
    startup_sleep: float,
    settle_sleep: float,
    work_dir: Path,
    backend_profile: str,
    drive_mode: str,
    duration_clock: str = "wall",
    max_wall_time_s: float = 0.0,
    drive_source: str = "fixed",
    cmd_vel_timeout: float = 0.75,
    cmd_vel_linear_limit: float = 0.25,
    cmd_vel_angular_limit: float = 0.15,
    cmd_vel_sim_linear_scale: float = 1.0,
    cmd_vel_sim_angular_scale: float = 1.0,
    cmd_vel_linear_accel_limit: float = 0.5,
    cmd_vel_angular_accel_limit: float = 1.0,
    nav_max_linear_speed: float = 0.25,
    nav_max_angular_z: float = 0.15,
    nav_turn_speed_yaw_rate_start: float = 0.0,
    nav_turn_speed_min_scale: float = 1.0,
    run_lingtu_frontier: bool = False,
    run_lingtu_tare: bool = False,
    run_lingtu_inspection: bool = False,
    tare_min_goals: int = 2,
    tare_start_delay: float = 0.0,
    tare_goal_timeout: float = 180.0,
    tare_scenario: str = "indoor",
    frontier_min_goals: int = 3,
    frontier_start_delay: float = 0.0,
    frontier_goal_timeout: float = 30.0,
    inspection_goals: str = "0.8,0.0;1.6,0.2;2.4,0.4",
    inspection_min_checkpoints: int = 3,
    inspection_start_delay: float = 0.0,
    inspection_goal_timeout: float = 90.0,
    inspection_planner: str = "astar",
    inspection_tomogram: Path | str | None = None,
    min_map_area_growth_m2: float = 0.25,
    min_explored_area_growth_m2: float = 0.25,
    min_exploration_coverage_growth_ratio: float = 0.001,
    max_fastlio_z_drift_m: float = 1.0,
    max_fastlio_yaw_drift_rad: float = 0.5,
    runtime_fault_confirm_samples: int = 2,
    fastlio_lidar_input: str = "livox_custom_msg",
    fastlio_lidar_filter_num: int = 4,
    fastlio_scan_resolution: float = 0.15,
    fastlio_map_resolution: float = 0.3,
    fastlio_near_search_num: int = 5,
    fastlio_ieskf_max_iter: int = 5,
    fastlio_lidar_cov_inv: float = 1000.0,
    fastlio_time_diff_lidar_to_imu: float = 0.0,
    fastlio_vertical_velocity_constraint: str = "off",
    scan_time_profile: str = "physical_rolling",
    imu_acc_mode: str = "finite_difference",
    nav_data_source: str = "fastlio2",
    show_mujoco_window: bool = False,
    mujoco_window_fps: float = 10.0,
    video_out: Path | str | None = None,
    video_fps: float = 8.0,
    moving_obstacle_mode: str = "none",
    moving_obstacle_count: int = 1,
    moving_obstacle_start_s: float = 8.0,
    moving_obstacle_duration_s: float = 0.0,
    moving_obstacle_period_s: float = 10.0,
    moving_obstacle_forward_m: float = 2.0,
    moving_obstacle_forward_step_m: float = 0.8,
    moving_obstacle_lateral_phase_step_rad: float = math.pi / 2.0,
    moving_obstacle_lateral_amplitude_m: float = 0.75,
    moving_obstacle_along_amplitude_m: float = 0.25,
    moving_obstacle_radius_m: float = 0.16,
    moving_obstacle_height_m: float = 0.60,
    moving_obstacle_point_spacing: float = 0.10,
    moving_obstacle_intensity: float = 220.0,
    moving_obstacle_robot_radius_m: float = 0.28,
    max_yaw_per_meter: float = 1.2,
    max_angular_saturation_ratio: float = 0.35,
    save_map_artifacts: bool = True,
    build_tomogram: bool = False,
    map_artifact_voxel_size: float = 0.10,
    map_artifact_max_points: int = 250_000,
    map_artifact_max_span_m: float = 120.0,
    tomogram_resolution: float = 0.20,
    tomogram_slice_dh: float = 0.25,
    tomogram_ground_h: float = 0.0,
    tomogram_max_cells: int = 50_000_000,
) -> dict[str, Any]:
    selected_lingtu_modes = [
        name
        for name, enabled in (
            ("frontier", run_lingtu_frontier),
            ("tare", run_lingtu_tare),
            ("inspection", run_lingtu_inspection),
        )
        if enabled
    ]
    if len(selected_lingtu_modes) > 1:
        raise ValueError(
            "--run-lingtu-frontier, --run-lingtu-tare, and "
            "--run-lingtu-inspection are mutually exclusive"
        )
    inspection_goal_list = _parse_inspection_goals(inspection_goals)
    if run_lingtu_inspection and not inspection_goal_list:
        raise ValueError("--run-lingtu-inspection requires at least one inspection goal")
    duration_clock = str(duration_clock or "wall").strip().lower()
    if duration_clock not in {"wall", "sim"}:
        raise ValueError(f"unsupported duration_clock: {duration_clock}")
    max_wall_time_s = float(max_wall_time_s or 0.0)
    if max_wall_time_s < 0.0:
        raise ValueError("--max-wall-time-s must be non-negative")
    moving_obstacle_mode = str(moving_obstacle_mode or "none").strip().lower()
    if moving_obstacle_mode not in {"none", "robot_crossing"}:
        raise ValueError(f"unsupported moving_obstacle_mode: {moving_obstacle_mode}")
    scan_time_profile = str(scan_time_profile or "physical_rolling").strip().lower()
    if scan_time_profile not in {"instantaneous", "synthetic_rolling", "physical_rolling"}:
        raise ValueError(f"unsupported scan_time_profile: {scan_time_profile}")
    (
        rclpy,
        Imu,
        Odometry,
        PointCloud2,
        PointField,
        CustomMsg,
        CustomPoint,
        TransformStamped,
        TransformBroadcaster,
        StaticTransformBroadcaster,
    ) = _load_ros_modules(
        require_livox=fastlio_lidar_input == "livox_custom_msg"
    )
    from builtin_interfaces.msg import Time as RosTime  # type: ignore
    from geometry_msgs.msg import TwistStamped  # type: ignore
    from sim.engine.core.engine import VelocityCommand
    from slam.slam_bridge_module import SlamBridgeModule

    work_dir.mkdir(parents=True, exist_ok=True)
    config_path = work_dir / "fastlio2_mujoco_live.yaml"
    log_path = work_dir / "fastlio2_node.log"
    mujoco_lidar_extrinsic = lidar_extrinsic("mujoco_thunder_v3")
    imu_acc_mode = str(imu_acc_mode or "gravity_only").strip().lower()
    if imu_acc_mode not in {"gravity_only", "finite_difference"}:
        raise ValueError(f"unsupported imu_acc_mode: {imu_acc_mode}")
    sim_is_commanded_to_move = bool(
        drive_source == "nav_cmd_vel"
        or abs(float(drive_vx)) > 1e-4
        or abs(float(drive_vy)) > 1e-4
        or abs(float(drive_wz)) > 1e-4
    )
    # Kinematic MuJoCo motion can have long constant-velocity segments. If the
    # synthetic IMU is gravity-only, Fast-LIO's ZUPT detector mistakes moving
    # straight lines for static operation and clamps the state while LiDAR is
    # moving. Disable ZUPT for this live simulator; the real robot keeps its
    # calibrated defaults in the normal Fast-LIO config.
    disable_sim_zupt = bool(sim_is_commanded_to_move)
    vertical_velocity_constraint_mode = str(
        fastlio_vertical_velocity_constraint or "auto"
    ).strip().lower()
    if vertical_velocity_constraint_mode not in {"auto", "on", "off"}:
        raise ValueError(
            "fastlio_vertical_velocity_constraint must be one of auto, on, off"
        )
    vertical_velocity_constraint_enabled = (
        disable_sim_zupt
        if vertical_velocity_constraint_mode == "auto"
        else vertical_velocity_constraint_mode == "on"
    )
    _write_fastlio2_config(
        config_path,
        imu_topic=TOPICS.raw_imu,
        lidar_topic=TOPICS.raw_lidar_points,
        lidar_type=1 if fastlio_lidar_input == "livox_custom_msg" else 2,
        scan_line=4,
        timestamp_unit=3 if fastlio_lidar_input == "livox_custom_msg" else 0,
        livox_scan_window=0.0 if fastlio_lidar_input == "livox_custom_msg" else None,
        t_il=mujoco_lidar_extrinsic.translation,
        imu_static_acc_thresh=0.0 if disable_sim_zupt else 0.04,
        imu_static_gyro_thresh=0.0 if disable_sim_zupt else 0.001,
        zupt_min_static_frames=1_000_000 if disable_sim_zupt else 5,
        lidar_filter_num=fastlio_lidar_filter_num,
        scan_resolution=fastlio_scan_resolution,
        map_resolution=fastlio_map_resolution,
        near_search_num=fastlio_near_search_num,
        ieskf_max_iter=fastlio_ieskf_max_iter,
        lidar_cov_inv=fastlio_lidar_cov_inv,
        time_diff_lidar_to_imu=fastlio_time_diff_lidar_to_imu,
        vertical_velocity_constraint_enabled=vertical_velocity_constraint_enabled,
    )

    ros2 = ensure_ros2_cli()

    lidar_period = 1.0 / 10.0
    engine = _build_engine(
        world=world,
        drive_mode=drive_mode,
        n_rays=n_rays,
        start=start,
        mujoco_memory=mujoco_memory,
        mid360_pattern=mid360_pattern,
        mid360_samples_per_frame=mid360_samples_per_frame,
    )
    resolved_pattern = _resolve_mid360_pattern(mid360_pattern)
    physical_rolling_subscan_count = 0
    physical_rolling_effective_samples_per_subscan = int(mid360_samples_per_frame)
    if scan_time_profile == "physical_rolling":
        control_dt_s = float(getattr(engine, "control_dt", 0.02) or 0.02)
        physical_rolling_subscan_count = max(
            1,
            int(round(float(lidar_period) / max(control_dt_s, 1e-6))),
        )
        physical_rolling_effective_samples_per_subscan = max(
            1,
            int(math.ceil(float(mid360_samples_per_frame) / physical_rolling_subscan_count)),
        )
        lidar = getattr(engine, "_lidar", None)
        lidar_cfg = getattr(lidar, "config", None)
        if lidar_cfg is not None and hasattr(lidar_cfg, "samples_per_frame"):
            lidar_cfg.samples_per_frame = physical_rolling_effective_samples_per_subscan

    rclpy.init(args=None)
    node = rclpy.create_node("mujoco_fastlio2_live_gate")
    bridge = SlamBridgeModule(
        backend_profile=backend_profile,
        odom_timeout=6.0,
        cloud_timeout=6.0,
        localizer_health_timeout=6.0,
        watchdog_hz=20,
        localizer_odom_loss_recovery=False,
        drift_recovery_enabled=False,
        drift_max_speed=1.0e6,
        drift_max_jump=1.0e6,
        drift_max_z_jump=1.0e6,
        drift_max_abs_z=1.0e6,
        drift_max_pos=1.0e6,
        drift_relocalize_cooldown=1.0e12,
    )

    bridge_statuses: list[dict[str, Any]] = []
    first_sim_xyz: list[float] | None = None
    last_sim_xyz: list[float] | None = None
    prev_sim_xyz: list[float] | None = None
    sim_path_length_m = 0.0
    runtime_faults: list[str] = []
    runtime_fault_events: list[dict[str, Any]] = []
    runtime_fault_streaks = {"motion": 0, "z": 0, "yaw": 0}
    wall_timeout = _wall_timeout_status(0.0, max_wall_time_s)
    source_end_bridge_status: dict[str, Any] | None = None
    latest_nav_cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "stamp": 0.0}
    nav_cmd_vel_samples: list[dict[str, float]] = []
    degeneracy_detail_samples: list[dict[str, Any]] = []
    degeneracy_detail_subscription_error = ""
    previous_applied_command = (0.0, 0.0, 0.0)
    frontier_goals: list[list[float]] = []
    frontier_batches: list[int] = []
    frontier_mission_events: list[dict[str, Any]] = []
    frontier_terminal_keys: set[str] = set()
    tare_goals: list[list[float]] = []
    tare_path_counts: list[int] = []
    tare_stats_samples: list[dict[str, Any]] = []
    tare_mission_events: list[dict[str, Any]] = []
    tare_terminal_keys: set[str] = set()
    inspection_goals_sent: list[list[float]] = []
    inspection_mission_events: list[dict[str, Any]] = []
    inspection_terminal_keys: set[str] = set()
    exploration_grid_counts: list[dict[str, int]] = []
    exploration_grid_metadata: list[dict[str, str]] = []
    exploration_area_samples: list[dict[str, float]] = []
    global_path_counts: list[int] = []
    local_path_counts: list[int] = []
    global_path_summaries: list[dict[str, Any]] = []
    local_path_summaries: list[dict[str, Any]] = []
    waypoint_count = 0
    frontier_begin_status = ""
    tare_start_status = ""
    inspection_start_status = ""
    lingtu_failed_modules: dict[str, str] = {}
    frontier_module: Any | None = None
    tare_module: Any | None = None
    nav_module: Any | None = None
    local_planner_module: Any | None = None
    path_follower_module: Any | None = None
    cmd_vel_mux_module: Any | None = None
    frontier_ready_since: float | None = None
    frontier_started_after_slam_ready = False
    tare_ready_since: float | None = None
    tare_started_after_slam_ready = False
    inspection_ready_since: float | None = None
    inspection_started_after_slam_ready = False
    final_navigation_health: dict[str, Any] = {}
    final_navigation_status: dict[str, Any] = {}
    final_frontier_health: dict[str, Any] = {}
    final_tare_status: dict[str, Any] = {}
    final_tare_health: dict[str, Any] = {}
    final_local_planner_health: dict[str, Any] = {}
    final_path_follower_health: dict[str, Any] = {}
    final_cmd_vel_mux_health: dict[str, Any] = {}
    visible_ros_topics: list[str] = []
    visible_ros_topic_error = ""
    nav_data_source = str(nav_data_source or "fastlio2").strip().lower()
    if nav_data_source not in {"fastlio2", "mujoco_ground_truth"}:
        raise ValueError(f"unsupported nav_data_source: {nav_data_source}")
    demo_truth_nav = nav_data_source == "mujoco_ground_truth"
    truth_nav_origin_world_xy = _map_frame_origin_world_xy_from_tomogram(
        inspection_tomogram
    )
    truth_nav_origin_source = (
        "inspection_tomogram_metadata" if truth_nav_origin_world_xy is not None else ""
    )
    video_samples: list[dict[str, Any]] = []
    navigation_diagnostic_samples: list[dict[str, Any]] = []
    imu_diagnostic_samples: list[dict[str, Any]] = []
    command_diagnostic_samples: list[dict[str, Any]] = []
    latest_scan_relative_times_s: list[float] = []
    navigation_diagnostic_sample_period_s = 2.0
    next_navigation_diagnostic_sample_s = 0.0
    video_out_path = Path(video_out) if video_out else None
    video_sample_period_s = 0.5
    next_video_sample_s = 0.0
    moving_obstacle_enabled = moving_obstacle_mode != "none"
    moving_obstacle_published_update_count = 0
    moving_obstacle_published_point_count_max = 0
    moving_obstacle_samples: list[dict[str, Any]] = []
    moving_obstacle_timed_trail: list[tuple[float, float, float, float]] = []
    sim_pose_timed_trail: list[tuple[float, float, float, float, float]] = []
    current_moving_obstacle_boxes: list[dict[str, Any]] = []
    current_moving_obstacle_points: list[tuple[float, float, float, float]] = []
    rolling_lidar_samples: list[tuple[float, np.ndarray, np.ndarray, int]] = []
    rolling_lidar_subscan_counts: list[int] = []
    live_window_name = "LingTu MuJoCo live"
    live_window_frames = 0
    live_window_error = ""
    live_window_created = False
    live_cv2: Any = None
    next_live_window_s = 0.0
    next_runtime_guard_s = 5.0
    latest_raw_world_points = np.empty((0, 3), dtype=np.float32)
    latest_map_points = np.empty((0, 3), dtype=np.float32)
    map_artifact_points_by_cell: dict[tuple[int, int, int], tuple[float, float, float]] = {}
    map_artifact_samples = 0
    map_artifact_frames: list[str] = []
    video_render_state: dict[str, Any] = {}

    def _accumulate_map_artifact(msg: Any) -> None:
        nonlocal latest_map_points, map_artifact_samples
        artifact_points = _pointcloud_xyz_sample(
            msg,
            max_points=min(max(int(map_artifact_max_points), 1), 50_000),
        )
        latest_map_points = _sample_points(artifact_points, 4500)
        if not save_map_artifacts:
            return
        frame = str(getattr(getattr(msg, "header", None), "frame_id", "") or "")
        if frame and frame not in map_artifact_frames:
            map_artifact_frames.append(frame)
        added = _add_points_to_voxel_store(
            map_artifact_points_by_cell,
            artifact_points,
            voxel_size=float(map_artifact_voxel_size),
            max_points=int(map_artifact_max_points),
        )
        if added > 0:
            map_artifact_samples += 1

    def on_nav_cmd_vel(msg: Any) -> None:
        twist = getattr(msg, "twist", msg)
        linear = getattr(twist, "linear", None)
        angular = getattr(twist, "angular", None)
        vx = float(getattr(linear, "x", 0.0) if linear is not None else 0.0)
        vy = float(getattr(linear, "y", 0.0) if linear is not None else 0.0)
        wz = float(getattr(angular, "z", 0.0) if angular is not None else 0.0)
        latest_nav_cmd.update({"vx": vx, "vy": vy, "wz": wz, "stamp": time.time()})
        nav_cmd_vel_samples.append({"vx": vx, "vy": vy, "wz": wz, "stamp": time.time()})

    def on_degeneracy_detail(msg: Any) -> None:
        sample = _degeneracy_detail_sample(
            getattr(msg, "data", []),
            stamp_s=time.time(),
        )
        if sample is None:
            return
        degeneracy_detail_samples.append(sample)
        if len(degeneracy_detail_samples) > 1000:
            del degeneracy_detail_samples[:500]

    bridge.localization_status._add_callback(bridge_statuses.append)
    nav_bridge = FastLio2NavBridgeRuntime(
        node=node,
        slam_bridge=bridge,
        odometry_cls=Odometry,
        pointcloud2_cls=PointCloud2,
        pointcloud_xy_stats=_pointcloud_xy_stats,
        odom_xyz=_odom_xyz,
        odom_yaw=_odom_yaw,
        on_nav_map_cloud=_accumulate_map_artifact,
        use_fastlio_for_nav=not demo_truth_nav,
        publish_tare_context_topics=bool(run_lingtu_tare),
    )

    def _current_navigation_health() -> dict[str, Any]:
        if nav_module is None:
            return {}
        try:
            return dict((nav_module.health() or {}).get("navigation") or {})
        except Exception as exc:
            return {"error": f"{type(exc).__name__}: {exc}"}

    def _append_navigation_diagnostic_sample(
        *,
        elapsed_sim_s: float,
        elapsed_wall_s: float,
        command_fresh: bool,
    ) -> None:
        navigation_diagnostic_samples.append(
            _navigation_diagnostic_sample(
                sim_time_s=elapsed_sim_s,
                wall_time_s=elapsed_wall_s,
                first_sim_xyz=first_sim_xyz,
                current_sim_xyz=last_sim_xyz,
                first_sim_yaw=first_sim_yaw,
                current_sim_yaw=last_sim_yaw,
                first_odom_xyz=nav_bridge.first_odom_xyz,
                current_odom_xyz=nav_bridge.last_odom_xyz,
                first_odom_yaw=nav_bridge.first_odom_yaw,
                current_odom_yaw=nav_bridge.last_odom_yaw,
                latest_nav_cmd=latest_nav_cmd,
                now_s=time.time(),
                cmd_vel_timeout_s=float(cmd_vel_timeout),
                command_fresh=command_fresh,
                global_path_counts=global_path_counts,
                local_path_counts=local_path_counts,
                waypoint_count=waypoint_count,
                navigation_health=_current_navigation_health(),
                runtime_faults=runtime_faults,
            )
        )

    node.create_subscription(TwistStamped, TOPICS.cmd_vel, on_nav_cmd_vel, 10)
    try:
        from std_msgs.msg import Float32MultiArray  # type: ignore

        node.create_subscription(
            Float32MultiArray,
            "/slam/degeneracy_detail",
            on_degeneracy_detail,
            10,
        )
    except Exception as exc:  # pragma: no cover - depends on ROS2 std_msgs env
        degeneracy_detail_subscription_error = f"{type(exc).__name__}: {exc}"
    imu_pub = node.create_publisher(Imu, TOPICS.raw_imu, 100)
    raw_lidar_msg_type = CustomMsg if fastlio_lidar_input == "livox_custom_msg" else PointCloud2
    cloud_pub = node.create_publisher(raw_lidar_msg_type, TOPICS.raw_lidar_points, 10)
    tf_broadcaster = TransformBroadcaster(node)
    static_tf_broadcaster = StaticTransformBroadcaster(node)
    static_tf_broadcaster.sendTransform(
        [
            _make_transform_msg(
                stamp=node.get_clock().now().to_msg(),
                transform_cls=TransformStamped,
                parent=SIM_MAP_FRAME_ID,
                child=SIM_ODOM_FRAME_ID,
            ),
            _make_transform_msg(
                stamp=node.get_clock().now().to_msg(),
                transform_cls=TransformStamped,
                parent=SIM_BODY_FRAME_ID,
                child=SIM_LIDAR_FRAME_ID,
                translation_xyz=mujoco_lidar_extrinsic.translation,
                rotation_xyzw=mujoco_lidar_extrinsic.rotation_xyzw,
            ),
        ]
    )

    process: FastLio2Process | None = None
    counts = {
        "imu_published": 0,
        "cloud_published": 0,
        "empty_cloud_frames": 0,
        "sim_steps": 0,
    }
    point_counts: list[int] = []
    subscription_counts = {"imu": 0, "cloud": 0}
    started = time.time()
    sim_loop_start_wall: float | None = None
    sim_loop_end_wall: float | None = None
    sim_loop_start_time = float(getattr(engine, "sim_time", 0.0))
    sim_loop_end_time = sim_loop_start_time
    prev_velocity: np.ndarray | None = None
    prev_lidar_pub_sim_time = (
        float(getattr(engine, "sim_time", 0.0))
        if scan_time_profile == "physical_rolling"
        else -1e9
    )
    applied_cmd_stats = {
        "samples": 0,
        "fresh_nav_cmd_samples": 0,
        "linear_nonzero_samples": 0,
        "angular_nonzero_samples": 0,
        "linear_norm_sum": 0.0,
        "angular_abs_sum": 0.0,
        "linear_distance_integral_m": 0.0,
        "angular_signed_integral_rad": 0.0,
        "angular_abs_integral_rad": 0.0,
        "max_linear_norm": 0.0,
        "max_angular_abs": 0.0,
        "smoothed_samples": 0,
        "max_linear_delta_mps": 0.0,
        "max_angular_delta_radps": 0.0,
        "angular_saturation_samples": 0,
    }
    first_sim_yaw: float | None = None
    last_sim_yaw: float | None = None
    lingtu_system = None

    def _read_current_lidar_world_xyzi() -> tuple[np.ndarray, int]:
        cloud_world = engine.get_lidar_points()
        if cloud_world is None or len(cloud_world) == 0:
            return np.zeros((0, 4), dtype=np.float32), 0
        cloud_world_xyzi = _coerce_xyzi_cloud(cloud_world)
        moving_count = 0
        if current_moving_obstacle_points:
            moving_points_xyzi = _coerce_xyzi_cloud(current_moving_obstacle_points)
            if len(moving_points_xyzi):
                cloud_world_xyzi = np.vstack([cloud_world_xyzi, moving_points_xyzi])
                moving_count = int(len(moving_points_xyzi))
        return cloud_world_xyzi, moving_count

    def _record_moving_obstacle_publication(moving_point_count: int) -> None:
        nonlocal moving_obstacle_published_update_count
        nonlocal moving_obstacle_published_point_count_max
        moving_count = max(0, int(moving_point_count))
        if moving_count <= 0:
            return
        moving_obstacle_published_update_count += 1
        moving_obstacle_published_point_count_max = max(
            moving_obstacle_published_point_count_max,
            moving_count,
        )
        if len(moving_obstacle_samples) < 12 and current_moving_obstacle_boxes:
            sample_box = current_moving_obstacle_boxes[0]
            moving_obstacle_samples.append(
                {
                    "elapsed_s": round(float(elapsed_sim_for_duration), 3),
                    "box_count": int(len(current_moving_obstacle_boxes)),
                    "point_count": int(moving_count),
                    "first_box": {
                        "position": [
                            round(float(v), 4)
                            for v in sample_box.get("position", [])[:3]
                        ],
                        "half_size": [
                            round(float(v), 4)
                            for v in sample_box.get("half_size", [])[:3]
                        ],
                    },
                }
            )

    try:
        bridge.start()

        def _record_global_path(path: Any) -> None:
            global_path_counts.append(len(getattr(path, "poses", path) or []))
            global_path_summaries.append(_path_summary(path))
            if len(global_path_summaries) > 240:
                del global_path_summaries[:120]

        def _record_local_path(path: Any) -> None:
            local_path_counts.append(len(getattr(path, "poses", path) or []))
            local_path_summaries.append(_path_summary(path))
            if len(local_path_summaries) > 500:
                del local_path_summaries[:250]

        if run_lingtu_frontier:
            stack = build_fastlio2_frontier_stack(
                cloud_topic=TOPICS.registered_cloud,
                cmd_vel_topic=TOPICS.cmd_vel,
                frontier_goal_timeout=max(1.0, float(frontier_goal_timeout)),
                nav_max_linear_speed=float(nav_max_linear_speed),
                nav_max_angular_z=float(nav_max_angular_z),
                nav_turn_speed_yaw_rate_start=float(nav_turn_speed_yaw_rate_start),
                nav_turn_speed_min_scale=float(nav_turn_speed_min_scale),
            )
            lingtu_system = stack.system
            frontier = stack.frontier
            ogm = stack.occupancy_grid
            nav = stack.navigation
            local_planner = stack.local_planner
            frontier_module = frontier
            nav_module = nav
            local_planner_module = local_planner
            path_follower_module = stack.path_follower
            cmd_vel_mux_module = stack.cmd_vel_mux

            frontier.exploration_goal._add_callback(
                lambda goal: frontier_goals.append(
                    [
                        float(goal.pose.position.x),
                        float(goal.pose.position.y),
                        float(goal.pose.position.z),
                    ]
                )
            )
            frontier.frontiers._add_callback(lambda batch: frontier_batches.append(len(batch or [])))
            def _on_exploration_grid(grid: dict[str, Any]) -> None:
                exploration_grid_counts.append(dict(grid.get("counts") or {}))
                exploration_grid_metadata.append(
                    {
                        "source": str(grid.get("source") or ""),
                        "accumulation": str(grid.get("accumulation") or ""),
                        "semantic": str(grid.get("semantic") or ""),
                    }
                )
                exploration_area_samples.append(_exploration_area_sample(grid))

            ogm.exploration_grid._add_callback(_on_exploration_grid)
            nav.global_path._add_callback(_record_global_path)
            def _on_waypoint(_: Any) -> None:
                nonlocal waypoint_count
                waypoint_count += 1

            nav.waypoint._add_callback(_on_waypoint)
            def _on_mission_status(status: dict[str, Any]) -> None:
                if not isinstance(status, dict):
                    return
                state = str(status.get("state") or "").upper()
                if state not in {"SUCCESS", "FAILED", "STUCK", "CANCELLED"}:
                    return
                status_xy = _goal_xy(status)
                current_xy = None
                if frontier_module is not None:
                    try:
                        current_xy = _goal_xy((frontier_module.health() or {}).get("current_goal"))
                    except Exception:
                        current_xy = None
                matched = _goal_xy_matches(status_xy, current_xy)
                if not matched:
                    matched = any(
                        _goal_xy_matches(status_xy, _goal_xy(goal))
                        for goal in frontier_goals
                    )
                if not matched:
                    return
                key = (
                    f"{state}:"
                    f"{round(float(status_xy[0]), 2) if status_xy else 'na'}:"
                    f"{round(float(status_xy[1]), 2) if status_xy else 'na'}"
                )
                if key in frontier_terminal_keys:
                    return
                frontier_terminal_keys.add(key)
                frontier_mission_events.append({
                    "state": state,
                    "goal": list(status_xy) if status_xy is not None else None,
                    "failure_reason": status.get("failure_reason", ""),
                    "last_plan_report": status.get("last_plan_report", {}),
                    "ts": time.time(),
                })

            nav.mission_status._add_callback(_on_mission_status)
            local_planner.local_path._add_callback(_record_local_path)
            lingtu_system.start()
            lingtu_failed_modules = dict(lingtu_system.health().get("failed_modules") or {})
        elif run_lingtu_tare:
            stack = build_fastlio2_tare_stack(
                cloud_topic=TOPICS.registered_cloud,
                cmd_vel_topic=TOPICS.cmd_vel,
                tare_scenario=str(tare_scenario or "indoor"),
                tare_goal_timeout=max(1.0, float(tare_goal_timeout)),
                nav_max_linear_speed=float(nav_max_linear_speed),
                nav_max_angular_z=float(nav_max_angular_z),
                nav_turn_speed_yaw_rate_start=float(nav_turn_speed_yaw_rate_start),
                nav_turn_speed_min_scale=float(nav_turn_speed_min_scale),
            )
            lingtu_system = stack.system
            tare = stack.tare
            ogm = stack.occupancy_grid
            nav = stack.navigation
            local_planner = stack.local_planner
            tare_module = tare
            nav_module = nav
            local_planner_module = local_planner
            path_follower_module = stack.path_follower
            cmd_vel_mux_module = stack.cmd_vel_mux

            tare.exploration_goal._add_callback(
                lambda goal: tare_goals.append(
                    [
                        float(goal.pose.position.x),
                        float(goal.pose.position.y),
                        float(goal.pose.position.z),
                    ]
                )
            )
            tare.exploration_path._add_callback(
                lambda path: tare_path_counts.append(len(path or []))
            )
            tare.tare_stats._add_callback(
                lambda stats: tare_stats_samples.append(dict(stats or {}))
            )

            def _on_tare_exploration_grid(grid: dict[str, Any]) -> None:
                exploration_grid_counts.append(dict(grid.get("counts") or {}))
                exploration_grid_metadata.append(
                    {
                        "source": str(grid.get("source") or ""),
                        "accumulation": str(grid.get("accumulation") or ""),
                        "semantic": str(grid.get("semantic") or ""),
                    }
                )
                exploration_area_samples.append(_exploration_area_sample(grid))

            ogm.exploration_grid._add_callback(_on_tare_exploration_grid)
            nav.global_path._add_callback(_record_global_path)

            def _on_tare_waypoint(_: Any) -> None:
                nonlocal waypoint_count
                waypoint_count += 1

            nav.waypoint._add_callback(_on_tare_waypoint)

            def _on_tare_mission_status(status: dict[str, Any]) -> None:
                if not isinstance(status, dict):
                    return
                state = str(status.get("state") or "").upper()
                if state not in {"SUCCESS", "FAILED", "STUCK", "CANCELLED"}:
                    return
                status_xy = _goal_xy(status)
                matched = any(
                    _goal_xy_matches(status_xy, _goal_xy(goal))
                    for goal in tare_goals
                )
                if not matched:
                    return
                key = (
                    f"{state}:"
                    f"{round(float(status_xy[0]), 2) if status_xy else 'na'}:"
                    f"{round(float(status_xy[1]), 2) if status_xy else 'na'}"
                )
                if key in tare_terminal_keys:
                    return
                tare_terminal_keys.add(key)
                tare_mission_events.append({
                    "state": state,
                    "goal": list(status_xy) if status_xy is not None else None,
                    "failure_reason": status.get("failure_reason", ""),
                    "last_plan_report": status.get("last_plan_report", {}),
                    "ts": time.time(),
                })

            nav.mission_status._add_callback(_on_tare_mission_status)
            local_planner.local_path._add_callback(_record_local_path)
            lingtu_system.start()
            lingtu_failed_modules = dict(lingtu_system.health().get("failed_modules") or {})
        elif run_lingtu_inspection:
            stack = build_fastlio2_inspection_stack(
                cloud_topic=TOPICS.registered_cloud,
                cmd_vel_topic=TOPICS.cmd_vel,
                planner_backend=str(inspection_planner or "astar"),
                tomogram=str(inspection_tomogram or ""),
                inspection_goal_timeout=max(1.0, float(inspection_goal_timeout)),
                nav_max_linear_speed=float(nav_max_linear_speed),
                nav_max_angular_z=float(nav_max_angular_z),
                nav_turn_speed_yaw_rate_start=float(nav_turn_speed_yaw_rate_start),
                nav_turn_speed_min_scale=float(nav_turn_speed_min_scale),
            )
            lingtu_system = stack.system
            nav = stack.navigation
            local_planner = stack.local_planner
            nav_module = nav
            local_planner_module = local_planner
            path_follower_module = stack.path_follower
            cmd_vel_mux_module = stack.cmd_vel_mux
            inspection_goals_sent = [list(goal) for goal in inspection_goal_list]

            nav.global_path._add_callback(_record_global_path)

            def _on_inspection_waypoint(_: Any) -> None:
                nonlocal waypoint_count
                waypoint_count += 1

            nav.waypoint._add_callback(_on_inspection_waypoint)

            def _on_inspection_mission_status(status: dict[str, Any]) -> None:
                if not isinstance(status, dict):
                    return
                state = str(status.get("state") or "").upper()
                if state not in {"SUCCESS", "FAILED", "STUCK", "CANCELLED"}:
                    return
                status_xy = _goal_xy(status)
                matched = any(
                    _goal_xy_matches(status_xy, _goal_xy(goal))
                    for goal in inspection_goals_sent
                )
                if not matched:
                    return
                key = (
                    f"{state}:"
                    f"{round(float(status_xy[0]), 2) if status_xy else 'na'}:"
                    f"{round(float(status_xy[1]), 2) if status_xy else 'na'}"
                )
                if key in inspection_terminal_keys:
                    return
                inspection_terminal_keys.add(key)
                inspection_mission_events.append({
                    "state": state,
                    "goal": list(status_xy) if status_xy is not None else None,
                    "failure_reason": status.get("failure_reason", ""),
                    "last_plan_report": status.get("last_plan_report", {}),
                    "ts": time.time(),
                })

            nav.mission_status._add_callback(_on_inspection_mission_status)
            local_planner.local_path._add_callback(_record_local_path)
            lingtu_system.start()
            lingtu_failed_modules = dict(lingtu_system.health().get("failed_modules") or {})
        process = FastLio2Process(
            ros2_executable=ros2,
            config_path=config_path,
            log_path=log_path,
            env=os.environ.copy(),
        )
        process.start()
        deadline = time.time() + startup_sleep
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if process.poll() is not None:
                break
            subscription_counts = {
                "imu": imu_pub.get_subscription_count(),
                "cloud": cloud_pub.get_subscription_count(),
            }
            if subscription_counts["imu"] > 0 and subscription_counts["cloud"] > 0:
                break

        sim_start_wall = time.time()
        sim_loop_start_wall = sim_start_wall
        sim_loop_start_time = float(getattr(engine, "sim_time", 0.0))
        sim_loop_end_wall = sim_start_wall
        sim_loop_end_time = sim_loop_start_time
        ros_time_origin_s = time.time() - float(getattr(engine, "sim_time", 0.0))
        while True:
            elapsed_wall_for_duration = time.time() - sim_start_wall
            elapsed_sim_for_duration = float(getattr(engine, "sim_time", 0.0)) - sim_loop_start_time
            current_wall_timeout = _wall_timeout_status(
                elapsed_wall_for_duration,
                max_wall_time_s,
            )
            if current_wall_timeout["triggered"]:
                wall_timeout = current_wall_timeout
                runtime_faults.append(str(current_wall_timeout["fault"]))
                runtime_fault_events.append(
                    {
                        "kind": "wall_timeout",
                        "message": current_wall_timeout["fault"],
                        "elapsed_wall_s": current_wall_timeout["elapsed_wall_s"],
                        "elapsed_sim_s": round(float(elapsed_sim_for_duration), 3),
                        "max_wall_time_s": current_wall_timeout["max_wall_time_s"],
                    }
                )
                break
            if duration_clock == "sim":
                if elapsed_sim_for_duration >= duration:
                    break
            elif elapsed_wall_for_duration >= duration:
                break
            if process.poll() is not None:
                break

            loop_wall = time.time()
            if drive_source == "nav_cmd_vel":
                selected_cmd = _select_nav_cmd_for_step(
                    latest_nav_cmd=latest_nav_cmd,
                    now_s=time.time(),
                    cmd_vel_timeout_s=float(cmd_vel_timeout),
                )
                step_vx = float(selected_cmd["vx"] or 0.0)
                step_vy = float(selected_cmd["vy"] or 0.0)
                step_wz = float(selected_cmd["wz"] or 0.0)
                step_vx = float(np.clip(step_vx, -cmd_vel_linear_limit, cmd_vel_linear_limit))
                step_vy = float(np.clip(step_vy, -cmd_vel_linear_limit, cmd_vel_linear_limit))
                step_wz = float(np.clip(step_wz, -cmd_vel_angular_limit, cmd_vel_angular_limit))
                step_vx *= float(cmd_vel_sim_linear_scale)
                step_vy *= float(cmd_vel_sim_linear_scale)
                step_wz *= float(cmd_vel_sim_angular_scale)
                command_fresh = bool(selected_cmd["fresh"])
            else:
                step_vx, step_vy, step_wz = drive_vx, drive_vy, drive_wz
                command_fresh = True
            target_command = (float(step_vx), float(step_vy), float(step_wz))
            smoothed_command = _limit_command_delta(
                target=target_command,
                previous=previous_applied_command,
                dt_s=float(engine.control_dt),
                linear_accel_limit=float(cmd_vel_linear_accel_limit),
                angular_accel_limit=float(cmd_vel_angular_accel_limit),
            )
            linear_delta = math.hypot(
                smoothed_command[0] - previous_applied_command[0],
                smoothed_command[1] - previous_applied_command[1],
            )
            angular_delta = abs(smoothed_command[2] - previous_applied_command[2])
            if any(abs(smoothed_command[i] - target_command[i]) > 1e-6 for i in range(3)):
                applied_cmd_stats["smoothed_samples"] += 1
            applied_cmd_stats["max_linear_delta_mps"] = max(
                float(applied_cmd_stats["max_linear_delta_mps"]),
                linear_delta,
            )
            applied_cmd_stats["max_angular_delta_radps"] = max(
                float(applied_cmd_stats["max_angular_delta_radps"]),
                angular_delta,
            )
            step_vx, step_vy, step_wz = smoothed_command
            previous_applied_command = smoothed_command

            state = engine.step(
                VelocityCommand(
                    linear_x=step_vx,
                    linear_y=step_vy,
                    angular_z=step_wz,
                )
            )
            counts["sim_steps"] += 1
            current_sim_xyz = [float(v) for v in state.position[:3]]
            first_sim_xyz = first_sim_xyz or current_sim_xyz
            if prev_sim_xyz is not None:
                sim_path_length_m += math.dist(prev_sim_xyz, current_sim_xyz)
            prev_sim_xyz = current_sim_xyz
            last_sim_xyz = current_sim_xyz
            yaw = _yaw_from_quat_xyzw(state.orientation)
            first_sim_yaw = first_sim_yaw if first_sim_yaw is not None else yaw
            last_sim_yaw = yaw
            moving_obstacle_timed_trail.append(
                (
                    float(elapsed_sim_for_duration),
                    float(current_sim_xyz[0]),
                    float(current_sim_xyz[1]),
                    float(yaw),
                )
            )
            sim_pose_timed_trail.append(
                (
                    float(elapsed_sim_for_duration),
                    float(current_sim_xyz[0]),
                    float(current_sim_xyz[1]),
                    float(current_sim_xyz[2]),
                    float(yaw),
                )
            )
            if len(sim_pose_timed_trail) > 5000:
                del sim_pose_timed_trail[:2500]
            current_moving_obstacle_boxes = _live_moving_obstacle_boxes_from_pose(
                position_xy=(float(current_sim_xyz[0]), float(current_sim_xyz[1])),
                yaw_rad=float(yaw),
                elapsed_s=float(elapsed_sim_for_duration),
                mode=moving_obstacle_mode,
                count=int(moving_obstacle_count),
                start_s=float(moving_obstacle_start_s),
                duration_s=float(moving_obstacle_duration_s),
                period_s=float(moving_obstacle_period_s),
                forward_m=float(moving_obstacle_forward_m),
                forward_step_m=float(moving_obstacle_forward_step_m),
                lateral_phase_step_rad=float(moving_obstacle_lateral_phase_step_rad),
                lateral_amplitude_m=float(moving_obstacle_lateral_amplitude_m),
                along_amplitude_m=float(moving_obstacle_along_amplitude_m),
                radius_m=float(moving_obstacle_radius_m),
                height_m=float(moving_obstacle_height_m),
            )
            current_moving_obstacle_points = _live_moving_obstacle_points(
                current_moving_obstacle_boxes,
                spacing=float(moving_obstacle_point_spacing),
                intensity=float(moving_obstacle_intensity),
            )
            linear_norm = float(math.hypot(step_vx, step_vy))
            angular_abs = float(abs(step_wz))
            applied_cmd_stats["samples"] += 1
            applied_cmd_stats["fresh_nav_cmd_samples"] += int(command_fresh)
            applied_cmd_stats["linear_nonzero_samples"] += int(linear_norm > 1e-4)
            applied_cmd_stats["angular_nonzero_samples"] += int(angular_abs > 1e-4)
            applied_cmd_stats["angular_saturation_samples"] += int(
                angular_abs >= float(nav_max_angular_z) * 0.98
                if float(nav_max_angular_z) > 0.0
                else False
            )
            applied_cmd_stats["linear_norm_sum"] += linear_norm
            applied_cmd_stats["angular_abs_sum"] += angular_abs
            applied_cmd_stats["linear_distance_integral_m"] += (
                linear_norm * float(engine.control_dt)
            )
            applied_cmd_stats["angular_signed_integral_rad"] += (
                float(step_wz) * float(engine.control_dt)
            )
            applied_cmd_stats["angular_abs_integral_rad"] += (
                angular_abs * float(engine.control_dt)
            )
            applied_cmd_stats["max_linear_norm"] = max(
                float(applied_cmd_stats["max_linear_norm"]),
                linear_norm,
            )
            applied_cmd_stats["max_angular_abs"] = max(
                float(applied_cmd_stats["max_angular_abs"]),
                angular_abs,
            )
            command_diagnostic_samples.append(
                {
                    "sim_time_s": round(float(elapsed_sim_for_duration), 3),
                    "linear_x": round(float(step_vx), 4),
                    "linear_y": round(float(step_vy), 4),
                    "angular_z": round(float(step_wz), 4),
                    "source": str(drive_source),
                    "fresh_nav_cmd": bool(command_fresh),
                }
            )
            if len(command_diagnostic_samples) > 5000:
                del command_diagnostic_samples[:2500]

            stamp = _stamp_from_seconds(ros_time_origin_s + float(engine.sim_time), RosTime)
            if demo_truth_nav:
                if truth_nav_origin_world_xy is None and first_sim_xyz is not None:
                    truth_nav_origin_world_xy = (
                        float(first_sim_xyz[0]),
                        float(first_sim_xyz[1]),
                    )
                    truth_nav_origin_source = "first_sim_pose"
                nav_state = _state_in_map_frame(state, truth_nav_origin_world_xy)
                tf_broadcaster.sendTransform(
                    _make_odom_body_tf(
                        state=nav_state,
                        stamp=stamp,
                        transform_cls=TransformStamped,
                    )
                )
                nav_msg = _make_sim_odometry_msg(
                    state=nav_state,
                    stamp=stamp,
                    odometry_cls=Odometry,
                )
                nav_bridge.publish_nav_odom(nav_msg)
            imu_msg = _make_imu_msg(
                state=state,
                prev_velocity=prev_velocity,
                dt=float(engine.control_dt),
                stamp=stamp,
                imu_cls=Imu,
                acc_mode=imu_acc_mode,
            )
            imu_pub.publish(imu_msg)
            counts["imu_published"] += 1
            imu_acc = getattr(imu_msg, "linear_acceleration", None)
            imu_gyro = getattr(imu_msg, "angular_velocity", None)
            if imu_acc is not None and imu_gyro is not None:
                imu_diagnostic_samples.append(
                    {
                        "sim_time_s": round(float(elapsed_sim_for_duration), 3),
                        "dt_s": round(float(engine.control_dt), 6),
                        "acc_norm": round(
                            float(
                                math.sqrt(
                                    float(imu_acc.x) ** 2
                                    + float(imu_acc.y) ** 2
                                    + float(imu_acc.z) ** 2
                                )
                            ),
                            6,
                        ),
                        "gyro_norm": round(
                            float(
                                math.sqrt(
                                    float(imu_gyro.x) ** 2
                                    + float(imu_gyro.y) ** 2
                                    + float(imu_gyro.z) ** 2
                                )
                            ),
                            6,
                        ),
                        "gyro_x_radps": round(float(imu_gyro.x), 6),
                        "gyro_y_radps": round(float(imu_gyro.y), 6),
                        "gyro_z_radps": round(float(imu_gyro.z), 6),
                    }
                )
                if len(imu_diagnostic_samples) > 5000:
                    del imu_diagnostic_samples[:2500]
            prev_velocity = np.asarray(state.linear_velocity, dtype=np.float64)

            if scan_time_profile == "physical_rolling":
                subscan_world_xyzi, subscan_moving_count = _read_current_lidar_world_xyzi()
                if len(subscan_world_xyzi) == 0:
                    counts["empty_cloud_frames"] += 1
                else:
                    subscan_sensor_xyzi = _world_xyzi_to_sensor_xyzi(
                        engine,
                        subscan_world_xyzi,
                    )
                    rolling_lidar_samples.append(
                        (
                            float(engine.sim_time),
                            subscan_sensor_xyzi,
                            subscan_world_xyzi,
                            int(subscan_moving_count),
                        )
                    )
                rolling_cutoff_s = (
                    float(engine.sim_time)
                    - float(lidar_period)
                    - max(float(getattr(engine, "control_dt", 0.0) or 0.0), 1e-6)
                )
                while rolling_lidar_samples and rolling_lidar_samples[0][0] < rolling_cutoff_s:
                    rolling_lidar_samples.pop(0)

            if engine.sim_time - prev_lidar_pub_sim_time >= lidar_period - 1e-6:
                scan_stamp = stamp
                if scan_time_profile == "physical_rolling":
                    scan_start_s = float(engine.sim_time) - float(lidar_period)
                    scan_stamp = _stamp_from_seconds(
                        ros_time_origin_s + scan_start_s,
                        RosTime,
                    )
                    (
                        cloud_sensor,
                        cloud_world_xyzi,
                        relative_times_s,
                        moving_point_count_for_scan,
                        selected_subscans,
                    ) = _physical_rolling_scan_from_samples(
                        rolling_lidar_samples,
                        scan_start_s=scan_start_s,
                        scan_end_s=float(engine.sim_time),
                    )
                    rolling_lidar_subscan_counts.append(int(selected_subscans))
                    if len(rolling_lidar_subscan_counts) > 5000:
                        del rolling_lidar_subscan_counts[:2500]
                else:
                    cloud_world_xyzi, moving_point_count_for_scan = _read_current_lidar_world_xyzi()
                    cloud_sensor = (
                        _world_xyzi_to_sensor_xyzi(engine, cloud_world_xyzi)
                        if len(cloud_world_xyzi)
                        else np.zeros((0, 4), dtype=np.float32)
                    )
                    relative_times_s = _relative_times_for_scan(
                        len(cloud_sensor),
                        lidar_period,
                        scan_time_profile=scan_time_profile,
                    )
                if len(cloud_sensor) == 0:
                    counts["empty_cloud_frames"] += 1
                else:
                    _record_moving_obstacle_publication(moving_point_count_for_scan)
                    latest_raw_world_points = _sample_points(
                        cloud_world_xyzi,
                        4500,
                    )
                    point_counts.append(int(len(cloud_sensor)))
                    latest_scan_relative_times_s = [
                        float(value) for value in relative_times_s.tolist()
                    ]
                    if fastlio_lidar_input == "livox_custom_msg":
                        cloud_pub.publish(
                            _make_livox_custom_msg(
                                points_xyzi=cloud_sensor,
                                stamp=scan_stamp,
                                frame_id=SIM_LIDAR_FRAME_ID,
                                custom_msg_cls=CustomMsg,
                                custom_point_cls=CustomPoint,
                                relative_times_s=relative_times_s,
                            )
                        )
                    else:
                        cloud_pub.publish(
                            _make_pointcloud2(
                                points_xyzi=cloud_sensor,
                                stamp=scan_stamp,
                                frame_id=SIM_LIDAR_FRAME_ID,
                                pointcloud_cls=PointCloud2,
                                pointfield_cls=PointField,
                                relative_times_s=relative_times_s,
                            )
                    )
                    if demo_truth_nav:
                        cloud_body = _sensor_xyzi_to_body_xyzi(
                            cloud_sensor,
                            mujoco_lidar_extrinsic,
                        )
                        registered_msg = _make_pointcloud2(
                            points_xyzi=cloud_body,
                            stamp=scan_stamp,
                            frame_id=SIM_NAV_REGISTERED_CLOUD_FRAME_ID,
                            pointcloud_cls=PointCloud2,
                            pointfield_cls=PointField,
                            relative_times_s=relative_times_s,
                        )
                        nav_bridge.publish_nav_registered_cloud(registered_msg)
                        map_msg = _make_pointcloud2(
                            points_xyzi=cloud_world_xyzi,
                            stamp=scan_stamp,
                            frame_id=SIM_FASTLIO_LIVE_MAP_FRAME_ID,
                            pointcloud_cls=PointCloud2,
                            pointfield_cls=PointField,
                            relative_times_s=relative_times_s,
                        )
                        nav_bridge.publish_nav_map_cloud(map_msg)
                    counts["cloud_published"] += 1
                prev_lidar_pub_sim_time = float(engine.sim_time)

            rclpy.spin_once(node, timeout_sec=0.0)
            elapsed_sim_wall = time.time() - sim_start_wall
            if elapsed_sim_wall >= next_runtime_guard_s:
                next_runtime_guard_s += 2.0
                if (
                    nav_bridge.first_odom_xyz
                    and nav_bridge.last_odom_xyz
                    and first_sim_xyz
                    and last_sim_xyz
                ):
                    first_aligned_sim = None
                    last_aligned_sim = None
                    if (
                        nav_bridge.first_odom_stamp_s is not None
                        and nav_bridge.last_odom_stamp_s is not None
                    ):
                        first_aligned_sim = _nearest_sim_pose_sample(
                            sim_pose_timed_trail,
                            target_sim_time_s=(
                                float(nav_bridge.first_odom_stamp_s)
                                - float(ros_time_origin_s)
                            ),
                        )
                        last_aligned_sim = _nearest_sim_pose_sample(
                            sim_pose_timed_trail,
                            target_sim_time_s=(
                                float(nav_bridge.last_odom_stamp_s)
                                - float(ros_time_origin_s)
                            ),
                        )
                    time_aligned_guard = bool(first_aligned_sim and last_aligned_sim)
                    guard_first_sim_xyz = (
                        first_aligned_sim["xyz"] if first_aligned_sim else first_sim_xyz
                    )
                    guard_last_sim_xyz = (
                        last_aligned_sim["xyz"] if last_aligned_sim else last_sim_xyz
                    )
                    guard_first_sim_yaw = (
                        float(first_aligned_sim["yaw"])
                        if first_aligned_sim
                        else first_sim_yaw
                    )
                    guard_last_sim_yaw = (
                        float(last_aligned_sim["yaw"])
                        if last_aligned_sim
                        else last_sim_yaw
                    )
                    guard_alignment = {
                        "time_aligned": time_aligned_guard,
                        "first_dt_s": (
                            _round_float(first_aligned_sim.get("dt_s"))
                            if first_aligned_sim
                            else None
                        ),
                        "last_dt_s": (
                            _round_float(last_aligned_sim.get("dt_s"))
                            if last_aligned_sim
                            else None
                        ),
                    }
                    guard_fastlio_m = math.dist(
                        nav_bridge.first_odom_xyz,
                        nav_bridge.last_odom_xyz,
                    )
                    guard_sim_m = math.dist(guard_first_sim_xyz, guard_last_sim_xyz)
                    guard_motion = _motion_consistency_report(
                        fastlio2_moved_m=guard_fastlio_m,
                        fastlio2_path_length_m=None,
                        sim_moved_m=guard_sim_m,
                        sim_path_length_m=None,
                    )
                    fault_kind = ""
                    fault_message = ""
                    if (
                        not bool(guard_motion.get("ok"))
                        and (guard_sim_m > 0.05 or guard_fastlio_m > 0.5)
                    ):
                        fault_kind = "motion"
                        fault_message = (
                            "runtime Fast-LIO motion divergence "
                            f"(fastlio2={guard_fastlio_m:.3f}m, "
                            f"sim={guard_sim_m:.3f}m, "
                            f"allowed_error={float(guard_motion.get('max_allowed_motion_error_m') or 0.0):.3f}m)"
                        )
                    guard_z_error = abs(
                        float(nav_bridge.last_odom_xyz[2] - nav_bridge.first_odom_xyz[2])
                        - float(guard_last_sim_xyz[2] - guard_first_sim_xyz[2])
                    )
                    if not fault_kind and guard_z_error > float(max_fastlio_z_drift_m):
                        fault_kind = "z"
                        fault_message = (
                            "runtime Fast-LIO Z drift "
                            f"(error={guard_z_error:.3f}m, "
                            f"allowed={float(max_fastlio_z_drift_m):.3f}m)"
                        )
                    if (
                        not fault_kind
                        and
                        nav_bridge.first_odom_yaw is not None
                        and nav_bridge.last_odom_yaw is not None
                        and first_sim_yaw is not None
                        and last_sim_yaw is not None
                    ):
                        guard_odom_dyaw = _angle_delta_rad(
                            nav_bridge.last_odom_yaw,
                            nav_bridge.first_odom_yaw,
                        )
                        guard_sim_dyaw = _angle_delta_rad(
                            guard_last_sim_yaw,
                            guard_first_sim_yaw,
                        )
                        guard_yaw_error = abs(
                            _angle_delta_rad(guard_odom_dyaw, guard_sim_dyaw)
                        )
                        if guard_yaw_error > float(max_fastlio_yaw_drift_rad):
                            fault_kind = "yaw"
                            fault_message = (
                                "runtime Fast-LIO yaw drift "
                                f"(error={guard_yaw_error:.3f}rad, "
                                f"allowed={float(max_fastlio_yaw_drift_rad):.3f}rad)"
                            )
                    fault_status = _update_runtime_fault_streak(
                        runtime_fault_streaks,
                        kind=fault_kind,
                        confirm_samples=runtime_fault_confirm_samples,
                    )
                    if fault_kind:
                        runtime_fault_events.append(
                            {
                                "kind": fault_kind,
                                "message": fault_message,
                                "streak": int(fault_status["streak"]),
                                "confirmed": bool(fault_status["confirmed"]),
                                "elapsed_sim_s": round(float(elapsed_sim_for_duration), 3),
                                "elapsed_wall_s": round(float(elapsed_sim_wall), 3),
                                "time_alignment": guard_alignment,
                            }
                        )
                        if len(runtime_fault_events) > 200:
                            del runtime_fault_events[:100]
                        if fault_status["confirmed"]:
                            runtime_faults.append(fault_message)
                            _append_navigation_diagnostic_sample(
                                elapsed_sim_s=elapsed_sim_for_duration,
                                elapsed_wall_s=elapsed_sim_wall,
                                command_fresh=command_fresh,
                            )
                            break
            if (
                (run_lingtu_frontier or run_lingtu_tare or run_lingtu_inspection)
                and elapsed_sim_for_duration >= next_navigation_diagnostic_sample_s
            ):
                _append_navigation_diagnostic_sample(
                    elapsed_sim_s=elapsed_sim_for_duration,
                    elapsed_wall_s=elapsed_sim_wall,
                    command_fresh=command_fresh,
                )
                next_navigation_diagnostic_sample_s += navigation_diagnostic_sample_period_s
            if show_mujoco_window and elapsed_sim_wall >= next_live_window_s:
                next_live_window_s += 1.0 / max(float(mujoco_window_fps), 1.0)
                try:
                    if live_cv2 is None:
                        import cv2 as _cv2  # type: ignore

                        live_cv2 = _cv2
                    overview = _render_mujoco_overview(engine, video_render_state)
                    if isinstance(overview, np.ndarray) and overview.size:
                        if not live_window_created:
                            live_cv2.namedWindow(live_window_name, live_cv2.WINDOW_NORMAL)
                            live_cv2.resizeWindow(live_window_name, 640, 360)
                            live_cv2.moveWindow(live_window_name, 16, 64)
                            live_window_created = True
                        frame_bgr = live_cv2.cvtColor(
                            np.asarray(overview, dtype=np.uint8),
                            live_cv2.COLOR_RGB2BGR,
                        )
                        live_cv2.imshow(live_window_name, frame_bgr)
                        live_cv2.waitKey(1)
                        live_window_frames += 1
                    elif video_render_state.get("error"):
                        live_window_error = str(video_render_state["error"])
                        show_mujoco_window = False
                except Exception as exc:
                    live_window_error = f"{type(exc).__name__}: {exc}"
                    show_mujoco_window = False
            if video_out_path is not None and elapsed_sim_wall >= next_video_sample_s:
                latest_status = bridge_statuses[-1] if bridge_statuses else {}
                current_odom_m = (
                    math.dist(nav_bridge.first_odom_xyz, nav_bridge.last_odom_xyz)
                    if nav_bridge.first_odom_xyz and nav_bridge.last_odom_xyz
                    else 0.0
                )
                current_sim_m = (
                    math.dist(first_sim_xyz, last_sim_xyz)
                    if first_sim_xyz and last_sim_xyz
                    else 0.0
                )
                latest_area = (
                    nav_bridge.nav_map_cloud_area_samples[-1]
                    if nav_bridge.nav_map_cloud_area_samples
                    else {}
                )
                video_samples.append({
                    "t_s": float(elapsed_sim_wall),
                    "overview_rgb": _render_mujoco_overview(engine, video_render_state),
                    "raw_points": latest_raw_world_points.copy(),
                    "map_points": latest_map_points.copy(),
                    "moving_obstacle_points": _sample_points(
                        np.asarray(current_moving_obstacle_points, dtype=np.float32),
                        1200,
                    ),
                    "moving_obstacle_box_count": int(len(current_moving_obstacle_boxes)),
                    "raw_point_count": int(point_counts[-1]) if point_counts else 0,
                    "map_cloud_count": len(nav_bridge.nav_map_cloud_out),
                    "nav_odom_count": len(nav_bridge.nav_odom_out),
                    "nav_map_count": len(nav_bridge.nav_map_cloud_out),
                    "nav_odom_frame": str(
                        getattr(getattr(nav_bridge.nav_odom_out[-1], "header", None), "frame_id", "")
                        if nav_bridge.nav_odom_out
                        else SIM_NAV_ODOMETRY_FRAME_ID
                    ),
                    "nav_map_frame": str(
                        getattr(getattr(nav_bridge.nav_map_cloud_out[-1], "header", None), "frame_id", "")
                        if nav_bridge.nav_map_cloud_out
                        else SIM_FASTLIO_LIVE_MAP_FRAME_ID
                    ),
                    "sim_xy": np.asarray(last_sim_xyz[:2], dtype=np.float32)
                    if last_sim_xyz
                    else np.array([np.nan, np.nan], dtype=np.float32),
                    "odom_xy": np.asarray(nav_bridge.last_odom_xyz[:2], dtype=np.float32)
                    if nav_bridge.last_odom_xyz
                    else np.array([np.nan, np.nan], dtype=np.float32),
                    "sim_xyz": np.asarray(last_sim_xyz[:3], dtype=np.float32)
                    if last_sim_xyz
                    else np.array([np.nan, np.nan, np.nan], dtype=np.float32),
                    "odom_xyz": np.asarray(nav_bridge.last_odom_xyz[:3], dtype=np.float32)
                    if nav_bridge.last_odom_xyz
                    else np.array([np.nan, np.nan, np.nan], dtype=np.float32),
                    "sim_moved_m": float(current_sim_m),
                    "odom_moved_m": float(current_odom_m),
                    "map_area_m2": float(latest_area.get("xy_area_m2") or 0.0),
                    "slam_state": str(latest_status.get("state") or "UNKNOWN"),
                })
                next_video_sample_s += video_sample_period_s
            if (
                run_lingtu_frontier
                and frontier_module is not None
                and not frontier_started_after_slam_ready
            ):
                latest_state = (
                    str((bridge_statuses[-1] or {}).get("state") or "")
                    if bridge_statuses
                    else ""
                )
                live_nav_inputs_ready = (
                    len(nav_bridge.nav_odom_out) > 0
                    and len(nav_bridge.nav_registered_cloud_out) > 0
                    and len(nav_bridge.nav_map_cloud_out) > 0
                )
                live_slam_ready = (
                    live_nav_inputs_ready
                    if demo_truth_nav
                    else latest_state == "TRACKING" and live_nav_inputs_ready
                )
                if live_slam_ready:
                    if frontier_ready_since is None:
                        frontier_ready_since = time.time()
                    if time.time() - frontier_ready_since >= frontier_start_delay:
                        frontier_begin_status = str(frontier_module.begin_exploration())
                        frontier_started_after_slam_ready = frontier_begin_status in {
                            "started",
                            "already_running",
                        }
                else:
                    frontier_ready_since = None
            if (
                run_lingtu_tare
                and tare_module is not None
                and not tare_started_after_slam_ready
            ):
                latest_state = (
                    str((bridge_statuses[-1] or {}).get("state") or "")
                    if bridge_statuses
                    else ""
                )
                live_nav_inputs_ready = (
                    len(nav_bridge.nav_odom_out) > 0
                    and len(nav_bridge.nav_registered_cloud_out) > 0
                    and len(nav_bridge.nav_map_cloud_out) > 0
                )
                planner_map_ready = _nav_planner_has_live_map(nav_module)
                live_slam_ready = (
                    live_nav_inputs_ready and planner_map_ready
                    if demo_truth_nav
                    else latest_state == "TRACKING"
                    and live_nav_inputs_ready
                    and planner_map_ready
                )
                if live_slam_ready:
                    if tare_ready_since is None:
                        tare_ready_since = time.time()
                    if time.time() - tare_ready_since >= tare_start_delay:
                        raw_status = str(tare_module.start_tare_exploration())
                        try:
                            parsed_status = json.loads(raw_status)
                            tare_start_status = str(
                                parsed_status.get("status") or raw_status
                            )
                        except Exception:
                            tare_start_status = raw_status
                        tare_started_after_slam_ready = tare_start_status in {
                            "started",
                            "already_running",
                        }
                else:
                    tare_ready_since = None
            if (
                run_lingtu_inspection
                and nav_module is not None
                and not inspection_started_after_slam_ready
            ):
                latest_state = (
                    str((bridge_statuses[-1] or {}).get("state") or "")
                    if bridge_statuses
                    else ""
                )
                live_nav_inputs_ready = (
                    len(nav_bridge.nav_odom_out) > 0
                    and len(nav_bridge.nav_registered_cloud_out) > 0
                    and len(nav_bridge.nav_map_cloud_out) > 0
                )
                planner_map_ready = _nav_planner_has_live_map(nav_module)
                live_slam_ready = (
                    live_nav_inputs_ready and planner_map_ready
                    if demo_truth_nav
                    else latest_state == "TRACKING"
                    and live_nav_inputs_ready
                    and planner_map_ready
                )
                if live_slam_ready:
                    if inspection_ready_since is None:
                        inspection_ready_since = time.time()
                    if time.time() - inspection_ready_since >= inspection_start_delay:
                        raw_status = str(nav_module.start_patrol(json.dumps(inspection_goal_list)))
                        try:
                            parsed_status = json.loads(raw_status)
                            inspection_start_status = str(
                                parsed_status.get("status") or raw_status
                            )
                        except Exception:
                            inspection_start_status = raw_status
                        inspection_started_after_slam_ready = inspection_start_status in {
                            "patrolling",
                            "started",
                            "already_running",
                        }
                else:
                    inspection_ready_since = None
            target_dt = float(engine.control_dt)
            elapsed = time.time() - loop_wall
            if elapsed < target_dt:
                deadline = time.time() + (target_dt - elapsed)
                while time.time() < deadline:
                    rclpy.spin_once(node, timeout_sec=min(0.005, max(0.0, deadline - time.time())))

        sim_loop_end_wall = time.time()
        sim_loop_end_time = float(getattr(engine, "sim_time", sim_loop_start_time))
        source_end_bridge_status = bridge_statuses[-1] if bridge_statuses else None
        deadline = time.time() + settle_sleep
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        try:
            topic_output = subprocess.check_output(
                [ros2, "topic", "list"],
                text=True,
                timeout=5,
            )
            visible_ros_topics = sorted(
                line.strip() for line in topic_output.splitlines() if line.strip()
            )
        except Exception as exc:
            visible_ros_topic_error = f"{type(exc).__name__}: {exc}"
    finally:
        if nav_module is not None:
            try:
                final_navigation_health = dict((nav_module.health() or {}).get("navigation") or {})
            except Exception as exc:
                final_navigation_health = {"error": f"{type(exc).__name__}: {exc}"}
            try:
                final_navigation_status = json.loads(str(nav_module.get_navigation_status()))
            except Exception as exc:
                final_navigation_status = {"error": f"{type(exc).__name__}: {exc}"}
        if frontier_module is not None:
            try:
                final_frontier_health = dict(frontier_module.health() or {})
            except Exception as exc:
                final_frontier_health = {"error": f"{type(exc).__name__}: {exc}"}
        if tare_module is not None:
            try:
                final_tare_health = dict(tare_module.health() or {})
            except Exception as exc:
                final_tare_health = {"error": f"{type(exc).__name__}: {exc}"}
            try:
                final_tare_status = json.loads(str(tare_module.get_tare_status()))
            except Exception as exc:
                final_tare_status = {"error": f"{type(exc).__name__}: {exc}"}
        if local_planner_module is not None:
            try:
                final_local_planner_health = dict(local_planner_module.health() or {})
            except Exception as exc:
                final_local_planner_health = {"error": f"{type(exc).__name__}: {exc}"}
        if path_follower_module is not None:
            try:
                final_path_follower_health = dict(path_follower_module.health() or {})
            except Exception as exc:
                final_path_follower_health = {"error": f"{type(exc).__name__}: {exc}"}
        if cmd_vel_mux_module is not None:
            try:
                final_cmd_vel_mux_health = dict(cmd_vel_mux_module.health() or {})
            except Exception as exc:
                final_cmd_vel_mux_health = {"error": f"{type(exc).__name__}: {exc}"}
        if lingtu_system is not None:
            try:
                lingtu_system.stop(timeout_per_module=2.0)
            except Exception:
                pass
            try:
                from core.ros2_context import shutdown_shared_executor

                shutdown_shared_executor()
            except Exception:
                pass
        if process is not None:
            process.stop(timeout_s=3)
        bridge.stop()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        renderer = video_render_state.get("renderer")
        if renderer is not None:
            try:
                renderer.close()
            except Exception:
                pass
        if live_cv2 is not None and live_window_created:
            try:
                live_cv2.destroyWindow(live_window_name)
            except Exception:
                pass
        if sim_loop_end_wall is None:
            sim_loop_end_wall = time.time()
            sim_loop_end_time = float(getattr(engine, "sim_time", sim_loop_start_time))
        engine.close()

    sim_wall_time_s = max(
        0.0,
        float(sim_loop_end_wall or time.time()) - float(sim_loop_start_wall or started),
    )
    sim_time_s = max(0.0, float(sim_loop_end_time) - float(sim_loop_start_time))
    sim_realtime_factor = sim_time_s / max(sim_wall_time_s, 1e-6)
    if not wall_timeout.get("triggered"):
        wall_timeout = _wall_timeout_status(sim_wall_time_s, max_wall_time_s)
    states = [str(item.get("state")) for item in bridge_statuses]
    source_end_state = str(source_end_bridge_status.get("state")) if source_end_bridge_status else ""
    shutdown_bridge_status = bridge_statuses[-1] if bridge_statuses else {}
    odom_out = nav_bridge.odom_out
    registered_cloud_out = nav_bridge.registered_cloud_out
    map_cloud_out = nav_bridge.map_cloud_out
    nav_odom_out = nav_bridge.nav_odom_out
    nav_registered_cloud_out = nav_bridge.nav_registered_cloud_out
    nav_map_cloud_out = nav_bridge.nav_map_cloud_out
    nav_map_cloud_area_samples = nav_bridge.nav_map_cloud_area_samples
    first_odom_xyz = nav_bridge.first_odom_xyz
    last_odom_xyz = nav_bridge.last_odom_xyz
    first_odom_yaw = nav_bridge.first_odom_yaw
    last_odom_yaw = nav_bridge.last_odom_yaw
    odom_path_length_m = nav_bridge.odom_path_length_m
    moved_m = math.dist(first_odom_xyz, last_odom_xyz) if first_odom_xyz and last_odom_xyz else None
    sim_moved_m = math.dist(first_sim_xyz, last_sim_xyz) if first_sim_xyz and last_sim_xyz else None
    fastlio_z_delta = (
        float(last_odom_xyz[2] - first_odom_xyz[2])
        if first_odom_xyz and last_odom_xyz
        else None
    )
    sim_z_delta = (
        float(last_sim_xyz[2] - first_sim_xyz[2])
        if first_sim_xyz and last_sim_xyz
        else None
    )
    z_delta_error = (
        abs(float(fastlio_z_delta) - float(sim_z_delta))
        if fastlio_z_delta is not None and sim_z_delta is not None
        else None
    )
    z_consistency = {
        "checked": bool(z_delta_error is not None),
        "ok": bool(
            z_delta_error is not None
            and float(z_delta_error) <= float(max_fastlio_z_drift_m)
        ),
        "fastlio2_z_delta_m": fastlio_z_delta,
        "sim_z_delta_m": sim_z_delta,
        "z_delta_error_m": z_delta_error,
        "max_allowed_z_drift_m": float(max_fastlio_z_drift_m),
        "first_odom_z": first_odom_xyz[2] if first_odom_xyz else None,
        "last_odom_z": last_odom_xyz[2] if last_odom_xyz else None,
        "first_sim_z": first_sim_xyz[2] if first_sim_xyz else None,
        "last_sim_z": last_sim_xyz[2] if last_sim_xyz else None,
    }
    fastlio_yaw_delta = (
        _angle_delta_rad(last_odom_yaw, first_odom_yaw)
        if first_odom_yaw is not None and last_odom_yaw is not None
        else None
    )
    sim_yaw_delta = (
        _angle_delta_rad(last_sim_yaw, first_sim_yaw)
        if first_sim_yaw is not None and last_sim_yaw is not None
        else None
    )
    yaw_delta_error = (
        abs(_angle_delta_rad(fastlio_yaw_delta, sim_yaw_delta))
        if fastlio_yaw_delta is not None and sim_yaw_delta is not None
        else None
    )
    yaw_consistency = {
        "checked": bool(yaw_delta_error is not None),
        "ok": bool(
            yaw_delta_error is not None
            and float(yaw_delta_error) <= float(max_fastlio_yaw_drift_rad)
        ),
        "fastlio2_yaw_delta_rad": fastlio_yaw_delta,
        "sim_yaw_delta_rad": sim_yaw_delta,
        "yaw_delta_error_rad": yaw_delta_error,
        "max_allowed_yaw_drift_rad": float(max_fastlio_yaw_drift_rad),
        "first_odom_yaw_rad": first_odom_yaw,
        "last_odom_yaw_rad": last_odom_yaw,
        "first_sim_yaw_rad": first_sim_yaw,
        "last_sim_yaw_rad": last_sim_yaw,
    }
    process_returncode = process.returncode if process is not None else None
    algorithm_verified = len(odom_out) > 0 and len(map_cloud_out) > 0
    canonical_nav_outputs_verified = (
        len(nav_odom_out) > 0
        and len(nav_registered_cloud_out) > 0
        and len(nav_map_cloud_out) > 0
    )
    bridge_verified = source_end_state == "TRACKING" or any(state == "TRACKING" for state in states)
    nav_cmd_nonzero = sum(
        1
        for item in nav_cmd_vel_samples
        if abs(item["vx"]) > 1e-4 or abs(item["vy"]) > 1e-4 or abs(item["wz"]) > 1e-4
    )
    last_plan_report = (
        final_navigation_health.get("last_plan_report")
        if isinstance(final_navigation_health.get("last_plan_report"), dict)
        else {}
    )
    if not last_plan_report and isinstance(final_navigation_status.get("last_plan_report"), dict):
        last_plan_report = final_navigation_status.get("last_plan_report") or {}
    direct_goal_fallback = (
        final_navigation_status.get("direct_goal_fallback")
        if isinstance(final_navigation_status.get("direct_goal_fallback"), dict)
        else final_navigation_health.get("direct_goal_fallback")
    )
    if not isinstance(direct_goal_fallback, dict):
        direct_goal_fallback = {}
    primary_planner = str(last_plan_report.get("primary_planner") or "").lower()
    selected_planner = str(last_plan_report.get("selected_planner") or "").lower()
    planner_fallback_used = bool(
        last_plan_report.get("fallback_used")
        or last_plan_report.get("fallback_reason")
        or (
            primary_planner
            and selected_planner
            and primary_planner != selected_planner
        )
    )
    planner_repair_used = bool(last_plan_report.get("primary_replan"))
    frontier_success_events = [
        item for item in frontier_mission_events
        if str(item.get("state") or "").upper() == "SUCCESS"
    ]
    frontier_failed_events = [
        item for item in frontier_mission_events
        if str(item.get("state") or "").upper() in {"FAILED", "STUCK", "CANCELLED"}
    ]
    tare_success_events = [
        item for item in tare_mission_events
        if str(item.get("state") or "").upper() == "SUCCESS"
    ]
    tare_failed_events = [
        item for item in tare_mission_events
        if str(item.get("state") or "").upper() in {"FAILED", "STUCK", "CANCELLED"}
    ]
    inspection_success_events = [
        item for item in inspection_mission_events
        if str(item.get("state") or "").upper() == "SUCCESS"
    ]
    inspection_failed_events = [
        item for item in inspection_mission_events
        if str(item.get("state") or "").upper() in {"FAILED", "STUCK", "CANCELLED"}
    ]
    inspection_state = str(
        final_navigation_health.get("state")
        or final_navigation_status.get("state")
        or ""
    ).upper()
    inspection_patrol_total = int(
        final_navigation_health.get("patrol_total")
        or len(inspection_goals_sent)
        or 0
    )
    inspection_patrol_index = int(final_navigation_health.get("patrol_index") or 0)
    inspection_success_count = min(
        max(inspection_patrol_index, 0),
        inspection_patrol_total,
    )
    if (
        run_lingtu_inspection
        and inspection_state == "SUCCESS"
        and inspection_patrol_total > 0
        and inspection_patrol_index >= inspection_patrol_total
    ):
        inspection_success_count = inspection_patrol_total
    last_tare_stats = tare_stats_samples[-1] if tare_stats_samples else {}
    if not last_tare_stats and final_tare_status:
        last_tare_stats = final_tare_status
    frontier_blocked_goal_count = int(final_frontier_health.get("blocked_goal_count") or 0)
    nav_map_area = _area_growth(
        [float(item.get("xy_area_m2") or 0.0) for item in nav_map_cloud_area_samples]
    )
    explored_known_area = _area_growth(
        [float(item.get("known_m2") or 0.0) for item in exploration_area_samples]
    )
    explored_free_area = _area_growth(
        [float(item.get("free_m2") or 0.0) for item in exploration_area_samples]
    )
    exploration_coverage = _coverage_growth(exploration_area_samples)
    last_exploration_grid_meta = (
        exploration_grid_metadata[-1] if exploration_grid_metadata else {}
    )
    exploration_grid_accumulation = str(
        last_exploration_grid_meta.get("accumulation") or ""
    )
    exploration_grid_growth_is_acceptance_metric = (
        exploration_grid_accumulation != "rolling_local_window"
    )
    motion_consistency = _motion_consistency_report(
        fastlio2_moved_m=moved_m,
        fastlio2_path_length_m=round(float(odom_path_length_m), 4),
        sim_moved_m=sim_moved_m,
        sim_path_length_m=round(float(sim_path_length_m), 4),
    )
    cmd_samples = int(applied_cmd_stats["samples"])
    applied_command_summary = {
        "samples": cmd_samples,
        "fresh_nav_cmd_samples": int(applied_cmd_stats["fresh_nav_cmd_samples"]),
        "stale_nav_cmd_samples": int(
            max(0, cmd_samples - int(applied_cmd_stats["fresh_nav_cmd_samples"]))
        ),
        "linear_nonzero_samples": int(applied_cmd_stats["linear_nonzero_samples"]),
        "angular_nonzero_samples": int(applied_cmd_stats["angular_nonzero_samples"]),
        "mean_linear_norm": (
            round(float(applied_cmd_stats["linear_norm_sum"]) / cmd_samples, 4)
            if cmd_samples
            else 0.0
        ),
        "mean_angular_abs": (
            round(float(applied_cmd_stats["angular_abs_sum"]) / cmd_samples, 4)
            if cmd_samples
            else 0.0
        ),
        "max_linear_norm": round(float(applied_cmd_stats["max_linear_norm"]), 4),
        "max_angular_abs": round(float(applied_cmd_stats["max_angular_abs"]), 4),
        "smoothed_samples": int(applied_cmd_stats["smoothed_samples"]),
        "linear_accel_limit_mps2": float(cmd_vel_linear_accel_limit),
        "angular_accel_limit_radps2": float(cmd_vel_angular_accel_limit),
        "max_linear_delta_mps": round(
            float(applied_cmd_stats["max_linear_delta_mps"]),
            4,
        ),
        "max_angular_delta_radps": round(
            float(applied_cmd_stats["max_angular_delta_radps"]),
            4,
        ),
        "angular_saturation_samples": int(
            applied_cmd_stats["angular_saturation_samples"]
        ),
        "linear_distance_integral_m": round(
            float(applied_cmd_stats["linear_distance_integral_m"]),
            4,
        ),
        "angular_signed_integral_rad": round(
            float(applied_cmd_stats["angular_signed_integral_rad"]),
            4,
        ),
        "angular_abs_integral_rad": round(
            float(applied_cmd_stats["angular_abs_integral_rad"]),
            4,
        ),
        "first_sim_yaw_rad": first_sim_yaw,
        "last_sim_yaw_rad": last_sim_yaw,
        "sim_yaw_delta_rad": (
            _angle_delta_rad(last_sim_yaw, first_sim_yaw)
            if first_sim_yaw is not None and last_sim_yaw is not None
            else None
        ),
    }
    navigation_diagnostics = {
        "sample_period_s": float(navigation_diagnostic_sample_period_s),
        "sample_count": len(navigation_diagnostic_samples),
        "samples_tail": navigation_diagnostic_samples[-60:],
        "last_sample": navigation_diagnostic_samples[-1] if navigation_diagnostic_samples else {},
    }
    diagnostic_segments: list[dict[str, Any]] = []
    for index in range(1, len(navigation_diagnostic_samples)):
        previous = navigation_diagnostic_samples[index - 1]
        current = navigation_diagnostic_samples[index]
        prev_sim = _round_xyz(previous.get("sim_xyz"))
        cur_sim = _round_xyz(current.get("sim_xyz"))
        prev_fastlio = _round_xyz(previous.get("fastlio2_xyz"))
        cur_fastlio = _round_xyz(current.get("fastlio2_xyz"))
        segment: dict[str, Any] = {
            "segment": f"diagnostic_{index - 1}_to_{index}",
            "start_sim_time_s": previous.get("sim_time_s"),
            "end_sim_time_s": current.get("sim_time_s"),
            "z_delta_error_m": current.get("fastlio2_z_delta_error_m"),
            "yaw_delta_error_rad": current.get("fastlio2_yaw_delta_error_rad"),
        }
        if prev_sim is not None and cur_sim is not None:
            segment["sim_delta_m"] = round(float(math.dist(prev_sim, cur_sim)), 4)
        if prev_fastlio is not None and cur_fastlio is not None:
            segment["fastlio_delta_m"] = round(float(math.dist(prev_fastlio, cur_fastlio)), 4)
        if (
            prev_sim is not None
            and cur_sim is not None
            and prev_fastlio is not None
            and cur_fastlio is not None
        ):
            segment["segment_z_delta_error_m"] = round(
                abs(
                    float(cur_fastlio[2] - prev_fastlio[2])
                    - float(cur_sim[2] - prev_sim[2])
                ),
                4,
            )
        diagnostic_segments.append(segment)
    fastlio_large_loop_diagnostic_report = _fastlio_large_loop_diagnostic_report(
        segment_consistency=diagnostic_segments,
        imu_samples=imu_diagnostic_samples,
        scan_relative_times_s=latest_scan_relative_times_s,
        scan_time_profile=scan_time_profile,
        command_samples=command_diagnostic_samples,
    )
    expected_ros_topics = [
        TOPICS.raw_lidar_points,
        TOPICS.raw_imu,
        FASTLIO_REGISTERED_CLOUD_TOPIC,
        FASTLIO_MAP_CLOUD_TOPIC,
        FASTLIO_ODOMETRY_TOPIC,
        TOPICS.odometry,
        TOPICS.registered_cloud,
        TOPICS.map_cloud,
    ]
    if run_lingtu_frontier:
        expected_ros_topics.extend(
            [
                TOPICS.cmd_vel,
                TOPICS.exploration_grid,
                TOPICS.global_path,
                TOPICS.local_path,
            ]
        )
    if run_lingtu_tare:
        expected_ros_topics.extend(
            [
                TOPICS.cmd_vel,
                TOPICS.exploration_way_point,
                TOPICS.exploration_start,
                TOPICS.exploration_grid,
                TOPICS.global_path,
                TOPICS.local_path,
                TOPICS.state_estimation_at_scan,
                TOPICS.terrain_map,
                TOPICS.terrain_map_ext,
            ]
        )
    if run_lingtu_inspection:
        expected_ros_topics.extend(
            [
                TOPICS.cmd_vel,
                TOPICS.global_path,
                TOPICS.local_path,
            ]
        )
    visible_ros_topic_set = set(visible_ros_topics)
    missing_visible_ros_topics = [
        topic for topic in expected_ros_topics if topic not in visible_ros_topic_set
    ]
    moving_obstacle_clearance = _live_moving_obstacle_trail_clearance(
        timed_trail=moving_obstacle_timed_trail,
        robot_radius_m=float(moving_obstacle_robot_radius_m),
        mode=moving_obstacle_mode,
        count=int(moving_obstacle_count),
        start_s=float(moving_obstacle_start_s),
        duration_s=float(moving_obstacle_duration_s),
        period_s=float(moving_obstacle_period_s),
        forward_m=float(moving_obstacle_forward_m),
        forward_step_m=float(moving_obstacle_forward_step_m),
        lateral_phase_step_rad=float(moving_obstacle_lateral_phase_step_rad),
        lateral_amplitude_m=float(moving_obstacle_lateral_amplitude_m),
        along_amplitude_m=float(moving_obstacle_along_amplitude_m),
        radius_m=float(moving_obstacle_radius_m),
        height_m=float(moving_obstacle_height_m),
    )
    moving_obstacle_speed = _live_moving_obstacle_speed_bounds(
        period_s=float(moving_obstacle_period_s),
        lateral_amplitude_m=float(moving_obstacle_lateral_amplitude_m),
        along_amplitude_m=float(moving_obstacle_along_amplitude_m),
    )
    moving_obstacle_ok = (
        not moving_obstacle_enabled
        or (
            int(moving_obstacle_published_update_count) > 0
            and int(moving_obstacle_published_point_count_max) > 0
            and moving_obstacle_clearance.get("checked") is True
            and not bool(moving_obstacle_clearance.get("collision"))
        )
    )
    control_quality = _control_quality_report(
        applied_cmd_stats=applied_cmd_stats,
        max_yaw_per_meter=float(max_yaw_per_meter),
        max_angular_saturation_ratio=float(max_angular_saturation_ratio),
    )
    runtime_contract_definition, runtime_contract_errors = _mujoco_fastlio_contract_definition()
    topic_evidence = {
        TOPICS.raw_lidar_points: {
            "ok": counts["cloud_published"] > 0,
            "samples": int(counts["cloud_published"]),
        },
        TOPICS.raw_imu: {
            "ok": counts["imu_published"] > 0,
            "samples": int(counts["imu_published"]),
        },
        TOPICS.odometry: {"ok": len(nav_odom_out) > 0, "samples": len(nav_odom_out)},
        TOPICS.registered_cloud: {
            "ok": len(nav_registered_cloud_out) > 0,
            "samples": len(nav_registered_cloud_out),
        },
        TOPICS.map_cloud: {
            "ok": len(nav_map_cloud_out) > 0,
            "samples": len(nav_map_cloud_out),
        },
        TOPICS.global_path: {
            "ok": len(global_path_counts) > 0 and max(global_path_counts or [0]) > 0,
            "samples": len(global_path_counts),
            "max_poses": max(global_path_counts) if global_path_counts else 0,
        },
        TOPICS.local_path: {
            "ok": len(local_path_counts) > 0 and max(local_path_counts or [0]) > 0,
            "samples": len(local_path_counts),
            "max_poses": max(local_path_counts) if local_path_counts else 0,
        },
        TOPICS.cmd_vel: {
            "ok": nav_cmd_nonzero > 0,
            "samples": len(nav_cmd_vel_samples),
            "nonzero_samples": int(nav_cmd_nonzero),
        },
        FASTLIO_ODOMETRY_TOPIC: {"ok": len(odom_out) > 0, "samples": len(odom_out)},
        FASTLIO_REGISTERED_CLOUD_TOPIC: {
            "ok": len(registered_cloud_out) > 0,
            "samples": len(registered_cloud_out),
        },
        FASTLIO_MAP_CLOUD_TOPIC: {
            "ok": len(map_cloud_out) > 0,
            "samples": len(map_cloud_out),
        },
    }
    odom_body_samples = max(len(nav_odom_out), len(odom_out))
    frame_evidence = _mujoco_frame_evidence(
        odom_body_samples=odom_body_samples,
        odom_body_source="truth_nav_tf" if demo_truth_nav else "fastlio_odometry",
        static_tf_published=True,
    )
    navigation_required = bool(run_lingtu_frontier or run_lingtu_tare or run_lingtu_inspection)
    data_flow_evidence = _mujoco_data_flow_evidence(
        topic_evidence=topic_evidence,
        navigation_required=navigation_required,
    )
    runtime_contract = _mujoco_runtime_contract(
        definition=runtime_contract_definition,
        definition_errors=runtime_contract_errors,
        topic_evidence=topic_evidence,
        frame_evidence=frame_evidence,
        data_flow_evidence=data_flow_evidence,
    )
    hardware_safety = _mujoco_hardware_safety()
    runtime_evidence = validate_runtime_evidence(
        {
            "runtime_contract": runtime_contract,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "outputs": {
                "global_path_count": len(global_path_counts),
                "local_path_count": len(local_path_counts),
                "nav_cmd_vel_nonzero": int(nav_cmd_nonzero),
            },
            "hardware_safety": hardware_safety,
        },
        "mujoco_fastlio2_live",
        require_paths=navigation_required,
        require_command=navigation_required,
        require_frame_links=True,
        require_data_flow=True,
    )
    base_blockers: list[str] = []
    base_blockers.extend(runtime_faults)
    if not algorithm_verified:
        base_blockers.append("Fast-LIO did not publish both odometry and map cloud")
    if not bridge_verified and not demo_truth_nav:
        base_blockers.append("SlamBridgeModule never reached TRACKING")
    if counts["imu_published"] <= 0:
        base_blockers.append("no IMU samples published")
    if counts["cloud_published"] <= 0:
        base_blockers.append("no LiDAR cloud samples published")
    if visible_ros_topic_error:
        base_blockers.append(f"could not list live ROS topics: {visible_ros_topic_error}")
    if missing_visible_ros_topics:
        base_blockers.append(
            "missing live ROS topics: " + ", ".join(missing_visible_ros_topics)
        )
    if (
        motion_consistency["checked"]
        and not motion_consistency["ok"]
        and ((sim_moved_m or 0.0) > 0.05 or (moved_m or 0.0) > 3.0)
    ):
        base_blockers.append(
            "Fast-LIO odometry diverged from MuJoCo motion "
            f"(fastlio2={float(moved_m or 0.0):.3f}m, "
            f"sim={float(sim_moved_m or 0.0):.3f}m, "
            f"allowed={float(motion_consistency['max_allowed_fastlio2_moved_m'] or 0.0):.3f}m)"
        )
    if z_consistency["checked"] and not z_consistency["ok"]:
        base_blockers.append(
            "Fast-LIO odometry Z drift exceeded limit "
            f"(fastlio2_delta={float(fastlio_z_delta or 0.0):.3f}m, "
            f"sim_delta={float(sim_z_delta or 0.0):.3f}m, "
            f"error={float(z_delta_error or 0.0):.3f}m, "
            f"allowed={float(max_fastlio_z_drift_m):.3f}m)"
        )
    if yaw_consistency["checked"] and not yaw_consistency["ok"]:
        base_blockers.append(
            "Fast-LIO odometry yaw drift exceeded limit "
            f"(fastlio2_delta={float(fastlio_yaw_delta or 0.0):.3f}rad, "
            f"sim_delta={float(sim_yaw_delta or 0.0):.3f}rad, "
            f"error={float(yaw_delta_error or 0.0):.3f}rad, "
            f"allowed={float(max_fastlio_yaw_drift_rad):.3f}rad)"
        )
    if moving_obstacle_enabled:
        if moving_obstacle_published_update_count <= 0:
            base_blockers.append("moving obstacle point injections missing")
        if moving_obstacle_published_point_count_max <= 0:
            base_blockers.append("moving obstacle point cloud is empty")
        if moving_obstacle_clearance.get("checked") is not True:
            base_blockers.append("moving obstacle trail clearance was not checked")
        if moving_obstacle_clearance.get("collision") is True:
            base_blockers.append("moving obstacle trail collision is true")
    if (
        run_lingtu_frontier or run_lingtu_tare or run_lingtu_inspection
    ) and control_quality.get("ok") is not True:
        base_blockers.extend(str(item) for item in control_quality.get("blockers") or [])
    if not runtime_evidence.ok:
        base_blockers.extend(runtime_evidence.blockers)
    frontier_blockers: list[str] = []
    frontier_warnings: list[str] = []
    if run_lingtu_frontier:
        frontier_success_requirement_met = len(frontier_success_events) >= int(frontier_min_goals)
        if frontier_begin_status not in {"started", "already_running"}:
            frontier_blockers.append(f"frontier begin status is {frontier_begin_status or 'missing'}")
        if not frontier_started_after_slam_ready:
            if demo_truth_nav:
                frontier_blockers.append("frontier did not start after live nav inputs became ready")
            else:
                frontier_blockers.append("frontier did not start after live SLAM reached TRACKING")
        if lingtu_failed_modules:
            frontier_blockers.append(f"LingTu modules failed: {sorted(lingtu_failed_modules)}")
        if not last_plan_report:
            frontier_blockers.append("navigation last_plan_report missing")
        if planner_fallback_used:
            frontier_blockers.append("navigation planner fallback was used")
        if planner_repair_used:
            frontier_blockers.append("navigation primary planner repair was used")
        if direct_goal_fallback.get("used") is True:
            frontier_blockers.append("navigation direct_goal_fallback was used")
        if str(final_navigation_health.get("plan_safety_policy") or "") != "reject":
            frontier_blockers.append("navigation plan_safety_policy is not reject")
        if len(frontier_goals) < int(frontier_min_goals):
            frontier_blockers.append(
                f"frontier goal count {len(frontier_goals)} < required {int(frontier_min_goals)}"
            )
        if not frontier_success_requirement_met:
            frontier_blockers.append(
                "frontier successful navigation count "
                f"{len(frontier_success_events)} < required {int(frontier_min_goals)}"
            )
        if frontier_failed_events:
            msg = f"frontier navigation terminal failures observed: {len(frontier_failed_events)}"
            if frontier_success_requirement_met:
                frontier_warnings.append(msg)
            else:
                frontier_blockers.append(msg)
        if frontier_blocked_goal_count > 0:
            msg = f"frontier blocked goal count is {frontier_blocked_goal_count}"
            if frontier_success_requirement_met:
                frontier_warnings.append(msg)
            else:
                frontier_blockers.append(msg)
        if not frontier_batches or max(frontier_batches) <= 0:
            frontier_blockers.append("no non-empty frontier batches observed")
        if not exploration_grid_counts:
            frontier_blockers.append("no exploration grid samples observed")
        else:
            last_counts = exploration_grid_counts[-1]
            if int(last_counts.get("unknown") or 0) <= 0:
                frontier_blockers.append("exploration grid has no unknown cells")
            if int(last_counts.get("free") or 0) <= 0:
                frontier_blockers.append("exploration grid has no free cells")
        if nav_map_area["growth_m2"] < float(min_map_area_growth_m2):
            frontier_blockers.append(
                f"{TOPICS.map_cloud} XY area growth {nav_map_area['growth_m2']:.3f}m2 "
                f"< required {float(min_map_area_growth_m2):.3f}m2"
            )
        if exploration_grid_growth_is_acceptance_metric:
            if explored_known_area["growth_m2"] < float(min_explored_area_growth_m2):
                frontier_blockers.append(
                    "exploration known area growth "
                    f"{explored_known_area['growth_m2']:.3f}m2 "
                    f"< required {float(min_explored_area_growth_m2):.3f}m2"
                )
            if exploration_coverage["growth_ratio"] < float(min_exploration_coverage_growth_ratio):
                frontier_blockers.append(
                    "exploration coverage growth "
                    f"{exploration_coverage['growth_ratio']:.6f} "
                    f"< required {float(min_exploration_coverage_growth_ratio):.6f}"
                )
        elif (
            explored_known_area["growth_m2"] < float(min_explored_area_growth_m2)
            or exploration_coverage["growth_ratio"]
            < float(min_exploration_coverage_growth_ratio)
        ):
            frontier_warnings.append(
                "exploration_grid is a rolling local frontier input; "
                f"cumulative mapping growth is judged from {TOPICS.map_cloud}"
            )
        if len(global_path_counts) <= 0 or max(global_path_counts or [0]) < 2:
            frontier_blockers.append("no non-empty global path observed")
        if len(local_path_counts) <= 0 or max(local_path_counts or [0]) < 2:
            frontier_blockers.append("no non-empty local path observed")
        if nav_cmd_nonzero <= 0:
            frontier_blockers.append(f"no non-zero {TOPICS.cmd_vel} observed")
        if (sim_moved_m or 0.0) <= 0.2:
            frontier_blockers.append("simulated robot motion <= 0.2m")
        if (
            motion_consistency["checked"]
            and not motion_consistency["ok"]
        ):
            frontier_blockers.append(
                "Fast-LIO odometry diverged from MuJoCo motion "
                f"(fastlio2={float(moved_m or 0.0):.3f}m, "
                f"sim={float(sim_moved_m or 0.0):.3f}m, "
                f"allowed={float(motion_consistency['max_allowed_fastlio2_moved_m'] or 0.0):.3f}m)"
            )
        if z_consistency["checked"] and not z_consistency["ok"]:
            frontier_blockers.append(
                "Fast-LIO odometry Z drift exceeded limit "
                f"(error={float(z_delta_error or 0.0):.3f}m, "
                f"allowed={float(max_fastlio_z_drift_m):.3f}m)"
            )

    tare_blockers: list[str] = []
    tare_warnings: list[str] = []
    if run_lingtu_tare:
        tare_success_requirement_met = len(tare_success_events) >= int(tare_min_goals)
        if tare_start_status not in {"started", "already_running"}:
            tare_blockers.append(f"TARE start status is {tare_start_status or 'missing'}")
        if not tare_started_after_slam_ready:
            if demo_truth_nav:
                tare_blockers.append("TARE did not start after live nav inputs became ready")
            else:
                tare_blockers.append("TARE did not start after live SLAM reached TRACKING")
        if lingtu_failed_modules:
            tare_blockers.append(f"LingTu modules failed: {sorted(lingtu_failed_modules)}")
        if not last_tare_stats and not final_tare_status:
            tare_blockers.append("TARE status/stats missing")
        if (last_tare_stats.get("alive") if isinstance(last_tare_stats, dict) else None) is False:
            tare_blockers.append("TARE bridge reports alive=false")
        if len(tare_goals) < int(tare_min_goals):
            tare_blockers.append(
                f"TARE goal count {len(tare_goals)} < required {int(tare_min_goals)}"
            )
        if not tare_success_requirement_met:
            tare_blockers.append(
                "TARE successful navigation count "
                f"{len(tare_success_events)} < required {int(tare_min_goals)}"
            )
        if tare_failed_events:
            msg = f"TARE navigation terminal failures observed: {len(tare_failed_events)}"
            if tare_success_requirement_met:
                tare_warnings.append(msg)
            else:
                tare_blockers.append(msg)
        if not last_plan_report:
            tare_blockers.append("navigation last_plan_report missing")
        if planner_fallback_used:
            tare_blockers.append("navigation planner fallback was used")
        if planner_repair_used:
            tare_blockers.append("navigation primary planner repair was used")
        if direct_goal_fallback.get("used") is True:
            tare_blockers.append("navigation direct_goal_fallback was used")
        if str(final_navigation_health.get("plan_safety_policy") or "") != "reject":
            tare_blockers.append("navigation plan_safety_policy is not reject")
        if not exploration_grid_counts:
            tare_blockers.append("no exploration grid samples observed")
        if nav_map_area["growth_m2"] < float(min_map_area_growth_m2):
            tare_blockers.append(
                f"{TOPICS.map_cloud} XY area growth {nav_map_area['growth_m2']:.3f}m2 "
                f"< required {float(min_map_area_growth_m2):.3f}m2"
            )
        if len(global_path_counts) <= 0 or max(global_path_counts or [0]) < 2:
            tare_blockers.append("no non-empty global path observed")
        if len(local_path_counts) <= 0 or max(local_path_counts or [0]) < 2:
            tare_blockers.append("no non-empty local path observed")
        if nav_cmd_nonzero <= 0:
            tare_blockers.append(f"no non-zero {TOPICS.cmd_vel} observed")
        if (sim_moved_m or 0.0) <= 0.2:
            tare_blockers.append("simulated robot motion <= 0.2m")
        if motion_consistency["checked"] and not motion_consistency["ok"]:
            tare_blockers.append(
                "Fast-LIO odometry diverged from MuJoCo motion "
                f"(fastlio2={float(moved_m or 0.0):.3f}m, "
                f"sim={float(sim_moved_m or 0.0):.3f}m, "
                f"allowed={float(motion_consistency['max_allowed_fastlio2_moved_m'] or 0.0):.3f}m)"
            )
        if z_consistency["checked"] and not z_consistency["ok"]:
            tare_blockers.append(
                "Fast-LIO odometry Z drift exceeded limit "
                f"(error={float(z_delta_error or 0.0):.3f}m, "
                f"allowed={float(max_fastlio_z_drift_m):.3f}m)"
            )

    inspection_blockers: list[str] = []
    inspection_warnings: list[str] = []
    if run_lingtu_inspection:
        inspection_success_requirement_met = (
            int(inspection_success_count) >= int(inspection_min_checkpoints)
        )
        if inspection_start_status not in {"patrolling", "started", "already_running"}:
            inspection_blockers.append(
                f"inspection start status is {inspection_start_status or 'missing'}"
            )
        if not inspection_started_after_slam_ready:
            if demo_truth_nav:
                inspection_blockers.append(
                    "inspection did not start after live nav inputs became ready"
                )
            else:
                inspection_blockers.append(
                    "inspection did not start after live SLAM reached TRACKING"
                )
        if lingtu_failed_modules:
            inspection_blockers.append(f"LingTu modules failed: {sorted(lingtu_failed_modules)}")
        if len(inspection_goals_sent) < int(inspection_min_checkpoints):
            inspection_blockers.append(
                "inspection checkpoint count "
                f"{len(inspection_goals_sent)} < required {int(inspection_min_checkpoints)}"
            )
        if not inspection_success_requirement_met:
            inspection_blockers.append(
                "inspection successful checkpoint count "
                f"{int(inspection_success_count)} < required {int(inspection_min_checkpoints)}"
            )
        if inspection_failed_events:
            msg = f"inspection terminal failures observed: {len(inspection_failed_events)}"
            if inspection_success_requirement_met:
                inspection_warnings.append(msg)
            else:
                inspection_blockers.append(msg)
        if not last_plan_report:
            inspection_blockers.append("navigation last_plan_report missing")
        if planner_fallback_used:
            inspection_blockers.append("navigation planner fallback was used")
        if planner_repair_used:
            inspection_blockers.append("navigation primary planner repair was used")
        if direct_goal_fallback.get("used") is True:
            inspection_blockers.append("navigation direct_goal_fallback was used")
        if str(final_navigation_health.get("plan_safety_policy") or "") != "reject":
            inspection_blockers.append("navigation plan_safety_policy is not reject")
        if nav_map_area["growth_m2"] < float(min_map_area_growth_m2):
            inspection_blockers.append(
                f"{TOPICS.map_cloud} XY area growth {nav_map_area['growth_m2']:.3f}m2 "
                f"< required {float(min_map_area_growth_m2):.3f}m2"
            )
        if len(global_path_counts) <= 0 or max(global_path_counts or [0]) < 2:
            inspection_blockers.append("no non-empty global path observed")
        if len(local_path_counts) <= 0 or max(local_path_counts or [0]) < 2:
            inspection_blockers.append("no non-empty local path observed")
        if nav_cmd_nonzero <= 0:
            inspection_blockers.append(f"no non-zero {TOPICS.cmd_vel} observed")
        if (sim_moved_m or 0.0) <= 0.2:
            inspection_blockers.append("simulated robot motion <= 0.2m")
        if motion_consistency["checked"] and not motion_consistency["ok"]:
            inspection_blockers.append(
                "Fast-LIO odometry diverged from MuJoCo motion "
                f"(fastlio2={float(moved_m or 0.0):.3f}m, "
                f"sim={float(sim_moved_m or 0.0):.3f}m, "
                f"allowed={float(motion_consistency['max_allowed_fastlio2_moved_m'] or 0.0):.3f}m)"
            )
        if z_consistency["checked"] and not z_consistency["ok"]:
            inspection_blockers.append(
                "Fast-LIO odometry Z drift exceeded limit "
                f"(error={float(z_delta_error or 0.0):.3f}m, "
                f"allowed={float(max_fastlio_z_drift_m):.3f}m)"
            )

    mapping_input_path = (
        f"{TOPICS.raw_lidar_points} + {TOPICS.raw_imu} -> fastlio2 -> "
        f"{FASTLIO_ODOMETRY_TOPIC} + {FASTLIO_MAP_CLOUD_TOPIC} -> "
        f"{TOPICS.odometry} + {TOPICS.map_cloud}"
        if not demo_truth_nav
        else (
            f"{TOPICS.raw_lidar_points} + {TOPICS.raw_imu} still feed "
            "fastlio2 diagnostics; "
            f"{TOPICS.odometry} + {TOPICS.registered_cloud} + {TOPICS.map_cloud} are "
            "published from MuJoCo ground truth for stable visible demo"
        )
    )
    map_artifact_points = (
        np.asarray(list(map_artifact_points_by_cell.values()), dtype=np.float32)
        if map_artifact_points_by_cell
        else np.empty((0, 3), dtype=np.float32)
    )
    map_artifacts: dict[str, Any] = {
        "ok": False,
        "enabled": bool(save_map_artifacts),
        "blockers": [],
        "assets": {},
        "source_contract": {
            "same_source_pcd": False,
            "same_source_tomogram": False,
        },
    }
    if save_map_artifacts:
        artifact_root = work_dir.parent if work_dir.name == "work" else work_dir
        map_artifacts = _write_same_source_map_artifacts(
            artifact_dir=artifact_root / "same_source_map",
            points=map_artifact_points,
            frame_id=SIM_FASTLIO_LIVE_MAP_FRAME_ID,
            world=world,
            source_topics=(TOPICS.map_cloud,),
            mapping_input_path=mapping_input_path,
            build_tomogram=bool(build_tomogram),
            tomogram_resolution=float(tomogram_resolution),
            tomogram_slice_dh=float(tomogram_slice_dh),
            tomogram_ground_h=float(tomogram_ground_h),
            map_artifact_max_span_m=float(map_artifact_max_span_m),
            tomogram_max_cells=int(tomogram_max_cells),
            source="mujoco_fastlio2_live_gate",
            extra_metadata={
                "scan_time_profile": scan_time_profile,
                "nav_data_source": nav_data_source,
                "fastlio_lidar_input": fastlio_lidar_input,
            },
        )
        map_artifacts["enabled"] = True
        map_artifacts["accumulator"] = {
            "voxel_size": float(map_artifact_voxel_size),
            "max_points": int(map_artifact_max_points),
            "point_count": int(len(map_artifact_points)),
            "samples_with_new_points": int(map_artifact_samples),
            "frames": map_artifact_frames,
        }
        if not map_artifacts.get("ok"):
            for blocker in map_artifacts.get("blockers") or ["same-source map artifact save failed"]:
                base_blockers.append(f"map artifact: {blocker}")

    frontier_verified = bool(run_lingtu_frontier and not frontier_blockers)
    tare_verified = bool(run_lingtu_tare and not tare_blockers)
    inspection_verified = bool(run_lingtu_inspection and not inspection_blockers)
    base_ok = not base_blockers
    ok = bool(
        base_ok
        and (not run_lingtu_frontier or frontier_verified)
        and (not run_lingtu_tare or tare_verified)
        and (not run_lingtu_inspection or inspection_verified)
        and bool(moving_obstacle_ok)
    )
    remaining_gaps = (
        list(runtime_faults)
        + list(base_blockers)
        + list(frontier_blockers)
        + list(tare_blockers)
        + list(inspection_blockers)
    )
    deliverable_contract = {
        "target": (
            "MuJoCo MID-360/IMU -> Fast-LIO2 -> LingTu canonical runtime topics -> "
            "map artifacts/tomogram -> visible RViz/MuJoCo evidence"
        ),
        "simulation_only": True,
        "hardware_output_forbidden": True,
        "checks": {
            "raw_mujoco_lidar": counts["cloud_published"] > 0 and bool(point_counts),
            "raw_mujoco_imu": counts["imu_published"] > 0,
            "fastlio2_odometry_and_map": algorithm_verified,
            "canonical_nav_topics": canonical_nav_outputs_verified,
            "live_ros_topic_visibility": not missing_visible_ros_topics
            and not visible_ros_topic_error,
            "fastlio2_xy_motion_consistency": bool(motion_consistency.get("ok")),
            "fastlio2_z_consistency": bool(z_consistency.get("ok")),
            "fastlio2_yaw_consistency": bool(yaw_consistency.get("ok")),
            "same_source_map_artifact": bool(map_artifacts.get("ok")),
            "nav_cmd_vel_nonzero": (
                nav_cmd_nonzero > 0
                if (run_lingtu_frontier or run_lingtu_tare or run_lingtu_inspection)
                else None
            ),
            "frontier_or_exploration": (
                frontier_verified
                if run_lingtu_frontier
                else tare_verified if run_lingtu_tare else None
            ),
            "tare_native_exploration": tare_verified if run_lingtu_tare else None,
            "inspection_patrol": inspection_verified if run_lingtu_inspection else None,
            "moving_obstacle_evidence": (
                bool(moving_obstacle_ok) if moving_obstacle_enabled else None
            ),
            "visible_mujoco_window": (
                live_window_frames > 0 if show_mujoco_window else None
            ),
        },
        "stop_condition": "ok=true and remaining_gaps=[]",
        "remaining_gaps": remaining_gaps,
    }
    validated_src_surfaces = [
        "src/core/runtime_interface.py",
        "src/drivers/sim/mujoco_lingtu_stack.py",
        "src/drivers/sim/mujoco_live_runtime.py",
        "src/drivers/sim/mujoco_sensor_bridge.py",
        "src/nav/services/same_source_map_artifacts.py",
        "src/slam/fastlio2_live_bridge.py",
        "src/slam/fastlio2_nav_bridge.py",
        "src/slam/slam_bridge_module.py",
    ]
    if run_lingtu_frontier:
        validated_src_surfaces.extend(
            [
                "src/core/blueprints/full_stack.py",
                "src/nav/navigation_module.py",
                "src/nav/occupancy_grid_module.py",
            ]
        )
    if run_lingtu_tare:
        validated_src_surfaces.extend(
            [
                "src/core/blueprints/full_stack.py",
                "src/core/native_module.py",
                "src/exploration/native_factories.py",
                "src/exploration/tare_explorer_module.py",
                "src/nav/navigation_module.py",
                "src/nav/occupancy_grid_module.py",
            ]
        )
    not_validated_by_this_gate = [
        "all src modules",
        "PCT saved-map planning and closed-loop following",
        "saved-map relocalization/localizer closure",
        "semantic/perception/gateway field stack",
        "real S100P hardware and safety actuation",
    ]
    if not run_lingtu_tare:
        not_validated_by_this_gate.insert(1, "TARE runtime exploration closure")
    module_first_boundary = {
        "gate_role": "validation_harness",
        "product_runtime": False,
        "validated_src_surfaces": validated_src_surfaces,
        "not_validated_by_this_gate": not_validated_by_this_gate,
        "module_first_gap": (
            "This script may orchestrate external MuJoCo/ROS/Fast-LIO processes "
            "for validation, but reusable runtime behavior must live in src "
            "Modules, NativeModules, or bridge services."
        ),
        "refactor_status": {
            "same_source_map_artifacts": "moved_to_src_nav_service",
            "mujoco_lingtu_frontier_stack": "moved_to_src_driver_runtime",
            "mujoco_live_runtime": "moved_to_src_driver_runtime",
            "mujoco_sensor_messages": "moved_to_src_driver_bridge",
            "fastlio2_process_lifecycle": "moved_to_src_slam_bridge_service",
            "fastlio2_nav_topic_normalization": "moved_to_src_slam_bridge_service",
            "remaining_gate_work": "thin the launch/evidence collection script further and move endpoint entrypoints behind lingtu.py profiles",
        },
    }
    fastlio2_log_diagnostics = _fastlio2_log_diagnostics(log_path)
    fastlio2_degeneracy_detail = _summarize_degeneracy_detail_samples(
        degeneracy_detail_samples
    )
    fastlio2_degeneracy_detail["subscription_error"] = (
        degeneracy_detail_subscription_error
    )
    report = {
        "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
        "ok": ok,
        "algorithm": "fastlio2",
        "deliverable_contract": deliverable_contract,
        "module_first_boundary": module_first_boundary,
        "remaining_gaps": remaining_gaps,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "runtime_contract": runtime_contract,
        "runtime_evidence": _runtime_evidence_report(runtime_evidence),
        "hardware_safety": hardware_safety,
        "simulation_motion": (
            (nav_cmd_nonzero > 0)
            if drive_source == "nav_cmd_vel"
            else (abs(drive_vx) > 0.0 or abs(drive_vy) > 0.0 or abs(drive_wz) > 0.0)
        ),
        "live_mujoco_lidar_verified": counts["cloud_published"] > 0 and bool(point_counts),
        "live_mujoco_imu_verified": counts["imu_published"] > 0,
        "slam_algorithm_output_verified": algorithm_verified,
        "canonical_nav_outputs_verified": canonical_nav_outputs_verified,
        "bridge_verified": bridge_verified,
        "base_blockers": base_blockers,
        "runtime_faults": runtime_faults,
        "runtime_fault_events": runtime_fault_events,
        "runtime_fault_confirm_samples": int(runtime_fault_confirm_samples),
        "gate_wall_timeout": wall_timeout,
        "nav_data_source": nav_data_source,
        "duration_clock": duration_clock,
        "imu_acc_mode": imu_acc_mode,
        "scan_time_profile": scan_time_profile,
        "sim_zupt_disabled": disable_sim_zupt,
        "demo_visualization_mode": demo_truth_nav,
        "mujoco_truth_odom_body_tf_published": bool(demo_truth_nav),
        "truth_nav_frame_alignment": {
            "enabled": bool(demo_truth_nav),
            "map_frame_origin_world_xy": (
                list(truth_nav_origin_world_xy)
                if truth_nav_origin_world_xy is not None
                else []
            ),
            "source": truth_nav_origin_source,
            "reason": (
                "MuJoCo ground-truth odometry must be expressed in the same "
                "map/odom frame as the tomogram and inspection goals."
            ),
        },
        "mujoco_live_window": {
            "enabled": bool(show_mujoco_window or live_window_frames > 0),
            "frames": int(live_window_frames),
            "error": live_window_error,
        },
        "true_mapping_input_path": mapping_input_path,
        "map_artifacts": map_artifacts,
        "assets": {
            "map_pcd": (map_artifacts.get("assets") or {}).get("map_pcd", {}).get("path", ""),
            "metadata": (map_artifacts.get("assets") or {}).get("metadata", {}).get("path", ""),
            "tomogram": (map_artifacts.get("assets") or {}).get("tomogram", {}).get("path", ""),
        },
        "fastlio2_motion_consistency": motion_consistency,
        "fastlio2_z_consistency": z_consistency,
        "fastlio2_yaw_consistency": yaw_consistency,
        "fastlio_large_loop_diagnostic_report": fastlio_large_loop_diagnostic_report,
        "fastlio2_log_diagnostics": fastlio2_log_diagnostics,
        "fastlio2_degeneracy_detail": fastlio2_degeneracy_detail,
        "frames": {
            "mujoco_world": SIM_WORLD_FRAME_ID,
            "published_lidar": SIM_LIDAR_FRAME_ID,
            "published_imu": SIM_BODY_FRAME_ID,
            "fastlio2_body_frame": SIM_BODY_FRAME_ID,
            "fastlio2_lidar_frame": SIM_LIDAR_FRAME_ID,
            "fastlio2_world_frame": SIM_FASTLIO_LIVE_MAP_FRAME_ID,
            "fastlio2_odometry": SIM_NAV_ODOMETRY_FRAME_ID,
            "fastlio2_cloud_map": SIM_FASTLIO_LIVE_MAP_FRAME_ID,
            "nav_odometry": SIM_NAV_ODOMETRY_FRAME_ID,
            "nav_registered_cloud": SIM_NAV_REGISTERED_CLOUD_FRAME_ID,
            "nav_map_cloud": SIM_FASTLIO_LIVE_MAP_FRAME_ID,
            "slam_bridge_output": SIM_NAV_ODOMETRY_FRAME_ID,
            "cmd_vel": (
                TOPICS.cmd_vel
                if (run_lingtu_frontier or run_lingtu_tare)
                else "not_published"
            ),
        },
        "world": str(world),
        "start_position": start or _scene_start(world),
        "mujoco_memory": mujoco_memory,
        "drive_mode": drive_mode,
        "sensor_timestamp_source": "mujoco_sim_time",
        "lidar_source": {
            "kind": (
                "MuJoCo mj_multiRay with official Livox MID-360 scan pattern"
                if resolved_pattern is not None
                else "MuJoCo golden-spiral fallback scan"
            ),
            "fastlio_lidar_input": fastlio_lidar_input,
            "pointcloud2_profile": (
                "not used; publishing Livox CustomMsg with offset_time/line/tag"
                if fastlio_lidar_input == "livox_custom_msg"
                else (
                    "xyz/intensity/time/ring with zero per-point times"
                    if scan_time_profile == "instantaneous"
                    else (
                        "xyz/intensity/time/ring with actual rolling subscan times"
                        if scan_time_profile == "physical_rolling"
                        else "xyz/intensity/time/ring with synthetic rolling per-point times"
                    )
                )
            ),
            "fastlio2_lidar_type": 1 if fastlio_lidar_input == "livox_custom_msg" else 2,
            "fastlio2_timestamp_unit": (
                "NS via Livox CustomMsg offset_time"
                if fastlio_lidar_input == "livox_custom_msg"
                else "SEC"
            ),
            "scan_time_profile": scan_time_profile,
            "scan_time_model_contract": (
                "snapshot_raycast_zero_offset_time"
                if scan_time_profile == "instantaneous"
                else (
                    "physical_subscans_with_actual_sim_time_offsets"
                    if scan_time_profile == "physical_rolling"
                    else "synthetic_mid360_scan_offsets_on_snapshot_rays"
                )
            ),
            "forced_pattern": resolved_pattern is not None,
            "pattern_path": str(resolved_pattern) if resolved_pattern is not None else "",
            "pattern_sha256": _sha256_file(resolved_pattern) if resolved_pattern is not None else "",
            "samples_per_frame": int(mid360_samples_per_frame),
            "physical_rolling_subscan_count": int(physical_rolling_subscan_count),
            "physical_rolling_effective_samples_per_subscan": int(
                physical_rolling_effective_samples_per_subscan
            ),
            "physical_rolling_observed_subscan_count_min": (
                min(rolling_lidar_subscan_counts) if rolling_lidar_subscan_counts else 0
            ),
            "physical_rolling_observed_subscan_count_max": (
                max(rolling_lidar_subscan_counts) if rolling_lidar_subscan_counts else 0
            ),
            "fallback_n_rays": int(n_rays),
            "body": "lidar_link",
            "body_to_lidar_m": list(mujoco_lidar_extrinsic.translation),
            "body_to_lidar_rotation_xyzw": list(mujoco_lidar_extrinsic.rotation_xyzw),
        },
        "fastlio2_sim_config": {
            "imu_static_acc_thresh": 0.0 if disable_sim_zupt else 0.04,
            "imu_static_gyro_thresh": 0.0 if disable_sim_zupt else 0.001,
            "zupt_min_static_frames": 1_000_000 if disable_sim_zupt else 5,
            "lidar_filter_num": int(fastlio_lidar_filter_num),
            "scan_resolution": float(fastlio_scan_resolution),
            "map_resolution": float(fastlio_map_resolution),
            "near_search_num": int(fastlio_near_search_num),
            "ieskf_max_iter": int(fastlio_ieskf_max_iter),
            "degeneracy_max_update_dof": 2,
            "degeneracy_max_condition": 50000.0,
            "max_update_translation_m": 0.5,
            "max_update_rotation_rad": 0.35,
            "reject_nonconverged_update": True,
            "reject_degenerate_nonconverged_update": True,
            "lidar_cov_inv": float(fastlio_lidar_cov_inv),
            "time_diff_lidar_to_imu": float(fastlio_time_diff_lidar_to_imu),
            "vertical_velocity_constraint": vertical_velocity_constraint_mode,
            "vertical_velocity_constraint_enabled": bool(
                vertical_velocity_constraint_enabled
            ),
            "vertical_velocity_sigma_v": 0.05,
            "config_path": str(config_path),
        },
        "trajectory_video_alignment": {
            "display_only": True,
            "reason": "MuJoCo world starts at the scene pose while Fast-LIO odom initializes at its own local origin.",
            "method": "fit a yaw-only SE(2) transform from MuJoCo world XY to Fast-LIO odom XY for video overlay; keep Z visible with an initial-height offset.",
        },
        "commanded_sim_velocity": {
            "linear_x": drive_vx,
            "linear_y": drive_vy,
            "angular_z": drive_wz,
            "source": drive_source,
            "nav_max_linear_speed": float(nav_max_linear_speed),
            "nav_max_angular_z": float(nav_max_angular_z),
            "nav_turn_speed_yaw_rate_start": float(nav_turn_speed_yaw_rate_start),
            "nav_turn_speed_min_scale": float(nav_turn_speed_min_scale),
            "cmd_vel_linear_limit": float(cmd_vel_linear_limit),
            "cmd_vel_angular_limit": float(cmd_vel_angular_limit),
            "cmd_vel_linear_accel_limit": float(cmd_vel_linear_accel_limit),
            "cmd_vel_angular_accel_limit": float(cmd_vel_angular_accel_limit),
            "nav_cmd_vel_sim_linear_scale": float(cmd_vel_sim_linear_scale),
            "nav_cmd_vel_sim_angular_scale": float(cmd_vel_sim_angular_scale),
        },
        "moving_obstacles": {
            "enabled": bool(moving_obstacle_enabled),
            "mode": moving_obstacle_mode,
            "count": int(moving_obstacle_count),
            "start_s": float(moving_obstacle_start_s),
            "duration_s": float(moving_obstacle_duration_s),
            "period_s": float(moving_obstacle_period_s),
            "forward_m": float(moving_obstacle_forward_m),
            "forward_step_m": float(moving_obstacle_forward_step_m),
            "lateral_phase_step_rad": float(moving_obstacle_lateral_phase_step_rad),
            "lateral_amplitude_m": float(moving_obstacle_lateral_amplitude_m),
            "along_amplitude_m": float(moving_obstacle_along_amplitude_m),
            "radius_m": float(moving_obstacle_radius_m),
            "height_m": float(moving_obstacle_height_m),
            "point_spacing_m": float(moving_obstacle_point_spacing),
            "intensity": float(moving_obstacle_intensity),
            "speed_bounds": moving_obstacle_speed,
            "published_update_count": int(moving_obstacle_published_update_count),
            "published_point_count_max": int(moving_obstacle_published_point_count_max),
            "samples": moving_obstacle_samples,
            "trail_clearance": moving_obstacle_clearance,
            "ok": bool(moving_obstacle_ok),
        },
        "control_quality": control_quality,
        "applied_sim_command": applied_command_summary,
        "counts": counts,
        "point_count": {
            "min": min(point_counts) if point_counts else 0,
            "max": max(point_counts) if point_counts else 0,
            "mean": round(float(np.mean(point_counts)), 1) if point_counts else 0.0,
        },
        "subscription_counts_at_start": subscription_counts,
        "outputs": {
            "fastlio2_odometry": len(odom_out),
            "fastlio2_cloud_registered": len(registered_cloud_out),
            "fastlio2_cloud_map": len(map_cloud_out),
            "nav_odometry": len(nav_odom_out),
            "nav_registered_cloud": len(nav_registered_cloud_out),
            "nav_map_cloud": len(nav_map_cloud_out),
            "bridge_localization_status": len(bridge_statuses),
            "nav_cmd_vel": len(nav_cmd_vel_samples),
            "nav_cmd_vel_nonzero": nav_cmd_nonzero,
        },
        "ros_topics": {
            "expected": expected_ros_topics,
            "visible": visible_ros_topics,
            "missing": missing_visible_ros_topics,
            "error": visible_ros_topic_error,
        },
        "map_growth": {
            "nav_map_cloud_xy_area": nav_map_area,
            "exploration_known_area": explored_known_area,
            "exploration_free_area": explored_free_area,
            "exploration_coverage": exploration_coverage,
            "accepted_cumulative_growth_source": TOPICS.map_cloud,
            "exploration_grid_growth_is_acceptance_metric": bool(
                exploration_grid_growth_is_acceptance_metric
            ),
            "exploration_grid_accumulation": exploration_grid_accumulation,
            "min_map_area_growth_m2": float(min_map_area_growth_m2),
            "min_explored_area_growth_m2": float(min_explored_area_growth_m2),
            "min_exploration_coverage_growth_ratio": float(
                min_exploration_coverage_growth_ratio
            ),
            "nav_map_cloud_area_samples": len(nav_map_cloud_area_samples),
            "exploration_area_samples": len(exploration_area_samples),
        },
        "motion_consistency": motion_consistency,
        "path_diagnostics": {
            "global_path": {
                "sample_count": len(global_path_summaries),
                "latest": global_path_summaries[-1] if global_path_summaries else {},
                "samples_tail": global_path_summaries[-12:],
            },
            "local_path": {
                "sample_count": len(local_path_summaries),
                "latest": local_path_summaries[-1] if local_path_summaries else {},
                "samples_tail": local_path_summaries[-12:],
            },
        },
        "lingtu_frontier": {
            "enabled": bool(run_lingtu_frontier),
            "verified": frontier_verified,
            "blockers": frontier_blockers,
            "warnings": frontier_warnings,
            "begin_status": frontier_begin_status,
            "started_after_slam_ready": frontier_started_after_slam_ready,
            "failed_modules": lingtu_failed_modules,
            "goal_count": len(frontier_goals),
            "successful_navigation_goal_count": len(frontier_success_events),
            "failed_navigation_goal_count": len(frontier_failed_events),
            "mission_terminal_events": frontier_mission_events[-20:],
            "frontier_health": final_frontier_health,
            "goals": frontier_goals[-10:],
            "frontier_batch_count": len(frontier_batches),
            "frontier_count_max": max(frontier_batches) if frontier_batches else 0,
            "exploration_grid_samples": len(exploration_grid_counts),
            "exploration_grid_first": exploration_grid_counts[0] if exploration_grid_counts else {},
            "exploration_grid_last": exploration_grid_counts[-1] if exploration_grid_counts else {},
            "exploration_grid_metadata_last": last_exploration_grid_meta,
            "global_path_count": len(global_path_counts),
            "global_path_points_max": max(global_path_counts) if global_path_counts else 0,
            "local_path_count": len(local_path_counts),
            "local_path_points_max": max(local_path_counts) if local_path_counts else 0,
            "waypoint_count": waypoint_count,
            "min_required_goals": int(frontier_min_goals),
            "planning_frame": SIM_NAV_ODOMETRY_FRAME_ID,
            "occupancy_frame": SIM_FASTLIO_LIVE_MAP_FRAME_ID,
        },
        "lingtu_tare": {
            "enabled": bool(run_lingtu_tare),
            "verified": tare_verified,
            "blockers": tare_blockers,
            "warnings": tare_warnings,
            "start_status": tare_start_status,
            "started_after_slam_ready": tare_started_after_slam_ready,
            "failed_modules": lingtu_failed_modules,
            "goal_count": len(tare_goals),
            "successful_navigation_goal_count": len(tare_success_events),
            "failed_navigation_goal_count": len(tare_failed_events),
            "mission_terminal_events": tare_mission_events[-20:],
            "tare_status": final_tare_status,
            "tare_health": final_tare_health,
            "tare_stats_last": last_tare_stats,
            "goals": tare_goals[-10:],
            "path_count": len(tare_path_counts),
            "path_points_max": max(tare_path_counts) if tare_path_counts else 0,
            "exploration_grid_samples": len(exploration_grid_counts),
            "exploration_grid_first": exploration_grid_counts[0] if exploration_grid_counts else {},
            "exploration_grid_last": exploration_grid_counts[-1] if exploration_grid_counts else {},
            "exploration_grid_metadata_last": last_exploration_grid_meta,
            "global_path_count": len(global_path_counts),
            "global_path_points_max": max(global_path_counts) if global_path_counts else 0,
            "local_path_count": len(local_path_counts),
            "local_path_points_max": max(local_path_counts) if local_path_counts else 0,
            "waypoint_count": waypoint_count,
            "min_required_goals": int(tare_min_goals),
            "scenario": str(tare_scenario or "indoor"),
            "planning_frame": SIM_NAV_ODOMETRY_FRAME_ID,
            "occupancy_frame": SIM_FASTLIO_LIVE_MAP_FRAME_ID,
        },
        "lingtu_inspection": {
            "enabled": bool(run_lingtu_inspection),
            "verified": inspection_verified,
            "blockers": inspection_blockers,
            "warnings": inspection_warnings,
            "start_status": inspection_start_status,
            "started_after_slam_ready": inspection_started_after_slam_ready,
            "failed_modules": lingtu_failed_modules,
            "goal_count": len(inspection_goals_sent),
            "successful_navigation_goal_count": int(inspection_success_count),
            "failed_navigation_goal_count": len(inspection_failed_events),
            "mission_terminal_events": inspection_mission_events[-20:],
            "goals": inspection_goals_sent[-20:],
            "global_path_count": len(global_path_counts),
            "global_path_points_max": max(global_path_counts) if global_path_counts else 0,
            "local_path_count": len(local_path_counts),
            "local_path_points_max": max(local_path_counts) if local_path_counts else 0,
            "waypoint_count": waypoint_count,
            "min_required_checkpoints": int(inspection_min_checkpoints),
            "patrol_state": inspection_state,
            "patrol_index": int(inspection_patrol_index),
            "patrol_total": int(inspection_patrol_total),
            "global_planner": str(inspection_planner or "astar"),
            "tomogram": str(inspection_tomogram or ""),
            "replan_on_costmap_update": final_navigation_health.get(
                "replan_on_costmap_update"
            ),
            "planning_frame": SIM_NAV_ODOMETRY_FRAME_ID,
            "occupancy_frame": SIM_FASTLIO_LIVE_MAP_FRAME_ID,
        },
        "navigation_chain": {
            "health": final_navigation_health,
            "status": final_navigation_status,
            "last_plan_report": last_plan_report,
            "direct_goal_fallback": direct_goal_fallback,
            "planner_fallback_used": planner_fallback_used,
            "planner_repair_used": planner_repair_used,
        },
        "navigation_diagnostics": navigation_diagnostics,
        "autonomy_chain": {
            "local_planner": final_local_planner_health,
            "path_follower": final_path_follower_health,
            "cmd_vel_mux": final_cmd_vel_mux_health,
        },
        "states_seen": states,
        "final_bridge_status": source_end_bridge_status or shutdown_bridge_status,
        "shutdown_bridge_status": shutdown_bridge_status,
        "first_odom_xyz": first_odom_xyz,
        "last_odom_xyz": last_odom_xyz,
        "first_odom_yaw_rad": first_odom_yaw,
        "last_odom_yaw_rad": last_odom_yaw,
        "fastlio2_moved_m": moved_m,
        "fastlio2_path_length_m": round(float(odom_path_length_m), 4),
        "first_sim_xyz": first_sim_xyz,
        "last_sim_xyz": last_sim_xyz,
        "first_sim_yaw_rad": first_sim_yaw,
        "last_sim_yaw_rad": last_sim_yaw,
        "sim_moved_m": sim_moved_m,
        "sim_path_length_m": round(float(sim_path_length_m), 4),
        "sim_time_s": round(sim_time_s, 3),
        "sim_wall_time_s": round(sim_wall_time_s, 3),
        "sim_realtime_factor": round(sim_realtime_factor, 4),
        "sim_control_dt_s": float(getattr(engine, "control_dt", 0.0)),
        "wall_time_s": round(time.time() - started, 3),
        "process_returncode": process_returncode,
        "config_path": str(config_path),
        "log_path": str(log_path),
        "log_tail": _tail(log_path),
    }
    if video_out_path is not None:
        if video_render_state.get("error"):
            report["video_mujoco_render_error"] = str(video_render_state["error"])
        _write_stage_video(video_samples, report, video_out_path, fps=video_fps)
        report["video_path"] = str(video_out_path)
        report["video_sample_count"] = len(video_samples)
    return report


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="building_scene")
    parser.add_argument("--start", default="", help="Optional start pose x,y,z; defaults to scene marker")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument(
        "--max-wall-time-s",
        type=float,
        default=0.0,
        help=(
            "Abort the live gate after this many wall-clock seconds while still "
            "writing a red diagnostic report. 0 disables the guard."
        ),
    )
    parser.add_argument(
        "--duration-clock",
        choices=["wall", "sim"],
        default="wall",
        help=(
            "Interpret --duration as wall-clock seconds or MuJoCo simulation "
            "seconds. Use sim for exploration validation when the server runs "
            "below realtime."
        ),
    )
    parser.add_argument("--drive-vx", type=float, default=0.25)
    parser.add_argument("--drive-vy", type=float, default=0.0)
    parser.add_argument("--drive-wz", type=float, default=0.06)
    parser.add_argument("--drive-mode", choices=["kinematic", "policy"], default="kinematic")
    parser.add_argument(
        "--drive-source",
        choices=["fixed", "nav_cmd_vel"],
        default="fixed",
        help=f"Use a fixed simulation command or feed MuJoCo from LingTu {TOPICS.cmd_vel}.",
    )
    parser.add_argument("--cmd-vel-timeout", type=float, default=0.75)
    parser.add_argument("--cmd-vel-linear-limit", type=float, default=0.25)
    parser.add_argument("--cmd-vel-angular-limit", type=float, default=0.45)
    parser.add_argument("--cmd-vel-linear-accel-limit", type=float, default=0.5)
    parser.add_argument("--cmd-vel-angular-accel-limit", type=float, default=1.0)
    parser.add_argument(
        "--cmd-vel-sim-linear-scale",
        type=float,
        default=1.0,
        help=(
            f"Scale LingTu {TOPICS.cmd_vel} linear velocity only when applying it "
            f"to the kinematic MuJoCo demo model. The published {TOPICS.cmd_vel} "
            "message remains unmodified."
        ),
    )
    parser.add_argument(
        "--cmd-vel-sim-angular-scale",
        type=float,
        default=1.0,
        help=(
            f"Scale LingTu {TOPICS.cmd_vel} angular velocity only when applying it "
            "to the kinematic MuJoCo demo model."
        ),
    )
    parser.add_argument(
        "--nav-max-linear-speed",
        type=float,
        default=0.25,
        help="PathFollower max linear speed for LingTu-driven live exploration.",
    )
    parser.add_argument(
        "--nav-max-angular-z",
        type=float,
        default=0.45,
        help="PathFollower max yaw rate for LingTu-driven live exploration.",
    )
    parser.add_argument(
        "--nav-turn-speed-yaw-rate-start",
        type=float,
        default=0.0,
        help="Yaw-rate threshold in rad/s where PathFollower starts reducing linear speed; <=0 disables it.",
    )
    parser.add_argument(
        "--nav-turn-speed-min-scale",
        type=float,
        default=1.0,
        help="Minimum PathFollower linear speed scale at max yaw rate.",
    )
    parser.add_argument("--run-lingtu-frontier", action="store_true")
    parser.add_argument("--run-lingtu-tare", action="store_true")
    parser.add_argument("--run-lingtu-inspection", action="store_true")
    parser.add_argument("--tare-min-goals", type=int, default=2)
    parser.add_argument("--tare-start-delay", type=float, default=0.0)
    parser.add_argument("--tare-goal-timeout", type=float, default=180.0)
    parser.add_argument("--tare-scenario", default="indoor")
    parser.add_argument("--frontier-min-goals", type=int, default=3)
    parser.add_argument("--frontier-start-delay", type=float, default=0.0)
    parser.add_argument("--frontier-goal-timeout", type=float, default=30.0)
    parser.add_argument(
        "--inspection-goals",
        default="0.8,0.0;1.6,0.2;2.4,0.4",
        help="Inspection patrol checkpoints as 'x,y[,z];...' or JSON list.",
    )
    parser.add_argument("--inspection-min-checkpoints", type=int, default=3)
    parser.add_argument("--inspection-start-delay", type=float, default=0.0)
    parser.add_argument("--inspection-goal-timeout", type=float, default=90.0)
    parser.add_argument(
        "--inspection-planner",
        choices=["astar", "pct"],
        default="astar",
        help=(
            "Global planner for LingTu inspection mode. Use pct with "
            "--inspection-tomogram to validate static tomogram global routing "
            "while Fast-LIO feeds live localization/local maps."
        ),
    )
    parser.add_argument(
        "--inspection-tomogram",
        type=Path,
        default=None,
        help="Tomogram path used when --inspection-planner=pct.",
    )
    parser.add_argument("--min-map-area-growth-m2", type=float, default=0.25)
    parser.add_argument("--min-explored-area-growth-m2", type=float, default=0.25)
    parser.add_argument("--min-exploration-coverage-growth-ratio", type=float, default=0.001)
    parser.add_argument(
        "--max-fastlio-z-drift-m",
        type=float,
        default=1.0,
        help="Maximum allowed Fast-LIO Z drift relative to MuJoCo motion in one gate.",
    )
    parser.add_argument(
        "--max-fastlio-yaw-drift-rad",
        type=float,
        default=0.5,
        help="Maximum allowed Fast-LIO yaw delta drift relative to MuJoCo motion.",
    )
    parser.add_argument(
        "--runtime-fault-confirm-samples",
        type=int,
        default=2,
        help="Consecutive runtime guard samples required before aborting a live run.",
    )
    parser.add_argument("--n-rays", type=int, default=6400)
    parser.add_argument("--mid360-pattern", type=Path, default=DEFAULT_MID360_PATTERN)
    parser.add_argument("--mid360-samples-per-frame", type=int, default=DEFAULT_MID360_SAMPLES_PER_FRAME)
    parser.add_argument(
        "--fastlio-lidar-input",
        choices=["livox_custom_msg", "timed_pointcloud2"],
        default="livox_custom_msg",
        help="Raw LiDAR message shape sent to Fast-LIO2.",
    )
    parser.add_argument("--fastlio-lidar-filter-num", type=int, default=4)
    parser.add_argument("--fastlio-scan-resolution", type=float, default=0.15)
    parser.add_argument("--fastlio-map-resolution", type=float, default=0.3)
    parser.add_argument("--fastlio-near-search-num", type=int, default=5)
    parser.add_argument("--fastlio-ieskf-max-iter", type=int, default=5)
    parser.add_argument("--fastlio-lidar-cov-inv", type=float, default=1000.0)
    parser.add_argument(
        "--fastlio-time-diff-lidar-to-imu",
        type=float,
        default=0.0,
        help=(
            "Fast-LIO time_diff_lidar_to_imu control in seconds. It is applied "
            "inside Fast-LIO to align IMU timestamps against LiDAR timestamps."
        ),
    )
    parser.add_argument(
        "--fastlio-vertical-velocity-constraint",
        choices=["auto", "on", "off"],
        default="off",
        help=(
            "Optional world-frame v_z=0 pseudo-observation for planar kinematic "
            "MuJoCo ablations. It is off by default because it can hide or move "
            "SLAM drift; real robot Fast-LIO configs also keep this disabled."
        ),
    )
    parser.add_argument(
        "--scan-time-profile",
        choices=["instantaneous", "synthetic_rolling", "physical_rolling"],
        default="physical_rolling",
        help=(
            "Per-point time model for the simulated scan. physical_rolling "
            "accumulates subscans captured across the LiDAR scan window and is "
            "the strict validation default. synthetic_rolling keeps the older "
            "single-snapshot rays with MID-360-style offsets for ablation; "
            "instantaneous uses zero offsets."
        ),
    )
    parser.add_argument(
        "--imu-acc-mode",
        choices=["gravity_only", "finite_difference"],
        default="finite_difference",
        help=(
            "IMU linear acceleration model. finite_difference is the MuJoCo "
            "live validation default because Fast-LIO needs translational "
            "acceleration during turns; moving live gates still disable ZUPT "
            "so constant-velocity motion is not misclassified as static."
        ),
    )
    parser.add_argument(
        "--nav-data-source",
        choices=["fastlio2", "mujoco_ground_truth"],
        default="fastlio2",
        help=(
            f"Source for canonical {TOPICS.odometry}, {TOPICS.registered_cloud}, "
            f"and {TOPICS.map_cloud}. Use fastlio2 for strict SLAM validation; "
            "use mujoco_ground_truth for stable visible demos while keeping "
            "Fast-LIO diagnostics on their raw topics."
        ),
    )
    parser.add_argument(
        "--allow-golden-spiral-lidar",
        action="store_true",
        help="Use the legacy synthetic ray fan instead of the official MID-360 scan pattern.",
    )
    parser.add_argument("--mujoco-memory", default="64M")
    parser.add_argument("--startup-sleep", type=float, default=2.0)
    parser.add_argument("--settle-sleep", type=float, default=1.0)
    parser.add_argument("--backend-profile", default="fastlio2")
    parser.add_argument("--work-dir", default="artifacts/mujoco_fastlio2_live")
    parser.add_argument("--json-out", default="")
    parser.add_argument(
        "--video-out",
        default="",
        help="Optional MP4 path rendering raw MID-360 scans, Fast-LIO map cloud, and odometry evidence.",
    )
    parser.add_argument("--video-fps", type=float, default=8.0)
    parser.add_argument(
        "--moving-obstacle-mode",
        choices=["none", "robot_crossing"],
        default="none",
        help="Inject simulation-only moving obstacle returns into the raw MID-360 input.",
    )
    parser.add_argument("--moving-obstacle-count", type=int, default=1)
    parser.add_argument("--moving-obstacle-start-s", type=float, default=8.0)
    parser.add_argument("--moving-obstacle-duration-s", type=float, default=0.0)
    parser.add_argument("--moving-obstacle-period-s", type=float, default=10.0)
    parser.add_argument("--moving-obstacle-forward-m", type=float, default=2.0)
    parser.add_argument("--moving-obstacle-forward-step-m", type=float, default=0.8)
    parser.add_argument("--moving-obstacle-lateral-phase-step-rad", type=float, default=math.pi / 2.0)
    parser.add_argument("--moving-obstacle-lateral-amplitude-m", type=float, default=0.75)
    parser.add_argument("--moving-obstacle-along-amplitude-m", type=float, default=0.25)
    parser.add_argument("--moving-obstacle-radius-m", type=float, default=0.16)
    parser.add_argument("--moving-obstacle-height-m", type=float, default=0.60)
    parser.add_argument("--moving-obstacle-point-spacing", type=float, default=0.10)
    parser.add_argument("--moving-obstacle-intensity", type=float, default=220.0)
    parser.add_argument("--moving-obstacle-robot-radius-m", type=float, default=0.28)
    parser.add_argument("--max-yaw-per-meter", type=float, default=1.2)
    parser.add_argument("--max-angular-saturation-ratio", type=float, default=0.35)
    parser.add_argument(
        "--no-save-map-artifacts",
        action="store_true",
        help=f"Do not persist same-source map.pcd/metadata artifacts from {TOPICS.map_cloud}.",
    )
    parser.add_argument(
        "--build-tomogram",
        action="store_true",
        help="Build a same-source PCT tomogram.pickle from the saved map.pcd artifact.",
    )
    parser.add_argument("--map-artifact-voxel-size", type=float, default=0.10)
    parser.add_argument("--map-artifact-max-points", type=int, default=250_000)
    parser.add_argument("--map-artifact-max-span-m", type=float, default=120.0)
    parser.add_argument("--tomogram-resolution", type=float, default=0.20)
    parser.add_argument("--tomogram-slice-dh", type=float, default=0.25)
    parser.add_argument("--tomogram-ground-h", type=float, default=0.0)
    parser.add_argument("--tomogram-max-cells", type=int, default=50_000_000)
    parser.add_argument(
        "--show-mujoco-window",
        action="store_true",
        help="Open a live MuJoCo chase-camera window for visible desktop demos.",
    )
    parser.add_argument("--mujoco-window-fps", type=float, default=10.0)
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    try:
        report = run_gate(
            world=_resolve_world(args.world),
            duration=args.duration,
            drive_vx=args.drive_vx,
            drive_vy=args.drive_vy,
            drive_wz=args.drive_wz,
            n_rays=args.n_rays,
            start=_parse_start(args.start),
            mujoco_memory=args.mujoco_memory,
            mid360_pattern=None if args.allow_golden_spiral_lidar else args.mid360_pattern,
            mid360_samples_per_frame=args.mid360_samples_per_frame,
            startup_sleep=args.startup_sleep,
            settle_sleep=args.settle_sleep,
            work_dir=Path(args.work_dir),
            backend_profile=args.backend_profile,
            drive_mode=args.drive_mode,
            duration_clock=args.duration_clock,
            max_wall_time_s=args.max_wall_time_s,
            drive_source=args.drive_source,
            cmd_vel_timeout=args.cmd_vel_timeout,
            cmd_vel_linear_limit=args.cmd_vel_linear_limit,
            cmd_vel_angular_limit=args.cmd_vel_angular_limit,
            cmd_vel_sim_linear_scale=args.cmd_vel_sim_linear_scale,
            cmd_vel_sim_angular_scale=args.cmd_vel_sim_angular_scale,
            cmd_vel_linear_accel_limit=args.cmd_vel_linear_accel_limit,
            cmd_vel_angular_accel_limit=args.cmd_vel_angular_accel_limit,
            nav_max_linear_speed=args.nav_max_linear_speed,
            nav_max_angular_z=args.nav_max_angular_z,
            nav_turn_speed_yaw_rate_start=args.nav_turn_speed_yaw_rate_start,
            nav_turn_speed_min_scale=args.nav_turn_speed_min_scale,
            run_lingtu_frontier=args.run_lingtu_frontier,
            run_lingtu_tare=args.run_lingtu_tare,
            run_lingtu_inspection=args.run_lingtu_inspection,
            tare_min_goals=args.tare_min_goals,
            tare_start_delay=args.tare_start_delay,
            tare_goal_timeout=args.tare_goal_timeout,
            tare_scenario=args.tare_scenario,
            frontier_min_goals=args.frontier_min_goals,
            frontier_start_delay=args.frontier_start_delay,
            frontier_goal_timeout=args.frontier_goal_timeout,
            inspection_goals=args.inspection_goals,
            inspection_min_checkpoints=args.inspection_min_checkpoints,
            inspection_start_delay=args.inspection_start_delay,
            inspection_goal_timeout=args.inspection_goal_timeout,
            inspection_planner=args.inspection_planner,
            inspection_tomogram=args.inspection_tomogram,
            min_map_area_growth_m2=args.min_map_area_growth_m2,
            min_explored_area_growth_m2=args.min_explored_area_growth_m2,
            min_exploration_coverage_growth_ratio=args.min_exploration_coverage_growth_ratio,
            max_fastlio_z_drift_m=args.max_fastlio_z_drift_m,
            max_fastlio_yaw_drift_rad=args.max_fastlio_yaw_drift_rad,
            runtime_fault_confirm_samples=args.runtime_fault_confirm_samples,
            fastlio_lidar_input=args.fastlio_lidar_input,
            fastlio_lidar_filter_num=args.fastlio_lidar_filter_num,
            fastlio_scan_resolution=args.fastlio_scan_resolution,
            fastlio_map_resolution=args.fastlio_map_resolution,
            fastlio_near_search_num=args.fastlio_near_search_num,
            fastlio_ieskf_max_iter=args.fastlio_ieskf_max_iter,
            fastlio_lidar_cov_inv=args.fastlio_lidar_cov_inv,
            fastlio_time_diff_lidar_to_imu=args.fastlio_time_diff_lidar_to_imu,
            fastlio_vertical_velocity_constraint=args.fastlio_vertical_velocity_constraint,
            scan_time_profile=args.scan_time_profile,
            imu_acc_mode=args.imu_acc_mode,
            nav_data_source=args.nav_data_source,
            show_mujoco_window=args.show_mujoco_window,
            mujoco_window_fps=args.mujoco_window_fps,
            video_out=args.video_out or None,
            video_fps=args.video_fps,
            moving_obstacle_mode=args.moving_obstacle_mode,
            moving_obstacle_count=args.moving_obstacle_count,
            moving_obstacle_start_s=args.moving_obstacle_start_s,
            moving_obstacle_duration_s=args.moving_obstacle_duration_s,
            moving_obstacle_period_s=args.moving_obstacle_period_s,
            moving_obstacle_forward_m=args.moving_obstacle_forward_m,
            moving_obstacle_forward_step_m=args.moving_obstacle_forward_step_m,
            moving_obstacle_lateral_phase_step_rad=args.moving_obstacle_lateral_phase_step_rad,
            moving_obstacle_lateral_amplitude_m=args.moving_obstacle_lateral_amplitude_m,
            moving_obstacle_along_amplitude_m=args.moving_obstacle_along_amplitude_m,
            moving_obstacle_radius_m=args.moving_obstacle_radius_m,
            moving_obstacle_height_m=args.moving_obstacle_height_m,
            moving_obstacle_point_spacing=args.moving_obstacle_point_spacing,
            moving_obstacle_intensity=args.moving_obstacle_intensity,
            moving_obstacle_robot_radius_m=args.moving_obstacle_robot_radius_m,
            max_yaw_per_meter=args.max_yaw_per_meter,
            max_angular_saturation_ratio=args.max_angular_saturation_ratio,
            save_map_artifacts=not args.no_save_map_artifacts,
            build_tomogram=args.build_tomogram,
            map_artifact_voxel_size=args.map_artifact_voxel_size,
            map_artifact_max_points=args.map_artifact_max_points,
            map_artifact_max_span_m=args.map_artifact_max_span_m,
            tomogram_resolution=args.tomogram_resolution,
            tomogram_slice_dh=args.tomogram_slice_dh,
            tomogram_ground_h=args.tomogram_ground_h,
            tomogram_max_cells=args.tomogram_max_cells,
        )
    except Exception as exc:
        report = _gate_exception_report(args, exc)
    text = json.dumps(report, indent=2, sort_keys=True, default=str)
    print(text)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n", encoding="utf-8")
    return 0 if report.get("ok") or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())

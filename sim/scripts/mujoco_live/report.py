"""Report and runtime-contract helpers for the MuJoCo live gate."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any, Sequence

from diagnostics.field.evidence import validate_runtime_evidence
from runtime.runtime_interface import FRAME_LINKS, TOPICS, resolved_runtime_data_flow


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _mujoco_fastlio_contract_definition() -> tuple[dict[str, Any], list[str]]:
    try:
        from runtime.contracts.simulation import simulation_runtime_contract

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
        "body_to_camera": {
            "ok": bool(static_tf_published),
            "parent": FRAME_LINKS["body_to_camera"].parent,
            "child": FRAME_LINKS["body_to_camera"].child,
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
    required_topics = (definition.get("required_runtime_topics") if isinstance(definition, dict) else ()) or ()
    required_slam_topics = (definition.get("required_slam_topics") if isinstance(definition, dict) else ()) or ()
    return {
        "name": "mujoco_fastlio2_live",
        "ok": (
            not definition_errors
            and all((topic_evidence.get(topic) or {}).get("ok") is True for topic in required_topics)
            and all((topic_evidence.get(topic) or {}).get("ok") is True for topic in required_slam_topics)
            and all((item or {}).get("ok") is True for item in frame_evidence.values())
            and all(
                (item or {}).get("ok") is True or (item or {}).get("required") is False
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
    requested_backend = str(getattr(args, "lidar_backend", "") or "")
    mujoco_lidar_backend = str(getattr(args, "mujoco_lidar_backend", "") or "")
    backend_report = {
        "backend": "uninitialized",
        "product_backend": False,
        "product_lidar_backend_verified": False,
        "fallback_used": False,
        "requested_backend": requested_backend,
        "mujoco_lidar_backend": mujoco_lidar_backend,
        "require_product_backend": True,
        "allow_legacy_fallback": bool(getattr(args, "allow_legacy_lidar_fallback", False)),
        "error": "gate_exception_before_lidar_backend_report",
    }
    try:
        pattern_sha256 = _sha256_file(pattern) if pattern_exists else ""
    except Exception:
        pattern_sha256 = ""
    return {
        "kind": (
            f"MuJoCo LiDAR backend {requested_backend or 'unknown'} did not initialize; "
            "official Livox MID-360 scan pattern was configured"
            if pattern_exists
            else f"MuJoCo LiDAR backend {requested_backend or 'unknown'} did not initialize"
        ),
        "backend_report": backend_report,
        "fastlio_lidar_input": getattr(args, "fastlio_lidar_input", ""),
        "scan_time_profile": getattr(args, "scan_time_profile", ""),
        "forced_pattern": pattern_exists,
        "pattern_path": str(pattern) if pattern else "",
        "pattern_sha256": pattern_sha256,
        "samples_per_frame": int(getattr(args, "mid360_samples_per_frame", 0) or 0),
        "fallback_n_rays": int(getattr(args, "n_rays", 0) or 0),
    }


def _exception_partial_report_path(args: argparse.Namespace) -> Path | None:
    explicit = getattr(args, "partial_json_out", None)
    if explicit:
        return Path(explicit)
    json_out = getattr(args, "json_out", None)
    if json_out:
        return Path(json_out).with_suffix(".partial.json")
    return None


def _load_exception_partial_report(
    args: argparse.Namespace,
) -> tuple[dict[str, Any] | None, Path | None]:
    partial_path = _exception_partial_report_path(args)
    if partial_path is None or not partial_path.is_file():
        return None, partial_path
    try:
        payload = json.loads(partial_path.read_text(encoding="utf-8"))
    except Exception:
        return None, partial_path
    if not isinstance(payload, dict):
        return None, partial_path
    return payload, partial_path


def _append_unique(items: list[str], values: Sequence[Any]) -> list[str]:
    seen = set(items)
    for value in values:
        text = str(value)
        if not text or text in seen:
            continue
        items.append(text)
        seen.add(text)
    return items


def _merge_exception_partial_report(
    report: dict[str, Any],
    partial_report: dict[str, Any] | None,
    partial_report_path: Path | None,
) -> dict[str, Any]:
    if not partial_report:
        if partial_report_path is not None:
            report["partial_report_path"] = partial_report_path.as_posix()
            report["partial_report_available"] = False
        return report

    report["partial_report_path"] = partial_report_path.as_posix() if partial_report_path else ""
    report["partial_report_available"] = True
    report["partial_report"] = partial_report
    for key in (
        "world",
        "nav_data_source",
        "elapsed_wall_s",
        "elapsed_sim_s",
        "counts",
        "outputs",
        "lingtu_inspection",
        "video",
        "runtime_fault_events",
        "gate_wall_timeout",
        "process_returncode",
    ):
        if key in partial_report:
            report[key] = partial_report[key]

    simulation_path = partial_report.get("simulation_path")
    if isinstance(simulation_path, dict):
        report["simulation_path"] = simulation_path
        for key in (
            "first_sim_xyz",
            "last_sim_xyz",
            "first_sim_yaw_rad",
            "last_sim_yaw_rad",
            "sim_path_length_m",
        ):
            if key in simulation_path:
                report[key] = simulation_path[key]

    partial_faults = [str(fault) for fault in (partial_report.get("runtime_faults") or []) if str(fault)]
    report["runtime_faults"] = _append_unique(
        list(report.get("runtime_faults") or []),
        partial_faults,
    )
    report["remaining_gaps"] = _append_unique(
        list(report.get("remaining_gaps") or []),
        [f"partial_runtime_fault: {fault}" for fault in partial_faults],
    )
    report["ok"] = False
    return report


def _gate_exception_report(
    args: argparse.Namespace,
    exc: Exception,
    *,
    partial_report: dict[str, Any] | None = None,
    partial_report_path: Path | None = None,
) -> dict[str, Any]:
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
    localization_backend = str(getattr(args, "localization_backend", "") or "removed")
    localization_algorithm = "removed"
    lidar_backend_report = {
        "backend": "uninitialized",
        "product_backend": False,
        "product_lidar_backend_verified": False,
        "fallback_used": False,
        "requested_backend": str(getattr(args, "lidar_backend", "") or ""),
        "mujoco_lidar_backend": str(getattr(args, "mujoco_lidar_backend", "") or ""),
        "require_product_backend": True,
        "allow_legacy_fallback": bool(getattr(args, "allow_legacy_lidar_fallback", False)),
        "error": runtime_fault,
    }
    remaining_gaps = [
        f"gate_exception: {runtime_fault}",
        *list(runtime_evidence.blockers),
    ]
    deliverable_contract = {
        "target": (
            "MuJoCo MID-360/IMU -> removed localization backend -> LingTu core msg /nav outputs -> "
            "map artifacts/tomogram"
        ),
        "simulation_only": True,
        "hardware_output_forbidden": True,
        "checks": {
            "product_mujoco_lidar_backend": False,
            "raw_mujoco_lidar": False,
            "raw_mujoco_imu": False,
            "localization_odometry_and_map": False,
            "canonical_nav_kernel_msgs": False,
            "no_ros_default_runtime": True,
            "no_ros_message_shim": True,
            "official_fastlio2_verified": False,
            "same_source_map_artifact": False,
            "nav_cmd_vel_nonzero": None,
            "frontier_or_exploration": None,
            "tare_native_exploration": None,
            "inspection_patrol": None,
            "moving_obstacle_evidence": None,
            "visible_mujoco_window": None,
        },
        "stop_condition": "ok=true and remaining_gaps=[]",
        "remaining_gaps": remaining_gaps,
    }
    report = {
        "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
        "ok": False,
        "algorithm": localization_algorithm,
        "localization_backend": localization_backend,
        "localization_runtime": {
            "backend": localization_backend,
            "algorithm": localization_algorithm,
            "transport": "in_process_core_msgs",
            "official_fastlio2": False,
            "official_fastlio2_verified": False,
            "requires_ros2": False,
            "requires_rclpy": False,
            "requires_sensor_msgs": False,
            "requires_nav_msgs": False,
            "requires_std_msgs": False,
            "requires_colcon": False,
            "requires_fastlio2_package": False,
            "uses_ros_message_shim": False,
            "uses_ros_graph": False,
        },
        "deliverable_contract": deliverable_contract,
        "remaining_gaps": remaining_gaps,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "duration_clock": args.duration_clock,
        "scan_time_profile": args.scan_time_profile,
        "runtime_faults": [runtime_fault],
        "runtime_contract": runtime_contract,
        "runtime_evidence": _runtime_evidence_report(runtime_evidence),
        "hardware_safety": hardware_safety,
        "lidar_backend": lidar_backend_report,
        "product_lidar_backend_verified": False,
        "lidar_source": _exception_lidar_source(args),
        "args": vars(args),
    }
    return _merge_exception_partial_report(
        report,
        partial_report=partial_report,
        partial_report_path=partial_report_path,
    )


def _wall_timeout_status(elapsed_wall_s: float, max_wall_time_s: float) -> dict[str, Any]:
    elapsed = max(0.0, float(elapsed_wall_s))
    limit = max(0.0, float(max_wall_time_s or 0.0))
    triggered = bool(limit > 0.0 and elapsed >= limit)
    return {
        "enabled": bool(limit > 0.0),
        "triggered": triggered,
        "elapsed_wall_s": round(elapsed, 3),
        "max_wall_time_s": float(limit),
        "fault": (f"gate wall timeout after {elapsed:.1f}s (limit={limit:.1f}s)" if triggered else ""),
    }


def _write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f"{path.name}.tmp")
    tmp.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )
    tmp.replace(path)


def _video_sample_elapsed_s(
    duration_clock: str,
    *,
    elapsed_sim_s: float,
    elapsed_wall_s: float,
) -> float:
    if str(duration_clock or "wall").strip().lower() == "sim":
        return float(elapsed_sim_s)
    return float(elapsed_wall_s)


def _inspection_gate_evidence_complete(
    *,
    run_lingtu_inspection: bool,
    navigation_health: dict[str, Any],
    inspection_goal_count: int,
    inspection_min_checkpoints: int,
    algorithm_verified: bool,
    canonical_nav_outputs_verified: bool,
    global_path_counts: Sequence[int],
    local_path_counts: Sequence[int],
    nav_cmd_nonzero: int,
    moving_obstacle_enabled: bool,
    moving_obstacle_published_update_count: int,
    moving_obstacle_published_point_count_max: int,
    video_required: bool,
    video_sample_count: int,
) -> dict[str, Any]:
    if not run_lingtu_inspection:
        return {"ok": False, "reason": "inspection disabled"}
    patrol_total = int(navigation_health.get("patrol_total") or inspection_goal_count or 0)
    patrol_index = int(navigation_health.get("patrol_index") or 0)
    patrol_state = str(navigation_health.get("state") or "").upper()
    required_checkpoints = max(1, int(inspection_min_checkpoints))
    successful_checkpoints = min(max(patrol_index, 0), patrol_total)
    patrol_success = bool(patrol_state == "SUCCESS" and patrol_total > 0 and patrol_index >= patrol_total)
    checkpoint_count_ok = bool(successful_checkpoints >= required_checkpoints)
    terminal_success = patrol_success and checkpoint_count_ok
    local_path_had_trackable_segment = bool(local_path_counts and max(int(count) for count in local_path_counts) >= 2)
    latest_local_path_active = bool(local_path_counts and int(local_path_counts[-1]) >= 2)
    early_success = bool(
        not terminal_success and latest_local_path_active and canonical_nav_outputs_verified and nav_cmd_nonzero > 0
    )
    checks = {
        "patrol_success": patrol_success,
        "checkpoint_count": checkpoint_count_ok,
        "fastlio2_algorithm_outputs": bool(algorithm_verified),
        "canonical_nav_outputs": bool(canonical_nav_outputs_verified),
        "global_path": bool(global_path_counts and max(global_path_counts) > 0),
        "local_path": bool(local_path_had_trackable_segment and (terminal_success or latest_local_path_active)),
        "nav_cmd_nonzero": bool(nav_cmd_nonzero > 0),
        "moving_obstacles": bool(
            not moving_obstacle_enabled
            or (moving_obstacle_published_update_count > 0 and moving_obstacle_published_point_count_max > 0)
        ),
        "video_samples": bool(not video_required or video_sample_count > 0),
    }
    missing = [name for name, passed in checks.items() if not passed]
    return {
        "ok": not missing,
        "reason": ("inspection evidence complete" if not missing else "inspection evidence incomplete"),
        "missing": missing,
        "checks": checks,
        "successful_checkpoints": successful_checkpoints,
        "required_checkpoints": required_checkpoints,
        "terminal_success": terminal_success,
        "early_success": early_success,
        "patrol_state": patrol_state,
        "patrol_index": patrol_index,
        "patrol_total": patrol_total,
    }

#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import json
import math
from pathlib import Path
from typing import Any, Sequence


ROOT = Path(__file__).resolve().parents[2]


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _safe_float(value: Any, *, default: float | None = None) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(result):
        return default
    return result


def _safe_int(value: Any, *, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _xy(value: Any) -> tuple[float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        return None
    x = _safe_float(value[0])
    y = _safe_float(value[1])
    if x is None or y is None:
        return None
    return x, y


def _xy_list(value: tuple[float, float] | None) -> list[float] | None:
    if value is None:
        return None
    return [round(float(value[0]), 4), round(float(value[1]), 4)]


def _distance(a: tuple[float, float] | None, b: tuple[float, float] | None) -> float | None:
    if a is None or b is None:
        return None
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _yaw_delta(a: Any, b: Any) -> float | None:
    first = _safe_float(a)
    last = _safe_float(b)
    if first is None or last is None:
        return None
    return abs((last - first + math.pi) % (2.0 * math.pi) - math.pi)


def _subcheck_ok(report: dict[str, Any], key: str, blockers: list[str]) -> dict[str, Any]:
    value = report.get(key) if isinstance(report.get(key), dict) else {}
    if value.get("checked") is not True:
        blockers.append(f"{key}.checked is not true")
    if value.get("ok") is not True:
        blockers.append(f"{key}.ok is not true")
    return value


def _string_list(value: Any) -> list[str]:
    if not isinstance(value, list):
        return []
    return [str(item) for item in value if str(item)]


def _is_validation_harness_fault(text: str) -> bool:
    normalized = text.lower()
    return any(
        marker in normalized
        for marker in (
            "gate_exception",
            "gate wall timeout",
            "publisher's context is invalid",
            "rclerror",
            "ros2 python modules are unavailable",
        )
    )


def _video_evidence(report: dict[str, Any]) -> dict[str, Any]:
    video = report.get("video") if isinstance(report.get("video"), dict) else {}
    return {
        "path": str(report.get("video_path") or video.get("path") or ""),
        "frame_count": _safe_int(report.get("video_frame_count", video.get("frames"))),
        "sample_count": _safe_int(report.get("video_sample_count", video.get("samples"))),
    }


def _scan_time_evidence(report: dict[str, Any]) -> dict[str, str]:
    lidar_source = (
        report.get("lidar_source") if isinstance(report.get("lidar_source"), dict) else {}
    )
    profile = str(
        report.get("scan_time_profile")
        or lidar_source.get("scan_time_profile")
        or ""
    ).strip()
    contract = str(
        report.get("scan_time_model_contract")
        or lidar_source.get("scan_time_model_contract")
        or ""
    ).strip()
    return {
        "profile": profile,
        "contract": contract,
    }


def _runtime_time_alignment(report: dict[str, Any]) -> dict[str, Any]:
    direct = report.get("time_alignment")
    if isinstance(direct, dict) and direct:
        return direct
    events = report.get("runtime_fault_events")
    if not isinstance(events, list):
        return {}
    for event in reversed(events):
        if not isinstance(event, dict):
            continue
        alignment = event.get("time_alignment")
        if isinstance(alignment, dict) and alignment:
            return alignment
    return {}


def _goal_span_m(start_xy: tuple[float, float] | None, goals: list[Any]) -> float | None:
    points = [point for point in [start_xy, *(_xy(goal) for goal in goals)] if point is not None]
    if len(points) < 2:
        return None
    return max(_distance(a, b) or 0.0 for a in points for b in points)


def _artifact_parent(value: Any) -> Path | None:
    if not isinstance(value, str) or not value.strip():
        return None
    path = Path(value.strip())
    if not path.is_absolute():
        path = ROOT / path
    return path.resolve(strict=False).parent


def _same_source_artifacts(report: dict[str, Any], inspection: dict[str, Any]) -> dict[str, Any]:
    assets = report.get("assets") if isinstance(report.get("assets"), dict) else {}
    world = str(report.get("world") or "")
    tomogram = str(inspection.get("tomogram") or assets.get("tomogram") or "")
    world_parent = _artifact_parent(world)
    tomogram_parent = _artifact_parent(tomogram)
    return {
        "ok": bool(world_parent is not None and tomogram_parent is not None and world_parent == tomogram_parent),
        "world": world,
        "tomogram": tomogram,
        "world_parent": "" if world_parent is None else str(world_parent),
        "tomogram_parent": "" if tomogram_parent is None else str(tomogram_parent),
    }


def _frame_metadata(same_source: dict[str, Any]) -> dict[str, Any]:
    parent = same_source.get("tomogram_parent") or same_source.get("world_parent") or ""
    path = Path(str(parent)) / "metadata.json" if parent else None
    result: dict[str, Any] = {
        "ok": False,
        "path": "" if path is None else str(path),
        "map_frame": "",
        "map_frame_origin_world_xy": None,
        "start": None,
        "blockers": [],
    }
    if not same_source.get("ok"):
        return result
    if path is None or not path.is_file():
        result["blockers"].append("same-source metadata.json missing")
        return result
    try:
        metadata = _load_json(path)
    except Exception as exc:
        result["blockers"].append(f"same-source metadata.json invalid: {exc}")
        return result
    origin = _xy(metadata.get("map_frame_origin_world_xy"))
    if origin is None:
        result["blockers"].append("same-source metadata.map_frame_origin_world_xy missing")
    result.update(
        {
            "map_frame": str(metadata.get("map_frame") or ""),
            "map_frame_origin_world_xy": None if origin is None else [origin[0], origin[1]],
            "start": metadata.get("start"),
        }
    )
    result["ok"] = not result["blockers"]
    return result


def _to_map_frame_xy(
    world_xy: tuple[float, float] | None,
    metadata: dict[str, Any],
) -> tuple[float, float] | None:
    if world_xy is None:
        return None
    origin = _xy(metadata.get("map_frame_origin_world_xy"))
    if origin is None:
        return world_xy
    return world_xy[0] - origin[0], world_xy[1] - origin[1]


def _goal_frame_metadata(
    metadata: dict[str, Any],
    inspection: dict[str, Any],
) -> dict[str, Any]:
    result = dict(metadata)
    origin = _xy(inspection.get("planning_frame_origin_world_xy"))
    if origin is None:
        origin = _xy(inspection.get("map_frame_origin_world_xy"))
    if origin is None:
        result["source"] = "same_source_metadata"
        return result
    frame = str(inspection.get("planning_frame") or inspection.get("frame_id") or "")
    result.update(
        {
            "map_frame": frame or str(result.get("map_frame") or ""),
            "map_frame_origin_world_xy": [origin[0], origin[1]],
            "source": "lingtu_inspection.planning_frame_origin_world_xy",
        }
    )
    return result


def _case_from_report(
    path: Path,
    report: dict[str, Any],
    *,
    min_path_length_m: float,
    min_goal_span_m: float,
    max_loop_closure_error_m: float,
    max_fastlio_loop_closure_error_m: float,
    max_loop_yaw_error_rad: float,
    max_goal_start_error_m: float,
    min_checkpoints: int,
    require_video: bool,
    require_video_file: bool,
    required_scan_time_profile: str,
) -> dict[str, Any]:
    blockers: list[str] = []
    outputs = report.get("outputs") if isinstance(report.get("outputs"), dict) else {}
    inspection = (
        report.get("lingtu_inspection")
        if isinstance(report.get("lingtu_inspection"), dict)
        else {}
    )
    navigation_chain = (
        report.get("navigation_chain")
        if isinstance(report.get("navigation_chain"), dict)
        else {}
    )
    remaining_gaps = _string_list(report.get("remaining_gaps"))
    runtime_faults = _string_list(report.get("runtime_faults"))
    gate_wall_timeout = (
        report.get("gate_wall_timeout")
        if isinstance(report.get("gate_wall_timeout"), dict)
        else {}
    )
    same_source = _same_source_artifacts(report, inspection)
    metadata = _frame_metadata(same_source)
    goal_metadata = _goal_frame_metadata(metadata, inspection)
    video = _video_evidence(report)
    scan_time = _scan_time_evidence(report)
    required_scan_time_profile = str(required_scan_time_profile or "").strip()
    first_sim_xy = _xy(report.get("first_sim_xyz"))
    last_sim_xy = _xy(report.get("last_sim_xyz"))
    first_sim_map_xy = _to_map_frame_xy(first_sim_xy, goal_metadata)
    last_sim_map_xy = _to_map_frame_xy(last_sim_xy, goal_metadata)
    first_odom_xy = _xy(report.get("first_odom_xyz"))
    last_odom_xy = _xy(report.get("last_odom_xyz"))
    sim_loop_error = _distance(first_sim_xy, last_sim_xy)
    fastlio_loop_error = _distance(first_odom_xy, last_odom_xy)
    sim_yaw_error = _yaw_delta(report.get("first_sim_yaw_rad"), report.get("last_sim_yaw_rad"))
    fastlio_yaw_error = _yaw_delta(report.get("first_odom_yaw_rad"), report.get("last_odom_yaw_rad"))
    sim_path_length = _safe_float(report.get("sim_path_length_m"), default=0.0) or 0.0
    fastlio_path_length = _safe_float(report.get("fastlio2_path_length_m"), default=0.0) or 0.0
    goals = list(inspection.get("goals") or [])
    final_goal_xy = _xy(goals[-1]) if goals else None
    metadata_start_xy = _xy(metadata.get("start"))
    runtime_start_xy = _xy(report.get("start_position"))
    final_goal_to_start = _distance(first_sim_map_xy or first_sim_xy, final_goal_xy)
    goal_span = _goal_span_m(first_sim_map_xy or first_sim_xy, goals)

    if report.get("ok") is not True:
        blockers.append("report.ok is not true")
    if required_scan_time_profile:
        if scan_time["profile"] != required_scan_time_profile:
            blockers.append(f"scan_time_profile is not {required_scan_time_profile}")
        if (
            required_scan_time_profile == "physical_rolling"
            and scan_time["contract"]
            != "physical_subscans_with_actual_sim_time_offsets"
        ):
            blockers.append("scan_time_model_contract is not physical rolling")
    for fault in [*remaining_gaps, *runtime_faults]:
        if _is_validation_harness_fault(fault):
            blockers.append(f"validation_harness fault: {fault}")
    if gate_wall_timeout.get("triggered") is True:
        blockers.append("validation_harness fault: gate_wall_timeout.triggered")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if report.get("real_robot_motion") is not False:
        blockers.append("real_robot_motion is not false")
    if report.get("cmd_vel_sent_to_hardware") is not False:
        blockers.append("cmd_vel_sent_to_hardware is not false")
    for key in (
        "live_mujoco_lidar_verified",
        "live_mujoco_imu_verified",
        "slam_algorithm_output_verified",
        "canonical_nav_outputs_verified",
        "bridge_verified",
    ):
        if report.get(key) is not True:
            blockers.append(f"{key} is not true")
    for key in ("nav_odometry", "nav_registered_cloud", "nav_map_cloud"):
        if _safe_int(outputs.get(key)) <= 0:
            blockers.append(f"outputs.{key} missing")
    if _safe_int(outputs.get("nav_cmd_vel_nonzero")) <= 0:
        blockers.append("outputs.nav_cmd_vel_nonzero missing")

    if inspection.get("enabled") is not True:
        blockers.append("lingtu_inspection.enabled is not true")
    if inspection.get("verified") is not True:
        blockers.append("lingtu_inspection.verified is not true")
    if str(inspection.get("global_planner") or "").lower() != "pct":
        blockers.append("lingtu_inspection.global_planner is not pct")
    if not same_source["ok"]:
        blockers.append("same-source world/tomogram mismatch")
    elif not metadata["ok"]:
        blockers.extend(str(blocker) for blocker in metadata.get("blockers") or [])
    else:
        start_delta = _distance(metadata_start_xy, runtime_start_xy)
        if (
            metadata_start_xy is not None
            and runtime_start_xy is not None
            and start_delta is not None
            and start_delta > 0.25
        ):
            blockers.append("runtime start_position does not match same-source metadata.start")
    if str(inspection.get("patrol_state") or "").upper() != "SUCCESS":
        blockers.append("lingtu_inspection.patrol_state is not SUCCESS")
    if _safe_int(inspection.get("goal_count")) < min_checkpoints:
        blockers.append(f"lingtu_inspection.goal_count < {min_checkpoints}")
    if _safe_int(inspection.get("successful_navigation_goal_count")) < min_checkpoints:
        blockers.append("lingtu_inspection successful checkpoints below loop threshold")
    if _safe_int(inspection.get("failed_navigation_goal_count")) > 0:
        blockers.append("lingtu_inspection.failed_navigation_goal_count is nonzero")
    if _safe_int(inspection.get("global_path_points_max")) < 2:
        blockers.append("lingtu_inspection.global_path_points_max below required")
    if _safe_int(inspection.get("local_path_points_max")) < 2:
        blockers.append("lingtu_inspection.local_path_points_max below required")
    if _safe_int(inspection.get("local_path_count")) <= 0:
        blockers.append("lingtu_inspection.local_path_count missing")

    if navigation_chain.get("planner_fallback_used") is not False:
        blockers.append("navigation_chain.planner_fallback_used is not false")
    if navigation_chain.get("planner_repair_used") is not False:
        blockers.append("navigation_chain.planner_repair_used is not false")
    direct_goal_fallback = navigation_chain.get("direct_goal_fallback")
    if isinstance(direct_goal_fallback, dict) and direct_goal_fallback.get("used") is True:
        blockers.append("navigation_chain.direct_goal_fallback.used is true")

    motion_check = _subcheck_ok(report, "fastlio2_motion_consistency", blockers)
    z_check = _subcheck_ok(report, "fastlio2_z_consistency", blockers)
    yaw_check = _subcheck_ok(report, "fastlio2_yaw_consistency", blockers)
    navigation_diagnostics = (
        report.get("navigation_diagnostics")
        if isinstance(report.get("navigation_diagnostics"), dict)
        else {}
    )
    diagnostic_report = (
        report.get("fastlio_large_loop_diagnostic_report")
        if isinstance(report.get("fastlio_large_loop_diagnostic_report"), dict)
        else {}
    )
    time_alignment = _runtime_time_alignment(report)
    diagnostic_samples = navigation_diagnostics.get("samples")
    if not isinstance(diagnostic_samples, list):
        diagnostic_samples = navigation_diagnostics.get("samples_tail")
    if not isinstance(diagnostic_samples, list):
        diagnostic_samples = []
    diagnostic_last_sample = navigation_diagnostics.get("last_sample")
    if not isinstance(diagnostic_last_sample, dict):
        diagnostic_last_sample = diagnostic_samples[-1] if diagnostic_samples else {}

    if sim_path_length < min_path_length_m:
        blockers.append(f"sim_path_length_m < {min_path_length_m:g}")
    if fastlio_path_length < min_path_length_m * 0.8:
        blockers.append(f"fastlio2_path_length_m < {min_path_length_m * 0.8:g}")
    if goal_span is None or goal_span < min_goal_span_m:
        blockers.append(f"inspection goal span < {min_goal_span_m:g}")
    if final_goal_to_start is None or final_goal_to_start > max_goal_start_error_m:
        blockers.append(f"final inspection goal is not near loop start <= {max_goal_start_error_m:g}m")
    if sim_loop_error is None or sim_loop_error > max_loop_closure_error_m:
        blockers.append(f"sim_loop_closure_error_m > {max_loop_closure_error_m:g}")
    if fastlio_loop_error is None or fastlio_loop_error > max_fastlio_loop_closure_error_m:
        blockers.append(f"fastlio2_loop_closure_error_m > {max_fastlio_loop_closure_error_m:g}")
    if sim_yaw_error is None or sim_yaw_error > max_loop_yaw_error_rad:
        blockers.append(f"sim_loop_yaw_error_rad > {max_loop_yaw_error_rad:g}")
    if fastlio_yaw_error is None or fastlio_yaw_error > max_loop_yaw_error_rad:
        blockers.append(f"fastlio2_loop_yaw_error_rad > {max_loop_yaw_error_rad:g}")

    if require_video:
        if not video["path"]:
            blockers.append("video_path missing")
        if int(video["frame_count"]) <= 0:
            blockers.append("video_frame_count missing")
        if int(video["sample_count"]) <= 0:
            blockers.append("video_sample_count missing")
    if require_video_file and video["path"] and not Path(str(video["path"])).is_file():
        blockers.append("video file missing")

    return {
        "path": str(path),
        "ok": not blockers,
        "global_planner": inspection.get("global_planner"),
        "scan_time_profile": scan_time["profile"],
        "scan_time_model_contract": scan_time["contract"],
        "world": same_source["world"],
        "inspection_tomogram": same_source["tomogram"],
        "same_source_artifacts": bool(same_source["ok"]),
        "world_parent": same_source["world_parent"],
        "tomogram_parent": same_source["tomogram_parent"],
        "deliverable_contract": (
            report.get("deliverable_contract")
            if isinstance(report.get("deliverable_contract"), dict)
            else {}
        ),
        "map_artifacts": (
            report.get("map_artifacts")
            if isinstance(report.get("map_artifacts"), dict)
            else {}
        ),
        "assets": report.get("assets") if isinstance(report.get("assets"), dict) else {},
        "metadata": metadata["path"],
        "map_frame": metadata["map_frame"],
        "map_frame_origin_world_xy": metadata["map_frame_origin_world_xy"],
        "goal_frame": goal_metadata["map_frame"],
        "goal_frame_origin_world_xy": goal_metadata["map_frame_origin_world_xy"],
        "goal_frame_origin_source": goal_metadata.get("source", "same_source_metadata"),
        "metadata_start_xy": _xy_list(metadata_start_xy),
        "runtime_start_position_xy": _xy_list(runtime_start_xy),
        "first_sim_xy": _xy_list(first_sim_xy),
        "last_sim_xy": _xy_list(last_sim_xy),
        "first_sim_map_xy": _xy_list(first_sim_map_xy),
        "last_sim_map_xy": _xy_list(last_sim_map_xy),
        "final_goal_xy": _xy_list(final_goal_xy),
        "inspection_goals": goals[-20:],
        "goal_count": inspection.get("goal_count"),
        "successful_navigation_goal_count": inspection.get("successful_navigation_goal_count"),
        "global_path_points_max": inspection.get("global_path_points_max"),
        "local_path_count": inspection.get("local_path_count"),
        "local_path_points_max": inspection.get("local_path_points_max"),
        "nav_cmd_vel_nonzero": outputs.get("nav_cmd_vel_nonzero"),
        "sim_path_length_m": round(float(sim_path_length), 4),
        "fastlio2_path_length_m": round(float(fastlio_path_length), 4),
        "fastlio2_moved_m": motion_check.get("fastlio2_moved_m"),
        "sim_moved_m": motion_check.get("sim_moved_m"),
        "fastlio2_z_delta_error_m": z_check.get("z_delta_error_m"),
        "max_allowed_z_drift_m": z_check.get("max_allowed_z_drift_m"),
        "fastlio2_yaw_delta_error_rad": yaw_check.get("yaw_delta_error_rad"),
        "max_allowed_yaw_drift_rad": yaw_check.get("max_allowed_yaw_drift_rad"),
        "time_alignment": time_alignment,
        "goal_span_m": None if goal_span is None else round(float(goal_span), 4),
        "final_goal_to_start_m": (
            None if final_goal_to_start is None else round(float(final_goal_to_start), 4)
        ),
        "sim_loop_closure_error_m": (
            None if sim_loop_error is None else round(float(sim_loop_error), 4)
        ),
        "fastlio2_loop_closure_error_m": (
            None if fastlio_loop_error is None else round(float(fastlio_loop_error), 4)
        ),
        "sim_loop_yaw_error_rad": None if sim_yaw_error is None else round(float(sim_yaw_error), 4),
        "fastlio2_loop_yaw_error_rad": (
            None if fastlio_yaw_error is None else round(float(fastlio_yaw_error), 4)
        ),
        "navigation_diagnostics": {
            "sample_count": navigation_diagnostics.get("sample_count", len(diagnostic_samples)),
            "sample_period_s": navigation_diagnostics.get("sample_period_s"),
            "last_sample": diagnostic_last_sample,
            "samples_tail": diagnostic_samples[-5:],
        },
        "diagnostic_report": diagnostic_report,
        "remaining_gaps": remaining_gaps,
        "runtime_faults": runtime_faults,
        "gate_wall_timeout": gate_wall_timeout,
        "video_path": video["path"],
        "video_frame_count": video["frame_count"],
        "video_sample_count": video["sample_count"],
        "blockers": blockers,
    }


def _blocking_subsystem(case: dict[str, Any]) -> str:
    blockers = [str(blocker) for blocker in case.get("blockers") or []]
    joined = "\n".join(blockers).lower()
    local_paths_alive = _safe_int(case.get("local_path_count")) > 0
    nav_cmd_alive = _safe_int(case.get("nav_cmd_vel_nonzero")) > 0
    mission_progress_fault = any(
        marker in joined
        for marker in (
            "patrol_state",
            "successful checkpoints",
            "checkpoint count",
            "sim_path_length_m",
            "fastlio2_path_length_m",
            "loop_closure_error",
            "loop_yaw_error",
        )
    )
    wall_timeout = (
        "gate_wall_timeout" in joined
        or "gate wall timeout" in joined
        or (
            isinstance(case.get("gate_wall_timeout"), dict)
            and case["gate_wall_timeout"].get("triggered") is True
        )
    )
    if wall_timeout and mission_progress_fault and (local_paths_alive or nav_cmd_alive):
        return "planning_tracking"
    if "validation_harness fault" in joined:
        return "validation_harness"
    if "same-source" in joined or "metadata" in joined:
        return "artifact_source"
    if (
        "fastlio2_" in joined
        or "slam_" in joined
        or "live_mujoco_lidar" in joined
        or "live_mujoco_imu" in joined
        or "canonical_nav_outputs" in joined
        or "bridge_verified" in joined
    ):
        return "slam_localization"
    if "global_planner" in joined or "goal span" in joined or "patrol_state" in joined:
        return "global_planning"
    if "local_path" in joined or "nav_cmd_vel" in joined or "cmd_vel" in joined:
        return "local_planning_control"
    if "video" in joined:
        return "video_evidence"
    return "unknown"


def _minimal_red_defect(cases: Sequence[dict[str, Any]]) -> dict[str, Any]:
    failed = [case for case in cases if case.get("ok") is not True]
    if not failed:
        return {}
    case = sorted(
        failed,
        key=lambda item: (
            0 if str(item.get("global_planner") or "").lower() == "pct" else 1,
            0 if _safe_int(item.get("local_path_count")) > 0 else 1,
            0 if _safe_int(item.get("nav_cmd_vel_nonzero")) > 0 else 1,
            -float(item.get("sim_path_length_m") or 0.0),
        ),
    )[0]
    return {
        "blocking_subsystem": _blocking_subsystem(case),
        "path": case.get("path"),
        "blockers": list(case.get("blockers") or []),
        "global_planner": case.get("global_planner"),
        "global_path_points_max": case.get("global_path_points_max"),
        "local_path_count": case.get("local_path_count"),
        "local_path_points_max": case.get("local_path_points_max"),
        "nav_cmd_vel_nonzero": case.get("nav_cmd_vel_nonzero"),
        "map_frame": case.get("map_frame"),
        "map_frame_origin_world_xy": case.get("map_frame_origin_world_xy"),
        "goal_frame": case.get("goal_frame"),
        "goal_frame_origin_world_xy": case.get("goal_frame_origin_world_xy"),
        "goal_frame_origin_source": case.get("goal_frame_origin_source"),
        "metadata_start_xy": case.get("metadata_start_xy"),
        "runtime_start_position_xy": case.get("runtime_start_position_xy"),
        "first_sim_xy": case.get("first_sim_xy"),
        "first_sim_map_xy": case.get("first_sim_map_xy"),
        "final_goal_xy": case.get("final_goal_xy"),
        "final_goal_to_start_m": case.get("final_goal_to_start_m"),
        "sim_path_length_m": case.get("sim_path_length_m"),
        "fastlio2_path_length_m": case.get("fastlio2_path_length_m"),
        "sim_loop_closure_error_m": case.get("sim_loop_closure_error_m"),
        "fastlio2_loop_closure_error_m": case.get("fastlio2_loop_closure_error_m"),
        "fastlio2_z_delta_error_m": case.get("fastlio2_z_delta_error_m"),
        "fastlio2_yaw_delta_error_rad": case.get("fastlio2_yaw_delta_error_rad"),
        "time_alignment": case.get("time_alignment"),
        "diagnostic_report": case.get("diagnostic_report") or {},
        "remaining_gaps": list(case.get("remaining_gaps") or []),
        "runtime_faults": list(case.get("runtime_faults") or []),
        "gate_wall_timeout": case.get("gate_wall_timeout") or {},
    }


def evaluate_reports(
    report_paths: Sequence[Path | str],
    *,
    min_path_length_m: float = 20.0,
    min_goal_span_m: float = 4.0,
    max_loop_closure_error_m: float = 0.75,
    max_fastlio_loop_closure_error_m: float = 1.0,
    max_loop_yaw_error_rad: float = 0.5,
    max_goal_start_error_m: float = 1.0,
    min_checkpoints: int = 4,
    require_video: bool = True,
    require_video_file: bool = False,
    required_scan_time_profile: str = "physical_rolling",
) -> dict[str, Any]:
    cases: list[dict[str, Any]] = []
    blockers: list[str] = []
    for path_like in report_paths:
        path = Path(path_like)
        try:
            cases.append(
                _case_from_report(
                    path,
                    _load_json(path),
                    min_path_length_m=min_path_length_m,
                    min_goal_span_m=min_goal_span_m,
                    max_loop_closure_error_m=max_loop_closure_error_m,
                    max_fastlio_loop_closure_error_m=max_fastlio_loop_closure_error_m,
                    max_loop_yaw_error_rad=max_loop_yaw_error_rad,
                    max_goal_start_error_m=max_goal_start_error_m,
                    min_checkpoints=min_checkpoints,
                    require_video=require_video,
                    require_video_file=require_video_file,
                    required_scan_time_profile=required_scan_time_profile,
                )
            )
        except Exception as exc:  # pragma: no cover - preserves CLI diagnostics.
            cases.append({"path": str(path), "ok": False, "blockers": [str(exc)]})

    passed = [case for case in cases if case.get("ok") is True]
    if not report_paths:
        blockers.append("no large-loop runtime reports provided")
    if not passed:
        blockers.append("no passing large-loop runtime report")

    best_case = {}
    if passed:
        best_case = sorted(
            passed,
            key=lambda case: (
                -float(case.get("sim_path_length_m") or 0.0),
                float(case.get("sim_loop_closure_error_m") or 999.0),
            ),
        )[0]
    minimal_red_defect = _minimal_red_defect(cases)
    blocking_subsystems = sorted(
        {
            str(defect)
            for defect in (
                _blocking_subsystem(case)
                for case in cases
                if case.get("ok") is not True
            )
            if defect
        }
    )

    return {
        "schema_version": "lingtu.large_loop_closure_gate.v1",
        "ok": not blockers,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "case_count": len(cases),
        "passed_case_count": len(passed),
        "failed_case_count": len(cases) - len(passed),
        "best_case": best_case,
        "minimal_red_defect": minimal_red_defect,
        "blocking_subsystems": blocking_subsystems,
        "thresholds": {
            "min_path_length_m": float(min_path_length_m),
            "min_goal_span_m": float(min_goal_span_m),
            "max_loop_closure_error_m": float(max_loop_closure_error_m),
            "max_fastlio_loop_closure_error_m": float(max_fastlio_loop_closure_error_m),
            "max_loop_yaw_error_rad": float(max_loop_yaw_error_rad),
            "max_goal_start_error_m": float(max_goal_start_error_m),
            "min_checkpoints": int(min_checkpoints),
            "require_video": bool(require_video),
            "require_video_file": bool(require_video_file),
            "required_scan_time_profile": str(required_scan_time_profile or ""),
        },
        "blockers": blockers,
        "cases": cases,
    }


def _discover_reports(reports: Sequence[Path], patterns: Sequence[str]) -> list[Path]:
    discovered: list[Path] = []
    seen: set[Path] = set()

    def add(path: Path) -> None:
        resolved = path if path.is_absolute() else ROOT / path
        if resolved.is_file() and resolved not in seen:
            seen.add(resolved)
            discovered.append(resolved)

    for report in reports:
        add(report)
    for pattern in patterns:
        glob_pattern = pattern if Path(pattern).is_absolute() else str(ROOT / pattern)
        for item in glob.glob(glob_pattern, recursive=True):
            add(Path(item))
    return sorted(discovered, key=lambda path: str(path))


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Evaluate large-range loop closure from live Fast-LIO/PCT inspection reports.",
    )
    parser.add_argument("--report", type=Path, action="append", default=[])
    parser.add_argument("--report-glob", action="append", default=[])
    parser.add_argument("--min-path-length-m", type=float, default=20.0)
    parser.add_argument("--min-goal-span-m", type=float, default=4.0)
    parser.add_argument("--max-loop-closure-error-m", type=float, default=0.75)
    parser.add_argument("--max-fastlio-loop-closure-error-m", type=float, default=1.0)
    parser.add_argument("--max-loop-yaw-error-rad", type=float, default=0.5)
    parser.add_argument("--max-goal-start-error-m", type=float, default=1.0)
    parser.add_argument("--min-checkpoints", type=int, default=4)
    parser.add_argument("--allow-missing-video", action="store_true")
    parser.add_argument("--require-video-file", action="store_true")
    parser.add_argument("--required-scan-time-profile", default="physical_rolling")
    parser.add_argument("--json-out", type=Path, default=None)
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    summary = evaluate_reports(
        _discover_reports(args.report, args.report_glob),
        min_path_length_m=float(args.min_path_length_m),
        min_goal_span_m=float(args.min_goal_span_m),
        max_loop_closure_error_m=float(args.max_loop_closure_error_m),
        max_fastlio_loop_closure_error_m=float(args.max_fastlio_loop_closure_error_m),
        max_loop_yaw_error_rad=float(args.max_loop_yaw_error_rad),
        max_goal_start_error_m=float(args.max_goal_start_error_m),
        min_checkpoints=int(args.min_checkpoints),
        require_video=not bool(args.allow_missing_video),
        require_video_file=bool(args.require_video_file or args.strict),
        required_scan_time_profile=str(args.required_scan_time_profile),
    )
    text = json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True)
    print(text)
    if args.json_out:
        output = args.json_out if args.json_out.is_absolute() else ROOT / args.json_out
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(text + "\n", encoding="utf-8")
    return 0 if summary.get("ok") or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())

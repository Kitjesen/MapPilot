#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import subprocess
import time
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[2]
VALID_SPEED_BINS = ("slow", "medium", "fast")
VALID_DENSITY_BINS = ("sparse", "medium", "dense")
BLOCKING_SUBSYSTEM_PRIORITY = (
    "slam_localization",
    "planning_tracking",
    "dynamic_obstacle",
    "sensor_time_model",
    "video_evidence",
    "unknown",
)
SPEED_TARGET_MPS = {
    "slow": 0.35,
    "medium": 0.62,
    "fast": 0.90,
}
DENSITY_POINT_SPACING_M = {
    "sparse": 0.16,
    "medium": 0.10,
    "dense": 0.05,
}
DEFAULT_INSPECTION_GOALS = "0.5,0.05;1.0,0.1;1.5,0.15"
DEFAULT_INSPECTION_TOMOGRAM = "artifacts/server_sim_closure/large_terrain/tomogram.pickle"


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


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _moving_obstacles_enabled(moving_obstacles: dict[str, Any]) -> bool:
    mode = str(moving_obstacles.get("mode") or "none").strip().lower()
    return bool(moving_obstacles.get("enabled")) or mode != "none"


def _trail_margin(trail: dict[str, Any]) -> float | None:
    margin = trail.get("min_clearance_minus_robot_radius_m")
    if margin is None:
        margin = trail.get("min_margin_m")
    return _safe_float(margin)


def _extract_speed_mps(moving_obstacles: dict[str, Any]) -> float | None:
    speed_bounds = moving_obstacles.get("speed_bounds")
    if isinstance(speed_bounds, dict):
        for key in (
            "peak_planar_speed_bound_mps",
            "peak_planar_speed_mps",
            "peak_lateral_speed_mps",
        ):
            speed = _safe_float(speed_bounds.get(key))
            if speed is not None:
                return speed

    period_s = _safe_float(moving_obstacles.get("period_s"))
    lateral_amp_m = _safe_float(moving_obstacles.get("lateral_amplitude_m"), default=0.0)
    along_amp_m = _safe_float(moving_obstacles.get("along_amplitude_m"), default=0.0)
    if period_s and period_s > 0.0:
        return 2.0 * math.pi * math.hypot(lateral_amp_m or 0.0, along_amp_m or 0.0) / period_s
    return None


def _speed_bin(speed_mps: float | None) -> str | None:
    if speed_mps is None:
        return None
    if speed_mps < 0.5:
        return "slow"
    if speed_mps < 0.75:
        return "medium"
    return "fast"


def _density_metric(moving_obstacles: dict[str, Any]) -> tuple[str, float | None]:
    spacing_m = _safe_float(moving_obstacles.get("point_spacing_m"))
    if spacing_m is None:
        spacing_m = _safe_float(moving_obstacles.get("point_spacing"))
    if spacing_m is not None:
        return "point_spacing_m", spacing_m

    point_count = _safe_float(moving_obstacles.get("published_point_count_max"))
    obstacle_count = max(1, _safe_int(moving_obstacles.get("count"), default=1))
    if point_count is not None:
        return "points_per_obstacle", point_count / obstacle_count
    return "unknown", None


def _density_bin(metric_name: str, metric_value: float | None) -> str | None:
    if metric_value is None:
        return None
    if metric_name == "point_spacing_m":
        if metric_value >= 0.14:
            return "sparse"
        if metric_value >= 0.08:
            return "medium"
        return "dense"

    if metric_name == "points_per_obstacle":
        if metric_value < 48:
            return "sparse"
        if metric_value < 96:
            return "medium"
        return "dense"
    return None


def _video_evidence(report: dict[str, Any]) -> dict[str, Any]:
    video = report.get("video") if isinstance(report.get("video"), dict) else {}
    path = str(report.get("video_path") or video.get("path") or "")
    frame_count = _safe_int(report.get("video_frame_count", video.get("frames")))
    sample_count = _safe_int(report.get("video_sample_count", video.get("samples")))
    return {
        "path": path,
        "frame_count": frame_count,
        "sample_count": sample_count,
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


def _live_nav_chain_evidence(report: dict[str, Any]) -> dict[str, Any]:
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

    nav_data_source = str(report.get("nav_data_source") or "").strip().lower()
    mapping_input_path = str(report.get("true_mapping_input_path") or "")
    mapping_lower = mapping_input_path.lower()
    required_mapping_tokens = (
        "/lidar/raw_frame",
        "/imu/raw",
        "fastlio2",
        "/slam/odometry",
        "/slam/map_cloud",
    )

    if nav_data_source != "fastlio2":
        blockers.append("nav_data_source is not fastlio2")
    for token in required_mapping_tokens:
        if token not in mapping_lower:
            blockers.append(f"true_mapping_input_path missing {token}")
    if _safe_int(outputs.get("nav_odometry")) <= 0:
        blockers.append("outputs.nav_odometry missing")
    if _safe_int(outputs.get("nav_map_cloud")) <= 0:
        blockers.append("outputs.nav_map_cloud missing")
    if _safe_int(outputs.get("nav_cmd_vel_nonzero")) <= 0:
        blockers.append("outputs.nav_cmd_vel_nonzero missing")
    if inspection.get("enabled") is not True:
        blockers.append("lingtu_inspection.enabled is not true")
    if inspection.get("verified") is not True:
        blockers.append("lingtu_inspection.verified is not true")
    if str(inspection.get("global_planner") or "").strip().lower() != "pct":
        blockers.append("lingtu_inspection.global_planner is not pct")
    if _safe_int(inspection.get("global_path_count")) <= 0:
        blockers.append("lingtu_inspection.global_path_count missing")
    if _safe_int(inspection.get("global_path_points_max")) <= 1:
        blockers.append("lingtu_inspection.global_path_points_max below minimum")
    if _safe_int(inspection.get("local_path_count")) <= 0:
        blockers.append("lingtu_inspection.local_path_count missing")
    if _safe_int(inspection.get("local_path_points_max")) <= 1:
        blockers.append("lingtu_inspection.local_path_points_max below minimum")
    min_required_checkpoints = _safe_int(
        inspection.get("min_required_checkpoints"), default=1
    )
    successful_checkpoints = _safe_int(
        inspection.get("successful_navigation_goal_count")
    )
    if min_required_checkpoints > 0 and successful_checkpoints < min_required_checkpoints:
        blockers.append("lingtu_inspection.successful_navigation_goal_count below required")
    if inspection.get("replan_on_costmap_update") is not False:
        blockers.append("lingtu_inspection.replan_on_costmap_update is not false")
    if navigation_chain.get("planner_fallback_used") is True:
        blockers.append("navigation_chain.planner_fallback_used is true")
    if navigation_chain.get("planner_repair_used") is True:
        blockers.append("navigation_chain.planner_repair_used is true")

    return {
        "ok": not blockers,
        "nav_data_source": nav_data_source,
        "true_mapping_input_path": mapping_input_path,
        "outputs": dict(outputs),
        "nav_outputs": {
            "nav_odometry": _safe_int(outputs.get("nav_odometry")),
            "nav_map_cloud": _safe_int(outputs.get("nav_map_cloud")),
            "nav_cmd_vel_nonzero": _safe_int(outputs.get("nav_cmd_vel_nonzero")),
        },
        "lingtu_inspection_raw": dict(inspection),
        "lingtu_inspection": {
            "enabled": inspection.get("enabled"),
            "verified": inspection.get("verified"),
            "global_planner": inspection.get("global_planner"),
            "global_path_count": _safe_int(inspection.get("global_path_count")),
            "global_path_points_max": _safe_int(
                inspection.get("global_path_points_max")
            ),
            "local_path_count": _safe_int(inspection.get("local_path_count")),
            "local_path_points_max": _safe_int(inspection.get("local_path_points_max")),
            "successful_navigation_goal_count": successful_checkpoints,
            "min_required_checkpoints": min_required_checkpoints,
            "replan_on_costmap_update": inspection.get("replan_on_costmap_update"),
        },
        "navigation_chain": {
            "planner_fallback_used": navigation_chain.get("planner_fallback_used"),
            "planner_repair_used": navigation_chain.get("planner_repair_used"),
            "last_plan_report": navigation_chain.get("last_plan_report"),
            "fallback_reason": navigation_chain.get("fallback_reason"),
        },
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
        "world": report.get("world") or report.get("scene_xml") or "",
        "inspection_tomogram": inspection.get("tomogram")
        or report.get("inspection_tomogram")
        or report.get("tomogram")
        or "",
        "world_parent": report.get("world_parent") or "",
        "tomogram_parent": report.get("tomogram_parent") or "",
        "blockers": blockers,
    }


def _subcheck(report: dict[str, Any], key: str) -> dict[str, Any]:
    value = report.get(key)
    return value if isinstance(value, dict) else {}


def _fastlio_consistency_evidence(report: dict[str, Any]) -> dict[str, Any]:
    motion = _subcheck(report, "fastlio2_motion_consistency")
    z_check = _subcheck(report, "fastlio2_z_consistency")
    yaw_check = _subcheck(report, "fastlio2_yaw_consistency")
    return {
        "motion_checked": motion.get("checked"),
        "motion_ok": motion.get("ok"),
        "fastlio2_moved_m": _safe_float(
            motion.get("fastlio2_moved_m"),
            default=_safe_float(report.get("fastlio2_moved_m")),
        ),
        "sim_moved_m": _safe_float(
            motion.get("sim_moved_m"),
            default=_safe_float(report.get("sim_moved_m")),
        ),
        "moved_error_m": _safe_float(motion.get("moved_error_m")),
        "z_checked": z_check.get("checked"),
        "z_ok": z_check.get("ok"),
        "z_delta_error_m": _safe_float(
            z_check.get("z_delta_error_m"),
            default=_safe_float(report.get("fastlio2_z_delta_error_m")),
        ),
        "max_allowed_z_drift_m": _safe_float(z_check.get("max_allowed_z_drift_m")),
        "yaw_checked": yaw_check.get("checked"),
        "yaw_ok": yaw_check.get("ok"),
        "yaw_delta_error_rad": _safe_float(
            yaw_check.get("yaw_delta_error_rad"),
            default=_safe_float(report.get("fastlio2_yaw_delta_error_rad")),
        ),
        "max_allowed_yaw_drift_rad": _safe_float(
            yaw_check.get("max_allowed_yaw_drift_rad")
        ),
    }


def _case_blocking_subsystems(
    blockers: Sequence[str],
    fastlio: Mapping[str, Any],
) -> list[str]:
    systems: set[str] = set()
    joined = "\n".join(str(blocker) for blocker in blockers).lower()
    if (
        fastlio.get("motion_ok") is False
        or fastlio.get("z_ok") is False
        or fastlio.get("yaw_ok") is False
        or "fastlio" in joined
        or "nav_data_source is not fastlio2" in joined
    ):
        systems.add("slam_localization")
    if "lingtu_inspection" in joined or "global_planner" in joined:
        systems.add("planning_tracking")
    if "moving_obstacles" in joined or "trail_clearance" in joined:
        systems.add("dynamic_obstacle")
    if "video" in joined:
        systems.add("video_evidence")
    if "scan_time" in joined:
        systems.add("sensor_time_model")
    if not systems and blockers:
        systems.add("unknown")
    ordered = [system for system in BLOCKING_SUBSYSTEM_PRIORITY if system in systems]
    ordered.extend(sorted(systems - set(ordered)))
    return ordered


def _case_from_report(
    path: Path,
    report: dict[str, Any],
    *,
    min_clearance_m: float,
    require_video: bool,
    require_video_file: bool,
    require_live_nav_chain: bool,
    required_scan_time_profile: str,
) -> dict[str, Any]:
    blockers: list[str] = []
    moving_obstacles = (
        report.get("moving_obstacles")
        if isinstance(report.get("moving_obstacles"), dict)
        else {}
    )
    trail = (
        moving_obstacles.get("trail_clearance")
        if isinstance(moving_obstacles.get("trail_clearance"), dict)
        else {}
    )
    margin = _trail_margin(trail)
    speed_mps = _extract_speed_mps(moving_obstacles)
    speed_label = _speed_bin(speed_mps)
    density_name, density_value = _density_metric(moving_obstacles)
    density_label = _density_bin(density_name, density_value)
    video = _video_evidence(report)
    scan_time = _scan_time_evidence(report)
    live_nav_chain = _live_nav_chain_evidence(report)
    fastlio_consistency = _fastlio_consistency_evidence(report)
    required_scan_time_profile = str(required_scan_time_profile or "").strip()

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
    if not _moving_obstacles_enabled(moving_obstacles):
        blockers.append("moving_obstacles are not enabled")
    if moving_obstacles.get("ok") is not True:
        blockers.append("moving_obstacles.ok is not true")
    if _safe_int(moving_obstacles.get("published_update_count")) <= 0:
        blockers.append("moving_obstacles.published_update_count missing")
    if _safe_int(moving_obstacles.get("published_point_count_max")) <= 0:
        blockers.append("moving_obstacles.published_point_count_max missing")
    if trail.get("checked") is not True:
        blockers.append("moving_obstacles.trail_clearance.checked is not true")
    if trail.get("collision") is True:
        blockers.append("moving_obstacles.trail_clearance.collision is true")
    if margin is None or margin < min_clearance_m:
        blockers.append(f"moving_obstacles trail clearance margin < {min_clearance_m:g}")
    if speed_label is None:
        blockers.append("moving_obstacles speed is not measurable")
    if density_label is None:
        blockers.append("moving_obstacles density is not measurable")
    if require_video:
        if not video["path"]:
            blockers.append("video_path missing")
        if int(video["frame_count"]) <= 0:
            blockers.append("video_frame_count missing")
        if int(video["sample_count"]) <= 0:
            blockers.append("video_sample_count missing")
    if require_video_file and video["path"] and not Path(str(video["path"])).is_file():
        blockers.append("video file missing")
    if require_live_nav_chain and live_nav_chain.get("ok") is not True:
        blockers.extend(
            f"live nav chain: {blocker}"
            for blocker in list(live_nav_chain.get("blockers") or [])
        )

    pair = f"{speed_label}:{density_label}" if speed_label and density_label else None
    blocking_subsystems = _case_blocking_subsystems(blockers, fastlio_consistency)
    return {
        "path": str(path),
        "ok": not blockers,
        "pair": pair,
        "speed_bin": speed_label,
        "density_bin": density_label,
        "speed_mps": speed_mps,
        "scan_time_profile": scan_time["profile"],
        "scan_time_model_contract": scan_time["contract"],
        "density_metric": density_name,
        "density_value": density_value,
        "moving_obstacle_count": moving_obstacles.get("count"),
        "published_update_count": moving_obstacles.get("published_update_count"),
        "published_point_count_max": moving_obstacles.get("published_point_count_max"),
        "trail_min_margin_m": margin,
        "video": video,
        "live_nav_chain": live_nav_chain,
        "fastlio2_consistency": fastlio_consistency,
        "blocking_subsystems": blocking_subsystems,
        "blockers": blockers,
    }


def _ordered_unique(values: Iterable[str], reference_order: Sequence[str]) -> list[str]:
    value_set = set(values)
    ordered = [value for value in reference_order if value in value_set]
    ordered.extend(sorted(value_set - set(reference_order)))
    return ordered


def _required_pairs(speed_bins: Sequence[str], density_bins: Sequence[str]) -> list[str]:
    return [f"{speed}:{density}" for speed in speed_bins for density in density_bins]


def _validate_bins(values: Sequence[str], valid_values: Sequence[str], label: str) -> list[str]:
    invalid = [value for value in values if value not in valid_values]
    return [f"unknown {label} bin: {value}" for value in invalid]


def _format_float(value: float) -> str:
    return f"{float(value):.6g}"


def _case_ros_domain_id(
    base_ros_domain_id: str | int | None,
    case_index: int,
    *,
    base_env: Mapping[str, str] | None = None,
) -> str:
    env = os.environ if base_env is None else base_env
    base_value = (
        base_ros_domain_id
        if base_ros_domain_id is not None and str(base_ros_domain_id)
        else env.get("LINGTU_MUJOCO_LIVE_ROS_DOMAIN_ID") or env.get("ROS_DOMAIN_ID") or "83"
    )
    try:
        base = int(str(base_value))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"ROS_DOMAIN_ID must be numeric, got {base_value!r}") from exc
    if base <= 0 or base >= 232:
        raise ValueError(f"ROS_DOMAIN_ID must be in 1..231, got {base}")
    return str(((base - 1 + int(case_index)) % 231) + 1)


def sweep_matrix_cases(
    *,
    required_speed_bins: Sequence[str] = ("slow", "fast"),
    required_density_bins: Sequence[str] = ("sparse", "dense"),
    moving_obstacle_count: int = 3,
    lateral_amplitude_m: float = 0.85,
    along_amplitude_m: float = 0.25,
) -> list[dict[str, Any]]:
    errors = [
        *_validate_bins(required_speed_bins, VALID_SPEED_BINS, "speed"),
        *_validate_bins(required_density_bins, VALID_DENSITY_BINS, "density"),
    ]
    if errors:
        raise ValueError("; ".join(errors))
    amplitude_norm = math.hypot(float(lateral_amplitude_m), float(along_amplitude_m))
    if amplitude_norm <= 0.0:
        raise ValueError("moving obstacle amplitude must be positive")

    cases: list[dict[str, Any]] = []
    for speed_bin in required_speed_bins:
        speed_target_mps = SPEED_TARGET_MPS[str(speed_bin)]
        period_s = 2.0 * math.pi * amplitude_norm / speed_target_mps
        for density_bin in required_density_bins:
            point_spacing_m = DENSITY_POINT_SPACING_M[str(density_bin)]
            cases.append(
                {
                    "case_id": f"{speed_bin}-{density_bin}",
                    "expected_speed_bin": str(speed_bin),
                    "expected_density_bin": str(density_bin),
                    "speed_target_mps": float(speed_target_mps),
                    "period_s": float(period_s),
                    "point_spacing_m": float(point_spacing_m),
                    "moving_obstacle_count": int(moving_obstacle_count),
                    "lateral_amplitude_m": float(lateral_amplitude_m),
                    "along_amplitude_m": float(along_amplitude_m),
                }
            )
    return cases


def build_case_environment(
    case: Mapping[str, Any],
    *,
    child_run_root: Path,
    duration_s: float,
    inspection_goals: str,
    inspection_tomogram: Path | str | None = DEFAULT_INSPECTION_TOMOGRAM,
    world: str | None = "industrial_park",
    ros_domain_id: str | int | None = None,
    base_env: Mapping[str, str] | None = None,
) -> dict[str, str]:
    env = dict(os.environ if base_env is None else base_env)
    case_run_root = Path(child_run_root) / str(case["case_id"])
    env.update(
        {
            "LINGTU_MUJOCO_LIVE_RUN_DIR": str(case_run_root),
            "LINGTU_MUJOCO_LIVE_DURATION_INSPECTION": _format_float(float(duration_s)),
            "LINGTU_MUJOCO_LIVE_DURATION_CLOCK": "sim",
            "LINGTU_MUJOCO_LIVE_SCAN_TIME_PROFILE": "physical_rolling",
            "LINGTU_MUJOCO_LIVE_INSPECTION_PLANNER": "pct",
            "LINGTU_MUJOCO_LIVE_INSPECTION_GOALS": str(inspection_goals),
            "LINGTU_MUJOCO_LIVE_INSPECTION_MIN_CHECKPOINTS": "3",
            "LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_MODE": "robot_crossing",
            "LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_COUNT": str(
                int(case.get("moving_obstacle_count", 3))
            ),
            "LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_START_S": "2",
            "LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_PERIOD_S": _format_float(
                float(case["period_s"])
            ),
            "LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_LATERAL_AMPLITUDE_M": _format_float(
                float(case["lateral_amplitude_m"])
            ),
            "LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_ALONG_AMPLITUDE_M": _format_float(
                float(case["along_amplitude_m"])
            ),
            "LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_POINT_SPACING": _format_float(
                float(case["point_spacing_m"])
            ),
            "LINGTU_MUJOCO_LIVE_STRICT": "1",
            "LINGTU_MUJOCO_LIVE_SAVE_MAP_ARTIFACTS": "1",
            "LINGTU_MUJOCO_LIVE_BUILD_TOMOGRAM": "1",
            "LINGTU_MUJOCO_LIVE_FASTLIO_IESKF_MAX_ITER": "10",
        }
    )
    if inspection_tomogram:
        tomogram_path = Path(inspection_tomogram)
        env["LINGTU_MUJOCO_LIVE_INSPECTION_TOMOGRAM"] = (
            tomogram_path.as_posix() if not tomogram_path.is_absolute() else str(tomogram_path)
        )
    if world:
        env["LINGTU_MUJOCO_LIVE_WORLD"] = str(world)
    if ros_domain_id is not None and str(ros_domain_id):
        env["ROS_DOMAIN_ID"] = str(ros_domain_id)
        env["LINGTU_MUJOCO_LIVE_ROS_DOMAIN_ID"] = str(ros_domain_id)
    return env


def _latest_report(run_root: Path) -> Path | None:
    latest = run_root / "latest.txt"
    if latest.is_file():
        for line in latest.read_text(encoding="utf-8").splitlines():
            if line.startswith("latest_run_dir="):
                run_dir = Path(line.split("=", 1)[1].strip())
                report = run_dir / "report.json"
                if report.is_file():
                    return report
    candidates = [path for path in run_root.glob("**/report.json") if path.is_file()]
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def _rooted_path(path: Path | str) -> Path:
    candidate = Path(path)
    return candidate if candidate.is_absolute() else ROOT / candidate


def _path_like(value: Path | str | None) -> bool:
    if value is None:
        return False
    text = str(value).strip()
    if not text:
        return False
    return (
        "/" in text
        or "\\" in text
        or text.startswith(".")
        or bool(Path(text).suffix)
    )


def run_matrix_preflight(
    *,
    launch_script: Path,
    world: Path | str | None,
    inspection_tomogram: Path | str | None,
) -> dict[str, Any]:
    environment_blockers: list[str] = []
    artifact_blockers: list[str] = []
    warnings: list[str] = []

    launch_path = _rooted_path(launch_script)
    launch_exists = launch_path.is_file()
    if not launch_exists:
        environment_blockers.append(f"launch_script missing: {launch_path}")

    world_path: Path | None = None
    world_exists: bool | None = None
    if _path_like(world):
        world_path = _rooted_path(str(world))
        world_exists = world_path.is_file()
        if not world_exists:
            artifact_blockers.append(f"world file missing: {world_path}")

    tomogram_path: Path | None = None
    tomogram_exists: bool | None = None
    if inspection_tomogram:
        tomogram_path = _rooted_path(str(inspection_tomogram))
        tomogram_exists = tomogram_path.is_file()
        if not tomogram_exists:
            artifact_blockers.append(f"inspection_tomogram missing: {tomogram_path}")

    metadata_path: Path | None = None
    if world_path is not None and tomogram_path is not None:
        if world_exists and tomogram_exists:
            if world_path.parent != tomogram_path.parent:
                artifact_blockers.append(
                    "world/tomogram are not from the same source directory: "
                    f"{world_path.parent} != {tomogram_path.parent}"
                )
            metadata_path = world_path.parent / "metadata.json"
            if not metadata_path.is_file():
                warnings.append(f"same-source metadata.json missing: {metadata_path}")

    blockers = [*environment_blockers, *artifact_blockers]
    blocking_subsystems = []
    if environment_blockers:
        blocking_subsystems.append("environment_runtime")
    if artifact_blockers:
        blocking_subsystems.append("artifact_contract")

    return {
        "ok": not blockers,
        "launch_script": {
            "path": str(launch_path),
            "exists": launch_exists,
        },
        "world": {
            "value": "" if world is None else str(world),
            "path": "" if world_path is None else str(world_path),
            "exists": world_exists,
            "path_checked": world_path is not None,
        },
        "inspection_tomogram": {
            "value": "" if inspection_tomogram is None else str(inspection_tomogram),
            "path": "" if tomogram_path is None else str(tomogram_path),
            "exists": tomogram_exists,
            "path_checked": tomogram_path is not None,
        },
        "same_source": {
            "checked": world_path is not None and tomogram_path is not None,
            "metadata_path": "" if metadata_path is None else str(metadata_path),
            "metadata_exists": None if metadata_path is None else metadata_path.is_file(),
        },
        "environment_blockers": environment_blockers,
        "artifact_blockers": artifact_blockers,
        "blocking_subsystems": blocking_subsystems,
        "blockers": blockers,
        "warnings": warnings,
        "next_action_hint": (
            "fix run-matrix preflight blockers before launching the long "
            "moving-obstacle sweep"
        ),
    }


def preflight_failure_summary(
    *,
    preflight: Mapping[str, Any],
    required_speed_bins: Sequence[str],
    required_density_bins: Sequence[str],
    min_clearance_m: float,
    require_video: bool,
    require_video_file: bool,
    require_live_nav_chain: bool,
    required_scan_time_profile: str,
) -> dict[str, Any]:
    required_pairs = _required_pairs(required_speed_bins, required_density_bins)
    blockers = [str(item) for item in preflight.get("blockers") or []]
    return {
        "schema_version": "lingtu.moving_obstacle_sweep_gate.v1",
        "ok": False,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "case_count": 0,
        "passed_case_count": 0,
        "failed_case_count": 0,
        "passed_pair_count": 0,
        "required_speed_bins": list(required_speed_bins),
        "required_density_bins": list(required_density_bins),
        "covered_speed_bins": [],
        "covered_density_bins": [],
        "required_pairs": required_pairs,
        "covered_pairs": [],
        "missing_speed_bins": list(required_speed_bins),
        "missing_density_bins": list(required_density_bins),
        "missing_pairs": required_pairs,
        "min_clearance_m": float(min_clearance_m),
        "require_video": bool(require_video),
        "require_video_file": bool(require_video_file),
        "required_live_nav_chain": bool(require_live_nav_chain),
        "required_scan_time_profile": str(required_scan_time_profile or ""),
        "minimal_red_defect": {
            "blocking_subsystem": next(
                iter(preflight.get("blocking_subsystems") or []),
                "environment_runtime",
            ),
            "blockers": blockers,
        },
        "blocking_subsystems": list(preflight.get("blocking_subsystems") or []),
        "environment_blockers": list(preflight.get("environment_blockers") or []),
        "artifact_blockers": list(preflight.get("artifact_blockers") or []),
        "preflight": dict(preflight),
        "next_action_hint": str(preflight.get("next_action_hint") or ""),
        "blockers": blockers,
        "cases": [],
        "run_matrix": {
            "enabled": True,
            "preflight": dict(preflight),
            "case_count": 0,
            "cases": [],
            "blockers": blockers,
            "warnings": list(preflight.get("warnings") or []),
        },
    }


Runner = Callable[..., subprocess.CompletedProcess[Any]]


def run_matrix_cases(
    cases: Sequence[Mapping[str, Any]],
    *,
    child_run_root: Path,
    launch_script: Path = Path("sim/scripts/mujoco/launch_fastlio2_live.sh"),
    case_timeout_s: float = 900.0,
    duration_s: float = 120.0,
    inspection_goals: str = DEFAULT_INSPECTION_GOALS,
    inspection_tomogram: Path | str | None = DEFAULT_INSPECTION_TOMOGRAM,
    world: str | None = "industrial_park",
    ros_domain_id: str | int | None = None,
    runner: Runner = subprocess.run,
) -> list[dict[str, Any]]:
    results: list[dict[str, Any]] = []
    script = Path(launch_script)
    command_script = str(script if script.is_absolute() else script.as_posix())
    for case_index, case in enumerate(cases):
        case_started = time.time()
        case_root = Path(child_run_root) / str(case["case_id"])
        case_root.mkdir(parents=True, exist_ok=True)
        case_ros_domain_id = _case_ros_domain_id(ros_domain_id, case_index)
        env = build_case_environment(
            case,
            child_run_root=Path(child_run_root),
            duration_s=duration_s,
            inspection_goals=inspection_goals,
            inspection_tomogram=inspection_tomogram,
            world=world,
            ros_domain_id=case_ros_domain_id,
        )
        command = ["bash", command_script, "inspection-moving-obstacle-video"]
        result: subprocess.CompletedProcess[Any] | None = None
        error = ""
        try:
            result = runner(
                command,
                cwd=ROOT,
                env=env,
                timeout=float(case_timeout_s),
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            error = f"timeout after {float(case_timeout_s):g}s: {exc}"
        except Exception as exc:  # pragma: no cover - CLI diagnostics path.
            error = f"{type(exc).__name__}: {exc}"

        report = _latest_report(case_root)
        results.append(
            {
                "case_id": str(case["case_id"]),
                "expected_speed_bin": case.get("expected_speed_bin"),
                "expected_density_bin": case.get("expected_density_bin"),
                "command": command,
                "run_root": str(case_root),
                "ros_domain_id": case_ros_domain_id,
                "returncode": None if result is None else int(result.returncode),
                "report_path": "" if report is None else str(report),
                "elapsed_s": round(time.time() - case_started, 3),
                "error": error,
            }
        )
    return results


def evaluate_reports(
    report_paths: Sequence[Path | str],
    *,
    required_speed_bins: Sequence[str] = ("slow", "fast"),
    required_density_bins: Sequence[str] = ("sparse", "dense"),
    min_clearance_m: float = 0.25,
    require_video: bool = True,
    require_video_file: bool = False,
    require_live_nav_chain: bool = True,
    required_scan_time_profile: str = "physical_rolling",
) -> dict[str, Any]:
    paths = [Path(path) for path in report_paths]
    required_speed_bins = tuple(required_speed_bins)
    required_density_bins = tuple(required_density_bins)
    required_pairs = _required_pairs(required_speed_bins, required_density_bins)
    required_pair_set = set(required_pairs)
    blockers: list[str] = []
    blockers.extend(_validate_bins(required_speed_bins, VALID_SPEED_BINS, "speed"))
    blockers.extend(_validate_bins(required_density_bins, VALID_DENSITY_BINS, "density"))

    cases: list[dict[str, Any]] = []
    for path in paths:
        try:
            report = _load_json(path)
            cases.append(
                _case_from_report(
                    path,
                    report,
                    min_clearance_m=min_clearance_m,
                    require_video=require_video,
                    require_video_file=require_video_file,
                    require_live_nav_chain=require_live_nav_chain,
                    required_scan_time_profile=required_scan_time_profile,
                )
            )
        except Exception as exc:  # pragma: no cover - keeps CLI diagnostics usable.
            cases.append(
                {
                    "path": str(path),
                    "ok": False,
                    "pair": None,
                    "speed_bin": None,
                    "density_bin": None,
                    "blockers": [f"failed to load report: {exc}"],
                }
            )

    passed_cases = [case for case in cases if case.get("ok") is True]
    covered_speed_bins = _ordered_unique(
        (str(case["speed_bin"]) for case in passed_cases if case.get("speed_bin")),
        VALID_SPEED_BINS,
    )
    covered_density_bins = _ordered_unique(
        (str(case["density_bin"]) for case in passed_cases if case.get("density_bin")),
        VALID_DENSITY_BINS,
    )
    covered_pair_set = {
        str(case["pair"]) for case in passed_cases if isinstance(case.get("pair"), str)
    }
    covered_pairs = [pair for pair in required_pairs if pair in covered_pair_set]
    covered_pairs.extend(sorted(covered_pair_set - required_pair_set))
    missing_speed_bins = [
        speed for speed in required_speed_bins if speed not in covered_speed_bins
    ]
    missing_density_bins = [
        density for density in required_density_bins if density not in covered_density_bins
    ]
    missing_pairs = [pair for pair in required_pairs if pair not in covered_pair_set]

    if not paths:
        blockers.append("no moving obstacle reports provided")
    if missing_speed_bins:
        blockers.append(f"missing required speed bins: {','.join(missing_speed_bins)}")
    if missing_density_bins:
        blockers.append(f"missing required density bins: {','.join(missing_density_bins)}")
    if missing_pairs:
        blockers.append(f"missing required speed/density pairs: {','.join(missing_pairs)}")

    failed_cases = [case for case in cases if case.get("ok") is not True]
    minimal_red_defect: dict[str, Any] = {}
    if failed_cases:
        defect_case = sorted(
            failed_cases,
            key=lambda case: (
                0
                if "slam_localization"
                in set(str(item) for item in case.get("blocking_subsystems") or [])
                else 1,
                -float(_case_local_path_count(case)),
            ),
        )[0]
        fastlio = (
            defect_case.get("fastlio2_consistency")
            if isinstance(defect_case.get("fastlio2_consistency"), dict)
            else {}
        )
        subsystems = list(defect_case.get("blocking_subsystems") or [])
        minimal_red_defect = {
            "blocking_subsystem": subsystems[0] if subsystems else "unknown",
            "path": defect_case.get("path"),
            "pair": defect_case.get("pair"),
            "blockers": list(defect_case.get("blockers") or []),
            "fastlio2_moved_m": fastlio.get("fastlio2_moved_m"),
            "sim_moved_m": fastlio.get("sim_moved_m"),
            "fastlio2_z_delta_error_m": fastlio.get("z_delta_error_m"),
            "fastlio2_yaw_delta_error_rad": fastlio.get("yaw_delta_error_rad"),
            "live_nav_chain": defect_case.get("live_nav_chain"),
        }
    blocking_subsystems = sorted(
        {
            str(system)
            for case in failed_cases
            for system in list(case.get("blocking_subsystems") or [])
            if str(system)
        }
    )

    return {
        "schema_version": "lingtu.moving_obstacle_sweep_gate.v1",
        "ok": not blockers,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "case_count": len(cases),
        "passed_case_count": len(passed_cases),
        "failed_case_count": len(cases) - len(passed_cases),
        "passed_pair_count": len(required_pair_set & covered_pair_set),
        "required_speed_bins": list(required_speed_bins),
        "required_density_bins": list(required_density_bins),
        "covered_speed_bins": covered_speed_bins,
        "covered_density_bins": covered_density_bins,
        "required_pairs": required_pairs,
        "covered_pairs": covered_pairs,
        "missing_speed_bins": missing_speed_bins,
        "missing_density_bins": missing_density_bins,
        "missing_pairs": missing_pairs,
        "min_clearance_m": float(min_clearance_m),
        "require_video": bool(require_video),
        "require_video_file": bool(require_video_file),
        "required_live_nav_chain": bool(require_live_nav_chain),
        "required_scan_time_profile": str(required_scan_time_profile or ""),
        "minimal_red_defect": minimal_red_defect,
        "blocking_subsystems": blocking_subsystems,
        "blockers": blockers,
        "cases": cases,
    }


def _report_path_keys(path_text: Any) -> set[str]:
    if not str(path_text or ""):
        return set()
    path = Path(str(path_text))
    keys = {str(path), path.as_posix()}
    try:
        resolved = path if path.is_absolute() else ROOT / path
        keys.add(str(resolved.resolve()))
        keys.add(resolved.resolve().as_posix())
    except OSError:
        pass
    return keys


def matrix_run_diagnostics(
    run_results: Sequence[Mapping[str, Any]],
    evaluated_cases: Sequence[Mapping[str, Any]],
) -> tuple[list[str], list[str]]:
    passed_report_keys: set[str] = set()
    for case in evaluated_cases:
        if case.get("ok") is True:
            passed_report_keys.update(_report_path_keys(case.get("path")))

    blockers: list[str] = []
    warnings: list[str] = []
    for result in run_results:
        case_id = str(result.get("case_id") or "")
        report_path = str(result.get("report_path") or "")
        report_passed = bool(_report_path_keys(report_path) & passed_report_keys)
        error = str(result.get("error") or "")
        returncode = result.get("returncode")
        if not report_path:
            blockers.append(f"{case_id} did not produce a report")
        if returncode not in (0, None) and not report_passed:
            blockers.append(
                f"{case_id} returncode={returncode}: {error or 'child run failed'}"
            )
        if error:
            if report_passed:
                warnings.append(f"{case_id}: {error}; accepted because the child report passed")
            else:
                blockers.append(f"{case_id}: {error}")
    return blockers, warnings


def _case_local_path_count(case: Mapping[str, Any]) -> int:
    live_nav_chain = (
        case.get("live_nav_chain") if isinstance(case.get("live_nav_chain"), dict) else {}
    )
    inspection = (
        live_nav_chain.get("lingtu_inspection")
        if isinstance(live_nav_chain.get("lingtu_inspection"), dict)
        else {}
    )
    return _safe_int(inspection.get("local_path_count"))


def _parse_csv(value: str) -> tuple[str, ...]:
    return tuple(item.strip() for item in value.split(",") if item.strip())


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
        for path_text in glob.glob(glob_pattern, recursive=True):
            add(Path(path_text))
    return sorted(discovered, key=lambda path: str(path))


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Aggregate moving-obstacle child reports into a speed/density sweep gate.",
    )
    parser.add_argument("--report", type=Path, action="append", default=[])
    parser.add_argument("--report-glob", action="append", default=[])
    parser.add_argument(
        "--run-matrix",
        action="store_true",
        help="Run inspection-moving-obstacle-video once per required speed/density bin before aggregating.",
    )
    parser.add_argument(
        "--include-existing-reports",
        action="store_true",
        help=(
            "With --run-matrix, also evaluate --report/--report-glob inputs. "
            "By default, matrix runs evaluate only the freshly generated child reports."
        ),
    )
    parser.add_argument(
        "--child-run-root",
        type=Path,
        default=ROOT / "artifacts/server_sim_closure/moving_obstacle_sweep/children",
    )
    parser.add_argument(
        "--launch-script",
        type=Path,
        default=ROOT / "sim/scripts/mujoco/launch_fastlio2_live.sh",
    )
    parser.add_argument("--case-timeout-s", type=float, default=900.0)
    parser.add_argument("--duration-s", type=float, default=120.0)
    parser.add_argument("--inspection-goals", default=DEFAULT_INSPECTION_GOALS)
    parser.add_argument("--inspection-tomogram", default=DEFAULT_INSPECTION_TOMOGRAM)
    parser.add_argument("--world", default="industrial_park")
    parser.add_argument("--ros-domain-id", default=os.environ.get("ROS_DOMAIN_ID", ""))
    parser.add_argument("--moving-obstacle-count", type=int, default=3)
    parser.add_argument("--moving-obstacle-lateral-amplitude-m", type=float, default=0.85)
    parser.add_argument("--moving-obstacle-along-amplitude-m", type=float, default=0.25)
    parser.add_argument("--required-speed-bins", default="slow,fast")
    parser.add_argument("--required-density-bins", default="sparse,dense")
    parser.add_argument("--min-clearance-m", type=float, default=0.25)
    parser.add_argument("--allow-missing-video", action="store_true")
    parser.add_argument("--require-video-file", action="store_true")
    parser.add_argument("--required-scan-time-profile", default="physical_rolling")
    parser.add_argument(
        "--allow-weak-nav-chain",
        action="store_true",
        help="Do not require each child report to prove Fast-LIO/PCT/local-path/cmd_vel wiring.",
    )
    parser.add_argument("--json-out", type=Path, default=None)
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    required_speed_bins = _parse_csv(args.required_speed_bins)
    required_density_bins = _parse_csv(args.required_density_bins)
    discovered_report_paths = _discover_reports(args.report, args.report_glob)
    report_paths = list(discovered_report_paths)
    run_results: list[dict[str, Any]] = []
    if args.run_matrix:
        preflight = run_matrix_preflight(
            launch_script=args.launch_script,
            world=str(args.world) if args.world else None,
            inspection_tomogram=str(args.inspection_tomogram)
            if args.inspection_tomogram
            else None,
        )
        if preflight.get("ok") is not True:
            summary = preflight_failure_summary(
                preflight=preflight,
                required_speed_bins=required_speed_bins,
                required_density_bins=required_density_bins,
                min_clearance_m=float(args.min_clearance_m),
                require_video=not bool(args.allow_missing_video),
                require_video_file=bool(args.require_video_file or args.strict),
                require_live_nav_chain=not bool(args.allow_weak_nav_chain),
                required_scan_time_profile=str(args.required_scan_time_profile),
            )
            text = json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True)
            print(text)
            if args.json_out:
                output = args.json_out if args.json_out.is_absolute() else ROOT / args.json_out
                output.parent.mkdir(parents=True, exist_ok=True)
                output.write_text(text + "\n", encoding="utf-8")
            return 0 if not args.strict else 1
        matrix_cases = sweep_matrix_cases(
            required_speed_bins=required_speed_bins,
            required_density_bins=required_density_bins,
            moving_obstacle_count=int(args.moving_obstacle_count),
            lateral_amplitude_m=float(args.moving_obstacle_lateral_amplitude_m),
            along_amplitude_m=float(args.moving_obstacle_along_amplitude_m),
        )
        run_results = run_matrix_cases(
            matrix_cases,
            child_run_root=args.child_run_root,
            launch_script=args.launch_script,
            case_timeout_s=float(args.case_timeout_s),
            duration_s=float(args.duration_s),
            inspection_goals=str(args.inspection_goals),
            inspection_tomogram=str(args.inspection_tomogram) if args.inspection_tomogram else None,
            world=str(args.world) if args.world else None,
            ros_domain_id=str(args.ros_domain_id) if args.ros_domain_id else None,
        )
        generated_reports = [
            Path(result["report_path"])
            for result in run_results
            if str(result.get("report_path") or "")
        ]
        report_paths = (
            [*report_paths, *generated_reports]
            if args.include_existing_reports
            else generated_reports
        )

    unique_report_paths: list[Path] = []
    seen: set[Path] = set()
    for path in report_paths:
        if path not in seen:
            seen.add(path)
            unique_report_paths.append(path)
    summary = evaluate_reports(
        unique_report_paths,
        required_speed_bins=required_speed_bins,
        required_density_bins=required_density_bins,
        min_clearance_m=float(args.min_clearance_m),
        require_video=not bool(args.allow_missing_video),
        require_video_file=bool(args.require_video_file or args.strict),
        require_live_nav_chain=not bool(args.allow_weak_nav_chain),
        required_scan_time_profile=str(args.required_scan_time_profile),
    )
    if run_results:
        matrix_blockers, matrix_warnings = matrix_run_diagnostics(
            run_results,
            list(summary.get("cases") or []),
        )
        summary["run_matrix"] = {
            "enabled": True,
            "child_run_root": str(args.child_run_root),
            "case_count": len(run_results),
            "cases": run_results,
            "included_existing_report_count": (
                len(discovered_report_paths)
                if args.include_existing_reports
                else 0
            ),
            "blockers": matrix_blockers,
            "warnings": matrix_warnings,
        }
        if matrix_blockers:
            summary["blockers"] = [*list(summary.get("blockers") or []), *matrix_blockers]
            summary["ok"] = False
    text = json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True)
    print(text)
    if args.json_out:
        output = args.json_out if args.json_out.is_absolute() else ROOT / args.json_out
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(text + "\n", encoding="utf-8")
    return 0 if summary.get("ok") or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())

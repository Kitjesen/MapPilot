"""Diagnostic summarizers for the MuJoCo live gate."""

from __future__ import annotations

import math
import re
from pathlib import Path
from typing import Any, Sequence

from runtime.msgs.numpy_compat import is_numpy_array, np
from sim.scripts.mujoco_live.sensors import _angle_delta_rad

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

def _round_float(value: Any, digits: int = 4) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return round(result, digits)

def _safe_int(value: Any) -> int:
    try:
        return int(value or 0)
    except (TypeError, ValueError):
        return 0

def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, dict) else {}

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
    if is_numpy_array(position):
        try:
            flat = np.asarray(position, dtype=np.float64).reshape(-1)
            if flat.shape[0] < 2:
                return None
            z_value = flat[2] if flat.shape[0] >= 3 else 0.0
            return [float(flat[0]), float(flat[1]), float(z_value)]
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

def _sim_pose_sample_from_xyz_yaw(
    xyz: list[float] | tuple[float, float, float] | None,
    yaw: float | None,
) -> dict[str, Any] | None:
    if xyz is None:
        return None
    return {
        "sim_time_s": None,
        "xyz": [float(xyz[0]), float(xyz[1]), float(xyz[2])],
        "yaw": float(yaw) if yaw is not None else None,
        "dt_s": None,
    }

def _aligned_motion_window(
    samples: list[tuple[float, float, float, float, float]],
    *,
    ros_time_origin_s: float,
    first_odom_stamp_s: float | None,
    last_odom_stamp_s: float | None,
    fallback_first_sim_xyz: list[float] | None,
    fallback_last_sim_xyz: list[float] | None,
    fallback_first_sim_yaw: float | None,
    fallback_last_sim_yaw: float | None,
    max_dt_s: float = 0.25,
) -> dict[str, Any]:
    first_target_sim_time_s = (
        float(first_odom_stamp_s) - float(ros_time_origin_s)
        if first_odom_stamp_s is not None
        else None
    )
    last_target_sim_time_s = (
        float(last_odom_stamp_s) - float(ros_time_origin_s)
        if last_odom_stamp_s is not None
        else None
    )
    first_aligned = (
        _nearest_sim_pose_sample(
            samples,
            target_sim_time_s=float(first_target_sim_time_s),
            max_dt_s=max_dt_s,
        )
        if first_target_sim_time_s is not None
        else None
    )
    last_aligned = (
        _nearest_sim_pose_sample(
            samples,
            target_sim_time_s=float(last_target_sim_time_s),
            max_dt_s=max_dt_s,
        )
        if last_target_sim_time_s is not None
        else None
    )
    fallback_first = _sim_pose_sample_from_xyz_yaw(
        fallback_first_sim_xyz,
        fallback_first_sim_yaw,
    )
    fallback_last = _sim_pose_sample_from_xyz_yaw(
        fallback_last_sim_xyz,
        fallback_last_sim_yaw,
    )
    first_sample = first_aligned or fallback_first
    last_sample = last_aligned or fallback_last
    time_aligned = bool(first_aligned and last_aligned)
    return {
        "time_aligned": time_aligned,
        "first_source": (
            "fastlio2_stamp_aligned" if first_aligned else "gate_first_sim_pose"
        ),
        "last_source": (
            "fastlio2_stamp_aligned" if last_aligned else "gate_last_sim_pose"
        ),
        "max_alignment_dt_s": float(max_dt_s),
        "sample_count": len(samples),
        "first_fastlio2_stamp_s": _round_float(first_odom_stamp_s),
        "last_fastlio2_stamp_s": _round_float(last_odom_stamp_s),
        "first_target_sim_time_s": _round_float(first_target_sim_time_s),
        "last_target_sim_time_s": _round_float(last_target_sim_time_s),
        "first_sim_time_s": _round_float(first_sample.get("sim_time_s") if first_sample else None),
        "last_sim_time_s": _round_float(last_sample.get("sim_time_s") if last_sample else None),
        "first_dt_s": _round_float(first_sample.get("dt_s") if first_sample else None),
        "last_dt_s": _round_float(last_sample.get("dt_s") if last_sample else None),
        "first_sim_xyz": first_sample.get("xyz") if first_sample else None,
        "last_sim_xyz": last_sample.get("xyz") if last_sample else None,
        "first_sim_yaw_rad": first_sample.get("yaw") if first_sample else None,
        "last_sim_yaw_rad": last_sample.get("yaw") if last_sample else None,
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
    local_planner_health: dict[str, Any] | None = None,
    path_follower_health: dict[str, Any] | None = None,
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
    local_planner = _mapping(
        _mapping(local_planner_health or {}).get("local_planner")
    )
    local_hint = _mapping(local_planner.get("last_control_hint"))
    local_result = _mapping(local_planner.get("last_result"))
    path_follower = _mapping(
        _mapping(path_follower_health or {}).get("path_follower")
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
            "wp_index": _safe_int(navigation_health.get("wp_index")),
            "wp_total": _safe_int(navigation_health.get("wp_total")),
            "failure_reason": str(navigation_health.get("failure_reason") or ""),
            "goal": navigation_health.get("goal"),
            "current_waypoint": navigation_health.get("current_waypoint"),
            "distance_to_goal_m": _round_float(
                navigation_health.get("distance_to_goal_m")
            ),
            "active_waypoint_distance_m": _round_float(
                navigation_health.get("active_waypoint_distance_m")
            ),
            "complete_path_on_goal_proximity": bool(
                navigation_health.get("complete_path_on_goal_proximity")
            ),
            "goal_proximity_completion_threshold": _round_float(
                navigation_health.get("goal_proximity_completion_threshold")
            ),
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
        "local_planner": {
            "last_local_path_points": _safe_int(
                local_planner.get("last_local_path_points")
            ),
            "last_local_path_span_m": _round_float(
                local_planner.get("last_local_path_span_m")
            ),
            "last_control_hint": {
                "reason": str(local_hint.get("reason") or ""),
                "safety_stop": bool(local_hint.get("safety_stop")),
                "near_field_stop": bool(local_hint.get("near_field_stop")),
                "path_found": (
                    bool(local_hint.get("path_found"))
                    if "path_found" in local_hint
                    else None
                ),
                "recovery_state": (
                    _safe_int(local_hint.get("recovery_state"))
                    if "recovery_state" in local_hint
                    else None
                ),
            },
            "last_result": {
                "path_point_count": _safe_int(
                    local_result.get("path_point_count")
                ),
                "path_length_m": _round_float(local_result.get("path_length_m")),
                "path_span_m": _round_float(local_result.get("path_span_m")),
                "path_found": (
                    bool(local_result.get("path_found"))
                    if "path_found" in local_result
                    else None
                ),
                "near_field_stop": (
                    bool(local_result.get("near_field_stop"))
                    if "near_field_stop" in local_result
                    else None
                ),
                "recovery_state": (
                    _safe_int(local_result.get("recovery_state"))
                    if "recovery_state" in local_result
                    else None
                ),
            },
        },
        "path_follower": {
            "has_path": (
                bool(path_follower.get("has_path"))
                if "has_path" in path_follower
                else None
            ),
            "vehicle_speed": _round_float(path_follower.get("vehicle_speed")),
            "control_hint": _mapping(path_follower.get("control_hint")),
        },
        "runtime_fault_count": len(runtime_faults),
        "latest_runtime_fault": runtime_faults[-1] if runtime_faults else "",
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

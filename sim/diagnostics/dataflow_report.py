"""Runtime dataflow extraction for DimOS-style simulation artifacts.

This module is intentionally read-only and dependency-light. It summarizes
existing JSON reports; it does not launch ROS, MuJoCo, PCT, or gate commands.
"""

from __future__ import annotations

import glob
import json
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from runtime.runtime_interface import TOPICS

RUNTIME_DATAFLOW_GATES = {
    "native_pct_mujoco",
    "dynamic_obstacle_local_planner",
    "fastlio2_dynamic_inspection",
    "moving_obstacle_sweep",
    "large_loop_closure",
    "gazebo_runtime",
    "large_terrain",
    "saved_map_relocalize",
    "pct_saved_map_navigation",
}

PRODUCT_NAV_RUNTIME_DATAFLOW_GATES = {"policy_nav"}

DEFAULT_CHILD_REPORT_MAX_PARENT_SKEW_S = 24.0 * 60.0 * 60.0
DEFAULT_CHILD_REPORT_MAX_ABSOLUTE_AGE_S = 24.0 * 60.0 * 60.0


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _count(mapping: Mapping[str, Any], key: str) -> int:
    try:
        return int(mapping.get(key) or 0)
    except (TypeError, ValueError):
        return 0


def _safe_int(value: Any) -> int:
    try:
        return int(value or 0)
    except (TypeError, ValueError):
        return 0


def _safe_float(value: Any) -> float:
    try:
        return float(value or 0.0)
    except (TypeError, ValueError):
        return 0.0


def _optional_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if result != result:
        return None
    return result


def _round_float(value: Any, digits: int = 4) -> float | None:
    result = _optional_float(value)
    if result is None:
        return None
    return round(float(result), digits)


def _value_from(*values: Any) -> Any:
    for value in values:
        if value not in (None, ""):
            return value
    return None


def _bool_true(value: Any) -> bool:
    if value is True:
        return True
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "y"}
    return bool(value) if isinstance(value, (int, float)) else False


def _load_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path} does not contain a JSON object")
    return data


def _subcheck_ok(report: Mapping[str, Any], key: str) -> bool:
    value = report.get(key)
    return isinstance(value, Mapping) and value.get("ok") is True


def _video_exists(
    report: Mapping[str, Any],
    *,
    base_dir: Path | None = None,
) -> bool:
    video = report.get("video")
    if not isinstance(video, Mapping):
        return False
    if video.get("exists") is True:
        return True
    raw_path = str(video.get("path") or "").strip()
    if not raw_path:
        return False
    path = Path(raw_path)
    if not path.is_absolute() and base_dir is not None:
        path = base_dir / path
    return path.is_file()


def _checkpoint_ok(inspection: Mapping[str, Any]) -> bool:
    successful = _safe_int(
        _value_from(
            inspection.get("successful_navigation_goal_count"),
            inspection.get("successful_checkpoints"),
            inspection.get("patrol_index"),
        )
    )
    required = _safe_int(
        _value_from(
            inspection.get("min_required_checkpoints"),
            inspection.get("min_checkpoints"),
        )
    )
    if required <= 0:
        required = (
            _safe_int(
                _value_from(
                    inspection.get("goal_count"),
                    inspection.get("patrol_total"),
                )
            )
            or 1
        )
    return successful >= required


def _inspection_terminal_success(inspection: Mapping[str, Any]) -> bool:
    state = str(
        _value_from(
            inspection.get("navigation_state"),
            inspection.get("patrol_state"),
        )
        or ""
    ).upper()
    patrol_index = _safe_int(inspection.get("patrol_index"))
    patrol_total = _safe_int(
        _value_from(
            inspection.get("patrol_total"),
            inspection.get("goal_count"),
        )
    )
    patrol_success = state == "SUCCESS" and patrol_total > 0 and patrol_index >= patrol_total
    return patrol_success and _checkpoint_ok(inspection)


def _tail_sample(mapping: Mapping[str, Any]) -> dict[str, Any]:
    last_sample = _mapping(mapping.get("last_sample"))
    if last_sample:
        return last_sample
    samples_tail = mapping.get("samples_tail")
    if isinstance(samples_tail, list) and samples_tail:
        return _mapping(samples_tail[-1])
    return {}


def _optional_bool_from_mapping(mapping: Mapping[str, Any], key: str) -> bool | None:
    if key not in mapping:
        return None
    return bool(mapping.get(key))


def _latest_local_path_status(
    report: Mapping[str, Any],
    inspection: Mapping[str, Any],
    case: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    case = _mapping(case)
    navigation_diagnostics = _mapping(report.get("navigation_diagnostics")) or _mapping(
        case.get("navigation_diagnostics")
    )
    nav_sample = _tail_sample(navigation_diagnostics)
    nav_paths = _mapping(nav_sample.get("paths"))
    path_diagnostics = _mapping(report.get("path_diagnostics")) or _mapping(case.get("path_diagnostics"))
    path_latest = _mapping(_mapping(path_diagnostics.get("local_path")).get("latest"))
    autonomy_chain = _mapping(report.get("autonomy_chain")) or _mapping(case.get("autonomy_chain"))
    local_planner = _mapping(_mapping(autonomy_chain.get("local_planner")).get("local_planner"))
    local_hint = _mapping(local_planner.get("last_control_hint"))
    path_follower = _mapping(_mapping(autonomy_chain.get("path_follower")).get("path_follower"))

    latest_points_raw = _value_from(
        path_latest.get("point_count"),
        nav_paths.get("local_path_points_latest"),
        local_planner.get("last_local_path_points"),
        inspection.get("local_path_points_latest"),
        case.get("local_path_points_latest"),
    )
    latest_span_raw = _value_from(
        path_latest.get("path_length_m"),
        local_planner.get("last_local_path_span_m"),
        inspection.get("local_path_span_latest_m"),
        case.get("local_path_span_latest_m"),
    )
    safety_stop = _optional_bool_from_mapping(local_hint, "safety_stop")
    path_found = _optional_bool_from_mapping(local_hint, "path_found")
    has_path = _optional_bool_from_mapping(path_follower, "has_path")
    checked = any(
        value is not None
        for value in (
            latest_points_raw,
            latest_span_raw,
            safety_stop,
            path_found,
            has_path,
        )
    )
    latest_points = _safe_int(latest_points_raw)
    terminal_inspection = {**case, **dict(inspection)}
    terminal_success = _inspection_terminal_success(terminal_inspection)
    blockers: list[str] = []
    if not terminal_success:
        if latest_points_raw is not None and latest_points < 2:
            blockers.append("latest local_path has fewer than 2 points")
        if safety_stop is True:
            blockers.append("local planner requested safety_stop")
        if has_path is False:
            blockers.append("path follower has no active path")
    return {
        "checked": checked,
        "ok": checked and not blockers,
        "terminal_success": terminal_success,
        "latest_local_path_points": latest_points if latest_points_raw is not None else None,
        "latest_local_path_span_m": _round_float(latest_span_raw),
        "local_planner_reason": str(local_hint.get("reason") or ""),
        "local_planner_safety_stop": safety_stop,
        "local_planner_path_found": path_found,
        "path_follower_has_path": has_path,
        "blockers": blockers,
    }


def _pct_provenance_from_live(
    report: Mapping[str, Any],
    inspection: Mapping[str, Any],
) -> dict[str, Any]:
    navigation_chain = _mapping(report.get("navigation_chain"))
    last_plan = _mapping(navigation_chain.get("last_plan_report"))
    preview = _mapping(last_plan.get("preview"))
    planner = str(
        _value_from(
            inspection.get("global_planner"),
            report.get("global_planner"),
            navigation_chain.get("selected_planner"),
            last_plan.get("selected_planner"),
            preview.get("selected_planner"),
            last_plan.get("planner"),
            last_plan.get("global_planner"),
        )
        or ""
    ).lower()
    fallback_reason = str(
        _value_from(
            navigation_chain.get("fallback_reason"),
            last_plan.get("fallback_reason"),
            preview.get("fallback_reason"),
        )
        or ""
    )
    fallback_flags = {
        "navigation_chain.planner_fallback_used": navigation_chain.get("planner_fallback_used"),
        "navigation_chain.planner_repair_used": navigation_chain.get("planner_repair_used"),
        "last_plan_report.fallback_used": last_plan.get("fallback_used"),
    }
    fallback_used = any(_bool_true(value) for value in fallback_flags.values())
    checked = bool(planner or fallback_reason or any(value is not None for value in fallback_flags.values()))
    if not checked:
        reason = "planner provenance missing"
    elif planner != "pct":
        reason = "global planner is not pct"
    elif fallback_used:
        reason = "planner fallback or repair was used"
    elif fallback_reason:
        reason = "planner fallback reason is present"
    else:
        reason = ""
    return {
        "checked": checked,
        "ok": checked and not reason,
        "planner": planner,
        "fallback_used": fallback_used,
        "fallback_reason": fallback_reason,
        "fallback_flags": fallback_flags,
        "reason": reason,
    }


def _nested_mapping_value(mapping: Mapping[str, Any], *keys: str) -> Any:
    current: Any = mapping
    for key in keys:
        current = _mapping(current).get(key)
    return current


def _same_source_provenance_from_live(
    report: Mapping[str, Any],
    inspection: Mapping[str, Any],
) -> dict[str, Any]:
    deliverable_checks = _mapping(_nested_mapping_value(report, "deliverable_contract", "checks"))
    map_artifacts = _mapping(report.get("map_artifacts"))
    source_contract = _mapping(map_artifacts.get("source_contract"))
    root_assets = _mapping(report.get("assets"))
    map_assets = _mapping(map_artifacts.get("assets"))
    map_pcd_asset = _mapping(map_assets.get("map_pcd"))
    tomogram_asset = _mapping(map_assets.get("tomogram"))
    tomogram = str(
        _value_from(
            inspection.get("tomogram"),
            report.get("inspection_tomogram"),
            report.get("tomogram"),
            root_assets.get("tomogram"),
            tomogram_asset.get("path"),
        )
        or ""
    )
    world = str(_value_from(report.get("world"), report.get("scene_xml")) or "")
    world_parent = str(report.get("world_parent") or "")
    tomogram_parent = str(report.get("tomogram_parent") or "")
    same_source_check = deliverable_checks.get("same_source_map_artifact")
    same_source_tomogram = source_contract.get("same_source_tomogram")
    same_source_pcd = source_contract.get("same_source_pcd")
    map_artifacts_ok = map_artifacts.get("ok")
    map_pcd_sha256 = str(
        _value_from(
            map_pcd_asset.get("sha256"),
            source_contract.get("map_pcd_sha256"),
            map_artifacts.get("map_pcd_sha256"),
        )
        or ""
    )
    map_pcd_point_count = _safe_int(map_pcd_asset.get("point_count"))
    tomogram_sha256 = str(tomogram_asset.get("sha256") or "")
    tomogram_source_map_sha256 = str(
        _value_from(
            tomogram_asset.get("source_map_sha256"),
            tomogram_asset.get("input_pcd_sha256"),
        )
        or ""
    )
    tomogram_expected = bool(tomogram or tomogram_asset or same_source_tomogram is not None)
    parent_match = bool(world_parent and tomogram_parent and world_parent == tomogram_parent)
    checked = any(
        value not in (None, "")
        for value in (
            same_source_check,
            same_source_tomogram,
            same_source_pcd,
            map_artifacts_ok,
            map_pcd_sha256,
            tomogram_sha256,
            tomogram_source_map_sha256,
            world_parent,
            tomogram_parent,
            tomogram,
        )
    )
    pcd_proven = same_source_pcd is True and bool(map_pcd_sha256) and map_pcd_point_count > 0
    tomogram_proven = not tomogram_expected or (
        same_source_tomogram is True
        and bool(tomogram_sha256)
        and bool(tomogram_source_map_sha256)
        and tomogram_source_map_sha256 == map_pcd_sha256
    )
    parents_consistent = not (world_parent or tomogram_parent) or parent_match
    supporting_checks_ok = same_source_check is not False and map_artifacts_ok is not False
    ok = pcd_proven and tomogram_proven and parents_consistent and supporting_checks_ok
    if ok:
        reason = ""
    elif not checked:
        reason = "same-source provenance missing"
    elif same_source_check is False:
        reason = "same-source map artifact check failed"
    elif map_artifacts_ok is False:
        reason = "map artifacts report is not ok"
    elif same_source_pcd is not True:
        reason = "same-source map.pcd check failed"
    elif not map_pcd_sha256:
        reason = "map_pcd.sha256 missing"
    elif map_pcd_point_count <= 0:
        reason = "map_pcd.point_count missing"
    elif same_source_tomogram is False:
        reason = "same-source tomogram check failed"
    elif tomogram_expected and not tomogram_sha256:
        reason = "tomogram.sha256 missing"
    elif tomogram_expected and not tomogram_source_map_sha256:
        reason = "tomogram.source_map_sha256 missing"
    elif tomogram_expected and tomogram_source_map_sha256 != map_pcd_sha256:
        reason = "tomogram.source_map_sha256 does not match map_pcd.sha256"
    elif world_parent or tomogram_parent:
        reason = "world/tomogram parents do not match"
    else:
        reason = "same-source map artifact is not proven"
    return {
        "checked": checked,
        "ok": ok,
        "reason": reason,
        "world": world,
        "tomogram": tomogram,
        "world_parent": world_parent,
        "tomogram_parent": tomogram_parent,
        "same_source_map_artifact": same_source_check,
        "same_source_tomogram": same_source_tomogram,
        "same_source_pcd": same_source_pcd,
        "map_artifacts_ok": map_artifacts_ok,
        "map_pcd_sha256": map_pcd_sha256,
        "map_pcd_point_count": map_pcd_point_count,
        "tomogram_sha256": tomogram_sha256,
        "tomogram_source_map_sha256": tomogram_source_map_sha256,
    }


def _pct_planner_runtime_contract(report: Mapping[str, Any]) -> dict[str, Any]:
    pct_planner_runtime = _mapping(report.get("pct_planner_runtime"))
    runtime_name = str(pct_planner_runtime.get("runtime") or "").strip()
    runtime_entry_ok = pct_planner_runtime.get("ok") is True
    runtime_ok = report.get("pct_planner_runtime_ok") is True
    global_planner_source = str(report.get("global_planner_source") or "")
    checked = "pct_planner_runtime" in report or "pct_planner_runtime_ok" in report or bool(global_planner_source)
    if not runtime_name:
        reason = "PCT planner runtime is not selected"
    elif not runtime_entry_ok:
        reason = "PCT planner runtime entry is not ok"
    elif not runtime_ok:
        reason = "pct_planner_runtime_ok is not true"
    elif global_planner_source != "source_report/pct_tomogram":
        reason = "global planner source is not source_report/pct_tomogram"
    else:
        reason = ""
    return {
        "checked": checked,
        "ok": checked and not reason,
        "pct_planner_runtime": dict(pct_planner_runtime),
        "pct_planner_runtime_name": runtime_name,
        "pct_planner_runtime_entry_ok": runtime_entry_ok,
        "pct_planner_runtime_ok": runtime_ok,
        "global_planner_source": global_planner_source,
        "reason": reason,
    }


def _pct_provenance_from_runtime_report(
    report: Mapping[str, Any],
) -> dict[str, Any]:
    planner = str(report.get("selected_planner") or report.get("planner") or "").lower()
    fallback_used = _bool_true(report.get("fallback_used"))
    runtime_contract = _pct_planner_runtime_contract(report)
    pct_path_count = _safe_int(report.get("pct_path_count"))
    ok = planner == "pct" and not fallback_used and runtime_contract["ok"] and pct_path_count >= 2
    if ok:
        reason = ""
    elif planner != "pct":
        reason = "selected planner is not pct"
    elif fallback_used:
        reason = "planner fallback was used"
    elif not runtime_contract["ok"]:
        reason = str(runtime_contract["reason"])
    else:
        reason = "PCT path is too short"
    return {
        "checked": True,
        "ok": ok,
        "planner": planner,
        "fallback_used": fallback_used,
        "pct_planner_runtime": runtime_contract["pct_planner_runtime"],
        "pct_planner_runtime_ok": runtime_contract["pct_planner_runtime_ok"],
        "global_planner_source": runtime_contract["global_planner_source"],
        "pct_path_count": pct_path_count,
        "runtime_contract": runtime_contract,
        "reason": reason,
    }


def _pct_optimizer_mode_from_runtime_report(
    report: Mapping[str, Any],
) -> dict[str, Any]:
    source_contract = _mapping(report.get("source_planning_contract"))
    enabled = _value_from(
        report.get("pct_optimizer_enabled"),
        source_contract.get("pct_optimizer_enabled"),
    )
    attempted = _value_from(
        report.get("pct_optimizer_attempted"),
        source_contract.get("pct_optimizer_attempted"),
    )
    accepted = _value_from(
        report.get("pct_optimizer_accepted"),
        source_contract.get("pct_optimizer_accepted"),
    )
    reject_reason = str(
        _value_from(
            report.get("pct_optimizer_reject_reason"),
            source_contract.get("pct_optimizer_reject_reason"),
        )
        or ""
    )
    blocked_sample_count = _safe_int(
        _value_from(
            report.get("pct_optimizer_blocked_sample_count"),
            source_contract.get("pct_optimizer_blocked_sample_count"),
        )
    )
    path_mode = str(
        _value_from(
            report.get("pct_planner_path_mode"),
            source_contract.get("pct_planner_path_mode"),
        )
        or ""
    )
    checked = enabled in (True, False) or bool(path_mode)
    rejection_recorded = attempted is True and accepted is False and bool(reject_reason)
    if enabled is False:
        ok = path_mode == "astar_raw_path"
        reason = "" if ok else "optimizer disabled but path mode is not astar_raw_path"
    elif enabled is True and path_mode == "optimized_trajectory":
        ok = accepted is not False
        reason = "" if ok else "optimized trajectory path mode is marked rejected"
    elif enabled is True and path_mode == "astar_raw_path":
        ok = rejection_recorded
        reason = "" if ok else "raw path mode lacks recorded optimizer rejection"
    elif not checked:
        ok = False
        reason = "optimizer mode evidence missing"
    else:
        ok = False
        reason = "unsupported PCT planner path mode"
    return {
        "checked": checked,
        "ok": ok,
        "optimizer_enabled": enabled,
        "optimizer_attempted": attempted,
        "optimizer_accepted": accepted,
        "optimizer_reject_reason": reject_reason,
        "optimizer_blocked_sample_count": blocked_sample_count,
        "planner_path_mode": path_mode,
        "optimizer_rejection_recorded": rejection_recorded,
        "reason": reason,
    }


def summarize_live_report(
    report: Mapping[str, Any],
    *,
    report_path: Path | None = None,
    require_video_file: bool = False,
) -> dict[str, Any]:
    """Summarize a flat live Fast-LIO navigation report."""
    outputs = _mapping(report.get("outputs"))
    inspection = _mapping(report.get("lingtu_inspection"))
    base_dir = report_path.parent if report_path is not None else None
    latest_local_path = _latest_local_path_status(report, inspection)
    historical_local_path_ok = _count(inspection, "local_path_count") > 0

    edges: list[dict[str, Any]] = [
        {
            "id": "raw_lidar_to_fastlio",
            "ok": _count(outputs, "fastlio2_cloud_registered") > 0 or _count(outputs, "fastlio2_cloud_map") > 0,
            "evidence": {
                "fastlio2_cloud_registered": _count(
                    outputs,
                    "fastlio2_cloud_registered",
                ),
                "fastlio2_cloud_map": _count(outputs, "fastlio2_cloud_map"),
            },
        },
        {
            "id": "fastlio_odometry_to_navigation",
            "ok": _count(outputs, "fastlio2_odometry") > 0 and _count(outputs, "nav_odometry") > 0,
            "evidence": {
                "fastlio2_odometry": _count(outputs, "fastlio2_odometry"),
                "nav_odometry": _count(outputs, "nav_odometry"),
            },
        },
        {
            "id": "fastlio_map_cloud_to_navigation",
            "ok": (_count(outputs, "fastlio2_cloud_map") > 0 or _count(outputs, "fastlio2_cloud_registered") > 0)
            and (_count(outputs, "nav_map_cloud") > 0 or _count(outputs, "nav_registered_cloud") > 0),
            "evidence": {
                "nav_map_cloud": _count(outputs, "nav_map_cloud"),
                "nav_registered_cloud": _count(outputs, "nav_registered_cloud"),
            },
        },
        {
            "id": "global_planner_to_local_planner",
            "ok": _count(inspection, "global_path_count") > 0
            and historical_local_path_ok
            and (latest_local_path.get("checked") is not True or latest_local_path.get("ok") is True),
            "evidence": {
                "global_path_count": _count(inspection, "global_path_count"),
                "local_path_count": _count(inspection, "local_path_count"),
                "latest_local_path": latest_local_path,
            },
        },
        {
            "id": "path_follower_to_cmd_vel",
            "ok": _count(outputs, "nav_cmd_vel_nonzero") > 0,
            "evidence": {
                "nav_cmd_vel": _count(outputs, "nav_cmd_vel"),
                "nav_cmd_vel_nonzero": _count(outputs, "nav_cmd_vel_nonzero"),
            },
        },
        {
            "id": "fastlio_motion_consistency",
            "ok": _subcheck_ok(report, "fastlio2_motion_consistency"),
            "evidence": report.get("fastlio2_motion_consistency") or {},
        },
        {
            "id": "fastlio_z_consistency",
            "ok": _subcheck_ok(report, "fastlio2_z_consistency"),
            "evidence": report.get("fastlio2_z_consistency") or {},
        },
        {
            "id": "fastlio_yaw_consistency",
            "ok": _subcheck_ok(report, "fastlio2_yaw_consistency"),
            "evidence": report.get("fastlio2_yaw_consistency") or {},
        },
        {
            "id": "inspection_checkpoints",
            "ok": _checkpoint_ok(inspection),
            "evidence": {
                "successful_navigation_goal_count": _count(
                    inspection,
                    "successful_navigation_goal_count",
                ),
                "min_required_checkpoints": _count(
                    inspection,
                    "min_required_checkpoints",
                ),
                "goal_count": _count(inspection, "goal_count"),
                "patrol_state": inspection.get("patrol_state"),
            },
        },
    ]

    if require_video_file:
        edges.append(
            {
                "id": "video_evidence",
                "ok": _video_exists(report, base_dir=base_dir),
                "evidence": report.get("video") or {},
            }
        )

    failed = [edge for edge in edges if edge["ok"] is not True]
    return {
        "ok": not failed,
        "primary_blocker": failed[0]["id"] if failed else "",
        "flow": edges,
        "pct_provenance": _pct_provenance_from_live(report, inspection),
        "same_source_provenance": _same_source_provenance_from_live(
            report,
            inspection,
        ),
        "remaining_gaps": list(report.get("remaining_gaps") or []),
        "source_report_ok": report.get("ok"),
        "source_report": str(report_path) if report_path is not None else "",
    }


def resolve_report_path(value: str) -> Path:
    path = Path(value)
    if path.is_dir():
        return path / "report.json"
    if path.name == "latest.txt":
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.startswith("latest_run_dir="):
                return Path(line.split("=", 1)[1]) / "report.json"
    return path


def _repo_path(value: Any, *, root: Path) -> Path | None:
    text = str(value or "").strip()
    if not text:
        return None
    artifact_path = _artifact_path_from_remote_text(text, root=root)
    path = Path(text)
    if path.is_absolute():
        if path.exists():
            return path
        if artifact_path is not None:
            return artifact_path
        return path
    if artifact_path is not None:
        return artifact_path
    path = root / path
    return path


def _artifact_path_from_remote_text(text: str, *, root: Path) -> Path | None:
    normalized = text.replace("\\", "/")
    marker = "/artifacts/"
    marker_index = normalized.find(marker)
    if marker_index >= 0:
        return root / normalized[marker_index + 1 :]
    if normalized.startswith("artifacts/"):
        return root / normalized
    return None


def _has_glob_chars(path: Path) -> bool:
    return any(char in str(path) for char in ("*", "?", "["))


def _glob_report_paths(pattern: Path) -> list[Path]:
    if not _has_glob_chars(pattern):
        return [pattern] if pattern.is_file() else []
    return [Path(match) for match in glob.glob(str(pattern), recursive=True) if Path(match).is_file()]


def _latest_report_path(paths: list[Path]) -> Path | None:
    if not paths:
        return None

    def sort_key(path: Path) -> tuple[float, str]:
        try:
            mtime = path.stat().st_mtime
        except OSError:
            mtime = 0.0
        return (mtime, str(path))

    return max(paths, key=sort_key)


def _path_mtime(path: Path) -> float:
    try:
        return float(path.stat().st_mtime)
    except OSError:
        return 0.0


def _report_reference_time(path: Path, report: Mapping[str, Any]) -> float:
    generated_at = _optional_float(report.get("generated_at"))
    if generated_at is not None and generated_at > 0.0:
        return generated_at
    return _path_mtime(path)


def _child_report_freshness(
    *,
    parent_path: Path,
    parent_report: Mapping[str, Any],
    child_path: Path,
    child_report: Mapping[str, Any],
    max_parent_skew_s: float = DEFAULT_CHILD_REPORT_MAX_PARENT_SKEW_S,
    max_absolute_age_s: float = DEFAULT_CHILD_REPORT_MAX_ABSOLUTE_AGE_S,
) -> dict[str, Any]:
    checked_at = time.time()
    parent_time = _report_reference_time(parent_path, parent_report)
    child_time = _report_reference_time(child_path, child_report)
    age_vs_parent_s = max(0.0, parent_time - child_time)
    child_age_s = max(0.0, checked_at - child_time) if child_time > 0.0 else None
    blockers: list[str] = []
    if child_time <= 0.0:
        blockers.append("child report timestamp missing")
    if parent_time > 0.0 and age_vs_parent_s > max_parent_skew_s:
        blockers.append(f"child_report_age_vs_parent_s {age_vs_parent_s:.3f} > {max_parent_skew_s:.3f}")
    if child_age_s is not None and child_age_s > max_absolute_age_s:
        blockers.append(f"child_report_age_s {child_age_s:.3f} > {max_absolute_age_s:.3f}")
    return {
        "checked": True,
        "fresh": not blockers,
        "stale": bool(blockers),
        "checked_at": checked_at,
        "parent_report": str(parent_path),
        "child_report": str(child_path),
        "parent_reference_time": parent_time,
        "child_reference_time": child_time,
        "age_vs_parent_s": round(age_vs_parent_s, 3),
        "child_age_s": None if child_age_s is None else round(child_age_s, 3),
        "max_parent_skew_s": max_parent_skew_s,
        "max_absolute_age_s": max_absolute_age_s,
        "blockers": blockers,
    }


def _glob_anchor(path: Path) -> Path | None:
    parts = path.parts
    anchor_parts: list[str] = []
    for part in parts:
        if any(char in part for char in ("*", "?", "[")):
            break
        anchor_parts.append(part)
    if not anchor_parts:
        return None
    return Path(*anchor_parts)


def _missing_report_candidate_paths(
    gate_name: str,
    requested_path: Path | None,
    *,
    root: Path,
    limit: int = 8,
) -> list[Path]:
    candidates: list[Path] = []
    if requested_path is not None:
        candidates.extend(_glob_report_paths(requested_path))

    if gate_name == "fastlio2_dynamic_inspection":
        anchor = _glob_anchor(requested_path) if requested_path is not None else None
        if anchor is not None and anchor.is_dir():
            candidates.extend(
                candidate
                for candidate in _glob_report_paths(anchor / "**" / "report.json")
                if "fastlio" in str(candidate).lower()
            )
        base = root / "artifacts" / "server_sim_closure"
        candidates.extend(
            _glob_report_paths(
                base / "mujoco_fastlio2_live*" / "**" / "report.json",
            )
        )
        candidates.extend(_glob_report_paths(base / "fastlio2_live" / "report.json"))

    unique: dict[str, Path] = {}
    for candidate in candidates:
        unique.setdefault(str(candidate), candidate)
    return sorted(unique.values(), key=lambda path: str(path))[:limit]


def _failed_edge_ids(summary: Mapping[str, Any]) -> list[str]:
    return [
        str(edge.get("id") or "")
        for edge in summary.get("flow") or []
        if isinstance(edge, Mapping) and edge.get("ok") is not True and str(edge.get("id") or "")
    ]


def _candidate_rejection_reason(
    gate_name: str,
    summary: Mapping[str, Any],
) -> str:
    if summary.get("ok") is True:
        return f"candidate dataflow passes but is not the expected {gate_name} gate report"
    blocker = str(summary.get("primary_blocker") or summary.get("reason") or "")
    if blocker:
        return f"candidate report does not satisfy {gate_name}: {blocker}"
    return f"candidate report does not satisfy {gate_name}"


def _missing_report_candidate_diagnostics(
    gate_name: str,
    requested_path: Path | None,
    *,
    root: Path,
) -> list[dict[str, Any]]:
    diagnostics: list[dict[str, Any]] = []
    for candidate in _missing_report_candidate_paths(
        gate_name,
        requested_path,
        root=root,
    ):
        item: dict[str, Any] = {
            "path": str(candidate),
            "readable": False,
            "checked": False,
            "ok": False,
            "schema_detected": "",
            "primary_blocker": "",
            "reason": "",
            "failed_edges": [],
            "rejection_reason": "",
        }
        try:
            report = _load_json(candidate)
        except Exception as exc:  # pragma: no cover - CLI diagnostics path.
            item["reason"] = f"candidate_unreadable: {exc}"
            item["rejection_reason"] = item["reason"]
            diagnostics.append(item)
            continue
        summary = summarize_runtime_report(report, report_path=candidate)
        item.update(
            {
                "readable": True,
                "checked": summary.get("checked") is not False,
                "ok": summary.get("ok") is True,
                "schema_detected": str(summary.get("schema_detected") or ""),
                "primary_blocker": str(summary.get("primary_blocker") or ""),
                "reason": str(summary.get("reason") or ""),
                "failed_edges": _failed_edge_ids(summary),
                "source_report_ok": summary.get("source_report_ok"),
                "rejection_reason": _candidate_rejection_reason(
                    gate_name,
                    summary,
                ),
            }
        )
        diagnostics.append(item)
    return diagnostics


def _has_live_dataflow_shape(report: Mapping[str, Any]) -> bool:
    return isinstance(report.get("outputs"), Mapping) or isinstance(
        report.get("lingtu_inspection"),
        Mapping,
    )


def _has_native_pct_shape(report: Mapping[str, Any]) -> bool:
    return (
        str(report.get("schema_version") or "").startswith("lingtu.native_pct_mujoco_gate")
        or "pct_planner_runtime" in report
        or "pct_planner_runtime_ok" in report
        or "cmd_count_nonzero" in report
    )


def _has_policy_nav_shape(report: Mapping[str, Any]) -> bool:
    schema = str(report.get("schema_version") or "")
    if schema.startswith("lingtu.policy_nav_smoke"):
        return True
    checks = [check for check in report.get("checks") or [] if isinstance(check, Mapping)]
    return any(str(check.get("mode") or "") == "full_stack_policy_nav" for check in checks)


def _has_pct_saved_map_navigation_shape(report: Mapping[str, Any]) -> bool:
    schema = str(report.get("schema_version") or "")
    return (
        schema.startswith("lingtu.pct_saved_map_navigation_gate")
        or str(report.get("validation_level") or "") == "saved_map_relocalized_pct_navigation"
        or (
            isinstance(report.get("relocalization"), Mapping)
            and isinstance(report.get("plan_preview"), Mapping)
            and isinstance(report.get("native_gate"), Mapping)
        )
    )


def _has_saved_map_relocalize_shape(report: Mapping[str, Any]) -> bool:
    schema = str(report.get("schema_version") or "")
    return (
        schema.startswith("lingtu.saved_map_relocalize_runtime")
        or str(report.get("validation_level") or "") == "runtime_relocalization"
        or str(report.get("runtime_stage") or "") == "saved_map_relocalization"
    )


def _has_large_terrain_shape(report: Mapping[str, Any]) -> bool:
    schema = str(report.get("schema_version") or "")
    return (
        schema.startswith("lingtu.large_terrain_nav_validation")
        or str(report.get("validation_level") or "") == "global_planning_assets"
        or (
            str(report.get("selection_policy") or "") == "first_route_ok_after_primary"
            and isinstance(report.get("cases"), list)
        )
    )


def _has_dynamic_obstacle_local_planner_shape(report: Mapping[str, Any]) -> bool:
    schema = str(report.get("schema_version") or "")
    return schema.startswith("lingtu.dynamic_obstacle_local_planner") or (
        "dynamic_replan_verified" in report
        and "obstacle_response_verified" in report
        and isinstance(report.get("phases"), list)
    )


def _has_gazebo_runtime_shape(report: Mapping[str, Any]) -> bool:
    schema = str(report.get("schema_version") or "")
    return schema.startswith("lingtu.gazebo_runtime_smoke") or (
        isinstance(report.get("nav_loop"), Mapping)
        and isinstance(report.get("frontier_exploration"), Mapping)
        and isinstance(report.get("topic_samples"), Mapping)
    )


def _has_moving_obstacle_sweep_shape(report: Mapping[str, Any]) -> bool:
    schema = str(report.get("schema_version") or "")
    return schema.startswith("lingtu.moving_obstacle_sweep_gate") or (
        isinstance(report.get("required_pairs"), list) and "passed_pair_count" in report
    )


def _has_large_loop_closure_shape(report: Mapping[str, Any]) -> bool:
    schema = str(report.get("schema_version") or "")
    return schema.startswith("lingtu.large_loop_closure_gate") or (
        isinstance(report.get("thresholds"), Mapping) and "passed_case_count" in report
    )


def _edge(edge_id: str, ok: bool, evidence: dict[str, Any]) -> dict[str, Any]:
    return {"id": edge_id, "ok": bool(ok), "evidence": evidence}


def _bool_false(value: Any) -> bool:
    if value is False:
        return True
    if isinstance(value, str):
        return value.strip().lower() in {"0", "false", "no", "n"}
    return False


def _sim_only_boundary(report: Mapping[str, Any]) -> bool:
    return (
        report.get("simulation_only") is True
        and _bool_false(report.get("real_robot_motion"))
        and _bool_false(report.get("cmd_vel_sent_to_hardware"))
    )


def _topic_samples_present(
    samples: Mapping[str, Any],
    topics: tuple[str, ...],
) -> bool:
    return all(_safe_int(samples.get(topic)) > 0 for topic in topics)


def _summarize_gazebo_runtime_report(
    report: Mapping[str, Any],
    *,
    report_path: Path | None,
) -> dict[str, Any]:
    samples = _mapping(report.get("topic_samples"))
    frames = _mapping(report.get("topic_frames"))
    point_counts = _mapping(report.get("point_counts"))
    nav_loop = _mapping(report.get("nav_loop"))
    nav_samples = _mapping(_value_from(nav_loop.get("samples"), nav_loop.get("topic_samples")))
    publisher_contract = _mapping(nav_loop.get("publisher_contract"))
    frontier = _mapping(report.get("frontier_exploration"))
    frontier_samples = _mapping(frontier.get("samples"))
    trajectory_quality = _mapping(frontier.get("trajectory_quality"))
    topic_sync = _mapping(frontier.get("topic_sync"))
    frontier_stall = _mapping(frontier.get("frontier_no_gain_stall"))
    cumulative = _mapping(frontier.get("cumulative_map_cloud"))
    registered = _mapping(frontier.get("registered_cloud"))
    static_obstacles = _mapping(frontier.get("static_obstacles"))
    tare = _mapping(report.get("tare_exploration"))

    tf_topics = (TOPICS.map_cloud, TOPICS.registered_cloud)
    tf_ok = (
        _safe_int(report.get("samples")) > 0
        and report.get("odometry_frame_id") == "odom"
        and report.get("odometry_child_frame_id") == "body"
        and _topic_samples_present(samples, tf_topics)
        and frames.get(TOPICS.map_cloud) == "odom"
        and frames.get(TOPICS.registered_cloud) == "body"
        and _safe_int(point_counts.get(TOPICS.map_cloud)) > 0
        and _safe_int(point_counts.get(TOPICS.registered_cloud)) > 0
    )

    nav_required_flags = (
        "goal_published",
        "odometry_seen",
        "global_path_seen",
        "local_path_seen",
        "cmd_vel_seen",
        "cmd_vel_nonzero",
    )
    nav_topics = (
        TOPICS.global_path,
        TOPICS.local_path,
        TOPICS.cmd_vel,
        TOPICS.odometry,
    )
    nav_loop_ok = (
        nav_loop.get("ok") is True
        and all(nav_loop.get(key) is True for key in nav_required_flags)
        and _safe_float(nav_loop.get("odom_delta_m")) >= 0.05
        and _topic_samples_present(nav_samples, nav_topics)
        and publisher_contract.get("ok") is True
        and not list(publisher_contract.get("errors") or [])
    )

    frontier_required_flags = (
        "frontier_started",
        "frontier_goal_seen",
        "frontier_goal_published",
        "odometry_seen",
        "map_cloud_seen",
        "global_path_seen",
        "local_path_seen",
        "cmd_vel_seen",
        "cmd_vel_nonzero",
    )
    frontier_topics = (
        TOPICS.goal_pose,
        TOPICS.global_path,
        TOPICS.local_path,
        TOPICS.cmd_vel,
        TOPICS.odometry,
        TOPICS.terrain_map,
        TOPICS.terrain_map_ext,
    )
    frontier_goal = frontier.get("frontier_goal") or []
    frontier_goal_ok = (
        isinstance(frontier_goal, list) and len(frontier_goal) >= 2 and _safe_float(frontier_goal[0]) >= 0.75
    )
    trajectory_ok = (
        trajectory_quality.get("ok") is True
        and _safe_int(trajectory_quality.get("room_violation_count")) == 0
        and _safe_float(trajectory_quality.get("max_out_of_room_m")) <= 0.05
        and _safe_int(trajectory_quality.get("local_path_occupied_overlap_count")) == 0
        and _safe_float(trajectory_quality.get("min_obstacle_clearance_m")) >= 0.18
    )
    sync_ok = topic_sync.get("ok") is True and _safe_float(topic_sync.get("max_cloud_odom_skew_ms")) <= 250.0
    frontier_ok = (
        frontier.get("ok") is True
        and all(frontier.get(key) is True for key in frontier_required_flags)
        and _safe_float(frontier.get("odom_delta_m")) >= 0.05
        and _safe_float(frontier.get("odom_delta_x_m")) >= 0.05
        and _safe_float(frontier.get("explored_area_delta_m2")) > 0.0
        and _safe_int(frontier.get("known_cells_delta")) > 0
        and _safe_int(frontier.get("frontier_count_max")) > 0
        and frontier_goal_ok
        and _topic_samples_present(frontier_samples, frontier_topics)
        and frontier.get("terrain_map_seen") is True
        and frontier.get("terrain_map_ext_seen") is True
        and trajectory_ok
        and sync_ok
    )

    required_observation_s = _safe_float(frontier_stall.get("required_observation_s"))
    observed_s = _safe_float(frontier_stall.get("observed_s"))
    frontier_stall_ok = (
        frontier_stall.get("checked") is True
        and frontier_stall.get("ok") is True
        and str(frontier_stall.get("stop_reason") or "") == "post_pass_observation_elapsed"
        and required_observation_s > 0.0
        and observed_s + 1e-6 >= required_observation_s
    )

    static_roi_ok = any(
        isinstance(item, Mapping)
        and _safe_int(item.get("samples")) >= 2
        and _safe_float(item.get("centroid_drift_max_m")) <= 0.25
        for item in static_obstacles.values()
    )
    cumulative_ok = (
        frontier.get("cumulative_map_cloud_seen") is True
        and "odom" in set(cumulative.get("frame_ids") or [])
        and _safe_int(cumulative.get("samples")) >= 8
        and _safe_int(cumulative.get("unique_voxels_delta")) > 0
        and _safe_float(cumulative.get("retention_min")) >= 0.6
        and _safe_float(registered.get("map_vs_registered_voxel_ratio")) >= 1.5
        and static_roi_ok
    )

    tare_ok = (
        tare.get("ok") is True
        and _sim_only_boundary(tare)
        and tare.get("source_contract_ok") is True
        and tare.get("backend") == "tare"
        and (tare.get("runtime_required") is not True or tare.get("runtime_available") is True)
    )

    non_motion_ok = (
        _sim_only_boundary(report)
        and _sim_only_boundary(nav_loop)
        and _sim_only_boundary(frontier)
        and _sim_only_boundary(tare)
    )
    edges = [
        _edge(
            "gazebo_tf_topic_contract",
            tf_ok,
            {
                "samples": _safe_int(report.get("samples")),
                "odometry_frame_id": report.get("odometry_frame_id"),
                "odometry_child_frame_id": report.get("odometry_child_frame_id"),
                "topic_samples": {topic: _safe_int(samples.get(topic)) for topic in tf_topics},
                "topic_frames": {topic: frames.get(topic) for topic in tf_topics},
                "point_counts": {topic: _safe_int(point_counts.get(topic)) for topic in tf_topics},
            },
        ),
        _edge(
            "nav_loop_closed",
            nav_loop_ok,
            {
                "ok": nav_loop.get("ok"),
                "required_flags": {key: nav_loop.get(key) for key in nav_required_flags},
                "odom_delta_m": _safe_float(nav_loop.get("odom_delta_m")),
                "topic_samples": {topic: _safe_int(nav_samples.get(topic)) for topic in nav_topics},
                "publisher_contract": publisher_contract,
            },
        ),
        _edge(
            "frontier_exploration_closed",
            frontier_ok,
            {
                "ok": frontier.get("ok"),
                "required_flags": {key: frontier.get(key) for key in frontier_required_flags},
                "odom_delta_m": _safe_float(frontier.get("odom_delta_m")),
                "odom_delta_x_m": _safe_float(frontier.get("odom_delta_x_m")),
                "known_cells_delta": _safe_int(frontier.get("known_cells_delta")),
                "explored_area_delta_m2": _safe_float(frontier.get("explored_area_delta_m2")),
                "frontier_count_max": _safe_int(frontier.get("frontier_count_max")),
                "frontier_goal": frontier_goal,
                "topic_samples": {topic: _safe_int(frontier_samples.get(topic)) for topic in frontier_topics},
                "trajectory_quality": trajectory_quality,
                "topic_sync": topic_sync,
            },
        ),
        _edge(
            "frontier_no_gain_stall",
            frontier_stall_ok,
            {
                "checked": frontier_stall.get("checked"),
                "ok": frontier_stall.get("ok"),
                "stop_reason": frontier_stall.get("stop_reason") or "",
                "required_observation_s": required_observation_s,
                "observed_s": observed_s,
            },
        ),
        _edge(
            "cumulative_map_retention",
            cumulative_ok,
            {
                "cumulative_map_cloud_seen": frontier.get("cumulative_map_cloud_seen"),
                "cumulative_map_cloud": cumulative,
                "registered_cloud": registered,
                "static_obstacles": static_obstacles,
                "static_roi_ok": static_roi_ok,
            },
        ),
        _edge(
            "tare_exploration_contract",
            tare_ok,
            {
                "ok": tare.get("ok"),
                "backend": tare.get("backend"),
                "source_contract_ok": tare.get("source_contract_ok"),
                "runtime_required": tare.get("runtime_required"),
                "runtime_available": tare.get("runtime_available"),
            },
        ),
        _edge(
            "non_motion_claim_boundary",
            non_motion_ok,
            {
                "gazebo_runtime": {
                    "simulation_only": report.get("simulation_only"),
                    "real_robot_motion": report.get("real_robot_motion"),
                    "cmd_vel_sent_to_hardware": report.get("cmd_vel_sent_to_hardware"),
                },
                "nav_loop": {
                    "simulation_only": nav_loop.get("simulation_only"),
                    "real_robot_motion": nav_loop.get("real_robot_motion"),
                    "cmd_vel_sent_to_hardware": nav_loop.get("cmd_vel_sent_to_hardware"),
                },
                "frontier_exploration": {
                    "simulation_only": frontier.get("simulation_only"),
                    "real_robot_motion": frontier.get("real_robot_motion"),
                    "cmd_vel_sent_to_hardware": frontier.get("cmd_vel_sent_to_hardware"),
                },
                "tare_exploration": {
                    "simulation_only": tare.get("simulation_only"),
                    "real_robot_motion": tare.get("real_robot_motion"),
                    "cmd_vel_sent_to_hardware": tare.get("cmd_vel_sent_to_hardware"),
                },
            },
        ),
    ]
    failed = [edge for edge in edges if edge["ok"] is not True]
    return {
        "checked": True,
        "ok": not failed,
        "schema_detected": "gazebo_runtime",
        "primary_blocker": failed[0]["id"] if failed else "",
        "flow": edges,
        "remaining_gaps": list(report.get("errors") or report.get("remaining_gaps") or []),
        "source_report_ok": report.get("ok"),
        "source_report": str(report_path) if report_path is not None else "",
        "claim_boundary": "gazebo_simulation_only_frontier_runtime_no_hardware",
    }


def _summarize_moving_obstacle_sweep_report(
    report: Mapping[str, Any],
    *,
    report_path: Path | None,
) -> dict[str, Any]:
    cases = [case for case in report.get("cases") or [] if isinstance(case, Mapping)]
    required_pairs = [str(pair) for pair in report.get("required_pairs") or []]
    covered_pairs = [str(pair) for pair in report.get("covered_pairs") or []]
    missing_pairs = [str(pair) for pair in report.get("missing_pairs") or []]
    required_speed_bins = [str(value) for value in report.get("required_speed_bins") or []]
    required_density_bins = [str(value) for value in report.get("required_density_bins") or []]
    passed_pair_count = _safe_int(report.get("passed_pair_count"))
    case_count = _safe_int(report.get("case_count"))
    min_clearance_m = _safe_float(report.get("min_clearance_m")) or 0.25
    require_video_file = report.get("require_video_file") is True
    required_live_nav_chain = report.get("required_live_nav_chain") is True
    required_scan_time_profile = str(report.get("required_scan_time_profile") or "")

    matrix_ok = (
        len(required_pairs) >= 4
        and "slow" in required_speed_bins
        and "fast" in required_speed_bins
        and "sparse" in required_density_bins
        and "dense" in required_density_bins
        and case_count >= len(required_pairs)
        and passed_pair_count >= len(required_pairs)
        and not missing_pairs
        and all(pair in covered_pairs for pair in required_pairs)
    )
    live_chain_status: dict[str, bool] = {}
    clearance_status: dict[str, bool] = {}
    scan_time_status: dict[str, bool] = {}
    video_status: dict[str, bool] = {}
    for index, case in enumerate(cases):
        pair = str(case.get("pair") or f"case[{index}]")
        live_nav_chain = _mapping(case.get("live_nav_chain"))
        live_chain_status[pair] = live_nav_chain.get("ok") is True
        clearance_status[pair] = (
            case.get("ok") is True and _safe_float(case.get("trail_min_margin_m")) >= min_clearance_m
        )
        scan_time_status[pair] = str(case.get("scan_time_profile") or "") == "physical_rolling"
        video = _mapping(case.get("video"))
        video_status[pair] = not require_video_file or (
            bool(case.get("video_path") or video.get("path"))
            and _safe_int(
                _value_from(
                    case.get("video_frame_count"),
                    video.get("frame_count"),
                    video.get("frames"),
                )
            )
            > 0
            and _safe_int(
                _value_from(
                    case.get("video_sample_count"),
                    video.get("sample_count"),
                    video.get("samples"),
                )
            )
            > 0
        )

    live_chain_ok = required_live_nav_chain and bool(cases) and all(live_chain_status.values())
    clearance_ok = bool(cases) and all(clearance_status.values())
    scan_time_ok = required_scan_time_profile == "physical_rolling" and bool(cases) and all(scan_time_status.values())
    video_ok = not require_video_file or (bool(cases) and all(video_status.values()))
    non_motion_ok = _sim_only_boundary(report)
    edges = [
        _edge(
            "speed_density_matrix",
            matrix_ok,
            {
                "case_count": case_count,
                "passed_pair_count": passed_pair_count,
                "required_pairs": required_pairs,
                "covered_pairs": covered_pairs,
                "missing_pairs": missing_pairs,
                "required_speed_bins": required_speed_bins,
                "required_density_bins": required_density_bins,
            },
        ),
        _edge(
            "live_fastlio_pct_nav_chain",
            live_chain_ok,
            {
                "required_live_nav_chain": report.get("required_live_nav_chain"),
                "case_status": live_chain_status,
            },
        ),
        _edge(
            "dynamic_obstacle_clearance",
            clearance_ok,
            {
                "min_clearance_m": min_clearance_m,
                "case_status": clearance_status,
            },
        ),
        _edge(
            "scan_time_model",
            scan_time_ok,
            {
                "required_scan_time_profile": required_scan_time_profile,
                "case_status": scan_time_status,
            },
        ),
        _edge(
            "video_evidence",
            video_ok,
            {
                "require_video": report.get("require_video"),
                "require_video_file": require_video_file,
                "case_status": video_status,
            },
        ),
        _edge(
            "non_motion_claim_boundary",
            non_motion_ok,
            {
                "simulation_only": report.get("simulation_only"),
                "real_robot_motion": report.get("real_robot_motion"),
                "cmd_vel_sent_to_hardware": report.get("cmd_vel_sent_to_hardware"),
            },
        ),
    ]
    failed = [edge for edge in edges if edge["ok"] is not True]
    return {
        "checked": True,
        "ok": not failed,
        "schema_detected": "moving_obstacle_sweep",
        "primary_blocker": failed[0]["id"] if failed else "",
        "flow": edges,
        "remaining_gaps": list(report.get("blockers") or []),
        "source_report_ok": report.get("ok"),
        "source_report": str(report_path) if report_path is not None else "",
        "claim_boundary": "aggregate_report_only_no_new_mujoco_run",
    }


def _summarize_large_loop_closure_report(
    report: Mapping[str, Any],
    *,
    report_path: Path | None,
) -> dict[str, Any]:
    cases = [case for case in report.get("cases") or [] if isinstance(case, Mapping)]
    best_case = _mapping(report.get("best_case"))
    thresholds = _mapping(report.get("thresholds"))
    passed_case_count = _safe_int(report.get("passed_case_count"))
    min_path_length_m = _safe_float(thresholds.get("min_path_length_m")) or 20.0
    min_goal_span_m = _safe_float(thresholds.get("min_goal_span_m")) or 4.0
    max_loop_error_m = _safe_float(thresholds.get("max_loop_closure_error_m")) or 0.75
    max_fastlio_loop_error_m = _safe_float(thresholds.get("max_fastlio_loop_closure_error_m")) or 1.0
    max_loop_yaw_error_rad = _safe_float(thresholds.get("max_loop_yaw_error_rad")) or 0.5
    require_video_file = thresholds.get("require_video_file") is True
    required_scan_time_profile = str(thresholds.get("required_scan_time_profile") or "")

    matrix_ok = bool(cases) and passed_case_count > 0 and bool(best_case)
    long_motion_ok = (
        matrix_ok
        and str(best_case.get("global_planner") or "").lower() == "pct"
        and _safe_float(best_case.get("sim_path_length_m")) >= min_path_length_m
        and _safe_float(best_case.get("fastlio2_path_length_m")) >= min_path_length_m * 0.8
        and _safe_float(best_case.get("goal_span_m")) >= min_goal_span_m
    )
    closure_ok = (
        matrix_ok
        and _safe_float(best_case.get("sim_loop_closure_error_m")) <= max_loop_error_m
        and _safe_float(best_case.get("fastlio2_loop_closure_error_m")) <= max_fastlio_loop_error_m
        and _safe_float(best_case.get("sim_loop_yaw_error_rad")) <= max_loop_yaw_error_rad
        and _safe_float(best_case.get("fastlio2_loop_yaw_error_rad")) <= max_loop_yaw_error_rad
    )
    nav_chain_ok = (
        matrix_ok
        and _safe_int(best_case.get("nav_cmd_vel_nonzero")) > 0
        and _safe_int(best_case.get("local_path_count")) > 0
        and best_case.get("same_source_artifacts") is True
    )
    scan_time_ok = (
        required_scan_time_profile == "physical_rolling"
        and matrix_ok
        and str(best_case.get("scan_time_profile") or "") == "physical_rolling"
    )
    video_ok = not require_video_file or (
        matrix_ok
        and bool(best_case.get("video_path"))
        and _safe_int(best_case.get("video_frame_count")) > 0
        and _safe_int(best_case.get("video_sample_count")) > 0
    )
    non_motion_ok = _sim_only_boundary(report)
    edges = [
        _edge(
            "large_loop_runtime_matrix",
            matrix_ok,
            {
                "case_count": _safe_int(report.get("case_count")),
                "passed_case_count": passed_case_count,
                "failed_case_count": _safe_int(report.get("failed_case_count")),
                "best_case_present": bool(best_case),
            },
        ),
        _edge(
            "long_range_loop_motion",
            long_motion_ok,
            {
                "global_planner": best_case.get("global_planner"),
                "sim_path_length_m": _safe_float(best_case.get("sim_path_length_m")),
                "fastlio2_path_length_m": _safe_float(best_case.get("fastlio2_path_length_m")),
                "goal_span_m": _safe_float(best_case.get("goal_span_m")),
                "min_path_length_m": min_path_length_m,
                "min_goal_span_m": min_goal_span_m,
            },
        ),
        _edge(
            "loop_closure_consistency",
            closure_ok,
            {
                "sim_loop_closure_error_m": _safe_float(best_case.get("sim_loop_closure_error_m")),
                "fastlio2_loop_closure_error_m": _safe_float(best_case.get("fastlio2_loop_closure_error_m")),
                "sim_loop_yaw_error_rad": _safe_float(best_case.get("sim_loop_yaw_error_rad")),
                "fastlio2_loop_yaw_error_rad": _safe_float(best_case.get("fastlio2_loop_yaw_error_rad")),
                "max_loop_closure_error_m": max_loop_error_m,
                "max_fastlio_loop_closure_error_m": max_fastlio_loop_error_m,
                "max_loop_yaw_error_rad": max_loop_yaw_error_rad,
            },
        ),
        _edge(
            "fastlio_pct_navigation_chain",
            nav_chain_ok,
            {
                "nav_cmd_vel_nonzero": _safe_int(best_case.get("nav_cmd_vel_nonzero")),
                "local_path_count": _safe_int(best_case.get("local_path_count")),
                "same_source_artifacts": best_case.get("same_source_artifacts"),
            },
        ),
        _edge(
            "scan_time_model",
            scan_time_ok,
            {
                "required_scan_time_profile": required_scan_time_profile,
                "best_case_scan_time_profile": best_case.get("scan_time_profile"),
            },
        ),
        _edge(
            "video_evidence",
            video_ok,
            {
                "require_video_file": require_video_file,
                "video_path": best_case.get("video_path") or "",
                "video_frame_count": _safe_int(best_case.get("video_frame_count")),
                "video_sample_count": _safe_int(best_case.get("video_sample_count")),
            },
        ),
        _edge(
            "non_motion_claim_boundary",
            non_motion_ok,
            {
                "simulation_only": report.get("simulation_only"),
                "real_robot_motion": report.get("real_robot_motion"),
                "cmd_vel_sent_to_hardware": report.get("cmd_vel_sent_to_hardware"),
            },
        ),
    ]
    failed = [edge for edge in edges if edge["ok"] is not True]
    return {
        "checked": True,
        "ok": not failed,
        "schema_detected": "large_loop_closure",
        "primary_blocker": failed[0]["id"] if failed else "",
        "flow": edges,
        "remaining_gaps": list(report.get("blockers") or []),
        "source_report_ok": report.get("ok"),
        "source_report": str(report_path) if report_path is not None else "",
        "claim_boundary": "aggregate_report_only_no_new_mujoco_run",
    }


def _summarize_dynamic_obstacle_local_planner_report(
    report: Mapping[str, Any],
    *,
    report_path: Path | None,
) -> dict[str, Any]:
    phases = [dict(phase) for phase in report.get("phases") or [] if isinstance(phase, Mapping)]
    by_name = {str(phase.get("name") or ""): phase for phase in phases}
    required_phase_names = (
        "clear_initial",
        "obstacle_left",
        "obstacle_right",
        "obstacle_center",
        "clear_recovered",
    )
    local_backend = _mapping(_mapping(report.get("algorithm_backends")).get("local_planner"))
    backend_actual = str(report.get("backend_actual") or local_backend.get("backend_actual") or "")
    backend_requested = str(report.get("backend_requested") or local_backend.get("requested") or "")
    backend_ok = (
        backend_actual == "nanobind"
        and report.get("native_backend_used") is True
        and local_backend.get("native_backend_used") is True
        and local_backend.get("degraded") is not True
    )
    phase_matrix_ok = bool(phases) and all(
        name in by_name
        and _safe_int(by_name[name].get("path_count")) >= 20
        and str(by_name[name].get("path_frame_id") or "") == "map"
        for name in required_phase_names
    )
    left_side = str(by_name.get("obstacle_left", {}).get("avoidance_side") or "")
    right_side = str(by_name.get("obstacle_right", {}).get("avoidance_side") or "")
    center_side = str(by_name.get("obstacle_center", {}).get("avoidance_side") or "")
    dynamic_response_ok = (
        report.get("dynamic_replan_verified") is True
        and left_side == "right"
        and right_side == "left"
        and center_side in {"left", "right"}
    )
    clearance = _safe_float(report.get("min_clearance_m"))
    obstacle_clearance_ok = report.get("obstacle_response_verified") is True and clearance >= 0.25
    clear_recovery_ok = (
        report.get("clear_path_recovery_verified") is True
        and str(by_name.get("clear_initial", {}).get("avoidance_side") or "") == "straight"
        and str(by_name.get("clear_recovered", {}).get("avoidance_side") or "") == "straight"
    )
    frames = _mapping(report.get("frames"))
    non_motion_ok = (
        report.get("simulation_only") is True
        and report.get("real_robot_motion") is False
        and report.get("cmd_vel_sent_to_hardware") is False
        and str(frames.get("cmd_vel") or "") == "not_published"
    )
    edges = [
        _edge(
            "local_planner_backend",
            backend_ok,
            {
                "backend_requested": backend_requested,
                "backend_actual": backend_actual,
                "native_backend_used": report.get("native_backend_used"),
                "local_planner_backend": local_backend,
            },
        ),
        _edge(
            "dynamic_obstacle_phase_matrix",
            phase_matrix_ok,
            {
                "required_phases": list(required_phase_names),
                "phase_count": len(phases),
                "phase_status": {
                    name: {
                        "present": name in by_name,
                        "path_count": _safe_int(by_name.get(name, {}).get("path_count")),
                        "path_frame_id": by_name.get(name, {}).get("path_frame_id"),
                    }
                    for name in required_phase_names
                },
            },
        ),
        _edge(
            "dynamic_replan_response",
            dynamic_response_ok,
            {
                "dynamic_replan_verified": report.get("dynamic_replan_verified"),
                "avoidance_sequence": [
                    by_name.get("clear_initial", {}).get("avoidance_side"),
                    left_side,
                    right_side,
                    center_side,
                    by_name.get("clear_recovered", {}).get("avoidance_side"),
                ],
            },
        ),
        _edge(
            "obstacle_clearance",
            obstacle_clearance_ok,
            {
                "obstacle_response_verified": report.get("obstacle_response_verified"),
                "min_clearance_m": clearance,
                "required_min_clearance_m": 0.25,
            },
        ),
        _edge(
            "clear_path_recovery",
            clear_recovery_ok,
            {
                "clear_path_recovery_verified": report.get("clear_path_recovery_verified"),
                "clear_initial": by_name.get("clear_initial", {}),
                "clear_recovered": by_name.get("clear_recovered", {}),
            },
        ),
        _edge(
            "non_motion_claim_boundary",
            non_motion_ok,
            {
                "simulation_only": report.get("simulation_only"),
                "real_robot_motion": report.get("real_robot_motion"),
                "cmd_vel_sent_to_hardware": report.get("cmd_vel_sent_to_hardware"),
                "frames": frames,
                "execution_mode": report.get("execution_mode") or "",
            },
        ),
    ]
    failed = [edge for edge in edges if edge["ok"] is not True]
    return {
        "checked": True,
        "ok": not failed,
        "schema_detected": "dynamic_obstacle_local_planner",
        "primary_blocker": failed[0]["id"] if failed else "",
        "flow": edges,
        "remaining_gaps": list(report.get("errors") or report.get("remaining_gaps") or []),
        "source_report_ok": report.get("ok"),
        "source_report": str(report_path) if report_path is not None else "",
        "claim_boundary": "local_planner_dynamic_obstacle_no_path_follower_or_cmd_vel",
    }


def _large_terrain_map_artifacts(report: Mapping[str, Any]) -> dict[str, Any]:
    root_artifacts = _mapping(report.get("map_artifacts"))
    if root_artifacts:
        return root_artifacts
    for case in report.get("cases") or []:
        if not isinstance(case, Mapping):
            continue
        artifacts = _mapping(case.get("map_artifacts"))
        if artifacts:
            return artifacts
    return {}


def _large_terrain_plan_rows(report: Mapping[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for case in report.get("cases") or []:
        if not isinstance(case, Mapping):
            continue
        route = str(case.get("route") or "")
        for plan in case.get("planning") or []:
            if isinstance(plan, Mapping):
                rows.append({"route": route, **dict(plan)})
    return rows


def _large_terrain_same_source_provenance(report: Mapping[str, Any]) -> dict[str, Any]:
    artifacts = _large_terrain_map_artifacts(report)
    contract = _mapping(artifacts.get("source_contract"))
    assets = _mapping(artifacts.get("assets"))
    metadata = _mapping(artifacts.get("metadata"))
    map_pcd = _mapping(assets.get("map_pcd"))
    tomogram = _mapping(assets.get("tomogram"))
    map_sha = str(map_pcd.get("sha256") or "")
    tomogram_sha = str(tomogram.get("sha256") or "")
    tomogram_source_sha = str(
        _value_from(
            tomogram.get("source_map_sha256"),
            tomogram.get("input_pcd_sha256"),
        )
        or ""
    )
    point_count = _safe_int(map_pcd.get("point_count"))
    same_source_pcd = contract.get("same_source_pcd") is True
    same_source_tomogram = contract.get("same_source_tomogram") is True
    checked = bool(artifacts or map_sha or tomogram_sha or metadata)
    ok = (
        artifacts.get("ok") is True
        and same_source_pcd
        and same_source_tomogram
        and bool(map_sha)
        and point_count > 0
        and bool(tomogram_sha)
        and tomogram_source_sha == map_sha
    )
    if ok:
        reason = ""
    elif not checked:
        reason = "large-terrain same-source map artifacts missing"
    elif artifacts.get("ok") is not True:
        reason = "large-terrain map_artifacts.ok is not true"
    elif not same_source_pcd:
        reason = "large-terrain same_source_pcd is not true"
    elif not same_source_tomogram:
        reason = "large-terrain same_source_tomogram is not true"
    elif not map_sha:
        reason = "large-terrain map_pcd.sha256 missing"
    elif point_count <= 0:
        reason = "large-terrain map_pcd.point_count missing"
    elif not tomogram_sha:
        reason = "large-terrain tomogram.sha256 missing"
    elif tomogram_source_sha != map_sha:
        reason = "large-terrain tomogram.source_map_sha256 does not match map_pcd.sha256"
    else:
        reason = "large-terrain same-source artifact proof failed"
    return {
        "checked": checked,
        "ok": ok,
        "reason": reason,
        "map_artifacts_ok": artifacts.get("ok"),
        "same_source_pcd": contract.get("same_source_pcd"),
        "same_source_tomogram": contract.get("same_source_tomogram"),
        "map_pcd_sha256": map_sha,
        "map_pcd_point_count": point_count,
        "tomogram_sha256": tomogram_sha,
        "tomogram_source_map_sha256": tomogram_source_sha,
        "metadata": metadata,
    }


def _large_terrain_pct_provenance(report: Mapping[str, Any]) -> dict[str, Any]:
    pct_rows = [
        row
        for row in _large_terrain_plan_rows(report)
        if str(_value_from(row.get("planner_requested"), row.get("planner")) or "").lower() == "pct"
        or str(row.get("planner") or "").lower() == "pct"
    ]
    pct_planner_runtime = _mapping(report.get("pct_planner_runtime"))
    runtime_name = str(pct_planner_runtime.get("runtime") or "").strip()
    pct_planner_runtime_ok = (
        bool(runtime_name) and pct_planner_runtime.get("ok") is True and report.get("pct_planner_runtime_ok") is True
    )
    fallback_rows = [
        row
        for row in pct_rows
        if str(row.get("selected_planner") or "").lower() != "pct" or bool(str(row.get("fallback_reason") or ""))
    ]
    planner_runtime_rows = []
    for row in pct_rows:
        row_runtime = _mapping(row.get("pct_planner_runtime"))
        if (
            str(row_runtime.get("runtime") or "").strip() == runtime_name
            and row_runtime.get("ok") is True
            and row.get("pct_planner_runtime_ok") is True
            and str(row.get("selected_planner") or "").lower() == "pct"
            and row.get("feasible") is True
            and row.get("route_ok") is True
        ):
            planner_runtime_rows.append(row)
    ok = bool(pct_rows) and len(planner_runtime_rows) == len(pct_rows) and not fallback_rows and pct_planner_runtime_ok
    if ok:
        reason = ""
    elif not pct_rows:
        reason = "no PCT planning case found"
    elif not pct_planner_runtime_ok:
        reason = "PCT planner runtime unavailable"
    elif fallback_rows:
        reason = "PCT planner fallback or repair was used"
    elif len(planner_runtime_rows) != len(pct_rows):
        reason = "not every PCT case used the selected planner runtime with a safe route"
    else:
        reason = "PCT global planning is not proven"
    return {
        "checked": bool(pct_rows or "pct_planner_runtime" in report or "pct_planner_runtime_ok" in report),
        "ok": ok,
        "reason": reason,
        "planner": "pct" if pct_rows else "",
        "pct_plan_count": len(pct_rows),
        "pct_planner_runtime_plan_count": len(planner_runtime_rows),
        "fallback_used": bool(fallback_rows),
        "pct_planner_runtime_ok": pct_planner_runtime_ok,
        "pct_planner_runtime": dict(pct_planner_runtime),
        "routes": [row.get("route") for row in pct_rows],
    }


def _large_terrain_case_safety(case: Mapping[str, Any]) -> dict[str, Any]:
    plans = [dict(plan) for plan in case.get("planning") or [] if isinstance(plan, Mapping)]
    pct_plans = [
        plan
        for plan in plans
        if str(_value_from(plan.get("planner_requested"), plan.get("planner")) or "").lower() == "pct"
        or str(plan.get("planner") or "").lower() == "pct"
    ]
    relevant_plans = pct_plans or plans
    path_safety = _mapping(case.get("path_safety"))
    plan_safety = [_mapping(plan.get("path_safety")) for plan in relevant_plans]
    safety_ok = (
        (path_safety.get("ok") is True or not path_safety)
        and bool(relevant_plans)
        and all(item.get("ok") is True for item in plan_safety)
    )
    route_ok = bool(relevant_plans) and all(plan.get("route_ok") is True for plan in relevant_plans)
    distance_ok = True
    distances: list[dict[str, Any]] = []
    for plan in relevant_plans:
        metrics = _mapping(plan.get("metrics"))
        routed = _safe_float(metrics.get("route_distance_m"))
        required = _safe_float(metrics.get("min_required_route_distance_m"))
        tolerance = _safe_float(metrics.get("route_distance_tolerance_m"))
        if metrics:
            item_ok = routed + tolerance >= required
            distance_ok = distance_ok and item_ok
            distances.append(
                {
                    "planner": plan.get("planner"),
                    "route_distance_m": routed,
                    "min_required_route_distance_m": required,
                    "route_distance_tolerance_m": tolerance,
                    "ok": item_ok,
                }
            )
    return {
        "route": case.get("route"),
        "ok": case.get("ok") is True,
        "route_ok": route_ok,
        "safety_ok": safety_ok,
        "distance_ok": distance_ok,
        "distances": distances,
    }


def _large_terrain_gate_crossing(report: Mapping[str, Any]) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    for row in _large_terrain_plan_rows(report):
        crossing = _mapping(row.get("gate_crossing"))
        if crossing.get("checked") is not True:
            continue
        checks.append(
            {
                "route": row.get("route"),
                "planner": row.get("planner"),
                "passed_gate": crossing.get("passed_gate") is True,
                "min_y_at_wall": crossing.get("min_y_at_wall"),
                "max_y_at_wall": crossing.get("max_y_at_wall"),
            }
        )
    return {
        "checked_count": len(checks),
        "ok": all(item["passed_gate"] for item in checks),
        "checks": checks,
    }


def _summarize_large_terrain_report(
    report: Mapping[str, Any],
    *,
    report_path: Path | None,
) -> dict[str, Any]:
    cases = [case for case in report.get("cases") or [] if isinstance(case, Mapping)]
    pct_provenance = _large_terrain_pct_provenance(report)
    same_source = _large_terrain_same_source_provenance(report)
    case_safety = [_large_terrain_case_safety(case) for case in cases]
    gate_crossing = _large_terrain_gate_crossing(report)
    non_motion_ok = (
        report.get("simulation_only") is True
        and report.get("real_robot_motion") is False
        and report.get("cmd_vel_sent_to_hardware") is False
    )
    safe_matrix_ok = bool(case_safety) and all(
        item["ok"] and item["route_ok"] and item["safety_ok"] and item["distance_ok"] for item in case_safety
    )
    edges = [
        _edge(
            "pct_planner_runtime",
            pct_provenance.get("ok") is True,
            pct_provenance,
        ),
        _edge(
            "same_source_large_terrain_assets",
            same_source.get("ok") is True,
            same_source,
        ),
        _edge(
            "large_terrain_route_safety",
            report.get("ok") is True and safe_matrix_ok,
            {
                "report_ok": report.get("ok"),
                "case_count": len(cases),
                "case_safety": case_safety,
                "environment_blockers": list(report.get("environment_blockers") or []),
            },
        ),
        _edge(
            "terrain_gate_constraints",
            gate_crossing["ok"],
            gate_crossing,
        ),
        _edge(
            "non_motion_claim_boundary",
            non_motion_ok,
            {
                "simulation_only": report.get("simulation_only"),
                "real_robot_motion": report.get("real_robot_motion"),
                "cmd_vel_sent_to_hardware": report.get("cmd_vel_sent_to_hardware"),
                "validation_level": report.get("validation_level") or "",
            },
        ),
    ]
    failed = [edge for edge in edges if edge["ok"] is not True]
    return {
        "checked": True,
        "ok": not failed,
        "schema_detected": "large_terrain",
        "primary_blocker": failed[0]["id"] if failed else "",
        "flow": edges,
        "pct_provenance": pct_provenance,
        "same_source_provenance": same_source,
        "remaining_gaps": list(report.get("remaining_gaps") or []),
        "source_report_ok": report.get("ok"),
        "source_report": str(report_path) if report_path is not None else "",
        "claim_boundary": "global_planning_assets_no_local_planner_or_cmd_vel",
    }


def _summarize_native_pct_report(
    report: Mapping[str, Any],
    *,
    report_path: Path | None,
) -> dict[str, Any]:
    pct_path_count = _safe_int(report.get("pct_path_count"))
    path_count = _safe_int(report.get("path_count"))
    max_path_poses = _safe_int(report.get("max_path_poses"))
    cmd_count_nonzero = _safe_int(report.get("cmd_count_nonzero"))
    moved_m = _safe_float(report.get("moved_m"))
    final_distance_m = _safe_float(report.get("final_distance_m"))
    reached_goal = report.get("reached_goal") is True
    planner = str(report.get("selected_planner") or report.get("planner") or "")
    claim_boundary = str(report.get("claim_boundary") or "")
    ros2_runtime = _mapping(report.get("environment"))
    ros2_runtime_ready = not (
        report.get("native_gate_skipped") is True and claim_boundary == "ros2_runtime_unavailable"
    )
    runtime_contract = _pct_planner_runtime_contract(report)
    optimizer_mode = _pct_optimizer_mode_from_runtime_report(report)
    edges = [
        _edge(
            "pct_planner_runtime",
            runtime_contract["ok"]
            and pct_path_count >= 2
            and planner.lower() == "pct"
            and report.get("fallback_used") is not True,
            {
                **runtime_contract,
                "pct_path_count": pct_path_count,
                "selected_planner": report.get("selected_planner") or report.get("planner"),
                "fallback_used": report.get("fallback_used"),
            },
        ),
        _edge(
            "pct_optimizer_mode",
            optimizer_mode["ok"],
            optimizer_mode,
        ),
        _edge(
            "ros2_runtime",
            ros2_runtime_ready,
            {
                "claim_boundary": claim_boundary,
                "native_gate_skipped": report.get("native_gate_skipped"),
                "ros2_executable": ros2_runtime.get("ros2_executable"),
                "ros_distro": ros2_runtime.get("ros_distro"),
                "diagnostic_commands": ros2_runtime.get("diagnostic_commands") or [],
                "blockers": list(report.get("blockers") or []),
            },
        ),
        _edge(
            "global_path_to_local_planner",
            path_count > 0 or max_path_poses >= 2,
            {
                "path_count": path_count,
                "max_path_poses": max_path_poses,
                "local_path_sample_count": len(report.get("local_path_samples") or []),
            },
        ),
        _edge(
            "path_follower_to_cmd_vel",
            cmd_count_nonzero > 0,
            {
                "cmd_count_nonzero": cmd_count_nonzero,
                "cmd_sample_count": len(report.get("cmd_samples") or []),
                "max_abs_linear": report.get("max_abs_linear"),
                "max_abs_angular_z": report.get("max_abs_angular_z"),
            },
        ),
        _edge(
            "cmd_vel_to_mujoco_motion",
            moved_m > 0.05,
            {
                "moved_m": moved_m,
                "steps": _safe_int(report.get("steps")),
                "duration_s": _safe_float(report.get("duration_s")),
            },
        ),
        _edge(
            "checkpoint_goal",
            reached_goal,
            {
                "reached_goal": reached_goal,
                "final_distance_m": final_distance_m,
                "target_idx_final": report.get("target_idx_final"),
                "waypoint_advance": report.get("waypoint_advance") or {},
            },
        ),
    ]
    failed = [edge for edge in edges if edge["ok"] is not True]
    return {
        "checked": True,
        "ok": not failed,
        "schema_detected": "native_pct_mujoco",
        "primary_blocker": failed[0]["id"] if failed else "",
        "flow": edges,
        "pct_provenance": _pct_provenance_from_runtime_report(report),
        "pct_optimizer_mode": optimizer_mode,
        "same_source_provenance": _same_source_provenance_from_live(report, {}),
        "remaining_gaps": list(report.get("remaining_gaps") or []),
        "claim_boundary": claim_boundary,
        "environment": dict(ros2_runtime),
        "source_report_ok": report.get("ok"),
        "source_report": str(report_path) if report_path is not None else "",
    }


def _summarize_policy_nav_report(
    report: Mapping[str, Any],
    *,
    report_path: Path | None,
) -> dict[str, Any]:
    checks = [check for check in report.get("checks") or [] if isinstance(check, Mapping)]
    nav_checks = [check for check in checks if str(check.get("mode") or "") == "full_stack_policy_nav"]
    nav = _mapping(nav_checks[-1]) if nav_checks else {}
    direct_checks = [check for check in checks if str(check.get("mode") or "") == "direct_policy"]
    direct = _mapping(direct_checks[-1]) if direct_checks else {}
    seen = _mapping(nav.get("seen"))
    planner_status = _mapping(nav.get("global_planner_backend_status"))
    costmap_readiness = _mapping(nav.get("costmap_readiness"))
    path_cmd_stats = _mapping(nav.get("path_follower_cmd_stats"))
    mux_cmd_stats = _mapping(nav.get("mux_cmd_stats"))
    global_path = _mapping(nav.get("global_path"))
    last_nonempty_local_path = _mapping(nav.get("last_nonempty_local_path"))
    contacts = _mapping(nav.get("contacts"))

    planner_configured = str(planner_status.get("configured_backend") or "")
    planner_backend = str(planner_status.get("backend") or "")
    local_backend = str(nav.get("local_planner_backend_actual") or "")
    path_follower_backend = str(nav.get("path_follower_backend_actual") or "")
    global_path_points = _safe_int(global_path.get("count"))
    local_nonempty_count = _safe_int(nav.get("local_path_nonempty_count"))
    last_local_points = _safe_int(last_nonempty_local_path.get("count"))
    path_cmd_nonzero = _safe_int(path_cmd_stats.get("nonzero_count"))
    mux_cmd_nonzero = _safe_int(mux_cmd_stats.get("nonzero_count"))
    dist_for_gate = _value_from(nav.get("dist_at_success_m"), nav.get("dist_to_goal_m"))
    close_enough = nav.get("success_seen") is True or (
        _optional_float(dist_for_gate) is not None and _safe_float(dist_for_gate) <= 0.10
    )

    top_boundary_ok = _sim_only_boundary(report)
    nav_boundary_ok = (
        nav.get("simulation_only") is True
        and nav.get("real_robot_motion") is False
        and nav.get("cmd_vel_sent_to_hardware") is False
    )
    contact_ok = (
        _safe_int(contacts.get("foot_contact_sample_count")) >= 3
        and _safe_int(contacts.get("unique_feet_count")) >= 2
        and _safe_int(contacts.get("non_foot_ground_contacts")) == 0
    )

    edges = [
        _edge(
            "simulation_only_boundary",
            top_boundary_ok and nav_boundary_ok,
            {
                "report_simulation_only": report.get("simulation_only"),
                "report_real_robot_motion": report.get("real_robot_motion"),
                "report_cmd_vel_sent_to_hardware": report.get("cmd_vel_sent_to_hardware"),
                "nav_simulation_only": nav.get("simulation_only"),
                "nav_real_robot_motion": nav.get("real_robot_motion"),
                "nav_cmd_vel_sent_to_hardware": nav.get("cmd_vel_sent_to_hardware"),
            },
        ),
        _edge(
            "policy_loaded",
            nav.get("policy_loaded") is True and bool(nav.get("policy_path")),
            {
                "nav_policy_loaded": nav.get("policy_loaded"),
                "nav_policy_backend": nav.get("policy_backend"),
                "nav_policy_path": nav.get("policy_path") or "",
                "direct_policy_loaded": direct.get("policy_loaded"),
            },
        ),
        _edge(
            "octoplanner3d_backend",
            planner_configured == "octoplanner3d" and planner_status.get("degraded") is not True,
            {
                "configured_backend": planner_configured,
                "backend": planner_backend,
                "degraded": planner_status.get("degraded"),
                "requested_backend": nav.get("global_planner_backend_requested"),
            },
        ),
        _edge(
            "global_path_to_waypoints",
            _safe_int(seen.get("waypoints")) > 0 and global_path_points >= 2,
            {
                "seen_waypoints": _safe_int(seen.get("waypoints")),
                "global_path_points": global_path_points,
                "global_path_frame": global_path.get("frame_id") or "",
            },
        ),
        _edge(
            "nanobind_local_planner",
            local_backend == "nanobind"
            and _safe_int(seen.get("local_path")) > 0
            and (local_nonempty_count > 0 or last_local_points >= 2),
            {
                "local_planner_backend_actual": local_backend,
                "local_planner_backend_requested": nav.get("local_planner_backend_requested"),
                "seen_local_path": _safe_int(seen.get("local_path")),
                "local_path_nonempty_count": local_nonempty_count,
                "last_nonempty_local_path_points": last_local_points,
                "control_hint_safety_stop_count": _safe_int(nav.get("control_hint_safety_stop_count")),
            },
        ),
        _edge(
            "nav_kernel_path_follower",
            path_follower_backend == "nav_kernel"
            and _safe_int(seen.get("path_follower_cmd")) > 3
            and path_cmd_nonzero > 0,
            {
                "path_follower_backend_actual": path_follower_backend,
                "path_follower_backend_requested": nav.get("path_follower_backend_requested"),
                "seen_path_follower_cmd": _safe_int(seen.get("path_follower_cmd")),
                "path_follower_cmd_nonzero": path_cmd_nonzero,
            },
        ),
        _edge(
            "cmd_vel_mux_to_policy_driver",
            _safe_int(seen.get("mux_cmd")) > 3 and mux_cmd_nonzero > 0 and _safe_int(seen.get("direct_fallback")) == 0,
            {
                "seen_mux_cmd": _safe_int(seen.get("mux_cmd")),
                "mux_cmd_nonzero": mux_cmd_nonzero,
                "direct_goal_fallback_count": _safe_int(seen.get("direct_fallback")),
                "last_mux": list(nav.get("last_mux") or []),
            },
        ),
        _edge(
            "policy_motion_goal",
            nav.get("passed") is True
            and nav.get("finite") is True
            and _safe_float(nav.get("moved_m")) >= 0.20
            and close_enough,
            {
                "passed": nav.get("passed"),
                "finite": nav.get("finite"),
                "success_seen": nav.get("success_seen"),
                "moved_m": _safe_float(nav.get("moved_m")),
                "dist_at_success_m": nav.get("dist_at_success_m"),
                "dist_to_goal_m": nav.get("dist_to_goal_m"),
                "nav_state": nav.get("nav_state") or "",
            },
        ),
        _edge(
            "policy_contact_stability",
            contact_ok,
            {
                "foot_contact_sample_count": _safe_int(contacts.get("foot_contact_sample_count")),
                "unique_feet_count": _safe_int(contacts.get("unique_feet_count")),
                "non_foot_ground_contacts": _safe_int(contacts.get("non_foot_ground_contacts")),
            },
        ),
        _edge(
            "planner_map_ready",
            _safe_int(seen.get("costmap")) > 0 or costmap_readiness.get("planner_has_map") is True,
            {
                "seen_costmap": _safe_int(seen.get("costmap")),
                "planner_has_map": costmap_readiness.get("planner_has_map"),
                "source": costmap_readiness.get("source") or "",
            },
        ),
    ]
    failed = [edge for edge in edges if edge["ok"] is not True]
    return {
        "checked": True,
        "ok": not failed,
        "schema_detected": "policy_nav",
        "primary_blocker": failed[0]["id"] if failed else "",
        "flow": edges,
        "planner_backend": {
            "configured_backend": planner_configured,
            "backend": planner_backend,
            "degraded": planner_status.get("degraded"),
        },
        "local_planner_backend": local_backend,
        "path_follower_backend": path_follower_backend,
        "remaining_gaps": list(report.get("remaining_gaps") or []),
        "source_report_ok": report.get("passed"),
        "source_report": str(report_path) if report_path is not None else "",
        "claim_boundary": "product_octoplanner_inprocess_nav_no_ros2",
    }


def _summarize_pct_saved_map_navigation_report(
    report: Mapping[str, Any],
    *,
    report_path: Path | None,
) -> dict[str, Any]:
    relocalization = _mapping(report.get("relocalization"))
    plan_preview = _mapping(report.get("plan_preview"))
    native = _mapping(report.get("native_gate"))
    preview_runtime = _mapping(plan_preview.get("pct_planner_runtime"))
    relocalization_ok = relocalization.get("ok") is True
    preview_ok = (
        plan_preview.get("ok") is True
        and str(plan_preview.get("selected_planner") or "").lower() == "pct"
        and not str(plan_preview.get("fallback_reason") or "")
        and _safe_int(plan_preview.get("path_count")) >= 2
        and bool(str(preview_runtime.get("runtime") or "").strip())
        and preview_runtime.get("ok") is True
    )
    native_planner = str(native.get("selected_planner") or native.get("planner") or "")
    runtime_contract = _pct_planner_runtime_contract(native)
    native_pct_ok = (
        runtime_contract["ok"]
        and _safe_int(native.get("pct_path_count")) >= 2
        and native_planner.lower() == "pct"
        and native.get("fallback_used") is not True
    )
    native_path_ok = _safe_int(native.get("path_count")) > 0 or _safe_int(native.get("max_path_poses")) >= 2
    native_cmd_ok = _safe_int(native.get("cmd_count_nonzero")) > 0
    native_motion_ok = _safe_float(native.get("moved_m")) > 0.05
    native_goal_ok = native.get("reached_goal") is True
    source_report = str(report.get("source_report") or "")
    same_source_provenance = _same_source_provenance_from_live(report, {})
    same_source_hash_identity = _mapping(report.get("same_source_hash_identity"))
    same_source_hash_ok = same_source_hash_identity.get("ok") is True
    same_source_ok = same_source_provenance.get("ok") is True and same_source_hash_ok
    edges = [
        _edge(
            "saved_map_relocalization",
            relocalization_ok,
            {
                "ok": relocalization.get("ok"),
                "latest_health_state": relocalization.get("latest_health_state"),
                "saved_map_cloud_points_latest": relocalization.get("saved_map_cloud_points_latest"),
                "relocalize_report": report.get("relocalize_report") or "",
            },
        ),
        _edge(
            "pct_plan_preview",
            preview_ok,
            {
                "ok": plan_preview.get("ok"),
                "selected_planner": plan_preview.get("selected_planner"),
                "fallback_reason": plan_preview.get("fallback_reason"),
                "path_count": _safe_int(plan_preview.get("path_count")),
                "path": plan_preview.get("path") or "",
                "pct_planner_runtime": dict(preview_runtime),
                "pct_optimizer_enabled": plan_preview.get("pct_optimizer_enabled"),
                "pct_planner_path_mode": plan_preview.get("pct_planner_path_mode") or "",
            },
        ),
        _edge(
            "source_report_to_native_gate",
            bool(source_report) and bool(native),
            {
                "source_report": source_report,
                "native_gate_present": bool(native),
            },
        ),
        _edge(
            "same_source_saved_map_artifacts",
            same_source_ok,
            {
                "same_source_provenance_ok": same_source_provenance.get("ok"),
                "same_source_reason": same_source_provenance.get("reason"),
                "same_source_hash_identity_ok": same_source_hash_ok,
                "same_source_hash_checks": same_source_hash_identity.get("checks") or {},
            },
        ),
        _edge(
            "pct_planner_runtime",
            native_pct_ok,
            {
                **runtime_contract,
                "pct_path_count": _safe_int(native.get("pct_path_count")),
                "selected_planner": native.get("selected_planner") or native.get("planner"),
                "fallback_used": native.get("fallback_used"),
            },
        ),
        _edge(
            "native_global_path_to_local_planner",
            native_path_ok,
            {
                "path_count": _safe_int(native.get("path_count")),
                "max_path_poses": _safe_int(native.get("max_path_poses")),
                "local_path_sample_count": len(native.get("local_path_samples") or []),
            },
        ),
        _edge(
            "native_path_follower_to_cmd_vel",
            native_cmd_ok,
            {
                "cmd_count_nonzero": _safe_int(native.get("cmd_count_nonzero")),
                "cmd_sample_count": len(native.get("cmd_samples") or []),
            },
        ),
        _edge(
            "native_cmd_vel_to_mujoco_motion",
            native_motion_ok,
            {
                "moved_m": _safe_float(native.get("moved_m")),
                "steps": _safe_int(native.get("steps")),
            },
        ),
        _edge(
            "native_checkpoint_goal",
            native_goal_ok,
            {
                "reached_goal": native.get("reached_goal"),
                "final_distance_m": _safe_float(native.get("final_distance_m")),
            },
        ),
    ]
    failed = [edge for edge in edges if edge["ok"] is not True]
    tomogram = str(report.get("tomogram") or "")
    map_metadata = str(report.get("map_metadata") or "")
    return {
        "checked": True,
        "ok": not failed,
        "schema_detected": "pct_saved_map_navigation",
        "primary_blocker": failed[0]["id"] if failed else "",
        "flow": edges,
        "pct_provenance": _pct_provenance_from_runtime_report(native),
        "same_source_provenance": {
            **same_source_provenance,
            "tomogram": same_source_provenance.get("tomogram") or tomogram,
            "map_metadata": map_metadata,
            "same_source_hash_identity_ok": same_source_hash_ok,
            "same_source_hash_identity": dict(same_source_hash_identity),
        },
        "remaining_gaps": list(report.get("remaining_gaps") or []),
        "source_report_ok": report.get("ok"),
        "source_report": str(report_path) if report_path is not None else "",
        "pct_saved_map_source_report": source_report,
    }


def _summarize_saved_map_relocalize_report(
    report: Mapping[str, Any],
    *,
    report_path: Path | None,
) -> dict[str, Any]:
    metadata_contract = _mapping(report.get("map_metadata_contract"))
    metadata_checks = _mapping(metadata_contract.get("checks"))
    service = _mapping(report.get("service"))
    live_feed = _mapping(report.get("live_feed"))
    outputs = _mapping(live_feed.get("outputs"))
    z_consistency = _mapping(live_feed.get("fastlio2_z_consistency"))
    localizer = _mapping(report.get("localizer"))
    thresholds = _mapping(report.get("thresholds"))

    min_points = _safe_int(thresholds.get("min_saved_map_points")) or 1000
    min_tracking = _safe_int(thresholds.get("min_tracking_health_samples")) or 1
    max_xy = _safe_float(thresholds.get("max_map_odom_xy_m")) or 5.0
    max_z = _safe_float(thresholds.get("max_map_odom_z_abs_m")) or 2.0
    map_xy = _safe_float(localizer.get("map_to_odom_xy_m"))
    map_z = _safe_float(localizer.get("map_to_odom_z_abs_m"))
    health_state = str(localizer.get("latest_health_state") or "").upper()
    global_reloc = report.get("global_relocalization_requested") is True

    metadata_ok = (
        metadata_contract.get("ok") is True
        and bool(metadata_checks)
        and all(value is True for value in metadata_checks.values())
    )
    live_feed_ok = (
        live_feed.get("ok") is True
        and z_consistency.get("ok") is True
        and _safe_int(outputs.get("fastlio2_odometry")) > 0
        and _safe_int(outputs.get("fastlio2_cloud_registered")) > 0
        and _safe_int(outputs.get("fastlio2_cloud_map")) > 0
    )
    service_ok = service.get("available") is True and (
        service.get("success") is True
        or (global_reloc and "already running" in str(service.get("message") or "").lower())
    )
    saved_map_cloud_ok = (
        _safe_int(localizer.get("saved_map_cloud_samples")) > 0
        and _safe_int(localizer.get("saved_map_cloud_points_latest")) >= min_points
    )
    health_ok = _safe_int(localizer.get("tracking_health_samples")) >= min_tracking and health_state in {
        "LOCKED",
        "RECOVERED",
    }
    tf_ok = _safe_int(localizer.get("map_to_odom_tf_samples")) > 0 and map_xy <= max_xy and map_z <= max_z
    global_reloc_ok = not global_reloc or (
        report.get("global_relocalization_validated") is True
        and localizer.get("bbs3d_success_observed") is True
        and localizer.get("bbs3d_disabled_observed") is not True
        and _safe_int(localizer.get("lost_health_samples")) > 0
    )
    edges = [
        _edge(
            "same_source_map_metadata_contract",
            metadata_ok,
            {
                "ok": metadata_contract.get("ok"),
                "path": metadata_contract.get("path") or "",
                "checks": metadata_checks,
                "blockers": list(metadata_contract.get("blockers") or []),
            },
        ),
        _edge(
            "live_fastlio_feed",
            live_feed_ok,
            {
                "ok": live_feed.get("ok"),
                "outputs": outputs,
                "fastlio2_z_consistency": z_consistency,
            },
        ),
        _edge(
            "relocalize_service",
            service_ok,
            {
                "available": service.get("available"),
                "success": service.get("success"),
                "message": service.get("message") or "",
                "global_relocalization_requested": global_reloc,
            },
        ),
        _edge(
            "saved_map_cloud",
            saved_map_cloud_ok,
            {
                "saved_map_cloud_samples": _safe_int(localizer.get("saved_map_cloud_samples")),
                "saved_map_cloud_points_latest": _safe_int(localizer.get("saved_map_cloud_points_latest")),
                "min_saved_map_points": min_points,
            },
        ),
        _edge(
            "localizer_health_lock",
            health_ok,
            {
                "latest_health_state": localizer.get("latest_health_state"),
                "tracking_health_samples": _safe_int(localizer.get("tracking_health_samples")),
                "min_tracking_health_samples": min_tracking,
            },
        ),
        _edge(
            "map_to_odom_correction",
            tf_ok,
            {
                "map_to_odom_tf_samples": _safe_int(localizer.get("map_to_odom_tf_samples")),
                "map_to_odom_xy_m": map_xy,
                "max_map_odom_xy_m": max_xy,
                "map_to_odom_z_abs_m": map_z,
                "max_map_odom_z_abs_m": max_z,
            },
        ),
        _edge(
            "global_relocalization",
            global_reloc_ok,
            {
                "global_relocalization_requested": global_reloc,
                "global_relocalization_validated": report.get("global_relocalization_validated"),
                "bbs3d_success_observed": localizer.get("bbs3d_success_observed"),
                "bbs3d_disabled_observed": localizer.get("bbs3d_disabled_observed"),
                "lost_health_samples": _safe_int(localizer.get("lost_health_samples")),
            },
        ),
    ]
    failed = [edge for edge in edges if edge["ok"] is not True]
    return {
        "checked": True,
        "ok": not failed,
        "schema_detected": "saved_map_relocalize",
        "primary_blocker": failed[0]["id"] if failed else "",
        "flow": edges,
        "source_report_ok": report.get("ok"),
        "source_report": str(report_path) if report_path is not None else "",
        "map_pcd": str(report.get("map_pcd") or ""),
        "map_metadata_contract": metadata_contract,
    }


def summarize_runtime_report(
    report: Mapping[str, Any],
    *,
    report_path: Path | None,
) -> dict[str, Any]:
    """Summarize one runtime or wrapper report into a dataflow verdict."""
    if _has_dynamic_obstacle_local_planner_shape(report):
        return _summarize_dynamic_obstacle_local_planner_report(
            report,
            report_path=report_path,
        )
    if _has_gazebo_runtime_shape(report):
        return _summarize_gazebo_runtime_report(
            report,
            report_path=report_path,
        )
    if _has_moving_obstacle_sweep_shape(report):
        return _summarize_moving_obstacle_sweep_report(
            report,
            report_path=report_path,
        )
    if _has_large_loop_closure_shape(report):
        return _summarize_large_loop_closure_report(
            report,
            report_path=report_path,
        )
    if _has_large_terrain_shape(report):
        return _summarize_large_terrain_report(
            report,
            report_path=report_path,
        )
    if _has_saved_map_relocalize_shape(report):
        return _summarize_saved_map_relocalize_report(
            report,
            report_path=report_path,
        )
    if _has_pct_saved_map_navigation_shape(report):
        return _summarize_pct_saved_map_navigation_report(
            report,
            report_path=report_path,
        )
    if _has_policy_nav_shape(report):
        return _summarize_policy_nav_report(
            report,
            report_path=report_path,
        )
    if _has_live_dataflow_shape(report):
        summary = summarize_live_report(report, report_path=report_path)
        summary["checked"] = True
        summary["schema_detected"] = "fastlio_live_navigation"
        return summary
    native_gate = _mapping(report.get("native_gate"))
    if native_gate:
        summary = _summarize_native_pct_report(
            native_gate,
            report_path=report_path,
        )
        summary["schema_detected"] = "embedded_native_pct_mujoco"
        return summary
    if _has_native_pct_shape(report):
        return _summarize_native_pct_report(report, report_path=report_path)
    return {
        "checked": False,
        "ok": False,
        "reason": "unsupported_report_shape",
        "schema_detected": str(report.get("schema_version") or ""),
        "source_report": str(report_path) if report_path is not None else "",
    }


def _summarize_aggregate_case(
    case: Mapping[str, Any],
    *,
    report_path: Path | None,
) -> dict[str, Any]:
    live = _mapping(case.get("live_nav_chain"))
    if _has_live_dataflow_shape(live):
        summary = summarize_runtime_report(live, report_path=report_path)
        summary["schema_detected"] = "embedded_live_nav_chain"
        return summary

    outputs = _mapping(_value_from(live.get("outputs"), case.get("outputs")))
    inspection = _mapping(_value_from(live.get("lingtu_inspection"), case.get("lingtu_inspection")))
    fastlio_consistency = _mapping(
        _value_from(
            live.get("fastlio2_motion_consistency"),
            case.get("fastlio2_consistency"),
            case.get("fastlio2_motion_consistency"),
        )
    )
    fastlio_path_length_m = _safe_float(
        _value_from(
            live.get("fastlio2_path_length_m"),
            case.get("fastlio2_path_length_m"),
            fastlio_consistency.get("fastlio2_path_length_m"),
        )
    )
    global_path_count = _safe_int(
        _value_from(
            outputs.get("global_path_count"),
            outputs.get("nav_global_path"),
            inspection.get("global_path_count"),
            case.get("global_path_count"),
        )
    )
    global_path_points_max = _safe_int(
        _value_from(
            inspection.get("global_path_points_max"),
            case.get("global_path_points_max"),
        )
    )
    local_path_count = _safe_int(
        _value_from(
            outputs.get("local_path_count"),
            outputs.get("nav_local_path"),
            inspection.get("local_path_count"),
            case.get("local_path_count"),
        )
    )
    local_path_points_max = _safe_int(
        _value_from(
            inspection.get("local_path_points_max"),
            case.get("local_path_points_max"),
        )
    )
    latest_local_path = _latest_local_path_status(live, inspection, case)
    historical_local_path_ok = local_path_count > 0 and local_path_points_max >= 2
    cmd_vel_nonzero = _safe_int(
        _value_from(
            outputs.get("nav_cmd_vel_nonzero"),
            live.get("nav_cmd_vel_nonzero"),
            case.get("nav_cmd_vel_nonzero"),
            case.get("cmd_vel_nonzero"),
        )
    )
    successful_checkpoints = _safe_int(
        _value_from(
            inspection.get("successful_navigation_goal_count"),
            case.get("successful_navigation_goal_count"),
        )
    )
    required_checkpoints = _safe_int(
        _value_from(
            inspection.get("min_required_checkpoints"),
            case.get("min_required_checkpoints"),
            inspection.get("goal_count"),
            case.get("goal_count"),
        )
    )
    if required_checkpoints <= 0:
        required_checkpoints = 1

    edges = [
        _edge(
            "fastlio_feedback",
            fastlio_path_length_m > 0 or fastlio_consistency.get("ok") is True,
            {
                "fastlio2_path_length_m": fastlio_path_length_m,
                "fastlio2_consistency": fastlio_consistency,
            },
        ),
        _edge(
            "global_path",
            global_path_count > 0 or global_path_points_max >= 2,
            {
                "global_path_count": global_path_count,
                "global_path_points_max": global_path_points_max,
                "global_planner": _value_from(
                    inspection.get("global_planner"),
                    case.get("global_planner"),
                ),
            },
        ),
        _edge(
            "local_path",
            historical_local_path_ok
            and (latest_local_path.get("checked") is not True or latest_local_path.get("ok") is True),
            {
                "local_path_count": local_path_count,
                "local_path_points_max": local_path_points_max,
                "latest_local_path": latest_local_path,
            },
        ),
        _edge(
            "cmd_vel",
            cmd_vel_nonzero > 0,
            {"nav_cmd_vel_nonzero": cmd_vel_nonzero},
        ),
        _edge(
            "checkpoint",
            successful_checkpoints >= required_checkpoints,
            {
                "successful_navigation_goal_count": successful_checkpoints,
                "min_required_checkpoints": required_checkpoints,
                "patrol_state": _value_from(
                    inspection.get("patrol_state"),
                    case.get("patrol_state"),
                ),
            },
        ),
    ]
    failed = [edge for edge in edges if edge["ok"] is not True]
    aggregate_report = {
        "navigation_chain": _value_from(
            live.get("navigation_chain"),
            case.get("navigation_chain"),
        ),
        "deliverable_contract": _value_from(
            live.get("deliverable_contract"),
            case.get("deliverable_contract"),
        ),
        "map_artifacts": _value_from(
            live.get("map_artifacts"),
            case.get("map_artifacts"),
        ),
        "assets": _value_from(live.get("assets"), case.get("assets")),
        "world": _value_from(live.get("world"), case.get("world")),
        "inspection_tomogram": _value_from(
            live.get("inspection_tomogram"),
            case.get("inspection_tomogram"),
            case.get("tomogram"),
        ),
        "world_parent": _value_from(
            live.get("world_parent"),
            case.get("world_parent"),
        ),
        "tomogram_parent": _value_from(
            live.get("tomogram_parent"),
            case.get("tomogram_parent"),
        ),
    }
    return {
        "checked": True,
        "ok": not failed,
        "schema_detected": "aggregate_case",
        "primary_blocker": failed[0]["id"] if failed else "",
        "flow": edges,
        "pct_provenance": _pct_provenance_from_live(aggregate_report, inspection),
        "same_source_provenance": _same_source_provenance_from_live(
            aggregate_report,
            inspection,
        ),
        "remaining_gaps": list(_value_from(live.get("remaining_gaps"), case.get("remaining_gaps")) or []),
        "source_report_ok": case.get("ok"),
        "source_report": str(report_path) if report_path is not None else "",
    }


def _case_has_dataflow_evidence(case: Mapping[str, Any]) -> bool:
    live = _mapping(case.get("live_nav_chain"))
    if _has_live_dataflow_shape(live):
        return True
    if isinstance(case.get("outputs"), Mapping) or isinstance(
        case.get("lingtu_inspection"),
        Mapping,
    ):
        return True
    keys = {
        "fastlio2_path_length_m",
        "fastlio2_consistency",
        "global_path_count",
        "global_path_points_max",
        "local_path_count",
        "local_path_points_max",
        "local_path_points_latest",
        "navigation_diagnostics",
        "path_diagnostics",
        "autonomy_chain",
        "nav_cmd_vel_nonzero",
        "cmd_vel_nonzero",
        "successful_navigation_goal_count",
        "min_required_checkpoints",
        "goal_count",
    }
    return any(key in case for key in keys)


def _select_embedded_dataflow_case(report: Mapping[str, Any]) -> dict[str, Any]:
    for key in ("minimal_red_defect", "best_case"):
        case = _mapping(report.get(key))
        if _case_has_dataflow_evidence(case):
            return case
    cases = [case for case in report.get("cases") or [] if isinstance(case, Mapping)]
    failed_cases = [case for case in cases if case.get("ok") is not True]
    for case in [*failed_cases, *cases]:
        if _case_has_dataflow_evidence(case):
            return dict(case)
    return {}


def _select_child_report_path(
    report: Mapping[str, Any],
    *,
    root: Path,
) -> Path | None:
    for key in ("minimal_red_defect", "best_case"):
        path = _repo_path(_mapping(report.get(key)).get("path"), root=root)
        if path is not None:
            return path
    cases = [case for case in report.get("cases") or [] if isinstance(case, Mapping)]
    failed_cases = [case for case in cases if case.get("ok") is not True]
    for case in [*failed_cases, *cases]:
        path = _repo_path(case.get("path"), root=root)
        if path is not None:
            return path
    return None


def _next_actions_by_gate(summary: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    validation = _mapping(summary.get("algorithm_validation"))
    return {
        str(action["gate"]): dict(action)
        for action in validation.get("next_actions") or []
        if isinstance(action, Mapping) and action.get("gate")
    }


def runtime_dataflow_for_gate(
    gate_name: str,
    gate: Mapping[str, Any],
    action: Mapping[str, Any],
    *,
    root: Path,
) -> dict[str, Any]:
    """Summarize runtime dataflow evidence for one gate report."""
    gate_report_path = _repo_path(
        gate.get("path") or action.get("report_path") or action.get("expected_report_path"),
        root=root,
    )
    if gate_report_path is None:
        return {
            "checked": False,
            "ok": False,
            "reason": "report_missing",
            "source_gate_report": "",
            "source_report": "",
            "candidate_reports": [],
        }
    matched_report_path = _latest_report_path(_glob_report_paths(gate_report_path))
    if matched_report_path is not None:
        gate_report_path = matched_report_path
    if not gate_report_path.is_file():
        candidate_reports = _missing_report_candidate_diagnostics(
            gate_name,
            gate_report_path,
            root=root,
        )
        return {
            "checked": False,
            "ok": False,
            "reason": "report_missing",
            "source_gate_report": str(gate_report_path),
            "source_report": str(gate_report_path),
            "candidate_reports": candidate_reports,
        }
    try:
        gate_report = _load_json(gate_report_path)
    except Exception as exc:  # pragma: no cover - CLI diagnostics path.
        return {
            "checked": False,
            "ok": False,
            "reason": f"report_unreadable: {exc}",
            "source_gate_report": str(gate_report_path),
            "source_report": str(gate_report_path),
        }

    runtime_report_path = gate_report_path
    runtime_report = gate_report
    if not _has_live_dataflow_shape(gate_report) and not _has_native_pct_shape(gate_report):
        child_path = _select_child_report_path(gate_report, root=root)
        if child_path is not None:
            runtime_report_path = child_path
            if child_path.is_file():
                try:
                    runtime_report = _load_json(child_path)
                except Exception as exc:  # pragma: no cover - CLI diagnostics path.
                    return {
                        "checked": False,
                        "ok": False,
                        "reason": f"child_report_unreadable: {exc}",
                        "source_gate_report": str(gate_report_path),
                        "source_report": str(child_path),
                    }
                child_freshness = _child_report_freshness(
                    parent_path=gate_report_path,
                    parent_report=gate_report,
                    child_path=child_path,
                    child_report=runtime_report,
                )
                if child_freshness.get("fresh") is not True:
                    return {
                        "checked": False,
                        "ok": False,
                        "reason": "child_report_stale",
                        "source_gate_report": str(gate_report_path),
                        "source_report": str(child_path),
                        "child_report": str(child_path),
                        "child_report_freshness": child_freshness,
                    }
            else:
                embedded = _select_embedded_dataflow_case(gate_report)
                if embedded:
                    summary = _summarize_aggregate_case(
                        embedded,
                        report_path=gate_report_path,
                    )
                    summary["source_gate_report"] = str(gate_report_path)
                    summary["source_report"] = str(gate_report_path)
                    summary["child_report"] = str(child_path)
                    summary["child_reason"] = "child_report_missing"
                    return summary
                return {
                    "checked": False,
                    "ok": False,
                    "reason": "child_report_missing",
                    "source_gate_report": str(gate_report_path),
                    "source_report": str(child_path),
                }
        else:
            embedded = _select_embedded_dataflow_case(gate_report)
            if embedded:
                summary = _summarize_aggregate_case(
                    embedded,
                    report_path=gate_report_path,
                )
                summary["source_gate_report"] = str(gate_report_path)
                summary["source_report"] = str(gate_report_path)
                return summary

    summary = summarize_runtime_report(runtime_report, report_path=runtime_report_path)
    summary["source_gate_report"] = str(gate_report_path)
    if not summary.get("source_report"):
        summary["source_report"] = str(runtime_report_path)
    return summary


def build_runtime_dataflow_from_summary(
    summary: Mapping[str, Any],
    *,
    root: Path,
    gates: set[str] | None = None,
) -> dict[str, Any]:
    """Build gate-keyed runtime dataflow from an existing closure summary."""
    summary_gates = _mapping(summary.get("gates"))
    next_actions = _next_actions_by_gate(summary)
    selected_gates = gates or RUNTIME_DATAFLOW_GATES
    return {
        gate_name: runtime_dataflow_for_gate(
            gate_name,
            _mapping(summary_gates.get(gate_name)),
            next_actions.get(gate_name, {}),
            root=root,
        )
        for gate_name in sorted(selected_gates)
    }

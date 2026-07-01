#!/usr/bin/env python3
"""Runtime gate for CMU Unity + LingTu external TARE simulation.

The preflight gate verifies files and remap contracts. This gate attaches to a
running isolated ROS 2 domain and verifies the actual closed loop:

* CMU or LingTu publishes real waypoint samples.
* LingTu publishes non-zero /nav/cmd_vel.
* Odom moves.
* Terrain/scan/map point coverage grows.
* No hardware-looking node subscribes to command topics.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import struct
import time
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


SCHEMA_VERSION = "lingtu.cmu_unity_runtime_gate.v1"

HARDWARE_NODE_TOKENS = (
    "thunder",
    "s100p",
    "han_dog",
    "dog_driver",
    "brainstem",
    "unitree",
)

COMMAND_TOPICS = ("/cmd_vel", "/nav/cmd_vel")
WAYPOINT_TOPICS = ("/way_point", "/exploration/way_point", "/nav/way_point")
ODOM_TOPICS = ("/slam/odometry", "/state_estimation")
START_TOPICS = ("/exploration/start", "/start_exploration")
PATH_TOPICS = (
    "/exploration/global_path_full",
    "/exploration/global_path",
    "/exploration/local_path",
    "/exploration/path",
    "/nav/global_path",
    "/nav/local_path",
)
CLOUD_TOPICS = (
    "/terrain_map",
    "/terrain_map_ext",
    "/registered_scan",
    "/nav/terrain_map",
    "/nav/terrain_map_ext",
    "/slam/registered_cloud",
    "/slam/map_cloud",
)
DEFAULT_REQUIRED_MAP_TOPICS = ("/nav/terrain_map_ext",)
DEFAULT_REQUIRED_SCAN_TOPICS = ("/registered_scan", "/slam/registered_cloud")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {"available": False, "path": str(path), "error": str(exc)}


def _same_source_tomogram_evidence() -> dict[str, Any]:
    tomogram_raw = os.environ.get("LINGTU_CMU_TOMOGRAM", "")
    run_dir_raw = os.environ.get("LINGTU_CMU_RUN_DIR", "")
    tomogram_path = Path(tomogram_raw) if tomogram_raw else None
    capture_path = (
        Path(run_dir_raw) / "cmu_unity_tomogram_capture.json"
        if run_dir_raw
        else None
    )
    evidence: dict[str, Any] = {
        "available": False,
        "ok": False,
        "tomogram": tomogram_raw,
        "tomogram_exists": bool(tomogram_path is not None and tomogram_path.exists()),
        "tomogram_sha256": (
            _sha256_file(tomogram_path)
            if tomogram_path is not None and tomogram_path.exists()
            else ""
        ),
        "capture_report": str(capture_path) if capture_path is not None else "",
        "capture_report_exists": bool(capture_path is not None and capture_path.exists()),
        "capture_ok": False,
        "same_source_tomogram": False,
        "errors": [],
    }
    if tomogram_path is None:
        evidence["errors"].append("LINGTU_CMU_TOMOGRAM is not set")
        return evidence
    if not evidence["tomogram_exists"]:
        evidence["errors"].append("same-source tomogram file missing")
        return evidence
    if capture_path is None:
        evidence["errors"].append("LINGTU_CMU_RUN_DIR is not set")
        return evidence
    if not capture_path.exists():
        evidence["errors"].append("cmu_unity_tomogram_capture.json missing")
        return evidence
    capture = _load_json(capture_path)
    if capture.get("available") is False and capture.get("error"):
        evidence["errors"].append(f"capture report unreadable: {capture.get('error')}")
        return evidence
    evidence["available"] = True
    evidence["capture_ok"] = capture.get("ok") is True
    source_contract = capture.get("source_contract") or {}
    artifacts = capture.get("artifacts") or {}
    tomo = artifacts.get("tomogram") or capture.get("tomogram_build") or {}
    pcd = artifacts.get("pcd") or {}
    evidence["pcd"] = pcd
    evidence["tomogram_artifact"] = tomo
    evidence["source_contract"] = source_contract
    evidence["same_source_tomogram"] = bool(
        source_contract.get("same_source_tomogram") is True
        and tomo.get("same_source_input") is True
        and str(tomo.get("input_pcd_sha256") or "")
        == str(pcd.get("sha256") or "")
        and str(tomo.get("sha256") or "") == evidence["tomogram_sha256"]
    )
    if not evidence["capture_ok"]:
        evidence["errors"].append("tomogram capture report ok is not true")
    if not evidence["same_source_tomogram"]:
        evidence["errors"].append("tomogram is not proven same-source with captured PCD")
    evidence["ok"] = not evidence["errors"]
    return evidence


def _finite(value: float) -> bool:
    return math.isfinite(float(value))


def _norm_cmd(msg: Any) -> float:
    twist = getattr(msg, "twist", msg)
    linear = getattr(twist, "linear", None)
    angular = getattr(twist, "angular", None)
    return (
        abs(float(getattr(linear, "x", 0.0)))
        + abs(float(getattr(linear, "y", 0.0)))
        + abs(float(getattr(angular, "z", 0.0)))
    )


def _xy_from_odom(msg: Any) -> tuple[float, float] | None:
    try:
        p = msg.pose.pose.position
        x = float(p.x)
        y = float(p.y)
    except Exception:
        return None
    if not (_finite(x) and _finite(y)):
        return None
    return x, y


def _xy_from_waypoint(msg: Any) -> tuple[float, float, float, str] | None:
    try:
        p = msg.point
        x = float(p.x)
        y = float(p.y)
        z = float(p.z)
        frame = str(msg.header.frame_id or "")
    except Exception:
        return None
    if not (_finite(x) and _finite(y) and _finite(z)):
        return None
    return x, y, z, frame


def _point_field_offsets(msg: Any) -> tuple[int, int] | None:
    offsets: dict[str, int] = {}
    for field in getattr(msg, "fields", []) or []:
        name = getattr(field, "name", "")
        if name in {"x", "y"}:
            offsets[name] = int(getattr(field, "offset", -1))
    if offsets.get("x", -1) < 0 or offsets.get("y", -1) < 0:
        return None
    return offsets["x"], offsets["y"]


def _iter_xy_points(msg: Any, *, max_points: int = 6000):
    offsets = _point_field_offsets(msg)
    if offsets is None:
        return
    x_offset, y_offset = offsets
    step = int(getattr(msg, "point_step", 0) or 0)
    if step <= 0:
        return
    data = bytes(getattr(msg, "data", b"") or b"")
    count = len(data) // step
    if count <= 0:
        return
    stride = max(1, count // max_points)
    endian = ">" if bool(getattr(msg, "is_bigendian", False)) else "<"
    for index in range(0, count, stride):
        base = index * step
        try:
            x = struct.unpack_from(endian + "f", data, base + x_offset)[0]
            y = struct.unpack_from(endian + "f", data, base + y_offset)[0]
        except Exception:
            continue
        if _finite(x) and _finite(y):
            yield float(x), float(y)


def _parse_required_map_topics(args: argparse.Namespace) -> dict[str, float]:
    specs = list(getattr(args, "required_map_topics", None) or DEFAULT_REQUIRED_MAP_TOPICS)
    default_area = float(getattr(args, "min_required_map_topic_area_m2", None) or args.min_map_area_delta_m2)
    parsed: dict[str, float] = {}
    for spec in specs:
        topic = str(spec).strip()
        required_area = default_area
        if not topic:
            continue
        if ":" in topic:
            topic, area_text = topic.rsplit(":", 1)
            topic = topic.strip()
            try:
                required_area = float(area_text)
            except ValueError:
                required_area = default_area
        if topic:
            parsed[topic] = required_area
    return parsed


def _parse_required_scan_topics(args: argparse.Namespace) -> list[str]:
    topics = list(getattr(args, "required_scan_topics", None) or DEFAULT_REQUIRED_SCAN_TOPICS)
    return [str(topic).strip() for topic in topics if str(topic).strip()]


def _window_start(metrics: dict[str, Any], window_sec: float) -> float:
    duration = float(metrics.get("duration_sec") or 0.0)
    return max(0.0, duration - max(0.0, float(window_sec)))


def _history_path_delta_m(history: list[dict[str, Any]], *, start_time_sec: float) -> float:
    points: list[tuple[float, float]] = []
    for sample in history:
        try:
            t = float(sample.get("t") or 0.0)
            xy = sample.get("xy") or []
            x = float(xy[0])
            y = float(xy[1])
        except Exception:
            continue
        if t >= start_time_sec and _finite(x) and _finite(y):
            points.append((x, y))
    if len(points) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(points, points[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


def _late_odom_report(metrics: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    window_sec = float(getattr(args, "late_window_sec", 0.0) or 0.0)
    required = float(getattr(args, "min_late_odom_delta_m", 0.0) or 0.0)
    start_time = _window_start(metrics, window_sec)
    topics: dict[str, Any] = {}
    best_delta = 0.0
    for topic, item in (metrics.get("odometry") or {}).items():
        delta = _history_path_delta_m(list((item or {}).get("history") or []), start_time_sec=start_time)
        best_delta = max(best_delta, delta)
        topics[topic] = {"delta_m": delta, "samples": len((item or {}).get("history") or [])}
    return {
        "window_sec": window_sec,
        "required_delta_m": required,
        "observed_best_delta_m": best_delta,
        "topics": topics,
        "ok": best_delta >= required,
    }


def _late_cmd_report(metrics: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    window_sec = float(getattr(args, "late_window_sec", 0.0) or 0.0)
    required = int(getattr(args, "min_late_cmd_vel_samples", 0) or 0)
    start_time = _window_start(metrics, window_sec)
    times = [
        float(t)
        for t in ((metrics.get("cmd_vel") or {}).get("nonzero_times_sec") or [])
        if float(t) >= start_time
    ]
    return {
        "window_sec": window_sec,
        "required_nonzero_samples": required,
        "observed_nonzero_samples": len(times),
        "ok": len(times) >= required,
    }


def _late_map_report(metrics: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    window_sec = float(getattr(args, "late_window_sec", 0.0) or 0.0)
    required = float(getattr(args, "min_late_map_area_delta_m2", 0.0) or 0.0)
    start_time = _window_start(metrics, window_sec)
    topics: dict[str, Any] = {}
    best_delta = 0.0
    for topic, item in ((metrics.get("cloud_coverage") or {}).get("topics") or {}).items():
        history = [
            sample
            for sample in list((item or {}).get("history") or [])
            if float(sample.get("t") or 0.0) >= start_time
        ]
        if len(history) < 2:
            delta = 0.0
        else:
            delta = max(0.0, float(history[-1].get("area_m2") or 0.0) - float(history[0].get("area_m2") or 0.0))
        best_delta = max(best_delta, delta)
        topics[topic] = {"area_delta_m2": delta, "samples": len(history)}
    return {
        "window_sec": window_sec,
        "required_area_delta_m2": required,
        "observed_best_area_delta_m2": best_delta,
        "topics": topics,
        "ok": best_delta >= required,
    }


def _late_path_report(
    metrics: dict[str, Any],
    args: argparse.Namespace,
    *,
    path_topics: list[str],
) -> dict[str, Any]:
    window_sec = float(getattr(args, "late_window_sec", 0.0) or 0.0)
    required = int(getattr(args, "min_late_path_samples", 0) or 0)
    start_time = _window_start(metrics, window_sec)
    topics: dict[str, Any] = {}
    ok = True
    for topic in path_topics:
        item = (metrics.get("paths") or {}).get(topic) or {}
        times = [
            float(t)
            for t in (item.get("nonempty_times_sec") or [])
            if float(t) >= start_time
        ]
        topic_ok = len(times) >= required
        ok = ok and topic_ok
        topics[topic] = {
            "required_nonempty_samples": required,
            "observed_nonempty_samples": len(times),
            "last_count": int(item.get("last_count") or 0),
            "max_poses": int(item.get("max_poses") or 0),
            "ok": topic_ok,
        }
    return {
        "window_sec": window_sec,
        "topics": topics,
        "ok": ok,
    }


def _frontier_no_gain_stall_report(
    metrics: dict[str, Any],
    args: argparse.Namespace,
    *,
    late_activity: dict[str, Any],
    tare_navigation: dict[str, Any],
) -> dict[str, Any]:
    checked = bool(getattr(args, "require_frontier_no_gain_stall", False))
    window_sec = float(getattr(args, "late_window_sec", 0.0) or 0.0)
    duration_sec = float(metrics.get("duration_sec") or 0.0)
    observed_s = max(0.0, min(duration_sec, window_sec)) if window_sec > 0.0 else 0.0
    blockers: list[str] = []
    if checked:
        if window_sec <= 0.0:
            blockers.append("late_window_sec <= 0")
        if duration_sec < window_sec:
            blockers.append("duration below required late observation window")
        for name in ("odometry", "cmd_vel", "paths"):
            evidence = late_activity.get(name)
            if not isinstance(evidence, dict):
                blockers.append(f"late {name} evidence missing")
            elif evidence.get("ok") is not True:
                blockers.append(f"late {name} evidence is not ok")
        map_growth = late_activity.get("map_growth")
        if not isinstance(map_growth, dict):
            blockers.append("late map_growth evidence missing")
        elif not (
            map_growth.get("ok") is True
            or map_growth.get("accepted_flat_after_total_growth") is True
        ):
            blockers.append("late map_growth evidence is not ok")
        if int(tare_navigation.get("failure_count") or 0) > 0:
            blockers.append("TARE navigation failure_count is nonzero")

    return {
        "checked": checked,
        "ok": checked and not blockers,
        "mode": "late_activity_observation",
        "stop_reason": (
            "late_activity_window_verified"
            if checked and not blockers
            else "late_activity_window_failed"
            if checked
            else "not_required"
        ),
        "required_observation_s": round(window_sec, 3) if checked else 0.0,
        "observed_s": round(observed_s, 3),
        "duration_sec": round(duration_sec, 3),
        "late_activity": {
            "odometry_ok": (late_activity.get("odometry") or {}).get("ok") is True,
            "cmd_vel_ok": (late_activity.get("cmd_vel") or {}).get("ok") is True,
            "paths_ok": (late_activity.get("paths") or {}).get("ok") is True,
            "map_growth_ok": (late_activity.get("map_growth") or {}).get("ok") is True,
            "map_growth_accepted_flat_after_total_growth": (
                (late_activity.get("map_growth") or {}).get("accepted_flat_after_total_growth")
                is True
            ),
        },
        "tare_navigation": {
            "available": tare_navigation.get("available"),
            "success_count": int(tare_navigation.get("success_count") or 0),
            "failure_count": int(tare_navigation.get("failure_count") or 0),
            "terminal_count": int(tare_navigation.get("terminal_count") or 0),
        },
        "blockers": blockers,
    }


def _fetch_gateway_navigation_status(gateway_url: str, timeout_sec: float) -> dict[str, Any]:
    """Fetch Gateway navigation diagnostics without making Gateway a hard dependency."""
    url = str(gateway_url or "").rstrip("/")
    if not url:
        return {"available": False, "reason": "gateway_url_disabled"}
    try:
        with urlopen(f"{url}/api/v1/navigation/status", timeout=float(timeout_sec)) as response:
            body = response.read()
    except URLError as exc:
        return {"available": False, "reason": f"{type(exc).__name__}: {exc}"}
    except Exception as exc:
        return {"available": False, "reason": f"{type(exc).__name__}: {exc}"}
    try:
        data = json.loads(body.decode("utf-8"))
    except Exception as exc:
        return {"available": False, "reason": f"invalid_json: {exc}"}
    if not isinstance(data, dict):
        return {"available": False, "reason": "navigation_status_not_object"}
    return {"available": True, "url": f"{url}/api/v1/navigation/status", "data": data}


def _fetch_gateway_exploration_status(gateway_url: str, timeout_sec: float) -> dict[str, Any]:
    """Fetch Gateway exploration diagnostics without making Gateway a hard dependency."""
    url = str(gateway_url or "").rstrip("/")
    if not url:
        return {"available": False, "reason": "gateway_url_disabled"}
    try:
        with urlopen(f"{url}/api/v1/explore/status", timeout=float(timeout_sec)) as response:
            raw = response.read()
        data = json.loads(raw.decode("utf-8"))
    except Exception as exc:
        return {"available": False, "reason": str(exc)}
    if not isinstance(data, dict):
        return {"available": False, "reason": "exploration_status_not_object"}
    return {"available": True, "url": f"{url}/api/v1/explore/status", "data": data}


def _post_gateway_json(
    gateway_url: str,
    path: str,
    payload: dict[str, Any],
    timeout_sec: float,
) -> dict[str, Any]:
    """POST to Gateway from the gate after ROS subscriptions are ready."""
    base_url = str(gateway_url or "").rstrip("/")
    if not base_url:
        return {"ok": False, "reason": "gateway_url_disabled"}
    url = f"{base_url}{path}"
    body = json.dumps(payload).encode("utf-8")
    request = Request(
        url,
        data=body,
        headers={"content-type": "application/json"},
        method="POST",
    )
    try:
        with urlopen(request, timeout=float(timeout_sec)) as response:
            response_body = response.read()
            status = int(getattr(response, "status", 0) or 0)
    except HTTPError as exc:
        status = int(getattr(exc, "code", 0) or 0)
        body = b""
        try:
            body = exc.read()
        except Exception:
            body = b""
        text = body.decode("utf-8", errors="replace") if body else ""
        if status == 409:
            try:
                data: Any = json.loads(text) if text else {}
            except Exception:
                data = {"raw": text}
            return {
                "ok": True,
                "url": url,
                "status": status,
                "already_active": True,
                "data": data,
            }
        return {
            "ok": False,
            "url": url,
            "status": status,
            "reason": f"{type(exc).__name__}: HTTP {status}",
            "body": text,
        }
    except URLError as exc:
        return {"ok": False, "url": url, "reason": f"{type(exc).__name__}: {exc}"}
    except Exception as exc:
        return {"ok": False, "url": url, "reason": f"{type(exc).__name__}: {exc}"}
    try:
        data: Any = json.loads(response_body.decode("utf-8"))
    except Exception as exc:
        return {
            "ok": False,
            "url": url,
            "status": status,
            "reason": f"invalid_json: {exc}",
        }
    ok = True
    if isinstance(data, dict) and "ok" in data:
        ok = bool(data.get("ok"))
    return {"ok": ok, "url": url, "status": status, "data": data}


def _planner_diagnostics_from_gateway(status: dict[str, Any]) -> dict[str, Any]:
    if not status.get("available"):
        return {"available": False, "reason": status.get("reason", "gateway_unavailable")}
    data = status.get("data") or {}
    diagnostics = data.get("diagnostics") or {}
    last_plan = diagnostics.get("last_plan_report") or {}
    if not isinstance(last_plan, dict) or not last_plan:
        return {"available": False, "reason": "last_plan_report_missing", "gateway_available": True}

    selected = last_plan.get("selected_planner")
    primary = last_plan.get("primary_planner") or last_plan.get("planner")
    primary_replan = last_plan.get("primary_replan") if isinstance(last_plan.get("primary_replan"), dict) else {}
    primary_replan_used = bool(primary_replan and primary_replan.get("used", True))
    rejected = list(last_plan.get("rejected_plans") or [])
    fallback_reason = str(last_plan.get("fallback_reason") or "")
    fallback_used = bool(primary and selected and selected != primary)
    selected_path_safety = (
        last_plan.get("selected_path_safety")
        if isinstance(last_plan.get("selected_path_safety"), dict)
        else {}
    )
    path_safety_ok = selected_path_safety.get("ok")
    if path_safety_ok is not None:
        path_safety_ok = bool(path_safety_ok)
    return {
        "available": True,
        "primary_planner": primary,
        "selected_planner": selected,
        "policy": last_plan.get("policy") or diagnostics.get("plan_safety_policy"),
        "fallback_used": fallback_used,
        "fallback_reason": fallback_reason,
        "unsafe_primary_rejected": bool(fallback_reason or rejected),
        "primary_replan_used": primary_replan_used,
        "primary_replan_reason": str(primary_replan.get("reason") or ""),
        "rejected_plan_count": len(rejected),
        "path_safety_ok": path_safety_ok,
        "path_safety_blocked_sample_count": int(selected_path_safety.get("blocked_sample_count") or 0),
        "reached_goal": last_plan.get("reached_goal"),
        "last_plan_report": last_plan,
    }


def _navigation_failure_from_gateway(status: dict[str, Any]) -> dict[str, Any]:
    if not status.get("available"):
        return {"failed": False, "reason": "gateway_unavailable"}
    data = status.get("data") or {}
    if not isinstance(data, dict):
        return {"failed": False, "reason": "navigation_status_not_object"}
    state = str(data.get("state") or "").upper()
    reason_codes = [str(code) for code in (data.get("reason_codes") or [])]
    failure_reason = str(data.get("failure_reason") or "")
    failed = (
        state in {"FAILED", "STUCK"}
        or "mission_failed" in reason_codes
        or "mission_stuck" in reason_codes
        or "failure_stuck_after_max_replans" in reason_codes
    )
    return {
        "failed": failed,
        "state": state,
        "reason_codes": reason_codes,
        "failure_reason": failure_reason,
    }


def _direct_goal_fallback_from_gateway(status: dict[str, Any]) -> dict[str, Any]:
    if not status.get("available"):
        return {"used": False, "reason": "gateway_unavailable"}
    data = status.get("data") or {}
    mission = data.get("mission") if isinstance(data, dict) else {}
    raw = mission.get("raw") if isinstance(mission, dict) else {}
    fallback = raw.get("direct_goal_fallback") if isinstance(raw, dict) else {}
    if not isinstance(fallback, dict):
        fallback = {}
    return {
        "used": bool(fallback.get("used")),
        "reason": str(fallback.get("reason") or ""),
        "goal": fallback.get("goal"),
        "ts": fallback.get("ts"),
    }


def _tare_navigation_from_gateway_exploration(status: dict[str, Any]) -> dict[str, Any]:
    if not status.get("available"):
        return {"available": False, "reason": status.get("reason", "gateway_unavailable")}
    data = status.get("data") or {}
    if not isinstance(data, dict):
        return {"available": False, "reason": "exploration_status_not_object"}
    tare = data.get("tare") if isinstance(data.get("tare"), dict) else {}
    raw_status = tare.get("status") if isinstance(tare, dict) else {}
    stats = tare.get("stats") if isinstance(tare, dict) else {}
    if not isinstance(raw_status, dict):
        raw_status = {}
    if not isinstance(stats, dict):
        stats = {}

    def _counter(name: str) -> int:
        for source in (stats, raw_status):
            if name not in source:
                continue
            try:
                return max(0, int(source.get(name) or 0))
            except (TypeError, ValueError):
                continue
        return 0
    last_navigation_status = stats.get("last_navigation_status")
    if not isinstance(last_navigation_status, dict) or not last_navigation_status:
        last_navigation_status = raw_status.get("last_navigation_status")
    if not isinstance(last_navigation_status, dict):
        last_navigation_status = {}

    return {
        "available": True,
        "backend": str(data.get("backend") or ""),
        "started": bool(raw_status.get("started") or data.get("exploring")),
        "success_count": _counter("navigation_success_count"),
        "failure_count": _counter("navigation_failure_count"),
        "terminal_count": _counter("navigation_terminal_count"),
        "last_navigation_status": last_navigation_status,
    }


def _tare_strategy_quality_from_gateway_exploration(
    status: dict[str, Any],
    args: argparse.Namespace,
) -> dict[str, Any]:
    min_waypoints = max(0, int(getattr(args, "min_tare_waypoints", 1) or 0))
    min_paths = max(0, int(getattr(args, "min_tare_paths", 1) or 0))
    min_strategy_paths = max(0, int(getattr(args, "min_tare_strategy_paths", 0) or 0))
    min_successes = max(
        0,
        int(
            getattr(
                args,
                "min_tare_navigation_successes",
                getattr(args, "min_exploration_navigation_successes", 1),
            )
            or 0
        ),
    )
    try:
        max_suppressed_ratio = float(
            getattr(args, "max_tare_suppressed_waypoint_ratio", 1.0)
        )
    except (TypeError, ValueError):
        max_suppressed_ratio = 1.0
    max_suppressed_ratio = min(1.0, max(0.0, max_suppressed_ratio))
    thresholds = {
        "min_tare_waypoints": min_waypoints,
        "min_tare_paths": min_paths,
        "min_tare_strategy_paths": min_strategy_paths,
        "min_tare_navigation_successes": min_successes,
        "max_tare_suppressed_waypoint_ratio": max_suppressed_ratio,
    }

    if not status.get("available"):
        return {
            "checked": False,
            "ok": False,
            "available": False,
            "reason": status.get("reason", "gateway_unavailable"),
            "thresholds": thresholds,
            "blockers": ["Gateway exploration status unavailable"],
        }
    data = status.get("data") or {}
    if not isinstance(data, dict):
        return {
            "checked": False,
            "ok": False,
            "available": False,
            "reason": "exploration_status_not_object",
            "thresholds": thresholds,
            "blockers": ["Gateway exploration status is not an object"],
        }

    tare = data.get("tare") if isinstance(data.get("tare"), dict) else {}
    raw_status = tare.get("status") if isinstance(tare, dict) else {}
    stats = tare.get("stats") if isinstance(tare, dict) else {}
    if not isinstance(raw_status, dict):
        raw_status = {}
    if not isinstance(stats, dict):
        stats = {}

    def _counter(name: str) -> int:
        for source in (stats, raw_status):
            if name not in source:
                continue
            try:
                return max(0, int(source.get(name) or 0))
            except (TypeError, ValueError):
                continue
        return 0

    def _text(name: str) -> str:
        for source in (stats, raw_status):
            value = source.get(name)
            if value is None:
                continue
            text = str(value).strip()
            if text:
                return text
        return ""

    waypoint_count = _counter("waypoint_count")
    path_count = _counter("path_count")
    strategy_path_count = _counter("strategy_path_count")
    suppressed_waypoint_count = _counter("suppressed_waypoint_count")
    suppressed_far_waypoint_count = _counter("suppressed_far_waypoint_count")
    navigation_terminal_count = _counter("navigation_terminal_count")
    navigation_success_count = _counter("navigation_success_count")
    navigation_failure_count = _counter("navigation_failure_count")
    suppressed_total = suppressed_waypoint_count + suppressed_far_waypoint_count
    suppression_denominator = waypoint_count + suppressed_total
    suppressed_ratio = (
        float(suppressed_total) / float(suppression_denominator)
        if suppression_denominator > 0
        else 0.0
    )
    reject_reasons = [
        reason
        for reason in (
            _text("last_waypoint_reject_reason"),
            _text("last_strategy_path_reject_reason"),
        )
        if reason
    ]

    blockers: list[str] = []
    if waypoint_count < min_waypoints:
        blockers.append("waypoint_count below threshold")
    if path_count < min_paths:
        blockers.append("path_count below threshold")
    if strategy_path_count < min_strategy_paths:
        blockers.append("strategy_path_count below threshold")
    if suppressed_ratio > max_suppressed_ratio:
        blockers.append("suppressed_waypoint_ratio above threshold")
    if navigation_success_count < min_successes:
        blockers.append("navigation_success_count below threshold")
    if navigation_failure_count > 0:
        blockers.append("navigation_failure_count is nonzero")

    return {
        "checked": True,
        "ok": not blockers,
        "available": True,
        "source": "gateway_exploration_status.tare.stats" if stats else "gateway_exploration_status.tare.status",
        "backend": str(data.get("backend") or ""),
        "started": bool(raw_status.get("started") or data.get("exploring")),
        "thresholds": thresholds,
        "waypoint_count": waypoint_count,
        "path_count": path_count,
        "strategy_path_count": strategy_path_count,
        "suppressed_waypoint_count": suppressed_waypoint_count,
        "suppressed_far_waypoint_count": suppressed_far_waypoint_count,
        "suppressed_waypoint_ratio": suppressed_ratio,
        "navigation_terminal_count": navigation_terminal_count,
        "navigation_success_count": navigation_success_count,
        "navigation_failure_count": navigation_failure_count,
        "reject_reasons": reject_reasons,
        "blockers": blockers,
    }


def _gateway_session_start_ok(session_start: dict[str, Any]) -> bool:
    if session_start.get("ok"):
        return True
    try:
        if int(session_start.get("status")) == 409:
            return True
    except Exception:
        pass
    reason = str(session_start.get("reason") or "")
    return "409" in reason and "Conflict" in reason


def _post_gateway_json_with_retry(
    gateway_url: str,
    path: str,
    payload: dict[str, Any],
    timeout_sec: float,
    wait_sec: float,
    retry_interval_sec: float = 0.5,
) -> dict[str, Any]:
    """POST to Gateway, allowing startup races with the LingTu stack."""
    attempts: list[dict[str, Any]] = []
    deadline = time.time() + max(0.0, float(wait_sec))
    while True:
        result = _post_gateway_json(gateway_url, path, payload, timeout_sec)
        attempts.append(dict(result))
        if _gateway_session_start_ok(result) or time.time() >= deadline:
            final = dict(result)
            final["attempts"] = attempts
            final["attempt_count"] = len(attempts)
            return final
        time.sleep(max(0.05, float(retry_interval_sec)))


class RuntimeSampler:
    def __init__(self, node: Any, *, min_cmd_vel: float, voxel_size: float) -> None:
        self.node = node
        self.min_cmd_vel = min_cmd_vel
        self.voxel_size = voxel_size
        self.start_time = time.time()
        self.waypoints: dict[str, dict[str, Any]] = {}
        self.paths: dict[str, dict[str, Any]] = {}
        self.cmd_vel = {
            "samples": 0,
            "nonzero_samples": 0,
            "max_norm": 0.0,
            "max_abs_linear_x": 0.0,
            "max_abs_linear_y": 0.0,
            "max_abs_angular_z": 0.0,
            "last_nonzero": None,
            "nonzero_times_sec": [],
        }
        self.odometry: dict[str, dict[str, Any]] = {}
        self.clouds: dict[str, dict[str, Any]] = {}
        self._latest_odom_xy: tuple[float, float] | None = None
        self._active_waypoint: dict[str, tuple[float, float]] = {}
        self.motion_progress: dict[str, dict[str, Any]] = {}

    def _elapsed(self) -> float:
        return max(0.0, time.time() - float(self.start_time))

    def on_waypoint(self, topic: str, msg: Any) -> None:
        xy = _xy_from_waypoint(msg)
        item = self.waypoints.setdefault(
            topic,
            {
                "samples": 0,
                "frames": [],
                "first": None,
                "last": None,
                "unique_count": 0,
                "unique_points": [],
                "times_sec": [],
            },
        )
        item["samples"] += 1
        if xy is None:
            return
        x, y, z, frame = xy
        point = [x, y, z]
        if item["first"] is None:
            item["first"] = point
        item["last"] = point
        if frame and frame not in item["frames"]:
            item["frames"].append(frame)
        item.setdefault("times_sec", []).append(round(self._elapsed(), 3))
        unique_points = item.setdefault("unique_points", [])
        if not any(
            math.hypot(x - float(existing[0]), y - float(existing[1])) <= 0.05
            and abs(z - float(existing[2])) <= 0.05
            for existing in unique_points
        ):
            unique_points.append(point)
        item["unique_count"] = len(unique_points)
        self._update_active_waypoint(topic, (x, y))

    def on_path(self, topic: str, msg: Any) -> None:
        poses = list(getattr(msg, "poses", []) or [])
        frame = str(getattr(getattr(msg, "header", None), "frame_id", "") or "")
        item = self.paths.setdefault(
            topic,
            {
                "samples": 0,
                "nonempty_samples": 0,
                "max_poses": 0,
                "frames": [],
                "first_count": None,
                "last_count": 0,
                "nonempty_times_sec": [],
            },
        )
        item["samples"] += 1
        item["last_count"] = len(poses)
        item["max_poses"] = max(int(item["max_poses"]), len(poses))
        if poses:
            item["nonempty_samples"] += 1
            item.setdefault("nonempty_times_sec", []).append(round(self._elapsed(), 3))
        if item["first_count"] is None:
            item["first_count"] = len(poses)
        if frame and frame not in item["frames"]:
            item["frames"].append(frame)

    def on_cmd_vel(self, msg: Any) -> None:
        norm = _norm_cmd(msg)
        twist = getattr(msg, "twist", msg)
        linear = getattr(twist, "linear", None)
        angular = getattr(twist, "angular", None)
        linear_x = float(getattr(linear, "x", 0.0) or 0.0)
        linear_y = float(getattr(linear, "y", 0.0) or 0.0)
        angular_z = float(getattr(angular, "z", 0.0) or 0.0)
        self.cmd_vel["samples"] += 1
        self.cmd_vel["max_norm"] = max(float(self.cmd_vel["max_norm"]), norm)
        self.cmd_vel["max_abs_linear_x"] = max(
            float(self.cmd_vel["max_abs_linear_x"]),
            abs(linear_x),
        )
        self.cmd_vel["max_abs_linear_y"] = max(
            float(self.cmd_vel["max_abs_linear_y"]),
            abs(linear_y),
        )
        self.cmd_vel["max_abs_angular_z"] = max(
            float(self.cmd_vel["max_abs_angular_z"]),
            abs(angular_z),
        )
        if norm >= self.min_cmd_vel:
            self.cmd_vel["nonzero_samples"] += 1
            self.cmd_vel["last_nonzero"] = {
                "linear_x": linear_x,
                "linear_y": linear_y,
                "angular_z": angular_z,
                "norm": norm,
            }
            self.cmd_vel.setdefault("nonzero_times_sec", []).append(round(self._elapsed(), 3))

    def on_odom(self, topic: str, msg: Any) -> None:
        xy = _xy_from_odom(msg)
        item = self.odometry.setdefault(
            topic,
            {"samples": 0, "first_xy": None, "last_xy": None, "delta_m": 0.0, "history": []},
        )
        item["samples"] += 1
        if xy is None:
            return
        if item["first_xy"] is None:
            item["first_xy"] = [xy[0], xy[1]]
        item["last_xy"] = [xy[0], xy[1]]
        first = item["first_xy"]
        item["delta_m"] = math.hypot(xy[0] - float(first[0]), xy[1] - float(first[1]))
        history = item.setdefault("history", [])
        elapsed = self._elapsed()
        if (
            not history
            or elapsed - float(history[-1]["t"]) >= 1.0
            or math.hypot(
                xy[0] - float(history[-1]["xy"][0]),
                xy[1] - float(history[-1]["xy"][1]),
            )
            >= 0.05
        ):
            history.append({"t": round(elapsed, 3), "xy": [xy[0], xy[1]]})
        self._latest_odom_xy = xy
        self._update_motion_progress_from_odom(xy)

    def on_cloud(self, topic: str, msg: Any) -> None:
        item = self.clouds.setdefault(
            topic,
            {
                "samples": 0,
                "frames": [],
                "first_cells": None,
                "cells": set(),
                "points_seen": 0,
                "history": [],
            },
        )
        item["samples"] += 1
        frame = str(getattr(getattr(msg, "header", None), "frame_id", "") or "")
        if frame and frame not in item["frames"]:
            item["frames"].append(frame)
        cells = set()
        inv = 1.0 / self.voxel_size
        for x, y in _iter_xy_points(msg):
            cells.add((math.floor(x * inv), math.floor(y * inv)))
        item["points_seen"] += len(cells)
        if item["first_cells"] is None:
            item["first_cells"] = set(cells)
        item["cells"].update(cells)
        item.setdefault("history", []).append(
            {
                "t": round(self._elapsed(), 3),
                "cells": len(item["cells"] or set()),
                "area_m2": len(item["cells"] or set()) * self.voxel_size * self.voxel_size,
            }
        )

    def cloud_report(self) -> dict[str, Any]:
        topics: dict[str, Any] = {}
        best_delta = 0
        best_topic = ""
        for topic, item in self.clouds.items():
            initial = item["first_cells"] or set()
            final = item["cells"] or set()
            delta = max(0, len(final) - len(initial))
            if delta > best_delta:
                best_delta = delta
                best_topic = topic
            topics[topic] = {
                "samples": item["samples"],
                "frames": item["frames"],
                "initial_cells": len(initial),
                "final_cells": len(final),
                "cells_delta": delta,
                "area_delta_m2": delta * self.voxel_size * self.voxel_size,
                "points_seen": item["points_seen"],
                "history": item.get("history", []),
            }
        return {
            "topics": topics,
            "best_topic": best_topic,
            "best_cells_delta": best_delta,
            "best_area_delta_m2": best_delta * self.voxel_size * self.voxel_size,
        }

    def _distance_to_waypoint(self, waypoint_xy: tuple[float, float]) -> float | None:
        if self._latest_odom_xy is None:
            return None
        return math.hypot(
            float(self._latest_odom_xy[0]) - float(waypoint_xy[0]),
            float(self._latest_odom_xy[1]) - float(waypoint_xy[1]),
        )

    def _update_active_waypoint(self, topic: str, waypoint_xy: tuple[float, float]) -> None:
        previous = self._active_waypoint.get(topic)
        if previous is not None and math.hypot(
            waypoint_xy[0] - previous[0],
            waypoint_xy[1] - previous[1],
        ) <= 0.05:
            return
        self._active_waypoint[topic] = waypoint_xy
        distance = self._distance_to_waypoint(waypoint_xy)
        progress = self.motion_progress.setdefault(
            topic,
            {"segments": [], "best_delta_toward_m": 0.0},
        )
        segment = {
            "target_xy": [float(waypoint_xy[0]), float(waypoint_xy[1])],
            "first_distance_m": distance,
            "last_distance_m": distance,
            "min_distance_m": distance,
            "delta_toward_m": 0.0,
            "samples": 0 if distance is None else 1,
        }
        progress["segments"].append(segment)

    def _update_motion_progress_from_odom(self, xy: tuple[float, float]) -> None:
        for topic, waypoint_xy in self._active_waypoint.items():
            progress = self.motion_progress.setdefault(
                topic,
                {"segments": [], "best_delta_toward_m": 0.0},
            )
            if not progress["segments"]:
                self._update_active_waypoint(topic, waypoint_xy)
            segment = progress["segments"][-1]
            distance = math.hypot(xy[0] - waypoint_xy[0], xy[1] - waypoint_xy[1])
            if segment.get("first_distance_m") is None:
                segment["first_distance_m"] = distance
                segment["min_distance_m"] = distance
            segment["last_distance_m"] = distance
            segment["min_distance_m"] = min(
                float(segment.get("min_distance_m") or distance),
                distance,
            )
            segment["samples"] = int(segment.get("samples") or 0) + 1
            first_distance = float(segment.get("first_distance_m") or distance)
            min_distance = float(segment.get("min_distance_m") or distance)
            segment["delta_toward_m"] = max(0.0, first_distance - min_distance)
            progress["best_delta_toward_m"] = max(
                float(progress.get("best_delta_toward_m") or 0.0),
                float(segment["delta_toward_m"]),
            )

    def report(self) -> dict[str, Any]:
        return {
            "duration_sec": self._elapsed(),
            "waypoints": self.waypoints,
            "paths": self.paths,
            "cmd_vel": self.cmd_vel,
            "odometry": self.odometry,
            "cloud_coverage": self.cloud_report(),
            "motion_progress": self.motion_progress,
        }


def _command_subscriber_report(node: Any) -> dict[str, Any]:
    subscribers: dict[str, Any] = {}
    publishers: dict[str, Any] = {}
    blocked: list[dict[str, str]] = []
    unexpected_publishers: list[dict[str, str]] = []
    for topic in COMMAND_TOPICS:
        sub_endpoints = []
        try:
            infos = node.get_subscriptions_info_by_topic(topic)
        except Exception:
            infos = []
        for info in infos:
            node_name = getattr(info, "node_name", "") or ""
            node_namespace = getattr(info, "node_namespace", "") or ""
            full = f"{node_namespace.rstrip('/')}/{node_name}".replace("//", "/")
            sub_endpoints.append(full)
            low = full.lower()
            if any(token in low for token in HARDWARE_NODE_TOKENS):
                blocked.append({"topic": topic, "node": full})
        subscribers[topic] = sub_endpoints

        pub_endpoints = []
        try:
            pub_infos = node.get_publishers_info_by_topic(topic)
        except Exception:
            pub_infos = []
        for info in pub_infos:
            node_name = getattr(info, "node_name", "") or ""
            node_namespace = getattr(info, "node_namespace", "") or ""
            full = f"{node_namespace.rstrip('/')}/{node_name}".replace("//", "/")
            pub_endpoints.append(full)
            low = full.lower()
            if any(token in low for token in HARDWARE_NODE_TOKENS):
                blocked.append({"topic": topic, "node": full})
            if topic == "/cmd_vel" and node_name != "lingtu_cmu_unity_adapter":
                unexpected_publishers.append({"topic": topic, "node": full})
        publishers[topic] = pub_endpoints
    return {
        "topics": subscribers,
        "subscribers": subscribers,
        "publishers": publishers,
        "blocked_hardware_nodes": blocked,
        "unexpected_command_publishers": unexpected_publishers,
    }


def _load_cmu_runtime_contract() -> dict[str, Any]:
    try:
        import sys

        root = Path(__file__).resolve().parents[2]
        for candidate in (root / "src", root):
            path = str(candidate)
            if path not in sys.path:
                sys.path.insert(0, path)
        from runtime.blueprints.simulation_contract import simulation_runtime_contract

        return simulation_runtime_contract("cmu_unity_external").as_report()
    except Exception as exc:
        return {
            "name": "cmu_unity_external",
            "provider": "cmu_unity",
            "ok": False,
            "import_error": f"{type(exc).__name__}: {exc}",
        }


def _runtime_topic_evidence(topic: str, metrics: dict[str, Any]) -> dict[str, Any]:
    waypoints = metrics.get("waypoints") or {}
    paths = metrics.get("paths") or {}
    odometry = metrics.get("odometry") or {}
    clouds = (metrics.get("cloud_coverage") or {}).get("topics") or {}
    if topic in waypoints:
        item = waypoints.get(topic) or {}
        samples = int(item.get("samples") or 0)
        return {
            "samples": samples,
            "frames": list(item.get("frames") or []),
            "ok": samples > 0,
        }
    if topic in paths:
        item = paths.get(topic) or {}
        nonempty = int(item.get("nonempty_samples") or 0)
        max_poses = int(item.get("max_poses") or 0)
        return {
            "samples": int(item.get("samples") or 0),
            "nonempty_samples": nonempty,
            "max_poses": max_poses,
            "frames": list(item.get("frames") or []),
            "ok": nonempty > 0 and max_poses >= 2,
        }
    if topic == "/nav/cmd_vel":
        item = metrics.get("cmd_vel") or {}
        return {
            "samples": int(item.get("samples") or 0),
            "nonzero_samples": int(item.get("nonzero_samples") or 0),
            "max_norm": float(item.get("max_norm") or 0.0),
            "ok": int(item.get("nonzero_samples") or 0) > 0,
        }
    if topic in odometry:
        item = odometry.get(topic) or {}
        samples = int(item.get("samples") or 0)
        delta_m = float(item.get("delta_m") or 0.0)
        return {
            "samples": samples,
            "delta_m": delta_m,
            "ok": samples > 0 and delta_m > 0.0,
        }
    if topic in clouds:
        item = clouds.get(topic) or {}
        samples = int(item.get("samples") or 0)
        area = float(item.get("area_delta_m2") or 0.0)
        return {
            "samples": samples,
            "area_delta_m2": area,
            "frames": list(item.get("frames") or []),
            "ok": samples > 0 or area > 0.0,
        }
    return {"samples": 0, "ok": False}


def _cmu_runtime_contract_evidence(
    metrics: dict[str, Any],
    *,
    ros_domain_id: str,
    path_requirements: dict[str, Any],
    scan_requirements: dict[str, Any],
    map_requirements: dict[str, Any],
    planner_diagnostics: dict[str, Any],
    require_planner_diagnostics: bool,
) -> dict[str, Any]:
    definition = _load_cmu_runtime_contract()
    errors: list[str] = []
    if definition.get("import_error"):
        errors.append(str(definition["import_error"]))
    if ros_domain_id in {"", "0"}:
        errors.append("ROS_DOMAIN_ID must be isolated and non-zero")

    evidence_topics = {
        str(topic)
        for key in (
            "required_runtime_topics",
            "required_path_topics",
            "required_scan_topics",
            "required_map_growth_topics",
            "required_slam_topics",
        )
        for topic in list(definition.get(key) or [])
    }
    topic_evidence = {
        topic: _runtime_topic_evidence(topic, metrics)
        for topic in sorted(evidence_topics)
    }
    for topic, evidence in topic_evidence.items():
        if evidence.get("ok") is not True:
            errors.append(f"{topic} did not satisfy runtime topic evidence")

    for topic in list(definition.get("required_path_topics") or []):
        req = path_requirements.get(topic)
        ok = req.get("ok") is True if req is not None else (topic_evidence.get(topic) or {}).get("ok") is True
        if not ok:
            errors.append(f"{topic} path contract not satisfied")

    for topic in list(definition.get("required_scan_topics") or []):
        req = scan_requirements.get(topic)
        ok = req.get("ok") is True if req is not None else (topic_evidence.get(topic) or {}).get("ok") is True
        if not ok:
            errors.append(f"{topic} scan contract not satisfied")

    for topic in list(definition.get("required_map_growth_topics") or []):
        req = map_requirements.get(topic)
        ok = req.get("ok") is True if req is not None else (topic_evidence.get(topic) or {}).get("ok") is True
        if not ok:
            errors.append(f"{topic} map-growth contract not satisfied")

    safety = metrics.get("hardware_safety") or {}
    if safety.get("blocked_hardware_nodes"):
        errors.append("hardware-looking command endpoint present")
    if safety.get("unexpected_command_publishers"):
        errors.append("unexpected simulation /cmd_vel publisher present")
    if require_planner_diagnostics and planner_diagnostics.get("available") is not True:
        errors.append("LingTu planner diagnostics unavailable")

    return {
        "name": "cmu_unity_external",
        "ok": not errors,
        "definition": definition,
        "topic_evidence": topic_evidence,
        "publisher_identity": {
            "subscribers": safety.get("subscribers") or safety.get("topics") or {},
            "publishers": safety.get("publishers") or {},
            "blocked_hardware_nodes": safety.get("blocked_hardware_nodes") or [],
            "unexpected_command_publishers": safety.get("unexpected_command_publishers") or [],
        },
        "planner_diagnostics_required": bool(require_planner_diagnostics),
        "planner_diagnostics_available": planner_diagnostics.get("available") is True,
        "errors": errors,
    }


def evaluate_report(metrics: dict[str, Any], args: argparse.Namespace, ros_domain_id: str) -> dict[str, Any]:
    blockers: list[str] = []
    warnings: list[str] = []
    if ros_domain_id in {"", "0"}:
        blockers.append("ROS_DOMAIN_ID must be non-empty and non-zero")

    waypoints = metrics["waypoints"]
    waypoint_samples = max(
        (int((waypoints.get(topic) or {}).get("samples") or 0) for topic in WAYPOINT_TOPICS),
        default=0,
    )
    if waypoint_samples < args.min_waypoint_samples:
        blockers.append("waypoint samples below threshold")
    waypoint_unique_requirements: dict[str, Any] = {}
    if int(getattr(args, "min_unique_waypoints", 0) or 0) > 0:
        required_unique = int(args.min_unique_waypoints)
        unique_topics = list(getattr(args, "unique_waypoint_topics", None) or [])
        if unique_topics:
            for topic in unique_topics:
                topic = str(topic).strip()
                observed = int((waypoints.get(topic) or {}).get("unique_count") or 0)
                ok = observed >= required_unique
                waypoint_unique_requirements[topic] = {
                    "required_unique_waypoints": required_unique,
                    "observed_unique_waypoints": observed,
                    "ok": ok,
                }
                if not ok:
                    blockers.append(f"{topic} unique waypoint count below threshold")
        else:
            observed = max(
                (
                    int((waypoints.get(topic) or {}).get("unique_count") or 0)
                    for topic in WAYPOINT_TOPICS
                ),
                default=0,
            )
            ok = observed >= required_unique
            waypoint_unique_requirements["any"] = {
                "required_unique_waypoints": required_unique,
                "observed_unique_waypoints": observed,
                "ok": ok,
            }
            if not ok:
                blockers.append("unique waypoint count below threshold")

    paths = metrics.get("paths") or {}
    path_requirements: dict[str, Any] = {}
    for topic in list(getattr(args, "required_path_topics", None) or []):
        item = paths.get(str(topic).strip()) or {}
        nonempty = int(item.get("nonempty_samples") or 0)
        max_poses = int(item.get("max_poses") or 0)
        ok = nonempty >= args.min_path_samples and max_poses >= args.min_path_poses
        path_requirements[str(topic).strip()] = {
            "required_nonempty_samples": args.min_path_samples,
            "required_min_poses": args.min_path_poses,
            "observed_nonempty_samples": nonempty,
            "observed_max_poses": max_poses,
            "ok": ok,
        }
        if not ok:
            blockers.append(f"{topic} path samples below threshold")

    cmd = metrics["cmd_vel"]
    if int(cmd.get("nonzero_samples") or 0) < args.min_cmd_vel_samples:
        blockers.append("/nav/cmd_vel nonzero samples below threshold")
    if float(cmd.get("max_norm") or 0.0) < args.min_cmd_vel:
        blockers.append("/nav/cmd_vel max_norm below threshold")

    odom = metrics["odometry"]
    odom_delta = max((float(item.get("delta_m") or 0.0) for item in odom.values()), default=0.0)
    if odom_delta < args.min_odom_delta_m:
        blockers.append("odom delta below threshold")

    motion_progress = metrics.get("motion_progress") or {}
    progress_requirements: dict[str, Any] = {}
    if args.require_motion_progress:
        topics = list(args.required_progress_topics or ["/nav/way_point"])
        for topic in topics:
            item = motion_progress.get(topic) or {}
            best_delta = float(item.get("best_delta_toward_m") or 0.0)
            ok = best_delta >= float(args.min_motion_progress_m)
            progress_requirements[topic] = {
                "required_delta_toward_m": float(args.min_motion_progress_m),
                "observed_best_delta_toward_m": best_delta,
                "ok": ok,
            }
            if not ok:
                blockers.append(f"{topic} motion progress below threshold")

    cloud = metrics["cloud_coverage"]
    if float(cloud.get("best_area_delta_m2") or 0.0) < args.min_map_area_delta_m2:
        blockers.append("map/exploration area delta below threshold")
    map_requirements: dict[str, Any] = {}
    scan_requirements: dict[str, Any] = {}
    cloud_topics = cloud.get("topics") or {}
    for topic in _parse_required_scan_topics(args):
        samples = int((cloud_topics.get(topic) or {}).get("samples") or 0)
        ok = samples >= args.min_scan_samples
        scan_requirements[topic] = {
            "required_samples": args.min_scan_samples,
            "observed_samples": samples,
            "ok": ok,
        }
        if not ok:
            blockers.append(f"{topic} samples below threshold")
    for topic, required_area in _parse_required_map_topics(args).items():
        observed_area = float((cloud_topics.get(topic) or {}).get("area_delta_m2") or 0.0)
        ok = observed_area >= required_area
        map_requirements[topic] = {
            "required_area_delta_m2": required_area,
            "observed_area_delta_m2": observed_area,
            "ok": ok,
        }
        if not ok:
            blockers.append(f"{topic} area delta below threshold")

    late_activity: dict[str, Any] = {}
    if float(getattr(args, "min_late_odom_delta_m", 0.0) or 0.0) > 0.0:
        late_odom = _late_odom_report(metrics, args)
        late_activity["odometry"] = late_odom
        if not late_odom["ok"]:
            blockers.append("late odom delta below threshold")
    if int(getattr(args, "min_late_cmd_vel_samples", 0) or 0) > 0:
        late_cmd = _late_cmd_report(metrics, args)
        late_activity["cmd_vel"] = late_cmd
        if not late_cmd["ok"]:
            blockers.append("late /nav/cmd_vel nonzero samples below threshold")
    if int(getattr(args, "min_late_path_samples", 0) or 0) > 0:
        late_path_topics = [
            str(topic).strip()
            for topic in list(getattr(args, "required_path_topics", None) or ["/nav/local_path"])
            if str(topic).strip()
        ]
        late_paths = _late_path_report(metrics, args, path_topics=late_path_topics)
        late_activity["paths"] = late_paths
        if not late_paths["ok"]:
            blockers.append("late path samples below threshold")
    if float(getattr(args, "min_late_map_area_delta_m2", 0.0) or 0.0) > 0.0:
        late_map = _late_map_report(metrics, args)
        late_activity["map_growth"] = late_map
        if not late_map["ok"]:
            total_map_ok = (
                float(cloud.get("best_area_delta_m2") or 0.0) >= args.min_map_area_delta_m2
                and all(bool(req.get("ok")) for req in map_requirements.values())
            )
            late_motion_ok = (
                (late_activity.get("odometry") or {}).get("ok") is True
                and (late_activity.get("cmd_vel") or {}).get("ok") is True
                and (late_activity.get("paths") or {}).get("ok") is True
            )
            accept_flat_late_map = (
                bool(getattr(args, "allow_flat_late_map_after_total_growth", False))
                and total_map_ok
                and late_motion_ok
            )
            late_map["accepted_flat_after_total_growth"] = accept_flat_late_map
            if accept_flat_late_map:
                warnings.append(
                    "late map growth was flat after total map growth passed; "
                    "accepted because odom, cmd_vel, and path activity remained live"
                )
            else:
                blockers.append("late map/exploration area delta below threshold")

    safety = metrics["hardware_safety"]
    if safety.get("blocked_hardware_nodes"):
        blockers.append("hardware-looking command subscriber present")
    if safety.get("unexpected_command_publishers"):
        blockers.append("unexpected /cmd_vel publisher present")

    gateway_navigation_status = metrics.get("gateway_navigation_status") or {}
    gateway_exploration_status = metrics.get("gateway_exploration_status") or {}
    planner_diagnostics = _planner_diagnostics_from_gateway(gateway_navigation_status)
    navigation_failure = _navigation_failure_from_gateway(gateway_navigation_status)
    direct_goal_fallback = _direct_goal_fallback_from_gateway(gateway_navigation_status)
    tare_navigation = _tare_navigation_from_gateway_exploration(gateway_exploration_status)
    tare_strategy_quality = _tare_strategy_quality_from_gateway_exploration(
        gateway_exploration_status,
        args,
    )
    frontier_no_gain_stall = _frontier_no_gain_stall_report(
        metrics,
        args,
        late_activity=late_activity,
        tare_navigation=tare_navigation,
    )
    same_source_tomogram = _same_source_tomogram_evidence()
    runtime_contract = _cmu_runtime_contract_evidence(
        metrics,
        ros_domain_id=ros_domain_id,
        path_requirements=path_requirements,
        scan_requirements=scan_requirements,
        map_requirements=map_requirements,
        planner_diagnostics=planner_diagnostics,
        require_planner_diagnostics=bool(getattr(args, "require_planner_diagnostics", False)),
    )
    gateway_session_start = metrics.get("gateway_session_start") or {}
    if (
        bool(getattr(args, "gateway_start_exploration_session", False))
        and not _gateway_session_start_ok(gateway_session_start)
    ):
        blockers.append("gateway exploration session start failed")
    if args.require_planner_diagnostics and not planner_diagnostics.get("available"):
        blockers.append("planner diagnostics unavailable")
    if args.require_no_planner_fallback and bool(planner_diagnostics.get("fallback_used")):
        blockers.append("planner fallback was used")
    if bool(getattr(args, "require_planner_path_safety", False)) and (
        planner_diagnostics.get("path_safety_ok") is False
    ):
        blockers.append("planner selected path safety failed")
    if bool(getattr(args, "require_no_primary_replan", False)) and bool(
        planner_diagnostics.get("primary_replan_used")
    ):
        blockers.append("planner primary replan was used")
    if navigation_failure.get("failed"):
        blockers.append("navigation mission failed")
    if bool(getattr(args, "require_exploration_navigation_success", False)):
        min_successes = max(1, int(getattr(args, "min_exploration_navigation_successes", 1) or 1))
        if tare_navigation.get("available") is not True:
            blockers.append("TARE navigation result unavailable")
        if int(tare_navigation.get("success_count") or 0) < min_successes:
            blockers.append("TARE navigation success count below threshold")
        if int(tare_navigation.get("failure_count") or 0) > 0:
            blockers.append("TARE navigation failure count is nonzero")
    if bool(getattr(args, "require_tare_strategy_quality", False)) and (
        tare_strategy_quality.get("ok") is not True
    ):
        for blocker in tare_strategy_quality.get("blockers") or [
            "TARE strategy quality failed"
        ]:
            blockers.append(f"TARE strategy quality: {blocker}")
    if bool(getattr(args, "require_frontier_no_gain_stall", False)) and (
        frontier_no_gain_stall.get("ok") is not True
    ):
        for blocker in frontier_no_gain_stall.get("blockers") or [
            "frontier late activity observation failed"
        ]:
            blockers.append(f"frontier_no_gain_stall: {blocker}")
    if bool(getattr(args, "require_same_source_tomogram", False)):
        if same_source_tomogram.get("ok") is not True:
            for error in same_source_tomogram.get("errors") or ["same-source tomogram missing"]:
                blockers.append(f"same-source tomogram: {error}")
    if bool(getattr(args, "require_runtime_contract", False)) and runtime_contract.get("ok") is not True:
        for error in runtime_contract.get("errors") or ["runtime contract failed"]:
            blockers.append(f"runtime contract: {error}")

    return {
        "schema_version": SCHEMA_VERSION,
        "ok": not blockers,
        "runtime_executed": True,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": bool(safety.get("blocked_hardware_nodes")),
        "ros_domain_id": ros_domain_id,
        "thresholds": {
            "min_waypoint_samples": args.min_waypoint_samples,
            "min_unique_waypoints": int(getattr(args, "min_unique_waypoints", 0) or 0),
            "min_cmd_vel": args.min_cmd_vel,
            "min_cmd_vel_samples": args.min_cmd_vel_samples,
            "min_odom_delta_m": args.min_odom_delta_m,
            "min_motion_progress_m": args.min_motion_progress_m,
            "min_map_area_delta_m2": args.min_map_area_delta_m2,
            "min_required_map_topic_area_m2": args.min_required_map_topic_area_m2,
            "late_window_sec": float(getattr(args, "late_window_sec", 0.0) or 0.0),
            "min_late_odom_delta_m": float(getattr(args, "min_late_odom_delta_m", 0.0) or 0.0),
            "min_late_cmd_vel_samples": int(getattr(args, "min_late_cmd_vel_samples", 0) or 0),
            "min_late_path_samples": int(getattr(args, "min_late_path_samples", 0) or 0),
            "min_late_map_area_delta_m2": float(
                getattr(args, "min_late_map_area_delta_m2", 0.0) or 0.0
            ),
            "allow_flat_late_map_after_total_growth": bool(
                getattr(args, "allow_flat_late_map_after_total_growth", False)
            ),
            "min_scan_samples": args.min_scan_samples,
            "voxel_size_m": args.voxel_size,
            "min_exploration_navigation_successes": int(
                getattr(args, "min_exploration_navigation_successes", 1) or 1
            ),
            "require_tare_strategy_quality": bool(
                getattr(args, "require_tare_strategy_quality", False)
            ),
            "min_tare_waypoints": int(getattr(args, "min_tare_waypoints", 1) or 1),
            "min_tare_paths": int(getattr(args, "min_tare_paths", 1) or 1),
            "min_tare_strategy_paths": int(
                getattr(args, "min_tare_strategy_paths", 0) or 0
            ),
            "max_tare_suppressed_waypoint_ratio": float(
                getattr(args, "max_tare_suppressed_waypoint_ratio", 1.0) or 1.0
            ),
            "require_frontier_no_gain_stall": bool(
                getattr(args, "require_frontier_no_gain_stall", False)
            ),
            "require_same_source_tomogram": bool(
                getattr(args, "require_same_source_tomogram", False)
            ),
        },
        "waypoints": waypoints,
        "waypoint_unique_requirements": waypoint_unique_requirements,
        "paths": paths,
        "path_requirements": path_requirements,
        "published_start_topics": metrics.get("published_start_topics") or [],
        "published_goal": metrics.get("published_goal"),
        "cmd_vel": cmd,
        "odometry": odom,
        "motion_progress": motion_progress,
        "progress_requirements": progress_requirements,
        "cloud_coverage": cloud,
        "scan_requirements": scan_requirements,
        "map_requirements": map_requirements,
        "late_activity": late_activity,
        "hardware_safety": safety,
        "cmd_vel_exclusive_to_lingtu": not bool(safety.get("unexpected_command_publishers")),
        "gateway_navigation_status": gateway_navigation_status,
        "gateway_exploration_status": gateway_exploration_status,
        "gateway_session_start": gateway_session_start,
        "runtime_contract": runtime_contract,
        "planner_diagnostics": planner_diagnostics,
        "navigation_failure": navigation_failure,
        "direct_goal_fallback": direct_goal_fallback,
        "tare_navigation": tare_navigation,
        "tare_strategy_quality": tare_strategy_quality,
        "frontier_no_gain_stall": frontier_no_gain_stall,
        "same_source_tomogram": same_source_tomogram,
        "warnings": warnings,
        "blockers": blockers,
    }


def run_gate(args: argparse.Namespace) -> dict[str, Any]:
    import rclpy
    from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from nav_msgs.msg import Path as ROSPath
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Bool

    rclpy.init(args=None)
    node = rclpy.create_node("cmu_unity_runtime_gate")
    sampler = RuntimeSampler(node, min_cmd_vel=args.min_cmd_vel, voxel_size=args.voxel_size)
    subscriptions = []
    qos = 10
    cloud_qos = qos_profile_sensor_data

    for topic in WAYPOINT_TOPICS:
        subscriptions.append(node.create_subscription(
            PointStamped, topic, lambda msg, topic=topic: sampler.on_waypoint(topic, msg), qos
        ))
    for topic in PATH_TOPICS:
        subscriptions.append(node.create_subscription(
            ROSPath, topic, lambda msg, topic=topic: sampler.on_path(topic, msg), qos
        ))
    subscriptions.append(node.create_subscription(TwistStamped, "/nav/cmd_vel", sampler.on_cmd_vel, qos))
    for topic in ODOM_TOPICS:
        subscriptions.append(node.create_subscription(
            Odometry, topic, lambda msg, topic=topic: sampler.on_odom(topic, msg), qos
        ))
    for topic in CLOUD_TOPICS:
        subscriptions.append(node.create_subscription(
            PointCloud2, topic, lambda msg, topic=topic: sampler.on_cloud(topic, msg), cloud_qos
        ))

    start_pubs = []
    if args.publish_start:
        start_pubs = [node.create_publisher(Bool, topic, qos) for topic in START_TOPICS]
    goal_pub = None
    published_goal: dict[str, Any] | None = None
    if args.goal_x is not None and args.goal_y is not None:
        goal_pub = node.create_publisher(PoseStamped, "/nav/goal_pose", qos)
        published_goal = {
            "topic": "/nav/goal_pose",
            "frame_id": args.goal_frame_id,
            "x": float(args.goal_x),
            "y": float(args.goal_y),
            "z": float(args.goal_z),
            "yaw": float(args.goal_yaw),
        }

    gateway_session_start: dict[str, Any] | None = None
    if args.gateway_start_exploration_session:
        gateway_session_start = _post_gateway_json_with_retry(
            args.gateway_url,
            "/api/v1/session/start",
            {"mode": "exploring", "slam_profile": "none"},
            args.gateway_timeout_sec,
            args.gateway_start_wait_sec,
        )

    deadline = time.time() + args.timeout_sec
    warmup_deadline = time.time() + args.warmup_sec
    next_goal_publish = warmup_deadline
    try:
        while time.time() < deadline:
            if start_pubs and time.time() >= warmup_deadline:
                msg = Bool()
                msg.data = True
                for pub in start_pubs:
                    pub.publish(msg)
            if goal_pub is not None and time.time() >= next_goal_publish:
                msg = PoseStamped()
                msg.header.frame_id = args.goal_frame_id
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.pose.position.x = float(args.goal_x)
                msg.pose.position.y = float(args.goal_y)
                msg.pose.position.z = float(args.goal_z)
                yaw = float(args.goal_yaw)
                msg.pose.orientation.z = math.sin(yaw / 2.0)
                msg.pose.orientation.w = math.cos(yaw / 2.0)
                goal_pub.publish(msg)
                next_goal_publish = time.time() + max(0.1, float(args.goal_republish_sec))
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        metrics = sampler.report()
        metrics["published_goal"] = published_goal
        metrics["published_start_topics"] = list(START_TOPICS) if args.publish_start else []
        metrics["gateway_session_start"] = gateway_session_start
        metrics["hardware_safety"] = _command_subscriber_report(node)
        metrics["gateway_navigation_status"] = _fetch_gateway_navigation_status(
            args.gateway_url,
            args.gateway_timeout_sec,
        )
        metrics["gateway_exploration_status"] = _fetch_gateway_exploration_status(
            args.gateway_url,
            args.gateway_timeout_sec,
        )
        for sub in subscriptions:
            try:
                node.destroy_subscription(sub)
            except Exception:
                pass
        for pub in start_pubs:
            try:
                node.destroy_publisher(pub)
            except Exception:
                pass
        if goal_pub is not None:
            try:
                node.destroy_publisher(goal_pub)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()

    return evaluate_report(metrics, args, os.environ.get("ROS_DOMAIN_ID", ""))


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout-sec", type=float, default=90.0)
    parser.add_argument("--warmup-sec", type=float, default=10.0)
    parser.add_argument("--min-waypoint-samples", type=int, default=1)
    parser.add_argument(
        "--min-unique-waypoints",
        type=int,
        default=0,
        help=(
            "Require distinct waypoint coordinates in addition to raw samples. "
            "Use with --unique-waypoint-topic for a specific topic."
        ),
    )
    parser.add_argument(
        "--unique-waypoint-topic",
        dest="unique_waypoint_topics",
        action="append",
        default=[],
        help="Topic whose unique waypoint count must meet --min-unique-waypoints.",
    )
    parser.add_argument("--min-path-samples", type=int, default=1)
    parser.add_argument("--min-path-poses", type=int, default=2)
    parser.add_argument(
        "--require-path-topic",
        dest="required_path_topics",
        action="append",
        default=[],
        help="Require a nav_msgs/Path topic to publish non-empty paths, e.g. /nav/global_path.",
    )
    parser.add_argument("--min-cmd-vel", type=float, default=0.01)
    parser.add_argument("--min-cmd-vel-samples", type=int, default=3)
    parser.add_argument("--min-odom-delta-m", type=float, default=0.10)
    parser.add_argument(
        "--late-window-sec",
        type=float,
        default=120.0,
        help="Window at the end of the run used by late activity checks.",
    )
    parser.add_argument(
        "--min-late-odom-delta-m",
        type=float,
        default=0.0,
        help="Require odometry movement during the final --late-window-sec seconds.",
    )
    parser.add_argument(
        "--min-late-cmd-vel-samples",
        type=int,
        default=0,
        help="Require non-zero /nav/cmd_vel samples during the final --late-window-sec seconds.",
    )
    parser.add_argument(
        "--min-late-path-samples",
        type=int,
        default=0,
        help=(
            "Require non-empty required path topic samples during the final "
            "--late-window-sec seconds."
        ),
    )
    parser.add_argument("--require-motion-progress", action="store_true")
    parser.add_argument(
        "--required-progress-topic",
        dest="required_progress_topics",
        action="append",
        default=[],
        help="Require odometry to move closer to this waypoint topic; defaults to /nav/way_point.",
    )
    parser.add_argument("--min-motion-progress-m", type=float, default=0.20)
    parser.add_argument("--min-map-area-delta-m2", type=float, default=0.5)
    parser.add_argument(
        "--min-late-map-area-delta-m2",
        type=float,
        default=0.0,
        help="Require map/cloud coverage growth during the final --late-window-sec seconds.",
    )
    parser.add_argument(
        "--allow-flat-late-map-after-total-growth",
        action="store_true",
        help=(
            "If total map growth already passed, treat a flat late map window "
            "as a warning when late odom, cmd_vel, and path activity are live. "
            "This avoids failing a run that is traversing already mapped cells."
        ),
    )
    parser.add_argument("--min-required-map-topic-area-m2", type=float, default=0.5)
    parser.add_argument("--min-scan-samples", type=int, default=1)
    parser.add_argument(
        "--required-scan-topic",
        dest="required_scan_topics",
        action="append",
        default=[],
        help=(
            "Require a scan/cloud topic to produce samples. "
            "Defaults to /registered_scan and /slam/registered_cloud."
        ),
    )
    parser.add_argument(
        "--required-map-topic",
        dest="required_map_topics",
        action="append",
        default=[],
        help=(
            "Require a specific map/cloud topic to grow. "
            "Format: TOPIC or TOPIC:MIN_AREA_M2. Defaults to /nav/terrain_map_ext. "
            "The CMU launch wrapper adds /slam/map_cloud for product runs."
        ),
    )
    parser.add_argument("--voxel-size", type=float, default=0.25)
    parser.add_argument("--publish-start", action="store_true")
    parser.add_argument("--goal-x", type=float)
    parser.add_argument("--goal-y", type=float)
    parser.add_argument("--goal-z", type=float, default=0.75)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--goal-frame-id", default="map")
    parser.add_argument("--goal-republish-sec", type=float, default=0.5)
    parser.add_argument(
        "--gateway-url",
        default="http://127.0.0.1:5050",
        help="Gateway base URL used to collect navigation planner diagnostics. Use empty string to disable.",
    )
    parser.add_argument("--gateway-timeout-sec", type=float, default=2.0)
    parser.add_argument(
        "--gateway-start-wait-sec",
        type=float,
        default=30.0,
        help="Retry Gateway session start for this many seconds to tolerate startup races.",
    )
    parser.add_argument(
        "--gateway-start-exploration-session",
        action="store_true",
        help=(
            "POST /api/v1/session/start after ROS subscriptions are ready. "
            "Use with --publish-start so TARE does not start before the gate can observe it."
        ),
    )
    parser.add_argument(
        "--require-planner-diagnostics",
        action="store_true",
        help="Fail strict runtime validation unless Gateway exposes diagnostics.last_plan_report.",
    )
    parser.add_argument(
        "--require-no-planner-fallback",
        action="store_true",
        help="Fail strict runtime validation if planner diagnostics show path-safety fallback was used.",
    )
    parser.add_argument(
        "--require-no-primary-replan",
        action="store_true",
        help="Fail strict runtime validation if PCT primary planning was repaired to a different goal.",
    )
    parser.add_argument(
        "--require-planner-path-safety",
        action="store_true",
        help="Fail strict runtime validation if the selected LingTu path is unsafe.",
    )
    parser.add_argument(
        "--require-exploration-navigation-success",
        action="store_true",
        help=(
            "Fail validation unless Gateway exploration status reports at "
            "least one TARE goal completed by LingTu nav.mission."
        ),
    )
    parser.add_argument(
        "--min-exploration-navigation-successes",
        type=int,
        default=1,
        help="Minimum successful TARE navigation terminal results required by the strict gate.",
    )
    parser.add_argument(
        "--require-tare-strategy-quality",
        action="store_true",
        help=(
            "Fail validation unless Gateway TARE stats show accepted waypoints, "
            "paths, bounded suppressions, and successful navigation."
        ),
    )
    parser.add_argument(
        "--min-tare-waypoints",
        type=int,
        default=1,
        help="Minimum accepted TARE waypoints required by strategy-quality validation.",
    )
    parser.add_argument(
        "--min-tare-paths",
        type=int,
        default=1,
        help="Minimum accepted TARE paths required by strategy-quality validation.",
    )
    parser.add_argument(
        "--min-tare-strategy-paths",
        type=int,
        default=0,
        help=(
            "Minimum accepted TARE strategy paths required by strategy-quality "
            "validation. Strict mode raises this to at least one."
        ),
    )
    parser.add_argument(
        "--min-tare-navigation-successes",
        type=int,
        default=1,
        help="Minimum successful TARE navigation results required by strategy-quality validation.",
    )
    parser.add_argument(
        "--max-tare-suppressed-waypoint-ratio",
        type=float,
        default=0.75,
        help=(
            "Maximum allowed ratio of suppressed TARE waypoints to accepted plus "
            "suppressed waypoints."
        ),
    )
    parser.add_argument(
        "--require-runtime-contract",
        action="store_true",
        help="Fail validation unless the CMU Unity runtime contract evidence is complete.",
    )
    parser.add_argument(
        "--require-frontier-no-gain-stall",
        action="store_true",
        help=(
            "Fail validation unless late-window odom, cmd_vel, path, and map/no-gain "
            "evidence proves exploration stayed live after the initial pass condition."
        ),
    )
    parser.add_argument(
        "--require-same-source-tomogram",
        action="store_true",
        help=(
            "Fail validation unless LINGTU_CMU_TOMOGRAM is backed by "
            "cmu_unity_tomogram_capture.json from the same run directory."
        ),
    )
    parser.add_argument("--json-out", type=Path, default=None)
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    if (args.goal_x is None) != (args.goal_y is None):
        raise SystemExit("--goal-x and --goal-y must be provided together")
    if args.strict:
        args.require_planner_diagnostics = True
        args.require_runtime_contract = True
        args.require_exploration_navigation_success = True
        args.require_tare_strategy_quality = True
        args.min_tare_strategy_paths = max(int(args.min_tare_strategy_paths or 0), 1)
        args.require_frontier_no_gain_stall = True
        if float(args.late_window_sec or 0.0) == 120.0:
            args.late_window_sec = 60.0
        args.min_late_odom_delta_m = max(float(args.min_late_odom_delta_m or 0.0), 0.5)
        args.min_late_cmd_vel_samples = max(int(args.min_late_cmd_vel_samples or 0), 10)
        args.min_late_path_samples = max(int(args.min_late_path_samples or 0), 5)
        args.min_late_map_area_delta_m2 = max(float(args.min_late_map_area_delta_m2 or 0.0), 1.0)
        args.allow_flat_late_map_after_total_growth = True
    try:
        report = run_gate(args)
    except Exception as exc:
        report = {
            "schema_version": SCHEMA_VERSION,
            "ok": False,
            "runtime_executed": False,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "runtime_contract": {
                "name": "cmu_unity_external",
                "ok": False,
                "errors": [str(exc)],
            },
            "blockers": [str(exc)],
        }

    text = json.dumps(report, indent=2, sort_keys=True)
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    print(text)
    if args.strict and not report.get("ok"):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

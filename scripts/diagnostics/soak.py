#!/usr/bin/env python3
"""Read-only non-motion readiness soak for LingTu field validation."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request
from urllib.parse import urlparse
from typing import Any


READ_ONLY_ENDPOINT_NAMES = {
    "/ready": "ready",
    "/api/v1/readiness": "readiness",
    "/api/v1/app/bootstrap": "bootstrap",
    "/api/v1/app/capabilities": "capabilities",
    "/api/v1/localization/status": "localization",
    "/api/v1/navigation/status": "navigation",
    "/api/v1/health": "health",
    "/api/v1/state": "state",
    "/api/v1/path": "path",
    "/api/v1/scene_graph": "scene_graph",
    "/api/v1/locations": "locations",
    "/api/v1/devices": "devices",
}
READ_ONLY_ENDPOINTS = tuple(READ_ONLY_ENDPOINT_NAMES)
SCHEMA_VERSIONED_ENDPOINTS = {
    "bootstrap",
    "capabilities",
    "readiness",
    "localization",
    "navigation",
    "state",
    "path",
    "scene_graph",
    "locations",
}

IDLE_SOURCES = {"", "none", "unknown", "null"}
MOVING_STATES = {
    "EXECUTING",
    "NAVIGATING",
    "PLANNING",
    "EXPLORING",
    "RECOVERY",
    "PATROLLING",
}
BAD_LOCALIZATION_STATES = {"no_odometry", "lost", "relocalizing", "initializing"}
NON_MOTION_NAVIGATION_BLOCKERS = {
    "navigation_session_inactive",
    "navigation_not_ready",
}
NON_MOTION_READY_REASONS = {
    f"navigation_blocked:{blocker}" for blocker in NON_MOTION_NAVIGATION_BLOCKERS
}


def env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw in (None, ""):
        return float(default)
    try:
        return float(raw)
    except (TypeError, ValueError):
        return float(default)


def env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw in (None, ""):
        return int(default)
    try:
        return int(raw)
    except (TypeError, ValueError):
        return int(default)


def thresholds() -> dict[str, float | int]:
    return {
        "min_samples": env_int("LINGTU_SOAK_MIN_SAMPLES", 3),
        "min_slam_hz": env_float("LINGTU_SOAK_MIN_SLAM_HZ", 1.0),
        "min_map_points": env_float("LINGTU_SOAK_MIN_MAP_POINTS", 1.0),
        "max_odom_age_ms": env_float("LINGTU_SOAK_MAX_ODOM_AGE_MS", 1500.0),
        "max_cloud_age_ms": env_float("LINGTU_SOAK_MAX_CLOUD_AGE_MS", 5000.0),
        "max_diag_age_ms": env_float("LINGTU_SOAK_MAX_DIAG_AGE_MS", 3000.0),
        "max_localizer_health_age_ms": env_float(
            "LINGTU_SOAK_MAX_LOCALIZER_HEALTH_AGE_MS",
            3000.0,
        ),
        "min_localization_confidence": env_float(
            "LINGTU_SOAK_MIN_LOCALIZATION_CONFIDENCE",
            0.5,
        ),
        "max_xy_drift_m": env_float("LINGTU_SOAK_MAX_XY_DRIFT_M", 0.30),
        "max_yaw_drift_rad": env_float("LINGTU_SOAK_MAX_YAW_DRIFT_RAD", 0.35),
        "max_map_points_drop_ratio": env_float(
            "LINGTU_SOAK_MAX_MAP_POINTS_DROP_RATIO",
            0.75,
        ),
    }


def mapping(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def as_float(value: Any) -> float | None:
    try:
        if value in (None, "", "unknown", "null"):
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def as_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"1", "true", "yes", "ok", "ready", "active"}:
            return True
        if lowered in {"0", "false", "no", "lost", "inactive"}:
            return False
    return None


def nested(data: dict[str, Any], *keys: str) -> Any:
    cur: Any = data
    for key in keys:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def first_float(*values: Any) -> float | None:
    for value in values:
        parsed = as_float(value)
        if parsed is not None:
            return parsed
    return None


def first_value(*values: Any) -> Any:
    for value in values:
        if value not in (None, ""):
            return value
    return None


def endpoint_label(path: Any) -> str | None:
    if not isinstance(path, str) or not path:
        return None
    parsed = urlparse(path)
    clean_path = parsed.path if parsed.scheme else path
    if clean_path == "/api/v1/app/traffic":
        return "traffic"
    return None


def advertised_read_only_endpoints(payloads: dict[str, dict[str, Any]]) -> dict[str, str]:
    endpoints: dict[str, str] = {}
    bootstrap_links = mapping(payloads.get("bootstrap", {}).get("links"))
    traffic_path = bootstrap_links.get("traffic")
    label = endpoint_label(traffic_path)
    if label:
        endpoints[str(traffic_path)] = label

    app_endpoints = mapping(nested(payloads.get("capabilities", {}), "endpoints", "app"))
    traffic_spec = mapping(app_endpoints.get("traffic"))
    traffic_path = traffic_spec.get("path")
    label = endpoint_label(traffic_path)
    if label:
        endpoints[str(traffic_path)] = label
    return endpoints


def client_contract_violations(payloads: dict[str, dict[str, Any]]) -> list[str]:
    violations: list[str] = []
    checked = set(SCHEMA_VERSIONED_ENDPOINTS)
    if "traffic" in payloads:
        checked.add("traffic")
    for name in sorted(checked):
        payload = payloads.get(name) or {}
        if payload.get("schema_version") != 1:
            violations.append(f"{name}.schema_version")

    bootstrap = payloads.get("bootstrap") or {}
    links = mapping(bootstrap.get("links"))
    for key in ("state", "events", "scene_graph", "locations", "path", "readiness"):
        if not links.get(key):
            violations.append(f"bootstrap.links.{key}")

    capabilities = payloads.get("capabilities") or {}
    if not mapping(capabilities.get("endpoints")):
        violations.append("capabilities.endpoints")
    readiness = payloads.get("readiness") or {}
    if readiness:
        if not isinstance(readiness.get("reasons"), list):
            violations.append("readiness.reasons")
        if not isinstance(readiness.get("modules"), dict):
            violations.append("readiness.modules")
        if readiness.get("status") not in {"ready", "degraded", "not_started"}:
            violations.append("readiness.status")
    return violations


def http_json(gateway: str, path: str, timeout: float = 3.0) -> tuple[int | None, dict[str, Any], str | None, float]:
    url = path if path.startswith("http") else f"{gateway}{path}"
    started = time.monotonic()
    try:
        req = urllib.request.Request(url, headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            body = resp.read(2_000_000).decode("utf-8", "replace")
            code = resp.getcode()
    except urllib.error.HTTPError as exc:
        body = exc.read(2_000_000).decode("utf-8", "replace")
        code = exc.code
    except Exception as exc:
        return None, {}, repr(exc), round((time.monotonic() - started) * 1000.0, 1)

    try:
        payload = json.loads(body or "{}")
    except Exception as exc:
        return code, {}, f"invalid json: {exc}", round((time.monotonic() - started) * 1000.0, 1)
    if not isinstance(payload, dict):
        payload = {"raw": payload}
    return code, payload, None, round((time.monotonic() - started) * 1000.0, 1)


def command_source_name(control: dict[str, Any]) -> str:
    source = control.get("active_cmd_source")
    if source in (None, "", "unknown"):
        source = control.get("active_source")
    if isinstance(source, dict):
        source = source.get("name") or source.get("source") or source.get("owner") or "none"
    if source in (None, ""):
        return "none"
    return str(source)


def ready_status_is_non_motion_safe(
    name: str,
    code: int | None,
    payload: dict[str, Any],
) -> bool:
    reasons = {str(reason) for reason in payload.get("reasons") or []}
    return (
        name == "ready"
        and code == 503
        and as_bool(payload.get("data_ready")) is True
        and as_bool(payload.get("non_motion_safe")) is True
        and not list(payload.get("failed_modules") or [])
        and bool(reasons)
        and reasons.issubset(NON_MOTION_READY_REASONS)
    )


def blockers_are_non_motion_safe(sample: dict[str, Any]) -> bool:
    blockers = {str(blocker) for blocker in sample.get("navigation_blockers") or []}
    if not blockers or not blockers.issubset(NON_MOTION_NAVIGATION_BLOCKERS):
        return False
    source = str(sample.get("active_cmd_source") or "none").lower()
    nav_state = str(sample.get("navigation_state") or "").upper()
    return (
        sample.get("non_motion_safe") is True
        and source in IDLE_SOURCES
        and nav_state not in MOVING_STATES
    )


def extract_pose(state_payload: dict[str, Any], path_payload: dict[str, Any]) -> dict[str, float | None] | None:
    candidates = [
        mapping(state_payload.get("odometry")),
        mapping(state_payload.get("robot")),
        mapping(nested(state_payload, "localization", "pose")),
        mapping(nested(state_payload, "localization", "robot")),
        mapping(path_payload.get("robot")),
    ]
    for item in candidates:
        if not item:
            continue
        pose = mapping(item.get("pose"))
        if pose:
            item = {**item, **pose}
        x = first_float(item.get("x"), item.get("px"), item.get("position_x"))
        y = first_float(item.get("y"), item.get("py"), item.get("position_y"))
        z = first_float(item.get("z"), item.get("pz"), item.get("position_z"))
        yaw = first_float(item.get("yaw"), item.get("theta"), item.get("heading"))
        if x is not None and y is not None:
            return {"x": x, "y": y, "z": z, "yaw": yaw}
    return None


def angle_delta(a: float | None, b: float | None) -> float | None:
    if a is None or b is None:
        return None
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


def numeric(values: list[Any]) -> list[float]:
    return [float(v) for v in values if isinstance(v, (int, float))]


def stat(values: list[Any]) -> dict[str, float | None]:
    vals = numeric(values)
    if not vals:
        return {"min": None, "avg": None, "max": None}
    return {
        "min": round(min(vals), 4),
        "avg": round(sum(vals) / len(vals), 4),
        "max": round(max(vals), 4),
    }


def sample_violations(sample: dict[str, Any], limits: dict[str, float | int]) -> tuple[list[str], list[str]]:
    violations: list[str] = []
    warnings: list[str] = []
    if sample["endpoint_errors"]:
        violations.extend(sample["endpoint_errors"])
    violations.extend(sample.get("client_contract_violations") or [])
    if sample["has_odometry"] is False:
        violations.append("has_odometry=false")
    if str(sample.get("localization_state") or "").lower() in BAD_LOCALIZATION_STATES:
        violations.append(f"localization_state={sample.get('localization_state')}")
    if sample["pose_fresh"] is False:
        violations.append("pose_fresh=false")
    for key, limit_name in (
        ("odom_age_ms", "max_odom_age_ms"),
        ("cloud_age_ms", "max_cloud_age_ms"),
        ("diag_age_ms", "max_diag_age_ms"),
        ("localizer_health_topic_age_ms", "max_localizer_health_age_ms"),
    ):
        value = sample.get(key)
        limit = float(limits[limit_name])
        if value is not None and value > limit:
            violations.append(f"{key}>{limit:g}")
    if sample["slam_hz"] is None or sample["slam_hz"] < float(limits["min_slam_hz"]):
        violations.append(f"slam_hz<{float(limits['min_slam_hz']):g}")
    if sample["map_points"] is None or sample["map_points"] < float(limits["min_map_points"]):
        violations.append(f"map_points<{float(limits['min_map_points']):g}")
    confidence = sample.get("confidence")
    if confidence is not None and confidence < float(limits["min_localization_confidence"]):
        warnings.append(f"confidence<{float(limits['min_localization_confidence']):g}")
    source = str(sample.get("active_cmd_source") or "none").lower()
    if source not in IDLE_SOURCES:
        violations.append(f"active_cmd_source={sample.get('active_cmd_source')}")
    nav_state = str(sample.get("navigation_state") or "").upper()
    if nav_state in MOVING_STATES:
        violations.append(f"navigation_state={nav_state}")
    blockers = sample.get("navigation_blockers") or []
    if blockers and not blockers_are_non_motion_safe(sample):
        violations.append("navigation_blockers=" + ",".join(map(str, blockers)))
    return violations, warnings


def sample_once(gateway: str, index: int, started_mono: float, limits: dict[str, float | int]) -> dict[str, Any]:
    payloads: dict[str, dict[str, Any]] = {}
    statuses: dict[str, int | None] = {}
    latencies: dict[str, float] = {}
    endpoint_errors: list[str] = []

    def fetch_endpoint(path: str, name: str) -> None:
        code, payload, error, latency = http_json(gateway, path)
        payloads[name] = payload
        statuses[name] = code
        latencies[name] = latency
        http_failed = code is None or (
            int(code) >= 400
            and not ready_status_is_non_motion_safe(name, code, payload)
        )
        if error or http_failed:
            endpoint_errors.append(f"{name}:{error or code}")

    for path in READ_ONLY_ENDPOINTS:
        fetch_endpoint(path, READ_ONLY_ENDPOINT_NAMES[path])
    for path, name in advertised_read_only_endpoints(payloads).items():
        if name not in payloads:
            fetch_endpoint(path, name)

    ready = payloads["ready"]
    client_readiness = payloads["readiness"]
    loc = payloads["localization"]
    nav = payloads["navigation"]
    health = payloads["health"]
    state_payload = payloads["state"]
    path_payload = payloads["path"]
    scene_payload = payloads["scene_graph"]
    locations_payload = payloads["locations"]
    capabilities_payload = payloads["capabilities"]

    failed_modules = list(ready.get("failed_modules") or [])
    data_ready = as_bool(ready.get("data_ready"))
    if data_ready is False:
        endpoint_errors.append("data_ready:false")
    elif data_ready is None and as_bool(ready.get("ready")) is False:
        endpoint_errors.append("ready:false")
    if failed_modules:
        endpoint_errors.append("ready_failed_modules=" + ",".join(map(str, failed_modules)))

    nav_readiness = mapping(nav.get("readiness"))
    nav_control = mapping(nav.get("control"))
    mission = mapping(nav.get("mission"))
    slam = mapping(nested(health, "sensors", "slam"))

    sample = {
        "index": index,
        "elapsed_s": round(max(0.0, time.monotonic() - started_mono), 3),
        "wall_ts": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "http_status": statuses,
        "http_latency_ms": latencies,
        "endpoint_errors": endpoint_errors,
        "gateway_ready": as_bool(ready.get("ready")),
        "data_ready": data_ready,
        "motion_ready": as_bool(ready.get("motion_ready")),
        "non_motion_safe": as_bool(ready.get("non_motion_safe")),
        "gateway_ready_reasons": list(ready.get("reasons") or []),
        "client_readiness_status": client_readiness.get("status"),
        "client_readiness_reasons": list(client_readiness.get("reasons") or []),
        "gateway_failed_modules": failed_modules,
        "client_contract_violations": client_contract_violations(payloads),
        "app_web": {
            "checked_endpoints": sorted(payloads.keys()),
            "readiness_status": client_readiness.get("status"),
            "readiness_reason_count": len(list(client_readiness.get("reasons") or [])),
            "scene_graph_count": as_float(scene_payload.get("count")),
            "locations_count": as_float(locations_payload.get("count")),
            "path_count": as_float(path_payload.get("count")),
            "capability_groups": sorted(mapping(capabilities_payload.get("endpoints")).keys()),
            "traffic_status": mapping(payloads.get("traffic")).get("status"),
        },
        "backend": first_value(
            loc.get("backend"),
            loc.get("localization_backend"),
            nested(state_payload, "localization", "backend"),
        ),
        "localization_state": first_value(loc.get("state"), nested(state_payload, "localization", "state")),
        "localization_ready": as_bool(
            first_value(loc.get("ready"), nested(state_payload, "localization", "ready"))
        ),
        "has_odometry": as_bool(
            first_value(
                loc.get("has_odometry"),
                health.get("has_odom"),
                nested(state_payload, "navigation", "has_odometry"),
            )
        ),
        "pose_fresh": as_bool(loc.get("pose_fresh")),
        "pose_freshness": loc.get("pose_freshness"),
        "confidence": first_float(loc.get("confidence"), nested(state_payload, "localization", "confidence")),
        "odom_age_ms": as_float(loc.get("odom_age_ms")),
        "cloud_age_ms": as_float(loc.get("cloud_age_ms")),
        "diag_age_ms": as_float(loc.get("diag_age_ms")),
        "localizer_health_topic_age_ms": as_float(loc.get("localizer_health_topic_age_ms")),
        "map_cloud_fresh": as_bool(loc.get("map_cloud_fresh")),
        "localizer_health": first_value(loc.get("localizer_health"), loc.get("localizer_health_raw")),
        "slam_hz": first_float(slam.get("hz"), health.get("slam_hz")),
        "map_points": first_float(health.get("map_points")),
        "navigation_state": first_value(nav.get("state"), mission.get("state")),
        "can_accept_goal": as_bool(
            first_value(
                nav.get("can_accept_goal"),
                nav_readiness.get("can_accept_goal"),
                nav_readiness.get("can_execute_autonomy"),
            )
        ),
        "navigation_blockers": list(nav_readiness.get("blockers") or []),
        "active_cmd_source": command_source_name(nav_control),
        "pose": extract_pose(state_payload, path_payload),
    }
    violations, warnings = sample_violations(sample, limits)
    sample["violations"] = violations
    sample["warnings"] = warnings
    return sample


def collect_samples(gateway: str, duration_s: float, interval_s: float, limits: dict[str, float | int]) -> tuple[list[dict[str, Any]], float]:
    samples: list[dict[str, Any]] = []
    started = time.monotonic()
    deadline = started + duration_s
    index = 0
    while True:
        samples.append(sample_once(gateway, index, started, limits))
        index += 1
        now = time.monotonic()
        if now >= deadline:
            break
        time.sleep(max(0.0, min(interval_s, deadline - now)))
    return samples, round(max(0.0, time.monotonic() - started), 3)


def summarize(samples: list[dict[str, Any]], elapsed_s: float, limits: dict[str, float | int]) -> tuple[dict[str, Any], list[str], list[str]]:
    poses = [sample["pose"] for sample in samples if sample.get("pose")]
    xy_drifts: list[float] = []
    yaw_drifts: list[float] = []
    if len(poses) >= 2:
        start_pose = poses[0]
        for pose in poses[1:]:
            xy_drifts.append(math.hypot(pose["x"] - start_pose["x"], pose["y"] - start_pose["y"]))
            yaw = angle_delta(pose.get("yaw"), start_pose.get("yaw"))
            if yaw is not None:
                yaw_drifts.append(yaw)

    map_points_values = numeric([sample.get("map_points") for sample in samples])
    map_points_drop_ratio = None
    if map_points_values and max(map_points_values) > 0.0:
        map_points_drop_ratio = round((max(map_points_values) - min(map_points_values)) / max(map_points_values), 4)

    violations: list[str] = []
    warnings: list[str] = []
    for sample in samples:
        violations.extend(f"sample[{sample['index']}]:{item}" for item in sample.get("violations", []))
        warnings.extend(f"sample[{sample['index']}]:{item}" for item in sample.get("warnings", []))

    if len(samples) < int(limits["min_samples"]):
        violations.append(f"sample_count<{int(limits['min_samples'])}")

    max_xy_drift_m = round(max(xy_drifts), 4) if xy_drifts else None
    max_yaw_drift_rad = round(max(yaw_drifts), 4) if yaw_drifts else None
    if max_xy_drift_m is None:
        warnings.append("pose_drift_unavailable")
    elif max_xy_drift_m > float(limits["max_xy_drift_m"]):
        violations.append(f"max_xy_drift_m>{float(limits['max_xy_drift_m']):g}")
    if max_yaw_drift_rad is None:
        warnings.append("yaw_drift_unavailable")
    elif max_yaw_drift_rad > float(limits["max_yaw_drift_rad"]):
        violations.append(f"max_yaw_drift_rad>{float(limits['max_yaw_drift_rad']):g}")
    if map_points_drop_ratio is not None and map_points_drop_ratio > float(limits["max_map_points_drop_ratio"]):
        warnings.append(f"map_points_drop_ratio>{float(limits['max_map_points_drop_ratio']):g}")

    summary = {
        "sample_count": len(samples),
        "elapsed_s": elapsed_s,
        "slam_hz": stat([sample.get("slam_hz") for sample in samples]),
        "confidence": stat([sample.get("confidence") for sample in samples]),
        "odom_age_ms": stat([sample.get("odom_age_ms") for sample in samples]),
        "cloud_age_ms": stat([sample.get("cloud_age_ms") for sample in samples]),
        "diag_age_ms": stat([sample.get("diag_age_ms") for sample in samples]),
        "localizer_health_topic_age_ms": stat(
            [sample.get("localizer_health_topic_age_ms") for sample in samples]
        ),
        "http_latency_ms": {
            name: stat([mapping(sample.get("http_latency_ms")).get(name) for sample in samples])
            for name in sorted(
                {
                    name
                    for sample in samples
                    for name in mapping(sample.get("http_latency_ms")).keys()
                }
            )
        },
        "app_web": {
            "checked_endpoints": sorted(
                {
                    name
                    for sample in samples
                    for name in mapping(sample.get("app_web")).get("checked_endpoints", [])
                }
            ),
            "readiness_statuses": sorted(
                {
                    str(status)
                    for sample in samples
                    for status in [mapping(sample.get("app_web")).get("readiness_status")]
                    if status
                }
            ),
            "readiness_reason_count": stat(
                [nested(sample, "app_web", "readiness_reason_count") for sample in samples]
            ),
            "scene_graph_count": stat(
                [nested(sample, "app_web", "scene_graph_count") for sample in samples]
            ),
            "locations_count": stat(
                [nested(sample, "app_web", "locations_count") for sample in samples]
            ),
            "path_count": stat(
                [nested(sample, "app_web", "path_count") for sample in samples]
            ),
            "capability_groups": sorted(
                {
                    group
                    for sample in samples
                    for group in mapping(sample.get("app_web")).get("capability_groups", [])
                }
            ),
        },
        "map_points": stat([sample.get("map_points") for sample in samples]),
        "map_points_drop_ratio": map_points_drop_ratio,
        "max_xy_drift_m": max_xy_drift_m,
        "max_yaw_drift_rad": max_yaw_drift_rad,
        "backends": sorted({str(sample.get("backend")) for sample in samples if sample.get("backend")}),
        "localization_states": sorted(
            {str(sample.get("localization_state")) for sample in samples if sample.get("localization_state")}
        ),
        "navigation_states": sorted(
            {str(sample.get("navigation_state")) for sample in samples if sample.get("navigation_state")}
        ),
        "active_cmd_source_values": sorted(
            {str(sample.get("active_cmd_source")) for sample in samples if sample.get("active_cmd_source")}
        ),
    }
    return summary, violations, warnings


def build_report(args: argparse.Namespace) -> dict[str, Any]:
    gateway = args.gateway.rstrip("/")
    limits = thresholds()
    samples, elapsed_s = collect_samples(gateway, args.duration, args.interval, limits)
    summary, violations, warnings = summarize(samples, elapsed_s, limits)
    return {
        "schema_version": 1,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "gateway": gateway,
        "mode": "non_motion_soak",
        "ok": not violations,
        "duration_s": args.duration,
        "interval_s": args.interval,
        "thresholds": limits,
        "summary": summary,
        "violations": violations,
        "warnings": warnings,
        "samples": samples,
    }


def print_text(report: dict[str, Any]) -> None:
    summary = report["summary"]
    status = "PASS" if report["ok"] else "FAIL"
    print(f"=== LingTu Soak @ {report['generated_at']} ===")
    print(
        f"{status}: non-motion readiness soak "
        f"samples={summary['sample_count']} elapsed={summary['elapsed_s']:.1f}s "
        f"gateway={report['gateway']}"
    )
    print(
        "  loc={} backend={} confidence={} slam_hz={} map_points={}".format(
            ",".join(summary["localization_states"]) or "-",
            ",".join(summary["backends"]) or "-",
            summary["confidence"],
            summary["slam_hz"],
            summary["map_points"],
        )
    )
    print(
        "  freshness odom={} cloud={} diag={} localizer_health={}".format(
            summary["odom_age_ms"],
            summary["cloud_age_ms"],
            summary["diag_age_ms"],
            summary["localizer_health_topic_age_ms"],
        )
    )
    print(
        "  drift max_xy_drift_m={} max_yaw_drift_rad={} map_points_drop_ratio={}".format(
            summary["max_xy_drift_m"],
            summary["max_yaw_drift_rad"],
            summary["map_points_drop_ratio"],
        )
    )
    print(
        "  navigation states={} active_cmd_source_values={}".format(
            ",".join(summary["navigation_states"]) or "-",
            ",".join(summary["active_cmd_source_values"]) or "-",
        )
    )
    app_web = summary["app_web"]
    print(
        "  app/web endpoints={} readiness={} readiness_reasons={} scene_graph={} locations={} path={}".format(
            ",".join(app_web["checked_endpoints"]) or "-",
            ",".join(app_web["readiness_statuses"]) or "-",
            app_web["readiness_reason_count"],
            app_web["scene_graph_count"],
            app_web["locations_count"],
            app_web["path_count"],
        )
    )
    if report["violations"]:
        print("  violations:")
        for item in report["violations"][:20]:
            print(f"    - {item}")
    if report["warnings"]:
        print("  warnings:")
        for item in report["warnings"][:20]:
            print(f"    - {item}")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gateway", default=os.environ.get("GW", "http://localhost:5050"))
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--interval", type=float, default=2.0)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration must be > 0")
    if args.interval <= 0:
        parser.error("--interval must be > 0")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    report = build_report(args)
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        print_text(report)
    return 1 if args.strict and report["violations"] else 0


if __name__ == "__main__":
    raise SystemExit(main())

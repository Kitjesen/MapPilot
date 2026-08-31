"""Read-only field diagnostics for an active LingTu RunPlan."""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import subprocess
import time
import urllib.error
import urllib.request
from collections import Counter


def env_float(name, default):
    raw = os.environ.get(name)
    if raw in (None, ""):
        return float(default)
    try:
        return float(raw)
    except (TypeError, ValueError):
        return float(default)


def check_status(ok, required):
    if ok:
        return "pass"
    return "fail" if required else "warn"


def check_level(required):
    return "p0" if required else "p1"


MIN_SLAM_HZ = env_float("LINGTU_DOCTOR_MIN_SLAM_HZ", 1.0)
MIN_MAP_POINTS = env_float("LINGTU_DOCTOR_MIN_MAP_POINTS", 1.0)
MAX_ODOM_AGE_MS = env_float("LINGTU_DOCTOR_MAX_ODOM_AGE_MS", 1500.0)
MAX_LOC_DIAG_AGE_MS = env_float("LINGTU_DOCTOR_MAX_LOC_DIAG_AGE_MS", 3000.0)
MAX_LOCALIZER_HEALTH_AGE_MS = env_float("LINGTU_DOCTOR_MAX_LOCALIZER_HEALTH_AGE_MS", 3000.0)
MAX_CLOUD_AGE_MS = env_float("LINGTU_DOCTOR_MAX_CLOUD_AGE_MS", 5000.0)
MIN_LOCALIZATION_CONFIDENCE = env_float("LINGTU_DOCTOR_MIN_LOCALIZATION_CONFIDENCE", 0.5)
DATA_NAV_BLOCKERS = {
    "odometry_missing",
    "localization_degraded",
    "localization_lost",
    "localization_relocalizing",
    "localization_initializing",
    "pose_stale",
}
MOVING_NAV_STATES = {"EXECUTING", "NAVIGATING", "PLANNING", "EXPLORING", "RECOVERY", "RUNNING"}


def resolve_tool(args):
    if not args:
        return args
    overrides = {
        "systemctl": os.environ.get("LINGTU_SYSTEMCTL_BIN"),
        "ip": os.environ.get("LINGTU_IP_BIN"),
    }
    override = overrides.get(args[0])
    if override:
        return [override, *args[1:]]
    return args


def run(args, timeout=3):
    try:
        proc = subprocess.run(
            resolve_tool(args),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout,
            check=False,
        )
        return proc.returncode, proc.stdout.strip(), proc.stderr.strip()
    except FileNotFoundError:
        return 127, "", "not found"
    except subprocess.TimeoutExpired as exc:
        return 124, (exc.stdout or "").strip(), "timeout"


def http_json(base_url, path, timeout=3):
    url = f"{base_url}{path}"
    try:
        req = urllib.request.Request(url, headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            body = resp.read(2_000_000).decode("utf-8", "replace")
            code = resp.getcode()
    except urllib.error.HTTPError as exc:
        body = exc.read(2_000_000).decode("utf-8", "replace")
        code = exc.code
    except Exception as exc:
        return None, None, str(exc)
    try:
        return code, json.loads(body or "{}"), None
    except Exception as exc:
        return code, None, f"invalid json: {exc}"


def as_float(value):
    try:
        if value in (None, "", "unknown", "null"):
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "ok", "ready", "active"}
    return bool(value)


def command_source_name(control):
    source = control.get("active_cmd_source")
    if source in (None, "", "unknown"):
        source = control.get("active_source")
    if isinstance(source, dict):
        source = source.get("name") or source.get("source") or source.get("owner") or "none"
    if source in (None, ""):
        return "none"
    return str(source)


def navigation_state_name(nav):
    mission = nav.get("mission") if isinstance(nav, dict) else {}
    if not isinstance(mission, dict):
        mission = {}
    return str(nav.get("state") or mission.get("state") or "unknown")


def navigation_state_is_idle(state):
    return str(state or "").upper() not in MOVING_NAV_STATES


def check_age(payload, key, max_age_ms, blockers):
    value = as_float(payload.get(key))
    if value is None:
        return None
    if value > max_age_ms:
        blockers.append(f"{key}>{max_age_ms:g}ms")
    return value


def netdev_state(name):
    base = f"/sys/class/net/{name}"
    evidence = {"interface": name}
    for field in ("operstate", "carrier"):
        path = os.path.join(base, field)
        try:
            with open(path, encoding="utf-8") as fh:
                evidence[field] = fh.read().strip()
        except OSError as exc:
            evidence[f"{field}_error"] = str(exc)
    _, out, err = run(["ip", "-br", "addr", "show", name], timeout=2)
    evidence["ip_br"] = out.strip()
    if err:
        evidence["ip_error"] = err
    return evidence


def add(checks, check_id, status, level, message, evidence=None):
    checks.append(
        {
            "id": check_id,
            "status": status,
            "level": level,
            "message": message,
            "evidence": evidence or {},
        }
    )


def current_run_path():
    from lingtu.product_lock import resolve_current_run_path

    return str(resolve_current_run_path(environment=os.environ))


def load_current_plan(requested_env, current_path):
    from lingtu.run_plan import CURRENT_RUN_SCHEMA, RunPlan

    evidence = {"current_run_path": current_path}
    try:
        with open(current_path, encoding="utf-8") as fh:
            current = json.load(fh)
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"cannot read current run record: {exc}") from exc
    if not isinstance(current, dict):
        raise RuntimeError("current run record must be a JSON object")
    if current.get("schema_version") != CURRENT_RUN_SCHEMA:
        raise RuntimeError("current run record has unsupported schema")

    required_fields = ("product", "env", "run_plan_path")
    values = {}
    for field in required_fields:
        value = current.get(field)
        if not isinstance(value, str) or not value.strip():
            raise RuntimeError(f"current run record requires {field}")
        values[field] = value.strip()

    configured_plan_path = os.path.expanduser(values["run_plan_path"])
    if not os.path.isabs(configured_plan_path):
        raise RuntimeError("current RunPlan path must be absolute")
    plan_path = os.path.realpath(configured_plan_path)
    plan = RunPlan.load(plan_path)
    if values["product"] != plan.product:
        raise RuntimeError("current Product does not match RunPlan")
    if values["env"] != plan.env:
        raise RuntimeError("current Env does not match RunPlan")
    if plan.env != requested_env:
        raise RuntimeError(f"current RunPlan belongs to Env {plan.env!r}, not requested Env {requested_env!r}")
    evidence.update(
        {
            "run_plan_path": plan_path,
            "product": plan.product,
            "env": plan.env,
        }
    )
    return plan, evidence


def runtime_dataflow_alignment(plan, payload, requested_env=None):
    """Compare Gateway runtime identity and primary transport with one RunPlan."""

    expected_env = requested_env or getattr(plan, "env", None)
    evidence = {
        "expected": {
            "product": getattr(plan, "product", None),
            "env": expected_env,
        }
    }
    blockers = []
    if plan is None:
        blockers.append("current_run_plan_unavailable")
        return blockers, evidence
    if not isinstance(payload, dict):
        blockers.append("runtime_dataflow_not_object")
        return blockers, evidence

    boundary = payload.get("runtime_boundary")
    boundary = boundary if isinstance(boundary, dict) else {}
    native_dds = payload.get("transport_layers")
    native_dds = native_dds if isinstance(native_dds, dict) else {}
    native_dds = native_dds.get("native_dds")
    native_dds = native_dds if isinstance(native_dds, dict) else {}
    evidence.update(
        {
            "runtime_boundary": boundary,
            "native_dds": native_dds,
        }
    )
    for key, expected in (
        ("product", plan.product),
        ("env", expected_env),
    ):
        if boundary.get(key) != expected:
            blockers.append(f"runtime_boundary_{key}_mismatch")
    if expected_env == "real" and native_dds.get("primary") is not True:
        blockers.append("real_env_native_dds_not_primary")
    return blockers, evidence


def driver_health_blockers(health):
    """Validate authoritative native driver evidence projected by Gateway."""

    if not isinstance(health, dict):
        return ["gateway_health_not_object"], {}
    driver_status = health.get("brainstem")
    if not isinstance(driver_status, dict):
        return ["native_driver_status_missing"], {}
    blockers = []
    if driver_status.get("source") != "lingtu-driver-status":
        blockers.append("native_driver_status_not_authoritative")
    if driver_status.get("status") != "connected":
        blockers.append("native_driver_not_connected")
    if driver_status.get("ready") is not True:
        blockers.append("native_driver_not_ready")
    if driver_status.get("stale") is True or driver_status.get("status") == "stale":
        blockers.append("native_driver_status_stale")
    return blockers, dict(driver_status)


def collect_report(options: argparse.Namespace) -> dict[str, object]:
    gw = options.gateway_url.rstrip("/")
    gateway_timeout_sec = options.gateway_timeout_sec
    if not math.isfinite(gateway_timeout_sec) or gateway_timeout_sec <= 0.0:
        raise ValueError("--gateway-timeout-sec must be finite and positive")
    non_motion = options.non_motion
    require_camera = options.require_camera
    requested_env = options.env
    checks: list[dict[str, object]] = []

    def gateway_json(path):
        return http_json(gw, path, timeout=gateway_timeout_sec)

    current_plan = None
    current_path = current_run_path()
    current_plan_evidence = {"current_run_path": current_path}
    try:
        current_plan, current_plan_evidence = load_current_plan(requested_env, current_path)
    except Exception as exc:
        current_plan_evidence["error"] = str(exc)
        add(
            checks,
            "run.current_plan",
            "fail",
            "p0",
            "current RunPlan is missing, invalid, or incompatible",
            current_plan_evidence,
        )
    else:
        add(
            checks,
            "run.current_plan",
            "pass",
            "p0",
            "current RunPlan is verified and compatible",
            current_plan_evidence,
        )

    dataflow_code, dataflow, dataflow_err = gateway_json("/api/v1/runtime/dataflow")
    dataflow_blockers, dataflow_evidence = runtime_dataflow_alignment(
        current_plan,
        dataflow,
        requested_env,
    )
    dataflow_evidence.update(
        {
            "http_status": dataflow_code,
            "error": dataflow_err,
        }
    )
    if dataflow_err:
        dataflow_blockers.append("runtime_dataflow_unavailable")
    add(
        checks,
        "dataflow.runtime_contract",
        "pass" if not dataflow_blockers else "fail",
        "p0",
        "Gateway runtime dataflow matches the current RunPlan"
        if not dataflow_blockers
        else "Gateway runtime dataflow does not match the current RunPlan",
        {
            **dataflow_evidence,
            "blockers": dataflow_blockers,
        },
    )

    processes = current_plan.processes if current_plan else ()
    required_process_names = {process.name for process in processes}
    required_services = {process.target for process in processes if process.manager == "systemd"}
    if require_camera:
        required_services.add("lt-camera.service")
    require_camera = require_camera or "camera" in required_process_names

    for svc in sorted(required_services):
        _, out, err = run(["systemctl", "is-active", svc], timeout=2)
        active = out.splitlines()[0] if out else "unknown"
        if active == "active":
            add(checks, f"service.{svc}", "pass", "p0", f"{svc} is active", {"active": active})
        else:
            add(checks, f"service.{svc}", "fail", "p0", f"{svc} is not active", {"active": active, "stderr": err})

    lidar_netdev_name = os.environ.get("LINGTU_LIVOX_NET_IFACE") or "eth0"
    lidar_netdev = netdev_state(lidar_netdev_name)
    lidar_netdev["selection"] = {
        "LINGTU_LIVOX_NET_IFACE": os.environ.get("LINGTU_LIVOX_NET_IFACE"),
    }
    if lidar_netdev.get("carrier") == "1":
        add(checks, "livox.netdev_carrier", "pass", "p0", "LiDAR network interface has carrier", lidar_netdev)
    elif "carrier" in lidar_netdev:
        add(
            checks,
            "livox.netdev_carrier",
            check_status(False, "lidar" in required_process_names),
            check_level("lidar" in required_process_names),
            "LiDAR network interface has no carrier",
            lidar_netdev,
        )
    else:
        add(
            checks,
            "livox.netdev_carrier",
            "warn",
            "p1",
            "could not inspect LiDAR network interface carrier",
            lidar_netdev,
        )

    ready_code, ready, ready_err = gateway_json("/ready")
    if ready_err or not isinstance(ready, dict):
        add(
            checks,
            "gateway.ready",
            "fail",
            "p0",
            "/ready unavailable",
            {"http_status": ready_code, "error": ready_err},
        )
    else:
        is_ready = bool(ready.get("ready"))
        data_ready = ready.get("data_ready")
        motion_ready = ready.get("motion_ready")
        if motion_ready is None:
            motion_ready = is_ready
        non_motion_safe = ready.get("non_motion_safe")
        if non_motion:
            if non_motion_safe is not True:
                ready_status = "fail"
                ready_message = "/ready does not explicitly permit non-motion verification"
            elif not as_bool(data_ready):
                ready_status = "fail"
                ready_message = "/ready data path is not ready"
            elif not is_ready:
                ready_status = "warn"
                ready_message = "/ready data path is ready but motion readiness is blocked"
            else:
                ready_status = "pass"
                ready_message = "/ready data path reports ready"
        else:
            ready_status = "pass" if is_ready else "fail"
            ready_message = "/ready reports ready" if is_ready else "/ready reports not ready"
        add(
            checks,
            "gateway.ready",
            ready_status,
            "p0",
            ready_message,
            {
                "http_status": ready_code,
                "ready": is_ready,
                "data_ready": as_bool(data_ready),
                "motion_ready": as_bool(motion_ready),
                "non_motion_safe": non_motion_safe is True,
                "reasons": ready.get("reasons", []),
                "failed_modules": ready.get("failed_modules", []),
                "data_blockers": (ready.get("runtime") or {}).get("summary", {}).get("data_blockers", []),
            },
        )

    client_ready_code, client_ready, client_ready_err = gateway_json("/api/v1/readiness")
    if client_ready_err or not isinstance(client_ready, dict):
        add(
            checks,
            "gateway.client_readiness",
            "fail",
            "p0",
            "/api/v1/readiness unavailable",
            {"http_status": client_ready_code, "error": client_ready_err},
        )
    else:
        status_value = client_ready.get("status")
        reasons_value = client_ready.get("reasons")
        modules_value = client_ready.get("modules")
        shape_ok = (
            client_ready.get("schema_version") == 1
            and status_value in {"ready", "degraded", "not_started"}
            and isinstance(reasons_value, list)
            and isinstance(modules_value, dict)
        )
        add(
            checks,
            "gateway.client_readiness",
            "pass" if shape_ok else "fail",
            "p0",
            "/api/v1/readiness exposes client-readable readiness"
            if shape_ok
            else "/api/v1/readiness schema is not client-readable",
            {
                "http_status": client_ready_code,
                "schema_version": client_ready.get("schema_version"),
                "status": status_value,
                "reasons": reasons_value if isinstance(reasons_value, list) else [],
                "module_count": len(modules_value) if isinstance(modules_value, dict) else None,
            },
        )

    health_code, health, health_err = gateway_json("/api/v1/health?details=true")
    if health_err or not isinstance(health, dict):
        add(
            checks,
            "gateway.health",
            "fail",
            "p0",
            "/api/v1/health unavailable",
            {"http_status": health_code, "error": health_err},
        )
        if "driver" in required_process_names:
            add(
                checks,
                "gateway.driver_status",
                "fail",
                "p0",
                "native driver status is unavailable because Gateway health failed",
                {
                    "http_status": health_code,
                    "error": health_err,
                    "blockers": ["gateway_health_unavailable"],
                },
            )
    else:
        modules_fail = health.get("modules_fail", 0)
        status = health.get("status")
        slam = health.get("sensors", {}).get("slam", {})
        slam_hz = as_float(slam.get("hz", health.get("slam_hz")))
        map_points = as_float(health.get("map_points"))
        has_odom = as_bool(health.get("has_odom"))
        ok = status in {"ok", "degraded"} and modules_fail in {0, "0"}
        add(
            checks,
            "gateway.health",
            "pass" if ok else "fail",
            "p0",
            "Gateway health is usable" if ok else "Gateway health reports failing modules",
            {
                "http_status": health_code,
                "status": status,
                "modules_ok": health.get("modules_ok"),
                "modules_fail": modules_fail,
                "slam_status": slam.get("status"),
                "slam_hz": slam_hz,
                "map_points": health.get("map_points"),
                "has_odom": health.get("has_odom"),
            },
        )
        if "driver" in required_process_names:
            driver_blockers, driver_evidence = driver_health_blockers(health)
            add(
                checks,
                "gateway.driver_status",
                "pass" if not driver_blockers else "fail",
                "p0",
                "native driver status is connected, ready, and fresh"
                if not driver_blockers
                else "native driver status is not ready",
                {
                    **driver_evidence,
                    "blockers": driver_blockers,
                },
            )
        stream_blockers = []
        if slam_hz is None or slam_hz < MIN_SLAM_HZ:
            stream_blockers.append(f"slam_hz<{MIN_SLAM_HZ:g}")
        if map_points is None or map_points < MIN_MAP_POINTS:
            stream_blockers.append(f"map_points<{MIN_MAP_POINTS:g}")
        if not has_odom:
            stream_blockers.append("has_odom=false")
        add(
            checks,
            "gateway.slam_stream",
            check_status(not stream_blockers, "slam" in required_process_names),
            check_level("slam" in required_process_names),
            "SLAM stream is fresh enough for readiness" if not stream_blockers else "SLAM stream is not ready",
            {
                "slam_hz": slam_hz,
                "min_slam_hz": MIN_SLAM_HZ,
                "map_points": map_points,
                "min_map_points": MIN_MAP_POINTS,
                "has_odom": has_odom,
                "blockers": stream_blockers,
            },
        )

    loc_code, loc, loc_err = gateway_json("/api/v1/localization/status")
    if loc_err or not isinstance(loc, dict):
        add(
            checks,
            "gateway.localization_status",
            check_status(False, "slam" in required_process_names),
            check_level("slam" in required_process_names),
            "localization status unavailable",
            {"http_status": loc_code, "error": loc_err},
        )
    else:
        state_value = str(loc.get("state") or loc.get("reported_state") or "").lower()
        loc_ready = loc.get("ready")
        if loc_ready is None:
            loc_ready = state_value in {"ready", "tracking", "good", "ok", "recovered"}
        pose_fresh = loc.get("pose_fresh")
        if pose_fresh is None:
            pose_fresh = str(loc.get("pose_freshness") or "").lower() in {"fresh", "ok"}
        has_odometry = loc.get("has_odometry")
        blockers = list(loc.get("reasons") or [])
        if not as_bool(loc_ready):
            blockers.append(f"state={state_value or 'unknown'}")
        if pose_fresh is False:
            blockers.append("pose_fresh=false")
        if has_odometry is False:
            blockers.append("has_odometry=false")
        odom_age = check_age(loc, "odom_age_ms", MAX_ODOM_AGE_MS, blockers)
        diag_age = check_age(loc, "diag_age_ms", MAX_LOC_DIAG_AGE_MS, blockers)
        health_age = check_age(loc, "localizer_health_topic_age_ms", MAX_LOCALIZER_HEALTH_AGE_MS, blockers)
        cloud_age = check_age(loc, "cloud_age_ms", MAX_CLOUD_AGE_MS, blockers)
        if loc.get("map_cloud_fresh") is False:
            blockers.append("map_cloud_fresh=false")
        localizer_health = str(loc.get("localizer_health") or loc.get("localizer_health_raw") or "").upper()
        if localizer_health and localizer_health in {"LOST", "DEGRADED", "UNHEALTHY", "ERROR"}:
            blockers.append(f"localizer_health={localizer_health}")
        add(
            checks,
            "gateway.localization_status",
            check_status(not blockers, "slam" in required_process_names),
            check_level("slam" in required_process_names),
            "localization is ready and fresh" if not blockers else "localization is not ready or stale",
            {
                "http_status": loc_code,
                "state": loc.get("state") or loc.get("reported_state"),
                "backend": loc.get("backend") or loc.get("localization_backend"),
                "pose_fresh": pose_fresh,
                "has_odometry": has_odometry,
                "odom_age_ms": odom_age,
                "max_odom_age_ms": MAX_ODOM_AGE_MS,
                "diag_age_ms": diag_age,
                "max_diag_age_ms": MAX_LOC_DIAG_AGE_MS,
                "localizer_health_topic_age_ms": health_age,
                "max_localizer_health_age_ms": MAX_LOCALIZER_HEALTH_AGE_MS,
                "cloud_age_ms": cloud_age,
                "max_cloud_age_ms": MAX_CLOUD_AGE_MS,
                "map_cloud_fresh": loc.get("map_cloud_fresh"),
                "localizer_health": localizer_health or None,
                "confidence": loc.get("confidence"),
                "blockers": blockers,
            },
        )
        confidence = as_float(loc.get("confidence"))
        if confidence is not None:
            add(
                checks,
                "gateway.localization_confidence",
                "pass" if confidence >= MIN_LOCALIZATION_CONFIDENCE else "warn",
                "p1",
                "localization confidence is above warning threshold"
                if confidence >= MIN_LOCALIZATION_CONFIDENCE
                else "localization confidence is below warning threshold",
                {"confidence": confidence, "min_confidence": MIN_LOCALIZATION_CONFIDENCE},
            )

    nav_code, nav, nav_err = gateway_json("/api/v1/navigation/status")
    if nav_err or not isinstance(nav, dict):
        add(
            checks,
            "gateway.navigation_status",
            check_status(False, "nav" in required_process_names),
            check_level("nav" in required_process_names),
            "navigation status unavailable",
            {"http_status": nav_code, "error": nav_err},
        )
    else:
        readiness = nav.get("readiness") or {}
        blockers = readiness.get("blockers") or []
        reason_codes = nav.get("reason_codes") or []
        can_accept = bool(nav.get("can_accept_goal", readiness.get("can_execute_autonomy", False)))
        state = navigation_state_name(nav)
        control = nav.get("control") or {}
        active_source = command_source_name(control)
        data_blockers = list(
            dict.fromkeys(blocker for blocker in list(blockers) + list(reason_codes) if blocker in DATA_NAV_BLOCKERS)
        )
        nav_ok = can_accept and not blockers
        if non_motion:
            if data_blockers:
                nav_status = "fail"
            elif blockers or not can_accept:
                nav_status = "warn"
            else:
                nav_status = "pass"
            if nav_status == "pass":
                nav_message = "navigation data path is ready for non-motion verification"
            elif nav_status == "fail":
                nav_message = "navigation has data blockers"
            else:
                nav_message = "navigation is not goal-ready but data path has no hard blockers"
        else:
            nav_status = "pass" if nav_ok else "fail"
            nav_message = "navigation can accept goals" if nav_ok else "navigation readiness has blockers"
        if nav_status == "fail" and "nav" not in required_process_names:
            nav_status = "warn"
        add(
            checks,
            "gateway.navigation_status",
            nav_status,
            "p0",
            nav_message,
            {
                "http_status": nav_code,
                "state": state,
                "can_accept_goal": can_accept,
                "blockers": blockers,
                "reason_codes": reason_codes,
                "data_blockers": data_blockers,
                "active_cmd_source": active_source,
            },
        )
        if non_motion and "nav" in required_process_names:
            source_idle = str(active_source).lower() in {"", "none", "unknown", "null"}
            state_idle = navigation_state_is_idle(state)
            add(
                checks,
                "safety.non_motion_guard",
                "pass" if source_idle and state_idle else "fail",
                "p0",
                "no active command source or executing mission"
                if source_idle and state_idle
                else "robot appears to have an active command source or mission",
                {"state": state, "active_cmd_source": active_source},
            )

    _, lsusb_out, _ = run(["lsusb"], timeout=3)
    usb_text = lsusb_out.lower()
    camera_usb = any(token in usb_text for token in ["orbbec", "gemini", "astra", "2bc5"])
    add(
        checks,
        "camera.usb",
        check_status(camera_usb, require_camera),
        check_level(require_camera),
        "camera USB device is enumerated" if camera_usb else "camera USB device is not enumerated",
        {"matched": camera_usb, "required": require_camera},
    )

    camera_snapshot_ok = False
    camera_snapshot_evidence = {"required": require_camera}
    try:
        req = urllib.request.Request(
            f"{gw}/api/v1/camera/snapshot",
            headers={"Accept": "image/jpeg"},
        )
        with urllib.request.urlopen(req, timeout=3) as resp:
            head = resp.read(3)
            content_type = resp.headers.get("content-type", "")
            camera_snapshot_ok = resp.getcode() == 200 and head.startswith(b"\xff\xd8")
            camera_snapshot_evidence.update(
                {
                    "http_status": resp.getcode(),
                    "content_type": content_type,
                    "jpeg_prefix": head.hex(),
                }
            )
    except urllib.error.HTTPError as exc:
        camera_snapshot_evidence.update({"http_status": exc.code, "error": exc.reason})
    except Exception as exc:
        camera_snapshot_evidence.update({"error": repr(exc)})
    add(
        checks,
        "camera.gateway_snapshot",
        check_status(camera_snapshot_ok, require_camera),
        check_level(require_camera),
        "Gateway camera snapshot endpoint returned JPEG"
        if camera_snapshot_ok
        else "Gateway camera snapshot endpoint did not return JPEG",
        camera_snapshot_evidence,
    )

    video_nodes = sorted(glob.glob("/dev/video*") + glob.glob("/dev/v4l/by-id/*"))
    if video_nodes:
        add(
            checks,
            "camera.video_nodes",
            "pass",
            check_level(require_camera),
            "video devices exist",
            {"nodes": video_nodes[:8], "required": require_camera},
        )
    else:
        add(
            checks,
            "camera.video_nodes",
            check_status(False, require_camera),
            check_level(require_camera),
            "no /dev/video or /dev/v4l/by-id devices found",
            {"nodes": [], "required": require_camera},
        )

    summary = Counter(check["status"] for check in checks)
    report = {
        "schema_version": 1,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "gateway": gw,
        "mode": "non_motion" if non_motion else "default",
        "ok": summary.get("fail", 0) == 0,
        "summary": {
            "pass": summary.get("pass", 0),
            "warn": summary.get("warn", 0),
            "fail": summary.get("fail", 0),
        },
        "checks": checks,
    }
    return report


def _render_check(check: dict[str, object]) -> None:
    status = str(check.get("status") or "warn")
    message = str(check.get("message") or check.get("id") or "check")
    evidence = check.get("evidence")
    if status == "pass":
        prefix = "\033[32mok\033[0m"
    elif status == "fail":
        prefix = "\033[31mFAIL\033[0m"
    else:
        prefix = "\033[33mWARN\033[0m"
    print(f"  {prefix}: {message}")
    if isinstance(evidence, dict) and evidence:
        detail = json.dumps(evidence, ensure_ascii=False, sort_keys=True, default=str)
        print(f"    {detail}")


def render_text(report: dict[str, object], options: argparse.Namespace) -> None:
    bold = "\033[1m"
    reset = "\033[0m"
    print(f"{bold}=== Lingtu Doctor @ {time.strftime('%H:%M:%S')} ==={reset}")
    if options.non_motion:
        print("Mode: non-motion read-only verification")
    if options.require_camera:
        print("Mode: camera required for full App/Web visual readiness")
    section_rules = (
        (
            "[1] Services",
            lambda check_id: check_id == "run.current_plan" or check_id.startswith("service."),
        ),
        ("[2] LiDAR network", lambda check_id: check_id.startswith("livox.")),
        (
            "[3] Runtime dataflow",
            lambda check_id: check_id.startswith("dataflow."),
        ),
        (
            "[4] Gateway",
            lambda check_id: check_id.startswith("gateway.") and check_id != "gateway.navigation_status",
        ),
        (
            "[5] Navigation readiness",
            lambda check_id: check_id.startswith("safety.") or check_id == "gateway.navigation_status",
        ),
        ("[6] Camera readiness", lambda check_id: check_id.startswith("camera.")),
    )
    report_checks = report.get("checks")
    if not isinstance(report_checks, list):
        report_checks = []
    rendered: set[int] = set()
    for title, predicate in section_rules:
        selected = []
        for index, check in enumerate(report_checks):
            if index in rendered or not isinstance(check, dict):
                continue
            if predicate(str(check.get("id") or "")):
                selected.append((index, check))
        if not selected:
            continue
        print(f"{bold}{title}{reset}")
        for index, check in selected:
            rendered.add(index)
            _render_check(check)
    remaining = [
        (index, check) for index, check in enumerate(report_checks) if index not in rendered and isinstance(check, dict)
    ]
    if remaining:
        print(f"{bold}Other{reset}")
        for _, check in remaining:
            _render_check(check)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run read-only LingTu field diagnostics.")
    parser.add_argument("--gateway-url", default=os.environ.get("GW", "http://localhost:5050"))
    parser.add_argument(
        "--gateway-timeout-sec",
        type=float,
        default=env_float("LINGTU_GATEWAY_TIMEOUT_SEC", 3.0),
    )
    parser.add_argument("--env", choices=("real", "sim"), default=os.environ.get("LINGTU_ENV", "real"))
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--non-motion", "--no-motion", dest="non_motion", action="store_true")
    parser.add_argument("--require-camera", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    options = parse_args(argv)
    report = collect_report(options)
    if options.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        render_text(report, options)
    summary = report.get("summary")
    failures = summary.get("fail", 0) if isinstance(summary, dict) else 0
    return 1 if options.strict and failures else 0


if __name__ == "__main__":
    raise SystemExit(main())

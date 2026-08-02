"""Read-only field diagnostics for an active LingTu RunPlan."""

from __future__ import annotations

import argparse
import asyncio
import glob
import json
import math
import os
import shlex
import subprocess
import sys
import time
import urllib.error
import urllib.request
from collections import Counter
from urllib.parse import urlparse


gw = ""
strict = False
non_motion = False
realtime = False
require_camera = False
ros2_enabled = False
requested_env = "real"
gateway_timeout_sec = 3.0
checks = []


def env_float(name, default):
    raw = os.environ.get(name)
    if raw in (None, ""):
        return float(default)
    try:
        return float(raw)
    except (TypeError, ValueError):
        return float(default)


def camera_check_status(ok):
    if ok:
        return "pass"
    return "fail" if require_camera else "warn"


def camera_check_level():
    return "p0" if require_camera else "p1"


MIN_SLAM_HZ = env_float("LINGTU_DOCTOR_MIN_SLAM_HZ", 1.0)
MIN_MAP_POINTS = env_float("LINGTU_DOCTOR_MIN_MAP_POINTS", 1.0)
MAX_ODOM_AGE_MS = env_float("LINGTU_DOCTOR_MAX_ODOM_AGE_MS", 1500.0)
MAX_LOC_DIAG_AGE_MS = env_float("LINGTU_DOCTOR_MAX_LOC_DIAG_AGE_MS", 3000.0)
MAX_LOCALIZER_HEALTH_AGE_MS = env_float("LINGTU_DOCTOR_MAX_LOCALIZER_HEALTH_AGE_MS", 3000.0)
MAX_CLOUD_AGE_MS = env_float("LINGTU_DOCTOR_MAX_CLOUD_AGE_MS", 5000.0)
MIN_LOCALIZATION_CONFIDENCE = env_float("LINGTU_DOCTOR_MIN_LOCALIZATION_CONFIDENCE", 0.5)
PLAN_PREVIEW_OFFSET_M = env_float("LINGTU_DOCTOR_PLAN_PREVIEW_OFFSET_M", 0.2)
REALTIME_TIMEOUT_S = env_float("LINGTU_DOCTOR_REALTIME_TIMEOUT_S", 6.0)
MAX_DATAFLOW_AVG_MS = env_float("LINGTU_DOCTOR_MAX_DATAFLOW_AVG_MS", 150.0)
MAX_DATAFLOW_P95_MS = env_float("LINGTU_DOCTOR_MAX_DATAFLOW_P95_MS", 250.0)
MAX_DATAFLOW_MAX_MS = env_float("LINGTU_DOCTOR_MAX_DATAFLOW_MAX_MS", 1000.0)
MAX_DATAFLOW_DROP_RATIO = env_float("LINGTU_DOCTOR_MAX_DATAFLOW_DROP_RATIO", 0.75)
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
        "ros2": os.environ.get("LINGTU_ROS2_BIN"),
        "systemctl": os.environ.get("LINGTU_SYSTEMCTL_BIN"),
        "journalctl": os.environ.get("LINGTU_JOURNALCTL_BIN"),
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
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
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


def http_json(path, timeout=3, method="GET", payload=None):
    url = path if path.startswith("http") else f"{gw}{path}"
    try:
        headers = {"Accept": "application/json"}
        data = None
        if payload is not None:
            data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(url, data=data, headers=headers, method=method)
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


def planning_pose_from_state(state_payload):
    odom = state_payload.get("odometry") if isinstance(state_payload, dict) else None
    if not isinstance(odom, dict):
        return None, None, None, {"reason": "odometry_unavailable"}
    x = as_float(odom.get("x"))
    y = as_float(odom.get("y"))
    z = as_float(odom.get("z"))
    if x is None or y is None:
        return None, None, None, {"reason": "odometry_position_unavailable"}
    if z is None:
        z = 0.0
    source_frame = str(odom.get("frame_id") or "").strip().lstrip("/")
    if source_frame in {"", "map"}:
        return x, y, z, {
            "reason": "already_in_map" if source_frame == "map" else "legacy_assumed_map",
            "source_frame": source_frame or None,
            "target_frame": "map",
            "transformed": False,
        }
    localization = state_payload.get("localization") or {}
    tf = localization.get("map_odom_tf") if isinstance(localization, dict) else None
    if not isinstance(tf, dict) or tf.get("valid") is False:
        return None, None, None, {
            "reason": "map_odom_tf_unavailable",
            "source_frame": source_frame,
            "target_frame": "map",
        }
    parent = str(tf.get("frame_id") or "").strip().lstrip("/")
    child = str(tf.get("child_frame_id") or "").strip().lstrip("/")
    if parent != "map" or child != source_frame:
        return None, None, None, {
            "reason": "map_odom_tf_frame_mismatch",
            "source_frame": source_frame,
            "target_frame": "map",
            "tf_parent": parent,
            "tf_child": child,
        }
    values = [as_float(tf.get(key)) for key in ("tx", "ty", "tz", "qx", "qy", "qz", "qw")]
    if any(value is None or not math.isfinite(value) for value in values):
        return None, None, None, {
            "reason": "map_odom_tf_invalid",
            "source_frame": source_frame,
            "target_frame": "map",
        }
    tx, ty, tz, qx, qy, qz, qw = values
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return None, None, None, {
            "reason": "map_odom_tf_zero_quaternion",
            "source_frame": source_frame,
            "target_frame": "map",
        }
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    rx = (1.0 - 2.0 * (qy * qy + qz * qz)) * x + 2.0 * (qx * qy - qz * qw) * y + 2.0 * (qx * qz + qy * qw) * z
    ry = 2.0 * (qx * qy + qz * qw) * x + (1.0 - 2.0 * (qx * qx + qz * qz)) * y + 2.0 * (qy * qz - qx * qw) * z
    rz = 2.0 * (qx * qz - qy * qw) * x + 2.0 * (qy * qz + qx * qw) * y + (1.0 - 2.0 * (qx * qx + qy * qy)) * z
    return tx + rx, ty + ry, tz + rz, {
        "reason": "map_odom_tf",
        "source_frame": source_frame,
        "target_frame": "map",
        "transformed": True,
        "tf_ts": tf.get("ts"),
    }


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


def module_detail(ready_payload, module_name):
    modules = ready_payload.get("modules") or {}
    module = modules.get(module_name) or {}
    if not isinstance(module, dict):
        return {}
    detail = module.get("detail")
    return detail if isinstance(detail, dict) else {}


def port_in_stats(ready_payload, module_name, port_name):
    detail = module_detail(ready_payload, module_name)
    ports_in = detail.get("ports_in") or {}
    port = ports_in.get(port_name) or {}
    return port if isinstance(port, dict) else {}


def add_dataflow_pressure_check(ready_payload, module_name, port_name, level="p1"):
    if not module_detail(ready_payload, module_name):
        return
    port = port_in_stats(ready_payload, module_name, port_name)
    if not port:
        add(
            f"dataflow.{module_name}.{port_name}",
            "warn",
            level,
            f"{module_name}.{port_name} stats are unavailable",
            {},
        )
        return
    latency = port.get("latency") if isinstance(port.get("latency"), dict) else {}
    avg_ms = as_float(port.get("avg_callback_ms"))
    max_ms = as_float(port.get("max_callback_ms"))
    p95_ms = as_float(latency.get("p95_ms"))
    p99_ms = as_float(latency.get("p99_ms"))
    rate_hz = as_float(port.get("rate_hz"))
    deliver_count = as_float(port.get("deliver_count")) or 0.0
    drop_count = as_float(port.get("drop_count")) or 0.0
    total_count = deliver_count + drop_count
    drop_ratio = (drop_count / total_count) if total_count > 0 else 0.0
    pressure = []
    if avg_ms is not None and avg_ms > MAX_DATAFLOW_AVG_MS:
        pressure.append(f"avg_callback_ms>{MAX_DATAFLOW_AVG_MS:g}")
    if p95_ms is not None and p95_ms > MAX_DATAFLOW_P95_MS:
        pressure.append(f"p95_ms>{MAX_DATAFLOW_P95_MS:g}")
    if max_ms is not None and max_ms > MAX_DATAFLOW_MAX_MS:
        pressure.append(f"max_callback_ms>{MAX_DATAFLOW_MAX_MS:g}")
    if total_count >= 100 and drop_ratio > MAX_DATAFLOW_DROP_RATIO:
        pressure.append(f"drop_ratio>{MAX_DATAFLOW_DROP_RATIO:g}")
    add(
        f"dataflow.{module_name}.{port_name}",
        "warn" if pressure else "pass",
        level,
        f"{module_name}.{port_name} callback pressure detected"
        if pressure
        else f"{module_name}.{port_name} callback pressure within thresholds",
        {
            "rate_hz": rate_hz,
            "avg_callback_ms": avg_ms,
            "p95_ms": p95_ms,
            "p99_ms": p99_ms,
            "max_callback_ms": max_ms,
            "deliver_count": deliver_count,
            "drop_count": drop_count,
            "drop_ratio": round(drop_ratio, 4),
            "thresholds": {
                "max_avg_ms": MAX_DATAFLOW_AVG_MS,
                "max_p95_ms": MAX_DATAFLOW_P95_MS,
                "max_max_ms": MAX_DATAFLOW_MAX_MS,
                "max_drop_ratio": MAX_DATAFLOW_DROP_RATIO,
            },
            "pressure": pressure,
        },
    )


def sse_smoke(timeout_s):
    url = f"{gw}/api/v1/events"
    started = time.time()
    try:
        req = urllib.request.Request(url, headers={"Accept": "text/event-stream"})
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            lines = []
            while time.time() - started < timeout_s and len(lines) < 12:
                raw = resp.readline()
                if not raw:
                    break
                line = raw.decode("utf-8", "replace").strip()
                if line:
                    lines.append(line[:200])
                if line.startswith("data:"):
                    return True, {"lines": lines, "first_data": line[:200]}
            return False, {"lines": lines, "error": "no data line before timeout"}
    except Exception as exc:
        return False, {"error": repr(exc)}


async def _ws_once(path, expected_prefix, timeout_s):
    try:
        import websockets
    except Exception as exc:
        return False, {"error": f"websockets unavailable: {exc}"}
    parsed = urlparse(gw)
    scheme = "wss" if parsed.scheme == "https" else "ws"
    url = f"{scheme}://{parsed.netloc}{path}"
    try:
        async with websockets.connect(url, max_size=8_000_000) as ws:
            msg = await asyncio.wait_for(ws.recv(), timeout=timeout_s)
            raw = msg.encode("utf-8") if isinstance(msg, str) else bytes(msg)
            prefix_ok = raw.startswith(expected_prefix)
            return prefix_ok, {"url": url, "bytes": len(raw), "prefix": raw[:3].hex()}
    except Exception as exc:
        return False, {"url": url, "error": repr(exc)}


def ws_smoke(path, expected_prefix, timeout_s):
    return asyncio.run(_ws_once(path, expected_prefix, timeout_s))


def topic_counts(topic):
    rc, out, err = run(["ros2", "topic", "info", topic], timeout=3)
    pubs = 0
    subs = 0
    if rc == 127:
        return None, None, "ros2 not found"
    if rc != 0 and not out:
        return 0, 0, err or f"ros2 topic info failed rc={rc}"
    for line in out.splitlines():
        if "Publisher count:" in line:
            try:
                pubs = int(line.rsplit(":", 1)[1].strip())
            except ValueError:
                pubs = 0
        if "Subscription count:" in line:
            try:
                subs = int(line.rsplit(":", 1)[1].strip())
            except ValueError:
                subs = 0
    return pubs, subs, None


def parse_camera_device_status(text):
    fields = {}
    parsers = {
        "device_online": "bool",
        "connection_type": "str",
        "color_frame_rate_cur": "float",
        "depth_frame_rate_cur": "float",
        "color_frame_rate_target": "float",
        "depth_frame_rate_target": "float",
    }
    for line in text.splitlines():
        if ":" not in line:
            continue
        key, raw = line.split(":", 1)
        key = key.strip()
        if key not in parsers:
            continue
        value = raw.strip().strip("'\"")
        if parsers[key] == "bool":
            lowered = value.lower()
            if lowered in {"true", "false"}:
                fields[key] = lowered == "true"
            else:
                fields[key] = value
        elif parsers[key] == "float":
            try:
                fields[key] = float(value)
            except ValueError:
                fields[key] = value
        else:
            fields[key] = value
    return fields


def camera_device_status_once():
    rc, out, err = run(["ros2", "topic", "echo", "/camera/device_status", "--once"], timeout=6)
    if rc == 127:
        return None, "ros2 not found"
    if rc != 0 and not out:
        return None, err or f"ros2 topic echo failed rc={rc}"
    status = parse_camera_device_status(out)
    if not status:
        return None, err or "device_status sample did not contain expected fields"
    return status, None


def latest_livox_sdk_event():
    rc, out, err = run(["journalctl", "-u", "lingtu-livox-dds.service", "-b", "-n", "200", "--no-pager"], timeout=5)
    if rc == 127:
        return "unknown", {"error": "journalctl not found"}
    if rc != 0 and not out:
        return "unknown", {"returncode": rc, "stderr": err}
    latest = None
    for line in out.splitlines():
        lower = line.lower()
        if "Init lds lidar success".lower() in lower:
            latest = ("success", line[-500:])
        elif (
            "init lds lidar fail" in lower
            or "failed to init livox lidar sdk" in lower
            or "bind failed" in lower
        ):
            latest = ("fail", line[-500:])
    if latest is None:
        return "unknown", {"returncode": rc, "stderr": err, "searched_lines": len(out.splitlines())}
    status, line = latest
    return status, {"returncode": rc, "latest_event": line}


def netdev_state(name):
    base = f"/sys/class/net/{name}"
    evidence = {"interface": name}
    for field in ("operstate", "carrier"):
        path = os.path.join(base, field)
        try:
            with open(path, "r", encoding="utf-8") as fh:
                evidence[field] = fh.read().strip()
        except OSError as exc:
            evidence[f"{field}_error"] = str(exc)
    rc, out, err = run(["ip", "-br", "addr", "show", name], timeout=2)
    evidence["ip_br"] = out.strip()
    if err:
        evidence["ip_error"] = err
    return evidence


def systemd_environment(service):
    rc, out, err = run(
        ["systemctl", "show", service, "-p", "Environment", "--value"],
        timeout=2,
    )
    values = {}
    if rc == 0 and out:
        try:
            entries = shlex.split(out)
        except ValueError:
            entries = out.split()
        for entry in entries:
            if "=" not in entry:
                continue
            key, value = entry.split("=", 1)
            values[key] = value
    return values, {"returncode": rc, "stderr": err}


def add(check_id, status, level, message, evidence=None):
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


def load_current_plan():
    from lingtu.run_plan import CURRENT_RUN_SCHEMA, RunPlan

    current_path = current_run_path()
    evidence = {"current_run_path": current_path}
    try:
        with open(current_path, "r", encoding="utf-8") as fh:
            current = json.load(fh)
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"cannot read current run record: {exc}") from exc
    if not isinstance(current, dict):
        raise RuntimeError("current run record must be a JSON object")
    if current.get("schema_version") != CURRENT_RUN_SCHEMA:
        raise RuntimeError("current run record has unsupported schema")

    required_fields = ("product", "env", "fingerprint", "run_plan_path")
    values = {}
    for field in required_fields:
        value = current.get(field)
        if not isinstance(value, str) or not value.strip():
            raise RuntimeError(f"current run record requires {field}")
        values[field] = value.strip()

    plan_path = os.path.expanduser(values["run_plan_path"])
    if not os.path.isabs(plan_path):
        raise RuntimeError("current RunPlan path must be absolute")
    plan = RunPlan.load(os.path.realpath(plan_path))
    plan.assert_compatible(environment=os.environ)
    if values["fingerprint"] != plan.fingerprint:
        raise RuntimeError("current RunPlan fingerprint does not match its record")
    if values["product"] != plan.product:
        raise RuntimeError("current Product does not match RunPlan")
    if values["env"] != plan.env:
        raise RuntimeError("current Env does not match RunPlan")
    if plan.env != requested_env:
        raise RuntimeError(
            f"current RunPlan belongs to Env {plan.env!r}, "
            f"not requested Env {requested_env!r}"
        )
    evidence.update(
        {
            "run_plan_path": os.path.realpath(plan_path),
            "product": plan.product,
            "env": plan.env,
            "fingerprint": plan.fingerprint,
        }
    )
    return plan, evidence


def runtime_dataflow_alignment(plan, payload):
    """Compare Gateway runtime identity and primary transport with one RunPlan."""

    evidence = {
        "expected": {
            "product": getattr(plan, "product", None),
            "env": getattr(plan, "env", None),
            "run_plan_fingerprint": getattr(plan, "fingerprint", None),
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
        ("env", plan.env),
        ("run_plan_fingerprint", plan.fingerprint),
    ):
        if boundary.get(key) != expected:
            blockers.append(f"runtime_boundary_{key}_mismatch")
    if plan.env == "real" and native_dds.get("primary") is not True:
        blockers.append("real_env_native_dds_not_primary")
    return blockers, evidence


def driver_health_blockers(health):
    """Validate authoritative native driver evidence projected by Gateway."""

    if not isinstance(health, dict):
        return ["gateway_health_not_object"], {}
    brainstem = health.get("brainstem")
    if not isinstance(brainstem, dict):
        return ["native_driver_status_missing"], {}
    blockers = []
    if brainstem.get("source") != "lingtu-driver-status":
        blockers.append("native_driver_status_not_authoritative")
    if brainstem.get("status") != "connected":
        blockers.append("native_driver_not_connected")
    if brainstem.get("ready") is not True:
        blockers.append("native_driver_not_ready")
    if brainstem.get("stale") is True or brainstem.get("status") == "stale":
        blockers.append("native_driver_status_stale")
    return blockers, dict(brainstem)




def collect_report(options: argparse.Namespace) -> dict[str, object]:
    global checks, gateway_timeout_sec, gw, non_motion, realtime, requested_env, require_camera, ros2_enabled, strict

    gw = options.gateway_url.rstrip("/")
    gateway_timeout_sec = options.gateway_timeout_sec
    if not math.isfinite(gateway_timeout_sec) or gateway_timeout_sec <= 0.0:
        raise ValueError("--gateway-timeout-sec must be finite and positive")
    strict = options.strict
    non_motion = options.non_motion
    realtime = options.realtime
    require_camera = options.require_camera
    ros2_enabled = options.ros2
    requested_env = options.env
    checks = []

    current_plan = None
    current_plan_evidence = {"current_run_path": current_run_path()}
    try:
        current_plan, current_plan_evidence = load_current_plan()
    except Exception as exc:
        current_plan_evidence["error"] = str(exc)
        add(
            "run.current_plan",
            "fail",
            "p0",
            "current RunPlan is missing, invalid, or incompatible",
            current_plan_evidence,
        )
    else:
        add(
            "run.current_plan",
            "pass",
            "p0",
            "current RunPlan is verified and compatible",
            current_plan_evidence,
        )

    dataflow_code, dataflow, dataflow_err = http_json(
        "/api/v1/runtime/dataflow",
        timeout=gateway_timeout_sec,
    )
    dataflow_blockers, dataflow_evidence = runtime_dataflow_alignment(
        current_plan,
        dataflow,
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

    required_process_names = {
        process.name for process in (current_plan.processes if current_plan else ())
    }
    required_services = {
        process.target
        for process in (current_plan.processes if current_plan else ())
        if process.manager == "systemd"
    }
    if require_camera:
        required_services.add("lingtu-camera-dds.service")
    require_camera = require_camera or "camera" in required_process_names


    def process_check_status(process_name, ok):
        if ok:
            return "pass"
        return "fail" if process_name in required_process_names else "warn"


    def process_check_level(process_name):
        return "p0" if process_name in required_process_names else "p1"


    for svc in sorted(required_services):
        rc, out, err = run(["systemctl", "is-active", svc], timeout=2)
        active = out.splitlines()[0] if out else "unknown"
        if active == "active":
            add(f"service.{svc}", "pass", "p0", f"{svc} is active", {"active": active})
        else:
            add(f"service.{svc}", "fail", "p0", f"{svc} is not active", {"active": active, "stderr": err})

    legacy_services = [
        "robot-lidar.service",
        "robot-fastlio2.service",
        "robot-localizer.service",
        "lidar.service",
        "slam.service",
        "localizer.service",
        "slam_pgo.service",
    ]
    active_legacy = []
    for svc in legacy_services:
        _, out, _ = run(["systemctl", "is-active", svc], timeout=2)
        if out.splitlines()[:1] == ["active"]:
            active_legacy.append(svc)
    if active_legacy:
        add(
            "service.unmanaged_known",
            "warn",
            "p1",
            "known services outside the active Product are running",
            {"active": active_legacy},
        )
    else:
        add("service.unmanaged_known", "pass", "p2", "no known undeclared services are running")

    livox_service_env, livox_service_env_evidence = systemd_environment("lingtu-livox-dds.service")
    lidar_netdev_name = (
        os.environ.get("LINGTU_LIDAR_NETDEV")
        or os.environ.get("LINGTU_LIVOX_NET_IFACE")
        or livox_service_env.get("LINGTU_LIVOX_NET_IFACE")
        or "eth0"
    )
    lidar_netdev = netdev_state(lidar_netdev_name)
    lidar_netdev["selection"] = {
        "LINGTU_LIDAR_NETDEV": os.environ.get("LINGTU_LIDAR_NETDEV"),
        "LINGTU_LIVOX_NET_IFACE": os.environ.get("LINGTU_LIVOX_NET_IFACE"),
        "service_environment": livox_service_env.get("LINGTU_LIVOX_NET_IFACE"),
        "service_environment_probe": livox_service_env_evidence,
    }
    if lidar_netdev.get("carrier") == "1":
        add("livox.netdev_carrier", "pass", "p0", "LiDAR network interface has carrier", lidar_netdev)
    elif "carrier" in lidar_netdev:
        add(
            "livox.netdev_carrier",
            process_check_status("lidar", False),
            process_check_level("lidar"),
            "LiDAR network interface has no carrier",
            lidar_netdev,
        )
    else:
        add("livox.netdev_carrier", "warn", "p1", "could not inspect LiDAR network interface carrier", lidar_netdev)

    livox_event, livox_evidence = latest_livox_sdk_event()
    if livox_event == "success":
        add("livox.sdk_init", "pass", "p0", "latest Livox SDK init event succeeded", livox_evidence)
    elif livox_event == "fail":
        add(
            "livox.sdk_init",
            process_check_status("lidar", False),
            process_check_level("lidar"),
            "latest Livox SDK init event failed",
            livox_evidence,
        )
    else:
        add("livox.sdk_init", "warn", "p1", "could not find a recent Livox SDK init event", livox_evidence)

    if ros2_enabled:
        rc, nodes_out, nodes_err = run(["ros2", "node", "list"], timeout=5)
        if rc == 127:
            add("ros.nodes", "fail", "p0", "ros2 CLI not found", {"stderr": nodes_err})
        else:
            nodes = [line.strip() for line in nodes_out.splitlines() if line.strip()]
            critical = {
                "/livox_driver_node",
                "/lio_node",
                "/relocation_node",
                "/localizer_node",
                "/camera/camera",
                "/camera/camera_container",
            }
            counts = Counter(nodes)
            dup = sorted(node for node, count in counts.items() if count > 1 and node in critical)
            if dup:
                add("ros.duplicate_nodes", "fail", "p0", "duplicate critical ROS nodes detected", {"duplicates": dup})
            else:
                add(
                    "ros.duplicate_nodes",
                    "pass",
                    "p0",
                    "no duplicate critical ROS node names; camera component/container pair is expected",
                    {"count": len(nodes)},
                )

        required_topics = ["/lidar/raw_frame", "/imu/raw", "/slam/odometry", "/slam/localization_health"]
        for topic in required_topics:
            topic_process = "lidar" if topic in {"/lidar/raw_frame", "/imu/raw"} else "slam"
            pubs, subs, err = topic_counts(topic)
            if err == "ros2 not found":
                add(
                    f"topic.{topic}",
                    process_check_status(topic_process, False),
                    process_check_level(topic_process),
                    "ros2 CLI not found",
                    {"topic": topic},
                )
                continue
            if pubs is None:
                add(
                    f"topic.{topic}",
                    process_check_status(topic_process, False),
                    process_check_level(topic_process),
                    f"could not inspect {topic}",
                    {"error": err},
                )
            elif pubs == 0:
                add(
                    f"topic.{topic}",
                    process_check_status(topic_process, False),
                    process_check_level(topic_process),
                    f"{topic} has no publishers",
                    {"publishers": pubs, "subscribers": subs, "error": err},
                )
            elif topic in {"/lidar/raw_frame", "/imu/raw", "/slam/odometry"} and pubs > 1:
                add(f"topic.{topic}", "fail", "p0", f"{topic} has duplicate publishers", {"publishers": pubs, "subscribers": subs})
            else:
                add(f"topic.{topic}", "pass", "p0", f"{topic} has publishers", {"publishers": pubs, "subscribers": subs})

        for topic in ["/slam/map_cloud", "/slam/saved_map_cloud"]:
            pubs, subs, err = topic_counts(topic)
            if pubs is None:
                add(f"topic.{topic}", "warn", "p1", f"could not inspect {topic}", {"error": err})
            elif pubs == 0:
                add(f"topic.{topic}", "warn", "p1", f"{topic} has no publishers", {"publishers": pubs, "subscribers": subs})
            else:
                add(f"topic.{topic}", "pass", "p1", f"{topic} has publishers", {"publishers": pubs, "subscribers": subs})

        quality_pubs, quality_subs, quality_err = topic_counts("/slam/localization_quality")
        if quality_pubs is None:
            add("topic./slam/localization_quality", "warn", "p1", "could not inspect /slam/localization_quality", {"error": quality_err})
        elif quality_pubs > 0:
            add(
                "topic./slam/localization_quality",
                "pass",
                "p1",
                "/slam/localization_quality has publishers",
                {"publishers": quality_pubs, "subscribers": quality_subs},
            )
        else:
            legacy_pubs, legacy_subs, legacy_err = topic_counts("/localization_quality")
            if legacy_pubs and legacy_pubs > 0:
                add(
                    "topic./slam/localization_quality",
                    "warn",
                    "p1",
                    "/localization_quality is publishing (legacy; remap to /slam/localization_quality)",
                    {"publishers": 0, "subscribers": quality_subs, "legacy_publishers": legacy_pubs, "legacy_subscribers": legacy_subs},
                )
            else:
                add(
                    "topic./slam/localization_quality",
                    "warn",
                    "p1",
                    "/slam/localization_quality has no publishers",
                    {"publishers": quality_pubs, "subscribers": quality_subs, "legacy_error": legacy_err},
                )
    else:
        add(
            "ros.compat_skipped",
            "pass",
            "p2",
            "ROS2 compatibility graph checks skipped; use --ros2 to inspect legacy topics and nodes",
            {"ros2_enabled": False},
        )

    ready_code, ready, ready_err = http_json("/ready", timeout=gateway_timeout_sec)
    if ready_err or not isinstance(ready, dict):
        add("gateway.ready", "fail", "p0", "/ready unavailable", {"http_status": ready_code, "error": ready_err})
    else:
        is_ready = bool(ready.get("ready"))
        data_ready = ready.get("data_ready")
        if data_ready is None:
            data_ready = is_ready
        motion_ready = ready.get("motion_ready")
        if motion_ready is None:
            motion_ready = is_ready
        non_motion_safe = ready.get("non_motion_safe")
        if non_motion_safe is None:
            non_motion_safe = True
        if non_motion:
            if not as_bool(data_ready):
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
            "gateway.ready",
            ready_status,
            "p0",
            ready_message,
            {
                "http_status": ready_code,
                "ready": is_ready,
                "data_ready": as_bool(data_ready),
                "motion_ready": as_bool(motion_ready),
                "non_motion_safe": as_bool(non_motion_safe),
                "reasons": ready.get("reasons", []),
                "failed_modules": ready.get("failed_modules", []),
                "data_blockers": (ready.get("runtime") or {}).get("summary", {}).get("data_blockers", []),
            },
        )
        for module_name, port_name, level in [
            ("VoxelGridModule", "map_cloud", "p1"),
            ("OccupancyGridModule", "map_cloud", "p1"),
            ("ESDFModule", "occupancy_grid", "p1"),
            ("TraversabilityCostModule", "costmap", "p1"),
            ("GatewayModule", "map_cloud", "p1"),
            ("GatewayModule", "localization_status", "p2"),
            ("PerceptionModule", "color_image", "p2"),
        ]:
            add_dataflow_pressure_check(ready, module_name, port_name, level)

    client_ready_code, client_ready, client_ready_err = http_json(
        "/api/v1/readiness",
        timeout=gateway_timeout_sec,
    )
    if client_ready_err or not isinstance(client_ready, dict):
        add(
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

    health_code, health, health_err = http_json(
        "/api/v1/health?details=true",
        timeout=gateway_timeout_sec,
    )
    if health_err or not isinstance(health, dict):
        add("gateway.health", "fail", "p0", "/api/v1/health unavailable", {"http_status": health_code, "error": health_err})
        if "driver" in required_process_names:
            add(
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
                "gateway.driver_status",
                "pass" if not driver_blockers else "fail",
                "p0",
                "native driver and Brainstem status is connected, ready, and fresh"
                if not driver_blockers
                else "native driver and Brainstem status is not ready",
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
            "gateway.slam_stream",
            process_check_status("slam", not stream_blockers),
            process_check_level("slam"),
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

    loc_code, loc, loc_err = http_json(
        "/api/v1/localization/status",
        timeout=gateway_timeout_sec,
    )
    if loc_err or not isinstance(loc, dict):
        add(
            "gateway.localization_status",
            process_check_status("slam", False),
            process_check_level("slam"),
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
            "gateway.localization_status",
            process_check_status("slam", not blockers),
            process_check_level("slam"),
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
                "gateway.localization_confidence",
                "pass" if confidence >= MIN_LOCALIZATION_CONFIDENCE else "warn",
                "p1",
                "localization confidence is above warning threshold"
                if confidence >= MIN_LOCALIZATION_CONFIDENCE
                else "localization confidence is below warning threshold",
                {"confidence": confidence, "min_confidence": MIN_LOCALIZATION_CONFIDENCE},
            )

    nav_code, nav, nav_err = http_json(
        "/api/v1/navigation/status",
        timeout=gateway_timeout_sec,
    )
    if nav_err or not isinstance(nav, dict):
        add(
            "gateway.navigation_status",
            process_check_status("nav", False),
            process_check_level("nav"),
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
            dict.fromkeys(
                blocker
                for blocker in list(blockers) + list(reason_codes)
                if blocker in DATA_NAV_BLOCKERS
            )
        )
        nav_ok = can_accept and not blockers
        if non_motion:
            nav_status = "fail" if data_blockers else ("warn" if blockers or not can_accept else "pass")
            nav_message = (
                "navigation data path is ready for non-motion verification"
                if nav_status == "pass"
                else "navigation has data blockers"
                if nav_status == "fail"
                else "navigation is not goal-ready but data path has no hard blockers"
            )
        else:
            nav_status = "pass" if nav_ok else "fail"
            nav_message = "navigation can accept goals" if nav_ok else "navigation readiness has blockers"
        if nav_status == "fail" and "nav" not in required_process_names:
            nav_status = "warn"
        add(
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
                "safety.non_motion_guard",
                "pass" if source_idle and state_idle else "fail",
                "p0",
                "no active command source or executing mission" if source_idle and state_idle else "robot appears to have an active command source or mission",
                {"state": state, "active_cmd_source": active_source},
            )

    state_code, state_payload, state_err = http_json(
        "/api/v1/state",
        timeout=gateway_timeout_sec,
    )
    if state_err or not isinstance(state_payload, dict):
        add("gateway.state", "warn", "p1", "/api/v1/state unavailable", {"http_status": state_code, "error": state_err})
    else:
        session = state_payload.get("session") or {}
        localization = state_payload.get("localization") or {}
        add(
            "gateway.state",
            "pass",
            "p1",
            "state snapshot available",
            {
                "http_status": state_code,
                "session_mode": session.get("mode"),
                "active_map": session.get("active_map"),
                "localization_backend": localization.get("backend") or localization.get("localization_backend"),
                "localization_state": localization.get("reported_state") or localization.get("state"),
                "confidence": localization.get("confidence"),
            },
        )

    if non_motion and "nav" in required_process_names:
        x, y, z, preview_origin = planning_pose_from_state(state_payload)
        attempts = []
        preview_ok = False
        preview_feasible = False
        if x is None or y is None:
            add(
                "gateway.navigation_plan_preview",
                "warn",
                "p1",
                "navigation plan preview skipped because the map-frame robot pose is unavailable",
                {
                    "http_status": state_code,
                    "state_error": state_err,
                    "preview_origin": preview_origin,
                },
            )
        else:
            offsets = [
                ("west", -PLAN_PREVIEW_OFFSET_M, 0.0),
                ("south", 0.0, -PLAN_PREVIEW_OFFSET_M),
                ("east", PLAN_PREVIEW_OFFSET_M, 0.0),
                ("north", 0.0, PLAN_PREVIEW_OFFSET_M),
                ("same", 0.0, 0.0),
            ]
            for label, dx, dy in offsets:
                body = {"x": x + dx, "y": y + dy, "z": z}
                code, preview, err = http_json(
                    "/api/v1/navigation/plan",
                    timeout=max(8.0, gateway_timeout_sec),
                    method="POST",
                    payload=body,
                )
                attempt = {
                    "label": label,
                    "http_status": code,
                    "feasible": bool(preview.get("feasible")) if isinstance(preview, dict) else False,
                    "count": preview.get("count") if isinstance(preview, dict) else None,
                    "planner": preview.get("planner") if isinstance(preview, dict) else None,
                    "reasons": preview.get("reasons") if isinstance(preview, dict) else [],
                    "error": err or (preview.get("error") if isinstance(preview, dict) else None),
                }
                attempts.append(attempt)
                preview_ok = preview_ok or (
                    code == 200 and isinstance(preview, dict) and preview.get("schema_version") == 1
                )
                if attempt["feasible"] and int(attempt["count"] or 0) > 1:
                    preview_feasible = True
                    break
            nav_after_code, nav_after, nav_after_err = http_json(
                "/api/v1/navigation/status",
                timeout=gateway_timeout_sec,
            )
            nav_after_ok = nav_after_code == 200 and isinstance(nav_after, dict)
            active_after = command_source_name((nav_after or {}).get("control") or {}) if nav_after_ok else "unavailable"
            state_after = navigation_state_name(nav_after) if nav_after_ok else "unavailable"
            source_idle_after = str(active_after).lower() in {"", "none", "null"}
            state_idle_after = navigation_state_is_idle(state_after) if nav_after_ok else False
            if not preview_ok:
                status = "fail"
                message = "navigation plan preview endpoint is unavailable or unversioned"
            elif not nav_after_ok:
                status = "fail"
                message = "navigation plan preview could not verify non-motion state afterwards"
            elif not source_idle_after:
                status = "fail"
                message = "navigation plan preview changed the active command source"
            elif not state_idle_after:
                status = "fail"
                message = "navigation plan preview changed the mission state"
            elif preview_feasible:
                status = "pass"
                message = "navigation plan preview produced a path without taking control"
            else:
                status = "warn"
                message = "navigation plan preview endpoint works but nearby moving candidates were not feasible"
            add(
                "gateway.navigation_plan_preview",
                status,
                "p0" if status == "fail" else "p1",
                message,
                {
                    "offset_m": PLAN_PREVIEW_OFFSET_M,
                    "preview_origin": preview_origin,
                    "attempts": attempts,
                    "active_cmd_source_after": active_after,
                    "state_after": state_after,
                    "nav_after_http_status": nav_after_code,
                    "nav_after_error": nav_after_err,
                },
            )

    rc, lsusb_out, _ = run(["lsusb"], timeout=3)
    usb_text = lsusb_out.lower()
    camera_usb = any(token in usb_text for token in ["orbbec", "gemini", "astra", "2bc5"])
    add(
        "camera.usb",
        camera_check_status(camera_usb),
        camera_check_level(),
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
        "camera.gateway_snapshot",
        camera_check_status(camera_snapshot_ok),
        camera_check_level(),
        "Gateway camera snapshot endpoint returned JPEG"
        if camera_snapshot_ok
        else "Gateway camera snapshot endpoint did not return JPEG",
        camera_snapshot_evidence,
    )

    camera_topics_ready = False
    if ros2_enabled:
        camera_topic_counts = {}
        for topic in ["/camera/color/image_raw", "/camera/depth/image_raw"]:
            pubs, subs, err = topic_counts(topic)
            camera_topic_counts[topic] = {"publishers": pubs, "subscribers": subs, "error": err}
            if pubs is None:
                add(f"camera.topic.{topic}", camera_check_status(False), camera_check_level(), f"could not inspect {topic}", {"error": err, "required": require_camera})
            elif pubs == 0:
                add(f"camera.topic.{topic}", camera_check_status(False), camera_check_level(), f"{topic} has no publishers", {"publishers": pubs, "subscribers": subs, "required": require_camera})
            else:
                add(f"camera.topic.{topic}", "pass", camera_check_level(), f"{topic} has publishers", {"publishers": pubs, "subscribers": subs, "required": require_camera})
        camera_topics_ready = all((camera_topic_counts.get(topic, {}).get("publishers") or 0) > 0 for topic in ["/camera/color/image_raw", "/camera/depth/image_raw"])
        camera_info_topics = [
            "/camera/color/camera_info",
            "/camera/depth/camera_info",
            "/camera/camera_info",
        ]
        camera_info_counts = {}
        for topic in camera_info_topics:
            pubs, subs, err = topic_counts(topic)
            camera_info_counts[topic] = {"publishers": pubs, "subscribers": subs, "error": err}
        camera_info_ready = any(
            (counts.get("publishers") or 0) > 0
            for counts in camera_info_counts.values()
        )
        add(
            "camera.camera_info",
            camera_check_status(camera_info_ready),
            camera_check_level(),
            "camera intrinsics topic has publishers"
            if camera_info_ready
            else "camera intrinsics topic has no publishers",
            {"topics": camera_info_counts, "required": require_camera},
        )
        device_status_pubs, device_status_subs, device_status_err = topic_counts("/camera/device_status")
        if device_status_pubs is None:
            add(
                "camera.device_status",
                camera_check_status(False),
                camera_check_level(),
                "could not inspect /camera/device_status",
                {"error": device_status_err, "required": require_camera},
            )
        elif device_status_pubs == 0:
            add(
                "camera.device_status",
                camera_check_status(False),
                camera_check_level(),
                "/camera/device_status has no publishers",
                {"publishers": device_status_pubs, "subscribers": device_status_subs, "error": device_status_err, "required": require_camera},
            )
        else:
            device_status, device_status_err = camera_device_status_once()
            if device_status is None:
                add(
                    "camera.device_status",
                    camera_check_status(False),
                    camera_check_level(),
                    "could not read /camera/device_status sample",
                    {"publishers": device_status_pubs, "subscribers": device_status_subs, "error": device_status_err, "required": require_camera},
                )
            elif device_status.get("device_online") is True:
                add(
                    "camera.device_status",
                    "pass",
                    camera_check_level(),
                    "camera device_status reports online",
                    {"publishers": device_status_pubs, "subscribers": device_status_subs, "required": require_camera, **device_status},
                )
            elif device_status.get("device_online") is False:
                add(
                    "camera.device_status",
                    camera_check_status(False),
                    camera_check_level(),
                    "camera driver is alive but /camera/device_status reports device_online=false; "
                    "check USB/power/cable before restarting software",
                    {"publishers": device_status_pubs, "subscribers": device_status_subs, "required": require_camera, **device_status},
                )
            else:
                add(
                    "camera.device_status",
                    camera_check_status(False),
                    camera_check_level(),
                    "camera device_status sample did not include device_online",
                    {"publishers": device_status_pubs, "subscribers": device_status_subs, "required": require_camera, **device_status},
                )
    else:
        add(
            "camera.ros2_topics_skipped",
            "pass",
            "p2",
            "ROS2 camera topic checks skipped; use --ros2 for legacy camera topic inspection",
            {"ros2_enabled": False},
        )
    video_nodes = sorted(glob.glob("/dev/video*") + glob.glob("/dev/v4l/by-id/*"))
    if video_nodes:
        add("camera.video_nodes", "pass", camera_check_level(), "video devices exist", {"nodes": video_nodes[:8], "required": require_camera})
    elif camera_usb and camera_topics_ready:
        add(
            "camera.video_nodes",
            "pass",
            camera_check_level(),
            "SDK camera is publishing ROS topics without V4L nodes",
            {"nodes": [], "camera_topics_ready": True, "required": require_camera},
        )
    else:
        add(
            "camera.video_nodes",
            camera_check_status(False),
            camera_check_level(),
            "no /dev/video or /dev/v4l/by-id devices found",
            {"nodes": [], "camera_topics_ready": camera_topics_ready, "required": require_camera},
        )

    if realtime:
        sse_ok, sse_evidence = sse_smoke(REALTIME_TIMEOUT_S)
        add(
            "gateway.realtime_sse",
            "pass" if sse_ok else "fail",
            "p1",
            "SSE event stream produced data" if sse_ok else "SSE event stream did not produce data",
            sse_evidence,
        )
        cam_ok, cam_evidence = ws_smoke("/ws/camera", b"\xff\xd8\xff", REALTIME_TIMEOUT_S)
        add(
            "gateway.websocket_camera",
            camera_check_status(cam_ok),
            camera_check_level(),
            "camera WebSocket produced a JPEG frame" if cam_ok else "camera WebSocket did not produce a JPEG frame",
            cam_evidence,
        )
        cloud_ok, cloud_evidence = ws_smoke("/ws/cloud", b"PCL", REALTIME_TIMEOUT_S)
        add(
            "gateway.websocket_cloud",
            "pass" if cloud_ok else "fail",
            "p1",
            "cloud WebSocket produced a PCL frame" if cloud_ok else "cloud WebSocket did not produce a PCL frame",
            cloud_evidence,
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
        "blockers": [check for check in checks if check["status"] == "fail"],
        "warnings": [check for check in checks if check["status"] == "warn"],
        "checks": checks,
    }
    return report


def _render_check(check: dict[str, object]) -> None:
    status = str(check.get("status") or "warn")
    message = str(check.get("message") or check.get("id") or "check")
    evidence = check.get("evidence")
    if check.get("id") == "gateway.navigation_plan_preview" and status == "pass":
        preview_evidence = evidence if isinstance(evidence, dict) else {}
        source = preview_evidence.get("active_cmd_source_after", "none")
        state = preview_evidence.get("state_after", "unknown")
        print(f"  \033[32mok\033[0m: preview did not take control; active_cmd_source_after={source} state_after={state}")
        return
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
    if options.ros2:
        print("Mode: ROS2 compatibility graph inspection enabled")

    section_rules = (
        ("[1] Services", lambda check_id: check_id == "run.current_plan" or check_id.startswith("service.")),
        ("[2] Livox SDK init", lambda check_id: check_id.startswith("livox.")),
        ("[3] ROS duplicate nodes", lambda check_id: check_id.startswith("ros.")),
        ("[4] Runtime dataflow", lambda check_id: check_id.startswith("dataflow.") or check_id.startswith("topic.")),
        ("[5] Gateway", lambda check_id: check_id.startswith("gateway.") and check_id not in {"gateway.navigation_status", "gateway.navigation_plan_preview"}),
        ("[6] Navigation readiness", lambda check_id: check_id.startswith("safety.") or check_id == "gateway.navigation_status"),
        ("[6b] Navigation plan preview (non-motion)", lambda check_id: check_id == "gateway.navigation_plan_preview"),
        ("[7] Camera readiness", lambda check_id: check_id.startswith("camera.")),
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
        (index, check)
        for index, check in enumerate(report_checks)
        if index not in rendered and isinstance(check, dict)
    ]
    if remaining:
        print(f"{bold}[5] Gateway{reset}")
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
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--require-camera", action="store_true")
    parser.add_argument("--ros2", action="store_true")
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

#!/usr/bin/env python3
"""Collect read-only Thunder service readiness evidence.

This script is safe to run on the robot or through SSH. It does not start,
stop, restart, enable, or publish anything; it only reads systemd state, status
files, Gateway health endpoints, and process listings.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from diagnostics.field.teleop_avoid_preflight import STAGES as TELEOP_AVOID_STAGES  # noqa: E402
from diagnostics.field.teleop_avoid_preflight import evaluate_teleop_avoid_preflight  # noqa: E402
from lingtu.product_lock import resolve_current_run_path  # noqa: E402
from lingtu.run_plan import RunPlan  # noqa: E402
from runtime.config import load_config  # noqa: E402
from runtime.service_catalogs.thunder import (  # noqa: E402
    thunder_runtime_dds_topics,
    thunder_runtime_native_binaries,
    thunder_runtime_units,
    thunder_runtime_status_files,
)

SERVICES = thunder_runtime_units()
STATUS_FILES = thunder_runtime_status_files()
DDS_TOPICS = thunder_runtime_dds_topics()
NATIVE_BINARIES = thunder_runtime_native_binaries()


NATIVE_BINARY_DEFAULTS: dict[str, str] = {name: item["path"] for name, item in NATIVE_BINARIES.items()}

NATIVE_BINARY_ENV: dict[str, str] = {name: item["env"] for name, item in NATIVE_BINARIES.items()}

LIDAR_IMU_EXPECTED_PROCESS = "livox_sdk2_stream"
LIDAR_IMU_EXPECTED_BINARY = "livox_dds"
LIDAR_IMU_EXPECTED_SERVICE = "lidar"
LIDAR_IMU_RAW_TOPIC = "rt/lidar/raw_frame"
LIDAR_IMU_IMU_TOPIC = "rt/imu/raw"
CAMERA_SERVICE_UNIT = "lingtu-camera-dds.service"
CAMERA_STATUS_NAME = "camera"
CAMERA_NATIVE_BINARIES = ("camera_dds", "orbbec_capture")
CAMERA_DDS_TOPICS = ("rt/camera/info",)
CAMERA_STATUS_FRAME_KEYS = ("color_frames", "depth_frames", "info_frames")
CAMERA_SHM_SEQUENCE_KEYS = (
    "color_shm_sequence",
    "depth_shm_sequence",
    "info_shm_sequence",
)
DRIVER_SERVICE_UNIT = "lingtu-driver.service"
DRIVER_STATUS_NAME = "driver"
DRIVER_NATIVE_BINARY = "driver"
DRIVER_DDS_TOPIC = "rt/nav/cmd_vel"
DRIVER_STATUS_MAX_AGE_S = 3.0
LEGACY_SENSOR_PROCESS_NEEDLES = (
    "livox_ros_driver2",
    "orbbec_camera",
)
IMU_DUPLICATE_PROCESS_NEEDLES = (
    "lingtu_imu_dds",
    "imu_publisher",
    "robot_imu",
    "robot-imu",
    "livox_imu",
    "microstrain",
    "xsens",
)


def _run(command: list[str], *, timeout: float = 5.0) -> dict[str, Any]:
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return {
            "ok": result.returncode == 0,
            "returncode": result.returncode,
            "stdout": result.stdout,
            "stderr": result.stderr,
        }
    except Exception as exc:
        return {"ok": False, "returncode": None, "stdout": "", "stderr": str(exc)}


def _parse_systemctl_show(text: str) -> dict[str, str]:
    parsed: dict[str, str] = {}
    for line in text.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        parsed[key] = value
    return parsed


def collect_systemd() -> dict[str, Any]:
    units: dict[str, Any] = {}
    for unit in SERVICES:
        show = _run(
            [
                "systemctl",
                "show",
                unit,
                "-p",
                "Id",
                "-p",
                "ActiveState",
                "-p",
                "SubState",
                "-p",
                "LoadState",
                "-p",
                "UnitFileState",
                "-p",
                "NRestarts",
                "--no-page",
            ]
        )
        data = _parse_systemctl_show(show["stdout"])
        load_state = data.get("LoadState", "")
        units[unit] = {
            "ok": show["ok"],
            "id": data.get("Id", unit),
            "load_state": load_state,
            "unit_file_state": data.get("UnitFileState", ""),
            "active_state": data.get("ActiveState", ""),
            "sub_state": data.get("SubState", ""),
            "restarts": _int_or_none(data.get("NRestarts")),
            "missing": load_state in {"not-found", "masked"},
            "error": show["stderr"].strip(),
        }
    return units


def _int_or_none(value: str | None) -> int | None:
    if value is None or value == "":
        return None
    try:
        return int(value)
    except ValueError:
        return None


def collect_status_files() -> dict[str, Any]:
    files: dict[str, Any] = {}
    for name, raw_path in STATUS_FILES.items():
        path = Path(raw_path)
        entry: dict[str, Any] = {"path": raw_path, "exists": path.exists()}
        if path.exists():
            try:
                entry["age_s"] = max(0.0, time.time() - path.stat().st_mtime)
            except OSError:
                entry["age_s"] = None
            text = path.read_text(encoding="utf-8", errors="replace")
            try:
                entry["json"] = json.loads(text)
            except json.JSONDecodeError:
                entry["raw"] = text[:4096]
        files[name] = entry
    return files


def _current_run_path() -> Path:
    return resolve_current_run_path(environment=os.environ)


def collect_current_run() -> dict[str, Any]:
    """Read and cryptographically verify the current RunPlan record."""

    current_path = _current_run_path()
    result: dict[str, Any] = {
        "path": str(current_path),
        "exists": current_path.is_file(),
        "state": {},
        "run_plan_path": "",
        "plan_exists": False,
        "plan_verified": False,
        "verified_fingerprint": "",
        "run_plan": {},
        "error": "",
    }
    if not result["exists"]:
        result["error"] = "current_run_missing"
        return result

    try:
        state = json.loads(current_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        result["error"] = f"current_run_invalid:{type(exc).__name__}"
        return result
    if not isinstance(state, dict):
        result["error"] = "current_run_not_object"
        return result
    result["state"] = state

    raw_run_plan_path = str(state.get("run_plan_path") or "").strip()
    if not raw_run_plan_path:
        result["error"] = "current_run_run_plan_path_missing"
        return result
    run_plan_path = Path(raw_run_plan_path).expanduser()
    if not run_plan_path.is_absolute():
        run_plan_path = current_path.parent / run_plan_path
    result["run_plan_path"] = str(run_plan_path)
    result["plan_exists"] = run_plan_path.is_file()
    if not result["plan_exists"]:
        result["error"] = "current_run_plan_missing"
        return result

    try:
        run_plan = RunPlan.load(run_plan_path)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        result["error"] = f"current_run_plan_invalid:{type(exc).__name__}:{exc}"
        return result
    result.update(
        {
            "plan_verified": True,
            "verified_fingerprint": run_plan.fingerprint,
            "run_plan": run_plan.as_dict(),
        }
    )
    return result


def collect_native_binaries() -> dict[str, Any]:
    binaries: dict[str, Any] = {}
    blockers: list[str] = []
    for name, default_path in NATIVE_BINARY_DEFAULTS.items():
        env_var = NATIVE_BINARY_ENV[name]
        raw_path = os.environ.get(env_var, default_path)
        path = Path(raw_path)
        exists = path.exists()
        executable = exists and os.access(path, os.X_OK)
        entry = {
            "path": raw_path,
            "env": env_var,
            "exists": exists,
            "executable": executable,
        }
        if not executable:
            blockers.append(f"native_binary_missing_or_not_executable:{name}:{raw_path}")
        binaries[name] = entry
    return {"ok": not blockers, "binaries": binaries, "blockers": blockers}


def collect_gnss_device() -> dict[str, Any]:
    """Collect read-only GNSS config and serial-device readiness evidence."""
    try:
        cfg = load_config()
    except Exception as exc:
        return {
            "ok": False,
            "configured": False,
            "enabled": False,
            "blockers": [f"gnss_config_error:{type(exc).__name__}"],
        }

    gnss = cfg.gnss
    raw_gnss = cfg.raw.get("gnss", {}) if isinstance(cfg.raw, dict) else {}
    rtcm = raw_gnss.get("rtcm", {}) if isinstance(raw_gnss, dict) else {}
    device = str(getattr(gnss, "device", "") or "")
    exists = bool(device and Path(device).exists())
    symlink_exists = bool(device and os.path.lexists(device))
    resolved = ""
    if symlink_exists:
        try:
            resolved = str(Path(device).resolve(strict=False))
        except Exception:
            resolved = ""

    blockers: list[str] = []
    enabled = bool(getattr(gnss, "enabled", False))
    if enabled and not exists:
        blockers.append(f"gnss_device_missing:{device or '<unset>'}")

    return {
        "ok": not blockers,
        "configured": isinstance(raw_gnss, dict) and bool(raw_gnss),
        "enabled": enabled,
        "model": str(getattr(gnss, "model", "")),
        "device": device,
        "baud": int(getattr(gnss, "baud", 0) or 0),
        "device_exists": exists,
        "symlink_exists": symlink_exists,
        "resolved_device": resolved,
        "rtcm_enabled": bool(rtcm.get("enabled", False)) if isinstance(rtcm, dict) else False,
        "fusion_enabled": bool(getattr(getattr(gnss, "fusion", object()), "enabled", False)),
        "blockers": blockers,
    }


def _http_json(url: str, *, timeout: float = 3.0) -> dict[str, Any]:
    try:
        with urlopen(url, timeout=timeout) as response:
            raw = response.read().decode("utf-8", errors="replace")
            payload: Any
            try:
                payload = json.loads(raw)
            except json.JSONDecodeError:
                payload = raw[:4096]
            return {
                "ok": 200 <= int(response.status) < 300,
                "status": int(response.status),
                "body": payload,
            }
    except HTTPError as exc:
        return {"ok": False, "status": int(exc.code), "body": exc.reason}
    except URLError as exc:
        return {"ok": False, "status": None, "body": str(exc.reason)}
    except Exception as exc:
        return {"ok": False, "status": None, "body": str(exc)}


def collect_gateway(base_url: str) -> dict[str, Any]:
    base = base_url.rstrip("/")
    return {
        "health": _http_json(f"{base}/health"),
        "services_status": _http_json(f"{base}/api/v1/services/status", timeout=30.0),
    }


def collect_processes() -> dict[str, Any]:
    result = _run(["ps", "-eo", "pid,comm,args"], timeout=5.0)
    needles = (
        "lingtu_camera",
        "lingtu_camera_dds",
        "orbbec",
        LIDAR_IMU_EXPECTED_PROCESS,
        "lingtu_slam",
        "lingtu_nav",
        "lingtu_traversability",
        "lingtu_driver",
        *IMU_DUPLICATE_PROCESS_NEEDLES,
    )
    lines = [line for line in result["stdout"].splitlines() if any(needle in line.lower() for needle in needles)]
    legacy_lines = [
        line
        for line in result["stdout"].splitlines()
        if any(needle in line.lower() for needle in LEGACY_SENSOR_PROCESS_NEEDLES)
    ]
    duplicate_imu_lines = [
        line
        for line in result["stdout"].splitlines()
        if any(needle in line.lower() for needle in IMU_DUPLICATE_PROCESS_NEEDLES)
    ]
    blockers = [
        f"legacy_process_observed:{needle}"
        for needle in LEGACY_SENSOR_PROCESS_NEEDLES
        if any(needle in line.lower() for line in legacy_lines)
    ]
    return {
        "ok": result["ok"] and not blockers,
        "lines": lines,
        "legacy_lines": legacy_lines,
        "duplicate_imu_lines": duplicate_imu_lines,
        "blockers": blockers,
        "error": result["stderr"].strip(),
    }


def collect_lidar_imu_ownership(
    *,
    processes: dict[str, Any],
    dds: dict[str, Any],
) -> dict[str, Any]:
    """Check that field LiDAR raw frames and IMU samples share the Livox owner."""

    lines = list(processes.get("lines", []))
    legacy_lines = list(processes.get("legacy_lines", []))
    duplicate_imu_lines = list(processes.get("duplicate_imu_lines", []))
    native_observed = any(LIDAR_IMU_EXPECTED_PROCESS in line.lower() for line in lines)
    legacy_livox_observed = any("livox_ros_driver2" in line.lower() for line in legacy_lines)

    duplicate_needles = [
        needle
        for needle in IMU_DUPLICATE_PROCESS_NEEDLES
        if any(needle in line.lower() for line in duplicate_imu_lines)
    ]

    dds_lidar = dds.get("services", {}).get(LIDAR_IMU_EXPECTED_SERVICE, {})
    dds_topics = list(dds_lidar.get("dds_topics", []))
    samples = dict(dds_lidar.get("samples", {}))
    checked = bool(dds.get("checked", False))
    raw_samples = int(samples.get(LIDAR_IMU_RAW_TOPIC, 0) or 0)
    imu_samples = int(samples.get(LIDAR_IMU_IMU_TOPIC, 0) or 0)

    blockers: list[str] = []
    if not native_observed:
        blockers.append(f"lidar_imu_owner_missing:{LIDAR_IMU_EXPECTED_PROCESS}")
    if legacy_livox_observed:
        blockers.append("legacy_process_observed:livox_ros_driver2")
    blockers.extend(f"imu_duplicate_writer_risk:{needle}" for needle in duplicate_needles)

    return {
        "ok": not blockers,
        "expected_owner": LIDAR_IMU_EXPECTED_BINARY,
        "expected_process": LIDAR_IMU_EXPECTED_PROCESS,
        "expected_service": LIDAR_IMU_EXPECTED_SERVICE,
        "raw_lidar_topic": LIDAR_IMU_RAW_TOPIC,
        "imu_topic": LIDAR_IMU_IMU_TOPIC,
        "same_owner_expected": True,
        "native_process_observed": native_observed,
        "legacy_livox_observed": legacy_livox_observed,
        "duplicate_imu_writer_observed": bool(duplicate_needles),
        "duplicate_imu_needles": duplicate_needles,
        "dds_checked": checked,
        "dds_topics": dds_topics,
        "raw_lidar_samples": raw_samples,
        "imu_samples": imu_samples,
        "blockers": blockers,
    }


def collect_camera_readiness(
    *,
    systemd: dict[str, Any],
    native_binaries: dict[str, Any],
    status_files: dict[str, Any],
    dds: dict[str, Any],
    processes: dict[str, Any],
) -> dict[str, Any]:
    """Collect camera-specific end-to-end readiness evidence."""

    unit = dict(systemd.get(CAMERA_SERVICE_UNIT, {}))
    binaries = dict(native_binaries.get("binaries", {}))
    status = dict(status_files.get(CAMERA_STATUS_NAME, {}))
    status_json = status.get("json", {}) if isinstance(status.get("json"), dict) else {}
    dds_camera = dict(dds.get("services", {}).get(CAMERA_STATUS_NAME, {}))
    dds_samples = dict(dds_camera.get("samples", {}))
    legacy_lines = list(processes.get("legacy_lines", []))

    blockers: list[str] = []
    if unit.get("missing"):
        blockers.append(f"systemd_unit_missing:{CAMERA_SERVICE_UNIT}")
    elif unit.get("active_state") and unit.get("active_state") != "active":
        blockers.append(f"systemd_unit_inactive:{CAMERA_SERVICE_UNIT}:{unit.get('active_state')}")

    for name in CAMERA_NATIVE_BINARIES:
        item = dict(binaries.get(name, {}))
        if not item.get("executable", False):
            blockers.append(f"native_binary_missing_or_not_executable:{name}:{item.get('path', '')}")

    if not status.get("exists", False):
        blockers.append(f"status_file_missing:{status.get('path', STATUS_FILES.get(CAMERA_STATUS_NAME, ''))}")
    else:
        for key in CAMERA_STATUS_FRAME_KEYS:
            if int(status_json.get(key, 0) or 0) <= 0:
                blockers.append(f"status_no_{key}:{status.get('path', '')}")
        if status_json.get("data_plane") != "posix_shm":
            blockers.append(f"status_camera_data_plane_not_shm:{status_json.get('data_plane', '')}")
        for key in CAMERA_SHM_SEQUENCE_KEYS:
            if int(status_json.get(key, 0) or 0) <= 0:
                blockers.append(f"status_no_{key}:{status.get('path', '')}")
        if str(status_json.get("status", "")).lower() == "error":
            blockers.append(f"status_error:{status_json.get('last_error', '')}")

    if dds.get("checked"):
        for topic in CAMERA_DDS_TOPICS:
            if int(dds_samples.get(topic, 0) or 0) <= 0:
                blockers.append(f"dds_topic_silent:{topic}")
    else:
        blockers.append("dds_unchecked")

    if any("orbbec_camera" in line.lower() for line in legacy_lines):
        blockers.append("legacy_process_observed:orbbec_camera")

    return {
        "ok": not blockers,
        "unit": CAMERA_SERVICE_UNIT,
        "status_file": status.get("path", STATUS_FILES.get(CAMERA_STATUS_NAME, "")),
        "native_binaries": {
            name: {
                "path": dict(binaries.get(name, {})).get("path", ""),
                "executable": bool(dict(binaries.get(name, {})).get("executable", False)),
            }
            for name in CAMERA_NATIVE_BINARIES
        },
        "status_frames": {key: int(status_json.get(key, 0) or 0) for key in CAMERA_STATUS_FRAME_KEYS},
        "data_plane": status_json.get("data_plane"),
        "shm_sequences": {key: int(status_json.get(key, 0) or 0) for key in CAMERA_SHM_SEQUENCE_KEYS},
        "dds_checked": bool(dds.get("checked", False)),
        "dds_topics": list(dds_camera.get("dds_topics", CAMERA_DDS_TOPICS)),
        "dds_samples": {topic: int(dds_samples.get(topic, 0) or 0) for topic in CAMERA_DDS_TOPICS},
        "legacy_orbbec_observed": any("orbbec_camera" in line.lower() for line in legacy_lines),
        "blockers": blockers,
    }


def collect_driver_readiness(
    *,
    systemd: dict[str, Any],
    native_binaries: dict[str, Any],
    status_files: dict[str, Any],
) -> dict[str, Any]:
    """Validate the idle-safe driver boundary without requiring cmd_vel traffic."""

    unit = dict(systemd.get(DRIVER_SERVICE_UNIT, {}))
    binary = dict(native_binaries.get("binaries", {}).get(DRIVER_NATIVE_BINARY, {}))
    status = dict(status_files.get(DRIVER_STATUS_NAME, {}))
    payload = status.get("json", {}) if isinstance(status.get("json"), dict) else {}
    brainstem = payload.get("brainstem", {}) if isinstance(payload.get("brainstem"), dict) else {}
    blockers: list[str] = []

    if unit.get("missing"):
        blockers.append(f"systemd_unit_missing:{DRIVER_SERVICE_UNIT}")
    elif unit.get("active_state") != "active":
        blockers.append(f"systemd_unit_inactive:{DRIVER_SERVICE_UNIT}:{unit.get('active_state', 'unknown')}")
    if not binary.get("executable", False):
        blockers.append(f"native_binary_missing_or_not_executable:{DRIVER_NATIVE_BINARY}")
    if not status.get("exists", False):
        blockers.append(f"status_file_missing:{status.get('path', STATUS_FILES.get(DRIVER_STATUS_NAME, ''))}")
    else:
        if payload.get("schema_version") != "lingtu.driver.status.v1":
            blockers.append("status_schema_invalid:driver")
        if payload.get("connected") is not True:
            blockers.append("brainstem_not_connected")
        elif payload.get("ready") is not True:
            blockers.append("driver_not_ready")
        if brainstem:
            if brainstem.get("motors_enabled") is not True:
                blockers.append("brainstem_motors_disabled")
            if brainstem.get("critical_fault") is True:
                blockers.append("brainstem_critical_motor_fault")
            if (
                brainstem.get("lease_valid") is not True
                or brainstem.get("owner") != "grpc"
                or brainstem.get("owner_id") != "lingtu-driver"
            ):
                blockers.append("brainstem_lease_not_owned_by_lingtu")
            if brainstem.get("fsm") not in {"standing", "walking"}:
                blockers.append("brainstem_fsm_not_ready")
        age_s = status.get("age_s")
        if not isinstance(age_s, (int, float)) or age_s > DRIVER_STATUS_MAX_AGE_S:
            blockers.append("driver_status_stale")

    return {
        "ok": not blockers,
        "unit": DRIVER_SERVICE_UNIT,
        "status_file": status.get("path", STATUS_FILES.get(DRIVER_STATUS_NAME, "")),
        "status_age_s": status.get("age_s"),
        "connected": payload.get("connected") is True,
        "ready": payload.get("ready") is True,
        "brainstem": brainstem,
        "dds_topic": DRIVER_DDS_TOPIC,
        "dds_idle_allowed": True,
        "blockers": blockers,
    }


def collect_dds(*, seconds: float = 0.0, domain_id: int = 0) -> dict[str, Any]:
    services = {
        name: {
            "topics": list(contract["topics"]),
            "dds_topics": list(contract["dds_topics"]),
            "samples": {topic: 0 for topic in contract["dds_topics"]},
        }
        for name, contract in DDS_TOPICS.items()
    }
    if seconds <= 0.0:
        return {
            "ok": False,
            "checked": False,
            "enabled": False,
            "seconds": seconds,
            "domain_id": domain_id,
            "services": services,
            "blockers": ["dds_unchecked"],
        }

    topic_index: dict[str, tuple[str, str]] = {}
    probe_topics: list[str] = []
    for service, contract in DDS_TOPICS.items():
        ros_topics = list(contract["topics"])
        dds_topics = list(contract["dds_topics"])
        for index, ros_topic in enumerate(ros_topics):
            dds_topic = dds_topics[index] if index < len(dds_topics) else ros_topic
            topic_index[dds_topic] = (service, ros_topic)
            probe_topics.append(dds_topic)

    blockers: list[str] = []
    try:
        probe_script = Path(
            os.environ.get(
                "LINGTU_DDS_PROBE_SCRIPT",
                str(ROOT / "scripts" / "diagnostics" / "dds_probe.py"),
            )
        )
        result = subprocess.run(
            [
                sys.executable,
                str(probe_script),
                "--json",
                "--seconds",
                str(max(0.1, seconds)),
                "--domain",
                str(domain_id),
                *probe_topics,
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=max(5.0, seconds + 10.0),
            cwd=str(ROOT),
        )
        if result.returncode not in (0, 1):
            blockers.append((result.stderr or "").strip() or f"dds_probe_failed:{result.returncode}")
            rows: list[dict[str, Any]] = []
        else:
            rows = json.loads(result.stdout or "[]")
    except Exception as exc:
        blockers.append(f"dds_check_error:{type(exc).__name__}")
        rows = []

    observed = {str(row.get("topic") or ""): row for row in rows}
    for dds_topic in probe_topics:
        item = observed.get(dds_topic, {})
        service, _ros_topic = topic_index[dds_topic]
        samples = int(item.get("samples", 0) or 0)
        services[service]["samples"][dds_topic] = samples
        services[service].setdefault("details", {})[dds_topic] = {
            "samples": samples,
            "hz": float(item.get("hz", 0.0) or 0.0),
            "frame_id": str(item.get("frame_id", "") or ""),
            "points": item.get("points"),
            "age_s": None,
        }
        if samples <= 0 and dds_topic != DRIVER_DDS_TOPIC:
            blockers.append(f"dds_topic_silent:{dds_topic}")
    for service, item in services.items():
        item["idle_allowed"] = service == DRIVER_STATUS_NAME
        item["ok"] = all(
            count > 0 or topic == DRIVER_DDS_TOPIC
            for topic, count in item["samples"].items()
        )
        if not item["ok"]:
            item.setdefault("blockers", []).extend(
                f"dds_topic_silent:{topic}"
                for topic, count in item["samples"].items()
                if count <= 0 and topic != DRIVER_DDS_TOPIC
            )
    return {
        "ok": not blockers,
        "checked": True,
        "enabled": True,
        "seconds": seconds,
        "domain_id": domain_id,
        "services": services,
        "blockers": blockers,
    }


def summarize_report(report: dict[str, Any]) -> dict[str, Any]:
    blockers: list[str] = []

    for unit, state in report.get("systemd", {}).items():
        if state.get("missing"):
            blockers.append(f"systemd:unit_missing:{unit}")
        elif state.get("active_state") and state.get("active_state") != "active":
            blockers.append(f"systemd:unit_inactive:{unit}:{state.get('active_state')}")

    for name, item in report.get("status_files", {}).items():
        if not item.get("exists", False):
            blockers.append(f"status_file:missing:{name}:{item.get('path', '')}")

    for blocker in report.get("native_binaries", {}).get("blockers", []):
        blockers.append(f"native_binaries:{blocker}")

    for blocker in report.get("gnss", {}).get("blockers", []):
        blockers.append(f"gnss:{blocker}")

    gateway = report.get("gateway", {})
    for name, item in gateway.items():
        if item and not item.get("ok", False):
            blockers.append(f"gateway:{name}:{item.get('status')}")

    for blocker in report.get("processes", {}).get("blockers", []):
        blockers.append(f"processes:{blocker}")

    for blocker in report.get("lidar_imu", {}).get("blockers", []):
        blockers.append(f"lidar_imu:{blocker}")

    for blocker in report.get("camera", {}).get("blockers", []):
        blockers.append(f"camera:{blocker}")

    for blocker in report.get("driver", {}).get("blockers", []):
        blockers.append(f"driver:{blocker}")

    dds = report.get("dds", {})
    if dds.get("checked"):
        for blocker in dds.get("blockers", []):
            blockers.append(f"dds:{blocker}")
    else:
        blockers.append("dds:unchecked")

    return {
        "ok": not blockers,
        "blockers": blockers,
        "blocker_count": len(blockers),
    }


def build_report(
    *,
    gateway_url: str,
    dds_seconds: float = 0.0,
    dds_domain: int = 0,
    teleop_avoid_stage: str | None = None,
    status_max_age_s: float = DRIVER_STATUS_MAX_AGE_S,
) -> dict[str, Any]:
    processes = collect_processes()
    dds = collect_dds(seconds=dds_seconds, domain_id=dds_domain)
    systemd = collect_systemd()
    status_files = collect_status_files()
    native_binaries = collect_native_binaries()
    driver_readiness = collect_driver_readiness(
        systemd=systemd,
        native_binaries=native_binaries,
        status_files=status_files,
    )
    report = {
        "schema": "lingtu.thunder.service_readiness.v1",
        "stamp_s": time.time(),
        "systemd": systemd,
        "status_files": status_files,
        "native_binaries": native_binaries,
        "gnss": collect_gnss_device(),
        "dds": dds,
        "gateway": collect_gateway(gateway_url),
        "processes": processes,
        "lidar_imu": collect_lidar_imu_ownership(processes=processes, dds=dds),
        "camera": collect_camera_readiness(
            systemd=systemd,
            native_binaries=native_binaries,
            status_files=status_files,
            dds=dds,
            processes=processes,
        ),
        "driver": driver_readiness,
    }
    report["summary"] = summarize_report(report)
    if teleop_avoid_stage is not None:
        current_run = collect_current_run()
        report["current_run"] = current_run
        report["teleop_avoid_preflight"] = evaluate_teleop_avoid_preflight(
            {
                "current_run": current_run,
                "status_files": status_files,
                "driver_readiness": driver_readiness,
                "expected_brainstem_host": os.environ.get(
                    "LINGTU_BRAINSTEM_EXPECTED_HOST", ""
                )
                or os.environ.get("LINGTU_BRAINSTEM_HOST", ""),
            },
            stage=teleop_avoid_stage,
            status_max_age_s=status_max_age_s,
        )
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gateway-url", default="http://127.0.0.1:5050")
    parser.add_argument("--dds-seconds", type=float, default=0.0)
    parser.add_argument("--dds-domain", type=int, default=0)
    parser.add_argument(
        "--teleop-avoid-stage",
        choices=sorted(TELEOP_AVOID_STAGES),
        help="evaluate the selected read-only teleop_avoid gate",
    )
    parser.add_argument(
        "--status-max-age-s",
        type=float,
        default=DRIVER_STATUS_MAX_AGE_S,
        help="maximum age for native status snapshots used by the selected gate",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="return non-zero when the selected gate (or general summary) is blocked",
    )
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--pretty", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = build_report(
        gateway_url=args.gateway_url,
        dds_seconds=args.dds_seconds,
        dds_domain=args.dds_domain,
        teleop_avoid_stage=args.teleop_avoid_stage,
        status_max_age_s=args.status_max_age_s,
    )
    text = json.dumps(report, indent=2 if args.pretty else None, sort_keys=True)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    print(text)
    selected_gate = (
        report.get("teleop_avoid_preflight")
        if args.teleop_avoid_stage is not None
        else report.get("summary")
    )
    return 1 if args.strict and not bool((selected_gate or {}).get("ok")) else 0


if __name__ == "__main__":
    raise SystemExit(main())

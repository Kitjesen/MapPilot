#!/usr/bin/env python3
"""Collect read-only real service readiness evidence.

This script is safe to run on the robot or through SSH. It does not start,
stop, restart, enable, or publish anything; it only reads systemd state, status
files, Gateway health endpoints, and process listings.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from diagnostics.field.dds_readiness import probe as probe_dds
from lingtu.product_lock import resolve_current_run_path
from lingtu.run_plan import RunPlan
from runtime.config import load_config
from runtime.service_catalogs.real import (
    camera_service_spec,
    runtime_dds_topics,
    runtime_native_binaries,
    runtime_services,
    runtime_status_files,
    service_install_unit,
    service_spec,
)

SERVICE_UNITS = {name: service_install_unit(name) for name in runtime_services()}
SERVICES = tuple(SERVICE_UNITS.values())
STATUS_FILES = runtime_status_files()
DDS_TOPICS = runtime_dds_topics()

LIDAR_IMU_EXPECTED_PROCESS = "livox_sdk2_stream"
LIDAR_IMU_EXPECTED_BINARY = "livox_dds"
LIDAR_IMU_EXPECTED_SERVICE = "lidar"
LIDAR_IMU_RAW_TOPIC = "rt/lidar/raw_frame"
LIDAR_IMU_IMU_TOPIC = "rt/imu/raw"
CAMERA_SERVICE_UNIT = service_install_unit("camera")
CAMERA_STATUS_NAME = "camera"
CAMERA_DDS_TOPICS = ("rt/camera/info",)
CAMERA_STATUS_FRAME_KEYS = ("color_frames", "depth_frames", "info_frames")
CAMERA_SHM_SEQUENCE_KEYS = (
    "color_shm_sequence",
    "depth_shm_sequence",
    "info_shm_sequence",
)
DRIVER_SERVICE_UNIT = service_install_unit("driver")
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


def collect_systemd(*, services: tuple[str, ...] = SERVICES) -> dict[str, Any]:
    units: dict[str, Any] = {}
    for unit in services:
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


def _collect_status_file(raw_path: str) -> dict[str, Any]:
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
    return entry


def collect_status_files() -> dict[str, Any]:
    return {name: _collect_status_file(raw_path) for name, raw_path in STATUS_FILES.items()}



def _current_run_path() -> Path:
    return resolve_current_run_path(environment=os.environ)


def collect_current_run() -> dict[str, Any]:
    """Read the current RunPlan record."""

    current_path = _current_run_path()
    result: dict[str, Any] = {
        "path": str(current_path),
        "exists": current_path.is_file(),
        "state": {},
        "run_plan_path": "",
        "plan_exists": False,
        "plan_verified": False,
        "run_plan": {},
        "verified_contract": {},
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
            "run_plan": run_plan.as_dict(),
            "verified_contract": {
                "product": run_plan.product,
                "env": run_plan.env,
                "required_topics": list(run_plan.required_topics),
                "required_capabilities": list(run_plan.required_capabilities),
                "native_nav": run_plan.native_nav,
                "native_process_environment": run_plan.native_process_environment,
                "systemd_targets": {
                    role: process.target
                    for process in run_plan.available_processes
                    if process.manager == "systemd"
                    for role in (process.name, *process.provides)
                },
                "selected_roles": sorted({role for process in run_plan.processes for role in process.provides}),
            },
        }
    )
    return result


def collect_native_binaries(*, environment: Mapping[str, str] | None = None) -> dict[str, Any]:
    environment = os.environ if environment is None else environment
    binaries: dict[str, Any] = {}
    blockers: list[str] = []
    camera_driver = environment.get("LINGTU_CAMERA_DRIVER", "orbbec_native")
    for name, binary in runtime_native_binaries(camera_driver=camera_driver).items():
        env_var = binary["env"]
        raw_path = environment.get(env_var, binary["path"])
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
        request = Request(url, headers={"Accept": "application/json"})
        api_key = os.environ.get("LINGTU_API_KEY", "").strip()
        if api_key:
            request.add_header("X-API-Key", api_key)
        with urlopen(request, timeout=timeout) as response:
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
        "messages",
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
    camera_driver: str = "orbbec_native",
    service_unit: str = CAMERA_SERVICE_UNIT,
) -> dict[str, Any]:
    """Collect camera-specific end-to-end readiness evidence."""

    unit = dict(systemd.get(service_unit, {}))
    binaries = dict(native_binaries.get("binaries", {}))
    status = dict(status_files.get(CAMERA_STATUS_NAME, {}))
    status_json = status.get("json", {}) if isinstance(status.get("json"), dict) else {}
    dds_camera = dict(dds.get("services", {}).get(CAMERA_STATUS_NAME, {}))
    dds_samples = dict(dds_camera.get("samples", {}))
    legacy_lines = list(processes.get("legacy_lines", []))
    camera_spec = camera_service_spec(service_spec("camera"), camera_driver)
    camera_binaries = tuple(name for name, _env, _path in camera_spec.binaries)

    blockers: list[str] = []
    if unit.get("missing"):
        blockers.append(f"systemd_unit_missing:{service_unit}")
    elif unit.get("active_state") and unit.get("active_state") != "active":
        blockers.append(f"systemd_unit_inactive:{service_unit}:{unit.get('active_state')}")

    for name in camera_binaries:
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

    if any("orbbec_camera" in line.lower() for line in legacy_lines):
        blockers.append("legacy_process_observed:orbbec_camera")

    return {
        "ok": not blockers,
        "driver": camera_driver,
        "unit": service_unit,
        "status_file": status.get("path", STATUS_FILES.get(CAMERA_STATUS_NAME, "")),
        "native_binaries": {
            name: {
                "path": dict(binaries.get(name, {})).get("path", ""),
                "executable": bool(dict(binaries.get(name, {})).get("executable", False)),
            }
            for name in camera_binaries
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
    service_unit: str = DRIVER_SERVICE_UNIT,
) -> dict[str, Any]:
    """Validate the idle-safe driver boundary without requiring cmd_vel traffic."""

    unit = dict(systemd.get(service_unit, {}))
    binary = dict(native_binaries.get("binaries", {}).get(DRIVER_NATIVE_BINARY, {}))
    status = dict(status_files.get(DRIVER_STATUS_NAME, {}))
    payload = status.get("json", {}) if isinstance(status.get("json"), dict) else {}
    backend = str(payload.get("backend") or "").strip().lower()
    adapter = payload.get("adapter", {}) if isinstance(payload.get("adapter"), dict) else {}
    control = payload.get("control", {}) if isinstance(payload.get("control"), dict) else {}
    blockers: list[str] = []

    if unit.get("missing"):
        blockers.append(f"systemd_unit_missing:{service_unit}")
    elif unit.get("active_state") != "active":
        blockers.append(f"systemd_unit_inactive:{service_unit}:{unit.get('active_state', 'unknown')}")
    if not binary.get("executable", False):
        blockers.append(f"native_binary_missing_or_not_executable:{DRIVER_NATIVE_BINARY}")
    if not status.get("exists", False):
        blockers.append(f"status_file_missing:{status.get('path', STATUS_FILES.get(DRIVER_STATUS_NAME, ''))}")
    else:
        if payload.get("schema_version") != "lingtu.driver.status.v2":
            blockers.append("status_schema_invalid:driver")
        age_s = status.get("age_s")
        if not isinstance(age_s, (int, float)) or age_s > DRIVER_STATUS_MAX_AGE_S:
            blockers.append("driver_status_stale")

    return {
        "ok": not blockers,
        "unit": service_unit,
        "status_file": status.get("path", STATUS_FILES.get(DRIVER_STATUS_NAME, "")),
        "status_age_s": status.get("age_s"),
        "motion": {
            "connected": payload.get("connected"),
            "ready": payload.get("ready"),
            "control": control,
        },
        "backend": backend,
        "adapter": adapter,
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
            "ok": None,
            "checked": False,
            "enabled": False,
            "seconds": seconds,
            "domain_id": domain_id,
            "services": services,
            "blockers": [],
        }

    topic_index: dict[str, list[tuple[str, str]]] = {}
    probe_topics: list[str] = []
    for service, contract in DDS_TOPICS.items():
        ros_topics = list(contract["topics"])
        dds_topics = list(contract["dds_topics"])
        for index, ros_topic in enumerate(ros_topics):
            dds_topic = dds_topics[index] if index < len(dds_topics) else ros_topic
            if dds_topic not in topic_index:
                probe_topics.append(dds_topic)
            topic_index.setdefault(dds_topic, []).append((service, ros_topic))

    blockers: list[str] = []
    try:
        stats = probe_dds(tuple(probe_topics), seconds=max(0.1, seconds), domain_id=domain_id)
        rows = [
            {
                "topic": topic,
                "samples": item.samples,
                "hz": item.hz(),
                "frame_id": getattr(item, "frame_id", ""),
                "points": getattr(item, "points", None),
            }
            for topic, item in stats.items()
        ]
    except Exception as exc:
        blockers.append(f"dds_check_error:{type(exc).__name__}")
        rows = []

    observed = {str(row.get("topic") or ""): row for row in rows}
    for dds_topic in probe_topics:
        item = observed.get(dds_topic, {})
        samples = int(item.get("samples", 0) or 0)
        for service, _ros_topic in topic_index[dds_topic]:
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
        item["ok"] = all(count > 0 or topic == DRIVER_DDS_TOPIC for topic, count in item["samples"].items())
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
) -> dict[str, Any]:
    current_run = collect_current_run()
    environment = dict(os.environ)
    systemd_targets = {}
    if current_run.get("plan_verified"):
        environment.update(current_run["verified_contract"]["native_process_environment"])
        systemd_targets = current_run["verified_contract"]["systemd_targets"]
    service_units = {
        name: systemd_targets.get("host" if name == "lingtu" else name, unit)
        for name, unit in SERVICE_UNITS.items()
    }
    processes = collect_processes()
    dds = collect_dds(seconds=dds_seconds, domain_id=dds_domain)
    systemd = collect_systemd(services=tuple(service_units.values()))
    native_binaries = collect_native_binaries(environment=environment)
    gnss = collect_gnss_device()
    gateway = collect_gateway(gateway_url)
    status_files = collect_status_files()
    driver_readiness = collect_driver_readiness(
        systemd=systemd,
        native_binaries=native_binaries,
        status_files=status_files,
        service_unit=service_units["driver"],
    )
    report = {
        "schema": "lingtu.thunder.service_readiness.v1",
        "stamp_s": time.time(),
        "systemd": systemd,
        "status_files": status_files,
        "native_binaries": native_binaries,
        "gnss": gnss,
        "dds": dds,
        "gateway": gateway,
        "processes": processes,
        "lidar_imu": collect_lidar_imu_ownership(processes=processes, dds=dds),
        "camera": collect_camera_readiness(
            systemd=systemd,
            native_binaries=native_binaries,
            status_files=status_files,
            dds=dds,
            processes=processes,
            camera_driver=environment.get("LINGTU_CAMERA_DRIVER", "orbbec_native"),
            service_unit=service_units["camera"],
        ),
        "driver": driver_readiness,
    }
    report["summary"] = summarize_report(report)
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gateway-url", default="http://127.0.0.1:5050")
    parser.add_argument("--dds-seconds", type=float, default=0.0)
    parser.add_argument("--dds-domain", type=int, default=0)
    parser.add_argument(
        "--strict",
        action="store_true",
        help="return non-zero when diagnostics report a failure",
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
    )
    text = json.dumps(report, indent=2 if args.pretty else None, sort_keys=True)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 1 if args.strict and not report["summary"]["ok"] else 0


if __name__ == "__main__":
    raise SystemExit(main())

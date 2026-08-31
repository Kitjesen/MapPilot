#!/usr/bin/env python3
"""Runtime saved-map relocalization gate for the MuJoCo/Fast-LIO source path.

This gate proves the saved-map stage with live simulated sensors:

raw MuJoCo MID-360 + IMU -> native DDS -> slamd (FastLIO2 localization)
  -> slamd loads the same-source map.pcd
  -> slamctl typed-DDS relocalization succeeds
  -> slamd health, saved-map points, and map->odom remain sane

It is simulation-only and never connects to robot hardware.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import shutil
import signal
import subprocess
import sys
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

DEFAULT_SLAM_CONFIG = (
    SRC / "localization/fastlio2/config/sim_mid360_slam.yaml"
)
DEFAULT_SLAM_RUNTIME_BIN = ROOT / "build/slam_core/slamd"
DEFAULT_SLAM_CONTROL_BIN = ROOT / "build/slam_core/slamctl"
NATIVE_SENSOR_BRIDGE = ROOT / "sim/scripts/mujoco/native_dds_sensors.py"
WINDOWS_SLAM_RUNTIME_BIN = ROOT / "build/slam-core-windows-x64/Release/slamd.exe"
WINDOWS_SLAM_CONTROL_BIN = ROOT / "build/slam-core-windows-x64/Release/slamctl.exe"
DEFAULT_SENSOR_PUBLISHER_BIN = (
    ROOT / "build/mujoco_native_dds/lingtu_mujoco_sensor_publisher"
)
WINDOWS_SENSOR_PUBLISHER_BIN = (
    ROOT
    / "build/windows-native-dds-adapter/Release/lingtu_mujoco_sensor_publisher.exe"
)
KNOWN_SAVED_MAP_METADATA_SCHEMA_PREFIXES = (
    "lingtu.same_source_map_artifacts",
    "lingtu.saved_map_artifacts",
)


def _runtime_dataflow(
    *,
    map_artifact_ok: bool,
    native_inputs_ok: bool,
    sensor_feed_ok: bool | None = None,
    relocalization_ok: bool | None = None,
    tracking_ok: bool | None = None,
    bbs3d_ok: bool | None = None,
) -> list[dict[str, Any]]:
    """Return the stable native saved-map runtime flow consumed by diagnostics."""

    checked_edges = (
        ("saved_map_artifact", map_artifact_ok),
        ("native_runtime_inputs", native_inputs_ok),
        ("native_dds_sensor_feed", sensor_feed_ok),
        ("native_relocalization_response", relocalization_ok),
        ("localization_tracking", tracking_ok),
        ("bbs3d_global_engine", bbs3d_ok),
    )
    return [
        {"id": edge_id, "ok": edge_ok}
        for edge_id, edge_ok in checked_edges
        if edge_ok is not None
    ]


def _bbs3d_succeeded(relocalization: Mapping[str, Any]) -> bool:
    return (
        relocalization.get("success") is True
        and relocalization.get("engine") == "bbs3d_gicp"
    )


def _resolve_latest_map() -> Path | None:
    candidates = [
        path
        for path in ROOT.glob("artifacts/sim_diagnostics/**/same_source_map/map.pcd")
        if path.is_file()
    ]
    return max(candidates, key=lambda path: path.stat().st_mtime, default=None)


def _resolve_map_path(value: str) -> Path | None:
    if value.strip().lower() in {"", "latest"}:
        return _resolve_latest_map()
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = (ROOT / path).resolve()
    return path


def _map_metadata_path(map_pcd: Path) -> Path:
    return map_pcd.parent / "metadata.json"


def _load_map_metadata(map_pcd: Path | None) -> dict[str, Any]:
    if map_pcd is None:
        return {}
    metadata_path = _map_metadata_path(map_pcd)
    if not metadata_path.is_file():
        return {}
    try:
        payload = json.loads(metadata_path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def _metadata_path_matches_map(raw_path: str, map_pcd: Path) -> bool:
    if not raw_path:
        return False
    candidate = Path(raw_path).expanduser()
    if not candidate.is_absolute():
        candidate = ROOT / candidate
    try:
        return candidate.resolve() == map_pcd.resolve()
    except OSError:
        return False


def _map_metadata_contract(map_pcd: Path | None) -> dict[str, Any]:
    blockers: list[str] = []
    if map_pcd is None:
        return {
            "ok": False,
            "path": "",
            "blockers": ["map metadata cannot be validated without map_pcd"],
            "checks": {"map_pcd_present": False},
        }

    metadata_path = _map_metadata_path(map_pcd)
    checks: dict[str, bool] = {
        "map_pcd_present": True,
        "map_pcd_format_ok": map_pcd.is_file() and map_pcd.stat().st_size > 0,
        "metadata_file_exists": metadata_path.is_file(),
    }
    payload: dict[str, Any] = {}
    if metadata_path.is_file():
        try:
            loaded = json.loads(metadata_path.read_text(encoding="utf-8"))
        except Exception:
            loaded = None
        if isinstance(loaded, dict):
            payload = loaded
            checks["metadata_json_object"] = True
        else:
            checks["metadata_json_object"] = False
    else:
        checks["metadata_json_object"] = False

    schema_version = str(payload.get("schema_version") or "")
    pcd_path = str(payload.get("pcd") or "")
    try:
        point_count = int(payload.get("point_count") or 0)
    except (TypeError, ValueError):
        point_count = 0
    checks.update(
        {
            "schema_version_known": schema_version.startswith(
                KNOWN_SAVED_MAP_METADATA_SCHEMA_PREFIXES
            ),
            "world_present": bool(str(payload.get("world") or "").strip()),
            "map_pcd_path_present": bool(pcd_path),
            "map_pcd_path_matches": (
                _metadata_path_matches_map(pcd_path, map_pcd) if pcd_path else False
            ),
            "map_pcd_point_count_positive": point_count > 0,
        }
    )
    scan_time_profile = str(payload.get("scan_time_profile") or "").strip().lower()
    checks["scan_time_profile_valid_or_absent"] = (
        not scan_time_profile
        or scan_time_profile
        in {"instantaneous", "synthetic_rolling", "physical_rolling"}
    )
    for name, ok in checks.items():
        if ok is not True:
            blockers.append(f"map_metadata.{name} is not true")
    return {
        "ok": not blockers,
        "path": str(metadata_path),
        "schema_version": schema_version,
        "world": str(payload.get("world") or ""),
        "scan_time_profile": scan_time_profile,
        "artifacts": {
            "map_pcd": {
                "path": pcd_path,
                "point_count": point_count,
            }
        },
        "checks": checks,
        "blockers": blockers,
    }


def _resolve_live_world_arg(world: str, map_metadata: dict[str, Any]) -> str:
    value = str(world or "map_metadata").strip()
    if value.lower() not in {"", "map_metadata", "same_source"}:
        return value
    metadata_world = str(map_metadata.get("world") or "").strip()
    return metadata_world or "industrial_park"


def _resolve_scan_time_profile_arg(
    scan_time_profile: str,
    map_metadata: dict[str, Any],
) -> str:
    value = str(scan_time_profile or "map_metadata").strip().lower()
    valid = {"instantaneous", "synthetic_rolling", "physical_rolling"}
    if value in valid:
        return value
    if value not in {"", "map_metadata", "same_source"}:
        raise ValueError(f"unsupported scan_time_profile: {scan_time_profile}")
    metadata_value = str(map_metadata.get("scan_time_profile") or "").strip().lower()
    if metadata_value in valid:
        return metadata_value
    return "physical_rolling"


def _resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else (ROOT / path).resolve()


def _binary_candidates(
    value: str, default: Path, *fallbacks: Path
) -> tuple[Path, ...]:
    raw = str(value or "").strip()
    if raw:
        requested = Path(raw).expanduser()
        if requested.is_absolute() or requested.parent != Path("."):
            return (_resolve_path(requested),)
        found = shutil.which(raw)
        return (Path(found),) if found else (_resolve_path(requested),)
    candidates = [default, *fallbacks]
    if os.name == "nt":
        candidates.insert(0, default.with_suffix(".exe"))
    return tuple(candidates)


def _resolve_binary(value: str, default: Path, *fallbacks: Path) -> Path:
    candidates = _binary_candidates(value, default, *fallbacks)
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    return candidates[0]


def _terminate_process(proc: subprocess.Popen[Any] | None, *, timeout_s: float = 6.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except Exception:
        proc.terminate()
    try:
        proc.wait(timeout=timeout_s)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except Exception:
        proc.kill()
    try:
        proc.wait(timeout=timeout_s)
    except Exception:
        pass


def _tail(path: Path, *, lines: int = 80) -> str:
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except Exception:
        return ""
    return "\n".join(text.splitlines()[-lines:])


def _current_host_report() -> dict[str, Any]:
    return {
        "platform_system": platform.system(),
        "platform_machine": platform.machine(),
        "python_version": platform.python_version(),
        "dds_implementation": "cyclonedds",
    }


def _read_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return payload if isinstance(payload, dict) else {}


class _StatusSampler:
    def __init__(self) -> None:
        self.samples = 0
        self.states: list[str] = []
        self.first_odom: tuple[float, float, float] | None = None
        self.last_odom: tuple[float, float, float] | None = None
        self.latest: dict[str, Any] = {}
        self._last_snapshot_written_at_s: float | None = None

    def sample(self, path: Path) -> dict[str, Any]:
        status = _read_json(path)
        if not status:
            return {}
        try:
            snapshot_written_at_s = float(status["snapshot_written_at_s"])
        except (KeyError, TypeError, ValueError):
            snapshot_written_at_s = None
        if (
            snapshot_written_at_s is not None
            and snapshot_written_at_s == self._last_snapshot_written_at_s
        ):
            return status
        self._last_snapshot_written_at_s = snapshot_written_at_s
        self.samples += 1
        self.latest = status
        state = str(status.get("state") or "").strip().upper()
        if state:
            self.states.append(state)
        pose = ((status.get("odometry") or {}).get("pose") or {})
        try:
            xyz = (float(pose["x"]), float(pose["y"]), float(pose["z"]))
        except (KeyError, TypeError, ValueError):
            return status
        if self.first_odom is None:
            self.first_odom = xyz
        self.last_odom = xyz
        return status

    @property
    def odom_delta_m(self) -> float | None:
        if self.first_odom is None or self.last_odom is None:
            return None
        return float(math.dist(self.first_odom, self.last_odom))


def _isolated_map_without_last_pose(map_pcd: Path, run_dir: Path) -> Path:
    map_dir = run_dir / "kidnap_map"
    map_dir.mkdir(parents=True, exist_ok=True)
    isolated = map_dir / "map.pcd"
    if map_pcd.resolve() != isolated.resolve():
        shutil.copy2(map_pcd, isolated)
    last_pose = map_dir / "last_pose.txt"
    if last_pose.exists():
        last_pose.unlink()
    return isolated


def _sensor_feed_command(
    args: argparse.Namespace,
    run_dir: Path,
    status_path: Path,
) -> list[str]:
    python = shutil.which("python3") or sys.executable
    cmd = [
        python,
        str(args.sensor_bridge),
        "--world",
        args.world,
        "--duration",
        str(args.duration),
        "--drive-mode",
        "kinematic",
        "--allow-kinematic-fastlio-acceptance",
        "--drive-vx",
        str(args.drive_vx),
        "--drive-vy",
        str(args.drive_vy),
        "--drive-wz",
        str(args.drive_wz),
        "--mid360-samples-per-frame",
        str(args.mid360_samples_per_frame),
        "--scan-time-profile",
        args.scan_time_profile,
        "--imu-acc-mode",
        args.imu_acc_mode,
        "--timestamp-clock",
        "sim_hardware" if args.duration_clock == "sim" else "wall",
        "--domain-id",
        str(args.domain_id),
        "--slam-status-json",
        str(status_path),
        "--require-slam-output",
        "--json-out",
        str(run_dir / "sensor_feed/report.json"),
    ]
    if args.check_global_relocalize:
        cmd.extend(
            [
                "--start",
                f"{args.kidnap_start_x},{args.kidnap_start_y},{args.kidnap_start_z}",
                "--start-anchor",
                "warmup",
            ]
        )
    if args.publisher_bin:
        cmd.extend(["--publisher-bin", str(args.publisher_bin)])
    return cmd


def _start_sensor_feed(
    args: argparse.Namespace,
    run_dir: Path,
    status_path: Path,
) -> subprocess.Popen[str]:
    cmd = _sensor_feed_command(args, run_dir, status_path)
    (run_dir / "sensor_feed").mkdir(parents=True, exist_ok=True)
    (run_dir / "sensor_feed/command.txt").write_text(
        " ".join(shlex_quote(item) for item in cmd) + "\n",
        encoding="utf-8",
    )
    log = (run_dir / "sensor_feed/gate.log").open("w", encoding="utf-8")
    return subprocess.Popen(
        cmd,
        cwd=str(ROOT),
        env=os.environ.copy(),
        stdout=log,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )


def shlex_quote(value: str) -> str:
    import shlex

    return shlex.quote(str(value))


def _slamd_command(
    args: argparse.Namespace,
    map_pcd: Path,
    status_path: Path,
) -> list[str]:
    return [
        str(args.slam_runtime_bin),
        "--backend", "fastlio2",
        "--mode", "localization",
        "--map", str(map_pcd),
        "--config", str(args.slam_config),
        "--domain-id", str(args.domain_id),
        "--status-json", str(status_path),
        "--status-json-hz", "10",
        "--log-status-s", "5",
    ]


def _start_slamd(
    args: argparse.Namespace,
    map_pcd: Path,
    run_dir: Path,
    status_path: Path,
) -> subprocess.Popen[str]:
    cmd = _slamd_command(args, map_pcd, status_path)
    (run_dir / "slamd_command.txt").write_text(
        " ".join(shlex_quote(item) for item in cmd) + "\n",
        encoding="utf-8",
    )
    log = (run_dir / "slamd.log").open("w", encoding="utf-8")
    return subprocess.Popen(
        cmd,
        cwd=str(ROOT),
        env=os.environ.copy(),
        stdout=log,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )


def _sensor_feed_timeout_s(args: argparse.Namespace) -> float:
    if float(args.live_process_timeout_s) > 0.0:
        return float(args.live_process_timeout_s)
    if str(args.duration_clock) == "sim":
        return max(120.0, float(args.duration) * 12.0 + 60.0)
    return max(5.0, float(args.duration) + 30.0)


def _wait_for_process(
    proc: subprocess.Popen[str], timeout_s: float, sampler: _StatusSampler, status_path: Path
) -> bool:
    deadline = time.time() + max(0.0, timeout_s)
    while time.time() < deadline:
        sampler.sample(status_path)
        if proc.poll() is not None:
            return True
        time.sleep(0.1)
    return proc.poll() is not None


def _wait_for_status(
    proc: subprocess.Popen[str], status_path: Path, sampler: _StatusSampler, timeout_s: float
) -> dict[str, Any]:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        status = sampler.sample(status_path)
        if int(status.get("registered_points") or 0) > 0 and status.get("map_loaded") is True:
            return status
        if proc.poll() is not None:
            break
        time.sleep(0.1)
    return sampler.latest


def _slamctl_command(args: argparse.Namespace) -> list[str]:
    command = [str(args.slam_control_bin)]
    if args.check_global_relocalize:
        command.append("global-relocalize")
    else:
        command.extend(
            [
                "relocalize",
                "--x", str(args.initial_x),
                "--y", str(args.initial_y),
                "--z", str(args.initial_z),
                "--yaw", str(args.initial_yaw),
            ]
        )
    command.extend(
        [
            "--domain-id",
            str(args.domain_id),
            "--timeout-s",
            str(args.relocalization_timeout_s),
        ]
    )
    return command


def _call_slamctl(args: argparse.Namespace) -> tuple[bool, dict[str, Any]]:
    command = _slamctl_command(args)
    completed = subprocess.run(
        command,
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        timeout=max(1.0, float(args.relocalization_timeout_s) + 5.0),
        check=False,
    )
    response: dict[str, Any] = {}
    for line in reversed(completed.stdout.splitlines()):
        try:
            parsed = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(parsed, dict):
            response = parsed
            break
    response.update(
        {
            "available": completed.returncode != 4,
            "success": completed.returncode == 0 and response.get("success") is True,
            "returncode": completed.returncode,
            "stdout": completed.stdout.strip(),
            "stderr": completed.stderr.strip(),
        }
    )
    return bool(response["success"]), response


def run_gate(args: argparse.Namespace) -> dict[str, Any]:
    started = time.time()
    run_dir = args.run_dir.resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    map_pcd = _resolve_map_path(args.map_pcd)
    blockers: list[str] = []
    if map_pcd is None:
        blockers.append("same-source map.pcd not found")
    elif not map_pcd.is_file():
        blockers.append(f"map_pcd not found: {map_pcd}")
    map_metadata = _load_map_metadata(map_pcd)
    map_metadata_contract = _map_metadata_contract(map_pcd)
    blockers.extend(map_metadata_contract.get("blockers") or [])
    live_world = _resolve_live_world_arg(args.world, map_metadata)
    scan_time_profile = _resolve_scan_time_profile_arg(
        args.scan_time_profile,
        map_metadata,
    )
    args.slam_config = _resolve_path(args.slam_config)
    args.sensor_bridge = _resolve_path(args.sensor_bridge)
    args.slam_runtime_bin = _resolve_binary(
        args.slam_runtime_bin, DEFAULT_SLAM_RUNTIME_BIN, WINDOWS_SLAM_RUNTIME_BIN
    )
    args.slam_control_bin = _resolve_binary(
        args.slam_control_bin, DEFAULT_SLAM_CONTROL_BIN, WINDOWS_SLAM_CONTROL_BIN
    )
    if not args.slam_config.is_file():
        blockers.append(f"FastLIO native config not found: {args.slam_config}")
    if not args.sensor_bridge.is_file():
        blockers.append(f"native DDS sensor bridge not found: {args.sensor_bridge}")
    if not args.slam_runtime_bin.is_file():
        blockers.append(f"slamd not found: {args.slam_runtime_bin}")
    if not args.slam_control_bin.is_file():
        blockers.append(f"slamctl not found: {args.slam_control_bin}")
    if not 1 <= int(args.domain_id) <= 231:
        blockers.append(f"DDS domain id must be in [1, 231]: {args.domain_id}")
    publisher_bin = _resolve_binary(
        args.publisher_bin,
        DEFAULT_SENSOR_PUBLISHER_BIN,
        WINDOWS_SENSOR_PUBLISHER_BIN,
    )
    if not publisher_bin.is_file():
        blockers.append(f"native DDS publisher not found: {publisher_bin}")
    args.publisher_bin = str(publisher_bin)
    args.world = live_world
    args.scan_time_profile = scan_time_profile

    report_base: dict[str, Any] = {
        "schema_version": "lingtu.saved_map_relocalize_runtime.v2",
        "validation_level": "runtime_relocalization",
        "execution_mode": "runtime_live",
        "runtime_stage": "saved_map_relocalization",
        "map_dependency": "saved_map_required",
        "requires_saved_map": True,
        "requires_live_slam": True,
        "runtime_relocalization_executed": False,
        "runtime_relocalization_validated": False,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "map_pcd": str(map_pcd) if map_pcd else "",
        "map_metadata": str(_map_metadata_path(map_pcd)) if map_pcd else "",
        "map_metadata_contract": map_metadata_contract,
        "map_metadata_world": str(map_metadata.get("world") or ""),
        "run_dir": str(run_dir),
        "global_relocalization_requested": bool(args.check_global_relocalize),
        "world": live_world,
        "scan_time_profile": scan_time_profile,
        "current_host": _current_host_report(),
        "native_runtime": {
            "slamd": str(args.slam_runtime_bin),
            "slamctl": str(args.slam_control_bin),
            "config": str(args.slam_config),
            "sensor_bridge": str(args.sensor_bridge),
            "publisher": str(args.publisher_bin or "auto"),
            "domain_id": int(args.domain_id),
        },
        "runtime_dataflow": _runtime_dataflow(
            map_artifact_ok=map_metadata_contract.get("ok") is True,
            native_inputs_ok=not blockers,
        ),
    }
    if getattr(args, "preflight_only", False):
        return {
            **report_base,
            "execution_mode": "host_preflight_only",
            "validation_only": True,
            "runtime_relocalization_executed": False,
            "runtime_relocalization_validated": False,
            "ok": not blockers,
            "blockers": blockers,
            "native_preflight": {"ok": not blockers},
            "claim_boundary": "preflight_only_no_live_slam_or_relocalization",
        }
    if blockers:
        return {**report_base, "ok": False, "blockers": blockers}

    if args.check_global_relocalize and map_pcd is not None and map_pcd.is_file():
        map_pcd = _isolated_map_without_last_pose(map_pcd, run_dir)

    sampler = _StatusSampler()
    sensor_proc: subprocess.Popen[str] | None = None
    slamd_proc: subprocess.Popen[str] | None = None
    relocalization: dict[str, Any] = {
        "available": False,
        "success": False,
        "message": "not called",
    }
    relocalization_ok = False
    sensor_report: dict[str, Any] = {}
    sensor_report_path = run_dir / "sensor_feed/report.json"
    status_path = run_dir / "slamd.status.json"
    status_path.unlink(missing_ok=True)
    sensor_report_path.unlink(missing_ok=True)

    try:
        slamd_proc = _start_slamd(args, map_pcd, run_dir, status_path)
        sensor_proc = _start_sensor_feed(args, run_dir, status_path)
        ready_status = _wait_for_status(
            slamd_proc, status_path, sampler, float(args.topic_timeout_s)
        )
        if int(ready_status.get("registered_points") or 0) <= 0:
            blockers.append("native FastLIO registered cloud did not become ready")
        if ready_status.get("map_loaded") is not True:
            blockers.append("slamd did not load the saved map")

        relocalization_ok, relocalization = _call_slamctl(args)
        if not relocalization_ok:
            blockers.append(
                "slamctl relocalization failed: "
                f"{relocalization.get('message') or relocalization.get('stderr')}"
            )

        monitor_deadline = time.time() + args.monitor_after_relocalization_s
        while time.time() < monitor_deadline:
            sampler.sample(status_path)
            time.sleep(0.1)

        sensor_done = _wait_for_process(
            sensor_proc, _sensor_feed_timeout_s(args), sampler, status_path
        )
        if not sensor_done:
            blockers.append("native DDS sensor feed process timed out")
            _terminate_process(sensor_proc)
        if sensor_report_path.is_file():
            sensor_report = _read_json(sensor_report_path)
        else:
            blockers.append("native DDS sensor report missing")
    except (OSError, subprocess.SubprocessError, ValueError) as exc:
        blockers.append(f"native localization runtime failed: {type(exc).__name__}: {exc}")
        relocalization = {
            **relocalization,
            "success": False,
            "message": str(exc),
            "error_type": type(exc).__name__,
        }
    finally:
        _terminate_process(slamd_proc)
        _terminate_process(sensor_proc)

    status = sampler.sample(status_path) or sampler.latest
    tracking_states = {"TRACKING"}
    tracking_health_samples = sum(1 for state in sampler.states if state in tracking_states)
    lost_health_samples = sum(1 for state in sampler.states if state == "LOST")
    latest_health_state = str(status.get("state") or "").upper()
    map_to_odom = status.get("map_odom_tf") or {}
    map_to_odom_xy = math.hypot(
        float(map_to_odom.get("tx", 0.0)),
        float(map_to_odom.get("ty", 0.0)),
    ) if map_to_odom else None
    map_to_odom_z_abs = abs(float(map_to_odom.get("tz", 0.0))) if map_to_odom else None

    slamd_tail = _tail(run_dir / "slamd.log")
    bbs3d_ok = _bbs3d_succeeded(relocalization)
    bbs3d_disabled = "unavailable" in str(
        relocalization.get("message") or ""
    ).lower()

    if sensor_report.get("ok") is not True:
        blockers.append("native DDS sensor report is not ok")
    saved_map_points = int(status.get("saved_map_points") or 0)
    if saved_map_points < int(args.min_saved_map_points):
        blockers.append("saved map point count below threshold")
    if tracking_health_samples < int(args.min_tracking_health_samples):
        blockers.append("native localization tracking health samples below threshold")
    if latest_health_state not in tracking_states:
        blockers.append("latest localization health is not TRACKING")
    if not map_to_odom or map_to_odom.get("valid") is not True:
        blockers.append("map->odom TF missing")
    if map_to_odom_xy is not None and map_to_odom_xy > float(args.max_map_odom_xy_m):
        blockers.append(
            f"map->odom XY correction {map_to_odom_xy:.3f}m exceeds "
            f"{float(args.max_map_odom_xy_m):.3f}m"
        )
    if map_to_odom_z_abs is not None and map_to_odom_z_abs > float(args.max_map_odom_z_abs_m):
        blockers.append(
            f"map->odom Z correction {map_to_odom_z_abs:.3f}m exceeds "
            f"{float(args.max_map_odom_z_abs_m):.3f}m"
        )
    if args.check_global_relocalize:
        expected_start = [
            float(args.kidnap_start_x),
            float(args.kidnap_start_y),
            float(args.kidnap_start_z),
        ]
        observed_start = sensor_report.get("start_anchor_xyz")
        try:
            observed_start_xyz = [float(observed_start[index]) for index in range(3)]
        except (IndexError, TypeError, ValueError):
            observed_start_xyz = []
        if not observed_start_xyz or math.dist(expected_start, observed_start_xyz) > 0.05:
            blockers.append("native DDS sensor feed did not apply the kidnapped start position")
        if bbs3d_disabled:
            blockers.append("BBS3D is disabled or did not load its map")
        if not bbs3d_ok:
            blockers.append("BBS3D global relocalize success was not observed")
        min_global_xy = float(args.min_global_map_odom_xy_m)
        if min_global_xy > 0.0 and map_to_odom_xy is not None and map_to_odom_xy < min_global_xy:
            blockers.append(
                f"kidnapped map->odom XY correction {map_to_odom_xy:.3f}m is below "
                f"{min_global_xy:.3f}m"
            )

    ok = not blockers
    tracking_ok = bool(
        tracking_health_samples >= int(args.min_tracking_health_samples)
        and latest_health_state in tracking_states
        and map_to_odom
        and map_to_odom.get("valid") is True
    )
    return {
        **report_base,
        "ok": ok,
        "runtime_relocalization_executed": bool(relocalization.get("available")),
        "runtime_relocalization_validated": ok,
        "global_relocalization_requested": bool(args.check_global_relocalize),
        "global_relocalization_validated": bool(args.check_global_relocalize and ok),
        "blockers": blockers,
        "runtime_dataflow": _runtime_dataflow(
            map_artifact_ok=map_metadata_contract.get("ok") is True,
            native_inputs_ok=True,
            sensor_feed_ok=sensor_report.get("ok") is True,
            relocalization_ok=(
                relocalization.get("success") is True and relocalization_ok
            ),
            tracking_ok=tracking_ok,
            bbs3d_ok=bbs3d_ok if args.check_global_relocalize else None,
        ),
        "relocalization": relocalization,
        "sensor_feed": {
            "ok": sensor_report.get("ok"),
            "report": str(sensor_report_path),
            "process_returncode": sensor_proc.returncode if sensor_proc is not None else None,
            "remaining_gaps": sensor_report.get("remaining_gaps") or [],
            "sensor_counts": sensor_report.get("sensor_counts") or {},
            "slam_counts": sensor_report.get("slam_counts") or {},
            "scan_time_profile": sensor_report.get("scan_time_profile") or "",
            "start_anchor_xyz": sensor_report.get("start_anchor_xyz"),
        },
        "localization": {
            "process_returncode": slamd_proc.returncode if slamd_proc is not None else None,
            "status_json": str(status_path),
            "health_samples": int(sampler.samples),
            "tracking_health_samples": int(tracking_health_samples),
            "lost_health_samples": int(lost_health_samples),
            "health_states_seen": sorted(set(sampler.states)),
            "latest_health_state": latest_health_state,
            "latest_health": str(status.get("reason") or ""),
            "saved_map_cloud_samples": int(saved_map_points > 0),
            "saved_map_cloud_points_latest": saved_map_points,
            "registered_cloud_points_latest": int(status.get("registered_points") or 0),
            "map_to_odom_tf_samples": int(bool(map_to_odom)),
            "map_to_odom_latest": map_to_odom,
            "map_to_odom_xy_m": map_to_odom_xy,
            "map_to_odom_z_abs_m": map_to_odom_z_abs,
            "odom_delta_m": sampler.odom_delta_m,
            "frames": {
                "registered_cloud": status.get("registered_cloud_frame_id") or "",
                "saved_map_cloud": status.get("saved_map_cloud_frame_id") or "",
                "map": map_to_odom.get("frame_id") or "",
                "odom": map_to_odom.get("child_frame_id") or "",
            },
            "relocalization_state": status.get("relocalization_state") or "",
            "relocalization_message": status.get("last_relocalization_message") or "",
            "relocalization_quality": status.get("relocalization_quality"),
            "relocalization_refine_backend": status.get("relocalization_refine_backend") or "",
            "bbs3d_success_observed": bbs3d_ok,
            "bbs3d_disabled_observed": bbs3d_disabled,
            "kidnap_start_xyz": {
                "x": float(args.kidnap_start_x),
                "y": float(args.kidnap_start_y),
                "z": float(args.kidnap_start_z),
            } if args.check_global_relocalize else None,
        },
        "thresholds": {
            "min_saved_map_points": int(args.min_saved_map_points),
            "min_tracking_health_samples": int(args.min_tracking_health_samples),
            "max_map_odom_xy_m": float(args.max_map_odom_xy_m),
            "max_map_odom_z_abs_m": float(args.max_map_odom_z_abs_m),
            "min_global_map_odom_xy_m": (
                float(args.min_global_map_odom_xy_m)
                if args.check_global_relocalize
                else None
            ),
        },
        "logs": {
            "slamd": str(run_dir / "slamd.log"),
            "slamd_tail": slamd_tail,
            "sensor_publisher": str(run_dir / "sensor_feed/gate.log"),
            "sensor_publisher_tail": _tail(run_dir / "sensor_feed/gate.log"),
        },
        "wall_time_s": round(time.time() - started, 3),
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-pcd", default="latest")
    parser.add_argument(
        "--run-dir",
        type=Path,
        default=ROOT / "artifacts/sim_diagnostics/saved_map_relocalize_runtime",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=ROOT / "artifacts/sim_diagnostics/saved_map_relocalize_runtime/report.json",
    )
    parser.add_argument(
        "--world",
        default="map_metadata",
        help=(
            "MuJoCo world for the live feed. The default map_metadata uses "
            "same_source_map/metadata.json next to --map-pcd and falls back "
            "to industrial_park only when metadata is unavailable."
        ),
    )
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument(
        "--duration-clock",
        choices=["wall", "sim"],
        default="sim",
        help="Interpret the live-feed duration as wall-clock or MuJoCo simulation seconds.",
    )
    parser.add_argument("--drive-vx", type=float, default=0.25)
    parser.add_argument("--drive-vy", type=float, default=0.0)
    parser.add_argument("--drive-wz", type=float, default=0.06)
    parser.add_argument("--mid360-samples-per-frame", type=int, default=24000)
    parser.add_argument(
        "--slam-runtime-bin", default="",
        help="Native slamd executable; empty auto-resolves current Linux/Windows builds.",
    )
    parser.add_argument(
        "--slam-control-bin", default="",
        help="Native typed-DDS slamctl executable; empty auto-resolves current builds.",
    )
    parser.add_argument(
        "--slam-config", default=str(DEFAULT_SLAM_CONFIG),
        help="FastLIO2 native runtime YAML.",
    )
    parser.add_argument("--sensor-bridge", default=str(NATIVE_SENSOR_BRIDGE))
    parser.add_argument("--publisher-bin", default="")
    parser.add_argument(
        "--domain-id",
        type=int,
        default=os.environ.get("LINGTU_DDS_DOMAIN_ID", "").strip() or "231",
    )
    parser.add_argument(
        "--scan-time-profile",
        choices=["map_metadata", "instantaneous", "synthetic_rolling", "physical_rolling"],
        default="map_metadata",
        help=(
            "Per-point time model passed to the MuJoCo Fast-LIO live feed. "
            "The default reads same_source_map/metadata.json and falls back "
            "to the strict physical_rolling profile when unavailable."
        ),
    )
    parser.add_argument(
        "--imu-acc-mode",
        choices=["sensor", "gravity_only", "finite_difference"],
        default="sensor",
    )
    parser.add_argument("--topic-timeout-s", type=float, default=25.0)
    parser.add_argument("--relocalization-timeout-s", type=float, default=25.0)
    parser.add_argument("--monitor-after-relocalization-s", type=float, default=18.0)
    parser.add_argument(
        "--check-global-relocalize",
        action="store_true",
        help="Use slamctl global-relocalize instead of seeded relocalize.",
    )
    parser.add_argument("--initial-x", type=float, default=0.0)
    parser.add_argument("--initial-y", type=float, default=0.0)
    parser.add_argument("--initial-z", type=float, default=0.0)
    parser.add_argument("--initial-yaw", type=float, default=0.0)
    parser.add_argument("--kidnap-start-x", type=float, default=3.0)
    parser.add_argument("--kidnap-start-y", type=float, default=2.0)
    parser.add_argument("--kidnap-start-z", type=float, default=0.0)
    parser.add_argument(
        "--min-global-map-odom-xy-m",
        type=float,
        default=1.0,
        help="Optional minimum map->odom XY correction expected in kidnapped global relocalization.",
    )
    parser.add_argument(
        "--live-process-timeout-s",
        type=float,
        default=0.0,
        help="Override live feed process wait timeout. Default accounts for sim-time RTF.",
    )
    parser.add_argument("--min-saved-map-points", type=int, default=1000)
    parser.add_argument("--min-tracking-health-samples", type=int, default=3)
    parser.add_argument("--max-map-odom-xy-m", type=float, default=5.0)
    parser.add_argument("--max-map-odom-z-abs-m", type=float, default=2.0)
    parser.add_argument("--strict", action="store_true")
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help=(
            "Check the saved map, native binaries, FastLIO config, and sensor "
            "publisher path without launching any process."
        ),
    )
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    report = run_gate(args)
    text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    print(text)
    args.json_out.parent.mkdir(parents=True, exist_ok=True)
    args.json_out.write_text(text + "\n", encoding="utf-8")
    return 0 if report.get("ok") or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())

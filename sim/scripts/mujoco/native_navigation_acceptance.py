#!/usr/bin/env python3
"""Run the ROS-free MuJoCo navigation chain with field C++ runtimes.

The Python process owns MuJoCo physics, simulated sensors, process lifecycle,
and acceptance reporting only. Global planning, local planning, path following,
and command safety remain inside ``navd``.
"""

# ruff: noqa: E402 - direct execution establishes the repository import paths.

from __future__ import annotations

import argparse
import json
import math
import os
import secrets
import shutil
import signal
import subprocess
import sys
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path, PurePath
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
RUNTIME_EVIDENCE_SAMPLE_PERIOD_S = 0.20
DEFAULT_PARENT_SENSOR_DIAGNOSTICS_PERIOD_S = 0.5
_ASCII_ALNUM = frozenset(
    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
)
_NATIVE_IDENTITY_CHARACTERS = _ASCII_ALNUM | frozenset("._:-")


def _phase_runtime_timeout_s(
    duration_s: float,
    shutdown_grace_s: float = 120.0,
    *,
    realtime_factor: float = 1.0,
) -> float:
    """Allow native Windows/WSL children to flush and terminate after simulation."""

    factor = max(0.05, float(realtime_factor))
    return max(0.0, float(duration_s)) / factor + max(
        60.0,
        float(shutdown_grace_s),
    )


def _parent_sensor_diagnostics_args(
    artifact: Path,
    runtime_tolerances: dict[str, Any],
) -> list[str]:
    period_s = float(
        runtime_tolerances.get(
            "parent_diagnostics_period_s",
            DEFAULT_PARENT_SENSOR_DIAGNOSTICS_PERIOD_S,
        )
    )
    if not math.isfinite(period_s) or period_s <= 0.0:
        raise ValueError("runtime_tolerances.parent_diagnostics_period_s must be positive and finite")
    return [
        "--parent-diagnostics-json",
        str(Path(artifact).expanduser().resolve()),
        "--parent-diagnostics-period-s",
        f"{period_s:g}",
    ]


def _motion_health_collection_active(motion_complete_marker: Path) -> bool:
    return not motion_complete_marker.is_file()


SENSOR_PUBLISHER_PROBE_TIMEOUT_S = 60.0
WSL_RUNTIME_PROBE_TIMEOUT_S = 5.0
_NON_BLOCKING_SLAM_ACCURACY_GAP_PREFIXES = (
    "native_slam_not_tracking:",
    "native_slam_quality_low:",
    "native_slam_motion_",
    "native_slam_yaw_",
    "native_slam_map_pose_",
    "native_slam_map_yaw_",
)
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from sim.runtime.process_owner import ProcessTreeOwner
from sim.runtime.windows_cpu_isolation import (
    WindowsCpuIsolationConfig,
    WindowsCpuIsolationPlan,
    WindowsCpuTopology,
    WindowsPhysicalCore,
    discover_windows_cpu_topology,
    resolve_windows_cpu_isolation,
)
from sim.scripts.mujoco.native_dds_sensors import (
    DEFAULT_THUNDERV4_ONNX_POLICY,
    _driver_producer_matches_host,
    _linux_binary_command,
    _managed_wsl_command,
    _read_linux_pid,
    _signal_wsl_pid,
    _wait_wsl_pid_exit,
    _wsl_path,
    _wsl_pid_alive,
)
from sim.scripts.mujoco.product_acceptance import classify_evidence

from lingtu.assembly.native_nav import mapd_environment
from lingtu.sim.acceptance import load_manifest as _load_acceptance_manifest
from lingtu.sim.acceptance import validate_runner_plan

DEFAULT_MANIFEST = ROOT / "config" / "runtime_graph" / "acceptance" / "mujoco_native_navigation_acceptance.json"
DEFAULT_THUNDERV4_MJCF = (
    ROOT / "sim" / "robots" / "doso" / "thunder_v4" / "mjcf" / "thunderv4.xml"
)


def _linux_arg(path: Path) -> str:
    """Format a path for a Linux process, including one launched through WSL."""

    return _wsl_path(path) if os.name == "nt" else str(path)


def _compiled_mujoco_site_offset_body(
    model_path: Path,
    *,
    site_name: str = "lidar_site",
    body_name: str = "base_link",
) -> tuple[float, float, float]:
    """Resolve a site's final compiled pose relative to the robot body."""

    import numpy as np

    import mujoco

    resolved = Path(model_path).expanduser().resolve()
    previous_cwd = Path.cwd()
    try:
        # MuJoCo resolves compiler meshdir relative to the process cwd on
        # Windows, so compile from the MJCF directory without parsing site.pos.
        os.chdir(resolved.parent)
        model = mujoco.MjModel.from_xml_path(resolved.name)
    finally:
        os.chdir(previous_cwd)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if site_id < 0 or body_id < 0:
        raise ValueError(
            f"MuJoCo LiDAR extrinsics require site={site_name!r} and body={body_name!r}"
        )
    body_to_world = np.asarray(data.xmat[body_id], dtype=np.float64).reshape(3, 3)
    world_delta = np.asarray(data.site_xpos[site_id], dtype=np.float64) - np.asarray(
        data.xpos[body_id], dtype=np.float64
    )
    offset = body_to_world.T @ world_delta
    if not np.isfinite(offset).all():
        raise ValueError("compiled MuJoCo LiDAR body-frame offset is non-finite")
    return tuple(float(value) for value in offset)


def _sensor_offset_args(offset_body: tuple[float, float, float]) -> list[str]:
    values = [format(float(value), ".12g") for value in offset_body]
    return [
        "--sensor-offset-x-m",
        values[0],
        "--sensor-offset-y-m",
        values[1],
        "--sensor-offset-z-m",
        values[2],
    ]


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _load_manifest(path: Path) -> dict[str, Any]:
    resolved = path.expanduser().resolve()
    try:
        resolved.relative_to(ROOT)
        root = ROOT
    except ValueError:
        root = resolved.parent
    return _load_acceptance_manifest(resolved, root=root)


def _write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=True, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _arm_mujoco_motion(
    *,
    arm_file: Path,
    status_file: Path,
    token: str,
    domain_id: int,
    scenario: str,
    sensor: "ManagedProcess",
    timeout_s: float = 5.0,
) -> dict[str, Any]:
    payload = {
        "schema": "lingtu.mujoco.external_arm.v1",
        "arm": True,
        "token": token,
        "domain_id": domain_id,
        "scenario": scenario,
    }
    arm_file.parent.mkdir(parents=True, exist_ok=True)
    temporary = arm_file.with_name(f".{arm_file.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=True, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )
    temporary.replace(arm_file)

    deadline = time.monotonic() + max(0.1, float(timeout_s))
    last_status: dict[str, Any] = {}
    while time.monotonic() < deadline:
        if sensor.poll() is not None:
            raise RuntimeError("MuJoCo sensor runner exited before motion arm acknowledgement")
        last_status = _load_json(status_file)
        state = str(last_status.get("state") or "")
        if state == "armed":
            return last_status
        if state in {"invalid", "timed_out"}:
            raise RuntimeError(
                str(last_status.get("last_error") or f"external_arm_{state}")
            )
        time.sleep(0.05)
    raise TimeoutError("MuJoCo motion arm acknowledgement timed out")


def _repo_path(value: str) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (ROOT / path).resolve()


def _ordered_binary_candidates(
    values: Sequence[str],
    *,
    platform_name: str | None = None,
) -> list[str]:
    """Order manifest candidates for the current native binary platform."""

    platform = os.name if platform_name is None else platform_name
    normalized = [str(value) for value in values if str(value)]
    if platform != "nt":
        return [
            value
            for value in normalized
            if PurePath(value).suffix.lower() != ".exe"
        ]
    canonical_windows_prefixes = (
        "build/maps-windows/Release/",
        "build/nav-cpp/windows-x64-nav-endpoint/Release/",
        "build/slam-core-windows-x64/stage/bin/",
        "build/windows-native-dds-adapter/Release/",
    )
    exe_values = [
        value
        for value in normalized
        if PurePath(value).suffix.lower() == ".exe"
        and PurePath(value).as_posix().startswith(canonical_windows_prefixes)
    ]
    fallback_values = [
        value
        for value in normalized
        if PurePath(value).suffix.lower() != ".exe"
    ]
    return [*exe_values, *fallback_values]


def _validated_native_clock_platform(
    navigation_binary: Path,
    driver_bridge_binary: Path,
    *,
    platform_name: str | None = None,
) -> str:
    """Require the FinalVelocityCommand producer and bridge to share one boot clock."""

    platform = os.name if platform_name is None else platform_name
    if platform != "nt":
        return "posix"
    navigation_platform = (
        "windows" if navigation_binary.suffix.lower() == ".exe" else "wsl"
    )
    bridge_platform = (
        "windows" if driver_bridge_binary.suffix.lower() == ".exe" else "wsl"
    )
    if navigation_platform != bridge_platform:
        raise ValueError(
            "navigation and MuJoCo driver bridge must use the same native boot clock: "
            f"navigation={navigation_platform}, driver_bridge={bridge_platform}"
        )
    return navigation_platform


def _native_path_arg(
    binary: Path,
    path: Path,
    *,
    platform_name: str | None = None,
) -> str:
    """Format a filesystem argument for the process that will consume it."""

    platform = os.name if platform_name is None else platform_name
    if platform == "nt" and binary.suffix.lower() != ".exe":
        return _wsl_path(path)
    return str(path.expanduser().resolve())


def _local_planner_backend(manifest: dict[str, Any]) -> str:
    backend = str(
        (manifest.get("navigation_runtime") or {}).get("local_planner") or "cmu"
    ).strip().lower()
    if backend not in {"cmu", "scan"}:
        raise ValueError(f"unsupported local planner backend: {backend}")
    return backend


def _path_library_args(
    backend: str,
    navigation_binary: Path,
    paths: dict[str, Path],
) -> tuple[str, ...]:
    if backend == "scan":
        return ()
    if backend != "cmu":
        raise ValueError(f"unsupported local planner backend: {backend}")
    return (
        "--path-library",
        _native_path_arg(navigation_binary, paths["path_library"]),
    )


def _bind_manifest_binaries_to_run_plan(
    manifest: dict[str, Any],
    plan: Any,
) -> dict[str, dict[str, str]]:
    """Replace component discovery with the RunPlan's concrete artifact paths."""

    process_names = {
        "sensor_publisher": "sensor_publisher",
        "slam": "slam_runtime",
        "mapd": "map_runtime",
        "traversability": "traversability_runtime",
        "navigation": "nav_runtime",
        "driver_bridge": "driver_bridge",
    }
    selected = {process.name: process for process in plan.processes}
    specs = manifest.get("binaries") or {}
    bindings: dict[str, dict[str, str]] = {}
    for binary_name, process_name in process_names.items():
        process = selected.get(process_name)
        spec = specs.get(binary_name)
        if process is None or process.command is None or not isinstance(spec, dict):
            continue
        artifact = process.command.artifact
        path = _repo_path(artifact.path)
        if not path.is_file():
            raise ValueError(f"RunPlan process artifact is missing: {process_name}:{path}")
        spec.pop("env", None)
        spec["candidates"] = [artifact.path]
        bindings[binary_name] = {
            "path": str(path),
            "process": process_name,
        }
    return bindings


def _text_tail(value: str | None, limit: int = 4000) -> str:
    return (value or "")[-limit:]


def _resolve_binary(spec: dict[str, Any]) -> Path | None:
    env_name = str(spec.get("env") or "")
    env_value = str(os.environ.get(env_name, "")) if env_name else ""
    if env_value:
        candidate = _repo_path(env_value)
        return candidate if candidate.is_file() else None
    values = _ordered_binary_candidates(
        [str(value) for value in spec.get("candidates") or []]
    )
    for value in values:
        if not value or (value.startswith("/home/") and os.name == "nt"):
            continue
        candidate = _repo_path(value)
        if candidate.is_file():
            return candidate
    return None


def _requires_wsl_runtime(
    manifest: dict[str, Any],
    *,
    platform_name: str | None = None,
) -> bool:
    """Return whether any resolved acceptance worker must run through WSL."""

    platform = os.name if platform_name is None else platform_name
    if platform != "nt":
        return False
    state_provider = str(
        ((manifest.get("slam_runtime") or {}).get("provider") or "fastlio2")
    ).strip().lower()
    require_traversability = bool(
        ((manifest.get("thresholds") or {}).get("require_traversability", True))
    )
    for name, raw_spec in (manifest.get("binaries") or {}).items():
        if name == "slam" and state_provider == "mujoco_navigation_fixture":
            continue
        if name == "traversability" and not require_traversability:
            continue
        spec = raw_spec if isinstance(raw_spec, dict) else {}
        binary = _resolve_binary(spec)
        if binary is not None and binary.suffix.lower() != ".exe":
            return True
    return False


def _native_command(binary: Path, *args: str) -> list[str]:
    return _linux_binary_command(binary, *args)


def _with_native_env(
    command: list[str],
    **values: str,
) -> tuple[list[str], Mapping[str, str]]:
    """Inject process-local identity while preserving Linux/WSL command style."""

    assignments = {name: str(value) for name, value in values.items()}
    if os.name == "nt" and len(command) >= 3 and command[1] == "-e":
        env_args = [f"{name}={value}" for name, value in assignments.items()]
        return [*command[:2], "env", *env_args, *command[2:]], {}
    if os.name != "nt":
        env_args = [f"{name}={value}" for name, value in assignments.items()]
        return ["env", *env_args, *command], {}
    return list(command), assignments


@dataclass(frozen=True)
class NativeDriverRuntimeLaunch:
    host_boot_id: str
    clock_platform: str
    navigation_command: tuple[str, ...]
    navigation_env: Mapping[str, str]
    driver_bridge_args: tuple[str, ...]


def _native_driver_runtime_launch(
    *,
    manifest: dict[str, Any],
    navigation_binary: Path,
    driver_bridge_binary: Path,
    navigation_command: Sequence[str],
    driver_bridge_pid: Path,
    host_boot_id: str | None = None,
    platform_name: str | None = None,
    navigation_environment: Mapping[str, str] | None = None,
) -> NativeDriverRuntimeLaunch:
    """Bind navd and the physical MuJoCo driver bridge to one runtime identity."""

    clock_platform = _validated_native_clock_platform(
        navigation_binary,
        driver_bridge_binary,
        platform_name=platform_name,
    )
    identity = host_boot_id if host_boot_id is not None else secrets.token_hex(16)
    if (
        not isinstance(identity, str)
        or not identity
        or len(identity) > 128
        or not identity.isascii()
        or identity[0] not in _ASCII_ALNUM
        or any(character not in _NATIVE_IDENTITY_CHARACTERS for character in identity)
    ):
        raise ValueError("native driver host boot identity must be one safe ASCII token")

    config = manifest.get("driver_runtime")
    if not isinstance(config, dict):
        raise ValueError("driver_runtime must be an object")
    expected_keys = {
        "max_linear_mps",
        "max_angular_rps",
        "command_timeout_ms",
        "heartbeat_timeout_ms",
        "apply_timeout_ms",
    }
    if set(config) != expected_keys:
        raise ValueError("driver_runtime must define the exact physical bridge contract")

    def positive_finite(name: str) -> float:
        value = config[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"driver_runtime.{name} must be a positive finite number")
        result = float(value)
        if not math.isfinite(result) or result <= 0.0:
            raise ValueError(f"driver_runtime.{name} must be a positive finite number")
        return result

    def positive_timeout(name: str) -> int:
        value = config[name]
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 60_000:
            raise ValueError(f"driver_runtime.{name} must be an integer in [1, 60000]")
        return value

    max_linear_mps = positive_finite("max_linear_mps")
    max_angular_rps = positive_finite("max_angular_rps")
    command_timeout_ms = positive_timeout("command_timeout_ms")
    heartbeat_timeout_ms = positive_timeout("heartbeat_timeout_ms")
    apply_timeout_ms = positive_timeout("apply_timeout_ms")
    environment_values = {
        **dict(navigation_environment or {}),
        "LINGTU_HOST_BOOT_ID": identity,
    }
    command, environment = _with_native_env(
        list(navigation_command),
        **environment_values,
    )
    bridge_args = (
        "--driver-bridge-bin",
        str(driver_bridge_binary),
        "--driver-bridge-pid-file",
        str(driver_bridge_pid),
        "--driver-expected-host-boot-id",
        identity,
        "--driver-max-linear-mps",
        f"{max_linear_mps:g}",
        "--driver-max-angular-rps",
        f"{max_angular_rps:g}",
        "--driver-command-timeout-ms",
        str(command_timeout_ms),
        "--driver-heartbeat-timeout-ms",
        str(heartbeat_timeout_ms),
        "--driver-apply-timeout-ms",
        str(apply_timeout_ms),
    )
    return NativeDriverRuntimeLaunch(
        host_boot_id=identity,
        clock_platform=clock_platform,
        navigation_command=tuple(command),
        navigation_env=dict(environment),
        driver_bridge_args=bridge_args,
    )


def _native_map_identity(
    *,
    paths: dict[str, Path],
    phase: str,
    domain_id: int,
    session_root: Path,
) -> dict[str, str]:
    """Resolve the native identity shared by Mapd and optional terrain."""

    map_dir = paths.get("map_dir")
    metadata = _load_json(paths.get("metadata", Path()))
    map_id = str(
        os.environ.get("LINGTU_MAP_ID")
        or metadata.get("map_name")
        or (map_dir.name if map_dir else "mujoco_map")
    )
    content_epoch = str(
        os.environ.get("LINGTU_MAP_CONTENT_EPOCH")
        or metadata.get("content_epoch")
        or metadata.get("created_at")
        or "1"
    ).strip()
    frame_id = str(
        os.environ.get("LINGTU_MAP_FRAME")
        or metadata.get("frame_id")
        or metadata.get("frame")
        or "map"
    ).strip()
    if (
        not content_epoch
        or content_epoch[0] not in "123456789"
        or not content_epoch.isascii()
        or not content_epoch.isdigit()
    ):
        raise ValueError("LINGTU_MAP_CONTENT_EPOCH must be a positive integer")
    if not frame_id:
        raise ValueError("LINGTU_MAP_FRAME must not be empty")
    session_id = str(os.environ.get("LINGTU_PRODUCT_SESSION_ID") or "").strip()
    if not session_id:
        session_id = f"mujoco-native-{phase}-{int(domain_id)}-{time.time_ns()}"
    return {
        "LINGTU_PRODUCT_SESSION_ID": session_id,
        "LINGTU_SESSION_ROOT": str(session_root.resolve()),
        "LINGTU_MAP_ID": map_id,
        "LINGTU_MAP_CONTENT_EPOCH": content_epoch,
        "LINGTU_MAP_FRAME": frame_id,
    }


def _native_mapd_launch(
    *,
    binary: Path,
    domain_id: int,
    status_file: Path,
    map_root: Path,
    runtime: Mapping[str, Any],
    environment: Mapping[str, str],
) -> tuple[list[str], Mapping[str, str]]:
    collision_cap = int(
        runtime.get("max_collision_snapshot_points")
        or environment.get("LINGTU_MAPD_MAX_COLLISION_SNAPSHOT_POINTS")
        or 50000
    )
    command = _native_command(
        binary,
        "--domain-id",
        str(domain_id),
        "--status-file",
        _native_path_arg(binary, status_file),
        "--map-root",
        _native_path_arg(binary, map_root),
        "--disable-query",
        "--state-hz",
        str(float(runtime.get("state_hz") or 5.0)),
        "--cloud-hz",
        str(float(runtime.get("cloud_hz") or 5.0)),
        "--map-hz",
        str(float(runtime.get("map_hz") or 5.0)),
        "--scene-hz",
        str(float(runtime.get("scene_hz") or 2.0)),
        "--max-points",
        str(int(runtime.get("max_points") or 5000)),
        "--max-collision-snapshot-points",
        str(collision_cap),
        "--min-range",
        str(float(runtime.get("min_range_m") or 0.1)),
        "--max-range",
        str(float(runtime.get("max_range_m") or 12.0)),
        "--decay-ms",
        str(int(runtime.get("decay_ms") or 250)),
        "--stale-ms",
        str(int(runtime.get("stale_ms") or 1000)),
    )
    return _with_native_env(command, **dict(environment))


def _probe_sensor_publisher(binary: Path) -> tuple[bool, str]:
    command = _native_command(binary, "--stdin-records", "--dds", "--domain-id", "0")
    try:
        proc = subprocess.run(
            command,
            cwd=ROOT,
            input=b"",
            capture_output=True,
            text=False,
            timeout=SENSOR_PUBLISHER_PROBE_TIMEOUT_S,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, f"{type(exc).__name__}:{exc}"
    output = "\n".join(
        _decode_native_probe_output(value)
        for value in (proc.stdout, proc.stderr)
        if value
    ).strip()
    if proc.returncode != 0 or "built without DDS support" in output:
        return False, output[-2000:]
    return True, output[-2000:]


def _probe_wsl_runtime() -> tuple[bool, str]:
    """Check the WSL control path before launching native acceptance workers."""

    if os.name != "nt":
        return True, "native_linux"
    launcher = shutil.which("wsl.exe") or shutil.which("wsl")
    if not launcher:
        return False, "wsl_executable_missing"
    try:
        probe = subprocess.run(
            [launcher, "-e", "true"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=False,
            timeout=WSL_RUNTIME_PROBE_TIMEOUT_S,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, f"{type(exc).__name__}:{exc}"
    output = "\n".join(
        _decode_native_probe_output(value)
        for value in (probe.stdout, probe.stderr)
        if value
    ).strip()
    if probe.returncode != 0:
        return False, output[-2000:] or f"wsl_exit_{probe.returncode}"
    return True, output[-2000:] or "ready"


def _decode_native_probe_output(value: bytes | str | None) -> str:
    if value is None:
        return ""
    if isinstance(value, str):
        return value
    if b"\x00" in value:
        mixed_boundary = None
        for index in range(1, max(1, len(value) - 7)):
            suffix = value[index : index + 8]
            if len(suffix) == 8 and b"\x00" not in suffix and all(
                byte in {9, 10, 13} or 32 <= byte < 127 for byte in suffix
            ):
                mixed_boundary = index
                break
        prefix = value if mixed_boundary is None else value[:mixed_boundary]
        suffix = b"" if mixed_boundary is None else value[mixed_boundary:]
        candidate = prefix[:-1] if len(prefix) % 2 else prefix
        try:
            decoded = candidate.decode("utf-16-le").lstrip("\ufeff").rstrip("\x00")
            if suffix:
                decoded += suffix.decode("utf-8", errors="replace")
            return decoded
        except UnicodeDecodeError:
            pass
    return value.decode("utf-8", errors="replace")


def _artifact_storage_probe(out_dir: Path) -> tuple[bool, str]:
    """Classify where high-rate native acceptance artifacts will be written."""

    if os.name != "nt":
        return True, "native_filesystem"
    resolved = Path(out_dir).expanduser().resolve()
    drive = str(resolved.drive).replace("/", "\\").lower()
    if (
        drive.startswith("\\\\?\\unc\\wsl.localhost\\")
        or drive.startswith("\\\\wsl.localhost\\")
        or drive.startswith("\\\\wsl$\\")
    ):
        return True, "wsl_ext4_unc"
    if len(drive) == 2 and drive[1] == ":":
        return False, "windows_9p_mount"
    return False, "windows_non_ext4_path"


def _navigation_affinity_masks(
    plan: WindowsCpuIsolationPlan,
    topology: WindowsCpuTopology | None,
    physical_cores: int,
) -> tuple[int, int]:
    """Give navd whole cores while retaining MuJoCo and one support core."""

    if isinstance(physical_cores, bool) or not isinstance(physical_cores, int) or physical_cores <= 0:
        raise ValueError("windows_navigation_physical_cores must be a positive integer")
    if physical_cores > len(plan.unreal_core_ids):
        raise ValueError(
            "windows_navigation_physical_cores must leave one non-MuJoCo support core"
        )
    if physical_cores == 1:
        return (
            plan.owner_thread_affinity_mask,
            plan.mujoco_affinity_mask | plan.unreal_affinity_mask,
        )
    if topology is None:
        raise ValueError("Windows CPU topology is required for multi-core navigation affinity")
    by_id = {core.core_id: core for core in topology.cores}
    available: list[WindowsPhysicalCore] = [
        by_id[core_id] for core_id in plan.unreal_core_ids if core_id in by_id
    ]
    if len(available) != len(plan.unreal_core_ids):
        raise ValueError("Windows CPU topology changed while resolving acceptance affinity")
    available.sort(key=lambda core: (-core.efficiency_class, core.core_id))
    extra = available[: physical_cores - 1]
    navigation_mask = plan.owner_thread_affinity_mask
    for core in extra:
        navigation_mask |= core.affinity_mask
    support_mask = (plan.mujoco_affinity_mask | plan.unreal_affinity_mask) & ~navigation_mask
    return navigation_mask, support_mask


def _windows_acceptance_affinity_masks(
    manifest: Mapping[str, Any],
) -> tuple[int | None, int | None]:
    """Reserve whole, non-overlapping cores for navd and simulation support."""

    enabled = manifest.get("windows_cpu_isolation", False)
    if not isinstance(enabled, bool):
        raise ValueError("windows_cpu_isolation must be boolean")
    if os.name != "nt" or not enabled:
        return None, None
    plan = resolve_windows_cpu_isolation(WindowsCpuIsolationConfig())
    physical_cores = manifest.get("windows_navigation_physical_cores", 1)
    if physical_cores == 1:
        return _navigation_affinity_masks(plan, None, physical_cores)
    return _navigation_affinity_masks(
        plan,
        discover_windows_cpu_topology(),
        physical_cores,
    )


def _navigation_smoother_environment(
    runtime: Mapping[str, Any],
) -> dict[str, str]:
    enabled = runtime.get("velocity_smoother_enabled", False)
    if not isinstance(enabled, bool):
        raise ValueError("navigation_runtime.velocity_smoother_enabled must be boolean")
    environment = {
        "LINGTU_NAV_SMOOTHER_ENABLED": "true" if enabled else "false",
    }
    raw = runtime.get("velocity_smoother") or {}
    if not isinstance(raw, Mapping):
        raise ValueError("navigation_runtime.velocity_smoother must be an object")
    if not raw:
        return environment

    scale_velocities = raw.get("scale_velocities", False)
    if not isinstance(scale_velocities, bool):
        raise ValueError("navigation_runtime.velocity_smoother.scale_velocities must be boolean")
    environment["LINGTU_NAV_SMOOTHER_SCALE_VELOCITIES"] = (
        "true" if scale_velocities else "false"
    )

    numeric_environment = {
        "linear_min_mps": (
            "LINGTU_NAV_SMOOTHER_X_MIN_MPS",
            "LINGTU_NAV_SMOOTHER_Y_MIN_MPS",
        ),
        "linear_max_mps": (
            "LINGTU_NAV_SMOOTHER_X_MAX_MPS",
            "LINGTU_NAV_SMOOTHER_Y_MAX_MPS",
        ),
        "linear_acceleration_mps2": (
            "LINGTU_NAV_SMOOTHER_X_ACCEL_MPS2",
            "LINGTU_NAV_SMOOTHER_Y_ACCEL_MPS2",
        ),
        "linear_deceleration_mps2": (
            "LINGTU_NAV_SMOOTHER_X_DECEL_MPS2",
            "LINGTU_NAV_SMOOTHER_Y_DECEL_MPS2",
        ),
        "yaw_min_radps": ("LINGTU_NAV_SMOOTHER_YAW_MIN_RADPS",),
        "yaw_max_radps": ("LINGTU_NAV_SMOOTHER_YAW_MAX_RADPS",),
        "yaw_acceleration_radps2": (
            "LINGTU_NAV_SMOOTHER_YAW_ACCEL_RADPS2",
        ),
        "yaw_deceleration_radps2": (
            "LINGTU_NAV_SMOOTHER_YAW_DECEL_RADPS2",
        ),
    }
    unknown = set(raw) - {"scale_velocities", *numeric_environment}
    if unknown:
        raise ValueError(
            "unsupported navigation_runtime.velocity_smoother fields: "
            + ", ".join(sorted(str(name) for name in unknown))
        )
    for name, environment_names in numeric_environment.items():
        if name not in raw:
            continue
        value = raw[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(
                f"navigation_runtime.velocity_smoother.{name} must be finite"
            )
        resolved = float(value)
        if not math.isfinite(resolved):
            raise ValueError(
                f"navigation_runtime.velocity_smoother.{name} must be finite"
            )
        for environment_name in environment_names:
            environment[environment_name] = f"{resolved:g}"
    return environment


def _scan_follower_environment(runtime: Mapping[str, Any]) -> dict[str, str]:
    raw = runtime.get("scan_follower") or {}
    if not isinstance(raw, Mapping):
        raise ValueError("navigation_runtime.scan_follower must be an object")
    fields = {
        "time_forward_s": "LINGTU_NAV_SCAN_TIME_FORWARD_S",
        "heading_error_rad": "LINGTU_NAV_SCAN_HEADING_ERROR_RAD",
        "position_gain": "LINGTU_NAV_SCAN_POSITION_GAIN",
        "yaw_gain": "LINGTU_NAV_SCAN_YAW_GAIN",
        "max_vx_mps": "LINGTU_NAV_SCAN_MAX_VX_MPS",
        "max_vy_mps": "LINGTU_NAV_SCAN_MAX_VY_MPS",
        "max_yaw_rate_rad_s": "LINGTU_NAV_SCAN_MAX_YAW_RATE_RAD_S",
    }
    environment: dict[str, str] = {}
    positive = {"time_forward_s", "max_vx_mps", "max_vy_mps"}
    for key, env_name in fields.items():
        if key not in raw:
            continue
        value = float(raw[key])
        if (
            not math.isfinite(value)
            or value < 0.0
            or (key in positive and value == 0.0)
        ):
            raise ValueError("navigation_runtime.scan_follower values are inconsistent")
        environment[env_name] = f"{value:g}"
    return environment


@dataclass
class ManagedProcess:
    name: str
    command: list[str]
    log_path: Path
    env: Mapping[str, str] | None = None
    affinity_mask: int | None = None
    process: subprocess.Popen[str] | None = None
    _log: Any = None
    linux_pid: int | None = None
    pid_path: Path | None = None
    cleanup: dict[str, Any] = field(default_factory=dict)
    _process_owner: ProcessTreeOwner | None = field(
        default=None, init=False, repr=False
    )

    def start(self) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log = self.log_path.open("w", encoding="utf-8")
        self._process_owner = (
            ProcessTreeOwner(affinity_mask=self.affinity_mask)
            if self.affinity_mask is not None
            else ProcessTreeOwner()
        )
        launch_command = self.command
        if os.name == "nt" and len(self.command) >= 3 and self.command[1] == "-e":
            self.pid_path = self.log_path.with_suffix(".pid")
            launch_command = _managed_wsl_command(self.command, self.pid_path)
        if self._uses_windows_console_group():
            popen_options = dict(
                self._process_owner.popen_options(
                    creationflags=subprocess.CREATE_NEW_PROCESS_GROUP
                )
            )
        else:
            popen_options = dict(self._process_owner.popen_options())
        if self.env:
            inherited = dict(os.environ)
            inherited.update(self.env)
            popen_options["env"] = inherited
        try:
            self.process = subprocess.Popen(
                launch_command,
                cwd=ROOT,
                stdout=self._log,
                stderr=subprocess.STDOUT,
                text=True,
                **popen_options,
            )
            self._process_owner.attach(self.process)
            if self.pid_path is not None:
                self.linux_pid = _read_linux_pid(self.pid_path)
                if self.linux_pid is None:
                    raise RuntimeError(
                        f"WSL process ownership handshake failed: {self.name}"
                    )
        except Exception:
            if self.process is not None:
                self._process_owner.terminate(self.process, timeout_s=2.0)
                self.process = None
            else:
                self._process_owner.close()
            self._process_owner = None
            self._log.close()
            self._log = None
            raise

    def poll(self) -> int | None:
        return self.process.poll() if self.process is not None else None

    def wait(self, timeout_s: float) -> int:
        if self.process is None:
            raise RuntimeError(f"process was not started: {self.name}")
        return int(self.process.wait(timeout=max(0.1, timeout_s)))

    def _uses_windows_console_group(self) -> bool:
        return (
            os.name == "nt"
            and self.pid_path is None
            and any(
                argument in {"operator-motion", "teleop-stream"}
                for argument in self.command[1:3]
            )
        )

    def _owned_process_alive(self) -> bool:
        if self.linux_pid is not None:
            return _wsl_pid_alive(self.linux_pid)
        return self.process is not None and self.process.poll() is None

    def request_graceful_stop(self) -> bool:
        """Ask the owned runtime to clean up its authority before escalation."""

        if not self._owned_process_alive():
            return False
        try:
            if self.linux_pid is not None:
                return bool(_signal_wsl_pid(self.linux_pid, "TERM"))
            if self.process is None:
                return False
            if self._uses_windows_console_group():
                self.process.send_signal(signal.CTRL_BREAK_EVENT)
            else:
                self.process.terminate()
            return True
        except (OSError, ValueError):
            return False

    def stop(self, *, graceful_timeout_s: float = 3.0) -> None:
        alive_before = self._owned_process_alive()
        graceful_signal_sent = self.request_graceful_stop()
        graceful_exit = not alive_before
        if graceful_signal_sent and self.linux_pid is not None:
            graceful_exit = _wait_wsl_pid_exit(
                self.linux_pid, max(0.0, float(graceful_timeout_s))
            )
        elif graceful_signal_sent and self.process is not None:
            try:
                self.process.wait(timeout=max(0.1, float(graceful_timeout_s)))
                graceful_exit = True
            except subprocess.TimeoutExpired:
                graceful_exit = False

        hard_cleanup_used = self.process is not None and self.process.poll() is None
        if self.process is not None and self._process_owner is not None:
            self._process_owner.terminate(self.process, timeout_s=2.0)
        elif self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1.0)
        if self._process_owner is not None:
            self._process_owner.close_after_exit()
            self._process_owner = None
        if self._log is not None:
            self._log.close()
            self._log = None
        alive_after = self._owned_process_alive()
        self.cleanup = {
            "name": self.name,
            "linux_pid": self.linux_pid,
            "pid_file": str(self.pid_path or ""),
            "alive_before_cleanup": alive_before,
            "alive_after_cleanup": alive_after,
            "graceful_signal_sent": graceful_signal_sent,
            "graceful_exit": graceful_exit,
            "hard_cleanup_used": hard_cleanup_used,
            "clean": not alive_after,
            "relay_returncode": (
                self.process.poll() if self.process is not None else None
            ),
        }

    def tail(self, limit: int = 5000) -> str:
        try:
            text = self.log_path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return ""
        return text[-limit:]


def _cleanup_pid_file(name: str, path: Path) -> dict[str, Any]:
    pid = _read_linux_pid(path, timeout_s=0.0) if path.is_file() else None
    alive_before = _wsl_pid_alive(pid)
    if alive_before:
        _signal_wsl_pid(pid, "TERM")
        if not _wait_wsl_pid_exit(pid, 2.0):
            _signal_wsl_pid(pid, "KILL")
            _wait_wsl_pid_exit(pid, 1.0)
    alive_after = _wsl_pid_alive(pid)
    return {
        "name": name,
        "linux_pid": pid,
        "pid_file": str(path),
        "alive_before_cleanup": alive_before,
        "alive_after_cleanup": alive_after,
        "clean": not alive_after,
    }


@dataclass
class NativeEvidence:
    samples: int = 0
    motion_health_samples: int = 0
    plan_accepted: bool = False
    local_path_found: bool = False
    max_global_path_points: int = 0
    max_local_path_points: int = 0
    max_cmd_vel_published: int = 0
    max_traversability_published: int = 0
    max_registered_clouds: int = 0
    max_path_follower_cmd_norm: float = 0.0
    max_computed_cmd_norm: float = 0.0
    local_path_role: str = ""
    path_follower_role: str = ""
    command_transport: str = ""
    max_command_requests: int = 0
    max_command_acks: int = 0
    command_last_accepted: bool = False
    goal_reached_observed: bool = False
    input_gate_ready_samples: int = 0
    max_odom_tf_rejections: int = 0
    max_cloud_pose_rejections: int = 0
    max_consecutive_input_stale_s: float = 0.0
    max_navigation_loop_overrun_ms: float = 0.0
    navigation_loop_overrun_samples_ms: list[float] = field(default_factory=list)
    max_navigation_loop_overrun_context: dict[str, Any] = field(default_factory=dict)
    near_field_stop_samples: int = 0
    pre_safety_command_samples: int = 0
    max_final_safety_cost: float = -1.0
    min_map_odom_tf_age_s: float = math.inf
    local_reason_counts: dict[str, int] = field(default_factory=dict)
    last_nav: dict[str, Any] = field(default_factory=dict)
    last_slam: dict[str, Any] = field(default_factory=dict)
    last_traversability: dict[str, Any] = field(default_factory=dict)
    _input_gate_seen_ready: bool = field(default=False, repr=False)
    _current_input_stale_samples: int = field(default=0, repr=False)
    _motion_health_frozen: bool = field(default=False, repr=False)
    _last_loop_overrun_snapshot_key: tuple[str, Any] | None = field(default=None, repr=False)

    def sample(
        self,
        *,
        nav_path: Path,
        slam_path: Path,
        traversability_path: Path,
        collect_motion_health: bool = True,
    ) -> None:
        nav = _load_json(nav_path)
        slam = _load_json(slam_path)
        traversability = _load_json(traversability_path)
        if nav:
            self.samples += 1
            self.last_nav = nav
            self.plan_accepted = self.plan_accepted or bool((nav.get("last_plan") or {}).get("accepted"))
            self.local_path_found = self.local_path_found or bool((nav.get("last_local") or {}).get("path_found"))
            self.max_global_path_points = max(
                self.max_global_path_points,
                int(nav.get("global_path_points") or 0),
            )
            self.max_local_path_points = max(
                self.max_local_path_points,
                int(nav.get("local_path_points") or 0),
                int((nav.get("last_local") or {}).get("local_path_points") or 0),
            )
            counters = nav.get("counters") or {}
            self.max_cmd_vel_published = max(
                self.max_cmd_vel_published,
                int(counters.get("cmd_vel_published") or 0),
            )
            cmd = (nav.get("last_local") or {}).get("cmd_vel") or {}
            path_follower_cmd = (nav.get("last_local") or {}).get("path_follower_cmd_vel") or {}
            path_follower_cmd_norm = math.hypot(
                float(path_follower_cmd.get("vx") or 0.0),
                float(path_follower_cmd.get("vy") or 0.0),
            ) + abs(float(path_follower_cmd.get("wz") or 0.0))
            self.max_path_follower_cmd_norm = max(
                self.max_path_follower_cmd_norm,
                path_follower_cmd_norm,
            )
            if path_follower_cmd_norm > 1e-4:
                self.pre_safety_command_samples += 1
            self.max_computed_cmd_norm = max(
                self.max_computed_cmd_norm,
                math.hypot(float(cmd.get("vx") or 0.0), float(cmd.get("vy") or 0.0)) + abs(float(cmd.get("wz") or 0.0)),
            )
            self.local_path_role = str(nav.get("local_path_role") or self.local_path_role)
            self.path_follower_role = str(nav.get("path_follower_role") or self.path_follower_role)
            command_boundary = nav.get("command_boundary") or {}
            self.command_transport = str(command_boundary.get("transport") or self.command_transport)
            self.max_command_requests = max(
                self.max_command_requests,
                int(command_boundary.get("received") or 0),
            )
            self.max_command_acks = max(
                self.max_command_acks,
                int(command_boundary.get("ack_sent") or 0),
            )
            self.command_last_accepted = self.command_last_accepted or bool(command_boundary.get("last_accepted"))
            goal_reached_now = bool((nav.get("last_local") or {}).get("goal_reached"))
            input_gate = nav.get("input_gate") or {}
            track_motion_health = (
                collect_motion_health
                and self.command_last_accepted
                and not self._motion_health_frozen
                and "ready" in input_gate
            )
            self.goal_reached_observed = self.goal_reached_observed or goal_reached_now
            if track_motion_health:
                self.motion_health_samples += 1
                if input_gate.get("ready") is True:
                    self.input_gate_ready_samples += 1
                    self._input_gate_seen_ready = True
                    self._current_input_stale_samples = 0
                elif self._input_gate_seen_ready:
                    self._current_input_stale_samples += 1
                    self.max_consecutive_input_stale_s = max(
                        self.max_consecutive_input_stale_s,
                        self._current_input_stale_samples * RUNTIME_EVIDENCE_SAMPLE_PERIOD_S,
                    )
                frame_gate = nav.get("frame_gate") or {}
                cloud_sync = nav.get("cloud_sync") or {}
                timing_ms = nav.get("timing_ms") or {}
                try:
                    self.max_odom_tf_rejections = max(
                        self.max_odom_tf_rejections,
                        int(frame_gate.get("odom_rejected") or 0),
                    )
                except (TypeError, ValueError):
                    pass
                try:
                    self.max_cloud_pose_rejections = max(
                        self.max_cloud_pose_rejections,
                        int(cloud_sync.get("pose_rejected") or 0),
                    )
                except (TypeError, ValueError):
                    pass
                try:
                    loop_overrun_ms = float(timing_ms.get("overrun") or 0.0)
                    if math.isfinite(loop_overrun_ms):
                        if nav.get("stamp_s") is not None:
                            snapshot_key = ("stamp_s", nav.get("stamp_s"))
                        elif nav.get("status_sequence") is not None:
                            snapshot_key = ("status_sequence", nav.get("status_sequence"))
                        else:
                            snapshot_key = ("evidence_sample", self.samples)
                        if snapshot_key != self._last_loop_overrun_snapshot_key:
                            self._last_loop_overrun_snapshot_key = snapshot_key
                            is_new_peak = (
                                not self.navigation_loop_overrun_samples_ms
                                or loop_overrun_ms > self.max_navigation_loop_overrun_ms
                            )
                            self.navigation_loop_overrun_samples_ms.append(loop_overrun_ms)
                            self.max_navigation_loop_overrun_ms = max(
                                self.max_navigation_loop_overrun_ms,
                                loop_overrun_ms,
                            )
                            if is_new_peak:
                                local = nav.get("last_local") or {}
                                self.max_navigation_loop_overrun_context = {
                                    "evidence_sample": self.samples,
                                    "status_stamp_s": nav.get("stamp_s"),
                                    "status_sequence": nav.get("status_sequence"),
                                    "overrun_ms": loop_overrun_ms,
                                    "timing_ms": dict(timing_ms),
                                    "input_gate_ready": input_gate.get("ready"),
                                    "input_gate_reason": str(input_gate.get("reason") or ""),
                                    "local_reason": str(local.get("reason") or ""),
                                    "goal_reached": goal_reached_now,
                                }
                except (TypeError, ValueError):
                    pass
            if goal_reached_now:
                self._motion_health_frozen = True
            local = nav.get("last_local") or {}
            if bool(local.get("near_field_stop")):
                self.near_field_stop_samples += 1
            local_reason = str(local.get("reason") or "")
            if local_reason:
                self.local_reason_counts[local_reason] = self.local_reason_counts.get(local_reason, 0) + 1
            final_safety = local.get("final_safety") or {}
            try:
                self.max_final_safety_cost = max(
                    self.max_final_safety_cost,
                    float(final_safety.get("traversability_cost") or -1.0),
                )
            except (TypeError, ValueError):
                pass
        if slam:
            self.last_slam = slam
        if traversability:
            self.last_traversability = traversability
            frame_contract = traversability.get("frame_contract") or {}
            try:
                tf_age_s = float(frame_contract.get("map_odom_tf_age_s"))
                if math.isfinite(tf_age_s):
                    self.min_map_odom_tf_age_s = min(
                        self.min_map_odom_tf_age_s,
                        abs(tf_age_s),
                    )
            except (TypeError, ValueError):
                pass
            counters = traversability.get("counters") or {}
            self.max_traversability_published = max(
                self.max_traversability_published,
                int(counters.get("published") or 0),
            )
            self.max_registered_clouds = max(
                self.max_registered_clouds,
                int(counters.get("registered_clouds") or 0),
            )

    def navigation_loop_overrun_percentile_ms(self, percentile: float) -> float:
        if not self.navigation_loop_overrun_samples_ms:
            return 0.0
        bounded_percentile = min(100.0, max(0.0, float(percentile)))
        ordered = sorted(self.navigation_loop_overrun_samples_ms)
        rank = max(1, math.ceil((bounded_percentile / 100.0) * len(ordered)))
        return float(ordered[rank - 1])

    def to_dict(self) -> dict[str, Any]:
        return {
            "samples": self.samples,
            "motion_health_samples": self.motion_health_samples,
            "plan_accepted": self.plan_accepted,
            "local_path_found": self.local_path_found,
            "max_global_path_points": self.max_global_path_points,
            "max_local_path_points": self.max_local_path_points,
            "max_cmd_vel_published": self.max_cmd_vel_published,
            "max_traversability_published": self.max_traversability_published,
            "max_registered_clouds": self.max_registered_clouds,
            "max_path_follower_cmd_norm": self.max_path_follower_cmd_norm,
            "max_computed_cmd_norm": self.max_computed_cmd_norm,
            "local_path_role": self.local_path_role,
            "path_follower_role": self.path_follower_role,
            "command_transport": self.command_transport,
            "max_command_requests": self.max_command_requests,
            "max_command_acks": self.max_command_acks,
            "command_last_accepted": self.command_last_accepted,
            "input_gate_ready_samples": self.input_gate_ready_samples,
            "max_odom_tf_rejections": self.max_odom_tf_rejections,
            "max_cloud_pose_rejections": self.max_cloud_pose_rejections,
            "max_consecutive_input_stale_s": self.max_consecutive_input_stale_s,
            "navigation_loop_overrun_samples_ms": list(self.navigation_loop_overrun_samples_ms),
            "navigation_loop_overrun_p95_ms": self.navigation_loop_overrun_percentile_ms(95.0),
            "navigation_loop_overrun_p99_ms": self.navigation_loop_overrun_percentile_ms(99.0),
            "max_navigation_loop_overrun_ms": self.max_navigation_loop_overrun_ms,
            "max_navigation_loop_overrun_context": dict(self.max_navigation_loop_overrun_context),
            "near_field_stop_samples": self.near_field_stop_samples,
            "pre_safety_command_samples": self.pre_safety_command_samples,
            "max_final_safety_cost": self.max_final_safety_cost,
            "min_map_odom_tf_age_s": (
                self.min_map_odom_tf_age_s if math.isfinite(self.min_map_odom_tf_age_s) else -1.0
            ),
            "local_reason_counts": dict(sorted(self.local_reason_counts.items())),
            "last_nav": self.last_nav,
            "last_slam": self.last_slam,
            "last_traversability": self.last_traversability,
        }


def _validate_map(manifest: dict[str, Any]) -> tuple[dict[str, Path], list[str], dict[str, Any]]:
    blockers: list[str] = []
    map_dir_override = str(os.environ.get("LINGTU_MUJOCO_MAP_DIR", ""))
    map_dir = _repo_path(map_dir_override or str(manifest.get("map_dir") or ""))
    names = manifest.get("map_files") or {}
    paths = {
        "map_dir": map_dir,
        "slam": map_dir / str(names.get("slam") or "map.pcd"),
        "planner": map_dir / str(names.get("planner") or "octomap.ot"),
        "metadata": map_dir / str(names.get("metadata") or "metadata.json"),
    }
    for name in ("slam", "planner", "metadata"):
        if not paths[name].is_file():
            blockers.append(f"map_artifact_missing:{name}:{paths[name]}")

    provenance: dict[str, Any] = {}
    metadata = _load_json(paths["metadata"]) if paths["metadata"].is_file() else {}
    if metadata and paths["slam"].is_file() and paths["planner"].is_file():
        artifacts = metadata.get("artifacts") or {}
        map_pcd = artifacts.get("map_pcd") or {}
        octomap = (metadata.get("artifacts") or {}).get("octomap") or {}
        metadata_frame = str(metadata.get("frame_id") or "").strip()
        map_frame = str(map_pcd.get("frame_id") or "").strip()
        planner_frame = str(octomap.get("frame_id") or "").strip()
        provenance = {
            "map_exists": paths["slam"].is_file(),
            "map_format_ok": paths["slam"].stat().st_size > 0,
            "planner_exists": paths["planner"].is_file(),
            "planner_format_ok": paths["planner"].stat().st_size > 0,
            "metadata_frame_id": metadata_frame,
        }
        if not provenance["map_format_ok"]:
            blockers.append("map_format_invalid")
        if not provenance["planner_format_ok"]:
            blockers.append("octomap_format_invalid")
        if not metadata_frame or map_frame != metadata_frame or planner_frame != metadata_frame:
            blockers.append("map_artifact_metadata_inconsistent")
        identity = manifest.get("source_identity") or {}
        expected_identity = identity.get("map_artifacts") or {}
        actual_schema = str(metadata.get("schema_version") or "")
        provenance["metadata_schema"] = actual_schema
        declared_schema = str(expected_identity.get("metadata_schema") or "")
        if declared_schema and declared_schema != actual_schema:
            blockers.append("selected_map_metadata_schema_mismatch")
    return paths, blockers, provenance


def _run_saved_map_asset_builder(
    spec: dict[str, Any],
    out_dir: Path,
) -> dict[str, Any]:
    from sim.scripts.mujoco import saved_map_plan_gate

    arguments = [
        "--out-dir",
        str(out_dir),
        "--scene-preset",
        str(spec.get("scene_preset") or "corridor"),
        "--length",
        str(float(spec.get("length") or 3.0)),
        "--width",
        str(float(spec.get("width") or 1.8)),
        "--spacing",
        str(float(spec.get("spacing") or 0.2)),
        "--hits-per-cell",
        str(int(spec.get("hits_per_cell") or 4)),
        "--resolution",
        str(float(spec.get("resolution") or 0.2)),
        "--map-source",
        str(spec.get("map_source") or "mujoco_lidar"),
        "--skip-plan",
    ]
    if "support_dilation_cells" in spec:
        arguments.extend(
            ["--support-dilation-cells", str(int(spec["support_dilation_cells"]))]
        )
    if str(spec.get("map_source") or "mujoco_lidar") == "mujoco_lidar":
        arguments.extend(
            [
                "--lidar-scans",
                str(int(spec.get("lidar_scans") or 12)),
                "--lidar-duration",
                str(float(spec.get("lidar_duration_s") or 8.0)),
                "--lidar-timeout",
                str(float(spec.get("lidar_timeout_s") or 20.0)),
                "--lidar-vx",
                str(float(spec.get("lidar_vx_mps") or 0.2)),
                "--lidar-publish-hz",
                str(float(spec.get("lidar_publish_hz") or 10.0)),
                "--mid360-samples-per-frame",
                str(int(spec.get("mid360_samples_per_frame") or 15000)),
            ]
        )
    builder_args = saved_map_plan_gate.build_parser().parse_args(arguments)
    return saved_map_plan_gate.run_gate(builder_args)


def _prepare_acceptance_assets(
    manifest: dict[str, Any],
    out_dir: Path,
) -> dict[str, Any]:
    world_value = str(manifest.get("world") or "")
    world_path = _repo_path(world_value) if world_value else None
    _, map_blockers, _ = _validate_map(manifest)
    if world_path is not None and world_path.is_file() and not map_blockers:
        from sim.scripts.mujoco.comparison_scene import (
            build_comparison_scene,
            is_comparison_scene,
        )

        effective_world = world_path
        presentation: dict[str, Any] = {
            "applied": False,
            "reason": "not_formal_comparison_scene",
        }
        if is_comparison_scene(world_path):
            effective_world = build_comparison_scene(
                world_path,
                out_dir / "prepared_assets" / "cmu_scan_comparison_scene.xml",
            )
            manifest["world"] = str(effective_world)
            presentation = {
                "applied": True,
                "source_scene": str(world_path),
                "effective_scene": str(effective_world),
                "collision_geometry": "preserved_from_source",
                "display_geometry_group": 5,
                "product_lidar_groups": [0, 1],
            }
        return {
            "attempted": False,
            "ok": True,
            "reason": "configured_assets_ready",
            "scene_xml": str(effective_world),
            "map_dir": str(_repo_path(str(manifest.get("map_dir") or ""))),
            "presentation": presentation,
        }

    spec = manifest.get("asset_builder") or {}
    if not isinstance(spec, dict) or not spec:
        return {
            "attempted": False,
            "ok": False,
            "reason": "asset_builder_not_configured",
        }
    if str(spec.get("kind") or "") != "saved_map_plan_gate":
        return {
            "attempted": False,
            "ok": False,
            "reason": f"unsupported_asset_builder:{spec.get('kind')}",
        }

    prepared_dir = out_dir / "prepared_assets"
    try:
        report = _run_saved_map_asset_builder(spec, prepared_dir)
    except Exception as exc:
        return {
            "attempted": True,
            "ok": False,
            "reason": f"asset_builder_failed:{type(exc).__name__}:{exc}",
            "out_dir": str(prepared_dir),
        }

    scene_xml = Path(str(report.get("scene_xml") or "")).expanduser().resolve()
    map_dir = Path(str(report.get("map_dir") or "")).expanduser().resolve()
    build = dict(report.get("build") or {})
    artifact_gate = dict(report.get("artifact_gate") or {})
    build_ok = bool(build.get("ok"))
    artifact_ok = bool(artifact_gate.get("ok"))
    ok = build_ok and artifact_ok and scene_xml.is_file() and map_dir.is_dir()
    blockers: list[str] = []
    if not build_ok:
        nested_build = build.get("octomap_result")
        build_reason = (
            str((nested_build or {}).get("reason_code") or "")
            if isinstance(nested_build, dict)
            else ""
        )
        build_reason = build_reason or str(
            build.get("reason_code") or build.get("status") or "unknown"
        )
        blockers.append(f"asset_octomap_build_failed:{build_reason}")
    if not artifact_ok:
        blockers.append("asset_artifact_gate_failed")
    if not scene_xml.is_file():
        blockers.append(f"asset_scene_missing:{scene_xml}")
    if not map_dir.is_dir():
        blockers.append(f"asset_map_dir_missing:{map_dir}")
    if ok:
        manifest["world"] = str(scene_xml)
        manifest["map_dir"] = str(map_dir)
    return {
        "attempted": True,
        "ok": ok,
        "reason": "prepared" if ok else "asset_builder_incomplete",
        "map_source": str(spec.get("map_source") or "mujoco_lidar"),
        "point_count": int(report.get("point_count") or 0),
        "registration_source": (
            "mujoco_ground_truth_pose"
            if str(spec.get("map_source") or "mujoco_lidar") == "mujoco_lidar"
            else "synthetic_scene_geometry"
        ),
        "out_dir": str(prepared_dir),
        "scene_xml": str(scene_xml),
        "map_dir": str(map_dir),
        "build_ok": build_ok,
        "artifact_gate_ok": artifact_ok,
        "blockers": blockers,
        "build": build,
        "artifact_gate": artifact_gate,
    }


def _preflight_runtime(
    manifest: dict[str, Any],
    *,
    require_map: bool,
) -> tuple[dict[str, Path], dict[str, Path], list[str], dict[str, Any]]:
    blockers: list[str] = []
    binaries: dict[str, Path] = {}
    state_provider = str(
        ((manifest.get("slam_runtime") or {}).get("provider") or "fastlio2")
    ).strip().lower()
    require_traversability = bool(
        ((manifest.get("thresholds") or {}).get("require_traversability", True))
    )
    for name, raw_spec in (manifest.get("binaries") or {}).items():
        if name == "slam" and state_provider == "mujoco_navigation_fixture":
            continue
        if name == "traversability" and not require_traversability:
            continue
        spec = raw_spec if isinstance(raw_spec, dict) else {}
        binary = _resolve_binary(spec)
        if binary is None:
            blockers.append(f"native_binary_missing:{name}")
        else:
            binaries[str(name)] = binary
    if "navigation" in binaries and "driver_bridge" in binaries:
        try:
            _validated_native_clock_platform(
                binaries["navigation"],
                binaries["driver_bridge"],
            )
        except ValueError:
            blockers.append("native_driver_clock_platform_mismatch")
    if "sensor_publisher" in binaries:
        publisher_ok, publisher_probe = _probe_sensor_publisher(binaries["sensor_publisher"])
        if not publisher_ok:
            blockers.append("sensor_publisher_dds_unavailable")
        provenance_probe = {"sensor_publisher_probe": publisher_probe}
    else:
        provenance_probe = {}

    paths_cfg = manifest.get("paths") or {}
    paths = {
        "sensor_runner": _repo_path(str(paths_cfg.get("sensor_runner") or "")),
    }
    if _local_planner_backend(manifest) == "cmu":
        paths["path_library"] = _repo_path(
            str(paths_cfg.get("path_library") or "")
        )
    if state_provider != "mujoco_navigation_fixture":
        paths["slam_config"] = _repo_path(str(paths_cfg.get("slam_config") or ""))
    for name, path in paths.items():
        if not path.exists():
            blockers.append(f"runtime_path_missing:{name}:{path}")

    world_value = str(manifest.get("world") or "")
    world_candidate = _repo_path(world_value) if world_value else None
    if world_candidate is not None and ("/" in world_value or "\\" in world_value):
        paths["world"] = world_candidate
        if not world_candidate.is_file():
            blockers.append(f"runtime_path_missing:world:{world_candidate}")

    if require_map:
        map_paths, map_blockers, provenance = _validate_map(manifest)
        blockers.extend(map_blockers)
        paths.update(map_paths)
    else:
        provenance = {
            "map_contract": {
                "required": False,
                "reason": "caller_selected_map_free_product",
            }
        }

    policy_value = str(paths_cfg.get("policy") or "").strip()
    policy = (
        _repo_path(policy_value)
        if policy_value
        else DEFAULT_THUNDERV4_ONNX_POLICY
    )
    paths["policy"] = policy
    if not policy.is_file():
        blockers.append(f"thunderv4_policy_missing:{policy}")
    driver_bridge = binaries.get("driver_bridge")
    if driver_bridge is not None:
        provenance["driver_bridge"] = {
            "path": str(driver_bridge),
        }
    provenance.update(provenance_probe)
    return binaries, paths, blockers, provenance


def _mirror_native_runtime_inputs(
    paths: dict[str, Path],
    out_dir: Path,
) -> tuple[dict[str, Path], dict[str, Any]]:
    """Keep native read-heavy inputs on the same Linux filesystem as workers.

    On Windows the native processes run inside WSL.  Passing repository paths
    through ``/mnt/<drive>`` is a 9p access path and makes octomap planning and
    path-library lookup contend with the control loop.  The supervisor owns a
    per-run ext4 mirror; source files remain untouched and all provenance stays
    visible in the acceptance report.
    """

    if os.name != "nt":
        return paths, {"enabled": False, "reason": "native_linux"}
    storage_ok, storage_detail = _artifact_storage_probe(out_dir)
    if not storage_ok:
        return paths, {
            "enabled": False,
            "reason": storage_detail,
            "ok": False,
        }

    mirror_root = (out_dir / "native_assets").resolve()
    mirrored = dict(paths)
    sources: dict[str, str] = {}
    destinations: dict[str, str] = {}

    def copy_file(name: str, source: Path, destination: Path) -> None:
        source = source.resolve()
        destination = destination.resolve()
        if not source.is_file():
            return
        try:
            destination.relative_to(source)
        except ValueError:
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, destination)
        mirrored[name] = destination
        sources[name] = str(source)
        destinations[name] = str(destination)

    map_dir = paths.get("map_dir")
    if map_dir is not None and map_dir.is_dir():
        source_map = map_dir.resolve()
        destination_map = (mirror_root / "map").resolve()
        if source_map != destination_map:
            shutil.copytree(source_map, destination_map, dirs_exist_ok=True)
        mirrored["map_dir"] = destination_map
        sources["map_dir"] = str(source_map)
        destinations["map_dir"] = str(destination_map)
        for name in ("slam", "planner", "metadata"):
            source = paths.get(name)
            if source is None:
                continue
            try:
                relative = source.resolve().relative_to(source_map)
            except ValueError:
                continue
            mirrored[name] = destination_map / relative
            sources[name] = str(source.resolve())
            destinations[name] = str((destination_map / relative).resolve())

    path_library = paths.get("path_library")
    if path_library is not None and path_library.is_dir():
        source_library = path_library.resolve()
        destination_library = (mirror_root / "path_library").resolve()
        if source_library != destination_library:
            shutil.copytree(source_library, destination_library, dirs_exist_ok=True)
        mirrored["path_library"] = destination_library
        sources["path_library"] = str(source_library)
        destinations["path_library"] = str(destination_library)

    slam_config = paths.get("slam_config")
    if slam_config is not None:
        copy_file("slam_config", slam_config, mirror_root / "slam_config.yaml")

    return mirrored, {
        "enabled": True,
        "ok": True,
        "filesystem": storage_detail,
        "root": str(mirror_root),
        "sources": sources,
        "destinations": destinations,
    }


def _preflight(
    manifest: dict[str, Any],
) -> tuple[dict[str, Path], dict[str, Path], list[str], dict[str, Any]]:
    """Resolve generic native-navigation inputs with a mandatory saved map."""

    return _preflight_runtime(manifest, require_map=True)


def _preflight_map_free(
    manifest: dict[str, Any],
) -> tuple[dict[str, Path], dict[str, Path], list[str], dict[str, Any]]:
    """Resolve runtime inputs for a Product that explicitly does not own a map."""

    return _preflight_runtime(manifest, require_map=False)


def _build_helper() -> dict[str, Any]:
    source = ROOT / "sim" / "adapters" / "dds"
    build = ROOT / "build" / "mujoco_native_dds"
    commands = (
        [
            ["wsl.exe", "-e", "cmake", "-S", _wsl_path(source), "-B", _wsl_path(build), "-DCMAKE_BUILD_TYPE=Release"],
            ["wsl.exe", "-e", "cmake", "--build", _wsl_path(build), "--parallel"],
        ]
        if os.name == "nt"
        else [
            ["cmake", "-S", str(source), "-B", str(build), "-DCMAKE_BUILD_TYPE=Release"],
            ["cmake", "--build", str(build), "--parallel"],
        ]
    )
    results = []
    for command in commands:
        proc = subprocess.run(
            command,
            cwd=ROOT,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            check=False,
        )
        results.append(
            {
                "command": command,
                "returncode": proc.returncode,
                "stdout_tail": _text_tail(proc.stdout),
                "stderr_tail": _text_tail(proc.stderr),
            }
        )
        if proc.returncode != 0:
            break
    return {"ok": len(results) == len(commands) and all(item["returncode"] == 0 for item in results), "steps": results}


def _valid_slam_odometry(slam: dict[str, Any]) -> bool:
    if not bool(slam.get("has_odom")):
        return False
    odometry = slam.get("odometry") or {}
    pose = odometry.get("pose") or {}
    if not str(odometry.get("frame_id") or "") or not str(odometry.get("child_frame_id") or ""):
        return False
    try:
        stamp_s = float(slam.get("stamp_s"))
        values = [float(pose[name]) for name in ("x", "y", "z", "qx", "qy", "qz", "qw")]
    except (KeyError, TypeError, ValueError):
        return False
    if stamp_s <= 0.0 or not math.isfinite(stamp_s) or not all(math.isfinite(value) for value in values):
        return False
    quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
    return 0.5 <= quaternion_norm <= 1.5


def _slam_navigation_readiness(
    *,
    slam: dict[str, Any],
    nav: dict[str, Any],
    thresholds: dict[str, Any],
) -> str | None:
    if str(thresholds.get("navigation_state_provider") or "").strip().lower() == "mujoco_navigation_fixture":
        if bool(nav.get("has_odom")) and int((nav.get("counters") or {}).get("odom") or 0) > 0:
            return "mujoco_navigation_fixture"
        return None
    if str(slam.get("state") or "").upper() == "TRACKING":
        return "tracking"
    require_slam_tracking = bool(thresholds.get("require_slam_tracking", True))
    allow_degraded = bool(thresholds.get("allow_degraded_slam_for_navigation_isolation", False))
    if require_slam_tracking and not allow_degraded:
        return None
    if require_slam_tracking and str(slam.get("state") or "").upper() != "DEGRADED":
        return None
    if not bool(slam.get("odom_prior_enabled")) or not bool(slam.get("odom_prior_active")):
        return None
    if not _valid_slam_odometry(slam):
        return None
    if not bool(nav.get("has_odom")):
        return None
    if int((nav.get("counters") or {}).get("odom") or 0) <= 0:
        return None
    return "navigation_isolation_odom_prior"


def _navigation_sensor_assessment(
    sensor_report: dict[str, Any],
    thresholds: dict[str, Any],
) -> dict[str, Any]:
    gaps = [str(value) for value in sensor_report.get("remaining_gaps") or []]
    require_slam_accuracy = bool(thresholds.get("require_slam_accuracy", True))
    non_blocking: list[str] = []
    blocking: list[str] = []
    for gap in gaps:
        is_accuracy_gap = gap.startswith(_NON_BLOCKING_SLAM_ACCURACY_GAP_PREFIXES)
        if is_accuracy_gap and not require_slam_accuracy:
            non_blocking.append(gap)
        else:
            blocking.append(gap)
    if sensor_report.get("ok") is not True and not gaps:
        blocking.append("sensor_report_not_ok_without_gap")
    return {
        "navigation_critical_ok": not blocking,
        "require_slam_accuracy": require_slam_accuracy,
        "blocking_gaps": blocking,
        "non_blocking_slam_accuracy_gaps": non_blocking,
        "raw_sensor_report_ok": sensor_report.get("ok") is True,
    }


def _wait_for_startup(
    *,
    sensor: ManagedProcess,
    evidence: NativeEvidence,
    nav_status: Path,
    slam_status: Path,
    traversability_status: Path,
    mapd_status: Path,
    timeout_s: float,
    thresholds: dict[str, Any],
) -> tuple[bool, str]:
    deadline = time.monotonic() + max(0.1, timeout_s)
    required_stable_s = max(0.0, float(thresholds.get("startup_ready_stable_s") or 0.0))
    require_traversability = bool(thresholds.get("require_traversability", True))
    ready_since_s: float | None = None
    while time.monotonic() < deadline:
        evidence.sample(
            nav_path=nav_status,
            slam_path=slam_status,
            traversability_path=traversability_status,
            collect_motion_health=False,
        )
        if sensor.poll() is not None:
            return False, "sensor_runner_exited_before_native_runtime_ready"
        slam = evidence.last_slam
        nav = evidence.last_nav
        traversability = evidence.last_traversability
        mapd = _load_json(mapd_status)
        slam_readiness = _slam_navigation_readiness(
            slam=slam,
            nav=nav,
            thresholds=thresholds,
        )
        fixture_gate_ready = (
            bool((nav.get("input_gate") or {}).get("ready"))
            if slam_readiness == "mujoco_navigation_fixture"
            else True
        )
        control_loop = nav.get("control_loop_health") or {}
        control_loop_ready = bool(control_loop.get("ready")) and bool(
            control_loop.get("healthy")
        )
        runtime_ready = (
            slam_readiness is not None
            and bool(nav.get("has_odom"))
            and fixture_gate_ready
            and control_loop_ready
            and (
                not require_traversability
                or int((traversability.get("counters") or {}).get("published") or 0) > 0
            )
            and mapd.get("ready") is True
            and mapd.get("live") is True
            and int(mapd.get("accepted_observations") or 0) > 0
            and int(mapd.get("map_layers_published_generation") or 0) > 0
            and int(
                (((nav.get("local_map") or {}).get("collision") or {}).get("generation"))
                or 0
            )
            > 0
        )
        if runtime_ready:
            now_s = time.monotonic()
            if ready_since_s is None:
                ready_since_s = now_s
            if now_s - ready_since_s < required_stable_s:
                time.sleep(0.1)
                continue
            if slam_readiness == "tracking":
                return True, "ready"
            return True, f"ready_{slam_readiness}"
        ready_since_s = None
        time.sleep(0.1)
    return False, "native_runtime_startup_timeout"


def _goal_command(
    binary: Path,
    goal: list[float],
    domain_id: int,
    timeout_s: float,
) -> dict[str, Any]:
    timeout_ms = max(1000, int(math.ceil(float(timeout_s) * 1000.0)))
    command = _native_command(
        binary,
        "goal",
        *(str(float(value)) for value in goal),
        "--domain-id",
        str(domain_id),
        "--timeout-ms",
        str(timeout_ms),
    )
    proc = subprocess.run(
        command,
        cwd=ROOT,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=max(5.0, float(timeout_s) + 5.0),
        check=False,
    )
    return {
        "command": command,
        "returncode": proc.returncode,
        "stdout": _text_tail(proc.stdout),
        "stderr": _text_tail(proc.stderr),
    }


def _resume_command(
    binary: Path,
    domain_id: int,
    timeout_s: float,
) -> dict[str, Any]:
    timeout_ms = max(1000, int(math.ceil(float(timeout_s) * 1000.0)))
    command = _native_command(
        binary,
        "resume",
        "mujoco_acceptance_startup",
        "--domain-id",
        str(domain_id),
        "--timeout-ms",
        str(timeout_ms),
    )
    proc = subprocess.run(
        command,
        cwd=ROOT,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=max(5.0, float(timeout_s) + 5.0),
        check=False,
    )
    return {
        "command": command,
        "returncode": proc.returncode,
        "stdout": _text_tail(proc.stdout),
        "stderr": _text_tail(proc.stderr),
    }


def _resume_when_ready(
    binary: Path,
    domain_id: int,
    timeout_s: float,
) -> dict[str, Any]:
    deadline = time.monotonic() + min(3.0, max(0.1, float(timeout_s)))
    attempts: list[dict[str, Any]] = []
    while True:
        result = _resume_command(binary, domain_id, timeout_s)
        attempts.append(dict(result))
        diagnostic = f"{result.get('stdout') or ''}\n{result.get('stderr') or ''}"
        if (
            int(result.get("returncode") or 0) == 0
            or "control_loop_recovery_pending" not in diagnostic
            or time.monotonic() >= deadline
        ):
            outcome = dict(result)
            outcome["attempts"] = attempts
            return outcome
        time.sleep(0.1)


def _deferred_goal_command(
    binary: Path,
    goal: list[float],
    domain_id: int,
    timeout_s: float,
    trigger_path: Path,
) -> list[str]:
    """Build a prestarted WSL relay that waits for a host-visible trigger.

    Starting the WSL session with the rest of the runtime avoids launching a
    new ``wsl.exe`` at the goal-submission instant.  The trigger releases the
    relay, which then execs the native DDS client; Python never publishes or
    transforms the navigation request.
    """

    timeout_ms = max(1000, int(math.ceil(float(timeout_s) * 1000.0)))
    native = _native_command(
        binary,
        "goal",
        *(str(float(value)) for value in goal),
        "--domain-id",
        str(domain_id),
        "--timeout-ms",
        str(timeout_ms),
    )
    if os.name != "nt" or len(native) < 3 or native[1] != "-e":
        raise ValueError("deferred goal launch requires a Windows-to-WSL native command")
    script = 'trigger="$1"; shift; while [ ! -f "$trigger" ]; do sleep 0.05; done; exec "$@"'
    return [
        native[0],
        "-e",
        "bash",
        "-lc",
        script,
        "lingtu-deferred-goal",
        _wsl_path(trigger_path),
        *native[2:],
    ]


def _goal_metrics(sensor_report: dict[str, Any], goal: list[float]) -> dict[str, Any]:
    motion = sensor_report.get("motion") or {}
    start = motion.get("sim_start_xyz") or []
    end = motion.get("sim_end_xyz") or []
    if len(start) < 2 or len(end) < 2:
        return {"available": False}
    start_distance = math.hypot(float(goal[0]) - float(start[0]), float(goal[1]) - float(start[1]))
    end_distance = math.hypot(float(goal[0]) - float(end[0]), float(goal[1]) - float(end[1]))
    metrics = {
        "available": True,
        "start_distance_m": start_distance,
        "end_distance_m": end_distance,
        "start_error_xy_m": start_distance,
        "end_error_xy_m": end_distance,
        "distance_reduction_m": start_distance - end_distance,
        "net_displacement_xy_m": math.hypot(
            float(end[0]) - float(start[0]),
            float(end[1]) - float(start[1]),
        ),
        "sim_path_length_xy_m": float(motion.get("sim_path_length_xy_m") or 0.0),
    }
    if len(goal) >= 3 and len(start) >= 3 and len(end) >= 3:
        start_z_error = abs(float(goal[2]) - float(start[2]))
        end_z_error = abs(float(goal[2]) - float(end[2]))
        start_3d_error = math.hypot(start_distance, start_z_error)
        end_3d_error = math.hypot(end_distance, end_z_error)
        metrics.update(
            {
                "start_error_z_m": start_z_error,
                "end_error_z_m": end_z_error,
                "start_error_3d_m": start_3d_error,
                "end_error_3d_m": end_3d_error,
                "distance_reduction_3d_m": start_3d_error - end_3d_error,
            }
        )
    return metrics


def _percentile(values: list[float], percentile: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.inf
    position = (len(ordered) - 1) * max(0.0, min(100.0, percentile)) / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _point_to_segment_distance(
    point: tuple[float, ...],
    start: tuple[float, ...],
    end: tuple[float, ...],
) -> float:
    delta = tuple(end[index] - start[index] for index in range(len(point)))
    length_squared = sum(value * value for value in delta)
    if length_squared <= 1e-12:
        return math.sqrt(
            sum((point[index] - start[index]) ** 2 for index in range(len(point)))
        )
    projection = sum(
        (point[index] - start[index]) * delta[index]
        for index in range(len(point))
    ) / length_squared
    projection = max(0.0, min(1.0, projection))
    return math.sqrt(
        sum(
            (
                point[index]
                - (start[index] + projection * delta[index])
            )
            ** 2
            for index in range(len(point))
        )
    )


def _polyline_distance(point: tuple[float, ...], raw_path: Any) -> float | None:
    path: list[tuple[float, ...]] = []
    for raw_point in raw_path if isinstance(raw_path, list) else []:
        if not isinstance(raw_point, list) or len(raw_point) < len(point):
            continue
        try:
            candidate = tuple(float(raw_point[index]) for index in range(len(point)))
        except (TypeError, ValueError):
            continue
        if all(math.isfinite(value) for value in candidate):
            path.append(candidate)
    if len(path) < 2:
        return None
    return min(
        _point_to_segment_distance(point, start, end)
        for start, end in zip(path, path[1:])
    )


def _trajectory_tracking_metrics(motion_log: Path) -> dict[str, Any]:
    xy_errors: list[float] = []
    errors_3d: list[float] = []
    sample_times: list[float] = []
    if not motion_log.is_file():
        return {"available": False, "samples": 0}
    for line in motion_log.read_text(encoding="utf-8").splitlines():
        try:
            row = json.loads(line)
            if not isinstance(row, dict) or not bool(row.get("driving")):
                continue
            point_xy = (float(row["x"]), float(row["y"]))
            point_3d = (point_xy[0], point_xy[1], float(row["z"]))
            timestamp = float(row.get("sim_time_s"))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            continue
        if not all(math.isfinite(value) for value in (*point_3d, timestamp)):
            continue
        xy_error = _polyline_distance(point_xy, row.get("local_path"))
        error_3d = _polyline_distance(point_3d, row.get("local_path"))
        if xy_error is None:
            continue
        xy_errors.append(xy_error)
        if error_3d is not None:
            errors_3d.append(error_3d)
        sample_times.append(timestamp)
    if not xy_errors:
        return {"available": False, "samples": 0}
    result = {
        "available": True,
        "samples": len(xy_errors),
        "elapsed_s": max(sample_times) - min(sample_times),
        "rmse_xy_m": math.sqrt(sum(value * value for value in xy_errors) / len(xy_errors)),
        "p95_xy_error_m": _percentile(xy_errors, 95.0),
        "max_xy_error_m": max(xy_errors),
    }
    if errors_3d:
        result.update(
            {
                "rmse_3d_m": math.sqrt(
                    sum(value * value for value in errors_3d) / len(errors_3d)
                ),
                "p95_3d_error_m": _percentile(errors_3d, 95.0),
                "max_3d_error_m": max(errors_3d),
            }
        )
    return result


def _mujoco_truth_speed_blocker(
    sensor_report: Mapping[str, Any],
    thresholds: Mapping[str, Any],
) -> str | None:
    required = thresholds.get("min_mujoco_truth_peak_xy_speed_mps")
    if required is None:
        return None
    observed = (sensor_report.get("mujoco_truth_velocity") or {}).get(
        "max_xy_speed_mps"
    )
    try:
        required_speed = float(required)
        observed_speed = float(observed)
    except (TypeError, ValueError):
        return "mujoco_truth_speed_evidence_missing"
    if not math.isfinite(required_speed) or required_speed < 0.0:
        raise ValueError(
            "thresholds.min_mujoco_truth_peak_xy_speed_mps must be finite and non-negative"
        )
    if not math.isfinite(observed_speed):
        return "mujoco_truth_speed_evidence_missing"
    if observed_speed + 1e-9 < required_speed:
        return "mujoco_truth_peak_xy_speed_below_threshold"
    return None


def _planner_constraint_args(manifest: dict[str, Any]) -> list[str]:
    config = dict(manifest.get("planner_constraints") or {})
    options = {
        "robot_radius_m": ("--octo-robot-radius-m", float),
        "body_clearance_below_m": ("--octo-body-clearance-below-m", float),
        "body_clearance_above_m": ("--octo-body-clearance-above-m", float),
        "max_iterations": ("--octo-max-iterations", int),
        "snap_radius_cells": ("--octo-snap-radius-cells", int),
        "require_ground_support": ("--octo-require-ground-support", bool),
        "strict_ground_support": ("--octo-strict-ground-support", bool),
        "ground_support_xy_radius_cells": ("--octo-ground-support-xy-radius-cells", int),
        "ground_support_depth_cells": ("--octo-ground-support-depth-cells", int),
        "support_height_m": ("--octo-support-height-m", float),
        "support_height_tolerance_m": ("--octo-support-height-tolerance-m", float),
        "support_patch_radius_cells": ("--octo-support-patch-radius-cells", int),
        "support_patch_min_samples": ("--octo-support-patch-min-samples", int),
        "enable_preblocked_costmap": ("--octo-enable-preblocked-costmap", bool),
        "preblocked_radius_cells": ("--octo-preblocked-radius-cells", int),
        "preblocked_weight": ("--octo-preblocked-weight", float),
        "lowest_traversable_only": ("--octo-lowest-traversable-only", bool),
        "floor_change_penalty": ("--octo-floor-change-penalty", float),
        "max_step_height_m": ("--octo-max-step-height-m", float),
        "max_slope": ("--octo-max-slope", float),
        "same_floor_preference": ("--octo-same-floor-preference", bool),
        "same_floor_z_tolerance_m": ("--octo-same-floor-z-tolerance-m", float),
        "max_same_floor_z_excursion_m": ("--octo-max-same-floor-z-excursion-m", float),
        "obstacle_clearance_radius_cells": ("--octo-obstacle-clearance-radius-cells", int),
        "obstacle_clearance_weight": ("--octo-obstacle-clearance-weight", float),
        "terminal_goal_tolerance_m": ("--octo-terminal-goal-tolerance-m", float),
        "terminal_goal_xy_tolerance_m": ("--octo-terminal-goal-xy-tolerance-m", float),
        "terminal_goal_z_tolerance_m": ("--octo-terminal-goal-z-tolerance-m", float),
    }
    unknown = sorted(set(config) - set(options))
    if unknown:
        raise ValueError(f"unknown OctoPlanner3D constraints: {', '.join(unknown)}")
    arguments: list[str] = []
    for key, (option, value_type) in options.items():
        if key not in config:
            continue
        raw = config[key]
        if value_type is bool:
            value = "true" if bool(raw) else "false"
        else:
            value = str(value_type(raw))
        arguments.extend([option, value])
    return arguments


def _path_follower_args(runtime: Mapping[str, Any]) -> list[str]:
    options = {
        "route_snap_m": ("--waypoint-reached-m", 0.0, True),
        "goal_reached_m": ("--goal-reached-m", 0.0, True),
        "path_follower_goal_tolerance_m": ("--path-follower-goal-tolerance-m", 0.0, False),
        "path_follower_lookahead_m": ("--path-follower-lookahead-m", 0.0, True),
        "path_follower_max_accel_mps2": ("--path-follower-max-accel-mps2", 0.0, True),
        "path_follower_max_speed_mps": ("--path-follower-max-speed-mps", 0.0, True),
        "path_follower_min_speed_mps": ("--path-follower-min-speed-mps", 0.0, False),
        "path_follower_max_yaw_rate_rad_s": ("--path-follower-max-yaw-rate-rad-s", 0.0, False),
        "path_follower_heading_align_enter_rad": (
            "--path-follower-heading-align-enter-rad",
            0.0,
            True,
        ),
        "path_follower_heading_align_exit_rad": (
            "--path-follower-heading-align-exit-rad",
            0.0,
            False,
        ),
    }
    values = {
        key: float(runtime[key])
        for key in options
        if key in runtime
    }
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError("navigation_runtime path follower values must be finite")
    for key, value in values.items():
        _, lower_bound, strictly_greater = options[key]
        if value < lower_bound or (strictly_greater and value == lower_bound):
            raise ValueError("navigation_runtime path follower values are inconsistent")
    minimum = values.get("path_follower_min_speed_mps")
    maximum = values.get("path_follower_max_speed_mps")
    if minimum is not None and maximum is not None and minimum > maximum:
        raise ValueError("navigation_runtime path follower values are inconsistent")
    align_enter = values.get("path_follower_heading_align_enter_rad")
    align_exit = values.get("path_follower_heading_align_exit_rad")
    if align_enter is not None and align_enter > math.pi:
        raise ValueError("navigation_runtime path follower values are inconsistent")
    if align_enter is not None and align_exit is not None and align_exit >= align_enter:
        raise ValueError("navigation_runtime path follower values are inconsistent")
    arguments: list[str] = []
    for key, (option, _, _) in options.items():
        if key in values:
            arguments.extend([option, str(values[key])])
    return arguments


def _sensor_start_args(start: list[float]) -> list[str]:
    if len(start) < 3:
        raise ValueError("MuJoCo navigation start pose requires x, y, and z")
    position = [float(value) for value in start[:3]]
    if not all(math.isfinite(value) for value in position):
        raise ValueError("MuJoCo navigation start position must be finite")
    arguments = ["--start", ",".join(str(value) for value in position)]
    if len(start) >= 4:
        yaw_rad = float(start[3])
        if not math.isfinite(yaw_rad):
            raise ValueError("MuJoCo navigation start yaw must be finite")
        arguments.extend(["--start-yaw-deg", str(math.degrees(yaw_rad))])
    return arguments


def _sensor_runtime_args(manifest: dict[str, Any]) -> list[str]:
    config = dict(manifest.get("sensor_runtime") or {})
    supported = {
        "publish_hz",
        "imu_hz",
        "mid360_samples_per_frame",
        "scan_time_profile",
        "physical_rolling_sample_mode",
        "publish_odom_prior",
        "odom_prior_velocity_window_s",
        "navigation_fixture_cloud_points",
        "navigation_fixture_ground_resolution_m",
        "navigation_fixture_ground_y_half_m",
        "stop_on_nav_goal_reached",
        "physics_integrator",
        "physics_timestep_s",
        "policy_cpu_threads",
        "publisher_write_mode",
    }
    unknown = sorted(set(config) - supported)
    if unknown:
        raise ValueError(f"unsupported sensor_runtime keys: {', '.join(unknown)}")
    arguments = [
        "--publish-hz",
        str(float(config.get("publish_hz") or 10.0)),
        "--imu-hz",
        str(float(config.get("imu_hz") or 200.0)),
        "--mid360-samples-per-frame",
        str(int(config.get("mid360_samples_per_frame") or 15000)),
    ]
    scan_time_profile = str(config.get("scan_time_profile") or "").strip()
    if scan_time_profile:
        if scan_time_profile not in {
            "instantaneous",
            "synthetic_rolling",
            "physical_rolling",
        }:
            raise ValueError(f"unsupported scan_time_profile: {scan_time_profile}")
        arguments.extend(["--scan-time-profile", scan_time_profile])
    if "physical_rolling_sample_mode" in config:
        physical_mode = str(config["physical_rolling_sample_mode"])
        if physical_mode not in {"full_frame", "subscan"}:
            raise ValueError(f"unsupported physical_rolling_sample_mode: {physical_mode}")
        arguments.extend(["--physical-rolling-sample-mode", physical_mode])
    if bool(config.get("publish_odom_prior")):
        arguments.append("--publish-odom-prior")
    if "odom_prior_velocity_window_s" in config:
        arguments.extend(
            [
                "--odom-prior-velocity-window-s",
                str(float(config["odom_prior_velocity_window_s"])),
            ]
        )
    if "navigation_fixture_cloud_points" in config:
        arguments.extend(
            [
                "--navigation-fixture-cloud-points",
                str(int(config["navigation_fixture_cloud_points"])),
            ]
        )
    if "navigation_fixture_ground_resolution_m" in config:
        ground_resolution_m = float(config["navigation_fixture_ground_resolution_m"])
        if (
            not math.isfinite(ground_resolution_m)
            or ground_resolution_m <= 0.0
            or ground_resolution_m > 0.2
        ):
            raise ValueError(
                "navigation_fixture_ground_resolution_m must be in (0, 0.2]"
            )
        arguments.extend(
            ["--navigation-fixture-ground-resolution-m", str(ground_resolution_m)]
        )
    if "navigation_fixture_ground_y_half_m" in config:
        ground_y_half_m = float(config["navigation_fixture_ground_y_half_m"])
        if not math.isfinite(ground_y_half_m) or ground_y_half_m <= 0.0:
            raise ValueError(
                "navigation_fixture_ground_y_half_m must be positive and finite"
            )
        arguments.extend(
            ["--navigation-fixture-ground-y-half-m", str(ground_y_half_m)]
        )
    if bool(config.get("stop_on_nav_goal_reached")):
        arguments.append("--stop-on-nav-goal-reached")
    if "physics_integrator" in config:
        integrator = str(config["physics_integrator"]).strip().lower()
        if integrator not in {"model", "euler", "rk4", "implicit", "implicitfast"}:
            raise ValueError(f"unsupported MuJoCo physics integrator: {integrator}")
        arguments.extend(["--physics-integrator", integrator])
    if "physics_timestep_s" in config:
        timestep_s = float(config["physics_timestep_s"])
        if not math.isfinite(timestep_s) or not 0.0005 <= timestep_s <= 0.005:
            raise ValueError("MuJoCo physics timestep must be in [0.0005, 0.005] seconds")
        arguments.extend(["--physics-timestep-s", str(timestep_s)])
    if "policy_cpu_threads" in config:
        policy_threads = int(config["policy_cpu_threads"])
        if not 1 <= policy_threads <= 8:
            raise ValueError("policy_cpu_threads must be in [1, 8]")
        arguments.extend(["--policy-cpu-threads", str(policy_threads)])
    if "publisher_write_mode" in config:
        write_mode = str(config["publisher_write_mode"]).strip().lower()
        if write_mode not in {"sync", "async_fifo"}:
            raise ValueError("publisher_write_mode must be sync or async_fifo")
        arguments.extend(["--publisher-write-mode", write_mode])
    return arguments


def _motion_capture_settings(
    *,
    record_video: bool,
    record_telemetry: bool,
    video_cfg: dict[str, Any],
    telemetry_cfg: dict[str, Any],
) -> tuple[float, int] | None:
    if not record_video and not record_telemetry:
        return None
    if record_video:
        # Encoding FPS is a presentation concern. The renderer already holds
        # lower-rate source samples to a CFR timeline, so driving the live
        # sensor loop at the encoder rate only adds load and sensor dropouts.
        capture_hz = float(
            video_cfg.get("capture_hz")
            or telemetry_cfg.get("hz")
            or 10.0
        )
        lidar_points = int(video_cfg.get("lidar_points") or 640)
    else:
        capture_hz = float(telemetry_cfg.get("hz") or 10.0)
        lidar_points = int(telemetry_cfg.get("lidar_points") or 0)
    if not math.isfinite(capture_hz) or capture_hz <= 0.0:
        raise ValueError("motion capture rate must be positive and finite")
    if lidar_points < 0:
        raise ValueError("motion capture LiDAR point limit must be non-negative")
    return capture_hz, lidar_points


def _traversability_frame_contract(
    evidence: NativeEvidence,
    contract: dict[str, Any],
) -> dict[str, Any]:
    transform = (evidence.last_slam or {}).get("map_odom_tf") or {}
    try:
        translation_m = math.sqrt(
            float(transform.get("tx") or 0.0) ** 2
            + float(transform.get("ty") or 0.0) ** 2
            + float(transform.get("tz") or 0.0) ** 2
        )
        qw = float(transform.get("qw") if transform.get("qw") is not None else 1.0)
        qw = max(-1.0, min(1.0, abs(qw)))
        rotation_rad = 2.0 * math.acos(qw)
    except (TypeError, ValueError):
        translation_m = math.inf
        rotation_rad = math.inf
    translation_tolerance = float(contract.get("map_odom_identity_translation_tolerance_m") or 0.05)
    rotation_tolerance = float(contract.get("map_odom_identity_rotation_tolerance_rad") or 0.05)
    runtime = (evidence.last_traversability or {}).get("frame_contract") or {}
    producer_geometry = str(
        runtime.get("geometry_frame") or contract.get("traversability_geometry_frame_current") or ""
    )
    producer_header = str(runtime.get("header_frame") or contract.get("traversability_header_frame_current") or "")
    consumer_frame = str(contract.get("navigation_query_frame") or "")
    odom_input_frame = str(runtime.get("odom_input_frame") or "")
    has_map_odom_tf = bool(
        (evidence.last_traversability or {}).get("has_map_odom_tf")
        if "has_map_odom_tf" in (evidence.last_traversability or {})
        else transform
    )
    try:
        runtime_tf_age_s = float(runtime.get("map_odom_tf_age_s") or 0.0)
    except (TypeError, ValueError):
        runtime_tf_age_s = math.inf
    tf_age_s = evidence.min_map_odom_tf_age_s if math.isfinite(evidence.min_map_odom_tf_age_s) else runtime_tf_age_s
    max_tf_age_s = float(contract.get("max_map_odom_tf_age_s") or 0.0)
    mislabeled = bool(producer_geometry and producer_header and producer_geometry != producer_header)
    non_identity = translation_m > translation_tolerance or rotation_rad > rotation_tolerance
    transform_ready = odom_input_frame != "odom" or has_map_odom_tf
    transform_fresh = max_tf_age_s <= 0.0 or tf_age_s <= max_tf_age_s
    aligned = (
        not mislabeled
        and producer_geometry == consumer_frame
        and producer_header == consumer_frame
        and transform_ready
        and transform_fresh
    )
    return {
        "ok": aligned,
        "status": "aligned" if aligned else "mismatch",
        "producer_geometry_frame": producer_geometry,
        "producer_header_frame": producer_header,
        "navigation_query_frame": consumer_frame,
        "odom_input_frame": odom_input_frame,
        "has_map_odom_tf": has_map_odom_tf,
        "map_odom_tf_age_s": tf_age_s,
        "last_map_odom_tf_age_s": runtime_tf_age_s,
        "max_map_odom_tf_age_s": max_tf_age_s,
        "map_odom_translation_m": translation_m,
        "map_odom_rotation_rad": rotation_rad,
        "translation_tolerance_m": translation_tolerance,
        "rotation_tolerance_rad": rotation_tolerance,
        "mislabeled": mislabeled,
        "map_odom_non_identity": non_identity,
        "transform_ready": transform_ready,
        "transform_fresh": transform_fresh,
        "required_fix": str(contract.get("required_fix") or ""),
    }


def _pre_safety_command_evidence(evidence: NativeEvidence) -> dict[str, Any]:
    observed = evidence.pre_safety_command_samples > 0 or evidence.max_path_follower_cmd_norm > 1e-4
    return {
        "observed": observed,
        "samples": evidence.pre_safety_command_samples,
        "raw_path_follower_max_norm": evidence.max_path_follower_cmd_norm,
        "post_safety_max_norm": evidence.max_computed_cmd_norm,
        "raw_vector_available_in_nav_status": True,
        "proof": "nav status stores PathFollower output before the independent final safety gate",
    }


def _evaluate_phase(
    *,
    phase: str,
    phase_cfg: dict[str, Any],
    thresholds: dict[str, Any],
    evidence: NativeEvidence,
    sensor_report: dict[str, Any],
    goal: list[float],
    frame_contract: dict[str, Any] | None = None,
) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    uses_slam_pose = (
        str(thresholds.get("navigation_state_provider") or "").strip().lower()
        != "mujoco_navigation_fixture"
    )
    if (
        _slam_navigation_readiness(
            slam=evidence.last_slam or {},
            nav=evidence.last_nav or {},
            thresholds=thresholds,
        )
        is None
    ):
        blockers.append("slam_not_tracking")
    map_tracking = (evidence.last_slam or {}).get("track_against_map") or {}
    if bool(thresholds.get("require_continuous_map_tracking", True)):
        if int(map_tracking.get("successes") or 0) < int(thresholds.get("min_track_against_map_successes") or 1):
            blockers.append("continuous_map_tracking_not_verified")
        max_track_age_s = thresholds.get("max_track_against_map_success_age_s")
        if max_track_age_s is not None:
            try:
                track_age_s = float(map_tracking.get("last_success_age_s"))
            except (TypeError, ValueError):
                track_age_s = math.inf
            if track_age_s < 0.0 or track_age_s > float(max_track_age_s):
                blockers.append("continuous_map_tracking_stale")
        max_track_failures = thresholds.get("max_consecutive_track_against_map_failures")
        if max_track_failures is not None and int(map_tracking.get("consecutive_failures") or 0) > int(
            max_track_failures
        ):
            blockers.append("continuous_map_tracking_degraded")
    require_traversability = bool(thresholds.get("require_traversability", True))
    if require_traversability:
        if evidence.max_registered_clouds <= 0:
            blockers.append("registered_cloud_not_consumed_by_traversability")
        if evidence.max_traversability_published <= 0:
            blockers.append("traversability_not_published")
    if not evidence.plan_accepted:
        blockers.append("octoplanner3d_plan_not_accepted")
    if evidence.max_global_path_points < int(thresholds.get("min_global_path_points") or 2):
        blockers.append("global_path_too_short")
    if not evidence.local_path_found:
        blockers.append("local_planner_path_not_found")
    if evidence.max_local_path_points < int(thresholds.get("min_local_path_points") or 2):
        blockers.append("local_path_too_short")
    if evidence.max_path_follower_cmd_norm <= 1e-4 and evidence.pre_safety_command_samples <= 0:
        blockers.append("embedded_path_follower_command_missing")
    if evidence.local_path_role != "dds_telemetry_and_preview":
        blockers.append("local_path_role_contract_mismatch")
    if evidence.path_follower_role != "embedded_before_cmd_vel_gate":
        blockers.append("path_follower_role_contract_mismatch")
    if evidence.command_transport != "typed_dds_request_ack":
        blockers.append("navigation_command_transport_mismatch")
    if evidence.max_command_requests <= 0:
        blockers.append("navigation_command_request_not_received")
    if evidence.max_command_acks <= 0:
        blockers.append("navigation_business_ack_not_sent")
    if not evidence.command_last_accepted:
        blockers.append("navigation_business_ack_not_accepted")
    if sensor_report.get("command_source") != "dds":
        blockers.append("mujoco_command_source_is_not_typed_dds")
    if sensor_report.get("policy_loaded") is not True:
        blockers.append("thunderv4_policy_not_loaded")
    if bool(thresholds.get("require_navigation_safety_contracts", False)):
        nav_status = evidence.last_nav or {}
        if str(nav_status.get("control_mode") or "") != "autonomy":
            blockers.append("native_control_mode_not_autonomy")
        if nav_status.get("check_obstacle") is not True:
            blockers.append("obstacle_slow_stop_not_enabled")
        if require_traversability and nav_status.get("use_traversability_cost") is not True:
            blockers.append("terrain_cost_not_enabled")
        input_gate = nav_status.get("input_gate") or {}
        required_inputs = [
            "require_odom",
            "require_cloud",
            "require_localization_health",
        ]
        if require_traversability:
            required_inputs.append("require_traversability")
        for required_input in required_inputs:
            if input_gate.get(required_input) is not True:
                blockers.append(f"stale_fail_safe_contract_missing:{required_input}")

    cmd_vel = sensor_report.get("cmd_vel") or {}
    motion = sensor_report.get("motion") or {}
    publish_cmd_vel = bool(phase_cfg.get("publish_cmd_vel"))
    if phase == "no_motion":
        if evidence.max_cmd_vel_published != 0:
            blockers.append("no_motion_phase_published_cmd_vel")
        if int(cmd_vel.get("nonzero_samples") or 0) != 0:
            blockers.append("no_motion_phase_consumed_nonzero_cmd_vel")
        max_static_map_error_m = thresholds.get("max_no_motion_slam_map_xy_error_m")
        if uses_slam_pose and max_static_map_error_m is not None:
            try:
                static_map_error_m = float(motion.get("slam_map_xy_error_m"))
            except (TypeError, ValueError):
                static_map_error_m = math.inf
            if static_map_error_m > float(max_static_map_error_m):
                blockers.append("no_motion_slam_map_pose_drift")
        max_static_odom_m = thresholds.get("max_no_motion_slam_odom_xy_m")
        if uses_slam_pose and max_static_odom_m is not None:
            try:
                static_odom_m = float(motion.get("slam_odom_xy_m"))
            except (TypeError, ValueError):
                static_odom_m = math.inf
            if static_odom_m > float(max_static_odom_m):
                blockers.append("no_motion_slam_odom_drift")
    elif phase == "motion":
        if (
            require_traversability
            and frame_contract is not None
            and not bool(frame_contract.get("ok"))
        ):
            blockers.append("traversability_frame_contract_mismatch")
        if not publish_cmd_vel:
            blockers.append("motion_phase_publish_cmd_vel_disabled")
        if evidence.max_cmd_vel_published <= 0:
            blockers.append("native_endpoint_did_not_publish_cmd_vel")
        if int(cmd_vel.get("nonzero_samples") or 0) < int(thresholds.get("min_cmd_vel_samples") or 3):
            blockers.append("mujoco_policy_received_too_few_nonzero_cmd_vel_samples")
        max_motion_map_error_m = thresholds.get("max_motion_slam_map_xy_error_m")
        if uses_slam_pose and max_motion_map_error_m is not None:
            try:
                motion_map_error_m = float(motion.get("slam_map_xy_error_m"))
            except (TypeError, ValueError):
                motion_map_error_m = math.inf
            if motion_map_error_m > float(max_motion_map_error_m):
                blockers.append("motion_slam_map_pose_error_too_large")

    goal_metrics = _goal_metrics(sensor_report, goal)
    forward_linear_samples = int(cmd_vel.get("forward_linear_samples") or 0)
    reverse_linear_samples = int(cmd_vel.get("reverse_linear_samples") or 0)
    lateral_linear_samples = int(cmd_vel.get("lateral_linear_samples") or 0)
    signed_linear_samples = forward_linear_samples + reverse_linear_samples
    forward_linear_fraction = (
        float(forward_linear_samples) / float(signed_linear_samples)
        if signed_linear_samples > 0
        else 0.0
    )
    reverse_linear_fraction = (
        float(reverse_linear_samples) / float(signed_linear_samples)
        if signed_linear_samples > 0
        else 0.0
    )
    goal_metrics["command_direction"] = {
        "forward_linear_samples": forward_linear_samples,
        "reverse_linear_samples": reverse_linear_samples,
        "lateral_linear_samples": lateral_linear_samples,
        "signed_linear_samples": signed_linear_samples,
        "forward_linear_fraction": forward_linear_fraction,
        "reverse_linear_fraction": reverse_linear_fraction,
    }
    native_goal_reached = (
        evidence.goal_reached_observed
        or bool(((evidence.last_nav or {}).get("last_local") or {}).get("goal_reached"))
        or bool(sensor_report.get("goal_reached_early"))
    )
    goal_metrics["native_goal_reached"] = native_goal_reached
    if phase == "motion":
        truth_velocity = sensor_report.get("mujoco_truth_velocity") or {}
        goal_metrics["mujoco_truth_peak_xy_speed_mps"] = truth_velocity.get(
            "max_xy_speed_mps"
        )
        truth_speed_blocker = _mujoco_truth_speed_blocker(sensor_report, thresholds)
        if truth_speed_blocker:
            blockers.append(truth_speed_blocker)
        min_forward_linear_fraction = thresholds.get(
            "min_forward_linear_command_fraction"
        )
        max_reverse_linear_fraction = thresholds.get(
            "max_reverse_linear_command_fraction"
        )
        if (
            min_forward_linear_fraction is not None
            or max_reverse_linear_fraction is not None
        ) and signed_linear_samples <= 0:
            blockers.append("signed_linear_command_evidence_missing")
        if (
            min_forward_linear_fraction is not None
            and forward_linear_fraction < float(min_forward_linear_fraction)
        ):
            blockers.append("forward_linear_command_fraction_below_threshold")
        if (
            max_reverse_linear_fraction is not None
            and reverse_linear_fraction > float(max_reverse_linear_fraction)
        ):
            blockers.append("reverse_linear_command_fraction_above_threshold")

        evidence_samples = max(
            0,
            int(evidence.motion_health_samples or evidence.samples),
        )
        goal_metrics["motion_health_samples"] = evidence_samples
        input_ready_fraction = (
            float(evidence.input_gate_ready_samples) / float(evidence_samples) if evidence_samples > 0 else 0.0
        )
        goal_metrics["input_gate_ready_fraction"] = input_ready_fraction
        min_input_ready_fraction = thresholds.get("min_input_gate_ready_fraction")
        if min_input_ready_fraction is not None and input_ready_fraction < float(min_input_ready_fraction):
            blockers.append("input_gate_ready_fraction_below_threshold")

        goal_metrics["odom_tf_rejections"] = evidence.max_odom_tf_rejections
        max_odom_tf_rejections = thresholds.get("max_odom_tf_rejections")
        if max_odom_tf_rejections is not None and evidence.max_odom_tf_rejections > int(max_odom_tf_rejections):
            blockers.append("odom_tf_rejections_above_threshold")

        goal_metrics["cloud_pose_rejections"] = evidence.max_cloud_pose_rejections
        max_cloud_pose_rejections = thresholds.get("max_cloud_pose_rejections")
        if max_cloud_pose_rejections is not None and evidence.max_cloud_pose_rejections > int(
            max_cloud_pose_rejections
        ):
            blockers.append("cloud_pose_rejections_above_threshold")

        goal_metrics["max_consecutive_input_stale_s"] = evidence.max_consecutive_input_stale_s
        max_consecutive_input_stale_s = thresholds.get("max_consecutive_input_stale_s")
        if max_consecutive_input_stale_s is not None and evidence.max_consecutive_input_stale_s > float(
            max_consecutive_input_stale_s
        ):
            blockers.append("consecutive_input_stale_above_threshold")

        loop_overrun_p95_ms = evidence.navigation_loop_overrun_percentile_ms(95.0)
        loop_overrun_p99_ms = evidence.navigation_loop_overrun_percentile_ms(99.0)
        goal_metrics["navigation_loop_overrun_p95_ms"] = loop_overrun_p95_ms
        goal_metrics["navigation_loop_overrun_p99_ms"] = loop_overrun_p99_ms
        goal_metrics["max_navigation_loop_overrun_ms"] = evidence.max_navigation_loop_overrun_ms
        goal_metrics["max_navigation_loop_overrun_context"] = dict(
            evidence.max_navigation_loop_overrun_context
        )
        max_navigation_loop_overrun_p99_ms = thresholds.get("max_navigation_loop_overrun_p99_ms")
        if max_navigation_loop_overrun_p99_ms is not None and loop_overrun_p99_ms > float(
            max_navigation_loop_overrun_p99_ms
        ):
            blockers.append("navigation_loop_overrun_p99_above_threshold")
        max_navigation_loop_overrun_peak_ms = thresholds.get("max_navigation_loop_overrun_peak_ms")
        if max_navigation_loop_overrun_peak_ms is not None and evidence.max_navigation_loop_overrun_ms > float(
            max_navigation_loop_overrun_peak_ms
        ):
            blockers.append("navigation_loop_overrun_peak_above_threshold")

        pacing = sensor_report.get("sim_hardware_pacing") or {}
        max_host_lag_s = thresholds.get("max_sim_hardware_lag_observed_s")
        if max_host_lag_s is not None and float(pacing.get("max_lag_observed_s") or 0.0) > float(max_host_lag_s):
            blockers.append("sim_hardware_lag_above_threshold")
        max_catch_up_steps = thresholds.get("max_sim_hardware_consecutive_catch_up_steps")
        if max_catch_up_steps is not None and int(pacing.get("max_consecutive_steps") or 0) > int(max_catch_up_steps):
            blockers.append("sim_hardware_catch_up_streak_above_threshold")

        if not goal_metrics.get("available"):
            blockers.append("mujoco_motion_metrics_missing")
        else:
            if float(goal_metrics.get("sim_path_length_xy_m") or 0.0) < float(thresholds.get("min_motion_m") or 0.15):
                blockers.append("thunderv4_motion_below_threshold")
            min_net_displacement_m = thresholds.get("min_net_displacement_m")
            if (
                min_net_displacement_m is not None
                and float(goal_metrics.get("net_displacement_xy_m") or 0.0)
                < float(min_net_displacement_m)
            ):
                blockers.append("net_displacement_below_threshold")
            min_goal_distance_reduction_m = thresholds.get(
                "min_goal_distance_reduction_m"
            )
            if (
                min_goal_distance_reduction_m is not None
                and float(goal_metrics.get("distance_reduction_m") or 0.0)
                < float(min_goal_distance_reduction_m)
            ):
                blockers.append("goal_distance_did_not_decrease")
            max_goal_error_m = thresholds.get("max_goal_error_m")
            if max_goal_error_m is not None and float(goal_metrics.get("end_distance_m")) > float(max_goal_error_m):
                blockers.append("final_goal_error_above_threshold")
            max_goal_error_xy_m = thresholds.get("max_goal_error_xy_m")
            if (
                max_goal_error_xy_m is not None
                and float(goal_metrics.get("end_error_xy_m", math.inf))
                > float(max_goal_error_xy_m)
            ):
                blockers.append("final_goal_xy_error_above_threshold")
            max_goal_error_z_m = thresholds.get("max_goal_error_z_m")
            if (
                max_goal_error_z_m is not None
                and float(goal_metrics.get("end_error_z_m", math.inf))
                > float(max_goal_error_z_m)
            ):
                blockers.append("final_goal_z_error_above_threshold")
            max_goal_error_3d_m = thresholds.get("max_goal_error_3d_m")
            if (
                max_goal_error_3d_m is not None
                and float(goal_metrics.get("end_error_3d_m", math.inf))
                > float(max_goal_error_3d_m)
            ):
                blockers.append("final_goal_3d_error_above_threshold")
        if bool(thresholds.get("require_goal_reached")) and not native_goal_reached:
            blockers.append("native_goal_not_reached")
    return not blockers, blockers, goal_metrics


def _video_artifact_blocker(
    *,
    required: bool,
    video_report: dict[str, Any],
    require_candidates: bool = True,
) -> str | None:
    """Return the video gate blocker, including presentation evidence gaps."""

    if not required:
        return None
    if (
        require_candidates
        and "candidate_frames" in video_report
        and int(video_report.get("candidate_frames") or 0) <= 0
    ):
        return "native_navigation_video_candidates_missing"
    if (
        require_candidates
        and "selected_candidate_frames" in video_report
        and int(video_report.get("selected_candidate_frames") or 0) <= 0
    ):
        return "native_navigation_video_selected_path_missing"
    if "local_map_frames" in video_report and int(video_report.get("local_map_frames") or 0) <= 0:
        return "native_navigation_video_local_map_missing"
    if (
        "visible_local_map_frames" in video_report
        and int(video_report.get("visible_local_map_frames") or 0) <= 0
    ):
        return "native_navigation_video_local_map_not_visible"
    if (
        "exact_planner_join_frames" in video_report
        and int(video_report.get("exact_planner_join_frames") or 0) <= 0
    ):
        return "native_navigation_video_exact_join_missing"
    if video_report.get("ok") is True:
        return None
    return "native_navigation_video_failed"


def _run_phase(
    *,
    phase: str,
    manifest: dict[str, Any],
    binaries: dict[str, Path],
    paths: dict[str, Path],
    out_dir: Path,
    domain_id: int,
    host_boot_id: str,
    native_environment: Mapping[str, str],
) -> dict[str, Any]:
    phase_cfg = dict((manifest.get("phases") or {}).get(phase) or {})
    thresholds = dict(manifest.get("thresholds") or {})
    runtime_tolerances = dict(manifest.get("runtime_tolerances") or {})
    navigation_runtime_cfg = dict(manifest.get("navigation_runtime") or {})
    require_traversability = bool(thresholds.get("require_traversability", True))
    use_traversability_cost = navigation_runtime_cfg.get(
        "use_traversability_cost", require_traversability
    )
    if not isinstance(use_traversability_cost, bool):
        raise ValueError("navigation_runtime.use_traversability_cost must be boolean")
    if use_traversability_cost and not require_traversability:
        raise ValueError(
            "navigation_runtime.use_traversability_cost requires thresholds.require_traversability"
        )
    start = [float(value) for value in manifest.get("start") or [0.0, 0.0, 0.48, 0.0]]
    goal = [float(value) for value in manifest.get("goal") or [1.0, 0.0, 0.3, 0.0]]
    world_value = str(manifest.get("world") or "building")
    world_candidate = _repo_path(world_value)
    world_arg = str(world_candidate) if world_candidate.is_file() else world_value
    phase_dir = out_dir / phase
    phase_dir.mkdir(parents=True, exist_ok=True)
    slam_status = phase_dir / "slam_status.json"
    slam_cloud_dir = phase_dir / "slam_clouds"
    slam_cloud_dir.mkdir(parents=True, exist_ok=True)
    traversability_status = phase_dir / "traversability_status.json"
    mapd_status = phase_dir / "mapd_status.json"
    nav_status = phase_dir / "nav_status.json"
    sensor_report_path = phase_dir / "sensor_report.json"
    parent_sensor_diagnostics_path = phase_dir / "parent_sensor_diagnostics.json"
    motion_log_path = phase_dir / "motion.jsonl"
    motion_complete_marker = phase_dir / "motion_complete.json"
    sensor_publisher_pid = phase_dir / "sensor_publisher.pid"
    driver_bridge_pid = phase_dir / "driver_bridge.pid"
    for path in (
        slam_status,
        traversability_status,
        mapd_status,
        nav_status,
        sensor_report_path,
        parent_sensor_diagnostics_path,
        motion_log_path,
        motion_complete_marker,
    ):
        path.unlink(missing_ok=True)

    video_cfg = dict(manifest.get("acceptance_video") or {})
    record_video = phase == "motion" and bool(video_cfg.get("enabled"))
    telemetry_cfg = dict(manifest.get("telemetry_log") or {})
    record_telemetry = phase == "motion" and bool(telemetry_cfg.get("enabled"))

    slam_runtime_cfg = dict(manifest.get("slam_runtime") or {})
    state_provider = str(slam_runtime_cfg.get("provider") or "fastlio2").strip().lower()
    if state_provider not in {"fastlio2", "mujoco_navigation_fixture"}:
        raise ValueError(f"unsupported navigation state provider: {state_provider}")
    use_slam_process = state_provider == "fastlio2"
    thresholds.setdefault("navigation_state_provider", state_provider)
    slam_mode = str(slam_runtime_cfg.get("mode") or "localization").strip().lower()
    if use_slam_process and slam_mode not in {"mapping", "localization"}:
        raise ValueError(f"unsupported slam_runtime.mode: {slam_mode}")
    slam_args: list[str] = []
    if use_slam_process:
        slam_args = [
            "--backend",
            "fastlio2",
            "--mode",
            slam_mode,
            "--config",
            _native_path_arg(binaries["slam"], paths["slam_config"]),
            "--domain-id",
            str(domain_id),
            "--tick-hz",
            "50",
            "--status-json",
            _native_path_arg(binaries["slam"], slam_status),
            "--status-json-hz",
            str(float(slam_runtime_cfg.get("status_json_hz") or 10.0)),
            "--cloud-snapshot-dir",
            _native_path_arg(binaries["slam"], slam_cloud_dir),
            "--cloud-snapshot-hz",
            str(float(slam_runtime_cfg.get("cloud_snapshot_hz") or 5.0)),
        ]
    if use_slam_process and slam_mode == "localization":
        slam_args.extend(
            [
                "--map",
                _native_path_arg(binaries["slam"], paths["slam"]),
                "--track-against-map-period-s",
                str(float(runtime_tolerances.get("track_against_map_period_s") or 1.0)),
                "--track-against-map-initial-pose",
                *(str(value) for value in start),
            ]
        )

    navigation_affinity_mask, support_affinity_mask = (
        _windows_acceptance_affinity_masks(manifest)
    )
    slam = (
        ManagedProcess(
            "slam",
            _native_command(binaries["slam"], *slam_args),
            phase_dir / "slam.log",
            affinity_mask=support_affinity_mask,
        )
        if use_slam_process
        else None
    )
    lidar_offset_body = _compiled_mujoco_site_offset_body(DEFAULT_THUNDERV4_MJCF)
    lidar_offset_args = _sensor_offset_args(lidar_offset_body)
    traversability_runtime_cfg = dict(manifest.get("traversability_runtime") or {})
    local_planner_backend = _local_planner_backend(manifest)
    phase_identity = _native_map_identity(
        paths=paths,
        phase=phase,
        domain_id=domain_id,
        session_root=phase_dir,
    )
    native_map_env = dict(native_environment)
    for key, value in phase_identity.items():
        native_map_env.setdefault(key, value)
    native_map_env.update(
        {
            "LINGTU_ENV": "sim",
            "LINGTU_PRODUCT": "nav",
        }
    )
    native_map_env.update(mapd_environment(local_planner_backend))
    mapd_command, mapd_env = _native_mapd_launch(
        binary=binaries["mapd"],
        domain_id=domain_id,
        status_file=mapd_status,
        map_root=paths["map_dir"].parent,
        runtime=dict(manifest.get("mapd_runtime") or {}),
        environment=native_map_env,
    )
    mapd = ManagedProcess(
        "mapd",
        mapd_command,
        phase_dir / "mapd.log",
        env=mapd_env,
        affinity_mask=support_affinity_mask,
    )
    traversability = None
    if require_traversability:
        traversability_command, traversability_env = _with_native_env(
            _native_command(
                binaries["traversability"],
                "--domain-id",
                str(domain_id),
                "--publish-hz",
                str(float(traversability_runtime_cfg.get("publish_hz") or 10.0)),
                "--slow-hz",
                str(float(traversability_runtime_cfg.get("slow_hz") or 5.0)),
                "--tick-hz",
                "20",
                "--resolution",
                "0.2",
                "--radius",
                "6",
                "--max-points",
                "5000",
                *lidar_offset_args,
                "--status-file",
                _native_path_arg(binaries["traversability"], traversability_status),
            ),
            **native_map_env,
        )
        traversability = ManagedProcess(
            "traversability",
            traversability_command,
            phase_dir / "traversability.log",
            env=traversability_env,
            affinity_mask=support_affinity_mask,
        )
    navigation_environment = dict(native_environment)
    navigation_environment.update(native_map_env)
    navigation_environment.update(
        _navigation_smoother_environment(navigation_runtime_cfg)
    )
    navigation_environment.update(
        _scan_follower_environment(navigation_runtime_cfg)
    )
    local_planner_debug_candidates = int(
        navigation_runtime_cfg.get("debug_candidate_limit", 36 if record_video else 0)
    )
    local_map_debug_points = int(
        navigation_runtime_cfg.get("debug_local_map_points", 320 if record_video else 0)
    )
    status_period_s = float(navigation_runtime_cfg.get("status_period_s") or 0.1)
    if record_video and (local_planner_debug_candidates > 0 or local_map_debug_points > 0):
        status_period_s = min(status_period_s, 0.2)
    navigation_command = _native_command(
        binaries["navigation"],
        "--control-mode",
        "autonomy",
        "--local-planner",
        local_planner_backend,
        *_path_follower_args(navigation_runtime_cfg),
        "--domain-id",
        str(domain_id),
        *_path_library_args(
            local_planner_backend,
            binaries["navigation"],
            paths,
        ),
        "--map",
        _native_path_arg(binaries["navigation"], paths["planner"]),
        *_planner_constraint_args(manifest),
        "--tick-hz",
        "20",
        "--control-loop-deadline-miss-ratio-limit",
        str(
            float(
                navigation_runtime_cfg.get(
                    "control_loop_deadline_miss_ratio_limit", 0.05
                )
            )
        ),
        "--control-loop-p95-utilization-limit",
        str(
            float(
                navigation_runtime_cfg.get(
                    "control_loop_p95_utilization_limit", 0.90
                )
            )
        ),
        "--corridor-lookahead-m",
        str(float(navigation_runtime_cfg.get("corridor_lookahead_m", 3.0))),
        "--max-obstacle-points",
        str(int(navigation_runtime_cfg.get("max_obstacle_points") or 5000)),
        "--local-planner-obstacle-height-max-m",
        str(float(navigation_runtime_cfg.get("obstacle_height_max_m", 1.2))),
        *lidar_offset_args,
        "--local-planner-debug-candidates",
        str(local_planner_debug_candidates),
        "--local-map-debug-points",
        str(local_map_debug_points),
        "--odom-max-age-s",
        str(float(runtime_tolerances.get("odom_max_age_s") or 0.6)),
        "--tf-max-age-s",
        str(float(runtime_tolerances.get("tf_max_age_s") or 0.6)),
        "--cloud-max-age-s",
        str(float(runtime_tolerances.get("cloud_max_age_s") or 0.6)),
        "--traversability-max-age-s",
        str(float(runtime_tolerances.get("traversability_max_age_s") or 1.5)),
        "--localization-health-max-age-s",
        str(float(runtime_tolerances.get("localization_health_max_age_s") or 0.5)),
        "--terrain-map-max-age-s",
        str(float(runtime_tolerances.get("terrain_map_max_age_s") or 0.5)),
        "--cloud-pose-max-gap-s",
        str(float(runtime_tolerances.get("cloud_pose_max_gap_s") or 0.2)),
        "--input-future-tolerance-s",
        str(float(runtime_tolerances.get("input_future_tolerance_s") or 0.05)),
        "--input-recovery-frames",
        str(int(runtime_tolerances.get("input_recovery_frames") or 1)),
        "--publish-cmd-vel",
        "true" if bool(phase_cfg.get("publish_cmd_vel")) else "false",
        "--check-obstacle",
        "true",
        "--use-traversability-cost",
        "true" if use_traversability_cost else "false",
        "--status-file",
        _native_path_arg(binaries["navigation"], nav_status),
        "--status-s",
        str(status_period_s),
    )
    driver_runtime = _native_driver_runtime_launch(
        manifest=manifest,
        navigation_binary=binaries["navigation"],
        driver_bridge_binary=binaries["driver_bridge"],
        navigation_command=navigation_command,
        driver_bridge_pid=driver_bridge_pid,
        host_boot_id=host_boot_id,
        navigation_environment=navigation_environment,
    )
    navigation = ManagedProcess(
        "navigation",
        list(driver_runtime.navigation_command),
        phase_dir / "navigation.log",
        env=driver_runtime.navigation_env,
        affinity_mask=navigation_affinity_mask,
    )
    motion_arm_enabled = bool(phase_cfg.get("publish_cmd_vel"))
    motion_arm_file = phase_dir / "motion_arm.json"
    motion_arm_status = phase_dir / "motion_arm_status.json"
    motion_arm_token = secrets.token_hex(16)
    motion_arm_scenario = f"{str(manifest.get('name') or 'navigation')}:{phase}:{domain_id}"
    motion_arm_timeout_s = max(
        30.0,
        float(thresholds.get("startup_timeout_s") or 18.0) + 30.0,
    )
    motion_arm_file.unlink(missing_ok=True)
    motion_arm_status.unlink(missing_ok=True)
    sensor_args = [
        sys.executable,
        str(paths["sensor_runner"]),
        "--world",
        world_arg,
        *_sensor_start_args(start),
        "--start-anchor",
        str(phase_cfg.get("start_anchor") or "off"),
        "--duration",
        str(float(phase_cfg.get("duration_s") or 12.0)),
        "--settle-s",
        "1.0",
        "--warmup-s",
        str(float(phase_cfg.get("warmup_s") or 4.0)),
        "--drive-ramp-s",
        "0",
        "--drive-mode",
        "policy",
        "--policy-path",
        str(paths["policy"]),
        "--command-source",
        "dds",
        *driver_runtime.driver_bridge_args,
        "--sim-hardware-realtime-factor",
        str(float(runtime_tolerances.get("sim_hardware_realtime_factor") or 0.75)),
        "--publisher-bin",
        str(binaries["sensor_publisher"]),
        "--publisher-pid-file",
        str(sensor_publisher_pid),
        "--domain-id",
        str(domain_id),
        "--nav-status-json", str(nav_status),
        "--motion-complete-marker",
        str(motion_complete_marker),
        "--json-out",
        str(sensor_report_path),
        *_parent_sensor_diagnostics_args(
            parent_sensor_diagnostics_path,
            runtime_tolerances,
        ),
    ]
    if motion_arm_enabled:
        sensor_args.extend(
            [
                "--external-arm-file",
                str(motion_arm_file),
                "--external-arm-token",
                motion_arm_token,
                "--external-arm-scenario",
                motion_arm_scenario,
                "--external-arm-timeout-s",
                f"{motion_arm_timeout_s:g}",
                "--external-arm-status-json",
                str(motion_arm_status),
            ]
        )
    if use_slam_process:
        sensor_args.extend(
            [
                "--slam-status-json",
                str(slam_status),
                "--require-slam-output",
            ]
        )
    else:
        sensor_args.append("--navigation-fixture")
    sensor_args.extend(_sensor_runtime_args(manifest))
    sensor_overrides = dict(manifest.get("diagnostic_sensor_overrides") or {})
    if sensor_overrides.get("imu_acc_mode"):
        sensor_args.extend(["--imu-acc-mode", str(sensor_overrides["imu_acc_mode"])])
    for key, option in (
        ("imu_acc_lowpass_hz", "--imu-acc-lowpass-hz"),
        ("imu_acc_max_dynamic_mps2", "--imu-acc-max-dynamic-mps2"),
        ("imu_acc_max_slew_mps3", "--imu-acc-max-slew-mps3"),
    ):
        if sensor_overrides.get(key) is not None:
            sensor_args.extend([option, str(sensor_overrides[key])])
    if sensor_overrides.get("scan_time_profile"):
        sensor_args.extend(["--scan-time-profile", str(sensor_overrides["scan_time_profile"])])
    if bool(phase_cfg.get("require_nonzero_cmd_vel")):
        sensor_args.append("--require-cmd-vel")
    motion_capture = _motion_capture_settings(
        record_video=record_video,
        record_telemetry=record_telemetry,
        video_cfg=video_cfg,
        telemetry_cfg=telemetry_cfg,
    )
    if motion_capture is not None:
        motion_log_hz, motion_log_lidar_points = motion_capture
        sensor_args.extend(
            [
                "--motion-log",
                str(motion_log_path),
                "--motion-log-hz",
                str(motion_log_hz),
                "--motion-log-lidar-points",
                str(motion_log_lidar_points),
            ]
        )
    sensor = ManagedProcess(
        "sensor",
        sensor_args,
        phase_dir / "sensor.log",
        affinity_mask=support_affinity_mask,
    )
    goal_trigger = phase_dir / "goal_control.trigger"
    goal_trigger.unlink(missing_ok=True)
    deferred_goal = (
        ManagedProcess(
            "goal_control",
            _deferred_goal_command(
                binaries["navigation_control"],
                goal,
                domain_id,
                timeout_s=float(thresholds.get("plan_timeout_s") or 12.0),
                trigger_path=goal_trigger,
            ),
            phase_dir / "goal_control.log",
            affinity_mask=support_affinity_mask,
        )
        if os.name == "nt" and binaries["navigation_control"].suffix.lower() != ".exe"
        else None
    )
    processes = [
        process
        for process in (slam, mapd, traversability, navigation, deferred_goal, sensor)
        if process is not None
    ]
    evidence = NativeEvidence()
    startup_ok = False
    startup_reason = "not_started"
    motion_arm_result: dict[str, Any] = {}
    resume_result: dict[str, Any] = {}
    goal_result: dict[str, Any] = {}
    phase_error = ""
    process_cleanup: list[dict[str, Any]] = []
    started_commands: list[dict[str, Any]] = []

    try:
        for process in processes:
            process.start()
            started_commands.append(
                {
                    "name": process.name,
                    "command": process.command,
                    "affinity_mask": process.affinity_mask,
                }
            )
        startup_ok, startup_reason = _wait_for_startup(
            sensor=sensor,
            evidence=evidence,
            nav_status=nav_status,
            slam_status=slam_status,
            traversability_status=traversability_status,
            mapd_status=mapd_status,
            timeout_s=float(thresholds.get("startup_timeout_s") or 18.0),
            thresholds=thresholds,
        )
        if startup_ok:
            plan_timeout_s = float(thresholds.get("plan_timeout_s") or 12.0)
            if motion_arm_enabled:
                motion_arm_result = _arm_mujoco_motion(
                    arm_file=motion_arm_file,
                    status_file=motion_arm_status,
                    token=motion_arm_token,
                    domain_id=domain_id,
                    scenario=motion_arm_scenario,
                    sensor=sensor,
                    timeout_s=min(10.0, plan_timeout_s),
                )
            if deferred_goal is not None:
                resume_result = _resume_when_ready(
                    binaries["navigation_control"],
                    domain_id,
                    timeout_s=plan_timeout_s,
                )
            if int(resume_result.get("returncode") or 0) == 0 and deferred_goal is not None:
                goal_trigger.write_text("go\n", encoding="ascii")
                goal_returncode = deferred_goal.wait(timeout_s=plan_timeout_s + 5.0)
                goal_result = {
                    "command": deferred_goal.command,
                    "returncode": goal_returncode,
                    "stdout": deferred_goal.tail(),
                    "stderr": "",
                    "wsl_relay_prestarted": True,
                    "trigger": str(goal_trigger),
                }
            elif deferred_goal is None:
                control_attempts: list[dict[str, Any]] = []
                for attempt in range(1, 5):
                    resume_result = _resume_when_ready(
                        binaries["navigation_control"],
                        domain_id,
                        timeout_s=plan_timeout_s,
                    )
                    if int(resume_result.get("returncode") or 0) != 0:
                        break
                    goal_result = _goal_command(
                        binaries["navigation_control"],
                        goal,
                        domain_id,
                        timeout_s=plan_timeout_s,
                    )
                    control_attempts.append(
                        {
                            "attempt": attempt,
                            "resume": dict(resume_result),
                            "goal": dict(goal_result),
                        }
                    )
                    if int(goal_result.get("returncode") or 0) == 0:
                        break
                    if "operator_takeover_resume_required" not in str(
                        goal_result.get("stderr") or ""
                    ):
                        break
                    time.sleep(0.05)
                if goal_result:
                    goal_result["control_attempts"] = control_attempts
        phase_duration_s = float(phase_cfg.get("duration_s") or 12.0)
        deadline = time.monotonic() + _phase_runtime_timeout_s(
            phase_duration_s,
            float(thresholds.get("runner_shutdown_grace_s") or 120.0),
            realtime_factor=float(
                runtime_tolerances.get("sim_hardware_realtime_factor") or 1.0
            ),
        )
        while sensor.poll() is None and time.monotonic() < deadline:
            evidence.sample(
                nav_path=nav_status,
                slam_path=slam_status,
                traversability_path=traversability_status,
                collect_motion_health=_motion_health_collection_active(
                    motion_complete_marker
                ),
            )
            time.sleep(RUNTIME_EVIDENCE_SAMPLE_PERIOD_S)
        if sensor.poll() is None:
            raise TimeoutError("MuJoCo sensor/locomotion runner exceeded phase deadline")
        evidence.sample(
            nav_path=nav_status,
            slam_path=slam_status,
            traversability_path=traversability_status,
            collect_motion_health=False,
        )
    except Exception as exc:
        phase_error = f"{type(exc).__name__}: {exc}"
    finally:
        for process in reversed(processes):
            process.stop()
        process_cleanup.extend(process.cleanup for process in processes)
        process_cleanup.append(_cleanup_pid_file("sensor_publisher", sensor_publisher_pid))
        process_cleanup.append(_cleanup_pid_file("driver_bridge", driver_bridge_pid))

    sensor_report = _load_json(sensor_report_path)
    parent_sensor_diagnostics = _load_json(parent_sensor_diagnostics_path)
    sensor_acceptance = _navigation_sensor_assessment(sensor_report, thresholds)
    terminal_driver_stop = _terminal_driver_stop_evidence(
        sensor_report=sensor_report,
        expected_host_boot_id=driver_runtime.host_boot_id,
        process_cleanup=process_cleanup,
        require_prior_output_ack=bool(phase_cfg.get("publish_cmd_vel")),
    )
    video_report: dict[str, Any] = {
        "requested": record_video,
        "ok": not record_video,
        "reason": "not_requested" if not record_video else "not_rendered",
    }
    if record_video:
        try:
            from sim.scripts.mujoco.native_navigation_video import (
                render_native_navigation_video,
            )

            video_report = render_native_navigation_video(
                world=world_arg,
                policy_path=paths["policy"],
                motion_log=motion_log_path,
                output=phase_dir / "native_navigation.mp4",
                goal=goal,
                width=int(video_cfg.get("width") or 1920),
                height=int(video_cfg.get("height") or 1080),
                fps=float(video_cfg.get("fps") or 24.0),
                require_candidate_evidence=local_planner_backend == "cmu",
            )
            video_report["requested"] = True
        except Exception as exc:
            video_report = {
                "requested": True,
                "ok": False,
                "reason": f"{type(exc).__name__}:{exc}",
                "motion_log": str(motion_log_path),
            }
    traversability_frame_contract = (
        _traversability_frame_contract(
            evidence,
            dict(manifest.get("frame_contract") or {}),
        )
        if require_traversability
        else None
    )
    ok, blockers, goal_metrics = _evaluate_phase(
        phase=phase,
        phase_cfg=phase_cfg,
        thresholds=thresholds,
        evidence=evidence,
        sensor_report=sensor_report,
        goal=goal,
        frame_contract=traversability_frame_contract,
    )
    tracking_metrics = _trajectory_tracking_metrics(motion_log_path)
    goal_metrics["trajectory_tracking"] = tracking_metrics
    min_tracking_samples = thresholds.get("min_tracking_samples")
    if (
        min_tracking_samples is not None
        and int(tracking_metrics.get("samples") or 0) < int(min_tracking_samples)
    ):
        blockers.append("trajectory_tracking_samples_below_threshold")
    max_tracking_p95_m = thresholds.get("max_tracking_xy_error_p95_m")
    if (
        max_tracking_p95_m is not None
        and float(tracking_metrics.get("p95_xy_error_m", math.inf))
        > float(max_tracking_p95_m)
    ):
        blockers.append("trajectory_tracking_p95_above_threshold")
    if not startup_ok:
        blockers.append(startup_reason)
    if motion_arm_enabled and str(motion_arm_result.get("state") or "") != "armed":
        blockers.append("mujoco_motion_arm_not_acknowledged")
    if int(resume_result.get("returncode") or 0) != 0 or not resume_result:
        blockers.append("native_resume_command_failed")
    if int(goal_result.get("returncode") or 0) != 0 or not goal_result:
        blockers.append("native_goal_command_failed")
    if phase_error:
        blockers.append("phase_runtime_error")
    if sensor_acceptance.get("navigation_critical_ok") is not True:
        blockers.append("sensor_or_slam_acceptance_failed")
    if not all(bool(item.get("clean")) for item in process_cleanup):
        blockers.append("acceptance_process_cleanup_failed")
    blockers.extend(terminal_driver_stop["blockers"])
    video_blocker = _video_artifact_blocker(
        required=bool(thresholds.get("require_video_artifact", True)),
        video_report=video_report,
        require_candidates=local_planner_backend == "cmu",
    )
    if video_blocker:
        blockers.append(video_blocker)
    blockers = list(dict.fromkeys(blockers))
    report = {
        "schema_version": "lingtu.mujoco.native_navigation.phase.v1",
        "phase": phase,
        "ok": ok and not blockers,
        "domain_id": domain_id,
        "uses_ros": False,
        "python_planner_used": False,
        "python_role": "mujoco_physics_sensor_bridge_process_supervisor_acceptance_only",
        "navigation_compute_owner": "navd",
        "navigation_state_provider": state_provider,
        "local_planner_backend": local_planner_backend,
        "driver_bridge": {
            "binary": str(binaries["driver_bridge"]),
            "host_boot_id": driver_runtime.host_boot_id,
            "native_clock_platform": driver_runtime.clock_platform,
            "contract": dict(manifest["driver_runtime"]),
            "ack_source": "physical_mujoco_step",
        },
        "startup": {"ok": startup_ok, "reason": startup_reason},
        "motion_arm": motion_arm_result,
        "resume_command": resume_result,
        "goal_command": goal_result,
        "goal_metrics": goal_metrics,
        "pre_safety_path_follower_command": _pre_safety_command_evidence(evidence),
        "post_safety_command": {
            "dds_topic": "rt/nav/cmd_vel",
            "published_samples": evidence.max_cmd_vel_published,
            "physically_applied_nonzero_samples": int(
                (sensor_report.get("cmd_vel") or {}).get("nonzero_samples") or 0
            ),
        },
        "terminal_driver_stop": terminal_driver_stop,
        "traversability_frame_contract": traversability_frame_contract,
        "process_cleanup": {
            "zero_leftovers": all(bool(item.get("clean")) for item in process_cleanup),
            "processes": process_cleanup,
        },
        "evidence": evidence.to_dict(),
        "mapd_status": _load_json(mapd_status),
        "sensor_report": sensor_report,
        "parent_sensor_diagnostics": {
            "path": str(parent_sensor_diagnostics_path),
            "snapshot": parent_sensor_diagnostics,
        },
        "sensor_acceptance": sensor_acceptance,
        "acceptance_scope": manifest.get("acceptance_scope") or {},
        "video": video_report,
        "blockers": blockers,
        "error": phase_error,
        "processes": started_commands,
        "logs": {process.name: str(process.log_path) for process in processes},
        "log_tails": {process.name: process.tail() for process in processes},
    }
    _write_json(phase_dir / "report.json", report)
    return report


def _terminal_driver_stop_evidence(
    *,
    sensor_report: Mapping[str, Any],
    expected_host_boot_id: str,
    process_cleanup: Sequence[Mapping[str, Any]],
    require_prior_output_ack: bool = True,
) -> dict[str, Any]:
    """Evaluate the strongest terminal stop proof emitted by the native bridge."""

    driver_value = sensor_report.get("cmd_vel")
    driver = driver_value if isinstance(driver_value, Mapping) else {}
    stop_value = driver.get("stopped_evidence")
    stopped = stop_value if isinstance(stop_value, Mapping) else {}
    ack_value = driver.get("observed_output_ack")
    output_ack = ack_value if isinstance(ack_value, Mapping) else {}
    command_value = driver.get("last_command")
    last_command = command_value if isinstance(command_value, Mapping) else {}
    driver_cleanup_value = driver.get("process_cleanup")
    driver_cleanup = (
        driver_cleanup_value if isinstance(driver_cleanup_value, Mapping) else {}
    )
    publisher_value = sensor_report.get("native_sensor_publisher_process")
    publisher_cleanup = publisher_value if isinstance(publisher_value, Mapping) else {}

    bridge_boot_id = str(driver.get("bridge_boot_id") or "")
    controller_boot_id = str(driver.get("controller_boot_id") or "")
    stopped_bridge_boot_id = str(stopped.get("bridge_boot_id") or "")
    stopped_controller_boot_id = str(stopped.get("controller_boot_id") or "")

    def positive_int(value: Any) -> int | None:
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            return None
        return value

    stop_command_sequence = positive_int(stopped.get("bridge_command_seq"))
    applied_step_sequence = positive_int(stopped.get("applied_step_seq"))
    observed_ack_sequence = positive_int(output_ack.get("accepted_sequence"))
    observed_output_sequence = positive_int(output_ack.get("output_sequence"))
    observed_producer = str(output_ack.get("producer_boot_id") or "")
    identity_matches = bool(
        bridge_boot_id
        and controller_boot_id
        and stopped_bridge_boot_id == bridge_boot_id
        and stopped_controller_boot_id == controller_boot_id
    )
    stop_ack_accepted = bool(
        identity_matches
        and stop_command_sequence is not None
        and applied_step_sequence is not None
        and str(stopped.get("kind") or "") == "deactivate_zero"
    )
    output_ack_exact = bool(
        observed_ack_sequence is not None
        and observed_output_sequence is not None
        and _driver_producer_matches_host(observed_producer, expected_host_boot_id)
    )

    zero_values: dict[str, float] = {}
    exact_zero = True
    for component in ("vx", "vy", "wz"):
        value = last_command.get(component)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            exact_zero = False
            zero_values[component] = math.nan
            continue
        parsed = float(value)
        zero_values[component] = parsed
        if not math.isfinite(parsed) or parsed != 0.0:
            exact_zero = False

    authority_cleared = bool(
        driver.get("driver_ready") is False
        and driver.get("accepted_sequence") == 0
        and str(driver.get("accepted_producer_boot_id") or "") == ""
        and driver.get("accepted_output_sequence") == 0
    )
    cleanup_entries = [item for item in process_cleanup if isinstance(item, Mapping)]
    owned_processes_stopped = bool(
        cleanup_entries
        and len(cleanup_entries) == len(process_cleanup)
        and all(item.get("clean") is True for item in cleanup_entries)
        and driver.get("process_returncode") == 0
        and driver_cleanup.get("clean") is True
        and publisher_cleanup.get("clean") is True
    )

    blockers: list[str] = []
    if not identity_matches:
        blockers.append("driver_terminal_stop_ack_identity_mismatch")
    if not stop_ack_accepted:
        blockers.append("driver_terminal_stop_ack_missing_or_invalid_sequence")
    if require_prior_output_ack and not output_ack_exact:
        blockers.append("driver_terminal_prior_output_ack_identity_or_sequence_invalid")
    if not exact_zero:
        blockers.append("driver_terminal_logical_cmd_vel_not_exact_zero")
    if not (stop_ack_accepted and exact_zero):
        blockers.append("driver_terminal_physical_stop_not_exact_zero")
    if not authority_cleared:
        blockers.append("driver_terminal_authority_not_cleared")
    if not owned_processes_stopped:
        blockers.append("driver_terminal_owned_process_cleanup_failed")
    blockers = list(dict.fromkeys(blockers))
    return {
        "ok": not blockers,
        "stop_ack": {
            "accepted": stop_ack_accepted,
            "bridge_boot_id": stopped_bridge_boot_id,
            "controller_boot_id": stopped_controller_boot_id,
            "bridge_command_seq": stopped.get("bridge_command_seq"),
            "applied_step_seq": stopped.get("applied_step_seq"),
            "kind": str(stopped.get("kind") or ""),
        },
        "prior_output_ack": {
            "required": require_prior_output_ack,
            "exact_identity_and_sequence": output_ack_exact,
            "accepted_sequence": output_ack.get("accepted_sequence"),
            "producer_boot_id": observed_producer,
            "output_sequence": output_ack.get("output_sequence"),
        },
        "logical_final_cmd_vel": {"exact_zero": exact_zero, **zero_values},
        "physical_stop": {
            "exact_zero": stop_ack_accepted and exact_zero,
            "proof": "deactivate_zero was physically applied before the exact STOPPED ACK",
        },
        "authority_cleared": authority_cleared,
        "owned_processes_stopped": owned_processes_stopped,
        "blockers": blockers,
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    manifest_path = Path(args.manifest).expanduser().resolve()
    manifest = _load_manifest(manifest_path)
    validated_run_plan = getattr(args, "validated_run_plan", None)
    native_environment = (
        validated_run_plan.native_process_environment
        if validated_run_plan is not None
        else {}
    )
    run_plan_binary_bindings = (
        _bind_manifest_binaries_to_run_plan(
            manifest,
            validated_run_plan,
        )
        if validated_run_plan is not None
        else {}
    )
    run_plan_verified = bool(getattr(args, "run_plan_verified", False))
    if not manifest:
        report = {
            "schema_version": "lingtu.mujoco.native_navigation.acceptance.v1",
            "ok": False,
            "blockers": [f"manifest_unreadable:{manifest_path}"],
        }
        report.update(
            classify_evidence(
                None,
                run_plan_verified=run_plan_verified,
                acceptance_evaluated=False,
                ok=False,
            )
        )
        return report
    world_override = str(getattr(args, "world", "") or "").strip()
    map_dir_override = str(getattr(args, "map_dir", "") or "").strip()
    if world_override:
        manifest["world"] = str(Path(world_override).expanduser().resolve())
    if map_dir_override:
        manifest["map_dir"] = str(Path(map_dir_override).expanduser().resolve())
    phase_duration_s = getattr(args, "phase_duration_s", None)
    if phase_duration_s is not None:
        duration_s = float(phase_duration_s)
        if duration_s <= 0.0:
            raise ValueError("--phase-duration-s must be positive")
        requested_phases = (
            ("no_motion", "motion")
            if str(args.mode) == "both"
            else (str(args.mode),)
        )
        phases = manifest.setdefault("phases", {})
        for phase_name in requested_phases:
            phases.setdefault(phase_name, {})["duration_s"] = duration_s
    diagnostic_sensor_overrides: dict[str, str] = {}
    imu_acc_mode = str(getattr(args, "diagnostic_imu_acc_mode", "") or "").strip()
    scan_time_profile = str(getattr(args, "diagnostic_scan_time_profile", "") or "").strip()
    if imu_acc_mode:
        diagnostic_sensor_overrides["imu_acc_mode"] = imu_acc_mode
    if scan_time_profile:
        diagnostic_sensor_overrides["scan_time_profile"] = scan_time_profile
    for attr, key in (
        ("diagnostic_imu_acc_lowpass_hz", "imu_acc_lowpass_hz"),
        ("diagnostic_imu_acc_max_dynamic_mps2", "imu_acc_max_dynamic_mps2"),
        ("diagnostic_imu_acc_max_slew_mps3", "imu_acc_max_slew_mps3"),
    ):
        value = getattr(args, attr, None)
        if value is not None:
            diagnostic_sensor_overrides[key] = float(value)
    if diagnostic_sensor_overrides:
        manifest["diagnostic_sensor_overrides"] = diagnostic_sensor_overrides
    configured_video = dict(manifest.get("acceptance_video") or {})
    record_video = getattr(args, "record_video", None)
    manifest["acceptance_video"] = {
        "enabled": (
            bool(configured_video.get("enabled"))
            if record_video is None
            else bool(record_video)
        ),
        "width": int(getattr(args, "video_width", 1920) or 1920),
        "height": int(getattr(args, "video_height", 1080) or 1080),
        "fps": float(getattr(args, "video_fps", 24.0) or 24.0),
        "lidar_points": int(getattr(args, "video_lidar_points", 640) or 640),
    }


    out_dir = Path(args.out_dir).expanduser().resolve()
    requires_wsl_runtime = _requires_wsl_runtime(manifest)
    if requires_wsl_runtime:
        native_runtime_ok, native_runtime_detail = _probe_wsl_runtime()
    else:
        native_runtime_ok = True
        native_runtime_detail = (
            "native_windows_pe" if os.name == "nt" else "native_linux"
        )
    if not native_runtime_ok:
        report = {
            "schema_version": "lingtu.mujoco.native_navigation.acceptance.v1",
            "ok": False,
            "manifest": str(manifest_path),
            "preflight": {
                "ok": False,
                "blockers": ["wsl_runtime_unavailable"],
                "native_runtime_probe": {
                    "ok": False,
                    "detail": native_runtime_detail,
                },
            },
            "phases": {},
            "blockers": ["wsl_runtime_unavailable"],
        }
        report.update(
            classify_evidence(
                manifest.get("acceptance_scope"),
                run_plan_verified=run_plan_verified,
                acceptance_evaluated=False,
                ok=False,
            )
        )
        _write_json(out_dir / "report.json", report)
        return report
    if requires_wsl_runtime:
        artifact_storage_ok, artifact_storage_detail = _artifact_storage_probe(out_dir)
    else:
        artifact_storage_ok = True
        artifact_storage_detail = (
            "native_windows_filesystem" if os.name == "nt" else "native_filesystem"
        )
    motion_requested = str(getattr(args, "mode", "both")) in {"motion", "both"}
    if (
        motion_requested
        and not artifact_storage_ok
        and not bool(getattr(args, "allow_windows_9p_artifacts", False))
    ):
        report = {
            "schema_version": "lingtu.mujoco.native_navigation.acceptance.v1",
            "ok": False,
            "manifest": str(manifest_path),
            "preflight": {
                "ok": False,
                "blockers": ["native_acceptance_artifacts_on_windows_9p"],
                "native_runtime_probe": {
                    "ok": True,
                    "detail": native_runtime_detail,
                },
                "artifact_storage": {
                    "ok": False,
                    "detail": artifact_storage_detail,
                    "path": str(out_dir),
                },
            },
            "phases": {},
            "blockers": ["native_acceptance_artifacts_on_windows_9p"],
        }
        report.update(
            classify_evidence(
                manifest.get("acceptance_scope"),
                run_plan_verified=run_plan_verified,
                acceptance_evaluated=False,
                ok=False,
            )
        )
        _write_json(out_dir / "report.json", report)
        return report
    asset_preparation = (
        _prepare_acceptance_assets(manifest, out_dir)
        if bool(getattr(args, "prepare_assets", True))
        else {"attempted": False, "ok": False, "reason": "disabled"}
    )

    build_result: dict[str, Any] = {}
    if args.build_helper:
        build_result = _build_helper()

    binaries, paths, blockers, provenance = _preflight(manifest)
    native_asset_mirror: dict[str, Any]
    if not requires_wsl_runtime:
        native_asset_mirror = {
            "enabled": False,
            "ok": True,
            "reason": native_runtime_detail,
        }
    elif blockers:
        native_asset_mirror = {
            "enabled": False,
            "ok": False,
            "reason": "preflight_blocked",
        }
    else:
        try:
            paths, native_asset_mirror = _mirror_native_runtime_inputs(paths, out_dir)
        except (OSError, shutil.Error) as exc:
            native_asset_mirror = {
                "enabled": True,
                "ok": False,
                "reason": f"copy_failed:{type(exc).__name__}:{exc}",
            }
            blockers.append("native_asset_mirror_failed")
    provenance["native_runtime_probe"] = {
        "ok": native_runtime_ok,
        "detail": native_runtime_detail,
    }
    provenance["native_asset_mirror"] = native_asset_mirror
    provenance["artifact_storage"] = {
        "ok": artifact_storage_ok,
        "detail": artifact_storage_detail,
        "path": str(out_dir),
    }
    provenance["run_plan_binary_bindings"] = run_plan_binary_bindings
    preflight = {
        "ok": not blockers,
        "blockers": blockers,
        "binaries": {name: str(path) for name, path in binaries.items()},
        "paths": {name: str(path) for name, path in paths.items()},
        "map_provenance": provenance,
        "asset_preparation": asset_preparation,
        "build_helper": build_result,
    }
    if args.preflight_only or blockers:
        report = {
            "schema_version": "lingtu.mujoco.native_navigation.acceptance.v1",
            "ok": not blockers,
            "manifest": str(manifest_path),
            "preflight": preflight,
            "phases": {},
            "blockers": blockers,
        }
        report.update(
            classify_evidence(
                manifest.get("acceptance_scope"),
                run_plan_verified=run_plan_verified,
                acceptance_evaluated=False,
                ok=not blockers,
            )
        )
        _write_json(out_dir / "report.json", report)
        return report

    requested = [args.mode] if args.mode in {"no_motion", "motion"} else ["no_motion", "motion"]
    phase_reports: dict[str, Any] = {}
    domain_base = int(args.domain_id if args.domain_id is not None else manifest.get("domain_id_base") or 220)
    if domain_base < 0 or domain_base + len(requested) - 1 > 232:
        report = {
            "schema_version": "lingtu.mujoco.native_navigation.acceptance.v1",
            "ok": False,
            "manifest": str(manifest_path),
            "preflight": preflight,
            "phases": {},
            "blockers": ["cyclonedds_domain_out_of_port_range"],
        }
        report.update(
            classify_evidence(
                manifest.get("acceptance_scope"),
                run_plan_verified=run_plan_verified,
                acceptance_evaluated=False,
                ok=False,
            )
        )
        _write_json(out_dir / "report.json", report)
        return report
    host_boot_id = secrets.token_hex(16)
    for offset, phase in enumerate(requested):
        phase_reports[phase] = _run_phase(
            phase=phase,
            manifest=manifest,
            binaries=binaries,
            paths=paths,
            out_dir=out_dir,
            domain_id=domain_base + offset,
            host_boot_id=host_boot_id,
            native_environment=native_environment,
        )
    all_blockers = [
        f"{phase}:{blocker}"
        for phase, phase_report in phase_reports.items()
        for blocker in phase_report.get("blockers") or []
    ]
    report = {
        "schema_version": "lingtu.mujoco.native_navigation.acceptance.v1",
        "ok": all(bool(value.get("ok")) for value in phase_reports.values()),
        "manifest": str(manifest_path),
        "preflight": preflight,
        "phases": phase_reports,
        "blockers": all_blockers,
        "acceptance_scope": manifest.get("acceptance_scope") or {},
        "host_boot_id": host_boot_id,
        "chain": [
            "existing saved map",
            "navd",
            "OctoPlanner3D -> LocalPlanner -> embedded PathFollower",
            "native command limits + input freshness",
            "rt/nav/cmd_vel",
            "C++ typed-DDS physical driver bridge",
            "ThunderV4 RL policy",
            "goal arrival",
        ],
        "supporting_inputs": [
            "MuJoCo-LiDAR 10 Hz full frame -> native DDS",
            (
                "MuJoCo truth odometry/TF/body cloud/health fixture -> native DDS"
                if str(((manifest.get("slam_runtime") or {}).get("provider") or "fastlio2")).lower()
                == "mujoco_navigation_fixture"
                else "Fast-LIO2 odometry/registered cloud/health -> native DDS"
            ),
            "Mapd robot-centred local collision snapshot",
            *(
                ["lingtu_traversability_dds"]
                if bool((manifest.get("thresholds") or {}).get("require_traversability", True))
                else []
            ),
        ],
    }
    report.update(
        classify_evidence(
            manifest.get("acceptance_scope"),
            run_plan_verified=run_plan_verified,
            acceptance_evaluated=True,
            ok=bool(report["ok"]),
        )
    )
    _write_json(out_dir / "report.json", report)
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", default=str(DEFAULT_MANIFEST))
    parser.add_argument("--run-plan", type=Path)
    parser.add_argument("--mode", choices=["no_motion", "motion", "both"], default="both")
    parser.add_argument("--domain-id", type=int, default=None)
    parser.add_argument(
        "--world",
        default="",
        help="Optional explicit MuJoCo XML override for a reproducible acceptance rerun.",
    )
    parser.add_argument(
        "--map-dir",
        default="",
        help="Optional explicit saved-map package override paired with --world.",
    )
    parser.add_argument(
        "--out-dir",
        default=str(ROOT / "artifacts" / "mujoco_native_navigation_acceptance"),
    )
    parser.add_argument("--build-helper", action="store_true")
    parser.add_argument(
        "--prepare-assets",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Generate a same-source MuJoCo scene and navigation map when configured assets are absent.",
    )
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Dispatcher compatibility flag; component evidence scope is unchanged.",
    )
    parser.add_argument(
        "--allow-windows-9p-artifacts",
        action="store_true",
        help=(
            "Diagnostic override: allow motion acceptance artifacts on a Windows "
            "/mnt/<drive> (9p) mount. Product motion acceptance should use a "
            "\\\\wsl.localhost\\<distro>\\... path instead."
        ),
    )
    parser.add_argument(
        "--phase-duration-s",
        type=float,
        default=None,
        help="Diagnostic phase-duration override; the product acceptance manifest remains unchanged.",
    )
    parser.add_argument(
        "--record-video",
        action=argparse.BooleanOptionalAction,
        default=None,
        help=(
            "Override acceptance video generation; the manifest default is used "
            "when omitted."
        ),
    )
    parser.add_argument("--video-width", type=int, default=1920)
    parser.add_argument("--video-height", type=int, default=1080)
    parser.add_argument("--video-fps", type=float, default=24.0)
    parser.add_argument("--video-lidar-points", type=int, default=640)
    parser.add_argument(
        "--diagnostic-imu-acc-mode",
        choices=["sensor", "gravity_only", "finite_difference"],
        default="",
    )
    parser.add_argument(
        "--diagnostic-scan-time-profile",
        choices=["physical_rolling", "instantaneous", "synthetic_rolling"],
        default="",
    )
    parser.add_argument("--diagnostic-imu-acc-lowpass-hz", type=float, default=None)
    parser.add_argument("--diagnostic-imu-acc-max-dynamic-mps2", type=float, default=None)
    parser.add_argument("--diagnostic-imu-acc-max-slew-mps3", type=float, default=None)
    args = parser.parse_args(argv)
    if args.run_plan is not None:
        plan = validate_runner_plan(
            ROOT,
            args.run_plan,
            Path(args.manifest),
            expected_products=("nav", "tracking", "inspection"),
        )
        args.run_plan_roles = tuple(
            sorted({role for process in plan.processes for role in process.provides})
        )
        args.run_plan_verified = True
        args.validated_run_plan = plan
    report = run(args)
    print(json.dumps(report, ensure_ascii=True, indent=2, sort_keys=True))
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())

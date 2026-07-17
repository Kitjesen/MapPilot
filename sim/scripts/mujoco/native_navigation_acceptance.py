#!/usr/bin/env python3
"""Run the ROS-free MuJoCo navigation chain with field C++ runtimes.

The Python process owns MuJoCo physics, simulated sensors, process lifecycle,
and acceptance reporting only. Global planning, local planning, path following,
and command safety remain inside ``lingtu_nav_native_endpoint``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
RUNTIME_EVIDENCE_SAMPLE_PERIOD_S = 0.20


def _phase_runtime_timeout_s(duration_s: float, shutdown_grace_s: float = 120.0) -> float:
    """Allow native Windows/WSL children to flush and terminate after simulation."""

    return max(0.0, float(duration_s)) + max(60.0, float(shutdown_grace_s))
SENSOR_PUBLISHER_PROBE_TIMEOUT_S = 60.0
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

from sim.scripts.mujoco.native_dds_sensors import (
    _linux_binary_command,
    _managed_wsl_command,
    _read_linux_pid,
    _signal_wsl_pid,
    _wait_wsl_pid_exit,
    _wsl_path,
    _wsl_pid_alive,
)

DEFAULT_MANIFEST = ROOT / "config" / "runtime_graph" / "endpoints" / "mujoco_native_navigation_acceptance.json"
DEFAULT_THUNDERV4_MJCF = ROOT / "sim" / "robots" / "thunderv4" / "mjcf" / "thunderv4.xml"


def _compiled_mujoco_site_offset_body(
    model_path: Path,
    *,
    site_name: str = "lidar_site",
    body_name: str = "base_link",
) -> tuple[float, float, float]:
    """Resolve a site's final compiled pose relative to the robot body."""

    import mujoco
    import numpy as np

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


def _merge_manifest(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, value in override.items():
        if key == "extends":
            continue
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _merge_manifest(dict(merged[key]), value)
        else:
            merged[key] = value
    return merged


def _load_manifest(path: Path, seen: set[Path] | None = None) -> dict[str, Any]:
    resolved = path.expanduser().resolve()
    visited = set(seen or set())
    if resolved in visited:
        raise ValueError(f"manifest inheritance cycle: {resolved}")
    visited.add(resolved)
    manifest = _load_json(resolved)
    parent_value = str(manifest.get("extends") or "").strip()
    if not parent_value:
        return manifest
    parent = Path(parent_value)
    if not parent.is_absolute():
        parent = resolved.parent / parent
    return _merge_manifest(_load_manifest(parent, visited), manifest)


def _write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=True, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _repo_path(value: str) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (ROOT / path).resolve()


def _linux_arg(path: Path) -> str:
    return _wsl_path(path) if os.name == "nt" else str(path)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _text_tail(value: str | None, limit: int = 4000) -> str:
    return (value or "")[-limit:]


def _resolve_binary(spec: dict[str, Any]) -> Path | None:
    env_name = str(spec.get("env") or "")
    env_value = str(os.environ.get(env_name, "")) if env_name else ""
    values = [env_value] if env_value else []
    values.extend(str(value) for value in spec.get("candidates") or [])
    for value in values:
        if not value or (value.startswith("/home/") and os.name == "nt"):
            continue
        candidate = _repo_path(value)
        if candidate.is_file():
            return candidate
    return None


def _native_command(binary: Path, *args: str) -> list[str]:
    return _linux_binary_command(binary, *args)


def _probe_sensor_publisher(binary: Path) -> tuple[bool, str]:
    command = _native_command(binary, "--stdin-records", "--dds", "--domain-id", "0")
    try:
        proc = subprocess.run(
            command,
            cwd=ROOT,
            input="",
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=SENSOR_PUBLISHER_PROBE_TIMEOUT_S,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, f"{type(exc).__name__}:{exc}"
    output = f"{proc.stdout or ''}\n{proc.stderr or ''}".strip()
    if proc.returncode != 0 or "built without DDS support" in output:
        return False, output[-2000:]
    return True, output[-2000:]


@dataclass
class ManagedProcess:
    name: str
    command: list[str]
    log_path: Path
    process: subprocess.Popen[str] | None = None
    _log: Any = None
    linux_pid: int | None = None
    pid_path: Path | None = None
    cleanup: dict[str, Any] = field(default_factory=dict)

    def start(self) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log = self.log_path.open("w", encoding="utf-8")
        launch_command = self.command
        if os.name == "nt" and len(self.command) >= 3 and self.command[1] == "-e":
            self.pid_path = self.log_path.with_suffix(".pid")
            launch_command = _managed_wsl_command(self.command, self.pid_path)
        self.process = subprocess.Popen(
            launch_command,
            cwd=ROOT,
            stdout=self._log,
            stderr=subprocess.STDOUT,
            text=True,
        )
        if self.pid_path is not None:
            self.linux_pid = _read_linux_pid(self.pid_path)

    def poll(self) -> int | None:
        return self.process.poll() if self.process is not None else None

    def wait(self, timeout_s: float) -> int:
        if self.process is None:
            raise RuntimeError(f"process was not started: {self.name}")
        return int(self.process.wait(timeout=max(0.1, timeout_s)))

    def stop(self) -> None:
        alive_before = _wsl_pid_alive(self.linux_pid)
        if alive_before:
            _signal_wsl_pid(self.linux_pid, "TERM")
            if not _wait_wsl_pid_exit(self.linux_pid, 3.0):
                _signal_wsl_pid(self.linux_pid, "KILL")
                _wait_wsl_pid_exit(self.linux_pid, 1.0)
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1.0)
        if self._log is not None:
            self._log.close()
            self._log = None
        alive_after = _wsl_pid_alive(self.linux_pid)
        self.cleanup = {
            "name": self.name,
            "linux_pid": self.linux_pid,
            "pid_file": str(self.pid_path or ""),
            "alive_before_cleanup": alive_before,
            "alive_after_cleanup": alive_after,
            "clean": not alive_after,
            "relay_returncode": self.process.poll() if self.process is not None else None,
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

    def sample(self, *, nav_path: Path, slam_path: Path, traversability_path: Path) -> None:
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
            collect_motion_health = not self._motion_health_frozen
            self.goal_reached_observed = self.goal_reached_observed or goal_reached_now
            if collect_motion_health:
                self.motion_health_samples += 1
                input_gate = nav.get("input_gate") or {}
                if "ready" in input_gate:
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
        map_hash = _sha256(paths["slam"])
        planner_hash = _sha256(paths["planner"])
        expected_map = str(((metadata.get("artifacts") or {}).get("map_pcd") or {}).get("sha256") or "")
        octomap = (metadata.get("artifacts") or {}).get("octomap") or {}
        expected_planner = str(octomap.get("sha256") or "")
        expected_source = str(octomap.get("source_map_sha256") or "")
        provenance = {
            "map_sha256": map_hash,
            "planner_sha256": planner_hash,
            "metadata_map_sha256": expected_map,
            "metadata_planner_sha256": expected_planner,
            "metadata_planner_source_sha256": expected_source,
        }
        if expected_map and expected_map != map_hash:
            blockers.append("map_metadata_hash_mismatch")
        if expected_planner and expected_planner != planner_hash:
            blockers.append("octomap_metadata_hash_mismatch")
        if expected_source and expected_source != map_hash:
            blockers.append("octomap_not_derived_from_selected_map")
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
        return {
            "attempted": False,
            "ok": True,
            "reason": "configured_assets_ready",
            "scene_xml": str(world_path),
            "map_dir": str(_repo_path(str(manifest.get("map_dir") or ""))),
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
    build_ok = bool((report.get("build") or {}).get("ok"))
    artifact_ok = bool((report.get("artifact_gate") or {}).get("ok"))
    ok = build_ok and artifact_ok and scene_xml.is_file() and map_dir.is_dir()
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
    }


def _preflight(manifest: dict[str, Any]) -> tuple[dict[str, Path], dict[str, Path], list[str], dict[str, Any]]:
    blockers: list[str] = []
    binaries: dict[str, Path] = {}
    state_provider = str(
        ((manifest.get("slam_runtime") or {}).get("provider") or "fastlio2")
    ).strip().lower()
    for name, raw_spec in (manifest.get("binaries") or {}).items():
        if name == "slam" and state_provider == "mujoco_navigation_fixture":
            continue
        spec = raw_spec if isinstance(raw_spec, dict) else {}
        binary = _resolve_binary(spec)
        if binary is None:
            blockers.append(f"native_binary_missing:{name}")
        else:
            binaries[str(name)] = binary
    if "sensor_publisher" in binaries:
        publisher_ok, publisher_probe = _probe_sensor_publisher(binaries["sensor_publisher"])
        if not publisher_ok:
            blockers.append("sensor_publisher_dds_unavailable")
        provenance_probe = {"sensor_publisher_probe": publisher_probe}
    else:
        provenance_probe = {}

    paths_cfg = manifest.get("paths") or {}
    paths = {
        "path_library": _repo_path(str(paths_cfg.get("path_library") or "")),
        "sensor_runner": _repo_path(str(paths_cfg.get("sensor_runner") or "")),
    }
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

    map_paths, map_blockers, provenance = _validate_map(manifest)
    blockers.extend(map_blockers)
    paths.update(map_paths)

    policy_value = str(paths_cfg.get("policy") or "").strip()
    policy = (
        _repo_path(policy_value)
        if policy_value
        else ROOT / "sim" / "robots" / "thunderv4" / "policy" / "pose_flat_low_kpkd_microterrain_model29600_policy.onnx"
    )
    paths["policy"] = policy
    if not policy.is_file():
        blockers.append(f"thunderv4_policy_missing:{policy}")
    provenance.update(provenance_probe)
    return binaries, paths, blockers, provenance


def _build_helper() -> dict[str, Any]:
    source = ROOT / "sim" / "native_dds"
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
    timeout_s: float,
    thresholds: dict[str, Any],
) -> tuple[bool, str]:
    deadline = time.monotonic() + max(0.1, timeout_s)
    required_stable_s = max(0.0, float(thresholds.get("startup_ready_stable_s") or 0.0))
    ready_since_s: float | None = None
    while time.monotonic() < deadline:
        evidence.sample(
            nav_path=nav_status,
            slam_path=slam_status,
            traversability_path=traversability_status,
        )
        if sensor.poll() is not None:
            return False, "sensor_runner_exited_before_native_runtime_ready"
        slam = evidence.last_slam
        nav = evidence.last_nav
        traversability = evidence.last_traversability
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
        runtime_ready = (
            slam_readiness is not None
            and bool(nav.get("has_odom"))
            and fixture_gate_ready
            and int((traversability.get("counters") or {}).get("published") or 0) > 0
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
    return {
        "available": True,
        "start_distance_m": start_distance,
        "end_distance_m": end_distance,
        "distance_reduction_m": start_distance - end_distance,
        "sim_path_length_xy_m": float(motion.get("sim_path_length_xy_m") or 0.0),
    }


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
        "stop_on_nav_goal_reached",
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
    if bool(config.get("stop_on_nav_goal_reached")):
        arguments.append("--stop-on-nav-goal-reached")
    return arguments


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
        if bool(nav_status.get("legacy_motion_inputs_enabled")):
            blockers.append("legacy_motion_inputs_enabled")
        if nav_status.get("check_obstacle") is not True:
            blockers.append("obstacle_slow_stop_not_enabled")
        if nav_status.get("use_traversability_cost") is not True:
            blockers.append("terrain_cost_not_enabled")
        input_gate = nav_status.get("input_gate") or {}
        for required_input in (
            "require_odom",
            "require_cloud",
            "require_traversability",
            "require_localization_health",
        ):
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
        if max_static_map_error_m is not None:
            try:
                static_map_error_m = float(motion.get("slam_map_xy_error_m"))
            except (TypeError, ValueError):
                static_map_error_m = math.inf
            if static_map_error_m > float(max_static_map_error_m):
                blockers.append("no_motion_slam_map_pose_drift")
        max_static_odom_m = thresholds.get("max_no_motion_slam_odom_xy_m")
        if max_static_odom_m is not None:
            try:
                static_odom_m = float(motion.get("slam_odom_xy_m"))
            except (TypeError, ValueError):
                static_odom_m = math.inf
            if static_odom_m > float(max_static_odom_m):
                blockers.append("no_motion_slam_odom_drift")
    elif phase == "motion":
        if frame_contract is not None and not bool(frame_contract.get("ok")):
            blockers.append("traversability_frame_contract_mismatch")
        if not publish_cmd_vel:
            blockers.append("motion_phase_publish_cmd_vel_disabled")
        if evidence.max_cmd_vel_published <= 0:
            blockers.append("native_endpoint_did_not_publish_cmd_vel")
        if int(cmd_vel.get("nonzero_samples") or 0) < int(thresholds.get("min_cmd_vel_samples") or 3):
            blockers.append("mujoco_policy_received_too_few_nonzero_cmd_vel_samples")
        max_motion_map_error_m = thresholds.get("max_motion_slam_map_xy_error_m")
        if max_motion_map_error_m is not None:
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
            if float(goal_metrics.get("distance_reduction_m") or 0.0) < float(
                thresholds.get("min_goal_distance_reduction_m") or 0.1
            ):
                blockers.append("goal_distance_did_not_decrease")
            max_goal_error_m = thresholds.get("max_goal_error_m")
            if max_goal_error_m is not None and float(goal_metrics.get("end_distance_m")) > float(max_goal_error_m):
                blockers.append("final_goal_error_above_threshold")
        if bool(thresholds.get("require_goal_reached")) and not native_goal_reached:
            blockers.append("native_goal_not_reached")
    return not blockers, blockers, goal_metrics


def _video_artifact_blocker(
    *,
    required: bool,
    video_report: dict[str, Any],
) -> str | None:
    """Return the video gate blocker, including when recording was not requested."""

    if not required or video_report.get("ok") is True:
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
) -> dict[str, Any]:
    phase_cfg = dict((manifest.get("phases") or {}).get(phase) or {})
    thresholds = dict(manifest.get("thresholds") or {})
    runtime_tolerances = dict(manifest.get("runtime_tolerances") or {})
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
    nav_status = phase_dir / "nav_status.json"
    sensor_report_path = phase_dir / "sensor_report.json"
    motion_log_path = phase_dir / "motion.jsonl"
    sensor_publisher_pid = phase_dir / "sensor_publisher.pid"
    cmd_vel_tap_pid = phase_dir / "cmd_vel_tap.pid"
    for path in (slam_status, traversability_status, nav_status, sensor_report_path, motion_log_path):
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
            _linux_arg(paths["slam_config"]),
            "--domain-id",
            str(domain_id),
            "--tick-hz",
            "50",
            "--status-json",
            _linux_arg(slam_status),
            "--status-json-hz",
            str(float(slam_runtime_cfg.get("status_json_hz") or 10.0)),
            "--cloud-snapshot-dir",
            _linux_arg(slam_cloud_dir),
            "--cloud-snapshot-hz",
            str(float(slam_runtime_cfg.get("cloud_snapshot_hz") or 5.0)),
        ]
    if use_slam_process and slam_mode == "localization":
        slam_args.extend(
            [
                "--map",
                _linux_arg(paths["slam"]),
                "--track-against-map-period-s",
                str(float(runtime_tolerances.get("track_against_map_period_s") or 1.0)),
                "--track-against-map-initial-pose",
                *(str(value) for value in start),
            ]
        )

    slam = (
        ManagedProcess(
            "slam",
            _native_command(binaries["slam"], *slam_args),
            phase_dir / "slam.log",
        )
        if use_slam_process
        else None
    )
    lidar_offset_body = _compiled_mujoco_site_offset_body(DEFAULT_THUNDERV4_MJCF)
    lidar_offset_args = _sensor_offset_args(lidar_offset_body)
    traversability_runtime_cfg = dict(manifest.get("traversability_runtime") or {})
    traversability = ManagedProcess(
        "traversability",
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
            _linux_arg(traversability_status),
        ),
        phase_dir / "traversability.log",
    )
    navigation_runtime_cfg = dict(manifest.get("navigation_runtime") or {})
    local_planner_debug_candidates = int(
        navigation_runtime_cfg.get("debug_candidate_limit", 36 if record_video else 0)
    )
    local_map_debug_points = int(
        navigation_runtime_cfg.get("debug_local_map_points", 320 if record_video else 0)
    )
    status_period_s = float(navigation_runtime_cfg.get("status_period_s") or 0.1)
    if record_video and (local_planner_debug_candidates > 0 or local_map_debug_points > 0):
        status_period_s = min(status_period_s, 0.2)
    navigation = ManagedProcess(
        "navigation",
        _native_command(
            binaries["navigation"],
            "--control-mode",
            "autonomy",
            "--domain-id",
            str(domain_id),
            "--path-library",
            _linux_arg(paths["path_library"]),
            "--map",
            _linux_arg(paths["planner"]),
            *_planner_constraint_args(manifest),
            "--tick-hz",
            "20",
            "--corridor-lookahead-m",
            str(float(navigation_runtime_cfg.get("corridor_lookahead_m", 3.0))),
            "--max-obstacle-points",
            str(int(navigation_runtime_cfg.get("max_obstacle_points") or 5000)),
            "--local-planner-obstacle-height-max-m",
            str(float(navigation_runtime_cfg.get("obstacle_height_max_m", 1.2))),
            "--obstacle-terrain-ext-share",
            str(float(navigation_runtime_cfg.get("obstacle_terrain_ext_share", 0.0))),
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
            "true",
            "--status-file",
            _linux_arg(nav_status),
            "--status-s",
            str(status_period_s),
        ),
        phase_dir / "navigation.log",
    )
    sensor_args = [
        sys.executable,
        str(paths["sensor_runner"]),
        "--world",
        world_arg,
        "--start",
        ",".join(str(value) for value in start[:3]),
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
        "--cmd-vel-tap-bin",
        str(binaries["cmd_vel_tap"]),
        "--cmd-vel-pid-file",
        str(cmd_vel_tap_pid),
        "--cmd-vel-timeout-s",
        "0.25",
        "--sim-hardware-realtime-factor",
        str(float(runtime_tolerances.get("sim_hardware_realtime_factor") or 0.75)),
        "--publisher-bin",
        str(binaries["sensor_publisher"]),
        "--publisher-pid-file",
        str(sensor_publisher_pid),
        "--domain-id",
        str(domain_id),
        "--nav-status-json", str(nav_status),
        "--json-out",
        str(sensor_report_path),
    ]
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
    if record_video or record_telemetry:
        motion_log_hz = float(video_cfg.get("fps") or 24.0) if record_video else float(telemetry_cfg.get("hz") or 10.0)
        motion_log_lidar_points = (
            int(video_cfg.get("lidar_points") or 640) if record_video else int(telemetry_cfg.get("lidar_points") or 0)
        )
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
    sensor = ManagedProcess("sensor", sensor_args, phase_dir / "sensor.log")
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
        )
        if os.name == "nt" and binaries["navigation_control"].suffix.lower() != ".exe"
        else None
    )
    processes = [
        process
        for process in (slam, traversability, navigation, deferred_goal, sensor)
        if process is not None
    ]
    evidence = NativeEvidence()
    startup_ok = False
    startup_reason = "not_started"
    goal_result: dict[str, Any] = {}
    phase_error = ""
    process_cleanup: list[dict[str, Any]] = []
    started_commands: list[dict[str, Any]] = []

    try:
        for process in processes:
            process.start()
            started_commands.append({"name": process.name, "command": process.command})
        startup_ok, startup_reason = _wait_for_startup(
            sensor=sensor,
            evidence=evidence,
            nav_status=nav_status,
            slam_status=slam_status,
            traversability_status=traversability_status,
            timeout_s=float(thresholds.get("startup_timeout_s") or 18.0),
            thresholds=thresholds,
        )
        if startup_ok:
            plan_timeout_s = float(thresholds.get("plan_timeout_s") or 12.0)
            if deferred_goal is not None:
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
            else:
                goal_result = _goal_command(
                    binaries["navigation_control"],
                    goal,
                    domain_id,
                    timeout_s=plan_timeout_s,
                )
        phase_duration_s = float(phase_cfg.get("duration_s") or 12.0)
        deadline = time.monotonic() + _phase_runtime_timeout_s(
            phase_duration_s,
            float(thresholds.get("runner_shutdown_grace_s") or 120.0),
        )
        while sensor.poll() is None and time.monotonic() < deadline:
            evidence.sample(
                nav_path=nav_status,
                slam_path=slam_status,
                traversability_path=traversability_status,
            )
            time.sleep(RUNTIME_EVIDENCE_SAMPLE_PERIOD_S)
        if sensor.poll() is None:
            raise TimeoutError("MuJoCo sensor/locomotion runner exceeded phase deadline")
        evidence.sample(
            nav_path=nav_status,
            slam_path=slam_status,
            traversability_path=traversability_status,
        )
    except Exception as exc:
        phase_error = f"{type(exc).__name__}: {exc}"
    finally:
        for process in reversed(processes):
            process.stop()
        process_cleanup.extend(process.cleanup for process in processes)
        process_cleanup.append(_cleanup_pid_file("sensor_publisher", sensor_publisher_pid))
        process_cleanup.append(_cleanup_pid_file("cmd_vel_tap", cmd_vel_tap_pid))

    sensor_report = _load_json(sensor_report_path)
    sensor_acceptance = _navigation_sensor_assessment(sensor_report, thresholds)
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
            )
            video_report["requested"] = True
        except Exception as exc:
            video_report = {
                "requested": True,
                "ok": False,
                "reason": f"{type(exc).__name__}:{exc}",
                "motion_log": str(motion_log_path),
            }
    traversability_frame_contract = _traversability_frame_contract(
        evidence,
        dict(manifest.get("frame_contract") or {}),
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
    if not startup_ok:
        blockers.append(startup_reason)
    if int(goal_result.get("returncode") or 0) != 0 or not goal_result:
        blockers.append("native_goal_command_failed")
    if phase_error:
        blockers.append("phase_runtime_error")
    if sensor_acceptance.get("navigation_critical_ok") is not True:
        blockers.append("sensor_or_slam_acceptance_failed")
    if not all(bool(item.get("clean")) for item in process_cleanup):
        blockers.append("acceptance_process_cleanup_failed")
    video_blocker = _video_artifact_blocker(
        required=bool(thresholds.get("require_video_artifact", True)),
        video_report=video_report,
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
        "navigation_compute_owner": "lingtu_nav_native_endpoint",
        "navigation_state_provider": state_provider,
        "startup": {"ok": startup_ok, "reason": startup_reason},
        "goal_command": goal_result,
        "goal_metrics": goal_metrics,
        "pre_safety_path_follower_command": _pre_safety_command_evidence(evidence),
        "post_safety_command": {
            "dds_topic": "rt/nav/cmd_vel",
            "published_samples": evidence.max_cmd_vel_published,
            "tap_nonzero_samples": int((sensor_report.get("cmd_vel") or {}).get("nonzero_samples") or 0),
        },
        "traversability_frame_contract": traversability_frame_contract,
        "process_cleanup": {
            "zero_leftovers": all(bool(item.get("clean")) for item in process_cleanup),
            "processes": process_cleanup,
        },
        "evidence": evidence.to_dict(),
        "sensor_report": sensor_report,
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


def run(args: argparse.Namespace) -> dict[str, Any]:
    manifest_path = Path(args.manifest).expanduser().resolve()
    manifest = _load_manifest(manifest_path)
    if not manifest:
        return {
            "schema_version": "lingtu.mujoco.native_navigation.acceptance.v1",
            "ok": False,
            "blockers": [f"manifest_unreadable:{manifest_path}"],
        }
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
    manifest["acceptance_video"] = {
        "enabled": bool(getattr(args, "record_video", False)),
        "width": int(getattr(args, "video_width", 1920) or 1920),
        "height": int(getattr(args, "video_height", 1080) or 1080),
        "fps": float(getattr(args, "video_fps", 24.0) or 24.0),
        "lidar_points": int(getattr(args, "video_lidar_points", 640) or 640),
    }

    out_dir = Path(args.out_dir).expanduser().resolve()
    asset_preparation = (
        _prepare_acceptance_assets(manifest, out_dir)
        if bool(getattr(args, "prepare_assets", True))
        else {"attempted": False, "ok": False, "reason": "disabled"}
    )

    build_result: dict[str, Any] = {}
    if args.build_helper:
        build_result = _build_helper()

    binaries, paths, blockers, provenance = _preflight(manifest)
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
        _write_json(out_dir / "report.json", report)
        return report
    for offset, phase in enumerate(requested):
        phase_reports[phase] = _run_phase(
            phase=phase,
            manifest=manifest,
            binaries=binaries,
            paths=paths,
            out_dir=out_dir,
            domain_id=domain_base + offset,
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
        "chain": [
            "existing saved map",
            "lingtu_nav_native_endpoint",
            "OctoPlanner3D -> LocalPlanner -> embedded PathFollower",
            "native slow/stop + terrain cost + stale fail-safe",
            "rt/nav/cmd_vel",
            "C++ typed-DDS tap",
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
            "lingtu_traversability_dds",
        ],
    }
    _write_json(out_dir / "report.json", report)
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", default=str(DEFAULT_MANIFEST))
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
        "--phase-duration-s",
        type=float,
        default=None,
        help="Diagnostic phase-duration override; the product acceptance manifest remains unchanged.",
    )
    parser.add_argument(
        "--record-video",
        action="store_true",
        help="Render a clean MuJoCo video from the exact native-DDS motion run.",
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
    report = run(args)
    print(json.dumps(report, ensure_ascii=True, indent=2, sort_keys=True))
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Drive the native MuJoCo ``teleop_avoid`` chain with a WASD keyboard.

This is an operator-intent client, not a MuJoCo controller.  Every command goes
through the typed navigation request/ACK boundary; the native endpoint remains
the only process that can publish final ``/nav/cmd_vel``.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.scripts.mujoco import native_navigation_acceptance as native
from sim.scripts.mujoco import teleop_avoid_native_acceptance as acceptance

SCHEMA_VERSION = "lingtu.mujoco.teleop_avoid_wasd.v1"
TELEOP_STREAM_TIMEOUT_STOP_MARKER = "LT_TELEOP_STREAM_TIMEOUT_STOP_V1"
DEFAULT_ARTIFACT_DIR = ROOT / "artifacts" / "mujoco_teleop_avoid_wasd"
MIN_ISOLATED_DDS_DOMAIN_ID = 200
MAX_CYCLONEDDS_UDP_DOMAIN_ID = 232
VIEWER_DRIVER_TIMEOUT_MS = 10_000
VIEWER_QUEUE_LIMITS = {
    "sensor_publisher_async_max_bytes": 64 * 1024 * 1024,
    "sensor_publisher_async_max_records": 4096,
    "sensor_publisher_async_max_batches": 1024,
    "sensor_publisher_async_oldest_s": 10.0,
    "sensor_publisher_async_shutdown_s": 10.0,
}
INTERACTIVE_SCENARIOS = (
    "free",
    "obstacle_detour_left",
    "obstacle_detour_right",
    "obstacle_slow",
    "obstacle_stop",
    "terrain_soft",
    "terrain_hard",
)
STATE_PROVIDERS = ("mujoco_fixture", "fastlio2")


@dataclass(frozen=True)
class TeleopTwist:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    manual_mode: bool = False

    def is_zero(self) -> bool:
        return abs(self.vx) < 1e-9 and abs(self.vy) < 1e-9 and abs(self.wz) < 1e-9


@dataclass(frozen=True)
class KeyboardCommandState:
    linear_speed_mps: float = 0.18
    lateral_speed_mps: float = 0.15
    yaw_speed_rad_s: float = 0.35
    require_deadman: bool = True

    def command(self, pressed: set[str] | frozenset[str]) -> TeleopTwist:
        keys = {str(value).strip().lower() for value in pressed}
        if "space" in keys or (self.require_deadman and "shift" not in keys):
            return TeleopTwist()
        forward = int("w" in keys) - int("s" in keys)
        lateral = int("a" in keys) - int("d" in keys)
        yaw = int("q" in keys) - int("e" in keys)
        return TeleopTwist(
            vx=float(forward) * max(0.0, float(self.linear_speed_mps)),
            vy=float(lateral) * max(0.0, float(self.lateral_speed_mps)),
            wz=float(yaw) * max(0.0, float(self.yaw_speed_rad_s)),
            manual_mode="m" in keys,
        )


class KeyPoller(Protocol):
    def start(self) -> None: ...

    def snapshot(self) -> set[str]: ...

    def close(self) -> None: ...


class WindowsKeyPoller:
    """Poll held keys without adding a Windows keyboard dependency."""

    _VIRTUAL_KEYS = {
        "w": ord("W"),
        "a": ord("A"),
        "s": ord("S"),
        "d": ord("D"),
        "q": ord("Q"),
        "e": ord("E"),
        "m": ord("M"),
        "r": ord("R"),
        "shift": 0x10,
        "space": 0x20,
        "escape": 0x1B,
    }

    def __init__(self, key_state: Callable[[int], int] | None = None) -> None:
        if key_state is None:
            import ctypes

            key_state = ctypes.windll.user32.GetAsyncKeyState
        self._key_state = key_state

    def start(self) -> None:
        return None

    def snapshot(self) -> set[str]:
        return {
            name
            for name, code in self._VIRTUAL_KEYS.items()
            if int(self._key_state(code)) & 0x8000
        }

    def close(self) -> None:
        return None


class PynputKeyPoller:
    """Track key press/release events on non-Windows desktop hosts."""

    def __init__(self) -> None:
        try:
            from pynput import keyboard
        except ImportError as exc:  # pragma: no cover - host dependent
            raise RuntimeError(
                "Non-Windows interactive keyboard input requires the existing optional pynput package"
            ) from exc
        self._keyboard = keyboard
        self._lock = threading.Lock()
        self._pressed: set[str] = set()
        self._listener: Any = None

    def _name(self, key: Any) -> str | None:
        char = getattr(key, "char", None)
        if isinstance(char, str) and char.lower() in {
            "w",
            "a",
            "s",
            "d",
            "q",
            "e",
            "m",
            "r",
        }:
            return char.lower()
        if key in {self._keyboard.Key.shift, self._keyboard.Key.shift_l, self._keyboard.Key.shift_r}:
            return "shift"
        if key == self._keyboard.Key.space:
            return "space"
        if key == self._keyboard.Key.esc:
            return "escape"
        return None

    def start(self) -> None:
        def on_press(key: Any) -> None:
            name = self._name(key)
            if name is not None:
                with self._lock:
                    self._pressed.add(name)

        def on_release(key: Any) -> None:
            name = self._name(key)
            if name is not None:
                with self._lock:
                    self._pressed.discard(name)

        self._listener = self._keyboard.Listener(on_press=on_press, on_release=on_release)
        self._listener.start()

    def snapshot(self) -> set[str]:
        with self._lock:
            return set(self._pressed)

    def close(self) -> None:
        if self._listener is not None:
            self._listener.stop()
            self._listener = None


def create_key_poller() -> KeyPoller:
    return WindowsKeyPoller() if os.name == "nt" else PynputKeyPoller()


def _pressed_once(pressed: set[str], previous: set[str], key: str) -> bool:
    return key in pressed and key not in previous


def _input_timeout(requested_ms: int, *, viewer: bool) -> int:
    timeout_ms = max(50, int(requested_ms))
    return max(timeout_ms, VIEWER_DRIVER_TIMEOUT_MS) if viewer else timeout_ms


def _send_resume(binary: Path, domain_id: int) -> Mapping[str, Any]:
    return acceptance._run_control(
        binary,
        ["resume", "mujoco_wasd_operator_resume"],
        domain_id=domain_id,
        timeout_s=8.0,
    )


def build_teleop_stream_command(
    binary: Path,
    ready_file: Path,
    *,
    domain_id: int,
    rate_hz: float = 10.0,
    timeout_ms: int = 3000,
    input_timeout_ms: int = 350,
) -> list[str]:
    """Build one stdin-driven typed-DDS teleop command stream."""

    ready_file_arg = (
        str(Path(ready_file).resolve())
        if Path(binary).suffix.lower() == ".exe"
        else native._linux_arg(Path(ready_file))
    )
    return native._native_command(
        Path(binary),
        "teleop-stream",
        "--rate-hz",
        str(max(1.0, float(rate_hz))),
        "--domain-id",
        str(int(domain_id)),
        "--timeout-ms",
        str(max(1, int(timeout_ms))),
        "--input-timeout-ms",
        str(max(50, int(input_timeout_ms))),
        "--ready-file",
        ready_file_arg,
    )


class TeleopStreamLike(Protocol):
    def start(self) -> None: ...

    def send(self, twist: TeleopTwist) -> None: ...

    def poll(self) -> int | None: ...

    def close(self, reason: str) -> Mapping[str, Any]: ...


@dataclass
class NativeTeleopStream:
    """Own one native command client and update its latest intent over stdin."""

    binary: Path
    domain_id: int
    log_path: Path
    ready_path: Path
    rate_hz: float = 10.0
    timeout_ms: int = 3000
    input_timeout_ms: int = 350
    startup_timeout_s: float = 8.0
    process: subprocess.Popen[str] | None = field(default=None, init=False)
    linux_pid: int | None = field(default=None, init=False)
    pid_path: Path | None = field(default=None, init=False)
    samples: int = field(default=0, init=False)
    _log: Any = field(default=None, init=False, repr=False)

    def start(self) -> None:
        if self.process is not None:
            raise RuntimeError("typed teleop stream already started")
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.ready_path.parent.mkdir(parents=True, exist_ok=True)
        self.ready_path.unlink(missing_ok=True)
        command = build_teleop_stream_command(
            self.binary,
            self.ready_path,
            domain_id=self.domain_id,
            rate_hz=self.rate_hz,
            timeout_ms=self.timeout_ms,
            input_timeout_ms=self.input_timeout_ms,
        )
        self._log = self.log_path.open("w", encoding="utf-8")
        launch_command = command
        if os.name == "nt" and len(command) >= 3 and command[1] == "-e":
            self.pid_path = self.log_path.with_suffix(".pid")
            launch_command = native._managed_wsl_command(command, self.pid_path)
        try:
            self.process = subprocess.Popen(
                launch_command,
                cwd=ROOT,
                stdin=subprocess.PIPE,
                stdout=self._log,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                bufsize=1,
            )
            if self.pid_path is not None:
                self.linux_pid = native._read_linux_pid(self.pid_path)
            deadline = time.monotonic() + max(0.1, float(self.startup_timeout_s))
            while time.monotonic() < deadline:
                returncode = self.poll()
                if returncode is not None:
                    raise RuntimeError(
                        f"typed teleop stream exited before ready: {returncode}; {self.tail()}"
                    )
                try:
                    if self.ready_path.read_text(encoding="ascii").strip() == "ready":
                        return
                except OSError:
                    pass
                time.sleep(0.02)
            raise TimeoutError("typed teleop stream did not acknowledge initial zero command")
        except Exception:
            self._force_cleanup()
            raise

    def send(self, twist: TeleopTwist) -> None:
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError(f"typed teleop stream is not running: {self.poll()}")
        if self.process.stdin is None:
            raise RuntimeError("typed teleop stream stdin is unavailable")
        self.process.stdin.write(
            f"{twist.vx:.12g} {twist.vy:.12g} {twist.wz:.12g} "
            f"{int(twist.manual_mode)}\n"
        )
        self.process.stdin.flush()
        self.samples += 1

    def poll(self) -> int | None:
        return self.process.poll() if self.process is not None else None

    def tail(self, limit: int = 2000) -> str:
        try:
            return self.log_path.read_text(encoding="utf-8", errors="replace")[-limit:]
        except OSError:
            return ""

    def _force_cleanup(self) -> dict[str, Any]:
        alive_before = native._wsl_pid_alive(self.linux_pid)
        if alive_before:
            native._signal_wsl_pid(self.linux_pid, "TERM")
            if not native._wait_wsl_pid_exit(self.linux_pid, 2.0):
                native._signal_wsl_pid(self.linux_pid, "KILL")
                native._wait_wsl_pid_exit(self.linux_pid, 1.0)
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1.0)
        if self.process is not None and self.process.stdin is not None:
            try:
                self.process.stdin.close()
            except OSError:
                pass
        if self._log is not None:
            self._log.close()
            self._log = None
        alive_after = native._wsl_pid_alive(self.linux_pid)
        return {
            "linux_pid": self.linux_pid,
            "pid_file": str(self.pid_path or ""),
            "alive_before_cleanup": alive_before,
            "alive_after_cleanup": alive_after,
            "clean": not alive_after,
        }

    def close(self, reason: str) -> Mapping[str, Any]:
        if self.process is None:
            return {
                "ok": False,
                "returncode": -1,
                "reason": "stream_not_started",
                "typed_stop": False,
            }
        error = ""
        graceful = False
        terminal_reason = str(reason)
        try:
            if self.process.poll() is None and self.process.stdin is not None:
                safe_reason = str(reason or "mujoco_wasd_exit").strip().replace(" ", "_")
                self.process.stdin.write(f"quit {safe_reason}\n")
                self.process.stdin.flush()
                self.process.stdin.close()
                returncode = int(self.process.wait(timeout=max(3.0, self.timeout_ms / 1000.0 + 2.0)))
                graceful = returncode == 0
            else:
                returncode = int(self.process.poll() if self.process.poll() is not None else -1)
                if returncode == 0 and TELEOP_STREAM_TIMEOUT_STOP_MARKER in self.tail():
                    graceful = True
                    terminal_reason = "teleop_stream_input_timeout"
        except (BrokenPipeError, OSError, subprocess.TimeoutExpired) as exc:
            error = f"{type(exc).__name__}:{exc}"
            returncode = int(self.process.poll() if self.process.poll() is not None else -1)
        cleanup = self._force_cleanup()
        self.ready_path.unlink(missing_ok=True)
        return {
            "ok": graceful and bool(cleanup.get("clean")),
            "returncode": returncode,
            "reason": terminal_reason,
            "typed_stop": graceful,
            "samples": self.samples,
            "transport": "persistent_typed_dds_navigation_command",
            "error": error,
            "cleanup": cleanup,
        }


@dataclass
class ContinuousTypedTeleop:
    binary: Path
    domain_id: int
    log_dir: Path
    rate_hz: float = 10.0
    timeout_ms: int = 3000
    input_timeout_ms: int = 350
    stream_factory: Callable[..., TeleopStreamLike] = NativeTeleopStream
    stop_sender: Callable[[str], Mapping[str, Any]] | None = None
    _stream: TeleopStreamLike | None = field(default=None, init=False)
    _twist: TeleopTwist | None = field(default=None, init=False)
    startup_latency_ms: float | None = field(default=None, init=False)
    transitions: list[dict[str, Any]] = field(default_factory=list, init=False)

    def apply(self, twist: TeleopTwist) -> None:
        if self._stream is None:
            startup_started_s = time.monotonic()
            self._stream = self.stream_factory(
                binary=self.binary,
                domain_id=self.domain_id,
                log_path=self.log_dir / "teleop_command_current.log",
                ready_path=self.log_dir / "teleop_command_current.ready",
                rate_hz=self.rate_hz,
                timeout_ms=self.timeout_ms,
                input_timeout_ms=self.input_timeout_ms,
            )
            self._stream.start()
            self.startup_latency_ms = (time.monotonic() - startup_started_s) * 1000.0
        changed = twist != self._twist
        if self._stream.poll() is not None:
            raise RuntimeError(f"typed teleop stream exited early: {self._stream.poll()}")
        send_started_s = time.monotonic()
        self._stream.send(twist)
        send_latency_ms = (time.monotonic() - send_started_s) * 1000.0
        self._twist = twist
        if not changed:
            return
        self.transitions.append(
            {
                "wall_s": time.time(),
                "vx": twist.vx,
                "vy": twist.vy,
                "wz": twist.wz,
                "manual_mode": twist.manual_mode,
                "transport": "persistent_typed_dds_navigation_command",
                "stdin_write_latency_ms": send_latency_ms,
            }
        )

    def check(self) -> None:
        if self._stream is None:
            return
        returncode = self._stream.poll()
        if returncode is not None:
            raise RuntimeError(f"typed teleop stream exited early: {returncode}")

    def poll(self) -> int | None:
        return self._stream.poll() if self._stream is not None else None

    def close(self, reason: str = "mujoco_wasd_exit") -> Mapping[str, Any]:
        stream = self._stream
        self._stream = None
        self._twist = None
        stream_result: Mapping[str, Any] = (
            stream.close(reason)
            if stream is not None
            else {"ok": False, "returncode": -1, "reason": "stream_not_started"}
        )
        if bool(stream_result.get("ok")) and int(stream_result.get("returncode") or 0) == 0:
            return stream_result
        if self.stop_sender is None:
            return stream_result
        fallback = self.stop_sender(reason)
        return {
            **dict(fallback),
            "ok": False,
            "fallback_stop_used": True,
            "fallback_stop": dict(fallback),
            "stream_stop": dict(stream_result),
        }


def build_interactive_plan(
    *,
    scenario: str,
    domain_id: int,
    binaries: Mapping[str, Path],
    paths: Mapping[str, Path],
    case_dir: Path,
    duration_s: float,
    warmup_s: float,
    manifest: Mapping[str, Any],
    viewer: bool,
    viewer_hz: float,
    state_provider: str = "mujoco_fixture",
) -> dict[str, Any]:
    if state_provider not in STATE_PROVIDERS:
        raise ValueError(f"unsupported state provider: {state_provider}")
    plan_manifest = dict(manifest)
    if state_provider == "mujoco_fixture":
        slam_runtime = dict(plan_manifest.get("slam_runtime") or {})
        slam_runtime.update(provider="mujoco_navigation_fixture", mode="mapping")
        plan_manifest["slam_runtime"] = slam_runtime
    if viewer:
        driver_bridge = dict(plan_manifest.get("driver_bridge") or {})
        for name in ("command_timeout_ms", "heartbeat_timeout_ms", "apply_timeout_ms"):
            value = driver_bridge.get(name)
            if value is None:
                driver_bridge[name] = VIEWER_DRIVER_TIMEOUT_MS
            elif isinstance(value, int) and not isinstance(value, bool):
                driver_bridge[name] = max(value, VIEWER_DRIVER_TIMEOUT_MS)
        plan_manifest["driver_bridge"] = driver_bridge
        tolerances = dict(plan_manifest.get("runtime_tolerances") or {})
        if tolerances.get("sensor_publisher_write_mode") == "async_fifo":
            for name, minimum in VIEWER_QUEUE_LIMITS.items():
                tolerances[name] = max(type(minimum)(tolerances.get(name, minimum)), minimum)
            plan_manifest["runtime_tolerances"] = tolerances
    plan = acceptance.build_execution_plan(
        scenario=scenario,
        domain_id=domain_id,
        binaries=binaries,
        paths=paths,
        case_dir=case_dir,
        duration_s=duration_s,
        warmup_s=warmup_s,
        command_vx=0.0,
        manifest=plan_manifest,
    )
    if scenario == "obstacle_stop" and state_provider == "mujoco_fixture":
        plan["scene_variant"] = "obstacle_stop_demo"
    sensor = next(item for item in plan["processes"] if item["name"] == "sensor")
    command = list(sensor["command"])
    for option in ("--motion-log", "--motion-log-hz", "--motion-log-lidar-points"):
        if option in command:
            index = command.index(option)
            del command[index : index + 2]
    require_nonzero_cmd_vel = scenario not in {"obstacle_stop", "terrain_hard"}
    if require_nonzero_cmd_vel and "--require-cmd-vel" not in command:
        command.append("--require-cmd-vel")
    if viewer:
        command.extend(["--viewer", "--viewer-hz", str(max(1.0, float(viewer_hz)))])
    sensor["command"] = command
    plan.pop("teleop_command", None)
    plan["interactive_control"] = {
        "ingress": "lingtu_nav_control typed /nav/command/request",
        "final_output": "native endpoint /nav/cmd_vel -> MuJoCo physical driver bridge",
        "direct_mujoco_control": False,
        "viewer": bool(viewer),
        "state_provider": state_provider,
        "product_slam_evaluated": state_provider == "fastlio2",
        "require_nonzero_cmd_vel": require_nonzero_cmd_vel,
    }
    return plan


def _format_status(path: Path, requested: TeleopTwist) -> str:
    nav = native._load_json(path)
    teleop = nav.get("teleop") or {}
    output = teleop.get("output") or nav.get("final_cmd_vel") or {}
    gate = nav.get("input_gate") or {}
    authority = nav.get("control_authority") or {}
    loop_health = nav.get("control_loop_health") or {}
    reason = str(teleop.get("reason") or (nav.get("last_local") or {}).get("reason") or "waiting")
    obstacle = teleop.get("obstacle_distance_m")
    terrain = teleop.get("traversability_cost")
    resume = "  resume=R" if bool(authority.get("resume_required")) else ""
    manual = "  manual=ON" if requested.manual_mode else ""
    return (
        f"intent=({requested.vx:+.2f},{requested.vy:+.2f},{requested.wz:+.2f})  "
        f"final=({float(output.get('vx') or 0.0):+.2f},"
        f"{float(output.get('vy') or 0.0):+.2f},"
        f"{float(output.get('wz') or 0.0):+.2f})  "
        f"gate={str(gate.get('reason') or 'unknown')}  reason={reason}{resume}{manual}  "
        f"loop={str(loop_health.get('reason') or 'warming_up')}  "
        f"obstacle={obstacle if obstacle is not None else '--'}  "
        f"terrain={terrain if terrain is not None else '--'}"
    )


def _record_runtime_evidence(
    path: Path,
    requested: TeleopTwist,
    evidence: dict[str, Any],
) -> None:
    nav = native._load_json(path)
    teleop = nav.get("teleop") or {}
    output = teleop.get("output") or nav.get("final_cmd_vel") or {}
    reason = str(teleop.get("reason") or (nav.get("last_local") or {}).get("reason") or "")
    reason_counts = evidence.setdefault("teleop_reason_counts", {})
    if reason:
        reason_counts[reason] = int(reason_counts.get(reason) or 0) + 1
    output_norm = math.sqrt(
        float(output.get("vx") or 0.0) ** 2
        + float(output.get("vy") or 0.0) ** 2
        + float(output.get("wz") or 0.0) ** 2
    )
    evidence["samples"] = int(evidence.get("samples") or 0) + 1
    evidence["gate_ready_samples"] = int(evidence.get("gate_ready_samples") or 0) + int(
        bool((nav.get("input_gate") or {}).get("ready"))
    )
    evidence["motion_intent_samples"] = int(evidence.get("motion_intent_samples") or 0) + int(
        not requested.is_zero()
    )
    evidence["manual_mode_samples"] = int(evidence.get("manual_mode_samples") or 0) + int(
        requested.manual_mode
    )
    evidence["final_nonzero_samples"] = int(evidence.get("final_nonzero_samples") or 0) + int(
        output_norm > 1e-4
    )
    evidence["max_final_cmd_norm"] = max(float(evidence.get("max_final_cmd_norm") or 0.0), output_norm)
    obstacle = teleop.get("obstacle_distance_m")
    if obstacle is not None and float(obstacle) >= 0.0:
        current = evidence.get("min_obstacle_distance_m")
        evidence["min_obstacle_distance_m"] = (
            float(obstacle) if current is None else min(float(current), float(obstacle))
        )
    terrain = teleop.get("traversability_cost")
    if terrain is not None and float(terrain) >= 0.0:
        evidence["max_traversability_cost"] = max(
            float(evidence.get("max_traversability_cost") or 0.0),
            float(terrain),
        )


def _isolated_domain_id(value: str) -> int:
    domain_id = int(value)
    if not MIN_ISOLATED_DDS_DOMAIN_ID <= domain_id <= MAX_CYCLONEDDS_UDP_DOMAIN_ID:
        raise argparse.ArgumentTypeError(
            "MuJoCo DDS domain must be isolated in the range "
            f"{MIN_ISOLATED_DDS_DOMAIN_ID}-{MAX_CYCLONEDDS_UDP_DOMAIN_ID}"
        )
    return domain_id


def _clean_viewer_exit(sensor: Any, sensor_report_path: Path) -> bool:
    """Return true only when the sensor confirms a user-closed passive viewer."""

    if sensor.poll() != 0:
        return False
    sensor_report = native._load_json(sensor_report_path)
    viewer = sensor_report.get("viewer") or {}
    return bool(
        sensor_report.get("ok") is True
        and viewer.get("enabled") is True
        and viewer.get("closed_early") is True
    )


def _wait_for_interactive_runtime_ready(
    *,
    sensor: Any,
    nav_status: Path,
    slam_status: Path,
    traversability_status: Path,
    state_provider: str,
    timeout_s: float,
) -> tuple[bool, str]:
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    while time.monotonic() < deadline:
        if sensor.poll() is not None:
            return False, "sensor_runner_exited_before_runtime_ready"
        nav = native._load_json(nav_status)
        traversability = native._load_json(traversability_status)
        gate_ready = bool((nav.get("input_gate") or {}).get("ready"))
        state_ready = bool(nav.get("has_odom"))
        if state_provider == "fastlio2":
            slam = native._load_json(slam_status)
            state_ready = state_ready and str(slam.get("state") or "").upper() == "TRACKING"
        if (
            state_ready
            and str(nav.get("control_mode") or "") == "teleop_avoid"
            and gate_ready
            and int((traversability.get("counters") or {}).get("published") or 0) > 0
        ):
            return True, f"ready_{state_provider}"
        time.sleep(0.1)
    return False, "native_runtime_startup_timeout"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=acceptance.DEFAULT_MANIFEST)
    parser.add_argument("--artifact-dir", type=Path, default=DEFAULT_ARTIFACT_DIR)
    parser.add_argument("--scenario", choices=INTERACTIVE_SCENARIOS, default="obstacle_stop")
    parser.add_argument(
        "--state-provider",
        choices=STATE_PROVIDERS,
        default="mujoco_fixture",
        help=(
            "mujoco_fixture isolates local avoidance with typed DDS ground-truth pose; "
            "fastlio2 also validates the full simulated SLAM chain"
        ),
    )
    parser.add_argument("--domain-id", type=_isolated_domain_id, default=231)
    parser.add_argument("--duration-s", type=float, default=600.0)
    parser.add_argument("--warmup-s", type=float, default=15.0)
    parser.add_argument("--startup-timeout-s", type=float, default=60.0)
    parser.add_argument("--linear-speed", type=float, default=0.18)
    parser.add_argument("--lateral-speed", type=float, default=0.15)
    parser.add_argument("--yaw-speed", type=float, default=0.35)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument(
        "--input-timeout-ms",
        type=int,
        default=350,
        help=(
            "Send typed zero plus stop and end the native stream if the Python "
            "keyboard heartbeat is silent this long."
        ),
    )
    parser.add_argument("--deadman", choices=("shift", "none"), default="shift")
    parser.add_argument("--viewer", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--viewer-hz", type=float, default=30.0)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--json-out", type=Path, default=None)
    return parser


def run(
    args: argparse.Namespace,
    *,
    key_poller: KeyPoller | None = None,
    stream_factory: Callable[..., TeleopStreamLike] = NativeTeleopStream,
) -> dict[str, Any]:
    artifact_dir = Path(args.artifact_dir).expanduser().resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    preflight_state_provider = (
        "mujoco_navigation_fixture"
        if str(args.state_provider) == "mujoco_fixture"
        else str(args.state_provider)
    )
    preflight_args = acceptance.build_parser().parse_args(
        [
            "--manifest",
            str(Path(args.manifest).expanduser().resolve()),
            "--artifact-dir",
            str(artifact_dir),
            "--scenario",
            str(args.scenario),
            "--state-provider",
            preflight_state_provider,
            "--preflight-only",
        ]
    )
    prepared = acceptance.prepare_runtime(preflight_args)
    report: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "ok": False,
        "scenario": str(args.scenario),
        "domain_id": int(args.domain_id),
        "state_provider": {
            "name": str(args.state_provider),
            "evidence_scope": (
                "local_planner_simulation_fixture"
                if str(args.state_provider) == "mujoco_fixture"
                else "product_slam_e2e"
            ),
            "product_slam_evaluated": str(args.state_provider) == "fastlio2",
        },
        "control_chain": [
            "WASD operator intent",
            "typed /nav/command/request + application ACK",
            "native LocalPlanner::planIntent",
            "native PathFollower",
            "native final safety",
            "typed /nav/cmd_vel",
            "MuJoCo physical driver bridge",
            "MuJoCo ThunderV4 policy",
        ],
        "preflight": prepared.get("details") or {},
        "blockers": list(prepared.get("blockers") or []),
    }
    if prepared.get("ok") is not True or bool(args.preflight_only):
        report["ok"] = prepared.get("ok") is True
        report["preflight_only"] = True
        return report

    binaries = {name: Path(path) for name, path in (prepared.get("binaries") or {}).items()}
    paths = {name: Path(path) for name, path in (prepared.get("paths") or {}).items()}
    case_dir = artifact_dir / "interactive" / str(args.scenario)
    plan = build_interactive_plan(
        scenario=str(args.scenario),
        domain_id=int(args.domain_id),
        binaries=binaries,
        paths=paths,
        case_dir=case_dir,
        duration_s=max(1.0, float(args.duration_s)),
        warmup_s=max(0.0, float(args.warmup_s)),
        manifest=dict(prepared.get("manifest") or {}),
        viewer=bool(args.viewer),
        viewer_hz=float(args.viewer_hz),
        state_provider=str(args.state_provider),
    )
    arm_contract = dict(plan.get("external_arm") or {})
    artifacts = {name: Path(value) for name, value in plan["artifacts"].items()}
    domain_tokens = ["--domain-id", str(int(args.domain_id))]
    cleanup_specs = {
        "prior_traversability": [Path(binaries["traversability"]).name, *domain_tokens],
        "prior_navigation": [Path(binaries["navigation"]).name, *domain_tokens],
        "prior_teleop_command": [Path(binaries["navigation_control"]).name, "teleop", *domain_tokens],
        "prior_teleop_command_current": [
            Path(binaries["navigation_control"]).name,
            "teleop",
            *domain_tokens,
        ],
        "prior_sensor_publisher": [Path(binaries["sensor_publisher"]).name, *domain_tokens],
        "prior_driver_bridge": [
            Path(binaries["driver_bridge"]).name,
            *domain_tokens,
        ],
    }
    if "slam" in binaries:
        cleanup_specs["prior_slam"] = [Path(binaries["slam"]).name, *domain_tokens]
    prior_cleanup = acceptance.reclaim_prior_case_processes(
        case_dir,
        artifacts,
        cleanup_specs,
    )
    if not all(bool(item.get("clean")) for item in prior_cleanup):
        report["blockers"] = [*report["blockers"], "prior_owned_process_cleanup_failed"]
        report["prior_process_cleanup"] = prior_cleanup
        return report
    acceptance.reset_case_artifacts(artifacts)
    acceptance.build_scene_variant(paths["world"], artifacts["scene"], str(plan["scene_variant"]))
    processes = [
        native.ManagedProcess(
            str(item["name"]),
            list(item["command"]),
            Path(str(item["log"])),
            env=item.get("env"),
        )
        for item in plan["processes"]
    ]
    by_name = {process.name: process for process in processes}
    poller = key_poller
    command_state = KeyboardCommandState(
        linear_speed_mps=float(args.linear_speed),
        lateral_speed_mps=float(args.lateral_speed),
        yaw_speed_rad_s=float(args.yaw_speed),
        require_deadman=str(args.deadman) == "shift",
    )
    input_timeout_ms = _input_timeout(int(args.input_timeout_ms), viewer=bool(args.viewer))

    def send_stop(reason: str) -> Mapping[str, Any]:
        return acceptance._run_control(
            binaries["navigation_control"],
            ["stop", reason],
            domain_id=int(args.domain_id),
            timeout_s=8.0,
        )

    teleop = ContinuousTypedTeleop(
        binary=binaries["navigation_control"],
        domain_id=int(args.domain_id),
        log_dir=case_dir,
        rate_hz=float(args.rate_hz),
        input_timeout_ms=input_timeout_ms,
        stream_factory=stream_factory,
        stop_sender=send_stop,
    )
    startup_ok = False
    startup_reason = "not_started"
    runtime_error = ""
    external_arm: dict[str, Any] = {}
    resume_attempts: list[dict[str, Any]] = []
    viewer_closed_by_user = False
    stop_result: Mapping[str, Any] = {}
    process_cleanup: list[dict[str, Any]] = []
    requested = TeleopTwist()
    runtime_evidence: dict[str, Any] = {
        "samples": 0,
        "gate_ready_samples": 0,
        "motion_intent_samples": 0,
        "manual_mode_samples": 0,
        "final_nonzero_samples": 0,
        "max_final_cmd_norm": 0.0,
        "min_obstacle_distance_m": None,
        "max_traversability_cost": 0.0,
        "teleop_reason_counts": {},
    }
    try:
        if poller is None:
            poller = create_key_poller()
        for process in processes:
            process.start()
        startup_ok, startup_reason = _wait_for_interactive_runtime_ready(
            sensor=by_name["sensor"],
            nav_status=artifacts["nav_status"],
            slam_status=artifacts["slam_status"],
            traversability_status=artifacts["traversability_status"],
            state_provider=str(args.state_provider),
            timeout_s=float(args.startup_timeout_s),
        )
        if not startup_ok:
            raise RuntimeError(startup_reason)
        teleop.apply(TeleopTwist())
        if arm_contract.get("required") is True:
            trigger = acceptance.trigger_external_arm(
                artifacts["sensor_arm"],
                token=str(arm_contract.get("token") or ""),
                domain_id=int(args.domain_id),
                scenario=str(args.scenario),
            )
            arm_ok, arm_reason, arm_evidence = acceptance._wait_for_external_arm_ack(
                sensor=by_name["sensor"],
                teleop=teleop,
                status_path=artifacts["sensor_arm_status"],
                domain_id=int(args.domain_id),
                scenario=str(args.scenario),
                not_before_ns=int(trigger["not_before_ns"]),
                timeout_s=float(args.startup_timeout_s),
            )
            external_arm = {
                "trigger": trigger,
                "ack": {
                    "ok": arm_ok,
                    "reason": arm_reason,
                    "evidence": arm_evidence,
                },
            }
            if not arm_ok:
                raise RuntimeError(arm_reason)
        poller.start()
        deadman_text = "Hold SHIFT + " if command_state.require_deadman else ""
        print(
            f"\nNative MuJoCo teleop_avoid ready. {deadman_text}WASD move, Q/E yaw, "
            "hold M for manual escape, SPACE stop, R resume, ESC exit.\n"
        )
        deadline = time.monotonic() + max(1.0, float(args.duration_s))
        next_status_s = 0.0
        previous_keys: set[str] = set()
        while time.monotonic() < deadline:
            exited = [process for process in processes if process.poll() is not None]
            if exited:
                if (
                    exited == [by_name["sensor"]]
                    and bool(args.viewer)
                    and _clean_viewer_exit(by_name["sensor"], artifacts["sensor_report"])
                ):
                    viewer_closed_by_user = True
                    break
                raise RuntimeError(
                    "native runtime process exited: "
                    + ",".join(f"{process.name}={process.poll()}" for process in exited)
                )
            keys = poller.snapshot()
            if "escape" in keys:
                break
            if _pressed_once(keys, previous_keys, "r"):
                teleop.apply(TeleopTwist())
                resume_result = dict(
                    _send_resume(binaries["navigation_control"], int(args.domain_id))
                )
                resume_attempts.append(resume_result)
                resume_text = str(resume_result.get("stdout") or resume_result.get("stderr") or "")
                print(f"\nresume: {resume_text.strip() or resume_result.get('returncode')}\n")
            requested = command_state.command(keys)
            teleop.apply(requested)
            teleop.check()
            previous_keys = keys
            now_s = time.monotonic()
            if now_s >= next_status_s:
                _record_runtime_evidence(artifacts["nav_status"], requested, runtime_evidence)
                print("\r" + _format_status(artifacts["nav_status"], requested)[:220].ljust(220), end="", flush=True)
                next_status_s = now_s + 0.20
            time.sleep(0.05)
    except KeyboardInterrupt:
        runtime_error = "keyboard_interrupt"
    except Exception as exc:
        runtime_error = f"{type(exc).__name__}:{exc}"
    finally:
        print()
        try:
            if poller is not None:
                poller.close()
        except Exception:
            pass
        try:
            stop_result = teleop.close("mujoco_wasd_exit")
        except Exception as exc:
            runtime_error = runtime_error or f"stop_failed:{type(exc).__name__}:{exc}"
        for process in reversed(processes):
            process.stop()
            process_cleanup.append(dict(process.cleanup))
        for name, key in (
            ("sensor_publisher", "sensor_publisher_pid"),
            ("driver_bridge", "driver_bridge_pid"),
        ):
            path = artifacts.get(key)
            if path is not None:
                process_cleanup.append(acceptance.cleanup_owned_pid_file(name, path))

    blockers = list(report["blockers"])
    if not startup_ok:
        blockers.append(startup_reason)
    if runtime_error and runtime_error != "keyboard_interrupt":
        blockers.append(runtime_error)
    if any(item.get("clean") is not True for item in process_cleanup):
        blockers.append("owned_process_cleanup_failed")
    if stop_result and int(stop_result.get("returncode") or 0) != 0:
        blockers.append("typed_stop_failed")
    if bool(stop_result.get("fallback_stop_used")):
        blockers.append("teleop_stream_stop_failed_fallback_used")
    report.update(
        {
            "ok": not blockers,
            "blockers": list(dict.fromkeys(blockers)),
            "startup": {"ok": startup_ok, "reason": startup_reason},
            "external_arm": external_arm,
            "resume_attempts": resume_attempts,
            "teleop_input_timeout_ms": input_timeout_ms,
            "runtime_error": runtime_error,
            "viewer_closed_by_user": viewer_closed_by_user,
            "requested_last": requested.__dict__,
            "runtime_evidence": runtime_evidence,
            "typed_stop": dict(stop_result),
            "stream_startup_latency_ms": teleop.startup_latency_ms,
            "transitions": teleop.transitions,
            "process_cleanup": process_cleanup,
            "prior_process_cleanup": prior_cleanup,
            "artifacts": {name: str(path) for name, path in artifacts.items()},
        }
    )
    return report


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(list(argv) if argv is not None else None)
    report = run(args)
    output = (
        Path(args.json_out).expanduser().resolve()
        if args.json_out
        else Path(args.artifact_dir).expanduser().resolve() / "wasd_report.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({"ok": report["ok"], "blockers": report["blockers"], "report": str(output)}, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

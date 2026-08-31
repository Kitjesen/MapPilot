"""Run-bound UE-input to MuJoCo-truth locomotion diagnostic gate."""

from __future__ import annotations

import json
import math
import os
import subprocess
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from sim.runtime.coordinator.atomic_file import replace_file_with_retry

_SUMMARY_FILENAME = "manual-locomotion-gate.json"
_PROBE_FILENAME = "runtime-locomotion-probe.json"
_RUNTIME_FILENAME = "session.runtime.json"
_INVOCATION_FILENAME = ".manual-locomotion-gate.invoked.json"


class _CommandResult(Protocol):
    returncode: int
    stdout: str
    stderr: str


CommandRunner = Callable[..., _CommandResult]
PresenceProof = Callable[[int], object]
ReleaseInput = Callable[[], object]


@dataclass(frozen=True)
class ManualLocomotionGateConfig:
    """Inputs for one bounded, non-qualification locomotion probe."""

    run_dir: Path
    run_id: str
    session_id: str
    unreal_pid: int
    profile_deadline_monotonic: float
    drive_probe_script: Path
    input_method: str = "ForegroundInput"
    hold_milliseconds: int = 3000
    release_settle_milliseconds: int = 800
    startup_timeout_s: float = 20.0
    completion_buffer_s: float = 2.0
    minimum_probe_budget_s: float = 12.0
    powershell_executable: str = "powershell"

    def __post_init__(self) -> None:
        run_dir = _canonical_directory(self.run_dir, "run_dir")
        probe_script = _canonical_file(self.drive_probe_script, "drive_probe_script")
        if not self.run_id.strip():
            raise ValueError("run_id must be non-empty")
        if not self.session_id.strip():
            raise ValueError("session_id must be non-empty")
        if isinstance(self.unreal_pid, bool) or not isinstance(self.unreal_pid, int):
            raise TypeError("unreal_pid must be an integer")
        if self.unreal_pid <= 0:
            raise ValueError("unreal_pid must be positive")
        for value, name in (
            (self.profile_deadline_monotonic, "profile_deadline_monotonic"),
            (self.startup_timeout_s, "startup_timeout_s"),
            (self.completion_buffer_s, "completion_buffer_s"),
            (self.minimum_probe_budget_s, "minimum_probe_budget_s"),
        ):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TypeError(f"{name} must be numeric")
            if not math.isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        for value, name in (
            (self.hold_milliseconds, "hold_milliseconds"),
            (self.release_settle_milliseconds, "release_settle_milliseconds"),
        ):
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{name} must be an integer")
            if value <= 0:
                raise ValueError(f"{name} must be positive")
        if not self.powershell_executable.strip():
            raise ValueError("powershell_executable must be non-empty")
        if self.input_method not in {"ForegroundInput", "TargetedMessages"}:
            raise ValueError(
                "input_method must be ForegroundInput or TargetedMessages"
            )
        object.__setattr__(self, "run_dir", run_dir)
        object.__setattr__(self, "drive_probe_script", probe_script)


@dataclass(frozen=True)
class ManualLocomotionGateResult:
    """Terminal evidence paths for one locomotion gate invocation."""

    passed: bool
    summary_path: Path
    probe_path: Path | None
    probe_invocation_count: int


def build_locomotion_probe_command(
    config: ManualLocomotionGateConfig,
) -> tuple[str, ...]:
    """Build the exact physical foreground-input probe command."""

    if not isinstance(config, ManualLocomotionGateConfig):
        raise TypeError("config must be ManualLocomotionGateConfig")
    return (
        config.powershell_executable,
        "-NoLogo",
        "-NoProfile",
        "-NonInteractive",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        str(config.drive_probe_script),
        "-ProcessId",
        str(config.unreal_pid),
        "-HoldMilliseconds",
        str(config.hold_milliseconds),
        "-ReleaseSettleMilliseconds",
        str(config.release_settle_milliseconds),
        "-InputMethod",
        config.input_method,
        "-RunDirectory",
        str(config.run_dir),
        "-ExpectedRunId",
        config.run_id,
        "-ProductSessionId",
        config.session_id,
        "-SnapshotPath",
        str(config.run_dir / "truth-snapshots.jsonl"),
        "-RequireLocomotion",
        *(
            ("-AllowTargetedLocomotionDiagnostic",)
            if config.input_method == "TargetedMessages"
            else ()
        ),
    )


def supervise_manual_locomotion_gate(
    config: ManualLocomotionGateConfig,
    *,
    prove_presence: PresenceProof,
    command_runner: CommandRunner = subprocess.run,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], object] = time.sleep,
    release_input: ReleaseInput | None = None,
) -> ManualLocomotionGateResult:
    """Wait for the owned UE window, run one probe, and preserve its truth verdict."""

    if not isinstance(config, ManualLocomotionGateConfig):
        raise TypeError("config must be ManualLocomotionGateConfig")
    if not callable(prove_presence) or not callable(command_runner):
        raise TypeError("prove_presence and command_runner must be callable")
    release_keys = _release_drive_keys if release_input is None else release_input
    if not callable(release_keys):
        raise TypeError("release_input must be callable")
    summary_path = config.run_dir / _SUMMARY_FILENAME
    probe_path = config.run_dir / _PROBE_FILENAME
    _reserve_invocation(config)
    if summary_path.exists() or probe_path.exists():
        raise RuntimeError("locomotion gate artifacts already exist in this run directory")

    trigger_deadline = min(
        config.profile_deadline_monotonic - config.completion_buffer_s,
        _finite_now(monotonic) + config.startup_timeout_s,
    )
    readiness_error = "runtime or owned RobotSimUE window is not ready"
    while _finite_now(monotonic) < trigger_deadline:
        runtime_ready, runtime_reason = _runtime_is_current_and_running(config)
        if runtime_ready:
            try:
                prove_presence(config.unreal_pid)
                break
            except Exception as exc:  # window appears asynchronously during UE startup
                readiness_error = _bounded_error(exc)
        else:
            readiness_error = runtime_reason
        sleep(0.05)
    else:
        return _write_result(
            config,
            summary_path=summary_path,
            probe_path=None,
            probe=None,
            returncode=None,
            invocation_count=0,
            passed=False,
            failure_reason=f"locomotion gate readiness timed out: {readiness_error}",
        )

    remaining = config.profile_deadline_monotonic - _finite_now(monotonic)
    available_probe_s = remaining - config.completion_buffer_s
    if available_probe_s < config.minimum_probe_budget_s:
        return _write_result(
            config,
            summary_path=summary_path,
            probe_path=None,
            probe=None,
            returncode=None,
            invocation_count=0,
            passed=False,
            failure_reason="profile deadline cannot safely cover the locomotion probe",
        )

    command = build_locomotion_probe_command(config)
    completed: _CommandResult | None = None
    execution_error: BaseException | None = None
    release_error: BaseException | None = None
    try:
        completed = command_runner(
            command,
            cwd=config.run_dir,
            capture_output=True,
            text=True,
            timeout=available_probe_s,
            check=False,
        )
    except Exception as exc:
        execution_error = exc
    finally:
        try:
            release_keys()
        except Exception as exc:
            release_error = exc

    if execution_error is not None or release_error is not None:
        reason_parts: list[str] = []
        if execution_error is not None:
            reason_parts.append(
                f"locomotion probe execution failed: {_bounded_error(execution_error)}"
            )
        if release_error is not None:
            reason_parts.append(
                f"parent key-release cleanup failed: {_bounded_error(release_error)}"
            )
        return _write_result(
            config,
            summary_path=summary_path,
            probe_path=None,
            probe=None,
            returncode=None,
            invocation_count=1,
            passed=False,
            failure_reason="; ".join(reason_parts),
        )
    if completed is None:
        raise RuntimeError("locomotion probe returned no command result")

    try:
        probe = _strict_probe(json.loads(completed.stdout), config)
    except (json.JSONDecodeError, TypeError, ValueError) as exc:
        return _write_result(
            config,
            summary_path=summary_path,
            probe_path=None,
            probe=None,
            returncode=completed.returncode,
            invocation_count=1,
            passed=False,
            failure_reason=f"locomotion probe artifact was invalid: {_bounded_error(exc)}",
            probe_stderr=_bounded_text(completed.stderr),
        )

    _atomic_write_json(probe_path, probe)
    passed = completed.returncode == 0 and _probe_passed(probe, config)
    failure_reason = None
    if not passed:
        failure_reason = "locomotion probe did not pass all physical gates"
    return _write_result(
        config,
        summary_path=summary_path,
        probe_path=probe_path,
        probe=probe,
        returncode=completed.returncode,
        invocation_count=1,
        passed=passed,
        failure_reason=failure_reason,
        probe_stderr=_bounded_text(completed.stderr),
    )


def _runtime_is_current_and_running(
    config: ManualLocomotionGateConfig,
) -> tuple[bool, str]:
    path = config.run_dir / _RUNTIME_FILENAME
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return False, "session.runtime.json is not readable"
    if not isinstance(document, Mapping):
        return False, "session.runtime.json is not an object"
    if document.get("run_id") != config.run_id:
        return False, "session.runtime.json run_id is not current"
    if document.get("session_id") != config.session_id:
        return False, "session.runtime.json session_id is not current"
    if document.get("state") != "RUNNING":
        return False, "session.runtime.json is not RUNNING"
    return True, ""


def _strict_probe(
    value: object,
    config: ManualLocomotionGateConfig,
) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise TypeError("probe output must be one JSON object")
    probe = dict(value)
    expected: Sequence[tuple[str, object]] = (
        ("schema", "lingtu.sim.debug.drive-input-probe.v3"),
        ("process_id", config.unreal_pid),
        ("input_method", config.input_method),
        ("run_directory", str(config.run_dir)),
        ("expected_run_id", config.run_id),
        ("expected_session_id", config.session_id),
        ("locomotion_qualification_required", True),
    )
    for name, expected_value in expected:
        if probe.get(name) != expected_value:
            raise ValueError(f"probe {name} does not match the current run")
    if not isinstance(probe.get("passed"), bool):
        raise TypeError("probe passed must be boolean")
    return probe


def _probe_passed(
    probe: Mapping[str, Any],
    config: ManualLocomotionGateConfig,
) -> bool:
    if probe.get("passed") is not True:
        return False
    window_handle = _positive_int(probe.get("window_handle"))
    if window_handle is None:
        return False
    if config.input_method == "ForegroundInput":
        if (
            probe.get("physical_foreground_input") is not True
            or _positive_int(probe.get("foreground_before_input")) != window_handle
            or _positive_int(probe.get("foreground_handle")) != window_handle
        ):
            return False
    elif probe.get("physical_foreground_input") is not False:
        return False
    if (
        probe.get("input_method") != config.input_method
        or probe.get("process_still_running") is not True
        or probe.get("command_accepted") is not True
        or probe.get("physics_motion_observed") is not True
        or probe.get("motion_observed") is not True
        or (_number(probe.get("max_observed_linear_speed_mps")) or 0.0) <= 0.01
        or (_positive_int(probe.get("new_received_intents")) or 0) <= 0
        or (_positive_int(probe.get("new_nonzero_accepted_commands")) or 0) <= 0
    ):
        return False
    if not _press_passed(probe.get("press")):
        return False
    if not _hold_passed(probe.get("hold")):
        return False
    if not _release_transport_passed(probe.get("release")):
        return False
    transport = probe.get("transport_probe")
    if not isinstance(transport, Mapping) or any(
        transport.get(name) is not True
        for name in (
            "passed",
            "input_received",
            "press",
            "hold",
            "release",
            "origin_correlation",
        )
    ):
        return False
    return (
        _identity_passed(probe.get("run_identity_qualification"))
        and _locomotion_passed(probe.get("locomotion_qualification"))
        and _release_stop_passed(probe.get("release_stop_qualification"))
        and _upright_passed(probe.get("upright_qualification"))
    )


def _press_passed(value: object) -> bool:
    return (
        isinstance(value, Mapping)
        and value.get("passed") is True
        and (_positive_int(value.get("accepted_zero_with_deadman")) or 0) > 0
        and (_positive_int(value.get("origin_matches")) or 0) > 0
    )


def _hold_passed(value: object) -> bool:
    if not isinstance(value, Mapping) or value.get("passed") is not True:
        return False
    samples = _positive_int(value.get("samples"))
    minimum_samples = _positive_int(value.get("minimum_samples"))
    window_ms = _number(value.get("window_milliseconds"))
    minimum_window_ms = _number(value.get("minimum_window_milliseconds"))
    return (
        samples is not None
        and minimum_samples is not None
        and samples >= minimum_samples >= 2
        and window_ms is not None
        and minimum_window_ms is not None
        and window_ms >= minimum_window_ms > 0.0
        and (_positive_int(value.get("origin_matches")) or 0) > 0
    )


def _release_transport_passed(value: object) -> bool:
    return isinstance(value, Mapping) and value.get("passed") is True and all(
        (_positive_int(value.get(name)) or 0) > 0
        for name in (
            "zero_while_deadman_held",
            "released_authority_statuses",
            "zero_audit_records",
            "origin_matches",
        )
    )


def _identity_passed(value: object) -> bool:
    return (
        _base_qualification_passed(value)
        and isinstance(value, Mapping)
        and (_positive_int(value.get("control_records")) or 0) > 0
        and (_positive_int(value.get("snapshot_documents")) or 0) > 0
        and value.get("failure_codes") == []
    )


def _locomotion_passed(value: object) -> bool:
    if not _base_qualification_passed(value) or not isinstance(value, Mapping):
        return False
    displacement = _number(value.get("position_displacement_m"))
    minimum = _number(value.get("minimum_position_displacement_m"))
    hold_start = _nonnegative_int(value.get("hold_start_time_ns"))
    hold_end = _nonnegative_int(value.get("hold_end_time_ns"))
    return (
        value.get("position_displacement_required") is True
        and displacement is not None
        and minimum is not None
        and displacement >= minimum > 0.0
        and (_positive_int(value.get("hold_samples")) or 0) >= 2
        and hold_start is not None
        and hold_end is not None
        and hold_end > hold_start
    )


def _release_stop_passed(value: object) -> bool:
    if not _base_qualification_passed(value) or not isinstance(value, Mapping):
        return False
    duration = _number(value.get("duration_seconds"))
    drift = _number(value.get("maximum_planar_drift_m"))
    drift_limit = _number(value.get("maximum_planar_drift_m_allowed"))
    speed = _number(value.get("final_planar_speed_mps"))
    speed_limit = _number(value.get("maximum_final_planar_speed_mps"))
    return (
        duration is not None
        and duration >= 0.5
        and drift is not None
        and drift_limit is not None
        and 0.0 <= drift <= drift_limit
        and speed is not None
        and speed_limit is not None
        and 0.0 <= speed <= speed_limit
        and _nonnegative_int(value.get("release_start_time_ns")) is not None
    )


def _upright_passed(value: object) -> bool:
    if not _base_qualification_passed(value) or not isinstance(value, Mapping):
        return False
    comparisons = (
        ("minimum_base_height_m", "minimum_base_height_m_allowed", True),
        ("maximum_base_height_drop_m", "maximum_base_height_drop_m_allowed", False),
        ("minimum_body_up_dot", "minimum_body_up_dot_allowed", True),
        ("maximum_absolute_roll_rad", "maximum_absolute_roll_rad_allowed", False),
        ("maximum_absolute_pitch_rad", "maximum_absolute_pitch_rad_allowed", False),
    )
    for observed_name, limit_name, minimum_gate in comparisons:
        observed = _number(value.get(observed_name))
        limit = _number(value.get(limit_name))
        if observed is None or limit is None:
            return False
        if minimum_gate and observed < limit:
            return False
        if not minimum_gate and observed > limit:
            return False
    hold_samples = _positive_int(value.get("hold_samples"))
    minimum_hold = _positive_int(value.get("minimum_hold_samples"))
    release_samples = _positive_int(value.get("release_samples"))
    minimum_release = _positive_int(value.get("minimum_release_samples"))
    return (
        hold_samples is not None
        and minimum_hold is not None
        and hold_samples >= minimum_hold >= 2
        and release_samples is not None
        and minimum_release is not None
        and release_samples >= minimum_release >= 2
        and value.get("failure_codes") == []
    )


def _base_qualification_passed(value: object) -> bool:
    return (
        isinstance(value, Mapping)
        and value.get("state") == "passed"
        and value.get("passed") is True
    )


def _number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _positive_int(value: object) -> int | None:
    number = _nonnegative_int(value)
    return number if number is not None and number > 0 else None


def _nonnegative_int(value: object) -> int | None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None
    return value


def _write_result(
    config: ManualLocomotionGateConfig,
    *,
    summary_path: Path,
    probe_path: Path | None,
    probe: Mapping[str, Any] | None,
    returncode: int | None,
    invocation_count: int,
    passed: bool,
    failure_reason: str | None,
    probe_stderr: str | None = None,
) -> ManualLocomotionGateResult:
    summary = {
        "schema": "lingtu.sim.manual-locomotion-gate.v1",
        "run_id": config.run_id,
        "session_id": config.session_id,
        "unreal_pid": config.unreal_pid,
        "input_method": config.input_method,
        "physical_foreground_input": config.input_method == "ForegroundInput",
        "passed": passed,
        "failure_reason": failure_reason,
        "probe_invocation_count": invocation_count,
        "probe_returncode": returncode,
        "probe_stderr": probe_stderr,
        "probe_path": str(probe_path) if probe_path is not None else None,
        "probe": dict(probe) if probe is not None else None,
        "qualification": "manual_diagnostic_only",
    }
    _atomic_write_json(summary_path, summary)
    return ManualLocomotionGateResult(
        passed=passed,
        summary_path=summary_path,
        probe_path=probe_path,
        probe_invocation_count=invocation_count,
    )


def _reserve_invocation(config: ManualLocomotionGateConfig) -> None:
    path = config.run_dir / _INVOCATION_FILENAME
    document = {
        "schema": "lingtu.sim.manual-locomotion-invocation.v1",
        "run_id": config.run_id,
        "session_id": config.session_id,
        "unreal_pid": config.unreal_pid,
    }
    try:
        with path.open("x", encoding="utf-8") as stream:
            json.dump(document, stream, ensure_ascii=False, sort_keys=True)
            stream.write("\n")
    except FileExistsError as exc:
        raise RuntimeError("locomotion gate input invocation is already reserved") from exc


def _release_drive_keys() -> None:
    """Release only the diagnostic keys from the parent process on Windows."""

    if os.name != "nt":
        return
    import ctypes

    key_event_key_up = 0x0002
    user32 = ctypes.WinDLL("user32", use_last_error=True)
    user32.keybd_event(0x57, 0x11, key_event_key_up, 0)
    user32.keybd_event(0x10, 0x2A, key_event_key_up, 0)


def _atomic_write_json(path: Path, document: Mapping[str, Any]) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(document, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
    )
    replace_file_with_retry(temporary, path)


def _canonical_directory(path: Path, name: str) -> Path:
    candidate = Path(path)
    if not candidate.is_absolute():
        raise ValueError(f"{name} must be absolute")
    resolved = candidate.resolve()
    if resolved != candidate or not resolved.is_dir():
        raise ValueError(f"{name} must resolve to its exact directory")
    return resolved


def _canonical_file(path: Path, name: str) -> Path:
    candidate = Path(path)
    if not candidate.is_absolute():
        raise ValueError(f"{name} must be absolute")
    resolved = candidate.resolve()
    if resolved != candidate or not resolved.is_file():
        raise ValueError(f"{name} must resolve to its exact regular file")
    return resolved


def _finite_now(monotonic: Callable[[], float]) -> float:
    value = monotonic()
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError("monotonic clock must return a numeric value")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("monotonic clock must return a finite value")
    return number


def _bounded_error(error: BaseException) -> str:
    message = " ".join(str(error).split())
    if len(message) > 240:
        message = message[:237] + "..."
    return f"{type(error).__name__}: {message}" if message else type(error).__name__


def _bounded_text(value: object, *, limit: int = 2000) -> str | None:
    if not isinstance(value, str):
        return None
    text = value.strip()
    if not text:
        return None
    return text if len(text) <= limit else text[-limit:]


__all__ = [
    "ManualLocomotionGateConfig",
    "ManualLocomotionGateResult",
    "build_locomotion_probe_command",
    "supervise_manual_locomotion_gate",
]

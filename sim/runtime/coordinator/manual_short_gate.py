"""One-shot, non-qualification supervisor for a manual SDK-quiet short gate."""

from __future__ import annotations

import json
import math
import os
import re
import subprocess
import tempfile
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

_RUN_ID_RE = re.compile(r"[A-Za-z0-9._-]+\Z")
_DIAGNOSTIC_CONTROL_RE = re.compile(r"[\x00-\x1f\x7f]+")
_UI_MARKER = "LINGTU_RUNTIME_UI_ATTACHED"
_CAMERA_STREAMS = {
    "thunder_01.front_rgb": "rgb",
    "thunder_01.front_depth": "depth",
}
_FORBIDDEN_UNREAL_MARKERS = (
    "ValidatePlatforms",
    "VerifySdk",
    "BuildCookRun",
    "RunUAT",
    "AutomationTool",
    "UnrealBuildTool",
    "Build.bat",
    "MSBuild.exe",
    "cl.exe",
    "link.exe",
    "CookCommandlet",
    "-run=Cook",
)
_SUMMARY_FILENAME = "manual-short-gate.json"
_WINDOW_PROOF_FILENAME = "manual-window-presence-proof.json"
_PERFORMANCE_PROBE_FILENAME = "runtime-performance-probe.json"
_MINIMUM_REALTIME_FACTOR = 0.8
_MINIMUM_CAMERA_RATE_HZ = 29.0
_MAX_DIAGNOSTIC_MESSAGE_CHARS = 512
_MAX_DIAGNOSTIC_CAUSE_DEPTH = 4


class ManualShortGateError(RuntimeError):
    """Raised internally when a short-gate contract fails closed."""


class _NotReady(ManualShortGateError):
    """A bounded readiness wait may retry this condition."""


class _CommandResult(Protocol):
    returncode: int
    stdout: str
    stderr: str


@dataclass(frozen=True)
class ManualShortGateConfig:
    """Exact immutable identity and timing budget for one diagnostic gate."""

    run_dir: Path
    run_id: str
    session_id: str
    unreal_pid: int
    profile_deadline_monotonic: float
    performance_probe_script: Path
    probe_duration_s: int = 12
    minimum_trigger_budget_s: float = 20.0
    probe_completion_buffer_s: float = 2.0
    readiness_timeout_s: float = 5.0
    poll_interval_s: float = 0.05
    maximum_health_age_s: float = 1.0
    powershell_executable: Path = Path(
        os.environ.get("SYSTEMROOT", r"C:\Windows")
    ) / "System32/WindowsPowerShell/v1.0/powershell.exe"

    def __post_init__(self) -> None:
        run_dir = _canonical_directory(self.run_dir, "run_dir")
        probe_script = _canonical_file(
            self.performance_probe_script,
            "performance_probe_script",
        )
        if _RUN_ID_RE.fullmatch(self.run_id) is None:
            raise ValueError("run_id must be canonical")
        if not self.session_id.strip():
            raise ValueError("session_id must be non-empty")
        if (
            isinstance(self.unreal_pid, bool)
            or not isinstance(self.unreal_pid, int)
            or self.unreal_pid <= 0
        ):
            raise ValueError("unreal_pid must be a positive integer")
        for value, label in (
            (self.profile_deadline_monotonic, "profile_deadline_monotonic"),
            (self.minimum_trigger_budget_s, "minimum_trigger_budget_s"),
            (self.probe_completion_buffer_s, "probe_completion_buffer_s"),
            (self.readiness_timeout_s, "readiness_timeout_s"),
            (self.poll_interval_s, "poll_interval_s"),
            (self.maximum_health_age_s, "maximum_health_age_s"),
        ):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TypeError(f"{label} must be numeric")
            if not math.isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError(f"{label} must be finite and positive")
        if (
            isinstance(self.probe_duration_s, bool)
            or not isinstance(self.probe_duration_s, int)
            or self.probe_duration_s != 12
        ):
            raise ValueError("manual short gate probe_duration_s must be exactly 12")
        if self.minimum_trigger_budget_s < (
            self.probe_duration_s + self.probe_completion_buffer_s
        ):
            raise ValueError("minimum trigger budget cannot cover the 12s probe")
        object.__setattr__(self, "run_dir", run_dir)
        object.__setattr__(self, "performance_probe_script", probe_script)


@dataclass(frozen=True)
class ManualShortGateResult:
    """Bounded result returned to the launcher without changing its lifecycle."""

    passed: bool
    failure_reason: str | None
    summary_path: Path
    window_proof_path: Path | None
    performance_probe_path: Path | None
    probe_invocation_count: int


def supervise_manual_short_gate(
    config: ManualShortGateConfig,
    *,
    prove_presence: Callable[[int], object],
    command_runner: Callable[[tuple[str, ...], float], _CommandResult] | None = None,
    read_json: Callable[[Path], Mapping[str, Any]] | None = None,
    read_text: Callable[[Path], str] | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    wall_time_ns: Callable[[], int] = time.time_ns,
    sleep: Callable[[float], object] = time.sleep,
) -> ManualShortGateResult:
    """Run pre/post window-presence proofs around one 12-second probe.

    Failures are evidence, not launcher lifecycle requests.  This function never
    sends input, never starts a long probe, and never terminates the owned UE tree.
    """

    if not isinstance(config, ManualShortGateConfig):
        raise TypeError("config must be ManualShortGateConfig")
    if not callable(prove_presence):
        raise TypeError("prove_presence must be callable")
    if not callable(monotonic) or not callable(wall_time_ns) or not callable(sleep):
        raise TypeError("short-gate clocks and sleep must be callable")
    json_reader = read_json or _read_json
    text_reader = read_text or _read_text
    runner = command_runner or _run_command
    if not callable(json_reader) or not callable(text_reader) or not callable(runner):
        raise TypeError("short-gate dependencies must be callable")

    summary_path = config.run_dir / _SUMMARY_FILENAME
    if summary_path.exists():
        return _existing_result(summary_path, config, json_reader)

    window_path = config.run_dir / _WINDOW_PROOF_FILENAME
    probe_path = config.run_dir / _PERFORMANCE_PROBE_FILENAME
    probe_calls = 0
    window_written = False
    probe_observed = False
    window_presence_failure: dict[str, Any] | None = None
    failure_reason: str | None = None
    passed = False

    try:
        if window_path.exists() or probe_path.exists():
            raise ManualShortGateError(
                "short-gate artifact already exists without a terminal summary"
            )
        trigger_now = _clock(monotonic)
        trigger_remaining = config.profile_deadline_monotonic - trigger_now
        if trigger_remaining < config.minimum_trigger_budget_s:
            raise ManualShortGateError(
                "manual profile has less than the required 20-second trigger budget"
            )

        readiness = _wait_until_ready(
            config,
            read_json=json_reader,
            read_text=text_reader,
            monotonic=monotonic,
            wall_time_ns=wall_time_ns,
            sleep=sleep,
        )
        try:
            pre_presence = prove_presence(config.unreal_pid)
        except Exception as exc:
            window_presence_failure = _window_presence_failure_diagnostic(exc)
            raise ManualShortGateError(
                "window presence proof dependency failed closed"
            ) from exc
        pre_presence_document = _presence_proof_document(
            pre_presence,
            expected_pid=config.unreal_pid,
        )
        presence_identity = _presence_identity(
            pre_presence_document,
            expected_pid=config.unreal_pid,
        )

        # Presence discovery can wait for a late UE window. Re-read all current
        # runtime evidence before spending the only probe invocation.
        pre_probe_readiness = _validate_ready_once(
            config,
            read_json=json_reader,
            read_text=text_reader,
            wall_time_ns=wall_time_ns,
        )
        if pre_probe_readiness != readiness:
            raise ManualShortGateError(
                "runtime generation changed before the performance probe"
            )
        remaining = config.profile_deadline_monotonic - _clock(monotonic)
        if remaining < config.probe_duration_s + config.probe_completion_buffer_s:
            raise ManualShortGateError(
                "manual profile no longer has enough budget for the 12-second probe"
            )

        command = _probe_command(config)
        command_timeout_s = remaining - config.probe_completion_buffer_s
        probe_calls += 1
        completed: _CommandResult | None = None
        probe_execution_error: BaseException | None = None
        try:
            completed = runner(command, command_timeout_s)
        except BaseException as exc:
            probe_execution_error = exc

        post_presence: object | None = None
        post_presence_error: Exception | None = None
        try:
            post_presence = prove_presence(config.unreal_pid)
        except Exception as exc:
            post_presence_error = exc

        probe: Mapping[str, Any] | None = None
        probe_failure: BaseException | None = probe_execution_error
        if probe_failure is None:
            if completed is None:
                probe_failure = ManualShortGateError(
                    "performance probe returned no command result"
                )
            elif (
                isinstance(completed.returncode, bool)
                or not isinstance(completed.returncode, int)
            ):
                probe_failure = ManualShortGateError(
                    "performance probe returned an invalid exit code"
                )
            else:
                try:
                    probe = _validate_probe_artifact(
                        probe_path,
                        config,
                        json_reader,
                        model_generation=readiness["model_generation"],
                        reset_generation=readiness["reset_generation"],
                    )
                except Exception as artifact_exc:
                    probe_failure = artifact_exc
                else:
                    probe_observed = True
                    if completed.returncode != 0 or probe.get("passed") is not True:
                        probe_failure = ManualShortGateError(
                            "the only 12-second runtime performance probe did not pass"
                        )
        else:
            try:
                probe = _validate_probe_artifact(
                    probe_path,
                    config,
                    json_reader,
                    model_generation=readiness["model_generation"],
                    reset_generation=readiness["reset_generation"],
                )
            except Exception as artifact_exc:
                _add_exception_note(
                    probe_failure,
                    "probe artifact was not accepted after the probe execution failed: "
                    + _bounded_diagnostic_message(artifact_exc),
                )
            else:
                probe_observed = True

        post_presence_document: Mapping[str, Any] | None = None
        post_gate_failure: ManualShortGateError | None = None
        post_diagnostic_source = post_presence_error
        if post_presence_error is not None:
            post_gate_failure = ManualShortGateError(
                "window presence proof dependency failed closed"
            )
        elif post_presence is None:
            post_diagnostic_source = ManualShortGateError(
                "window presence proof returned no result"
            )
            post_gate_failure = ManualShortGateError(
                "window presence proof dependency failed closed"
            )
        else:
            try:
                post_presence_document = _presence_proof_document(
                    post_presence,
                    expected_pid=config.unreal_pid,
                )
                post_identity = _presence_identity(
                    post_presence_document,
                    expected_pid=config.unreal_pid,
                )
            except Exception as exc:
                post_diagnostic_source = exc
                post_gate_failure = ManualShortGateError(
                    "window presence proof dependency failed closed"
                )
            else:
                if post_identity != presence_identity:
                    post_diagnostic_source = ManualShortGateError(
                        "RobotSimUE window presence identity changed during the performance probe"
                    )
                    post_gate_failure = post_diagnostic_source

        if post_gate_failure is not None:
            if post_diagnostic_source is None:
                raise ManualShortGateError(
                    "window presence proof failed without a diagnostic source"
                )
            role = (
                "secondary_to_probe_failure"
                if probe_failure is not None
                else "primary"
            )
            window_presence_failure = _window_presence_failure_diagnostic(
                post_diagnostic_source,
                role=role,
            )
            if probe_failure is not None:
                _add_exception_note(
                    probe_failure,
                    "post-probe window presence failed: "
                    + _bounded_diagnostic_message(post_diagnostic_source),
                )
                raise probe_failure
            else:
                raise post_gate_failure from post_diagnostic_source
        if probe_execution_error is not None:
            raise probe_execution_error
        if post_presence_document is None:
            raise ManualShortGateError("short-gate dependency outcome is incomplete")

        presence_artifact = {
            "schema": "lingtu.sim.manual-window-presence-proof.v1",
            "qualification": False,
            "evidence_class": "manual_diagnostic_only",
            "run_id": config.run_id,
            "session_id": config.session_id,
            "unreal_pid": config.unreal_pid,
            "model_generation": readiness["model_generation"],
            "reset_generation": readiness["reset_generation"],
            "selected_window_identity": {
                "hwnd": presence_identity[0],
                "pid": presence_identity[1],
                "eligible": presence_identity[2],
            },
            "pre_probe": pre_presence_document,
            "post_probe": post_presence_document,
        }
        try:
            post_probe_readiness = _validate_ready_once(
                config,
                read_json=json_reader,
                read_text=text_reader,
                wall_time_ns=wall_time_ns,
            )
            if post_probe_readiness != readiness:
                raise ManualShortGateError(
                    "runtime generation changed during the performance probe"
                )
        except Exception as readiness_exc:
            if probe_failure is not None:
                _add_exception_note(
                    probe_failure,
                    "post-probe runtime readiness failed: "
                    + _bounded_diagnostic_message(readiness_exc),
                )
                raise probe_failure from None
            raise
        try:
            _atomic_write_json(window_path, presence_artifact)
        except Exception as artifact_exc:
            if probe_failure is not None:
                _add_exception_note(
                    probe_failure,
                    "window presence artifact publication failed: "
                    + _bounded_diagnostic_message(artifact_exc),
                )
                raise probe_failure from None
            raise
        window_written = True
        if probe_failure is not None:
            raise probe_failure
        if completed is None or probe is None:
            raise ManualShortGateError("short-gate probe outcome is incomplete")
        passed = True
    except ManualShortGateError as exc:
        failure_reason = f"ManualShortGateError: {exc}"
    except subprocess.TimeoutExpired:
        failure_reason = "ManualShortGateError: the only performance probe exceeded its profile deadline"
    except Exception as exc:
        failure_reason = (
            f"{type(exc).__name__}: short-gate dependency failed closed"
        )

    summary = {
        "schema": "lingtu.sim.manual-short-gate.v1",
        "qualification": False,
        "evidence_class": "manual_diagnostic_only",
        "passed": passed,
        "failure_reason": failure_reason,
        "run_id": config.run_id,
        "session_id": config.session_id,
        "run_dir": str(config.run_dir),
        "unreal_pid": config.unreal_pid,
        "probe_duration_seconds": config.probe_duration_s,
        "probe_invocation_count": probe_calls,
        "input_foreground_status": "not_evaluated",
        "input_ready": False,
        "input_sent": False,
        "long_probe_run": False,
        "retry_count": 0,
        "launcher_lifecycle_action": "return_and_wait_for_natural_profile_deadline",
        "process_evidence_scope": {
            "during_gate": "forbidden SDK/build markers are rejected from Unreal.log",
            "postflight": "owned by the outer launcher orchestrator after CLOSED",
        },
        "window_presence_failure": window_presence_failure,
        "evidence": {
            "runtime_manifest": _evidence(
                config.run_dir / "session.runtime.json",
                missing_reason="runtime manifest was unavailable",
            ),
            "sensor_readiness": _evidence(
                config.run_dir / "logs" / "sensor-readiness.json",
                missing_reason="sensor readiness was unavailable",
            ),
            "unreal_log": _evidence(
                config.run_dir / "logs" / "Unreal.log",
                missing_reason="Unreal log was unavailable",
            ),
            "runtime_health": _evidence(
                config.run_dir / "manual-runtime-health.json",
                missing_reason="manual runtime health was unavailable",
            ),
            "window_presence_proof": _evidence(
                window_path,
                missing_reason=(
                    "pre/post window presence proof was not completed before the gate failed"
                    if not window_written
                    else None
                ),
            ),
            "runtime_performance_probe": _evidence(
                probe_path,
                missing_reason=(
                    "performance probe was not invoked before the gate failed"
                    if probe_calls == 0
                    else "performance probe did not publish a readable artifact"
                    if not probe_observed
                    else None
                ),
            ),
            "episode_result": _evidence(
                config.run_dir / "episode_result.json",
                missing_reason="launcher has not reached its natural terminal state yet",
            ),
        },
    }
    _atomic_write_json(summary_path, summary)
    return ManualShortGateResult(
        passed=passed,
        failure_reason=failure_reason,
        summary_path=summary_path,
        window_proof_path=window_path if window_written else None,
        performance_probe_path=probe_path if probe_observed else None,
        probe_invocation_count=probe_calls,
    )


def _window_presence_failure_diagnostic(
    exc: Exception,
    *,
    role: str = "primary",
) -> dict[str, Any]:
    """Return bounded, single-line local diagnostics without a traceback."""

    if role not in {"primary", "secondary_to_probe_failure"}:
        raise ValueError("window presence diagnostic role is invalid")

    cause = exc.__cause__
    cause_chain: list[dict[str, str]] = []
    observed: set[int] = {id(exc)}
    current = cause
    while current is not None and len(cause_chain) < _MAX_DIAGNOSTIC_CAUSE_DEPTH:
        if id(current) in observed:
            break
        observed.add(id(current))
        cause_chain.append(
            {
                "exception_type": type(current).__name__,
                "message": _bounded_diagnostic_message(current),
            }
        )
        current = current.__cause__
    return {
        "phase": "window_presence_proof",
        "role": role,
        "exception_type": type(exc).__name__,
        "message": _bounded_diagnostic_message(exc),
        "cause_type": None if cause is None else type(cause).__name__,
        "cause_message": (
            None if cause is None else _bounded_diagnostic_message(cause)
        ),
        "cause_chain": cause_chain,
        "cause_chain_truncated": current is not None,
    }


def _bounded_diagnostic_message(exc: BaseException) -> str:
    message = _DIAGNOSTIC_CONTROL_RE.sub(" ", str(exc))
    message = " ".join(message.split()).strip()
    if not message:
        message = "exception had no message"
    return message[:_MAX_DIAGNOSTIC_MESSAGE_CHARS]


def _add_exception_note(exc: BaseException, note: str) -> None:
    """Attach secondary context when the active Python supports PEP 678."""

    add_note = getattr(exc, "add_note", None)
    if callable(add_note):
        add_note(note)


def _wait_until_ready(
    config: ManualShortGateConfig,
    *,
    read_json: Callable[[Path], Mapping[str, Any]],
    read_text: Callable[[Path], str],
    monotonic: Callable[[], float],
    wall_time_ns: Callable[[], int],
    sleep: Callable[[float], object],
) -> Mapping[str, int]:
    started = _clock(monotonic)
    deadline = min(
        started + config.readiness_timeout_s,
        config.profile_deadline_monotonic
        - config.probe_duration_s
        - config.probe_completion_buffer_s,
    )
    last_error = "runtime evidence has not been observed"
    while True:
        try:
            return _validate_ready_once(
                config,
                read_json=read_json,
                read_text=read_text,
                wall_time_ns=wall_time_ns,
            )
        except _NotReady as exc:
            last_error = str(exc)
        now = _clock(monotonic)
        if now >= deadline:
            raise ManualShortGateError(
                f"timed out waiting for SDK-quiet short-gate readiness: {last_error}"
            )
        sleep(min(config.poll_interval_s, deadline - now))


def _validate_ready_once(
    config: ManualShortGateConfig,
    *,
    read_json: Callable[[Path], Mapping[str, Any]],
    read_text: Callable[[Path], str],
    wall_time_ns: Callable[[], int],
) -> Mapping[str, int]:
    try:
        unreal_log = read_text(config.run_dir / "logs" / "Unreal.log")
    except (OSError, UnicodeError) as exc:
        raise _NotReady("Unreal log is not readable yet") from exc
    _require_quiet_unreal_log(unreal_log)
    marker_count = unreal_log.count(_UI_MARKER)
    if marker_count == 0:
        raise _NotReady("runtime UI attach marker has not appeared")
    if marker_count != 1:
        raise ManualShortGateError(
            "runtime UI attach marker must appear exactly once"
        )

    try:
        runtime = read_json(config.run_dir / "session.runtime.json")
        health = read_json(config.run_dir / "manual-runtime-health.json")
        sensors = read_json(config.run_dir / "logs" / "sensor-readiness.json")
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
        raise _NotReady("runtime JSON evidence is not stable yet") from exc
    model_generation, reset_generation = _require_runtime(runtime, config)
    _require_runtime_health(
        health,
        config,
        model_generation=model_generation,
        reset_generation=reset_generation,
        observed_wall_time_ns=_wall_clock_ns(wall_time_ns),
    )
    _require_sensor_evidence(
        sensors,
        config,
        model_generation=model_generation,
        reset_generation=reset_generation,
    )
    return {
        "model_generation": model_generation,
        "reset_generation": reset_generation,
    }


def _require_quiet_unreal_log(unreal_log: str) -> None:
    if not isinstance(unreal_log, str):
        raise TypeError("Unreal log reader must return text")
    matches = tuple(
        marker for marker in _FORBIDDEN_UNREAL_MARKERS if marker in unreal_log
    )
    if matches:
        raise ManualShortGateError(
            "forbidden SDK/build marker appeared in Unreal.log: "
            + ", ".join(matches)
        )


def _require_runtime(
    runtime: Mapping[str, Any],
    config: ManualShortGateConfig,
) -> tuple[int, int]:
    _require_mapping(runtime, "runtime")
    if runtime.get("schema") != "lingtu.sim.session-runtime.v1":
        raise ManualShortGateError("runtime schema is invalid")
    _require_identity(runtime, config, "runtime")
    state = runtime.get("state")
    if state != "RUNNING":
        if state in {"FAILED", "STOPPED"}:
            raise ManualShortGateError(f"runtime reached terminal state {state}")
        raise _NotReady("runtime is not RUNNING yet")
    allocation = _require_mapping(runtime.get("allocation"), "runtime allocation")
    allocated_run_dir = allocation.get("run_dir")
    if not isinstance(allocated_run_dir, str):
        raise ManualShortGateError("runtime allocation has no run_dir")
    try:
        allocated = Path(allocated_run_dir).resolve(strict=True)
    except OSError as exc:
        raise ManualShortGateError("runtime allocation run_dir is invalid") from exc
    if allocated != config.run_dir:
        raise ManualShortGateError("runtime allocation run_dir identity mismatch")
    model_generation = _nonnegative_int(runtime.get("model_generation"), "model_generation")
    reset_generation = _nonnegative_int(runtime.get("reset_generation"), "reset_generation")
    sensor_runtime = _require_mapping(runtime.get("sensor_streams"), "sensor_streams")
    if sensor_runtime.get("is_ready") is not True:
        raise _NotReady("runtime sensor aggregate is not ready")
    summary = _require_mapping(sensor_runtime.get("summary"), "sensor summary")
    if summary.get("schema") != "lingtu.sim.sensor-stream-summary.v1":
        raise ManualShortGateError("runtime sensor summary schema is invalid")
    if (
        summary.get("is_ready") is not True
        or summary.get("session_id") != config.session_id
        or summary.get("model_generation") != model_generation
        or summary.get("reset_generation") != reset_generation
    ):
        raise _NotReady("runtime sensor summary identity is not current")
    streams = _require_mapping(summary.get("streams"), "sensor summary streams")
    for sensor_id, kind in _CAMERA_STREAMS.items():
        stream = _require_mapping(streams.get(sensor_id), f"runtime {sensor_id}")
        if stream.get("state") != "ACTIVE":
            raise _NotReady(f"runtime camera {sensor_id} is not ACTIVE")
        expected = {
            "stream_id": sensor_id,
            "stream_kind": kind,
            "owner": "visual",
            "source": "unreal_camera",
            "transport": "camera_shm",
            "model_generation": model_generation,
            "reset_generation": reset_generation,
            "session_id": config.session_id,
        }
        if any(stream.get(field) != value for field, value in expected.items()):
            raise ManualShortGateError(
                f"runtime camera {sensor_id} identity or route mismatch"
            )
        if _nonnegative_int(stream.get("sample_count"), f"{sensor_id}.sample_count") < 1:
            raise _NotReady(f"runtime camera {sensor_id} has no samples")
    return model_generation, reset_generation


def _require_runtime_health(
    health: Mapping[str, Any],
    config: ManualShortGateConfig,
    *,
    model_generation: int,
    reset_generation: int,
    observed_wall_time_ns: int,
) -> None:
    _require_mapping(health, "runtime health")
    if health.get("schema") != "lingtu.sim.manual-runtime-health.v1":
        raise ManualShortGateError("manual runtime health schema is invalid")
    _require_identity(health, config, "manual runtime health")
    if (
        health.get("qualification") is not False
        or health.get("evidence_class") != "manual_diagnostic_only"
    ):
        raise ManualShortGateError("manual runtime health classification is invalid")
    if health.get("session_state") != "RUNNING":
        raise _NotReady("manual runtime health is not RUNNING")
    if health.get("owner_thread_alive") is not True:
        raise ManualShortGateError("manual runtime owner is not alive")
    observed_model_generation = health.get("model_generation")
    observed_reset_generation = health.get("reset_generation")
    if observed_model_generation is None or observed_reset_generation is None:
        raise _NotReady("manual runtime health has not observed a truth generation")
    observed_model_generation = _nonnegative_int(
        observed_model_generation,
        "health.model_generation",
    )
    observed_reset_generation = _nonnegative_int(
        observed_reset_generation,
        "health.reset_generation",
    )
    if (
        observed_model_generation != model_generation
        or observed_reset_generation != reset_generation
    ):
        raise ManualShortGateError("manual runtime health generation mismatch")
    for field in ("non_running_transition_count", "owner_thread_stop_count"):
        if _nonnegative_int(health.get(field), f"health.{field}") != 0:
            raise ManualShortGateError(f"manual runtime health {field} is not zero")
    if _nonnegative_int(health.get("monitor_sequence"), "health.monitor_sequence") < 1:
        raise _NotReady("manual runtime health has no heartbeat")
    updated_unix_ns = _nonnegative_int(
        health.get("updated_unix_ns"),
        "health.updated_unix_ns",
    )
    if updated_unix_ns < 1:
        raise _NotReady("manual runtime health timestamp is absent")
    if updated_unix_ns > observed_wall_time_ns:
        raise ManualShortGateError("manual runtime health timestamp is in the future")
    maximum_age_ns = int(config.maximum_health_age_s * 1_000_000_000)
    if observed_wall_time_ns - updated_unix_ns > maximum_age_ns:
        raise _NotReady("manual runtime health heartbeat is stale")


def _require_sensor_evidence(
    evidence: Mapping[str, Any],
    config: ManualShortGateConfig,
    *,
    model_generation: int,
    reset_generation: int,
) -> None:
    _require_mapping(evidence, "sensor readiness")
    if evidence.get("schema") != "lingtu.sim.sensor-readiness-evidence.v1":
        raise ManualShortGateError("sensor readiness schema is invalid")
    if (
        evidence.get("session_id") != config.session_id
        or evidence.get("model_generation") != model_generation
        or evidence.get("reset_generation") != reset_generation
    ):
        raise ManualShortGateError("sensor readiness identity mismatch")
    raw_streams = evidence.get("streams")
    if not isinstance(raw_streams, Sequence) or isinstance(raw_streams, (str, bytes)):
        raise ManualShortGateError("sensor readiness streams must be an array")
    indexed: dict[str, Mapping[str, Any]] = {}
    for value in raw_streams:
        stream = _require_mapping(value, "sensor readiness stream")
        sensor_id = stream.get("sensor_id")
        if not isinstance(sensor_id, str) or sensor_id in indexed:
            raise ManualShortGateError("sensor readiness has invalid or duplicate IDs")
        indexed[sensor_id] = stream
    for sensor_id in _CAMERA_STREAMS:
        current_stream = indexed.get(sensor_id)
        if current_stream is None or current_stream.get("state") != "ACTIVE":
            raise _NotReady(f"sensor evidence {sensor_id} is not ACTIVE")
        if _nonnegative_int(
            current_stream.get("published_frames"),
            f"{sensor_id}.published_frames",
        ) < 1:
            raise _NotReady(f"sensor evidence {sensor_id} has no rendered frames")


def _presence_proof_document(
    proof: object,
    *,
    expected_pid: int,
) -> Mapping[str, Any]:
    to_dict = getattr(proof, "to_dict", None)
    if not callable(to_dict):
        raise ManualShortGateError("window proof must expose to_dict()")
    return _validated_presence_document(to_dict(), expected_pid=expected_pid)


def _validated_presence_document(
    value: object,
    *,
    expected_pid: int,
) -> Mapping[str, Any]:
    from .playable_vertical_slice import (
        _is_eligible_robotsimue_game_window,
        _TopLevelWindowSnapshot,
    )

    document = _require_mapping(value, "window proof")
    if (
        document.get("qualification") is not False
        or document.get("evidence_class") != "manual_diagnostic_only"
    ):
        raise ManualShortGateError("window proof classification is invalid")
    expected_fields = {
        "schema",
        "qualification",
        "evidence_class",
        "owned_unreal_pid",
        "candidates",
        "selected_hwnd",
        "selected_pid",
        "observed_at_unix_ns",
    }
    if set(document) != expected_fields:
        raise ManualShortGateError("window presence proof fields are invalid")
    if (
        document.get("schema")
        != "lingtu.sim.owned-robotsimue-window-presence-proof.v1"
    ):
        raise ManualShortGateError("window presence proof schema is invalid")
    if document.get("owned_unreal_pid") != expected_pid:
        raise ManualShortGateError("window presence proof PID identity mismatch")
    selected_hwnd = _positive_int(document.get("selected_hwnd"), "selected_hwnd")
    if document.get("selected_pid") != expected_pid:
        raise ManualShortGateError("window presence proof selected PID mismatch")
    if _positive_int(
        document.get("observed_at_unix_ns"),
        "observed_at_unix_ns",
    ) < 1:
        raise ManualShortGateError("window proof observation time is invalid")
    candidates = document.get("candidates")
    if not isinstance(candidates, Sequence) or isinstance(candidates, (str, bytes)):
        raise ManualShortGateError("window proof candidates must be an array")
    selected_matches = 0
    eligible_candidates: list[tuple[int, int]] = []
    observed_candidates: set[tuple[object, ...]] = set()
    for value in candidates:
        candidate = _require_mapping(value, "window proof candidate")
        expected_candidate_fields = {
            "hwnd",
            "pid",
            "title",
            "title_redacted",
            "visible",
            "enabled",
            "owner_hwnd",
            "window_area",
            "client_area",
            "eligible",
        }
        if set(candidate) != expected_candidate_fields:
            raise ManualShortGateError("window proof candidate fields are invalid")
        hwnd = _positive_int(candidate.get("hwnd"), "window proof candidate hwnd")
        pid = _positive_int(candidate.get("pid"), "window proof candidate pid")
        title = candidate.get("title")
        title_redacted = candidate.get("title_redacted")
        if not isinstance(title_redacted, bool):
            raise ManualShortGateError(
                "window proof candidate title_redacted is invalid"
            )
        for field in ("visible", "enabled", "eligible"):
            if not isinstance(candidate.get(field), bool):
                raise ManualShortGateError(
                    f"window proof candidate {field} is invalid"
                )
        owner_hwnd = candidate.get("owner_hwnd")
        if owner_hwnd is not None:
            _positive_int(owner_hwnd, "window proof candidate owner_hwnd")
        window_area = _nonnegative_int(
            candidate.get("window_area"),
            "window proof candidate window_area",
        )
        client_area = _nonnegative_int(
            candidate.get("client_area"),
            "window proof candidate client_area",
        )
        structurally_eligible = (
            pid == expected_pid
            and candidate["visible"]
            and candidate["enabled"]
            and owner_hwnd is None
            and window_area > 0
            and client_area > 0
        )
        if candidate.get("eligible") is True and not structurally_eligible:
            raise ManualShortGateError(
                "window proof candidate eligibility is inconsistent"
            )
        if candidate.get("eligible") is True:
            if title_redacted or not isinstance(title, str):
                raise ManualShortGateError(
                    "eligible window proof candidate title disclosure is invalid"
                )
            expected_eligibility = _is_eligible_robotsimue_game_window(
                _TopLevelWindowSnapshot(
                    hwnd=hwnd,
                    pid=pid,
                    title=title,
                    visible=candidate["visible"],
                    enabled=candidate["enabled"],
                    owner_hwnd=owner_hwnd,
                    window_area=window_area,
                    client_area=client_area,
                ),
                expected_pid=expected_pid,
            )
            if candidate.get("eligible") is not expected_eligibility:
                raise ManualShortGateError(
                    "window proof candidate eligibility is inconsistent"
                )
        elif not title_redacted or title is not None:
            raise ManualShortGateError(
                "ineligible window proof candidate title must be redacted"
            )
        identity = (
            hwnd,
            pid,
            title,
            title_redacted,
            candidate.get("visible"),
            candidate.get("enabled"),
            owner_hwnd,
            window_area,
            client_area,
            candidate.get("eligible"),
        )
        if identity in observed_candidates:
            raise ManualShortGateError("window proof candidates are not deduplicated")
        observed_candidates.add(identity)
        if candidate.get("eligible") is True:
            eligible_candidates.append((hwnd, client_area))
        if (
            hwnd == selected_hwnd
            and pid == expected_pid
            and candidate.get("eligible") is True
        ):
            selected_matches += 1
    if selected_matches != 1:
        raise ManualShortGateError(
            "window proof must contain exactly one eligible selected candidate"
        )
    if not eligible_candidates:
        raise ManualShortGateError("window proof has no eligible candidates")
    largest_area = max(client_area for _hwnd, client_area in eligible_candidates)
    largest = tuple(
        hwnd
        for hwnd, client_area in eligible_candidates
        if client_area == largest_area
    )
    if largest != (selected_hwnd,):
        raise ManualShortGateError(
            "window proof selected candidate is not the unique largest client area"
        )
    # Strict serialization also rejects NaN and unserializable callback output.
    _json_bytes(document)
    return document


def _presence_identity(
    document: Mapping[str, Any],
    *,
    expected_pid: int,
) -> tuple[int, int, bool]:
    selected_hwnd = _positive_int(document.get("selected_hwnd"), "selected_hwnd")
    selected_pid = _positive_int(document.get("selected_pid"), "selected_pid")
    if selected_pid != expected_pid:
        raise ManualShortGateError("window presence proof selected PID mismatch")
    candidates = document.get("candidates")
    if not isinstance(candidates, Sequence) or isinstance(candidates, (str, bytes)):
        raise ManualShortGateError("window proof candidates must be an array")
    selected = tuple(
        candidate
        for candidate in candidates
        if isinstance(candidate, Mapping)
        and candidate.get("hwnd") == selected_hwnd
        and candidate.get("pid") == selected_pid
    )
    if len(selected) != 1 or selected[0].get("eligible") is not True:
        raise ManualShortGateError("window presence proof selected identity is invalid")
    return selected_hwnd, selected_pid, True


def _probe_command(config: ManualShortGateConfig) -> tuple[str, ...]:
    return (
        str(config.powershell_executable),
        "-NoLogo",
        "-NoProfile",
        "-NonInteractive",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        str(config.performance_probe_script),
        "-RunDir",
        str(config.run_dir),
        "-ExpectedRunId",
        config.run_id,
        "-ProductSessionId",
        config.session_id,
        "-DurationSeconds",
        "12",
    )


def _run_command(
    command: tuple[str, ...],
    timeout_s: float,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(  # noqa: S603
        command,
        check=False,
        capture_output=True,
        text=True,
        creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        timeout=timeout_s,
    )


def _validate_probe_artifact(
    path: Path,
    config: ManualShortGateConfig,
    read_json: Callable[[Path], Mapping[str, Any]],
    *,
    model_generation: int,
    reset_generation: int,
) -> Mapping[str, Any]:
    try:
        probe = read_json(path)
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
        raise ManualShortGateError(
            "performance probe did not publish a readable artifact"
        ) from exc
    _require_mapping(probe, "performance probe")
    if probe.get("schema") != "lingtu.sim.runtime-performance-probe.v1":
        raise ManualShortGateError("performance probe schema is invalid")
    _require_identity(probe, config, "performance probe")
    if (
        probe.get("qualification") is not False
        or probe.get("evidence_class") != "manual_diagnostic_only"
    ):
        raise ManualShortGateError("performance probe classification is invalid")
    run_dir = probe.get("run_dir")
    if not isinstance(run_dir, str):
        raise ManualShortGateError("performance probe run_dir is absent")
    try:
        resolved = Path(run_dir).resolve(strict=True)
    except OSError as exc:
        raise ManualShortGateError("performance probe run_dir is invalid") from exc
    if resolved != config.run_dir:
        raise ManualShortGateError("performance probe run_dir identity mismatch")
    verdict = probe.get("passed")
    if not isinstance(verdict, bool):
        raise ManualShortGateError("performance probe verdict is invalid")
    if (
        probe.get("model_generation") != model_generation
        or probe.get("reset_generation") != reset_generation
    ):
        raise ManualShortGateError("performance probe generation mismatch")
    session_state = probe.get("session_state")
    if not isinstance(session_state, str) or not session_state:
        raise ManualShortGateError("performance probe session_state is invalid")

    duration_seconds = _finite_number(
        probe.get("duration_seconds"),
        "performance probe duration_seconds",
    )
    minimum_duration = config.probe_duration_s * 0.95
    maximum_duration = config.probe_duration_s + config.probe_completion_buffer_s
    if not 0.0 <= duration_seconds <= maximum_duration:
        raise ManualShortGateError("performance probe duration is outside the 12s gate")
    duration_passed = duration_seconds >= minimum_duration

    monitor_start = _nonnegative_int(
        probe.get("monitor_sequence_start"),
        "performance probe monitor_sequence_start",
    )
    monitor_end = _nonnegative_int(
        probe.get("monitor_sequence_end"),
        "performance probe monitor_sequence_end",
    )
    non_running_count = _nonnegative_int(
        probe.get("non_running_transition_count"),
        "performance probe non_running_transition_count",
    )
    owner_stop_count = _nonnegative_int(
        probe.get("owner_thread_stop_count"),
        "performance probe owner_thread_stop_count",
    )
    continuous_passed = (
        session_state == "RUNNING"
        and monitor_start >= 1
        and monitor_end > monitor_start
        and non_running_count == 0
        and owner_stop_count == 0
    )

    minimum_realtime = _finite_number(
        probe.get("minimum_median_realtime_factor"),
        "performance probe minimum_median_realtime_factor",
    )
    realtime = _finite_number(
        probe.get("median_realtime_factor"),
        "performance probe median_realtime_factor",
    )
    if minimum_realtime != _MINIMUM_REALTIME_FACTOR:
        raise ManualShortGateError("performance probe realtime threshold mismatch")
    realtime_passed = realtime >= minimum_realtime

    minimum_camera_rate = _finite_number(
        probe.get("minimum_camera_rate_hz"),
        "performance probe minimum_camera_rate_hz",
    )
    if minimum_camera_rate != _MINIMUM_CAMERA_RATE_HZ:
        raise ManualShortGateError("performance probe camera threshold mismatch")
    camera_rates = _require_mapping(
        probe.get("camera_rates_hz"),
        "performance probe camera_rates_hz",
    )
    if set(camera_rates) != set(_CAMERA_STREAMS):
        raise ManualShortGateError("performance probe camera rate IDs are invalid")
    camera_rates_passed = True
    for sensor_id in _CAMERA_STREAMS:
        rate = _finite_number(
            camera_rates.get(sensor_id),
            f"performance probe {sensor_id} rate",
        )
        if rate < minimum_camera_rate:
            camera_rates_passed = False

    checks = _require_mapping(probe.get("checks"), "performance probe checks")
    expected_checks = {
        "continuous_running",
        "duration",
        "realtime_factor",
        "camera_rates",
    }
    if set(checks) != expected_checks or any(
        not isinstance(checks.get(field), bool) for field in expected_checks
    ):
        raise ManualShortGateError("performance probe checks are invalid")
    computed_checks = {
        "continuous_running": continuous_passed,
        "duration": duration_passed,
        "realtime_factor": realtime_passed,
        "camera_rates": camera_rates_passed,
    }
    if any(checks.get(field) is not expected for field, expected in computed_checks.items()):
        raise ManualShortGateError("performance probe checks contradict its metrics")
    if verdict is not all(computed_checks.values()):
        raise ManualShortGateError("performance probe verdict contradicts its checks")
    return probe


def _existing_result(
    path: Path,
    config: ManualShortGateConfig,
    read_json: Callable[[Path], Mapping[str, Any]],
) -> ManualShortGateResult:
    document = read_json(path)
    _require_mapping(document, "manual short-gate summary")
    if document.get("schema") != "lingtu.sim.manual-short-gate.v1":
        raise ManualShortGateError("existing short-gate summary schema is invalid")
    _require_identity(document, config, "existing short-gate summary")
    if (
        document.get("qualification") is not False
        or document.get("evidence_class") != "manual_diagnostic_only"
        or document.get("input_foreground_status") != "not_evaluated"
        or document.get("input_ready") is not False
        or document.get("input_sent") is not False
        or document.get("long_probe_run") is not False
        or document.get("retry_count") != 0
    ):
        raise ManualShortGateError("existing short-gate invariant is invalid")
    existing_run_dir = document.get("run_dir")
    if not isinstance(existing_run_dir, str):
        raise ManualShortGateError("existing short-gate run_dir is invalid")
    try:
        resolved_run_dir = Path(existing_run_dir).resolve(strict=True)
    except OSError as exc:
        raise ManualShortGateError("existing short-gate run_dir is invalid") from exc
    if resolved_run_dir != config.run_dir:
        raise ManualShortGateError("existing short-gate run_dir identity mismatch")
    if document.get("unreal_pid") != config.unreal_pid:
        raise ManualShortGateError("existing short-gate Unreal PID identity mismatch")
    if document.get("probe_duration_seconds") != config.probe_duration_s:
        raise ManualShortGateError("existing short-gate probe duration mismatch")
    passed = document.get("passed")
    if not isinstance(passed, bool):
        raise ManualShortGateError("existing short-gate verdict is invalid")
    calls = _nonnegative_int(
        document.get("probe_invocation_count"),
        "probe_invocation_count",
    )
    if calls > 1 or (passed and calls != 1):
        raise ManualShortGateError("existing short-gate invocation count is invalid")
    failure = document.get("failure_reason")
    if failure is not None and not isinstance(failure, str):
        raise ManualShortGateError("existing short-gate failure reason is invalid")
    if passed and failure is not None:
        raise ManualShortGateError("passing short-gate summary has a failure reason")
    if not passed and (not isinstance(failure, str) or not failure):
        raise ManualShortGateError("failed short-gate summary has no failure reason")
    _validate_existing_window_presence_failure(
        document.get("window_presence_failure"),
        passed=passed,
        failure_reason=failure,
        probe_invocation_count=calls,
    )
    window_path = config.run_dir / _WINDOW_PROOF_FILENAME
    probe_path = config.run_dir / _PERFORMANCE_PROBE_FILENAME
    presence_generation: tuple[int, int] | None = None
    if window_path.is_file():
        try:
            presence_generation = _validate_existing_presence_artifact(
                read_json(window_path),
                config=config,
            )
        except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
            raise ManualShortGateError(
                "existing window presence artifact is unreadable"
            ) from exc
    if calls == 0:
        if window_path.is_file():
            raise ManualShortGateError(
                "existing presence artifact exists while probe invocation count is zero"
            )
        if probe_path.is_file():
            raise ManualShortGateError(
                "existing probe artifact exists while invocation count is zero"
            )
    else:
        if not probe_path.is_file():
            raise ManualShortGateError(
                "existing probe artifact is missing after one invocation"
            )
        runtime_generation = _existing_runtime_generation(
            config,
            read_json=read_json,
        )
        if (
            presence_generation is not None
            and presence_generation != runtime_generation
        ):
            raise ManualShortGateError(
                "existing window presence artifact generation mismatch"
            )
        expected_generation = presence_generation or runtime_generation
        _validate_probe_artifact(
            probe_path,
            config,
            read_json,
            model_generation=expected_generation[0],
            reset_generation=expected_generation[1],
        )
    if passed:
        # This API is deliberately one-shot.  A previously published success
        # cannot authorize a new caller without re-running the live proof and
        # probe, so fail closed instead of trusting summary-only state.
        raise ManualShortGateError(
            "existing passing short-gate summary cannot authorize re-entry"
        )
    return ManualShortGateResult(
        passed=passed,
        failure_reason=failure,
        summary_path=path,
        window_proof_path=window_path if window_path.is_file() else None,
        performance_probe_path=probe_path if probe_path.is_file() else None,
        probe_invocation_count=calls,
    )


def _validate_existing_presence_artifact(
    value: object,
    *,
    config: ManualShortGateConfig,
) -> tuple[int, int]:
    try:
        artifact = _require_mapping(value, "existing window presence artifact")
        expected_fields = {
            "schema",
            "qualification",
            "evidence_class",
            "run_id",
            "session_id",
            "unreal_pid",
            "model_generation",
            "reset_generation",
            "selected_window_identity",
            "pre_probe",
            "post_probe",
        }
        if set(artifact) != expected_fields:
            raise ManualShortGateError(
                "existing window presence artifact fields are invalid"
            )
        if artifact.get("schema") != "lingtu.sim.manual-window-presence-proof.v1":
            raise ManualShortGateError(
                "existing window presence artifact schema is invalid"
            )
        if (
            artifact.get("qualification") is not False
            or artifact.get("evidence_class") != "manual_diagnostic_only"
        ):
            raise ManualShortGateError(
                "existing window presence artifact classification is invalid"
            )
        _require_identity(artifact, config, "existing window presence artifact")
        if artifact.get("unreal_pid") != config.unreal_pid:
            raise ManualShortGateError(
                "existing window presence artifact Unreal PID mismatch"
            )
        model_generation = _nonnegative_int(
            artifact.get("model_generation"),
            "existing window presence artifact model_generation",
        )
        reset_generation = _nonnegative_int(
            artifact.get("reset_generation"),
            "existing window presence artifact reset_generation",
        )
        pre_document = _validated_presence_document(
            artifact.get("pre_probe"),
            expected_pid=config.unreal_pid,
        )
        post_document = _validated_presence_document(
            artifact.get("post_probe"),
            expected_pid=config.unreal_pid,
        )
        pre_identity = _presence_identity(
            pre_document,
            expected_pid=config.unreal_pid,
        )
        if _presence_identity(
            post_document,
            expected_pid=config.unreal_pid,
        ) != pre_identity:
            raise ManualShortGateError(
                "existing window presence artifact pre/post identity mismatch"
            )
        selected = _require_mapping(
            artifact.get("selected_window_identity"),
            "existing window presence artifact selected identity",
        )
        if set(selected) != {"hwnd", "pid", "eligible"} or (
            selected.get("hwnd"),
            selected.get("pid"),
            selected.get("eligible"),
        ) != pre_identity:
            raise ManualShortGateError(
                "existing window presence artifact selected identity mismatch"
            )
        return model_generation, reset_generation
    except ManualShortGateError as exc:
        if str(exc).startswith("existing window presence artifact"):
            raise
        raise ManualShortGateError(
            f"existing window presence artifact is invalid: {exc}"
        ) from exc


def _existing_runtime_generation(
    config: ManualShortGateConfig,
    *,
    read_json: Callable[[Path], Mapping[str, Any]],
) -> tuple[int, int]:
    runtime = _require_mapping(
        read_json(config.run_dir / "session.runtime.json"),
        "existing runtime",
    )
    if runtime.get("schema") != "lingtu.sim.session-runtime.v1":
        raise ManualShortGateError("existing runtime schema is invalid")
    _require_identity(runtime, config, "existing runtime")
    if runtime.get("state") not in {"RUNNING", "STOPPED", "FAILED"}:
        raise ManualShortGateError("existing runtime state is invalid")
    allocation = _require_mapping(
        runtime.get("allocation"),
        "existing runtime allocation",
    )
    run_dir = allocation.get("run_dir")
    if not isinstance(run_dir, str):
        raise ManualShortGateError("existing runtime allocation run_dir is invalid")
    try:
        resolved_run_dir = Path(run_dir).resolve(strict=True)
    except OSError as exc:
        raise ManualShortGateError(
            "existing runtime allocation run_dir is invalid"
        ) from exc
    if resolved_run_dir != config.run_dir:
        raise ManualShortGateError(
            "existing runtime allocation run_dir identity mismatch"
        )
    return (
        _nonnegative_int(
            runtime.get("model_generation"),
            "existing runtime model_generation",
        ),
        _nonnegative_int(
            runtime.get("reset_generation"),
            "existing runtime reset_generation",
        ),
    )


def _validate_existing_window_presence_failure(
    value: object,
    *,
    passed: bool,
    failure_reason: str | None,
    probe_invocation_count: int,
) -> None:
    is_primary_window_failure = (
        isinstance(failure_reason, str)
        and (
            "window presence proof dependency failed closed" in failure_reason
            or "window presence identity changed" in failure_reason
        )
    )
    if value is None:
        if is_primary_window_failure:
            raise ManualShortGateError(
                "existing short-gate window presence diagnostic is missing"
            )
        return
    if passed or (
        not is_primary_window_failure and probe_invocation_count != 1
    ):
        raise ManualShortGateError(
            "existing short-gate window presence diagnostic is inconsistent"
        )
    diagnostic = _require_mapping(
        value,
        "existing short-gate window presence diagnostic",
    )
    expected_fields = {
        "phase",
        "role",
        "exception_type",
        "message",
        "cause_type",
        "cause_message",
        "cause_chain",
        "cause_chain_truncated",
    }
    if set(diagnostic) != expected_fields:
        raise ManualShortGateError(
            "existing short-gate window presence diagnostic fields are invalid"
        )
    if diagnostic.get("phase") != "window_presence_proof":
        raise ManualShortGateError(
            "existing short-gate window presence diagnostic phase is invalid"
        )
    expected_role = (
        "primary"
        if is_primary_window_failure
        else "secondary_to_probe_failure"
    )
    if diagnostic.get("role") != expected_role:
        raise ManualShortGateError(
            "existing short-gate window presence diagnostic role is invalid"
        )
    _require_diagnostic_text(
        diagnostic.get("exception_type"),
        "existing short-gate window presence exception_type",
        maximum=128,
    )
    _require_diagnostic_text(
        diagnostic.get("message"),
        "existing short-gate window presence message",
        maximum=_MAX_DIAGNOSTIC_MESSAGE_CHARS,
    )
    cause_type = diagnostic.get("cause_type")
    cause_message = diagnostic.get("cause_message")
    if (cause_type is None) != (cause_message is None):
        raise ManualShortGateError(
            "existing short-gate window presence direct cause is invalid"
        )
    chain = diagnostic.get("cause_chain")
    if not isinstance(chain, list) or len(chain) > _MAX_DIAGNOSTIC_CAUSE_DEPTH:
        raise ManualShortGateError(
            "existing short-gate window presence cause chain is invalid"
        )
    validated_chain: list[tuple[str, str]] = []
    for item in chain:
        record = _require_mapping(
            item,
            "existing short-gate window presence cause",
        )
        if set(record) != {"exception_type", "message"}:
            raise ManualShortGateError(
                "existing short-gate window presence cause fields are invalid"
            )
        validated_chain.append(
            (
                _require_diagnostic_text(
                    record.get("exception_type"),
                    "existing short-gate window presence cause exception_type",
                    maximum=128,
                ),
                _require_diagnostic_text(
                    record.get("message"),
                    "existing short-gate window presence cause message",
                    maximum=_MAX_DIAGNOSTIC_MESSAGE_CHARS,
                ),
            )
        )
    if not isinstance(diagnostic.get("cause_chain_truncated"), bool):
        raise ManualShortGateError(
            "existing short-gate window presence cause truncation flag is invalid"
        )
    if validated_chain:
        if (cause_type, cause_message) != validated_chain[0]:
            raise ManualShortGateError(
                "existing short-gate window presence direct cause mismatch"
            )
    elif cause_type is not None:
        raise ManualShortGateError(
            "existing short-gate window presence direct cause mismatch"
        )


def _require_diagnostic_text(
    value: object,
    label: str,
    *,
    maximum: int,
) -> str:
    if not isinstance(value, str) or not value or len(value) > maximum:
        raise ManualShortGateError(f"{label} is invalid")
    if _DIAGNOSTIC_CONTROL_RE.search(value) is not None or " ".join(value.split()) != value:
        raise ManualShortGateError(f"{label} is invalid")
    return value


def _require_identity(
    document: Mapping[str, Any],
    config: ManualShortGateConfig,
    label: str,
) -> None:
    if document.get("run_id") != config.run_id:
        raise ManualShortGateError(f"{label} run_id identity mismatch")
    if document.get("session_id") != config.session_id:
        raise ManualShortGateError(f"{label} session_id identity mismatch")


def _require_mapping(value: object, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ManualShortGateError(f"{label} must be an object")
    return value


def _nonnegative_int(value: object, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ManualShortGateError(f"{label} must be a non-negative integer")
    return value


def _positive_int(value: object, label: str) -> int:
    result = _nonnegative_int(value, label)
    if result < 1:
        raise ManualShortGateError(f"{label} must be a positive integer")
    return result


def _finite_number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ManualShortGateError(f"{label} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ManualShortGateError(f"{label} must be finite")
    return result


def _clock(monotonic: Callable[[], float]) -> float:
    value = monotonic()
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ManualShortGateError("monotonic clock returned a non-number")
    result = float(value)
    if not math.isfinite(result):
        raise ManualShortGateError("monotonic clock returned a non-finite value")
    return result


def _wall_clock_ns(wall_time_ns: Callable[[], int]) -> int:
    value = wall_time_ns()
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ManualShortGateError("wall clock returned an invalid nanosecond value")
    return value


def _evidence(path: Path, *, missing_reason: str | None) -> dict[str, object]:
    present = path.is_file()
    return {
        "path": str(path),
        "present": present,
        "missing_reason": None if present else missing_reason,
    }


def _read_json(path: Path) -> Mapping[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, Mapping):
        raise ValueError(f"JSON artifact is not an object: {path.name}")
    return value


def _read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _atomic_write_json(path: Path, document: Mapping[str, Any]) -> None:
    payload = _json_bytes(document)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            descriptor = -1
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary, path)
        except FileExistsError as exc:
            raise ManualShortGateError(
                f"short-gate artifact already exists: {path.name}"
            ) from exc
        except OSError as exc:
            raise ManualShortGateError(
                f"cannot publish short-gate artifact: {path.name}"
            ) from exc
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        temporary.unlink(missing_ok=True)


def _json_bytes(document: Mapping[str, Any]) -> bytes:
    try:
        return (
            json.dumps(
                document,
                ensure_ascii=False,
                sort_keys=True,
                indent=2,
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise ManualShortGateError("short-gate artifact is not strict JSON") from exc


def _canonical_directory(path: Path, label: str) -> Path:
    candidate = Path(path)
    if not candidate.is_absolute():
        raise ValueError(f"{label} must be absolute")
    resolved = candidate.resolve(strict=True)
    if candidate != resolved or not resolved.is_dir():
        raise ValueError(f"{label} must be an existing canonical directory")
    return resolved


def _canonical_file(path: Path, label: str) -> Path:
    candidate = Path(path)
    if not candidate.is_absolute():
        raise ValueError(f"{label} must be absolute")
    resolved = candidate.resolve(strict=True)
    if candidate != resolved or not resolved.is_file():
        raise ValueError(f"{label} must be an existing canonical file")
    return resolved


__all__ = [
    "ManualShortGateConfig",
    "ManualShortGateError",
    "ManualShortGateResult",
    "supervise_manual_short_gate",
]

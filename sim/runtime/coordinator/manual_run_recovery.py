"""Explicit recovery for abandoned manual diagnostic run allocations."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import socket
import stat
import sys
import tempfile
import time
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from sim.runtime.coordinator.atomic_file import replace_file_with_retry
from sim.runtime.coordinator.run_allocation import RunAllocation, load_run_allocation
from sim.runtime.recording import EpisodeResult, EpisodeStatus

_MANUAL_RUN_ID_RE = re.compile(r"manual-[A-Za-z0-9_.-]{1,121}\Z")
_TERMINAL_STATES = {"STOPPED", "FAILED"}
_REPARSE_POINT = 0x0400
_RECOVERY_SCHEMA = "lingtu.sim.stale-allocation-recovery.v1"
_RUNTIME_SCHEMA = "lingtu.sim.session-runtime.v1"
_HEALTH_SCHEMA = "lingtu.sim.manual-runtime-health.v1"
_RECOVERY_FILENAME = "stale-allocation-recovery.json"
_EPISODE_FILENAME = "episode_result.json"
_RUNTIME_FILENAME = "session.runtime.json"
_HEALTH_FILENAME = "manual-runtime-health.json"
_TRUTH_FILENAME = "truth-snapshots.jsonl"
_SHUTDOWN_FILENAME = "shutdown-evidence.json"
_DEFAULT_STABLE_SAMPLE_DELAY_S = 0.05
_KNOWN_PROCESS_NAMES = {
    "UnrealEditor.exe",
    "UnrealEditor-Cmd.exe",
    "RobotSimUE.exe",
    "RobotSimUE-Win64-Release.exe",
    "RobotSimUE-Win64-Shipping.exe",
    "lingtu_mujoco_headless.exe",
    "lingtu_mujoco_driver_bridge.exe",
    "lingtu_imu_publisher.exe",
    "lingtu_truth_odom_publisher.exe",
    "lingtu_mujoco_sensor_publisher.exe",
    "UnrealBuildTool.exe",
    "AutomationTool.exe",
    "dotnet.exe",
    "cl.exe",
    "link.exe",
    "msbuild.exe",
}
_KNOWN_PROCESS_NAMES_CASEFOLD = {name.casefold() for name in _KNOWN_PROCESS_NAMES}
_ERROR_ACCESS_DENIED = 5
_ERROR_FILE_NOT_FOUND = 2
_ERROR_INVALID_PARAMETER = 87
_ERROR_NO_MORE_FILES = 18


class ManualRunRecoveryError(RuntimeError):
    """Stable fail-closed error for manual abandoned-run recovery."""


@dataclass(frozen=True)
class ManualRunRecoveryResult:
    """Outcome of one explicit manual abandoned-run recovery attempt."""

    run_dir: Path
    run_id: str
    recovered: bool
    already_terminal: bool = False
    recovery_path: Path | None = None
    episode_path: Path | None = None
    runtime_path: Path | None = None


@dataclass(frozen=True)
class ManualRunRecoveryProbes:
    """OS probes used by the production CLI and replaceable in tests."""

    process_exists: Callable[[int], bool]
    known_processes_clean: Callable[[], Mapping[str, Any]]
    udp_ports_available: Callable[[Iterable[int]], Mapping[str, Any]]
    camera_shm_available: Callable[[Iterable[str]], Mapping[str, Any]]
    sleep: Callable[[float], object] = time.sleep
    now_ns: Callable[[], int] = time.time_ns


@dataclass(frozen=True)
class _FileSample:
    path: Path
    identity: tuple[int, int, int, int]
    size: int
    mtime_ns: int
    sha256: str
    payload: bytes


def recover_manual_abandoned_run(
    *,
    run_dir: Path,
    expected_run_id: str,
    expected_session_id: str,
    expected_dds_domain: int,
    stale_after_s: float = 30.0,
    reason: str = "manual abandoned run recovery: stale non-terminal manual runtime was fail-closed",
    probes: ManualRunRecoveryProbes | None = None,
    stop_before_manifest_replace_for_test: bool = False,
) -> ManualRunRecoveryResult:
    """Fail-close one explicit abandoned manual run and release its allocation.

    This function never scans for candidates and never mutates the immutable
    ``run-allocation.json``.  It only terminalizes the exact manual diagnostic
    run named by the caller after proving that the recorded resources are dead.
    """

    resolved_run_dir = _validated_run_dir(run_dir, expected_run_id)
    session_id = _validated_session_id(expected_session_id)
    if expected_dds_domain < 0 or expected_dds_domain > 232:
        raise ManualRunRecoveryError("expected dds_domain must be in [0, 232]")
    if not reason.strip():
        raise ManualRunRecoveryError("recovery reason must be non-empty")
    probe_set = default_manual_run_recovery_probes() if probes is None else probes

    paths = {
        "allocation": resolved_run_dir / "run-allocation.json",
        "runtime": resolved_run_dir / _RUNTIME_FILENAME,
        "health": resolved_run_dir / _HEALTH_FILENAME,
        "truth": resolved_run_dir / _TRUTH_FILENAME,
    }
    first_samples = _sample_required_files(paths.values())
    probe_set.sleep(_DEFAULT_STABLE_SAMPLE_DELAY_S)
    second_samples = _sample_required_files(paths.values())
    _require_same_samples(first_samples, second_samples)

    allocation = load_run_allocation(paths["allocation"])
    runtime = _read_json_object_from_sample(second_samples[paths["runtime"]], "runtime manifest")
    health = _read_json_object_from_sample(second_samples[paths["health"]], "manual runtime health")
    _validate_allocation_identity(
        allocation,
        expected_run_id=expected_run_id,
        expected_session_id=session_id,
        expected_dds_domain=expected_dds_domain,
    )
    _validate_runtime_identity(runtime, allocation)

    state = runtime.get("state")
    if state in _TERMINAL_STATES:
        return ManualRunRecoveryResult(
            run_dir=resolved_run_dir,
            run_id=expected_run_id,
            recovered=False,
            already_terminal=True,
            recovery_path=resolved_run_dir / _RECOVERY_FILENAME,
            episode_path=resolved_run_dir / _EPISODE_FILENAME,
            runtime_path=paths["runtime"],
        )
    _validate_health_identity(health, allocation, runtime)
    _validate_truth_identity(second_samples[paths["truth"]], allocation)
    if not isinstance(state, str) or not state:
        raise ManualRunRecoveryError("runtime state is invalid")
    if (resolved_run_dir / _SHUTDOWN_FILENAME).exists():
        raise ManualRunRecoveryError("shutdown evidence exists for non-terminal manual run")

    recovery_path = resolved_run_dir / _RECOVERY_FILENAME
    episode_path = resolved_run_dir / _EPISODE_FILENAME
    existing_recovery = _read_optional_json(recovery_path)
    existing_episode = _read_optional_json(episode_path)
    if existing_episode is not None and existing_recovery is None:
        raise ManualRunRecoveryError("existing episode_result.json has no matching stale-allocation-recovery")

    timestamp_ns = _recovery_timestamp_or_new(existing_recovery, probe_set)
    clean_probes = _prove_resources_clean(allocation, runtime, probe_set)
    _require_health_stale(health, now_ns=timestamp_ns, stale_after_s=stale_after_s)

    recovery = _recovery_document(
        allocation=allocation,
        runtime=runtime,
        runtime_sample=second_samples[paths["runtime"]],
        health_sample=second_samples[paths["health"]],
        truth_sample=second_samples[paths["truth"]],
        reason=reason,
        timestamp_ns=timestamp_ns,
        clean_probes=clean_probes,
    )
    episode = EpisodeResult(
        run_id=allocation.run_id,
        session_id=allocation.session_id,
        model_generation=_non_negative_int(runtime.get("model_generation", 0), "model_generation"),
        reset_generation=_non_negative_int(runtime.get("reset_generation", 0), "reset_generation"),
        start_sim_time_ns=0,
        end_sim_time_ns=_runtime_sim_time_ns(runtime),
        status=EpisodeStatus.FAILED,
        failure_reason=reason,
        artifact_references={
            "run_allocation": "run-allocation.json",
            "runtime_manifest": _RUNTIME_FILENAME,
            "stale_allocation_recovery": _RECOVERY_FILENAME,
        },
    ).to_dict()

    _write_new_or_matching_json(recovery_path, recovery, label="stale-allocation-recovery")
    _write_new_or_matching_json(episode_path, episode, label="episode_result")
    if stop_before_manifest_replace_for_test:
        return ManualRunRecoveryResult(
            run_dir=resolved_run_dir,
            run_id=expected_run_id,
            recovered=False,
            recovery_path=recovery_path,
            episode_path=episode_path,
            runtime_path=paths["runtime"],
        )

    third_samples = _sample_required_files(paths.values())
    _require_same_samples(second_samples, third_samples)
    _prove_resources_clean(allocation, runtime, probe_set)
    failed_runtime = dict(runtime)
    failed_runtime["state"] = "FAILED"
    _atomic_replace_json(paths["runtime"], failed_runtime, expected_sample=third_samples[paths["runtime"]])
    return ManualRunRecoveryResult(
        run_dir=resolved_run_dir,
        run_id=expected_run_id,
        recovered=True,
        recovery_path=recovery_path,
        episode_path=episode_path,
        runtime_path=paths["runtime"],
    )


def recover_manual_abandoned_startup(
    *,
    run_dir: Path,
    expected_run_id: str,
    expected_session_id: str,
    expected_dds_domain: int,
    stale_after_s: float = 30.0,
    reason: str = (
        "manual abandoned startup recovery: PREPARING runtime was terminated "
        "before health and truth journals were created"
    ),
    probes: ManualRunRecoveryProbes | None = None,
) -> ManualRunRecoveryResult:
    """Fail-close an exact PREPARING run terminated before journals existed.

    This narrow recovery path never invents missing health or truth evidence.
    Both startup journals must remain absent across the stable inspection, and
    all recorded processes, ports, and shared-memory names must already be
    released before the runtime manifest can become terminal.
    """

    resolved_run_dir = _validated_run_dir(run_dir, expected_run_id)
    session_id = _validated_session_id(expected_session_id)
    if expected_dds_domain < 0 or expected_dds_domain > 232:
        raise ManualRunRecoveryError("expected dds_domain must be in [0, 232]")
    if not reason.strip():
        raise ManualRunRecoveryError("recovery reason must be non-empty")
    probe_set = default_manual_run_recovery_probes() if probes is None else probes

    paths = {
        "allocation": resolved_run_dir / "run-allocation.json",
        "runtime": resolved_run_dir / _RUNTIME_FILENAME,
        "health": resolved_run_dir / _HEALTH_FILENAME,
        "truth": resolved_run_dir / _TRUTH_FILENAME,
    }
    _require_startup_journals_absent(paths["health"], paths["truth"])
    first_samples = _sample_required_files((paths["allocation"], paths["runtime"]))
    probe_set.sleep(_DEFAULT_STABLE_SAMPLE_DELAY_S)
    _require_startup_journals_absent(paths["health"], paths["truth"])
    second_samples = _sample_required_files((paths["allocation"], paths["runtime"]))
    _require_same_samples(first_samples, second_samples)

    allocation = load_run_allocation(paths["allocation"])
    runtime = _read_json_object_from_sample(second_samples[paths["runtime"]], "runtime manifest")
    _validate_allocation_identity(
        allocation,
        expected_run_id=expected_run_id,
        expected_session_id=session_id,
        expected_dds_domain=expected_dds_domain,
    )
    _validate_runtime_identity(runtime, allocation)
    state = runtime.get("state")
    if state in _TERMINAL_STATES:
        return ManualRunRecoveryResult(
            run_dir=resolved_run_dir,
            run_id=expected_run_id,
            recovered=False,
            already_terminal=True,
            recovery_path=resolved_run_dir / _RECOVERY_FILENAME,
            episode_path=resolved_run_dir / _EPISODE_FILENAME,
            runtime_path=paths["runtime"],
        )
    if state != "PREPARING":
        raise ManualRunRecoveryError(
            "startup-before-journals recovery requires runtime state PREPARING"
        )
    if (resolved_run_dir / _SHUTDOWN_FILENAME).exists():
        raise ManualRunRecoveryError("shutdown evidence exists for non-terminal manual run")

    recovery_path = resolved_run_dir / _RECOVERY_FILENAME
    episode_path = resolved_run_dir / _EPISODE_FILENAME
    existing_recovery = _read_optional_json(recovery_path)
    existing_episode = _read_optional_json(episode_path)
    if existing_episode is not None and existing_recovery is None:
        raise ManualRunRecoveryError(
            "existing episode_result.json has no matching stale-allocation-recovery"
        )

    timestamp_ns = _recovery_timestamp_or_new(existing_recovery, probe_set)
    clean_probes = _prove_resources_clean(allocation, runtime, probe_set)
    _require_startup_runtime_stale(
        second_samples[paths["runtime"]],
        now_ns=timestamp_ns,
        stale_after_s=stale_after_s,
    )
    recovery = _startup_recovery_document(
        allocation=allocation,
        runtime=runtime,
        allocation_sample=second_samples[paths["allocation"]],
        runtime_sample=second_samples[paths["runtime"]],
        reason=reason,
        timestamp_ns=timestamp_ns,
        clean_probes=clean_probes,
    )
    episode = EpisodeResult(
        run_id=allocation.run_id,
        session_id=allocation.session_id,
        model_generation=_non_negative_int(
            runtime.get("model_generation", 0), "model_generation"
        ),
        reset_generation=_non_negative_int(
            runtime.get("reset_generation", 0), "reset_generation"
        ),
        start_sim_time_ns=0,
        end_sim_time_ns=_runtime_sim_time_ns(runtime),
        status=EpisodeStatus.FAILED,
        failure_reason=reason,
        artifact_references={
            "run_allocation": "run-allocation.json",
            "runtime_manifest": _RUNTIME_FILENAME,
            "stale_allocation_recovery": _RECOVERY_FILENAME,
        },
    ).to_dict()

    _write_new_or_matching_json(recovery_path, recovery, label="stale-allocation-recovery")
    _write_new_or_matching_json(episode_path, episode, label="episode_result")
    _require_startup_journals_absent(paths["health"], paths["truth"])
    third_samples = _sample_required_files((paths["allocation"], paths["runtime"]))
    _require_same_samples(second_samples, third_samples)
    _prove_resources_clean(allocation, runtime, probe_set)
    failed_runtime = dict(runtime)
    failed_runtime["state"] = "FAILED"
    _atomic_replace_json(
        paths["runtime"],
        failed_runtime,
        expected_sample=third_samples[paths["runtime"]],
    )
    return ManualRunRecoveryResult(
        run_dir=resolved_run_dir,
        run_id=expected_run_id,
        recovered=True,
        recovery_path=recovery_path,
        episode_path=episode_path,
        runtime_path=paths["runtime"],
    )


def default_manual_run_recovery_probes() -> ManualRunRecoveryProbes:
    """Return production OS probes for explicit manual recovery."""

    return ManualRunRecoveryProbes(
        process_exists=_process_exists,
        known_processes_clean=_known_processes_clean,
        udp_ports_available=_udp_ports_available,
        camera_shm_available=_camera_shm_available,
    )


def main(argv: Sequence[str] | None = None) -> int:
    """Run the explicit manual abandoned-run recovery CLI."""

    parser = argparse.ArgumentParser(
        description="Fail-close one explicit abandoned manual diagnostic run."
    )
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--expected-run-id", required=True)
    parser.add_argument("--expected-session-id", required=True)
    parser.add_argument("--expected-dds-domain", required=True, type=int)
    parser.add_argument("--stale-after-seconds", type=float, default=30.0)
    parser.add_argument(
        "--startup-before-journals",
        action="store_true",
        help="recover only an exact PREPARING run with both startup journals absent",
    )
    parser.add_argument(
        "--reason",
        default="manual abandoned run recovery: stale non-terminal manual runtime was fail-closed",
    )
    arguments = parser.parse_args(argv)
    try:
        recovery = (
            recover_manual_abandoned_startup
            if arguments.startup_before_journals
            else recover_manual_abandoned_run
        )
        result = recovery(
            run_dir=arguments.run_dir,
            expected_run_id=arguments.expected_run_id,
            expected_session_id=arguments.expected_session_id,
            expected_dds_domain=arguments.expected_dds_domain,
            stale_after_s=arguments.stale_after_seconds,
            reason=arguments.reason,
        )
    except ManualRunRecoveryError as exc:
        print(f"manual run recovery failed: {exc}", file=sys.stderr)
        return 1
    except Exception:
        print("manual run recovery failed", file=sys.stderr)
        return 1
    print(
        json.dumps(
            {
                "schema": "lingtu.sim.manual-run-recovery-cli-result.v1",
                "run_id": result.run_id,
                "run_dir": str(result.run_dir),
                "recovered": result.recovered,
                "already_terminal": result.already_terminal,
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 0


def _validated_run_dir(run_dir: Path, expected_run_id: str) -> Path:
    if _MANUAL_RUN_ID_RE.fullmatch(expected_run_id) is None:
        raise ManualRunRecoveryError("expected run_id must be a manual-* run")
    requested = Path(run_dir)
    if not requested.is_absolute():
        raise ManualRunRecoveryError("run_dir must be an absolute canonical path")
    try:
        metadata = os.lstat(requested)
        resolved = requested.resolve(strict=True)
    except OSError as exc:
        raise ManualRunRecoveryError("run_dir must exist") from exc
    if not stat.S_ISDIR(metadata.st_mode) or _is_reparse_metadata(metadata):
        raise ManualRunRecoveryError("run_dir must be a plain directory, not a link or reparse point")
    if os.path.normcase(os.path.normpath(os.fspath(resolved))) != os.path.normcase(
        os.path.normpath(os.fspath(requested))
    ):
        raise ManualRunRecoveryError("run_dir must be an absolute canonical path")
    if resolved.name != expected_run_id:
        raise ManualRunRecoveryError("run_dir basename must match expected run_id")
    return resolved


def _validated_session_id(value: str) -> str:
    session_id = value.strip()
    if not session_id or session_id != value:
        raise ManualRunRecoveryError("expected session_id mismatch")
    return session_id


def _strict_json(text: str) -> Any:
    def object_from_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key: {key}")
            result[key] = value
        return result

    def reject_constant(value: str) -> Any:
        raise ValueError(f"non-finite JSON value: {value}")

    return json.loads(text, object_pairs_hook=object_from_pairs, parse_constant=reject_constant)


def _is_reparse_metadata(metadata: os.stat_result) -> bool:
    if stat.S_ISLNK(metadata.st_mode):
        return True
    return bool(getattr(metadata, "st_file_attributes", 0) & _REPARSE_POINT)


def _metadata_identity(metadata: os.stat_result) -> tuple[int, int, int, int]:
    return (
        int(getattr(metadata, "st_dev", 0)),
        int(getattr(metadata, "st_ino", 0)),
        int(metadata.st_size),
        int(metadata.st_mtime_ns),
    )


def _sample_required_files(paths: Iterable[Path]) -> dict[Path, _FileSample]:
    samples: dict[Path, _FileSample] = {}
    for path in paths:
        try:
            metadata = os.lstat(path)
        except OSError as exc:
            raise ManualRunRecoveryError(f"required recovery file is missing: {path.name}") from exc
        if not stat.S_ISREG(metadata.st_mode) or _is_reparse_metadata(metadata):
            raise ManualRunRecoveryError(f"required recovery file is not a plain file: {path.name}")
        payload = path.read_bytes()
        samples[path] = _FileSample(
            path=path,
            identity=_metadata_identity(metadata),
            size=len(payload),
            mtime_ns=int(metadata.st_mtime_ns),
            sha256=hashlib.sha256(payload).hexdigest(),
            payload=payload,
        )
    return samples


def _require_same_samples(
    first: Mapping[Path, _FileSample],
    second: Mapping[Path, _FileSample],
) -> None:
    if set(first) != set(second):
        raise ManualRunRecoveryError("required files changed during recovery inspection")
    for path, left in first.items():
        right = second[path]
        if (
            left.identity != right.identity
            or left.size != right.size
            or left.mtime_ns != right.mtime_ns
            or left.sha256 != right.sha256
        ):
            raise ManualRunRecoveryError(f"{path.name} changed during recovery inspection")


def _require_startup_journals_absent(health_path: Path, truth_path: Path) -> None:
    present: list[str] = []
    for path in (health_path, truth_path):
        try:
            os.lstat(path)
        except FileNotFoundError:
            continue
        except OSError as exc:
            raise ManualRunRecoveryError(
                f"cannot inspect startup journal: {path.name}"
            ) from exc
        present.append(path.name)
    if present:
        raise ManualRunRecoveryError(
            "startup health and truth journals must both be absent"
        )


def _read_json_object_from_sample(sample: _FileSample, label: str) -> dict[str, Any]:
    try:
        value = _strict_json(sample.payload.decode("utf-8"))
    except (UnicodeError, ValueError) as exc:
        raise ManualRunRecoveryError(f"{label} is not strict JSON") from exc
    if type(value) is not dict:
        raise ManualRunRecoveryError(f"{label} must be a JSON object")
    return value


def _read_optional_json(path: Path) -> dict[str, Any] | None:
    try:
        payload = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return None
    except OSError as exc:
        raise ManualRunRecoveryError(f"cannot read existing artifact: {path.name}") from exc
    try:
        value = _strict_json(payload)
    except ValueError as exc:
        raise ManualRunRecoveryError(f"existing {path.name} is not strict JSON") from exc
    if type(value) is not dict:
        raise ManualRunRecoveryError(f"existing {path.name} must be a JSON object")
    return value


def _validate_allocation_identity(
    allocation: RunAllocation,
    *,
    expected_run_id: str,
    expected_session_id: str,
    expected_dds_domain: int,
) -> None:
    if allocation.run_id != expected_run_id:
        raise ManualRunRecoveryError("allocation run_id mismatch")
    if allocation.session_id != expected_session_id:
        raise ManualRunRecoveryError("expected session_id mismatch")
    if allocation.dds_domain != expected_dds_domain:
        raise ManualRunRecoveryError("allocation dds_domain mismatch")


def _validate_runtime_identity(runtime: Mapping[str, Any], allocation: RunAllocation) -> None:
    if runtime.get("schema") != _RUNTIME_SCHEMA:
        raise ManualRunRecoveryError("runtime manifest schema mismatch")
    for field, expected in (
        ("run_id", allocation.run_id),
        ("session_id", allocation.session_id),
    ):
        if runtime.get(field) != expected:
            raise ManualRunRecoveryError(f"runtime {field} mismatch")
    runtime_allocation = runtime.get("allocation")
    if type(runtime_allocation) is not dict:
        raise ManualRunRecoveryError("runtime allocation must be an object")
    expected_allocation: Mapping[str, object] = {
        "run_dir": str(allocation.run_dir),
        "log_dir": str(allocation.log_dir),
        "boot_id": allocation.boot_id,
        "dds_domain": allocation.dds_domain,
        "ports": dict(allocation.ports),
        "shm": dict(allocation.shm),
    }
    for allocation_field, allocation_expected in expected_allocation.items():
        if runtime_allocation.get(allocation_field) != allocation_expected:
            raise ManualRunRecoveryError(
                f"runtime allocation {allocation_field} mismatch"
            )


def _validate_health_identity(
    health: Mapping[str, Any],
    allocation: RunAllocation,
    runtime: Mapping[str, Any],
) -> None:
    if health.get("schema") != _HEALTH_SCHEMA:
        raise ManualRunRecoveryError("manual runtime health schema mismatch")
    if health.get("evidence_class") != "manual_diagnostic_only" or health.get("qualification") is not False:
        raise ManualRunRecoveryError("manual runtime health identity mismatch")
    for field, expected in (
        ("run_id", allocation.run_id),
        ("session_id", allocation.session_id),
        ("session_state", runtime.get("state")),
        ("model_generation", runtime.get("model_generation")),
        ("reset_generation", runtime.get("reset_generation")),
    ):
        if field in health and health.get(field) != expected:
            raise ManualRunRecoveryError(f"manual runtime health {field} mismatch")


def _validate_truth_identity(sample: _FileSample, allocation: RunAllocation) -> None:
    text = sample.payload.decode("utf-8")
    if not text.strip():
        raise ManualRunRecoveryError("truth snapshots are empty")
    for line in reversed(text.splitlines()):
        if not line.strip():
            continue
        try:
            value = _strict_json(line)
        except ValueError as exc:
            raise ManualRunRecoveryError("truth snapshots must be JSON lines") from exc
        if type(value) is not dict:
            raise ManualRunRecoveryError("truth snapshot line must be an object")
        for field, expected in (
            ("run_id", allocation.run_id),
            ("session_id", allocation.session_id),
        ):
            if field in value and value.get(field) != expected:
                raise ManualRunRecoveryError(f"truth snapshot {field} mismatch")
        return
    raise ManualRunRecoveryError("truth snapshots are empty")


def _require_health_stale(
    health: Mapping[str, Any],
    *,
    now_ns: int,
    stale_after_s: float,
) -> None:
    if stale_after_s <= 0:
        raise ManualRunRecoveryError("stale_after_s must be positive")
    updated_ns = _non_negative_int(health.get("updated_unix_ns"), "updated_unix_ns")
    if now_ns - updated_ns < int(stale_after_s * 1_000_000_000):
        raise ManualRunRecoveryError("manual runtime health is not stale enough for recovery")


def _require_startup_runtime_stale(
    runtime_sample: _FileSample,
    *,
    now_ns: int,
    stale_after_s: float,
) -> None:
    if stale_after_s <= 0:
        raise ManualRunRecoveryError("stale_after_s must be positive")
    if now_ns - runtime_sample.mtime_ns < int(stale_after_s * 1_000_000_000):
        raise ManualRunRecoveryError(
            "manual PREPARING runtime is not stale enough for startup recovery"
        )


def _prove_resources_clean(
    allocation: RunAllocation,
    runtime: Mapping[str, Any],
    probes: ManualRunRecoveryProbes,
) -> dict[str, Any]:
    runtime_allocation = runtime.get("allocation")
    if type(runtime_allocation) is not dict:
        raise ManualRunRecoveryError("runtime allocation must be an object")
    physics_pid = _optional_pid(runtime_allocation.get("physics_pid"), "physics_pid")
    if physics_pid is None:
        raise ManualRunRecoveryError("runtime allocation physics_pid is required for abandoned recovery")
    if probes.process_exists(physics_pid):
        raise ManualRunRecoveryError(f"physics_pid is still alive: {physics_pid}")
    process_probe = dict(probes.known_processes_clean())
    if process_probe.get("clean") is not True:
        raise ManualRunRecoveryError("known runtime/build processes are still running")
    port_probe = dict(probes.udp_ports_available(allocation.ports.values()))
    if port_probe.get("available") is not True:
        raise ManualRunRecoveryError("UDP ports are still busy")
    shm_probe = dict(probes.camera_shm_available(allocation.shm.values()))
    if shm_probe.get("available") is not True:
        raise ManualRunRecoveryError("camera SHM names are still busy")
    return {
        "known_processes": process_probe,
        "udp_ports": port_probe,
        "camera_shm": shm_probe,
    }


def _recovery_document(
    *,
    allocation: RunAllocation,
    runtime: Mapping[str, Any],
    runtime_sample: _FileSample,
    health_sample: _FileSample,
    truth_sample: _FileSample,
    reason: str,
    timestamp_ns: int,
    clean_probes: Mapping[str, Any],
) -> dict[str, Any]:
    runtime_allocation = runtime.get("allocation")
    if type(runtime_allocation) is not dict:
        raise ManualRunRecoveryError("runtime allocation must be an object")
    physics_pid = _optional_pid(runtime_allocation.get("physics_pid"), "physics_pid")
    return {
        "schema": _RECOVERY_SCHEMA,
        "manual_diagnostic_only": True,
        "qualification": False,
        "reason": reason,
        "run_id": allocation.run_id,
        "session_id": allocation.session_id,
        "dds_domain": allocation.dds_domain,
        "recovered_from_state": runtime.get("state"),
        "identity": {
            "run_dir": str(allocation.run_dir),
            "log_dir": str(allocation.log_dir),
            "boot_id": allocation.boot_id,
            "ports": dict(allocation.ports),
            "shm": dict(allocation.shm),
        },
        "original_runtime_manifest": {
            "path": _RUNTIME_FILENAME,
            "bytes": runtime_sample.size,
            "sha256": runtime_sample.sha256,
        },
        "observed_dead_pids": {"physics_pid": physics_pid},
        "stable_files": {
            _RUNTIME_FILENAME: _sample_document(runtime_sample),
            _HEALTH_FILENAME: _sample_document(health_sample),
            _TRUTH_FILENAME: _sample_document(truth_sample),
        },
        "clean_probes": clean_probes,
        "timestamp_unix_ns": timestamp_ns,
    }


def _startup_recovery_document(
    *,
    allocation: RunAllocation,
    runtime: Mapping[str, Any],
    allocation_sample: _FileSample,
    runtime_sample: _FileSample,
    reason: str,
    timestamp_ns: int,
    clean_probes: Mapping[str, Any],
) -> dict[str, Any]:
    runtime_allocation = runtime.get("allocation")
    if type(runtime_allocation) is not dict:
        raise ManualRunRecoveryError("runtime allocation must be an object")
    physics_pid = _optional_pid(runtime_allocation.get("physics_pid"), "physics_pid")
    return {
        "schema": _RECOVERY_SCHEMA,
        "manual_diagnostic_only": True,
        "qualification": False,
        "reason": reason,
        "run_id": allocation.run_id,
        "session_id": allocation.session_id,
        "dds_domain": allocation.dds_domain,
        "recovered_from_state": runtime.get("state"),
        "startup_journals_missing": [_HEALTH_FILENAME, _TRUTH_FILENAME],
        "identity": {
            "run_dir": str(allocation.run_dir),
            "log_dir": str(allocation.log_dir),
            "boot_id": allocation.boot_id,
            "ports": dict(allocation.ports),
            "shm": dict(allocation.shm),
        },
        "original_runtime_manifest": {
            "path": _RUNTIME_FILENAME,
            "bytes": runtime_sample.size,
            "sha256": runtime_sample.sha256,
        },
        "observed_dead_pids": {"physics_pid": physics_pid},
        "stable_files": {
            "run-allocation.json": _sample_document(allocation_sample),
            _RUNTIME_FILENAME: _sample_document(runtime_sample),
        },
        "clean_probes": clean_probes,
        "timestamp_unix_ns": timestamp_ns,
    }


def _sample_document(sample: _FileSample) -> dict[str, Any]:
    return {
        "bytes": sample.size,
        "mtime_ns": sample.mtime_ns,
        "sha256": sample.sha256,
    }


def _runtime_sim_time_ns(runtime: Mapping[str, Any]) -> int:
    clock = runtime.get("clock")
    if type(clock) is not dict:
        return 0
    return _non_negative_int(clock.get("sim_time_ns", 0), "sim_time_ns")


def _recovery_timestamp_or_new(
    existing_recovery: Mapping[str, Any] | None,
    probes: ManualRunRecoveryProbes,
) -> int:
    if existing_recovery is None:
        return _non_negative_int(probes.now_ns(), "timestamp_unix_ns")
    if existing_recovery.get("schema") != _RECOVERY_SCHEMA:
        raise ManualRunRecoveryError("existing stale-allocation-recovery artifact conflicts with recovery")
    try:
        return _non_negative_int(existing_recovery.get("timestamp_unix_ns"), "timestamp_unix_ns")
    except ManualRunRecoveryError as exc:
        raise ManualRunRecoveryError(
            "existing stale-allocation-recovery artifact conflicts with recovery"
        ) from exc


def _non_negative_int(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ManualRunRecoveryError(f"{field} must be a non-negative integer")
    return value


def _optional_pid(value: Any, field: str) -> int | None:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ManualRunRecoveryError(f"{field} must be a positive integer")
    return value


def _write_new_or_matching_json(path: Path, payload: Mapping[str, Any], *, label: str) -> None:
    encoded = _json_bytes(payload)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
    )
    temporary = Path(temporary_name)
    try:
        stream = os.fdopen(descriptor, "wb")
        descriptor = -1
        with stream as handle:
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())
        try:
            os.link(temporary, path)
        except FileExistsError:
            existing = _sample_existing_artifact_for_match(path, label).payload
            if existing != encoded:
                raise ManualRunRecoveryError(
                    f"existing {label} artifact conflicts with recovery"
                ) from None
    except OSError as exc:
        raise ManualRunRecoveryError(f"cannot write {label} artifact") from exc
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        temporary.unlink(missing_ok=True)


def _sample_existing_artifact_for_match(path: Path, label: str) -> _FileSample:
    try:
        before = os.lstat(path)
    except OSError as exc:
        raise ManualRunRecoveryError(f"cannot inspect existing {label} artifact") from exc
    if not stat.S_ISREG(before.st_mode) or _is_reparse_metadata(before):
        raise ManualRunRecoveryError(f"existing {label} artifact is not a plain file")
    if int(getattr(before, "st_nlink", 1)) != 1:
        raise ManualRunRecoveryError(f"existing {label} artifact has multiple links")
    payload = path.read_bytes()
    try:
        after = os.lstat(path)
    except OSError as exc:
        raise ManualRunRecoveryError(f"cannot inspect existing {label} artifact") from exc
    if _metadata_identity(before) != _metadata_identity(after):
        raise ManualRunRecoveryError(f"existing {label} artifact changed while inspecting")
    return _FileSample(
        path=path,
        identity=_metadata_identity(after),
        size=len(payload),
        mtime_ns=int(after.st_mtime_ns),
        sha256=hashlib.sha256(payload).hexdigest(),
        payload=payload,
    )


def _atomic_replace_json(
    path: Path,
    payload: Mapping[str, Any],
    *,
    expected_sample: _FileSample,
) -> None:
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
    )
    temporary = Path(temporary_name)
    try:
        stream = os.fdopen(descriptor, "wb")
        descriptor = -1
        with stream as handle:
            handle.write(_json_bytes(payload))
            handle.flush()
            os.fsync(handle.fileno())
        _require_path_matches_sample(path, expected_sample, "runtime manifest")
        replace_file_with_retry(temporary, path)
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        temporary.unlink(missing_ok=True)


def _require_path_matches_sample(path: Path, expected: _FileSample, label: str) -> None:
    try:
        metadata = os.lstat(path)
    except OSError as exc:
        raise ManualRunRecoveryError(f"{label} changed before terminal write") from exc
    if not stat.S_ISREG(metadata.st_mode) or _is_reparse_metadata(metadata):
        raise ManualRunRecoveryError(f"{label} is not a plain file before terminal write")
    current = _FileSample(
        path=path,
        identity=_metadata_identity(metadata),
        size=int(metadata.st_size),
        mtime_ns=int(metadata.st_mtime_ns),
        sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        payload=b"",
    )
    if (
        current.identity != expected.identity
        or current.size != expected.size
        or current.mtime_ns != expected.mtime_ns
        or current.sha256 != expected.sha256
    ):
        raise ManualRunRecoveryError(f"{label} changed before terminal write")


def _json_bytes(payload: Mapping[str, Any]) -> bytes:
    return (
        json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")


def _process_exists(pid: int) -> bool:
    if pid <= 0:
        return False
    if os.name == "nt":
        import ctypes
        from ctypes import wintypes

        process_query_limited_information = 0x1000
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        open_process = kernel32.OpenProcess
        open_process.argtypes = (wintypes.DWORD, wintypes.BOOL, wintypes.DWORD)
        open_process.restype = wintypes.HANDLE
        close_handle = kernel32.CloseHandle
        close_handle.argtypes = (wintypes.HANDLE,)
        close_handle.restype = wintypes.BOOL
        handle = open_process(process_query_limited_information, False, pid)
        if handle:
            close_handle(handle)
            return True
        handle_value = 0 if handle is None else int(handle)
        return _windows_open_process_exists_result(
            handle_value,
            ctypes.get_last_error(),
            pid=pid,
        )
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError as exc:
        raise ManualRunRecoveryError(f"cannot inspect process {pid}") from exc
    return True


def _windows_open_process_exists_result(handle: int, error: int, *, pid: int) -> bool:
    if handle:
        return True
    if error == _ERROR_INVALID_PARAMETER:
        return False
    if error == _ERROR_ACCESS_DENIED:
        raise ManualRunRecoveryError(f"cannot inspect process {pid}: access denied")
    raise ManualRunRecoveryError(f"cannot inspect process {pid}: Windows error {error}")


def _known_processes_clean() -> Mapping[str, Any]:
    if os.name != "nt":
        return {"clean": True, "processes": [], "platform": os.name}
    processes = _windows_toolhelp_processes()
    offenders: list[dict[str, Any]] = []
    for process in processes:
        name = str(process["name"])
        if name.casefold() in _KNOWN_PROCESS_NAMES_CASEFOLD:
            pid = process["pid"]
            if isinstance(pid, bool) or not isinstance(pid, int):
                raise ManualRunRecoveryError("cannot inspect Windows process table")
            offenders.append(
                {
                    "name": name,
                    "pid": pid,
                    "path": _windows_process_image_path(pid),
                }
            )
    return {"clean": not offenders, "processes": offenders}


def _windows_toolhelp_processes() -> list[dict[str, object]]:
    import ctypes
    from ctypes import wintypes

    th32cs_snapprocess = 0x00000002
    invalid_handle_value = ctypes.c_void_p(-1).value

    class ProcessEntry32W(ctypes.Structure):
        _fields_ = [
            ("dwSize", wintypes.DWORD),
            ("cntUsage", wintypes.DWORD),
            ("th32ProcessID", wintypes.DWORD),
            ("th32DefaultHeapID", ctypes.POINTER(wintypes.ULONG)),
            ("th32ModuleID", wintypes.DWORD),
            ("cntThreads", wintypes.DWORD),
            ("th32ParentProcessID", wintypes.DWORD),
            ("pcPriClassBase", wintypes.LONG),
            ("dwFlags", wintypes.DWORD),
            ("szExeFile", wintypes.WCHAR * 260),
        ]

    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    create_snapshot = kernel32.CreateToolhelp32Snapshot
    create_snapshot.argtypes = (wintypes.DWORD, wintypes.DWORD)
    create_snapshot.restype = wintypes.HANDLE
    process_first = kernel32.Process32FirstW
    process_first.argtypes = (wintypes.HANDLE, ctypes.POINTER(ProcessEntry32W))
    process_first.restype = wintypes.BOOL
    process_next = kernel32.Process32NextW
    process_next.argtypes = (wintypes.HANDLE, ctypes.POINTER(ProcessEntry32W))
    process_next.restype = wintypes.BOOL
    close_handle = kernel32.CloseHandle
    close_handle.argtypes = (wintypes.HANDLE,)
    close_handle.restype = wintypes.BOOL

    snapshot = create_snapshot(th32cs_snapprocess, 0)
    if snapshot == invalid_handle_value:
        error = ctypes.get_last_error()
        raise ManualRunRecoveryError("cannot inspect Windows process table") from OSError(
            error,
            os.strerror(error),
        )
    try:
        entry = ProcessEntry32W()
        entry.dwSize = ctypes.sizeof(ProcessEntry32W)
        processes: list[dict[str, object]] = []
        if not process_first(snapshot, ctypes.byref(entry)):
            error = ctypes.get_last_error()
            raise ManualRunRecoveryError("cannot read Windows process table") from OSError(
                error,
                os.strerror(error),
            )
        while True:
            processes.append(
                {
                    "pid": int(entry.th32ProcessID),
                    "name": str(entry.szExeFile),
                }
            )
            entry.dwSize = ctypes.sizeof(ProcessEntry32W)
            if not process_next(snapshot, ctypes.byref(entry)):
                _windows_process_next_finished_or_raise(ctypes.get_last_error())
                break
        return processes
    finally:
        close_handle(snapshot)


def _windows_process_next_finished_or_raise(error: int) -> None:
    if error == _ERROR_NO_MORE_FILES:
        return
    raise ManualRunRecoveryError("cannot read Windows process table")


def _windows_process_image_path(pid: int) -> str:
    import ctypes
    from ctypes import wintypes

    process_query_limited_information = 0x1000
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    open_process = kernel32.OpenProcess
    open_process.argtypes = (wintypes.DWORD, wintypes.BOOL, wintypes.DWORD)
    open_process.restype = wintypes.HANDLE
    query_image = kernel32.QueryFullProcessImageNameW
    query_image.argtypes = (
        wintypes.HANDLE,
        wintypes.DWORD,
        wintypes.LPWSTR,
        ctypes.POINTER(wintypes.DWORD),
    )
    query_image.restype = wintypes.BOOL
    close_handle = kernel32.CloseHandle
    close_handle.argtypes = (wintypes.HANDLE,)
    close_handle.restype = wintypes.BOOL

    handle = open_process(process_query_limited_information, False, pid)
    if not handle:
        error = ctypes.get_last_error()
        raise ManualRunRecoveryError(f"cannot inspect known runtime/build process path: {pid}") from OSError(
            error,
            os.strerror(error),
        )
    try:
        size = wintypes.DWORD(32768)
        buffer = ctypes.create_unicode_buffer(size.value)
        if not query_image(handle, 0, buffer, ctypes.byref(size)):
            error = ctypes.get_last_error()
            raise ManualRunRecoveryError(
                f"cannot inspect known runtime/build process path: {pid}"
            ) from OSError(error, os.strerror(error))
        return buffer.value
    finally:
        close_handle(handle)


def _udp_ports_available(ports: Iterable[int]) -> Mapping[str, Any]:
    checked: list[int] = []
    busy: list[int] = []
    for port in sorted({int(port) for port in ports}):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            sock.bind(("127.0.0.1", port))
            checked.append(port)
        except OSError:
            busy.append(port)
        finally:
            sock.close()
    return {"available": not busy, "ports": checked, "busy": busy}


def _camera_shm_available(names: Iterable[str]) -> Mapping[str, Any]:
    unique_names = sorted({str(name) for name in names})
    if os.name != "nt":
        return {"available": True, "names": unique_names, "platform": os.name}
    import ctypes
    from ctypes import wintypes

    file_map_read = 0x0004
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    open_mapping = kernel32.OpenFileMappingW
    open_mapping.argtypes = (wintypes.DWORD, wintypes.BOOL, wintypes.LPCWSTR)
    open_mapping.restype = wintypes.HANDLE
    close_handle = kernel32.CloseHandle
    close_handle.argtypes = (wintypes.HANDLE,)
    close_handle.restype = wintypes.BOOL
    busy: list[str] = []
    for name in unique_names:
        handle = open_mapping(file_map_read, False, name)
        if handle:
            close_handle(handle)
            busy.append(name)
            continue
        error = ctypes.get_last_error()
        if not _windows_open_file_mapping_absent_result(error):
            raise ManualRunRecoveryError(
                f"cannot inspect camera SHM mapping {name!r}: Windows error {error}"
            )
    return {"available": not busy, "names": unique_names, "busy": busy}


def _windows_open_file_mapping_absent_result(error: int) -> bool:
    if error == _ERROR_FILE_NOT_FOUND:
        return True
    if error == _ERROR_ACCESS_DENIED:
        raise ManualRunRecoveryError("cannot inspect camera SHM mapping: access denied")
    return False


if __name__ == "__main__":
    raise SystemExit(main())

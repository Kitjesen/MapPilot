"""RobotSimUE presentation session for one validated simulation recording."""

from __future__ import annotations

import json
import math
import re
import time
import uuid
from collections.abc import Callable, Mapping
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Protocol

from sim.runtime.coordinator.atomic_file import replace_file_with_retry
from sim.runtime.coordinator.coordinator import PhysicsPlan, load_physics_plan
from sim.runtime.coordinator.external_evidence import (
    EvidenceState,
    ExternalRuntimeEvidence,
)
from sim.runtime.coordinator.live_snapshot import UdpLoopbackSnapshotPublisher
from sim.runtime.coordinator.run_allocation import (
    RunAllocation,
    create_run_allocation,
    load_resolved_session_bundle,
)
from sim.runtime.coordinator.unreal_process import UnrealProcess

from .timeline import SimulationReplay, SimulationReplayError, replay_snapshots

VISUAL_REPLAY_RESULT_SCHEMA = "lingtu.sim.visual-replay-result.v1"
VISUAL_REPLAY_RESULT_FILENAME = "visual-replay-result.json"
_VISUAL_EVIDENCE_FILENAME = "visual-readiness.json"
_FIRST_FRAME_FILENAME = "visual-first-frame.png"
_CAPTURE_DIRECTORY = "replay-frames"
_VISUAL_SOURCE_ID = "robotsimue-visual"
_VISUAL_ACTIVE_BASIS = "truth_snapshot_applied_to_visual_bindings"
_UNREAL_LEVEL_RE = re.compile(r"^/Game/\S(?:.*\S)?$")
_PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


class VisualReplayError(RuntimeError):
    """Fail-closed visual replay setup, runtime, or evidence failure."""


class _VisualProcess(Protocol):
    @property
    def pid(self) -> int | None: ...

    def start(
        self,
        *,
        bundle_dir: Path,
        allocation: RunAllocation,
        plan: PhysicsPlan,
        snapshot_port: int,
    ) -> None: ...

    def poll(self) -> int | None: ...

    def terminate(self) -> None: ...


class _SnapshotPublisher(Protocol):
    def publish(self, event: Mapping[str, Any]) -> int: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class VisualReplayConfig:
    """Trusted local launch inputs, kept outside the recording artifact."""

    bundle_dir: Path
    recording_dir: Path
    repo_root: Path
    run_root: Path
    unreal_editor: Path
    uproject: Path
    run_id: str | None = None
    map_name: str | None = None
    snapshot_port: int = 25124
    dds_domain: int = 0
    rate: float = 1.0
    pace: bool = True
    ready_timeout_s: float = 180.0
    screenshot_timeout_s: float = 30.0
    frame_timeout_s: float = 30.0
    capture_every: int = 10
    maximum_frames: int = 12
    minimum_frames: int = 2
    session_camera_tag: str | None = None
    motion_camera_stable_id: str | None = None

    def __post_init__(self) -> None:
        for field in (
            "bundle_dir",
            "recording_dir",
            "repo_root",
            "run_root",
            "unreal_editor",
            "uproject",
        ):
            object.__setattr__(self, field, Path(getattr(self, field)).resolve())
        if self.run_id is not None and (
            not isinstance(self.run_id, str)
            or not self.run_id
            or self.run_id != self.run_id.strip()
        ):
            raise ValueError("run_id must be non-empty trimmed text when set")
        if self.map_name is not None:
            _unreal_level(self.map_name, "map_name")
        if (
            isinstance(self.snapshot_port, bool)
            or not isinstance(self.snapshot_port, int)
            or not 1 <= self.snapshot_port <= 65535
        ):
            raise ValueError("snapshot_port must be an integer in [1, 65535]")
        if (
            isinstance(self.dds_domain, bool)
            or not isinstance(self.dds_domain, int)
            or not 0 <= self.dds_domain <= 232
        ):
            raise ValueError("dds_domain must be an integer in [0, 232]")
        rate = _positive_finite(self.rate, "rate")
        object.__setattr__(self, "rate", rate)
        for field in ("ready_timeout_s", "screenshot_timeout_s", "frame_timeout_s"):
            object.__setattr__(
                self,
                field,
                _positive_finite(getattr(self, field), field),
            )
        for field in ("capture_every", "maximum_frames", "minimum_frames"):
            value = getattr(self, field)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{field} must be a positive integer")
        if self.minimum_frames > self.maximum_frames:
            raise ValueError("minimum_frames must not exceed maximum_frames")
        for field in ("session_camera_tag", "motion_camera_stable_id"):
            value = getattr(self, field)
            if value is not None and (
                not isinstance(value, str) or not value or value != value.strip()
            ):
                raise ValueError(f"{field} must be non-empty trimmed text when set")


@dataclass(frozen=True)
class VisualReplayResult:
    """Committed evidence for one bounded presentation-only replay."""

    status: str
    run_id: str
    session_id: str
    source_run_id: str
    replay_frames_presented: int
    replay_frames_dropped: int
    captured_frames: int
    first_frame_screenshot: Path
    capture_directory: Path
    visual_evidence: Path
    unreal_log: Path
    runtime_manifest: Path
    result_path: Path
    physics_process_launched: bool = False

    def to_dict(self) -> dict[str, Any]:
        """Return the strict result document written to the run directory."""

        return {
            "schema": VISUAL_REPLAY_RESULT_SCHEMA,
            "status": self.status,
            "run_id": self.run_id,
            "session_id": self.session_id,
            "source_run_id": self.source_run_id,
            "replay_frames_presented": self.replay_frames_presented,
            "replay_frames_dropped": self.replay_frames_dropped,
            "captured_frames": self.captured_frames,
            "first_frame_screenshot": str(self.first_frame_screenshot),
            "capture_directory": str(self.capture_directory),
            "visual_evidence": str(self.visual_evidence),
            "unreal_log": str(self.unreal_log),
            "runtime_manifest": str(self.runtime_manifest),
            "result_path": str(self.result_path),
            "physics_process_launched": False,
            "clock_authority": "recorded_mujoco",
        }


def run_visual_replay(
    config: VisualReplayConfig,
    *,
    unreal_factory: Callable[..., _VisualProcess] = UnrealProcess,
    publisher_factory: Callable[[int], _SnapshotPublisher] = UdpLoopbackSnapshotPublisher,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> VisualReplayResult:
    """Launch RobotSimUE, present a recording, and commit replay evidence.

    The function deliberately never creates a PhysicsHost. MuJoCo timestamps in
    the recording remain authoritative; pacing changes only wall-clock display.
    """

    if not isinstance(config, VisualReplayConfig):
        raise TypeError("config must be a VisualReplayConfig")
    try:
        replay = SimulationReplay.open(config.recording_dir)
    except SimulationReplayError as exc:
        raise VisualReplayError(str(exc)) from exc
    bundle = load_resolved_session_bundle(config.bundle_dir, repo_root=config.repo_root)
    if replay.session_id != bundle.session_id:
        raise VisualReplayError(
            "recording session_id does not match the resolved SessionBundle"
        )
    plan = load_physics_plan(bundle.bundle_dir, config.repo_root)
    if replay.model_generation != plan.model_generation:
        raise VisualReplayError(
            "recording model_generation does not match the resolved SessionBundle"
        )
    replay_plan = replace(plan, reset_generation=replay.start_reset_generation)
    level = _resolved_level(bundle.plans["visual.plan.json"], config.map_name)
    run_id = config.run_id or f"visual-replay-{uuid.uuid4().hex[:12]}"
    allocation = create_run_allocation(
        bundle.bundle_dir,
        config.run_root,
        dds_domain=config.dds_domain,
        ports={"visual_snapshot_udp": config.snapshot_port},
        run_id=run_id,
        repo_root=config.repo_root,
    )
    capture_dir = allocation.log_dir / _CAPTURE_DIRECTORY
    capture_started_at_ns = _prepare_capture_directory(capture_dir)
    visual_evidence = allocation.log_dir / _VISUAL_EVIDENCE_FILENAME
    screenshot = allocation.log_dir / _FIRST_FRAME_FILENAME
    runtime_manifest = allocation.run_dir / "session.runtime.json"
    result_path = allocation.run_dir / VISUAL_REPLAY_RESULT_FILENAME
    unreal_log = allocation.log_dir / "Unreal.log"
    visual: _VisualProcess | None = None
    publisher: _SnapshotPublisher | None = None
    visual_pid: int | None = None
    report = None
    captured_frames: tuple[Path, ...] = ()
    terminal_failure: str | None = None

    _write_runtime_manifest(
        runtime_manifest,
        allocation=allocation,
        bundle_dir=bundle.bundle_dir,
        replay=replay,
        state="PREPARING",
        visual_state="PREPARING",
        visual_pid=None,
        failure_reason=None,
    )
    try:
        visual = unreal_factory(
            config.unreal_editor,
            config.uproject,
            level,
            frame_capture_dir=capture_dir,
            frame_capture_every=config.capture_every,
            frame_capture_max=config.maximum_frames,
            session_camera_tag=config.session_camera_tag,
            motion_camera_stable_id=config.motion_camera_stable_id,
        )
        publisher = publisher_factory(config.snapshot_port)
        visual.start(
            bundle_dir=bundle.bundle_dir,
            allocation=allocation,
            plan=replay_plan,
            snapshot_port=config.snapshot_port,
        )
        visual_pid = visual.pid
        first_event = _coordinator_event(replay.frames[0].snapshot)
        _wait_for_visual_active(
            visual,
            publisher,
            first_event,
            visual_evidence,
            replay,
            timeout_s=config.ready_timeout_s,
            monotonic=monotonic,
            sleep=sleep,
        )
        _wait_for_png(
            screenshot,
            visual=visual,
            publisher=publisher,
            keepalive=first_event,
            timeout_s=config.screenshot_timeout_s,
            monotonic=monotonic,
            sleep=sleep,
        )

        def publish(snapshot: dict[str, Any]) -> int:
            _fail_if_visual_exited(visual, "during replay")
            return publisher.publish(_coordinator_event(snapshot))

        report = replay_snapshots(
            replay,
            publish,
            pace=config.pace,
            rate=config.rate,
        )
        captured_frames = _wait_for_frames(
            capture_dir,
            minimum=config.minimum_frames,
            capture_started_at_ns=capture_started_at_ns,
            visual=visual,
            timeout_s=config.frame_timeout_s,
            monotonic=monotonic,
            sleep=sleep,
        )
    except BaseException as exc:
        terminal_failure = _error_summary(exc)
        cleanup_failures = _cleanup(publisher, visual)
        if cleanup_failures:
            terminal_failure += "; cleanup failed: " + "; ".join(cleanup_failures)
        _write_runtime_manifest(
            runtime_manifest,
            allocation=allocation,
            bundle_dir=bundle.bundle_dir,
            replay=replay,
            state="FAILED",
            visual_state="FAILED",
            visual_pid=visual_pid,
            failure_reason=terminal_failure,
        )
        _write_failure_result(
            result_path,
            allocation=allocation,
            replay=replay,
            failure_reason=terminal_failure,
        )
        if isinstance(exc, (KeyboardInterrupt, SystemExit)):
            raise
        if isinstance(exc, VisualReplayError):
            raise
        raise VisualReplayError(terminal_failure) from exc

    cleanup_failures = _cleanup(publisher, visual)
    if cleanup_failures:
        terminal_failure = "cleanup failed: " + "; ".join(cleanup_failures)
        _write_runtime_manifest(
            runtime_manifest,
            allocation=allocation,
            bundle_dir=bundle.bundle_dir,
            replay=replay,
            state="FAILED",
            visual_state="FAILED",
            visual_pid=visual_pid,
            failure_reason=terminal_failure,
        )
        _write_failure_result(
            result_path,
            allocation=allocation,
            replay=replay,
            failure_reason=terminal_failure,
        )
        raise VisualReplayError(terminal_failure)
    if report is None:
        raise VisualReplayError("visual replay finished without a replay report")
    result = VisualReplayResult(
        status="SUCCEEDED",
        run_id=allocation.run_id,
        session_id=replay.session_id,
        source_run_id=replay.run_id,
        replay_frames_presented=report.frames_presented,
        replay_frames_dropped=report.frames_dropped,
        captured_frames=len(captured_frames),
        first_frame_screenshot=screenshot.resolve(),
        capture_directory=capture_dir.resolve(),
        visual_evidence=visual_evidence.resolve(),
        unreal_log=unreal_log.resolve(),
        runtime_manifest=runtime_manifest.resolve(),
        result_path=result_path.resolve(),
    )
    _write_json(result_path, result.to_dict())
    _write_runtime_manifest(
        runtime_manifest,
        allocation=allocation,
        bundle_dir=bundle.bundle_dir,
        replay=replay,
        state="STOPPED",
        visual_state="ACTIVE",
        visual_pid=visual_pid,
        failure_reason=None,
    )
    return result


def _resolved_level(visual_plan: Mapping[str, Any], override: str | None) -> str:
    world = visual_plan.get("world")
    if not isinstance(world, Mapping):
        raise VisualReplayError("visual.plan.json world must be an object")
    value = override if override is not None else world.get("level")
    try:
        return _unreal_level(value, "visual.plan.world.level")
    except ValueError as exc:
        raise VisualReplayError(str(exc)) from exc


def _unreal_level(value: object, field: str) -> str:
    if not isinstance(value, str) or _UNREAL_LEVEL_RE.fullmatch(value) is None:
        raise ValueError(f"{field} must be a valid /Game/... level")
    return value


def _coordinator_event(snapshot: Mapping[str, Any]) -> dict[str, Any]:
    if snapshot.get("schema") != "lingtu.sim.truth-snapshot.v1":
        raise VisualReplayError("replay snapshot schema is invalid")
    return {
        "event": "snapshot",
        **{key: value for key, value in snapshot.items() if key != "schema"},
    }


def _wait_for_visual_active(
    visual: _VisualProcess,
    publisher: _SnapshotPublisher,
    first_event: Mapping[str, Any],
    evidence_path: Path,
    replay: SimulationReplay,
    *,
    timeout_s: float,
    monotonic: Callable[[], float],
    sleep: Callable[[float], None],
) -> None:
    deadline = monotonic() + timeout_s
    while True:
        _fail_if_visual_exited(visual, "before visual ACTIVE")
        publisher.publish(first_event)
        if evidence_path.is_file():
            evidence = ExternalRuntimeEvidence.from_path(evidence_path)
            _validate_visual_evidence(evidence, replay)
            if evidence.visual_state is EvidenceState.FAILED:
                raise VisualReplayError(
                    f"RobotSimUE visual binding failed: {evidence.visual_reason}"
                )
            if evidence.visual_state is EvidenceState.ACTIVE:
                return
        if monotonic() >= deadline:
            raise VisualReplayError("timed out waiting for RobotSimUE visual ACTIVE evidence")
        sleep(min(0.05, max(0.0, deadline - monotonic())))


def _validate_visual_evidence(
    evidence: ExternalRuntimeEvidence,
    replay: SimulationReplay,
) -> None:
    expected = {
        "session_id": (evidence.session_id, replay.session_id),
        "model_generation": (evidence.model_generation, replay.model_generation),
        "reset_generation": (
            evidence.reset_generation,
            replay.start_reset_generation,
        ),
        "source_id": (evidence.source_id, _VISUAL_SOURCE_ID),
        "basis": (evidence.basis, _VISUAL_ACTIVE_BASIS),
    }
    for field, (observed, required) in expected.items():
        if observed != required:
            raise VisualReplayError(f"RobotSimUE visual evidence {field} mismatch")


def _wait_for_png(
    path: Path,
    *,
    visual: _VisualProcess,
    publisher: _SnapshotPublisher,
    keepalive: Mapping[str, Any],
    timeout_s: float,
    monotonic: Callable[[], float],
    sleep: Callable[[float], None],
) -> None:
    deadline = monotonic() + timeout_s
    stable_size: int | None = None
    while True:
        _fail_if_visual_exited(visual, "before first-frame screenshot")
        publisher.publish(keepalive)
        size = _png_size(path)
        if size is not None:
            if stable_size == size:
                return
            stable_size = size
        else:
            stable_size = None
        if monotonic() >= deadline:
            raise VisualReplayError(
                f"RobotSimUE did not produce a stable PNG screenshot: {path}"
            )
        sleep(min(0.05, max(0.0, deadline - monotonic())))


def _wait_for_frames(
    directory: Path,
    *,
    minimum: int,
    capture_started_at_ns: int,
    visual: _VisualProcess,
    timeout_s: float,
    monotonic: Callable[[], float],
    sleep: Callable[[float], None],
) -> tuple[Path, ...]:
    deadline = monotonic() + timeout_s
    while True:
        _fail_if_visual_exited(visual, "before replay frame capture completed")
        frames = _valid_pngs(directory, capture_started_at_ns=capture_started_at_ns)
        if len(frames) >= minimum:
            return frames
        if monotonic() >= deadline:
            raise VisualReplayError(
                "RobotSimUE replay frame capture did not reach its minimum: "
                f"{len(frames)} < {minimum} in {directory}"
            )
        sleep(min(0.05, max(0.0, deadline - monotonic())))


def _prepare_capture_directory(path: Path) -> int:
    if path.exists():
        if not path.is_dir() or next(path.iterdir(), None) is not None:
            raise VisualReplayError(f"replay frame capture directory must be empty: {path}")
    else:
        path.mkdir()
    return time.time_ns()


def _valid_pngs(directory: Path, *, capture_started_at_ns: int) -> tuple[Path, ...]:
    frames: list[Path] = []
    for path in sorted(directory.glob("frame_*.png")):
        try:
            metadata = path.stat()
            if metadata.st_mtime_ns < capture_started_at_ns or _png_size(path) is None:
                continue
        except OSError:
            continue
        frames.append(path.resolve())
    return tuple(frames)


def _png_size(path: Path) -> int | None:
    try:
        size = path.stat().st_size
        if size <= len(_PNG_SIGNATURE):
            return None
        with path.open("rb") as handle:
            if handle.read(len(_PNG_SIGNATURE)) != _PNG_SIGNATURE:
                return None
        return size
    except OSError:
        return None


def _fail_if_visual_exited(visual: _VisualProcess, context: str) -> None:
    exit_code = visual.poll()
    if exit_code is not None:
        raise VisualReplayError(f"Unreal process exited {context} (exit={exit_code})")


def _cleanup(
    publisher: _SnapshotPublisher | None,
    visual: _VisualProcess | None,
) -> tuple[str, ...]:
    errors: list[str] = []
    if visual is not None:
        try:
            visual.terminate()
        except Exception as exc:
            errors.append(_error_summary(exc))
    if publisher is not None:
        try:
            publisher.close()
        except Exception as exc:
            errors.append(_error_summary(exc))
    return tuple(errors)


def _runtime_manifest(
    *,
    allocation: RunAllocation,
    bundle_dir: Path,
    replay: SimulationReplay,
    state: str,
    visual_state: str,
    visual_pid: int | None,
    failure_reason: str | None,
) -> dict[str, Any]:
    last = replay.frames[-1].snapshot
    return {
        "schema": "lingtu.sim.session-runtime.v1",
        "run_id": allocation.run_id,
        "session_id": replay.session_id,
        "model_generation": replay.model_generation,
        "reset_generation": replay.end_reset_generation,
        "state": state,
        "failure_reason": failure_reason,
        "bindings": {
            "physics": {"required": False, "state": "UNBOUND", "source_id": None},
            "visual": {
                "required": True,
                "state": visual_state,
                "source_id": _VISUAL_SOURCE_ID,
            },
            "sensors": {"required": False, "state": "UNBOUND", "source_id": None},
            "control": {"required": False, "state": "UNBOUND", "source_id": None},
        },
        "sensor_streams": None,
        "bundle_dir": str(bundle_dir),
        "allocation": {
            "run_dir": str(allocation.run_dir),
            "log_dir": str(allocation.log_dir),
            "boot_id": allocation.boot_id,
            "physics_pid": None,
            "visual_pid": visual_pid,
            "dds_domain": allocation.dds_domain,
            "ports": dict(allocation.ports),
            "shm": dict(allocation.shm),
            "shared_memory": dict(allocation.shared_memory),
        },
        "clock": {
            "authority": "recorded_mujoco",
            "sequence": last.get("sequence", 0),
            "physics_step": last.get("physics_step", 0),
            "sim_time_ns": last.get("sim_time_ns", 0),
        },
        "replay": {
            "source_run_id": replay.run_id,
            "source_recording": str(replay.root),
            "clock_authority": "recorded_mujoco",
            "frame_count": replay.frame_count,
            "physics_process_launched": False,
        },
    }


def _write_runtime_manifest(
    path: Path,
    **values: Any,
) -> None:
    _write_json(path, _runtime_manifest(**values))


def _write_failure_result(
    path: Path,
    *,
    allocation: RunAllocation,
    replay: SimulationReplay,
    failure_reason: str,
) -> None:
    _write_json(
        path,
        {
            "schema": VISUAL_REPLAY_RESULT_SCHEMA,
            "status": "FAILED",
            "run_id": allocation.run_id,
            "session_id": replay.session_id,
            "source_run_id": replay.run_id,
            "failure_reason": failure_reason,
            "physics_process_launched": False,
            "clock_authority": "recorded_mujoco",
        },
    )


def _write_json(path: Path, document: Mapping[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(
            document,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    replace_file_with_retry(temporary, path)


def _positive_finite(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{field} must be positive and finite")
    return result


def _error_summary(error: BaseException) -> str:
    message = str(error).strip()
    return f"{type(error).__name__}: {message}" if message else type(error).__name__


__all__ = [
    "VISUAL_REPLAY_RESULT_FILENAME",
    "VISUAL_REPLAY_RESULT_SCHEMA",
    "VisualReplayConfig",
    "VisualReplayError",
    "VisualReplayResult",
    "run_visual_replay",
]

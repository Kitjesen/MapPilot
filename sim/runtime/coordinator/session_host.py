"""Unified host for one prepared physics plus RobotSimUE session."""

# ruff: noqa: D102

from __future__ import annotations

import json
import math
import time
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any, Protocol

from sim.runtime.control.contracts import CommandSubmitResult, ControllerCommand

from .coordinator import CoordinatorError, RuntimeState
from .external_evidence import ExternalEvidenceTarget
from .live_snapshot import SnapshotPublisher
from .readiness import BindingFacet, BindingState

DEFAULT_REALTIME_SNAPSHOT_PERIOD_NS = 20_000_000
DEFAULT_REALTIME_EVIDENCE_PERIOD_S = 0.02


class SessionCoordinator(ExternalEvidenceTarget, Protocol):
    """Coordinator surface used by SessionHost."""

    @property
    def state(self) -> RuntimeState: ...

    @property
    def bundle_dir(self) -> Any: ...

    @property
    def allocation(self) -> Any: ...

    @property
    def plan(self) -> Any: ...

    @property
    def readiness(self) -> Any: ...

    @property
    def sensor_readiness(self) -> Any: ...

    @property
    def manifest_path(self) -> Path: ...

    def prepare(self) -> Mapping[str, Any]: ...

    def snapshot(self) -> Mapping[str, Any]: ...

    def warmup(self, steps: int = 1) -> Mapping[str, Any]: ...

    def start(self) -> Mapping[str, Any]: ...

    def advance(self, steps: int = 1) -> Mapping[str, Any]: ...

    def advance_realtime(self, steps: int = 1) -> Mapping[str, Any]: ...

    def pause(self) -> Mapping[str, Any]: ...

    def reset(self) -> Mapping[str, Any]: ...

    def submit_controller_command(
        self,
        controller_id: str,
        command: ControllerCommand,
    ) -> CommandSubmitResult: ...

    def hold_controller_commands(self) -> None: ...

    def stop(
        self,
        *,
        failure_reason: str | None = None,
    ) -> Mapping[str, Any]: ...

    def finalize_terminal_failure(
        self,
        failure_reason: str,
    ) -> Mapping[str, Any]: ...


class VisualProcess(Protocol):
    """Visual runtime process surface used by SessionHost."""

    def start(
        self,
        *,
        bundle_dir: Any,
        allocation: Any,
        plan: Any,
        snapshot_port: int,
    ) -> None: ...

    def poll(self) -> int | None: ...

    def terminate(self) -> None: ...


class EvidenceWatcher(Protocol):
    """External evidence watcher applied during prepare and run loops."""

    def apply(self, target: SessionCoordinator) -> bool: ...

    def advance_generation(
        self,
        *,
        model_generation: int,
        reset_generation: int,
    ) -> None: ...


class ClosableSnapshotPublisher(SnapshotPublisher, Protocol):
    """Publisher that releases its transport at the session boundary."""

    def close(self) -> None: ...


class SessionHost:
    """Own the unified visual session entry and fail-closed cleanup order."""

    def __init__(
        self,
        *,
        coordinator: SessionCoordinator,
        unreal_process: VisualProcess,
        publisher: ClosableSnapshotPublisher,
        evidence_watchers: Sequence[EvidenceWatcher] = (),
        snapshot_port: int | None = None,
        warmup_steps: int = 1,
        ready_timeout_s: float = 15.0,
        sleep_s: float = 0.01,
        realtime_snapshot_period_ns: int = DEFAULT_REALTIME_SNAPSHOT_PERIOD_NS,
        realtime_evidence_period_s: float = DEFAULT_REALTIME_EVIDENCE_PERIOD_S,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        if isinstance(warmup_steps, bool) or not isinstance(warmup_steps, int):
            raise ValueError("warmup_steps must be a positive integer")
        if warmup_steps <= 0:
            raise ValueError("warmup_steps must be a positive integer")
        if ready_timeout_s <= 0:
            raise ValueError("ready_timeout_s must be positive")
        if sleep_s < 0:
            raise ValueError("sleep_s must be non-negative")
        if (
            isinstance(realtime_snapshot_period_ns, bool)
            or not isinstance(realtime_snapshot_period_ns, int)
            or realtime_snapshot_period_ns <= 0
        ):
            raise ValueError("realtime_snapshot_period_ns must be a positive integer")
        if (
            isinstance(realtime_evidence_period_s, bool)
            or not isinstance(realtime_evidence_period_s, (int, float))
            or not math.isfinite(realtime_evidence_period_s)
            or realtime_evidence_period_s <= 0
        ):
            raise ValueError("realtime_evidence_period_s must be positive and finite")
        self._coordinator = coordinator
        self._unreal = unreal_process
        self._publisher = publisher
        self._evidence_watchers = tuple(evidence_watchers)
        self._snapshot_port = snapshot_port
        self._warmup_steps = warmup_steps
        self._ready_timeout_s = ready_timeout_s
        self._sleep_s = sleep_s
        self._realtime_snapshot_period_ns = realtime_snapshot_period_ns
        self._next_realtime_snapshot_sim_time_ns: int | None = None
        self._realtime_evidence_period_s = float(realtime_evidence_period_s)
        self._next_realtime_evidence_wall_time: float | None = None
        self._monotonic = monotonic
        self._sleep = sleep
        self._prepared = False
        self._closed = False
        self._runtime_stopped = False
        self._visual_exit_code: int | None = None
        self._terminal_event: Mapping[str, Any] | None = None

    @property
    def state(self) -> RuntimeState:
        """Expose coordinator state for InteractiveSimulationSession."""

        return self._coordinator.state

    def prepare(self) -> Mapping[str, Any]:
        """Prepare coordinator, launch Unreal, and warm up until READY."""

        if self._prepared:
            raise CoordinatorError("session host is already prepared")
        deadline = self._monotonic() + self._ready_timeout_s
        try:
            event = self._coordinator.prepare()
            snapshot_port = self._resolve_snapshot_port()
            self._unreal.start(
                bundle_dir=self._coordinator.bundle_dir,
                allocation=self._coordinator.allocation,
                plan=self._coordinator.plan,
                snapshot_port=snapshot_port,
            )
            while self._coordinator.state is not RuntimeState.READY:
                self._fail_if_unreal_exited("before session READY")
                self._publish_snapshot()
                self._apply_external_evidence()
                if self._coordinator.state is RuntimeState.READY:
                    break
                if self._monotonic() >= deadline:
                    raise CoordinatorError("timed out waiting for session READY; " + self._blocking_reasons())
                self._coordinator.warmup(self._warmup_steps)
                if self._sleep_s:
                    self._sleep(self._sleep_s)
            self._prepared = True
            return event
        except BaseException as exc:
            self._close_preserving(exc)
            raise

    def prepare_until_visual_applied(self) -> Mapping[str, Any]:
        """Prepare and wait until RobotSimUE proves visual ACTIVE."""

        if self._prepared:
            raise CoordinatorError("session host is already prepared")
        deadline = self._monotonic() + self._ready_timeout_s
        try:
            event = self._coordinator.prepare()
            snapshot_port = self._resolve_snapshot_port()
            self._unreal.start(
                bundle_dir=self._coordinator.bundle_dir,
                allocation=self._coordinator.allocation,
                plan=self._coordinator.plan,
                snapshot_port=snapshot_port,
            )
            while not self._visual_is_active():
                self._fail_if_unreal_exited("before visual ACTIVE")
                self._publish_snapshot()
                self._apply_external_evidence()
                if self._visual_is_active():
                    break
                if self._monotonic() >= deadline:
                    raise CoordinatorError("timed out waiting for visual ACTIVE; " + self._blocking_reasons())
                if self._sleep_s:
                    self._sleep(self._sleep_s)
            self._prepared = True
            return event
        except BaseException as exc:
            self._close_preserving(exc)
            raise

    def run(
        self,
        *,
        frame_limit: int,
        steps_per_frame: int = 1,
        close_on_finish: bool = True,
    ) -> int:
        """Run a bounded live session and publish each truth snapshot."""

        if isinstance(frame_limit, bool) or not isinstance(frame_limit, int):
            raise ValueError("frame_limit must be a positive integer")
        if frame_limit <= 0:
            raise ValueError("frame_limit must be a positive integer")
        if isinstance(steps_per_frame, bool) or not isinstance(steps_per_frame, int):
            raise ValueError("steps_per_frame must be a positive integer")
        if steps_per_frame <= 0:
            raise ValueError("steps_per_frame must be a positive integer")

        frames = 0
        try:
            if not self._prepared:
                self.prepare()
            self._coordinator.start()
            while frames < frame_limit:
                self._fail_if_unreal_exited("during session run")
                snapshot = self._coordinator.advance(steps_per_frame)
                self._publisher.publish(snapshot)
                self._apply_external_evidence()
                frames += 1
        except BaseException as exc:
            self._close_preserving(exc)
            raise
        if close_on_finish:
            self.close()
        return frames

    def run_visual_only(
        self,
        *,
        frame_limit: int,
        steps_per_frame: int = 1,
        close_on_finish: bool = True,
    ) -> int:
        """Run a bounded live visual stream without requiring full session READY."""

        if isinstance(frame_limit, bool) or not isinstance(frame_limit, int):
            raise ValueError("frame_limit must be a positive integer")
        if frame_limit <= 0:
            raise ValueError("frame_limit must be a positive integer")
        if isinstance(steps_per_frame, bool) or not isinstance(steps_per_frame, int):
            raise ValueError("steps_per_frame must be a positive integer")
        if steps_per_frame <= 0:
            raise ValueError("steps_per_frame must be a positive integer")

        frames = 0
        try:
            if not self._prepared:
                self.prepare_until_visual_applied()
            if self._coordinator.state is RuntimeState.READY:
                self._coordinator.start()
                while frames < frame_limit:
                    self._fail_if_unreal_exited("during visual-only session run")
                    snapshot = self._coordinator.advance(steps_per_frame)
                    self._publisher.publish(snapshot)
                    self._apply_external_evidence()
                    frames += 1
            else:
                while frames < frame_limit:
                    self._fail_if_unreal_exited("during visual-only session run")
                    snapshot = self._coordinator.warmup(steps_per_frame)
                    self._publisher.publish(snapshot)
                    self._apply_external_evidence()
                    frames += 1
                    if self._sleep_s:
                        self._sleep(self._sleep_s)
        except BaseException as exc:
            self._close_preserving(exc)
            raise
        if close_on_finish:
            self.close()
        return frames

    def start(self) -> Mapping[str, Any]:
        """Start a prepared full visual session."""

        return self._coordinator.start()

    def advance(self, steps: int = 1) -> Mapping[str, Any]:
        """Advance physics, publish the snapshot, and apply external evidence."""

        self._fail_if_unreal_exited("during session advance")
        snapshot = self._coordinator.advance(steps)
        self._publisher.publish(snapshot)
        self._apply_external_evidence()
        return snapshot

    def advance_realtime(self, steps: int = 1) -> Mapping[str, Any]:
        """Advance one interactive chunk and publish only its final truth frame."""

        self._fail_if_unreal_exited("during realtime session advance")
        snapshot = self._coordinator.advance_realtime(steps)
        sim_time_ns = snapshot.get("sim_time_ns")
        if isinstance(sim_time_ns, bool) or not isinstance(sim_time_ns, int) or sim_time_ns < 0:
            raise CoordinatorError("realtime snapshot has invalid sim_time_ns")
        next_publish = self._next_realtime_snapshot_sim_time_ns
        if next_publish is None or sim_time_ns >= next_publish:
            self._publisher.publish(snapshot)
            self._next_realtime_snapshot_sim_time_ns = (
                sim_time_ns + self._realtime_snapshot_period_ns
            )
        self._apply_realtime_external_evidence()
        return snapshot

    def pause(self) -> Mapping[str, Any]:
        """Pause the coordinator through the hosted protocol."""

        return self._coordinator.pause()

    def reset(self) -> Mapping[str, Any]:
        """Reset physics, publish reset snapshot, and accept new-generation evidence."""

        event = self._coordinator.reset()
        self._next_realtime_snapshot_sim_time_ns = None
        model_generation = _event_generation(event, "model_generation")
        reset_generation = _event_generation(event, "reset_generation")
        for watcher in self._evidence_watchers:
            watcher.advance_generation(
                model_generation=model_generation,
                reset_generation=reset_generation,
            )
        self._publisher.publish(event)
        self._apply_external_evidence()
        self._schedule_next_realtime_evidence_poll()
        return event

    def submit_controller_command(
        self,
        controller_id: str,
        command: ControllerCommand,
    ) -> CommandSubmitResult:
        """Submit a command through the hosted coordinator owner seam."""

        return self._coordinator.submit_controller_command(controller_id, command)

    def hold_controller_commands(self) -> None:
        """Quiesce hosted Controller Runtime state before lifecycle changes."""

        self._coordinator.hold_controller_commands()

    def control_status_authority_snapshot(
        self,
        snapshot: Mapping[str, Any],
        *,
        recording_snapshot: Mapping[str, Any] | None = None,
    ) -> Mapping[str, Any]:
        """Build one current-generation status projection on the owner thread.

        The mutable runtime manifest is committed atomically by the coordinator
        after advance/evidence application.  The production coordinator exposes
        that committed document in memory; the file path remains a compatibility
        fallback for injected hosts.  Socket threads never touch coordinator
        state, and only ``InteractiveSimulationSession`` assembles full status.
        """

        if not isinstance(snapshot, Mapping):
            raise CoordinatorError("control status snapshot must be an object")
        manifest_source = getattr(
            self._coordinator,
            "runtime_manifest_snapshot",
            None,
        )
        if callable(manifest_source):
            raw_manifest = manifest_source()
        else:
            manifest_path = Path(self._coordinator.manifest_path)
            try:
                raw_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            except (OSError, UnicodeError, json.JSONDecodeError) as exc:
                raise CoordinatorError(
                    "cannot read the current runtime manifest for control status"
                ) from exc
        if not isinstance(raw_manifest, dict):
            raise CoordinatorError("control status runtime manifest must be an object")

        allocation = self._coordinator.allocation
        from .control_intent_udp import ControlIntentValidationError
        from .control_status import (  # local import keeps the host seam narrow
            build_control_status_authority,
        )

        try:
            status = build_control_status_authority(
                identity={
                    "run_id": allocation.run_id,
                    "session_id": allocation.session_id,
                    "boot_id": allocation.boot_id,
                    "model_generation": snapshot.get("model_generation"),
                    "reset_generation": snapshot.get("reset_generation"),
                },
                runtime_state=self._coordinator.state.value,
                snapshot=snapshot,
                runtime_manifest=raw_manifest,
                recording_snapshot=recording_snapshot,
            )
        except ControlIntentValidationError as exc:
            raise CoordinatorError(f"control status authority is invalid: {exc}") from exc
        if not isinstance(status, Mapping):
            raise CoordinatorError("control status authority must be an object")
        return status

    def stop(
        self,
        *,
        failure_reason: str | None = None,
    ) -> Mapping[str, Any]:
        """Stop the hosted runtimes with SessionHost cleanup ordering."""

        if self._closed:
            return dict(self._terminal_event or {})
        errors = self._cleanup_errors(failure_reason=failure_reason)
        if errors:
            raise CoordinatorError(_cleanup_message(errors)) from errors[0]
        return dict(self._terminal_event or {})

    def stop_runtime_before_visual(
        self,
        *,
        failure_reason: str | None = None,
    ) -> Mapping[str, Any]:
        """Stop the simulation core while retaining UE and snapshot transport.

        This narrow phase is reserved for an Exit request whose terminal full
        status still has to reach UE.  Ordinary ``stop``/``close`` retain their
        existing fail-closed cleanup ordering.
        """

        if self._closed:
            return dict(self._terminal_event or {})
        if not self._prepared:
            raise CoordinatorError(
                "interactive visual shutdown requires a prepared session host"
            )
        exit_code = self._unreal.poll()
        if exit_code is not None:
            raise CoordinatorError(
                "Unreal exited before the simulation core terminal phase "
                f"(exit={exit_code})"
            )
        error = self._stop_coordinator_once(failure_reason=failure_reason)
        if error is not None:
            raise CoordinatorError(f"session runtime stop failed: {error}") from error
        exit_code = self._unreal.poll()
        if exit_code is not None:
            message = (
                "Unreal exited before terminal control status delivery "
                f"(exit={exit_code})"
            )
            finalizer_error = self._finalize_stopped_runtime_failure(message)
            try:
                self.close()
            except BaseException as cleanup_error:
                error = CoordinatorError(message)
                if finalizer_error is not None:
                    _add_exception_note(error, f"failure finalizer failed: {finalizer_error}")
                _add_exception_note(error, f"cleanup failed: {cleanup_error}")
                raise error from cleanup_error
            if finalizer_error is not None:
                error = CoordinatorError(message)
                _add_exception_note(error, f"failure finalizer failed: {finalizer_error}")
                raise error from finalizer_error
            raise CoordinatorError(message)
        return dict(self._terminal_event or {})

    def finalize_visual_after_terminal(self) -> int:
        """Wait for UE's normal Exit readback, then release visual resources."""

        if not self._runtime_stopped:
            raise CoordinatorError(
                "visual finalization requires the simulation runtime to be stopped"
            )
        if self._closed:
            if self._visual_exit_code is None:
                raise CoordinatorError("visual runtime closed without a natural exit readback")
            return self._visual_exit_code

        errors: list[Exception] = []
        exit_code: int | None = None
        deadline = self._monotonic() + self._ready_timeout_s
        try:
            while exit_code is None:
                exit_code = self._unreal.poll()
                if exit_code is not None:
                    break
                if self._monotonic() >= deadline:
                    errors.append(
                        CoordinatorError(
                            "timed out waiting for Unreal's terminal status Exit readback"
                        )
                    )
                    break
                self._sleep(self._sleep_s)
        except Exception as exc:
            errors.append(exc)

        self._visual_exit_code = exit_code
        try:
            self._unreal.terminate()
        except Exception as exc:
            errors.append(exc)
        try:
            self._publisher.close()
        except Exception as exc:
            errors.append(exc)
        if exit_code is not None and exit_code != 0:
            errors.append(
                CoordinatorError(
                    f"Unreal terminal status Exit returned non-zero code {exit_code}"
                )
            )
        if errors:
            self._finalize_stopped_runtime_failure(_cleanup_message(errors))
            self._closed = True
            raise CoordinatorError(_cleanup_message(errors)) from errors[0]
        if exit_code is None:
            message = "Unreal terminal status Exit has no process readback"
            self._finalize_stopped_runtime_failure(message)
            self._closed = True
            raise CoordinatorError("Unreal terminal status Exit has no process readback")
        self._closed = True
        return exit_code

    def close(self, *, failure_reason: str | None = None) -> None:
        """Quiesce runtimes and release resources in fail-closed order."""

        errors = self._cleanup_errors(failure_reason=failure_reason)
        if errors:
            raise CoordinatorError(_cleanup_message(errors)) from errors[0]

    def _close_preserving(self, original: BaseException) -> None:
        errors = self._cleanup_errors(original)
        for error in errors:
            _add_exception_note(original, f"cleanup failed: {error}")

    def _cleanup_errors(
        self,
        original: BaseException | None = None,
        failure_reason: str | None = None,
    ) -> list[Exception]:
        if self._closed:
            return []
        self._closed = True
        errors: list[Exception] = []
        if self._coordinator.state is RuntimeState.RUNNING:
            try:
                self._coordinator.pause()
            except Exception as exc:
                errors.append(exc)
        try:
            self._unreal.terminate()
        except Exception as exc:
            errors.append(exc)
        try:
            self._publisher.close()
        except Exception as exc:
            errors.append(exc)
        terminal_failure_reason = _terminal_failure_reason(
            original,
            errors,
            failure_reason=failure_reason,
        )
        if self._runtime_stopped:
            stop_error = self._finalize_stopped_runtime_failure(
                terminal_failure_reason
            )
        else:
            stop_error = self._stop_coordinator_once(
                failure_reason=terminal_failure_reason,
            )
        if stop_error is not None:
            errors.append(stop_error)
        return errors

    def _stop_coordinator_once(
        self,
        *,
        failure_reason: str | None,
    ) -> Exception | None:
        if self._runtime_stopped:
            return None
        try:
            if failure_reason is None:
                self._terminal_event = self._coordinator.stop()
            else:
                self._terminal_event = self._coordinator.stop(
                    failure_reason=failure_reason
                )
        except Exception as exc:
            return exc
        self._runtime_stopped = True
        return None

    def _finalize_stopped_runtime_failure(
        self,
        failure_reason: str | None,
    ) -> Exception | None:
        if failure_reason is None:
            return None
        finalizer = getattr(self._coordinator, "finalize_terminal_failure", None)
        if not callable(finalizer):
            return CoordinatorError(
                "stopped runtime cannot persist post-stop terminal failure"
            )
        try:
            self._terminal_event = finalizer(failure_reason)
        except Exception as exc:
            return exc
        return None

    def _resolve_snapshot_port(self) -> int:
        if self._snapshot_port is not None:
            return self._snapshot_port
        ports = getattr(self._coordinator.allocation, "ports", {})
        port = ports.get("visual_snapshot_udp") if isinstance(ports, Mapping) else None
        if isinstance(port, bool) or not isinstance(port, int):
            raise CoordinatorError("visual_snapshot_udp port is required")
        if port < 1 or port > 65535:
            raise CoordinatorError("visual_snapshot_udp port is outside [1, 65535]")
        self._snapshot_port = port
        return port

    def _publish_snapshot(self) -> None:
        snapshot = self._coordinator.snapshot()
        self._publisher.publish(snapshot)

    def _apply_external_evidence(self) -> None:
        for watcher in self._evidence_watchers:
            watcher.apply(self._coordinator)

    def _apply_realtime_external_evidence(self) -> None:
        if not self._evidence_watchers:
            return
        now = self._monotonic()
        deadline = self._next_realtime_evidence_wall_time
        if deadline is not None and now < deadline:
            return
        self._apply_external_evidence()
        self._next_realtime_evidence_wall_time = now + self._realtime_evidence_period_s

    def _schedule_next_realtime_evidence_poll(self) -> None:
        if not self._evidence_watchers:
            self._next_realtime_evidence_wall_time = None
            return
        self._next_realtime_evidence_wall_time = (
            self._monotonic() + self._realtime_evidence_period_s
        )

    def _fail_if_unreal_exited(self, context: str) -> None:
        exit_code = self._unreal.poll()
        if exit_code is not None:
            raise CoordinatorError(f"Unreal process exited {context} (exit={exit_code})")

    def _visual_is_active(self) -> bool:
        readiness = getattr(self._coordinator, "readiness", None)
        if readiness is None:
            return False
        try:
            return readiness.state(BindingFacet.VISUAL) is BindingState.ACTIVE
        except Exception:
            return False

    def _blocking_reasons(self) -> str:
        reasons: dict[str, str] = {}
        readiness = getattr(self._coordinator, "readiness", None)
        if readiness is not None:
            for facet, reason in getattr(readiness, "blocking_reasons", {}).items():
                key = getattr(facet, "value", str(facet))
                reasons[f"binding {key}"] = str(reason)
        try:
            sensor_readiness = self._coordinator.sensor_readiness
        except CoordinatorError:
            sensor_readiness = None
        if sensor_readiness is not None:
            for stream_id, reason in sensor_readiness.blocking_reasons.items():
                reasons[f"sensor {stream_id}"] = str(reason)
        if not reasons:
            return "blocking reasons unavailable"
        return "blocking reasons: " + "; ".join(f"{key}: {reason}" for key, reason in sorted(reasons.items()))


def _exception_summary(error: BaseException) -> str:
    message = str(error).strip()
    return f"{type(error).__name__}: {message}" if message else type(error).__name__


def _terminal_failure_reason(
    original: BaseException | None,
    cleanup_errors: Sequence[Exception],
    *,
    failure_reason: str | None = None,
) -> str | None:
    reasons: list[str] = []
    if failure_reason is not None:
        reasons.append(failure_reason)
    if original is not None:
        reasons.append(_exception_summary(original))
    reasons.extend(f"cleanup failed: {_exception_summary(error)}" for error in cleanup_errors)
    return "; ".join(reasons) if reasons else None


def _add_exception_note(error: BaseException, note: str) -> None:
    add_note = getattr(error, "add_note", None)
    if callable(add_note):
        add_note(note)
        return
    notes = getattr(error, "__notes__", None)
    if not isinstance(notes, list):
        notes = []
        error.__notes__ = notes  # type: ignore[attr-defined]
    notes.append(note)


def _cleanup_message(errors: Sequence[Exception]) -> str:
    return "session cleanup failed: " + "; ".join(str(error) for error in errors)


def _event_generation(event: Mapping[str, Any], field: str) -> int:
    value = event.get(field)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise CoordinatorError(f"coordinator event {field} must be a non-negative integer")
    return value

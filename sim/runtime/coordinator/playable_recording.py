"""Owner-thread recording lifecycle for the RobotSimUE playable session."""

from __future__ import annotations

import re
import threading
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Protocol

from sim.runtime.recording import (
    RECORDING_FILENAME,
    SensorPayloadSample,
    SimulationRecordingWriter,
)

RECORDING_STATUS_SCHEMA = "lingtu.sim.recording-status.v1"

_RUN_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z")


class PlayableRecordingError(RuntimeError):
    """Stable fail-closed recording lifecycle error."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


class RecordingLifecycleState(str, Enum):
    """Internal writer lifecycle; wire status is projected separately."""

    INACTIVE = "INACTIVE"
    CAPTURING = "CAPTURING"
    COMMITTED = "COMMITTED"
    FAILED = "FAILED"


@dataclass(frozen=True, slots=True)
class RecordingStatusSnapshot:
    """Immutable authoritative status for one interactive recording."""

    schema: str
    run_id: str
    session_id: str
    model_generation: int | None
    reset_generation: int | None
    lifecycle_state: RecordingLifecycleState
    state: str
    elapsed_sim_time_ns: int
    artifact_id: str
    blocker: str

    def to_document(self) -> dict[str, Any]:
        """Return the exact copy-safe v1 source document."""

        return {
            "schema": self.schema,
            "run_id": self.run_id,
            "session_id": self.session_id,
            "model_generation": self.model_generation,
            "reset_generation": self.reset_generation,
            "state": self.state,
            "elapsed_sim_time_ns": self.elapsed_sim_time_ns,
            "artifact_id": self.artifact_id,
            "blocker": self.blocker,
        }


class InteractiveRecordingSession(Protocol):
    """Narrow ordered-observer seam implemented by InteractiveSimulationSession."""

    @property
    def model_generation(self) -> int | None:
        """Return the currently admitted model generation."""

        ...

    @property
    def reset_generation(self) -> int | None:
        """Return the currently admitted reset generation."""

        ...

    def attach_event_observer(
        self,
        observer: Callable[[Mapping[str, Any]], object],
        *,
        replay_latest_snapshot: bool = False,
    ) -> int:
        """Attach an ordered observer and return its token."""

        ...

    def detach_event_observer(self, token: int) -> bool:
        """Detach an observer at an ordered event boundary."""

        ...

    def capture_sensor_payloads(
        self,
        snapshot: Mapping[str, Any],
    ) -> Sequence[SensorPayloadSample]:
        """Capture payloads aligned to the supplied snapshot."""

        ...


class RecordingWriter(Protocol):
    """Narrow writer seam used by the interactive lifecycle."""

    def append(
        self,
        snapshot_event: Mapping[str, Any],
        *,
        sensor_payloads: Sequence[SensorPayloadSample] | None = None,
    ) -> None: ...

    def close(self) -> Path: ...

    def abort(self) -> None: ...


class RecordingWriterFactory(Protocol):
    """Factory compatible with SimulationRecordingWriter."""

    def __call__(
        self,
        run_dir: Path,
        *,
        run_id: str,
        session_id: str,
        run_allocation: object | None = None,
        required_content: Sequence[str] = (),
    ) -> RecordingWriter: ...


class InteractiveRecordingController:
    """Attach, capture, and atomically commit recording on one owner thread."""

    def __init__(
        self,
        *,
        run_dir: Path,
        run_id: str,
        session_id: str,
        allocation_provider: Callable[[], object],
        writer_factory: RecordingWriterFactory = SimulationRecordingWriter,
    ) -> None:
        if not isinstance(run_id, str) or _RUN_ID_RE.fullmatch(run_id) is None:
            raise ValueError("run_id has an invalid value")
        if not isinstance(session_id, str) or not session_id.strip():
            raise ValueError("session_id must be non-empty")
        if not callable(allocation_provider):
            raise TypeError("allocation_provider must be callable")
        if not callable(writer_factory):
            raise TypeError("writer_factory must be callable")
        self._run_dir = Path(run_dir).resolve()
        self._run_id = run_id
        self._session_id = session_id
        self._allocation_provider = allocation_provider
        self._writer_factory = writer_factory
        self._lock = threading.RLock()
        self._session: InteractiveRecordingSession | None = None
        self._owner_thread_id: int | None = None
        self._writer: RecordingWriter | None = None
        self._observer_token: int | None = None
        self._lifecycle_state = RecordingLifecycleState.INACTIVE
        self._model_generation: int | None = None
        self._reset_generation: int | None = None
        self._previous_sequence: int | None = None
        self._previous_sim_time_ns: int | None = None
        self._elapsed_sim_time_ns = 0
        self._artifact_id = ""
        self._blocker = "recording controller is not bound"

    @property
    def is_capturing(self) -> bool:
        """Return whether exit must remain blocked pending explicit commit."""

        with self._lock:
            return self._lifecycle_state is RecordingLifecycleState.CAPTURING

    def bind_session(self, session: InteractiveRecordingSession) -> None:
        """Bind the one interactive session exactly once before owner-thread use."""

        for name in (
            "attach_event_observer",
            "detach_event_observer",
            "capture_sensor_payloads",
        ):
            if not callable(getattr(session, name, None)):
                raise TypeError(f"session must expose {name}()")
        with self._lock:
            if self._session is not None:
                raise PlayableRecordingError(
                    "recording_session_already_bound",
                    "recording controller session is already bound",
                )
            self._session = session
            self._blocker = ""

    def start(
        self,
        *,
        model_generation: int,
        reset_generation: int,
    ) -> RecordingStatusSnapshot:
        """Attach an ordered observer and replay the latest truth snapshot."""

        self._require_owner_thread()
        session = self._require_bound_session()
        requested_model = _generation(model_generation, "model_generation")
        requested_reset = _generation(reset_generation, "reset_generation")
        self._require_session_generation(
            session,
            model_generation=requested_model,
            reset_generation=requested_reset,
        )
        with self._lock:
            if self._lifecycle_state is not RecordingLifecycleState.INACTIVE:
                raise self._invalid_start_error()

        writer: RecordingWriter | None = None
        try:
            allocation = self._allocation_provider()
            _validate_allocation(
                allocation,
                run_dir=self._run_dir,
                run_id=self._run_id,
                session_id=self._session_id,
            )
            writer = self._writer_factory(
                self._run_dir,
                run_id=self._run_id,
                session_id=self._session_id,
                run_allocation=allocation,
                required_content=("truth_snapshot", "sensor_payload"),
            )
            with self._lock:
                self._writer = writer
                self._lifecycle_state = RecordingLifecycleState.CAPTURING
                self._model_generation = requested_model
                self._reset_generation = requested_reset
                self._previous_sequence = None
                self._previous_sim_time_ns = None
                self._elapsed_sim_time_ns = 0
                # This is the reserved target identity, not a claim that the
                # atomic manifest exists before the COMMITTED transition.
                self._artifact_id = RECORDING_FILENAME
                self._blocker = ""
            token = session.attach_event_observer(
                self._observe_event,
                replay_latest_snapshot=True,
            )
            if isinstance(token, bool) or not isinstance(token, int) or token < 1:
                raise PlayableRecordingError(
                    "recording_observer_token_invalid",
                    "recording observer returned an invalid token",
                )
            with self._lock:
                if self._lifecycle_state is not RecordingLifecycleState.CAPTURING:
                    raise PlayableRecordingError(
                        "recording_observer_attach_failed",
                        "recording failed while attaching its observer",
                    )
                self._observer_token = token
            return self.status_snapshot()
        except BaseException as exc:
            with self._lock:
                already_failed = (
                    self._lifecycle_state is RecordingLifecycleState.FAILED
                )
            if not already_failed:
                self._fail(exc, session=session, writer=writer)
            if isinstance(exc, PlayableRecordingError):
                raise
            raise PlayableRecordingError(
                "recording_start_failed",
                _error_summary(exc),
            ) from exc

    def commit(
        self,
        *,
        model_generation: int,
        reset_generation: int,
    ) -> RecordingStatusSnapshot:
        """Detach at an ordered boundary, then atomically close the writer."""

        self._require_owner_thread()
        session = self._require_bound_session()
        requested_model = _generation(model_generation, "model_generation")
        requested_reset = _generation(reset_generation, "reset_generation")
        self._require_session_generation(
            session,
            model_generation=requested_model,
            reset_generation=requested_reset,
        )
        with self._lock:
            if self._lifecycle_state is not RecordingLifecycleState.CAPTURING:
                raise self._invalid_commit_error()
            token = self._observer_token
            writer = self._writer
        if token is None or writer is None:
            error = PlayableRecordingError(
                "recording_state_corrupt",
                "capturing recording has no observer or writer",
            )
            self._fail(error, session=session, writer=writer)
            raise error
        try:
            if not session.detach_event_observer(token):
                raise PlayableRecordingError(
                    "recording_observer_detach_failed",
                    "recording observer was not attached at commit",
                )
            with self._lock:
                self._observer_token = None
            manifest_path = Path(writer.close()).resolve()
            if (
                manifest_path.parent != self._run_dir
                or manifest_path.name != RECORDING_FILENAME
            ):
                raise PlayableRecordingError(
                    "recording_manifest_path_invalid",
                    "recording writer committed outside its allocated manifest path",
                )
            with self._lock:
                self._writer = None
                self._lifecycle_state = RecordingLifecycleState.COMMITTED
                self._artifact_id = RECORDING_FILENAME
                self._blocker = ""
            return self.status_snapshot()
        except BaseException as exc:
            self._fail(exc, session=session, writer=writer)
            if isinstance(exc, PlayableRecordingError):
                raise
            raise PlayableRecordingError(
                "recording_commit_failed",
                _error_summary(exc),
            ) from exc

    def status_snapshot(self) -> RecordingStatusSnapshot:
        """Return immutable state without exposing the writer or session."""

        session = self._session
        session_model = None if session is None else session.model_generation
        session_reset = None if session is None else session.reset_generation
        with self._lock:
            model_generation = (
                session_model if session_model is not None else self._model_generation
            )
            reset_generation = (
                session_reset if session_reset is not None else self._reset_generation
            )
            state = _wire_state(self._lifecycle_state, bound=session is not None)
            return RecordingStatusSnapshot(
                schema=RECORDING_STATUS_SCHEMA,
                run_id=self._run_id,
                session_id=self._session_id,
                model_generation=model_generation,
                reset_generation=reset_generation,
                lifecycle_state=self._lifecycle_state,
                state=state,
                elapsed_sim_time_ns=self._elapsed_sim_time_ns,
                artifact_id=self._artifact_id,
                blocker=self._blocker,
            )

    def status_document(self) -> dict[str, Any]:
        """Return a copy-safe document consumed by full owner-thread status."""

        snapshot = self.status_snapshot()
        if snapshot.model_generation is None or snapshot.reset_generation is None:
            raise PlayableRecordingError(
                "recording_status_generation_unavailable",
                "recording status requires admitted model and reset generations",
            )
        return snapshot.to_document()

    def abort_failed(self, reason: str) -> RecordingStatusSnapshot:
        """Abort a still-open writer after the interactive owner is quiesced.

        This is the exceptional cleanup seam, not a user recording action.
        Cross-thread use is accepted only after the bound session proves its
        background owner has stopped.
        """

        if (
            not isinstance(reason, str)
            or not reason
            or reason != reason.strip()
            or len(reason) > 512
            or any(ord(character) < 32 for character in reason)
        ):
            raise ValueError("recording abort reason must be bounded trimmed text")
        session = self._require_bound_session()
        current_thread = threading.get_ident()
        with self._lock:
            capturing = self._lifecycle_state is RecordingLifecycleState.CAPTURING
            owner_thread = self._owner_thread_id
            writer = self._writer
        if not capturing:
            return self.status_snapshot()
        if owner_thread is not None and owner_thread != current_thread:
            if getattr(session, "background_thread_alive", None) is not False:
                raise PlayableRecordingError(
                    "recording_owner_not_quiesced",
                    "cross-thread recording abort requires a stopped interactive owner",
                )
        error = PlayableRecordingError("recording_aborted", reason)
        self._fail(error, session=session, writer=writer)
        return self.status_snapshot()

    def _observe_event(self, event: Mapping[str, Any]) -> None:
        self._require_owner_thread()
        if not isinstance(event, Mapping) or event.get("event") != "snapshot":
            return
        session = self._require_bound_session()
        with self._lock:
            writer = self._writer
            if (
                self._lifecycle_state is not RecordingLifecycleState.CAPTURING
                or writer is None
            ):
                raise PlayableRecordingError(
                    "recording_observer_inactive",
                    "recording observer received a snapshot while inactive",
                )
        try:
            sequence, sim_time_ns, reset_generation = self._validate_snapshot(event)
            payloads = tuple(session.capture_sensor_payloads(event))
            writer.append(event, sensor_payloads=payloads)
            with self._lock:
                previous_time = self._previous_sim_time_ns
                previous_reset = self._reset_generation
                if previous_time is not None and previous_reset == reset_generation:
                    self._elapsed_sim_time_ns += sim_time_ns - previous_time
                self._previous_sequence = sequence
                self._previous_sim_time_ns = sim_time_ns
                self._reset_generation = reset_generation
        except BaseException as exc:
            self._fail(exc, session=session, writer=writer)
            if isinstance(exc, PlayableRecordingError):
                raise
            raise PlayableRecordingError(
                "recording_append_failed",
                _error_summary(exc),
            ) from exc

    def _validate_snapshot(
        self,
        event: Mapping[str, Any],
    ) -> tuple[int, int, int]:
        if event.get("session_id") != self._session_id:
            raise PlayableRecordingError(
                "recording_snapshot_identity_mismatch",
                "recording snapshot session_id mismatch",
            )
        model_generation = _generation(event.get("model_generation"), "model_generation")
        reset_generation = _generation(event.get("reset_generation"), "reset_generation")
        sequence = _generation(event.get("sequence"), "sequence")
        sim_time_ns = _generation(event.get("sim_time_ns"), "sim_time_ns")
        with self._lock:
            if model_generation != self._model_generation:
                raise PlayableRecordingError(
                    "recording_snapshot_generation_mismatch",
                    "recording snapshot model_generation changed",
                )
            previous_reset = self._reset_generation
            previous_sequence = self._previous_sequence
            previous_time = self._previous_sim_time_ns
        if previous_reset is not None:
            if reset_generation == previous_reset:
                if previous_sequence is not None and sequence <= previous_sequence:
                    raise PlayableRecordingError(
                        "recording_snapshot_not_ordered",
                        "recording snapshot sequence is stale or duplicated",
                    )
                if previous_time is not None and sim_time_ns < previous_time:
                    raise PlayableRecordingError(
                        "recording_snapshot_not_ordered",
                        "recording snapshot simulation time moved backwards",
                    )
            elif reset_generation != previous_reset + 1:
                raise PlayableRecordingError(
                    "recording_snapshot_generation_mismatch",
                    "recording snapshot reset_generation is stale or skipped",
                )
        return sequence, sim_time_ns, reset_generation

    def _fail(
        self,
        error: BaseException,
        *,
        session: InteractiveRecordingSession,
        writer: RecordingWriter | None,
    ) -> None:
        with self._lock:
            token = self._observer_token
            self._observer_token = None
            self._writer = None
            self._lifecycle_state = RecordingLifecycleState.FAILED
            self._artifact_id = ""
            self._blocker = _error_summary(error)
        detach_error: BaseException | None = None
        if token is not None:
            try:
                if not session.detach_event_observer(token):
                    detach_error = RuntimeError("recording observer was already detached")
            except BaseException as exc:
                detach_error = exc
        abort_error: BaseException | None = None
        if writer is not None:
            try:
                writer.abort()
            except BaseException as exc:
                abort_error = exc
        if detach_error is not None or abort_error is not None:
            details = [self._blocker]
            if detach_error is not None:
                details.append(f"detach failed: {_error_summary(detach_error)}")
            if abort_error is not None:
                details.append(f"abort failed: {_error_summary(abort_error)}")
            with self._lock:
                self._blocker = "; ".join(details)

    def _require_owner_thread(self) -> None:
        thread_id = threading.get_ident()
        with self._lock:
            if self._owner_thread_id is None:
                self._owner_thread_id = thread_id
                return
            if self._owner_thread_id != thread_id:
                raise PlayableRecordingError(
                    "recording_owner_thread_mismatch",
                    "recording lifecycle may run only on its interactive owner thread",
                )

    def _require_bound_session(self) -> InteractiveRecordingSession:
        with self._lock:
            if self._session is None:
                raise PlayableRecordingError(
                    "recording_controller_unbound",
                    "recording controller is not bound to an interactive session",
                )
            return self._session

    @staticmethod
    def _require_session_generation(
        session: InteractiveRecordingSession,
        *,
        model_generation: int,
        reset_generation: int,
    ) -> None:
        if (
            session.model_generation != model_generation
            or session.reset_generation != reset_generation
        ):
            raise PlayableRecordingError(
                "recording_request_generation_mismatch",
                "recording request generation does not match the interactive session",
            )

    def _invalid_start_error(self) -> PlayableRecordingError:
        if self._lifecycle_state is RecordingLifecycleState.CAPTURING:
            return PlayableRecordingError(
                "recording_already_capturing",
                "recording is already capturing",
            )
        if self._lifecycle_state is RecordingLifecycleState.COMMITTED:
            return PlayableRecordingError(
                "recording_already_committed",
                "recording has already committed",
            )
        return PlayableRecordingError(
            "recording_failed",
            "recording is failed and cannot be restarted",
        )

    def _invalid_commit_error(self) -> PlayableRecordingError:
        if self._lifecycle_state is RecordingLifecycleState.INACTIVE:
            return PlayableRecordingError(
                "recording_not_capturing",
                "recording has not started",
            )
        if self._lifecycle_state is RecordingLifecycleState.COMMITTED:
            return PlayableRecordingError(
                "recording_already_committed",
                "recording has already committed",
            )
        return PlayableRecordingError(
            "recording_failed",
            "recording is failed and cannot commit",
        )


def _wire_state(state: RecordingLifecycleState, *, bound: bool) -> str:
    if not bound:
        return "unavailable"
    return {
        RecordingLifecycleState.INACTIVE: "idle",
        RecordingLifecycleState.CAPTURING: "recording",
        RecordingLifecycleState.COMMITTED: "committed",
        RecordingLifecycleState.FAILED: "failed",
    }[state]


def _generation(value: object, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise PlayableRecordingError(
            "recording_generation_invalid",
            f"{field} must be a non-negative integer",
        )
    return value


def _allocation_document(value: object) -> Mapping[str, Any]:
    candidate = value
    to_dict = getattr(value, "to_dict", None)
    if callable(to_dict):
        candidate = to_dict()
    if not isinstance(candidate, Mapping):
        raise PlayableRecordingError(
            "recording_allocation_invalid",
            "recording allocation provider must return an object",
        )
    return candidate


def _validate_allocation(
    value: object,
    *,
    run_dir: Path,
    run_id: str,
    session_id: str,
) -> None:
    allocation = _allocation_document(value)
    if (
        allocation.get("schema") != "lingtu.sim.run-allocation.v1"
        or allocation.get("run_id") != run_id
        or allocation.get("session_id") != session_id
    ):
        raise PlayableRecordingError(
            "recording_allocation_invalid",
            "recording allocation identity does not match the session",
        )
    artifact_root = allocation.get("artifact_root")
    if not isinstance(artifact_root, str) or Path(artifact_root).resolve() != run_dir:
        raise PlayableRecordingError(
            "recording_allocation_invalid",
            "recording allocation artifact_root must equal the owned run directory",
        )


def _error_summary(error: BaseException) -> str:
    message = str(error).strip()
    return f"{type(error).__name__}: {message}" if message else type(error).__name__


__all__ = [
    "RECORDING_STATUS_SCHEMA",
    "InteractiveRecordingController",
    "InteractiveRecordingSession",
    "PlayableRecordingError",
    "RecordingLifecycleState",
    "RecordingStatusSnapshot",
]

"""Owner-thread pump from validated RobotSimUE intent to Controller Runtime."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any, Protocol, TypeAlias

from sim.runtime.control.contracts import (
    CommandSubmitResult,
    ControllerCommand,
    GenerationStamp,
)

from .control_intent_udp import (
    CONTROL_ACK_SCHEMA,
    BoundedRuntimeRequestInbox,
    LatestOperatorIntentInbox,
    OperatorMotionIntent,
    OperatorRuntimeRequest,
)
from .control_status import (
    ControlStatusControlSnapshot,
    ControlStatusReporter,
)
from .controlled_run import (
    BaseTwist,
    BaseTwistTarget,
    resolve_base_twist_target,
)
from .interactive_session import InteractiveControlRequest
from .playable_recording import (
    InteractiveRecordingController,
    PlayableRecordingError,
)

MAX_TRANSLATION_MPS = 0.10
MAX_YAW_RADPS = 0.35
MAX_INTENT_AGE_NS = 100_000_000

ControlCorrelation: TypeAlias = OperatorMotionIntent | OperatorRuntimeRequest


class PlayableControlError(RuntimeError):
    """Raised when the playable control owner cannot fail closed."""


class ControllerCommandSubmitter(Protocol):
    """Narrow SessionHost command seam used only by the interactive owner."""

    def submit_controller_command(
        self,
        controller_id: str,
        command: ControllerCommand,
    ) -> CommandSubmitResult: ...

    def control_status_authority_snapshot(
        self,
        snapshot: Mapping[str, Any],
        *,
        recording_snapshot: Mapping[str, Any] | None = None,
    ) -> Mapping[str, Any]: ...


class ControlAckPublisher(Protocol):
    """Minimal ACK publisher boundary implemented by the UDP adapter."""

    def publish(self, document: Mapping[str, Any]) -> int: ...


class MotionAdmission(Protocol):
    """Optional authority boundary evaluated before controller submission."""

    def rejection_reason(self, intent: OperatorMotionIntent) -> str | None:
        """Return a stable reason to reject, or ``None`` to admit."""

        ...


class RuntimeRequestAdmission(Protocol):
    """Optional authority boundary evaluated before lifecycle handling."""

    def rejection_reason(self, request: OperatorRuntimeRequest) -> str | None:
        """Return a stable reason to reject, or ``None`` to handle."""

        ...


class PlayableControlPump:
    """Consume validated UE inboxes and submit commands on one owner thread."""

    def __init__(
        self,
        *,
        session_host: ControllerCommandSubmitter,
        target: BaseTwistTarget,
        motion_inbox: LatestOperatorIntentInbox,
        request_inbox: BoundedRuntimeRequestInbox,
        ack_publisher: ControlAckPublisher | None = None,
        trace_sink: Callable[[Mapping[str, Any]], object] | None = None,
        recording_controller: InteractiveRecordingController | None = None,
        motion_admission: MotionAdmission | None = None,
        runtime_request_admission: RuntimeRequestAdmission | None = None,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        if not isinstance(target, BaseTwistTarget):
            raise TypeError("target must be a BaseTwistTarget")
        self._session_host = session_host
        self._target = target
        self._motion_inbox = motion_inbox
        self._request_inbox = request_inbox
        self._ack_publisher = ack_publisher
        self._trace_sink = trace_sink
        self._recording_controller = recording_controller
        self._motion_admission = motion_admission
        self._runtime_request_admission = runtime_request_admission
        self._monotonic_ns = monotonic_ns
        self._owner_thread_id: int | None = None
        self._last_command_sequence = 0
        self._server_status_sequence = 0
        self._active_intent: OperatorMotionIntent | None = None
        self._pending_zero: tuple[ControlCorrelation | None, str, str] | None = None
        self._lifecycle_correlation: OperatorRuntimeRequest | None = None
        self._pending_terminal_exit: OperatorRuntimeRequest | None = None
        self._last_current_event: tuple[int, int, int, int] | None = None
        self._latest_status_control: ControlStatusControlSnapshot | None = None
        self._status_control_projection: tuple[object, ...] | None = None
        self._status_dirty = False
        self._control_owner = "unavailable"
        self._ui_mode = "unavailable"
        self._camera_mode = "unavailable"
        self._admitted_twist = (0.0, 0.0, 0.0)
        self._admitted_available = False
        self._status_reporter = (
            ControlStatusReporter(
                publisher=ack_publisher,
                monotonic_ns=monotonic_ns,
                published_sink=trace_sink,
            )
            if ack_publisher is not None
            else None
        )

    @classmethod
    def from_bundle(
        cls,
        *,
        session_host: ControllerCommandSubmitter,
        bundle_dir: Path,
        motion_inbox: LatestOperatorIntentInbox,
        request_inbox: BoundedRuntimeRequestInbox,
        controller_id: str | None = None,
        ack_publisher: ControlAckPublisher | None = None,
        trace_sink: Callable[[Mapping[str, Any]], object] | None = None,
        recording_controller: InteractiveRecordingController | None = None,
        motion_admission: MotionAdmission | None = None,
        runtime_request_admission: RuntimeRequestAdmission | None = None,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> PlayableControlPump:
        """Resolve the bundle-declared base-twist target exactly once."""

        return cls(
            session_host=session_host,
            target=resolve_base_twist_target(Path(bundle_dir), controller_id),
            motion_inbox=motion_inbox,
            request_inbox=request_inbox,
            ack_publisher=ack_publisher,
            trace_sink=trace_sink,
            recording_controller=recording_controller,
            motion_admission=motion_admission,
            runtime_request_admission=runtime_request_admission,
            monotonic_ns=monotonic_ns,
        )

    def process_runtime_requests(self) -> InteractiveControlRequest | None:
        """Consume at most one runtime request on the interactive owner thread."""

        self._require_owner_thread()
        request = self._request_inbox.pop_request()
        if request is None:
            return None
        self._emit_trace(_runtime_request_trace(request, event="runtime_request_received"))
        admission = self._runtime_request_admission
        if admission is not None:
            rejection_reason = admission.rejection_reason(request)
            if rejection_reason is not None:
                if (
                    not isinstance(rejection_reason, str)
                    or not rejection_reason
                    or rejection_reason != rejection_reason.strip()
                ):
                    raise PlayableControlError(
                        "runtime request admission rejection reason must be a non-empty trimmed string"
                    )
                self._complete_runtime_request(
                    request,
                    event="runtime_request_rejected",
                    status="rejected",
                    reason=rejection_reason,
                )
                return None
        if request.request in {"record_start", "record_stop_commit"}:
            self._process_recording_request(request)
            return None
        if request.request == "ui_state_update":
            self._update_ui_echo(request)
            self._complete_runtime_request(
                request,
                event="runtime_request_accepted",
                status="accepted",
                reason="",
            )
            return None
        lifecycle_request = {
            "pause": InteractiveControlRequest.PAUSE,
            "resume": InteractiveControlRequest.RESUME,
            "exit": InteractiveControlRequest.EXIT,
        }.get(request.request)
        if lifecycle_request is not None:
            if (
                lifecycle_request is InteractiveControlRequest.EXIT
                and self._recording_controller is not None
                and self._recording_controller.is_capturing
            ):
                self._complete_runtime_request(
                    request,
                    event="runtime_request_rejected",
                    status="rejected",
                    reason="recording_commit_required_before_exit",
                )
                return None
            self._update_ui_echo(request)
            self._lifecycle_correlation = request
            if lifecycle_request is InteractiveControlRequest.EXIT:
                self._pending_terminal_exit = request
            self._complete_runtime_request(
                request,
                event="runtime_request_accepted",
                status="accepted",
                reason="",
            )
            return lifecycle_request
        if request.request == "control_claim":
            self._update_ui_echo(request)
            self._complete_runtime_request(
                request,
                event="runtime_request_accepted",
                status="accepted",
                reason="",
            )
            return None
        if request.request in {"control_release", "safe_stop"}:
            self._update_ui_echo(request)
            reason = f"runtime_request:{request.request}"
            self._pending_zero = (request, "released", reason)
            self._motion_inbox.clear()
            self._active_intent = None
            self._complete_runtime_request(
                request,
                event="runtime_request_accepted",
                status="pending",
                reason=f"safe_zero_pending:{request.request}",
            )
            return None
        reason = f"unsupported_runtime_request:{request.request}"
        self._complete_runtime_request(
            request,
            event="runtime_request_rejected",
            status="rejected",
            reason=reason,
        )
        return None

    def process_before_advance(self, current_event: Mapping[str, Any]) -> None:
        """Submit the latest admitted motion command before physics advances."""

        self._require_owner_thread()
        model_generation = _event_integer(current_event, "model_generation")
        reset_generation = _event_integer(current_event, "reset_generation")
        event_sequence = _event_integer(current_event, "sequence")
        sim_time_ns = _event_integer(current_event, "sim_time_ns")
        self._last_current_event = (
            model_generation,
            reset_generation,
            event_sequence,
            sim_time_ns,
        )
        if self._flush_pending_zero(
            model_generation=model_generation,
            reset_generation=reset_generation,
            event_sequence=event_sequence,
            sim_time_ns=sim_time_ns,
        ):
            return
        intent = self._motion_inbox.take_latest()
        if intent is None:
            active_intent = self._active_intent
            if active_intent is not None and self._intent_age_ns(active_intent) > MAX_INTENT_AGE_NS:
                command, result = self._submit_twist(
                    BaseTwist(),
                    model_generation=model_generation,
                    reset_generation=reset_generation,
                    event_sequence=event_sequence,
                    sim_time_ns=sim_time_ns,
                )
                self._emit_trace(
                    _zero_trace(
                        active_intent,
                        command,
                        result,
                        reason="intent_stale",
                    )
                )
                self._publish_ack(
                    active_intent,
                    status=("timeout_zero" if result is CommandSubmitResult.ACCEPTED else "rejected"),
                    reason=(
                        "intent_stale" if result is CommandSubmitResult.ACCEPTED else f"intent_stale:{result.value}"
                    ),
                )
                self._active_intent = None
                self._raise_if_zero_rejected(result, "intent_stale")
            return
        self._emit_trace(_received_trace(intent))
        admission = self._motion_admission
        if admission is not None:
            rejection_reason = admission.rejection_reason(intent)
            if rejection_reason is not None:
                if (
                    not isinstance(rejection_reason, str)
                    or not rejection_reason
                    or rejection_reason != rejection_reason.strip()
                ):
                    raise PlayableControlError(
                        "motion admission rejection reason must be a non-empty trimmed string"
                    )
                active_intent = self._active_intent
                same_active_source = (
                    active_intent is not None
                    and active_intent.identity.source_id == intent.identity.source_id
                    and active_intent.source_epoch == intent.source_epoch
                )
                if same_active_source:
                    command, result = self._submit_twist(
                        BaseTwist(),
                        model_generation=model_generation,
                        reset_generation=reset_generation,
                        event_sequence=event_sequence,
                        sim_time_ns=sim_time_ns,
                    )
                    self._emit_trace(
                        _zero_trace(
                            intent,
                            command,
                            result,
                            reason=rejection_reason,
                        )
                    )
                    self._publish_ack(
                        intent,
                        status=(
                            "released"
                            if result is CommandSubmitResult.ACCEPTED
                            else "rejected"
                        ),
                        reason=(
                            rejection_reason
                            if result is CommandSubmitResult.ACCEPTED
                            else f"{rejection_reason}:{result.value}"
                        ),
                    )
                    self._active_intent = None
                    self._raise_if_zero_rejected(result, rejection_reason)
                else:
                    self._emit_trace(
                        _admission_rejected_trace(intent, reason=rejection_reason)
                    )
                    self._publish_ack(
                        intent,
                        status="rejected",
                        reason=rejection_reason,
                    )
                return
        identity = intent.identity
        zero_disposition: tuple[str, str] | None
        if identity.model_generation != model_generation or identity.reset_generation != reset_generation:
            zero_disposition = ("rejected", "intent_generation_mismatch")
        else:
            self._update_ui_echo(intent)
            zero_disposition = self._zero_disposition(intent)
        if zero_disposition is not None:
            status, reason = zero_disposition
            command, result = self._submit_twist(
                BaseTwist(),
                model_generation=model_generation,
                reset_generation=reset_generation,
                event_sequence=event_sequence,
                sim_time_ns=sim_time_ns,
            )
            self._emit_trace(_zero_trace(intent, command, result, reason=reason))
            self._publish_ack(
                intent,
                status=(status if result is CommandSubmitResult.ACCEPTED else "rejected"),
                reason=(reason if result is CommandSubmitResult.ACCEPTED else f"{reason}:{result.value}"),
            )
            self._active_intent = None
            self._raise_if_zero_rejected(result, reason)
            return
        twist = _scale_motion_intent(intent)
        command, result = self._submit_twist(
            twist,
            model_generation=model_generation,
            reset_generation=reset_generation,
            event_sequence=event_sequence,
            sim_time_ns=sim_time_ns,
        )
        if result is not CommandSubmitResult.ACCEPTED:
            self._active_intent = None
            self._emit_trace(_rejected_trace(intent, command, twist, result))
            self._publish_ack(intent, status="rejected", reason=result.value)
            raise PlayableControlError(f"controller rejected playable command: {result.value}")
        self._active_intent = intent
        self._emit_trace(
            _accepted_trace(
                intent,
                command,
                twist,
                result,
                controller_id=self._target.controller_id,
            )
        )
        self._publish_ack(intent, status="accepted", reason="")

    def clear(self, *, reason: str) -> None:
        """Clear motion and explicitly reject queued lifecycle work."""

        self.clear_motion(reason=reason)
        recording_controller = self._recording_controller
        if (
            reason == "stop"
            and recording_controller is not None
            and recording_controller.is_capturing
        ):
            recording_controller.abort_failed(
                "interactive session stopped before recording commit"
            )
        if self._owner_thread_id == threading.get_ident() and self._last_current_event is not None:
            (
                model_generation,
                reset_generation,
                event_sequence,
                sim_time_ns,
            ) = self._last_current_event
            self._flush_pending_zero(
                model_generation=model_generation,
                reset_generation=reset_generation,
                event_sequence=event_sequence,
                sim_time_ns=sim_time_ns,
            )
        while True:
            request = self._request_inbox.pop_request()
            if request is None:
                break
            rejection_reason = f"request_cleared:{reason}"
            self._complete_runtime_request(
                request,
                event="runtime_request_rejected",
                status="rejected",
                reason=rejection_reason,
            )

    def clear_motion(self, *, reason: str) -> None:
        """Clear only motion state while preserving the runtime-request FIFO."""

        if not isinstance(reason, str) or not reason or reason != reason.strip():
            raise ValueError("clear reason must be a non-empty trimmed string")
        existing_correlation = None if self._pending_zero is None else self._pending_zero[0]
        correlation = (
            self._lifecycle_correlation
            or self._active_intent
            or self._motion_inbox.peek_latest()
            or existing_correlation
        )
        self._lifecycle_correlation = None
        self._pending_zero = (correlation, "released", f"cleared:{reason}")
        self._motion_inbox.clear()
        self._active_intent = None

    def publish_status_after_advance(
        self,
        snapshot: Mapping[str, Any],
        *,
        runtime_state: str,
    ) -> int:
        """Publish full status from the post-advance owner-thread boundary.

        A full status cannot exist before one UE event has been admitted or
        explicitly rejected because UE validates its correlation against the
        successful-send hash.  The SessionHost supplies the runtime/truth/
        readiness/sensor authority; this pump supplies only Control admission.
        """

        self._require_owner_thread()
        control = self._latest_status_control
        reporter = self._status_reporter
        if control is None or reporter is None:
            return 0
        force = self._status_dirty
        if not force and not reporter.periodic_due():
            return 0
        authority_source = getattr(
            self._session_host,
            "control_status_authority_snapshot",
            None,
        )
        if not callable(authority_source):
            raise PlayableControlError(
                "session host does not expose control status authority"
            )
        recording_controller = self._recording_controller
        authority = dict(
            authority_source(snapshot)
            if recording_controller is None
            else authority_source(
                snapshot,
                recording_snapshot=recording_controller.status_document(),
            )
        )
        if authority.get("runtime_state") != runtime_state:
            raise PlayableControlError(
                "control status runtime_state disagrees with the interactive owner"
            )
        published = reporter.publish_after_advance(control, authority, force=force)
        if isinstance(published, bool) or not isinstance(published, int) or published < 0:
            raise PlayableControlError(
                "control status reporter returned an invalid byte count"
            )
        if published > 0:
            self._status_dirty = False
        return published

    def publish_terminal_status_after_stop(
        self,
        snapshot: Mapping[str, Any],
        *,
        runtime_state: str,
    ) -> int:
        """Force the exact Exit-correlated STOPPED confirmation onto the wire."""

        self._require_owner_thread()
        if runtime_state != "STOPPED":
            raise PlayableControlError(
                "terminal control status requires runtime_state STOPPED"
            )
        request = self._pending_terminal_exit
        reporter = self._status_reporter
        if request is None:
            raise PlayableControlError(
                "terminal control status requires an accepted Exit correlation"
            )
        if reporter is None:
            raise PlayableControlError(
                "terminal control status publisher is unavailable"
            )
        if not self._admitted_available or self._admitted_twist != (0.0, 0.0, 0.0):
            raise PlayableControlError(
                "terminal control status requires an admitted safety zero"
            )
        self._control_owner = "unavailable"
        self._latest_status_control = ControlStatusControlSnapshot(
            identity=request.identity,
            source_epoch=request.source_epoch,
            source_sequence=request.source_sequence,
            event_id=request.event_id,
            intent_datagram_sha256=request.datagram_sha256,
            status="confirmed",
            reason="",
            control_owner="unavailable",
            deadman=False,
            sample_age_ns=self._correlation_age_ns(request),
            safe_stop_state="zeroed",
            requested_axes=(0.0, 0.0, 0.0),
            requested_available=False,
            admitted_twist=(0.0, 0.0, 0.0),
            admitted_available=True,
            ui_mode=self._ui_mode,
            camera_mode=self._camera_mode,
        )
        authority_source = getattr(
            self._session_host,
            "control_status_authority_snapshot",
            None,
        )
        if not callable(authority_source):
            raise PlayableControlError(
                "session host does not expose control status authority"
            )
        recording_controller = self._recording_controller
        authority = dict(
            authority_source(snapshot)
            if recording_controller is None
            else authority_source(
                snapshot,
                recording_snapshot=recording_controller.status_document(),
            )
        )
        if authority.get("runtime_state") != runtime_state:
            raise PlayableControlError(
                "terminal control status runtime_state disagrees with STOPPED"
            )
        published = reporter.publish_after_advance(
            self._latest_status_control,
            authority,
            force=True,
        )
        if isinstance(published, bool) or not isinstance(published, int) or published <= 0:
            raise PlayableControlError(
                "terminal control status was not completely published"
            )
        self._pending_terminal_exit = None
        return published

    def _flush_pending_zero(
        self,
        *,
        model_generation: int,
        reset_generation: int,
        event_sequence: int,
        sim_time_ns: int,
    ) -> bool:
        pending_zero = self._pending_zero
        if pending_zero is None:
            return False
        correlation, status, reason = pending_zero
        self._pending_zero = None
        command, result = self._submit_twist(
            BaseTwist(),
            model_generation=model_generation,
            reset_generation=reset_generation,
            event_sequence=event_sequence,
            sim_time_ns=sim_time_ns,
        )
        if correlation is None:
            trace = _uncorrelated_zero_trace(command, result, reason=reason)
        else:
            trace = _zero_trace(correlation, command, result, reason=reason)
        self._emit_trace(trace)
        if correlation is not None:
            self._publish_ack(
                correlation,
                status=(status if result is CommandSubmitResult.ACCEPTED else "rejected"),
                reason=(reason if result is CommandSubmitResult.ACCEPTED else f"{reason}:{result.value}"),
            )
        self._active_intent = None
        self._raise_if_zero_rejected(result, reason)
        return True

    def _submit_twist(
        self,
        twist: BaseTwist,
        *,
        model_generation: int,
        reset_generation: int,
        event_sequence: int,
        sim_time_ns: int,
    ) -> tuple[ControllerCommand, CommandSubmitResult]:
        command_sequence = max(self._last_command_sequence + 1, event_sequence + 1)
        command = ControllerCommand(
            channel_id=self._target.channel_id,
            instance_id=self._target.instance_id,
            generation=GenerationStamp(
                model_generation=model_generation,
                reset_generation=reset_generation,
            ),
            sequence=command_sequence,
            apply_time_ns=sim_time_ns,
            payload=twist.payload(),
        )
        result = self._session_host.submit_controller_command(
            self._target.controller_id,
            command,
        )
        self._last_command_sequence = command_sequence
        return command, result

    def _intent_age_ns(self, intent: OperatorMotionIntent) -> int:
        now_ns = self._monotonic_ns()
        if isinstance(now_ns, bool) or not isinstance(now_ns, int) or now_ns < 0:
            raise PlayableControlError("monotonic clock must return a non-negative integer")
        arrival_monotonic_ns = intent.arrival_monotonic_ns
        if (
            isinstance(arrival_monotonic_ns, bool)
            or not isinstance(arrival_monotonic_ns, int)
            or arrival_monotonic_ns < 0
        ):
            raise PlayableControlError("intent arrival_monotonic_ns must be a non-negative integer")
        if now_ns < arrival_monotonic_ns:
            raise PlayableControlError("monotonic clock moved before intent arrival")
        return now_ns - arrival_monotonic_ns

    def _zero_disposition(
        self,
        intent: OperatorMotionIntent,
    ) -> tuple[str, str] | None:
        if self._intent_age_ns(intent) > MAX_INTENT_AGE_NS:
            return "timeout_zero", "intent_stale"
        if intent.input_mode != "drive":
            return "released", "input_mode_not_drive"
        if not intent.viewport_focused:
            return "released", "viewport_unfocused"
        if not intent.deadman:
            return "released", "deadman_released"
        return None

    @staticmethod
    def _raise_if_zero_rejected(
        result: CommandSubmitResult,
        reason: str,
    ) -> None:
        if result is not CommandSubmitResult.ACCEPTED:
            raise PlayableControlError(f"controller rejected safety zero ({reason}): {result.value}")

    def _require_owner_thread(self) -> None:
        thread_id = threading.get_ident()
        if self._owner_thread_id is None:
            self._owner_thread_id = thread_id
            return
        if thread_id != self._owner_thread_id:
            raise PlayableControlError("playable control pump may run only on its interactive owner thread")

    def _emit_trace(self, record: Mapping[str, Any]) -> None:
        if self._trace_sink is not None:
            self._trace_sink(record)

    def _process_recording_request(
        self,
        request: OperatorRuntimeRequest,
    ) -> None:
        controller = self._recording_controller
        if controller is None:
            self._complete_runtime_request(
                request,
                event="runtime_request_rejected",
                status="rejected",
                reason="recording_controller_unavailable",
            )
            return
        identity = request.identity
        try:
            if request.request == "record_start":
                controller.start(
                    model_generation=identity.model_generation,
                    reset_generation=identity.reset_generation,
                )
            else:
                controller.commit(
                    model_generation=identity.model_generation,
                    reset_generation=identity.reset_generation,
                )
        except PlayableRecordingError as exc:
            self._complete_runtime_request(
                request,
                event="runtime_request_rejected",
                status="rejected",
                reason=exc.code,
            )
            return
        self._update_ui_echo(request)
        self._complete_runtime_request(
            request,
            event="runtime_request_accepted",
            status="accepted",
            reason="",
        )

    def _update_ui_echo(self, intent: ControlCorrelation) -> None:
        ui_mode = getattr(intent, "ui_mode", "unavailable")
        camera_mode = getattr(intent, "camera_mode", "unavailable")
        self._ui_mode = ui_mode
        self._camera_mode = camera_mode

    def _complete_runtime_request(
        self,
        request: OperatorRuntimeRequest,
        *,
        event: str,
        status: str,
        reason: str,
    ) -> None:
        self._emit_trace(
            _runtime_request_trace(
                request,
                event=event,
                status=status,
                reason=reason,
            )
        )
        self._publish_ack(request, status=status, reason=reason)

    def _publish_ack(
        self,
        intent: ControlCorrelation,
        *,
        status: str,
        reason: str,
    ) -> None:
        publisher = self._ack_publisher
        identity = intent.identity
        latest_status = self._control_status_snapshot(
            intent,
            status=status,
            reason=reason,
        )
        projection = _control_status_projection(latest_status)
        if projection != self._status_control_projection:
            self._status_control_projection = projection
            self._status_dirty = True
        self._latest_status_control = latest_status
        if publisher is not None:
            self._server_status_sequence += 1
            publisher.publish(
                {
                    "schema": CONTROL_ACK_SCHEMA,
                    "run_id": identity.run_id,
                    "session_id": identity.session_id,
                    "boot_id": identity.boot_id,
                    "model_generation": identity.model_generation,
                    "reset_generation": identity.reset_generation,
                    "server_status_sequence": self._server_status_sequence,
                    "source_id": identity.source_id,
                    "source_epoch": intent.source_epoch,
                    "source_sequence": intent.source_sequence,
                    "event_id": intent.event_id,
                    "intent_datagram_sha256": intent.datagram_sha256,
                    "status": status,
                    "reason": reason,
                }
            )

    def _control_status_snapshot(
        self,
        intent: ControlCorrelation,
        *,
        status: str,
        reason: str,
    ) -> ControlStatusControlSnapshot:
        if isinstance(intent, OperatorMotionIntent):
            is_motion = True
            requested = (
                intent.axes.forward,
                intent.axes.left,
                intent.axes.yaw_left,
            )
            deadman = intent.deadman
            if status == "accepted":
                scaled = _scale_motion_intent(intent)
                self._admitted_twist = (
                    scaled.linear_x,
                    scaled.linear_y,
                    scaled.angular_z,
                )
                self._admitted_available = True
        else:
            is_motion = False
            requested = (0.0, 0.0, 0.0)
            deadman = False
        if status == "accepted" and (
            is_motion
            or (
                isinstance(intent, OperatorRuntimeRequest)
                and intent.request == "control_claim"
            )
        ):
            self._control_owner = intent.identity.source_id
        elif status in {"released", "timeout_zero"}:
            self._control_owner = "unavailable"

        if status in {"released", "timeout_zero"}:
            self._admitted_twist = (0.0, 0.0, 0.0)
            self._admitted_available = True

        safe_stop_state = {
            "pending": "pending",
            "released": "zeroed",
            "timeout_zero": "zeroed",
            "rejected": "blocked",
        }.get(status, "clear")
        age_ns = self._correlation_age_ns(intent)
        return ControlStatusControlSnapshot(
            identity=intent.identity,
            source_epoch=intent.source_epoch,
            source_sequence=intent.source_sequence,
            event_id=intent.event_id,
            intent_datagram_sha256=intent.datagram_sha256,
            status=status,
            reason=reason,
            control_owner=self._control_owner,
            deadman=deadman,
            sample_age_ns=age_ns,
            safe_stop_state=safe_stop_state,
            requested_axes=requested,
            requested_available=is_motion,
            admitted_twist=self._admitted_twist,
            admitted_available=self._admitted_available,
            ui_mode=self._ui_mode,
            camera_mode=self._camera_mode,
        )

    def _correlation_age_ns(self, intent: ControlCorrelation) -> int:
        now_ns = self._monotonic_ns()
        if isinstance(now_ns, bool) or not isinstance(now_ns, int) or now_ns < 0:
            raise PlayableControlError(
                "monotonic clock must return a non-negative integer"
            )
        arrival_monotonic_ns = intent.arrival_monotonic_ns
        if (
            isinstance(arrival_monotonic_ns, bool)
            or not isinstance(arrival_monotonic_ns, int)
            or arrival_monotonic_ns < 0
        ):
            raise PlayableControlError(
                "intent arrival_monotonic_ns must be a non-negative integer"
            )
        if now_ns < arrival_monotonic_ns:
            raise PlayableControlError("monotonic clock moved before intent arrival")
        return now_ns - arrival_monotonic_ns


def _event_integer(event: Mapping[str, Any], field: str) -> int:
    value = event.get(field)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise PlayableControlError(f"current runtime event {field} must be a non-negative integer")
    return value


def _control_status_projection(
    status: ControlStatusControlSnapshot,
) -> tuple[object, ...]:
    return (
        status.status,
        status.reason,
        status.control_owner,
        status.deadman,
        status.safe_stop_state,
        status.requested_axes,
        status.requested_available,
        status.admitted_twist,
        status.admitted_available,
        status.ui_mode,
        status.camera_mode,
    )


def _scale_motion_intent(intent: OperatorMotionIntent) -> BaseTwist:
    forward = _finite_axis(intent.axes.forward, "forward")
    left = _finite_axis(intent.axes.left, "left")
    yaw_left = _finite_axis(intent.axes.yaw_left, "yaw_left")
    magnitude = math.hypot(forward, left)
    if magnitude > 1.0:
        forward /= magnitude
        left /= magnitude
    yaw_left = max(-1.0, min(1.0, yaw_left))
    return BaseTwist(
        linear_x=forward * MAX_TRANSLATION_MPS,
        linear_y=left * MAX_TRANSLATION_MPS,
        angular_z=yaw_left * MAX_YAW_RADPS,
    )


def _finite_axis(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PlayableControlError(f"operator axis {field} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise PlayableControlError(f"operator axis {field} must be finite")
    return result


def _intent_identity_trace(intent: ControlCorrelation) -> dict[str, Any]:
    identity = intent.identity
    return {
        "run_id": identity.run_id,
        "session_id": identity.session_id,
        "boot_id": identity.boot_id,
        "model_generation": identity.model_generation,
        "reset_generation": identity.reset_generation,
        "source_id": identity.source_id,
        "source_epoch": intent.source_epoch,
        "source_sequence": intent.source_sequence,
        "event_id": intent.event_id,
        "source_monotonic_ns": intent.source_monotonic_ns,
        "arrival_monotonic_ns": intent.arrival_monotonic_ns,
        "datagram_sha256": intent.datagram_sha256,
    }


def _runtime_request_trace(
    request: OperatorRuntimeRequest,
    *,
    event: str,
    status: str | None = None,
    reason: str | None = None,
) -> dict[str, Any]:
    identity = request.identity
    record: dict[str, Any] = {
        "schema": "lingtu.sim.runtime-request-trace.v1",
        "event": event,
        "run_id": identity.run_id,
        "session_id": identity.session_id,
        "boot_id": identity.boot_id,
        "model_generation": identity.model_generation,
        "reset_generation": identity.reset_generation,
        "source_id": identity.source_id,
        "source_epoch": request.source_epoch,
        "source_sequence": request.source_sequence,
        "event_id": request.event_id,
        "request": request.request,
        "source_monotonic_ns": request.source_monotonic_ns,
        "arrival_monotonic_ns": request.arrival_monotonic_ns,
        "datagram_sha256": request.datagram_sha256,
    }
    if status is not None:
        record["status"] = status
    if reason is not None:
        record["reason"] = reason
    return record


def _received_trace(intent: OperatorMotionIntent) -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.control-intent-received.v1",
        "event": "control_intent_received",
        **_intent_identity_trace(intent),
    }


def _accepted_trace(
    intent: OperatorMotionIntent,
    command: ControllerCommand,
    twist: BaseTwist,
    result: CommandSubmitResult,
    *,
    controller_id: str,
) -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.control-intent-accepted.v1",
        "event": "control_command_accepted",
        **_intent_identity_trace(intent),
        "controller_id": controller_id,
        "channel_id": command.channel_id,
        "controller_sequence": command.sequence,
        "apply_time_ns": command.apply_time_ns,
        "submit_result": result.value,
        "admitted_twist": twist.payload(),
    }


def _rejected_trace(
    intent: OperatorMotionIntent,
    command: ControllerCommand,
    twist: BaseTwist,
    result: CommandSubmitResult,
) -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.control-intent-rejected.v1",
        "event": "control_command_rejected",
        **_intent_identity_trace(intent),
        "controller_sequence": command.sequence,
        "apply_time_ns": command.apply_time_ns,
        "submit_result": result.value,
        "admitted_twist": twist.payload(),
        "reason": result.value,
    }


def _admission_rejected_trace(
    intent: OperatorMotionIntent,
    *,
    reason: str,
) -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.control-intent-rejected.v1",
        "event": "control_intent_rejected",
        **_intent_identity_trace(intent),
        "reason": reason,
    }


def _zero_trace(
    intent: ControlCorrelation,
    command: ControllerCommand,
    result: CommandSubmitResult,
    *,
    reason: str,
) -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.control-command-zero.v1",
        "event": "control_command_zero",
        **_intent_identity_trace(intent),
        "controller_sequence": command.sequence,
        "apply_time_ns": command.apply_time_ns,
        "submit_result": result.value,
        "admitted_twist": dict(command.payload),
        "reason": reason,
    }


def _uncorrelated_zero_trace(
    command: ControllerCommand,
    result: CommandSubmitResult,
    *,
    reason: str,
) -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.control-command-zero.v1",
        "event": "control_command_zero",
        "datagram_sha256": None,
        "controller_sequence": command.sequence,
        "apply_time_ns": command.apply_time_ns,
        "submit_result": result.value,
        "admitted_twist": dict(command.payload),
        "reason": reason,
    }


__all__ = [
    "MAX_INTENT_AGE_NS",
    "MAX_TRANSLATION_MPS",
    "MAX_YAW_RADPS",
    "MotionAdmission",
    "PlayableControlError",
    "PlayableControlPump",
    "RuntimeRequestAdmission",
]

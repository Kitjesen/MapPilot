"""Connection-scoped SDK teleop through the existing native operator session."""

import logging
import math
import queue
import threading
import time
from collections import deque
from uuid import uuid4

import grpc

from gateway.services.safety_status import safety_stop_active
from gateway.services.teleop import NativeTeleopSession

from . import control
from .codec import _abort, command
from .generated import motion_pb2 as pb

logger = logging.getLogger(__name__)
INPUT_WINDOW_S = 0.35
WINDOW_INTERVAL_S = 0.05
CONTROL_INTERVAL_S = 0.02
IDLE_TIMEOUT_S = 3.0


def _velocity(request):
    return request is not None and request.WhichOneof("action") == "velocity" and request.velocity.deadman


class _Pending:
    """One pending input, replacing velocities while preserving control order."""

    def __init__(self, stopped):
        self._stopped = stopped
        self._condition = threading.Condition()
        self._items = deque()

    def put(self, request):
        with self._condition:
            while self._items and not self._stopped.is_set():
                previous = self._items[0]
                if (
                    _velocity(request)
                    and _velocity(previous)
                    and request.sequence > previous.sequence
                    and request.command.client_id == previous.command.client_id
                ):
                    self._items[0] = request
                    return
                self._condition.wait(0.1)
            if not self._stopped.is_set():
                self._items.append(request)
                self._condition.notify_all()

    def get(self, timeout):
        with self._condition:
            if not self._condition.wait_for(lambda: bool(self._items), timeout):
                raise queue.Empty
            request = self._items.popleft()
            self._condition.notify_all()
            return request


def _requests(iterator, pending, stopped):
    try:
        for request in iterator:
            pending.put(request)
            if stopped.is_set():
                return
    except grpc.RpcError:
        pass
    finally:
        pending.put(None)


def teleop(gateway, requests, context, slots):
    if not slots.acquire(blocking=False):
        context.abort(grpc.StatusCode.RESOURCE_EXHAUSTED, "SDK stream limit reached")
    stopped = threading.Event()
    pending = _Pending(stopped)
    source_id = f"sdk:{uuid4()}"
    session = NativeTeleopSession(gateway, source_id)
    opened = False
    window = 0
    windows = deque(maxlen=8)
    next_window = 0.0
    next_control = 0.0
    last_sequence = 0
    client_id = None

    def grant(reply):
        nonlocal window, next_window
        window += 1
        now = time.monotonic()
        windows.append((window, now + INPUT_WINDOW_S))
        next_window = now + WINDOW_INTERVAL_S
        reply.input_window = window
        return reply

    def feedback(reply):
        reply.control.CopyFrom(control.state(gateway, session.identity))
        return reply

    try:
        if not context.add_callback(stopped.set):
            return
        result = session.open()
        if not result.accepted:
            context.abort(grpc.StatusCode.RESOURCE_EXHAUSTED, result.reason)
        opened = True
        threading.Thread(target=_requests, args=(requests, pending, stopped), daemon=True).start()
        context.send_initial_metadata(())
        last_input = time.monotonic()
        while context.is_active() and not stopped.is_set():
            now = time.monotonic()
            if now - last_input >= IDLE_TIMEOUT_S:
                context.abort(grpc.StatusCode.DEADLINE_EXCEEDED, "Teleop input timed out")
            if client_id is not None and now >= min(next_window, next_control):
                update = pb.TeleopReply(accepted=True, reason="input_ready", input_window=window)
                if now >= next_window:
                    grant(update)
                next_control = now + CONTROL_INTERVAL_S
                yield feedback(update)
            try:
                timeout = min(0.1, max(0.0, IDLE_TIMEOUT_S - (now - last_input)))
                if client_id is not None:
                    timeout = min(timeout, max(0.0, min(next_window, next_control) - time.monotonic()))
                request = pending.get(timeout=timeout)
            except queue.Empty:
                continue
            if request is None:
                return
            last_input = time.monotonic()
            identity = command(context, request.command)
            action = request.WhichOneof("action")
            if client_id is None:
                if action != "input" or request.sequence == 0:
                    context.abort(grpc.StatusCode.INVALID_ARGUMENT, "Teleop input handshake is required")
                client_id = identity["client_id"]
            elif client_id != identity["client_id"]:
                context.abort(grpc.StatusCode.INVALID_ARGUMENT, "Teleop client identity changed")
            reply = pb.TeleopReply(request_id=identity["request_id"], sequence=request.sequence)
            if request.sequence <= last_sequence:
                reply.reason = "stale_sequence"
                yield feedback(reply)
                continue
            last_sequence = request.sequence
            if action == "input":
                reply.accepted = True
                reply.reason = "input_ready"
                grant(reply)
            elif action == "velocity":
                velocity = request.velocity
                deadline = next((expiry for token, expiry in windows if token == request.input_window), 0.0)
                valid_window = time.monotonic() < deadline
                if not velocity.deadman:
                    windows.clear()
                    next_window = 0.0
                    result = session.hold(request_id=identity["request_id"], reason="sdk_deadman_released")
                elif not valid_window:
                    windows.clear()
                    next_window = 0.0
                    held = session.hold(request_id=identity["request_id"], reason="sdk_input_expired")
                    reply.reason = "input_expired" if held.accepted else "input_expired_hold_unconfirmed"
                    reply.final_output_confirmed = held.final_output_confirmed
                    yield feedback(grant(reply))
                    continue
                elif not all(math.isfinite(value) for value in (velocity.vx_m_s, velocity.vy_m_s, velocity.yaw_rad_s)):
                    context.abort(grpc.StatusCode.INVALID_ARGUMENT, "Teleop velocity must be finite")
                else:
                    with gateway._state_lock:
                        safety = getattr(gateway, "_navigation_state", None)
                    if safety_stop_active(safety):
                        reply.reason = "safety_stop"
                        yield feedback(reply)
                        continue
                    result = session.move(
                        velocity.vx_m_s,
                        velocity.vy_m_s,
                        velocity.yaw_rad_s,
                        request_id=identity["request_id"],
                        manual_mode=velocity.manual_mode,
                        deadline_monotonic_s=deadline,
                    )
                    reply.ingress_accepted = result.accepted
                reply.accepted = result.accepted
                reply.reason = result.reason
                reply.final_output_confirmed = result.final_output_confirmed
                if not velocity.deadman:
                    grant(reply)
            elif action == "claim":
                result = session.claim()
                if result.reason == "claim_failed":
                    _abort(context, grpc.StatusCode.UNAVAILABLE, {
                        "reason_code": "control_claim_ack_unknown", "request_id": identity["request_id"],
                    }, "Native control claim acknowledgement is unavailable")
                reply.accepted = result.accepted
                reply.reason = result.reason
            elif action in {"hold", "close"}:
                windows.clear()
                next_window = 0.0
                if action == "close":
                    opened = False
                result = (
                    session.disconnect(request_id=identity["request_id"])
                    if action == "close"
                    else session.hold(request_id=identity["request_id"], reason="sdk_operator_hold")
                )
                reply.accepted = result.accepted
                reply.reason = result.reason
                reply.final_output_confirmed = result.final_output_confirmed
                if action == "close":
                    yield feedback(reply)
                    return
                grant(reply)
            else:
                context.abort(grpc.StatusCode.INVALID_ARGUMENT, "Teleop action is required")
            yield feedback(reply)
    finally:
        stopped.set()
        try:
            if opened:
                result = session.disconnect(request_id=f"{source_id}:disconnect")
                if not result.accepted:
                    logger.error("SDK teleop disconnect unconfirmed: %s", result.reason)
        finally:
            slots.release()

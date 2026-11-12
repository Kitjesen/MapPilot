"""External joint control sessions use native ownership and robot-issued windows."""

import logging
import math
import queue
import threading
import time
from collections import deque
from collections.abc import Mapping
from uuid import uuid4

import grpc

from gateway.navigation.commands import navigation_commands
from gateway.services.safety_status import safety_stop_active

from .codec import _abort, command
from .generated import motion_pb2 as pb

logger = logging.getLogger(__name__)
STATUS_INTERVAL_S = 0.02
IDLE_TIMEOUT_S = 0.3
UNKNOWN_REASONS = {
    "posture_ack_timeout", "posture_cancel_requested", "posture_action_ack_unknown", "posture_publish_failed",
}
KINDS = {"enter": 16, "enable": 18, "disable": 14, "close": 17}


def _snapshot(gateway):
    read = getattr(navigation_commands(gateway), "external_status", None)
    if not callable(read):
        return pb.ControllerState(reason="external_status_unavailable"), 0
    try:
        snapshot = read()
    except (OSError, RuntimeError, ValueError) as error:
        return pb.ControllerState(reason=str(error)), 0
    if not isinstance(snapshot, Mapping):
        return pb.ControllerState(reason="external_status_unavailable"), 0
    result = pb.ControllerState(reason=snapshot.get("reason") or "")
    age = snapshot.get("received_age_ms")
    if isinstance(age, (float, int)) and math.isfinite(age) and age >= 0:
        result.received_age_ms = age
    count = snapshot.get("joint_count")
    if isinstance(count, int) and 0 < count <= 16:
        result.joint_count = count
    if snapshot.get("fresh") is not True:
        return result, 0
    result.fresh = True
    selected = snapshot.get("selected") is True
    result.mode = pb.CONTROLLER_MODE_EXTERNAL if selected else pb.CONTROLLER_MODE_BUILTIN
    result.session_id = snapshot.get("session_id") or ""
    result.active = selected and snapshot.get("active") is True
    result.motor_output_enabled = snapshot.get("motors_enabled") is True
    result.last_sequence = snapshot.get("last_sequence") or 0
    result.motor_disable_confirmed = snapshot.get("motor_disable_confirmed") is True
    window = snapshot.get("input_window") or 0
    return result, window if selected and isinstance(window, int) and 0 <= window < 2**64 else 0


def state(gateway):
    return _snapshot(gateway)[0]


class _Pending:
    """Keep one pending frame and preserve explicit session operations in order."""

    def __init__(self, stopped):
        self.stopped = stopped
        self.condition = threading.Condition()
        self.items = deque()

    def put(self, request):
        with self.condition:
            while self.items and not self.stopped.is_set():
                previous = self.items[0]
                if (
                    request is not None and previous is not None
                    and request.WhichOneof("action") == previous.WhichOneof("action") == "frame"
                    and request.sequence > previous.sequence
                    and request.command.client_id == previous.command.client_id
                ):
                    self.items[0] = request
                    return
                self.condition.wait(0.02)
            if not self.stopped.is_set():
                self.items.append(request)
                self.condition.notify_all()

    def get(self, timeout):
        with self.condition:
            if not self.condition.wait_for(lambda: bool(self.items), timeout):
                raise queue.Empty
            request = self.items.popleft()
            self.condition.notify_all()
            return request


def _requests(requests, pending, stopped):
    try:
        for request in requests:
            pending.put(request)
            if stopped.is_set():
                return
    except grpc.RpcError:
        pass
    finally:
        pending.put(None)


def _action(gateway, context, action, session_id, request_id):
    method = "external_exit" if action == "close" else f"external_{action}"
    operation = getattr(navigation_commands(gateway), method, None)
    if not callable(operation):
        context.abort(grpc.StatusCode.UNIMPLEMENTED, f"Native {method} is unavailable")
    values = {"reason": f"sdk_external_{action}", "request_id": request_id}
    if action != "enter":
        values["session_id"] = session_id
    try:
        result = operation(**values)
    except Exception as error:
        result = None
        reason = str(error)
    else:
        reason = str(result.get("reason") or "") if isinstance(result, Mapping) else "invalid native receipt"
    if (
        not isinstance(result, Mapping) or not isinstance(result.get("accepted"), bool)
        or result.get("request_id") != request_id or result.get("kind") != KINDS[action]
        or reason in UNKNOWN_REASONS
    ):
        _abort(context, grpc.StatusCode.UNAVAILABLE, {"detail": {
            "reason_code": f"external_{action}_ack_unknown", "request_id": request_id, "reason": reason,
        }})
    return result


def _admit(gateway, context, client_id, action):
    if action in {"close", "disable"}:
        return
    lease = getattr(gateway, "_lease", None)
    if lease is not None and not lease.check(client_id):
        _abort(context, grpc.StatusCode.PERMISSION_DENIED, {"detail": {
            "reason_code": "control_lease", "reason": "Another operator owns motion control",
        }})
    with gateway._state_lock:
        safety = getattr(gateway, "_navigation_state", None)
    if safety_stop_active(safety):
        _abort(context, grpc.StatusCode.FAILED_PRECONDITION, {"detail": {
            "reason_code": "safety_stop", "reason": "Safety STOP is active",
        }})


def control(gateway, requests, context, slots):
    if not slots.acquire(blocking=False):
        context.abort(grpc.StatusCode.RESOURCE_EXHAUSTED, "SDK stream limit reached")
    stopped = threading.Event()
    pending = _Pending(stopped)
    session_id = None
    client_id = None
    last_sequence = 0

    def feedback(reply):
        snapshot, window = _snapshot(gateway)
        reply.state.CopyFrom(snapshot)
        reply.input_window = window if snapshot.session_id == session_id else 0
        return reply

    try:
        if not context.add_callback(stopped.set):
            return
        threading.Thread(target=_requests, args=(requests, pending, stopped), daemon=True).start()
        last_input = time.monotonic()
        next_status = last_input
        while context.is_active() and not stopped.is_set():
            now = time.monotonic()
            if now - last_input >= IDLE_TIMEOUT_S:
                context.abort(grpc.StatusCode.DEADLINE_EXCEEDED, "External control input timed out")
            if session_id is not None and now >= next_status:
                next_status = now + STATUS_INTERVAL_S
                yield feedback(pb.ExternalControlReply())
            timeout = min(STATUS_INTERVAL_S, max(0, IDLE_TIMEOUT_S - (now - last_input)))
            try:
                request = pending.get(timeout)
            except queue.Empty:
                continue
            if request is None:
                return
            last_input = time.monotonic()
            identity = command(context, request.command)
            action = request.WhichOneof("action")
            if client_id is None:
                if action != "enter" or request.sequence == 0:
                    context.abort(grpc.StatusCode.INVALID_ARGUMENT, "External enter handshake is required")
                client_id = identity["client_id"]
            elif client_id != identity["client_id"]:
                context.abort(grpc.StatusCode.INVALID_ARGUMENT, "External client identity changed")
            reply = pb.ExternalControlReply(request_id=identity["request_id"], sequence=request.sequence)
            if action == "heartbeat" and request.sequence != 0:
                context.abort(grpc.StatusCode.INVALID_ARGUMENT, "External heartbeat sequence must be zero")
            if action != "heartbeat" and request.sequence <= last_sequence:
                reply.reason = "stale_sequence"
                yield feedback(reply)
                continue
            if action != "heartbeat":
                last_sequence = request.sequence
            _admit(gateway, context, client_id, action)
            if action == "enter":
                if session_id is not None:
                    context.abort(grpc.StatusCode.INVALID_ARGUMENT, "External session is already entered")
                session_id = identity["request_id"]
                result = _action(gateway, context, action, session_id, identity["request_id"])
                last_input = time.monotonic()
                reply.accepted = result["accepted"]
                reply.reason = result["reason"]
                if not reply.accepted:
                    session_id = None
                    yield feedback(reply)
                    return
            elif action in {"enable", "disable", "close"}:
                result = _action(gateway, context, action, session_id, identity["request_id"])
                last_input = time.monotonic()
                reply.accepted = result["accepted"]
                reply.reason = result["reason"]
                yield feedback(reply)
                if action == "close":
                    if reply.accepted:
                        session_id = None
                    return
                continue
            elif action in {"frame", "heartbeat"}:
                operation = getattr(navigation_commands(gateway), f"external_{action}", None)
                if not callable(operation):
                    context.abort(grpc.StatusCode.UNIMPLEMENTED, f"Native external_{action} is unavailable")
                if action == "frame":
                    snapshot, _ = _snapshot(gateway)
                    joints = request.frame.joints
                    if not snapshot.fresh or not snapshot.joint_count:
                        context.abort(grpc.StatusCode.UNAVAILABLE, "External joint layout is unavailable")
                    if len(joints) != snapshot.joint_count or request.input_window == 0:
                        context.abort(grpc.StatusCode.INVALID_ARGUMENT, "External frame requires the robot joint count and window")
                    values = [{name: getattr(joint, name) for name in ("q", "dq", "kp", "kd", "tau")} for joint in joints]
                    if not all(math.isfinite(value) for joint in values for value in joint.values()):
                        context.abort(grpc.StatusCode.INVALID_ARGUMENT, "External joint commands must be finite")
                    if any(joint["kp"] < 0 or joint["kd"] < 0 for joint in values):
                        context.abort(grpc.StatusCode.INVALID_ARGUMENT, "External joint gains must be nonnegative")
                    arguments = (session_id, request.sequence, values, request.input_window)
                else:
                    arguments = (session_id,)
                try:
                    queued = operation(*arguments)
                except Exception as error:
                    _abort(context, grpc.StatusCode.UNAVAILABLE, {"detail": {
                        "reason_code": f"external_{action}_ack_unknown",
                        "request_id": identity["request_id"], "reason": str(error),
                    }})
                reply.accepted = queued is True
                reply.ingress_accepted = queued is True
                reply.reason = "queued" if queued else "native_queue_rejected"
                if action == "heartbeat":
                    reply.request_id = ""
                    reply.sequence = 0
            else:
                context.abort(grpc.StatusCode.INVALID_ARGUMENT, "External action is required")
            yield feedback(reply)
    finally:
        stopped.set()
        try:
            if session_id is not None:
                try:
                    result = navigation_commands(gateway).external_disable(
                        session_id=session_id, reason="sdk_external_disconnect", request_id=f"external-disable-{uuid4()}",
                    )
                    if not isinstance(result, Mapping) or result.get("accepted") is not True:
                        logger.error("External disconnect disable acknowledgement is unconfirmed")
                except Exception:
                    logger.exception("External disconnect failed")
        finally:
            slots.release()

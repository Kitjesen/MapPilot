"""Shared resume admission preserves native receipts and request replay."""

import json
import time

import pytest
from fastapi import HTTPException
from fastapi.responses import JSONResponse

from gateway.gateway_module import GatewayModule
from gateway.schemas import StopRequest
from gateway.services.resume import resume
from runtime.msgs import NavigationCommandKind


class Controls:
    def __init__(self, *, accepted=True, reason="autonomy_already_ready"):
        self.calls = []
        self.accepted = accepted
        self.reason = reason
        self.failure = None
        self.returned_request_id = None

    def resume_autonomy_with_receipt(self, reason, *, request_id):
        self.calls.append((reason, request_id))
        if self.failure:
            raise self.failure
        return {
            "accepted": self.accepted,
            "kind": int(NavigationCommandKind.RESUME_AUTONOMY),
            "task_id": "",
            "request_id": self.returned_request_id or request_id,
            "reason": self.reason,
            "endpoint_timestamp_s": 123.5,
        }


def gateway(controls):
    robot = GatewayModule()
    robot.setup()
    robot.on_system_modules({"nav.commands": controls})
    return robot


def command():
    return StopRequest(client_id="sdk", request_id="resume-1")


def test_resume_replays_acknowledged_request_once():
    controls = Controls()
    robot = gateway(controls)
    first = resume(robot, command())
    repeated = resume(robot, command())
    assert first["ok"] is True
    assert first["command"]["name"] == "navigation_resume"
    assert first["command"]["replay"] is False
    assert repeated["command"]["replay"] is True
    assert repeated["native_request_id"] == "resume-1"
    assert controls.calls == [("operator_resume", "resume-1")]


def test_resume_rejects_other_control_owner_before_native_call():
    controls = Controls()
    robot = gateway(controls)
    assert robot._lease.acquire("other", 30)
    response = resume(robot, command())
    assert isinstance(response, JSONResponse)
    assert response.status_code == 403
    assert json.loads(response.body)["error"] == "control_lease"
    assert controls.calls == []


@pytest.mark.parametrize("reason", ["estop_latched", "operator_authority_active", "loop_hold"])
def test_resume_preserves_native_rejection_reason(reason):
    controls = Controls(accepted=False, reason=reason)
    response = resume(gateway(controls), command())
    assert response.status_code == 409
    payload = json.loads(response.body)
    assert payload["ok"] is False
    assert payload["command"]["accepted"] is False
    assert reason in payload["detail"]["reason"]
    assert controls.calls == [("operator_resume", "resume-1")]


def test_resume_native_ack_overrides_stale_mode_inference(monkeypatch):
    from gateway.services import native_control

    monkeypatch.setattr(native_control, "read_navigation_status", lambda: {
        "stamp_s": time.time(),
        "control_mode": "autonomy",
        "control_authority": {"resume_required": True},
    })
    controls = Controls(reason="autonomy_already_ready")
    result = resume(gateway(controls), command())
    assert result["native_reason"] == "autonomy_already_ready"
    assert result["native_endpoint_timestamp_s"] == 123.5
    assert result["resume_was_required"] is False
    assert result["goal_reissue_required"] is False
    assert result["previous_motion_restored"] is False


@pytest.mark.parametrize("failure", ["timeout", "wrong_request", "missing_acceptance"])
def test_resume_cannot_succeed_without_correlated_native_ack(failure):
    controls = Controls()
    if failure == "timeout":
        controls.failure = TimeoutError("native ACK timed out")
    elif failure == "wrong_request":
        controls.returned_request_id = "other-request"
    else:
        controls.accepted = None
    with pytest.raises(HTTPException) as error:
        resume(gateway(controls), command())
    assert error.value.status_code == 503
    assert error.value.detail["reason_code"] == "motion_resume_ack_unknown"
    assert error.value.detail["request_id"] == "resume-1"
    assert error.value.detail["reason"]
    assert "accepted" not in error.value.detail
    assert controls.calls == [("operator_resume", "resume-1")]

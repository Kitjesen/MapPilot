"""Tests for the four-axis public navigation projection."""

from __future__ import annotations

import json
import time
from typing import Any

import pytest

from gateway.navigation.projection import project_navigation_status

pytestmark = [pytest.mark.sim]


def _facts() -> dict[str, Any]:
    return {
        "navigation_state_fresh": True,
        "navigation_state": {
            "boot_id": "boot-1",
            "lifecycle_state_name": "EXECUTING",
            "active_task_id": "task-1",
            "active_request_id": "request-1",
            "recovery_state_name": "IDLE",
            "authority": "autonomy",
        },
        "goal_status": {
            "boot_id": "boot-1",
            "task_id": "task-1",
            "request_id": "request-1",
            "state_name": "EXECUTING",
            "reason": "",
        },
        "can_accept_goal": True,
        "blockers": [],
        "advisories": [],
        "control": {
            "command_owner": "autonomy",
            "resume_required": False,
            "estop_latched": False,
        },
        "native_endpoint": {
            "required": True,
            "status_available": True,
            "input_gate": {"ready": True},
            "motion_stop_evidence": {"state": "NOT_REQUESTED"},
        },
        "odometry": {"vx": 0.0, "wz": 0.0},
        "odometry_fresh": True,
    }


def test_runtime_hold_does_not_rewrite_task_as_paused() -> None:
    facts = _facts()
    facts["navigation_state"]["lifecycle_state_name"] = "PAUSED"
    facts.update(can_accept_goal=False, blockers=["native_input_gate_not_ready"])
    facts["native_endpoint"]["input_gate"] = {
        "ready": False,
        "reason": "stale_collision_map",
    }

    state = project_navigation_status(facts)

    assert state["task"] == {"state": "EXECUTING", "task_id": "task-1", "reason": ""}
    assert state["motion"]["permission"] == "HELD"
    assert state["motion"]["reason"] == "stale_collision_map"


@pytest.mark.parametrize(
    ("goal_state", "recovery", "expected"),
    [
        ("PAUSED", "IDLE", "PAUSED"),
        ("EXECUTING", "ACTIVE", "RECOVERING"),
        ("SUCCESS", "IDLE", "SUCCESS"),
        ("FAILED", "IDLE", "FAILED"),
        ("CANCELLED", "IDLE", "CANCELLED"),
    ],
)
def test_task_state_comes_from_exact_event_with_recovery_overlay(
    goal_state: str,
    recovery: str,
    expected: str,
) -> None:
    facts = _facts()
    facts["goal_status"]["state_name"] = goal_state
    facts["navigation_state"]["recovery_state_name"] = recovery

    state = project_navigation_status(facts)

    assert state["task"]["state"] == expected
    assert state["goal_admission"]["state"] == "ACCEPTING"


def test_unrelated_task_event_is_not_used() -> None:
    facts = _facts()
    facts["goal_status"]["task_id"] = "other-task"

    state = project_navigation_status(facts)

    assert state["task"] == {
        "state": "UNKNOWN",
        "task_id": "task-1",
        "reason": "task_status_unavailable",
    }


def test_stale_task_source_does_not_erase_independent_axes() -> None:
    facts = _facts()
    facts["navigation_state_fresh"] = False

    state = project_navigation_status(facts)

    assert state["task"]["state"] == "UNKNOWN"
    assert state["goal_admission"]["state"] == "ACCEPTING"
    assert state["control"]["authority"] == "AUTONOMY"
    assert state["motion"] == {
        "permission": "CLEAR",
        "observation": "QUIET",
        "stop_confirmation": "NOT_REQUESTED",
        "reason": "motion_clear",
    }


@pytest.mark.parametrize(
    ("evidence", "vx", "observation", "confirmation", "reason"),
    [
        ({"state": "PENDING", "reason": "driver_ack_pending"}, 0.0, "QUIET", "PENDING", "driver_ack_pending"),
        ({"state": "CONFIRMED", "reason": "stop_confirmed"}, 0.0, "QUIET", "CONFIRMED", "stop_confirmed"),
        ({"state": "FAILED", "reason": "timed_out"}, 0.0, "QUIET", "FAILED", "timed_out"),
        (
            {"state": "NOT_REQUESTED", "reason": "nonzero_output_published"},
            0.04,
            "MOVING",
            "NOT_REQUESTED",
            "nonzero_output_published",
        ),
    ],
)
def test_motion_observation_and_stop_evidence_remain_independent(
    evidence: dict[str, str],
    vx: float,
    observation: str,
    confirmation: str,
    reason: str,
) -> None:
    facts = _facts()
    facts["native_endpoint"]["motion_stop_evidence"] = evidence
    facts["odometry"]["vx"] = vx

    motion = project_navigation_status(facts)["motion"]

    assert motion["observation"] == observation
    assert motion["stop_confirmation"] == confirmation
    assert motion["reason"] == reason


def test_estop_has_no_control_authority_and_blocks_motion() -> None:
    facts = _facts()
    facts["navigation_state"].update(
        lifecycle_state_name="IDLE",
        active_task_id="",
        active_request_id="",
        authority="estop",
    )
    facts["goal_status"] = None
    facts["control"]["estop_latched"] = True

    state = project_navigation_status(facts)

    assert state["task"]["state"] == "IDLE"
    assert state["control"]["authority"] == "NONE"
    assert state["motion"]["permission"] == "ESTOPPED"


def test_operator_takeover_is_projected_as_requiring_resume() -> None:
    facts = _facts()
    facts["navigation_state"].update(
        lifecycle_state_name="IDLE",
        active_task_id="",
        active_request_id="",
        authority="operator",
    )
    facts["goal_status"] = None
    facts["control"].update(
        active_cmd_source="teleop",
        operator_takeover_latched=True,
        resume_required=False,
    )
    facts["native_endpoint"].update(active_cmd_source="teleop")

    state = project_navigation_status(facts)

    assert state["control"] == {
        "authority": "OPERATOR",
        "resume_required": True,
        "reason": "operator_takeover",
    }
    assert state["motion"]["permission"] == "HELD"


def test_motion_hold_reason_is_not_hidden_by_confirmed_stop() -> None:
    facts = _facts()
    facts.update(can_accept_goal=False, blockers=["resume_required"])
    facts["control"]["resume_required"] = True
    facts["native_endpoint"]["motion_stop_evidence"] = {
        "state": "CONFIRMED",
        "reason": "stop_confirmed",
    }

    motion = project_navigation_status(facts)["motion"]

    assert motion == {
        "permission": "HELD",
        "observation": "QUIET",
        "stop_confirmation": "CONFIRMED",
        "reason": "resume_required",
    }


def test_gateway_selects_only_the_active_task_event() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.navigation.status import build_navigation_status

    gateway = GatewayModule()
    active = {
        "boot_id": "boot-1",
        "task_id": "task-1",
        "request_id": "request-1",
        "state_name": "EXECUTING",
    }
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "vx": 0.0, "wz": 0.0}
        gateway._navigation_state = {
            "ts": time.time(),
            "boot_id": "boot-1",
            "lifecycle_state_name": "EXECUTING",
            "active_task_id": "task-1",
            "active_request_id": "request-1",
            "recovery_state_name": "IDLE",
            "authority": "autonomy",
        }
        gateway._navigation_goal_status_by_task.update(
            {"task-1": active, "other-task": {**active, "task_id": "other-task"}}
        )
        gateway._latest_navigation_goal_status = {**active, "task_id": "other-task"}

    state = build_navigation_status(gateway)

    assert state["task"] == {"state": "EXECUTING", "task_id": "task-1", "reason": ""}


def test_gateway_falls_through_stale_task_cache_to_exact_request_event() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.navigation.status import build_navigation_status

    gateway = GatewayModule()
    exact = {
        "boot_id": "boot-2",
        "task_id": "task-1",
        "request_id": "request-new",
        "state_name": "EXECUTING",
    }
    with gateway._state_lock:
        gateway._navigation_state = {
            "ts": time.time(),
            "boot_id": "boot-2",
            "lifecycle_state_name": "EXECUTING",
            "active_task_id": "task-1",
            "active_request_id": "request-new",
            "recovery_state_name": "IDLE",
            "authority": "autonomy",
        }
        gateway._navigation_goal_status_by_task["task-1"] = {
            **exact,
            "request_id": "request-old",
        }
        gateway._navigation_goal_status_by_request["request-new"] = exact

    state = build_navigation_status(gateway)

    assert state["task"] == {"state": "EXECUTING", "task_id": "task-1", "reason": ""}


def test_gateway_receive_age_invalidates_optimistic_pose_freshness() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.navigation.status import build_navigation_status, evaluate_navigation_gate

    gateway = GatewayModule()
    gateway._session_snapshot = lambda: {"mode": "navigating"}
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "vx": 0.0, "wz": 0.0}
        gateway._odom_timestamps.append(time.time() - 60.0)
        gateway._localization_status = {"state": "TRACKING", "pose_fresh": True}

    gate = evaluate_navigation_gate(gateway)
    status = build_navigation_status(gateway)

    assert gate["can_accept_goal"] is None
    assert gate["reason"] == "odometry_stale"
    assert status["goal_admission"] == {"state": "UNKNOWN", "reason": "odometry_stale"}
    assert status["motion"]["observation"] == "UNKNOWN"


def test_gateway_reads_stop_evidence_from_native_status(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
) -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.navigation.status import build_navigation_status

    status_path = tmp_path / "nav_endpoint_status.json"
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(status_path))
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "30")
    status_path.write_text(
        json.dumps(
            {
                "stamp_s": time.time(),
                "control_loop_health": {"ready": True, "healthy": True},
                "input_gate": {"ready": True, "reason": "ready"},
                "active_cmd_source": "none",
                "control_authority": {"owner": "native_endpoint"},
                "control_mode": "autonomy",
                "global_planner": "octoplanner3d",
                "planner_map": "/maps/active/octomap.ot",
                "publish_cmd_vel": True,
                "motion_stop_evidence": {
                    "state": "CONFIRMED",
                    "reason": "stop_confirmed",
                    "output_sequence": 17,
                },
            }
        ),
        encoding="utf-8",
    )
    gateway = GatewayModule()
    gateway._compiled_command_output_mode = ""
    gateway._session_snapshot = lambda: {"mode": "navigating", "product": "nav"}
    with gateway._state_lock:
        gateway._mode = "autonomous"
        gateway._odom = {"x": 0.0, "y": 0.0, "vx": 0.0, "wz": 0.0}
        gateway._odom_timestamps.append(time.time())
        gateway._localization_status = {"state": "TRACKING", "pose_fresh": True}
        gateway._navigation_state = {
            "ts": time.time(),
            "boot_id": "boot-1",
            "lifecycle_state_name": "IDLE",
            "active_task_id": "",
            "active_request_id": "",
            "authority": "none",
        }

    motion = build_navigation_status(gateway)["motion"]

    assert motion["stop_confirmation"] == "CONFIRMED"
    assert motion["observation"] == "QUIET"


def test_openapi_exposes_only_the_public_navigation_contract() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    schemas = gateway._app.openapi()["components"]["schemas"]
    properties = schemas["NavigationStatusResponse"]["properties"]

    assert set(properties) == {
        "schema_version",
        "task",
        "goal_admission",
        "control",
        "motion",
        "ts",
    }


def test_http_sse_and_snapshot_share_one_navigation_projection(monkeypatch) -> None:
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule
    from gateway.navigation import status
    from gateway.navigation.routes import register_navigation_routes
    from gateway.services.state_snapshot import build_state_snapshot

    gateway = GatewayModule()
    gate = {**_facts(), "ts": 123.0}
    gate["native_endpoint"]["input_gate"] = {"ready": False, "reason": "stale_collision_map"}
    gate.update(can_accept_goal=False, blockers=["native_input_gate_not_ready"])
    calls = []

    def evaluate(owner, *, facts=None):
        assert owner is gateway
        calls.append(facts)
        return gate

    monkeypatch.setattr(status, "evaluate_navigation_gate", evaluate)
    events = []
    monkeypatch.setattr(gateway, "push_event", events.append)
    app = FastAPI()
    register_navigation_routes(app, gateway)
    with TestClient(app) as client:
        response = client.get("/api/v1/navigation/status")
    assert response.status_code == 200
    http_status = response.json()
    status.handle_navigation_state(gateway, gate["navigation_state"])
    snapshot = build_state_snapshot(gateway)

    assert events == [{"type": "navigation_status", "data": http_status}]
    assert snapshot["navigation"] == http_status
    assert http_status["task"]["state"] == "EXECUTING"
    assert http_status["motion"]["permission"] == "HELD"
    assert len(calls) == 3
    assert calls[-1] is not None

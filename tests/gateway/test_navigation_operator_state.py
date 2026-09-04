from __future__ import annotations

import json
import time

import pytest

pytestmark = [pytest.mark.sim]


def test_path_active_with_stale_input_gate_keeps_task_executing_and_holds_motion() -> None:
    from gateway.services.navigation_operator_state import (
        project_navigation_operator_state,
    )

    operator_state = project_navigation_operator_state(
        navigation_state_fresh=True,
        navigation_state={
            "boot_id": "navd-boot-1",
            "lifecycle_state_name": "PAUSED",
            "active_task_id": "navigation-task-1",
            "active_request_id": "goal-1",
            "recovery_state_name": "IDLE",
            "authority": "autonomy",
            "hold_reason": "stale_collision_map",
            "progress": 0.4,
        },
        goal_status={
            "boot_id": "navd-boot-1",
            "task_id": "navigation-task-1",
            "request_id": "goal-1",
            "state_name": "EXECUTING",
            "terminal": False,
            "reason": "",
        },
        readiness={
            "can_accept_goal": False,
            "blockers": ["native_input_gate_not_ready"],
            "advisories": [],
        },
        control={
            "command_owner": "autonomy",
            "resume_required": False,
            "estop_latched": False,
        },
        native_endpoint={
            "required": True,
            "status_available": True,
            "input_gate": {"ready": False, "reason": "stale_collision_map"},
            "blockers": ["native_input_gate_not_ready"],
            "motion_stop_evidence": {
                "state": "NOT_REQUESTED",
                "reason": "not_requested",
            },
        },
        odometry={"vx": 0.0, "wz": 0.0},
        odometry_fresh=True,
    )

    assert operator_state["task"]["state"] == "EXECUTING"
    assert operator_state["task"]["task_id"] == "navigation-task-1"
    assert operator_state["motion"]["permission"] == "HELD"
    assert operator_state["motion"]["reason"] == "stale_collision_map"
    assert operator_state["summary"]["code"] == "MOTION_HELD"


def test_only_explicit_goal_status_pause_marks_task_paused() -> None:
    from gateway.services.navigation_operator_state import (
        project_navigation_operator_state,
    )

    operator_state = project_navigation_operator_state(
        navigation_state_fresh=True,
        navigation_state={
            "boot_id": "navd-boot-1",
            "lifecycle_state_name": "PAUSED",
            "active_task_id": "navigation-task-1",
            "active_request_id": "goal-1",
            "recovery_state_name": "IDLE",
            "authority": "autonomy",
            "progress": 0.4,
        },
        goal_status={
            "boot_id": "navd-boot-1",
            "task_id": "navigation-task-1",
            "request_id": "goal-1",
            "state_name": "PAUSED",
            "terminal": False,
            "reason": "operator_requested",
        },
        readiness={"can_accept_goal": True, "blockers": [], "advisories": []},
        control={
            "command_owner": "autonomy",
            "resume_required": False,
            "estop_latched": False,
        },
        native_endpoint={
            "required": True,
            "status_available": True,
            "input_gate": {"ready": True},
            "blockers": [],
            "motion_stop_evidence": {"state": "NOT_REQUESTED"},
        },
        odometry={"vx": 0.0, "wz": 0.0},
        odometry_fresh=True,
    )

    assert operator_state["task"] == {
        "state": "PAUSED",
        "task_id": "navigation-task-1",
        "request_id": "goal-1",
        "terminal": False,
        "progress": 0.4,
        "reason": "operator_requested",
    }
    assert operator_state["summary"] == {
        "severity": "INFO",
        "code": "TASK_PAUSED",
        "next_action": "resume_or_cancel",
    }


def test_recovery_overlay_and_motion_hold_remain_independent() -> None:
    from gateway.services.navigation_operator_state import (
        project_navigation_operator_state,
    )

    operator_state = project_navigation_operator_state(
        navigation_state_fresh=True,
        navigation_state={
            "boot_id": "navd-boot-2",
            "active_task_id": "navigation-task-2",
            "active_request_id": "goal-2",
            "recovery_state_name": "ACTIVE",
            "authority": "recovery",
            "hold_reason": "collision_map_refreshing",
            "progress": 0.55,
        },
        goal_status={
            "boot_id": "navd-boot-2",
            "task_id": "navigation-task-2",
            "request_id": "goal-2",
            "state_name": "EXECUTING",
            "reason": "",
        },
        readiness={
            "can_accept_goal": False,
            "blockers": ["native_input_gate_not_ready"],
            "advisories": [],
        },
        control={
            "command_owner": "recovery",
            "resume_required": False,
            "estop_latched": False,
        },
        native_endpoint={
            "required": True,
            "status_available": True,
            "input_gate": {"ready": False, "reason": "collision_map_refreshing"},
            "motion_stop_evidence": {"state": "NOT_REQUESTED"},
        },
        odometry={"vx": 0.0, "wz": 0.0},
        odometry_fresh=True,
    )

    assert operator_state["task"]["state"] == "RECOVERING"
    assert operator_state["control"]["authority"] == "AUTONOMY"
    assert operator_state["motion"]["permission"] == "HELD"
    assert operator_state["summary"]["code"] == "MOTION_HELD"


def test_fresh_navigation_estop_is_not_reduced_to_a_generic_hold() -> None:
    from gateway.services.navigation_operator_state import (
        project_navigation_operator_state,
    )

    operator_state = project_navigation_operator_state(
        navigation_state_fresh=True,
        navigation_state={
            "boot_id": "navd-boot-estop",
            "lifecycle_state_name": "IDLE",
            "active_task_id": "",
            "active_request_id": "",
            "authority": "estop",
            "hold_reason": "operator_estop",
        },
        goal_status=None,
        readiness={
            "can_accept_goal": False,
            "blockers": ["emergency_stop"],
            "advisories": [],
        },
        control={
            "command_owner": "estop",
            "active_cmd_source": "unknown",
            "resume_required": False,
            "estop_latched": False,
        },
        native_endpoint={
            "required": False,
            "status_available": None,
            "motion_stop_evidence": {"state": "PENDING"},
        },
        odometry={"vx": 0.0, "wz": 0.0},
        odometry_fresh=True,
    )

    assert operator_state["control"]["authority"] == "NONE"
    assert operator_state["motion"]["permission"] == "ESTOPPED"
    assert operator_state["summary"] == {
        "severity": "CRITICAL",
        "code": "ESTOPPED",
        "next_action": "clear_estop",
    }


@pytest.mark.parametrize(
    ("goal_state", "task_state"),
    [
        ("SUCCESS", "SUCCESS"),
        ("FAILED", "FAILED"),
        ("CANCELLED", "CANCELLED"),
    ],
)
def test_terminal_task_is_retained_while_new_goal_is_accepted(
    goal_state: str,
    task_state: str,
) -> None:
    from gateway.services.navigation_operator_state import (
        project_navigation_operator_state,
    )

    operator_state = project_navigation_operator_state(
        navigation_state_fresh=True,
        navigation_state={
            "boot_id": "navd-boot-3",
            "active_task_id": "navigation-task-3",
            "active_request_id": "goal-3",
            "recovery_state_name": "IDLE",
            "authority": "none",
            "progress": 1.0 if goal_state == "SUCCESS" else -1.0,
        },
        goal_status={
            "boot_id": "navd-boot-3",
            "task_id": "navigation-task-3",
            "request_id": "goal-3",
            "state_name": goal_state,
            "reason": "planner_failed" if goal_state == "FAILED" else "",
        },
        readiness={"can_accept_goal": True, "blockers": [], "advisories": []},
        control={
            "command_owner": "none",
            "resume_required": False,
            "estop_latched": False,
        },
        native_endpoint={
            "required": True,
            "status_available": True,
            "input_gate": {"ready": True},
            "motion_stop_evidence": {"state": "NOT_REQUESTED"},
        },
        odometry={"vx": 0.0, "wz": 0.0},
        odometry_fresh=True,
    )

    assert operator_state["task"]["state"] == task_state
    assert operator_state["task"]["terminal"] is True
    assert operator_state["goal_admission"]["state"] == "ACCEPTING"


def test_navigation_status_never_falls_back_to_another_tasks_latest_event() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "vx": 0.0, "wz": 0.0}
        gateway._navigation_state = {
            "ts": time.time(),
            "boot_id": "navd-boot-new",
            "lifecycle_state_name": "EXECUTING",
            "active_task_id": "navigation-task-new",
            "active_request_id": "goal-new",
            "recovery_state_name": "IDLE",
            "authority": "autonomy",
            "progress": 0.1,
        }
        old_status = {
            "boot_id": "navd-boot-old",
            "task_id": "navigation-task-old",
            "request_id": "goal-old",
            "state_name": "SUCCESS",
            "terminal": True,
            "reason": "",
        }
        gateway._navigation_goal_status_by_task["navigation-task-old"] = old_status
        gateway._navigation_goal_status_by_request["goal-old"] = old_status
        gateway._latest_navigation_goal_status = old_status

    payload = build_navigation_status(gateway)

    assert payload["goal_status"] is None
    assert payload["operator_state"]["task"] == {
        "state": "UNKNOWN",
        "task_id": "navigation-task-new",
        "request_id": "goal-new",
        "terminal": False,
        "progress": None,
        "reason": "task_status_unavailable",
    }


def test_navigation_status_selects_exact_active_task_amid_interleaved_events() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    gateway = GatewayModule()
    active_status = {
        "boot_id": "navd-boot-shared",
        "task_id": "navigation-task-active",
        "request_id": "goal-active",
        "state_name": "EXECUTING",
        "terminal": False,
        "reason": "",
    }
    queued_status = {
        "boot_id": "navd-boot-shared",
        "task_id": "navigation-task-queued",
        "request_id": "goal-queued",
        "state_name": "PLANNING",
        "terminal": False,
        "reason": "",
    }
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "vx": 0.0, "wz": 0.0}
        gateway._navigation_state = {
            "ts": time.time(),
            "boot_id": "navd-boot-shared",
            "lifecycle_state_name": "EXECUTING",
            "active_task_id": "navigation-task-active",
            "active_request_id": "goal-active",
            "recovery_state_name": "IDLE",
            "authority": "autonomy",
            "progress": 0.3,
        }
        gateway._navigation_goal_status_by_task.update(
            {
                "navigation-task-active": active_status,
                "navigation-task-queued": queued_status,
            }
        )
        gateway._navigation_goal_status_by_request.update({"goal-active": active_status, "goal-queued": queued_status})
        gateway._latest_navigation_goal_status = queued_status

    payload = build_navigation_status(gateway)

    assert payload["goal_status"]["task_id"] == "navigation-task-active"
    assert payload["operator_state"]["task"]["state"] == "EXECUTING"
    assert payload["operator_state"]["task"]["task_id"] == "navigation-task-active"
    assert payload["operator_state"]["task"]["request_id"] == "goal-active"


def test_stale_required_sources_are_unknown_instead_of_optimistically_clear() -> None:
    from gateway.services.navigation_operator_state import (
        project_navigation_operator_state,
    )

    operator_state = project_navigation_operator_state(
        navigation_state_fresh=True,
        navigation_state={
            "boot_id": "navd-boot-4",
            "active_task_id": "navigation-task-4",
            "active_request_id": "goal-4",
            "recovery_state_name": "IDLE",
            "authority": "autonomy",
            "progress": 0.2,
        },
        goal_status={
            "boot_id": "navd-boot-4",
            "task_id": "navigation-task-4",
            "request_id": "goal-4",
            "state_name": "EXECUTING",
            "reason": "",
        },
        readiness={
            "can_accept_goal": False,
            "blockers": ["native_endpoint_status_missing_or_stale", "pose_stale"],
            "advisories": [],
        },
        control={
            "command_owner": "autonomy",
            "resume_required": False,
            "estop_latched": False,
        },
        native_endpoint={
            "required": True,
            "status_available": False,
            "blockers": ["native_endpoint_status_missing_or_stale"],
        },
        odometry={"vx": 0.0, "wz": 0.0},
        odometry_fresh=False,
    )

    assert operator_state["task"]["state"] == "EXECUTING"
    assert operator_state["goal_admission"]["state"] == "UNKNOWN"
    assert operator_state["control"] == {
        "authority": "UNKNOWN",
        "resume_required": False,
        "reason": "control_state_unknown",
    }
    assert operator_state["motion"]["permission"] == "UNKNOWN"
    assert operator_state["motion"]["observation"] == "UNKNOWN"
    assert operator_state["motion"]["stop_confirmation"] == "UNKNOWN"
    assert operator_state["summary"] == {
        "severity": "WARNING",
        "code": "STATUS_SOURCE_UNKNOWN",
        "next_action": "check_status_sources",
    }


def test_stale_odometry_makes_admission_and_motion_observation_unknown() -> None:
    from gateway.services.navigation_operator_state import (
        project_navigation_operator_state,
    )

    operator_state = project_navigation_operator_state(
        navigation_state_fresh=True,
        navigation_state={
            "boot_id": "navd-boot-4",
            "lifecycle_state_name": "IDLE",
            "active_task_id": "",
            "active_request_id": "",
            "authority": "autonomy",
        },
        goal_status=None,
        readiness={
            "can_accept_goal": False,
            "blockers": ["pose_stale"],
            "advisories": [],
        },
        control={
            "command_owner": "autonomy",
            "resume_required": False,
            "estop_latched": False,
        },
        native_endpoint={
            "required": True,
            "status_available": True,
            "input_gate": {"ready": True},
            "motion_stop_evidence": {"state": "NOT_REQUESTED"},
        },
        odometry={"vx": 0.0, "wz": 0.0},
        odometry_fresh=False,
    )

    assert operator_state["goal_admission"]["state"] == "UNKNOWN"
    assert operator_state["control"]["authority"] == "AUTONOMY"
    assert operator_state["motion"]["observation"] == "UNKNOWN"
    assert operator_state["summary"]["code"] == "STATUS_SOURCE_UNKNOWN"


def test_stale_idle_navigation_state_is_not_presented_as_fresh_idle() -> None:
    from gateway.services.navigation_operator_state import (
        project_navigation_operator_state,
    )

    operator_state = project_navigation_operator_state(
        navigation_state={
            "boot_id": "navd-boot-stale",
            "lifecycle_state_name": "IDLE",
            "active_task_id": "",
            "active_request_id": "",
            "authority": "none",
        },
        navigation_state_fresh=False,
        goal_status=None,
        readiness={"can_accept_goal": True, "blockers": [], "advisories": []},
        control={"command_owner": "none", "resume_required": False, "estop_latched": False},
        native_endpoint={
            "required": True,
            "status_available": True,
            "active_cmd_source": "none",
            "input_gate": {"ready": True},
            "motion_stop_evidence": {"state": "NOT_REQUESTED"},
        },
        odometry={"vx": 0.0, "wz": 0.0},
        odometry_fresh=True,
    )

    assert operator_state["task"] == {
        "state": "UNKNOWN",
        "task_id": "",
        "request_id": "",
        "terminal": False,
        "progress": None,
        "reason": "navigation_state_stale",
    }
    assert operator_state["goal_admission"]["state"] == "UNKNOWN"
    assert operator_state["control"]["authority"] == "UNKNOWN"
    assert operator_state["motion"] == {
        "permission": "UNKNOWN",
        "observation": "UNKNOWN",
        "stop_confirmation": "UNKNOWN",
        "linear_speed_mps": None,
        "angular_speed_radps": None,
        "reason": "navigation_state_stale",
    }
    assert operator_state["summary"]["code"] == "STATUS_SOURCE_UNKNOWN"


def test_quiet_odometry_does_not_infer_stop_confirmation() -> None:
    from gateway.services.navigation_operator_state import (
        project_navigation_operator_state,
    )

    operator_state = project_navigation_operator_state(
        navigation_state_fresh=True,
        navigation_state={
            "boot_id": "navd-boot-5",
            "lifecycle_state_name": "IDLE",
            "active_task_id": "",
            "active_request_id": "",
            "authority": "none",
        },
        goal_status=None,
        readiness={"can_accept_goal": True, "blockers": [], "advisories": []},
        control={"command_owner": "none", "resume_required": False, "estop_latched": False},
        native_endpoint={
            "required": True,
            "status_available": True,
            "input_gate": {"ready": True},
            "motion_stop_evidence": {
                "state": "PENDING",
                "reason": "driver_ack_pending",
                "driver_ack_observed": False,
                "quiet_odometry_samples": 8,
                "required_quiet_odometry_samples": 8,
                "linear_speed_threshold_mps": 0.03,
                "angular_speed_threshold_radps": 0.08,
            },
        },
        odometry={"vx": 0.0, "wz": 0.0},
        odometry_fresh=True,
    )

    assert operator_state["motion"]["observation"] == "QUIET"
    assert operator_state["motion"]["stop_confirmation"] == "PENDING"
    assert operator_state["summary"] == {
        "severity": "WARNING",
        "code": "STOP_CONFIRMATION_PENDING",
        "next_action": "wait_for_stop_confirmation",
    }


@pytest.mark.parametrize(
    ("evidence_state", "reason"),
    [
        ("CONFIRMED", "driver_and_quiet_odometry_confirmed"),
        ("FAILED", "driver_rejected"),
        ("FAILED", "timed_out"),
        ("NOT_REQUESTED", "nonzero_output_published"),
    ],
)
def test_native_stop_evidence_state_and_reason_are_preserved(
    evidence_state: str,
    reason: str,
) -> None:
    from gateway.services.navigation_operator_state import (
        project_navigation_operator_state,
    )

    operator_state = project_navigation_operator_state(
        navigation_state_fresh=True,
        navigation_state={
            "boot_id": "navd-boot-6",
            "lifecycle_state_name": "IDLE",
            "active_task_id": "",
            "active_request_id": "",
            "authority": "none",
        },
        goal_status=None,
        readiness={"can_accept_goal": True, "blockers": [], "advisories": []},
        control={"command_owner": "none", "resume_required": False, "estop_latched": False},
        native_endpoint={
            "required": True,
            "status_available": True,
            "input_gate": {"ready": True},
            "motion_stop_evidence": {
                "state": evidence_state,
                "reason": reason,
                "linear_speed_threshold_mps": 0.03,
                "angular_speed_threshold_radps": 0.08,
            },
        },
        odometry={"vx": 0.04 if reason == "nonzero_output_published" else 0.0, "wz": 0.0},
        odometry_fresh=True,
    )

    assert operator_state["motion"]["stop_confirmation"] == evidence_state
    assert operator_state["motion"]["reason"] == reason


def test_navigation_status_projects_fresh_native_stop_evidence(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
) -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

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
                "control_authority": {
                    "owner": "native_endpoint",
                    "operator_takeover_latched": False,
                    "resume_required": False,
                    "estop_latched": False,
                },
                "control_mode": "autonomy",
                "global_planner": "octoplanner3d",
                "planner_map": "/maps/active/octomap.ot",
                "publish_cmd_vel": True,
                "motion_stop_evidence": {
                    "state": "CONFIRMED",
                    "reason": "driver_and_quiet_odometry_confirmed",
                    "output_sequence": 17,
                    "updated_at_s": time.time(),
                    "confirmation_state": "confirmed",
                    "driver_ack_observed": True,
                    "driver_accepted": True,
                    "quiet_odometry_samples": 8,
                    "required_quiet_odometry_samples": 8,
                    "last_linear_speed_mps": 0.0,
                    "last_angular_speed_radps": 0.0,
                    "linear_speed_threshold_mps": 0.03,
                    "angular_speed_threshold_radps": 0.08,
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
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "pose_fresh": True,
            "localizer_health": "RECOVERED",
        }
        gateway._navigation_state = {
            "ts": time.time(),
            "boot_id": "navd-boot-7",
            "lifecycle_state_name": "IDLE",
            "active_task_id": "",
            "active_request_id": "",
            "authority": "none",
        }

    payload = build_navigation_status(gateway)

    assert payload["native_endpoint"]["motion_stop_evidence"]["output_sequence"] == 17
    assert payload["operator_state"]["motion"]["stop_confirmation"] == "CONFIRMED"
    assert payload["operator_state"]["motion"]["observation"] == "QUIET"


def test_navigation_status_openapi_declares_structured_operator_state() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    openapi = gateway._app.openapi()
    response_schema = openapi["paths"]["/api/v1/navigation/status"]["get"]["responses"]["200"]["content"][
        "application/json"
    ]["schema"]
    assert response_schema == {"$ref": "#/components/schemas/NavigationStatusResponse"}

    schemas = openapi["components"]["schemas"]
    navigation_properties = schemas["NavigationStatusResponse"]["properties"]
    assert navigation_properties["operator_state"] == {
        "$ref": "#/components/schemas/NavigationOperatorState",
    }
    operator_properties = schemas["NavigationOperatorState"]["properties"]
    assert set(operator_properties) == {
        "schema_version",
        "task",
        "goal_admission",
        "control",
        "motion",
        "summary",
    }
    task_ref = operator_properties["task"]["$ref"].rsplit("/", 1)[-1]
    task_state_schema = schemas[task_ref]["properties"]["state"]
    assert set(task_state_schema["enum"]) == {
        "IDLE",
        "PLANNING",
        "EXECUTING",
        "RECOVERING",
        "PAUSED",
        "SUCCESS",
        "FAILED",
        "CANCELLED",
        "UNKNOWN",
    }

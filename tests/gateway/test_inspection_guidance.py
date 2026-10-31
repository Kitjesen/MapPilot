from __future__ import annotations

import pytest

from gateway.services.inspection_guidance import build_inspection_guidance


def _task(**overrides):
    task = {
        "task_id": "inspection-task-42",
        "identity": {
            "task_id": "inspection-task-42",
            "route_id": "route-a",
            "route_revision": 7,
            "map_id": "field-map",
            "map_content_epoch": 42,
        },
        "execution_confirmed": True,
        "terminal": False,
        "current_state": "EXECUTING",
        "phase": "NAVIGATING",
        "delivery": {"continuity": "verified"},
        "latest_event": {"point_index": 0},
        "route_snapshot": {
            "points": [
                {
                    "id": "live-target",
                    "action": "capture:overview",
                    "enabled": True,
                    "x": 8.0,
                    "y": 9.0,
                    "z": 1.0,
                    "frame_id": "map",
                }
            ]
        },
    }
    task.update(overrides)
    return task


def _localization(**overrides):
    localization = {
        "state": "tracking",
        "pose_fresh": True,
        "odometry": {
            "frame_id": "map",
            "x": 1.0,
            "y": 2.0,
            "z": 0.2,
            "yaw": 0.5,
        },
    }
    localization.update(overrides)
    return localization


def test_guidance_uses_full_native_index_to_select_enabled_frozen_target() -> None:
    task = {
        "task_id": "inspection-task-42",
        "identity": {
            "task_id": "inspection-task-42",
            "route_id": "route-a",
            "route_revision": 7,
            "map_id": "field-map",
            "map_content_epoch": 42,
        },
        "execution_confirmed": True,
        "terminal": False,
        "current_state": "EXECUTING",
        "phase": "NAVIGATING",
        "delivery": {"continuity": "verified"},
        "latest_event": {"point_index": 1},
        "route_snapshot": {
            "points": [
                {
                    "id": "disabled-staging",
                    "action": "capture:staging",
                    "enabled": False,
                    "x": 1.0,
                    "y": 1.0,
                    "z": 0.0,
                    "frame_id": "map",
                },
                {
                    "id": "live-target",
                    "action": "capture:overview",
                    "enabled": True,
                    "x": 8.0,
                    "y": 9.0,
                    "z": 1.0,
                    "frame_id": "map",
                },
            ]
        },
    }
    localization = {
        "state": "tracking",
        "pose_fresh": True,
        "odometry": {
            "frame_id": "map",
            "x": 1.0,
            "y": 2.0,
            "z": 0.2,
            "yaw": 0.5,
        },
    }

    guidance = build_inspection_guidance(
        task,
        localization,
        active_map_id="field-map",
        active_map_content_epoch=42,
    )

    assert guidance == {
        "schema_version": "lingtu.inspection.guidance.v1",
        "task_id": "inspection-task-42",
        "identity": {
            "task_id": "inspection-task-42",
            "route_id": "route-a",
            "route_revision": 7,
            "map_id": "field-map",
            "map_content_epoch": 42,
        },
        "available": True,
        "reason": "",
        "point_id": "live-target",
        "phase": "NAVIGATING",
        "goal": {"frame_id": "map", "x": 8.0, "y": 9.0, "z": 1.0},
        "robot_pose": {
            "frame_id": "map",
            "x": 1.0,
            "y": 2.0,
            "z": 0.2,
            "yaw": 0.5,
        },
        "path": {
            "available": False,
            "reason": "execution_path_unbound",
            "points": [],
        },
    }


def test_guidance_is_unavailable_while_task_is_paused() -> None:
    guidance = build_inspection_guidance(
        _task(current_state="PAUSED", phase="PAUSED"),
        _localization(),
        active_map_id="field-map",
        active_map_content_epoch=42,
    )

    assert guidance["available"] is False
    assert guidance["reason"] == "task_paused"
    assert guidance["path"] == {
        "available": False,
        "reason": "execution_path_unbound",
        "points": [],
    }


def test_guidance_accepts_runtime_ready_as_confirmed_tracking() -> None:
    guidance = build_inspection_guidance(
        _task(),
        _localization(state="ready", reported_state="TRACKING"),
        active_map_id="field-map",
        active_map_content_epoch=42,
    )

    assert guidance["available"] is True
    assert guidance["reason"] == ""


def test_actual_runtime_localization_ready_state_can_produce_guidance() -> None:
    from gateway.services.runtime_status import build_localization_status_from_parts

    localization = build_localization_status_from_parts(
        odometry={
            "frame_id": "map",
            "x": 1.0,
            "y": 2.0,
            "z": 0.2,
            "yaw": 0.5,
        },
        session={"mode": "navigating", "localizer_ready": True},
        icp_quality=0.03,
        status={
            "state": "TRACKING",
            "pose_fresh": True,
            "confidence": 0.9,
            "degeneracy": "NONE",
            "localizer_health": "RECOVERED",
        },
    )
    assert localization["state"] == "ready"
    assert localization["reported_state"] == "TRACKING"
    assert localization["pose_fresh"] is True

    guidance = build_inspection_guidance(
        _task(),
        localization,
        active_map_id="field-map",
        active_map_content_epoch=42,
    )

    assert guidance["available"] is True
    assert guidance["robot_pose"]["frame_id"] == "map"


@pytest.mark.parametrize(
    (
        "task_overrides",
        "localization_overrides",
        "active_map_id",
        "active_map_epoch",
        "reason",
    ),
    [
        (
            {"terminal": True, "current_state": "SUCCESS", "phase": "SUCCEEDED"},
            {},
            "field-map",
            42,
            "task_terminal",
        ),
        (
            {"execution_confirmed": False},
            {},
            "field-map",
            42,
            "task_execution_unconfirmed",
        ),
        (
            {"delivery": {"continuity": "endpoint_restart_observed"}},
            {},
            "field-map",
            42,
            "task_continuity_unverified",
        ),
        ({}, {"pose_fresh": False}, "field-map", 42, "localization_pose_stale"),
        ({}, {"state": "lost"}, "field-map", 42, "localization_not_tracking"),
        (
            {},
            {
                "odometry": {
                    "frame_id": "odom",
                    "x": 1.0,
                    "y": 2.0,
                    "z": 0.2,
                    "yaw": 0.5,
                }
            },
            "field-map",
            42,
            "robot_pose_frame_mismatch",
        ),
        ({}, {}, "other-map", 42, "active_map_mismatch"),
        ({}, {}, "field-map", None, "active_map_epoch_unavailable"),
        ({}, {}, "field-map", 43, "active_map_epoch_mismatch"),
        (
            {
                "route_snapshot": {
                    "points": [
                        {
                            "id": "legacy-no-pose",
                            "action": "capture:overview",
                            "enabled": True,
                        }
                    ]
                }
            },
            {},
            "field-map",
            42,
            "goal_geometry_unavailable",
        ),
    ],
)
def test_guidance_unavailable_conditions(
    task_overrides,
    localization_overrides,
    active_map_id,
    active_map_epoch,
    reason,
) -> None:
    guidance = build_inspection_guidance(
        _task(**task_overrides),
        _localization(**localization_overrides),
        active_map_id=active_map_id,
        active_map_content_epoch=active_map_epoch,
    )

    assert guidance["available"] is False
    assert guidance["reason"] == reason
    assert guidance["path"] == {
        "available": False,
        "reason": "execution_path_unbound",
        "points": [],
    }


def test_inspection_guidance_route_projects_read_only_runtime_facts(monkeypatch) -> None:
    pytest.importorskip("fastapi")
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.routes.inspection as inspection
    from gateway.services.inspection_task_lifecycle import InspectionTaskTimeline

    timeline = InspectionTaskTimeline()
    task = _task()
    timeline.record_submission(
        task_id="inspection-task-42",
        action="start",
        request_id="inspection-request-42",
        product_session_id="inspection-sim-session",
        route_snapshot={
            **task["identity"],
            "id": "route-a",
            "revision": 7,
            "loop_count": 1,
            "failure_policy": "stop",
            **task["route_snapshot"],
        },
    )
    assert timeline.observe(
        {
            "boot_id": "navd-boot-a",
            "event_id": "event-1",
            "event_sequence": 1,
            "kind": 2,
            "task_id": "inspection-task-42",
            "request_id": "inspection-request-42",
            "state": 3,
            "map_id": "field-map",
            "map_content_epoch": 42,
            "route_id": "route-a",
            "route_revision": 7,
            "point_index": 0,
            "point_count": 1,
            "loop_index": 0,
            "retry_count": 0,
            "action": "capture:overview",
            "action_request_id": "",
            "evidence_id": "",
            "reason": "navigating",
            "ts": 10.0,
        }
    )

    gateway = type(
        "Gateway",
        (),
        {
            "_inspection_task_timeline": timeline,
            "_compiled_env": "sim",
            "_compiled_product": "inspection",
            "_compiled_run_plan": object(),
            "_compiled_product_session_id": "inspection-sim-session",
        },
    )()
    monkeypatch.setattr(
        inspection,
        "build_localization_status",
        lambda _gateway: _localization(),
        raising=False,
    )
    monkeypatch.setattr(inspection, "active_map", lambda _gateway: "field-map")
    monkeypatch.setattr(
        inspection,
        "_map_content_epoch",
        lambda _gateway, map_id: 42 if map_id == "field-map" else None,
    )
    app = FastAPI()
    inspection.register_inspection_routes(app, gateway)

    response = TestClient(app).get("/api/v1/inspection/tasks/inspection-task-42/guidance")

    assert response.status_code == 200
    assert response.json() == {
        "schema_version": "lingtu.inspection.guidance.v1",
        "task_id": "inspection-task-42",
        "product_session_id": "inspection-sim-session",
        "identity": {
            "task_id": "inspection-task-42",
            "route_id": "route-a",
            "route_revision": 7,
            "map_id": "field-map",
            "map_content_epoch": 42,
        },
        "available": True,
        "reason": "",
        "point_id": "live-target",
        "phase": "NAVIGATING",
        "goal": {"frame_id": "map", "x": 8.0, "y": 9.0, "z": 1.0},
        "robot_pose": {
            "frame_id": "map",
            "x": 1.0,
            "y": 2.0,
            "z": 0.2,
            "yaw": 0.5,
        },
        "path": {
            "available": False,
            "reason": "execution_path_unbound",
            "points": [],
        },
        "runtime": {
            "env": "sim",
            "product": "inspection",
            "product_session_id": "inspection-sim-session",
            "state": "active",
        },
    }

from __future__ import annotations

import asyncio
import json
import math
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

pytestmark = [pytest.mark.sim]
from pydantic import ValidationError

pytest.importorskip("fastapi")
from gateway.services.sse import subscribe, unsubscribe


def _endpoint(gateway, path: str):
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def _payload(response_or_payload):
    if hasattr(response_or_payload, "body"):
        return json.loads(response_or_payload.body)
    return response_or_payload


def _write_active_same_source_octomap(map_root: Path) -> Path:
    active_dir = map_root / "active"
    active_dir.mkdir(parents=True)
    map_path = active_dir / "map.pcd"
    octomap_path = active_dir / "octomap.ot"
    map_path.write_text(
        "\n".join(
            [
                "# .PCD v0.7 - Point Cloud Data file format",
                "VERSION 0.7",
                "FIELDS x y z",
                "SIZE 4 4 4",
                "TYPE F F F",
                "COUNT 1 1 1",
                "WIDTH 1",
                "HEIGHT 1",
                "VIEWPOINT 0 0 0 1 0 0 0",
                "POINTS 1",
                "DATA ascii",
                "0.0 0.0 0.0",
            ]
        )
        + "\n",
        encoding="ascii",
    )
    octomap_path.write_bytes(b"gateway-active-octomap")
    (active_dir / "metadata.json").write_text(
        json.dumps(
            {
                "schema_version": "lingtu.saved_map_artifacts.v1",
                "source_profile": "thunder",
                "data_source": "field",
                "slam_source": "fastlio2",
                "localization_source": "fastlio2",
                "mapping_source": "fastlio2",
                "frame_id": "map",
                "created_at": "2026-05-25T00:00:00Z",
                "artifacts": {
                    "map_pcd": {
                        "path": "map.pcd",
                        "source_profile": "thunder",
                        "data_source": "field",
                        "slam_source": "fastlio2",
                        "frame_id": "map",
                        "point_count": 1,
                    },
                    "octomap": {
                        "path": "octomap.ot",
                        "source_profile": "thunder",
                        "data_source": "field",
                        "frame_id": "map",
                        "resolution": 0.2,
                    },
                },
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    return active_dir


class _TypedActiveMapsService:
    def __init__(self, map_root: Path):
        self.map_root = map_root
        self.commands = []

    def execute(self, request):
        command = request.to_mapping()
        self.commands.append(command)
        action = command["action"]
        if action == "get_active":
            return {"action": action, "success": True, "active": "active"}
        if action == "validate_artifacts":
            map_dir = self.map_root / "active"
            octomap_ok = (map_dir / "octomap.ot").is_file()
            occupancy_ok = (map_dir / "occupancy.npz").is_file()
            gate = {
                "ok": (not command.get("require_octomap") or octomap_ok)
                and (not command.get("require_occupancy") or occupancy_ok),
                "artifacts": {
                    "octomap": {"exists": octomap_ok, "format_ok": octomap_ok},
                    "occupancy_grid": {"exists": occupancy_ok, "format_ok": occupancy_ok},
                },
                "blockers": [],
            }
            return {
                "action": action,
                "success": True,
                "gate": gate,
            }
        return {
            "action": action,
            "success": False,
            "reason_code": "unsupported_test_action",
        }

    def service(self, action, **arguments):
        public_action = "get_active" if action == "get_active_map" else action
        command = {"action": public_action, **arguments}
        return self.execute(SimpleNamespace(to_mapping=lambda: command))


def _attach_active_maps_service(gateway, map_root: Path) -> _TypedActiveMapsService:
    service = _TypedActiveMapsService(map_root)
    gateway._map_client = service
    return service


def _mark_navigation_ready(gateway) -> None:
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "z": 0.0, "ts": time.time()}
        gateway._navigation_state = {
            "lifecycle_state_name": "IDLE",
            "authority": "none",
        }
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "odom_age_ms": 100.0,
            "localizer_health": "RECOVERED",
        }


def _install_saved_location(
    gateway,
    *,
    name: str = "pump",
    position: tuple[float, float, float] = (1.0, 2.0, 0.0),
    yaw: float = 0.0,
    tags: tuple[str, ...] = ("inspection",),
) -> None:
    class Store:
        def __init__(self):
            self.entry = {
                "name": name,
                "position": list(position),
                "yaw": yaw,
                "tags": list(tags),
            }

        def list_all(self):
            return [self.entry]

        def query(self, requested: str):
            return self.entry if requested == name else None

        def query_fuzzy(self, requested: str):
            return None

    gateway._tagged_loc_module = SimpleNamespace(store=Store())


def test_readiness_request_defaults_to_simulation_mode():
    from gateway.schemas import InspectionAcceptanceRequest, ProductFieldCheckRequest

    assert ProductFieldCheckRequest().mode == "simulation"
    assert InspectionAcceptanceRequest().mode == "simulation"


def test_inspection_acceptance_request_rejects_arbitrary_point_payloads():
    import pytest

    from gateway.schemas import InspectionAcceptanceRequest

    with pytest.raises(ValidationError):
        InspectionAcceptanceRequest(
            mode="non_motion",
            points=[{"x": 1.0, "y": 2.0, "z": 0.0, "label": "pump"}],
            client_id="web",
        )


def test_product_field_check_uses_active_map_id(
    monkeypatch,
    tmp_path,
):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ProductFieldCheckRequest, ProductFieldCheckResponse

    map_root = tmp_path / "maps"
    _write_active_same_source_octomap(map_root)
    monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
    gateway = GatewayModule()
    gateway.setup()
    maps_client = _attach_active_maps_service(gateway, map_root)
    monkeypatch.setattr("runtime.endpoints.mapd.MapClient", lambda: maps_client)
    post_field_check = _endpoint(gateway, "/api/v1/diagnostics/field-check")

    result = asyncio.run(
        post_field_check(
            ProductFieldCheckRequest(
                mode="non_motion",
                require_octomap=True,
            )
        )
    )
    model = ProductFieldCheckResponse.model_validate(result)

    assert model.map["active"] == "active"
    assert model.map["artifacts"] == "PASS"
    assert model.map["octomap"] == "PASS"
    assert result["evidence"]["map"]["ok"] is True
    assert result["evidence"]["map"]["map_id"] == "active"
    assert "map_dir" not in result["evidence"]["map"]
    assert result["evidence"]["map"]["artifacts"]["octomap"]["format_ok"] is True
    assert "map artifacts not checked" not in "\n".join(result["advisories"])


@pytest.mark.parametrize("request_type", ["field", "inspection"])
def test_diagnostic_requests_reject_map_directory(request_type):
    from gateway.schemas import InspectionAcceptanceRequest, ProductFieldCheckRequest

    model = ProductFieldCheckRequest if request_type == "field" else InspectionAcceptanceRequest
    with pytest.raises(ValidationError):
        model.model_validate({"map_dir": "injected-map"})




@pytest.mark.parametrize("mode", ["non_motion", "simulation", "field"])
def test_product_field_check_endpoint_is_read_only_and_typed(mode: str):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ProductFieldCheckRequest, ProductFieldCheckResponse

    gateway = GatewayModule()
    gateway.setup()
    post_field_check = _endpoint(gateway, "/api/v1/diagnostics/field-check")

    result = asyncio.run(post_field_check(ProductFieldCheckRequest(mode=mode)))
    model = ProductFieldCheckResponse.model_validate(result)

    assert model.schema_version == "lingtu.product_field_check.v1"
    assert model.mode == mode
    assert isinstance(model.ok, bool)
    assert model.evidence["mode"] == mode
    assert set(model.navigation) == {"can_send_goal", "driver_command"}
    assert gateway.instruction.msg_count == 0
    assert gateway.servo_target.msg_count == 0
    assert gateway.mode_cmd.msg_count == 0






















def test_navigation_goal_requests_are_map_frame_only():
    from gateway.schemas import (
        ClickNavRequest,
        GoalCandidateRequest,
        GoalRequest,
        PlanPreviewRequest,
    )

    assert GoalRequest(x=1.0, y=2.0).frame_id == "map"
    assert ClickNavRequest(x=1.0, y=2.0).frame_id == "map"
    assert PlanPreviewRequest(x=1.0, y=2.0).frame_id == "map"
    assert GoalCandidateRequest(x=1.0, y=2.0).frame_id == "map"
    with pytest.raises(ValidationError):
        GoalRequest(x=1.0, y=2.0, frame_id="odom")
    with pytest.raises(ValidationError):
        ClickNavRequest(x=1.0, y=2.0, frame_id="odom")
    with pytest.raises(ValidationError):
        PlanPreviewRequest(x=1.0, y=2.0, frame_id="odom")
    with pytest.raises(ValidationError):
        GoalCandidateRequest(x=1.0, y=2.0, frame_id="odom")
    with pytest.raises(ValidationError):
        GoalRequest(x=float("nan"), y=2.0)

    from gateway.services.goal_builder import construct_goal_from_request

    clicked = construct_goal_from_request(
        ClickNavRequest(x=1.0, y=2.0),
        default_source="map_click",
        default_target_type="map_point",
    )
    assert clicked.yaw is None
    orientation = clicked.pose_stamped().pose.orientation
    assert (orientation.x, orientation.y, orientation.z, orientation.w) == (0.0, 0.0, 0.0, 0.0)


def test_plan_preview_uses_native_navigation_without_readiness_gate():
    from gateway.schemas import PlanPreviewRequest, PlanPreviewResponse
    from gateway.services.control_commands import ControlCommandService

    calls = []

    class Commands:
        def preview_plan(self, x, y, z):
            calls.append((x, y, z))
            return {
                "start_valid": True,
                "start": {"x": 0.0, "y": 0.0, "z": 0.0},
                "goal": {"x": x, "y": y, "z": z},
                "path": [
                    {"x": 0.0, "y": 0.0, "z": 0.0},
                    {"x": x, "y": y, "z": z},
                ],
                "feasible": True,
                "reason": "",
                "elapsed_ms": 2.5,
                "planner": "native_global",
                "frame_id": "map",
                "timestamp_s": 10.0,
            }

    gateway = SimpleNamespace(_nav_commands=Commands())
    payload = ControlCommandService(gateway).preview_plan(
        PlanPreviewRequest(x=3.0, y=4.0)
    )
    preview = PlanPreviewResponse.model_validate(payload)

    assert calls == [(3.0, 4.0, 0.0)]
    assert preview.feasible is True
    assert preview.count == 2
    assert preview.distance_m == 5.0
    assert preview.source == "native_nav"
    assert preview.reasons == []


def test_plan_preview_explains_missing_native_preview():
    from gateway.schemas import PlanPreviewRequest
    from gateway.services.control_commands import ControlCommandService

    payload = ControlCommandService(SimpleNamespace()).preview_plan(
        PlanPreviewRequest(x=1.0, y=2.0)
    )

    assert payload["ok"] is False
    assert payload["reasons"] == ["native navigation plan preview is unavailable"]
    assert payload["error"] == "native navigation plan preview is unavailable"


def test_plan_preview_preserves_native_failure_reason():
    from gateway.schemas import PlanPreviewRequest
    from gateway.services.control_commands import ControlCommandService

    class Commands:
        def preview_plan(self, x, y, z):
            raise RuntimeError("native planner map is not loaded")

    payload = ControlCommandService(SimpleNamespace(_nav_commands=Commands())).preview_plan(
        PlanPreviewRequest(x=1.0, y=2.0)
    )

    assert payload["ok"] is False
    assert payload["reasons"] == ["native planner map is not loaded"]
    assert payload["error"] == "native planner map is not loaded"


def test_inspection_candidate_preserves_preview_service_failure(monkeypatch):
    from gateway.routes.diagnostics import _inspection_candidate
    from gateway.services.control_commands import ControlCommandService

    monkeypatch.setattr(
        ControlCommandService,
        "preview_plan",
        lambda self, body: {
            "ok": False,
            "feasible": False,
            "reasons": ["preview_unavailable"],
            "error": "native endpoint unavailable",
        },
    )

    candidate = _inspection_candidate(
        SimpleNamespace(),
        {"x": 1.0, "y": 2.0, "z": 0.0, "source": "coordinate", "target_type": "coordinate"},
    )

    assert candidate["ok"] is False
    assert candidate["status"] == "preview_unavailable"
    assert candidate["error"] == "native endpoint unavailable"


def test_saved_map_preview_checks_active_map_then_returns_native_preview(tmp_path):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest

    class Commands:
        def preview_plan(self, x, y, z):
            return {
                "start_valid": False,
                "start": {"x": 0.0, "y": 0.0, "z": 0.0},
                "goal": {"x": x, "y": y, "z": z},
                "path": [],
                "feasible": False,
                "reason": "no_path",
                "elapsed_ms": 1.0,
                "planner": "native_global",
                "frame_id": "map",
                "timestamp_s": 10.0,
            }

    gateway = GatewayModule()
    gateway.setup()
    maps = _attach_active_maps_service(gateway, tmp_path)
    gateway._nav_commands = Commands()

    payload = asyncio.run(
        _endpoint(gateway, "/api/v1/maps/{name}/validate_plan")(
            "active",
            PlanPreviewRequest(x=3.0, y=4.0),
        )
    )

    assert [command["action"] for command in maps.commands] == ["get_active"]
    assert payload["ok"] is True
    assert payload["feasible"] is False
    assert payload["reasons"] == ["no_path"]
    assert payload["map_id"] == "active"
    assert payload["motion_published"] is False


def test_goal_candidate_distinguishes_unavailable_preview():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GoalCandidateRequest

    gateway = GatewayModule()
    gateway.setup()
    payload = asyncio.run(
        _endpoint(gateway, "/api/v1/navigation/goal_candidate")(
            GoalCandidateRequest(x=1.0, y=2.0, preview=True)
        )
    )

    assert payload["ok"] is False
    assert payload["status"] == "preview_unavailable"
    assert payload["preview"]["reasons"] == [
        "native navigation plan preview is unavailable"
    ]
















@pytest.mark.parametrize(
    "rejection",
    [
        pytest.param(False, id="false"),
        pytest.param(True, id="true"),
        pytest.param(None, id="none"),
        pytest.param(object(), id="arbitrary-object"),
        pytest.param(
            {"status": "rejected", "ok": False, "error": "policy_denied"},
            id="ok-false",
        ),
        pytest.param(
            {"status": "rejected", "success": False},
            id="success-false",
        ),
        pytest.param(
            {"status": "rejected", "accepted": False},
            id="accepted-false",
        ),
        pytest.param(
            {"status": "rejected", "ok": True, "accepted": False},
            id="explicit-false-wins",
        ),
        pytest.param({}, id="empty-mapping"),
        pytest.param({"status": "queued"}, id="queued-without-acceptance"),
        pytest.param(
            {"status": "submitted", "stage": "submitted"},
            id="submitted-without-acceptance",
        ),
    ],
)
def test_explicit_action_rejection_is_not_accepted_or_cached(rejection):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, StopRequest

    gateway = GatewayModule()
    body = StopRequest(request_id="rejected-action", client_id="web")
    calls = 0

    def action():
        nonlocal calls
        calls += 1
        if calls == 1:
            return rejection
        return {"accepted": True, "status": "accepted_on_retry"}

    first_response = gateway._run_control_command("test_command", body, action)
    first = GatewayErrorResponse.model_validate(_payload(first_response))
    second = gateway._run_control_command("test_command", body, action)
    third = gateway._run_control_command("test_command", body, action)

    assert first_response.status_code == 409
    assert first.ok is False
    assert first.command is not None
    assert first.command.accepted is False
    assert first.command.replay is False
    if not isinstance(rejection, dict) or not rejection:
        assert first.error == "invalid_command_response"
    if isinstance(rejection, dict) and rejection.get("status") in {"queued", "submitted"}:
        assert first.error == "invalid_command_response"
    assert second["status"] == "accepted_on_retry"
    assert second["command"]["accepted"] is True
    assert second["command"]["replay"] is False
    assert third["command"]["replay"] is True
    assert calls == 2
    stats = gateway._command_journal.snapshot()
    assert stats["accepted_commands"] == 1
    assert stats["stored_requests"] == 1


@pytest.mark.parametrize(
    "accepted_response",
    [
        pytest.param({"ok": True, "status": "ok"}, id="ok-true"),
        pytest.param(
            {"accepted": True, "status": "submitted", "stage": "submitted"},
            id="accepted-true",
        ),
        pytest.param(
            {"success": True, "status": "completed"},
            id="success-true",
        ),
        pytest.param(
            {"command": {"accepted": True}, "status": "acknowledged"},
            id="nested-command-accepted",
        ),
    ],
)
def test_explicit_positive_action_signal_is_accepted_and_replayable(accepted_response):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import StopRequest

    gateway = GatewayModule()
    body = StopRequest(request_id="positive-action", client_id="web")
    calls = 0

    def action():
        nonlocal calls
        calls += 1
        return accepted_response

    first = gateway._run_control_command("test_command", body, action)
    second = gateway._run_control_command("test_command", body, action)

    assert first["command"]["accepted"] is True
    assert first["command"]["replay"] is False
    if first.get("stage") == "submitted":
        assert "success" not in first
    assert second["command"]["accepted"] is True
    assert second["command"]["replay"] is True
    assert calls == 1


def test_malformed_truthy_command_receipt_cannot_claim_ack_success():
    from gateway.services.commands import publish_command_ack, run_control_command

    events: list[dict] = []
    class MalformedJournal:
        def execute(self, *_args, **_kwargs):
            return {
                "ok": "false",
                "command": {
                    "accepted": "false",
                    "replay": "false",
                },
            }

    gateway = SimpleNamespace(
        _command_journal=MalformedJournal(),
        push_event=events.append,
    )
    body = SimpleNamespace(request_id="malformed-receipt", client_id="web")

    direct_payload = {
        "ok": "false",
        "command": {"accepted": "false", "replay": "false"},
    }
    publish_command_ack(gateway, direct_payload, status_code=409)
    response = run_control_command(gateway, "test_command", body, lambda: {"accepted": True})

    assert events[0]["data"]["ok"] is False
    assert events[0]["data"]["accepted"] is False
    assert events[0]["data"]["replay"] is False
    assert response.status_code == 409
    assert _payload(response)["command"]["accepted"] == "false"
















def test_goal_route_uses_persistent_native_client_when_configured(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ControlCommandResponse, GoalRequest

    class FakeClient:
        def __init__(self) -> None:
            self.goals = []

        def send_goal(self, x, y, z, yaw, *, task_id=None, request_id=None):
            self.goals.append((x, y, z, yaw, task_id, request_id))
            return True

    class FakeGoals:
        def __init__(self, commands) -> None:
            self.commands = commands

        def submit_goal(
            self,
            goal,
            *,
            task_id=None,
            request_id=None,
            action="goal",
        ):
            self.commands.send_goal(
                goal.x,
                goal.y,
                goal.z,
                goal.yaw,
                task_id=task_id,
                request_id=request_id,
            )
            return {
                "accepted": True,
                "action": action,
                "task_id": task_id,
                "request_id": request_id,
            }

    client = FakeClient()

    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules(
        {
            "nav.commands": client,
            "nav.goals": FakeGoals(client),
        }
    )
    _mark_navigation_ready(gateway)
    post_goal = _endpoint(gateway, "/api/v1/goal")

    result = asyncio.run(
        post_goal(
            GoalRequest(
                x=1.0,
                y=2.0,
                z=0.3,
                yaw=math.pi / 2,
                instruction="go",
                request_id="native-goal",
                client_id="web",
            )
        )
    )
    model = ControlCommandResponse.model_validate(result)

    assert model.ok is True
    assert "goal_pose" not in gateway.ports_out
    assert gateway.instruction.msg_count == 0
    assert len(client.goals) == 1
    x, y, z, yaw, task_id, request_id = client.goals[0]
    assert (x, y, z, request_id) == (1.0, 2.0, 0.3, "native-goal")
    assert task_id == model.command.task_id
    assert task_id != request_id
    assert yaw == pytest.approx(math.pi / 2)
    assert result["execution_confirmed"] is False


def test_navigation_task_cancel_targets_task_and_only_acknowledges_request():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import CancelRequest, ControlCommandResponse

    class FakeGoals:
        def __init__(self) -> None:
            self.cancels = []

        def submit_cancel(self, reason, *, task_id=None, request_id=None):
            self.cancels.append((task_id, request_id, reason))
            return {
                "accepted": True,
                "state": "cancel_requested",
                "task_id": task_id,
                "request_id": request_id,
                "reason": reason,
            }

    goals = FakeGoals()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.goals": goals})
    post_cancel = _endpoint(
        gateway,
        "/api/v1/navigation/tasks/{task_id}/cancel",
    )

    result = asyncio.run(
        post_cancel(
            "navigation-task-1",
            CancelRequest(
                reason="operator_cancel",
                request_id="cancel-attempt-1",
                client_id="web",
            ),
        )
    )
    model = ControlCommandResponse.model_validate(result)

    assert goals.cancels == [
        ("navigation-task-1", "cancel-attempt-1", "operator_cancel")
    ]
    assert model.command.task_id == "navigation-task-1"
    assert model.command.request_id == "cancel-attempt-1"
    assert result["status"] == "cancel_requested"
    assert result["execution_confirmed"] is False
    assert "cancelled" not in str(result).lower()
    assert gateway.cancel.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0


def test_navigation_task_cancel_rejects_blank_route_task_identity():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import CancelRequest

    class FakeGoals:
        def submit_cancel(self, *_args, **_kwargs):
            raise AssertionError("blank task identity must not reach GoalService")

    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.goals": FakeGoals()})
    post_cancel = _endpoint(
        gateway,
        "/api/v1/navigation/tasks/{task_id}/cancel",
    )

    response = asyncio.run(
        post_cancel(
            "   ",
            CancelRequest(
                reason="operator_cancel",
                request_id="cancel-attempt-blank-task",
                client_id="web",
            ),
        )
    )
    payload = _payload(response)

    assert response.status_code == 409
    assert payload["error"] == "task_identity_invalid"
    assert payload["command"]["accepted"] is False


def test_navigation_task_cancel_rejection_preserves_target_task_identity():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import CancelRequest

    class RejectingGoals:
        def submit_cancel(self, reason, *, task_id=None, request_id=None):
            del reason, request_id
            return {
                "accepted": False,
                "task_id": task_id,
                "message": "task_not_active",
            }

    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.goals": RejectingGoals()})
    post_cancel = _endpoint(
        gateway,
        "/api/v1/navigation/tasks/{task_id}/cancel",
    )

    response = asyncio.run(
        post_cancel(
            "navigation-task-missing",
            CancelRequest(
                reason="operator_cancel",
                request_id="cancel-attempt-missing-task",
                client_id="web",
            ),
        )
    )
    payload = _payload(response)

    assert response.status_code == 409
    assert payload["error"] == "native_command_rejected"
    assert payload["command"]["task_id"] == "navigation-task-missing"
    assert payload["command"]["accepted"] is False
    assert "cancelled" not in str(payload).lower()


def test_endpoint_only_goal_fails_closed_when_native_client_is_missing(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, GoalRequest
    from gateway.services import runtime_status

    status_file = tmp_path / "nav_endpoint_status.json"
    status_file.write_text(
        json.dumps(
            {
                "stamp_s": time.time(),
                "control_mode": "autonomy",
                "control_loop_health": {
                    "ready": True,
                    "healthy": True,
                    "reason": "healthy",
                },
                "input_gate": {"ready": True},
                "publish_cmd_vel": True,
                "control_authority": {
                    "owner": "native_endpoint",
                    "estop_latched": False,
                    "operator_takeover_latched": False,
                    "resume_required": False,
                },
                "active_cmd_source": "autonomy",
                "global_planner": "octoplanner3d",
                "planner_map": "test_map",
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(status_file))
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "60")
    monkeypatch.setattr(
        runtime_status,
        "build_navigation_status",
        lambda _gateway: {
            "can_accept_goal": True,
            "has_odometry": True,
            "readiness": {"blockers": [], "advisories": []},
        },
    )

    class MissingGoals:
        def submit_goal(self, goal, *, task_id=None, request_id=None, action="goal"):
            del goal, task_id, request_id, action
            return {
                "accepted": False,
                "success": False,
                "message": "native navigation command boundary is unavailable",
            }

    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.goals": MissingGoals()})
    _mark_navigation_ready(gateway)
    post_goal = _endpoint(gateway, "/api/v1/goal")

    response = asyncio.run(
        post_goal(
            GoalRequest(
                x=1.0,
                y=2.0,
                z=0.0,
                request_id="endpoint-only-missing",
                client_id="web",
            )
        )
    )
    payload = _payload(response)
    model = GatewayErrorResponse.model_validate(payload)

    assert response.status_code == 409
    assert model.error == "native_command_rejected"
    assert "native navigation command boundary is unavailable" in model.detail["reason"]
    assert "goal_pose" not in gateway.ports_out


def test_goal_route_surfaces_native_business_rejection(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GoalRequest

    class RejectingGoals:
        def submit_goal(self, goal, *, task_id=None, request_id=None, action="goal"):
            del goal, task_id, request_id, action
            return {
                "accepted": False,
                "success": False,
                "message": "navigation command rejected: active_octomap_not_configured",
            }

    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules(
        {
            "nav.goals": RejectingGoals(),
        }
    )
    _mark_navigation_ready(gateway)
    post_goal = _endpoint(gateway, "/api/v1/goal")

    response = asyncio.run(
        post_goal(
            GoalRequest(
                x=1.0,
                y=2.0,
                request_id="native-reject",
                client_id="web",
            )
        )
    )
    payload = _payload(response)

    assert response.status_code == 409
    assert payload["error"] == "native_command_rejected"
    assert payload["command"]["request_id"] == "native-reject"
    assert "active_octomap_not_configured" in payload["detail"]["reason"]


def test_instruction_route_stays_on_semantic_module_path(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import InstructionRequest
    gateway = GatewayModule()
    gateway.setup()
    received = []
    gateway.instruction.subscribe(received.append)
    post_instruction = _endpoint(gateway, "/api/v1/instruction")

    result = asyncio.run(
        post_instruction(
            InstructionRequest(
                text="go to the charging dock",
                request_id="semantic-001",
                client_id="web",
            )
        )
    )

    assert result["status"] == "ok"
    assert received == ["go to the charging dock"]




def test_instruction_rejects_safety_stop_without_publishing():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, InstructionRequest

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._navigation_state = {"authority": "estop", "hold_reason": "operator_estop"}
    post_instruction = _endpoint(gateway, "/api/v1/instruction")

    instruction_response = asyncio.run(
        post_instruction(
            InstructionRequest(
                text="go to dock",
                request_id="safety-stop-instruction",
                client_id="web",
            )
        )
    )
    instruction_model = GatewayErrorResponse.model_validate(_payload(instruction_response))

    assert instruction_response.status_code == 409
    assert instruction_model.error == "safety_stop"
    assert instruction_model.command is not None
    assert instruction_model.command.name == "instruction"
    assert gateway.instruction.msg_count == 0


def test_visual_servo_hot_command_publishes_servo_target():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ControlCommandResponse, VisualServoRequest

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {
        "VisualServoModule": SimpleNamespace(
            can_select_follow_target=lambda: True,
        ),
        "PerceptionModule": SimpleNamespace(
            health=lambda: {"detector_ready": True},
        ),
    }
    targets = []
    gateway.servo_target._add_callback(targets.append)
    post_visual_servo = _endpoint(gateway, "/api/v1/visual_servo")

    result = asyncio.run(
        post_visual_servo(
            VisualServoRequest(
                mode="follow",
                target="person in red",
                request_id="visual-servo-follow",
                client_id="web",
            )
        )
    )

    model = ControlCommandResponse.model_validate(result)
    assert model.command.name == "visual_servo"
    assert targets == ["follow:person in red"]
    assert gateway.servo_target.msg_count == 1


def test_visual_follow_accepts_an_explicit_person_track_without_a_description_selector():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ControlCommandResponse, VisualServoRequest

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {
        "VisualServoModule": SimpleNamespace(
            can_select_follow_target=lambda: False,
        ),
        "PerceptionModule": SimpleNamespace(
            health=lambda: {"detector_ready": True},
        ),
    }
    targets = []
    gateway.servo_target._add_callback(targets.append)
    post_visual_servo = _endpoint(gateway, "/api/v1/visual_servo")

    result = asyncio.run(
        post_visual_servo(
            VisualServoRequest(
                mode="follow",
                target_id="person-7",
                request_id="visual-servo-follow-id",
                client_id="web",
            )
        )
    )

    model = ControlCommandResponse.model_validate(result)
    assert model.command.name == "visual_servo"
    assert targets == ["follow_id:person-7"]
    assert gateway.servo_target.msg_count == 1


def test_gateway_explicit_person_track_is_consumed_by_visual_servo():
    from decision.modules.visual_servo import VisualServoModule
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import VisualServoRequest
    from runtime.msgs.geometry import Vector3
    from runtime.msgs.semantic import Detection3D

    visual_servo = VisualServoModule()
    visual_servo.setup()
    visual_servo.detections_3d._deliver(
        [
            Detection3D(
                id="person-7",
                label="person",
                confidence=0.9,
                position=Vector3(4.0, 0.0, 1.2),
                bbox_2d=[0.0, 0.0, 30.0, 30.0],
                ts=20.0,
            )
        ]
    )

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {
        "VisualServoModule": visual_servo,
        "PerceptionModule": SimpleNamespace(
            health=lambda: {"detector_ready": True},
        ),
    }
    gateway.servo_target._add_callback(visual_servo._on_servo_target)
    post_visual_servo = _endpoint(gateway, "/api/v1/visual_servo")

    result = asyncio.run(
        post_visual_servo(
            VisualServoRequest(
                mode="follow",
                target_id="person-7",
                request_id="visual-servo-follow-id-integration",
                client_id="web",
            )
        )
    )

    assert result["ok"] is True
    status = visual_servo.get_servo_status()
    assert status["mode"] == "follow"
    assert status["person"]["id"] == "person-7"
    assert status["state"] == "following"


def test_visual_follow_rejects_when_target_selection_is_unavailable():
    from decision.modules.visual_servo import VisualServoModule
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, VisualServoRequest

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {
        "VisualServoModule": VisualServoModule(),
        "PerceptionModule": SimpleNamespace(
            health=lambda: {"detector_ready": True},
        ),
    }
    post_visual_servo = _endpoint(gateway, "/api/v1/visual_servo")

    response = asyncio.run(
        post_visual_servo(
            VisualServoRequest(
                mode="follow",
                target="person in red",
                request_id="visual-follow-no-selector",
                client_id="web",
            )
        )
    )

    model = GatewayErrorResponse.model_validate(_payload(response))
    assert response.status_code == 409
    assert model.error == "target_selection_unavailable"
    assert gateway.servo_target.msg_count == 0


def test_visual_servo_stop_allowed_under_safety_stop_but_find_rejected():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, VisualServoRequest

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {
        "VisualServoModule": object(),
        "PerceptionModule": SimpleNamespace(
            health=lambda: {"detector_ready": True},
        ),
    }
    targets = []
    gateway.servo_target._add_callback(targets.append)
    with gateway._state_lock:
        gateway._navigation_state = {"authority": "estop", "hold_reason": "operator_estop"}
    post_visual_servo = _endpoint(gateway, "/api/v1/visual_servo")

    stop_result = asyncio.run(
        post_visual_servo(
            VisualServoRequest(
                mode="stop",
                request_id="visual-servo-stop",
                client_id="web",
            )
        )
    )
    find_response = asyncio.run(
        post_visual_servo(
            VisualServoRequest(
                mode="find",
                target="dock",
                request_id="visual-servo-find",
                client_id="web",
            )
        )
    )

    find_model = GatewayErrorResponse.model_validate(_payload(find_response))
    assert stop_result["ok"] is True
    assert find_response.status_code == 409
    assert find_model.error == "safety_stop"
    assert targets == ["stop"]
    assert gateway.servo_target.msg_count == 1


def test_visual_servo_hot_command_rejects_when_module_unavailable():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, VisualServoRequest

    gateway = GatewayModule()
    gateway.setup()
    post_visual_servo = _endpoint(gateway, "/api/v1/visual_servo")

    response = asyncio.run(
        post_visual_servo(
            VisualServoRequest(
                mode="find",
                target="dock",
                request_id="visual-servo-unavailable",
                client_id="web",
            )
        )
    )

    model = GatewayErrorResponse.model_validate(_payload(response))
    assert response.status_code == 409
    assert model.error == "visual_servo_unavailable"
    assert gateway.servo_target.msg_count == 0


def test_visual_servo_status_is_forwarded_to_sse():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    events = []
    gateway.push_event = events.append

    gateway.visual_servo_status._deliver(
        {
            "mode": "follow",
            "target": "person in red",
            "follow_available": True,
            "state": "following",
            "navigation_state": "path_active",
            "goal_rate_hz": 1.0,
        }
    )

    assert gateway._visual_servo_status["mode"] == "follow"
    assert events == [
        {
            "type": "visual_servo_status",
            "data": gateway._visual_servo_status,
        }
    ]


def test_field_stop_reports_missing_native_boundary_without_local_cmd_vel():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, StopRequest

    gateway = GatewayModule()
    gateway.setup()
    gateway._nav_commands = None
    post_stop = _endpoint(gateway, "/api/v1/stop")

    response = asyncio.run(
        post_stop(
            StopRequest(
                request_id="native-stop-missing",
                client_id="web",
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "native_command_rejected"
    assert "cmd_vel" not in gateway.ports_out


def test_field_emergency_stop_uses_native_estop_latch(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.routes import commands
    from gateway.schemas import ControlCommandResponse, StopRequest

    calls = []
    monkeypatch.setattr(
        commands,
        "native_estop",
        lambda _gw, reason="estop", *, request_id=None: calls.append((reason, request_id)) or True,
    )
    gateway = GatewayModule()
    gateway.setup()
    post_stop = _endpoint(gateway, "/api/v1/stop")

    result = asyncio.run(post_stop(StopRequest(request_id="native-stop", client_id="web")))
    model = ControlCommandResponse.model_validate(result)

    assert model.ok is True
    assert model.status == "stopped"
    assert calls == [("rest_emergency_stop", "native-stop")]
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0


def test_native_stop_ack_does_not_block_gateway_event_loop(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.routes import commands
    from gateway.schemas import StopRequest

    entered = threading.Event()
    release = threading.Event()
    ack_threads: list[int] = []

    def blocking_estop(_gw, reason="estop", *, request_id=None):
        ack_threads.append(threading.get_ident())
        entered.set()
        release.wait(timeout=1.0)
        return True

    monkeypatch.setattr(commands, "native_estop", blocking_estop)
    gateway = GatewayModule()
    gateway.setup()
    post_stop = _endpoint(gateway, "/api/v1/stop")

    async def run_request():
        loop_thread = threading.get_ident()
        started_at = time.perf_counter()
        request_task = asyncio.create_task(
            post_stop(
                StopRequest(
                    request_id="nonblocking-stop",
                    client_id="web",
                )
            )
        )
        await asyncio.sleep(0.02)
        heartbeat_delay = time.perf_counter() - started_at
        release.set()
        response = await request_task
        return loop_thread, heartbeat_delay, response

    loop_thread, heartbeat_delay, response = asyncio.run(run_request())

    assert entered.is_set()
    assert heartbeat_delay < 0.15
    assert len(ack_threads) == 1
    assert ack_threads[0] != loop_thread
    assert response["status"] == "stopped"


def test_mode_estop_and_explicit_reset_use_native_boundary(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.routes import commands
    from gateway.schemas import ControlCommandResponse, ModeRequest, StopRequest

    calls = []
    monkeypatch.setattr(
        commands,
        "native_estop",
        lambda _gw, reason="estop", *, request_id=None: calls.append(("estop", reason, request_id)) or True,
    )
    monkeypatch.setattr(
        commands,
        "native_clear_estop",
        lambda _gw, reason="clear_estop", *, request_id=None: calls.append(("clear", reason, request_id)) or True,
    )
    gateway = GatewayModule()
    gateway.setup()
    post_mode = _endpoint(gateway, "/api/v1/mode")
    post_reset = _endpoint(gateway, "/api/v1/estop/reset")

    estop_result = asyncio.run(post_mode(ModeRequest(mode="estop", request_id="estop-1", client_id="web")))
    reset_result = asyncio.run(post_reset(StopRequest(request_id="reset-1", client_id="web")))

    assert ControlCommandResponse.model_validate(estop_result).mode == "estop"
    assert ControlCommandResponse.model_validate(reset_result).status == "estop_cleared"
    assert calls == [
        ("estop", "mode_estop", "estop-1"),
        ("clear", "operator_reset", "reset-1"),
    ]
    assert "cmd_vel" not in gateway.ports_out
    assert "stop_cmd" not in gateway.ports_out


def test_field_estop_mode_failure_does_not_claim_mode_change(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.routes import commands
    from gateway.schemas import GatewayErrorResponse, ModeRequest

    monkeypatch.setattr(
        commands,
        "native_estop",
        lambda *args, **kwargs: False,
    )
    gateway = GatewayModule()
    gateway.setup()
    post_mode = _endpoint(gateway, "/api/v1/mode")

    response = asyncio.run(post_mode(ModeRequest(mode="estop", request_id="estop-fail", client_id="web")))
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "native_command_rejected"
    assert gateway._mode != "estop"


def test_navigation_cancel_publishes_cancel_without_motion_outputs():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import CancelRequest, ControlCommandResponse

    gateway = GatewayModule()
    gateway.setup()
    cancel_msgs: list[str] = []
    gateway.cancel.subscribe(cancel_msgs.append)
    post_cancel = _endpoint(gateway, "/api/v1/navigation/cancel")

    result = asyncio.run(
        post_cancel(
            CancelRequest(
                reason="operator_cancel",
                request_id="cancel-001",
                client_id="web",
            )
        )
    )
    model = ControlCommandResponse.model_validate(result)

    assert model.ok is True
    assert model.status == "cancel_requested"
    assert model.reason == "operator_cancel"
    assert model.command.name == "navigation_cancel"
    assert model.command.request_id == "cancel-001"
    assert gateway.cancel.msg_count == 1
    assert cancel_msgs == ["operator_cancel"]
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0


def test_navigation_resume_releases_takeover_without_replaying_old_motion(
    monkeypatch,
):
    from gateway.gateway_module import GatewayModule
    from gateway.routes import commands
    from gateway.schemas import ControlCommandResponse, StopRequest

    calls: list[tuple[str, str | None]] = []
    monkeypatch.setattr(
        commands,
        "native_resume_control",
        lambda _gw, reason, *, request_id=None: calls.append((reason, request_id)) or True,
    )
    monkeypatch.setattr(
        commands,
        "native_motion_resume_context",
        lambda: {
            "status_fresh": True,
            "observed_control_mode": "autonomy",
            "resume_was_required": True,
            "goal_reissue_required": True,
            "fresh_operator_command_required": False,
        },
    )
    gateway = GatewayModule()
    gateway.setup()
    post_resume = _endpoint(gateway, "/api/v1/navigation/resume")

    result = asyncio.run(post_resume(StopRequest(request_id="resume-001", client_id="web")))
    model = ControlCommandResponse.model_validate(result)

    assert model.ok is True
    assert model.status == "motion_resume_acknowledged"
    assert result["observed_control_mode"] == "autonomy"
    assert result["goal_reissue_required"] is True
    assert result["fresh_operator_command_required"] is False
    assert result["previous_motion_restored"] is False
    assert calls == [("operator_resume", "resume-001")]
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0


def test_navigation_resume_in_teleop_avoid_requires_fresh_operator_command(
    monkeypatch,
):
    from gateway.gateway_module import GatewayModule
    from gateway.routes import commands
    from gateway.schemas import StopRequest

    monkeypatch.setattr(commands, "native_resume_control", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(
        commands,
        "native_motion_resume_context",
        lambda: {
            "status_fresh": True,
            "observed_control_mode": "teleop_avoid",
            "resume_was_required": True,
            "goal_reissue_required": False,
            "fresh_operator_command_required": True,
        },
    )
    gateway = GatewayModule()
    gateway.setup()

    result = asyncio.run(
        _endpoint(gateway, "/api/v1/navigation/resume")(
            StopRequest(request_id="resume-teleop", client_id="web")
        )
    )

    assert result["status"] == "motion_resume_acknowledged"
    assert result["observed_control_mode"] == "teleop_avoid"
    assert result["goal_reissue_required"] is False
    assert result["fresh_operator_command_required"] is True
    assert result["previous_motion_restored"] is False


def test_navigation_resume_with_unknown_status_does_not_guess_goal_semantics(
    monkeypatch,
):
    from gateway.gateway_module import GatewayModule
    from gateway.routes import commands

    monkeypatch.setattr(commands, "native_resume_control", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(
        commands,
        "native_motion_resume_context",
        lambda: {
            "status_fresh": False,
            "observed_control_mode": None,
            "resume_was_required": None,
            "goal_reissue_required": None,
            "fresh_operator_command_required": None,
        },
    )
    gateway = GatewayModule()
    gateway.setup()

    result = asyncio.run(_endpoint(gateway, "/api/v1/navigation/resume")())

    assert result["observed_control_mode"] is None
    assert result["goal_reissue_required"] is None
    assert result["fresh_operator_command_required"] is None


def test_navigation_resume_uses_authoritative_native_reason(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.routes import commands
    from runtime.msgs import NavigationCommandKind

    monkeypatch.setattr(
        commands,
        "native_resume_control",
        lambda *_args, **_kwargs: {
            "accepted": True,
            "kind": int(NavigationCommandKind.RESUME_AUTONOMY),
            "task_id": "",
            "request_id": "resume-receipt",
            "reason": "autonomy_already_ready",
            "endpoint_timestamp_s": 123.5,
        },
    )
    monkeypatch.setattr(
        commands,
        "native_motion_resume_context",
        lambda: {
            "status_fresh": False,
            "observed_control_mode": None,
            "resume_was_required": None,
            "goal_reissue_required": None,
            "fresh_operator_command_required": None,
        },
    )
    gateway = GatewayModule()
    gateway.setup()

    result = asyncio.run(_endpoint(gateway, "/api/v1/navigation/resume")())

    assert result["native_reason"] == "autonomy_already_ready"
    assert result["resume_was_required"] is False
    assert result["goal_reissue_required"] is False
    assert result["fresh_operator_command_required"] is False


def test_navigation_resume_rejects_missing_local_resume_implementation(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.routes import commands

    monkeypatch.setattr(commands, "native_resume_control", lambda *_args, **_kwargs: False)
    gateway = GatewayModule()
    gateway.setup()

    response = _payload(
        asyncio.run(_endpoint(gateway, "/api/v1/navigation/resume")())
    )

    assert response["ok"] is False
    assert response["error"] == "native_command_rejected"
    assert "unavailable" in response["message"].lower()


def test_active_control_lease_blocks_other_rest_resume_clients(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.routes import commands
    from gateway.schemas import StopRequest

    calls = []
    monkeypatch.setattr(
        commands,
        "native_resume_control",
        lambda _gw, reason, *, request_id=None: calls.append((reason, request_id)) or True,
    )
    gateway = GatewayModule()
    gateway.setup()
    assert gateway._lease.acquire("operator-a", 30.0) is True
    post_resume = _endpoint(gateway, "/api/v1/navigation/resume")

    response = _payload(
        asyncio.run(
            post_resume(
                StopRequest(
                    request_id="blocked-resume",
                    client_id="operator-b",
                )
            )
        )
    )

    assert response["ok"] is False
    assert response["error"] == "control_lease"
    assert response["detail"]["lease"]["holder"] == "operator-a"
    assert calls == []


def test_commands_without_request_id_preserve_existing_execute_every_time_behavior():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ControlCommandResponse

    gateway = GatewayModule()
    gateway.setup()
    post_stop = _endpoint(gateway, "/api/v1/stop")

    first = asyncio.run(post_stop())
    second = asyncio.run(post_stop())
    model = ControlCommandResponse.model_validate(first)

    assert gateway.stop_cmd.msg_count == 2
    assert gateway.cmd_vel.msg_count == 2
    assert model.schema_version == 1
    assert model.ok is True
    assert model.status == "stopped"
    assert first["status"] == "stopped"
    assert first["command"]["request_id"] is None
    assert second["command"]["replay"] is False


def test_lease_command_uses_receipt_and_replays_duplicate_request_id():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LeaseRequest, LeaseResponse

    gateway = GatewayModule()
    gateway.setup()
    post_lease = _endpoint(gateway, "/api/v1/lease")
    queue = subscribe(gateway)

    try:
        acquire = LeaseRequest(
            action="acquire",
            client_id="web",
            ttl=30.0,
            request_id="lease-001",
        )
        first = asyncio.run(post_lease(acquire))
        second = asyncio.run(post_lease(acquire))
        release = asyncio.run(
            post_lease(
                LeaseRequest(
                    action="release",
                    client_id="web",
                    ttl=30.0,
                    request_id="lease-release-001",
                )
            )
        )
        events = []
        while not queue.empty():
            events.append(queue.get_nowait())
    finally:
        unsubscribe(gateway, queue)

    acquired = LeaseResponse.model_validate(first)
    replayed = LeaseResponse.model_validate(second)
    released = LeaseResponse.model_validate(release)
    command_stats = gateway._command_journal.snapshot()

    assert acquired.schema_version == 1
    assert acquired.ok is True
    assert acquired.status == "acquired"
    assert acquired.holder == "web"
    assert acquired.active is True
    assert acquired.command.name == "lease"
    assert acquired.command.request_id == "lease-001"
    assert acquired.command.client_id == "web"
    assert acquired.command.replay is False
    assert replayed.command.replay is True
    assert replayed.holder == "web"
    assert released.status == "released"
    assert released.active is False
    assert released.command.request_id == "lease-release-001"
    assert command_stats["accepted_commands"] == 2
    assert command_stats["replayed_commands"] == 1
    lease_events = [event for event in events if event["type"] == "lease"]
    ack_events = [event for event in events if event["type"] == "command_ack"]
    assert [event["data"]["status"] for event in lease_events] == ["acquired", "released"]
    assert ack_events[0]["data"]["command"]["name"] == "lease"


def test_lease_conflict_emits_rejected_ack_and_lease_event():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LeaseRequest

    gateway = GatewayModule()
    gateway.setup()
    post_lease = _endpoint(gateway, "/api/v1/lease")
    queue = subscribe(gateway)

    try:
        first = asyncio.run(
            post_lease(
                LeaseRequest(
                    action="acquire",
                    client_id="web",
                    ttl=30.0,
                    request_id="lease-web",
                )
            )
        )
        conflict = _payload(
            asyncio.run(
                post_lease(
                    LeaseRequest(
                        action="acquire",
                        client_id="mobile",
                        ttl=30.0,
                        request_id="lease-mobile",
                    )
                )
            )
        )
        events = []
        while not queue.empty():
            events.append(queue.get_nowait())
    finally:
        unsubscribe(gateway, queue)

    lease_events = [event for event in events if event["type"] == "lease"]
    ack_events = [event for event in events if event["type"] == "command_ack"]

    assert first["ok"] is True
    assert conflict["ok"] is False
    assert conflict["error"] == "lease_conflict"
    assert conflict["command"]["accepted"] is False
    assert conflict["detail"]["reason_code"] == "lease_conflict"
    assert conflict["detail"]["source"] == "control_lease"
    assert conflict["detail"]["path"] == "/api/v1/lease"
    assert conflict["detail"]["lease"]["holder"] == "web"
    assert [event["data"]["status"] for event in lease_events] == [
        "acquired",
        "rejected",
    ]
    assert ack_events[-1]["data"]["ok"] is False
    assert ack_events[-1]["data"]["status_code"] == 409
    assert ack_events[-1]["data"]["command"]["client_id"] == "mobile"
    assert ack_events[-1]["data"]["detail"]["reason_code"] == "lease_conflict"


def test_non_holder_cannot_release_another_clients_control_lease():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LeaseRequest

    gateway = GatewayModule()
    gateway.setup()
    post_lease = _endpoint(gateway, "/api/v1/lease")

    acquired = asyncio.run(
        post_lease(
            LeaseRequest(
                action="acquire",
                client_id="operator-a",
                ttl=30.0,
                request_id="lease-owner",
            )
        )
    )
    rejected = asyncio.run(
        post_lease(
            LeaseRequest(
                action="release",
                client_id="operator-b",
                ttl=30.0,
                request_id="lease-intruder",
            )
        )
    )
    payload = _payload(rejected)

    assert acquired["status"] == "acquired"
    assert rejected.status_code == 403
    assert payload["ok"] is False
    assert payload["error"] == "not_lease_holder"
    assert payload["detail"]["lease"]["holder"] == "operator-a"
    assert gateway._lease.to_dict()["holder"] == "operator-a"
    assert gateway._lease.to_dict()["active"] is True


def test_bootstrap_and_health_expose_command_policy():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_bootstrap

    gateway = GatewayModule()

    bootstrap = build_app_bootstrap(gateway)
    health = gateway.health()

    policy = bootstrap["control"]["command_policy"]
    assert policy["idempotency_supported"] is True
    assert policy["request_id_field"] == "request_id"
    assert policy["client_id_field"] == "client_id"
    assert policy["acceptance_signal_policy"] == "explicit_positive_required"
    assert policy["rate_policy_hz"]["cmd_vel"] == 20.0
    assert health["gateway"]["commands"]["rate_policy_enforcement"] == "advisory"
    assert policy["rate_policy_hz"]["cmd_vel"] == 20.0
    assert health["gateway"]["commands"]["rate_policy_enforcement"] == "advisory"

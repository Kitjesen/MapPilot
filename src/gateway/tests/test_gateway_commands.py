from __future__ import annotations

import asyncio
import hashlib
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
    map_sha = hashlib.sha256(map_path.read_bytes()).hexdigest()
    octomap_sha = hashlib.sha256(octomap_path.read_bytes()).hexdigest()
    (active_dir / "metadata.json").write_text(
        json.dumps(
            {
                "schema_version": "lingtu.saved_map_artifacts.v1",
                "source_profile": "thunder_field",
                "data_source": "thunder_field",
                "slam_source": "fastlio2",
                "localization_source": "fastlio2",
                "mapping_source": "fastlio2",
                "frame_id": "map",
                "created_at": "2026-05-25T00:00:00Z",
                "artifacts": {
                    "map_pcd": {
                        "path": "map.pcd",
                        "sha256": map_sha,
                        "source_profile": "thunder_field",
                        "data_source": "thunder_field",
                        "slam_source": "fastlio2",
                        "frame_id": "map",
                        "point_count": 1,
                    },
                    "octomap": {
                        "path": "octomap.ot",
                        "sha256": octomap_sha,
                        "source_map_sha256": map_sha,
                        "source_profile": "thunder_field",
                        "data_source": "thunder_field",
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

    def execute(self, request):
        command = request.to_mapping()
        action = command["action"]
        if action == "get_active":
            return {"action": action, "success": True, "active": "active"}
        if action == "validate_artifacts":
            from maps.artifacts import validate_saved_map_artifact_dir

            gate = validate_saved_map_artifact_dir(
                self.map_root / "active",
                require_octomap=bool(command.get("require_octomap", False)),
                require_occupancy=bool(command.get("require_occupancy", False)),
                expected_data_source=command.get("expected_data_source"),
                expected_source_profile=command.get("expected_source_profile"),
                expected_frame_id=command.get("expected_frame_id"),
            )
            return {"action": action, "success": True, "gate": gate}
        return {
            "action": action,
            "success": False,
            "reason_code": "unsupported_test_action",
        }


def _attach_active_maps_service(gateway, map_root: Path) -> _TypedActiveMapsService:
    service = _TypedActiveMapsService(map_root)
    gateway._map_mgr = service
    gateway._all_modules = {"maps.service": service}
    return service


def _mark_navigation_ready(gateway) -> None:
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "z": 0.0, "ts": time.time()}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "odom_age_ms": 100.0,
            "localizer_health": "RECOVERED",
        }


class _FakePlanPreviewNav:
    def __init__(
        self,
        *,
        ok: bool = True,
        feasible: bool = True,
        reasons: list[str] | None = None,
        plan_safety_policy: str | None = None,
        path_safety: dict | None = None,
    ) -> None:
        self.calls: list[tuple[float, float, float]] = []
        self.ok = ok
        self.feasible = feasible
        self.reasons = list(reasons or [])
        self.plan_safety_policy = plan_safety_policy
        self.path_safety = path_safety

    def preview_plan(self, x: float, y: float, z: float) -> dict:
        self.calls.append((x, y, z))
        ts = time.time()
        if not self.feasible:
            return {
                "schema_version": 1,
                "ok": self.ok,
                "feasible": False,
                "frame_id": "map",
                "start": {"x": 0.0, "y": 0.0, "z": 0.0, "frame_id": "map", "ts": ts},
                "goal": {"x": x, "y": y, "z": z, "frame_id": "map", "ts": ts},
                "adjusted_goal": None,
                "path": [],
                "count": 0,
                "distance_m": None,
                "plan_ms": 0.5,
                "planner": "fake",
                "selected_planner": "fake",
                "plan_safety_policy": self.plan_safety_policy,
                "path_safety": self.path_safety,
                "fallback_reason": "",
                "rejected_plans": [],
                "source": "navigation_preview",
                "reasons": self.reasons or ["blocked_by_costmap"],
                "error": None,
                "ts": ts,
            }
        return {
            "schema_version": 1,
            "ok": self.ok,
            "feasible": True,
            "frame_id": "map",
            "start": {"x": 0.0, "y": 0.0, "z": 0.0, "frame_id": "map", "ts": ts},
            "goal": {"x": x, "y": y, "z": z, "frame_id": "map", "ts": ts},
            "adjusted_goal": None,
            "path": [
                {"x": 0.0, "y": 0.0, "z": 0.0, "frame_id": "map", "ts": ts},
                {"x": x, "y": y, "z": z, "frame_id": "map", "ts": ts},
            ],
            "count": 2,
            "distance_m": 1.0,
            "plan_ms": 0.5,
            "planner": "fake",
            "selected_planner": "fake",
            "plan_safety_policy": self.plan_safety_policy,
            "path_safety": self.path_safety,
            "fallback_reason": "",
            "rejected_plans": [],
            "source": "navigation_preview",
            "reasons": [],
            "error": None,
            "ts": ts,
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


def _write_algorithm_benchmark_summary(root: Path) -> Path:
    from gateway.routes.diagnostics import DIMOS_BENCHMARK_REQUIRED_GATES

    root.mkdir(parents=True, exist_ok=True)
    path = root / "summary_dimos_benchmark_sim_acceptance.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.server_sim_closure.summary.v1",
                "ok": True,
                "missing_or_failed": [],
                "algorithm_validation": {
                    "claim_allowed": True,
                    "required_gate_sequence": list(DIMOS_BENCHMARK_REQUIRED_GATES),
                    "validation_flow": [{"gate": "dynamic_obstacle_local_planner", "ok": True}],
                    "claim_boundary": {
                        "simulation_only": True,
                        "global_planning_source": "static_saved_map_octomap",
                        "live_costmap_role": "local_planning_and_safety_only",
                    },
                    "blocking_categories": {},
                },
            }
        ),
        encoding="utf-8",
    )
    return path


def test_readiness_request_defaults_to_server_simulation_mode():
    from gateway.schemas import InspectionAcceptanceRequest, ProductFieldCheckRequest

    assert ProductFieldCheckRequest().mode == "simulation"
    assert InspectionAcceptanceRequest().mode == "simulation"


def test_server_sim_acceptance_chain_reads_algorithm_artifact_without_motion_publish(
    monkeypatch,
    tmp_path,
):
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    artifact_root = tmp_path / "server_sim_closure"
    summary_path = _write_algorithm_benchmark_summary(artifact_root)
    monkeypatch.setenv("LINGTU_ALGORITHM_BENCHMARK_ROOT", str(artifact_root))
    monkeypatch.setenv("LINGTU_ALGORITHM_BENCHMARK_MAX_AGE_SEC", "1000")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "mujoco_fastlio2_live")
    monkeypatch.setenv("LINGTU_ENDPOINT", "mujoco_live")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "mujoco_fastlio2_live")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "1")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "mujoco_velocity_adapter")

    gateway = GatewayModule()
    gateway.setup()
    _install_saved_location(gateway, name="pump")
    client = TestClient(gateway._app)

    algorithm = client.get("/api/v1/diagnostics/algorithm-benchmark/latest")
    field_check = client.post("/api/v1/diagnostics/field-check", json={})
    acceptance = client.post(
        "/api/v1/inspection/acceptance",
        json={
            "points": ["pump"],
            "client_id": "server-sim-test",
        },
    )

    assert algorithm.status_code == 200
    algorithm_body = algorithm.json()
    assert algorithm_body["ok"] is True
    assert algorithm_body["summary_path"] == str(summary_path)
    assert algorithm_body["read_only"] is True
    assert algorithm_body["ros2_topic_required"] is False
    assert algorithm_body["publishes"] == []

    assert field_check.status_code == 200
    field_body = field_check.json()
    strict = field_body["algorithm"]["strict_benchmark"]
    assert field_body["mode"] == "simulation"
    assert strict["status"] == "PASS"
    assert strict["summary_path"] == algorithm_body["summary_path"]
    assert strict["source"] == algorithm_body["source"]
    assert strict["read_only"] is True
    assert strict["ros2_topic_required"] is False
    assert strict["publishes"] == []

    assert acceptance.status_code == 200
    acceptance_body = acceptance.json()
    assert acceptance_body["mode"] == "simulation"
    assert (
        acceptance_body["evidence"]["field_check"]["algorithm"]["strict_benchmark"]["summary_path"]
        == algorithm_body["summary_path"]
    )
    assert all(target["command_published"] is False for target in acceptance_body["targets"])
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0
    assert gateway.instruction.msg_count == 0


def test_navigation_plan_preview_is_non_motion_and_typed():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest, PlanPreviewResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    post_plan = _endpoint(gateway, "/api/v1/navigation/plan")

    result = asyncio.run(
        post_plan(
            PlanPreviewRequest(
                x=1.0,
                y=2.0,
                z=0.0,
                client_id="web",
            )
        )
    )
    model = PlanPreviewResponse.model_validate(result)

    assert nav.calls == [(1.0, 2.0, 0.0)]
    assert model.schema_version == 1
    assert model.ok is True
    assert model.feasible is True
    assert model.path[-1].x == 1.0
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0


def test_map_plan_preview_can_ignore_inactive_navigation_session():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest
    from gateway.services.control_commands import ControlCommandService

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    gateway._session_mode = "idle"
    with gateway._state_lock:
        gateway._safety = {"level": 2}
    service = ControlCommandService(gateway)

    blocked = service.preview_navigation_plan(
        PlanPreviewRequest(x=1.0, y=0.0, z=0.0),
    )
    preview = service.preview_navigation_plan(
        PlanPreviewRequest(x=1.0, y=0.0, z=0.0),
        ignore_blockers={"navigation_session_inactive", "safety_stop"},
    )

    assert nav.calls == [(1.0, 0.0, 0.0)]
    assert blocked["feasible"] is False
    assert "navigation_session_inactive" in blocked["reasons"]
    assert "safety_stop" in blocked["reasons"]
    assert preview["feasible"] is True
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0


def test_navigation_plan_preview_does_not_publish_any_control_outputs():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    post_plan = _endpoint(gateway, "/api/v1/navigation/plan")

    asyncio.run(
        post_plan(
            PlanPreviewRequest(
                x=1.0,
                y=2.0,
                z=0.0,
                client_id="web",
            )
        )
    )

    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.instruction.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0
    assert gateway.cancel.msg_count == 0
    assert gateway.mode_cmd.msg_count == 0


def test_navigation_goal_candidate_constructs_coordinate_without_publishing():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GoalCandidateRequest, GoalCandidateResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    post_candidate = _endpoint(gateway, "/api/v1/navigation/goal_candidate")

    result = asyncio.run(
        post_candidate(
            GoalCandidateRequest(
                x=1.0,
                y=2.0,
                z=0.0,
                yaw=0.25,
                label="dock approach",
                acceptance_radius_m=0.6,
                max_speed_mps=0.3,
                client_id="web",
            )
        )
    )
    model = GoalCandidateResponse.model_validate(result)

    assert model.schema_version == 1
    assert model.ok is True
    assert model.status == "preview_feasible"
    assert model.target is not None
    assert model.target.x == 1.0
    assert model.target.y == 2.0
    assert model.target.yaw == 0.25
    assert model.target.frame_id == "map"
    assert model.target.source == "coordinate"
    assert model.target.target_type == "coordinate"
    assert model.target.label == "dock approach"
    assert model.target.acceptance_radius_m == 0.6
    assert model.target.max_speed_mps == 0.3
    assert model.preview is not None
    assert model.preview.feasible is True
    assert nav.calls == [(1.0, 2.0, 0.0)]
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0


def test_navigation_goal_candidate_previews_frontier_target_without_publishing():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GoalCandidateRequest, GoalCandidateResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    post_candidate = _endpoint(gateway, "/api/v1/navigation/goal_candidate")

    result = asyncio.run(
        post_candidate(
            GoalCandidateRequest(
                x=2.0,
                y=3.0,
                z=0.0,
                source="frontier",
                target_type="frontier",
                label="traversable frontier candidate",
                metadata={"candidate_id": "traversable_frontier_0"},
                client_id="web",
            )
        )
    )
    model = GoalCandidateResponse.model_validate(result)

    assert model.ok is True
    assert model.status == "preview_feasible"
    assert model.target is not None
    assert model.target.x == 2.0
    assert model.target.y == 3.0
    assert model.target.source == "frontier"
    assert model.target.target_type == "frontier"
    assert model.preview is not None
    assert model.preview.feasible is True
    assert nav.calls == [(2.0, 3.0, 0.0)]
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.instruction.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0
    assert gateway.cancel.msg_count == 0
    assert gateway.mode_cmd.msg_count == 0


def test_navigation_goal_candidate_constructs_saved_location_without_publishing():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GoalCandidateRequest, GoalCandidateResponse

    class Store:
        def query(self, name: str):
            if name == "dock":
                return {
                    "name": "dock",
                    "position": [5.0, 6.0, 0.1],
                    "yaw": 0.75,
                }
            return None

        def query_fuzzy(self, name: str):
            return None

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    gateway._tagged_loc_module = SimpleNamespace(store=Store())
    _mark_navigation_ready(gateway)
    post_candidate = _endpoint(gateway, "/api/v1/navigation/goal_candidate")

    result = asyncio.run(
        post_candidate(
            GoalCandidateRequest(
                location_name="dock",
                client_id="mobile",
            )
        )
    )
    model = GoalCandidateResponse.model_validate(result)

    assert model.ok is True
    assert model.status == "preview_feasible"
    assert model.target is not None
    assert model.target.x == 5.0
    assert model.target.y == 6.0
    assert model.target.z == 0.1
    assert model.target.yaw == 0.75
    assert model.target.source == "saved_location"
    assert model.target.target_type == "saved_location"
    assert model.target.location_name == "dock"
    assert model.target.label == "dock"
    assert model.preview is not None
    assert model.preview.goal.x == 5.0
    assert nav.calls == [(5.0, 6.0, 0.1)]
    assert gateway.goal_pose.msg_count == 0
    assert gateway.instruction.msg_count == 0


def test_inspection_acceptance_previews_saved_location_without_publishing():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import (
        InspectionAcceptanceRequest,
        InspectionAcceptanceResponse,
    )

    class Store:
        def __init__(self):
            self.entry = {
                "name": "dock",
                "position": [5.0, 6.0, 0.1],
                "yaw": 0.75,
                "tags": ["inspection"],
            }

        def list_all(self):
            return [self.entry]

        def query(self, name: str):
            return self.entry if name == "dock" else None

        def query_fuzzy(self, name: str):
            return None

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    gateway._tagged_loc_module = SimpleNamespace(store=Store())
    _mark_navigation_ready(gateway)
    post_acceptance = _endpoint(gateway, "/api/v1/inspection/acceptance")

    result = asyncio.run(
        post_acceptance(
            InspectionAcceptanceRequest(
                mode="non_motion",
                points=["dock"],
                client_id="web",
            )
        )
    )
    model = InspectionAcceptanceResponse.model_validate(result)

    assert model.schema_version == "lingtu.inspection_acceptance.v1"
    assert model.mode == "non_motion"
    assert model.target_count == 1
    assert model.targets[0].name == "dock"
    assert model.targets[0].status == "PASS"
    assert model.targets[0].preview_feasible is True
    assert model.targets[0].command_published is False
    assert nav.calls == [(5.0, 6.0, 0.1)]
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0
    assert gateway.instruction.msg_count == 0


def test_inspection_acceptance_request_rejects_arbitrary_point_payloads():
    import pytest

    from gateway.schemas import InspectionAcceptanceRequest

    with pytest.raises(ValidationError):
        InspectionAcceptanceRequest(
            mode="non_motion",
            points=[{"x": 1.0, "y": 2.0, "z": 0.0, "label": "pump"}],
            client_id="web",
        )


def test_product_field_check_uses_active_map_when_map_dir_is_omitted(
    monkeypatch,
    tmp_path,
):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ProductFieldCheckRequest, ProductFieldCheckResponse

    map_root = tmp_path / "maps"
    active_dir = _write_active_same_source_octomap(map_root)
    monkeypatch.setenv("NAV_MAP_DIR", str(map_root))

    gateway = GatewayModule()
    gateway.setup()
    _attach_active_maps_service(gateway, map_root)
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

    assert model.map["active"] == str(active_dir)
    assert model.map["provenance"] == "PASS"
    assert model.map["octomap"] == "PASS"
    assert result["evidence"]["map"]["ok"] is True
    assert result["evidence"]["map"]["map_dir"] == str(active_dir)
    assert result["evidence"]["map"]["artifacts"]["octomap"]["sha256_ok"] is True
    assert "map provenance not checked" not in "\n".join(result["advisories"])


def test_inspection_acceptance_uses_active_map_when_map_dir_is_omitted(
    monkeypatch,
    tmp_path,
):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import (
        InspectionAcceptanceRequest,
        InspectionAcceptanceResponse,
    )

    map_root = tmp_path / "maps"
    active_dir = _write_active_same_source_octomap(map_root)
    monkeypatch.setenv("NAV_MAP_DIR", str(map_root))

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    maps_service = _attach_active_maps_service(gateway, map_root)
    gateway.on_system_modules({"nav.mission": nav, "maps.service": maps_service})
    _install_saved_location(gateway, name="pump")
    _mark_navigation_ready(gateway)
    post_acceptance = _endpoint(gateway, "/api/v1/inspection/acceptance")

    result = asyncio.run(
        post_acceptance(
            InspectionAcceptanceRequest(
                mode="non_motion",
                points=["pump"],
                require_octomap=True,
                client_id="web",
            )
        )
    )
    model = InspectionAcceptanceResponse.model_validate(result)

    assert model.evidence["field_check"]["map"]["active"] == str(active_dir)
    assert model.evidence["field_check"]["map"]["provenance"] == "PASS"
    assert model.evidence["field_check"]["map"]["octomap"] == "PASS"
    assert result["targets"][0]["command_published"] is False
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0


@pytest.mark.parametrize("mode", ["field", "simulation"])
def test_inspection_acceptance_field_and_simulation_modes_do_not_publish(mode: str):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import (
        InspectionAcceptanceRequest,
        InspectionAcceptanceResponse,
    )

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    _install_saved_location(gateway, name="pump")
    _mark_navigation_ready(gateway)
    post_acceptance = _endpoint(gateway, "/api/v1/inspection/acceptance")

    result = asyncio.run(
        post_acceptance(
            InspectionAcceptanceRequest(
                mode=mode,
                points=["pump"],
                client_id="web",
            )
        )
    )
    model = InspectionAcceptanceResponse.model_validate(result)

    assert model.mode == mode
    assert model.target_count == 1
    assert model.targets[0].command_published is False
    assert nav.calls == [(1.0, 2.0, 0.0)]
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0
    assert gateway.instruction.msg_count == 0


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
    assert model.algorithm["strict_benchmark"]["read_only"] is True
    assert model.algorithm["strict_benchmark"]["ros2_topic_required"] is False
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0
    assert gateway.instruction.msg_count == 0


def test_runtime_switch_plan_endpoint_is_read_only_and_typed():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeSwitchPlanRequest, RuntimeSwitchPlanResponse

    gateway = GatewayModule()
    gateway.setup()
    post_switch_plan = _endpoint(gateway, "/api/v1/runtime/switch-plan")

    result = asyncio.run(
        post_switch_plan(
            RuntimeSwitchPlanRequest(
                current_profile="sim_mujoco_live",
                target_profile="explore",
            )
        )
    )
    model = RuntimeSwitchPlanResponse.model_validate(result)

    assert model.schema_version == "lingtu.runtime_switch_plan.v1"
    assert model.ok is True
    assert model.read_only is True
    assert model.motion is False
    assert model.publishes == []
    assert model.from_["runtime_contract"] == "mujoco_fastlio2_live"
    assert model.to["runtime_contract"] == "thunder_field"
    assert model.from_["command_sink"] == "mujoco_velocity_adapter"
    assert model.to["command_sink"] == "driver"
    assert "command_sink" in model.changed
    assert "simulation_only" in model.changed
    assert "resolved_runtime_data_flow" in model.changed
    assert model.current_validation.ok is True
    assert model.target_validation.ok is True
    assert {
        "dynamic_obstacle_gate",
        "command_boundary",
    } <= {str(stage.get("name")) for stage in model.from_["resolved_runtime_data_flow"]}
    assert {
        "dynamic_obstacle_gate",
        "command_boundary",
    } <= {str(stage.get("name")) for stage in model.to["resolved_runtime_data_flow"]}
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0
    assert gateway.instruction.msg_count == 0


def test_runtime_switch_plan_exposes_resolved_endpoint_processes():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeSwitchPlanRequest, RuntimeSwitchPlanResponse

    gateway = GatewayModule()
    gateway.setup()
    post_switch_plan = _endpoint(gateway, "/api/v1/runtime/switch-plan")

    result = asyncio.run(
        post_switch_plan(
            RuntimeSwitchPlanRequest(
                current_profile="teleop",
                target_profile="nav",
                target_endpoint="thunder_field",
            )
        )
    )
    model = RuntimeSwitchPlanResponse.model_validate(result)

    assert model.ok is True
    assert model.product_mode_switch is not None
    runtime_plan = model.product_mode_switch["runtime_plan"]
    assert runtime_plan["endpoint"] == "thunder_field"
    assert [process["name"] for process in runtime_plan["processes"]] == [
        "lidar",
        "slam",
        "traversability",
        "nav",
        "driver",
        "runtime",
    ]
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0


def test_runtime_switch_plan_inherits_current_env_endpoint_when_profile_matches(
    monkeypatch,
):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeSwitchPlanRequest, RuntimeSwitchPlanResponse

    monkeypatch.setenv("LINGTU_PROFILE", "nav")
    monkeypatch.setenv("LINGTU_ENDPOINT", "thunder_field")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "thunder_field")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "driver")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "0")

    gateway = GatewayModule()
    gateway.setup()
    post_switch_plan = _endpoint(gateway, "/api/v1/runtime/switch-plan")

    result = asyncio.run(
        post_switch_plan(
            RuntimeSwitchPlanRequest(
                current_profile="nav",
                target_profile="explore",
            )
        )
    )
    model = RuntimeSwitchPlanResponse.model_validate(result)

    assert model.ok is True
    assert model.read_only is True
    assert model.motion is False
    assert model.publishes == []
    assert model.inputs["current_endpoint_source"] == "env"
    assert model.from_["profile"] == "nav"
    assert model.from_["endpoint"] == "thunder_field"
    assert model.from_["data_source"] == "thunder_field"
    assert model.from_["command_sink"] == "driver"
    assert model.to["endpoint"] == "thunder_field"
    assert model.to["command_sink"] == "driver"
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0
    assert gateway.instruction.msg_count == 0


def test_runtime_switch_plan_endpoint_reports_invalid_current_boundary(monkeypatch):
    import gateway.services.runtime_switch_plan as switch_plan_mod
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeSwitchPlanRequest, RuntimeSwitchPlanResponse
    from runtime.runtime_switch import RuntimeSwitchValidation

    original_validate = switch_plan_mod.validate_runtime_switch

    def fake_validate(spec):
        if spec.profile == "sim_mujoco_live":
            return RuntimeSwitchValidation(
                ok=False,
                blockers=("forced current blocker",),
            )
        return original_validate(spec)

    monkeypatch.setattr(switch_plan_mod, "validate_runtime_switch", fake_validate)

    gateway = GatewayModule()
    gateway.setup()
    post_switch_plan = _endpoint(gateway, "/api/v1/runtime/switch-plan")

    result = asyncio.run(
        post_switch_plan(
            RuntimeSwitchPlanRequest(
                current_profile="sim_mujoco_live",
                target_profile="explore",
            )
        )
    )
    model = RuntimeSwitchPlanResponse.model_validate(result)

    assert model.ok is False
    assert model.current_validation.ok is False
    assert "current runtime boundary: forced current blocker" in model.blockers
    assert model.from_["runtime_contract"] == "mujoco_fastlio2_live"
    assert model.to["runtime_contract"] == "thunder_field"
    assert model.changed
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0
    assert gateway.instruction.msg_count == 0


def test_runtime_switch_endpoint_defaults_to_plan_only_for_app_clients():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeSwitchRequest, RuntimeSwitchResponse

    gateway = GatewayModule()
    gateway.setup()
    post_runtime_switch = _endpoint(gateway, "/api/v1/runtime/switch")

    result = asyncio.run(
        post_runtime_switch(
            RuntimeSwitchRequest(
                current_profile="nav",
                target_profile="inspection",
                map_name="field_map",
            )
        )
    )
    model = RuntimeSwitchResponse.model_validate(result)

    assert model.schema_version == "lingtu.runtime_switch.v1"
    assert model.ok is True
    assert model.accepted is False
    assert model.read_only is True
    assert model.dry_run is True
    assert model.motion is False
    assert model.status == "planned"
    assert model.lifecycle == "cold_restart"
    assert model.strategy == "auto"
    assert model.product_mode_switch is not None
    assert model.product_mode_switch["required_lifecycle"] == "cold_restart"
    assert model.effects == []
    assert model.target_profile == "inspection"
    assert model.command[:3] == ["bash", model.command[1], "mode"]
    assert model.command[3:5] == ["switch", "inspection"]
    assert "--map" in model.command
    assert "/api/v1/mode" not in " ".join(model.command)
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0
    assert gateway.instruction.msg_count == 0


def test_runtime_switch_endpoint_rejects_hot_when_graph_requires_restart():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeSwitchRequest, RuntimeSwitchResponse

    gateway = GatewayModule()
    gateway.setup()
    gateway._session_mode = "navigating"
    gateway._session_map = "field_map"
    post_runtime_switch = _endpoint(gateway, "/api/v1/runtime/switch")

    result = asyncio.run(
        post_runtime_switch(
            RuntimeSwitchRequest(
                current_profile="nav",
                target_profile="map",
                execute=True,
                strategy="hot",
            )
        )
    )
    model = RuntimeSwitchResponse.model_validate(result)

    assert model.ok is False
    assert model.accepted is False
    assert model.status == "rejected"
    assert model.lifecycle == "cold_restart"
    assert model.command
    assert gateway.cancel.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0


def test_runtime_switch_allows_map_free_teleop_avoid_plan():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeSwitchRequest, RuntimeSwitchResponse

    gateway = GatewayModule()
    gateway.setup()
    post_runtime_switch = _endpoint(gateway, "/api/v1/runtime/switch")

    result = asyncio.run(
        post_runtime_switch(
            RuntimeSwitchRequest(
                current_profile="nav",
                target_profile="teleop_avoid",
            )
        )
    )
    model = RuntimeSwitchResponse.model_validate(result)

    assert model.ok is True
    assert model.status == "planned"
    assert model.map_name is None
    assert model.lifecycle == "cold_restart"


def test_runtime_switch_endpoint_accepts_tare_explore_product_mode():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeSwitchRequest, RuntimeSwitchResponse

    gateway = GatewayModule()
    gateway.setup()
    post_runtime_switch = _endpoint(gateway, "/api/v1/runtime/switch")

    result = asyncio.run(
        post_runtime_switch(
            RuntimeSwitchRequest(
                current_profile="nav",
                target_profile="tare_explore",
                request_id="tare-explore-switch-test",
            )
        )
    )
    model = RuntimeSwitchResponse.model_validate(result)

    assert model.ok is True
    assert model.accepted is False
    assert model.status == "planned"
    assert model.lifecycle == "cold_restart"
    assert model.product_mode_switch is not None
    assert model.product_mode_switch["target"]["profile"] == "tare_explore"
    assert model.command[:3] == ["bash", model.command[1], "mode"]
    assert model.command[3:5] == ["switch", "tare_explore"]
    assert "--map" not in model.command
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0


def test_navigation_plan_preview_runs_outside_the_event_loop_thread():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    worker_threads: list[int] = []
    original_preview = nav.preview_plan

    def preview_plan(*args, **kwargs):
        worker_threads.append(threading.get_ident())
        return original_preview(*args, **kwargs)

    nav.preview_plan = preview_plan
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    post_plan = _endpoint(gateway, "/api/v1/navigation/plan")
    event_loop_thread = threading.get_ident()

    asyncio.run(post_plan(PlanPreviewRequest(x=1.0, y=2.0, z=0.0)))

    assert worker_threads
    assert worker_threads[0] != event_loop_thread
    assert gateway.stop_cmd.msg_count == 0


def test_runtime_switch_endpoint_can_launch_robot_side_mode_switch(
    monkeypatch,
    tmp_path,
):
    import gateway.services.runtime_switch_execute as switch_execute
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeSwitchRequest, RuntimeSwitchResponse

    calls = []

    class FakePopen:
        pid = 4321

        def __init__(self, command, **kwargs):
            calls.append((command, kwargs))

    monkeypatch.setenv("LINGTU_RUNTIME_SWITCH_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(switch_execute, "_requires_transient_unit", lambda: False)
    monkeypatch.setattr(
        switch_execute,
        "build_runtime_switch_plan",
        lambda raw: {
            "ok": True,
            "blockers": [],
            "inputs": {"current_profile": raw.get("current_profile")},
            "product_mode_switch": {"required_lifecycle": "cold_restart"},
        },
    )
    monkeypatch.setattr(switch_execute.subprocess, "Popen", FakePopen)

    gateway = GatewayModule()
    gateway.setup()
    post_runtime_switch = _endpoint(gateway, "/api/v1/runtime/switch")

    result = asyncio.run(
        post_runtime_switch(
            RuntimeSwitchRequest(
                current_profile="nav",
                target_profile="nav",
                map_name="field_map",
                allow_restart=True,
                request_id="switch-test",
            )
        )
    )
    model = RuntimeSwitchResponse.model_validate(result)

    assert model.ok is True
    assert model.accepted is True
    assert model.read_only is False
    assert model.dry_run is False
    assert model.status == "accepted"
    assert model.pid == 4321
    assert model.command_id == "switch-test"
    assert calls
    command = calls[0][0]
    assert command[0] == "bash"
    assert command[2:5] == ["mode", "switch", "nav"]
    assert "--map" in command
    assert "field_map" in command
    assert "--relocalize" in command
    assert calls[0][1]["stdin"] is switch_execute.subprocess.DEVNULL
    assert calls[0][1]["close_fds"] is (switch_execute.os.name != "nt")
    if switch_execute.os.name != "nt":
        assert calls[0][1]["start_new_session"] is True
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cancel.msg_count == 1
    assert gateway.cmd_vel.msg_count == 1
    assert gateway.stop_cmd.msg_count == 1
    assert gateway.instruction.msg_count == 0
    assert gateway._runtime_switch_pending is not None

    duplicate = asyncio.run(
        post_runtime_switch(
            RuntimeSwitchRequest(
                current_profile="nav",
                target_profile="nav",
                map_name="other_field_map",
                allow_restart=True,
                request_id="switch-test-duplicate",
            )
        )
    )
    duplicate_model = RuntimeSwitchResponse.model_validate(duplicate)

    assert duplicate_model.ok is False
    assert duplicate_model.status == "rejected"
    assert duplicate_model.blockers == ["runtime switch already in progress: switch-test"]
    assert len(calls) == 1


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


def test_navigation_plan_preview_degrades_without_odometry_and_does_not_plan():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest, PlanPreviewResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    post_plan = _endpoint(gateway, "/api/v1/navigation/plan")

    result = asyncio.run(post_plan(PlanPreviewRequest(x=1.0, y=2.0)))
    model = PlanPreviewResponse.model_validate(result)

    assert nav.calls == []
    assert model.ok is True
    assert model.feasible is False
    assert "odometry_missing" in model.reasons
    assert model.goal.x == 1.0
    assert model.path == []
    assert gateway.goal_pose.msg_count == 0


def test_navigation_plan_preview_omits_invalid_start_when_unavailable():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest, PlanPreviewResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    gateway._mode = "estop"
    with gateway._state_lock:
        gateway._odom = {"x": "bad", "y": 0.0, "z": 0.0, "ts": time.time()}
    post_plan = _endpoint(gateway, "/api/v1/navigation/plan")

    result = asyncio.run(post_plan(PlanPreviewRequest(x=1.0, y=2.0)))
    model = PlanPreviewResponse.model_validate(result)

    assert nav.calls == []
    assert model.feasible is False
    assert model.start is None
    assert "estop_active" in model.reasons
    assert gateway.goal_pose.msg_count == 0


def test_navigation_plan_preview_preserves_non_map_start_frame_when_blocked():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest, PlanPreviewResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    gateway._mode = "estop"
    with gateway._state_lock:
        gateway._odom = {
            "x": 1.0,
            "y": 2.0,
            "z": 0.0,
            "frame_id": "odom",
            "ts": time.time(),
        }
    post_plan = _endpoint(gateway, "/api/v1/navigation/plan")

    result = asyncio.run(post_plan(PlanPreviewRequest(x=3.0, y=4.0)))
    model = PlanPreviewResponse.model_validate(result)

    assert nav.calls == []
    assert model.feasible is False
    assert model.frame_id == "map"
    assert model.start is not None
    assert model.start.frame_id == "odom"
    assert "estop_active" in model.reasons
    assert gateway.goal_pose.msg_count == 0


def test_command_journal_replays_duplicate_request_id_without_republish():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ControlCommandResponse, GoalRequest

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    post_goal = _endpoint(gateway, "/api/v1/goal")

    body = GoalRequest(
        x=1.0,
        y=2.0,
        z=0.0,
        instruction="dock",
        request_id="goal-001",
        client_id="web",
    )

    first = asyncio.run(post_goal(body))
    second = asyncio.run(post_goal(body))
    model = ControlCommandResponse.model_validate(first)

    assert gateway.goal_pose.msg_count == 1
    assert gateway.instruction.msg_count == 0
    assert nav.calls == [(1.0, 2.0, 0.0)]
    assert model.schema_version == 1
    assert model.ok is True
    assert model.status == "accepted"
    assert model.goal == [1.0, 2.0, 0.0]
    assert model.command.name == "goal"
    assert model.command.request_id == "goal-001"
    assert model.command.client_id == "web"
    assert first["command"]["accepted"] is True
    assert first["command"]["replay"] is False
    assert second["command"]["replay"] is True
    assert second["goal"] == [1.0, 2.0, 0.0]
    assert second["command"]["request_id"] == "goal-001"


def test_runtime_switch_cold_restart_uses_independent_systemd_unit(
    monkeypatch,
    tmp_path,
):
    import gateway.services.runtime_switch_execute as switch_execute
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeSwitchRequest, RuntimeSwitchResponse

    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return type(
            "Result",
            (),
            {"returncode": 0, "stdout": "", "stderr": ""},
        )()

    def fail_popen(*_args, **_kwargs):
        raise AssertionError("cold switch must escape lingtu.service cgroup")

    monkeypatch.setenv("LINGTU_RUNTIME_SWITCH_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(
        switch_execute,
        "_requires_transient_unit",
        lambda: True,
        raising=False,
    )
    monkeypatch.setattr(
        switch_execute,
        "build_runtime_switch_plan",
        lambda raw: {
            "ok": True,
            "blockers": [],
            "inputs": {"current_profile": raw.get("current_profile")},
            "product_mode_switch": {"required_lifecycle": "cold_restart"},
        },
    )
    monkeypatch.setattr(switch_execute.subprocess, "run", fake_run)
    monkeypatch.setattr(switch_execute.subprocess, "Popen", fail_popen)

    stale_log = tmp_path / "systemd-switch-test.log"
    stale_log.write_text("stale", encoding="utf-8")

    gateway = GatewayModule()
    gateway.setup()
    result = asyncio.run(
        _endpoint(gateway, "/api/v1/runtime/switch")(
            RuntimeSwitchRequest(
                current_profile="map",
                target_profile="nav",
                map_name="field_map",
                allow_restart=True,
                request_id="systemd-switch-test",
            )
        )
    )
    model = RuntimeSwitchResponse.model_validate(result)

    assert model.ok is True
    assert model.accepted is True
    assert len(calls) == 2
    assert calls[0][0] == ["sudo", "-n", "true"]
    launch = calls[1][0]
    assert launch[:3] == ["sudo", "-n", "systemd-run"]
    assert "--collect" in launch
    assert "--no-block" in launch
    assert "--unit=lingtu-runtime-switch-systemd-switch-test" in launch
    assert "bash" in launch
    assert "mode" in launch
    assert "switch" in launch
    assert "nav" in launch
    assert "HOME=/home/sunrise" in launch
    assert "USER=sunrise" in launch
    assert "LOGNAME=sunrise" in launch
    assert not stale_log.exists()


def test_runtime_switch_linux_dev_process_does_not_require_transient_unit(
    monkeypatch,
):
    import gateway.services.runtime_switch_execute as switch_execute

    monkeypatch.setattr(switch_execute.os, "name", "posix")
    monkeypatch.delenv("INVOCATION_ID", raising=False)
    monkeypatch.delenv("SYSTEMD_EXEC_PID", raising=False)
    monkeypatch.delenv("JOURNAL_STREAM", raising=False)

    assert switch_execute._requires_transient_unit() is False


def test_runtime_switch_linux_systemd_process_requires_transient_unit(
    monkeypatch,
):
    import gateway.services.runtime_switch_execute as switch_execute

    monkeypatch.setattr(switch_execute.os, "name", "posix")
    monkeypatch.setenv("INVOCATION_ID", "systemd-invocation")
    monkeypatch.delenv("SYSTEMD_EXEC_PID", raising=False)

    assert switch_execute._requires_transient_unit() is True


def test_runtime_switch_log_path_cannot_escape_configured_directory(
    monkeypatch,
    tmp_path,
):
    import gateway.services.runtime_switch_execute as switch_execute

    monkeypatch.setenv("LINGTU_RUNTIME_SWITCH_LOG_DIR", str(tmp_path))

    log_path = switch_execute._log_path("../../outside/runtime-switch")

    assert log_path.parent == tmp_path
    assert log_path.name == "outside-runtime-switch.log"


def test_runtime_switch_transient_unit_name_is_unique_and_sanitized():
    import gateway.services.runtime_switch_execute as switch_execute

    first = switch_execute._transient_unit_name("switch/one")
    second = switch_execute._transient_unit_name("switch two")

    assert first == "lingtu-runtime-switch-switch-one"
    assert second == "lingtu-runtime-switch-switch-two"
    assert first != second


def test_runtime_switch_requires_native_stop_ack_and_clears_pending_on_failure(
    monkeypatch,
    tmp_path,
):
    import gateway.services.native_control as native_control
    import gateway.services.runtime_switch_execute as switch_execute
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeSwitchRequest, RuntimeSwitchResponse

    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setenv("LINGTU_RUNTIME_SWITCH_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(
        native_control,
        "stop",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError("native stop ack timeout")),
    )
    monkeypatch.setattr(
        switch_execute.subprocess,
        "Popen",
        lambda *_args, **_kwargs: pytest.fail("switch must not launch without stop ack"),
    )

    gateway = GatewayModule()
    gateway.setup()
    result = asyncio.run(
        _endpoint(gateway, "/api/v1/runtime/switch")(
            RuntimeSwitchRequest(
                current_profile="nav",
                target_profile="nav",
                map_name="field_map",
                allow_restart=True,
                request_id="stop-ack-failure",
            )
        )
    )
    model = RuntimeSwitchResponse.model_validate(result)

    assert model.ok is False
    assert model.status == "error"
    assert "native stop ack timeout" in (model.error or "")
    assert getattr(gateway, "_runtime_switch_pending", None) is None
    assert gateway.cancel.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0


def test_control_commands_publish_command_ack_events():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GoalRequest

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    post_goal = _endpoint(gateway, "/api/v1/goal")
    queue = gateway._sse_subscribe()

    try:
        body = GoalRequest(
            x=1.0,
            y=2.0,
            z=0.0,
            request_id="goal-ack-001",
            client_id="web",
        )
        first = asyncio.run(post_goal(body))
        second = asyncio.run(post_goal(body))

        first_event = queue.get_nowait()
        second_event = queue.get_nowait()
    finally:
        gateway._sse_unsubscribe(queue)

    assert first["command"]["replay"] is False
    assert second["command"]["replay"] is True
    assert first_event["type"] == "command_ack"
    assert first_event["data"]["ok"] is True
    assert first_event["data"]["status"] == "accepted"
    assert first_event["data"]["status_code"] == 200
    assert first_event["data"]["command"]["name"] == "goal"
    assert first_event["data"]["command"]["request_id"] == "goal-ack-001"
    assert first_event["data"]["command"]["replay"] is False
    assert second_event["type"] == "command_ack"
    assert second_event["data"]["command"]["replay"] is True


def test_goal_request_yaw_is_published_as_pose_orientation():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ControlCommandResponse, GoalRequest

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    sent_goals = []
    gateway.goal_pose._add_callback(sent_goals.append)
    post_goal = _endpoint(gateway, "/api/v1/goal")

    result = asyncio.run(
        post_goal(
            GoalRequest(
                x=1.0,
                y=2.0,
                z=0.0,
                yaw=math.pi / 2,
                client_id="script",
            )
        )
    )
    model = ControlCommandResponse.model_validate(result)

    assert gateway.goal_pose.msg_count == 1
    assert nav.calls == [(1.0, 2.0, 0.0)]
    assert len(sent_goals) == 1
    assert sent_goals[0].pose.orientation.yaw == pytest.approx(math.pi / 2)
    assert model.goal == [1.0, 2.0, 0.0]
    assert model.yaw == pytest.approx(math.pi / 2)
    assert model.frame_id == "map"
    assert model.target is not None
    assert model.target.source == "coordinate"
    assert model.target.target_type == "coordinate"
    assert model.target.yaw == pytest.approx(math.pi / 2)
    assert result["yaw"] == pytest.approx(math.pi / 2)
    assert result["frame_id"] == "map"
    assert result["target"]["source"] == "coordinate"


def test_goal_route_uses_persistent_native_client_when_configured(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ControlCommandResponse, GoalRequest

    class FakeClient:
        def __init__(self) -> None:
            self.goals = []

        def send_goal(self, x, y, z, yaw, *, request_id=None) -> bool:
            self.goals.append((x, y, z, yaw, request_id))
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
            self.commands.send_goal(goal.x, goal.y, goal.z, goal.yaw, request_id=request_id)
            resolved_task_id = task_id or "task-generated-by-fake-goals"
            return {
                "accepted": True,
                "success": True,
                "action": action,
                "task_id": resolved_task_id,
                "request_id": request_id,
                "native_task_id": resolved_task_id,
                "native_request_id": request_id,
                "native_ack": {
                    "accepted": True,
                    "task_id": resolved_task_id,
                    "request_id": request_id,
                },
                "sink": "native_dds",
            }

    client = FakeClient()

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules(
        {
            "nav.mission": nav,
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
    assert gateway.goal_pose.msg_count == 0
    assert gateway.instruction.msg_count == 0
    assert len(client.goals) == 1
    x, y, z, yaw, request_id = client.goals[0]
    assert (x, y, z, request_id) == (1.0, 2.0, 0.3, "native-goal")
    assert yaw == pytest.approx(math.pi / 2)


def test_endpoint_only_goal_fails_closed_when_native_client_is_missing(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, GoalRequest

    status_file = tmp_path / "nav_endpoint_status.json"
    status_file.write_text(
        json.dumps(
            {
                "stamp_s": time.time(),
                "control_mode": "autonomy",
                "input_gate": {"ready": True},
                "publish_cmd_vel": True,
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(status_file))
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "60")

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
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav, "nav.goals": MissingGoals()})
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
    assert gateway.goal_pose.msg_count == 0


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
            "nav.mission": _FakePlanPreviewNav(),
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


def test_goal_route_rejects_infeasible_plan_preview_without_publishing():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, GoalRequest

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav(feasible=False, reasons=["blocked_by_costmap"])
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    sent_goals = []
    gateway.goal_pose._add_callback(sent_goals.append)
    post_goal = _endpoint(gateway, "/api/v1/goal")
    queue = gateway._sse_subscribe()

    try:
        response = asyncio.run(
            post_goal(
                GoalRequest(
                    x=1.0,
                    y=2.0,
                    z=0.0,
                    request_id="blocked-goal",
                    client_id="web",
                )
            )
        )
        event = queue.get_nowait()
    finally:
        gateway._sse_unsubscribe(queue)
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.ok is False
    assert model.error == "navigation_plan_infeasible"
    assert model.command is not None
    assert model.command.name == "goal"
    assert model.command.request_id == "blocked-goal"
    assert model.command.accepted is False
    assert model.detail["reason_code"] == "navigation_plan_infeasible"
    assert model.detail["blockers"] == ["blocked_by_costmap"]
    assert model.detail["source"] == "navigation_preview"
    assert model.detail["path"] == "/api/v1/navigation/plan"
    assert model.detail["preview"]["reasons"] == ["blocked_by_costmap"]
    assert event["type"] == "command_ack"
    assert event["data"]["ok"] is False
    assert event["data"]["error"] == "navigation_plan_infeasible"
    assert event["data"]["command"]["accepted"] is False
    assert event["data"]["command"]["request_id"] == "blocked-goal"
    assert event["data"]["detail"]["reason_code"] == "navigation_plan_infeasible"
    assert event["data"]["detail"]["path"] == "/api/v1/navigation/plan"
    assert nav.calls == [(1.0, 2.0, 0.0)]
    assert sent_goals == []
    assert gateway.goal_pose.msg_count == 0


@pytest.mark.parametrize(
    "nav",
    [
        _FakePlanPreviewNav(ok=False, feasible=True),
        _FakePlanPreviewNav(
            feasible=True,
            plan_safety_policy="reject",
            path_safety={"ok": False, "blocked_sample_count": 3},
        ),
    ],
)
def test_goal_route_rejects_inconsistent_or_unsafe_feasible_preview(nav):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, GoalRequest

    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    sent_goals = []
    gateway.goal_pose._add_callback(sent_goals.append)
    post_goal = _endpoint(gateway, "/api/v1/goal")

    response = asyncio.run(
        post_goal(
            GoalRequest(
                x=1.0,
                y=2.0,
                z=0.0,
                request_id="unsafe-feasible-goal",
                client_id="web",
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "navigation_plan_infeasible"
    assert model.detail["preview"]["feasible"] is True
    assert nav.calls == [(1.0, 2.0, 0.0)]
    assert sent_goals == []
    assert gateway.goal_pose.msg_count == 0


def test_goal_route_rejects_infeasible_preview_with_instruction_without_any_publish():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, GoalRequest

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav(feasible=False, reasons=["blocked_by_costmap"])
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    post_goal = _endpoint(gateway, "/api/v1/goal")

    response = asyncio.run(
        post_goal(
            GoalRequest(
                x=1.0,
                y=2.0,
                z=0.0,
                instruction="dock",
                request_id="blocked-goal-with-instruction",
                client_id="web",
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "navigation_plan_infeasible"
    assert model.detail["preview"]["reasons"] == ["blocked_by_costmap"]
    assert nav.calls == [(1.0, 2.0, 0.0)]
    assert gateway.goal_pose.msg_count == 0
    assert gateway.instruction.msg_count == 0


def test_goal_route_rejects_safety_stop_without_planning_or_publishing():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, GoalRequest

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    with gateway._state_lock:
        gateway._safety = {"level": 2}
    post_goal = _endpoint(gateway, "/api/v1/goal")

    response = asyncio.run(
        post_goal(
            GoalRequest(
                x=1.0,
                y=2.0,
                z=0.0,
                request_id="safety-stop-goal",
                client_id="web",
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "safety_stop"
    assert model.command is not None
    assert model.command.name == "goal"
    assert model.command.accepted is False
    assert model.detail["reason_code"] == "safety_stop"
    assert model.detail["blockers"] == ["safety_stop"]
    assert model.detail["source"] == "safety"
    assert model.detail["safety"]["stop_active"] is True
    assert nav.calls == []
    assert gateway.goal_pose.msg_count == 0


def test_goal_route_rejects_inactive_navigation_session_without_planning_or_publishing():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, GoalRequest

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    gateway._session_mode = "idle"
    post_goal = _endpoint(gateway, "/api/v1/goal")

    response = asyncio.run(
        post_goal(
            GoalRequest(
                x=1.0,
                y=2.0,
                z=0.0,
                request_id="inactive-session-goal",
                client_id="web",
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "navigation_not_ready"
    assert model.command is not None
    assert model.command.name == "goal"
    assert model.command.accepted is False
    assert model.detail["reason_code"] == "navigation_not_ready"
    assert model.detail["source"] == "gateway_readiness"
    assert "navigation_session_inactive" in model.detail["blockers"]
    assert nav.calls == []
    assert gateway.goal_pose.msg_count == 0


def test_click_navigation_rejects_infeasible_plan_preview_without_publishing():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ClickNavRequest, GatewayErrorResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav(feasible=False, reasons=["blocked_by_costmap"])
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    sent_goals = []
    gateway.goal_pose._add_callback(sent_goals.append)
    post_click = _endpoint(gateway, "/api/v1/navigate/click")

    response = asyncio.run(
        post_click(
            ClickNavRequest(
                x=3.0,
                y=4.0,
                z=0.0,
                request_id="blocked-click",
                client_id="web",
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "navigation_plan_infeasible"
    assert model.command is not None
    assert model.command.name == "navigate_click"
    assert model.command.request_id == "blocked-click"
    assert model.command.accepted is False
    assert model.detail["preview"]["reasons"] == ["blocked_by_costmap"]
    assert nav.calls == [(3.0, 4.0, 0.0)]
    assert sent_goals == []
    assert gateway.goal_pose.msg_count == 0


def test_click_navigation_rejects_selected_map_that_is_not_active():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ClickNavRequest, GatewayErrorResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    gateway._session_map = "active_map"
    post_click = _endpoint(gateway, "/api/v1/navigate/click")

    response = asyncio.run(
        post_click(
            ClickNavRequest(
                x=3.0,
                y=4.0,
                z=0.0,
                request_id="wrong-map-click",
                client_id="web",
                metadata={"map_name": "previewed_other_map"},
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "active_map_mismatch"
    assert model.detail["reason_code"] == "active_map_mismatch"
    assert model.detail["requested_map"] == "previewed_other_map"
    assert model.detail["active_map"] == "active_map"
    assert nav.calls == []
    assert gateway.goal_pose.msg_count == 0


def test_click_navigation_rejects_safety_stop_without_planning_or_publishing():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ClickNavRequest, GatewayErrorResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    with gateway._state_lock:
        gateway._safety = {"level": 2}
    post_click = _endpoint(gateway, "/api/v1/navigate/click")

    response = asyncio.run(
        post_click(
            ClickNavRequest(
                x=3.0,
                y=4.0,
                z=0.0,
                request_id="safety-stop-click",
                client_id="web",
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "safety_stop"
    assert model.command is not None
    assert model.command.name == "navigate_click"
    assert nav.calls == []
    assert gateway.goal_pose.msg_count == 0


def test_click_navigation_previews_publishes_and_replays_request_id_once():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ClickNavRequest, ControlCommandResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    sent_goals = []
    gateway.goal_pose._add_callback(sent_goals.append)
    post_click = _endpoint(gateway, "/api/v1/navigate/click")
    body = ClickNavRequest(
        x=3.0,
        y=4.0,
        z=0.0,
        request_id="click-001",
        client_id="web",
    )

    first = asyncio.run(post_click(body))
    second = asyncio.run(post_click(body))
    model = ControlCommandResponse.model_validate(first)

    assert model.ok is True
    assert model.command.name == "navigate_click"
    assert model.command.accepted is True
    assert model.command.replay is False
    assert model.goal == [3.0, 4.0, 0.0]
    assert first["command"]["replay"] is False
    assert second["command"]["replay"] is True
    assert second["goal"] == [3.0, 4.0, 0.0]
    assert nav.calls == [(3.0, 4.0, 0.0)]
    assert gateway.goal_pose.msg_count == 1
    assert len(sent_goals) == 1
    assert sent_goals[0].pose.position.x == pytest.approx(3.0)
    assert sent_goals[0].pose.position.y == pytest.approx(4.0)
    assert sent_goals[0].frame_id == "map"
    assert model.frame_id == "map"
    assert model.target is not None
    assert model.target.source == "map_click"
    assert model.target.target_type == "map_point"


def test_goal_route_rejects_missing_plan_preview_without_publishing():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, GoalRequest

    gateway = GatewayModule()
    gateway.setup()
    _mark_navigation_ready(gateway)
    sent_goals = []
    gateway.goal_pose._add_callback(sent_goals.append)
    post_goal = _endpoint(gateway, "/api/v1/goal")

    response = asyncio.run(
        post_goal(
            GoalRequest(
                x=1.0,
                y=2.0,
                z=0.0,
                request_id="missing-preview",
                client_id="web",
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "navigation_plan_infeasible"
    assert model.command is not None
    assert model.command.name == "goal"
    assert model.command.accepted is False
    assert model.detail["preview"]["source"] == "gateway_modules"
    assert model.detail["preview"]["reasons"] == ["nav_mission_unavailable"]
    assert sent_goals == []
    assert gateway.goal_pose.msg_count == 0


def test_direct_motion_commands_reject_safety_stop_without_publishing():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import CmdVelRequest, GatewayErrorResponse, InstructionRequest

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._safety = {"level": 2}
    post_cmd_vel = _endpoint(gateway, "/api/v1/cmd_vel")
    post_instruction = _endpoint(gateway, "/api/v1/instruction")

    cmd_response = asyncio.run(
        post_cmd_vel(
            CmdVelRequest(
                vx=0.2,
                wz=0.1,
                request_id="safety-stop-cmd",
                client_id="web",
            )
        )
    )
    instruction_response = asyncio.run(
        post_instruction(
            InstructionRequest(
                text="go to dock",
                request_id="safety-stop-instruction",
                client_id="web",
            )
        )
    )
    cmd_model = GatewayErrorResponse.model_validate(_payload(cmd_response))
    instruction_model = GatewayErrorResponse.model_validate(_payload(instruction_response))

    assert cmd_response.status_code == 409
    assert cmd_model.error == "safety_stop"
    assert cmd_model.command is not None
    assert cmd_model.command.name == "cmd_vel"
    assert instruction_response.status_code == 409
    assert instruction_model.error == "safety_stop"
    assert instruction_model.command is not None
    assert instruction_model.command.name == "instruction"
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.instruction.msg_count == 0


def test_visual_servo_hot_command_publishes_servo_target():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ControlCommandResponse, VisualServoRequest

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {"VisualServoModule": object()}
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


def test_visual_servo_stop_allowed_under_safety_stop_but_find_rejected():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, VisualServoRequest

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {"VisualServoModule": object()}
    targets = []
    gateway.servo_target._add_callback(targets.append)
    with gateway._state_lock:
        gateway._safety = {"level": 2}
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


def test_cmd_vel_rejects_safety_stop_without_publishing_and_emits_rejected_ack():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import CmdVelRequest, GatewayErrorResponse

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._safety = {"level": 2}
    post_cmd_vel = _endpoint(gateway, "/api/v1/cmd_vel")
    queue = gateway._sse_subscribe()

    try:
        response = asyncio.run(
            post_cmd_vel(
                CmdVelRequest(
                    vx=0.2,
                    wz=0.1,
                    request_id="safety-stop-cmd-ack",
                    client_id="web",
                )
            )
        )
        event = queue.get_nowait()
    finally:
        gateway._sse_unsubscribe(queue)
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "safety_stop"
    assert model.command is not None
    assert model.command.accepted is False
    assert gateway.cmd_vel.msg_count == 0
    assert event["type"] == "command_ack"
    assert event["data"]["status_code"] == 409
    assert event["data"]["command"]["name"] == "cmd_vel"
    assert event["data"]["command"]["accepted"] is False


def test_cmd_vel_replays_duplicate_request_id_without_republish():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import CmdVelRequest, ControlCommandResponse

    gateway = GatewayModule()
    gateway.setup()
    post_cmd_vel = _endpoint(gateway, "/api/v1/cmd_vel")
    body = CmdVelRequest(
        vx=0.2,
        vy=0.0,
        wz=0.1,
        request_id="cmd-001",
        client_id="web",
    )

    first = asyncio.run(post_cmd_vel(body))
    second = asyncio.run(post_cmd_vel(body))
    first_model = ControlCommandResponse.model_validate(first)
    second_model = ControlCommandResponse.model_validate(second)

    assert first_model.ok is True
    assert first_model.command.name == "cmd_vel"
    assert first_model.command.replay is False
    assert second_model.command.replay is True
    assert gateway.cmd_vel.msg_count == 1


def test_field_cmd_vel_rejects_missing_native_boundary_without_local_fallback():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import CmdVelRequest, GatewayErrorResponse

    gateway = GatewayModule()
    gateway.setup()
    gateway._teleop_dds_enabled = True
    gateway._nav_commands = None
    post_cmd_vel = _endpoint(gateway, "/api/v1/cmd_vel")

    response = asyncio.run(
        post_cmd_vel(
            CmdVelRequest(
                vx=0.2,
                wz=0.1,
                request_id="native-boundary-missing",
                client_id="web",
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "native_command_rejected"
    assert "command capability is unavailable" in model.detail["reason"]
    assert gateway.cmd_vel.msg_count == 0


def test_native_cmd_vel_ack_does_not_block_gateway_event_loop():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import CmdVelRequest

    entered = threading.Event()
    release = threading.Event()
    ack_threads: list[int] = []

    def blocking_publish(_twist, *, request_id=None):
        ack_threads.append(threading.get_ident())
        entered.set()
        release.wait(timeout=1.0)
        return True

    gateway = GatewayModule()
    gateway.setup()
    gateway.publish_remote_velocity_request = blocking_publish
    post_cmd_vel = _endpoint(gateway, "/api/v1/cmd_vel")

    async def run_request():
        loop_thread = threading.get_ident()
        started_at = time.perf_counter()
        request_task = asyncio.create_task(
            post_cmd_vel(
                CmdVelRequest(
                    vx=0.2,
                    wz=0.1,
                    request_id="nonblocking-cmd-vel",
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
    assert response["teleop_cmd_vel_dds"] is True


def test_cmd_vel_rejects_non_finite_vy():
    from gateway.schemas import CmdVelRequest

    with pytest.raises(ValueError):
        CmdVelRequest(vx=0.0, vy=float("nan"), wz=0.0)


def test_stop_command_remains_available_when_safety_stop_is_active():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ControlCommandResponse

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._safety = {"level": 2}
    post_stop = _endpoint(gateway, "/api/v1/stop")

    result = asyncio.run(post_stop())
    model = ControlCommandResponse.model_validate(result)

    assert model.ok is True
    assert model.status == "stopped"
    assert gateway.stop_cmd.msg_count == 1
    assert gateway.cmd_vel.msg_count == 1


def test_field_stop_reports_missing_native_boundary_without_local_cmd_vel():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, StopRequest

    gateway = GatewayModule()
    gateway.setup()
    gateway._teleop_dds_enabled = True
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
    assert gateway.cmd_vel.msg_count == 0


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
    gateway._teleop_dds_enabled = True
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
    gateway._teleop_dds_enabled = True
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
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0


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
    gateway._teleop_dds_enabled = True
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
    assert model.stage == "local_published"
    assert model.execution_confirmed is False
    assert model.task_id is None
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
        "native_resume_autonomy",
        lambda _gw, reason, *, request_id=None: calls.append((reason, request_id)) or True,
    )
    gateway = GatewayModule()
    gateway.setup()
    post_resume = _endpoint(gateway, "/api/v1/navigation/resume")

    result = asyncio.run(post_resume(StopRequest(request_id="resume-001", client_id="web")))
    model = ControlCommandResponse.model_validate(result)

    assert model.ok is True
    assert model.status == "autonomy_resume_ready"
    assert result["goal_reissue_required"] is True
    assert calls == [("operator_resume", "resume-001")]
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0


def test_active_control_lease_blocks_other_rest_motion_clients():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import CmdVelRequest

    gateway = GatewayModule()
    gateway.setup()
    assert gateway._lease.acquire("operator-a", 30.0) is True
    post_cmd_vel = _endpoint(gateway, "/api/v1/cmd_vel")

    response = _payload(
        asyncio.run(
            post_cmd_vel(
                CmdVelRequest(
                    vx=0.2,
                    vy=0.0,
                    wz=0.1,
                    request_id="blocked-motion",
                    client_id="operator-b",
                )
            )
        )
    )

    assert response["ok"] is False
    assert response["error"] == "control_lease"
    assert response["detail"]["lease"]["holder"] == "operator-a"
    assert gateway.cmd_vel.msg_count == 0


def test_active_control_lease_blocks_other_rest_resume_clients(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.routes import commands
    from gateway.schemas import StopRequest

    calls = []
    monkeypatch.setattr(
        commands,
        "native_resume_autonomy",
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
    queue = gateway._sse_subscribe()

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
        gateway._sse_unsubscribe(queue)

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
    queue = gateway._sse_subscribe()

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
        gateway._sse_unsubscribe(queue)

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
    assert policy["rate_policy_hz"]["cmd_vel"] == 20.0
    assert health["gateway"]["commands"]["rate_policy_enforcement"] == "advisory"


def test_goal_route_returns_goal_service_task_receipt_without_claiming_execution():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ControlCommandResponse, GoalRequest

    class GeneratedTaskGoals:
        def __init__(self) -> None:
            self.calls = []

        def submit_goal(
            self,
            goal,
            *,
            task_id=None,
            request_id=None,
            action="goal",
        ):
            self.calls.append((goal, task_id, request_id, action))
            return {
                "accepted": True,
                "success": True,
                "task_id": "task-generated-by-goal-service",
                "request_id": request_id,
                "native_task_id": "task-generated-by-goal-service",
                "native_request_id": request_id,
                "native_ack": {
                    "accepted": True,
                    "task_id": "task-generated-by-goal-service",
                    "request_id": request_id,
                },
                "sink": "native_dds",
            }

    gateway = GatewayModule()
    gateway.setup()
    goals = GeneratedTaskGoals()
    gateway.on_system_modules(
        {"nav.mission": _FakePlanPreviewNav(), "nav.goals": goals}
    )
    _mark_navigation_ready(gateway)
    post_goal = _endpoint(gateway, "/api/v1/goal")

    result = asyncio.run(
        post_goal(
            GoalRequest(
                x=1.0,
                y=2.0,
                request_id="attempt-1",
                client_id="web",
            )
        )
    )
    model = ControlCommandResponse.model_validate(result)

    assert model.task_id == "task-generated-by-goal-service"
    assert model.native_request_id == "attempt-1"
    assert model.stage == "native_acknowledged"
    assert model.execution_confirmed is False
    assert goals.calls[0][1:] == (None, "attempt-1", "goal")
    assert gateway.goal_pose.msg_count == 0


def test_exact_task_cancel_forwards_path_identity_and_reports_request_stage():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import CancelRequest, ControlCommandResponse

    class ExactCancelGoals:
        def __init__(self) -> None:
            self.calls = []

        def submit_cancel(self, reason, *, task_id=None, request_id=None):
            self.calls.append((reason, task_id, request_id))
            return {
                "accepted": True,
                "success": True,
                "task_id": task_id,
                "request_id": request_id,
                "native_task_id": task_id,
                "native_request_id": request_id,
                "native_ack": {
                    "accepted": True,
                    "task_id": task_id,
                    "request_id": request_id,
                },
                "sink": "native_dds",
            }

    gateway = GatewayModule()
    gateway.setup()
    goals = ExactCancelGoals()
    gateway.on_system_modules({"nav.goals": goals})
    post_cancel = _endpoint(
        gateway,
        "/api/v1/navigation/tasks/{task_id}/cancel",
    )

    result = asyncio.run(
        post_cancel(
            "task-7",
            CancelRequest(
                reason="operator_cancel",
                request_id="cancel-attempt-1",
                client_id="web",
            ),
        )
    )
    model = ControlCommandResponse.model_validate(result)

    assert model.status == "cancel_requested"
    assert model.task_id == "task-7"
    assert model.stage == "native_acknowledged"
    assert model.execution_confirmed is False
    assert goals.calls == [("operator_cancel", "task-7", "cancel-attempt-1")]


def test_legacy_native_cancel_without_task_id_fails_closed():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import CancelRequest, GatewayErrorResponse

    class NativeCancelGoals:
        def __init__(self) -> None:
            self.task_ids = []

        def submit_cancel(self, reason, *, task_id=None, request_id=None):
            del reason, request_id
            self.task_ids.append(task_id)
            return {
                "accepted": False,
                "success": False,
                "message": "task_id is required to cancel a native navigation task",
            }

    gateway = GatewayModule()
    gateway.setup()
    goals = NativeCancelGoals()
    gateway.on_system_modules({"nav.goals": goals})
    post_cancel = _endpoint(gateway, "/api/v1/navigation/cancel")

    response = asyncio.run(
        post_cancel(
            CancelRequest(
                reason="operator_cancel",
                request_id="cancel-attempt-2",
                client_id="web",
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "native_command_rejected"
    assert "task_id is required" in model.detail["reason"]
    assert goals.task_ids == [None]
    assert gateway.cancel.msg_count == 0


def test_navigation_task_and_request_identity_must_be_distinct():
    from gateway.schemas import GoalRequest

    with pytest.raises(ValueError, match="distinct"):
        GoalRequest(
            x=1.0,
            y=2.0,
            task_id="same-id",
            request_id="same-id",
        )


def test_navigation_routes_expose_stable_task_replay_without_claiming_redispatch():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import (
        CancelRequest,
        ClickNavRequest,
        ControlCommandResponse,
        GoalRequest,
    )

    class ReplayedTaskGoals:
        def submit_goal(self, goal, *, task_id=None, request_id=None, action="goal"):
            del goal, action
            return self._receipt(task_id, request_id, state="running")

        def submit_cancel(self, reason, *, task_id=None, request_id=None):
            del reason
            return self._receipt(task_id, request_id, state="cancel_requested")

        @staticmethod
        def _receipt(task_id, request_id, *, state):
            native_ack = {
                "accepted": True,
                "success": True,
                "task_id": task_id,
                "request_id": request_id,
            }
            return {
                "accepted": True,
                "success": True,
                "task_id": task_id,
                "request_id": request_id,
                "native_task_id": task_id,
                "native_request_id": request_id,
                "native_ack": native_ack,
                "sink": "native_dds",
                "state": state,
                "task_state": state,
                "replay": True,
                "admission_confirmed": True,
                "admission_unconfirmed": False,
                "history_recorded": True,
            }

    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules(
        {"nav.mission": _FakePlanPreviewNav(), "nav.goals": ReplayedTaskGoals()}
    )
    _mark_navigation_ready(gateway)

    openapi = gateway._app.openapi()
    for path in (
        "/api/v1/goal",
        "/api/v1/navigate/click",
        "/api/v1/navigation/cancel",
        "/api/v1/navigation/tasks/{task_id}/cancel",
    ):
        schema = openapi["paths"][path]["post"]["responses"]["202"]["content"][
            "application/json"
        ]["schema"]
        assert schema["$ref"].endswith("/ControlCommandResponse")

    goal = asyncio.run(
        _endpoint(gateway, "/api/v1/goal")(
            GoalRequest(
                x=1.0,
                y=2.0,
                task_id="task-goal",
                request_id="attempt-goal",
            )
        )
    )
    click = asyncio.run(
        _endpoint(gateway, "/api/v1/navigate/click")(
            ClickNavRequest(
                x=2.0,
                y=3.0,
                task_id="task-click",
                request_id="attempt-click",
            )
        )
    )
    cancel = asyncio.run(
        _endpoint(gateway, "/api/v1/navigation/tasks/{task_id}/cancel")(
            "task-cancel",
            CancelRequest(request_id="attempt-cancel"),
        )
    )

    for payload, expected_state in (
        (goal, "running"),
        (click, "running"),
        (cancel, "cancel_requested"),
    ):
        model = ControlCommandResponse.model_validate(payload)
        assert model.task_replay is True
        assert model.task_state == expected_state
        assert model.admission_confirmed is True
        assert model.admission_unconfirmed is False
        assert model.history_recorded is True
        assert model.stage == "task_replayed"
        assert model.command.replay is False

    assert gateway.goal_pose.msg_count == 0
    assert gateway.cancel.msg_count == 0


def test_goal_route_reports_unconfirmed_native_admission_without_journal_accept():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ControlCommandResponse, GoalRequest

    class UnconfirmedGoals:
        def __init__(self):
            self.calls = 0

        def submit_goal(self, goal, *, task_id=None, request_id=None, action="goal"):
            del goal, action
            self.calls += 1
            return {
                "accepted": False,
                "success": False,
                "task_id": task_id,
                "request_id": request_id,
                "state": "unknown",
                "task_state": "unknown",
                "replay": self.calls > 1,
                "admission_confirmed": False,
                "admission_unconfirmed": True,
                "history_recorded": True,
                "message": "native acknowledgement timed out",
                "sink": "native_dds",
            }

    gateway = GatewayModule()
    gateway.setup()
    goals = UnconfirmedGoals()
    gateway.on_system_modules(
        {"nav.mission": _FakePlanPreviewNav(), "nav.goals": goals}
    )
    _mark_navigation_ready(gateway)

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/goal")(
            GoalRequest(
                x=1.0,
                y=2.0,
                task_id="task-unknown",
                request_id="attempt-unknown",
            )
        )
    )
    replay_response = asyncio.run(
        _endpoint(gateway, "/api/v1/goal")(
            GoalRequest(
                x=1.0,
                y=2.0,
                task_id="task-unknown",
                request_id="attempt-unknown",
            )
        )
    )
    model = ControlCommandResponse.model_validate(_payload(response))
    replay_model = ControlCommandResponse.model_validate(_payload(replay_response))

    assert response.status_code == 202
    assert model.ok is False
    assert model.status == "native_command_unconfirmed"
    assert model.task_id == "task-unknown"
    assert model.task_state == "unknown"
    assert model.admission_confirmed is False
    assert model.admission_unconfirmed is True
    assert model.history_recorded is True
    assert model.stage == "native_unconfirmed"
    assert model.command.accepted is False
    assert replay_response.status_code == 202
    assert replay_model.task_replay is True
    assert replay_model.admission_unconfirmed is True
    assert replay_model.command.accepted is False
    assert goals.calls == 2
    assert gateway._command_stats_snapshot()["accepted_commands"] == 0


def test_exact_cancel_reports_unconfirmed_native_admission_without_journal_accept():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import CancelRequest, ControlCommandResponse

    class UnconfirmedCancelGoals:
        def submit_cancel(self, reason, *, task_id=None, request_id=None):
            del reason
            return {
                "accepted": False,
                "success": False,
                "task_id": task_id,
                "request_id": request_id,
                "state": "unknown",
                "task_state": "unknown",
                "replay": True,
                "admission_confirmed": False,
                "admission_unconfirmed": True,
                "history_recorded": True,
                "message": "cancel acknowledgement timed out",
                "sink": "native_dds",
            }

    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.goals": UnconfirmedCancelGoals()})

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/navigation/tasks/{task_id}/cancel")(
            "task-cancel-unknown",
            CancelRequest(request_id="cancel-attempt-unknown"),
        )
    )
    model = ControlCommandResponse.model_validate(_payload(response))

    assert response.status_code == 202
    assert model.status == "native_command_unconfirmed"
    assert model.task_id == "task-cancel-unknown"
    assert model.task_replay is True
    assert model.admission_unconfirmed is True
    assert model.command.accepted is False
    assert gateway._command_stats_snapshot()["accepted_commands"] == 0


def test_command_ack_sse_whitelists_navigation_task_truth_without_internal_fields():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    queue = gateway._sse_subscribe()
    try:
        gateway._publish_command_ack(
            {
                "schema_version": 1,
                "ok": False,
                "status": "native_command_unconfirmed",
                "command": {
                    "name": "goal",
                    "request_id": "attempt-unknown",
                    "client_id": "web",
                    "accepted": False,
                    "replay": False,
                    "ts": time.time(),
                },
                "task_id": "task-unknown",
                "task_replay": True,
                "task_state": "unknown",
                "admission_confirmed": False,
                "admission_unconfirmed": True,
                "history_recorded": True,
                "history_warning": None,
                "task_message": "Query or cancel this task by task_id.",
                "stage": "native_unconfirmed",
                "execution_confirmed": False,
                "internal_ledger_row": {"database_id": 42},
            },
            status_code=202,
        )
        event = queue.get_nowait()
    finally:
        gateway._sse_unsubscribe(queue)

    assert event["type"] == "command_ack"
    data = event["data"]
    assert data["status_code"] == 202
    assert data["task_id"] == "task-unknown"
    assert data["task_replay"] is True
    assert data["task_state"] == "unknown"
    assert data["admission_confirmed"] is False
    assert data["admission_unconfirmed"] is True
    assert data["history_recorded"] is True
    assert data["stage"] == "native_unconfirmed"
    assert data["execution_confirmed"] is False
    assert "internal_ledger_row" not in data


class _PreGateReplayGoals:
    def __init__(self, response_factory) -> None:
        self._response_factory = response_factory
        self.lookup_calls = []
        self.submit_calls = []

    def lookup_goal_replay(
        self,
        goal,
        *,
        task_id=None,
        request_id=None,
        action="goal",
    ):
        self.lookup_calls.append((action, goal, task_id, request_id))
        response = self._response_factory(task_id, request_id)
        if isinstance(response, Exception):
            raise response
        return response

    def submit_goal(
        self,
        goal,
        *,
        task_id=None,
        request_id=None,
        action="goal",
    ):
        self.submit_calls.append((action, goal, task_id, request_id))
        raise AssertionError("durable replay must not redispatch the goal")


def _accepted_replay_receipt(task_id, request_id):
    native_ack = {
        "accepted": True,
        "success": True,
        "task_id": task_id,
        "request_id": request_id,
    }
    return {
        "accepted": True,
        "success": True,
        "task_id": task_id,
        "request_id": request_id,
        "native_task_id": task_id,
        "native_request_id": request_id,
        "native_ack": native_ack,
        "sink": "native_dds",
        "state": "running",
        "task_state": "running",
        "replay": True,
        "admission_confirmed": True,
        "admission_unconfirmed": False,
        "history_recorded": True,
    }


@pytest.mark.parametrize(
    ("path", "command_name", "request_type"),
    [
        ("/api/v1/goal", "goal", "goal"),
        ("/api/v1/navigate/click", "navigate_click", "click"),
    ],
)
def test_durable_goal_replay_runs_before_all_motion_gates_and_preserves_http_replay(
    monkeypatch,
    path,
    command_name,
    request_type,
):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ClickNavRequest, ControlCommandResponse, GoalRequest
    from gateway.services.control_commands import ControlCommandService

    goals = _PreGateReplayGoals(_accepted_replay_receipt)
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.goals": goals})

    def unexpected_gate(*_args, **_kwargs):
        raise AssertionError("durable replay must be resolved before motion gates")

    for name in (
        "motion_safety_rejection",
        "_goal_readiness_rejection",
        "_goal_map_identity_rejection",
        "_goal_plan_preview_rejection",
    ):
        monkeypatch.setattr(ControlCommandService, name, unexpected_gate)

    request_class = GoalRequest if request_type == "goal" else ClickNavRequest
    body = request_class(
        x=1.0,
        y=2.0,
        task_id=f"task-{request_type}",
        request_id=f"attempt-{request_type}",
        client_id="web",
        metadata={"map_name": "no-longer-active"},
    )
    endpoint = _endpoint(gateway, path)

    first = asyncio.run(endpoint(body))
    second = asyncio.run(endpoint(body))
    first_model = ControlCommandResponse.model_validate(_payload(first))
    second_model = ControlCommandResponse.model_validate(_payload(second))

    assert first_model.stage == "task_replayed"
    assert first_model.task_replay is True
    assert first_model.task_state == "running"
    assert first_model.command.name == command_name
    assert first_model.command.replay is False
    assert second_model.command.replay is True
    assert second_model.task_replay is True
    assert len(goals.lookup_calls) == 1
    assert goals.submit_calls == []
    assert gateway.goal_pose.msg_count == 0


def test_unconfirmed_durable_replay_remains_202_before_motion_gates(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ControlCommandResponse, GoalRequest
    from gateway.services.control_commands import ControlCommandService

    def unconfirmed(task_id, request_id):
        return {
            "accepted": False,
            "success": False,
            "task_id": task_id,
            "request_id": request_id,
            "state": "unknown",
            "task_state": "unknown",
            "replay": True,
            "admission_confirmed": False,
            "admission_unconfirmed": True,
            "history_recorded": True,
            "message": "native acknowledgement timed out",
            "sink": "native_dds",
        }

    goals = _PreGateReplayGoals(unconfirmed)
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.goals": goals})
    monkeypatch.setattr(
        ControlCommandService,
        "motion_safety_rejection",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("unconfirmed replay must be resolved before safety")
        ),
    )

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/goal")(
            GoalRequest(
                x=1.0,
                y=2.0,
                task_id="task-unknown-pre-gate",
                request_id="attempt-unknown-pre-gate",
            )
        )
    )
    model = ControlCommandResponse.model_validate(_payload(response))

    assert response.status_code == 202
    assert model.status == "native_command_unconfirmed"
    assert model.task_replay is True
    assert model.admission_unconfirmed is True
    assert model.command.accepted is False
    assert goals.submit_calls == []
    assert gateway._command_stats_snapshot()["accepted_commands"] == 0


@pytest.mark.parametrize(
    "probe_result, expected_reason",
    [
        (
            {
                "accepted": False,
                "success": False,
                "task_id": "task-conflict",
                "request_id": "attempt-conflict",
                "message": "task admission conflict: target changed",
            },
            "target changed",
        ),
        (RuntimeError("task history unavailable"), "task history unavailable"),
    ],
)
def test_durable_replay_probe_conflict_or_failure_blocks_before_motion_gates(
    monkeypatch,
    probe_result,
    expected_reason,
):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, GoalRequest
    from gateway.services.control_commands import ControlCommandService

    goals = _PreGateReplayGoals(lambda _task_id, _request_id: probe_result)
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.goals": goals})
    monkeypatch.setattr(
        ControlCommandService,
        "motion_safety_rejection",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("probe failure must fail closed before safety")
        ),
    )

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/goal")(
            GoalRequest(
                x=1.0,
                y=2.0,
                task_id="task-conflict",
                request_id="attempt-conflict",
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "navigation_task_replay_rejected"
    assert model.detail["reason_code"] == "navigation_task_replay_rejected"
    assert model.detail["source"] == "navigation_task_history"
    assert expected_reason in model.detail["reason"]
    assert goals.submit_calls == []
    assert gateway.goal_pose.msg_count == 0


def test_unknown_durable_attempt_still_runs_the_existing_motion_gates():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, GoalRequest

    goals = _PreGateReplayGoals(lambda _task_id, _request_id: None)
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.goals": goals})
    with gateway._state_lock:
        gateway._safety = {"level": 2}

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/goal")(
            GoalRequest(
                x=1.0,
                y=2.0,
                task_id="task-new",
                request_id="attempt-new",
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "safety_stop"
    assert len(goals.lookup_calls) == 1
    assert goals.submit_calls == []
    assert gateway.goal_pose.msg_count == 0

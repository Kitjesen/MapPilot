from __future__ import annotations

import asyncio
import hashlib
import json
import math
import threading
import time
from concurrent.futures import ThreadPoolExecutor
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
                "source_profile": "thunder",
                "data_source": "thunder",
                "slam_source": "fastlio2",
                "localization_source": "fastlio2",
                "mapping_source": "fastlio2",
                "frame_id": "map",
                "created_at": "2026-05-25T00:00:00Z",
                "artifacts": {
                    "map_pcd": {
                        "path": "map.pcd",
                        "sha256": map_sha,
                        "source_profile": "thunder",
                        "data_source": "thunder",
                        "slam_source": "fastlio2",
                        "frame_id": "map",
                        "point_count": 1,
                    },
                    "octomap": {
                        "path": "octomap.ot",
                        "sha256": octomap_sha,
                        "source_map_sha256": map_sha,
                        "source_profile": "thunder",
                        "data_source": "thunder",
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

    def preview_plan(self, x: float, y: float, z: float, **_options) -> dict:
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










def test_runtime_switch_plan_rejects_configuration_profile_as_product() -> None:
    from gateway.schemas import RuntimeSwitchPlanRequest

    with pytest.raises(ValidationError):
        RuntimeSwitchPlanRequest(
            current_product="sim_mujoco_live",
            target_product="explore",
        )














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
    assert model.status == "ok"
    assert model.goal == [1.0, 2.0, 0.0]
    assert model.command.name == "goal"
    assert model.command.request_id == "goal-001"
    assert model.command.task_id
    assert model.command.task_id != model.command.request_id
    assert model.command.client_id == "web"
    assert first["command"]["accepted"] is True
    assert first["command"]["replay"] is False
    assert second["command"]["replay"] is True
    assert second["goal"] == [1.0, 2.0, 0.0]
    assert second["command"]["request_id"] == "goal-001"
    assert second["command"]["task_id"] == first["command"]["task_id"]


def test_command_journal_does_not_replay_another_clients_request_id():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GoalRequest

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    post_goal = _endpoint(gateway, "/api/v1/goal")

    first = asyncio.run(
        post_goal(
            GoalRequest(
                x=1.0,
                y=2.0,
                request_id="shared-request-id",
                client_id="operator-a",
            )
        )
    )
    second = asyncio.run(
        post_goal(
            GoalRequest(
                x=3.0,
                y=4.0,
                request_id="shared-request-id",
                client_id="operator-b",
            )
        )
    )

    assert first["command"]["replay"] is False
    assert second["command"]["replay"] is False
    assert second["command"]["client_id"] == "operator-b"
    assert second["goal"] == [3.0, 4.0, 0.0]
    assert gateway.goal_pose.msg_count == 2


def test_anonymous_clients_do_not_share_an_idempotency_scope():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GoalRequest

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    post_goal = _endpoint(gateway, "/api/v1/goal")

    first = asyncio.run(
        post_goal(GoalRequest(x=1.0, y=2.0, request_id="anonymous-request"))
    )
    second = asyncio.run(
        post_goal(GoalRequest(x=3.0, y=4.0, request_id="anonymous-request"))
    )

    assert first["command"]["client_id"] == "unknown"
    assert second["command"]["client_id"] == "unknown"
    assert first["command"]["replay"] is False
    assert second["command"]["replay"] is False
    assert first["goal"] == [1.0, 2.0, 0.0]
    assert second["goal"] == [3.0, 4.0, 0.0]
    assert gateway.goal_pose.msg_count == 2
    assert gateway._command_journal.snapshot()["stored_requests"] == 0


def test_command_journal_rejects_changed_payload_for_the_same_request_identity():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, GoalRequest

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway)
    post_goal = _endpoint(gateway, "/api/v1/goal")

    first = asyncio.run(
        post_goal(
            GoalRequest(
                x=1.0,
                y=2.0,
                request_id="changed-payload",
                client_id="operator-a",
            )
        )
    )
    second_response = asyncio.run(
        post_goal(
            GoalRequest(
                x=5.0,
                y=6.0,
                request_id="changed-payload",
                client_id="operator-a",
            )
        )
    )
    second = _payload(second_response)
    model = GatewayErrorResponse.model_validate(second)

    assert first["command"]["replay"] is False
    assert second_response.status_code == 409
    assert model.error == "idempotency_conflict"
    assert model.command is not None
    assert model.command.accepted is False
    assert model.command.replay is False
    assert model.command.client_id == "operator-a"
    assert model.command.request_id == "changed-payload"
    assert model.detail["reason_code"] == "idempotency_conflict"
    assert gateway.goal_pose.msg_count == 1
    assert nav.calls == [(1.0, 2.0, 0.0)]
    assert gateway._command_journal.snapshot()["conflicting_commands"] == 1


def test_concurrent_exact_command_retries_execute_once_then_replay():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import CmdVelRequest

    gateway = GatewayModule()
    gateway.setup()
    post_cmd_vel = _endpoint(gateway, "/api/v1/cmd_vel")
    entered = threading.Event()
    release = threading.Event()
    calls: list[str | None] = []

    def blocking_publish(_twist, *, request_id=None):
        calls.append(request_id)
        entered.set()
        assert release.wait(timeout=3.0)
        return False

    gateway.publish_remote_velocity_request = blocking_publish
    body = CmdVelRequest(
        vx=0.2,
        vy=0.0,
        wz=0.1,
        request_id="concurrent-command",
        client_id="web",
    )

    with ThreadPoolExecutor(max_workers=2) as pool:
        first_future = pool.submit(lambda: asyncio.run(post_cmd_vel(body)))
        assert entered.wait(timeout=2.0)
        second_future = pool.submit(lambda: asyncio.run(post_cmd_vel(body)))
        try:
            time.sleep(0.1)
            assert calls == ["concurrent-command"]
            assert second_future.done() is False
        finally:
            release.set()
        results = [first_future.result(timeout=3.0), second_future.result(timeout=3.0)]

    assert sorted(result["command"]["replay"] for result in results) == [False, True]
    assert calls == ["concurrent-command"]
    stats = gateway._command_journal.snapshot()
    assert stats["accepted_commands"] == 1
    assert stats["replayed_commands"] == 1
    assert stats["pending_requests"] == 0


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


def test_malformed_truthy_command_receipt_cannot_claim_ack_or_audit_success():
    from gateway.services.commands import publish_command_ack, run_control_command

    events: list[dict] = []
    audit_records: list[dict] = []

    class MalformedJournal:
        def execute(self, *_args, **_kwargs):
            return {
                "ok": "false",
                "command": {
                    "accepted": "false",
                    "replay": "false",
                },
            }

    class AuditJournal:
        def record(self, command, **kwargs):
            audit_records.append({"command": command, **kwargs})

    gateway = SimpleNamespace(
        _command_journal=MalformedJournal(),
        _audit_journal=AuditJournal(),
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
    assert audit_records == [
        {
            "command": "test_command",
            "client_id": "web",
            "ok": False,
            "error": None,
        }
    ]












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
    assert first_event["data"]["accepted"] is True
    assert first_event["data"]["status"] == "ok"
    assert first_event["data"]["stage"] == "submitted"
    assert first_event["data"]["success"] is None
    assert first_event["data"]["execution_confirmed"] is False
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
    assert first["accepted"] is True
    assert first["stage"] == "source_request_accepted"
    assert first["source_request_accepted"] is True
    assert first["final_output_confirmed"] is False
    assert first["execution_confirmed"] is False
    assert first["final_output_confirmed"] is False
    assert "success" not in first
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
    assert "native operator motion capability is unavailable" in model.detail["reason"]
    assert gateway.cmd_vel.msg_count == 0


def test_field_cmd_vel_rejects_stateless_native_motion_even_when_publisher_exists():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import CmdVelRequest, GatewayErrorResponse

    class Publisher:
        def submit(self, *_args, **_kwargs):
            raise AssertionError("stateless REST must not enqueue a native sample")

    gateway = GatewayModule()
    gateway.setup()
    gateway._teleop_dds_enabled = True
    gateway._teleop_native_publisher = Publisher()
    post_cmd_vel = _endpoint(gateway, "/api/v1/cmd_vel")

    response = asyncio.run(
        post_cmd_vel(
            CmdVelRequest(
                vx=0.2,
                wz=0.1,
                request_id="stateless-native-motion",
                client_id="web",
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "native_command_rejected"
    assert "explicit claimed source session" in model.detail["reason"]
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
    assert response["stage"] == "source_request_accepted"
    assert response["source_request_accepted"] is True
    assert response["adapter_reported_write"] is True
    assert response["endpoint_submission_confirmed"] is False
    assert response["execution_confirmed"] is False
    assert response["final_output_confirmed"] is False
    assert response["teleop_cmd_vel_dds"] is False


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

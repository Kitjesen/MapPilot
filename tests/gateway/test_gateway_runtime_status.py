from __future__ import annotations

import asyncio
import json
import time

import pytest

pytestmark = [pytest.mark.sim]


pytest.importorskip("fastapi")
from gateway.services.sse import subscribe, unsubscribe
from tests.runtime.numpy_guard import NUMPY_UNSAFE_REASON, numpy_import_is_safe

_NUMPY_IMPORT_SAFE = numpy_import_is_safe()


def _endpoint(gateway, path: str):
    gateway.setup()
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def _payload(response_or_payload):
    if hasattr(response_or_payload, "body"):
        return json.loads(response_or_payload.body)
    return response_or_payload


def _field_manifest(profile: str, *, variant: str | None = None, local_planner: str | None = None):
    from lingtu.assembly.compiler import compile_run_plan
    from lingtu.assembly.products import resolve_product_host_runtime

    resolved = resolve_product_host_runtime(
        profile,
        "real",
        robot="unitree/go2",
        product_variant=variant,
    )
    return compile_run_plan(
        resolved.product,
        resolved.env,
        robot="unitree/go2",
        product_variant=variant,
        local_planner=local_planner,
    )


def _field_gateway(profile: str, *, variant: str | None = None, local_planner: str | None = None):
    from gateway.gateway_module import GatewayModule

    return GatewayModule(run_plan=_field_manifest(profile, variant=variant, local_planner=local_planner))


def _set_session_mode(gateway, mode: str) -> None:
    original_snapshot = gateway._session_snapshot

    def snapshot():
        payload = dict(original_snapshot())
        payload["mode"] = mode
        gateway._session_mode = mode
        return payload

    gateway._session_mode = mode
    gateway._session_snapshot = snapshot


def _write_active_same_source_octomap(map_root):
    active_dir = map_root / "demo"
    active_dir.mkdir(parents=True)
    (map_root / "active_map.txt").write_text("demo\n", encoding="utf-8")
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
    octomap_path.write_bytes(b"lingtu-test-octomap")
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


class _FilesystemMapdClient:
    """Stateless mapd test transport backed by one isolated map root."""

    def __init__(self, map_root):
        self.map_root = map_root
        self.calls = []

    def service(self, action, **arguments):
        self.calls.append({"action": action, **arguments})
        if action in {"get_active", "get_active_map"}:
            state = self.map_root / "active_map.txt"
            active = state.read_text(encoding="utf-8").strip() if state.is_file() else ""
            return {"action": action, "success": True, "active": active}
        if action == "validate_artifacts":
            map_id = str(arguments.get("map_id") or "")
            map_dir = self.map_root / map_id
            octomap_ok = (map_dir / "octomap.ot").is_file()
            occupancy_ok = (map_dir / "occupancy.npz").is_file()
            gate = {
                "ok": (not arguments.get("require_octomap") or octomap_ok)
                and (not arguments.get("require_occupancy") or occupancy_ok),
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
        return {"action": action, "success": False, "reason_code": "unsupported_test_action"}


def test_diagnostics_plugin_catalog_route():
    from gateway.gateway_module import GatewayModule
    from gateway.routes.diagnostics import clear_diagnostics_cache

    clear_diagnostics_cache()

    gateway = GatewayModule()
    payload = asyncio.run(_endpoint(gateway, "/api/v1/diagnostics/plugins")())

    assert payload["schema_version"] == 1
    assert "gateway" in payload["categories"]


def test_localization_status_covers_product_states():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()

    payload = build_localization_status(gateway)
    assert payload["state"] == "no_odometry"
    assert payload["has_odometry"] is False
    assert payload["can_relocalize"] is False

    with gateway._state_lock:
        gateway._odom = {"x": 1.0}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    payload = build_localization_status(gateway)
    assert payload["state"] == "tracking"
    assert payload["reported_state"] == "TRACKING"

    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "localizer_health": "RECOVERED",
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "ready"
    assert payload["ready"] is True
    assert payload["algorithm_healthy"] is True
    assert payload["pose_fresh"] is True

    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.89,
            "degeneracy": "NONE",
            "icp_fitness": 0.0,
            "localizer_health": "RECOVERED",
            "localizer_health_source": "localizer_health_topic",
            "localizer_health_fitness": 0.0246,
            "odom_age_ms": 222.4,
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "ready"
    assert payload["ready"] is True
    assert payload["algorithm_healthy"] is True

    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.7,
            "degeneracy": "MILD",
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "map_cloud_fresh": True,
            "icp_fitness": 0.0,
            "odom_age_ms": 150.0,
            "cloud_age_ms": 120.0,
            "localizer_health": "LIO_TRACKING",
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "ready"
    assert payload["ready"] is True
    assert payload["algorithm_healthy"] is True
    assert payload["degeneracy"] == "MILD"
    assert payload["reasons"] == []

    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.66,
            "quality": 0.66,
            "backend": "fastlio2",
            "health_source": "slam_runtime",
            "pose_fresh": True,
            "has_odom": True,
            "map_odom_tf": {
                "valid": True,
                "frame_id": "map",
                "child_frame_id": "odom",
                "tx": 0.0,
                "ty": 0.0,
                "tz": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0,
                "ts": 123.0,
            },
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "ready"
    assert payload["ready"] is True
    assert payload["algorithm_healthy"] is True
    assert payload["backend"] == "native_dds"
    assert payload["algorithm_profile"] == "fastlio2"
    assert payload["health_source"] == "slam_runtime"
    assert payload["has_map_odom_tf"] is True

    _set_session_mode(gateway, "navigating")
    gateway._icp_quality = 0.2
    with gateway._state_lock:
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    payload = build_localization_status(gateway)
    assert payload["state"] == "ready"
    assert payload["ready"] is True
    with gateway._state_lock:
        gateway._localization_status = {
            "backend": "fastlio2",
            "health_source": "slam_runtime",
            "state": "DEGRADED",
            "confidence": 0.4,
            "degeneracy": "MILD",
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "degraded"
    assert payload["can_relocalize"] is True
    assert "low_confidence" in payload["reasons"]

    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.28,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 1440.0,
            "cloud_age_ms": 120.0,
            "localizer_health": "RECOVERED",
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "ready"
    assert payload["algorithm_healthy"] is True
    assert payload["pose_fresh"] is True
    assert payload["pose_freshness"] == "fresh"
    assert payload["stale_odometry"] is False
    assert payload["odom_age_ms"] == 1440.0
    assert payload["reasons"] == []

    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.28,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 2500.0,
            "cloud_age_ms": 120.0,
            "localizer_health": "RECOVERED",
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "degraded"
    assert payload["algorithm_healthy"] is True
    assert payload["pose_fresh"] is False
    assert payload["pose_freshness"] == "stale"
    assert payload["stale_odometry"] is True
    assert payload["odom_age_ms"] == 2500.0
    assert payload["reasons"] == ["reported_state:tracking", "stale_odometry"]

    with gateway._state_lock:
        gateway._localization_status = {
            "backend": "fastlio2",
            "health_source": "slam_runtime",
            "state": "LOST",
            "confidence": 0.0,
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "lost"
    assert payload["can_relocalize"] is True

    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "localizer_health": "LOST",
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "lost"
    assert "localizer_health:lost" in payload["reasons"]


def test_localization_status_rejects_incomplete_map_odom_tf() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "frame_id": "odom"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "map_odom_tf": {
                "valid": True,
                "frame_id": "map",
                "child_frame_id": "odom",
                "tx": 0.0,
                "ty": 0.0,
                "tz": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0,
            },
        }

    payload = build_localization_status(gateway)

    assert payload["map_odom_tf"] is None
    assert payload["has_map_odom_tf"] is False




def test_localization_status_route_returns_stable_schema():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.8}

    payload = asyncio.run(_endpoint(gateway, "/api/v1/localization/status")())

    assert payload["schema_version"] == 1
    assert payload["state"] == "tracking"
    assert payload["has_odometry"] is True
    assert payload["reported_state"] == "TRACKING"


def test_localization_status_exposes_backend_reason():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "frame_id": "odom"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "reason": "tracking",
        }

    payload = build_localization_status(gateway)

    assert payload["reason"] == "tracking"
    assert payload["backend_reason"] == "tracking"


@pytest.mark.parametrize("source,contract,sink", [
    (None, None, None),
    ("field", "real", "driver"),
    ("unknown", "real", "driver"),
    ("field", "unknown", "driver"),
])
def test_runtime_boundary_uses_only_its_static_contract_parts(monkeypatch, source, contract, sink):
    from dataclasses import asdict
    from types import SimpleNamespace

    import diagnostics.runtime_contract as contracts
    from gateway.services.runtime_status import _runtime_boundary_status
    from runtime.tf.frames import FRAMES

    manifest = contracts.runtime_contract_manifest()
    for name, value in {
        "LINGTU_DATA_SOURCE": source, "LINGTU_RUNTIME_CONTRACT": contract,
        "LINGTU_COMMAND_SINK": sink,
    }.items():
        if value is None:
            monkeypatch.delenv(name, raising=False)
        else:
            monkeypatch.setenv(name, value)

    def full_manifest_is_not_a_status_dependency():
        raise AssertionError("status must not reload Product YAML or unrelated robot calibrations")

    monkeypatch.setattr(contracts, "runtime_contract_manifest", full_manifest_is_not_a_status_dependency)
    gateway = SimpleNamespace(
        _compiled_env="real", _compiled_product="teleop_avoid",
        _compiled_run_plan=object(), _compiled_product_session_id="session-a",
    )
    status = _runtime_boundary_status(gateway)
    declared_source = bool(source or contract)
    assert status["frames"] == (manifest["frames"] if declared_source else asdict(FRAMES))
    assert status["frame_links"] == (manifest["frame_links"] if declared_source else {})
    expected_source = manifest["data_sources"].get(source, {})
    for key in ("slam_source", "localization_source", "mapping_source"):
        assert status[key] == expected_source.get(key)
    assert status["expected_command_sink"] == expected_source.get("command_sink")
    assert ("data_source_unknown" in status["blockers"]) == (source == "unknown")
    assert ("topic_frame_contract_unavailable" in status["blockers"]) == (contract == "unknown")

    gateway._compiled_run_plan = None
    gateway._compiled_product_session_id = "session-b"
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "wrong")
    changed = _runtime_boundary_status(gateway)
    assert changed["state"] == "standby"
    assert changed["product_session_id"] == "session-b"
    assert changed["command_sink"] == "wrong"
    assert ("command_sink_mismatch" in changed["blockers"]) == (source == "field")


def test_localization_status_reports_runtime_boundary_and_topic_frames(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocalizationStatusResponse
    from gateway.services.runtime_status import build_localization_status

    monkeypatch.setenv("LINGTU_PRODUCT", "nav")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "field")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "driver")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "0")

    gateway = GatewayModule()
    _set_session_mode(gateway, "navigating")
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "frame_id": "odom"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "registered_cloud_frame_id": "body",
            "map_cloud_frame_id": "map",
            "localizer_health": "RECOVERED",
            "odom_age_ms": 80.0,
            "cloud_age_ms": 60.0,
            "map_cloud_fresh": True,
            "status_target_hz": 10.0,
            "imu_input_hz": 198.5,
            "lidar_input_hz": 10.0,
            "slam_tick_hz": 50.0,
            "processed_scan_hz": 9.8,
            "registered_points": 24000,
            "map_points": 512000,
            "imu_buffer": 4,
            "lidar_buffer": 1,
            "imu_batch": 20,
            "dropped_lidar_frames": 0,
            "dropped_imu_frames": 1,
            "scan_start_s": 122.9,
            "scan_end_s": 123.0,
            "last_imu_s": 123.01,
            "sync_wait_count": 2,
            "imu_rollback_count": 0,
            "lidar_rollback_count": 0,
            "map_loaded": True,
            "map_frame_jump": False,
            "scene_mode": "outdoor",
            "gnss_fusion_health": {"enabled": True, "alignment_locked": True},
            "map_odom_tf": {
                "valid": True,
                "frame_id": "map",
                "child_frame_id": "odom",
                "tx": 0.0,
                "ty": 0.0,
                "tz": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0,
                "ts": 123.0,
            },
        }

    payload = build_localization_status(gateway)
    model = LocalizationStatusResponse.model_validate(payload)

    runtime = payload["runtime"]
    assert runtime["ok"] is True
    assert runtime["data_source"] == "field"
    assert runtime["runtime_contract"] == "real"
    assert runtime["frames"]["map"] == "map"
    assert runtime["frames"]["odom"] == "odom"
    assert runtime["frames"]["body"] == "body"
    assert runtime["topic_default_frame_ids"]["/slam/odometry"] == "odom"
    assert runtime["topic_default_frame_ids"]["/slam/registered_cloud"] == "body"
    assert runtime["topic_default_frame_ids"]["/slam/map_cloud"] == "map"
    assert runtime["required_topic_frame_ids"][:5] == [
        "/lidar/raw_frame",
        "/imu/raw",
        "/slam/odometry",
        "/slam/registered_cloud",
        "/slam/map_cloud",
    ]
    assert runtime["runtime_data_flow_topics"][:3] == [
        "/lidar/raw_frame",
        "/imu/raw",
        "/slam/odometry",
    ]

    frames = payload["frames"]
    assert frames["runtime_contract"] == "real"
    assert frames["odometry_frame_id"] == "odom"
    assert frames["registered_cloud_frame_id"] == "body"
    assert frames["map_cloud_frame_id"] == "map"
    assert payload["registered_cloud_frame_id"] == "body"
    assert payload["map_cloud_frame_id"] == "map"
    assert frames["odometry_expected_frame_ids"] == ["odom", "map"]
    assert frames["registered_cloud_expected_frame_ids"] == ["body"]
    assert frames["map_cloud_expected_frame_ids"] == ["map"]
    assert frames["missing_required_topic_frame_ids"] == []
    assert frames["mismatches"] == []
    assert frames["ok"] is True
    assert payload["status_target_hz"] == 10.0
    assert payload["imu_input_hz"] == 198.5
    assert payload["lidar_input_hz"] == 10.0
    assert payload["slam_tick_hz"] == 50.0
    assert payload["processed_scan_hz"] == 9.8
    assert payload["registered_points"] == 24000
    assert payload["map_points"] == 512000
    assert payload["imu_buffer"] == 4
    assert payload["lidar_buffer"] == 1
    assert payload["imu_batch"] == 20
    assert payload["dropped_imu_frames"] == 1
    assert payload["sync_wait_count"] == 2
    assert payload["map_loaded"] is True
    assert payload["map_frame_jump"] is False
    assert payload["scene_mode"] == "outdoor"
    assert payload["has_map_odom_tf"] is True
    assert payload["gnss_fusion_health"]["alignment_locked"] is True
    assert model.runtime.data_source == "field"
    assert model.frames.ok is True
    assert model.status_target_hz == 10.0
    assert model.processed_scan_hz == 9.8
    assert model.has_map_odom_tf is True
    assert model.map_odom_tf["child_frame_id"] == "odom"


def test_localization_status_exposes_gateway_diagnostic_age():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocalizationStatusResponse
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
    gateway._on_localization_status({"state": "TRACKING", "confidence": 0.9})

    payload = build_localization_status(gateway)
    model = LocalizationStatusResponse.model_validate(payload)

    assert model.diag_received_ts is not None
    assert model.diag_age_ms is not None
    assert model.diag_age_ms >= 0.0
    assert "_gateway_received_mono" in payload["raw"]


def test_native_slam_status_separates_runtime_identity_algorithm_and_mode():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._get_slam_profile = lambda: "native_dds"

    gateway._on_localization_status(
        {
            "backend": "fastlio2",
            "health_source": "slam_runtime",
            "mode": "localization",
            "state": "TRACKING",
        }
    )

    assert gateway._localization_status["backend"] == "native_dds"
    assert gateway._localization_status["algorithm_profile"] == "fastlio2"
    assert gateway._localization_status["slam_mode"] == "localization"
    assert gateway._localization_status["saved_map_relocalization_supported"] is True


def test_localization_runtime_generation_and_map_jump_reset_viewer_epoch():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._on_localization_status(
        {
            "state": "TRACKING",
            "runtime_instance_id": "slam-a",
            "observation_sequence": 10,
            "map_frame_jump": False,
            "map_frame_jump_sequence": 0,
        }
    )
    assert gateway._traffic_stats_snapshot()["scan"]["scene_epoch"] == 1

    gateway._on_localization_status(
        {
            "state": "STALE",
            "runtime_instance_id": "slam-a",
            "observation_sequence": 10,
            "map_frame_jump_sequence": 0,
        }
    )
    gateway._on_localization_status(
        {
            "state": "TRACKING",
            "runtime_instance_id": "slam-b",
            "observation_sequence": 1,
            "map_frame_jump": False,
            "map_frame_jump_sequence": 0,
        }
    )
    assert gateway._traffic_stats_snapshot()["scan"]["scene_epoch"] == 2
    assert gateway._localization_status["viewer_epoch_reset_reason"] == "slam_runtime_changed"

    gateway._on_localization_status(
        {
            "state": "TRACKING",
            "runtime_instance_id": "slam-b",
            "observation_sequence": 2,
            "map_frame_jump": False,
            "map_frame_jump_sequence": 1,
        }
    )
    assert gateway._traffic_stats_snapshot()["scan"]["scene_epoch"] == 3
    gateway._on_localization_status(
        {
            "state": "TRACKING",
            "runtime_instance_id": "slam-b",
            "observation_sequence": 3,
            "map_frame_jump": False,
            "map_frame_jump_sequence": 1,
        }
    )
    assert gateway._traffic_stats_snapshot()["scan"]["scene_epoch"] == 3


def test_localization_observation_sequence_rollback_resets_legacy_runtime_viewer():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._on_localization_status({"state": "TRACKING", "observation_sequence": 20})
    gateway._on_localization_status({"state": "TRACKING", "observation_sequence": 1})

    assert gateway._traffic_stats_snapshot()["scan"]["scene_epoch"] == 2
    assert gateway._localization_status["viewer_epoch_reset_reason"] == ("slam_observation_sequence_rollback")


def test_localization_status_exposes_slam_quality_diagnostics():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocalizationStatusResponse
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {
            "state": "DEGRADED",
            "confidence": 0.1,
            "degeneracy": "CRITICAL",
            "icp_fitness": 0.3049,
            "effective_ratio": 1.0,
            "condition_number": 50.4,
            "degenerate_dof_count": 0,
            "pos_cov_trace": 0.000017,
            "ieskf_iter_num": 10,
            "ieskf_converged": False,
            "localizer_health": "DEGRADED",
            "localizer_health_fitness": 0.3049,
            "localizer_health_iter": 11,
            "localizer_health_cov_trace": 0.000019,
        }

    payload = build_localization_status(gateway)
    model = LocalizationStatusResponse.model_validate(payload)

    assert model.state == "degraded"
    assert model.icp_fitness == 0.3049
    assert model.effective_ratio == 1.0
    assert model.condition_number == 50.4
    assert model.degenerate_dof_count == 0
    assert model.pos_cov_trace == 0.000017
    assert model.ieskf_iter_num == 10
    assert model.ieskf_converged is False
    assert model.localizer_health == "DEGRADED"
    assert model.localizer_health_fitness == 0.3049
    assert model.localizer_health_iter == 11
    assert model.localizer_health_cov_trace == 0.000019
    assert payload["raw"]["icp_fitness"] == 0.3049


def test_mapping_session_snapshot_does_not_expose_saved_map_as_active(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionResponse

    class _MapdClient:
        @staticmethod
        def service(action, **arguments):
            assert action in {"get_active", "get_active_map"}
            assert arguments == {}
            return {"active": "old_nav_map"}

    gateway = GatewayModule()
    gateway._map_client = _MapdClient()
    _set_session_mode(gateway, "mapping")
    gateway._session_product = "map"

    session = gateway._session_snapshot()
    model = SessionResponse.model_validate(session)

    assert model.active_map is None
    assert model.saved_active_map == "old_nav_map"
    assert session["active_map"] is None
    assert session["saved_active_map"] == "old_nav_map"


def test_slam_profile_prefers_live_native_dds_status_over_stopped_session():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionResponse

    gateway = GatewayModule()
    gateway._session_slam_profile = "stopped"
    gateway._cached_slam_profile = "stopped"
    gateway._localization_status = {
        "backend": "fastlio2",
        "state": "TRACKING",
        "confidence": 0.95,
        "pose_fresh": True,
        "map_cloud_fresh": True,
        "health_source": "slam_runtime",
    }

    assert gateway._get_slam_profile() == "native_dds"

    session = gateway._session_snapshot()
    model = SessionResponse.model_validate(session)

    assert model.slam_profile == "native_dds"
    assert model.localization_backend == "native_dds"
    assert model.map_save_supported is True


def test_slam_profile_keeps_native_dds_runtime_profile():
    from gateway.gateway_module import GatewayModule

    assert (
        GatewayModule._slam_profile_from_status(
            {
                "backend": "fastlio2",
                "health_source": "slam_runtime",
                "mode": "mapping",
                "state": "MAPPING",
            }
        )
        == "native_dds"
    )
    assert (
        GatewayModule._slam_profile_from_status(
            {
                "backend": "fastlio2",
                "health_source": "slam_runtime",
                "mode": "localization",
                "state": "TRACKING",
            }
        )
        == "native_dds"
    )
    assert (
        GatewayModule._slam_profile_from_status(
            {
                "backend": "fastlio2",
                "health_source": "slam_runtime",
                "mode": "localization",
                "state": "FAILED",
            }
        )
        == ""
    )


@pytest.mark.parametrize(
    "rejected_backend",
    (
        "fastlio2",
        "pointlio",
        "genz",
        "localizer",
        "genz-icp",
        "genz_icp",
        "point-lio",
        "point_lio",
        "cpp_dds_slam",
        "messages_dds",
        "lingtu-slam-dds",
        "native_slam",
        "slam",
        "unexpected_backend",
    ),
)
def test_slam_profile_rejects_noncanonical_backend_names(rejected_backend):
    from gateway.gateway_module import GatewayModule

    assert GatewayModule._slam_profile_from_status({"backend": rejected_backend, "state": "TRACKING"}) == ""


def test_slam_profile_accepts_only_native_runtime_identity():
    from gateway.gateway_module import GatewayModule

    assert GatewayModule._slam_profile_from_status({"backend": "native_dds", "state": "TRACKING"}) == "native_dds"
    assert (
        GatewayModule._slam_profile_from_status(
            {
                "backend": "fastlio2",
                "backend_profile": "native_dds",
                "health_source": "dds_endpoint",
                "state": "TRACKING",
            }
        )
        == "native_dds"
    )






def test_drift_watchdog_classifies_nan_odom_as_diverged():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()

    assert (
        gateway._drift_odom_diverged(
            {
                "x": float("nan"),
                "y": 0.0,
                "z": 0.0,
                "vx": 0.0,
            }
        )[0]
        is True
    )


def test_gateway_quarantines_non_finite_odometry_before_publication():
    from gateway.gateway_module import GatewayModule
    from runtime.msgs.geometry import Pose, Quaternion, Twist, Vector3
    from runtime.msgs.nav import Odometry

    gateway = GatewayModule()
    events = []
    gateway.push_event = events.append
    previous = {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0}
    with gateway._state_lock:
        gateway._odom = dict(previous)

    gateway._on_odometry(
        Odometry(
            pose=Pose(
                position=Vector3(float("nan"), 0.0, 0.0),
                orientation=Quaternion(),
            ),
            twist=Twist(linear=Vector3(0.0, 0.0, 0.0)),
        )
    )

    assert gateway._odom == previous
    assert [event.get("type") for event in events] == []
    assert gateway._last_invalid_odometry["reason"] == "non_finite_odometry"


def test_drift_watchdog_uses_quarantined_non_finite_odometry():
    from gateway.gateway_module import GatewayModule
    from runtime.msgs.geometry import Pose, Quaternion, Twist, Vector3
    from runtime.msgs.nav import Odometry

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "z": 0.0, "vx": 0.0}

    gateway._on_odometry(
        Odometry(
            pose=Pose(
                position=Vector3(float("nan"), 0.0, 0.0),
                orientation=Quaternion(),
            ),
            twist=Twist(linear=Vector3(0.0, 0.0, 0.0)),
        )
    )

    diverged, _x, _y, _z, _v, invalid = gateway._drift_current_odom_divergence()
    assert diverged is True
    assert invalid is True

    gateway._on_odometry(
        Odometry(
            pose=Pose(
                position=Vector3(0.0, 0.0, 0.0),
                orientation=Quaternion(),
            ),
            twist=Twist(linear=Vector3(0.0, 0.0, 0.0)),
        )
    )

    diverged, _x, _y, _z, _v, invalid = gateway._drift_current_odom_divergence()
    assert diverged is False
    assert invalid is False




















def test_gateway_navigation_state_pushes_navigation_status_update():
    from gateway.gateway_module import GatewayModule
    from runtime.msgs.nav import (
        NavigationControlMode,
        NavigationExecutionState,
        NavigationLifecycle,
        NavigationPlanningState,
        NavigationState,
    )

    gateway = GatewayModule()
    _set_session_mode(gateway, "navigating")
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "frame_id": "map"}
        gateway._mode = "autonomous"
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    queue = subscribe(gateway)

    try:
        gateway._on_navigation_state(
            NavigationState(
                boot_id="nav-boot",
                sequence=1,
                control_mode=int(NavigationControlMode.AUTONOMY),
                lifecycle_state=int(NavigationLifecycle.EXECUTING),
                planning_state=int(NavigationPlanningState.READY),
                execution_state=int(NavigationExecutionState.FOLLOWING),
                authority="autonomy",
            )
        )
        events = []
        while not queue.empty():
            events.append(queue.get_nowait())
    finally:
        unsubscribe(gateway, queue)

    assert [event["type"] for event in events] == ["navigation_status"]
    assert events[0]["data"]["task"]["state"] == "UNKNOWN"
    assert set(events[0]["data"]) == {
        "schema_version",
        "task",
        "goal_admission",
        "control",
        "motion",
        "ts",
    }
























@pytest.mark.parametrize("local_planner", ["scan", "cmu"])
def test_native_endpoint_readiness_requires_product_control_mode_and_cmd_vel_publish(
    monkeypatch,
    tmp_path,
    local_planner,
):
    from gateway.navigation.status import _native_endpoint_readiness

    status_path = tmp_path / "nav_endpoint_status.json"
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(status_path))
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "30")

    def write_status(
        control_mode: str,
        publish_cmd_vel: bool,
        *,
        global_planner: str = "octoplanner3d",
        planner_map: str = "/maps/active/octomap.ot",
        product: str | None = None,
        far_input: dict | None = None,
        ) -> None:
        native_status = {}
        if product:
            manifest = _field_manifest(product, local_planner=local_planner)
            environment = manifest.native_process_environment
            parameter_environment = (
                ("path_follower_max_speed_mps", "LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS"),
                ("path_follower_min_speed_mps", "LINGTU_NAV_PATH_FOLLOWER_MIN_SPEED_MPS"),
                ("path_follower_max_accel_mps2", "LINGTU_NAV_PATH_FOLLOWER_MAX_ACCEL_MPS2"),
                ("path_follower_lookahead_m", "LINGTU_NAV_PATH_FOLLOWER_LOOKAHEAD_M"),
                ("path_follower_goal_tolerance_m", "LINGTU_NAV_PATH_FOLLOWER_GOAL_TOLERANCE_M"),
                ("waypoint_reached_m", "LINGTU_NAV_WAYPOINT_REACHED_M"),
                ("goal_reached_m", "LINGTU_NAV_GOAL_REACHED_M"),
                ("corridor_lookahead_m", "LINGTU_NAV_CORRIDOR_LOOKAHEAD_M"),
                ("teleop_planner_horizon_m", "LINGTU_TELEOP_PLANNER_HORIZON_M"),
                ("teleop_planner_max_deviation_deg", "LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG"),
            )
            parameters = {
                parameter: float(environment[environment_name])
                for parameter, environment_name in parameter_environment
            }
            native_status = {
                "native_product": {
                    "product": product,
                },
                "path_follower": {
                    "max_speed_mps": parameters["path_follower_max_speed_mps"],
                    "min_speed_mps": parameters["path_follower_min_speed_mps"],
                    "max_accel_mps2": parameters["path_follower_max_accel_mps2"],
                    "lookahead_m": parameters["path_follower_lookahead_m"],
                    "goal_tolerance_m": parameters["path_follower_goal_tolerance_m"],
                },
                "nav_loop": {
                    "waypoint_reached_m": parameters["waypoint_reached_m"],
                    "goal_reached_m": parameters["goal_reached_m"],
                    "corridor_lookahead_m": parameters["corridor_lookahead_m"],
                },
            }
            if manifest.native_nav["teleop_local_planner"]:
                native_status.update(
                    teleop_local_planner=True,
                    check_obstacle=True,
                    use_traversability_cost=manifest.native_nav["use_traversability_cost"],
                    teleop_planner_horizon_m=parameters["teleop_planner_horizon_m"],
                    teleop_planner_max_deviation_deg=parameters[
                        "teleop_planner_max_deviation_deg"
                    ],
                )
        if far_input is None and global_planner == "far":
            far_input = {
                "required": True,
                "ready": True,
                "reason": "ready",
                "map_id": "yard",
                "content_epoch": 7,
            }
        status_path.write_text(
            json.dumps(
                {
                    "stamp_s": time.time(),
                        "control_loop_health": {
                            "ready": True,
                            "healthy": True,
                            "reason": "healthy",
                        },
                        "navigation_ready": True,
                        "input_gate": {"ready": True, "reason": "ready"},
                    "control_mode": control_mode,
                    "operator_motion": {
                        "schema_version": 1,
                        "interface_enabled": True,
                        "authority_owner": "native_endpoint",
                        "control_mode": control_mode,
                        "allow_teleop_takeover": control_mode == "autonomy",
                        "control_ack_scope": "claim_hold_release",
                        "sample_evidence": "status_sequences",
                    },
                    "global_planner": global_planner,
                    "planner_map": planner_map,
                    **({"far_input": far_input} if far_input is not None else {}),
                    "publish_cmd_vel": publish_cmd_vel,
                    "active_cmd_source": "none",
                    "control_authority": {
                        "owner": "native_endpoint",
                        "estop_latched": False,
                        "operator_takeover_latched": False,
                        "resume_required": False,
                    },
                    **native_status,
                }
            ),
            encoding="utf-8",
        )

    write_status("teleop", False)
    blocked = _native_endpoint_readiness({"mode": "navigating"})

    assert blocked["ok"] is False
    assert blocked["blockers"] == [
        "native_control_mode_mismatch",
        "native_cmd_vel_publish_disabled",
    ]

    write_status("autonomy", True)
    ready = _native_endpoint_readiness({"mode": "navigating"})

    assert ready["ok"] is True
    assert ready["blockers"] == []
    assert ready["global_planner"] == "octoplanner3d"
    assert {"planner_map", "active_octomap", "active_occupancy"}.isdisjoint(ready)

    write_status("teleop_avoid", True, product="teleop_avoid")
    assisted_gateway = _field_gateway("teleop_avoid", local_planner=local_planner)
    assisted = _native_endpoint_readiness(
        {"mode": "navigating", "product": "teleop_avoid"},
        assisted_gateway,
    )

    assert assisted["ok"] is True
    assert assisted["expected_control_mode"] == "teleop_avoid"
    assert assisted["blockers"] == []
    assert assisted["operator_motion"]["required"] is True
    assert assisted["operator_motion"]["status_available"] is True

    assisted_status = json.loads(status_path.read_text(encoding="utf-8"))
    assert assisted_status["use_traversability_cost"] is False
    for field, actual, blocker in (
        ("use_traversability_cost", True, "native_traversability_cost_mismatch"),
        ("use_traversability_cost", None, "native_traversability_cost_mismatch"),
        ("teleop_local_planner", False, "native_teleop_local_planner_disabled"),
        ("check_obstacle", False, "native_obstacle_check_disabled"),
        ("input_gate", {"ready": False}, "native_input_gate_not_ready"),
    ):
        status_path.write_text(json.dumps({**assisted_status, field: actual}), encoding="utf-8")
        rejected = _native_endpoint_readiness(
            {"mode": "navigating", "product": "teleop_avoid"}, assisted_gateway
        )
        assert rejected["ok"] is False
        assert rejected["blockers"] == [blocker]
    status_path.write_text(json.dumps(assisted_status), encoding="utf-8")

    status_without_operator_motion = json.loads(status_path.read_text(encoding="utf-8"))
    status_without_operator_motion.pop("operator_motion")
    status_path.write_text(json.dumps(status_without_operator_motion), encoding="utf-8")
    missing_operator_motion = _native_endpoint_readiness(
        {"mode": "navigating", "product": "teleop_avoid"},
        assisted_gateway,
    )
    assert missing_operator_motion["ok"] is False
    assert missing_operator_motion["blockers"] == ["native_operator_motion_status_missing"]

    write_status("autonomy", True, product="nav")
    terrain_gateway = _field_gateway("nav", local_planner=local_planner)
    terrain_required = _native_endpoint_readiness({"mode": "navigating", "product": "nav"}, terrain_gateway)
    assert terrain_required["ok"] is True
    terrain_status = json.loads(status_path.read_text(encoding="utf-8"))
    assert terrain_status["use_traversability_cost"] is True
    terrain_status["use_traversability_cost"] = False
    status_path.write_text(json.dumps(terrain_status), encoding="utf-8")
    terrain_disabled = _native_endpoint_readiness({"mode": "navigating", "product": "nav"}, terrain_gateway)
    assert terrain_disabled["ok"] is False
    assert terrain_disabled["blockers"] == ["native_traversability_cost_disabled"]

    write_status(
        "autonomy",
        True,
        global_planner="far",
        planner_map="/maps/active/occupancy.npz",
    )
    far = _native_endpoint_readiness({"mode": "navigating", "global_planner": "far"})
    assert far["ok"] is True
    assert far["navigation_ready"] is True
    assert far["far_input"]["content_epoch"] == 7
    assert far["global_planner"] == "far"
    assert {"planner_map", "active_octomap", "active_occupancy"}.isdisjoint(far)

    write_status(
        "autonomy",
        True,
        global_planner="far",
        planner_map="/maps/active/occupancy.npz",
        far_input={},
    )
    far_missing = _native_endpoint_readiness(
        {"mode": "navigating", "global_planner": "far"}
    )
    assert far_missing["navigation_ready"] is False
    assert "native_far_input_status_missing" in far_missing["blockers"]

    write_status(
        "autonomy",
        True,
        global_planner="far",
        planner_map="/maps/active/occupancy.npz",
        far_input={
            "required": True,
            "ready": False,
            "reason": "required occupancy artifact missing",
            "map_id": "yard",
            "content_epoch": 7,
        },
    )
    far_not_ready = _native_endpoint_readiness(
        {"mode": "navigating", "global_planner": "far"}
    )
    assert far_not_ready["navigation_ready"] is False
    assert "native_far_input_not_ready" in far_not_ready["blockers"]

    write_status("autonomy", True)
    octo_without_far_status = _native_endpoint_readiness({"mode": "navigating"})
    assert octo_without_far_status["ok"] is True
    assert octo_without_far_status["far_input"] == {}

    mismatch = _native_endpoint_readiness({"mode": "navigating", "global_planner": "far"})
    assert mismatch["ok"] is False
    assert "native_global_planner_mismatch" in mismatch["blockers"]


@pytest.mark.parametrize(
    ("product", "expected_control_mode"),
    [
        ("teleop", "teleop"),
        ("teleop_avoid", "teleop_avoid"),
        ("map", "teleop"),
        ("nav", "autonomy"),
        ("explore", "autonomy"),
    ],
)
def test_native_endpoint_readiness_is_required_by_product_contract_without_endpoint_env(
    monkeypatch,
    tmp_path,
    product,
    expected_control_mode,
):
    from gateway.navigation.status import _native_endpoint_readiness

    status_path = tmp_path / "missing_nav_endpoint_status.json"
    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(status_path))
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "30")

    result = _native_endpoint_readiness(
        {"product": product},
        _field_gateway(product),
    )

    assert result["required"] is True
    assert result["ok"] is False
    assert result["status_available"] is False
    assert result["expected_control_mode"] == expected_control_mode
    assert result["blockers"] == ["native_endpoint_status_missing_or_stale"]
















def test_localizer_health_topic_recovered_marks_gateway_ready_when_icp_quality_is_zero():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    _set_session_mode(gateway, "navigating")
    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {
            "backend": "fastlio2",
            "health_source": "slam_runtime",
            "state": "TRACKING",
            "confidence": 0.91,
            "degeneracy": "NONE",
            "icp_fitness": 0.0,
            "odom_age_ms": 120.0,
            "cloud_age_ms": 90.0,
            "localizer_health": "RECOVERED",
            "localizer_health_source": "localizer_health_topic",
            "localizer_health_fitness": 0.0215,
            "relocalization_supported": True,
            "saved_map_relocalization_supported": True,
            "restart_recovery_supported": True,
            "recovery_method": "relocalize_service",
            "relocalization_state": "idle",
        }

    payload = build_localization_status(gateway)

    assert payload["state"] == "ready"
    assert payload["ready"] is True
    assert payload["algorithm_healthy"] is True
    assert payload["backend"] == "native_dds"
    assert payload["algorithm_profile"] == "fastlio2"
    assert payload["localizer_health"] == "RECOVERED"
    assert payload["localizer_health_source"] == "localizer_health_topic"
    assert payload["localizer_health_fitness"] == 0.0215
    assert payload["relocalization_supported"] is True
    assert payload["saved_map_relocalization_supported"] is True
    assert payload["restart_recovery_supported"] is True
    assert payload["recovery_method"] == "relocalize_service"
    assert payload["relocalization_state"] == "idle"
    assert payload["reasons"] == []












def test_navigation_status_route_returns_stable_schema():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/navigation/status")
    with gateway._state_lock:
        gateway._navigation_state = {"lifecycle_state_name": "IDLE", "ts": time.time()}

    payload = asyncio.run(endpoint())

    assert payload["schema_version"] == 3
    assert payload["task"]["state"] == "IDLE"
    assert payload["goal_admission"] == {
        "state": "BLOCKED",
        "reason": "navigation_session_inactive",
    }
    assert set(payload) == {
        "schema_version",
        "task",
        "goal_admission",
        "control",
        "motion",
        "ts",
    }


def test_fallback_navigation_gate_rejects_stale_odometry() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.navigation.status import evaluate_navigation_gate

    gateway = GatewayModule()
    _set_session_mode(gateway, "navigating")
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0}
        gateway._localization_status = {
            "state": "TRACKING",
            "odom_age_ms": 2500.0,
        }

    gate = evaluate_navigation_gate(gateway)

    assert gate["can_accept_goal"] is None
    assert gate["reason"] == "odometry_stale"


def test_navigation_status_routes_pass_fastapi_response_validation():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._navigation_state = {"lifecycle_state_name": "IDLE", "ts": time.time()}

    client = TestClient(gateway._app)
    response = client.get("/api/v1/navigation/status")

    assert response.status_code == 200
    payload = response.json()
    assert payload["schema_version"] == 3
    assert payload["task"] == {"state": "IDLE", "task_id": "", "reason": ""}
    assert payload["goal_admission"]["state"] == "BLOCKED"
    assert payload["goal_admission"]["reason"] == "navigation_session_inactive"
    assert set(payload) == {
        "schema_version",
        "task",
        "goal_admission",
        "control",
        "motion",
        "ts",
    }
    assert client.get("/api/v1/navigation").status_code == 404


def test_drift_watchdog_reports_missing_product() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    events = []
    gateway.push_event = events.append
    with gateway._state_lock:
        gateway._odom = {"x": 999.0}
        gateway._odom_timestamps.append(123.0)

    reported = gateway._drift_report_divergence(xy=999.0, y_abs=0.0, v=0.0)

    assert reported is True
    assert gateway._odom is None
    assert gateway._odom_timestamps == []
    assert events[-1]["action"] == "product_switch_unavailable"
    assert events[-1]["reason"] == "current_product_missing"
    assert "operator_command" not in events[-1]


def test_native_product_drift_report_requires_operator_control() -> None:
    from gateway.gateway_module import GatewayModule

    plan = _field_manifest("nav")
    gateway = GatewayModule(run_plan=plan)
    events = []
    gateway.push_event = events.append
    with gateway._state_lock:
        gateway._odom = {"x": 999.0}
        gateway._odom_timestamps.append(123.0)

    reported = gateway._drift_report_divergence(xy=999.0, y_abs=0.0, v=0.0)

    assert reported is True
    assert gateway._odom is None
    assert gateway._odom_timestamps == []
    assert events[-1]["action"] == "operator_product_switch_required"
    assert events[-1]["reason"] == "operator_product_control_required"
    assert events[-1]["current_product"] == "nav"
    assert "operator_command" not in events[-1]


def test_drift_watchdog_report_noops_after_shutdown() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule(run_plan=_field_manifest("nav"))
    gateway._stop_event.set()

    reported = gateway._drift_report_divergence(xy=999.0, y_abs=0.0, v=0.0)

    assert reported is False


def test_runtime_dataflow_route_exposes_product_runtime_observability(monkeypatch):
    from gateway.schemas import RuntimeDataflowResponse
    from message.topics import TOPICS
    from runtime.msgs.nav import Odometry

    monkeypatch.setenv("LINGTU_PROFILE", "nav")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "field")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "driver")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "0")

    gateway = _field_gateway("nav")
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow")
    payload = asyncio.run(endpoint())
    RuntimeDataflowResponse.model_validate(payload)
    initial_payload = payload

    assert payload["schema_version"] == 1
    assert payload["runtime_contract"] == "real"
    assert payload["runtime_boundary"]["runtime_contract"] == "real"
    assert "ros2_topic_required" not in payload
    assert payload["transport_layers"]["native_dds"]["primary"] is True
    assert payload["transport_layers"]["module_port_bus"]["primary"] is False
    assert "ros2_adapter" not in payload["transport_layers"]
    assert payload["motion_path"]["motion_owner"] == "nav"
    assert payload["motion_path"]["final_velocity_writer"] == "nav"
    assert payload["motion_path"]["actuator_owner"] == "driver"
    assert [stage["owner"] for stage in payload["motion_path"]["stages"]] == [
        "host",
        "slam",
        "traversability",
        "nav",
        "driver",
    ]
    assert payload["motion_path"]["control_paths"] == {
        "autonomy_goal": [
            "CommandIngress",
            "GlobalPlanner",
            "LocalPlanner",
            "PathFollower",
            "CommandSafety",
        ],
        "external_path": [
            "PathIngress",
            "LocalPlanner",
            "PathFollower",
            "CommandSafety",
        ],
        "teleop_avoid": [
            "OperatorIntent",
            "LocalPlanner",
            "PathFollower",
            "CommandSafety",
        ],
        "teleop": ["OperatorCommand", "CommandSafety"],
    }
    assert payload["control_boundary"]["arbitrary_publish_supported"] is False
    assert payload["control_boundary"]["policy"] == "whitelisted_gateway_commands_only"

    gateway_ports = payload["module_ports"]["GatewayModule"]
    assert "odometry" in gateway_ports["ports_in"]
    assert "instruction" in gateway_ports["ports_out"]
    assert "cmd_vel" not in gateway_ports["ports_out"]

    topics = {item["topic"]: item for item in payload["topics"]}
    assert TOPICS.odometry in topics
    assert TOPICS.cmd_vel in topics

    odometry_observability = topics[TOPICS.odometry]["observability"]
    assert "ros2_topic_required" not in odometry_observability
    assert "module_port_bus" in odometry_observability["observable_via"]
    assert "gateway_sse" in odometry_observability["observable_via"]
    assert "gateway_rest" in odometry_observability["observable_via"]
    assert odometry_observability["live_module_samples"] is False

    gateway.odometry._deliver(Odometry())
    payload = asyncio.run(endpoint())
    topics = {item["topic"]: item for item in payload["topics"]}
    odometry_observability = topics[TOPICS.odometry]["observability"]
    assert odometry_observability["live_module_samples"] is True
    assert odometry_observability["has_fresh_module_sample"] is True
    assert odometry_observability["module_port_candidates"][0]["msg_count"] > 0
    assert odometry_observability["module_port_candidates"][0]["stale_ms"] >= 0

    stages = {stage["name"]: stage for stage in initial_payload["stage_evidence"]}
    assert "global_planning" in stages
    assert stages["global_planning"]["owner"] == "lingtu_navigation_or_planner_backend"
    assert TOPICS.odometry in stages["global_planning"]["inputs"]
    assert TOPICS.global_path in stages["global_planning"]["outputs"]
    assert TOPICS.odometry in stages["global_planning"]["not_live_inputs"]
    odom_stage_input = next(
        item for item in stages["global_planning"]["input_evidence"] if item["token"] == TOPICS.odometry
    )
    assert odom_stage_input["observable"] is True
    assert odom_stage_input["live"] is False
    assert odom_stage_input["reason"] == "metadata_only"
    assert stages["command_boundary"]["output_evidence"][0]["kind"] == "runtime_boundary"
    assert stages["command_boundary"]["output_evidence"][0]["reason"] == "runtime_boundary_declared"

    cmd_vel_communication = topics[TOPICS.cmd_vel]["communication"]
    assert cmd_vel_communication["allowed"] is False
    assert cmd_vel_communication["arbitrary_publish_supported"] is False
    assert cmd_vel_communication["interfaces"] == []
    assert (
        cmd_vel_communication["policy"]
        == "read_only_observation_or_endpoint_adapter_owned"
    )


@pytest.mark.parametrize(
    ("product", "variant"),
    [
        ("teleop", None),
        ("teleop_avoid", None),
        ("map", None),
        ("nav", None),
        ("tracking", None),
        ("inspection", None),
        ("explore", "live"),
        ("explore", "map"),
    ],
)
def test_runtime_dataflow_covers_each_product_run_plan(
    monkeypatch,
    product: str,
    variant: str | None,
) -> None:
    from diagnostics.runtime_contract import runtime_data_flow_topics
    from gateway.services.runtime_dataflow import build_runtime_dataflow_snapshot
    from message.topics import TOPICS

    monkeypatch.setenv("LINGTU_DATA_SOURCE", "field")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "driver")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "0")

    gateway = _field_gateway(product, variant=variant)
    snapshot = build_runtime_dataflow_snapshot(gateway)
    declared = set(gateway._compiled_run_plan.required_topics)
    topics = {item["topic"]: item for item in snapshot["topics"]}
    expected = set(runtime_data_flow_topics("real")) & declared

    assert expected <= set(topics)
    assert set(topics) - declared == {TOPICS.robot_joint_states}
    assert topics[TOPICS.robot_joint_states]["inspection"]["live"] is False
    assert topics
    for topic, item in topics.items():
        assert item["topic"] == topic
        assert item["required_by_product"] is (topic in declared)
        assert "ros2_topic_required" not in item["inspection"]
        assert item["inspection"]["arbitrary_publish_supported"] is False
        assert item["communication"]["arbitrary_publish_supported"] is False


def test_runtime_dataflow_route_validates_active_saved_octomap_artifact(
    monkeypatch,
    tmp_path,
):
    from gateway.schemas import RuntimeDataflowResponse
    from message.topics import TOPICS

    map_root = tmp_path / "maps"
    _write_active_same_source_octomap(map_root)
    monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = _field_gateway("nav")
    gateway._map_client = _FilesystemMapdClient(map_root)
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow")
    payload = asyncio.run(endpoint())
    RuntimeDataflowResponse.model_validate(payload)

    stages = {stage["name"]: stage for stage in payload["stage_evidence"]}
    global_stage = stages["global_planning"]
    octomap_input = next(item for item in global_stage["input_evidence"] if item["token"] == "artifact:octomap")

    assert "artifact:octomap" in global_stage["inputs"]
    assert "artifact:octomap" not in global_stage["missing_inputs"]
    assert octomap_input["kind"] == "artifact"
    assert octomap_input["observable"] is True
    assert octomap_input["live"] is False
    assert octomap_input["reason"] == "saved_map_artifact_ok"
    assert octomap_input["artifact_gate"]["ok"] is True
    assert octomap_input["artifact_gate"]["map_id"] == "demo"
    assert "map_dir" not in octomap_input["artifact_gate"]
    assert "map_root" not in octomap_input["artifact_gate"]
    assert octomap_input["artifact_gate"]["artifacts"]["octomap"]["exists"] is True
    assert octomap_input["artifact_gate"]["artifacts"]["octomap"]["format_ok"] is True
    assert "ros2_topic_required" not in octomap_input["artifact_gate"]
    assert TOPICS.global_path in global_stage["outputs"]


def test_runtime_dataflow_route_marks_missing_active_octomap_artifact(
    monkeypatch,
    tmp_path,
):
    map_root = tmp_path / "maps"
    map_root.mkdir()
    monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = _field_gateway("nav")
    gateway._map_client = _FilesystemMapdClient(map_root)
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow")
    payload = asyncio.run(endpoint())

    stages = {stage["name"]: stage for stage in payload["stage_evidence"]}
    global_stage = stages["global_planning"]
    octomap_input = next(item for item in global_stage["input_evidence"] if item["token"] == "artifact:octomap")

    assert "artifact:octomap" in global_stage["missing_inputs"]
    assert octomap_input["kind"] == "artifact"
    assert octomap_input["observable"] is False
    assert octomap_input["live"] is False
    assert octomap_input["reason"] == "saved_map_artifact_missing_or_invalid"
    assert octomap_input["artifact_gate"]["ok"] is False
    assert "ros2_topic_required" not in octomap_input["artifact_gate"]
    assert "active map unavailable from mapd" in octomap_input["artifact_gate"]["blockers"]


def test_runtime_dataflow_route_is_read_only_for_module_ports(monkeypatch):
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow")
    before = {name: port.msg_count for name, port in gateway.ports_out.items()}

    payload = asyncio.run(endpoint())

    after = {name: port.msg_count for name, port in gateway.ports_out.items()}
    assert "ros2_topic_required" not in payload
    assert after == before


def test_runtime_dataflow_route_does_not_mark_stale_port_as_live(
    monkeypatch,
):
    import gateway.services.runtime_dataflow as dataflow_mod
    from message.topics import TOPICS
    from runtime.msgs.nav import Odometry

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setattr(dataflow_mod, "LIVE_MODULE_SAMPLE_STALE_MS", -1.0)

    gateway = _field_gateway("nav")
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow")
    gateway.odometry._deliver(Odometry())

    payload = asyncio.run(endpoint())
    topics = {item["topic"]: item for item in payload["topics"]}
    odometry_observability = topics[TOPICS.odometry]["observability"]

    assert odometry_observability["module_port_candidates"][0]["msg_count"] > 0
    assert odometry_observability["has_fresh_module_sample"] is False
    assert odometry_observability["live_module_samples"] is False


def test_runtime_dataflow_topic_route_answers_one_stream_without_ros2(monkeypatch):
    from gateway.schemas import RuntimeDataflowTopicDetailResponse
    from message.topics import TOPICS
    from runtime.msgs.nav import Odometry

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = _field_gateway("nav")
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/topic")
    gateway.odometry._deliver(Odometry())

    payload = asyncio.run(endpoint(topic="odometry"))
    RuntimeDataflowTopicDetailResponse.model_validate(payload)

    assert payload["ok"] is True
    assert payload["selector"] == "odometry"
    assert payload["topic"]["topic"] == TOPICS.odometry
    assert payload["inspection"]["live"] is True
    assert payload["inspection"]["observation_level"] == "fresh_module_sample"
    assert "ros2_topic_required" not in payload["inspection"]
    assert payload["inspection"]["arbitrary_publish_supported"] is False
    assert payload["inspection"]["payload_available"] is True
    assert {channel["transport"] for channel in payload["inspection"]["payload_interfaces"]} >= {
        "gateway_rest",
        "gateway_sse",
    }
    assert payload["inspection"]["stream_interfaces"] == [
        {
            "transport": "gateway_sse",
            "path": "/api/v1/events",
            "query": {"topic": TOPICS.odometry},
            "event_type": "odometry",
        }
    ]
    assert payload["inspection"]["communicate"] is False


def test_runtime_dataflow_subscribe_route_returns_read_only_sse_plan(monkeypatch):
    from gateway.schemas import (
        RuntimeDataflowSubscribeRequest,
        RuntimeDataflowSubscribeResponse,
    )
    from message.topics import TOPICS
    from runtime.msgs.nav import Odometry

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = _field_gateway("nav")
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/subscribe")
    gateway.odometry._deliver(Odometry())

    payload = asyncio.run(endpoint(RuntimeDataflowSubscribeRequest(selector="odometry")))
    RuntimeDataflowSubscribeResponse.model_validate(payload)

    assert payload["ok"] is True
    assert payload["read_only"] is True
    assert "ros2_topic_required" not in payload
    assert payload["arbitrary_publish_supported"] is False
    assert payload["publishes"] == []
    assert payload["selector"] == "odometry"
    assert payload["topic"] == TOPICS.odometry
    assert payload["event_types"] == ["odometry"]
    assert payload["stream_url"] == "/api/v1/events?topic=%2Fslam%2Fodometry"
    assert payload["stream_interfaces"] == [
        {
            "transport": "gateway_sse",
            "path": "/api/v1/events",
            "query": {"topic": TOPICS.odometry},
            "event_type": "odometry",
        }
    ]
    assert payload["blockers"] == []


def test_runtime_dataflow_subscribe_route_rejects_unknown_selector_without_publish(
    monkeypatch,
):
    from gateway.schemas import RuntimeDataflowSubscribeRequest

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = _field_gateway("nav")
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/subscribe")

    payload = asyncio.run(endpoint(RuntimeDataflowSubscribeRequest(selector="not_a_stream")))

    assert payload["ok"] is False
    assert payload["read_only"] is True
    assert "ros2_topic_required" not in payload
    assert payload["arbitrary_publish_supported"] is False
    assert payload["publishes"] == []
    assert payload["stream_url"] == ""
    assert "runtime_topic_not_found" in payload["blockers"]


def test_runtime_dataflow_topic_route_accepts_canonical_stream_token(
    monkeypatch,
):
    from message.topics import TOPICS
    from runtime.msgs.nav import Odometry

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = _field_gateway("nav")
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/topic")
    gateway.odometry._deliver(Odometry())

    payload = asyncio.run(endpoint(topic=TOPICS.odometry))

    assert payload["ok"] is True
    assert payload["selector"] == TOPICS.odometry
    assert payload["topic"]["topic"] == TOPICS.odometry
    assert payload["inspection"]["live"] is True
    assert "ros2_topic_required" not in payload["inspection"]


def test_runtime_dataflow_topic_route_exposes_whitelisted_command_interfaces(
    monkeypatch,
):
    from message.topics import TOPICS

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = _field_gateway("nav")
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/topic")

    payload = asyncio.run(endpoint(topic=TOPICS.nav_command_request))

    assert payload["ok"] is True
    assert payload["topic"]["topic"] == TOPICS.nav_command_request
    assert payload["inspection"]["communicate"] is True
    assert payload["inspection"]["arbitrary_publish_supported"] is False
    assert {item["path"] for item in payload["inspection"]["write_interfaces"]} == {
        "/api/v1/goal",
        "/api/v1/navigate/click",
        "/api/v1/navigation/cancel",
        "/api/v1/stop",
    }
    interfaces = payload["inspection"]["write_interfaces"]
    stop_interface = next(item for item in interfaces if item["path"] == "/api/v1/stop")
    assert stop_interface["final_output_confirmed"] is False
    assert stop_interface["response_evidence"] == "command_ack_not_driver_execution"
    assert all(
        channel.get("event_type") != "command_ack" for channel in payload["topic"]["observability"]["gateway_channels"]
    )


def test_runtime_dataflow_topic_route_reports_unknown_selector(monkeypatch):
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = _field_gateway("nav")
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/topic")

    payload = asyncio.run(endpoint(topic="not_a_lingtu_stream"))

    assert payload["ok"] is False
    assert payload["error"] == "runtime_topic_not_found"
    assert payload["inspection"]["observable"] is False
    assert payload["inspection"]["communicate"] is False
    assert "ros2_topic_required" not in payload["inspection"]
    assert "/slam/odometry" in payload["available_topics"]


@pytest.mark.skipif(not _NUMPY_IMPORT_SAFE, reason=NUMPY_UNSAFE_REASON)
def test_runtime_dataflow_reports_live_samples_for_field_topics(monkeypatch):
    import numpy as np

    from message.topics import TOPICS
    from runtime.msgs.map import MapSceneFrame
    from runtime.msgs.nav import Odometry, Path
    from runtime.msgs.sensor import PointCloud2

    class LiveNativeNav:
        @staticmethod
        def port_summary():
            return {
                "module": "NativeNav",
                "running": True,
                "ports_in": {},
                "ports_out": {
                    "cmd_vel": {
                        "type": "Twist",
                        "msg_count": 1,
                        "rate_hz": 10.0,
                        "stale_ms": 0.0,
                    }
                },
            }

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = _field_gateway("nav")
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow")

    gateway.odometry._deliver(Odometry())
    gateway.map_scene._deliver(
        MapSceneFrame(
            frame_id="map",
            source="maps.scene",
            layers=[
                {
                    "id": "maps.live_cloud",
                    "type": "pointcloud",
                    "payload": PointCloud2.from_numpy(
                        np.zeros((1, 3), dtype=np.float32),
                        frame_id="map",
                    ),
                }
            ],
        )
    )
    gateway.global_path._deliver([np.array([0.0, 0.0, 0.0])])
    gateway.local_path._deliver(Path())
    gateway._all_modules = {"native_nav": LiveNativeNav()}

    payload = asyncio.run(endpoint())
    topics = {item["topic"]: item for item in payload["topics"]}

    live_topics = (
        TOPICS.odometry,
        TOPICS.maps_scene,
        TOPICS.global_path,
        TOPICS.local_path,
        TOPICS.cmd_vel,
    )
    for topic in live_topics:
        observability = topics[topic]["observability"]
        assert observability["has_fresh_module_sample"] is True, topic
        assert observability["live_module_samples"] is True, topic
        assert "ros2_topic_required" not in observability, topic
        assert any(port["msg_count"] > 0 for port in observability["module_port_candidates"]), topic

    assert topics[TOPICS.cmd_vel]["communication"]["arbitrary_publish_supported"] is False


def test_runtime_dataflow_subscribe_route_covers_every_gateway_realtime_stream(
    monkeypatch,
):
    from gateway.schemas import RuntimeDataflowSubscribeRequest

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = _field_gateway("nav")
    gateway.setup()
    routes = {route.path: route.endpoint for route in gateway._app.routes if hasattr(route, "endpoint")}
    dataflow_endpoint = routes["/api/v1/runtime/dataflow"]
    subscribe_endpoint = routes["/api/v1/runtime/dataflow/subscribe"]

    snapshot = asyncio.run(dataflow_endpoint())
    stream_topics = [
        topic["topic"]
        for topic in snapshot["topics"]
        if any(stream.get("transport") == "gateway_sse" for stream in topic["inspection"].get("stream_interfaces", []))
    ]

    assert stream_topics
    for topic in stream_topics:
        payload = asyncio.run(subscribe_endpoint(RuntimeDataflowSubscribeRequest(selector=topic)))

        assert payload["ok"] is True, topic
        assert payload["read_only"] is True, topic
        assert "ros2_topic_required" not in payload, topic
        assert payload["arbitrary_publish_supported"] is False, topic
        assert payload["publishes"] == [], topic
        assert payload["topic"] == topic
        assert payload["transport"] == "gateway_sse", topic
        assert payload["stream_url"] == f"/api/v1/events?topic={topic.replace('/', '%2F')}", topic
        assert payload["event_types"], topic


def test_localization_status_preserves_relocalization_overlap_evidence():
    from gateway.services.runtime_status import build_localization_status_from_parts

    diagnostics = {
        "state": "TRACKING",
        "confidence": 0.9,
        "health_source": "slam_runtime",
        "relocalization_refine_backend": "fixed_transform_seed_check",
        "relocalization_refine_iterations": 0,
        "relocalization_refine_inliers": 292,
        "relocalization_refine_input_points": 485,
        "relocalization_refine_evaluated_points": 298,
        "relocalization_min_inliers": 30,
        "relocalization_min_evaluated_points": 100,
        "relocalization_refine_support_ratio": 298 / 485,
        "relocalization_refine_overlap_inlier_ratio": 292 / 298,
        "relocalization_refine_converged": True,
        "relocalization_refine_pos_cov_trace": 0.01,
    }

    payload = build_localization_status_from_parts(
        odometry={"x": 0.0, "y": 0.0, "frame_id": "odom"},
        session={"mode": "navigating", "localizer_ready": True},
        icp_quality=0.01,
        status=diagnostics,
    )

    assert payload["relocalization_refine_backend"] == "fixed_transform_seed_check"
    assert payload["relocalization_refine_iterations"] == 0
    assert payload["relocalization_refine_inliers"] == 292
    assert payload["relocalization_refine_input_points"] == 485
    assert payload["relocalization_refine_evaluated_points"] == 298
    assert payload["relocalization_min_inliers"] == 30
    assert payload["relocalization_min_evaluated_points"] == 100
    assert payload["relocalization_refine_support_ratio"] == pytest.approx(298 / 485)
    assert payload["relocalization_refine_overlap_inlier_ratio"] == pytest.approx(292 / 298)
    assert payload["relocalization_refine_converged"] is True

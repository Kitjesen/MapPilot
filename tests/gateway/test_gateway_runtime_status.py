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


def _field_manifest(profile: str):
    from lingtu.assembly.compiler import compile_run_plan
    from lingtu.assembly.products import resolve_product_host_runtime

    resolved = resolve_product_host_runtime(profile, "real", robot="unitree/go2")
    return compile_run_plan(
        resolved.product,
        resolved.env,
        robot="unitree/go2",
    )


def _field_gateway(profile: str):
    from gateway.gateway_module import GatewayModule

    return GatewayModule(run_plan=_field_manifest(profile))


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

    gateway._session_mode = "navigating"
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


def test_navigation_frame_summary_rejects_incomplete_map_odom_tf() -> None:
    from gateway.services.runtime_status import _navigation_frame_summary

    summary = _navigation_frame_summary(
        {
            "planning_frame_id": "map",
            "costmap_frame_id": "map",
            "map_odom_tf": {
                "valid": True,
                "frame_id": "map",
                "child_frame_id": "odom",
            },
        },
        {"frame_id": "odom"},
    )

    assert summary["has_map_odom_tf"] is False
    assert summary["map_odom_tf"] is None
    assert summary["mismatches"] == [
        {
            "source": "odometry",
            "expected_frame": "map",
            "received_frame": "odom",
        }
    ]


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


def test_localization_status_reports_runtime_boundary_and_topic_frames(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocalizationStatusResponse
    from gateway.services.runtime_status import build_localization_status

    monkeypatch.setenv("LINGTU_PROFILE", "nav")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "field")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "driver")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "0")

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
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
    gateway._session_mode = "mapping"
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




def test_navigation_status_prefers_native_navigation_state() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status
    from runtime.msgs.nav import NavigationState

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "vx": 0.0, "vy": 0.0}
        gateway._mode = "autonomous"
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "icp_fitness": 0.03,
        }
    gateway._on_navigation_state(
        NavigationState(
            ts=123.0,
            frame_id="map",
            boot_id="navd-boot",
            sequence=4,
            control_mode=1,
            lifecycle_state=2,
            active_task_id="navigation-task-3",
            active_request_id="goal-3",
            goal_epoch=3,
            map_id="site-a",
            map_content_epoch=2,
            planning_state=2,
            execution_state=1,
            recovery_state=0,
            progress=0.6,
            authority="autonomy",
        )
    )

    payload = build_navigation_status(gateway)

    assert payload["state_source"] == "native_navigation_state"
    assert payload["state"] == "EXECUTING"
    assert payload["failure_reason"] == ""
    assert payload["progress"]["fraction"] == pytest.approx(0.6)
    assert payload["control"]["command_owner"] == "autonomy"
    assert payload["navigation_state"]["boot_id"] == "navd-boot"
    assert payload["mission"]["raw"]["active_task_id"] == "navigation-task-3"
    assert payload["mission"]["raw"]["active_request_id"] == "goal-3"


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


def test_navigation_status_reports_current_runtime_boundary(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationStatusResponse
    from gateway.services.runtime_status import build_navigation_status

    monkeypatch.setenv("LINGTU_PROFILE", "nav")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "field")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "driver")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "0")

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "frame_id": "map"}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "EXECUTING",
            "planning_frame_id": "map",
            "costmap_frame_id": "map",
            "goal_frame_id": "map",
        }
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}

    payload = build_navigation_status(gateway)
    NavigationStatusResponse.model_validate(payload)

    runtime = payload["runtime"]
    assert runtime["ok"] is True
    assert runtime["declared"] is True
    assert runtime["profile"] == "nav"
    assert runtime["endpoint"] == "thunder_dds"
    assert runtime["data_source"] == "field"
    assert runtime["runtime_contract"] == "real"
    assert runtime["simulation_only"] is False
    assert runtime["command_sink"] == "driver"
    assert runtime["expected_command_sink"] == "driver"
    assert runtime["slam_source"] == "lingtu_fastlio_or_external_robot_slam"
    assert runtime["localization_source"] == "slam_localizer"
    assert runtime["mapping_source"] == "slam_map_cloud"
    assert runtime["frames"]["map"] == "map"
    assert runtime["frames"]["odom"] == "odom"
    assert runtime["frames"]["body"] == "body"
    assert runtime["frames"]["axis_convention"] == "x_forward_y_left_z_up"
    assert runtime["frame_links"]["body_to_lidar"] == {
        "parent": "body",
        "child": "lidar_link",
        "required": True,
    }
    assert runtime["topic_allowed_frame_ids"]["/slam/map_cloud"] == ["map"]
    assert runtime["topic_allowed_frame_ids"]["/nav/global_path"] == ["map"]
    assert runtime["topic_default_frame_ids"]["/slam/map_cloud"] == "map"
    assert runtime["topic_default_frame_ids"]["/nav/cmd_vel"] == "body"
    assert runtime["required_topic_frame_ids"] == [
        "/lidar/raw_frame",
        "/imu/raw",
        "/slam/odometry",
        "/slam/registered_cloud",
        "/slam/map_cloud",
        "/nav/global_path",
        "/nav/local_path",
        "/nav/cmd_vel",
    ]
    assert runtime["runtime_data_flow_topics"] == [
        "/lidar/raw_frame",
        "/imu/raw",
        "/slam/odometry",
        "/slam/registered_cloud",
        "/slam/map_cloud",
        "/slam/localization_health",
        "/slam/localization_quality",
        "/nav/exploration_grid",
        "/nav/terrain_map_ext",
        "/exploration/way_point",
        "/nav/command/request",
        "/nav/global_path",
        "/nav/way_point",
        "/nav/terrain_map",
        "/nav/traversability",
        "/nav/local_path",
        "/nav/local_planner/control_hint",
        "/nav/cmd_vel",
        "/nav/added_obstacles",
        "/nav/check_obstacle",
        "/nav/planner_status",
    ]
    flow = {stage["name"]: stage for stage in runtime["resolved_runtime_data_flow"]}
    assert list(flow["endpoint_adapter"]["inputs"]) == [
        "/lidar/raw_frame",
        "/imu/raw",
    ]
    assert list(flow["command_boundary"]["outputs"]) == ["driver"]
    assert runtime["runtime_data_flow_stage_algorithm_interfaces"]["global_planning"] == [
        "global_planning",
        "octoplanner3d_global_planning",
    ]
    assert runtime["runtime_data_flow_stage_algorithm_interfaces"]["local_planning_and_following"] == [
        "local_planning_and_following"
    ]


def test_navigation_frame_summary_defaults_planning_frame_from_runtime_contract(
    monkeypatch,
):
    import runtime.runtime_interface as runtime_interface
    from gateway.services import runtime_status

    calls: list[tuple[str | None, str]] = []

    def fake_runtime_topic_default_frame_id(
        runtime_contract: str | None,
        topic: str,
    ) -> str:
        calls.append((runtime_contract, topic))
        return "contract_map"

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setattr(
        runtime_interface,
        "runtime_topic_default_frame_id",
        fake_runtime_topic_default_frame_id,
    )

    summary = runtime_status._navigation_frame_summary({}, None)

    assert summary["planning_frame_id"] == "contract_map"
    assert summary["ok"] is True
    assert calls == [("real", runtime_interface.TOPICS.global_path)]


def test_navigation_status_flags_runtime_boundary_mismatch(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    monkeypatch.setenv("LINGTU_PROFILE", "explore")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "mujoco_fastlio2_live")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "driver")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "1")

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "frame_id": "map"}
        gateway._mission = {"state": "EXECUTING", "planning_frame_id": "map"}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}

    runtime = build_navigation_status(gateway)["runtime"]

    assert runtime["ok"] is False
    assert "runtime_contract_data_source_mismatch" in runtime["blockers"]
    assert "command_sink_mismatch" in runtime["blockers"]
    assert runtime["expected_command_sink"] == "mujoco_velocity_adapter"
    assert runtime["topic_allowed_frame_ids"]["/slam/map_cloud"] == ["map"]
    assert runtime["required_topic_frame_ids"] == [
        "/lidar/raw_frame",
        "/imu/raw",
        "/slam/odometry",
        "/slam/registered_cloud",
        "/slam/map_cloud",
        "/nav/global_path",
        "/nav/local_path",
        "/nav/cmd_vel",
    ]
    assert runtime["runtime_data_flow_topics"][:2] == ["/lidar/raw_frame", "/imu/raw"]


def test_navigation_status_reports_sim_runtime_topic_frames(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationStatusResponse
    from gateway.services.runtime_status import build_navigation_status

    monkeypatch.setenv("LINGTU_PROFILE", "explore")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "mujoco_fastlio2_live")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "mujoco_fastlio2_live")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "mujoco_velocity_adapter")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "1")

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "frame_id": "odom"}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "EXECUTING",
            "planning_frame_id": "map",
            "costmap_frame_id": "map",
            "goal_frame_id": "map",
        }
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}

    payload = build_navigation_status(gateway)
    NavigationStatusResponse.model_validate(payload)

    runtime = payload["runtime"]
    assert runtime["ok"] is True
    assert runtime["topic_allowed_frame_ids"]["/slam/map_cloud"] == ["map", "odom"]
    assert runtime["topic_allowed_frame_ids"]["/nav/global_path"] == ["map", "odom"]
    assert runtime["required_topic_frame_ids"] == []
    assert runtime["runtime_data_flow_topics"][:2] == ["/lidar/raw_frame", "/imu/raw"]


def test_navigation_status_flags_unknown_topic_frame_contract(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    monkeypatch.setenv("LINGTU_PROFILE", "nav")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "field")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "typo_contract")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "driver")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "0")

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "frame_id": "map"}
        gateway._mission = {"state": "EXECUTING", "planning_frame_id": "map"}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}

    runtime = build_navigation_status(gateway)["runtime"]

    assert runtime["ok"] is False
    assert "runtime_contract_data_source_mismatch" in runtime["blockers"]
    assert "topic_frame_contract_unavailable" in runtime["blockers"]
    assert runtime["topic_allowed_frame_ids"] == {}




def test_navigation_status_blocks_goal_on_odometry_frame_mismatch():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationStatusResponse
    from gateway.services.runtime_status import build_navigation_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "frame_id": "camera_link"}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "IDLE",
            "planning_frame_id": "map",
            "costmap_frame_id": "map",
        }
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    gateway._all_modules = {}

    payload = build_navigation_status(gateway)
    NavigationStatusResponse.model_validate(payload)

    assert payload["frames"]["ok"] is False
    assert payload["frames"]["mismatches"] == [
        {
            "source": "odometry",
            "expected_frame": "map",
            "received_frame": "camera_link",
        }
    ]
    assert "frame_mismatch_odometry" in payload["reason_codes"]
    assert "frame_mismatch_odometry" in payload["readiness"]["blockers"]
    assert payload["diagnostics"]["frame_mismatches"] == payload["frames"]["mismatches"]
    assert payload["can_accept_goal"] is False
    assert payload["readiness"]["can_execute_autonomy"] is False


def test_gateway_odometry_preserves_frame_for_navigation_status():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status
    from runtime.msgs.geometry import Pose, Quaternion, Vector3
    from runtime.msgs.nav import Odometry

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._mode = "autonomous"
        gateway._mission = {"state": "IDLE", "planning_frame_id": "map"}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}

    gateway._on_odometry(
        Odometry(
            pose=Pose(position=Vector3(1.0, 2.0, 0.0), orientation=Quaternion()),
            frame_id="odom",
            child_frame_id="base_link",
        )
    )
    payload = build_navigation_status(gateway)

    assert gateway._odom["frame_id"] == "odom"
    assert gateway._odom["child_frame_id"] == "base_link"
    assert payload["frames"]["odom_frame_id"] == "odom"
    assert "frame_mismatch_odometry" in payload["reason_codes"]
    assert payload["frames"]["mismatches"] == [
        {
            "source": "odometry",
            "expected_frame": "map",
            "received_frame": "odom",
        }
    ]


def test_gateway_navigation_status_accepts_odom_when_map_odom_tf_is_valid():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status
    from runtime.msgs.geometry import Pose, Quaternion, Vector3
    from runtime.msgs.nav import Odometry

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "IDLE",
            "planning_frame_id": "map",
            "costmap_frame_id": "map",
        }
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
                "ts": 123.0,
            },
        }

    gateway._on_odometry(
        Odometry(
            pose=Pose(position=Vector3(1.0, 2.0, 0.0), orientation=Quaternion()),
            frame_id="odom",
            child_frame_id="base_link",
        )
    )
    payload = build_navigation_status(gateway)

    assert payload["frames"]["ok"] is True
    assert payload["frames"]["mismatches"] == []
    assert payload["frames"]["odometry_expected_frame_ids"] == ["map", "odom"]
    assert payload["readiness"]["tf_ok"] is True
    assert "frame_mismatch_odometry" not in payload["reason_codes"]


def test_gateway_mission_event_pushes_navigation_status_update():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "frame_id": "map"}
        gateway._mode = "autonomous"
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    queue = subscribe(gateway)

    try:
        gateway._on_mission(
            {
                "state": "EXECUTING",
                "planning_frame_id": "map",
                "odom_frame_id": "map",
                "costmap_frame_id": "map",
            }
        )
        events = []
        while not queue.empty():
            events.append(queue.get_nowait())
    finally:
        unsubscribe(gateway, queue)

    assert [event["type"] for event in events] == ["mission", "navigation_status"]
    assert events[1]["data"]["state"] == "EXECUTING"
    assert events[1]["data"]["frames"]["ok"] is True


def test_navigation_status_reports_costmap_frame_mismatch():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationStatusResponse
    from gateway.services.runtime_status import build_navigation_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "frame_id": "map"}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "IDLE",
            "planning_frame_id": "map",
            "odom_frame_id": "map",
            "costmap_frame_id": "odom",
        }
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    gateway._all_modules = {}

    payload = build_navigation_status(gateway)
    NavigationStatusResponse.model_validate(payload)

    assert payload["frames"]["ok"] is False
    assert payload["frames"]["mismatches"] == [
        {
            "source": "costmap",
            "expected_frame": "map",
            "received_frame": "odom",
        }
    ]
    assert "frame_mismatch_costmap" in payload["reason_codes"]
    assert "frame_mismatch_costmap" in payload["readiness"]["blockers"]
    assert payload["can_accept_goal"] is False


def test_navigation_status_reads_idle_costmap_frame_from_nav_mission():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationStatusResponse
    from gateway.services.runtime_status import build_navigation_status

    class FakeNavigation:
        def get_navigation_status(self):
            return json.dumps(
                {
                    "state": "IDLE",
                    "planning_frame_id": "map",
                    "odom_frame_id": "map",
                    "costmap_frame_id": "odom",
                }
            )

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "frame_id": "map"}
        gateway._mode = "autonomous"
        gateway._mission = {"state": "IDLE", "planning_frame_id": "map"}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    gateway._all_modules = {}

    payload = build_navigation_status(gateway)
    NavigationStatusResponse.model_validate(payload)

    assert payload["frames"]["costmap_frame_id"] == "odom"
    assert payload["frames"]["ok"] is False
    assert payload["frames"]["mismatches"] == [
        {
            "source": "costmap",
            "expected_frame": "map",
            "received_frame": "odom",
        }
    ]
    assert "frame_mismatch_costmap" in payload["reason_codes"]
    assert "frame_mismatch_costmap" in payload["readiness"]["blockers"]
    assert payload["can_accept_goal"] is False


def test_navigation_status_blocks_goal_when_session_is_not_navigating():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    gateway = GatewayModule()
    gateway._session_mode = "idle"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0}
        gateway._mode = "autonomous"
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    gateway._all_modules = {}

    payload = build_navigation_status(gateway)

    assert payload["can_accept_goal"] is False
    assert "navigation_session_inactive" in payload["reason_codes"]
    assert "navigation_session_inactive" in payload["readiness"]["blockers"]
    assert payload["readiness"]["session_mode"] == "idle"
    assert payload["feedback"]["next_action"] == "resolve_blockers"


def test_navigation_status_can_disable_real_runtime_evidence_gate_for_commissioning(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_REQUIRE_REAL_RUNTIME_EVIDENCE", "0")

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.9
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "frame_id": "odom"}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "IDLE",
            "planning_frame_id": "map",
            "odom_frame_id": "odom",
        }
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "algorithm_healthy": True,
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
    gateway._all_modules = {}

    payload = build_navigation_status(gateway)

    assert "real_runtime_evidence_missing_or_stale" not in payload["reason_codes"]
    assert "real_runtime_evidence_missing_or_stale" not in payload["readiness"]["blockers"]
    assert payload["readiness"]["real_runtime_evidence_ok"] is None
    assert payload["diagnostics"]["real_runtime_evidence"]["reason"] == "disabled_for_commissioning"


def test_navigation_status_requires_passing_real_runtime_evidence(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_REQUIRE_REAL_RUNTIME_EVIDENCE", "1")
    monkeypatch.setattr(
        "gateway.routes.diagnostics.build_real_runtime_evidence_latest_summary",
        lambda: {
            "ok": False,
            "blockers": ["real-runtime-evidence real_robot_motion is not true"],
        },
    )

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.9
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "frame_id": "odom"}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "IDLE",
            "planning_frame_id": "map",
            "odom_frame_id": "odom",
        }
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "algorithm_healthy": True,
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
    gateway._all_modules = {}

    payload = build_navigation_status(gateway)

    assert payload["can_accept_goal"] is False
    assert "real_runtime_evidence_missing_or_stale" in payload["reason_codes"]
    assert payload["readiness"]["real_runtime_evidence_ok"] is False


def test_navigation_status_allows_exploring_session_for_external_tare():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    gateway = GatewayModule()
    gateway._session_mode = "exploring"
    gateway._session_slam_profile = "none"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0}
        gateway._mode = "autonomous"
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "pose_fresh": True,
            "odom_age_ms": 100.0,
        }
    gateway._all_modules = {}

    payload = build_navigation_status(gateway)

    assert payload["readiness"]["session_mode"] == "exploring"
    assert "navigation_session_inactive" not in payload["reason_codes"]
    assert "navigation_session_inactive" not in payload["readiness"]["blockers"]








def test_navigation_status_blocks_autonomy_when_pose_is_stale_but_algorithm_healthy():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.28,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 2500.0,
            "localizer_health": "RECOVERED",
        }
    gateway._all_modules = {}

    payload = build_navigation_status(gateway)

    assert payload["localization"]["algorithm_healthy"] is True
    assert payload["localization"]["pose_fresh"] is False
    assert "pose_stale" in payload["reason_codes"]
    assert "pose_stale" in payload["readiness"]["blockers"]
    assert payload["can_accept_goal"] is False
    assert payload["readiness"]["can_accept_goal"] is False
    assert payload["readiness"]["can_execute_autonomy"] is False


def test_navigation_status_allows_fresh_pose_with_low_confidence_snapshot():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.28,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 1440.0,
            "localizer_health": "RECOVERED",
        }
    gateway._all_modules = {}

    payload = build_navigation_status(gateway)

    assert payload["localization"]["algorithm_healthy"] is True
    assert payload["localization"]["pose_fresh"] is True
    assert payload["localization"]["pose_freshness"] == "fresh"
    assert "pose_stale" not in payload["reason_codes"]
    assert payload["can_accept_goal"] is True
    assert payload["readiness"]["blockers"] == []
    assert payload["readiness"]["can_accept_goal"] is True
    assert payload["readiness"]["can_execute_autonomy"] is True

    session = gateway._session_snapshot()
    assert session["localizer_ready"] is True
    assert session["pose_fresh"] is True
    assert session["pose_freshness"] == "fresh"


def test_navigation_status_blocks_when_native_input_gate_is_not_ready(
    monkeypatch,
    tmp_path,
):
    from gateway.services.runtime_status import build_navigation_status

    status_path = tmp_path / "nav_endpoint_status.json"
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(status_path))
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "30")

    gateway = _field_gateway("nav")
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "localizer_health": "RECOVERED",
            "odom_age_ms": 0.0,
        }
    gateway._all_modules = {}

    status_path.write_text(
        json.dumps(
            {
                "stamp_s": time.time(),
                "input_gate": {
                    "ready": False,
                    "reason": "localization_unhealthy",
                },
            }
        ),
        encoding="utf-8",
    )

    payload = build_navigation_status(gateway)

    assert payload["can_accept_goal"] is False
    assert "native_input_gate_not_ready" in payload["readiness"]["blockers"]
    assert payload["readiness"]["native_endpoint"]["input_gate"]["reason"] == ("localization_unhealthy")


def test_native_endpoint_readiness_requires_product_control_mode_and_cmd_vel_publish(
    monkeypatch,
    tmp_path,
):
    from gateway.services.runtime_status import _native_endpoint_readiness

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
            manifest = _field_manifest(product)
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
            if product == "teleop_avoid":
                native_status.update(
                    teleop_local_planner=True,
                    check_obstacle=True,
                    use_traversability_cost=True,
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
    assisted_gateway = _field_gateway("teleop_avoid")
    assisted = _native_endpoint_readiness(
        {"mode": "navigating", "product": "teleop_avoid"},
        assisted_gateway,
    )

    assert assisted["ok"] is True
    assert assisted["expected_control_mode"] == "teleop_avoid"
    assert assisted["blockers"] == []
    assert assisted["operator_motion"]["required"] is True
    assert assisted["operator_motion"]["status_available"] is True

    status_without_operator_motion = json.loads(status_path.read_text(encoding="utf-8"))
    status_without_operator_motion.pop("operator_motion")
    status_path.write_text(json.dumps(status_without_operator_motion), encoding="utf-8")
    missing_operator_motion = _native_endpoint_readiness(
        {"mode": "navigating", "product": "teleop_avoid"},
        assisted_gateway,
    )
    assert missing_operator_motion["ok"] is False
    assert missing_operator_motion["blockers"] == ["native_operator_motion_status_missing"]
    from gateway.services.runtime_status import _navigation_blockers

    assert _navigation_blockers(missing_operator_motion["blockers"]) == ["native_operator_motion_status_missing"]

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
    from gateway.services.runtime_status import _native_endpoint_readiness

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


def test_navigation_status_treats_diverged_slam_as_lost():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "DIVERGED",
            "confidence": 0.0,
            "reason": "fastlio_velocity_out_of_bounds",
            "fastlio_speed_mps": 2805.0,
            "odom_age_ms": 0.0,
        }
    gateway._all_modules = {}

    payload = build_navigation_status(gateway)

    assert payload["localization"]["state"] == "lost"
    assert payload["localization"]["fastlio_speed_mps"] == 2805.0
    assert "localization_lost" in payload["readiness"]["blockers"]
    assert payload["can_accept_goal"] is False


def test_navigation_status_blocks_goal_when_map_artifact_gate_fails():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeNavigation:
        def get_navigation_status(self):
            return json.dumps(
                {
                    "state": "IDLE",
                    "planning_frame_id": "map",
                    "odom_frame_id": "map",
                    "costmap_frame_id": "map",
                    "map_artifact_gate": {
                        "required": True,
                        "ok": False,
                        "blockers": ["metadata.json missing"],
                    },
                }
            )

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "frame_id": "map"}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "localizer_health": "RECOVERED",
        }
    gateway._all_modules = {}

    payload = build_navigation_status(gateway)

    assert payload["can_accept_goal"] is False
    assert payload["readiness"]["map_artifacts_ok"] is False
    assert "map_artifact_gate_failed" in payload["readiness"]["blockers"]
    assert payload["readiness"]["map_artifact_gate"]["blockers"] == ["metadata.json missing"]


def test_map_artifact_consistency_compares_map_ids_directly():
    from gateway.services.runtime_status import _with_active_map_artifact_consistency

    checked = _with_active_map_artifact_consistency(
        {"required": True, "ok": True, "map_id": "old-map", "blockers": []},
        {"active_map": "current-map"},
        {},
    )

    assert checked["ok"] is False
    assert checked["gate_map"] == "old-map"
    assert checked["active_map"] == "current-map"
    assert "map_dir" not in checked


def test_navigation_status_blocks_native_saved_map_before_relocalize():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    gateway = GatewayModule()
    gateway._icp_quality = 0.03
    gateway._session_snapshot = lambda: {
        "mode": "navigating",
        "active_map": "accept_ready",
        "localizer_ready": True,
    }
    with gateway._state_lock:
        gateway._odom = {"x": 219143.1, "y": 421310.9, "frame_id": "odom"}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "health_source": "slam_runtime",
            "backend": "fastlio2",
            "mode": "localization",
            "map_loaded": True,
            "saved_map_relocalization_supported": True,
            "relocalization_state": "idle",
            "relocalization_quality": -1.0,
            "track_against_map": {
                "enabled": True,
                "successes": 1,
                "degraded": False,
            },
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
    gateway._all_modules = {}

    payload = build_navigation_status(gateway)

    assert payload["localization"]["ready"] is True
    assert payload["can_accept_goal"] is False
    assert "saved_map_relocalization_missing" in payload["readiness"]["blockers"]
    assert "saved_map_relocalization_missing" in payload["reason_codes"]


def test_navigation_status_blocks_unhealthy_native_saved_map_tracking():
    from gateway.schemas import LocalizationStatusResponse
    from gateway.services.runtime_status import build_localization_status, build_navigation_status

    gateway = _field_gateway("nav")
    gateway._icp_quality = 0.03
    gateway._session_snapshot = lambda: {
        "mode": "navigating",
        "active_map": "accept_ready",
        "localizer_ready": True,
    }
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "frame_id": "odom"}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "health_source": "slam_runtime",
            "backend": "fastlio2",
            "mode": "localization",
            "map_loaded": True,
            "saved_map_relocalization_supported": True,
            "relocalization_state": "completed",
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
            "track_against_map": {
                "enabled": True,
                "successes": 0,
                "degraded": False,
            },
        }
    gateway._all_modules = {}

    localization_model = LocalizationStatusResponse.model_validate(build_localization_status(gateway))
    payload = build_navigation_status(gateway)

    assert localization_model.map_tracking == {
        "enabled": True,
        "successes": 0,
        "degraded": False,
    }
    assert payload["can_accept_goal"] is False
    assert "saved_map_tracking_unhealthy" in payload["reason_codes"]
    assert "saved_map_tracking_unhealthy" in payload["readiness"]["blockers"]


def test_saved_map_relocalization_gate_fails_closed_without_capability_or_commit():
    from gateway.services.runtime_status import _saved_map_relocalization_missing

    localization = {
        "active_map": "accept_ready",
        "backend": "native_dds",
        "native_mode": "localization",
        "map_loaded": True,
        "saved_map_relocalization_supported": False,
        "relocalization_state": "unsupported",
        "relocalization_quality": -1.0,
    }

    assert _saved_map_relocalization_missing(localization) is True

    localization.update(
        {
            "saved_map_relocalization_supported": True,
            "relocalization_state": "rejected",
            "relocalization_quality": 0.02,
        }
    )
    assert _saved_map_relocalization_missing(localization) is True

    localization["relocalization_state"] = "completed"
    assert _saved_map_relocalization_missing(localization) is False


def test_navigation_status_treats_mild_degeneracy_as_advisory():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE", "speed_scale": 0.7}
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
    gateway._all_modules = {}

    payload = build_navigation_status(gateway)

    assert payload["can_accept_goal"] is True
    assert payload["localization"]["ready"] is True
    assert payload["localization"]["degraded"] is False
    assert payload["localization"]["degeneracy"] == "MILD"
    assert payload["readiness"]["blockers"] == []
    assert payload["readiness"]["advisories"] == ["localization_mild_degeneracy"]
    assert payload["reason_codes"] == ["localization_mild_degeneracy"]


def test_navigation_status_uses_localizer_health_fitness_when_icp_quality_is_zero():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status, build_navigation_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.92,
            "degeneracy": "NONE",
            "icp_fitness": 0.0,
            "odom_age_ms": 150.0,
            "cloud_age_ms": 140.0,
            "localizer_health": "RECOVERED",
            "localizer_health_source": "localizer_health_topic",
            "localizer_health_fitness": 0.0223,
        }
    gateway._all_modules = {}

    localization = build_localization_status(gateway)
    navigation = build_navigation_status(gateway)
    session = gateway._session_snapshot()

    assert localization["state"] == "ready"
    assert localization["ready"] is True
    assert localization["algorithm_healthy"] is True
    assert localization["reasons"] == []
    assert navigation["can_accept_goal"] is True
    assert navigation["reason_codes"] == []
    assert session["localizer_ready"] is True


def test_localizer_health_topic_recovered_marks_gateway_ready_when_icp_quality_is_zero():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
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


def test_navigation_status_blocks_ready_when_map_cloud_is_stale():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status, build_navigation_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.92,
            "degeneracy": "NONE",
            "icp_fitness": 0.0,
            "odom_age_ms": 150.0,
            "cloud_age_ms": 140.0,
            "map_cloud_fresh": False,
            "localizer_health": "RECOVERED",
            "localizer_health_source": "localizer_health_topic",
            "localizer_health_fitness": 0.0223,
        }
    gateway._all_modules = {}

    localization = build_localization_status(gateway)
    navigation = build_navigation_status(gateway)
    session = gateway._session_snapshot()

    assert localization["state"] == "initializing"
    assert localization["ready"] is False
    assert localization["algorithm_healthy"] is False
    assert localization["reasons"] == ["localizer_not_ready"]
    assert navigation["can_accept_goal"] is False
    assert "localization_initializing" in navigation["reason_codes"]
    assert session["localizer_ready"] is False


def test_navigation_status_blocks_goal_when_native_recovery_signal_is_active():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status, build_navigation_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "backend": "native_dds",
            "state": "TRACKING",
            "confidence": 0.92,
            "health_source": "slam_runtime",
            "pose_fresh": True,
            "map_cloud_fresh": True,
            "recovery_signal": "LOC_DIVERGED",
            "recovery_action": "restart_native_dds_slam",
            "localizer_health": "LIO_TRACKING",
            "odom_age_ms": 120.0,
            "cloud_age_ms": 80.0,
        }
    gateway._all_modules = {}

    localization = build_localization_status(gateway)
    navigation = build_navigation_status(gateway)

    assert localization["state"] == "degraded"
    assert localization["ready"] is False
    assert localization["algorithm_healthy"] is False
    assert localization["reasons"] == ["recovery_signal:loc_diverged"]
    assert localization["map_save_source"] == "native_slam_dds_control"
    assert navigation["can_accept_goal"] is False
    assert "localization_recovery_active" in navigation["reason_codes"]
    assert "localization_recovery_active" in navigation["readiness"]["blockers"]


def test_goal_route_rejects_stale_localization_without_publishing():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, GoalRequest

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.28,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 2500.0,
            "localizer_health": "RECOVERED",
        }
    sent_goals = []
    gateway.goal_pose._add_callback(sent_goals.append)

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/goal")(
            GoalRequest(
                x=1.0,
                y=2.0,
                request_id="stale-goal",
                client_id="web",
            )
        )
    )
    payload = _payload(response)
    model = GatewayErrorResponse.model_validate(payload)

    assert response.status_code == 409
    assert model.error == "navigation_not_ready"
    assert model.command is not None
    assert model.command.name == "goal"
    assert model.command.accepted is False
    assert model.detail["blockers"] == ["pose_stale"]
    assert sent_goals == []






def test_navigation_status_route_returns_stable_schema():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._navigation_state = {"lifecycle_state_name": "IDLE"}

    payload = asyncio.run(_endpoint(gateway, "/api/v1/navigation/status")())

    assert payload["schema_version"] == 1
    assert payload["state"] == "IDLE"
    assert payload["path"]["endpoint"] == "/api/v1/path"
    assert payload["control"]["active_cmd_source"] == "unknown"
    assert "odometry_missing" in payload["reason_codes"]
    assert payload["readiness"]["blockers"] == [
        "odometry_missing",
        "navigation_session_inactive",
    ]


def test_navigation_status_routes_pass_fastapi_response_validation():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._mission = {"state": "IDLE"}

    client = TestClient(gateway._app)
    response = client.get("/api/v1/navigation/status")

    assert response.status_code == 200
    payload = response.json()
    assert payload["schema_version"] == 1
    assert payload["state"] == "IDLE"
    assert payload["frames"]["planning_frame_id"] == "map"
    assert payload["target"] == {
        "goal": None,
        "current_waypoint": None,
        "distance_to_goal_m": None,
        "active_waypoint_distance_m": None,
        "remaining_waypoints": None,
    }
    assert payload["motion"]["active_cmd_source"] == "unknown"
    assert payload["feedback"]["next_action"] == "resolve_blockers"
    assert payload["feedback"]["blockers"] == [
        "odometry_missing",
        "navigation_session_inactive",
    ]
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
    from runtime.msgs.nav import Odometry
    from runtime.runtime_interface import TOPICS

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
    assert "cmd_vel" in gateway_ports["ports_out"]

    topics = {item["topic"]: item for item in payload["topics"]}
    assert TOPICS.odometry in topics
    assert TOPICS.lidar_scan in topics
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

    lidar_flow = {stage["name"]: stage for stage in topics[TOPICS.lidar_scan]["data_flow_stages"]}
    assert "endpoint_adapter" in lidar_flow
    assert "input" in lidar_flow["endpoint_adapter"]["roles"]

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
    assert cmd_vel_communication["allowed"] is True
    assert cmd_vel_communication["arbitrary_publish_supported"] is False
    assert {interface["path"] for interface in cmd_vel_communication["interfaces"]} == {"/api/v1/stop"}


def test_runtime_dataflow_route_validates_active_saved_octomap_artifact(
    monkeypatch,
    tmp_path,
):
    from gateway.schemas import RuntimeDataflowResponse
    from runtime.runtime_interface import TOPICS

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
    from gateway.gateway_module import GatewayModule
    from runtime.msgs.nav import Odometry
    from runtime.runtime_interface import TOPICS

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setattr(dataflow_mod, "LIVE_MODULE_SAMPLE_STALE_MS", -1.0)

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow")
    gateway.odometry._deliver(Odometry())

    payload = asyncio.run(endpoint())
    topics = {item["topic"]: item for item in payload["topics"]}
    odometry_observability = topics[TOPICS.odometry]["observability"]

    assert odometry_observability["module_port_candidates"][0]["msg_count"] > 0
    assert odometry_observability["has_fresh_module_sample"] is False
    assert odometry_observability["live_module_samples"] is False


def test_runtime_dataflow_topic_route_answers_one_stream_without_ros2(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeDataflowTopicDetailResponse
    from runtime.msgs.nav import Odometry
    from runtime.runtime_interface import TOPICS

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = GatewayModule()
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
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import (
        RuntimeDataflowSubscribeRequest,
        RuntimeDataflowSubscribeResponse,
    )
    from runtime.msgs.nav import Odometry
    from runtime.runtime_interface import TOPICS

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = GatewayModule()
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
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeDataflowSubscribeRequest

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = GatewayModule()
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
    from gateway.gateway_module import GatewayModule
    from runtime.msgs.nav import Odometry
    from runtime.runtime_interface import TOPICS

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = GatewayModule()
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
    from runtime.runtime_interface import TOPICS

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = _field_gateway("nav")
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/topic")

    payload = asyncio.run(endpoint(topic="/cmd_vel"))

    assert payload["ok"] is True
    assert payload["topic"]["topic"] == TOPICS.cmd_vel
    assert payload["inspection"]["communicate"] is True
    assert payload["inspection"]["arbitrary_publish_supported"] is False
    assert {item["path"] for item in payload["inspection"]["write_interfaces"]} == {"/api/v1/stop"}
    interfaces = payload["inspection"]["write_interfaces"]
    assert all(item["final_output_confirmed"] is False for item in interfaces)
    assert {item["response_evidence"] for item in interfaces} == {"command_ack_not_driver_execution"}
    assert all(
        channel.get("event_type") != "command_ack" for channel in payload["topic"]["observability"]["gateway_channels"]
    )


@pytest.mark.parametrize(
    (
        "contract",
        "data_source",
        "command_sink",
        "simulation_only",
    ),
    [
        (
            "real",
            "field",
            "driver",
            "0",
        ),
        (
            "mujoco_fastlio2_live",
            "mujoco_fastlio2_live",
            "mujoco_velocity_adapter",
            "1",
        ),
    ],
)
def test_runtime_dataflow_exposes_all_contract_streams_for_real_and_sim(
    monkeypatch,
    contract: str,
    data_source: str,
    command_sink: str,
    simulation_only: str,
):
    from gateway.gateway_module import GatewayModule
    from runtime.runtime_interface import runtime_data_flow_topics

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", contract)
    monkeypatch.setenv("LINGTU_DATA_SOURCE", data_source)
    monkeypatch.setenv("LINGTU_COMMAND_SINK", command_sink)
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", simulation_only)

    gateway = GatewayModule()
    endpoint_fn = _endpoint(gateway, "/api/v1/runtime/dataflow")

    payload = asyncio.run(endpoint_fn())
    topics = {item["topic"]: item for item in payload["topics"]}

    assert set(runtime_data_flow_topics(contract)) <= set(topics)
    assert payload["runtime_contract"] == contract
    assert "ros2_topic_required" not in payload
    for topic in topics.values():
        assert "ros2_topic_required" not in topic["inspection"]
        assert topic["inspection"]["arbitrary_publish_supported"] is False
        assert topic["communication"]["arbitrary_publish_supported"] is False


def test_runtime_dataflow_topic_route_exposes_all_whitelisted_write_interfaces(
    monkeypatch,
):
    from runtime.runtime_interface import TOPICS

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = _field_gateway("nav")
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/topic")
    expected = {
        TOPICS.nav_command_request: {
            "/api/v1/goal",
            "/api/v1/navigate/click",
            "/api/v1/navigation/cancel",
        },
        TOPICS.cmd_vel: {"/api/v1/stop"},
    }

    for topic, paths in expected.items():
        payload = asyncio.run(endpoint(topic=topic))
        assert payload["ok"] is True, topic
        assert payload["topic"]["topic"] == topic
        assert payload["inspection"]["communicate"] is True
        assert payload["inspection"]["arbitrary_publish_supported"] is False
        assert {item["path"] for item in payload["inspection"]["write_interfaces"]} == paths

    command_payload = asyncio.run(endpoint(topic=TOPICS.nav_command_request))
    for interface in command_payload["inspection"]["write_interfaces"]:
        assert interface["command_path"] == [
            "Gateway",
            "nav.goals",
            "nav.commands",
            TOPICS.nav_command_request,
        ]


def test_runtime_dataflow_reflects_environment_change_without_cache(
    monkeypatch,
):
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow")

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "mujoco_fastlio2_live")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "mujoco_fastlio2_live")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "mujoco_velocity_adapter")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "1")
    sim_payload = asyncio.run(endpoint())

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "field")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "driver")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "0")
    real_payload = asyncio.run(endpoint())

    assert sim_payload["runtime_contract"] == "mujoco_fastlio2_live"
    assert sim_payload["runtime_boundary"]["simulation_only"] is True
    assert sim_payload["runtime_boundary"]["command_sink"] == "mujoco_velocity_adapter"
    assert real_payload["runtime_contract"] == "real"
    assert real_payload["runtime_boundary"]["simulation_only"] is False
    assert real_payload["runtime_boundary"]["command_sink"] == "driver"
    sim_topics = {item["topic"] for item in sim_payload["topics"]}
    real_topics = {item["topic"] for item in real_payload["topics"]}
    assert "/lidar/raw_frame" in sim_topics
    assert "/lidar/raw_frame" in real_topics
    assert sim_topics != real_topics


def test_runtime_dataflow_topic_route_reports_unknown_selector(monkeypatch):
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = GatewayModule()
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

    from diagnostics.field.gateway_acceptance import (
        FIELD_LIVE_TOPICS,
    )
    from runtime.msgs.map import MapSceneFrame
    from runtime.msgs.nav import Odometry, Path
    from runtime.msgs.sensor import PointCloud2
    from runtime.runtime_interface import TOPICS

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

    for topic in FIELD_LIVE_TOPICS:
        observability = topics[topic]["observability"]
        assert observability["has_fresh_module_sample"] is True, topic
        assert observability["live_module_samples"] is True, topic
        assert "ros2_topic_required" not in observability, topic
        assert any(port["msg_count"] > 0 for port in observability["module_port_candidates"]), topic

    assert topics[TOPICS.cmd_vel]["communication"]["arbitrary_publish_supported"] is False


def test_runtime_dataflow_topic_route_answers_every_product_observable_stream_without_ros2(
    monkeypatch,
):
    from diagnostics.field.gateway_acceptance import PRODUCT_OBSERVABLE_TOPICS
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/topic")

    for topic in PRODUCT_OBSERVABLE_TOPICS:
        payload = asyncio.run(endpoint(topic=topic))

        assert payload["ok"] is True, topic
        assert payload["topic"]["topic"] == topic
        assert "ros2_topic_required" not in payload["inspection"], topic
        assert payload["inspection"]["arbitrary_publish_supported"] is False, topic
        assert payload["inspection"]["payload_available"] is True, topic
        assert payload["inspection"]["payload_interfaces"] or payload["inspection"]["stream_interfaces"], topic


def test_runtime_dataflow_subscribe_route_covers_every_gateway_realtime_stream(
    monkeypatch,
):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeDataflowSubscribeRequest

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = GatewayModule()
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

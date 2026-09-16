import importlib.util
import io
import json
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]


def _read(path: str) -> str:
    return (REPO_ROOT / path).read_text(encoding="utf-8")


def _load_soak_module():
    spec = importlib.util.spec_from_file_location(
        "lingtu_soak_under_test",
        REPO_ROOT / "src" / "diagnostics" / "field" / "soak.py",
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _healthy_soak_sample() -> dict:
    return {
        "endpoint_errors": [],
        "client_contract_violations": [],
        "has_odometry": True,
        "localization_state": "ready",
        "pose_fresh": True,
        "odom_age_ms": 90.0,
        "cloud_age_ms": 120.0,
        "diag_age_ms": 300.0,
        "localizer_health_topic_age_ms": 250.0,
        "slam_hz": 10.0,
        "map_points": 5000.0,
        "confidence": 0.95,
        "task_state": "IDLE",
        "goal_admission": "BLOCKED",
        "control_authority": "NONE",
        "motion_permission": "HELD",
        "motion_observation": "QUIET",
        "stop_confirmation": "CONFIRMED",
        "navigation_blockers": ["navigation_session_inactive"],
        "non_motion_safe": True,
    }


def _soak_limits() -> dict:
    return {
        "min_slam_hz": 1.0,
        "max_odom_age_ms": 1500.0,
        "max_cloud_age_ms": 5000.0,
        "max_diag_age_ms": 3000.0,
        "max_localizer_health_age_ms": 3000.0,
        "min_localization_confidence": 0.5,
    }


def test_legacy_docs_service_graph_stays_removed():
    assert not (REPO_ROOT / "docs" / "04-deployment" / "services").exists()


@pytest.mark.parametrize(
    "filename",
    ("lidar.service", "slam.service", "slam_pgo.service", "localizer.service", "genz_icp.service", "hba.service"),
)
def test_s100p_ros2_systemd_service_templates_stay_removed(filename: str):
    assert not (REPO_ROOT / "scripts" / "deploy" / "s100p" / filename).exists()


def test_slamd_publishes_native_localization_health_on_every_runtime_tick():
    runtime = _read("src/localization/slam/cpp/cyclone_runtime.cpp")
    health = _read("src/localization/slam/cpp/map_tracking_health.hpp")

    write_health = "dds.writeHealth(healthJson(out, tracking_status, imu_frame_input));"
    status_timer = "if (!cli.status_json_path.empty() && status_json_period_s > 0.0)"
    assert write_health in runtime
    assert runtime.index(write_health) < runtime.index(status_timer)
    assert "backend_state == SlamState::Lost" in health
    assert 'out.reason = "map_tracking_repeated_failures";' in health


def test_field_doctor_uses_current_camera_and_runtime_contracts():
    text = _read("src/diagnostics/field/doctor.py")
    for expected in (
        "camera.usb",
        "/api/v1/camera/snapshot",
        "camera.gateway_snapshot",
        "camera.video_nodes",
        "--require-camera",
        "RunPlan.load",
        "gateway.client_readiness",
        '"/api/v1/readiness"',
        'nav.get("goal_admission")',
    ):
        assert expected in text


def test_soak_accepts_ready_503_when_only_navigation_session_is_inactive():
    soak = _load_soak_module()
    payload = {
        "data_ready": True,
        "non_motion_safe": True,
        "reasons": ["navigation_blocked:navigation_session_inactive"],
        "failed_modules": [],
    }
    assert soak.ready_status_is_non_motion_safe("ready", 503, payload) is True
    assert soak.sample_violations(_healthy_soak_sample(), _soak_limits()) == ([], [])


def test_soak_checks_client_readiness_contract_shape():
    soak = _load_soak_module()
    payloads = {
        "bootstrap": {
            "schema_version": 4,
            "links": {
                "state": "/api/v1/state",
                "events": "/api/v1/events",
                "scene_graph": "/api/v1/scene_graph",
                "locations": "/api/v1/locations",
                "path": "/api/v1/path",
                "readiness": "/api/v1/readiness",
            },
        },
        "capabilities": {"schema_version": 2, "endpoints": {"state": {}}},
        "readiness": {
            "schema_version": 1,
            "status": "degraded",
            "reasons": ["navigation_blocked:navigation_session_inactive"],
            "modules": {},
        },
        "localization": {"schema_version": 1},
        "navigation": {"schema_version": 3},
        "state": {"schema_version": 4},
        "path": {"schema_version": 1},
        "scene_graph": {"schema_version": 1},
        "locations": {"schema_version": 1},
    }
    assert soak.client_contract_violations(payloads) == []


def test_soak_rejects_unsafe_ready_503_states():
    soak = _load_soak_module()
    not_ready = {
        "data_ready": False,
        "non_motion_safe": True,
        "reasons": ["navigation_blocked:navigation_session_inactive"],
        "failed_modules": [],
    }
    assert soak.ready_status_is_non_motion_safe("ready", 503, not_ready) is False
    sample = _healthy_soak_sample()
    sample["control_authority"] = "OPERATOR"
    violations, _warnings = soak.sample_violations(sample, _soak_limits())
    assert "control_authority=OPERATOR" in violations


@pytest.mark.parametrize("api_key", [None, "  field-soak-test-key  "])
@pytest.mark.parametrize(
    ("path", "same_gateway"),
    [
        ("/api/v1/readiness", True),
        ("http://127.0.0.1:5050/api/v1/app/traffic", True),
        ("http://other-gateway:5050/api/v1/app/traffic", False),
    ],
)
def test_soak_http_get_uses_credentials_only_for_configured_gateway(monkeypatch, api_key, path, same_gateway):
    soak = _load_soak_module()
    if api_key is None:
        monkeypatch.delenv("LINGTU_API_KEY", raising=False)
    else:
        monkeypatch.setenv("LINGTU_API_KEY", api_key)

    class Response(io.BytesIO):
        def getcode(self):
            return 200

    def open_request(request, *, timeout):
        assert request.get_method() == "GET"
        assert request.get_header("X-api-key") == (
            api_key.strip() if api_key and same_gateway else None
        )
        assert timeout == 3.0
        return Response(b'{"ready":true}')

    monkeypatch.setattr(soak.urllib.request, "urlopen", open_request)
    result = soak.http_json("http://127.0.0.1:5050", path)

    assert result[:3] == (200, {"ready": True}, None)
    assert "field-soak-test-key" not in repr(result)


def test_soak_http_retains_failed_readiness_body_without_credentials(monkeypatch):
    soak = _load_soak_module()
    monkeypatch.setenv("LINGTU_API_KEY", "field-soak-test-key")
    payload = {"data_ready": False, "reasons": ["localization:lost"]}

    def open_request(request, *, timeout):
        raise soak.urllib.error.HTTPError(
            request.full_url, 503, "not ready", {}, io.BytesIO(json.dumps(payload).encode())
        )

    monkeypatch.setattr(soak.urllib.request, "urlopen", open_request)
    result = soak.http_json("http://127.0.0.1:5050", "/ready")

    assert result[:3] == (503, payload, None)
    assert "field-soak-test-key" not in repr(result)


@pytest.mark.parametrize(
    ("overrides", "expected_violation"),
    [
        ({}, None),
        ({"slam_hz": 0.0}, "slam_hz<1"),
        ({"slam_hz": None}, "slam_hz<1"),
        ({"has_odometry": False}, "has_odometry=false"),
        ({"pose_fresh": False}, "pose_fresh=false"),
        ({"data_ready": False}, "data_ready:false"),
        ({"control_authority": "OPERATOR"}, "control_authority=OPERATOR"),
        ({"motion_observation": "MOVING"}, "motion_observation=MOVING"),
    ],
)
def test_soak_samples_native_runtime_without_viewer_cache(monkeypatch, overrides, expected_violation):
    soak = _load_soak_module()
    runtime = {**_healthy_soak_sample(), "data_ready": True, **overrides}
    payloads = {
        name: {"schema_version": version}
        for name, version in soak.EXPECTED_SCHEMA_VERSIONS.items()
    }
    payloads["bootstrap"]["links"] = {
        name: f"/api/v1/{name}"
        for name in ("state", "events", "scene_graph", "locations", "path", "readiness")
    }
    payloads["capabilities"]["endpoints"] = {"state": {}}
    payloads["ready"] = {
        "ready": runtime["data_ready"], "data_ready": runtime["data_ready"],
        "non_motion_safe": True, "failed_modules": [], "reasons": [],
    }
    payloads["readiness"].update({
        "status": "ready", "reasons": [], "modules": {},
        "runtime": {"navigation": {"blockers": []}},
    })
    payloads["localization"].update({
        "state": "ready", "has_odometry": runtime["has_odometry"],
        "pose_fresh": runtime["pose_fresh"], "confidence": 0.95,
    })
    payloads["navigation"].update({
        "task": {"state": "IDLE"}, "goal_admission": {"state": "ACCEPTING"},
        "control": {"authority": runtime["control_authority"]},
        "motion": {"permission": "CLEAR", "observation": runtime["motion_observation"],
                   "stop_confirmation": "NOT_REQUESTED"},
    })
    payloads["health"] = {"map_points": 0, "sensors": {"slam": {"hz": runtime["slam_hz"]}}}

    def http_json(_gateway, path):
        name = soak.READ_ONLY_ENDPOINT_NAMES[path]
        code = 503 if name == "ready" and not runtime["data_ready"] else 200
        return code, payloads[name], None, 0.1

    monkeypatch.setattr(soak, "http_json", http_json)
    monkeypatch.setattr(soak, "process_rows", lambda: [])
    sample = soak.sample_once("http://127.0.0.1:5050", 0, soak.time.monotonic(), soak.thresholds())

    assert sample["map_points"] == 0
    if expected_violation is None:
        assert sample["violations"] == []
    else:
        assert expected_violation in sample["violations"]


def test_soak_viewer_cache_drop_is_telemetry_not_native_map_failure():
    soak = _load_soak_module()
    samples = [
        {**_healthy_soak_sample(), "index": index, "map_points": points,
         "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}}
        for index, points in enumerate((5000, 0, 0))
    ]
    summary, violations, warnings = soak.summarize(samples, 6.0, soak.thresholds())

    assert summary["map_points_drop_ratio"] == 1.0
    assert summary["map_points_source"] == "gateway_viewer_cache"
    assert violations == []
    assert warnings == []


def test_soak_process_summary_only_tracks_native_slamd_and_host():
    text = _read("src/diagnostics/field/soak.py").lower()
    assert "super_lio" not in text
    assert "super-lio" not in text
    assert "relocation_node" not in text
    assert "lio_node" not in text
    assert "fastlio" not in text
    assert '"slamd"' in text
    assert '"host"' in text
    assert '"avg_rss_mb"' in text
    soak = _load_soak_module()
    summary = soak.process_summary(
        [
            {"processes": [{"label": "slamd", "pcpu": 10.0, "pmem": 2.0, "rss_kb": 1024}]},
            {"processes": [{"label": "slamd", "pcpu": 20.0, "pmem": 4.0, "rss_kb": 3072}]},
        ]
    )
    assert summary["slamd"] == {
        "avg_pcpu": 15.0,
        "max_pcpu": 20.0,
        "avg_rss_mb": 2.0,
        "max_rss_mb": 3.0,
        "avg_pmem": 3.0,
    }


def test_thunder_service_installer_has_no_s100p_ros2_dispatch():
    text = _read("scripts/deploy/thunder/install_services.sh")
    assert "../s100p/install_services.sh" not in text
    assert "ros-compat|legacy)" not in text
    assert "LINGTU_ENABLE_LEGACY_ROS2_SERVICES" not in text


def test_explore_endpoint_exit_always_requests_native_motion_stop():
    unit = _read("scripts/deploy/thunder/lt-explore.service")
    assert (
        "ExecStopPost=/opt/lingtu/current/bin/"
        "lingtu_nav_control stop explore_endpoint_exit --timeout-ms 7000"
    ) in unit
    assert "TimeoutStopSec=30" in unit


def test_field_host_uses_current_task_journals():
    unit = _read("scripts/deploy/thunder/lt-host.service")
    assert "/var/lib/lingtu/task_journal/explore_runs.json" in unit
    assert "/var/lib/lingtu/task_journal/navigation_tasks.sqlite3" in unit

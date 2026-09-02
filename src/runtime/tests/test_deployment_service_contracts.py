import importlib.util
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]


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
        "active_cmd_source": "none",
        "navigation_state": "IDLE",
        "navigation_blockers": ["navigation_session_inactive"],
        "non_motion_safe": True,
    }


def _soak_limits() -> dict:
    return {
        "min_slam_hz": 1.0,
        "min_map_points": 1.0,
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
        "def command_source_name(control):",
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
            "schema_version": 1,
            "links": {
                "state": "/api/v1/state",
                "events": "/api/v1/events",
                "scene_graph": "/api/v1/scene_graph",
                "locations": "/api/v1/locations",
                "path": "/api/v1/path",
                "readiness": "/api/v1/readiness",
            },
        },
        "capabilities": {"schema_version": 1, "endpoints": {"state": {}}},
        "readiness": {
            "schema_version": 1,
            "status": "degraded",
            "reasons": ["navigation_blocked:navigation_session_inactive"],
            "modules": {},
        },
        "localization": {"schema_version": 1},
        "navigation": {"schema_version": 1},
        "state": {"schema_version": 1},
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
    sample["active_cmd_source"] = "teleop"
    violations, _warnings = soak.sample_violations(sample, _soak_limits())
    assert "active_cmd_source=teleop" in violations


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

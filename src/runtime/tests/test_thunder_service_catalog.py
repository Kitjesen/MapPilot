from __future__ import annotations

import json
import os
import time
from pathlib import Path


def test_thunder_catalog_classifies_pgo_and_hba_as_optimization_services():
    from runtime.service_catalogs.thunder import (
        thunder_optimization_services,
        thunder_service_groups,
        thunder_service_metadata,
    )

    assert thunder_optimization_services() == ("slam_pgo", "hba")

    groups = thunder_service_groups()
    assert groups["native_dds"] == [
        "lidar",
        "slam",
        "traversability",
        "nav",
        "driver",
        "explore",
    ]
    assert groups["runtime"] == ["gateway", "lingtu"]
    assert groups["hardware"][:1] == ["camera"]
    assert "slam_pgo" in groups["legacy_ros2_compat"]
    assert "hba" in groups["experimental"]
    assert "nav_dds" not in groups["native_dds"]

    metadata = thunder_service_metadata()
    assert metadata["nav"]["role"] == "navigation_runtime"
    assert metadata["nav"]["product_default"] is True
    assert metadata["nav"]["install_enable_default"] is False
    assert metadata["nav_dds"]["catalog_alias"] is True
    assert metadata["driver"]["role"] == "motion_output"
    assert metadata["driver"]["product_default"] is True
    assert metadata["driver"]["install_enable_default"] is True
    assert metadata["driver"]["checks"] == [
        "systemd",
        "native_binary",
        "status_file",
    ]
    assert metadata["driver"]["status_max_age_s"] == 3.0
    assert metadata["camera"]["optional"] is True
    assert metadata["camera"]["install_enable_default"] is False
    assert metadata["gnss"]["optional"] is True
    assert metadata["gnss"]["role"] == "sensor_input"
    assert metadata["gnss"]["ros2_compat"] is False
    assert metadata["traversability"]["install_enable_default"] is False
    assert metadata["slam_pgo"]["role"] == "map_optimization"
    assert metadata["slam_pgo"]["ros2_compat"] is True
    assert metadata["slam_pgo"]["product_default"] is False
    assert metadata["hba"]["role"] == "map_optimization"
    assert metadata["hba"]["experimental"] is True
    assert metadata["hba"]["product_default"] is False


def test_service_manager_uses_thunder_catalog_aliases():
    from runtime.service_catalogs.thunder import thunder_service_aliases
    from runtime.service_manager import SERVICE_ALIASES

    aliases = thunder_service_aliases()
    assert SERVICE_ALIASES["slam_pgo"] == aliases["slam_pgo"]
    assert SERVICE_ALIASES["hba"] == aliases["hba"]
    assert SERVICE_ALIASES["slam"][0] == "lingtu-slam-dds.service"
    assert SERVICE_ALIASES["nav"][0] == "lingtu-nav-dds.service"
    assert SERVICE_ALIASES["nav_dds"][0] == "lingtu-nav-dds.service"
    assert SERVICE_ALIASES["driver"][0] == "lingtu-driver.service"
    assert SERVICE_ALIASES["camera"][0] == "lingtu-camera-dds.service"
    assert SERVICE_ALIASES["traversability"][0] == "lingtu-traversability-dds.service"
    assert SERVICE_ALIASES["explore"][0] == "lingtu-explore-dds.service"
    assert SERVICE_ALIASES["gateway"][0] == "lingtu.service"
    assert SERVICE_ALIASES["lingtu"][0] == "lingtu.service"


def test_thunder_catalog_declares_product_readiness_contracts():
    from runtime.runtime_interface import TOPICS
    from runtime.service_catalogs.thunder import thunder_service_metadata

    metadata = thunder_service_metadata()

    assert metadata["camera"]["checks"] == ["systemd", "native_binary", "dds", "status_file"]
    assert metadata["camera"]["topics"] == [TOPICS.camera_info]
    assert metadata["camera"]["dds_topics"] == ["rt/camera/info"]
    assert metadata["camera"]["shm_topics"] == [
        TOPICS.camera_color,
        TOPICS.camera_depth,
        TOPICS.camera_info,
    ]
    assert metadata["camera"]["shm_channels"] == [
        "/lingtu_camera_color",
        "/lingtu_camera_depth",
        "/lingtu_camera_info",
    ]
    assert metadata["camera"]["files"] == ["/dev/shm/lingtu/camera_status.json"]
    assert metadata["camera"]["binaries"] == [
        {
            "name": "camera_dds",
            "env": "LINGTU_CAMERA_DDS_BIN",
            "path": "/opt/lingtu/current/build/camera_dds/lingtu_camera_dds",
        },
        {
            "name": "orbbec_capture",
            "env": "LINGTU_ORBBEC_CAPTURE_BIN",
            "path": "/opt/lingtu/current/build/orbbec_native/orbbec_capture",
        },
    ]

    assert metadata["lidar"]["checks"] == ["systemd", "native_binary", "dds"]
    assert metadata["lidar"]["topics"] == [TOPICS.raw_lidar_points, TOPICS.raw_imu]
    assert metadata["lidar"]["dds_topics"] == ["rt/lidar/raw_frame", "rt/imu/raw"]
    assert metadata["lidar"]["binaries"] == [
        {
            "name": "livox_dds",
            "env": "LINGTU_LIVOX_BIN",
            "path": "/opt/lingtu/current/build/livox_sdk2_stream/livox_sdk2_stream",
        }
    ]

    assert metadata["slam"]["checks"] == ["systemd", "native_binary", "dds", "status_file"]
    assert metadata["slam"]["files"] == ["/tmp/lingtu_slam_status.json"]
    assert metadata["slam"]["binaries"] == [
        {
            "name": "slam_dds",
            "env": "LINGTU_SLAM_BIN",
            "path": "/opt/lingtu/current/build/slam_core/lingtu_slam_cyclone_runtime",
        }
    ]

    assert metadata["traversability"]["checks"] == [
        "systemd",
        "native_binary",
        "dds",
        "status_file",
    ]
    assert metadata["traversability"]["topics"] == [
        TOPICS.traversability,
        TOPICS.terrain_map,
        TOPICS.terrain_map_ext,
    ]
    assert metadata["traversability"]["dds_topics"] == [
        "rt/nav/traversability",
        "rt/nav/terrain_map",
        "rt/nav/terrain_map_ext",
    ]
    assert metadata["traversability"]["binaries"] == [
        {
            "name": "traversability_dds",
            "env": "LINGTU_TRAVERSABILITY_DDS_BIN",
            "path": "/opt/lingtu/current/build/nav_endpoint/lingtu_traversability_dds",
        }
    ]

    assert metadata["nav"]["checks"] == ["systemd", "native_binary", "status_file"]
    assert metadata["nav"]["topics"] == []
    assert metadata["nav"]["dds_topics"] == []
    assert metadata["nav"]["files"] == ["/dev/shm/lingtu/nav_endpoint_status.json"]
    assert metadata["nav"]["binaries"] == [
        {
            "name": "nav_dds",
            "env": "LINGTU_NAV_DDS_BIN",
            "path": "/opt/lingtu/current/build/nav_endpoint/navd",
        }
    ]
    assert metadata["explore"]["checks"] == [
        "systemd",
        "native_binary",
        "dds",
        "status_file",
    ]
    assert metadata["explore"]["topics"] == [
        TOPICS.odometry,
        TOPICS.exploration_snapshot,
        TOPICS.exploration_command,
        TOPICS.exploration_ack,
        TOPICS.nav_command_request,
        TOPICS.nav_command_ack,
    ]
    assert metadata["explore"]["dds_topics"] == [
        "rt/slam/odometry",
        "rt/nav/exploration_snapshot",
    ]
    assert metadata["gateway"]["checks"] == ["systemd", "http"]


def test_service_manager_status_details_include_readiness_contract(monkeypatch):
    from runtime.service_manager import ServiceManager

    manager = ServiceManager()

    def exists(unit: str) -> bool:
        return unit in {"lingtu-camera-dds.service", "lingtu-livox-dds.service"}

    def active(unit: str) -> bool:
        return unit == "lingtu-camera-dds.service"

    def status_file(files: list[str]) -> dict[str, object]:
        return {
            "ok": True,
            "files": [{"path": item, "exists": True, "status": "running"} for item in files],
            "blockers": [],
        }

    monkeypatch.setattr(manager, "_unit_exists", exists)
    monkeypatch.setattr(manager, "_is_active_unit", active)
    monkeypatch.setattr(manager, "_status_file_observation", status_file)
    monkeypatch.setattr(
        manager,
        "_native_binary_observation",
        lambda binaries: {"ok": True, "binaries": binaries, "blockers": []},
    )

    details = manager.status_details("camera", "lidar")

    camera = details["camera"]
    assert camera["status"] == "running"
    assert camera["ready"] is False
    assert camera["observed"]["systemd"] is True
    assert camera["observed"]["native_binary"]["ok"] is True
    assert camera["observed"]["status_file"]["ok"] is True
    assert camera["observed"]["dds"] == {
        "ok": False,
        "checked": False,
        "enabled": False,
        "reason": "set LINGTU_SERVICE_DDS_CHECK=1 to sample DDS topics",
        "topics": ["rt/camera/info"],
        "samples": {},
        "blockers": ["dds_unchecked"],
    }
    assert camera["blockers"] == ["dds_unchecked"]
    assert camera["contract"]["checks"] == ["systemd", "native_binary", "dds", "status_file"]
    assert camera["contract"]["topics"] == ["/camera/color/camera_info"]
    assert camera["contract"]["shm_topics"] == [
        "/camera/color/image_raw",
        "/camera/depth/image_raw",
        "/camera/color/camera_info",
    ]
    assert camera["contract"]["shm_channels"] == [
        "/lingtu_camera_color",
        "/lingtu_camera_depth",
        "/lingtu_camera_info",
    ]
    assert camera["contract"]["dds_topics"] == ["rt/camera/info"]
    assert camera["contract"]["binaries"][0]["name"] == "camera_dds"

    lidar = details["lidar"]
    assert lidar["status"] == "stopped"
    assert lidar["ready"] is False
    assert lidar["observed"] == {
        "systemd": False,
        "native_binary": {
            "ok": True,
            "binaries": [
                {
                    "name": "livox_dds",
                    "env": "LINGTU_LIVOX_BIN",
                    "path": "/opt/lingtu/current/build/livox_sdk2_stream/livox_sdk2_stream",
                }
            ],
            "blockers": [],
        },
        "dds": {
            "ok": False,
            "checked": False,
            "enabled": False,
            "reason": "set LINGTU_SERVICE_DDS_CHECK=1 to sample DDS topics",
            "topics": ["rt/lidar/raw_frame", "rt/imu/raw"],
            "samples": {},
            "blockers": ["dds_unchecked"],
        },
    }
    assert lidar["blockers"] == ["systemd_inactive", "dds_unchecked"]
    assert lidar["contract"]["topics"] == ["/lidar/raw_frame", "/imu/raw"]


def test_service_manager_dds_check_uses_native_probe(monkeypatch):
    import runtime.service_manager as service_manager
    from runtime.service_manager import ServiceManager

    manager = ServiceManager()
    calls = []

    class Result:
        returncode = 0
        stderr = ""
        stdout = json.dumps(
            [
                {
                    "topic": "rt/camera/color",
                    "samples": 3,
                    "hz": 25.0,
                    "frame_id": "camera_link",
                    "points": 307200,
                }
            ]
        )

    def fake_run(command, **_kwargs):
        calls.append(command)
        return Result()

    monkeypatch.setenv("LINGTU_SERVICE_DDS_CHECK", "1")
    monkeypatch.setenv("LINGTU_SERVICE_DDS_CHECK_TIMEOUT", "0.1")
    monkeypatch.setattr(service_manager.subprocess, "run", fake_run)

    observed = manager._dds_topic_observation(
        ["/camera/color/image_raw"],
        ["rt/camera/color"],
    )

    assert observed["ok"] is True
    assert observed["checked"] is True
    assert observed["samples"] == {"rt/camera/color": 3}
    assert observed["blockers"] == []
    assert "dds_probe.py" in calls[0][1]
    assert "--json" in calls[0]


def test_service_manager_status_file_missing_blocks_readiness(monkeypatch):
    from runtime.service_manager import ServiceManager

    manager = ServiceManager()

    monkeypatch.setattr(
        manager,
        "_unit_exists",
        lambda unit: unit == "lingtu-camera-dds.service",
    )
    monkeypatch.setattr(
        manager,
        "_is_active_unit",
        lambda unit: unit == "lingtu-camera-dds.service",
    )
    monkeypatch.setattr(
        manager,
        "_status_file_observation",
        lambda files: {
            "ok": False,
            "files": [{"path": item, "exists": False} for item in files],
            "blockers": [f"status_file_missing:{item}" for item in files],
        },
    )
    monkeypatch.setattr(
        manager,
        "_native_binary_observation",
        lambda binaries: {"ok": True, "binaries": binaries, "blockers": []},
    )

    camera = manager.status_details("camera")["camera"]

    assert camera["status"] == "running"
    assert camera["ready"] is False
    assert camera["observed"]["status_file"]["ok"] is False
    assert camera["blockers"] == [
        "status_file_missing:/dev/shm/lingtu/camera_status.json",
        "dds_unchecked",
    ]


def test_driver_status_requires_ready_and_fresh_heartbeat(monkeypatch, tmp_path):
    import runtime.service_manager as service_manager

    status_path = tmp_path / "driver_status.json"
    status_path.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.driver.status.v1",
                "ready": False,
                "connected": False,
            }
        ),
        encoding="utf-8",
    )
    metadata = dict(service_manager.SERVICE_METADATA["driver"])
    metadata.update(
        {
            "checks": ["status_file"],
            "files": [str(status_path)],
            "binaries": [],
            "status_max_age_s": 3.0,
        }
    )
    monkeypatch.setitem(service_manager.SERVICE_METADATA, "driver", metadata)
    manager = service_manager.ServiceManager()
    monkeypatch.setattr(manager, "_unit_exists", lambda _unit: False)
    monkeypatch.setattr(manager, "_is_active_unit", lambda _unit: False)

    disconnected = manager.status_details("driver")["driver"]
    assert disconnected["ready"] is False
    assert disconnected["blockers"] == [f"status_file_not_ready:{status_path}"]

    status_path.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.driver.status.v1",
                "ready": True,
                "connected": True,
            }
        ),
        encoding="utf-8",
    )
    old = time.time() - 10.0
    os.utime(status_path, (old, old))
    stale = manager.status_details("driver")["driver"]
    assert stale["ready"] is False
    assert stale["blockers"] == [f"status_file_stale:{status_path}"]


def test_service_manager_missing_unit_blocks_readiness_with_concrete_unit(monkeypatch):
    from runtime.service_manager import ServiceManager

    manager = ServiceManager()

    monkeypatch.setattr(manager, "_unit_exists", lambda unit: False)
    monkeypatch.setattr(manager, "_is_active_unit", lambda unit: False)
    monkeypatch.setattr(
        manager,
        "_status_file_observation",
        lambda files: {"ok": True, "files": [], "blockers": []},
    )
    monkeypatch.setattr(
        manager,
        "_native_binary_observation",
        lambda binaries: {"ok": True, "binaries": binaries, "blockers": []},
    )

    camera = manager.status_details("camera")["camera"]

    assert camera["status"] == "stopped"
    assert camera["ready"] is False
    assert camera["canonical_unit"] == "lingtu-camera-dds.service"
    assert camera["installed_units"] == []
    assert camera["blockers"] == [
        "systemd_unit_missing:lingtu-camera-dds.service",
        "dds_unchecked",
    ]


def test_service_manager_http_unchecked_blocks_gateway_readiness(monkeypatch):
    from runtime.service_manager import ServiceManager

    manager = ServiceManager()
    monkeypatch.delenv("LINGTU_SERVICE_HTTP_CHECK", raising=False)
    monkeypatch.setattr(
        manager,
        "_unit_exists",
        lambda unit: unit == "lingtu.service",
    )
    monkeypatch.setattr(
        manager,
        "_is_active_unit",
        lambda unit: unit == "lingtu.service",
    )

    gateway = manager.status_details("gateway")["gateway"]

    assert gateway["status"] == "running"
    assert gateway["ready"] is False
    assert gateway["observed"]["systemd"] is True
    assert gateway["observed"]["http"] == {
        "ok": False,
        "checked": False,
        "enabled": False,
        "reason": "set LINGTU_SERVICE_HTTP_CHECK=1 to probe HTTP readiness",
        "url": "http://127.0.0.1:5050/health",
        "blockers": ["http_unchecked"],
    }
    assert gateway["blockers"] == ["http_unchecked"]


def test_service_manager_dds_silence_blocks_readiness_when_check_enabled(monkeypatch):
    from runtime.service_manager import ServiceManager

    manager = ServiceManager()

    monkeypatch.setenv("LINGTU_SERVICE_DDS_CHECK", "1")
    monkeypatch.setattr(
        manager,
        "_unit_exists",
        lambda unit: unit == "lingtu-camera-dds.service",
    )
    monkeypatch.setattr(
        manager,
        "_is_active_unit",
        lambda unit: unit == "lingtu-camera-dds.service",
    )
    monkeypatch.setattr(
        manager,
        "_status_file_observation",
        lambda files: {
            "ok": True,
            "files": [{"path": item, "exists": True, "status": "running"} for item in files],
            "blockers": [],
        },
    )
    monkeypatch.setattr(
        manager,
        "_native_binary_observation",
        lambda binaries: {"ok": True, "binaries": binaries, "blockers": []},
    )
    monkeypatch.setattr(
        manager,
        "_dds_topic_observation",
        lambda topics, dds_topics, **_kwargs: {
            "ok": False,
            "checked": True,
            "enabled": True,
            "topics": dds_topics,
            "samples": {item: 0 for item in dds_topics},
            "blockers": [f"dds_topic_silent:{item}" for item in dds_topics],
        },
    )

    camera = manager.status_details("camera")["camera"]

    assert camera["status"] == "running"
    assert camera["ready"] is False
    assert camera["observed"]["dds"]["checked"] is True
    assert camera["observed"]["dds"]["enabled"] is True
    assert camera["blockers"] == ["dds_topic_silent:rt/camera/info"]


def test_service_manager_camera_native_binary_missing_blocks_readiness(monkeypatch):
    from runtime.service_manager import ServiceManager

    manager = ServiceManager()

    monkeypatch.setattr(
        manager,
        "_unit_exists",
        lambda unit: unit == "lingtu-camera-dds.service",
    )
    monkeypatch.setattr(
        manager,
        "_is_active_unit",
        lambda unit: unit == "lingtu-camera-dds.service",
    )
    monkeypatch.setattr(
        manager,
        "_status_file_observation",
        lambda files: {"ok": True, "files": [], "blockers": []},
    )
    monkeypatch.setattr(
        manager,
        "_dds_topic_observation",
        lambda topics, dds_topics, **_kwargs: {
            "ok": True,
            "topics": dds_topics,
            "blockers": [],
        },
    )
    monkeypatch.delenv("LINGTU_CAMERA_DDS_BIN", raising=False)
    monkeypatch.delenv("LINGTU_ORBBEC_CAPTURE_BIN", raising=False)

    camera = manager.status_details("camera")["camera"]

    assert camera["ready"] is False
    assert camera["observed"]["native_binary"]["ok"] is False
    assert camera["blockers"] == [
        "native_binary_missing_or_not_executable:camera_dds:/opt/lingtu/current/build/camera_dds/lingtu_camera_dds",
        "native_binary_missing_or_not_executable:orbbec_capture:/opt/lingtu/current/build/orbbec_native/orbbec_capture",
    ]


def test_service_manager_core_native_binary_missing_blocks_readiness(monkeypatch):
    from runtime.service_manager import ServiceManager

    manager = ServiceManager()

    monkeypatch.setattr(
        manager,
        "_unit_exists",
        lambda unit: unit == "lingtu-nav-dds.service",
    )
    monkeypatch.setattr(
        manager,
        "_is_active_unit",
        lambda unit: unit == "lingtu-nav-dds.service",
    )
    monkeypatch.setattr(
        manager,
        "_status_file_observation",
        lambda files: {"ok": True, "files": [], "blockers": []},
    )
    monkeypatch.setattr(
        manager,
        "_dds_topic_observation",
        lambda topics, dds_topics, **_kwargs: {
            "ok": True,
            "topics": dds_topics,
            "blockers": [],
        },
    )
    monkeypatch.delenv("LINGTU_NAV_DDS_BIN", raising=False)

    nav = manager.status_details("nav")["nav"]

    assert nav["ready"] is False
    assert nav["observed"]["native_binary"]["ok"] is False
    assert nav["blockers"] == [
        "native_binary_missing_or_not_executable:nav_dds:/opt/lingtu/current/build/nav_endpoint/navd"
    ]


def test_thunder_installer_covers_cataloged_product_and_optional_services():
    from runtime.service_catalogs.thunder import (
        thunder_field_dds_topics,
        thunder_field_native_binaries,
        thunder_field_readiness_services,
        thunder_field_readiness_units,
        thunder_field_shm_channels,
        thunder_field_status_files,
        thunder_install_plan,
        thunder_install_services,
        thunder_service_install_unit,
        thunder_service_installers,
        thunder_service_specs,
    )

    installer = Path("scripts/deploy/thunder/install_services.sh").read_text(encoding="utf-8")

    expected_installers = {
        "driver": "install_driver_service.sh",
        "camera": "install_camera_dds_service.sh",
        "gnss": "install_gnss_dds_service.sh",
        "lidar": "install_livox_dds_service.sh",
        "slam": "install_slam_dds_service.sh",
        "traversability": "install_traversability_dds_service.sh",
        "nav": "install_nav_dds_service.sh",
        "explore": "install_explore_dds_service.sh",
        "endpoint": "install_dds_endpoint_service.sh",
        "lingtu": "install_lingtu_service.sh",
        "gateway": "install_lingtu_service.sh",
    }
    catalog_names = {
        spec.name
        for spec in thunder_service_specs()
        if (spec.product_default or spec.optional)
        and not spec.catalog_alias
        and not spec.ros2_compat
        and not spec.experimental
    }
    catalog_names.add("explore")

    assert catalog_names <= set(expected_installers)
    for name in sorted(catalog_names):
        assert thunder_service_installers()[name] == expected_installers[name]
        assert expected_installers[name] not in installer

    assert "install-services" in installer
    assert "runtime.service_catalogs.thunder installer" in installer
    assert 'catalog_installer "${service}"' in installer
    assert 'bash "${SCRIPT_DIR}/${installer}"' in installer

    assert thunder_install_plan("field-cpp") == (
        "install_livox_dds_service.sh",
        "install_camera_dds_service.sh",
        "install_slam_dds_service.sh",
        "install_traversability_dds_service.sh",
        "install_nav_dds_service.sh",
        "install_driver_service.sh",
        "install_lingtu_service.sh",
    )
    assert thunder_install_services("field-cpp") == (
        "lidar",
        "camera",
        "slam",
        "traversability",
        "nav",
        "driver",
        "lingtu",
    )
    assert thunder_field_readiness_services() == (
        "lidar",
        "camera",
        "slam",
        "traversability",
        "nav",
        "driver",
        "lingtu",
    )
    assert thunder_field_readiness_units() == (
        "lingtu-livox-dds.service",
        "lingtu-camera-dds.service",
        "lingtu-slam-dds.service",
        "lingtu-traversability-dds.service",
        "lingtu-nav-dds.service",
        "lingtu-driver.service",
        "lingtu.service",
    )
    assert thunder_field_status_files() == {
        "camera": "/dev/shm/lingtu/camera_status.json",
        "slam": "/tmp/lingtu_slam_status.json",
        "traversability": "/dev/shm/lingtu/traversability_status.json",
        "nav": "/dev/shm/lingtu/nav_endpoint_status.json",
        "driver": "/dev/shm/lingtu/driver_status.json",
    }
    assert thunder_field_native_binaries() == {
        "livox_dds": {
            "service": "lidar",
            "env": "LINGTU_LIVOX_BIN",
            "path": "/opt/lingtu/current/build/livox_sdk2_stream/livox_sdk2_stream",
        },
        "camera_dds": {
            "service": "camera",
            "env": "LINGTU_CAMERA_DDS_BIN",
            "path": "/opt/lingtu/current/build/camera_dds/lingtu_camera_dds",
        },
        "orbbec_capture": {
            "service": "camera",
            "env": "LINGTU_ORBBEC_CAPTURE_BIN",
            "path": "/opt/lingtu/current/build/orbbec_native/orbbec_capture",
        },
        "slam_dds": {
            "service": "slam",
            "env": "LINGTU_SLAM_BIN",
            "path": "/opt/lingtu/current/build/slam_core/lingtu_slam_cyclone_runtime",
        },
        "traversability_dds": {
            "service": "traversability",
            "env": "LINGTU_TRAVERSABILITY_DDS_BIN",
            "path": "/opt/lingtu/current/build/nav_endpoint/lingtu_traversability_dds",
        },
        "nav_dds": {
            "service": "nav",
            "env": "LINGTU_NAV_DDS_BIN",
            "path": "/opt/lingtu/current/build/nav_endpoint/navd",
        },
        "driver": {
            "service": "driver",
            "env": "LINGTU_DRIVER_BIN",
            "path": "/opt/lingtu/current/build/driver/lingtu_driver",
        },
    }
    assert thunder_field_dds_topics()["lidar"]["dds_topics"] == (
        "rt/lidar/raw_frame",
        "rt/imu/raw",
    )
    assert thunder_field_dds_topics()["camera"]["dds_topics"] == ("rt/camera/info",)
    assert thunder_field_shm_channels()["camera"]["topics"] == (
        "/camera/color/image_raw",
        "/camera/depth/image_raw",
        "/camera/color/camera_info",
    )
    assert thunder_field_shm_channels()["camera"]["shm_channels"] == (
        "/lingtu_camera_color",
        "/lingtu_camera_depth",
        "/lingtu_camera_info",
    )
    assert thunder_field_dds_topics()["slam"]["dds_topics"] == (
        "rt/slam/odometry",
        "rt/slam/map_cloud",
        "rt/slam/localization_health",
    )
    assert thunder_field_dds_topics()["traversability"]["dds_topics"] == (
        "rt/nav/traversability",
        "rt/nav/terrain_map",
        "rt/nav/terrain_map_ext",
    )
    assert "nav" not in thunder_field_dds_topics()
    assert thunder_field_dds_topics()["driver"]["dds_topics"] == (
        "rt/nav/cmd_vel",
        "rt/driver/control_state",
    )
    assert thunder_install_plan("lidar-dds") == ("install_livox_dds_service.sh",)
    assert thunder_install_plan("dds-endpoint") == ("install_dds_endpoint_service.sh",)
    assert thunder_install_plan("native-driver") == ("install_driver_service.sh",)
    assert thunder_install_plan("camera-dds") == ("install_camera_dds_service.sh",)
    assert thunder_install_plan("gnss-dds") == ("install_gnss_dds_service.sh",)
    assert thunder_install_plan("explore-dds") == ("install_explore_dds_service.sh",)
    assert thunder_install_plan("unknown") == ()
    assert thunder_install_services("lidar-dds") == ("lidar",)
    assert thunder_install_services("dds-endpoint") == ("endpoint",)
    assert thunder_install_services("native-driver") == ("driver",)
    assert thunder_install_services("camera-dds") == ("camera",)
    assert thunder_install_services("gnss-dds") == ("gnss",)
    assert thunder_install_services("explore-dds") == ("explore",)
    assert thunder_install_services("unknown") == ()

    assert thunder_service_install_unit("lidar") == "lingtu-livox-dds.service"
    assert thunder_service_install_unit("camera-dds") == "lingtu-camera-dds.service"
    assert thunder_service_install_unit("gnss-dds") == "lingtu-gnss-dds.service"
    assert thunder_service_install_unit("slam") == "lingtu-slam-dds.service"
    assert thunder_service_install_unit("traversability") == ("lingtu-traversability-dds.service")
    assert thunder_service_install_unit("nav-dds") == "lingtu-nav-dds.service"
    assert thunder_service_install_unit("driver") == "lingtu-driver.service"
    assert thunder_service_install_unit("lingtu") == "lingtu.service"
    assert thunder_service_install_unit("unknown") == ""

    from runtime.service_catalogs.thunder import thunder_service_install_enable_default

    assert thunder_service_install_enable_default("traversability") == "0"
    assert thunder_service_install_enable_default("camera-dds") == "0"
    assert thunder_service_install_enable_default("gnss-dds") == "0"
    assert thunder_service_install_enable_default("nav") == "0"
    assert thunder_service_install_enable_default("driver") == "1"
    assert thunder_service_install_enable_default("unknown") == ""


def test_thunder_catalog_cli_exports_field_readiness_targets(capsys):
    from runtime.service_catalogs.thunder import _main

    assert _main(["thunder", "install-services", "field-cpp"]) == 0
    assert capsys.readouterr().out.splitlines() == [
        "lidar",
        "camera",
        "slam",
        "traversability",
        "nav",
        "driver",
        "lingtu",
    ]

    assert _main(["thunder", "install-services", "camera-dds"]) == 0
    assert capsys.readouterr().out.splitlines() == ["camera"]

    assert _main(["thunder", "install-services", "unknown"]) == 2
    assert capsys.readouterr().out == ""

    assert _main(["thunder", "install-modes"]) == 0
    install_modes = capsys.readouterr().out.splitlines()
    for mode in (
        "camera-dds",
        "field-cpp",
        "gnss-dds",
        "lidar-dds",
        "native-driver",
        "nav-dds",
        "slam-dds",
        "traversability-dds",
    ):
        assert mode in install_modes

    assert _main(["thunder", "readiness-services"]) == 0
    assert capsys.readouterr().out.splitlines() == [
        "lidar",
        "camera",
        "slam",
        "traversability",
        "nav",
        "driver",
        "lingtu",
    ]

    assert _main(["thunder", "readiness-units"]) == 0
    assert capsys.readouterr().out.splitlines() == [
        "lingtu-livox-dds.service",
        "lingtu-camera-dds.service",
        "lingtu-slam-dds.service",
        "lingtu-traversability-dds.service",
        "lingtu-nav-dds.service",
        "lingtu-driver.service",
        "lingtu.service",
    ]

    assert _main(["thunder", "status-files"]) == 0
    assert capsys.readouterr().out.splitlines() == [
        "camera=/dev/shm/lingtu/camera_status.json",
        "slam=/tmp/lingtu_slam_status.json",
        "traversability=/dev/shm/lingtu/traversability_status.json",
        "nav=/dev/shm/lingtu/nav_endpoint_status.json",
        "driver=/dev/shm/lingtu/driver_status.json",
    ]

    assert _main(["thunder", "readiness-dds-topics"]) == 0
    assert capsys.readouterr().out.splitlines() == [
        "lidar=rt/lidar/raw_frame,rt/imu/raw",
        "camera=rt/camera/info",
        "slam=rt/slam/odometry,rt/slam/map_cloud,rt/slam/localization_health",
        "traversability=rt/nav/traversability,rt/nav/terrain_map,rt/nav/terrain_map_ext",
        "driver=rt/nav/cmd_vel,rt/driver/control_state",
    ]

    assert _main(["thunder", "readiness-shm-channels"]) == 0
    assert capsys.readouterr().out.splitlines() == [
        "camera=/lingtu_camera_color,/lingtu_camera_depth,/lingtu_camera_info",
    ]

    assert _main(["thunder", "readiness-binaries"]) == 0
    assert capsys.readouterr().out.splitlines() == [
        "livox_dds=lidar|LINGTU_LIVOX_BIN|/opt/lingtu/current/build/livox_sdk2_stream/livox_sdk2_stream",
        "camera_dds=camera|LINGTU_CAMERA_DDS_BIN|/opt/lingtu/current/build/camera_dds/lingtu_camera_dds",
        "orbbec_capture=camera|LINGTU_ORBBEC_CAPTURE_BIN|/opt/lingtu/current/build/orbbec_native/orbbec_capture",
        "slam_dds=slam|LINGTU_SLAM_BIN|/opt/lingtu/current/build/slam_core/lingtu_slam_cyclone_runtime",
        "traversability_dds=traversability|LINGTU_TRAVERSABILITY_DDS_BIN|/opt/lingtu/current/build/nav_endpoint/lingtu_traversability_dds",
        "nav_dds=nav|LINGTU_NAV_DDS_BIN|/opt/lingtu/current/build/nav_endpoint/navd",
        "driver=driver|LINGTU_DRIVER_BIN|/opt/lingtu/current/build/driver/lingtu_driver",
    ]

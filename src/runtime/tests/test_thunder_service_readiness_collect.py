from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "gates" / "thunder_service_readiness_collect.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("thunder_service_readiness_collect", SCRIPT)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_thunder_service_readiness_collector_is_read_only() -> None:
    module = _load_module()
    source = SCRIPT.read_text(encoding="utf-8")

    assert "lt-camera.service" in module.SERVICES
    assert module.STATUS_FILES["camera"] == "/dev/shm/lingtu/camera_status.json"
    assert module.STATUS_FILES["driver"] == "/dev/shm/lingtu/driver_status.json"
    assert module.SERVICES == (
        "lt-lidar.service",
        "lt-camera.service",
        "lt-slam.service",
        "lt-maps.service",
        "lt-terrain.service",
        "lt-nav.service",
        "lt-driver.service",
        "lt-host.service",
    )
    assert module.NATIVE_BINARY_DEFAULTS["nav_dds"] == ("/opt/lingtu/current/build/nav_endpoint/navd")
    assert module.NATIVE_BINARY_ENV["nav_dds"] == "LINGTU_NAV_DDS_BIN"
    assert module.NATIVE_BINARY_DEFAULTS["driver"] == ("/opt/lingtu/current/build/driver/lingtu_driver")
    assert module.NATIVE_BINARY_ENV["driver"] == "LINGTU_DRIVER_BIN"
    assert module.NATIVE_BINARY_DEFAULTS["traversability_dds"] == (
        "/opt/lingtu/current/build/nav_endpoint/lingtu_traversability_dds"
    )
    assert module.NATIVE_BINARY_ENV["traversability_dds"] == ("LINGTU_TRAVERSABILITY_DDS_BIN")
    assert module.NATIVE_BINARY_DEFAULTS == {name: item["path"] for name, item in module.NATIVE_BINARIES.items()}
    assert module.NATIVE_BINARY_ENV == {name: item["env"] for name, item in module.NATIVE_BINARIES.items()}
    assert "thunder_runtime_units" in source
    assert "thunder_runtime_status_files" in source
    assert "thunder_runtime_native_binaries" in source
    assert "thunder_runtime_dds_topics" in source
    assert "/api/v1/services/status" in source

    for mutating in (
        "systemctl start",
        "systemctl stop",
        "systemctl restart",
        "systemctl enable",
        "systemctl disable",
        "dds_write",
        "publish_cmd_vel",
    ):
        assert mutating not in source


def test_thunder_service_readiness_report_shape(monkeypatch) -> None:
    module = _load_module()

    monkeypatch.setattr(
        module,
        "collect_systemd",
        lambda: {"lt-camera.service": {"active_state": "inactive"}},
    )
    monkeypatch.setattr(
        module,
        "collect_status_files",
        lambda: {"camera": {"path": "/dev/shm/lingtu/camera_status.json", "exists": False}},
    )
    monkeypatch.setattr(
        module,
        "collect_native_binaries",
        lambda: {
            "ok": False,
            "binaries": {
                "camera_dds": {
                    "path": "/opt/lingtu/current/build/camera_dds/lingtu_camera_dds",
                    "exists": False,
                    "executable": False,
                }
            },
            "blockers": [
                "native_binary_missing_or_not_executable:"
                "camera_dds:/opt/lingtu/current/build/camera_dds/lingtu_camera_dds"
            ],
        },
    )
    monkeypatch.setattr(
        module,
        "collect_gnss_device",
        lambda: {
            "ok": False,
            "enabled": True,
            "device": "/dev/wtrtk980",
            "device_exists": False,
            "blockers": ["gnss_device_missing:/dev/wtrtk980"],
        },
    )
    monkeypatch.setattr(
        module,
        "collect_gateway",
        lambda base_url: {
            "health": {"ok": True, "status": 200},
            "services_status": {"ok": False, "status": 404},
        },
    )
    monkeypatch.setattr(
        module,
        "collect_processes",
        lambda: {"ok": True, "lines": [], "legacy_lines": [], "blockers": []},
    )
    monkeypatch.setattr(
        module,
        "collect_dds",
        lambda **_: {
            "ok": False,
            "checked": False,
            "services": {
                "camera": {
                    "dds_topics": [
                        "rt/camera/color",
                        "rt/camera/depth",
                        "rt/camera/info",
                    ],
                    "samples": {},
                }
            },
            "blockers": ["dds_unchecked"],
        },
    )

    report = module.build_report(gateway_url="http://127.0.0.1:5050")

    assert report["schema"] == "lingtu.thunder.service_readiness.v1"
    assert report["systemd"]["lt-camera.service"]["active_state"] == "inactive"
    assert report["status_files"]["camera"]["exists"] is False
    assert report["native_binaries"]["binaries"]["camera_dds"]["exists"] is False
    assert report["camera"]["ok"] is False
    assert report["camera"]["unit"] == "lt-camera.service"
    assert "camera:status_file_missing:/dev/shm/lingtu/camera_status.json" in report["summary"]["blockers"]
    assert report["gnss"]["blockers"] == ["gnss_device_missing:/dev/wtrtk980"]
    assert report["dds"]["blockers"] == ["dds_unchecked"]
    assert report["summary"]["ok"] is False
    assert (
        "native_binaries:native_binary_missing_or_not_executable:camera_dds:/opt/lingtu/current/build/camera_dds/lingtu_camera_dds"
        in report["summary"]["blockers"]
    )
    assert "gnss:gnss_device_missing:/dev/wtrtk980" in report["summary"]["blockers"]
    assert "dds:unchecked" in report["summary"]["blockers"]
    assert report["dds"]["services"]["camera"]["dds_topics"] == [
        "rt/camera/color",
        "rt/camera/depth",
        "rt/camera/info",
    ]
    assert report["gateway"]["health"]["ok"] is True
    assert report["gateway"]["services_status"]["status"] == 404


def test_thunder_service_readiness_collector_marks_missing_systemd_units(monkeypatch):
    module = _load_module()

    def fake_run(command, *, timeout=5.0):
        unit = command[2]
        if unit == "lt-camera.service":
            return {
                "ok": True,
                "returncode": 0,
                "stdout": (
                    "Id=lt-camera.service\n"
                    "LoadState=not-found\n"
                    "ActiveState=inactive\n"
                    "SubState=dead\n"
                    "UnitFileState=\n"
                    "NRestarts=0\n"
                ),
                "stderr": "",
            }
        return {
            "ok": True,
            "returncode": 0,
            "stdout": (
                f"Id={unit}\n"
                "LoadState=loaded\n"
                "ActiveState=active\n"
                "SubState=running\n"
                "UnitFileState=enabled\n"
                "NRestarts=0\n"
            ),
            "stderr": "",
        }

    monkeypatch.setattr(module, "_run", fake_run)

    systemd = module.collect_systemd()
    camera = systemd["lt-camera.service"]

    assert camera["load_state"] == "not-found"
    assert camera["active_state"] == "inactive"
    assert camera["missing"] is True
    assert systemd["lt-lidar.service"]["missing"] is False


def test_thunder_service_readiness_collector_marks_missing_gnss_device(monkeypatch):
    module = _load_module()

    monkeypatch.setattr(
        module,
        "load_config",
        lambda: SimpleNamespace(
            raw={
                "gnss": {
                    "enabled": True,
                    "device": "/dev/wtrtk980",
                    "rtcm": {"enabled": False},
                }
            },
            gnss=SimpleNamespace(
                enabled=True,
                model="WTRTK-980",
                device="/dev/wtrtk980",
                baud=115200,
                fusion=SimpleNamespace(enabled=True),
            ),
        ),
    )

    gnss = module.collect_gnss_device()

    assert gnss["ok"] is False
    assert gnss["enabled"] is True
    assert gnss["device"] == "/dev/wtrtk980"
    assert gnss["device_exists"] is False
    assert gnss["blockers"] == ["gnss_device_missing:/dev/wtrtk980"]


def test_thunder_service_readiness_collector_flags_legacy_ros2_processes(monkeypatch):
    module = _load_module()

    monkeypatch.setattr(
        module,
        "_run",
        lambda command, *, timeout=5.0: {
            "ok": True,
            "returncode": 0,
            "stdout": "\n".join(
                [
                    " 101 livox_sdk2_stream /opt/lingtu/livox_sdk2_stream --dds",
                    " 202 livox_ros_driver2 /opt/ros/humble/lib/livox_ros_driver2/livox_ros_driver2_node",
                    " 303 orbbec_camera /opt/ros/humble/lib/orbbec_camera/orbbec_camera_node",
                ]
            ),
            "stderr": "",
        },
    )

    processes = module.collect_processes()

    assert processes["ok"] is False
    assert any("livox_sdk2_stream" in line for line in processes["lines"])
    assert processes["blockers"] == [
        "legacy_process_observed:livox_ros_driver2",
        "legacy_process_observed:orbbec_camera",
    ]


def test_thunder_service_readiness_accepts_livox_lidar_imu_owner() -> None:
    module = _load_module()

    ownership = module.collect_lidar_imu_ownership(
        processes={
            "ok": True,
            "lines": [
                " 101 livox_sdk2_stream /opt/lingtu/livox_sdk2_stream --dds",
            ],
            "legacy_lines": [],
            "duplicate_imu_lines": [],
            "blockers": [],
        },
        dds={
            "checked": True,
            "services": {
                "lidar": {
                    "dds_topics": [
                        "rt/lidar/raw_frame",
                        "rt/imu/raw",
                    ],
                    "samples": {
                        "rt/lidar/raw_frame": 10,
                        "rt/imu/raw": 100,
                    },
                }
            },
            "blockers": [],
        },
    )

    assert ownership["ok"] is True
    assert ownership["expected_owner"] == "livox_dds"
    assert ownership["expected_process"] == "livox_sdk2_stream"
    assert ownership["same_owner_expected"] is True
    assert ownership["native_process_observed"] is True
    assert ownership["legacy_livox_observed"] is False
    assert ownership["raw_lidar_samples"] == 10
    assert ownership["imu_samples"] == 100
    assert ownership["blockers"] == []


def test_thunder_service_readiness_blocks_legacy_livox_and_duplicate_imu_owner(monkeypatch):
    module = _load_module()

    monkeypatch.setattr(
        module,
        "_run",
        lambda command, *, timeout=5.0: {
            "ok": True,
            "returncode": 0,
            "stdout": "\n".join(
                [
                    " 101 livox_sdk2_stream /opt/lingtu/livox_sdk2_stream --dds",
                    " 202 livox_ros_driver2 /opt/ros/humble/lib/livox_ros_driver2/livox_ros_driver2_node",
                    " 303 lingtu_imu_dds /opt/lingtu/current/build/imu/lingtu_imu_dds",
                ]
            ),
            "stderr": "",
        },
    )

    processes = module.collect_processes()
    ownership = module.collect_lidar_imu_ownership(
        processes=processes,
        dds={
            "checked": False,
            "services": {
                "lidar": {
                    "dds_topics": [
                        "rt/lidar/raw_frame",
                        "rt/imu/raw",
                    ],
                    "samples": {},
                }
            },
            "blockers": ["dds_unchecked"],
        },
    )
    summary = module.summarize_report(
        {
            "systemd": {},
            "status_files": {},
            "native_binaries": {"blockers": []},
            "gnss": {"blockers": []},
            "gateway": {},
            "processes": processes,
            "lidar_imu": ownership,
            "dds": {"checked": True, "blockers": []},
        }
    )

    assert processes["ok"] is False
    assert any("lingtu_imu_dds" in line for line in processes["duplicate_imu_lines"])
    assert ownership["ok"] is False
    assert ownership["blockers"] == [
        "legacy_process_observed:livox_ros_driver2",
        "imu_duplicate_writer_risk:lingtu_imu_dds",
    ]
    assert "lidar_imu:legacy_process_observed:livox_ros_driver2" in summary["blockers"]
    assert "lidar_imu:imu_duplicate_writer_risk:lingtu_imu_dds" in summary["blockers"]


def test_thunder_service_readiness_collector_reports_missing_camera_binaries(monkeypatch):
    module = _load_module()

    monkeypatch.setattr(
        module,
        "NATIVE_BINARY_DEFAULTS",
        {
            "camera_dds": "/missing/lingtu_camera_dds",
            "orbbec_capture": "/missing/orbbec_capture",
        },
    )
    monkeypatch.setattr(
        module,
        "NATIVE_BINARY_ENV",
        {
            "camera_dds": "LINGTU_CAMERA_DDS_BIN",
            "orbbec_capture": "LINGTU_ORBBEC_CAPTURE_BIN",
        },
    )
    monkeypatch.delenv("LINGTU_CAMERA_DDS_BIN", raising=False)
    monkeypatch.delenv("LINGTU_ORBBEC_CAPTURE_BIN", raising=False)

    native = module.collect_native_binaries()

    assert native["ok"] is False
    assert native["binaries"]["camera_dds"]["exists"] is False
    assert native["binaries"]["orbbec_capture"]["executable"] is False
    assert native["blockers"] == [
        "native_binary_missing_or_not_executable:camera_dds:/missing/lingtu_camera_dds",
        "native_binary_missing_or_not_executable:orbbec_capture:/missing/orbbec_capture",
    ]


def test_thunder_service_readiness_collector_accepts_executable_camera_binaries(monkeypatch, tmp_path):
    module = _load_module()

    camera_dds = tmp_path / "lingtu_camera_dds"
    capture = tmp_path / "orbbec_capture"
    camera_dds.write_text("#!/bin/sh\n", encoding="utf-8")
    capture.write_text("#!/bin/sh\n", encoding="utf-8")

    monkeypatch.setattr(
        module,
        "NATIVE_BINARY_DEFAULTS",
        {
            "camera_dds": str(camera_dds),
            "orbbec_capture": str(capture),
        },
    )
    monkeypatch.setattr(
        module,
        "NATIVE_BINARY_ENV",
        {
            "camera_dds": "LINGTU_CAMERA_DDS_BIN",
            "orbbec_capture": "LINGTU_ORBBEC_CAPTURE_BIN",
        },
    )
    monkeypatch.delenv("LINGTU_CAMERA_DDS_BIN", raising=False)
    monkeypatch.delenv("LINGTU_ORBBEC_CAPTURE_BIN", raising=False)
    monkeypatch.setattr(module.os, "access", lambda path, mode: True)

    native = module.collect_native_binaries()

    assert native["ok"] is True
    assert native["blockers"] == []
    assert native["binaries"]["camera_dds"]["executable"] is True
    assert native["binaries"]["orbbec_capture"]["executable"] is True


def test_thunder_service_readiness_accepts_camera_closed_loop_evidence() -> None:
    module = _load_module()

    camera = module.collect_camera_readiness(
        systemd={
            "lt-camera.service": {
                "missing": False,
                "active_state": "active",
            }
        },
        native_binaries={
            "binaries": {
                "camera_dds": {
                    "path": "/opt/lingtu/current/build/camera_dds/lingtu_camera_dds",
                    "executable": True,
                },
                "orbbec_capture": {
                    "path": "/opt/lingtu/current/build/orbbec_native/orbbec_capture",
                    "executable": True,
                },
            }
        },
        status_files={
            "camera": {
                "path": "/dev/shm/lingtu/camera_status.json",
                "exists": True,
                "json": {
                    "status": "running",
                    "data_plane": "posix_shm",
                    "color_frames": 4,
                    "depth_frames": 4,
                    "info_frames": 1,
                    "color_shm_sequence": 4,
                    "depth_shm_sequence": 4,
                    "info_shm_sequence": 1,
                    "last_error": "",
                },
            }
        },
        dds={
            "checked": True,
            "services": {
                "camera": {
                    "dds_topics": [
                        "rt/camera/info",
                    ],
                    "samples": {
                        "rt/camera/info": 1,
                    },
                }
            },
            "blockers": [],
        },
        processes={"legacy_lines": [], "blockers": []},
    )

    assert camera["ok"] is True
    assert camera["status_frames"] == {
        "color_frames": 4,
        "depth_frames": 4,
        "info_frames": 1,
    }
    assert camera["data_plane"] == "posix_shm"
    assert camera["shm_sequences"] == {
        "color_shm_sequence": 4,
        "depth_shm_sequence": 4,
        "info_shm_sequence": 1,
    }
    assert camera["dds_samples"] == {"rt/camera/info": 1}
    assert camera["legacy_orbbec_observed"] is False
    assert camera["blockers"] == []


def test_thunder_service_readiness_blocks_incomplete_camera_closed_loop() -> None:
    module = _load_module()

    camera = module.collect_camera_readiness(
        systemd={
            "lt-camera.service": {
                "missing": False,
                "active_state": "failed",
            }
        },
        native_binaries={
            "binaries": {
                "camera_dds": {
                    "path": "/missing/lingtu_camera_dds",
                    "executable": False,
                },
                "orbbec_capture": {
                    "path": "/missing/orbbec_capture",
                    "executable": False,
                },
            }
        },
        status_files={
            "camera": {
                "path": "/dev/shm/lingtu/camera_status.json",
                "exists": True,
                "json": {
                    "status": "error",
                    "data_plane": "posix_shm",
                    "color_frames": 0,
                    "depth_frames": 2,
                    "info_frames": 0,
                    "color_shm_sequence": 0,
                    "depth_shm_sequence": 2,
                    "info_shm_sequence": 0,
                    "last_error": "camera disconnected",
                },
            }
        },
        dds={
            "checked": True,
            "services": {
                "camera": {
                    "dds_topics": [
                        "rt/camera/info",
                    ],
                    "samples": {
                        "rt/camera/info": 0,
                    },
                }
            },
            "blockers": [],
        },
        processes={
            "legacy_lines": [" 303 orbbec_camera /opt/ros/humble/lib/orbbec_camera/orbbec_camera_node"],
            "blockers": ["legacy_process_observed:orbbec_camera"],
        },
    )

    assert camera["ok"] is False
    assert camera["blockers"] == [
        "systemd_unit_inactive:lt-camera.service:failed",
        "native_binary_missing_or_not_executable:camera_dds:/missing/lingtu_camera_dds",
        "native_binary_missing_or_not_executable:orbbec_capture:/missing/orbbec_capture",
        "status_no_color_frames:/dev/shm/lingtu/camera_status.json",
        "status_no_info_frames:/dev/shm/lingtu/camera_status.json",
        "status_no_color_shm_sequence:/dev/shm/lingtu/camera_status.json",
        "status_no_info_shm_sequence:/dev/shm/lingtu/camera_status.json",
        "status_error:camera disconnected",
        "dds_topic_silent:rt/camera/info",
        "legacy_process_observed:orbbec_camera",
    ]


def test_thunder_service_readiness_summary_aggregates_blockers() -> None:
    module = _load_module()

    report = {
        "systemd": {
            "lt-camera.service": {
                "missing": True,
                "active_state": "inactive",
            },
            "lt-host.service": {
                "missing": False,
                "active_state": "active",
            },
        },
        "status_files": {
            "camera": {
                "exists": False,
                "path": "/dev/shm/lingtu/camera_status.json",
            }
        },
        "native_binaries": {
            "blockers": [
                "native_binary_missing_or_not_executable:camera_dds:/opt/lingtu/current/build/camera_dds/lingtu_camera_dds"
            ]
        },
        "gnss": {"blockers": ["gnss_device_missing:/dev/wtrtk980"]},
        "gateway": {
            "health": {"ok": True, "status": 200},
            "services_status": {"ok": False, "status": 404},
        },
        "processes": {"blockers": ["legacy_process_observed:livox_ros_driver2"]},
        "camera": {"blockers": ["dds_topic_silent:rt/camera/info"]},
        "dds": {
            "checked": True,
            "blockers": ["dds_topic_silent:rt/camera/color"],
        },
    }

    summary = module.summarize_report(report)

    assert summary["ok"] is False
    assert summary["blocker_count"] == 8
    assert summary["blockers"] == [
        "systemd:unit_missing:lt-camera.service",
        "status_file:missing:camera:/dev/shm/lingtu/camera_status.json",
        "native_binaries:native_binary_missing_or_not_executable:camera_dds:/opt/lingtu/current/build/camera_dds/lingtu_camera_dds",
        "gnss:gnss_device_missing:/dev/wtrtk980",
        "gateway:services_status:404",
        "processes:legacy_process_observed:livox_ros_driver2",
        "camera:dds_topic_silent:rt/camera/info",
        "dds:dds_topic_silent:rt/camera/color",
    ]


def test_driver_readiness_uses_connection_heartbeat_not_idle_cmd_samples() -> None:
    module = _load_module()

    ready = module.collect_driver_readiness(
        systemd={
            "lt-driver.service": {
                "missing": False,
                "active_state": "active",
            }
        },
        native_binaries={
            "binaries": {
                "driver": {
                    "path": "/opt/lingtu/current/build/driver/lingtu_driver",
                    "executable": True,
                }
            }
        },
        status_files={
            "driver": {
                "path": "/dev/shm/lingtu/driver_status.json",
                "exists": True,
                "age_s": 0.5,
                "json": {
                    "schema_version": "lingtu.driver.status.v2",
                    "backend": "doso",
                    "ready": True,
                    "connected": True,
                    "adapter": {
                        "protocol": "brainstem_grpc",
                        "target": "192.168.66.9:50051",
                        "control_owner": "grpc",
                        "control_owner_id": "lingtu-driver@robot",
                    },
                    "control": {
                        "control_assured": True,
                        "fsm": "standing",
                        "motors_enabled": True,
                        "critical_fault": False,
                        "lease_valid": True,
                        "initial_zero_acknowledged": True,
                        "decision": "none",
                    },
                },
            }
        },
    )

    assert ready["ok"] is True
    assert ready["dds_topic"] == "rt/nav/cmd_vel"
    assert ready["dds_idle_allowed"] is True
    assert ready["blockers"] == []

    disconnected = module.collect_driver_readiness(
        systemd={
            "lt-driver.service": {
                "missing": False,
                "active_state": "active",
            }
        },
        native_binaries={"binaries": {"driver": {"executable": True}}},
        status_files={
            "driver": {
                "exists": True,
                "age_s": 5.0,
                "json": {
                    "schema_version": "lingtu.driver.status.v2",
                    "ready": False,
                    "connected": False,
                },
            }
        },
    )
    assert disconnected["ok"] is False
    assert disconnected["blockers"] == [
        "driver_not_connected",
        "driver_not_ready",
        "driver_status_stale",
    ]


def test_driver_readiness_requires_motor_enable_and_lingtu_lease() -> None:
    module = _load_module()
    result = module.collect_driver_readiness(
        systemd={
            "lt-driver.service": {
                "missing": False,
                "active_state": "active",
            }
        },
        native_binaries={"binaries": {"driver": {"executable": True}}},
        status_files={
            "driver": {
                "exists": True,
                "age_s": 0.1,
                "json": {
                    "schema_version": "lingtu.driver.status.v2",
                    "backend": "doso",
                    "ready": False,
                    "connected": True,
                    "adapter": {
                        "protocol": "brainstem_grpc",
                        "target": "192.168.66.9:50051",
                        "control_owner": "yunzhuo",
                        "control_owner_id": "",
                    },
                    "control": {
                        "control_assured": False,
                        "fsm": "standing",
                        "motors_enabled": False,
                        "critical_fault": False,
                        "lease_valid": False,
                        "initial_zero_acknowledged": True,
                        "decision": "control_busy",
                    },
                },
            }
        },
    )
    assert result["ok"] is False
    assert result["blockers"] == [
        "driver_backend_control_invalid",
        "driver_motors_disabled",
        "driver_control_not_assured",
        "driver_control_not_owned_by_lingtu",
        "driver_not_ready",
    ]


def test_driver_readiness_accepts_go2_sdk2_control() -> None:
    module = _load_module()
    result = module.collect_driver_readiness(
        systemd={"lt-driver.service": {"missing": False, "active_state": "active"}},
        native_binaries={"binaries": {"driver": {"executable": True}}},
        status_files={
            "driver": {
                "exists": True,
                "age_s": 0.1,
                "json": {
                    "schema_version": "lingtu.driver.status.v2",
                    "backend": "go2",
                    "ready": True,
                    "connected": True,
                    "adapter": {
                        "protocol": "unitree_sdk2",
                        "target": "dds://eth0/rt/api/sport/request",
                        "control_owner": "none",
                        "control_owner_id": "",
                    },
                    "control": {
                        "control_assured": True,
                        "fsm": "standing",
                        "motors_enabled": True,
                        "critical_fault": False,
                        "lease_valid": False,
                        "initial_zero_acknowledged": True,
                    },
                },
            }
        },
    )

    assert result["ok"] is True
    assert result["backend"] == "go2"
    assert result["adapter"]["protocol"] == "unitree_sdk2"


def test_thunder_service_readiness_collector_lists_catalog_dds_topics() -> None:
    module = _load_module()

    report = module.collect_dds(seconds=0.0, domain_id=7)

    assert report["checked"] is False
    assert report["domain_id"] == 7
    assert report["blockers"] == ["dds_unchecked"]
    assert report["services"]["lidar"]["dds_topics"] == [
        "rt/lidar/raw_frame",
        "rt/imu/raw",
    ]
    assert report["services"]["camera"]["dds_topics"] == ["rt/camera/info"]
    assert report["services"]["slam"]["dds_topics"] == [
        "rt/slam/odometry",
        "rt/slam/map_cloud",
        "rt/slam/localization_health",
    ]
    assert report["services"]["traversability"]["dds_topics"] == [
        "rt/nav/traversability",
        "rt/nav/terrain_map",
        "rt/nav/terrain_map_ext",
    ]
    assert report["services"]["driver"]["dds_topics"] == [
        "rt/nav/cmd_vel",
        "rt/driver/control_state",
    ]


def test_driver_dds_topic_may_be_silent_while_idle(monkeypatch) -> None:
    module = _load_module()
    rows = []
    for contract in module.DDS_TOPICS.values():
        for topic in contract["dds_topics"]:
            rows.append(
                {
                    "topic": topic,
                    "samples": 0 if topic == "rt/nav/cmd_vel" else 1,
                    "hz": 0.0 if topic == "rt/nav/cmd_vel" else 10.0,
                }
            )
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda *args, **kwargs: SimpleNamespace(
            returncode=0,
            stdout=json.dumps(rows),
            stderr="",
        ),
    )

    report = module.collect_dds(seconds=0.1, domain_id=7)

    assert report["ok"] is True
    assert "dds_topic_silent:rt/nav/cmd_vel" not in report["blockers"]
    assert report["services"]["driver"]["ok"] is True
    assert report["services"]["driver"]["idle_allowed"] is True

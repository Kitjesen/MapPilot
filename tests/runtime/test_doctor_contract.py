from __future__ import annotations

import io
from types import SimpleNamespace

import pytest

import diagnostics.field.doctor as doctor
from diagnostics.field import service_readiness
from diagnostics.field.doctor import (
    driver_health_blockers,
    parse_args,
    runtime_dataflow_alignment,
)


@pytest.mark.parametrize("api_key", [None, "field-test-key"])
@pytest.mark.parametrize("collector", ["doctor", "service_readiness"])
def test_field_http_uses_configured_gateway_credentials(monkeypatch, api_key, collector):
    if api_key is None:
        monkeypatch.delenv("LINGTU_API_KEY", raising=False)
    else:
        monkeypatch.setenv("LINGTU_API_KEY", api_key)

    class Response(io.BytesIO):
        status = 200

        def getcode(self):
            return self.status

    def open_request(request, *, timeout):
        assert request.full_url == "http://127.0.0.1:5050/api/v1/readiness"
        assert request.get_method() == "GET"
        assert request.get_header("X-api-key") == api_key
        assert timeout == 3.0
        return Response(b'{"ready":true}')

    if collector == "doctor":
        monkeypatch.setattr(doctor.urllib.request, "urlopen", open_request)
        result = doctor.http_json("http://127.0.0.1:5050", "/api/v1/readiness")
        assert result == (200, {"ready": True}, None)
    else:
        monkeypatch.setattr(service_readiness, "urlopen", open_request)
        result = service_readiness._http_json("http://127.0.0.1:5050/api/v1/readiness")
        assert result == {"ok": True, "status": 200, "body": {"ready": True}}
    assert "field-test-key" not in repr(result)


def _collect_non_motion_report(
    monkeypatch, ready_payload, *, lidar_interface: list[str] | None = None, health_payload=None,
    usb_text="", camera_snapshot=False, require_camera=False, camera_driver="orbbec_native",
):
    plan = SimpleNamespace(product="nav", env="real", processes=(),
                           native_process_environment={"LINGTU_CAMERA_DRIVER": camera_driver})
    monkeypatch.setattr(doctor, "current_run_path", lambda: "current.json")
    monkeypatch.setattr(doctor, "load_current_plan", lambda *_args: (plan, {}))

    def http_json(_base, path, **_kwargs):
        if path == "/api/v1/runtime/dataflow":
            return 200, {
                "runtime_boundary": {"product": "nav", "env": "real"},
                "transport_layers": {"native_dds": {"primary": True}},
            }, ""
        if path == "/ready":
            return 200, ready_payload, ""
        if path == "/api/v1/health?details=true" and health_payload is not None:
            return 200, health_payload, ""
        return 503, None, "unavailable"

    monkeypatch.setattr(doctor, "http_json", http_json)
    monkeypatch.setattr(doctor, "run", lambda args, **_kwargs: (0, usb_text if args == ["lsusb"] else "", ""))
    monkeypatch.setattr(
        doctor,
        "netdev_state",
        lambda name: lidar_interface.append(name) or {} if lidar_interface is not None else {},
    )
    class CameraResponse(io.BytesIO):
        headers = {"content-type": "image/jpeg"}

        def getcode(self):
            return 200

    def camera_open(*_args, **_kwargs):
        if camera_snapshot:
            return CameraResponse(b"\xff\xd8\xff")
        raise OSError()

    monkeypatch.setattr(doctor.urllib.request, "urlopen", camera_open)
    monkeypatch.setattr(doctor.glob, "glob", lambda _pattern: [])
    args = ["--non-motion"] + (["--require-camera"] if require_camera else [])
    return doctor.collect_report(doctor.parse_args(args))


@pytest.mark.parametrize("usb_present,snapshot_ok", [(True, True), (False, True), (True, False)])
def test_realsense_rsusb_needs_device_and_capture_but_not_v4l(monkeypatch, usb_present, snapshot_ok):
    monkeypatch.setenv("LINGTU_CAMERA_DRIVER", "orbbec_native")
    report = _collect_non_motion_report(
        monkeypatch, {}, require_camera=True, camera_snapshot=snapshot_ok,
        camera_driver="realsense_native",
        usb_text="Bus 002 Device 002: ID 8086:0b3a Intel RealSense Depth Camera 435i" if usb_present else "",
    )
    checks = {c["id"]: c for c in report["checks"]}
    assert checks["camera.usb"]["status"] == ("pass" if usb_present else "fail")
    assert checks["camera.gateway_snapshot"]["status"] == ("pass" if snapshot_ok else "fail")
    assert checks["camera.video_nodes"]["status"] == "warn"
    assert checks["camera.video_nodes"]["evidence"]["required"] is False


@pytest.mark.parametrize(
    ("hz", "has_odom", "blockers"),
    [(10.0, True, []), (0.0, True, ["slam_hz<1"]), (10.0, False, ["has_odom=false"])],
)
def test_slam_readiness_does_not_depend_on_viewer_point_cache(monkeypatch, hz, has_odom, blockers):
    report = _collect_non_motion_report(
        monkeypatch,
        {},
        health_payload={"status": "ok", "modules_fail": 0, "slam_hz": hz, "map_points": 0, "has_odom": has_odom},
    )
    stream = next(c for c in report["checks"] if c["id"] == "gateway.slam_stream")

    assert stream["evidence"]["blockers"] == blockers
    assert stream["evidence"]["viewer_map_points"] == 0


def test_runtime_dataflow_must_match_current_real_run_plan() -> None:
    plan = SimpleNamespace(product="nav", env="real")
    payload = {
        "runtime_boundary": {
            "product": "nav",
            "env": "real",
        },
        "transport_layers": {
            "native_dds": {
                "primary": True,
            }
        },
    }

    blockers, evidence = runtime_dataflow_alignment(plan, payload)

    assert blockers == []
    assert evidence["expected"] == {"product": "nav", "env": "real"}


def test_runtime_dataflow_rejects_identity_or_transport_mismatch() -> None:
    plan = SimpleNamespace(product="nav", env="real")
    payload = {
        "runtime_boundary": {
            "product": "map",
            "env": "sim",
        },
        "transport_layers": {"native_dds": {"primary": False}},
    }

    blockers, _ = runtime_dataflow_alignment(plan, payload)

    assert blockers == [
        "runtime_boundary_product_mismatch",
        "runtime_boundary_env_mismatch",
        "real_env_native_dds_not_primary",
    ]


def test_driver_health_requires_authoritative_connected_ready_fresh_status() -> None:
    blockers, evidence = driver_health_blockers(
        {
            "brainstem": {
                "source": "lingtu-driver-status",
                "status": "connected",
                "ready": True,
                "stale": False,
            }
        }
    )

    assert blockers == []
    assert evidence["source"] == "lingtu-driver-status"

    blockers, _ = driver_health_blockers(
        {
            "brainstem": {
                "source": "tcp-probe",
                "status": "stale",
                "ready": False,
                "stale": True,
            }
        }
    )
    assert blockers == [
        "native_driver_status_not_authoritative",
        "native_driver_not_connected",
        "native_driver_not_ready",
        "native_driver_status_stale",
    ]


def test_doctor_accepts_cli_gateway_timeout() -> None:
    options = parse_args(["--gateway-timeout-sec", "4.5"])

    assert options.gateway_timeout_sec == 4.5


def test_non_motion_requires_explicit_safe_flag(monkeypatch) -> None:
    for ready_payload in (
        {"ready": True, "data_ready": True},
        {"ready": True, "data_ready": True, "non_motion_safe": False},
        {"ready": True, "data_ready": True, "non_motion_safe": 1},
    ):
        report = _collect_non_motion_report(monkeypatch, ready_payload)
        ready_check = next(check for check in report["checks"] if check["id"] == "gateway.ready")

        assert ready_check["status"] == "fail"
        assert ready_check["evidence"]["non_motion_safe"] is False


def test_non_motion_requires_explicit_data_readiness(monkeypatch) -> None:
    report = _collect_non_motion_report(
        monkeypatch,
        {"ready": True, "non_motion_safe": True},
    )
    ready_check = next(check for check in report["checks"] if check["id"] == "gateway.ready")

    assert ready_check["status"] == "fail"
    assert ready_check["evidence"]["data_ready"] is False


def test_doctor_uses_only_current_livox_interface_variable(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_LIDAR_NETDEV", "legacy0")
    monkeypatch.setenv("LINGTU_LIVOX_NET_IFACE", "livox0")
    selected: list[str] = []

    report = _collect_non_motion_report(
        monkeypatch,
        {"ready": True, "data_ready": True, "non_motion_safe": True},
        lidar_interface=selected,
    )
    carrier_check = next(check for check in report["checks"] if check["id"] == "livox.netdev_carrier")

    assert selected == ["livox0"]
    assert carrier_check["evidence"]["selection"] == {"LINGTU_LIVOX_NET_IFACE": "livox0"}

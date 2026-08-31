from __future__ import annotations

from types import SimpleNamespace

import diagnostics.field.doctor as doctor
from diagnostics.field.doctor import (
    driver_health_blockers,
    parse_args,
    runtime_dataflow_alignment,
)


def _collect_non_motion_report(monkeypatch, ready_payload, *, lidar_interface: list[str] | None = None):
    plan = SimpleNamespace(product="nav", env="real", processes=())
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
        return 503, None, "unavailable"

    monkeypatch.setattr(doctor, "http_json", http_json)
    monkeypatch.setattr(doctor, "run", lambda *_args, **_kwargs: (0, "", ""))
    monkeypatch.setattr(
        doctor,
        "netdev_state",
        lambda name: lidar_interface.append(name) or {} if lidar_interface is not None else {},
    )
    monkeypatch.setattr(doctor.urllib.request, "urlopen", lambda *_args, **_kwargs: (_ for _ in ()).throw(OSError()))
    monkeypatch.setattr(doctor.glob, "glob", lambda _pattern: [])
    return doctor.collect_report(doctor.parse_args(["--non-motion"]))


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

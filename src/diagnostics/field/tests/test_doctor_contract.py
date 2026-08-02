from __future__ import annotations

from types import SimpleNamespace

from diagnostics.field.doctor import (
    driver_health_blockers,
    parse_args,
    runtime_dataflow_alignment,
)


def test_runtime_dataflow_must_match_current_real_run_plan() -> None:
    plan = SimpleNamespace(product="nav", env="real", fingerprint="plan-sha")
    payload = {
        "runtime_boundary": {
            "product": "nav",
            "env": "real",
            "run_plan_fingerprint": "plan-sha",
        },
        "transport_layers": {
            "native_dds": {
                "primary": True,
            }
        },
    }

    blockers, evidence = runtime_dataflow_alignment(plan, payload)

    assert blockers == []
    assert evidence["expected"]["run_plan_fingerprint"] == "plan-sha"


def test_runtime_dataflow_rejects_identity_or_transport_mismatch() -> None:
    plan = SimpleNamespace(product="nav", env="real", fingerprint="plan-sha")
    payload = {
        "runtime_boundary": {
            "product": "map",
            "env": "sim",
            "run_plan_fingerprint": "other-sha",
        },
        "transport_layers": {"native_dds": {"primary": False}},
    }

    blockers, _ = runtime_dataflow_alignment(plan, payload)

    assert blockers == [
        "runtime_boundary_product_mismatch",
        "runtime_boundary_env_mismatch",
        "runtime_boundary_run_plan_fingerprint_mismatch",
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

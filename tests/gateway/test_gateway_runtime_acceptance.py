from __future__ import annotations

import pytest

pytestmark = [pytest.mark.sim]


def _topic(topic: str, *, live: bool) -> dict:
    return {
        "topic": topic,
        "observability": {
            "observable": True,
            "live_module_samples": live,
            "module_port_candidates": [
                {
                    "module": "GatewayModule",
                    "port": "sample",
                    "direction": "in",
                    "msg_count": 1 if live else 0,
                    "rate_hz": 1.0 if live else 0.0,
                    "stale_ms": 10.0 if live else None,
                }
            ],
        },
    }


def _real_evidence(*, ok: bool = True) -> dict:
    return {
        "_http_status": 200,
        "ok": ok,
        "runtime_evidence_ok": ok,
        "runtime_contract": "real",
        "simulation_only": False,
        "real_robot_motion": ok,
        "cmd_vel_sent_to_hardware": ok,
        "report_age_s": 20.0,
        "max_age_s": 3600.0,
    }


def _snapshots(
    *,
    env: str = "real",
    live: bool = False,
    evidence: dict | None = None,
) -> dict:
    simulation = env == "sim"
    topic = "/slam/odometry"
    return {
        "readiness": {
            "_http_status": 200,
            "status": "ready",
            "ready": True,
            "runtime": {
                "summary": {
                    "data_ready": True,
                    "motion_ready": live,
                    "non_motion_safe": True,
                }
            },
        },
        "runtime_dataflow": {
            "_http_status": 200,
            "runtime_contract": "sim" if simulation else "real",
            "run_plan": {"required_topics": [topic]},
            "runtime_boundary": {
                "env": env,
                "data_source": "mujoco" if simulation else "field",
                "simulation_only": simulation,
                "command_sink": "mujoco_velocity_adapter" if simulation else "driver",
            },
            "topics": [_topic(topic, live=live)],
        },
        "localization_status": {
            "_http_status": 200,
            "state": "tracking" if live else "no_odometry",
            "has_odometry": live,
        },
        "navigation_status": {
            "_http_status": 200,
            "state": "IDLE",
            "readiness": {"can_send_goal": live, "blockers": []},
        },
        "real_runtime_evidence": evidence
        or {
            "_http_status": 404,
            "ok": False,
            "reason": "real_runtime_evidence_report_not_found",
        },
    }


def test_non_motion_accepts_available_read_only_gateway_dataflow():
    from diagnostics.field.gateway_acceptance import evaluate_gateway_runtime_acceptance

    payload = evaluate_gateway_runtime_acceptance(_snapshots(), mode="non_motion")

    assert payload["ok"] is True


def test_non_motion_treats_missing_live_data_as_advisory():
    from diagnostics.field.gateway_acceptance import evaluate_gateway_runtime_acceptance

    payload = evaluate_gateway_runtime_acceptance(_snapshots(), mode="non_motion")

    assert payload["blockers"] == []
    assert any("live field samples absent" in item for item in payload["advisories"])


def test_native_only_product_topic_is_not_forced_into_gateway_observability():
    from diagnostics.field.gateway_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots()
    snapshots["runtime_dataflow"]["run_plan"]["required_topics"].append("/product/missing")

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="non_motion")

    observability = payload["checks"]["gateway_observability"]
    assert observability["missing_topics"] == []
    assert payload["ok"] is True


@pytest.mark.parametrize("field", ["data_ready", "non_motion_safe"])
def test_readiness_requires_explicit_non_motion_guarantees(field: str):
    from diagnostics.field.gateway_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots()
    snapshots["readiness"]["runtime"]["summary"].pop(field)

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="non_motion")

    assert payload["ok"] is False
    assert f"readiness requires {field}=true" in payload["blockers"]


def test_simulation_requires_explicit_motion_ready():
    from diagnostics.field.gateway_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(env="sim", live=True)
    snapshots["readiness"]["runtime"]["summary"].pop("motion_ready")

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="simulation")

    assert payload["ok"] is False
    assert "simulation acceptance requires motion_ready=true" in payload["blockers"]


def test_field_accepts_live_gateway_with_real_runtime_evidence():
    from diagnostics.field.gateway_acceptance import evaluate_gateway_runtime_acceptance

    payload = evaluate_gateway_runtime_acceptance(
        _snapshots(live=True, evidence=_real_evidence()),
        mode="field",
    )

    assert payload["ok"] is True
    assert payload["cmd_vel_sent_to_hardware"] is True


def test_field_requires_real_runtime_evidence():
    from diagnostics.field.gateway_acceptance import evaluate_gateway_runtime_acceptance

    payload = evaluate_gateway_runtime_acceptance(_snapshots(live=True), mode="field")

    assert payload["ok"] is False
    assert "field acceptance requires passing real-runtime-evidence" in payload["blockers"]


def test_simulation_rejects_real_hardware_boundary():
    from diagnostics.field.gateway_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(env="sim", live=True)
    snapshots["runtime_dataflow"]["runtime_boundary"]["command_sink"] = "driver"

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="simulation")

    assert payload["ok"] is False
    assert "simulation acceptance must not use hardware command sink" in payload["blockers"]


def test_failed_readiness_blocks_acceptance():
    from diagnostics.field.gateway_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots()
    snapshots["readiness"].update({"status": "not_started", "ready": False})

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="non_motion")

    assert payload["ok"] is False
    assert "readiness status is not_started" in payload["blockers"]


def test_lost_localization_blocks_field_acceptance():
    from diagnostics.field.gateway_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(live=True, evidence=_real_evidence())
    snapshots["localization_status"]["state"] = "lost"

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="field")

    assert payload["ok"] is False
    assert "field acceptance localization state is lost" in payload["blockers"]


def test_missing_localization_blocks_field_product_that_requires_odometry():
    from diagnostics.field.gateway_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(live=True, evidence=_real_evidence())
    snapshots["localization_status"] = {"_http_status": 200}

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="field")

    assert payload["ok"] is False
    assert "field acceptance localization state is unknown" in payload["blockers"]


def test_map_free_teleop_does_not_require_localization():
    from diagnostics.field.gateway_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(live=True, evidence=_real_evidence())
    snapshots["runtime_dataflow"]["run_plan"]["required_topics"] = ["/nav/cmd_vel"]
    snapshots["runtime_dataflow"]["topics"] = [_topic("/nav/cmd_vel", live=True)]
    snapshots["localization_status"] = {"_http_status": 200}

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="field")

    assert payload["ok"] is True
    assert payload["checks"]["localization"]["required"] is False


def test_gateway_channel_topic_does_not_require_module_port_samples():
    from diagnostics.field.gateway_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(live=True, evidence=_real_evidence())
    snapshots["runtime_dataflow"]["run_plan"]["required_topics"] = ["/nav/status"]
    snapshots["runtime_dataflow"]["topics"] = [
        {
            "topic": "/nav/status",
            "observability": {
                "observable": True,
                "gateway_channels": [
                    {"transport": "gateway_rest", "path": "/api/v1/navigation/status"}
                ],
                "module_port_candidates": [],
            },
        }
    ]

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="field")

    assert payload["ok"] is True
    assert payload["checks"]["gateway_observability"]["missing_live_topics"] == []


@pytest.mark.parametrize(
    ("age", "max_age"),
    [(-1.0, 60.0), (1.0, -1.0), (float("inf"), 60.0), (1.0, float("nan"))],
)
def test_field_rejects_invalid_evidence_age(age: float, max_age: float):
    from diagnostics.field.gateway_acceptance import evaluate_gateway_runtime_acceptance

    evidence = _real_evidence()
    evidence.update({"report_age_s": age, "max_age_s": max_age})

    payload = evaluate_gateway_runtime_acceptance(
        _snapshots(live=True, evidence=evidence),
        mode="field",
    )

    assert payload["ok"] is False
    assert "real-runtime-evidence age is invalid" in payload["blockers"]


def test_navigation_blockers_block_field_acceptance():
    from diagnostics.field.gateway_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(live=True, evidence=_real_evidence())
    snapshots["navigation_status"]["readiness"] = {
        "can_send_goal": False,
        "blockers": ["map unavailable"],
    }

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="field")

    assert payload["ok"] is False
    assert "navigation readiness blockers: map unavailable" in payload["blockers"]

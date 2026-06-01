from __future__ import annotations

import json
import os
import sys
from types import SimpleNamespace

import pytest

pytestmark = [pytest.mark.sim]


def _topic_entry(
    topic: str,
    *,
    live: bool = False,
    stale_ms: float | None = None,
) -> dict:
    event_type = topic.rsplit("/", 1)[-1]
    latest_payload = None
    if live and topic == "/nav/frontier_candidate":
        latest_payload = {
            "source": "traversable_frontier",
            "centroid_3d": [1.0, 2.0, 0.25],
            "reachable_score": 0.82,
            "support_type": "flat",
            "reasons": [],
            "preview": True,
            "command_published": False,
        }
    elif live and topic == "/nav/traversable_frontiers":
        latest_payload = [
            {
                "source": "traversable_frontier",
                "centroid_3d": [1.0, 2.0, 0.25],
                "reachable_score": 0.82,
                "support_type": "flat",
                "reasons": [],
                "preview": True,
                "command_published": False,
            }
        ]
    entry = {
        "topic": topic,
        "observability": {
            "observable": True,
            "observable_via": ["module_port_bus", "gateway_rest", "gateway_sse"],
            "module_port_candidates": [
                {
                    "module": "GatewayModule",
                    "port": topic.rsplit("/", 1)[-1],
                    "direction": "in",
                    "msg_count": 1 if live else 0,
                    "rate_hz": 1.0 if live else 0.0,
                    "stale_ms": stale_ms,
                }
            ],
            "gateway_channels": [
                {"transport": "gateway_rest"},
                {
                    "transport": "gateway_sse",
                    "path": "/api/v1/events",
                    "event_type": event_type,
                },
            ],
            "live_module_samples": live,
            "ros2_topic_required": False,
        },
        "communication": {
            "allowed": topic in {"/nav/cmd_vel", "/nav/goal_pose"},
            "interfaces": (
                [{"path": "/api/v1/cmd_vel"}] if topic == "/nav/cmd_vel" else []
            ),
            "arbitrary_publish_supported": False,
            "policy": "command_whitelist_only",
        },
        "inspection": {
            "ros2_topic_required": False,
            "arbitrary_publish_supported": False,
            "stream_interfaces": [
                {
                    "transport": "gateway_sse",
                    "path": "/api/v1/events",
                    "event_type": event_type,
                }
            ],
        },
    }
    if latest_payload is not None:
        entry["inspection"]["latest_payload"] = latest_payload
        entry["inspection"]["payload_sample_available"] = True
    else:
        entry["inspection"]["payload_sample_available"] = False
    return entry


def _stage_entry(
    name: str,
    *,
    live: bool,
    inputs: tuple[str, ...],
    outputs: tuple[str, ...],
    missing_inputs: tuple[str, ...] = (),
    missing_outputs: tuple[str, ...] = (),
) -> dict:
    not_live_inputs = () if live else inputs
    not_live_outputs = () if live else outputs
    status = "live" if live else "metadata_only"
    if missing_inputs or missing_outputs:
        status = "missing"
    return {
        "name": name,
        "owner": f"{name}_owner",
        "frame_role": "map",
        "map_dependency": "runtime_contract",
        "inputs": list(inputs),
        "outputs": list(outputs),
        "input_evidence": [],
        "output_evidence": [],
        "observable": not missing_inputs and not missing_outputs,
        "live": live,
        "status": status,
        "missing_inputs": list(missing_inputs),
        "missing_outputs": list(missing_outputs),
        "not_live_inputs": list(not_live_inputs),
        "not_live_outputs": list(not_live_outputs),
    }


def _stage_evidence(*, live: bool) -> list[dict]:
    return [
        _stage_entry(
            "endpoint_adapter",
            live=live,
            inputs=("/nav/lidar_scan", "/nav/imu"),
            outputs=("/nav/lidar_scan", "/nav/imu"),
        ),
        _stage_entry(
            "slam_or_relayed_localization_map",
            live=live,
            inputs=("/nav/lidar_scan", "/nav/imu"),
            outputs=("/nav/odometry", "/nav/map_cloud"),
        ),
        _stage_entry(
            "map_layers_and_exploration",
            live=live,
            inputs=("/nav/odometry", "/nav/map_cloud"),
            outputs=("/nav/goal_pose",),
        ),
        _stage_entry(
            "traversable_frontier_preview",
            live=live,
            inputs=("/nav/odometry", "/nav/exploration_grid"),
            outputs=("/nav/traversable_frontiers", "/nav/frontier_candidate"),
        ),
        _stage_entry(
            "global_planning",
            live=live,
            inputs=("/nav/odometry", "/nav/map_cloud", "/nav/goal_pose"),
            outputs=("/nav/global_path",),
        ),
        _stage_entry(
            "local_planning_and_following",
            live=live,
            inputs=("/nav/odometry", "/nav/global_path"),
            outputs=("/nav/local_path", "/nav/cmd_vel"),
        ),
        _stage_entry(
            "dynamic_obstacle_gate",
            live=live,
            inputs=("/nav/added_obstacles", "/nav/local_path", "/nav/cmd_vel"),
            outputs=("/nav/check_obstacle", "/nav/planner_status"),
        ),
        _stage_entry(
            "command_boundary",
            live=live,
            inputs=("/nav/cmd_vel",),
            outputs=("hardware_driver_after_cmd_vel_mux",),
        ),
    ]


def _real_runtime_evidence_snapshot(
    *,
    ok: bool = True,
    age_s: float = 30.0,
) -> dict:
    return {
        "_http_status": 200,
        "ok": ok,
        "runtime_evidence_ok": ok,
        "report_age_s": age_s,
        "max_age_s": 3600.0,
        "runtime_contract": "real_s100p",
        "simulation_only": False,
        "real_robot_motion": True,
        "cmd_vel_sent_to_hardware": True,
        "checked_real_motion_evidence": {"ok": True},
        "checked_hardware_boundary_evidence": {"ok": True},
        "checked_live_topic_freshness": {"/nav/odometry": {"ok": True}},
        "checked_runtime_data_flow_evidence": {
            "command_boundary": {"ok": True},
        },
    }


def _snapshots(
    *,
    live: bool = False,
    mode: str = "field",
    real_evidence: dict | None = None,
) -> dict:
    from core.gateway_runtime_acceptance import PRODUCT_OBSERVABLE_TOPICS

    simulation = mode == "simulation"
    runtime_contract = "mujoco_fastlio2_live" if simulation else "real_s100p"

    return {
        "capabilities": {
            "_http_status": 200,
            "links": {
                "runtime_dataflow": "/api/v1/runtime/dataflow",
                "runtime_dataflow_topic": "/api/v1/runtime/dataflow/topic",
                "runtime_dataflow_subscribe": "/api/v1/runtime/dataflow/subscribe",
                "runtime_switch_plan": "/api/v1/runtime/switch-plan",
                "diagnostic_pack": "/api/v1/diagnostic_pack",
                "field_check": "/api/v1/diagnostics/field-check",
                "inspection_acceptance": "/api/v1/inspection/acceptance",
                "routecheck_latest": "/api/v1/diagnostics/routecheck/latest",
                "real_runtime_evidence_latest": (
                    "/api/v1/diagnostics/real-runtime-evidence/latest"
                ),
                "algorithm_benchmark_latest": (
                    "/api/v1/diagnostics/algorithm-benchmark/latest"
                ),
                "readiness": "/api/v1/readiness",
                "navigation_status": "/api/v1/navigation/status",
                "localization_status": "/api/v1/localization/status",
                "navigation_goal_candidate": "/api/v1/navigation/goal_candidate",
            },
        },
        "readiness": {
            "_http_status": 200,
            "status": "ready",
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
            "runtime_contract": runtime_contract,
            "runtime_boundary": {
                "endpoint": "mujoco_live" if simulation else "real_s100p",
                "data_source": runtime_contract,
                "runtime_contract": runtime_contract,
                "simulation_only": simulation,
                "command_sink": (
                    "mujoco_velocity_adapter"
                    if simulation
                    else "hardware_driver_after_cmd_vel_mux"
                ),
            },
            "ros2_topic_required": False,
            "transport_layers": {
                "module_port_bus": {"primary": True},
                "ros2_adapter": {"primary": False},
            },
            "control_boundary": {
                "arbitrary_publish_supported": False,
                "command_interfaces": [
                    {"path": "/api/v1/goal"},
                    {"path": "/api/v1/cmd_vel"},
                    {"path": "/api/v1/stop"},
                ],
            },
            "topics": [
                _topic_entry(topic, live=live)
                for topic in PRODUCT_OBSERVABLE_TOPICS
            ],
            "stage_evidence": _stage_evidence(live=live),
            "links": {
                "runtime_dataflow_subscribe": "/api/v1/runtime/dataflow/subscribe",
            },
        },
        "localization_status": {
            "_http_status": 200,
            "state": "ready" if live else "no_odometry",
            "has_odometry": live,
        },
        "navigation_status": {
            "_http_status": 200,
            "state": "IDLE",
            "readiness": {"can_send_goal": live, "blockers": []},
        },
        "routecheck_latest": {
            "_http_status": 200,
            "ok": True,
            "outcome": "pass",
            "non_motion": True,
            "gateway_used": True,
            "driver_used": False,
            "published": {"goal_pose": 0, "cmd_vel": 0, "stop_cmd": 0},
            "latest": {
                "outcome": "pass",
                "non_motion": True,
                "gateway_used": True,
                "driver_used": False,
                "published": {"goal_pose": 0, "cmd_vel": 0, "stop_cmd": 0},
            },
        },
        "real_runtime_evidence": real_evidence or {
            "_http_status": 200,
            "ok": False,
            "reason": "real_runtime_evidence_report_not_found",
        },
    }


def test_gateway_runtime_acceptance_passes_non_motion_without_ros2_topic():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    payload = evaluate_gateway_runtime_acceptance(_snapshots(), mode="non_motion")

    assert payload["ok"] is True
    assert payload["runtime_contract"] == "real_s100p"
    assert payload["ros2_topic_required"] is False
    assert payload["checks"]["module_first_dataflow"]["module_port_bus_primary"] is True
    assert payload["checks"]["module_first_dataflow"]["ros2_adapter_primary"] is False
    assert payload["checks"]["module_first_dataflow"]["missing_live_topics"]
    assert payload["checks"]["stage_evidence"]["ok"] is True
    assert payload["checks"]["stage_evidence"]["stage_count"] == 8
    assert payload["checks"]["stage_evidence"]["not_live_stages"] == []
    assert payload["checks"]["frontier_preview"]["ok"] is True
    assert payload["checks"]["frontier_preview"]["read_only"] is True
    assert payload["checks"]["frontier_preview"]["command_published"] is False
    assert "live field samples absent" in payload["advisories"][0]


def test_gateway_runtime_acceptance_non_motion_exposes_top_level_sim_safety_flags():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    payload = evaluate_gateway_runtime_acceptance(
        _snapshots(mode="simulation"),
        mode="non_motion",
    )

    assert payload["ok"] is True
    assert payload["simulation_only"] is True
    assert payload["real_robot_motion"] is False
    assert payload["cmd_vel_sent_to_hardware"] is False


def test_gateway_runtime_acceptance_fetches_client_readiness_snapshot():
    from core.gateway_runtime_acceptance import GATEWAY_ACCEPTANCE_ENDPOINTS

    assert GATEWAY_ACCEPTANCE_ENDPOINTS["readiness"] == "/api/v1/readiness"


def test_gateway_runtime_acceptance_non_motion_uses_local_stub_when_gateway_is_down(
    monkeypatch,
):
    import core.gateway_runtime_acceptance as acceptance_mod

    def _offline_fetch(base_url, name, path, timeout_sec):
        return acceptance_mod.GatewayFetchResult(
            name=name,
            path=path,
            ok=False,
            status=None,
            payload={"_fetch_error": "connection refused"},
            error="connection refused",
        )

    monkeypatch.setattr(acceptance_mod, "_fetch_json", _offline_fetch)

    payload = acceptance_mod.collect_gateway_runtime_acceptance(
        gateway_url=acceptance_mod.DEFAULT_GATEWAY_URL,
        timeout_sec=0.01,
        mode="non_motion",
    )

    assert payload["ok"] is True
    assert payload["mode"] == "non_motion"
    assert payload["runtime_contract"] == "in_process_stub"
    assert payload["snapshot_source"] == "in_process_stub"
    assert payload["simulation_only"] is True
    assert payload["real_robot_motion"] is False
    assert payload["cmd_vel_sent_to_hardware"] is False
    assert payload["ros2_topic_required"] is False
    assert payload["checks"]["gateway_contract"]["ok"] is True
    assert payload["checks"]["readiness"]["non_motion_safe"] is True


def test_gateway_runtime_acceptance_field_does_not_use_local_stub_when_gateway_is_down(
    monkeypatch,
):
    import core.gateway_runtime_acceptance as acceptance_mod

    def _offline_fetch(base_url, name, path, timeout_sec):
        return acceptance_mod.GatewayFetchResult(
            name=name,
            path=path,
            ok=False,
            status=None,
            payload={"_fetch_error": "connection refused"},
            error="connection refused",
        )

    monkeypatch.setattr(acceptance_mod, "_fetch_json", _offline_fetch)

    payload = acceptance_mod.collect_gateway_runtime_acceptance(
        gateway_url=acceptance_mod.DEFAULT_GATEWAY_URL,
        timeout_sec=0.01,
        mode="field",
    )

    assert payload["ok"] is False
    assert payload.get("snapshot_source") != "in_process_stub"
    assert "gateway endpoint unavailable: capabilities" in "\n".join(
        payload["blockers"]
    )


def test_gateway_runtime_acceptance_in_process_stub_stays_non_motion(monkeypatch):
    import core.gateway_runtime_acceptance as acceptance_mod
    from core.utils.blackbox_recorder import BlackBoxRecorder
    from gateway.gateway_module import GatewayModule

    start_calls = []
    blackbox_env = []

    def _fail_start(self):
        start_calls.append("start")
        raise AssertionError("in-process fallback must not start GatewayModule")

    def _fake_blackbox_from_env(cls):
        blackbox_env.append(os.environ.get("LINGTU_BLACKBOX_ENABLED"))
        return SimpleNamespace(record=lambda *args, **kwargs: None)

    monkeypatch.setattr(GatewayModule, "start", _fail_start)
    monkeypatch.setattr(BlackBoxRecorder, "from_env", classmethod(_fake_blackbox_from_env))
    monkeypatch.setenv("LINGTU_BLACKBOX_ENABLED", "1")

    snapshots = acceptance_mod._collect_in_process_stub_gateway_snapshots()

    ports_out = snapshots["runtime_dataflow"]["module_ports"]["GatewayModule"][
        "ports_out"
    ]
    for port_name in ("goal_pose", "cmd_vel", "stop_cmd", "instruction"):
        assert ports_out[port_name]["msg_count"] == 0
    assert start_calls == []
    assert blackbox_env == ["0"]
    assert os.environ["LINGTU_BLACKBOX_ENABLED"] == "1"


def test_gateway_runtime_acceptance_field_requires_live_samples():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    payload = evaluate_gateway_runtime_acceptance(_snapshots(), mode="field")

    assert payload["ok"] is False
    assert "field acceptance missing live Module samples" in "\n".join(
        payload["blockers"]
    )
    assert "field acceptance localization state is no_odometry" in payload["blockers"]
    assert "field acceptance requires navigation can_send_goal=true" in payload[
        "blockers"
    ]


def test_gateway_runtime_acceptance_field_passes_with_live_samples():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    payload = evaluate_gateway_runtime_acceptance(
        _snapshots(live=True, real_evidence=_real_runtime_evidence_snapshot()),
        mode="field",
    )

    assert payload["ok"] is True
    assert payload["blockers"] == []
    assert payload["checks"]["readiness"]["motion_ready"] is True
    assert payload["checks"]["navigation"]["can_send_goal"] is True
    assert payload["checks"]["stage_evidence"]["ok"] is True
    assert payload["checks"]["stage_evidence"]["not_live_stages"] == []
    assert payload["checks"]["frontier_preview"]["ok"] is True
    assert payload["checks"]["frontier_preview"]["candidate_source"] == "traversable_frontier"


def test_gateway_runtime_acceptance_field_rejects_partially_live_stage_tokens():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(
        live=True,
        real_evidence=_real_runtime_evidence_snapshot(),
    )
    stages = {
        stage["name"]: stage
        for stage in snapshots["runtime_dataflow"]["stage_evidence"]
    }
    stages["local_planning_and_following"]["not_live_inputs"] = [
        "/nav/registered_cloud"
    ]
    stages["local_planning_and_following"]["not_live_outputs"] = []

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="field")

    assert payload["ok"] is False
    assert payload["checks"]["stage_evidence"]["not_live_stages"] == [
        "local_planning_and_following"
    ]
    assert payload["checks"]["stage_evidence"]["not_live_stage_tokens"] == {
        "local_planning_and_following": {
            "inputs": ["/nav/registered_cloud"],
            "outputs": [],
        }
    }
    assert "field acceptance requires live dataflow stages" in "\n".join(
        payload["blockers"]
    )


def test_gateway_runtime_acceptance_format_includes_stage_evidence():
    from core.gateway_runtime_acceptance import (
        evaluate_gateway_runtime_acceptance,
        format_gateway_runtime_acceptance,
    )

    payload = evaluate_gateway_runtime_acceptance(
        _snapshots(live=True, real_evidence=_real_runtime_evidence_snapshot()),
        mode="field",
    )

    output = format_gateway_runtime_acceptance(payload)

    assert "Gateway runtime acceptance: PASS" in output
    assert "ROS2 topic required: false" in output
    assert "Stage evidence: ok=true stages=8 live=" in output
    assert "Frontier preview: ok=true source=traversable_frontier command_published=false" in output
    assert "global_planning" in output


def test_gateway_runtime_acceptance_simulation_passes_without_real_runtime_evidence():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    payload = evaluate_gateway_runtime_acceptance(
        _snapshots(live=True, mode="simulation"),
        mode="simulation",
    )

    assert payload["ok"] is True
    assert payload["runtime_contract"] == "mujoco_fastlio2_live"
    assert payload["checks"]["runtime_mode"]["simulation_only"] is True
    assert payload["checks"]["real_runtime_evidence"]["required"] is False


def test_gateway_runtime_acceptance_simulation_rejects_real_runtime_boundary():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    payload = evaluate_gateway_runtime_acceptance(
        _snapshots(live=True, real_evidence=_real_runtime_evidence_snapshot()),
        mode="simulation",
    )

    assert payload["ok"] is False
    assert "simulation acceptance must not run against real_s100p runtime" in payload[
        "blockers"
    ]
    assert "simulation acceptance requires simulation_only=true" in payload["blockers"]


def test_gateway_runtime_acceptance_simulation_rejects_hardware_sink():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(live=True, mode="simulation")
    snapshots["runtime_dataflow"]["runtime_boundary"][
        "command_sink"
    ] = "hardware_driver_after_cmd_vel_mux"

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="simulation")

    assert payload["ok"] is False
    assert "simulation acceptance must not use hardware command sink" in payload[
        "blockers"
    ]


def test_gateway_runtime_acceptance_simulation_rejects_failed_routecheck():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(live=True, mode="simulation")
    snapshots["routecheck_latest"]["latest"]["outcome"] = "fail"
    snapshots["routecheck_latest"]["outcome"] = "fail"

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="simulation")

    assert payload["ok"] is False
    assert "simulation acceptance requires passing latest routecheck" in "\n".join(
        payload["blockers"]
    )
    assert "latest routecheck outcome is fail" in "\n".join(payload["blockers"])
    assert payload["checks"]["routecheck_latest"]["ok"] is False


def test_gateway_runtime_acceptance_field_rejects_routecheck_with_motion_publish():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(
        live=True,
        real_evidence=_real_runtime_evidence_snapshot(),
    )
    snapshots["routecheck_latest"]["published"]["cmd_vel"] = 1

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="field")

    assert payload["ok"] is False
    assert "field acceptance requires passing latest routecheck" in "\n".join(
        payload["blockers"]
    )
    assert "latest routecheck published.cmd_vel is not 0" in "\n".join(
        payload["blockers"]
    )
    assert payload["checks"]["routecheck_latest"]["published"]["cmd_vel"] == 1


def test_gateway_runtime_acceptance_non_motion_keeps_weak_routecheck_advisory():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots()
    snapshots["routecheck_latest"]["published"].pop("stop_cmd")

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="non_motion")

    assert payload["ok"] is True
    assert "latest routecheck published.stop_cmd is missing" in "\n".join(
        payload["advisories"]
    )
    assert payload["checks"]["routecheck_latest"]["ok"] is False


def test_gateway_runtime_acceptance_rejects_runtime_boundary_blockers():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots()
    snapshots["runtime_dataflow"]["runtime_boundary"]["ok"] = False
    snapshots["runtime_dataflow"]["runtime_boundary"]["blockers"] = [
        "command_sink_env_mismatch"
    ]

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="non_motion")

    assert payload["ok"] is False
    assert "runtime boundary blockers: command_sink_env_mismatch" in payload[
        "blockers"
    ]


def test_gateway_runtime_acceptance_requires_product_inspection_links():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(live=True, real_evidence=_real_runtime_evidence_snapshot())
    links = snapshots["capabilities"]["links"]
    links.pop("runtime_dataflow_topic")
    links.pop("runtime_dataflow_subscribe")
    links.pop("runtime_switch_plan")
    links.pop("field_check")
    links.pop("inspection_acceptance")
    links.pop("algorithm_benchmark_latest")
    links.pop("navigation_goal_candidate")

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="field")

    assert payload["ok"] is False
    assert payload["checks"]["gateway_contract"]["missing_links"] == [
        "runtime_dataflow_topic",
        "runtime_dataflow_subscribe",
        "runtime_switch_plan",
        "field_check",
        "inspection_acceptance",
        "algorithm_benchmark_latest",
        "navigation_goal_candidate",
    ]
    assert "capabilities missing product Gateway links" in "\n".join(
        payload["blockers"]
    )


def test_gateway_runtime_acceptance_rejects_missing_subscribe_link_only():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(live=True, real_evidence=_real_runtime_evidence_snapshot())
    snapshots["capabilities"]["links"].pop("runtime_dataflow_subscribe")

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="field")

    assert payload["ok"] is False
    assert payload["checks"]["gateway_contract"]["missing_links"] == [
        "runtime_dataflow_subscribe"
    ]
    assert "capabilities missing product Gateway links" in "\n".join(
        payload["blockers"]
    )


def test_gateway_runtime_acceptance_field_requires_real_runtime_evidence():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    payload = evaluate_gateway_runtime_acceptance(_snapshots(live=True), mode="field")

    assert payload["ok"] is False
    assert "field acceptance requires passing real-runtime-evidence" in "\n".join(
        payload["blockers"]
    )


def test_gateway_runtime_acceptance_field_rejects_smoke_shape_without_real_evidence():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    evidence = _real_runtime_evidence_snapshot(ok=False)
    evidence.update(
        {
            "_http_status": 200,
            "runtime_contract": "real_s100p",
            "simulation_only": False,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "checked_real_motion_evidence": {"ok": False},
            "checked_hardware_boundary_evidence": {"ok": False},
            "blockers": ["real motion evidence missing"],
        }
    )

    payload = evaluate_gateway_runtime_acceptance(
        _snapshots(live=True, real_evidence=evidence),
        mode="field",
    )

    assert payload["ok"] is False
    assert payload["checks"]["stage_evidence"]["ok"] is True
    assert payload["checks"]["real_runtime_evidence"]["ok"] is False
    assert "field acceptance requires passing real-runtime-evidence" in "\n".join(
        payload["blockers"]
    )
    assert "real-runtime-evidence: real motion evidence missing" in "\n".join(
        payload["blockers"]
    )


def test_gateway_runtime_acceptance_field_rejects_stale_real_runtime_evidence():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    payload = evaluate_gateway_runtime_acceptance(
        _snapshots(
            live=True,
            real_evidence=_real_runtime_evidence_snapshot(age_s=7200.0),
        ),
        mode="field",
    )

    assert payload["ok"] is False
    assert "real-runtime-evidence is stale" in "\n".join(payload["blockers"])


def test_gateway_runtime_acceptance_rejects_ros2_as_primary_boundary():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(live=True)
    dataflow = snapshots["runtime_dataflow"]
    dataflow["ros2_topic_required"] = True
    dataflow["transport_layers"]["module_port_bus"]["primary"] = False
    dataflow["transport_layers"]["ros2_adapter"]["primary"] = True
    dataflow["control_boundary"]["arbitrary_publish_supported"] = True

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="field")

    assert payload["ok"] is False
    assert "Gateway acceptance must not require ros2 topic" in payload["blockers"]
    assert "module_port_bus must be the primary dataflow boundary" in payload[
        "blockers"
    ]
    assert "ros2_adapter must not be the primary acceptance boundary" in payload[
        "blockers"
    ]
    assert "Gateway must not expose arbitrary publish as product control" in payload[
        "blockers"
    ]


def test_gateway_runtime_acceptance_requires_gateway_sse_for_product_streams():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(live=True, real_evidence=_real_runtime_evidence_snapshot())
    odom_topic = snapshots["runtime_dataflow"]["topics"][0]
    odom_topic["observability"]["gateway_channels"] = [
        {"transport": "gateway_rest"}
    ]
    odom_topic["inspection"]["stream_interfaces"] = []

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="field")

    assert payload["ok"] is False
    assert payload["checks"]["module_first_dataflow"][
        "missing_stream_interfaces"
    ] == [odom_topic["topic"]]
    assert "runtime dataflow topics missing Gateway SSE subscription" in "\n".join(
        payload["blockers"]
    )


def test_gateway_runtime_acceptance_rejects_not_started_readiness_even_if_http_200():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots()
    snapshots["readiness"] = {
        "_http_status": 200,
        "status": "not_started",
        "ready": False,
        "data_ready": False,
        "motion_ready": False,
        "non_motion_safe": True,
        "reasons": ["no_modules_loaded"],
    }

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="non_motion")

    assert payload["ok"] is False
    assert "readiness status is not_started" in payload["blockers"]
    assert "readiness reports data_ready=false" in payload["blockers"]
    assert payload["checks"]["readiness"]["data_ready"] is False


def test_gateway_runtime_acceptance_field_rejects_live_flag_without_module_samples():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(
        live=True,
        real_evidence=_real_runtime_evidence_snapshot(),
    )
    odom_topic = snapshots["runtime_dataflow"]["topics"][0]
    odom_topic["observability"]["live_module_samples"] = True
    odom_topic["observability"]["module_port_candidates"] = [
        {
            "module": "GatewayModule",
            "port": "odometry",
            "direction": "in",
            "msg_count": 0,
            "rate_hz": 0.0,
        }
    ]

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="field")

    assert payload["ok"] is False
    assert snapshots["runtime_dataflow"]["topics"][0]["topic"] in payload["checks"][
        "module_first_dataflow"
    ]["missing_live_topics"]
    assert "field acceptance missing live Module samples" in "\n".join(
        payload["blockers"]
    )


def test_gateway_runtime_acceptance_field_rejects_gateway_only_live_sample():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(
        live=True,
        real_evidence=_real_runtime_evidence_snapshot(),
    )
    odom_topic = snapshots["runtime_dataflow"]["topics"][0]
    odom_topic["observability"]["observable_via"] = ["gateway_rest"]
    odom_topic["observability"]["module_port_candidates"] = []
    odom_topic["observability"]["live_module_samples"] = True

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="field")

    assert payload["ok"] is False
    assert snapshots["runtime_dataflow"]["topics"][0]["topic"] in payload["checks"][
        "module_first_dataflow"
    ]["missing_live_topics"]


def test_gateway_runtime_acceptance_field_rejects_stale_module_sample():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(
        live=True,
        real_evidence=_real_runtime_evidence_snapshot(),
    )
    odom_topic = snapshots["runtime_dataflow"]["topics"][0]
    odom_topic["observability"]["module_port_candidates"][0]["stale_ms"] = 5000.0

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="field")

    assert payload["ok"] is False
    assert snapshots["runtime_dataflow"]["topics"][0]["topic"] in payload["checks"][
        "module_first_dataflow"
    ]["missing_live_topics"]


def test_gateway_runtime_acceptance_rejects_incomplete_command_whitelist():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots()
    snapshots["runtime_dataflow"]["control_boundary"]["command_interfaces"] = [
        {"path": "/api/v1/debug/publish"}
    ]

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="non_motion")

    assert payload["ok"] is False
    assert "Gateway command whitelist missing required interfaces" in "\n".join(
        payload["blockers"]
    )
    assert payload["checks"]["module_first_dataflow"]["command_interface_count"] == 1


def test_gateway_runtime_acceptance_rejects_unexpected_command_interfaces():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots()
    snapshots["runtime_dataflow"]["control_boundary"]["command_interfaces"].append(
        {"path": "/api/v1/debug/publish"}
    )

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="non_motion")

    assert payload["ok"] is False
    assert "Gateway command whitelist includes unexpected interfaces" in "\n".join(
        payload["blockers"]
    )
    assert payload["checks"]["module_first_dataflow"][
        "unexpected_command_interfaces"
    ] == ["/api/v1/debug/publish"]


def test_gateway_runtime_acceptance_rejects_missing_stage_evidence():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots()
    snapshots["runtime_dataflow"].pop("stage_evidence")

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="non_motion")

    assert payload["ok"] is False
    assert "runtime dataflow missing stage_evidence" in payload["blockers"]
    assert payload["checks"]["stage_evidence"]["stage_count"] == 0
    assert payload["checks"]["stage_evidence"]["available"] is False


def test_gateway_runtime_acceptance_rejects_missing_required_stage():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots()
    snapshots["runtime_dataflow"]["stage_evidence"] = [
        stage
        for stage in snapshots["runtime_dataflow"]["stage_evidence"]
        if stage["name"] != "global_planning"
    ]

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="non_motion")

    assert payload["ok"] is False
    assert "runtime dataflow missing required stages: global_planning" in payload[
        "blockers"
    ]
    assert payload["checks"]["stage_evidence"]["missing_stages"] == [
        "global_planning"
    ]


def test_gateway_runtime_acceptance_non_motion_keeps_stage_missing_inputs_advisory():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots()
    stages = snapshots["runtime_dataflow"]["stage_evidence"]
    global_stage = next(stage for stage in stages if stage["name"] == "global_planning")
    global_stage["missing_inputs"] = ["/nav/map_cloud"]
    global_stage["observable"] = False
    global_stage["status"] = "missing"

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="non_motion")

    assert payload["ok"] is True
    assert "runtime stage evidence missing tokens: global_planning" not in payload[
        "blockers"
    ]
    assert "runtime stage evidence missing tokens: global_planning" in payload[
        "advisories"
    ]
    assert payload["checks"]["stage_evidence"]["ok"] is True
    assert payload["checks"]["stage_evidence"]["missing_tokens"] == {
        "global_planning": {"inputs": ["/nav/map_cloud"], "outputs": []}
    }


def test_gateway_runtime_acceptance_field_rejects_stage_missing_inputs():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(
        live=True,
        real_evidence=_real_runtime_evidence_snapshot(),
    )
    stages = snapshots["runtime_dataflow"]["stage_evidence"]
    global_stage = next(stage for stage in stages if stage["name"] == "global_planning")
    global_stage["missing_inputs"] = ["/nav/map_cloud"]
    global_stage["observable"] = False
    global_stage["status"] = "missing"

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="field")

    assert payload["ok"] is False
    assert "runtime stage evidence missing tokens: global_planning" in payload[
        "blockers"
    ]
    assert payload["checks"]["stage_evidence"]["ok"] is False


def test_gateway_runtime_acceptance_rejects_missing_tomogram_artifact():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots()
    stages = snapshots["runtime_dataflow"]["stage_evidence"]
    global_stage = next(stage for stage in stages if stage["name"] == "global_planning")
    global_stage["inputs"] = ["/nav/odometry", "artifact:tomogram", "/nav/goal_pose"]
    global_stage["input_evidence"] = [
        {
            "token": "artifact:tomogram",
            "kind": "artifact",
            "observable": False,
            "live": False,
            "reason": "saved_map_artifact_missing_or_invalid",
            "artifact_gate": {
                "ok": False,
                "blockers": ["tomogram required but missing"],
                "ros2_topic_required": False,
            },
        }
    ]
    global_stage["missing_inputs"] = ["artifact:tomogram"]
    global_stage["observable"] = False
    global_stage["status"] = "missing"

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="non_motion")

    assert payload["ok"] is True
    assert "runtime stage evidence missing tokens: global_planning" in payload[
        "advisories"
    ]
    assert payload["checks"]["stage_evidence"]["missing_tokens"] == {
        "global_planning": {"inputs": ["artifact:tomogram"], "outputs": []}
    }


def test_gateway_runtime_acceptance_field_rejects_non_live_required_stage():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(
        live=True,
        real_evidence=_real_runtime_evidence_snapshot(),
    )
    global_stage = next(
        stage
        for stage in snapshots["runtime_dataflow"]["stage_evidence"]
        if stage["name"] == "global_planning"
    )
    global_stage["live"] = False
    global_stage["status"] = "metadata_only"
    global_stage["not_live_inputs"] = ["/nav/odometry"]

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="field")

    assert payload["ok"] is False
    assert "field acceptance requires live dataflow stages: global_planning" in (
        payload["blockers"]
    )
    assert payload["checks"]["stage_evidence"]["not_live_stages"] == [
        "global_planning"
    ]


def test_gateway_runtime_acceptance_field_requires_live_traversable_frontier_preview():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(live=True)
    dataflow = snapshots["runtime_dataflow"]
    dataflow["topics"] = [
        topic
        for topic in dataflow["topics"]
        if topic["topic"] not in {"/nav/traversable_frontiers", "/nav/frontier_candidate"}
    ]
    dataflow["stage_evidence"] = [
        stage
        for stage in dataflow["stage_evidence"]
        if stage["name"] != "traversable_frontier_preview"
    ]

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="field")

    assert (
        "runtime dataflow missing product topics: "
        "/nav/traversable_frontiers, /nav/frontier_candidate"
    ) in payload["blockers"]
    assert "runtime dataflow missing required stages: traversable_frontier_preview" in payload["blockers"]
    assert "field acceptance requires live traversable frontier preview" in payload["blockers"]
    assert payload["checks"]["frontier_preview"]["ok"] is False
    assert payload["checks"]["frontier_preview"]["missing_topics"] == [
        "/nav/traversable_frontiers",
        "/nav/frontier_candidate",
    ]


def test_gateway_runtime_acceptance_rejects_frontier_preview_motion_publish():
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance

    snapshots = _snapshots(live=True)
    topics = {
        topic["topic"]: topic
        for topic in snapshots["runtime_dataflow"]["topics"]
    }
    topics["/nav/frontier_candidate"]["inspection"]["latest_payload"][
        "command_published"
    ] = True

    payload = evaluate_gateway_runtime_acceptance(snapshots, mode="field")

    assert "traversable frontier preview published a command" in payload["blockers"]
    assert payload["checks"]["frontier_preview"]["ok"] is False
    assert payload["checks"]["frontier_preview"]["command_published"] is True


def test_gateway_runtime_acceptance_cli_writes_json(monkeypatch, tmp_path, capsys):
    import cli.main as main_mod
    import core.gateway_runtime_acceptance as acceptance_mod

    out_path = tmp_path / "gateway_acceptance.json"

    def _fake_collect(*, gateway_url: str, timeout_sec: float, mode: str):
        return {
            "schema_version": "lingtu.gateway_runtime_acceptance.v1",
            "ok": True,
            "mode": mode,
            "gateway_url": gateway_url,
            "timeout_sec": timeout_sec,
            "runtime_contract": "real_s100p",
            "ros2_topic_required": False,
            "blockers": [],
            "advisories": [],
            "checks": {"module_first_dataflow": {"ok": True}},
        }

    monkeypatch.setattr(acceptance_mod, "collect_gateway_runtime_acceptance", _fake_collect)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "lingtu.py",
            "gateway-runtime-acceptance",
            "--gateway-url",
            "http://robot.local:5050",
            "--gateway-timeout-sec",
            "4.5",
            "--acceptance-mode",
            "field",
            "--json-out",
            str(out_path),
        ],
    )

    main_mod.main()

    assert capsys.readouterr().out == ""
    payload = json.loads(out_path.read_text(encoding="utf-8"))
    assert payload["ok"] is True
    assert payload["mode"] == "field"
    assert payload["gateway_url"] == "http://robot.local:5050"
    assert payload["timeout_sec"] == 4.5


def test_gateway_runtime_acceptance_cli_exits_nonzero_on_blockers(
    monkeypatch, capsys
):
    import cli.main as main_mod
    import core.gateway_runtime_acceptance as acceptance_mod

    def _fake_collect(*, gateway_url: str, timeout_sec: float, mode: str):
        return {
            "schema_version": "lingtu.gateway_runtime_acceptance.v1",
            "ok": False,
            "mode": mode,
            "runtime_contract": None,
            "ros2_topic_required": False,
            "blockers": ["gateway endpoint unavailable: runtime_dataflow"],
            "advisories": [],
            "checks": {"gateway_contract": {"ok": False}},
        }

    monkeypatch.setattr(acceptance_mod, "collect_gateway_runtime_acceptance", _fake_collect)
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "gateway-runtime-acceptance"],
    )

    with pytest.raises(SystemExit) as excinfo:
        main_mod.main()

    out = capsys.readouterr().out
    assert excinfo.value.code == 2
    assert "Gateway runtime acceptance: FAIL" in out
    assert "gateway endpoint unavailable: runtime_dataflow" in out

"""Read-only field acceptance for the map-free ``teleop`` Product."""

from __future__ import annotations

import ipaddress
from collections.abc import Mapping
from typing import Any

from diagnostics.field import _preflight
from lingtu.run_plan import CURRENT_RUN_SCHEMA, RUN_PLAN_SCHEMA

SCHEMA_VERSION = "lingtu.teleop.preflight.v1"
STAGES = frozenset({"contract", "motion"})

_PRODUCT = "teleop"
_ENV = "real"
_REQUIRED_ROLES = frozenset({"nav", "driver", "host"})
_FORBIDDEN_ROLES = frozenset({"lidar", "slam", "maps", "traversability", "camera", "explore"})
_REQUIRED_TOPICS = frozenset(
    {
        "/nav/command/request",
        "/nav/command/ack",
        "/nav/state",
        "/nav/operator_motion/control",
        "/nav/operator_motion/sample",
        "/nav/operator_motion/ack",
        "/nav/operator_motion/status",
        "/nav/cmd_vel",
    }
)
_REQUIRED_CAPABILITIES = frozenset(
    {
        "teleop",
        "operator_motion_typed_dds_interface",
        "native_operator_motion_authority",
        "command_staleness_gate",
        "velocity_limit",
        "final_cmd_vel_single_writer",
    }
)
_FORBIDDEN_TOPIC_PREFIXES = ("/slam/", "/maps/")
_FORBIDDEN_TOPICS = frozenset({"/nav/traversability", "/nav/local_path"})
_TWIST_ALIASES = (("vx",), ("vy",), ("wz",))


def _remote_host(host: str) -> bool:
    if not host:
        return False
    try:
        return not ipaddress.ip_address(host).is_loopback
    except ValueError:
        return host.lower() not in {"localhost", "ip6-localhost"}


def evaluate_teleop_preflight(
    snapshot: Mapping[str, Any],
    *,
    stage: str = "contract",
    status_max_age_s: float = _preflight.STATUS_MAX_AGE_S,
) -> dict[str, Any]:
    """Evaluate Product and native driver readiness without publishing a command."""

    normalized_stage = _preflight.text(stage).lower()
    if normalized_stage not in STAGES:
        raise ValueError(f"unsupported teleop preflight stage: {stage!r}")
    max_age_s = _preflight.number(status_max_age_s)
    if max_age_s is None or max_age_s <= 0.0:
        raise ValueError("status_max_age_s must be a positive finite number")

    evaluation = _preflight.Evaluation(normalized_stage, SCHEMA_VERSION)
    current_run = _preflight.mapping(snapshot.get("current_run"))
    current = _preflight.mapping(current_run.get("state"))
    plan_payload = _preflight.mapping(current_run.get("run_plan"))
    identity = _preflight.mapping(plan_payload.get("identity"))
    contract = _preflight.mapping(current_run.get("verified_contract"))
    native_environment = _preflight.native_environment(current_run)

    evaluation.check(
        "product.current_run",
        current_run.get("exists") is True
        and current.get("schema_version") == CURRENT_RUN_SCHEMA
        and current_run.get("plan_verified") is True
        and identity.get("schema") == RUN_PLAN_SCHEMA
        and bool(contract),
        expected={"current": CURRENT_RUN_SCHEMA, "run_plan": RUN_PLAN_SCHEMA},
        observed={
            "exists": current_run.get("exists"),
            "current_schema": current.get("schema_version"),
            "verified": current_run.get("plan_verified"),
            "run_plan_schema": identity.get("schema"),
            "contract_projection": bool(contract),
        },
    )
    evaluation.check(
        "product.identity",
        all(item.get("product") == _PRODUCT and item.get("env") == _ENV for item in (current, identity, contract)),
        expected={"product": _PRODUCT, "env": _ENV},
        observed={
            "current": {"product": current.get("product"), "env": current.get("env")},
            "run_plan": {"product": identity.get("product"), "env": identity.get("env")},
            "verified_contract": {
                "product": contract.get("product"),
                "env": contract.get("env"),
            },
        },
    )
    topics = _preflight.strings(contract.get("required_topics"))
    capabilities = _preflight.strings(contract.get("required_capabilities"))
    roles = _preflight.strings(contract.get("selected_roles"))
    forbidden_topics = sorted(
        topic for topic in topics if topic in _FORBIDDEN_TOPICS or topic.startswith(_FORBIDDEN_TOPIC_PREFIXES)
    )
    evaluation.check(
        "product.topics",
        _REQUIRED_TOPICS <= topics and not forbidden_topics,
        expected={"required": sorted(_REQUIRED_TOPICS), "forbidden": []},
        observed={
            "missing": sorted(_REQUIRED_TOPICS - topics),
            "forbidden": forbidden_topics,
        },
    )
    evaluation.check(
        "product.capabilities",
        _REQUIRED_CAPABILITIES <= capabilities,
        expected=sorted(_REQUIRED_CAPABILITIES),
        observed={"missing": sorted(_REQUIRED_CAPABILITIES - capabilities)},
    )
    evaluation.check(
        "product.roles",
        roles == _REQUIRED_ROLES and not (_FORBIDDEN_ROLES & roles),
        expected=sorted(_REQUIRED_ROLES),
        observed=sorted(roles),
    )

    native_nav = _preflight.mapping(contract.get("native_nav"))
    expected_nav = {
        "control_mode": "teleop",
        "publish_cmd_vel": True,
        "check_obstacle": False,
        "use_traversability_cost": False,
        "allow_teleop_takeover": False,
        "teleop_local_planner": False,
    }
    evaluation.check(
        "product.native_nav",
        all(native_nav.get(key) == value for key, value in expected_nav.items()),
        expected=expected_nav,
        observed={key: native_nav.get(key) for key in expected_nav},
    )

    nav_entry, nav = _preflight.status_entry(snapshot, "nav")
    driver_entry, driver = _preflight.status_entry(snapshot, "driver")
    for name, entry, payload, schema in (
        ("nav", nav_entry, nav, _preflight.NAV_STATUS_SCHEMA),
        ("driver", driver_entry, driver, _preflight.DRIVER_STATUS_SCHEMA),
    ):
        evaluation.check(
            f"status.{name}",
            _preflight.fresh(entry, max_age_s) and payload.get("schema_version") == schema,
            expected={"schema": schema, "max_age_s": max_age_s},
            observed={
                "exists": entry.get("exists"),
                "age_s": entry.get("age_s"),
                "schema": payload.get("schema_version"),
            },
        )

    nav_product = _preflight.mapping(nav.get("native_product"))
    evaluation.check(
        "nav.identity",
        nav_product.get("product") == _PRODUCT,
        expected={"product": _PRODUCT},
        observed={"product": nav_product.get("product")},
    )
    evaluation.check(
        "nav.runtime_policy",
        nav.get("control_mode") == "teleop"
        and nav.get("publish_cmd_vel") is True
        and nav.get("check_obstacle") is False
        and nav.get("use_traversability_cost") is False
        and nav.get("allow_teleop_takeover") is False
        and nav.get("teleop_local_planner") is False
        and nav.get("stop_confirmation_evidence") == "driver_ack",
        expected={**expected_nav, "stop_confirmation_evidence": "driver_ack"},
        observed={
            **{key: nav.get(key) for key in expected_nav},
            "stop_confirmation_evidence": nav.get("stop_confirmation_evidence"),
        },
    )
    operator_motion = _preflight.mapping(nav.get("operator_motion"))
    operator_status = _preflight.mapping(operator_motion.get("status"))
    evaluation.check(
        "nav.operator_motion",
        operator_motion.get("interface_enabled") is True
        and operator_motion.get("authority_owner") == "native_endpoint"
        and operator_motion.get("control_mode") == "teleop"
        and operator_motion.get("control_ack_scope") == "claim_hold_release"
        and operator_motion.get("sample_evidence") == "status_sequences"
        and operator_motion.get("ack_publish_failed") == 0
        and operator_motion.get("status_publish_failed") == 0,
        expected="typed native operator-motion authority with healthy transport",
        observed={
            "interface_enabled": operator_motion.get("interface_enabled"),
            "authority_owner": operator_motion.get("authority_owner"),
            "control_mode": operator_motion.get("control_mode"),
            "control_ack_scope": operator_motion.get("control_ack_scope"),
            "sample_evidence": operator_motion.get("sample_evidence"),
            "ack_publish_failed": operator_motion.get("ack_publish_failed"),
            "status_publish_failed": operator_motion.get("status_publish_failed"),
        },
    )
    evaluation.check(
        "nav.idle_zero",
        operator_status.get("has_active_authority") is False
        and operator_status.get("has_active_sample") is False
        and _preflight.zero_twist(operator_status.get("final_cmd_vel"), _TWIST_ALIASES)
        and _preflight.zero_twist(nav.get("final_cmd_vel"), _TWIST_ALIASES),
        expected="no authority and exact zero output",
        observed={
            "has_active_authority": operator_status.get("has_active_authority"),
            "has_active_sample": operator_status.get("has_active_sample"),
            "operator_final": operator_status.get("final_cmd_vel"),
            "nav_final": nav.get("final_cmd_vel"),
        },
    )
    input_gate = _preflight.mapping(nav.get("input_gate"))
    evaluation.check(
        "nav.map_free_gate",
        input_gate.get("require_driver_control") is True
        and input_gate.get("require_odom") is False
        and input_gate.get("require_cloud") is False
        and input_gate.get("require_traversability") is False
        and input_gate.get("require_localization_health") is False,
        expected={
            "driver": True,
            "odom": False,
            "cloud": False,
            "traversability": False,
            "localization": False,
        },
        observed={
            "driver": input_gate.get("require_driver_control"),
            "odom": input_gate.get("require_odom"),
            "cloud": input_gate.get("require_cloud"),
            "traversability": input_gate.get("require_traversability"),
            "localization": input_gate.get("require_localization_health"),
        },
    )

    driver_dds = _preflight.mapping(driver.get("dds"))
    driver_backend = _preflight.text(driver.get("backend")).lower()
    evaluation.check(
        "driver.boundary",
        driver.get("role") == "driver"
        and driver_backend in _preflight.SUPPORTED_DRIVER_BACKENDS
        and driver_dds.get("topic") == "/nav/cmd_vel"
        and driver_dds.get("wire_topic") == "rt/nav/cmd_vel",
        expected="selected native motion backend owns /nav/cmd_vel",
        observed={
            "role": driver.get("role"),
            "backend": driver.get("backend"),
            "topic": driver_dds.get("topic"),
            "wire_topic": driver_dds.get("wire_topic"),
        },
    )

    if normalized_stage == "motion":
        adapter = _preflight.mapping(driver.get("adapter"))
        control = _preflight.mapping(driver.get("control"))
        target = _preflight.text(adapter.get("target"))
        host = _preflight.target_host(target)
        expected_target = _preflight.text(native_environment.get("LINGTU_DRIVER_TARGET"))
        expected_interface = _preflight.text(native_environment.get("LINGTU_DRIVER_NETWORK_INTERFACE"))
        evaluation.check(
            "motion.driver_ready",
            _preflight.mapping(snapshot.get("driver_readiness")).get("ok") is True
            and driver.get("connected") is True
            and driver.get("ready") is True
            and driver_dds.get("cmd_vel_writer_ready") is True
            and driver_dds.get("matched_cmd_vel_writers") == 1
            and input_gate.get("ready") is True
            and input_gate.get("driver_control_ready") is True,
            expected="one nav writer and fresh ready driver control",
            observed={
                "collector_ready": _preflight.mapping(snapshot.get("driver_readiness")).get("ok"),
                "connected": driver.get("connected"),
                "ready": driver.get("ready"),
                "writer_ready": driver_dds.get("cmd_vel_writer_ready"),
                "matched_writers": driver_dds.get("matched_cmd_vel_writers"),
                "input_gate_ready": input_gate.get("ready"),
                "driver_control_ready": input_gate.get("driver_control_ready"),
            },
        )
        shared_control_ready = (
            control.get("initial_zero_acknowledged") is True
            and control.get("motors_enabled") is True
            and control.get("critical_fault") is False
            and control.get("control_assured") is True
            and _preflight.text(control.get("fsm")).lower() in {"standing", "walking"}
        )
        if driver_backend == "doso":
            control_check_id = "motion.brainstem_control"
            backend_control_ready = (
                adapter.get("protocol") == "brainstem_grpc"
                and adapter.get("control_owner") == "grpc"
                and adapter.get("control_owner_id") == _preflight.DRIVER_MOTION_PRINCIPAL
                and control.get("lease_valid") is True
                and _remote_host(host)
                and bool(expected_target)
                and target == expected_target
            )
            expected_control = (
                "remote driver lease owned by "
                f"{_preflight.DRIVER_MOTION_PRINCIPAL} with checked initial zero"
            )
        elif driver_backend == "go2":
            control_check_id = "motion.go2_control"
            backend_control_ready = (
                bool(expected_interface)
                and _preflight.sdk2_interface(target) == expected_interface
                and adapter.get("protocol") == "unitree_sdk2"
                and adapter.get("control_owner") == "none"
                and not _preflight.text(adapter.get("control_owner_id"))
                and control.get("lease_valid") is False
            )
            expected_control = (
                "Unitree SDK2 control on one configured network interface with "
                "checked initial zero and no fabricated lease"
            )
        else:
            control_check_id = "motion.driver_control"
            backend_control_ready = False
            expected_control = "supported motion backend control evidence"
        evaluation.check(
            control_check_id,
            shared_control_ready and backend_control_ready,
            expected=expected_control,
            observed={
                "backend": driver_backend,
                "protocol": adapter.get("protocol"),
                "target": target,
                "expected_target": expected_target or None,
                "expected_interface": expected_interface or None,
                "initial_zero_acknowledged": control.get("initial_zero_acknowledged"),
                "motors_enabled": control.get("motors_enabled"),
                "critical_fault": control.get("critical_fault"),
                "control_assured": control.get("control_assured"),
                "lease_valid": control.get("lease_valid"),
                "owner": adapter.get("control_owner"),
                "owner_id": adapter.get("control_owner_id"),
                "fsm": control.get("fsm"),
            },
        )

        final_output = _preflight.mapping(nav.get("final_output"))
        driver_control = _preflight.mapping(nav.get("driver_control"))
        driver_ack = _preflight.mapping(driver.get("output_ack"))
        producer_boot_id = _preflight.text(final_output.get("producer_boot_id"))
        output_sequence = final_output.get("output_sequence")
        exact_idle_zero_ack = (
            final_output.get("published") is True
            and bool(producer_boot_id)
            and _preflight.positive_int(output_sequence)
            and final_output.get("driver_delivery_accepted") is True
            and driver_control.get("received") is True
            and driver_control.get("ready") is True
            and driver_control.get("last_command_accepted") is True
            and driver_control.get("accepted_producer_boot_id") == producer_boot_id
            and driver_control.get("accepted_output_sequence") == output_sequence
            and driver_ack.get("accepted") is True
            and driver_ack.get("producer_boot_id") == producer_boot_id
            and driver_ack.get("output_sequence") == output_sequence
        )
        evaluation.check(
            "motion.exact_idle_zero_ack",
            exact_idle_zero_ack,
            expected=(
                "latest idle zero has one exact producer_boot_id/output_sequence "
                "across nav publication, driver control, and backend-confirmed driver ACK"
            ),
            observed={
                "nav": dict(final_output),
                "nav_driver_control": dict(driver_control),
                "driver": dict(driver_ack),
            },
        )

    return evaluation.result()


__all__ = ["SCHEMA_VERSION", "STAGES", "evaluate_teleop_preflight"]

"""Pure evaluation of the field ``teleop_avoid`` motion boundary.

The collector that calls this module owns filesystem and process inspection.
This module only evaluates an already collected snapshot. It never compiles a
Product, publishes DDS data, acquires operator authority, or changes runtime
state.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from diagnostics.field import _preflight
from lingtu.run_plan import CURRENT_RUN_SCHEMA, RUN_PLAN_SCHEMA

SCHEMA_VERSION = "lingtu.teleop_avoid.preflight.v1"
STAGES = frozenset({"contract", "motion"})

_PRODUCT = "teleop_avoid"
_ENV = "real"
_TRAVERSABILITY_STATUS_SCHEMA = "lingtu.traversability.status.v2"
_MAPS_STATUS_SCHEMA = "lingtu.maps.runtime.v1"
_DRIVER_ACK_MAX_SEQUENCE_LAG = 2
_TRAVERSABILITY_STATUS_MAX_AGE_S = 6.0
_TWIST_ALIASES = (
    ("vx_mps", "vx", "x"),
    ("vy_mps", "vy", "y"),
    ("yaw_rps", "wz", "z"),
)
_REQUIRED_CAPABILITIES = frozenset(
    {
        "operator_motion_typed_dds_interface",
        "native_operator_motion_authority",
        "registered_cloud_collision_check",
        "traversability_costmap",
        "local_planner_collision_and_traversability_scoring",
        "path_follower_pre_command_output",
        "operator_assisted_local_planner_control",
        "final_cmd_vel_single_writer",
    }
)
_REQUIRED_TOPICS = frozenset(
    {
        "/nav/operator_motion/control",
        "/nav/operator_motion/sample",
        "/nav/operator_motion/ack",
        "/nav/operator_motion/status",
        "/slam/odometry",
        "/slam/registered_cloud",
        "/slam/map_observation",
        "/maps/state",
        "/maps/scene",
        "/nav/traversability",
        "/nav/local_path",
        "/nav/cmd_vel",
    }
)
_REQUIRED_ROLES = frozenset({"lidar", "slam", "maps", "traversability", "nav", "driver", "host"})


def _zero_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value == 0


def _non_negative_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _twist_equal(left: Any, right: Any) -> bool:
    left_twist = _preflight.mapping(left)
    right_twist = _preflight.mapping(right)
    for aliases in (("vx_mps", "vx", "x"), ("vy_mps", "vy", "y"), ("yaw_rps", "wz", "z")):
        left_value = _preflight.number(next((left_twist[key] for key in aliases if key in left_twist), None))
        right_value = _preflight.number(next((right_twist[key] for key in aliases if key in right_twist), None))
        if left_value is None or right_value is None:
            return False
        if abs(left_value - right_value) > _preflight.ZERO_EPSILON:
            return False
    return True


def _linear_motion(value: Any) -> bool:
    twist = _preflight.mapping(value)
    vx = _preflight.number(twist.get("vx", twist.get("x")))
    vy = _preflight.number(twist.get("vy", twist.get("y")))
    return bool(
        vx is not None
        and vy is not None
        and (abs(vx) > _preflight.ZERO_EPSILON or abs(vy) > _preflight.ZERO_EPSILON)
    )


def _observed(mapping: Mapping[str, Any], keys: tuple[str, ...]) -> dict[str, Any]:
    return {key: mapping.get(key) for key in keys}


_LOOPBACK_OR_UNSPECIFIED_HOSTS = frozenset(
    {"", "localhost", "127.0.0.1", "::1", "0.0.0.0", "::"}  # noqa: S104
)


def _evaluate_maps_status(
    evaluation: _preflight.Evaluation,
    maps: Mapping[str, Any],
) -> None:
    runtime_keys = ("process", "status", "ready", "running", "live")
    evaluation.check(
        "maps.runtime",
        maps.get("process") == "mapd"
        and maps.get("status") == "ready"
        and maps.get("ready") is True
        and maps.get("running") is True
        and maps.get("live") is True,
        expected={
            "process": "mapd",
            "status": "ready",
            "ready": True,
            "running": True,
            "live": True,
        },
        observed=_observed(maps, runtime_keys),
    )

    observation_keys = (
        "generation",
        "accepted_observations",
        "processed_observations",
        "dds_received",
        "dds_decoded",
    )
    accepted = maps.get("accepted_observations")
    processed = maps.get("processed_observations")
    dds_received = maps.get("dds_received")
    dds_decoded = maps.get("dds_decoded")
    evaluation.check(
        "maps.observation_pipeline",
        _preflight.positive_int(maps.get("generation"))
        and _preflight.positive_int(accepted)
        and _preflight.positive_int(processed)
        and processed <= accepted
        and _preflight.positive_int(dds_received)
        and _preflight.positive_int(dds_decoded)
        and dds_decoded <= dds_received,
        expected=(
            "positive generation, accepted/processed observations, and DDS "
            "received/decoded samples with monotonic counter relationships"
        ),
        observed=_observed(maps, observation_keys),
    )

    dds_health_keys = (
        "dds_write_attempts",
        "dds_rejected",
        "dds_write_failures",
        "dds_serialization_rejections",
        "dds_scene_oversize_rejections",
        "dds_unhealthy_writers",
        "voxel_snapshot_omitted_cells",
    )
    evaluation.check(
        "maps.dds_health",
        _preflight.positive_int(maps.get("dds_write_attempts"))
        and _zero_int(maps.get("dds_rejected"))
        and _zero_int(maps.get("dds_write_failures"))
        and _zero_int(maps.get("dds_serialization_rejections"))
        and _zero_int(maps.get("dds_scene_oversize_rejections"))
        and _zero_int(maps.get("dds_unhealthy_writers")),
        expected={
            "dds_write_attempts": "> 0",
            "dds_rejected": 0,
            "dds_write_failures": 0,
            "dds_serialization_rejections": 0,
            "dds_scene_oversize_rejections": 0,
            "dds_unhealthy_writers": 0,
        },
        observed=_observed(maps, dds_health_keys),
    )
    evaluation.check(
        "maps.resource_accounting",
        _non_negative_int(maps.get("voxel_snapshot_omitted_cells")),
        expected="non-negative bounded voxel snapshot omission count",
        observed={"voxel_snapshot_omitted_cells": maps.get("voxel_snapshot_omitted_cells")},
    )
    resource_capacity_keys = (
        "capacity_limited",
        "voxel_capacity_rejections",
        "accumulated_capacity_rejections",
    )
    evaluation.check(
        "maps.resource_capacity",
        maps.get("capacity_limited") is False
        and _zero_int(maps.get("voxel_capacity_rejections"))
        and _zero_int(maps.get("accumulated_capacity_rejections")),
        expected={
            "capacity_limited": False,
            "voxel_capacity_rejections": 0,
            "accumulated_capacity_rejections": 0,
        },
        observed=_observed(maps, resource_capacity_keys),
    )

    generation = maps.get("generation")
    publication_generation_keys = (
        "state_published_generation",
        "realtime_clouds_published_generation",
        "map_layers_published_generation",
        "scene_published_generation",
    )
    publication_generations = _observed(maps, publication_generation_keys)
    evaluation.check(
        "maps.required_publications",
        maps.get("required_publications_ready") is True
        and maps.get("current_generation_published") is True
        and _preflight.positive_int(generation)
        and all(_preflight.positive_int(value) and value == generation for value in publication_generations.values()),
        expected=(
            "all required mapd channels published at least once and each "
            "publication cursor equals the current generation"
        ),
        observed={
            "generation": generation,
            "required_publications_ready": maps.get("required_publications_ready"),
            "current_generation_published": maps.get("current_generation_published"),
            **publication_generations,
        },
    )


def evaluate_teleop_avoid_preflight(
    snapshot: Mapping[str, Any],
    *,
    stage: str = "contract",
    status_max_age_s: float = _preflight.STATUS_MAX_AGE_S,
) -> dict[str, Any]:
    """Evaluate collected field evidence without touching runtime state.

    ``contract`` confirms the committed Product and native process contracts.
    ``motion`` additionally requires live input/control readiness and an exact
    Nav Endpoint -> Driver output acknowledgement. An active operator sample
    is not required in either stage.
    """

    normalized_stage = _preflight.text(stage).lower()
    if normalized_stage not in STAGES:
        raise ValueError(f"unsupported teleop_avoid preflight stage: {stage!r}")
    max_age_s = _preflight.number(status_max_age_s)
    if max_age_s is None or max_age_s <= 0.0:
        raise ValueError("status_max_age_s must be a positive finite number")

    evaluation = _preflight.Evaluation(normalized_stage, SCHEMA_VERSION)
    current_run = _preflight.mapping(snapshot.get("current_run"))
    state = _preflight.mapping(current_run.get("state"))
    run_plan_payload = _preflight.mapping(current_run.get("run_plan"))
    plan_identity = _preflight.mapping(run_plan_payload.get("identity"))
    run_plan = _preflight.mapping(current_run.get("verified_contract"))
    native_environment = _preflight.native_environment(current_run)

    evaluation.check(
        "product.current_run_present",
        current_run.get("exists") is True,
        expected=True,
        observed=current_run.get("exists"),
    )
    evaluation.check(
        "product.current_run_schema",
        state.get("schema_version") == CURRENT_RUN_SCHEMA,
        expected=CURRENT_RUN_SCHEMA,
        observed=state.get("schema_version"),
    )
    evaluation.check(
        "product.run_plan_verified",
        current_run.get("plan_verified") is True and bool(run_plan) and plan_identity.get("schema") == RUN_PLAN_SCHEMA,
        expected={"verified": True, "schema": RUN_PLAN_SCHEMA},
        observed={
            "verified": current_run.get("plan_verified"),
            "schema": plan_identity.get("schema"),
            "contract_projection": bool(run_plan),
        },
        detail=_preflight.text(current_run.get("error")),
    )
    evaluation.check(
        "product.identity",
        state.get("product") == _PRODUCT
        and plan_identity.get("product") == _PRODUCT
        and run_plan.get("product") == _PRODUCT,
        expected=_PRODUCT,
        observed={
            "current": state.get("product"),
            "run_plan": plan_identity.get("product"),
            "verified_contract": run_plan.get("product"),
        },
    )
    evaluation.check(
        "product.env",
        state.get("env") == _ENV and plan_identity.get("env") == _ENV and run_plan.get("env") == _ENV,
        expected=_ENV,
        observed={
            "current": state.get("env"),
            "run_plan": plan_identity.get("env"),
            "verified_contract": run_plan.get("env"),
        },
    )
    capabilities = _preflight.strings(run_plan.get("required_capabilities"))
    topics = _preflight.strings(run_plan.get("required_topics"))
    missing_capabilities = sorted(_REQUIRED_CAPABILITIES - capabilities)
    missing_topics = sorted(_REQUIRED_TOPICS - topics)
    selected_roles = _preflight.strings(run_plan.get("selected_roles"))
    missing_roles = sorted(_REQUIRED_ROLES - selected_roles)
    evaluation.check(
        "product.required_capabilities",
        not missing_capabilities,
        expected=sorted(_REQUIRED_CAPABILITIES),
        observed={"missing": missing_capabilities},
    )
    evaluation.check(
        "product.required_topics",
        not missing_topics,
        expected=sorted(_REQUIRED_TOPICS),
        observed={"missing": missing_topics},
    )
    evaluation.check(
        "product.selected_roles",
        not missing_roles,
        expected=sorted(_REQUIRED_ROLES),
        observed={"missing": missing_roles, "selected": sorted(selected_roles)},
    )

    native_nav = _preflight.mapping(run_plan.get("native_nav"))
    expected_native_nav = {
        "control_mode": _PRODUCT,
        "publish_cmd_vel": True,
        "check_obstacle": True,
        "use_traversability_cost": True,
        "allow_teleop_takeover": False,
        "teleop_local_planner": True,
    }
    evaluation.check(
        "product.native_nav",
        all(native_nav.get(key) == value for key, value in expected_native_nav.items()),
        expected=expected_native_nav,
        observed={key: native_nav.get(key) for key in expected_native_nav},
    )

    nav_entry, nav = _preflight.status_entry(snapshot, "nav")
    traversability_entry, traversability = _preflight.status_entry(snapshot, "traversability")
    maps_entry, maps = _preflight.status_entry(snapshot, "maps")
    driver_entry, driver = _preflight.status_entry(snapshot, "driver")
    for name, entry, payload, schema in (
        ("nav", nav_entry, nav, _preflight.NAV_STATUS_SCHEMA),
        ("traversability", traversability_entry, traversability, _TRAVERSABILITY_STATUS_SCHEMA),
        ("maps", maps_entry, maps, _MAPS_STATUS_SCHEMA),
        ("driver", driver_entry, driver, _preflight.DRIVER_STATUS_SCHEMA),
    ):
        entry_max_age_s = (
            max(max_age_s, _TRAVERSABILITY_STATUS_MAX_AGE_S)
            if name == "traversability"
            else max_age_s
        )
        evaluation.check(
            f"status.{name}.fresh",
            _preflight.fresh(entry, entry_max_age_s),
            expected=f"exists and age_s <= {entry_max_age_s:g}",
            observed={"exists": entry.get("exists"), "age_s": entry.get("age_s")},
        )
        evaluation.check(
            f"status.{name}.schema",
            payload.get("schema_version") == schema,
            expected=schema,
            observed=payload.get("schema_version"),
        )

    _evaluate_maps_status(evaluation, maps)

    nav_runtime = _preflight.mapping(nav.get("native_product"))
    evaluation.check(
        "nav.product_runtime_identity",
        nav_runtime.get("product") == _PRODUCT,
        expected={"product": _PRODUCT},
        observed={"product": nav_runtime.get("product")},
    )
    runtime_policy_keys = (
        "control_mode",
        "publish_cmd_vel",
        "check_obstacle",
        "use_traversability_cost",
        "allow_teleop_takeover",
        "teleop_local_planner",
    )
    evaluation.check(
        "nav.runtime_policy",
        nav.get("control_mode") == _PRODUCT
        and nav.get("publish_cmd_vel") is True
        and nav.get("check_obstacle") is True
        and nav.get("use_traversability_cost") is True
        and nav.get("allow_teleop_takeover") is False
        and nav.get("teleop_local_planner") is True,
        expected=expected_native_nav,
        observed=_observed(nav, runtime_policy_keys),
    )

    operator_motion = _preflight.mapping(nav.get("operator_motion"))
    operator_contract_keys = (
        "interface_enabled",
        "authority_owner",
        "control_mode",
        "control_ack_scope",
        "sample_evidence",
    )
    evaluation.check(
        "nav.operator_motion_contract",
        operator_motion.get("interface_enabled") is True
        and operator_motion.get("authority_owner") == "native_endpoint"
        and operator_motion.get("control_mode") == _PRODUCT
        and operator_motion.get("control_ack_scope") == "claim_hold_release"
        and operator_motion.get("sample_evidence") == "status_sequences",
        expected={
            "interface_enabled": True,
            "authority_owner": "native_endpoint",
            "control_mode": _PRODUCT,
            "control_ack_scope": "claim_hold_release",
            "sample_evidence": "status_sequences",
        },
        observed=_observed(operator_motion, operator_contract_keys),
    )
    evaluation.check(
        "nav.operator_motion_transport",
        operator_motion.get("ack_publish_failed") == 0 and operator_motion.get("status_publish_failed") == 0,
        expected={"ack_publish_failed": 0, "status_publish_failed": 0},
        observed={
            "ack_publish_failed": operator_motion.get("ack_publish_failed"),
            "status_publish_failed": operator_motion.get("status_publish_failed"),
        },
    )
    evaluation.check(
        "traversability.endpoint",
        traversability.get("endpoint") == "lingtu_traversability_dds",
        expected="lingtu_traversability_dds",
        observed=traversability.get("endpoint"),
    )
    driver_dds = _preflight.mapping(driver.get("dds"))
    driver_backend = _preflight.text(driver.get("backend")).lower()
    evaluation.check(
        "driver.boundary",
        driver.get("role") == "driver"
        and driver_backend in _preflight.SUPPORTED_DRIVER_BACKENDS
        and driver_dds.get("topic") == "/nav/cmd_vel"
        and driver_dds.get("wire_topic") == "rt/nav/cmd_vel",
        expected={
            "role": "driver",
            "backend": sorted(_preflight.SUPPORTED_DRIVER_BACKENDS),
            "topic": "/nav/cmd_vel",
            "wire_topic": "rt/nav/cmd_vel",
        },
        observed={
            "role": driver.get("role"),
            "backend": driver.get("backend"),
            "topic": driver_dds.get("topic"),
            "wire_topic": driver_dds.get("wire_topic"),
        },
    )

    if normalized_stage == "motion":
        _evaluate_motion_stage(
            evaluation,
            snapshot=snapshot,
            nav=nav,
            traversability=traversability,
            driver=driver,
            native_environment=native_environment,
        )

    return evaluation.result()


def _evaluate_motion_stage(
    evaluation: _preflight.Evaluation,
    *,
    snapshot: Mapping[str, Any],
    nav: Mapping[str, Any],
    traversability: Mapping[str, Any],
    driver: Mapping[str, Any],
    native_environment: Mapping[str, Any],
) -> None:
    nav_counters = _preflight.mapping(nav.get("counters"))
    traversability_counters = _preflight.mapping(traversability.get("counters"))
    evaluation.check(
        "motion.sensor_chain",
        nav.get("has_odom") is True
        and nav.get("has_traversability") is True
        and _preflight.positive_int(nav_counters.get("odom"))
        and _preflight.positive_int(nav_counters.get("registered_clouds"))
        and _preflight.positive_int(nav_counters.get("traversability"))
        and traversability.get("has_odom") is True
        and _preflight.positive_int(traversability_counters.get("odom"))
        and _preflight.positive_int(traversability_counters.get("registered_clouds"))
        and _preflight.positive_int(traversability_counters.get("published")),
        expected="positive odom/cloud/traversability evidence at both native endpoints",
        observed={
            "nav": {
                "has_odom": nav.get("has_odom"),
                "has_traversability": nav.get("has_traversability"),
                "odom": nav_counters.get("odom"),
                "registered_clouds": nav_counters.get("registered_clouds"),
                "traversability": nav_counters.get("traversability"),
            },
            "traversability": {
                "has_odom": traversability.get("has_odom"),
                "odom": traversability_counters.get("odom"),
                "registered_clouds": traversability_counters.get("registered_clouds"),
                "published": traversability_counters.get("published"),
            },
        },
    )

    input_gate = _preflight.mapping(nav.get("input_gate"))
    input_gate_keys = (
        "ready",
        "reason",
        "require_odom",
        "require_cloud",
        "require_traversability",
        "require_driver_control",
        "require_localization_health",
        "driver_control_ready",
        "driver_control_reason",
        "localization_healthy",
    )
    evaluation.check(
        "motion.input_gate",
        input_gate.get("ready") is True
        and input_gate.get("reason") == "ready"
        and input_gate.get("require_odom") is True
        and input_gate.get("require_cloud") is True
        and input_gate.get("require_traversability") is True
        and input_gate.get("require_driver_control") is True
        and input_gate.get("require_localization_health") is True
        and input_gate.get("localization_healthy") is True
        and input_gate.get("driver_control_ready") is True,
        expected={
            "ready": True,
            "reason": "ready",
            "require_odom": True,
            "require_cloud": True,
            "require_traversability": True,
            "require_driver_control": True,
            "require_localization_health": True,
            "localization_healthy": True,
            "driver_control_ready": True,
        },
        observed=_observed(input_gate, input_gate_keys),
    )
    control_loop = _preflight.mapping(nav.get("control_loop_health"))
    evaluation.check(
        "motion.control_loop",
        control_loop.get("ready") is True and control_loop.get("healthy") is True,
        expected={"ready": True, "healthy": True},
        observed={
            "ready": control_loop.get("ready"),
            "healthy": control_loop.get("healthy"),
            "reason": control_loop.get("reason"),
        },
    )

    driver_readiness = _preflight.mapping(snapshot.get("driver_readiness"))
    evaluation.check(
        "motion.driver_readiness",
        driver_readiness.get("ok") is True,
        expected=True,
        observed={
            "ok": driver_readiness.get("ok"),
            "blockers": driver_readiness.get("blockers"),
        },
    )
    driver_backend = _preflight.text(driver.get("backend")).lower()
    adapter = _preflight.mapping(driver.get("adapter"))
    control = _preflight.mapping(driver.get("control"))
    target = _preflight.text(adapter.get("target"))
    target_host = _preflight.target_host(target)
    expected_target = _preflight.text(native_environment.get("LINGTU_DRIVER_TARGET"))
    expected_interface = _preflight.text(native_environment.get("LINGTU_DRIVER_NETWORK_INTERFACE"))
    shared_control_ready = (
        driver.get("connected") is True
        and driver.get("ready") is True
        and _preflight.mapping(driver.get("dds")).get("cmd_vel_writer_ready") is True
        and _preflight.mapping(driver.get("dds")).get("matched_cmd_vel_writers") == 1
        and control.get("initial_zero_acknowledged") is True
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
            and target_host not in _LOOPBACK_OR_UNSPECIFIED_HOSTS
            and bool(expected_target)
            and target == expected_target
        )
        expected_control = "connected ready remote Brainstem with motors, LingTu lease, and standing/walking FSM"
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
            "connected ready Unitree SDK2 control on one configured network "
            "interface with standing/walking FSM and no fabricated lease"
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
            "connected": driver.get("connected"),
            "protocol": adapter.get("protocol"),
            "target": target,
            "expected_target": expected_target or None,
            "expected_interface": expected_interface or None,
            "ready": driver.get("ready"),
            "cmd_vel_writer_ready": _preflight.mapping(driver.get("dds")).get("cmd_vel_writer_ready"),
            "matched_cmd_vel_writers": _preflight.mapping(driver.get("dds")).get("matched_cmd_vel_writers"),
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
    driver_control_sequence = driver_control.get("accepted_output_sequence")
    driver_ack_sequence = driver_ack.get("output_sequence")
    correlated_ack = (
        final_output.get("published") is True
        and bool(producer_boot_id)
        and _preflight.positive_int(output_sequence)
        and driver_control.get("received") is True
        and driver_control.get("ready") is True
        and driver_control.get("last_command_accepted") is True
        and driver_control.get("accepted_producer_boot_id") == producer_boot_id
        and _preflight.positive_int(driver_control_sequence)
        and 0 <= output_sequence - driver_control_sequence <= _DRIVER_ACK_MAX_SEQUENCE_LAG
        and driver_ack.get("accepted") is True
        and driver_ack.get("producer_boot_id") == producer_boot_id
        and _preflight.positive_int(driver_ack_sequence)
        and driver_ack_sequence <= output_sequence + _DRIVER_ACK_MAX_SEQUENCE_LAG
    )
    evaluation.check(
        "motion.correlated_driver_ack",
        correlated_ack,
        expected=(
            "same producer, nav-embedded ACK no more than two outputs behind, "
            "and a fresh driver status snapshot that accepted this producer without being more than two outputs ahead"
        ),
        observed={
            "nav": dict(final_output),
            "nav_driver_control": dict(driver_control),
            "driver": dict(driver_ack),
        },
    )

    operator_motion = _preflight.mapping(nav.get("operator_motion"))
    last_ack = _preflight.mapping(operator_motion.get("last_ack"))
    operator_status = _preflight.mapping(operator_motion.get("status"))
    evaluation.check(
        "motion.operator_status_mirror",
        operator_status.get("observed") is True
        and operator_status.get("published") is True
        and (last_ack.get("observed") is not True or last_ack.get("published") is True)
        and operator_status.get("input_gate_reason") == input_gate.get("reason")
        and _twist_equal(operator_status.get("final_cmd_vel"), nav.get("final_cmd_vel"))
        and _twist_equal(
            operator_status.get("teleop_output"),
            _preflight.mapping(nav.get("teleop")).get("output"),
        ),
        expected="published operator status mirrors input gate, teleop output, and final output",
        observed={"last_ack": dict(last_ack), "status": dict(operator_status)},
    )
    active_sample = operator_status.get("has_active_sample") is True
    evaluation.check(
        "motion.operator_sample_correlation",
        not active_sample
        or (
            _preflight.positive_int(operator_status.get("admitted_sequence"))
            and operator_status.get("final_output_sequence") == output_sequence
            and _twist_equal(operator_status.get("final_cmd_vel"), nav.get("final_cmd_vel"))
            and _twist_equal(
                operator_status.get("teleop_output"),
                _preflight.mapping(nav.get("teleop")).get("output"),
            )
        ),
        expected="idle, or admitted operator sample correlated to the exact final output",
        observed={
            "has_active_sample": active_sample,
            "admitted_sequence": operator_status.get("admitted_sequence"),
            "operator_final_output_sequence": operator_status.get("final_output_sequence"),
            "nav_final_output_sequence": output_sequence,
        },
    )

    teleop = _preflight.mapping(nav.get("teleop"))
    linear_motion = active_sample and _linear_motion(teleop.get("request"))
    last_local = _preflight.mapping(nav.get("last_local"))
    final_safety = _preflight.mapping(last_local.get("final_safety"))
    evaluation.check(
        "motion.assisted_local_path",
        not linear_motion
        or (
            teleop.get("seen") is True
            and teleop.get("published") is True
            and _preflight.positive_int(nav.get("local_path_points"))
            and last_local.get("seen") is True
            and final_safety.get("applied") is True
        ),
        expected=("idle/rotation-only, or linear operator motion with LocalPlanner path and final safety evidence"),
        observed={
            "active_sample": active_sample,
            "linear_motion": linear_motion,
            "teleop_seen": teleop.get("seen"),
            "teleop_published": teleop.get("published"),
            "teleop_reason": teleop.get("reason"),
            "local_path_points": nav.get("local_path_points"),
            "local_seen": last_local.get("seen"),
            "final_safety": dict(final_safety),
        },
    )

    idle = not active_sample
    evaluation.check(
        "motion.idle_zero",
        not idle
        or (
            _preflight.zero_twist(nav.get("final_cmd_vel"), _TWIST_ALIASES)
            and _preflight.zero_twist(operator_status.get("final_cmd_vel"), _TWIST_ALIASES)
            and _preflight.zero_twist(driver.get("last_velocity"), _TWIST_ALIASES)
        ),
        expected="when no active operator sample, nav/operator/driver outputs are zero",
        observed={
            "idle": idle,
            "nav_final_cmd_vel": nav.get("final_cmd_vel"),
            "operator_final_cmd_vel": operator_status.get("final_cmd_vel"),
            "driver_last_velocity": driver.get("last_velocity"),
        },
    )


__all__ = [
    "SCHEMA_VERSION",
    "STAGES",
    "evaluate_teleop_avoid_preflight",
]

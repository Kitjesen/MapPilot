"""Pure evaluation of the field ``teleop_avoid`` motion boundary.

The collector that calls this module owns filesystem and process inspection.
This module only evaluates an already collected snapshot. It never compiles a
Product, publishes DDS data, acquires operator authority, or changes runtime
state.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any

from lingtu.run_plan import CURRENT_RUN_SCHEMA

SCHEMA_VERSION = "lingtu.teleop_avoid.preflight.v1"
STAGES = frozenset({"contract", "motion"})

_PRODUCT = "teleop_avoid"
_ENV = "real"
_NAV_STATUS_SCHEMA = "lingtu.nav.endpoint.status.v1"
_DRIVER_STATUS_SCHEMA = "lingtu.driver.status.v1"
_TRAVERSABILITY_STATUS_SCHEMA = "lingtu.traversability.status.v2"
_MAPS_STATUS_SCHEMA = "lingtu.maps.runtime.v1"
_STATUS_MAX_AGE_S = 3.0
_ZERO_EPSILON = 1e-6
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


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _text(value: Any) -> str:
    return str(value or "").strip()


def _number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _positive_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _zero_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value == 0


def _non_negative_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _fresh(entry: Mapping[str, Any], max_age_s: float) -> bool:
    age_s = _number(entry.get("age_s"))
    return bool(entry.get("exists") is True and age_s is not None and 0.0 <= age_s <= max_age_s)


def _zero_twist(value: Any) -> bool:
    twist = _mapping(value)
    components = (
        twist.get("vx", twist.get("x")),
        twist.get("vy", twist.get("y")),
        twist.get("wz", twist.get("z")),
    )
    parsed = tuple(_number(component) for component in components)
    return all(component is not None and abs(component) <= _ZERO_EPSILON for component in parsed)


class _Evaluation:
    def __init__(self, stage: str) -> None:
        self.stage = stage
        self.checks: list[dict[str, Any]] = []
        self.blockers: list[str] = []

    def check(
        self,
        check_id: str,
        ok: bool,
        *,
        expected: Any,
        observed: Any,
        required: bool = True,
        detail: str = "",
    ) -> None:
        passed = bool(ok)
        self.checks.append(
            {
                "id": check_id,
                "required": required,
                "ok": passed,
                "expected": expected,
                "observed": observed,
                "detail": detail,
            }
        )
        if required and not passed:
            self.blockers.append(check_id)

    def result(self, *, evidence: Mapping[str, Any]) -> dict[str, Any]:
        nonzero_motion_allowed = self.stage == "motion" and not self.blockers
        nonzero_motion_blockers = (
            []
            if nonzero_motion_allowed
            else (
                list(self.blockers)
                if self.stage == "motion"
                else ["motion_stage_not_evaluated"]
            )
        )
        return {
            "schema_version": SCHEMA_VERSION,
            "stage": self.stage,
            "read_only": True,
            "authority_acquired": False,
            "command_published": False,
            "nonzero_motion_allowed": nonzero_motion_allowed,
            "nonzero_motion_blockers": nonzero_motion_blockers,
            "ok": not self.blockers,
            "blocker_count": len(self.blockers),
            "blockers": list(self.blockers),
            "checks": list(self.checks),
            "evidence": dict(evidence),
        }


def _twist_equal(left: Any, right: Any) -> bool:
    left_twist = _mapping(left)
    right_twist = _mapping(right)
    for aliases in (("vx", "x"), ("vy", "y"), ("wz", "z")):
        left_value = _number(next((left_twist[key] for key in aliases if key in left_twist), None))
        right_value = _number(next((right_twist[key] for key in aliases if key in right_twist), None))
        if left_value is None or right_value is None:
            return False
        if abs(left_value - right_value) > _ZERO_EPSILON:
            return False
    return True


def _linear_motion(value: Any) -> bool:
    twist = _mapping(value)
    vx = _number(twist.get("vx", twist.get("x")))
    vy = _number(twist.get("vy", twist.get("y")))
    return bool(
        vx is not None
        and vy is not None
        and (abs(vx) > _ZERO_EPSILON or abs(vy) > _ZERO_EPSILON)
    )


def _status_entry(
    snapshot: Mapping[str, Any], name: str
) -> tuple[Mapping[str, Any], Mapping[str, Any]]:
    status_files = _mapping(snapshot.get("status_files"))
    entry = _mapping(status_files.get(name))
    return entry, _mapping(entry.get("json"))


def _required_strings(value: Any) -> frozenset[str]:
    if not isinstance(value, (list, tuple, set, frozenset)):
        return frozenset()
    return frozenset(_text(item) for item in value if _text(item))


def _observed(mapping: Mapping[str, Any], keys: tuple[str, ...]) -> dict[str, Any]:
    return {key: mapping.get(key) for key in keys}


def _target_host(value: Any) -> str:
    target = _text(value).lower()
    if "://" in target:
        target = target.split("://", 1)[1]
    target = target.split("/", 1)[0]
    if target.startswith("["):
        closing = target.find("]")
        return target[1:closing] if closing > 0 else target
    if target.count(":") == 1:
        return target.rsplit(":", 1)[0]
    return target


_LOOPBACK_OR_UNSPECIFIED_HOSTS = frozenset(
    {"", "localhost", "127.0.0.1", "::1", "0.0.0.0", "::"}  # noqa: S104
)


def _evaluate_maps_status(
    evaluation: _Evaluation,
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
        _positive_int(maps.get("generation"))
        and _positive_int(accepted)
        and _positive_int(processed)
        and processed <= accepted
        and _positive_int(dds_received)
        and _positive_int(dds_decoded)
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
        _positive_int(maps.get("dds_write_attempts"))
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
        detail=(
            "voxel_snapshot_omitted_cells is reported as bounded-scene evidence; "
            "it is not the /nav/traversability safety authority"
        ),
    )
    evaluation.check(
        "maps.resource_accounting",
        _non_negative_int(maps.get("voxel_snapshot_omitted_cells")),
        expected="non-negative bounded voxel snapshot omission count",
        observed={
            "voxel_snapshot_omitted_cells": maps.get(
                "voxel_snapshot_omitted_cells"
            )
        },
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
        detail=(
            "internal map storage exhaustion is a hard preflight blocker; "
            "bounded scene snapshot omission remains separately accounted"
        ),
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
        and _positive_int(generation)
        and all(
            _positive_int(value) and value == generation
            for value in publication_generations.values()
        ),
        expected=(
            "all required mapd channels published at least once and each "
            "publication cursor equals the current generation"
        ),
        observed={
            "generation": generation,
            "required_publications_ready": maps.get(
                "required_publications_ready"
            ),
            "current_generation_published": maps.get(
                "current_generation_published"
            ),
            **publication_generations,
        },
    )


def evaluate_teleop_avoid_preflight(
    snapshot: Mapping[str, Any],
    *,
    stage: str = "contract",
    status_max_age_s: float = _STATUS_MAX_AGE_S,
) -> dict[str, Any]:
    """Evaluate collected field evidence without touching runtime state.

    ``contract`` confirms the committed Product and native process contracts.
    ``motion`` additionally requires live input/control readiness and an exact
    Nav Endpoint -> Driver output acknowledgement. An active operator sample
    is not required in either stage.
    """

    normalized_stage = _text(stage).lower()
    if normalized_stage not in STAGES:
        raise ValueError(f"unsupported teleop_avoid preflight stage: {stage!r}")
    max_age_s = _number(status_max_age_s)
    if max_age_s is None or max_age_s <= 0.0:
        raise ValueError("status_max_age_s must be a positive finite number")

    evaluation = _Evaluation(normalized_stage)
    current_run = _mapping(snapshot.get("current_run"))
    state = _mapping(current_run.get("state"))
    run_plan = _mapping(current_run.get("run_plan"))
    current_fingerprint = _text(state.get("fingerprint"))
    plan_fingerprint = _text(run_plan.get("fingerprint"))

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
        current_run.get("plan_verified") is True,
        expected=True,
        observed=current_run.get("plan_verified"),
        detail=_text(current_run.get("error")),
    )
    evaluation.check(
        "product.identity",
        state.get("product") == _PRODUCT and run_plan.get("product") == _PRODUCT,
        expected=_PRODUCT,
        observed={"current": state.get("product"), "run_plan": run_plan.get("product")},
    )
    evaluation.check(
        "product.env",
        state.get("env") == _ENV and run_plan.get("env") == _ENV,
        expected=_ENV,
        observed={"current": state.get("env"), "run_plan": run_plan.get("env")},
    )
    evaluation.check(
        "product.fingerprint_match",
        bool(current_fingerprint)
        and current_fingerprint == plan_fingerprint
        and current_fingerprint == _text(current_run.get("verified_fingerprint")),
        expected="current == verified run_plan",
        observed={
            "current": current_fingerprint,
            "run_plan": plan_fingerprint,
            "verified": current_run.get("verified_fingerprint"),
        },
    )

    capabilities = _required_strings(run_plan.get("required_capabilities"))
    topics = _required_strings(run_plan.get("required_topics"))
    missing_capabilities = sorted(_REQUIRED_CAPABILITIES - capabilities)
    missing_topics = sorted(_REQUIRED_TOPICS - topics)
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

    native_nav = _mapping(run_plan.get("native_nav"))
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

    nav_entry, nav = _status_entry(snapshot, "nav")
    traversability_entry, traversability = _status_entry(snapshot, "traversability")
    maps_entry, maps = _status_entry(snapshot, "maps")
    driver_entry, driver = _status_entry(snapshot, "driver")
    for name, entry, payload, schema in (
        ("nav", nav_entry, nav, _NAV_STATUS_SCHEMA),
        ("traversability", traversability_entry, traversability, _TRAVERSABILITY_STATUS_SCHEMA),
        ("maps", maps_entry, maps, _MAPS_STATUS_SCHEMA),
        ("driver", driver_entry, driver, _DRIVER_STATUS_SCHEMA),
    ):
        evaluation.check(
            f"status.{name}.fresh",
            _fresh(entry, max_age_s),
            expected=f"exists and age_s <= {max_age_s:g}",
            observed={"exists": entry.get("exists"), "age_s": entry.get("age_s")},
        )
        evaluation.check(
            f"status.{name}.schema",
            payload.get("schema_version") == schema,
            expected=schema,
            observed=payload.get("schema_version"),
        )

    _evaluate_maps_status(evaluation, maps)

    nav_runtime = _mapping(nav.get("native_product"))
    evaluation.check(
        "nav.product_runtime_identity",
        nav_runtime.get("product") == _PRODUCT
        and _text(nav_runtime.get("config_fingerprint")) == current_fingerprint,
        expected={"product": _PRODUCT, "config_fingerprint": current_fingerprint},
        observed={
            "product": nav_runtime.get("product"),
            "config_fingerprint": nav_runtime.get("config_fingerprint"),
        },
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

    operator_motion = _mapping(nav.get("operator_motion"))
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
        operator_motion.get("ack_publish_failed") == 0
        and operator_motion.get("status_publish_failed") == 0,
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
    driver_dds = _mapping(driver.get("dds"))
    evaluation.check(
        "driver.boundary",
        driver.get("role") == "driver"
        and driver.get("backend") == "thunder"
        and driver_dds.get("topic") == "/nav/cmd_vel"
        and driver_dds.get("wire_topic") == "rt/nav/cmd_vel",
        expected={
            "role": "driver",
            "backend": "thunder",
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
        )

    final_output = _mapping(nav.get("final_output"))
    driver_ack = _mapping(driver.get("output_ack"))
    return evaluation.result(
        evidence={
            "product": state.get("product"),
            "env": state.get("env"),
            "fingerprint": current_fingerprint,
            "status_paths": {
                "nav": nav_entry.get("path"),
                "traversability": traversability_entry.get("path"),
                "maps": maps_entry.get("path"),
                "driver": driver_entry.get("path"),
            },
            "maps_runtime": {
                "process": maps.get("process"),
                "status": maps.get("status"),
                "generation": maps.get("generation"),
                "accepted_observations": maps.get("accepted_observations"),
                "processed_observations": maps.get("processed_observations"),
                "dds_received": maps.get("dds_received"),
                "dds_decoded": maps.get("dds_decoded"),
                "dds_rejected": maps.get("dds_rejected"),
                "required_publications_ready": maps.get(
                    "required_publications_ready"
                ),
                "current_generation_published": maps.get(
                    "current_generation_published"
                ),
                "scene_published_generation": maps.get(
                    "scene_published_generation"
                ),
                "voxel_snapshot_omitted_cells": maps.get(
                    "voxel_snapshot_omitted_cells"
                ),
                "capacity_limited": maps.get("capacity_limited"),
                "voxel_capacity_rejections": maps.get(
                    "voxel_capacity_rejections"
                ),
                "accumulated_capacity_rejections": maps.get(
                    "accumulated_capacity_rejections"
                ),
                "dds_scene_oversize_rejections": maps.get(
                    "dds_scene_oversize_rejections"
                ),
            },
            "final_output": {
                "producer_boot_id": final_output.get("producer_boot_id"),
                "output_sequence": final_output.get("output_sequence"),
                "driver_acknowledged": final_output.get("driver_acknowledged"),
            },
            "driver_output_ack": {
                "producer_boot_id": driver_ack.get("producer_boot_id"),
                "output_sequence": driver_ack.get("output_sequence"),
                "accepted": driver_ack.get("accepted"),
            },
        }
    )


def _evaluate_motion_stage(
    evaluation: _Evaluation,
    *,
    snapshot: Mapping[str, Any],
    nav: Mapping[str, Any],
    traversability: Mapping[str, Any],
    driver: Mapping[str, Any],
) -> None:
    nav_counters = _mapping(nav.get("counters"))
    traversability_counters = _mapping(traversability.get("counters"))
    evaluation.check(
        "motion.sensor_chain",
        nav.get("has_odom") is True
        and nav.get("has_traversability") is True
        and _positive_int(nav_counters.get("odom"))
        and _positive_int(nav_counters.get("registered_clouds"))
        and _positive_int(nav_counters.get("traversability"))
        and traversability.get("has_odom") is True
        and _positive_int(traversability_counters.get("odom"))
        and _positive_int(traversability_counters.get("registered_clouds"))
        and _positive_int(traversability_counters.get("published")),
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

    input_gate = _mapping(nav.get("input_gate"))
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
    control_loop = _mapping(nav.get("control_loop_health"))
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

    driver_readiness = _mapping(snapshot.get("driver_readiness"))
    evaluation.check(
        "motion.driver_readiness",
        driver_readiness.get("ok") is True,
        expected=True,
        observed={
            "ok": driver_readiness.get("ok"),
            "blockers": driver_readiness.get("blockers"),
        },
    )
    brainstem = _mapping(driver.get("brainstem"))
    target_host = _target_host(brainstem.get("target"))
    expected_host = _target_host(snapshot.get("expected_brainstem_host"))
    evaluation.check(
        "motion.brainstem_control",
        driver.get("connected") is True
        and target_host not in _LOOPBACK_OR_UNSPECIFIED_HOSTS
        and (not expected_host or target_host == expected_host)
        and driver.get("ready") is True
        and _mapping(driver.get("dds")).get("cmd_vel_writer_ready") is True
        and _mapping(driver.get("dds")).get("matched_cmd_vel_writers") == 1
        and brainstem.get("initial_zero_acknowledged") is True
        and brainstem.get("motors_enabled") is True
        and brainstem.get("critical_fault") is False
        and brainstem.get("lease_valid") is True
        and brainstem.get("owner") == "grpc"
        and brainstem.get("owner_id") == "lingtu-driver"
        and brainstem.get("fsm") in {"standing", "walking"},
        expected=(
            "connected ready remote Brainstem with motors, Lingtu lease, "
            "and standing/walking FSM"
        ),
        observed={
            "connected": driver.get("connected"),
            "target": brainstem.get("target"),
            "expected_host": expected_host or None,
            "ready": driver.get("ready"),
            "cmd_vel_writer_ready": _mapping(driver.get("dds")).get("cmd_vel_writer_ready"),
            "matched_cmd_vel_writers": _mapping(driver.get("dds")).get("matched_cmd_vel_writers"),
            "initial_zero_acknowledged": brainstem.get("initial_zero_acknowledged"),
            "motors_enabled": brainstem.get("motors_enabled"),
            "critical_fault": brainstem.get("critical_fault"),
            "lease_valid": brainstem.get("lease_valid"),
            "owner": brainstem.get("owner"),
            "owner_id": brainstem.get("owner_id"),
            "fsm": brainstem.get("fsm"),
        },
    )

    final_output = _mapping(nav.get("final_output"))
    driver_control = _mapping(nav.get("driver_control"))
    driver_ack = _mapping(driver.get("output_ack"))
    producer_boot_id = _text(final_output.get("producer_boot_id"))
    output_sequence = final_output.get("output_sequence")
    exact_ack = (
        final_output.get("published") is True
        and bool(producer_boot_id)
        and _positive_int(output_sequence)
        and final_output.get("driver_acknowledged") is True
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
        "motion.exact_driver_ack",
        exact_ack,
        expected=(
            "one exact producer_boot_id/output_sequence across nav publication, "
            "driver control, and driver ACK"
        ),
        observed={
            "nav": dict(final_output),
            "nav_driver_control": dict(driver_control),
            "driver": dict(driver_ack),
        },
    )

    operator_motion = _mapping(nav.get("operator_motion"))
    last_ack = _mapping(operator_motion.get("last_ack"))
    operator_status = _mapping(operator_motion.get("status"))
    evaluation.check(
        "motion.operator_status_mirror",
        operator_status.get("observed") is True
        and operator_status.get("published") is True
        and (
            last_ack.get("observed") is not True
            or last_ack.get("published") is True
        )
        and operator_status.get("input_gate_reason") == input_gate.get("reason")
        and _twist_equal(operator_status.get("final_cmd_vel"), nav.get("final_cmd_vel"))
        and _twist_equal(
            operator_status.get("teleop_output"),
            _mapping(nav.get("teleop")).get("output"),
        ),
        expected="published operator status mirrors input gate, teleop output, and final output",
        observed={"last_ack": dict(last_ack), "status": dict(operator_status)},
    )
    active_sample = operator_status.get("has_active_sample") is True
    evaluation.check(
        "motion.operator_sample_correlation",
        not active_sample
        or (
            _positive_int(operator_status.get("admitted_sequence"))
            and operator_status.get("final_output_sequence") == output_sequence
            and _twist_equal(operator_status.get("final_cmd_vel"), nav.get("final_cmd_vel"))
            and _twist_equal(
                operator_status.get("teleop_output"),
                _mapping(nav.get("teleop")).get("output"),
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

    teleop = _mapping(nav.get("teleop"))
    linear_motion = active_sample and _linear_motion(teleop.get("request"))
    last_local = _mapping(nav.get("last_local"))
    final_safety = _mapping(last_local.get("final_safety"))
    evaluation.check(
        "motion.assisted_local_path",
        not linear_motion
        or (
            teleop.get("seen") is True
            and teleop.get("published") is True
            and _positive_int(nav.get("local_path_points"))
            and last_local.get("seen") is True
            and final_safety.get("applied") is True
        ),
        expected=(
            "idle/rotation-only, or linear operator motion with LocalPlanner path "
            "and final safety evidence"
        ),
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
            _zero_twist(nav.get("final_cmd_vel"))
            and _zero_twist(operator_status.get("final_cmd_vel"))
            and _zero_twist(driver.get("last_walk"))
        ),
        expected="when no active operator sample, nav/operator/driver outputs are zero",
        observed={
            "idle": idle,
            "nav_final_cmd_vel": nav.get("final_cmd_vel"),
            "operator_final_cmd_vel": operator_status.get("final_cmd_vel"),
            "driver_last_walk": driver.get("last_walk"),
        },
    )


__all__ = [
    "SCHEMA_VERSION",
    "STAGES",
    "evaluate_teleop_avoid_preflight",
]

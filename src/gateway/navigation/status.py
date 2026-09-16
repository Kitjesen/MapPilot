"""Navigation admission, native evidence, and public status publication."""

from __future__ import annotations

import logging
import math
import os
from collections.abc import Mapping
from typing import Any

from gateway.navigation.projection import project_navigation_status
from gateway.services.native_control import endpoint_only_enabled
from gateway.services.native_control import status_is_fresh as native_control_status_is_fresh
from gateway.services.native_status import read_navigation_status
from gateway.services.runtime_status import (
    LOST_STATES,
    POSE_FRESH_MAX_ODOM_AGE_MS,
    _active_recovery_signal,
    _as_float,
    _as_optional_bool,
    _mapping,
    _reported_state,
    classify_pose_freshness,
    compiled_session_context,
    safe_session,
)
from gateway.services.safety_status import safety_stop_active

logger = logging.getLogger(__name__)
NAVIGATION_STATUS_SCHEMA_VERSION = 3


def _as_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _finite_float(value: Any) -> float | None:
    parsed = _as_float(value, None)
    if parsed is None or not math.isfinite(parsed):
        return None
    return parsed


def _session_mode(session: Mapping[str, Any]) -> str:
    mode = str(session.get("mode") or "unknown").strip().lower()
    return mode or "unknown"


def _native_endpoint_readiness(
    session: Mapping[str, Any],
    gw: Any | None = None,
) -> dict[str, Any]:
    plan = getattr(gw, "_compiled_run_plan", None) if gw is not None else None
    raw_product = str(
        getattr(plan, "product", "") or session.get("product") or os.environ.get("LINGTU_PRODUCT") or ""
    ).strip()
    product = raw_product
    required_capabilities = tuple(getattr(plan, "required_capabilities", ()) if plan is not None else ())
    operator_motion_required = bool(
        {
            "operator_motion_typed_dds_interface",
            "native_operator_motion_authority",
        }
        & set(required_capabilities)
    )
    native_nav = getattr(plan, "native_nav", {}) if plan is not None else {}
    expected_control_mode = (
        str(native_nav.get("control_mode", "autonomy") if isinstance(native_nav, Mapping) else "autonomy")
        .strip()
        .lower()
    )
    product_requires_native_endpoint = plan is not None and plan.has_process("nav")
    managed_run_plan_missing = bool(
        plan is None and gw is not None and getattr(gw, "_compiled_command_output_mode", "") == "endpoint_only"
    )
    legacy_runtime_requires_native_endpoint = (
        plan is None and endpoint_only_enabled(gw) and _session_mode(session) in {"navigating", "exploring"}
    )
    required = product_requires_native_endpoint or managed_run_plan_missing or legacy_runtime_requires_native_endpoint
    if not required:
        return {
            "required": False,
            "ok": None,
            "navigation_ready": None,
            "blockers": [],
            "far_input": {"required": False, "ready": True, "reason": "not_required"},
            "input_gate": {},
            "status_available": None,
            "active_cmd_source": None,
            "control_authority": {},
            "expected_control_mode": expected_control_mode,
            "operator_motion": {
                "required": operator_motion_required,
                "status_available": None,
            },
        }
    if managed_run_plan_missing:
        return {
            "required": True,
            "ok": False,
            "navigation_ready": False,
            "blockers": ["run_plan_missing"],
            "far_input": {},
            "input_gate": {},
            "status_available": None,
            "active_cmd_source": "unknown",
            "control_authority": {},
            "expected_control_mode": expected_control_mode,
            "operator_motion": {
                "required": False,
                "status_available": None,
            },
        }
    snapshot = read_navigation_status()
    if not native_control_status_is_fresh(snapshot):
        return {
            "required": True,
            "ok": False,
            "navigation_ready": False,
            "blockers": ["native_endpoint_status_missing_or_stale"],
            "far_input": {},
            "input_gate": {},
            "status_available": False,
            "active_cmd_source": "unknown",
            "control_authority": {},
            "expected_control_mode": expected_control_mode,
            "operator_motion": {
                "required": operator_motion_required,
                "status_available": False,
            },
        }
    input_gate = _mapping(snapshot.get("input_gate"))
    control_loop_health = _mapping(snapshot.get("control_loop_health"))
    blockers: list[str] = []
    control_loop_reason = str(control_loop_health.get("reason") or "").strip().lower()
    if not control_loop_health:
        blockers.append("native_control_loop_health_unavailable")
    elif control_loop_health.get("ready") is False:
        if control_loop_reason != "warming_up":
            blockers.append("native_control_loop_health_unavailable")
    elif control_loop_health.get("ready") is not True:
        blockers.append("native_control_loop_health_unavailable")
    if input_gate.get("ready") is not True:
        blockers.append("native_input_gate_not_ready")
    active_cmd_source = str(snapshot.get("active_cmd_source") or "").strip().lower()
    if active_cmd_source not in {"none", "autonomy", "teleop", "manual_hold", "estop"}:
        blockers.append("native_active_cmd_source_invalid")
    control_authority = _mapping(snapshot.get("control_authority"))
    # The native runtime owns motion holds. Rolling performance warnings are
    # diagnostics, not a second admission policy in Gateway.
    if control_authority.get("control_loop_hold") is True:
        blockers.append("native_control_loop_unhealthy")
    if str(control_authority.get("owner") or "").strip().lower() != "native_endpoint":
        blockers.append("native_control_authority_invalid")
    estop_latched = control_authority.get("estop_latched") is True
    operator_takeover_latched = control_authority.get("operator_takeover_latched") is True
    resume_required = control_authority.get("resume_required") is True
    if estop_latched or active_cmd_source == "estop":
        blockers.append("native_estop_latched")
    if operator_takeover_latched or resume_required or active_cmd_source in {"teleop", "manual_hold"}:
        blockers.append("native_resume_required")
    control_mode = str(snapshot.get("control_mode") or "").strip().lower()
    parameter_mismatches: dict[str, dict[str, float | None]] = {}
    expected_parameters: dict[str, float] = {}
    observed_native_product = _mapping(snapshot.get("native_product"))
    native_product_required = product_requires_native_endpoint
    if native_product_required:
        actual_product = str(observed_native_product.get("product") or "").strip()
        if actual_product != product:
            blockers.append("native_product_mismatch")

        expected_environment = getattr(plan, "native_process_environment", {})
        parameter_environment = (
            ("path_follower_max_speed_mps", "LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS"),
            ("path_follower_min_speed_mps", "LINGTU_NAV_PATH_FOLLOWER_MIN_SPEED_MPS"),
            ("path_follower_max_accel_mps2", "LINGTU_NAV_PATH_FOLLOWER_MAX_ACCEL_MPS2"),
            ("path_follower_lookahead_m", "LINGTU_NAV_PATH_FOLLOWER_LOOKAHEAD_M"),
            ("path_follower_goal_tolerance_m", "LINGTU_NAV_PATH_FOLLOWER_GOAL_TOLERANCE_M"),
            ("waypoint_reached_m", "LINGTU_NAV_WAYPOINT_REACHED_M"),
            ("goal_reached_m", "LINGTU_NAV_GOAL_REACHED_M"),
            ("corridor_lookahead_m", "LINGTU_NAV_CORRIDOR_LOOKAHEAD_M"),
            ("teleop_planner_horizon_m", "LINGTU_TELEOP_PLANNER_HORIZON_M"),
            ("teleop_planner_max_deviation_deg", "LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG"),
        )
        for parameter, environment_name in parameter_environment:
            value = _finite_float(expected_environment.get(environment_name))
            if value is None:
                expected_parameters.clear()
                break
            expected_parameters[parameter] = value
        if not expected_parameters:
            blockers.append("native_product_expectation_unavailable")
        else:
            observed_path_follower = _mapping(snapshot.get("path_follower"))
            observed_nav_loop = _mapping(snapshot.get("nav_loop"))
            observed_parameters = (
                (
                    "path_follower.max_speed_mps",
                    observed_path_follower.get("max_speed_mps"),
                    "path_follower_max_speed_mps",
                ),
                (
                    "path_follower.min_speed_mps",
                    observed_path_follower.get("min_speed_mps"),
                    "path_follower_min_speed_mps",
                ),
                (
                    "path_follower.max_accel_mps2",
                    observed_path_follower.get("max_accel_mps2"),
                    "path_follower_max_accel_mps2",
                ),
                (
                    "path_follower.lookahead_m",
                    observed_path_follower.get("lookahead_m"),
                    "path_follower_lookahead_m",
                ),
                (
                    "path_follower.goal_tolerance_m",
                    observed_path_follower.get("goal_tolerance_m"),
                    "path_follower_goal_tolerance_m",
                ),
                (
                    "nav_loop.waypoint_reached_m",
                    observed_nav_loop.get("waypoint_reached_m"),
                    "waypoint_reached_m",
                ),
                (
                    "nav_loop.goal_reached_m",
                    observed_nav_loop.get("goal_reached_m"),
                    "goal_reached_m",
                ),
                (
                    "nav_loop.corridor_lookahead_m",
                    observed_nav_loop.get("corridor_lookahead_m"),
                    "corridor_lookahead_m",
                ),
            )
            for field, raw_actual, expected_key in observed_parameters:
                actual = _as_float(raw_actual, None)
                expected = float(expected_parameters[expected_key])
                if actual is None or not math.isclose(
                    actual,
                    expected,
                    rel_tol=1e-9,
                    abs_tol=1e-9,
                ):
                    parameter_mismatches[field] = {
                        "actual": actual,
                        "expected": expected,
                    }
            if parameter_mismatches:
                blockers.append("native_product_parameters_mismatch")
    if control_mode != expected_control_mode:
        blockers.append("native_control_mode_mismatch")
    global_planner = str(snapshot.get("global_planner") or "").strip().lower()
    far_input = _mapping(snapshot.get("far_input"))
    assisted_teleop_required = plan is not None and bool(
        {
            "operator_assisted_local_planner_control",
            "operator_assisted_local_planner_takeover",
        }
        & set(required_capabilities)
    )
    teleop_local_planner = _as_optional_bool(snapshot.get("teleop_local_planner"))
    check_obstacle = _as_optional_bool(snapshot.get("check_obstacle"))
    use_traversability_cost = _as_optional_bool(snapshot.get("use_traversability_cost"))
    if assisted_teleop_required and expected_parameters:
        for field, raw_actual, expected_key in (
            (
                "teleop_planner_horizon_m",
                snapshot.get("teleop_planner_horizon_m"),
                "teleop_planner_horizon_m",
            ),
            (
                "teleop_planner_max_deviation_deg",
                snapshot.get("teleop_planner_max_deviation_deg"),
                "teleop_planner_max_deviation_deg",
            ),
        ):
            actual = _as_float(raw_actual, None)
            expected = float(expected_parameters[expected_key])
            if actual is None or not math.isclose(actual, expected, rel_tol=1e-9, abs_tol=1e-9):
                parameter_mismatches[field] = {"actual": actual, "expected": expected}
        if parameter_mismatches and "native_product_parameters_mismatch" not in blockers:
            blockers.append("native_product_parameters_mismatch")
    if assisted_teleop_required:
        if teleop_local_planner is not True:
            blockers.append("native_teleop_local_planner_disabled")
        if check_obstacle is not True:
            blockers.append("native_obstacle_check_disabled")
        expected_traversability_cost = _as_optional_bool(_mapping(native_nav).get("use_traversability_cost"))
        if expected_traversability_cost is None or use_traversability_cost is not expected_traversability_cost:
            blockers.append(
                "native_traversability_cost_disabled"
                if expected_traversability_cost is True
                else "native_traversability_cost_mismatch"
            )
    expected_global_planner = (
        str(native_nav.get("global_planner") or "" if isinstance(native_nav, Mapping) else "").strip().lower()
    )
    if not expected_global_planner:
        expected_global_planner = (
            str(
                session.get("global_planner")
                or session.get("planner")
                or os.environ.get("NAV_GLOBAL_PLANNER")
                or "octoplanner3d"
            )
            .strip()
            .lower()
        )
    aliases = {"octo": "octoplanner3d", "octplanner": "octoplanner3d"}
    global_planner = aliases.get(global_planner, global_planner)
    expected_global_planner = aliases.get(expected_global_planner, expected_global_planner)
    if global_planner == "far":
        if not far_input:
            blockers.append("native_far_input_status_missing")
        elif far_input.get("required") is not True:
            blockers.append("native_far_input_contract_invalid")
        elif far_input.get("ready") is not True:
            blockers.append("native_far_input_not_ready")
        elif not str(far_input.get("map_id") or "").strip() or _as_int(far_input.get("content_epoch"), 0) <= 0:
            blockers.append("native_far_input_identity_invalid")
    planner_map = str(snapshot.get("planner_map") or "").strip()
    operator_motion = _mapping(snapshot.get("operator_motion"))
    operator_motion_status_available = bool(operator_motion)
    if operator_motion_required:
        if not operator_motion_status_available:
            blockers.append("native_operator_motion_status_missing")
        else:
            if _as_int(operator_motion.get("schema_version"), -1) != 1:
                blockers.append("native_operator_motion_schema_mismatch")
            if operator_motion.get("interface_enabled") is not True:
                blockers.append("native_operator_motion_interface_disabled")
            if str(operator_motion.get("authority_owner") or "").strip().lower() != "native_endpoint":
                blockers.append("native_operator_motion_authority_invalid")
            if str(operator_motion.get("control_mode") or "").strip().lower() != expected_control_mode:
                blockers.append("native_operator_motion_control_mode_mismatch")
            if operator_motion.get("control_ack_scope") != "claim_hold_release":
                blockers.append("native_operator_motion_ack_scope_invalid")
            if operator_motion.get("sample_evidence") != "status_sequences":
                blockers.append("native_operator_motion_sample_evidence_invalid")
    lifecycle = getattr(plan, "lifecycle", {}) if plan is not None else {}
    requires_map = bool(isinstance(lifecycle, Mapping) and lifecycle.get("requires_map") is True)
    global_planner_required = (
        plan is None
        or requires_map
        or bool({"global_planning", "octoplanner3d_global_planning"} & set(required_capabilities))
    )
    planner_map_required = plan is None or requires_map
    if global_planner_required:
        if not global_planner:
            blockers.append("native_global_planner_missing")
        elif global_planner != expected_global_planner:
            blockers.append("native_global_planner_mismatch")
    if planner_map_required and not planner_map:
        blockers.append("native_planner_map_missing")
    publish_cmd_vel = _as_optional_bool(snapshot.get("publish_cmd_vel"))
    if publish_cmd_vel is not True:
        blockers.append("native_cmd_vel_publish_disabled")
    reported_navigation_ready = _as_optional_bool(snapshot.get("navigation_ready"))
    if not blockers and reported_navigation_ready is not True:
        blockers.append(
            "native_navigation_not_ready"
            if reported_navigation_ready is False
            else "native_navigation_ready_unavailable"
        )
    blockers = list(dict.fromkeys(blockers))
    return {
        "required": True,
        "ok": not blockers,
        "navigation_ready": reported_navigation_ready is True and not blockers,
        "blockers": blockers,
        "status_available": True,
        "active_cmd_source": active_cmd_source,
        "control_authority": control_authority,
        "stamp_s": _as_float(snapshot.get("stamp_s"), None),
        "control_loop_health": control_loop_health,
        "input_gate": input_gate,
        "control_mode": control_mode,
        "expected_control_mode": expected_control_mode,
        "native_product": observed_native_product,
        "parameter_mismatches": parameter_mismatches,
        "global_planner": global_planner,
        "expected_global_planner": expected_global_planner,
        "publish_cmd_vel": publish_cmd_vel,
        "assisted_teleop_required": assisted_teleop_required,
        "teleop_local_planner": teleop_local_planner,
        "check_obstacle": check_obstacle,
        "use_traversability_cost": use_traversability_cost,
        "far_input": dict(far_input),
        "motion_stop_evidence": _mapping(_mapping(snapshot).get("motion_stop_evidence")),
        "operator_motion": {
            "required": operator_motion_required,
            "status_available": operator_motion_status_available,
            **operator_motion,
        },
    }


def evaluate_navigation_gate(
    gw: Any,
    *,
    facts: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Evaluate goal admission once for commands, readiness, and presentation."""

    if facts is None:
        from gateway.services.runtime_facts import capture_runtime_facts

        facts = capture_runtime_facts(gw)
    now = float(facts["ts"])
    native_state = _mapping(facts.get("navigation_state"))
    odometry = _mapping(facts.get("odometry"))
    mode = str(facts.get("mode") or "")
    safety = facts.get("navigation_state")
    localization_status = _mapping(facts.get("localization_status"))
    odometry_received_at = _as_float(facts.get("odometry_received_at"), None)
    task_statuses = _mapping(facts.get("navigation_goal_status_by_task"))
    goal_statuses = _mapping(facts.get("navigation_goal_status_by_request"))

    active_task_id = str(native_state.get("active_task_id") or "")
    active_request_id = str(native_state.get("active_request_id") or "")
    active_boot_id = str(native_state.get("boot_id") or "")
    goal_status: Mapping[str, Any] | None = None
    candidates = (
        goal_statuses.get(active_request_id) if active_request_id else None,
        task_statuses.get(active_task_id) if active_task_id else None,
    )
    for candidate in candidates:
        if not isinstance(candidate, Mapping):
            continue
        if str(candidate.get("task_id") or "") != active_task_id:
            continue
        if str(candidate.get("request_id") or "") != active_request_id:
            continue
        if active_boot_id and str(candidate.get("boot_id") or "") != active_boot_id:
            continue
        goal_status = candidate
        break

    session = compiled_session_context(gw) or safe_session(gw)
    native_endpoint = _native_endpoint_readiness(session, gw=gw)
    session_mode = _session_mode(session)
    reported_pose_fresh, _ = classify_pose_freshness(localization_status)
    received_odom_fresh = (
        (now - odometry_received_at) * 1000.0 <= POSE_FRESH_MAX_ODOM_AGE_MS
        if odometry_received_at is not None
        else None
    )
    if reported_pose_fresh is False or received_odom_fresh is False:
        odometry_fresh: bool | None = False
    elif received_odom_fresh is True:
        odometry_fresh = True
    else:
        odometry_fresh = None
    navigation_state_fresh = bool(native_state and native_control_status_is_fresh({"stamp_s": native_state.get("ts")}))
    authority = _mapping(native_endpoint.get("control_authority"))
    active_source = str(native_endpoint.get("active_cmd_source") or "").strip().lower()
    control = {
        "active_cmd_source": active_source,
        "operator_takeover_latched": authority.get("operator_takeover_latched") is True,
        "resume_required": authority.get("resume_required") is True,
        "estop_latched": (
            authority.get("estop_latched") is True
            or active_source == "estop"
            or mode == "estop"
            or safety_stop_active(safety)
        ),
    }

    blockers: list[str] = []
    if control["estop_latched"]:
        blockers.append("estop_latched")
    if control["operator_takeover_latched"]:
        blockers.append("operator_takeover")
    elif control["resume_required"]:
        blockers.append("resume_required")
    input_gate = _mapping(native_endpoint.get("input_gate"))
    if input_gate.get("ready") is False:
        blockers.append(str(input_gate.get("reason") or "input_gate_blocked"))
    if session_mode not in {"navigating", "exploring"}:
        blockers.append("navigation_session_inactive")

    unknown_reason = ""
    endpoint_blockers = [str(code) for code in native_endpoint.get("blockers", []) if str(code)]
    source_unknown_codes = {
        "native_endpoint_status_missing_or_stale",
        "native_navigation_ready_unavailable",
    }
    for code in endpoint_blockers:
        if code in source_unknown_codes:
            unknown_reason = unknown_reason or code
        else:
            blockers.append(code)

    if native_endpoint.get("required") is not True:
        if not odometry:
            unknown_reason = unknown_reason or "odometry_unavailable"
        elif odometry_fresh is not True:
            unknown_reason = unknown_reason or "odometry_stale"

        localization_state = _reported_state(localization_status.get("state"))
        localizer_health = _reported_state(localization_status.get("localizer_health"))
        recovery_signal = _active_recovery_signal(localization_status.get("recovery_signal"))
        if localization_state in LOST_STATES or localizer_health == "LOST":
            blockers.append("localization_unhealthy")
        elif "RELOCAL" in localization_state or recovery_signal:
            blockers.append("localization_recovery_active")

    blockers = list(dict.fromkeys(blockers))
    if blockers:
        can_accept_goal: bool | None = False
        reason = blockers[0]
    elif unknown_reason:
        can_accept_goal = None
        reason = unknown_reason
    else:
        can_accept_goal = True
        reason = ""

    return {
        "can_accept_goal": can_accept_goal,
        "can_execute_autonomy": can_accept_goal is True,
        "blockers": blockers,
        "advisories": [],
        "reason": reason,
        "session_mode": session_mode,
        "native_endpoint": native_endpoint,
        "control": control,
        "navigation_state": native_state,
        "navigation_state_fresh": navigation_state_fresh,
        "goal_status": goal_status,
        "odometry": odometry,
        "odometry_fresh": odometry_fresh,
        "ts": now,
    }


def build_navigation_status(
    gw: Any,
    *,
    facts: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Return the only operator-facing navigation state."""

    gate = evaluate_navigation_gate(gw, facts=facts)
    projection = project_navigation_status(gate)
    return {
        "schema_version": NAVIGATION_STATUS_SCHEMA_VERSION,
        **projection,
        "ts": gate["ts"],
    }


def _publish_navigation_status(gw: Any) -> None:
    try:
        gw.push_event({"type": "navigation_status", "data": build_navigation_status(gw)})
    except Exception as exc:
        logger.debug("navigation status projection failed: %s", exc)


def handle_navigation_state(gw: Any, state: Any) -> None:
    from gateway.services.event_handlers import json_payload

    data = state.to_dict() if hasattr(state, "to_dict") else json_payload(state)
    if not isinstance(data, dict):
        data = {"raw": str(data)}
    with gw._state_lock:
        gw._navigation_state = data
    _publish_navigation_status(gw)


def handle_navigation_goal_status(gw: Any, status: Any) -> None:
    from gateway.services.event_handlers import json_payload

    data = status.to_dict() if hasattr(status, "to_dict") else json_payload(status)
    if not isinstance(data, dict):
        return
    from gateway.navigation.tasks import (
        NavigationTaskProjectionError,
        project_navigation_goal_status,
    )

    try:
        data = project_navigation_goal_status(data)
    except NavigationTaskProjectionError as exc:
        logger.warning("Rejected navigation goal status: %s", exc)
        return
    boot_id = str(data.get("boot_id") or "")
    sequence = int(data.get("sequence") or 0)
    task_id = str(data.get("task_id") or "")
    request_id = str(data.get("request_id") or "")
    if not boot_id or sequence <= 0 or not task_id or not request_id:
        return
    with gw._state_lock:
        sequences = gw._navigation_goal_status_sequences
        if sequence <= int(sequences.get(boot_id, 0)):
            return
        sequences[boot_id] = sequence
        tasks = gw._navigation_goal_status_by_task
        tasks.pop(task_id, None)
        tasks[task_id] = dict(data)
        while len(tasks) > 256:
            tasks.popitem(last=False)
        statuses = gw._navigation_goal_status_by_request
        statuses.pop(request_id, None)
        statuses[request_id] = dict(data)
        while len(statuses) > 256:
            statuses.popitem(last=False)
        gw._latest_navigation_goal_status = dict(data)
    _publish_navigation_status(gw)

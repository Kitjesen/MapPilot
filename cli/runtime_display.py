"""Shared runtime boundary formatting for CLI operator output."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from runtime.runtime_interface import (
    REAL_RUNTIME_CONTRACT,
    runtime_required_topic_frame_ids,
)

TOPIC_FRAME_SUMMARY_TOPICS = runtime_required_topic_frame_ids(REAL_RUNTIME_CONTRACT)


def _get_attr_or_item(runtime: Any, name: str, default: Any = None) -> Any:
    if isinstance(runtime, Mapping):
        return runtime.get(name, default)
    return getattr(runtime, name, default)


def _has_attr_or_item(runtime: Any, name: str) -> bool:
    if isinstance(runtime, Mapping):
        return name in runtime
    return hasattr(runtime, name)


def join_runtime_items(items: Any) -> str:
    if isinstance(items, str):
        return items
    values = tuple(str(item) for item in (items or ()) if item)
    return ",".join(values) if values else "none"


def format_runtime_boundary(runtime: Any) -> str:
    endpoint = _get_attr_or_item(runtime, "endpoint") or "in_process"
    runtime_contract = _get_attr_or_item(runtime, "runtime_contract") or "none"
    simulation_only = str(bool(_get_attr_or_item(runtime, "simulation_only"))).lower()
    text = (
        f"endpoint={endpoint} "
        f"data_source={_get_attr_or_item(runtime, 'data_source')} "
        f"runtime_contract={runtime_contract} "
        f"module_transport={_get_attr_or_item(runtime, 'module_transport', 'local')} "
        f"endpoint_transport={_get_attr_or_item(runtime, 'endpoint_transport', 'local')} "
        f"command_sink={_get_attr_or_item(runtime, 'command_sink')} "
        f"simulation_only={simulation_only}"
    )
    endpoint_contract = _get_attr_or_item(runtime, "endpoint_contract")
    if endpoint_contract:
        text = f"{text} endpoint_contract={endpoint_contract}"
    localization_adapter = _get_attr_or_item(runtime, "localization_adapter")
    if localization_adapter:
        text = f"{text} localization_adapter={localization_adapter}"
    return text


def format_runtime_sources(runtime: Any) -> str:
    parts = [
        f"slam_source={_get_attr_or_item(runtime, 'slam_source')}",
        f"localization_source={_get_attr_or_item(runtime, 'localization_source')}",
        f"mapping_source={_get_attr_or_item(runtime, 'mapping_source')}",
    ]
    if _has_attr_or_item(runtime, "lidar_extrinsic_profile"):
        lidar_profile = _get_attr_or_item(runtime, "lidar_extrinsic_profile") or "none"
        parts.append(f"lidar_extrinsic={lidar_profile}")
    return " ".join(parts)


def format_runtime_algorithms(runtime: Any) -> str:
    global_planner = _get_attr_or_item(runtime, "global_planner") or "unknown"
    fallback_planners = join_runtime_items(
        _get_attr_or_item(runtime, "fallback_global_planners")
    )
    latency_budget = _get_attr_or_item(runtime, "planner_latency_budget_ms")
    safety_policy = _get_attr_or_item(runtime, "plan_safety_policy") or "unknown"
    autonomy = _get_attr_or_item(runtime, "autonomy_backends") or {}
    if isinstance(autonomy, Mapping):
        terrain_backend = autonomy.get("terrain_backend", "unknown")
        local_planner_backend = autonomy.get("local_planner_backend", "unknown")
        path_follower_backend = autonomy.get("path_follower_backend", "unknown")
    else:
        terrain_backend = "unknown"
        local_planner_backend = "unknown"
        path_follower_backend = "unknown"
    return (
        f"global_planner={global_planner} "
        f"fallback={fallback_planners} "
        f"latency_budget_ms={latency_budget if latency_budget is not None else 'unknown'} "
        f"plan_safety_policy={safety_policy} "
        f"terrain={terrain_backend} "
        f"local_planner={local_planner_backend} "
        f"path_follower={path_follower_backend}"
    )


def format_runtime_frames(runtime: Any) -> str:
    frames = _get_attr_or_item(runtime, "frames") or {}
    if not isinstance(frames, Mapping):
        return "none"
    ordered = (
        "map",
        "odom",
        "body",
        "lidar",
        "real_lidar",
        "camera",
        "axis_convention",
    )
    parts: list[str] = []
    for name in ordered:
        value = frames.get(name)
        if value:
            parts.append(f"{name}={value}")
    return " ".join(parts) if parts else "none"


def format_frame_links(runtime: Any) -> str:
    raw_links = _get_attr_or_item(runtime, "frame_links") or {}
    if not isinstance(raw_links, Mapping):
        return "none"
    links: list[str] = []
    for link in raw_links.values():
        if not isinstance(link, Mapping):
            continue
        parent = link.get("parent")
        child = link.get("child")
        if parent and child:
            links.append(f"{parent}->{child}")
    return ", ".join(links) if links else "none"


def format_runtime_topic_frames(runtime: Any) -> str:
    frame_rules = _get_attr_or_item(runtime, "topic_allowed_frame_ids") or {}
    if not isinstance(frame_rules, Mapping):
        return "none"
    ordered_topics = [
        topic for topic in TOPIC_FRAME_SUMMARY_TOPICS if topic in frame_rules
    ]
    ordered_topics.extend(topic for topic in frame_rules if topic not in ordered_topics)
    parts: list[str] = []
    for topic in ordered_topics:
        frames = join_runtime_items(frame_rules.get(topic))
        if frames == "none":
            continue
        label = str(topic).rsplit("/", 1)[-1] or str(topic)
        parts.append(f"{label}={frames}")
    return " ".join(parts) if parts else "none"


def format_runtime_flow(runtime: Any) -> str:
    stages = tuple(_get_attr_or_item(runtime, "resolved_runtime_data_flow") or ())
    first_stage = stages[0] if stages else {}
    slam_stage = next(
        (
            stage
            for stage in stages
            if isinstance(stage, Mapping)
            and stage.get("name") == "slam_or_relayed_localization_map"
        ),
        {},
    )
    command_stage = next(
        (
            stage
            for stage in stages
            if isinstance(stage, Mapping) and stage.get("name") == "command_boundary"
        ),
        {},
    )
    first_stage = first_stage if isinstance(first_stage, Mapping) else {}
    sensors = join_runtime_items(first_stage.get("inputs"))
    localization_map = join_runtime_items(slam_stage.get("outputs"))
    command = join_runtime_items(
        command_stage.get("outputs") or (_get_attr_or_item(runtime, "command_sink"),)
    )
    return f"sensors={sensors} localization_map={localization_map} command={command}"


def format_runtime_flow_stages(runtime: Any) -> str:
    stages = tuple(_get_attr_or_item(runtime, "resolved_runtime_data_flow") or ())
    stage_interfaces = _get_attr_or_item(
        runtime,
        "runtime_data_flow_stage_algorithm_interfaces",
    ) or {}
    if not isinstance(stage_interfaces, Mapping):
        stage_interfaces = {}
    parts: list[str] = []
    for stage in stages:
        if not isinstance(stage, Mapping):
            continue
        name = stage.get("name")
        if not name:
            continue
        inputs = join_runtime_items(stage.get("inputs"))
        outputs = join_runtime_items(stage.get("outputs"))
        owner = stage.get("owner") or "unknown"
        frame_role = stage.get("frame_role") or "unknown"
        interfaces = join_runtime_items(stage_interfaces.get(str(name)))
        interface_prefix = f"interfaces={interfaces} " if interfaces != "none" else ""
        parts.append(
            f"{name}[{owner}|{frame_role}] {interface_prefix}{inputs}->{outputs}"
        )
    return " | ".join(parts) if parts else "none"


def format_product_runtime_boundary(runtime: Any) -> str:
    """Return the product-facing runtime interface boundary."""

    simulation_only = bool(_get_attr_or_item(runtime, "simulation_only"))
    runtime_contract = _get_attr_or_item(runtime, "runtime_contract") or "none"
    mode = "simulation" if simulation_only else "field"
    return (
        f"mode={mode} runtime_contract={runtime_contract} "
        f"module_transport={_get_attr_or_item(runtime, 'module_transport', 'local')} "
        f"endpoint_transport={_get_attr_or_item(runtime, 'endpoint_transport', 'local')} "
        "primary=Gateway+ModulePorts "
        "adapter=endpoint_only "
        "ros2_topic_inspection_required=false"
    )


def format_product_semantic_overrides(runtime: Any) -> str:
    """Return endpoint-induced product semantic changes, if any."""

    raw_overrides = _get_attr_or_item(runtime, "product_semantic_overrides") or ()
    if not isinstance(raw_overrides, (list, tuple)):
        return "none"
    parts: list[str] = []
    for item in raw_overrides:
        if not isinstance(item, Mapping):
            continue
        field = item.get("field")
        if not field:
            continue
        product_value = _format_override_value(item.get("product_value"))
        endpoint_value = _format_override_value(item.get("endpoint_value"))
        parts.append(f"{field}={product_value}->{endpoint_value}")
    return " | ".join(parts) if parts else "none"


def format_product_acceptance_commands(runtime: Any) -> str:
    """Return the smallest useful product acceptance command sequence."""

    simulation_only = bool(_get_attr_or_item(runtime, "simulation_only"))
    runtime_audit = "python lingtu.py runtime-audit"
    gateway_simulation = (
        "python lingtu.py gateway-runtime-acceptance --acceptance-mode simulation"
    )
    if simulation_only:
        return f"{runtime_audit} | {gateway_simulation}"
    real_evidence = (
        "python lingtu.py real-runtime-evidence --collector gateway "
        "--gateway-url http://<robot>:5050 --duration-sec 20 "
        "--json-out artifacts/thunder_field_runtime/report.json"
    )
    gateway_field = (
        "python lingtu.py gateway-runtime-acceptance --acceptance-mode field "
        "--gateway-url http://<robot>:5050"
    )
    return f"{runtime_audit} | {real_evidence} | {gateway_field}"


def format_runtime_switch_plan(payload: Mapping[str, Any]) -> str:
    """Return a human-readable sim/replay/real runtime switch diff."""

    current = payload.get("from")
    target = payload.get("to")
    current_payload = current if isinstance(current, Mapping) else {}
    target_payload = target if isinstance(target, Mapping) else {}
    current_validation = payload.get("current_validation")
    target_validation = payload.get("target_validation")
    current_validation_payload = (
        current_validation if isinstance(current_validation, Mapping) else {}
    )
    target_validation_payload = (
        target_validation if isinstance(target_validation, Mapping) else {}
    )
    ok = payload.get("ok")
    status = "PASS" if ok is True else "FAIL" if ok is False else "UNKNOWN"

    lines = [
        f"Runtime switch plan: {status}",
        "Switch lifecycle: dry-run/preflight; endpoint changes require a fresh launcher or explicit stop/start",
        _format_switch_profile_line("Current", current_payload),
        _format_switch_profile_line("Target", target_payload),
        f"Current runtime: {format_runtime_boundary(current_payload)}",
        f"Target runtime: {format_runtime_boundary(target_payload)}",
        f"Current SLAM: {format_runtime_sources(current_payload)}",
        f"Target SLAM: {format_runtime_sources(target_payload)}",
        f"Current product boundary: {format_product_runtime_boundary(current_payload)}",
        f"Target product boundary: {format_product_runtime_boundary(target_payload)}",
        f"Current frames: {format_runtime_frames(current_payload)}",
        f"Target frames: {format_runtime_frames(target_payload)}",
        f"Current frame links: {format_frame_links(current_payload)}",
        f"Target frame links: {format_frame_links(target_payload)}",
    ]
    product_switch = payload.get("product_mode_switch")
    if isinstance(product_switch, Mapping):
        lines.append(
            "Product mode switch: "
            f"required_lifecycle={product_switch.get('required_lifecycle', 'unknown')} "
            f"online_hot_switch_supported={str(bool(product_switch.get('online_hot_switch_supported'))).lower()} "
            f"reason={product_switch.get('reason', '')}"
        )
    _append_contract_section(
        lines,
        "Current product acceptance",
        format_product_acceptance_commands(current_payload),
        separator=" | ",
    )
    _append_contract_section(
        lines,
        "Target product acceptance",
        format_product_acceptance_commands(target_payload),
        separator=" | ",
    )
    _append_contract_section(
        lines,
        "Current product semantic overrides",
        format_product_semantic_overrides(current_payload),
    )
    _append_contract_section(
        lines,
        "Target product semantic overrides",
        format_product_semantic_overrides(target_payload),
    )
    _append_contract_section(
        lines,
        "Current topic frames",
        format_runtime_topic_frames(current_payload),
    )
    _append_contract_section(
        lines,
        "Target topic frames",
        format_runtime_topic_frames(target_payload),
    )
    _append_contract_section(
        lines,
        "Current required topic frames",
        join_runtime_items(current_payload.get("required_topic_frame_ids")),
    )
    _append_contract_section(
        lines,
        "Target required topic frames",
        join_runtime_items(target_payload.get("required_topic_frame_ids")),
    )
    _append_contract_section(
        lines,
        "Current data-flow topics",
        join_runtime_items(current_payload.get("runtime_data_flow_topics")),
    )
    _append_contract_section(
        lines,
        "Target data-flow topics",
        join_runtime_items(target_payload.get("runtime_data_flow_topics")),
    )
    _append_contract_section(
        lines,
        "Current data flow",
        format_runtime_flow_stages(current_payload),
    )
    _append_contract_section(
        lines,
        "Target data flow",
        format_runtime_flow_stages(target_payload),
    )
    _append_contract_section(
        lines,
        "Changed fields",
        join_runtime_items(payload.get("changed")),
        separator=",",
    )
    _append_switch_validation(
        lines,
        "Current validation",
        current_validation_payload,
    )
    _append_switch_validation(
        lines,
        "Target validation",
        target_validation_payload,
    )
    return "\n".join(lines)


def format_runtime_contract_manifest(manifest: Mapping[str, Any]) -> str:
    """Return a compact operator-facing view of the canonical runtime contract."""

    lines = [
        f"Runtime contract: {manifest.get('schema_version', 'unknown')}",
        f"Frames: {format_runtime_frames(manifest)}",
        f"Frame links: {format_frame_links(manifest)}",
    ]
    _append_contract_section(
        lines,
        "Real topic frames",
        _format_manifest_real_topic_frames(manifest),
        separator=" ",
    )
    _append_contract_section(
        lines,
        "Real data flow",
        _format_manifest_data_flow(manifest, REAL_RUNTIME_CONTRACT),
    )
    _append_contract_section(
        lines,
        "Data sources",
        _format_manifest_data_sources(manifest),
    )
    _append_contract_section(
        lines,
        "Profile bindings",
        _format_manifest_profile_bindings(manifest),
    )
    _append_contract_section(
        lines,
        "Artifact formats",
        _format_manifest_artifact_formats(manifest),
    )
    _append_contract_section(
        lines,
        "Adapter aliases",
        _format_manifest_adapter_mappings(manifest, "adapter_aliases"),
    )
    _append_contract_section(
        lines,
        "Adapter relays",
        _format_manifest_adapter_mappings(manifest, "adapter_relays"),
    )
    _append_contract_section(
        lines,
        "Stage interfaces",
        _format_manifest_stage_interfaces(manifest),
    )
    _append_contract_section(
        lines,
        "Algorithm interfaces",
        _format_manifest_algorithm_interfaces(manifest),
    )
    return "\n".join(lines)


def format_runtime_spec_payload(payload: Mapping[str, Any]) -> str:
    """Return an operator-facing view of one resolved runtime-spec payload."""

    spec = payload.get("spec")
    spec_payload = spec if isinstance(spec, Mapping) else {}
    validation = payload.get("validation")
    validation_payload = validation if isinstance(validation, Mapping) else {}
    env = payload.get("env")
    env_payload = env if isinstance(env, Mapping) else {}
    ok = validation_payload.get("ok")
    status = "PASS" if ok is True else "FAIL" if ok is False else "UNKNOWN"

    lines = [
        f"Runtime spec: {status}",
        (
            "Profile: "
            f"profile={spec_payload.get('profile', 'unknown')} "
            f"endpoint={spec_payload.get('endpoint', 'unknown')} "
            f"robot_preset={spec_payload.get('robot_preset', 'unknown')}"
        ),
        f"Runtime: {format_runtime_boundary(spec_payload)}",
        f"Algorithms: {format_runtime_algorithms(spec_payload)}",
        f"SLAM: {format_runtime_sources(spec_payload)}",
        f"Product boundary: {format_product_runtime_boundary(spec_payload)}",
        f"Frames: {format_runtime_frames(spec_payload)}",
        f"Frame links: {format_frame_links(spec_payload)}",
    ]
    _append_contract_section(
        lines,
        "Product acceptance",
        format_product_acceptance_commands(spec_payload),
        separator=" | ",
    )
    _append_contract_section(
        lines,
        "Startup gates",
        join_runtime_items(spec_payload.get("startup_gates")),
    )
    _append_contract_section(
        lines,
        "Product semantic overrides",
        format_product_semantic_overrides(spec_payload),
    )
    _append_contract_section(
        lines,
        "Topic frames",
        format_runtime_topic_frames(spec_payload),
        separator=" ",
    )
    _append_contract_section(
        lines,
        "Required topic frames",
        join_runtime_items(spec_payload.get("required_topic_frame_ids")),
    )
    _append_contract_section(
        lines,
        "Runtime data-flow topics",
        join_runtime_items(spec_payload.get("runtime_data_flow_topics")),
    )
    _append_contract_section(
        lines,
        "Data flow",
        format_runtime_flow_stages(spec_payload),
    )
    _append_contract_section(
        lines,
        "Validation blockers",
        " | ".join(str(item) for item in validation_payload.get("blockers", ())),
    )
    _append_contract_section(
        lines,
        "Validation warnings",
        " | ".join(str(item) for item in validation_payload.get("warnings", ())),
    )
    _append_contract_section(
        lines,
        "Runtime env",
        _format_runtime_env(env_payload),
        separator=" ",
    )
    return "\n".join(lines)


def format_runtime_audit_payload(payload: Mapping[str, Any]) -> str:
    """Return an operator-facing view of the offline runtime contract audit."""

    ok = payload.get("ok")
    status = "PASS" if ok is True else "FAIL" if ok is False else "UNKNOWN"
    lines = [
        f"Runtime audit: {status}",
        f"Schema: {payload.get('schema_version', 'unknown')}",
    ]
    _append_contract_section(
        lines,
        "Validation gate",
        _format_validation_gate(payload),
    )
    _append_contract_section(
        lines,
        "Blockers",
        " | ".join(str(item) for item in payload.get("blockers", ())),
    )
    _append_contract_section(
        lines,
        "Checks",
        _format_runtime_audit_checks(payload),
    )
    _append_contract_section(
        lines,
        "Validation gate sequence",
        _format_runtime_audit_acceptance(payload),
    )
    _append_contract_section(
        lines,
        "Validation commands",
        _format_runtime_audit_commands(payload),
    )
    return "\n".join(lines)


def format_saved_map_artifact_gate_payload(payload: Mapping[str, Any]) -> str:
    """Return an operator-facing view of saved-map artifact provenance."""

    ok = payload.get("ok")
    status = "PASS" if ok is True else "FAIL" if ok is False else "UNKNOWN"
    lines = [
        f"Saved map artifact gate: {status}",
        f"Map dir: {payload.get('map_dir', 'unknown')}",
        f"Frame: observed={payload.get('checked_frame_id') or 'unknown'} "
        f"allowed={join_runtime_items(payload.get('checked_allowed_frame_ids'))}",
    ]
    _append_contract_section(
        lines,
        "Validation gate",
        _format_validation_gate(payload),
    )
    _append_contract_section(
        lines,
        "Expected",
        _format_saved_map_expected(payload),
        separator=" ",
    )
    _append_contract_section(
        lines,
        "Required artifacts",
        join_runtime_items(payload.get("checked_required_artifacts")),
        separator=",",
    )
    _append_contract_section(
        lines,
        "Metadata",
        _format_saved_map_metadata(payload),
        separator=" ",
    )
    _append_contract_section(
        lines,
        "Artifacts",
        _format_saved_map_artifacts(payload),
    )
    _append_contract_section(
        lines,
        "Blockers",
        " | ".join(str(item) for item in payload.get("blockers", ())),
    )
    return "\n".join(lines)


def format_real_runtime_evidence_summary(report: Mapping[str, Any]) -> str:
    """Return a concise operator-facing summary of a real runtime evidence report."""

    validation = report.get("runtime_evidence")
    validation_payload = validation if isinstance(validation, Mapping) else {}
    runtime_contract = report.get("runtime_contract")
    runtime_contract_payload = (
        runtime_contract if isinstance(runtime_contract, Mapping) else {}
    )
    motion = report.get("motion")
    motion_payload = motion if isinstance(motion, Mapping) else {}
    outputs = report.get("outputs")
    output_payload = outputs if isinstance(outputs, Mapping) else {}
    hardware = report.get("hardware_boundary")
    hardware_payload = hardware if isinstance(hardware, Mapping) else {}

    ok = validation_payload.get("ok")
    status = "PASS" if ok is True else "FAIL" if ok is False else "NOT_VALIDATED"
    lines = [
        f"Real runtime evidence: {status}",
        (
            "Runtime contract: "
            f"name={runtime_contract_payload.get('name', 'unknown')} "
            f"ok={str(runtime_contract_payload.get('ok')).lower()}"
        ),
        (
            "Motion: "
            f"real_robot_motion={str(report.get('real_robot_motion')).lower()} "
            f"odom_delta_m={motion_payload.get('odom_delta_m', 'unknown')} "
            f"min_motion_m={motion_payload.get('min_motion_m', 'unknown')}"
        ),
        (
            "Paths: "
            f"global={output_payload.get('global_path_count', 'unknown')} "
            f"local={output_payload.get('local_path_count', 'unknown')} "
            f"cmd_vel_nonzero={output_payload.get('nav_cmd_vel_nonzero', 'unknown')}"
        ),
        (
            "Command boundary: "
            f"cmd_vel_sent_to_hardware="
            f"{str(report.get('cmd_vel_sent_to_hardware')).lower()} "
            f"sink={hardware_payload.get('command_sink', 'unknown')}"
        ),
    ]
    _append_contract_section(
        lines,
        "Validation gate",
        _format_validation_gate(validation_payload, report),
    )
    if validation_payload:
        _append_contract_section(
            lines,
            "Blockers",
            " | ".join(str(item) for item in validation_payload.get("blockers", ())),
        )
        _append_contract_section(
            lines,
            "Checked runtime topics",
            " ".join(
                str(topic)
                for topic in validation_payload.get("checked_runtime_topics", ())
            ),
            separator=" ",
        )
        _append_contract_section(
            lines,
            "Checked frame links",
            " ".join(
                str(link)
                for link in validation_payload.get("checked_frame_links", ())
            ),
            separator=" ",
        )
        _append_contract_section(
            lines,
            "Topic frame evidence",
            _format_real_topic_frame_evidence(validation_payload),
        )
        _append_contract_section(
            lines,
            "Frame link evidence",
            _format_real_frame_link_evidence(validation_payload),
        )
        _append_contract_section(
            lines,
            "Stage evidence matrix",
            _format_real_stage_evidence_matrix(validation_payload),
        )
        _append_contract_section(
            lines,
            "Data-flow evidence",
            _format_real_data_flow_evidence(validation_payload),
        )
    else:
        lines.append("Validation: skipped or unavailable")
    return "\n".join(lines)


def format_product_field_check(payload: Mapping[str, Any]) -> str:
    """Return a compact field-readiness summary for operators."""

    map_payload = _get_mapping(payload, "map")
    runtime = _get_mapping(payload, "runtime")
    stage_evidence = _get_mapping(payload, "stage_evidence")
    frontier_preview = _get_mapping(payload, "frontier_preview")
    runtime_switch = _get_mapping(payload, "runtime_switch")
    navigation = _get_mapping(payload, "navigation")
    evidence = _get_mapping(payload, "evidence")
    algorithm_payload = _get_mapping(payload, "algorithm")
    algorithm = _get_mapping(algorithm_payload, "strict_benchmark")
    active_product_profile = (
        algorithm_payload.get("active_product_profile") or "inspection_mvp"
    )
    product_profile = _get_mapping(
        _get_mapping(algorithm_payload, "product_profiles"),
        str(active_product_profile),
    )
    status = payload.get("summary") or ("PASS" if payload.get("ok") else "FAIL")
    mode = payload.get("mode")
    title = {
        "simulation": "LingTu Product Check",
        "non_motion": "LingTu Non-motion Ready",
    }.get(str(mode), "LingTu Field Ready")
    age_s = evidence.get("age_s")
    age = "unknown" if age_s is None else f"{age_s}s"
    missing = join_runtime_items(algorithm.get("missing_or_failed"))
    lines = [
        f"{title}: {status}",
        (
            "Map: "
            f"active={map_payload.get('active', 'unknown')} "
            f"provenance={map_payload.get('provenance', 'UNKNOWN')} "
            f"tomogram={map_payload.get('tomogram', 'UNKNOWN')} "
            f"occupancy={map_payload.get('occupancy', 'UNKNOWN')}"
        ),
        (
            "Runtime: "
            f"gateway={runtime.get('gateway', 'UNKNOWN')} "
            f"readiness={runtime.get('readiness', 'UNKNOWN')} "
            f"localization={runtime.get('localization', 'UNKNOWN')} "
            f"dataflow={runtime.get('dataflow', 'UNKNOWN')} "
            f"stages={runtime.get('stages', 'UNKNOWN')} "
            f"command_boundary={runtime.get('command_boundary', 'UNKNOWN')} "
            f"frontier_preview={runtime.get('frontier_preview', 'UNKNOWN')} "
            f"runtime_switch={runtime.get('runtime_switch', 'UNKNOWN')}"
        ),
        (
            "Stage evidence: "
            f"live={join_runtime_items(stage_evidence.get('live_stages'))} "
            f"not_live={join_runtime_items(stage_evidence.get('not_live_stages'))} "
            f"missing={join_runtime_items(stage_evidence.get('missing_stages'))}"
        ),
        (
            "Frontier preview: "
            f"status={frontier_preview.get('status', 'UNKNOWN')} "
            f"source={frontier_preview.get('candidate_source') or 'unknown'} "
            "command_published="
            f"{str(bool(frontier_preview.get('command_published'))).lower()}"
        ),
        (
            "Runtime switch: "
            f"status={runtime_switch.get('status', 'UNKNOWN')} "
            f"dry_run={str(bool(runtime_switch.get('dry_run'))).lower()} "
            f"motion={str(bool(runtime_switch.get('motion'))).lower()} "
            f"publishes={join_runtime_items(runtime_switch.get('publishes'))} "
            f"from={_get_mapping(runtime_switch, 'from').get('endpoint', 'unknown')} "
            f"to={_get_mapping(runtime_switch, 'to').get('endpoint', 'unknown')}"
        ),
        (
            "Navigation: "
            f"can_send_goal={navigation.get('can_send_goal', 'UNKNOWN')} "
            f"route_preview={navigation.get('route_preview', 'UNKNOWN')} "
            f"local_path={navigation.get('local_path', 'UNKNOWN')} "
            f"cmd_vel_mux={navigation.get('cmd_vel_mux', 'UNKNOWN')}"
        ),
        (
            "Evidence: "
            f"thunder_field={evidence.get('thunder_field', 'UNKNOWN')} "
            f"age={age} mode={evidence.get('mode', payload.get('mode', 'unknown'))}"
        ),
        (
            "Algorithm: "
            f"strict_benchmark={algorithm.get('status', 'UNKNOWN')} "
            f"claim_allowed={str(bool(algorithm.get('claim_allowed'))).lower()} "
            f"missing={missing} "
            f"product_profile={active_product_profile}:"
            f"{product_profile.get('status', 'UNKNOWN')}"
        ),
    ]
    _append_contract_section(
        lines,
        "Blockers",
        " | ".join(str(item) for item in payload.get("blockers", ())),
    )
    _append_contract_section(
        lines,
        "Advisories",
        " | ".join(str(item) for item in payload.get("advisories", ())),
    )
    return "\n".join(lines)


def format_inspection_acceptance(payload: Mapping[str, Any]) -> str:
    """Return a compact inspection-point acceptance summary for operators."""

    status = payload.get("summary") or ("PASS" if payload.get("ok") else "FAIL")
    lines = [
        f"LingTu Inspection Acceptance: {status}",
        (
            "Field ready: "
            f"{payload.get('field_summary', 'UNKNOWN')} "
            f"targets={payload.get('pass_count', 0)}/{payload.get('target_count', 0)} "
            f"gateway={payload.get('gateway_url', 'unknown')}"
        ),
    ]
    motion_safety = _get_mapping(payload, "motion_safety")
    if motion_safety:
        published = _get_mapping(motion_safety, "published")
        lines.append(
            "Motion safety: "
            f"{motion_safety.get('status', 'UNKNOWN')} "
            f"non_motion={motion_safety.get('non_motion')} "
            f"command_published={motion_safety.get('command_published')} "
            f"published="
            f"goal_pose:{published.get('goal_pose', 'unknown')},"
            f"cmd_vel:{published.get('cmd_vel', 'unknown')},"
            f"stop_cmd:{published.get('stop_cmd', 'unknown')}"
        )
    runtime_switch = _get_mapping(payload, "runtime_switch")
    if runtime_switch:
        lines.append(
            "Runtime switch: "
            f"status={runtime_switch.get('status', 'UNKNOWN')} "
            f"dry_run={str(bool(runtime_switch.get('dry_run'))).lower()} "
            f"motion={str(bool(runtime_switch.get('motion'))).lower()} "
            f"publishes={join_runtime_items(runtime_switch.get('publishes'))} "
            f"from={_get_mapping(runtime_switch, 'from').get('endpoint', 'unknown')} "
            f"to={_get_mapping(runtime_switch, 'to').get('endpoint', 'unknown')}"
        )
    for target in payload.get("targets") or []:
        if not isinstance(target, Mapping):
            continue
        reasons = " | ".join(str(item) for item in target.get("reasons", ()))
        suffix = f" reasons={reasons}" if reasons else ""
        lines.append(
            "  - "
            f"{target.get('name', 'target')}: {target.get('status', 'UNKNOWN')} "
            f"preview={target.get('preview_feasible', False)} "
            f"points={target.get('preview_count', 0)} "
            f"planner={target.get('planner') or 'unknown'}"
            f"{suffix}"
        )
    _append_contract_section(
        lines,
        "Blockers",
        " | ".join(str(item) for item in payload.get("blockers", ())),
    )
    _append_contract_section(
        lines,
        "Advisories",
        " | ".join(str(item) for item in payload.get("advisories", ())),
    )
    return "\n".join(lines)


def _get_mapping(payload: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = payload.get(key)
    return value if isinstance(value, Mapping) else {}


def _format_runtime_env(env: Mapping[str, Any]) -> str:
    keys = (
        "LINGTU_ENDPOINT",
        "LINGTU_DATA_SOURCE",
        "LINGTU_MODULE_TRANSPORT",
        "LINGTU_ENDPOINT_TRANSPORT",
        "LINGTU_RUNTIME_CONTRACT",
        "LINGTU_COMMAND_SINK",
        "LINGTU_SIMULATION_ONLY",
        "LINGTU_FASTLIO2_PORTABLE_LIB",
    )
    parts: list[str] = []
    for key in keys:
        if key in env:
            parts.append(f"{key}={env[key]}")
    return " ".join(parts) if parts else "none"


def _format_switch_profile_line(label: str, runtime: Mapping[str, Any]) -> str:
    return (
        f"{label} profile: "
        f"profile={runtime.get('profile', 'unknown')} "
        f"endpoint={runtime.get('endpoint', 'unknown')} "
        f"robot_preset={runtime.get('robot_preset', 'unknown')}"
    )


def _append_switch_validation(
    lines: list[str],
    title: str,
    validation: Mapping[str, Any],
) -> None:
    ok = validation.get("ok")
    status = "PASS" if ok is True else "FAIL" if ok is False else "UNKNOWN"
    lines.append(f"{title}: {status}")
    blockers = validation.get("blockers")
    if not isinstance(blockers, (list, tuple)):
        return
    for blocker in blockers:
        lines.append(f"  - {blocker}")
    warnings = validation.get("warnings")
    if not isinstance(warnings, (list, tuple)):
        return
    for warning in warnings:
        lines.append(f"  warning: {warning}")


def _format_saved_map_expected(payload: Mapping[str, Any]) -> str:
    expected = payload.get("checked_expected")
    if not isinstance(expected, Mapping):
        return "none"
    return (
        f"data_source={expected.get('data_source') or 'any'} "
        f"source_profile={expected.get('source_profile') or 'any'} "
        f"frame_id={expected.get('frame_id') or 'any'}"
    )


def _format_saved_map_metadata(payload: Mapping[str, Any]) -> str:
    metadata = payload.get("metadata")
    metadata_payload = metadata if isinstance(metadata, Mapping) else {}
    validation = payload.get("metadata_validation")
    validation_payload = validation if isinstance(validation, Mapping) else {}
    return (
        f"path={metadata_payload.get('path', 'unknown')} "
        f"exists={_format_bool(metadata_payload.get('exists'))} "
        f"ok={_format_bool(validation_payload.get('ok'))}"
    )


def _format_saved_map_artifacts(payload: Mapping[str, Any]) -> str:
    artifacts = payload.get("artifacts")
    if not isinstance(artifacts, Mapping):
        return "none"
    parts: list[str] = []
    for name, raw_artifact in artifacts.items():
        if not isinstance(raw_artifact, Mapping):
            continue
        parts.append(
            f"{name} exists={_format_bool(raw_artifact.get('exists'))} "
            f"sha256_ok={_format_bool(raw_artifact.get('sha256_ok'))} "
            f"path={raw_artifact.get('path', 'unknown')}"
        )
    return " | ".join(parts) if parts else "none"


def _format_runtime_audit_checks(payload: Mapping[str, Any]) -> str:
    raw_checks = payload.get("checks")
    if not isinstance(raw_checks, Mapping):
        return "none"
    parts: list[str] = []
    for name, raw_check in raw_checks.items():
        if not isinstance(raw_check, Mapping):
            continue
        blockers = raw_check.get("blockers")
        blocker_count = len(blockers) if isinstance(blockers, list) else 0
        parts.append(
            f"{name} ok={_format_bool(raw_check.get('ok'))} blockers={blocker_count}"
        )
    return " | ".join(parts) if parts else "none"


def _format_runtime_audit_acceptance(payload: Mapping[str, Any]) -> str:
    gate_check = _runtime_audit_validation_gate_check(payload)
    acceptance = gate_check.get("acceptance") if isinstance(gate_check, Mapping) else {}
    if not isinstance(acceptance, Mapping):
        return "none"
    ordered = sorted(
        (
            (str(name), raw_gate)
            for name, raw_gate in acceptance.items()
            if isinstance(raw_gate, Mapping)
        ),
        key=lambda item: (
            item[1].get("acceptance_step")
            if isinstance(item[1].get("acceptance_step"), int)
            else 999,
            item[0],
        ),
    )
    parts: list[str] = []
    for name, raw_gate in ordered:
        parts.append(
            f"step={raw_gate.get('acceptance_step', 'unknown')} "
            f"{name} required_when={raw_gate.get('required_when', 'unknown')} "
            f"prior={join_runtime_items(raw_gate.get('requires_prior_gates'))} "
            f"conditional_prior={join_runtime_items(raw_gate.get('conditional_prior_gates'))} "
            f"proves={join_runtime_items(raw_gate.get('proves'))} "
            f"summary_sections={join_runtime_items(raw_gate.get('operator_summary_sections'))}"
        )
    return " | ".join(parts) if parts else "none"


def _format_runtime_audit_commands(payload: Mapping[str, Any]) -> str:
    gate_check = _runtime_audit_validation_gate_check(payload)
    commands = gate_check.get("commands") if isinstance(gate_check, Mapping) else {}
    if not isinstance(commands, Mapping):
        return "none"
    parts = [
        f"{name}={command}"
        for name, command in commands.items()
        if command
    ]
    return " | ".join(parts) if parts else "none"


def _runtime_audit_validation_gate_check(payload: Mapping[str, Any]) -> Mapping[str, Any]:
    raw_checks = payload.get("checks")
    if not isinstance(raw_checks, Mapping):
        return {}
    raw_gate_check = raw_checks.get("runtime_validation_gates")
    return raw_gate_check if isinstance(raw_gate_check, Mapping) else {}


def _append_contract_section(
    lines: list[str],
    title: str,
    value: str,
    *,
    separator: str = " | ",
) -> None:
    lines.append(f"{title}:")
    if not value or value == "none":
        lines.append("  none")
        return
    for item in value.split(separator):
        item = item.strip()
        if item:
            lines.append(f"  {item}")


def _format_override_value(value: Any) -> str:
    if isinstance(value, Mapping):
        return "{" + ",".join(f"{key}:{item}" for key, item in value.items()) + "}"
    if isinstance(value, (list, tuple)):
        return "[" + ",".join(str(item) for item in value) + "]"
    return str(value)


def _format_manifest_real_topic_frames(manifest: Mapping[str, Any]) -> str:
    frame_rules = manifest.get("real_runtime_topic_allowed_frame_ids")
    if not isinstance(frame_rules, Mapping):
        return "none"
    return format_runtime_topic_frames({"topic_allowed_frame_ids": frame_rules})


def _format_manifest_data_flow(
    manifest: Mapping[str, Any],
    runtime_contract: str,
) -> str:
    flows = manifest.get("resolved_runtime_data_flow")
    if not isinstance(flows, Mapping):
        return "none"
    stages = flows.get(runtime_contract)
    if not isinstance(stages, list):
        return "none"
    return format_runtime_flow_stages(
        {
            "resolved_runtime_data_flow": stages,
            "runtime_data_flow_stage_algorithm_interfaces": manifest.get(
                "runtime_data_flow_stage_algorithm_interfaces",
                {},
            ),
        }
    )


def _format_manifest_data_sources(manifest: Mapping[str, Any]) -> str:
    raw_sources = manifest.get("data_sources")
    if not isinstance(raw_sources, Mapping):
        return "none"
    parts: list[str] = []
    for name, raw_source in raw_sources.items():
        if not isinstance(raw_source, Mapping):
            continue
        parts.append(
            f"{name}[{raw_source.get('provider') or 'unknown'}] "
            f"source={join_runtime_items(raw_source.get('source_outputs'))} "
            f"normalized={join_runtime_items(raw_source.get('normalized_outputs'))} "
            f"algorithm_entry={join_runtime_items(raw_source.get('algorithm_entry_outputs'))} "
            f"command={raw_source.get('command_sink') or 'unknown'} "
            f"slam={raw_source.get('slam_source') or 'unknown'} "
            f"mapping={raw_source.get('mapping_source') or 'unknown'} "
            f"lidar_extrinsic={raw_source.get('lidar_extrinsic_profile') or 'none'}"
        )
    return " | ".join(parts) if parts else "none"


def _format_manifest_profile_bindings(manifest: Mapping[str, Any]) -> str:
    raw_bindings = manifest.get("profile_data_sources")
    if not isinstance(raw_bindings, Mapping):
        return "none"
    parts: list[str] = []
    for profile, raw_binding in raw_bindings.items():
        if not isinstance(raw_binding, Mapping):
            continue
        data_source = raw_binding.get("data_source") or "unknown"
        mode = raw_binding.get("mode") or "unknown"
        note = raw_binding.get("note") or ""
        note_suffix = f" note={note}" if note else ""
        parts.append(f"{profile}->{data_source} mode={mode}{note_suffix}")
    return " | ".join(parts) if parts else "none"


def _format_manifest_artifact_formats(manifest: Mapping[str, Any]) -> str:
    raw_artifacts = manifest.get("artifact_formats")
    if not isinstance(raw_artifacts, Mapping):
        return "none"
    parts: list[str] = []
    for name, raw_artifact in raw_artifacts.items():
        if not isinstance(raw_artifact, Mapping):
            continue
        parts.append(
            f"{name} path={raw_artifact.get('path') or 'unknown'} "
            f"type={raw_artifact.get('artifact_type') or 'unknown'} "
            f"frame_role={raw_artifact.get('frame_role') or 'unknown'} "
            f"metadata={join_runtime_items(raw_artifact.get('required_metadata'))}"
        )
    return " | ".join(parts) if parts else "none"


def _format_manifest_adapter_mappings(
    manifest: Mapping[str, Any],
    section_name: str,
) -> str:
    raw_surfaces = manifest.get(section_name)
    if not isinstance(raw_surfaces, Mapping):
        return "none"
    parts: list[str] = []
    for surface, raw_mappings in raw_surfaces.items():
        if not isinstance(raw_mappings, (list, tuple)):
            continue
        mappings: list[str] = []
        for raw_mapping in raw_mappings:
            if not isinstance(raw_mapping, Mapping):
                continue
            source = raw_mapping.get("source") or "unknown"
            target = raw_mapping.get("target") or "unknown"
            msg_format = raw_mapping.get("msg_format") or "unknown"
            mappings.append(f"{source}->{target}({msg_format})")
        if mappings:
            parts.append(f"{surface} {join_runtime_items(mappings)}")
    return " | ".join(parts) if parts else "none"


def _format_manifest_stage_interfaces(manifest: Mapping[str, Any]) -> str:
    stage_interfaces = manifest.get("runtime_data_flow_stage_algorithm_interfaces")
    if not isinstance(stage_interfaces, Mapping):
        return "none"
    parts: list[str] = []
    for stage, interfaces in stage_interfaces.items():
        parts.append(f"{stage}={join_runtime_items(interfaces)}")
    return " | ".join(parts) if parts else "none"


def _format_manifest_algorithm_interfaces(manifest: Mapping[str, Any]) -> str:
    interfaces = manifest.get("algorithm_interfaces")
    if not isinstance(interfaces, Mapping):
        return "none"
    parts: list[str] = []
    for name, raw_interface in interfaces.items():
        if not isinstance(raw_interface, Mapping):
            continue
        inputs = join_runtime_items(raw_interface.get("inputs"))
        outputs = join_runtime_items(raw_interface.get("outputs"))
        owner = raw_interface.get("owner") or "unknown"
        map_dependency = raw_interface.get("map_dependency") or "unknown"
        parts.append(
            f"{name}[{owner}|{map_dependency}] {inputs}->{outputs}"
        )
    return " | ".join(parts) if parts else "none"


def _format_bool(value: Any) -> str:
    if isinstance(value, bool):
        return str(value).lower()
    if value is None:
        return "unknown"
    return str(value).lower()


def _format_validation_gate(*sources: Mapping[str, Any]) -> str:
    raw_gate: Any = None
    for source in sources:
        candidate = source.get("validation_gate")
        if isinstance(candidate, Mapping):
            raw_gate = candidate
            break
    if not isinstance(raw_gate, Mapping):
        return "none"
    return (
        f"step={raw_gate.get('acceptance_step', 'unknown')} "
        f"required_when={raw_gate.get('required_when', 'unknown')} "
        f"prior={join_runtime_items(raw_gate.get('requires_prior_gates'))} "
        f"conditional_prior={join_runtime_items(raw_gate.get('conditional_prior_gates'))} "
        f"proves={join_runtime_items(raw_gate.get('proves'))} "
        f"summary_sections={join_runtime_items(raw_gate.get('operator_summary_sections'))}"
    )


def _format_real_topic_frame_evidence(validation: Mapping[str, Any]) -> str:
    raw_report = validation.get("checked_required_topic_frame_report")
    if not isinstance(raw_report, Mapping):
        return "none"
    ordered_topics = [topic for topic in TOPIC_FRAME_SUMMARY_TOPICS if topic in raw_report]
    ordered_topics.extend(topic for topic in raw_report if topic not in ordered_topics)

    parts: list[str] = []
    for topic in ordered_topics:
        entry = raw_report.get(topic)
        if not isinstance(entry, Mapping):
            continue
        observed = entry.get("observed_frame_id") or "missing"
        default = entry.get("default_frame_id") or "missing"
        allowed = join_runtime_items(entry.get("allowed_frame_ids"))
        parts.append(
            f"{topic} default={default} observed={observed} "
            f"allowed={allowed} ok={_format_bool(entry.get('ok'))}"
        )
    return " | ".join(parts) if parts else "none"


def _format_real_frame_link_evidence(validation: Mapping[str, Any]) -> str:
    raw_report = validation.get("checked_frame_link_evidence")
    if not isinstance(raw_report, Mapping):
        return "none"

    parts: list[str] = []
    for name, entry in raw_report.items():
        if not isinstance(entry, Mapping):
            continue
        expected_parent = entry.get("expected_parent") or "missing"
        expected_child = entry.get("expected_child") or "missing"
        observed_parent = entry.get("observed_parent")
        observed_child = entry.get("observed_child")
        if observed_parent and observed_child:
            observed = f"{observed_parent}->{observed_child}"
        else:
            observed = "missing"
        samples = entry.get("samples")
        sample_text = "unknown" if samples is None else str(samples)
        error = entry.get("error")
        error_suffix = f" error={error}" if error else ""
        parts.append(
            f"{name} expected={expected_parent}->{expected_child} "
            f"observed={observed} samples={sample_text} "
            f"static={_format_bool(entry.get('static'))} "
            f"published={_format_bool(entry.get('published'))} "
            f"ok={_format_bool(entry.get('ok'))}{error_suffix}"
        )
    return " | ".join(parts) if parts else "none"


def _format_real_data_flow_evidence(validation: Mapping[str, Any]) -> str:
    raw_report = validation.get("checked_runtime_data_flow_evidence")
    if not isinstance(raw_report, Mapping):
        return "none"
    raw_order = validation.get("checked_data_flow_stages")
    ordered_stages = [
        stage
        for stage in raw_order or ()
        if isinstance(stage, str) and stage in raw_report
    ]
    ordered_stages.extend(stage for stage in raw_report if stage not in ordered_stages)

    parts: list[str] = []
    for stage in ordered_stages:
        entry = raw_report.get(stage)
        if not isinstance(entry, Mapping):
            continue
        reason = entry.get("reason") or "none"
        fragments = [
            f"{stage}[{entry.get('owner') or 'unknown'}|{entry.get('frame_role') or 'unknown'}]",
            f"ok={_format_bool(entry.get('ok'))}",
        ]
        if entry.get("required") is False:
            fragments.append("required=false")
        for label in (
            "observed_inputs",
            "observed_outputs",
            "missing_inputs",
            "missing_outputs",
            "missing_signals",
        ):
            fragments.append(f"{label}={join_runtime_items(entry.get(label))}")
        fragments.append(f"reason={reason}")
        parts.append(" ".join(fragments))
    return " | ".join(parts) if parts else "none"


def _format_real_stage_evidence_matrix(validation: Mapping[str, Any]) -> str:
    raw_report = validation.get("checked_runtime_data_flow_evidence")
    if not isinstance(raw_report, Mapping):
        return "none"
    raw_order = validation.get("checked_data_flow_stages")
    ordered_stages = [
        stage
        for stage in raw_order or ()
        if isinstance(stage, str) and stage in raw_report
    ]
    ordered_stages.extend(stage for stage in raw_report if stage not in ordered_stages)
    raw_interfaces = validation.get(
        "checked_runtime_data_flow_stage_algorithm_interfaces"
    )
    stage_interfaces = raw_interfaces if isinstance(raw_interfaces, Mapping) else {}

    parts: list[str] = []
    for stage in ordered_stages:
        entry = raw_report.get(stage)
        if not isinstance(entry, Mapping):
            continue
        observed = _stage_observed_tokens(entry)
        missing = _stage_missing_tokens(entry)
        parts.append(
            f"{stage} ok={_format_bool(entry.get('ok'))} "
            f"owner={entry.get('owner') or 'unknown'} "
            f"frame={entry.get('frame_role') or 'unknown'} "
            f"map={entry.get('map_dependency') or 'unknown'} "
            f"interfaces={join_runtime_items(stage_interfaces.get(stage))} "
            f"observed={observed} missing={missing}"
        )
    return " | ".join(parts) if parts else "none"


def _stage_observed_tokens(entry: Mapping[str, Any]) -> str:
    observed_signals = join_runtime_items(entry.get("observed_signals"))
    observed_inputs = join_runtime_items(entry.get("observed_inputs"))
    observed_outputs = join_runtime_items(entry.get("observed_outputs"))
    if observed_signals != "none":
        return observed_signals
    observed = join_runtime_items(
        tuple(
            item
            for item in (
                *(_tuple_for_display(entry.get("observed_inputs"))),
                *(_tuple_for_display(entry.get("observed_outputs"))),
            )
        )
    )
    return observed if observed != "none" else "none"


def _stage_missing_tokens(entry: Mapping[str, Any]) -> str:
    missing_signals = join_runtime_items(entry.get("missing_signals"))
    if missing_signals != "none":
        return missing_signals
    missing = join_runtime_items(
        tuple(
            item
            for item in (
                *(_tuple_for_display(entry.get("missing_inputs"))),
                *(_tuple_for_display(entry.get("missing_outputs"))),
            )
        )
    )
    return missing if missing != "none" else "none"


def _tuple_for_display(value: Any) -> tuple[str, ...]:
    if isinstance(value, str):
        return (value,)
    if isinstance(value, (list, tuple)):
        return tuple(str(item) for item in value if item)
    return ()


format_runtime_frame_links = format_frame_links

"""Runtime switch comparison and safety guards."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Mapping, Sequence

from runtime.profiles.catalog.profile_adapters import RuntimeRunSpec
from runtime.runtime_interface import (
    FRAME_LINKS,
    FRAMES,
    RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES,
    resolved_runtime_data_flow,
    runtime_data_flow_topics,
    runtime_required_topic_frame_ids,
    runtime_topic_allowed_frame_ids,
    runtime_topic_default_frame_ids,
)
from runtime.transport.abc import TransportStrategy


@dataclass(frozen=True)
class RuntimeSwitchValidation:
    """Result of validating runtime feature switches before applying them."""

    ok: bool
    blockers: tuple[str, ...]
    warnings: tuple[str, ...] = ()


def runtime_spec_summary(spec: RuntimeRunSpec) -> dict[str, object]:
    """Return the operator-facing runtime boundary for one resolved profile."""

    validation = validate_runtime_switch(spec)
    profile_semantic_overrides = [
        _profile_semantic_override_summary(item)
        for item in spec.profile_semantic_overrides
    ]
    return {
        "profile": spec.profile,
        "adapter": spec.adapter or "in_process",
        "data_source": spec.data_source,
        "runtime_contract": spec.runtime_contract,
        "module_transport": spec.module_transport,
        "endpoint_transport": spec.endpoint_transport,
        "endpoint_contract": spec.endpoint_contract,
        "route_contract": spec.route_contract,
        "localization_adapter": spec.localization_adapter,
        "global_planner": spec.global_planner,
        "fallback_global_planners": list(spec.fallback_global_planners),
        "planner_latency_budget_ms": spec.planner_latency_budget_ms,
        "plan_safety_policy": spec.plan_safety_policy,
        "autonomy_backends": {str(name): str(value) for name, value in spec.autonomy_backends.items()},
        "simulation_only": spec.simulation_only,
        "command_sink": spec.command_sink,
        "slam_source": spec.slam_source,
        "localization_source": spec.localization_source,
        "mapping_source": spec.mapping_source,
        "lidar_extrinsic_profile": spec.lidar_extrinsic_profile,
        "frames": {str(name): _json_value(value) for name, value in spec.frames.items()},
        "frame_links": {str(name): dict(link) for name, link in spec.frame_links.items()},
        "topic_allowed_frame_ids": {
            topic: _list_value(frames) for topic, frames in spec.topic_allowed_frame_ids.items()
        },
        "topic_default_frame_ids": {str(topic): str(frame) for topic, frame in spec.topic_default_frame_ids.items()},
        "required_topic_frame_ids": _list_value(
            runtime_required_topic_frame_ids(spec.runtime_contract or spec.data_source)
        ),
        "runtime_data_flow_topics": _list_value(runtime_data_flow_topics(spec.data_source)),
        "resolved_runtime_data_flow": [_runtime_data_flow_summary(stage) for stage in spec.resolved_runtime_data_flow],
        "runtime_data_flow_stage_algorithm_interfaces": {
            stage: _list_value(interfaces)
            for stage, interfaces in (spec.runtime_data_flow_stage_algorithm_interfaces.items())
        },
        "launcher": spec.launcher,
        "launcher_args": list(spec.launcher_args),
        "profile_semantic_overrides": profile_semantic_overrides,
        "startup_gates": _startup_gates_for_spec(spec),
        "validation": {
            "ok": validation.ok,
            "blockers": list(validation.blockers),
            "warnings": list(validation.warnings),
        },
    }


def _startup_gates_for_spec(spec: RuntimeRunSpec) -> list[str]:
    gates: list[str] = []
    if spec.global_planner == "octoplanner3d":
        gates.append("map_artifact")
        gates.append("planner_runtime")
    if spec.localization_source not in {"", "none", "stub"}:
        gates.append("localization")
    return gates


def compare_runtime_switch(
    current: RuntimeRunSpec,
    target: RuntimeRunSpec,
) -> dict[str, object]:
    left = runtime_spec_summary(current)
    right = runtime_spec_summary(target)
    changed = sorted(key for key in left if left.get(key) != right.get(key))
    return {
        "from": left,
        "to": right,
        "changed": changed,
    }


def validate_runtime_switch(spec: RuntimeRunSpec) -> RuntimeSwitchValidation:
    blockers: list[str] = []
    warnings = list(_profile_semantic_warnings(spec.profile_semantic_overrides))
    is_hardware_sink = spec.command_sink == "driver"
    expected_simulation_flag = "1" if spec.simulation_only else "0"
    if spec.adapter not in (None, "in_process"):
        if not spec.runtime_contract:
            blockers.append("Profile adapter has no runtime contract")
        elif spec.runtime_contract != spec.data_source:
            blockers.append("runtime contract does not match data source")
    if spec.simulation_only and is_hardware_sink:
        blockers.append("simulation runtime uses hardware command sink")
    if not spec.simulation_only and not is_hardware_sink:
        blockers.append("real runtime does not use hardware command sink")
    if spec.simulation_only and spec.env.get("LINGTU_SIMULATION_ONLY") == "0":
        blockers.append("simulation runtime exports real-mode flag")
    if not spec.simulation_only and spec.env.get("LINGTU_SIMULATION_ONLY") == "1":
        blockers.append("real runtime exports simulation-mode flag")
    if spec.env.get("LINGTU_SIMULATION_ONLY") != expected_simulation_flag:
        blockers.append("env simulation flag does not match run spec")
    if spec.adapter and spec.env.get("LINGTU_PROFILE_ADAPTER") != spec.adapter:
        blockers.append("env Profile adapter does not match run spec")
    if not spec.adapter and "LINGTU_PROFILE_ADAPTER" in spec.env:
        blockers.append("env Profile adapter exists without run spec adapter")
    if spec.env.get("LINGTU_DATA_SOURCE") != spec.data_source:
        blockers.append("env data source does not match run spec")
    if spec.env.get("LINGTU_MODULE_TRANSPORT") != spec.module_transport:
        blockers.append("env module transport does not match run spec")
    if spec.module_transport not in _known_transport_strategies():
        blockers.append("module transport is not a known transport strategy")
    if spec.env.get("LINGTU_ENDPOINT_TRANSPORT") != spec.endpoint_transport:
        blockers.append("env endpoint transport does not match run spec")
    if spec.endpoint_transport not in _known_transport_strategies():
        blockers.append("endpoint transport is not a known transport strategy")
    if spec.endpoint_contract and (spec.env.get("LINGTU_ENDPOINT_CONTRACT") != spec.endpoint_contract):
        blockers.append("env endpoint contract does not match run spec")
    if not spec.endpoint_contract and "LINGTU_ENDPOINT_CONTRACT" in spec.env:
        blockers.append("env endpoint contract exists without run spec contract")
    if spec.route_contract and (spec.env.get("LINGTU_ROUTE_CONTRACT") != spec.route_contract):
        blockers.append("env route contract does not match run spec")
    if not spec.route_contract and "LINGTU_ROUTE_CONTRACT" in spec.env:
        blockers.append("env route contract exists without run spec contract")
    if spec.endpoint_transport == "dds" and not spec.route_contract:
        blockers.append("dds endpoint has no route contract")
    blockers.extend(_route_contract_blockers(spec))
    if spec.localization_adapter and (spec.env.get("LINGTU_LOCALIZATION_ADAPTER") != spec.localization_adapter):
        blockers.append("env localization adapter does not match run spec")
    if not spec.localization_adapter and "LINGTU_LOCALIZATION_ADAPTER" in spec.env:
        blockers.append("env localization adapter exists without run spec adapter")
    if spec.runtime_contract and spec.env.get("LINGTU_RUNTIME_CONTRACT") != spec.runtime_contract:
        blockers.append("env runtime contract does not match run spec")
    if spec.env.get("LINGTU_COMMAND_SINK") != spec.command_sink:
        blockers.append("env command sink does not match run spec")
    if dict(spec.frames) != _expected_frames():
        blockers.append("frames do not match runtime contract")
    if dict(spec.frame_links) != _expected_frame_links():
        blockers.append("frame links do not match runtime contract")
    if _normalized_topic_allowed_frame_ids(spec.topic_allowed_frame_ids) != (
        _expected_topic_allowed_frame_ids(spec.runtime_contract or spec.data_source)
    ):
        blockers.append("topic frame_id contract does not match runtime contract")
    if _normalized_topic_default_frame_ids(spec.topic_default_frame_ids) != (
        _expected_topic_default_frame_ids(spec.runtime_contract or spec.data_source)
    ):
        blockers.append("topic default frame_id contract does not match runtime contract")
    try:
        expected_flow = _expected_runtime_data_flow(spec.data_source)
    except ValueError as exc:
        blockers.append(f"resolved runtime data flow unavailable: {exc}")
    else:
        if _normalized_runtime_data_flow(spec.resolved_runtime_data_flow) != expected_flow:
            blockers.append("resolved runtime data flow does not match data source")
    if (
        _normalized_stage_algorithm_interfaces(spec.runtime_data_flow_stage_algorithm_interfaces)
        != _expected_stage_algorithm_interfaces()
    ):
        blockers.append("runtime data flow stage algorithm interfaces do not match contract")
    return RuntimeSwitchValidation(
        ok=not blockers,
        blockers=tuple(blockers),
        warnings=tuple(warnings),
    )


def _route_contract_blockers(spec: RuntimeRunSpec) -> list[str]:
    if not spec.route_contract:
        return []
    try:
        from runtime.route_contract import load_route_contract, validate_route_contract
    except Exception as exc:  # pragma: no cover - import environment failure
        return [f"route contract validator unavailable: {exc}"]

    try:
        contract = load_route_contract(spec.route_contract)
    except Exception as exc:
        return [f"route contract '{spec.route_contract}' cannot load: {exc}"]

    issues = validate_route_contract(contract)
    blockers = [
        f"route contract '{spec.route_contract}' invalid: "
        f"{issue.code} [{issue.scope}]: {issue.message}"
        for issue in issues
    ]
    route_endpoint_contract = contract.route.endpoint_contract
    if spec.endpoint_contract and route_endpoint_contract != spec.endpoint_contract:
        blockers.append(
            "route contract endpoint contract does not match run spec "
            f"({route_endpoint_contract!r} != {spec.endpoint_contract!r})"
        )
    return blockers


def _expected_frames() -> dict[str, object]:
    return asdict(FRAMES)


def _known_transport_strategies() -> set[str]:
    return {strategy.value for strategy in TransportStrategy}


def _expected_frame_links() -> dict[str, dict[str, object]]:
    return {name: asdict(link) for name, link in FRAME_LINKS.items()}


def _expected_topic_allowed_frame_ids(
    runtime_contract: str | None,
) -> dict[str, tuple[str, ...]]:
    return runtime_topic_allowed_frame_ids(runtime_contract)


def _expected_topic_default_frame_ids(
    runtime_contract: str | None,
) -> dict[str, str]:
    return runtime_topic_default_frame_ids(runtime_contract)


def _expected_runtime_data_flow(data_source: str) -> tuple[dict[str, object], ...]:
    return _normalized_runtime_data_flow(tuple(asdict(stage) for stage in resolved_runtime_data_flow(data_source)))


def _expected_stage_algorithm_interfaces() -> dict[str, tuple[str, ...]]:
    return {
        str(stage): tuple(str(interface) for interface in interfaces)
        for stage, interfaces in RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES.items()
    }


def _normalized_stage_algorithm_interfaces(
    mapping: Mapping[str, Any],
) -> dict[str, tuple[str, ...]]:
    return {str(stage): _tuple_value(interfaces) for stage, interfaces in mapping.items()}


def _normalized_runtime_data_flow(
    stages: Sequence[Mapping[str, Any]],
) -> tuple[dict[str, object], ...]:
    normalized = []
    for stage in stages:
        item = dict(stage)
        item["inputs"] = _tuple_value(item.get("inputs"))
        item["outputs"] = _tuple_value(item.get("outputs"))
        normalized.append(item)
    return tuple(normalized)


def _normalized_topic_allowed_frame_ids(
    frame_rules: Mapping[str, Any],
) -> dict[str, tuple[str, ...]]:
    return {str(topic): _tuple_value(frames) for topic, frames in frame_rules.items()}


def _normalized_topic_default_frame_ids(
    frame_rules: Mapping[str, Any],
) -> dict[str, str]:
    return {str(topic): str(frame) for topic, frame in frame_rules.items()}


def _runtime_data_flow_summary(stage: Mapping[str, Any]) -> dict[str, object]:
    item = dict(stage)
    item["inputs"] = _list_value(item.get("inputs"))
    item["outputs"] = _list_value(item.get("outputs"))
    return item


def _profile_semantic_override_summary(item: Mapping[str, Any]) -> dict[str, object]:
    return {
        "field": str(item.get("field", "")),
        "override_scope": str(item.get("override_scope") or "profile_adapter_override"),
        "profile_value": _json_value(item.get("profile_value")),
        "adapter_value": _json_value(item.get("adapter_value")),
    }


def _profile_semantic_warnings(
    overrides: Sequence[Mapping[str, Any]],
) -> tuple[str, ...]:
    warnings: list[str] = []
    for item in overrides:
        field = item.get("field")
        if not field:
            continue
        profile_value = _warning_value(item.get("profile_value"))
        adapter_value = _warning_value(item.get("adapter_value"))
        warnings.append(f"Profile adapter override: {field} {profile_value} -> {adapter_value}")
    return tuple(warnings)


def _list_value(value: Any) -> list[str]:
    if isinstance(value, str):
        return [value]
    if isinstance(value, Sequence):
        return [str(item) for item in value]
    return []


def _json_value(value: Any) -> object:
    if isinstance(value, tuple):
        return [str(item) for item in value]
    if isinstance(value, list):
        return [str(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    return value


def _warning_value(value: Any) -> str:
    if isinstance(value, Mapping):
        return str(_json_value(value))
    if isinstance(value, Sequence) and not isinstance(value, str):
        return str(_json_value(value))
    return str(value)


def _tuple_value(value: Any) -> tuple[str, ...]:
    if isinstance(value, str):
        return (value,)
    if isinstance(value, Sequence):
        return tuple(str(item) for item in value)
    return ()

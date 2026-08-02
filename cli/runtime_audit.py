"""Offline runtime contract audit helpers for LingTu CLI and scripts."""

from __future__ import annotations

import importlib.util
import re
from dataclasses import asdict
from pathlib import Path
from typing import Any, Mapping

import yaml

from diagnostics.field.gates import (
    RUNTIME_AUDIT_CHECKS,
    runtime_validation_gates,
    validate_runtime_validation_gates,
)
from lingtu.assembly.products import (
    FIELD_PRODUCT_HOST_DEFAULTS,
    FIELD_PRODUCT_NAMES,
    resolve_product_host_config,
)
from runtime.endpoints.dds.contracts import THUNDER_DDS_CONTRACT
from runtime.contracts.product_runtime import resolve_product_spec_contracts
from runtime.graph import load_runtime_graph
from runtime.profiles.binding_policy import ros2_runtime_binding_violations
from runtime.profiles.catalog.host_defaults import HOST_PROFILE_DEFAULTS
from runtime.profiles.profile_adapters import (
    PROFILE_ADAPTERS,
    resolve_runtime_run_spec,
)
from runtime.profiles.resolver import resolve_profile_config
from runtime.routes import robot
from runtime.runtime_interface import (
    DATA_SOURCE_CONTRACTS,
    FRAME_LINKS,
    PRODUCT_DATA_SOURCE_BINDINGS,
    PRODUCT_SCOPED_RUNTIME_DATA_FLOW_STAGE_NAMES,
    PROFILE_DATA_SOURCE_BINDINGS,
    REAL_RUNTIME_CONTRACT,
    TOPICS,
    expand_frame_id_aliases,
    normalize_algorithm_interface_contract,
    normalize_runtime_frames_contract,
    resolved_product_runtime_data_flow,
    runtime_algorithm_interface_contract,
    runtime_contract_manifest,
    runtime_contract_data_source,
    runtime_data_flow_topics,
    runtime_frames_contract,
    runtime_stage_algorithm_interface_contract,
    runtime_topic_allowed_frame_contract,
    runtime_topic_default_frame_contract,
)
from runtime.runtime_switch import validate_runtime_switch


def _resolve_audit_host_config(
    name: str,
    *,
    profile_adapter: str | None = None,
    env: str = "real",
    backend: str | None = None,
) -> dict[str, Any]:
    if name in FIELD_PRODUCT_HOST_DEFAULTS:
        if profile_adapter is not None:
            raise ValueError("Products are resolved by Env, not a Profile adapter")
        env_config = {"backend": backend} if backend is not None else None
        return resolve_product_host_config(name, env, env_config=env_config)
    return resolve_profile_config(name, profile_adapter=profile_adapter)

REPO_ROOT = Path(__file__).resolve().parents[1]
ROS_FRAME_CONTRACT_DOC = REPO_ROOT / "docs" / "architecture" / "ros_frame_contract.md"
ROS_FREE_DEFAULT_PROFILES = frozenset(
    (
        *FIELD_PRODUCT_NAMES,
        "stub",
        "dev",
        "sim_nav",
        "sim",
        "portable_mujoco",
    )
)
SOURCE_FRAME_CONTRACT_ROOTS = (
    "src/drivers/real/lidar/module.py",
    "src/drivers/sim",
    "src/drivers/real/thunder/han_dog_module.py",
    "src/localization",
    "src/nav",
    "src/nav/local",
    "src/runtime/msgs",
    "src/lingtu/assembly",
    "src/gateway/schemas.py",
    "src/gateway/services/control_commands.py",
    "src/gateway/services/goal_builder.py",
    "src/gateway/services/runtime_status.py",
    "src/gateway/services/telemetry_normalizers.py",
    "src/gateway/services/readiness.py",
    "src/gateway/routes/diagnostics.py",
    "src/decision/modules/semantic_planner.py",
    "src/decision/modules/visual_servo.py",
    "src/decision/goals",
    "src/decision/tasks",
    "src/decision/frontiers",
    "src/perception/perception_module.py",
    "src/perception/tracking/instance_tracker.py",
    "src/runtime/profiles/catalog/host_defaults.py",
    "sim/scripts/mujoco/live_gate.py",
    "sim/scripts/saved_map_relocalize_runtime_gate.py",
)
SOURCE_TOPIC_CONTRACT_ROOTS = (
    *SOURCE_FRAME_CONTRACT_ROOTS,
    "scripts/monitor",
)
FORBIDDEN_SOURCE_FRAME_PATTERNS = (
    (
        "direct_FRAMES_runtime_frames",
        re.compile(r"\bFRAMES\.(?:map|odom|body|lidar|real_lidar|world|simulator_world)\b"),
    ),
    (
        "hardcoded_frame_id_assignment",
        re.compile(
            r"\b(?:header\.)?_?(?:frame_id|child_frame_id)\s*=\s*['\"]/?"
            r"(?:map|odom|body|base_link|lidar_link|livox_frame)['\"]"
        ),
    ),
    (
        "hardcoded_frame_config_literal",
        re.compile(
            r"(?:['\"](?:planning_frame_id|goal_frame_id|odom_frame_id|"
            r"costmap_frame_id|occupancy_frame_id|frame_id|child_frame_id)['\"]\s*:|"
            r"\b_?(?:planning_frame_id|goal_frame_id|odom_frame_id|"
            r"costmap_frame_id|occupancy_frame_id)\s*=)\s*['\"]/?"
            r"(?:map|odom|body|base_link|lidar_link|livox_frame)['\"]"
        ),
    ),
    (
        "hardcoded_frame_field_default",
        re.compile(
            r"\b_?(?:planning_frame_id|goal_frame_id|odom_frame_id|"
            r"costmap_frame_id|occupancy_frame_id|frame_id|child_frame_id)\s*:"
            r"[^=\n]+=\s*['\"]/?"
            r"(?:map|odom|body|base_link|lidar_link|livox_frame)['\"]"
        ),
    ),
    (
        "hardcoded_frame_default_argument",
        re.compile(
            r"\bgetattr\([^,\n]+,\s*['\"](?:planning_frame_id|goal_frame_id|"
            r"odom_frame_id|costmap_frame_id|occupancy_frame_id|frame_id|"
            r"child_frame_id|frame)['\"]"
            r"\s*,\s*['\"]/?(?:map|odom|body|base_link|lidar_link|livox_frame)['\"]"
        ),
    ),
    (
        "hardcoded_frame_or_default",
        re.compile(
            r"\b_?(?:planning_frame_id|goal_frame_id|odom_frame_id|costmap_frame_id|"
            r"occupancy_frame_id|frame_id|child_frame_id|frame)\b.*\bor\s*['\"]/?"
            r"(?:map|odom|body|base_link|lidar_link|livox_frame)['\"]"
        ),
    ),
)
FORBIDDEN_SOURCE_TOPIC_PATTERNS = (
    (
        "hardcoded_canonical_runtime_topic",
        re.compile(
            r"(?:[fFrR]{0,2})?['\"][^'\"]*(?<![A-Za-z0-9_.-])/(?:nav|exploration)/"
            r"[A-Za-z0-9_./-]*[^'\"]*['\"]"
        ),
    ),
)


def _read_yaml(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML object")
    return data


def _as_tuple_stage(stage: Mapping[str, Any]) -> dict[str, Any]:
    return {
        **stage,
        "inputs": tuple(stage.get("inputs") or ()),
        "outputs": tuple(stage.get("outputs") or ()),
        "consumers": tuple(stage.get("consumers") or ()),
    }


def _as_tuple_stages(stages: list[Mapping[str, Any]]) -> list[dict[str, Any]]:
    return [_as_tuple_stage(stage) for stage in stages]


def _as_tuple_mapping(mapping: Mapping[str, Any] | None) -> dict[str, tuple[str, ...]]:
    if not isinstance(mapping, Mapping):
        return {}
    return {str(key): tuple(str(item) for item in (value or ())) for key, value in mapping.items()}


def _as_str_mapping(mapping: Mapping[str, Any] | None) -> dict[str, str]:
    if not isinstance(mapping, Mapping):
        return {}
    return {str(key): str(value) for key, value in mapping.items()}


def _normalized_message_formats(
    formats: Mapping[str, Any] | None,
) -> dict[str, dict[str, Any]]:
    """Normalize the YAML/runtime payload fields that must stay identical."""

    if not isinstance(formats, Mapping):
        return {}
    normalized: dict[str, dict[str, Any]] = {}
    for name, spec in formats.items():
        entry = spec if isinstance(spec, Mapping) else {}
        normalized[str(name)] = {
            "ros_type": str(entry.get("ros_type") or ""),
            "required_fields": tuple(
                str(field) for field in (entry.get("required_fields") or ())
            ),
        }
    return normalized


def _normalized_adapter_aliases(
    aliases_by_surface: Mapping[str, Any] | None,
) -> dict[str, list[dict[str, str]]]:
    if not isinstance(aliases_by_surface, Mapping):
        return {}
    normalized: dict[str, list[dict[str, str]]] = {}
    for surface, aliases in aliases_by_surface.items():
        entries: list[dict[str, str]] = []
        if not isinstance(aliases, (list, tuple)):
            normalized[str(surface)] = entries
            continue
        for alias in aliases:
            if not isinstance(alias, Mapping):
                continue
            entries.append(
                {
                    "source": str(alias.get("source") or ""),
                    "target": str(alias.get("target") or ""),
                    "msg_format": str(alias.get("msg_format") or ""),
                    "scope": str(alias.get("scope") or ""),
                }
            )
        normalized[str(surface)] = entries
    return normalized


def _check_yaml_contract(
    contract: Mapping[str, Any],
    manifest: Mapping[str, Any],
) -> dict[str, Any]:
    blockers: list[str] = []

    tf = contract.get("tf")
    if not isinstance(tf, Mapping):
        blockers.append("topic_contract.yaml tf section missing")
    else:
        frame_map = {
            "map_frame": "map",
            "odom_frame": "odom",
            "body_frame": "body",
            "lidar_frame": "lidar",
            "axis_convention": "axis_convention",
        }
        for yaml_key, manifest_key in frame_map.items():
            if tf.get(yaml_key) != manifest["frames"][manifest_key]:
                blockers.append(f"tf.{yaml_key} does not mirror runtime frames")
        if tf.get("links") != manifest["frame_links"]:
            blockers.append("tf.links does not mirror runtime frame_links")

    if tuple(contract.get("required_nav_topics") or ()) != tuple(manifest["core_required_topics"]):
        blockers.append("required_nav_topics does not mirror core_required_topics")

    if _as_tuple_stages(contract.get("runtime_data_flow") or []) != list(manifest["runtime_data_flow"]):
        blockers.append("runtime_data_flow does not mirror runtime contract")

    yaml_resolved = {
        name: _as_tuple_stages(stages) for name, stages in (contract.get("resolved_runtime_data_flow") or {}).items()
    }
    if yaml_resolved != manifest["resolved_runtime_data_flow"]:
        blockers.append(
            "resolved_runtime_data_flow does not mirror runtime contract",
        )

    if _as_tuple_mapping(contract.get("topic_allowed_frame_ids")) != (
        _as_tuple_mapping(manifest.get("topic_allowed_frame_ids"))
    ):
        blockers.append(
            "topic_allowed_frame_ids does not mirror runtime contract",
        )

    if _as_str_mapping(contract.get("topic_default_frame_ids")) != manifest["topic_default_frame_ids"]:
        blockers.append(
            "topic_default_frame_ids does not mirror runtime contract",
        )

    if _as_tuple_mapping(contract.get("real_runtime_topic_allowed_frame_ids")) != (
        _as_tuple_mapping(manifest.get("real_runtime_topic_allowed_frame_ids"))
    ):
        blockers.append(
            "real_runtime_topic_allowed_frame_ids does not mirror runtime contract",
        )

    if (
        _as_str_mapping(contract.get("real_runtime_topic_default_frame_ids"))
        != manifest["real_runtime_topic_default_frame_ids"]
    ):
        blockers.append(
            "real_runtime_topic_default_frame_ids does not mirror runtime contract",
        )

    if tuple(contract.get("real_runtime_required_topic_frame_ids") or ()) != tuple(
        manifest["real_runtime_required_topic_frame_ids"]
    ):
        blockers.append(
            "real_runtime_required_topic_frame_ids does not mirror runtime contract",
        )

    if tuple(contract.get("real_runtime_required_endpoint_input_topics") or ()) != tuple(
        manifest["real_runtime_required_endpoint_input_topics"]
    ):
        blockers.append(
            "real_runtime_required_endpoint_input_topics does not mirror runtime contract",
        )

    if _as_tuple_mapping(contract.get("runtime_data_flow_topics")) != manifest["runtime_data_flow_topics"]:
        blockers.append(
            "runtime_data_flow_topics does not mirror runtime contract",
        )

    if _as_tuple_mapping(contract.get("runtime_data_flow_stage_algorithm_interfaces")) != _as_tuple_mapping(
        manifest.get("runtime_data_flow_stage_algorithm_interfaces")
    ):
        blockers.append(
            "runtime_data_flow_stage_algorithm_interfaces does not mirror runtime contract",
        )

    if _as_tuple_mapping(contract.get("topic_formats")) != manifest["topic_formats"]:
        blockers.append("topic_formats does not mirror runtime contract")

    if _normalized_message_formats(contract.get("data_formats")) != (
        _normalized_message_formats(manifest.get("message_formats"))
    ):
        blockers.append("data_formats does not mirror runtime message_formats")

    if _as_tuple_mapping(contract.get("topic_ros_types")) != manifest["topic_ros_types"]:
        blockers.append("topic_ros_types does not mirror runtime contract")

    if _normalized_adapter_aliases(contract.get("adapter_aliases")) != (
        _normalized_adapter_aliases(manifest.get("adapter_aliases"))
    ):
        blockers.append("adapter_aliases does not mirror runtime contract")

    if contract.get("data_sources") is None:
        blockers.append("data_sources section missing")
    else:
        yaml_sources = set(contract["data_sources"])
        manifest_sources = set(manifest["data_sources"])
        if yaml_sources != manifest_sources:
            blockers.append("data_sources keys do not mirror runtime contract")

    if contract.get("profile_data_sources") is None:
        blockers.append("profile_data_sources section missing")
    else:
        yaml_profiles = set(contract["profile_data_sources"])
        manifest_profiles = set(manifest["profile_data_sources"])
        if yaml_profiles != manifest_profiles:
            blockers.append("profile_data_sources keys do not mirror runtime contract")

    if contract.get("product_data_sources") is None:
        blockers.append("product_data_sources section missing")
    else:
        yaml_products = set(contract["product_data_sources"])
        manifest_products = set(manifest["product_data_sources"])
        if yaml_products != manifest_products:
            blockers.append("product_data_sources keys do not mirror runtime contract")

    return {
        "ok": not blockers,
        "blockers": blockers,
    }


def _runtime_tokens(stages: list[Mapping[str, Any]]) -> tuple[str, ...]:
    tokens: list[str] = []
    for stage in stages:
        tokens.extend(str(token) for token in stage.get("inputs") or ())
        tokens.extend(str(token) for token in stage.get("outputs") or ())
    return tuple(tokens)


def _dedupe_tokens(tokens: tuple[Any, ...]) -> tuple[str, ...]:
    return tuple(dict.fromkeys(str(token) for token in tokens if token))


def _contains_unresolved_placeholder(token: str) -> bool:
    return token.startswith("source:data_source.") or token.startswith("sink:data_source.")


def _format_is_declared(format_name: str, declared_formats: set[str]) -> bool:
    return format_name in declared_formats or format_name == "service" or "/msg/" in format_name


def _topic_formats_are_metadata_only(
    topic: str,
    *,
    topic_format_map: Mapping[str, Any],
    message_format_map: Mapping[str, Any],
) -> bool:
    format_names = tuple(str(name) for name in (topic_format_map.get(topic) or ()))
    if not format_names:
        return False
    return all(
        isinstance(message_format_map.get(format_name), Mapping)
        and str(message_format_map[format_name].get("frame_role") or "").strip()
        == "metadata"
        for format_name in format_names
    )


def _is_numeric(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _normalized_runtime_frame(value: Any) -> str:
    return str(value or "").strip().lstrip("/")


def _fixed_runtime_frame_ids() -> set[str]:
    frames = runtime_frames_contract()
    return {
        str(frames.get("map") or ""),
        str(frames.get("odom") or ""),
        str(frames.get("simulator_world") or ""),
    } - {""}


def _append_profile_adapter_frame_override_blockers(
    blockers: list[str],
    *,
    adapter_name: str,
    section_name: str,
    overrides: Mapping[str, Any],
    fixed_frames: set[str],
) -> None:
    fixed_frame_keys = {
        "planning_frame_id",
        "goal_frame_id",
        "occupancy_frame_id",
        "costmap_frame_id",
    }
    for key in sorted(fixed_frame_keys & set(overrides)):
        frame_id = _normalized_runtime_frame(overrides.get(key))
        if frame_id not in fixed_frames:
            blockers.append(
                f"Profile adapter {adapter_name} {section_name} {key} {frame_id} "
                "is not a fixed runtime frame"
            )


def _append_profile_adapter_topic_override_blockers(
    blockers: list[str],
    *,
    adapter_name: str,
    section_name: str,
    overrides: Mapping[str, Any],
    topic_formats: set[str],
) -> None:
    for key in sorted(str(name) for name in overrides if str(name).endswith("_topic")):
        topic = str(overrides.get(key) or "").strip()
        if not topic.startswith("/"):
            blockers.append(
                f"Profile adapter {adapter_name} {section_name} {key} "
                f"{topic or '<empty>'} is not an absolute runtime topic"
            )
            continue
        if topic not in topic_formats:
            blockers.append(f"Profile adapter {adapter_name} {section_name} {key} {topic} has no topic format")


def _append_profile_adapter_file_override_blockers(
    blockers: list[str],
    *,
    adapter_name: str,
    section_name: str,
    overrides: Mapping[str, Any],
) -> None:
    for key in ("planner_map", "map_path", "octomap"):
        path_value = str(overrides.get(key) or "").strip()
        if not path_value:
            continue
        path = Path(path_value)
        resolved_path = path if path.is_absolute() else REPO_ROOT / path
        if not resolved_path.is_file():
            blockers.append(f"Profile adapter {adapter_name} {section_name} {key} path missing: {path_value}")


def _iter_env_implementations(
    graph: Any,
) -> tuple[tuple[str, str | None, Mapping[str, Any]], ...]:
    implementations: list[tuple[str, str | None, Mapping[str, Any]]] = []
    for env_name, env_data in sorted(graph.envs.items()):
        backends = env_data.get("backends")
        if isinstance(backends, Mapping):
            for backend_name, backend_data in sorted(backends.items()):
                if isinstance(backend_data, Mapping):
                    implementations.append((env_name, str(backend_name), backend_data))
            continue
        implementations.append((env_name, None, env_data))
    return tuple(implementations)


def _product_env_matrix(graph: Any | None = None) -> dict[str, list[str]]:
    runtime_graph = graph or load_runtime_graph()
    matrix = {product: [] for product in FIELD_PRODUCT_NAMES}
    for env_name, backend_name, implementation in _iter_env_implementations(runtime_graph):
        label = f"{env_name}:{backend_name}" if backend_name is not None else env_name
        supported = implementation.get("supported_products")
        if not isinstance(supported, (list, tuple)):
            continue
        for product in supported:
            product_name = str(product)
            if product_name in matrix:
                matrix[product_name].append(label)
    return matrix


def _append_product_env_matrix_blockers(blockers: list[str]) -> None:
    for product, implementations in _product_env_matrix().items():
        if not implementations:
            blockers.append(f"Product {product} has no Env implementation")


def _append_real_env_product_topic_closure_blockers(
    blockers: list[str],
) -> dict[str, list[str]]:
    graph = load_runtime_graph()
    real_env = graph.envs.get("real")
    if not isinstance(real_env, Mapping):
        blockers.append("real Env missing from runtime graph")
        return {}
    if str(real_env.get("process_control") or "") != "systemd":
        blockers.append("real Env process_control is not systemd")
        return {}
    supported_products = {str(product) for product in (real_env.get("supported_products") or ())}
    route_topics = set(robot().routes)
    endpoint_topics = set(THUNDER_DDS_CONTRACT.topics)
    checked_topics: dict[str, list[str]] = {}
    for profile in sorted(supported_products & set(graph.products)):
        product = graph.products[profile]
        try:
            required_topics = sorted(
                resolve_product_spec_contracts(profile, product).topics
            )
        except ValueError as exc:
            blockers.append(f"real Env Product {profile} contract is invalid: {exc}")
            checked_topics[profile] = []
            continue
        checked_topics[profile] = required_topics
        for topic in required_topics:
            if topic not in route_topics:
                blockers.append(
                    f"real Env Product {profile} required topic {topic} "
                    "missing from robot route layer"
                )
            if topic not in endpoint_topics:
                blockers.append(
                    f"real Env Product {profile} required topic {topic} "
                    f"missing from typed endpoint layer {THUNDER_DDS_CONTRACT.name}"
                )
    return checked_topics


def _append_product_runtime_data_flow_blockers(
    blockers: list[str],
    *,
    products: Mapping[str, Mapping[str, Any]],
) -> dict[str, list[str]]:
    """Require Product-scoped stages to have a complete Product topic closure."""

    checked_stages: dict[str, list[str]] = {}
    bound_stage_names: set[str] = set()
    for product_name, product in sorted(products.items()):
        try:
            required_topics = frozenset(
                resolve_product_spec_contracts(product_name, product).topics
            )
        except ValueError as exc:
            blockers.append(f"product {product_name} contract is invalid: {exc}")
            checked_stages[str(product_name)] = []
            continue
        binding = PRODUCT_DATA_SOURCE_BINDINGS.get(product_name)
        if binding is None:
            blockers.append(
                f"product {product_name} has no declared data-source binding"
            )
            checked_stages[str(product_name)] = []
            continue
        product_stages = resolved_product_runtime_data_flow(
            binding.data_source,
            required_topics,
        )
        scoped_stages = tuple(
            stage
            for stage in product_stages
            if stage.name in PRODUCT_SCOPED_RUNTIME_DATA_FLOW_STAGE_NAMES
        )
        checked_stages[str(product_name)] = [stage.name for stage in scoped_stages]
        for stage in scoped_stages:
            bound_stage_names.add(stage.name)
            stage_topics = {
                token
                for token in (*stage.inputs, *stage.outputs)
                if token.startswith("/")
            }
            missing_topics = sorted(stage_topics - required_topics)
            if missing_topics:
                blockers.append(
                    f"product {product_name} runtime stage {stage.name} topics missing from "
                    "required_topics: " + ", ".join(missing_topics)
                )

    for stage_name in sorted(
        PRODUCT_SCOPED_RUNTIME_DATA_FLOW_STAGE_NAMES - bound_stage_names
    ):
        blockers.append(
            f"product-scoped runtime stage {stage_name} is not bound to any Product"
        )
    return checked_stages


def _append_product_planner_backend_blockers(blockers: list[str]) -> None:
    for product in FIELD_PRODUCT_NAMES:
        if "planner_backend" in FIELD_PRODUCT_HOST_DEFAULTS.get(product, {}):
            blockers.append(f"Field Product {product} must use planner, not planner_backend")

    graph = load_runtime_graph()
    for env_name, backend_name, implementation in _iter_env_implementations(graph):
        host_config = implementation.get("host_config")
        if not isinstance(host_config, Mapping) or "planner_backend" not in host_config:
            continue
        label = f"{env_name}:{backend_name}" if backend_name is not None else env_name
        blockers.append(
            f"Env implementation {label} host_config must not set planner_backend for Products"
        )


def _append_external_profile_metadata_blockers(
    blockers: list[str],
    *,
    profile_name: str,
    profile_data: Mapping[str, Any],
) -> None:
    launcher = str(profile_data.get("_external_launcher") or "").strip()
    runtime_contract = str(profile_data.get("_runtime_contract") or "").strip()
    if not launcher and not runtime_contract:
        return

    if not launcher:
        blockers.append(f"profile {profile_name} external launcher missing")
    elif not (REPO_ROOT / launcher).is_file():
        blockers.append(f"profile {profile_name} external launcher missing: {launcher}")

    if not runtime_contract:
        blockers.append(f"profile {profile_name} external runtime contract missing")
        return
    if runtime_contract not in DATA_SOURCE_CONTRACTS:
        blockers.append(f"profile {profile_name} external runtime contract unknown: {runtime_contract}")
        return

    adapter_matches = [
        (adapter_name, adapter)
        for adapter_name, adapter in PROFILE_ADAPTERS.items()
        if adapter.runtime_contract == runtime_contract
    ]
    if not adapter_matches:
        blockers.append(f"profile {profile_name} external runtime contract has no Profile adapter: {runtime_contract}")
        return
    if len(adapter_matches) > 1:
        blockers.append(
            f"profile {profile_name} external runtime contract has multiple Profile adapters: {runtime_contract}"
        )
        return

    adapter_name, adapter = adapter_matches[0]
    if profile_name not in adapter.supported_profiles:
        blockers.append(
            f"profile {profile_name} external runtime contract adapter {adapter_name} does not support profile"
        )
    if launcher and adapter.external_launcher != launcher:
        blockers.append(f"profile {profile_name} external launcher does not match adapter {adapter_name}")
    for field_name, action_name, action_table in (
        ("_external_default_args", "default", adapter.default_actions),
        ("_external_record_args", "record", adapter.record_actions),
    ):
        if field_name not in profile_data:
            continue
        explicit_args = profile_data.get(field_name)
        if not isinstance(explicit_args, (list, tuple)):
            blockers.append(f"profile {profile_name} external {action_name} args are not a list")
            continue
        if profile_name not in action_table:
            blockers.append(
                f"profile {profile_name} external {action_name} args have no adapter action on {adapter_name}"
            )
            continue
        if tuple(str(item) for item in explicit_args) != tuple(str(item) for item in action_table[profile_name]):
            blockers.append(f"profile {profile_name} external {action_name} args do not match adapter {adapter_name}")


def _append_selection_binding_spec_blockers(
    blockers: list[str],
    *,
    binding: Any,
    spec_label: str,
    spec: Any,
) -> None:
    if spec.data_source != binding.data_source:
        blockers.append(f"{spec_label} data_source drifted from its selection binding")
    if spec.runtime_contract and spec.runtime_contract != binding.data_source:
        blockers.append(f"{spec_label} runtime contract drifted from its selection binding")


def _append_ros2_runtime_binding_blockers(
    blockers: list[str],
    *,
    spec_label: str,
    config: Mapping[str, Any],
) -> None:
    blockers.extend(f"{spec_label}: {violation}" for violation in _ros2_runtime_binding_violations_for_config(config))


def _ros2_runtime_binding_violations_for_config(
    config: Mapping[str, Any],
) -> list[str]:
    enable_native = bool(config.get("enable_native", False))
    return ros2_runtime_binding_violations(
        config,
        enable_native=enable_native,
    )


def _enforces_ros2_runtime_binding_audit(
    profile: str,
    adapter: Any | None = None,
) -> bool:
    """Return True for runtime specs that must stay free of ROS2 bindings."""

    if adapter is not None:
        return True
    return profile in ROS_FREE_DEFAULT_PROFILES or _profile_has_external_contract(profile)


def _ros2_runtime_binding_skip_reason(
    profile: str,
    adapter: Any | None = None,
) -> str:
    """Return why a runtime spec is outside no-ROS acceptance."""

    return "outside_no_ros_acceptance_set"


def _profile_has_external_contract(profile: str) -> bool:
    profile_data = HOST_PROFILE_DEFAULTS.get(profile, {})
    return bool(profile_data.get("_external_launcher") or profile_data.get("_runtime_contract"))


def _append_artifact_reference_blockers(
    blockers: list[str],
    *,
    label: str,
    tokens: tuple[str, ...],
    declared_artifacts: set[str],
) -> None:
    for token in tokens:
        if not token.startswith("artifact:"):
            continue
        artifact_name = token.split(":", 1)[1]
        if artifact_name not in declared_artifacts:
            blockers.append(f"{label} references undeclared artifact {artifact_name}")


def _stage_map(stages: list[Mapping[str, Any]]) -> dict[str, Mapping[str, Any]]:
    return {str(stage.get("name")): stage for stage in stages if stage.get("name")}


def _required_mapping_section(
    manifest: Mapping[str, Any],
    section_name: str,
    blockers: list[str],
) -> Mapping[str, Any]:
    section = manifest.get(section_name)
    if not isinstance(section, Mapping) or not section:
        blockers.append(f"{section_name} section missing")
        return {}
    return section


def _append_adapter_mapping_blockers(
    blockers: list[str],
    *,
    section_label: str,
    aliases_by_surface: Mapping[str, Any],
    topic_formats: set[str],
    declared_formats: set[str],
) -> None:
    for surface_name, aliases in aliases_by_surface.items():
        label = f"{section_label} {surface_name}"
        if not isinstance(aliases, (list, tuple)):
            blockers.append(f"{label} entries are not a list")
            continue
        for alias in aliases:
            if not isinstance(alias, Mapping):
                blockers.append(f"{label} entry is not a mapping")
                continue
            msg_format = str(alias.get("msg_format") or "")
            if not msg_format:
                blockers.append(f"{label} msg_format missing")
            elif not _format_is_declared(msg_format, declared_formats):
                blockers.append(f"{label} references undeclared format {msg_format}")

            for field in ("source", "target"):
                topic = str(alias.get(field) or "")
                if topic.startswith("/") and topic not in topic_formats:
                    blockers.append(f"{label} {field} {topic} has no topic format")


def _check_runtime_contract_integrity(manifest: Mapping[str, Any]) -> dict[str, Any]:
    blockers: list[str] = []
    stage_metadata_fields = ("owner", "frame_role", "map_dependency")

    template_stages = list(manifest.get("runtime_data_flow") or [])
    template_stage_names = [str(stage.get("name")) for stage in template_stages]
    if len(template_stage_names) != len(set(template_stage_names)):
        blockers.append("runtime_data_flow stage names are not unique")
    template_stage_by_name: dict[str, Mapping[str, Any]] = {}
    for stage in template_stages:
        stage_name = str(stage.get("name") or "")
        if not stage_name:
            blockers.append("runtime_data_flow stage name missing")
            continue
        template_stage_by_name[stage_name] = stage
        for field in stage_metadata_fields:
            if not str(stage.get(field) or "").strip():
                blockers.append(f"runtime_data_flow {stage_name} {field} missing")

    data_sources = _required_mapping_section(manifest, "data_sources", blockers)
    resolved_flows = _required_mapping_section(
        manifest,
        "resolved_runtime_data_flow",
        blockers,
    )
    if set(resolved_flows) != set(data_sources):
        blockers.append("resolved_runtime_data_flow keys do not match data_sources")
    runtime_flow_topics = _required_mapping_section(
        manifest,
        "runtime_data_flow_topics",
        blockers,
    )
    if set(runtime_flow_topics) != set(data_sources):
        blockers.append("runtime_data_flow_topics keys do not match data_sources")

    frames = _required_mapping_section(manifest, "frames", blockers)
    topic_format_map = _required_mapping_section(manifest, "topic_formats", blockers)
    topic_formats = set(topic_format_map)
    topic_ros_type_map = _required_mapping_section(
        manifest,
        "topic_ros_types",
        blockers,
    )
    topic_allowed_frames = _required_mapping_section(
        manifest,
        "topic_allowed_frame_ids",
        blockers,
    )
    topic_default_frames = _required_mapping_section(
        manifest,
        "topic_default_frame_ids",
        blockers,
    )
    real_topic_allowed_frames = _required_mapping_section(
        manifest,
        "real_runtime_topic_allowed_frame_ids",
        blockers,
    )
    real_topic_default_frames = _required_mapping_section(
        manifest,
        "real_runtime_topic_default_frame_ids",
        blockers,
    )
    real_required_frame_topics = tuple(
        str(topic) for topic in (manifest.get("real_runtime_required_topic_frame_ids") or ())
    )
    if not real_required_frame_topics:
        blockers.append("real_runtime_required_topic_frame_ids section missing or empty")
    real_endpoint_input_topics = tuple(
        str(topic) for topic in (manifest.get("real_runtime_required_endpoint_input_topics") or ())
    )
    if not real_endpoint_input_topics:
        blockers.append("real_runtime_required_endpoint_input_topics section missing or empty")
    message_format_map = _required_mapping_section(
        manifest,
        "message_formats",
        blockers,
    )
    declared_formats = set(message_format_map)
    if set(topic_ros_type_map) != topic_formats:
        blockers.append("topic_ros_types keys do not match topic_formats")
    for topic, ros_types in sorted(topic_ros_type_map.items()):
        if not isinstance(ros_types, (list, tuple)) or not ros_types:
            blockers.append(f"topic_ros_types {topic} must be non-empty")
            continue
        for ros_type in ros_types:
            ros_type = str(ros_type)
            if (
                ros_type == "service"
                or ros_type == "application/json"
                or "/msg/" in ros_type
                or ros_type.startswith("lingtu.dds.")
            ):
                continue
            blockers.append(f"topic_ros_types {topic} references non-ROS payload type {ros_type}")
    artifact_format_map = _required_mapping_section(
        manifest,
        "artifact_formats",
        blockers,
    )
    declared_artifacts = set(artifact_format_map)
    extrinsic_map = _required_mapping_section(
        manifest,
        "lidar_extrinsics",
        blockers,
    )
    extrinsics = set(extrinsic_map)
    algorithm_interfaces = _required_mapping_section(
        manifest,
        "algorithm_interfaces",
        blockers,
    )
    stage_algorithm_interfaces = _required_mapping_section(
        manifest,
        "runtime_data_flow_stage_algorithm_interfaces",
        blockers,
    )
    adapter_aliases = _required_mapping_section(manifest, "adapter_aliases", blockers)
    adapter_relays = _required_mapping_section(manifest, "adapter_relays", blockers)
    profile_bindings = _required_mapping_section(
        manifest,
        "profile_data_sources",
        blockers,
    )
    product_bindings = _required_mapping_section(
        manifest,
        "product_data_sources",
        blockers,
    )
    frame_links = _required_mapping_section(manifest, "frame_links", blockers)
    canonical_body = str(frames.get("body") or "")
    canonical_lidar = str(frames.get("lidar") or "")
    canonical_camera = str(frames.get("camera") or "")
    canonical_gnss = str(frames.get("gnss") or "")
    known_frames = set(
        expand_frame_id_aliases(
            (
                str(frames.get("map") or ""),
                str(frames.get("odom") or ""),
                canonical_body,
                canonical_lidar,
                canonical_camera,
                canonical_gnss,
            )
        )
    )
    allowed_lidar_children = set(expand_frame_id_aliases((canonical_lidar,)))

    for format_name, format_spec in message_format_map.items():
        label = f"message format {format_name}"
        if not isinstance(format_spec, Mapping):
            blockers.append(f"{label} is not a mapping")
            continue
        if format_spec.get("name") != format_name:
            blockers.append(f"{label} name field mismatch")
        for field in ("ros_type", "frame_role"):
            if not str(format_spec.get(field) or "").strip():
                blockers.append(f"{label} {field} missing")

    for artifact_name, artifact_spec in artifact_format_map.items():
        label = f"artifact format {artifact_name}"
        if not isinstance(artifact_spec, Mapping):
            blockers.append(f"{label} is not a mapping")
            continue
        if artifact_spec.get("name") != artifact_name:
            blockers.append(f"{label} name field mismatch")
        for field in ("path", "artifact_type", "frame_role"):
            if not str(artifact_spec.get(field) or "").strip():
                blockers.append(f"{label} {field} missing")

    for topic, formats in topic_format_map.items():
        for format_name in tuple(formats or ()):
            format_name = str(format_name)
            if not _format_is_declared(format_name, declared_formats):
                blockers.append(f"topic format {topic} references undeclared format {format_name}")

    for section_name, frame_rules in (
        ("topic_allowed_frame_ids", topic_allowed_frames),
        ("real_runtime_topic_allowed_frame_ids", real_topic_allowed_frames),
    ):
        for topic, allowed_frames in frame_rules.items():
            if topic not in topic_formats:
                blockers.append(f"{section_name} topic {topic} has no topic format")
            frame_tuple = tuple(str(frame) for frame in (allowed_frames or ()))
            if not frame_tuple and not _topic_formats_are_metadata_only(
                str(topic),
                topic_format_map=topic_format_map,
                message_format_map=message_format_map,
            ):
                blockers.append(f"{section_name} topic {topic} has no allowed frames")
            for frame in frame_tuple:
                if known_frames and frame not in known_frames:
                    blockers.append(f"{section_name} topic {topic} allows unknown frame {frame}")

    for section_name, default_frames, allowed_section_name, frame_rules in (
        (
            "topic_default_frame_ids",
            topic_default_frames,
            "topic_allowed_frame_ids",
            topic_allowed_frames,
        ),
        (
            "real_runtime_topic_default_frame_ids",
            real_topic_default_frames,
            "real_runtime_topic_allowed_frame_ids",
            real_topic_allowed_frames,
        ),
    ):
        explicitly_unframed_topics = {
            topic
            for topic, allowed_frames in frame_rules.items()
            if not tuple(allowed_frames or ())
            and _topic_formats_are_metadata_only(
                str(topic),
                topic_format_map=topic_format_map,
                message_format_map=message_format_map,
            )
        }
        missing_defaults = set(frame_rules) - set(default_frames) - explicitly_unframed_topics
        extra_defaults = set(default_frames) - set(frame_rules)
        if missing_defaults or extra_defaults:
            blockers.append(f"{section_name} keys do not match {allowed_section_name}")
        for topic, default_frame in default_frames.items():
            topic = str(topic)
            if topic not in topic_formats:
                blockers.append(f"{section_name} topic {topic} has no topic format")
            frame = str(default_frame or "")
            allowed = tuple(str(item) for item in (frame_rules.get(topic) or ()))
            if not frame:
                if allowed or not _topic_formats_are_metadata_only(
                    topic,
                    topic_format_map=topic_format_map,
                    message_format_map=message_format_map,
                ):
                    blockers.append(f"{section_name} topic {topic} has no default frame")
            elif not (set(expand_frame_id_aliases((frame,))) & set(expand_frame_id_aliases(allowed))):
                blockers.append(f"{section_name} topic {topic} default frame {frame} is not allowed")

    for topic, real_frames in real_topic_allowed_frames.items():
        base_frames = set(tuple(topic_allowed_frames.get(topic) or ()))
        if topic not in topic_allowed_frames:
            blockers.append(f"real_runtime_topic_allowed_frame_ids topic {topic} missing from topic_allowed_frame_ids")
            continue
        extra = sorted(set(tuple(real_frames or ())) - base_frames)
        if extra:
            blockers.append(
                f"real_runtime_topic_allowed_frame_ids topic {topic} "
                "allows frames outside topic_allowed_frame_ids: " + ", ".join(extra)
            )

    real_data_source = runtime_contract_data_source(REAL_RUNTIME_CONTRACT)
    real_flow_topics = set(tuple(runtime_flow_topics.get(real_data_source) or ()))
    missing_required_flow_topics = sorted(set(real_required_frame_topics) - real_flow_topics)
    if missing_required_flow_topics:
        blockers.append(
            "real_runtime_required_topic_frame_ids missing from "
            f"{REAL_RUNTIME_CONTRACT} data flow: " + ", ".join(missing_required_flow_topics)
        )
    missing_required_frame_rules = sorted(set(real_required_frame_topics) - set(real_topic_allowed_frames))
    if missing_required_frame_rules:
        blockers.append(
            "real_runtime_required_topic_frame_ids missing real frame rules: " + ", ".join(missing_required_frame_rules)
        )
    missing_required_formats = sorted(set(real_required_frame_topics) - topic_formats)
    if missing_required_formats:
        blockers.append(
            "real_runtime_required_topic_frame_ids missing topic formats: " + ", ".join(missing_required_formats)
        )
    missing_endpoint_flow_topics = sorted(set(real_endpoint_input_topics) - real_flow_topics)
    if missing_endpoint_flow_topics:
        blockers.append(
            "real_runtime_required_endpoint_input_topics missing from "
            f"{REAL_RUNTIME_CONTRACT} data flow: " + ", ".join(missing_endpoint_flow_topics)
        )
    missing_endpoint_frame_requirements = sorted(set(real_endpoint_input_topics) - set(real_required_frame_topics))
    if missing_endpoint_frame_requirements:
        blockers.append(
            "real_runtime_required_endpoint_input_topics missing required frame "
            "evidence: " + ", ".join(missing_endpoint_frame_requirements)
        )
    missing_endpoint_frame_rules = sorted(set(real_endpoint_input_topics) - set(real_topic_allowed_frames))
    if missing_endpoint_frame_rules:
        blockers.append(
            "real_runtime_required_endpoint_input_topics missing real frame rules: "
            + ", ".join(missing_endpoint_frame_rules)
        )
    missing_endpoint_formats = sorted(set(real_endpoint_input_topics) - topic_formats)
    if missing_endpoint_formats:
        blockers.append(
            "real_runtime_required_endpoint_input_topics missing topic formats: " + ", ".join(missing_endpoint_formats)
        )

    for stage in template_stages:
        _append_artifact_reference_blockers(
            blockers,
            label=f"runtime_data_flow {stage.get('name')}",
            tokens=_runtime_tokens([stage]),
            declared_artifacts=declared_artifacts,
        )

    body_to_lidar = frame_links.get("body_to_lidar")
    if isinstance(body_to_lidar, Mapping):
        if canonical_body and body_to_lidar.get("parent") != canonical_body:
            blockers.append("frame_links.body_to_lidar parent is not canonical body")
        if canonical_lidar and body_to_lidar.get("child") != canonical_lidar:
            blockers.append("frame_links.body_to_lidar child is not canonical lidar")
    body_to_camera = frame_links.get("body_to_camera")
    if isinstance(body_to_camera, Mapping):
        if canonical_body and body_to_camera.get("parent") != canonical_body:
            blockers.append("frame_links.body_to_camera parent is not canonical body")
        if canonical_camera and body_to_camera.get("child") != canonical_camera:
            blockers.append("frame_links.body_to_camera child is not canonical camera")

    for profile_name, transform in extrinsic_map.items():
        if not isinstance(transform, Mapping):
            blockers.append(f"lidar extrinsic {profile_name} is not a mapping")
            continue
        if canonical_body and transform.get("parent") != canonical_body:
            blockers.append(f"lidar extrinsic {profile_name} parent is not canonical body")
        child = str(transform.get("child") or "")
        if allowed_lidar_children and child not in allowed_lidar_children:
            blockers.append(f"lidar extrinsic {profile_name} child {child} is not canonical lidar frame or alias")
        for field in ("x", "y", "z", "roll", "pitch", "yaw"):
            if not _is_numeric(transform.get(field)):
                blockers.append(f"lidar extrinsic {profile_name} {field} is not numeric")

    algorithm_interface_tokens: dict[str, tuple[str, ...]] = {}
    for interface_name, interface in algorithm_interfaces.items():
        interface_inputs = _dedupe_tokens(tuple(interface.get("inputs") or ()))
        interface_outputs = _dedupe_tokens(tuple(interface.get("outputs") or ()))
        tokens = tuple(
            str(token)
            for token in (
                *interface_inputs,
                *interface_outputs,
            )
        )
        _append_artifact_reference_blockers(
            blockers,
            label=f"algorithm interface {interface_name}",
            tokens=tokens,
            declared_artifacts=declared_artifacts,
        )
        for token in tokens:
            if token.startswith("artifact:") or not token.startswith("/"):
                continue
            if token not in topic_formats:
                blockers.append(f"algorithm interface {interface_name} references topic without format {token}")
        algorithm_interface_tokens[str(interface_name)] = tokens
        if not any(
            set(interface_inputs) <= set(tuple(stage.get("inputs") or ()))
            and set(interface_outputs) <= set(tuple(stage.get("outputs") or ()))
            for stage in template_stages
        ):
            blockers.append(f"algorithm interface {interface_name} is not covered by any runtime_data_flow stage")

    assigned_algorithm_interfaces: set[str] = set()
    for stage_name, interfaces in stage_algorithm_interfaces.items():
        stage_name = str(stage_name)
        if stage_name not in template_stage_by_name:
            blockers.append(f"runtime_data_flow_stage_algorithm_interfaces references unknown stage {stage_name}")
            continue
        if not isinstance(interfaces, (list, tuple)) or not interfaces:
            blockers.append(f"runtime_data_flow_stage_algorithm_interfaces {stage_name} must list interfaces")
            continue
        stage_tokens = set(_runtime_tokens([template_stage_by_name[stage_name]]))
        for interface_name in tuple(str(item) for item in interfaces):
            if interface_name not in algorithm_interfaces:
                blockers.append(
                    "runtime_data_flow_stage_algorithm_interfaces "
                    f"{stage_name} references unknown interface {interface_name}"
                )
                continue
            assigned_algorithm_interfaces.add(interface_name)
            interface_tokens = set(algorithm_interface_tokens.get(interface_name, ()))
            if not interface_tokens <= stage_tokens:
                blockers.append(
                    f"runtime_data_flow stage {stage_name} does not cover algorithm interface {interface_name}"
                )
    unassigned_algorithm_interfaces = sorted(
        set(str(name) for name in algorithm_interfaces) - assigned_algorithm_interfaces
    )
    if unassigned_algorithm_interfaces:
        blockers.append(
            "algorithm interfaces missing runtime data-flow stage binding: "
            + ", ".join(unassigned_algorithm_interfaces)
        )

    _append_adapter_mapping_blockers(
        blockers,
        section_label="adapter alias",
        aliases_by_surface=adapter_aliases,
        topic_formats=topic_formats,
        declared_formats=declared_formats,
    )
    _append_adapter_mapping_blockers(
        blockers,
        section_label="adapter relay",
        aliases_by_surface=adapter_relays,
        topic_formats=topic_formats,
        declared_formats=declared_formats,
    )

    if set(profile_bindings) != set(PROFILE_DATA_SOURCE_BINDINGS):
        blockers.append("profile_data_sources keys do not match runtime bindings")
    for profile_name, binding in profile_bindings.items():
        label = f"profile binding {profile_name}"
        if not isinstance(binding, Mapping):
            blockers.append(f"{label} is not a mapping")
            continue
        expected_binding = PROFILE_DATA_SOURCE_BINDINGS.get(str(profile_name))
        if binding.get("profile") != profile_name:
            blockers.append(f"{label} profile field mismatch")
        data_source_name = str(binding.get("data_source") or "")
        if not data_source_name:
            blockers.append(f"{label} data_source missing")
        elif data_source_name not in data_sources:
            blockers.append(f"{label} references unknown data source {data_source_name}")
        elif expected_binding is not None and data_source_name != expected_binding.data_source:
            blockers.append(f"{label} data_source drifted from runtime binding")
        if not str(binding.get("mode") or "").strip():
            blockers.append(f"{label} mode missing")
        elif expected_binding is not None and binding.get("mode") != expected_binding.mode:
            blockers.append(f"{label} mode drifted from runtime binding")
        if expected_binding is not None and (str(binding.get("note") or "") != expected_binding.note):
            blockers.append(f"{label} note drifted from runtime binding")

    if set(product_bindings) != set(PRODUCT_DATA_SOURCE_BINDINGS):
        blockers.append("product_data_sources keys do not match runtime bindings")
    for product_name, binding in product_bindings.items():
        label = f"Product binding {product_name}"
        if not isinstance(binding, Mapping):
            blockers.append(f"{label} is not a mapping")
            continue
        expected_binding = PRODUCT_DATA_SOURCE_BINDINGS.get(str(product_name))
        if binding.get("product") != product_name:
            blockers.append(f"{label} product field mismatch")
        data_source_name = str(binding.get("data_source") or "")
        if not data_source_name:
            blockers.append(f"{label} data_source missing")
        elif data_source_name not in data_sources:
            blockers.append(f"{label} references unknown data source {data_source_name}")
        elif expected_binding is not None and data_source_name != expected_binding.data_source:
            blockers.append(f"{label} data_source drifted from runtime binding")
        if not str(binding.get("mode") or "").strip():
            blockers.append(f"{label} mode missing")
        elif expected_binding is not None and binding.get("mode") != expected_binding.mode:
            blockers.append(f"{label} mode drifted from runtime binding")
        if expected_binding is not None and str(binding.get("note") or "") != expected_binding.note:
            blockers.append(f"{label} note drifted from runtime binding")

    for data_source_name, source in data_sources.items():
        if source.get("name") != data_source_name:
            blockers.append(f"data source {data_source_name} name field mismatch")

        extrinsic_profile = source.get("lidar_extrinsic_profile")
        if extrinsic_profile and extrinsic_profile not in extrinsics:
            blockers.append(f"data source {data_source_name} lidar extrinsic profile missing")

        stages = list(resolved_flows.get(data_source_name) or [])
        stage_names = [str(stage.get("name")) for stage in stages]
        expected_stage_names = [
            name
            for name in template_stage_names
            if name not in PRODUCT_SCOPED_RUNTIME_DATA_FLOW_STAGE_NAMES
        ]
        command_only = (
            not source.get("normalized_outputs")
            and not source.get("algorithm_entry_outputs")
            and not source.get("algorithm_context_outputs")
        )
        if command_only:
            expected_stage_names = ["endpoint_adapter", "command_boundary"]
        if stage_names != expected_stage_names:
            blockers.append(f"resolved {data_source_name} stage order mismatch")

        for stage in stages:
            stage_name = str(stage.get("name") or "")
            if not stage_name:
                blockers.append(f"resolved {data_source_name} stage name missing")
            template_stage = template_stage_by_name.get(stage_name)
            if template_stage is not None:
                for field in stage_metadata_fields:
                    if stage.get(field) != template_stage.get(field):
                        blockers.append(f"resolved {data_source_name} {stage_name} {field} drifted")
            _append_artifact_reference_blockers(
                blockers,
                label=f"resolved {data_source_name} {stage.get('name')}",
                tokens=_runtime_tokens([stage]),
                declared_artifacts=declared_artifacts,
            )

        unresolved = [token for token in _runtime_tokens(stages) if _contains_unresolved_placeholder(token)]
        if unresolved:
            for stage in stages:
                stage_tokens = _runtime_tokens([stage])
                if any(_contains_unresolved_placeholder(token) for token in stage_tokens):
                    blockers.append(f"resolved {data_source_name} {stage.get('name')} contains unresolved placeholder")
            continue

        by_name = _stage_map(stages)
        command_boundary = by_name.get("command_boundary")
        if command_boundary is None:
            blockers.append(f"resolved {data_source_name} command_boundary missing")
        else:
            outputs = tuple(command_boundary.get("outputs") or ())
            expected = (source.get("command_sink"),)
            if outputs != expected:
                blockers.append(f"resolved {data_source_name} command_boundary does not match data source sink")

        endpoint_adapter = by_name.get("endpoint_adapter")
        if endpoint_adapter is not None:
            if tuple(endpoint_adapter.get("inputs") or ()) != tuple(source.get("source_outputs") or ()):
                blockers.append(f"resolved {data_source_name} endpoint_adapter inputs drifted")
            if tuple(endpoint_adapter.get("outputs") or ()) != tuple(source.get("normalized_outputs") or ()):
                blockers.append(f"resolved {data_source_name} endpoint_adapter outputs drifted")

        slam_stage = by_name.get("slam_or_relayed_localization_map")
        if slam_stage is not None:
            if tuple(slam_stage.get("inputs") or ()) != tuple(source.get("normalized_outputs") or ()):
                blockers.append(f"resolved {data_source_name} slam inputs drifted")
            if tuple(slam_stage.get("outputs") or ()) != tuple(source.get("algorithm_entry_outputs") or ()):
                blockers.append(f"resolved {data_source_name} algorithm entry outputs drifted")

        map_stage = by_name.get("map_layers_and_exploration")
        if map_stage is not None:
            expected_map_inputs = _dedupe_tokens(
                tuple(source.get("algorithm_entry_outputs") or ())
                + tuple(source.get("algorithm_context_outputs") or ())
                + (
                    TOPICS.exploration_grid,
                    TOPICS.terrain_map_ext,
                    "module:TraversableFrontierModule.fused_cost",
                    "module:TraversableFrontierModule.slope_grid",
                    "module:TraversableFrontierModule.esdf_field",
                    "module:TraversableFrontierModule.elevation_map",
                )
            )
            if tuple(map_stage.get("inputs") or ()) != expected_map_inputs:
                blockers.append(f"resolved {data_source_name} map layer inputs drifted")

        if command_boundary is not None and tuple(command_boundary.get("inputs") or ()) != (TOPICS.cmd_vel,):
            blockers.append(f"resolved {data_source_name} command input drifted")

        topic_tokens = {
            token
            for token in (
                *source.get("source_outputs", ()),
                *source.get("normalized_outputs", ()),
                *source.get("algorithm_entry_outputs", ()),
                *source.get("algorithm_context_outputs", ()),
                *_runtime_tokens(stages),
            )
            if isinstance(token, str) and token.startswith("/")
        }
        missing_topic_formats = sorted(topic_tokens - topic_formats)
        if missing_topic_formats:
            blockers.append(f"resolved {data_source_name} topic formats missing: " + ", ".join(missing_topic_formats))
        actual_flow_topics = _dedupe_tokens(tuple(token for token in _runtime_tokens(stages) if token.startswith("/")))
        declared_flow_topics = tuple(str(topic) for topic in (runtime_flow_topics.get(data_source_name) or ()))
        if declared_flow_topics != actual_flow_topics:
            blockers.append(f"runtime_data_flow_topics {data_source_name} does not match resolved data flow")

    return {
        "ok": not blockers,
        "blockers": blockers,
        "checked_data_sources": sorted(data_sources),
        "checked_runtime_stages": template_stage_names,
        "checked_algorithm_interfaces": sorted(algorithm_interfaces),
        "checked_runtime_stage_algorithm_interfaces": {
            str(name): [str(item) for item in interfaces]
            for name, interfaces in stage_algorithm_interfaces.items()
            if isinstance(interfaces, (list, tuple))
        },
        "checked_profile_data_sources": sorted(profile_bindings),
        "checked_product_data_sources": sorted(product_bindings),
        "checked_frame_links": sorted(frame_links),
        "checked_topic_default_frame_ids": {str(topic): str(frame) for topic, frame in topic_default_frames.items()},
        "checked_real_runtime_topic_default_frame_ids": {
            str(topic): str(frame) for topic, frame in real_topic_default_frames.items()
        },
        "checked_real_required_topic_frame_ids": list(real_required_frame_topics),
        "checked_real_required_endpoint_input_topics": list(real_endpoint_input_topics),
    }


def _check_profile_runtime_specs() -> dict[str, Any]:
    blockers: list[str] = []
    checked_profiles: list[str] = []
    checked_products: list[str] = []
    checked_external_profiles: list[str] = []
    checked_external_profile_specs: list[str] = []
    checked_profile_binding_specs: list[str] = []
    checked_product_binding_specs: list[str] = []
    checked_ros2_binding_specs: list[str] = []
    skipped_ros2_binding_specs: list[str] = []
    skipped_ros2_binding_reasons: dict[str, str] = {}
    skipped_ros2_binding_violations: dict[str, list[str]] = {}
    checked_profile_adapters: list[str] = []
    checked_env_products: list[str] = []
    fixed_frames = _fixed_runtime_frame_ids()
    manifest = runtime_contract_manifest()
    topic_formats = set(_required_mapping_section(manifest, "topic_formats", blockers))
    _append_product_env_matrix_blockers(blockers)
    checked_real_env_product_topics = (
        _append_real_env_product_topic_closure_blockers(blockers)
    )
    _required_mapping_section(
        manifest,
        "product_data_sources",
        blockers,
    )
    checked_product_runtime_stages = _append_product_runtime_data_flow_blockers(
        blockers,
        products=load_runtime_graph().products,
    )
    _append_product_planner_backend_blockers(blockers)

    if set(HOST_PROFILE_DEFAULTS) != set(PROFILE_DATA_SOURCE_BINDINGS):
        blockers.append("Host Profile defaults and Profile data-source bindings differ")
    if set(FIELD_PRODUCT_HOST_DEFAULTS) != set(PRODUCT_DATA_SOURCE_BINDINGS):
        blockers.append("Field Product Host defaults and Product data-source bindings differ")

    for profile, profile_data in HOST_PROFILE_DEFAULTS.items():
        checked_profiles.append(profile)
        if profile_data.get("_external_launcher") or profile_data.get("_runtime_contract"):
            checked_external_profiles.append(profile)
            _append_external_profile_metadata_blockers(
                blockers,
                profile_name=profile,
                profile_data=profile_data,
            )
            for record in (False, True):
                action_name = "record" if record else "default"
                checked_external_profile_specs.append(f"{profile}:{action_name}")
                try:
                    external_spec = resolve_runtime_run_spec(
                        profile,
                        profile_data,
                        record=record,
                    )
                    _append_selection_binding_spec_blockers(
                        blockers,
                        binding=PROFILE_DATA_SOURCE_BINDINGS[profile],
                        spec_label=(f"profile {profile} external {action_name} runtime spec"),
                        spec=external_spec,
                    )
                    checked_profile_binding_specs.append(f"{profile}:external_{action_name}")
                    external_result = validate_runtime_switch(external_spec)
                except Exception as exc:  # pragma: no cover - surfaced in audit payload
                    blockers.append(f"profile {profile} external {action_name} runtime spec failed: {exc}")
                    continue
                if not external_result.ok:
                    blockers.extend(
                        f"profile {profile} external {action_name} runtime spec: {blocker}"
                        for blocker in external_result.blockers
                    )
        try:
            config = _resolve_audit_host_config(profile)
            default_ros2_spec = f"{profile}:default"
            if _enforces_ros2_runtime_binding_audit(profile):
                checked_ros2_binding_specs.append(default_ros2_spec)
                _append_ros2_runtime_binding_blockers(
                    blockers,
                    spec_label=f"profile {profile} runtime bindings",
                    config=config,
                )
            else:
                skipped_ros2_binding_specs.append(default_ros2_spec)
                skipped_ros2_binding_reasons[default_ros2_spec] = _ros2_runtime_binding_skip_reason(profile)
                skipped_ros2_binding_violations[default_ros2_spec] = _ros2_runtime_binding_violations_for_config(config)
            spec = resolve_runtime_run_spec(profile, config)
            _append_selection_binding_spec_blockers(
                blockers,
                binding=PROFILE_DATA_SOURCE_BINDINGS[profile],
                spec_label=f"profile {profile} runtime spec",
                spec=spec,
            )
            checked_profile_binding_specs.append(f"{profile}:default")
            result = validate_runtime_switch(spec)
        except Exception as exc:  # pragma: no cover - surfaced in audit payload
            blockers.append(f"profile {profile} runtime spec failed: {exc}")
            continue
        if not result.ok:
            blockers.extend(f"profile {profile}: {blocker}" for blocker in result.blockers)

    for product in FIELD_PRODUCT_HOST_DEFAULTS:
        checked_products.append(product)
        try:
            config = _resolve_audit_host_config(product, env="real")
            spec_label = f"{product}:real"
            checked_ros2_binding_specs.append(spec_label)
            _append_ros2_runtime_binding_blockers(
                blockers,
                spec_label=f"Product {product} real Env runtime bindings",
                config=config,
            )
            checked_product_binding_specs.append(spec_label)
        except Exception as exc:  # pragma: no cover - surfaced in audit payload
            blockers.append(f"Product {product} real Env config failed: {exc}")

    for adapter_name, adapter in PROFILE_ADAPTERS.items():
        if adapter.name != adapter_name:
            blockers.append(f"Profile adapter {adapter_name} name field mismatch")
        if adapter.data_source not in DATA_SOURCE_CONTRACTS:
            blockers.append(f"Profile adapter {adapter_name} data source missing")
            source = None
        else:
            source = DATA_SOURCE_CONTRACTS[adapter.data_source]
            expected_simulation_only = source.provider != "hardware"
            if adapter.simulation_only is not expected_simulation_only:
                blockers.append(f"Profile adapter {adapter_name} simulation_only does not match data source provider")
            launcher = str(adapter.external_launcher or "")
            if source.provider == "hardware" and launcher:
                blockers.append(f"Profile adapter {adapter_name} for hardware must not declare external launcher")
            elif launcher and not (REPO_ROOT / launcher).exists():
                blockers.append(f"Profile adapter {adapter_name} external launcher missing: {launcher}")
            elif adapter.simulation_only and adapter.default_actions and not launcher:
                blockers.append(f"Profile adapter {adapter_name} with actions is missing an external launcher")
        if adapter.runtime_contract != adapter.data_source:
            blockers.append(f"Profile adapter {adapter_name} runtime contract mismatch")
        supported_profiles = set(adapter.supported_profiles)
        unknown_profiles = sorted(supported_profiles - set(HOST_PROFILE_DEFAULTS))
        if unknown_profiles:
            blockers.append(
                f"Profile adapter {adapter_name} references unknown supported profiles: " + ", ".join(unknown_profiles)
            )
        extras = sorted(set(adapter.profile_overrides) - supported_profiles)
        for name in extras:
            blockers.append(
                f"Profile adapter {adapter_name} profile_overrides references unsupported name {name}"
            )
        supported_actions = supported_profiles
        for table_name, table in (
            ("default_actions", adapter.default_actions),
            ("record_actions", adapter.record_actions),
        ):
            extras = sorted(set(table) - supported_actions)
            for name in extras:
                blockers.append(
                    f"Profile adapter {adapter_name} {table_name} references unsupported name {name}"
                )
            if table:
                for name in sorted(supported_actions - set(table)):
                    blockers.append(
                        f"Profile adapter {adapter_name} {table_name} missing supported name {name}"
                    )
        _append_profile_adapter_frame_override_blockers(
            blockers,
            adapter_name=adapter_name,
            section_name="config_overrides",
            overrides=adapter.config_overrides,
            fixed_frames=fixed_frames,
        )
        _append_profile_adapter_topic_override_blockers(
            blockers,
            adapter_name=adapter_name,
            section_name="config_overrides",
            overrides=adapter.config_overrides,
            topic_formats=topic_formats,
        )
        _append_profile_adapter_file_override_blockers(
            blockers,
            adapter_name=adapter_name,
            section_name="config_overrides",
            overrides=adapter.config_overrides,
        )
        for profile_name, overrides in adapter.profile_overrides.items():
            _append_profile_adapter_frame_override_blockers(
                blockers,
                adapter_name=adapter_name,
                section_name=f"profile_overrides.{profile_name}",
                overrides=overrides,
                fixed_frames=fixed_frames,
            )
            _append_profile_adapter_topic_override_blockers(
                blockers,
                adapter_name=adapter_name,
                section_name=f"profile_overrides.{profile_name}",
                overrides=overrides,
                topic_formats=topic_formats,
            )
            _append_profile_adapter_file_override_blockers(
                blockers,
                adapter_name=adapter_name,
                section_name=f"profile_overrides.{profile_name}",
                overrides=overrides,
            )
        for selection_name in adapter.supported_profiles:
            selection_kind = "profile"
            checked_profile_adapters.append(f"{adapter_name}:{selection_name}")
            try:
                config = _resolve_audit_host_config(selection_name, profile_adapter=adapter_name)
                adapter_ros2_spec = f"{adapter_name}:{selection_name}"
                if _enforces_ros2_runtime_binding_audit(selection_name, adapter):
                    checked_ros2_binding_specs.append(adapter_ros2_spec)
                    _append_ros2_runtime_binding_blockers(
                        blockers,
                        spec_label=(
                            f"Profile adapter {adapter_name} {selection_kind} "
                            f"{selection_name} runtime bindings"
                        ),
                        config=config,
                    )
                else:
                    skipped_ros2_binding_specs.append(adapter_ros2_spec)
                    skipped_ros2_binding_reasons[adapter_ros2_spec] = _ros2_runtime_binding_skip_reason(
                        selection_name, adapter
                    )
                    skipped_ros2_binding_violations[adapter_ros2_spec] = _ros2_runtime_binding_violations_for_config(
                        config
                    )
                spec = resolve_runtime_run_spec(selection_name, config)
                result = validate_runtime_switch(spec)
            except Exception as exc:  # pragma: no cover - surfaced in audit payload
                blockers.append(
                    f"Profile adapter {adapter_name} {selection_kind} {selection_name} "
                    f"runtime spec failed: {exc}"
                )
                continue
            if not result.ok:
                blockers.extend(
                    f"Profile adapter {adapter_name} {selection_kind} {selection_name}: {blocker}"
                    for blocker in result.blockers
                )

    runtime_graph = load_runtime_graph()
    known_products = set(FIELD_PRODUCT_HOST_DEFAULTS)
    for env_name, backend_name, implementation in _iter_env_implementations(runtime_graph):
        implementation_label = (
            f"{env_name}:{backend_name}" if backend_name is not None else env_name
        )
        supported_products = {
            str(product) for product in (implementation.get("supported_products") or ())
        }
        unknown_products = sorted(supported_products - known_products)
        if unknown_products:
            blockers.append(
                f"Env implementation {implementation_label} references unknown Products: "
                + ", ".join(unknown_products)
            )
        for product in sorted(supported_products & known_products):
            checked_label = f"{implementation_label}:{product}"
            checked_env_products.append(checked_label)
            try:
                config = _resolve_audit_host_config(
                    product,
                    env=env_name,
                    backend=backend_name,
                )
                checked_ros2_binding_specs.append(checked_label)
                _append_ros2_runtime_binding_blockers(
                    blockers,
                    spec_label=(
                        f"Env implementation {implementation_label} Product {product} "
                        "runtime bindings"
                    ),
                    config=config,
                )
            except Exception as exc:  # pragma: no cover - surfaced in audit payload
                blockers.append(
                    f"Env implementation {implementation_label} Product {product} "
                    f"config failed: {exc}"
                )

    return {
        "ok": not blockers,
        "blockers": blockers,
        "checked_profiles": checked_profiles,
        "checked_products": checked_products,
        "checked_external_profiles": checked_external_profiles,
        "checked_external_profile_specs": checked_external_profile_specs,
        "checked_profile_binding_specs": checked_profile_binding_specs,
        "checked_product_binding_specs": checked_product_binding_specs,
        "checked_ros2_binding_specs": checked_ros2_binding_specs,
        "skipped_ros2_binding_specs": skipped_ros2_binding_specs,
        "skipped_ros2_binding_reasons": skipped_ros2_binding_reasons,
        "skipped_ros2_binding_violations": skipped_ros2_binding_violations,
        "checked_profile_adapters": checked_profile_adapters,
        "checked_env_products": checked_env_products,
        "product_env_matrix": _product_env_matrix(),
        "checked_real_env_product_topics": checked_real_env_product_topics,
        "checked_product_runtime_data_flow_stages": checked_product_runtime_stages,
    }


def _load_real_collector_module():
    script = REPO_ROOT / "scripts" / "gates" / "real_runtime_evidence_collect.py"
    spec = importlib.util.spec_from_file_location(
        "lingtu_real_runtime_evidence_collect_audit",
        script,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load real_runtime_evidence_collect.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _check_real_collector(manifest: Mapping[str, Any]) -> dict[str, Any]:
    blockers: list[str] = []
    collector = _load_real_collector_module()
    observed_topics = set(collector.OBSERVED_TOPICS)
    required_topics = set(runtime_data_flow_topics(REAL_RUNTIME_CONTRACT))
    missing_topics = sorted(required_topics - observed_topics)
    if missing_topics:
        blockers.append(
            f"real collector does not observe required {REAL_RUNTIME_CONTRACT} topics: " + ", ".join(missing_topics)
        )

    frame_evidence = collector.build_frame_evidence(
        {name: 1 for name in FRAME_LINKS},
    )
    expected_frame_links = {name: asdict(link) for name, link in FRAME_LINKS.items()}
    for name, link in expected_frame_links.items():
        entry = frame_evidence.get(name)
        if entry is None:
            blockers.append(f"real collector frame evidence missing {name}")
            continue
        if entry.get("parent") != link["parent"] or entry.get("child") != link["child"]:
            blockers.append(f"real collector frame evidence mismatch for {name}")

    expected_topic_frame_contract = _expected_real_required_topic_frame_contract(manifest)
    collector_topic_frame_contract = collector.build_required_topic_frame_contract(REAL_RUNTIME_CONTRACT)
    if collector_topic_frame_contract != expected_topic_frame_contract:
        blockers.append("real collector required topic frame contract does not match runtime contract")

    collector_report_contract = collector.build_real_runtime_report(
        topic_evidence={},
        frame_samples={},
        command_subscribers=[],
        duration_sec=0.0,
    )["runtime_contract"]

    expected_frames_contract = normalize_runtime_frames_contract(manifest.get("frames"))
    collector_frames_contract = normalize_runtime_frames_contract(collector_report_contract.get("frames"))
    if collector_frames_contract != expected_frames_contract:
        blockers.append("real collector runtime frames contract does not match runtime contract")
    if runtime_frames_contract() != expected_frames_contract:
        blockers.append("canonical runtime frames contract does not match runtime contract")

    expected_topic_default_frame_contract = _as_str_mapping(manifest.get("real_runtime_topic_default_frame_ids"))
    collector_topic_default_frame_contract = _as_str_mapping(collector_report_contract.get("topic_default_frame_ids"))
    if collector_topic_default_frame_contract != expected_topic_default_frame_contract:
        blockers.append("real collector topic default frame contract does not match runtime contract")
    if runtime_topic_default_frame_contract(REAL_RUNTIME_CONTRACT) != expected_topic_default_frame_contract:
        blockers.append("canonical topic default frame contract does not match runtime contract")

    expected_topic_allowed_frame_contract = {
        topic: list(frames)
        for topic, frames in _as_tuple_mapping(manifest.get("real_runtime_topic_allowed_frame_ids")).items()
    }
    collector_topic_allowed_frame_contract = {
        topic: list(frames)
        for topic, frames in _as_tuple_mapping(collector_report_contract.get("topic_allowed_frame_ids")).items()
    }
    if collector_topic_allowed_frame_contract != expected_topic_allowed_frame_contract:
        blockers.append("real collector topic allowed frame contract does not match runtime contract")
    if runtime_topic_allowed_frame_contract(REAL_RUNTIME_CONTRACT) != expected_topic_allowed_frame_contract:
        blockers.append("canonical topic allowed frame contract does not match runtime contract")

    expected_algorithm_interface_contract = normalize_algorithm_interface_contract(manifest.get("algorithm_interfaces"))
    collector_algorithm_interface_contract = normalize_algorithm_interface_contract(
        collector_report_contract.get("algorithm_interfaces")
    )
    if collector_algorithm_interface_contract != expected_algorithm_interface_contract:
        blockers.append("real collector algorithm interface contract does not match runtime contract")
    if runtime_algorithm_interface_contract() != expected_algorithm_interface_contract:
        blockers.append("canonical algorithm interface contract does not match runtime contract")

    expected_stage_algorithm_interface_contract = {
        stage: list(interfaces)
        for stage, interfaces in _as_tuple_mapping(manifest.get("runtime_data_flow_stage_algorithm_interfaces")).items()
    }
    collector_stage_algorithm_interface_contract = {
        stage: list(interfaces)
        for stage, interfaces in _as_tuple_mapping(
            collector_report_contract.get("runtime_data_flow_stage_algorithm_interfaces")
        ).items()
    }
    if collector_stage_algorithm_interface_contract != expected_stage_algorithm_interface_contract:
        blockers.append("real collector runtime stage algorithm interface contract does not match runtime contract")
    if runtime_stage_algorithm_interface_contract() != expected_stage_algorithm_interface_contract:
        blockers.append("canonical runtime stage algorithm interface contract does not match runtime contract")

    source = (REPO_ROOT / "scripts" / "gates" / "real_runtime_evidence_collect.py").read_text(encoding="utf-8")
    if ".create_publisher(" in source or ".publish(" in source:
        blockers.append("real collector publishes a control-capable topic")

    return {
        "ok": not blockers,
        "blockers": blockers,
        "runtime_contract": REAL_RUNTIME_CONTRACT,
        "observed_topics": sorted(observed_topics),
        "required_runtime_topics": sorted(required_topics),
        "required_real_runtime_topics": sorted(required_topics),
        "frames": collector_frames_contract,
        "topic_default_frame_ids": collector_topic_default_frame_contract,
        "topic_allowed_frame_ids": collector_topic_allowed_frame_contract,
        "algorithm_interfaces": collector_algorithm_interface_contract,
        "runtime_data_flow_stage_algorithm_interfaces": (collector_stage_algorithm_interface_contract),
        "required_topic_frame_contract": collector_topic_frame_contract,
    }


def _expected_real_required_topic_frame_contract(
    manifest: Mapping[str, Any],
) -> dict[str, dict[str, Any]]:
    required_topics = tuple(str(topic) for topic in (manifest.get("real_runtime_required_topic_frame_ids") or ()))
    default_frame_ids = _as_str_mapping(manifest.get("real_runtime_topic_default_frame_ids"))
    allowed_frame_ids = _as_tuple_mapping(manifest.get("real_runtime_topic_allowed_frame_ids"))
    return {
        topic: {
            "default_frame_id": default_frame_ids.get(topic),
            "allowed_frame_ids": list(allowed_frame_ids.get(topic, ())),
        }
        for topic in required_topics
    }


def _check_runtime_validation_gates() -> dict[str, Any]:
    result = validate_runtime_validation_gates()
    return {
        "ok": result.ok,
        "blockers": list(result.blockers),
        "commands": result.commands,
        "validates": result.validates,
        "checks": result.checks,
        "coverage": result.coverage,
        "acceptance": result.acceptance,
    }


def _display_path(path: Path) -> str:
    if path.is_relative_to(REPO_ROOT):
        return path.relative_to(REPO_ROOT).as_posix()
    return path.as_posix()


def _markdown_frame_list(frames: tuple[str, ...]) -> str:
    if not frames:
        return "`n/a`"
    return ", ".join(f"`{frame}`" for frame in frames)


def _check_ros_frame_contract_doc(
    manifest: Mapping[str, Any],
    doc_path: Path | None = None,
) -> dict[str, Any]:
    blockers: list[str] = []
    path = doc_path or ROS_FRAME_CONTRACT_DOC
    if not path.is_file():
        return {
            "ok": False,
            "blockers": [f"ros_frame_contract.md missing: {path}"],
            "checked_doc": _display_path(path),
            "checked_topics": [],
        }

    doc = path.read_text(encoding="utf-8")
    general_allowed = _as_tuple_mapping(manifest.get("topic_allowed_frame_ids"))
    real_allowed = _as_tuple_mapping(manifest.get("real_runtime_topic_allowed_frame_ids"))
    real_defaults = _as_str_mapping(manifest.get("real_runtime_topic_default_frame_ids"))
    required = {str(topic) for topic in (manifest.get("real_runtime_required_topic_frame_ids") or ())}
    topic_format_map = _as_tuple_mapping(manifest.get("topic_formats"))
    message_format_map = manifest.get("message_formats") or {}
    checked_topics: list[str] = []
    for topic, real_frames in real_allowed.items():
        checked_topics.append(topic)
        if topic not in general_allowed:
            blockers.append(f"ros_frame_contract.md topic {topic} has no general frame rule")
            continue
        default_frame = real_defaults.get(topic)
        if default_frame is None:
            if not _topic_formats_are_metadata_only(
                topic,
                topic_format_map=topic_format_map,
                message_format_map=message_format_map,
            ):
                blockers.append(f"ros_frame_contract.md topic {topic} has no real default frame")
                continue
            default_frame = "n/a"
        expected_row = (
            f"| `{topic}` | "
            f"`{default_frame}` | "
            f"{_markdown_frame_list(general_allowed[topic])} | "
            f"{'yes' if topic in required else 'no'} | "
            f"{_markdown_frame_list(real_frames)} |"
        )
        if expected_row not in doc:
            blockers.append(f"ros_frame_contract.md topic frame row drifted for {topic}")

    required_phrase = f"must reject `{TOPICS.map_cloud}` outside `map`"
    if required_phrase not in doc:
        blockers.append("ros_frame_contract.md missing real map_cloud rejection phrase")

    return {
        "ok": not blockers,
        "blockers": blockers,
        "checked_doc": _display_path(path),
        "checked_topics": checked_topics,
        "required_topics": sorted(required),
    }


def _source_contract_paths(roots: tuple[str, ...]) -> list[tuple[str, Path]]:
    paths: list[tuple[str, Path]] = []
    for relative_root in roots:
        root = REPO_ROOT / relative_root
        if root.is_file():
            paths.append((Path(relative_root).as_posix(), root))
            continue
        if not root.is_dir():
            paths.append((Path(relative_root).as_posix(), root))
            continue
        for path in sorted(root.rglob("*.py")):
            if "__pycache__" in path.parts or "tests" in path.parts:
                continue
            paths.append((path.relative_to(REPO_ROOT).as_posix(), path))
    return paths


def _source_frame_contract_paths() -> list[tuple[str, Path]]:
    return _source_contract_paths(SOURCE_FRAME_CONTRACT_ROOTS)


def _check_source_patterns(
    *,
    roots: tuple[str, ...],
    patterns: tuple[tuple[str, re.Pattern[str]], ...],
    violation_label: str,
) -> dict[str, Any]:
    blockers: list[str] = []
    matches: list[dict[str, Any]] = []
    checked_files: list[str] = []

    for relative_path, path in _source_contract_paths(roots):
        if not path.is_file():
            blockers.append(f"{violation_label} file missing: {relative_path}")
            continue
        checked_files.append(relative_path)
        for line_number, line in enumerate(
            path.read_text(encoding="utf-8").splitlines(),
            1,
        ):
            if "# noqa" in line:
                continue
            for pattern_name, pattern in patterns:
                if pattern.search(line) is None:
                    continue
                matches.append(
                    {
                        "file": relative_path,
                        "line": line_number,
                        "pattern": pattern_name,
                        "text": line.strip(),
                    }
                )

    for match in matches:
        blockers.append(f"{violation_label} violation {match['file']}:{match['line']} {match['pattern']}")

    return {
        "ok": not blockers,
        "blockers": blockers,
        "checked_roots": list(roots),
        "checked_files": checked_files,
        "forbidden_patterns": [name for name, _ in patterns],
        "matches": matches,
    }


def _check_source_frame_contracts() -> dict[str, Any]:
    return _check_source_patterns(
        roots=SOURCE_FRAME_CONTRACT_ROOTS,
        patterns=FORBIDDEN_SOURCE_FRAME_PATTERNS,
        violation_label="source frame contract",
    )


def _check_source_topic_contracts() -> dict[str, Any]:
    return _check_source_patterns(
        roots=SOURCE_TOPIC_CONTRACT_ROOTS,
        patterns=FORBIDDEN_SOURCE_TOPIC_PATTERNS,
        violation_label="source topic contract",
    )


def build_runtime_contract_audit(
    topic_contract_path: Path | None = None,
) -> dict[str, Any]:
    """Build the complete offline runtime-contract audit report."""

    manifest = runtime_contract_manifest()
    contract = _read_yaml(topic_contract_path or REPO_ROOT / "config" / "topic_contract.yaml")
    checks = {
        "yaml_contract": _check_yaml_contract(contract, manifest),
        "profile_runtime_specs": _check_profile_runtime_specs(),
        "runtime_contract_integrity": _check_runtime_contract_integrity(manifest),
        "real_runtime_collector": _check_real_collector(manifest),
        "runtime_validation_gates": _check_runtime_validation_gates(),
        "ros_frame_contract_doc": _check_ros_frame_contract_doc(manifest),
        "source_frame_contracts": _check_source_frame_contracts(),
        "source_topic_contracts": _check_source_topic_contracts(),
    }
    blockers: list[str] = []
    if tuple(checks) != RUNTIME_AUDIT_CHECKS:
        blockers.append(
            f"runtime_audit checks list drifted: expected={list(RUNTIME_AUDIT_CHECKS)} actual={list(checks)}"
        )
    blockers.extend(
        [f"{check_name}: {blocker}" for check_name, check in checks.items() for blocker in check["blockers"]]
    )
    return {
        "schema_version": "lingtu.runtime_contract_audit.v1",
        "ok": not blockers,
        "blockers": blockers,
        "validation_gate": runtime_validation_gates()["runtime_audit"],
        "checks": checks,
        "summary": {
            "data_sources": sorted(manifest["data_sources"]),
            "profiles": sorted(HOST_PROFILE_DEFAULTS),
            "products": sorted(FIELD_PRODUCT_HOST_DEFAULTS),
            "profile_adapters": sorted(PROFILE_ADAPTERS),
            "frame_links": manifest["frame_links"],
        },
    }

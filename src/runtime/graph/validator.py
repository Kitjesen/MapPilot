"""Validation helpers for Runtime Graph contracts."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from runtime.contracts.product_runtime import resolve_product_spec_contracts
from runtime.runtime_interface import TOPICS

from .loader import RuntimeGraph, load_runtime_graph, resolve_product_variant_spec

ENV_SCHEMA_VERSION = "lingtu.runtime_graph.env.v1"
PRODUCT_SCHEMA_VERSION = "lingtu.runtime_graph.product.v1"
REQUIRED_ENVS = frozenset({"real", "sim"})
SIM_BACKENDS = frozenset({"mujoco_native", "mujoco_host", "gazebo"})
PRODUCT_MODE_REQUIRED_FIELDS = (
    "product_mode",
    "product_session",
    "session_mode",
    "native_control_mode",
    "slam_mode",
    "requires_map",
    "switch_policy",
    "online_hot_switch_supported",
    "processes",
)
PRODUCT_SWITCH_POLICIES = frozenset({"cold_restart", "hot_switch"})
PRODUCT_SESSION_MODES = frozenset({"none", "mapping", "navigating", "exploring"})
PRODUCT_SLAM_MODES = frozenset({"none", "mapping", "localization"})
PRODUCT_CONTROL_MODES = frozenset({"teleop", "teleop_avoid", "autonomy"})
OPERATOR_MOTION_TOPICS = frozenset(
    {
        TOPICS.operator_motion_control,
        TOPICS.operator_motion_sample,
        TOPICS.operator_motion_ack,
        TOPICS.operator_motion_status,
    }
)
OPERATOR_MOTION_CAPABILITIES = frozenset(
    {
        "operator_motion_typed_dds_interface",
        "native_operator_motion_authority",
    }
)
OPERATOR_MOTION_CRITICAL_MODULES = frozenset({"nav.commands", "GatewayModule"})
PYTHON_MOTION_AUTHORITY_MODULES = frozenset({"TeleopModule", "nav.velocity_mux"})
PYTHON_REALTIME_MAP_MODULES = frozenset(
    {
        "OccupancyGridModule",
        "VoxelGridModule",
        "ElevationMapModule",
        "ESDFModule",
        "TraversabilityCostModule",
        "SemanticMapModule",
    }
)
ResolvedProductVariants = dict[
    str,
    tuple[tuple[str | None, dict[str, Any]], ...],
]


@dataclass(frozen=True)
class RuntimeGraphIssue:
    """A machine-readable Runtime Graph validation issue."""

    code: str
    message: str
    scope: str = "runtime_graph"
    severity: str = "error"

    def as_dict(self) -> dict[str, str]:
        """Return the stable JSON representation used by CLI and Gateway."""

        return {
            "code": self.code,
            "message": self.message,
            "scope": self.scope,
            "severity": self.severity,
        }


def validate_runtime_graph(graph: RuntimeGraph | None = None) -> list[RuntimeGraphIssue]:
    """Validate the loaded Runtime Graph contract files."""

    graph = graph or load_runtime_graph()
    issues: list[RuntimeGraphIssue] = []
    topics = graph.topic_contracts

    if not topics:
        issues.append(_issue("topics_missing", "Runtime Graph has no topics"))
        return issues

    for topic in graph.native_contract_topics:
        if topic not in topics:
            issues.append(
                _issue(
                    "native_topic_missing",
                    f"native contract topic {topic} is not declared",
                    scope="topics",
                )
            )

    resolved_products, variant_issues = _resolve_products_for_validation(
        graph.products
    )
    issues.extend(variant_issues)
    for name, variants in resolved_products.items():
        for variant, product in variants:
            issues.extend(
                _tag_variant_issues(
                    name,
                    variant,
                    _validate_product(name, product, topics),
                )
            )
    issues.extend(_validate_product_session_defaults(graph))

    actual_envs = set(graph.envs)
    if actual_envs != REQUIRED_ENVS:
        missing = sorted(REQUIRED_ENVS - actual_envs)
        unexpected = sorted(actual_envs - REQUIRED_ENVS)
        issues.append(
            _issue(
                "env_catalog_invalid",
                f"Runtime Graph Envs must be exactly real and sim; "
                f"missing={missing}, unexpected={unexpected}",
                scope="envs",
            )
        )
    for name, env in graph.envs.items():
        issues.extend(
            _validate_env(
                name,
                env,
                products=graph.products,
                resolved_products=resolved_products,
                topics=topics,
                native_topics=graph.native_contract_topics,
            )
        )

    issues.extend(
        _validate_real_product_topic_closure(graph, resolved_products)
    )
    issues.extend(
        _validate_real_product_process_closure(graph, resolved_products)
    )
    issues.extend(_validate_native_contract_parity(graph))
    return issues


def assert_runtime_graph_valid(graph: RuntimeGraph | None = None) -> RuntimeGraph:
    """Return *graph* if valid, otherwise raise ValueError with issue details."""

    graph = graph or load_runtime_graph()
    issues = validate_runtime_graph(graph)
    if issues:
        joined = "; ".join(f"{issue.code}: {issue.message}" for issue in issues)
        raise ValueError(f"Runtime Graph validation failed: {joined}")
    return graph


def _resolve_products_for_validation(
    products: dict[str, dict[str, Any]],
) -> tuple[ResolvedProductVariants, list[RuntimeGraphIssue]]:
    """Resolve every declared Product variant for the validation passes."""

    resolved: ResolvedProductVariants = {}
    issues: list[RuntimeGraphIssue] = []
    for name, product in products.items():
        variants = product.get("variants")
        if variants is None:
            resolved[name] = ((None, product),)
            continue
        if not isinstance(variants, Mapping) or not variants:
            issues.append(
                _issue(
                    "product_variants_invalid",
                    f"Product {name} variants must be a non-empty mapping",
                    scope=f"product:{name}",
                )
            )
            resolved[name] = ()
            continue

        default_variant = str(product.get("default_variant") or "").strip()
        if not default_variant or default_variant not in variants:
            issues.append(
                _issue(
                    "product_default_variant_invalid",
                    f"Product {name} default_variant {default_variant!r} is not declared",
                    scope=f"product:{name}",
                )
            )

        selections: list[tuple[str | None, dict[str, Any]]] = []
        for raw_variant in sorted(variants, key=str):
            variant = str(raw_variant).strip()
            if not variant or variant != raw_variant:
                issues.append(
                    _issue(
                        "product_variant_name_invalid",
                        f"Product {name} has invalid variant name {raw_variant!r}",
                        scope=f"product:{name}",
                    )
                )
                continue
            try:
                spec = resolve_product_variant_spec(
                    name,
                    product,
                    product_variant=variant,
                )
            except ValueError as exc:
                issues.append(
                    _issue(
                        "product_variant_invalid",
                        f"Product {name} variant {variant!r}: {exc}",
                        scope=_product_variant_scope(name, variant),
                    )
                )
                continue
            selections.append((variant, spec))
        resolved[name] = tuple(selections)
    return resolved, issues


def _tag_variant_issues(
    product: str,
    variant: str | None,
    issues: list[RuntimeGraphIssue],
) -> list[RuntimeGraphIssue]:
    if variant is None:
        return issues
    return [
        RuntimeGraphIssue(
            code=issue.code,
            message=f"Product {product} variant {variant!r}: {issue.message}",
            scope=(
                _product_variant_scope(product, variant)
                if issue.scope == f"product:{product}"
                else issue.scope
            ),
            severity=issue.severity,
        )
        for issue in issues
    ]


def _product_variant_label(product: str, variant: str | None) -> str:
    if variant is None:
        return product
    return f"{product} variant {variant!r}"


def _product_variant_scope(product: str, variant: str | None) -> str:
    if variant is None:
        return f"product:{product}"
    return f"product:{product}:variant:{variant}"


def _validate_product(
    name: str,
    product: dict[str, Any],
    topics: dict[str, Any],
) -> list[RuntimeGraphIssue]:
    issues: list[RuntimeGraphIssue] = []
    if product.get("schema_version") != PRODUCT_SCHEMA_VERSION:
        issues.append(
            _issue(
                "product_schema_invalid",
                f"Product {name} must use schema {PRODUCT_SCHEMA_VERSION}",
                scope=f"product:{name}",
            )
        )
    try:
        contract = resolve_product_spec_contracts(name, product)
    except ValueError as exc:
        issues.append(
            _issue(
                "product_contract_invalid",
                str(exc),
                scope=f"product:{name}",
            )
        )
        required: tuple[str, ...] = ()
        capabilities: tuple[str, ...] = ()
    else:
        required = contract.topics
        capabilities = contract.capabilities
    if not _string_tuple(product.get("processes")):
        issues.append(
            _issue(
                "product_processes_missing",
                f"product {name} must explicitly declare its processes",
                scope=f"product:{name}",
            )
        )
    for topic in required:
        if topic not in topics:
            issues.append(
                _issue(
                    "product_required_topic_missing",
                    f"product {name} requires undeclared topic {topic}",
                    scope=f"product:{name}",
                )
            )
    issues.extend(
        _validate_operator_motion_contract(
            name,
            product,
            required,
            capabilities,
        )
    )
    issues.extend(
        _validate_native_final_writer_contract(name, product, capabilities)
    )
    issues.extend(_validate_native_map_ownership(name, product))
    if product.get("operator_switchable") is True:
        for field in PRODUCT_MODE_REQUIRED_FIELDS:
            if field not in product or product.get(field) in (None, ""):
                issues.append(
                    _issue(
                        "product_mode_field_missing",
                        f"operator-switchable product {name} is missing {field}",
                        scope=f"product:{name}",
                    )
                )
        switch_policy = str(product.get("switch_policy") or "")
        if switch_policy and switch_policy not in PRODUCT_SWITCH_POLICIES:
            issues.append(
                _issue(
                    "product_switch_policy_invalid",
                    f"product {name} has unsupported switch policy {switch_policy!r}",
                    scope=f"product:{name}",
                )
            )
        online_hot_switch = bool(product.get("online_hot_switch_supported", False))
        if online_hot_switch and switch_policy != "hot_switch":
            issues.append(
                _issue(
                    "product_hot_switch_policy_conflict",
                    f"product {name} enables online hot switch but requires {switch_policy!r}",
                    scope=f"product:{name}",
                )
            )
        issues.extend(
            _validate_product_mode_semantics(
                name,
                product,
                required,
                capabilities,
            )
        )
    if name == "nav":
        for topic in (TOPICS.odometry, TOPICS.map_cloud, TOPICS.localization_health):
            if topic not in required:
                issues.append(
                    _issue(
                        "nav_core_topic_missing",
                        f"nav product must require {topic}",
                        scope="product:nav",
                    )
                )
    return issues


def _validate_operator_motion_contract(
    name: str,
    product: dict[str, Any],
    required_topics: tuple[str, ...],
    required_capabilities: tuple[str, ...],
) -> list[RuntimeGraphIssue]:
    """Require the typed motion boundary when native mode permits operators.

    The requirement is derived from ``native_nav`` policy rather than from a
    capability flag, so deleting the capability cannot make the validation
    requirement disappear.
    """

    native_nav = product.get("native_nav")
    if not isinstance(native_nav, dict):
        return []

    control_mode = str(native_nav.get("control_mode") or "").strip()
    allow_takeover = native_nav.get("allow_teleop_takeover") is True
    requires_operator_motion = control_mode in {"teleop", "teleop_avoid"} or (
        control_mode == "autonomy" and allow_takeover
    )
    if not requires_operator_motion:
        return []

    issues: list[RuntimeGraphIssue] = []
    topics = set(required_topics)
    missing_topics = sorted(OPERATOR_MOTION_TOPICS - topics)
    if missing_topics:
        issues.append(
            _issue(
                "product_operator_motion_boundary_incomplete",
                f"product {name} permits operator motion but misses topics: {', '.join(missing_topics)}",
                scope=f"product:{name}",
            )
        )

    capabilities = set(required_capabilities)
    missing_capabilities = sorted(OPERATOR_MOTION_CAPABILITIES - capabilities)
    if missing_capabilities:
        issues.append(
            _issue(
                "product_operator_motion_capability_missing",
                f"product {name} permits operator motion but misses capabilities: {', '.join(missing_capabilities)}",
                scope=f"product:{name}",
            )
        )

    # Products with a managed Host expose operator motion through the Host's
    # typed native client and WebSocket adapter.  Those adapters must be
    # startup-critical; otherwise the Product can report ready while its
    # declared operator-motion ingress is absent.
    processes = set(_string_tuple(product.get("processes")))
    if "host" in processes:
        critical_modules = set(_string_tuple(product.get("critical_modules")))
        missing_modules = sorted(OPERATOR_MOTION_CRITICAL_MODULES - critical_modules)
        if missing_modules:
            issues.append(
                _issue(
                    "product_operator_motion_critical_adapter_missing",
                    f"product {name} permits Host operator motion but misses critical modules: "
                    f"{', '.join(missing_modules)}",
                    scope=f"product:{name}",
                )
            )
    return issues


def _validate_native_final_writer_contract(
    name: str,
    product: dict[str, Any],
    required_capabilities: tuple[str, ...],
) -> list[RuntimeGraphIssue]:
    capabilities = set(required_capabilities)
    if "final_cmd_vel_single_writer" not in capabilities:
        return []

    forbidden_modules = set(_string_tuple(product.get("forbidden_modules")))
    missing = sorted(PYTHON_MOTION_AUTHORITY_MODULES - forbidden_modules)
    if not missing:
        return []
    return [
        _issue(
            "product_python_motion_authority_not_forbidden",
            f"native final-writer product {name} must forbid Python motion authorities: "
            f"{', '.join(missing)}",
            scope=f"product:{name}",
        )
    ]


def _validate_native_map_ownership(
    name: str,
    product: dict[str, Any],
) -> list[RuntimeGraphIssue]:
    """Keep Python realtime map layers out of Products with a native maps role."""

    if "maps" not in set(_string_tuple(product.get("processes"))):
        return []
    forbidden_modules = set(_string_tuple(product.get("forbidden_modules")))
    missing = sorted(PYTHON_REALTIME_MAP_MODULES - forbidden_modules)
    if not missing:
        return []
    return [
        _issue(
            "product_python_realtime_map_not_forbidden",
            f"native maps product {name} must forbid Python realtime map modules: "
            f"{', '.join(missing)}",
            scope=f"product:{name}",
        )
    ]


def _validate_real_product_topic_closure(
    graph: RuntimeGraph,
    resolved_products: ResolvedProductVariants,
) -> list[RuntimeGraphIssue]:
    real = graph.envs.get("real")
    if real is None:
        return []
    endpoint = _env_endpoint_contract(real)
    available = _endpoint_topics(endpoint)
    issues: list[RuntimeGraphIssue] = []
    for name, variants in sorted(resolved_products.items()):
        for variant, product in variants:
            try:
                required_topics = resolve_product_spec_contracts(
                    name,
                    product,
                ).topics
            except ValueError:
                # _validate_product reports the malformed contract once.
                continue
            missing = sorted(set(required_topics) - available)
            if missing:
                issues.append(
                    _issue(
                        "field_product_topic_missing",
                        f"real Product {_product_variant_label(name, variant)} "
                        f"has no Env endpoint provider for: {', '.join(missing)}",
                        scope=_product_variant_scope(name, variant),
                    )
                )
    return issues


def _validate_real_product_process_closure(
    graph: RuntimeGraph,
    resolved_products: ResolvedProductVariants,
) -> list[RuntimeGraphIssue]:
    from .processes import resolve_processes

    real = graph.envs.get("real")
    if real is None:
        return []
    definitions = real.get("processes")
    mapped = set(definitions) if isinstance(definitions, dict) else set()
    issues: list[RuntimeGraphIssue] = []
    for name, variants in sorted(resolved_products.items()):
        for variant, product in variants:
            required = set(_string_tuple(product.get("processes")))
            missing = sorted(required - mapped)
            if missing:
                issues.append(
                    _issue(
                        "field_product_process_unmapped",
                        f"real Product {_product_variant_label(name, variant)} "
                        f"has unmapped process roles: {', '.join(missing)}",
                        scope=_product_variant_scope(name, variant),
                    )
                )
                continue
            validation_graph = RuntimeGraph(
                root=graph.root,
                topics=graph.topics,
                products={**graph.products, name: product},
                envs=graph.envs,
            )
            try:
                resolve_processes(name, "real", graph=validation_graph)
            except ValueError as exc:
                issues.append(
                    _issue(
                        "product_processes_invalid",
                        f"Product {_product_variant_label(name, variant)}: {exc}",
                        scope=_product_variant_scope(name, variant),
                    )
                )
    return issues


def _validate_product_session_defaults(graph: RuntimeGraph) -> list[RuntimeGraphIssue]:
    switchable = {
        name: product
        for name, product in graph.products.items()
        if product.get("operator_switchable") is True
    }
    active_modes = {
        str(product.get("session_mode") or "")
        for product in switchable.values()
        if str(product.get("session_mode") or "") != "none"
    }
    issues: list[RuntimeGraphIssue] = []
    for mode in sorted(active_modes):
        defaults = [
            name
            for name, product in switchable.items()
            if product.get("default_for_session_mode") is True
            and str(product.get("session_mode") or "") == mode
        ]
        if len(defaults) != 1:
            issues.append(
                _issue(
                    "product_session_default_invalid",
                    f"session mode {mode!r} must have exactly one default product; found {defaults}",
                    scope=f"session_mode:{mode}",
                )
            )
    return issues


def _validate_product_mode_semantics(
    name: str,
    product: dict[str, Any],
    required_topics: tuple[str, ...],
    required_capabilities: tuple[str, ...],
) -> list[RuntimeGraphIssue]:
    issues: list[RuntimeGraphIssue] = []
    topics = set(required_topics)
    capabilities = set(required_capabilities)
    product_mode = str(product.get("product_mode") or "")
    session_mode = str(product.get("session_mode") or "")
    slam_mode = str(product.get("slam_mode") or "")
    control_mode = str(product.get("native_control_mode") or "")
    requires_map = bool(product.get("requires_map", False))
    processes = set(_string_tuple(product.get("processes")))
    native_nav = product.get("native_nav") or {}
    if not isinstance(native_nav, dict):
        issues.append(
            _issue(
                "product_native_nav_invalid",
                f"product {name} native_nav must be a mapping",
                scope=f"product:{name}",
            )
        )
        native_nav = {}

    for field, value, allowed in (
        ("session_mode", session_mode, PRODUCT_SESSION_MODES),
        ("slam_mode", slam_mode, PRODUCT_SLAM_MODES),
        ("native_control_mode", control_mode, PRODUCT_CONTROL_MODES),
    ):
        if value and value not in allowed:
            issues.append(
                _issue(
                    "product_mode_value_invalid",
                    f"product {name} has unsupported {field} {value!r}",
                    scope=f"product:{name}",
                )
            )

    if requires_map != (slam_mode == "localization"):
        issues.append(
            _issue(
                "product_map_slam_conflict",
                f"product {name} requires_map={requires_map} but slam_mode={slam_mode!r}",
                scope=f"product:{name}",
            )
        )

    if product_mode == "exploration" and not requires_map:
        if "rolling_map_segment_execution" not in capabilities:
            issues.append(
                _issue(
                    "explore_live_missing",
                    f"map-free exploration product {name} must require rolling_map_segment_execution",
                    scope=f"product:{name}",
                )
            )
        if "octoplanner3d_global_planning" in capabilities:
            issues.append(
                _issue(
                    "explore_map_conflict",
                    f"map-free exploration product {name} must not require static-map global planning",
                    scope=f"product:{name}",
                )
            )
    elif product_mode == "exploration" and "octoplanner3d_global_planning" not in capabilities:
        issues.append(
            _issue(
                "explore_map_missing",
                f"saved-map exploration product {name} must require static-map global planning",
                scope=f"product:{name}",
            )
        )

    # Product process names are stable RunPlan roles declared by
    # the Runtime Graph.  The Python application role is ``host``; ``runtime``
    # is an architecture/package name and is not a deployable process target.
    expected_processes = {"nav", "driver", "host"}
    if slam_mode != "none":
        expected_processes.update(("lidar", "slam"))
    if "local_planner_collision_and_traversability_scoring" in capabilities:
        expected_processes.add("traversability")
    if "inspection_evidence_capture_and_result_ack" in capabilities:
        expected_processes.add("camera")
    exploration_capabilities = {
        "frontier_or_tare_goal_source",
        "tare_frontier_or_viewpoint_goal_source",
    }
    if exploration_capabilities.intersection(capabilities):
        expected_processes.add("explore")
    for process in sorted(expected_processes - processes):
        issues.append(
            _issue(
                "product_process_missing",
                f"product {name} must explicitly require process {process}",
                scope=f"product:{name}",
            )
        )

    command_topics = {TOPICS.nav_command_request, TOPICS.nav_command_ack, TOPICS.cmd_vel}
    if not command_topics <= topics:
        missing_command_topics = ", ".join(sorted(command_topics - topics))
        issues.append(
            _issue(
                "product_command_boundary_incomplete",
                f"operator-switchable product {name} misses command boundary topics: {missing_command_topics}",
                scope=f"product:{name}",
            )
        )
    if exploration_capabilities.intersection(capabilities):
        exploration_topics = {
            TOPICS.exploration_command,
            TOPICS.exploration_ack,
            TOPICS.exploration_snapshot,
            TOPICS.exploration_execution_snapshot,
            TOPICS.exploration_segment_request,
            TOPICS.exploration_segment_ack,
            TOPICS.exploration_segment_status,
        }
        missing_exploration_topics = exploration_topics - topics
        if missing_exploration_topics:
            issues.append(
                _issue(
                    "product_exploration_boundary_incomplete",
                    f"exploration product {name} misses topics: "
                    f"{', '.join(sorted(missing_exploration_topics))}",
                    scope=f"product:{name}",
                )
            )

    local_planner = "local_planner_collision_and_traversability_scoring" in capabilities
    if control_mode in PRODUCT_CONTROL_MODES:
        native_control_mode = str(native_nav.get("control_mode") or "").strip()
        if native_control_mode != control_mode:
            issues.append(
                _issue(
                    "product_native_control_mode_mismatch",
                    f"product {name} native_nav.control_mode must be {control_mode!r}",
                    scope=f"product:{name}",
                )
            )
    if local_planner:
        if native_nav.get("check_obstacle") is not True:
            issues.append(
                _issue(
                    "product_native_obstacle_check_missing",
                    f"product {name} must explicitly enable native obstacle checks",
                    scope=f"product:{name}",
                )
            )
        if native_nav.get("use_traversability_cost") is not True:
            issues.append(
                _issue(
                    "product_native_traversability_check_missing",
                    f"product {name} must explicitly enable native traversability checks",
                    scope=f"product:{name}",
                )
            )
        planner_topics = {
            TOPICS.registered_cloud,
            TOPICS.traversability,
            TOPICS.local_path,
        }
        missing_planner_topics = sorted(planner_topics - topics)
        if missing_planner_topics:
            issues.append(
                _issue(
                    "product_local_planner_boundary_incomplete",
                    f"product {name} claims native local planning but misses: "
                    f"{', '.join(missing_planner_topics)}",
                    scope=f"product:{name}",
                )
            )

    if "operator_assisted_local_planner_control" in capabilities and native_nav.get("teleop_local_planner") is not True:
        issues.append(
            _issue(
                "product_native_teleop_local_planner_missing",
                f"product {name} must explicitly enable the native teleop local planner",
                scope=f"product:{name}",
            )
        )

    if "final_cmd_vel_single_writer" not in capabilities:
        issues.append(
            _issue(
                "product_final_writer_capability_missing",
                f"operator-switchable product {name} must declare final_cmd_vel_single_writer",
                scope=f"product:{name}",
            )
        )
    return issues


def _validate_env(
    name: str,
    env: dict[str, Any],
    *,
    products: dict[str, dict[str, Any]],
    resolved_products: ResolvedProductVariants,
    topics: dict[str, Any],
    native_topics: tuple[str, ...],
) -> list[RuntimeGraphIssue]:
    issues: list[RuntimeGraphIssue] = []
    scope = f"env:{name}"
    if env.get("schema_version") != ENV_SCHEMA_VERSION:
        issues.append(
            _issue(
                "env_schema_invalid",
                f"Env {name} must use schema {ENV_SCHEMA_VERSION}",
                scope=scope,
            )
        )
    if env.get("name") != name:
        issues.append(
            _issue(
                "env_name_mismatch",
                f"Env catalog key {name!r} does not match its declared name",
                scope=scope,
            )
        )

    if name == "real":
        if env.get("robot_config_ref") != "config/robot_config.yaml":
            issues.append(
                _issue(
                    "real_robot_config_ref_invalid",
                    "real Env must reference config/robot_config.yaml",
                    scope=scope,
                )
            )
        if "backends" in env:
            issues.append(
                _issue(
                    "real_backend_catalog_invalid",
                    "real Env has one physical implementation and must not declare backends",
                    scope=scope,
                )
            )
        issues.extend(
            _validate_env_implementation(
                "real",
                env,
                products=products,
                resolved_products=resolved_products,
                topics=topics,
                native_topics=native_topics,
                native_contract_required=True,
            )
        )
        return issues

    if name != "sim":
        return issues
    if "robot_config_ref" in env:
        issues.append(
            _issue(
                "sim_robot_config_ref_invalid",
                "sim Env must not own a physical RobotConfig reference",
                scope=scope,
            )
        )
    if any(key in env for key in ("backend", "default_backend")):
        issues.append(
            _issue(
                "sim_default_backend_forbidden",
                "sim Env must not choose an implicit backend",
                scope=scope,
            )
        )
    backends = env.get("backends")
    if not isinstance(backends, dict):
        issues.append(
            _issue(
                "sim_backends_missing",
                "sim Env must declare backend implementations",
                scope=scope,
            )
        )
        return issues
    actual_backends = set(backends)
    if actual_backends != SIM_BACKENDS:
        issues.append(
            _issue(
                "sim_backend_catalog_invalid",
                f"sim backends must be exactly {sorted(SIM_BACKENDS)}; "
                f"found {sorted(actual_backends)}",
                scope=scope,
            )
        )
    for backend_name, implementation in sorted(backends.items()):
        backend_scope = f"sim backend:{backend_name}"
        if not isinstance(implementation, dict):
            issues.append(
                _issue(
                    "sim_backend_invalid",
                    f"{backend_scope} must be a mapping",
                    scope=scope,
                )
            )
            continue
        issues.extend(
            _validate_env_implementation(
                backend_scope,
                implementation,
                products=products,
                resolved_products=resolved_products,
                topics=topics,
                native_topics=native_topics,
                native_contract_required=backend_name == "mujoco_native",
            )
        )
    return issues


def _validate_env_implementation(
    label: str,
    implementation: dict[str, Any],
    *,
    products: dict[str, dict[str, Any]],
    resolved_products: ResolvedProductVariants,
    topics: dict[str, Any],
    native_topics: tuple[str, ...],
    native_contract_required: bool,
) -> list[RuntimeGraphIssue]:
    issues: list[RuntimeGraphIssue] = []
    scope = f"env:{label}"
    supported = _string_tuple(implementation.get("supported_products"))
    if not supported or len(set(supported)) != len(supported):
        issues.append(
            _issue(
                "env_supported_products_invalid",
                f"{label} must declare unique supported_products",
                scope=scope,
            )
        )
    unknown_products = sorted(set(supported) - set(products))
    if unknown_products:
        issues.append(
            _issue(
                "env_supported_product_unknown",
                f"{label} references unknown Products: {', '.join(unknown_products)}",
                scope=scope,
            )
        )
    variant_limits, variant_limit_issues = _validate_supported_product_variants(
        label,
        implementation,
        supported_products=set(supported),
        products=products,
        scope=scope,
    )
    issues.extend(variant_limit_issues)

    roles = _string_tuple(implementation.get("provided_roles"))
    if not roles or len(set(roles)) != len(roles):
        issues.append(
            _issue(
                "env_provided_roles_invalid",
                f"{label} must declare uniquely owned provided_roles",
                scope=scope,
            )
        )
    if not isinstance(implementation.get("host_config"), dict):
        issues.append(
            _issue(
                "env_host_config_missing",
                f"{label} must declare environment-owned host_config",
                scope=scope,
            )
        )
    if "native_services" in implementation or "mode_specific_services" in implementation:
        issues.append(
            _issue(
                "env_legacy_service_list",
                f"{label} must use role-to-process mappings only",
                scope=scope,
            )
        )

    process_control = str(implementation.get("process_control") or "").strip()
    process_definitions = implementation.get("processes")
    if process_control == "systemd":
        issues.extend(
            _validate_systemd_processes(
                label,
                implementation,
                provided_roles=set(roles),
                scope=scope,
            )
        )
    elif process_control == "acceptance_runner":
        if process_definitions is not None:
            issues.append(
                _issue(
                    "env_process_control_invalid",
                    f"{label} is acceptance-runner owned and must not declare ProductControl processes",
                    scope=scope,
                )
            )
        if not str(implementation.get("acceptance_runner") or "").strip():
            issues.append(
                _issue(
                    "env_acceptance_runner_missing",
                    f"{label} must declare its acceptance runner",
                    scope=scope,
                )
            )
        manifests = implementation.get("acceptance_manifests")
        if not isinstance(manifests, dict) or not manifests:
            issues.append(
                _issue(
                    "env_acceptance_manifests_missing",
                    f"{label} must declare Product acceptance manifests",
                    scope=scope,
                )
            )
    elif process_control == "external_runner":
        if process_definitions is not None:
            issues.append(
                _issue(
                    "env_process_control_invalid",
                    f"{label} is external-runner owned and must not declare ProductControl processes",
                    scope=scope,
                )
            )
        if not str(implementation.get("runner") or "").strip():
            issues.append(
                _issue(
                    "env_external_runner_missing",
                    f"{label} must declare its external runner",
                    scope=scope,
                )
            )
    else:
        issues.append(
            _issue(
                "env_process_control_invalid",
                f"{label} has invalid process_control {process_control!r}",
                scope=scope,
            )
        )

    endpoint = _env_endpoint_contract(implementation)
    if not endpoint:
        issues.append(
            _issue(
                "endpoint_contract_missing",
                f"{label} must declare endpoints.contract",
                scope=scope,
            )
        )
    else:
        issues.extend(
            _validate_endpoint_contract(
                label,
                endpoint,
                topics=topics,
                native_topics=native_topics,
                native_contract_required=native_contract_required,
                scope=scope,
            )
        )
        available_topics = _endpoint_topics(endpoint)
        for product_name in sorted(set(supported) & set(products)):
            for variant, product in resolved_products.get(product_name, ()):
                allowed_variants = variant_limits.get(product_name)
                if (
                    allowed_variants is not None
                    and variant not in allowed_variants
                ):
                    continue
                try:
                    required_topics = set(
                        resolve_product_spec_contracts(
                            product_name,
                            product,
                        ).topics
                    )
                except ValueError:
                    # Product-level validation reports the malformed contract.
                    continue
                missing_topics = sorted(required_topics - available_topics)
                if missing_topics:
                    issues.append(
                        _issue(
                            "env_product_topic_missing",
                            f"{label} cannot provide Product "
                            f"{_product_variant_label(product_name, variant)} topics: "
                            f"{', '.join(missing_topics)}",
                            scope=scope,
                        )
                    )
    return issues


def _validate_supported_product_variants(
    label: str,
    implementation: dict[str, Any],
    *,
    supported_products: set[str],
    products: dict[str, dict[str, Any]],
    scope: str,
) -> tuple[dict[str, frozenset[str]], list[RuntimeGraphIssue]]:
    raw_limits = implementation.get("supported_product_variants")
    if raw_limits is None:
        return {}, []
    if not isinstance(raw_limits, Mapping):
        return {}, [
            _issue(
                "env_supported_product_variants_invalid",
                f"{label} supported_product_variants must be a mapping",
                scope=scope,
            )
        ]

    limits: dict[str, frozenset[str]] = {}
    issues: list[RuntimeGraphIssue] = []
    for raw_product, raw_variants in raw_limits.items():
        product = str(raw_product).strip()
        if product != raw_product or product not in supported_products:
            issues.append(
                _issue(
                    "env_supported_product_variants_invalid",
                    f"{label} restricts variants for unsupported Product {raw_product!r}",
                    scope=scope,
                )
            )
            continue
        product_spec = products.get(product)
        if product_spec is None:
            issues.append(
                _issue(
                    "env_supported_product_variants_invalid",
                    f"{label} restricts unknown Product {product}",
                    scope=scope,
                )
            )
            continue
        declared = product_spec.get("variants")
        if not isinstance(declared, Mapping) or not declared:
            issues.append(
                _issue(
                    "env_supported_product_variants_invalid",
                    f"{label} restricts Product {product}, which declares no variants",
                    scope=scope,
                )
            )
            continue
        if not isinstance(raw_variants, list | tuple) or not raw_variants:
            issues.append(
                _issue(
                    "env_supported_product_variants_invalid",
                    f"{label} Product {product} variant restriction must be a non-empty list",
                    scope=scope,
                )
            )
            continue
        if any(
            not isinstance(item, str)
            or not item.strip()
            or item != item.strip()
            for item in raw_variants
        ):
            issues.append(
                _issue(
                    "env_supported_product_variants_invalid",
                    f"{label} Product {product} variant restriction contains invalid names",
                    scope=scope,
                )
            )
            continue
        variants = tuple(raw_variants)
        if len(set(variants)) != len(variants):
            issues.append(
                _issue(
                    "env_supported_product_variants_invalid",
                    f"{label} Product {product} variant restriction contains invalid or duplicate names",
                    scope=scope,
                )
            )
            continue
        unknown = sorted(set(variants) - set(declared))
        if unknown:
            issues.append(
                _issue(
                    "env_supported_product_variant_unknown",
                    f"{label} Product {product} references unknown variants: "
                    f"{', '.join(unknown)}",
                    scope=scope,
                )
            )
            continue
        default_variant = str(product_spec.get("default_variant") or "").strip()
        if default_variant not in variants:
            issues.append(
                _issue(
                    "env_default_product_variant_unsupported",
                    f"{label} Product {product} must support its default variant "
                    f"{default_variant!r}",
                    scope=scope,
                )
            )
            continue
        if set(variants) == set(declared):
            issues.append(
                _issue(
                    "env_product_variant_restriction_redundant",
                    f"{label} Product {product} supports every declared variant; "
                    "remove its supported_product_variants entry",
                    scope=scope,
                )
            )
            continue
        limits[product] = frozenset(variants)
    return limits, issues


def _validate_systemd_processes(
    label: str,
    implementation: dict[str, Any],
    *,
    provided_roles: set[str],
    scope: str,
) -> list[RuntimeGraphIssue]:
    issues: list[RuntimeGraphIssue] = []
    process_definitions = implementation.get("processes")
    if not isinstance(process_definitions, dict):
        return [
            _issue(
                "env_process_mapping_missing",
                f"ProductControl-managed {label} must declare a process mapping",
                scope=scope,
            )
        ]
    process_roles = set(process_definitions)
    if process_roles != provided_roles:
        issues.append(
            _issue(
                "env_role_ownership_mismatch",
                f"{label} provided_roles must exactly match its process role owners",
                scope=scope,
            )
        )
    manager = str(implementation.get("process_manager") or "").strip()
    if manager not in {"systemd", "direct", "external"}:
        issues.append(
            _issue(
                "env_process_manager_invalid",
                f"{label} has invalid process manager {manager!r}",
                scope=scope,
            )
        )
    orders: dict[int, str] = {}
    targets: dict[str, str] = {}
    for process_name, process in process_definitions.items():
        if not isinstance(process, dict):
            issues.append(
                _issue(
                    "env_process_invalid",
                    f"{label} process role {process_name} must be a mapping",
                    scope=scope,
                )
            )
            continue
        target = str(process.get("target") or "").strip()
        lifecycle = str(process.get("lifecycle") or "").strip()
        order = process.get("order")
        timeout_s = process.get("timeout_s")
        if not target or lifecycle not in {"mode", "persistent"}:
            issues.append(
                _issue(
                    "env_process_invalid",
                    f"{label} process role {process_name} has invalid target or lifecycle",
                    scope=scope,
                )
            )
        if target in targets:
            issues.append(
                _issue(
                    "env_process_owner_duplicate",
                    f"{label} roles {targets[target]} and {process_name} share one process owner",
                    scope=scope,
                )
            )
        elif target:
            targets[target] = str(process_name)
        if (
            isinstance(order, bool)
            or not isinstance(order, int)
            or order < 0
            or isinstance(timeout_s, bool)
            or not isinstance(timeout_s, int)
            or timeout_s <= 0
        ):
            issues.append(
                _issue(
                    "env_process_invalid",
                    f"{label} process role {process_name} has invalid order or timeout_s",
                    scope=scope,
                )
            )
        elif order in orders:
            issues.append(
                _issue(
                    "env_process_order_duplicate",
                    f"{label} process roles {orders[order]} and {process_name} share order {order}",
                    scope=scope,
                )
            )
        else:
            orders[order] = str(process_name)
    return issues


def _validate_endpoint_contract(
    label: str,
    endpoint: dict[str, Any],
    *,
    topics: dict[str, Any],
    native_topics: tuple[str, ...],
    native_contract_required: bool,
    scope: str,
) -> list[RuntimeGraphIssue]:
    issues: list[RuntimeGraphIssue] = []
    declared = _endpoint_topics(endpoint)
    for topic in sorted(declared):
        if topic not in topics:
            issues.append(
                _issue(
                    "endpoint_contract_topic_missing",
                    f"{label} endpoint contract references undeclared topic {topic}",
                    scope=scope,
                )
            )
    real_equivalent = bool(endpoint.get("real_equivalent", False))
    runtime_class = str(endpoint.get("runtime_class") or "")
    data_plane = str(endpoint.get("data_plane") or "")
    if runtime_class == "host_simulation" and real_equivalent:
        issues.append(
            _issue(
                "host_simulation_cannot_be_real_equivalent",
                f"{label} endpoint contract is Host simulation but marked real-equivalent",
                scope=scope,
            )
        )
    if real_equivalent and data_plane != "native_dds":
        issues.append(
            _issue(
                "real_equivalent_requires_native_dds",
                f"{label} endpoint contract is real-equivalent but data_plane is {data_plane!r}",
                scope=scope,
            )
        )
    if native_contract_required:
        missing = sorted(set(native_topics) - declared)
        if missing:
            issues.append(
                _issue(
                    "native_endpoint_contract_missing",
                    f"{label} endpoint contract misses native topics: {', '.join(missing)}",
                    scope=scope,
                )
            )
    return issues


def _validate_native_contract_parity(graph: RuntimeGraph) -> list[RuntimeGraphIssue]:
    real = graph.envs.get("real")
    sim = graph.envs.get("sim")
    backends = sim.get("backends") if isinstance(sim, dict) else None
    mujoco_native = backends.get("mujoco_native") if isinstance(backends, dict) else None
    if not isinstance(real, dict) or not isinstance(mujoco_native, dict):
        return []

    native_topics = set(graph.native_contract_topics)
    contracts = (
        ("real", _env_endpoint_contract(real)),
        ("sim", _env_endpoint_contract(mujoco_native)),
    )
    issues: list[RuntimeGraphIssue] = []
    for env_name, contract in contracts:
        missing_topics = sorted(native_topics - _endpoint_topics(contract))
        if missing_topics:
            issues.append(
                _issue(
                    "native_endpoint_parity_missing",
                    f"{env_name} Env does not expose native contract topics: "
                    f"{', '.join(missing_topics)}",
                    scope=f"env:{env_name}",
                )
            )
    return issues


def _env_endpoint_contract(implementation: dict[str, Any]) -> dict[str, Any]:
    endpoints = implementation.get("endpoints")
    if not isinstance(endpoints, dict):
        return {}
    contract = endpoints.get("contract")
    return contract if isinstance(contract, dict) else {}


def _endpoint_topics(endpoint: dict[str, Any]) -> set[str]:
    return (
        set(_string_tuple(endpoint.get("source_topics")))
        | set(_string_tuple(endpoint.get("exposed_topics")))
        | set(_string_tuple(endpoint.get("provided_topics")))
    )


def _string_tuple(value: Any) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        return (value,)
    if isinstance(value, list | tuple | set):
        return tuple(str(item) for item in value)
    return ()


def _issue(code: str, message: str, *, scope: str = "runtime_graph") -> RuntimeGraphIssue:
    return RuntimeGraphIssue(code=code, message=message, scope=scope)

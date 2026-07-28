"""Validation helpers for Runtime Graph contracts."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from runtime.runtime_interface import TOPICS

from .loader import RuntimeGraph, load_runtime_graph

REQUIRED_NATIVE_ENDPOINTS = frozenset({"thunder_field", "mujoco_native_dds"})
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

    for name, product in graph.products.items():
        issues.extend(_validate_product(name, product, topics))
    issues.extend(_validate_product_session_defaults(graph))

    for name, endpoint in graph.endpoints.items():
        issues.extend(_validate_endpoint(name, endpoint, topics, graph.native_contract_topics))

    issues.extend(_validate_field_product_topic_closure(graph))
    issues.extend(_validate_field_product_process_closure(graph))
    issues.extend(_validate_native_endpoint_parity(graph))
    return issues


def assert_runtime_graph_valid(graph: RuntimeGraph | None = None) -> RuntimeGraph:
    """Return *graph* if valid, otherwise raise ValueError with issue details."""

    graph = graph or load_runtime_graph()
    issues = validate_runtime_graph(graph)
    if issues:
        joined = "; ".join(f"{issue.code}: {issue.message}" for issue in issues)
        raise ValueError(f"Runtime Graph validation failed: {joined}")
    return graph


def _validate_product(
    name: str,
    product: dict[str, Any],
    topics: dict[str, Any],
) -> list[RuntimeGraphIssue]:
    issues: list[RuntimeGraphIssue] = []
    required = _string_tuple(product.get("required_topics"))
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
        issues.extend(_validate_product_mode_semantics(name, product, required))
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


def _validate_field_product_topic_closure(graph: RuntimeGraph) -> list[RuntimeGraphIssue]:
    endpoint = graph.endpoints.get("thunder_field")
    if endpoint is None:
        return []
    available = _endpoint_topics(endpoint)
    issues: list[RuntimeGraphIssue] = []
    for name, product in sorted(graph.products.items()):
        missing = sorted(set(_string_tuple(product.get("required_topics"))) - available)
        if missing:
            issues.append(
                _issue(
                    "field_product_topic_missing",
                    f"field product {name} has no endpoint provider for: {', '.join(missing)}",
                    scope=f"product:{name}@thunder_field",
                )
            )
    return issues


def _validate_field_product_process_closure(graph: RuntimeGraph) -> list[RuntimeGraphIssue]:
    from .plan import build_runtime_plan

    endpoint = graph.endpoints.get("thunder_field")
    if endpoint is None:
        return []
    definitions = endpoint.get("processes")
    mapped = set(definitions) if isinstance(definitions, dict) else set()
    issues: list[RuntimeGraphIssue] = []
    for name, product in sorted(graph.products.items()):
        required = set(_string_tuple(product.get("processes")))
        missing = sorted(required - mapped)
        if missing:
            issues.append(
                _issue(
                    "field_product_process_unmapped",
                    f"field product {name} has unmapped processes: {', '.join(missing)}",
                    scope=f"product:{name}@thunder_field",
                )
            )
            continue
        try:
            build_runtime_plan(name, "thunder_field", graph=graph)
        except ValueError as exc:
            issues.append(
                _issue(
                    "runtime_plan_invalid",
                    str(exc),
                    scope=f"product:{name}@thunder_field",
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
) -> list[RuntimeGraphIssue]:
    issues: list[RuntimeGraphIssue] = []
    topics = set(required_topics)
    capabilities = set(_string_tuple(product.get("required_capabilities")))
    session_mode = str(product.get("session_mode") or "")
    slam_mode = str(product.get("slam_mode") or "")
    control_mode = str(product.get("native_control_mode") or "")
    requires_map = bool(product.get("requires_map", False))
    processes = set(_string_tuple(product.get("processes")))

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

    expected_processes = {"nav", "driver", "runtime"}
    if slam_mode != "none":
        expected_processes.update(("lidar", "slam"))
    if "local_planner_collision_and_traversability_scoring" in capabilities:
        expected_processes.add("traversability")
    if "inspection_evidence_capture_and_result_ack" in capabilities:
        expected_processes.add("camera")
    if "tare_frontier_or_viewpoint_goal_source" in capabilities:
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
        missing = ", ".join(sorted(command_topics - topics))
        issues.append(
            _issue(
                "product_command_boundary_incomplete",
                f"operator-switchable product {name} misses command boundary topics: {missing}",
                scope=f"product:{name}",
            )
        )

    local_planner = "local_planner_collision_and_traversability_scoring" in capabilities
    if local_planner:
        planner_topics = {
            TOPICS.registered_cloud,
            TOPICS.traversability,
            TOPICS.local_path,
        }
        missing = sorted(planner_topics - topics)
        if missing:
            issues.append(
                _issue(
                    "product_local_planner_boundary_incomplete",
                    f"product {name} claims native local planning but misses: {', '.join(missing)}",
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


def _validate_endpoint(
    name: str,
    endpoint: dict[str, Any],
    topics: dict[str, Any],
    native_topics: tuple[str, ...],
) -> list[RuntimeGraphIssue]:
    issues: list[RuntimeGraphIssue] = []
    if "native_services" in endpoint or "mode_specific_services" in endpoint:
        issues.append(
            _issue(
                "endpoint_legacy_service_list",
                f"endpoint {name} must use the process mapping only",
                scope=f"endpoint:{name}",
            )
        )
    process_control = str(endpoint.get("process_control") or "").strip()
    process_definitions = endpoint.get("processes")
    if process_control == "runtime_plan" and process_definitions is None:
        issues.append(
            _issue(
                "endpoint_process_mapping_missing",
                f"RuntimePlan-managed endpoint {name} must declare processes",
                scope=f"endpoint:{name}",
            )
        )
    if process_definitions is not None and process_control != "runtime_plan":
        issues.append(
            _issue(
                "endpoint_process_control_invalid",
                f"endpoint {name} declares processes but is not RuntimePlan-managed",
                scope=f"endpoint:{name}",
            )
        )
    if process_control == "acceptance_runner" and not str(
        endpoint.get("acceptance_runner") or ""
    ).strip():
        issues.append(
            _issue(
                "endpoint_acceptance_runner_missing",
                f"acceptance-runner endpoint {name} must declare acceptance_runner",
                scope=f"endpoint:{name}",
            )
        )
    if process_definitions is not None:
        manager = str(endpoint.get("process_manager") or "").strip()
        if manager not in {"systemd", "direct", "external"}:
            issues.append(
                _issue(
                    "endpoint_process_manager_invalid",
                    f"endpoint {name} has invalid process manager {manager!r}",
                    scope=f"endpoint:{name}",
                )
            )
        if not isinstance(process_definitions, dict):
            issues.append(
                _issue(
                    "endpoint_process_mapping_invalid",
                    f"endpoint {name} process mapping must be an object",
                    scope=f"endpoint:{name}",
                )
            )
        else:
            orders: dict[int, str] = {}
            for process_name, process in process_definitions.items():
                if not isinstance(process, dict):
                    issues.append(
                        _issue(
                            "endpoint_process_invalid",
                            f"endpoint {name} process {process_name} must be an object",
                            scope=f"endpoint:{name}",
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
                            "endpoint_process_invalid",
                            f"endpoint {name} process {process_name} has invalid target or lifecycle",
                            scope=f"endpoint:{name}",
                        )
                    )
                if not isinstance(order, int) or order < 0 or not isinstance(timeout_s, int) or timeout_s <= 0:
                    issues.append(
                        _issue(
                            "endpoint_process_invalid",
                            f"endpoint {name} process {process_name} has invalid order or timeout_s",
                            scope=f"endpoint:{name}",
                        )
                    )
                elif order in orders:
                    issues.append(
                        _issue(
                            "endpoint_process_order_duplicate",
                            f"endpoint {name} processes {orders[order]} and {process_name} share order {order}",
                            scope=f"endpoint:{name}",
                        )
                    )
                else:
                    orders[order] = str(process_name)
    declared = set(_string_tuple(endpoint.get("source_topics"))) | set(_string_tuple(endpoint.get("exposed_topics")))
    for topic in sorted(declared):
        if topic not in topics:
            issues.append(
                _issue(
                    "endpoint_topic_missing",
                    f"endpoint {name} references undeclared topic {topic}",
                    scope=f"endpoint:{name}",
                )
            )

    real_equivalent = bool(endpoint.get("real_equivalent", False))
    runtime_class = str(endpoint.get("runtime_class") or "")
    data_plane = str(endpoint.get("data_plane") or "")
    if runtime_class == "module_sim_harness" and real_equivalent:
        issues.append(
            _issue(
                "module_sim_cannot_be_real_equivalent",
                f"endpoint {name} is a module simulation harness but is marked real_equivalent",
                scope=f"endpoint:{name}",
            )
        )
    if real_equivalent and data_plane != "native_dds":
        issues.append(
            _issue(
                "real_equivalent_requires_native_dds",
                f"endpoint {name} is real_equivalent but data_plane is {data_plane!r}",
                scope=f"endpoint:{name}",
            )
        )
    if name in REQUIRED_NATIVE_ENDPOINTS:
        missing = [topic for topic in native_topics if topic not in declared]
        if missing:
            issues.append(
                _issue(
                    "native_endpoint_contract_missing",
                    f"endpoint {name} misses native contract topics: {', '.join(missing)}",
                    scope=f"endpoint:{name}",
                )
            )
    return issues


def _validate_native_endpoint_parity(graph: RuntimeGraph) -> list[RuntimeGraphIssue]:
    missing = [name for name in REQUIRED_NATIVE_ENDPOINTS if name not in graph.endpoints]
    if missing:
        return [
            _issue(
                "native_endpoint_missing",
                f"required Runtime Graph endpoint missing: {', '.join(sorted(missing))}",
            )
        ]

    native_topics = set(graph.native_contract_topics)
    thunder = _endpoint_topics(graph.endpoints["thunder_field"])
    mujoco = _endpoint_topics(graph.endpoints["mujoco_native_dds"])
    issues: list[RuntimeGraphIssue] = []
    for endpoint_name, endpoint_topics in (
        ("thunder_field", thunder),
        ("mujoco_native_dds", mujoco),
    ):
        missing_topics = sorted(native_topics - endpoint_topics)
        if missing_topics:
            issues.append(
                _issue(
                    "native_endpoint_parity_missing",
                    f"{endpoint_name} does not expose native contract topics: {', '.join(missing_topics)}",
                    scope=f"endpoint:{endpoint_name}",
                )
            )
    return issues


def _endpoint_topics(endpoint: dict[str, Any]) -> set[str]:
    return set(_string_tuple(endpoint.get("source_topics"))) | set(_string_tuple(endpoint.get("exposed_topics")))


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

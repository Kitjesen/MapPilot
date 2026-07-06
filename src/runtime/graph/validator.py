"""Validation helpers for Runtime Graph contracts."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .loader import RuntimeGraph, load_runtime_graph


REAL_PRODUCT_PROFILES = frozenset({"map", "nav", "explore", "teleop_avoid"})
LEGACY_MODULE_SIM_PROFILES = frozenset({"sim_mujoco_live", "sim_mujoco_octo_live"})
REQUIRED_NATIVE_ENDPOINTS = frozenset({"thunder_field", "mujoco_native_dds"})


@dataclass(frozen=True)
class RuntimeGraphIssue:
    """A machine-readable Runtime Graph validation issue."""

    code: str
    message: str
    scope: str = "runtime_graph"
    severity: str = "error"

    def as_dict(self) -> dict[str, str]:
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

    for name, endpoint in graph.endpoints.items():
        issues.extend(_validate_endpoint(name, endpoint, topics, graph.native_contract_topics))

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


def validate_profile_against_runtime_graph(
    profile: str,
    graph: RuntimeGraph | None = None,
) -> list[RuntimeGraphIssue]:
    """Validate a resolved profile against Runtime Graph product boundaries."""

    graph = graph or load_runtime_graph()
    issues: list[RuntimeGraphIssue] = []
    profile_name = str(profile)

    if profile_name in REAL_PRODUCT_PROFILES:
        issues.extend(_validate_real_product_profile(profile_name, graph))
    if profile_name in LEGACY_MODULE_SIM_PROFILES:
        issues.extend(_validate_legacy_module_sim_profile(profile_name))
    return issues


def _validate_product(
    name: str,
    product: dict[str, Any],
    topics: dict[str, Any],
) -> list[RuntimeGraphIssue]:
    issues: list[RuntimeGraphIssue] = []
    required = _string_tuple(product.get("required_topics"))
    for topic in required:
        if topic not in topics:
            issues.append(
                _issue(
                    "product_required_topic_missing",
                    f"product {name} requires undeclared topic {topic}",
                    scope=f"product:{name}",
                )
            )
    if name == "nav":
        for topic in ("/slam/odometry", "/slam/map_cloud", "/slam/localization_health"):
            if topic not in required:
                issues.append(
                    _issue(
                        "nav_core_topic_missing",
                        f"nav product must require {topic}",
                        scope="product:nav",
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
    declared = set(_string_tuple(endpoint.get("source_topics"))) | set(
        _string_tuple(endpoint.get("exposed_topics"))
    )
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


def _validate_real_product_profile(
    profile: str,
    graph: RuntimeGraph,
) -> list[RuntimeGraphIssue]:
    from runtime.introspection.profile_graph import graph_for_profile
    from runtime.profiles.resolver import resolve_profile_config

    config = resolve_profile_config(profile)
    profile_graph = graph_for_profile(profile)
    modules = set(profile_graph.modules)
    product = graph.products.get(profile, {})
    endpoint = graph.endpoints.get("thunder_field", {})
    forbidden_modules = set(_string_tuple(product.get("forbidden_modules"))) | set(
        _string_tuple(endpoint.get("forbidden_modules"))
    )
    issues: list[RuntimeGraphIssue] = []

    if config.get("_runtime_endpoint") != "thunder_field":
        issues.append(
            _issue(
                "real_profile_endpoint_drift",
                f"{profile} must resolve to thunder_field endpoint",
                scope=f"profile:{profile}",
            )
        )
    if config.get("localization_adapter") != "cpp_slam_status":
        issues.append(
            _issue(
                "real_profile_localization_adapter_drift",
                f"{profile} must use cpp_slam_status localization_adapter",
                scope=f"profile:{profile}",
            )
        )
    forbidden_present = sorted(forbidden_modules & modules)
    if forbidden_present:
        issues.append(
            _issue(
                "real_profile_forbidden_module",
                f"{profile} includes forbidden modules: {', '.join(forbidden_present)}",
                scope=f"profile:{profile}",
            )
        )
    return issues


def _validate_legacy_module_sim_profile(profile: str) -> list[RuntimeGraphIssue]:
    from runtime.profiles.catalog.products import PROFILES

    config = PROFILES.get(profile, {})
    issues: list[RuntimeGraphIssue] = []
    if config.get("_runtime_graph_role") != "module_sim_harness":
        issues.append(
            _issue(
                "legacy_module_sim_role_missing",
                f"{profile} must be marked _runtime_graph_role=module_sim_harness",
                scope=f"profile:{profile}",
            )
        )
    if config.get("_real_equivalent") is not False:
        issues.append(
            _issue(
                "legacy_module_sim_real_equivalent_flag",
                f"{profile} must be marked _real_equivalent=False",
                scope=f"profile:{profile}",
            )
        )
    return issues


def _endpoint_topics(endpoint: dict[str, Any]) -> set[str]:
    return set(_string_tuple(endpoint.get("source_topics"))) | set(
        _string_tuple(endpoint.get("exposed_topics"))
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
